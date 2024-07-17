/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "hobot_dev_cim.h"
#include "cim_hw_api.h"
#include "hobot_cim_ops.h"
#include "hobot_vin_common.h"
#include "vio_node_api.h"
#include "vin_node_config.h"
/* #include "hobot_cim_stl.h" */

#include "cam_buf.h"
#include "cam_uapi.h"
#include "sif.h"
#include "csi.h"
/**
 * Purpose: point to cim device struct
 * Range: 1-5
 * Attention: NA
 */
extern int fusa_skip_num;

/**
 * Purpose: Reset or not
 * Range: 0~1
 * Attention: NA
 */
extern int rst_en;

static struct vin_cim_private_s *
cim_get_priv_by_subdev(const struct vin_node_subdev *subdev)
{
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_node_subdev *src_subdev;

	vin_node_dev = subdev->vin_node_dev;
	src_subdev = &vin_node_dev->src_subdev[subdev->ctx_id];

	return &src_subdev->cim_private_attr;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cim param check
 * @retval 0: success
 * @retval <0: fail
 * @param[in] cim_cfg: configurations of cim
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_config_param_check(const cim_attr_t *cim_cfg)
{
	s32 ret = 0;
	u32 err_count = 0;
	const cim_func_desc_t *func;
	const cim_input_tpg_t *tpg_input;

	func = &cim_cfg->func;
	tpg_input = &cim_cfg->tpg_input;
	if (cim_cfg->vc_index > VC_INDEX_MAX) {
		vio_err("vc index %d is beyond 0~3\n", cim_cfg->vc_index);
		err_count++;
	}

	if (cim_cfg->ipi_channel >= VC_CHANNELS_MAX) {
		vio_err("ipi_channel %d is beyond 0~3\n", cim_cfg->ipi_channel);
		err_count++;
	}

	if ((func->skip_frame == 1u) && (tpg_input->tpg_en == 1u)) {
		vio_err("can't support skip frame in tpg mode\n");
		err_count++;
	}

	if ((func->skip_frame == 1u) && (func->input_fps <= func->output_fps)) {
		vio_err("input_fps can't <= output_fps\n");
		err_count++;
	}

	if (err_count > 0u) {
		vio_err("%s err count = %d\n", __func__, err_count);
		ret = -EINVAL;
	}

	return ret;
}

static void vinattr_ex_to_sifsetting(vin_attr_ex_t *vin_attr_ex , struct sif_cfg *sif_cfg)
{
	// fps control
	sif_cfg->fps_ctrl = vin_attr_ex->cim_static_attr.frame_drop;
	// yuv conv
	sif_cfg->yuv_conv = vin_attr_ex->cim_static_attr.yuv_conv;
	// sram merge
	sif_cfg->sram_merge = vin_attr_ex->cim_static_attr.sram_merge;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set cim extend attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] vin_attr_ex: configurations of vin_attr
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_subdev_init_ex(struct vio_video_ctx *vctx, vin_attr_ex_t *vin_attr_ex)
{
	s32 ret = 0;
	u32 ipi_channel, i;
	struct vio_node *vnode;
	struct vin_node_subdev *subdev;
	struct j6_cim_dev *cim;
	struct vio_subdev *vdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_instance *ins;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim = cim_get_dev(vin_node_dev->hw_id);
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	ipi_channel = cim_priv_attr->ipi_index;

	for (i = 0; i < cim->sif.ipi_channel_num; i++) {
		ins = &cim->sif.insts[ipi_channel + i];
		vinattr_ex_to_sifsetting(vin_attr_ex, &ins->sif_cfg);
		ret = sif_set_format(&cim->sif, ipi_channel + i, &ins->fmt, false, EX_FEAT_CHANNEL);
		if (ret < 0) {
			vio_err("[S%d][ipi%d]%s failed to call sif_set_format for extended feature\n",
				vctx->ctx_id, ipi_channel, __func__);
			return ret;
		}
	}
	return ret;
}

static void vinattr_to_camfmt(const vin_basic_attr_t *vin_basic_attr,
			      struct cam_format *cam_fmt, bool yuv_conv)
{
	vio_info("vin_basic_attr->format:%d\n", vin_basic_attr->format);
	switch (vin_basic_attr->format) {
	case DATA_TYPE_RGB888:
		cam_fmt->format = CAM_FMT_RGB888X;
		break;
	case DATA_TYPE_RAW8:
		cam_fmt->format = CAM_FMT_RAW8;
		break;
	case DATA_TYPE_RAW10:
		cam_fmt->format = CAM_FMT_RAW10;
		break;
	case DATA_TYPE_RAW12:
		cam_fmt->format = CAM_FMT_RAW12;
		break;
	case DATA_TYPE_YUV420_SHIFT:
		if (yuv_conv)
			cam_fmt->format = CAM_FMT_NV16;
		else
			cam_fmt->format = CAM_FMT_NV12;
		break;
	case DATA_TYPE_YUV422_8:
		if (vin_basic_attr->pack_mode == 1)
			cam_fmt->format = CAM_FMT_YUYV;
		else if (vin_basic_attr->pack_mode == 2)
			cam_fmt->format = CAM_FMT_NV16;
		break;
	default:
		vio_err("%s: wrong cam format %d pack_mode:%d\n", __func__, vin_basic_attr->format,
				vin_basic_attr->pack_mode);
		return;
	}
}


/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Configure fifo dma properties
 * @retval 0: success
 * @retval <0: fail
 * @param[in] subdev: VIN_NODE sub-device definition.
 * @param[in] ddr: Write some attribute configurations for DDR
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void cim_config_main_frame(struct vin_node_subdev *subdev,
				  const vin_ochn_attr_t *vin_ochn_attr)
{
	u8 ipi_channel;
	bool yuv_conv;
	struct j6_cim_dev *cim;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_node_subdev *src_subdev;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_device *sif;
	struct sif_instance *ins;
	struct cim_attr cim_attr;
	vin_ichn_attr_t  *vin_ichn_attr;
	int i;

	cim_attr = subdev->vin_attr.vin_node_attr.cim_attr;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	ipi_channel = cim_priv_attr->ipi_index;
	cim = cim_get_dev(subdev->vin_node_dev->hw_id);
	vin_node_dev = subdev->vin_node_dev;
	src_subdev = &vin_node_dev->src_subdev[subdev->ctx_id];
	vin_ichn_attr = &src_subdev->vin_attr.vin_ichn_attr;

	sif = &cim->sif;
	for (i = 0; i < sif->ipi_channel_num; i++) {
		ins = &sif->insts[ipi_channel + i];
		yuv_conv = src_subdev->vin_attr.vin_attr_ex.cim_static_attr.yuv_conv;
		vinattr_to_camfmt(&vin_ochn_attr->vin_basic_attr, &ins->fmt, yuv_conv);
		ins->fmt.width = vin_ichn_attr->width;
		ins->fmt.height = vin_ichn_attr->height;
		sif_set_format(sif, ipi_channel + i, &ins->fmt, false, OUT_CHANNEL_MAIN);
	}
}

static void ochn_attr2sif_ebd(const vin_ochn_attr_t *vin_ochn_attr,
			      struct sif_cfg *sif_cfg)
{
	struct vin_emb_attr_s *emb_attr =
		(struct vin_emb_attr_s *)&vin_ochn_attr->emb_attr;

	sif_cfg->ebd_ctrl.ebd_en = vin_ochn_attr->emb_en;
	sif_cfg->ebd_ctrl.ebd_post = emb_attr->embeded_post;
	sif_cfg->ebd_ctrl.ebd_hsize = emb_attr->embeded_width;
	sif_cfg->ebd_ctrl.ebd_vsize = emb_attr->embeded_height;
	sif_cfg->ebd_ctrl.ebd_stride = SIF_EBD_HSIZE_ALIGN;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Configuration of embeded_data
 * @retval 0: success
 * @retval <0: fail
 * @param[in] subdev: VIN_NODE sub-device definition.
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void cim_config_embeded_data(struct vin_node_subdev *subdev,
				    const vin_ochn_attr_t *vin_ochn_attr)
{
	u8 ipi_channel;
	struct j6_cim_dev *cim;
	struct vin_emb_attr_s *emb_attr;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_device *sif;
	struct sif_instance *ins;
	int i;

	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	ipi_channel = cim_priv_attr->ipi_index;
	cim = cim_get_dev(subdev->vin_node_dev->hw_id);
	emb_attr = (struct vin_emb_attr_s *)&vin_ochn_attr->emb_attr;

	if (!vin_ochn_attr->emb_en)
		return;

	sif = &cim->sif;
	for (i = 0; i < cim->sif.ipi_channel_num; i++) {
		ins = &sif->insts[ipi_channel + i];
		ochn_attr2sif_ebd(vin_ochn_attr, &ins->sif_cfg);
		sif_set_format(sif, ipi_channel + i, &ins->fmt, false, OUT_CHANNEL_EMB);
	}

	/* cim_priv_attr->.embeded_start_cnt = 2; */
	cim_priv_attr->embeded_dependence = (u8)emb_attr->embeded_dependence;
	cim_priv_attr->embeded_en = (u8)vin_ochn_attr->emb_en;
	cim_priv_attr->pack_mode[VIN_EMB] =
		(u8)vin_ochn_attr->vin_basic_attr.pack_mode;
	vio_info("%s embeded_en %d, embeded_dependence %d\n", __func__,
		 cim_priv_attr->embeded_en, cim_priv_attr->embeded_dependence);
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set ochn attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] cim_ochn_attr: Output channel attribute
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_set_ochn_attr(struct vio_video_ctx *vctx,
		      vin_ochn_attr_t *cim_ochn_attr)
{
	s32 ret = 0;
	u32 ipi_channel, ochn_id;
	struct j6_cim_dev *cim;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_cim_private_s *cim_priv_attr;

	vdev = vctx->vdev;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	ipi_channel = cim_priv_attr->ipi_index;
	cim = cim_get_dev(subdev->vin_node_dev->hw_id);
	ochn_id = vctx->id - VNODE_ID_CAP;

	cim_priv_attr->ddr_en = cim_ochn_attr->ddr_en;

	// cim_config_common_ochn(subdev, ochn_id, &cim_ochn_attr->vin_basic_attr);

	if (ochn_id == VIN_MAIN_FRAME) {
		cim_config_main_frame(subdev, cim_ochn_attr);
	} else if (ochn_id == VIN_EMB) {
		cim_config_embeded_data(subdev, cim_ochn_attr);
	} else {
		vio_err(" %s err invalid ochn_id", __func__);
		return -EINVAL;
	}

	vio_info("[S%d][ipi%d][ochn%d]%s done\n", vctx->ctx_id, ipi_channel,
		 ochn_id, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set ochn buff attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] cim_ochn_buff_attr: numbers of buffer
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int cim_set_ochn_buff_attr(struct vio_video_ctx *vctx,
			   vin_ochn_buff_attr_t *cim_ochn_buff_attr)
{
	s32 ret = 0;

	return ret;
}

static void vinattr_to_sifsetting(vin_attr_t *vin_attr, struct sif_cfg *sif_cfg)
{
	cim_func_desc_t *func;
	vin_attr_ex_t *vin_attr_ex;

	func = &vin_attr->vin_node_attr.cim_attr.func;
	vin_attr_ex = &vin_attr->vin_attr_ex;

	// frame id control
	sif_cfg->f_id_ctrl.frame_id_en = func->enable_frame_id;
	sif_cfg->f_id_ctrl.initial_frame_id = func->set_init_frame_id;
	// time stamp control
	sif_cfg->ts_ctrl.time_stamp_en = func->time_stamp_en;
	sif_cfg->ts_ctrl.trigger_mode = func->time_stamp_mode;
	sif_cfg->ts_ctrl.ts_trigger_src = func->ts_src;
	sif_cfg->ts_ctrl.pps_trigger_src = func->pps_src;
	// hdr mode
	sif_cfg->hdr_mode = func->hdr_mode;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set cim input channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] cim_ichn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int cim_set_ichn_attr(struct vio_video_ctx *vctx,
		      vin_ichn_attr_t *cim_ichn_attr)
{
	s32 ret = 0;
	u32 ipi_channel;
	struct vin_node_subdev *subdev;
	struct j6_cim_dev *cim;
	struct vio_subdev *vdev;
	struct vio_node *vnode;
	struct j6_vin_node_dev *vin_node_dev;
	vin_attr_t *vin_attr;
	// u32 bytesPerLine;
	// u32 stride;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_instance *ins;
	int rc, i;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim = cim_get_dev(vin_node_dev->hw_id);
	vin_attr = &subdev->vin_attr;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	ipi_channel = cim_priv_attr->ipi_index;

	for (i = 0; i < cim->sif.ipi_channel_num; i++) {
		ins = &cim->sif.insts[ipi_channel + i];
		ins->ipi_base = ipi_channel;
		vinattr_to_sifsetting(vin_attr, &ins->sif_cfg);
		rc = sif_set_format(&cim->sif, ipi_channel + i, &ins->fmt, false, IN_CHANNEL);
		if (rc < 0)
			vio_err("[S%d][ipi%d]%s failed to call sif_set_format\n",
				vctx->ctx_id, ipi_channel, __func__);
	}

	vio_info("[S%d][ipi%d]%s done\n", vctx->ctx_id, ipi_channel, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set cim attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] cim_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_subdev_init(struct vio_video_ctx *vctx, cim_attr_t *cim_attr)
{
	s32 ret = 0;
	struct vio_node *vnode;
	struct vin_node_subdev *subdev;
	struct j6_cim_dev *cim;
	struct vio_subdev *vdev;
	struct j6_vin_node_dev *vin_node_dev;
	// struct vio_framemgr *framemgr;
	// struct vio_frame *frame;
	u8 mipi_index;
	u8 vc_index;
	struct vin_cim_private_s *cim_priv_attr;
	cim_func_desc_t *func;
	// u64 flags = 0;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	subdev->ctx_id = vctx->ctx_id;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);

	ret = cim_config_param_check(cim_attr);
	if (ret < 0)
		return -EFAULT;

	if (vdev->id != VNODE_ID_SRC)
		return 0;

	cim = cim_get_dev(vin_node_dev->hw_id);
	mipi_index = (u8)cim_attr->mipi_rx;
	vc_index = (u8)cim_attr->vc_index;
	cim_priv_attr->ipi_index = vc_index;
	func = &cim_attr->func;
	cim->sif.ipi_base = vc_index;
	cim->sif.ipi_channel_num = cim_attr->ipi_channel;

	init_waitqueue_head(&cim_priv_attr->cim_done_wq);
	osal_spin_init(&cim_priv_attr->ipi_slock);

	if (func->enable_frame_id == 1u) {
		// cim_set_frame_id(cim->base_reg, cim_priv_attr->ipi_index,
		// 		func->enable_frame_id, func->set_init_frame_id);
		cim_priv_attr->initial_frameid = 1u;
		cim_priv_attr->cnt_shift = 0;
		cim_priv_attr->last_hw_frameid = 0;
	}
	cim_priv_attr->tpg_en = func->enable_pattern;
	cim_priv_attr->fps_ctrl.dynamic_fps.skip_mode = func->skip_frame;
	if (cim_priv_attr->fps_ctrl.dynamic_fps.skip_mode == 1) {
		cim_priv_attr->fps_ctrl.curr_cnt = 0;
		cim_priv_attr->fps_ctrl.dynamic_fps.out_fps = func->output_fps;
		cim_priv_attr->fps_ctrl.dynamic_fps.in_fps = func->input_fps;
	}
	osal_atomic_set(&cim_priv_attr->fps_ctrl.lost_this_frame, 0);
	osal_atomic_set(&cim_priv_attr->fps_ctrl.lost_next_frame, 0);

	vdev->leader = 1;
	vnode->leader = 1;
	// vnode->get_timestamps = 1;
	vnode->frame_work = cim_frame_work;

	osal_set_bit((u32)CIM_IPI0_USED + (u32)cim_priv_attr->ipi_index,
		     &cim->state);
	osal_set_bit((u32)VIO_NODE_INIT, &vnode->state);
	osal_set_bit((u32)VIO_NODE_DMA_OUTPUT, &vnode->state);
	cim->vnode[cim_priv_attr->ipi_index] = vnode;

	// framemgr = vdev->cur_fmgr;
	// vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	// frame = peek_frame(framemgr, FS_REQUEST);
	// vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/

	// framemgr = vdev->cur_fmgr;

	vio_info("[S%d][C%d][ipi%d]%s\n", vnode->flow_id, vnode->ctx_id,
		 cim_priv_attr->ipi_index, __func__);

	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief:
 * @param[in] *subdev
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void cim_check_exit_state(const struct vin_node_subdev *subdev)
{
	u32 cnt = CIM_TIMEOUT_CNT;
	struct vin_cim_private_s *cim_priv_attr;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);

	while (cnt > 0u) {
		if (cim_priv_attr->irq_status != CIM_FRAME_START)
			break;

		mdelay(CIM_MSLEEP_TIME); /*PRQA S 2880*/
		cnt--;
	}
	vio_info("%s cnt %d status %d\n", __func__, cnt,
		 cim_priv_attr->irq_status);
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Start cim from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] tpn_fps: testpattern sfps
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_subdev_start(struct vio_video_ctx *vctx, u32 tpn_fps)
{
	s32 ret = 0;
	struct j6_cim_dev *cim;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	u8 ipi_index;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_irq_ctx ctx = {0};
	struct cim_attr cim_attr;
	int rc;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	cim = cim_get_dev(vin_node_dev->hw_id);
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_attr = subdev->vin_attr.vin_node_attr.cim_attr;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	cim_priv_attr->irq_status = 0;
	cim_priv_attr->start_flag = 1;
	cim_priv_attr->sw_frameid = 0;
	cim_priv_attr->force_drop = 0;
	ipi_index = cim_priv_attr->ipi_index;
	osal_atomic_set(&cim->enable_cnt[ipi_index], 0);
	cim->cur_output_flag[ipi_index] = 0;
	osal_atomic_set(&cim->backup_fcount[ipi_index], 0);
	osal_atomic_set(&cim->sensor_fcount[ipi_index], 0);

	ctx.sink_ctx = (struct cam_ctx *)vnode->ich_subdev[VIN_MAIN_FRAME];
	if (cim_priv_attr->ddr_en) {
		ctx.buf_ctx = (struct cam_ctx *)vnode->och_subdev[VIN_MAIN_FRAME];
		ctx.buf = NULL;
		ctx.next_buf = NULL;
	}
	if (cim_priv_attr->embeded_en) {
		ctx.emb_buf_ctx = (struct cam_ctx *)vnode->och_subdev[VIN_EMB];
		ctx.emb_buf = NULL;
	}
	rc = sif_set_ctx(&cim->sif, ipi_index, &ctx);
	if (rc < 0)
		vio_err("[S%d][ipi%d]%s failed to call sif_set_ctx\n",
			vctx->ctx_id, ipi_index, __func__);

	//	if (cim_priv_attr->ddr_en)
	//		cim_config_next_frame_addr(vnode, VIN_MAIN_FRAME);

	//	if (cim_priv_attr->embeded_en)
	//		cim_config_next_frame_addr(vnode, VIN_EMB);

	rc = sif_set_state(&cim->sif, ipi_index, 1, false);

	if (rc < 0)
		vio_err("[S%d]%s failed to call sif_set_state\n",
			vnode->flow_id, __func__);

	vio_info("[S%d]%s\n", vnode->flow_id, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Stop cim from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_subdev_stop(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct j6_cim_dev *cim;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	u8 ipi_index;
	struct vin_cim_private_s *cim_priv_attr;
	struct sif_irq_ctx ctx = {0};
	struct cim_attr cim_attr;
	int rc;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	cim = cim_get_dev(vin_node_dev->hw_id);
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_attr = subdev->vin_attr.vin_node_attr.cim_attr;
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	cim_priv_attr->start_flag = 0;
	ipi_index = cim_priv_attr->ipi_index;

	cim_check_exit_state(subdev);

	rc = sif_set_state(&cim->sif, ipi_index, 0, false);

	if (rc < 0)
		vio_err("[S%d]%s failed to call sif_set_state\n",
			vnode->flow_id, __func__);

	if (cim_priv_attr->ddr_en) {
		ctx.buf_ctx = NULL;
		ctx.buf = NULL;
		ctx.next_buf = NULL;
	}
	if (cim_priv_attr->embeded_en) {
		ctx.emb_buf_ctx = NULL;
		ctx.emb_buf = NULL;
	}
	rc = sif_set_ctx(&cim->sif, ipi_index, &ctx);
	if (rc < 0)
		vio_err("[S%d][ipi%d]%s failed to call sif_set_ctx\n",
			vctx->ctx_id, ipi_index, __func__);

	osal_atomic_set(&vnode->rcount, 0);
	memset(&cim_priv_attr->fps[0], 0, sizeof(struct fps_stats));
	memset(&cim_priv_attr->fps[1], 0, sizeof(struct fps_stats));
	vio_info("[S%d]%s\n", vnode->flow_id, __func__);

	return ret;
}

void cim_config_next_frame_addr(struct vio_node *vnode, u8 chn)
{
	struct j6_cim_dev *cim;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_subdev *vdev;
	u32 ipi_index, dma_en;
	u32 flow_id = 0;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_cim_private_s *cim_priv_attr;
	u64 flags = 0;

	if (chn == VIN_DDRIN)
		vdev = vnode->ich_subdev[0];
	else
		vdev = vnode->och_subdev[chn];
	if (vdev == NULL) {
		vio_err("%s error subdev null,flow_id %d", __func__,
			vnode->flow_id);
		return;
	}
	flow_id = vnode->flow_id;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	vin_node_dev = subdev->vin_node_dev;
	cim = cim_get_dev(vin_node_dev->hw_id);
	ipi_index = cim_priv_attr->ipi_index;
	osal_atomic_inc(&cim->backup_fcount[ipi_index]);
	//osal_atomic_set(&vnode->flow_id, flow_id);

	framemgr = vdev->cur_fmgr;
	vio_e_barrier_irqs(framemgr, flags); /*PRQA S 2996*/
	frame = peek_frame(framemgr, FS_REQUEST);
	vio_x_barrier_irqr(framemgr, flags); /*PRQA S 2996*/
	framemgr_print_queues(framemgr);
	if (frame != NULL) {
		switch (vdev->id) {
		case VNODE_ID_SRC:
			// TODO hzj
			// dma_set_base_addr(cim->base_reg, ipi_index, DDRIN_WR_CH, frame->vbuf.iommu_paddr[0][0]);
			// cim_set_rd_en(cim->base_reg, ipi_index, 0);
			// cim_set_rd_en(cim->base_reg, ipi_index, 1);
			break;
		case VNODE_ID_CAP:
			vio_info(
				"[S%d] dma_set_base_addr cap iommu_paddr 0x%x\n",
				vnode->flow_id, frame->vbuf.iommu_paddr[0][0]);
			//				sif_dma_addr_cfg(&cim->sif, ipi_index, frame->vbuf.iommu_paddr[0][0], MAIN_IMG);
			//				if (cim_priv_attr->yuv_format == 1u) {
			//					sif_dma_addr_cfg(&cim->sif, ipi_index,
			//								frame->vbuf.iommu_paddr[0][1], MAIN_IMG);
			//				}
			break;
		case VNODE_ID_CAP + VIN_EMB:
			//				sif_dma_addr_cfg(&cim->sif, ipi_index, frame->vbuf.iommu_paddr[0][0], EMB_DATA_POST);
			break;
		default:
			break;
		}

		vio_e_barrier_irqs(framemgr, flags); /*PRQA S 2996*/
		trans_frame(framemgr, frame, FS_PROCESS);
		vio_x_barrier_irqr(framemgr, flags); /*PRQA S 2996*/
		dma_en = 1;
	} else {
		vio_warn("S[%d] chn%d %s invalid work\n", flow_id, chn,
			 __func__);
		dma_en = 0;
	}

	//osal_atomic_inc(&cim->enable_cnt[ipi_index]);
	vio_dbg("S[%d] chn%d %s: done dma_en%d\n", flow_id, chn, __func__,
		dma_en); /*PRQA S 0685,1294*/
	return;
}

#ifdef CONFIG_HOBOT_VIO_STL
static s32 cim_check_irq_misinfo(struct j6_cim_dev *cim,
				 struct cim_irq_src_s *irq_src)
{
	/*  if (irq_src->cim_intr_fe != 0 || irq_src->cim_intr_fs != 0) */
	/* return 0; */

	/* if (irq_src->cim_intr_herr != 0 || irq_src->cim_intr_werr != 0) */
	/* return 0; */

	/* if (irq_src->cim_dma_status != 0 || irq_src->fifo_intr.dma_status != 0) */
	/* return 0; */

	/* cim_stl_send_diag_event(FUSA_SW_ERR_CODE, DiagMsgLevel3, (u16)EventIdIntMisinfoErr); */
	/* vio_err("%s error\n", __func__); */
	/* return -1; */
	return 0;
}
#endif

s32 cim_skip_fusa_check(struct j6_cim_dev *cim, u8 ipi_channel)
{
	struct vio_node *vnode;
	struct vin_node_subdev *subdev;
	struct vio_subdev *vdev;
	struct vin_cim_private_s *cim_priv_attr;

	vnode = cim->vnode[ipi_channel];
	if (vnode == NULL ||
	    osal_test_bit((s32)VIO_NODE_START, &vnode->state) == 0)
		return 1;

	vdev = vnode->ich_subdev[0];
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_priv_attr = cim_get_priv_by_subdev(subdev);

	return cim_priv_attr->sw_frameid > (u32)fusa_skip_num;
}

#if 0
static void cim_vdev_init(struct vio_subdev *vdev)
{
	osal_spin_init(&vdev->slock);/*PRQA S 3334*/
	osal_atomic_set(&vdev->refcount, 0);
}

void j6_cim_subdev_init(struct j6_cim_dev *cim)
{
	u32 i;
	struct cim_subdev *subdev;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		subdev = &cim->subdev[i];
		cim_vdev_init(&subdev->vdev);
		osal_complete_init(&subdev->stop_complete);
	}
}
#endif

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: clk enable
 * @retval 0: success
 * @retval <0: fail
 * @param[in] enable: Enable or disable
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void cim_set_clk_enable(u32 enable)
{
	if (enable == 1) {
		/* clock TODO */
		/* vio_set_clk_rate("cam_apb", MAX_CAM_APB_FREQ); */
		/* vio_clk_enable("cam_apb"); */
		/* vio_set_clk_rate("cam_dma", MAX_CIM_DMA_FREQ); */
		/* vio_clk_enable("cam_dma"); */
	} else {
		/* vio_clk_disable("cam_dma"); */
		/* vio_clk_disable("cam_apb"); */
	}
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: close subdev
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vdev: Abstraction of vio sub-device
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void cim_subdev_close(struct vio_subdev *vdev)
{
	u32 flow_id = 0;
	struct vio_framemgr *framemgr;
	struct vio_node *vnode;
	struct j6_cim_dev *cim;
	struct vin_node_subdev *subdev;
	struct vin_cim_private_s *cim_priv_attr;

	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	cim_priv_attr = cim_get_priv_by_subdev(subdev);
	cim = cim_get_dev(subdev->vin_node_dev->hw_id);
	if (osal_atomic_read(&vdev->refcount) == 0) {
		if (vnode != NULL) {
			osal_clear_bit((s32)VIO_NODE_INIT, &vnode->state);
			vnode->state = 0;
			flow_id = vnode->flow_id;
			//ret = vio_group_task_stop(vnode->gtask);
			// if (ret < 0)
			// 	vio_err("vio_group_task_stop failed, ret = %d\n", ret);
			osal_clear_bit((u32)VIO_NODE_SHOT, &vnode->state);
		}
		vdev->state = 0;
		vdev->reqbuf_flag = 0;
		/* osal_clear_bit((u32)CIM_IPI0_USED + cim_priv_attr->ipi_index, &cim->state); */
		cim->sif.insts[cim_priv_attr->ipi_index].frameid_cnt = 0;
		memset(cim_priv_attr, 0, sizeof(struct vin_cim_private_s));
		memset(&subdev->vin_attr, 0, sizeof(vin_attr_t));
	}

	vio_info("[S%d] %s\n", flow_id, __func__);
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cim reset
 * @retval: None
 * @param[in] cim: cim hardware abstract architecture
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void cim_sw_rst(struct j6_cim_dev *cim)
{
	vio_reset_module((u32)CIM_RST, SOFT_RST_ALL);
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cim close
 * @retval: None
 * @param[in] cim: cim hardware abstract architecture
 * @param[in] rst_en: Whether to perform reset
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void cim_device_close(struct j6_cim_dev *cim, u32 rst_en)
{
	osal_mutex_lock(&cim->mlock);
	if (osal_atomic_dec_return(&cim->open_cnt) == 0) {
		osal_clear_bit((s32)CIM_OTF_INPUT, &cim->state);
		osal_clear_bit((s32)CIM_DMA_INPUT, &cim->state);
		osal_clear_bit((s32)CIM_REUSE_SHADOW0, &cim->state);
		cim->state = 0;
		disable_irq((u32)cim->irq);

		if (rst_en == 1) {
			cim_sw_rst(cim);
			sif_reset(&cim->sif);
		}
		cim_set_clk_enable(0);
		sif_runtime_suspend(cim->sif.dev);
	}
	osal_mutex_unlock(&cim->mlock);
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Open the calling function of the cim device
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */

s32 cim_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0, i;
	struct j6_cim_dev *cim;
	struct j6_vin_node_dev *vin_node_dev;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	cim = cim_get_dev(vin_node_dev->hw_id);
	cim->vin_node_dev = vin_node_dev;

	osal_mutex_lock(&cim->mlock);
	if (osal_atomic_inc_return(&cim->open_cnt) == 1) {
		cim_set_clk_enable(1);
		sif_runtime_resume(cim->sif.dev);
		if (rst_en == 1) {
			cim_sw_rst(cim);
			sif_reset(&cim->sif);
		}
		// cim_set_ipis_enable(cim->base_reg, 0);

		for (i = 0; i < CIM_IPI_MAX_NUM; i++) {
			osal_atomic_set(&cim->backup_fcount[i], 0);
			osal_atomic_set(&cim->sensor_fcount[i], 0);
			osal_atomic_set(&cim->enable_cnt[i], 0);
		}

		osal_atomic_set(&cim->rsccount, 0);
		memset(cim->hw_drop_count, 0, sizeof(u32) * VIO_MAX_STREAM);
		memset(cim->sw_drop_count, 0, sizeof(u32) * VIO_MAX_STREAM);
		memset(cim->isp_drop_count, 0, sizeof(u32) * VIO_MAX_STREAM);
		// enable_irq((u32)cim->irq);
#if 0 // TODO hobot_systime
		hobot_get_global_time_type(&cim->global_time_type);
#endif
	}
	osal_mutex_unlock(&cim->mlock);
	vio_info("%s open_cnt :%d", __func__, osal_atomic_read(&cim->open_cnt));
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Close the calling function of the cim device
 * @retval 0: success
 * @retval <0: fail
 * @param[in] vctx: vio_video_ctx
 * @param[in] rst_en: Whether to perform reset
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_close(struct vio_video_ctx *vctx, u32 rst_en)
{
	struct j6_cim_dev *cim;
	struct vio_subdev *vdev;
	struct vio_node *vnode;
	struct j6_vin_node_dev *vin_node_dev;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	cim = cim_get_dev(vin_node_dev->hw_id);
	if (vctx->state & BIT(VIO_VIDEO_OPEN)) {
		cim_device_close(cim, rst_en);
		return 0;
	}

	//vio_group_close(vnode);
	if (vctx->state & BIT(VIO_VIDEO_START)) {
		//vio_group_stop_sensor(vnode->flow_id);
		cim_subdev_stop(vctx);
	}
	cim_subdev_close(vdev);
	cim_device_close(cim, rst_en);

	vctx->state = BIT(VIO_VIDEO_CLOSE);

	//osal_spin_lock(&vdev->slock);
	//osal_clear_bit(vctx->ctx_index, &vdev->val_ctx_mask);
	//osal_spin_unlock(&vdev->slock);
	vio_info("[S%d] %s open_cnt %d\n", vnode->flow_id, __func__,
		 osal_atomic_read(&cim->open_cnt));

	return 0;
}

void cim_isp_write_reg(uint32_t flow_id, uint32_t offset, uint32_t *value)
{
	struct j6_cim_dev *cim;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;

	vnode = vio_get_vnode(flow_id, VIN_MODULE);
	vdev = vnode->ich_subdev[0];
	if (vdev == NULL) {
		vio_err("%s error subdev null,flow_id %d", __func__,
			vnode->flow_id);
		return;
	}
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	vin_node_dev = subdev->vin_node_dev;
	cim = cim_get_dev(vin_node_dev->hw_id);
	if (cim == NULL) {
		vio_err("%s %d cim register base address is null\n", __func__,
			__LINE__);
		return;
	}

	vio_dbg("flow id:%d offset:0x%x value:0x%x", flow_id, offset, *value);
	return writel(*value, cim->base_reg + offset);
}

s32 cim_isp_read_reg(uint32_t flow_id, uint32_t offset, uint32_t *value)
{
	s32 ret = 0;
	struct j6_cim_dev *cim;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;

	vnode = vio_get_vnode(flow_id, VIN_MODULE);
	vdev = vnode->ich_subdev[0];
	if (vdev == NULL) {
		vio_err("%s error subdev null,flow_id %d", __func__,
			vnode->flow_id);
		return -EFAULT;
	}
	subdev = container_of(vdev, struct vin_node_subdev,
			      vdev); /*PRQA S 2810,0497*/
	vin_node_dev = subdev->vin_node_dev;
	cim = cim_get_dev(vin_node_dev->hw_id);
	if (cim == NULL) {
		vio_err("%s %d cim register base address is null\n", __func__,
			__LINE__);
		return -EFAULT;
	}

	*value = readl(cim->base_reg + offset);
	vio_dbg("flow id:%d offset:0x%x value:0x%x", flow_id, offset, *value);
	return ret;
}

void cim_frame_work(struct vio_node *vnode)
{
	cim_config_next_frame_addr(vnode, VIN_DDRIN);

	vio_dbg("[S%d][N%d][C%d] %s done \n", vnode->flow_id, vnode->id,
		vnode->ctx_id, __func__);
}
