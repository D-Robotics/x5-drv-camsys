/**
 * @file: hobot_gdc_ops.c
 * @brief       gdc driver internal api
 * @details     provide gdc process and cofigure api
 * @author      kaikai.sun@horizon.ai
 * @date        2023-3-27
 * @version     v0.0.1
 * @copyright   Copyright (C) 2022 Horizon Robotics Inc.
 * @NO{S09E03C01}
 * @ASIL{B}
 */

#define pr_fmt(fmt)    "[GDC ops]:" fmt
#include "hobot_dev_gdc.h"
#include "gdc_hw_api.h"
#include "hobot_gdc_ops.h"

static void gdc_subdev_process(struct vio_subdev *vdev);
/**
 * Purpose: gdc default color bar
 * Value: bit0~7: v, bit 8~15 u, bit 16~23 y.
 * Range: hobot_gdc_ops.c
 * Attention: NA
 */
int g_default_color = 0x008080;

#ifndef HOBOT_MCU_CAMSYS
module_param(g_default_color, int, 0644);/*PRQA S 0605,0636,4501*/
#endif

/**
 * Purpose: ko version
 * Value: NA
 * Range: hobot_gdc_ops.c
 * Attention: NA
 */
static struct vio_version_info g_gdc_version = {
	.major = 1,
	.minor = 0
};

s32 gdc_get_version(struct vio_version_info *version)
{
	s32 ret = 0;

	(void)memcpy(version, &g_gdc_version, sizeof(struct vio_version_info));

	return ret;
}
/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief gdc character device node open interface
* @param[in] *vctx: vpf framework context
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
s32 gdc_subdev_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct hobot_gdc_dev *gdc;

	gdc = (struct hobot_gdc_dev *)vctx->device;
	osal_mutex_lock(&gdc->mlock);

	if (osal_atomic_inc_return(&gdc->open_cnt) == 1) {
		osal_sema_init(&gdc->gdc_hw_resource, 1);
		osal_sema_init(&gdc->gdc_done_resource, 0);
		// vio_set_clk_rate("gdc_core", 600000000); //600M
		// vio_set_clk_rate("gdc_axi", 600000000);
		// vio_set_clk_rate("vse_axi", 600000000);
		// vio_set_clk_rate("gdc_hclk", 199875000);
		vio_clk_enable("gdc_core");
		vio_clk_enable("gdc_axi");
		vio_clk_enable("vse_axi");
		vio_clk_enable("gdc_hclk");
#if defined CONFIG_HOBOT_J5
		gdc_set_irq_mask(gdc->base_reg, 1);
#elif defined (CONFIG_HOBOT_VIO_STL)
		gdc_fusa_set_pwd(gdc->base_reg, PASSWD_KEY);
		gdc_fusa_set_irq_mask(gdc->base_reg, 0);
		if (gdc->stl.stl_ops.stl_enable != NULL)
			gdc->stl.stl_ops.stl_enable(gdc);
#endif
	}
	osal_mutex_unlock(&gdc->mlock);
	vctx->state = BIT((s32)VIO_VIDEO_OPEN);
	vio_info("[%s][C%d] %s: cnt %d\n", vctx->name, vctx->ctx_id,
		__func__, osal_atomic_read(&gdc->open_cnt));

	return ret;
}

/* code review E1: internal logic function, no need error return */
static void gdc_device_close(struct vio_video_ctx *vctx, u32 rst_en)
{
	struct hobot_gdc_dev *gdc;

	gdc = (struct hobot_gdc_dev *)vctx->device;
	if (osal_atomic_dec_return(&gdc->open_cnt) == 0) {
		gdc_force_stop(vctx);
#if defined CONFIG_HOBOT_J5
		gdc_set_irq_mask(gdc->base_reg, 0);
#elif defined (CONFIG_HOBOT_VIO_STL)
		if (gdc->stl.stl_ops.stl_disable != NULL)
			gdc->stl.stl_ops.stl_disable(gdc);
		gdc_fusa_set_irq_mask(gdc->base_reg, 1);
		gdc_fusa_set_pwd(gdc->base_reg, DEFAULT_PASSWD_KEY);
#endif
		if (rst_en == 1) {
			vio_reset_module((u32)GDC_RST, SOFT_RST_ALL);
		}
		vio_clk_disable("gdc_core");
		vio_clk_disable("gdc_axi");
		vio_clk_disable("vse_axi");
		vio_clk_disable("gdc_hclk");
	}
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief gdc character device node close interface
* @param[in] *vctx: vpf framework context
* @param[in] rst_en: Decide whether to rest ip
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
s32 gdc_subdev_close(struct vio_video_ctx *vctx, u32 rst_en)
{
	struct vio_subdev *vdev;
	struct vio_node *vnode;
	struct hobot_gdc_dev *gdc;
	struct gdc_subdev *subdev;

	gdc = (struct hobot_gdc_dev *)vctx->device;
	if ((vctx->state & BIT((s32)VIO_VIDEO_OPEN)) != 0u) {
		gdc_device_close(vctx, rst_en);
		return 0;
	}

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	gdc_iommu_ummap(subdev);
	osal_mutex_lock(&gdc->mlock);
	gdc_device_close(vctx, rst_en);
	osal_mutex_unlock(&gdc->mlock);

	vio_info("[%s][S%d][C%d] %s: cnt %d\n", vctx->name, vnode->flow_id, vnode->ctx_id,
		__func__, osal_atomic_read(&gdc->open_cnt));

	return 0;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief The gdc worker thread callback function is called by the worker thread created by vpf
* @param[in] *vnode: Device nodes managed by VPF
* @retval None
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
void gdc_frame_work(struct vio_node *vnode)
{
	u64 flags = 0;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	struct vio_subdev *och_vdev;
	struct vio_framemgr *framemgr, *out_framemgr;
	struct vio_frame *frame, *out_frame;
	struct gdc_iommu_addr *map_addr;

	vdev = vnode->ich_subdev[0];
	och_vdev = vnode->och_subdev[0];
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	framemgr = &vdev->framemgr;
	out_framemgr = &och_vdev->framemgr;
	map_addr = &subdev->map_addr;
	vio_dbg("[S%d][C%d] %s: start, rcount %d\n", vnode->flow_id, vnode->ctx_id,
		__func__, osal_atomic_read(&vnode->rcount));/*PRQA S 0685,1294*/

	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	frame = peek_frame(framemgr, FS_REQUEST);
	vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
	if (frame != NULL) {
		map_addr->in_iommu_addr[0] = frame->vbuf.iommu_paddr[0][0];
		map_addr->in_iommu_addr[1] = frame->vbuf.iommu_paddr[0][1];
		vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
		trans_frame(framemgr, frame, FS_PROCESS);
		vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
		vio_fps_calculate(&vdev->fdebug, &vnode->frameid);
		(void)memcpy(&vnode->frameid, &frame->frameinfo.frameid, sizeof(struct frame_id_desc));
	} else {
		vio_set_hw_free(vnode);
		framemgr_print_queues(framemgr);
		vio_err("[S%d][C%d] %s: no src frame\n", vnode->flow_id, vnode->ctx_id, __func__);
		return;
	}

	vio_e_barrier_irqs(out_framemgr, flags);/*PRQA S 2996*/
	out_frame = peek_frame(out_framemgr, FS_REQUEST);
	vio_x_barrier_irqr(out_framemgr, flags);/*PRQA S 2996*/
	if (out_frame != NULL) {
		map_addr->out_iommu_addr[0] = out_frame->vbuf.iommu_paddr[0][0];
		map_addr->out_iommu_addr[1] = out_frame->vbuf.iommu_paddr[0][1];
		vio_e_barrier_irqs(out_framemgr, flags);/*PRQA S 2996*/
		trans_frame(out_framemgr, out_frame, FS_PROCESS);
		vio_x_barrier_irqr(out_framemgr, flags);/*PRQA S 2996*/

		vio_dbg("[S%d][C%d] %s: in 0x%x, 0x%x, out 0x%x, 0x%x\n",
			vnode->flow_id, vnode->ctx_id, __func__,
			map_addr->in_iommu_addr[0], map_addr->in_iommu_addr[1],
			map_addr->out_iommu_addr[0], map_addr->out_iommu_addr[1]);

		gdc_subdev_process(vdev);
	} else {
		vio_warn("[S%d][C%d] %s: no output frame\n", vnode->flow_id, vnode->ctx_id, __func__);
		framemgr_print_queues(out_framemgr);
		vio_frame_done(vdev);//src frame invalid run;
		vio_set_hw_free(vnode);
	}
	vio_dbg("[%s][S%d][C%d] %s: done\n", vnode->name, vnode->flow_id, vnode->ctx_id, __func__);/*PRQA S 0685,1294*/
}

/**
 * This function starts the gdc block
 * Writing 0->1 transition is necessary for trigger
 * @param  gdc_settings - overall gdc settings and state
 */
/* code review E1: internal logic function, no need error return */
static void gdc_hw_start(struct hobot_gdc_dev *gdc)
{
	gdc_process_enable(gdc->base_reg, 0);
	gdc_process_enable(gdc->base_reg, 1);
	vio_dbg("[C%d] %s\n", osal_atomic_read(&gdc->ctx_id), __func__);/*PRQA S 0685,1294*/
}

/**
 * Configure the output gdc configuration address/size and buffer address/size; and resolution.
 * More than one gdc settings can be accessed by index to a gdc_settings_t.
 */
/* code review E1: internal logic function, no need error return */

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief init gdc hardware
* @param[in] *gdc: gdc ip device
* @param[in] *gdc_settings: gdc settings parameters
* @retval None
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static void gdc_init(struct gdc_subdev *subdev)
{
	struct hobot_gdc_dev *gdc;

	gdc = subdev->gdc;
	gdc_process_enable(gdc->base_reg, 0);
	gdc_process_reset(gdc->base_reg, 1);
	gdc_process_reset(gdc->base_reg, 0);
	gdc_set_default_ch1(gdc->base_reg, ((u32)g_default_color >> SHIFT_16) & MASK_8);
	gdc_set_default_ch2(gdc->base_reg, ((u32)g_default_color >> SHIFT_8) & MASK_8);
	gdc_set_default_ch3(gdc->base_reg, (u32)g_default_color & MASK_8);
}

static void gdc_config_binary(struct gdc_subdev *subdev)
{
	struct hobot_gdc_dev *gdc;

	gdc = subdev->gdc;
	gdc_set_config_addr(gdc->base_reg, subdev->map_addr.bin_iommu_addr);
	gdc_set_config_size(gdc->base_reg,
		subdev->gdc_setting.gdc_config.config_size / CONFIG_SIZE_OFFSET);

	vio_dbg("%s: config_addr 0x%x config_size %x\n",/*PRQA S 0685,1294*/
		__func__,
		subdev->map_addr.bin_iommu_addr,
		subdev->gdc_setting.gdc_config.config_size);
}

s32 gdc_iommu_map(struct gdc_subdev *subdev)
{
	s32 ret = 0;
	gdc_settings_t *gdc_setting;
	struct vio_subdev *vdev;
	struct vio_frame *frame;
	struct vio_node *vnode;

	vdev = &subdev->vdev;
	vnode = vdev->vnode;
	frame = &subdev->bin_frame;
	gdc_setting = &subdev->gdc_setting;

	if (frame->frameinfo.ion_id[0] != gdc_setting->binary_ion_id)
		frame->buf_shared = 0;
	frame->frameinfo.ion_id[0] = gdc_setting->binary_ion_id;
	frame->frameinfo.is_contig = 1;
	frame->vbuf.group_info.bit_map = 1;

	if (frame->buf_shared == 0 && gdc_setting->binary_ion_id != 0) {
		vio_frame_iommu_unmap(vdev->iommu_dev, frame);
		ret = vio_frame_iommu_map(vdev->iommu_dev, frame);
		if (ret < 0) {
			vio_err("[S%d]%s: bin_frame vio_frame_iommu_map failed\n",
				vnode->flow_id, __func__);
			return ret;
		}
		subdev->map_addr.bin_iommu_addr = frame->vbuf.iommu_paddr[0][0] +
			gdc_setting->binary_offset;
	}

	return ret;
}

/* code review E1: internal logic function, no need error return */
void gdc_iommu_ummap(struct gdc_subdev *subdev)
{
	struct vio_subdev *vdev;
	struct vio_frame *frame;

	vdev = &subdev->vdev;
	frame = &subdev->bin_frame;
	vio_frame_iommu_unmap(vdev->iommu_dev, frame);
	frame->buf_shared = 0;
}

void gdc_attr_trans_to_settings(gdc_attr_t *gdc_attr, gdc_ichn_attr_t *ichn_attr,
					gdc_ochn_attr_t *ochn_attr, gdc_settings_t *gdc_setting)
{
	if (!gdc_setting)
		return;

	if (gdc_attr != NULL) {
		gdc_setting->gdc_config.config_addr = gdc_attr->config_addr;
		gdc_setting->gdc_config.config_size = gdc_attr->config_size;
		gdc_setting->gdc_config.div_width = gdc_attr->div_width;
		gdc_setting->gdc_config.div_height = gdc_attr->div_height;
		gdc_setting->gdc_config.total_planes = gdc_attr->total_planes;
		gdc_setting->binary_ion_id = gdc_attr->binary_ion_id;
		gdc_setting->binary_offset = gdc_attr->binary_offset;
	}

	if (ichn_attr != NULL) {
		gdc_setting->gdc_config.input_width = ichn_attr->input_width;
		gdc_setting->gdc_config.input_height = ichn_attr->input_height;
		gdc_setting->gdc_config.input_stride = ichn_attr->input_stride;
	}

	if (ochn_attr != NULL) {
		gdc_setting->gdc_config.output_width = ochn_attr->output_width;
		gdc_setting->gdc_config.output_height = ochn_attr->output_height;
		gdc_setting->gdc_config.output_stride = ochn_attr->output_stride;
	}
}

void gdc_settings_trans_to_attr(gdc_attr_t *gdc_attr, gdc_ichn_attr_t *ichn_attr,
					gdc_ochn_attr_t *ochn_attr, gdc_settings_t *gdc_setting)
{
	if (!gdc_setting)
		return;

	if (gdc_attr != NULL) {
		gdc_attr->config_addr = gdc_setting->gdc_config.config_addr;
		gdc_attr->config_size = gdc_setting->gdc_config.config_size;
		gdc_attr->div_width = gdc_setting->gdc_config.div_width;
		gdc_attr->div_height = gdc_setting->gdc_config.div_height;
		gdc_attr->total_planes = gdc_setting->gdc_config.total_planes;
		gdc_attr->binary_ion_id = gdc_setting->binary_ion_id;
		gdc_attr->binary_offset = gdc_setting->binary_offset;
	}

	if (ichn_attr != NULL) {
		ichn_attr->input_width = gdc_setting->gdc_config.input_width;
		ichn_attr->input_height = gdc_setting->gdc_config.input_height;
		ichn_attr->input_stride = gdc_setting->gdc_config.input_stride;
	}

	if (ochn_attr != NULL) {
		ochn_attr->output_width = gdc_setting->gdc_config.output_width;
		ochn_attr->output_height = gdc_setting->gdc_config.output_height;
		ochn_attr->output_stride = gdc_setting->gdc_config.output_stride;
	}
}

/* code review E1: internal logic function, no need error return */
static void gdc_config_input(struct gdc_subdev *subdev)
{
	u32 lineoffset, height;
	u32 total_planes;
	u32 *input_addr;
	struct hobot_gdc_dev *gdc;
	struct gdc_config *gdc_cfg;

	gdc = subdev->gdc;
	gdc_cfg = &subdev->gdc_setting.gdc_config;
	total_planes = gdc_cfg->total_planes;
	input_addr = subdev->map_addr.in_iommu_addr;
	vio_dbg("%s: plane_num %d stride %d height %d div_w %d div_h %d\n",/*PRQA S 0685,1294*/
			__func__, total_planes,
			gdc_cfg->input_stride, gdc_cfg->input_height,
			gdc_cfg->div_width, gdc_cfg->div_height);

	gdc_set_rdma_img_width(gdc->base_reg, gdc_cfg->input_width);
	gdc_set_rdma_img_height(gdc->base_reg, gdc_cfg->input_height);
	if (total_planes > YUV_PLANE_0) {
		lineoffset = gdc_cfg->input_stride;
		height = gdc_cfg->input_height;
		gdc_set_rdma0_img_addr(gdc->base_reg, input_addr[YUV_PLANE_0]);
		gdc_set_rdma0_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P0 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P0 addr 0x%x\n", __func__, input_addr[YUV_PLANE_0]);/*PRQA S 0685,1294*/
	}

	if (total_planes > YUV_PLANE_1) {
		lineoffset = gdc_cfg->input_stride >> gdc_cfg->div_width;
		height = gdc_cfg->input_height >> gdc_cfg->div_height;
		gdc_set_rdma1_img_addr(gdc->base_reg, input_addr[YUV_PLANE_1]);
		gdc_set_rdma1_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P1 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P1 addr 0x%x\n", __func__, input_addr[YUV_PLANE_1]);/*PRQA S 0685,1294*/
	}

	if (total_planes > YUV_PLANE_2) {
		lineoffset = gdc_cfg->input_stride >> gdc_cfg->div_width;
		height = gdc_cfg->input_height >> gdc_cfg->div_height;
		gdc_set_rdma2_img_addr(gdc->base_reg, input_addr[YUV_PLANE_2]);
		gdc_set_rdma2_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P2 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P2 addr 0x%x\n", __func__, input_addr[YUV_PLANE_2]);/*PRQA S 0685,1294*/
	}
}

/* code review E1: internal logic function, no need error return */
static void gdc_config_output(struct gdc_subdev *subdev)
{
	u32 lineoffset, height;
	u32 total_planes;
	u32 *output_addr;
	struct hobot_gdc_dev *gdc;
	struct gdc_config *gdc_cfg;

	gdc = subdev->gdc;
	gdc_cfg = &subdev->gdc_setting.gdc_config;
	total_planes = gdc_cfg->total_planes;
	output_addr = subdev->map_addr.out_iommu_addr;
	vio_dbg("%s: plane_num %d stride %d height %d div_w %d div_h %d\n",/*PRQA S 0685,1294*/
			__func__, total_planes,
			gdc_cfg->output_stride, gdc_cfg->output_height,
			gdc_cfg->div_width, gdc_cfg->div_height);

	gdc_set_wdma_img_width(gdc->base_reg, gdc_cfg->output_width);
	gdc_set_wdma_img_height(gdc->base_reg, gdc_cfg->output_height);
	if (total_planes > YUV_PLANE_0) {
		lineoffset = gdc_cfg->output_stride;
		height = gdc_cfg->output_height;
		gdc_set_wdma0_img_addr(gdc->base_reg, output_addr[YUV_PLANE_0]);
		gdc_set_wdma0_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P0 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P0 addr 0x%x\n", __func__, output_addr[YUV_PLANE_0]);/*PRQA S 0685,1294*/
	}

	if (total_planes > YUV_PLANE_1) {
		lineoffset = gdc_cfg->output_stride >> gdc_cfg->div_width;
		height = gdc_cfg->output_height >> gdc_cfg->div_height;
		gdc_set_wdma1_img_addr(gdc->base_reg, output_addr[YUV_PLANE_1]);
		gdc_set_wdma1_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P1 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P1 addr 0x%x\n", __func__, output_addr[YUV_PLANE_1]);/*PRQA S 0685,1294*/
	}

	if (total_planes > YUV_PLANE_2) {
		lineoffset = gdc_cfg->output_width >> gdc_cfg->div_width;
		height = gdc_cfg->output_height >> gdc_cfg->div_height;
		gdc_set_wdma2_img_addr(gdc->base_reg, output_addr[YUV_PLANE_2]);
		gdc_set_wdma2_line_offset(gdc->base_reg, lineoffset);
		vio_dbg("%s: P2 lineoffset %d height %d\n", __func__, lineoffset, height);/*PRQA S 0685,1294*/
		vio_dbg("%s: P2 addr 0x%x\n", __func__, output_addr[YUV_PLANE_2]);/*PRQA S 0685,1294*/
	}
}

/**
 * This function points gdc to its input resolution and yuv address and offsets
 * Shown inputs to GDC are Y and UV plane address and offsets
 * @return 0 - success;-1 - no interrupt from GDC.
 */

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief set gdc_settings parameters
* @param[in] *gdc: gdc ip device
* @param[in] *gdc_settings: gdc settings parameters
* @retval None
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static void gdc_hw_config(struct gdc_subdev *subdev)
{
	//reset gdc hw
	gdc_init(subdev);

	//config gdc binary
	gdc_config_binary(subdev);

	//config input
	gdc_config_input(subdev);

	//config outputs
	gdc_config_output(subdev);
}

static s32 gdc_check_ichn_bind_param(struct vio_subdev *vdev, struct chn_attr *chn_attr)
{
	s32 ret = 0;
	struct gdc_subdev *subdev;
	gdc_config_t *gdc_cfg;

	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	gdc_cfg = &subdev->gdc_setting.gdc_config;
	if (gdc_cfg->input_width != chn_attr->width ||
		gdc_cfg->input_height != chn_attr->height) {
		vio_err("%s: wrong resolution(%d * %d), expect(%d * %d)", __func__,
			gdc_cfg->input_width, gdc_cfg->input_height,
			chn_attr->width, chn_attr->height);
		return -EFAULT;
	}

	if (gdc_cfg->input_stride != chn_attr->wstride) {
		vio_err("%s: wrong stride %d, expect %d", __func__,
			gdc_cfg->input_stride, chn_attr->wstride);
		return -EFAULT;
	}

	if (chn_attr->format != MEM_PIX_FMT_NV12) {
		vio_err("%s: wrong format(%d), expect(%d)\n", __func__,
			chn_attr->format, MEM_PIX_FMT_NV12);
		return -EFAULT;
	}

	return ret;
}

static void gdc_set_ochn_bind_param(struct vio_subdev *vdev)
{
	struct gdc_subdev *subdev;
	gdc_config_t *gdc_cfg;

	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	gdc_cfg = &subdev->gdc_setting.gdc_config;
	vdev->chn_attr.width = gdc_cfg->output_width;
	vdev->chn_attr.height = gdc_cfg->output_height;
	vdev->chn_attr.wstride = gdc_cfg->output_stride;
	vdev->chn_attr.vstride =  gdc_cfg->output_height;
}

s32 gdc_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	s32 ret;

	if (osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) != 0)
		return CHN_BIND_REP;

	if (vdev->id == VNODE_ID_SRC) {
		ret = gdc_check_ichn_bind_param(vdev, &remote_vdev->chn_attr);
		if (ret < 0)
			return CHN_BIND_NONE;
	} else {
		gdc_set_ochn_bind_param(vdev);
	}

	return CHN_BIND_M2M;
}

/* code review E1: internal logic function, no need error return */
/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief check gdc interrupt status
* @param[in] status: interrupt status
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 gdc_check_status(u32 status)
{
	s32 ret = 0;

	if ((status & INTR_GDC_BUSY) != 0u) {
		vio_err("%s GDC busy\n", __func__);
	}
	if ((status & INTR_GDC_ERROR) != 0u) {
		if ((status & INTR_GDC_CONF_ERROR) != 0u)
			vio_err("%s GDC configuration error\n", __func__);

		if ((status & INTR_GDC_USER_ABORT) != 0u)
			vio_err("%s GDC user abort(stop/reset command)\n", __func__);

		if ((status & INTR_GDC_AXI_READER_ERROR) != 0u)
			vio_err("%s GDC AXI reader error\n", __func__);

		if ((status & INTR_GDC_AXI_WRITER_ERROR) != 0u)
			vio_err("%s GDC AXI writer error\n", __func__);

		if ((status & INTR_GDC_UNALIGNED_ACCESS) != 0u)
			vio_err("%s GDC address pointer is not aligned\n", __func__);

		if ((status & INTR_GDC_INCOMPATIBLE_CONF) != 0u)
			vio_err("%s GDC incopatible configuration\n", __func__);
		ret = -1;
	}
	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief check gdc settings parameter
* @param[in] *gdc_cfg: gdc settings parameters
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
s32 gdc_setting_check(gdc_settings_t *gdc_setting)
{
	s32 ret = 0;

	if (gdc_setting->gdc_config.input_width == 0 ||
		gdc_setting->gdc_config.input_width > GDC_MAX_WIDTH) {
		vio_err("%s: wrong input_width(%d)\n", __func__,
			gdc_setting->gdc_config.input_width);
		ret = -EFAULT;
	}

	if (gdc_setting->gdc_config.input_height == 0 ||
		gdc_setting->gdc_config.input_height > GDC_MAX_HEIGHT) {
		vio_err("%s: wrong input_height(%d)\n", __func__,
			gdc_setting->gdc_config.input_height);
		ret = -EFAULT;
	}

	if (gdc_setting->gdc_config.output_width > GDC_MAX_WIDTH) {
		vio_err("%s: wrong output_width(%d)\n", __func__,
			gdc_setting->gdc_config.output_width);
		ret = -EFAULT;
	}

	if (gdc_setting->gdc_config.output_height > GDC_MAX_HEIGHT) {
		vio_err("%s: wrong output_height(%d)\n", __func__,
			gdc_setting->gdc_config.output_height);
		ret = -EFAULT;
	}

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief force gdc hardware stop
* @param[in] *vctx: vpf framework context
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
s32 gdc_force_stop(struct vio_video_ctx *vctx)
{
	u32 status;
	s32 ret = 0;
	struct hobot_gdc_dev *gdc;

	gdc = vctx->device;
	if (gdc->state == (u32)GDC_DEV_PROCESS) {
		gdc_process_reset(gdc->base_reg, 1);
		gdc_process_reset(gdc->base_reg, 0);

		status = gdc_get_status(gdc->base_reg);
		if ((status & INTR_GDC_BUSY) != 0u) {
			osal_msleep(WAIT_TIMEOUT);
			vio_dbg("%s:status = 0x%x", __func__, status);/*PRQA S 0685,1294*/
		}
		vctx->event = (u32)VIO_FRAME_NDONE;
		osal_sema_up(&gdc->gdc_done_resource);
		vio_info("%s sucessfully\n", __func__);
	} else {
		vio_info("%s: gdc is already free\n", __func__);
	}

	return ret;
}

#ifdef CONFIG_HOBOT_VIO_STL
void gdc_interrupt_missing(struct hobot_gdc_dev *gdc)
{
	u32 status;

	status = gdc_get_irq_status(gdc->base_reg);
	if (status == 1u) {
		vio_err("gdc interrupt missing\n");
	}
}
#endif

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief start once gdc process
* @param[in] *vctx: vpf framework context
* @param[in] *gdc_cfg: gdc settings parameters
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static void gdc_subdev_process(struct vio_subdev *vdev)
{
	struct hobot_gdc_dev *gdc;
	struct vio_node *vnode;
	struct gdc_subdev *subdev;

	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	gdc = subdev->gdc;
	vnode = vdev->vnode;
	osal_atomic_set(&gdc->ctx_id, vnode->ctx_id);
	vio_set_stat_info(vnode->flow_id, GDC_MODULE, STAT_FS, vnode->frameid.frame_id);
	gdc_hw_config(subdev);
	gdc_hw_start(gdc);
	vio_loading_calculate(&gdc->loading, STAT_FS);
	gdc->state = (u32)GDC_DEV_PROCESS;

	vio_dbg("[S%d][C%d] %s done\n", vnode->flow_id, vnode->ctx_id, __func__);/*PRQA S 0685,1294*/
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief gdc interrupt handle
* @param[in] *gdc: gdc ip device
* @param[in] *status: interrupt status
* @retval None
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
void gdc_handle_interrupt(struct hobot_gdc_dev *gdc, u32 status)
{
	u32 ctx_id;
	u32 gdc_status;
	struct vio_node *vnode;

#ifdef CONFIG_HOBOT_VIO_STL
	if (status == 0) {
		vio_err("%s misinfo error\n", __func__);
		return;
	}
#endif
	ctx_id = (u32)osal_atomic_read(&gdc->ctx_id);
	vnode = &gdc->vnode[ctx_id];
	vio_dbg("[S%d][C%d] %s: status 0x%x gdc_status = 0x%x\n", vnode->flow_id, ctx_id, __func__, status, gdc_status);

	gdc_status = gdc_get_status(gdc->base_reg);
	vio_frame_done(vnode->ich_subdev[0]);
	if (gdc_check_status(gdc_status) == 0)
		vio_frame_done(vnode->och_subdev[0]);
	else
		vio_frame_ndone(vnode->och_subdev[0]);

	vio_set_stat_info(vnode->flow_id, GDC_MODULE, STAT_FE, vnode->frameid.frame_id);

	gdc->state = (u32)GDC_DEV_FREE;
	vio_loading_calculate(&gdc->loading, STAT_FE);
	vio_set_hw_free(vnode);
}

