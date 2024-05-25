// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/timekeeping.h>

#include "hobot_dev_vin_node.h"
#include "vin_node_config.h"

#include "hbn_isp_api.h"
#include "isp_drv.h"

#define ISP_DT_NAME     "verisilicon,isp"
#define ISP_DEV_NAME    "vs-isp"

static struct vio_version_info g_isp_version = {
	.major = 1,
	.minor = 0
};

static s32 isp_video_get_version(struct vio_version_info *version)
{
	memcpy(version, &g_isp_version, sizeof(struct vio_version_info));
	return 0;
}

int32_t isp_video_get_struct_size(struct vio_struct_size *vio_size)
{
	int32_t ret = 0;

	switch (vio_size->type)
	{
	case BASE_ATTR:
		vio_size->size = sizeof(isp_attr_t);
		break;
	case ICHN_ATTR:
		vio_size->size = sizeof(isp_ichn_attr_t);
		break;
	case OCHN_ATTR:
		vio_size->size = sizeof(isp_ochn_attr_t);
		break;
	case EX_ATTR:
		vio_size->size = 0;
		break;
	case OCHN_EX_ATTR:
		vio_size->size = 0;
		break;
	default:
		ret = -EINVAL;
		vio_err("Unknown isp struct type-%d\n", vio_size->type);
		break;
	}

	return ret;
}

s32 isp_get_inst(struct vio_subdev *vdev, int *inst_id)
{
	struct isp_nat_instance *isp_inst = NULL;

	isp_inst = container_of(vdev, struct isp_nat_instance, vdev);
	if (!isp_inst)
		return -1;

	if (inst_id)
		*inst_id = isp_inst->id;

	return 0;
}
EXPORT_SYMBOL(isp_get_inst);

static s32 isp_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	struct isp_nat_instance *inst;
	enum vio_bind_type bind_type = CHN_BIND_OTF;

	inst = container_of(vdev, struct isp_nat_instance, vdev);

	if (vdev) {
		if (vdev->id == VNODE_ID_SRC)
			vdev->prev = remote_vdev;
	}
	if (vdev->id == VNODE_ID_CAP) {
		vdev->chn_attr.format = MEM_PIX_FMT_NV12;
		vdev->chn_attr.height = inst->attr.crop.h;
		vdev->chn_attr.width = inst->attr.crop.w;
		vdev->chn_attr.wstride = inst->attr.crop.w;
	}

	if (online_mode)
		bind_type = CHN_BIND_OTF;
	else
		bind_type = CHN_BIND_M2M;
	inst->online_mode = online_mode;

	pr_info("%s online_mode=%d,bind_type=%d\n", __func__, online_mode, bind_type);

	return bind_type;
}

static void isp_frame_work(struct vio_node *vnode)
{
	struct vio_subdev *vdev;
	struct isp_nat_instance *inst;
	int rc = 0;

	vdev = vnode->ich_subdev[0];
	inst = container_of(vdev, struct isp_nat_instance, vdev);

	rc = isp_set_mcm_sch_offline(&inst->dev->isp_dev, vnode->ctx_id);
	if (rc)
		pr_err("%s: failed to call isp_set_mcm_sch_offline.\n", __func__);

	vio_set_hw_free(vnode);
	osal_clear_bit(VIO_NODE_SHOT, &vnode->state);
}

static s32 isp_nat_open(struct vio_video_ctx *vctx)
{
	struct isp_nat_device *nat_dev;
	int rc;

	nat_dev = (struct isp_nat_device *)vctx->device;

	if (nat_dev && vctx->id == VNODE_ID_CAP) {
		rc = isp_open(&nat_dev->isp_dev, vctx->ctx_id);
		if (rc < 0) {
			pr_err("%s failed to call isp_open(err=%d).\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}

static s32 isp_nat_close(struct vio_video_ctx *vctx)
{
	struct isp_nat_instance *inst;
	int rc = 0;

	if (!vctx->vdev)
		return 0;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	if (vctx->id == VNODE_ID_SRC) {
		pr_debug("clean stream vnode, ctxid %d, online_mode %d, stream_idx %d\n",
				vctx->ctx_id, inst->online_mode, inst->stream_idx);
		if (inst->stream_idx > -1) {
			mutex_lock(&inst->dev->stream_vnodes_lock);
			inst->dev->stream_vnodes[inst->stream_idx] = NULL;
			mutex_unlock(&inst->dev->stream_vnodes_lock);
			isp_set_stream_idx(&inst->dev->isp_dev, vctx->ctx_id, -1);
			inst->stream_idx = -1;
		}
	} else if (vctx->id == VNODE_ID_CAP) {
		pr_info("%s set isp state to CLOSED\n", __func__);
		rc = isp_close(&inst->dev->isp_dev, vctx->ctx_id);
		if (rc < 0) {
			pr_err("%s failed to call isp_close(err=%d).\n", __func__, rc);
			return rc;
		}
	}
	inst->online_mode = 0;
	memset(&inst->attr, 0, sizeof(inst->attr));
	memset(&inst->ichn_attr, 0, sizeof(inst->ichn_attr));
	memset(&inst->ochn_attr, 0, sizeof(inst->ochn_attr));
	return rc;
}

static s32 isp_get_sensor_param(struct vio_video_ctx *vctx, void *arg)
{
	s32 ret = -1;
	struct isp_nat_instance *inst;
	struct sensor_isp_ops_s *cops;
	struct _setting_param_t user_para;
	hbn_isp_sensor_param_t sensor_param;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	cops = (struct sensor_isp_ops_s *)inst->dev->sensor_ops->cops;
	if (cops && cops->sensor_get_para) {
		ret = cops->sensor_get_para(vctx->flow_id, &user_para);
		if (ret < 0) {
			pr_err("get sensor param fail\n");
			return ret;
		}

		sensor_param.lines_per_second = user_para.lines_per_second;
		sensor_param.again_max = user_para.analog_gain_max;
		sensor_param.dgain_max = user_para.digital_gain_max;
		sensor_param.exp_time_max = user_para.exposure_time_max;
		sensor_param.exp_time_min = user_para.exposure_time_min;

		ret = copy_to_user((void __user *)arg, &sensor_param, sizeof(sensor_param));
		if (ret < 0) {
			pr_err("%s copy to user fail\n", __func__);
			return ret;
		}
	}

	return ret;
}

static s32 isp_video_s_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	struct isp_nat_instance *inst;
	u32 size = 0;
	int ret = 0;

	if (!vctx || !vctx->vdev)
		return -EINVAL;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);
	pr_debug("%s: vctx->ctx_id=%d, inst->id=%d\n", __func__, vctx->ctx_id, inst->id);

	if (vctx->vdev->id == VNODE_ID_SRC) {
		switch (cmd) {
		case ctrl_id_module_ctrl:
			size = sizeof(hbn_isp_module_ctrl_t);
			break;
		case ctrl_id_exposure_attr:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case ctrl_id_hdr_exposure_attr:
			size = sizeof(hbn_isp_hdr_exposure_attr_t);
			break;
		case ctrl_id_awb_attr:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		case ctrl_id_color_process_attr:
			size = sizeof(hbn_isp_color_process_attr_t);
			break;
		case ctrl_id_ae_zone_weight_attr:
			size = sizeof(hbn_isp_ae_zone_weight_attr_t);
			break;
		case ctrl_id_af_zone_weight_attr:
			size = sizeof(hbn_isp_af_zone_weight_attr_t);
			break;
		case ctrl_id_ae_exposure_table:
			size = sizeof(hbn_isp_exposure_table_t);
			break;
		default:
			return -EINVAL;
		}

		ret = isp_set_subctrl(&inst->dev->isp_dev, vctx->ctx_id,
				      cmd, (void *)arg, size);
	}
	return ret;
}

static s32 isp_video_g_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	struct isp_nat_instance *inst;
	u32 size = 0;
	int ret = 0;

	if (!vctx || !vctx->vdev)
		return -EINVAL;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);
	pr_debug("%s: vctx->ctx_id=%d, inst->id=%d\n", __func__, vctx->ctx_id, inst->id);

	if (vctx->vdev->id == VNODE_ID_SRC) {
		switch (cmd) {
		case ctrl_id_module_ctrl:
			size = sizeof(hbn_isp_module_ctrl_t);
			break;
		case ctrl_id_exposure_attr:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case ctrl_id_hdr_exposure_attr:
			size = sizeof(hbn_isp_hdr_exposure_attr_t);
			break;
		case ctrl_id_awb_attr:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		case ctrl_id_awb_get_gain_by_temper:
			size = sizeof(hbn_isp_awb_gain_t);
			break;
		case ctrl_id_color_process_attr:
			size = sizeof(hbn_isp_color_process_attr_t);
			break;
		case ctrl_id_ae_zone_weight_attr:
			size = sizeof(hbn_isp_ae_zone_weight_attr_t);
			break;
		case ctrl_id_af_zone_weight_attr:
			size = sizeof(hbn_isp_af_zone_weight_attr_t);
			break;
		case ctrl_id_ae_statistics:
			size = sizeof(hbn_isp_ae_statistics_t);
			break;
		case ctrl_id_awb_statistics:
			size = sizeof(hbn_isp_awb_statistics_t);
			break;
		case ctrl_id_af_statistics:
			size = sizeof(hbn_isp_af_statistics_t);
			break;
		case ctrl_id_ae_exposure_table:
			size = sizeof(hbn_isp_exposure_table_t);
			break;
		case ctrl_id_sensor_param:
			ret = isp_get_sensor_param(vctx, (void *)arg);
			return ret;
		default:
			return -EINVAL;
		}

		ret = isp_get_subctrl(&inst->dev->isp_dev, vctx->ctx_id,
				      cmd, (void *)arg, size);
	}
	return ret;
}

static s32 isp_attr_check(struct isp_nat_instance *inst)
{
	isp_attr_t *isp_attr = &inst->attr;

	vpf_param_range_check(isp_attr->input_mode, 0, DDR_MODE);
	vpf_param_range_check(isp_attr->sched_mode, 0, FIFO);
	vpf_param_range_check(isp_attr->sensor_mode, 0, ISP_PWL_M);

	return 0;
}

static s32 isp_video_set_cfg(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	struct isp_nat_device *dev;
	struct vio_node **vn = NULL;
	struct isp_msg msg;
	int i, rc;

	if (!vctx || !vctx->vdev)
		return -EINVAL;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_from_user(&inst->attr, (void *)arg, sizeof(inst->attr));
	if (rc < 0)
		return rc;

	rc = isp_attr_check(inst);
	if (rc)
		return rc;

	dev = inst->dev;
	if (inst->attr.input_mode == MCM_MODE) {
		dev->isp_dev.mode = MCM_WORK_MODE;
		dev->isp_dev.insts[vctx->ctx_id].online_mcm = 1;
	} else if (inst->attr.input_mode == DDR_MODE) {
		/** handling frame buffer from previous IP modules,
		 *  we treat it as MCM work mode too.
		 */
		dev->isp_dev.mode = MCM_WORK_MODE;
		dev->isp_dev.insts[vctx->ctx_id].online_mcm = 0;
	} else {
		dev->isp_dev.mode = STRM_WORK_MODE;
		dev->isp_dev.insts[vctx->ctx_id].online_mcm = 0;
		dev->isp_dev.cur_mi_irq_ctx = 0;
		dev->isp_dev.next_mi_irq_ctx = 0;
	}
	memset(&inst->ctx, 0, sizeof(inst->ctx));

	if (vctx->id == VNODE_ID_SRC) {
		vctx->vdev->leader = 1;
		if (inst->attr.input_mode != DDR_MODE) {
			mutex_lock(&dev->stream_vnodes_lock);
			if (inst->attr.sensor_mode == ISP_DOL2_M ||
			    inst->attr.input_mode == PASSTHROUGH_MODE) {
				vn = &dev->stream_vnodes[0];
				i = 0;
			} else {
				for (i = ARRAY_SIZE(dev->stream_vnodes) - 1; i > -1; i--) {
					if (!dev->stream_vnodes[i]) {
						vn = &dev->stream_vnodes[i];
						pr_debug("got index %d for saving vnode\n", i);
						break;
					}
				}
			}
			if (!vn || *vn) {
				mutex_unlock(&dev->stream_vnodes_lock);
				pr_err("%s no stream input channel available!\n", __func__);
				return -EINVAL;
			}
			*vn = &dev->vnode[vctx->ctx_id];
			mutex_unlock(&dev->stream_vnodes_lock);
		}
		inst->stream_idx = dev->isp_dev.insts[vctx->ctx_id].online_mcm ? i : 0;
		isp_set_stream_idx(&dev->isp_dev, vctx->ctx_id, inst->stream_idx);

		msg.id = CAM_MSG_STATE_CHANGED;
		msg.inst = vctx->ctx_id;
		msg.state = CAM_STATE_INITED;
		dev->isp_dev.insts[vctx->ctx_id].state = CAM_STATE_INITED;
		pr_info("%s set isp state to INITED\n", __func__);
		return isp_post(&dev->isp_dev, &msg, true);
	}
	return 0;
}

static s32 isp_video_get_cfg(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	int rc = 0;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_to_user((void __user *)arg, &inst->attr, sizeof(inst->attr));
	if (rc) {
		pr_err("%s: copy_to_user failed!\n", __func__);
		return rc;
	}

	pr_info("%s done\n", __func__);
	return rc;
}

static s32 isp_video_set_cfg_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	return 0;
}

static s32 isp_video_get_cfg_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	return 0;
}

static void isp_get_plane(u32 format, struct vbuf_group_info *group_attr)
{
	if (format == HW_FORMAT_RAW12 || format == HW_FORMAT_RAW10 ||
	    format == HW_FORMAT_RAW8 || format == HW_FORMAT_RAW14 ||
	    format == HW_FORMAT_RAW16 || format == HW_FORMAT_RAW20) {
		group_attr->info[0].buf_attr.planecount = 1;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_RAW12;
	} else if ((format == HW_FORMAT_YUV422_10BIT) || (format == HW_FORMAT_YUV422_8BIT)) {
		group_attr->info[0].buf_attr.planecount = 2;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
	} else {
		pr_err("error format %d\n", format);
	}
}

static s32 isp_video_reqbufs(struct vio_video_ctx *vctx,
			     struct vbuf_group_info *group_attr)
{
	struct isp_nat_instance *inst;
	s32 ret = 0;
	u32 format;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);
	if (vctx->id == VNODE_ID_SRC) {
		group_attr->bit_map = 1;
		group_attr->is_contig = 1;
		switch (inst->ichn_attr.bit_width) {
		case 8:
			format = HW_FORMAT_RAW8;
			group_attr->info[0].buf_attr.wstride = inst->attr.crop.w;
			break;
		case 10:
			format = HW_FORMAT_RAW10;
			group_attr->info[0].buf_attr.wstride = inst->attr.crop.w * 2;
			break;
		case 12:
			format = HW_FORMAT_RAW12;
			group_attr->info[0].buf_attr.wstride = inst->attr.crop.w * 2;
			break;
		default:
			return -EINVAL;
		}
		group_attr->info[0].buf_attr.width = inst->attr.crop.w;
		group_attr->info[0].buf_attr.height = inst->attr.crop.h;
		group_attr->info[0].buf_attr.vstride = inst->attr.crop.h;
		isp_get_plane(format, group_attr);
		group_attr->is_alloc = 0;
	} else if (vctx->id == VNODE_ID_CAP) {
		if (inst->ochn_attr.fmt == FRM_FMT_NV12)
			format = HW_FORMAT_YUV422_8BIT;
		else
			return -EINVAL;

		group_attr->bit_map |= 1;
		group_attr->is_contig = 1;
		group_attr->info[0].buf_attr.width = inst->attr.crop.w;
		group_attr->info[0].buf_attr.height = inst->attr.crop.h;
		group_attr->info[0].buf_attr.wstride = inst->attr.crop.w;
		group_attr->info[0].buf_attr.vstride = inst->attr.crop.h;
		isp_get_plane(format, group_attr);
		group_attr->is_alloc = 0;
	}

	pr_info("%s done bit_map 0x%x planecount %d\n", __func__, group_attr->bit_map,
		group_attr->info[0].buf_attr.planecount);
	return ret;
}

static int get_csi_ipi_idx(struct vio_subdev *vdev, int *csi_idx, int *ipi_idx, int *ipi_num)
{
	struct vio_node *vnode;
	struct vin_node_subdev *subdev;
	cim_attr_t *cim_attr;

	vnode = vdev->vnode;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);
	cim_attr = &subdev->vin_attr.vin_node_attr.cim_attr;

	*ipi_num = cim_attr->ipi_channel;
	*csi_idx = cim_attr->mipi_rx;
	*ipi_idx = cim_attr->vc_index;
	return 0;
}

static s32 isp_video_streamon(struct vio_video_ctx *vctx)
{
	struct isp_nat_instance *inst, *src_inst;
	struct isp_nat_device *dev;
	int csi_idx, ipi_idx, ipi_num;
	int rc;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	if (vctx->id == VNODE_ID_SRC) {
		vctx->vdev->vnode->leader = inst->online_mode ? 0 : 1;
	} else if (vctx->id == VNODE_ID_CAP) {
		dev = inst->dev;
		src_inst = &dev->src_instance[vctx->ctx_id];
		if (src_inst->online_mode) {
			get_csi_ipi_idx(src_inst->vdev.prev, &csi_idx, &ipi_idx, &ipi_num);
			pr_info("%s stream_idx=%d,csi_idx=%d,ipi_idx=%d,ipi_num=%d\n", __func__,
				src_inst->stream_idx, csi_idx, ipi_idx, ipi_num);
			isp_set_input_select(&dev->isp_dev, src_inst->stream_idx, csi_idx, ipi_idx);
		}

		inst->ctx.is_sink_online_mode = !!src_inst->online_mode;
		inst->ctx.is_src_online_mode = !!inst->online_mode;
		inst->ctx.src_ctx = (struct cam_buf_ctx *)vctx->vdev;
		inst->ctx.ddr_en = inst->ochn_attr.ddr_en;
		if (!inst->ctx.is_sink_online_mode)
			inst->ctx.sink_ctx = (struct cam_buf_ctx *)&src_inst->vdev;
		inst->ctx.stat_ctx = (struct cam_buf_ctx *)&src_inst->vdev;
		rc = isp_set_ctx(&dev->isp_dev, vctx->ctx_id, &inst->ctx);
		if (rc < 0) {
			pr_err("%s failed to call isp_set_ctx (rc=%d)!\n", __func__, rc);
			return rc;
		}

		pr_info("%s set isp state to STARTED\n", __func__);
		return isp_set_state(&dev->isp_dev, vctx->ctx_id, 1);
	}
	return 0;
}

static s32 isp_video_streamoff(struct vio_video_ctx *vctx)
{
	struct isp_nat_instance *inst;
	struct isp_nat_device *dev;
	struct isp_irq_ctx ctx;
	int rc;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	if (vctx->id == VNODE_ID_CAP) {
		dev = inst->dev;

		pr_info("%s set isp state to STOPPED\n", __func__);
		rc = isp_set_state(&dev->isp_dev, vctx->ctx_id, 0);
		if (rc < 0) {
			pr_err("%s failed to call isp_set_state(err=%d).\n", __func__, rc);
			return rc;
		}

		memset(&ctx, 0, sizeof(ctx));
		rc = isp_set_ctx(&dev->isp_dev, vctx->ctx_id, &ctx);
		if (rc < 0) {
			pr_err("%s failed to call isp_set_ctx(err=%d).\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}

static s32 isp_ichn_attr_check(struct vio_video_ctx *vctx)
{
	struct isp_nat_instance *inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);
	isp_ichn_attr_t *isp_ichn_attr = &inst->ichn_attr;
	isp_attr_t *isp_attr;

	/* crop used by reqbuf, so need change in i/o */
	isp_attr = &inst->dev->src_instance[vctx->ctx_id].attr;
	if (isp_attr->crop.w == 0 || isp_attr->crop.h == 0) {
		isp_attr->crop.x = 0;
		isp_attr->crop.y = 0;
		isp_attr->crop.w = isp_ichn_attr->width;
		isp_attr->crop.h = isp_ichn_attr->height;
		pr_debug("isp i use input size if crop disable\n");
	}
	isp_attr = &inst->dev->cap_instance[vctx->ctx_id].attr;
	if (isp_attr->crop.w == 0 || isp_attr->crop.h == 0) {
		isp_attr->crop.x = 0;
		isp_attr->crop.y = 0;
		isp_attr->crop.w = isp_ichn_attr->width;
		isp_attr->crop.h = isp_ichn_attr->height;
		pr_debug("isp o use input size if crop disable\n");
	}

	vpf_param_range_check(isp_attr->crop.x, 0, isp_ichn_attr->width);
	vpf_param_range_check(isp_attr->crop.y, 0, isp_ichn_attr->height);
	vpf_param_range_check(isp_attr->crop.w, 0, isp_ichn_attr->width);
	vpf_param_range_check(isp_attr->crop.h, 0, isp_ichn_attr->height);

	vpf_param_range_check(isp_ichn_attr->tpg_en, 0, CAM_TRUE);
	vpf_param_range_check(isp_ichn_attr->width, 0, 5472);
	vpf_param_range_check(isp_ichn_attr->height, 0, 3076);
	vpf_param_range_check(isp_ichn_attr->fmt, FRM_FMT_NULL, FRM_FMT_UYVY);
	vpf_param_range_check(isp_ichn_attr->bit_width, 8, 12);

	return 0;
}

static s32 isp_video_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	struct cam_format fmt;
	struct cam_rect crop;
	bool hdr_en = false;
	int rc;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_from_user(&inst->ichn_attr, (void *)arg, sizeof(inst->ichn_attr));
	if (rc < 0)
		return rc;

	rc = isp_ichn_attr_check(vctx);
	if (rc)
		return rc;

	switch (inst->ichn_attr.bit_width) {
	case 8:
		fmt.format = CAM_FMT_RAW8;
		fmt.stride = inst->ichn_attr.width;
		break;
	case 10:
		fmt.format = CAM_FMT_RAW10;
		fmt.stride = inst->ichn_attr.width * 2;
		break;
	case 12:
		fmt.format = CAM_FMT_RAW12;
		fmt.stride = inst->ichn_attr.width * 2;
		break;
	default:
		return -EINVAL;
	}

	if (inst->attr.sensor_mode == ISP_DOL2_M)
		hdr_en = true;

	fmt.width = inst->ichn_attr.width;
	fmt.height = inst->ichn_attr.height;

	crop.x = inst->attr.crop.x;
	crop.y = inst->attr.crop.y;
	crop.w = inst->attr.crop.w;
	crop.h = inst->attr.crop.h;

	pr_info("%s format=%d,width=%d,stride=%d,height=%d,hdr_en=%d\n", __func__,
		fmt.format, fmt.width, fmt.stride, fmt.height, hdr_en);
	return isp_set_iformat(&inst->dev->isp_dev, vctx->ctx_id, &fmt, &crop, hdr_en);
}

static s32 isp_ochn_attr_check(struct isp_nat_instance *inst)
{
	isp_ochn_attr_t *isp_ochn_attr = &inst->ochn_attr;

	/* todo: check or remove ochn out */
	vpf_param_range_check(isp_ochn_attr->ddr_en, 0, CAM_TRUE);
	vpf_param_range_check(isp_ochn_attr->fmt, FRM_FMT_NV12, FRM_FMT_NV12);
	vpf_param_range_check(isp_ochn_attr->bit_width, 8, 8);

	return 0;
}

static s32 isp_video_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	struct cam_format fmt;
	int rc;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_from_user(&inst->ochn_attr, (void *)arg, sizeof(inst->ochn_attr));
	if (rc < 0)
		return rc;

	rc = isp_ochn_attr_check(inst);
	if (rc)
		return rc;

	pr_info("%s fmt=%d\n", __func__, inst->ochn_attr.fmt);
	if (inst->ochn_attr.fmt == FRM_FMT_NV12)
		fmt.format = CAM_FMT_NV12;
	else
		return 0;

	fmt.width = inst->attr.crop.w;
	fmt.stride = inst->attr.crop.w;
	fmt.height = inst->attr.crop.h;

	pr_info("%s format=%d,width=%d,stride=%d,height=%d\n", __func__,
		fmt.format, fmt.width, fmt.stride, fmt.height);
	return isp_set_oformat(&inst->dev->isp_dev, vctx->ctx_id, &fmt);
}

static s32 isp_video_get_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	int rc = 0;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_to_user((void __user *)arg, &inst->ichn_attr, sizeof(inst->ichn_attr));
	if (rc) {
		pr_err("%s: copy_to_user failed!\n", __func__);
		return rc;
	}

	pr_info("%s done\n", __func__);
	return rc;
}

static s32 isp_video_get_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	struct isp_nat_instance *inst;
	int rc = 0;

	inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);

	rc = copy_to_user((void __user *)arg, &inst->ochn_attr, sizeof(inst->ochn_attr));
	if (rc) {
		pr_err("%s: copy_to_user failed!\n", __func__);
		return rc;
	}

	pr_info("%s done\n", __func__);
	return rc;
}

static s32 isp_video_set_inter_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	int rc;
	struct cam_input in;
	struct isp_nat_instance *inst;
    struct sensor_isp_ops_s *cops;
    struct isi_sensor_base_info_s sensor_base_param;

    inst = container_of(vctx->vdev, struct isp_nat_instance, vdev);
	cops = (struct sensor_isp_ops_s *)inst->dev->sensor_ops->cops;
	if (cops && cops->sensor_get_base_info) {
		rc = cops->sensor_get_base_info(vctx->flow_id, &sensor_base_param);
		if (rc < 0) {
			pr_err("get sensor base info param fail\n");
			return rc;
		}
    }

    pr_info("%s flow_id = %d, sensor_name = %s \n", __func__, vctx->flow_id, sensor_base_param.sensor_name);

	in.type = CAM_INPUT_SENSOR;
	in.index = vctx->flow_id;
	snprintf(in.sens.name, sizeof(in.sens.name), "%s", sensor_base_param.sensor_name);
	rc = isp_set_input(&inst->dev->isp_dev, vctx->ctx_id, &in);

	pr_info("ctx_id %d, inst id %d, sensor %s\n", vctx->ctx_id, inst->id, in.sens.name);

	return rc;
}

static struct vio_common_ops isp_vops = {
	.open = isp_nat_open,
	.close = isp_nat_close,
	.video_get_version = isp_video_get_version,
	.video_s_ctrl = isp_video_s_ctrl,
	.video_g_ctrl = isp_video_g_ctrl,
	.video_set_attr = isp_video_set_cfg,
	.video_get_attr = isp_video_get_cfg,
	.video_set_attr_ex = isp_video_set_cfg_ex,
	.video_get_attr_ex = isp_video_get_cfg_ex,
	.video_get_buf_attr = isp_video_reqbufs,
	.video_set_obuf = NULL,
	.video_set_ibuf = NULL,
	.video_start = isp_video_streamon,
	.video_stop = isp_video_streamoff,
	.video_set_ichn_attr = isp_video_set_ichn_attr,
	.video_set_ochn_attr = isp_video_set_ochn_attr,
	.video_get_ichn_attr = isp_video_get_ichn_attr,
	.video_get_ochn_attr = isp_video_get_ochn_attr,
	.video_set_inter_attr = isp_video_set_inter_attr,
	.video_get_struct_size = isp_video_get_struct_size,
};

static int32_t empty_common_get_param(uint32_t chn, struct _setting_param_t *user_para)
{
	pr_info("%s: empty function\n", __func__);

	return 0;
}

static int32_t empty_common_get_base_info(uint32_t chn, struct isi_sensor_base_info_s *user_para)
{
        pr_info("%s:empty function\n", __func__);

        return 0;
}

static struct sensor_isi_ops_s empty_sensor_cops = {
	.sensor_get_para = empty_common_get_param,
        .sensor_get_base_info = empty_common_get_base_info,
};

static ssize_t isp_stat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct isp_nat_device *nat_dev;
	struct isp_nat_instance *src_ins, *cap_ins;
	struct vio_node *vnode;
	ssize_t size = PAGE_SIZE, len = 0;
	size_t i;

	nat_dev = (struct isp_nat_device *)dev_get_drvdata(dev);

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		src_ins = &nat_dev->src_instance[i];
		cap_ins = &nat_dev->cap_instance[i];
		vnode = &nat_dev->vnode[i];
		if (!cap_ins->ochn_attr.fmt)
			continue;

		len += snprintf(&buf[len], size - len,
			       "------------------- flow%d info -------------------\n",
			       vnode->flow_id);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "input_mode:%d\nsched_mode:%d\nsensor_mode:%d\n",
			       src_ins->attr.input_mode, src_ins->attr.sched_mode,
			       src_ins->attr.sensor_mode);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "input_width:%d\ninput_height:%d\ninput_format:%d\ninput_bit_width:%d\n",
			       src_ins->ichn_attr.width, src_ins->ichn_attr.height,
			       src_ins->ichn_attr.fmt, src_ins->ichn_attr.bit_width);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "input_crop_x:%d\ninput_crop_y:%d\ninput_crop_w:%d\ninput_crop_h:%d\n",
			       src_ins->attr.crop.x, src_ins->attr.crop.y,
			       src_ins->attr.crop.w, src_ins->attr.crop.h);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "ddr_en:%d\noutput_format:%d\noutput_bit_width:%d\n",
			       cap_ins->ochn_attr.ddr_en, cap_ins->ochn_attr.fmt,
			       cap_ins->ochn_attr.bit_width);
		if (size - len <= 0)
			break;
	}
	return len;
}
static DEVICE_ATTR(stat, 0444, isp_stat_show, NULL);

static int isp_nat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct isp_nat_device *nat_dev;
	char name[64];
	u32 i;
	s32 ret;
	int rc;
	ktime_t start, end;

	start = ktime_get_boottime();
	nat_dev = devm_kzalloc(dev, sizeof(*nat_dev), GFP_KERNEL);
	if (!nat_dev)
		return -ENOMEM;

	rc = isp_probe(pdev, &nat_dev->isp_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call isp_probe (err=%d)\n", rc);
		return rc;
	}

	nat_dev->gtask.no_worker = 1;
	nat_dev->gtask.id = ISP_MODULE;
	mutex_init(&nat_dev->stream_vnodes_lock);

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		nat_dev->vnode[i].flow_id = INVALID_FLOW_ID;
		nat_dev->vnode[i].ctx_id = i;
		nat_dev->vnode[i].id = ISP_MODULE;
		nat_dev->vnode[i].gtask = &nat_dev->gtask;
		nat_dev->vnode[i].allow_bind = isp_allow_bind;
		nat_dev->vnode[i].frame_work = isp_frame_work;
		nat_dev->vnode[i].ich_subdev[ISP_MAIN_FRAME] = &nat_dev->src_instance[i].vdev;
		nat_dev->vnode[i].active_ich = 1 << ISP_MAIN_FRAME;
		nat_dev->vnode[i].och_subdev[ISP_MAIN_FRAME] = &nat_dev->cap_instance[i].vdev;
		nat_dev->vnode[i].active_och = 1 << ISP_MAIN_FRAME;
		nat_dev->src_instance[i].vdev.vnode = &nat_dev->vnode[i];
		nat_dev->src_instance[i].dev = nat_dev;
		nat_dev->src_instance[i].id = i;
		nat_dev->cap_instance[i].vdev.vnode = &nat_dev->vnode[i];
		nat_dev->cap_instance[i].vdev.pingpong_ring = 1;
		nat_dev->cap_instance[i].dev = nat_dev;
		nat_dev->cap_instance[i].id = i;
	}

	nat_dev->vps_dev[SRC_INDEX].vps_ops = &isp_vops;
	nat_dev->vps_dev[SRC_INDEX].ip_dev = nat_dev;
	nat_dev->vps_dev[SRC_INDEX].vnode = nat_dev->vnode;
	// nat_dev->vps_dev[SRC_INDEX].vnode_id = ISP_MODULE;
	nat_dev->vps_dev[SRC_INDEX].max_ctx = VIO_MAX_STREAM;
	nat_dev->vps_dev[SRC_INDEX].iommu_dev = dev;
	nat_dev->vps_dev[SRC_INDEX].vid = VNODE_ID_SRC;
	snprintf(name, sizeof(name), "%s%d_src", ISP_DEV_NAME, nat_dev->isp_dev.id);
	ret = vio_register_device_node(name, &nat_dev->vps_dev[SRC_INDEX]);
	if (ret < 0) {
		dev_err(dev, "failed to call vio_register_device_node(err=%d).\n", ret);
		return -EFAULT;
	}

	nat_dev->vps_dev[CAP_INDEX].vps_ops = &isp_vops;
	nat_dev->vps_dev[CAP_INDEX].ip_dev = nat_dev;
	nat_dev->vps_dev[CAP_INDEX].vnode = nat_dev->vnode;
	// nat_dev->vps_dev[CAP_INDEX].vnode_id = ISP_MODULE;
	nat_dev->vps_dev[CAP_INDEX].max_ctx = VIO_MAX_STREAM;
	nat_dev->vps_dev[CAP_INDEX].iommu_dev = dev;
	nat_dev->vps_dev[CAP_INDEX].vid = VNODE_ID_CAP;
	snprintf(name, sizeof(name), "%s%d_cap", ISP_DEV_NAME, nat_dev->isp_dev.id);
	ret = vio_register_device_node(name, &nat_dev->vps_dev[CAP_INDEX]);
	if (ret < 0) {
		dev_err(dev, "failed to call vio_register_device_node(err=%d).\n", ret);
		vio_unregister_device_node(&nat_dev->vps_dev[SRC_INDEX]);
		return -EFAULT;
	}

	platform_set_drvdata(pdev, nat_dev);
	nat_dev->sensor_ops = vio_get_callback_ops(&empty_sensor_cops, VIN_MODULE, COPS_4);

	ret = device_create_file(dev, &dev_attr_stat);
	if (ret < 0)
		dev_warn(dev, "failed to call device_create_file(err=%d).\n", ret);

#ifdef CONFIG_DEBUG_FS
	isp_debugfs_init(&nat_dev->isp_dev);
#endif

	end = ktime_get_boottime();
	dev_info(dev, "VS ISP driver (native) probed done, time used: %lldus\n", ktime_to_us(ktime_sub(end, start)));
	return 0;
}

static int isp_nat_remove(struct platform_device *pdev)
{
	struct isp_nat_device *nat_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int rc;

	vio_unregister_device_node(&nat_dev->vps_dev[CAP_INDEX]);
	vio_unregister_device_node(&nat_dev->vps_dev[SRC_INDEX]);

	rc = isp_remove(pdev, &nat_dev->isp_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call isp_remove (err=%d)\n", rc);
		return rc;
	}
	mutex_destroy(&nat_dev->stream_vnodes_lock);

	device_remove_file(dev, &dev_attr_stat);
#ifdef CONFIG_DEBUG_FS
	isp_debugfs_remo(&nat_dev->isp_dev);
#endif

	dev_dbg(dev, "VS ISP driver (native) removed\n");
	return 0;
}

static const struct dev_pm_ops isp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isp_system_suspend, isp_system_resume)
};

static const struct of_device_id isp_of_match[] = {
	{ .compatible = ISP_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, isp_of_match);

static struct platform_driver isp_driver = {
	.probe  = isp_nat_probe,
	.remove = isp_nat_remove,
	.driver = {
		.name = ISP_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = isp_of_match,
		.pm = &isp_pm_ops,
	}
};

static int __init isp_init_module(void)
{
	return platform_driver_register(&isp_driver);
}

static void __exit isp_exit_module(void)
{
	platform_driver_unregister(&isp_driver);
}

module_init(isp_init_module);
module_exit(isp_exit_module);

MODULE_DESCRIPTION("VeriSilicon ISP Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VeriSilicon-ISP");
