// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "cam_dev.h"
#include "csi_drv.h"
#include "isp_uapi.h"
#include "v4l2_usr_api.h"

#define sd_to_csi_v4l_instance(s) \
({ \
	struct subdev_node *sn = container_of(s, struct subdev_node, sd); \
	container_of(sn, struct csi_v4l_instance, node); \
})

static long csi_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);

	if (csi->is_idi_mode)
		return csi_idi_ioctl(sd, cmd, arg);

	return -ENOIOCTLCMD;
}

static int csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);
	struct csi_v4l_device *v4l_dev;
	struct v4l2_subdev *rsd;
	int rc = 0;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);
	rsd = &v4l_dev->insts[0].node.sd;

	if (!enable) {
		rc = subdev_set_stream(rsd, enable);
		if (rc < 0)
			return rc;
	}

	if (csi->is_idi_mode)
		return 0;

	if (enable) {
		csi_ipi_start(csi->dev, csi->id);
		rc = subdev_set_stream(rsd, enable);
		if (rc < 0)
			return rc;
	} else {
		csi_ipi_stop(csi->dev, csi->id);
	}
	return rc;
}

static int csi_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int csi_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static void csi_subirq_callback(struct csi_device *csi_dev, u32 sub_val, const struct csi_irq_reg *err_reg, u32 reset_flag)
{
	int i = 0;

	if (reset_flag) {
		dev_dbg(csi_dev->dev, "trigger ipi overflow reset");
		csi_ipi_overflow_reset(csi_dev, err_reg->id - CSI_ERR_IPI1);
	}

	while (sub_val) {
		if (sub_val & 0x01)
			dev_err(csi_dev->dev, "get err %s", err_reg->errstr[i]);
		i++;
		sub_val >>= 1;
	}
}

static u32 mbus_code_to_csi_format(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return CAM_FMT_RAW8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return CAM_FMT_RAW10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return CAM_FMT_RAW12;
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return CAM_FMT_YUYV;
	case MEDIA_BUS_FMT_YUYV8_1_5X8:
		return CAM_FMT_NV12;
	case MEDIA_BUS_FMT_RGB888_1X32_PADHI:
		return CAM_FMT_RGB888X;
	default:
		return CAM_FMT_NULL;
	}
}

static int csi_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);
	struct cam_format f;
	struct csi_ipi_base_cfg ipi_cfg;
	struct csi_v4l_device *v4l_dev;
	struct v4l2_subdev *rsd;
	int rc;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);
	rsd = &v4l_dev->insts[0].node.sd;

	rc = subdev_set_fmt(rsd, state, fmt);
	if (rc < 0)
		return rc;

	if (csi->is_idi_mode)
		return 0;

	f.width = fmt->format.width;
	f.height = fmt->format.height;
	f.format = mbus_code_to_csi_format(fmt->format.code);
	ipi_cfg.id = csi->id;
	ipi_cfg.adv_val = CSI_IPI_ADV_FEAT_EVSELPROG | CSI_IPI_ADV_FEAT_EN_VIDEO
			| CSI_IPI_ADV_FEAT_EN_EBD | CSI_IPI_ADV_FEAT_MODE_LEGACY;
	ipi_cfg.cut_through = true;
	ipi_cfg.mem_auto_flush = true;
	csi_ipi_init(csi->dev, &ipi_cfg, &f);

	csi->dev->subirq_func = csi_subirq_callback;
	csi->dev->irq_done_func = NULL;
	csi_irq_enable(csi->dev);

	return 0;
}

static int csi_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int csi_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int csi_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);
	struct csi_v4l_device *v4l_dev;
	struct v4l2_subdev *rsd;
	int rc = 0;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);
	rsd = &v4l_dev->insts[0].node.sd;

	rc = subdev_enum_frame_size(rsd, state, fse);
	if (rc < 0)
		return rc;
	return 0;
}

static int csi_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static int v4l2_ctrl_sensor_need_off_auto(uint32_t ctrl_id, uint32_t *auto_id, uint32_t *auto_off)
{
	switch (ctrl_id) {
		case V4L2_CID_EXPOSURE:
			*auto_id = V4L2_CID_EXPOSURE_AUTO;
			*auto_off = (uint32_t)V4L2_EXPOSURE_MANUAL;
			return 1;
		case V4L2_CID_ANALOGUE_GAIN:
			*auto_id = V4L2_CID_AUTOGAIN;
			*auto_off = 0;
			return 1;
		case V4L2_CID_RED_BALANCE:
		case V4L2_CID_BLUE_BALANCE:
			*auto_id = V4L2_CID_AUTO_WHITE_BALANCE;
			*auto_off = 0;
			return 1;
		default:
			return 0;
	}
}

static int csi_s_sensor_ext_ctrl(struct v4l2_subdev *sd, void *arg)
{
	return 0;
}

static int csi_g_sensor_ext_ctrl(struct v4l2_subdev *sd, void *arg)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	struct v4l2_ext_control *ext_ctrl;
	struct v4l2_ctrl *vctrl;
	u16 i = 0;
	int rc = 0;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	ext_ctrl = (struct v4l2_ext_control *)arg;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);

			switch(ext_ctrl->id) {
				case V4L2_CID_PIXEL_RATE:
					vctrl = v4l2_ctrl_find(rsd->ctrl_handler, ext_ctrl->id);
					if (!vctrl) {
						pr_debug("%s ctrl 0x%x not found\n", __func__, ext_ctrl->id);
						return -EINVAL;
					}
					ext_ctrl->value64 = v4l2_ctrl_g_ctrl_int64(vctrl);
					break;
				default:
					break;
			}
			return rc;
		}
		i++;
	}
	return rc;
}

static int csi_s_sensor_ctrl(struct v4l2_subdev *sd, void *arg)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	struct sen_ctrl *ctrl;
	struct v4l2_ctrl *vctrl;
	int32_t ctrl_value = 0;
	uint32_t auto_id, auto_off;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			ctrl = (struct sen_ctrl *)arg;
			if (v4l2_ctrl_sensor_need_off_auto(ctrl->ctrl_id, &auto_id, &auto_off)) {
				vctrl = v4l2_ctrl_find(rsd->ctrl_handler, auto_id);
				if (vctrl) {
					if (vctrl->flags & V4L2_CTRL_FLAG_READ_ONLY) {
						pr_debug("%s ctrl 0x%x is read only\n", __func__, ctrl->ctrl_id);
						return -EINVAL;
					}
					rc = v4l2_ctrl_s_ctrl(vctrl, auto_off);
					if (rc < 0)
						return rc;
				}
			}
			vctrl = v4l2_ctrl_find(rsd->ctrl_handler, ctrl->ctrl_id);
			if (!vctrl) {
				pr_debug("%s ctrl 0x%x not found\n", __func__, ctrl->ctrl_id);
				return -EINVAL;
			}
			if (vctrl->flags & V4L2_CTRL_FLAG_READ_ONLY) {
				pr_debug("%s ctrl 0x%x is read only\n", __func__, ctrl->ctrl_id);
				return -EINVAL;
			}
			memcpy(&ctrl_value, ctrl->ctrl_data, sizeof(ctrl_value));
			return v4l2_ctrl_s_ctrl(vctrl, ctrl_value);
		}
		i++;
	}
	return -EINVAL;
}

static int csi_g_sensor_ctrl(struct v4l2_subdev *sd, void *arg)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	struct sen_ctrl *ctrl;
	struct v4l2_ctrl *vctrl;
	u16 i = 0;
	int rc = 0;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			ctrl = (struct sen_ctrl *)arg;
			switch(ctrl->ctrl_id) {
				case V4L2_CID_FPS:
					uint32_t fps = 30;
					memcpy(ctrl->ctrl_data, &fps, sizeof(fps));
					break;
				case V4L2_CID_EXP_RANGE:
					uint32_t exp_range[3];
					exp_range[0] = 1;
					exp_range[1] = 1104;
					exp_range[2] = 1;
					memcpy(ctrl->ctrl_data, &exp_range, sizeof(exp_range));
					break;
				case V4L2_CID_AGAIN_RANGE:
					uint32_t again_range[3];
					again_range[0] = 0;
					again_range[1] = 1023;
					again_range[2] = 16;
					memcpy(ctrl->ctrl_data, &again_range, sizeof(again_range));
					break;
				case V4L2_CID_DGAIN_RANGE:
					uint32_t dgain_range[3];
					dgain_range[0] = 1;
					dgain_range[1] = 1;
					dgain_range[2] = 1;
					memcpy(ctrl->ctrl_data, &dgain_range, sizeof(dgain_range));
					break;
				case V4L2_CID_BAYER_PATTERN:
					uint32_t bayerPattern = 3;
					memcpy(ctrl->ctrl_data, &bayerPattern, sizeof(bayerPattern));
					break;
				case V4L2_CID_SENSOR_NAME:
					char* space_pos = strchr(rsd->name, ' ');
					ctrl->size = space_pos - rsd->name;
					memcpy(ctrl->ctrl_data, rsd->name, ctrl->size);
					break;
				default:
					int32_t ctrl_value = 0;
					vctrl = v4l2_ctrl_find(rsd->ctrl_handler, ctrl->ctrl_id);
					if (!vctrl) {
						pr_debug("%s ctrl 0x%x not found\n", __func__, ctrl->ctrl_id);
						return -EINVAL;
					}
					ctrl_value = v4l2_ctrl_g_ctrl(vctrl);
					memcpy(ctrl->ctrl_data, &ctrl_value, sizeof(ctrl_value));
					break;
			}
			return rc;
		}
		i++;
	}
	return rc;
}

static int csi_query_sensor_ctrl(struct v4l2_subdev *sd, void *arg)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	struct v4l2_queryctrl *qctrl;
	struct v4l2_ctrl *vctrl;
	u16 i = 0;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			qctrl = (struct v4l2_queryctrl *)arg;
			vctrl = v4l2_ctrl_find(rsd->ctrl_handler, qctrl->id);
			if (!vctrl) {
				pr_debug("%s ctrl 0x%x not found\n", __func__, qctrl->id);
				return -EINVAL;
			}
			qctrl->type = vctrl->type;
			memcpy(qctrl->name, vctrl->name, sizeof(qctrl->name));
			qctrl->minimum = vctrl->minimum;
			qctrl->maximum = vctrl->maximum;
			qctrl->step = vctrl->step;
			qctrl->default_value = vctrl->default_value;
			qctrl->flags = vctrl->flags;
			return 0;
		}
		i++;
	}
	return 0;
}

static long csi_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);
	struct csi_v4l_device *v4l_dev;
	struct v4l2_subdev *rsd;
	int rc = 0;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);
	rsd = &v4l_dev->insts[0].node.sd;

	switch(cmd) {
		case CAM_SET_CTRL:
			rc = csi_s_sensor_ctrl(rsd, arg);
			break;
		case CAM_SET_EXT_CTRL:
			rc = csi_s_sensor_ext_ctrl(rsd, arg);
			break;
		case CAM_GET_CTRL:
			rc = csi_g_sensor_ctrl(rsd, arg);
			break;
		case CAM_GET_EXT_CTRL:
			rc = csi_g_sensor_ext_ctrl(rsd, arg);
			break;
		case CAM_QUERY_CTRL:
			rc = csi_query_sensor_ctrl(rsd, arg);
			break;
		default:
			break;
	}

	return rc;
}

static const struct v4l2_subdev_core_ops csi_core_ops = {
	.command = csi_command,
	.ioctl = csi_ioctl,
};

static const struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
	.g_frame_interval = csi_g_frame_interval,
	.s_frame_interval = csi_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops csi_pad_ops = {
	.set_fmt = csi_set_fmt,
	.get_fmt = csi_get_fmt,
	.enum_mbus_code = csi_enum_mbus_code,
	.enum_frame_size = csi_enum_frame_size,
	.enum_frame_interval = csi_enum_frame_interval,
};

static const struct v4l2_subdev_ops csi_subdev_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

static int csi_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct csi_v4l_instance *inst = sd_to_csi_v4l_instance(sd);

	return csi_open(inst->dev, inst->id);
}

static int csi_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct csi_v4l_instance *inst = sd_to_csi_v4l_instance(sd);

	return csi_close(inst->dev, inst->id);
}

static const struct v4l2_subdev_internal_ops csi_internal_ops = {
	.open = csi_v4l_open,
	.close = csi_v4l_close,
};

static void csi_inst_remove(struct csi_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int config_csi(struct csi_v4l_instance *csi)
{
	struct csi_device *csi_dev;
	struct v4l2_subdev *sd;
	struct csi_vc_cfg vc_cfg;
	u32 lane_rate, num_lanes;
	struct device *dev;
	struct fwnode_handle *csi_fwnode, *endpoint;
	int rc = 0;

	vc_cfg.vc_mode   = CSI_VC_CAMERA;
	vc_cfg.vc_ebd_en = 1;
	sd = &csi->node.sd;
	csi_dev = csi->dev;

	dev = sd->dev;
	if (!dev) {
		pr_err("No device found for csi subdev\n");
		return -EINVAL;
	}

	csi_fwnode = dev_fwnode(dev);
	if (!csi_fwnode) {
		dev_err(dev, "No fwnode found for csi device\n");
		return -EINVAL;
	}

	endpoint = fwnode_graph_get_next_endpoint(csi_fwnode, NULL);
	if (!endpoint) {
		dev_err(dev, "No endpoint found for csi\n");
		return -EINVAL;
	}

	rc = fwnode_property_read_u32(endpoint, "vc_id", &vc_cfg.vc_id);
	if (rc) {
		dev_err(dev, "Failed to read vc_id: %d\n", rc);
		goto _exit;
	}

	rc = fwnode_property_read_u32(endpoint, "lane-rate", &lane_rate);
	if (rc) {
		dev_err(dev, "Failed to read lane-rate: %d\n", rc);
		goto _exit;
	}

	num_lanes = fwnode_property_count_u32(endpoint, "data-lanes");
	if (num_lanes <= 0) {
		dev_err(dev, "Invalid number of data lanes: %d\n", num_lanes);
		rc = -EINVAL;
		goto _exit;
	}

	rc = csi_ipi_set_vc_cfg(csi_dev, csi->id, &vc_cfg);
	if (rc) {
		dev_err(dev, "csi_ipi_set_vc_cfg failed: %d\n", rc);
		goto _exit;
	}

	csi_set_lanes(csi_dev, num_lanes, 0);

	rc = csi_set_lane_rate(csi_dev, lane_rate);
	if (rc) {
		dev_err(dev, "csi_set_lane_rate failed: %d\n", rc);
		goto _exit;
	}

_exit:
	fwnode_handle_put(endpoint);
	return rc;
}

static int csi_async_bound(struct subdev_node *sn)
{
	struct csi_v4l_instance *csi =
			container_of(sn, struct csi_v4l_instance, node);
	struct csi_v4l_instance *ins;
	struct csi_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);

	while (i < csi->dev->num_insts) {
		ins = &v4l_dev->insts[i];

		rc = config_csi(ins);
		if (rc < 0)
			return rc;

		if (ins != csi) {
			rc = v4l2_device_register_subdev
					(sn->sd.v4l2_dev, &ins->node.sd);
			if (rc < 0)
				goto _err;
		}
		i++;
	}
	return 0;

_err:
	while (j < i) {
		ins = &v4l_dev->insts[j];

		if (ins != csi)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int csi_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csi_v4l_device *v4l_dev;
	struct csi_v4l_instance *insts;
	u32 i;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = csi_probe(pdev, &v4l_dev->csi_dev, false);
	if (rc < 0) {
		dev_err(dev, "failed to call csi_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->csi_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->csi_dev.num_insts; i++) {
		struct csi_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->csi_dev;

		n->async_bound = csi_async_bound;

		n->dev = dev;
		n->num_pads = 2;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			csi_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;

		rc = subdev_init(n, CSI_DEV_NAME, v4l_dev->csi_dev.id,
				 i, &csi_subdev_ops, NULL);
		if (rc < 0) {
			csi_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &csi_internal_ops;

		/* the last one is for idi-control */
		if (i == v4l_dev->csi_dev.num_insts - 1) {
			inst->is_idi_mode = true;
			n->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		}
	}
	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		csi_inst_remove(insts, v4l_dev->csi_dev.num_insts);
		csi_remove(pdev, &v4l_dev->csi_dev);
		return rc;
	}

	platform_set_drvdata(pdev, v4l_dev);

	rc = csi_runtime_resume(dev);
	if (rc) {
		dev_err(dev, "failed to call csi_runtime_resume (err=%d)\n", rc);
		return rc;
	}

	if (v4l_dev->csi_dev.ipi)
		dev_dbg(dev, "ipi clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.ipi));
	if (v4l_dev->csi_dev.pclk)
		dev_dbg(dev, "pclk clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.pclk));
	if (v4l_dev->csi_dev.cfg)
		dev_dbg(dev, "cfg clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.cfg));

	dev_dbg(dev, "VS CSI driver (v4l) probed done\n");
	return 0;
}

static int csi_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csi_v4l_device *v4l_dev = platform_get_drvdata(pdev);
	int rc;
	u32 i;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->csi_dev.num_insts; i++) {
		subdev_deinit(&v4l_dev->insts[i].node);
		devm_kfree(dev, v4l_dev->insts[i].node.pads);
	}
	devm_kfree(dev, v4l_dev->insts);

	rc = csi_remove(pdev, &v4l_dev->csi_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call csi_remove (err=%d)\n", rc);
		return rc;
	}

	rc = csi_runtime_suspend(dev);
	if (rc) {
		dev_err(dev, "failed to call csi_runtime_suspend (err=%d)\n", rc);
		return rc;
	}
	devm_kfree(dev, v4l_dev);

	dev_dbg(dev, "VS CSI driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(csi_system_suspend, csi_system_resume)
};

static const struct of_device_id csi_of_match[] = {
	{ .compatible = CSI_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, csi_of_match);

static struct platform_driver csi_driver = {
	.probe  = csi_v4l_probe,
	.remove = csi_v4l_remove,
	.driver = {
		.name = CSI_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = csi_of_match,
		.pm = &csi_pm_ops,
	}
};

static int __init csi_init_module(void)
{
	return platform_driver_register(&csi_driver);
}

static void __exit csi_exit_module(void)
{
	platform_driver_unregister(&csi_driver);
}

module_init(csi_init_module);
module_exit(csi_exit_module);

MODULE_DESCRIPTION("SNPS MIPI CSI Controller Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("SNPS-CSI");
