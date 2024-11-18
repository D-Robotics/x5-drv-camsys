// SPDX-License-Identifier: GPL-2.0-only
#include <media/videobuf2-dma-contig.h>

#include "cam_uapi.h"

#include "utils.h"
#include "isp.h"
#include "isp_drv.h"
#include "sif.h"
#include "sif_drv.h"

#define BCTX_MAGIC         (0x12345678)
#define is_v4l2_buf_ctx(x) ((x) && (x)->magic == BCTX_MAGIC)

int subdev_init(struct subdev_node *n, const char *name, u32 hwid, int inst,
		const struct v4l2_subdev_ops *ops,
		const struct media_entity_operations *mops)
{
	struct v4l2_subdev *sd = &n->sd;
	int rc;

	v4l2_subdev_init(sd, ops);
	if (inst < 0)
		snprintf(sd->name, sizeof(sd->name), "%s%d", name, hwid);
	else
		snprintf(sd->name, sizeof(sd->name), "%s%d-%d", name, hwid, inst);
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, &n->bctx);
	sd->dev = n->dev;

	sd->entity.name = sd->name;
	sd->entity.obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	sd->entity.ops = mops;

	if (n->num_pads && n->pads) {
		rc = media_entity_pads_init(&sd->entity, n->num_pads, n->pads);
		if (rc < 0)
			return rc;
	}

	sd->fwnode = of_fwnode_handle(n->dev->of_node);
	n->bctx.magic = BCTX_MAGIC;
	return 0;
}

void subdev_deinit(struct subdev_node *n)
{
	media_entity_cleanup(&n->sd.entity);
}

int subdev_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		   struct v4l2_subdev_format *fmt)
{
	struct media_entity *ent;
	struct media_pad *pad;
	struct v4l2_subdev *rsd;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			fmt->pad = pad->index;
			pr_debug("%s call %s set_fmt\n", sd->name, rsd->name);
			rc = v4l2_subdev_call(rsd, pad, set_fmt, state, fmt);
			if (rc < 0)
				return rc;
		}
		i++;
	}
	return 0;
}

int subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct media_entity *ent;
	struct media_pad *pad;
	struct v4l2_subdev *rsd;
	struct v4l2_buf_ctx *ctx;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			ctx = v4l2_get_subdevdata(rsd);

			if (enable && is_v4l2_buf_ctx(ctx) && ctx->set_stream) {
				pr_debug("%s call %s set_stream on\n", sd->name, rsd->name);
				ctx->set_stream(ctx, pad->index, 1);
			}
			pr_debug("%s call %s s_stream, enable=%d\n", sd->name, rsd->name, enable);
			rc = v4l2_subdev_call(rsd, video, s_stream, enable);
			if (rc < 0)
				return rc;

			if (!enable && is_v4l2_buf_ctx(ctx) && ctx->set_stream) {
				pr_debug("%s call %s set_stream off\n", sd->name, rsd->name);
				ctx->set_stream(ctx, pad->index, 0);
			}
		}
		i++;
	}
	return 0;
}

int subdev_call_command(struct v4l2_subdev *sd, uint32_t cmd, void *arg)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			rc = v4l2_subdev_call(rsd, core, command, cmd, arg);
			if (rc < 0)
				return rc;
		}
		i++;
	}
	return 0;
}

struct v4l2_subdev *get_remote_src_subdev(struct v4l2_subdev *sd, struct media_pad **rpad)
{
	struct media_entity *ent;
	struct media_pad *pad;
	struct v4l2_subdev *rsd;
	u16 i = 0;

	if (unlikely(!sd || !sd->entity.pads))
		return NULL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
				return NULL;
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			if (rpad)
				*rpad = pad;
			return rsd;
		}
		i++;
	}
	return NULL;
}

int subdev_open(struct v4l2_subdev *sd)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			if (rsd->internal_ops && rsd->internal_ops->open) {
				pr_debug("%s call %s open\n", sd->name, rsd->name);
				rc = rsd->internal_ops->open(rsd, NULL/*subdev_fh*/);
				if (rc < 0)
					return rc;
			}
		}
		i++;
	}
	return 0;
}

int subdev_close(struct v4l2_subdev *sd)
{
	struct media_entity *ent;
	struct v4l2_subdev *rsd;
	struct media_pad *pad;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			if (rsd->internal_ops && rsd->internal_ops->close) {
				pr_debug("%s call %s close\n", sd->name, rsd->name);
				rc = rsd->internal_ops->close(rsd, NULL/*subdev_fh*/);
				if (rc < 0)
					return rc;
			}
		}
		i++;
	}
	return 0;
}

int get_front_info(struct v4l2_subdev *sd, u32 *devid, u32 *insid)
{
	struct media_entity *ent;
	struct media_pad *pad;
	struct v4l2_subdev *rsd;
	struct v4l2_buf_ctx *ctx;
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;
	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad || !is_media_entity_v4l2_subdev(pad->entity)) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			ctx = v4l2_get_subdevdata(rsd);

			if (is_v4l2_buf_ctx(ctx) && ctx->map_info) {
				rc = ctx->map_info(ctx, devid, insid);
				if (rc < 0)
					return rc;
				pr_debug("%s get %s info, devid:%d, insid:%d\n",
					 sd->name, rsd->name, *devid, *insid);

			}
		}
		i++;
	}
	return 0;
}

u32 pixelformat_to_cam_format(u32 format)
{
	switch (format) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return CAM_FMT_RAW8;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return CAM_FMT_RAW10;
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return CAM_FMT_RAW12;
	case V4L2_PIX_FMT_YUYV:
		return CAM_FMT_YUYV;
	case V4L2_PIX_FMT_NV12:
		return CAM_FMT_NV12;
	case V4L2_PIX_FMT_NV16:
		return CAM_FMT_NV16;
	case V4L2_PIX_FMT_RGB32:
		return CAM_FMT_RGB888X;
	default:
		return CAM_FMT_NULL;
	}
}

u32 cam_format_to_pixelformat(u32 format, u32 bayer_format)
{
	switch (format) {
	case CAM_FMT_YUYV:
		return V4L2_PIX_FMT_YUYV;
	case CAM_FMT_NV12:
		return V4L2_PIX_FMT_NV12;
	case CAM_FMT_NV16:
		return V4L2_PIX_FMT_NV16;
	case CAM_FMT_RGB888X:
		return V4L2_PIX_FMT_RGB32;
	default:
		break;
	}
	if (bayer_format == BAYER_FMT_BGGR) {
		switch (format) {
		case CAM_FMT_RAW8:
			return V4L2_PIX_FMT_SBGGR8;
		case CAM_FMT_RAW10:
			return V4L2_PIX_FMT_SBGGR10;
		case CAM_FMT_RAW12:
			return V4L2_PIX_FMT_SBGGR12;
		default:
			break;
		}
	} else if (bayer_format == BAYER_FMT_GBRG) {
		switch (format) {
		case CAM_FMT_RAW8:
			return V4L2_PIX_FMT_SGBRG8;
		case CAM_FMT_RAW10:
			return V4L2_PIX_FMT_SGBRG10;
		case CAM_FMT_RAW12:
			return V4L2_PIX_FMT_SGBRG12;
		default:
			break;
		}
	} else if (bayer_format == BAYER_FMT_RGGB) {
		switch (format) {
		case CAM_FMT_RAW8:
			return V4L2_PIX_FMT_SRGGB8;
		case CAM_FMT_RAW10:
			return V4L2_PIX_FMT_SRGGB10;
		case CAM_FMT_RAW12:
			return V4L2_PIX_FMT_SRGGB12;
		default:
			break;
		}
	} else {
		switch (format) {
		case CAM_FMT_RAW8:
			return V4L2_PIX_FMT_SGRBG8;
		case CAM_FMT_RAW10:
			return V4L2_PIX_FMT_SGRBG10;
		case CAM_FMT_RAW12:
			return V4L2_PIX_FMT_SGRBG12;
		default:
			break;
		}
	}
	return 0;
}

u32 mbus_code_to_cam_format(u32 format)
{
	switch (format) {
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
	case MEDIA_BUS_FMT_YUYV8_1_5X8:
		return CAM_FMT_NV12;
	default:
		return CAM_FMT_NULL;
	}
}

int pixelformat_to_mbus_code(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_SBGGR8:
		return MEDIA_BUS_FMT_SBGGR8_1X8;
	case V4L2_PIX_FMT_SGBRG8:
		return MEDIA_BUS_FMT_SGBRG8_1X8;
	case V4L2_PIX_FMT_SGRBG8:
		return MEDIA_BUS_FMT_SGRBG8_1X8;
	case V4L2_PIX_FMT_SRGGB8:
		return MEDIA_BUS_FMT_SRGGB8_1X8;
	case V4L2_PIX_FMT_SBGGR10:
		return MEDIA_BUS_FMT_SBGGR10_1X10;
	case V4L2_PIX_FMT_SGBRG10:
		return MEDIA_BUS_FMT_SGBRG10_1X10;
	case V4L2_PIX_FMT_SGRBG10:
		return MEDIA_BUS_FMT_SGRBG10_1X10;
	case V4L2_PIX_FMT_SRGGB10:
		return MEDIA_BUS_FMT_SRGGB10_1X10;
	case V4L2_PIX_FMT_SBGGR12:
		return MEDIA_BUS_FMT_SBGGR12_1X12;
	case V4L2_PIX_FMT_SGBRG12:
		return MEDIA_BUS_FMT_SGBRG12_1X12;
	case V4L2_PIX_FMT_SGRBG12:
		return MEDIA_BUS_FMT_SGRBG12_1X12;
	case V4L2_PIX_FMT_SRGGB12:
		return MEDIA_BUS_FMT_SRGGB12_1X12;
	case V4L2_PIX_FMT_NV12:
		return MEDIA_BUS_FMT_YUYV8_1_5X8;
	case V4L2_PIX_FMT_YUYV:
		return MEDIA_BUS_FMT_YUYV8_1X16;
	default:
		return -EINVAL;
	}
}

u32 mbus_code_to_bayer_pattern(u32 code, bool isISI)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		if (isISI)
			return ISI_BPAT_BGGR;
		else
			return BAYER_FMT_BGGR;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		if (isISI)
			return ISI_BPAT_GBRG;
		else
			return BAYER_FMT_GBRG;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		if (isISI)
			return ISI_BPAT_GRBG;
		else
			return BAYER_FMT_GRBG;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		if (isISI)
			return ISI_BPAT_RGGB;
		else
			return BAYER_FMT_RGGB;
	default:
		return -EINVAL;
	}
}

u32 mbus_code_to_pixelformat(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return V4L2_PIX_FMT_SBGGR8;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		return V4L2_PIX_FMT_SGBRG8;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return V4L2_PIX_FMT_SGRBG8;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return V4L2_PIX_FMT_SRGGB8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return V4L2_PIX_FMT_SBGGR10;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return V4L2_PIX_FMT_SGBRG10;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return V4L2_PIX_FMT_SGRBG10;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return V4L2_PIX_FMT_SRGGB10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return V4L2_PIX_FMT_SBGGR12;
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		return V4L2_PIX_FMT_SGBRG12;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return V4L2_PIX_FMT_SGRBG12;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return V4L2_PIX_FMT_SRGGB12;
	case MEDIA_BUS_FMT_YUYV8_1_5X8:
		return V4L2_PIX_FMT_NV12;
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return V4L2_PIX_FMT_YUYV;
	default:
		return -EINVAL;
	}
}

struct v4l2_subdev *isp_device_to_v4l2_subdev(void *data, uint32_t inst)
{
	struct isp_device *isp_dev = (struct isp_device *)data;
	struct isp_v4l_device *isp_v4l_dev =
			container_of(isp_dev, struct isp_v4l_device, isp_dev);
	struct isp_v4l_instance *ins = &isp_v4l_dev->insts[inst];

	return &ins->node.sd;
}

struct v4l2_subdev *sif_device_to_v4l2_subdev(void *data, uint32_t inst)
{
	struct sif_device *sif_dev = (struct sif_device *)data;
	struct sif_v4l_device *sif_v4l_dev =
			container_of(sif_dev, struct sif_v4l_device, sif_dev);
	struct sif_v4l_instance *ins = &sif_v4l_dev->insts[inst];

	return &ins->node.sd;
}
