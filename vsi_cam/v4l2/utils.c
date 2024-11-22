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

int subdev_set_fmt(struct v4l2_subdev *sd,
		   struct v4l2_subdev_state *state,
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
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			fmt->pad = pad->index;
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
			if (!pad) {
				i++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			ctx = v4l2_get_subdevdata(rsd);

			if (enable && is_v4l2_buf_ctx(ctx) && ctx->set_stream)
				ctx->set_stream(ctx, pad->index, 1);

			rc = v4l2_subdev_call(rsd, video, s_stream, enable);
			if (rc < 0)
				return rc;

			if (!enable && is_v4l2_buf_ctx(ctx) && ctx->set_stream)
				ctx->set_stream(ctx, pad->index, 0);
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
			if (!pad) {
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

int subdev_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct media_entity *ent;
	struct media_pad *pad;
	struct v4l2_subdev *rsd;
	uint32_t count[2] = {0};
	u16 i = 0;
	int rc;

	if (unlikely(!sd || !sd->entity.pads))
		return -EINVAL;

	ent = &sd->entity;

	while (i < ent->num_pads) {
		if (ent->pads[i].flags & MEDIA_PAD_FL_SINK) {
			count[0] ++;
			pad = media_pad_remote_pad_first(&ent->pads[i]);
			if (!pad) {
				i++;
				count[1] ++;
				continue;
			}
			rsd = media_entity_to_v4l2_subdev(pad->entity);
			fse->pad = pad->index;
			rc = rsd->ops->pad->enum_frame_size(rsd, sd_state, fse);
			if (rc < 0)
				return rc;
		}
		i++;
	}
	if(count[0] == count[1])
		return -1;
	return 0;
}

int subdev_open(struct v4l2_subdev *sd)
{
	struct media_entity *ent;
	struct media_pad *pad;
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
			sd = media_entity_to_v4l2_subdev(pad->entity);
			if (sd->internal_ops && sd->internal_ops->open) {
				rc = sd->internal_ops->open(sd, NULL/*subdev_fh*/);
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
	struct media_pad *pad;
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
			sd = media_entity_to_v4l2_subdev(pad->entity);
			if (sd->internal_ops && sd->internal_ops->close) {
				rc = sd->internal_ops->close(sd, NULL/*subdev_fh*/);
				if (rc < 0)
					return rc;
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
	default:
		return CAM_FMT_NULL;
	}
}

u32 cam_format_to_mbus_code(u32 format, u32 bayer_format)
{
	if (format == CAM_FMT_RAW8) {
		switch (bayer_format) {
		case BAYER_FMT_BGGR:
			return MEDIA_BUS_FMT_SBGGR8_1X8;
		case BAYER_FMT_GBRG:
			return MEDIA_BUS_FMT_SGBRG8_1X8;
		case BAYER_FMT_GRBG:
			return MEDIA_BUS_FMT_SGRBG8_1X8;
		default:
			return MEDIA_BUS_FMT_SRGGB8_1X8;
		}
	} else if (format == CAM_FMT_RAW10) {
		switch (bayer_format) {
		case BAYER_FMT_BGGR:
			return MEDIA_BUS_FMT_SBGGR10_1X10;
		case BAYER_FMT_GBRG:
			return MEDIA_BUS_FMT_SGBRG10_1X10;
		case BAYER_FMT_GRBG:
			return MEDIA_BUS_FMT_SGRBG10_1X10;
		default:
			return MEDIA_BUS_FMT_SRGGB10_1X10;
		}
	} else {
		switch (bayer_format) {
		case BAYER_FMT_BGGR:
			return MEDIA_BUS_FMT_SBGGR12_1X12;
		case BAYER_FMT_GBRG:
			return MEDIA_BUS_FMT_SGBRG12_1X12;
		case BAYER_FMT_GRBG:
			return MEDIA_BUS_FMT_SGRBG12_1X12;
		default:
			return MEDIA_BUS_FMT_SRGGB12_1X12;
		}
	}
}

struct v4l2_subdev *isp_device_to_v4l2_subdev(void *data, uint32_t inst)
{
    struct isp_device *isp_dev = (struct isp_device *)data;
    struct v4l2_subdev *sd;
    struct isp_v4l_device *isp_v4l_dev = container_of(isp_dev, struct isp_v4l_device, isp_dev);
    struct isp_v4l_instance *ins = &isp_v4l_dev->insts[inst];
    sd = &ins->node.sd;
    return sd;
}

struct v4l2_subdev *sif_device_to_v4l2_subdev(void *data, uint32_t inst)
{
    struct sif_device *sif_dev = (struct sif_device *)data;
    struct v4l2_subdev *sd;
    struct sif_v4l_device *sif_v4l_dev = container_of(sif_dev, struct sif_v4l_device, sif_dev);
    struct sif_v4l_instance *ins = &sif_v4l_dev->insts[inst];
    sd = &ins->node.sd;
    return sd;
}
