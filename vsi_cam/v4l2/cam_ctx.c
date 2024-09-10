// SPDX-License-Identifier: GPL-2.0-only
#include <media/videobuf2-dma-contig.h>

#include "utils.h"
#include "cam_buf.h"

#include "cam_ctx.h"
#include "isp_uapi.h"

extern struct v4l2_subdev *isp_device_to_v4l2_subdev(void *isp_dev, uint32_t inst);
extern struct v4l2_subdev *sif_device_to_v4l2_subdev(void *sif_dev, uint32_t inst);

int cam_trigger(struct cam_ctx *ctx)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return -EINVAL;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return -EINVAL;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		return -EINVAL;
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx)
		return -EINVAL;

	vctx->trigger(vctx);
	return 0;
}

bool cam_is_completed(struct cam_ctx *ctx)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	struct v4l2_buf_ctx *vctx = NULL;

	if (!ctx || !ctx->pad)
		return true;

	pad = media_pad_remote_pad_first(ctx->pad);
	if (!pad)
		return true;

	if (is_media_entity_v4l2_video_device(pad->entity)) {
		return true;
	} else if (is_media_entity_v4l2_subdev(pad->entity)) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd)
			vctx = (struct v4l2_buf_ctx *)v4l2_get_subdevdata(sd);
	}

	if (!vctx)
		return true;

	return vctx->is_completed(vctx);
}

int cam_ctx_init(struct cam_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf)
{
	return cam_buf_ctx_init(ctx, dev, data, has_internal_buf);
}

void cam_ctx_release(struct cam_ctx *ctx)
{
	cam_buf_ctx_release(ctx);
}

void sif_set_frame_des(struct cam_ctx *ctx, void *data)
{
}

void sif_get_frame_des(struct cam_ctx *ctx)
{
}

void isp_update_frame_info(void *data,struct cam_ctx *ctx)
{
}

void cam_set_stat_info(struct cam_ctx *ctx, u32 type)
{
}

bool cam_osd_update(struct cam_ctx *ctx)
{
	return false;
}

int cam_osd_set_cfg(struct cam_ctx *ctx, u32 ochn_id)
{
	return 0;
}

int cam_read_hist(struct cam_ctx *ctx, u32 ochn_id)
{
	return 0;
}

int cam_set_mode(struct cam_ctx *ctx, u32 mode)
{
	return 0;
}

void cam_set_frame_status(void *cam_ctx, enum cam_frame_status status)
{
	struct cam_ctx *ctx = (struct cam_ctx *)cam_ctx;

	if (ctx)
		ctx->status = status;
}

u8 cam_get_frame_status(void *cam_ctx)
{
	struct cam_ctx *ctx = (struct cam_ctx *)cam_ctx;

	if (ctx)
		return ctx->status;
	return 0;
}

void cam_dec_frame_status(void *cam_ctx)
{
	struct cam_ctx *ctx = (struct cam_ctx *)cam_ctx;

	if (ctx && ctx->status)
		ctx->status--;
}

void isp_handle_set_sensor_ctrl(void *isp_dev, uint32_t inst, void *data)
{
	struct sen_ctrl *ctrl = (struct sen_ctrl *)data;
	struct v4l2_subdev *sd = isp_device_to_v4l2_subdev(isp_dev, inst);

	uint32_t cmd = CAM_SET_SENSOR_CTRL;
	v4l2_subdev_call(sd, core, command, cmd, (void *)ctrl);
}

void isp_handle_get_sensor_ctrl(void *isp_dev, uint32_t inst, void *data)
{
	struct sen_ctrl *ctrl = (struct sen_ctrl *)data;
	struct v4l2_subdev *sd = isp_device_to_v4l2_subdev(isp_dev, inst);

	uint32_t cmd = CAM_GET_SENSOR_CTRL;
	v4l2_subdev_call(sd, core, command, cmd, (void *)ctrl);
}

int cam_get_frame_info(struct cam_ctx *ctx, struct cam_frame_info *info)
{
	return 0;
}

int cam_update_frame_info(struct cam_ctx *ctx, struct cam_frame_info *info)
{
	return 0;
}