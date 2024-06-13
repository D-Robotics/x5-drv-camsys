// SPDX-License-Identifier: GPL-2.0-only
#include <media/videobuf2-dma-contig.h>

#include "utils.h"
#include "cam_buf.h"

#include "cam_ctx.h"

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

int cam_next_set_mode(struct cam_ctx *ctx, u32 mode)
{
	return 0;
}
