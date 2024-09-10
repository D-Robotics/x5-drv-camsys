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

}

u8 cam_get_frame_status(void *cam_ctx)
{
	return 0;
}

void cam_dec_frame_status(void *cam_ctx)
{

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

int sif_handle_get_fmt_cap(void *sif_dev, uint32_t inst, void *data)
{
	int rc = 0;
	struct sen_ctrl *ctrl = (struct sen_ctrl *)data;
	struct v4l2_subdev *sd = sif_device_to_v4l2_subdev(sif_dev, inst);
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev_state state;
	uint32_t index, fmt, framesize[2] = {0};

	memcpy(&index, ctrl->ctrl_data, sizeof(index));
	memcpy(&fmt, ctrl->ctrl_data + sizeof(index), sizeof(fmt));
	memset(&state, 0, sizeof(state));
	memset(&fse, 0, sizeof(fse));
	fse.index = index;
	fse.code = cam_format_to_mbus_code(CAM_FMT_RAW8, BAYER_FMT_BGGR);
	rc = sd->ops->pad->enum_frame_size(sd, &state, &fse);
	if (rc < 0)
		return rc;
	framesize[0] = fse.min_width;
	framesize[1] = fse.min_height;
	memcpy(ctrl->ctrl_data, framesize, sizeof(framesize));
	ctrl->size = sizeof(framesize);

	return 0;
}

int isp_handle_get_fmt_cap(void *isp_dev, uint32_t inst, void *data)
{
	int rc = 0;
	struct sen_ctrl *ctrl = (struct sen_ctrl *)data;
	struct v4l2_subdev *sd = isp_device_to_v4l2_subdev(isp_dev, inst);
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev_state state;
	uint32_t index, fmt, framesize[2] = {0};

	memcpy(&index, ctrl->ctrl_data, sizeof(index));
	memcpy(&fmt, ctrl->ctrl_data + sizeof(index), sizeof(fmt));
	memset(&state, 0, sizeof(state));
	memset(&fse, 0, sizeof(fse));
	fse.index = index;
	fse.code = cam_format_to_mbus_code(CAM_FMT_RAW8, BAYER_FMT_BGGR);
	rc = sd->ops->pad->enum_frame_size(sd, &state, &fse);
	if (rc < 0)
		return rc;
	framesize[0] = fse.min_width;
	framesize[1] = fse.min_height;
	memcpy(ctrl->ctrl_data, framesize, sizeof(framesize));
	ctrl->size = sizeof(framesize);

	return 0;
}
