// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[vse_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"

#include "vse_drv.h"
#include "v4l2_usr_api.h"

#define VSE_SINK_ONLINE_PATH_MAX (4)

static int scene;
module_param(scene, int, 0644);
#if 0
static struct vse_stitching stitchesx[] = {
	{ false },
	{ false },
	{ false },
	{ false },
	{ false },
	{ false },
};

static struct vse_stitching *stitches[] = {
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
};
#endif

static struct cam_rect cropsx[] = {
	{}, {}, {}, {}, {}, {},
};

static struct cam_rect *crops[] = {
	cropsx,
	cropsx,
	cropsx,
	cropsx,
	cropsx,
};

static struct ires {
	u16 width, height;
} iress[] = {
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
};

#define sd_to_vse_v4l_instance(s) \
({ \
	struct subdev_node *sn = container_of(s, struct subdev_node, sd); \
	container_of(sn, struct vse_v4l_instance, node); \
})

#define buf_ctx_to_vse_v4l_instance(ctx) \
({ \
	struct subdev_node *sn = container_of(ctx, struct subdev_node, bctx); \
	container_of(sn, struct vse_v4l_instance, node); \
})

static int get_channel_index(struct vse_v4l_instance *vse,
			     const struct media_pad *pad)
{
	int i;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++)
		if (vse->src_pads[i] == pad)
			return i;
	return -1;
}

static int vse_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct vse_v4l_instance *vse;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	int rc = 0;
	int index = -1;
	bool has_internal_buf = false;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	vse = sd_to_vse_v4l_instance(sd);

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &vse->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SOURCE) {
			lctx->is_src_online_mode = rctx->is_sink_online_mode;
		} else {
			if (lctx->is_sink_online_mode != rctx->is_src_online_mode)
				return -EBUSY;
			has_internal_buf = !lctx->is_sink_online_mode;
		}
	}

	if (local->flags & MEDIA_PAD_FL_SINK) {
		buf_ctx = &vse->sink_ctx;
	} else {
		index = get_channel_index(vse, local);
		if (index < 0)
			return -EINVAL;
		buf_ctx = &vse->src_ctx[index];
	}

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (buf_ctx->pad)
			return -EBUSY;

		rc = cam_ctx_init(buf_ctx, sd->dev, (void *)local,
				      has_internal_buf);
		if (rc < 0)
			return rc;
		if (index >= 0)
			vse->is_out_chnl_connected[index] = true;
	} else {
		cam_ctx_release(buf_ctx);
		if (index >= 0)
			vse->is_out_chnl_connected[index] = false;
	}
	return rc;
}

static const struct media_entity_operations vse_media_ops = {
	.link_setup = vse_link_setup,
};

static void vse_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	if (ctx && on)
		vse_wake_up(vse->dev, vse->id);
}

static int vse_qbuf(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);
	int rc;

	if (!ctx || !buf)
		return -EINVAL;

	if (ctx->is_sink_online_mode)
		return -EBUSY;

	rc = cam_qbuf(&vse->sink_ctx, buf);
	if (rc < 0)
		return rc;

	return vse_add_job(vse->dev, vse->id);
}

static int vse_drop(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	if (!ctx || !buf)
		return -EINVAL;

	if (ctx->is_sink_online_mode)
		return -EBUSY;

	return cam_drop(&vse->sink_ctx, buf);
}

static struct cam_buf *vse_dqbuf(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	if (!ctx)
		return NULL;

	if (ctx->is_sink_online_mode)
		return NULL;

	return cam_dqbuf(&vse->sink_ctx);
}

static void vse_trigger(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	if (ctx)
		vse_add_job(vse->dev, vse->id);
}

static bool vse_is_completed(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);
	bool rc = false;

	if (ctx)
		rc = vse->dev->is_completed;
	return rc;
}

static u32 vse_get_out_format(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	return vse->out_pixelformat;
}

static int vse_set_remote_format(struct v4l2_subdev *sd, u32 pixelformat, bool is_try)
{
	struct media_entity *ent;
	struct media_pad *pad;
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
			sd = media_entity_to_v4l2_subdev(pad->entity);
			ctx = v4l2_get_subdevdata(sd);
			if (ctx && ctx->set_format) {
				rc = ctx->set_format(ctx, pixelformat, is_try);
				if (rc < 0)
					return rc;
			}
		}
		i++;
	}
	return 0;
}

static int vse_set_out_format(struct v4l2_buf_ctx *ctx, u32 format, bool is_try)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);

	vse_set_remote_format(&vse->node.sd, format, is_try);

	if (!is_try)
		vse->out_pixelformat = format;
	return 0;
}

static int vse_enum_out_format(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct vse_instance *vse;

	if (!format)
		return -EINVAL;

	vse = &inst->dev->insts[inst->id];

	if (index >= ARRAY_SIZE(vse->fmt_cap))
		return -EINVAL;

	*format = cam_format_to_pixelformat(vse->fmt_cap[index].format, 0);
	return *format ? 0 : -EINVAL;
}

static int vse_enum_out_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct vse_instance *vse;
	struct vse_format_cap *cap = NULL;
	struct cam_res_cap *res;
	int channel = -1;
	int hfactor = 1, vfactor = 1;
	u32 i, format;

	if (pad < inst->node.num_pads)
		channel = get_channel_index(inst, &inst->node.pads[pad]);

	if (channel < 0)
		return -EINVAL;

	vse = &inst->dev->insts[inst->id];

	for (i = 0; i < ARRAY_SIZE(vse->fmt_cap); i++) {
		format = cam_format_to_pixelformat(vse->fmt_cap[i].format, 0);
		if (format == fsize->pixel_format) {
			cap = &vse->fmt_cap[i];
			break;
		}
	}

	if (!cap)
		return -EINVAL;

	if (fsize->index >= ARRAY_SIZE(cap->res[channel]))
		return -EINVAL;

	if (channel < 3 || channel == 5) {
		hfactor = 2;
		vfactor = 2;
	}

	res = &cap->res[channel][fsize->index];
	if (res->type == CAP_DC) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = res->dc.width * hfactor;
		fsize->discrete.height = res->dc.height * vfactor;
	} else if (res->type == CAP_SW) {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = res->sw.min_width * hfactor;
		fsize->stepwise.max_width = res->sw.max_width * hfactor;
		fsize->stepwise.min_height = res->sw.min_height * vfactor;
		fsize->stepwise.max_height = res->sw.max_height * vfactor;
		fsize->stepwise.step_width = res->sw.step_width * hfactor;
		fsize->stepwise.step_height = res->sw.step_height * vfactor;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int vse_enum_out_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct vse_v4l_instance *ins = buf_ctx_to_vse_v4l_instance(ctx);
	struct vse_instance *vse;
	struct vse_format_cap *cap = NULL;
	struct cam_res_cap *res;
	bool found = false;
	u32 i, format;

	if (fival->index > 0)
		return -EINVAL;

	vse = &ins->dev->insts[ins->id];

	for (i = 0; i < ARRAY_SIZE(vse->fmt_cap); i++) {
		format = cam_format_to_pixelformat(vse->fmt_cap[i].format, 0);
		if (format == fival->pixel_format) {
			cap = &vse->fmt_cap[i];
			break;
		}
	}

	if (!cap)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(cap->res[pad]); i++) {
		res = &cap->res[pad][i];
		if (res->type == CAP_SW) {
			int diff;

			if (fival->width > res->sw.max_width)
				continue;
			if (fival->height > res->sw.max_height)
				continue;
			diff = fival->width - res->sw.min_width;
			if (diff % res->sw.step_width)
				continue;
			diff = fival->height - res->sw.min_height;
			if (diff % res->sw.step_height)
				continue;
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	fival->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fival->discrete.numerator = ins->out_fps[pad].numerator;
	fival->discrete.denominator = ins->out_fps[pad].denominator;
	return 0;
}

static void vse_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct vse_instance *ins;
	struct vse_format_cap *cap;
	uint32_t support_fmt = CAM_FMT_NV12;
	int i;

	ins = &inst->dev->insts[inst->id];

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));
	cap = &ins->fmt_cap[0];
	cap->format = support_fmt;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		cap->res[i][0].type           = CAP_SW;
		cap->res[i][0].sw.min_width   = 64;
		cap->res[i][0].sw.min_height  = 64;
		cap->res[i][0].sw.step_width  = 2;
		cap->res[i][0].sw.step_height = 2;
	}

	cap->res[0][0].sw.max_width  = 4096;
	cap->res[0][0].sw.max_height = 3076;
	cap->res[1][0].sw.max_width  = 1920;
	cap->res[1][0].sw.max_height = 1080;
	cap->res[2][0].sw.max_width  = 1920;
	cap->res[2][0].sw.max_height = 1080;
	cap->res[3][0].sw.max_width  = 1280;
	cap->res[3][0].sw.max_height = 720;
	cap->res[4][0].sw.max_width  = 1280;
	cap->res[4][0].sw.max_height = 720;
	cap->res[5][0].sw.max_width  = 4096;
	cap->res[5][0].sw.max_height = 3076;
}

static void fill_irq_ctx(struct vse_v4l_instance *vse, struct vse_irq_ctx *ctx)
{
	u32 i;

	memset(ctx, 0, sizeof(*ctx));
	ctx->is_sink_online_mode = vse->node.bctx.is_sink_online_mode;
	if (vse->sink_ctx.pad)
		ctx->sink_ctx = &vse->sink_ctx;
	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (vse->src_ctx[i].pad)
			ctx->src_ctx[i] = &vse->src_ctx[i];
		// ctx->stitches[i] = stitches[scene][i];
	}
}

static int vse_queue_setup(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct vse_v4l_instance *ins = container_of(ctx, struct vse_v4l_instance, sink_ctx);
	struct vse_instance *vse;
	unsigned int size = 0;

	if (ins) {
		vse = &ins->dev->insts[ins->id];
		size = get_framebuf_size(&vse->ifmt);
	}

	if (!size)
		return -ENOMEM;

	if (!*num_buffers)
		*num_buffers = 1;

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static struct cam_buf_ops vse_buf_ops = {
	.queue_setup = vse_queue_setup,
};

static int vse_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vse_v4l_instance *vse = sd_to_vse_v4l_instance(sd);
	struct vse_irq_ctx ctx;
	int rc;

	if (enable) {
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL) {
			refcount_inc(&vse->state_count);
			return 0;
		}

		refcount_inc(&vse->state_count);

		if (vse->id < VSE_SINK_ONLINE_PATH_MAX)
			rc = vse_set_source(vse->dev, vse->id, VSE_SRC_STRM0);
		else
			rc = vse_set_source(vse->dev, vse->id, VSE_SRC_RDMA);
		if (rc < 0) {
			pr_err("%s failed to call vse_set_source (rc=%d)!\n", __func__, rc);
			return rc;
		}

		if (!vse->node.bctx.is_sink_online_mode)
			cam_reqbufs(&vse->sink_ctx, 4, &vse_buf_ops);

		fill_irq_ctx(vse, &ctx);
		vse_set_ctx(vse->dev, vse->id, &ctx);

		rc = vse_set_state(vse->dev, vse->id, enable);
		if (rc < 0)
			return rc;

		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	} else {
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL)
			refcount_dec(&vse->state_count);
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL)
			return 0;

		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;

		memset(&ctx, 0, sizeof(ctx));
		vse_set_ctx(vse->dev, vse->id, &ctx);
		if (!vse->node.bctx.is_sink_online_mode)
			cam_reqbufs(&vse->sink_ctx, 0, NULL);

		rc = vse_set_state(vse->dev, vse->id, enable);
		if (rc < 0)
			return rc;
		vse->fmt_changed = false;
	}
	return 0;
}

static int vse_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct vse_v4l_instance *ins = sd_to_vse_v4l_instance(sd);

	if (fiv->pad >= VSE_OUT_CHNL_MAX)
		return -EINVAL;
	fiv->interval = ins->out_fps[fiv->pad];
	return 0;
}

static int vse_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct vse_v4l_instance *ins = sd_to_vse_v4l_instance(sd);

	if (fiv->pad >= VSE_OUT_CHNL_MAX)
		return -EINVAL;
	fiv->interval = ins->out_fps[fiv->pad];
	return 0;
}

static uint32_t vse_get_sensor_fps(struct v4l2_subdev *sd)
{
	struct sen_ctrl sctrl = {0};
	uint32_t fps;
	sctrl.ctrl_id = V4L2_CID_FPS;
	subdev_call_command(sd, CAM_GET_CTRL, &sctrl);
	memcpy(&fps, &sctrl.ctrl_data, sizeof(fps));
	return fps;
}

static int vse_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	struct cam_format f;
	// struct vse_stitching *stitch;
	struct v4l2_subdev_format s_f = *fmt;
	struct cam_rect crop;
	struct vse_fps_rate fps;
	int channel = -1;
	int hfactor = 1, vfactor = 1;
	int rc;

	if (fmt->pad < inst->node.num_pads)
		channel = get_channel_index(inst, &inst->node.pads[fmt->pad]);

	if (channel < 0)
		return -EINVAL;
#if 0
	stitch = stitches[scene];
	if (stitch[channel].enabled) {
		hfactor = stitch[channel].hfactor;
		vfactor = stitch[channel].vfactor;
	}
#endif
	memset(&f, 0, sizeof(f));
	f.format = CAM_FMT_NV12;
	f.width = iress[scene].width;
	f.height = iress[scene].height;
	f.stride = ALIGN(f.width, STRIDE_ALIGN);
	if (!inst->fmt_changed || memcmp(&f, &inst->ifmt, sizeof(f))) {
		struct vse_msg msg;

		s_f.format.width = f.width;
		s_f.format.height = f.height;
		rc = subdev_set_fmt(sd, state, &s_f);
		if (rc < 0)
			return rc;

		msg.id = CAM_MSG_STATE_CHANGED;
		msg.inst = inst->id;
		msg.state = CAM_STATE_INITED;
		pr_info("%s set vse state to INITED\n", __func__);
		vse_post(inst->dev, &msg, true);

		rc = vse_set_iformat(inst->dev, inst->id, &f);
		if (rc < 0)
			return rc;
		memcpy(&inst->ifmt, &f, sizeof(f));
		inst->fmt_changed = true;
	}

	if (inst->node.bctx.is_sink_online_mode)
		vse_set_cascade(inst->dev, inst->id, inst->id, true); //FIXME
	else
		vse_set_cascade(inst->dev, inst->id, inst->id, false);

	f.width = ALIGN_DOWN(fmt->format.width / hfactor, 16);
	f.height = fmt->format.height / vfactor;
	f.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	crop = crops[scene][channel];
	if (inst->out_pixelformat)
		f.format = pixelformat_to_cam_format(inst->out_pixelformat);
	else
		f.format = mbus_code_to_cam_format(fmt->format.code);
	rc = vse_set_oformat(inst->dev, inst->id, channel, &f, &crop, true);
	if (rc < 0)
		return rc;

	fps.src = vse_get_sensor_fps(sd);
	fps.dst = fps.src;
	rc = vse_set_fps_rate(inst->dev, inst->id, channel, &fps);
	if (rc < 0) {
		pr_err("vse_set_fps_dst_rate failed");
		return rc;
	}
#if 0
	if (!stitch[channel].enabled)
		return 0;

	if (WARN_ON(channel != stitch[channel].left_top))
		return 0;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (stitch[i].enabled && i != channel) {
			crop = crops[scene][i];
			rc = vse_set_oformat(inst->dev, inst->id, i, &f, &crop, true);
			if (rc < 0)
				return rc;
		}
	}
#endif
	return 0;
}

static int vse_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int vse_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int vse_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	return 0;
}

static int vse_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static void vse_get_cur_attr(struct vse_instance *ins, int chnl, vse_ochn_attr_ex_t *vse_attr)
{
	vse_attr->src_fps = ins->fps[chnl].src;
	vse_attr->dst_fps = ins->fps[chnl].dst;
	if(ins->fps[chnl].dst == 0)
		vse_attr->chn_en = 0;
	else
		vse_attr->chn_en = 1;
	vse_attr->roi.x = ins->crop[chnl].x;
	vse_attr->roi.y = ins->crop[chnl].y;
	vse_attr->roi.w = ins->crop[chnl].w;
	vse_attr->roi.h = ins->crop[chnl].h;
	vse_attr->target_w = ins->ofmt[chnl].width;
	vse_attr->target_h = ins->ofmt[chnl].height;
}


static int vse_s_attr(struct vse_v4l_instance *inst, void *arg)
{
	struct vse_instance *ins;
	vse_ochn_attr_ex_t vse_attr;
	struct cam_v4l2_ext_control *cam_ext_ctrl;
	struct cam_format f;
	struct cam_rect crop;
	struct vse_fps_rate fps;
	int hfactor = 1, vfactor = 1;
	int chnl, rc;

	ins = &inst->dev->insts[inst->id];
	cam_ext_ctrl = (struct cam_v4l2_ext_control *)arg;

	if (cam_ext_ctrl->pad < inst->node.num_pads)
		chnl = get_channel_index(inst, &inst->node.pads[cam_ext_ctrl->pad]);

	rc = copy_from_user(&vse_attr, cam_ext_ctrl->controls->ptr, sizeof(vse_attr));
	if (rc < 0)
		return rc;

	f.format = ins->ofmt[chnl].format;
	f.width = ALIGN_DOWN(vse_attr.target_w / hfactor, 16);
	f.height = vse_attr.target_h / vfactor;
	f.stride = ALIGN(vse_attr.target_w, STRIDE_ALIGN);
	crop.x = vse_attr.roi.x;
	crop.y = vse_attr.roi.y;
	crop.w = vse_attr.roi.w;
	crop.h = vse_attr.roi.h;

	rc = vse_set_oformat(inst->dev, inst->id, chnl, &f, &crop, vse_attr.chn_en);
	if (rc < 0) {
		pr_err("%s vse set oformat fail\n", __func__);
		return rc;
	}

	fps.src = vse_attr.src_fps;
	fps.dst = vse_attr.dst_fps;
	rc = vse_set_fps_rate(inst->dev, inst->id, chnl, &fps);
	if (rc < 0) {
		pr_err("%s vse set fps rate fail\n", __func__);
		return rc;
	}
	return 0;
}

static int vse_g_attr(struct vse_v4l_instance *inst, void *arg)
{
	struct vse_instance *ins;
	struct cam_v4l2_ext_control *cam_ext_ctrl;
	vse_ochn_attr_ex_t vse_attr = {0};
	int chnl, rc;

	ins = &inst->dev->insts[inst->id];
	cam_ext_ctrl = (struct cam_v4l2_ext_control *)arg;

	if (cam_ext_ctrl->pad < inst->node.num_pads)
		chnl = get_channel_index(inst, &inst->node.pads[cam_ext_ctrl->pad]);

	if (chnl < 0)
		return -EINVAL;

	vse_get_cur_attr(ins, chnl, &vse_attr);

	rc = copy_to_user(cam_ext_ctrl->controls->ptr,  &vse_attr, sizeof(vse_ochn_attr_ex_t));
	if (rc) {
		pr_err("%s: ctrl_data copy_to_user failed!\n", __func__);
		return rc;
	}

	return 0;
}

static int get_name_for_ext_ctrl(uint32_t id, char *name)
{
	const char *source;

	switch (id) {
		case V4L2_CID_DR_VSE_ATTR:
			source = "vse_ochn_attr_ex_t";
			break;
		default:
			return -1;
	}
	memcpy(name, source, strlen(source)+1);
	return 0;
}

static long vse_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct vse_v4l_instance *vse = sd_to_vse_v4l_instance(sd);
	struct v4l2_query_ext_ctrl *qectrl;
	struct cam_v4l2_ext_control *cam_ext_ctrl;
	int rc = 0;

	switch(cmd) {
		case CAM_SET_CTRL:
		case CAM_GET_CTRL:
		case CAM_QUERY_CTRL:
			rc = subdev_call_command(sd, cmd, arg);
			break;
		case CAM_SET_EXT_CTRL:
			cam_ext_ctrl = (struct cam_v4l2_ext_control*)arg;
			switch (cam_ext_ctrl->controls->id) {
				case V4L2_CID_DR_VSE_ATTR:
					rc = vse_s_attr(vse, arg);
					break;
				default:
					rc = subdev_call_command(sd, cmd, (void *)cam_ext_ctrl->controls);
					break;
			}
			break;
		case CAM_GET_EXT_CTRL:
			cam_ext_ctrl = (struct cam_v4l2_ext_control *)arg;
			switch (cam_ext_ctrl->controls->id) {
				case V4L2_CID_DR_VSE_ATTR:
					rc = vse_g_attr(vse, arg);
					break;
				default:
					rc = subdev_call_command(sd, cmd, (void *)cam_ext_ctrl->controls);
					break;
			}
			break;
		case CAM_QUERY_EXT_CTRL:
			qectrl = (struct v4l2_query_ext_ctrl *)arg;
			switch (qectrl->id) {
				case V4L2_CID_DR_VSE_ATTR:
					rc = get_name_for_ext_ctrl(qectrl->id, qectrl->name);
					break;
				default:
					return -EINVAL;
			}
			break;
		default:
			break;
	}

	return rc;
}

static const struct v4l2_subdev_core_ops vse_core_ops = {
	.command = vse_command,
};

static const struct v4l2_subdev_video_ops vse_video_ops = {
	.s_stream = vse_s_stream,
	.g_frame_interval = vse_g_frame_interval,
	.s_frame_interval = vse_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops vse_pad_ops = {
	.set_fmt = vse_set_fmt,
	.get_fmt = vse_get_fmt,
	.enum_mbus_code = vse_enum_mbus_code,
	.enum_frame_size = vse_enum_frame_size,
	.enum_frame_interval = vse_enum_frame_interval,
};

static const struct v4l2_subdev_ops vse_subdev_ops = {
	.core = &vse_core_ops,
	.video = &vse_video_ops,
	.pad = &vse_pad_ops,
};

static int vse_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	int rc;

	rc = subdev_open(sd);
	if (rc < 0)
		return rc;

	return vse_open(inst->dev, inst->id);
}

static int vse_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	int rc;

	rc = subdev_close(sd);
	if (rc < 0) {
		pr_err("%s failed to call subdev_close (err=%d)\n", __func__, rc);
		return rc;
	}

	rc = vse_close(inst->dev, inst->id);
	if (rc < 0) {
		pr_err("%s failed to call vse_close (err=%d)\n", __func__, rc);
		return rc;
	}

	memset(&inst->ifmt, 0, sizeof(inst->ifmt));
	inst->out_pixelformat = 0;
	return 0;
}

static const struct v4l2_subdev_internal_ops vse_internal_ops = {
	.open = vse_v4l_open,
	.close = vse_v4l_close,
};

static void vse_inst_remove(struct vse_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int vse_async_bound(struct subdev_node *sn)
{
	struct vse_v4l_instance *vse =
			container_of(sn, struct vse_v4l_instance, node);
	struct vse_v4l_instance *ins;
	struct vse_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(vse->dev, struct vse_v4l_device, vse_dev);

	while (i < vse->dev->num_insts) {
		ins = &v4l_dev->insts[i];
		cam_ctx_release(&ins->sink_ctx);
		for (j = 0; j < VSE_OUT_CHNL_MAX; j++)
			cam_ctx_release(&ins->src_ctx[j]);

		if (ins != vse) {
			rc = v4l2_device_register_subdev
					(sn->sd.v4l2_dev, &ins->node.sd);
			if (rc < 0)
				goto _err;
		}
		i++;
	}
	return 0;

_err:
	j = 0;
	while (j < i) {
		ins = &v4l_dev->insts[j];

		if (ins != vse)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int vse_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vse_v4l_device *v4l_dev;
	struct vse_v4l_instance *insts;
	u32 i, j;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = vse_probe(pdev, &v4l_dev->vse_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call vse_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->vse_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->vse_dev.num_insts; i++) {
		struct vse_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->vse_dev;
		refcount_set(&inst->state_count, REFCNT_INIT_VAL);
		for (j = 0; j < VSE_OUT_CHNL_MAX; j++) {
			inst->out_fps[j].numerator = 30;
			inst->out_fps[j].denominator = 1;
		}

		n->async_bound = vse_async_bound;

		n->bctx.ready = vse_buf_ready;
		n->bctx.qbuf = vse_qbuf;
		n->bctx.drop = vse_drop;
		n->bctx.dqbuf = vse_dqbuf;
		n->bctx.trigger = vse_trigger;
		n->bctx.is_completed = vse_is_completed;
		n->bctx.get_format = vse_get_out_format;
		n->bctx.set_format = vse_set_out_format;
		n->bctx.enum_format = vse_enum_out_format;
		n->bctx.enum_framesize = vse_enum_out_framesize;
		n->bctx.enum_frameinterval = vse_enum_out_frameinterval;
		n->bctx.set_cap = vse_set_cap;

		n->dev = dev;
		n->num_pads = VSE_OUT_CHNL_MAX + 1;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			vse_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		for (j = 1; j < n->num_pads; j++) {
			n->pads[j].flags = MEDIA_PAD_FL_SOURCE;
			inst->src_pads[j - 1] = &n->pads[j];
		}
		n->pads[1].flags |= MEDIA_PAD_FL_MUST_CONNECT;

		rc = subdev_init(n, VSE_DEV_NAME, v4l_dev->vse_dev.id,
				 i, &vse_subdev_ops, &vse_media_ops);
		if (rc < 0) {
			vse_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &vse_internal_ops;

		if (i < VSE_SINK_ONLINE_PATH_MAX)
			n->bctx.is_sink_online_mode = true;
	}
	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		vse_inst_remove(insts, v4l_dev->vse_dev.num_insts);
		vse_remove(pdev, &v4l_dev->vse_dev);
		return rc;
	}

	platform_set_drvdata(pdev, v4l_dev);

#ifdef CONFIG_DEBUG_FS
	vse_debugfs_init(&v4l_dev->vse_dev);
#endif

	if (v4l_dev->vse_dev.axi)
		dev_dbg(dev, "axi clock: %ld Hz\n", clk_get_rate(v4l_dev->vse_dev.axi));
	if (v4l_dev->vse_dev.core)
		dev_dbg(dev, "core clock: %ld Hz\n", clk_get_rate(v4l_dev->vse_dev.core));
	if (v4l_dev->vse_dev.ups)
		dev_dbg(dev, "ups clock: %ld Hz\n", clk_get_rate(v4l_dev->vse_dev.ups));

	dev_dbg(dev, "VS VSE driver (v4l) probed done\n");
	return 0;
}

static int vse_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vse_v4l_device *v4l_dev = platform_get_drvdata(pdev);
	int rc;
	u32 i, j;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->vse_dev.num_insts; i++) {
		cam_ctx_release(&v4l_dev->insts[i].sink_ctx);
		for (j = 0; j < VSE_OUT_CHNL_MAX; j++)
			cam_ctx_release(&v4l_dev->insts[i].src_ctx[j]);
		subdev_deinit(&v4l_dev->insts[i].node);
	}

	rc = vse_remove(pdev, &v4l_dev->vse_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call vse_remove (err=%d)\n", rc);
		return rc;
	}

#ifdef CONFIG_DEBUG_FS
	vse_debugfs_remo(&v4l_dev->vse_dev);
#endif

	dev_dbg(dev, "VS VSE driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops vse_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vse_system_suspend, vse_system_resume)
};

static const struct of_device_id vse_of_match[] = {
	{ .compatible = VSE_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, vse_of_match);

static struct platform_driver vse_driver = {
	.probe  = vse_v4l_probe,
	.remove = vse_v4l_remove,
	.driver = {
		.name = VSE_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vse_of_match,
		.pm = &vse_pm_ops,
	}
};

static int __init vse_init_module(void)
{
	return platform_driver_register(&vse_driver);
}

static void __exit vse_exit_module(void)
{
	platform_driver_unregister(&vse_driver);
}

module_init(vse_init_module);
module_exit(vse_exit_module);

MODULE_DESCRIPTION("VeriSilicon VSE Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VeriSilicon-VSE");
