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

static int get_channel_index(struct vse_v4l_instance *vse, u32 pad)
{
	int i;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++)
		if (vse->src_pads[i] && vse->src_pads[i]->index == pad)
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
	struct init_attr attr = {
		.en_reqbufs = true,
	}, *p_attr = NULL;
	int rc = 0, index = -1;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	vse = sd_to_vse_v4l_instance(sd);
	attr.dev = sd->dev;

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (local->flags & MEDIA_PAD_FL_SINK)
		vse->m2m_en = false;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &vse->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SOURCE) {
			lctx->is_src_online_mode = rctx->is_sink_online_mode;
		} else {
			if (lctx->is_sink_online_mode != rctx->is_src_online_mode)
				return -EBUSY;
			if (!lctx->is_sink_online_mode)
				p_attr = &attr;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		vse->m2m_en = true;
		attr.en_reqbufs = false;
		p_attr = &attr;
	}

	if (local->flags & MEDIA_PAD_FL_SINK) {
		buf_ctx = &vse->sink_ctx;
	} else {
		index = get_channel_index(vse, local->index);
		if (index < 0)
			return -EINVAL;
		buf_ctx = &vse->src_ctx[index];
	}

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (buf_ctx->pad)
			return -EBUSY;

		rc = cam_ctx_init(buf_ctx, (void *)local, p_attr);
		if (rc < 0)
			return rc;
	} else {
		cam_ctx_release(buf_ctx);
	}
	return rc;
}

static const struct media_entity_operations vse_media_ops = {
	.link_setup = vse_link_setup,
};

static void vse_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);
	int rc;

	if (!ctx || !on)
		return;

	if (vse->node.bctx.is_sink_online_mode)
		rc = cam_ready(&vse->sink_ctx, on);
	else
		rc = vse_wake_up(vse->dev, vse->id);
	if (rc < 0)
		pr_err("%s failed to handle buf ready (err=%d)\n", __func__, rc);
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

static u32 vse_get_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad)
{
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

static void vse_set_res_cap(struct vse_v4l_instance *inst);
static int vse_set_ctx_format_out(struct v4l2_buf_ctx *ctx, struct v4l2_format *format, bool is_try)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);

	if (!is_try) {
		inst->input_fmt = format->fmt.pix.pixelformat;
		inst->input_res.dc.width = format->fmt.pix.width;
		inst->input_res.dc.height = format->fmt.pix.height;
		vse_set_res_cap(inst);
	}
	return 0;
}

static int vse_enum_ctx_format_out(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);

	if (index >= inst->input_fmt_cap_num)
		return -EINVAL;

	*format = inst->input_fmt_cap[index];
	return 0;
}

static int vse_enum_ctx_framesize_out(struct v4l2_buf_ctx *ctx, struct v4l2_frmsizeenum *fsize)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct cam_res_cap *res;

	res = &inst->input_res_range;
	fsize->type = res->type;
	fsize->stepwise.min_width = res->sw.min_width;
	fsize->stepwise.max_width = res->sw.max_width;
	fsize->stepwise.min_height = res->sw.min_height;
	fsize->stepwise.max_height = res->sw.max_height;
	fsize->stepwise.step_width = res->sw.step_width;
	fsize->stepwise.step_height = res->sw.step_height;
	return 0;
}

static int vse_set_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
							  struct v4l2_format *format, bool is_try)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct cam_format f;
	struct v4l2_format s_f = *format;
	struct cam_rect crop = {0};
	struct vse_fps_rate fps;
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	u32 devid, insid;
	int channel = -1;
	int hfactor = 1, vfactor = 1;
	int rc = 0;

	if (!pad)
		return vse_set_ctx_format_out(ctx, format, is_try);

	if (pad < inst->node.num_pads)
		channel = pad - 1;

	if (channel < 0)
		return -EINVAL;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd && !inst->m2m_en)
		return -EINVAL;

	memset(&f, 0, sizeof(f));
	f.format = pixelformat_to_cam_format(inst->input_fmt);
	f.width = inst->input_res.dc.width;
	f.height = inst->input_res.dc.height;
	f.stride = ALIGN(f.width, STRIDE_ALIGN);
	mutex_lock(&inst->fmt_lock);
	if (!inst->fmt_changed || memcmp(&f, &inst->ifmt, sizeof(f))) {
		struct vse_msg msg;

		s_f.fmt.pix.width = f.width;
		s_f.fmt.pix.height = f.height;
		s_f.fmt.pix.pixelformat = inst->input_fmt;
		if (!inst->m2m_en) {
			rc = v4l2_subdev_ctx_call(rsd, set_format, rpad->index, &s_f, is_try);
			if (rc < 0) {
				pr_err("%s v4l2_subdev_ctx_call failed\n", __func__);
				goto _exit;
			}
		}
		msg.id = CAM_MSG_STATE_CHANGED;
		msg.inst = inst->id;
		msg.state = CAM_STATE_INITED;
		pr_debug("%s set vse state to INITED\n", __func__);
		vse_post(inst->dev, &msg, true);

		rc = vse_set_iformat(inst->dev, inst->id, &f);
		if (rc < 0)
			goto _exit;
		memcpy(&inst->ifmt, &f, sizeof(f));
		inst->fmt_changed = true;
	}

	get_front_info(sd, &devid, &insid);
	if (inst->node.bctx.is_sink_online_mode)
		vse_set_cascade(inst->dev, inst->id, insid, true);
	else
		vse_set_cascade(inst->dev, inst->id, insid, false);

	f.width = ALIGN_DOWN(format->fmt.pix.width / hfactor, 16);
	f.height = format->fmt.pix.height / vfactor;
	f.stride = ALIGN(format->fmt.pix.width, STRIDE_ALIGN);
	f.format = pixelformat_to_cam_format(format->fmt.pix.pixelformat);
	rc = vse_set_oformat(inst->dev, inst->id, channel, &f, &crop, true);
	if (rc < 0) {
		pr_err("%s vse_set_oformat failed\n", __func__);
		goto _exit;
	}

	if (!inst->m2m_en)
		fps.src = vse_get_sensor_fps(sd);
	else
		fps.src = 30;
	fps.dst = fps.src;
	rc = vse_set_fps_rate(inst->dev, inst->id, channel, &fps);
	if (rc < 0) {
		pr_err("%s vse_set_fps_dst_rate failed", __func__);
		goto _exit;
	}

_exit:
	mutex_unlock(&inst->fmt_lock);
	return rc;
}

static int vse_enum_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad, u32 index, u32 *format)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);

	if (!pad)
		return vse_enum_ctx_format_out(ctx, index, format);

	if (index >= inst->fmt_cap_num)
		return -EINVAL;

	*format = inst->fmt_cap[index];
	return 0;
}

static int vse_enum_ctx_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
								  struct v4l2_frmsizeenum *fsize)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct cam_res_cap *res;
	int channel = -1;

	if (!pad)
		return vse_enum_ctx_framesize_out(ctx, fsize);

	if (pad < inst->node.num_pads)
		channel = pad - 1;

	if (channel < 0 || fsize->index != 0)
		return -EINVAL;

	res = &inst->res_cap[channel];
	fsize->type = res->type;
	fsize->stepwise.min_width = res->sw.min_width;
	fsize->stepwise.max_width = res->sw.max_width;
	fsize->stepwise.min_height = res->sw.min_height;
	fsize->stepwise.max_height = res->sw.max_height;
	fsize->stepwise.step_width = res->sw.step_width;
	fsize->stepwise.step_height = res->sw.step_height;

	return 0;
}

static bool vse_check_res(struct cam_res_cap *res, u32 width, u32 height)
{
	int diff;

	if (width > res->sw.max_width || width < res->sw.min_width)
		return false;

	if (height > res->sw.max_height || height < res->sw.min_height)
		return false;

	diff = width - res->sw.min_width;
	if (diff % res->sw.step_width != 0)
		return false;

	diff = height - res->sw.min_height;
	if (diff % res->sw.step_height != 0)
		return false;

	return true;
}

static int vse_enum_ctx_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	int channel = -1;
	struct v4l2_frmivalenum fiv;
	int rc;

	if (pad < inst->node.num_pads)
		channel = pad - 1;

	if (channel < 0 || fival->index != 0)
		return -EINVAL;

	if (!vse_check_res(&inst->res_cap[channel], fival->width, fival->height))
		return -EINVAL;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return -EINVAL;

	memcpy(&fiv, fival, sizeof(fiv));
	fiv.pixel_format = inst->input_fmt;
	fiv.width = inst->input_res.dc.width;
	fiv.height = inst->input_res.dc.height;
	fiv.pixel_format = inst->input_fmt;
	rc = v4l2_subdev_ctx_call(rsd, enum_frameinterval, rpad->index, &fiv);
	if (rc < 0)
		return rc;

	fival->type = fiv.type;
	fival->discrete = fiv.discrete;
	fival->stepwise = fiv.stepwise;
	return rc;
}

static void vse_set_res_cap(struct vse_v4l_instance *inst)
{
	struct cam_res_cap *res, *input_res;
	int i;
	u32 iwidth, iheight;

	iwidth = inst->input_res.dc.width;
	iheight = inst->input_res.dc.height;

	memset(inst->res_cap, 0, sizeof(inst->res_cap));
	res = inst->res_cap;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		res[i].type           = V4L2_FRMSIZE_TYPE_STEPWISE;
		res[i].sw.step_width  = 2;
		res[i].sw.step_height = 2;
		if (i < 5) {
			res[i].sw.min_width = 64;
			res[i].sw.min_height = 64;
		} else {
			res[i].sw.min_width = iwidth;
			res[i].sw.min_height = iheight;
		}
		switch (i) {
		case 0:
			res[i].sw.max_width = V4L2_MIN(4096, iwidth);
			res[i].sw.max_height = V4L2_MIN(3076, iheight);
			break;
		case 1:
		case 2:
			res[i].sw.max_width = V4L2_MIN(1920, iwidth);
			res[i].sw.max_height = V4L2_MIN(1080, iheight);
			break;
		case 3:
		case 4:
			res[i].sw.max_width = V4L2_MIN(1280, iwidth);
			res[i].sw.max_height = V4L2_MIN(720, iheight);
			break;
		case 5:
			res[i].sw.max_width = 4096;
			res[i].sw.max_height = 3076;
			break;
		default:
			break;
		}
	}

	//fixme
	input_res = &inst->input_res_range;
	input_res->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	input_res->sw.min_width = 1;
	input_res->sw.min_height = 1;
	input_res->sw.max_width = 5432;
	input_res->sw.max_height = 3076;
	input_res->sw.step_width = 1;
	input_res->sw.step_height = 1;
}

static void vse_set_default_input(struct vse_v4l_instance *inst)
{
	inst->input_fmt = V4L2_PIX_FMT_NV12;
	inst->input_res.type = V4L2_FRMIVAL_TYPE_DISCRETE;
	inst->input_res.dc.width = 1920;
	inst->input_res.dc.height = 1080;
}

static void vse_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct vse_v4l_instance *inst = buf_ctx_to_vse_v4l_instance(ctx);
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	struct v4l2_frmsizeenum fsize;
	int i, j = 0, rc;
	bool sensor_support_default_size = false;

	inst->fmt_cap[0] = V4L2_PIX_FMT_NV12;
	inst->fmt_cap_num = 1;
	inst->input_fmt_cap[0] = V4L2_PIX_FMT_NV12;
	inst->input_fmt_cap_num = 1;
	vse_set_default_input(inst);

	if (inst->m2m_en)
		goto set_cap;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return;

	v4l2_subdev_ctx_call_no_return(rsd, set_cap);

	memset(inst->input_res_cap, 0, sizeof(inst->input_res_cap));

	for (i = 0; i < ARRAY_SIZE(inst->input_res_cap); i++) {
		memset(&fsize, 0, sizeof(fsize));
		fsize.index = i;
		fsize.pixel_format = inst->input_fmt;
		rc = v4l2_subdev_ctx_call(rsd, enum_framesize, rpad->index, &fsize);
		if (rc < 0)
			break;
		inst->input_res_cap[j].type = V4L2_FRMIVAL_TYPE_DISCRETE;
		inst->input_res_cap[j].dc.width = fsize.discrete.width;
		inst->input_res_cap[j].dc.height = fsize.discrete.height;
		if (fsize.discrete.width == inst->input_res.dc.width &&
			fsize.discrete.height == inst->input_res.dc.height)
			sensor_support_default_size = true;
		j++;
	}

	inst->input_res_cap_num = j;
	if (!sensor_support_default_size && j > 0)
		memcpy(&inst->input_res, &inst->input_res_cap[inst->input_res_cap_num-1],
			sizeof(inst->input_res_cap[0]));

set_cap:
	vse_set_res_cap(inst);
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

static void fill_irq_ctx(struct vse_v4l_instance *vse, u32 i, int enable,
			 struct vse_irq_ctx *ctx)
{
	ctx->is_sink_online_mode = vse->node.bctx.is_sink_online_mode;
	if (vse->sink_ctx.pad)
		ctx->sink_ctx = &vse->sink_ctx;
	if (vse->src_ctx[i].pad) {
		if (enable)
			ctx->src_ctx[i] = &vse->src_ctx[i];
		else
			ctx->src_ctx[i] = NULL;
	}
}

static int vse_set_stream(struct v4l2_buf_ctx *ctx, u32 pad, int enable)
{
	struct vse_v4l_instance *vse = buf_ctx_to_vse_v4l_instance(ctx);
	struct vse_irq_ctx irq_ctx;
	int index;

	if (pad >= vse->node.num_pads)
		return -EINVAL;

	index = get_channel_index(vse, pad);
	if (index < 0)
		return -EINVAL;

	vse_get_ctx(vse->dev, vse->id, &irq_ctx);
	fill_irq_ctx(vse, index, enable, &irq_ctx);
	vse_set_ctx(vse->dev, vse->id, &irq_ctx);
	return 0;
}

static int vse_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vse_v4l_instance *vse = sd_to_vse_v4l_instance(sd);
	int rc;

	if (enable) {
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL) {
			refcount_inc(&vse->state_count);
			return 0;
		}

		refcount_inc(&vse->state_count);
		if (vse->node.bctx.is_sink_online_mode)
			rc = vse_set_source(vse->dev, vse->id, VSE_SRC_STRM0);
		else
			rc = vse_set_source(vse->dev, vse->id, VSE_SRC_RDMA);
		if (rc < 0) {
			pr_err("%s failed to call vse_set_source (rc=%d)!\n", __func__, rc);
			return rc;
		}

		if (!vse->node.bctx.is_sink_online_mode)
			cam_reqbufs(&vse->sink_ctx, 4, &vse_buf_ops);

		rc = vse_set_state(vse->dev, vse->id, enable);
		if (rc < 0)
			return rc;

		if (!vse->m2m_en) {
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
		}
	} else {
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL)
			refcount_dec(&vse->state_count);
		if (refcount_read(&vse->state_count) > REFCNT_INIT_VAL)
			return 0;

		if (!vse->m2m_en) {
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
		}

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
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return -EINVAL;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, g_frame_interval, fiv);
}

static int vse_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return -EINVAL;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, s_frame_interval, fiv);
}

static void vse_get_cur_attr(struct vse_instance *ins, int chnl, vse_ochn_attr_ex_t *vse_attr)
{
	vse_attr->src_fps = ins->fps[chnl].src;
	vse_attr->dst_fps = ins->fps[chnl].dst;
	if (ins->fps[chnl].dst == 0)
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
		chnl = get_channel_index(inst, cam_ext_ctrl->pad);

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
		chnl = get_channel_index(inst, cam_ext_ctrl->pad);

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
	int rc = -EINVAL;

	switch (cmd) {
	case CAM_SET_CTRL:
	case CAM_GET_CTRL:
	case CAM_QUERY_CTRL:
		if (!vse->m2m_en)
			rc = subdev_call_command(sd, cmd, arg);
		break;
	case CAM_SET_EXT_CTRL:
		cam_ext_ctrl = (struct cam_v4l2_ext_control *)arg;
		switch (cam_ext_ctrl->controls->id) {
		case V4L2_CID_DR_VSE_ATTR:
			rc = vse_s_attr(vse, arg);
			break;
		default:
			if (!vse->m2m_en)
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
			if (!vse->m2m_en)
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
		rc = -EINVAL;
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

static const struct v4l2_subdev_ops vse_subdev_ops = {
	.core = &vse_core_ops,
	.video = &vse_video_ops,
};

static int vse_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	int rc = 0;

	mutex_lock(&inst->open_lock);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL) {
		refcount_inc(&inst->open_count);
		goto _exit;
	}
	refcount_inc(&inst->open_count);
	if (!inst->m2m_en) {
		rc = subdev_open(sd);
		if (rc < 0)
			goto _exit;
	}
	rc = vse_open(inst->dev, inst->id);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
}

static int vse_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	int rc = 0;

	mutex_lock(&inst->open_lock);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL)
		refcount_dec(&inst->open_count);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL)
		goto _exit;

	if (!inst->m2m_en) {
		rc = subdev_close(sd);
		if (rc < 0) {
			pr_err("%s failed to call subdev_close (err=%d)\n", __func__, rc);
			goto _exit;
		}
	}
	rc = vse_close(inst->dev, inst->id);
	if (rc < 0) {
		pr_err("%s failed to call vse_close (err=%d)\n", __func__, rc);
		goto _exit;
	}

	memset(&inst->ifmt, 0, sizeof(inst->ifmt));
	vse_set_default_input(inst);
	vse_set_res_cap(inst);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
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
		mutex_init(&inst->open_lock);
		mutex_init(&inst->fmt_lock);
		refcount_set(&inst->state_count, REFCNT_INIT_VAL);
		refcount_set(&inst->open_count, REFCNT_INIT_VAL);

		n->async_bound = vse_async_bound;

		n->bctx.ready = vse_buf_ready;
		n->bctx.qbuf = vse_qbuf;
		n->bctx.drop = vse_drop;
		n->bctx.dqbuf = vse_dqbuf;
		n->bctx.trigger = vse_trigger;
		n->bctx.is_completed = vse_is_completed;
		n->bctx.get_format = vse_get_ctx_format;
		n->bctx.set_format = vse_set_ctx_format;
		n->bctx.enum_format = vse_enum_ctx_format;
		n->bctx.enum_framesize = vse_enum_ctx_framesize;
		n->bctx.enum_frameinterval = vse_enum_ctx_frameinterval;
		n->bctx.set_stream = vse_set_stream;
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
		devm_kfree(dev, v4l_dev->insts[i].node.pads);
		mutex_destroy(&v4l_dev->insts[i].open_lock);
		mutex_destroy(&v4l_dev->insts[i].fmt_lock);
	}
	devm_kfree(dev, v4l_dev->insts);

	rc = vse_remove(pdev, &v4l_dev->vse_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call vse_remove (err=%d)\n", rc);
		return rc;
	}

#ifdef CONFIG_DEBUG_FS
	vse_debugfs_remo(&v4l_dev->vse_dev);
#endif
	devm_kfree(dev, v4l_dev);

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
