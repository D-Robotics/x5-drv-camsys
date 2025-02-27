// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[gdc_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "v4l2_usr_api.h"
#include "gdc_drv.h"

static int gdc_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct gdc_v4l_instance *gdc;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	struct init_attr attr = {
		.en_reqbufs = true,
	}, *p_attr = NULL;
	int rc = 0;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	gdc = sd_to_v4l_instance(gdc, sd);
	attr.dev = sd->dev;

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (local->flags & MEDIA_PAD_FL_SINK)
		gdc->m2m_en = false;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &gdc->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SINK) {
			if (sink_online_en(lctx) != rctx->src_online_en)
				return -EBUSY;
			if (is_sink_online_en(lctx))
				return -EBUSY;
			p_attr = &attr;
		} else {
			lctx->src_online_en = sink_online_en(rctx);
			if (lctx->src_online_en)
				return -EBUSY;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		gdc->m2m_en = true;
		attr.en_reqbufs = false;
		p_attr = &attr;
	}

	if (local->flags & MEDIA_PAD_FL_SINK)
		buf_ctx = &gdc->sink_ctx;
	else
		buf_ctx = &gdc->src_ctx;

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

static const struct media_entity_operations gdc_media_ops = {
	.link_setup = gdc_link_setup,
};

static void gdc_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);
	int rc;

	if (gdc && on) {
		rc = gdc_wake_up(gdc->dev, gdc->id);
		if (rc < 0)
			pr_err("%s failed to handle buf ready (err=%d)\n", __func__, rc);
	}
}

static int gdc_drop(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);

	if (!ctx || !buf)
		return -EINVAL;

	return cam_drop(&gdc->sink_ctx, buf);
}

static int gdc_qbuf(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);
	int rc;

	if (!ctx || !buf)
		return -EINVAL;

	if (is_sink_online_en(ctx))
		return -EBUSY;

	rc = cam_qbuf(&gdc->sink_ctx, buf);
	if (rc < 0)
		return rc;

	return gdc_add_job(gdc->dev, gdc->id);
}

static struct cam_buf *gdc_dqbuf(struct v4l2_buf_ctx *ctx, u32 pad)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);

	if (!ctx)
		return NULL;

	if (is_sink_online_en(ctx))
		return NULL;

	return cam_dqbuf(&gdc->sink_ctx);
}

static struct cam_buf *gdc_acqbuf(struct v4l2_buf_ctx *ctx, u32 pad)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);

	if (!ctx)
		return NULL;

	if (is_sink_online_en(ctx))
		return NULL;

	return cam_acqbuf(&gdc->sink_ctx);
}

static u32 gdc_get_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_format *format)
{
	return 0;
}

static int gdc_set_ctx_format_out(struct v4l2_buf_ctx *ctx,
				  struct v4l2_format *format, bool is_try)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);

	if (!is_try)
		inst->input_fmt = format->fmt.pix.pixelformat;
	return 0;
}

static int gdc_enum_ctx_format_out(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);

	if (index >= inst->fmt_cap_num)
		return -EINVAL;

	*format = inst->fmt_cap[index];
	return 0;
}

static int gdc_enum_ctx_framesize_out(struct v4l2_buf_ctx *ctx,
				      struct v4l2_frmsizeenum *fsize)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);
	struct cam_res_cap *res;

	res = &inst->input_res;
	fsize->type = res->type;
	fsize->discrete.width = res->dc.width;
	fsize->discrete.height = res->dc.height;
	return 0;
}

static int gdc_set_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_format *format, bool is_try)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);
	struct gdc_format f;
	struct v4l2_format s_f = *format;
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	int rc;

	if (is_sink_pad(inst, pad))
		return gdc_set_ctx_format_out(ctx, format, is_try);

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd && !inst->m2m_en)
		return -EINVAL;

	if (!inst->m2m_en) {
		s_f.fmt.pix.pixelformat = inst->input_fmt;
		s_f.fmt.pix.width = inst->input_res.dc.width;
		s_f.fmt.pix.height = inst->input_res.dc.height;
		rc = v4l2_subdev_ctx_call(rsd, set_format, rpad->index, &s_f, is_try);
		if (rc < 0) {
			pr_err("%s v4l2_subdev_ctx_call failed\n", __func__);
			return rc;
		}
	}

	memset(&f, 0, sizeof(f));
	f.ifmt.format = pixelformat_to_cam_format(inst->input_fmt);
	f.ifmt.width  = inst->input_res.dc.width;
	f.ifmt.height = inst->input_res.dc.height;
	f.ifmt.stride = ALIGN(f.ifmt.width, STRIDE_ALIGN);
	f.ofmt.width  = format->fmt.pix.width;
	f.ofmt.height = format->fmt.pix.height;
	f.ofmt.stride = ALIGN(f.ofmt.width, STRIDE_ALIGN);
	f.ofmt.format = pixelformat_to_cam_format(format->fmt.pix.pixelformat);

	return gdc_set_format(inst->dev, inst->id, &f);
}

static int gdc_enum_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad, u32 index, u32 *format)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);

	if (is_sink_pad(inst, pad))
		return gdc_enum_ctx_format_out(ctx, index, format);

	if (index >= inst->fmt_cap_num)
		return -EINVAL;

	*format = inst->fmt_cap[index];
	return 0;
}

static int gdc_enum_ctx_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);
	struct cam_res_cap *res;

	if(pad >= inst->node.num_pads || fsize->index != 0)
		return -EINVAL;

	if (is_sink_pad(inst, pad))
		return gdc_enum_ctx_framesize_out(ctx, fsize);

	res = &inst->res_cap;
	fsize->type = res->type;
	fsize->stepwise.min_width = res->sw.min_width;
	fsize->stepwise.max_width = res->sw.max_width;
	fsize->stepwise.min_height = res->sw.min_height;
	fsize->stepwise.max_height = res->sw.max_height;
	fsize->stepwise.step_width = res->sw.step_width;
	fsize->stepwise.step_height = res->sw.step_height;
	return 0;
}

static int gdc_enum_ctx_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	struct v4l2_frmivalenum fiv;
	int rc;

	if (!check_stepwise_res(&inst->res_cap, fival->width, fival->height))
		return -EINVAL;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd)
		return -ENOLINK;

	memcpy(&fiv, fival, sizeof(fiv));
	fiv.pixel_format = inst->input_fmt;
	fiv.width = inst->input_res.dc.width;
	fiv.height = inst->input_res.dc.height;
	rc = v4l2_subdev_ctx_call(rsd, enum_frameinterval, rpad->index, &fiv);
	if (rc < 0)
		return rc;

	fival->type = fiv.type;
	fival->discrete = fiv.discrete;
	fival->stepwise = fiv.stepwise;
	return rc;
}

static void gdc_set_res_cap(struct gdc_v4l_instance *inst)
{
	struct cam_res_cap *res;

	res = &inst->res_cap;
	res->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	res->sw.step_width  = 2;
	res->sw.step_height = 2;
	res->sw.min_width = 32;
	res->sw.max_width = 4096;
	res->sw.min_height = 32;
	res->sw.max_height = 4096;
}

static void gdc_set_default_input(struct gdc_v4l_instance *inst)
{
	inst->input_fmt = inst->fmt_cap[inst->fmt_cap_num - 1];
	inst->input_res.type = V4L2_FRMIVAL_TYPE_DISCRETE;
	inst->input_res.dc.width = 1920;
	inst->input_res.dc.height = 1080;
}

static void gdc_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_v4l_instance(gdc, ctx);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	struct v4l2_frmsizeenum fsize;
	bool subdev_support_default_size = false;
	int i, j = 0, rc;

	inst->fmt_cap[0] = V4L2_PIX_FMT_NV12;
	inst->fmt_cap_num = 1;
	gdc_set_default_input(inst);
	gdc_set_res_cap(inst);

	if (inst->m2m_en)
		return;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
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
		if (fsize.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
			inst->input_res_cap[j].type = V4L2_FRMIVAL_TYPE_DISCRETE;
			inst->input_res_cap[j].dc.width = fsize.discrete.width;
			inst->input_res_cap[j].dc.height = fsize.discrete.height;
			if (fsize.discrete.width == inst->input_res.dc.width &&
				fsize.discrete.height == inst->input_res.dc.height)
				subdev_support_default_size = true;
		} else if (fsize.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
			inst->input_res_cap[j].type = V4L2_FRMIVAL_TYPE_STEPWISE;
			inst->input_res_cap[j].sw.min_width = fsize.stepwise.min_width;
			inst->input_res_cap[j].sw.max_width = fsize.stepwise.max_width;
			inst->input_res_cap[j].sw.min_height = fsize.stepwise.min_height;
			inst->input_res_cap[j].sw.max_height = fsize.stepwise.max_height;
			inst->input_res_cap[j].sw.step_width = fsize.stepwise.step_width;
			inst->input_res_cap[j].sw.step_height = fsize.stepwise.step_height;
			if (check_stepwise_res(&inst->input_res_cap[j],
				inst->input_res.dc.width, inst->input_res.dc.height))
				subdev_support_default_size = true;
		} else {
			break;
		}
		j++;
	}
	inst->input_res_cap_num = j;
	if (!subdev_support_default_size && j > 0)
		memcpy(&inst->input_res, &inst->input_res_cap[inst->input_res_cap_num - 1],
		sizeof(struct cam_res_cap));
}

static void fill_irq_ctx(struct gdc_v4l_instance *gdc, int enable, struct gdc_irq_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	if (enable) {
		if (gdc->sink_ctx.pad)
			ctx->sink_ctx = &gdc->sink_ctx;
		if (gdc->src_ctx.pad)
			ctx->src_ctx = &gdc->src_ctx;
	}
}

static int gdc_queue_setup(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct gdc_v4l_instance *ins =
			container_of(ctx, struct gdc_v4l_instance, sink_ctx);
	struct gdc_instance *gdc;
	unsigned int size = 0;

	if (ins) {
		gdc = &ins->dev->insts[ins->id];
		size = get_framebuf_size(&gdc->fmt.ifmt);
	}

	if (!size)
		return -ENOMEM;

	if (!*num_buffers)
		*num_buffers = 1;

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static struct cam_buf_ops gdc_buf_ops = {
	.queue_setup = gdc_queue_setup,
};

static int gdc_set_stream(struct v4l2_buf_ctx *ctx, u32 pad, int enable)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_v4l_instance(gdc, ctx);
	struct gdc_irq_ctx irq_ctx;

	if (pad >= gdc->node.num_pads)
		return -EINVAL;

	fill_irq_ctx(gdc, enable, &irq_ctx);
	return gdc_set_ctx(gdc->dev, gdc->id, &irq_ctx);
}

static int gdc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gdc_v4l_instance *gdc = sd_to_v4l_instance(gdc, sd);
	int rc = 0;

	if (enable) {
		if (!gdc->m2m_en && !is_sink_online_en(&gdc->node.bctx)) {
			rc = cam_reqbufs(&gdc->sink_ctx, V4L2_SUBDEV_BUF_NUM, &gdc_buf_ops);
			if (rc < 0)
				return rc;
		}

		rc = gdc_set_state(gdc->dev, gdc->id, enable);
		if (rc < 0)
			return rc;
		if (!gdc->m2m_en)
			rc = subdev_set_stream(sd, enable);
	} else {
		if (!gdc->m2m_en) {
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
		}

		if (!gdc->m2m_en && !is_sink_online_en(&gdc->node.bctx)) {
			rc = cam_reqbufs(&gdc->sink_ctx, 0, NULL);
			if (rc < 0)
				return rc;
		}

		rc = gdc_set_state(gdc->dev, gdc->id, enable);
	}
	return rc;
}

static int gdc_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct gdc_v4l_instance *gdc = sd_to_v4l_instance(gdc, sd);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rpad = get_remote_pad_sd(sink_pad(gdc), &rsd);
	if (!rsd)
		return -ENOLINK;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, g_frame_interval, fiv);
}

static int gdc_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct gdc_v4l_instance *gdc = sd_to_v4l_instance(gdc, sd);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rpad = get_remote_pad_sd(sink_pad(gdc), &rsd);
	if (!rsd)
		return -ENOLINK;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, s_frame_interval, fiv);
}

static int gdc_re_set_format(struct gdc_v4l_instance *inst, struct gdc_format *fmt)
{
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	struct v4l2_format format;
	int rc;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd && !inst->m2m_en)
		return -EINVAL;

	format.fmt.pix.pixelformat = cam_format_to_pixelformat(fmt->ifmt.format, 0);
	format.fmt.pix.width = fmt->ifmt.width;
	format.fmt.pix.height = fmt->ifmt.height;

	if (!inst->m2m_en) {
		rc = v4l2_subdev_ctx_call(rsd, set_format, rpad->index, &format, true);
		if (rc < 0) {
			pr_err("%s v4l2_subdev_ctx_call failed\n", __func__);
			return rc;
		}
	}

	return gdc_set_format(inst->dev, inst->id, fmt);
}

static int gdc_s_ctrl(struct gdc_v4l_instance *inst, void *arg)
{
	struct gdc_instance *ins;
	gdc_attr_t gdc_attr;
	struct v4l2_ext_control *ext_ctrl;
	struct gdc_format fmt;
	int rc = 0;

	ins = &inst->dev->insts[inst->id];
	ext_ctrl = (struct v4l2_ext_control *)arg;
	rc = copy_from_user(&gdc_attr, ext_ctrl->ptr, sizeof(gdc_attr));
	if (rc < 0)
		return rc;

	gdc_get_format(inst->dev, inst->id, &fmt);

	if ((fmt.ifmt.width  != gdc_attr.input_width) ||
	    (fmt.ifmt.height != gdc_attr.input_height) ||
	    (fmt.ofmt.width  != gdc_attr.output_width) ||
	    (fmt.ofmt.height != gdc_attr.output_height)) {
		pr_debug("gdc%d format need to be changed\n", inst->id);
		pr_debug("input: %dx%d -> %dx%d\n", fmt.ifmt.width, fmt.ifmt.height,
			 gdc_attr.input_width, gdc_attr.input_height);
		pr_debug("output: %dx%d -> %dx%d\n", fmt.ofmt.width, fmt.ofmt.height,
			 gdc_attr.output_width, gdc_attr.output_height);
		fmt.ifmt.width  = gdc_attr.input_width;
		fmt.ifmt.height = gdc_attr.input_height;
		fmt.ifmt.stride = gdc_attr.input_stride;
		fmt.ofmt.width  = gdc_attr.output_width;
		fmt.ofmt.height = gdc_attr.output_height;
		fmt.ofmt.stride = gdc_attr.output_stride;

		rc = gdc_re_set_format(inst, &fmt);
		if (rc < 0) {
			pr_err("gdc re-set format failed, rc=%d\n", rc);
			return rc;
		}
	}

	rc = gdc_set_attr(inst->dev, inst->id, gdc_attr.config_addr, gdc_attr.config_size);
	if (rc < 0) {
		pr_err("gdc set attr failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int gdc_g_ctrl(struct gdc_v4l_instance *inst, void *arg)
{
	struct gdc_instance *ins;
	gdc_attr_t gdc_attr;
	struct gdc_format fmt;
	struct v4l2_ext_control *ext_ctrl;
	int rc = 0;

	ins = &inst->dev->insts[inst->id];
	ext_ctrl = (struct v4l2_ext_control *)arg;

	rc = copy_from_user(&gdc_attr, ext_ctrl->ptr, sizeof(gdc_attr));
	if (rc < 0)
		return rc;

	rc = gdc_get_attr(inst->dev, inst->id, &gdc_attr.config_addr, &gdc_attr.config_size);
	if (rc < 0) {
		pr_err("gdc get cur attr failed, rc=%d\n", rc);
		return rc;
	}

	gdc_get_format(inst->dev, inst->id, &fmt);

	gdc_attr.input_width   = fmt.ifmt.width;
	gdc_attr.input_height  = fmt.ifmt.height;
	gdc_attr.input_stride  = fmt.ifmt.stride;
	gdc_attr.output_width  = fmt.ofmt.width;
	gdc_attr.output_height = fmt.ofmt.height;
	gdc_attr.output_stride = fmt.ofmt.stride;

	rc = copy_to_user(ext_ctrl->ptr,  &gdc_attr, sizeof(gdc_attr));
	if (rc < 0)
		return rc;

	return rc;
}

static int get_name_for_ext_ctrl(uint32_t id, char *name)
{
	const char *source;

	switch (id) {
		case V4L2_CID_DR_GDC_ATTR:
			source = "gdc_attr_t";
			break;
		default:
			return -1;
	}
	memcpy(name, source, strlen(source)+1);
	return 0;
}

static long gdc_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gdc_v4l_instance *gdc = sd_to_v4l_instance(gdc, sd);
	struct v4l2_query_ext_ctrl *qectrl;
	struct v4l2_ext_control *vectl;
	int rc = -EINVAL;

	switch (cmd) {
	case CAM_SET_CTRL:
	case CAM_GET_CTRL:
	case CAM_QUERY_CTRL:
		if (!gdc->m2m_en)
			rc = subdev_call_command(sd, cmd, arg);
		break;
	case CAM_SET_EXT_CTRL:
		vectl = (struct v4l2_ext_control *)arg;
		switch (vectl->id) {
		case V4L2_CID_DR_GDC_ATTR:
			rc = gdc_s_ctrl(gdc, arg);
			break;
		default:
			if (!gdc->m2m_en)
				rc = subdev_call_command(sd, cmd, arg);
			break;
		}
		break;
	case CAM_GET_EXT_CTRL:
		vectl = (struct v4l2_ext_control *)arg;
		switch (vectl->id) {
		case V4L2_CID_DR_GDC_ATTR:
			rc = gdc_g_ctrl(gdc, arg);
			break;
		default:
			if (!gdc->m2m_en)
				rc = subdev_call_command(sd, cmd, arg);
			break;
		}
		break;
	case CAM_QUERY_EXT_CTRL:
		qectrl = (struct v4l2_query_ext_ctrl *)arg;
		switch (qectrl->id) {
		case V4L2_CID_DR_GDC_ATTR:
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

static const struct v4l2_subdev_core_ops gdc_core_ops = {
	.command = gdc_command,
};

static const struct v4l2_subdev_video_ops gdc_video_ops = {
	.s_stream = gdc_s_stream,
	.g_frame_interval = gdc_g_frame_interval,
	.s_frame_interval = gdc_s_frame_interval,
};

static const struct v4l2_subdev_ops gdc_subdev_ops = {
	.core = &gdc_core_ops,
	.video = &gdc_video_ops,
};

static int gdc_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gdc_v4l_instance *inst = sd_to_v4l_instance(gdc, sd);
	int rc;

	if (!inst->m2m_en) {
		rc = subdev_open(sd);
		if (rc < 0)
			return rc;
	}

	return gdc_open(inst->dev, inst->id);
}

static int gdc_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gdc_v4l_instance *inst = sd_to_v4l_instance(gdc, sd);
	int rc;

	if (!inst->m2m_en) {
		rc = subdev_close(sd);
		if (rc < 0)
			return rc;
	}

	return gdc_close(inst->dev, inst->id);
}

static const struct v4l2_subdev_internal_ops gdc_internal_ops = {
	.open = gdc_v4l_open,
	.close = gdc_v4l_close,
};

static void gdc_inst_remove(struct gdc_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int gdc_async_bound(struct subdev_node *sn)
{
	struct gdc_v4l_instance *gdc =
			container_of(sn, struct gdc_v4l_instance, node);
	struct gdc_v4l_instance *ins;
	struct gdc_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(gdc->dev, struct gdc_v4l_device, gdc_dev);

	while (i < gdc->dev->num_insts) {
		ins = &v4l_dev->insts[i];
		cam_ctx_release(&ins->sink_ctx);
		cam_ctx_release(&ins->src_ctx);

		if (ins != gdc) {
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

		if (ins != gdc)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int gdc_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gdc_v4l_device *v4l_dev;
	struct gdc_v4l_instance *insts;
	u32 i;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = gdc_probe(pdev, &v4l_dev->gdc_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call gdc_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->gdc_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->gdc_dev.num_insts; i++) {
		struct gdc_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->gdc_dev;

		n->async_bound = gdc_async_bound;

		n->bctx.ready = gdc_buf_ready;
		n->bctx.qbuf = gdc_qbuf;
		n->bctx.dqbuf = gdc_dqbuf;
		n->bctx.acqbuf = gdc_acqbuf;
		n->bctx.drop = gdc_drop;
		n->bctx.get_format = gdc_get_ctx_format;
		n->bctx.set_format = gdc_set_ctx_format;
		n->bctx.enum_format = gdc_enum_ctx_format;
		n->bctx.enum_framesize = gdc_enum_ctx_framesize;
		n->bctx.enum_frameinterval = gdc_enum_ctx_frameinterval;
		n->bctx.set_stream = gdc_set_stream;
		n->bctx.set_cap = gdc_set_cap;

		n->dev = dev;
		n->num_pads = 2;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			gdc_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		sink_pad(inst)->flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;

		rc = subdev_init(n, GDC_DEV_NAME, v4l_dev->gdc_dev.id,
				 i, &gdc_subdev_ops, &gdc_media_ops);
		if (rc < 0) {
			gdc_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &gdc_internal_ops;
	}

	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		gdc_inst_remove(insts, v4l_dev->gdc_dev.num_insts);
		gdc_remove(pdev, &v4l_dev->gdc_dev);
		return rc;
	}

	platform_set_drvdata(pdev, &v4l_dev->gdc_dev);

	rc = gdc_runtime_resume(dev);
	if (rc) {
		dev_err(dev, "failed to call gdc_runtime_resume (err=%d)\n", rc);
		return rc;
	}

	if (v4l_dev->gdc_dev.axi)
		dev_dbg(dev, "axi clock: %ld Hz\n", clk_get_rate(v4l_dev->gdc_dev.axi));
	if (v4l_dev->gdc_dev.core)
		dev_dbg(dev, "core clock: %ld Hz\n", clk_get_rate(v4l_dev->gdc_dev.core));
	if (v4l_dev->gdc_dev.hclk)
		dev_dbg(dev, "hclk clock: %ld Hz\n", clk_get_rate(v4l_dev->gdc_dev.hclk));

	dev_dbg(dev, "ARM GDC driver (v4l) probed done\n");
	return 0;
}

static int gdc_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gdc_device *gdc_dev = platform_get_drvdata(pdev);
	struct gdc_v4l_device *v4l_dev =
			container_of(gdc_dev, struct gdc_v4l_device, gdc_dev);
	int rc;
	u32 i;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->gdc_dev.num_insts; i++) {
		cam_ctx_release(&v4l_dev->insts[i].sink_ctx);
		cam_ctx_release(&v4l_dev->insts[i].src_ctx);
		subdev_deinit(&v4l_dev->insts[i].node);
		devm_kfree(dev, v4l_dev->insts[i].node.pads);
	}
	devm_kfree(dev, v4l_dev->insts);

	rc = gdc_remove(pdev, &v4l_dev->gdc_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call gdc_remove (err=%d)\n", rc);
		return rc;
	}

	rc = gdc_runtime_suspend(dev);
	if (rc) {
		dev_err(dev, "failed to call gdc_runtime_suspend (err=%d)\n", rc);
		return rc;
	}
	devm_kfree(dev, v4l_dev);

	dev_dbg(dev, "ARM GDC driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops gdc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gdc_system_suspend, gdc_system_resume)
};

static const struct of_device_id gdc_of_match[] = {
	{ .compatible = GDC_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, gdc_of_match);

static struct platform_driver gdc_driver = {
	.probe  = gdc_v4l_probe,
	.remove = gdc_v4l_remove,
	.driver = {
		.name = GDC_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gdc_of_match,
		.pm = &gdc_pm_ops,
	}
};

static int __init gdc_init_module(void)
{
	return platform_driver_register(&gdc_driver);
}

static void __exit gdc_exit_module(void)
{
	platform_driver_unregister(&gdc_driver);
}

module_init(gdc_init_module);
module_exit(gdc_exit_module);

MODULE_DESCRIPTION("ARM GDC Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ARM-GDC");
