// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "cam_uapi.h"
#include "v4l2_usr_api.h"
#include "isp_drv.h"

#define ISP_OFFLINE_IN_BUF_NUM (4)

#define sd_to_isp_v4l_instance(s) \
({ \
	struct subdev_node *sn = container_of(s, struct subdev_node, sd); \
	container_of(sn, struct isp_v4l_instance, node); \
})

#define buf_ctx_to_isp_v4l_instance(ctx) \
({ \
	struct subdev_node *sn = container_of(ctx, struct subdev_node, bctx); \
	container_of(sn, struct isp_v4l_instance, node); \
})

static char input_fmt_str[16];
module_param_string(input_fmt, input_fmt_str, 16, 0644);

static int isp_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct isp_v4l_instance *isp;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	int rc = 0;
	bool has_internal_buf = false;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	isp = sd_to_isp_v4l_instance(sd);

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &isp->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SINK) {
			if (lctx->is_sink_online_mode != rctx->is_src_online_mode)
				return -EBUSY;
			if (lctx->is_sink_online_mode)
				return 0;
			has_internal_buf = !lctx->is_sink_online_mode;
		} else {
			lctx->is_src_online_mode = rctx->is_sink_online_mode;
		}
	}

	if (local->flags & MEDIA_PAD_FL_SINK)
		buf_ctx = &isp->sink_ctx;
	else
		buf_ctx = &isp->src_ctx;

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (buf_ctx->pad)
			return -EBUSY;

		rc = cam_ctx_init(buf_ctx, sd->dev, (void *)local,
				      has_internal_buf);
		if (rc < 0)
			return rc;
	} else {
		cam_ctx_release(buf_ctx);
	}
	return rc;
}

static const struct media_entity_operations isp_media_ops = {
	.link_setup = isp_link_setup,
};

static void isp_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
}

static int isp_qbuf(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct isp_v4l_instance *isp = buf_ctx_to_isp_v4l_instance(ctx);
	int rc;

	if (!ctx || !buf)
		return -EINVAL;

	if (ctx->is_sink_online_mode)
		return -EBUSY;

	rc = cam_qbuf(&isp->sink_ctx, buf);
	if (rc < 0)
		return rc;

	isp_add_job(isp->dev, isp->id);
	return 0;
}

static int isp_drop(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct isp_v4l_instance *isp = buf_ctx_to_isp_v4l_instance(ctx);

	if (!ctx || !buf)
		return -EINVAL;

	if (ctx->is_sink_online_mode)
		return -EBUSY;

	return cam_drop(&isp->sink_ctx, buf);
}

static struct cam_buf *isp_dqbuf(struct v4l2_buf_ctx *ctx)
{
	struct isp_v4l_instance *isp = buf_ctx_to_isp_v4l_instance(ctx);

	if (!ctx)
		return NULL;

	if (ctx->is_sink_online_mode)
		return NULL;

	return cam_dqbuf(&isp->sink_ctx);
}

static u32 isp_get_out_format(struct v4l2_buf_ctx *ctx)
{
	struct isp_v4l_instance *isp = buf_ctx_to_isp_v4l_instance(ctx);

	return isp->out_pixelformat;
}

static int isp_set_out_format(struct v4l2_buf_ctx *ctx, u32 format, bool is_try)
{
	struct isp_v4l_instance *isp = buf_ctx_to_isp_v4l_instance(ctx);

	if (!is_try)
		isp->out_pixelformat = format;
	return 0;
}

static int isp_enum_out_format(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct isp_v4l_instance *inst = buf_ctx_to_isp_v4l_instance(ctx);
	struct isp_instance *isp;

	if (!format)
		return -EINVAL;

	isp = &inst->dev->insts[inst->id];

	if (index >= ARRAY_SIZE(isp->fmt_cap))
		return -EINVAL;

	*format = cam_format_to_pixelformat
			(isp->fmt_cap[index].format, isp->input_bayer_format);
	return *format ? 0 : -EINVAL;
}

static int isp_enum_out_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct isp_v4l_instance *inst = buf_ctx_to_isp_v4l_instance(ctx);
	struct isp_instance *isp;
	struct isp_format_cap *cap = NULL;
	struct cam_res_cap *res;
	u32 i, format;

	isp = &inst->dev->insts[inst->id];

	for (i = 0; i < ARRAY_SIZE(isp->fmt_cap); i++) {
		format = cam_format_to_pixelformat
				(isp->fmt_cap[i].format, isp->input_bayer_format);
		if (format == fsize->pixel_format) {
			cap = &isp->fmt_cap[i];
			break;
		}
	}

	if (!cap)
		return -EINVAL;

	if (fsize->index >= ARRAY_SIZE(cap->res))
		return -EINVAL;

	res = &cap->res[fsize->index];
	if (res->type == CAP_DC) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = res->dc.width;
		fsize->discrete.height = res->dc.height;
	} else if (res->type == CAP_SW) {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = res->sw.min_width;
		fsize->stepwise.max_width = res->sw.max_width;
		fsize->stepwise.min_height = res->sw.min_height;
		fsize->stepwise.max_height = res->sw.max_height;
		fsize->stepwise.step_width = res->sw.step_width;
		fsize->stepwise.step_height = res->sw.step_height;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int isp_enum_out_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct isp_v4l_instance *inst = buf_ctx_to_isp_v4l_instance(ctx);
	struct isp_instance *isp;
	struct isp_format_cap *cap = NULL;
	struct cam_res_cap *res;
	bool found = false;
	u32 i, format;

	if (fival->index > 0)
		return -EINVAL;

	isp = &inst->dev->insts[inst->id];

	for (i = 0; i < ARRAY_SIZE(isp->fmt_cap); i++) {
		format = cam_format_to_pixelformat
				(isp->fmt_cap[i].format, isp->input_bayer_format);
		if (format == fival->pixel_format) {
			cap = &isp->fmt_cap[i];
			break;
		}
	}

	if (!cap)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(cap->res); i++) {
		res = &cap->res[i];
		if (res->type == CAP_DC) {
			if (res->dc.width == fival->width &&
			    res->dc.height == fival->height) {
				found = true;
				break;
			}
		}
	}

	if (!found)
		return -EINVAL;

	fival->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fival->discrete.numerator = inst->out_fps.numerator;
	fival->discrete.denominator = inst->out_fps.denominator;
	return 0;
}

static void isp_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct isp_v4l_instance *inst = buf_ctx_to_isp_v4l_instance(ctx);
	struct isp_instance *ins;
	struct isp_format_cap *cap;
	struct cam_res_cap *res;
	struct v4l2_subdev *sd;
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev_state state;
	uint32_t support_fmt = CAM_FMT_NV12;
	int i, rc;

	ins = &inst->dev->insts[inst->id];
	sd = &inst->node.sd;
	ins->input_bayer_format = BAYER_FMT_BGGR;

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));
	cap = &ins->fmt_cap[0];
	cap->format = support_fmt;

	for (i = 0; i < ARRAY_SIZE(cap->res); i++) {
		res = &cap->res[i];
		memset(&state, 0, sizeof(state));
		memset(&fse, 0, sizeof(fse));
		fse.index = i;
		fse.code = cam_format_to_mbus_code(CAM_FMT_RAW8, ins->input_bayer_format);
		rc = sd->ops->pad->enum_frame_size(sd, &state, &fse);
		if (rc < 0)
			break;
		res->type = CAP_DC;
		res->dc.width = fse.min_width;
		res->dc.height = fse.min_height;
	}
}

static void fill_irq_ctx(struct isp_v4l_instance *isp, struct isp_irq_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->is_sink_online_mode = isp->node.bctx.is_sink_online_mode;
	ctx->is_src_online_mode = isp->node.bctx.is_src_online_mode;
	if (isp->sink_ctx.pad)
		ctx->sink_ctx = &isp->sink_ctx;
	if (isp->src_ctx.pad)
		ctx->src_ctx = &isp->src_ctx;
}

static int isp_queue_setup(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct isp_v4l_instance *ins = container_of(ctx, struct isp_v4l_instance, sink_ctx);
	struct isp_instance *isp;
	unsigned int size = 0;

	if (ins) {
		isp = &ins->dev->insts[ins->id];
		size = get_framebuf_size(&isp->fmt.ifmt);
	}

	if (!size)
		return -ENOMEM;

	if (!*num_buffers)
		*num_buffers = 1;

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static struct cam_buf_ops isp_buf_ops = {
	.queue_setup = isp_queue_setup,
};

static int isp_s_ctrl(struct isp_v4l_instance *isp, void *arg)
{
	struct v4l2_ext_control *ext_ctrl = (struct v4l2_ext_control *)arg;
	u32 size = 0;

	switch (ext_ctrl->id) {
		case V4L2_CID_DR_EXPOSURE:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case V4L2_CID_DR_AWB:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		default:
			return -EINVAL;
	}
	return isp_set_subctrl(isp->dev, isp->id, ext_ctrl->id,
				       (void *)ext_ctrl->ptr, size);
}

static int isp_g_ctrl(struct isp_v4l_instance *isp, void *arg)
{
	struct v4l2_ext_control *ext_ctrl = (struct v4l2_ext_control *)arg;
	u32 size = 0;

	switch (ext_ctrl->id) {
		case V4L2_CID_DR_EXPOSURE:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case V4L2_CID_DR_AWB:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		default:
			return -EINVAL;
	}
	return isp_get_subctrl(isp->dev, isp->id, ext_ctrl->id,
				       (void *)ext_ctrl->ptr, size);
}

static int get_name_for_ext_ctrl(uint32_t id, char *name)
{
	const char *source;

	switch (id) {
		case V4L2_CID_DR_EXPOSURE:
			source = "hbn_isp_exposure_attr_t";
			break;
		case V4L2_CID_DR_AWB:
			source = "hbn_isp_awb_attr_t";
			break;
		default:
			return -1;
	}
	memcpy(name, source, strlen(source)+1);
	return 0;
}

static long isp_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_v4l_instance *isp = sd_to_isp_v4l_instance(sd);
	struct v4l2_query_ext_ctrl *qectrl;
	struct v4l2_ext_control *vectl;
	int rc = 0;

	switch(cmd) {
		case CAM_SET_CTRL:
		case CAM_GET_CTRL:
		case CAM_QUERY_CTRL:
			rc = subdev_call_command(sd, cmd, arg);
			break;
		case CAM_SET_EXT_CTRL:
			vectl = (struct v4l2_ext_control *)arg;
			switch (vectl->id) {
				case V4L2_CID_DR_EXPOSURE:
				case V4L2_CID_DR_AWB:
					rc = isp_s_ctrl(isp, arg);
					break;
				default:
					rc = subdev_call_command(sd, cmd, arg);
					break;
			}
			break;
		case CAM_GET_EXT_CTRL:
			vectl = (struct v4l2_ext_control *)arg;
			switch (vectl->id) {
				case V4L2_CID_DR_EXPOSURE:
				case V4L2_CID_DR_AWB:
					rc = isp_g_ctrl(isp, arg);
					break;
				default:
					rc = subdev_call_command(sd, cmd, arg);
					break;
			}
			break;
		case CAM_QUERY_EXT_CTRL:
			qectrl = (struct v4l2_query_ext_ctrl *)arg;
			switch (qectrl->id) {
				case V4L2_CID_DR_EXPOSURE:
				case V4L2_CID_DR_AWB:
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

static int isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_v4l_instance *isp = sd_to_isp_v4l_instance(sd);
	struct isp_irq_ctx ctx;
	int rc;

	if (enable) {
		if (!isp->node.bctx.is_sink_online_mode)
			cam_reqbufs(&isp->sink_ctx, ISP_OFFLINE_IN_BUF_NUM, &isp_buf_ops);
		else
			isp_set_input_select(isp->dev, isp->id, isp->id, 0);  //FIXME
		fill_irq_ctx(isp, &ctx);
		isp_set_ctx(isp->dev, isp->id, &ctx);
	} else {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	}

	rc = isp_set_state(isp->dev, isp->id, enable ? CAM_STATE_STARTED : CAM_STATE_STOPPED);
	if (rc < 0)
		return rc;

	if (enable) {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	} else {
		memset(&ctx, 0, sizeof(ctx));
		isp_set_ctx(isp->dev, isp->id, &ctx);

		if (!isp->node.bctx.is_sink_online_mode)
			cam_reqbufs(&isp->sink_ctx, 0, NULL);
		else
			isp_set_stream_idx(isp->dev, isp->id, -1);
	}
	return 0;
}

static int isp_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct isp_v4l_instance *ins = sd_to_isp_v4l_instance(sd);

	fiv->interval = ins->out_fps;
	return 0;
}

static int isp_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct isp_v4l_instance *ins = sd_to_isp_v4l_instance(sd);

	fiv->interval = ins->out_fps;
	return 0;
}

static int check_input_fmt_param(enum cam_format_type *fmt)
{
	size_t len = strlen(input_fmt_str);
	int rc = 0;

	if (!len)
		return -EINVAL;

	if (input_fmt_str[len - 1] == '\n')
		input_fmt_str[len - 1] = 0;
	if (!strcmp(input_fmt_str, "raw8"))
		*fmt = CAM_FMT_RAW8;
	else if (!strcmp(input_fmt_str, "raw10"))
		*fmt = CAM_FMT_RAW10;
	else if (!strcmp(input_fmt_str, "raw12"))
		*fmt = CAM_FMT_RAW12;
	else if (!strcmp(input_fmt_str, "raw16"))
		*fmt = CAM_FMT_RAW16;
	else
		rc = -EINVAL;
	pr_info("got input format from param: %s\n", input_fmt_str);
	return rc;
}

static int isp_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct isp_v4l_instance *inst = sd_to_isp_v4l_instance(sd);
	struct isp_instance *isp;
	struct isp_format f;
	struct v4l2_subdev_format s_f = *fmt;
	enum cam_format_type in_fmt_preferred_list[] = {
		CAM_FMT_RAW8, CAM_FMT_RAW10, CAM_FMT_RAW12
	};
	int rc, i;

	memset(&f, 0, sizeof(f));

	if (inst->out_pixelformat)
		f.ofmt.format = pixelformat_to_cam_format(inst->out_pixelformat);
	else
		f.ofmt.format = mbus_code_to_cam_format(fmt->format.code);

	if (f.ofmt.format != CAM_FMT_NV12)
		return -EINVAL;

	isp = &inst->dev->insts[inst->id];
	rc = check_input_fmt_param(&f.ifmt.format);
	if (!rc) {
		s_f.format.code = cam_format_to_mbus_code
				(f.ifmt.format, isp->input_bayer_format);
		rc = subdev_set_fmt(sd, state, &s_f);
		if (rc < 0)
			return rc;
	} else {
		for (i = 0; i < ARRAY_SIZE(in_fmt_preferred_list); i++) {
			f.ifmt.format = in_fmt_preferred_list[i];
			s_f.format.code = cam_format_to_mbus_code
					(f.ifmt.format, isp->input_bayer_format);
			rc = subdev_set_fmt(sd, state, &s_f);
			if (!rc)
				break;
			if (rc < 0 && rc != -EINVAL)
				return rc;
		}
	}

	switch (f.ifmt.format) {
	case CAM_FMT_RAW8:
		f.ifmt.stride = fmt->format.width;
		break;
	case CAM_FMT_RAW10:
	case CAM_FMT_RAW12:
		f.ifmt.stride = fmt->format.width * 2;
		break;
	default:
		break;
	}

	inst->dev->mode = ISP_MCM_MODE;
	if (inst->id < ISP_SINK_ONLINE_PATH_MAX) {
		isp->online_mcm = true;
		isp_set_stream_idx(inst->dev, inst->id, inst->id);
	} else {
		isp->online_mcm = false;
	}
	pr_debug("isp inst%d online_mcm=%d, mode=%d, stream_idx=%d\n",
		 inst->id, isp->online_mcm, inst->dev->mode, inst->id);

	// FIXME
	isp_set_state(inst->dev, inst->id, CAM_STATE_INITED);

	f.ifmt.width  = fmt->format.width;
	f.ifmt.height = fmt->format.height;
	f.ifmt.stride = ALIGN(f.ifmt.stride, STRIDE_ALIGN);
	f.ofmt.width  = fmt->format.width;
	f.ofmt.height = fmt->format.height;
	f.ofmt.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	rc = isp_set_format(inst->dev, inst->id, &f);
	if (rc < 0)
		return rc;

	struct isp_ctrl sen_ctrl = {0};
	sen_ctrl.ctrl_id = V4L2_CID_SENSOR_NAME;
	v4l2_subdev_call(sd, core, command, CAM_GET_CTRL, &sen_ctrl);

	struct cam_input in;
	memset(&in, 0, sizeof(in));
	in.index = inst->id;
	in.type = CAM_INPUT_SENSOR;
	snprintf(in.sens.name, sizeof(in.sens.name), "%s_%dx%d_tuning.json",
		 sen_ctrl.ctrl_data, f.ifmt.width, f.ifmt.height);
	return isp_set_input(inst->dev, inst->id, &in);
}

static int isp_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int isp_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int isp_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	int rc;
	rc = subdev_enum_frame_size(sd, state, fse);
	if (rc < 0)
		return rc;
	return 0;
}

static int isp_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static const struct v4l2_subdev_core_ops isp_core_ops = {
	.command = isp_command,
};

static const struct v4l2_subdev_video_ops isp_video_ops = {
	.s_stream = isp_s_stream,
	.g_frame_interval = isp_g_frame_interval,
	.s_frame_interval = isp_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops isp_pad_ops = {
	.set_fmt = isp_set_fmt,
	.get_fmt = isp_get_fmt,
	.enum_mbus_code = isp_enum_mbus_code,
	.enum_frame_size = isp_enum_frame_size,
	.enum_frame_interval = isp_enum_frame_interval,
};

static const struct v4l2_subdev_ops isp_subdev_ops = {
	.video = &isp_video_ops,
	.pad = &isp_pad_ops,
	.core = &isp_core_ops,
};

static int isp_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isp_v4l_instance *inst = sd_to_isp_v4l_instance(sd);
	int rc;

	rc = subdev_open(sd);
	if (rc < 0)
		return rc;

	return isp_open(inst->dev, inst->id);
}

static int isp_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isp_v4l_instance *inst = sd_to_isp_v4l_instance(sd);
	int rc;

	rc = subdev_close(sd);
	if (rc < 0)
		return rc;

	return isp_close(inst->dev, inst->id);
}

static const struct v4l2_subdev_internal_ops isp_internal_ops = {
	.open = isp_v4l_open,
	.close = isp_v4l_close,
};

static void isp_inst_remove(struct isp_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int isp_async_bound(struct subdev_node *sn)
{
	struct isp_v4l_instance *isp =
			container_of(sn, struct isp_v4l_instance, node);
	struct isp_v4l_instance *ins;
	struct isp_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(isp->dev, struct isp_v4l_device, isp_dev);

	while (i < isp->dev->num_insts) {
		ins = &v4l_dev->insts[i];
		cam_ctx_release(&ins->sink_ctx);
		cam_ctx_release(&ins->src_ctx);

		if (ins != isp) {
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

		if (ins != isp)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int isp_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct isp_v4l_device *v4l_dev;
	struct isp_v4l_instance *insts;
	u32 i;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = isp_probe(pdev, &v4l_dev->isp_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call isp_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->isp_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->isp_dev.num_insts; i++) {
		struct isp_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->isp_dev;
		inst->out_fps.numerator = 30;
		inst->out_fps.denominator = 1;

		n->async_bound = isp_async_bound;

		n->bctx.ready = isp_buf_ready;
		n->bctx.qbuf = isp_qbuf;
		n->bctx.dqbuf = isp_dqbuf;
		n->bctx.drop = isp_drop;
		n->bctx.get_format = isp_get_out_format;
		n->bctx.set_format = isp_set_out_format;
		n->bctx.enum_format = isp_enum_out_format;
		n->bctx.enum_framesize = isp_enum_out_framesize;
		n->bctx.enum_frameinterval = isp_enum_out_frameinterval;
		n->bctx.set_cap = isp_set_cap;

		n->dev = dev;
		if (i < ISP_SINK_ONLINE_PATH_MAX)
			n->num_pads = 3;
		else
			n->num_pads = 2;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			isp_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;
		if (i < ISP_SINK_ONLINE_PATH_MAX)
			n->pads[2].flags = MEDIA_PAD_FL_SOURCE;

		rc = subdev_init(n, ISP_DEV_NAME, v4l_dev->isp_dev.id,
				 i, &isp_subdev_ops, &isp_media_ops);
		if (rc < 0) {
			isp_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &isp_internal_ops;

		if (i < ISP_SINK_ONLINE_PATH_MAX)
			n->bctx.is_sink_online_mode = true;
	}

	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		isp_inst_remove(insts, v4l_dev->isp_dev.num_insts);
		isp_remove(pdev, &v4l_dev->isp_dev);
		return rc;
	}

	platform_set_drvdata(pdev, &v4l_dev->isp_dev);

#ifdef CONFIG_DEBUG_FS
	isp_debugfs_init(&v4l_dev->isp_dev);
#endif

	if (v4l_dev->isp_dev.axi)
		dev_dbg(dev, "axi clock: %ld Hz\n", clk_get_rate(v4l_dev->isp_dev.axi));
	if (v4l_dev->isp_dev.core)
		dev_dbg(dev, "core clock: %ld Hz\n", clk_get_rate(v4l_dev->isp_dev.core));
	if (v4l_dev->isp_dev.mcm)
		dev_dbg(dev, "mcm clock: %ld Hz\n", clk_get_rate(v4l_dev->isp_dev.mcm));
	if (v4l_dev->isp_dev.hclk)
		dev_dbg(dev, "hclk clock: %ld Hz\n", clk_get_rate(v4l_dev->isp_dev.hclk));

	dev_dbg(dev, "VS ISP driver (v4l) probed done\n");
	return 0;
}

static int isp_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct isp_device *isp_dev = platform_get_drvdata(pdev);
	struct isp_v4l_device *v4l_dev =
			container_of(isp_dev, struct isp_v4l_device, isp_dev);
	int rc;
	u32 i;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->isp_dev.num_insts; i++) {
		cam_ctx_release(&v4l_dev->insts[i].sink_ctx);
		cam_ctx_release(&v4l_dev->insts[i].src_ctx);
		subdev_deinit(&v4l_dev->insts[i].node);
	}

	rc = isp_remove(pdev, &v4l_dev->isp_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call isp_remove (err=%d)\n", rc);
		return rc;
	}

#ifdef CONFIG_DEBUG_FS
	isp_debugfs_remo(&v4l_dev->isp_dev);
#endif

	dev_dbg(dev, "VS ISP driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops isp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isp_system_suspend, isp_system_resume)
	SET_RUNTIME_PM_OPS(isp_runtime_suspend, isp_runtime_resume, NULL)
};

static const struct of_device_id isp_of_match[] = {
	{ .compatible = ISP_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, isp_of_match);

static struct platform_driver isp_driver = {
	.probe  = isp_v4l_probe,
	.remove = isp_v4l_remove,
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
