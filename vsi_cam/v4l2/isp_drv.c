// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "v4l2_usr_api.h"
#include "video_fmt.h"
#include "isp_drv.h"

static char input_fmt_str[16];
module_param_string(input_fmt, input_fmt_str, 16, 0644);

static char work_mode_str[16];
module_param_string(work_mode, work_mode_str, 16, 0644);

static bool hdr_en;
module_param(hdr_en, bool, 0644);

static bool metadata_en;
module_param(metadata_en, bool, 0644);

static ushort sub_chnl_width = 1920;
module_param(sub_chnl_width, ushort, 0644);

static ushort sub_chnl_height = 1080;
module_param(sub_chnl_height, ushort, 0644);

#define sub_sink_pad(i) (&(i)->node.pads[1])
#define src_pad(i)      (&(i)->node.pads[2])

#define is_sub_sink_pad(i, pad) \
	(sub_sink_pad(i)->index == pad)

#define sink_ctx(i)     (&(i)->sink_ctx[0])
#define sub_sink_ctx(i) (&(i)->sink_ctx[1])

static int get_src_pad_index(struct isp_v4l_instance *isp, u32 pad)
{
	int i;

	for (i = 0; i < ISP_OUT_CHNL_MAX; i++)
		if (isp->src_pads[i] && isp->src_pads[i]->index == pad)
			return i;
	return -1;
}

static int isp_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct isp_v4l_instance *isp;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	struct init_attr attr = {
		.en_reqbufs = true,
	}, *p_attr = NULL;
	int index, rc = 0;
	bool online;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	isp = sd_to_v4l_instance(isp, sd);
	attr.dev = sd->dev;

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &isp->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SINK) {
			if (sink_online_en(lctx) != rctx->src_online_en)
				return -EBUSY;
			if (is_sink_online_en(lctx))
				return 0;
			p_attr = &attr;
			online = false;
		} else {
			lctx->src_online_en = sink_online_en(rctx);
			online = sink_online_en(rctx);
		}
	} else {
		online = false;
	}

	if (local->flags & MEDIA_PAD_FL_SINK) {
		buf_ctx = &isp->sink_ctx[local->index];
	} else {
		index = get_src_pad_index(isp, local->index);
		if (index < 0)
			return -EINVAL;
		buf_ctx = &isp->src_ctx[index];
	}

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (buf_ctx->pad)
			return -EBUSY;

		rc = cam_ctx_init(buf_ctx, (void *)local, p_attr);
		if (rc < 0)
			return rc;
		buf_ctx->online = online;
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

static int isp_qbuf(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct isp_v4l_instance *isp = buf_ctx_to_v4l_instance(isp, ctx);
	int rc;

	if (!ctx || !buf)
		return -EINVAL;

	if (is_sub_sink_pad(isp, pad) && isp->metadata_en)
		return cam_qbuf(&isp->sink_ctx[pad], buf);

	if (is_sink_online_en(ctx))
		return -EBUSY;

	rc = cam_qbuf(&isp->sink_ctx[pad], buf);
	if (rc < 0)
		return rc;

	return isp_add_job(isp->dev, isp->id);
}

static int isp_drop(struct v4l2_buf_ctx *ctx, u32 pad, struct cam_buf *buf)
{
	struct isp_v4l_instance *isp = buf_ctx_to_v4l_instance(isp, ctx);

	if (!ctx || !buf)
		return -EINVAL;

	if (is_sub_sink_pad(isp, pad) && isp->metadata_en)
		return cam_drop(&isp->sink_ctx[pad], buf);

	if (is_sink_online_en(ctx))
		return -EBUSY;

	return cam_drop(&isp->sink_ctx[pad], buf);
}

static struct cam_buf *isp_dqbuf(struct v4l2_buf_ctx *ctx, u32 pad)
{
	struct isp_v4l_instance *isp = buf_ctx_to_v4l_instance(isp, ctx);

	if (!ctx)
		return NULL;

	if (is_sub_sink_pad(isp, pad) && isp->metadata_en)
		return cam_dqbuf(&isp->sink_ctx[pad]);

	if (is_sink_online_en(ctx))
		return NULL;

	return cam_dqbuf(&isp->sink_ctx[pad]);
}

static bool is_support_fmt(u32 fmt)
{
	switch (fmt) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return true;
	default:
		return false;
	}
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

static int check_work_mode_param(enum isp_work_mode *mode)
{
	size_t len = strlen(work_mode_str);
	int rc = 0;

	*mode = ISP_MODE_INVALID;
	if (!len)
		return -EINVAL;

	if (work_mode_str[len - 1] == '\n')
		work_mode_str[len - 1] = '\0';

	if (!strcmp(work_mode_str, "stream"))
		*mode = ISP_STRM_MODE;
	else if (!strcmp(work_mode_str, "mcm"))
		*mode = ISP_MCM_MODE;
	else
		rc = -EINVAL;

	return rc;
}

static u32 isp_get_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_format *format)
{
	return 0;
}

static inline int isp_set_sub_chnl_format(struct isp_v4l_instance *ins,
					  struct cam_format *f,
					  struct v4l2_format *fmt,
					  bool is_try)
{
	struct isp_instance *isp;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int rc;

	if (!ins->hdr_en && !ins->metadata_en)
		return 0;

	isp = &ins->dev->insts[ins->id];
	if (ins->metadata_en) {
		if (sub_chnl_width < MIN_W || sub_chnl_height < MIN_H ||
		    sub_chnl_width > MAX_W || sub_chnl_height > MAX_H) {
			pr_err("invalid size (%dx%d) for sub channel.\n",
			       sub_chnl_width, sub_chnl_height);
			return -EINVAL;
		}

		fmt->fmt.pix.width = sub_chnl_width;
		fmt->fmt.pix.height = sub_chnl_height;
	}

	pad = get_remote_pad_sd(sub_sink_pad(ins), &sd);
	if (!sd)
		return -ENOLINK;
	rc = v4l2_subdev_ctx_call(sd, set_format, pad->index, fmt, is_try);
	if (rc < 0) {
		pr_err("%s v4l2_subdev_ctx_call failed (err=%d)\n", __func__, rc);
		return rc;
	}
	isp->sub_ifmt = *f;
	return 0;
}

static int isp_set_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
			      struct v4l2_format *format, bool is_try)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);
	struct isp_instance *isp;
	struct isp_format f = {0};
	struct v4l2_format s_f = *format;
	struct isp_ctrl sen_ctrl = {0};
	struct cam_input in = {0};
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	int rc = 0;

	if (metadata_en && hdr_en) {
		pr_err("currently, metadata and hdr cannot be both enabled.\n");
		return -EINVAL;
	}

	inst->metadata_en = metadata_en;
	inst->hdr_en = hdr_en;

	sd = &inst->node.sd;
	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd)
		return -ENOLINK;

	f.ofmt.format = pixelformat_to_cam_format(format->fmt.pix.pixelformat);
	f.ifmt.width  = format->fmt.pix.width;
	f.ifmt.height = format->fmt.pix.height;
	f.ifmt.format = pixelformat_to_cam_format(inst->input_fmt);
	f.ifmt.stride = inst->fmt.ifmt.stride;
	f.ofmt.width  = format->fmt.pix.width;
	f.ofmt.height = format->fmt.pix.height;
	f.ofmt.stride = ALIGN(f.ofmt.width, STRIDE_ALIGN);
	mutex_lock(&inst->fmt_lock);

	if (inst->fmt_changed && !memcmp(&f, &inst->fmt, sizeof(f))) {
		rc = 0;
		goto _exit;
	}

	s_f.fmt.pix.pixelformat = inst->input_fmt;
	rc = v4l2_subdev_ctx_call(rsd, set_format, rpad->index, &s_f, is_try);
	if (rc < 0) {
		pr_err("%s v4l2_subdev_ctx_call failed (err=%d)\n", __func__, rc);
		goto _exit;
	}

	switch (f.ifmt.format) {
	case CAM_FMT_RAW8:
		f.ifmt.stride = format->fmt.pix.width;
		break;
	case CAM_FMT_RAW10:
	case CAM_FMT_RAW12:
		f.ifmt.stride = format->fmt.pix.width * 2;
		break;
	default:
		break;
	}

	rc = isp_set_sub_chnl_format(inst, &f.ifmt, &s_f, is_try);
	if (rc < 0)
		goto _exit;

	isp = &inst->dev->insts[inst->id];
	rc = check_work_mode_param(&inst->dev->mode);
	if (rc < 0)
		goto _exit;
	if (inst->dev->mode == ISP_MODE_INVALID) {
		pr_debug("set isp default input mode as MCM mode\n");
		inst->dev->mode = ISP_MCM_MODE;
	}
	if (inst->dev->mode != ISP_STRM_MODE) {
		if (is_sink_online_en(&inst->node.bctx)) {
			isp->online_mcm = true;
			rc = isp_set_stream_idx(inst->dev, inst->id, inst->id);
			if (rc < 0)
				goto _exit;
		} else {
			isp->online_mcm = false;
		}
	} else {
		isp->online_mcm = false;
	}
	pr_debug("isp inst%d online_mcm=%d, mode=%d, stream_idx=%d\n",
		 inst->id, isp->online_mcm, inst->dev->mode, inst->id);

	rc = isp_set_state(inst->dev, inst->id, CAM_STATE_INITED, V4L_GROUP);
	if (rc < 0) {
		pr_err("%s isp_set_state failed\n", __func__);
		goto _exit;
	}

	rc = isp_set_iformat(inst->dev, inst->id, &f.ifmt, &f.icrop, inst->hdr_en);
	if (rc < 0) {
		pr_err("%s isp_set_iformat failed\n", __func__);
		goto _exit;
	}
	rc = isp_set_oformat(inst->dev, inst->id, &f.ofmt);
	if (rc < 0) {
		pr_err("%s isp_set_oformat failed\n", __func__);
		goto _exit;
	}

	sen_ctrl.ctrl_id = V4L2_CID_SENSOR_NAME;
	rc = v4l2_subdev_call(sd, core, command, CAM_GET_CTRL, &sen_ctrl);
	if (rc < 0) {
		pr_err("%s v4l2_subdev_call failed\n", __func__);
		goto _exit;
	}

	in.index = inst->id;
	in.type = CAM_INPUT_SENSOR;
	snprintf(in.sens.name, sizeof(in.sens.name), "%s_%dx%d_tuning.json",
		 sen_ctrl.ctrl_data, f.ifmt.width, f.ifmt.height);
	rc = isp_set_input(inst->dev, inst->id, &in);
	if (rc < 0) {
		pr_err("%s isp_set_input failed\n", __func__);
		goto _exit;
	}

	inst->fmt = f;
	inst->fmt_changed = true;

_exit:
	mutex_unlock(&inst->fmt_lock);
	return rc;

}

static int isp_enum_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad, u32 index, u32 *format)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);

	if (index >= inst->fmt_cap_num)
		return -EINVAL;

	*format = inst->fmt_cap[index];

	return 0;
}

static int isp_enum_ctx_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	struct v4l2_frmsizeenum fse;
	int rc;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd)
		return -ENOLINK;

	memcpy(&fse, fsize, sizeof(fse));
	fse.pixel_format = inst->input_fmt;
	rc = v4l2_subdev_ctx_call(rsd, enum_framesize, rpad->index, &fse);
	if (rc < 0)
		return rc;

	fsize->type = fse.type;
	fsize->discrete = fse.discrete;
	fsize->stepwise = fse.stepwise;
	return rc;
}

static int isp_enum_ctx_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	struct v4l2_frmivalenum fiv;
	int rc;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd)
		return -ENOLINK;

	memcpy(&fiv, fival, sizeof(fiv));
	fiv.pixel_format = inst->input_fmt;
	rc = v4l2_subdev_ctx_call(rsd, enum_frameinterval, rpad->index, &fiv);
	if (rc < 0)
		return rc;

	fival->type = fiv.type;
	fival->discrete = fiv.discrete;
	fival->stepwise = fiv.stepwise;
	return rc;
}

static void isp_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;
	int i, j = 0, rc;
	u32 input_cam_fmt, pixelformat;

	rpad = get_remote_pad_sd(sink_pad(inst), &rsd);
	if (!rsd)
		return;

	v4l2_subdev_ctx_call_no_return(rsd, set_cap);
	memset(inst->input_fmt_cap, 0, sizeof(inst->input_fmt_cap));

	for (i = 0; i < ARRAY_SIZE(inst->input_fmt_cap); i++) {
		rc = v4l2_subdev_ctx_call(rsd, enum_format, rpad->index, i, &pixelformat);
		if (rc < 0)
			break;
		if (!is_support_fmt(pixelformat))
			continue;
		inst->input_fmt_cap[j] = pixelformat;
		j++;
	}

	inst->input_fmt_cap_num = j;
	inst->fmt_cap[0] = V4L2_PIX_FMT_NV12;
	inst->fmt_cap_num = 1;
	rc = check_input_fmt_param(&input_cam_fmt);
	if (rc < 0)
		goto default_fmt;

	for (i = 0; i < j; i++) {
		if (pixelformat_to_cam_format(inst->input_fmt_cap[i]) == input_cam_fmt) {
			inst->input_fmt = inst->input_fmt_cap[i];
			return;
		}
	}

default_fmt:
	inst->input_fmt = inst->input_fmt_cap[0];
}

static int isp_map_info(struct v4l2_buf_ctx *ctx, u32 *devid, u32 *insid)
{
	struct isp_v4l_instance *isp = buf_ctx_to_v4l_instance(isp, ctx);

	if (!devid || !insid)
		return -EINVAL;

	*devid = isp->dev->id;
	*insid = isp->id;

	return 0;
}

static int isp_check_datapath(struct v4l2_buf_ctx *ctx, bool *online)
{
	struct isp_v4l_instance *inst = buf_ctx_to_v4l_instance(isp, ctx);
	struct isp_instance *isp;

	isp = &inst->dev->insts[inst->id];
	if (inst->dev->mode == ISP_STRM_MODE)
		*online = true;
	else if (inst->dev->mode == ISP_MCM_MODE)
		*online = isp->online_mcm ? true : false;
	else
		*online = false;

	return 0;
}

static int isp_queue_setup(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct isp_v4l_instance *ins;
	struct isp_instance *isp;
	unsigned int size = 0, index;

	if (!ctx)
		return -EINVAL;

	index = ctx->pad->index;
	pr_debug("isp input channel: %d\n", index);
	if (index >= ISP_IN_CHNL_MAX)
		return -EINVAL;

	ins = container_of(ctx, struct isp_v4l_instance, sink_ctx[index]);
	isp = &ins->dev->insts[ins->id];
	size = get_framebuf_size(&isp->fmt.ifmt);

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

static int isp_sub_chnl_queue_setup(struct cam_ctx *ctx,
				    unsigned int *num_buffers,
				    unsigned int *num_planes,
				    unsigned int sizes[],
				    struct device *alloc_devs[])
{
	struct isp_v4l_instance *ins;
	struct isp_instance *isp;
	unsigned int size = 0, index;

	if (!ctx)
		return -EINVAL;

	index = ctx->pad->index;
	pr_debug("isp input channel: %d\n", index);
	if (index >= ISP_IN_CHNL_MAX)
		return -EINVAL;

	ins = container_of(ctx, struct isp_v4l_instance, sink_ctx[index]);
	if (ins) {
		isp = &ins->dev->insts[ins->id];
		size = get_framebuf_size(&isp->sub_ifmt);
	}

	if (!size)
		return -ENOMEM;

	if (!*num_buffers)
		*num_buffers = 1;

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static struct cam_buf_ops isp_sub_chnl_buf_ops = {
	.queue_setup = isp_sub_chnl_queue_setup,
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
	memcpy(name, source, strlen(source) + 1);
	return 0;
}

static long isp_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_v4l_instance *isp = sd_to_v4l_instance(isp, sd);
	struct v4l2_query_ext_ctrl *qectrl;
	struct v4l2_ext_control *vectl;
	int rc = 0;

	switch (cmd) {
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

static void fill_irq_ctx(struct isp_v4l_instance *isp, u32 i, int enable,
			 struct isp_irq_ctx *ctx)
{
	ctx->sink_online_en = sink_online_en(&isp->node.bctx);
	if (sink_ctx(isp)->pad)
		ctx->sink_ctx = sink_ctx(isp);
	if (isp->src_ctx[i].pad) {
		if (enable) {
			ctx->src_ctx[i] = &isp->src_ctx[i];
			if (isp->src_ctx[i].online)
				set_online(ctx->src_online_stat, i);
			else
				set_offline(ctx->src_online_stat, i);
		} else {
			ctx->src_ctx[i] = NULL;
			set_online(ctx->src_online_stat, i); /* set back to default */
		}
	}
}

static int isp_set_stream(struct v4l2_buf_ctx *ctx, u32 pad, int enable)
{
	struct isp_v4l_instance *isp = buf_ctx_to_v4l_instance(isp, ctx);
	struct isp_irq_ctx irq_ctx;
	int index = get_src_pad_index(isp, pad);
	int rc;

	if (pad >= isp->node.num_pads || index < 0)
		return -EINVAL;

	rc = isp_get_ctx(isp->dev, isp->id, &irq_ctx);
	if (rc < 0)
		return rc;
	fill_irq_ctx(isp, index, enable, &irq_ctx);
	return isp_set_ctx(isp->dev, isp->id, &irq_ctx);
}

static int isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_v4l_instance *isp = sd_to_v4l_instance(isp, sd);
	u32 devid, insid;
	int rc = 0;

	if (cam_refcount_check(&isp->start_count, enable))
		return rc;

	if (enable) {
		if (is_sink_online_en(&isp->node.bctx)) {
			rc = get_front_info(sink_pad(isp), &devid, &insid);
			if (rc < 0)
				return rc;
			rc = isp_set_input_select(isp->dev, isp->id, devid, insid);
		} else {
			rc = cam_reqbufs(sink_ctx(isp), V4L2_SUBDEV_BUF_NUM, &isp_buf_ops);
		}
		if (rc < 0)
			return rc;
		if (isp->metadata_en) {
			rc = cam_reqbufs(sub_sink_ctx(isp), V4L2_SUBDEV_BUF_NUM,
					&isp_sub_chnl_buf_ops);
			if (rc < 0)
				return rc;
		}
		rc = isp_set_state(isp->dev, isp->id, CAM_STATE_STARTED, V4L_GROUP);
		if (rc < 0)
			return rc;

		rc = subdev_set_stream(sd, enable);
	} else {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;

		rc = isp_set_state(isp->dev, isp->id, CAM_STATE_STOPPED, V4L_GROUP);
		if (rc < 0)
			return rc;
		if (is_sink_online_en(&isp->node.bctx))
			rc = isp_set_stream_idx(isp->dev, isp->id, -1);
		else
			rc = cam_reqbufs(sink_ctx(isp), 0, NULL);
		if (rc < 0)
			return rc;
		if (isp->metadata_en)
			rc = cam_reqbufs(sub_sink_ctx(isp), 0, NULL);
		isp->fmt_changed = false;
	}
	return rc;
}

static int isp_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct isp_v4l_instance *isp = sd_to_v4l_instance(isp, sd);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rpad = get_remote_pad_sd(sink_pad(isp), &rsd);
	if (!rsd)
		return -ENOLINK;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, g_frame_interval, fiv);
}

static int isp_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct isp_v4l_instance *isp = sd_to_v4l_instance(isp, sd);
	struct v4l2_subdev *rsd;
	struct media_pad *rpad;

	rpad = get_remote_pad_sd(sink_pad(isp), &rsd);
	if (!rsd)
		return -ENOLINK;

	fiv->pad = rpad->index;

	return v4l2_subdev_call(rsd, video, s_frame_interval, fiv);
}

static const struct v4l2_subdev_core_ops isp_core_ops = {
	.command = isp_command,
};

static const struct v4l2_subdev_video_ops isp_video_ops = {
	.s_stream = isp_s_stream,
	.g_frame_interval = isp_g_frame_interval,
	.s_frame_interval = isp_s_frame_interval,
};

static const struct v4l2_subdev_ops isp_subdev_ops = {
	.video = &isp_video_ops,
	.core = &isp_core_ops,
};

static int isp_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isp_v4l_instance *inst = sd_to_v4l_instance(isp, sd);
	int rc = 0;

	mutex_lock(&inst->open_lock);
	if (cam_refcount_check(&inst->open_count, true))
		goto _exit;

	rc = subdev_open(sd);
	if (rc < 0)
		goto _exit;
	rc = isp_open(inst->dev, inst->id);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
}

static int isp_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isp_v4l_instance *inst = sd_to_v4l_instance(isp, sd);
	int rc;

	mutex_lock(&inst->open_lock);
	if (cam_refcount_check(&inst->open_count, false))
		goto _exit;

	rc = subdev_close(sd);
	if (rc < 0)
		goto _exit;
	rc = isp_close(inst->dev, inst->id, V4L_GROUP);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
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
	u32 i = 0, j;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(isp->dev, struct isp_v4l_device, isp_dev);

	while (i < isp->dev->num_insts) {
		ins = &v4l_dev->insts[i];
		for (j = 0; j < ISP_IN_CHNL_MAX; j++)
			cam_ctx_release(&ins->sink_ctx[j]);
		for (j = 0; j < ISP_OUT_CHNL_MAX; j++)
			cam_ctx_release(&ins->src_ctx[j]);

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
	j = 0;
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

		mutex_init(&inst->open_lock);
		mutex_init(&inst->fmt_lock);
		refcount_set(&inst->start_count, REFCNT_INIT_VAL);
		refcount_set(&inst->open_count, REFCNT_INIT_VAL);

		n->async_bound = isp_async_bound;

		n->bctx.ready = isp_buf_ready;
		n->bctx.qbuf = isp_qbuf;
		n->bctx.dqbuf = isp_dqbuf;
		n->bctx.drop = isp_drop;
		n->bctx.get_format = isp_get_ctx_format;
		n->bctx.set_format = isp_set_ctx_format;
		n->bctx.enum_format = isp_enum_ctx_format;
		n->bctx.enum_framesize = isp_enum_ctx_framesize;
		n->bctx.enum_frameinterval = isp_enum_ctx_frameinterval;
		n->bctx.set_stream = isp_set_stream;
		n->bctx.set_cap = isp_set_cap;
		n->bctx.map_info = isp_map_info;
		n->bctx.check_datapath = isp_check_datapath;

		n->dev = dev;
		n->num_pads = (i < ISP_SINK_ONLINE_PATH_MAX) ? 4 : 3;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			isp_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		sink_pad(inst)->flags = MEDIA_PAD_FL_SINK;
		sub_sink_pad(inst)->flags = MEDIA_PAD_FL_SINK;
		src_pad(inst)->flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;
		inst->src_pads[0] = src_pad(inst);
		if (i < ISP_SINK_ONLINE_PATH_MAX) {
			n->pads[3].flags = MEDIA_PAD_FL_SOURCE;
			inst->src_pads[1] = &n->pads[3];
		}

		rc = subdev_init(n, ISP_DEV_NAME, v4l_dev->isp_dev.id,
				 i, &isp_subdev_ops, &isp_media_ops);
		if (rc < 0) {
			isp_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &isp_internal_ops;

		if (i < ISP_SINK_ONLINE_PATH_MAX)
			sink_online_en(&n->bctx) = true;
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
	u32 i, j;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->isp_dev.num_insts; i++) {
		for (j = 0; j < ISP_IN_CHNL_MAX; j++)
			cam_ctx_release(&v4l_dev->insts[i].sink_ctx[j]);
		for (j = 0; j < ISP_OUT_CHNL_MAX; j++)
			cam_ctx_release(&v4l_dev->insts[i].src_ctx[j]);
		subdev_deinit(&v4l_dev->insts[i].node);
		devm_kfree(dev, v4l_dev->insts[i].node.pads);
	}
	devm_kfree(dev, v4l_dev->insts);

	rc = isp_remove(pdev, &v4l_dev->isp_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call isp_remove (err=%d)\n", rc);
		return rc;
	}

#ifdef CONFIG_DEBUG_FS
	isp_debugfs_remo(&v4l_dev->isp_dev);
#endif
	devm_kfree(dev, v4l_dev);

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
