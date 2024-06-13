// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"

#include "vse_drv.h"

#define VSE_SINK_ONLINE_PATH_MAX (4)

static int scene;
module_param(scene, int, 0644);

static struct vse_stitching stitchesx[] = {
	{ false },
	{ false },
	{ false },
	{ false },
	{ false },
	{ false },
};

static struct vse_stitching stitches9[] = {
	{ true, 2, 2, 5, 0, 1 },
	{ true, 2, 2, 5, 1, 0 },
	{ true, 2, 2, 5, 1, 1 },
	{ false },
	{ false },
	{ true, 2, 2, 5, 0, 0 },
};

static struct vse_stitching *stitches[] = {
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
	stitches9,
	stitchesx,
	stitchesx,
	stitchesx,
	stitchesx,
};

static struct cam_rect cropsx[] = {
	{}, {}, {}, {}, {}, {},
};

static struct cam_rect crops2[] = {
	{ 0, 0, 1280, 720 },
	{ 0, 0, 1280, 720 },
	{ 0, 0, 1280, 720 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 1280, 720 },
};

static struct cam_rect crops6[] = {
	{ 0, 0, 1280, 720 },
	{ 0, 0, 1280, 720 },
	{ 0, 0, 1280, 720 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 0, 0 },
};

static struct cam_rect crops9[] = {
	{ 780, 420, 360, 240 },
	{ 1238, 652, 682, 428 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 0, 0 },
	{ 0, 0, 260, 194 },
};

static struct cam_rect *crops[] = {
	cropsx,
	cropsx,
	crops2,
	cropsx,
	cropsx,
	cropsx,
	crops6,
	cropsx,
	cropsx,
	crops9,
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
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1920, 1080 },
	{ 1280,  720 },
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
	if (!*format)
		return -EINVAL;
	return 0;
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
	return -EINVAL;
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
		ctx->stitches[i] = stitches[scene][i];
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

static int count_channels(struct vse_v4l_instance *inst)
{
	int i, count = 0;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++)
		if (inst->is_out_chnl_connected[i])
			count++;
	return count;
}

static int vse_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vse_v4l_instance *vse = sd_to_vse_v4l_instance(sd);
	struct vse_irq_ctx ctx;
	u32 set_state_count = 0, out_chnl_connected_num;
	int rc;

	if (enable)
		set_state_count = vse->set_state_count + 1;
	else if (vse->set_state_count > 0)
		set_state_count = vse->set_state_count - 1;
	out_chnl_connected_num = count_channels(vse);

	if (enable) {
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
	} else {
		if (!set_state_count) {
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
		}

		memset(&ctx, 0, sizeof(ctx));
		vse_set_ctx(vse->dev, vse->id, &ctx);

		if (!vse->node.bctx.is_sink_online_mode)
			cam_reqbufs(&vse->sink_ctx, 0, NULL);
	}

	rc = vse_set_state(vse->dev, vse->id, enable, set_state_count, out_chnl_connected_num);
	if (rc < 0)
		return rc;

	if (enable && set_state_count >= out_chnl_connected_num) {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	}

	vse->set_state_count = set_state_count;
	return 0;
}

static int vse_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int vse_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int vse_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct vse_v4l_instance *inst = sd_to_vse_v4l_instance(sd);
	struct cam_format f;
	struct vse_stitching *stitch;
	struct v4l2_subdev_format s_f = *fmt;
	struct cam_rect crop;
	int channel = -1;
	int hfactor = 1, vfactor = 1;
	int rc, i;

	if (fmt->pad < inst->node.num_pads)
		channel = get_channel_index(inst, &inst->node.pads[fmt->pad]);

	if (channel < 0)
		return -EINVAL;

	stitch = stitches[scene];
	if (stitch[channel].enabled) {
		hfactor = stitch[channel].hfactor;
		vfactor = stitch[channel].vfactor;
	}

	memset(&f, 0, sizeof(f));
	f.format = CAM_FMT_NV12;
	f.width = iress[scene].width;
	f.height = iress[scene].height;
	f.stride = ALIGN(f.width, STRIDE_ALIGN);
	if (memcmp(&f, &inst->ifmt, sizeof(f))) {
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
	}

	f.width = ALIGN_DOWN(fmt->format.width / hfactor, 16);
	f.height = fmt->format.height / vfactor;
	f.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	crop = crops[scene][channel];
	if (inst->out_pixelformat)
		f.format = pixelformat_to_cam_format(inst->out_pixelformat);
	else
		f.format = mbus_code_to_cam_format(fmt->format.code);
	rc = vse_set_oformat(inst->dev, inst->id, channel, &f, &crop);
	if (rc < 0)
		return rc;

	if (!stitch[channel].enabled)
		return 0;

	if (WARN_ON(channel != stitch[channel].left_top))
		return 0;

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (stitch[i].enabled && i != channel) {
			crop = crops[scene][i];
			rc = vse_set_oformat(inst->dev, inst->id, i, &f, &crop);
			if (rc < 0)
				return rc;
		}
	}
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
	.video = &vse_video_ops,
	.pad = &vse_pad_ops,
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

		n->async_bound = vse_async_bound;

		n->bctx.ready = vse_buf_ready;
		n->bctx.qbuf = vse_qbuf;
		n->bctx.dqbuf = vse_dqbuf;
		n->bctx.trigger = vse_trigger;
		n->bctx.is_completed = vse_is_completed;
		n->bctx.get_format = vse_get_out_format;
		n->bctx.set_format = vse_set_out_format;
		n->bctx.enum_format = vse_enum_out_format;
		n->bctx.enum_framesize = vse_enum_out_framesize;
		n->bctx.enum_frameinterval = vse_enum_out_frameinterval;

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

	rc = vse_runtime_resume(dev);
		if (rc) {
		dev_err(dev, "failed to call vse_runtime_resume (err=%d)\n", rc);
		return rc;
	}

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

	rc = vse_runtime_suspend(dev);
		if (rc) {
		dev_err(dev, "failed to call vse_runtime_suspend (err=%d)\n", rc);
		return rc;
	}

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
