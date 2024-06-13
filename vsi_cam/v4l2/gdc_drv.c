// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "cam_uapi.h"

#include "gdc_drv.h"

#define sd_to_gdc_v4l_instance(s) \
({ \
	struct subdev_node *sn = container_of(s, struct subdev_node, sd); \
	container_of(sn, struct gdc_v4l_instance, node); \
})

#define buf_ctx_to_gdc_v4l_instance(ctx) \
({ \
	struct subdev_node *sn = container_of(ctx, struct subdev_node, bctx); \
	container_of(sn, struct gdc_v4l_instance, node); \
})

static int gdc_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct gdc_v4l_instance *gdc;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	int rc = 0;
	bool has_internal_buf = false;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	gdc = sd_to_gdc_v4l_instance(sd);

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &gdc->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SINK) {
			if (lctx->is_sink_online_mode != rctx->is_src_online_mode)
				return -EBUSY;
			if (lctx->is_sink_online_mode)
				return -EBUSY;
			has_internal_buf = true;
		} else {
			lctx->is_src_online_mode = rctx->is_sink_online_mode;
			if (lctx->is_src_online_mode)
				return -EBUSY;
		}
	}

	if (local->flags & MEDIA_PAD_FL_SINK)
		buf_ctx = &gdc->sink_ctx;
	else
		buf_ctx = &gdc->src_ctx;

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

static const struct media_entity_operations gdc_media_ops = {
	.link_setup = gdc_link_setup,
};

static void gdc_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
}

static int gdc_qbuf(struct v4l2_buf_ctx *ctx, struct cam_buf *buf)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_gdc_v4l_instance(ctx);
	int rc;

	if (!ctx || !buf)
		return -EINVAL;

	if (ctx->is_sink_online_mode)
		return -EBUSY;

	rc = cam_qbuf(&gdc->sink_ctx, buf);
	if (rc < 0)
		return rc;

	return gdc_add_job(gdc->dev, gdc->id);
}

static struct cam_buf *gdc_dqbuf(struct v4l2_buf_ctx *ctx)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_gdc_v4l_instance(ctx);

	if (!ctx)
		return NULL;

	if (ctx->is_sink_online_mode)
		return NULL;

	return cam_dqbuf(&gdc->sink_ctx);
}

static u32 gdc_get_out_format(struct v4l2_buf_ctx *ctx)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_gdc_v4l_instance(ctx);

	return gdc->out_pixelformat;
}

static int gdc_set_remote_format(struct v4l2_subdev *sd, u32 pixelformat, bool is_try)
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

static int gdc_set_out_format(struct v4l2_buf_ctx *ctx, u32 format, bool is_try)
{
	struct gdc_v4l_instance *gdc = buf_ctx_to_gdc_v4l_instance(ctx);

	gdc_set_remote_format(&gdc->node.sd, format, is_try);

	if (!is_try)
		gdc->out_pixelformat = format;
	return 0;
}

static int gdc_enum_out_format(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_gdc_v4l_instance(ctx);
	struct gdc_instance *gdc;

	if (!format)
		return -EINVAL;

	gdc = &inst->dev->insts[inst->id];

	if (index >= ARRAY_SIZE(gdc->fmt_cap))
		return -EINVAL;

	*format = cam_format_to_pixelformat
			(gdc->fmt_cap[index].format, 0);
	if (!*format)
		return -EINVAL;
	return 0;
}

static int gdc_enum_out_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct gdc_v4l_instance *inst = buf_ctx_to_gdc_v4l_instance(ctx);
	struct gdc_instance *gdc;
	struct gdc_format_cap *cap = NULL;
	struct cam_res_cap *res;
	u32 i, format;

	gdc = &inst->dev->insts[inst->id];

	for (i = 0; i < ARRAY_SIZE(gdc->fmt_cap); i++) {
		format = cam_format_to_pixelformat
				(gdc->fmt_cap[i].format, 0);
		if (format == fsize->pixel_format) {
			cap = &gdc->fmt_cap[i];
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

static int gdc_enum_out_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	return -EINVAL;
}

static void fill_irq_ctx(struct gdc_v4l_instance *gdc, struct gdc_irq_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	if (gdc->sink_ctx.pad)
		ctx->sink_ctx = &gdc->sink_ctx;
	if (gdc->src_ctx.pad)
		ctx->src_ctx = &gdc->src_ctx;
}

static int gdc_queue_setup(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct gdc_v4l_instance *ins = container_of(ctx, struct gdc_v4l_instance, sink_ctx);
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

static int gdc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gdc_v4l_instance *gdc = sd_to_gdc_v4l_instance(sd);
	struct gdc_irq_ctx ctx;
	int rc;

	if (enable) {
		if (!gdc->node.bctx.is_sink_online_mode)
			cam_reqbufs(&gdc->sink_ctx, 4, &gdc_buf_ops);

		fill_irq_ctx(gdc, &ctx);
		gdc_set_ctx(gdc->dev, gdc->id, &ctx);
	} else {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;

		memset(&ctx, 0, sizeof(ctx));
		gdc_set_ctx(gdc->dev, gdc->id, &ctx);

		if (!gdc->node.bctx.is_sink_online_mode)
			cam_reqbufs(&gdc->sink_ctx, 0, NULL);
	}

	rc = gdc_set_state(gdc->dev, gdc->id, enable);
	if (rc < 0)
		return rc;

	if (enable) {
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static int gdc_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int gdc_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int gdc_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct gdc_v4l_instance *inst = sd_to_gdc_v4l_instance(sd);
	struct gdc_format f;
	int rc;

	rc = subdev_set_fmt(sd, state, fmt);
	if (rc < 0)
		return rc;

	memset(&f, 0, sizeof(f));
	f.ifmt.format = CAM_FMT_NV12;
	f.ifmt.width  = fmt->format.width;
	f.ifmt.height = fmt->format.height;
	f.ifmt.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	f.ofmt.width  = fmt->format.width;
	f.ofmt.height = fmt->format.height;
	f.ofmt.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	if (inst->out_pixelformat)
		f.ofmt.format = pixelformat_to_cam_format(inst->out_pixelformat);
	else
		f.ofmt.format = mbus_code_to_cam_format(fmt->format.code);

	return gdc_set_format(inst->dev, inst->id, &f);
}

static int gdc_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int gdc_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int gdc_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	return 0;
}

static int gdc_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static const struct v4l2_subdev_video_ops gdc_video_ops = {
	.s_stream = gdc_s_stream,
	.g_frame_interval = gdc_g_frame_interval,
	.s_frame_interval = gdc_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops gdc_pad_ops = {
	.set_fmt = gdc_set_fmt,
	.get_fmt = gdc_get_fmt,
	.enum_mbus_code = gdc_enum_mbus_code,
	.enum_frame_size = gdc_enum_frame_size,
	.enum_frame_interval = gdc_enum_frame_interval,
};

static const struct v4l2_subdev_ops gdc_subdev_ops = {
	.video = &gdc_video_ops,
	.pad = &gdc_pad_ops,
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
		n->bctx.get_format = gdc_get_out_format;
		n->bctx.set_format = gdc_set_out_format;
		n->bctx.enum_format = gdc_enum_out_format;
		n->bctx.enum_framesize = gdc_enum_out_framesize;
		n->bctx.enum_frameinterval = gdc_enum_out_frameinterval;

		n->dev = dev;
		n->num_pads = 2;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			gdc_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;

		rc = subdev_init(n, GDC_DEV_NAME, v4l_dev->gdc_dev.id,
				 i, &gdc_subdev_ops, &gdc_media_ops);
		if (rc < 0) {
			gdc_inst_remove(insts, i - 1);
			return rc;
		}
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
	}

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
