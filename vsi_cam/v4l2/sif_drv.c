// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "cam_uapi.h"

#include "sif_drv.h"

#define sd_to_sif_v4l_instance(s)                                \
	({                                                       \
		struct subdev_node *sn =                         \
			container_of(s, struct subdev_node, sd); \
		container_of(sn, struct sif_v4l_instance, node); \
	})

#define buf_ctx_to_sif_v4l_instance(ctx)                             \
	({                                                           \
		struct subdev_node *sn =                             \
			container_of(ctx, struct subdev_node, bctx); \
		container_of(sn, struct sif_v4l_instance, node);     \
	})

static inline void update_en_post_status(struct sif_v4l_instance *sif,
					 struct v4l2_subdev *sd, bool connect)
{
	/* if isp sub device is connected, format and state changed
	 * notification (event post) should be disabled from sif side
	 */
	if (strstr(sd->entity.name, ISP_DEV_NAME))
		sif->en_post = !connect;
}

static int sif_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd;
	struct sif_v4l_instance *sif;
	struct media_pad *pad;
	struct cam_ctx *buf_ctx;
	struct v4l2_buf_ctx *rctx, *lctx;
	int rc = 0;

	if (!entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(entity);
	sif = sd_to_sif_v4l_instance(sd);

	pad = media_pad_remote_pad_first(local);
	if (pad && pad != remote)
		return -EBUSY;

	if (is_media_entity_v4l2_subdev(remote->entity)) {
		sd = media_entity_to_v4l2_subdev(remote->entity);
		rctx = v4l2_get_subdevdata(sd);
		lctx = &sif->node.bctx;

		if (local->flags & MEDIA_PAD_FL_SOURCE) {
			update_en_post_status(sif, sd,
					      flags & MEDIA_LNK_FL_ENABLED);
			lctx->is_src_online_mode = rctx->is_sink_online_mode;
			if (rctx->is_sink_online_mode)
				return 0;
		}
	}

	if (local->flags & MEDIA_PAD_FL_SINK)
		return 0;

	buf_ctx = &sif->buf_ctx;
	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (buf_ctx->pad)
			return -EBUSY;

		rc = cam_ctx_init(buf_ctx, sd->dev, (void *)local, false);
		if (rc < 0)
			return rc;
	} else {
		cam_ctx_release(buf_ctx);
	}
	return rc;
}

static const struct media_entity_operations sif_media_ops = {
	.link_setup = sif_link_setup,
};

static void sif_buf_ready(struct v4l2_buf_ctx *ctx, u32 pad, int on)
{
}

static u32 sif_get_out_format(struct v4l2_buf_ctx *ctx)
{
	struct sif_v4l_instance *sif = buf_ctx_to_sif_v4l_instance(ctx);

	return sif->out_pixelformat;
}

static int sif_set_out_format(struct v4l2_buf_ctx *ctx, u32 format, bool is_try)
{
	struct sif_v4l_instance *sif = buf_ctx_to_sif_v4l_instance(ctx);

	if (!is_try)
		sif->out_pixelformat = format;
	return 0;
}

static int sif_enum_out_format(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct sif_instance *sif;

	if (!format)
		return -EINVAL;

	sif = &inst->dev->insts[inst->id];

	if (index >= ARRAY_SIZE(sif->fmt_cap))
		return -EINVAL;

	*format = cam_format_to_pixelformat(sif->fmt_cap[index].format,
					    sif->input_bayer_format);
	return *format ? 0 : -EINVAL;
}

static int sif_enum_out_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct sif_instance *sif;
	struct sif_format_cap *cap = NULL;
	struct cam_res_cap *res;
	u32 i, format;

	sif = &inst->dev->insts[inst->id];

	for (i = 0; i < ARRAY_SIZE(sif->fmt_cap); i++) {
		format = cam_format_to_pixelformat(sif->fmt_cap[i].format,
						   sif->input_bayer_format);
		if (format == fsize->pixel_format) {
			cap = &sif->fmt_cap[i];
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

static int sif_enum_out_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct sif_v4l_instance *ins = buf_ctx_to_sif_v4l_instance(ctx);
	struct sif_instance *sif;
	struct sif_format_cap *cap = NULL;
	struct cam_res_cap *res;
	bool found = false;
	u32 i, format;

	if (fival->index > 0)
		return -EINVAL;

	sif = &ins->dev->insts[ins->id];

	for (i = 0; i < ARRAY_SIZE(sif->fmt_cap); i++) {
		format = cam_format_to_pixelformat
				(sif->fmt_cap[i].format, sif->input_bayer_format);
		if (format == fival->pixel_format) {
			cap = &sif->fmt_cap[i];
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
	fival->discrete.numerator = ins->out_fps.numerator;
	fival->discrete.denominator = ins->out_fps.denominator;
	return 0;
}

static void sif_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct sif_instance *ins;
	struct sif_format_cap *cap;
	struct cam_res_cap *res;
	struct v4l2_subdev *sd;
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev_state state;
	uint32_t support_fmt[2] = {CAM_FMT_RAW8, CAM_FMT_NV12};
	int i, j, rc;

	ins = &inst->dev->insts[inst->id];
	sd = &inst->node.sd;
	ins->input_bayer_format = BAYER_FMT_BGGR;

	memset(ins->fmt_cap, 0, sizeof(ins->fmt_cap));

	for(j = 0; j < ARRAY_SIZE(support_fmt); j++){
		cap = &ins->fmt_cap[j];
		cap->format = support_fmt[j];
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
}

static int sif_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	struct sif_irq_ctx ctx;
	int rc;

	if (!enable) {
		if (refcount_read(&inst->start_refcnt) == REFCNT_INIT_VAL)
			return 0;

		if (inst->buf_ctx.pad) {
			memset(&ctx, 0, sizeof(ctx));
			sif_set_ctx(inst->dev, inst->id, &ctx);
		}

		if (refcount_read(&inst->start_refcnt) > REFCNT_INIT_VAL)
			refcount_dec(&inst->start_refcnt);

		if (refcount_read(&inst->start_refcnt) == REFCNT_INIT_VAL) {
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;

			rc = sif_set_state(inst->dev, inst->id, enable, inst->en_post);
			if (rc < 0)
				return rc;
		}
	} else {
		if (inst->buf_ctx.pad) {
			memset(&ctx, 0, sizeof(ctx));
			ctx.buf_ctx = &inst->buf_ctx;
			sif_set_ctx(inst->dev, inst->id, &ctx);
		}

		if (refcount_read(&inst->start_refcnt) == REFCNT_INIT_VAL) {
			rc = sif_set_state(inst->dev, inst->id, enable, inst->en_post);
			if (rc < 0)
				return rc;

			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
		}
		refcount_inc(&inst->start_refcnt);
	}
	return 0;
}

static int sif_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct sif_v4l_instance *ins = sd_to_sif_v4l_instance(sd);

	fiv->interval = ins->out_fps;
	return 0;
}

static int sif_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	struct sif_v4l_instance *ins = sd_to_sif_v4l_instance(sd);

	fiv->interval = ins->out_fps;
	return 0;
}

static int sif_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	struct sif_instance *sif;
	struct cam_format f;
	struct v4l2_subdev_format senfmt;
	int rc;

	memcpy(&senfmt, fmt, sizeof(senfmt));
	if(senfmt.format.code == MEDIA_BUS_FMT_YUYV8_1_5X8)
		senfmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;

	rc = subdev_set_fmt(sd, state, &senfmt);
	if (rc < 0)
		return rc;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	f.width = fmt->format.width;
	f.height = fmt->format.height;
	f.stride = ALIGN(fmt->format.width, STRIDE_ALIGN);
	if (senfmt.format.code == MEDIA_BUS_FMT_YUYV8_1X16)
		f.format = pixelformat_to_cam_format(V4L2_PIX_FMT_NV16);
	else if (inst->out_pixelformat)
		f.format = pixelformat_to_cam_format(inst->out_pixelformat);
	else
		f.format = mbus_code_to_cam_format(fmt->format.code);

	inst->dev->ipi_base = inst->id;
	inst->dev->ipi_channel_num = 1;

	sif = &inst->dev->insts[inst->id];
	memset(&sif->sif_cfg, 0, sizeof(sif->sif_cfg));
	if(senfmt.format.code == MEDIA_BUS_FMT_YUYV8_1X16)
		sif->sif_cfg.yuv_conv = 1;

	rc = sif_set_format(inst->dev, inst->id, &f, inst->en_post,
			    BOTH_CHANNEL);
	if (rc < 0)
		return rc;

	inst->out_fmt = *fmt;
	return 0;
}

static int sif_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);

	*fmt = inst->out_fmt;
	return 0;
}

static int sif_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int sif_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	int rc;
	rc = subdev_enum_frame_size(sd, state, fse);
	if (rc < 0)
		return rc;
	return 0;
}

static int sif_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static long sif_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	return subdev_call_command(sd, cmd, arg);
}

static const struct v4l2_subdev_core_ops sif_core_ops = {
	.command = sif_command,
};

static const struct v4l2_subdev_video_ops sif_video_ops = {
	.s_stream = sif_s_stream,
	.g_frame_interval = sif_g_frame_interval,
	.s_frame_interval = sif_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops sif_pad_ops = {
	.set_fmt = sif_set_fmt,
	.get_fmt = sif_get_fmt,
	.enum_mbus_code = sif_enum_mbus_code,
	.enum_frame_size = sif_enum_frame_size,
	.enum_frame_interval = sif_enum_frame_interval,
};

static const struct v4l2_subdev_ops sif_subdev_ops = {
	.core = &sif_core_ops,
	.video = &sif_video_ops,
	.pad = &sif_pad_ops,
};

static int sif_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	int rc;

	rc = subdev_open(sd);
	if (rc < 0)
		return rc;

	return sif_open(inst->dev, inst->id);
}

static int sif_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	int rc;

	rc = subdev_close(sd);
	if (rc < 0)
		return rc;

	inst->out_pixelformat = 0;
	return sif_close(inst->dev, inst->id);
}

static const struct v4l2_subdev_internal_ops sif_internal_ops = {
	.open = sif_v4l_open,
	.close = sif_v4l_close,
};

static void sif_inst_remove(struct sif_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int sif_async_bound(struct subdev_node *sn)
{
	struct sif_v4l_instance *sif =
		container_of(sn, struct sif_v4l_instance, node);
	struct sif_v4l_instance *ins;
	struct sif_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(sif->dev, struct sif_v4l_device, sif_dev);

	while (i < sif->dev->num_insts) {
		ins = &v4l_dev->insts[i];
		cam_ctx_release(&ins->buf_ctx);

		if (ins != sif) {
			rc = v4l2_device_register_subdev(sn->sd.v4l2_dev,
							 &ins->node.sd);
			if (rc < 0)
				goto _err;
		}
		i++;
	}
	return 0;

_err:
	while (j < i) {
		ins = &v4l_dev->insts[j];

		if (ins != sif)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int sif_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sif_v4l_device *v4l_dev;
	struct sif_v4l_instance *insts;
	u32 i;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = sif_probe(pdev, &v4l_dev->sif_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call sif_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->sif_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->sif_dev.num_insts; i++) {
		struct sif_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->sif_dev;
		inst->en_post = true;
		refcount_set(&inst->start_refcnt, REFCNT_INIT_VAL);
		inst->out_fps.numerator = 30;
		inst->out_fps.denominator = 1;

		n->async_bound = sif_async_bound;

		n->bctx.ready = sif_buf_ready;
		n->bctx.get_format = sif_get_out_format;
		n->bctx.set_format = sif_set_out_format;
		n->bctx.enum_format = sif_enum_out_format;
		n->bctx.enum_framesize = sif_enum_out_framesize;
		n->bctx.enum_frameinterval = sif_enum_out_frameinterval;
		n->bctx.set_cap = sif_set_cap;

		n->dev = dev;
		n->num_pads = 3;
		n->pads = devm_kzalloc(dev, sizeof(*n->pads) * n->num_pads,
				       GFP_KERNEL);
		if (!n->pads) {
			sif_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags = MEDIA_PAD_FL_SOURCE |
				   MEDIA_PAD_FL_MUST_CONNECT;
		n->pads[2].flags = MEDIA_PAD_FL_SOURCE;

		rc = subdev_init(n, SIF_DEV_NAME, v4l_dev->sif_dev.id, i,
				 &sif_subdev_ops, &sif_media_ops);
		if (rc < 0) {
			sif_inst_remove(insts, i - 1);
			return rc;
		}
		n->sd.internal_ops = &sif_internal_ops;
	}
	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		sif_inst_remove(insts, v4l_dev->sif_dev.num_insts);
		sif_remove(pdev, &v4l_dev->sif_dev);
		return rc;
	}

	platform_set_drvdata(pdev, v4l_dev);

	if (v4l_dev->sif_dev.axi)
		dev_dbg(dev, "axi clock: %ld Hz\n", clk_get_rate(v4l_dev->sif_dev.axi));
	if (v4l_dev->sif_dev.pclk)
		dev_dbg(dev, "pclk clock: %ld Hz\n", clk_get_rate(v4l_dev->sif_dev.pclk));


	dev_dbg(dev, "VS SIF driver (v4l) probed done\n");
	return 0;
}

static int sif_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sif_v4l_device *v4l_dev = platform_get_drvdata(pdev);
	int rc;
	u32 i;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);

	for (i = 0; i < v4l_dev->sif_dev.num_insts; i++) {
		cam_ctx_release(&v4l_dev->insts[i].buf_ctx);
		subdev_deinit(&v4l_dev->insts[i].node);
	}

	rc = sif_remove(pdev, &v4l_dev->sif_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call sif_remove (err=%d)\n", rc);
		return rc;
	}

	dev_dbg(dev, "VS SIF driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops sif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sif_system_suspend, sif_system_resume)
};

static const struct of_device_id sif_of_match[] = {
	{ .compatible = SIF_DT_NAME },
	{},
};

MODULE_DEVICE_TABLE(of, sif_of_match);

static struct platform_driver sif_driver = { .probe = sif_v4l_probe,
					     .remove = sif_v4l_remove,
					     .driver = {
						     .name = SIF_DEV_NAME,
						     .owner = THIS_MODULE,
						     .of_match_table =
							     sif_of_match,
						     .pm = &sif_pm_ops,
					     } };

static int __init sif_init_module(void)
{
	return platform_driver_register(&sif_driver);
}

static void __exit sif_exit_module(void)
{
	platform_driver_unregister(&sif_driver);
}

module_init(sif_init_module);
module_exit(sif_exit_module);

MODULE_DESCRIPTION("VeriSilicon SIF Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VeriSilicon-SIF");
