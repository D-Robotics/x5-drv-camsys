// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
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

	return isp_add_job(isp->dev, isp->id, false);
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
	if (!*format)
		return -EINVAL;
	return 0;
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
	return -EINVAL;
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
	struct v4l2_ext_control ext_ctrl;
	int ret;

	ret = copy_from_user(&ext_ctrl, arg, sizeof(struct v4l2_ext_control));
	if (ret) {
		pr_err("%s: ctrl_data copy_from_user failed!\n", __func__);
		return ret;
	}
	u32 size = 0;
	switch (ext_ctrl.id) {
		case V4L2_CID_DR_EXPOSURE:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case V4L2_CID_DR_AWB:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		default:
			return -1;
	}
	return isp_set_subctrl(isp->dev, isp->id, ext_ctrl.id,
				       (void *)ext_ctrl.ptr, size);
}

static int isp_g_ctrl(struct isp_v4l_instance *isp, void *arg)
{
	struct v4l2_ext_control ext_ctrl;
	int ret;

	ret = copy_from_user(&ext_ctrl, arg, sizeof(struct v4l2_ext_control));
	if (ret) {
		pr_err("%s: ctrl_data copy_from_user failed!\n", __func__);
		return ret;
	}

	u32 size = 0;
	switch (ext_ctrl.id) {
		case V4L2_CID_DR_EXPOSURE:
			size = sizeof(hbn_isp_exposure_attr_t);
			break;
		case V4L2_CID_DR_AWB:
			size = sizeof(hbn_isp_awb_attr_t);
			break;
		default:
			return -1;
	}
	return isp_get_subctrl(isp->dev, isp->id, ext_ctrl.id,
				       (void *)ext_ctrl.ptr, size);
}

static long isp_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_v4l_instance *isp = sd_to_isp_v4l_instance(sd);
        int rc = 0;

	if (cmd)
		rc = isp_s_ctrl(isp, arg);
	else
		rc = isp_g_ctrl(isp, arg);
	if (rc < 0)
		pr_err("%s call isp ctrl failed\n", __func__);

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
			isp_set_stream_idx(isp->dev, isp->id, isp->id);

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
		rc = isp_close(isp->dev, isp->id);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static int isp_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int isp_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
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
	for (i = 0; i < ARRAY_SIZE(in_fmt_preferred_list); i++) {
		f.ifmt.format    = in_fmt_preferred_list[i];
		s_f.format.code =
				cam_format_to_mbus_code(in_fmt_preferred_list[i], isp->input_bayer_format);

		rc = subdev_set_fmt(sd, state, &s_f);
		if (!rc)
			break;
		if (rc < 0 && rc != -EINVAL)
			return rc;
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

	if (inst->id < ISP_SINK_ONLINE_PATH_MAX) {
		isp->online_mcm = true;
		inst->dev->mode = ISP_STRM_MODE;
	} else {
		isp->online_mcm = false;
		inst->dev->mode = ISP_MCM_MODE;
	}
	pr_debug("isp inst%d online_mcm=%d, mode=%d\n", inst->id, isp->online_mcm, inst->dev->mode);

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
	{
		// FIXME
		struct cam_input in;

		in.index = inst->id;
		in.type = CAM_INPUT_SENSOR;
		snprintf(in.sens.name, sizeof(in.sens.name), "ov5640");
		return isp_set_input(inst->dev, inst->id, &in);
	}
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

		n->async_bound = isp_async_bound;

		n->bctx.ready = isp_buf_ready;
		n->bctx.qbuf = isp_qbuf;
		n->bctx.dqbuf = isp_dqbuf;
		n->bctx.get_format = isp_get_out_format;
		n->bctx.set_format = isp_set_out_format;
		n->bctx.enum_format = isp_enum_out_format;
		n->bctx.enum_framesize = isp_enum_out_framesize;
		n->bctx.enum_frameinterval = isp_enum_out_frameinterval;

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

	pm_runtime_set_active(dev);
	rc = pm_runtime_get_sync(dev);
	if (rc < 0) {
		pm_runtime_disable(dev);
		return rc;
	}

	rc = isp_runtime_resume(dev);
	if (rc) {
		dev_err(dev, "failed to call isp_runtime_resume (err=%d)\n", rc);
		return rc;
	}

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

	isp_reset(isp_dev);

	rc = isp_runtime_suspend(dev);
	if (rc < 0)
		return rc;

	rc = pm_runtime_put_sync(dev);
	if (rc < 0)
		return rc;

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
