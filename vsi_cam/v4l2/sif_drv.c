// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[sif_drv]: %s: " fmt, __func__

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
	struct cam_ctx *buf_ctx, *src_ctx;
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
			src_ctx = &sif->src_ctx;
			if (src_ctx->pad)
				return -EBUSY;
			rc = cam_ctx_init(src_ctx, sd->dev, (void *)local, false);
			if (rc < 0)
				return rc;
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
	case V4L2_PIX_FMT_NV12:
		return true;
	default:
		return false;
	}
}

static u32 sif_get_ctx_format(struct v4l2_buf_ctx *ctx)
{
	return 0;
}

static int sif_set_ctx_format(struct v4l2_buf_ctx *ctx, u32 pad,
						      struct v4l2_format *format, bool is_try)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct v4l2_subdev *sd;
	struct v4l2_subdev_state state = {0};
	struct v4l2_subdev_pad_config pads = {0};
	struct sif_instance *sif;
	struct cam_format f = {0};
	struct v4l2_subdev_format senfmt = {0};
	int rc = 0;

	sd = &inst->node.sd;
	if (is_try)
		senfmt.which = V4L2_SUBDEV_FORMAT_TRY;
	else
		senfmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	senfmt.format.width = format->fmt.pix.width;
	senfmt.format.height = format->fmt.pix.height;
	f.width = format->fmt.pix.width;
	f.height = format->fmt.pix.height;
	f.stride = ALIGN(f.width, STRIDE_ALIGN);
	if (format->fmt.pix.pixelformat == V4L2_PIX_FMT_NV12 && inst->conv_nv12) {
		senfmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
		f.format = pixelformat_to_cam_format(V4L2_PIX_FMT_NV16);
	} else {
		senfmt.format.code = pixelformat_to_mbus_code(format->fmt.pix.pixelformat);
		f.format = pixelformat_to_cam_format(format->fmt.pix.pixelformat);
	}

	mutex_lock(&inst->fmt_lock);
	if (inst->fmt_changed && !memcmp(&f, &inst->fmt, sizeof(f)))
		goto _exit;

	state.pads = &pads;
	rc = subdev_set_fmt(sd, &state, &senfmt);
	if (rc < 0) {
		pr_err("%s subdev_set_fmt failed\n", __func__);
		goto _exit;
	}

	if (is_try)
		goto _exit;

	inst->dev->ipi_base = inst->id;
	inst->dev->ipi_channel_num = 1;
	sif = &inst->dev->insts[inst->id];
	memset(&sif->sif_cfg, 0, sizeof(sif->sif_cfg));
	if (format->fmt.pix.pixelformat == V4L2_PIX_FMT_NV12 && inst->conv_nv12)
		sif->sif_cfg.yuv_conv = 1;

	rc = sif_set_format(inst->dev, inst->id, &f, inst->en_post, BOTH_CHANNEL);
	if (rc < 0) {
		pr_err("%s sif_set_format failed\n", __func__);
		goto _exit;
	}

	inst->fmt = f;
	inst->fmt_changed = true;

_exit:
	mutex_unlock(&inst->fmt_lock);
	return rc;
}

static int sif_enum_ctx_format(struct v4l2_buf_ctx *ctx, u32 index, u32 *format)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);

	if (index >= inst->fmt_cap_num)
		return -EINVAL;

	*format = inst->fmt_cap[index];

	return 0;
}

static int sif_enum_ctx_framesize(struct v4l2_buf_ctx *ctx, u32 pad,
				  struct v4l2_frmsizeenum *fsize)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int rc = 0;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return -EINVAL;

	fse.pad = rpad->index;
	if (fsize->pixel_format == V4L2_PIX_FMT_NV12 && inst->conv_nv12)
		fse.code = pixelformat_to_mbus_code(V4L2_PIX_FMT_YUYV);
	else
		fse.code = pixelformat_to_mbus_code(fsize->pixel_format);

	rc = v4l2_subdev_call(rsd, pad, enum_frame_size, NULL, &fse);
	if (rc < 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.min_width;
	fsize->discrete.height = fse.min_height;

	return rc;
}

static int sif_enum_ctx_frameinterval(struct v4l2_buf_ctx *ctx, u32 pad,
				      struct v4l2_frmivalenum *fival)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct v4l2_subdev *sd, *rsd;
	struct media_pad *rpad;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int rc = 0;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return -EINVAL;

	fie.pad = rpad->index;
	if (fival->pixel_format == V4L2_PIX_FMT_NV12 && inst->conv_nv12)
		fie.code = pixelformat_to_mbus_code(V4L2_PIX_FMT_YUYV);
	else
		fie.code = pixelformat_to_mbus_code(fival->pixel_format);

	rc = v4l2_subdev_call(rsd, pad, enum_frame_interval, NULL, &fie);
	if (rc < 0)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return rc;
}

static void sif_set_cap(struct v4l2_buf_ctx *ctx)
{
	struct sif_v4l_instance *inst = buf_ctx_to_sif_v4l_instance(ctx);
	struct v4l2_subdev *sd, *rsd;
	struct v4l2_subdev_mbus_code_enum mbus_code;
	struct media_pad *rpad;
	int i, j = 0, rc;
	u32 pixelformat;
	bool sensor_support_nv12 = false;
	bool sensor_support_yuv422 = false;

	sd = &inst->node.sd;
	rsd = get_remote_src_subdev(sd, &rpad);
	if (!rsd)
		return;

	memset(inst->fmt_cap, 0, sizeof(inst->fmt_cap));

	for (i = 0; i < ARRAY_SIZE(inst->fmt_cap); i++) {
		memset(&mbus_code, 0, sizeof(mbus_code));
		mbus_code.index = i;
		mbus_code.pad = rpad->index;
		mbus_code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		rc = v4l2_subdev_call(rsd, pad, enum_mbus_code, NULL, &mbus_code);
		if (rc < 0)
			break;
		pixelformat = mbus_code_to_pixelformat(mbus_code.code);
		if (pixelformat == V4L2_PIX_FMT_YUYV)
			sensor_support_yuv422 = true;
		if (!is_support_fmt(pixelformat))
			continue;
		inst->fmt_cap[j] = pixelformat;
		if (pixelformat == V4L2_PIX_FMT_NV12)
			sensor_support_nv12 = true;
		j++;
	}
	if (!sensor_support_nv12 && sensor_support_yuv422) {
		inst->fmt_cap[j] = V4L2_PIX_FMT_NV12;
		inst->conv_nv12 = 1;
		j++;
	}
	inst->fmt_cap_num = j;
}

static int sif_map_info(struct v4l2_buf_ctx *ctx, u32 *devid, u32 *insid)
{
	struct sif_v4l_instance *sif = buf_ctx_to_sif_v4l_instance(ctx);

	if (!devid || !insid)
		return -EINVAL;

	*devid = sif->dev->id;
	*insid = sif->id;

	return 0;
}

static int sif_set_stream(struct v4l2_buf_ctx *ctx, u32 pad, int enable)
{
	struct sif_v4l_instance *sif = buf_ctx_to_sif_v4l_instance(ctx);
	struct sif_irq_ctx irq_ctx;
	int rc = 0;
	u32 set_dma = 0, set_isp = 0;

	if (pad >= sif->node.num_pads)
		return -EINVAL;

	memset(&irq_ctx, 0, sizeof(irq_ctx));
	rc = sif_get_ctx(sif->dev, sif->id, &irq_ctx);
	if (rc)
		return -EINVAL;

	if (enable) {
		if (!sif->buf_ctx.pad || sif->buf_ctx.pad->index != pad) {
			irq_ctx.src_ctx = &sif->src_ctx;
			set_isp = 1;
		} else {
			irq_ctx.buf_ctx = &sif->buf_ctx;
			set_dma = 1;
		}

		rc = sif_set_ctx(sif->dev, sif->id, &irq_ctx, 1);
		if (rc)
			return -EINVAL;

		if (refcount_read(&sif->start_refcnt) > REFCNT_INIT_VAL) {
			if (set_dma)
				sif_set_dma(sif->dev, sif->id, 1);
			if (set_isp)
				sif_set_isp_ctrl(sif->dev, sif->id, 1, true);
		}
	} else {
		if (!sif->buf_ctx.pad || sif->buf_ctx.pad->index != pad) {
			if (refcount_read(&sif->start_refcnt) > REFCNT_INIT_VAL && irq_ctx.src_ctx)
				sif_set_isp_ctrl(sif->dev, sif->id, 0, true);
			irq_ctx.src_ctx = NULL;
		} else {
			if (refcount_read(&sif->start_refcnt) > REFCNT_INIT_VAL && irq_ctx.buf_ctx)
				sif_set_dma(sif->dev, sif->id, 0);
			irq_ctx.buf_ctx = NULL;
		}

		rc = sif_set_ctx(sif->dev, sif->id, &irq_ctx, 0);
		if (rc)
			return -EINVAL;
	}

	return rc;
}

static int sif_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	int rc;

	if (!enable) {
		if (refcount_read(&inst->start_refcnt) == REFCNT_INIT_VAL)
			return 0;
		if (refcount_read(&inst->start_refcnt) > REFCNT_INIT_VAL)
			refcount_dec(&inst->start_refcnt);

		if (refcount_read(&inst->start_refcnt) == REFCNT_INIT_VAL) {
			sif_set_isp_ctrl(inst->dev, inst->id, 0, true);
			sif_set_dma(inst->dev, inst->id, 0);
			rc = subdev_set_stream(sd, enable);
			if (rc < 0)
				return rc;
			rc = sif_set_state(inst->dev, inst->id, enable, inst->en_post);
			if (rc < 0)
				return rc;

			inst->fmt_changed = false;
		}
	} else {
		if (refcount_read(&inst->start_refcnt) > REFCNT_INIT_VAL) {
			refcount_inc(&inst->start_refcnt);
			return 0;
		}

		refcount_inc(&inst->start_refcnt);
		rc = sif_set_state(inst->dev, inst->id, enable, inst->en_post);
		if (rc < 0)
			return rc;
		sif_set_dma(inst->dev, inst->id, 1);
		sif_set_isp_ctrl(inst->dev, inst->id, 1, true);
		rc = subdev_set_stream(sd, enable);
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int sif_g_frame_interval(struct v4l2_subdev *sd,
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

static int sif_s_frame_interval(struct v4l2_subdev *sd,
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

static const struct v4l2_subdev_ops sif_subdev_ops = {
	.core = &sif_core_ops,
	.video = &sif_video_ops,
};

static int sif_v4l_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	int rc = 0;

	mutex_lock(&inst->open_lock);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL) {
		refcount_inc(&inst->open_count);
		goto _exit;
	}
	refcount_inc(&inst->open_count);

	rc = subdev_open(sd);
	if (rc < 0)
		goto _exit;

	rc = sif_open(inst->dev, inst->id);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
}

static int sif_v4l_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sif_v4l_instance *inst = sd_to_sif_v4l_instance(sd);
	int rc = 0;

	mutex_lock(&inst->open_lock);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL)
		refcount_dec(&inst->open_count);
	if (refcount_read(&inst->open_count) > REFCNT_INIT_VAL)
		goto _exit;

	rc = subdev_close(sd);
	if (rc < 0)
		goto _exit;

	rc = sif_close(inst->dev, inst->id);

_exit:
	mutex_unlock(&inst->open_lock);
	return rc;
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
		cam_ctx_release(&ins->src_ctx);
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

		mutex_init(&inst->open_lock);
		mutex_init(&inst->fmt_lock);
		refcount_set(&inst->start_refcnt, REFCNT_INIT_VAL);
		refcount_set(&inst->open_count, REFCNT_INIT_VAL);

		n->async_bound = sif_async_bound;

		n->bctx.ready = sif_buf_ready;
		n->bctx.get_format = sif_get_ctx_format;
		n->bctx.set_format = sif_set_ctx_format;
		n->bctx.enum_format = sif_enum_ctx_format;
		n->bctx.enum_framesize = sif_enum_ctx_framesize;
		n->bctx.enum_frameinterval = sif_enum_ctx_frameinterval;
		n->bctx.set_cap = sif_set_cap;
		n->bctx.set_stream = sif_set_stream;
		n->bctx.map_info = sif_map_info;

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
		cam_ctx_release(&v4l_dev->insts[i].src_ctx);
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
