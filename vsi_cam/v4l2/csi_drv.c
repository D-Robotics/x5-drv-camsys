// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>

#include "cam_dev.h"
#include "csi_drv.h"

#define sd_to_csi_v4l_instance(s) \
({ \
	struct subdev_node *sn = container_of(s, struct subdev_node, sd); \
	container_of(sn, struct csi_v4l_instance, node); \
})

static long csi_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);

	if (csi->is_idi_mode)
		return csi_idi_ioctl(sd, cmd, arg);

	return -ENOIOCTLCMD;
}

static int csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);

	if (csi->is_idi_mode)
		return 0;

	if (enable)
		csi_ipi_start(csi->dev, csi->id);
	else
		csi_ipi_stop(csi->dev, csi->id);
	return 0;
}

static int csi_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static int csi_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fiv)
{
	return 0;
}

static void csi_subirq_callback(struct csi_device *csi_dev, u32 sub_val, const struct csi_irq_reg *err_reg, u32 reset_flag)
{
	int i = 0;

	if (reset_flag) {
		dev_dbg(csi_dev->dev, "trigger ipi overflow reset");
		csi_ipi_overflow_reset(csi_dev, err_reg->id - CSI_ERR_IPI1);
	}

	while (sub_val) {
		if (sub_val & 0x01)
			dev_err(csi_dev->dev, "get err %s", err_reg->errstr[i]);
		i++;
		sub_val >>= 1;
	}
}

static u32 mbus_code_to_csi_format(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return CAM_FMT_RAW8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return CAM_FMT_RAW10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return CAM_FMT_RAW12;
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		return CAM_FMT_YUYV;
	case MEDIA_BUS_FMT_YUYV8_1_5X8:
		return CAM_FMT_NV12;
	case MEDIA_BUS_FMT_RGB888_1X32_PADHI:
		return CAM_FMT_RGB888X;
	default:
		return CAM_FMT_NULL;
	}
}

static int csi_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	struct csi_v4l_instance *csi = sd_to_csi_v4l_instance(sd);
	struct cam_format f;
	struct csi_ipi_base_cfg ipi_cfg;

	if (csi->is_idi_mode)
		return 0;

	f.width = fmt->format.width;
	f.height = fmt->format.height;
	f.format = mbus_code_to_csi_format(fmt->format.code);
	ipi_cfg.id = csi->id;
	ipi_cfg.adv_val = CSI_IPI_ADV_FEAT_EVSELPROG | CSI_IPI_ADV_FEAT_EN_VIDEO
			| CSI_IPI_ADV_FEAT_EN_EBD | CSI_IPI_ADV_FEAT_MODE_LEGACY;
	ipi_cfg.cut_through = true;
	ipi_cfg.mem_auto_flush = true;
	csi_ipi_init(csi->dev, &ipi_cfg, &f);
	csi->dev->subirq_func = csi_subirq_callback;
	csi->dev->irq_done_func = NULL;
	csi_irq_enable(csi->dev);

	return 0;
}

static int csi_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int csi_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int csi_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	return 0;
}

static int csi_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0;
}

static const struct v4l2_subdev_core_ops csi_core_ops = {
	.ioctl = csi_ioctl,
};

static const struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
	.g_frame_interval = csi_g_frame_interval,
	.s_frame_interval = csi_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops csi_pad_ops = {
	.set_fmt = csi_set_fmt,
	.get_fmt = csi_get_fmt,
	.enum_mbus_code = csi_enum_mbus_code,
	.enum_frame_size = csi_enum_frame_size,
	.enum_frame_interval = csi_enum_frame_interval,
};

static const struct v4l2_subdev_ops csi_subdev_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

static void csi_inst_remove(struct csi_v4l_instance *insts, u32 num)
{
	u32 i;

	for (i = 0; i < num; i++)
		subdev_deinit(&insts[i].node);
}

static int csi_async_bound(struct subdev_node *sn)
{
	struct csi_v4l_instance *csi =
			container_of(sn, struct csi_v4l_instance, node);
	struct csi_v4l_instance *ins;
	struct csi_v4l_device *v4l_dev;
	u32 i = 0, j = 0;
	int rc;

	if (unlikely(!sn))
		return -EINVAL;

	v4l_dev = container_of(csi->dev, struct csi_v4l_device, csi_dev);

	while (i < csi->dev->num_insts) {
		ins = &v4l_dev->insts[i];

		if (ins != csi) {
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

		if (ins != csi)
			v4l2_device_unregister_subdev(&ins->node.sd);
		j++;
	}
	return rc;
}

static int csi_v4l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csi_v4l_device *v4l_dev;
	struct csi_v4l_instance *insts;
	u32 i;
	int rc;

	v4l_dev = devm_kzalloc(dev, sizeof(*v4l_dev), GFP_KERNEL);
	if (!v4l_dev)
		return -ENOMEM;

	rc = csi_probe(pdev, &v4l_dev->csi_dev, false);
	if (rc < 0) {
		dev_err(dev, "failed to call csi_probe (err=%d)\n", rc);
		return rc;
	}

	insts = devm_kzalloc(dev, sizeof(*insts) * v4l_dev->csi_dev.num_insts,
			     GFP_KERNEL);
	if (!insts)
		return -ENOMEM;

	for (i = 0; i < v4l_dev->csi_dev.num_insts; i++) {
		struct csi_v4l_instance *inst = &insts[i];
		struct subdev_node *n = &inst->node;

		inst->id = i;
		inst->dev = &v4l_dev->csi_dev;

		n->async_bound = csi_async_bound;

		n->dev = dev;
		n->num_pads = 2;
		n->pads = devm_kzalloc
				(dev, sizeof(*n->pads) * n->num_pads, GFP_KERNEL);
		if (!n->pads) {
			csi_inst_remove(insts, i - 1);
			return -ENOMEM;
		}

		n->pads[0].flags = MEDIA_PAD_FL_SINK;
		n->pads[1].flags =
				MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;

		rc = subdev_init(n, CSI_DEV_NAME, v4l_dev->csi_dev.id,
				 i, &csi_subdev_ops, NULL);
		if (rc < 0) {
			csi_inst_remove(insts, i - 1);
			return rc;
		}

		/* the last one is for idi-control */
		if (i == v4l_dev->csi_dev.num_insts - 1) {
			inst->is_idi_mode = true;
			n->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		}
	}
	v4l_dev->insts = insts;

	rc = v4l2_async_register_subdev(&insts[0].node.sd);
	if (rc < 0) {
		csi_inst_remove(insts, v4l_dev->csi_dev.num_insts);
		csi_remove(pdev, &v4l_dev->csi_dev);
		return rc;
	}

	platform_set_drvdata(pdev, v4l_dev);

	rc = csi_runtime_resume(dev);
		if (rc) {
		dev_err(dev, "failed to call csi_runtime_resume (err=%d)\n", rc);
		return rc;
	}

	if (v4l_dev->csi_dev.ipi)
		dev_dbg(dev, "ipi clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.ipi));
	if (v4l_dev->csi_dev.pclk)
		dev_dbg(dev, "pclk clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.pclk));
	if (v4l_dev->csi_dev.cfg)
		dev_dbg(dev, "cfg clock: %ld Hz\n", clk_get_rate(v4l_dev->csi_dev.cfg));


	dev_dbg(dev, "VS CSI driver (v4l) probed done\n");
	return 0;
}

static int csi_v4l_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csi_v4l_device *v4l_dev = platform_get_drvdata(pdev);
	int rc;

	v4l2_async_unregister_subdev(&v4l_dev->insts[0].node.sd);
	csi_inst_remove(v4l_dev->insts, v4l_dev->csi_dev.num_insts);

	rc = csi_remove(pdev, &v4l_dev->csi_dev);
	if (rc < 0) {
		dev_err(dev, "failed to call csi_remove (err=%d)\n", rc);
		return rc;
	}

	rc = csi_runtime_suspend(dev);
		if (rc) {
		dev_err(dev, "failed to call csi_runtime_suspend (err=%d)\n", rc);
		return rc;
	}

	dev_dbg(dev, "VS CSI driver (v4l) removed\n");
	return 0;
}

static const struct dev_pm_ops csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(csi_system_suspend, csi_system_resume)
};

static const struct of_device_id csi_of_match[] = {
	{ .compatible = CSI_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, csi_of_match);

static struct platform_driver csi_driver = {
	.probe  = csi_v4l_probe,
	.remove = csi_v4l_remove,
	.driver = {
		.name = CSI_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = csi_of_match,
		.pm = &csi_pm_ops,
	}
};

static int __init csi_init_module(void)
{
	return platform_driver_register(&csi_driver);
}

static void __exit csi_exit_module(void)
{
	platform_driver_unregister(&csi_driver);
}

module_init(csi_init_module);
module_exit(csi_exit_module);

MODULE_DESCRIPTION("SNPS MIPI CSI Controller Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("SNPS-CSI");
