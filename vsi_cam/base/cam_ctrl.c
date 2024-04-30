// SPDX-License-Identifier: GPL-2.0-only
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "cam_ctrl.h"

#define CAM_CTRL_DT_NAME    "verisilicon,cam-ctrl-sr"
#define CAM_CTRL_DEV_NAME   "vs-cam-ctrl"

#define REG_VIDEO_STREAM_CTRL       (0x00)
#define REG_GDC_INTR_STAT           (0x0c)

#define cam_ctrl_write(ctrl, offset, value) \
	__raw_writel(value, (ctrl)->base + (offset))

#define cam_ctrl_read(ctrl, offset) \
	__raw_readl((ctrl)->base + (offset))

union video_stream_ctrl_reg {
	struct {
		u32 isp_s1_ipi_sel : 2;
		u32 isp_s1_csi_sel : 2;
		u32 isp_s2_ipi_sel : 2;
		u32 isp_s2_csi_sel : 2;
		u32 isp_s3_ipi_sel : 2;
		u32 isp_s3_csi_sel : 2;
		u32 isp_s4_ipi_sel : 2;
		u32 isp_s4_csi_sel : 2;
		u32 idi_sel : 2;
		u32 fifo_wr_en : 1;
		u32 reserved : 13;
	};
	u32 value;
};

union gdc_intr_stat_reg {
	struct {
		u32 done : 1;
		u32 reserved : 31;
	};
	u32 value;
};

struct cam_ctrl_device {
	struct device *dev;
	void __iomem *base;
};

int set_idi_bypass_select(struct cam_ctrl_device *dev, unsigned int csi_index, bool enable)
{
	union video_stream_ctrl_reg ctrl;

	if (!dev)
		return -ENODEV;

	if (csi_index > 3)
		return -EINVAL;

	ctrl.value = cam_ctrl_read(dev, REG_VIDEO_STREAM_CTRL);
	ctrl.idi_sel = csi_index;
	ctrl.fifo_wr_en = enable ? 1 : 0;
	cam_ctrl_write(dev, REG_VIDEO_STREAM_CTRL, ctrl.value);
	return 0;
}
EXPORT_SYMBOL(set_idi_bypass_select);

int set_isp_input_select(struct cam_ctrl_device *dev, unsigned int id, unsigned int csi_index,
			 unsigned int ipi_index)
{
	union video_stream_ctrl_reg ctrl;

	if (!dev)
		return -ENODEV;

	if (id > 3 || csi_index > 3 || ipi_index > 3)
		return -EINVAL;

	ctrl.value = cam_ctrl_read(dev, REG_VIDEO_STREAM_CTRL);
	switch (id) {
	case 0:
		ctrl.isp_s1_csi_sel = csi_index;
		ctrl.isp_s1_ipi_sel = ipi_index;
		break;
	case 1:
		ctrl.isp_s2_csi_sel = csi_index;
		ctrl.isp_s2_ipi_sel = ipi_index;
		break;
	case 2:
		ctrl.isp_s3_csi_sel = csi_index;
		ctrl.isp_s3_ipi_sel = ipi_index;
		break;
	case 3:
		ctrl.isp_s4_csi_sel = csi_index;
		ctrl.isp_s4_ipi_sel = ipi_index;
		break;
	default:
		break;
	}
	cam_ctrl_write(dev, REG_VIDEO_STREAM_CTRL, ctrl.value);
	return 0;
}
EXPORT_SYMBOL(set_isp_input_select);

int get_gdc_intr_stat_and_clear(struct cam_ctrl_device *dev, unsigned int *p_stat, bool *is_done)
{
	union gdc_intr_stat_reg stat;

	if (!dev)
		return -ENODEV;

	stat.value = cam_ctrl_read(dev, REG_GDC_INTR_STAT);
	if (stat.done)
		cam_ctrl_write(dev, REG_GDC_INTR_STAT, stat.value);
	if (p_stat)
		*p_stat = stat.value;
	if (is_done)
		*is_done = !!stat.done;
	return 0;
}
EXPORT_SYMBOL(get_gdc_intr_stat_and_clear);

void put_cam_ctrl_device(struct cam_ctrl_device *dev)
{
	if (dev)
		put_device(dev->dev);
}
EXPORT_SYMBOL(put_cam_ctrl_device);

static int cam_ctrl_probe(struct platform_device *pdev)
{
	struct cam_ctrl_device *ctrl;
	struct resource *res;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctrl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctrl->base)) {
		dev_err(&pdev->dev, "cannot get reg mem resource\n");
		return PTR_ERR(ctrl->base);
	}

	ctrl->dev = &pdev->dev;
	platform_set_drvdata(pdev, ctrl);
	return 0;
}

static int cam_ctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cam_ctrl_of_match[] = {
	{ .compatible = CAM_CTRL_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, cam_ctrl_of_match);

static struct platform_driver cam_ctrl_driver = {
	.probe	= cam_ctrl_probe,
	.remove = cam_ctrl_remove,
	.driver = {
		.name = CAM_CTRL_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_ctrl_of_match,
	}
};

static int __init cam_ctrl_init_module(void)
{
	return platform_driver_register(&cam_ctrl_driver);
}

static void __exit cam_ctrl_exit_module(void)
{
	platform_driver_unregister(&cam_ctrl_driver);
}

module_init(cam_ctrl_init_module);
module_exit(cam_ctrl_exit_module);

MODULE_DESCRIPTION("VeriSilicon Camera Control Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VS-CAM-CTRL");
