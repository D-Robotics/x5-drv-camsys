// SPDX-License-Identifier: GPL-2.0-only
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define REG_CAM_SEC_PULSE_CTRL_0    (0x00) // 0x8c
#define REG_CAM_SEC_PULSE_TAR_H     (0x04) // 0x90
#define REG_CAM_SEC_PULSE_TAR_L     (0x08) // 0x94
#define REG_CAM_SEC_PULSE_INTVAL    (0x0c) // 0x98
#define REG_CAM_SEC_PULSE_OFFSET    (0x10) // 0x9c
#define REG_CAM_SEC_PULSE_CTRL_1    (0x14) // 0xa0

union cam_sec_pulse_ctrl_0 {
	struct {
		u32 enable : 1;
		u32 reserved_0 : 3;
		u32 pulse_width : 4;
		u32 reserved_1 : 24;
	};
	u32 value;
};

struct cam_pulse_gen {
	u32 cam_pulse_width;
	u64 cam_pulse_wait;
	u32 cam_pulse_interval;
	u32 cam_pulse_offset;
	u32 cam_pulse_start_mode;
};

struct cam_pulse_device {
	struct device *dev;
	void __iomem *base;
	struct cam_pulse_gen cam_pulse_ctrl;
};

#define cam_pulse_write(cam_pulse, offset, value) \
	__raw_writel(value, (cam_pulse)->base + (offset))

#define cam_pulse_read(cam_pulse, offset) \
	__raw_readl((cam_pulse)->base + (offset))

#define CAM_PULSE_DT_NAME    "verisilicon,cam-pulse-gen"
#define CAM_PULSE_DEV_NAME   "vs-cam-pulse"

static int cam_pulse_parse_dt(struct platform_device *pdev, struct cam_pulse_gen *cam_pulse_ctrl)
{
	int rc = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "cam-pulse-width", &cam_pulse_ctrl->cam_pulse_width);
	if (rc < 0)
		cam_pulse_ctrl->cam_pulse_width = 0;

	rc = of_property_read_u64(pdev->dev.of_node, "cam-pulse-wait", &cam_pulse_ctrl->cam_pulse_wait);
	if (rc < 0)
		cam_pulse_ctrl->cam_pulse_wait = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "cam-pulse-interval", &cam_pulse_ctrl->cam_pulse_interval);
	if (rc < 0)
		cam_pulse_ctrl->cam_pulse_interval = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "cam-pulse-offset", &cam_pulse_ctrl->cam_pulse_offset);
	if (rc < 0)
		cam_pulse_ctrl->cam_pulse_offset = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "cam-pulse-start-mode", &cam_pulse_ctrl->cam_pulse_start_mode);
	if (rc < 0)
		cam_pulse_ctrl->cam_pulse_start_mode = 0;

	return rc;
}

static int start_cam_pulse_gen(struct cam_pulse_device *dev)
{
	union cam_sec_pulse_ctrl_0 ctrl_0;
	struct cam_pulse_gen *cam_pulse_ctrl = &dev->cam_pulse_ctrl;

	if (!dev)
		return -ENODEV;

	ctrl_0.value       = cam_pulse_read(dev, REG_CAM_SEC_PULSE_CTRL_0);
	ctrl_0.pulse_width = cam_pulse_ctrl->cam_pulse_width;
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_0, ctrl_0.value);
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_TAR_H, (u32)(cam_pulse_ctrl->cam_pulse_wait >> 32));
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_TAR_H, (u32)(cam_pulse_ctrl->cam_pulse_wait & (0xffffffff)));
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_INTVAL, cam_pulse_ctrl->cam_pulse_interval);
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_OFFSET, cam_pulse_ctrl->cam_pulse_offset);

	ctrl_0.value = cam_pulse_read(dev, REG_CAM_SEC_PULSE_CTRL_0);
	ctrl_0.enable = 1;
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_0, ctrl_0.value);
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_1, 0x4 | BIT(cam_pulse_ctrl->cam_pulse_start_mode));

	return 0;
}

static int stop_cam_pulse_gen(struct cam_pulse_device *dev)
{
	union cam_sec_pulse_ctrl_0 ctrl_0;

	if (!dev)
		return -ENODEV;

	ctrl_0.value = cam_pulse_read(dev, REG_CAM_SEC_PULSE_CTRL_0);
	ctrl_0.enable = 0;
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_0, ctrl_0.value);
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_0, 0);
	cam_pulse_write(dev, REG_CAM_SEC_PULSE_CTRL_1, 0);

	return 0;
}

static int cam_pulse_probe(struct platform_device *pdev)
{
	struct cam_pulse_device *cam_pulse;
	struct resource *res;
	int rc;

	cam_pulse = devm_kzalloc(&pdev->dev, sizeof(*cam_pulse), GFP_KERNEL);
	if (!cam_pulse)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cam_pulse->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cam_pulse->base)) {
		dev_err(&pdev->dev, "cannot get reg mem resource\n");
		return PTR_ERR(cam_pulse->base);
	}

	cam_pulse->dev = &pdev->dev;
	platform_set_drvdata(pdev, cam_pulse);

	rc = cam_pulse_parse_dt(pdev, &cam_pulse->cam_pulse_ctrl);
	if (rc < 0) {
		dev_err(&pdev->dev, "parse cam pulse dts node failed!\n");
		return rc;
	}

	rc = start_cam_pulse_gen(cam_pulse);
	if (rc < 0) {
		dev_err(&pdev->dev, "start cam pulse failed!\n");
		return rc;
	}

	dev_info(&pdev->dev, "%s successfully done!\n", __func__);

	return 0;
}

static int cam_pulse_remove(struct platform_device *pdev)
{
	int rc;
	struct cam_pulse_device *cam_pulse;

	cam_pulse = platform_get_drvdata(pdev);
	rc = stop_cam_pulse_gen(cam_pulse);
	if (rc < 0) {
		dev_err(&pdev->dev, "cam pulse stop failed!\n");
		return rc;
	}

	return 0;
}

static const struct of_device_id cam_pulse_of_match[] = {
	{ .compatible = CAM_PULSE_DT_NAME },
	{ },
};

MODULE_DEVICE_TABLE(of, cam_pulse_of_match);

static struct platform_driver cam_pulse_driver = {
	.probe	= cam_pulse_probe,
	.remove = cam_pulse_remove,
	.driver = {
		.name = CAM_PULSE_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_pulse_of_match,
	}
};

static int __init cam_pulse_init_module(void)
{
	return platform_driver_register(&cam_pulse_driver);
}

static void __exit cam_pulse_exit_module(void)
{
	platform_driver_unregister(&cam_pulse_driver);
}

module_init(cam_pulse_init_module);
module_exit(cam_pulse_exit_module);

MODULE_DESCRIPTION("VeriSilicon Camera Pulse Generater Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VS-CAM-PULSE");