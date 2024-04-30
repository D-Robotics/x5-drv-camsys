/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt)    "[Camsys]:" fmt

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/poll.h>

#include "hobot_dev_camsys.h"
#include "hobot_camsys_ops.h"
#include "vio_config.h"

#define MODULE_NAME "CAM_SUBSYS"

static s32 camsys_suspend(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static s32 camsys_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static s32 camsys_runtime_suspend(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static s32 camsys_runtime_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops j6_camsys_pm_ops = {
	.suspend		= camsys_suspend,
	.resume			= camsys_resume,
	.runtime_suspend	= camsys_runtime_suspend,
	.runtime_resume		= camsys_runtime_resume,
};

#if 0
static ssize_t ipe0_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct j6_camsys_dev *camsys;

	camsys = dev_get_drvdata(dev);

	dev_info(dev, "/*********IPE0 REG DUMP***********/");
	ipe_hw_dump(camsys->base_reg[DEV_HW_IPE0]);

	return 0;
}
static DEVICE_ATTR(ipe0_regdump, 0444, ipe0_reg_dump, NULL);/*PRQA S 4501,0636*/

static ssize_t ipe1_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct j6_camsys_dev *camsys;

	camsys = dev_get_drvdata(dev);

	dev_info(dev, "/*********IPE1 REG DUMP***********/");
	ipe_hw_dump(camsys->base_reg[DEV_HW_IPE1]);

	return 0;
}
static DEVICE_ATTR(ipe1_regdump, 0444, ipe1_reg_dump, NULL);/*PRQA S 4501,0636*/

static ssize_t camsys_reg_dump(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	struct j6_camsys_dev *camsys;

	camsys = dev_get_drvdata(dev);

	dev_info(dev, "/*********CAMSYS REG DUMP***********/");
	camsys_hw_dump(camsys->base_reg[DEV_HW_CAM_SYS]);

	return 0;
}
static DEVICE_ATTR(camsys_regdump, 0444, camsys_reg_dump, NULL);/*PRQA S 4501,0636*/

struct camsys_interface_ops g_camsys_cops = {
		.ip_reset_func = camsys_set_module_reset,
};

DECLARE_VIO_CALLBACK_OPS(camsys_interface, 0, &g_camsys_cops);
#endif

static s32 camsys_probe(struct platform_device *pdev)
{
	s32 ret = 0;
#if 1
	struct device *dev = NULL;
	dev = &pdev->dev;
	ret = vio_get_clk(dev);
#else
	u32 i = 0;
	struct j6_camsys_dev *camsys;
	struct resource *mem_res;
	struct device *dev = NULL;
#ifdef CONFIG_HOBOT_CAMSYS_STL
	struct device_node *dnode;
#endif

	dev = &pdev->dev;

	camsys = devm_kzalloc(dev, sizeof(struct j6_camsys_dev), GFP_KERNEL);
	if (!camsys) {
		dev_err(dev, "%s camsys is NULL", __func__);
		ret = -ENOMEM;
		return ret;
	}

	for (i = DEV_HW_CAM_SYS; i < DEV_HW_NUM; i++) {
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!mem_res) {
			dev_err(dev, "Failed to get io memory region(%p)", mem_res);
			ret = -EBUSY;
			return ret;
		}

		camsys->regs_start[i] = mem_res->start;
		camsys->regs_end[i] = mem_res->end;
		camsys->base_reg[i] = devm_ioremap(&pdev->dev,
								mem_res->start, resource_size(mem_res));
		if (!camsys->base_reg[i]) {
			dev_err(dev, "Failed to remap io region(%p)", camsys->base_reg);
			ret = -ENOMEM;
			return ret;
		}
	}

	ret = device_create_file(dev, &dev_attr_ipe0_regdump);
	if (ret < 0) {
		dev_err(dev, "create ipe0 regdump failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_ipe1_regdump);
	if (ret < 0) {
		device_remove_file(dev, &dev_attr_ipe0_regdump);
		dev_err(dev, "create ipe1 regdump failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_camsys_regdump);
	if (ret < 0) {
		device_remove_file(dev, &dev_attr_ipe0_regdump);
		device_remove_file(dev, &dev_attr_ipe1_regdump);
		dev_err(dev, "create camsys regdump failed (%d)\n", ret);
		return ret;
	}
	ret = vio_get_clk(dev);

	osal_spin_init(&camsys->slock);/*PRQA S 3334*/
	camsys_set_global_ops(camsys);
	vio_register_callback_ops(&cb_camsys_interface, VIN_MODULE, COPS_7);

	platform_set_drvdata(pdev, camsys);
#endif
	dev_info(dev, "[FRT:D] %s(%d)\n", __func__, ret);

	return 0;
}

static s32 camsys_remove(struct platform_device *pdev)
{
	s32 ret = 0;
#if 1
	struct device *dev;
	dev = &pdev->dev;
	vio_put_clk(dev);
#else
	struct j6_camsys_dev *camsys;
	struct device *dev;

	if (pdev == NULL) {
		vio_err("%s g_camsys_dev = NULL\n", __func__);
		return -EFAULT;
	}
	camsys = platform_get_drvdata(pdev);
	dev = &pdev->dev;

	device_remove_file(&pdev->dev, &dev_attr_ipe0_regdump);
	device_remove_file(&pdev->dev, &dev_attr_ipe1_regdump);
	device_remove_file(&pdev->dev, &dev_attr_camsys_regdump);

	vio_unregister_callback_ops(VIN_MODULE, COPS_7);
	vio_put_clk(dev);

	devm_kfree(dev, camsys);
#endif
	dev_info(dev, "%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id j6_camsys_match[] = {
	{
		.compatible = "hobot,cam-subsys",
	},
	{},
};
MODULE_DEVICE_TABLE(of, j6_camsys_match);

static struct platform_driver j6_camsys_driver = {
	.probe		= camsys_probe,
	.remove 	= camsys_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &j6_camsys_pm_ops,
		.of_match_table = j6_camsys_match,
#if defined(CONFIG_FAULT_INJECTION_ATTR) && defined(CONFIG_HOBOT_CAMSYS_STL)
		.fault_injection_store = j6_camsys_fault_injection_store,
		.fault_injection_show = j6_camsys_fault_injection_show,
#endif
	}
};

#else
static struct platform_device_id j6_camsys_driver_ids[] = {
	{
		.name		= MODULE_NAME,
		.driver_data	= 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, j6_camsys_driver_ids);

static struct platform_driver j6_camsys_driver = {
	.probe		= camsys_probe,
	.remove		= __devexit_p(camsys_remove),
	.id_table	= j6_camsys_driver_ids,
	.driver	  = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &j6_camsys_pm_ops,
	}
};
#endif

static s32 __init j6_camsys_init(void)
{
	s32 ret = platform_driver_register(&j6_camsys_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(j6_camsys_init);/*PRQA S 0605*/

static void __exit j6_camsys_exit(void)
{
	platform_driver_unregister(&j6_camsys_driver);
}
module_exit(j6_camsys_exit);/*PRQA S 0605*/

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("J5 CAMSYS driver");
MODULE_LICENSE("GPL v2");
