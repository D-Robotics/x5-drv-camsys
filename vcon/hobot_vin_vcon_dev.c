/*
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_vin_vcon_dev.c
 *
 * @NO{S10E01C02}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/timer.h>
#include <linux/pinctrl/consumer.h>


#include "hobot_vin_vcon_ops.h"

/**
 * @var dummy_ok
 * dummy is ok if no sub insmod
 */
static uint32_t dummy_ok = 0;
module_param(dummy_ok, uint, 0644);

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get the flag of dummy if ok?
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t vcon_sub_dummy_return(void)
{
	return (dummy_ok) ? 0 : (-EPERM);
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcont probe get index a device struct from platform
 *
 * @param[in] pdev: vin vcon platform device
 * @param[in] pvcon: vin vcon device struct point store addr
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_probe_get_vcon(struct platform_device *pdev, struct vcon_device_s **pvcon)
{
	int32_t index;

	index = of_alias_get_id(pdev->dev.of_node, "vcon");
	if (index >= VCON_DEV_MAX_NUM) {
		vcon_err(NULL, "[%s] vcon %d >= %d overflow error\n", __func__, index,
				VCON_DEV_MAX_NUM);
		return -ERANGE;
	}
	if (index < 0) {
		index = 0;
	}

	return vcon_device_init_get(index, pvcon);
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief read vcon read int array from of platform dts
 *
 * @param[in] pdev: vin vcon platform device
 * @param[in] name: name string of dts array
 * @param[in] pvals: the array buffer to store values
 * @param[in] num: the array max size to read
 * @param[in] dft: the default value if not read done
 * @param[in] valid: valid value to return if not 0
 *
 * @return valid mask of read operation
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_of_read_array(struct platform_device *pdev, const char *name,
		int32_t *pvals, int32_t num, int32_t dft, int32_t valid)
{
	int32_t i, ret;

	ret = of_property_read_variable_u32_array(pdev->dev.of_node, name, (uint32_t *)pvals, 1, num);
	if (ret < 0) {
		i = 0;
		ret = 0;
	} else {
		i = ret;
		ret = (valid != 0) ? valid : ((0x1 << i) - 1);
	}
	while (i < num) {
		pvals[i] = dft;
		i++;
	}

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon device attr init from dts
 *
 * @param[in] pdev: vin vcon platform device
 * @param[in] vcon: vin vcon device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_probe_attr_init(struct platform_device *pdev, struct vcon_device_s *vcon)
{
	int32_t ret, valid = 0;
	struct vcon_attr_s *info = &vcon->info;
	struct os_dev *dev = &vcon->osdev;

	(void)hobot_vin_vcon_of_read_array(pdev, "type", &info->vcon_type, 1, VCON_SELF, 0);
	if ((info->vcon_type == VCON_MAIN) || (info->vcon_type == VCON_FOLLOW)) {
		ret = hobot_vin_vcon_of_read_array(pdev, "link", &info->vcon_link, 1, VCON_ATTR_INVALID, 0);
		if (ret == 0) {
			vcon_err(dev, "no link config for type %d error\n",
				info->vcon_type);
		} else if (info->vcon_link == vcon->index) {
			vcon_err(dev, "link %d self for type %d error\n",
				info->vcon_link, info->vcon_type);
		} else if (info->vcon_link > VCON_DEV_MAX_NUM) {
			vcon_err(dev, "link %d over max %d for type %d error\n",
				info->vcon_link, VCON_DEV_MAX_NUM, info->vcon_type);
		} else {
			valid |= VCON_ATTR_V_TYPE;
		}
	} else if (info->vcon_type != VCON_SELF) {
		vcon_info(dev, "type %d invalid force to self\n",
			info->vcon_type);
		info->vcon_type = VCON_SELF;
		valid |= VCON_ATTR_V_TYPE;
	} else {
		valid |= VCON_ATTR_V_TYPE;
	}
	valid |= hobot_vin_vcon_of_read_array(pdev, "bus", &info->bus_main,
			2, VCON_ATTR_INVALID, VCON_ATTR_V_BUS_MAIN);
	if (info->bus_second != VCON_ATTR_INVALID)
		valid |= VCON_ATTR_V_BUS_SEC;
	valid |= hobot_vin_vcon_of_read_array(pdev, "gpio_poc", &info->gpios[VGPIO_POC_BASE],
			VGPIO_POC_NUM, VCON_ATTR_INVGPIO, VCON_ATTR_V_GPIO_POC);
	valid |= hobot_vin_vcon_of_read_array(pdev, "gpio_des", &info->gpios[VGPIO_DES_BASE],
			VGPIO_DES_NUM, VCON_ATTR_INVGPIO, VCON_ATTR_V_GPIO_DES);
	valid |= hobot_vin_vcon_of_read_array(pdev, "gpio_ser", &info->gpios[VGPIO_SER_BASE],
			VGPIO_SER_NUM, VCON_ATTR_INVGPIO, VCON_ATTR_V_GPIO_SER);
	valid |= hobot_vin_vcon_of_read_array(pdev, "gpio_oth", &info->gpios[VGPIO_OTH_BASE],
			VGPIO_OTH_NUM, VCON_ATTR_INVGPIO, VCON_ATTR_V_GPIO_OTH);
	valid |= hobot_vin_vcon_of_read_array(pdev, "sensor_err", info->sensor_err,
			VCON_SENSOR_ERR_MAX, VCON_ATTR_INVALID, VCON_ATTR_V_SENSOR_ERR);
	valid |= hobot_vin_vcon_of_read_array(pdev, "lpwm_chn", info->lpwm_chn,
			VCON_LPWM_CHN_MAX, VCON_ATTR_INVALID, VCON_ATTR_V_LPWM);
	valid |= hobot_vin_vcon_of_read_array(pdev, "rx_phy", &info->rx_phy_mode,
			3, VCON_ATTR_INVALID, VCON_ATTR_V_MIPI_RX);
	valid |= hobot_vin_vcon_of_read_array(pdev, "tx_phy", &info->tx_phy_mode,
			3, VCON_ATTR_INVALID, VCON_ATTR_V_MIPI_TX);

	info->attr_valid = valid;
	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon device free to exit
 *
 * @param[in] pdev: vin vcon platform device
 * @param[in] vcon: vin vcon operation struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void hobot_vin_vcon_probe_free(struct platform_device *pdev, struct vcon_device_s *vcon)
{
	platform_set_drvdata(pdev, NULL);
	vcon_device_exit_put(vcon);
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon device probe from platform
 *
 * @param[in] pdev: vin vcon platform device
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_probe(struct platform_device *pdev)
{
	struct vcon_device_s *vcon;
	int32_t ret;
	/* uint32_t node_val; */

	/* prepare: hdev */
	ret = hobot_vin_vcon_probe_get_vcon(pdev, &vcon);
	if (ret != 0) {
		return ret;
	}
	/* prepare: base */
	platform_set_drvdata(pdev, (void *)vcon);
	/* prepare: reg */
	ret = hobot_vin_vcon_probe_attr_init(pdev, vcon);
	if(ret < 0) {
		hobot_vin_vcon_probe_free(pdev, vcon);
		return ret;
	}
	ret = vcon_device_attr_parse(vcon);
	if(ret < 0) {
		hobot_vin_vcon_probe_free(pdev, vcon);
		return ret;
	}
	vcon_device_attr_show(vcon);

	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon device remove from platform
 *
 * @param[in] pdev: vin vcon platform device
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_remove(struct platform_device *pdev)
{
	struct vcon_device_s *vcon = (struct vcon_device_s *)platform_get_drvdata(pdev);

	hobot_vin_vcon_probe_free(pdev, vcon);

	return 0;
}


#ifdef CONFIG_PM_SLEEP
/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vin vcon device suspend
 *
 * @param[in] dev: vin vcon device
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_suspend(struct device *dev)
{
	/* struct vcon_device_s *vcon = (struct vcon_device_s*)dev_get_drvdata(dev); */

	// TODO:

	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vin vcon device resume
 *
 * @param[in] dev: vin vcon device
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_vin_vcon_resume(struct device *dev)
{
	/* struct vcon_device_s *vcon = (struct vcon_device_s*)dev_get_drvdata(dev); */

	// TODO:

	return 0;
}

static const struct dev_pm_ops hobot_vin_vcon_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS((hobot_vin_vcon_suspend),
			(hobot_vin_vcon_resume))
};
#endif

/**
 * @var hobot_vin_vcon_match
 * vcon driver match table
 */
static const struct of_device_id hobot_vin_vcon_match[] = {
	{.compatible = "hobot,vin-vcon"},
	{}
};
MODULE_DEVICE_TABLE(of, hobot_vin_vcon_match);

/**
 * @var hobot_vin_vcon_driver
 * vcon driver struct
 */
static struct platform_driver hobot_vin_vcon_driver = {
	.probe	= hobot_vin_vcon_probe,
	.remove = hobot_vin_vcon_remove,
	.driver = {
		.name = VIN_VCON_DNAME,
		.of_match_table = hobot_vin_vcon_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &hobot_vin_vcon_dev_pm_ops,
#endif
	},
};

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon vin vcon driver init
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_vin_vcon_module_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&hobot_vin_vcon_driver);
	if (ret < 0)
		return ret;

	ret = vcon_driver_init_done();
	if (ret < 0) {
		platform_driver_unregister(&hobot_vin_vcon_driver);
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon vin vcon driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_vin_vcon_module_exit(void)
{
	platform_driver_unregister(&hobot_vin_vcon_driver);
	vcon_driver_exit_done();
}

late_initcall_sync(hobot_vin_vcon_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_vin_vcon_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT VIN VCON Driver"); /* PRQA S ALL */ /* linux macro */
