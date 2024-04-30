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

/**
 * @file hobot_mipi_host.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#ifdef CONFIG_PM_SLEEP
#include <linux/suspend.h>
#endif

#include "hobot_mipi_csi.h"
#include "hobot_mipi_host.h"
#include "hobot_mipi_host_ops.h"

#ifdef CONFIG_HOBOT_FUSA_DIAG
EXPORT_SYMBOL_GPL(hobot_mipi_host_stl_setup_do); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
#endif

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device open
 *
 * @param[in] index: mipi host(rx) device index
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
static int32_t mipi_host_open(int32_t index)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_host_open_do(hdev);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device close
 *
 * @param[in] index: mipi host(rx) device index
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
static int32_t mipi_host_close(int32_t index)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_host_close_do(hdev);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device ioctl operation
 *
 * @param[in] index: mipi host(rx) device index
 * @param[in] cmd: ioctl command
 * @param[in] arg: ioctl arg
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
static int32_t mipi_host_ioctl(int32_t index, uint32_t cmd, unsigned long arg)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_host_ioctl_do(hdev, cmd, arg);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device callback set
 *
 * @param[in] index: mipi host(rx) device index
 * @param[in] drop_cb: frame drop callbcak
 * @param[in] int_cb: interrupt callback for error
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
static int32_t mipi_host_setcb(int32_t index, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_host_setcb_do(hdev, drop_cb, int_cb);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device sys operation
 *
 * @param[in] index: mipi host(rx) device index
 * @param[in] type: sys operation type
 * @param[in] sub: sys operation sub type show or stroe
 * @param[in] name: sys node name
 * @param[in] buf: info to show or store
 * @param[in] count: buf size

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
static int32_t mipi_host_sys(int32_t index, int32_t type, int32_t sub, const char *name, char *buf, int32_t count)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_host_sys_do(type, sub, hdev, name, buf, count);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device get osdev struct pointer operation
 *
 * @param[in] index: mipi host(rx) device index

 * @return !NULL:Success os_dev point, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static os_dev_t *mipi_host_osdev(int32_t index)
{
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(index);

	if (hdev == NULL) {
		return NULL;
	}

	return &hdev->osdev;
}

/**
 * @brief mipi host driver ops
 */
static const struct mipi_sub_ops_s mipi_host_ops = {
	.open = mipi_host_open,
	.close = mipi_host_close,
	.ioctl = mipi_host_ioctl,
	.setcb = mipi_host_setcb,
	.sys = mipi_host_sys,
	.osdev = mipi_host_osdev,
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device interrupt handle
 *
 * @param[in] this_irq: mipi host(rx) irq number
 * @param[in] data: mipi host(rx) device struct
 *
 * @return IRQ_HANDLED:Success, IRQ_NONE:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static irqreturn_t mipi_host_irq_func(int32_t this_irq, void *data)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)data;

	if (hdev == NULL) {
		return IRQ_NONE;
	}

	if (this_irq >= 0) {
		disable_irq_nosync((uint32_t)this_irq);
	}

	hobot_mipi_host_irq_func_do(hdev);

	if (this_irq >= 0) {
		enable_irq((uint32_t)this_irq);
	}

	return IRQ_HANDLED;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device remove
 *
 * @param[in] pdev: mipi host(rx) platform device
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
static int32_t hobot_mipi_host_remove(struct platform_device *pdev)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)platform_get_drvdata(pdev);
	struct mipi_host_s *host = &hdev->host;

#if 0
	hobot_mipi_host_remove_cdev(hdev);
#endif
#if MIPI_HOST_INT_DBG
#ifdef MIPI_HOST_INT_USE_TIMER
	(void)osal_timer_stop(&hdev->irq_timer);
#else
#ifdef MIPI_HOST_J5_CIMDMA_OPTIMIZE
	(void)irq_set_affinity_hint((uint32_t)host->irq, NULL); /* qacfix: conversion */
#endif
	(void)devm_free_irq(&pdev->dev, (uint32_t)host->irq, (void *)hdev);
#endif
#endif
	devm_iounmap(&pdev->dev, host->iomem);
	hobot_mipi_host_remove_do(hdev);
	devm_kfree(&pdev->dev, (void *)hdev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device creat from platfrom
 *
 * @param[in] pdev: mipi host(rx) platform device
 * @param[out] phdev: hdev point alloc created with port index
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
static int32_t hobot_mipi_host_probe_alloc_hdev(struct platform_device *pdev, struct mipi_hdev_s **phdev)
{
	struct mipi_hdev_s *hdev;
	int32_t port;

	port = of_alias_get_id(pdev->dev.of_node, "mipihost");
	if (port >= MIPI_HOST_MAX_NUM) {
		mipi_err(NULL, "[%s] port %d >= %d overflow error\n\n", __func__, port,
				MIPI_HOST_MAX_NUM);
		return -ERANGE;
	}
	if (port < 0) {
		port = 0;
	}
	hdev = (struct mipi_hdev_s *)devm_kmalloc(&pdev->dev, sizeof(struct mipi_hdev_s), GFP_KERNEL);
	if (hdev == NULL) {
		mipi_err(NULL, "[%s] devm_kmalloc failed\n\n", __func__);
		return -ENOMEM;
	}
	(void)memset((void *)hdev, 0, sizeof(struct mipi_hdev_s));
	/* prepare: report */
	hdev->port = port;
	hdev->osdev.devno = port;
	*phdev = hdev;
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device iomem map from platform
 *
 * @param[in] pdev: mipi host(rx) platform device
 * @param[in] hdev: mipi host(rx) operation hdev struct
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
static int32_t hobot_mipi_host_probe_iomem_init(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct resource *res;
	int32_t ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->iomem)) {
		mipi_err(NULL, "[%s] get mem res error\n\n", __func__);
		ret = (int32_t)PTR_ERR(host->iomem);
		host->iomem = NULL;
	}
	host->reg.base = (uint32_t)res->start;
	host->reg.size = (uint32_t)(res->end - res->start + 1U);

	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device irq requeset from platform
 *
 * @param[in] pdev: mipi host(rx) platform device
 * @param[in] hdev: mipi host(rx) operation hdev struct
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
static int32_t hobot_mipi_host_probe_irq_init(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
#ifdef MIPI_HOST_INT_USE_TIMER
	osal_timer_init(&hdev->irq_timer, mipi_host_irq_timer_func, hdev);

	return 0;
#else
	struct mipi_host_s *host = &hdev->host;
	int32_t ret;

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		mipi_err(NULL, "[%s] get irq num error\n\n", __func__);
		host->irq = 0;
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, (uint32_t)host->irq,
				mipi_host_irq_func,
#ifdef MIPI_HOST_J5_CIMDMA_OPTIMIZE
				/* no thread for RT if cimdma optimize */
				IRQF_TRIGGER_HIGH + IRQF_NO_THREAD,
#else
				IRQF_TRIGGER_HIGH,
#endif
				dev_name(&pdev->dev),
				(void *)hdev);
	if (ret != 0) {
		mipi_err(NULL, "[%s] request irq error %d\n\n", __func__, ret);
	}
#ifdef MIPI_HOST_J5_CIMDMA_OPTIMIZE
	ret = irq_set_affinity_hint((uint32_t)host->irq, cpumask_of(MIPI_HOST_IRQ_CPUMASK));
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to set interrupt affinity!\n");
		return ret;
	}
#endif
	return ret;
#endif
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device snrclk init from platform
 *
 * @param[in] pdev: mipi host(rx) platform device
 * @param[in] hdev: mipi host(rx) operation hdev struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void hobot_mipi_host_probe_snrclk_init(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	struct mipi_host_s *host = &hdev->host;
	uint32_t node_val;
	int32_t ret;

	host->snrclk.pinctrl = (void *)devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->snrclk.pinctrl)) {
		host->snrclk.pinctrl = NULL;
	} else {
		host->snrclk.enable = (void *)pinctrl_lookup_state(host->snrclk.pinctrl,
			"enable");
		if (IS_ERR(host->snrclk.enable)) {
			host->snrclk.enable = NULL;
		}
		host->snrclk.disable = (void *)pinctrl_lookup_state(host->snrclk.pinctrl,
			"disable");
		if (IS_ERR(host->snrclk.disable)) {
			host->snrclk.disable = NULL;
		}
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"snrclk-idx", &node_val);
	host->snrclk.index = (ret != 0) ? -1 : (int32_t)node_val;

	ret = of_property_read_u32(pdev->dev.of_node,
			"clock-frequency", &node_val);
	if (ret == 0) {
		host->snrclk.probe_init = 1U;
		host->snrclk.probe_freq = node_val;
	} else {
		host->snrclk.probe_init = 0U;
		host->snrclk.probe_freq = 0U;
	}
#endif
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device free to exit
 *
 * @param[in] pdev: mipi host(rx) platform device
 * @param[in] hdev: mipi host(rx) operation hdev struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void hobot_mipi_host_probe_free(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
#if MIPI_HOST_INT_DBG
#ifdef MIPI_HOST_INT_USE_TIMER
	if (hdev->irq_timer.function != NULL) {
		(void)osal_timer_stop(&hdev->irq_timer);
		hdev->irq_timer.function = NULL;
	}
#else
	if (host->irq != 0) {
#ifdef MIPI_HOST_J5_CIMDMA_OPTIMIZE
		(void)irq_set_affinity_hint((uint32_t)host->irq, NULL); /* qacfix: conversion */
#endif
		(void)devm_free_irq(&pdev->dev, (uint32_t)host->irq, (void *)hdev);
		host->irq = 0;;
	}
#endif
#endif
	if (host->iomem != NULL) {
		devm_iounmap(&pdev->dev, host->iomem);
		host->iomem = NULL;
	}
	devm_kfree(&pdev->dev, (void *)hdev);
	platform_set_drvdata(pdev, NULL);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device probe from platform
 *
 * @param[in] pdev: mipi host(rx) platform device
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
static int32_t hobot_mipi_host_probe(struct platform_device *pdev)
{
	struct mipi_hdev_s *hdev;
	int32_t ret;
	uint32_t node_val;

	/* prepare: hdev */
	ret = hobot_mipi_host_probe_alloc_hdev(pdev, &hdev);
	if (ret != 0) {
		return ret;
	}
	/* prepare: base */
	platform_set_drvdata(pdev, (void *)hdev);
	/* prepare: reg */
	ret = hobot_mipi_host_probe_iomem_init(pdev, hdev);
	if(ret != 0) {
		hobot_mipi_host_probe_free(pdev, hdev);
		return ret;
	}
	/* prepare: irq */
#if MIPI_HOST_INT_DBG
	ret = hobot_mipi_host_probe_irq_init(pdev, hdev);
	if (ret != 0) {
		hobot_mipi_host_probe_free(pdev, hdev);
		return ret;
	}
#endif
	/* prepare: snrclk */
	hobot_mipi_host_probe_snrclk_init(pdev, hdev);
	/* prepare: mode */
	ret = of_property_read_u32(pdev->dev.of_node,
			"ap-mode", &node_val);
	if (ret != 0) {
		node_val = 0U;
	}
	hdev->host.ap = (int32_t)node_val;
	ret = of_property_read_u32(pdev->dev.of_node,
			"hw-mode", &node_val);
	if (ret != 0) {
		node_val = 0U;
	}
	hdev->host.hw = (int32_t)node_val;

	/* do: probe */
	ret = hobot_mipi_host_probe_do(hdev);
	if (ret < 0) {
		hobot_mipi_host_probe_free(pdev, hdev);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host device suspend
 *
 * @param[in] dev: mipi host device
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
static int32_t hobot_mipi_host_suspend(struct device *dev)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE) {
		return 0;
	}

	return hobot_mipi_host_suspend_do(hdev);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host device resume
 *
 * @param[in] dev: mipi host device
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
static int32_t hobot_mipi_host_resume(struct device *dev)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE) {
		return 0;
	}

	return hobot_mipi_host_resume_do(hdev);
}

static const struct dev_pm_ops hobot_mipi_host_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS((hobot_mipi_host_suspend),
			(hobot_mipi_host_resume))
};
#endif

static const struct of_device_id hobot_mipi_host_match[] = {
	{.compatible = "hobot,mipi-host"},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_mipi_host_match);

static struct platform_driver hobot_mipi_host_driver = {
	.probe	= hobot_mipi_host_probe,
	.remove = hobot_mipi_host_remove,
	.driver = {
		.name = MIPI_HOST_DNAME,
		.of_match_table = hobot_mipi_host_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &hobot_mipi_host_dev_pm_ops,
#endif
	},
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) driver setup to probe devices
 *
 * @param[in] rx_ops: rx ops struct to store for csi driver
 *
 * @return >=0:Success and return host device valid mask, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_setup(const struct mipi_sub_ops_s **rx_ops)
{
	int32_t ret = 0, i;

	if (rx_ops != NULL) {
		ret = platform_driver_register(&hobot_mipi_host_driver);
		if (ret == 0) {
			for (i = 0; i < MIPI_HOST_MAX_NUM; i++) {
				if (hobot_mipi_host_hdev(i) != NULL)
					ret |= (0x1 << i);
			}
			*rx_ops = &mipi_host_ops;
		}
	} else {
		platform_driver_unregister(&hobot_mipi_host_driver);
	}

	return ret;
}

