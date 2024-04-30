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
 * @file hobot_mipi_phy.c
 *
 * @NO{S10E03C02}
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
#include <linux/seq_file.h>
#include <linux/of.h>

#include "hobot_mipi_phy_ops.h"

/* module params of reged num */
module_param(host_num, uint, 0444); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(dev_num, uint, 0444); /* PRQA S 0605,0636,4501 */ /* module_param macro */

/* module params: txout freq */
module_param(txout_freq_mode, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(txout_freq_autolarge_enbale, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(txout_freq_gain_precent, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(txout_freq_force, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */

/* module params: rxdphy vref */
module_param(rxdphy_vrefcd_lprx, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(rxdphy_v400_prog, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(rxdphy_deskew_cfg, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */

EXPORT_SYMBOL_GPL(mipi_phy_register); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_phy_unregister); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_host_dphy_initialize); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_host_dphy_reset); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_host_dphy_set_source);
EXPORT_SYMBOL_GPL(mipi_dev_dphy_initialize); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dev_dphy_reset); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_get_ctl); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_set_ctl); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_get_freqrange); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_set_freqrange); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_get_lanemode); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_set_lanemode); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
EXPORT_SYMBOL_GPL(mipi_dphy_outclk_config); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */

#ifdef CONFIG_HOBOT_FUSA_DIAG
EXPORT_SYMBOL_GPL(hobot_mipi_phy_stl_setup_do); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
#endif

const struct mipi_phy_dbg_ops_s *g_phy_dbg_ops;

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device ioctl operation
 *
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
static int32_t mipi_phy_ioctl(uint32_t cmd, unsigned long arg)
{
	struct mipi_pdev_s *pdev = hobot_mipi_phy_pdev();

	if (pdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_phy_ioctl_do(pdev, cmd, arg);
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device sys operation
 *
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
static int32_t mipi_phy_sys(int32_t type, int32_t sub, const char *name, char *buf, int32_t count)
{
	struct mipi_pdev_s *pdev = hobot_mipi_phy_pdev();

	if (pdev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_phy_sys_do(type, sub, pdev, name, buf, count);
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device get osdev struct pointer operation
 *
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
static os_dev_t *mipi_phy_osdev(void)
{
	struct mipi_pdev_s *pdev = hobot_mipi_phy_pdev();

	if (pdev == NULL) {
		return NULL;
	}

	return &pdev->osdev;
}

/**
 * @brief mipi phy driver ops
 */
static const struct mipi_phy_ops_s mipi_phy_ops = {
	.ioctl = mipi_phy_ioctl,
	.sys = mipi_phy_sys,
	.osdev = mipi_phy_osdev,
};

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy debug setup
 *
 * @param[in] phy_dbg_ops: phy debug callback struct for setup
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_phy_debug_setup(const mipi_phy_dbg_ops_t *phy_dbg_ops)
{
	int32_t ret;

	if ((phy_dbg_ops != NULL) && (g_phy_dbg_ops != NULL))
		return -EBUSY;

	if ((phy_dbg_ops != NULL) && (phy_dbg_ops->setup != NULL)) {
		ret = phy_dbg_ops->setup(&mipi_phy_ops);
		if (ret < 0)
			return ret;
	} else if ((g_phy_dbg_ops != NULL) && (g_phy_dbg_ops->setup != NULL)) {
		g_phy_dbg_ops->setup(NULL);
	}
	g_phy_dbg_ops = phy_dbg_ops;
	return 0;
}
EXPORT_SYMBOL_GPL(hobot_mipi_phy_debug_setup); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device remove
 *
 * @param[in] pdev: mipi phy platform device
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
static int32_t hobot_mipi_phy_remove(struct platform_device *pdev)
{
	struct mipi_pdev_s *p_pdev = (struct mipi_pdev_s *)platform_get_drvdata(pdev);
	struct mipi_dphy_s *dphy = &p_pdev->dphy;

	if (dphy->iomem != NULL) {
		devm_iounmap(&pdev->dev, dphy->iomem);
		dphy->iomem = NULL;
	}
#if MIPI_DPHY_CLK_24M_OUT_J5
	if (dphy->outclk.iomem != NULL) {
		devm_iounmap(&pdev->dev, dphy->outclk.iomem);
		dphy->outclk.iomem = NULL;
	}
#endif
#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	if (dphy->erm.iomem != NULL) {
		devm_iounmap(&pdev->dev, dphy->erm.iomem);
		dphy->erm.iomem = NULL;
	}
#endif
	hobot_mipi_phy_remove_do(p_pdev);
	devm_kfree(&pdev->dev, (void *)p_pdev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void hobot_mipi_phy_probe_res(struct platform_device *pdev, struct mipi_pdev_s *p_pdev)
{
	struct mipi_dphy_s *dphy = &p_pdev->dphy;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, MIPI_DPHY_MEM_RES_CTRL_IDX);
	if (res != NULL) {
		dphy->iomem = devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if (IS_ERR(dphy->iomem)) {
			dphy->iomem = NULL;
		}
		dphy->reg.base = (uint32_t)(res->start);
		dphy->reg.size = (uint32_t)(res->end - res->start + 1U);
	}

#if MIPI_DPHY_CLK_24M_OUT_J5
	res = platform_get_resource(pdev, IORESOURCE_MEM, MIPI_DPHY_MEM_RES_OUTCLK_IDX);
	if (res != NULL) {
		dphy->outclk.iomem = devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if (IS_ERR(dphy->outclk.iomem)) {
			dphy->outclk.iomem = NULL;
		}
		dphy->outclk.reg.base = (uint32_t)(res->start);
		dphy->outclk.reg.size = (uint32_t)(res->end - res->start + 1U);
	}
#endif

#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	res = platform_get_resource(pdev, IORESOURCE_MEM, MIPI_DPHY_MEM_RES_STLERM_IDX);
	if (res != NULL) {
		dphy->erm.iomem = devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if (IS_ERR(dphy->erm.iomem)) {
			dphy->erm.iomem = NULL;
		}
		dphy->erm.reg.base = (uint32_t)(res->start);
		dphy->erm.reg.size = (uint32_t)(res->end - res->start + 1U);
	}
#endif
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device probe
 *
 * @param[in] pdev: mipi phy platform device
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
static int32_t hobot_mipi_phy_probe(struct platform_device *pdev)
{
	int32_t ret;
	struct mipi_pdev_s *p_pdev;

	p_pdev = (struct mipi_pdev_s *)devm_kmalloc(&pdev->dev, sizeof(struct mipi_pdev_s), GFP_KERNEL);
	if (p_pdev == NULL) {
		mipi_err(NULL, "[%s] devm_kmalloc failed\n", __func__);
		return -ENOMEM;
	}
	(void)memset((void *)p_pdev, 0, sizeof(struct mipi_pdev_s));
	/* prepare: base */
	platform_set_drvdata(pdev, (void *)(p_pdev));
	/* prepare: reg */
	hobot_mipi_phy_probe_res(pdev, p_pdev);

	/* do: probe */
	ret = hobot_mipi_phy_probe_do(p_pdev);
	if (ret < 0) {
		hobot_mipi_phy_remove(pdev);
		return ret;
	}
	return 0;
}

static const struct of_device_id hobot_mipi_phy_match[] = {
	{.compatible = "hobot,mipi-dphy"},
	{.compatible = "hobot,mipi-phy"},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_mipi_phy_match);

static struct platform_driver hobot_mipi_phy_driver = {
	.probe  = hobot_mipi_phy_probe,
	.remove = hobot_mipi_phy_remove,
	.driver	= {
		.name = MIPI_PHY_DNAME,
		.of_match_table = hobot_mipi_phy_match,
	},
};

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy driver init
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
static int32_t __init hobot_mipi_phy_module_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&hobot_mipi_phy_driver);

	return ret;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_mipi_phy_module_exit(void)
{
	platform_driver_unregister(&hobot_mipi_phy_driver);
}

late_initcall(hobot_mipi_phy_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_mipi_phy_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT MIPI DPHY Driver"); /* PRQA S ALL */ /* linux macro */

