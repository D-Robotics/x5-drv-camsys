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
 * @file hobot_mipi_debug_pdev.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>

#include "hobot_mipi_phy.h"
#include "hobot_mipi_phy_ops.h"

/**
 * @struct mipi_dpdev_s
 * mipi phy debug struct for char device
 * @NO{S10E03C01}
 */
typedef struct mipi_dpdev_s {
	struct os_dev      osdev;
	struct class      *p_class;
	const struct mipi_phy_ops_s *ops;
	const struct mipi_phy_dbg_ops_s *dops;
} mipi_dpdev_t;

/* global var */
/* the major id of device, valid if not 0.
 * see: hobot_mipi_dphy_probe_cdev, hobot_mipi_dphy_remove_cdev
 */
static uint32_t g_mp_major;
/**
 * @var g_dpdevs
 * the struct array of debug devices.
 */
static struct mipi_dpdev_s g_dpdevs;

static int32_t hobot_mipi_phy_regs_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s for reg control %p\n",
			MIPI_PHY_DNAME, unused);
	return 0;
}

static int32_t hobot_mipi_phy_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hobot_mipi_phy_regs_show, inode->i_private);
}

static mipi_ioc_ret_t hobot_mipi_phy_regs_ioctl(struct file *file, uint32_t cmd, mipi_ioc_arg_t arg)
{
	int32_t ret = -ENODEV;

	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->ioctl != NULL))
		ret = g_dpdevs.ops->ioctl(cmd, arg);

	return ret;
}
static const struct file_operations hobot_mipi_phy_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_mipi_phy_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.unlocked_ioctl = hobot_mipi_phy_regs_ioctl,
	.compat_ioctl = hobot_mipi_phy_regs_ioctl,
};

/* sysfs show for mipi phy devices' param/ */
static ssize_t mipi_phy_param_show(struct device *dev,/* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_PARAM, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs store for mipi phy devices' param/ */
static ssize_t mipi_phy_param_store(struct device *dev,/* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_PARAM, MIPI_SYS_STORE,
			attr->attr.name, (char *)buf, count);
	return ret;
}

/* sysfs for mipi phy devices' param */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(dbg_value, (S_IWUSR | S_IRUGO), mipi_phy_param_show, mipi_phy_param_store);
#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
static DEVICE_ATTR(stl_dbg, (S_IWUSR | S_IRUGO), mipi_phy_param_show, mipi_phy_param_store);
static DEVICE_ATTR(stl_mask, (S_IWUSR | S_IRUGO), mipi_phy_param_show, mipi_phy_param_store);
static DEVICE_ATTR(stl_pile, (S_IWUSR | S_IRUGO), mipi_phy_param_show, mipi_phy_param_store);
#endif
/* PRQA S ALL -- */

static struct attribute *param_attr[] = {
	&dev_attr_dbg_value.attr,
#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	&dev_attr_stl_dbg.attr,
	&dev_attr_stl_mask.attr,
	&dev_attr_stl_pile.attr,
#endif
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs show for mipi phy devices' status/info */
static ssize_t mipi_phy_status_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_STATUS_INFO, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi phy devices' status/host */
static ssize_t mipi_phy_status_host_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_STATUS_HOST, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi phy devices' status/dev */
static ssize_t mipi_phy_status_dev_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_STATUS_DEV, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi phy devices' status/regs */
static ssize_t mipi_phy_status_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_STATUS_REGS, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs attr for mipi phy devices' status */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(info, S_IRUGO, mipi_phy_status_info_show, NULL);
static DEVICE_ATTR(host, S_IRUGO, mipi_phy_status_host_show, NULL);
static DEVICE_ATTR(dev, S_IRUGO, mipi_phy_status_dev_show, NULL);
static DEVICE_ATTR(regs, S_IRUGO, mipi_phy_status_regs_show, NULL);
/* PRQA S ALL -- */

static struct attribute *status_attr[] = {
	&dev_attr_info.attr,
	&dev_attr_host.attr,
	&dev_attr_dev.attr,
	&dev_attr_regs.attr,
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

#ifndef CONFIG_FAULT_INJECTION_ATTR
/* sysfs show for mipi phy devices' fault_injection */
static ssize_t mipi_phy_fault_injection_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_FAULT_INJECT, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs store for mipi phy devices' fault_injection */
static ssize_t mipi_phy_fault_injection_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t ret = -ENODEV;
	if ((g_dpdevs.ops != NULL) && (g_dpdevs.ops->sys != NULL))
		ret = g_dpdevs.ops->sys(MIPI_PHY_SYS_FAULT_INJECT, MIPI_SYS_STORE,
			attr->attr.name, (char *)buf, count);
	return ret;
}

/* sysfs for mipi phy devices' fault_injection */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(fault_injection, (S_IWUSR | S_IRUGO), mipi_phy_fault_injection_show, mipi_phy_fault_injection_store);
/* PRQA S ALL -- */

static struct attribute *fault_injection_attr[] = {
	&dev_attr_fault_injection.attr,
	NULL,
};

static const struct attribute_group fault_injection_attr_group = {
	.name = NULL,
	.attrs = fault_injection_attr,
};
#endif

/* sysfs attr groups */
static const struct attribute_group *attr_groups[] = {
	&param_attr_group,
	&status_attr_group,
#ifndef CONFIG_FAULT_INJECTION_ATTR
	&fault_injection_attr_group,
#endif
	NULL,
};

static int32_t hobot_mipi_phy_probe_cdev(struct mipi_dpdev_s *dpdev)
{
	int32_t ret;
	struct os_dev *osdev = &dpdev->osdev;
	struct cdev *p_cdev = &osdev->cdev;
	struct class *p_class = NULL;

	if ((dpdev->dops != NULL) && (dpdev->dops->class != NULL))
		p_class = (struct class *)dpdev->dops->class(1);
	if(p_class == NULL) {
		mipi_info(NULL, "[%s] no class support\n", __func__);
		return 0;
	}

	osdev->devno = MKDEV(g_mp_major, 0U);
	cdev_init(p_cdev, &hobot_mipi_phy_regs_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, osdev->devno, 1);
	if (ret != 0) {
		mipi_err(NULL, "[%s] cdev add error %d\n", __func__, ret);
		dpdev->dops->class(0);
		return ret;
	}
	osdev->dev = device_create_with_groups(p_class, NULL,
			osdev->devno, dpdev,
			attr_groups, MIPI_PHY_DNAME);
	if (IS_ERR((void *)osdev->dev)) {
		ret = (int32_t)(PTR_ERR((void *)osdev->dev));
		osdev->dev = NULL;
		mipi_err(NULL, "[%s] deivce create error %d\n", __func__, ret);
		cdev_del(&osdev->cdev);
		dpdev->dops->class(0);
		return ret;
	}
	dpdev->p_class = p_class;

	return 0;
}

static void hobot_mipi_phy_remove_cdev(struct mipi_dpdev_s *dpdev)
{
	struct os_dev *osdev = &dpdev->osdev;

	if (dpdev->p_class != NULL) {
		device_destroy(dpdev->p_class, osdev->devno);
		cdev_del(&osdev->cdev);
	}

	osdev->dev = NULL;
	return;
}

int32_t hobot_mipi_phy_debug_remove(void)
{
	struct mipi_dpdev_s *dpdev = &g_dpdevs;

	mipi_info(NULL, "phy debug remove\n");
	(void)hobot_mipi_phy_remove_cdev(dpdev);

	if ((dpdev->dops != NULL) && (dpdev->dops->class != NULL))
		dpdev->dops->class(0);

	if (g_mp_major != 0U) {
		unregister_chrdev_region((dev_t)(MKDEV(g_mp_major, 0U)), 1U);
		g_mp_major = 0U;
	}

	return 0;
}

int32_t hobot_mipi_phy_debug_probe(const struct mipi_phy_ops_s *ops,
		const struct mipi_phy_dbg_ops_s *dops)
{
	struct mipi_dpdev_s *p_dpdev;
	dev_t devno;
	int32_t ret = 0;

	p_dpdev = &g_dpdevs;
	(void)memset((void *)p_dpdev, 0, sizeof(struct mipi_dpdev_s));

	if (g_mp_major == 0U) {
		ret = alloc_chrdev_region(&devno, 0, 1U, MIPI_PHY_DNAME);
		if (ret < 0) {
			mipi_err(NULL, "[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_PHY_DNAME, ret);
			return ret;
		}
		g_mp_major = MAJOR(devno);
	}

	/* prepare: base */
	p_dpdev->ops = ops;
	p_dpdev->dops = dops;
	/* prepare: cdev */
	ret = hobot_mipi_phy_probe_cdev(p_dpdev);
	if (ret != 0) {
		mipi_err(NULL, "[%s] hobot_mipi_phy_probe_cdev error %d\n", __func__, ret);
		return ret;
	}
	if ((p_dpdev->ops != NULL) && (p_dpdev->ops->osdev != NULL)) {
		struct os_dev *osdev = p_dpdev->ops->osdev();
		if ((osdev != NULL) && (osdev->dev == NULL))  {
			osdev->dev = p_dpdev->osdev.dev;
		}
	}
	mipi_info(NULL, "phy debug probed\n");

	return 0;
}

