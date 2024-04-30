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
 * @file hobot_mipi_debug_hdev.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/cdev.h>

#include "hobot_mipi_csi.h"
#include "hobot_mipi_host_ops.h"

/**
 * @struct mipi_dhdev_s
 * mipi host debug struct for char device
 * @NO{S10E03C01}
 */
typedef struct mipi_dhdev_s {
	int32_t            port;
	struct os_dev      osdev;
	struct class      *p_class;
	const struct mipi_sub_ops_s *ops;
	const struct mipi_dbg_ops_s *dops;
} mipi_dhdev_t;

/* global var */
/**
 * @var g_mh_majo
 * the major id of device, valid if not 0.
 * see: hobot_mipi_host_probe_cdev, hobot_mipi_host_remove_cdev
 */
static uint32_t g_mh_major;
/**
 * @var g_dhdevs
 * the struct array of debug devices.
 */
static struct mipi_dhdev_s *g_dhdevs;
/**
 * @var g_dhdev_mask
 * the number mask of debug devices.
 */
static uint32_t g_dhdev_mask;

static int32_t hobot_mipi_host_open(struct inode *inode, struct file *file) /* PRQA S 3673 */ /* linux cb func */
{
	struct os_dev *dev = (struct os_dev*)container_of(inode->i_cdev, struct os_dev, cdev); /* PRQA S ALL */ /* linux macro */
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)container_of(dev, struct mipi_dhdev_s, osdev); /* PRQA S ALL */ /* linux macro */
	int dbg = ((dhdev->dops != NULL) && (dhdev->dops->open != NULL)) ? 1 : 0;
	int ret = -EFAULT;

	if (dbg != 0)
		(void)dhdev->dops->open(MIPI_CSI_DBG_HOOKPRE, dhdev->port);

	if ((dhdev->ops != NULL) && (dhdev->ops->open != NULL))
		ret = dhdev->ops->open(dhdev->port);

	if (dbg != 0)
		(void)dhdev->dops->open((ret < 0) ? MIPI_CSI_DBG_HOOKERR : MIPI_CSI_DBG_HOOKPOST,
					dhdev->port);
	if (ret < 0)
		return ret;

	file->private_data = (void *)dhdev;
	return 0;
}

static int32_t hobot_mipi_host_close(struct inode *inode, struct file *file) /* PRQA S 3206,3673 */ /* linux cb func */
{
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)file->private_data;
	int dbg = ((dhdev->dops != NULL) && (dhdev->dops->close != NULL)) ? 1 : 0;
	int ret = -EFAULT;

	if (dbg != 0)
		(void)dhdev->dops->close(MIPI_CSI_DBG_HOOKPRE, dhdev->port);

	if ((dhdev->ops != NULL) && (dhdev->ops->close != NULL))
		ret = dhdev->ops->close(dhdev->port);

	if (dbg != 0)
		(void)dhdev->dops->close((ret < 0) ? MIPI_CSI_DBG_HOOKERR : MIPI_CSI_DBG_HOOKPOST,
					dhdev->port);
	return ret;
}

static mipi_ioc_ret_t hobot_mipi_host_ioctl(struct file *file, uint32_t cmd, mipi_ioc_arg_t arg) /* PRQA S 3673 */ /* linux cb func */
{
	struct mipi_dhdev_s *dhdev;
	int dbg;
	int ret = -EFAULT;

	/* Check type and command number */
	if ((file == NULL) || (file->private_data == NULL)) {
		/* do not need report */
		return -ENODEV;
	}
	dhdev = (struct mipi_dhdev_s *)file->private_data;
	dbg = ((dhdev->dops != NULL) && (dhdev->dops->ioctl != NULL)) ? 1 : 0;

	if (dbg != 0)
		(void)dhdev->dops->ioctl(MIPI_CSI_DBG_HOOKPRE, dhdev->port, cmd, arg);

	if ((dhdev->ops != NULL) && (dhdev->ops->ioctl != NULL))
		ret = dhdev->ops->ioctl(dhdev->port, cmd, arg);

	if (dbg != 0)
		(void)dhdev->dops->ioctl((ret < 0) ? MIPI_CSI_DBG_HOOKERR : MIPI_CSI_DBG_HOOKPOST,
					dhdev->port, cmd, arg);
	return ret;
}

static const struct file_operations hobot_mipi_host_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_mipi_host_open,
	.release	= hobot_mipi_host_close,
	.unlocked_ioctl = hobot_mipi_host_ioctl,
	.compat_ioctl = hobot_mipi_host_ioctl,
};

/* sysfs show for mipi host devices' param/ */
static ssize_t mipi_host_param_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3673 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_PARAM, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs store for mipi host devices' param/ */
static ssize_t mipi_host_param_store(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, const char *buf, size_t count) /* PRQA S 3673 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_PARAM, MIPI_SYS_STORE,
			attr->attr.name, (char *)buf, count);
	return ret;
}

/* sysfs for mipi host devices' param */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(nocheck, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(notimeout, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(wait_ms, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(dbg_value, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(adv_value, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(need_stop_check, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stop_check_instart, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(cut_through, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(mem_flush, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(data_ids_1, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(data_ids_2, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(data_ids_vc1, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(data_ids_vc2, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi_16bit, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi_force, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi_limit, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi_overst, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(pkt2pkt_time, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(snrclk_en, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(snrclk_freq, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(vcext_en, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(error_diag, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi1_dt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi2_dt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi3_dt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(ipi4_dt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(cfg_nocheck, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(drop_func, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(drop_mask, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(irq_cnt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(irq_debug, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_dbg, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_mask, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_pile, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_ovif, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_stif, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(fatal_ap, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
/* PRQA S ALL -- */

static struct attribute *param_attr[] = {
	&dev_attr_nocheck.attr,
	&dev_attr_notimeout.attr,
	&dev_attr_wait_ms.attr,
	&dev_attr_dbg_value.attr,
	&dev_attr_adv_value.attr,
	&dev_attr_need_stop_check.attr,
	&dev_attr_stop_check_instart.attr,
	&dev_attr_cut_through.attr,
	&dev_attr_mem_flush.attr,
	&dev_attr_data_ids_1.attr,
	&dev_attr_data_ids_2.attr,
	&dev_attr_data_ids_vc1.attr,
	&dev_attr_data_ids_vc2.attr,
	&dev_attr_ipi_16bit.attr,
	&dev_attr_ipi_force.attr,
	&dev_attr_ipi_limit.attr,
	&dev_attr_ipi_overst.attr,
	&dev_attr_pkt2pkt_time.attr,
	&dev_attr_snrclk_en.attr,
	&dev_attr_snrclk_freq.attr,
	&dev_attr_vcext_en.attr,
	&dev_attr_error_diag.attr,
	&dev_attr_ipi1_dt.attr,
	&dev_attr_ipi2_dt.attr,
	&dev_attr_ipi3_dt.attr,
	&dev_attr_ipi4_dt.attr,
	&dev_attr_cfg_nocheck.attr,
	&dev_attr_drop_func.attr,
	&dev_attr_drop_mask.attr,
	&dev_attr_irq_cnt.attr,
	&dev_attr_irq_debug.attr,
	&dev_attr_stl_dbg.attr,
	&dev_attr_stl_mask.attr,
	&dev_attr_stl_pile.attr,
	&dev_attr_stl_ovif.attr,
	&dev_attr_stl_stif.attr,
	&dev_attr_fatal_ap.attr,
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs show for mipi dev devices' status/clock */
static ssize_t mipi_host_status_clock_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_CLOCK, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/info */
static ssize_t mipi_host_status_info_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_INFO, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/cfg */
static ssize_t mipi_host_status_cfg_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_CFG, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/regs */
static ssize_t mipi_host_status_regs_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_REGS, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/snrclk */
static ssize_t mipi_host_status_snrclk_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_SNRCLK, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/user */
static ssize_t mipi_host_status_user_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_USER, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs show for mipi host devices' status/icnt */
static ssize_t mipi_host_status_icnt_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_STATUS_ICNT, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs attr for mipi host devices' status */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(clock, S_IRUGO, mipi_host_status_clock_show, NULL);
static DEVICE_ATTR(info, S_IRUGO, mipi_host_status_info_show, NULL);
static DEVICE_ATTR(cfg, S_IRUGO, mipi_host_status_cfg_show, NULL);
static DEVICE_ATTR(regs, S_IRUGO, mipi_host_status_regs_show, NULL);
static DEVICE_ATTR(snrclk, S_IRUGO, mipi_host_status_snrclk_show, NULL);
static DEVICE_ATTR(user, S_IRUGO, mipi_host_status_user_show, NULL);
static DEVICE_ATTR(icnt, S_IRUGO, mipi_host_status_icnt_show, NULL);
/* PRQA S ALL -- */

static struct attribute *status_attr[] = {
	&dev_attr_clock.attr,
	&dev_attr_info.attr,
	&dev_attr_cfg.attr,
	&dev_attr_regs.attr,
	&dev_attr_snrclk.attr,
	&dev_attr_user.attr,
	&dev_attr_icnt.attr,
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

/* sysfs show for mipi host devices' fatal/ */
static ssize_t mipi_host_fatal_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3673 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_FATAL, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs store for mipi host devices' fatal/ */
static ssize_t mipi_host_fatal_store(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, const char *buf, size_t count) /* PRQA S 3673 */ /* linux cb func */
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_FATAL, MIPI_SYS_STORE,
			attr->attr.name, (char *)buf, count);
	return ret;
}

/* sysfs for mipi host devices' fatal */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(st_main, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(phy_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(pkt_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(frm_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(bndry_frm_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(seq_frm_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(crc_frm_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(pld_crc_fatal, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(data_id, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ecc_corrected, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(phy, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(pkt, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(line, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ipi, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ipi2, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ipi3, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ipi4, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ap_generic, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ap_ipi, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ap_ipi2, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ap_ipi3, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
static DEVICE_ATTR(ap_ipi4, (S_IWUSR | S_IRUGO), mipi_host_fatal_show, mipi_host_fatal_store);
/* PRQA S ALL -- */

static struct attribute *fatal_attr[] = {
	&dev_attr_st_main.attr,
	&dev_attr_phy_fatal.attr,
	&dev_attr_pkt_fatal.attr,
	&dev_attr_frm_fatal.attr,
	&dev_attr_bndry_frm_fatal.attr,
	&dev_attr_seq_frm_fatal.attr,
	&dev_attr_crc_frm_fatal.attr,
	&dev_attr_pld_crc_fatal.attr,
	&dev_attr_data_id.attr,
	&dev_attr_ecc_corrected.attr,
	&dev_attr_phy.attr,
	&dev_attr_pkt.attr,
	&dev_attr_line.attr,
	&dev_attr_ipi.attr,
	&dev_attr_ipi2.attr,
	&dev_attr_ipi3.attr,
	&dev_attr_ipi4.attr,
	&dev_attr_ap_generic.attr,
	&dev_attr_ap_ipi.attr,
	&dev_attr_ap_ipi2.attr,
	&dev_attr_ap_ipi3.attr,
	&dev_attr_ap_ipi4.attr,
	NULL,
};

static const struct attribute_group fatal_attr_group = {
	.name = __stringify(fatal),
	.attrs = fatal_attr,
};

#ifndef CONFIG_FAULT_INJECTION_ATTR
/* sysfs show for mipi host devices' fault_injection */
static ssize_t mipi_host_fault_injection_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_FAULT_INJECT, MIPI_SYS_SHOW,
			attr->attr.name, buf, PAGE_SIZE);
	return ret;
}

/* sysfs store for mipi host devices' fault_injection */
static ssize_t mipi_host_fault_injection_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = -EFAULT;
	struct mipi_dhdev_s *dhdev = (struct mipi_dhdev_s *)dev_get_drvdata(dev);

	if ((dhdev->ops != NULL) && (dhdev->ops->sys != NULL))
		ret = dhdev->ops->sys(dhdev->port, MIPI_HOST_SYS_FAULT_INJECT, MIPI_SYS_STORE,
			attr->attr.name, (char *)buf, count);
	return ret;
}

/* sysfs for mipi host devices' fault_injection */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(fault_injection, (S_IWUSR | S_IRUGO), mipi_host_fault_injection_show, mipi_host_fault_injection_store);
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
	&fatal_attr_group,
#ifndef CONFIG_FAULT_INJECTION_ATTR
	&fault_injection_attr_group,
#endif
	NULL,
};

static int32_t hobot_mipi_host_probe_cdev(struct mipi_dhdev_s *dhdev)
{
	int32_t ret;
	dev_t devno;
	struct os_dev *osdev = &dhdev->osdev;
	struct cdev *p_cdev;
	struct class *p_class = NULL;


	if ((dhdev->dops != NULL) && (dhdev->dops->class != NULL))
		p_class = (struct class *)dhdev->dops->class(1);
	if(p_class == NULL) {
		mipi_info(NULL, "[%s] no class support\n", __func__);
		return 0;
	}

	devno = MKDEV(g_mh_major, (uint32_t)(dhdev->port));
	osdev->devno = devno;
	p_cdev = &osdev->cdev;
	cdev_init(p_cdev, &hobot_mipi_host_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret != 0) {
		mipi_err(NULL, "[%s] cdev add error %d\n", __func__, ret);
		dhdev->dops->class(0);
		return ret;
	}
	osdev->dev = device_create_with_groups(p_class, NULL, devno,
			(void *)dhdev, attr_groups,
			"%s%d", MIPI_HOST_DNAME, dhdev->port);
	if (IS_ERR((void *)osdev->dev)) {
		ret = (int32_t)PTR_ERR((void *)osdev->dev);
		osdev->dev = NULL;
		mipi_err(NULL, "[%s] deivce create error %d\n", __func__, ret);
		cdev_del(&osdev->cdev);
		dhdev->dops->class(0);
		return ret;
	}
	dhdev->p_class = p_class;

	return 0;
}

static void hobot_mipi_host_remove_cdev(struct mipi_dhdev_s *dhdev)
{
	struct os_dev *osdev = &dhdev->osdev;

	if (dhdev->p_class != NULL) {
		device_destroy(dhdev->p_class, osdev->devno);
		cdev_del(&osdev->cdev);
	}

	osdev->dev = NULL;
}

int32_t hobot_mipi_host_debug_remove(uint32_t mask)
{
	struct mipi_dhdev_s *dhdev;
	uint32_t i;
	int32_t n = (mask == 0U) ? g_dhdev_mask : mask;

	if (g_dhdevs != NULL) {
		for (i = 0; i < MIPI_CSI_RXNUM; i++) {
			if ((n & (0x1U << i)) == 0U)
				continue;
			dhdev = &g_dhdevs[i];

			mipi_info(&dhdev->osdev, "debug remove\n");
			hobot_mipi_host_remove_cdev(dhdev);
			if ((dhdev->ops != NULL) && (dhdev->ops->osdev != NULL)) {
				struct os_dev *osdev = dhdev->ops->osdev(i);
				if ((osdev != NULL) && (osdev->dev != NULL))  {
					osdev->dev = NULL;
				}
			}
			if ((dhdev->dops != NULL) && (dhdev->dops->class != NULL))
				dhdev->dops->class(0);
		}
		g_dhdev_mask &= ~n;
		if (g_dhdev_mask == 0U) {
			osal_kfree(g_dhdevs);
			g_dhdevs = NULL;

			if (g_mh_major != 0U) {
				unregister_chrdev_region((dev_t)(MKDEV(g_mh_major, 0U)), MIPI_CSI_RXNUM);
				g_mh_major = 0U;
			}
		}
	}

	return 0;
}

int32_t hobot_mipi_host_debug_probe(uint32_t mask, const struct mipi_sub_ops_s *ops,
		const struct mipi_dbg_ops_s *dops)
{
	struct mipi_dhdev_s *dhdev;
	dev_t devno;
	int32_t ret, i, size = 0;
	uint32_t done = 0U;

	if (mask == 0)
		return -ENODEV;
	if (g_dhdev_mask != 0U)
		return -EBUSY;

	for (i = 0; i < MIPI_CSI_RXNUM; i++) {
		if ((mask & (0x1U << i)) != 0U)
			size = i +1;
	}
	g_dhdevs = (struct mipi_dhdev_s *)osal_kmalloc(sizeof(struct mipi_dhdev_s) * size, GFP_KERNEL);
	if (IS_ERR(g_dhdevs)) {
		mipi_err(NULL, "[%s] malloc failed\n", __func__);
		g_dhdevs = NULL;
		return -ENOMEM;
	}
	(void)memset((void *)g_dhdevs, 0, sizeof(struct mipi_dhdev_s) * size);

	if (g_mh_major == 0U) {
		ret = alloc_chrdev_region(&devno, 0, MIPI_CSI_RXNUM, MIPI_HOST_DNAME);
		if (ret < 0) {
			mipi_err(NULL, "[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_HOST_DNAME, ret);
			return ret;
		}
		g_mh_major = MAJOR(devno);
	}

	for (i = 0; i < MIPI_CSI_RXNUM; i++) {
		if ((mask & (0x1U << i)) == 0U)
			continue;
		/* prepare: dhdev */
		dhdev = &g_dhdevs[i];
		/* prepare: base */
		dhdev->port = i;
		dhdev->ops = ops;
		dhdev->dops = dops;
		/* prepare: cdev */
		ret = hobot_mipi_host_probe_cdev(dhdev);
		if (ret != 0) {
			mipi_err(NULL, "[%s] hobot_mipi_host_probe_cdev error %d\n", __func__, ret);
			unregister_chrdev_region((dev_t)(MKDEV(g_mh_major, 0U)), MIPI_CSI_RXNUM);
			g_mh_major = 0U;
			hobot_mipi_host_debug_remove(done);
			return ret;
		}
		if ((dhdev->ops != NULL) && (dhdev->ops->osdev != NULL)) {
			struct os_dev *osdev = dhdev->ops->osdev(i);
			if ((osdev != NULL) && (osdev->dev == NULL))  {
				osdev->dev = dhdev->osdev.dev;
			}
		}
		done |= (0x1U << i);
		mipi_info(&dhdev->osdev, "debug probed\n");
	}
	g_dhdev_mask = mask;

	return 0;
}
