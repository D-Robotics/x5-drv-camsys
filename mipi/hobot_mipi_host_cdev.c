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


#include "hobot_mipi_host_ops.h"
#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_phy.h"

/* module params of port num */
static uint32_t port_num;
module_param(port_num, uint, 0444); /* PRQA S 0605,0636,4501 */ /* module_param macro */

#ifdef EX_MODULE
/* module params of reg area for ko */
static uint32_t reg_addr = MIPI_HOST_REG_ADDR;
static uint32_t reg_size = MIPI_HOST_REG_SIZE;
static uint32_t init_num = MIPI_HOST_MAX_NUM;
static uint32_t hw_mode = 1;
static uint32_t ap_mode = 1;
module_param(reg_addr, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(reg_size, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(init_num, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(hw_mode, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
module_param(ap_mode, uint, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */
#endif

/* global var */
/* the major id of device, valid if not 0.
 * see: hobot_mipi_host_probe_cdev, hobot_mipi_host_remove_cdev
 */
static uint32_t g_mh_major;
/* the class pointer of device, valid if not NULL.
 * see: hobot_mipi_host_class_get, hobot_mipi_host_class_put
 */
static struct class *g_mh_class;

#ifdef CONFIG_HOBOT_FUSA_DIAG
EXPORT_SYMBOL_GPL(hobot_mipi_host_stl_setup_do); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */
#endif

/**
 * brief mipi_host_irq_func : irq func
 *
 * param [in] this_irq : irq num
 * param [in] data : user data
 *
 * return irqreturn_t
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

static int32_t hobot_mipi_host_open(struct inode *inode, struct file *file) /* PRQA S 3673 */ /* linux cb func */
{
	struct os_dev *dev = (struct os_dev*)container_of(inode->i_cdev, struct os_dev, cdev); /* PRQA S ALL */ /* linux macro */
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)container_of(dev, struct mipi_hdev_s, osdev); /* PRQA S ALL */ /* linux macro */
	int ret;

	ret = hobot_mipi_host_open_do(hdev);
	if (ret < 0) {
		return ret;
	}

	file->private_data = (void *)hdev;
	return 0;
}

static int32_t hobot_mipi_host_close(struct inode *inode, struct file *file) /* PRQA S 3206,3673 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)file->private_data;

	return hobot_mipi_host_close_do(hdev);
}

static mipi_ioc_ret_t hobot_mipi_host_ioctl(struct file *file, uint32_t cmd, mipi_ioc_arg_t arg) /* PRQA S 3673 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev;

	/* Check type and command number */
	if ((file == NULL) || (file->private_data == NULL)) {
		/* do not need report */
		return -ENODEV;
	}
	hdev = (struct mipi_hdev_s *)file->private_data;

	return hobot_mipi_host_ioctl_do(hdev, cmd, arg);
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
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_PARAM, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs store for mipi host devices' param/ */
static ssize_t mipi_host_param_store(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, const char *buf, size_t count) /* PRQA S 3673 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_PARAM, MIPI_SYS_STORE,
			hdev, attr->attr.name, (char *)buf, count);
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
#if MIPI_HOST_INT_DBG
static DEVICE_ATTR(irq_cnt, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(irq_debug, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
#endif
#ifdef CONFIG_HOBOT_FUSA_DIAG
static DEVICE_ATTR(stl_dbg, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_mask, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_pile, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_ovif, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
static DEVICE_ATTR(stl_stif, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
#if MIPI_HOST_SYSFS_FATAL_EN
static DEVICE_ATTR(fatal_ap, (S_IWUSR | S_IRUGO), mipi_host_param_show, mipi_host_param_store);
#endif
#endif
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
#if MIPI_HOST_INT_DBG
	&dev_attr_irq_cnt.attr,
	&dev_attr_irq_debug.attr,
#endif
#ifdef CONFIG_HOBOT_FUSA_DIAG
	&dev_attr_stl_dbg.attr,
	&dev_attr_stl_mask.attr,
	&dev_attr_stl_pile.attr,
	&dev_attr_stl_ovif.attr,
	&dev_attr_stl_stif.attr,
#if MIPI_HOST_SYSFS_FATAL_EN
	&dev_attr_fatal_ap.attr,
#endif
#endif
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
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_CLOCK, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs show for mipi host devices' status/info */
static ssize_t mipi_host_status_info_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_INFO, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs show for mipi host devices' status/cfg */
static ssize_t mipi_host_status_cfg_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_CFG, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs show for mipi host devices' status/regs */
static ssize_t mipi_host_status_regs_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_REGS, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs show for mipi host devices' status/snrclk */
static ssize_t mipi_host_status_snrclk_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_SNRCLK, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs show for mipi host devices' status/user */
static ssize_t mipi_host_status_user_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_USER, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

#if MIPI_HOST_INT_DBG
/* sysfs show for mipi host devices' status/icnt */
static ssize_t mipi_host_status_icnt_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3206 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_STATUS_ICNT, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}
#endif

/* sysfs attr for mipi host devices' status */
/* PRQA S ALL ++ */ /* linux macro */
static DEVICE_ATTR(clock, S_IRUGO, mipi_host_status_clock_show, NULL);
static DEVICE_ATTR(info, S_IRUGO, mipi_host_status_info_show, NULL);
static DEVICE_ATTR(cfg, S_IRUGO, mipi_host_status_cfg_show, NULL);
static DEVICE_ATTR(regs, S_IRUGO, mipi_host_status_regs_show, NULL);
static DEVICE_ATTR(snrclk, S_IRUGO, mipi_host_status_snrclk_show, NULL);
static DEVICE_ATTR(user, S_IRUGO, mipi_host_status_user_show, NULL);
#if MIPI_HOST_INT_DBG
static DEVICE_ATTR(icnt, S_IRUGO, mipi_host_status_icnt_show, NULL);
#endif
/* PRQA S ALL -- */

static struct attribute *status_attr[] = {
	&dev_attr_clock.attr,
	&dev_attr_info.attr,
	&dev_attr_cfg.attr,
	&dev_attr_regs.attr,
	&dev_attr_snrclk.attr,
	&dev_attr_user.attr,
#if MIPI_HOST_INT_DBG
	&dev_attr_icnt.attr,
#endif
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

#if MIPI_HOST_INT_DBG && MIPI_HOST_SYSFS_FATAL_EN
/* sysfs show for mipi host devices' fatal/ */
static ssize_t mipi_host_fatal_show(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, char *buf) /* PRQA S 3673 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_FATAL, MIPI_SYS_SHOW,
			hdev, attr->attr.name, buf, PAGE_SIZE);
}

/* sysfs store for mipi host devices' fatal/ */
static ssize_t mipi_host_fatal_store(struct device *dev, /* PRQA S 3673 */ /* linux cb func */
		struct device_attribute *attr, const char *buf, size_t count) /* PRQA S 3673 */ /* linux cb func */
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_FATAL, MIPI_SYS_STORE,
			hdev, attr->attr.name, (char *)buf, count);
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
#endif

#if defined CONFIG_FAULT_INJECTION_ATTR || defined CONFIG_HOBOT_FUSA_DIAG
/* sysfs show for mipi host driver' fault_injection */
static int32_t hobot_mipi_host_fault_injection_show(struct device *dev,
		char *buf, size_t count)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_FAULT_INJECT, MIPI_SYS_SHOW,
			hdev, "fault_injection", buf, count);
}

/* sysfs store for mipi host driver' fault_injection */
static int32_t hobot_mipi_host_fault_injection_store(struct device *dev,
		const char *buf, size_t count)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	return hobot_mipi_host_sys_do(MIPI_HOST_SYS_FAULT_INJECT, MIPI_SYS_STORE,
			hdev, "fault_injection", (char *)buf, count);
}
#endif

#if (!defined CONFIG_FAULT_INJECTION_ATTR) && (defined CONFIG_HOBOT_FUSA_DIAG)
/* sysfs show for mipi host devices' fault_injection */
static ssize_t mipi_host_fault_injection_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return hobot_mipi_host_fault_injection_show(dev, buf, PAGE_SIZE);
}

/* sysfs store for mipi host devices' fault_injection */
static ssize_t mipi_host_fault_injection_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return hobot_mipi_host_fault_injection_store(dev, buf, count);
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
#if MIPI_HOST_INT_DBG && MIPI_HOST_SYSFS_FATAL_EN
	&fatal_attr_group,
#endif
#if (!defined CONFIG_FAULT_INJECTION_ATTR) && (defined CONFIG_HOBOT_FUSA_DIAG)
	&fault_injection_attr_group,
#endif
	NULL,
};

static int32_t hobot_mipi_host_class_get(void)
{
	int32_t ret = 0;

	if (g_mh_class == NULL) {
#ifdef CONFIG_HOBOT_MIPI_PHY
		g_mh_class = mipi_phy_class();
		if (g_mh_class == NULL) {
			mipi_err(NULL, "[%s] dphy class null\n", __func__);
			ret = -ENODEV;
		}
#else
		g_mh_class = class_create(THIS_MODULE, MIPI_HOST_DNAME);
		if (IS_ERR((void *)g_mh_class)) {
			ret = (int32_t)PTR_ERR((void *)g_mh_class);
			g_mh_class = NULL;
			mipi_err(NULL, "[%s] class error %d\n", __func__,
					ret);
		}
#endif
	}
	return ret;
}

static void hobot_mipi_host_class_put(void)
{
#ifndef CONFIG_HOBOT_MIPI_PHY
	if (g_mh_class != NULL) {
		class_destroy(g_mh_class);
	}
#endif
	g_mh_class = NULL;
}

static int32_t hobot_mipi_host_probe_cdev(struct mipi_hdev_s *hdev)
{
	int32_t ret;
	dev_t devno;
	struct os_dev *osdev = &hdev->osdev;
	struct cdev *p_cdev;

	if (g_mh_major == 0U) {
		ret = alloc_chrdev_region(&devno, 0, MIPI_HOST_MAX_NUM, MIPI_HOST_DNAME);
		if (ret < 0) {
			mipi_err(NULL, "[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_HOST_DNAME, ret);
			return ret;
		}
		g_mh_major = MAJOR(devno);
	}
	ret = hobot_mipi_host_class_get();
	if (ret != 0) {
		return ret;
	}
	devno = MKDEV(g_mh_major, (uint32_t)(hdev->port));
	osdev->devno = devno;
	p_cdev = &osdev->cdev;
	cdev_init(p_cdev, &hobot_mipi_host_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret != 0) {
		mipi_err(NULL, "[%s] cdev add error %d\n", __func__, ret);
		return ret;
	}
	osdev->dev = device_create_with_groups(g_mh_class, NULL, devno,
			(void *)hdev, attr_groups,
			"%s%d", MIPI_HOST_DNAME, hdev->port);
	if (IS_ERR((void *)osdev->dev)) {
		ret = (int32_t)PTR_ERR((void *)osdev->dev);
		osdev->dev = NULL;
		mipi_err(NULL, "[%s] deivce create error %d\n", __func__, ret);
		cdev_del(&osdev->cdev);
		return ret;
	}

	return 0;
}

static void hobot_mipi_host_remove_cdev(struct mipi_hdev_s *hdev)
{
	struct os_dev *osdev = &hdev->osdev;

	mipi_info(osdev, "remove\n");
	if (g_mh_class != NULL) {
		device_destroy(g_mh_class, osdev->devno);
	}
	cdev_del(&osdev->cdev);

	if (port_num <= 1U) {
		hobot_mipi_host_class_put();
		if (g_mh_major != 0U) {
			unregister_chrdev_region((dev_t)(MKDEV(g_mh_major, 0U)),
				MIPI_HOST_MAX_NUM);
			g_mh_major = 0U;
		}
	}
	osdev->dev = NULL;
}

#ifdef EX_MODULE
static int32_t hobot_mipi_host_remove_param(void)
{
	struct mipi_hdev_s *hdev;
	struct mipi_host_s *host;
	uint32_t i;

	for (i = 0U; i < MIPI_HOST_MAX_NUM; i ++) {
		hdev = hobot_mipi_host_hdev(i);
		if (hdev == NULL) {
			continue;
		}
		host = &hdev->host;

#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		(void)osal_timer_stop(&hdev->irq_timer);
#endif
		hobot_mipi_host_remove_cdev(i, &hdev->osdev);
		if (host->iomem != NULL) {
			iounmap(host->iomem);
		}
		hobot_mipi_host_remove_do(hdev);
		kfree(hdev);
		port_num --;
	}
	return 0;
}

static int32_t hobot_mipi_host_probe_param(void)
{
	void __iomem *iomem;
	struct mipi_hdev_s *hdev;
	struct mipi_host_s *host;
	uint64_t reg_addr_dev;
	int32_t i, ret = 0;

	if (init_num <= 0) {
		/* do not need report */
		return -ENODEV;
	}

	if (init_num > MIPI_HOST_MAX_NUM) {
		init_num = MIPI_HOST_MAX_NUM;
	}

	port_num = 0;
	for (i = 0; i < init_num; i++) {
		/* prepare: hdev */
		hdev = (struct mipi_hdev_s *)kmalloc(sizeof(struct mipi_hdev_s), GFP_KERNEL);
		if (hdev == NULL) {
			mipi_err(NULL, "[%s] malloc failed\n", __func__);
			ret = -ENOMEM;
			hobot_mipi_host_remove_param();
			/* do not need report */
			return ret;
		}
		(void)memset((void *)hdev, 0, sizeof(struct mipi_hdev_s));
		/* prepare: base */
		hdev->port = i;
		hdev->hw_mode = (void *)hw_mode;
		host=&hdev->host;
		host->ap = ap_mode;
		/* prepare: reg */
		reg_addr_dev = reg_addr + (reg_size * i);
		host->iomem = ioremap(reg_addr_dev, reg_size);
		if (IS_ERR(host->iomem)) {
			mipi_err(NULL, "[%s] ioremap error\n", __func__);
			ret = (int32_t)PTR_ERR(host->iomem);
			host->iomem = NULL;
			kfree(hdev);
			hobot_mipi_host_remove_param();
			return ret;
		}
		host.reg.base = reg_addr_dev;
		host.reg.size = reg_size;
		/* prepare: cdev */
		ret = hobot_mipi_host_probe_cdev(hdev);
		if (ret != 0) {
			mipi_err(NULL, "[%s] hobot_mipi_host_probe_cdev error %d\n", __func__, ret);
			iounmap(host->iomem);
			iomem = NULL;
			kfree(hdev);
			hobot_mipi_host_remove_param();
			return ret;
		}
		/* prepare: irq */
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		osal_timer_init(&hdev->irq_timer, mipi_host_irq_timer_func, hdev);
#else
		mipi_info(NULL, "[%s] no int32_t timer\n", __func__);
#endif
		/* prepare: snrclk */
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
		mipi_debug(NULL, "[%s] no snrclk support\n", __func__);
#endif
		/* prepare: probe */
		ret = hobot_mipi_host_probe_do(hdev);
		if (ret < 0) {
			mipi_err(NULL, "[%s] hobot_mipi_host_probe_cdev error %d\n", __func__, ret);
			hobot_mipi_host_remove_cdev(i, &hdev->osdev);
			iounmap(host->iomem);
			host->iomem = NULL;
			kfree(hdev);
			hobot_mipi_host_remove_param();
			return ret;
		}
		port_num ++;
	}

	return 0;
}

#else
static int32_t hobot_mipi_host_remove(struct platform_device *pdev)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)platform_get_drvdata(pdev);
	struct mipi_host_s *host = &hdev->host;

	hobot_mipi_host_remove_cdev(hdev);
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
	port_num --;
	return 0;
}

static int32_t hobot_mipi_host_probe_alloc_hdev(struct platform_device *pdev, struct mipi_hdev_s **phdev)
{
	struct mipi_hdev_s *hdev;
	int32_t port;

	port = of_alias_get_id(pdev->dev.of_node, "mipihost");
	if (port >= MIPI_HOST_MAX_NUM) {
		mipi_err(NULL, "[%s] port %d >= %d overflow error\n", __func__, port,
				MIPI_HOST_MAX_NUM);
		return -ERANGE;
	}
	if (port < 0) {
		port = 0;
	}
	hdev = (struct mipi_hdev_s *)devm_kmalloc(&pdev->dev, sizeof(struct mipi_hdev_s), GFP_KERNEL);
	if (hdev == NULL) {
		mipi_err(NULL, "[%s] devm_kmalloc failed\n", __func__);
		return -ENOMEM;
	}
	(void)memset((void *)hdev, 0, sizeof(struct mipi_hdev_s));
	/* prepare: report */
	hdev->port = port;
	*phdev = hdev;
	return 0;
}

static int32_t hobot_mipi_host_probe_iomem_init(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct resource *res;
	int32_t ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->iomem)) {
		mipi_err(NULL, "[%s] get mem res error\n", __func__);
		ret = (int32_t)PTR_ERR(host->iomem);
		host->iomem = NULL;
	}
	host->reg.base = (uint32_t)res->start;
	host->reg.size = (uint32_t)(res->end - res->start + 1U);

	return ret;
}

static int32_t hobot_mipi_host_probe_irq_init(struct platform_device *pdev, struct mipi_hdev_s *hdev)
{
#ifdef MIPI_HOST_INT_USE_TIMER
	osal_timer_init(&hdev->irq_timer, mipi_host_irq_timer_func, hdev);

	return 0;
#else
	struct mipi_host_s *host = &hdev->host;
	int32_t ret;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		mipi_err(NULL, "[%s] get irq res error\n", __func__);
		return -ENODEV;
	}
	host->irq = (int32_t)res->start;
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
		mipi_err(NULL, "[%s] request irq error %d\n", __func__, ret);
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

static struct platform_driver hobot_mipi_host_driver;
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
	/* prepare: cdev */
	ret = hobot_mipi_host_probe_cdev(hdev);
	if (ret != 0) {
		hobot_mipi_host_probe_free(pdev, hdev);
		return ret;
	}
	hdev->osdev.dev->driver = &hobot_mipi_host_driver.driver;
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
	port_num ++;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int32_t hobot_mipi_host_suspend(struct device *dev)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)dev_get_drvdata(dev);

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE) {
		return 0;
	}

	return hobot_mipi_host_suspend_do(hdev);
}

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
#ifdef CONFIG_FAULT_INJECTION_ATTR
		.fault_injection_store = hobot_mipi_host_fault_injection_store,
		.fault_injection_show = hobot_mipi_host_fault_injection_show,
#endif
	},
};
#endif

static int32_t __init hobot_mipi_host_module_init(void)
{
	int32_t ret;

#ifdef EX_MODULE
	ret = hobot_mipi_host_probe_param();
#else
	ret = platform_driver_register(&hobot_mipi_host_driver);
#endif

	return ret;
}

static void __exit hobot_mipi_host_module_exit(void)
{
#ifdef EX_MODULE
	hobot_mipi_host_remove_param();
#else
	platform_driver_unregister(&hobot_mipi_host_driver);
#endif
}

late_initcall_sync(hobot_mipi_host_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_mipi_host_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT MIPI Host Driver"); /* PRQA S ALL */ /* linux macro */
