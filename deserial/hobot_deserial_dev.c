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
 * @file hobot_deserial_dev.c
 *
 * @NO{S10E02C11}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>

#include "hobot_deserial_ops.h"

/**
 * @var deserial_num
 * deserial device number
 */
static uint32_t deserial_num = DESERIAL_NUM_DEFAULT;
module_param(deserial_num, uint, 0644);

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief get deserial device struct by minor index
 *
 * @param[in] minor: minor index of misc device
 *
 * @return !NULL:Success as deserial device struct, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct deserial_device_s* deserial_dev_by_minor(int32_t minor)
{
	int32_t i;
	struct deserial_device_s *des;
	struct deserial_s *gdes = deserial_global();

	if (gdes == NULL)
		return NULL;

	for (i = 0; i < gdes->des_num; i++) {
		des = &gdes->des[i];
		if (des->osdev.minor_id == minor)
			return des;
	}

	return NULL;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief get deserial device struct by os device pointer
 *
 * @param[in] dev: the misc device pointer to match
 *
 * @return !NULL:Success as deserial device struct, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct deserial_device_s* deserial_dev_by_device(struct device *dev)
{
	int32_t i;
	struct deserial_device_s *des;
	struct deserial_s *gdes = deserial_global();

	if (gdes == NULL)
		return NULL;

	for (i = 0; i < gdes->des_num; i++) {
		des = &gdes->des[i];
		if (des->osdev.miscdev.this_device == dev)
			return des;
	}

	return NULL;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device file operation: open
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
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
static int32_t deserial_fop_open(struct inode *pinode, struct file *pfile)
{
	int32_t ret;
	int32_t minor = iminor(pinode);
	struct deserial_device_s *des = deserial_dev_by_minor(minor);

	ret = deserial_device_open(des);
	if (ret < 0)
		return ret;

	pfile->private_data = (void *)des;
	return ret;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device file operation: release
 *
 * @param[in] pinode: file node point
 * @param[in] pfile: file point
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
static int32_t deserial_fop_release(struct inode *pinode, struct file *pfile)
{
	int32_t ret, i, cur_pid;
	int32_t minor = iminor(pinode);
	struct deserial_device_s *des = deserial_dev_by_minor(minor);
	struct os_dev *dev = &des->osdev;
	cur_pid = current->pid;

	for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		if (des->link[i].init_by_pid == cur_pid) {
			des->link[i].init_by_pid = 0;
			des_info(dev, "pid%d realse link-%d\n", cur_pid, i);
		}
	}

	ret = deserial_device_close(des);
	pfile->private_data = NULL;

	return ret;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial device file operation: ioctl
 *
 * @param[in] pfile: file point
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
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
static long deserial_fop_ioctl(struct file *pfile, uint32_t cmd, unsigned long arg)
{
	int32_t ret;
	struct deserial_device_s *des = (struct deserial_device_s *)pfile->private_data;

	ret = deserial_device_ioctl(des, cmd, arg);
	return ret;
}

/**
 * deserial driver ops for dev file
 */
static const struct file_operations deserial_fops = {
	.owner = THIS_MODULE,
	.open = deserial_fop_open,
	.release = deserial_fop_release,
	.unlocked_ioctl = deserial_fop_ioctl,
	.compat_ioctl = deserial_fop_ioctl,
};

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial status info show of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_status_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_status_info_show(des, buf);
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial status cfg show of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_status_cfg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_status_cfg_show(des, buf);
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial status regs show of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_status_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_status_regs_show(des, buf);
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial status user show of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_status_user_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_status_user_show(des, buf);
}

/* sysfs attr for deserial devices' status */
static DEVICE_ATTR(info, S_IRUGO, deserial_status_info_show, NULL);
static DEVICE_ATTR(cfg, S_IRUGO, deserial_status_cfg_show, NULL);
static DEVICE_ATTR(regs, S_IRUGO, deserial_status_regs_show, NULL);
static DEVICE_ATTR(user, S_IRUGO, deserial_status_user_show, NULL);

static struct attribute *status_attr[] = {
	&dev_attr_info.attr,
	&dev_attr_cfg.attr,
	&dev_attr_regs.attr,
	&dev_attr_user.attr,
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial param show of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[out] buf: the buffer to show string store
 *
 * @return >=0:Success-string length, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_param_show(des, attr->attr.name, buf);
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief camera deserial param store of sysfs
 *
 * @param[in] dev: the deserial device struct
 * @param[in] attr: the sysfs attr struct
 * @param[in] buf: the buffer to store string
 * @param[in] count: the buffer string length
 *
 * @return >0:Success-the count, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t deserial_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct deserial_device_s *des = deserial_dev_by_device(dev);
	return deserial_device_param_store(des, attr->attr.name, buf, count);
}

/* sysfs for deserial devices' param */
static DEVICE_ATTR(op_timeout_ms, (S_IWUSR | S_IRUGO), deserial_param_show, deserial_param_store);
static DEVICE_ATTR(op_retry_max, (S_IWUSR | S_IRUGO), deserial_param_show, deserial_param_store);

static struct attribute *param_attr[] = {
	&dev_attr_op_timeout_ms.attr,
	&dev_attr_op_retry_max.attr,
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs attr groups */
static const struct attribute_group *attr_groups[] = {
	&param_attr_group,
	&status_attr_group,
	NULL,
};

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial misc char device init
 *
 * @param[in] des: deserial device struct
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
static int32_t deserial_miscdev_init(struct deserial_device_s *des)
{
	int32_t ret;
	struct deserial_miscdev_s *mdev = &des->mdev;
	struct os_dev *dev = &des->osdev;

	snprintf(mdev->name, DESERIAL_NAME_LEN_MAX, "%s%d", DESERIAL_DEV_NAME, des->index);
	dev->miscdev.name = mdev->name;
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.fops = &deserial_fops;
	dev->miscdev.groups = attr_groups;

	ret = misc_register(&dev->miscdev);
	if (ret < 0) {
		des_err(dev, "misc_register failed %d\n", ret);
		return ret;
	}

	dev->minor_id = dev->miscdev.minor;

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial misc char device exit
 *
 * @param[in] des: deserial device struct
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
static void deserial_miscdev_exit(struct deserial_device_s *des)
{
	struct os_dev *dev = &des->osdev;

	misc_deregister(&dev->miscdev);
	dev->minor_id = -1;

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device init to creat chardev
 *
 * @param[in] des: deserial device struct
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
static int32_t deserial_dev_init(struct deserial_device_s *des)
{
	int32_t ret;
	struct os_dev *dev = &des->osdev;

	dev->devno = des->index;

	ret = deserial_miscdev_init(des);
	if (ret < 0)
		return ret;

	ret = deserial_device_init(des);
	if (ret < 0) {
		deserial_miscdev_exit(des);
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief deserial device exit and destory chardev
 *
 * @param[in] des: deserial device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void deserial_dev_exit(struct deserial_device_s *des)
{
	deserial_device_exit(des);
	deserial_miscdev_exit(des);
	return;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial driver init
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_deserial_module_init(void)
{
	int32_t ret, i;
	struct deserial_s *gdes = deserial_global();
	struct deserial_device_s *des_a, *des;

	if (gdes == NULL)
		return -EFAULT;

	if ((deserial_num == 0) || (deserial_num > DESERIAL_NUM_MAX)) {
		des_info(NULL, "deserial_num %d error set default %d\n",
			deserial_num, DESERIAL_NUM_DEFAULT);
		deserial_num = DESERIAL_NUM_DEFAULT;
	}

	des_a = osal_kmalloc(sizeof(struct deserial_device_s) * deserial_num, GFP_KERNEL);
	if (des_a == NULL) {
		des_err(NULL, "alloc %d deserial device error\n", deserial_num);
		return -ENOMEM;
	}
	memset(des_a, 0, sizeof(struct deserial_device_s) * deserial_num);

	for (i = 0; i < deserial_num; i++) {
		des = &des_a[i];
		des->index = i;
		ret = deserial_dev_init(des);
		if (ret < 0) {
			des_err(NULL, "deserial_dev_init %d failed\n", i);
			while (i > 0) {
				i--;
				deserial_dev_exit(&des_a[i]);
			}
			osal_kfree(des_a);
			return ret;
		}
	}

	ret = deserial_driver_init();
	if (ret < 0) {
		des_err(NULL, "deserial cops register failed %d\n", ret);
		for (i = 0; i < deserial_num; i++)
			deserial_dev_exit(&des_a[i]);
		osal_kfree(des_a);
		return ret;
	}
	gdes->des = des_a;
	gdes->des_num = deserial_num;

	des_info(NULL, "init %d deserial v%u.%u done\n",
		gdes->des_num, gdes->ver.major, gdes->ver.minor);

	return ret;
}

/**
 * @NO{S10E02C11I}
 * @ASIL{B}
 * @brief deserial driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_deserial_module_exit(void)
{
	struct deserial_s *gdes = deserial_global();
	int32_t i;

	if (gdes == NULL)
		return;

	deserial_driver_exit();

	for (i = 0; i < gdes->des_num; i++) {
		deserial_dev_exit(&gdes->des[i]);
	}
	osal_kfree(gdes->des);
	gdes->des = NULL;

	des_info(NULL, "exit %d deserial v%u.%u done\n",
		gdes->des_num, gdes->ver.major, gdes->ver.minor);
	gdes->des_num = 0;

	return;
}

late_initcall_sync(hobot_deserial_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_deserial_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT VIN DESERIAL Driver"); /* PRQA S ALL */ /* linux macro */
