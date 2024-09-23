/*   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

/**
 * @file hobot_sensor_dev.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>

#include "hobot_sensor_ops.h"

/**
 * @var sensor_num
 * sensor device number
 */
static uint32_t sensor_num = SENSOR_NUM_DEFAULT;
module_param(sensor_num, uint, 0644);

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor device struct by minor index
 *
 * @param[in] minor: minor index of misc device
 *
 * @return !NULL:Success as sensor device struct, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct sensor_device_s* sensor_dev_by_minor(int32_t minor)
{
	int32_t i;
	struct sensor_device_s *sen;
	struct sensor_s *gsen = sensor_global();

	if (gsen == NULL)
		return NULL;

	for (i = 0; i < gsen->sen_num; i++) {
		sen = &gsen->sen[i];
		if (sen->osdev.minor_id == minor)
			return sen;
	}

	return NULL;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get sensor device struct by os device pointer
 *
 * @param[in] dev: the misc device pointer to match
 *
 * @return !NULL:Success as sensor device struct, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct sensor_device_s *sensor_dev_by_device(struct device *dev)
{
	int32_t i;
	struct sensor_device_s *sen;
	struct sensor_s *gsen = sensor_global();

	if (gsen == NULL)
		return NULL;

	for (i = 0; i < gsen->sen_num; i++) {
		sen = &gsen->sen[i];
		if (sen->osdev.miscdev.this_device == dev)
			return sen;
	}

	return NULL;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor device file operation: open
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
static int32_t sensor_fop_open(struct inode *pinode, struct file *pfile)
{
	int32_t ret;
	int32_t minor = iminor(pinode);
	struct sensor_device_s *sen = sensor_dev_by_minor(minor);

	if (sen == NULL)
		return -ENODEV;

	ret = sensor_device_open(sen);
	if (ret < 0)
		return ret;

	pfile->private_data = (void *)sen;
	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor device file operation: release
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
static int32_t sensor_fop_release(struct inode *pinode, struct file *pfile)
{
	int32_t ret;
	int32_t minor = iminor(pinode);
	struct sensor_device_s *sen = sensor_dev_by_minor(minor);

	ret = sensor_device_close(sen);
	pfile->private_data = NULL;

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief camera sensor file operation: ioctl
 *
 * @param[in] pfile: file point
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static long sensor_fop_ioctl(struct file *pfile, uint32_t cmd, unsigned long arg)
{
	int32_t ret;
	struct sensor_device_s *sen = (struct sensor_device_s *)pfile->private_data;

	ret = sensor_device_ioctl(sen, cmd, arg);
	return ret;
}

/**
 * camera sensor driver file operation functions
 */
static const struct file_operations sensor_fops = {
	.owner = THIS_MODULE,
	.open = sensor_fop_open,
	.release = sensor_fop_release,
	.unlocked_ioctl = sensor_fop_ioctl,
	.compat_ioctl = sensor_fop_ioctl,
};

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status info show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_info_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status iparam show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_iparam_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_iparam_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status cfg show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_cfg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_cfg_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status regs show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_regs_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status user show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_user_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_user_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status frame show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_frame_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_frame_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status fps show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_fps_show(sen, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor status fps_record show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_status_fps_record_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_status_fps_record_show(sen, buf);
}

/* sysfs attr for sensor devices' status */
static DEVICE_ATTR(info, S_IRUGO, sensor_status_info_show, NULL);
static DEVICE_ATTR(iparam, S_IRUGO, sensor_status_iparam_show, NULL);
static DEVICE_ATTR(cfg, S_IRUGO, sensor_status_cfg_show, NULL);
static DEVICE_ATTR(regs, S_IRUGO, sensor_status_regs_show, NULL);
static DEVICE_ATTR(user, S_IRUGO, sensor_status_user_show, NULL);
static DEVICE_ATTR(frame, S_IRUGO, sensor_status_frame_show, NULL);
static DEVICE_ATTR(fps, S_IRUGO, sensor_status_fps_show, NULL);
static DEVICE_ATTR(fps_record, S_IRUGO, sensor_status_fps_record_show, NULL);

static struct attribute *status_attr[] = {
	&dev_attr_info.attr,
	&dev_attr_iparam.attr,
	&dev_attr_cfg.attr,
	&dev_attr_regs.attr,
	&dev_attr_user.attr,
	&dev_attr_frame.attr,
	&dev_attr_fps.attr,
	&dev_attr_fps_record.attr,
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor param show of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_param_show(sen, attr->attr.name, buf);
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor param store of sysfs
 *
 * @param[in] dev: the sensor device struct
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
static ssize_t sensor_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_device_s *sen = sensor_dev_by_device(dev);
	return sensor_device_param_store(sen, attr->attr.name, buf, count);
}

/* sysfs for sensor devices' param */
static DEVICE_ATTR(i2c_debug, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(frame_debug, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(stop_wait_ms, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ctrl_mode, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ae_share_flag, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ae_event_flag, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ts_compensate, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(iparam_timeout_ms, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ctrl_timeout_ms, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ev_timeout_ms, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);
static DEVICE_ATTR(ev_retry_max, (S_IWUSR | S_IRUGO), sensor_param_show, sensor_param_store);

static struct attribute *param_attr[] = {
	&dev_attr_i2c_debug.attr,
	&dev_attr_frame_debug.attr,
	&dev_attr_stop_wait_ms.attr,
	&dev_attr_ctrl_mode.attr,
	&dev_attr_ae_share_flag.attr,
	&dev_attr_ae_event_flag.attr,
	&dev_attr_ts_compensate.attr,
	&dev_attr_iparam_timeout_ms.attr,
	&dev_attr_ctrl_timeout_ms.attr,
	&dev_attr_ev_timeout_ms.attr,
	&dev_attr_ev_retry_max.attr,
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
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor misc char device init
 *
 * @param[in] sen: sensor device struct
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
static int32_t sensor_miscdev_init(struct sensor_device_s *sen)
{
	int32_t ret;
	struct sensor_miscdev_s *mdev = &sen->mdev;
	struct os_dev *dev = &sen->osdev;

	snprintf(mdev->name, SENSOR_NAME_LEN_MAX, "%s%d", SENSOR_DEV_NAME, sen->port);
	dev->miscdev.name = mdev->name;
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.fops = &sensor_fops;
	dev->miscdev.groups = attr_groups;
	dev->miscdev.mode = 0666;

	ret = misc_register(&dev->miscdev);
	if (ret < 0) {
		sen_err(&dev, "misc_register failed %d\n", ret);
		return ret;
	}

	dev->minor_id = dev->miscdev.minor;

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor misc char device exit
 *
 * @param[in] sen: sensor device struct
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
static void sensor_miscdev_exit(struct sensor_device_s *sen)
{
	struct os_dev *dev = &sen->osdev;

	misc_deregister(&dev->miscdev);
	dev->minor_id = -1;

	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device init device
 *
 * @param[in] sen: camera sensor device struct with port
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
static int32_t sensor_dev_init(struct sensor_device_s *sen)
{
	int32_t ret;

	sen->osdev.devno = sen->port;

	ret = sensor_miscdev_init(sen);
	if (ret < 0)
		return ret;

	ret = sensor_device_init(sen);
	if (ret < 0) {
		sensor_miscdev_exit(sen);
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor device exit and destory chardev
 *
 * @param[in] sen: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_dev_exit(struct sensor_device_s *sen)
{
	sensor_device_exit(sen);
	sensor_miscdev_exit(sen);
	return;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief sensor driver init
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_sensor_module_init(void)
{
	int32_t ret, i;
	struct sensor_s *gsen = sensor_global();
	struct sensor_device_s *sen_a, *sen;

	if (gsen == NULL)
		return -EFAULT;

	if ((sensor_num == 0) || (sensor_num > SENSOR_NUM_MAX)) {
		sen_info(NULL, "sensor_num %d error set default %d\n",
			sensor_num, SENSOR_NUM_DEFAULT);
		sensor_num = SENSOR_NUM_DEFAULT;
	}

	sen_a = osal_kmalloc(sizeof(struct sensor_device_s) * sensor_num, GFP_KERNEL);
	if (sen_a == NULL) {
		sen_err(NULL, "alloc %d sensor device error\n", sensor_num);
		return -ENOMEM;
	}
	memset(sen_a, 0, sizeof(struct sensor_device_s) * sensor_num);

	for (i = 0; i < sensor_num; i++) {
		sen = &sen_a[i];
		sen->port = i;
		ret = sensor_dev_init(sen);
		if (ret < 0) {
			sen_err(NULL, "sensor_dev_init %d failed\n", i);
			while (i > 0) {
				i--;
				sensor_dev_exit(&sen_a[i]);
			}
			osal_kfree(sen_a);
			return ret;
		}
	}

	ret = sensor_driver_init();
	if (ret < 0)
		goto init_error_devexit;

	gsen->sen = sen_a;
	gsen->sen_num = sensor_num;

	sen_info(NULL, "init %d sensor v%u.%u done\n",
		gsen->sen_num, gsen->ver.major, gsen->ver.minor);

	return ret;

init_error_devexit:
	for (i = 0; i < sensor_num; i++)
		sensor_dev_exit(&sen_a[i]);
	osal_kfree(sen_a);

	return ret;
}

/**
 * @NO{S10E02C08I}
 * @ASIL{B}
 * @brief sensor driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @senign
 */
static void __exit hobot_sensor_module_exit(void)
{
	int32_t i;
	struct sensor_s *gsen = sensor_global();

	if (gsen == NULL)
		return;

	sensor_driver_exit();

	for (i = 0; i < gsen->sen_num; i++) {
		sensor_dev_exit(&gsen->sen[i]);
	}
	osal_kfree(gsen->sen);
	gsen->sen = NULL;

	sen_info(NULL, "exit %d sensor v%u.%u done\n",
		gsen->sen_num, gsen->ver.major, gsen->ver.minor);
	gsen->sen_num = 0;

	return;
}

late_initcall_sync(hobot_sensor_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_sensor_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT VIN SENSOR Driver"); /* PRQA S ALL */ /* linux macro */
