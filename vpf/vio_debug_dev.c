/**
 * @file: vio_debug_dev.c
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)    "[VIO dbg dev]:" fmt
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include "hobot_vpf_manager.h"
#include "vio_debug_dev.h"
#include "vio_debug_api.h"
/**
 * Purpose: vps module name for print
 * Value: NA
 * Range: hobot_vpf_ops.c
 * Attention: NA
 */
static const char* vps_stat_name[] = {
	"sif",
	"isp",
	"vse",
	"gdc",
	"n2d",
	"ipu",
	"codec"
};

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Print the statistic infomation of one pipeline chain;
 * @param[in] flow_id: pipe id;
 * @param[in] *buf: store statistic string;
 * @param[in] size: max string size
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_print_delay_log(s32 flow_id, char *buf, size_t size)/*PRQA S 2775*/
{
	u32 i, j;
	ssize_t offset = 0;
	ssize_t len;
	char module_name[7];
	struct vio_chain *vchain;
	struct module_stat *mstat;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vpf_get_drvdata();
	vchain = &vpf_dev->iscore.vchain[flow_id];
	len = snprintf(&buf[offset], size - (size_t)offset,
		"------------------------------- pipe %d vio info -------------------------------\n", flow_id);
	offset = offset + len;
	len = snprintf(&buf[offset], size - (size_t)offset,
		"frameid  module FS              FE              QB              DQ\n");
	offset = offset + len;
	for (i = 0; i <= N2D_MODULE; i++) {
		for (j = 0; j < MAX_DELAY_FRAMES; j++) {
			mstat = &vchain->mstat[i][j];
			sprintf(module_name, "%s", "      ");
			(void)memcpy(module_name, vps_stat_name[i], strlen(vps_stat_name[i]));
			len = snprintf(&buf[offset], size - (size_t)offset,
				"%08d %s %08llu.%06llu %08llu.%06llu %08llu.%06llu %08llu.%06llu\n",
				mstat->sinfo[STAT_FS].frameid,
				module_name,
				mstat->sinfo[STAT_FS].tv_sec % 100000000,
				mstat->sinfo[STAT_FS].tv_usec,
				mstat->sinfo[STAT_FE].tv_sec % 100000000,
				mstat->sinfo[STAT_FE].tv_usec,
				mstat->sinfo[STAT_QB].tv_sec % 100000000,
				mstat->sinfo[STAT_QB].tv_usec,
				mstat->sinfo[STAT_DQ].tv_sec % 100000000,
				mstat->sinfo[STAT_DQ].tv_usec);
			offset += len;
		}
		len = snprintf(&buf[offset], size - (size_t)offset, "\n");
		offset += len;
	}
	return offset;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Set the pipeline id which used to show debug information;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @param[in] buf: store information string;
 * @param[in] buf: information string length;
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vpf_fps_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	vpf_dev->flowid_mask = (u32)simple_strtoul(buf, NULL, 0);
	if (vpf_dev->flowid_mask == 0)
		vpf_dev->flowid_mask = (1 << VIO_MAX_STREAM) - 1u;

	return (ssize_t)len;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the fps of every ip for each pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vpf_fps_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	ssize_t offset;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	offset = vio_fps_snapshot(buf, PAGE_SIZE, vpf_dev->flowid_mask);

	return offset;
}
static DEVICE_ATTR(fps, 0660, vpf_fps_show, vpf_fps_store);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the fps of every ip for each pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vpf_fps_stats_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	ssize_t offset = 0;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	offset = vio_fps_stats(buf, PAGE_SIZE, vpf_dev->flowid_mask);

	return offset;
}
static DEVICE_ATTR(fps_stats, 0440, vpf_fps_stats_show, NULL);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the timestamp of every ip in pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_delay_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	ssize_t offset;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;

	offset = vio_print_delay_log(vpf_dev->stat_flow_id, buf, PAGE_SIZE);

	return offset;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Set the pipeline id which used to show debug information;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @param[in] buf: store information string;
 * @param[in] buf: information string length;
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	vpf_dev->stat_flow_id = (u32)simple_strtoul(buf, NULL, 0);
	if (vpf_dev->stat_flow_id >= VIO_MAX_STREAM)
		vpf_dev->stat_flow_id = VIO_MAX_STREAM - 1u;

	return (ssize_t)len;
}
static DEVICE_ATTR(vio_delay, 0660, vio_delay_show, vio_delay_store);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show drop frame id;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_drop_info_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 i, j, index;
	ssize_t len;
	ssize_t offset = 0;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;
	struct vio_core *iscore;
	struct vio_chain *vchain;
	struct vio_drop_mgr *drop_mgr;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	iscore = &vpf_dev->iscore;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vchain = &iscore->vchain[i];
		if (osal_test_bit(VIO_CHAIN_OPEN, &vchain->state) == 0)
			continue;
		len = snprintf(&buf[offset], PAGE_SIZE - (size_t)offset,
					"pipe%2d: ", i);
		offset += len;

		drop_mgr = &vchain->drop_mgr;
		index = drop_mgr->head_index;
		for (j = 0; j < DROP_INFO_NUM; j++) {
			len = snprintf(&buf[offset], PAGE_SIZE - (size_t)offset,
						"[F%08d] ", drop_mgr->drop_frameid[index]);
			offset += len;
			index = (index + DROP_INFO_NUM - 1) % DROP_INFO_NUM;
		}
		len = snprintf(&buf[offset], PAGE_SIZE - (size_t)offset, "\n");
		offset += len;
	}

	return offset;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that inject drop frame id;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @param[in] buf: store information string;
 * @param[in] buf: information string length;
 * @retval "> 0": success
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_drop_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	u32 flow_id;

	flow_id = (u32)simple_strtoul(buf, NULL, 0);
	vio_set_drop_info(flow_id, 0xffffffff);

	return (ssize_t)len;
}
static DEVICE_ATTR(drop_info, 0660, vio_drop_info_show, vio_drop_info_store);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the fps of every ip for each pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vpf_drop_stats_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	ssize_t offset = 0;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;

	offset = vio_drop_stats(buf, PAGE_SIZE, vpf_dev->flowid_mask);

	return offset;
}
static DEVICE_ATTR(drop_stats, 0440, vpf_drop_stats_show, NULL);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the fps of every ip for each pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vio_fmgr_stats_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	ssize_t offset = 0;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;

	offset = vio_fmgr_stats(buf, PAGE_SIZE, 0, vpf_dev->flowid_mask);

	return offset;
}
static DEVICE_ATTR(fmgr_stats, 0440, vio_fmgr_stats_show, NULL);/*PRQA S 4501,0636*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Debug interface that show the fps of every ip for each pipeline;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t vpf_delay_stats_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	ssize_t offset = 0;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	offset = vio_delay_stats(buf, PAGE_SIZE, vpf_dev->flowid_mask);

	return offset;
}
static DEVICE_ATTR(delay_stats, 0440, vpf_delay_stats_show, NULL);/*PRQA S 4501,0636*/

static ssize_t vio_path_stat_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	u32 i;
	ssize_t offset = 0;
	ssize_t len, size;
	struct vio_chain *vchain;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	vpf_device = (struct vpf_device *)dev_get_drvdata(dev);
	vpf_dev = (struct hobot_vpf_dev *)vpf_device->ip_dev;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vchain = &vpf_dev->iscore.vchain[i];
		size = strlen(vchain->path);
		if (size > 0) {
			if (osal_test_bit(VIO_CHAIN_OPEN, &vchain->state) == 0)
				len = snprintf(&buf[offset], PAGE_SIZE - offset, "(inactive)");
			else
				len = snprintf(&buf[offset], PAGE_SIZE - offset, "(active)");
			offset += len;

			memcpy(&buf[offset], vchain->path, size);
			offset += size;
		}
	}

	return offset;
}
static DEVICE_ATTR(path_stat, 0440, vio_path_stat_show, NULL);/*PRQA S 4501,0636*/
/**
 * Purpose: debug sys attributes
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct attribute *debug_attributes[] = {
	&dev_attr_fps.attr,
	&dev_attr_drop_info.attr,
	&dev_attr_fps_stats.attr,
	&dev_attr_drop_stats.attr,
	&dev_attr_delay_stats.attr,
	&dev_attr_fmgr_stats.attr,
	&dev_attr_vio_delay.attr,
	&dev_attr_path_stat.attr,
	NULL,
};

/**
 * Purpose: debug sys group attributes
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct attribute_group debug_attr_group = {
	.attrs	= debug_attributes,
};


s32 vio_debug_create(struct device *dev)
{
    s32 ret;

	ret = sysfs_create_group(&dev->kobj, &debug_attr_group);
	if (ret < 0)
		vio_err("%s: create debugfs failed (%d)\n", __func__, ret);

    return ret;
}

void vio_debug_destroy(struct device *dev)
{
    sysfs_remove_group(&dev->kobj, &debug_attr_group);
}

static int32_t vpf_fmgr_stats_show(struct seq_file *s, void *unused) /* PRQA S 3206 */
{
	char buf[DEBUG_SIZE];
	u32 len = 0;
	u32 i;

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		len = vio_fmgr_stats(buf, DEBUG_SIZE, i, 1 << i);
		if (len != 0) {
			seq_printf(s, "%s", buf);
			(void)memset(buf, 0, len);
		}
	}

	return 0;
}

static int32_t vpf_fmgr_stats_open(struct inode *inode, struct file *file) /* PRQA S 3673 */
{
	return single_open(file, vpf_fmgr_stats_show, inode->i_private);
}

static const struct file_operations fmgr_stats_fops = {
	.open = vpf_fmgr_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

s32 vpf_create_debug_file(struct hobot_vpf_dev *vpf_dev)
{
	s32 ret = 0;

	vpf_dev->debug_root = debugfs_create_dir("vps", NULL);
	if (vpf_dev->debug_root == NULL) {
		vio_err("%s: failed to create debugfs root directory.\n", __func__);
		return -EINVAL;
	}

	vpf_dev->debug_file_fmgr_stats = debugfs_create_file("fmgr_stats", 0664, /* PRQA S 0339,3120 */
						vpf_dev->debug_root,
						(void *)vpf_dev, &fmgr_stats_fops);
	if (IS_ERR(vpf_dev->debug_file_fmgr_stats)) {
		debugfs_remove_recursive(vpf_dev->debug_root);
		return PTR_ERR(vpf_dev->debug_file_fmgr_stats);
	}

	return ret;
}

void vpf_destroy_debug_file(struct hobot_vpf_dev *vpf_dev)
{
	debugfs_remove_recursive(vpf_dev->debug_file_fmgr_stats);
	debugfs_remove_recursive(vpf_dev->debug_root);
}
