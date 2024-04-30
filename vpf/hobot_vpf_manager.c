/**
 * @file: hobot_vpf_manager.c
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
#define pr_fmt(fmt)    "[VPF mgr]:" fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/poll.h>
#include <linux/platform_device.h>

#include "hobot_vpf_manager.h"
#include "hobot_vpf_ops.h"
#include "vio_debug_dev.h"

#define VPS_NAME  "vps"

/**
 * Purpose: multi-process flag for vio service; User set this sys paramater
 * to use vio service or not;
 * Value: 1~
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
int vio_mp_en = 0;
module_param(vio_mp_en, int, 0644);/*PRQA S 0605,0636,4501*/

/**
 * Purpose: point to hobot_vpf_dev struct, for extern interface
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct hobot_vpf_dev *g_vpf_dev;

void vpf_set_drvdata(struct hobot_vpf_dev *vpf_dev)
{
	g_vpf_dev = vpf_dev;
}

struct hobot_vpf_dev *vpf_get_drvdata(void)
{
	return g_vpf_dev;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: VPF ioctl function of device node;
 * @param[in] *file: point to struct file instance;
 * @param[in] cmd: Interactive command that device driver performs the corresponding operation according to the cmd.
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static long hobot_vpf_manager_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	s32 ret;
	struct vio_video_ctx *vctx;

	vctx = (struct vio_video_ctx *)file->private_data;
	if (vctx == NULL) {
		vio_err("%s: vctx = NULL\n", __func__);
		return -EFAULT;
	}

	if (_IOC_TYPE(cmd) != (u32)VIO_IOC_MAGIC)
		return -ENOTTY;

	ret = vpf_manager_ioctl(vctx, cmd, arg);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: VPF open function, create vctx instane and call open function of ip driver; 
 * @param[in] *inode: point to struct inode instance;
 * @param[in] *file: point to struct file instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 hobot_vpf_manager_open(struct inode *inode, struct file *file)
{
	s32 ret = 0;
	u32 minor;
	struct vio_video_ctx *vctx;
	struct vpf_device *vpf_device;
	struct hobot_vpf_dev *vpf_dev;

	minor = MINOR(inode->i_rdev);
	vctx = (struct vio_video_ctx *)kzalloc(sizeof(struct vio_video_ctx), GFP_ATOMIC);
	if (vctx == NULL) {
		vio_err("%s: kzalloc is fail", __func__);
		return -ENOMEM;
	}

	vpf_dev = vpf_get_drvdata();
	vctx->file = file;
	file->private_data = (void *)vctx;
	vpf_device = vpf_dev->vpf_device[minor];
	ret = vpf_device_open(vctx, vpf_device);
	if (ret < 0) {
		osal_kfree(vctx);
		return ret;
	}

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Poll notify function of device node;
 *
 * Multi threads could get the same poll event, but we don't care;
 * we suggest only one thread get frame for one context;
 * @param[in] *file: point to struct file instance;
 * @param[in] *wait: point to struct poll_table_struct instance;
 * @retval "= 0": sleep in wait table
 * @retval "> 0": event occur
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static u32 hobot_vpf_manager_poll(struct file *file, struct poll_table_struct *wait)
{
	u32 ret = 0;
	u64 flags = 0;
	struct vio_video_ctx *vctx;
	struct vio_framemgr *framemgr;
	osal_list_head_t *done_list;

	vctx = (struct vio_video_ctx *)file->private_data;
	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_warn("%s: invalid POLL is requested(%llX), please ignore", __func__, vctx->state);
		return POLLHUP;
	}

	poll_wait(file, &vctx->done_wq, wait);
	vio_dbg("[S%d][%s] %s: event = %d\n", vctx->flow_id, vctx->name, __func__, vctx->event);

	framemgr = vctx->framemgr;
	done_list = &framemgr->queued_list[FS_COMPLETE];
	vio_e_barrier_irqs(framemgr, flags);
	if (osal_list_empty(done_list) == 0) {
		vio_x_barrier_irqr(framemgr, flags);
		return POLLIN;
	}
	vio_x_barrier_irqr(framemgr, flags);

	if ((vctx->event == (u32)VIO_FRAME_DONE) ||
		(vctx->event == (u32)VIO_FRAME_PREINT) ||
		(vctx->event == (u32)VIO_FRAME_DROP)) {
		vctx->event = 0;
		ret = POLLIN;
	}
	if (vctx->event == (u32)VIO_FRAME_NDONE) {
		vctx->event = 0;
		ret = POLLERR;
	}
	if (vctx->event == (u32)VIO_FRAME_HUP) {
		vctx->event = 0;
		ret = POLLHUP;
	}

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: VPF close function, destroy vctx instane and call close function of ip driver; 
 * @param[in] *inode: point to struct inode instance;
 * @param[in] *file: point to struct file instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 hobot_vpf_manager_close(struct inode *inode, struct file *file)
{
	s32 ret = 0;
	struct vio_video_ctx *vctx;

	vctx = (struct vio_video_ctx *)file->private_data;
	vpf_device_close(vctx);
	osal_kfree(vctx);
	file->private_data = NULL;

	return ret;
}

/**
 * Purpose: vflow file operation functions
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct file_operations hobot_vpf_manager_fops = {
	.owner = THIS_MODULE,
	.open = hobot_vpf_manager_open,
	.poll = hobot_vpf_manager_poll,
	.release = hobot_vpf_manager_close,
	.unlocked_ioctl = hobot_vpf_manager_ioctl,
	.compat_ioctl = hobot_vpf_manager_ioctl,
};

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Alloc free minor number of device node;
 * @param[in] *vpf_dev: point to struct hobot_vpf_dev instance
 * @retval ">= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vpf_alloc_minor_number(struct hobot_vpf_dev *vpf_dev)
{
	s32 i,j;
	s32 minor_num;

	osal_mutex_lock(&vpf_dev->mlock);
	for (i = 0; i < MINOR_NUM; i++) {
		for (j = 0; j < MINOR_BITS; j++) {
			if ((vpf_dev->minor[i] & 1 << j) == 0) {
				minor_num = i * MINOR_BITS + j;
				vpf_dev->minor[i] |= 1 << j;
				vio_info("%s: minor[%d] bit%d is used", __func__, i, j);
				osal_mutex_unlock(&vpf_dev->mlock);
				return minor_num;
			}
		}
	}
	osal_mutex_unlock(&vpf_dev->mlock);

	vio_err("%s: can not alloc minor id", __func__);
	return -EFAULT;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Free minor number of device node;
 * @param[in] *vpf_dev: point to struct hobot_vpf_dev instance
 * @param[in] minor_num: minor number of device node;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vpf_free_minor_number(struct hobot_vpf_dev *vpf_dev, s32 minor_num)
{
	osal_mutex_lock(&vpf_dev->mlock);
	if (minor_num < MAXIMUM_DEVICE && minor_num > 0) {
		vpf_dev->minor[minor_num / 32] &= ~(1 << (minor_num % MINOR_BITS));
		vio_info("%s: minor[%d] bit%d is free", __func__,
			minor_num / MINOR_BITS, minor_num % MINOR_BITS);
	}
	osal_mutex_unlock(&vpf_dev->mlock);
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Null function, only for cdev_del;
 * @param[in] *device: point to struct device instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vpf_device_release(struct device *device)
{
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Common function to register device node of vpf device;
 * @param[in] *node_name: device node name;
 * @param[in] *vpf_device: point to struct vpf_device instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_register_device_node(char *node_name, struct vpf_device *vpf_device)
{
	s32 ret;
	dev_t devno;
	struct hobot_vpf_dev *vpf_dev;

	if (vpf_device == NULL || node_name == NULL) {
		vio_err("%s: vpf device is NULL or node_name is NULL\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	ret = vpf_device_probe(vpf_device);
	if (ret < 0)
		goto err;

	vpf_dev = vpf_get_drvdata();
	vpf_device->minor = vpf_alloc_minor_number(vpf_dev);
	if (vpf_device->minor < 0) {
		ret = -EFAULT;
		goto err;
	}
	devno = MKDEV(MAJOR(vpf_dev->devno), vpf_device->minor);

	vpf_device->cdev = cdev_alloc();
	if (vpf_device->cdev == NULL) {
		ret = -ENOMEM;
		goto cleanup;
	}

	cdev_init(vpf_device->cdev, &hobot_vpf_manager_fops);
	vpf_device->cdev->owner = THIS_MODULE;
	if (vpf_device->owner != NULL)
		vpf_device->cdev->owner = vpf_device->owner;

	ret = cdev_add(vpf_device->cdev, devno, 1);
	if (ret != 0) {
		vio_err("%s: add vpf_device cdev error, ret(%d)", __func__, ret);
		osal_kfree(vpf_device->cdev);
		vpf_device->cdev = NULL;
		goto cleanup;
	}

	// create sys node;
	vpf_device->dev.class = vpf_dev->class;
	vpf_device->dev.devt = devno;
	vpf_device->dev.parent = vpf_dev->dev;
	dev_set_name(&vpf_device->dev, node_name);
	ret = device_register(&vpf_device->dev);
	if (ret < 0) {
		vio_err("%s: device_register failed, ret(%d)\n", __func__, ret);
		goto cleanup;
	}
	vpf_device->dev.release = vpf_device_release;
	vpf_dev->vpf_device[vpf_device->minor] = vpf_device;
	(void)memcpy(vpf_device->name, node_name, NODE_NAME_SIZE);

	return 0;
cleanup:
	if (vpf_device->cdev)
		cdev_del(vpf_device->cdev);

	vpf_free_minor_number(vpf_dev, vpf_device->minor);
	vpf_dev->vpf_device[vpf_device->minor] = NULL;
err:
	return ret;
}
EXPORT_SYMBOL(vio_register_device_node);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Common function to unregister device node of vpf device;
 * @param[in] *vpf_device: point to struct vpf_device instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_unregister_device_node(struct vpf_device *vpf_device)
{
	struct hobot_vpf_dev *vpf_dev;
	dev_t devno;

	if (vpf_device == NULL) {
		vio_err("%s: vpf_device is NULL\n", __func__);
		return -EFAULT;
	}

	vpf_dev = vpf_get_drvdata();
	devno = MKDEV(MAJOR(vpf_dev->devno), vpf_device->minor);

	if (vpf_device->cdev) {
		cdev_del(vpf_device->cdev);
		vpf_device->cdev = NULL;
	}

	vpf_free_minor_number(vpf_dev, vpf_device->minor);
	vpf_dev->vpf_device[vpf_device->minor] = NULL;
	device_unregister(&vpf_device->dev);

	return 0;
}
EXPORT_SYMBOL(vio_unregister_device_node);

/**
 * Purpose: vflow callback functions
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct vio_common_ops vflow_vops = {
	.open = vpf_subdev_open,
	.close = vpf_subdev_close,
	.video_get_version = vpf_subdev_get_version,
};

/**
 * Purpose: empty frame id function
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static void empty_get_frame_id(struct vio_node *vnode, struct frame_id_desc *frameid)
{
	vio_info("%s: empty function", __func__);
}

static s32 empty_camsys_reset(u32 module, u32 rst_flag)
{
	vio_info("%s: empty function", __func__);
	return 0;
}

static int32_t empty_vtrace_send(uint32_t module_type, uint32_t param_type,
	uint32_t *param, uint32_t flow_id, uint32_t frame_id, uint32_t ctx_id, uint32_t chnid)
{
	vio_dbg("%s: empty function", __func__);
	return 0;
}
/**
 * Purpose: dbg interface
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct dbg_interface_ops g_dbg_cops = {
	.vtrace_send = empty_vtrace_send,
};

/**
 * Purpose: cim interface
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct cim_interface_ops g_cim_cops = {
	.get_frame_id = empty_get_frame_id,
};

/**
 * Purpose: camsys interface
 * Value: NA
 * Range: hobot_vpf_manager.c
 * Attention: NA
 */
static struct camsys_interface_ops g_camsys_cops = {
	.ip_reset_func = empty_camsys_reset,
};
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Register device node;
 * @param[in] *vpf_dev: point to struct hobot_vpf_dev instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 hobot_vpf_device_node_init(struct hobot_vpf_dev *vpf_dev)
{
	s32 ret;
	char name[32];
	struct device *dev;
	struct vpf_device *vpf_device;

	snprintf(name, sizeof(name), "flow");
	ret = alloc_chrdev_region(&vpf_dev->devno, 0, MAXIMUM_DEVICE, name);
	if (ret < 0) {
		vio_err("%s: alloc chrdev flow failed, ret(%d)\n", __func__, ret);
		return ret;
	}

	cdev_init(&vpf_dev->cdev, &hobot_vpf_manager_fops);
	vpf_dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&vpf_dev->cdev, vpf_dev->devno, MAXIMUM_DEVICE);
	if (ret != 0) {
		vio_err("%s: add vpf cdev failed, ret(%d)\n", __func__, ret);
		unregister_chrdev_region(vpf_dev->devno, MAXIMUM_DEVICE);
		return ret;
	}

	vpf_dev->class = class_create(THIS_MODULE, VPS_NAME);
	dev = device_create(vpf_dev->class, NULL, MKDEV(MAJOR(vpf_dev->devno), 0), NULL, name);
	if (IS_ERR((void *)dev)) {
		class_destroy(vpf_dev->class);
		unregister_chrdev_region(vpf_dev->devno, MAXIMUM_DEVICE);
		return -EINVAL;
	}
	vpf_dev->dev = dev;
	vpf_dev->minor[0] = 1;
	vpf_device = &vpf_dev->vflow_dev;
	vpf_device->minor = 0;
	vpf_device->ip_dev = vpf_dev;
	vpf_device->vps_ops = &vflow_vops;
	vpf_dev->vpf_device[0] = vpf_device;
	memcpy(vpf_device->name, name, sizeof(vpf_device->name));

	dev_set_drvdata(vpf_dev->dev, vpf_device);
	ret = vio_debug_create(dev);
	if (ret < 0)
		vio_err("%s: create debugfs failed (%d)\n", __func__, ret);

	return ret;
}

s32 hobot_vpf_manager_probe(void)
{
	s32 ret = 0;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = (struct hobot_vpf_dev *)kzalloc(sizeof(struct hobot_vpf_dev), GFP_ATOMIC);
	if (vpf_dev == NULL) {
		vio_err("%s: vpf_dev is NULL\n", __func__);
		return -ENOMEM;
	}
	ret = vio_ion_create();
	if (ret < 0) {
		osal_kfree(vpf_dev);
		return ret;
	}

	ret = vpf_create_debug_file(vpf_dev);
	if (ret < 0) {
		osal_kfree(vpf_dev);
		vio_ion_destroy();
		return ret;
	}

	ret = hobot_vpf_device_node_init(vpf_dev);
	if (ret < 0) {
		vpf_destroy_debug_file(vpf_dev);
		osal_kfree(vpf_dev);
		vio_ion_destroy();
		return ret;
	}

	osal_mutex_init(&vpf_dev->mlock);
	osal_mutex_init(&vpf_dev->iscore.mlock);
	osal_atomic_set(&vpf_dev->open_cnt, 0);
	vpf_set_drvdata(vpf_dev);
	vio_get_callback_ops(&g_cim_cops, VIN_MODULE, COPS_0);
	vio_get_callback_ops(&g_dbg_cops, DPU_MODULE, COPS_0);	//vtrace tmp using ipu module id
	vio_get_callback_ops(&g_camsys_cops, VIN_MODULE, COPS_7);
	vpf_dev->flowid_mask = (1 << VIO_MAX_STREAM) - 1u;

	return ret;
}

void hobot_vpf_manager_remove(void)
{
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vpf_get_drvdata();
	vpf_destroy_debug_file(vpf_dev);
	vio_debug_destroy(vpf_dev->dev);
	device_destroy(vpf_dev->class, vpf_dev->devno);
	class_destroy(vpf_dev->class);
	cdev_del(&vpf_dev->cdev);
	unregister_chrdev_region(vpf_dev->devno, MAXIMUM_DEVICE);
	osal_kfree(vpf_dev);
	vpf_set_drvdata(NULL);
	vio_ion_destroy();
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: VPF manager device intialization function;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static int __init hobot_vpf_manager_init(void)
{
	s32 ret = 0;

	ret = hobot_vpf_manager_probe();

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: VPF manager device deintialization function;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_vpf_manager_exit(void)
{
	hobot_vpf_manager_remove();
}

#ifndef CONFIG_PCIE_HOBOT_EP_AI
module_init(hobot_vpf_manager_init);
module_exit(hobot_vpf_manager_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.ai>");
MODULE_DESCRIPTION("hobot VPF Manager driver");
MODULE_LICENSE("GPL");
#endif
