/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_dev.c
 *
 * Copyright (C) 2020 horizon
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <vio_config.h>
#include <hobot_vpf_manager.h>
#include "vio_framemgr.h"
#include "vio_node_api.h"
#include <idu_cfg.h>

#include <hb_idu_common.h>
#include <hb_idu_hw.h>
#include "hobot_idu_vnode_ops.h"
#include "hobot_idu_vnode_dev.h"
#include "hobot_idu_vnode_debug.h"


// #define pr_fmt(fmt) "[IDU_VNODE]: " fmt

#define MODULE_NAME "IDU_VNODE"
#define DRV_VERSION "1.0.0"

int rst_en = 0;
module_param(rst_en, int, 0644); /*PRQA S 0605,0636,4501*/

static int32_t hbmem_format_to_planecnt(uint32_t format)
{
	int32_t plane_num = 0;

	// RGB format
	switch (format) {
		case MEM_PIX_FMT_RGB565:
		case MEM_PIX_FMT_RGB24:
		case MEM_PIX_FMT_ARGB:
		case MEM_PIX_FMT_RGBA:
		case MEM_PIX_FMT_RAW8:
		case MEM_PIX_FMT_UYVY422:
		case MEM_PIX_FMT_VYUY422:
		case MEM_PIX_FMT_YUYV422:
		case MEM_PIX_FMT_YVYU422:
			plane_num = 1;
			break;
		case MEM_PIX_FMT_NV16:
		case MEM_PIX_FMT_NV61:
		case MEM_PIX_FMT_NV12:
		case MEM_PIX_FMT_NV21:
			plane_num = 2;
			break;
		case MEM_PIX_FMT_YUV422P:
		case MEM_PIX_FMT_YUV420P:
			plane_num = 3;
			break;
		default:
			plane_num = -EINVAL;
			break;
	}

	return plane_num;
}

static int32_t hbmem_format_to_bpp(uint32_t format)
{
	int32_t bpp = 0;

	// RGB format
	switch (format) {
		case MEM_PIX_FMT_RGB565:
		case MEM_PIX_FMT_UYVY422:
		case MEM_PIX_FMT_VYUY422:
		case MEM_PIX_FMT_YUYV422:
		case MEM_PIX_FMT_YVYU422:
		case MEM_PIX_FMT_NV16:
		case MEM_PIX_FMT_NV61:
		case MEM_PIX_FMT_YUV422P:
			bpp = 16;
			break;
		case MEM_PIX_FMT_RGB24:
			bpp = 24;
			break;
		case MEM_PIX_FMT_ARGB:
		case MEM_PIX_FMT_RGBA:
			bpp = 32;
			break;
		case MEM_PIX_FMT_RAW8:
			bpp = 8;
			break;
		case MEM_PIX_FMT_NV12:
		case MEM_PIX_FMT_NV21:
		case MEM_PIX_FMT_YUV420P:
			bpp = 12;
			break;
		default:
			bpp = -EINVAL;
			break;
	}

	return bpp;
}

/**
 * @NO{S12E01C11}
 * @brief idu isr handle interface
 *
 * @param[in] irq: irq number
 * @param[in] data: file to handle
 * @param[out] None
 *
 * @retval "= 1": hanled interrupt
 *
 * @data_read None
 * @data_updated None
 * @callergraph
 * @design
 */
static irqreturn_t idu_vnode_isr(int this_irq, void *data)
{
	struct hobot_idu_dev *idu = (struct hobot_idu_dev *)data;
	uint32_t	      irq_status = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	uint64_t flags;
	uint32_t i;

	if (!IS_ERR_OR_NULL(idu->hw)) {
		// read and clear interrupt
		irq_status = dc_hw_get_irq_status(idu->port);
		dc_hw_clear_interrupt(idu->port);

		if (irq_status & BIT(23)) {
			idu->subnode.frame_end = 1;
			if (idu->hw->capture.base.enable) {
				vio_set_hw_free(&idu->vnode);
				vio_frame_done(&idu->subnode.idu_ochn_sdev[OCHN_WRITEBACK].vdev);
				wake_up(&idu->subnode.done);
			}
		}
		for (i = 0; i < IDU_ICH_MAX; i++) {
			if (irq_status & BIT(i+1)) {
				vio_frame_done(&idu->subnode.idu_ichn_sdev[i].vdev);
				framemgr = &idu->vnode.ich_subdev[i]->framemgr;
				vio_e_barrier_irqs(framemgr, flags);
				frame = peek_frame(framemgr, FS_COMPLETE);
				trans_frame(framemgr, frame, FS_USED);
				vio_x_barrier_irqr(framemgr, flags);
			}
		}

		if (irq_status & BIT(0)) {
			idu_handle_vsync(&idu->subnode.vsync);
		}
		if (irq_status & BIT(26)) {
			vio_err("Capture fifo overflow - %d\n", irq_status);
		}

		if (irq_status & (BIT(13) | BIT(14) | BIT(15) | BIT(16) | BIT(17) | BIT(18))) {
			vio_err("Layer buffer underflow - %d\n", irq_status);
		}
	}

	return IRQ_HANDLED;
}

/**
 * @NO{S12E01C11}
 * @ASIL{B}
 * @brief open callback interface of vpf framework
 *
 * @param[in] vctx: vpf contex info
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t idu_vpf_open(struct vio_video_ctx *vctx)
{
	int32_t ret;
	ret = idu_open(vctx, rst_en);

	return ret;
}

/**
 * @NO{S12E01C11}
 * @ASIL{B}
 * @brief close Callback interface of vpf framework
 *
 * @param[in] vctx: vpf contex info
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t idu_vpf_close(struct vio_video_ctx *vctx)
{
	int32_t ret;
	ret = idu_close(vctx, rst_en);

	return ret;
}

/**
 * @NO{S12E01C11}
 * @ASIL{B}
 * @brief close Callback interface of vpf framework
 *
 * @param[in] vctx: vpf contex info
 * @param[out] group_attr: buffer attribute of current pieline
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t idu_get_buf_attr(struct vio_video_ctx	  *vctx,
				struct vbuf_group_info *group_attr)
{
	int32_t		    ret = 0;
	struct idu_subdev  *subdev;
	struct idu_subnode *subnode;
	struct vio_subdev  *vdev;
	struct hobot_idu_dev *idu;
	int32_t format = 0;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct idu_subdev, vdev);
	subnode = subdev->subnode;
	idu = subnode->idu;

	if (vctx->id < VNODE_ID_SRC + IDU_ICHN_NUM) {
		group_attr->info[0].buf_attr.width = idu->hw->plane[vdev->id].chan_base.img_width;
		group_attr->info[0].buf_attr.height = idu->hw->plane[vdev->id].chan_base.img_height;
		group_attr->bit_map = 1;
		if (vctx->id == IDU_ICHN3 || vctx->id == IDU_ICHN4) {
			format = idu_rgbformat_to_hbmem(idu->hw->plane[vdev->id].chan_base.format);
		} else {
			format = idu_yuvformat_to_hbmem(idu->hw->plane[vdev->id].chan_base.format);
		}
		group_attr->info[0].buf_attr.format = format;
		group_attr->info[0].buf_attr.planecount = hbmem_format_to_planecnt(format);
		if (group_attr->info[0].buf_attr.planecount == 1) {
			group_attr->info[0].buf_attr.wstride =
				hbmem_format_to_bpp(group_attr->info[0].buf_attr.format) * group_attr->info[0].buf_attr.width / 8;
		} else {
			group_attr->info[0].buf_attr.wstride = group_attr->info[0].buf_attr.width;
		}
		group_attr->info[0].buf_attr.vstride = group_attr->info[0].buf_attr.height;
		group_attr->is_alloc = 0;
	}  else if (vctx->id == VNODE_ID_CAP + OCHN_WRITEBACK) {
		group_attr->info[0].buf_attr.format = idu_capture_to_hbmem(idu->hw->capture.base.format);
		group_attr->info[0].buf_attr.width = idu->hw->display.h_active;
		group_attr->info[0].buf_attr.height = idu->hw->display.v_active;
		group_attr->info[0].buf_attr.vstride = group_attr->info[0].buf_attr.height;
		group_attr->info[0].buf_attr.planecount = hbmem_format_to_planecnt(group_attr->info[0].buf_attr.format);
		if (group_attr->info[0].buf_attr.planecount == 1) {
			group_attr->info[0].buf_attr.wstride =
				hbmem_format_to_bpp(group_attr->info[0].buf_attr.format) * group_attr->info[0].buf_attr.width / 8;
		} else {
			group_attr->info[0].buf_attr.wstride = group_attr->info[0].buf_attr.width;
		}
		group_attr->bit_map = 1;
		group_attr->is_alloc = 0;
	}

	vio_info("[V%d]: format %d plane_cnt %d\n", vctx->id,
		 group_attr->info[0].buf_attr.format,
		 group_attr->info[0].buf_attr.planecount);

	return ret;
}

static int32_t idu_vpf_get_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	int32_t ret;
	ulong copy_ret;
	uint64_t req_seq;
	struct hobot_idu_dev *idu = (struct hobot_idu_dev *)vctx->device;

	switch (cmd) {
		case IDU_EXT_GET_WAIT_VSYNC:
			copy_ret = copy_from_user(&req_seq, (void __user *)arg, sizeof(req_seq));
			if (copy_ret != 0u) {
				vio_err("%s: failed to copy from user, ret = %ld\n", __func__, copy_ret);
				return -EFAULT;
			}
			vio_info("Request %lld vsync\n", req_seq);
			ret = idu_wait_vsync(&idu->subnode.vsync, req_seq);
			if (ret != 0u) {
				vio_err("%s: failed to wait %lld vsync signal, ret = %d\n", __func__, req_seq, ret);
				return -EFAULT;
			}
			break;
	}

	return ret;
}

static struct vio_version_info g_idu_vnode_version = {
	.major = 1,
	.minor = 0
};

static int32_t idu_vnode_get_version(struct vio_version_info * version)
{
	memcpy(version, &g_idu_vnode_version, sizeof(struct vio_version_info));

	return 0;
}

static struct vio_common_ops idu_vops = {
	.open = idu_vpf_open,
	.close = idu_vpf_close,
	.video_set_attr = idu_set_init_attr,
	.video_get_buf_attr = idu_get_buf_attr,
	.video_set_attr_ex = NULL,
	.video_get_attr = NULL,
	.video_set_ichn_attr = idu_set_ichn_attr,
	.video_set_ichn_attr_ex = idu_set_ichn_attr_ex,
	.video_set_ochn_attr = idu_set_ochn_attr,
	.video_set_ochn_attr_ex = idu_set_ochn_attr_ex,
	.video_start = idu_vpf_streamon,
	.video_stop = idu_vpf_streamoff,
	.video_g_ctrl = idu_vpf_get_ctrl,
	.video_get_version = idu_vnode_get_version,
};

/**
 * @NO{S12E01C11}
 * @brief idu rigister char device interface
 *
 * @param[in] vctx: vpf contex info
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t idu_device_node_init(struct hobot_idu_dev *idu)
{
	int32_t		 ret = 0;
	uint32_t	 j, k;
	char		 name[32];
	struct vio_node *vnode;

	vnode = &idu->vnode;
	vnode->id = DPU_MODULE;
	vnode->hw_id = idu->port;
	vnode->flow_id = INVALID_FLOW_ID;
	for (j = 0; j < IDU_ICHN_NUM; j++) {
		vnode->ich_subdev[j] =
			&idu->subnode.idu_ichn_sdev[j].vdev;
		vnode->ich_subdev[j]->id = j; //channal id
		vnode->ich_subdev[j]->vdev_work = idu_channel_frame_handle;
		vnode->active_ich |= 1 << j;
		idu->subnode.idu_ichn_sdev[j].subnode =
			&idu->subnode;
		idu->subnode.idu_ichn_sdev[j].vnode = vnode;
		idu->subnode.idu_ichn_sdev[j].vdev.vnode = vnode;
	}

	for (k = 0; k < OCHN_NUM; k++) {
		vnode->och_subdev[k] =
			&idu->subnode.idu_ochn_sdev[k].vdev;
		vnode->och_subdev[k]->id = k + VNODE_ID_CAP;
		vnode->active_och |= 1 << k;
		idu->subnode.idu_ochn_sdev[k].subnode =
			&idu->subnode;
		idu->subnode.idu_ochn_sdev[k].vnode = vnode;
		idu->subnode.idu_ochn_sdev[k].vdev.vnode = vnode;
	}

	vnode->gtask = &idu->gtask;
	vnode->gtask->no_worker = 1;
	vnode->frame_work = idu_frame_work;
	vnode->allow_bind = idu_allow_bind;
	idu->subnode.flow_id = 0;
	idu->subnode.idu = idu;

	for (j = 0; j < IDU_ICHN_NUM; j++) {
		idu->vps_device[j].vps_ops = &idu_vops;
		idu->vps_device[j].ip_dev = idu;
		idu->vps_device[j].vid = VNODE_ID_SRC + j;
		idu->vps_device[j].vnode = vnode;
		idu->vps_device[j].max_ctx = MAX_IDU_PROCESS_PIPELINE;
		idu->vps_device[j].iommu_dev = &idu->pdev->dev;
		sprintf(name, "idu%d_ich%d", idu->port, j);
		vio_register_device_node(name, &idu->vps_device[j]);
	}

	for (k = 0; k < OCHN_NUM; k++) {
		idu->vps_device[IDU_ICHN_NUM + k].vps_ops = &idu_vops;
		idu->vps_device[IDU_ICHN_NUM + k].ip_dev = idu;
		idu->vps_device[IDU_ICHN_NUM + k].vid = VNODE_ID_CAP + k;
		idu->vps_device[IDU_ICHN_NUM + k].vnode = vnode;
		idu->vps_device[IDU_ICHN_NUM + k].max_ctx =
			MAX_IDU_PROCESS_PIPELINE;
		idu->vps_device[IDU_ICHN_NUM + k].iommu_dev = &idu->pdev->dev;
		sprintf(name, "idu%d_och%d", idu->port, k);
		vio_register_device_node(name,
					 &idu->vps_device[IDU_ICHN_NUM + k]);
	}

	vio_info("ichn_active 0x%x, ochn_active 0x%x\n", vnode[0].active_ich,
		 vnode[0].active_och);

	return ret;
}

/**
 * @NO{S12E01C11}
 * @brief idu platform driver probe interface
 *
 * @param[in] pdev: platform device struct
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_idu_probe(struct platform_device *pdev)
{
	int32_t		      ret;
	struct hobot_idu_dev *idu;
	struct device	     *dev = &pdev->dev;

	idu = (struct hobot_idu_dev *)devm_kzalloc(
		dev, sizeof(struct hobot_idu_dev), GFP_KERNEL);
	if (idu == NULL) {
		dev_err(dev, "idu is NULL");
		ret = -ENOMEM;

		return ret;
	}

	idu->port = of_alias_get_id(dev->of_node, "iduvnode");
	if (idu->port < 0 && idu->port >= IDU_DEV_NUM) {
		dev_err(dev, "Failed to get idu port(%d)", idu->port);
		ret = -EINVAL;
		return ret;
	}

	/* Get func IRQ  number */
	idu->irq = platform_get_irq(pdev, 0);
	if (idu->irq < 0) {
		dev_err(dev, "Failed to get idu_irq(%d)", idu->irq);
		ret = -EBUSY;
		return ret;
	}
	ret = devm_request_irq(dev, (uint32_t)idu->irq, idu_vnode_isr,
			       IRQF_TRIGGER_HIGH | IRQF_SHARED, dev_name(dev),
			       (void *)idu);
	if (ret != 0) {
		dev_err(dev, "request_irq(IRQ_idu %d) is fail(%d)", idu->irq,
			ret);
		return ret;
	}

	idu->pdev = pdev;
	ret = idu_device_node_init(idu);
	if (ret < 0) {
		dev_err(dev, "idu device node init is fail(%d)", ret);
		ret = -EBUSY;
		return ret;
	}
	idu->subnode.frame_end = 0;
	init_waitqueue_head(&idu->subnode.done);
	osal_mutex_init(&idu->mlock);
	idu_vsync_init(&idu->subnode.vsync);

	idu->debugfs = hobot_idu_vnode_create_debugfs(&idu->debug, idu->port);
	if (NULL == idu->debugfs) {
		dev_err(dev, "idu_vnode create debugfs file(%d)", ret);
		ret = -EFAULT;
		return ret;
	}

	platform_set_drvdata(pdev, (void *)idu);

	dev_info(dev, "%s(%d) sucess\n", __func__, ret);
	return 0;
}

/**
 * @NO{S12E01C11}
 * @brief idu platform driver remove interface
 *
 * @param[in] pdev: platform device struct
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_idu_remove(struct platform_device *pdev) /*PRQA S 3673*/
{
	int32_t		      ret = 0;
	uint32_t	      i;
	struct hobot_idu_dev *idu;

	if (pdev == NULL) {
		vio_err("%s: pdev = NULL\n", __func__);
		return -EFAULT;
	}
	idu = (struct hobot_idu_dev *)platform_get_drvdata(pdev);

	// idu_cfg_buffer_free(idu, &idu->cfg_buff);
	for (i = 0; i < IDU_MAX_DEVICE; i++) {
		vio_unregister_device_node(&idu->vps_device[i]);
	}

	dev_info(&idu->pdev->dev, "%s\n", __func__);
	devm_kfree(&pdev->dev, (void *)idu);

	return ret;
}

static const struct of_device_id idu_match[] = {
	{
		.compatible = "hobot,hobot-vnode-idu",
	},
	{ /* sentinel */ } /*PRQA S 1041*/
};

MODULE_DEVICE_TABLE(of, idu_match); /*PRQA S 0605*/

static struct platform_driver idu_driver = { .probe = hobot_idu_probe,
					     .remove = hobot_idu_remove,
					     .driver = {
						     .name = MODULE_NAME,
						     .owner = THIS_MODULE,
						     // .pm = &idu_pm_ops,
						     .of_match_table =
							     idu_match,
					     } };

/**
 * @NO{S12E01C11}
 * @brief idu platform driver register interface
 *
 * @param[in] None
 * @param[out] None
 *
 * @retval "= 0": success
 * @retval "< 0": failure
 *
 * @data_read None
 * @data_updated None
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_idu_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&idu_driver); /*PRQA S 3469*/
	if (ret != 0) {
		pr_err("platform_driver_register failed: %d\n", ret);
	}

	return ret;
}

/**
 * @NO{S12E01C11}
 * @brief idu platform driver unregister interface
 *
 * @param[in] None
 * @param[out] None
 *
 *
 * @data_read None
 * @data_updated None
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_idu_exit(void)
{
	platform_driver_unregister(&idu_driver);
}

late_initcall(hobot_idu_init); /*PRQA S 0605,3219*/
module_exit(hobot_idu_exit); /*PRQA S 0605,3219*/

MODULE_VERSION(DRV_VERSION);
MODULE_DESCRIPTION(MODULE_NAME); /*PRQA S 0633*/
MODULE_LICENSE("GPL"); /*PRQA S 0633,0629*/
