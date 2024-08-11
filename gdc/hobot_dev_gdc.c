/**
 * @file: hobot_dev_gdc.c
 * @brief       gdc driver
 * @details     provide device driver interface
 * @author      kaikai.sun@horizon.ai
 * @date        2023-3-31
 * @version     v0.0.1
 * @copyright   Copyright (C) 2023 Horizon Robotics Inc.
 * @NO{S09E03C01}
 * @ASIL{B}
 */
#include "gdc_config.h"
#define pr_fmt(fmt)    "[GDC dev]:" fmt

#ifndef HOBOT_MCU_CAMSYS
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/poll.h>
#endif

#include "hobot_dev_gdc.h"
#include "gdc_hw_api.h"
#include "hobot_gdc_ops.h"
#include "cam_ctrl.h"

/**
 * @def MODULE_NAME
 * module name
 * @NO{S09E03C01}
 */
#define MODULE_NAME "HOBOT GDC"

/**
 * Purpose: gdc reset flag
 * Value: 0:disable reset when exiting; 1:enable reset when exiting.
 * Range: gdc_dev.c
 * Attention: NA
 */
static int g_rst_en = 0;

#ifndef HOBOT_MCU_CAMSYS
module_param(g_rst_en, int, 0644);/*PRQA S 0605,0636,4501*/
#endif

static struct hobot_gdc_dev *g_gdc_dev[GDC_MAX_HW_ID];

/* GDC STL use this internal interface to get g_gdc_dev point (hw_id < GDC_MAX_HW_ID)*/
struct hobot_gdc_dev *gdc_get_drvdata(u32 hw_id)
{
	if (hw_id >= GDC_MAX_HW_ID)
		return NULL;

	return g_gdc_dev[hw_id];
}
EXPORT_SYMBOL(gdc_get_drvdata);

int32_t gdc_get_struct_size(struct vio_struct_size *vio_size)
{
	int32_t ret = 0;

	switch (vio_size->type)
	{
	case BASE_ATTR:
		vio_size->size = sizeof(gdc_attr_t);
		break;
	case ICHN_ATTR:
		vio_size->size = sizeof(gdc_ichn_attr_t);
		break;
	case OCHN_ATTR:
		vio_size->size = sizeof(gdc_ochn_attr_t);
		break;
	case EX_ATTR:
		vio_size->size = 0;
		break;
	case OCHN_EX_ATTR:
		vio_size->size = 0;
		break;
	default:
		ret = -EINVAL;
		vio_err("Unknown vin_node struct type-%d\n", vio_size->type);
		break;
	}

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief Configuration initial parameters function of GDC
* @param[in] *vctx: vpf contex info
* @param[in] arg: User configuration parameters
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
static s32 gdc_video_set_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_attr_t gdc_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	copy_ret = osal_copy_from_app((void *)&gdc_attr, (void __user *) arg, sizeof(gdc_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	gdc_attr_trans_to_settings(&gdc_attr, NULL, NULL, &subdev->gdc_setting);

	if (vdev->id == VNODE_ID_SRC) {
		ret = gdc_iommu_map(subdev);
		if (ret < 0)
			return ret;

		vdev->leader = 1;
	}

	osal_set_bit((s32)VIO_SUBDEV_DMA_OUTPUT, &vdev->state);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief Get Configuration initial parameters function of GDC
* @param[in] *vctx: vpf contex info
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] arg: User configuration parameters
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 gdc_video_get_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_attr_t gdc_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/

	gdc_settings_trans_to_attr(&gdc_attr, NULL, NULL, &subdev->gdc_setting);

	copy_ret = osal_copy_to_app((void __user *) arg, (void *)&gdc_attr, sizeof(gdc_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

static s32 gdc_ichn_attr_check(gdc_ichn_attr_t *ichn_attr)
{
	vpf_param_range_check(ichn_attr->input_width, 32, 4096);
	vpf_param_range_check(ichn_attr->input_height, 32, 4096);

	return 0;
}

static s32 gdc_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_ichn_attr_t ichn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	copy_ret = osal_copy_from_app((void *)&ichn_attr, (void __user *) arg, sizeof(gdc_ichn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = gdc_ichn_attr_check(&ichn_attr);
	if (ret)
		return ret;

	gdc_attr_trans_to_settings(NULL, &ichn_attr, NULL, &subdev->gdc_setting);

	return ret;
}

static s32 gdc_get_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_ichn_attr_t ichn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/

	gdc_settings_trans_to_attr(NULL, &ichn_attr, NULL, &subdev->gdc_setting);

	copy_ret = osal_copy_to_app((void __user *) arg, (void *)&ichn_attr, sizeof(gdc_ichn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

static s32 gdc_ochn_attr_check(gdc_ochn_attr_t *ochn_attr)
{
	vpf_param_range_check(ochn_attr->output_width, 32, 4096);
	vpf_param_range_check(ochn_attr->output_height, 32, 4096);

	return 0;
}

static s32 gdc_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev, *srcdev;
	struct vio_subdev *vdev;
	gdc_ochn_attr_t ochn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	copy_ret = osal_copy_from_app((void *)&ochn_attr, (void __user *) arg, sizeof(gdc_ochn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	if (vctx->ctx_id >= VIO_MAX_STREAM) {
		vio_err("invalid ctxid %d\n", vctx->ctx_id);
		return -EFAULT;
	}

	ret = gdc_ochn_attr_check(&ochn_attr);
	if (ret)
		return ret;

	gdc_attr_trans_to_settings(NULL, NULL, &ochn_attr, &subdev->gdc_setting);
	srcdev = &subdev->gdc->subdev[vctx->ctx_id][0];
	gdc_attr_trans_to_settings(NULL, NULL, &ochn_attr, &srcdev->gdc_setting);

	return ret;
}

static s32 gdc_get_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_ochn_attr_t ochn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/

	gdc_settings_trans_to_attr(NULL, NULL, &ochn_attr, &subdev->gdc_setting);

	copy_ret = osal_copy_to_app((void __user *) arg, (void *)&ochn_attr, sizeof(gdc_ochn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief Get buffer attribute function of gdc
* @param[in] *vctx: vpf contex info
* @retval "= 0": success
* @retval "< 0": failure
* @param[out] *group_attr: buffer attribute parameters
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 gdc_video_reqbufs(struct vio_video_ctx *vctx, struct vbuf_group_info *group_attr)
{
	s32 ret = 0;
	struct gdc_subdev *subdev;
	struct vio_subdev *vdev;
	gdc_settings_t *gdc_cfg;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct gdc_subdev, vdev);/*PRQA S 2810,0497*/
	gdc_cfg = &subdev->gdc_setting;

	if (vctx->id == VNODE_ID_SRC) {
		group_attr->bit_map = 1;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
		group_attr->info[0].buf_attr.planecount = 2;
		group_attr->info[0].buf_attr.width = gdc_cfg->gdc_config.input_width;
		group_attr->info[0].buf_attr.height = gdc_cfg->gdc_config.input_height;
		group_attr->info[0].buf_attr.wstride = gdc_cfg->gdc_config.input_stride;
		group_attr->info[0].buf_attr.vstride = gdc_cfg->gdc_config.input_height;
	} else {
		group_attr->bit_map = 1;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
		group_attr->info[0].buf_attr.planecount = 2;
		group_attr->info[0].buf_attr.width = gdc_cfg->gdc_config.output_width;
		group_attr->info[0].buf_attr.height = gdc_cfg->gdc_config.output_height;
		group_attr->info[0].buf_attr.wstride = gdc_cfg->gdc_config.output_stride;
		group_attr->info[0].buf_attr.vstride = gdc_cfg->gdc_config.output_height;
		group_attr->is_contig = 1;
	}

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC isr handle interface
* @param[in] irq: irq number
* @param[in] *data: file to handle
* @retval "= 1": hanled interrupt
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static irqreturn_t gdc_isr(s32 irq, void *data)
{
	struct hobot_gdc_dev *gdc;
	bool is_done = false;
	gdc = (struct hobot_gdc_dev *)data;
#if defined CONFIG_HOBOT_J5
	status = gdc_get_irq_status(gdc->base_reg);
#elif defined (CONFIG_HOBOT_VIO_STL)
	status = gdc_fusa_get_irq_status(gdc->base_reg);
#endif

	get_gdc_intr_stat_and_clear((struct cam_ctrl_device *)gdc->wrap, NULL, &is_done);
	if (is_done)
		gdc_handle_interrupt(gdc, is_done);

	return IRQ_HANDLED;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief turn on frame stream Callback interface of vpf framework
* @param[in] *vctx: vpf contex info
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
static s32 gdc_video_streamon(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct vio_node *vnode;
	struct vio_subdev *vdev;

	vctx->event = 0;
	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vnode->leader = 1;
	vio_info("[%s][S%d][C%d] %s leader %d\n", vctx->name, vnode->flow_id,
		vnode->ctx_id, __func__, vdev->leader);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief turn off frame stream Callback interface of vpf framework
* @param[in] *vctx: vpf contex info
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
static s32 gdc_video_streamoff(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief close Callback interface of vpf framework
* @param[in] *vctx: vpf contex info
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
static s32 gdc_vpf_close(struct vio_video_ctx *vctx)
{
	s32 ret;

	ret = gdc_subdev_close(vctx, g_rst_en);

	return ret;
}


static struct vio_common_ops gdc_vops = {
	.open = gdc_subdev_open,
	.close = gdc_vpf_close,
	.video_set_attr = gdc_video_set_attr,
	.video_get_attr = gdc_video_get_attr,
	.video_set_ichn_attr = gdc_set_ichn_attr,
	.video_get_ichn_attr = gdc_get_ichn_attr,
	.video_set_ochn_attr = gdc_set_ochn_attr,
	.video_get_ochn_attr = gdc_get_ochn_attr,
	.video_set_attr_ex = gdc_video_set_attr,
	.video_get_attr_ex = gdc_video_get_attr,
	.video_get_buf_attr = gdc_video_reqbufs,
	.video_start = gdc_video_streamon,
	.video_stop = gdc_video_streamoff,
	.video_get_version = gdc_get_version,
	.video_get_struct_size = gdc_get_struct_size,
};

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC rigister char device interface
* @param[in] *gdc: gdc ip device
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
static s32 hobot_gdc_device_node_init(struct hobot_gdc_dev *gdc)
{
	s32 ret = 0;
	u32 i;
	char name[32];
	struct vio_node *vnode;

	vnode = gdc->vnode;
	gdc->gtask.no_worker = 1;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vnode[i].id = GDC_MODULE;
		vnode[i].hw_id = gdc->hw_id;
		vnode[i].flow_id = INVALID_FLOW_ID;
		vnode[i].ctx_id = i;
		vnode[i].ich_subdev[0] = &gdc->subdev[i][0].vdev;
		vnode[i].active_ich = 1;
		vnode[i].och_subdev[0] = &gdc->subdev[i][1].vdev;
		vnode[i].active_och = 1;
		gdc->subdev[i][0].vdev.vnode = &vnode[i];
		gdc->subdev[i][0].gdc = gdc;
		gdc->subdev[i][1].vdev.vnode = &vnode[i];
		gdc->subdev[i][1].vdev.pingpong_ring = 1;
		gdc->subdev[i][1].gdc = gdc;
		vnode[i].gtask = &gdc->gtask;
		vnode[i].allow_bind = gdc_allow_bind;
		vnode[i].frame_work = gdc_frame_work;
	}

	gdc->vps_device[0].vps_ops = &gdc_vops;
	gdc->vps_device[0].ip_dev = gdc;
	gdc->vps_device[0].vid = VNODE_ID_SRC;
	gdc->vps_device[0].vnode = vnode;
	gdc->vps_device[0].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	gdc->vps_device[0].iommu_dev = &gdc->pdev->dev;
	gdc->vps_device[0].owner = THIS_MODULE;
	#endif

	snprintf(name, sizeof(name), "gdc%d_src", gdc->hw_id);
	ret = vio_register_device_node(name, &gdc->vps_device[0]);
	if (ret < 0)
		goto err;

	gdc->vps_device[1].vps_ops = &gdc_vops;
	gdc->vps_device[1].ip_dev = gdc;
	gdc->vps_device[1].vid = VNODE_ID_CAP;
	gdc->vps_device[1].vnode = vnode;
	gdc->vps_device[1].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	gdc->vps_device[1].iommu_dev = &gdc->pdev->dev;
	gdc->vps_device[1].owner = THIS_MODULE;
	#endif

	snprintf(name, sizeof(name), "gdc%d_cap", gdc->hw_id);
	ret = vio_register_device_node(name, &gdc->vps_device[1]);
	if (ret < 0)
		goto err1;

	return ret;
err1:
	vio_unregister_device_node(&gdc->vps_device[0]);
err:
	return ret;
}

#ifdef HOBOT_MCU_CAMSYS

#else

VPF_LOADING_SHOW_MACRO(gdc, struct hobot_gdc_dev, loading);

VPF_LOADING_STORE_MACRO(gdc, struct hobot_gdc_dev, loading);

static DEVICE_ATTR(loading, 0660, gdc_loading_show, gdc_loading_store);/*PRQA S 4501,0636*/


static ssize_t gdc_reg_dump(struct device *dev, struct device_attribute *attr, char* buf)
{
	struct hobot_gdc_dev *gdc;

	gdc = (struct hobot_gdc_dev *)dev_get_drvdata(dev);
	vio_clk_enable("gdc_core");
	vio_clk_enable("vse_core");
	vio_clk_enable("vse_ups");
	vio_clk_enable("gdc_hclk");
	gdc_hw_dump(gdc->base_reg);
	vio_clk_disable("gdc_core");
	vio_clk_disable("vse_core");
	vio_clk_disable("vse_ups");
	vio_clk_disable("gdc_hclk");

	return 0;
}
static DEVICE_ATTR(regdump, 0444, gdc_reg_dump, NULL);/*PRQA S 4501,0636*/

static int gdc_wrapper_init(struct platform_device *pdev)
{
	struct hobot_gdc_dev *gdc;
	struct platform_device *cam_ctrl_pdev;
	struct device_node *dn;
	void *ph;

	dn = of_parse_phandle(pdev->dev.of_node, "cam-ctrl", 0);
	if (!dn)
		return -1;

	cam_ctrl_pdev = of_find_device_by_node(dn);
	of_node_put(dn);
	if (!cam_ctrl_pdev)
		return -1;

	ph = platform_get_drvdata(cam_ctrl_pdev);
	if (!ph) {
		platform_device_put(cam_ctrl_pdev);
		return -1;
	}

	gdc = platform_get_drvdata(pdev);
	if (!gdc) {
		platform_device_put(pdev);
		return -1;
	}
	gdc->wrap = ph;

	return 0;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC platform driver probe interface
* @param[in] *pdev: platform device struct
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
static s32 hobot_gdc_probe(struct platform_device *pdev)
{
	s32 ret;
	struct hobot_gdc_dev *gdc;
	struct resource *mem_res;
	struct device_node *dnode;
	struct device *dev;

	dev = &pdev->dev;
	dnode = dev->of_node;
	gdc = (struct hobot_gdc_dev *)devm_kzalloc(dev, sizeof(struct hobot_gdc_dev), GFP_KERNEL);
	if (gdc == NULL) {
		dev_err(dev, "%s is NULL", __func__);
		ret = -ENOMEM;
		return ret;
	}
	gdc->pdev = pdev;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem_res == NULL) {
		dev_err(dev, "Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		return ret;
	}

	gdc->regs_start = mem_res->start;
	gdc->regs_end = mem_res->end;
	gdc->base_reg = devm_ioremap(&pdev->dev, mem_res->start, resource_size(mem_res));
	if (gdc->base_reg == NULL) {
		dev_err(dev, "Failed to remap io region(%p)", gdc->base_reg);
		ret = -ENOMEM;
		return ret;
	}

	/* Get IRQ SPI number */
	gdc->irq = platform_get_irq(pdev, 0);
	if (gdc->irq < 0) {
		dev_err(dev, "Failed to get gdc_irq(%d)", gdc->irq);
		ret = -EBUSY;
		return ret;
	}

	ret = devm_request_irq(dev, (u32)gdc->irq, gdc_isr, IRQF_TRIGGER_HIGH, "gdc", (void *)gdc);
	if (ret != 0) {
		dev_err(dev, "request_irq(IRQ_GDC %d) is fail(%d)", gdc->irq, ret);
		return ret;
	}

#ifdef CONFIG_OF
	ret = of_property_read_u32(dnode, "id", &gdc->hw_id);
	if (ret != 0) {
		dev_err(dev, "id read is fail(%d)", ret);
	}
#endif
	ret = device_create_file(dev, &dev_attr_loading);
	if (ret < 0) {
		dev_err(dev, "create loading failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		device_remove_file(&pdev->dev, &dev_attr_loading);
		dev_err(dev, "create regdump failed (%d)\n", ret);
		return ret;
	}
	ret = hobot_gdc_device_node_init(gdc);
	if (ret < 0) {
		device_remove_file(&pdev->dev, &dev_attr_loading);
		device_remove_file(dev, &dev_attr_regdump);
		return ret;
	}

	platform_set_drvdata(pdev, (void *)gdc);
	osal_spin_init(&gdc->shared_slock);/*PRQA S 3334*/
	osal_mutex_init(&gdc->mlock);/*PRQA S 3334*/
	g_gdc_dev[gdc->hw_id] = gdc;

	gdc_wrapper_init(pdev);

	dev_info(dev, "[FRT:D] %s(%d)\n", __func__, ret);

	return 0;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC platform driver remove interface
* @param[in] *pdev: platform device struct
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
static s32 hobot_gdc_remove(struct platform_device *pdev)
{
	s32 ret = 0;
	struct hobot_gdc_dev *gdc;
	struct device *dev;

	if (pdev == NULL) {
		vio_err("%s: pdev = NULL\n", __func__);
		return -EFAULT;
	}
	gdc = (struct hobot_gdc_dev *)platform_get_drvdata(pdev);
	dev = &pdev->dev;
	device_remove_file(&pdev->dev, &dev_attr_loading);
	device_remove_file(dev, &dev_attr_regdump);
	vio_unregister_device_node(&gdc->vps_device[0]);
	vio_unregister_device_node(&gdc->vps_device[1]);

	devm_free_irq(dev, (u32)gdc->irq, (void *)gdc);
	devm_kfree(dev, (void *)gdc);

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief gdc's sleep processing function
* @param[in] *dev: device abstraction
* @retval 0: success
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 hobot_gdc_suspend(struct device *dev)
{
	s32 ret = 0;
	struct hobot_gdc_dev *gdc;

	gdc = (struct hobot_gdc_dev *)dev_get_drvdata(dev);
	if (osal_atomic_read(&gdc->open_cnt) > 0) {
		dev_err(dev, "%s open cnt %d\n", __func__, osal_atomic_read(&gdc->open_cnt));
		ret = -EBUSY;
	}

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief Wake-up function of GDC
* @param[in] *dev: device abstraction
* @retval 0: success
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 hobot_gdc_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief gdc's runtime sleep processing function
* @param[in] *dev: device abstraction
* @retval 0: success
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 hobot_gdc_runtime_suspend(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief Wake-up runtime function of GDC
* @param[in] *dev: device abstraction
* @retval 0: success
* @param[out] None
* @data_read None
* @data_updated None
* @compatibility None
* @callgraph
* @callergraph
* @design
*/
static s32 hobot_gdc_runtime_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops hobot_gdc_pm_ops = {
	.suspend = hobot_gdc_suspend,
	.resume = hobot_gdc_resume,
	.runtime_suspend = hobot_gdc_runtime_suspend,
	.runtime_resume = hobot_gdc_runtime_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id hobot_gdc_match[] = {
	{
	 .compatible = "arm,gdc",
	},
	{},
};

MODULE_DEVICE_TABLE(of, hobot_gdc_match);

static struct platform_driver hobot_gdc_driver = {
	.probe = hobot_gdc_probe,
	.remove = hobot_gdc_remove,
	.driver = {
			.name = MODULE_NAME,
			.owner = THIS_MODULE,
			.pm = &hobot_gdc_pm_ops,
			.of_match_table = hobot_gdc_match,
		   }
};

#else
static struct platform_device_id hobot_gdc_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, hobot_gdc_driver_ids);

static struct platform_driver hobot_gdc_driver = {
	.probe = hobot_gdc_probe,
	.remove = __devexit_p(hobot_gdc_remove),
	.id_table = hobot_gdc_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &hobot_gdc_pm_ops,
		   }
};
#endif

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC platform driver register interface
* @param[in] None
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
static s32 __init hobot_gdc_init(void)
{
	s32 ret = platform_driver_register(&hobot_gdc_driver);
	if (ret != 0)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(hobot_gdc_init);/*PRQA S 0605*/

/**
* @NO{S09E03C01}
* @ASIL{B}
* @brief GDC platform driver unregister interface
* @param[in] None
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
static void __exit hobot_gdc_exit(void)
{
	platform_driver_unregister(&hobot_gdc_driver);
}

module_exit(hobot_gdc_exit);/*PRQA S 0605*/

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("Hobot GDC driver");
MODULE_LICENSE("GPL v2");
#endif
