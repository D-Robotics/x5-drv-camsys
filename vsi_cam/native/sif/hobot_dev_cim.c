/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright TIMEOUT_CNT19 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt) "[CIM]:" fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/timekeeping.h>

#include "cim_hw_api.h"
#include "hobot_cim_ops.h"
/* #include "hobot_cim_stl.h" */
/* #include "hobot_dev_vin_node.h" */
#include "hobot_dev_cim.h"

#define SIF_DT_NAME "verisilicon,sif"

/**
 * @def cim_dev
 * Global Structure
 * @NO{S10E04C01}
 */
struct j6_cim_dev *g_cim_dev[VIN_NODE_MAX_DEVICE];
struct vio_callback_ops *vio_cops;

/* extern struct j6_vin_node_dev *vin_node_dev[VIN_NODE_MAX_DEVICE]; */

/**
 * @def MODULE_NAME
 * driver name
 * @NO{S10E04C01}/
 */
#define MODULE_NAME "hobot-sif"
/**
 * Purpose: cim test pattern fps
 * Value: 1~30
 * Range: hobot_dev_cim.c
 * Attention: NA
 */
int tpn_fps = 30;
module_param(tpn_fps, int, 0644); /*PRQA S 0605,0636,4501*/

/**
 * Purpose: cim reset flag
 * Value: 0:disable reset when exiting; 1:enable reset when exiting.
 * Range: hobot_dev_cim.c
 * Attention: NA
 */
int rst_en = 1;
module_param(rst_en, int, 0644); /*PRQA S 0605,0636,4501*/
EXPORT_SYMBOL(rst_en);

/**
 * Purpose: fusa skip frame number
 * Value: 0~N:skip N frames.
 * Range: hobot_dev_cim.c
 * Attention: NA
 */
int fusa_skip_num = 4;
module_param(fusa_skip_num, int, 0644); /*PRQA S 0605,0636,4501*/
EXPORT_SYMBOL(fusa_skip_num);

/**
 * Purpose: config cim blank
 * Value: 0~N
 * Range: cim driver
 * Attention: NA
 */
uint32_t g_cim_tp_blank = 0;

struct j6_cim_dev *cim_get_dev(u8 hw_id)
{
	return g_cim_dev[hw_id];
}
EXPORT_SYMBOL(cim_get_dev);

void cim_set_dev(struct j6_cim_dev *cim_dev, u8 hw_id)
{
	g_cim_dev[hw_id] = cim_dev;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}:
 * @brief: Start cim from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_streamon(struct vio_video_ctx *vctx)
{
	s32 ret;

	ret = cim_subdev_start(vctx, (u32)tpn_fps);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}

	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Stop cim from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_streamoff(struct vio_video_ctx *vctx)
{
	s32 ret;

	ret = cim_subdev_stop(vctx);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Open the calling function of the cim device node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	ret = cim_open(vctx);

	vio_info("[S%d]%s done ret val:%d", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Close the calling function of the cims device node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	ret = cim_close(vctx, (u32)rst_en);
	vio_info("[S%d]%s done ret:%d \n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Configuration function of cim
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_attr(struct vio_video_ctx *vctx, void *attr)
{
	s32 ret = 0;
	cim_attr_t *cim_attr;

	cim_attr = (cim_attr_t *)attr;
	ret = cim_subdev_init(vctx, cim_attr);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[C%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Configuration function of cim
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *attr_ex: User configuration extend parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_attr_ex(struct vio_video_ctx *vctx, void *attr_ex)
{
	s32 ret = 0;
	vin_attr_ex_t *vin_attr_ex;

	vin_attr_ex = (vin_attr_ex_t *)attr_ex;
	ret = cim_subdev_init_ex(vctx, vin_attr_ex);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set cim input channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ichn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_ichn_attr(struct vio_video_ctx *vctx, void *ichn_attr)
{
	s32 ret = 0;
	vin_ichn_attr_t *cim_ichn_attr;

	cim_ichn_attr = (vin_ichn_attr_t *)ichn_attr;
	ret = cim_set_ichn_attr(vctx, cim_ichn_attr);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Set cim output channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_ochn_attr(struct vio_video_ctx *vctx, void *ochn_attr)
{
	s32 ret = 0;
	vin_ochn_attr_t *cim_ochn_attr;

	cim_ochn_attr = (vin_ochn_attr_t *)ochn_attr;
	ret = cim_set_ochn_attr(vctx, cim_ochn_attr);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief:  Set cim output channel buffer attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_buff_attr: buffer attributes
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_ochn_buff_attr(struct vio_video_ctx *vctx,
				 void *ochn_buff_attr)
{
	s32 ret = 0;
	vin_ochn_buff_attr_t *cim_ochn_buff_attr;

	cim_ochn_buff_attr = (vin_ochn_buff_attr_t *)ochn_buff_attr;
	ret = cim_set_ochn_buff_attr(vctx, cim_ochn_buff_attr);
	if (ret < 0) {
		vio_err("[S%d][V%d]%s error\n", vctx->vdev->vnode->flow_id,
			vctx->ctx_id, __func__);
	}
	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Get Attributes of cim
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *cim_attr: cim extend parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_get_attr_ex(struct vio_video_ctx *vctx, void *cim_attr)
{
	s32 ret = 0;

	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief:  Get cim attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *cim_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_get_attr(struct vio_video_ctx *vctx, void *cim_attr)
{
	s32 ret = 0;
	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cim reset
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_reset(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	vio_info("[S%d]%s done ret val:%d\n", vctx->ctx_id, __func__, ret);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Error callback function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *error_info: error information
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_error_callback(struct vio_video_ctx *vctx, void *error_info)
{
	s32 ret = 0;
	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

#if 0
/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Error callback function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *error_info: error information
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_video_set_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	s32 ret = 0;
	struct vio_subdev *vdev;
	u32 pause_enable;

	vdev = vctx->vdev;
	if (vdev->id != VNODE_ID_SRC)
		return 0;

	switch (cmd) {
		case CIM_PAUSE_CTRL:
			pause_enable = arg;
			ret = cim_pause_ctrl(vctx, pause_enable);
			break;
		default:
			vio_err("unsupported ctrl\n");
			break;
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}
#endif
/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Update rgbir parameters
 * @retval 0: success
 * @retval <0: fail
 * @param[in] flow_id: One sign along the way
 * @param[in] module_type:cim_isp modules
 * @param[in] *param: parameter information
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 cim_isp_video_update_calib_param(uint32_t flow_id, uint32_t module_type,
				     void *param)
{
	s32 ret = 0;

	// ret = cim_isp_update_calib_param(flow_id, module_type, param);
	// if (ret < 0) {
	// 	vio_err("[S%d]module_type %d %s error\n", flow_id, module_type,__func__);
	// }
	vio_info("[S%d]%s done\n", flow_id, __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Get frameid from cim
 * @retval None
 * @param[in] *vnode: Abstraction along the way
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void cim_video_get_frameid(struct vio_node *vnode, struct frame_id_desc *frameid)
{
	if (vnode != NULL) {
		// use the last frame in the driver
		memcpy(frameid, &vnode->frameid, sizeof(struct frame_id_desc));
	}
}

struct cim_interface_ops cim_cops = {
	.get_frame_id = cim_video_get_frameid,
	// .cim_get_lpwm_timestamps = cim_get_lpwm_timestamps,
};

DECLARE_VIO_CALLBACK_OPS(cim_interface, 0, &cim_cops);

#ifdef CIM_ISP_COPS
static void empty_ipi_lost_fe_state(struct vio_node *vnode)
{
	vio_info("empty_ipi_lost_fe_state\n");
}

static void empty_cim_trans_raw_frameid(struct vio_node *vnode)
{
	vio_info("empty_cim_trans_raw_frameid\n");
}

struct isp_interface_ops g_cim_isp_cops = {
	.ipi_lost_fe_state = empty_ipi_lost_fe_state,
	.cim_trans_raw_frameid = empty_cim_trans_raw_frameid,

};
#endif

#ifdef CIM_SENSOR_COPS
static int32_t empty_common_get_param(uint32_t chn,
				      struct _setting_param_t *user_para)
{
	vio_err("%s\n", __func__);
	return 0;
}

static void empty_sensor_frame_event_2a(int32_t flow_id,
					enum _sensor_frame_event_e event)
{
	vio_err("%s\n", __func__);
	return;
}

static int32_t empty_sensor_get_ts_compen(int32_t flow_id)
{
	vio_err("%s\n", __func__);
	return 0;
}

struct sensor_cim_ops_s g_cim_sensor_cops = {
	.sensor_get_para = empty_common_get_param,
	.sensor_frame_event = empty_sensor_frame_event_2a,
	.sensor_get_ts_compensate = empty_sensor_get_ts_compen,
};
#endif

struct vin_common_ops cim_vops = {
	.open = cim_video_open,
	.close = cim_video_close,
	.video_set_attr = cim_video_set_attr,
	.video_get_attr = cim_video_get_attr,
	.video_set_attr_ex = cim_video_set_attr_ex,
	.video_get_attr_ex = cim_video_get_attr_ex,
	.video_set_ichn_attr = cim_video_set_ichn_attr,
	.video_set_ochn_attr = cim_video_set_ochn_attr,
	.video_start = cim_video_streamon,
	.video_stop = cim_video_streamoff,
	.video_reset = cim_video_reset,
	.video_error_callback = cim_video_error_callback,
	// .video_s_ctrl = cim_video_set_ctrl,
};

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief cim driver node registration function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *cim
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 j6_cim_device_node_init(void)
{
	s32 ret = 0;

	vin_register_device_node(VIN_CIM, &cim_vops);
	//sensor_iq_register_calib_update_ops(cim_isp_video_update_calib_param);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: dump cim regs
 * @param[in] *dev
 * @param[in] *attr
 * @param[in] buf
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t cim_reg_dump(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct j6_cim_dev *cim;

	cim = (struct j6_cim_dev *)dev_get_drvdata(dev);

	cim_set_clk_enable(1);
	// cim_hw_dump(cim->base_reg);
	cim_set_clk_enable(0);

	return 0;
}
static DEVICE_ATTR(regdump, 0444, cim_reg_dump, NULL); /*PRQA S 4501,0636*/

#ifdef CIM_DEBUG
static ssize_t cim_simu_irq(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct j6_cim_dev *cim;

	cim = (struct j6_cim_dev *)dev_get_drvdata(dev);

	cim_set_clk_enable(1);

	writel(0x1, cim->base_reg + IPI0_REG_OFFSET + 0x1c); // fs
	cim_isr(0, cim);
	writel(0x0, cim->base_reg + IPI0_REG_OFFSET + 0x1c);

	writel(0x1, cim->base_reg + IPI0_REG_OFFSET + 0x20); // fe
	cim_isr(0, cim);
	writel(0x0, cim->base_reg + IPI0_REG_OFFSET + 0x20);

	writel(0x1, cim->base_reg + 0x20); // wch0 done
	cim_isr(0, cim);
	writel(0x0, cim->base_reg + 0x20);

	cim_set_clk_enable(0);
	return 0;
}
static DEVICE_ATTR(simu_irq, 0444, cim_simu_irq, NULL); /*PRQA S 4501,0636*/
#endif

// static ssize_t cim_tp_blank_show(struct device *dev,
// 		struct device_attribute *attr, char *buf)
// {
// 	int32_t len;

// 	len = snprintf(buf, "cim tp blank 0x%x\n", g_cim_tp_blank);

// 	return len;
// }

// static ssize_t cim_tp_blank_store(struct device *dev,
// 		struct device_attribute *attr, const char* buf, size_t count)
// {
// 	int ret;

// 	ret = kstrtou32(buf, 16, &g_cim_tp_blank);
// 	if (ret)
// 		return ret;
// 	vio_info("set cim tp blank 0x%x\n", g_cim_tp_blank);

// 	return count;
// }
// static DEVICE_ATTR(tp_blank, (S_IWUSR | S_IRUGO), cim_tp_blank_show, cim_tp_blank_store);/*PRQA S 4501,0636*/

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: Debug interface that show cim pipeline information and buffer information;
 * @param[in] *dev: point to struct device instance;
 * @param[in] *attr: point to struct device_attribute instance;
 * @retval {*}
 * @param[out] *buf: store information string;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static ssize_t cim_stat_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	ssize_t size, len = 0;
	struct j6_cim_dev *cim;
	struct j6_vin_node_dev *vin_node_dev;
	vin_attr_t *vin_attr;
	cim_attr_t *cim_attr;
	vin_ichn_attr_t *vin_ichn_attr;
	vin_ochn_attr_t *main_ochn_attr;
	vin_ochn_attr_t *emb_ochn_attr;
	struct vin_node_subdev *src_subdev;
	vin_cim_private_t *cim_priv_attr;
	u8 i;

	cim = (struct j6_cim_dev *)dev_get_drvdata(dev);

	vin_node_dev = (struct j6_vin_node_dev *)(cim->vin_node_dev);
	if (!vin_node_dev)
		return 0;

	size = PAGE_SIZE;
	for (i = 0; i < CIM_IPI_MAX_NUM; i++) {
		src_subdev = &vin_node_dev->src_subdev[i];
		vin_attr = &src_subdev->vin_attr;
		cim_attr = &vin_attr->vin_node_attr.cim_attr;
		cim_priv_attr = &src_subdev->cim_private_attr;
		vin_ichn_attr = &vin_attr->vin_ichn_attr;
		main_ochn_attr = &vin_attr->vin_ochn_attr[VIN_MAIN_FRAME];
		emb_ochn_attr = &vin_attr->vin_ochn_attr[VIN_EMB];
		if (!vin_ichn_attr->format)
			continue;

		len += snprintf(&buf[len], size - len,
			       "------------------- flow%d info -------------------\n",
			       src_subdev->vdev.vnode->flow_id);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "rx_index:%d\nvc_index:%d\nipi_channels:%d\n",
			       cim_attr->mipi_rx, cim_attr->vc_index,
			       cim_attr->ipi_channel);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "width:%d\nheight:%d\nformat:0x%x\n",
			       vin_ichn_attr->width, vin_ichn_attr->height,
			       vin_ichn_attr->format);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "online_isp:%d\n", cim_attr->cim_isp_flyby);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "ddr_en:%d\nbufnum:%d\n", main_ochn_attr->ddr_en,
			       vin_attr->vin_ochn_buff_attr[VIN_MAIN_FRAME].buffers_num);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "emb_en:%d\nembeded_dependence:%d\nembeded_width:%d\nembeded_height:%d\n",
			       emb_ochn_attr->emb_en,
			       emb_ochn_attr->emb_attr.embeded_dependence,
			       emb_ochn_attr->emb_attr.embeded_width,
			       emb_ochn_attr->emb_attr.embeded_height);
		if (size - len <= 0)
			break;

		len += snprintf(&buf[len], size - len,
			       "size_err_cnt:%d\n", cim->sif.insts[i].size_err_cnt);
		if (size - len <= 0)
			break;
	}

	return len;
}
static DEVICE_ATTR(cim_stat, 0444, cim_stat_show, NULL); /*PRQA S 4501,0636*/

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cimdriver detection registration function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *pdev: platform_device
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_probe(struct platform_device *pdev)
{
	s32 ret;
	struct j6_cim_dev *cim;
	struct resource *mem_res;
	struct device *dev;
#ifdef CONFIG_OF
	struct device_node *dnode;
#endif
	ktime_t start, end;

	start = ktime_get_boottime();
	dev = &pdev->dev;
	cim = (struct j6_cim_dev *)devm_kzalloc(
		&pdev->dev, sizeof(struct j6_cim_dev), GFP_KERNEL);
	if (cim == NULL) {
		dev_err(dev, "cim is NULL");
		return -ENOMEM;
	}
	ret = sif_probe(pdev, &cim->sif);
	if (ret < 0) {
		dev_err(dev, "sif probe failed\n");
		return -EINVAL;
	}
	cim->pdev = pdev;
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem_res == NULL) {
		dev_err(dev, "Failed to get io memory region(%p)", mem_res);
		return -EBUSY;
	}

	cim->regs_start = mem_res->start;
	cim->regs_end = mem_res->end;
	cim->base_reg = devm_ioremap(&pdev->dev, mem_res->start,
				     resource_size(mem_res));
	if (cim->base_reg == NULL) {
		dev_err(dev, "Failed to remap io region(%p)", cim->base_reg);
		return -ENOMEM;
	}

#ifdef CIM_DEBUG
	cim->base_reg = devm_kzalloc(&pdev->dev, 0x20000, GFP_KERNEL);
	dev_err(dev, "fake reg addr %p\n", cim->base_reg);
#endif

#ifdef CONFIG_OF
	dnode = dev->of_node;
	ret = of_property_read_u32(dnode, "id", &cim->hw_id);
	if (ret < 0)
		dev_err(dev, "id read is fail(%d)", ret);
#endif
	/* ret = diagnose_report_startup_status((u16)ModuleDiag_cim + (u16)cim->hw_id, (u8)MODULE_STARTUP_BEGIN); */
	/* if (ret != 0) */
	/* dev_warn(dev, "diagnose_report_startup_status fail! ret=%d\n", ret); */

	ret = vin_device_node_init(cim->hw_id, cim);
	if (ret < 0) {
		vio_err("vin_device_node_init fail\n");
	}

	ret = vin_node_device_node_init(cim->hw_id);
	// ret = vin_node_device_node_init(cim->hw_id);
	if (ret < 0) {
		vio_err("vin_node_device_node_init fail\n");
		return ret;
	}
	ret = device_create_file(dev, &dev_attr_regdump);
	if (ret < 0) {
		dev_err(dev, "create regdump failed (%d)\n", ret);
		return ret;
	}

#ifdef CIM_DEBUG
	ret = device_create_file(dev, &dev_attr_simu_irq);
	if (ret < 0) {
		dev_err(dev, "create simu_irq failed (%d)\n", ret);
		return ret;
	}
#endif

	ret = device_create_file(dev, &dev_attr_cim_stat);
	if (ret < 0) {
		dev_err(dev, "create cim_stat failed (%d)\n", ret);
		return ret;
	}

#ifdef CIM_ISP_COPS
	cim->isp_cops =
		vio_get_callback_ops(&g_cim_isp_cops, ISP_MODULE, COPS_0);
#endif
#ifdef CIM_SENSOR_COPS
	cim->sensor_cops =
		vio_get_callback_ops(&g_cim_sensor_cops, VIN_MODULE, COPS_5);
#endif
	vio_register_callback_ops(&cb_cim_interface, VIN_MODULE, COPS_0);

	platform_set_drvdata(pdev, (void *)cim);
	osal_spin_init(&cim->slock);
	osal_mutex_init(&cim->mlock); /*PRQA S 3334*/
	osal_atomic_set(&cim->rsccount, 0);
	osal_atomic_set(&cim->open_cnt, 0);

#ifdef CONFIG_HOBOT_VIO_STL
	ret = of_property_read_u32(dnode, "stl_enable", &cim->stl.stl_enable);
	if (ret != 0)
		dev_err(dev, "Failed to get stl_enable.\n");

	cim_fusa_setup_ops(cim);
	if (cim->stl.stl_ops->stl_init != NULL)
		cim->stl.stl_ops->stl_init((void *)cim);
#endif

	// cim_set_irq_thread_priority(cim);
	dev_info(dev, "[FRT:D] %s(%d)\n", __func__, ret);
	/*  ret = diagnose_report_startup_status((u16)ModuleDiag_cim + (u16)cim->hw_id, (u8)MODULE_STARTUP_END); */
	/* if (ret != 0) */
	/* dev_warn(dev, "diagnose_report_startup_status fail! ret=%d\n", ret); */
	g_cim_dev[cim->hw_id] = cim;

	end = ktime_get_boottime();
	vio_info("%s hw_id %d done, time used: %lldus\n", __func__, cim->hw_id, ktime_to_us(ktime_sub(end, start)));
	return 0;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief: cim driver removal function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *pdev: platform_device
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_remove(struct platform_device *pdev)
{
	s32 ret = 0;
	struct j6_cim_dev *cim;
	struct device *dev;

	if (pdev == NULL) {
		vio_err("%s failed\n", __func__);
		return -EFAULT;
	}

	dev = &pdev->dev;
	cim = (struct j6_cim_dev *)platform_get_drvdata(pdev);
#ifdef CONFIG_HOBOT_VIO_STL
	if (cim->stl.stl_ops != NULL && cim->stl.stl_ops->stl_deinit != NULL)
		cim->stl.stl_ops->stl_deinit((void *)cim);
#endif

	ret = sif_remove(pdev, &cim->sif);
	if (ret < 0) {
		vio_err("%s sif remove failed\n", __func__);
		return -EFAULT;
	}

	device_remove_file(dev, &dev_attr_regdump);
	device_remove_file(dev, &dev_attr_cim_stat);
	// devm_free_irq(dev, (u32)cim->irq, cim);
	devm_kfree(dev, (void *)cim);
	vio_unregister_callback_ops(VIN_MODULE, COPS_0);
	vio_unregister_callback_ops(VIN_MODULE, COPS_1);
	// vio_unregister_callback_ops(VIN_MODULE, COPS_8);
	// vio_unregister_callback_ops(VIN_MODULE, COPS_9);
	vin_node_device_node_deinit(cim->hw_id);
	dev_info(dev, "%s\n", __func__);
	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief cim sleep processing function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_suspend(struct device *dev)
{
	s32 ret = 0;
	struct j6_cim_dev *cim;

	cim = (struct j6_cim_dev *)dev_get_drvdata(dev);
	if (osal_atomic_read(&cim->open_cnt) > 0) {
		dev_err(dev, "%s open cnt %d\n", __func__,
			osal_atomic_read(&cim->open_cnt));
		ret = -EBUSY;
	}

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief Wake-up function of cim
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief cim's runtime sleep processing function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_runtime_suspend(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

/**
 * @NO{S10E04C01}
 * @ASIL{B}
 * @brief Wake-up runtime function of cim
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 cim_runtime_resume(struct device *dev)
{
	s32 ret = 0;

	dev_info(dev, "%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops j6_cim_pm_ops = {
	.suspend = cim_suspend,
	.resume = cim_resume,
	.runtime_suspend = cim_runtime_suspend,
	.runtime_resume = cim_runtime_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id j6_cim_match[] = {
	{
		.compatible = SIF_DT_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, j6_cim_match);

static struct platform_driver
	j6_cim_driver = { .probe = cim_probe,
			  .remove = cim_remove,
			  .driver = {
				  .name = MODULE_NAME,
				  .owner = THIS_MODULE,
				  .pm = &j6_cim_pm_ops,
				  .of_match_table = j6_cim_match,
#if defined(CONFIG_FAULT_INJECTION_ATTR) && defined(CONFIG_HOBOT_CAMSYS_STL)
				  .fault_injection_store =
					  j6_cim_fault_injection_store,
				  .fault_injection_show =
					  j6_cim_fault_injection_show,
#endif
			  } };

#else
static struct platform_device_id j6_cim_driver_ids[] = {
	{
		.name = MODULE_NAME,
		.driver_data = 0,
	},
	{},
};

MODULE_DEVICE_TABLE(platform, j6_cim_driver_ids);

static struct platform_driver j6_cim_driver = { .probe = cim_probe,
						.remove = __devexit_p(
							cim_remove),
						.id_table = j6_cim_driver_ids,
						.driver = {
							.name = MODULE_NAME,
							.owner = THIS_MODULE,
							.pm = &j6_cim_pm_ops,
						} };
#endif

static s32 __init j6_cim_init(void)
{
	s32 ret = platform_driver_register(&j6_cim_driver);
	if (ret != 0)
		vio_err("platform_driver_register failed: %d\n", ret);

	ret = j6_cim_device_node_init();
	if (ret < 0) {
		vio_err("j6_cim_device_node_init fail\n");
	}
	return ret;
}

late_initcall(j6_cim_init); /*PRQA S 0605*/

static void __exit j6_cim_exit(void)
{
	platform_driver_unregister(&j6_cim_driver);
}

module_exit(j6_cim_exit); /*PRQA S 0605*/
MODULE_AUTHOR("Wang Fenfen<fenfen.wang@horizon.com>");
MODULE_DESCRIPTION("j6 CIM DMA driver");
MODULE_LICENSE("GPL");
