/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt)    "[VIN_NODE]:" fmt

#ifdef HOBOT_MCU_CAMSYS

#else
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#endif

#include "hobot_dev_vin_node.h"
#include "hobot_vin_node_ops.h"
#include "hobot_vpf_manager.h"
#include "vio_node_api.h"

#define MODULE_NAME "VIN_NODE"

struct vin_common_ops vin_ops[MAX_VIN_HW_ID][VIN_MAX_DEVICES];
struct j6_vin_node_dev *g_vin_node_dev[VIN_NODE_MAX_DEVICE];
/* EXPORT_SYMBOL(vin_node_dev); */

vin_get_frame_info_callback vin_node_get_frame_info;

static struct vio_version_info g_vin_node_version = { .major = 1, .minor = 1 };

int32_t vin_node_get_version(struct vio_version_info *version)
{
	int32_t ret = 0;

	vio_info("The vin node driver version is v%d.%d", g_vin_node_version.major, g_vin_node_version.minor);
	memcpy(version, &g_vin_node_version, sizeof(struct vio_version_info));

	return ret;
}

int32_t vin_node_get_struct_size(struct vio_struct_size *vio_size)
{
	int32_t ret = 0;

	switch (vio_size->type)
	{
	case BASE_ATTR:
		vio_size->size = sizeof(vin_node_attr_t);
		break;
	case ICHN_ATTR:
		vio_size->size = sizeof(vin_ichn_attr_t);
		break;
	case OCHN_ATTR:
		vio_size->size = sizeof(vin_ochn_attr_t);
		break;
	case EX_ATTR:
		vio_size->size = sizeof(vin_attr_ex_t);
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
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Calculate the length of each row based on the format
 * @retval 0: success
 * @retval <0: fail
 * @param[in] width: Frame width
 * @param[in] pack_mode: loose or tight
 * @param[in] format: data format
 * @param[out] bytesPerLine
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
u32 vin_get_perline_size(u32 width, u8 pack_mode, u32 format)
{
	u32 bytesPerLine;

	if (width == 0u) {
		vio_err("%s Invalid input width size = 0", __func__);
		return 0;
	}

	if (pack_mode == 0u) {
		switch (format) {
			case HW_FORMAT_RAW20:
				bytesPerLine = width * 4u;
				break;
			case HW_FORMAT_RAW16:
			case HW_FORMAT_RAW14:
			case HW_FORMAT_RAW12:
			case HW_FORMAT_RAW10:
			case HW_FORMAT_YUV422_10BIT:
				bytesPerLine = width * 2u;
				break;
			case HW_FORMAT_RAW8:
			case HW_FORMAT_YUV422_8BIT:
				bytesPerLine = width;
				break;
			default:
				vio_err("Invalid cim pix len (%d)!!!\n", format);
				bytesPerLine = width;
				break;
		}
	} else {
		switch (format) {
			case HW_FORMAT_RAW20:
				bytesPerLine = width * 5u / 2u;
				break;
			case HW_FORMAT_RAW16:
				bytesPerLine = width * 2u;
				break;
			case HW_FORMAT_RAW14:
				bytesPerLine = width * 7u / 4u;
				break;
			case HW_FORMAT_RAW12:
				bytesPerLine = width * 3u / 2u;
				break;
			case HW_FORMAT_RAW10:
			case HW_FORMAT_YUV422_10BIT:
				bytesPerLine = width * 5u / 4u;
				break;
			case HW_FORMAT_RAW8:
			case HW_FORMAT_YUV422_8BIT:
			case HW_FORMAT_YUV420_SHIFT_8BIT:
				bytesPerLine = width;
				break;
			default:
				vio_err("Invalid cim format (%d)!!!\n", format);
				bytesPerLine = width;
				break;
		}
	}

	vio_dbg("cim calc bytesPerLine = %d \n", bytesPerLine);/*PRQA S 0685,1294*/

	return bytesPerLine;
}
EXPORT_SYMBOL(vin_get_perline_size);


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Interface for obtaining frameid and timestamp
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vnode: One vnode along the way
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_get_frame_info_ops(struct vio_node *vnode)
{
	s32 ret = 0;

	// vin_node_dev = (struct j6_vin_node_dev *)vnode->device;  // VPF 加个void*来保存vin_node_dev
	if (vin_node_get_frame_info)
		vin_node_get_frame_info(vnode);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Register the interface for obtaining the frameid and timestamp
 * @retval None
 * @param[in] cim_get_frame_info: the function to get frameid and timestamp
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vin_register_frame_info_func(vin_get_frame_info_callback cim_get_frame_info)
{
	vin_node_get_frame_info = cim_get_frame_info;
}

EXPORT_SYMBOL(vin_register_frame_info_func);

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Configuration function of vin_node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] arg: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	vin_node_attr_t vin_node_attr;

	copy_ret = osal_copy_from_app((void *)&vin_node_attr,
			(void __user *)arg, sizeof(vin_node_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = vin_node_set_attr(vctx, &vin_node_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Get vin_node attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] arg: User configuration parameters
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_get_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	vin_node_attr_t vin_node_attr = { 0 };

	/* to support tx vcon info get */
	copy_ret = osal_copy_from_app((void *)&vin_node_attr,
			(void __user *) arg, sizeof(vin_node_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = vin_node_get_attr(vctx, &vin_node_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}
	vin_node_attr.flow_id = vctx->flow_id;

	copy_ret = osal_copy_to_app((void __user *) arg,
			(void *)&vin_node_attr, sizeof(vin_node_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Configuration function of vin_node extend attr
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] arg: User extend configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	vin_attr_ex_t vin_attr_ex;

	copy_ret = osal_copy_from_app((void *)&vin_attr_ex,
			(void __user *) arg, sizeof(vin_attr_ex_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	ret = vin_node_set_attr_ex(vctx, &vin_attr_ex);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node output channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] arg: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	vin_ochn_attr_t vin_ochn_attr;

	memset(&vin_ochn_attr, 0, sizeof(vin_ochn_attr_t));
	copy_ret = osal_copy_from_app((void *)&vin_ochn_attr,
			(void __user *) arg, sizeof(vin_ochn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = vin_node_set_ochn_attr(vctx, &vin_ochn_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node input channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx; vio_video_ctx
 * @param[in] arg: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	vin_ichn_attr_t vin_ichn_attr;

	memset(&vin_ichn_attr, 0, sizeof(vin_ichn_attr_t));
	copy_ret = osal_copy_from_app((void *)&vin_ichn_attr,
			(void __user *) arg, sizeof(vin_ichn_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = vin_node_set_ichn_attr(vctx, &vin_ichn_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node output channel buffer attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *buf: numbers of buffer
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_ochn_buff_attr(struct vio_video_ctx *vctx, void *buf)
{
	s32 ret;
	vin_ochn_buff_attr_t ochn_buff_attr;

	ret = vin_node_set_ochn_buff_attr(vctx, &ochn_buff_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}
	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

#if 0
s32 vin_node_video_get_frameid(struct vio_node *vnode)
{
	s32 ret;
	struct j6_vin_node_dev *vin_node_dev;
	vin_node_dev = (struct j6_vin_node_dev *)vnode->device;  // VPF 加个void*来保存vin_node_dev

	if (vin_node_dev->vin_ops[VIN_CIM] && vin_node_dev->vin_ops[VIN_CIM]->video_get_frameid) {
		ret = vin_node_dev->vin_ops[VIN_CIM]->video_get_frameid(vnode);
		if (ret < 0)
			return ret;
	}
	return ret;
}
#endif

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node internal_attr
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *mipi_attr: mipi attr
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_video_set_internal_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct j6_vin_node_dev *vin_node_dev;
	vin_inter_attr_t inter_attr;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;  // VPF 加个void*来保存vin_node_devs

	copy_ret = osal_copy_from_app((void *)&inter_attr,
			(void __user *)arg, sizeof(vin_inter_attr_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	ret = vin_node_set_internal_attr(vctx, &inter_attr);
	if (ret < 0) {
		vio_err("flow_id %d %s error", vin_node_dev->flow_id, __func__);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Start vin_node from working
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
static s32 vin_node_video_streamon(struct vio_video_ctx *vctx)
{
	s32 ret;

	if (vctx->id != VNODE_ID_SRC)
		return 0;

	ret = vin_node_start(vctx);
	if (ret < 0) {
		vio_info("[C%d]%s fail\n", vctx->ctx_id, __func__);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: stop vin_node from working
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
static s32 vin_node_video_streamoff(struct vio_video_ctx *vctx)
{
	s32 ret;

	if (vctx->id != VNODE_ID_SRC)
		return 0;

	ret = vin_node_stop(vctx);
	if (ret < 0) {
		vio_info("[C%d]%s fail\n", vctx->ctx_id, __func__);
		return ret;
	}

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: vin_node register interface
 * @retval 0: success
 * @retval <0: fail
 * @param[in] type: type of vins
 * @param[in] *ops: Module registration
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vin_register_device_node(vin_type_e type, struct vin_common_ops *ops)
{
	u8 i;

	if (type >= VIN_MAX_DEVICES || ops == NULL) {
		vio_err("invlide type\n");
		return;
	}

	for (i = 0; i < MAX_VIN_HW_ID; i++)
		memcpy(&vin_ops[i][type], ops, sizeof(struct vin_common_ops));

	vio_info("type %d %s done\n", type, __func__);

	return;
}
EXPORT_SYMBOL(vin_register_device_node);

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: vin_node unregister interface
 * @retval 0: success
 * @retval <0: fail
 * @param[in] type: type of vins
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vin_unregister_device_node(vin_type_e type)
{
	u8 i;

	if (type >= VIN_MAX_DEVICES) {
		vio_err("invlide type\n");
		return;
	}

	for (i = 0; i < MAX_VIN_HW_ID; i++)
		memset(&vin_ops[i][type], 0, sizeof(struct vin_common_ops));

	vio_info("type %d %s done\n", type, __func__);

	return;
}
EXPORT_SYMBOL(vin_unregister_device_node);

void vin_register_device_node_hwid(vin_type_e type, u8 hw_id, struct vin_common_ops *ops)
{
	if (type >= VIN_MAX_DEVICES || ops == NULL || hw_id >= MAX_VIN_HW_ID) {
		vio_err("invlide type\n");
		return;
	}

	memcpy(&vin_ops[hw_id][type], ops, sizeof(struct vin_common_ops));

	vio_info("type %d %s done\n", type, __func__);

	return;
}
EXPORT_SYMBOL(vin_register_device_node_hwid);


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Open the calling function of the vin_node device node
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
static int vin_node_video_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	if (NULL == vctx) {
		vio_err("vctx is NULL\n");
		return -1;
	}

	ret = vin_node_open(vctx);
	if (ret < 0) {
		vio_err("[C%d] open fail\n", vctx->ctx_id);
		return -1;
	}
	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Close the calling function of the vin_node device node
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
static int vin_node_video_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	ret = vin_node_close(vctx);
	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Obtain the attributes of the buffer request
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] group_attr: buff attribute
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vin_node_video_reqbufs(struct vio_video_ctx *vctx,
		struct vbuf_group_info *group_attr)
{
	s32 ret = 0;

	ret = vin_node_reqbufs(vctx, group_attr);
	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: vin_node reset
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
static s32 vin_node_video_reset(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
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
static s32 vin_node_video_error_callback(struct vio_video_ctx *vctx,
		void *error_info)
{
	s32 ret = 0;

	vio_info("[C%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set ctrl function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] cmd: ctrl commod
 * @param[in] arg: arg
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vin_node_video_set_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	return vin_node_set_ctrl(vctx, cmd, arg);
}

struct vio_common_ops vin_node_vops = {
	.open = vin_node_video_open,
	.close = vin_node_video_close,
	.video_set_attr = vin_node_video_set_attr,
	.video_get_attr = vin_node_video_get_attr,
	.video_set_attr_ex = vin_node_video_set_attr_ex,
	.video_get_buf_attr = vin_node_video_reqbufs,
	.video_set_inter_attr = vin_node_video_set_internal_attr,
	.video_set_ichn_attr = vin_node_video_set_ichn_attr,
	.video_set_ochn_attr = vin_node_video_set_ochn_attr,
	.video_set_obuf = vin_node_video_set_ochn_buff_attr,
	.video_start = vin_node_video_streamon,
	.video_stop = vin_node_video_streamoff,
	.video_reset = vin_node_video_reset,
	.video_error_callback = vin_node_video_error_callback,
	.video_get_version = vin_node_get_version,
	.video_s_ctrl = vin_node_video_set_ctrl,
	.video_get_struct_size = vin_node_get_struct_size,
};

s32 vin_device_node_init(u32 hw_id, void *cim)
{
	if (g_vin_node_dev[hw_id] != NULL) {
		g_vin_node_dev[hw_id]->cim_dev = (struct j6_cim_dev *)cim;
	}

	vio_info("hw_id %d %s done\n", hw_id, __func__);
	return 0;
}
EXPORT_SYMBOL(vin_device_node_init);

s32 vin_device_node_deinit(u32 hw_id)
{
	if (g_vin_node_dev[hw_id] != NULL) {
		g_vin_node_dev[hw_id]->cim_dev = NULL;
	}
	vio_info("hw_id %d %s done\n", hw_id, __func__);
	return 0;
}
EXPORT_SYMBOL(vin_device_node_deinit);

s32 vin_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	enum vio_bind_type bind_type;

	if (osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) != 0)
		return CHN_BIND_REP;

	if (online_mode) {
		bind_type = CHN_BIND_OTF;
	} else {
		bind_type = CHN_BIND_M2M;
	}

	vin_node_set_ochn_bind_param(vdev->vctx[0]);

	if (vin_node_bind_check(vdev, remote_vdev, online_mode) != 0)
		bind_type = CHN_BIND_NONE;

	return bind_type;
}

#ifdef HOBOT_MCU_CAMSYS
#else
struct class* vin_node_get_class(void)
{
	if (g_vin_node_dev[0] != NULL)
		return g_vin_node_dev[0]->vps_device[SRC_INDEX].dev.class;

	return NULL;
}
EXPORT_SYMBOL(vin_node_get_class);
#endif

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Vin_node driver node registration function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vin_node_dev: j6 hardware struct
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_device_node_init(u8 hw_id)
{
	u32 i;
	char name[32];
	struct vio_node *vnode;
	struct j6_vin_node_dev *vin_node_dev;

	vin_node_dev = g_vin_node_dev[hw_id];
	if (vin_node_dev == NULL) {
		vio_err("NULL vin_node_dev\n");
		return 0;  // TODO
	}

	// 一路一个vnode,和ctx对应
	vnode = vin_node_dev->vnode;
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vnode[i].id = VIN_MODULE;
		vnode[i].hw_id = vin_node_dev->hw_id;
		vnode[i].ctx_id = i;
		vnode[i].flow_id = INVALID_FLOW_ID;
		vnode[i].ich_subdev[VIN_MAIN_FRAME] = &vin_node_dev->src_subdev[i].vdev; // src
		vnode[i].active_ich = 1;
		vnode[i].och_subdev[VIN_MAIN_FRAME] = &vin_node_dev->cap_subdev[i].vdev; // frame cap
		//vnode[i].och_subdev[1] = &vin_node->subdev[i][2].vdev;   // online
		vnode[i].och_subdev[VIN_EMB] = &vin_node_dev->emb_subdev[i].vdev;    //emb                       // 聚合
		vnode[i].och_subdev[VIN_ROI] = &vin_node_dev->roi_subdev[i].vdev;   // roi
		vnode[i].active_och |= 1 << VIN_MAIN_FRAME;
		vnode[i].active_och |= 1 << VIN_EMB;
		vnode[i].active_och |= 1 << VIN_ROI;
		vin_node_dev->src_subdev[i].vdev.vnode = &vnode[i];
		vin_node_dev->src_subdev[i].vin_node_dev = vin_node_dev;
		vin_node_dev->cap_subdev[i].vdev.vnode = &vnode[i];
		vin_node_dev->cap_subdev[i].vin_node_dev = vin_node_dev;
		vin_node_dev->emb_subdev[i].vdev.vnode = &vnode[i];
		vin_node_dev->emb_subdev[i].vin_node_dev = vin_node_dev;
		vin_node_dev->roi_subdev[i].vdev.vnode = &vnode[i];
		vin_node_dev->roi_subdev[i].vin_node_dev = vin_node_dev;
		vnode[i].gtask = &vin_node_dev->gtask;
		vnode[i].allow_bind = vin_allow_bind;
	}

	vin_node_dev->vps_device[SRC_INDEX].vps_ops = &vin_node_vops;
	vin_node_dev->vps_device[SRC_INDEX].ip_dev = vin_node_dev;
	vin_node_dev->vps_device[SRC_INDEX].vid = VNODE_ID_SRC;    //vctx->id
	vin_node_dev->vps_device[SRC_INDEX].vnode = vnode;
	vin_node_dev->vps_device[SRC_INDEX].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	//vin_node_dev->vps_device[SRC_INDEX].iommu_dev = &vin_node_dev->pdev->dev;
	vin_node_dev->vps_device[SRC_INDEX].iommu_dev = &vin_node_dev->cim_dev->pdev->dev;
	#endif

	snprintf(name, sizeof(name), "vin%d_src", vin_node_dev->hw_id);
	vio_register_device_node(name, &vin_node_dev->vps_device[0]);

	vin_node_dev->vps_device[CAP_INDEX].vps_ops = &vin_node_vops;
	vin_node_dev->vps_device[CAP_INDEX].ip_dev = vin_node_dev;
	vin_node_dev->vps_device[CAP_INDEX].vid = VNODE_ID_CAP;
	vin_node_dev->vps_device[CAP_INDEX].vnode = vnode;
	vin_node_dev->vps_device[CAP_INDEX].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	//vin_node_dev->vps_device[CAP_INDEX].iommu_dev = &vin_node_dev->pdev->dev;
	vin_node_dev->vps_device[CAP_INDEX].iommu_dev = &vin_node_dev->cim_dev->pdev->dev;
	#endif

	snprintf(name, sizeof(name), "vin%d_cap", vin_node_dev->hw_id);
	vio_register_device_node(name, &vin_node_dev->vps_device[1]);

	vin_node_dev->vps_device[EMB_INDEX].vps_ops = &vin_node_vops;
	vin_node_dev->vps_device[EMB_INDEX].ip_dev = vin_node_dev;
	vin_node_dev->vps_device[EMB_INDEX].vid = VNODE_ID_CAP + VIN_EMB;      //公共定义  CIM 4个节点
	vin_node_dev->vps_device[EMB_INDEX].vnode = vnode;
	vin_node_dev->vps_device[EMB_INDEX].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	//vin_node_dev->vps_device[EMB_INDEX].iommu_dev = &vin_node_dev->pdev->dev;
	vin_node_dev->vps_device[EMB_INDEX].iommu_dev = &vin_node_dev->cim_dev->pdev->dev;
	#endif

	snprintf(name, sizeof(name), "vin%d_emb", vin_node_dev->hw_id);
	vio_register_device_node(name, &vin_node_dev->vps_device[3]);

	vin_node_dev->vps_device[ROI_INDEX].vps_ops = &vin_node_vops;
	vin_node_dev->vps_device[ROI_INDEX].ip_dev = vin_node_dev;
	vin_node_dev->vps_device[ROI_INDEX].vid = VNODE_ID_CAP + VIN_ROI;
	vin_node_dev->vps_device[ROI_INDEX].vnode = vnode;
	vin_node_dev->vps_device[ROI_INDEX].max_ctx = VIO_MAX_STREAM;
	#ifndef HOBOT_MCU_CAMSYS
	//vin_node_dev->vps_device[ROI_INDEX].iommu_dev = &vin_node_dev->pdev->dev;
	vin_node_dev->vps_device[ROI_INDEX].iommu_dev = &vin_node_dev->cim_dev->pdev->dev;
	#endif
	snprintf(name, sizeof(name), "vin%d_roi", vin_node_dev->hw_id);
	vio_register_device_node(name, &vin_node_dev->vps_device[4]);

	vio_info("[S%d]%s hw_id %d done\n", vin_node_dev->flow_id, __func__, vin_node_dev->hw_id);
	return 0;
}
EXPORT_SYMBOL(vin_node_device_node_init);

void vin_node_device_node_deinit(u8 hw_id)
{
	struct j6_vin_node_dev *vin_node_dev;
	vin_node_dev = g_vin_node_dev[hw_id];
	if (vin_node_dev == NULL) {
		vio_err("NULL vin_node_dev unregister fail\n");
		return;
	}
	vio_unregister_device_node(&vin_node_dev->vps_device[SRC_INDEX]);
	memset(&vin_node_dev->vps_device[SRC_INDEX], 0, sizeof(struct vpf_device));
	vio_unregister_device_node(&vin_node_dev->vps_device[CAP_INDEX]);
	memset(&vin_node_dev->vps_device[CAP_INDEX], 0, sizeof(struct vpf_device));
	vio_unregister_device_node(&vin_node_dev->vps_device[EMB_INDEX]);
	memset(&vin_node_dev->vps_device[EMB_INDEX], 0, sizeof(struct vpf_device));
	vio_unregister_device_node(&vin_node_dev->vps_device[ROI_INDEX]);
	memset(&vin_node_dev->vps_device[ROI_INDEX], 0, sizeof(struct vpf_device));

	vio_info("hw_id %d %s done\n", hw_id, __func__);
}
EXPORT_SYMBOL(vin_node_device_node_deinit);
/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Vin_node driver detection registration function
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
#ifdef HOBOT_MCU_CAMSYS

#else
static s32 hobot_vin_node_probe(struct platform_device *pdev)
{
	s32 ret;
	s32 i;
	struct j6_vin_node_dev *vin_node;
	struct device *dev;
#ifdef CONFIG_OF
	struct device_node *dnode;
#endif

	dev = &pdev->dev;
	vin_node = (struct j6_vin_node_dev *)devm_kzalloc(dev, sizeof(struct j6_vin_node_dev), GFP_KERNEL);
	if (vin_node == NULL) {
		dev_err(dev, "vin_node is NULL");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	dnode = dev->of_node;
	ret = of_property_read_u32(dnode, "id", &vin_node->hw_id);
	if (ret < 0)
		dev_err(dev, "id read is fail(%d)", ret);
#endif
	//ret = vin_node_device_node_init(vin_node);
	if (ret < 0) {
		vio_err("vin_node_device_node_init fail\n");
		return ret;
	}
	platform_set_drvdata(pdev, (void *)vin_node);
	osal_mutex_init(&vin_node->mlock);  /*PRQA S 3334*/
	osal_atomic_set(&vin_node->open_cnt, 0);

	for (i = 0; i < VIN_MAX_DEVICES; i++) {
		vin_node->vin_ops[i] = &vin_ops[vin_node->hw_id][i];
	}
	g_vin_node_dev[vin_node->hw_id] = vin_node;
	vio_info("%s hw_id %d done\n", __func__, vin_node->hw_id);
	dev_info(dev, "[FRT:D] %s(%d)\n", __func__, ret);
	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Vin_node driver removal function
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
static s32 hobot_vin_node_remove(struct platform_device *pdev)
{
	s32 ret = 0;
	struct j6_vin_node_dev *vin_node;
	struct device *dev;

	if (pdev == NULL) {
		vio_err("%s: pdev = NULL\n", __func__);
		return -EFAULT;
	}

	dev = &pdev->dev;
	vin_node = (struct j6_vin_node_dev *)platform_get_drvdata(pdev);

	devm_kfree(dev, (void *)vin_node);
	dev_info(dev, "%s\n", __func__);
	return ret;
}


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief vin_node sleep processing function
 * @retval 0: success
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_vin_node_suspend(struct device *dev)
{
	vio_dbg("%s %d\n", __func__, __LINE__);
	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief Wake-up function of VIN_NODE
 * @retval 0: success
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_vin_node_resume(struct device *dev)
{
	vio_dbg("%s %d\n", __func__, __LINE__);
	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief vin_node's runtime sleep processing function
 * @retval 0: success
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_vin_node_runtime_suspend(struct device *dev)
{
	vio_dbg("%s %d\n", __func__, __LINE__);
	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief Wake-up runtime function of VIN_NODE
 * @retval 0: success
 * @param[in] *dev: device abstraction
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_vin_node_runtime_resume(struct device *dev)
{
	vio_dbg("%s %d\n", __func__, __LINE__);
	return 0;
}


static const struct of_device_id hobot_vin_node_match[] = {
	{
		.compatible = "hobot,j6-vin_node",
	},
	{},
};

static const struct dev_pm_ops hobot_vin_node_pm_ops = {
	.suspend = hobot_vin_node_suspend,
	.resume = hobot_vin_node_resume,
	.runtime_suspend = hobot_vin_node_runtime_suspend,
	.runtime_resume = hobot_vin_node_runtime_resume,
};

MODULE_DEVICE_TABLE(of, hobot_vin_node_match);

static struct platform_driver hobot_vin_node_driver = {
	.probe = hobot_vin_node_probe,
	.remove = hobot_vin_node_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &hobot_vin_node_pm_ops,
		.of_match_table = hobot_vin_node_match,
	}
};


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief VIN_NODE driver initialization function
 * @retval 0: success
 * @retval <0: fail
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_vin_node_init(void)
{
	int32_t ret = platform_driver_register(&hobot_vin_node_driver);
	if (ret != 0)
		vio_err("J6 vin_node register failed: %d\n", ret);

	return ret;
}

late_initcall(hobot_vin_node_init);  /*PRQA S 0605*/

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief VIN_NODE driver exit function
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_vin_node_exit(void)
{
	platform_driver_unregister(&hobot_vin_node_driver);
}

module_exit(hobot_vin_node_exit); /*PRQA S 0605*/
MODULE_AUTHOR("Wang Fenfen<fenfen.wang@horizon.com>");
MODULE_DESCRIPTION("J6 VIN_NODE driver");
MODULE_LICENSE("GPL");
#endif

