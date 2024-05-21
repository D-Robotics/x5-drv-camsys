/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt)    "[CODEC_NODE]:" fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/platform_device.h>

#include "hobot_dev_codec_node.h"
#include "hobot_codec_node_ops.h"
#include "vio_node_api.h"

#define CODEC_MODULE_NAME "CODEC_NODE"

#define CODEC_CMD_BIND_VCTX 	0
#define CODEC_CMD_DQBUF     	1
#define CODEC_CMD_QBUF      	2
#define CODEC_CMD_GET_CFG   	3
#define CODEC_CMD_QUERYBUF  	4

static struct vio_version_info g_codec_version = {
	.major = 1,
	.minor = 0
};

s32 codec_node_get_version(struct vio_version_info *version)
{
	s32 ret = 0;

	memcpy(version, &g_codec_version, sizeof(struct vio_version_info));

	return ret;
}

int32_t codec_node_get_struct_size(struct vio_struct_size *vio_size)
{
	int32_t ret = 0;

	switch (vio_size->type)
	{
	case BASE_ATTR:
		vio_size->size = sizeof(codec_node_attr_t);
		break;
	case ICHN_ATTR:
		vio_size->size = sizeof(codec_ichn_attr_t);
		break;
	case OCHN_ATTR:
		vio_size->size = sizeof(codec_ochn_attr_t);
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

s32 codec_node_video_set_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct codec_node_attr_s codec_attr;

	copy_ret = osal_copy_from_app((void *)&codec_attr, (void __user *) arg, sizeof(struct codec_node_attr_s));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	ret = codec_node_set_attr(vctx, &codec_attr);
	if (ret < 0) {
		vio_err("%s: codec node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[S%d][V%d]%s done\n", vctx->ctx_id, vctx->id, __func__);

	return ret;
}

s32 codec_node_video_get_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct codec_node_attr_s codec_node_attr = { 0 };

	ret = codec_node_get_attr(vctx, &codec_node_attr);
	if (ret < 0) {
		vio_err("%s: vin node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	copy_ret = osal_copy_to_app((void __user *) arg,
			(void *)&codec_node_attr, sizeof(struct codec_node_attr_s));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[S%d][V%d]%s done\n", vctx->ctx_id, vctx->id, __func__);

	return ret;
}

s32 codec_node_video_set_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct codec_attr_ex_s codec_attr_ex;

	copy_ret = osal_copy_from_app((void *)&codec_attr_ex, (void __user *) arg, sizeof(struct codec_attr_ex_s));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	ret = codec_node_set_attr_ex(vctx, &codec_attr_ex);
	if (ret < 0) {
		vio_err("%s: codec node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set codec_node output channel attributes
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
s32 codec_node_video_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct codec_ochn_attr_s codec_ochn_attr;

	memset(&codec_ochn_attr, 0, sizeof(struct codec_ochn_attr_s));
	copy_ret = osal_copy_from_app((void *)&codec_ochn_attr, (void __user *) arg, sizeof(struct codec_ochn_attr_s));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	ret = codec_node_set_ochn_attr(vctx, &codec_ochn_attr);
	if (ret < 0) {
		vio_err("%s: codec node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[S%d][V%d]%s done\n", vctx->ctx_id, vctx->id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set codec_node input channel attributes
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
s32 codec_node_video_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct codec_ichn_attr_s codec_ichn_attr;

	memset(&codec_ichn_attr, 0, sizeof(struct codec_ichn_attr_s));
	copy_ret = osal_copy_from_app((void *)&codec_ichn_attr, (void __user *) arg, sizeof(struct codec_ichn_attr_s));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}
	ret = codec_node_set_ichn_attr(vctx, &codec_ichn_attr);
	if (ret < 0) {
		vio_err("%s: codec node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[S%d][V%d]%s done\n", vctx->ctx_id, vctx->id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set codec_node output channel buffer attributes
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
s32 codec_node_video_set_ochn_buff_attr(struct vio_video_ctx *vctx, void *buf)
{
	s32 ret;
	struct codec_ochn_buff_attr_s ochn_buff_attr;

	ret = codec_node_set_ochn_buff_attr(vctx, &ochn_buff_attr);
	if (ret < 0) {
		vio_err("%s: codec node init error, ret = %d\n", __func__, ret);
		return ret;
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Start codec_node from working
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
static s32 codec_node_video_streamon(struct vio_video_ctx *vctx)
{
	s32 ret;

	ret = codec_node_start(vctx);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: stop codec_node from working
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
static s32 codec_node_video_streamoff(struct vio_video_ctx *vctx)
{
	s32 ret;

	ret = codec_node_stop(vctx);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Open the calling function of the codec_node device node
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
static int codec_node_video_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	ret = codec_node_open(vctx);
	if(ret < 0) {
		vio_err("[S%d] open fail\n", vctx->ctx_id);
		return -EFAULT;
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Close the calling function of the codec_node device node
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
static int codec_node_video_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	ret = codec_node_close(vctx);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

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
static s32 codec_node_video_reqbufs(struct vio_video_ctx *vctx,
		struct vbuf_group_info *group_attr)
{
	s32 ret = 0;

	ret = codec_node_reqbufs(vctx, group_attr);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: codec_node reset
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
static s32 codec_node_video_reset(struct vio_video_ctx *vctx)
{
	s32 ret = 0;

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

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
static s32 codec_node_video_error_callback(struct vio_video_ctx *vctx,
		void *error_info)
{
	s32 ret = 0;

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

static s32 codec_node_video_s_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	s32 ret = 0;

	switch (cmd) {
	case CODEC_CMD_BIND_VCTX:
		ret = codec_node_bind_flow_id(vctx, arg);
		break;
	case CODEC_CMD_DQBUF:
		ret = codec_node_dqbuf(vctx, arg);
		break;
	case CODEC_CMD_QBUF:
		ret = codec_node_qbuf(vctx, arg);
		break;
	case CODEC_CMD_GET_CFG:
		ret = codec_node_get_buf_cfg(vctx, arg);
		break;
	case CODEC_CMD_QUERYBUF:
		ret = codec_node_querybuf(vctx, arg);
		break;
	default:
		break;
	}

	vio_info("[S%d]%s cmd[%d] done\n", vctx->ctx_id, __func__, cmd);

	return ret;
}

struct vio_common_ops codec_node_vops = {
	.open = codec_node_video_open,
	.close = codec_node_video_close,
	.video_set_attr = codec_node_video_set_attr,
	.video_get_attr = codec_node_video_get_attr,
   	.video_set_attr_ex = codec_node_video_set_attr_ex,
	.video_get_buf_attr = codec_node_video_reqbufs,
	.video_set_ichn_attr = codec_node_video_set_ichn_attr,
	.video_set_ochn_attr = codec_node_video_set_ochn_attr,
	.video_set_obuf = codec_node_video_set_ochn_buff_attr,
	.video_start = codec_node_video_streamon,
	.video_stop = codec_node_video_streamoff,
	.video_reset = codec_node_video_reset,
	.video_error_callback = codec_node_video_error_callback,
	.video_s_ctrl = codec_node_video_s_ctrl,
	.video_get_version = codec_node_get_version,
	.video_get_struct_size = codec_node_get_struct_size,
};

static s32 codec_node_check_ichn_bind_param(struct vio_subdev *vdev, struct chn_attr *chn_attr)
{
	s32 ret = 0;
	struct codec_node_subdev *subdev;

	subdev = container_of(vdev, struct codec_node_subdev, vdev);

	if (subdev->codec_attr.input_width != chn_attr->width ||
		subdev->codec_attr.input_height != chn_attr->height) {
		vio_err("%s: wrong resolution(%d * %d), expect(%d * %d)", __func__,
			subdev->codec_attr.input_width,	subdev->codec_attr.input_height,
			chn_attr->width, chn_attr->height);
		return -EFAULT;
	}

	if (subdev->codec_attr.input_stride != chn_attr->wstride) {
		vio_err("%s: wrong stride %d, expect %d", __func__,
			subdev->codec_attr.input_stride, chn_attr->wstride);
		return -EFAULT;
	}

	return ret;
}

static void codec_node_set_ochn_bind_param(struct vio_subdev *vdev)
{
	struct codec_node_subdev *subdev;

	subdev = container_of(vdev, struct codec_node_subdev, vdev);
	vdev->chn_attr.width = subdev->codec_attr.output_width;
	vdev->chn_attr.height = subdev->codec_attr.output_height;
	vdev->chn_attr.wstride = subdev->codec_attr.output_stride;
	vdev->chn_attr.vstride =  subdev->codec_attr.output_height;
	vdev->chn_attr.format = MEM_PIX_FMT_NV12;
}

s32 codec_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode)
{
	s32 ret;

	if (osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) != 0)
		return CHN_BIND_REP;

	if (vdev->id == VNODE_ID_SRC) {
		ret = codec_node_check_ichn_bind_param(vdev, &remote_vdev->chn_attr);
		if (ret < 0)
			return CHN_BIND_NONE;
	} else {
		codec_node_set_ochn_bind_param(vdev);
	}

	return CHN_BIND_M2M;
}

void codec_src_worker(struct vio_subdev *vdev, struct vio_frame *frame)
{
	struct vio_framemgr *framemgr;
	struct vio_node *vnode;

	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (frame != NULL) {
		if (vdev->next != NULL) {
			wake_up(&vdev->vctx[0]->done_wq);
		}

		if (vdev->prev != NULL) {
			wake_up(&vdev->vctx[0]->done_wq);
		}
	}

	vio_info("[S%d][N%d][V%d] %s \n", vnode->flow_id, vnode->id, vdev->id, __func__);

	return;
}
/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Vin_node driver node registration function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *codec_node_dev: j6 hardware struct
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 codec_node_device_node_init(struct j6_codec_node_dev *codec_node_dev)
{
	u32 i;
	char name[32];
	struct vio_node *vnode;

	vnode = codec_node_dev->vnode;
	codec_node_dev->hw_id = 0;
	for (i = 0; i < CODEC_MAX_CHANNEL; i++) {
		vnode[i].id = CODEC_MODULE;
		vnode[i].hw_id = codec_node_dev->hw_id;
		vnode[i].flow_id = INVALID_FLOW_ID;
		vnode[i].ctx_id = i;
		vnode[i].ich_subdev[0] = &codec_node_dev->subdev[i][0].vdev;
		vnode[i].active_ich = 1;
		vnode[i].och_subdev[0] = &codec_node_dev->subdev[i][1].vdev;
		vnode[i].active_och = 1;
		codec_node_dev->subdev[i][0].vdev.vnode = &vnode[i];
		codec_node_dev->subdev[i][1].vdev.vnode = &vnode[i];
		vnode[i].allow_bind = codec_allow_bind;
		vnode[i].ich_subdev[0]->vdev_work = codec_src_worker;
		codec_node_dev->subdev[i][0].codec_attr.channel_idx = -1;
		codec_node_dev->subdev[i][1].codec_attr.channel_idx = -1;
	}

	codec_node_dev->codec_device[0].vps_ops = &codec_node_vops;
	codec_node_dev->codec_device[0].ip_dev = codec_node_dev;
	codec_node_dev->codec_device[0].vid = VNODE_ID_SRC;
	codec_node_dev->codec_device[0].vnode = vnode;
	codec_node_dev->codec_device[0].max_ctx = CODEC_MAX_CHANNEL;
	codec_node_dev->codec_device[0].iommu_dev = &codec_node_dev->pdev->dev;

	snprintf(name, sizeof(name), "codec%d_src", codec_node_dev->hw_id);
	vio_register_device_node(name, &codec_node_dev->codec_device[0]);

	codec_node_dev->codec_device[1].vps_ops = &codec_node_vops;
	codec_node_dev->codec_device[1].ip_dev = codec_node_dev;
	codec_node_dev->codec_device[1].vid = VNODE_ID_CAP;
	codec_node_dev->codec_device[1].vnode = vnode;
	codec_node_dev->codec_device[1].max_ctx = CODEC_MAX_CHANNEL;
	codec_node_dev->codec_device[1].iommu_dev = &codec_node_dev->pdev->dev;

	snprintf(name, sizeof(name), "codec%d_cap", codec_node_dev->hw_id);
	vio_register_device_node(name, &codec_node_dev->codec_device[1]);

	return 0;
}
EXPORT_SYMBOL(codec_node_device_node_init);

void codec_node_device_node_deinit(struct j6_codec_node_dev *codec_node_dev)
{
}
EXPORT_SYMBOL(codec_node_device_node_deinit);
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
static s32 hobot_codec_node_probe(struct platform_device *pdev)
{
	s32 ret = 0, i = 0;
	struct j6_codec_node_dev *codec_node;
	struct device *dev;

	vio_info("%s start\n", __func__);

	dev = &pdev->dev;
	codec_node = (struct j6_codec_node_dev *)devm_kzalloc(dev, sizeof(struct j6_codec_node_dev), GFP_KERNEL);
	if (codec_node == NULL) {
		dev_err(dev, "codec_node is NULL");
		return -ENOMEM;
	}

	codec_node->pdev = pdev;
	ret = codec_node_device_node_init(codec_node);
	if (ret < 0) {
		vio_err("codec_node_device_node_init fail\n");
		return ret;
	}

	platform_set_drvdata(pdev, (void *)codec_node);
	mutex_init(&codec_node->mlock);
	atomic_set(&codec_node->open_cnt, 0);
	for (i = 0; i < CODEC_MAX_SUBDEV; i++) {
		(void)test_and_clear_bit(i, codec_node->channel_idx_bitmap);
	}

	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

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
static s32 hobot_codec_node_remove(struct platform_device *pdev)
{
	s32 ret = 0;
	struct j6_codec_node_dev *codec_node;
	struct device *dev;

	if (pdev == NULL) {
		vio_err("%s: pdev = NULL\n", __func__);
		return -EFAULT;
	}

	dev = &pdev->dev;
	codec_node = (struct j6_codec_node_dev *)platform_get_drvdata(pdev);

	vio_unregister_device_node(&codec_node->codec_device[0]);
	vio_unregister_device_node(&codec_node->codec_device[1]);

	devm_kfree(dev, (void *)codec_node);

	dev_info(dev, "%s\n", __func__);

	return ret;
}


/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief codec_node sleep processing function
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
int32_t hobot_codec_node_suspend(struct device *dev)
{
	vio_info("%s %d\n", __func__, __LINE__);
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
int32_t hobot_codec_node_resume(struct device *dev)
{
	vio_info("%s %d\n", __func__, __LINE__);
	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief codec_node's runtime sleep processing function
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
int32_t hobot_codec_node_runtime_suspend(struct device *dev)
{
	vio_info("%s %d\n", __func__, __LINE__);
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
int32_t hobot_codec_node_runtime_resume(struct device *dev)
{
	vio_info("%s %d\n", __func__, __LINE__);
	return 0;
}


static const struct of_device_id hobot_codec_node_match[] = {
	{
	 .compatible = "hobot,j6-codec-vnode",
	},
	{
	 .compatible = "hobot,x5-codec-vnode",
	},
	{},
};

static const struct dev_pm_ops hobot_codec_node_pm_ops = {
	.suspend = hobot_codec_node_suspend,
	.resume = hobot_codec_node_resume,
	.runtime_suspend = hobot_codec_node_runtime_suspend,
	.runtime_resume = hobot_codec_node_runtime_resume,
};

MODULE_DEVICE_TABLE(of, hobot_codec_node_match);

static struct platform_driver hobot_codec_node_driver = {
	.probe = hobot_codec_node_probe,
	.remove = hobot_codec_node_remove,
	.driver = {
			.name = CODEC_MODULE_NAME,
			.owner = THIS_MODULE,
			.pm = &hobot_codec_node_pm_ops,
			.of_match_table = hobot_codec_node_match,
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
static int32_t __init hobot_codec_node_init(void)
{
	int32_t ret = platform_driver_register(&hobot_codec_node_driver);
	if (ret != 0)
		vio_err("J6 codec_node register failed: %d\n", ret);

	return ret;
}

late_initcall(hobot_codec_node_init);

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
static void __exit hobot_codec_node_exit(void)
{
	platform_driver_unregister(&hobot_codec_node_driver);
}

module_exit(hobot_codec_node_exit);
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("zhaojun.li<zhaojun.li@horizon.com>");
MODULE_DESCRIPTION("J6 CODEC_NODE driver");
MODULE_LICENSE("GPL");
