/**
 * @file: hobot_vpf_ops.c
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
#include "vio_config.h"
#define pr_fmt(fmt)    "[VPF ops]:" fmt

#include "hobot_vpf_ops.h"
#include "hobot_vpf_manager.h"

#define PIPELINE_MAGIC_NUM 0x5050
#define PIPELINE_MAGIC_MASK 0xffff

/**
 * Purpose: ko version
 * Value: NA
 * Range: hobot_vpf_ops.c
 * Attention: NA
 */
static struct vio_version_info g_vpf_version = {
	.major = 1,
	.minor = 0
};


static s32 vpf_alloc_default_frames(struct vio_video_ctx *vctx);
static s32 vpf_video_stop(struct vio_video_ctx *vctx);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Free and ummap buffer which alloced by driver;
 * @param[in] *framemgr: point to struct vio_framemgr instance;
 * @param[in] *iommu_dev: point to struct device instance which used for immou ummap;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vpf_free_buffer(struct vio_framemgr *framemgr, void *iommu_dev)
{
	u32 i;
	struct vio_frame *frame;

	for (i = 0; i < framemgr->num_frames; i++) {
		frame = &framemgr->frames[i];
		if (iommu_dev != NULL && frame->vbuf.iommu_map != 0)
			vio_frame_iommu_unmap(iommu_dev, frame);

		if (frame->vbuf.ion_alloced != 0u)
			vio_ion_free(&frame->vbuf);
	}
}

static void vpf_wait_vnode_shot(const struct vio_node *vnode)
{
	u32 cnt = 10;

	while (cnt > 0u) {
		if (osal_test_bit((s32)VIO_NODE_SHOT, &vnode->state) == 0)
			break;

		osal_msleep(5);
		cnt--;
	}

	vio_info("[S%d][%s] %s: cnt %d\n", vnode->flow_id, vnode->name, __func__, cnt);
}

static void vdev_sudden_close(struct vio_subdev *vdev)
{
	u32 i;
	u64 flags = 0;
	struct vio_video_ctx *vctx;

	if (osal_test_bit(VIO_SUBDEV_STREAM_ON, &vdev->state) == 0)
		return;

	vio_e_barrier_irqs(vdev, flags);
	for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
		if (osal_test_bit(i, &vdev->val_ctx_mask) != 0) {
			vctx = vdev->vctx[i];
			if ((vctx->state & BIT((s32)VIO_VIDEO_CLOSE)) == 0u) {
				vctx->state = BIT((s32)VIO_VIDEO_START);
				break;
			}
		}
	}
	vio_x_barrier_irqr(vdev, flags);

	if (i == VIO_MAX_SUB_PROCESS) {
		vio_err("[%s] %s: invalid vdev state\n", vdev->name, __func__);
		return;
	}

	vpf_video_stop(vctx);
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call stop function of previous ip driver in vio chain;
 * @param[in] *vnode: point to struct vio_node instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vpf_safety_close(const struct vio_node *vnode)
{
	u32 i, j, k;
	struct vio_chain *vchain;
	struct vio_node_mgr *vnode_mgr;
	struct vio_node *tmp_vnode;

	vchain = vnode->vchain;
	if (vchain == NULL)
		return;

	for (i = 0; i < MODULE_NUM; i++) {
		vnode_mgr = &vchain->vnode_mgr[i];
		for (j = 0; j < MAX_VNODE_NUM; j++) {
			tmp_vnode = vnode_mgr->vnode[j];
			if (tmp_vnode == NULL)
				continue;

			if (osal_atomic_read(&tmp_vnode->start_cnt) > 0) {
				for (k = 0; k < MAXIMUM_CHN; k++) {
					if ((tmp_vnode->active_ich & (1 << k)) != 0)
						vdev_sudden_close(tmp_vnode->ich_subdev[k]);
				}

				for (k = 0; k < MAXIMUM_CHN; k++) {
					if ((tmp_vnode->active_och & (1 << k)) != 0)
						vdev_sudden_close(tmp_vnode->och_subdev[k]);
				}
				vio_warn("[S%d][%s] %s: sudden close\n", tmp_vnode->flow_id, tmp_vnode->name, __func__);
			}
		}
	}
}

static void vpf_vdev_free_resource(struct vio_subdev *vdev)
{
	struct vio_framemgr *framemgr;

	if (vdev != NULL && osal_atomic_read(&vdev->refcount) == 0) {
		framemgr = vdev->cur_fmgr;
		vpf_free_buffer(framemgr, vdev->iommu_dev);
		frame_manager_close(framemgr);
	}
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Deinit vdev instance and release related resources;
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vpf_vdev_close(struct vio_subdev *vdev)
{
	s32 ret;
	struct vio_node *vnode;

	if (vdev == NULL)
		return;

	vnode = vdev->vnode;
	if (osal_atomic_dec_return(&vdev->refcount) == 0) {
		vpf_safety_close(vnode);

		if (vdev->leader == 1u) {
			ret = vio_group_task_stop(vnode->gtask);
			if (ret < 0)
				vio_err("%s: vio_group_task_stop failed, ret(%d)\n", __func__, ret);
		}

		osal_mutex_lock(&vdev->mlock);
		vdev->leader = 0;
		vdev->reqbuf_flag = 0;
		vdev->state = 0;
		vdev->prev = NULL;
		vdev->next = NULL;
		osal_mutex_unlock(&vdev->mlock);
	}
	vio_info("[%s][S%d][C%d] %s: refcount %d\n", vdev->name, vnode->flow_id, vnode->ctx_id,
		__func__, osal_atomic_read(&vdev->refcount));
}

static s32 vpf_bind_vdev(struct vio_subdev *src_vdev, struct vio_subdev *dst_vdev, enum vio_bind_type btype)
{
	s32 ret = 0;

	if (btype == CHN_BIND_OTF) {//consider TDMF mode
		src_vdev->vnode->next = dst_vdev->vnode;
		dst_vdev->vnode->prev = src_vdev->vnode;
		dst_vdev->vnode->head = src_vdev->vnode;
		osal_set_bit((s32)VIO_NODE_OTF_OUTPUT, &src_vdev->vnode->state);
	}

	if (btype == CHN_BIND_M2M) {
		src_vdev->next = dst_vdev;
		dst_vdev->prev = src_vdev;
		osal_set_bit((s32)VIO_NODE_M2M_OUTPUT, &src_vdev->vnode->state);
		ret = vio_framemgr_share_buf(src_vdev->cur_fmgr, dst_vdev->cur_fmgr, dst_vdev->iommu_dev);
	}

	osal_set_bit((s32)VIO_SUBDEV_BIND_DONE, &src_vdev->state);
	osal_set_bit((s32)VIO_SUBDEV_BIND_DONE, &dst_vdev->state);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind previous vio node output channel to current vio node input channel;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] bind_flag: bind vnode flag;
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
static s32 vpf_bind_vnode(struct vio_video_ctx *vctx, u32 bind_flag)
{
	s32 ret = 0;
	u32 inode, onode, in_ctx, out_ctx, ichn, ochn, ohwid, ihwid;
	struct hobot_vpf_dev *vpf_dev;
	struct vio_chain *vchain;
	struct vio_subdev *src_vdev, *dst_vdev;
	struct vio_node *src_vnode, *dst_vnode;
	enum vio_bind_type src_btype = 0, dst_btype = 0;
	u32 online_mode = 0;

	onode = (bind_flag & OUTPUT_MODULE_MASK) >> OUTPUT_MODULE_SHIFT;
	inode = (bind_flag & INPUT_MODULE_MASK) >> INPUT_MODULE_SHIFT;
	out_ctx = (bind_flag & OUTPUT_CTXID_MASK) >> OUTPUT_CTXID_SHIFT;
	in_ctx = (bind_flag & INPUT_CTXID_MASK) >> INPUT_CTXID_SHIFT;
	ochn = (bind_flag & OUTPUT_CH_MASK) >> OUTPUT_CH_SHIFT;
	ichn = (bind_flag & INPUT_CH_MASK) >> INPUT_CH_SHIFT;
	ohwid = (bind_flag & OUTPUT_HW_ID_MASK) >> OUTPUT_HW_ID_SHIFT;
	ihwid = (bind_flag & INPUT_HW_ID_MASK) >> INPUT_HW_ID_SHIFT;

	vpf_dev = (struct hobot_vpf_dev *)vctx->device;
	vchain = &vpf_dev->iscore.vchain[vctx->ctx_id];
	src_vnode = vnode_mgr_find_member(&vchain->vnode_mgr[onode], ohwid, out_ctx);
	dst_vnode = vnode_mgr_find_member(&vchain->vnode_mgr[inode], ihwid, in_ctx);
	if (src_vnode == NULL || dst_vnode == NULL) {
		vio_err("[S%d] %s: M(%d %d) C%d = %p M(%d %d) C%d = %p, bind failed\n", vctx->ctx_id, __func__,
			onode, ohwid, out_ctx, src_vnode, inode, ihwid, in_ctx, dst_vnode);
		ret = -EFAULT;
		goto err;
	}

	if (ochn == 1 && src_vnode->no_online_support == 0) {
		ochn = 0;
		online_mode = 1;
	}
	src_vdev = src_vnode->och_subdev[ochn];
	dst_vdev = dst_vnode->ich_subdev[ichn];
	if (src_vnode->allow_bind != NULL)
		src_btype = src_vnode->allow_bind(src_vdev, dst_vdev, online_mode);

	if (src_btype == CHN_BIND_REP || src_btype == CHN_BIND_NONE) {
		vio_err("[S%d][%s] %s: bind failed\n", src_vnode->flow_id, src_vdev->name, __func__);
		ret = -EFAULT;
		goto err;
	}

	if (dst_vnode->allow_bind != NULL)
		dst_btype = dst_vnode->allow_bind(dst_vdev, src_vdev, online_mode);

	if (dst_btype == CHN_BIND_REP || dst_btype == CHN_BIND_NONE) {
		vio_err("[S%d][%s] %s: bind failed\n", dst_vnode->flow_id, dst_vdev->name, __func__);
		ret = -EFAULT;
		goto err;
	}
	if (src_btype != dst_btype) {
		vio_err("[S%d] %s: %s btype%d can't bind %s btype %d \n", src_vnode->flow_id, __func__,
			src_vdev->name, src_btype, dst_vdev->name, dst_btype);
		ret = -EFAULT;
		goto err;
	}

	ret = vpf_bind_vdev(src_vdev, dst_vdev, src_btype);
	if (ret < 0)
		goto err;

	vio_info("[S%d] %s: %s bind_type(%d) %s\n", src_vnode->flow_id, __func__,
		src_vdev->name, src_btype, dst_vdev->name);
err:
	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vio chain and get flow id;
 * @param[in] *iscore: point to struct vio_core instance;
 * @retval "= 0": success
 * @retval "< 0": failure
 * @param[out] *flow_id: flow id;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vpf_bind_chain(struct vio_core *iscore, u32 *flow_id)
{
	s32 i;
	s32 ret = 0;
	struct vio_chain *vchain;

	osal_mutex_lock(&iscore->mlock);
	for (i = 0; i < VIO_MAX_STREAM; i++) {
		vchain = &iscore->vchain[i];
		if (osal_test_bit(VIO_CHAIN_OPEN, &vchain->state) == 0) {
			osal_set_bit(VIO_CHAIN_OPEN, &vchain->state);
			*flow_id = i;
			break;
		}
	}
	if (i == VIO_MAX_STREAM) {
		vio_err("%s: alloc chain failed\n", __func__);
		ret = -EFAULT;
	}
	osal_mutex_unlock(&iscore->mlock);
	vio_info("[S%d] %s: bind done", i, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vio chain by pipe id;
 * @param[in] *iscore: point to struct vio_core instance;
 * @param[in] flow_id: pipeline id;
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
static s32 vpf_bind_chain_with_flowid(struct vio_core *iscore, u32 flow_id)
{
	s32 ret = 0;
	struct vio_chain *vchain;

	if (flow_id >= VIO_MAX_STREAM) {
		vio_err("%s: wrong flow_id %d\n", __func__, flow_id);
		return -EFAULT;
	}

	vchain = &iscore->vchain[flow_id];
	osal_mutex_lock(&iscore->mlock);
	if (osal_test_bit(VIO_CHAIN_OPEN, &vchain->state) == 0) {
		osal_set_bit(VIO_CHAIN_OPEN, &vchain->state);
		vio_info("[S%d] %s: done", flow_id, __func__);
	} else {
		ret = -EFAULT;
		vio_err("[S%d] %s: already bind, can't bind again\n", flow_id, __func__);
	}
	osal_mutex_unlock(&iscore->mlock);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vctx to vdev;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] *vdev: point to struct vio_subdev instance;
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
static s32 vpf_vctx_bind_vdev(struct vio_video_ctx *vctx, struct vio_subdev *vdev)
{
	u32 i = 0, ctx_mask = 1;
	s32 ret = 0;
	u64 flags = 0;

	if (vdev->multi_process == 1)
		ctx_mask = VIO_MAX_SUB_PROCESS;

	vio_e_barrier_irqs(vdev, flags);
	for (i = 0; i < ctx_mask; i++) {
		if (osal_test_and_set_bit(i, &vdev->val_ctx_mask) == 0) {
			vdev->vctx[i] = vctx;
			vctx->ctx_index = i;
			vctx->vdev = vdev;
			osal_atomic_inc(&vdev->refcount);
			vctx->framemgr = &vdev->framemgr;
			vdev->cur_fmgr = &vdev->framemgr;
			vio_info("[%s][C%d] %s: done, ctx_index(%d) refcount(%d)\n",
				vctx->name, vctx->ctx_id, __func__,
				i, osal_atomic_read(&vdev->refcount));
			break;
		}
	}
	vio_x_barrier_irqr(vdev, flags);

	if (i == ctx_mask) {
		vio_err("[%s] %s: failed, ctx_mask = %d\n", vctx->name, __func__, ctx_mask);
		ret = -EFAULT;
	}

	return ret;
}

static void vpf_vctx_unbind_vdev(struct vio_video_ctx *vctx)
{
	u64 flags = 0;
	struct vio_subdev *vdev;

	vdev = vctx->vdev;
	if (vdev != NULL) {
		vio_e_barrier_irqs(vdev, flags);
		osal_clear_bit(vctx->ctx_index, &vdev->val_ctx_mask);
		vctx->vdev = NULL;
		vdev->vctx[vctx->ctx_index] = NULL;
		vio_x_barrier_irqr(vdev, flags);
	}
}

static void vpf_free_ctx_id(struct vio_video_ctx *vctx)
{
	u32 i;
	u32 ctx_id;
	struct vpf_device *dev;
	struct vio_subdev *vdev;
	struct vio_node *vnode;

	dev = vctx->dev;
	ctx_id = vctx->ctx_id;
	if (ctx_id < dev->max_ctx) {
		vnode = &dev->vnode[ctx_id];
		for (i = 0; i < MAXIMUM_CHN; i++) {
			if ((vnode->active_ich & 1 << i) != 0) {
				vdev = vnode->ich_subdev[i];
				if (osal_atomic_read(&vdev->refcount) != 0)
					break;
			}

			if ((vnode->active_och & 1 << i) != 0) {
				vdev = vnode->och_subdev[i];
				if (osal_atomic_read(&vdev->refcount) != 0)
					break;
			}
		}

		if (i == MAXIMUM_CHN)
			vnode->state = 0;
	}
	vio_dbg("[%s] %s: ctx_id = %d\n", vctx->name, __func__, ctx_id);
}

static s32 vpf_alloc_ctx_id(struct vio_video_ctx *vctx, s32 ctx_id)
{
	u32 i = 0;
	u32 active_ctx_id;
	struct vio_node *vnode = NULL;
	struct vpf_device *dev;

	active_ctx_id = ctx_id;
	if (vctx->id == VNODE_ID_SRC) {
		dev = vctx->dev;
		if (ctx_id < AUTO_CTX_ID || ctx_id >= dev->max_ctx) {
			vio_err("[%s] %s: wrong ctx_id %d, max_ctx %d\n",
				vctx->name, __func__, ctx_id, dev->max_ctx);
			return -EFAULT;
		}

		if (ctx_id == AUTO_CTX_ID) {
			for (i = 0; i < dev->max_ctx; i++) {
				vnode = &dev->vnode[i];
				if (osal_test_and_set_bit(VIO_NODE_BIND_CTX, &vnode->state) == 0u) {
					active_ctx_id = i;
					break;
				}
			}

			if (i == dev->max_ctx) {
				vio_err("[%s] %s: all ctx id is used, max_ctx %d\n",
					vctx->name, __func__, dev->max_ctx);
				return -EFAULT;
			}
		} else {
			vnode = &dev->vnode[ctx_id];
			osal_set_bit(VIO_NODE_BIND_CTX, &vnode->state);
		}
	} 
	vctx->ctx_id = active_ctx_id;

	vio_dbg("[%s] %s: ctx_id = %d\n", vctx->name, __func__, vctx->ctx_id);

	return 0;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind the dependencies of the three(vctx, vnode and vdev);
 * @param[in] *vctx: point to struct vio_video_ctx instance
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
static s32 vpf_bind_context(struct vio_video_ctx *vctx, s32 ctx_id)
{
	s32 ret = 0;
	u32 i = 0;
	struct vio_node *vnode;
	struct vpf_device *dev;

	ret = vpf_alloc_ctx_id(vctx, ctx_id);
	if (ret < 0)
		return ret;

	dev = vctx->dev;
	vnode = &dev->vnode[vctx->ctx_id];
	for (i = 0; i < MAXIMUM_CHN; i++) {
		if (vctx->id == i) {
			ret = vpf_vctx_bind_vdev(vctx, vnode->ich_subdev[i]);
			if (ret < 0) {
				vio_err("[%s] %s: failed\n", vctx->name, __func__);
				return ret;
			}
		}

		if (vctx->id == VNODE_ID_CAP + i) {
			ret = vpf_vctx_bind_vdev(vctx, vnode->och_subdev[i]);
			if (ret < 0) {
				vio_err("[%s] %s: failed\n", vctx->name, __func__);
				return ret;
			}
		}
	}
	vio_vnode_init(vnode);
	vctx->flow_id = vnode->flow_id;// for multi-process to get same context data

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Transfer frame from free queue to request queue;
 * @param[in] *vdev: point to struct vio_subdev instance;
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
static s32 vpf_prepare_buffers(struct vio_subdev *vdev)
{
	u32 i;
	s32 ret = 0;
	struct frame_info frameinfo;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;

	if (vdev->reqbuf_flag == 0)
		return ret;

	framemgr = vdev->cur_fmgr;
	for (i = 0; i < framemgr->num_frames; i++) {
		frame = &framemgr->frames[i];
		if (frame->state != FS_FREE || frame->internal_buf == 0)
			continue;
		(void)memcpy(&frameinfo, &frame->frameinfo, sizeof(frameinfo));
		frameinfo.bufferindex = (s32)i;
		ret = vio_subdev_qbuf(vdev, &frameinfo);
	}
	vio_info("[%s][S%d] %s: done\n", vdev->name, vdev->vnode->flow_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_attr of ip driver which set global configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_set_init_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;
	struct vio_subdev *vdev;
	struct vio_node *vnode;

	if ((vctx->state & (BIT((s32)VIO_VIDEO_S_INPUT))) == 0u) {
		vio_err("[%s][C%d] %s: invalid INIT_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}

	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_INIT_ATTR, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: invalid INIT_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		ret = -EINVAL;
		goto err;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_attr) {
		ret = dev->vps_ops->video_set_attr(vctx, arg);
		if (ret < 0)
			goto err;
	}

	vnode = vdev->vnode;
	if (vdev->leader == 1u)
		ret = vio_group_task_start(vnode->gtask);

	osal_set_bit((s32)VIO_SUBDEV_INIT_ATTR, &vdev->state);
	vctx->state = BIT((s32)VIO_VIDEO_INIT_ATTR);

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);
err:
	osal_mutex_unlock(&vdev->mlock);
	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_get_attr of ip driver which get global configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_get_init_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_attr) {
		ret = dev->vps_ops->video_get_attr(vctx, arg);
		if (ret < 0)
			return ret;
	}

	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_attr_ex of ip driver which set global dynamic configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_set_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	if ((vctx->state & (BIT((s32)VIO_VIDEO_CHN_ATTR) |
		BIT((s32)VIO_VIDEO_REBUFS)|
		BIT((s32)VIO_VIDEO_START))) == 0) {
		vio_err("[%s][C%d] %s: invalid ATTR_EX is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_attr_ex) {
		ret = dev->vps_ops->video_set_attr_ex(vctx, arg);
		if (ret < 0)
			return ret;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_get_attr_ex of ip driver which get global dynamic configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_get_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_attr_ex) {
		ret = dev->vps_ops->video_get_attr_ex(vctx, arg);
		if (ret < 0)
			return ret;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which set input channel related configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_set_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;
	struct vio_subdev *vdev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_INIT_ATTR)) == 0) {
		vio_err("[%s][C%d] %s: invalid ICHN_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_CHN_ATTR, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: invalid ICHN_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		ret = -EINVAL;
		goto err;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_ichn_attr) {
		ret = dev->vps_ops->video_set_ichn_attr(vctx, arg);
		if (ret < 0)
			goto err;
	}

	ret = vpf_alloc_default_frames(vctx);
	if (ret < 0)
		goto err;

	osal_set_bit((s32)VIO_SUBDEV_CHN_ATTR, &vdev->state);
	vctx->state = BIT((s32)VIO_VIDEO_CHN_ATTR);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

err:
	osal_mutex_unlock(&vdev->mlock);
	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which set input channel related dynamic configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_set_ichn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s][C%d] %s: invalid ICHN_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_ichn_attr_ex) {
		ret = dev->vps_ops->video_set_ichn_attr_ex(vctx, arg);
		if (ret < 0)
			return ret;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which get input channel related configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_get_ichn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_ichn_attr) {
		ret = dev->vps_ops->video_get_ichn_attr(vctx, arg);
		if (ret < 0)
			return ret;
	}
	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which set output channel related configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_set_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;
	struct vio_subdev *vdev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_INIT_ATTR)) == 0) {
		vio_err("[%s][C%d] %s: invalid OCHN_ATTR is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_CHN_ATTR, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: subdev already init, current refcount(%d)\n", vctx->name,
			vctx->ctx_id, __func__, osal_atomic_read(&vdev->refcount));
		ret = -EINVAL;
		goto err;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_ochn_attr) {
		ret = dev->vps_ops->video_set_ochn_attr(vctx, arg);
		if (ret < 0)
			goto err;
	}

	ret = vpf_alloc_default_frames(vctx);
	if (ret < 0)
		goto err;

	osal_set_bit((s32)VIO_SUBDEV_CHN_ATTR, &vdev->state);
	vctx->state = BIT((s32)VIO_VIDEO_CHN_ATTR);

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);
err:
	osal_mutex_unlock(&vdev->mlock);

	return ret;
}

static s32 vpf_video_set_inter_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;
	struct vio_subdev *vdev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_inter_attr) {
		ret = dev->vps_ops->video_set_inter_attr(vctx, arg);
		if (ret < 0)
			return ret;
	}

	vdev = vctx->vdev;
	if (osal_test_and_set_bit((s32)VIO_SUBDEV_REQBUF, &vdev->state) == 0) {
		ret = vpf_alloc_default_frames(vctx);
		if (ret < 0)
			return ret;
	}

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

static s32 vpf_video_get_version(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	s64 copy_ret;
	struct vpf_device *dev;
	struct vio_version_info version;

	(void)memset(&version, 0, sizeof(version));
	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_version) {
		ret = dev->vps_ops->video_get_version(&version);
		if (ret < 0)
			return ret;
	}

	copy_ret = osal_copy_to_app((void __user *) arg, &version, sizeof(version));
	if (copy_ret != 0) {
		vio_err("[%s] %s: copy_to_user failed, ret(%lld)", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: get the size of given struct;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_get_struct_size(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	s64 copy_ret;
	struct vpf_device *dev;
	struct vio_struct_size vio_size = {0};

	copy_ret = osal_copy_from_app(&vio_size, (void __user *) arg, sizeof(vio_size));
	if (copy_ret != 0) {
		vio_err("[%s] %s: copy_from_user failed, ret(%lld)",
			vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_struct_size) {
		ret = dev->vps_ops->video_get_struct_size(&vio_size);
		if (ret < 0)
			return ret;
	}

	copy_ret = osal_copy_to_app((void __user *) arg, &vio_size, sizeof(vio_size));
	if (copy_ret != 0) {
		vio_err("[%s] %s: copy_to_user failed, ret(%lld)", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which set output channel related dynamic configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_set_ochn_attr_ex(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s][C%d] %s: invalid OCHN_ATTR_EX is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_set_ochn_attr_ex) {
		ret = dev->vps_ops->video_set_ochn_attr_ex(vctx, arg);
		if (ret < 0)
			return ret;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_set_ichn_attr of ip driver which get output channel related configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type
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
static s32 vpf_video_get_ochn_attr(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	struct vpf_device *dev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_ochn_attr) {
		ret = dev->vps_ops->video_get_ochn_attr(vctx, arg);
		if (ret < 0)
			return ret;
	}

	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Set buffer infomation in vio_buffer and alloc buffer;
 * @param[in] *buffer: point to struct vio_buffer instance;
 * @param[in] *group_attr: point to struct vbuf_group_info instance;
 * @param[in] alloc: 1 means alloc buffer, 0 means not alloc;
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
static s32 vpf_fill_frame_info(struct vio_frame *frame, struct vbuf_group_info *group_attr)
{
	s32 ret = 0;
	u32 i;
	struct vbuf_group_info *group_info;
	struct vbuf_info *meta_info = NULL;
	struct vio_buffer *vbuf;
	struct vbuf_attr *buf_attr;

	vbuf = &frame->vbuf;
	group_info = &vbuf->group_info;
	(void)memcpy(group_info, group_attr, sizeof(struct vbuf_group_info));

	for (i = 0; i < HBN_LAYER_MAXIMUM; i++) {
		buf_attr = &group_info->info[i].buf_attr;
		if ((1 << i & group_attr->bit_map) != 0) {
			if (meta_info == NULL)
				meta_info = &group_info->info[i];

			if (buf_attr->wstride == 0) {
				buf_attr->wstride = buf_attr->width;
				vio_warn("[F%d] %s: L%d use width instead of wstride\n", frame->index, __func__, i);
			}
			if (buf_attr->vstride == 0) {
				buf_attr->vstride = buf_attr->height;
				vio_warn("[F%d] %s: L%d use height instead of vstride\n", frame->index, __func__, i);
			}
		}
		switch (buf_attr->format) {
			case MEM_PIX_FMT_RAW8:
			case MEM_PIX_FMT_RAW10:
			case MEM_PIX_FMT_RAW12:
			case MEM_PIX_FMT_RAW14:
			case MEM_PIX_FMT_RAW16:
			case MEM_PIX_FMT_RAW20:
			case MEM_PIX_FMT_RAW24:
			case MEM_PIX_FMT_RGB24:
			case MEM_PIX_FMT_RGB565:
				if (group_attr->info[0].buf_attr.planecount == 2) {
					group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
					group_info->info[i].planeSize[1] = buf_attr->wstride * buf_attr->vstride;
				} else {
					group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				}
				break;
			case MEM_PIX_FMT_ARGB:
#ifdef X5_CHIP
				/* 测试 csc 的时候 nv12 转 rgba libhbmem.so size 检查会报错,
				 * 可能 libhbmem.so 的计算方式出错,所以这里进行了单独处理.
				 */
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride * 4;
#else
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
#endif
				break;
			case MEM_PIX_FMT_NV12:
			case MEM_PIX_FMT_NV21:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				group_info->info[i].planeSize[1] = group_info->info[i].planeSize[0] / 2;
				break;
			case MEM_PIX_FMT_NV16:
			case MEM_PIX_FMT_NV61:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				group_info->info[i].planeSize[1] = group_info->info[i].planeSize[0];
				break;
			case MEM_PIX_FMT_NV24:
			case MEM_PIX_FMT_NV42:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				group_info->info[i].planeSize[1] = group_info->info[i].planeSize[0] * 2;
				break;
			case MEM_PIX_FMT_YUV420P:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				group_info->info[i].planeSize[1] = group_info->info[i].planeSize[0] / 4;
				group_info->info[i].planeSize[2] = group_info->info[i].planeSize[1];
				break;
			case MEM_PIX_FMT_YUV422P:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride;
				group_info->info[i].planeSize[1] = group_info->info[i].planeSize[0] / 2;
				group_info->info[i].planeSize[2] = group_info->info[i].planeSize[1];
				break;
			case MEM_PIX_FMT_YUYV422:
				group_info->info[i].planeSize[0] = buf_attr->wstride * buf_attr->vstride * 2;
				break;
			default:
				vio_dbg("[F%d] %s: L%d wrong format(%d)\n",
					group_info->index, __func__, i, buf_attr->format);
				break;
		}
		vio_dbg("[F%d] %s: L%d plane0 0x%lx plane1 0x%lx\n", group_info->index, __func__, i,
			group_info->info[i].planeSize[0], group_info->info[i].planeSize[1]);
	}
	group_info->index = frame->index;

	if (group_attr->metadata_en == 1 && meta_info != NULL)
		meta_info->planeSize[VIO_META_PLANE] = METADATA_SIZE;

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Initialize framemgr and alloc vio_buffer
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @param[in] *group_attr: point to struct vbuf_group_info instance;
 * @param[in] buffers: buffer number;
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
static s32 vpf_request_frames(struct vio_subdev *vdev, struct vbuf_group_info *group_attr)
{
	s32 ret = 0;
	u32 i;
	u32 buffers;
	struct vio_framemgr *framemgr;
	struct vio_node *vnode;
	struct vio_frame *frame;

	vnode = vdev->vnode;
	buffers = group_attr->buffers_num;
	if (buffers == 0) {
		vio_err("[%s][C%d] %s: wrong buffers 0, please check it\n",
			vdev->name, vnode->ctx_id, __func__);
		return -EFAULT;
	}

	framemgr = &vdev->framemgr;
	if (framemgr->num_frames == buffers) {
		vio_dbg("[%s] %s: same num_frames and ignore reqbufs\n", vdev->name, __func__);
		return ret;
	}
	ret = frame_manager_open(framemgr, buffers);
	if (ret < 0) {
		vio_err("[%s] %s: frame manage open failed, ret(%d)",
			vdev->name, __func__, ret);
		return ret;
	}
	framemgr->name = vdev->name;
	for (i = 0; i < buffers; i++) {
		frame = &framemgr->frames[i];
		frame->vnode = (void *)vnode;
		ret = vpf_fill_frame_info(frame, group_attr);
		if (ret < 0) {
			vio_err("[%s][F%d] %s: alloc failed\n", vdev->name, i, __func__);
			return ret;
		}
	}
	vio_info("[%s][C%d] %s: reqbuf number %d\n",
		vdev->name, vnode->ctx_id, __func__, buffers);

	return ret;
}

static s32 vpf_alloc_buffer(struct vio_framemgr *framemgr, void *iommu_dev,
	struct vbuf_group_info *group_attr)
{
	u32 i;
	s32 ret = 0;
	struct vio_frame *frame;

	if (group_attr->is_alloc != 0) {
		for (i = 0; i < framemgr->num_frames; i++) {
			frame = &framemgr->frames[i];
			frame->vbuf.group_info.is_contig = group_attr->is_contig;
			frame->frameinfo.is_contig = group_attr->is_contig;
			frame->frameinfo.num_planes = group_attr->info[0].buf_attr.planecount;
			ret = vio_ion_alloc(&frame->vbuf);
			if (ret < 0)
				return ret;

			frame->internal_buf = 1;
			(void)memcpy(frame->frameinfo.ion_id, frame->vbuf.group_info.info[0].share_id,
				sizeof(u32) * VIO_BUFFER_MAX_PLANES);
			if (group_attr->is_alloc == BUF_ALLOC_AND_MAP) {
				ret = vio_frame_iommu_map(iommu_dev, frame);
				if (ret < 0) {
					vio_err("[Fmgr%d][F%d] %s: iommu map failed\n", framemgr->id, i, __func__);
					return ret;
				}
			}
		}
	}

	return ret;
}

static s32 vpf_request_buffers(struct vio_video_ctx *vctx,
	struct hbn_buf_alloc_attr *alloc_attr, u8 user_set)
{
	s32 ret = 0;
	struct vbuf_group_info group_attr;
	struct vio_subdev *vdev;
	struct vpf_device *dev;

	vdev = vctx->vdev;
	(void)memset(&group_attr, 0, sizeof(group_attr));
	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_buf_attr) {
		ret = dev->vps_ops->video_get_buf_attr(vctx, &group_attr);
		if (ret < 0)
			goto err;
	}

	if (user_set == 1 && group_attr.is_alloc != 0) {
		vio_err("[%s] %s: already alloc default buffers and isn't allowed to alloc by user\n",
			vctx->name, __func__);
		ret = -EFAULT;
		goto err;
	}

	if (user_set == 1) {
		group_attr.flags = alloc_attr->flags;
		group_attr.is_contig = alloc_attr->is_contig;
		group_attr.buffers_num = alloc_attr->buffers_num;
		group_attr.is_alloc = 1;
		group_attr.metadata_en = 1;
	}

	if (group_attr.buffers_num == 0)
		group_attr.buffers_num = alloc_attr->buffers_num;

	ret = vpf_request_frames(vdev, &group_attr);
	if (ret < 0)
		goto err;

	ret = vpf_alloc_buffer(vdev->cur_fmgr, vdev->iommu_dev,  &group_attr);
	if (ret < 0)
		goto err;

	vdev->reqbuf_flag = 1;

err:
	return ret;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_get_buf_attr of ip driver to get buffer information an alloc buffer stored in framemgr;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_reqbufs(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct hbn_buf_alloc_attr alloc_attr;
	struct vio_subdev *vdev;

	if ((vctx->state & (BIT(VIO_VIDEO_CHN_ATTR))) == 0u) {
		vio_err("[%s][C%d] %s: invalid REQBUFS is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}

	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_REQBUF, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: subdev already reqbufs, current refcount(%d)\n", vctx->name,
			vctx->ctx_id, __func__, osal_atomic_read(&vdev->refcount));
		ret = -EINVAL;
		goto err;
	}

	copy_ret = osal_copy_from_app(&alloc_attr, (void __user *) arg, sizeof(alloc_attr));
	if (copy_ret != 0) {
		vio_err("[%s] %s: copy_from_user failed, ret(%lld)",
			vctx->name, __func__, copy_ret);
		ret = -EINVAL;
		goto err;
	}

	ret = vpf_request_buffers(vctx, &alloc_attr, 1);
	if (ret < 0)
		goto err;

	osal_set_bit((s32)VIO_SUBDEV_REQBUF, &vdev->state);
	vctx->state = BIT((s32)VIO_VIDEO_REBUFS);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

err:
	osal_mutex_unlock(&vdev->mlock);

	return ret;
}

static s32 vpf_alloc_default_frames(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct hbn_buf_alloc_attr alloc_attr;

	(void)memset(&alloc_attr, 0, sizeof(alloc_attr));
	alloc_attr.buffers_num = VIO_MAX_FRAMES;
	ret = vpf_request_buffers(vctx, &alloc_attr, 0);
	if (ret < 0) {
		vio_err("[%s][C%d] %s: alloc frame failed\n",
			vctx->name, vctx->ctx_id, __func__);
		return -EINVAL;
	}

	return ret;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_stop of ip driver to make ip standy state;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
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
static s32 vpf_video_stop(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	s32 i = 0;
	struct vpf_device *dev;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vio_framemgr *framemgr;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0u) {
		vio_err("[%s][C%d] %s: invalid STREAMOFF is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}

	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_STREAM_OFF, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: subdev already Stream off, current refcount(%d)\n", vctx->name,
			vctx->ctx_id, __func__, osal_atomic_read(&vdev->refcount));
		ret = -EINVAL;
		goto err;
	}

	vnode = vdev->vnode;
	osal_clear_bit((s32)VIO_NODE_START, &vnode->state);
	if (vdev->id < VNODE_ID_CAP && vdev->leader == 1) {
		framemgr = vdev->cur_fmgr;
		for (i = 0; i < framemgr->num_frames; i++)
			vio_group_cancel_work(vnode, &framemgr->frames[i]);
		vpf_wait_vnode_shot(vnode);
	}

	osal_atomic_dec(&vnode->start_cnt);
	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_stop) {
		ret = dev->vps_ops->video_stop(vctx);
		if (ret < 0)
			goto err;
	}

	if (vdev->reqbuf_flag == 1u) {
		frame_manager_flush(vdev->cur_fmgr);
		osal_atomic_set(&vnode->rcount, 0);
		vctx->event = 0;
	}

	vctx->state = BIT((s32)VIO_VIDEO_STOP);
	osal_set_bit((s32)VIO_SUBDEV_STREAM_OFF, &vdev->state);
	osal_clear_bit((s32)VIO_SUBDEV_STREAM_ON, &vdev->state);

	vio_info("[%s][S%d][C%d] %s: done start_cnt %d\n", vctx->name, vnode->flow_id,
		vctx->ctx_id, __func__,
		osal_atomic_read(&vnode->start_cnt));
err:
	osal_mutex_unlock(&vdev->mlock);

	return ret;
}

static void vpf_fdebug_init(struct vio_subdev *vdev)
{
	struct frame_debug *fdebug;
	struct vio_node *vnode;

	vnode = vdev->vnode;
	fdebug = &vdev->fdebug;
	(void)memset(&vdev->fdebug, 0, sizeof(struct frame_debug));
	fdebug->idset.flow_id = vnode->flow_id;
	fdebug->idset.module_id = vnode->id;
	fdebug->idset.hw_id = vnode->id;
	fdebug->idset.ctx_id = vnode->ctx_id;
	fdebug->idset.chn_id = vdev->id;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Call video_stop of ip driver to make ip ready state;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
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
static s32 vpf_video_start(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct vpf_device *dev;
	struct vio_node *vnode;
	struct vio_subdev *vdev;

	if ((vctx->state & (BIT((s32)VIO_VIDEO_STOP)
			| BIT((s32)VIO_VIDEO_REBUFS)
			| BIT((s32)VIO_VIDEO_CHN_ATTR))) == 0u) {
		vio_err("[%s][C%d] %s: invalid STREAMON is requested(0x%llX)",
			vctx->name, vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}
	vdev = vctx->vdev;
	osal_mutex_lock(&vdev->mlock);
	if (osal_test_bit((s32)VIO_SUBDEV_STREAM_ON, &vdev->state) != 0) {
		vio_err("subdev already Stream on, current refcount(%d)\n", osal_atomic_read(&vdev->refcount));
		ret = -EINVAL;
		goto err;
	}

	vnode = vdev->vnode;
	vctx->flow_id = vnode->flow_id;
	/* debug used*/
	vpf_fdebug_init(vdev);
	vio_chain_path_show(vnode->vchain);
	if (osal_atomic_inc_return(&vnode->start_cnt) == 1)
		osal_set_bit((s32)VIO_NODE_START, &vnode->state);

	if (vdev->id >= VNODE_ID_CAP) {
		ret = vpf_prepare_buffers(vctx->vdev);
		if (ret < 0)
			goto err;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_start) {
		ret = dev->vps_ops->video_start(vctx);
		if (ret < 0)
			goto err;
	}

	vctx->state = BIT((s32)VIO_VIDEO_START);
	osal_set_bit((s32)VIO_SUBDEV_STREAM_ON, &vdev->state);
	osal_clear_bit((s32)VIO_SUBDEV_STREAM_OFF, &vdev->state);

	vio_info("[%s][S%d][C%d] %s: done start_cnt %d\n", vctx->name, vnode->flow_id,
		vctx->ctx_id, __func__,
		osal_atomic_read(&vnode->start_cnt));
err:
	osal_mutex_unlock(&vdev->mlock);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vio chain to get flow id;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] flow_id: flow id;
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
static s32 vpf_chain_add_vnode(struct vio_video_ctx *vctx, u32 flow_id)
{
	s32 ret = 0;
	struct vio_chain *vchain;
	struct vio_node *vnode;

	vchain = vio_get_chain(flow_id);
	if (vchain == NULL) {
		vio_err("%s: wrong flow id %d\n", __func__, flow_id);
		return -EFAULT;
	}

	vnode = vctx->vdev->vnode;
	ret = vnode_mgr_add_member(&vchain->vnode_mgr[vnode->id], vnode);
	if (ret < 0)
		return ret;

	vnode->vchain = vchain;
	vnode->flow_id = flow_id;
	vctx->flow_id = flow_id;

	vio_info("[%s][S%d] %s: done\n", vctx->name, vctx->flow_id, __func__);
	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Initialize struct vio_subdev instance;
 * @param[in] *vnode: point to struct vio_node instance;
 * @param[in] id: device node id;
 * @param[in] *iommu_dev: point to struct device instance which used for immou ummap;
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vpf_device_probe(struct vpf_device *vpf_device)
{
	s32 i;
	u32 vid;
	struct vio_subdev *vdev;
	struct vio_node *vnode;

	vid = vpf_device->vid;
	if (vid >= VNODE_ID_MAX) {
		vio_err("[%s] %s: wrong vid %d\n", vpf_device->name, __func__, vid);
		return -EFAULT;
	}

	for (i = 0; i < vpf_device->max_ctx; i++) {
		vdev = NULL;
		vnode = &vpf_device->vnode[i];
		if (vid < VNODE_ID_CAP) {
			if ((vnode->active_ich & 1 << vid) != 0)
				vdev = vnode->ich_subdev[vid];
		} else {
			if ((vnode->active_och & 1 << (vid - VNODE_ID_CAP)) != 0)
				vdev = vnode->och_subdev[vid - VNODE_ID_CAP];
		}

		if (vdev == NULL) {
			vio_err("[%s] %s: vid %d not match vdev\n",
				vpf_device->name, __func__, vid);
			return -EFAULT;
		}

		osal_spin_init(&vdev->slock);/*PRQA S 3334*/
		osal_mutex_init(&vdev->mlock);/*PRQA S 3334*/
		osal_atomic_set(&vdev->refcount, 0);
		vdev->name = vpf_device->name;
		vdev->id = vid;
		vdev->iommu_dev = vpf_device->iommu_dev;
	}
	if (vpf_device->max_ctx > 0 && vpf_device->gtask == NULL)
		vpf_device->gtask = vpf_device->vnode[vpf_device->max_ctx -1].gtask;

	return 0;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vio node to vio chain;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_add_vnode(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	u32 flow_id = 0;

	copy_ret = osal_copy_from_app(&flow_id, (void __user *) arg, sizeof(u32));
	if (copy_ret != 0) {
		vio_err("[%s][C%d] %s: failed to get user, ret = %lld\n", vctx->name,
			vctx->ctx_id, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vpf_chain_add_vnode(vctx, flow_id);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind the dependencies of the three(vctx, vnode and vdev);
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_bind_context(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	s32 ctx_id = 0;

	if ((vctx->state & BIT((s32)VIO_VIDEO_OPEN)) == 0u) {
		vio_err("[%s][C%d] %s: invalid BIND is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}

	copy_ret = osal_copy_from_app(&ctx_id, (void __user *) arg, sizeof(s32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vpf_bind_context(vctx, ctx_id);
	if (ret < 0)
		return ret;

	copy_ret = osal_copy_to_app((void __user *) arg, &vctx->ctx_id, sizeof(s32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: failed to copy to app, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vctx->state = BIT((s32)VIO_VIDEO_S_INPUT);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id,  __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Queue frame buffer into request queue;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_qbuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct frame_info frameinfo;
	struct vio_subdev *vdev;

	if ((vctx->state & (BIT(VIO_VIDEO_START) | BIT(VIO_VIDEO_CHN_ATTR) |
		BIT(VIO_VIDEO_REBUFS) | BIT(VIO_VIDEO_STOP))) == 0) {
		vio_err("[%s][C%d] %s: invalid qbuf is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	vdev = vctx->vdev;
	if (vdev->id == VNODE_ID_SRC && osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) != 0) {
		vio_err("[%s][C%d] %s: src node already bind, can't send again",
			vctx->name, vctx->ctx_id, __func__);
		return -EFAULT;
	}

	copy_ret = osal_copy_from_app((void *)&frameinfo, (void __user *)arg, sizeof(struct frame_info));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vio_subdev_qbuf(vctx->vdev, &frameinfo);
	vio_dbg("[%s][S%d] %s: done\n", vctx->name, vctx->flow_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Dequeue frame buffer from complete queue;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_dqbuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct frame_info frameinfo;
	struct vio_subdev *vdev;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s][C%d] %s: invalid qbuf is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EFAULT;
	}

	vdev = vctx->vdev;
	if (vctx->event == VIO_FRAME_PREINT) {
		(void)memcpy(&frameinfo, &vdev->curinfo, sizeof(struct frame_info));
		vio_dbg("[%s][S%d] %s: preint %d\n", vctx->name, vctx->flow_id, __func__, frameinfo.frame_done);
	} else {
		ret = vio_subdev_dqbuf(vdev, &frameinfo);
	}
	vctx->event = 0;

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&frameinfo, sizeof(struct frame_info));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy to user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}
	vio_dbg("[%s][S%d] %s: done\n", vctx->name, vctx->flow_id, __func__);

	return ret;
}

static s32 vpf_subdev_querybuf(struct vio_subdev *vdev, struct vbuf_group_info *info)
{
	u32 index;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;

	framemgr = vdev->cur_fmgr;
	index = (u32)info->index;
	if (index >= framemgr->num_frames) {
		vio_err("[%s] %s: wrong index %d, num_frames %d\n", vdev->name,
			__func__, index, framemgr->num_frames);
		return -EFAULT;
	}

	frame = &framemgr->frames[index];
	(void)memcpy(info, &frame->vbuf.group_info, sizeof(struct vbuf_group_info));

	return 0;
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Query frame buffers information which alloced by driver
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] *info: point to struct vbuf_group_info instance;
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
static s32 vpf_querybuf(const struct vio_video_ctx *vctx, struct vbuf_group_info *info)
{
	s32 ret = 0;

	if ((vctx->state & (BIT((s32)VIO_VIDEO_STOP) | BIT((s32)VIO_VIDEO_REBUFS))) == 0u) {
		vio_err("[%s][C%d] %s: invalid QUERYBUF is requested(0x%llX)", vctx->name,
			vctx->ctx_id, __func__, vctx->state);
		return -EINVAL;
	}

	ret = vpf_subdev_querybuf(vctx->vdev, info);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Query frame buffers which alloced by driver;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_querybuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct vbuf_group_info group_info;

	copy_ret = osal_copy_from_app((void *)&group_info, (void __user *)arg, sizeof(struct vbuf_group_info));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vpf_querybuf(vctx, &group_info);
	if (ret < 0) {
		return -EFAULT;
	}

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&group_info, sizeof(struct vbuf_group_info));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy to user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Set extra configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_s_ctrl(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct vpf_ext_ctrl ext_ctrl;
	struct vpf_device *dev;

	copy_ret = osal_copy_from_app((void *)&ext_ctrl, (void __user *)arg, sizeof(struct vpf_ext_ctrl));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_s_ctrl) {
		ret = dev->vps_ops->video_s_ctrl(vctx, ext_ctrl.id, (unsigned long)ext_ctrl.arg);
		if (ret < 0)
			return ret;
	}
	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Get extra configuration;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_g_ctrl(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct vpf_ext_ctrl ext_ctrl;
	struct vpf_device *dev;

	copy_ret = osal_copy_from_app((void *)&ext_ctrl, (void __user *)arg, sizeof(struct vpf_ext_ctrl));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_g_ctrl) {
		ret = dev->vps_ops->video_g_ctrl(vctx, ext_ctrl.id, (unsigned long)ext_ctrl.arg);
		if (ret < 0)
			return ret;
	}
	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

static s32 vpf_video_dbg_ctrl(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct vpf_ext_ctrl ext_ctrl;

	copy_ret = osal_copy_from_app((void *)&ext_ctrl, (void __user *)arg, sizeof(struct vpf_ext_ctrl));
	if (copy_ret != 0u) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n", vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vio_debug_ctrl(vctx, ext_ctrl.id, (unsigned long)ext_ctrl.arg);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind vio chain automatically or by pipeline id;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_bind_pipeline(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	u32 flow_id = 0;
	struct hobot_vpf_dev *vpf_dev;

	copy_ret = osal_copy_from_app(&flow_id, (void __user *) arg, sizeof(u32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: failed to copy from user, ret = %lld\n",
			vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vpf_dev = vctx->device;
	if ((flow_id >> 8 & PIPELINE_MAGIC_MASK) == PIPELINE_MAGIC_NUM) {
		flow_id = flow_id & 0xff;
		ret = vpf_bind_chain_with_flowid(&vpf_dev->iscore, flow_id);
		if (ret < 0)
			return ret;
	} else {
		ret = vpf_bind_chain(&vpf_dev->iscore, &flow_id);
		if (ret < 0)
			return ret;
	}

	vctx->ctx_id = flow_id;
	vctx->flow_id = flow_id;
	ret = vio_chain_init(&vpf_dev->iscore.vchain[flow_id], flow_id);
	if (ret < 0)
		return ret;

	copy_ret = osal_copy_to_app((void __user *) arg, &flow_id, sizeof(u32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: failed to copy to app, ret = %lld\n",
			vctx->name, __func__, copy_ret);
		return -EFAULT;
	}
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Bind previous vio node output channel to current vio node input channel;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @param[in] arg: variable argument arg depends on cmd to specify the length and type;
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
static s32 vpf_video_bind_vnode(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	u32 bind_flag = 0;

	copy_ret = osal_copy_from_app(&bind_flag, (void __user *) arg, sizeof(u32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: failed to copy from app ret = %lld\n",
			vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	ret = vpf_bind_vnode(vctx, bind_flag);
	vio_info("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

static u32 vpf_get_hw_tasks_cnt(struct vio_group_task * gtask)
{
	u32 task_cnt = 0;

	if (gtask->rcount != 0 ||
		osal_test_bit(VIO_GTASK_SHOT, &gtask->state) != 0)
		task_cnt = gtask->rcount + 1;

	vio_dbg("cur task cnt = %d\n", task_cnt);
	return task_cnt;
}


static s32 vpf_video_get_hw_status(struct vio_video_ctx *vctx, unsigned long arg)
{
	s64 copy_ret;
	u32 status = 0;
	struct vpf_device *dev;

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->video_get_hw_status) {
		status = dev->vps_ops->video_get_hw_status(vctx);
	} else {
		if (dev->gtask != NULL)
			status = vpf_get_hw_tasks_cnt(dev->gtask);
		else {
			vio_err("[%s][C%d] %s: invalid ! gtask is null", vctx->name,
				vctx->ctx_id, __func__);
			return -EINVAL;
		}
	}

	copy_ret = osal_copy_to_app((void __user *) arg, &status, sizeof(u32));
	if (copy_ret != 0) {
		vio_err("[%s] %s: copy_to_user failed, ret(%lld)",
			vctx->name, __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[%s][C%d] %s: status = %d\n", vctx->name, vctx->ctx_id, __func__, status);
	return 0;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Flush all frame from complete queue;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 vpf_flush_com_frame(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	u64 flags = 0;
	osal_list_head_t *done_list;
	struct vio_framemgr *framemgr;
	struct vio_node *vnode;
	struct vio_frame *frame;

	if (vctx->id == VNODE_ID_SRC) {
		vio_info("%s: src node don't need flush", __func__);
		return ret;
	}

	framemgr = vctx->vdev->cur_fmgr;
	vnode = vctx->vdev->vnode;
	done_list = &framemgr->queued_list[FS_COMPLETE];
	vio_dbg("[%s][S%d] %s event = %d\n",
		vctx->name, vnode->flow_id, __func__, vctx->event);/*PRQA S 0685,1294*/

	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	while (osal_list_empty(done_list) == 0) {
		frame = peek_frame(framemgr, FS_COMPLETE);
		if (frame == NULL)
			break;

		vio_dbg("[%s][S%d][F%d] %s: frameid %d\n", vctx->name, vnode->flow_id,
			frame->index, __func__, frame->frameinfo.frameid.frame_id);/*PRQA S 0685,1294*/
		vio_drop_calculate(&vctx->vdev->fdebug, USER_DROP, &frame->frameinfo.frameid);
		trans_frame(framemgr, frame, FS_REQUEST);
		if ((vnode->leader == 1u) && (vctx->vdev->leader == 1u))
			vio_group_start_trigger(vnode, frame);
	}
	vctx->event = 0;
	vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/

	vio_dbg("[%s][C%d] %s: done\n", vctx->name, vctx->ctx_id, __func__);

	return ret;
}

s32 vpf_manager_ioctl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
    s32 ret = 0;

	switch (cmd) {
		case VIO_IOC_BIND_CTX:
			ret = vpf_video_bind_context(vctx, arg);
			break;
		case VIO_IOC_SET_ATTR:
			ret = vpf_video_set_init_attr(vctx, arg);
			break;
		case VIO_IOC_GET_ATTR:
			ret = vpf_video_get_init_attr(vctx, arg);
			break;
		case VIO_IOC_SET_ATTR_EX:
			ret = vpf_video_set_attr_ex(vctx, arg);
			break;
		case VIO_IOC_GET_ATTR_EX:
			ret = vpf_video_get_attr_ex(vctx, arg);
			break;
		case VIO_IOC_SET_ICH_ATTR:
			ret = vpf_video_set_ichn_attr(vctx, arg);
			break;
		case VIO_IOC_GET_ICH_ATTR:
			ret = vpf_video_get_ichn_attr(vctx, arg);
			break;
		case VIO_IOC_SET_OCH_ATTR:
			ret = vpf_video_set_ochn_attr(vctx, arg);
			break;
		case VIO_IOC_GET_OCH_ATTR:
			ret = vpf_video_get_ochn_attr(vctx, arg);
			break;
		case VIO_IOC_QUERYBUF:
			ret = vpf_video_querybuf(vctx, arg);
			break;
		case VIO_IOC_REQBUFS:
			ret = vpf_video_reqbufs(vctx, arg);
			break;
		case VIO_IOC_START:
			ret = vpf_video_start(vctx);
			break;
		case VIO_IOC_STOP:
			ret = vpf_video_stop(vctx);
			break;
		case VIO_IOC_DQBUF:
			ret = vpf_video_dqbuf(vctx, arg);
			break;
		case VIO_IOC_QBUF:
			ret = vpf_video_qbuf(vctx, arg);
			break;
		case VIO_IOC_SET_CTRL:
			ret = vpf_video_s_ctrl(vctx, arg);
			break;
		case VIO_IOC_GET_CTRL:
			ret = vpf_video_g_ctrl(vctx, arg);
			break;
		case VIO_IOC_FLUSH_FRAME:
			ret = vpf_flush_com_frame(vctx);
			break;
		case VIO_IOC_SET_OCH_ATTR_EX:
			ret = vpf_video_set_ochn_attr_ex(vctx, arg);
			break;
		case VIO_IOC_SET_ICH_ATTR_EX:
			ret = vpf_video_set_ichn_attr_ex(vctx, arg);
			break;
		case VIO_IOC_SET_INTER_ATTR:
			ret = vpf_video_set_inter_attr(vctx, arg);
			break;
		case VIO_IOC_GET_VERSION:
			ret = vpf_video_get_version(vctx, arg);
			break;
		case VIO_IOC_GET_STRUCT_SIZE:
			ret = vpf_video_get_struct_size(vctx, arg);
			break;
		case VIO_IOC_DBG_CTRL:
			ret = vpf_video_dbg_ctrl(vctx, arg);
			break;
		/* operation only for flow node */
		case VIO_IOC_BIND_FLOW:
			ret = vpf_video_bind_pipeline(vctx, arg);
			break;
		case VIO_IOC_ADD_NODE:
			ret = vpf_video_add_vnode(vctx, arg);
			break;
		case VIO_IOC_BIND_NODE:
			ret = vpf_video_bind_vnode(vctx, arg);
			break;
		case VIO_IOC_GET_HW_STATUS:
			ret = vpf_video_get_hw_status(vctx, arg);
			break;
		default:
			vio_err("%s wrong command 0x%x\n", __func__, cmd);
			ret = -EFAULT;
			break;
	}

    return ret;
}


/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Close function of /dev/flow;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
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
s32 vpf_subdev_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	u32 flow_id = 0;
	struct hobot_vpf_dev *vpf_dev;
	struct vio_core *iscore;

	vpf_dev = vctx->device;
	iscore = &vpf_dev->iscore;
	flow_id = vctx->ctx_id;

	osal_atomic_dec(&vpf_dev->open_cnt);
	osal_mutex_lock(&iscore->mlock);
	if (flow_id < VIO_MAX_STREAM)
		osal_clear_bit(VIO_CHAIN_OPEN, &iscore->vchain[flow_id].state);
	osal_mutex_unlock(&iscore->mlock);
	vio_info("[%s][S%d] %s: done\n", vctx->name, vctx->flow_id, __func__);

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Open function of /dev/flow;
 * @param[in] *vctx: point to struct vio_video_ctx instance;
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
s32 vpf_subdev_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct hobot_vpf_dev *vpf_dev;

	vpf_dev = vctx->device;
	osal_mutex_lock(&vpf_dev->mlock);
	if (osal_atomic_inc_return(&vpf_dev->open_cnt) == 1) {
	}

	osal_mutex_unlock(&vpf_dev->mlock);

	return ret;
}

s32 vpf_subdev_get_version(struct vio_version_info *version)
{
	s32 ret = 0;

	(void)memcpy(version, &g_vpf_version, sizeof(struct vio_version_info));

	return ret;
}

s32 vpf_device_close(struct vio_video_ctx *vctx)
{
    s32 ret = 0;
	struct vpf_device *dev;

	vpf_vdev_close(vctx->vdev);

	dev = vctx->dev;
	if (dev->vps_ops && dev->vps_ops->close) {
		ret = dev->vps_ops->close(vctx);
		if (ret < 0)
			vio_err("[%s] %s: dev close error(%d)\n", vctx->name, __func__, ret);
	}

	vctx->state = BIT((s32)VIO_VIDEO_CLOSE);
	vpf_vdev_free_resource(vctx->vdev);
	vpf_vctx_unbind_vdev(vctx);
	vpf_free_ctx_id(vctx);
	vio_info("[%s] %s: done\n", vctx->name, __func__);

    return ret;
}

s32 vpf_device_open(struct vio_video_ctx *vctx, struct vpf_device *vpf_device)
{
    s32 ret = 0;

	vctx->id = vpf_device->vid;
	vctx->device = vpf_device->ip_dev;
	vctx->dev = vpf_device;
	vctx->name = vpf_device->name;
	if (vpf_device->vps_ops && vpf_device->vps_ops->open) {
		ret = vpf_device->vps_ops->open(vctx);
		if (ret < 0)
			vio_err("[%s] %s: dev open error(%d)\n", vctx->name, __func__, ret);
	}

	osal_waitqueue_init(&vctx->done_wq);
	vctx->state = BIT(VIO_VIDEO_OPEN);
	vio_info("[%s] %s: done\n", vctx->name, __func__);

    return ret;
}
