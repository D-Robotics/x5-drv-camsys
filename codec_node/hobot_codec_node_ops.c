/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "hobot_codec_common.h"
#include "codec_node_config.h"
#include "hobot_codec_node_ops.h"
#include "hobot_dev_codec_node.h"

s32 codec_node_get_attr(struct vio_video_ctx *vctx, struct codec_node_attr_s *codec_node_attr)
{
	s32 ret = 0;
	struct j6_codec_node_dev *codec_node_dev;
	struct codec_node_subdev *subdev;

	codec_node_dev = (struct j6_codec_node_dev *)vctx->device;
	if (vctx->id == VNODE_ID_SRC)
		subdev = &codec_node_dev->subdev[vctx->ctx_id][0];
	else
		subdev = &codec_node_dev->subdev[vctx->ctx_id][1];
	osal_mutex_lock(&codec_node_dev->mlock);
	memcpy(codec_node_attr, &subdev->codec_attr, sizeof(struct codec_node_attr_s));
	osal_mutex_unlock(&codec_node_dev->mlock);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

s32 codec_node_set_attr(struct vio_video_ctx *vctx, struct codec_node_attr_s *codec_node_attr)
{
	s32 ret = 0;
	s32 channel_idx = 0;
	struct j6_codec_node_dev *codec_node_dev;
	struct codec_node_subdev *subdev;

	codec_node_dev = (struct j6_codec_node_dev *)vctx->device;
	if (vctx->id == VNODE_ID_SRC)
		subdev = &codec_node_dev->subdev[vctx->ctx_id][0];
	else
		subdev = &codec_node_dev->subdev[vctx->ctx_id][1];

	osal_mutex_lock(&codec_node_dev->mlock);
	memcpy(&subdev->codec_attr, codec_node_attr, sizeof(struct codec_node_attr_s));
	channel_idx = (uint32_t)find_first_zero_bit(codec_node_dev->channel_idx_bitmap,
					CODEC_MAX_SUBDEV);
	if (channel_idx >= CODEC_MAX_SUBDEV) {
		osal_mutex_unlock(&codec_node_dev->mlock);
		vio_err("err channel_idx: %d\n", channel_idx);
		return -EINVAL;
	}
	set_bit(channel_idx, codec_node_dev->channel_idx_bitmap);
	subdev->codec_attr.channel_idx = channel_idx;
	osal_mutex_unlock(&codec_node_dev->mlock);

	vio_info("vctx->id: %d channel_idx: %d\n", vctx->id, channel_idx);

	return ret;
}

s32 codec_node_set_attr_ex(struct vio_video_ctx *vctx, struct  codec_attr_ex_s*codec_attr_ex)
{
	s32 ret = 0;
	return ret;
}

 /**
  * @NO{S10E01C01}
  * @ASIL{B}
  * @brief: Set codec_node output channel attributes
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
s32 codec_node_set_ochn_attr(struct vio_video_ctx *vctx, struct  codec_ochn_attr_s *ochn_attr)
{
	s32 ret = 0;
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set codec_node input channel attributes
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
s32 codec_node_set_ichn_attr(struct vio_video_ctx *vctx, struct  codec_ichn_attr_s *ichn_attr)
{
	s32 ret = 0;
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief:  Set codec_node output channel buffer attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_buff_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 codec_node_set_ochn_buff_attr(struct vio_video_ctx *vctx,
		struct codec_ochn_buff_attr_s *group_attr)
{
	s32 ret = 0;
	return ret;
}

static s32 get_planecount(s32 format)
{
	s32 plane_count = 2;

	switch (format) {
		case MEM_PIX_FMT_RAW8:
		case MEM_PIX_FMT_RAW10:
		case MEM_PIX_FMT_RAW12:
		case MEM_PIX_FMT_RAW14:
		case MEM_PIX_FMT_RAW16:
		case MEM_PIX_FMT_RAW20:
		case MEM_PIX_FMT_RAW24:
			plane_count = 1;
			break;
		case MEM_PIX_FMT_NV12:
		case MEM_PIX_FMT_NV21:
		case MEM_PIX_FMT_NV16:
		case MEM_PIX_FMT_NV61:
		case MEM_PIX_FMT_NV24:
		case MEM_PIX_FMT_NV42:
			plane_count = 2;
			break;
		default:
			vio_err("%s unsupport format(%d)\n", __func__, format);
			break;
	}

	return plane_count;
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
s32 codec_node_reqbufs(struct vio_video_ctx *vctx, struct vbuf_group_info *group_attr)
{
	s32 ret = 0;
	s32 plane_count = 2;
	struct j6_codec_node_dev *codec_node_dev;
	struct codec_node_subdev *subdev;

	codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	plane_count = get_planecount(MEM_PIX_FMT_NV12);

	if (vctx->id == VNODE_ID_SRC) {
		subdev = &codec_node_dev->subdev[vctx->flow_id][0];
		group_attr->bit_map = BUF_BITMAP_1;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
		group_attr->info[0].buf_attr.planecount = plane_count;
		group_attr->info[0].buf_attr.width = subdev->codec_attr.input_width;
		group_attr->info[0].buf_attr.height = subdev->codec_attr.input_height;
		group_attr->info[0].buf_attr.wstride = subdev->codec_attr.input_stride;
		group_attr->info[0].buf_attr.vstride = subdev->codec_attr.input_height;
		group_attr->is_contig = BUF_CONTIG;
		group_attr->is_alloc = BUF_NO_ALLOC;
	} else {
		subdev = &codec_node_dev->subdev[vctx->flow_id][1];
		group_attr->bit_map = BUF_BITMAP_1;
		group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV12;
		group_attr->info[0].buf_attr.planecount = plane_count;
		group_attr->info[0].buf_attr.width = subdev->codec_attr.output_width;
		group_attr->info[0].buf_attr.height = subdev->codec_attr.output_height;
		group_attr->info[0].buf_attr.wstride = subdev->codec_attr.output_stride;
		group_attr->info[0].buf_attr.vstride = subdev->codec_attr.output_height;
		group_attr->is_contig = BUF_CONTIG;
		group_attr->is_alloc = BUF_NO_ALLOC;
	}

	vio_info("[S] %s done bit_map 0x%x, is_alloc %d, iscontig %d, planecount %d, wxh %dx%d, stride %d \n",
		__func__, group_attr->bit_map, group_attr->is_alloc, group_attr->is_contig,
		group_attr->info[0].buf_attr.planecount,
		group_attr->info[0].buf_attr.width,
		group_attr->info[0].buf_attr.height,
		group_attr->info[0].buf_attr.wstride);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief:Start codec_node from working
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
s32 codec_node_start(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop codec_node from working
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
s32 codec_node_stop(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop codec_node from working
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
static s32 codec_src_dqbuf(
	struct vio_video_ctx *vctx, struct frame_info *frameinfo, int32_t timeout)
{
	s32 ret = 0;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct j6_codec_node_dev *codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	vdev = codec_node_dev->vnode[vctx->ctx_id].ich_subdev[0];
	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (framemgr->queued_count[FS_REQUEST] == 0) {
		ret = wait_event_interruptible_timeout(vdev->vctx[0]->done_wq,
				framemgr->queued_count[FS_REQUEST] > 0, msecs_to_jiffies(timeout));
		if (ret == 0) {
			return -ETIMEDOUT;
		} else if (ret < 0) {
			vio_err("%s:%d, wq failed %d\n", __func__, __LINE__, ret);
			return ret;
		}
	}

	vio_e_barrier_irqs(framemgr, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame != NULL) {
		trans_frame(framemgr, frame, FS_PROCESS);
		memcpy(&vdev->curinfo, &frame->frameinfo, sizeof(struct frame_info));
		vio_x_barrier_irqr(framemgr, flags);
		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
	} else {
		ret = -EINVAL;
		framemgr_print_queues(framemgr);
		vio_x_barrier_irqr(framemgr, flags);
	}

	vio_info("[S%d][%s][V%d] %s index %d internal_buf %d\n", vnode->flow_id, vnode->name, vdev->id,
			__func__, frameinfo->bufferindex, frameinfo->internal_buf);

	return ret;
}

static s32 codec_cap_dqbuf(struct vio_video_ctx *vctx, struct frame_info *frameinfo, int32_t timeout)
{
	s32 ret = 0;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct j6_codec_node_dev *codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	vdev = codec_node_dev->vnode[vctx->ctx_id].och_subdev[0];
	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (framemgr->queued_count[FS_REQUEST] == 0) {
		ret = wait_event_interruptible_timeout(vdev->vctx[0]->done_wq,
				framemgr->queued_count[FS_REQUEST] > 0, msecs_to_jiffies(timeout));
		if (ret == 0) {
			return -ETIMEDOUT;
		} else if (ret < 0) {
			vio_err("%s:%d, wq failed %d\n", __func__, __LINE__, ret);
			return ret;
		}
	}

	vio_e_barrier_irqs(framemgr, flags);
	frame = peek_frame(framemgr, FS_REQUEST);
	if (frame != NULL) {
		trans_frame(framemgr, frame, FS_FREE);
		memcpy(&vdev->curinfo, &frame->frameinfo, sizeof(struct frame_info));
		vio_x_barrier_irqr(framemgr, flags);

		memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
	} else {
		ret = -EINVAL;
		framemgr_print_queues(framemgr);
		vio_x_barrier_irqr(framemgr, flags);
	}

	vio_info("[S%d][%s][V%d] %s index %d internal_buf %d\n",
		vnode->flow_id, vnode->name, vdev->id, __func__, frameinfo->bufferindex, frameinfo->internal_buf);

	return ret;
}

s32 codec_node_dqbuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct frame_info frameinfo;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s]invalid dqbuf is requested(%llX)", __func__, vctx->state);
		return -EFAULT;
	}

	if (vctx->dev->vid == VNODE_ID_SRC) {
		ret = codec_src_dqbuf(vctx, &frameinfo, SRC_DQ_TIMEOUT);
	} else {
		ret = codec_cap_dqbuf(vctx, &frameinfo, CAP_DQ_TIMEOUT);
	}

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&frameinfo, sizeof(struct frame_info));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[S%d][%s][V%d] %s done\n", vctx->flow_id, vctx->name, vctx->id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop codec_node from working
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
s32 codec_src_qbuf(struct vio_subdev *vdev, const struct frame_info *frameinfo)
{
	s32 ret = 0;
	u32 index;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;

	index = (u32)frameinfo->bufferindex;
	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (index >= framemgr->num_frames) {
		vio_err("[S%d][%s][V%d]%s: wrong frame index(%d)\n",
				vnode->flow_id, vnode->name, vdev->id, __func__, index);
		return -EINVAL;
	}

	frame = &framemgr->frames[index];
	if ((frame->state == FS_FREE) || (frame->state == FS_PROCESS)) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		frame->internal_buf = frameinfo->internal_buf;

		vio_e_barrier_irqs(framemgr, flags);
		trans_frame(framemgr, frame, FS_COMPLETE);
		vio_x_barrier_irqr(framemgr, flags);
	} else {
		vio_err("[S%d][%s][V%d] %s:F%d is invalid state(%d)\n",
			vnode->flow_id, vnode->name, vdev->id, __func__, index, frame->state);
		framemgr_print_queues(framemgr);
		return -EINVAL;
	}

	if (vdev->prev != NULL)
		vio_return_buf_to_prev(vdev);

	vio_info("[S%d][%s][V%d] %s index %d internal_buf %d\n", vnode->flow_id, vnode->name,
			vdev->id, __func__, frameinfo->bufferindex, frameinfo->internal_buf);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop codec_node from working
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
s32 codec_cap_qbuf(struct vio_subdev *vdev, const struct frame_info *frameinfo)
{
	s32 ret = 0;
	u32 index;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;

	index = (u32)frameinfo->bufferindex;
	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (index >= framemgr->num_frames) {
		vio_err("[S%d][%s][V%d]%s: wrong frame index(%d)\n",
			vnode->flow_id, vnode->name, vdev->id, __func__, index);
		return -EINVAL;
	}

	frame = &framemgr->frames[index];
	if (frame->state == FS_FREE) {
		memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));
		vio_e_barrier_irqs(framemgr, flags);
		trans_frame(framemgr, frame, FS_COMPLETE);
		vio_x_barrier_irqr(framemgr, flags);
	} else {
		vio_err("[S%d][%s][V%d] %s:F%d is invalid state(%d)\n",
			vnode->flow_id, vnode->name, vdev->id, __func__, index, frame->state);
		framemgr_print_queues(framemgr);
		return -EINVAL;
	}
	if (vdev->next != NULL)
		vio_push_buf_to_next(vdev);

	vio_info("[S%d][%s][V%d] %s index %d internal_buf %d\n", vnode->flow_id, vnode->name,
			vdev->id, __func__, frameinfo->bufferindex, frameinfo->internal_buf);

	return ret;
}

s32 codec_node_qbuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct frame_info frameinfo;
	struct vio_subdev *vdev;
	struct j6_codec_node_dev *codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_warn("[%s]invalid qbuf is requested(%llX)", __func__, vctx->state);
		return -EFAULT;
	}

	copy_ret = osal_copy_from_app((void *)&frameinfo, (void __user *)arg, sizeof(struct frame_info));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	if (vctx->dev->vid == VNODE_ID_SRC) {
		vdev = codec_node_dev->vnode[vctx->ctx_id].ich_subdev[0];
		ret = codec_src_qbuf(vdev, &frameinfo);
	} else {
		vdev = codec_node_dev->vnode[vctx->ctx_id].och_subdev[0];
		ret = codec_cap_qbuf(vdev, &frameinfo);
	}

	vio_info("[S%d][%s][V%d] %s done\n", vctx->flow_id, vctx->name, vctx->id, __func__);

	return ret;
}

s32 codec_node_get_buf_cfg(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0;
	u64 copy_ret;
	struct vio_subdev *vdev;
	u32 num_frames = 0;
	struct j6_codec_node_dev *codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_warn("[%s]invalid dqbuf is requested(%llX)", __func__, vctx->state);
		return -EFAULT;
	}

	if (vctx->dev->vid == VNODE_ID_SRC) {
		vdev = codec_node_dev->vnode[vctx->ctx_id].ich_subdev[0];
		if (vdev->prev != NULL) {
			num_frames = vdev->prev->cur_fmgr->num_frames;
		} else {
			num_frames = vdev->cur_fmgr->num_frames;
		}
	} else {
		vdev = codec_node_dev->vnode[vctx->ctx_id].och_subdev[0];
		num_frames = vdev->cur_fmgr->num_frames;
	}

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&num_frames, sizeof(uint32_t));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[S%d][%s][V%d] %s done\n", vctx->flow_id, vctx->name, vctx->id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop codec_node from working
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
s32 codec_node_bind_flow_id(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret = 0, i = 0;
	s32 ctx_id = -1, channel_idx = -1;
	u64 copy_ret = 0;
	struct j6_codec_node_dev *codec_node_dev;
	struct codec_node_subdev *subdev;
	struct vio_subdev *vdev;
	struct vio_video_ctx *flow_vctx;

	copy_ret = osal_copy_from_app(&channel_idx, (void __user *) arg, sizeof(s32));
	if (copy_ret != 0) {
		vio_err("%s: failed to osal_copy_from_app, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	codec_node_dev = (struct j6_codec_node_dev *)vctx->device;
	osal_mutex_lock(&codec_node_dev->mlock);
	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) != 0) {
		vio_err("[%s]vctx invalid state(%llX)", __func__, vctx->state);
		osal_mutex_unlock(&codec_node_dev->mlock);
		return -EFAULT;
	}

	for (i = 0; i < CODEC_MAX_CHANNEL; i++) {
		subdev = vctx->dev->vid == VNODE_ID_SRC ?
			&codec_node_dev->subdev[i][0] : &codec_node_dev->subdev[i][1];
		if (subdev->codec_attr.channel_idx == channel_idx) {
			ctx_id = i;
			vio_info("find channel_idx: %d,%d, ctx_id: %d\n",
				subdev->codec_attr.channel_idx, channel_idx, ctx_id);
			break;
		}
	}

	if (ctx_id < 0 || ctx_id > CODEC_MAX_CHANNEL || subdev->user_bind == USER_BIND) {
		osal_mutex_unlock(&codec_node_dev->mlock);
		vio_err("%s: error ctx_id(%d) or user_bind(%d)", __func__, ctx_id, subdev->user_bind);
		return -EFAULT;
	}

	if (vctx->dev->vid == VNODE_ID_SRC) {
		vdev = codec_node_dev->vnode[ctx_id].ich_subdev[0];
	} else {
		vdev = codec_node_dev->vnode[ctx_id].och_subdev[0];
	}
	if (osal_test_bit((s32)VIO_SUBDEV_BIND_DONE, &vdev->state) == 0) {
		osal_mutex_unlock(&codec_node_dev->mlock);
		vio_err("[%s]invalid state(%llx)", __func__, vdev->state);
		return -EFAULT;
	}

	flow_vctx = vdev->vctx[0];
	if (flow_vctx == NULL) {
		osal_mutex_unlock(&codec_node_dev->mlock);
		vio_err("[%s]invalid flow_vctx", __func__);
		return -EFAULT;
	}

	if ((flow_vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		osal_mutex_unlock(&codec_node_dev->mlock);
		vio_err("[%s]invalid bind is requested(%llX)", __func__, flow_vctx->state);
		return -EFAULT;
	}

	vctx->ctx_id = ctx_id;
	vctx->framemgr = &vdev->framemgr;
	vctx->state = BIT((s32)VIO_VIDEO_START);
	subdev->user_bind = USER_BIND;
	osal_mutex_unlock(&codec_node_dev->mlock);

	vio_info("%s: ctx_id: %d\n", __func__, ctx_id);

	return ret;
}

s32 codec_node_src_querybuf(const struct vio_video_ctx *vctx, struct vbuf_group_info *info)
{
	s32 ret = 0;
	u32 index;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_subdev *vdev;
	struct j6_codec_node_dev *codec_node_dev = (struct j6_codec_node_dev *)vctx->device;

	if ((vctx->state & BIT((s32)VIO_VIDEO_START)) == 0) {
		vio_err("[%s]invalid src querybuf(%llX)", __func__, vctx->state);
		return -EFAULT;
	}

	vdev = codec_node_dev->vnode[vctx->ctx_id].ich_subdev[0];
	if (vdev->prev == NULL) {
		vio_err("%s vdev->prev %p\n", __func__, vdev->prev);
		return -EFAULT;
	}

	framemgr = vdev->prev->cur_fmgr;
	index = (u32)info->index;
	if (index >= framemgr->num_frames) {
		vio_err("%s wrong index %d, num_frames %d\n", __func__, index, framemgr->num_frames);
		return -EFAULT;
	}

	frame = &framemgr->frames[index];
	memcpy(info, &frame->vbuf.group_info, sizeof(struct vbuf_group_info));

	return ret;
}

s32 codec_node_querybuf(struct vio_video_ctx *vctx, unsigned long arg)
{
	s32 ret;
	u64 copy_ret;
	struct vbuf_group_info group_info;

	copy_ret = osal_copy_from_app((void *)&group_info, (void __user *)arg, sizeof(struct vbuf_group_info));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy from user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	if (vctx->dev->vid == VNODE_ID_SRC) {
		ret = codec_node_src_querybuf(vctx, &group_info);
		if (ret < 0) {
			vio_err("%s: codec_node_src_querybuf failed\n", __func__);
			return -EFAULT;
		}
	} else {
		vio_err("%s: codec node cap donot support\n", __func__);
		return -EFAULT;
	}

	copy_ret = osal_copy_to_app((void __user *)arg, (void *)&group_info, sizeof(struct vbuf_group_info));
	if (copy_ret != 0u) {
		vio_err("%s: failed to copy to user, ret = %lld\n", __func__, copy_ret);
		return -EFAULT;
	}

	vio_info("[%s][C%d][V%d] %s done\n", vctx->name, vctx->ctx_id, vctx->id, __func__);

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
s32 codec_node_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	vctx->state = BIT((s32)VIO_VIDEO_OPEN);
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
s32 codec_node_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct j6_codec_node_dev *codec_node_dev;
	struct codec_node_subdev *subdev;

	codec_node_dev = (struct j6_codec_node_dev *)vctx->device;
	osal_mutex_lock(&codec_node_dev->mlock);
	if (vctx->id == VNODE_ID_SRC)
		subdev = &codec_node_dev->subdev[vctx->ctx_id][0];
	else
		subdev = &codec_node_dev->subdev[vctx->ctx_id][1];
	clear_bit(subdev->codec_attr.channel_idx, codec_node_dev->channel_idx_bitmap);
	subdev->codec_attr.channel_idx = -1;
	subdev->user_bind = 0;
	osal_mutex_unlock(&codec_node_dev->mlock);

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}