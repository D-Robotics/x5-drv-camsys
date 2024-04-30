/**
 * @file: vio_video_api.c
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
#define pr_fmt(fmt)    "[VIO video]:" fmt
#include "vio_node_api.h"
#include "hobot_vpf_manager.h"

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Queue frame buffer into request queue;
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @param[in] *frameinfo: point to struct frame_info instance;
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
s32 vio_subdev_qbuf(struct vio_subdev *vdev, const struct frame_info *frameinfo)
{
	s32 ret = 0;
	u32 index;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;

	if (vdev == NULL || frameinfo == NULL) {
		vio_err("%s: vdev is 0x%p and frameinfo is 0x%p\n", __func__, vdev, frameinfo);
		return -EINVAL;
	}

	index = (u32)frameinfo->bufferindex;
	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;

	if (index >= framemgr->num_frames) {
		vio_err("[%s][S%d] %s: wrong frame index(%d)\n",
			vdev->name, vnode->flow_id, __func__, index);
		return -EINVAL;
	}

	frame = &framemgr->frames[index];
	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	if (frame->dispatch_cnt > 0)
		frame->dispatch_cnt--;

	if (frame->dispatch_cnt > 0u) {
		vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
		return ret;
	}
	vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/

	if ((frame->state == FS_FREE) || (frame->state == FS_USED)) {
		if (frame->frameinfo.ion_id[0] != frameinfo->ion_id[0])
			frame->buf_shared = 0;
		(void)memcpy(&frame->frameinfo, frameinfo, sizeof(struct frame_info));

		if (frame->internal_buf == 0 && frame->buf_shared == 0) {
			vio_frame_iommu_unmap(vdev->iommu_dev, frame);
			ret = vio_frame_iommu_map(vdev->iommu_dev, frame);
			if (ret < 0) {
				vio_err("[%s][S%d][F%d] %s: vio_frame_iommu_map failed\n", vdev->name,
					vnode->flow_id, index, __func__);
				return ret;
			}
		}

		vio_frame_sync_for_device(frame);

		vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
		trans_frame(framemgr, frame, FS_REQUEST);
		vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
	} else {
		vio_err("[%s][S%d][F%d] %s: invalid frame state(%d)\n", vdev->name, vnode->flow_id,
			index, __func__, frame->state);
		framemgr_print_queues(framemgr);
		return -EINVAL;
	}

	if (vdev->vdev_work != NULL)
		vdev->vdev_work(vdev, frame);

	if ((vnode->leader == 1u) && (vdev->leader == 1u))
		vio_group_start_trigger(vnode, frame);

	vio_dbg("[%s][S%d][F%d] %s: internal_buf %d\n",
		vdev->name, vnode->flow_id,/*PRQA S 0685,1294*/
		index, __func__, frame->internal_buf);

	return ret;
}
EXPORT_SYMBOL(vio_subdev_qbuf);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Dequeue frame buffer from complete queue;
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @retval "= 0": failure
 * @retval "> 0": success
 * @param[out] *frameinfo: store frame information;
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vio_subdev_dqbuf(struct vio_subdev *vdev, struct frame_info *frameinfo)
{
	s32 ret = 0;
	s32 index;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;

	if (vdev == NULL || frameinfo == NULL) {
		vio_err("%s: vdev is 0x%p and frameinfo is 0x%p\n", __func__, vdev, frameinfo);
		return -EINVAL;
	}

	framemgr = vdev->cur_fmgr;
	vnode = vdev->vnode;
	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	frame = peek_frame(framemgr, FS_COMPLETE);
	if (frame != NULL) {
		trans_frame(framemgr, frame, FS_USED);
		frame->dispatch_cnt++;
		(void)memcpy(&vdev->curinfo, &frame->frameinfo, sizeof(struct frame_info));
		vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/

		(void)memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
		vio_frame_sync_for_cpu(frame);
	} else {
		index = vdev->curinfo.bufferindex;
		frame = &framemgr->frames[index];
		if (frame->state == FS_USED || frame->state == FS_REQUEST) {
			(void)memcpy(frameinfo, &frame->frameinfo, sizeof(struct frame_info));
			frame->dispatch_cnt++;
			if (frame->state == FS_REQUEST)
				trans_frame(framemgr, frame, FS_USED);
			vio_dbg("[%s][S%d][F%d], state %d, use_cnt %d\n", vdev->name, vnode->flow_id,
					index, frame->state, frame->dispatch_cnt);
		} else {
			framemgr_print_queues(framemgr);
			vio_err("[%s][S%d] %s: Complete queue has no member\n", vdev->name,
				vnode->flow_id, __func__);
			ret = -EINVAL;
		}
		vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
	}

	vio_dbg("[%s][S%d][F%d] %s: internal_buf %d\n",
		vdev->name, vnode->flow_id,/*PRQA S 0685,1294*/
		frameinfo->bufferindex, __func__, frame->internal_buf);

	return ret;
}
EXPORT_SYMBOL(vio_subdev_dqbuf);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Push frame buffer from current ip driver to next ip driver;
 * @param[in] *vdev: point to struct vio_subdev instance;
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
s32 vio_push_buf_to_next(struct vio_subdev *vdev)
{
	s32 ret = 0;
	struct vio_node *vnode;
	struct vio_subdev *next_vdev;
	struct frame_info frameinfo;

	if (vdev == NULL) {
		vio_err("%s: vdev is NULL\n", __func__);
		return -EINVAL;
	}

	next_vdev = vdev->next;
	if (next_vdev != NULL) {
		vnode = next_vdev->vnode;
		if (osal_test_bit((s32)VIO_NODE_START, &vnode->state) != 0) {
			ret = vio_subdev_dqbuf(vdev, &frameinfo);
			ret = vio_subdev_qbuf(next_vdev, &frameinfo);
		}
	}

	return ret;
}
EXPORT_SYMBOL(vio_push_buf_to_next);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Return frame buffer from current ip driver to previous ip driver;
 * @param[in] *vdev: point to struct vio_subdev instance;
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
s32 vio_return_buf_to_prev(struct vio_subdev *vdev)
{
	s32 ret = 0;
	struct vio_node *vnode;
	struct vio_subdev *pre_vdev;
	struct frame_info frameinfo;

	if (vdev == NULL) {
		vio_err("%s: vdev is NULL\n", __func__);
		return -EINVAL;
	}

	pre_vdev = vdev->prev;
	if (pre_vdev != NULL) {
		vnode = pre_vdev->vnode;
		if (osal_test_bit((s32)VIO_NODE_START, &vnode->state) != 0) {
			ret = vio_subdev_dqbuf(vdev, &frameinfo);
			ret = vio_subdev_qbuf(pre_vdev, &frameinfo);
		}
	}

	return ret;
}
EXPORT_SYMBOL(vio_return_buf_to_prev);/*PRQA S 0605,0307*/

static void vio_check_delay_time(struct vio_subdev *vdev, struct frame_id_desc *frameid)
{
	u64 cur_timestamps;
	u32 diff_mtime;

	if (vdev->threshold_time != 0) {
		cur_timestamps = osal_time_get_ns();
		diff_mtime = (cur_timestamps - frameid->timestamps) / 1000000ULL;
		if (diff_mtime > vdev->threshold_time)
			vio_err("[%s]: %s delay time beyond threshold time\n", vdev->name, __func__);
	}
}
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Common frame done function for fe interrupt;
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
void vio_frame_done(struct vio_subdev *vdev)
{
	u32 event = 0;
	u32 i;
	u64 flags = 0;
	void *metadata;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;
	struct vio_video_ctx *vctx;

	if (vdev == NULL) {
		vio_err("%s: vdev is NULL\n", __func__);
		return;
	}

	vnode = vdev->vnode;
	framemgr = vdev->cur_fmgr;
	vio_e_barrier_irqs(framemgr, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame != NULL) {
		(void)memcpy(&frame->frameinfo.frameid, &vnode->frameid, sizeof(struct frame_id_desc));
		(void)memcpy(&frame->frameinfo.crc_value[0], &vdev->crc_value[0],
				sizeof(u32) * VIO_BUFFER_MAX_PLANES);

		if (vdev->pingpong_ring == 1u &&
				vdev->id >= VNODE_ID_CAP &&
				(framemgr->queued_count[FS_REQUEST] + framemgr->queued_count[FS_PROCESS]) <= 1u) {
			vio_drop_calculate(&vdev->fdebug, SW_DROP, &vnode->frameid);
			trans_frame(framemgr, frame, FS_REQUEST);
		} else {
			event = VIO_FRAME_DONE;
			frame->frameinfo.frame_done |= FRAME_DONE;
			trans_frame(framemgr, frame, FS_COMPLETE);
		}
		vio_x_barrier_irqr(framemgr, flags);

		metadata = vio_get_metadata(vnode->flow_id, vnode->frameid.frame_id);
		if (metadata != NULL && frame->vbuf.metadata != NULL)
			memcpy(frame->vbuf.metadata, metadata, METADATA_SIZE);
	} else {
		vio_x_barrier_irqr(framemgr, flags);
		event = VIO_FRAME_NDONE;
		/* code review B12: this print info is used for quickly debug, only used in debug version,
		   it will be removed in release version */
		vio_err("[%s][S%d] %s: PROCESS queue has no member;\n",
			vdev->name, vnode->flow_id, __func__);
		framemgr_print_queues(framemgr);
	}

	if (event == VIO_FRAME_DONE) {
		if (vdev->id >= VNODE_ID_CAP) {
			vio_set_stat_info(vnode->flow_id, vnode->id, STAT_FE,
				vnode->frameid.frame_id);
			vio_fps_calculate(&vdev->fdebug, &vnode->frameid);
			vio_check_delay_time(vdev, &vnode->frameid);
		}

		vio_e_barrier_irqs(vdev, flags);
		for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
			if (osal_test_bit(i, &vdev->val_ctx_mask) != 0) {
				vctx = vdev->vctx[i];
				vctx->event = event;
				osal_wake_up(&vctx->done_wq);
			}
		}
		vio_x_barrier_irqr(vdev, flags);

		if (vdev->next != NULL)
			vio_push_buf_to_next(vdev);

		if (vdev->prev != NULL)
			vio_return_buf_to_prev(vdev);
	}

	osal_clear_bit(VIO_NODE_SHOT, &vnode->state);
	osal_clear_bit(VIO_GTASK_SHOT, &vnode->gtask->state);
	if ((event == 0) && (vnode->leader == 1u) && (vdev->leader == 1u))
		vio_group_start_trigger(vnode, frame);

	vio_dbg("[%s][S%d] %s: event = %d", vdev->name, vnode->flow_id, __func__, event);
}
EXPORT_SYMBOL(vio_frame_done);

/* code review E1: internal logic function, no need error return */
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Common frame drop function;
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
void vio_frame_ndone(struct vio_subdev *vdev)
{
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;

	if (vdev == NULL) {
		vio_err("%s: vdev is NULL\n", __func__);
		return;
	}

	vnode = vdev->vnode;
	framemgr = vdev->cur_fmgr;
	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame != NULL) {
		trans_frame(framemgr, frame, FS_REQUEST);
		if ((vnode->leader == 1u) && (vdev->leader == 1u))
			vio_group_start_trigger(vnode, frame);
	} else {
		/* code review B12: this print info is used for quickly debug, only used in debug version,
		   it will be removed in release version */
		vio_err("[%s][S%d] %s: PROCESS queue has no member;\n",
			vdev->name, vnode->flow_id, __func__);
		framemgr_print_queues(framemgr);
	}
	vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/
	osal_clear_bit(VIO_NODE_SHOT, &vnode->state);
	osal_clear_bit(VIO_GTASK_SHOT, &vnode->gtask->state);
	vio_drop_calculate(&vdev->fdebug, HW_DROP, &vnode->frameid);
	vio_dbg("[%s][S%d] %s: done", vdev->name, vnode->flow_id, __func__);
}
EXPORT_SYMBOL(vio_frame_ndone);

/* code review E1: internal logic function, no need error return */
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Common frame predone function of water level interrupt;
 * @param[in] *vdev: point to struct vio_subdev instance;
 * @param[in] pre: water level interrupt number;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_frame_predone(struct vio_subdev *vdev, u32 pre)
{
	u32 event = 0;
	u32 i;
	u64 flags = 0;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	struct vio_node *vnode;
	struct vio_video_ctx *vctx;

	if (vdev == NULL) {
		vio_err("%s: vdev is NULL\n", __func__);
		return;
	}

	vnode = vdev->vnode;
	framemgr = vdev->cur_fmgr;
	vio_e_barrier_irqs(framemgr, flags);/*PRQA S 2996*/
	frame = peek_frame(framemgr, FS_PROCESS);
	if (frame != NULL) {
		memcpy(&frame->frameinfo.frameid, &vnode->frameid, sizeof(struct frame_id_desc));
		frame->frameinfo.frame_done = pre;
		(void)memcpy((void *)&vdev->curinfo, (void *)&frame->frameinfo, sizeof(frame->frameinfo));
		event = (u32)VIO_FRAME_PREINT;

		if (vdev->pingpong_ring == 1u &&
            (framemgr->queued_count[FS_REQUEST] + framemgr->queued_count[FS_PROCESS]) <= 2u)
            event = 0;
	} else {
		event = (u32)VIO_FRAME_NDONE;
	}
	vio_x_barrier_irqr(framemgr, flags);/*PRQA S 2996*/

	if (event == (u32)VIO_FRAME_NDONE) {
		vio_err("[%s][S%d] %s PROCESS queue has no member;\n", vdev->name, vnode->flow_id, __func__);
		framemgr_print_queues(framemgr);
	}

	if (event == VIO_FRAME_PREINT) {
		vio_e_barrier_irqs(vdev, flags);
		for (i = 0; i < VIO_MAX_SUB_PROCESS; i++) {
			if (osal_test_bit(i, &vdev->val_ctx_mask) != 0) {
				vctx = vdev->vctx[i];
				vctx->event = event;
				osal_wake_up(&vctx->done_wq);
			}
		}
		vio_x_barrier_irqr(vdev, flags);
	}

	vio_dbg("[%s][S%d] %s: done", vdev->name, vnode->flow_id, __func__);
}
EXPORT_SYMBOL(vio_frame_predone);

s32 vio_hw_format_cov_hbmem_format(u32 hw_format)
{
	s32 hbmem_format;

	switch (hw_format) {
		case HW_FORMAT_YUV420_SHIFT_8BIT:
		case HW_FORMAT_YUV420_SHIFT_10BIT:
		case HW_FORMAT_YUV420_LEG_8BIT:
		case HW_FORMAT_YUV420_8BIT:
		case HW_FORMAT_YUV420_10BIT:
			hbmem_format = MEM_PIX_FMT_NV12;
			break;
		case HW_FORMAT_RAW8:
			hbmem_format = MEM_PIX_FMT_RAW8;
			break;
		case HW_FORMAT_RAW10:
			hbmem_format = MEM_PIX_FMT_RAW10;
			break;
		case HW_FORMAT_RAW12:
			hbmem_format = MEM_PIX_FMT_RAW12;
			break;
		case HW_FORMAT_RAW14:
			hbmem_format = MEM_PIX_FMT_RAW14;
			break;
		case HW_FORMAT_RAW16:
			hbmem_format = MEM_PIX_FMT_RAW16;
			break;
		case HW_FORMAT_RAW20:
			hbmem_format = MEM_PIX_FMT_RAW20;
			break;
		case HW_FORMAT_RAW24:
			hbmem_format = MEM_PIX_FMT_RAW24;
			break;
		default:
			hbmem_format = MEM_PIX_FMT_TOTAL;
		break;
	}

	return hbmem_format;
}
EXPORT_SYMBOL(vio_hw_format_cov_hbmem_format);