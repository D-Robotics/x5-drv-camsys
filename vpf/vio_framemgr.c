/**
 * @file: vio_framemgr.c
 * @
 * @NO{S090E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)    "[VIO fmgr]:" fmt
#include "vio_framemgr.h"
#include "vio_node_api.h"
/**
 * Purpose: frame state name for print
 * Value: NA
 * Range: vio_framemgr.c
 * Attention: NA
 */
static const char * const frame_state_name[NR_FRAME_STATE] = {
	"Free",
	"Request",
	"Process",
	"Complete",
	"Used"
};
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Push frame into state queue of framemgr;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] *frame: point to struct vio_frame instance;
 * @param[in] state: frame state;
 * @retval {*}
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static s32 put_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return -EINVAL;

	if (frame == NULL) {
		vio_err("%s: invalid frame", __func__);
		return -EFAULT;
	}

	frame->state = state;
	osal_list_add_tail(&frame->list, &this->queued_list[state]);
	this->queued_count[state]++;

	return 0;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Pop frame from state queue of framemgr;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] state: frame state;
 * @retval "!NULL": success
 * @retval "NULL": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
struct vio_frame *get_frame(struct vio_framemgr *this, enum vio_frame_state state)
{
	struct vio_frame *frame;

	if (state == FS_INVALID)
		return NULL;

	if (this->queued_count[state] == 0u)
		return NULL;

	frame = osal_list_first_entry(&this->queued_list[state], struct vio_frame, list);/*PRQA S 2810,0497*/
	osal_list_del(&frame->list);
	this->queued_count[state]--;
	frame->state = FS_INVALID;

	return frame;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Transfer the frame info state queue of framemgr;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] *frame: point to struct vio_frame instance;
 * @param[in] state: frame state;
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
s32 trans_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return -EFAULT;
	}

	if (frame == NULL) {
		vio_err("%s: frame is NULL", __func__);
		return -EFAULT;
	}

	if ((frame->state == FS_INVALID) || (state == FS_INVALID))
		return -EINVAL;

	if (this->queued_count[frame->state] == 0u) {
		vio_err("%s: frame queue is empty (0x%08x)", frame_state_name[frame->state], this->id);
		return -EINVAL;
	}

	osal_list_del(&frame->list);
	this->queued_count[frame->state]--;

	return put_frame(this, frame, state);
}
EXPORT_SYMBOL(trans_frame);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Get list head of state queue;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] state: frame state;
 * @retval "!NULL": success
 * @retval "NULL": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
struct vio_frame *peek_frame(const struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return NULL;
	}

	if (state == FS_INVALID)
		return NULL;

	if (this->queued_count[state] == 0u)
		return NULL;

	return osal_list_first_entry(&this->queued_list[state], struct vio_frame, list);/*PRQA S 2810,0497*/
}
EXPORT_SYMBOL(peek_frame);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: get list tail of state queue;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] state: frame state;
 * @retval "!NULL": success
 * @retval "NULL": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
struct vio_frame *peek_frame_tail(const struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return NULL;
	}

	if (state == FS_INVALID)
		return NULL;

	if (this->queued_count[state] == 0u)
		return NULL;

	return osal_list_last_entry(&this->queued_list[state], struct vio_frame, list);/*PRQA S 2810,0497*/
}
EXPORT_SYMBOL(peek_frame_tail);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Find frame with frame count form state queue;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] state: frame state;
 * @param[in] fcount: frame count;
 * @retval "!NULL": success
 * @retval "NULL": failure
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
struct vio_frame *find_frame(const struct vio_framemgr *this,
			     enum vio_frame_state state, u32 fcount)
{
	struct vio_frame *frame;

	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return NULL;
	}

	if (state == FS_INVALID)
		return NULL;

	if (this->queued_count[state] == 0u) {
		vio_err("%s: frame queue is empty (0x%08x)",
			frame_state_name[state], this->id);
		return NULL;
	}

	osal_list_for_each_entry(frame, &this->queued_list[state], list) {/*PRQA S 2810,0497*/
		if (frame->fcount == fcount)
			return frame;
	}

	return NULL;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Initialize vio framemgr and malloc frame;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @param[in] buffers: frame number;
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
s32 frame_manager_open(struct vio_framemgr *this, u32 buffers)
{
	s32 ret = 0;
	u32 i;
	u64 flags = 0;

	osal_spin_init(&this->slock);/*PRQA S 3334*/
	/*
	 * We already have frames allocated, so we should free them first.
	 * reqbufs(n) could be called multiple times from userspace after
	 * each video node was opened.
	 */
	if (this->frames != NULL)
		osal_kfree((void *)this->frames);

	this->frames = (struct vio_frame *)osal_kzalloc(sizeof(struct vio_frame) * buffers, GFP_ATOMIC);
	if (this->frames == NULL) {
		vio_err("%s: failed to allocate frames", __func__);
		return -ENOMEM;
	}

	vio_e_barrier_irqs(this, flags);/*PRQA S 2996*/
	this->num_frames = buffers;
	for (i = 0; i < (u32)NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		osal_list_head_init(&this->queued_list[i]);
	}
	for (i = 0; i < buffers; ++i) {
		this->frames[i].dispatch_cnt = 0;
		this->frames[i].index = i;
		ret = put_frame(this, &this->frames[i], FS_FREE);
		osal_list_head_init(&this->frames[i].work_list);
	}
	vio_x_barrier_irqr(this, flags);/*PRQA S 2996*/

	return ret;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Free frame in vio framemgr instance;
 * @param[in] *this: point to struct vio_framemgr instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void frame_manager_close(struct vio_framemgr *this)
{
	u32 i;
	u64 flags = 0;

	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return;
	}

	vio_e_barrier_irqs(this, flags);/*PRQA S 2996*/
	this->num_frames = 0;
	for (i = 0; i < (u32)NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		osal_list_head_init(&this->queued_list[i]);
	}
	vio_x_barrier_irqr(this, flags);/*PRQA S 2996*/

	if (this->frames != NULL) {
		osal_kfree((void *)this->frames);
		this->frames = NULL;
	}
}
EXPORT_SYMBOL(frame_manager_close);/*PRQA S 0605,0307*/
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Deinitialize vio framemgr;
 * @param[in] *this: point to struct vio_framemgr instance;
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
s32 frame_manager_flush(struct vio_framemgr *this)
{
	u32 i;
	s32 ret = 0;
	u64 flags = 0;
	struct vio_frame *frame, *temp;

	if (this == NULL) {
		vio_err("%s: framemgr is NULL", __func__);
		return -EFAULT;
	}

	vio_e_barrier_irqs(this, flags);/*PRQA S 2996*/
	for (i = (u32)FS_REQUEST; i < (u32)FS_INVALID; i++) {
		osal_list_for_each_entry_safe(frame, temp, &this->queued_list[i], list) {/*PRQA S 2810,2741,0497*/
			frame->dispatch_cnt = 0;
			ret = trans_frame(this, frame, FS_FREE);
			if (ret < 0)
				vio_err("%s: failed\n", __func__);
		}
	}
	vio_x_barrier_irqr(this, flags);/*PRQA S 2996*/

	if (this->queued_count[FS_FREE] != this->num_frames) {
		vio_err("%s: Free count != frame numbers\n", __func__);
		return -EFAULT;
	}

	return ret;
}
EXPORT_SYMBOL(frame_manager_flush);/*PRQA S 0605,0307*/
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Print the queue count of every queue;
 * @param[in] *framemgr: point to struct vio_framemgr instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void framemgr_print_queues(const struct vio_framemgr *framemgr)
{
	if (framemgr != NULL)
		vio_info("[%s] FRM(%s:%d; %s:%d; %s:%d; %s:%d; %s:%d)\n",
			(char *)framemgr->name,
			frame_state_name[FS_FREE], framemgr->queued_count[FS_FREE],
			frame_state_name[FS_REQUEST], framemgr->queued_count[FS_REQUEST],
			frame_state_name[FS_PROCESS], framemgr->queued_count[FS_PROCESS],
			frame_state_name[FS_COMPLETE], framemgr->queued_count[FS_COMPLETE],
			frame_state_name[FS_USED], framemgr->queued_count[FS_USED]);
}
EXPORT_SYMBOL(framemgr_print_queues);/*PRQA S 0605,0307*/

s32 vio_framemgr_share_buf(struct vio_framemgr *src_framemgr,
	struct vio_framemgr *dst_framemgr, void *dst_iommu_dev)
{
	s32 ret = 0;
	s32 i;

	if (src_framemgr == NULL || dst_framemgr == NULL) {
		vio_err("%s: null point, src_framemgr 0x%p, dst_framemgr 0x%p\n",
			__func__, src_framemgr, dst_framemgr);
		return -EFAULT;
	}
	if (src_framemgr->num_frames > dst_framemgr->num_frames) {
		vio_err("%s: src num_frames %d > dst num_frames %d\n", __func__,
			src_framemgr->num_frames, dst_framemgr->num_frames);
		return -EFAULT;
	}

	for (i = 0; i < src_framemgr->num_frames; i++) {
		if (src_framemgr->frames[i].vbuf.ion_alloced == 0)
			continue;
		(void)memcpy(&dst_framemgr->frames[i].frameinfo,
				&src_framemgr->frames[i].frameinfo, sizeof(struct frame_info));
		ret = vio_frame_iommu_map(dst_iommu_dev, &dst_framemgr->frames[i]);
		if (ret < 0)
			return ret;
		dst_framemgr->frames[i].frameinfo.internal_buf = 1;
	}

	return ret;
}
EXPORT_SYMBOL(vio_framemgr_share_buf);
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Map the phyaddr to iommu address;
 * @param[in] *iommu_dev: point to struct device instance;
 * @param[in] *frame: point to struct vio_frame instance;
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
s32 vio_frame_iommu_map(void *iommu_dev,
		struct vio_frame *frame)
{
	s32 ret = 0;

	if (iommu_dev == NULL)
		return ret;

	if (frame->internal_buf == 0 && frame->buf_shared == 0) {
		frame->vbuf.group_info.is_contig = frame->frameinfo.is_contig;//use user configuration
		ret = vio_handle_ext_buffer(&frame->vbuf, frame->frameinfo.ion_id);
		if (ret < 0)
			return -EINVAL;
		frame->buf_shared = 1;
	}

	ret = vio_iommu_map(iommu_dev, &frame->vbuf);
	if (ret < 0) {
		vio_err("%s: vio iommu map failed\n", __func__);
		return -EFAULT;
	}

	vio_dbg("F[%d] %s: done\n", frame->index,  __func__);/*PRQA S 0685,1294*/

	return ret;
}
EXPORT_SYMBOL(vio_frame_iommu_map);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Ummap iommu address;
 * @param[in] *iommu_dev: point to struct device instance;
 * @param[in] *frame: point to struct vio_frame instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_frame_iommu_unmap(void *iommu_dev,
		struct vio_frame *frame)
{
	if (iommu_dev == NULL)
		return;

	vio_iommu_unmap(iommu_dev, &frame->vbuf);

	if (frame->internal_buf == 0)
		vio_ion_free(&frame->vbuf);

	vio_dbg("[F%d] %s: done\n", frame->index, __func__);/*PRQA S 0685,1294*/
}
EXPORT_SYMBOL(vio_frame_iommu_unmap);

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Sync cache data to ddr;
 * @param[in] *frame: point to struct vio_frame instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_frame_sync_for_device(const struct vio_frame *frame)
{
	const struct vio_buffer *vbuf;

	vbuf = &frame->vbuf;
	if (frame->internal_buf != 0u) {
		if (vbuf->ion_alloced == 0u) {
			vio_err("%s: vio buffer not alloc or free\n", __func__);
			return;
		}

		if (vbuf->ion_cached == 0u)
			return;

		vio_ion_sync_for_device(vbuf);
	}
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Sync ddr data to cache;
 * @param[in] *frame: point to struct vio_frame instance;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_frame_sync_for_cpu(const struct vio_frame *frame)
{
	const struct vio_buffer *vbuf;

	vbuf = &frame->vbuf;
	if (frame->internal_buf != 0u) {
		if (vbuf->ion_alloced == 0u) {
			vio_err("%s: vio buffer not alloc or free\n", __func__);
			return;
		}

		if (vbuf->ion_cached == 0u)
			return;

		vio_ion_sync_for_cpu(vbuf);
	}
}
