/**
 * @file: vio_framemgr.h
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

#ifndef VIO_FRAME_MGR_H
#define VIO_FRAME_MGR_H

#include "osal.h"
#include "vio_config.h"
#include "vio_mem.h"

#define VIO_MAX_FRAMES 16u
#define VIO_MAX_SUB_PROCESS	8u

/**
 * @enum vio_frame_state
 * Describe frame state
 */
enum vio_frame_state {
	FS_FREE,
	FS_REQUEST,
	FS_PROCESS,
	FS_COMPLETE,
	FS_USED,
	FS_INVALID
};

#define NR_FRAME_STATE FS_INVALID

#define TRACE_ID		(1u)

/**
 * @struct frame_id_desc
 * @brief Define the descriptor of frame id and timestamp.
 * @NO{S09E05C01}
 */
struct frame_id_desc {
	u32 frame_id; //frame count
	u64 timestamps;//readout system count
	u64 tv_sec;// readout system time (unit:s)
	u64 tv_usec;//readout system time (unit:us)
	u64 trig_tv_sec;// lpwm trigger system time (unit:s)
	u64 trig_tv_usec;//lpwm trigger system time (unit:us)
};

/**
 * @struct frame_info
 * @brief Define the descriptor of frame information to transfer information between IP drivers.
 * @NO{S09E05C01}
 */
struct frame_info {
	struct frame_id_desc frameid;
	u32 frame_done;
	s32 bufferindex;
	u32 num_planes;
	u32 internal_buf;
	u32 is_contig;
	s32 ion_id[VIO_BUFFER_MAX_PLANES];
	u32 addr_offset[VIO_BUFFER_MAX_PLANES];
	u32 crc_value[VIO_BUFFER_MAX_PLANES];
};

/**
 * @struct vio_frame
 * @brief Define the descriptor of frame.
 * @NO{S09E05C01}
 */
struct vio_frame {
	osal_list_head_t list;
	osal_list_head_t work_list;
	u8 work_queued;

	void *vnode;
	/* common use */
	struct vio_buffer vbuf;
	struct frame_info frameinfo;
	enum vio_frame_state state;

	u32	flow_id; /* device instance */
	u32	fcount;
	u32 index;

	u16	dispatch_cnt;
	u8 internal_buf;
	u8 buf_shared;
	void *ext_data;
};

/**
 * @struct vio_framemgr
 * @brief Define the descriptor of frame manager.
 * @NO{S09E05C01}
 */
struct vio_framemgr {
	void *name;
	u32	id;
	osal_spinlock_t slock;

	u32 sindex;
	u32	num_frames;
	struct vio_frame *frames;

	u32	queued_count[NR_FRAME_STATE];
	osal_list_head_t queued_list[NR_FRAME_STATE];
};

s32 trans_frame(struct vio_framemgr *this, struct vio_frame *frame,
		enum vio_frame_state state);
struct vio_frame *peek_frame(const struct vio_framemgr *this,
		enum vio_frame_state state);
struct vio_frame *peek_frame_tail(const struct vio_framemgr *this,
		enum vio_frame_state state);
struct vio_frame *find_frame(const struct vio_framemgr *this,
		enum vio_frame_state state, u32 fcount);
s32 frame_manager_open(struct vio_framemgr *this, u32 buffers);
void frame_manager_close(struct vio_framemgr *this);
s32 frame_manager_flush(struct vio_framemgr *this);
void framemgr_print_queues(const struct vio_framemgr *this);
s32 vio_frame_iommu_map(void *iommu_dev,
		struct vio_frame *frame);
void vio_frame_iommu_unmap(void *iommu_dev,
		struct vio_frame *frame);
void vio_frame_sync_for_device(const struct vio_frame *frame);
void vio_frame_sync_for_cpu(const struct vio_frame *frame);
s32 vio_framemgr_share_buf(struct vio_framemgr *src_framemgr,
	struct vio_framemgr *dts_framemgr, void *dst_iommu_dev);
#endif
