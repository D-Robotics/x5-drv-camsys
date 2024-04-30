/**
 * @file: vio_debug_api.h
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

#ifndef VIO_DEBUG_API_H
#define VIO_DEBUG_API_H

/* debug sys */
#define VIO_DBG_GET_FMGR_STATS   0xf500001
#define VIO_DBG_GET_ACTIVE_CTX   0xf500002
#define VIO_DBG_SET_CTX_ID   	 0xf500003
#define VIO_DBG_GET_BUF_NUM		 0xf500004
#define VIO_DBG_CHK_DMA_OUT   	 0xf500005

#define DEBUG_SIZE 1536

#define FS_EVENT 1u
#define FE_EVENT 2u
#define QBUF_EVENT 3u
#define DQBUF_EVENT 4u
#define DROP_EVENT 5u

enum vio_stat_type {
	STAT_FS,
	STAT_FE,
	STAT_QB,
	STAT_DQ,
	STAT_NUM,
};

enum vio_drop_type {
    HW_DROP,
    SW_DROP,
    USER_DROP,
	DROP_TYPE_NUM,
};

struct vio_hw_loading {
	u64 start_timestamp;
	u64 pre_fs_timestamp;
	u64 loading_time;
	u32 loading_value; // xx.xx%
	u32 sampling_time;
	osal_atomic_t enable_loading;
};

#define VPF_LOADING_SHOW_MACRO(_name, _private_struct, _member) \
	static ssize_t _name##_loading_show(struct device *dev, struct device_attribute *attr, char* buf) \
	{ \
		_private_struct *private_dev; \
		struct vio_hw_loading *loading; \
		private_dev = (_private_struct *)dev_get_drvdata(dev); \
		loading = &private_dev->_member; \
		vio_loading_calculate(loading, STAT_QB); \
		msleep(loading->sampling_time); \
		vio_loading_calculate(loading, STAT_DQ); \
		vio_dbg("loading->loading_time = %lld, loading->loading_value = %d, loading->sampling_time = %d\n", \
				loading->loading_time, loading->loading_value, loading->sampling_time); \
		return snprintf(buf, PAGE_SIZE, "loading: %d.%d %%, sampling: %d ms\n", \
				loading->loading_value/100, loading->loading_value%100, loading->sampling_time); \
	}

#define VPF_LOADING_STORE_MACRO(_name, _private_struct, _member) \
	static ssize_t _name##_loading_store(struct device *dev, struct device_attribute *attr, \
									const char* buf, size_t len) \
	{ \
		_private_struct *private_dev; \
		struct vio_hw_loading *loading; \
		private_dev = (_private_struct *)dev_get_drvdata(dev); \
		loading = &private_dev->_member; \
		loading->sampling_time = (u32)simple_strtoul(buf, NULL, 0); \
		vio_dbg("loading->sampling_time = %d\n", loading->sampling_time); \
		return (ssize_t)len; \
	}


/**
 * @struct fps_debug
 * @brief Define the descriptor of fps debug information.
 * @NO{S09E05C01}
 */
struct fps_stats {
	u64 last_time;
	u32 last_frame_count;
	u32 cur_frame_count;
	u32 last_update;
    u64 last_timestamps;
    u64 cur_timestamps;
};

struct frame_delay {
    u32 cur_delay_ns;
    u32 min_delay_ns;
    u32 max_delay_ns;
    u64 avg_delay_ns;
    u32 fcount;
};

struct frame_drop_stats {
    u32 last_drop_id;
	u32 drop_count;
    u64 last_timestamps;
    u64 cur_timestamps;
};

struct id_set {
    u32 flow_id;
    u32 module_id;
    u32 hw_id;
    u32 ctx_id;
    u32 chn_id;
};

struct frame_debug {
    u32 frame_id;
    u32 fcount;
    u64 init_timestamps;
    struct frame_drop_stats drop_stats[DROP_TYPE_NUM];
	struct fps_stats fps;
    struct frame_delay fdelay;
    struct id_set idset;
};
/**
 * @struct statinfo
 * @brief Describes debug information about frame id and timestamp;
 * @NO{S09E05C01}
 */
struct statinfo {
	u32 frameid;
	u64 tv_sec;
	u64 tv_usec;
};

struct module_stat {
	struct statinfo sinfo[STAT_NUM];
};

void vio_fps_calculate(struct frame_debug *fdebug, struct frame_id_desc *frameid);
void vio_set_stat_info(u32 flow_id, u32 module, enum vio_stat_type stype, u32 frameid);
void vio_drop_calculate(struct frame_debug *fdebug, enum vio_drop_type drop_type,
    struct frame_id_desc *frameid);
u32 vio_fps_snapshot(char* buf, u32 size, u32 flowid_mask);
u32 vio_fmgr_stats(char* buf, u32 size, u32 head_index, u32 flowid_mask);
u32 vio_fps_stats(char* buf, u32 size, u32 flowid_mask);
u32 vio_drop_stats(char* buf, u32 size, u32 flowid_mask);
u32 vio_delay_stats(char* buf, u32 size, u32 flowid_mask);
void vio_loading_calculate(struct vio_hw_loading* loading, enum vio_stat_type stype);
#endif//VIO_DEBUG_API_H