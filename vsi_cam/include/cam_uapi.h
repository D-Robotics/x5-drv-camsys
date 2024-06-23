/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UAPI_LINUX_CAM_H_
#define _UAPI_LINUX_CAM_H_

#include <linux/types.h>
#include <linux/videodev2.h>

enum cam_state {
	CAM_STATE_INITED = 0,
	CAM_STATE_STOPPED,
	CAM_STATE_STARTED,
	CAM_STATE_CLOSED,
};

enum cam_error {
	CAM_ERROR_WAIT_SRC_BUF = 1,
	CAM_ERROR_WAIT_SINK_BUF = 1 << 1,
};

enum cam_stat_type {
	CAM_STAT_FS,
	CAM_STAT_FE,
};

enum cam_format_type {
	CAM_FMT_NULL,
	CAM_FMT_RAW8,
	CAM_FMT_RAW10,
	CAM_FMT_RAW12,
	CAM_FMT_RAW16,
	CAM_FMT_YUYV,
	CAM_FMT_NV12,
	CAM_FMT_NV16,
	CAM_FMT_RGB888X,
};

enum aligned_mode {
	ALIGNED_OFF,
	ALIGNED_MODE0,
	ALIGNED_MODE1,
};

enum bayer_format {
	BAYER_FMT_BGGR,
	BAYER_FMT_GBRG,
	BAYER_FMT_RGGB,
	BAYER_FMT_GRBG,
};

enum cap_type {
	CAP_DC = 1,
	CAP_SW,
};

struct cam_res_cap {
	__u8 type;
	union {
		struct {
			__u16 width;
			__u16 height;
		} dc;
		struct {
			__u16 min_width;
			__u16 min_height;
			__u16 max_width;
			__u16 max_height;
			__u16 step_width;
			__u16 step_height;
		} sw;
	};
};

struct cam_format_cap {
	__u32 format;
	__u8 index;
	struct cam_res_cap res;
};

enum cam_io_type {
	CAM_OUTPUT,
	CAM_INPUT,
};

struct cam_format {
	__u32 format;
	__u16 width;
	__u16 stride;
	__u16 height;
};

struct cam_rect {
	__u16 x;
	__u16 y;
	__u16 w;
	__u16 h;
};

enum cam_input_type {
	CAM_INPUT_SENSOR,
	CAM_INPUT_IMAGE
};

struct cam_input {
	__u8 index;
	__u8 type;
	union {
		struct {
			char name[100]; /* sensor name */
			__u8 bayer_format;
		} sens;
	};
};

enum cam_work_mode {
	CAM_SIMPLEX_MODE,
	CAM_MULTIPLEX_MODE,
};

struct cam_reg {
	__u32 offset;
	__u32 value;
};

/* clock type definition */
#define CAM_CORE_CLOCK          (0x001)
#define CAM_AXI_CLOCK           (0x002)
#define CAM_APB_CLOCK           (0x003)
#define CAM_CLOCK_EXT_BASE      (0x100)

struct cam_clk {
	__u32 clk; /* clock type */
	__u64 rate;
};

/* log system id */
#define CAM_LOG_MAIN       (0x000)
#define CAM_LOG_USR1       (0x001)
#define CAM_LOG_USR2       (0x002)

/* log priority */
#define CAM_LOG_ERROR      (1 << 0)
#define CAM_LOG_WARN       (1 << 1)
#define CAM_LOG_INFO       (1 << 2)
#define CAM_LOG_DEBUG      (1 << 3)
#define CAM_LOG_VERBOSE    (1 << 4)

struct cam_log {
	__u16 id;    /* log system id */
	__u16 level; /* log priority */
};

/* send */
#define CAM_MSG_READ_REG        (0x1 << 16)
#define CAM_MSG_WRITE_REG       (0x2 << 16)
#define CAM_MSG_CHANGE_INPUT    (0x3 << 16)
#define CAM_MSG_SET_FMT_CAP     (0x4 << 16)
#define CAM_MSG_GET_FORMAT      (0x5 << 16)
#define CAM_MSG_SET_FORMAT      (0x6 << 16)
#define CAM_MSG_GET_STATE       (0x7 << 16)
#define CAM_MSG_SET_STATE       (0x8 << 16)
#define CAM_MSG_GET_CLOCK       (0x9 << 16)
#define CAM_MSG_SET_CLOCK       (0xa << 16)
#define CAM_MSG_RESET_CONTROL   (0xb << 16)
/* recv */
#define CAM_MSG_FORMAT_CHANGED      (0x1 << 24)
#define CAM_MSG_STATE_CHANGED       (0x2 << 24)
#define CAM_MSG_INPUT_CHANGED       (0x3 << 24)
#define CAM_MSG_CTRL_CHANGED        (0x4 << 24)
#define CAM_MSG_CTRL_EXT_CHANGED    (0x5 << 24)
#define CAM_MSG_LOG_STATE_CHANGED   (0x6 << 24)

/* Four-character-code (FOURCC) */
#define cam_fourcc(a, b, c, d) \
	((__u32)(a) | ((__u32)(b) << 8) | ((__u32)(c) << 16) | ((__u32)(d) << 24))

#define VIDIOC_GET_BUF_PHYS      _IOR('V', BASE_VIDIOC_PRIVATE, unsigned int)

#endif /* _UAPI_LINUX_CAM_H_ */
