/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_DEF_H_
#define _CAM_DEF_H_

// clang-format off

typedef enum enum_cam_bool_e {
	CAM_FALSE = 0,
	CAM_TRUE,
} cam_bool_e;

typedef enum enum_frame_format_e {
	FRM_FMT_NULL,
	FRM_FMT_RAW,
	FRM_FMT_NV12,
	FRM_FMT_UYVY,
} frame_format_e;

typedef struct image_size_s {
	__u32 width;
	__u32 height;
} image_size_t;

typedef struct rect_s {
	__u32 x;
	__u32 y;
	__u32 w;
	__u32 h;
} rect_t;

typedef struct frame_fps_ctrl_s {
	__u16 src;
	__u16 dst;
} frame_fps_ctrl_t;

// clang-format on

#endif /* _CAM_DEF_H_ */
