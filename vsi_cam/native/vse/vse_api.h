/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VSE_API_H_
#define _VSE_API_H_

#include "cam_def.h"

// clang-format off

typedef struct vse_ichn_attr_s {
	cam_bool_e tpg_en;
	__u32 width;
	__u32 height;
	frame_format_e fmt;
	__u32 bit_width;
} vse_ichn_attr_t;

typedef struct vse_attr_s {
	frame_fps_ctrl_t fps;
} vse_attr_t;

typedef struct vse_ochn_attr_s {
	cam_bool_e chn_en;
	rect_t roi;
	__u32 target_w;
	__u32 target_h;
	__u32 y_stride;
	__u32 uv_stride;
	frame_format_e fmt;
	__u32 bit_width;
	frame_fps_ctrl_t fps;
} vse_ochn_attr_t;

typedef struct vse_ochn_attr_ex_s {
	__u32 src_fps;
	__u32 dst_fps;
	__u32 chn_en;
	rect_t roi;
	__u32 target_w;
	__u32 target_h;
} vse_ochn_attr_ex_t;

// clang-format on

#endif /* _VSE_API_H_ */
