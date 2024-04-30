/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ISP_API_H_
#define _ISP_API_H_

#include "cam_def.h"

// clang-format off

typedef enum enum_mcm_sched_mode_e {
	ROUND_ROBIN = 0,
	FIXED_SEQUENCE,
	FIFO,
} sched_mode_e;

typedef enum enum_input_mode_e {
	PASSTHROUGH_MODE = 0,
	MCM_MODE,
	DDR_MODE,
} input_mode_e;

typedef enum isp_sensor_mode_e {
	ISP_NORMAL_M = 0,
	ISP_DOL2_M = 1,
	ISP_PWL_M = 2,
	ISP_INVALID_MOD,
} isp_sensor_mode_t;

typedef struct isp_attr_s {
	__u32 input_mode;
	__u32 sched_mode;
	isp_sensor_mode_t sensor_mode;
	rect_t crop;
} isp_attr_t;

typedef struct isp_ochn_attr_s {
	cam_bool_e ddr_en;
	rect_t out;
	frame_format_e fmt;
	__u32 bit_width;
} isp_ochn_attr_t;

typedef struct isp_ichn_attr_s {
	cam_bool_e tpg_en;
	__u32 width;
	__u32 height;
	frame_format_e fmt;
	__u32 bit_width;
} isp_ichn_attr_t;

// clang-format on

#endif /* _ISP_API_H_ */
