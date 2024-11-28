/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VSE_DRV_H_
#define _VSE_DRV_H_

#include <linux/refcount.h>

#include "utils.h"
#include "vse.h"

#define VSE_INPUT_RES_MAX (10)
#define VSE_FMT_MAX (5)
#define VSE_INPUT_FMT_MAX (5)

struct vse_v4l_instance {
	struct subdev_node node;
	struct vse_device *dev;
	struct cam_ctx sink_ctx;
	struct cam_ctx src_ctx[VSE_OUT_CHNL_MAX];
	struct media_pad *src_pads[VSE_OUT_CHNL_MAX];
	struct cam_format ifmt;
	bool fmt_changed;
	unsigned long capture_queue_offset;
	u32 id;
	struct cam_res_cap res_cap[VSE_OUT_CHNL_MAX];
	struct cam_res_cap input_res_cap[VSE_INPUT_RES_MAX];
	struct cam_res_cap input_res;
	u32 input_res_cap_num;
	u32 fmt_cap[VSE_FMT_MAX]; /* pixelformat */
	u32 fmt_cap_num;
	u32 input_fmt;
	u32 input_fmt_cap[VSE_INPUT_FMT_MAX]; /* pixelformat */
	u32 input_fmt_cap_num;
	struct mutex open_lock;
	struct mutex fmt_lock;
	struct mutex ctx_lock;
	refcount_t state_count;
	refcount_t open_count;
	bool m2m_en;
};

struct vse_v4l_device {
	struct vse_device vse_dev;
	struct vse_v4l_instance *insts;
};

#endif /* _VSE_DRV_H_ */
