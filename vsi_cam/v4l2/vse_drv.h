/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VSE_DRV_H_
#define _VSE_DRV_H_

#include <linux/refcount.h>

#include "utils.h"
#include "vse.h"

struct vse_v4l_instance {
	struct subdev_node node;
	struct vse_device *dev;
	struct cam_ctx sink_ctx;
	struct cam_ctx src_ctx[VSE_OUT_CHNL_MAX];
	struct media_pad *src_pads[VSE_OUT_CHNL_MAX];
	struct cam_format ifmt;
	struct v4l2_fract out_fps[VSE_OUT_CHNL_MAX];
	bool fmt_changed;
	u32 out_pixelformat;
	unsigned long capture_queue_offset;
	u32 id;
	struct mutex open_lock;
	struct mutex fmt_lock;
	struct mutex standalone_lock;
	refcount_t state_count;
	refcount_t open_count;
};

struct vse_v4l_device {
	struct vse_device vse_dev;
	struct vse_v4l_instance *insts;
};

#endif /* _VSE_DRV_H_ */
