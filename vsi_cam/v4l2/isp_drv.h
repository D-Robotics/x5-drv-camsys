/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ISP_DRV_H_
#define _ISP_DRV_H_

#include <linux/refcount.h>

#include "isp.h"
#include "utils.h"

struct isp_v4l_instance {
	struct subdev_node node;
	struct isp_device *dev;
	struct cam_ctx sink_ctx, src_ctx[ISP_OUT_CHNL_MAX];
	struct media_pad *src_pads[ISP_OUT_CHNL_MAX];
	struct v4l2_fract out_fps;
	struct mutex open_lock;
	struct mutex fmt_lock;
	u32 out_pixelformat;
	u32 id;
	int enabled;
	bool fmt_changed;
	struct isp_format fmt;
	refcount_t start_count;
	refcount_t open_count;
};

struct isp_v4l_device {
	struct isp_device isp_dev;
	struct isp_v4l_instance *insts;
};

#endif /* _ISP_DRV_H_ */
