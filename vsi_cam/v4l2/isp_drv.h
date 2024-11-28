/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ISP_DRV_H_
#define _ISP_DRV_H_

#include <linux/refcount.h>

#include "isp.h"
#include "utils.h"

#define ISP_IN_CHNL_MAX (SINK_PADS_MAX)
#define ISP_INPUT_FMT_MAX (15)
#define ISP_FMT_MAX (3)

struct isp_v4l_instance {
	struct subdev_node node;
	struct isp_device *dev;
	struct cam_ctx sink_ctx[ISP_IN_CHNL_MAX], src_ctx[ISP_OUT_CHNL_MAX];
	struct media_pad *src_pads[ISP_OUT_CHNL_MAX];
	struct mutex open_lock;
	struct mutex fmt_lock;
	u32 id;
	int enabled;
	bool fmt_changed;
	struct isp_format fmt;
	u32 input_fmt_cap[ISP_INPUT_FMT_MAX]; /* pixelformat */
	u32 input_fmt_cap_num;
	u32 fmt_cap[ISP_FMT_MAX]; /* pixelformat */
	u32 fmt_cap_num;
	u32 input_fmt; /* pixelformat */
	refcount_t start_count;
	refcount_t open_count;
	bool metadata_en;
	bool hdr_en;
};

struct isp_v4l_device {
	struct isp_device isp_dev;
	struct isp_v4l_instance *insts;
};

#endif /* _ISP_DRV_H_ */
