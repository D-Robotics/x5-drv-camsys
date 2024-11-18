/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GDC_DRV_H_
#define _GDC_DRV_H_

#include "gdc.h"
#include "utils.h"

#define GDC_FMT_MAX (2)

struct gdc_v4l_instance {
	struct subdev_node node;
	struct gdc_device *dev;
	struct cam_ctx sink_ctx, src_ctx;
	u32 id;
	u32 fmt_cap[GDC_FMT_MAX]; /* pixelformat */
	u32 fmt_cap_num;
	u32 input_fmt; /* pixelformat */
	int enabled;
};

struct gdc_v4l_device {
	struct gdc_device gdc_dev;
	struct gdc_v4l_instance *insts;
};

#endif /* _GDC_DRV_H_ */
