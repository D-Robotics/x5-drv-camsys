/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GDC_DRV_H_
#define _GDC_DRV_H_

#include "gdc.h"
#include "utils.h"

struct gdc_v4l_instance {
	struct subdev_node node;
	struct gdc_device *dev;
	struct cam_buf_ctx sink_ctx, src_ctx;
	u32 out_pixelformat;
	u32 id;
	int enabled;
};

struct gdc_v4l_device {
	struct gdc_device gdc_dev;
	struct gdc_v4l_instance *insts;
};

#endif /* _GDC_DRV_H_ */
