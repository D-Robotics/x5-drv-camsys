/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GDC_DRV_H_
#define _GDC_DRV_H_

#include "gdc.h"
#include "utils.h"

#define GDC_INPUT_RES_MAX (5)
#define GDC_FMT_MAX (2)

struct gdc_v4l_instance {
	struct subdev_node node;
	struct gdc_device *dev;
	struct cam_ctx sink_ctx, src_ctx;
	u32 id;
	struct cam_res_cap res_cap;
	struct cam_res_cap input_res;
	struct cam_res_cap input_res_cap[GDC_INPUT_RES_MAX];
	u32 input_res_cap_num;
	u32 fmt_cap[GDC_FMT_MAX]; /* pixelformat */
	u32 fmt_cap_num;
	u32 input_fmt; /* pixelformat */
	int enabled;
	bool m2m_en;
};

struct gdc_v4l_device {
	struct gdc_device gdc_dev;
	struct gdc_v4l_instance *insts;
	struct cam_dev cam_dev;
};

#endif /* _GDC_DRV_H_ */
