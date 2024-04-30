/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ISP_DRV_H_
#define _ISP_DRV_H_

#include "isp.h"
#include "utils.h"

struct isp_v4l_instance {
	struct subdev_node node;
	struct isp_device *dev;
	struct cam_buf_ctx sink_ctx, src_ctx;
	u32 out_pixelformat;
	u32 id;
	int enabled;
};

struct isp_v4l_device {
	struct isp_device isp_dev;
	struct isp_v4l_instance *insts;
};

#endif /* _ISP_DRV_H_ */
