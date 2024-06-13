/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VSE_DRV_H_
#define _VSE_DRV_H_

#include "utils.h"
#include "vse.h"

struct vse_v4l_instance {
	struct subdev_node node;
	struct vse_device *dev;
	struct cam_ctx sink_ctx;
	struct cam_ctx src_ctx[VSE_OUT_CHNL_MAX];
	struct media_pad *src_pads[VSE_OUT_CHNL_MAX];
	bool is_out_chnl_connected[VSE_OUT_CHNL_MAX];
	struct vse_format ifmt;
	u32 set_state_count;
	u32 out_pixelformat;
	u32 id;
};

struct vse_v4l_device {
	struct vse_device vse_dev;
	struct vse_v4l_instance *insts;
};

#endif /* _VSE_DRV_H_ */
