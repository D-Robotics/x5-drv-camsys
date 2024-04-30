// SPDX-License-Identifier: GPL-2.0-only
#include "cam_uapi.h"
#include "vio_config.h"

#include "utils.h"

int cam_fmt2vpf_fmt(int cam_fmt)
{
	switch (cam_fmt) {
	case CAM_FMT_RAW8:
		return MEM_PIX_FMT_RAW8;
	case CAM_FMT_RAW10:
		return MEM_PIX_FMT_RAW8;
	case CAM_FMT_RAW12:
		return MEM_PIX_FMT_RAW12;
	case CAM_FMT_NV12:
		return MEM_PIX_FMT_NV12;
	case CAM_FMT_NV16:
		return MEM_PIX_FMT_NV16;
	default:
		vio_err("%s: wrong cam format %d\n", __func__, cam_fmt);
		return -EINVAL;
	}
}
