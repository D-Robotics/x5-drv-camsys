/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_COM_H_
#define _CAM_COM_H_

#include <linux/device.h>

#include "cam_buf.h"

int cam_trigger(struct cam_ctx *ctx);

bool cam_is_completed(struct cam_ctx *ctx);

int cam_ctx_init(struct cam_ctx *ctx, struct device *dev, void *data,
		  bool has_internal_buf);

void cam_ctx_release(struct cam_ctx *ctx);

void sif_set_frame_des(struct cam_ctx *ctx, void *data);

void sif_get_frame_des(struct cam_ctx *ctx);

void cam_set_stat_info(struct cam_ctx *ctx, u32 type);

bool cam_osd_update(struct cam_ctx *ctx);

int cam_osd_set_cfg(struct cam_ctx *ctx, u32 ochn_id);

int cam_next_set_mode(struct cam_ctx *ctx, u32 mode);

#endif /* _CAM_COM_H_ */
