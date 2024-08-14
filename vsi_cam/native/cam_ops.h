/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_OPS_H_
#define _CAM_OPS_H_

#include <linux/device.h>

struct cam_ctx;
struct cam_ops {
	bool (*is_completed)(struct cam_ctx *ctx);
	int (*trigger)(struct cam_ctx *ctx);
	bool (*osd_update)(struct cam_ctx *ctx);
	int (*osd_set_cfg)(struct cam_ctx *ctx, u32 ochn_id);
	int (*read_hist)(struct cam_ctx *ctx, u32 ochn_id);
	int (*set_mode)(struct cam_ctx *ctx, u32 mode);
};

int add_ops(u32 id, const struct cam_ops *ops);
const struct cam_ops *get_ops(u32 id);

#endif /* _CAM_OPS_H_ */
