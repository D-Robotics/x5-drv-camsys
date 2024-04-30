/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_ONLINE_OPS_H_
#define _CAM_ONLINE_OPS_H_

#include <linux/device.h>

struct cam_buf_ctx;
struct cam_online_ops {
	bool (*is_completed)(struct cam_buf_ctx *ctx);
	int (*trigger)(struct cam_buf_ctx *ctx);
};

int add_online_ops(u32 id, const struct cam_online_ops *ops);
const struct cam_online_ops *get_online_ops(u32 id);

#endif /* _CAM_ONLINE_OPS_H_ */
