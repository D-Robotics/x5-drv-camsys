/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_BUF_H_
#define _CAM_BUF_H_

#include <linux/device.h>

struct cam_ctx;
struct cam_buf;

struct cam_buf_ops {
	int (*queue_setup)(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[]);
};

int cam_reqbufs(struct cam_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops);

int cam_qbuf_irq(struct cam_ctx *ctx, struct cam_buf *buf, bool remote);

struct cam_buf *cam_dqbuf_irq(struct cam_ctx *ctx, bool remote);

struct cam_buf *cam_acqbuf_irq(struct cam_ctx *ctx);

int cam_qbuf(struct cam_ctx *ctx, struct cam_buf *buf);

struct cam_buf *cam_dqbuf(struct cam_ctx *ctx);

int cam_buf_ctx_init(struct cam_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf);

void cam_buf_ctx_release(struct cam_ctx *ctx);

phys_addr_t get_phys_addr(struct cam_buf *buf, unsigned int plane);

void cam_drop(struct cam_ctx *ctx);

#endif /* _CAM_BUF_H_ */
