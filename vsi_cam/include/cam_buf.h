/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_BUF_H_
#define _CAM_BUF_H_

#include <linux/device.h>

struct cam_buf_ctx;
struct cam_buf;

struct cam_buf_ops {
	int (*queue_setup)(struct cam_buf_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[]);
};

int cam_reqbufs(struct cam_buf_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops);

int cam_qbuf_irq(struct cam_buf_ctx *ctx, struct cam_buf *buf, bool remote);

struct cam_buf *cam_dqbuf_irq(struct cam_buf_ctx *ctx, bool remote);

struct cam_buf *cam_acqbuf_irq(struct cam_buf_ctx *ctx);

int cam_trigger(struct cam_buf_ctx *ctx);

bool cam_is_completed(struct cam_buf_ctx *ctx);

int cam_qbuf(struct cam_buf_ctx *ctx, struct cam_buf *buf);

struct cam_buf *cam_dqbuf(struct cam_buf_ctx *ctx);

int cam_buf_ctx_init(struct cam_buf_ctx *ctx, struct device *dev, void *data,
		     bool has_internal_buf);

void cam_buf_ctx_release(struct cam_buf_ctx *ctx);

phys_addr_t get_phys_addr(struct cam_buf *buf, unsigned int plane);

void sif_set_frame_des(struct cam_buf_ctx *buf_ctx, void *data);

void sif_get_frame_des(struct cam_buf_ctx *buf_ctx);

void cam_drop(struct cam_buf_ctx *buf_ctx);

void cam_set_stat_info(struct cam_buf_ctx *buf_ctx, u32 type);

bool cam_osd_update(struct cam_buf_ctx *buf_ctx);

int cam_osd_set_cfg(struct cam_buf_ctx *buf_ctx, u32 ochn_id);

#endif /* _CAM_BUF_H_ */
