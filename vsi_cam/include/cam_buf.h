/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_BUF_H_
#define _CAM_BUF_H_

#include <linux/device.h>

struct cam_ctx;
struct cam_buf;
struct cam_frame_info;

struct cam_buf_ops {
	int (*queue_setup)(struct cam_ctx *ctx,
			   unsigned int *num_buffers, unsigned int *num_planes,
			   unsigned int sizes[], struct device *alloc_devs[]);
};

__weak
int cam_reqbufs(struct cam_ctx *ctx, unsigned int num,
		struct cam_buf_ops *ops);

int cam_qbuf_irq(struct cam_ctx *ctx, struct cam_buf *buf, bool remote);

struct cam_buf *cam_dqbuf_irq(struct cam_ctx *ctx, bool remote);

struct cam_buf *cam_acqbuf_irq(struct cam_ctx *ctx, bool remote);

__weak
int cam_qbuf(struct cam_ctx *ctx, struct cam_buf *buf);

__weak
struct cam_buf *cam_dqbuf(struct cam_ctx *ctx);

__weak
struct cam_buf *cam_acqbuf(struct cam_ctx *ctx);

struct init_attr;
__weak
int cam_buf_ctx_init(struct cam_ctx *ctx, void *data, struct init_attr *attr);

__weak
void cam_buf_ctx_release(struct cam_ctx *ctx);

struct cam_dev;
phys_addr_t get_phys_addr(struct cam_dev *dev, struct cam_buf *buf, unsigned int plane);

unsigned long get_buf_size(struct cam_buf *buf, unsigned int plane);

int cam_drop_irq(struct cam_ctx *ctx, struct cam_buf *buf);

int cam_drop_irq_ext(struct cam_ctx *ctx, struct cam_buf *buf);

int cam_drop(struct cam_ctx *ctx, struct cam_buf *buf);

__weak
int cam_ready(struct cam_ctx *ctx, int on);

int cam_iommu_unmap(struct cam_dev *dev);

#endif /* _CAM_BUF_H_ */
