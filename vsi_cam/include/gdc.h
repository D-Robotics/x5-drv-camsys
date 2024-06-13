/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _GDC_H_
#define _GDC_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include "gdc_uapi.h"
#include "job_queue.h"
#include "mem_helper.h"

#define GDC_FMT_MAX (2)
#define GDC_RES_MAX (2)

#define gdc_write(gdc, offset, value) \
	__raw_writel(value, (gdc)->base + (offset))

#define gdc_read(gdc, offset) __raw_readl((gdc)->base + (offset))

struct gdc_irq_ctx {
	struct cam_buf *sink_buf, *src_buf;
	struct cam_ctx *sink_ctx, *src_ctx;
};

struct gdc_format_cap {
	u32 format;
	struct cam_res_cap res[GDC_RES_MAX];
};

struct gdc_instance {
	spinlock_t lock; /* lock for handling ctx */
	struct gdc_irq_ctx ctx;
	struct gdc_format_cap fmt_cap[GDC_FMT_MAX];
	struct gdc_format fmt;
	struct mem_buf cfg_buf;
	enum cam_state state;
	enum cam_error error;
};

struct gdc_device {
	u32 id, num_insts;
	struct device *dev;
	void __iomem *base;
	struct clk *core, *axi, *hclk;
	struct reset_control *rst;
	struct isc_handle *isc;
	spinlock_t isc_lock; /* lock for sending msg */
	struct cam_ctrl_device *ctrl_dev;
	struct job_queue *jq; /* offline job queue */
	struct gdc_instance *insts;
	struct list_head in_buf_list;
	struct mem_buf in_bufs;
	u32 next_irq_ctx;
	enum cam_error error;
};

void gdc_post(struct gdc_device *dev, void *msg, u32 len);
void set_ibuffer(struct gdc_device *gdc, struct cam_format *fmt,
		 struct mem_buf *cfg, phys_addr_t buf);
void set_obuffer(struct gdc_device *gdc, struct cam_format *fmt, phys_addr_t buf);
void gdc_start(struct gdc_device *gdc);
void gdc_stop(struct gdc_device *gdc);
int gdc_set_format(struct gdc_device *dev, u32 inst, struct gdc_format *fmt);
int gdc_set_state(struct gdc_device *dev, u32 inst, int enable);
int gdc_set_ctx(struct gdc_device *dev, u32 inst, struct gdc_irq_ctx *ctx);
int gdc_add_job(struct gdc_device *dev, u32 inst);
int gdc_probe(struct platform_device *pdev, struct gdc_device *dev);
int gdc_remove(struct platform_device *pdev, struct gdc_device *dev);

#ifdef CONFIG_PM_SLEEP
int gdc_system_suspend(struct device *dev);
int gdc_system_resume(struct device *dev);
#endif
#ifdef CONFIG_PM
int gdc_runtime_suspend(struct device *dev);
int gdc_runtime_resume(struct device *dev);
#endif

s32 gdc_msg_handler(void *msg, u32 len, void *arg);
int new_frame(struct gdc_irq_ctx *ctx);
struct gdc_irq_ctx *get_next_irq_ctx(struct gdc_device *gdc);
irqreturn_t gdc_irq_handler(int irq, void *arg);

#endif /* _GDC_H_ */
