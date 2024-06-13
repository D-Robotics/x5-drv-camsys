/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VSE_H_
#define _VSE_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include "cam_uapi.h"
#include "job_queue.h"
#include "vse_uapi.h"

#define VSE_FMT_MAX (2)
#define VSE_RES_MAX (2)

#define vse_write(vse, offset, value) \
	__raw_writel(value, (vse)->base + (offset))

#define vse_read(vse, offset) \
	__raw_readl((vse)->base + (offset))

struct vse_stitching {
	bool enabled;
	u8 hfactor, vfactor;
	u8 left_top;
	u8 x, y;
};

struct vse_irq_ctx {
	bool is_sink_online_mode;
	struct vse_stitching stitches[VSE_OUT_CHNL_MAX];
	struct cam_buf *sink_buf, *src_buf[VSE_OUT_CHNL_MAX];
	struct cam_buf_ctx *sink_ctx, *src_ctx[VSE_OUT_CHNL_MAX], *stat_ctx;
	struct vse_fps_rate *fps;
};

struct vse_format_cap {
	u32 format;
	struct cam_res_cap res[VSE_OUT_CHNL_MAX][VSE_RES_MAX];
};

struct vse_fps_rate {
	u16 src;
	u16 dst;
	u16 cnt;
};

struct vse_instance {
	spinlock_t lock; /* lock for handling ctx */
	struct vse_irq_ctx ctx;
	struct vse_format_cap fmt_cap[VSE_FMT_MAX];
	struct cam_format ifmt, ofmt[VSE_OUT_CHNL_MAX];
	struct cam_rect crop[VSE_OUT_CHNL_MAX];
	struct vse_cmd_buf *cmd_buf_va;
	struct mem_buf cmd_buf;
	struct mem_buf osd[VSE_OUT_CHNL_MAX][4];
	struct vse_mcm_sch sch;
	struct vse_fps_rate fps[VSE_OUT_CHNL_MAX];
	enum vse_src source;
	enum cam_state state;
	enum cam_error error;
};

struct vse_device {
	u32 id, num_insts;
	struct device *dev;
	void __iomem *base;
	struct clk *core, *axi, *ups, *gdc_axi, *gdc_hclk;
	struct reset_control *rst;
	struct isc_handle *isc;
	spinlock_t isc_lock; /* lock for sending msg */
	struct cam_ctrl_device *ctrl_dev;
	struct job_queue *jq; /* offline job queue */
	struct vse_instance *insts;
	u32 next_irq_ctx;
	enum cam_error error;
	bool is_completed;
	struct mutex open_lock; /* lock for open_cnt */
	refcount_t open_cnt;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_file;
#endif
};

void vse_set_rdma_buffer(struct vse_device *vse, phys_addr_t phys_addr, struct cam_format *fmt);
void vse_set_stitch_buffer(struct vse_device *vse, u32 chnl, struct vse_stitching *stitch,
			   phys_addr_t phys_addr, struct cam_format *fmt);
void vse_set_mi_buffer(struct vse_device *vse, u32 chnl,
		       phys_addr_t phys_addr, struct cam_format *fmt);
void vse_set_cmd(struct vse_device *vse, u32 inst);
int vse_post(struct vse_device *vse, struct vse_msg *msg, bool sync);
int vse_set_source(struct vse_device *vse, u32 inst, u32 source);
int vse_set_fps_rate(struct vse_device *vse, u32 inst, u32 chnl, struct vse_fps_rate *fps);
int vse_set_iformat(struct vse_device *vse, u32 inst, struct cam_format *fmt);
int vse_set_oformat(struct vse_device *vse, u32 inst, u32 chnl, struct cam_format *fmt,
		    struct cam_rect *crop);
int vse_set_format(struct vse_device *vse, u32 inst, u32 chnl,
		   struct vse_format *fmt, struct vse_stitching *stitch);
int vse_set_cascade(struct vse_device *vse, u32 inst, u32 cas_id, bool en_cas);
int vse_set_state(struct vse_device *vse, u32 inst, int enable, u32 cur_cnt, u32 total_cnt);
int vse_set_osd_info(struct vse_device *vse, u32 inst, u32 chnl, struct vse_osd_info *info);
int vse_set_osd_buf(struct vse_device *vse, u32 inst, u32 chnl, struct vse_osd_buf *osd_buf);
int vse_set_osd_lut(struct vse_device *vse, u32 inst, u32 chnl, struct vse_lut_tbl *lut_tbl);
int vse_set_src_ctx(struct vse_device *vse, u32 inst, u32 chnl, struct cam_buf_ctx *ctx);
int vse_set_ctx(struct vse_device *vse, u32 inst, struct vse_irq_ctx *ctx);
int vse_add_job(struct vse_device *vse, u32 inst);
int vse_open(struct vse_device *vse, u32 inst);
int vse_close(struct vse_device *vse, u32 inst);
int vse_probe(struct platform_device *pdev, struct vse_device *vse);
int vse_remove(struct platform_device *pdev, struct vse_device *vse);

void vse_reset(struct vse_device *vse);
#ifdef CONFIG_DEBUG_FS
void vse_debugfs_init(struct vse_device *vse);
void vse_debugfs_remo(struct vse_device *vse);
#endif
#ifdef CONFIG_PM_SLEEP
int vse_system_suspend(struct device *dev);
int vse_system_resume(struct device *dev);
#endif
#ifdef CONFIG_PM
int vse_runtime_suspend(struct device *dev);
int vse_runtime_resume(struct device *dev);
#endif

s32 vse_msg_handler(void *msg, u32 len, void *arg);
int new_frame(struct vse_irq_ctx *ctx);
struct vse_irq_ctx *get_next_irq_ctx(struct vse_device *vse);
irqreturn_t vse_irq_handler(int irq, void *arg);
irqreturn_t vse_fe_irq_handler(int irq, void *arg);

#endif /* _VSE_H_ */
