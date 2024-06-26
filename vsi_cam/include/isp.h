/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ISP_H_
#define _ISP_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>

#include "isp_uapi.h"
#include "job_queue.h"
#include "mem_helper.h"

#define ISP_SINK_ONLINE_PATH_MAX  (4)
#define ISP_SINK_OFFLINE_PATH_MAX (2)
#define ISP_SINK_PATH_MAX (ISP_SINK_ONLINE_PATH_MAX + ISP_SINK_OFFLINE_PATH_MAX)
#define SRC_BUF_NUM (16)
#define MCM_BUF_NUM (6)
#define HDR_BUF_NUM (2)

#define ISP_FMT_MAX (3)
#define ISP_RES_MAX (3)

#define INVALID_MCM_SCH_INST (0xff)

#define isp_write(isp, offset, value) \
	__raw_writel(value, (isp)->base + (offset))

#define isp_read(isp, offset) __raw_readl((isp)->base + (offset))

struct isp_irq_ctx {
	bool is_sink_online_mode, is_src_online_mode, ddr_en;
	struct cam_buf *sink_buf, *src_buf;
	struct cam_ctx *sink_ctx, *src_ctx, *stat_ctx;
	struct list_head *src_buf_list1, *src_buf_list2, *src_buf_list3;
};

struct isp_format_cap {
	u32 format;
	struct cam_res_cap res[ISP_RES_MAX];
};

struct cam_list_node {
	void *data;
	struct list_head entry;
};

struct isp_instance {
	spinlock_t lock; /* lock for handling ctx */
	struct isp_irq_ctx ctx;
	struct list_head src_buf_list1, src_buf_list2, src_buf_list3;
	struct cam_list_node src_bufs[SRC_BUF_NUM];
	struct ibuf *mcm_ib, *mcm_ib1, *prev_mcm_ib;
	struct isp_format_cap fmt_cap[ISP_FMT_MAX];
	struct isp_format fmt;
	struct cam_input in;
	u32 input_bayer_format;
	enum cam_state state;
	enum cam_error error;
	int stream_idx;
	bool hdr_en;
	u32 rdma_buf_count;
	u32 online_mcm;
};

struct ibuf {
	struct mem_buf buf;
	struct list_head entry;
};

struct ibuf_manage {
	struct list_head list1;
	struct list_head list2;
	struct list_head list3;
};

struct mcm_sch_node {
	u32 inst;
	struct list_head entry;
};

struct isp_device {
	u32 id, num_insts;
	struct device *dev;
	void __iomem *base;
	struct clk *core, *axi, *mcm, *hclk;
	struct reset_control *rst;
	struct isc_handle *isc;
	spinlock_t isc_lock; /* lock for sending msg */
	struct cam_ctrl_device *ctrl_dev;
	struct job_queue *jq; /* offline job queue */
	struct isp_instance *insts;
	struct list_head in_buf_list;
	struct mem_buf in_bufs[ISP_SINK_ONLINE_PATH_MAX];
	struct ibuf_manage ibm[ISP_SINK_ONLINE_PATH_MAX];
	struct ibuf ib[ISP_SINK_ONLINE_PATH_MAX][MCM_BUF_NUM];
	int stream_idx_mapping[ISP_SINK_ONLINE_PATH_MAX];
	struct list_head hdr_buf_list;
	struct mem_buf hdr_bufs[HDR_BUF_NUM];
	u32 in_counts[ISP_SINK_ONLINE_PATH_MAX];
	u32 cur_mi_irq_ctx, next_mi_irq_ctx;
	refcount_t set_state_refcnt;
	enum cam_error error;
	bool rdma_busy;
	bool unit_test;
	enum isp_work_mode mode;
	struct mutex open_lock; /* lock for open_cnt */
	struct mutex set_input_lock; /* lock for set_input */
	struct mutex set_state_lock; /* lock for set_state */
	spinlock_t mcm_sch_lock; /* lock for mcm_sch */
	struct list_head mcm_sch_idle_list, mcm_sch_busy_list;
	struct mcm_sch_node sch_node[ISP_SINK_PATH_MAX * SRC_BUF_NUM];
	refcount_t open_cnt;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_log_file;
	struct dentry *debugfs_tune_file;
#endif
};

void isp_set_mcm_raw_buffer(struct isp_device *isp, u32 path_id,
			    phys_addr_t phys_addr, struct cam_format *fmt);
void isp_set_rdma_buffer(struct isp_device *isp, phys_addr_t rdma_addr);
void isp_set_mp_buffer(struct isp_device *isp, phys_addr_t phys_addr, struct cam_format *fmt);
int isp_post(struct isp_device *isp, struct isp_msg *msg, bool sync);
int isp_post_ex(struct isp_device *isp, struct isp_msg *msg,
		struct mem_buf *extra, bool sync);
int isp_set_input(struct isp_device *isp, u32 inst, struct cam_input *in);
int isp_set_input_select(struct isp_device *isp, u32 inst, u32 in_id, u32 in_chnl);
int isp_set_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size);
int isp_get_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size);
int isp_set_iformat(struct isp_device *isp, u32 inst, struct cam_format *fmt, struct cam_rect *crop,
		    bool hdr_en);
int isp_set_oformat(struct isp_device *isp, u32 inst, struct cam_format *fmt);
int isp_set_format(struct isp_device *isp, u32 inst, struct isp_format *fmt);
int isp_set_state(struct isp_device *isp, u32 inst, int state);
int isp_set_ctx(struct isp_device *isp, u32 inst, struct isp_irq_ctx *ctx);
int isp_set_stream_idx(struct isp_device *isp, u32 inst, int idx);
int isp_add_job(struct isp_device *isp, u32 inst, bool mcm_online);
int isp_remove_job(struct isp_device *isp, u32 inst);
int isp_set_schedule_offline(struct isp_device *isp, u32 inst, bool isp_irq_call);
int isp_set_schedule(struct isp_device *isp, struct isp_mcm_sch *sch, bool mcm_online);
int isp_get_schedule(struct isp_device *isp, u32 *inst);
int isp_reset_schedule(struct isp_device *isp);
int isp_check_schedule(struct isp_device *isp, u32 *inst);
int isp_open(struct isp_device *isp, u32 inst);
int isp_close(struct isp_device *isp, u32 inst);
int isp_probe(struct platform_device *pdev, struct isp_device *isp);
int isp_remove(struct platform_device *pdev, struct isp_device *isp);

void isp_reset(struct isp_device *isp);
#ifdef CONFIG_DEBUG_FS
void isp_debugfs_init(struct isp_device *isp);
void isp_debugfs_remo(struct isp_device *isp);
#endif
#ifdef CONFIG_PM_SLEEP
int isp_system_suspend(struct device *dev);
int isp_system_resume(struct device *dev);
#endif
#ifdef CONFIG_PM
int isp_runtime_suspend(struct device *dev);
int isp_runtime_resume(struct device *dev);
#endif

s32 isp_msg_handler(void *msg, u32 len, void *arg);
struct isp_irq_ctx *get_next_irq_ctx(struct isp_device *isp);
int new_frame(struct isp_irq_ctx *ctx);
irqreturn_t isp_irq_handler(int irq, void *arg);
irqreturn_t mi_irq_handler(int irq, void *arg);
irqreturn_t fe_irq_handler(int irq, void *arg);

#endif /* _ISP_H_ */
