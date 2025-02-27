/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ISP_H_
#define _ISP_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/timekeeping.h>

#include "cam_ctx.h"
#include "isp_uapi.h"
#include "job_queue.h"
#include "mem_helper.h"

#define ISP_SINK_ONLINE_PATH_MAX  (4)
#define ISP_SINK_OFFLINE_PATH_MAX (4)
#define ISP_SINK_PATH_MAX (ISP_SINK_ONLINE_PATH_MAX + ISP_SINK_OFFLINE_PATH_MAX)
#define ISP_OUT_CHNL_MAX (2)
#define SRC_BUF_NUM (16)
#define MCM_BUF_NUM (4)
#define HDR_BUF_NUM (2)
#define TILE_COUNT (2)

#define INVALID_INST (0xff)

#define isp_write(isp, offset, value) \
	__raw_writel(value, (isp)->base + (offset))

#define isp_read(isp, offset) __raw_readl((isp)->base + (offset))

#define ISP_MP_FRAME_END  (0x1 << 0)
#define ISP_RDMA_END      (0x1 << 1)
#define ISP_MIS_FRAME_END (0x1 << 2)
#define ISP_SW_FRAME_DONE (ISP_MP_FRAME_END | ISP_RDMA_END | ISP_MIS_FRAME_END)

union u32_byte_map {
	u32 v;
	u8 b[4];
};

#define set_online(x, n)    (x).b[n] = 0
#define set_offline(x, n)   (x).b[n] = 1
#define is_online(x, n)     (!(x).b[n])
#define is_offline(x, n)    ((x).b[n])
#define has_offline(x)      ((x).v)
#define get_offline(x)      (((x).v & 1) + \
                            (((x).v >> 8) & 1) * 2 + \
                            (((x).v >> 16) & 1) * 3 + \
                            (((x).v >> 24) & 1) * 4 - 1)

struct isp_irq_ctx {
	bool sink_online_en;
	union u32_byte_map src_online_stat;
	struct cam_buf *sink_buf, *src_buf, *next_src_buf, *pd_buf;
	struct cam_ctx *sink_ctx, *src_ctx[ISP_OUT_CHNL_MAX], *stat_ctx, *pd_ctx;
	struct list_head *src_buf_list1, *src_buf_list2, *src_buf_list3;
};

struct cam_list_node {
	void *data;
	struct list_head entry;
};

struct isp_rgbgamma_data {
	u32 rgbgc_r_px[64];
	u32 rgbgc_r_datax[63];
	u32 rgbgc_r_datay[64];
	u32 rgbgc_g_px[64];
	u32 rgbgc_g_datax[63];
	u32 rgbgc_g_datay[64];
	u32 rgbgc_b_px[64];
	u32 rgbgc_b_datax[63];
	u32 rgbgc_b_datay[64];
	u8  flag;
};

struct isp_wdr5_data {
	u32 lut_histogram_write_data[65];
	u32 lut_shift_write_data[65];
	u32 lut_shift0_write_data[65];
	u32 lut_gammapre_write_data[65];
	u32 lut_gammadown_write_data[65];
	u32 lut_entropy_write_data[65];

	u32 lut_distance_weight_write_data[65];
	u32 lut_difference_weight_write_data[65];
	u32 lut_flat_factor_write_data[272];
	u8 lut_flat_level_write_data[68];
	u32 lut_sat_shift_write_data[18];

	u8 histogram_w_data_changed;
	u8 lut_shift_w_data_changed;
	u8 lut_shift0_w_data_changed;
	u8 gammapre_w_data_changed;
	u8 gammadown_w_data_changed;
	u8 entropy_w_data_changed;

	u8 lut_distance_weight_w_data_changed;
	u8 difference_weight_w_data_changed;
	u8 flat_factor_w_data_changed;
	u8 flat_level_w_data_changed;
	u8 sat_shift_w_data_changed;
};

struct isp_instance {
	spinlock_t lock; /* lock for handling ctx */
	struct isp_irq_ctx ctx;
	struct list_head src_buf_list1, src_buf_list2, src_buf_list3;
	struct cam_list_node src_bufs[SRC_BUF_NUM];
	struct ibuf *mcm_ib, *mcm_ib1, *prev_mcm_ib;
	struct isp_format fmt;
	struct cam_format sub_ifmt;
	struct cam_input in;
	enum cam_state state;
	enum cam_error error;
	int stream_idx;
	bool hdr_en;
	bool tile_en;
	u32 tile_count;
	u32 rdma_buf_count;
	u32 online_mcm;
	ktime_t last_frame_done, frame_interval;
	u32 frame_count;
	struct isp_gamma_febe_ctrl febe_ctrl;
	struct isp_rgbgamma_data rgbgamma_data;
	struct isp_wdr5_data wdr5_data;
	u32 af_mode;
	void *prev;
};

struct ibuf {
	struct mem_buf buf;
	struct cam_frame_info info;
	struct list_head entry;
};

struct ibuf_manage {
	struct list_head list1;
	struct list_head list2;
	struct list_head list3;
};

struct isp_schedule {
	spinlock_t lock; /* lock for isp schedule function */
	u32 next_mi_inst;
	bool mi_idle;
	u32 frame_done_mask;
};

struct isp_ctrl_msg {
	int rc;
	union {
		struct isp_ctrl ctrl;
		struct isp_ctrl_ext ctrl_ext;
	};
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
	struct job_queue *jq; /* online & offline job queue */
	struct isp_schedule sch;
	struct isp_instance *insts;
	struct mem_list in_buf_list;
	struct mem_buf in_bufs[ISP_SINK_ONLINE_PATH_MAX][MCM_BUF_NUM];
	struct ibuf_manage ibm[ISP_SINK_ONLINE_PATH_MAX];
	struct ibuf ib[ISP_SINK_ONLINE_PATH_MAX][MCM_BUF_NUM];
	int stream_idx_mapping[ISP_SINK_ONLINE_PATH_MAX];
	struct mem_list hdr_buf_list;
	struct mem_buf hdr_bufs[HDR_BUF_NUM];
	u32 hdr_sram[2];
	bool hdr_sram_rsvd;
	u32 cur_mi_irq_ctx, next_mi_irq_ctx;
	refcount_t set_state_refcnt;
	enum cam_error error;
	bool unit_test;
	enum isp_work_mode mode;
	struct mutex open_lock; /* lock for open_cnt */
	struct mutex set_input_lock; /* lock for set_input */
	struct mutex set_state_lock; /* lock for set_state */
	refcount_t open_cnt;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_log_file;
	struct dentry *debugfs_tune_file;
	struct dentry *debugfs_fps_file;
#endif
	struct tasklet_struct update_lut_tbl;
	struct isp_ctrl_msg ctrl_msg;
	wait_queue_head_t ctrl_waitq;
	bool ctrl_cond, ctrl_exit;
	struct mutex ctrl_lock; /* lock for isp ctrl */
};

bool isp_get_hdr_sram_enabled(struct isp_device *isp, u32 inst);
void isp_set_mcm_buffer(struct isp_device *isp, u32 path, phys_addr_t phys_addr);
void isp_set_mp_buffer(struct isp_device *isp, phys_addr_t phys_addr, struct cam_format *fmt);
int isp_post(struct isp_device *isp, struct isp_msg *msg, bool sync);
int isp_post_ex(struct isp_device *isp, struct isp_msg *msg,
		struct mem_buf *extra, bool sync, int *result);
int isp_set_input(struct isp_device *isp, u32 inst, struct cam_input *in);
int isp_set_input_select(struct isp_device *isp, u32 inst, u32 in_id, u32 in_chnl);
int isp_set_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size);
int isp_get_subctrl(struct isp_device *isp, u32 inst, u32 cmd, void *data, u32 size);
int isp_set_iformat(struct isp_device *isp, u32 inst, struct cam_format *fmt, struct cam_rect *crop,
		    bool hdr_en);
int isp_set_oformat(struct isp_device *isp, u32 inst, struct cam_format *fmt);
int isp_set_format(struct isp_device *isp, u32 inst, struct isp_format *fmt);
int isp_set_state(struct isp_device *isp, u32 inst, int state, enum group_type type);
int isp_get_ctx(struct isp_device *isp, u32 inst, struct isp_irq_ctx *ctx);
int isp_set_ctx(struct isp_device *isp, u32 inst, struct isp_irq_ctx *ctx);
int isp_set_stream_idx(struct isp_device *isp, u32 inst, int idx);
int isp_add_job(struct isp_device *isp, u32 inst);
int isp_fetch_job(struct isp_device *isp, u32 *inst);
int isp_query_job(struct isp_device *isp, u32 *inst);
int isp_remove_job(struct isp_device *isp, u32 inst);
int isp_set_schedule(struct isp_device *isp, struct isp_mcm_sch *sch, u32 miv2_mis,
		     u32 isp_mis, bool isp_irq_call);
int isp_get_schedule(struct isp_device *isp, struct mi_mis_group *mi_mis);
int isp_reset_schedule(struct isp_device *isp, u32 inst, bool force_reset);
int isp_add_schedule(struct isp_device *isp, struct mi_mis_group *mi_mis);
int isp_open(struct isp_device *isp, u32 inst);
int isp_close(struct isp_device *isp, u32 inst, enum group_type type);
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
void frame_done(struct isp_device *isp, u32 inst, bool timeout);
struct isp_irq_ctx *get_next_irq_ctx(struct isp_device *isp);
int new_frame(struct isp_irq_ctx *ctx);
int handle_mcm(struct isp_device *isp, u32 path, bool error);
irqreturn_t isp_irq_handler(int irq, void *arg);
irqreturn_t mi_irq_handler(int irq, void *arg);
irqreturn_t fe_irq_handler(int irq, void *arg);
void isp_update_none_shd_regs(unsigned long data);

#endif /* _ISP_H_ */
