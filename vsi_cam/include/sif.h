/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _SIF_H_
#define _SIF_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "cam_buf.h"
#include "cam_uapi.h"
#include "sif_uapi.h"

#define SIF_FMT_MAX (4)
#define SIF_RES_MAX (3)

#define sif_write(sif, offset, value) \
	__raw_writel(value, (sif)->base + (offset))

#define sif_read(sif, offset) __raw_readl((sif)->base + (offset))
#define FRAME_ID_MAXIMUM 0xFFFF

enum sif_input_type {
	MAIN_IMG = 0,
	EMB_DATA_PRE = 1,
	EMB_DATA_POST = 2,
};

enum sif_channel_type {
	IN_CHANNEL       = 0,
	OUT_CHANNEL_MAIN = 1,
	OUT_CHANNEL_EMB  = 2,
	BOTH_CHANNEL     = 3,
	INVALID_CHANNEL  = 4,
};

enum time_stamp_type {
	IPI_VSYNC    = BIT(0),
	IPI_TRIGGER  = BIT(1),
	PPS0_TRIGGER = BIT(2),
	PPS1_TRIGGER = BIT(3),
};

enum ts_trigger_source {
	TS_SRC_INVALID,
	TS_SRC_LPWM0_CHN0,
	TS_SRC_LPWM0_CHN1,
	TS_SRC_LPWM0_CHN2,
	TS_SRC_LPWM0_CHN3,
	TS_SRC_LPWM1_CHN0,
	TS_SRC_LPWM1_CHN1,
	TS_SRC_LPWM1_CHN2,
	TS_SRC_LPWM1_CHN3,
	TS_SRC_ENET_PTP,
	TS_SRC_SW,
	TS_SRC_MCU,
	TS_SRC_GPS,
};

enum pps_trigger_source {
	PPS_SRC_INVALID,
	PPS_SRC_MCU,
	PPS_SRC_GPS,
	PPS_SRC_TIME_SYNC2,
	PPS_SRC_TIME_SYNC3,
	PPS_SRC_ENET_PTP,
	PPS_SRC_SW,
};

struct sif_irq_ctx {
	struct cam_buf_ctx *sink_ctx;
	struct cam_buf_ctx *buf_ctx;
	struct cam_buf *buf;
	struct cam_buf_ctx *emb_buf_ctx;
	struct cam_buf *emb_buf;
};

struct sif_format_cap {
	u32 format;
	struct cam_res_cap res[SIF_RES_MAX];
};

struct sif_instance {
	spinlock_t lock; /* lock for handling ctx */
	struct sif_irq_ctx ctx;
	struct sif_format_cap fmt_cap[SIF_FMT_MAX];
	u32 input_bayer_format;
	enum cam_state state;
	enum cam_error error;
	struct sif_cfg sif_cfg;
	struct cam_format fmt;
	u32 frameid_cnt;
	u32 size_err_cnt;
	u32 ipi_base;
};

struct sif_device {
	u32 id, num_insts;
	struct device *dev;
	void __iomem *base;
	unsigned int timestamp_clk;
	unsigned int ipi_trigger_src;
	unsigned int pps_trigger_src;
	u32 ipi_channel_num; // indicate how many ipis will be used.
	u32 ipi_base; // indicate the base ipi num.
	struct clk *axi, *pclk;
	struct reset_control *rst;
	struct isc_handle *isc;
	spinlock_t isc_lock; /* lock for sending msg */
	spinlock_t cfg_reg_lock; /* lock for cfg register*/
	struct cam_ctrl_device *ctrl_dev;
	struct sif_instance *insts;
};

struct sif_frame_des {
	u32 frame_id; // frame id read from sif register.
	u32 trigger_freq; // trigger source frequency.
	u64 fs_ts; // frame start timestamp read from sif register.
	u64 trigger_ts; // frame start trigger timestamp read from sif register.
	u64 pps1_ts; // pps1 timestamp read from sif register.
	u64 pps2_ts; // pps2 timestamp read from sif register.
	u64 timestamps; // kernel time read from kernel api.
};

void sif_post(struct sif_device *sif, void *msg, u32 len);
int sif_set_format(struct sif_device *sif, u32 inst, struct cam_format *fmt,
		   bool post, enum sif_channel_type channel_type);
int sif_set_state(struct sif_device *sif, u32 inst, int enable, bool post);
int sif_set_ctx(struct sif_device *sif, u32 inst, struct sif_irq_ctx *ctx);
int sif_probe(struct platform_device *pdev, struct sif_device *sif);
int sif_remove(struct platform_device *pdev, struct sif_device *sif);

int sif_reset_ipi(struct sif_device *sif, u32 inst);

void sif_reset(struct sif_device *sif);
#ifdef CONFIG_PM_SLEEP
int sif_system_suspend(struct device *dev);
int sif_system_resume(struct device *dev);
#endif
#ifdef CONFIG_PM
int sif_runtime_suspend(struct device *dev);
int sif_runtime_resume(struct device *dev);
#endif

s32 sif_msg_handler(void *msg, u32 len, void *arg);
irqreturn_t sif_irq_handler(int irq, void *arg);

#endif /* _SIF_H_ */
