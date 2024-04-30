/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CSI_H_
#define _CSI_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "csi_uapi.h"

#define PLATFORM_ASIC

#define csi_write(csi, offset, value) \
	__raw_writel(value, (csi)->base + (offset))

#define csiex_write(csi, offset, value) \
	__raw_writel(value, (csi)->base_ex + (offset))

#define csi_read(csi, offset) \
	__raw_readl((csi)->base + (offset))

#define CSI_INT_ERRSTR_BITLEN (32)

enum csi_data_type {
	DATA_TYPE_YUV420_8 = 0x18,
	DATA_TYPE_YUV420_10 = 0x19,
	DATA_TYPE_YUV420_SHIFT = 0x1c,
	DATA_TYPE_YUV422_8 = 0x1e,
	DATA_TYPE_YUV422_10 = 0x1f,
	DATA_TYPE_RGB888 = 0x24,
	DATA_TYPE_RAW8 = 0x2a,
	DATA_TYPE_RAW10 = 0x2b,
	DATA_TYPE_RAW12 = 0x2c,
	DATA_TYPE_RAW14 = 0x2d,
};

enum csi_err_type {
	CSI_ERR_PHY_FATAL = 0,
	CSI_ERR_PKT_FATAL = 1,
	CSI_ERR_BNDRY_FRAME_FATAL = 2,
	CSI_ERR_SEQ_FRAME_FATAL = 3,
	CSI_ERR_CRC_FRAME_FATAL = 4,
	CSI_ERR_PLD_CRC_FATAL = 5,
	CSI_ERR_DATA_ID = 6,
	CSI_ERR_ECC_CORRECTED = 7,
	CSI_ERR_PHY = 8,
	CSI_ERR_LINE = 9,
	CSI_ERR_IPI1 = 10,
	CSI_ERR_IPI2 = 11,
	CSI_ERR_IPI3 = 12,
	CSI_ERR_IPI4 = 13,
};

struct csi_irq_reg {
	u32 id;
	u32 st_mask;
	u32 reg_st;
	u32 reg_mask;
	u32 reg_force;
	u32 err_mask;
	const char *errstr[CSI_INT_ERRSTR_BITLEN];
};

struct csi_device {
	u32 id, num_insts;
	struct device *dev;
	void __iomem *base, *base_ex;
	struct clk *ipi, *pclk, *cfg;
	struct reset_control *rst;
	struct isc_handle *isc;
	spinlock_t isc_lock; /* lock for sending msg */
	struct csiw_device *csiw_dev;
	struct cam_ctrl_device *cam_ctl_dev;

	bool is_native;
	bool is_bypass;
	bool ipi0_ctrlmode;
	u32 ipiclk;
	bool tpg_enable;
	bool tpg_horizontal;
	u8 tpg_vc;
	u8 tpg_dt;
	u32 pkt2pkt_time;
	u32 lane_rate;
	u32 lanes;
	void (*subirq_func)(struct csi_device *csi, u32 subirq, const struct csi_irq_reg *err_reg, u32 reset_flag);
	void (*irq_done_func)(struct csi_device *csi, u32 irq_main_st);
};

struct csi_ipi_base_cfg {
	u32 id;
	u32 adv_val;
	bool cut_through;
	bool mem_auto_flush;
	u32 hsa;
	u32 hbp;
	u32 hsd;
};

enum csi_ipi_info {
	CSI_IPI_INFO_MODE = 0,
	CSI_IPI_INFO_VC,
	CSI_IPI_INFO_DT,
	CSI_IPI_INFO_MEM,
	CSI_IPI_INFO_HSA,
	CSI_IPI_INFO_HBP,
	CSI_IPI_INFO_HSD,
	CSI_IPI_INFO_ADV,
	CSI_IPI_INFO_FATAL,
	CSI_IPI_INFO_SOFTRSTN,
	CSI_IPI_INFO_NUM,
};

int csi_probe(struct platform_device *pdev, struct csi_device *csi, bool is_native);
int csi_remove(struct platform_device *pdev, struct csi_device *csi);
void csi_set_tpg_mode(struct csi_device *csi, u32 tpg_en, u32 pkt2pkt_time, u8 horizontal, u8 vc, u8 datatype);
int csi_ipi_set_vc_cfg(struct csi_device *csi, u32 ipi_id,
		       struct csi_vc_cfg *vc_cfg);
void csi_dphy_reset(struct csi_device *csi);
void csi_set_lanes(struct csi_device *csi, u32 lanes, u32 vcext);
int csi_set_lane_rate(struct csi_device *csi, u32 rate);
int csi_wait_stop_state(struct csi_device *csi, u32 nowait, u32 wait_ms, u32 lanes, u32 *errstate);

int csi_ipi_init(struct csi_device *csi, struct csi_ipi_base_cfg *ipi_cfg, struct cam_format *fmt);
int csi_set_tpg(struct csi_device *csi, bool is_en);
int csi_ipi_start(struct csi_device *csi, u32 ipi_id);
int csi_ipi_stop(struct csi_device *csi, u32 ipi_id);
void csi_ipi_reset_and_down(struct csi_device *csi, u32 ipi_id, bool enable);
void csi_ipi_overflow_reset(struct csi_device *csi, u32 ipi_id);
void csi_ipi_reg_dump(const struct csi_device *csi, u32 ipi_id, u32 *reg_vals);
void csi_ipi_reg_set(struct csi_device *csi, u32 ipi_id, u32 *reg_vals, u32 set_mask);

int csi_idi_set_vc_cfg(struct csi_device *csi, u32 idi_id,
		       struct csi_vc_cfg *vc_cfg);
int csi_idi_set_data_type(struct csi_device *csi, u32 idi_id,
			  struct cam_format *fmt);
u32 csi_get_phy_rx_status(struct csi_device *csi);

#ifdef PLATFORM_ASIC
int csi_dphy_init(struct csi_device *csi);
#endif

#ifdef CONFIG_PM_SLEEP
int csi_system_suspend(struct device *dev);
int csi_system_resume(struct device *dev);
#endif
#ifdef CONFIG_PM
int csi_runtime_suspend(struct device *dev);
int csi_runtime_resume(struct device *dev);
#endif

s32 csi_msg_handler(void *msg, u32 len, void *arg);
void csi_irq_enable(struct csi_device *csi);
void csi_irq_disable(struct csi_device *csi);
int csi_irq_disable_mask(struct csi_device *csi, u32 mask);
irqreturn_t csi_irq_handler(int irq, void *arg);

#endif /* _CSI_H_ */
