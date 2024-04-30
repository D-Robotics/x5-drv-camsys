/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_mipi_phy_ops.h
 *
 * @NO{S10E03C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_PHY_OPS_H__
#define __HOBOT_MIPI_PHY_OPS_H__ /* PRQA S 0603 */ /* header file macro */

#include "hobot_mipi_osal.h"

#include "hobot_mipi_phy.h"
#include "hobot_mipi_utils.h"

#ifdef CONFIG_HOBOT_FUSA_DIAG
#include "hobot_mipi_csi_stl.h"
#endif

/* module params of reged num */
extern uint32_t host_num;
extern uint32_t dev_num;

#if defined MODULE && !defined CONFIG_ARCH_HOBOT
#define EX_MODULE
#endif

/* phy driver config */
#define MIPI_PHY_DNAME		        "mipi_phy"

#define MIPICLK_FREQ_MHZ_2_MBPS		(2U)
#define OSC_FREQ_DEFAULT_1P4		(460U)
#define OSC_FREQ_DEFAULT_1P3		(438U)
#define TXOUT_FREQ_GAIN_DEFAULT		(5U)
#define TXOUT_FREQ_GAIN_PERCENT		(100U)
#define TXOUT_FREQ_FORCE_MIN_MHZ	(40U)
#define TXOUT_FREQ_SLEWRATE_MBPS	(1500U)
#define TXOUT_FREQ_CHGPUMP_MBPS		(2300U)
#define RXPHY_MERGE_LANE_MIN		(2U)
#define RXPHY_VREFCD_LPRX_DEFFAULT	(1U)
#define RXPHY_V400_PROG_DEFFAULT	(4U)
#define RXPHY_DESKEW_DEFFAULT		(0U)

/* module params: txout freq */
extern uint32_t txout_freq_mode;
extern uint32_t txout_freq_autolarge_enbale;
extern uint32_t txout_freq_gain_precent;
extern uint32_t txout_freq_force;

/* module params: rxdphy vref */
extern uint32_t rxdphy_vrefcd_lprx;
extern uint32_t rxdphy_v400_prog;
extern uint32_t rxdphy_deskew_cfg;

#if defined CONFIG_HOBOT_J5
#define MIPI_DPHY_CLK_24M_OUT_J5   (1)

/* pin reg core for j5 clk_24m_out */
#define REG_MIPI_DPHY_PINREG_CTRL0          (0x0)
#define REG_MIPI_DPHY_PINREG_CTRL1          (0x4)
#define MIPI_DPHY_PINREG_CTRL0_SEL_MASK     (0x3U)
#define MIPI_DPHY_PINREG_CTRL0_SEL_OFFS     (0x0)
#define MIPI_DPHY_PINREG_CTRL0_SEL_24M      (0)
#define MIPI_DPHY_PINREG_CTRL0_SEL_ARMPLL1  (1)
#define MIPI_DPHY_PINREG_CTRL0_SEL_PERIPLL2 (2)
#define MIPI_DPHY_PINREG_CTRL0_EN12         (0x1UL << 3U)
#define MIPI_DPHY_PINREG_CTRL0_RESERVED     (0x1UL << 2U)
static inline uint32_t mipi_dphy_pinctrl0_sel(int32_t sel) {
	uint32_t ctrl0 = (uint32_t)MIPI_DPHY_PINREG_CTRL0_RESERVED; /* qacfix: conversion */
	ctrl0 |= ((uint32_t)sel & MIPI_DPHY_PINREG_CTRL0_SEL_MASK) << MIPI_DPHY_PINREG_CTRL0_SEL_OFFS; /* qacfix: conversion */
	if ((sel == MIPI_DPHY_PINREG_CTRL0_SEL_ARMPLL1) ||
		(sel == MIPI_DPHY_PINREG_CTRL0_SEL_PERIPLL2)) {
		ctrl0 |= (uint32_t)MIPI_DPHY_PINREG_CTRL0_EN12; /* qacfix: conversion */
	}
	return ctrl0;
}
#define MIPI_DPHY_PINREG_CTRL1_PLL1_MASK  (0x3FU)
#define MIPI_DPHY_PINREG_CTRL1_PLL1_OFFS  (0)
#define MIPI_DPHY_PINREG_CTRL1_PLL1_DFT   (0x002BU)
#define MIPI_DPHY_PINREG_CTRL1_PLL2_MASK (0x3FU)
#define MIPI_DPHY_PINREG_CTRL1_PLL2_OFFS (8)
#define MIPI_DPHY_PINREG_CTRL1_PLL2_DFT  (0x2B00U)
#define MIPI_DPHY_CLK_24M_SEL_24M           (24000000UL)
#define MIPI_DPHY_CLK_24M_SEL_ARMPLL1       (1200000000UL)
#define MIPI_DPHY_CLK_24M_SEL_PERIPLL2      (1300000000UL)
#else
#define MIPI_DPHY_CLK_24M_OUT_J5   (0)
#endif

/* mem res index */
#define MIPI_DPHY_MEM_RES_CTRL_IDX		(0)
#define MIPI_DPHY_MEM_RES_OUTCLK_IDX	(1)
#define MIPI_DPHY_MEM_RES_STLERM_IDX	(2)

/* reg value macro */
#define DPHY_RAISE               (1)
#define DPHY_RESETN              (0)

#define DPHY_TEST_CLEAR          (0x00000001U)
#define DPHY_TEST_RESETN         (0x00000000U)
#define DPHY_TEST_CLK            (0x00000002U)
#define DPHY_TEST_ENABLE         (0x00010000U)
#define DPHY_TEST_DATA_MAX       (4)

#define TX_REFSCLK_DEFAULT       (24U)
#define TX_PLL_INPUT_DIV_MIN     (1U)
#define TX_PLL_INPUT_DIV_MAX     (16U)
#define TX_PLL_FB_MULTI_MIN      (64U)
#define TX_PLL_FB_MULTI_MAX      (625U)
#define TX_PLL_INPUT_FEQ_MIN     (2U)
#define TX_PLL_INPUT_FEQ_MAX     (8U)

/*test code: addr*/
#define REGS_RX_SYS_7            (0x08)
#define REGS_RX_SYSTIMERS_0      (0x7A)
#define REGS_RX_STARTUP_OVR_2    (0xE2)
#define REGS_RX_STARTUP_OVR_3    (0xE3)
#define REGS_RX_STARTUP_OVR_4    (0xE4)
#define REGS_RX_STARTUP_OVR_5    (0xE5)
#define REGS_RX_STARTUP_OVR_17   (0xF1)
#define REGS_RX_CB_0             (0x1aa)
#define REGS_RX_CB_2             (0x1ac)
#define REGS_RX_CLKLANE_LANE_6   (0x307)
#define REGS_RX_CLKLANE_LANE_7   (0x308)
#define REGS_RX_LANE0_DDL_4      (0x60A)
#define REGS_RX_LANE0_DDL_5      (0x60B)
#define REGS_RX_LANE0_DDL_6      (0x60C)
#define REGS_RX_LANE1_DDL_4      (0x80A)
#define REGS_RX_LANE1_DDL_5      (0x80B)
#define REGS_RX_LANE1_DDL_6      (0x80C)
#define REGS_RX_LANE2_DDL_4      (0xA0A)
#define REGS_RX_LANE2_DDL_5      (0xA0B)
#define REGS_RX_LANE2_DDL_6      (0xA0C)
#define REGS_RX_LANE3_DDL_4      (0xC0A)
#define REGS_RX_LANE3_DDL_5      (0xC0B)
#define REGS_RX_LANE3_DDL_6      (0xC0C)

#define REGS_TX_TESECODE_08      (0x08)
#define REGS_TX_PLL_PRO_CHG_PUMP (0x0E)
#define REGS_TX_PLL_INT_CHG_PUMP (0x0F)
#define REGS_TX_PLL_GMP_DIG      (0x13)
#define REGS_TX_PLL_PHASE_ERR    (0x14)
#define REGS_TX_PLL_LOCK_FILT    (0x15)
#define REGS_TX_PLL_UNLOCK_FILT  (0x16)
#define REGS_TX_PLL_TH_DELAY     (0x1B)
#define REGS_TX_PLL_CPBIAS_CNTRL (0x1C)
#define REGS_TX_PLL_LOCK_DETMODE (0x1D)
#define REGS_TX_PLL_ANALOG_PROG  (0x1F)
#define REGS_TX_BANDGA_CNTRL     (0x24)
#define REGS_TX_SYSTIMERS_23     (0x65)
#define REGS_TX_HSTXTHSZERO_OVR  (0x72)
#define REGS_TX_SLEWRATE_FSM_OVR (0xA0)
#define REGS_TX_SLEWRATE_DDL_CFG (0xA3)
#define REGS_TX_PLL_2            (0x160)
#define REGS_TX_PLL_1            (0x15E)
#define REGS_TX_PLL_2            (0x160)
#define REGS_TX_PLL_3            (0x161)
#define REGS_TX_PLL_4            (0x162)
#define REGS_TX_PLL_17           (0x16E)
#define REGS_TX_PLL_19           (0x170)
#define REGS_TX_PLL_27           (0x178)
#define REGS_TX_PLL_28           (0x179)
#define REGS_TX_PLL_29           (0x17A)
#define REGS_TX_PLL_30           (0x17B)
#define REGS_TX_CB_2             (0x1AC)
#define REGS_TX_SLEW_5           (0x270)
#define REGS_TX_SLEW_7           (0x272)

/*test code: data*/
#define RX_CLK_SETTLE_EN         (0x01)
#define RX_CLK_SETTLE            (0x10U)
#define RX_CLK_200MODE           (0x10U)
#define RX_HS_SETTLE_DFT         (0x80U)
#define RX_HS_SETTLE_MASK        (0x7FU)
static inline uint8_t RX_HS_SETTLE(uint16_t settle) {
	return (uint8_t)(RX_HS_SETTLE_DFT | ((settle) & RX_HS_SETTLE_MASK));
}
#define RX_SYSTEM_CONFIG         (0x38)
#define RX_CB_BIAS_ATB           (0x4B)
#define RX_CB_VREF_DFT           (0x3U)
#define RX_CB_VREF_LP_MASK       (0x3U)
#define RX_CB_VREF_LP_OFFS       (5)
#define RX_CB_VREF_V4_MASK       (0x7U)
#define RX_CB_VREF_V4_OFFS       (2)
static inline uint8_t RX_CB_VREF_CB(uint32_t lp, uint32_t v4) {
	return (uint8_t)((((lp) & RX_CB_VREF_LP_MASK) << RX_CB_VREF_LP_OFFS) |
					 (((v4) & RX_CB_VREF_V4_MASK) << RX_CB_VREF_V4_OFFS) |
					 RX_CB_VREF_DFT);
}
#define RX_CLKLANE_PULLLONG      (0x80)
#define RX_OSCFREQ_HIGH_MASK     (0xFU)
#define RX_OSCFREQ_HIGH_OFFS     (8)
#define RX_OSCFREQ_LOW_MASK      (0xFFU)
#define RX_OSCFREQ_LOW_OFFS      (0x0)
static inline uint8_t RX_OSCFREQ_HIGH(uint16_t freq) {
	return (uint8_t)(((freq) >> RX_OSCFREQ_HIGH_OFFS) & RX_OSCFREQ_HIGH_MASK);
}
static inline uint8_t RX_OSCFREQ_LOW(uint16_t freq) {
	return (uint8_t)(((freq) >> RX_OSCFREQ_LOW_OFFS) & RX_OSCFREQ_LOW_MASK);
}
#define RX_OSCFREQ_EN            (0x1U)

#define TX_PLL_ANALOG_PROG_CTL   (0x1)
#define TX_PLL_GMP_DIG_LOCK      (0x1)
#define TX_PLL_PRO_CHG_PUMP_CTLD (0x0D)
#define TX_PLL_PRO_CHG_PUMP_CTLE (0x0E)
#define TX_PLL_INT_CHG_PUMP_CTL  (0x0)
#define TX_PLL_PHASE_ERR_TH1     (0x3)
#define TX_PLL_LOCK_FILT_TH2     (0x2D)
#define TX_PLL_UNLOCK_FILT_TH3   (0x3)
#define TX_SLEWRATE_FSM_OVR_EN   (0x2)
#define TX_SLEWRATE_DDL_CFG_SEL  (0x0)
#define TX_BANDGA_CNTRL_VAL      (0x7C)
static inline uint8_t TX_HS_ZERO(uint16_t settle) {
	return (uint8_t)(RX_HS_SETTLE_DFT | ((settle) & RX_HS_SETTLE_MASK));
}
#define TX_SLEW_RATE_CAL         (0x5E)
#define TX_SLEW_RATE_CTL         (0x11)
#define TX_PLL_DIV_DFL           (0x82U)
#define TX_PLL_DIV_N_OFFS        (3)
static inline uint8_t TX_PLL_DIV(uint8_t n) {
	return (uint8_t)(TX_PLL_DIV_DFL | ((n) << TX_PLL_DIV_N_OFFS));
}
#define TX_PLL_MULTI_H_MASK      (0x3U)
#define TX_PLL_MULTI_H_OFFS      (8)
#define TX_PLL_MULTI_L_MASK      (0xFFU)
#define TX_PLL_MULTI_L_OFFS      (0)
static inline uint8_t TX_PLL_MULTI_H(uint16_t m) {
	return (uint8_t)(((m) >> TX_PLL_MULTI_H_OFFS) & TX_PLL_MULTI_H_MASK);
}
static inline uint8_t TX_PLL_MULTI_L(uint16_t m) {
	return (uint8_t)(((m) >> TX_PLL_MULTI_L_OFFS) & TX_PLL_MULTI_L_MASK);
}
#define TX_PLL_VCO_DFL           (0x81U)
#define TX_PLL_VCO_MASK          (0x3FU)
#define TX_PLL_VCO_OFFS          (1)
static inline uint8_t TX_PLL_VCO(uint16_t v) {
	return (uint8_t)(TX_PLL_VCO_DFL | (((v) & TX_PLL_VCO_MASK) << TX_PLL_VCO_OFFS));
}
#define TX_PLL_CPBIAS            (0x10)
#define TX_PLL_INT_CTL           (0x4)
#define TX_PLL_PROP_CNTRL        (0x0C)
#define TX_PLL_RST_TIME_L        (0xFF)
#define TX_PLL_GEAR_SHIFT_L      (0x6)
#define TX_PLL_GEAR_SHIFT_H      (0x1)
#define TX_PLL_CLKDIV_CLK_LMT    (450U)
#define TX_PLL_CLKDIV_CLK_EN     (0x10)
#define TX_PLL_FORCE_LOCK        (0x4)
#define TX_TESETCODE_08_DATA     (0x3)
#define TX_HSTXTHSZERO_DATALANES (0x11)
#define TX_PLL_TH_DELAY          (0xaa)
#define TX_PLL_CPBIAS_CNTRL0     (0x10)
#define TX_PLL_CPBIAS_CNTRL1     (0xaa)

#define TX_PLL_N_BASE            (1U)
#define TX_PLL_M_BASE            (2U)
#define TX_PLL_VCO_DIV_MASK      (0x3U)
#define TX_PLL_VCO_DIV_OFFS      (4)
static inline uint16_t TX_PLL_VCO_DIV(uint32_t vco) {
	return (uint16_t)(((vco) >> TX_PLL_VCO_DIV_OFFS) & TX_PLL_VCO_DIV_MASK);
}

#define TESTCODE_BYTE_MASK       (0xFFU)
#define TESTCODE_MSBS_OFFS       (8)
static inline uint8_t TESTCODE_MSBS_BYTE(uint16_t t) {
	return (uint8_t)(((t) >> TESTCODE_MSBS_OFFS) & TESTCODE_BYTE_MASK);
}
static inline uint8_t TESTCODE_LSBS_BYTE(uint16_t t) {
	return (uint8_t)((t) & TESTCODE_BYTE_MASK);
}

/* run params */
struct mipi_phy_param_s {
	/* type must be: uint32_t */
	uint32_t dbg_value;
#ifdef CONFIG_HOBOT_FUSA_DIAG
	uint32_t stl_dbg;
	uint32_t stl_mask;
	uint32_t stl_pile;
#endif
};
#define MIPI_PHY_PARAMS_NUM	((int32_t)(sizeof(struct mipi_phy_param_s)/sizeof(uint32_t)))

/* run params name of sysfs, see: struct mipi_phy_param_s */
#ifdef CONFIG_HOBOT_FUSA_DIAG
#define MIPI_PHY_STL_PARAM_STRINGS \
	"stl_dbg", \
	"stl_mask", \
	"stl_pile",
#else
#define MIPI_PHY_STL_PARAM_STRINGS
#endif

#define MIPI_PHY_PARAM_STRINGS { \
	"dbg_value", \
	MIPI_PHY_STL_PARAM_STRINGS \
}

/* mipi host/dev phy info struct */
struct mipi_phy_s {
	struct mipi_phy_sub_s sub;
	uint16_t              reged;
	uint16_t              lane;
	uint16_t              mipiclk;
	uint16_t              settle;
	const void           *pll_sel;
	uint32_t              init_cnt;
	uint32_t              reset_cnt;
};

#if MIPI_DPHY_CLK_24M_OUT_J5
/* dphy out clock name struct */
struct mipi_dphy_outclk_s {
	void __iomem            *iomem;
	struct mipi_reg_s        reg;
	uint64_t				 freq;
};
#endif

#ifdef CONFIG_HOBOT_FUSA_DIAG
/* mipi erm struct for stl */
struct mipi_csi_erm_s {
	void __iomem            *iomem;
	struct mipi_reg_s        reg;
	struct mipi_csi_stl_priv  stl;
};
#endif

/* mipi dphy struct */
struct mipi_dphy_s {
	void __iomem            *iomem;
	struct mipi_reg_s        reg;
	/*
	 * osal_spin_lock: lock
	 * protect: dphy register operations(read & write).
	 * init: probe, see hobot_mipi_dphy_probe.
	 * call: reg set/get, see xxx_mipi_get_ctrl, xxx_mipi_set_ctrl,etc.
	 * note: externed as apis, maybe run in interrupt contex.
	 */
	osal_spinlock_t lock;
#if MIPI_DPHY_CLK_24M_OUT_J5
	struct mipi_dphy_outclk_s outclk;
#endif
#ifdef CONFIG_HOBOT_FUSA_DIAG
	struct mipi_csi_erm_s    erm;
#endif
	struct mipi_phy_param_s param;
};

/* mipi dphy op funs */
struct mipi_dphy_ops_s {
	const char *name;
	const struct mipi_dump_reg *regs;
	int32_t (*get_ctl)(int32_t type, int32_t port, int32_t region);
	int32_t (*set_ctl)(int32_t type, int32_t port, int32_t region, int32_t value);
	int32_t (*get_freqrange)(int32_t type, int32_t port, int32_t region);
	int32_t (*set_freqrange)(int32_t type, int32_t port, int32_t region, int32_t value);
	int32_t (*get_lanemode)(int32_t type, int32_t port);
	int32_t (*set_lanemode)(int32_t type, int32_t port, int32_t lanemode);
	int32_t (*set_testcode)(int32_t type, int32_t port, int32_t code);
};

/* mipi dphy device struct */
struct mipi_pdev_s {
	struct os_dev		    osdev;
	struct mipi_dphy_s	    dphy;
	struct mipi_dphy_ops_s *ops;
	struct mipi_phy_s       phy_host[MIPI_HOST_MAX_NUM];
	struct mipi_phy_s       phy_dev[MIPI_DEV_MAX_NUM];
};

/* mipi dphy pll range table */
struct pll_range_table_s {
	uint16_t     low;
	uint16_t     high;
	uint32_t     value;
};
struct pll_sel_table_s {
	uint16_t     osc_freq;
	uint16_t     osc_freq_1p4;
	uint16_t     freq;
	uint32_t     value;
};

enum {
	MIPI_PHY_SYS_PARAM,
	MIPI_PHY_SYS_STATUS_INFO,
	MIPI_PHY_SYS_STATUS_HOST,
	MIPI_PHY_SYS_STATUS_DEV,
	MIPI_PHY_SYS_STATUS_REGS,
	MIPI_PHY_SYS_FAULT_INJECT,
	MIPI_PHY_SYS_NUM,
};

extern int32_t hobot_mipi_phy_ioctl_do(struct mipi_pdev_s *pdev, uint32_t cmd, mipi_ioc_arg_t arg);
extern int32_t hobot_mipi_phy_sys_do(int32_t type, int32_t sub,
		struct mipi_pdev_s *pdev, const char *name, char *buf, int32_t count);

extern void hobot_mipi_phy_remove_do(struct mipi_pdev_s *pdev);
extern int32_t hobot_mipi_phy_probe_do(struct mipi_pdev_s *pdev);
#ifdef CONFIG_HOBOT_FUSA_DIAG
extern int32_t hobot_mipi_phy_stl_setup_do(MIPI_CSI_STL_SETUP setup, const struct mipi_csi_stl_call *call);
#endif

extern struct mipi_pdev_s *hobot_mipi_phy_pdev(void);

#endif //__HOBOT_MIPI_PHY_OPS_H__
