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
 * @file hobot_mipi_csi.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/pinctrl/consumer.h>

#include "hobot_mipi_host_ops.h"
#include "hobot_mipi_host_regs.h"

#include "hobot_mipi_phy.h"

#ifdef CONFIG_HOBOT_IPS_X2
#include "soc/hobot/hobot_ips_x2.h"
#elif defined CONFIG_HOBOT_XJ3
#include "../ips/hobot_dev_ips.h"
#elif defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
#include "../camsys/hobot_dev_camsys.h"
#endif

#ifdef MODULE
#define CONFIG_HOBOT_MIPI_PHY
#ifndef CONFIG_ARCH_HOBOT
#define EX_MODULE
#endif
#endif

#if (defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5)
#include "../vpf/vio_node_api.h"
#ifdef CONFIG_HOBOT_FUSA_DIAG
#include <linux/hobot_diag.h>
#endif
#endif

static const uint32_t g_mh_ipireg[][HOST_IPI_INFO_NUM] = MIPI_HOST_IPI_INFO_REGS;

/* mipi host config check: all uint16_t type */
typedef union {
	mipi_host_cfg_t cfg;
	uint16_t val[MIPI_HOST_CFG_NUM];
} mipi_host_cfg_union_t;
static const char* mipi_host_cfg_name[] = MIPI_HOST_CFG_STRINGS;
static const mipi_host_cfg_union_t mipi_host_cfg_min = {
	.cfg = {
		.phy = 0u,
		.lane = 1u,
		.datatype = 0x00u,
		.fps = 1u,
		.mclk = 0u,
		.mipiclk = 80u,
		.width = 1u,
		.height = 1u,
		.linelenth = 1u,
		.framelenth = 1u,
		.settle = 0u,
		.ppi_pg = 0u,
		.hsaTime = 0u,
		.hbpTime = 0u,
		.hsdTime = 0u,
		.channel_num = 0u,
		.channel_sel = { 0u, 0u, 0u, 0u },
	}
};
static const mipi_host_cfg_union_t mipi_host_cfg_max = {
	.cfg = {
		.phy = 1u,
		.lane = 4u,
		.datatype = 0x13Fu,
		.fps = 240u,
		.mclk = 65535u,
		.mipiclk = 23940u,
#ifdef X5_CHIP
		.width = 4656u,
#else
		.width = 4096u,
#endif
		.height = 4096u,
		.linelenth = 65535u,
		.framelenth = 65535u,
		.settle = 127u,
		.ppi_pg = 65535u,
		.hsaTime = 4095u,
		.hbpTime = 4095u,
		.hsdTime = 4095u,
		.channel_num = 4u,
		.channel_sel = { 3u, 3u, 3u, 3u },
	}
};

/* mipi host error diag event desp */
#define EventIdSwMipiHostErrEventBase		(33u)
#ifdef MIPI_HOST_ERR_DIAG_EVENT
static struct mipi_host_err_event_desp mipi_host_err_events[] = {
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostProbeErr",      EventIdSwMipiHostProbeErr,      DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostIocCheckErr",   EventIdSwMipiHostIocCheckErr,   DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostIocUserErr",    EventIdSwMipiHostIocUserErr,    DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostParamCheckErr", EventIdSwMipiHostParamCheckErr, DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostMutexLockErr",  EventIdSwMipiHostMutexLockErr,  DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostStatusErr",     EventIdSwMipiHostStatusErr,     DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostIomemErr",      EventIdSwMipiHostIomemErr,      DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostHwConfigErr",   EventIdSwMipiHostHwConfigErr,   DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostDphyOpErr",     EventIdSwMipiHostDphyOpErr,     DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostDphyStateErr",  EventIdSwMipiHostDphyStateErr,  DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostIpiOpErr",      EventIdSwMipiHostIpiOpErr,      DiagMsgLevel1),
	MIPI_HOST_ERR_EVENT_D("EventIdSwMipiHostSnrclkSetErr",  EventIdSwMipiHostSnrclkSetErr,  DiagMsgLevel1),
};
#endif

/* host run state */
static const char *g_mh_state[MIPI_STATE_MAX] = MIPI_STATE_NAMES;

/* host pre-op state */
static const char *g_mh_pre_state[MIPI_PRE_STATE_MAX] = MIPI_PRE_STSTE_STRINGS;

#if defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
/* host cfg clock name table */
static const char *g_mh_cfgclk_name[] = {
	"cam_dphy_cfg_rx0",
	"cam_dphy_cfg_rx1",
	"cam_dphy_cfg_rx2",
	"cam_dphy_cfg_rx3",
	"cam_dphy_cfg_rx4",
	"cam_dphy_cfg_rx5",
};

/* host ref clock name table */
static const char *g_mh_refclk_name[] = {
	"cam_dphy_ref",
	"cam_dphy_ref",
	"cam_dphy_ref",
	"cam_dphy_ref",
	"cam_dphy_ref",
	"cam_dphy_ref",
};

/* host ipi clock name table */
static const char *g_mh_ipiclk_name[][MIPI_HOST_HW_IPI_MAX] = {
	{ "cam_sys_ipi0", "cam_sys_ipi1", "cam_sys_ipi2", "cam_sys_ipi3", },
	{ "cam_sys_ipi4", "cam_sys_ipi5", "cam_sys_ipi6", "cam_sys_ipi7", },
	{ "cam_dma_ipi8", "cam_dma_ipi9", "cam_dma_ipi10", "cam_dma_ipi11", },
	{ "cam_dma_ipi12", "cam_dma_ipi13", "cam_dma_ipi14", "cam_dma_ipi15", },
	{ "cam_dma_ipi16", "cam_dma_ipi17", "cam_dma_ipi18", "cam_dma_ipi19", },
	{ "cam_dma_ipi20", "cam_dma_ipi21", "cam_dma_ipi22", "cam_dma_ipi23", },
};
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
/* host sensor mclk clock name table */
static const char *g_mh_snrclk_name[] = {
	"clk_24m_out0",
	"clk_24m_out1",
	"clk_24m_out2",
	"clk_24m_out3",
	"clk_24m_out4",
	"clk_24m_out5",
};
#endif
#elif defined X5_CHIP
static const char *g_mh_cfgclk_name[] = {
	"csi0_cfg",
	"csi1_cfg",
	"csi2_cfg",
	"csi3_cfg",
};
static const char *g_mh_refclk_name[] = {
};
static const char *g_mh_ipiclk_name[][MIPI_HOST_HW_IPI_MAX] = {
        {"csi0_ipi_pixel_clk", "csi0_ipi_pixel_clk", "csi0_ipi_pixel_clk", "csi0_ipi_pixel_clk",},
        {"csi1_ipi_pixel_clk", "csi1_ipi_pixel_clk", "csi1_ipi_pixel_clk", "csi1_ipi_pixel_clk",},
        {"csi2_ipi_pixel_clk", "csi2_ipi_pixel_clk", "csi2_ipi_pixel_clk", "csi2_ipi_pixel_clk",},
        {"csi3_ipi_pixel_clk", "csi3_ipi_pixel_clk", "csi3_ipi_pixel_clk", "csi3_ipi_pixel_clk",},
};

static const char *g_mh_pclk_name[] = {
       "csi0_pclk",
       "csi1_pclk",
       "csi2_pclk",
       "csi3_pclk",
};
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
/* host sensor mclk clock name table */
static const char *g_mh_snrclk_name[] = {
       "sensor0_mclk",
       "sensor1_mclk",
       "sensor2_mclk",
       "sensor3_mclk",
};
#endif
#else
/* host cfg clock name table */
static const char *g_mh_cfgclk_name[] = {
	"mipi_cfg_host",
	"mipi_cfg_host",
	"mipi_cfg_host",
	"mipi_cfg_host",
	"mipi_cfg_host",
	"mipi_cfg_host",
};

/* host ref clock name table */
static const char *g_mh_refclk_name[] = {
	"mipi_host_ref",
	"mipi_host_ref",
	"mipi_host_ref",
	"mipi_host_ref",
	"mipi_host_ref",
	"mipi_host_ref",
};

/* host ipi clock name table */
static const char *g_mh_ipiclk_name[][MIPI_HOST_HW_IPI_MAX] = {
	{ "mipi_rx0_ipi", "mipi_rx0_ipi", "mipi_rx0_ipi", "mipi_rx0_ipi", },
	{ "mipi_rx1_ipi", "mipi_rx1_ipi", "mipi_rx1_ipi", "mipi_rx1_ipi", },
	{ "mipi_rx2_ipi", "mipi_rx2_ipi", "mipi_rx2_ipi", "mipi_rx2_ipi", },
	{ "mipi_rx3_ipi", "mipi_rx3_ipi", "mipi_rx3_ipi", "mipi_rx3_ipi", },
	{ "mipi_rx4_ipi", "mipi_rx4_ipi", "mipi_rx4_ipi", "mipi_rx4_ipi", },
	{ "mipi_rx5_ipi", "mipi_rx5_ipi", "mipi_rx5_ipi", "mipi_rx5_ipi", },
};

#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
/* host sensor mclk clock name table */
static const char *g_mh_snrclk_name[] = {
	"sensor0_mclk",
	"sensor1_mclk",
	"sensor2_mclk",
	"sensor3_mclk",
	"sensor4_mclk",
	"sensor5_mclk",
};
#endif
#endif

#define MIPI_HOST_CFGCLK_NUM (sizeof(g_mh_cfgclk_name)/sizeof(g_mh_cfgclk_name[0]))
#define MIPI_HOST_REFCLK_NUM (sizeof(g_mh_refclk_name)/sizeof(g_mh_refclk_name[0]))
#define MIPI_HOST_IPICLK_NUM (sizeof(g_mh_ipiclk_name)/sizeof(g_mh_ipiclk_name[0]))
#define MIPI_HOST_SNRCLK_NUM (sizeof(g_mh_snrclk_name)/sizeof(g_mh_snrclk_name[0]))
#define MIPI_HOST_PCLK_NUM   (sizeof(g_mh_pclk_name)/sizeof(g_mh_pclk_name[0]))

/* run params name of sysfs, see: struct mipi_host_param_s */
static const char *g_mh_param_names[MIPI_HOST_PARAMS_NUM] = MIPI_HOST_PARAM_STRINGS;

#if MIPI_HOST_INT_DBG
/* interrupt error counts of sysfs, see: struct mipi_host_icnt_s */
static const char *g_mh_icnt_names[] = MIPI_HOST_ICNT_STRINGS;

/* host interrupt regs table for V1P3 */
static const struct mipi_host_ireg_s mh_int_regs_1p3[] = {
	{ 1, MIPI_HOST_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_PHY_FATAL,
		0x0000000fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
		  [2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
		  [4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
		  [6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
		  [8 ... 31] = NULL }
#endif
	},
	{ 2, MIPI_HOST_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_PKT_FATAL,
		0x0000010fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "vc0_err_crc", [1] = "vc1_err_crc",
		  [2] = "vc2_err_crc", [3] = "vc3_err_crc",
		  [ 4 ... 15] = NULL,
		  [16] = "err_ecc_double",
		  [17 ... 31] = NULL }
#endif
	},
	{ 3, MIPI_HOST_INT_FRM_FATAL, REG_MIPI_HOST_INT_ST_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FRAME_FATAL,
		0x000f0f0fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
		  [2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  [4 ... 7] = NULL,
		  [8] = "err_f_seq_vc0", [9] = "err_f_seq_vc1",
		  [10] = "err_f_seq_vc2", [11] = "err_f_seq_vc3",
		  [12 ... 15] = NULL,
		  [16] = "err_frame_data_vc0", [17] = "err_frame_data_vc1",
		  [18] = "err_frame_data_vc2", [19] = "err_frame_data_vc3",
		  [20 ... 31] = NULL }

#endif
	},
	{ 10, MIPI_HOST_INT_PHY, REG_MIPI_HOST_INT_ST_PHY,
		REG_MIPI_HOST_INT_MSK_PHY, REG_MIPI_HOST_INT_FORCE_PHY,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
		  [2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
		  [4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
		  [6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
		  [8 ... 15] = NULL,
		  [16] = "phy_erresc_0", [17] = "phy_erresc_1",
		  [18] = "phy_erresc_2", [19] = "phy_erresc_3",
		  [20] = "phy_erresc_4", [21] = "phy_erresc_5",
		  [22] = "phy_erresc_6", [23] = "phy_erresc_7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 12, MIPI_HOST_1P4_INT_LINE, REG_MIPI_HOST_INT_ST_LINE,
		REG_MIPI_HOST_INT_MSK_LINE, REG_MIPI_HOST_INT_FORCE_LINE,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
		  [2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
		  [4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
		  [6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
		  [8 ... 15] = NULL,
		  [16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
		  [18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
		  [20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
		  [22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 13, MIPI_HOST_INT_IPI, REG_MIPI_HOST_INT_ST_IPI,
		REG_MIPI_HOST_INT_MSK_IPI, REG_MIPI_HOST_INT_FORCE_IPI,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 14, MIPI_HOST_INT_IPI2, REG_MIPI_HOST_INT_ST_IPI2,
		REG_MIPI_HOST_INT_MSK_IPI2, REG_MIPI_HOST_INT_FORCE_IPI2,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 15, MIPI_HOST_INT_IPI3, REG_MIPI_HOST_INT_ST_IPI3,
		REG_MIPI_HOST_INT_MSK_IPI3, REG_MIPI_HOST_INT_FORCE_IPI3,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 16, MIPI_HOST_INT_IPI4, REG_MIPI_HOST_INT_ST_IPI4,
		REG_MIPI_HOST_INT_MSK_IPI4, REG_MIPI_HOST_INT_FORCE_IPI4,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	}
};
#define MIPI_HOST_IREG_NUM_1P3 (sizeof(mh_int_regs_1p3)/sizeof(mh_int_regs_1p3[0]))

/* host interrupt regs table for V1P4 */
static const struct mipi_host_ireg_s mh_int_regs_1p4[] = {
	{ 1, MIPI_HOST_1P4_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_PHY_FATAL,
		0x0000000fU, /* ignore err_deskew */
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
		  [2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
		  [4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
		  [6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
		  [8] = "err_deskew",
		  [9 ... 31] = NULL }
#endif
	},
	{ 2, MIPI_HOST_1P4_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_PKT_FATAL,
		0x00000001U,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_double",
		  [1 ... 31] = NULL }
#endif
	},
	{ 4, MIPI_HOST_1P4_INT_BNDRY_FRM_FATAL, REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_BNDRY_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
		  [2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  [4] = "err_f_bndry_match_vc4", [5] = "err_f_bndry_match_vc5",
		  [6] = "err_f_bndry_match_vc6", [7] = "err_f_bndry_match_vc7",
		  [8] = "err_f_bndry_match_vc8", [9] = "err_f_bndry_match_vc9",
		  [10] = "err_f_bndry_match_vc10", [11] = "err_f_bndry_match_vc11",
		  [12] = "err_f_bndry_match_vc12", [13] = "err_f_bndry_match_vc13",
		  [14] = "err_f_bndry_match_vc14", [15] = "err_f_bndry_match_vc15",
		  [16] = "err_f_bndry_match_vc16", [17] = "err_f_bndry_match_vc17",
		  [18] = "err_f_bndry_match_vc18", [19] = "err_f_bndry_match_vc19",
		  [20] = "err_f_bndry_match_vc20", [21] = "err_f_bndry_match_vc21",
		  [22] = "err_f_bndry_match_vc22", [23] = "err_f_bndry_match_vc23",
		  [24] = "err_f_bndry_match_vc24", [25] = "err_f_bndry_match_vc25",
		  [26] = "err_f_bndry_match_vc26", [27] = "err_f_bndry_match_vc27",
		  [28] = "err_f_bndry_match_vc28", [29] = "err_f_bndry_match_vc29",
		  [30] = "err_f_bndry_match_vc30", [31] = "err_f_bndry_match_vc31" }
#endif
	},
	{ 5, MIPI_HOST_1P4_INT_SEQ_FRM_FATAL, REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_SEQ_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_seq_vc0", [1] = "err_f_seq_vc1",
		  [2] = "err_f_seq_vc2", [3] = "err_f_seq_vc3",
		  [4] = "err_f_seq_vc4", [5] = "err_f_seq_vc5",
		  [6] = "err_f_seq_vc6", [7] = "err_f_seq_vc7",
		  [8] = "err_f_seq_vc8", [9] = "err_f_seq_vc9",
		  [10] = "err_f_seq_vc10", [11] = "err_f_seq_vc11",
		  [12] = "err_f_seq_vc12", [13] = "err_f_seq_vc13",
		  [14] = "err_f_seq_vc14", [15] = "err_f_seq_vc15",
		  [16] = "err_f_seq_vc16", [17] = "err_f_seq_vc17",
		  [18] = "err_f_seq_vc18", [19] = "err_f_seq_vc19",
		  [20] = "err_f_seq_vc20", [21] = "err_f_seq_vc21",
		  [22] = "err_f_seq_vc22", [23] = "err_f_seq_vc23",
		  [24] = "err_f_seq_vc24", [25] = "err_f_seq_vc25",
		  [26] = "err_f_seq_vc26", [27] = "err_f_seq_vc27",
		  [28] = "err_f_seq_vc28", [29] = "err_f_seq_vc29",
		  [30] = "err_f_seq_vc30", [31] = "err_f_seq_vc31" }
#endif
	},
	{ 6, MIPI_HOST_1P4_INT_CRC_FRM_FATAL, REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_CRC_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_frame_data_vc0", [1] = "err_frame_data_vc1",
		  [2] = "err_frame_data_vc2", [3] = "err_frame_data_vc3",
		  [4] = "err_frame_data_vc4", [5] = "err_frame_data_vc5",
		  [6] = "err_frame_data_vc6", [7] = "err_frame_data_vc7",
		  [8] = "err_frame_data_vc8", [9] = "err_frame_data_vc9",
		  [10] = "err_frame_data_vc10", [11] = "err_frame_data_vc11",
		  [12] = "err_frame_data_vc12", [13] = "err_frame_data_vc13",
		  [14] = "err_frame_data_vc14", [15] = "err_frame_data_vc15",
		  [16] = "err_frame_data_vc16", [17] = "err_frame_data_vc17",
		  [18] = "err_frame_data_vc18", [19] = "err_frame_data_vc19",
		  [20] = "err_frame_data_vc20", [21] = "err_frame_data_vc21",
		  [22] = "err_frame_data_vc22", [23] = "err_frame_data_vc23",
		  [24] = "err_frame_data_vc24", [25] = "err_frame_data_vc25",
		  [26] = "err_frame_data_vc26", [27] = "err_frame_data_vc27",
		  [28] = "err_frame_data_vc28", [29] = "err_frame_data_vc29",
		  [30] = "err_frame_data_vc30", [31] = "err_frame_data_vc31" }
#endif
	},
	{ 7, MIPI_HOST_1P4_INT_PLD_CRC_FATAL, REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL,
		REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL, REG_MIPI_HOST_INT_FORCE_PLD_CRC_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_crc_vc0", [1] = "err_crc_vc1",
		  [2] = "err_crc_vc2", [3] = "err_crc_vc3",
		  [4] = "err_crc_vc4", [5] = "err_crc_vc5",
		  [6] = "err_crc_vc6", [7] = "err_crc_vc7",
		  [8] = "err_crc_vc8", [9] = "err_crc_vc9",
		  [10] = "err_crc_vc10", [11] = "err_crc_vc11",
		  [12] = "err_crc_vc12", [13] = "err_crc_vc13",
		  [14] = "err_crc_vc14", [15] = "err_crc_vc15",
		  [16] = "err_crc_vc16", [17] = "err_crc_vc17",
		  [18] = "err_crc_vc18", [19] = "err_crc_vc19",
		  [20] = "err_crc_vc20", [21] = "err_crc_vc21",
		  [22] = "err_crc_vc22", [23] = "err_crc_vc23",
		  [24] = "err_crc_vc24", [25] = "err_crc_vc25",
		  [26] = "err_crc_vc26", [27] = "err_crc_vc27",
		  [28] = "err_crc_vc28", [29] = "err_crc_vc29",
		  [30] = "err_crc_vc30", [31] = "err_crc_vc31" }
#endif
	},
	{ 8, MIPI_HOST_1P4_INT_DATA_ID, REG_MIPI_HOST_INT_ST_DATA_ID,
		REG_MIPI_HOST_INT_MSK_DATA_ID, REG_MIPI_HOST_INT_FORCE_DATA_ID,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_id_vc0", [1] = "err_id_vc1",
		  [2] = "err_id_vc2", [3] = "err_id_vc3",
		  [4] = "err_id_vc4", [5] = "err_id_vc5",
		  [6] = "err_id_vc6", [7] = "err_id_vc7",
		  [8] = "err_id_vc8", [9] = "err_id_vc9",
		  [10] = "err_id_vc10", [11] = "err_id_vc11",
		  [12] = "err_id_vc12", [13] = "err_id_vc13",
		  [14] = "err_id_vc14", [15] = "err_id_vc15",
		  [16] = "err_id_vc16", [17] = "err_id_vc17",
		  [18] = "err_id_vc18", [19] = "err_id_vc19",
		  [20] = "err_id_vc20", [21] = "err_id_vc21",
		  [22] = "err_id_vc22", [23] = "err_id_vc23",
		  [24] = "err_id_vc24", [25] = "err_id_vc25",
		  [26] = "err_id_vc26", [27] = "err_id_vc27",
		  [28] = "err_id_vc28", [29] = "err_id_vc29",
		  [30] = "err_id_vc30", [31] = "err_id_vc31" }
#endif
	},
	{ 9, MIPI_HOST_1P4_INT_ECC_CORRECTED, REG_MIPI_HOST_INT_ST_ECC_CORRECT,
		REG_MIPI_HOST_INT_MSK_ECC_CORRECT, REG_MIPI_HOST_INT_FORCE_ECC_CORRECT,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_corrected0", [1] = "err_ecc_corrected1",
		  [2] = "err_ecc_corrected2", [3] = "err_ecc_corrected3",
		  [4] = "err_ecc_corrected4", [5] = "err_ecc_corrected5",
		  [6] = "err_ecc_corrected6", [7] = "err_ecc_corrected7",
		  [8] = "err_ecc_corrected8", [9] = "err_ecc_corrected9",
		  [10] = "err_ecc_corrected10", [11] = "err_ecc_corrected11",
		  [12] = "err_ecc_corrected12", [13] = "err_ecc_corrected13",
		  [14] = "err_ecc_corrected14", [15] = "err_ecc_corrected15",
		  [16] = "err_ecc_corrected16", [17] = "err_ecc_corrected17",
		  [18] = "err_ecc_corrected18", [19] = "err_ecc_corrected19",
		  [20] = "err_ecc_corrected20", [21] = "err_ecc_corrected21",
		  [22] = "err_ecc_corrected22", [23] = "err_ecc_corrected23",
		  [24] = "err_ecc_corrected24", [25] = "err_ecc_corrected25",
		  [26] = "err_ecc_corrected26", [27] = "err_ecc_corrected27",
		  [28] = "err_ecc_corrected28", [29] = "err_ecc_corrected29",
		  [30] = "err_ecc_corrected30", [31] = "err_ecc_corrected31" }
#endif
	},
	{ 10, MIPI_HOST_1P4_INT_PHY, REG_MIPI_HOST_INT_ST_PHY,
		REG_MIPI_HOST_INT_MSK_PHY, REG_MIPI_HOST_INT_FORCE_PHY,
		0x000f000fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
		  [2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
		  [4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
		  [6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
		  [8 ... 15] = NULL,
		  [16] = "phy_erresc_0", [17] = "phy_erresc_1",
		  [18] = "phy_erresc_2", [19] = "phy_erresc_3",
		  [20] = "phy_erresc_4", [21] = "phy_erresc_5",
		  [22] = "phy_erresc_6", [23] = "phy_erresc_7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 12, MIPI_HOST_1P4_INT_LINE, REG_MIPI_HOST_INT_ST_LINE,
		REG_MIPI_HOST_INT_MSK_LINE, REG_MIPI_HOST_INT_FORCE_LINE,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
		  [2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
		  [4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
		  [6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
		  [8 ... 15] = NULL,
		  [16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
		  [18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
		  [20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
		  [22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 13, MIPI_HOST_1P4_INT_IPI, REG_MIPI_HOST_INT_ST_IPI,
		REG_MIPI_HOST_INT_MSK_IPI, REG_MIPI_HOST_INT_FORCE_IPI,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 14, MIPI_HOST_1P4_INT_IPI2, REG_MIPI_HOST_INT_ST_IPI2,
		REG_MIPI_HOST_INT_MSK_IPI2, REG_MIPI_HOST_INT_FORCE_IPI2,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 15, MIPI_HOST_1P4_INT_IPI3, REG_MIPI_HOST_INT_ST_IPI3,
		REG_MIPI_HOST_INT_MSK_IPI3, REG_MIPI_HOST_INT_FORCE_IPI3,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 16, MIPI_HOST_1P4_INT_IPI4, REG_MIPI_HOST_INT_ST_IPI4,
		REG_MIPI_HOST_INT_MSK_IPI4, REG_MIPI_HOST_INT_FORCE_IPI4,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	}
};
#define MIPI_HOST_IREG_NUM_1P4 (sizeof(mh_int_regs_1p4)/sizeof(mh_int_regs_1p4[0]))

static const struct mipi_host_ireg_s mh_int_regs_1p4ap[] = {
	{ 1, MIPI_HOST_1P4AP_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_FAP_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PHY_FATAL,
		0x0000000fU, /* ignore err_deskew */
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
		  [2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
		  [4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
		  [6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
		  [8] = "err_deskew",
		  [9 ... 31] = NULL }
#endif
	},
	{ 2, MIPI_HOST_1P4AP_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_FAP_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PKT_FATAL,
		0x00000001U,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_double",
		  [1 ... 31] = NULL }
#endif
	},
	{ 4, MIPI_HOST_1P4AP_INT_BNDRY_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_BNDRY_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_BNDRY_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_BNDRY_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
		  [2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  [4] = "err_f_bndry_match_vc4", [5] = "err_f_bndry_match_vc5",
		  [6] = "err_f_bndry_match_vc6", [7] = "err_f_bndry_match_vc7",
		  [8] = "err_f_bndry_match_vc8", [9] = "err_f_bndry_match_vc9",
		  [10] = "err_f_bndry_match_vc10", [11] = "err_f_bndry_match_vc11",
		  [12] = "err_f_bndry_match_vc12", [13] = "err_f_bndry_match_vc13",
		  [14] = "err_f_bndry_match_vc14", [15] = "err_f_bndry_match_vc15",
		  [16] = "err_f_bndry_match_vc16", [17] = "err_f_bndry_match_vc17",
		  [18] = "err_f_bndry_match_vc18", [19] = "err_f_bndry_match_vc19",
		  [20] = "err_f_bndry_match_vc20", [21] = "err_f_bndry_match_vc21",
		  [22] = "err_f_bndry_match_vc22", [23] = "err_f_bndry_match_vc23",
		  [24] = "err_f_bndry_match_vc24", [25] = "err_f_bndry_match_vc25",
		  [26] = "err_f_bndry_match_vc26", [27] = "err_f_bndry_match_vc27",
		  [28] = "err_f_bndry_match_vc28", [29] = "err_f_bndry_match_vc29",
		  [30] = "err_f_bndry_match_vc30", [31] = "err_f_bndry_match_vc31" }
#endif
	},
	{ 5, MIPI_HOST_1P4AP_INT_SEQ_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_SEQ_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_SEQ_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_SEQ_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_seq_vc0", [1] = "err_f_seq_vc1",
		  [2] = "err_f_seq_vc2", [3] = "err_f_seq_vc3",
		  [4] = "err_f_seq_vc4", [5] = "err_f_seq_vc5",
		  [6] = "err_f_seq_vc6", [7] = "err_f_seq_vc7",
		  [8] = "err_f_seq_vc8", [9] = "err_f_seq_vc9",
		  [10] = "err_f_seq_vc10", [11] = "err_f_seq_vc11",
		  [12] = "err_f_seq_vc12", [13] = "err_f_seq_vc13",
		  [14] = "err_f_seq_vc14", [15] = "err_f_seq_vc15",
		  [16] = "err_f_seq_vc16", [17] = "err_f_seq_vc17",
		  [18] = "err_f_seq_vc18", [19] = "err_f_seq_vc19",
		  [20] = "err_f_seq_vc20", [21] = "err_f_seq_vc21",
		  [22] = "err_f_seq_vc22", [23] = "err_f_seq_vc23",
		  [24] = "err_f_seq_vc24", [25] = "err_f_seq_vc25",
		  [26] = "err_f_seq_vc26", [27] = "err_f_seq_vc27",
		  [28] = "err_f_seq_vc28", [29] = "err_f_seq_vc29",
		  [30] = "err_f_seq_vc30", [31] = "err_f_seq_vc31" }
#endif
	},
	{ 6, MIPI_HOST_1P4AP_INT_CRC_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_CRC_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_CRC_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_CRC_FRAME_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_frame_data_vc0", [1] = "err_frame_data_vc1",
		  [2] = "err_frame_data_vc2", [3] = "err_frame_data_vc3",
		  [4] = "err_frame_data_vc4", [5] = "err_frame_data_vc5",
		  [6] = "err_frame_data_vc6", [7] = "err_frame_data_vc7",
		  [8] = "err_frame_data_vc8", [9] = "err_frame_data_vc9",
		  [10] = "err_frame_data_vc10", [11] = "err_frame_data_vc11",
		  [12] = "err_frame_data_vc12", [13] = "err_frame_data_vc13",
		  [14] = "err_frame_data_vc14", [15] = "err_frame_data_vc15",
		  [16] = "err_frame_data_vc16", [17] = "err_frame_data_vc17",
		  [18] = "err_frame_data_vc18", [19] = "err_frame_data_vc19",
		  [20] = "err_frame_data_vc20", [21] = "err_frame_data_vc21",
		  [22] = "err_frame_data_vc22", [23] = "err_frame_data_vc23",
		  [24] = "err_frame_data_vc24", [25] = "err_frame_data_vc25",
		  [26] = "err_frame_data_vc26", [27] = "err_frame_data_vc27",
		  [28] = "err_frame_data_vc28", [29] = "err_frame_data_vc29",
		  [30] = "err_frame_data_vc30", [31] = "err_frame_data_vc31" }
#endif
	},
	{ 7, MIPI_HOST_1P4AP_INT_PLD_CRC_FATAL, REG_MIPI_HOST_INT_ST_FAP_PLD_CRC_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PLD_CRC_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PLD_CRC_FATAL,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_crc_vc0", [1] = "err_crc_vc1",
		  [2] = "err_crc_vc2", [3] = "err_crc_vc3",
		  [4] = "err_crc_vc4", [5] = "err_crc_vc5",
		  [6] = "err_crc_vc6", [7] = "err_crc_vc7",
		  [8] = "err_crc_vc8", [9] = "err_crc_vc9",
		  [10] = "err_crc_vc10", [11] = "err_crc_vc11",
		  [12] = "err_crc_vc12", [13] = "err_crc_vc13",
		  [14] = "err_crc_vc14", [15] = "err_crc_vc15",
		  [16] = "err_crc_vc16", [17] = "err_crc_vc17",
		  [18] = "err_crc_vc18", [19] = "err_crc_vc19",
		  [20] = "err_crc_vc20", [21] = "err_crc_vc21",
		  [22] = "err_crc_vc22", [23] = "err_crc_vc23",
		  [24] = "err_crc_vc24", [25] = "err_crc_vc25",
		  [26] = "err_crc_vc26", [27] = "err_crc_vc27",
		  [28] = "err_crc_vc28", [29] = "err_crc_vc29",
		  [30] = "err_crc_vc30", [31] = "err_crc_vc31" }
#endif
	},
	{ 8, MIPI_HOST_1P4AP_INT_DATA_ID, REG_MIPI_HOST_INT_ST_FAP_DATA_ID,
		REG_MIPI_HOST_INT_MSK_FAP_DATA_ID, REG_MIPI_HOST_INT_FORCE_FAP_DATA_ID,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_id_vc0", [1] = "err_id_vc1",
		  [2] = "err_id_vc2", [3] = "err_id_vc3",
		  [4] = "err_id_vc4", [5] = "err_id_vc5",
		  [6] = "err_id_vc6", [7] = "err_id_vc7",
		  [8] = "err_id_vc8", [9] = "err_id_vc9",
		  [10] = "err_id_vc10", [11] = "err_id_vc11",
		  [12] = "err_id_vc12", [13] = "err_id_vc13",
		  [14] = "err_id_vc14", [15] = "err_id_vc15",
		  [16] = "err_id_vc16", [17] = "err_id_vc17",
		  [18] = "err_id_vc18", [19] = "err_id_vc19",
		  [20] = "err_id_vc20", [21] = "err_id_vc21",
		  [22] = "err_id_vc22", [23] = "err_id_vc23",
		  [24] = "err_id_vc24", [25] = "err_id_vc25",
		  [26] = "err_id_vc26", [27] = "err_id_vc27",
		  [28] = "err_id_vc28", [29] = "err_id_vc29",
		  [30] = "err_id_vc30", [31] = "err_id_vc31" }
#endif
	},
	{ 9, MIPI_HOST_1P4AP_INT_ECC_CORRECTED, REG_MIPI_HOST_INT_ST_FAP_ECC_CORRECT,
		REG_MIPI_HOST_INT_MSK_FAP_ECC_CORRECT, REG_MIPI_HOST_INT_FORCE_FAP_ECC_CORRECT,
		0x0000ffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_corrected0", [1] = "err_ecc_corrected1",
		  [2] = "err_ecc_corrected2", [3] = "err_ecc_corrected3",
		  [4] = "err_ecc_corrected4", [5] = "err_ecc_corrected5",
		  [6] = "err_ecc_corrected6", [7] = "err_ecc_corrected7",
		  [8] = "err_ecc_corrected8", [9] = "err_ecc_corrected9",
		  [10] = "err_ecc_corrected10", [11] = "err_ecc_corrected11",
		  [12] = "err_ecc_corrected12", [13] = "err_ecc_corrected13",
		  [14] = "err_ecc_corrected14", [15] = "err_ecc_corrected15",
		  [16] = "err_ecc_corrected16", [17] = "err_ecc_corrected17",
		  [18] = "err_ecc_corrected18", [19] = "err_ecc_corrected19",
		  [20] = "err_ecc_corrected20", [21] = "err_ecc_corrected21",
		  [22] = "err_ecc_corrected22", [23] = "err_ecc_corrected23",
		  [24] = "err_ecc_corrected24", [25] = "err_ecc_corrected25",
		  [26] = "err_ecc_corrected26", [27] = "err_ecc_corrected27",
		  [28] = "err_ecc_corrected28", [29] = "err_ecc_corrected29",
		  [30] = "err_ecc_corrected30", [31] = "err_ecc_corrected31" }
#endif
	},
	{ 10, MIPI_HOST_1P4AP_INT_PHY, REG_MIPI_HOST_INT_ST_FAP_PHY,
		REG_MIPI_HOST_INT_MSK_FAP_PHY, REG_MIPI_HOST_INT_FORCE_FAP_PHY,
		0x000f000fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
		  [2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
		  [4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
		  [6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
		  [8 ... 15] = NULL,
		  [16] = "phy_erresc_0", [17] = "phy_erresc_1",
		  [18] = "phy_erresc_2", [19] = "phy_erresc_3",
		  [20] = "phy_erresc_4", [21] = "phy_erresc_5",
		  [22] = "phy_erresc_6", [23] = "phy_erresc_7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 12, MIPI_HOST_1P4AP_INT_LINE, REG_MIPI_HOST_INT_ST_FAP_LINE,
		REG_MIPI_HOST_INT_MSK_FAP_LINE, REG_MIPI_HOST_INT_FORCE_FAP_LINE,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
		  [2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
		  [4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
		  [6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
		  [8 ... 15] = NULL,
		  [16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
		  [18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
		  [20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
		  [22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 13, MIPI_HOST_1P4AP_INT_IPI, REG_MIPI_HOST_INT_ST_FAP_IPI,
		REG_MIPI_HOST_INT_MSK_FAP_IPI, REG_MIPI_HOST_INT_FORCE_FAP_IPI,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 14, MIPI_HOST_1P4AP_INT_IPI2, REG_MIPI_HOST_INT_ST_FAP_IPI2,
		REG_MIPI_HOST_INT_MSK_FAP_IPI2, REG_MIPI_HOST_INT_FORCE_FAP_IPI2,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 15, MIPI_HOST_1P4AP_INT_IPI3, REG_MIPI_HOST_INT_ST_FAP_IPI3,
		REG_MIPI_HOST_INT_MSK_FAP_IPI3, REG_MIPI_HOST_INT_FORCE_FAP_IPI3,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 16, MIPI_HOST_1P4AP_INT_IPI4, REG_MIPI_HOST_INT_ST_FAP_IPI4,
		REG_MIPI_HOST_INT_MSK_FAP_IPI4, REG_MIPI_HOST_INT_FORCE_FAP_IPI4,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6 ... 31] = NULL }
#endif
	},
	{ 17, MIPI_HOST_1P4AP_INT_ST_AP_GENERIC, REG_MIPI_HOST_INT_ST_AP_GENERIC,
		REG_MIPI_HOST_INT_MSK_AP_GENERIC, REG_MIPI_HOST_INT_FORCE_AP_GENERIC,
#ifdef CONFIG_ARCH_ZYNQMP
		0x000fffccU,
#else
		0x000fffcfU,
#endif
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "apb_ap_err_parerr", [1] = "apb_ap_err_intparerr",
		  [2] = "reg_bank_ap_err_parerr", [3] = "eg_bank_ap_err_scycerr",
		  [4] = "de_skew_ap_err", [5] = "pipeline_delay_ap_err",
		  [6] = "descrambler_ap_err", [7] = "phy_adapter_ap_err",
		  [8] = "packet_analyzer_ap_err_parerr", [9] = "packet_analyzer_ap_err_rederr",
		  [10] = "prep_outs_ap_err_parerr", [11] = "prep_outs_ap_err_rederr",
		  [12] = "err_msgr_ap_err", [13] = "err_handler_ap_err",
		  [14] = "synchronizer_fpclk_ap_err", [15] = "synchronizer_rxbyteclkhs_ap_err",
		  [16] = "synchronizer_pixclk_ap_err", [17] = "synchronizer_pixclk_2if_ap_err",
		  [18] = "synchronizer_pixclk_3if_ap_err", [19] = "synchronizer_pixclk_4if_ap_err",
		  [20 ... 31] = NULL }
#endif
	},
	{ 18, MIPI_HOST_1P4AP_INT_ST_AP_IPI, REG_MIPI_HOST_INT_ST_AP_IPI,
		REG_MIPI_HOST_INT_MSK_AP_IPI, REG_MIPI_HOST_INT_FORCE_AP_IPI,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 19, MIPI_HOST_1P4AP_INT_ST_AP_IPI2, REG_MIPI_HOST_INT_ST_AP_IPI2,
		REG_MIPI_HOST_INT_MSK_AP_IPI2, REG_MIPI_HOST_INT_FORCE_AP_IPI2,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 20, MIPI_HOST_1P4AP_INT_ST_AP_IPI3, REG_MIPI_HOST_INT_ST_AP_IPI3,
		REG_MIPI_HOST_INT_MSK_AP_IPI3, REG_MIPI_HOST_INT_FORCE_AP_IPI3,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 21, MIPI_HOST_1P4AP_INT_ST_AP_IPI4, REG_MIPI_HOST_INT_ST_AP_IPI4,
		REG_MIPI_HOST_INT_MSK_AP_IPI4, REG_MIPI_HOST_INT_FORCE_AP_IPI4,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	}
};
#define MIPI_HOST_IREG_NUM_1P4AP (sizeof(mh_int_regs_1p4ap)/sizeof(mh_int_regs_1p4ap[0]))

/* host interrupt regs table for V1P5 */
static const struct mipi_host_ireg_s mh_int_regs_1p5[] = {
	{ 1, MIPI_HOST_1P5_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_PHY_FATAL,
		0x0007010fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
		  [2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
		  [4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
		  [6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
		  [8] = "err_deskew",
		  [9 ... 15] = NULL,
		  [16] = "phy_rxinvalidcodehs_0", [17] = "phy_rxinvalidcodehs_1",
		  [18] = "phy_rxinvalidcodehs_2",
		  [19 ... 31] = NULL }
#endif
	},
	{ 2, MIPI_HOST_1P5_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_PKT_FATAL,
		0x00000003U,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_double", [1] = "shorter_payload",
		  [2 ... 31] = NULL }
#endif
	},
	{ 4, MIPI_HOST_1P5_INT_BNDRY_FRM_FATAL, REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_BNDRY_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
		  [2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  [4] = "err_f_bndry_match_vc4", [5] = "err_f_bndry_match_vc5",
		  [6] = "err_f_bndry_match_vc6", [7] = "err_f_bndry_match_vc7",
		  [8] = "err_f_bndry_match_vc8", [9] = "err_f_bndry_match_vc9",
		  [10] = "err_f_bndry_match_vc10", [11] = "err_f_bndry_match_vc11",
		  [12] = "err_f_bndry_match_vc12", [13] = "err_f_bndry_match_vc13",
		  [14] = "err_f_bndry_match_vc14", [15] = "err_f_bndry_match_vc15",
		  [16] = "err_f_bndry_match_vc16", [17] = "err_f_bndry_match_vc17",
		  [18] = "err_f_bndry_match_vc18", [19] = "err_f_bndry_match_vc19",
		  [20] = "err_f_bndry_match_vc20", [21] = "err_f_bndry_match_vc21",
		  [22] = "err_f_bndry_match_vc22", [23] = "err_f_bndry_match_vc23",
		  [24] = "err_f_bndry_match_vc24", [25] = "err_f_bndry_match_vc25",
		  [26] = "err_f_bndry_match_vc26", [27] = "err_f_bndry_match_vc27",
		  [28] = "err_f_bndry_match_vc28", [29] = "err_f_bndry_match_vc29",
		  [30] = "err_f_bndry_match_vc30", [31] = "err_f_bndry_match_vc31" }
#endif
	},
	{ 5, MIPI_HOST_1P5_INT_SEQ_FRM_FATAL, REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_SEQ_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_seq_vc0", [1] = "err_f_seq_vc1",
		  [2] = "err_f_seq_vc2", [3] = "err_f_seq_vc3",
		  [4] = "err_f_seq_vc4", [5] = "err_f_seq_vc5",
		  [6] = "err_f_seq_vc6", [7] = "err_f_seq_vc7",
		  [8] = "err_f_seq_vc8", [9] = "err_f_seq_vc9",
		  [10] = "err_f_seq_vc10", [11] = "err_f_seq_vc11",
		  [12] = "err_f_seq_vc12", [13] = "err_f_seq_vc13",
		  [14] = "err_f_seq_vc14", [15] = "err_f_seq_vc15",
		  [16] = "err_f_seq_vc16", [17] = "err_f_seq_vc17",
		  [18] = "err_f_seq_vc18", [19] = "err_f_seq_vc19",
		  [20] = "err_f_seq_vc20", [21] = "err_f_seq_vc21",
		  [22] = "err_f_seq_vc22", [23] = "err_f_seq_vc23",
		  [24] = "err_f_seq_vc24", [25] = "err_f_seq_vc25",
		  [26] = "err_f_seq_vc26", [27] = "err_f_seq_vc27",
		  [28] = "err_f_seq_vc28", [29] = "err_f_seq_vc29",
		  [30] = "err_f_seq_vc30", [31] = "err_f_seq_vc31" }
#endif
	},
	{ 6, MIPI_HOST_1P5_INT_CRC_FRM_FATAL, REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_CRC_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_frame_data_vc0", [1] = "err_frame_data_vc1",
		  [2] = "err_frame_data_vc2", [3] = "err_frame_data_vc3",
		  [4] = "err_frame_data_vc4", [5] = "err_frame_data_vc5",
		  [6] = "err_frame_data_vc6", [7] = "err_frame_data_vc7",
		  [8] = "err_frame_data_vc8", [9] = "err_frame_data_vc9",
		  [10] = "err_frame_data_vc10", [11] = "err_frame_data_vc11",
		  [12] = "err_frame_data_vc12", [13] = "err_frame_data_vc13",
		  [14] = "err_frame_data_vc14", [15] = "err_frame_data_vc15",
		  [16] = "err_frame_data_vc16", [17] = "err_frame_data_vc17",
		  [18] = "err_frame_data_vc18", [19] = "err_frame_data_vc19",
		  [20] = "err_frame_data_vc20", [21] = "err_frame_data_vc21",
		  [22] = "err_frame_data_vc22", [23] = "err_frame_data_vc23",
		  [24] = "err_frame_data_vc24", [25] = "err_frame_data_vc25",
		  [26] = "err_frame_data_vc26", [27] = "err_frame_data_vc27",
		  [28] = "err_frame_data_vc28", [29] = "err_frame_data_vc29",
		  [30] = "err_frame_data_vc30", [31] = "err_frame_data_vc31" }
#endif
	},
	{ 7, MIPI_HOST_1P5_INT_PLD_CRC_FATAL, REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL,
		REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL, REG_MIPI_HOST_INT_FORCE_PLD_CRC_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_crc_vc0", [1] = "err_crc_vc1",
		  [2] = "err_crc_vc2", [3] = "err_crc_vc3",
		  [4] = "err_crc_vc4", [5] = "err_crc_vc5",
		  [6] = "err_crc_vc6", [7] = "err_crc_vc7",
		  [8] = "err_crc_vc8", [9] = "err_crc_vc9",
		  [10] = "err_crc_vc10", [11] = "err_crc_vc11",
		  [12] = "err_crc_vc12", [13] = "err_crc_vc13",
		  [14] = "err_crc_vc14", [15] = "err_crc_vc15",
		  [16] = "err_crc_vc16", [17] = "err_crc_vc17",
		  [18] = "err_crc_vc18", [19] = "err_crc_vc19",
		  [20] = "err_crc_vc20", [21] = "err_crc_vc21",
		  [22] = "err_crc_vc22", [23] = "err_crc_vc23",
		  [24] = "err_crc_vc24", [25] = "err_crc_vc25",
		  [26] = "err_crc_vc26", [27] = "err_crc_vc27",
		  [28] = "err_crc_vc28", [29] = "err_crc_vc29",
		  [30] = "err_crc_vc30", [31] = "err_crc_vc31" }
#endif
	},
	{ 8, MIPI_HOST_1P5_INT_DATA_ID, REG_MIPI_HOST_INT_ST_DATA_ID,
		REG_MIPI_HOST_INT_MSK_DATA_ID, REG_MIPI_HOST_INT_FORCE_DATA_ID,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_id_vc0", [1] = "err_id_vc1",
		  [2] = "err_id_vc2", [3] = "err_id_vc3",
		  [4] = "err_id_vc4", [5] = "err_id_vc5",
		  [6] = "err_id_vc6", [7] = "err_id_vc7",
		  [8] = "err_id_vc8", [9] = "err_id_vc9",
		  [10] = "err_id_vc10", [11] = "err_id_vc11",
		  [12] = "err_id_vc12", [13] = "err_id_vc13",
		  [14] = "err_id_vc14", [15] = "err_id_vc15",
		  [16] = "err_id_vc16", [17] = "err_id_vc17",
		  [18] = "err_id_vc18", [19] = "err_id_vc19",
		  [20] = "err_id_vc20", [21] = "err_id_vc21",
		  [22] = "err_id_vc22", [23] = "err_id_vc23",
		  [24] = "err_id_vc24", [25] = "err_id_vc25",
		  [26] = "err_id_vc26", [27] = "err_id_vc27",
		  [28] = "err_id_vc28", [29] = "err_id_vc29",
		  [30] = "err_id_vc30", [31] = "err_id_vc31" }
#endif
	},
	{ 9, MIPI_HOST_1P5_INT_ECC_CORRECTED, REG_MIPI_HOST_INT_ST_ECC_CORRECT,
		REG_MIPI_HOST_INT_MSK_ECC_CORRECT, REG_MIPI_HOST_INT_FORCE_ECC_CORRECT,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_corrected0", [1] = "err_ecc_corrected1",
		  [2] = "err_ecc_corrected2", [3] = "err_ecc_corrected3",
		  [4] = "err_ecc_corrected4", [5] = "err_ecc_corrected5",
		  [6] = "err_ecc_corrected6", [7] = "err_ecc_corrected7",
		  [8] = "err_ecc_corrected8", [9] = "err_ecc_corrected9",
		  [10] = "err_ecc_corrected10", [11] = "err_ecc_corrected11",
		  [12] = "err_ecc_corrected12", [13] = "err_ecc_corrected13",
		  [14] = "err_ecc_corrected14", [15] = "err_ecc_corrected15",
		  [16] = "err_ecc_corrected16", [17] = "err_ecc_corrected17",
		  [18] = "err_ecc_corrected18", [19] = "err_ecc_corrected19",
		  [20] = "err_ecc_corrected20", [21] = "err_ecc_corrected21",
		  [22] = "err_ecc_corrected22", [23] = "err_ecc_corrected23",
		  [24] = "err_ecc_corrected24", [25] = "err_ecc_corrected25",
		  [26] = "err_ecc_corrected26", [27] = "err_ecc_corrected27",
		  [28] = "err_ecc_corrected28", [29] = "err_ecc_corrected29",
		  [30] = "err_ecc_corrected30", [31] = "err_ecc_corrected31" }
#endif
	},
	{ 10, MIPI_HOST_1P5_INT_PHY, REG_MIPI_HOST_INT_ST_PHY,
		REG_MIPI_HOST_INT_MSK_PHY, REG_MIPI_HOST_INT_FORCE_PHY,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
		  [2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
		  [4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
		  [6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
		  [8 ... 15] = NULL,
		  [16] = "phy_erresc_0", [17] = "phy_erresc_1",
		  [18] = "phy_erresc_2", [19] = "phy_erresc_3",
		  [20] = "phy_erresc_4", [21] = "phy_erresc_5",
		  [22] = "phy_erresc_6", [23] = "phy_erresc_7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 12, MIPI_HOST_1P5_INT_LINE, REG_MIPI_HOST_INT_ST_LINE,
		REG_MIPI_HOST_INT_MSK_LINE, REG_MIPI_HOST_INT_FORCE_LINE,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
		  [2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
		  [4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
		  [6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
		  [8 ... 15] = NULL,
		  [16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
		  [18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
		  [20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
		  [22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 13, MIPI_HOST_1P5_INT_IPI, REG_MIPI_HOST_INT_ST_IPI,
		REG_MIPI_HOST_INT_MSK_IPI, REG_MIPI_HOST_INT_FORCE_IPI,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 14, MIPI_HOST_1P5_INT_IPI2, REG_MIPI_HOST_INT_ST_IPI2,
		REG_MIPI_HOST_INT_MSK_IPI2, REG_MIPI_HOST_INT_FORCE_IPI2,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 15, MIPI_HOST_1P5_INT_IPI3, REG_MIPI_HOST_INT_ST_IPI3,
		REG_MIPI_HOST_INT_MSK_IPI3, REG_MIPI_HOST_INT_FORCE_IPI3,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 16, MIPI_HOST_1P5_INT_IPI4, REG_MIPI_HOST_INT_ST_IPI4,
		REG_MIPI_HOST_INT_MSK_IPI4, REG_MIPI_HOST_INT_FORCE_IPI4,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 17, MIPI_HOST_1P5_INT_IPI5, REG_MIPI_HOST_INT_ST_IPI5,
		REG_MIPI_HOST_INT_MSK_IPI5, REG_MIPI_HOST_INT_FORCE_IPI5,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 18, MIPI_HOST_1P5_INT_IPI6, REG_MIPI_HOST_INT_ST_IPI6,
		REG_MIPI_HOST_INT_MSK_IPI6, REG_MIPI_HOST_INT_FORCE_IPI6,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 19, MIPI_HOST_1P5_INT_IPI7, REG_MIPI_HOST_INT_ST_IPI7,
		REG_MIPI_HOST_INT_MSK_IPI7, REG_MIPI_HOST_INT_FORCE_IPI7,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 20, MIPI_HOST_1P5_INT_IPI8, REG_MIPI_HOST_INT_ST_IPI8,
		REG_MIPI_HOST_INT_MSK_IPI8, REG_MIPI_HOST_INT_FORCE_IPI8,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	}
};
#define MIPI_HOST_IREG_NUM_1P5 (sizeof(mh_int_regs_1p5)/sizeof(mh_int_regs_1p5[0]))

static const struct mipi_host_ireg_s mh_int_regs_1p5ap[] = {
	{ 1, MIPI_HOST_1P5AP_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_FAP_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PHY_FATAL,
		0x0007010fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
		  [2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
		  [4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
		  [6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
		  [8] = "err_deskew",
		  [9 ... 15] = NULL,
		  [16] = "phy_rxinvalidcodehs_0", [17] = "phy_rxinvalidcodehs_1",
		  [18] = "phy_rxinvalidcodehs_2",
		  [19 ... 31] = NULL }
#endif
	},
	{ 2, MIPI_HOST_1P5AP_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_FAP_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PKT_FATAL,
		0x00000003U,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_double", [1] = "shorter_payload",
		  [2 ... 31] = NULL }
#endif
	},
	{ 4, MIPI_HOST_1P5AP_INT_BNDRY_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_BNDRY_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_BNDRY_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_BNDRY_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
		  [2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  [4] = "err_f_bndry_match_vc4", [5] = "err_f_bndry_match_vc5",
		  [6] = "err_f_bndry_match_vc6", [7] = "err_f_bndry_match_vc7",
		  [8] = "err_f_bndry_match_vc8", [9] = "err_f_bndry_match_vc9",
		  [10] = "err_f_bndry_match_vc10", [11] = "err_f_bndry_match_vc11",
		  [12] = "err_f_bndry_match_vc12", [13] = "err_f_bndry_match_vc13",
		  [14] = "err_f_bndry_match_vc14", [15] = "err_f_bndry_match_vc15",
		  [16] = "err_f_bndry_match_vc16", [17] = "err_f_bndry_match_vc17",
		  [18] = "err_f_bndry_match_vc18", [19] = "err_f_bndry_match_vc19",
		  [20] = "err_f_bndry_match_vc20", [21] = "err_f_bndry_match_vc21",
		  [22] = "err_f_bndry_match_vc22", [23] = "err_f_bndry_match_vc23",
		  [24] = "err_f_bndry_match_vc24", [25] = "err_f_bndry_match_vc25",
		  [26] = "err_f_bndry_match_vc26", [27] = "err_f_bndry_match_vc27",
		  [28] = "err_f_bndry_match_vc28", [29] = "err_f_bndry_match_vc29",
		  [30] = "err_f_bndry_match_vc30", [31] = "err_f_bndry_match_vc31" }
#endif
	},
	{ 5, MIPI_HOST_1P5AP_INT_SEQ_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_SEQ_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_SEQ_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_SEQ_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_f_seq_vc0", [1] = "err_f_seq_vc1",
		  [2] = "err_f_seq_vc2", [3] = "err_f_seq_vc3",
		  [4] = "err_f_seq_vc4", [5] = "err_f_seq_vc5",
		  [6] = "err_f_seq_vc6", [7] = "err_f_seq_vc7",
		  [8] = "err_f_seq_vc8", [9] = "err_f_seq_vc9",
		  [10] = "err_f_seq_vc10", [11] = "err_f_seq_vc11",
		  [12] = "err_f_seq_vc12", [13] = "err_f_seq_vc13",
		  [14] = "err_f_seq_vc14", [15] = "err_f_seq_vc15",
		  [16] = "err_f_seq_vc16", [17] = "err_f_seq_vc17",
		  [18] = "err_f_seq_vc18", [19] = "err_f_seq_vc19",
		  [20] = "err_f_seq_vc20", [21] = "err_f_seq_vc21",
		  [22] = "err_f_seq_vc22", [23] = "err_f_seq_vc23",
		  [24] = "err_f_seq_vc24", [25] = "err_f_seq_vc25",
		  [26] = "err_f_seq_vc26", [27] = "err_f_seq_vc27",
		  [28] = "err_f_seq_vc28", [29] = "err_f_seq_vc29",
		  [30] = "err_f_seq_vc30", [31] = "err_f_seq_vc31" }
#endif
	},
	{ 6, MIPI_HOST_1P5AP_INT_CRC_FRM_FATAL, REG_MIPI_HOST_INT_ST_FAP_CRC_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_CRC_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_CRC_FRAME_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_frame_data_vc0", [1] = "err_frame_data_vc1",
		  [2] = "err_frame_data_vc2", [3] = "err_frame_data_vc3",
		  [4] = "err_frame_data_vc4", [5] = "err_frame_data_vc5",
		  [6] = "err_frame_data_vc6", [7] = "err_frame_data_vc7",
		  [8] = "err_frame_data_vc8", [9] = "err_frame_data_vc9",
		  [10] = "err_frame_data_vc10", [11] = "err_frame_data_vc11",
		  [12] = "err_frame_data_vc12", [13] = "err_frame_data_vc13",
		  [14] = "err_frame_data_vc14", [15] = "err_frame_data_vc15",
		  [16] = "err_frame_data_vc16", [17] = "err_frame_data_vc17",
		  [18] = "err_frame_data_vc18", [19] = "err_frame_data_vc19",
		  [20] = "err_frame_data_vc20", [21] = "err_frame_data_vc21",
		  [22] = "err_frame_data_vc22", [23] = "err_frame_data_vc23",
		  [24] = "err_frame_data_vc24", [25] = "err_frame_data_vc25",
		  [26] = "err_frame_data_vc26", [27] = "err_frame_data_vc27",
		  [28] = "err_frame_data_vc28", [29] = "err_frame_data_vc29",
		  [30] = "err_frame_data_vc30", [31] = "err_frame_data_vc31" }
#endif
	},
	{ 7, MIPI_HOST_1P5AP_INT_PLD_CRC_FATAL, REG_MIPI_HOST_INT_ST_FAP_PLD_CRC_FATAL,
		REG_MIPI_HOST_INT_MSK_FAP_PLD_CRC_FATAL, REG_MIPI_HOST_INT_FORCE_FAP_PLD_CRC_FATAL,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_crc_vc0", [1] = "err_crc_vc1",
		  [2] = "err_crc_vc2", [3] = "err_crc_vc3",
		  [4] = "err_crc_vc4", [5] = "err_crc_vc5",
		  [6] = "err_crc_vc6", [7] = "err_crc_vc7",
		  [8] = "err_crc_vc8", [9] = "err_crc_vc9",
		  [10] = "err_crc_vc10", [11] = "err_crc_vc11",
		  [12] = "err_crc_vc12", [13] = "err_crc_vc13",
		  [14] = "err_crc_vc14", [15] = "err_crc_vc15",
		  [16] = "err_crc_vc16", [17] = "err_crc_vc17",
		  [18] = "err_crc_vc18", [19] = "err_crc_vc19",
		  [20] = "err_crc_vc20", [21] = "err_crc_vc21",
		  [22] = "err_crc_vc22", [23] = "err_crc_vc23",
		  [24] = "err_crc_vc24", [25] = "err_crc_vc25",
		  [26] = "err_crc_vc26", [27] = "err_crc_vc27",
		  [28] = "err_crc_vc28", [29] = "err_crc_vc29",
		  [30] = "err_crc_vc30", [31] = "err_crc_vc31" }
#endif
	},
	{ 8, MIPI_HOST_1P5AP_INT_DATA_ID, REG_MIPI_HOST_INT_ST_FAP_DATA_ID,
		REG_MIPI_HOST_INT_MSK_FAP_DATA_ID, REG_MIPI_HOST_INT_FORCE_FAP_DATA_ID,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_id_vc0", [1] = "err_id_vc1",
		  [2] = "err_id_vc2", [3] = "err_id_vc3",
		  [4] = "err_id_vc4", [5] = "err_id_vc5",
		  [6] = "err_id_vc6", [7] = "err_id_vc7",
		  [8] = "err_id_vc8", [9] = "err_id_vc9",
		  [10] = "err_id_vc10", [11] = "err_id_vc11",
		  [12] = "err_id_vc12", [13] = "err_id_vc13",
		  [14] = "err_id_vc14", [15] = "err_id_vc15",
		  [16] = "err_id_vc16", [17] = "err_id_vc17",
		  [18] = "err_id_vc18", [19] = "err_id_vc19",
		  [20] = "err_id_vc20", [21] = "err_id_vc21",
		  [22] = "err_id_vc22", [23] = "err_id_vc23",
		  [24] = "err_id_vc24", [25] = "err_id_vc25",
		  [26] = "err_id_vc26", [27] = "err_id_vc27",
		  [28] = "err_id_vc28", [29] = "err_id_vc29",
		  [30] = "err_id_vc30", [31] = "err_id_vc31" }
#endif
	},
	{ 9, MIPI_HOST_1P5AP_INT_ECC_CORRECTED, REG_MIPI_HOST_INT_ST_FAP_ECC_CORRECT,
		REG_MIPI_HOST_INT_MSK_FAP_ECC_CORRECT, REG_MIPI_HOST_INT_FORCE_FAP_ECC_CORRECT,
		0xffffffffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_ecc_corrected0", [1] = "err_ecc_corrected1",
		  [2] = "err_ecc_corrected2", [3] = "err_ecc_corrected3",
		  [4] = "err_ecc_corrected4", [5] = "err_ecc_corrected5",
		  [6] = "err_ecc_corrected6", [7] = "err_ecc_corrected7",
		  [8] = "err_ecc_corrected8", [9] = "err_ecc_corrected9",
		  [10] = "err_ecc_corrected10", [11] = "err_ecc_corrected11",
		  [12] = "err_ecc_corrected12", [13] = "err_ecc_corrected13",
		  [14] = "err_ecc_corrected14", [15] = "err_ecc_corrected15",
		  [16] = "err_ecc_corrected16", [17] = "err_ecc_corrected17",
		  [18] = "err_ecc_corrected18", [19] = "err_ecc_corrected19",
		  [20] = "err_ecc_corrected20", [21] = "err_ecc_corrected21",
		  [22] = "err_ecc_corrected22", [23] = "err_ecc_corrected23",
		  [24] = "err_ecc_corrected24", [25] = "err_ecc_corrected25",
		  [26] = "err_ecc_corrected26", [27] = "err_ecc_corrected27",
		  [28] = "err_ecc_corrected28", [29] = "err_ecc_corrected29",
		  [30] = "err_ecc_corrected30", [31] = "err_ecc_corrected31" }
#endif
	},
	{ 10, MIPI_HOST_1P5AP_INT_PHY, REG_MIPI_HOST_INT_ST_FAP_PHY,
		REG_MIPI_HOST_INT_MSK_FAP_PHY, REG_MIPI_HOST_INT_FORCE_FAP_PHY,
		0x000f000fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
		  [2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
		  [4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
		  [6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
		  [8 ... 15] = NULL,
		  [16] = "phy_erresc_0", [17] = "phy_erresc_1",
		  [18] = "phy_erresc_2", [19] = "phy_erresc_3",
		  [20] = "phy_erresc_4", [21] = "phy_erresc_5",
		  [22] = "phy_erresc_6", [23] = "phy_erresc_7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 12, MIPI_HOST_1P5AP_INT_LINE, REG_MIPI_HOST_INT_ST_FAP_LINE,
		REG_MIPI_HOST_INT_MSK_FAP_LINE, REG_MIPI_HOST_INT_FORCE_FAP_LINE,
		0x00ff00ffU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
		  [2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
		  [4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
		  [6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
		  [8 ... 15] = NULL,
		  [16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
		  [18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
		  [20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
		  [22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
		  [24 ... 31] = NULL }
#endif
	},
	{ 13, MIPI_HOST_1P5AP_INT_IPI, REG_MIPI_HOST_INT_ST_FAP_IPI,
		REG_MIPI_HOST_INT_MSK_FAP_IPI, REG_MIPI_HOST_INT_FORCE_FAP_IPI,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 14, MIPI_HOST_1P5AP_INT_IPI2, REG_MIPI_HOST_INT_ST_FAP_IPI2,
		REG_MIPI_HOST_INT_MSK_FAP_IPI2, REG_MIPI_HOST_INT_FORCE_FAP_IPI2,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 15, MIPI_HOST_1P5AP_INT_IPI3, REG_MIPI_HOST_INT_ST_FAP_IPI3,
		REG_MIPI_HOST_INT_MSK_FAP_IPI3, REG_MIPI_HOST_INT_FORCE_FAP_IPI3,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 16, MIPI_HOST_1P5AP_INT_IPI4, REG_MIPI_HOST_INT_ST_FAP_IPI4,
		REG_MIPI_HOST_INT_MSK_FAP_IPI4, REG_MIPI_HOST_INT_FORCE_FAP_IPI4,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 17, MIPI_HOST_1P5AP_INT_IPI5, REG_MIPI_HOST_INT_ST_FAP_IPI5,
		REG_MIPI_HOST_INT_MSK_FAP_IPI5, REG_MIPI_HOST_INT_FORCE_FAP_IPI5,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 18, MIPI_HOST_1P5AP_INT_IPI6, REG_MIPI_HOST_INT_ST_FAP_IPI6,
		REG_MIPI_HOST_INT_MSK_FAP_IPI6, REG_MIPI_HOST_INT_FORCE_FAP_IPI6,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 19, MIPI_HOST_1P5AP_INT_IPI7, REG_MIPI_HOST_INT_ST_FAP_IPI7,
		REG_MIPI_HOST_INT_MSK_FAP_IPI7, REG_MIPI_HOST_INT_FORCE_FAP_IPI7,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 20, MIPI_HOST_1P5AP_INT_IPI8, REG_MIPI_HOST_INT_ST_FAP_IPI8,
		REG_MIPI_HOST_INT_MSK_FAP_IPI8, REG_MIPI_HOST_INT_FORCE_FAP_IPI8,
		0x0000007fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
		  [2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
		  [4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
		  [6] = "int_pulse_delay_overflow",
		  [7 ... 31] = NULL }
#endif
	},
	{ 21, MIPI_HOST_1P5AP_INT_ST_AP_GENERIC, REG_MIPI_HOST_INT_ST_AP_GENERIC,
		REG_MIPI_HOST_INT_MSK_AP_GENERIC, REG_MIPI_HOST_INT_FORCE_AP_GENERIC,
#ifdef CONFIG_ARCH_ZYNQMP
		0x000fffccU,
#else
		0x000fffcfU,
#endif
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "apb_ap_err_parerr", [1] = "apb_ap_err_intparerr",
		  [2] = "reg_bank_ap_err_parerr", [3] = "eg_bank_ap_err_scycerr",
		  [4] = "de_skew_ap_err", [5] = "pipeline_delay_ap_err",
		  [6] = "descrambler_ap_err", [7] = "phy_adapter_ap_err",
		  [8] = "packet_analyzer_ap_err_parerr", [9] = "packet_analyzer_ap_err_rederr",
		  [10] = "prep_outs_ap_err_parerr", [11] = "prep_outs_ap_err_rederr",
		  [12] = "err_msgr_ap_err", [13] = "err_handler_ap_err",
		  [14] = "synchronizer_fpclk_ap_err", [15] = "synchronizer_rxbyteclkhs_ap_err",
		  [16] = "synchronizer_pixclk_ap_err", [17] = "synchronizer_pixclk_2if_ap_err",
		  [18] = "synchronizer_pixclk_3if_ap_err", [19] = "synchronizer_pixclk_4if_ap_err",
		  [20] = "synchronizer_pixclk_5if_ap_err", [21] = "synchronizer_pixclk_6if_ap_err",
		  [22] = "synchronizer_pixclk_7if_ap_err", [23] = "synchronizer_pixclk_8if_ap_err",
		  [24] = "cphy_ap_err_parerr", [25] = "cphy_ap_err_rederr",
		  [26 ... 31] = NULL }
#endif
	},
	{ 22, MIPI_HOST_1P5AP_INT_ST_AP_IPI, REG_MIPI_HOST_INT_ST_AP_IPI,
		REG_MIPI_HOST_INT_MSK_AP_IPI, REG_MIPI_HOST_INT_FORCE_AP_IPI,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 23, MIPI_HOST_1P5AP_INT_ST_AP_IPI2, REG_MIPI_HOST_INT_ST_AP_IPI2,
		REG_MIPI_HOST_INT_MSK_AP_IPI2, REG_MIPI_HOST_INT_FORCE_AP_IPI2,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 24, MIPI_HOST_1P5AP_INT_ST_AP_IPI3, REG_MIPI_HOST_INT_ST_AP_IPI3,
		REG_MIPI_HOST_INT_MSK_AP_IPI3, REG_MIPI_HOST_INT_FORCE_AP_IPI3,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 25, MIPI_HOST_1P5AP_INT_ST_AP_IPI4, REG_MIPI_HOST_INT_ST_AP_IPI4,
		REG_MIPI_HOST_INT_MSK_AP_IPI4, REG_MIPI_HOST_INT_FORCE_AP_IPI4,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 26, MIPI_HOST_1P5AP_INT_ST_AP_IPI5, REG_MIPI_HOST_INT_ST_AP_IPI5,
		REG_MIPI_HOST_INT_MSK_AP_IPI5, REG_MIPI_HOST_INT_FORCE_AP_IPI5,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 27, MIPI_HOST_1P5AP_INT_ST_AP_IPI6, REG_MIPI_HOST_INT_ST_AP_IPI6,
		REG_MIPI_HOST_INT_MSK_AP_IPI6, REG_MIPI_HOST_INT_FORCE_AP_IPI6,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 28, MIPI_HOST_1P5AP_INT_ST_AP_IPI7, REG_MIPI_HOST_INT_ST_AP_IPI7,
		REG_MIPI_HOST_INT_MSK_AP_IPI7, REG_MIPI_HOST_INT_FORCE_AP_IPI7,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 29, MIPI_HOST_1P5AP_INT_ST_AP_IPI8, REG_MIPI_HOST_INT_ST_AP_IPI8,
		REG_MIPI_HOST_INT_MSK_AP_IPI8, REG_MIPI_HOST_INT_FORCE_AP_IPI8,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "parity_tx_err", [1] = "parity_rx_err",
		  [2] = "ecc_single_err", [3] = "ecc_multiple_err",
		  [4] = "crc_err", [5] = "redundancy_err",
		  [6 ... 31] = NULL }
#endif
	},
	{ 30, MIPI_HOST_1P5AP_INT_ST_LOGGER_ERR, REG_MIPI_HOST_INT_ST_LOGGER_ERR,
		REG_MIPI_HOST_INT_MASK_LOGGER_ERR, REG_MIPI_HOST_INT_FORCE_LOGGER_ERR,
		0x0000003fU,
#if MIPI_HOST_INT_DBG_ERRSTR
		{ [0] = "adv_log_ap_overflow", [1] = "adv_log_ap_underflow",
		  [2 ... 31] = NULL }
#endif
	}
};
#define MIPI_HOST_IREG_NUM_1P5AP (sizeof(mh_int_regs_1p5ap)/sizeof(mh_int_regs_1p5ap[0]))

#define MIPI_HOST_IREG_ERR_OVERFLOW (0x22U)
#define MIPI_HOST_IREG_ERR_REDUNDANCY (0x20U)

typedef struct mipi_host_s mipi_host_t;
struct mipi_host_icnt_drop {
	uint32_t icnt;
	uint32_t ipi_default;
	uint32_t (*mipi_host_icnt2ipi)(mipi_host_t*, uint32_t);
};
static uint32_t mipi_host_drop_vc2ipi(mipi_host_t *host, uint32_t vc_mask);
static uint32_t mipi_host_drop_mvc2ipi(mipi_host_t *host, uint32_t vc_mask);
/* icnt see: g_mh_icnt_names */
static struct mipi_host_icnt_drop g_mh_icnt_drops_1p4[] = {
	{ 0U, 0x0U, NULL },
	{ 1U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 2U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 3U, 0x0U, NULL },
	{ 4U, 0x0U, mipi_host_drop_vc2ipi },
	{ 5U, 0x0U, mipi_host_drop_vc2ipi },
	{ 6U, 0x0U, mipi_host_drop_vc2ipi },
	{ 7U, 0x0U, mipi_host_drop_vc2ipi },
	{ 8U, 0x0U, mipi_host_drop_vc2ipi },
	{ 9U, 0x0U, NULL },
	{ 10U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 11U, 0x0U, NULL},
	{ 12U, 0x0U, mipi_host_drop_mvc2ipi },
	{ 13U, MIPI_HOST_DROP_IPI_1, NULL},
	{ 14U, MIPI_HOST_DROP_IPI_2, NULL},
	{ 15U, MIPI_HOST_DROP_IPI_3, NULL},
	{ 16U, MIPI_HOST_DROP_IPI_4, NULL},
	{ 17U, 0x0U, NULL },
	{ 18U, 0x0U, NULL },
	{ 19U, 0x0U, NULL },
	{ 20U, 0x0U, NULL },
	{ 21U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 22U, MIPI_HOST_DROP_IPI_1, NULL},
	{ 23U, MIPI_HOST_DROP_IPI_2, NULL},
	{ 24U, MIPI_HOST_DROP_IPI_3, NULL},
	{ 25U, MIPI_HOST_DROP_IPI_4, NULL},
	{ 26U, 0x0U, NULL },
	{ 27U, 0x0U, NULL },
	{ 28U, 0x0U, NULL },
	{ 29U, 0x0U, NULL },
	{ 30U, 0x0U, NULL },
};
static struct mipi_host_icnt_drop g_mh_icnt_drops_1p5[] = {
	{ 0U, 0x0U, NULL },
	{ 1U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 2U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 3U, 0x0U, NULL },
	{ 4U, 0x0U, mipi_host_drop_vc2ipi },
	{ 5U, 0x0U, mipi_host_drop_vc2ipi },
	{ 6U, 0x0U, mipi_host_drop_vc2ipi },
	{ 7U, 0x0U, mipi_host_drop_vc2ipi },
	{ 8U, 0x0U, mipi_host_drop_vc2ipi },
	{ 9U, 0x0U, NULL },
	{ 10U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 11U, 0x0U, NULL},
	{ 12U, 0x0U, mipi_host_drop_mvc2ipi },
	{ 13U, MIPI_HOST_DROP_IPI_1, NULL},
	{ 14U, MIPI_HOST_DROP_IPI_2, NULL},
	{ 15U, MIPI_HOST_DROP_IPI_3, NULL},
	{ 16U, MIPI_HOST_DROP_IPI_4, NULL},
	{ 17U, MIPI_HOST_DROP_IPI_5, NULL},
	{ 18U, MIPI_HOST_DROP_IPI_6, NULL},
	{ 19U, MIPI_HOST_DROP_IPI_7, NULL},
	{ 20U, MIPI_HOST_DROP_IPI_8, NULL},
	{ 21U, MIPI_HOST_DROP_IPI_ALL, NULL},
	{ 22U, MIPI_HOST_DROP_IPI_1, NULL},
	{ 23U, MIPI_HOST_DROP_IPI_2, NULL},
	{ 24U, MIPI_HOST_DROP_IPI_3, NULL},
	{ 25U, MIPI_HOST_DROP_IPI_4, NULL},
	{ 26U, MIPI_HOST_DROP_IPI_5, NULL},
	{ 27U, MIPI_HOST_DROP_IPI_6, NULL},
	{ 28U, MIPI_HOST_DROP_IPI_7, NULL},
	{ 29U, MIPI_HOST_DROP_IPI_8, NULL},
	{ 30U, MIPI_HOST_DROP_IPI_ALL, NULL},
};
#endif

/**
 * comb 1 group by 2 ports:
 * port group index lane ipi
 * 0    0     0     2/4  4
 * 1    1     0     2/4  4
 * 2    0     1     2/0  2
 * 3    1     1     2/0  2
 * ...
 */
static const struct mipi_host_port_hw_s mh_port_hw_comb2[] = {
	{ 0, 0, 2, 4, 4 },
	{ 0, 1, 2, 4, 4 },
	{ 1, 0, 2, 4, 4 },
	{ 1, 1, 2, 4, 4 },
};

#define MIPI_HOST_PORT_HW_COMB2_UNIT (sizeof(mh_port_hw_comb2)/sizeof(mh_port_hw_comb2[0]))
#define MIPI_HOST_PORT_HW_COMB2_UGRP (2)

/**
 * alone 1 group with 1 port:
 * port group index lane ipi
 * 0    0     0     4    4
 * ...
 */
static const struct mipi_host_port_hw_s mh_port_hw_alone[] = {
	{ 0, 0, 4, 4, 4 },
};
#define MIPI_HOST_PORT_HW_ALONE_UNIT (sizeof(mh_port_hw_alone)/sizeof(mh_port_hw_alone[0]))
#define MIPI_HOST_PORT_HW_ALONE_UGRP (1)

/**
 * port hw mode:
 * 0: comb2, as default.
 * 1: alone.
 */
static const struct mipi_host_port_hw_mode_s g_mh_hw_modes[] = {
	{ mh_port_hw_comb2, (uint16_t)MIPI_HOST_PORT_HW_COMB2_UNIT, (uint16_t)MIPI_HOST_PORT_HW_COMB2_UGRP },
	{ mh_port_hw_alone, (uint16_t)MIPI_HOST_PORT_HW_ALONE_UNIT, (uint16_t)MIPI_HOST_PORT_HW_ALONE_UGRP },
};
#define MIPI_HOST_PORT_HW_MODES_NUM (sizeof(g_mh_hw_modes)/sizeof(g_mh_hw_modes[0]))

/* global var */
static struct mipi_hdev_s *g_hdev[MIPI_HOST_MAX_NUM];

/* mipi host reg r/w */
static uint32_t mhost_getreg(const struct mipi_host_s *host, uint32_t offs)
{
	return mipi_getreg(host->iomem, offs);
}
static void mhost_putreg(struct mipi_host_s *host, uint32_t offs, uint32_t val)
{
	mipi_putreg(host->iomem, offs, val);
#ifdef CONFIG_HOBOT_FUSA_DIAG
	/* read & compare */
	(void)mipi_csi_stl_rcompare(&host->stl, offs, val);
#endif
}

/* get group of host port */
static int32_t mipi_host_port_group(const struct mipi_hdev_s *hdev)
{
	int32_t units, idx;

	if (hdev->hw_mode == NULL) {
		return (hdev->port);
	}

	units = hdev->port / (int32_t)(hdev->hw_mode->unum);
	idx = hdev->port % (int32_t)(hdev->hw_mode->unum);

	return ((units * (int32_t)hdev->hw_mode->ugrp) + (int32_t)hdev->hw_mode->unit[idx].group);
}

/* get index(in group) of host port */
static int32_t mipi_host_port_index(const struct mipi_hdev_s *hdev)
{
	int32_t idx;

	if (hdev->hw_mode == NULL) {
		return 0;
	}

	idx = hdev->port % (int32_t)(hdev->hw_mode->unum);

	return (int32_t)(hdev->hw_mode->unit[idx].index);
}

/* get num of another host port in the same group */
static int32_t mipi_host_port_other(const struct mipi_hdev_s *hdev)
{
	int32_t units, idx, i;

	if (hdev->hw_mode == NULL) {
		return (-1);
	}

	units = hdev->port / (int32_t)(hdev->hw_mode->unum);
	idx = hdev->port % (int32_t)(hdev->hw_mode->unum);
	for (i = 0; i < (int32_t)(hdev->hw_mode->unum); i++) {
		if ((i != idx) && (hdev->hw_mode->unit[idx].group == hdev->hw_mode->unit[i].group)) {
			return ((units * (int32_t)hdev->hw_mode->unum) + i);
		}
	}

	return (-1);
}

/* get lane num of host port in this mode */
static int32_t mipi_host_port_lane(const struct mipi_hdev_s *hdev, int32_t mode)
{
	int32_t idx;

	if (hdev->hw_mode == NULL) {
		return (int32_t)HOST_DPHY_LANE_MAX;
	}

	idx = hdev->port % (int32_t)(hdev->hw_mode->unum);

	if (mode != MIPIHOST_LANEMODE_ALONE) {
		return (int32_t)(hdev->hw_mode->unit[idx].lane_group);
	} else {
		return (int32_t)(hdev->hw_mode->unit[idx].lane_alone);
	}
}

/* get ipi num of host port */
static int32_t mipi_host_port_ipi(const struct mipi_hdev_s *hdev)
{
	int32_t idx;

	if (hdev->hw_mode == NULL) {
		return MIPI_HOST_HW_IPI_MAX;
	}

	idx = hdev->port % (int32_t)(hdev->hw_mode->unum);

	return (int32_t)(hdev->hw_mode->unit[idx].ipi);
}

/* error diag event report */
static void mipi_host_error_report(const struct mipi_hdev_s *hdev,
			uint32_t event_index, uint8_t sub_event_id,
			uint8_t data, uint16_t line_number)
{
#ifdef CONFIG_HOBOT_FUSA_DIAG
	const struct os_dev *dev;
	const struct mipi_host_param_s *param;
	const uint16_t mod_ids[] = { (uint16_t)ModuleDiag_mipihost0, (uint16_t)ModuleDiag_mipihost1, /* qacfix: conversion */
 		(uint16_t)ModuleDiag_mipihost2, (uint16_t)ModuleDiag_mipihost3 }; /* qacfix: conversion */

	if (hdev == NULL)
		return;

	param = &hdev->host.param;
	if (param->error_diag == 0U)
		return;

	dev = &hdev->osdev;
	if ((hdev->port < (int32_t)(sizeof(mod_ids)/sizeof(mod_ids[0]))) && /* qacfix: conversion */
		(event_index < (uint32_t)ESW_MipiHostErrEventMax)) { /* qacfix: conversion */
#ifdef MIPI_HOST_ERR_DIAG_EVENT
		struct mipi_host_err_event_desp *ped = &mipi_host_err_events[event_index];

		if (param->error_diag == 1U) {
			int32_t ret;
			uint8_t i = 0U;
			struct diag_event event = { 0 };

			event.module_id = mod_ids[hdev->port];
			event.event_id = ped->id;
			event.payload[i] = sub_event_id;
			i++;
			event.payload[i] = data;
			i++;
			event.payload[i] = (uint8_t)(line_number & LOW_8_BIT_MASK); /* qacfix: conversion */
			i++;
			event.payload[i] = (uint8_t)(line_number >> SHIFT_8); /* qacfix: conversion */
			i++;
			event.env_len = i;
			event.event_prio = ped->prio;
			event.event_sta = (uint8_t)DiagEventStaFail;
			event.fchm_err_code = FCHM_ERR_CODE_SW;

			ret = diagnose_send_event(&event);
			if (ret < 0)
				mipi_err(dev, "%d: event %u(%s) sub=%u data=0x%02x send %d\n",
						line_number, ped->id, ped->name, sub_event_id, data, ret);
			else
				mipi_dbg(param, dev, "%d: event %u(%s) sub=%u data=0x%02x send\n",
						line_number, ped->id, ped->name, sub_event_id, data);
		} else {
			mipi_dbg(param, dev, "%d: event %u(%s) sub=%u data=0x%02x info\n",
					line_number, ped->id, ped->name, sub_event_id, data);
		}
#else
		mipi_dbg(param, dev, "%d: event %u sub=%u data=0x%02x info\n",
			line_number, EventIdSwMipiHostErrEventBase + event_index,
			sub_event_id, data);
#endif
	} else {
		mipi_err(dev, "%d: event index %u overflow\n",
				line_number, event_index);
	}
#endif
	return;
}

/* drop functions */
static uint32_t mipi_host_drop_vc2ipi(mipi_host_t *host, uint32_t vc_mask)
{
	mipi_host_cfg_t *cfg = &host->cfg;
	uint32_t i, ipi_num;
	uint32_t ipi_mask = 0U;

	if (vc_mask == 0U)
		return 0U;

	ipi_num = (cfg->channel_num != 0U) ? (uint32_t)(cfg->channel_num) : 1U;
	for (i = 0U; i < ipi_num; i++) {
		if (((0x1UL << cfg->channel_sel[i]) & vc_mask) != 0U)
			ipi_mask |= (uint32_t)(0x1UL << i);
	}

	return ipi_mask;
}
static uint32_t mipi_host_drop_mvc2ipi(mipi_host_t *host, uint32_t vc_mask)
{
	uint32_t vch_mask = ((vc_mask) >> MIPI_HOST_DROP_MVC_OFFSET) & MIPI_HOST_DROP_MVC_MASK;
	uint32_t vcl_mask = (vc_mask) & MIPI_HOST_DROP_MVC_MASK;
	return mipi_host_drop_vc2ipi(host, vch_mask | vcl_mask);
}

static void mipi_host_ipi_overflow_handle(struct mipi_host_s *host, uint32_t ipi);
/**
 * brief mipi_host_drop_func: drop frame by irq status
 *
 * param [in] host: host struct
 * param [in] icnt: irq icnt
 * param [in] subirq: subirq status
 *
 * return void
 */
static void mipi_host_drop_func(struct mipi_host_s *host, uint32_t icnt, uint32_t subirq)
{
	struct mipi_hdev_s *hdev = (struct mipi_hdev_s *)container_of(host, struct mipi_hdev_s, host); /* PRQA S 2810,0497 */ /* container_of macro */
	struct mipi_host_param_s *param = &host->param;
	struct mipi_host_icnt_drop *drops = host->drops;
	struct mipi_host_icnt_drop *drop;
	struct os_dev *dev = &hdev->osdev;
	uint32_t ipi_mask;
	const char *err_op = "";
	int32_t need_ipireset = 0;

	if ((drops == NULL) || (icnt >= MIPI_HOST_ICNT_NUM)) {
		return;
	}
	if (((0x1UL << icnt) & param->drop_mask) != 0U) {
		return;
	}
	drop = &drops[icnt];
	if (drop->mipi_host_icnt2ipi != NULL) {
		ipi_mask = drop->mipi_host_icnt2ipi(host, subirq);
	} else {
		ipi_mask = drop->ipi_default;
	}
	if (ipi_mask == 0U) {
		mipi_dbg(param, dev, "drop %s:0x%x ipi invalid\n",
				g_mh_icnt_names[icnt], subirq);
		return;
	}

	if ((icnt >= MIPI_HOST_ICNT_APIPI) && (icnt <= MIPI_HOST_ICNT_APIPIE) &&
		((subirq & MIPI_HOST_IREG_ERR_REDUNDANCY) != 0U)) {
		err_op = " --> reset";
		need_ipireset = 1;
	}

#ifdef MIPI_HOST_J5_DROP
	mipi_dbg(param, dev, "drop %s:0x%x as 0x%x%s\n",
			g_mh_icnt_names[icnt], subirq, ipi_mask, err_op);
	vio_set_drop_info((uint32_t)hdev->port, ipi_mask);
#else
	if (hdev->cb.drop_cb != NULL) {
		/* drop callback */
		mipi_dbg(param, dev, "drop_cb %s:0x%x as 0x%x%s\n",
			g_mh_icnt_names[icnt], subirq, ipi_mask, err_op);
		hdev->cb.drop_cb(hdev->port, ipi_mask);
	} else {
		mipi_dbg(param, dev, "drop %s:0x%x as 0x%x do nothing%s\n",
			g_mh_icnt_names[icnt], subirq, ipi_mask, err_op);
	}
#endif

	if (need_ipireset != 0) {
		mipi_host_ipi_overflow_handle(&hdev->host, icnt - MIPI_HOST_ICNT_APIPI);
	}
}

#ifdef CONFIG_HOBOT_FUSA_DIAG
/**
 * brief mipi_host_stl_drop_func: stl cb to drop frame by irq status
 *
 * param [in] stl: stl struct
 * param [in] icnt: irq icnt
 * param [in] subirq: subirq status
 *
 * return void
 */
static int32_t mipi_host_stl_drop_func(struct mipi_csi_stl_priv *stl, uint32_t icnt, uint32_t subirq)
{
	struct mipi_host_s *host = (struct mipi_host_s *)container_of(stl, struct mipi_host_s, stl); /* PRQA S 2810,0497 */ /* container_of macro */
	struct mipi_host_param_s *param = &host->param;

	if (param->drop_func != MIPI_HOST_PARAM_DROP_IN_IRQ) {
		mipi_host_drop_func(host, icnt, subirq);
	}
	return 0;
}
#endif

/**
 * brief mipi_host_configure_lanemode: configure dphy for lane mode
 *
 * param [in] lane : mipi host lane
 *
 * return int32_t: 0/-1
 */
static int32_t mipi_host_configure_lanemode(struct mipi_hdev_s *hdev, int32_t lane)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	int32_t group, poth;
	int32_t i, ret, target_mode = -1;

	for (i = 0; i < MIPIHOST_LANEMODE_MAX; i++) {
		if (lane <= mipi_host_port_lane(hdev, i)) {
			target_mode = i;
			break;
		}
	}

	/* upd hw lane_mode */
	hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST,
							hdev->port);

	group = mipi_host_port_group(hdev);
	poth = mipi_host_port_other(hdev);
	if (target_mode < 0) {
		mipi_err(dev, "port%d not support %dlane\n",
				hdev->port, lane);
		mipi_host_error_report(hdev, ESW_MipiHostHwConfigErr, SUB_ID_0, (uint8_t)(lane), __LINE__);
		return -1;
	} else if (target_mode != hdev->lane_mode) {
		mipi_info(dev, "change group%d lane_mode to %d for %dlane\n",
				group, target_mode, lane);
		if (host->state != MIPI_STATE_DEFAULT) {
			mipi_err(dev, "port%d busy error\n", hdev->port);
			mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_0, (uint8_t)(host->state), __LINE__);
			return -1;
		}
		if ((poth >= 0) && (poth < MIPI_HOST_MAX_NUM) && (g_hdev[poth] != NULL) &&
			(g_hdev[poth]->host.state != MIPI_STATE_DEFAULT)) {
			mipi_err(dev, "port%d busy error\n", poth);
			mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_1, (uint8_t)(g_hdev[poth]->host.state), __LINE__);
			return -1;
		}
		ret = mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_HOST, hdev->port,
									 target_mode);
		if (ret != 0) {
			mipi_err(dev, "dphy lane_mode ctrl error %d\n", ret);
			mipi_host_error_report(hdev, ESW_MipiHostDphyOpErr, SUB_ID_0, (uint8_t)(target_mode), __LINE__);
			return -1;
		}

		/* sync lane_mode */
		hdev->lane_mode = target_mode;
		if ((poth >= 0) && (poth < MIPI_HOST_MAX_NUM) && (g_hdev[poth] != NULL)) {
			g_hdev[poth]->lane_mode = target_mode;
		}
	} else {
		/* current mode is target */
	}

	if (target_mode == MIPIHOST_LANEMODE_GROUP) {
		if ((poth >= 0) && (poth < MIPI_HOST_MAX_NUM) && (g_hdev[poth] != NULL) &&
			(g_hdev[poth]->host.state == MIPI_STATE_DEFAULT)) {
			/* match ex_hdev and mark it as ex */
			hdev->ex_hdev = (void *)g_hdev[poth];
			hdev->is_ex = 0;
			(g_hdev[poth])->is_ex = 1;
		}
	} else {
		hdev->ex_hdev = NULL;
		hdev->is_ex = 0;
	}

	return 0;
}

/**
 * brief mipi_host_configure_ipi : configure ipi mode of mipi host
 *
 * param [in] cfg : mipi host cfgler's setting
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_configure_ipi(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	void __iomem *iomem = host->iomem;
	int32_t ipi_max, vcid, ipi_num = 0;
	uint32_t datatype, datatype_real, datatype_ipi, ipi_mode_dft, ipi_mode, mem_flush, index = 0U;
	uint32_t adv_value;
	int32_t vcid_dft[] = {MIPI_VC0, MIPI_VC1, MIPI_VC2, MIPI_VC3};
	uint32_t *dt_sys = &param->ipi1_dt;
	uint32_t ipi_16bit = 0U, ipi_infshow = 0U;
	int32_t is_1p5 = MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5);

	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_4, 0U, __LINE__);
		return -1;
	}

	ipi_max = mipi_host_port_ipi(hdev);
	if (cfg->channel_num > (uint16_t)ipi_max) {
		mipi_err(dev, "channel_num %d error, max %d\n", cfg->channel_num, ipi_max);
		mipi_host_error_report(hdev, ESW_MipiHostHwConfigErr, SUB_ID_1, (uint8_t)cfg->channel_num, __LINE__);
		return -1;
	}

	ipi_mode_dft = MIPI_HOST_IPI_ENABLE;
	if ((param->cut_through & MIPI_HOST_CUT_THROUGH_EN) != 0U) {
		mipi_dbg(param, dev, "ipi_mode enable cut_through\n");
		ipi_mode_dft |= MIPI_HOST_CUT_THROUGH;
	}
	mem_flush = (param->mem_flush != 0U) ? MIPI_HOST_MEMFLUSN_ENABLE : 0U;

	/* ipi# config */
	do {
		vcid = (int32_t)cfg->channel_sel[index];
		if (vcid < MIPI_HOST_HW_VC_MAX) {
			adv_value = param->adv_value;
			datatype = (dt_sys[index] != 0U) ? dt_sys[index] : cfg->datatype;
			datatype_real = datatype & (uint32_t)MIPI_HOST_IPI_DT_MASK; /* qacfix: conversion */
			if ((datatype & MIPI_HOST_EMB_DATA) != 0U)
				adv_value |= MIPI_HOST_ADV_EN_EMB;
			if ((is_1p5 != 0) && (cfg->ppi_pg != 0U)) {
				/* no  sync event for camera mode if ppi_pg */
				adv_value &= MIPI_HOST_ADV_DT_OV_MASK;
				if (datatype_real != MIPI_CSI2_DT_RGB888) {
					mipi_info(dev, "ipi%u ppi_pg 0x%02x over to 0x%02x\n",
						index + 1U, MIPI_CSI2_DT_RGB888, datatype_real);
					if ((adv_value & MIPI_HOST_ADV_DT_OV_MASK) == 0U)
						adv_value |= ((datatype_real << MIPI_HOST_ADV_DT_OFFSET) | MIPI_HOST_ADV_DT_OV_ENABLE);
					datatype_real = MIPI_CSI2_DT_RGB888;
					datatype &= ~MIPI_HOST_IPI_DT_MASK;
					datatype |= datatype_real;
					ipi_infshow = 1U;
				}
			} else if ((is_1p5 == 0) && ((datatype_real == MIPI_CSI2_DT_RAW_20) ||
						(datatype_real == MIPI_CSI2_DT_RAW_24))) {
				if ((adv_value & MIPI_HOST_ADV_DT_OV_MASK) == 0U) {
					if (datatype_real == MIPI_CSI2_DT_RAW_20)
						adv_value |= MIPI_HOST_ADV_RAW10_OV;
					else
						adv_value |= MIPI_HOST_ADV_RAW12_OV;
				}
			}
			if ((adv_value & MIPI_HOST_ADV_DT_OV_MASK) != 0U)
				datatype_ipi = ((adv_value & MIPI_HOST_ADV_DT_OV_MASK) >> MIPI_HOST_ADV_DT_OFFSET);
			else
				datatype_ipi = datatype_real;
			if (param->ipi_16bit == 0U) {
#if !defined CONFIG_HOBOT_XJ3 && !defined CONFIG_HOBOT_IPS_X2
				/* j5/j6 ipi: rgb(expect ppipg)/raw(expect raw20/raw24) as 16bit, others as 48bit */
				if ((datatype_ipi & MIPI_CSI2_DT_RGB_OR_RAW) != 0U) {
					if ((datatype_ipi == MIPI_CSI2_DT_RAW_20) || (datatype_ipi == MIPI_CSI2_DT_RAW_20) ||
						((is_1p5 != 0) && (cfg->ppi_pg != 0U)))
						ipi_16bit = 0U;
					else
						ipi_16bit = 1U;
				} else {
					ipi_16bit = 0U;
				}
#endif
			} else {
				ipi_16bit = (param->ipi_16bit == 1U) ? 1U : 0U;
				ipi_infshow = 1U;
			}
			ipi_mode = ipi_mode_dft | ((ipi_16bit != 0U) ? MIPI_HOST_BITWIDTH_16 : MIPI_HOST_BITWIDTH_48);
			if ((dt_sys[index] != 0U) || (vcid != vcid_dft[index]) ||
				((datatype & MIPI_HOST_EMB_DATA) != 0U) || (ipi_infshow != 0U)) {
				mipi_info(dev, "ipi%u vc%d datatype 0x%02x %dbit%s\n", index + 1U,
						vcid, datatype_real,
						((ipi_mode & MIPI_HOST_BITWIDTH_16) != 0U) ? MIPI_HOST_IPI_16BIT: MIPI_HOST_IPI_48BIT,
						((datatype & MIPI_HOST_EMB_DATA) != 0U) ? " as embedded" : "");
			}

			/* reset ipi */
			mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, ~g_mh_ipireg[index][HOST_IPI_INFO_SOFTRSTN]);
			mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, MIPI_HOST_ALLE_SOFTRSTN);

			/* config ipi */
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_VC], (uint32_t)vcid);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_DT], datatype);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_MEM], mem_flush);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_MODE], ipi_mode);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_HSA], cfg->hsaTime);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_HBP], cfg->hbpTime);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_HSD], cfg->hsdTime);
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_ADV], adv_value);
			ipi_num ++;
		} else {
			mhost_putreg(host, g_mh_ipireg[index][HOST_IPI_INFO_MODE],
						 MIPI_HOST_IPI_DISABLE);
		}
		index++;
	} while (index < cfg->channel_num);

	mipi_info(dev, "config %d/%d ipi done\n", ipi_num, cfg->channel_num);
	return 0;
}

/**
 * brief mipi_host_get_bpp: get the bpp of datatype of config
 *
 * param [in] cfg : mipi host config's setting
 *
 * return uint32_t
 */
static uint32_t mipi_host_get_bpp(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg)
{
	struct mipi_host_param_s *param = &hdev->host.param;
	void __iomem *iomem = &hdev->host.iomem;
	uint32_t bits_per_pixel;

	bits_per_pixel = mipi_datatype2bpp(cfg->datatype);
	if (((cfg->datatype == MIPI_CSI2_DT_RAW_20) || (cfg->datatype == MIPI_CSI2_DT_RAW_24)) &&
		((MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5) == 0) || ((param->adv_value & MIPI_HOST_ADV_DT_OV_MASK) != 0U))) {
			bits_per_pixel = bits_per_pixel >> 1U;
	}

	return bits_per_pixel;
}

/**
 * brief mipi_host_get_pkt2pkt: get the pkt to pkt time
 *
 * param [in] cfg : mipi host config's setting
 * param [in] ppi_width: the ppi width config: 0-8bit, 1-16bit
 *
 * return uint32_t
 */
static uint32_t mipi_host_get_pkt2pkt(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg,  uint32_t ppi_width)
{
	/* line state: LS->BLK->PH->PD->PF->BLK->LE->BLK
	 * frame state: IDLE-> BLK ->FS->BLK->(lines)->FE
	 * short_pkt_bytes = 4, long_pkt_bytes=6
	 * pkt2pkt_time=pkt2pkt*lanebyteclkperiod=pkt2pkt/lanebyteclk
	 * ppi_width=16=bytes_per_hsclk*BITS_PER_BYTE
	 * line_time: 3*pkt2pkt_time+2*short_packet_over(LS/LE)+long_pkt_over+active_lines_time
	 * = height*(3*pkt2pkt_time+(2*4bytes+6bytes)*1000/lanebyteclk/2/lanes+width*24*1000/(lanebyteclk*ppi_width*lanes))
	 * frame_time=(height+3)*line_time+2*pkt2pkt_time+2*short_packets
	 * = (height+3)*(3*pkt2pkt_time+short_packet_time+active_lines_time)+1000/lanebyteclk+pkt2pkt*2000/lanebyteclk
	 * fps = 1000000000/frame_time
	 * pkt2pkt=((1000000000*lanebyteclk/fps-1750*height-1000*height*width*bpp/(ppi_width*lanes)
	 * -3000*width*bpp/(ppi_width*lanes)-6250)/(3*height+11)
	 */
	struct mipi_host_param_s *param = &hdev->host.param;
	struct os_dev *dev = &hdev->osdev;
	uint64_t ipiclk = (hdev->ipi_clock != 0UL) ? hdev->ipi_clock : MIPI_HOST_IPICLK_DEFAULT;
	uint32_t bpp = mipi_host_get_bpp(hdev, cfg);
	uint32_t bytes_per_hsclk = (ppi_width == MIPI_HOST_PPI_WIDTH_16BIT) ? 2U : 1U;
	uint32_t t_ppi_hs = HOST_US_TO_NS * HOST_BITS_PER_BYTE * cfg->lane / cfg->mipiclk;
	uint32_t lanebyteclk = cfg->mipiclk / cfg->lane / HOST_BITS_PER_BYTE;
	uint32_t pkt2pkt_min = (4U + 2U * MIPI_HOST_FSYNC_TYPE_DEFULT) * t_ppi_hs / bytes_per_hsclk
		+ (4U + MIPI_HOST_FSYNC_TYPE_DEFULT) * (1U * HOST_S_TO_NS / ipiclk) + 1U;
	uint32_t pkt2pkt = 0U;

	if (cfg->ppi_pg == 0U) {
		/*adjust pkt2pkt according to expected fps*/
		pkt2pkt = (uint32_t)((uint64_t)HOST_S_TO_NS * lanebyteclk / cfg->fps - 1750U * cfg->height
			- (uint64_t)1000U * cfg->height * cfg->width * bpp / (bytes_per_hsclk * HOST_BITS_PER_BYTE * cfg->lane)
			- (uint64_t)3000U * cfg->width * bpp / (bytes_per_hsclk * HOST_BITS_PER_BYTE * cfg->lane) - 6250U)
			/ (3U * cfg->height + 11U) / HOST_S_TO_US;
		pkt2pkt = (param->pkt2pkt_time != 0U) ? param->pkt2pkt_time : (pkt2pkt < pkt2pkt_min ? pkt2pkt_min : pkt2pkt);
	} else {
		pkt2pkt = (param->pkt2pkt_time != 0U) ? param->pkt2pkt_time : pkt2pkt_min;
	}

	mipi_dbg(param, dev, "ppi_width:%d, t_ppi_hs:%d, ipi_clk:%llu, pkt2pkt:%d%s\n",
			ppi_width, t_ppi_hs, ipiclk, pkt2pkt,
			(param->pkt2pkt_time != 0U) ? "(force)" : "");
	return pkt2pkt;

}

/**
 * brief mipi_host_configure_ppi_pg : configure ppi pg of mipi host
 *
 * param [in] cfg : mipi host config's setting
 * param [in] ppi_width: the ppi width config: 0-8bit, 1-16bit
 *
 * return void
 */
static void mipi_host_configure_ppi_pg(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg, uint32_t ppi_width)
{
	struct mipi_host_s *host = &hdev->host;
	const struct os_dev *dev = &hdev->osdev;
	uint32_t pattern, vc, datatype, bits_per_pixel, rgb_width;
	/*Pkt2PktTime > (4 + 2*CSI2_HOST_DFLT_F_SYNC_TYPE).T(PPI-HS)/BytesPerHsClk + (4 + CSI2_HOST_DFLT_F_SYNC_TYPE).T(IPI)*/
        uint32_t pkt2pkt_time = mipi_host_get_pkt2pkt(hdev, cfg, ppi_width);

	if (MIPI_HOST_PPIPGC_TYPE(cfg->ppi_pg) != 0U)
		datatype = MIPI_CSI2_DT_EMB_8;
	else
		datatype = cfg->datatype;
	vc = MIPI_HOST_PPIPGC_VC(cfg->ppi_pg);
	pattern = MIPI_HOST_PPIPGC_MODE(cfg->ppi_pg);

	/* ppi_pg only  support rgb888 */
	switch (datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = 8 * 3 / 2;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = 10 * 3 / 2;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = 16;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = 20;
		break;
	case MIPI_CSI2_DT_EMB_8:
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = 8;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = 10;
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = 12;
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = 14;
		break;
	case MIPI_CSI2_DT_RAW_16:
	case MIPI_CSI2_DT_RGB565:
		bits_per_pixel = 16;
		break;
	case MIPI_CSI2_DT_RGB666:
		bits_per_pixel = 18;
		break;
	case MIPI_CSI2_DT_RGB888:
		bits_per_pixel = 24;
		break;
	case MIPI_CSI2_DT_RAW_20:
		bits_per_pixel = 20;
		break;
	case MIPI_CSI2_DT_RAW_24:
		bits_per_pixel = 24;
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	rgb_width = ((cfg->width * bits_per_pixel) + 23) / 24;

	mipi_info(dev, "ppi_pg: %d(%d)x%d vc%d 0x%02x pkt2pkt %d %s\n",
		cfg->width,  rgb_width, cfg->height, vc, datatype, pkt2pkt_time,
		(pattern != 0U) ? "HORIZONTAL" : "VERTICAL");
	mhost_putreg(host, REG_MIPI_HOST_PPI_PG_PATTERN_VRES, cfg->height);
	mhost_putreg(host, REG_MIPI_HOST_PPI_PG_PATTERN_HRES, MIPI_HOST_PPIPG_HRES(pkt2pkt_time, rgb_width));
	mhost_putreg(host, REG_MIPI_HOST_PPI_PG_CONFIG, MIPI_HOST_PPIPG_CONFIG(pattern, datatype, vc));
	return;
}

/**
 * brief mipi_host_enable_ppi_pg: enable/disable ppi pg of mipi host
 *
 * param [in] enable: the ppi pg enable: 0-disable(stop), 1-enable(start)
 * param [in] cfg : mipi host config's setting
 *
 * return void
 */
static int32_t mipi_host_enable_ppi_pg(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg, int32_t enable)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	const struct os_dev *dev = &hdev->osdev;
	uint32_t enable_v = (enable != 0) ? MIPI_HOST_PPI_PG_ENABLE : MIPI_HOST_PPI_PG_DISABLE;
        uint32_t ncount = 0U;
        uint32_t status;
	int32_t ret = 0;

	mhost_putreg(host, REG_MIPI_HOST_PPI_PG_ENABLE, enable_v);
	mipi_info(dev, "ppi_pg: %s\n",  (enable != 0) ? "enable" : "disable");
	if (enable == 0) {
		/* check innactive if disable */
		do {
			status = mhost_getreg(host, REG_MIPI_HOST_PPI_PG_STATUS);
			if (status == 0U) {
				break;
			}
			osal_mdelay(1); /* PRQA S 2877,2880 */ /* osal_mdelay macro */
			ncount++;
		} while ((param->notimeout != 0U) || (ncount <= param->wait_ms));
		if (status != 0U) {
			mipi_err(dev, "ppi_pg: disable status 0x%x timeout\n", status);
			ret = -1;
		}
	}

        return ret;
}

/**
 * brief mipi_host_configure_check: configure check.
 *
 * param [in] cfg: mipi host configure
 *
 * return int32_t: 0/-1
 */
static int32_t mipi_host_configure_check(const struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	const struct os_dev *dev = &hdev->osdev;
	const struct mipi_host_param_s *param = &hdev->host.param;
	const uint16_t *min, *max;
	mipi_host_cfg_union_t cfgu;
	uint16_t *c;
	int32_t ret = 0;
	uint8_t i, data = 0U;

	if (cfg == NULL) {
		mipi_err(dev, "check cfg NULL error\n");
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_0, 0U, __LINE__);
		return -1;
	}

	if (param->cfg_nocheck != 0U) {
		mipi_dbg(param, dev, "cfg nocheck\n");
		return 0;
	}

	(void)memcpy((void *)&cfgu.cfg, (const void *)cfg, sizeof(mipi_host_cfg_t));
	c = cfgu.val;
	min = mipi_host_cfg_min.val;
	max = mipi_host_cfg_max.val;
	for (i = 0U; i < (uint32_t)MIPI_HOST_CFG_NUM; i++) { /* qacfix: conversion */
		if ((c[i] < min[i]) || (c[i] > max[i])) {
			mipi_err(dev, "check cfg %s=%u overrange [%u, %u] error\n",
					mipi_host_cfg_name[i], c[i], min[i], max[i]);
			data = i;
			ret = -1;
		}
	}

	if (ret != 0) {
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_1, data, __LINE__);
	} else {
		mipi_dbg(param, dev, "cfg check done\n");
	}

	return ret;
}

/**
 * brief mipi_host_configure_cmp: configure compare.
 *
 * param [in] scfg dcfg: mipi host configure
 *
 * return int32_t: 0/-1
 */
static int32_t mipi_host_configure_cmp(const mipi_host_cfg_t *scfg, const mipi_host_cfg_t *dcfg)
{
	uint32_t i;

	if ((scfg == NULL) || (dcfg == NULL)) {
		/* do not need report */
		return -1;
	}

	if ((scfg->lane != dcfg->lane) ||
		(scfg->datatype != dcfg->datatype) ||
		(scfg->fps != dcfg->fps) ||
		(scfg->mclk != dcfg->mclk) ||
		(scfg->mipiclk != dcfg->mipiclk) ||
		(scfg->width != dcfg->width) ||
		(scfg->height != dcfg->height) ||
		(scfg->linelenth != dcfg->linelenth) ||
		(scfg->framelenth != dcfg->framelenth) ||
		(scfg->settle != dcfg->settle) ||
		((dcfg->hsaTime != 0U) && (scfg->hsaTime != dcfg->hsaTime)) ||
		((dcfg->hbpTime != 0U) && (scfg->hbpTime != dcfg->hbpTime)) ||
		((dcfg->hsdTime != 0U) && (scfg->hsdTime != dcfg->hsdTime)) ||
		(scfg->channel_num != dcfg->channel_num)) {
		/* do not need report */
		return -1;
	}
	for (i = 0U; (i < scfg->channel_num) && (i < (uint32_t)MIPIHOST_CHANNEL_NUM); i++) {
		if (scfg->channel_sel[i] != dcfg->channel_sel[i]) {
			/* do not need report */
			return -1;
		}
	}

	return 0;
}

/**
 * brief mipi_host_ipi_reset_do: do reset ipi of mipi host
 *
 * param [in] ipi: ipi index.
 * param [in] enable: ipi status.
 *
 * return void: NA
 */
static void mipi_host_ipi_reset_do(struct mipi_hdev_s *hdev, int32_t ipi, uint32_t enable)
{
	struct mipi_host_s *host = &hdev->host;
	struct os_dev *dev = &hdev->osdev;
	uint32_t softrstn, rstn_bit, reg_mode, ipi_mode;

	switch (ipi) {
		case MIPI_IPI4:
			reg_mode = REG_MIPI_HOST_IPI4_MODE;
			rstn_bit = MIPI_HOST_IPI4_SOFTRSTN;
			break;
		case MIPI_IPI3:
			reg_mode = REG_MIPI_HOST_IPI3_MODE;
			rstn_bit = MIPI_HOST_IPI3_SOFTRSTN;
			break;
		case MIPI_IPI2:
			reg_mode = REG_MIPI_HOST_IPI2_MODE;
			rstn_bit = MIPI_HOST_IPI2_SOFTRSTN;
			break;
		case MIPI_IPI1:
		default:
			reg_mode = REG_MIPI_HOST_IPI_MODE;
			rstn_bit = MIPI_HOST_IPI1_SOFTRSTN;
			break;
	}
#ifdef CONFIG_HOBOT_FUSA_DIAG
	(void)mipi_csi_stl_invalid(&host->stl,
		MIPI_HOST_ST_INVALID_US(host->cfg.fps, MIPI_HOST_IPIRST_INV_DFT_FRAME));
#endif
	softrstn = mhost_getreg(host, REG_MIPI_HOST_IPI_SOFTRSTN);
	ipi_mode = mhost_getreg(host, reg_mode);
	if (enable != 0U) {
		mipi_info(dev, "%d:ipi%d reset: enable\n", ipi, ipi + 1);
		softrstn |= rstn_bit;
		ipi_mode |= MIPI_HOST_IPI_ENABLE;
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, softrstn);
		mhost_putreg(host, reg_mode, ipi_mode);
	} else {
		mipi_info(dev, "%d:ipi%d reset: disable\n", ipi, ipi + 1);
		softrstn &= ~rstn_bit;
		ipi_mode &= ~(MIPI_HOST_IPI_ENABLE);
		mhost_putreg(host, reg_mode, ipi_mode);
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, softrstn);
	}
}

/**
 * brief mipi_host_ipi_reset: reset ipi of mipi host
 *
 * param [in] ipi_reset: ipi reset struct.
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_reset(struct mipi_hdev_s *hdev, const mipi_host_ipi_reset_t *ipi_reset)
{
	struct mipi_host_s *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	struct mipi_host_param_s *param = &host->param;
	struct os_dev *dev = &hdev->osdev;
	void __iomem *iomem = host->iomem;
	int32_t i, ipi_max, done = 0;

	if (ipi_reset == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_2, 0U, __LINE__);
		return -1;
	}
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_5, 0U, __LINE__);
		return -1;
	}

	ipi_max = mipi_host_port_ipi(hdev);
	for (i = 0; i < MIPIHOST_CHANNEL_NUM; i++) {
		if ((ipi_reset->mask & ((uint32_t)0x1U << (uint32_t)i)) == 0U) {
			continue;
		}
		if (i >= ipi_max) {
			mipi_info(dev, "%d:ipi%d reset: not support drop\n", i, i + 1);
			continue;
		} else if (i >= (int32_t)cfg->channel_num) {
			mipi_dbg(param, dev, "%d:ipi%d reset: not inited warning\n", i, i + 1);
		} else {
			/* ipi index verified */
		}
		mipi_host_ipi_reset_do(hdev, i, ipi_reset->enable);
		done++;
	}
	if (done == 0) {
		mipi_err(dev, "ipi reset error: none\n");
		mipi_host_error_report(hdev, ESW_MipiHostIpiOpErr, SUB_ID_0, (uint8_t)ipi_reset->mask, __LINE__);
		return -1;
	}

	return 0;
}

/**
 * brief mipi_host_ipi_get_info: get ipi info of mipi host
 *
 * param [in/out] ipi_info: ipi info struct.
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_get_info(const struct mipi_hdev_s *hdev, mipi_host_ipi_info_t *ipi_info)
{
	const struct mipi_host_s *host = &hdev->host;
	const mipi_host_cfg_t *cfg = &host->cfg;
	const struct mipi_host_param_s *param = &host->param;
	const struct os_dev *dev = &hdev->osdev;
	void __iomem *iomem = host->iomem;
	int32_t index, ipi_max;
	uint32_t r_fatal, r_mode, r_vc, r_datatype, r_hsa, r_hbp, r_hsd, r_adv;

	if (ipi_info == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_3, 0U, __LINE__);
		return -1;
	}
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_6, 0U, __LINE__);
		return -1;
	}

	index = (int32_t)ipi_info->index;
	ipi_max = mipi_host_port_ipi(hdev);
	if (index >= ipi_max) {
		mipi_err(dev, "%d:ipi%d get: not suppor error\n", index, index + 1);
		mipi_host_error_report(hdev, ESW_MipiHostIpiOpErr, SUB_ID_1, (uint8_t)index, __LINE__);
		return -1;
	}
	if (index >= (int32_t)cfg->channel_num) {
		mipi_dbg(param, dev, "%d:ipi%d get: not inited warning\n", index, index + 1);
	}
	/* index verified */
	r_mode = g_mh_ipireg[index][HOST_IPI_INFO_MODE];
	r_vc = g_mh_ipireg[index][HOST_IPI_INFO_VC];
	r_datatype = g_mh_ipireg[index][HOST_IPI_INFO_DT];
	r_hsa = g_mh_ipireg[index][HOST_IPI_INFO_HSA];
	r_hbp = g_mh_ipireg[index][HOST_IPI_INFO_HBP];
	r_hsd = g_mh_ipireg[index][HOST_IPI_INFO_HSD];
	r_adv = g_mh_ipireg[index][HOST_IPI_INFO_ADV];
	r_fatal = g_mh_ipireg[index][HOST_IPI_INFO_FATAL];

	ipi_info->fatal = (uint16_t)mhost_getreg(host, r_fatal);
	ipi_info->mode = (uint16_t)mhost_getreg(host, r_mode);
	ipi_info->vc = (uint8_t)mhost_getreg(host, r_vc);
	ipi_info->datatype = (uint16_t)mhost_getreg(host, r_datatype);
	ipi_info->hsa = (uint16_t)mhost_getreg(host, r_hsa);
	ipi_info->hbp = (uint16_t)mhost_getreg(host, r_hbp);
	ipi_info->hsd = (uint16_t)mhost_getreg(host, r_hsd);
	ipi_info->adv = (uint32_t)mhost_getreg(host, r_adv);

	return 0;
}

static void mipi_host_ipi_set_info_ex(struct mipi_hdev_s *hdev, const mipi_host_ipi_info_t *ipi_info,
					int32_t index, uint32_t set_mask)
{
	struct mipi_host_s *host = &hdev->host;
	uint32_t r_vc, r_datatype, r_hsa, r_hbp, r_hsd, r_adv;

	r_vc = g_mh_ipireg[index][HOST_IPI_INFO_VC];
	r_datatype = g_mh_ipireg[index][HOST_IPI_INFO_DT];
	r_hsa = g_mh_ipireg[index][HOST_IPI_INFO_HSA];
	r_hbp = g_mh_ipireg[index][HOST_IPI_INFO_HBP];
	r_hsd = g_mh_ipireg[index][HOST_IPI_INFO_HSD];
	r_adv = g_mh_ipireg[index][HOST_IPI_INFO_ADV];

	if ((set_mask & (0x1U << HOST_IPI_INFO_VC)) != 0U) {
		mhost_putreg(host, r_vc, ipi_info->vc);
	}
	if ((set_mask & (0x1U << HOST_IPI_INFO_DT)) != 0U) {
		mhost_putreg(host, r_datatype, ipi_info->datatype);
	}
	if ((set_mask & (0x1U << HOST_IPI_INFO_HSA)) != 0U) {
		mhost_putreg(host, r_hsa, ipi_info->hsa);
	}
	if ((set_mask & (0x1U << HOST_IPI_INFO_HBP)) != 0U) {
		mhost_putreg(host, r_hbp, ipi_info->hbp);
	}
	if ((set_mask & (0x1U << HOST_IPI_INFO_HSD)) != 0U) {
		mhost_putreg(host, r_hsd, ipi_info->hsd);
	}
	if ((set_mask & (0x1U << HOST_IPI_INFO_ADV)) != 0U) {
		mhost_putreg(host, r_adv, ipi_info->adv);
	}
}

/**
 * brief mipi_host_ipi_set_info: set ipi info of mipi host
 *
 * param [in] ipi_info: ipi info struct.
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_set_info(struct mipi_hdev_s *hdev, const mipi_host_ipi_info_t *ipi_info)
{
	struct mipi_host_s *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	struct mipi_host_param_s *param = &host->param;
	struct os_dev *dev = &hdev->osdev;
	void __iomem *iomem = host->iomem;
	int32_t index, ipi_max;
	uint32_t set_mask;
	uint32_t r_mode, r_vc, r_datatype, r_hsa, r_hbp, r_hsd, r_adv;

	if (ipi_info == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_4, 0U, __LINE__);
		return -1;
	}
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_7, 0U, __LINE__);
		return -1;
	}

	index = (int32_t)ipi_info->index;
	ipi_max = mipi_host_port_ipi(hdev);
	if (index >= ipi_max) {
		mipi_err(dev, "%d:ipi%d set: not suppor error\n", index, index + 1);
		mipi_host_error_report(hdev, ESW_MipiHostIpiOpErr, SUB_ID_2, (uint8_t)index, __LINE__);
		return -1;
	}
	if (index >= (int32_t)cfg->channel_num) {
		mipi_dbg(param, dev, "%d:ipi%d set: not inited warning\n", index, index + 1);
	}
	/* index verified */
	r_mode = g_mh_ipireg[index][HOST_IPI_INFO_MODE];
	r_vc = g_mh_ipireg[index][HOST_IPI_INFO_VC];
	r_datatype = g_mh_ipireg[index][HOST_IPI_INFO_DT];
	r_hsa = g_mh_ipireg[index][HOST_IPI_INFO_HSA];
	r_hbp = g_mh_ipireg[index][HOST_IPI_INFO_HBP];
	r_hsd = g_mh_ipireg[index][HOST_IPI_INFO_HSD];
	r_adv = g_mh_ipireg[index][HOST_IPI_INFO_ADV];

	/* set some masked if fatal.b15, or set all
	 * ps: disable first, enable last */
	set_mask = ((ipi_info->fatal & HOST_IPI_INFO_B_SETV) != 0U) ? ipi_info->fatal : HOST_IPI_INFO_B_ALL;
	if (((set_mask & (0x1U << HOST_IPI_INFO_MODE)) != 0U) &&
		((ipi_info->mode & MIPI_HOST_IPI_ENABLE) == 0U)) {
		mhost_putreg(host, r_mode, ipi_info->mode);
	}
	mipi_host_ipi_set_info_ex(hdev, ipi_info, index, set_mask);
	if (((set_mask & (0x1U << HOST_IPI_INFO_MODE)) != 0U) &&
		((ipi_info->mode & MIPI_HOST_IPI_ENABLE) != 0U)) {
		mhost_putreg(host, r_mode, ipi_info->mode);
	}

	mipi_info(dev, "%d:ipi%d set: mode=0x%x,vc=0x%x,dt=0x%x,hsa=%d,hbp=%d,hsd=%d,adv=0x%x\n",
		index, index + 1,
		mhost_getreg(host, (uint32_t)r_mode),
		mhost_getreg(host, (uint32_t)r_vc),
		mhost_getreg(host, (uint32_t)r_datatype),
		mhost_getreg(host, (uint32_t)r_hsa),
		mhost_getreg(host, (uint32_t)r_hbp),
		mhost_getreg(host, (uint32_t)r_hsd),
		mhost_getreg(host, (uint32_t)r_adv));
	return 0;
}


/**
 * brief mipi_host_configure_clk: configure clk of mipi host
 *
 * param [in] name/freq/checkequ: mipi host clk's setting
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_configure_clk(const struct mipi_hdev_s *hdev, const char *name,
				uint64_t freq, int32_t checkequ)
{
	int32_t ret;
	const struct os_dev *dev = &hdev->osdev;
#if defined CONFIG_HOBOT_XJ3 || defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5 || defined X5_CHIP
	uint64_t clk;

	if (name == NULL) {
		/* do not need report */
		return -1;
	}
	if (strcmp(name, MIPI_HOST_CLOCK_NONE) == 0) {
		return 0;
	}

#if defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
	/* clk_24m_out config by dphy for j5 */
	if (name == hdev->host.socclk.snrclk) {
		ret = mipi_dphy_outclk_config(name, freq, &clk);
		if (checkequ != 0) {
			if (clk != freq) {
				mipi_err(dev, "%s = %llu != %llu\n", name, clk, freq);
				/* do not need report */
				return (-1);
			}
			mipi_info(dev, "%s = %llu\n", name, clk);
		}
		return ret;
	}
#endif

	if (freq == 0U) {
		return vio_clk_disable(name);
	}

	clk = vio_get_clk_rate(name);
	if (clk != freq) {
		(void)vio_set_clk_rate(name, freq);
		if (checkequ != 0) {
			clk = vio_get_clk_rate(name);
			if (clk != freq) {
				mipi_err(dev, "%s = %llu != %llu\n", name, clk, freq);
				/* do not need report */
				return (-1);
			}
			mipi_info(dev, "%s = %llu\n", name, clk);
		}
	}
	ret = vio_clk_enable(name);
#else
	mipi_info(dev, "should: %s %s %llu\n", __func__, name, freq);
	ret = 0;
#endif
	return ret;
}

/**
 * brief mipi_host_get_clk: configure clk of mipi host
 *
 * param [in] name: mipi host clk's getting
 *
 * return int32_t : 0/-1
 */
static uint64_t mipi_host_get_clk(const struct mipi_hdev_s *hdev, const char *name)
{
	uint64_t clk;
	const struct os_dev *dev = &hdev->osdev;

	if (name == NULL) {
		return 0U;
	}

#if defined CONFIG_HOBOT_XJ3 || defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5 || defined X5_CHIP
	if (strcmp(name, MIPI_HOST_CLOCK_NONE) == 0) {
		return 0U;
	}

#if defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
	/* clk_24m_out config by dphy for j5 */
	if (name == hdev->host.socclk.snrclk) {
		if (mipi_dphy_outclk_config(name, 0UL, &clk) != 0)
			clk = 0;
	} else {
		clk = vio_get_clk_rate(name);
	}
#else
	clk = vio_get_clk_rate(name);
#endif
	{
		const struct mipi_host_param_s *param = &hdev->host.param;
		mipi_dbg(param, dev, "%s: %s = %llu\n", __func__, name, clk);
	}
#else
	clk = 0U;
	mipi_info(dev, "should: %s %s %llu\n", __func__, name, clk);
#endif
	return clk;
}

/**
 * brief mipi_host_snrclk_set_freq: configure snrclk of mipi host
 *
 * param [in] freq: mipi host snrclk's setting
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_snrclk_set_freq(struct mipi_hdev_s *hdev, uint32_t freq)
{
	int32_t ret;
	struct os_dev *dev = &hdev->osdev;
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	struct mipi_host_snrclk_s *snrclk = &hdev->host.snrclk;
	struct mipi_host_param_s *param = &hdev->host.param;
	const char *name = hdev->host.socclk.snrclk;
	uint32_t diff;

	if ((snrclk->index < 0) || (name == NULL)) {
		mipi_info(dev, "snrclk set freq not support\n");
		mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_0, 0U, __LINE__);
		return -1;
	}
	ret = mipi_host_configure_clk(hdev, name, freq, 0);
	if (ret != 0) {
		mipi_err(dev, "%s set %u error\n", name, freq);
		mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_1, (uint8_t)snrclk->index, __LINE__);
		return ret;
	}
	if (freq != 0U) {
		param->snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev, name));
		diff = (freq > param->snrclk_freq) ? (freq - param->snrclk_freq) :
			(param->snrclk_freq - freq);
		if ((diff * MIPI_HOST_SNRCLK_DIFF_PERC) > freq) {
			mipi_err(dev, "%s set %u but %u error\n", name, freq, param->snrclk_freq);
			mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_2, (uint8_t)snrclk->index, __LINE__);
			return -1;
		}
	}

	mipi_info(dev, "%s set %u as %u\n", name, freq, param->snrclk_freq);
#else
	mipi_err(dev, "snrclk not support\n");
	mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_0, 0U, __LINE__);
	ret = -1;
#endif
	return ret;
}

/**
 * brief mipi_host_snrclk_set_en: enable/disable snrclk of mipi host
 *
 * param [in] enable: mipi host snrclk's setting
 *
 * return int32_t : 0/-1
 */

static int32_t mipi_host_snrclk_set_en(struct mipi_hdev_s *hdev, int32_t enable)
{
	int32_t ret;
	struct os_dev *dev = &hdev->osdev;
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	struct mipi_host_snrclk_s *snrclk = &hdev->host.snrclk;
	struct mipi_host_param_s *param = &hdev->host.param;

	if (snrclk->pinctrl == NULL) {
		mipi_info(dev, "snrclk set en not support\n");
		mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_3, (uint8_t)snrclk->index, __LINE__);
		return -1;
	}
#if 0
	/* enable clk as default at first time */
	if (param->snrclk_en == MIPI_HOST_SNRCLK_NOUSED) {
		mipi_info(dev, "snrclk clk default as %lu\n",
			mipi_host_get_clk(hdev, hdev->host.socclk.snrclk));
		(void)mipi_host_configure_clk(hdev, hdev->host.socclk.snrclk,
			mipi_host_get_clk(hdev, hdev->host.socclk.snrclk), 0);
	}
#endif
	if (enable != 0) {
		if (snrclk->enable != NULL) {
			mipi_info(dev, "snrclk set enable\n");
			ret = 0;
			ret = pinctrl_select_state(snrclk->pinctrl, snrclk->enable);
			if (ret == 0) {
				param->snrclk_en = MIPI_HOST_SNRCLK_ENABLE;
			}
		} else {
			mipi_err(dev, "snrclk set enable not support\n");
			mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_4, (uint8_t)snrclk->index, __LINE__);
			ret = -1;
		}
	} else {
		if (snrclk->disable != NULL) {
			mipi_info(dev, "snrclk set disable\n");
			ret = 0;
			ret = pinctrl_select_state(snrclk->pinctrl, snrclk->disable);
			if (ret == 0) {
				param->snrclk_en = MIPI_HOST_SNRCLK_DISABLE;
			}
		} else {
			mipi_err(dev, "snrclk set disable not support\n");
			mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_5, (uint8_t)snrclk->index, __LINE__);
			ret = -1;
		}
	}
#else
	mipi_err(dev, "snrclk not support\n");
	mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_3, 0U, __LINE__);
	ret = -1;
#endif

	return ret;
}

/* mipi host functions */
static uint64_t mipi_host_pixel_pixclk_cal(const struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	const struct os_dev *dev = &hdev->osdev;
	const struct mipi_host_s *host = &hdev->host;
	const struct mipi_host_param_s *param = &host->param;
	uint64_t pixclk;
	uint64_t linelenth = (cfg->linelenth == 0U) ? cfg->width : cfg->linelenth;
	uint64_t framelenth = (cfg->framelenth == 0U) ? cfg->height : cfg->framelenth;

	if (cfg->fps == 0U) {
		mipi_info(dev, "input FPS can't be zero\n");
		return 0;
	}

	if (param->ipi_force >= MIPI_HOST_IPIFORCE_MIN) {
		pixclk = param->ipi_force;
		mipi_info(dev, "ipiclk force as %llu\n", pixclk);
	} else {
		pixclk = linelenth * framelenth * cfg->fps;
		if (cfg->datatype < (uint16_t)MIPI_CSI2_DT_RAW_8) {
			pixclk = pixclk;
		} else {
			pixclk = (pixclk + MIPI_HOST_PIXELS_RAW - 1U) / MIPI_HOST_PIXELS_RAW;
		}

		if ((param->ipi_limit != 0U) && (pixclk < param->ipi_limit)) {
			mipi_info(dev, "ipiclk limit %llu up to %u\n", pixclk, param->ipi_limit);
			pixclk = param->ipi_limit;
		}
	}

	return pixclk;
}

static uint64_t mipi_host_pixel_clk_select(const struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	const struct os_dev *dev = &hdev->osdev;
	uint64_t pixclk;
	uint64_t pixclk_act;
	int32_t i;

	pixclk = mipi_host_pixel_pixclk_cal(hdev, cfg);
#ifdef CONFIG_HOBOT_IPS_X2
	if (ips_set_mipi_ipi_clk(pixclk) < 0) {
		mipi_info(dev, "ips_set_mipi_ipi_clk %llu error\n", pixclk);
		pixclk_act = 0U;
	} else {
		mipi_info(dev, "host fifo clk pixclk: %llu\n", pixclk);
		pixclk_act = ips_get_mipi_ipi_clk();
		mipi_info(dev, "host fifo clk pixclk: %llu\n", pixclk_act);
	}
#else
	if (hdev->port >= (int32_t)MIPI_HOST_IPICLK_NUM) {
		pixclk_act = 0U;
	} else {
		i = 0;
		do {
			if (mipi_host_configure_clk(hdev, hdev->host.socclk.ipiclk[i], pixclk, 0) != 0) {
				break;
			}
			i ++;
		} while (i < (int32_t)cfg->channel_num); /* qacfix: conversion */

		pixclk_act = mipi_host_get_clk(hdev, hdev->host.socclk.ipiclk[0]);
		if (pixclk_act == 0U) {
			pixclk_act = pixclk;
		}
	}

	if (pixclk_act == 0U) {
		mipi_err(dev, "mipi_host_configure_clk %llu error\n", pixclk);
	} else {
		mipi_info(dev, "ipiclk set %llu get %llu\n", pixclk, pixclk_act);
	}
#endif
	return pixclk_act;
}

/**
 * brief mipi_host_get_hsd_legacy :
 *
 * param [] cfg :
 *
 * return uint16_t
 */
static uint16_t mipi_host_get_hsd_legacy(const struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg,
		uint64_t pixclk)
{
	/**
	 * Rxbyteclk = (Rxbitclk / Number_of_lanes) / 8
	 * Rxbyteclk_per = 1 / Rxbyteclk
	 * Bytes_to_transmi = BitsPerPixel * Line_size / 8
	 * time to transmit last pixel in PPI
	 * = (Bytes_to_transmit / Number_of_lanes) *  Rxbyteclk_per
	 * = ((BitsPerPixel * Line_size / 8) / Number_of_lanes) /((Rxbitclk /  Number_of_lanes) / 8)
	 * = (BitsPerPixel * Line_size) / Rxbitclk
	 *
	 * pixel_clk_per = 1 / pixel_clk
	 * time to transmit last pixel in IPI
	 * = (hsa + hbp + hsd + cycles_to_transmit_data) * pixel_clk_per
	 * = (hsa + hbp + hsd + cycles_to_transmit_data) / pixel_clk
	 *
	 * T(ppi) < T(ipi) ==>
	 * BitsPerPixel * Line_size * pixel_clk / Rxbitclk < (hsa + hbp + hsd + cycles_to_trans) ==>
	 *  hsd > BitsPerPixel * Line_size * pixel_clk / Rxbitclk - (cycles_to_trans+hsa+ hbp)
	 *  cycles_to_trans = Line_size in 48 bits IPI
	 *
	 */
	const struct os_dev *dev = &hdev->osdev;
	const struct mipi_host_param_s *param = &hdev->host.param;
	uint64_t rx_bit_clk;
	uint64_t bits_per_pixel;
	uint64_t line_size;
	uint64_t cycles_to_trans;
	uint64_t time_ppi;
	uint64_t time_ipi;
	uint32_t  hsdtime;
	uint32_t yuv_cycle = (param->ipi_16bit != 0U) ? (uint32_t)MIPI_IPI16_YUV_CYCLE : (uint32_t)MIPI_IPI48_YUV_CYCLE;
	uint32_t raw_pixel = (param->ipi_16bit != 0U) ? (uint32_t)MIPI_IPI16_RAW_PIXEL : (uint32_t)MIPI_IPI48_RAW_PIXEL;

	switch (cfg->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_12;
		cycles_to_trans = (uint64_t)cfg->width * yuv_cycle;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_24;
		cycles_to_trans = (uint64_t)cfg->width * yuv_cycle;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_16;
		cycles_to_trans = (uint64_t)cfg->width * yuv_cycle;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_32;
		cycles_to_trans = (uint64_t)cfg->width * yuv_cycle;
		break;
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_8;
		cycles_to_trans = ((uint64_t)cfg->width + raw_pixel - 1U) / raw_pixel;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_10;
		cycles_to_trans = ((uint64_t)cfg->width + raw_pixel - 1U) / raw_pixel;
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = MIPI_CSI2_DT_BITS_12;
		cycles_to_trans = ((uint64_t)cfg->width + raw_pixel - 1U) / raw_pixel;
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = MIPI_CSI2_DT_BITS_14;
		cycles_to_trans = ((uint64_t)cfg->width + raw_pixel - 1U) / raw_pixel;
		break;
	default:
		bits_per_pixel = MIPI_CSI2_DT_BITS_16;
		cycles_to_trans = 0U;
		break;
	}
	if (cfg->linelenth == 0U) {
		rx_bit_clk = (uint64_t)cfg->mipiclk * MIPI_HOST_FREQ_MHZ;
		line_size = cfg->width;
	} else {
		rx_bit_clk = (uint64_t)cfg->linelenth * cfg->framelenth * cfg->fps * bits_per_pixel;
		line_size = cfg->linelenth;
	}
	mipi_info(dev, "linelenth: %u, framelenth: %u, fps: %u, bits_per_pixel: %llu, pixclk: %llu, rx_bit_clk: %llu\n",
			 cfg->linelenth, cfg->framelenth, cfg->fps, bits_per_pixel, pixclk, rx_bit_clk);
	time_ppi = (MIPI_HOST_FREQ_MHZ_TO_NS * bits_per_pixel * line_size * MIPI_HOST_FREQ_MHZ / rx_bit_clk);
	mipi_info(dev, "time to transmit last pixel in ppi: %llu\n", time_ppi);
	if ((bits_per_pixel * line_size * pixclk / rx_bit_clk) > ((uint64_t)cfg->hsaTime + (uint64_t)cfg->hbpTime + cycles_to_trans)) {
		hsdtime = (uint32_t)((bits_per_pixel * line_size * pixclk / rx_bit_clk) - ((uint64_t)cfg->hsaTime + (uint64_t)cfg->hbpTime + cycles_to_trans));
	} else {
		hsdtime = 0U;
	}
	if (hsdtime == 0U) {
		hsdtime = MIPI_HOST_HSDTIME;
		mipi_info(dev, "default hsdtime: %u\n", hsdtime);
	}
	if (hsdtime > MIPI_HOST_HSDTIME_MAX) {
		hsdtime = MIPI_HOST_HSDTIME_MAX;
		mipi_info(dev, "hsdtime max as: %u\n", hsdtime);
	}
	time_ipi = (uint64_t)cfg->hsaTime + (uint64_t)cfg->hbpTime + (uint64_t)hsdtime + cycles_to_trans;
	time_ipi = MIPI_HOST_FREQ_MHZ_TO_NS * time_ipi * MIPI_HOST_FREQ_MHZ / pixclk;
	mipi_info(dev, "time to transmit last pixel in ipi: %llu\n", time_ipi);
	return (uint16_t)hsdtime;
}

/**
 * brief mipi_host_get_hsd_cutthrough:
 *
 * param [] cfg :
 *
 * return uint16_t
 */
static uint16_t mipi_host_get_hsd_cutthrough(const struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg,
		uint64_t pixclk)
{
	/**
	 * ipi_hsd_time > ((2*idi_clk_period)/ipi_clk_period)/(nlanes_active/4)) + 6
	 * ipi_hsd_time > ((2*ipi_clk)/rx_lane_byte_clk)/(nlanes_active/4)) + 6
	 * ipi_hsd_time > ((8*ipi_clk)/rx_lane_byte_clk)/nlanes_active) + 6
	 * ipi_hsd_time > (8*ipi_clk/rx_lane_byte_clk*nlanes_active) + 6
	 * ipi_hsd_time > (8*ipi_clk/rx_byte_clk) + 6
	 */
	const struct os_dev *dev = &hdev->osdev;
	uint64_t rx_bit_clk = (uint64_t)cfg->mipiclk * MIPI_HOST_FREQ_MHZ;
	uint16_t hsd = (uint16_t)(((pixclk * MIPI_HOST_HSD_BYTE_BITS) /
		(rx_bit_clk / MIPI_HOST_HSD_BYTE_BITS)) + MIPI_HOST_HSD_CAL_MIN);
	uint16_t hbp = cfg->hbpTime;
	uint16_t hsa = cfg->hsaTime;

	mipi_info(dev, "linelenth: %u, framelenth: %u, fps: %u, ipiclk: %llu, rx_bit_clk: %llu\n",
			cfg->linelenth, cfg->framelenth, cfg->fps, pixclk, rx_bit_clk);

	if(hsd > (MIPI_HOST_HSDTIME_MAX - MIPI_HOST_HSDTIME_MIN)) {
		hbp = hbp + (hsd - MIPI_HOST_HSDTIME_MAX);
		hsd = MIPI_HOST_HSDTIME_MAX;
		mipi_info(dev, "hsdtime max as: %u\n", hsd);
		if (hbp > (MIPI_HOST_HBPTIME_MAX - MIPI_HOST_HBPTIME_MIN)) {
			hsa = hsa + (hbp - MIPI_HOST_HBPTIME_MAX);
			hbp = MIPI_HOST_HBPTIME_MAX;
			mipi_info(dev, "hbptime max as: %u\n", hbp);
			if (hsa > (MIPI_HOST_HSATIME_MAX - MIPI_HOST_HSATIME_MIN)) {
				hsa = MIPI_HOST_HSATIME_MAX;
				mipi_info(dev, "hsatime max as: %u\n", hsa);
			} else {
				hsa += MIPI_HOST_HSATIME_MIN;
			}
		} else {
			hbp += MIPI_HOST_HBPTIME_MIN;
		}
	} else {
		hsd += MIPI_HOST_HSDTIME_MIN;
	}

	cfg->hbpTime = hbp;
	cfg->hsaTime = hsa;
	return hsd;
}

/**
 * brief mipi_host_get_hbp: get the ipi timing of hbp
 *
 * param [in] cfg : mipi host config's setting
 * param [in] ppi_width: the ppi width config: 0-8bit, 1-16bit
 *
 * return uint16_t
 */
static uint16_t mipi_host_get_hbp(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg, uint32_t ppi_width)
{
	const struct os_dev *dev = &hdev->osdev;
	struct mipi_host_param_s *param = &hdev->host.param;
	uint64_t ipiclk = (hdev->ipi_clock != 0UL) ? hdev->ipi_clock : MIPI_HOST_IPICLK_DEFAULT;
	uint32_t short_pkt_bytes = 4;
	uint32_t bytes_per_hsclk = (ppi_width == MIPI_HOST_PPI_WIDTH_16BIT) ? 2U : 1U;
	uint32_t t_ppi_hs = HOST_US_TO_NS * HOST_BITS_PER_BYTE * cfg->lane / cfg->mipiclk;
	uint32_t pkt2pkt = (cfg->ppi_pg != 0U) ?  mipi_host_get_pkt2pkt(hdev, cfg, ppi_width) : 0U;
	// TODO: when ppi_pg is disabled, get pkt2pkt(lp2hs, hs2lp) from phy
	uint32_t pkt2pkt_time = pkt2pkt * (HOST_US_TO_NS / (cfg->mipiclk /cfg->lane / HOST_BITS_PER_BYTE));
	uint32_t hsa = cfg->hsaTime;
	int32_t hbp = 0;
	uint32_t bits_per_pixel = mipi_host_get_bpp(hdev, cfg);
	uint32_t bytes_to_trans = cfg->width * bits_per_pixel / HOST_BITS_PER_BYTE;
	uint8_t  n_active_ipi = 0U;
	uint8_t  has_lsle = ((param->adv_value & MIPI_HOST_ADV_EN_LINE_START) != 0U) ? 1U : 0U;

        if (cfg->phy != 0U)
		short_pkt_bytes = 14U * cfg->lane;

#if 0
	for (vc = 0; vc < MAX_OF_CSI_VC; vc++) {
		if (NULL_OF_VC_ID != config->channel_sel[vc])
			n_active_ipi++;
	}
#else
	n_active_ipi = 1U;
#endif

	hbp = (has_lsle * (pkt2pkt_time + (2U * short_pkt_bytes / cfg->lane + 2U) * t_ppi_hs / bytes_per_hsclk) +
			has_lsle * pkt2pkt_time + bytes_to_trans / cfg->lane * t_ppi_hs / bytes_per_hsclk) \
		* (n_active_ipi - 1U) * ipiclk / HOST_S_TO_NS - hsa;

	if (hbp < MIPI_HOST_HBPTIME_P)
		hbp = MIPI_HOST_HBPTIME_P;
	else
		hbp++;
	mipi_dbg(param, dev, "has_lsle:%d, n_active_ipi:%d, hbp:%d\n", has_lsle, n_active_ipi, hbp);
	return hbp;
}

/**
 * brief mipi_host_get_hbp: get the ipi timing of hsd
 *
 * param [in] cfg : mipi host config's setting
 * param [in] ppi_width: the ppi width config: 0-8bit, 1-16bit
 *
 * return uint16_t
 */
static uint16_t mipi_host_get_hsd(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg, uint32_t ppi_width)
{
	uint64_t ipiclk = (hdev->ipi_clock != 0UL) ? hdev->ipi_clock : MIPI_HOST_IPICLK_DEFAULT;
	struct mipi_host_param_s *param = &hdev->host.param;
	void __iomem *iomem = &hdev->host.iomem;
	struct os_dev *dev = &hdev->osdev;
	uint64_t bits_per_pixel = mipi_host_get_bpp(hdev, cfg);
	uint64_t line_size = 0U;
	uint64_t cycles_to_trans = 0U;
	uint64_t rx_bit_clk = cfg->mipiclk * MIPI_HOST_FREQ_MHZ;
	uint32_t bytes_per_hsclk = (ppi_width == MIPI_HOST_PPI_WIDTH_16BIT) ? 2U : 1U;
	uint32_t t_ppi_hs = HOST_US_TO_NS * HOST_BITS_PER_BYTE * cfg->lane / cfg->mipiclk;
	uint32_t long_pkt_bytes = 6U;
	uint32_t hbp = cfg->hbpTime;
	uint32_t hsa = cfg->hsaTime;
	uint32_t bytes_to_trans = 0U;
	int32_t  hsd = 0;
	int32_t  hsd_min = 0;
	int32_t  hsd_max = 0;

	if (cfg->phy != 0U) {
		long_pkt_bytes = 14U * cfg->lane + 2U + 2U * (cfg->lane - 1U);
	}
	cycles_to_trans = cfg->width;
	line_size = cfg->linelenth;
	if ((MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5) == 0) || ((param->adv_value & MIPI_HOST_ADV_DT_OV_MASK) != 0U)) {
		if (cfg->datatype == MIPI_CSI2_DT_RAW_20 || cfg->datatype == MIPI_CSI2_DT_RAW_24) {
			cycles_to_trans = cfg->width * 2U;
			line_size = cfg->linelenth * 2U;
		}
	}
	bytes_to_trans = line_size * bits_per_pixel / HOST_BITS_PER_BYTE;
	if ((param->cut_through & MIPI_HOST_CUT_THROUGH_EN) == 0U) {
		hsd_min = ((long_pkt_bytes + bytes_to_trans) / cfg->lane)* t_ppi_hs / bytes_per_hsclk * ipiclk / HOST_S_TO_NS \
				- (hsa + hbp + cycles_to_trans);
	}
	hsd_max = (((long_pkt_bytes + bytes_to_trans) / cfg->lane) * t_ppi_hs / bytes_per_hsclk +
			((HOST_IPI_FIFO_DEPTH * HOST_BITS_PER_BYTE) / cfg->lane) * t_ppi_hs / bytes_per_hsclk) * ipiclk / HOST_S_TO_NS
			- (hsa + hbp + cycles_to_trans);

	if (hsd_min > hsd_max || hsd_max < 0) {
		mipi_info(dev, "Cannot find proper hsd in range %d~%d, please check mipiclk, lanes and ipiclk\n",
			hsd_min, hsd_max);
		hsd = 1;
		return (uint16_t)hsd;
	}
	if (hsd_min < 0) {
		hsd = 1;
	} else if (hsd_min >= 4096) {
		hsd = 4095;
		hbp += 1;
		if (hbp >= 4095) {
			hbp = 4095;
			hsa = (bits_per_pixel * line_size * ipiclk / rx_bit_clk) - (hbp + hsd + cycles_to_trans);
			if (hsa >= 4095) {
				hsa = 4095;
				mipi_info(dev, "mipi host ipi config out of range, check the input timing config\n");
			} else {
				hsa += 1;
			}
		}
		mipi_info(dev, "mipi host hsdtime out of range, reconfig hsa %d hbp %d hsd %d\n", hsa, hbp, hsd);
	} else {
		hsd = hsd_min + 1;
	}
	cfg->hbpTime = hbp;
	cfg->hsaTime = hsa;
	mipi_dbg(param, dev, "mipi host%s: min hsd: %d, max hsd: %d, set to %d\n",
		(param->cut_through & MIPI_HOST_CUT_THROUGH_EN) ? " cut_through" : "", hsd_min, hsd_max, hsd);
	return (uint16_t)hsd;
}

#if MIPI_HOST_INT_DBG
/**
 * brief mipi_host_irq_enable : Enale mipi host IRQ
 *
 * param [in] irq : IRQ Flag
 *
 * return void
 */
static void mipi_host_irq_enable(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_ierr_s *ierr = &host->ierr;
	const struct mipi_host_ireg_s *ireg;
	void __iomem *iomem = host->iomem;
	uint32_t temp;
	uint32_t i;

	if (iomem == NULL) {
		return;
	}

	(void)mhost_getreg(host, ierr->st_main);
	for (i = 0U; i < ierr->num; i++) {
		ireg = &ierr->iregs[i];
		(void)mhost_getreg(host, ireg->reg_st);
		temp = mhost_getreg(host, ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		temp |= ireg->err_mask;
		mhost_putreg(host, ireg->reg_mask, temp);
	}

#ifdef MIPI_HOST_INT_USE_TIMER
	hdev->irq_timer_en = 1U;
#endif

	return;
}

/**
 * brief mipi_host_irq_disable : Disable mipi host IRQ
 *
 * param [in] irq : IRQ Flag
 *
 * return void
 */
static void mipi_host_irq_disable(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_ierr_s *ierr = &host->ierr;
	const struct mipi_host_ireg_s *ireg;
	void __iomem *iomem = host->iomem;
	uint32_t temp;
	uint32_t i;

	if (iomem == NULL) {
		return;
	}

	for (i = 0U; i < ierr->num; i ++) {
		ireg = &ierr->iregs[i];
		temp = mhost_getreg(host, ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		mhost_putreg(host, ireg->reg_mask, temp);
	}

#if defined MIPI_HOST_INT_USE_TIMER
	hdev->irq_timer_en = 0U;
#endif

	return;
}

/**
 * brief mipi_host_irq_disable_mask : Disable mipi host IRQ with mask
 *
 * param [in] irq : IRQ Flag
 *
 * return void
 */
static void mipi_host_irq_disable_mask(struct mipi_hdev_s *hdev, uint32_t mask)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_ierr_s *ierr = &host->ierr;
	const struct mipi_host_ireg_s *ireg;
	void __iomem *iomem = host->iomem;
	uint32_t temp;
	uint32_t i;
#if defined MIPI_HOST_INT_USE_TIMER
	int32_t keep = 0;
#endif

	if (iomem == NULL) {
		return;
	}

	for (i = 0U; i < ierr->num; i ++) {
		ireg = &ierr->iregs[i];
		if ((mask & (uint32_t)(0x1UL << ireg->icnt_n)) != 0U) { /* qacfix: conversion */
			temp = mhost_getreg(host, ireg->reg_mask);
			temp &= ~(ireg->err_mask);
			mhost_putreg(host, ireg->reg_mask, temp);
#if defined MIPI_HOST_INT_USE_TIMER
		} else {
			temp = mhost_getreg(host, ireg->reg_mask);
			if (temp != 0U)
				keep = 1;
#endif
		}
	}

#if defined MIPI_HOST_INT_USE_TIMER
	if (keep == 0)
		hdev->irq_timer_en = 0U;
#endif

	return;
}

#if MIPI_HOST_INT_DBG_ERRSTR
static void mipi_host_subirq_parse_errstr(uint32_t irq_debug, uint32_t subirq,
					const struct mipi_host_ireg_s *ireg, char *s, size_t len)
{
	s[0] = '\0';
	if ((irq_debug & MIPI_HOST_IRQ_DEBUG_ERRSTR) != 0U) {
		int32_t j= 0, l = 0;
		uint32_t subirq_do = subirq;

		while((subirq_do != 0U) && (j < MIPI_HOST_INT_DBG_ERRBIT) && ((uint32_t)l < len)) {
			if ((subirq_do & (uint32_t)(0x1UL << (uint32_t)j)) != 0U) {
				l += snprintf(&s[l], len - (uint32_t)l, " %d:%s",
						j, (ireg->err_str[j] != NULL) ? ireg->err_str[j] : "rsv");
				subirq_do &= ~(uint32_t)(0x1UL << (uint32_t)j);
			}
			j++;
		}
	}
}
#endif

static void mipi_host_ipi_overflow_handle(struct mipi_host_s *host, uint32_t ipi)
{
	struct mipi_host_param_s *param = &host->param;
	uint32_t mem_flush = MIPI_HOST_MEMFLUSN_MANUAL;

#ifdef CONFIG_HOBOT_FUSA_DIAG
	(void)mipi_csi_stl_ignore(&host->stl,
			MIPI_HOST_OV_IGNORE_US(host->cfg.fps, param->stl_ovif));
#endif

	mem_flush |= (param->mem_flush != 0U) ? MIPI_HOST_MEMFLUSN_ENABLE : 0U;
	switch (ipi) {
	case MIPI_IPI1:
		mhost_putreg(host, REG_MIPI_HOST_IPI_MEM_FLUSH, mem_flush);
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, ~MIPI_HOST_IPI1_SOFTRSTN);
		break;
	case MIPI_IPI2:
		mhost_putreg(host, REG_MIPI_HOST_IPI2_MEM_FLUSH, mem_flush);
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, ~MIPI_HOST_IPI2_SOFTRSTN);
		break;
	case MIPI_IPI3:
		mhost_putreg(host, REG_MIPI_HOST_IPI3_MEM_FLUSH, mem_flush);
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, ~MIPI_HOST_IPI3_SOFTRSTN);
		break;
	case MIPI_IPI4:
		mhost_putreg(host, REG_MIPI_HOST_IPI4_MEM_FLUSH, mem_flush);
		mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, ~MIPI_HOST_IPI4_SOFTRSTN);
		break;
	default:
		/* do nothing */
		break;
	}
	mhost_putreg(host, REG_MIPI_HOST_IPI_SOFTRSTN, MIPI_HOST_ALLE_SOFTRSTN);
}

static uint32_t mipi_host_subirq_func(struct mipi_hdev_s *hdev, const struct mipi_host_ireg_s *ireg)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	struct os_dev *dev = &hdev->osdev;
	uint32_t reg = ireg->reg_st;
	uint32_t icnt_n = ireg->icnt_n;
	uint32_t subirq;
#if MIPI_HOST_INT_DBG_ERRSTR
	char err_str[MIPI_HOST_INT_DBG_ERRBUF];
#else
	char err_str[1] = '\0';
#endif
	const char *err_op = "";
	int32_t need_ipireset = 0;

#if defined MIPI_HOST_INT_USE_TIMER && defined CONFIG_ARCH_ZYNQMP
	if ((host->ap != 0) && (reg == REG_MIPI_HOST_INT_ST_AP_GENERIC)) {
		subirq = hdev->fatal_ap_generic;
	} else
#endif
	{
		subirq = mhost_getreg(host, reg) & ireg->err_mask;
	}

	if (param->drop_func != MIPI_HOST_PARAM_DROP_IN_STL) {
		mipi_host_drop_func(host, ireg->icnt_n, subirq);
	}

	if (subirq != 0U) {
#if MIPI_HOST_INT_DBG_ERRSTR
		mipi_host_subirq_parse_errstr(param->irq_debug, subirq, ireg, err_str, sizeof(err_str));
#endif
		if ((icnt_n >= MIPI_HOST_ICNT_IPI) && (icnt_n <= MIPI_HOST_ICNT_IPIE) &&
			((subirq & MIPI_HOST_IREG_ERR_OVERFLOW) != 0U) && (param->ipi_overst != 0U)) {
			err_op = " --> reset";
			need_ipireset = 1;
		}
		if ((param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR) != 0U) {
			mipi_err(dev, "  %s: 0x%x%s%s\n",
					g_mh_icnt_names[icnt_n], subirq, err_str, err_op);
		} else {
			mipi_dbg(param, dev, "  %s: 0x%x%s%s\n",
					g_mh_icnt_names[icnt_n], subirq, err_str, err_op);
		}

		if (need_ipireset != 0) {
			mipi_host_ipi_overflow_handle(&hdev->host, icnt_n - MIPI_HOST_ICNT_IPI);
		}

		/* interrupt callbcak */
		if (hdev->cb.int_cb != NULL) {
			hdev->cb.int_cb(hdev->port, icnt_n, subirq);
		}
	}

#ifdef CONFIG_HOBOT_FUSA_DIAG
	if (host->ap != 0) {
		(void)mipi_csi_stl_interrupt(&host->stl, icnt_n, subirq);
#if 0
	} else {
		/* debug for diag callbcak */
		if ((host->stl.callback != NULL) && (host->stl.callback->fusa_handler_cb != NULL)) {
			struct diag_cb_info info = { 0 };
			info.fchm_err_info.fchm_err_code = 1;
			host->stl.callback->fusa_handler_cb(&host->stl, &info);
		}
#endif
	}
#endif

	return subirq;
}

/**
 * brief mipi_host_subirq_loop: irq bit loop
 *
 * param [in] struct mipi_hdev_s *hdev: host device
 * param [in] uint32_t irq: irq st_main
 *
 * return uint32_t: valid bit count
 */
static uint32_t mipi_host_subirq_loop(struct mipi_hdev_s *hdev, uint32_t irq)
{
	struct mipi_host_ierr_s *ierr = &hdev->host.ierr;
	const struct mipi_host_ireg_s *ireg;
	struct mipi_host_icnt_s *icnt = &hdev->host.icnt;
	uint32_t *icnt_p = &icnt->st_main;
	uint32_t mask, icnt_n;
	uint32_t irq_do;
	uint32_t subirq;
	uint32_t i, valid_bits = 0U;

	if(irq == 0U) {
		return valid_bits;
	}

	irq_do = irq;
	icnt->st_main++;
	for (i = 0U; i < ierr->num; i++) {
		ireg = &ierr->iregs[i];
		mask = ireg->st_mask;
		if ((irq_do & mask) == 0U) {
			continue;
		}

		subirq = mipi_host_subirq_func(hdev, ireg);
		if (subirq == 0U) {
			irq_do &= ~mask;
			continue;
		}

		icnt_n = ireg->icnt_n;
		icnt_p[icnt_n]++;
		valid_bits++;
		irq_do &= ~mask;
		if (irq_do == 0U) {
			break;
		}
	}
	return valid_bits;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) irq fucntion
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void hobot_mipi_host_irq_func_do(struct mipi_hdev_s *hdev)
{
	const struct os_dev *dev;
	struct mipi_host_s *host;
	struct mipi_host_ierr_s *ierr;
	struct mipi_host_param_s *param;
	struct mipi_host_icnt_s *icnt;
	void __iomem *iomem;
	uint32_t irq;

	if (hdev == NULL) {
		return;
	}
	dev = &hdev->osdev;
	host = &hdev->host;
	iomem = host->iomem;
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_10, 0U, __LINE__);
		return;
	}
	ierr = &host->ierr;
	param = &host->param;
	icnt = &host->icnt;

#ifdef MIPI_HOST_INT_USE_TIMER
	irq = hdev->irq_st_main;
#else
	irq = mhost_getreg(host, ierr->st_main);
#endif
	if ((param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR) != 0U) {
		mipi_err(dev, "irq%s status 0x%x\n", (host->ap != 0) ? " ap" : "", irq);
	} else {
		mipi_dbg(param, dev, "irq%s status 0x%x\n", (host->ap != 0) ? " ap" : "", irq);
	}

	(void)mipi_host_subirq_loop(hdev, irq);

	if (icnt->st_main > param->irq_cnt) {
		if (param->ipi_overst != 0U)
			mipi_host_irq_disable_mask(hdev, ~MIPI_HOST_ICNT_IPI_MASK);
		else
			mipi_host_irq_disable(hdev);
	}

	return;
}

#ifdef MIPI_HOST_INT_USE_TIMER
void mipi_host_irq_timer_func(osal_timer_t *t)
{
	struct mipi_hdev_s *hdev = from_timer(hdev, t, irq_timer);
	struct mipi_host_s *host;
	struct mipi_host_ierr_s *ierr;
	void __iomem *iomem;

	if (hdev == NULL) {
		return;
	}
	host = &hdev->host;
	iomem = host->iomem;
	if (iomem == NULL) {
		return;
	}
	ierr = &host->ierr;

	if (hdev->irq_timer_en != 0U) {
		hdev->irq_st_main = mhost_getreg(host, ierr->st_main);
#ifdef CONFIG_ARCH_ZYNQMP
		/* ignore AP_GENERIC: apb_ap_err */
		if ((host->ap != 0) && ((hdev->irq_st_main & MIPI_HOST_1P4AP_INT_ST_AP_GENERIC) != 0U)) {
			hdev->fatal_ap_generic = mhost_getreg(host, REG_MIPI_HOST_INT_ST_AP_GENERIC);
			hdev->fatal_ap_generic &= ~MIPI_HOST_1P4AP_INT_APB_APERR_MASK;
			if (hdev->fatal_ap_generic == 0U) {
				hdev->irq_st_main &= ~MIPI_HOST_1P4AP_INT_ST_AP_GENERIC;
			}
		}
#endif
		if (hdev->irq_st_main != 0U) {
			mipi_host_irq_func(-1, hdev);
		}
		osal_timer_start(&hdev->irq_timer, 10);
	} else {
		osal_timer_start(&hdev->irq_timer, 10);
	}
}
#endif
#endif

static int32_t mipi_host_dphy_wait_stop(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint32_t ncount = 0U;
	uint32_t stopstate;

	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_1, 0U, __LINE__);
		return -1;
	}
	if (cfg == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostParamCheckErr, SUB_ID_5, 0U, __LINE__);
		return -1;
	}

	mipi_info(dev, "check phy stop state\n");
	/*Check that data lanes are in Stop state*/
	do {
		stopstate = mhost_getreg(host, REG_MIPI_HOST_PHY_STOPSTATE);
#ifdef X5_CHIP
		if ((stopstate & HOST_DPHY_LANE_STOP(cfg->lane)) == HOST_DPHY_LANE_STOP(cfg->lane)) {
#else
		if ((stopstate & HOST_DPHY_LANE_STOP_ALL) == HOST_DPHY_LANE_STOP(cfg->lane)) {
#endif
#ifdef CONFIG_MIPI_CSI_STL_PILE_ENABLE
			(void)mipi_csi_stl_phychk(&host->stl, cfg->lane, MIPI_CSI_PILE_PHYCHK);
#endif
			return 0;
		}
		osal_mdelay(1); /* PRQA S 2877,2880 */ /* osal_mdelay macro */
		ncount++;
	} while ((param->notimeout != 0U) || (ncount <= param->wait_ms));

#ifdef CONFIG_HOBOT_FUSA_DIAG
	(void)mipi_csi_stl_phychk(&host->stl, cfg->lane, 0);
#endif
	mipi_err(dev, "lane state of host phy is error: 0x%x\n", stopstate);
	mipi_host_error_report(hdev, ESW_MipiHostDphyStateErr, SUB_ID_0, (uint8_t)stopstate, __LINE__);
	return -1;
}

/**
 * brief mipi_host_dphy_start_hs_reception : check if mipi host in hs mode
 *
 * param []
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_dphy_start_hs_reception(struct mipi_hdev_s *hdev)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param;
	void __iomem *iomem = host->iomem;
	uint16_t ncount = 0;
	uint32_t state;
	int32_t poth;

	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_3, 0U, __LINE__);
		return -1;
	}

	mipi_info(dev, "check hs reception\n");

	/* param from main host if ex */
	if (hdev->is_ex != 0) {
		poth = mipi_host_port_other(hdev);
		if ((poth < 0) || (poth >= MIPI_HOST_MAX_NUM) || (g_hdev[poth] == NULL)) {
			mipi_err(dev, "start hs get main host error\n");
			mipi_host_error_report(hdev, ESW_MipiHostDphyOpErr, SUB_ID_2, 0U, __LINE__);
			return -1;
		}
		param = &(g_hdev[poth]->host.param);
	} else {
		param = &host->param;
	}
	/*Check that clock lane is in HS mode*/
	do {
		state = mhost_getreg(host, REG_MIPI_HOST_PHY_RX);
		if ((state & HOST_DPHY_RX_HS) == HOST_DPHY_RX_HS) {
			mipi_info(dev, "entry hs reception\n");
#ifdef CONFIG_HOBOT_FUSA_DIAG
			(void)mipi_csi_stl_phychk(&host->stl, host->cfg.lane,
				(uint32_t)(host->cfg.mipiclk) / host->cfg.lane);
#endif
			return 0;
		}
		ncount++;
		osal_mdelay(1); /* PRQA S 2877,2880 */ /* osal_mdelay macro */
	} while ((param->notimeout != 0U) || (ncount <= param->wait_ms));

#ifdef CONFIG_HOBOT_FUSA_DIAG
	(void)mipi_csi_stl_phychk(&host->stl, host->cfg.lane, 1);
#endif
	mipi_err(dev, "hs reception check error 0x%x\n", state);
	mipi_host_error_report(hdev, ESW_MipiHostDphyStateErr, SUB_ID_1, HOST_DPHY_RX_HS_STA(state), __LINE__);
	return -1;
}

/**
 * brief mipi_host_start : set mipi host start working
 *
 * param [] void :
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_start(struct mipi_hdev_s *hdev)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint32_t nocheck;
	int32_t poth;

	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_2, 0U, __LINE__);
		return -1;
	}

	if ((hdev->is_ex == 0) && (param->need_stop_check == 0U) && (param->stop_check_instart != 0U)) {
		if (0 != mipi_host_dphy_wait_stop(hdev, &host->cfg)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipi_err(dev, "wait phy stop state error\n");
			return -1;
		}
	}
	/* param from main host if ex */
	if (hdev->is_ex != 0) {
		poth = mipi_host_port_other(hdev);
		if ((poth < 0) || (poth >= MIPI_HOST_MAX_NUM) || (g_hdev[poth] == NULL)) {
			mipi_err(dev, "start get main host error\n");
			mipi_host_error_report(hdev, ESW_MipiHostDphyOpErr, SUB_ID_3, 0U, __LINE__);
			return -1;
		}
		nocheck = g_hdev[poth]->host.param.nocheck;
	} else {
		nocheck = param->nocheck;
	}
	if ((host->cfg.ppi_pg != 0U) && (MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5) != 0)) {
		if (0 != mipi_host_enable_ppi_pg(hdev, &host->cfg, 1)) {
			mipi_err(dev, "ppi pg 0x%x enable error\n", host->cfg.ppi_pg);
			return -1;
		}
		/* no need check hs if ppi pg enable */
		nocheck = 1U;
	}

	/* merge lane mode(4 lane mode) don't need to set source */
	if (hdev->lane_mode == 0) {
		(void)mipi_host_dphy_set_source(iomem);
	}

	if (nocheck == 0U) {
		if (0 != mipi_host_dphy_start_hs_reception(hdev)) {
			mipi_err(dev, "hs reception state error\n");
			return -1;
		}
	}

#if MIPI_HOST_INT_DBG
	if(hdev->is_ex == 0) {
		mipi_host_irq_enable(hdev);
	}
#endif

#ifdef CONFIG_HOBOT_FUSA_DIAG
	if(hdev->is_ex == 0) {
#if MIPI_HOST_INT_DBG
		/* no irq print and count limit when stl */
		uint32_t irq_debug = 0U, irq_cnt = MIPI_HOST_IRQ_CNT_MAX, dbg_value = 0U;
		mipi_host_param_dbg_set_save(param, &irq_debug, &irq_cnt, &dbg_value);
#endif
		(void)mipi_csi_stl_start(&host->stl);
		(void)mipi_csi_stl_invalid(&host->stl,
			MIPI_HOST_ST_INVALID_US(host->cfg.fps, param->stl_stif));
#if MIPI_HOST_INT_DBG
		/* backup irq config */
		mipi_host_param_dbg_set_save(param, &irq_debug, &irq_cnt, &dbg_value);
		/* reset icnt */
		(void)memset((void *)(&host->icnt), 0, sizeof(host->icnt));
#endif
	}
#endif

	return 0;
}

/**
 * brief mipi_host_stop : set mipi host stop working
 *
 * param [] void :
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_stop(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	void __iomem *iomem = host->iomem;
	int32_t i = 0;

	if ((host->cfg.ppi_pg != 0U) && (MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5) != 0))
		(void)mipi_host_enable_ppi_pg(hdev, &host->cfg, 0);

	/*stop mipi host here?*/
#if MIPI_HOST_INT_DBG
	if(hdev->is_ex == 0) {
		mipi_host_irq_disable(hdev);
	}
#endif

#ifdef CONFIG_HOBOT_FUSA_DIAG
	if(hdev->is_ex == 0) {
		(void)mipi_csi_stl_stop(&hdev->host.stl);
	}
#endif

	/* In x5 start stop loop testing, incomplete frames often occur.
	 * These data should be cleared when stopping, otherwise ipi overflow maybe occur.
	 * */
	for (i = 0; i < MIPIHOST_CHANNEL_NUM; i++) {
		if (i >= (int32_t)cfg->channel_num) {
			continue;
		} else {
			mipi_host_ipi_overflow_handle(host, i);
		}
	}

        return 0;
}

/**
 * brief mipi_host_deinit : mipi host deinit function
 *
 * param [] void :
 *
 * return int32_t : 0/-1
 */
static void mipi_host_deinit(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	void __iomem  *iomem = host->iomem;
	int32_t i;

	if (iomem == NULL) {
		return;
	}

#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_host_dphy_reset(iomem);
#endif
	/*Set Synopsys D-PHY Reset*/
	mhost_putreg(host, REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RESETN);
	mhost_putreg(host, REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RESETN);
	/*Release DWC_mipi_csi2_host from reset*/
	mhost_putreg(host, REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);

	if (hdev->ipi_clock != 0U) {
		i = 0;
		do {
			(void)mipi_host_configure_clk(hdev, host->socclk.ipiclk[i], 0, 0);
			i++;
		} while (i < (int32_t)host->cfg.channel_num); /* qacfix: conversion */
		hdev->ipi_clock = 0U;
	}

#ifdef CONFIG_HOBOT_FUSA_DIAG
	if (hdev->is_ex == 0) {
		(void)mipi_csi_stl_deinit(&host->stl);
	}
#endif

	return;
}

/* host init sensor_mclk */
static int32_t mipi_host_init_mclk(struct mipi_hdev_s *hdev, const mipi_host_cfg_t *cfg)
{
	struct os_dev *dev = &hdev->osdev;

	/* cfg mclk value:
	 * 0      : disable only
	 * 1      : enable only
	 * 2~24   : invalid and drop
	 * 25~636 : invalid and error
	 * 637~   : *10K = freq -> 6.37~655.35MHz
	 */
	if (cfg->mclk >= (uint16_t)(MIPI_HOST_SNRCLK_FREQ_MIN / MIPI_HOST_SNRCLK_FREQ_BASE)) {
		if (mipi_host_snrclk_set_freq(hdev, (uint32_t)(cfg->mclk * MIPI_HOST_SNRCLK_FREQ_BASE)) != 0) {
			return -1;
		}
		if (mipi_host_snrclk_set_en(hdev, 1) != 0) {
			return -1;
		}
	} else if (cfg->mclk > (uint16_t)MIPI_HOST_SNRCLK_IGNORE) {
		mipi_info(dev, "mclk %d should >= %lu(%luHz)\n", cfg->mclk,
				(MIPI_HOST_SNRCLK_FREQ_MIN / MIPI_HOST_SNRCLK_FREQ_BASE), MIPI_HOST_SNRCLK_FREQ_MIN);
		mipi_host_error_report(hdev, ESW_MipiHostSnrclkSetErr, SUB_ID_6, (uint8_t)(cfg->mclk / MIPI_HOST_FREQ_MHZ), __LINE__);
		return -1;
	} else if (cfg->mclk > (uint16_t)MIPI_HOST_SNRCLK_ENABLE) {
		mipi_info(dev, "mclk %d ignore\n", cfg->mclk);
	} else if (cfg->mclk == (uint16_t)MIPI_HOST_SNRCLK_ENABLE) {
		(void)mipi_host_snrclk_set_en(hdev, 1);
	} else {
		(void)mipi_host_snrclk_set_en(hdev, 0);
	}

	return 0;
}

/* host init common for main&extra port */
static int32_t mipi_host_init_common(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	void __iomem  *iomem = host->iomem;
	const char *phy_mode = "dphy";
	uint32_t ppi_width = MIPI_HOST_PPI_WIDTH_8BIT;
	int32_t is_1p5;

	if (iomem == NULL) {
		return -1;
	}
	is_1p5 = MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5);

	/*Set DWC_mipi_csi2_host reset*/
	mhost_putreg(host, REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);
	/*Set Synopsys D-PHY Reset*/
	mhost_putreg(host, REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RESETN);
	mhost_putreg(host, REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RESETN);

	/* phy mode config */
	if (is_1p5 != 0) {
		if (cfg->phy != 0U) {
			mhost_putreg(host, REG_MIPI_HOST_PHY_MODE, MIPI_HOST_PHY_MODE_CPHY);
			phy_mode = "cphy";
			ppi_width = MIPI_HOST_PPI_WIDTH_16BIT;
		} else {
			mhost_putreg(host, REG_MIPI_HOST_PHY_MODE, MIPI_HOST_PHY_MODE_DPHY);
			if (cfg->mipiclk > (cfg->lane * HOST_DPHY_CLK_PPI8_MAX))
				ppi_width = MIPI_HOST_PPI_WIDTH_16BIT;
		}
		if (cfg->ppi_pg != 0U)
			ppi_width = MIPI_HOST_PPI_WIDTH_16BIT;
		mhost_putreg(host, REG_MIPI_HOST_PHY_CFG, ppi_width);
	}

#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_info(dev, "%s %dMbps/%dlane: ppi %dbit settle %d\n",
		phy_mode, cfg->mipiclk, cfg->lane, ((ppi_width + 1) * 8), cfg->settle);
	if (0 != mipi_host_dphy_initialize(cfg->mipiclk, cfg->lane, cfg->settle, iomem)) {
		mipi_err(dev, "%s initialize error\n", phy_mode);
		mipi_host_error_report(hdev, ESW_MipiHostDphyOpErr, SUB_ID_1, (uint8_t)cfg->lane, __LINE__);
		(void)mipi_host_deinit(hdev);
		return -1;
	}

	(void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, hdev->port,
		MIPI_CFG_CLK_FREQRANGE, MIPI_HOST_CFGCLK_DEFAULT);
	osal_udelay(1);

	(void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, hdev->port,
		MIPI_PHY_ENABLE_CLK, 0x1);
#endif

	/*Clear Synopsys D-PHY Reset*/
	mhost_putreg(host, REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RAISE);
	mhost_putreg(host, REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RAISE);
	/*Configure the number of active lanes*/
	mhost_putreg(host, REG_MIPI_HOST_N_LANES, (uint32_t)(cfg->lane) - 1U);
	osal_udelay(1000); /* PRQA S 2880 */ /* osal_udelay macro */
	/*Configure the vc&dt lines monitoring*/
	if (param->data_ids_1 != 0U) {
		mipi_info(dev, "data id monitor 0~3: vc=0x%x dt=0x%x\n",
			param->data_ids_vc1, param->data_ids_1);
		mhost_putreg(host, REG_MIPI_HOST_DATA_IDS_1, param->data_ids_1);
		mhost_putreg(host, REG_MIPI_HOST_DATA_IDS_VC1, param->data_ids_vc1);
	}
	if (param->data_ids_2 != 0U) {
		mipi_info(dev, "data id monitor 4~7: vc=0x%x dt=0x%x\n",
			param->data_ids_vc2, param->data_ids_2);
		mhost_putreg(host, REG_MIPI_HOST_DATA_IDS_2, param->data_ids_2);
		mhost_putreg(host, REG_MIPI_HOST_DATA_IDS_VC2, param->data_ids_vc2);
	}
	/*Configure the vc extersion function*/
	if (param->vcext_en != 0U) {
		mhost_putreg(host, REG_MIPI_HOST_VC_EXTENSION, MIPI_HOST_VC_EXT_LEGACY);
	} else {
		mhost_putreg(host, REG_MIPI_HOST_VC_EXTENSION, MIPI_HOST_VC_EXT_ENABLE);
	}

	if (hdev->is_ex == 0) {
		if ((param->cut_through & MIPI_HOST_CUT_HSD_LEGACY) != 0U) {
			cfg->hsaTime = (cfg->hsaTime != 0U) ? cfg->hsaTime : (uint16_t)MIPI_HOST_HSATIME;
			cfg->hbpTime = (cfg->hbpTime != 0U) ? cfg->hbpTime : (uint16_t)MIPI_HOST_HBPTIME;
			cfg->hsdTime = (cfg->hsdTime != 0U) ? cfg->hsdTime :
				(((param->cut_through & MIPI_HOST_CUT_THROUGH_EN) != 0U) ?
					mipi_host_get_hsd_cutthrough(hdev, cfg, hdev->ipi_clock) :
					mipi_host_get_hsd_legacy(hdev, cfg, hdev->ipi_clock));
		} else {
			cfg->hsaTime = (cfg->hsaTime != 0U) ? cfg->hsaTime : (uint16_t)MIPI_HOST_HSATIME_P;
			cfg->hbpTime = (cfg->hbpTime != 0U) ? cfg->hbpTime : mipi_host_get_hbp(hdev, cfg, ppi_width);
			cfg->hsdTime = (cfg->hsdTime != 0U) ? cfg->hsdTime : mipi_host_get_hsd(hdev, cfg, ppi_width);
		}
		mipi_info(dev, "ipi config hsa: %u, hbp: %u, hsd: %u",
				cfg->hsaTime, cfg->hbpTime, cfg->hsdTime);
		if (0 != mipi_host_configure_ipi(hdev, cfg)) {
			mipi_err(dev, "configure ipi error!!!\n");
			(void)mipi_host_deinit(hdev);
			return -1;
		}

		if ((is_1p5 != 0) && (cfg->ppi_pg != 0U)) {
			mipi_host_configure_ppi_pg(hdev, cfg, ppi_width);
		}
	}

	/*Release DWC_mipi_csi2_host from reset*/
	mhost_putreg(host, REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RAISE);

	return 0;
}

/**
 * brief mipi_host_init : mipi host init function
 *
 * param [in] cfg : mipi host cfgler's setting
 *
 * return int32_t : 0/-1
 */
static int32_t mipi_host_init(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_param_s *param = &host->param;
	void __iomem  *iomem = host->iomem;
	uint64_t pixclk;

	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_0, 0U, __LINE__);
		return -1;
	}

	mipi_info(dev, "init begin\n");
	mipi_info(dev, "%d lane %dx%d %dfps datatype 0x%x\n",
			 cfg->lane, cfg->width, cfg->height, cfg->fps, cfg->datatype);
	if (hdev->is_ex == 0) {
		/* mclk clock */
		if (mipi_host_init_mclk(hdev, cfg) != 0) {
			mipi_err(dev, "mclk config error!\n");
			return -1;
		}
		/* ipi clock */
		pixclk = mipi_host_pixel_clk_select(hdev, cfg);
		if (pixclk == 0U) {
			mipi_err(dev, "pixel clk config error!\n");
			mipi_host_error_report(hdev, ESW_MipiHostHwConfigErr, SUB_ID_2, 0U, __LINE__);
			return -1;
		}
		hdev->ipi_clock = pixclk;
	}

	if(mipi_host_init_common(hdev, cfg) != 0) {
		return -1;
	}
	if ((hdev->is_ex == 0) && (param->need_stop_check == 0U) && (param->stop_check_instart == 0U)) {
		if (0 != mipi_host_dphy_wait_stop(hdev, cfg)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipi_err(dev, "wait phy stop state error\n");
			(void)mipi_host_deinit(hdev);
			return -1;
		}
	}
	if (hdev->is_ex == 0) {
#ifdef CONFIG_HOBOT_FUSA_DIAG
		{
#if MIPI_HOST_INT_DBG
			/* no irq print32_t and count limit when stl */
			uint32_t irq_debug = 0U, irq_cnt = MIPI_HOST_IRQ_CNT_MAX, dbg_value = 0U;
			mipi_host_param_dbg_set_save(param, &irq_debug, &irq_cnt, &dbg_value);
#ifdef MIPI_HOST_INT_USE_TIMER
			/* enable irq timer for stl */
			hdev->irq_timer_en = 1U;
#endif
#endif
			(void)mipi_csi_stl_init(&host->stl);
#if MIPI_HOST_INT_DBG
			/* backup irq config */
			mipi_host_param_dbg_set_save(param, &irq_debug, &irq_cnt, &dbg_value);
#ifdef MIPI_HOST_INT_USE_TIMER
			hdev->irq_timer_en = 0U;
#endif
#endif
		}
#endif
	}

#if MIPI_HOST_INT_DBG
	(void)memset((void *)(&host->icnt), 0, sizeof(host->icnt));
#endif
	(void)memcpy(&host->cfg, cfg, sizeof(mipi_host_cfg_t));
	mipi_info(dev, "init end\n");
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device open operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_open_do(struct mipi_hdev_s *hdev)
{
	struct mipi_user_s *user = &hdev->user;
	struct mipi_host_param_s *param = &hdev->host.param;
	struct os_dev *dev = &hdev->osdev;

	osal_mutex_lock(&user->open_mutex);
	mipi_dbg(param, dev, "open as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
		osal_mutex_init(&user->mutex); /* PRQA S 3334 */ /* mutex_init macro */
		osal_waitqueue_init(&user->pre_wq);
		user->pre_state = MIPI_PRE_STATE_DEFAULT;
		user->init_cnt = 0U;
		user->start_cnt = 0U;
		(void)mipi_host_configure_clk(hdev, hdev->host.socclk.refclk, MIPI_HOST_REFCLK_MHZ, 1);
	}
	user->open_cnt++;
	osal_mutex_unlock(&user->open_mutex);

	// file->private_data = (void *)hdev;
	return 0;
}

static void hobot_mipi_host_params_release(struct mipi_host_param_s *param)
{
	param->ipi1_dt = 0U;
	param->ipi2_dt = 0U;
	param->ipi3_dt = 0U;
	param->ipi4_dt = 0U;
	param->pkt2pkt_time = 0U;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device close operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_close_do(struct mipi_hdev_s *hdev)
{
	struct mipi_hdev_s *ex_hdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_user_s *user = &hdev->user;
	struct mipi_host_param_s *param = &host->param;
	struct os_dev *dev = &hdev->osdev;

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt > 0U) {
		user->open_cnt--;
	}
	mipi_dbg(param, dev, "close as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
		if (host->state != MIPI_STATE_DEFAULT) {
			(void)mipi_host_stop(hdev);
			(void)mipi_host_deinit(hdev);
			if (hdev->ex_hdev != NULL) {
				ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
#ifndef X5_CHIP
				(void)mipi_host_stop(ex_hdev);
#endif
				(void)mipi_host_deinit(ex_hdev);
			}
			host->state = MIPI_STATE_DEFAULT;
		}
		hobot_mipi_host_params_release(param);
		(void)mipi_host_configure_clk(hdev, hdev->host.socclk.refclk, 0, 0);
                (void)mipi_host_snrclk_set_freq(hdev, 0);  //disable clk
                (void)mipi_host_snrclk_set_en(hdev,0);    //select pinctrl to gpio

	}
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}

static int32_t hobot_mipi_host_ioc_init_do(struct mipi_hdev_s *hdev, mipi_host_cfg_t *cfg)
{
	struct mipi_hdev_s *ex_hdev;
	struct os_dev *dev = &hdev->osdev;
	int32_t ret;

	if (0 != mipi_host_configure_check(hdev, cfg)) {
		mipi_err(dev, "init cfg check error\n");
		return -EINVAL;
	}
	if (0 != mipi_host_configure_lanemode(hdev, (int32_t)cfg->lane)) {
		mipi_err(dev, "require %dlane error\n", cfg->lane);
		return -EACCES;
	}
	if (hdev->ex_hdev != NULL) {
		mipi_host_cfg_t mipi_exth_cfg;
		(void)memcpy(&mipi_exth_cfg, cfg, sizeof(mipi_host_cfg_t));
		ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
		ret = mipi_host_init(ex_hdev, &mipi_exth_cfg);
		if (ret != 0) {
			return ret;
		}
	}
	ret = mipi_host_init(hdev, cfg);
	if (ret != 0) {
		mipi_err(dev, "init error: %d\n", ret);
		return ret;
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_init(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	mipi_host_cfg_t mipi_host_cfg;

	if (arg == 0U) {
		mipi_err(dev, "cmd init, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_2, (uint8_t)(user->init_cnt), __LINE__);
		return -EINVAL;
	}
	if (mipi_copy_from_app((void *)&mipi_host_cfg,
				(void __user *)arg, sizeof(mipi_host_cfg_t)) != 0U) {
		mipi_err(dev, "init error, config %px from user error\n", (void __user *)arg);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_0, 0U, __LINE__);
		return -EINVAL;
	}

	osal_mutex_lock(&user->mutex);
	mipi_info(dev, "init cmd: %u %s\n", user->init_cnt,
			(user->init_cnt != 0U) ? "drop" : "real");
	if (user->init_cnt == 0U) {
		if (MIPI_STATE_DEFAULT != host->state) {
			mipi_info(dev, "re-init, pre state: %d(%s)\n",
					host->state, g_mh_state[host->state]);
		}
		ret = hobot_mipi_host_ioc_init_do(hdev, &mipi_host_cfg);
		if (ret != 0) {
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if (user->pre_state == (uint32_t)MIPI_PRE_STATE_INITING) {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_INITED;
			user->pre_done = (bool)true;
			osal_wake_up(&user->pre_wq);
		} else {
			user->pre_state = MIPI_PRE_STATE_INITED;
		}
		host->state = MIPI_STATE_INIT;
	} else if (mipi_host_configure_cmp(&host->cfg, &mipi_host_cfg) != 0) {
		mipi_info(dev, "warning: init config mismatch\n");
	} else {
		/* init done */
	}
	user->init_cnt++;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_deinit(struct mipi_hdev_s *hdev)
{
	struct mipi_hdev_s *ex_hdev;
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;

	osal_mutex_lock(&user->mutex);
	if (user->init_cnt > 0U) {
		user->init_cnt--;
	}
	mipi_info(dev, "deinit cmd: %u %s\n", user->init_cnt,
			(user->init_cnt != 0U) ? "drop" : "real");
	if (user->init_cnt == 0U) {
		if (MIPI_STATE_START == host->state) {
			(void)mipi_host_stop(hdev);
			if (hdev->ex_hdev != NULL) {
				ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
				(void)mipi_host_stop(ex_hdev);
			}
			user->start_cnt = 0U;
		}
		(void)mipi_host_deinit(hdev);
		if (hdev->ex_hdev != NULL) {
			ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
			(void)mipi_host_deinit(ex_hdev);
			ex_hdev->is_ex = 0;
			hdev->ex_hdev = NULL;
		}
		user->pre_state = MIPI_PRE_STATE_DEFAULT;
		host->state = MIPI_STATE_DEFAULT;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_start(struct mipi_hdev_s *hdev)
{
#ifndef X5_CHIP
	struct mipi_hdev_s *ex_hdev;
#endif
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;

	osal_mutex_lock(&user->mutex);
	mipi_info(dev, "start cmd: %u %s\n", user->start_cnt,
			(user->start_cnt != 0U) ? "drop" : "real");
	if (user->start_cnt == 0U) {
		if (MIPI_STATE_START == host->state) {
			mipi_info(dev, "already in start state\n");
			user->start_cnt++;
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if ((MIPI_STATE_INIT != host->state) &&
			(MIPI_STATE_STOP != host->state)) {
			mipi_err(dev, "state error, current state: %d(%s)\n",
					host->state, g_mh_state[host->state]);
			osal_mutex_unlock(&user->mutex);
			mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_2, (uint8_t)(host->state), __LINE__);
			return -EBUSY;
		}
#ifndef X5_CHIP
		/* state ok */
		if (hdev->ex_hdev != NULL) {
			ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
			ret = mipi_host_start(ex_hdev);
			if (ret != 0) {
				osal_mutex_unlock(&user->mutex);
				return ret;
			}
		}
#endif
		ret = mipi_host_start(hdev);
		if (ret != 0) {
			mipi_err(dev, "start error: %d\n", ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if (user->pre_state == (uint32_t)MIPI_PRE_STATE_STARTING) {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_STARTED;
			user->pre_done = (bool)true;
			osal_wake_up(&user->pre_wq);
		} else {
			user->pre_state = MIPI_PRE_STATE_STARTED;
		}
		host->state = MIPI_STATE_START;
	}
	user->start_cnt++;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_stop(struct mipi_hdev_s *hdev)
{
#ifndef X5_CHIP
	struct mipi_hdev_s *ex_hdev;
#endif
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	uint32_t start_cnt_save;

	osal_mutex_lock(&user->mutex);
	start_cnt_save = user->start_cnt;
	if (user->start_cnt > 0U) {
		user->start_cnt--;
	}
	mipi_info(dev, "stop cmd: %u %s\n", user->start_cnt,
			(user->start_cnt != 0U) ? "drop" : "real");
	if (user->start_cnt == 0U) {
		if (MIPI_STATE_STOP == host->state) {
			mipi_info(dev, "already in stop state\n");
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if (MIPI_STATE_START != host->state) {
			mipi_err(dev, "state error, current state: %d(%s)\n",
					host->state, g_mh_state[host->state]);
			user->start_cnt = start_cnt_save;
			osal_mutex_unlock(&user->mutex);
			mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_3, (uint8_t)(host->state), __LINE__);
			return -EBUSY;
		}
#ifndef X5_CHIP
		/* state ok */
		if (hdev->ex_hdev != NULL) {
			ex_hdev = (struct mipi_hdev_s *)(hdev->ex_hdev);
			(void)mipi_host_stop(ex_hdev);
		}
#endif
		ret = mipi_host_stop(hdev);
		if (ret != 0) {
			mipi_err(dev, "stop error: %d\n", ret);
			user->start_cnt = start_cnt_save;
			osal_mutex_unlock(&user->mutex);
			mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_4, (uint8_t)(host->state), __LINE__);
			return ret;
		}
		user->pre_state = MIPI_PRE_STATE_INITED;
		host->state = MIPI_STATE_STOP;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_snrclk_set_en(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	int32_t ret = 0;
	uint32_t enable;

	if (arg == 0U) {
		mipi_err(dev, "cmd snrclk_set_en, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_3, 0U, __LINE__);
		return -EINVAL;
	}
	mipi_info(dev, "snrclk set en cmd\n");
	if (mipi_get_app(enable, (uint32_t *)arg) != 0U) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
		mipi_err(dev, "get data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_1, 0U, __LINE__);
		return -EFAULT;
	}
	if (mipi_host_snrclk_set_en(hdev, (int32_t)enable) != 0) {
		return -EACCES;
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_snrclk_set_freq(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	int32_t ret = 0;
	uint32_t freq;

	if (arg == 0U) {
		mipi_err(dev, "cmd snrclk_set_freq, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_4, 0U, __LINE__);
		return -EINVAL;
	}
	mipi_info(dev, "snrclk set freq cmd\n");
	if (mipi_get_app(freq, (uint32_t *)arg) != 0U) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
		mipi_err(dev, "get data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_2, 0U, __LINE__);
		return -EFAULT;
	}
	if (mipi_host_snrclk_set_freq(hdev, freq) != 0) {
		return -EACCES;
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_pre_init_request(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	uint32_t timeout_ms = 0U, wait = 0U;

	if (arg != 0U) {
		if (mipi_get_app(timeout_ms, (uint32_t *)arg) != 0) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
			mipi_err(dev, "get data from user failed\n");
			mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_3, 0U, __LINE__);
			return -EFAULT;
		}
	}

	osal_mutex_lock(&user->mutex);
	if (user->init_cnt == 0U) {
		if (user->pre_state == (uint32_t)MIPI_PRE_STATE_DEFAULT) {
			mipi_info(dev, "pre_init_request cmd: initing\n");
			user->pre_state = (uint32_t)MIPI_PRE_STATE_INITING;
		} else if (user->pre_state == (uint32_t)MIPI_PRE_STATE_INITING) {
			wait = 1;
		} else {
			mipi_info(dev, "pre_init_request cmd: preinited drop\n");
			/* do not need report */
			ret = -EACCES;
		}
	} else {
		mipi_info(dev, "pre_init_request cmd: inited drop\n");
		/* do not need report */
		ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);
	if (wait != 0U) {
		user->pre_done = (bool)false;
		if (timeout_ms != 0U) {
			osal_wait_event_interruptible_timeout(user->pre_wq, /* PRQA S 2996 */ /* wait_event_interruptible_timeout macro */
				user->pre_done, timeout_ms);
		} else {
			osal_wait_event_interruptible(user->pre_wq, /* PRQA S 4460 */ /* wait_event_interruptible macro */
									 user->pre_done);
		}

		osal_mutex_lock(&user->mutex);
		if ((user->init_cnt == 0U) &&
			(user->pre_state == (uint32_t)MIPI_PRE_STATE_DEFAULT)) {
			mipi_info(dev, "pre_init_request cmd: wait & initing\n");
			user->pre_state = (uint32_t)MIPI_PRE_STATE_INITING;
		} else {
			mipi_info(dev, "pre_init_request cmd: wait & drop\n");
			/* do not need report */
			ret = -EACCES;
		}
		osal_mutex_unlock(&user->mutex);
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_pre_start_request(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	uint32_t timeout_ms = 0U, wait = 0U;

	if (arg != 0U) {
		if (mipi_get_app(timeout_ms, (uint32_t *)arg) != 0U) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
			mipi_err(dev, "get data from user failed\n");
			mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_4, 0U, __LINE__);
			return -EFAULT;
		}
	}

	osal_mutex_lock(&user->mutex);
	if (user->start_cnt == 0U) {
		if (user->pre_state == (uint32_t)MIPI_PRE_STATE_INITED) {
			mipi_info(dev, "pre_start_request cmd: starting\n");
			user->pre_state = (uint32_t)MIPI_PRE_STATE_STARTING;
		} else if (user->pre_state == (uint32_t)MIPI_PRE_STATE_STARTING) {
			wait = 1;
		} else {
			mipi_info(dev, "pre_start_request cmd: prestarted drop\n");
			/* do not need report */
			ret = -EACCES;
		}
	} else {
		mipi_info(dev, "pre_start_request cmd: started drop\n");
		/* do not need report */
		ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);
	if (wait != 0U) {
		user->pre_done = (bool)false;
		if (timeout_ms != 0U) {
			osal_wait_event_interruptible_timeout(user->pre_wq, /* PRQA S 2996 */ /* wait_event_interruptible_timeout macro */
				user->pre_done, timeout_ms);
		} else {
			osal_wait_event_interruptible(user->pre_wq, /* PRQA S 4460 */ /* wait_event_interruptible macro */
									 user->pre_done);
		}

		osal_mutex_lock(&user->mutex);
		if ((user->start_cnt == 0U) &&
			(user->pre_state == (uint32_t)MIPI_PRE_STATE_INITED)) {
			mipi_info(dev, "pre_start_request cmd: wait & starting\n");
			user->pre_state = (uint32_t)MIPI_PRE_STATE_STARTING;
		} else {
			mipi_info(dev, "pre_start_request cmd: wait & drop\n");
			/* do not need report */
			ret = -EACCES;
		}
		osal_mutex_unlock(&user->mutex);
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_pre_init_result(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	uint32_t result = 0U, wake = 0U;

	if (arg == 0U) {
		mipi_err(dev, "cmd pre_init_result, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_5, 0U, __LINE__);
		return -EINVAL;
	}
	if (mipi_get_app(result, (uint32_t *)arg) != 0U) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
		mipi_err(dev, "get data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_5, 0U, __LINE__);
		return -EFAULT;
	}

	osal_mutex_lock(&user->mutex);
	if ((user->init_cnt == 0U) &&
		(user->pre_state == (uint32_t)MIPI_PRE_STATE_INITING)) {
		if (result != 0U) {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_DEFAULT;
		} else {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_INITED;
		}
		wake = 1U;
	}
	osal_mutex_unlock(&user->mutex);
	if (wake != 0U) {
		mipi_info(dev, "pre_init_result cmd: %s wake\n",
				(result != 0U) ? "falied" : "done");
		user->pre_done = (bool)true;
		osal_wake_up(&user->pre_wq);
	} else {
		mipi_info(dev, "pre_init_result cmd: %s drop\n",
				(result != 0U) ? "falied" : "done");
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_pre_start_result(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	uint32_t result = 0U, wake = 0U;

	if (arg == 0U) {
		mipi_err(dev, "cmd pre_start_result, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_6, 0U, __LINE__);
		return -EINVAL;
	}
	if (mipi_get_app(result, (uint32_t *)arg) != 0U) { /* PRQA S 0556,2753,1823 */ /* get_user macro */
		mipi_err(dev, "get data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_6, 0U, __LINE__);
		return -EFAULT;
	}

	osal_mutex_lock(&user->mutex);
	if ((user->start_cnt == 0U) &&
		(user->pre_state == (uint32_t)MIPI_PRE_STATE_STARTING)) {
		if (result != 0U) {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_INITED;
		} else {
			user->pre_state = (uint32_t)MIPI_PRE_STATE_STARTED;
		}
		wake = 1U;
	}
	osal_mutex_unlock(&user->mutex);
	if (wake != 0U) {
		mipi_info(dev, "pre_start_result cmd: %s wake\n",
				(result != 0U) ? "falied" : "done");
		user->pre_done = (bool)true;
		osal_wake_up(&user->pre_wq);
	} else {
		mipi_info(dev, "pre_start_result cmd: %s drop\n",
				(result != 0U) ? "falied" : "done");
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_ipi_reset(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	mipi_host_ipi_reset_t ipi_reset;

	if (arg == 0U) {
		mipi_err(dev, "cmd ipi_reset, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_7, 0U, __LINE__);
		return -EINVAL;
	}

	osal_mutex_lock(&user->mutex);
	mipi_info(dev, "ipi reset cmd\n");
	if (mipi_copy_from_app((void *)&ipi_reset,
					   (void __user *)arg, sizeof(mipi_host_ipi_reset_t)) != 0U) {
		mipi_err(dev, "ipi reset erorr, %px from user error\n", (void __user *)arg);
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_7, 0U, __LINE__);
		return -EINVAL;
	}
	if (user->init_cnt == 0U) {
		mipi_err(dev, "state error: not inited\n");
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_5, (uint8_t)(hdev->host.state), __LINE__);
		return -EACCES;
	}
	if (mipi_host_ipi_reset(hdev, &ipi_reset) != 0) {
		ret = -EFAULT;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_ipi_get_info(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	mipi_host_ipi_info_t ipi_info;

	if (arg == 0U) {
		mipi_err(dev, "cmd ipi_get_info, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_8, 0U, __LINE__);
		return -EINVAL;
	}

	osal_mutex_lock(&user->mutex);
	mipi_info(dev, "ipi get info cmd\n");
	if (mipi_copy_from_app((void *)&ipi_info,
					   (void __user *)arg, sizeof(mipi_host_ipi_info_t)) != 0U) {
		mipi_err(dev, "ipi get erorr, %px from user error\n", (void __user *)arg);
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_8, 0U, __LINE__);
		return -EINVAL;
	}
	if (user->init_cnt == 0U) {
		mipi_err(dev, "state error: not inited\n");
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_6, (uint8_t)(hdev->host.state), __LINE__);
		return -EACCES;
	}
	if (mipi_host_ipi_get_info(hdev, &ipi_info) != 0) {
		ret = -EFAULT;
	} else if (mipi_copy_to_app((void __user *)arg,
							(void *)&ipi_info, sizeof(mipi_host_ipi_info_t)) != 0U) {
		mipi_err(dev, "ipi get erorr, %px to user error\n", (void __user *)arg);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_9, 1U, __LINE__);
		ret = -EINVAL;
	} else {
		/* get info done */
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_ipi_set_info(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_user_s *user = &hdev->user;
	int32_t ret = 0;
	mipi_host_ipi_info_t ipi_info;

	if (arg == 0U) {
		mipi_err(dev, "cmd ipi_set_info, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_9, 0U, __LINE__);
		return -EINVAL;
	}

	osal_mutex_lock(&user->mutex);
	mipi_info(dev, "ipi set info cmd\n");
	if (mipi_copy_from_app((void *)&ipi_info,
					   (void __user *)arg, sizeof(mipi_host_ipi_info_t)) != 0U) {
		mipi_err(dev, "ipi set erorr, %px from user error\n", (void __user *)arg);
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_10, 0U, __LINE__);
		return -EINVAL;
	}
	if (user->init_cnt == 0U) {
		mipi_err(dev, "state error: not inited\n");
		osal_mutex_unlock(&user->mutex);
		mipi_host_error_report(hdev, ESW_MipiHostStatusErr, SUB_ID_7, (uint8_t)(hdev->host.state), __LINE__);
		return -EACCES;
	}
	if (mipi_host_ipi_set_info(hdev, &ipi_info) != 0) {
		ret = -EFAULT;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static int32_t hobot_mipi_host_ioc_get_param(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	int32_t ret = 0;

	if (arg == 0U) {
		mipi_err(dev, "cmd ipi_get_info, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_10, 0U, __LINE__);
		return -EINVAL;
	}
	mipi_info(dev, "get param cmd\n");
	if (mipi_copy_to_app((void __user *)arg,
				(void *)&hdev->host.param, sizeof(mipi_host_param_t)) != 0U) {
		mipi_err(dev, "param get erorr, %px to user error\n", (void __user *)arg);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_11, 1U, __LINE__);
		ret = -EINVAL;
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_set_param(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	mipi_host_param_t param;
	int32_t ret = 0;

	if (arg == 0U) {
		mipi_err(dev, "cmd ipi_set_info, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_11, 0U, __LINE__);
		return -EINVAL;
	}
	mipi_info(dev, "set param cmd\n");
	if (mipi_copy_from_app((void *)&param,
				(void __user *)arg, sizeof(mipi_host_param_t)) != 0U) {
		mipi_err(dev, "param set erorr, %px from user error\n", (void __user *)arg);
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_12, 0U, __LINE__);
		return -EINVAL;
	}
	memcpy(&hdev->host.param, &param, sizeof(mipi_host_param_t));

	return ret;
}

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
static int32_t hobot_mipi_host_ioc_read(const struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	const struct mipi_host_s *host = &hdev->host;
	int32_t ret = 0;
	reg_t reg;

	if (arg == 0U) {
		mipi_err(dev, "cmd read, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_12, 0U, __LINE__);
		return -EINVAL;
	}
	if (mipi_copy_from_app((void *)&reg, (void __user *)arg, sizeof(reg)) != 0U) {
		mipi_err(dev, "reg read error, copy data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_13, 0U, __LINE__);
		return -EFAULT;
	}
	reg.value = mhost_getreg(host, reg.offset);
	if (mipi_copy_to_app((void __user *)arg, (void *)&reg, sizeof(reg)) != 0U) {
		mipi_err(dev, "reg read error, copy data to user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_14, 1U, __LINE__);
		return -EFAULT;
	}

	return ret;
}

static int32_t hobot_mipi_host_ioc_write(struct mipi_hdev_s *hdev, mipi_ioc_arg_t arg)
{
	struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	int32_t ret = 0;
	reg_t reg;
	uint32_t regv;

	if (arg == 0U) {
		mipi_err(dev, "cmd write, arg NULL\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_13, 0U, __LINE__);
		return -EINVAL;
	}
	if (mipi_copy_from_app((void *)&reg, (void __user *)arg, sizeof(reg)) != 0U) {
		mipi_err(dev, "reg write error, copy data from user failed\n");
		mipi_host_error_report(hdev, ESW_MipiHostIocUserErr, SUB_ID_15, 0U, __LINE__);
		return -EFAULT;
	}
	mhost_putreg(host, reg.offset, reg.value);
	regv = mhost_getreg(host, reg.offset);
	if (regv != reg.value) {
		mipi_err(dev, "reg write error, write 0x%x got 0x%x\n", reg.value, regv);
		return -EFAULT;
	}

	return ret;
}
#endif

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device ioctl operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 * @param[in] cmd: ioctl cmd
 * @param[in] arg: ioctl arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
mipi_ioc_ret_t hobot_mipi_host_ioctl_do(struct mipi_hdev_s *hdev, uint32_t cmd, mipi_ioc_arg_t arg)
{
	int32_t ret;

	if ((char)(_IOC_TYPE(cmd)) != MIPIHOSTIOC_MAGIC) {
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_0, (uint8_t)(_IOC_TYPE(cmd)), __LINE__);
		return -ENOTTY;
	}

	switch (cmd) {
	case MIPIHOSTIOC_INIT: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_init(hdev, arg);
		break;
	case MIPIHOSTIOC_DEINIT: /* PRQA S 4513 */ /* _IO macro */
		ret = hobot_mipi_host_ioc_deinit(hdev);
		break;
	case MIPIHOSTIOC_START: /* PRQA S 4513 */ /* _IO macro */
		ret = hobot_mipi_host_ioc_start(hdev);
		break;
	case MIPIHOSTIOC_STOP: /* PRQA S 4513 */ /* _IO macro */
		ret = hobot_mipi_host_ioc_stop(hdev);
		break;
	case MIPIHOSTIOC_SNRCLK_SET_EN: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_snrclk_set_en(hdev, arg);
		break;
	case MIPIHOSTIOC_SNRCLK_SET_FREQ: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_snrclk_set_freq(hdev, arg);
		break;
	case MIPIHOSTIOC_PRE_INIT_REQUEST: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_pre_init_request(hdev, arg);
		break;
	case MIPIHOSTIOC_PRE_START_REQUEST: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_pre_start_request(hdev, arg);
		break;
	case MIPIHOSTIOC_PRE_INIT_RESULT: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_pre_init_result(hdev, arg);
		break;
	case MIPIHOSTIOC_PRE_START_RESULT: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_pre_start_result(hdev, arg);
		break;
	case MIPIHOSTIOC_IPI_RESET: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_ipi_reset(hdev, arg);
		break;
	case MIPIHOSTIOC_IPI_GET_INFO: /* PRQA S 0591,4513 */ /* _IOR macro */
		ret = hobot_mipi_host_ioc_ipi_get_info(hdev, arg);
		break;
	case MIPIHOSTIOC_IPI_SET_INFO: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_ipi_set_info(hdev, arg);
		break;
	case MIPIHOSTIOC_GET_PARAM: /* PRQA S 0591,4513 */ /* _IOR macro */
		ret = hobot_mipi_host_ioc_get_param(hdev, arg);
		break;
	case MIPIHOSTIOC_SET_PARAM: /* PRQA S 0591,4513 */ /* _IOR macro */
		ret = hobot_mipi_host_ioc_set_param(hdev, arg);
		break;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	case MIPIHOSTIOC_READ: /* PRQA S 0591,4513 */ /* _IOR macro */
		ret = hobot_mipi_host_ioc_read(hdev, arg);
		break;
	case MIPIHOSTIOC_WRITE: /* PRQA S 0591,4513 */ /* _IOW macro */
		ret = hobot_mipi_host_ioc_write(hdev, arg);
		break;
#endif
	default:
		ret = -ERANGE;
		mipi_host_error_report(hdev, ESW_MipiHostIocCheckErr, SUB_ID_1, (uint8_t)(_IOC_NR(cmd)), __LINE__);
		break;
	}
	return ret;
}

/* get index of mipi host devices' param/ */
static int32_t mipi_host_param_idx(const char *name)
{
	int32_t i;

	for (i = 0; i < MIPI_HOST_PARAMS_NUM; i++) {
		if (strcmp(g_mh_param_names[i], name) == 0) {
			return i;
		}
	}

	return -1;
}

/* sprintf show for mipi host devices' param/ */
static int32_t mipi_host_param_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	uint32_t *param;
	char *s = buf;
	int32_t idx, l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	param = (uint32_t *)((void *)(&hdev->host.param));

	idx = mipi_host_param_idx(name);
	if ((idx >= 0) && (idx < MIPI_HOST_PARAMS_NUM)) {
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
		/* update the real value for snrclk_freq */
		if((strcmp(name, "snrclk_freq") == 0) &&
			(hdev->host.snrclk.index >= 0)) {
			hdev->host.param.snrclk_freq =
				(uint32_t)(mipi_host_get_clk(hdev, hdev->host.socclk.snrclk));
		}
#endif
		l += snprintf(&s[l], (count - l), "%u\n", param[idx]);
	}

	return l;
}

/* string store for mipi host devices' param/ */
static int32_t mipi_host_param_store_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	uint32_t *param;
	int32_t ret, error = -EINVAL;
	int32_t idx;
	uint32_t val, pbak;

	if ((hdev == NULL) || (name == NULL) || (buf == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	param = (uint32_t *)((void *)(&hdev->host.param));

	idx = mipi_host_param_idx(name);
	if ((idx >= 0) && (idx < MIPI_HOST_PARAMS_NUM)) {
		ret = kstrtouint(buf, 0, &val);
		if (ret == 0) {
			pbak = param[idx];
			param[idx] = val;
			error = 0;
		}
	}

	if (error == 0) {
		/* update the snrclk_freq to hw */
		if(strcmp(name, "snrclk_en") == 0) {
			/* snrclk_en keep here, will be changed in mipi_host_snrclk_set_en() */
			param[idx] = pbak;
			error = mipi_host_snrclk_set_en(hdev, (int32_t)val);
		} else if (strcmp(name, "snrclk_freq") == 0) {
			error = mipi_host_snrclk_set_freq(hdev, val);
		} else {
			/* other params */
		}
	}

	return ((error != 0) ? error : count);
}

/* sprintf show for mipi dev devices' status/clock */
static int32_t mipi_host_status_clock_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_socclk_s *socclk;
	char *s = buf;
	int32_t l = 0, i;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	socclk = &hdev->host.socclk;

	l += snprintf(&s[l], (count - l), "%-15s: %s(%llu)\n", "cfg_clock",
		(socclk->cfgclk) ? (socclk->cfgclk) : "null",
		mipi_host_get_clk(hdev, socclk->cfgclk));
	l += snprintf(&s[l], (count - l), "%-15s: %s(%llu)\n", "ref_clock",
		(socclk->refclk) ? (socclk->refclk) : "null",
		mipi_host_get_clk(hdev, socclk->refclk));
	l += snprintf(&s[l], (count - l), "%-15s: %s(%llu)\n", "snr_clock",
		(socclk->snrclk) ? (socclk->snrclk) : "null",
		mipi_host_get_clk(hdev, socclk->snrclk));
	for (i = 0; i < MIPI_HOST_HW_IPI_MAX; i++) {
		l += snprintf(&s[l], (count - l), "%-15s: %s(%llu)\n", "ipi_clock",
			(socclk->ipiclk[i]) ? (socclk->ipiclk[i]) : "null",
			mipi_host_get_clk(hdev, socclk->ipiclk[i]));
	}
	l += snprintf(&s[l], (count - l), "%-15s: %llu\n", "ipi_runclk", hdev->ipi_clock);

	return l;
}

/* sprintf show for mipi host devices' status/info */
static int32_t mipi_host_status_info_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	char *s = buf;
	int32_t l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;

	l += snprintf(&s[l], (count - l), "%-15s: %d\n", "port", hdev->port);
	l += snprintf(&s[l], (count - l), "%-15s: g%d:%d l%d/%d i%d%s\n", "hw_mode",
			mipi_host_port_group(hdev), mipi_host_port_index(hdev),
			mipi_host_port_lane(hdev, 0), mipi_host_port_lane(hdev, 1),
			mipi_host_port_ipi(hdev),
			(host->ap != 0) ? " ap" : "");
	l += snprintf(&s[l], (count - l), "%-15s: %s\n", "mode", (hdev->is_ex != 0) ? "group_ext" :
			((hdev->ex_hdev != NULL) ? "group_mst" : "alone"));
	l += snprintf(&s[l], (count - l), "%-15s: %d(%dlane)\n", "lane_mode", hdev->lane_mode,
			mipi_host_port_lane(hdev, hdev->lane_mode));
	l += snprintf(&s[l], (count - l), "%-15s: 0x%08x +0x%x\n", "reg",
			host->reg.base, host->reg.size);
	l += snprintf(&s[l], (count - l), "%-15s: %p\n", "iomem", host->iomem);
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
	l += snprintf(&s[l], (count - l), "%-15s: %s\n", "timer",
			(hdev->irq_timer_en != 0U) ? "enable" : "disable");
#else
	l += snprintf(&s[l], (count - l), "%-15s: %d\n", "irq", host->irq);
#endif
	l += snprintf(&s[l], (count - l), "%-15s: %d(%s)\n", "state", (int32_t)host->state,
			g_mh_state[host->state]);
#ifdef CONFIG_HOBOT_FUSA_DIAG
	l += mipi_csi_stl_info(&host->stl, &s[l]);
#endif
#ifdef MIPI_HOST_ERR_DIAG_EVENT
	{
		int32_t i;
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "event_cnt(SW)", ESW_MipiHostErrEventMax);
		for (i = 0; i < ESW_MipiHostErrEventMax; i++) {
			l += snprintf(&s[l], (count - l), "%-15s: [%d]%-2d +   %s\n",
					"event(SW)", mipi_host_err_events[i].prio,
					mipi_host_err_events[i].id,
					mipi_host_err_events[i].name);
		}
	}
#endif


	return l;
}

/* sprintf show for mipi host devices' status/cfg */
static int32_t mipi_host_status_cfg_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	mipi_host_cfg_t *cfg;
	char *s = buf;
	const char * mclk_desp;
	int32_t i, l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;
	cfg = &host->cfg;

	if (host->state >= MIPI_STATE_INIT) {
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "lane", cfg->lane);
		l += snprintf(&s[l], (count - l), "%-15s: 0x%02x\n", "datatype", cfg->datatype);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "fps", cfg->fps);
		if (cfg->mclk >= (uint16_t)(MIPI_HOST_SNRCLK_FREQ_MIN / MIPI_HOST_SNRCLK_FREQ_BASE)) {
			l += snprintf(&s[l], (count - l), "%-15s: %d -> %luHz\n", "mclk",
					cfg->mclk, cfg->mclk * MIPI_HOST_SNRCLK_FREQ_BASE);
		} else {
			if (cfg->mclk > (uint16_t)MIPI_HOST_SNRCLK_IGNORE) {
				mclk_desp = "error";
			} else if (cfg->mclk > (uint16_t)MIPI_HOST_SNRCLK_ENABLE) {
				mclk_desp = "ignore";
			} else if (cfg->mclk == (uint16_t)MIPI_HOST_SNRCLK_ENABLE) {
				mclk_desp = "enable";
			} else {
				mclk_desp = "disable";
			}
			l += snprintf(&s[l], (count - l), "%-15s: %d -> %s\n", "mclk", cfg->mclk, mclk_desp);
		}
		l += snprintf(&s[l], (count - l), "%-15s: %d Mbps -> %dMbps/lane\n", "mipiclk",
				cfg->mipiclk, cfg->mipiclk / cfg->lane);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "width", cfg->width);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "height", cfg->height);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "linelenth", cfg->linelenth);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "framelenth", cfg->framelenth);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "settle", cfg->settle);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "hsaTime", cfg->hsaTime);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "hbpTime", cfg->hbpTime);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "hsdTime", cfg->hsdTime);
		l += snprintf(&s[l], (count - l), "%-15s: %d\n", "channel_num", cfg->channel_num);
		for (i = 0; i < MIPIHOST_CHANNEL_NUM; i++) {
			l += snprintf(&s[l], (count - l), "channel_sel[%d] : %d\n",
					i, cfg->channel_sel[i]);
		}
	} else {
		l += snprintf(&s[l], (count - l), "not inited\n" );
	}

	return l;
}

/* mipi host dump regs */
static const struct mipi_dump_reg host_base_dump_regs[] = {
	{ "VERSION", REG_MIPI_HOST_VERSION },
	{ "N_LANES", REG_MIPI_HOST_N_LANES },
	{ "CSI2_RESETN", REG_MIPI_HOST_CSI2_RESETN },
	{ "PHY_SHUTDOWNZ", REG_MIPI_HOST_PHY_SHUTDOWNZ },
	{ "DPHY_RSTZ", REG_MIPI_HOST_DPHY_RSTZ },
	{ "PHY_RX", REG_MIPI_HOST_PHY_RX },
	{ "PHY_STOPSTATE", REG_MIPI_HOST_PHY_STOPSTATE },
	{ "PHY_CAL", REG_MIPI_HOST_PHY_CAL },
	{ "IPI_SOFTRSTN", REG_MIPI_HOST_IPI_SOFTRSTN},
	{ "DATA_IDS_1", REG_MIPI_HOST_DATA_IDS_1},
	{ "DATA_IDS_2", REG_MIPI_HOST_DATA_IDS_2},
	{ "PHY_CFG", REG_MIPI_HOST_PHY_CFG },
	{ "PHY_MODE", REG_MIPI_HOST_PHY_MODE },
	{ "DATA_IDS_VC1", REG_MIPI_HOST_DATA_IDS_VC1},
	{ "DATA_IDS_VC2", REG_MIPI_HOST_DATA_IDS_VC2},
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_ppipg_dump_regs[] = {
	{ "PPI_PG_PATTERN_VRES", REG_MIPI_HOST_PPI_PG_PATTERN_VRES },
	{ "PPI_PG_PATTERN_HRES", REG_MIPI_HOST_PPI_PG_PATTERN_HRES },
	{ "PPI_PG_CONFIG", REG_MIPI_HOST_PPI_PG_CONFIG },
	{ "PPI_PG_ENABLE", REG_MIPI_HOST_PPI_PG_ENABLE },
	{ "PPI_PG_STATUS", REG_MIPI_HOST_PPI_PG_STATUS },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_intap_dump_regs[] = {
	{ "INT_ST_AP_MAIN", REG_MIPI_HOST_INT_ST_AP_MAIN },
	{ "INT_ST_FAP_PHY_FATAL", REG_MIPI_HOST_INT_ST_FAP_PHY_FATAL },
	{ "INT_ST_FAP_PKT_FATAL", REG_MIPI_HOST_INT_ST_FAP_PKT_FATAL },
	{ "INT_ST_FAP_PHY", REG_MIPI_HOST_INT_ST_FAP_PHY },
	{ "INT_ST_FAP_LINE", REG_MIPI_HOST_INT_ST_FAP_LINE },
	{ "INT_ST_FAP_IPI", REG_MIPI_HOST_INT_ST_FAP_IPI },
	{ "INT_ST_FAP_IPI2", REG_MIPI_HOST_INT_ST_FAP_IPI2 },
	{ "INT_ST_FAP_IPI3", REG_MIPI_HOST_INT_ST_FAP_IPI3 },
	{ "INT_ST_FAP_IPI4", REG_MIPI_HOST_INT_ST_FAP_IPI4 },
	{ "INT_ST_FAP_BNDRY_FRAME_FATAL", REG_MIPI_HOST_INT_ST_FAP_BNDRY_FRAME_FATAL },
	{ "INT_ST_FAP_SEQ_FRAME_FATAL", REG_MIPI_HOST_INT_ST_FAP_SEQ_FRAME_FATAL },
	{ "INT_ST_FAP_CRC_FRAME_FATAL", REG_MIPI_HOST_INT_ST_FAP_CRC_FRAME_FATAL },
	{ "INT_ST_FAP_PLD_CRC_FATAL", REG_MIPI_HOST_INT_ST_FAP_PLD_CRC_FATAL },
	{ "INT_ST_FAP_DATA_ID", REG_MIPI_HOST_INT_ST_FAP_DATA_ID },
	{ "INT_ST_FAP_ECC_CORRECT", REG_MIPI_HOST_INT_ST_FAP_ECC_CORRECT },
	{ "INT_ST_AP_GENERIC", REG_MIPI_HOST_INT_ST_AP_GENERIC },
	{ "INT_ST_AP_IPI", REG_MIPI_HOST_INT_ST_AP_IPI },
	{ "INT_ST_AP_IPI2", REG_MIPI_HOST_INT_ST_AP_IPI2 },
	{ "INT_ST_AP_IPI3", REG_MIPI_HOST_INT_ST_AP_IPI3 },
	{ "INT_ST_AP_IPI4", REG_MIPI_HOST_INT_ST_AP_IPI4 },
	{ "INT_MSK_FAP_PHY_FATAL", REG_MIPI_HOST_INT_MSK_FAP_PHY_FATAL },
	{ "INT_MSK_FAP_PKT_FATAL", REG_MIPI_HOST_INT_MSK_FAP_PKT_FATAL },
	{ "INT_MSK_FAP_PHY", REG_MIPI_HOST_INT_MSK_FAP_PHY },
	{ "INT_MSK_FAP_LINE", REG_MIPI_HOST_INT_MSK_FAP_LINE },
	{ "INT_MSK_FAP_IPI", REG_MIPI_HOST_INT_MSK_FAP_IPI },
	{ "INT_MSK_FAP_IPI2", REG_MIPI_HOST_INT_MSK_FAP_IPI2 },
	{ "INT_MSK_FAP_IPI3", REG_MIPI_HOST_INT_MSK_FAP_IPI3 },
	{ "INT_MSK_FAP_IPI4", REG_MIPI_HOST_INT_MSK_FAP_IPI4 },
	{ "INT_MSK_FAP_BNDRY_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_FAP_BNDRY_FRAME_FATAL },
	{ "INT_MSK_FAP_SEQ_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_FAP_SEQ_FRAME_FATAL },
	{ "INT_MSK_FAP_CRC_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_FAP_CRC_FRAME_FATAL },
	{ "INT_MSK_FAP_PLD_CRC_FATAL", REG_MIPI_HOST_INT_MSK_FAP_PLD_CRC_FATAL },
	{ "INT_MSK_FAP_DATA_ID", REG_MIPI_HOST_INT_MSK_FAP_DATA_ID },
	{ "INT_MSK_FAP_ECC_CORRECT", REG_MIPI_HOST_INT_MSK_FAP_ECC_CORRECT },
	{ "INT_MSK_AP_GENERIC", REG_MIPI_HOST_INT_MSK_AP_GENERIC },
	{ "INT_MSK_AP_IPI", REG_MIPI_HOST_INT_MSK_AP_IPI },
	{ "INT_MSK_AP_IPI2", REG_MIPI_HOST_INT_MSK_AP_IPI2 },
	{ "INT_MSK_AP_IPI3", REG_MIPI_HOST_INT_MSK_AP_IPI3 },
	{ "INT_MSK_AP_IPI4", REG_MIPI_HOST_INT_MSK_AP_IPI4 },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_int1p4_dump_regs[] = {
	{ "INT_ST_MAIN", REG_MIPI_HOST_INT_ST_MAIN },
	{ "INT_ST_PHY_FATAL", REG_MIPI_HOST_INT_ST_PHY_FATAL },
	{ "INT_ST_PKT_FATAL", REG_MIPI_HOST_INT_ST_PKT_FATAL },
	{ "INT_ST_PHY", REG_MIPI_HOST_INT_ST_PHY },
	{ "INT_ST_LINE", REG_MIPI_HOST_INT_ST_LINE },
	{ "INT_ST_IPI", REG_MIPI_HOST_INT_ST_IPI },
	{ "INT_ST_IPI2", REG_MIPI_HOST_INT_ST_IPI2 },
	{ "INT_ST_IPI3", REG_MIPI_HOST_INT_ST_IPI3 },
	{ "INT_ST_IPI4", REG_MIPI_HOST_INT_ST_IPI4 },
	{ "INT_ST_BNDRY_FRAME_FATAL", REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL },
	{ "INT_ST_SEQ_FRAME_FATAL", REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL },
	{ "INT_ST_CRC_FRAME_FATAL", REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL },
	{ "INT_ST_PLD_CRC_FATAL", REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL },
	{ "INT_ST_DATA_ID", REG_MIPI_HOST_INT_ST_DATA_ID },
	{ "INT_ST_ECC_CORRECT", REG_MIPI_HOST_INT_ST_ECC_CORRECT },
	{ "INT_MSK_PHY_FATAL", REG_MIPI_HOST_INT_MSK_PHY_FATAL },
	{ "INT_MSK_PKT_FATAL", REG_MIPI_HOST_INT_MSK_PKT_FATAL },
	{ "INT_MSK_PHY", REG_MIPI_HOST_INT_MSK_PHY },
	{ "INT_MSK_LINE", REG_MIPI_HOST_INT_MSK_LINE },
	{ "INT_MSK_IPI", REG_MIPI_HOST_INT_MSK_IPI },
	{ "INT_MSK_IPI2", REG_MIPI_HOST_INT_MSK_IPI2 },
	{ "INT_MSK_IPI3", REG_MIPI_HOST_INT_MSK_IPI3 },
	{ "INT_MSK_IPI4", REG_MIPI_HOST_INT_MSK_IPI4 },
	{ "INT_MSK_BNDRY_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL },
	{ "INT_MSK_SEQ_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL },
	{ "INT_MSK_CRC_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL },
	{ "INT_MSK_PLD_CRC_FATAL", REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL },
	{ "INT_MSK_DATA_ID", REG_MIPI_HOST_INT_MSK_DATA_ID },
	{ "INT_MSK_ECC_CORRECT", REG_MIPI_HOST_INT_MSK_ECC_CORRECT },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_int_dump_regs[] = {
	{ "INT_ST_MAIN", REG_MIPI_HOST_INT_ST_MAIN },
	{ "INT_ST_PHY_FATAL", REG_MIPI_HOST_INT_ST_PHY_FATAL },
	{ "INT_ST_PKT_FATAL", REG_MIPI_HOST_INT_ST_PKT_FATAL },
	{ "INT_ST_PHY", REG_MIPI_HOST_INT_ST_PHY },
	{ "INT_ST_LINE", REG_MIPI_HOST_INT_ST_LINE },
	{ "INT_ST_IPI", REG_MIPI_HOST_INT_ST_IPI },
	{ "INT_ST_IPI2", REG_MIPI_HOST_INT_ST_IPI2 },
	{ "INT_ST_IPI3", REG_MIPI_HOST_INT_ST_IPI3 },
	{ "INT_ST_IPI4", REG_MIPI_HOST_INT_ST_IPI4 },
	{ "INT_ST_FRAME_FATAL", REG_MIPI_HOST_INT_ST_FRAME_FATAL },
	{ "INT_ST_PKT", REG_MIPI_HOST_INT_ST_PKT },
	{ "INT_MSK_PHY_FATAL", REG_MIPI_HOST_INT_MSK_PHY_FATAL },
	{ "INT_MSK_PKT_FATAL", REG_MIPI_HOST_INT_MSK_PKT_FATAL },
	{ "INT_MSK_PHY", REG_MIPI_HOST_INT_MSK_PHY },
	{ "INT_MSK_LINE", REG_MIPI_HOST_INT_MSK_LINE },
	{ "INT_MSK_IPI", REG_MIPI_HOST_INT_MSK_IPI },
	{ "INT_MSK_IPI2", REG_MIPI_HOST_INT_MSK_IPI2 },
	{ "INT_MSK_IPI3", REG_MIPI_HOST_INT_MSK_IPI3 },
	{ "INT_MSK_IPI4", REG_MIPI_HOST_INT_MSK_IPI4 },
	{ "INT_MSK_FRAME_FATAL", REG_MIPI_HOST_INT_MSK_FRAME_FATAL },
	{ "INT_MSK_PKT", REG_MIPI_HOST_INT_MSK_PKT },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_ipi1_dump_regs[] = {
	{ "IPI_MODE", REG_MIPI_HOST_IPI_MODE },
	{ "IPI_VCID", REG_MIPI_HOST_IPI_VCID },
	{ "IPI_DATA_TYPE", REG_MIPI_HOST_IPI_DATA_TYPE },
	{ "IPI_MEM_FLUSH", REG_MIPI_HOST_IPI_MEM_FLUSH },
	{ "IPI_HSA_TIME", REG_MIPI_HOST_IPI_HSA_TIME },
	{ "IPI_HBP_TIME", REG_MIPI_HOST_IPI_HBP_TIME },
	{ "IPI_HSD_TIME", REG_MIPI_HOST_IPI_HSD_TIME },
	{ "IPI_HLINE_TIME", REG_MIPI_HOST_IPI_HLINE_TIME },
	{ "IPI_ADV_FEATURES", REG_MIPI_HOST_IPI_ADV_FEATURES },
	{ "IPI_VSA_LINES", REG_MIPI_HOST_IPI_VSA_LINES },
	{ "IPI_VBP_LINES", REG_MIPI_HOST_IPI_VBP_LINES },
	{ "IPI_VFP_LINES", REG_MIPI_HOST_IPI_VFP_LINES },
	{ "IPI_VACTIVE_LINES", REG_MIPI_HOST_IPI_VACTIVE_LINES },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_ipi2_dump_regs[] = {
	{ "IPI2_MODE", REG_MIPI_HOST_IPI2_MODE },
	{ "IPI2_VCID", REG_MIPI_HOST_IPI2_VCID },
	{ "IPI2_DATA_TYPE", REG_MIPI_HOST_IPI2_DATA_TYPE },
	{ "IPI2_MEM_FLUSH", REG_MIPI_HOST_IPI2_MEM_FLUSH },
	{ "IPI2_HSA_TIME", REG_MIPI_HOST_IPI2_HSA_TIME },
	{ "IPI2_HBP_TIME", REG_MIPI_HOST_IPI2_HBP_TIME },
	{ "IPI2_HSD_TIME", REG_MIPI_HOST_IPI2_HSD_TIME },
	{ "IPI2_ADV_FEATURES", REG_MIPI_HOST_IPI2_ADV_FEATURES },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_ipi3_dump_regs[] = {
	{ "IPI3_MODE", REG_MIPI_HOST_IPI3_MODE },
	{ "IPI3_VCID", REG_MIPI_HOST_IPI3_VCID },
	{ "IPI3_DATA_TYPE", REG_MIPI_HOST_IPI3_DATA_TYPE },
	{ "IPI3_MEM_FLUSH", REG_MIPI_HOST_IPI3_MEM_FLUSH },
	{ "IPI3_HSA_TIME", REG_MIPI_HOST_IPI3_HSA_TIME },
	{ "IPI3_HBP_TIME", REG_MIPI_HOST_IPI3_HBP_TIME },
	{ "IPI3_HSD_TIME", REG_MIPI_HOST_IPI3_HSD_TIME },
	{ "IPI3_ADV_FEATURES", REG_MIPI_HOST_IPI3_ADV_FEATURES },
	{ NULL, 0 },
};
static const struct mipi_dump_reg host_ipi4_dump_regs[] = {
	{ "IPI4_MODE", REG_MIPI_HOST_IPI4_MODE },
	{ "IPI4_VCID", REG_MIPI_HOST_IPI4_VCID },
	{ "IPI4_DATA_TYPE", REG_MIPI_HOST_IPI4_DATA_TYPE },
	{ "IPI4_MEM_FLUSH", REG_MIPI_HOST_IPI4_MEM_FLUSH },
	{ "IPI4_HSA_TIME", REG_MIPI_HOST_IPI4_HSA_TIME },
	{ "IPI4_HBP_TIME", REG_MIPI_HOST_IPI4_HBP_TIME },
	{ "IPI4_HSD_TIME", REG_MIPI_HOST_IPI4_HSD_TIME },
	{ "IPI4_ADV_FEATURES", REG_MIPI_HOST_IPI4_ADV_FEATURES },
	{ NULL, 0 },
};

/* sprintf show for mipi host devices' status/regs */
static int32_t mipi_host_status_regs_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	void __iomem *iomem;
	mipi_host_cfg_t *cfg;
	char *s = buf;
	int32_t l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;
	iomem = host->iomem;
	cfg = &host->cfg;

	if (iomem == NULL) {
		l += snprintf(&s[l], (count - l), "not ioremap\n" );
		return l;
	}

	l += mipi_dumpregs(iomem, host_base_dump_regs, &s[l], (count - l));

	if ((cfg->ppi_pg != 0U) && (MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P5) != 0)) {
		l += mipi_dumpregs(iomem, host_ppipg_dump_regs, &s[l], (count - l));
	}
	if (host->ap != 0) {
		l += mipi_dumpregs(iomem, host_intap_dump_regs, &s[l], (count - l));
	} else {
		if (MIPI_VERSION_GE(iomem, MIPI_IP_VERSION_1P4) != 0) {
			l += mipi_dumpregs(iomem, host_int1p4_dump_regs, &s[l], (count - l));
		} else {
			l += mipi_dumpregs(iomem, host_int_dump_regs, &s[l], (count - l));
		}
#ifdef CONFIG_HOBOT_FUSA_DIAG
		l += mipi_dumpregs(iomem, host_intap_dump_regs, &s[l], (count - l));
#endif
	}
	if (cfg->channel_sel[MIPI_IPI1] < (uint16_t)MIPI_HOST_HW_VC_MAX) {
		l += mipi_dumpregs(iomem, host_ipi1_dump_regs, &s[l], (count - l));
	}
	if ((cfg->channel_num > (uint16_t)MIPI_IPI2) && (cfg->channel_sel[MIPI_IPI2] < (uint16_t)MIPI_HOST_HW_VC_MAX)) {
		l += mipi_dumpregs(iomem, host_ipi2_dump_regs, &s[l], (count - l));
	}
	if ((cfg->channel_num > (uint16_t)MIPI_IPI3) && (cfg->channel_sel[MIPI_IPI3] < (uint16_t)MIPI_HOST_HW_VC_MAX)) {
		l += mipi_dumpregs(iomem, host_ipi3_dump_regs, &s[l], (count - l));
	}
	if ((cfg->channel_num > (uint16_t)MIPI_IPI4) && (cfg->channel_sel[MIPI_IPI4] < (uint16_t)MIPI_HOST_HW_VC_MAX)) {
		l += mipi_dumpregs(iomem, host_ipi4_dump_regs, &s[l], (count - l));
	}

	return l;
}

/* sprintf show for mipi host devices' status/snrclk */
static int32_t mipi_host_status_snrclk_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	char *s = buf;
	int32_t l = 0;
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	struct mipi_host_s *host;
	struct mipi_host_snrclk_s *snrclk;
	struct mipi_host_param_s *param;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;
	snrclk = &host->snrclk;
	param = &host->param;

	if (hdev->host.socclk.snrclk != NULL) {
		l += snprintf(&s[l], (count - l), "%-15s: %s\n", "snrclk", hdev->host.socclk.snrclk);
	} else {
		l += snprintf(&s[l], (count - l), "%-15s: %s\n", "snrclk", "none");
	}
	if (snrclk->pinctrl != NULL) {
		l += snprintf(&s[l], (count - l), "%-15s: %s %s\n", "support",
				(snrclk->enable != NULL) ? "enable" : "",
				(snrclk->disable != NULL) ? "disable" : "");
	} else {
		l += snprintf(&s[l], (count - l), "%-15s: %s\n", "support", "none");
	}
	l += snprintf(&s[l], (count - l), "%-15s: %s\n", "state",
			(param->snrclk_en == MIPI_HOST_SNRCLK_NOUSED) ? "noused" :
			((param->snrclk_en == MIPI_HOST_SNRCLK_ENABLE) ? "enable" : "disable"));
	l += snprintf(&s[l], (count - l), "%-15s: %u\n", "freq", param->snrclk_freq);
#else
	l += snprintf(&s[l], (count - l), "%-15s: %s\n", "snrclk", "not support");
#endif

	return l;
}

/* sprintf show for mipi host devices' status/user */
static int32_t mipi_host_status_user_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_user_s *user;
	char *s = buf;
	int32_t l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	user = &hdev->user;

	l += snprintf(&s[l], (count - l), "%-15s: %u\n", "user", user->open_cnt);
	l += snprintf(&s[l], (count - l), "%-15s: %u\n", "init", user->init_cnt);
	l += snprintf(&s[l], (count - l), "%-15s: %u\n", "start", user->start_cnt);
	l += snprintf(&s[l], (count - l), "%-15s: %s\n", "pre_state", g_mh_pre_state[user->pre_state]);

	return l;
}

#if MIPI_HOST_INT_DBG
/* sprintf show for mipi host devices' status/icnt */
static int32_t mipi_host_status_icnt_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	char *s = buf;
	uint32_t i;
	int32_t l = 0;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;

	for (i = 0U; i < MIPI_HOST_ICNT_NUM; i++) {
		l += snprintf(&s[l], (count - l), "%-15s: %u\n", g_mh_icnt_names[i],
				((uint32_t *)(&host->icnt.st_main))[i]);
	}

	return l;
}
#endif

#if MIPI_HOST_INT_DBG && MIPI_HOST_SYSFS_FATAL_EN
/* get ireg of mipi host devices' ierr */
static const struct mipi_host_ireg_s* mipi_host_get_ireg(const struct mipi_host_ierr_s *ierr, const char *name)
{
	const struct mipi_host_ireg_s *ireg = NULL, *itmp;
	uint32_t i;

	for (i = 0U; i < ierr->num; i++) {
		itmp = &ierr->iregs[i];
		if (itmp->icnt_n < MIPI_HOST_ICNT_NUM) {
			if (strcmp(g_mh_icnt_names[itmp->icnt_n], name) == 0) {
				ireg = itmp;
				break;
			}
		}
	}

	return ireg;
}

/* sprintf show for mipi host devices' fatal/ */
static int32_t mipi_host_fatal_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	struct mipi_host_ierr_s *ierr;
	const struct mipi_host_ireg_s *ireg;
	void __iomem *iomem;
	char *s = buf;
	int32_t l = 0;
	uint32_t reg = 0U;

	if ((hdev == NULL) || (name == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;
#ifdef CONFIG_HOBOT_FUSA_DIAG
	if (host->param.fatal_ap != 0U)
		ierr = &host->ieap;
	else
#endif
		ierr = &host->ierr;
	iomem = hdev->host.iomem;
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_8, 0U, __LINE__);
		return -EACCES;
	}

	if (strcmp(g_mh_icnt_names[0], name) == 0) {
		reg = ierr->st_main;
	} else {
		ireg = mipi_host_get_ireg(ierr, name);
		if (ireg != NULL) {
			reg = ireg->reg_st;
		}
	}
	if (reg > 0U) {
		l += snprintf(&s[l], (count - l), "0x%08x\n", mhost_getreg(host, reg));
	}

	return l;
}

/* sprintf store for mipi host devices' fatal/ */
static int32_t mipi_host_fatal_store_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_host_s *host;
	struct mipi_host_ierr_s *ierr;
	const struct mipi_host_ireg_s *ireg;
	void __iomem *iomem;
	int32_t ret, error = -EINVAL;
	uint32_t val;
	uint32_t i;

	if ((hdev == NULL) || (name == NULL) || (buf == NULL)) {
		/* do not need report */
		return -EFAULT;
	}
	host = &hdev->host;
#ifdef CONFIG_HOBOT_FUSA_DIAG
	if (host->param.fatal_ap != 0U)
		ierr = &host->ieap;
	else
#endif
		ierr = &host->ierr;
	iomem = hdev->host.iomem;
	if (iomem == NULL) {
		mipi_host_error_report(hdev, ESW_MipiHostIomemErr, SUB_ID_9, 0U, __LINE__);
		return -EACCES;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret != 0) {
		/* do not need report */
		return error;
	}

	if (strcmp(g_mh_icnt_names[0], name) == 0) {
		for (i = 0U; i < ierr->num; i++) {
			ireg = &ierr->iregs[i];
			if ((val & ireg->st_mask) != 0U) {
				mhost_putreg(host, ireg->reg_force, ireg->err_mask);
				val &= ~ireg->st_mask;
				if (val == 0U) {
					break;
				}
			}
		}
		error = 0;
	} else {
		ireg = mipi_host_get_ireg(ierr, name);
		if (ireg != NULL) {
			mhost_putreg(host, ireg->reg_force, val);
			error = 0;
		}
	}

	return ((error != 0) ? error : count);
}
#endif

#if defined CONFIG_FAULT_INJECTION_ATTR || defined CONFIG_HOBOT_FUSA_DIAG
/* sprintf store for mipi host devices' fault_injection */
static int32_t mipi_host_fault_injection_show_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	char *s = buf;
	int32_t l = 0;

	if ((hdev == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}

#ifdef CONFIG_HOBOT_FUSA_DIAG
	l += mipi_csi_stl_inshow(&hdev->host.stl, &s[l], (count - l));
#else
	l = -EINVAL;
#endif

	return l;
}

/* sysfs store for mipi host driver' fault_injection */
static int32_t mipi_host_fault_injection_store_do(struct mipi_hdev_s *hdev,
		const char *name, char *buf, int32_t count)
{
	const char *s = buf;
	int32_t error;

	if ((hdev == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}

#ifdef CONFIG_HOBOT_FUSA_DIAG
	error = mipi_csi_stl_inject(&hdev->host.stl, s, (int32_t)count); /* qacfix: conversion */
#else
	error = -EINVAL;
#endif

	return ((error != 0) ? error : count);
}
#endif

typedef int32_t (* mipi_host_sys_do_func)(struct mipi_hdev_s *, const char *, char *, int32_t);
struct mipi_host_sys_s {
	int32_t type;
	mipi_host_sys_do_func sys_do[MIPI_SYS_INVALID];
};
static struct mipi_host_sys_s mipi_host_sys[MIPI_HOST_SYS_NUM] = {
	{ MIPI_HOST_SYS_PARAM, { mipi_host_param_show_do, mipi_host_param_store_do } },
	{ MIPI_HOST_SYS_STATUS_CLOCK, { mipi_host_status_clock_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_INFO, { mipi_host_status_info_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_CFG, { mipi_host_status_cfg_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_REGS, { mipi_host_status_regs_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_SNRCLK, { mipi_host_status_snrclk_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_USER, { mipi_host_status_user_show_do, NULL } },
	{ MIPI_HOST_SYS_STATUS_ICNT, { mipi_host_status_icnt_show_do, NULL } },
	{ MIPI_HOST_SYS_FATAL, { mipi_host_fatal_show_do, mipi_host_fatal_store_do } },
#if defined CONFIG_FAULT_INJECTION_ATTR || defined CONFIG_HOBOT_FUSA_DIAG
	{ MIPI_HOST_SYS_FAULT_INJECT, { mipi_host_fault_injection_show_do, mipi_host_fault_injection_store_do } },
#endif
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device sys operation
 *
 * @param[in] type: sys operation type
 * @param[in] sub: sys operation sub type show or stroe
 * @param[in] hdev: mipi host(rx) device struct
 * @param[in] name: sys node name
 * @param[in] buf: info to show or store
 * @param[in] count: buf size
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_sys_do(int32_t type, int32_t sub, struct mipi_hdev_s *hdev,
			       const char *name, char *buf, int32_t count)
{
	if ((type >= MIPI_HOST_SYS_NUM) || (sub >= MIPI_SYS_INVALID) ||
	    (mipi_host_sys[type].sys_do[sub] == NULL)) {
		return -EINVAL;
	}
	return mipi_host_sys[type].sys_do[sub](hdev, name, buf, count);
}

static int32_t hobot_mipi_host_phy_register(const struct mipi_hdev_s *hdev)
{
	int32_t ret;

#ifdef CONFIG_HOBOT_MIPI_PHY
	struct mipi_phy_sub_s sub = { 0 };

	sub.iomem = hdev->host.iomem;
	sub.osdev = (struct os_dev *)&hdev->osdev;
	sub.param = NULL;
	sub.port = hdev->port;
	ret = mipi_phy_register(MIPI_DPHY_TYPE_HOST, hdev->port, &sub);
#else
	struct os_dev *dev = &hdev->osdev;

	mipi_info(dev, "no dphy driver\n");
	ret = 0;
#endif

	return ret;
}

static void hobot_mipi_host_phy_unregister(const struct mipi_hdev_s *hdev)
{
#ifdef CONFIG_HOBOT_MIPI_PHY
	(void)mipi_phy_unregister(MIPI_DPHY_TYPE_HOST, hdev->port);
#endif
	return;
}

static void hobot_mipi_host_params_init(struct mipi_hdev_s *hdev)
{
	struct mipi_host_param_s *param = &hdev->host.param;

	param->irq_cnt = MIPI_HOST_IRQ_CNT;
	param->irq_debug = MIPI_HOST_IRQ_DEBUG;
	param->adv_value = MIPI_HOST_ADV_DEFAULT;
	if (MIPI_VERSION_GE(hdev->host.iomem, MIPI_IP_VERSION_1P5) == 0)
		param->cut_through = MIPI_HOST_CUT_DEFAULT | MIPI_HOST_CUT_HSD_LEGACY;
	else
		param->cut_through = MIPI_HOST_CUT_DEFAULT;
#ifdef X5_CHIP
        param->cut_through = MIPI_HOST_CUT_DEFAULT | MIPI_HOST_CUT_HSD_LEGACY;
#endif
	param->mem_flush = MIPI_HOST_MEM_DEFAULT;
	param->ipi_limit = MIPI_HOST_IPILIMIT_DEFAULT;
	param->ipi_overst = MIPI_HOST_IPI_OVERST_DEFAULT;
	param->wait_ms = HOST_DPHY_CHECK_MAX;
	param->error_diag = MIPI_HOST_ERROR_DIAG_DEFAULT;
#ifdef CONFIG_HOBOT_FUSA_DIAG
	param->stl_ovif = MIPI_HOST_OV_IGNORE_DFT_FRAME;
	param->stl_stif = MIPI_HOST_ST_INVALID_DFT_FRAME;
	param->drop_func = MIPI_HOST_PARAM_DROP_IN_STL;
#else
	param->drop_func = MIPI_HOST_PARAM_DROP_IN_IRQ;
#endif
}

static void hobot_mipi_host_ierrs_init(struct mipi_host_s *host)
{
	if (host->ap != 0) {
		host->ierr.st_main = REG_MIPI_HOST_INT_ST_AP_MAIN;
		if (MIPI_VERSION_GE(host->iomem, MIPI_IP_VERSION_1P5) != 0) {
			host->ierr.iregs = mh_int_regs_1p5ap;
			host->ierr.num = (uint32_t)MIPI_HOST_IREG_NUM_1P5AP;
			host->drops = g_mh_icnt_drops_1p5;
		} else {
			host->ierr.iregs = mh_int_regs_1p4ap;
			host->ierr.num = (uint32_t)MIPI_HOST_IREG_NUM_1P4AP;
			host->drops = g_mh_icnt_drops_1p4;
		}
	} else {
		host->ierr.st_main = REG_MIPI_HOST_INT_ST_MAIN;
		if (MIPI_VERSION_GE(host->iomem, MIPI_IP_VERSION_1P5) != 0) {
			host->ierr.iregs = mh_int_regs_1p5;
			host->ierr.num = (uint32_t)MIPI_HOST_IREG_NUM_1P5;
			host->drops = g_mh_icnt_drops_1p5;
		} else if (MIPI_VERSION_GE(host->iomem, MIPI_IP_VERSION_1P4) != 0) {
			host->ierr.iregs = mh_int_regs_1p4;
			host->ierr.num = (uint32_t)MIPI_HOST_IREG_NUM_1P4;
			host->drops = g_mh_icnt_drops_1p4;
		} else {
			host->ierr.iregs = mh_int_regs_1p3;
			host->ierr.num = (uint32_t)MIPI_HOST_IREG_NUM_1P3;
			host->drops = NULL;
		}
	}
#ifdef CONFIG_HOBOT_FUSA_DIAG
	host->ieap.st_main = REG_MIPI_HOST_INT_ST_AP_MAIN;

	if (MIPI_VERSION_GE(host->iomem, MIPI_IP_VERSION_1P5) != 0) {
		host->ieap.iregs = mh_int_regs_1p5ap;
		host->ieap.num = (uint32_t)MIPI_HOST_IREG_NUM_1P5AP;
	} else {
		host->ieap.iregs = mh_int_regs_1p4ap;
		host->ieap.num = (uint32_t)MIPI_HOST_IREG_NUM_1P4AP;
	}
#endif
}

static void hobot_mipi_host_probe_socclk_init(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_socclk_s *socclk = &host->socclk;
	int32_t i;

	if (hdev->port < (int32_t)MIPI_HOST_CFGCLK_NUM) { /* qacfix: conversion */
		socclk->cfgclk = g_mh_cfgclk_name[hdev->port];
	}

	if (hdev->port < (int32_t)MIPI_HOST_REFCLK_NUM) { /* qacfix: conversion */
		socclk->refclk = g_mh_refclk_name[hdev->port];
	}

	if (hdev->port < (int32_t)MIPI_HOST_PCLK_NUM) { /* qacfix: conversion */
		socclk->pclk = g_mh_pclk_name[hdev->port];
	}

	if (hdev->port < (int32_t)MIPI_HOST_IPICLK_NUM) { /* qacfix: conversion */
		for (i = 0; i < MIPI_HOST_HW_IPI_MAX; i++) {
			socclk->ipiclk[i] = g_mh_ipiclk_name[hdev->port][i];
		}
	}
}

/* report status to diag: status: 0-startup, 1-wakeup; step: 0-start, 1-end */
static void hobot_mipi_host_diag_report_status(const struct mipi_hdev_s *hdev, int32_t status, int32_t step)
{
#ifdef CONFIG_HOBOT_FUSA_DIAG
	int32_t ret;
	const struct os_dev *dev = &hdev->osdev;
	const uint16_t mod_ids[] = { (uint16_t)ModuleDiag_mipihost0, (uint16_t)ModuleDiag_mipihost1, /* qacfix: conversion */
		(uint16_t)ModuleDiag_mipihost2, (uint16_t)ModuleDiag_mipihost3 }; /* qacfix: conversion */

	if (hdev->port >= (int32_t)(sizeof(mod_ids)/sizeof(mod_ids[0]))) { /* qacfix: conversion */
		mipi_err(dev, "report status error, port %d overflow\n", hdev->port);
		return;
	}

	if (status == 0) {
		/* startup status */
		ret = diagnose_report_startup_status(mod_ids[hdev->port],
				(uint8_t)((step == 0) ? MODULE_STARTUP_BEGIN : MODULE_STARTUP_END));
		if (ret != 0) {
			mipi_err(dev, "report startup status %d error %d\n", step, ret);
		}
	} else {
		/* wakeup status */
		ret = diagnose_report_wakeup_status(mod_ids[hdev->port],
				(uint8_t)((step == 0) ? MODULE_WAKEUP_BEGIN : MODULE_WAKEUP_END));
		if (ret != 0) {
			mipi_err(dev, "report wakeup status %d error %d\n", step, ret);
		}
	}
#endif
	return;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device suspend operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_suspend_do(struct mipi_hdev_s *hdev)
{
	const struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;

	mipi_info(dev, "%s:%s enter suspend...\n", __FILE__, __func__);

	if (host->state == MIPI_STATE_START) {
		(void)mipi_host_stop(hdev);
		(void)mipi_host_deinit(hdev);
	} else if ((host->state == MIPI_STATE_INIT) ||
			(host->state == MIPI_STATE_STOP)) {
		(void)mipi_host_deinit(hdev);
	} else {
		/* state others */
	}

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device resume operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_resume_do(struct mipi_hdev_s *hdev)
{
	const struct os_dev *dev = &hdev->osdev;
	struct mipi_host_s *host = &hdev->host;
	int32_t ret = 0;

	mipi_info(dev, "%s:%s enter resume...\n", __FILE__, __func__);
	hobot_mipi_host_diag_report_status(hdev, 1, 0);

	if (host->state == MIPI_STATE_START) {
		/* if state == MIPI_STATE_START, it has been initialized. */
		ret = mipi_host_init(hdev, &host->cfg);
		if (ret != 0) {
			mipi_err(dev, "[%s] init error %d\n", __func__, ret);
			return ret;
		}

		/* start again */
		ret = mipi_host_start(hdev);
		if (ret != 0) {
			mipi_err(dev, "[%s] start error %d\n", __func__, ret);
			return ret;
		}
	} else if (host->state == MIPI_STATE_STOP) {
		/* if state == MIPI_STATE_STOP, it has been initialized. */
		ret = mipi_host_init(hdev, &host->cfg);
		if (ret != 0) {
			mipi_err(dev, "[%s] init error %d\n", __func__, ret);
			return ret;
		}
		(void)mipi_host_stop(hdev);
	} else if (host->state == MIPI_STATE_INIT) {
		ret = mipi_host_init(hdev, &host->cfg);
		if (ret != 0) {
			mipi_err(dev, "[%s] init error %d\n", __func__, ret);
			return ret;
		}
	} else {
		/* state others */
	}

	hobot_mipi_host_diag_report_status(hdev, 1, 1);
	return ret;
}

#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
static void hobot_mipi_host_probe_snrclk_do(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host = &hdev->host;
	struct mipi_host_snrclk_s *snrclk = &host->snrclk;

	if ((snrclk->index >= 0) && (snrclk->index < (int32_t)MIPI_HOST_SNRCLK_NUM)) { /* qacfix: conversion */
		host->socclk.snrclk = g_mh_snrclk_name[snrclk->index];
	}
	host->param.snrclk_en = MIPI_HOST_SNRCLK_NOUSED;
	if (snrclk->probe_init != 0U) {
		if (snrclk->probe_freq <= 1U) {
			(void)mipi_host_snrclk_set_en(hdev, snrclk->probe_freq);
		} else {
			(void)mipi_host_snrclk_set_freq(hdev, snrclk->probe_freq);
			(void)mipi_host_snrclk_set_en(hdev, 1U);
		}
	} else {
		host->param.snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev, host->socclk.snrclk));
                (void)mipi_host_snrclk_set_en(hdev, 0U); //select gpio, don't be random state
	}

	return;
}
#endif

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device probe operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_probe_do(struct mipi_hdev_s *hdev)
{
	uint32_t ver;
	struct mipi_host_s *host;

	if (hdev == NULL) {
		return -EINVAL;
	}
	if ((hdev->port >= MIPI_HOST_MAX_NUM) || (g_hdev[hdev->port] != NULL)) {
		mipi_host_error_report(hdev, ESW_MipiHostProbeErr, SUB_ID_0, 0U, __LINE__);
		return -EBUSY;
	}
	host = &hdev->host;
	if ((host->hw >= MIPI_HOST_PORT_HW_MODES_NUM) ||
		(host->iomem == NULL))	{
		mipi_host_error_report(hdev, ESW_MipiHostProbeErr, SUB_ID_1, 0U, __LINE__);
		return -EACCES;
	}
	/* hdev probe */
	hobot_mipi_host_diag_report_status(hdev, 0, 0);
	hdev->hw_mode = &g_mh_hw_modes[host->hw];
	hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST, hdev->port);

	/* param probe */
	hobot_mipi_host_params_init(hdev);

	/* socclk/snrclk probe */
	hobot_mipi_host_probe_socclk_init(hdev);
#ifdef CONFIG_HOBOT_MIPI_HOST_SNRCLK
	hobot_mipi_host_probe_snrclk_do(hdev);
#endif

	/* user probe */
	osal_mutex_init(&hdev->user.open_mutex); /* PRQA S 3334 */ /* mutex_init macro */
	hdev->user.open_cnt = 0U;

	/* phy probe */
	(void)hobot_mipi_host_phy_register(hdev);

	/* done & hw init */
	(void)mipi_host_configure_clk(hdev, hdev->host.socclk.cfgclk, MIPI_HOST_CFGCLK_MHZ, 1);
	(void)vio_clk_enable(hdev->host.socclk.pclk);/*pclk maybe adjust by clk_owner, we only need enable it.*/
#if MIPI_HOST_INT_DBG
	hobot_mipi_host_ierrs_init(host);
#endif
	ver = mipi_getreg(host->iomem, REG_MIPI_HOST_VERSION);
	mipi_info(&hdev->osdev, "ver %c%c%c%c%s port%d(%d:%d)\n",
			MIPI_BYTE3(ver), MIPI_BYTE2(ver),
			MIPI_BYTE1(ver), MIPI_BYTE0(ver),
			(host->ap != 0) ? "ap" : "", hdev->port, mipi_host_port_group(hdev),
			mipi_host_port_index(hdev));

	hobot_mipi_host_diag_report_status(hdev, 0, 1);

	g_hdev[hdev->port] = hdev;

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device remove operation
 *
 * @param[in] hdev: mipi host(rx) device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void hobot_mipi_host_remove_do(struct mipi_hdev_s *hdev)
{
	struct mipi_host_s *host;

	if ((hdev == NULL) || (g_hdev[hdev->port] == NULL)) {
		return;
	}
	host = &hdev->host;

#ifdef CONFIG_HOBOT_FUSA_DIAG
	mipi_csi_stl_remove(&host->stl);
#endif
	hobot_mipi_host_phy_unregister(hdev);

	(void)mipi_host_configure_clk(hdev, hdev->host.socclk.cfgclk, 0, 0);
	(void)mipi_host_configure_clk(hdev, hdev->host.socclk.pclk, 0, 0);

	g_hdev[hdev->port] = NULL;

	return;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) device set callback
 *
 * @param[in] hdev: mipi host(rx) device struct
 * @param[in] drop_cb: fram drop callback function
 * @param[in] int_cb: error interrupt callback function
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_setcb_do(struct mipi_hdev_s *hdev, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb)
{
	if (hdev == NULL) {
		return -EINVAL;
	}

	hdev->cb.drop_cb = drop_cb;
	hdev->cb.int_cb = int_cb;

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) hdev struct get
 *
 * @param[in] port: mipi host(rx) port index
 *
 * @return mipi host(rx) device struct point
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct mipi_hdev_s *hobot_mipi_host_hdev(int32_t port)
{
	if (port < MIPI_HOST_MAX_NUM)
		return g_hdev[port];
	return NULL;
}

#ifdef CONFIG_HOBOT_FUSA_DIAG
/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi host(rx) stl setup operation
 *
 * @param[in] port: mipi host(rx) port index
 * @param[in] setup: mipi host(rx) stl setup function
 * @param[in] call: mipi host(rx) stl functions to be called
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_host_stl_setup_do(int32_t port,
		MIPI_CSI_STL_SETUP setup, const struct mipi_csi_stl_call *call)
{
	struct mipi_csi_stl_setup_s s = { 0 };
	struct mipi_hdev_s *hdev = hobot_mipi_host_hdev(port);
	int32_t ret;

	if (hdev == NULL) {
		return -EINVAL;
	}

	if ((setup == NULL) || (call == NULL)) {
		/* remove */
		mipi_csi_stl_remove(&hdev->host.stl);
		hdev->host.stl.call = NULL;
		return 0;
	}

	/* setup */
	if (hdev->host.stl.call != NULL)  {
		return -EBUSY;
	}
	s.port = hdev->port;
	s.iomem = hdev->host.iomem;
	s.osdev = &hdev->osdev;
	s.dbg_level = &hdev->host.param.stl_dbg;
	s.fun_mask = &hdev->host.param.stl_mask;
	s.fun_pile = &hdev->host.param.stl_pile;
	s.ierr = &hdev->host.ierr;
	s.ieap = &hdev->host.ieap;
	s.iname = g_mh_icnt_names;
	s.cb = mipi_host_stl_drop_func;
	ret = setup(&hdev->host.stl, &s);
	if (ret < 0) {
		return ret;
	}
	hdev->host.stl.call = call;

	return 0;
}
#endif


