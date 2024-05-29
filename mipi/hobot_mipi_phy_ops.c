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
 * @file hobot_mipi_phy_ops.c
 *
 * @NO{S10E03C02}
 * @ASIL{B}
 */

#include "hobot_mipi_phy_ops.h"
#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_dev_regs.h"
#include "hobot_mipi_phy_regs.h"

/* module params of reged num */
uint32_t host_num;
uint32_t dev_num;

/* module params: txout freq */
uint32_t txout_freq_mode;
uint32_t txout_freq_autolarge_enbale;
uint32_t txout_freq_gain_precent = TXOUT_FREQ_GAIN_DEFAULT;
uint32_t txout_freq_force;

/* module params: rxdphy vref */
uint32_t rxdphy_vrefcd_lprx = RXPHY_VREFCD_LPRX_DEFFAULT;
uint32_t rxdphy_v400_prog = RXPHY_V400_PROG_DEFFAULT;
uint32_t rxdphy_deskew_cfg = RXPHY_DESKEW_DEFFAULT;

/* global var */
/* the main struct pointer of mipi dphy
 * see: hobot_mipi_dphy_probe
 */
struct mipi_pdev_s *g_pdev;

/* run params */
/* run params name of sysfs, see: struct mipi_phy_param_s */
static const char *g_mp_param_names[MIPI_PHY_PARAMS_NUM] = MIPI_PHY_PARAM_STRINGS;

/* debug loglevel by param->dbg_value */
#define mipi_dbg(param, dev, fmt, ...) do { \
		if (param->dbg_value != 0U) \
			mipi_debug(dev, fmt, ##__VA_ARGS__); \
	} while(0)

/* the type name of all phy type
 */
static const char *g_mp_type[] = { "host", "dev", "dsi" };

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy register for controller device
 *
 * @param[in] type: mipi controller type: host, etc.
 * @param[in] port: mipi controller port index
 * @param[in] sub: mipi phy sub struct
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
int32_t mipi_phy_register(int32_t type, int32_t port, const struct mipi_phy_sub_s *sub)
{
	struct mipi_phy_s *phy;
	struct mipi_dphy_tx_param_s *ptx;
	struct os_dev *dev;
	int32_t phy_max;
	uint32_t *num;

	if (sub == NULL) {
		return -EINVAL;
	}
	dev = (struct os_dev*)sub->osdev;

	if (type == MIPI_DPHY_TYPE_DEV) {
		phy = g_pdev->phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
		num = &dev_num;
	} else if (type == MIPI_DPHY_TYPE_HOST) {
		phy = g_pdev->phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
		num = &host_num;
	} else {
		mipi_err(dev, "dphy type %d error\n", type);
		return -EINVAL;
	}
	if ((port >= phy_max) || (phy[port].reged != 0U)) {
		mipi_err(dev, "dphy register error\n");
		return -EFAULT;
	}
	(void)memset((void *)(&phy[port]), 0, sizeof(struct mipi_phy_s));
	(void)memcpy(&phy[port].sub, sub, sizeof(struct mipi_phy_sub_s));
	if (type == MIPI_DPHY_TYPE_DEV) {
		/* init txout params of dev by dphy */
		ptx = (struct mipi_dphy_tx_param_s *)(phy[port].sub.param);
		if ((ptx != NULL) && (ptx->txout_param_valid == 0U)) {
			ptx->txout_freq_mode = txout_freq_mode;
			ptx->txout_freq_autolarge_enbale = txout_freq_autolarge_enbale;
			ptx->txout_freq_gain_precent = txout_freq_gain_precent;
			ptx->txout_freq_force = txout_freq_force;
		}
	}
	phy[port].reged = 1U;
	*num = *num + 1U;

	return 0;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy unregister for controller device
 *
 * @param[in] type: mipi controller type: host, etc.
 * @param[in] port: mipi controller port index
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
int32_t mipi_phy_unregister(int32_t type, int32_t port)
{
	struct mipi_phy_s *phy;
	int32_t phy_max;
	uint32_t *num;

	if (type == MIPI_DPHY_TYPE_DEV) {
		phy = g_pdev->phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
		num = &dev_num;
	} else if (type == MIPI_DPHY_TYPE_HOST) {
		phy = g_pdev->phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
		num = &host_num;
	} else {
		return -EINVAL;
	}
	if (port >= phy_max) {
		return -EFAULT;
	}
	phy[port].reged = 0U;
	*num = *num - 1U;

	return 0;
}

/* get osdev */
static inline struct os_dev *mipi_dphy_osdev(const struct mipi_phy_s *phy)
{
	return (struct os_dev *)((phy != NULL) ? phy->sub.osdev : &g_pdev->osdev);
}

/* mipi phy struct get by iomem */
static struct mipi_phy_s *mipi_dphy_get_phy(int32_t type, const void __iomem  *iomem)
{
	struct mipi_phy_s *phy;
	int32_t phy_max;
	int32_t i;

	if (type == MIPI_DPHY_TYPE_DEV) {
		phy = g_pdev->phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
	} else if (type == MIPI_DPHY_TYPE_HOST) {
		phy = g_pdev->phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
	} else {
		return NULL;
	}

	for (i = 0; i < phy_max; i ++) {
		if ((phy[i].reged != 0U) && (phy[i].sub.iomem == iomem)) {
			return &phy[i];
		}
	}

	return NULL;
}

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
/* mipi phy struct get by type & port */
static struct mipi_phy_s *mipi_dphy_get_phy_byport(int32_t type, int32_t port)
{
	struct mipi_phy_s *phy;
	int32_t phy_max;

	if (type == MIPI_DPHY_TYPE_DEV) {
		phy = g_pdev->phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
	} else if (type == MIPI_DPHY_TYPE_HOST) {
		phy = g_pdev->phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
	} else {
		return NULL;
	}

	if ((port < phy_max) && (phy[port].reged != 0U)) {
		return &phy[port];
	}

	return NULL;
}
#endif

/* mipi dphy pll range table */
static const struct pll_sel_table_s g_pll_sel_table[] = {
	{438, 460, 80, 0x00},
	{438, 460, 90, 0x10},
	{438, 460, 100, 0x20},
	{438, 460, 110, 0x30},
	{438, 460, 120, 0x01},
	{438, 460, 130, 0x11},
	{438, 460, 140, 0x21},
	{438, 460, 150, 0x31},
	{438, 460, 160, 0x02},
	{438, 460, 170, 0x12},
	{438, 460, 180, 0x22},
	{438, 460, 190, 0x32},
	{438, 460, 205, 0x03},
	{438, 460, 220, 0x13},
	{438, 460, 235, 0x23},
	{438, 460, 250, 0x33},
	{438, 460, 270, 0x04},
	{438, 460, 290, 0x14},
	{438, 460, 310, 0x25},
	{438, 460, 330, 0x35},
	{438, 460, 375, 0x05},
	{438, 460, 425, 0x16},
	{438, 460, 475, 0x26},
	{438, 460, 525, 0x37},
	{438, 460, 575, 0x07},
	{438, 460, 630, 0x18},
	{438, 460, 680, 0x28},
	{438, 460, 720, 0x39},
	{438, 460, 780, 0x09},
	{438, 460, 820, 0x19},
	{438, 460, 880, 0x29},
	{438, 460, 920, 0x3A},
	{438, 460, 980, 0x0A},
	{438, 460, 1020, 0x1A},
	{438, 460, 1100, 0x2A},
	{438, 460, 1150, 0x3B},
	{438, 460, 1200, 0x0B},
	{438, 460, 1250, 0x1B},
	{438, 460, 1300, 0x2B},
	{438, 460, 1350, 0x3C},
	{438, 460, 1400, 0x0C},
	{438, 460, 1450, 0x1C},
	{438, 460, 1500, 0x2C},
	{271, 285, 1550, 0x3D},
	{280, 295, 1600, 0x0D},
	{289, 304, 1650, 0x1D},
	{298, 313, 1700, 0x2E},
	{306, 322, 1750, 0x3E},
	{315, 331, 1800, 0x0E},
	{324, 341, 1850, 0x1E},
	{333, 350, 1900, 0x2F},
	{341, 359, 1950, 0x3F},
	{350, 368, 2000, 0x0F},
	{359, 377, 2050, 0x40},
	{368, 387, 2100, 0x41},
	{376, 396, 2150, 0x42},
	{385, 405, 2200, 0x43},
	{394, 414, 2250, 0x44},
	{403, 423, 2300, 0x45},
	{411, 432, 2350, 0x46},
	{420, 442, 2400, 0x47},
	{429, 451, 2450, 0x48},
	{438, 460, 2500, 0x49},
	{438, 460, 2501, 0x49},
};
#define PLL_SEL_TABLE_SIZE	((int32_t)(sizeof(g_pll_sel_table)/sizeof(g_pll_sel_table[0])))

/* get clk range from table by mipiclk */
static int32_t mipi_dphy_clk_range(struct mipi_phy_s *phy, uint16_t mipiclk, uint16_t *osc_freq)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t  index;
	uint16_t osc_freq_v;
	int32_t is_1p4 = 0;

	for (index = 0; index < (PLL_SEL_TABLE_SIZE - 1); index++) {
		if ((mipiclk >= g_pll_sel_table[index].freq) &&
			(mipiclk < g_pll_sel_table[index + 1].freq)) {
			mipi_dbg(param, dev, "pll div mipiclk: %d, clk: %d-%d, range: 0x%x\n",
			 mipiclk, g_pll_sel_table[index].freq,
			 g_pll_sel_table[index + 1].freq,
			 g_pll_sel_table[index].value);
			if (osc_freq != NULL) {
				if (phy != NULL) {
					if (MIPI_VERSION_GE(phy->sub.iomem, MIPI_IP_VERSION_1P4) != 0) {
						is_1p4 = 1;
					}
				}
				if (is_1p4 != 0) {
					osc_freq_v = g_pll_sel_table[index].osc_freq_1p4;
				} else {
					osc_freq_v = g_pll_sel_table[index].osc_freq;
				}
				mipi_dbg(param, dev, "pll osc_freq: %d\n", osc_freq_v);
				*osc_freq = osc_freq_v;
			}
			if (phy != NULL) {
				phy->pll_sel = (const void *)&g_pll_sel_table[index];
			}
			return (int32_t)(g_pll_sel_table[index].value);
		}
	}

	mipi_dbg(param, dev, "mipi clock %d not supported\n", mipiclk);
	return 0;
}

/**
 * brief mipi_host_dphy_testdata : write test data
 *
 * param [in] testdata : testdatas' array
 * param [in] size : size of testdata's array
 *
 * return void
 */
static void mipi_host_dphy_testdata(const struct mipi_phy_s *phy, const void __iomem *iomem,
				uint16_t testcode, uint8_t testdata)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
#ifdef X5_CHIP
	uint16_t port = (phy != NULL) ? (phy->sub.port) : 0;
#else
	uint32_t regv;
#endif
	if (iomem == NULL) {
		return;
	}

#ifdef X5_CHIP
	/*write test code*/
	mipi_dphy_set_testcode(MIPI_DPHY_TYPE_HOST, port, testcode);

	/*write test data*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, testdata); /*set test data*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	osal_mdelay(1);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	osal_mdelay(1);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
#else

	/*write test code*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, TESTCODE_MSBS_BYTE(testcode));    /*set testen to low, set test code MSBS*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | TESTCODE_LSBS_BYTE(testcode));  /*set test code LSBS*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
#endif
	mipi_dbg(param, dev, "host dphy test code:0x%x, data: 0x%x\n", testcode, testdata);
}

/**
 * @brief mipi_host_dphy_testdata_read: read test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdata's array
 *
 * @return void
 */
uint32_t mipi_host_dphy_testdata_read(const struct mipi_phy_s *phy, const void __iomem *iomem, uint16_t testcode)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
        uint32_t testdata;
#ifdef X5_CHIP
	uint16_t port = (phy != NULL) ? (phy->sub.port) : 0;
#else
	uint32_t regv;
#endif
	if (iomem == NULL) {
		return -1;
	}

#ifdef X5_CHIP
	/*write test code*/
	mipi_dphy_set_testcode(MIPI_DPHY_TYPE_HOST, port, testcode);

	/*read test data*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK); /*set testclk to high*/
	osal_mdelay(1);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
#else
        /*write test code*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, testcode >> 8);    /*set testen to low, set test code MSBS*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
        regv = mipi_getreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1);
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | (testcode & 0xff));  /*set test code LSBS*/
        mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
#endif
        testdata = (mipi_getreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1) >> 8) & 0xff;
	mipi_dbg(param, dev, "host dphy test code:0x%x, read data: 0x%x\n", testcode, testdata);
        return testdata;
}

static void dphy_merge_write(struct mipi_phy_s *phy, const void __iomem *iomem, uint32_t reg_addr,
				uint32_t major, uint32_t minor)
{
	uint16_t port = (phy != NULL) ? (phy->sub.port) : 0;

	mipi_dphy_set_test_sel_4l(MIPI_DPHY_TYPE_HOST, port, 0);
	mipi_host_dphy_testdata(phy, iomem, reg_addr, major);
	mipi_dphy_set_test_sel_4l(MIPI_DPHY_TYPE_HOST, port, 1);
	mipi_host_dphy_testdata(phy, iomem, reg_addr, minor);
}

/* host 1p4 dphy init */
static void mipi_host_dphy_initialize_1p4(const struct mipi_phy_s *phy, const void __iomem *iomem,
				uint8_t osc_freq_low, uint8_t osc_freq_high)
{
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_6, RX_CLKLANE_PULLLONG);
#ifdef CONFIG_HOBOT_XJ3
		if (phy && (phy->lane > RXPHY_MERGE_LANE_MIN)) {
			mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_7, RX_CLK_200MODE);
		}
#endif
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CB_0, RX_CB_VREF_CB(rxdphy_vrefcd_lprx, rxdphy_v400_prog));
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CB_2, RX_CB_BIAS_ATB);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_2, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_3, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_4, RX_CLK_SETTLE | RX_OSCFREQ_EN);

		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_6, RX_OSCFREQ_EN);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_6, RX_OSCFREQ_EN);
}

/* host 1p3 dphy init */
static void mipi_host_dphy_initialize_1p3(const struct mipi_phy_s *phy, const void __iomem *iomem,
				uint8_t osc_freq_low, uint8_t osc_freq_high)
{
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_6, RX_OSCFREQ_EN);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_6, RX_OSCFREQ_EN);

		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_6, RX_OSCFREQ_EN);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_4, osc_freq_low);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_5, osc_freq_high);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_6, RX_OSCFREQ_EN);
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi cphy initialize for host controller
 *
 * @param[in] mipiclk: mipi clock frequency as Mbps
 * @param[in] lane: mipi lane num of cphy
 * @param[in] settle: mipi hs settle config for dphy
 * @param[in] iomem: iomem reg base address
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
int32_t mipi_host_cphy_initialize(uint16_t mipiclk, uint16_t lane, uint16_t settle, const void __iomem *iomem)
{
	// TODO: add
	return 0;
}

/* for X5, as soon as PHY-A finishes starting up,
 * rxclk_rxhs_feed_int_clk is overridden to 1'b0 with the override registers:
 * rxclk_rxhs_feed_int_clk_ovr_en_rw = 1 and rxclk_rxhs_feed_int_clk_ovr_rw = 0
 * */
int32_t mipi_host_dphy_set_source(const void __iomem *iomem)
{
	struct mipi_phy_s *phy = mipi_dphy_get_phy(MIPI_DPHY_TYPE_HOST, iomem);
	uint16_t rxclk_rxhs_feed_int_clk = 0;
	if(phy != NULL) {
		rxclk_rxhs_feed_int_clk = mipi_host_dphy_testdata_read(phy, iomem, REGS_RX_CLKLANE_LANE_6);
		//rxclk_rxhs_feed_int_clk_ovr_en_rw set 1 - bit5
		//rxclk_rxhs_feed_int_clk_ovr_rw set 0 - bit4
		rxclk_rxhs_feed_int_clk |= 0x20;
		rxclk_rxhs_feed_int_clk &= 0xEF;
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_6, rxclk_rxhs_feed_int_clk);
	}
	return 0;
}


/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy initialize for host controller
 *
 * @param[in] mipiclk: mipi clock frequency as Mbps
 * @param[in] lane: mipi lane num of dphy
 * @param[in] settle: mipi hs settle config for dphy
 * @param[in] iomem: iomem reg base address
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
int32_t mipi_host_dphy_initialize(uint16_t mipiclk, uint16_t lane, uint16_t settle, const void __iomem *iomem)
{
	struct mipi_phy_s *phy = mipi_dphy_get_phy(MIPI_DPHY_TYPE_HOST, iomem);
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t is_1p4 = (phy != NULL) ? MIPI_VERSION_GE(phy->sub.iomem, MIPI_IP_VERSION_1P4) : 1;
	uint16_t osc_freq = (is_1p4 != 0) ? OSC_FREQ_DEFAULT_1P4: OSC_FREQ_DEFAULT_1P3;
	uint16_t port = (phy != NULL) ? (phy->sub.port) : 0;
	uint8_t osc_freq_low, osc_freq_high;

	mipi_dbg(param, dev, "host dphy initialize begin\n");
	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/

	if (settle == 0) {
		settle = mipi_host_dphy_testdata_read(phy, iomem, REGS_RX_SYSTIMERS_0);
		mipi_info(dev, "host dphy settle as default %d\n", settle);
	} else {
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_5, RX_CLK_SETTLE_EN);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_4, RX_CLK_SETTLE);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_17, RX_HS_SETTLE(settle));
	}
	/*Configure the D-PHY frequency range*/
#ifdef X5_CHIP
       (void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, port,
               MIPI_PHY_HS_FREQRANGE, mipi_dphy_clk_range(phy, mipiclk / lane, &osc_freq));

#else
	(void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, port,
		MIPI_HSFREQRANGE, mipi_dphy_clk_range(phy, mipiclk / lane, &osc_freq));
#endif

	if(lane >= 3 && (port == 0 || port == 2)) {
		dphy_merge_write(phy, iomem, REGS_RX_DUAL_PHY_0, 1, 0);
		dphy_merge_write(phy, iomem, REGS_RX_CLKLANE_LANE_6, 4, 0);
		dphy_merge_write(phy, iomem, REGS_RX_LANE0_LANE_7, 0x20, 0x20);
		dphy_merge_write(phy, iomem, REGS_RX_LANE1_LANE_7, 0x20, 0x20);

		dphy_merge_write(phy, iomem, REGS_RX_LANE2_LANE7, 0x20, 0x20);
		dphy_merge_write(phy, iomem, REGS_RX_LANE3_LANE7, 0x20, 0x20);
		dphy_merge_write(phy, iomem, REGS_RX_CLKLANE_LANE_7, 0x00, 0x08);

		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_0, 0x03);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_1, 0x02);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_6, 0x08);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_3, 0x80);
		mipi_host_dphy_testdata(phy, iomem, REGS_RX_CLKLANE_LANE_4, 0xa);

	} else {
		osc_freq_low = RX_OSCFREQ_LOW(osc_freq);
		osc_freq_high = RX_OSCFREQ_HIGH(osc_freq);
		if (rxdphy_deskew_cfg != 0U) {
			mipi_host_dphy_testdata(phy, iomem, REGS_RX_SYS_7, (uint8_t)rxdphy_deskew_cfg); /* qacfix: conversion */
		} else {
			if (mipiclk < (lane * TXOUT_FREQ_SLEWRATE_MBPS)) {
				mipi_host_dphy_testdata(phy, iomem, REGS_RX_SYS_7, RX_SYSTEM_CONFIG);
			}
		}
		if (is_1p4 != 0) {
			mipi_host_dphy_initialize_1p4(phy, iomem, osc_freq_low, osc_freq_high);
		} else {
			mipi_host_dphy_initialize_1p3(phy, iomem, osc_freq_low, osc_freq_high);
		}
	}

	/* record host */
	if (phy != NULL) {
		phy->mipiclk = mipiclk;
		phy->lane= lane;
		phy->settle = settle;
		phy->init_cnt ++;
	}
	return 0;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy reset for host controller
 *
 * @param[in] iomem: iomem reg base address
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void mipi_host_dphy_reset(const void __iomem *iomem)
{
	struct mipi_phy_s *phy = mipi_dphy_get_phy(MIPI_DPHY_TYPE_HOST, iomem);

	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(iomem, REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);

	/* record host */
	if (phy != NULL) {
		phy->reset_cnt ++;
	}
}

/**
 * brief mipi_dev_dphy_testdata : write test data
 *
 * param [in] testdata : testdatas' array
 * param [in] size : size of testdata's array
 *
 * return void
 */
static void mipi_dev_dphy_testdata(const struct mipi_phy_s *phy, const void __iomem *iomem,
				uint16_t testcode, uint8_t testdata)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	uint32_t regv;

	if (iomem == NULL) {
		return;
	}

	/*write test code*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1, TESTCODE_MSBS_BYTE(testcode));    /*set testen to low, set test code MSBS*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1);
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | TESTCODE_LSBS_BYTE(testcode));  /*set test code LSBS*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_dbg(param, dev, "dev dphy test code:0x%x, data: 0x%x\n", testcode, testdata);
}

static const struct pll_range_table_s g_vco_range_table[] = {
	{1150, 1250, 0x01},
	{1100, 1152, 0x01},
	{630,  1149, 0x03},
	{420,  660,  0x09},
	{320,  440,  0x0F},
	{210,  330,  0x19},
	{160,  220,  0x1F},
	{105,  165,  0x29},
	{80,   110,  0x2F},
	{53,   83,   0x39},
	{40,   55,   0x3F},
};
#define VCO_RANGE_TABLE_SIZE	((int32_t)(sizeof(g_vco_range_table)/sizeof(g_vco_range_table[0])))

static uint32_t mipi_tx_vco_range(const struct mipi_phy_s *phy, uint16_t outclk, uint16_t *outmax)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t  index;

	for (index = 0; index < VCO_RANGE_TABLE_SIZE; index++) {
		if ((outclk >= g_vco_range_table[index].low) &&
			(outclk <= g_vco_range_table[index].high)) {
			mipi_dbg(param, dev, "outclk: %d, selected: %d-%d, range: %d\n",
				outclk, g_vco_range_table[index].low,
				g_vco_range_table[index].high,
				g_vco_range_table[index].value);
			*outmax = g_vco_range_table[index].high;
			return g_vco_range_table[index].value;
		}
	}
	mipi_dbg(param, dev, "out clock %d not supported\n", outclk);
	return 0;
}

static int32_t mipi_tx_pll_div_mn(uint16_t refsclk, uint16_t fvco, uint16_t *m, uint16_t *n)
{
	uint16_t n_tmp = TX_PLL_INPUT_DIV_MAX;
	uint16_t m_tmp;

	if ((m == NULL) || (n == NULL)) {
		return -1;
	}
	while (n_tmp >= TX_PLL_INPUT_DIV_MIN) {
		if ((refsclk / n_tmp) < TX_PLL_INPUT_FEQ_MIN) {
			n_tmp--;
			continue;
		}
		m_tmp = fvco * n_tmp / refsclk;
		if ((m_tmp >= TX_PLL_FB_MULTI_MIN) &&
			(m_tmp < TX_PLL_FB_MULTI_MAX)) {
			m_tmp++;
			break;
		}
		n_tmp--;
	}
	if (n_tmp < TX_PLL_INPUT_DIV_MIN) {
		return -1;
	}

	*m = m_tmp;
	*n = n_tmp;
	return 0;
}

static uint16_t mipi_tx_pll_div(const struct mipi_phy_s *phy, uint16_t refsclk, uint16_t laneclk, uint8_t *n, uint16_t *m, uint16_t *vco)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_dphy_tx_param_s *ptx = NULL;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	uint32_t vco_tmp;
	uint16_t n_tmp = TX_PLL_INPUT_DIV_MAX;
	uint16_t m_tmp;
	uint16_t fout;
	uint16_t fvco;
	uint16_t fout_max = 0;
	uint16_t vco_div;
	uint16_t outclk;
	uint16_t laneclk_s = laneclk;
	uint32_t s_txout_freq_autolarge_enbale;
	int32_t ret;

	if ((refsclk == 0U) || (laneclk_s == 0U) || (n == NULL) || (m == NULL)) {
		mipi_err(dev, "pll input error!!!\n");
		return 0U;
	}

	if (phy != NULL) {
		ptx = (struct mipi_dphy_tx_param_s *)(phy->sub.param);
	}
	if ((ptx != NULL) && (ptx->txout_param_valid != 0U)) {
		s_txout_freq_autolarge_enbale = ptx->txout_freq_autolarge_enbale;
	} else {
		s_txout_freq_autolarge_enbale = txout_freq_autolarge_enbale;
	}

	fout = laneclk_s / MIPICLK_FREQ_MHZ_2_MBPS; /* data rate(Gbps) = PLL Fout(GHz)*2 */
	vco_tmp = mipi_tx_vco_range(phy, fout, &fout_max);
	if (vco_tmp == 0U) {
		mipi_err(dev, "pll output clk error!!! laneclk: %d\n", laneclk_s);
		return 0U;
	}
	if (s_txout_freq_autolarge_enbale != 0U) {
		mipi_dbg(param, dev, "txout freq autolarge to %dMHz\n", fout_max);
		fout = fout_max;
		laneclk_s = fout * MIPICLK_FREQ_MHZ_2_MBPS;
	}
	vco_div = TX_PLL_VCO_DIV(vco_tmp);
	fvco = fout << vco_div;
	if ((TX_PLL_INPUT_FEQ_MIN > (refsclk / TX_PLL_INPUT_DIV_MIN)) ||
		(TX_PLL_INPUT_FEQ_MAX < (refsclk / TX_PLL_INPUT_DIV_MAX))) {
		mipi_err(dev, "pll parameter error!!! refsclk: %d, laneclk: %d\n", refsclk, laneclk_s);
		return 0U;
	}
	ret = mipi_tx_pll_div_mn(refsclk, fvco, &m_tmp, &n_tmp);
	if (ret != 0) {
		mipi_err(dev, "pll output clk error!!! fvco: %d, refsclk: %d\n",
			fvco, refsclk);
		return 0U;
	}
	*vco = (uint16_t)vco_tmp;
	*n = (uint8_t)(n_tmp - TX_PLL_N_BASE);
	*m = m_tmp - TX_PLL_M_BASE;
	fvco = (refsclk * (*m + TX_PLL_M_BASE)) / ((uint16_t)(*n) + TX_PLL_N_BASE);
	fout = fvco >> vco_div;
	outclk = fout * MIPICLK_FREQ_MHZ_2_MBPS;
	mipi_dbg(param, dev, "pll div refsclk: %d, laneclk: %dMbps, n: %d, m: %d\n",
			 refsclk, laneclk_s, *n, *m);
	mipi_dbg(param, dev, "pll vco: %d, outclk: %dMbps(%dMHz)\n",
			 *vco, outclk, fout);
	return outclk;
}

/* dev dphy init with auto mode */
static int32_t mipi_dev_dphy_initialize_automode(struct mipi_phy_s *phy, const void __iomem *iomem,
			uint16_t setclk, uint16_t lane)
{
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	uint16_t outclk;

	outclk = (uint16_t)(setclk / lane);
	mipi_dbg(param, dev, "freq automode outclk: %dMbps(%dMHz)\n",
			outclk, outclk / MIPICLK_FREQ_MHZ_2_MBPS);

	/*Configure the D-PHY frequency range*/
	(void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DEV, (phy != NULL) ? (phy->sub.port) : 0,
								  MIPI_HSFREQRANGE, mipi_dphy_clk_range(phy, setclk / lane, NULL));
	if ((setclk / lane) >= TXOUT_FREQ_SLEWRATE_MBPS) {
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEWRATE_FSM_OVR, TX_SLEWRATE_FSM_OVR_EN);
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEWRATE_DDL_CFG, TX_SLEWRATE_DDL_CFG_SEL);
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_ANALOG_PROG, TX_PLL_ANALOG_PROG_CTL);
	}
	/* fix for 2Gbps */
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_CPBIAS_CNTRL, TX_PLL_CPBIAS_CNTRL0);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_GMP_DIG, TX_PLL_GMP_DIG_LOCK);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_INT_CHG_PUMP, TX_PLL_INT_CHG_PUMP_CTL);
	if ((setclk / lane) >= TXOUT_FREQ_CHGPUMP_MBPS) {
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_PRO_CHG_PUMP, TX_PLL_PRO_CHG_PUMP_CTLE);
	} else {
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_PRO_CHG_PUMP, TX_PLL_PRO_CHG_PUMP_CTLD);
	}
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_PHASE_ERR, TX_PLL_PHASE_ERR_TH1);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_LOCK_FILT, TX_PLL_LOCK_FILT_TH2);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_UNLOCK_FILT, TX_PLL_UNLOCK_FILT_TH3);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_BANDGA_CNTRL, TX_BANDGA_CNTRL_VAL);

	return 0;
}

/* dev dphy init with cal mode */
static int32_t mipi_dev_dphy_initialize_calmode(struct mipi_phy_s *phy, const void __iomem *iomem,
			uint16_t setclk, uint16_t lane, uint16_t settle)
{
	uint16_t outclk;
	uint8_t n = 0U;
	uint16_t m = 0U;
	uint16_t vco = 0U;

	outclk = mipi_tx_pll_div(phy, TX_REFSCLK_DEFAULT, (setclk / lane), &n, &m, &vco);
	if (outclk == 0U) {
		return -1;
	}

	/*Configure the D-PHY frequency range*/
	(void)mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DEV, (phy != NULL) ? (phy->sub.port) : 0,
								  MIPI_HSFREQRANGE, mipi_dphy_clk_range(phy, setclk / lane, NULL));

	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEW_5, TX_SLEW_RATE_CAL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEW_7, TX_SLEW_RATE_CTL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_27, TX_PLL_DIV(n));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_28, TX_PLL_MULTI_L(m));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_29, TX_PLL_MULTI_H(m));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_30, TX_PLL_VCO(vco));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SYSTIMERS_23, TX_HS_ZERO(settle));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_1,  TX_PLL_CPBIAS);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_4,  TX_PLL_INT_CTL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_17, TX_PLL_PROP_CNTRL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_19, TX_PLL_RST_TIME_L);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_2,  TX_PLL_GEAR_SHIFT_L);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_3,  TX_PLL_GEAR_SHIFT_H);
	if (outclk < TX_PLL_CLKDIV_CLK_LMT) {
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_CB_2, TX_PLL_CLKDIV_CLK_EN);
	}
#if defined CONFIG_HOBOT_XJ3
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_LOCK_DETMODE, TX_PLL_FORCE_LOCK);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_TESECODE_08, TX_TESETCODE_08_DATA);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_HSTXTHSZERO_OVR, TX_HSTXTHSZERO_DATALANES);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_TH_DELAY, TX_PLL_TH_DELAY);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_CPBIAS_CNTRL, TX_PLL_CPBIAS_CNTRL1);
#endif

	return 0;
}

/**
 * brief mipi_dev_initialize_dphy : initialize dev phy
 *
 * param [in] control : the dev controller's setting
 *
 * return int32_t : 0/-1
 */
int32_t mipi_dev_dphy_initialize(const void __iomem *iomem, uint16_t mipiclk, uint16_t lane, uint16_t settle)
{
	struct mipi_phy_s *phy = mipi_dphy_get_phy(MIPI_DPHY_TYPE_DEV, iomem);
	struct os_dev *dev = mipi_dphy_osdev(phy);
	struct mipi_dphy_tx_param_s *ptx = NULL;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	uint16_t   outclk, setclk;
	uint32_t s_txout_freq_mode, s_txout_freq_force, s_txout_freq_gain_precent;
	int32_t ret;

	mipi_dbg(param, dev, "device dphy initialize dphy begin\n");

	if (phy != NULL) {
		ptx = (struct mipi_dphy_tx_param_s *)(phy->sub.param);
	}
	if ((ptx != NULL) && (ptx->txout_param_valid != 0U)) {
		s_txout_freq_mode = ptx->txout_freq_mode;
		s_txout_freq_force = ptx->txout_freq_force;
		s_txout_freq_gain_precent = ptx->txout_freq_gain_precent;
	} else {
		s_txout_freq_mode = txout_freq_mode;
		s_txout_freq_force = txout_freq_force;
		s_txout_freq_gain_precent = txout_freq_gain_precent;
	}

	if (s_txout_freq_force >= TXOUT_FREQ_FORCE_MIN_MHZ) {
		mipi_dbg(param, dev, "txout freq force as %dMHz\n", s_txout_freq_force);
		setclk = (uint16_t)(s_txout_freq_force * MIPICLK_FREQ_MHZ_2_MBPS * lane);
	} else if (s_txout_freq_gain_precent != 0U) {
		outclk = (uint16_t)(mipiclk * (TXOUT_FREQ_GAIN_PERCENT + s_txout_freq_gain_precent) / TXOUT_FREQ_GAIN_PERCENT);
		mipi_dbg(param, dev, "txout freq %dMHz gain %d%% to %dMHz\n",
			mipiclk / lane / MIPICLK_FREQ_MHZ_2_MBPS,
			s_txout_freq_gain_precent,
			outclk / lane / MIPICLK_FREQ_MHZ_2_MBPS);
		setclk = outclk;
	} else {
		setclk = mipiclk;
	}

	/*Configure the D-PHY PLL*/
	mipi_putreg(iomem, (uint32_t)REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/
	if (s_txout_freq_mode == 0U) {
		ret = mipi_dev_dphy_initialize_automode(phy, iomem, setclk, lane);
	} else {
		ret = mipi_dev_dphy_initialize_calmode(phy, iomem, setclk, lane, settle);
	}
	if (ret != 0) {
		return ret;
	}

	/* record host */
	if (phy != NULL) {
		phy->mipiclk = setclk;
		phy->lane= lane;
		phy->settle = settle;
		phy->init_cnt ++;
	}
	return 0;
}

/**
 * brief mipi_dev_initialize_dphy : reset dev phy
 *
 * param [] void :
 *
 * return void
 */
void mipi_dev_dphy_reset(const void __iomem *iomem)
{
	struct mipi_phy_s *phy = mipi_dphy_get_phy(MIPI_DPHY_TYPE_DEV ,iomem);

	mipi_putreg(iomem, REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLEAR);

	/* record host */
	if (phy != NULL) {
		phy->reset_cnt ++;
	}
}

#if defined CONFIG_HOBOT_XJ2
#define MIPI_HOST_HW_PORT_NUM	(1)
#define MIPI_DEV_HW_PORT_NUM	(1)
static int32_t x2ips_mipi_get_ctl(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, REG_X2IPS_MIPI_CTRL);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		ret = DP_REG2V(val, X2IPS_MIPI_BPASS_GEN_DLY_MASK, X2IPS_MIPI_BPASS_GEN_DLY_OFFS);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		ret = DP_REG2V(val, X2IPS_MIPI_BPASS_GEN_HSYNC_MASK, X2IPS_MIPI_BPASS_GEN_HSYNC_OFFS);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		ret = DP_REG2V(val, X2IPS_MIPI_DEV_SHADOW_CLR_MASK, X2IPS_MIPI_DEV_SHADOW_CLR_OFFS);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static int32_t x2ips_mipi_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret = 0;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, REG_X2IPS_MIPI_CTRL);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val &= ~DP_VMASK(X2IPS_MIPI_BPASS_GEN_DLY_MASK, X2IPS_MIPI_BPASS_GEN_DLY_OFFS);
		val |= DP_V2REG(value, X2IPS_MIPI_BPASS_GEN_DLY_MASK, X2IPS_MIPI_BPASS_GEN_DLY_OFFS);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val &= ~DP_VMASK(X2IPS_MIPI_BPASS_GEN_HSYNC_MASK, X2IPS_MIPI_BPASS_GEN_HSYNC_OFFS);
		val |= DP_V2REG(value, X2IPS_MIPI_BPASS_GEN_HSYNC_MASK, X2IPS_MIPI_BPASS_GEN_HSYNC_OFFS);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val &= ~DP_VMASK(X2IPS_MIPI_DEV_SHADOW_CLR_MASK, X2IPS_MIPI_DEV_SHADOW_CLR_OFFS);
		val |= DP_V2REG(value, X2IPS_MIPI_DEV_SHADOW_CLR_MASK, X2IPS_MIPI_DEV_SHADOW_CLR_OFFS);
		break;
	default:
		ret = -1;
		break;
	}
	mipi_putreg(iomem, REG_X2IPS_MIPI_CTRL, val);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */

	mipi_dbg(param, dev, "set mipi%s%d ctl region %d value %d, regv 0x%x\n",
			g_mp_type[type], port, region, value, val);
	return ret;
}

static int32_t x2ips_mipi_get_freqrange(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, REG_X2IPS_MIPI_FREQRANGE);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			ret = DP_REG2V(val, X2IPS_MIPI_HOST_CFGCLK_FRANGE_MASK, X2IPS_MIPI_HOST_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			ret = DP_REG2V(val, X2IPS_MIPI_HOST_HS_FRANGE_MASK, X2IPS_MIPI_HOST_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM)) {
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			ret = DP_REG2V(val, X2IPS_MIPI_DEV_CFGCLK_FRANGE_MASK, X2IPS_MIPI_DEV_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			ret = DP_REG2V(val, X2IPS_MIPI_DEV_HS_FRANGE_MASK, X2IPS_MIPI_DEV_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t x2ips_mipi_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		val = mipi_getreg(iomem, REG_X2IPS_MIPI_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X2IPS_MIPI_HOST_CFGCLK_FRANGE_MASK, X2IPS_MIPI_HOST_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, X2IPS_MIPI_HOST_CFGCLK_FRANGE_MASK, X2IPS_MIPI_HOST_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X2IPS_MIPI_HOST_HS_FRANGE_MASK, X2IPS_MIPI_HOST_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, X2IPS_MIPI_HOST_HS_FRANGE_MASK, X2IPS_MIPI_HOST_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, REG_X2IPS_MIPI_FREQRANGE, val);
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM)) {
		val = mipi_getreg(iomem, REG_X2IPS_MIPI_DEV_PLL_CTRL2);
		val &= ~DP_VMASK(X2IPS_MIPI_PLL_SEL_CLR_MASK, X2IPS_MIPI_PLL_SEL_CLR_OFFS);
		val |= DP_V2REG(1, X2IPS_MIPI_PLL_SEL_CLR_MASK, X2IPS_MIPI_PLL_SEL_CLR_OFFS);
		mipi_putreg(iomem, REG_X2IPS_MIPI_DEV_PLL_CTRL2, val);

		val = mipi_getreg(iomem, REG_X2IPS_MIPI_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X2IPS_MIPI_DEV_CFGCLK_FRANGE_MASK, X2IPS_MIPI_DEV_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, X2IPS_MIPI_DEV_CFGCLK_FRANGE_MASK, X2IPS_MIPI_DEV_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X2IPS_MIPI_DEV_HS_FRANGE_MASK, X2IPS_MIPI_DEV_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, X2IPS_MIPI_DEV_HS_FRANGE_MASK, X2IPS_MIPI_DEV_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, REG_X2IPS_MIPI_FREQRANGE, val);
	} else {
		ret = -1;
	}
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */

	mipi_dbg(param, dev, "set mipi%s%d freq region %d range %d, regv 0x%x\n",
			g_mp_type[type], port, region, value, val);
	return ret;
}

static int32_t x2ips_mipi_get_lanemode(int32_t type, int32_t port)
{
	if (((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) ||
		((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM))) {
		return 0;
	}
	return -1;
}

static int32_t x2ips_mipi_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	if ((((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) ||
		 ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM))) &&
		(lanemode == 0)) {
		return 0;
	}
	return -1;
}

static const struct mipi_dump_reg x2ips_dump_regs[] = {
	{ "MIPI_DEV_PLL_CTRL1", REG_X2IPS_MIPI_DEV_PLL_CTRL1 },
	{ "MIPI_DEV_PLL_CTRL2", REG_X2IPS_MIPI_DEV_PLL_CTRL2 },
	{ "MIPI_HOST_PLL_CTRL1", REG_X2IPS_MIPI_HOST_PLL_CTRL1 },
	{ "MIPI_HOST_PLL_CTRL2", REG_X2IPS_MIPI_HOST_PLL_CTRL2 },
	{ "MIPI_CTRL", REG_X2IPS_MIPI_CTRL },
	{ "MIPI_FREQRANGE", REG_X2IPS_MIPI_FREQRANGE },
	{ NULL, 0 },
};

static struct mipi_dphy_ops_s x2ips_dphy_ops = {
	.name = "x2ips",
	.regs = x2ips_dump_regs,
	.get_ctl = x2ips_mipi_get_ctl,
	.set_ctl = x2ips_mipi_set_ctl,
	.get_freqrange = x2ips_mipi_get_freqrange,
	.set_freqrange = x2ips_mipi_set_freqrange,
	.get_lanemode = x2ips_mipi_get_lanemode,
	.set_lanemode = x2ips_mipi_set_lanemode,
};
#elif defined CONFIG_HOBOT_XJ3
#define MIPI_HOST_HW_PORT_NUM	(4)
#define MIPI_DEV_HW_PORT_NUM	(1)
static int32_t x3vio_mipi_get_ctl(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, REG_X3VIO_MIPI_DEV_CTRL);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		ret = DP_REG2V(val, X3VIO_MIPI_BPASS_GEN_DLY_MASK, X3VIO_MIPI_BPASS_GEN_DLY_OFFS);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		ret = DP_REG2V(val, X3VIO_MIPI_BPASS_GEN_HSYNC_MASK, X3VIO_MIPI_BPASS_GEN_HSYNC_OFFS);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		ret = DP_REG2V(val, X3VIO_MIPI_DEV_SHADOW_CLR_MASK, X3VIO_MIPI_DEV_SHADOW_CLR_OFFS);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static int32_t x3vio_mipi_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret = 0;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, REG_X3VIO_MIPI_DEV_CTRL);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val &= ~DP_VMASK(X3VIO_MIPI_BPASS_GEN_DLY_MASK, X3VIO_MIPI_BPASS_GEN_DLY_OFFS);
		val |= DP_V2REG(value, X3VIO_MIPI_BPASS_GEN_DLY_MASK, X3VIO_MIPI_BPASS_GEN_DLY_OFFS);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val &= ~DP_VMASK(X3VIO_MIPI_BPASS_GEN_HSYNC_MASK, X3VIO_MIPI_BPASS_GEN_HSYNC_OFFS);
		val |= DP_V2REG(value, X3VIO_MIPI_BPASS_GEN_HSYNC_MASK, X3VIO_MIPI_BPASS_GEN_HSYNC_OFFS);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val &= ~DP_VMASK(X3VIO_MIPI_DEV_SHADOW_CLR_MASK, X3VIO_MIPI_DEV_SHADOW_CLR_OFFS);
		val |= DP_V2REG(value, X3VIO_MIPI_DEV_SHADOW_CLR_MASK, X3VIO_MIPI_DEV_SHADOW_CLR_OFFS);
		break;
	default:
		ret = -1;
		break;
	}
	mipi_putreg(iomem, REG_X3VIO_MIPI_DEV_CTRL, val);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */

	mipi_dbg(param, dev, "set mipi%s%d ctl region %d value %d, regv 0x%x, %d\n",
			g_mp_type[type], port, region, value, val, ret);
	return ret;
}

static int32_t x3vio_mipi_get_freqrange(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t reg, val;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
		case MIPI_PORT0:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX0;
			break;
		case MIPI_PORT1:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX1;
			break;
		case MIPI_PORT2:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX2;
			break;
		case MIPI_PORT3:
		default:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX3;
			break;
		}
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			ret = DP_REG2V(val, X3VIO_MIPI_RXN_CFGCLK_FRANGE_MASK, X3VIO_MIPI_RXN_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			ret = DP_REG2V(val, X3VIO_MIPI_RXN_HS_FRANGE_MASK, X3VIO_MIPI_RXN_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
	} else if (((type == MIPI_DPHY_TYPE_DEV) ||
			(type == MIPI_DPHY_TYPE_DSI)) && (port < MIPI_DEV_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_DEV_FREQRANGE);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		switch (region) {
			case MIPI_CFGCLKFREQRANGE:
				ret = DP_REG2V(val, X3VIO_MIPI_DEV_CFGCLK_FRANGE_MASK, X3VIO_MIPI_DEV_CFGCLK_FRANGE_OFFS);
				break;
			case MIPI_HSFREQRANGE:
				ret = DP_REG2V(val, X3VIO_MIPI_DEV_HS_FRANGE_MASK, X3VIO_MIPI_DEV_HS_FRANGE_OFFS);
				break;
			default:
				ret = -1;
				break;
		}
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t x3vio_mipi_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
		case MIPI_PORT0:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX0;
			break;
		case MIPI_PORT1:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX1;
			break;
		case MIPI_PORT2:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX2;
			break;
		case MIPI_PORT3:
		default:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX3;
			break;
		}
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X3VIO_MIPI_RXN_CFGCLK_FRANGE_MASK, X3VIO_MIPI_RXN_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, X3VIO_MIPI_RXN_CFGCLK_FRANGE_MASK, X3VIO_MIPI_RXN_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X3VIO_MIPI_RXN_HS_FRANGE_MASK, X3VIO_MIPI_RXN_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, X3VIO_MIPI_RXN_HS_FRANGE_MASK, X3VIO_MIPI_RXN_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else if (((type == MIPI_DPHY_TYPE_DEV) ||
			(type == MIPI_DPHY_TYPE_DSI)) && (port < MIPI_DEV_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_DEV_PLL_CTRL2);
		val &= ~DP_VMASK(X3VIO_MIPI_PLL_SEL_CLR_MASK, X3VIO_MIPI_PLL_SEL_CLR_OFFS);
		val |= DP_V2REG(1, X3VIO_MIPI_PLL_SEL_CLR_MASK, X3VIO_MIPI_PLL_SEL_CLR_OFFS);
		mipi_putreg(iomem, REG_X3VIO_MIPI_DEV_PLL_CTRL2, val);

		val = mipi_getreg(iomem, REG_X3VIO_MIPI_DEV_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X3VIO_MIPI_DEV_CFGCLK_FRANGE_MASK, X3VIO_MIPI_DEV_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, X3VIO_MIPI_DEV_CFGCLK_FRANGE_MASK, X3VIO_MIPI_DEV_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X3VIO_MIPI_DEV_HS_FRANGE_MASK, X3VIO_MIPI_DEV_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, X3VIO_MIPI_DEV_HS_FRANGE_MASK, X3VIO_MIPI_DEV_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, REG_X3VIO_MIPI_DEV_FREQRANGE, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d freq region %d range %d, regv 0x%x, %d\n",
			g_mp_type[type], port, region, value, val, ret);
	return ret;
}

static int32_t x3vio_mipi_get_lanemode(int32_t type, int32_t port)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t val;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_RX_DPHY_CTRL);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		if ((port == MIPI_PORT0) || (port == MIPI_PORT2)) {
			ret = DP_REG2V(val, X3VIO_MIPI_RX_DPHY_MODE02_MASK, X3VIO_MIPI_RX_DPHY_MODE02_OFFS);
		} else {
			ret = DP_REG2V(val, X3VIO_MIPI_RX_DPHY_MODE13_MASK, X3VIO_MIPI_RX_DPHY_MODE13_OFFS);
		}
	} else if (((type == MIPI_DPHY_TYPE_DEV) ||
			(type == MIPI_DPHY_TYPE_DSI)) && (port < MIPI_DEV_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_TX_DPHY_CTRL);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		ret = DP_REG2V(val, X3VIO_MIPI_TX_DPHY_SEL_MASK, X3VIO_MIPI_TX_DPHY_SEL_OFFS);
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t x3vio_mipi_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_RX_DPHY_CTRL);
		if ((port == MIPI_PORT0) || (port == MIPI_PORT2)) {
			val &= ~DP_VMASK(X3VIO_MIPI_RX_DPHY_MODE02_MASK, X3VIO_MIPI_RX_DPHY_MODE02_OFFS);
			val |= DP_V2REG(lanemode, X3VIO_MIPI_RX_DPHY_MODE02_MASK, X3VIO_MIPI_RX_DPHY_MODE02_OFFS);
		} else {
			val &= ~DP_VMASK(X3VIO_MIPI_RX_DPHY_MODE13_MASK, X3VIO_MIPI_RX_DPHY_MODE13_OFFS);
			val |= DP_V2REG(lanemode, X3VIO_MIPI_RX_DPHY_MODE13_MASK, X3VIO_MIPI_RX_DPHY_MODE13_OFFS);
		}
		mipi_putreg(iomem, REG_X3VIO_MIPI_RX_DPHY_CTRL, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else if (((type == MIPI_DPHY_TYPE_DEV) ||
			(type == MIPI_DPHY_TYPE_DSI)) && (port < MIPI_DEV_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, REG_X3VIO_MIPI_TX_DPHY_CTRL);
		val &= ~DP_VMASK(X3VIO_MIPI_TX_DPHY_SEL_MASK, X3VIO_MIPI_TX_DPHY_SEL_OFFS);
		val |= DP_V2REG(lanemode, X3VIO_MIPI_TX_DPHY_SEL_MASK, X3VIO_MIPI_TX_DPHY_SEL_OFFS);
		mipi_putreg(iomem, REG_X3VIO_MIPI_TX_DPHY_CTRL, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d lanemode %d, regv 0x%x, %d\n",
			g_mp_type[type], port, lanemode, val, ret);
	return ret;
}

static const struct mipi_dump_reg x3vio_dump_regs[] = {
	{ "MIPI_DEV_PLL_CTRL1", REG_X3VIO_MIPI_DEV_PLL_CTRL1 },
	{ "MIPI_DEV_PLL_CTRL2", REG_X3VIO_MIPI_DEV_PLL_CTRL2 },
	{ "MIPI_DEV_CTRL", REG_X3VIO_MIPI_DEV_CTRL },
	{ "MIPI_DEV_FREQRANGE", REG_X3VIO_MIPI_DEV_FREQRANGE },
	{ "MIPI_RX0_PLL_CTRL1", REG_X3VIO_MIPI_RX0_PLL_CTRL1 },
	{ "MIPI_RX0_PLL_CTRL2", REG_X3VIO_MIPI_RX0_PLL_CTRL2 },
	{ "MIPI_RX1_PLL_CTRL1", REG_X3VIO_MIPI_RX1_PLL_CTRL1 },
	{ "MIPI_RX1_PLL_CTRL2", REG_X3VIO_MIPI_RX1_PLL_CTRL2 },
	{ "MIPI_RX2_PLL_CTRL1", REG_X3VIO_MIPI_RX2_PLL_CTRL1 },
	{ "MIPI_RX2_PLL_CTRL2", REG_X3VIO_MIPI_RX2_PLL_CTRL2 },
	{ "MIPI_RX3_PLL_CTRL1", REG_X3VIO_MIPI_RX3_PLL_CTRL1 },
	{ "MIPI_RX3_PLL_CTRL2", REG_X3VIO_MIPI_RX3_PLL_CTRL2 },
	{ "MIPI_RX0", REG_X3VIO_MIPI_RX0 },
	{ "MIPI_FREQRANGE_RX0", REG_X3VIO_MIPI_FREQRANGE_RX0 },
	{ "MIPI_RX1", REG_X3VIO_MIPI_RX1 },
	{ "MIPI_FREQRANGE_RX1", REG_X3VIO_MIPI_FREQRANGE_RX1 },
	{ "MIPI_RX2", REG_X3VIO_MIPI_RX2 },
	{ "MIPI_FREQRANGE_RX2", REG_X3VIO_MIPI_FREQRANGE_RX2 },
	{ "MIPI_RX3", REG_X3VIO_MIPI_RX3 },
	{ "MIPI_FREQRANGE_RX3", REG_X3VIO_MIPI_FREQRANGE_RX3 },
	{ "MIPI_TX_DPHY_CTRL", REG_X3VIO_MIPI_TX_DPHY_CTRL },
	{ "MIPI_RX_DPHY_CTRL", REG_X3VIO_MIPI_RX_DPHY_CTRL },
	{ NULL, 0 },
};

static struct mipi_dphy_ops_s x3vio_dphy_ops = {
	.name = "x3vio",
	.regs = x3vio_dump_regs,
	.get_ctl = x3vio_mipi_get_ctl,
	.set_ctl = x3vio_mipi_set_ctl,
	.get_freqrange = x3vio_mipi_get_freqrange,
	.set_freqrange = x3vio_mipi_set_freqrange,
	.get_lanemode = x3vio_mipi_get_lanemode,
	.set_lanemode = x3vio_mipi_set_lanemode,
};
#elif defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
#define MIPI_HOST_HW_PORT_NUM	(4)
#define MIPI_DEV_HW_PORT_NUM	(2)
static int32_t j5sys_mipi_get_ctl(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t reg, val;
	int32_t ret;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	switch (port) {
		case MIPI_PORT0:
			reg = (uint32_t)REG_J5SYS_MIPI_DEV0_CTRL3;
			break;
		case MIPI_PORT1:
		default:
			reg = (uint32_t)REG_J5SYS_MIPI_DEV1_CTRL3;
			break;
	}
	osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
	val = mipi_getreg(iomem, reg);
	osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */

	switch (region) {
	case MIPI_DEV_SHADOW_CLEAR:
		ret = DP_REG2V(val, J5SYS_MIPI_DEV_SHADOW_CLR_MASK, J5SYS_MIPI_DEV_SHADOW_CLR_OFFS);
		break;
	case MIPI_DEV_SHADOW_CONTROL:
		ret = DP_REG2V(val, J5SYS_MIPI_DEV_SHADOW_CTL_MASK, J5SYS_MIPI_DEV_SHADOW_CTL_OFFS);
		break;
	case MIPI_DEV_SEL_CLR:
		ret = DP_REG2V(val, J5SYS_MIPI_DEV_SEL_CLR_MASK, J5SYS_MIPI_DEV_SEL_CLR_OFFS);
		break;
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
	case MIPI_BYPASS_GEN_HSYNC_EN:
		/* j5 not support */
	default:
		ret = -1;
		break;
	}

	return ret;
}

static int32_t j5sys_mipi_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	uint32_t reg, val = 0U;
	int32_t ret = 0;

	if ((iomem == NULL) || (type != MIPI_DPHY_TYPE_DEV) || (port >= MIPI_DEV_HW_PORT_NUM)) {
		return -1;
	}

	switch (port) {
		case MIPI_PORT0:
			reg = (uint32_t)REG_J5SYS_MIPI_DEV0_CTRL3;
			break;
		case MIPI_PORT1:
		default:
			reg = (uint32_t)REG_J5SYS_MIPI_DEV1_CTRL3;
			break;
	}

	switch (region) {
	case MIPI_DEV_SHADOW_CLEAR:
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		val &= ~DP_VMASK(J5SYS_MIPI_DEV_SHADOW_CLR_MASK, J5SYS_MIPI_DEV_SHADOW_CLR_OFFS);
		val |= DP_V2REG(value, J5SYS_MIPI_DEV_SHADOW_CLR_MASK, J5SYS_MIPI_DEV_SHADOW_CLR_OFFS);
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		break;
	case MIPI_DEV_SHADOW_CONTROL:
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		val &= ~DP_VMASK(J5SYS_MIPI_DEV_SHADOW_CTL_MASK, J5SYS_MIPI_DEV_SHADOW_CTL_OFFS);
		val |= DP_V2REG(value, J5SYS_MIPI_DEV_SHADOW_CTL_MASK, J5SYS_MIPI_DEV_SHADOW_CTL_OFFS);
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		break;
	case MIPI_DEV_SEL_CLR:
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		val &= ~DP_VMASK(J5SYS_MIPI_DEV_SEL_CLR_MASK, J5SYS_MIPI_DEV_SEL_CLR_OFFS);
		val |= DP_V2REG(value, J5SYS_MIPI_DEV_SEL_CLR_MASK, J5SYS_MIPI_DEV_SEL_CLR_OFFS);
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		break;
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
	case MIPI_BYPASS_GEN_HSYNC_EN:
		/* j5 not support */
	default:
		ret = -1;
		break;
	}

	mipi_dbg(param, dev, "set mipi%s%d ctl region %d value %d, regv 0x%x, %d\n",
			g_mp_type[type], port, region, value, val, ret);
	return ret;
}

static int32_t j5sys_mipi_get_freqrange_host(int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t reg, val;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	if (port < MIPI_HOST_HW_PORT_NUM) {
		uint32_t reg_host_ctrl[MIPI_HOST_HW_PORT_NUM] = {
			REG_J5SYS_MIPI_HOST0_CTRL,
			REG_J5SYS_MIPI_HOST1_CTRL,
			REG_J5SYS_MIPI_HOST2_CTRL,
			REG_J5SYS_MIPI_HOST3_CTRL,
		};
		reg = reg_host_ctrl[port];
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			ret = DP_REG2V(val, J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			ret = DP_REG2V(val, J5SYS_MIPI_HOSTN_HS_FRANGE_MASK, J5SYS_MIPI_HOSTN_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t j5sys_mipi_get_freqrange_dev(int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	mipi_flags_t flags;
	uint32_t reg, val;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	if (port < MIPI_DEV_HW_PORT_NUM) {
		uint32_t reg_dev_ctrl[MIPI_DEV_HW_PORT_NUM] = {
			REG_J5SYS_MIPI_DEV0_CTRL0,
			REG_J5SYS_MIPI_DEV1_CTRL0,
		};
		reg = reg_dev_ctrl[port];
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
		switch (region) {
			case MIPI_CFGCLKFREQRANGE:
				ret = DP_REG2V(val, J5SYS_MIPI_DEVN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_DEVN_CFGCLK_FRANGE_OFFS);
				break;
			case MIPI_HSFREQRANGE:
				ret = DP_REG2V(val, J5SYS_MIPI_DEVN_HS_FRANGE_MASK, J5SYS_MIPI_DEVN_HS_FRANGE_OFFS);
				break;
			default:
				ret = -1;
				break;
		}
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t j5sys_mipi_get_freqrange(int32_t type, int32_t port, int32_t region)
{
	int32_t ret;

	if (type == MIPI_DPHY_TYPE_HOST) {
		ret = j5sys_mipi_get_freqrange_host(port, region);
	} else if (type == MIPI_DPHY_TYPE_DEV) {
		ret = j5sys_mipi_get_freqrange_dev(port, region);
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t j5sys_mipi_set_freqrange_host(int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if (port < MIPI_HOST_HW_PORT_NUM) {
		uint32_t reg_host_ctrl[MIPI_HOST_HW_PORT_NUM] = {
			REG_J5SYS_MIPI_HOST0_CTRL,
			REG_J5SYS_MIPI_HOST1_CTRL,
			REG_J5SYS_MIPI_HOST2_CTRL,
			REG_J5SYS_MIPI_HOST3_CTRL,
		};
		reg = reg_host_ctrl[port];
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_HOSTN_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(J5SYS_MIPI_HOSTN_HS_FRANGE_MASK, J5SYS_MIPI_HOSTN_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, J5SYS_MIPI_HOSTN_HS_FRANGE_MASK, J5SYS_MIPI_HOSTN_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipihost%d freq region %d range %d, regv 0x%x, %d\n",
			port, region, value, val, ret);
	return ret;
}

static int32_t j5sys_mipi_set_freqrange_dev(int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if (port < MIPI_DEV_HW_PORT_NUM) {
		uint32_t reg_dev_ctrl0[MIPI_DEV_HW_PORT_NUM] = {
			REG_J5SYS_MIPI_DEV0_CTRL0,
			REG_J5SYS_MIPI_DEV1_CTRL0,
		};
		uint32_t reg_dev_ctrl3[MIPI_DEV_HW_PORT_NUM] = {
			REG_J5SYS_MIPI_DEV0_CTRL3,
			REG_J5SYS_MIPI_DEV1_CTRL3,
		};
		reg = reg_dev_ctrl3[port];
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		val &= ~DP_VMASK(J5SYS_MIPI_DEV_SEL_CLR_MASK, J5SYS_MIPI_DEV_SEL_CLR_OFFS);
		val |= DP_V2REG(1, J5SYS_MIPI_DEV_SEL_CLR_MASK, J5SYS_MIPI_DEV_SEL_CLR_OFFS);
		val &= ~DP_VMASK(J5SYS_MIPI_DEV_SHADOW_CTL_MASK, J5SYS_MIPI_DEV_SHADOW_CTL_OFFS);
		val |= DP_V2REG(1, J5SYS_MIPI_DEV_SHADOW_CTL_MASK, J5SYS_MIPI_DEV_SHADOW_CTL_OFFS);
		mipi_putreg(iomem, reg, val);

		reg = reg_dev_ctrl0[port];
		val = mipi_getreg(iomem, reg);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(J5SYS_MIPI_DEVN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_DEVN_CFGCLK_FRANGE_OFFS);
			val |= DP_V2REG(value, J5SYS_MIPI_DEVN_CFGCLK_FRANGE_MASK, J5SYS_MIPI_DEVN_CFGCLK_FRANGE_OFFS);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(J5SYS_MIPI_DEVN_HS_FRANGE_MASK, J5SYS_MIPI_DEVN_HS_FRANGE_OFFS);
			val |= DP_V2REG(value, J5SYS_MIPI_DEVN_HS_FRANGE_MASK, J5SYS_MIPI_DEVN_HS_FRANGE_OFFS);
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipidev%d freq region %d range %d, regv 0x%x, %d\n",
			port, region, value, val, ret);
	return ret;
}

static int32_t j5sys_mipi_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value)
{
	int32_t ret;

	if (type == MIPI_DPHY_TYPE_HOST) {
		ret = j5sys_mipi_set_freqrange_host(port, region,value);
	} else if (type == MIPI_DPHY_TYPE_DEV) {
		ret = j5sys_mipi_set_freqrange_dev(port, region,value);
	} else {
		ret = -1;
	}

	return ret;
}

static int32_t j5sys_mipi_get_lanemode(int32_t type, int32_t port)
{
	if (((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) ||
		((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM))) {
		return 0;
	}
	return -1;
}

static int32_t j5sys_mipi_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	if ((((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) ||
		 ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_HW_PORT_NUM))) &&
		(lanemode == 0)) {
		return 0;
	}
	return -1;
}

static const struct mipi_dump_reg j5sys_dump_regs[] = {
	{ "MIPI_HOST0_CTRL", REG_J5SYS_MIPI_HOST0_CTRL },
	{ "MIPI_HOST1_CTRL", REG_J5SYS_MIPI_HOST1_CTRL },
	{ "MIPI_HOST2_CTRL", REG_J5SYS_MIPI_HOST2_CTRL },
	{ "MIPI_HOST3_CTRL", REG_J5SYS_MIPI_HOST3_CTRL },
	{ "MIPI_HOST0_CTRL1", REG_J5SYS_MIPI_HOST0_CTRL1 },
	{ "MIPI_HOST1_CTRL1", REG_J5SYS_MIPI_HOST1_CTRL1 },
	{ "MIPI_HOST2_CTRL1", REG_J5SYS_MIPI_HOST2_CTRL1 },
	{ "MIPI_HOST3_CTRL1", REG_J5SYS_MIPI_HOST3_CTRL1 },
	{ "MIPI_RX_DEBUG", REG_J5SYS_MIPI_RX_DEBUG },
	{ "MIPI_DEV0_CTRL0", REG_J5SYS_MIPI_DEV0_CTRL0 },
	{ "MIPI_DEV0_CTRL1", REG_J5SYS_MIPI_DEV0_CTRL1 },
	{ "MIPI_DEV0_CTRL2", REG_J5SYS_MIPI_DEV0_CTRL2 },
	{ "MIPI_DEV0_CTRL3", REG_J5SYS_MIPI_DEV0_CTRL3 },
	{ "MIPI_DEV1_CTRL0", REG_J5SYS_MIPI_DEV1_CTRL0 },
	{ "MIPI_DEV1_CTRL1", REG_J5SYS_MIPI_DEV1_CTRL1 },
	{ "MIPI_DEV1_CTRL2", REG_J5SYS_MIPI_DEV1_CTRL2 },
	{ "MIPI_DEV1_CTRL3", REG_J5SYS_MIPI_DEV1_CTRL3 },
	{ "MIPI_TX_DEBUG", REG_J5SYS_MIPI_TX_DEBUG },
	{ NULL, 0 },
};

static struct mipi_dphy_ops_s j5sys_dphy_ops = {
	.name = "j5sys",
	.regs = j5sys_dump_regs,
	.get_ctl = j5sys_mipi_get_ctl,
	.set_ctl = j5sys_mipi_set_ctl,
	.get_freqrange = j5sys_mipi_get_freqrange,
	.set_freqrange = j5sys_mipi_set_freqrange,
	.get_lanemode = j5sys_mipi_get_lanemode,
	.set_lanemode = j5sys_mipi_set_lanemode,
};
#elif defined X5_CHIP
#define MIPI_HOST_HW_PORT_NUM	(4)
#define MIPI_DEV_HW_PORT_NUM	(1)
static int32_t x5sys_mipi_set_cfg(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
		case MIPI_PORT0:
		case MIPI_PORT1:
		default:
			reg = REG_X5SYS_MIPI_PHY_CFG0;
			break;
		case MIPI_PORT2:
		case MIPI_PORT3:
			reg = REG_X5SYS_MIPI_PHY_CFG1;
			break;
		}
		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		switch (region) {
		case MIPI_PHY_ENABLE_CLK:
			if (port == MIPI_PORT0 || port == MIPI_PORT2) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_MASK, REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_MASK, REG_X5SYS_MIPI_PHY_PORT02_ENABLE_CLK_OFFS); //set bit
			} else if (port == MIPI_PORT1 || port == MIPI_PORT3) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_MASK, REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_MASK, REG_X5SYS_MIPI_PHY_PORT13_ENABLE_CLK_OFFS); //set bit
			}
			break;
		case MIPI_PHY_FORCE_RXMODE:
                       if (port == MIPI_PORT0 || port == MIPI_PORT2) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_MASK, REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_MASK, REG_X5SYS_MIPI_PHY_PORT02_FORCE_RXMODE_OFFS); //set bit
			} else if (port == MIPI_PORT1 || port == MIPI_PORT3) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_MASK, REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_MASK, REG_X5SYS_MIPI_PHY_PORT13_FORCE_RXMODE_OFFS); //set bit
			}
			break;
		case MIPI_PHY_HS_FREQRANGE:
			if (port == MIPI_PORT0 || port == MIPI_PORT2) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_MASK, REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_MASK, REG_X5SYS_MIPI_PHY_PORT02_HS_FRANGE_OFFS); //set bit
			} else if (port == MIPI_PORT1 || port == MIPI_PORT3) {
				val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_MASK, REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_OFFS); //clean bit
				val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_MASK, REG_X5SYS_MIPI_PHY_PORT13_HS_FRANGE_OFFS); //set bit
			}
			break;
		case MIPI_PHY_CTRL_SEL:
			val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_CTRL_MASK, REG_X5SYS_MIPI_PHY_CTRL_OFFS);	//clean bit
			val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_CTRL_MASK, REG_X5SYS_MIPI_PHY_CTRL_OFFS); //set bit
			break;
                case MIPI_CFG_CLK_FREQRANGE:
			reg = REG_X5SYS_MIPI_PHY_CFG2;
			val = mipi_getreg(iomem, reg);
			val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_CFG_CLK_MASK, REG_X5SYS_MIPI_PHY_CFG_CLK_OFFS); //clean bit
			val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_CFG_CLK_MASK, REG_X5SYS_MIPI_PHY_CFG_CLK_OFFS); //set bit
			break;
		default:
			ret = -1;
			break;
		}
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_info(dev, "set mipi%s%d freq region %d range %x, reg=0x%x,val=0x%x, %d\n",
			g_mp_type[type], port, region, value, reg, val, ret);
	return ret;
}

static int32_t x5sys_mipi_set_testcode(int32_t type, int32_t port, int32_t code)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
		case MIPI_PORT0:
		case MIPI_PORT1:
		default:
			reg = REG_X5SYS_MIPI_PHY_TESTCODE0;
			break;
		case MIPI_PORT2:
		case MIPI_PORT3:
			reg = REG_X5SYS_MIPI_PHY_TESTCODE1;
			break;
		}

		osal_spin_lock_irqsave(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_lock_irqsave macro */
		val = mipi_getreg(iomem, reg);
		if (port == MIPI_PORT1 || port == MIPI_PORT3) {
			val = (code << 12) | (val & 0xfff);
		} else if (port == MIPI_PORT0 || port == MIPI_PORT2) {
			val = (code & 0xfff) | (val & 0xfff000);
		}
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags); /* PRQA S 2996 */ /* osal_spin_unlock_irqrestore macro */
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d testcode 0x%x = 0x%x, ret = %d\n",
			g_mp_type[type], port, reg, val, ret);
	return ret;

}

static int32_t x5sys_mipi_get_lanemode(int32_t type, int32_t port)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	mipi_flags_t flags;
	uint32_t reg, val = 0U;
	int32_t ret;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
		case MIPI_PORT0:
		case MIPI_PORT1:
		default:
			reg = REG_X5SYS_MIPI_PHY_CFG0;
			break;
		case MIPI_PORT2:
		case MIPI_PORT3:
			reg = REG_X5SYS_MIPI_PHY_CFG1;
			break;
		}

		osal_spin_lock_irqsave(&dphy->lock, &flags);
		val = mipi_getreg(iomem, reg);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags);

		ret = DP_REG2V(val, REG_X5SYS_MIPI_PHY_CTRL_MASK, REG_X5SYS_MIPI_PHY_CTRL_OFFS);
	} else {
		ret = -1;
	}

	mipi_info(dev, "get mipi%s%d lanemode 0x%x = %d\n",
			g_mp_type[type], port, val, ret);
	return ret;
}

static int32_t x5sys_mipi_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t reg, val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		switch (port) {
			case MIPI_PORT0:
			case MIPI_PORT1:
			default:
				reg = REG_X5SYS_MIPI_PHY_CFG0;
				break;
			case MIPI_PORT2:
			case MIPI_PORT3:
				reg = REG_X5SYS_MIPI_PHY_CFG1;
				break;
		}

		osal_spin_lock_irqsave(&dphy->lock, &flags);
		val = mipi_getreg(iomem, reg);
		val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_CTRL_MASK, REG_X5SYS_MIPI_PHY_CTRL_OFFS);	//clean bit
		val |= DP_V2REG(lanemode, REG_X5SYS_MIPI_PHY_CTRL_MASK, REG_X5SYS_MIPI_PHY_CTRL_OFFS); //set bit
		mipi_putreg(iomem, reg, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags);
	} else {
		ret = -1;
	}

	mipi_info(dev, "set mipi%s%d lanemode %d, regv 0x%x, %d\n",
			g_mp_type[type], port, lanemode, val, ret);
	return ret;
}

static int32_t x5sys_mipi_set_test_sel(int32_t type, int32_t port, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	mipi_flags_t flags;
	int32_t ret = 0;
	uint32_t val = 0U;

	if (iomem == NULL) {
		return -1;
	}

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_HW_PORT_NUM)) {
		osal_spin_lock_irqsave(&dphy->lock, &flags);
		val = mipi_getreg(iomem, REG_X5SYS_MIPI_PHY_CFG2);
		if (port == MIPI_PORT0 || port == MIPI_PORT1) {
			val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_TEST_SEL_4L_MASK, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_0_OFFS);	//clean bit
			val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_MASK, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_0_OFFS); //set bit
		} else if (port == MIPI_PORT2 || port == MIPI_PORT3) {
			val &= ~DP_VMASK(REG_X5SYS_MIPI_PHY_TEST_SEL_4L_MASK, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_1_OFFS);	//clean bit
			val |= DP_V2REG(value, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_MASK, REG_X5SYS_MIPI_PHY_TEST_SEL_4L_1_OFFS); //set bit
		}
		mipi_putreg(iomem, REG_X5SYS_MIPI_PHY_CFG2, val);
		osal_spin_unlock_irqrestore(&dphy->lock, &flags);
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d test_sel_4l %d\n",
			g_mp_type[type], port, value);
	return ret;
}

static struct mipi_dphy_ops_s x5sys_dphy_ops = {
	.name = "x5sys",
	.set_freqrange = x5sys_mipi_set_cfg,
	.set_testcode = x5sys_mipi_set_testcode,
	.get_lanemode = x5sys_mipi_get_lanemode,
	.set_lanemode = x5sys_mipi_set_lanemode,
	.set_test_sel = x5sys_mipi_set_test_sel,
};
#endif

#define DUMMY_REGION_SIZE	(4)
enum {
	DUMMY_TYPE_CTRL = 0,
	DUMMY_TYPE_FREQRANGE,
	DUMMY_TYPE_LANEMODE,
	DUMMY_TYPE_MAX,
};
struct dummy_region_s {
	int32_t region[DUMMY_REGION_SIZE];
};
static struct dummy_region_s region_host_dummy[MIPI_HOST_MAX_NUM][DUMMY_TYPE_MAX] = { 0 };
static struct dummy_region_s region_dev_dummy[MIPI_DEV_MAX_NUM][DUMMY_TYPE_MAX] = { 0 };

static int32_t dummy_mipi_get_ctl(int32_t type, int32_t port, int32_t region)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t val;

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM) &&
		(region < DUMMY_REGION_SIZE)) {
		val = region_host_dummy[port][DUMMY_TYPE_CTRL].region[region];
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM) &&
			   (region < DUMMY_REGION_SIZE)) {
		val = region_dev_dummy[port][0].region[region];
	} else {
		val = -1;
	}

	mipi_dbg(param, dev, "get mipi%s%d ctl region %d value %d, dummy\n",
			g_mp_type[type], port, region, val);
	return val;
}

static int32_t dummy_mipi_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t ret = 0;

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM) &&
		(region < DUMMY_REGION_SIZE)) {
		region_host_dummy[port][DUMMY_TYPE_CTRL].region[region] = value;
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM) &&
			   (region < DUMMY_REGION_SIZE)) {
		region_dev_dummy[port][DUMMY_TYPE_CTRL].region[region] = value;
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d ctl region %d value %d, dummy %d\n",
			g_mp_type[type], port, region, value, ret);
	return ret;
}

static int32_t dummy_mipi_get_freqrange(int32_t type, int32_t port, int32_t region)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t val;

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM) &&
		(region < DUMMY_REGION_SIZE)) {
		val = region_host_dummy[port][DUMMY_TYPE_FREQRANGE].region[region];
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM) &&
			   (region < DUMMY_REGION_SIZE)) {
		val = region_dev_dummy[port][DUMMY_TYPE_FREQRANGE].region[region];
	} else {
		val = -1;
	}

	mipi_dbg(param, dev, "get mipi%s%d freq region %d value %d, dummy\n",
			g_mp_type[type], port, region, val);
	return val;
}

static int32_t dummy_mipi_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t ret = 0;


	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM) &&
		(region < DUMMY_REGION_SIZE)) {
		region_host_dummy[port][DUMMY_TYPE_FREQRANGE].region[region] = value;
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM) &&
			   (region < DUMMY_REGION_SIZE)) {
		region_dev_dummy[port][DUMMY_TYPE_FREQRANGE].region[region] = value;
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d freq region %d value %d, dummy %d\n",
			g_mp_type[type], port, region, value, ret);
	return ret;
}

static int32_t dummy_mipi_get_lanemode(int32_t type, int32_t port)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t val;

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM)) {
		val = region_host_dummy[port][DUMMY_TYPE_LANEMODE].region[0];
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM)) {
		val = region_dev_dummy[port][DUMMY_TYPE_LANEMODE].region[0];
	} else {
		val = -1;
	}

	mipi_dbg(param, dev, "get mipi%s%d lanemode region %d value %d, dummy\n",
			g_mp_type[type], port, DUMMY_TYPE_LANEMODE, val);
	return val;
}

static int32_t dummy_mipi_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	struct os_dev *dev = &g_pdev->osdev;
	struct mipi_phy_param_s *param = &g_pdev->dphy.param;
	int32_t ret = 0;

	if ((type == MIPI_DPHY_TYPE_HOST) && (port < MIPI_HOST_MAX_NUM)) {
		region_host_dummy[port][DUMMY_TYPE_LANEMODE].region[0] = lanemode;
	} else if ((type == MIPI_DPHY_TYPE_DEV) && (port < MIPI_DEV_MAX_NUM)) {
		region_dev_dummy[port][DUMMY_TYPE_LANEMODE].region[0] = lanemode;
	} else {
		ret = -1;
	}

	mipi_dbg(param, dev, "set mipi%s%d lanemode region %d value %d, dummy %d\n",
			g_mp_type[type], port, DUMMY_TYPE_LANEMODE, lanemode, ret);
	return ret;
}

static void mipi_dphy_ops_init(struct mipi_pdev_s *pdev)
{
#if defined CONFIG_HOBOT_XJ2
	pdev->ops = &x2ips_dphy_ops;
#elif defined CONFIG_HOBOT_XJ3
	pdev->ops = &x3vio_dphy_ops;
#elif defined X5_CHIP
	pdev->ops = &x5sys_dphy_ops;
#elif defined CONFIG_HOBOT_J5 || defined CONFIG_HOBOT_FPGA_J5
	pdev->ops = &j5sys_dphy_ops;
#endif
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy get region value of ctrl reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 * @param[in] region: reg region select
 *
 * @return value of reg region
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t mipi_dphy_get_ctl(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->get_ctl == NULL)) {
		return dummy_mipi_get_ctl(type, port, region);
	} else {
		return ops->get_ctl(type, port, region);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy set region value of ctrl reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 * @param[in] region: reg region select
 * @param[in] value: reg region value to set
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
int32_t mipi_dphy_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->set_ctl == NULL)) {
		return dummy_mipi_set_ctl(type, port, region, value);
	} else {
		return ops->set_ctl(type, port, region, value);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy get region value of freqrange reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 * @param[in] region: reg region select
 *
 * @return value of reg region
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t mipi_dphy_get_freqrange(int32_t type, int32_t port, int32_t region)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->get_freqrange == NULL)) {
		return dummy_mipi_get_freqrange(type, port, region);
	} else {
		return ops->get_freqrange(type, port, region);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy set region value of freqrange reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 * @param[in] region: reg region select
 * @param[in] value: reg region value to set
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
int32_t mipi_dphy_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->set_freqrange == NULL)) {
		return dummy_mipi_set_freqrange(type, port, region, value);
	} else {
		return ops->set_freqrange(type, port, region, value);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy get region value of lanemode reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 *
 * @return value of reg region
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t mipi_dphy_get_lanemode(int32_t type, int32_t port)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->get_lanemode == NULL)) {
		return dummy_mipi_get_lanemode(type, port);
	} else {
		return ops->get_lanemode(type, port);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi dphy set region value of lanemode reg
 *
 * @param[in] type: controller type
 * @param[in] port: controller port index
 * @param[in] value: reg region value to set
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
int32_t mipi_dphy_set_lanemode(int32_t type, int32_t port, int32_t lanemode)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->set_lanemode == NULL)) {
		return dummy_mipi_set_lanemode(type, port, lanemode);
	} else {
		return ops->set_lanemode(type, port, lanemode);
	}
}

int32_t mipi_dphy_set_testcode(int32_t type, int32_t port, int32_t code)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->set_testcode == NULL)) {
		mipi_err(NULL, "mipi_dphy_set_testcode dummy \n");
		return 0;
	} else {
		return ops->set_testcode(type, port, code);
	}
}

int32_t mipi_dphy_set_test_sel_4l(int32_t type, int32_t port, int32_t value)
{
	struct mipi_dphy_s *dphy = &g_pdev->dphy;
	void __iomem *iomem = dphy->iomem;
	struct mipi_dphy_ops_s *ops = g_pdev->ops;

	if ((iomem == NULL) || (ops == NULL) || (ops->set_test_sel == NULL)) {
		mipi_err(NULL, "mipi_dphy_set_testcode dummy \n");
		return 0;
	} else {
		return ops->set_test_sel(type, port, value);
	}
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi mclk output config
 *
 * @param[in] name: clock name string of mclk
 * @param[in] freq_set: clock frequency Hz to set
 * @param[in] freq_get: clock frequency Hz to readback
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
int32_t mipi_dphy_outclk_config(const char *name, uint64_t freq_set, uint64_t *freq_get)
{
	uint64_t freq = 0;
	int32_t ret = -1;

	if ((name == NULL) || ((freq_set == 0UL) && (freq_get == NULL))) {
		return -1;
	}

#if MIPI_DPHY_CLK_24M_OUT_J5
	if (g_pdev->dphy.outclk.iomem != NULL) {
		struct mipi_dphy_outclk_s *outclk = &g_pdev->dphy.outclk;
		struct os_dev *dev = &g_pdev->osdev;
		struct mipi_phy_param_s *param = &g_pdev->dphy.param;
		uint32_t ctrl0, ctrl1;

		if (freq_set != 0UL) {
			/* set out clk freq */
			if (freq_set != MIPI_DPHY_CLK_24M_SEL_24M) {
				if ((MIPI_DPHY_CLK_24M_SEL_ARMPLL1 % freq_set) < (MIPI_DPHY_CLK_24M_SEL_PERIPLL2 % freq_set)) {
					ctrl0 = mipi_dphy_pinctrl0_sel(MIPI_DPHY_PINREG_CTRL0_SEL_ARMPLL1);
					ctrl1 = (uint32_t)((MIPI_DPHY_CLK_24M_SEL_ARMPLL1 / freq_set) - 1U); /* qacfix: conversion */
					if (ctrl1 > MIPI_DPHY_PINREG_CTRL1_PLL1_MASK)
						ctrl1 = MIPI_DPHY_PINREG_CTRL1_PLL1_MASK;
					ctrl1 = (uint32_t)((ctrl1 << MIPI_DPHY_PINREG_CTRL1_PLL1_OFFS) | MIPI_DPHY_PINREG_CTRL1_PLL2_DFT); /* qacfix: conversion */
				} else {
					ctrl0 = mipi_dphy_pinctrl0_sel(MIPI_DPHY_PINREG_CTRL0_SEL_PERIPLL2);
					ctrl1 = (uint32_t)((MIPI_DPHY_CLK_24M_SEL_PERIPLL2 / freq_set) - 1U); /* qacfix: conversion */
					if (ctrl1 > MIPI_DPHY_PINREG_CTRL1_PLL2_MASK)
						ctrl1 = MIPI_DPHY_PINREG_CTRL1_PLL2_MASK;
					ctrl1 = (uint32_t)((ctrl1 << MIPI_DPHY_PINREG_CTRL1_PLL2_OFFS) | MIPI_DPHY_PINREG_CTRL1_PLL1_DFT); /* qacfix: conversion */
				}
			} else {
				ctrl0 = mipi_dphy_pinctrl0_sel(MIPI_DPHY_PINREG_CTRL0_SEL_24M);
				ctrl1 = (uint32_t)(MIPI_DPHY_PINREG_CTRL1_PLL1_DFT | MIPI_DPHY_PINREG_CTRL1_PLL2_DFT); /* qacfix: conversion */
			}
			mipi_putreg(outclk->iomem, REG_MIPI_DPHY_PINREG_CTRL0, ctrl0);
			mipi_putreg(outclk->iomem, REG_MIPI_DPHY_PINREG_CTRL1, ctrl1);
		} else {
			/* get out clk freq */
			ctrl0 = mipi_getreg(outclk->iomem, REG_MIPI_DPHY_PINREG_CTRL0);
			ctrl1 = mipi_getreg(outclk->iomem, REG_MIPI_DPHY_PINREG_CTRL1);
		}


		/* update freq */
		switch  ((ctrl0 & MIPI_DPHY_PINREG_CTRL0_SEL_MASK) >> MIPI_DPHY_PINREG_CTRL0_SEL_OFFS) {
			case MIPI_DPHY_PINREG_CTRL0_SEL_24M:
				freq = MIPI_DPHY_CLK_24M_SEL_24M;
				break;
			case MIPI_DPHY_PINREG_CTRL0_SEL_ARMPLL1:
				freq = MIPI_DPHY_CLK_24M_SEL_ARMPLL1 /
					(uint32_t)(((ctrl1 >> MIPI_DPHY_PINREG_CTRL1_PLL1_OFFS) & MIPI_DPHY_PINREG_CTRL1_PLL1_MASK) + 1U); /* qacfix: conversion */
				break;
			case MIPI_DPHY_PINREG_CTRL0_SEL_PERIPLL2:
				freq = MIPI_DPHY_CLK_24M_SEL_PERIPLL2 /
					(uint32_t)(((ctrl1 >> MIPI_DPHY_PINREG_CTRL1_PLL2_OFFS) & MIPI_DPHY_PINREG_CTRL1_PLL2_MASK) + 1U); /* qacfix: conversion */
				break;
			default:
				freq = 0;
				break;
		}

		mipi_dbg(param, dev, "%s: ctrl0=0x%x ctrl1=0x%x freq=%lld\n", name, ctrl0, ctrl1, freq);
		outclk->freq = freq;
		ret = 0;
	}
#else
	if (freq_set == 0UL) {
		ret = 0;
	}
#endif

	if (((ret == 0) || (freq_set == 0U)) && (freq_get != NULL)) {
		*freq_get = freq;
	}

	return ret;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy ioctl operation
 *
 * @param[in] pdev: mipi phy device struct
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
int32_t hobot_mipi_phy_ioctl_do(struct mipi_pdev_s *pdev, uint32_t cmd, mipi_ioc_arg_t arg)
{
	int32_t ret = 0;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	struct os_dev *osdev = &g_pdev->osdev;
	struct mipi_phy_s *phy;
	void __iomem  *iomem;
	reg_t reg;
    int32_t type, port;
	uint32_t offset, regv;
#endif

	/* Check type and command number */
	if ((pdev == NULL) || ((char)(_IOC_TYPE(cmd)) != MIPIDPHYIOC_MAGIC)) {
		return -ENOTTY;
	}

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	switch (cmd) {
	case MIPIDPHYTIOC_READ:
		{
			if (arg == 0U) {
				mipi_err(dev, "reg read error, reg should not be NULL\n");
				return -EINVAL;
			}
			if (osap_copy_from_app((void *)&reg, (void __user *)arg, sizeof(reg)) != 0U) {
				mipi_err(dev, "reg read error, copy data from user failed\n\n");
				return -EFAULT;
			}
			offset = (uint32_t)(reg.offset);
			port = (int32_t)(reg.port);
			type = (int32_t)(reg.type);
			phy = mipi_dphy_get_phy_byport(type, port);
			if ((phy == NULL) || (phy->sub.iomem == NULL)) {
				mipi_err(dev, "reg read error, %s%d not registered\n\n",
						(type == MIPI_DPHY_TYPE_DEV) ? "dev" : "host", port);
				return -ENODEV;
			}
			iomem = phy->sub.iomem;
			reg.value = mipi_getreg(iomem, offset);
			if (osal_copy_to_app((void __user *)arg, (void *)&reg, sizeof(reg)) != 0U) {
				mipi_err(dev, "reg read error, copy data to user failed\n\n");
				return -EFAULT;
			}
		}
		break;
	case MIPIDPHYTIOC_WRITE:
		{
			if (arg == 0U) {
				mipi_err(dev, "reg write error, reg should not be NULL\n");
				return -EINVAL;
			}
			if (osal_copy_from_app((void *)&reg, (void __user *)arg, sizeof(reg)) != 0U) {
				mipi_err(dev, "reg write error, copy data from user failed\n");
				return -EFAULT;
			}
			offset = (uint32_t)(reg.offset);
			port = (int32_t)(reg.port);
			type = (int32_t)(reg.type);
			phy = mipi_dphy_get_phy_byport(type, port);
			if ((phy == NULL) || (phy->sub.iomem == NULL)) {
				mipi_err(dev, "reg write error, %s%d not registered\n\n",
						(type == MIPI_DPHY_TYPE_DEV) ? "dev" : "host", port);
				return -ENODEV;
			}
			iomem = phy->sub.iomem;
			mipi_putreg(iomem, offset, reg.value);
			regv = mipi_getreg(iomem, offset);
			if (regv != reg.value) {
				mipi_err(dev, "reg write error, write 0x%x got 0x%x\n", reg.value, regv);
				return -EFAULT;
			}
		}
		break;
	default:
		ret = -ERANGE;
		break;
	}
#endif

	return ret;
}

/* get index of mipi phy devices' param/ */
static int32_t mipi_phy_param_idx(const char *name)
{
	int32_t i;

	for (i = 0; i < MIPI_PHY_PARAMS_NUM; i++) {
		if (strcmp(g_mp_param_names[i], name) == 0) {
			return i;
		}
	}

	return -1;
}

/* sprintf show for mipi phy devices' param/ */
int32_t mipi_phy_param_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	uint32_t *param;
	char *s = buf;
	int32_t idx, l = 0;

	if ((pdev == NULL) || (s == NULL)) {
		return -EFAULT;
	}
	param = (uint32_t *)((void *)(&pdev->dphy.param));

	idx = mipi_phy_param_idx(name);
	if ((idx >= 0) && (idx < MIPI_PHY_PARAMS_NUM)) {
		l += sprintf(&s[l], "%u\n", param[idx]);
	}

	return l;
}

/* string store for mipi phy devices' param/ */
int32_t mipi_phy_param_store_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	uint32_t *param;
	int32_t ret, error = -EINVAL;
	int32_t idx;
	uint32_t val;

	if ((pdev == NULL) || (buf == NULL)) {
		return -EFAULT;
	}
	param = (uint32_t *)((void *)(&pdev->dphy.param));

	idx = mipi_phy_param_idx(name);
	if ((idx >= 0) && (idx < MIPI_PHY_PARAMS_NUM)) {
		ret = kstrtouint(buf, 0, &val);
		if (ret == 0) {
			param[idx] = val;
			error = 0;
		}
	}

	return ((error != 0) ? error : count);
}

/* sprintf show for mipi phy devices' status/info */
int32_t mipi_phy_status_info_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_phy_s *phy;
	char *s = buf;
	int32_t i, l = 0;

	if ((pdev == NULL) || (s == NULL)) {
		return -EFAULT;
	}

	l = sprintf(&s[l], "%-15s: %d\n", "host", host_num);
	for (i = 0; i < MIPI_HOST_MAX_NUM; i++) {
		phy = &pdev->phy_host[i];
		if (phy->reged != 0U) {
			l += sprintf(&s[l], "%-15s: mipi_host%d\n", "____", i);
		}
	}
	l += sprintf(&s[l], "%-15s: %d\n", "dev", dev_num);
	for (i = 0; i < MIPI_DEV_MAX_NUM; i++) {
		phy = &pdev->phy_dev[i];
		if (phy->reged != 0U) {
			l += sprintf(&s[l], "%-15s: mipi_dev%d\n", "____", i);
		}
	}

	l += sprintf(&s[l], "%-15s: %s\n", "ops", pdev->ops->name);
	l += sprintf(&s[l], "%-15s: 0x%08x +0x%x\n", "reg",
		pdev->dphy.reg.base, pdev->dphy.reg.size);
	l += sprintf(&s[l], "%-15s: %p\n", "iomem", pdev->dphy.iomem);
#if MIPI_DPHY_CLK_24M_OUT_J5
	if (pdev->dphy.outclk.iomem != NULL) {
		l += sprintf(&s[l], "%-15s: supported\n", "outclk");
		l += sprintf(&s[l], "%-15s: 0x%08x +0x%x\n", "reg",
			pdev->dphy.outclk.reg.base, pdev->dphy.outclk.reg.size);
		l += sprintf(&s[l], "%-15s: %p\n", "iomem", pdev->dphy.outclk.iomem);
		l += sprintf(&s[l], "%-15s: %llu\n", "freq", pdev->dphy.outclk.freq);
	}
#endif
#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	if (pdev->dphy.erm.iomem != NULL) {
		l += sprintf(&s[l], "%-15s: supported\n", "ermstl");
		l += sprintf(&s[l], "%-15s: 0x%08x +0x%x\n", "reg",
			pdev->dphy.erm.reg.base, pdev->dphy.erm.reg.size);
		l += sprintf(&s[l], "%-15s: %p\n", "iomem", pdev->dphy.erm.iomem);
		l += mipi_csi_stl_info(&pdev->dphy.erm.stl, &s[l]);
	}
#endif
	return l;
}

/* sprintf show for mipi phy devices' status/host */
int32_t mipi_phy_status_host_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_phy_s *phy;
	const struct pll_sel_table_s *pll_sel;
	char *s = buf;
	int32_t i, l = 0;

	if ((pdev == NULL) || (s == NULL)) {
		return -EFAULT;
	}

	for (i = 0; i < MIPI_HOST_MAX_NUM; i++) {
		phy = &pdev->phy_host[i];
		if (phy->reged == 0U) {
			continue;
		}
		l += sprintf(&s[l], "%-14s%d: --------\n", "host", i);
		l += sprintf(&s[l], "%-15s: %p\n", "iomem", phy->sub.iomem);
		if (phy->init_cnt > 0U) {
			l += sprintf(&s[l], "%-15s: %u\n", "lane", phy->lane);
			l += sprintf(&s[l], "%-15s: %u Mbps\n", "mipiclk", phy->mipiclk);
			l += sprintf(&s[l], "%-15s: %u\n", "settle", phy->settle);
			pll_sel = (const struct pll_sel_table_s *)phy->pll_sel;
			if (pll_sel != NULL) {
				l += sprintf(&s[l], "%-15s: %u Mbps\n", "laneclk", pll_sel->freq);
				l += sprintf(&s[l], "%-15s: %u\n", "osc_freq", pll_sel->osc_freq);
				l += sprintf(&s[l], "%-15s: %u\n", "range", pll_sel->value);
			}
			l += sprintf(&s[l], "%-15s: %u\n", "init_cnt", phy->init_cnt);
			l += sprintf(&s[l], "%-15s: %u\n", "reset_cnt", phy->reset_cnt);
		}
	}
	return l;
}

/* sprintf show for mipi phy devices' status/dev */
int32_t mipi_phy_status_dev_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	struct mipi_phy_s *phy;
	const struct pll_sel_table_s *pll_sel;
	struct mipi_dphy_tx_param_s *ptx;
	char *s = buf;
	int32_t i, l = 0;

	if ((pdev == NULL) || (s == NULL)) {
		return -EFAULT;
	}

	for (i = 0; i < MIPI_DEV_MAX_NUM; i++) {
		phy = &pdev->phy_dev[i];
		if (phy->reged == 0U) {
			continue;
		}
		l += sprintf(&s[l], "%-14s%d: --------\n", "dev", i);
		l += sprintf(&s[l], "%-15s: %p\n", "iomem", phy->sub.iomem);
		if (phy->init_cnt > 0U) {
			l += sprintf(&s[l], "%-15s: %u\n", "lane", phy->lane);
			l += sprintf(&s[l], "%-15s: %u Mbps\n", "mipiclk", phy->mipiclk);
			l += sprintf(&s[l], "%-15s: %u\n", "settle", phy->settle);
			pll_sel = (const struct pll_sel_table_s *)phy->pll_sel;
			if (pll_sel != NULL) {
				l += sprintf(&s[l], "%-15s: %u Mbps\n", "laneclk", pll_sel->freq);
				l += sprintf(&s[l], "%-15s: %u\n", "osc_freq", pll_sel->osc_freq);
				l += sprintf(&s[l], "%-15s: %u\n", "range", pll_sel->value);
			}
			l += sprintf(&s[l], "%-15s: %u\n", "init_cnt", phy->init_cnt);
			l += sprintf(&s[l], "%-15s: %u\n", "reset_cnt", phy->reset_cnt);
		}
		ptx = (struct mipi_dphy_tx_param_s *)phy->sub.param;
		if (ptx != NULL) {
			if (ptx->txout_param_valid != 0U) {
				l += sprintf(&s[l], "%-15s: %s\n", "txout_param_valid", "dev");
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_autolarge_enbale",
						ptx->txout_freq_autolarge_enbale);
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_gain_precent",
						ptx->txout_freq_gain_precent);
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_force",
						ptx->txout_freq_force);
			} else {
				l += sprintf(&s[l], "%-15s: %s\n", "txout_param_valid", "dphy");
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_autolarge_enbale",
						txout_freq_autolarge_enbale);
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_gain_precent",
						txout_freq_gain_precent);
				l += sprintf(&s[l], "%-15s: %u\n", "txout_freq_force",
						txout_freq_force);
			}
		}
	}
	return l;
}

int32_t mipi_phy_status_regs_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	const struct mipi_dump_reg *regs;
	void __iomem *iomem;
	char *s = buf;
	int32_t l = 0;

	if ((pdev == NULL) || (pdev->ops == NULL) || (s == NULL)) {
		return -EFAULT;
	}
	regs = pdev->ops->regs;

	if ((regs == NULL) || (regs->name == NULL)) {
		l += snprintf(&s[l], (count - l), "no op dump regs\n" );
		return l;
	}
	iomem = pdev->dphy.iomem;

	if (iomem == NULL) {
		l += snprintf(&s[l], (count - l), "not ioremap\n" );
		return l;
	}

	l = snprintf(&s[l], (count - l), "%s regs:\n", pdev->ops->name);
	l += mipi_dumpregs(iomem, regs, s, (count - l));

	return l;
}

#if defined CONFIG_FAULT_INJECTION_ATTR || defined CONFIG_HOBOT_MIPI_CSI_ERM_STL
/* sprintf show for mipi phy driver' fault_injection */
int32_t mipi_phy_fault_injection_show_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	char *s = buf;
	int32_t l = 0;

	if ((pdev == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}

#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	l += mipi_csi_stl_inshow(&pdev->dphy.erm.stl, &s[l], (count -l));
#else
	l = -EINVAL;
#endif

	return l;
}

/* sysfs store for mipi phy driver' fault_injection */
int32_t mipi_phy_fault_injection_store_do(struct mipi_pdev_s *pdev,
		const char *name, char *buf, int32_t count)
{
	const char *s = buf;
	int32_t error;

	if ((pdev == NULL) || (s == NULL)) {
		/* do not need report */
		return -EFAULT;
	}

#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	error = mipi_csi_stl_inject(&pdev->dphy.erm.stl, s, (int32_t)count); /* qacfix: conversion */
#else
	error = -EINVAL;
#endif

	return ((error != 0) ? error : count); /* qacfix: conversion */
}
#endif

typedef int32_t (* mipi_phy_sys_do_func)(struct mipi_pdev_s *, const char *, char *, int32_t);
struct mipi_phy_sys_s {
	int32_t type;
	mipi_phy_sys_do_func sys_do[MIPI_SYS_INVALID];
};
static struct mipi_phy_sys_s mipi_phy_sys[MIPI_PHY_SYS_NUM] = {
	{ MIPI_PHY_SYS_PARAM, { mipi_phy_param_show_do, mipi_phy_param_store_do } },
	{ MIPI_PHY_SYS_STATUS_INFO, { mipi_phy_status_info_show_do, NULL } },
	{ MIPI_PHY_SYS_STATUS_HOST, { mipi_phy_status_host_show_do, NULL } },
	{ MIPI_PHY_SYS_STATUS_DEV, { mipi_phy_status_dev_show_do, NULL } },
	{ MIPI_PHY_SYS_STATUS_REGS, { mipi_phy_status_regs_show_do, NULL } },
#if defined CONFIG_FAULT_INJECTION_ATTR || defined CONFIG_HOBOT_MIPI_CSI_ERM_STL
	{ MIPI_PHY_SYS_FAULT_INJECT, { mipi_phy_fault_injection_show_do, mipi_phy_fault_injection_store_do } },
#endif
};

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy sys operation
 *
 * @param[in] type: mipi phy sys type
 * @param[in] sub: sys sub type as show or store
 * @param[in] pdev: mipi phy device struct
 * @param[in] name: sys node name string
 * @param[in] buf: buf to show or store
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
int32_t hobot_mipi_phy_sys_do(int32_t type, int32_t sub, struct mipi_pdev_s *pdev,
			       const char *name, char *buf, int32_t count)
{
	if ((type >= MIPI_PHY_SYS_NUM) || (sub >= MIPI_SYS_INVALID) ||
	    (mipi_phy_sys[type].sys_do[sub] == NULL)) {
		return -EINVAL;
	}
	return mipi_phy_sys[type].sys_do[sub](pdev, name, buf, count);
}

/* report status to diag: status: 0-startup, 1-wakeup; step: 0-start, 1-end */
static void hobot_mipi_phy_diag_report_status(struct mipi_pdev_s *pdev, int32_t status, int32_t step)
{
#if defined CONFIG_HOBOT_FUSA_DIAG && defined CONFIG_HOBOT_MIPI_CSI_ERM_STL
	int32_t ret;
	struct os_dev *dev = &pdev->osdev;
	const uint16_t mod_ids = (uint16_t)ModuleDiag_mipi_erm; /* qacfix: conversion */

	if (status == 0) {
		/* startup status */
		ret = diagnose_report_startup_status(mod_ids,
				(uint8_t)((step == 0) ? MODULE_STARTUP_BEGIN : MODULE_STARTUP_END));
		if (ret != 0) {
			mipi_err(dev, "report startup status %d error %d\n", step, ret);
		}
	} else {
		/* wakeup status */
		ret = diagnose_report_wakeup_status(mod_ids,
				(uint8_t)((step == 0) ? MODULE_WAKEUP_BEGIN : MODULE_WAKEUP_END));
		if (ret != 0) {
			mipi_err(dev, "report wakeup status %d error %d\n", step, ret);
		}
	}
#endif
	return;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device remove operation
 *
 * @param[in] pdev: mipi phy device struct
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
void hobot_mipi_phy_remove_do(struct mipi_pdev_s *pdev)
{
	struct mipi_dphy_s *dphy;

	if ((pdev == NULL) || (g_pdev != pdev)) {
		return;
	}
	dphy = &pdev->dphy;

#ifdef CONFIG_HOBOT_MIPI_CSI_ERM_STL
	mipi_csi_stl_remove(&dphy->erm.stl);
#endif
	g_pdev = NULL;
	return;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy probe operation
 *
 * @param[in] pdev: mipi phy device struct
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
int32_t hobot_mipi_phy_probe_do(struct mipi_pdev_s *pdev)
{
	struct mipi_dphy_s *dphy;

	if (pdev == NULL) {
		return -EINVAL;
	}
	if (g_pdev != NULL) {
		return -EBUSY;
	}
	dphy = &pdev->dphy;
	/* pdev probe */
	hobot_mipi_phy_diag_report_status(pdev, 0, 0);
	osal_spin_init(&dphy->lock); /* PRQA S 3334 */ /* osal_spin_init macro */

	/* dphy probe */
	mipi_dphy_ops_init(pdev);

	if ((dphy->iomem == NULL) || (pdev->ops == NULL) || (pdev->ops->name == NULL)) {
		mipi_info(&pdev->osdev, "probe with dummy done\n");
	} else {
		mipi_info(&pdev->osdev, "probe with %s done\n", pdev->ops->name);
	}

	hobot_mipi_phy_diag_report_status(pdev, 0, 1);

	g_pdev = pdev;
	return 0;
}

/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy device struct get
 *
 * @return mipi phy device struct point
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
struct mipi_pdev_s *hobot_mipi_phy_pdev(void)
{
	return g_pdev;
}

#ifdef CONFIG_HOBOT_FUSA_DIAG
/**
 * @NO{S10E03C02I}
 * @ASIL{B}
 * @brief mipi phy stl setup operation
 *
 * @param[in] setup: mipi phy stl setup fuction
 * @param[in] call: mipi phy stl functions to be called
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
int32_t hobot_mipi_phy_stl_setup_do(MIPI_CSI_STL_SETUP setup, const struct mipi_csi_stl_call *call)
{
	struct mipi_csi_stl_setup_s s = { 0 };
	struct mipi_pdev_s *pdev = hobot_mipi_phy_pdev();
	struct mipi_dphy_s *dphy;
	int32_t ret;

	if (pdev == NULL) {
		return -EEXIST;
	}

	dphy = &pdev->dphy;
	if ((setup == NULL) || (call == NULL)) {
		/* remove */
		mipi_csi_stl_remove(&dphy->erm.stl);
		dphy->erm.stl.call = NULL;
		return 0;
	}

	/* setup */
	if (dphy->erm.stl.call != NULL) {
		return -EBUSY;
	}
	s.iomem = dphy->erm.iomem;
	s.osdev = &pdev->osdev;
	s.dbg_level = &dphy->param.stl_dbg;
	s.fun_mask = &dphy->param.stl_mask;
	s.fun_pile = &dphy->param.stl_pile;
	ret = setup(&dphy->erm.stl, &s);
	if (ret < 0) {
		return ret;
	}
	dphy->erm.stl.call = call;

	return 0;
}
#endif

