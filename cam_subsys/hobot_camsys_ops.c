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

#define pr_fmt(fmt)    "[Camsys]:" fmt

#include "hobot_dev_camsys.h"
#include "hobot_camsys_ops.h"
#include "camsys_hw_api.h"

#include "vio_config.h"


#define REGISTER_CLK(name) {name, NULL}

/**
 * Purpose: point to camsys device struct, for extern interface
 * Range: hobot_dev_camsys.c
 * Attention: NA
 */
struct j6_camsys_dev *g_camsys_dev;

/**
 * Purpose: clock description in cam_subsys
 * Range: hobot_dev_camsys.c
 * Attention: NA
 */

#if 1
struct vio_clk vio_clk_list[] = {
	REGISTER_CLK("csi0_ipi_pixel_clk"),
	REGISTER_CLK("csi0_pclk"),
	REGISTER_CLK("csi0_cfg"),
	REGISTER_CLK("csi1_ipi_pixel_clk"),
	REGISTER_CLK("csi1_pclk"),
	REGISTER_CLK("csi1_cfg"),
	REGISTER_CLK("csi2_ipi_pixel_clk"),
	REGISTER_CLK("csi2_pclk"),
	REGISTER_CLK("csi2_cfg"),
	REGISTER_CLK("csi3_ipi_pixel_clk"),
	REGISTER_CLK("csi3_pclk"),
	REGISTER_CLK("csi3_cfg"),
	REGISTER_CLK("gdc_core"),
	REGISTER_CLK("gdc_axi"),
	REGISTER_CLK("vse_axi"),
	REGISTER_CLK("gdc_hclk"),
	REGISTER_CLK("vse_core"),
	REGISTER_CLK("vse_ups"),
        REGISTER_CLK("sensor0_mclk"),
        REGISTER_CLK("sensor1_mclk"),
        REGISTER_CLK("sensor2_mclk"),
        REGISTER_CLK("sensor3_mclk"),
};
#else
struct vio_clk vio_clk_list[] = {
	REGISTER_CLK("cam_apb"),
	REGISTER_CLK("cam_isp_axi"),
	REGISTER_CLK("cam_noc"),
	REGISTER_CLK("cam_gdc"),
	REGISTER_CLK("cam_sth"),
	REGISTER_CLK("cam_dma"),
	REGISTER_CLK("cam_sys"),
	REGISTER_CLK("cam_dphy_pllext"),
	REGISTER_CLK("cam_mipi_scan"),
	REGISTER_CLK("cam_sys_cim"),
	REGISTER_CLK("cam_sys_pym2"),
	REGISTER_CLK("cam_apb_host0"),
	REGISTER_CLK("cam_sys_ipi0"),
	REGISTER_CLK("cam_sys_ipi1"),
	REGISTER_CLK("cam_sys_ipi2"),
	REGISTER_CLK("cam_sys_ipi3"),
	REGISTER_CLK("cam_apb_host1"),
	REGISTER_CLK("cam_sys_ipi4"),
	REGISTER_CLK("cam_sys_ipi5"),
	REGISTER_CLK("cam_sys_ipi6"),
	REGISTER_CLK("cam_sys_ipi7"),
	REGISTER_CLK("cam_apb_host2"),
	REGISTER_CLK("cam_dma_ipi8"),
	REGISTER_CLK("cam_dma_ipi9"),
	REGISTER_CLK("cam_dma_ipi10"),
	REGISTER_CLK("cam_dma_ipi11"),
	REGISTER_CLK("cam_apb_host3"),
	REGISTER_CLK("cam_dma_ipi12"),
	REGISTER_CLK("cam_dma_ipi13"),
	REGISTER_CLK("cam_dma_ipi14"),
	REGISTER_CLK("cam_dma_ipi15"),
	REGISTER_CLK("cam_dphy_cfg"),
	REGISTER_CLK("cam_dphy_cfg_rx0"),
	REGISTER_CLK("cam_dphy_cfg_rx1"),
	REGISTER_CLK("cam_dphy_cfg_rx2"),
	REGISTER_CLK("cam_dphy_cfg_rx3"),
	REGISTER_CLK("cam_dphy_cfg_tx0"),
	REGISTER_CLK("cam_dphy_cfg_tx1"),
	REGISTER_CLK("cam_dphy_ref"),
	REGISTER_CLK("cam_dphy_ref_tx0"),
	REGISTER_CLK("cam_dphy_ref_tx1"),
	REGISTER_CLK("cam_sys_ipe0_isp"),
	REGISTER_CLK("cam_sys_ipe0_pym"),
	REGISTER_CLK("cam_isp_axi_ipe0"),
	REGISTER_CLK("cam_sys_ipe1_isp"),
	REGISTER_CLK("cam_sys_ipe1_pym"),
	REGISTER_CLK("cam_isp_axi_ipe1"),
	REGISTER_CLK("cam_mipi_idi0_clk"),
	REGISTER_CLK("cam_mipi_idi1_clk"),
	REGISTER_CLK("cam_mipi_idi2_clk"),
	REGISTER_CLK("cam_mipi_idi3_clk"),
};
#endif

static void camsys_set_isp_ecc_sram_code(u32 module)
{
    if(ISP0_RST == module)
    {
       ipe_set_isp_ecc_sram_code_reg(g_camsys_dev->base_reg[DEV_HW_IPE0]);
    }
    else
    {
       ipe_set_isp_ecc_sram_code_reg(g_camsys_dev->base_reg[DEV_HW_IPE1]);
    }
}


s32 camsys_set_module_reset(u32 module, u32 rst_flag)
{
	u64 flags = 0;
	s32 ret = 0;

	if (g_camsys_dev == NULL) {
		vio_err("%s: g_camsys_dev = NULL\n", __func__);
		return -EFAULT;
	}
	vio_e_barrier_irqs(g_camsys_dev, flags);/*PRQA S 2996*/
	ret = cam_sys_module_reset(g_camsys_dev->base_reg[DEV_HW_CAM_SYS], module, rst_flag);
	if (ret < 0) {
		vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/
		return ret;
	}

    if(ISP0_RST == module || ISP1_RST == module)
    {
       camsys_set_isp_ecc_sram_code(module);
    }
	vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/
	udelay(100);/*PRQA S 2880*/

	vio_e_barrier_irqs(g_camsys_dev, flags);/*PRQA S 2996*/
	ret = cam_sys_module_reset(g_camsys_dev->base_reg[DEV_HW_CAM_SYS], module, 0);
	vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/

	return 0;
}

static void ipe_fusa_set_enable(u32 hw_id, u32 enable)
{
#ifdef CONFIG_HOBOT_CAMSYS_STL
	u64 flags;

	vio_e_barrier_irqs(g_camsys_dev, flags);/*PRQA S 2996*/
	if (enable == 1) {
		if ((g_camsys_dev->ipe_fusa_enable & 1 << hw_id) == 0) {
			ipe_fusa_enable(g_camsys_dev->base_reg[hw_id], 1);
			g_camsys_dev->ipe_fusa_enable |= 1 << hw_id;
		}
	} else {
		if ((g_camsys_dev->ipe_fusa_enable & 1 << hw_id) != 0) {
			ipe_fusa_enable(g_camsys_dev->base_reg[hw_id], 0);
			g_camsys_dev->ipe_fusa_enable &= ~(1 << hw_id);
		}
	}
	vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/
#endif
}

s32 camsys_set_cim_pym_path(u8 index, u32 enable)
{
	u64 flags = 0;

	if (g_camsys_dev == NULL) {
		vio_err("%s: g_camsys_dev = NULL\n", __func__);
		return -EFAULT;
	}
	if (index >= DEV_HW_IPE1) {
		vio_err("%s: wrong index %d\n", __func__, index);
		return -EFAULT;
	}

	ipe_fusa_set_enable(DEV_HW_IPE0 + index, 0);
	vio_e_barrier_irqs(g_camsys_dev, flags);/*PRQA S 2996*/
	ipe_path_cim_pym(g_camsys_dev->base_reg[DEV_HW_IPE0 + index], enable);
	vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/
	ipe_fusa_set_enable(DEV_HW_IPE0 + index, 1);

	return 0;
}
EXPORT_SYMBOL(camsys_set_cim_pym_path);/*PRQA S 0605,0307*/

u32 camsys_get_ipe_sel(phys_addr_t addr)
{
	u32 hw_index = 0;
	u32 value;

	if (addr == 0x470b0000)
		hw_index = DEV_HW_IPE0;

	if (addr == 0x47100000)
		hw_index = DEV_HW_IPE1;

	if (hw_index == 0)
		return 0;
	value = ipe_get_reg_value(g_camsys_dev->base_reg[hw_index]);
	vio_dbg("%s: addr = 0x%llx, value = 0x%x\n", __func__, addr, value);

	return value;
}

void camsys_set_ipe_sel(u32 value, phys_addr_t addr)
{
	u32 hw_index = 0;
	u64 flags = 0;

	if (addr == 0x470b0000)
		hw_index = DEV_HW_IPE0;

	if (addr == 0x47100000)
		hw_index = DEV_HW_IPE1;

	if (hw_index == 0)
		return;

	vio_e_barrier_irqs(g_camsys_dev, flags);/*PRQA S 2996*/
	ipe_set_reg_value(g_camsys_dev->base_reg[hw_index], value);
	vio_x_barrier_irqr(g_camsys_dev, flags);/*PRQA S 2996*/
	vio_dbg("%s: addr = 0x%llx, value = 0x%x\n", __func__, addr, value);
}

static void ipe_fusa_clk_en(const char *name, u32 enable)
{
	if (strcmp(name, "cam_sys_ipe0_isp") == 0 || strcmp(name, "cam_sys_ipe0_pym") == 0) {
		ipe_fusa_set_enable(DEV_HW_IPE0, enable);
	}
	if (strcmp(name, "cam_sys_ipe1_isp") == 0 || strcmp(name, "cam_sys_ipe1_pym") == 0)
		ipe_fusa_set_enable(DEV_HW_IPE1, enable);
}

s32 vio_clk_enable(const char *name)
{
	s32 ret = 0;
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (strcmp(name, vio_clk_list[index].name) == 0)
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("%s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	ipe_fusa_clk_en(name, 0);
	ret = clk_prepare_enable(clk);
	ipe_fusa_clk_en(name, 1);
	if (ret) {
		vio_err("%s: clk_prepare_enable is fail(%s)\n", __func__, name);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(vio_clk_enable);/*PRQA S 0605,0307*/

s32 vio_clk_disable(const char *name)
{
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (strcmp(name, vio_clk_list[index].name) == 0)
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("%s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}
	ipe_fusa_clk_en(name, 0);
	clk_disable_unprepare(clk);
	ipe_fusa_clk_en(name, 1);

	return 0;
}
EXPORT_SYMBOL(vio_clk_disable);/*PRQA S 0605,0307*/

s32 vio_set_clk_rate(const char *name, u64 frequency)
{
	s32 ret = 0;
	size_t index;
	u64 round_rate = 0;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (strcmp(name, vio_clk_list[index].name) == 0)
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_err("%s: clk_target_list is NULL : %s\n", __func__, name);
		return -EINVAL;
	}

	round_rate = clk_round_rate(clk, frequency);
	ret = clk_set_rate(clk, round_rate);
	if (ret) {
		vio_err("%s: clk_set_rate is fail(%s)\n", __func__, name);
		return ret;
	}

	vio_dbg("%s: frequence %lld round_rate %lld\n", __func__, frequency, round_rate);/*PRQA S 0685,1294*/

	return ret;
}
EXPORT_SYMBOL(vio_set_clk_rate);/*PRQA S 0605,0307*/

u64 vio_get_clk_rate(const char *name)
{
	u64 frequency;
	size_t index;
	struct clk *clk = NULL;

	for (index = 0; index < ARRAY_SIZE(vio_clk_list); index++) {
		if (strcmp(name, vio_clk_list[index].name) == 0)
			clk = vio_clk_list[index].clk;
	}

	if (IS_ERR_OR_NULL(clk)) {
		vio_info("[@][ERR] %s: clk_target_list is NULL : %s\n", __func__, name);
		return 0;
	}

	frequency = clk_get_rate(clk);

	return frequency;
}
EXPORT_SYMBOL(vio_get_clk_rate);/*PRQA S 0605,0307*/

s32 vio_get_clk(struct device *dev)
{
	struct clk *clk;
	size_t i = 0;

	for (i = 0; i < ARRAY_SIZE(vio_clk_list); i++) {
		clk = devm_clk_get(dev, vio_clk_list[i].name);
		if (IS_ERR_OR_NULL(clk)) {
			vio_err("[@][ERR] %s: could not lookup clock : %s\n",
				__func__, vio_clk_list[i].name);
			return -EINVAL;
		}
		vio_clk_list[i].clk = clk;
		dev_dbg(dev, "%s clock frequence is %lld\n",/*PRQA S 0685,1294*/
			vio_clk_list[i].name, vio_get_clk_rate(vio_clk_list[i].name));
	}

	return 0;
}

void vio_put_clk(struct device *dev)
{
	size_t i = 0;

	for (i = 0; i < ARRAY_SIZE(vio_clk_list); i++) {
		devm_clk_put(dev, vio_clk_list[i].clk);
		vio_clk_list[i].clk = NULL;
	}
}

void camsys_set_global_ops(struct j6_camsys_dev *camsys)
{
	g_camsys_dev = camsys;
}

void camsys_set_clk_enable(u32 enable)
{
	if (enable == 1) {
		vio_clk_enable("cam_sys_ipe0_pym");
		vio_clk_enable("cam_sys_ipe1_pym");
	} else {
		vio_clk_disable("cam_sys_ipe0_pym");
		vio_clk_disable("cam_sys_ipe1_pym");
	}
}
