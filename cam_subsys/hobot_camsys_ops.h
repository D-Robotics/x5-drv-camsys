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

#ifndef HOBOT_CAMSYS_OPS_H
#define HOBOT_CAMSYS_OPS_H

/**
 * camsys_set_module_reset - set reset register in cam_subsys module;
 * @module: IP number
 * @enable: 1: pull up reset register; 0: pull down reset register;
 * Returns: 0 on success or a negative error code on failure.
 */
s32 camsys_set_module_reset(u32 module, u32 rst_flag);

/**
 * ipe_set_cim_pym_path - select path between cim-otf-pym and cim-otf-isp;
 * @module: IPE number, 0: IPE0; 1: IPE1;
 * @enable: 1: select cim-otf-pym path; 0: select cim-otf-isp path;
 * Returns: 0 on success or a negative error code on failure.
 */
s32 camsys_set_cim_pym_path(u8 index, u32 enable);

/**
 * vio_clk_enable - enable the clock in cam_subsys;
 * @name: clock name;
 * Returns: 0 on success or a negative error code on failure.
 */
s32 vio_clk_enable(const char *name);

/**
 * vio_clk_disable - disable the clock in cam_subsys;
 * @name: clock name;
 * Returns: 0 on success or a negative error code on failure.
 */
s32 vio_clk_disable(const char *name);

/**
 * vio_set_clk_rate - set clock frequency in cam_subsys;
 * @name: clock name;
 * @frequency: clock frequency;
 * Returns: 0 on success or a negative error code on failure.
 */
s32 vio_set_clk_rate(const char *name, u64 frequency);

/**
 * vio_get_clk_rate - get clock frequency in cam_subsys;
 * @name: clock name;
 * Returns: clock frequency.
 */
u64 vio_get_clk_rate(const char *name);
void vio_put_clk(struct device *dev);

u32 camsys_get_ipe_sel(phys_addr_t addr);
void camsys_set_ipe_sel(u32 value, phys_addr_t addr);

#endif
