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

#ifndef HOBOT_DEV_CAMSYS_H
#define HOBOT_DEV_CAMSYS_H

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>

#include "vio_node_api.h"
#include "camsys_hw_api.h"

struct vio_clk {
	const char *name;
	struct clk *clk;
};

struct j6_camsys_dev {
	void __iomem			*base_reg[DEV_HW_NUM];
	resource_size_t			regs_start[DEV_HW_NUM];
	resource_size_t			regs_end[DEV_HW_NUM];
	spinlock_t	slock;
	/* fusa features */
#ifdef CONFIG_HOBOT_CAMSYS_STL
	u32 ipe_fusa_enable;
	struct vio_stl stl;
#endif
};


s32 vio_get_clk(struct device *dev);
void camsys_set_global_ops(struct j6_camsys_dev *camsys);
void camsys_set_clk_enable(u32 enable);
#ifdef CONFIG_HOBOT_CAMSYS_STL
void camsys_fusa_setup_ops(struct j6_camsys_dev *dev);
int j6_camsys_fault_injection_show(struct device *dev, char *buf, size_t size);
int j6_camsys_fault_injection_store(struct device *dev, const char *buf, size_t count);
void ipe_fusa_enable(void __iomem *base_reg, u32 enable);
#endif
#endif
