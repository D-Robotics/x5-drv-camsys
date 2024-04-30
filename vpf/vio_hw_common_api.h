/**
 * @file: vio_hw_common_api.h
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef VIO_HW_API_COMMON_H
#define VIO_HW_API_COMMON_H

#ifdef HOBOT_MCU_CAMSYS
#include "camsys_common.h"
#else
#include <linux/io.h>
#include <linux/module.h>
#include <linux/errno.h>
#endif

#include "vio_config.h"

enum regdata_type { 
	/* read write */ 
	RW = 0, 
	/* read only */ 
	RO = 1, 
	/* write only */ 
	WO = 2,
	/* clear after read */ 
	RAC = 3, 
	/* write 1 -> clear */ 
	W1C = 4, 
	/* write 1 set */ 
	W1S = 5, 
	/* write 1 trigger */ 
	W1T = 6,

	/* FUSA register*/
	FUSA_RW = 10,
	FUSA_RO = 11,
	FUSA_WO = 12,
	FUSA_W1C = 13,
};

#define REG_BIT_LENGTH 32u
#define REG_MASK 0xffffffffu
#define REG_MASK_16BIT 0xffffu
#define BIT_16 16u
struct vio_reg_def {
	char *reg_name;
	u32 sfr_offset;
	enum regdata_type attr;
	u32 default_val;
	u32 module_id;
};

struct vio_field_def {
	u32 reg;
	u32 index;
	u8 bit_start;
	u8 bit_width;
	u32 default_val;
};

u32 vio_hw_get_reg(const void __iomem *base_addr, const struct vio_reg_def *reg);
void vio_hw_set_reg(void __iomem *base_addr, const struct vio_reg_def *reg,
				u32 val);
u32 vio_hw_get_field(const void __iomem *base_addr, const struct vio_reg_def *reg,
				const struct vio_field_def *field);
void vio_hw_set_field(void __iomem *base_addr, const struct vio_reg_def *reg,
				const struct vio_field_def *field, u32 val);
void vio_hw_dump_regs(const void __iomem *base_addr, const struct vio_reg_def *regs,
				u32 total_cnt);
#endif /* VIO_HW_API_COMMON_H */
