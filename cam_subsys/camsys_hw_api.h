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

#ifndef HOBOT_IPE_HW_API
#define HOBOT_IPE_HW_API

#include "vio_config.h"

#define DEV_HW_CAM_SYS	0u
#define DEV_HW_IPE0	1u
#define DEV_HW_IPE1	2u
#define DEV_HW_NUM	3u


void ipe_set_isp_ecc_sram_code_reg(void __iomem *base_reg);
s32 cam_sys_module_reset(void __iomem *base_reg, u32 module, u32 enable);
void ipe_path_cim_pym(void __iomem *base_reg, u32 enable);
void camsys_set_module_id(u16 module_id, u16 event_id);
void camsys_set_parity_inject_value(void __iomem *base_reg, u32 value);
u32 camsys_get_parity_inject_value(void __iomem *base_reg);
u32 ipe_get_reg_value(void __iomem *base_reg);
void ipe_set_reg_value(void __iomem *base_reg, u32 value);
void ipe_set_parity_inject_value(void __iomem *base_reg, u32 value);
u32 ipe_get_parity_inject_value(void __iomem *base_reg);
u32 cam_erm_get_intr_status(void __iomem *base_reg, u32 ipi_ch);
void ipe_hw_dump(void __iomem *base_reg);
void camsys_hw_dump(void __iomem *base_reg);
s32 camsys_check_default_value(void __iomem *base_reg, u32 hw_id);
#endif //__HOBOT_IPE_HW_API__
