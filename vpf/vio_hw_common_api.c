/**
 * @file: vio_hw_common_api.c
 * @
 * @NO{S090E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)    "[VIO regs]:" fmt
#include "vio_hw_common_api.h"
#include "vio_config.h"
#ifdef CONFIG_HOBOT_VIO_STL
#include <linux/hobot_diag.h>
#endif

static u32 vio_hw_get_field_value(u32 reg_value, const struct vio_field_def *field);
static u32 vio_hw_set_field_value(u32 reg_value, const struct vio_field_def *field, u32 val);

#ifdef CONFIG_HOBOT_VIO_STL
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: send read back error event to diagnose driver;
 * @param[in] module_id: module id;
 * @param[in] prio: message level;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vio_fusa_send_rback_error(u32 module_id, enum diag_msg_level prio)
{
	struct diag_event event;

	event.module_id = (u16)((module_id >> BIT_16) & REG_MASK_16BIT);
	event.event_id = (u16)(module_id & REG_MASK_16BIT);
	event.event_prio = (u8)prio;
	event.event_sta = 1u;
	event.fchm_err_code = FUSA_SW_ERR_CODE;
	event.env_len = FUSA_ENV_LEN;
	if (diagnose_send_event(&event) < 0)
		vio_err("%s diagnose_send_event failed\n", __func__);
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: Check whether the written value and read back value are the same. If the three times are different, report fusa error;
 * @param[in] *base_addr: io base address;
 * @param[in] *reg: point to struct vio_reg_def instance;
 * @param[in] val: register value;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
static void vio_fusa_check_rback_error(void __iomem *base_addr, const struct vio_reg_def *reg, u32 val)
{
	u32 i;

	if (reg->attr == RW) {
		for (i = 0; i < FUSA_RBACK_TIMES; i++) {
			if (val == vio_hw_get_reg(base_addr, reg))/*PRQA S 0497*/
				break;
			writel(val, base_addr + reg->sfr_offset);/*PRQA S 0497*/
		}

		if (i > 0) {
			vio_err("%s the value from register 0x%x != which write 0x%x, retry(%d)",
					reg->reg_name, vio_hw_get_reg(base_addr, reg), val, i);
			if (i == FUSA_RBACK_TIMES)
				vio_fusa_send_rback_error(reg->module_id, DiagMsgLevel3);
			else
				vio_fusa_send_rback_error(reg->module_id, DiagMsgLevel2);
		}
	}
}
#endif
/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: read the register value;
 * @param[in] *base_addr: io base address;
 * @param[in] *reg: point to struct vio_reg_def instance;
 * @retval register value;
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
u32 vio_hw_get_reg(const void __iomem *base_addr, const struct vio_reg_def *reg)
{
	u32 reg_value;

	reg_value = readl(base_addr + reg->sfr_offset);/*PRQA S 0497*/

	vio_dbg("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%08X]\n",
			reg->reg_name, reg->sfr_offset, reg_value);

	return reg_value;
}
EXPORT_SYMBOL(vio_hw_get_reg);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: write value to register;
 * @param[in] *base_addr: io base address;
 * @param[in] *reg: point to struct vio_reg_def instance;
 * @param[in] val: value;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_hw_set_reg(void __iomem *base_addr, const struct vio_reg_def *reg, u32 val)
{
	vio_dbg("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%08X]\n",
			reg->reg_name, reg->sfr_offset, val);

	writel(val, base_addr + reg->sfr_offset);/*PRQA S 0497*/

#ifdef CONFIG_HOBOT_VIO_STL
	vio_fusa_check_rback_error(base_addr, reg, val);
#endif
}
EXPORT_SYMBOL(vio_hw_set_reg);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: read field value of register;
 * @param[in] *base_addr: io base address;
 * @param[in] *reg: point to struct vio_reg_def instance;
 * @param[in] *field: point to struct vio_field_def instance;
 * @retval: field value of register
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
u32 vio_hw_get_field(const void __iomem *base_addr,
		const struct vio_reg_def *reg,
		const struct vio_field_def *field) 
{
	u32 reg_value;
	u32 field_value;

	reg_value = readl(base_addr + reg->sfr_offset);/*PRQA S 0497*/
	field_value = vio_hw_get_field_value(reg_value, field);

	vio_dbg("[GET_FIELD] reg:[%s][0x%04X], field:[%d~%d], reg_value(R):[0x%08X] val(R):[%d]\n",
		reg->reg_name, reg->sfr_offset, field->bit_start,
		field->bit_start + field->bit_width, reg_value, field_value);

	return field_value;
}
EXPORT_SYMBOL(vio_hw_get_field);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: write field value of register;
 * @param[in] *base_addr: io base address;
 * @param[in] *reg: point to struct vio_reg_def instance;
 * @param[in] *field: point to struct vio_field_def instance;
 * @param[in] val: field value;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_hw_set_field(void __iomem *base_addr,
		const struct vio_reg_def *reg,
		const struct vio_field_def *field, u32 val) 
{
	u32 reg_value;

	/* previous value reading */ 
	reg_value = readl(base_addr + (reg->sfr_offset));/*PRQA S 0497*/
	reg_value = vio_hw_set_field_value(reg_value, field, val);

	vio_dbg("[SET_FIELD] reg:[%s][0x%04X], field:[%d~%d], reg_value(W):[0x%08X] val(W):[%d]\n",
		reg->reg_name, reg->sfr_offset, field->bit_start,
		field->bit_start + field->bit_width, reg_value, val);

	/* store reg value */ 
	writel(reg_value, base_addr + (reg->sfr_offset));/*PRQA S 0497*/

#ifdef CONFIG_HOBOT_VIO_STL
	vio_fusa_check_rback_error(base_addr, reg, reg_value);
#endif
}
EXPORT_SYMBOL(vio_hw_set_field);/*PRQA S 0605,0307*/

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: get field value from register value;
 * @param[in] reg_value: register value;
 * @param[in] *field: point to struct vio_field_def instance;
 * @retval field value
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
u32 vio_hw_get_field_value(u32 reg_value,
		const struct vio_field_def *field) 
{
	u32 vio_field_mask;
	u32 field_value;

	if (field->bit_width >= REG_BIT_LENGTH)
		vio_field_mask = REG_MASK;
	else
		vio_field_mask = ((u32)1u << field->bit_width) - 1u;
	field_value = (reg_value >> (field->bit_start)) & (vio_field_mask);

	return field_value;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: transfer field value to register value;
 * @param[in] reg_value: register value;
 * @param[in] *field: point to struct vio_field_def instance;
 * @param[in] val: field value;
 * @retval register value
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
u32 vio_hw_set_field_value(u32 reg_value, const struct vio_field_def *field,
		u32 val)
{
	u32 vio_field_mask;
	u32 field_value;

	if (field->bit_width >= REG_BIT_LENGTH)
		vio_field_mask = REG_MASK;
	else
		vio_field_mask = ((u32)1u << field->bit_width) - 1u;

	/* bit clear */ 
	field_value = reg_value & ~(vio_field_mask << field->bit_start);

	/* setting value */ 
	field_value |= (val & vio_field_mask) << (field->bit_start);

	return field_value;
}

/**
 * @NO{S09E05C01}
 * @ASIL{B}
 * @brief: print all register value;
 * @param[in] base_addr: io base address;
 * @param[in] *regs: point to struct vio_reg_def instance;
 * @param[in] total_cnt: the number of registers;
 * @retval None
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
void vio_hw_dump_regs(const void __iomem *base_addr,
		const struct vio_reg_def *regs, u32 total_cnt)
{
	u32 i;
	u32 reg_value;

	for(i = 0; i < total_cnt; i++) {
		reg_value = readl(base_addr + regs[i].sfr_offset);/*PRQA S 0497*/
		vio_info("[DUMP] reg:[%s][0x%04X], value:[0x%08X]\n",
			regs[i].reg_name, regs[i].sfr_offset, reg_value);
	}
}
EXPORT_SYMBOL(vio_hw_dump_regs);/*PRQA S 0605,0307*/
