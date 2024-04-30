/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "vio_hw_common_api.h"
#include "hobot_lpwm_hw_reg.h"

static struct vio_reg_def lpwm_regs[]={
	{"LPWM_GLB_CFG", 0x0000, RW, 0x257000},
	{"LPWM_SW_TRIG", 0x0004, WO, 0x0},
	{"LPWM_RST", 0x0008, WO, 0x0},
	{"LPWM_0_CFG0", 0x0010, RW, 0x0},
	{"LPWM_0_CFG1", 0x0014, RW, 0x108235},
	{"LPWM_0_CFG2", 0x0018, RW, 0x0},
	{"LPWM_1_CFG0", 0x001C, RW, 0x0},
	{"LPWM_1_CFG1", 0x0020, RW, 0x108235},
	{"LPWM_1_CFG2", 0x0024, RW, 0x0},
	{"LPWM_2_CFG0", 0x0028, RW, 0x0},
	{"LPWM_2_CFG1", 0x002C, RW, 0x108235},
	{"LPWM_2_CFG2", 0x0030, RW, 0x0},
	{"LPWM_3_CFG0", 0x0034, RW, 0x0},
	{"LPWM_3_CFG1", 0x0038, RW, 0x108235},
	{"LPWM_3_CFG2", 0x003C, RW, 0x0},
	{"LPWM_PPS_FAIL_CNT", 0x0040, W1C, 0x0},
};

static struct vio_field_def lpwm_fields[] = {
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_0_EN, 0, 1, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_1_EN, 1, 1, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_2_EN, 2, 1, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_3_EN, 3, 1, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_INT_EN, 4, 1 , 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_MODE_SEL, 5, 1, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_TRIGGER_SRC_SEL, 8, 4, 0x0},
	{(u32)LPWM_GLB_CFG, (u32)LPWMF_DIV_RATIO, 12, 10 , 0x257},
	{(u32)LPWM_SW_TRIG, (u32)LPWMF_SW_TEIG, 0, 1, 0x0},
	{(u32)LPWM_RST, (u32)LPWMF_RST, 0, 1, 0x0},
	{(u32)LPWM_0_CFG0, (u32)LPWMF_0_OFFSET, 0, 20, 0x0},
	{(u32)LPWM_0_CFG1, (u32)LPWMF_0_CFG1, 0, 32, 0x108235},
	{(u32)LPWM_0_CFG2, (u32)LPWMF_0_CFG2, 0, 21, 0x0},
	{(u32)LPWM_1_CFG0, (u32)LPWMF_1_OFFSET, 0, 20, 0x0},
	{(u32)LPWM_1_CFG1, (u32)LPWMF_1_CFG1, 0, 32, 0x108235},
	{(u32)LPWM_1_CFG2, (u32)LPWMF_1_CFG2, 0, 21, 0x0},
	{(u32)LPWM_2_CFG0, (u32)LPWMF_2_OFFSET, 0, 20, 0x0},
	{(u32)LPWM_2_CFG1, (u32)LPWMF_2_CFG1, 0, 32, 0x108235},
	{(u32)LPWM_2_CFG2, (u32)LPWMF_2_CFG2, 0, 21, 0x0},
	{(u32)LPWM_3_CFG0, (u32)LPWMF_3_OFFSET, 0, 20, 0x0},
	{(u32)LPWM_3_CFG1, (u32)LPWMF_3_CFG1, 0, 32, 0x108235},
	{(u32)LPWM_3_CFG2, (u32)LPWMF_3_CFG2, 0, 21, 0x0},
	{(u32)LPWM_PPS_FAIL_CNT, (u32)LPWMF_0_PPS_FAIL_CNT, 0, 8, 0x0},
	{(u32)LPWM_PPS_FAIL_CNT, (u32)LPWMF_1_PPS_FAIL_CNT, 8, 8, 0x0},
	{(u32)LPWM_PPS_FAIL_CNT, (u32)LPWMF_2_PPS_FAIL_CNT, 16, 8, 0x0},
	{(u32)LPWM_PPS_FAIL_CNT, (u32)LPWMF_3_PPS_FAIL_CNT, 24, 8, 0x0},
};

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Enable one channel of lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] enable: Determine whether to enable/disable
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_channel_enable_single(void __iomem *base_reg, uint32_t c_id, uint32_t enable)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_0_EN + c_id;
	val = enable > 0 ? 1 : 0;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config interrupt of lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] enable: Determine whether to enable the interrupt
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_interrupt_config(void __iomem *base_reg, uint32_t enable)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_INT_EN;
	val = enable > 0 ? 1 : 0;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Read interrupt state of lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] enable: Determine whether to enable the interrupt
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_interrupt_read(void __iomem *base_reg, uint32_t *enable)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_INT_EN;
	val = vio_hw_get_field(base_reg, &lpwm_regs[offset],
			       &lpwm_fields[field_offset]);
	if (enable != NULL) {
		*enable = val;
	}
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config trigger mode of lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] mode: The mode to config
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_trigger_mode_config(void __iomem *base_reg, uint32_t mode)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_MODE_SEL;
	val = mode > 0 ? 1 : 0;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config trigger source of lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] source: The source to config
 * range: [TRIGGER_AON_RTC, LPWM_TRIG_SOURCE_MAX);
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_trigger_source_config(void __iomem *base_reg, uint32_t source)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_TRIGGER_SRC_SEL;
	val = source;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config offset of lpwm channel
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] offset: The offset to config
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_offset_config_single(void __iomem *base_reg, uint32_t c_id, uint32_t ofs)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_0_CFG0 + c_id * LPWM_REG_CFG_OFF;
	field_offset = LPWMF_0_OFFSET + c_id * LPWM_FILED_CFG_OFF;
	val = ofs & LPWM_OFFSET_MAX;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config cfg1 of lpwm channel
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] period: The period to config
 * @param[in] duty_time: The duty_time to config
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_cfg1_config_single(void __iomem *base_reg, uint32_t c_id, uint32_t period,
			     uint32_t duty_time)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_0_CFG1 + c_id * LPWM_REG_CFG_OFF;
	field_offset = LPWMF_0_CFG1 + c_id * LPWM_FILED_CFG_OFF;
	val = (duty_time << LPWM_DUTY_TIME_SHIFT) | (period & LPWM_PERIOD_MAX);

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config cfg2 of lpwm channel
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] adjust_step: The adjust_step to config
 * @param[in] threshold: The threshold to config
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_cfg2_config_single(void __iomem *base_reg, uint32_t c_id,
			     uint32_t threshold, uint32_t adjust_step)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_0_CFG2 + c_id * LPWM_REG_CFG_OFF;
	field_offset = LPWMF_0_CFG2 + c_id * LPWM_FILED_CFG_OFF;
	val = (adjust_step << LPWM_ADJUST_STEP_SHIFT) | (threshold & LPWM_THRESHOLD_MAX);

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Trigger lpwm by SW
 * @param[in] *base_reg: The base addr of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_sw_trigger(void __iomem *base_reg)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_SW_TRIG;
	field_offset = LPWMF_SW_TEIG;
	val = 1;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Reset lpwm
 * @param[in] *base_reg: The base addr of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_ins_reset(void __iomem *base_reg)
{
	uint32_t offset;
	uint32_t field_offset;
	uint32_t val;

	offset = LPWM_RST;
	field_offset = LPWMF_RST;
	val = 1;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], val);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Div ratio of lpwm clk
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] ratio: Div ratio config to lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_div_ratio_config(void __iomem *base_reg, uint32_t ratio)
{
	uint32_t offset;
	uint32_t field_offset;

	offset = LPWM_GLB_CFG;
	field_offset = LPWMF_DIV_RATIO;

	vio_hw_set_field(base_reg, &lpwm_regs[offset], &lpwm_fields[field_offset], ratio);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Enable lpwm channel directly
 * @param[in] *base_reg: The base addr of lpwm
 * @param[in] config: Attr config to lpwm
 * @param[in] c_id: Channel id to enable
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_single_channel_config_enable(void __iomem *base_reg, uint32_t c_id,
				       struct lpwm_attr_single *config)
{
	/* config static ctrl regs */
	lpwm_cfg2_config_single(base_reg, c_id, config->threshold, config->adjust_step);
	lpwm_trigger_mode_config(base_reg, config->trigger_mode);
	lpwm_trigger_source_config(base_reg, config->trigger_source);

	/* config dynamic ctrl regs */
	lpwm_offset_config_single(base_reg, c_id, config->threshold);
	lpwm_cfg1_config_single(base_reg, c_id, config->period, config->duty_time);

	/* turn on */
	lpwm_channel_enable_single(base_reg, c_id, 1);

	/* sw trigger */
	if (config->trigger_mode == 0) {
		lpwm_sw_trigger(base_reg);
	}
}

/* LPWM STL regs */
static struct vio_reg_def cam_sub_sys_regs[]={
	{"TOP_SOFT_RST", 0x024, RW, 0x0},
	{"LPWM_CLK_EN", 0x028, RW, 0xF},
	{"PASSWORD", 0x300, RW, 0x95F2D303},
	{"FUSA_CTRL", 0x304, RW, 0x0},
	{"MISSIONINT_MASK", 0x328, RW, 0x1FF},
	{"MISSIONINT_STATUS_SET", 0x32C, WO, 0x0},
	{"MISSIONINT_STATUS", 0x330, W1C, 0x0},
};

static struct vio_field_def cam_sub_sys_reg_fields[] = {
	{(u32)TOP_SOFT_RST, (u32)LPWM0_SOFT_RST, 0, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM0_APB_SOFT_RST, 1, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM1_SOFT_RST, 2, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM1_APB_SOFT_RST, 3, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM2_SOFT_RST, 4, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM2_APB_SOFT_RST, 5, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM3_SOFT_RST, 6, 1, 0x0},
	{(u32)TOP_SOFT_RST, (u32)LPWM3_APB_SOFT_RST, 7, 1, 0x0},

	{(u32)LPWM_CLK_EN, (u32)LPWM_CLK_APB_EN, 0, 4, 0xF},

	{(u32)PASSWORD, (u32)PASSWD, 0, 32, 0x95F2D303},

	{(u32)FUSA_CTRL, (u32)LPWM_DCLS_FAULT_INJECT, 15, 8, 0x0},

	{(u32)MISSIONINT_MASK, (u32)LPWM_DCLS_CORE_MASK, 1, 4, 0xF},
	{(u32)MISSIONINT_MASK, (u32)LPWM_DCLS_APB_MASK, 5, 4, 0xF},

	{(u32)MISSIONINT_STATUS_SET, (u32)LPWM_DCLS_CORE_SET, 1, 4, 0x0},
	{(u32)MISSIONINT_STATUS_SET, (u32)LPWM_DCLS_APB_SET, 5, 4, 0x0},

	{(u32)MISSIONINT_STATUS, (u32)LPWM_DCLS_CORE_STATUS, 1, 4, 0x0},
	{(u32)MISSIONINT_STATUS, (u32)LPWM_DCLS_APB_STATUS, 5, 4, 0x0},
};

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lock/unlock lpwm fusa reg
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] lock: Lock-1 / unlock-0
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_reg_passwd(void __iomem *base, uint32_t lock)
{
	uint32_t cfg_val = LOCK_PASSWORD;
	uint32_t offset;
	uint32_t field_offset;

	if (!lock) {
		cfg_val = UNLOCK_PASSWORD;
	}
	offset = PASSWORD;
	field_offset = PASSWD;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_reg_passwd);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Update fusa core mask
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @param[in] val: Config val
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_update_core_mask(void __iomem *base, uint32_t i_id, uint32_t val)
{
	
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	offset = MISSIONINT_MASK;
	field_offset = LPWM_DCLS_CORE_MASK;

	cfg_val = vio_hw_get_field(base, &cam_sub_sys_regs[offset],
				   &cam_sub_sys_reg_fields[field_offset]);

	cfg_val &= ~(0x1u << i_id);
	cfg_val |= val << i_id;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_update_core_mask);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Update fusa apb mask
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @param[in] val: Config val
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_update_apb_mask(void __iomem *base, uint32_t i_id, uint32_t val)
{
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	offset = MISSIONINT_MASK;
	field_offset = LPWM_DCLS_APB_MASK;

	cfg_val = vio_hw_get_field(base, &cam_sub_sys_regs[offset],
				   &cam_sub_sys_reg_fields[field_offset]);

	cfg_val &= ~(0x1u << i_id);
	cfg_val |= val << i_id;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_update_apb_mask);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Update fusa update fault injection
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @param[in] val: Config val
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_update_fault_injection(void __iomem *base, uint32_t i_id, uint32_t val)
{

	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	offset = FUSA_CTRL;
	field_offset = LPWM_DCLS_FAULT_INJECT;

	cfg_val = vio_hw_get_field(base, &cam_sub_sys_regs[offset],
				   &cam_sub_sys_reg_fields[field_offset]);

	/* 2bit read clear */
	cfg_val &= ~(0x11u << (i_id * LPWM_FAULT_INJECT_WIDTH));
	cfg_val |= val << (i_id * LPWM_FAULT_INJECT_WIDTH);

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_update_fault_injection);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Clear err status of core
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_core_status_clear(void __iomem *base, uint32_t i_id)
{
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	cfg_val = 0x1u;
	offset = MISSIONINT_STATUS;
	field_offset = LPWM_DCLS_CORE_STATUS;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_core_status_clear);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Clear err status of apb
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_apb_status_clear(void __iomem *base, uint32_t i_id)
{
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	cfg_val = 0x1u;
	offset = MISSIONINT_STATUS;
	field_offset = LPWM_DCLS_APB_STATUS;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_apb_status_clear);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Read err status of core
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
uint32_t lpwm_fusa_core_status_read(void __iomem *base, uint32_t i_id)
{
	uint32_t val, shift;

	val = readl(base + LPWM_ERR_STATUS_OFFSET);

	shift = LPWM_CORE_ERR_STATUS_SHIFT + i_id;
	return (val & (0x1u << shift));
}
EXPORT_SYMBOL_GPL(lpwm_fusa_core_status_read);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Read err status of apb
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
uint32_t lpwm_fusa_apb_status_read(void __iomem *base, uint32_t i_id)
{
	uint32_t val, shift;

	val = readl(base + LPWM_ERR_STATUS_OFFSET);

	shift = LPWM_APB_ERR_STATUS_SHIFT + i_id;
	return (val & (0x1u << shift));
}
EXPORT_SYMBOL_GPL(lpwm_fusa_apb_status_read);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Set err status of core
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_core_err_set(void __iomem *base, uint32_t i_id)
{
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	cfg_val = 0x1u;
	offset = MISSIONINT_STATUS_SET;
	field_offset = LPWM_DCLS_CORE_SET;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_core_err_set);

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Set err status of apb
 * @param[in] *base: The base addr of cam-subsys
 * @param[in] i_id: Instance id of lpwm
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void lpwm_fusa_apb_err_set(void __iomem *base, uint32_t i_id)
{
	uint32_t cfg_val;
	uint32_t offset;
	uint32_t field_offset;

	cfg_val = 0x1u;
	offset = MISSIONINT_STATUS_SET;
	field_offset = LPWM_DCLS_APB_SET;

	vio_hw_set_field(base, &cam_sub_sys_regs[offset],
			 &cam_sub_sys_reg_fields[field_offset], cfg_val);
}
EXPORT_SYMBOL_GPL(lpwm_fusa_apb_err_set);