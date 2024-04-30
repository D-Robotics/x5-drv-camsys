/*************************************************************
  ****                    COPYRIGHT NOTICE
  ****            Copyright      2020 Horizon Robotics, Inc.
  ****                    All rights reserved.
  *************************************************************/

#ifndef HOBOT_LPWM_HW_REG_J6_H_
#define HOBOT_LPWM_HW_REG_J6_H_

#define LPWM_CORE_REG_OFFSET	12		/**< Offset of each lpwm core attr reg */

/* LPWM reg bit */
#define LPWM_INT_EN_SHIFT	4
#define LPWM_MODE_SEL_SHIFT	5
#define LPWM_TRIG_SOURCE_SHIFT	8
#define LPWM_TRIG_SOURCE_MASK	(BIT(8) | BIT(9) | BIT(10) | BIT(11))
// #define LPWM_OFFSET_MASK	0x000FFFFF

#define LPWM_DUTY_TIME_MASK	0xFFF00000
#define LPWM_DUTY_TIME_SHIFT	20
#define LPWM_ADJUST_STEP_MASK	0xFFFF0000
#define LPWM_ADJUST_STEP_SHIFT	16

/* LPWM trigger source */
#define TRIGGER_AON_RTC		0
#define TRIGGER_SOFTWARE_RTC0	1
#define TRIGGER_SOFTWARE_RTC1	2
#define TRIGGER_SOFTWARE_RTC2	3
#define TRIGGER_SOFTWARE_RTC3	4
#define TRIGGER_PAD0		5
#define TRIGGER_PAD1		6
#define TRIGGER_PAD2		7
#define TRIGGER_PAD3		8
#define PCIE_ETH		9
#define MCU_ETH			10
#define LPWM_TRIG_SOURCE_MAX	11

/* lpwm config range */
#define LPWM_OFFSET_MAX		0xFFFFF
#define LPWM_PERIOD_MIN		2
#define LPWM_PERIOD_MAX		0xFFFFF
#define LPWM_HIGH_MAX		0xFFF
#define LPWM_THRESHOLD_MAX	0xFFFF
#define LPWM_STEP_MAX		0xF

#define LPWM_REG_NUM		16
#define LPWM_REG_FILED_NUM	34
#define LPWM_REG_CFG_OFF	3
#define LPWM_FILED_CFG_OFF	3

enum lpwm_reg_e {
	LPWM_GLB_CFG,
	LPWM_SW_TRIG,
	LPWM_RST,
	LPWM_0_CFG0,
	LPWM_0_CFG1,
	LPWM_0_CFG2,
	LPWM_1_CFG0,
	LPWM_1_CFG1,
	LPWM_1_CFG2,
	LPWM_2_CFG0,
	LPWM_2_CFG1,
	LPWM_2_CFG2,
	LPWM_3_CFG0,
	LPWM_3_CFG1,
	LPWM_3_CFG2,
	LPWM_PPS_FAIL_CNT,
};

enum lpwm_reg_field_e {
	// LPWM_GLB_CFG
	LPWMF_0_EN,
	LPWMF_1_EN,
	LPWMF_2_EN,
	LPWMF_3_EN,
	LPWMF_INT_EN,
	LPWMF_MODE_SEL,
	LPWMF_TRIGGER_SRC_SEL,
	LPWMF_DIV_RATIO,
	// LPWM_SW_TRIG
	LPWMF_SW_TEIG,
	// LPWM_RST
	LPWMF_RST,

	// LPWM_0_CFG0
	LPWMF_0_OFFSET,
	// LPWM_0_CFG1
	LPWMF_0_CFG1,

	// LPWM_0_CFG2
	LPWMF_0_CFG2,

	// LPWM_1_CFG0
	LPWMF_1_OFFSET,
	// LPWM_1_CFG1
	LPWMF_1_CFG1,
	// LPWM_1_CFG2
	LPWMF_1_CFG2,

	// LPWM_2_CFG0
	LPWMF_2_OFFSET,
	// LPWM_2_CFG1
	LPWMF_2_CFG1,
	// LPWM_2_CFG2
	LPWMF_2_CFG2,

	// LPWM_3_CFG0
	LPWMF_3_OFFSET,
	// LPWM_3_CFG1
	LPWMF_3_CFG1,
	// LPWM_3_CFG2
	LPWMF_3_CFG2,

	// LPWM_PPS_FAIL_CNT
	LPWMF_0_PPS_FAIL_CNT,
	LPWMF_1_PPS_FAIL_CNT,
	LPWMF_2_PPS_FAIL_CNT,
	LPWMF_3_PPS_FAIL_CNT,
};

/* Used by MCU, should be consistent with Acore  */
struct lpwm_attr_single {
	uint32_t trigger_source;
	uint32_t trigger_mode;
	uint32_t period;
	uint32_t offset;
	uint32_t duty_time;
	uint32_t threshold;
	uint32_t adjust_step;
};

// lpwm hw interface
extern void lpwm_channel_enable_single(void __iomem *base_reg, uint32_t c_id, uint32_t enable);
extern void lpwm_interrupt_config(void __iomem *base_reg, uint32_t enable);
extern void lpwm_interrupt_read(void __iomem *base_reg, uint32_t *enable);
extern void lpwm_trigger_mode_config(void __iomem *base_reg, uint32_t mode);
extern void lpwm_trigger_source_config(void __iomem *base_reg, uint32_t source);
extern void lpwm_offset_config_single(void __iomem *base_reg, uint32_t c_id, uint32_t ofs);
extern void lpwm_cfg1_config_single(void __iomem *base_reg, uint32_t c_id, uint32_t period,
				    uint32_t duty_time);
extern void lpwm_cfg2_config_single(void __iomem *base_reg, uint32_t c_id,
			     uint32_t threshold, uint32_t adjust_step);
extern void lpwm_sw_trigger(void __iomem *base_reg);
extern void lpwm_ins_reset(void __iomem *base_reg);
extern void lpwm_div_ratio_config(void __iomem *base_reg, uint32_t ratio);
extern void lpwm_single_channel_config_enable(void __iomem *base_reg, uint32_t c_id,
				       struct lpwm_attr_single *config);


/* lpwm fusa control register */
#define LPWM_SUBSYS_PASSWD_OFFSET	0x300	/**< Offset of passwd to base */
#define LPWM_FAULT_INJECT_OFFSET	0x304	/**< Offset of fault injection to base */
#define LPWM_FAULT_INJECT_SHIFT 	15	/**< Shift of fault injection */
#define LPWM_FAULT_INJECT_WIDTH 	2	/**< width of fault injection */
#define LPWM_ERR_MASK_OFFSET 		0X328	/**< Offset of err mask to base */
#define LPWM_CORE_ERR_MASK_SHIFT 	1	/**< Shift of core err mask */
#define LPWM_APB_ERR_MASK_SHIFT 	5	/**< Shift of apb err mask */
#define LPWM_ERR_SET_OFFSET 		0x32C	/**< Offset of err set to base */
#define LPWM_CORE_ERR_SET_SHIFT 	1	/**< Shift of core err set */
#define LPWM_APB_ERR_SET_SHIFT 		5	/**< Shift of apb err set */
#define LPWM_ERR_STATUS_OFFSET 		0x330	/**< Offset of err status to base */
#define LPWM_CORE_ERR_STATUS_SHIFT 	1	/**< Shift of core err status */
#define LPWM_APB_ERR_STATUS_SHIFT 	5	/**< Shift of apb err status */

#define UNLOCK_PASSWORD			0x95F2D303u
#define LOCK_PASSWORD			0x0u
#define LPWM_FAULT_INJECT_INTERVAL	20
#define LPWM_FAULT_INJECT_VAL1		0x10u
#define LPWM_FAULT_INJECT_VAL2		0x01u

enum lpwm_stl_reg_e {
	TOP_SOFT_RST,
	LPWM_CLK_EN,
	PASSWORD,
	FUSA_CTRL,
	MISSIONINT_MASK,
	MISSIONINT_STATUS_SET,
	MISSIONINT_STATUS,
};

enum lpwm_stl_reg_field_e {
	// TOP_SOFT_RST
	LPWM0_SOFT_RST,
	LPWM0_APB_SOFT_RST,
	LPWM1_SOFT_RST,
	LPWM1_APB_SOFT_RST,
	LPWM2_SOFT_RST,
	LPWM2_APB_SOFT_RST,
	LPWM3_SOFT_RST,
	LPWM3_APB_SOFT_RST,

	// LPWM_CLK_EN
	LPWM_CLK_APB_EN,

	// PASSWORD
	PASSWD,

	// FUSA_CTRL
	LPWM_DCLS_FAULT_INJECT,

	// MISSIONINT_MASK
	LPWM_DCLS_CORE_MASK,
	LPWM_DCLS_APB_MASK,

	// MISSIONINT_STATUS_SET
	LPWM_DCLS_CORE_SET,
	LPWM_DCLS_APB_SET,

	// MISSIONINT_STATUS
	LPWM_DCLS_CORE_STATUS,
	LPWM_DCLS_APB_STATUS,
};

// lpwm fusa interface
extern void lpwm_fusa_reg_passwd(void __iomem *base, uint32_t lock);
extern void lpwm_fusa_update_core_mask(void __iomem *base, uint32_t i_id, uint32_t val);
extern void lpwm_fusa_update_apb_mask(void __iomem *base, uint32_t i_id, uint32_t val);
extern void lpwm_fusa_update_fault_injection(void __iomem *base, uint32_t i_id, uint32_t val);
extern void lpwm_fusa_core_status_clear(void __iomem *base, uint32_t i_id);
extern void lpwm_fusa_apb_status_clear(void __iomem *base, uint32_t i_id);
extern uint32_t lpwm_fusa_core_status_read(void __iomem *base, uint32_t i_id);
extern uint32_t lpwm_fusa_apb_status_read(void __iomem *base, uint32_t i_id);
extern void lpwm_fusa_core_err_set(void __iomem *base, uint32_t i_id);
extern void lpwm_fusa_apb_err_set(void __iomem *base, uint32_t i_id);

#endif // HOBOT_LPWM_HW_REG_J6_H_