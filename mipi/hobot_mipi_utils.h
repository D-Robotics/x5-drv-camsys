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
 * @file hobot_mipi_utils.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_UTILS_H__
#define __HOBOT_MIPI_UTILS_H__ /* PRQA S 0603 */ /* header file macro */

#include "hobot_mipi_osal.h"

#define MIPI_MODULE_CLASS_NAME		"vin"

#ifdef CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#define MIPI_HOST_MAX_NUM	CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#else
#define MIPI_HOST_MAX_NUM	(4)
#endif
#ifdef CONFIG_HOBOT_MIPI_DEV_MAX_NUM
#define MIPI_DEV_MAX_NUM	CONFIG_HOBOT_MIPI_DEV_MAX_NUM
#else
#define MIPI_DEV_MAX_NUM	(2)
#endif

/* port index */
#define MIPI_PORT0			(0)
#define MIPI_PORT1			(1)
#define MIPI_PORT2			(2)
#define MIPI_PORT3			(3)
#define MIPI_PORT4			(4)
#define MIPI_PORT5			(5)

/* ipi index */
#define MIPI_IPI1			(0)
#define MIPI_IPI2			(1)
#define MIPI_IPI3			(2)
#define MIPI_IPI4			(3)

/* vc index */
#define MIPI_VC0			(0)
#define MIPI_VC1			(1)
#define MIPI_VC2			(2)
#define MIPI_VC3			(3)

/* run state */
enum mipi_state_e {
	MIPI_STATE_DEFAULT = 0,
	MIPI_STATE_INIT,
	MIPI_STATE_START,
	MIPI_STATE_STOP,
	MIPI_STATE_MAX,
};
#define MIPI_STATE_NAMES { \
	"default", \
	"init", \
	"start", \
	"stop", \
}

/* mipi dphy tx param struct */
struct mipi_dphy_tx_param_s {
	/* type must be: uint32_t */
	uint32_t txout_param_valid;
	uint32_t txout_freq_mode;
	uint32_t txout_freq_autolarge_enbale;
	uint32_t txout_freq_gain_precent;
	uint32_t txout_freq_force;
};
#define MIPI_DPHY_TX_PARAM_NAMES \
	"txout_param_valid", \
	"txout_freq_mode", \
	"txout_freq_autolarge_enbale", \
	"txout_freq_gain_precent", \
	"txout_freq_force"

#if 0 /* need if not sub of vin */
#define mipi_copy_from_app(l, u, s)	osal_copy_from_app(l, u, s)
#define mipi_copy_to_app(u, l, s)	osal_copy_to_app(u, l, s)
#define mipi_get_app(l, u)		osal_copy_from_app(&l, u, sizeof(l))
#else
#define mipi_copy_from_app(l, u, s)	({ int32_t ret = 0; memcpy(l, u, s); ret; })
#define mipi_copy_to_app(u, l, s)	({ int32_t ret = 0; memcpy(u, l, s); ret; })
#define mipi_get_app(l, u)		({ int32_t ret = 0; memcpy(&l, u, sizeof(int32_t)); ret; })
#endif

/* reg byte offset */
#define MIPI_REG_BYTE0_OFFS	(0)
#define MIPI_REG_BYTE1_OFFS	(8)
#define MIPI_REG_BYTE2_OFFS	(16)
#define MIPI_REG_BYTE3_OFFS	(24)

/* typedef */
typedef uint64_t mipi_flags_t;
typedef long mipi_ioc_ret_t;
typedef long mipi_intptr_t;
typedef unsigned long mipi_ioc_arg_t;

/* callbcak */
typedef void (*MIPI_DROP_CB)(int32_t port, uint32_t ipi_mask);
typedef void (*MIPI_INT_CB)(int32_t port, uint32_t icnt, uint32_t subirq);

/* mipi callbck struct */
struct mipi_cb_s {
	MIPI_DROP_CB       drop_cb;
	MIPI_INT_CB        int_cb;
};

/* sys subtype */
enum {
	MIPI_SYS_SHOW,
	MIPI_SYS_STORE,
	MIPI_SYS_INVALID,
};

/* reg res */
struct mipi_reg_s {
	uint32_t base;
	uint32_t size;
};

#ifndef HB_MIPI_TX_DPHY_OPS_H
/* mipi reg r/w */
static inline uint32_t mipi_getreg(const volatile void __iomem *iomem, uint32_t offs) {
	return (readl((volatile void __iomem *)(&(((const volatile uint8_t __iomem *)iomem)[offs])))); /* PRQA S ALL */ /* linux macro */
}
static inline void mipi_putreg(const volatile void __iomem *iomem, uint32_t offs, uint32_t val) {
	writel(val, (volatile void __iomem *)(&(((volatile uint8_t __iomem *)iomem)[offs]))); /* PRQA S ALL */ /* linux macro */
}

/* IP VERSION */
#define MIPI_IP_VERSION_1P2  (0x31323000)
#define MIPI_IP_VERSION_1P3  (0x31333000)
#define MIPI_IP_VERSION_1P4  (0x31343000)
#define MIPI_IP_VERSION_1P5  (0x31353000)

/* version judge */
static inline int32_t MIPI_VERSION_GE(const volatile void __iomem *iomem, uint32_t v) {
	uint32_t ver = mipi_getreg(iomem, 0U);
	if (ver < MIPI_IP_VERSION_1P2)
		ver = (ver << 8U);
	return (ver >= v) ? 1 : 0;
}

/* mipi reg dump */
struct mipi_dump_reg {
	const char * name;
	uint32_t offset;
};
static inline int32_t mipi_dumpregs(const volatile void __iomem *iomem,
				const struct mipi_dump_reg *regs, char *s, uint32_t count) {
	int32_t l = 0;
	const struct mipi_dump_reg *r = regs;
	while (r->name != NULL) {
		l += snprintf(&s[l], (count -l), "0x%03x: 0x%x\t- %s\n", r->offset,
			mipi_getreg(iomem, r->offset), r->name);
		r++;
	}
	return l;
}

/* reg macro */
static inline uint32_t DP_VMASK(uint32_t mask, int32_t offs) {
	return ((mask) << (uint32_t)(offs));
}
static inline int32_t DP_REG2V(uint32_t regv, uint32_t mask, int32_t offs) {
	return (int32_t)(((regv) >> (uint32_t)(offs)) & (mask)); /* PRQA S 4394 */
}
static inline uint32_t DP_V2REG(int32_t val, uint32_t mask, int32_t offs) {
	return (((uint32_t)(val) & (mask)) << (uint32_t)(offs));
}

/* byte macro */
static inline uint8_t MIPI_BYTE0(uint32_t val) {
	return (uint8_t)(val >> MIPI_REG_BYTE0_OFFS);
}
static inline uint8_t MIPI_BYTE1(uint32_t val) {
	return (uint8_t)(val >> MIPI_REG_BYTE1_OFFS);
}
static inline uint8_t MIPI_BYTE2(uint32_t val) {
	return (uint8_t)(val >> MIPI_REG_BYTE2_OFFS);
}
static inline uint8_t MIPI_BYTE3(uint32_t val) {
	return (uint8_t)(val >> MIPI_REG_BYTE3_OFFS);
}
static inline void MIPI_TOBYTES(uint32_t val, uint8_t *buf, uint32_t l)  {
	uint32_t n = (l > sizeof(uint32_t) ? sizeof(uint32_t) : l);
	uint32_t i;
	for (i = 0U; i < n; i++) {
		buf[i] = (uint8_t)(val >> (i * MIPI_REG_BYTE1_OFFS));
	}
}
#endif

#ifndef HB_MIPI_COMMON_H
#define MIPI_CSI2_DT_YUV420_8   (0x18U)
#define MIPI_CSI2_DT_YUV420_10  (0x19U)
#define MIPI_CSI2_DT_YUV422_8   (0x1EU)
#define MIPI_CSI2_DT_YUV422_10  (0x1FU)
#define MIPI_CSI2_DT_RGB565     (0x22U)
#define MIPI_CSI2_DT_RGB666     (0x23U)
#define MIPI_CSI2_DT_RGB888     (0x24U)
#define MIPI_CSI2_DT_RAW_24     (0x27U) // New added in MIPI CSI-2 v3
#define MIPI_CSI2_DT_RAW_6      (0x28U)
#define MIPI_CSI2_DT_RAW_7      (0x29U)
#define MIPI_CSI2_DT_RAW_8      (0x2AU)
#define MIPI_CSI2_DT_RAW_10     (0x2BU)
#define MIPI_CSI2_DT_RAW_12     (0x2CU)
#define MIPI_CSI2_DT_RAW_14     (0x2DU)
#define MIPI_CSI2_DT_RAW_16     (0x2EU)
#define MIPI_CSI2_DT_RAW_20     (0x2FU)
#endif
#define MIPI_CSI2_DT_RAW_MAX    (0x2FU)
#define MIPI_CSI2_DT_RGB_OR_RAW (0x20U)
#define MIPI_CSI2_DT_EMB_8      (0x12U)

#define MIPI_CSI2_DT_BITS_8	(8U)
#define MIPI_CSI2_DT_BITS_10    (10U)
#define MIPI_CSI2_DT_BITS_12    (12U)
#define MIPI_CSI2_DT_BITS_14    (14U)
#define MIPI_CSI2_DT_BITS_15    (15U)
#define MIPI_CSI2_DT_BITS_16    (16U)
#define MIPI_CSI2_DT_BITS_18    (18U)
#define MIPI_CSI2_DT_BITS_20    (20U)
#define MIPI_CSI2_DT_BITS_24    (24U)
#define MIPI_CSI2_DT_BITS_32    (32U)

static inline uint32_t mipi_datatype2bpp(uint16_t datatype)
{
	uint32_t bits_per_pixel;

	switch (datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_12;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_15;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_16;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_20;
		break;
	case MIPI_CSI2_DT_EMB_8:
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = MIPI_CSI2_DT_BITS_8;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = MIPI_CSI2_DT_BITS_10;
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = MIPI_CSI2_DT_BITS_12;
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = MIPI_CSI2_DT_BITS_14;
		break;
	case MIPI_CSI2_DT_RAW_16:
	case MIPI_CSI2_DT_RGB565:
		bits_per_pixel = MIPI_CSI2_DT_BITS_16;
		break;
	case MIPI_CSI2_DT_RGB666:
		bits_per_pixel = MIPI_CSI2_DT_BITS_18;
		break;
	case MIPI_CSI2_DT_RGB888:
		bits_per_pixel = MIPI_CSI2_DT_BITS_24;
		break;
	case MIPI_CSI2_DT_RAW_20:
		bits_per_pixel = MIPI_CSI2_DT_BITS_20;
		break;
	case MIPI_CSI2_DT_RAW_24:
		bits_per_pixel = MIPI_CSI2_DT_BITS_24;
		break;
	default:
		bits_per_pixel = MIPI_CSI2_DT_BITS_16;
		break;
	}

	return bits_per_pixel;
}

#endif //__HOBOT_MIPI_UTILS_H__
