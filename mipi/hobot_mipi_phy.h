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
 * @file hobot_mipi_phy.h
 *
 * @NO{S10E03C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_PHY_H__
#define __HOBOT_MIPI_PHY_H__

#include <linux/types.h>

#include "hobot_mipi_utils.h"

/**
 * struct mipi_phy_sub_s - define the dphy node of mipi host or dev
 * @NO{S10E03C02}
 *
 * @iomem:		mapped address of host or dev.
 * @dev:		device pointer of host or dev.
 * @param:		addition param point of host or dev.
 * @port:		port index of host or dev.
 */
struct mipi_phy_sub_s {
	void __iomem  *iomem;
	struct os_dev *osdev;
	void          *param;
	int32_t        port;
};

/**
 * enum - define the type of dphy operate apis
 * @NO{S10E03C02}
 *
 * @MIPI_DPHY_TYPE_HOST:	mipi csi host type.
 * @MIPI_DPHY_TYPE_DEV:		mipi csi dev tyep.
 * @MIPI_DPHY_TYPE_DSI:		mipi dsi tyep.
 */
enum {
	MIPI_DPHY_TYPE_HOST,
	MIPI_DPHY_TYPE_DEV,
	MIPI_DPHY_TYPE_DSI,
};

/**
 * enum - define the region of dphy freqrang operate apis
 * @NO{S10E03C02}
 *
 * @MIPI_CFGCLKFREQRANGE:	cfgclk_freqrange region.
 * @MIPI_HSFREQRANGE:		hs_freqrange region.
 */
enum {
	MIPI_CFGCLKFREQRANGE,
	MIPI_HSFREQRANGE,
};

/**
 * enum - define the region of dphy ctl operate apis
 * @NO{S10E03C02}
 *
 * @MIPI_BYPASS_GEN_HSYNC_DLY_CNT:	bypass_gen_hsync_dly_cnt region.
 * @MIPI_BYPASS_GEN_HSYNC_EN:	bypass_gen_hsync_en region.
 * @MIPI_DEV_SHADOW_CLEAR:		dev_shadow_clear region.
 * @MIPI_DEV_SHADOW_CONTROL:	dev_shadow_control region.
 * @MIPI_DEV_SEL_CLR:			dev_sel_ctl region.
 */
enum {
	MIPI_BYPASS_GEN_HSYNC_DLY_CNT,
	MIPI_BYPASS_GEN_HSYNC_EN,
	MIPI_DEV_SHADOW_CLEAR,
	MIPI_DEV_SHADOW_CONTROL,
	MIPI_DEV_SEL_CLR,
};

#ifdef X5_CHIP
/**
 * enum - define the region of dphy cfg operate apis
 * @NO{}
 *
 * @MIPI_PHY_ENABLE_CLK:	mipi phy enable clk region.
 * @MIPI_PHY_FORCE_RXMODE:	mipi phy forece enter rx mode region.
 * @MIPI_PHY_HS_FREQRANGE:	mipi phy set hs freqrange region.
 * @MIPI_PHY_CTRL_SEL:		mipi phy set  2lane or 4lane region.
 */
enum {
	MIPI_PHY_ENABLE_CLK,
	MIPI_PHY_FORCE_RXMODE,
	MIPI_PHY_HS_FREQRANGE,
	MIPI_PHY_CTRL_SEL,
        MIPI_CFG_CLK_FREQRANGE,
};
#endif

/**
 * struct mipi_phy_reg_s - define the struct to operate register of mipi dphy
 * @NO{S10E03C02}
 *
 * @offset:		the address offset of register to operate.
 * @port:		the device port index to operate.
 * @type:		the device type to operate.
 * @value:		the value of register to read or write.
 */
typedef struct mipi_phy_reg_s {
	uint16_t offset;
	uint8_t port;
	uint8_t type;
	uint32_t value;
} mipi_phy_reg_t;

#define MIPIDPHYIOC_MAGIC 'v'
#define MIPIDPHYTIOC_READ        _IOWR(MIPIDPHYIOC_MAGIC, 16, mipi_phy_reg_t)
#define MIPIDPHYTIOC_WRITE       _IOW(MIPIDPHYIOC_MAGIC, 17, mipi_phy_reg_t)

/**
 * struct mipi_phy_register - register a host or dev device to dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST/MIPI_DPHY_TYPE_DEV.
 * @param[in] port:		port index of this type.
 * @param[in] sub:		sub info of this device.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_phy_register(int32_t type, int32_t port, const struct mipi_phy_sub_s *sub);
/**
 * struct mipi_phy_unregister - unregister a host or dev device from dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST/MIPI_DPHY_TYPE_DEV.
 * @param[in] port:		port index of this type.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_phy_unregister(int32_t type, int32_t port);

extern int32_t mipi_host_dphy_set_source(const void __iomem *iomem);

/**
 * struct mipi_host_dphy_initialize - initialize mipi host dphy
 *
 * @param[in] mipiclk:	total bitrate of all data lane, Unit: Mbps.
 * @param[in] lane:		number of data lane, Range: 1:4.
 * @param[in] settle:	settle setting of dphy.
 * @param[in] iomem:	mapped address of this host device.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_host_dphy_initialize(uint16_t mipiclk, uint16_t lane, uint16_t settle, const void __iomem *iomem);
/**
 * struct mipi_host_dphy_reset - reset mipi host dphy
 *
 * @param[in] iomem:	mapped address of this host device.
 *
 * @return void:		NONE.
 */
extern void    mipi_host_dphy_reset(const void __iomem *iomem);

/**
 * struct mipi_dev_dphy_initialize - initialize mipi dev dphy
 *
 * @param[in] iomem:	mapped address of this dev device.
 * @param[in] mipiclk:	total bitrate of all data lane, Unit: Mbps.
 * @param[in] lane:		number of data lane, Range: 1:4.
 * @param[in] settle:	settle setting of dphy.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_dev_dphy_initialize(const void __iomem *iomem, uint16_t mipiclk, uint16_t lane, uint16_t settle);
/**
 * struct mipi_dev_dphy_reset - reset mipi dev dphy
 *
 * @param[in] iomem:	mapped address of this dev device.
 *
 * @return void:		NONE.
 */
extern void    mipi_dev_dphy_reset(const void __iomem *iomem);

/**
 * struct mipi_dphy_get_ctl - get the ctl setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 * @param[in] region:	region as MIPI_BYPASS_GEN_HSYNC_DLY_CNT,etc.
 *
 * @return int32_t:		ctl setting value(>=0-value, <0-errorcode).
 */
extern int32_t mipi_dphy_get_ctl(int32_t type, int32_t port, int32_t region);
/**
 * struct mipi_dphy_set_ctl - set the ctl setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 * @param[in] region:	region as MIPI_BYPASS_GEN_HSYNC_DLY_CNT,etc.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_dphy_set_ctl(int32_t type, int32_t port, int32_t region, int32_t value);
/**
 * struct mipi_dphy_get_freqrange - get the freqrange setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST/MIPI_DPHY_TYPE_DEV.
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 * @param[in] region:	region as MIPI_CFGCLKFREQRANGE,etc.
 *
 * @return int32_t:		freqrange setting value(>=0-value, <0-errorcode).
 */
extern int32_t mipi_dphy_get_freqrange(int32_t type, int32_t port, int32_t region);
/**
 * struct mipi_dphy_set_freqrange - set the freqrange setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 * @param[in] region:	region as MIPI_CFGCLKFREQRANGE,etc.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_dphy_set_freqrange(int32_t type, int32_t port, int32_t region, int32_t value);
/**
 * struct mipi_dphy_get_lanemode - get the lanemode setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 *
 * @return int32_t:		lane mode(<0-errorcodem, host:0-alone,1-group, dev:0-csi,1-dsi).
 */
extern int32_t mipi_dphy_get_lanemode(int32_t type, int32_t port);
/**
 * struct mipi_dphy_set_lanemode - set the lanemode setting of dphy
 *
 * @param[in] type:		type as MIPI_DPHY_TYPE_HOST,etc.
 * @param[in] port:		port index of this type.
 * @param[in] lanemode:	host:0-alone,1-group, dev:0-csi,1-dsi.
 *
 * @return int32_t:		error code(0-sucess, <0-errorcode).
 */
extern int32_t mipi_dphy_set_lanemode(int32_t type, int32_t port, int32_t lanemode);
/**
 * struct mipi_dphy_outclk_config - config the ouclk for sensor mclk
 *
 * @param[in] name:		the clock name to config.
 * @param[in] freq_set:	the output frequency to set, Unit: Hz.
 * @param[out] freq_get:	the real output frequency to get, Unit: Hz.
 *
 * @return int32_t:		lane mode(<0-errorcodem, host:0-alone,1-group, dev:0-csi,1-dsi).
 */
extern int32_t mipi_dphy_outclk_config(const char *name, uint64_t freq_set, uint64_t *freq_get);

/**
 * struct mipi_dphy_class - get the parent class of mipi devices
 *
 * @param[in] void:		NONE.
 *
 * @return class:		parent class pointer.
 */
extern struct class* mipi_phy_class(void);


/**
 * @struct mipi_phy_ops_s
 * sub driver ops struct of mipi phy
 * @NO{S10E03C02}
 */
typedef struct mipi_phy_ops_s {
	int32_t (*ioctl)(uint32_t cmd, mipi_ioc_arg_t arg);
	int32_t (*sys)(int32_t type, int32_t sub, const char *name, char *buf, int32_t count);
	os_dev_t* (*osdev)(void);
} mipi_phy_ops_t;

/**
 * @struct mipi_phy_dbg_ops_s
 * debug driver ops struct of mipi phy
 * @NO{S10E03C02}
 */
typedef struct mipi_phy_dbg_ops_s {
	int32_t (*setup)(const mipi_phy_ops_t *ops);
	void* (*class)(int32_t op);
} mipi_phy_dbg_ops_t;

extern int32_t hobot_mipi_phy_debug_setup(const mipi_phy_dbg_ops_t *phy_dbg_ops);

extern int32_t mipi_dphy_set_testcode(int32_t type, int32_t port, int32_t testcode);
extern int32_t mipi_dphy_set_test_sel_4l(int32_t type, int32_t port, int32_t value);


#endif /*__HOBOT_MIPI_PHY_H__*/
