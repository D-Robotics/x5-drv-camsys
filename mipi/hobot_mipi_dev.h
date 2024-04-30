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
 * @file hobot_mipi_dev.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_DEV_H__
#define __HOBOT_MIPI_DEV_H__

#include <linux/types.h>

#if 1
/* config defined in vin nod */
#include "vin_node_config.h"
#else

/* the ipi channel number and index */
#define MIPIDEV_CHANNEL_NUM (4)
#define MIPIDEV_CHANNEL_0   (0)
#define MIPIDEV_CHANNEL_1   (1)
#define MIPIDEV_CHANNEL_2   (2)
#define MIPIDEV_CHANNEL_3   (3)

/**
 * struct mipi_dev_cfg_s - define the configuration to init mipi dev
 * @NO{S10E03C01}
 *
 * @lane:		number of mipi dphy data lane, Range: 1:4.
 * @datatype:	datatype of mipi data packages, Range: 0x00:0x3F.
 * @fps:		frames per second of mipi data to send.
 * @mclk:		ignored for mipi dev.
 * @mipiclk:	total bitrate of mipi dev with all data lane, Unit: Mbps.
 * @width:		width of image resolution to send, Unit: pixel.
 * @height:		height of image resolution to send, Unit: line.
 * @linelenth:	total width lenth of image line to send, Unit: pixel.
 * @framelenth:	total height lenth of image frame to send, Unit: line.
 * @settle:		settle time of mipi dev dphy, Range: 1:127.
 * @vpg:		configuration of video pattern generator, 0 to disable:
 *					b0: set 1 only to enable vpg with default config;
 *					b1: set 0 to vertical mode, set 1 to horiontal mode;
 *					b2: set 0 to BER pattern, set 1 to Color bar;
 *					b3-4: vc index setting of vpg;
 * @ipi_lines:	lines setting of ipi channel to send, 0 as height+1.
 * @channle_num:	ipi channel number to be used, Range: 0:4.
 * @channle_sel:	vc index of each ipi channel to recive, Range: 0:3.
 *
 */
typedef struct mipi_dev_cfg_s {
	/* type must be: uint16_t */
	uint16_t lane;
	uint16_t datatype;
	uint16_t fps;
	uint16_t mclk;
	uint16_t mipiclk;
	uint16_t width;
	uint16_t height;
	uint16_t linelenth;
	uint16_t framelenth;
	uint16_t settle;
	uint16_t vpg;
	uint16_t ipi_lines;
	uint16_t channel_num;
	uint16_t channel_sel[MIPIDEV_CHANNEL_NUM];
} mipi_dev_cfg_t;
#define MIPI_DEV_CFG_NUM	((uint32_t)(sizeof(mipi_dev_cfg_t)/sizeof(uint16_t)))

/* config name strings, see: struct mipi_dev_cfg_s */
#define MIPI_DEV_CFG_STRINGS { \
	"lane", \
	"datatype", \
	"fps", \
	"mclk", \
	"mipiclk", \
	"width", \
	"height", \
	"linelenth", \
	"framelenth", \
	"settle", \
	"vpg", \
	"ipi_lines", \
	"channel_num", \
	"channel_sel0", \
	"channel_sel1", \
	"channel_sel2", \
	"channel_sel3", \
}

/**
 * struct mipi_dev_param_s - define the runtime parameters of mipi dev
 * @NO{S10E03C01}
 *
 * @nocheck:	disable the check operation when init and start, Range: 0:1.
 * @notimeout:	disable the check timeout and wait forever, Range: 0:1.
 * @wait_ms:	the wait time of check operation, Unit: ms.
 * @dbg_value:	enable the debug print show, Range: 0:1.
 * @power_instart: do the power check when start, Range: 0:1.
 * @hsync_pkt:	enable the hsync packet of IPI, Range: 0:1.
 * @init_retry:	enable retry when init failed , Range: 0:1.
 * @ipi_force:	force set the IPI clock freq, Unit: Hz.
 * @ipi_limit:	teh minimum limit of IPI clock freq set, Unit: Hz.
 * @error_diag:	enable diag report of soft error.
 * @ipi1_dt:	force datatype config of ipi1.
 * @ipi2_dt:	force datatype config of ipi2.
 * @ipi3_dt:	force datatype config of ipi3.
 * @ipi4_dt:	force datatype config of ipi4.
 * @cfg_nocheck: disable config check function.
 * @irq_cnt:	irq count limit set.
 * @irq_debug:	irq debug print level.
 * @stl_dbg:	stl debug print level.
 * @stl_mask:	stl function mask.
 * @stl_pile:	stl pile function for test.
 * @fatal_ap:	fatal inject function select: fun/ap.
 * @txout_param_valid: set the txout param vaild, Range: 0:1.
 * @txout_freq_mode: set the txout freq mode as auto/cal, Range: 0:1.
 * @txout_freq_autolarge_enbale: enable the txout freq auto large, Range: 0:1.
 * @txout_freq_gain_precent: set the txout freq gain precent, Range: 0:100.
 * @txout_freq_force: set the txout freq force, Unit: Hz.
 *
 */
typedef struct mipi_dev_param_s {
	/* type must be: uint32_t */
	uint32_t nocheck;
	uint32_t notimeout;
	uint32_t wait_ms;
	uint32_t dbg_value;
	uint32_t power_instart;
	uint32_t hsync_pkt;
	uint32_t init_retry;
	uint32_t ipi_force;
	uint32_t ipi_limit;
	uint32_t error_diag;
	uint32_t ipi1_dt;
	uint32_t ipi2_dt;
	uint32_t ipi3_dt;
	uint32_t ipi4_dt;
	uint32_t cfg_nocheck;
	uint32_t irq_cnt;
	uint32_t irq_debug;
	uint32_t stl_dbg;
	uint32_t stl_mask;
	uint32_t stl_pile;
	uint32_t fatal_ap;
	uint32_t txout_param_valid;
	uint32_t txout_freq_mode;
	uint32_t txout_freq_autolarge_enbale;
	uint32_t txout_freq_gain_precent;
	uint32_t txout_freq_force;
} mipi_dev_param_t;
#define MIPI_DEV_PARAMS_NUM	((uint32_t)(sizeof(struct mipi_dev_param_s)/sizeof(uint32_t)))

/* run params name strings, see: struct mipi_host_param_s */
#define MIPI_DEV_PARAM_STRINGS { \
	"nocheck", \
	"notimeout", \
	"wait_ms", \
	"dbg_value", \
	"power_instart", \
	"hsync_pkt", \
	"init_retry", \
	"ipi_force", \
	"ipi_limit", \
	"error_diag", \
	"ipi1_dt", \
	"ipi2_dt", \
	"ipi3_dt", \
	"ipi4_dt", \
	"cfg_nocheck", \
	"irq_cnt", \
	"irq_debug", \
	"stl_dbg", \
	"stl_mask", \
	"stl_pile", \
	"fatal_ap", \
	"txout_param_valid", \
	"txout_freq_mode", \
	"txout_freq_autolarge_enbale", \
	"txout_freq_gain_precent", \
	"txout_freq_force", \
}

/**
 * struct mipi_dev_ipi_info_s - define the struct to get/set ipi info
 * @NO{S10E03C01}
 *
 * @index:		ipi channel index, Range: 0:3.
 * @fatal:		for get: the value of ipi fatal register.
 *              for set: the bit mask of set operate:
 *					b15: 0-all set, 1-masked set;
 *					b0: mod; b1: vc; b2: datatype;
 *					b3: maxfnum; b4: pixels; b5: lines;
 * @mode:		mode configuration of ipi.
 * @vc:			vc index config of ipi channel, Range: 0:3.
 * @datatype:	datatype of ipi channel, Range: 0x00:0x3F.
 * @maxfnum:	max frame number setting of ipi frame, 0 as default.
 * @pixels:		pixel number of ipi data packet.
 * @lines:		line number of a frame of ipi.
 */
typedef struct mipi_dev_ipi_info_s {
	uint16_t index;
	uint16_t fatal;
	uint16_t mode;
	uint16_t vc;
	uint16_t datatype;
	uint16_t maxfnum;
	uint32_t pixels;
	uint32_t lines;
} mipi_dev_ipi_info_t;
#endif

/**
 * struct mipi_dev_reg_s - define the struct to operate register of mipi dev
 * @NO{S10E03C01}
 *
 * @offset:		the address offset of register to operate.
 * @value:		the value of register to read or write.
 */
typedef struct mipi_dev_reg_s {
	uint32_t offset;
	uint32_t value;
} mipi_dev_reg_t;

#define MIPIDEVIOC_MAGIC 'v'
#define MIPIDEVIOC_INIT             _IOW(MIPIDEVIOC_MAGIC, 0, mipi_dev_cfg_t)
#define MIPIDEVIOC_DEINIT           _IO(MIPIDEVIOC_MAGIC,  1)
#define MIPIDEVIOC_START            _IO(MIPIDEVIOC_MAGIC,  2)
#define MIPIDEVIOC_STOP             _IO(MIPIDEVIOC_MAGIC,  3)
#define MIPIDEVIOC_IPI_GET_INFO     _IOR(MIPIDEVIOC_MAGIC, 4, mipi_dev_ipi_info_t)
#define MIPIDEVIOC_IPI_SET_INFO     _IOW(MIPIDEVIOC_MAGIC, 5, mipi_dev_ipi_info_t)
#define MIPIDEVIOC_GET_PARAM        _IOR(MIPIDEVIOC_MAGIC, 6, mipi_dev_param_t)
#define MIPIDEVIOC_SET_PARAM        _IOW(MIPIDEVIOC_MAGIC, 7, mipi_dev_param_t)
#define MIPIDEVIOC_SET_BYPASS       _IOW(MIPIDEVIOC_MAGIC, 8, uint32_t)

#define MIPIDEVIOC_READ		        _IOWR(MIPIDEVIOC_MAGIC, 16, mipi_dev_reg_t)
#define MIPIDEVIOC_WRITE	        _IOW(MIPIDEVIOC_MAGIC, 17, mipi_dev_reg_t)

#endif /*__HOBOT_MIPI_DEV_H__*/
