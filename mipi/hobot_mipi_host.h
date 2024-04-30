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
 * @file hobot_mipi_host.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_HOST_H__
#define __HOBOT_MIPI_HOST_H__

#include <linux/types.h>

#if 1
/* config defined in vin nod */
#include "vin_node_config.h"
#else

/* the ipi channel number and index */
#define MIPIHOST_CHANNEL_NUM (4)
#define MIPIHOST_CHANNEL_0   (0)
#define MIPIHOST_CHANNEL_1   (1)
#define MIPIHOST_CHANNEL_2   (2)
#define MIPIHOST_CHANNEL_3   (3)

/**
 * struct mipi_host_cfg_s - define the configuration to init mipi host
 * @NO{S10E03C01}
 *
 * @phy:	the phy type of host: dphy/cphy, Range: 0:1.
 * @lane:	number of mipi dphy data lane or cphy trio, Range: 1:4.
 * @datatype:	datatype of mipi data packages, Range: 0x00:0x3F.
 * @fps:	frames per second of mipi data to recive.
 * @mclk:	the clock of sensor which output from soc, set 0/1 to disable/enable
 *		only, set <=24 to ignore, set other to enable with freq=mclk*100KHz.
 * @mipiclk:	total bitrate of mipi host with all data lane, Unit: Mbps.
 * @width:	width of image resolution to recive, Unit: pixel.
 * @height:	height of image resolution to recive, Unit: line.
 * @linelenth:	total width lenth of image line to recive, Unit: pixel.
 * @framelenth:	total height lenth of image frame to recive, Unit: line.
 * @settle:	settle time of mipi host dphy, Range: 1:127.
 * @ppi_pg:     configuration of ppi pattern generator, 0 to disable:
 *                  b0: set 1 only to enable ppi pg with default config;
 *                  b1: set 0 to vertical mode, set 1 to horiontal mode;
 *                  b2: set 0 to normal data type, set 1 to emb type;
 *                  b3-4: vc index setting of ppi pg;
 * @hsaTime:	time of horizontal synchronism active for ipi, 0 as default.
 * @hbpTime:	time of horizontal back period for ipi, 0 as default.
 * @hsdTime:	time of horizontal sync porch delay period for ipi, 0 as default.
 * @channle_num:ipi channel number to be used, Range: 0:4.
 * @channle_sel:vc index of each ipi channel to recive, Range: 0:3.
 *
 */
typedef struct mipi_host_cfg_s {
	/* type must be: uint16_t */
	uint16_t phy;
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
	uint16_t ppi_pg;
	uint16_t hsaTime;
	uint16_t hbpTime;
	uint16_t hsdTime;
	uint16_t channel_num;
	uint16_t channel_sel[MIPIHOST_CHANNEL_NUM];
} mipi_host_cfg_t;
#define MIPI_HOST_CFG_NUM	((uint32_t)(sizeof(mipi_host_cfg_t)/sizeof(uint16_t)))

/* config name strings, see: struct mipi_host_cfg_s */
#define MIPI_HOST_CFG_STRINGS { \
	"phy", \
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
	"ppi_pg", \
	"hsaTime", \
	"hbpTime", \
	"hsdTime", \
	"channel_num", \
	"channel_sel0", \
	"channel_sel1", \
	"channel_sel2", \
	"channel_sel3", \
}

/**
 * struct mipi_host_param_s - define the runtime parameters of mipi host
 * @NO{S10E03C01}
 *
 * @nocheck:	disable the check operation when init and start, Range: 0:1.
 * @notimeout:	disable the check timeout and wait forever, Range: 0:1.
 * @wait_ms:	the wait time of check operation, Unit: ms.
 * @dbg_value:	enable the debug print show, Range: 0:1.
 * @adv_value:	the config value of IPI adv_feature register.
 * @need_stop_check: disable the stop check operation, Range: 0:1.
 * @stop_check_instart:	do the stop check operation when start, Range: 0:1.
 * @cut_through: enable the cut_through mode for IPI, Range: 0:1.
 * @mem_flush:	enabel the auto flush function for IPI, Range: 0:1.
 * @data_ids_1:	debug monitor set of ids1.
 * @data_ids_2:	debug monotor set of ids2.
 * @data_ids_vc1: debug monitor set of ids_vc1.
 * @data_ids_vc2: debug monitor set of ids_vc2.
 * @ipi_16bit:	the value of IPI bit mode configuration.
 * @ipi_force:	force set the IPI clock freq, Unit: Hz.
 * @ipi_limit:	teh minimum limit of IPI clock freq set, Unit: Hz.
 * @ipi_overst:	enable the ipi reset operation when ipi overflow, Range: 0:1.
 * @pkt2pkt_time: manual set the pkt2pkt time value, Range: 0:255.
 * @snrclk_en:	enable the sensor mclk output.
 * @snrclk_freq: the sensor mclk output clock freq, Unit: Hz.
 * @vcext_en:	enable extern VC function, Range: 0:1.
 * @error_diag:	enable diag report of soft error.
 * @ipi1_dt:	force datatype config of ipi1.
 * @ipi2_dt:	force datatype config of ipi2.
 * @ipi3_dt:	force datatype config of ipi3.
 * @ipi4_dt:	force datatype config of ipi4.
 * @cfg_nocheck: disable config check function.
 * @drop_func:	frame drop when error function select stl/irq: Range: 0:1.
 * @drop_mask:	disable mask of error type for frame drop function.
 * @irq_cnt:	irq count limit set.
 * @irq_debug:	irq debug print level.
 * @stl_dbg:	stl debug print level.
 * @stl_mask:	stl function mask.
 * @stl_pile:	stl pile function for test.
 * @stl_ovif:	ignore frame count for ipi overflow error.
 * @stl_stif:	ignore frame count for start.
 * @fatal_ap:	fatal inject function select: fun/ap.
 *
 */
typedef struct mipi_host_param_s {
	/* type must be: uint32_t */
	uint32_t nocheck;
	uint32_t notimeout;
	uint32_t wait_ms;
	uint32_t dbg_value;
	uint32_t adv_value;
	uint32_t need_stop_check;
	uint32_t stop_check_instart;
	uint32_t cut_through;
	uint32_t mem_flush;
	uint32_t data_ids_1;
	uint32_t data_ids_2;
	uint32_t data_ids_vc1;
	uint32_t data_ids_vc2;
	uint32_t ipi_16bit;
	uint32_t ipi_force;
	uint32_t ipi_limit;
	uint32_t ipi_overst;
	uint32_t pkt2pkt_time;
	uint32_t snrclk_en;
	uint32_t snrclk_freq;
	uint32_t vcext_en;
	uint32_t error_diag;
	uint32_t ipi1_dt;
	uint32_t ipi2_dt;
	uint32_t ipi3_dt;
	uint32_t ipi4_dt;
	uint32_t cfg_nocheck;
	uint32_t drop_func;
	uint32_t drop_mask;
	uint32_t irq_cnt;
	uint32_t irq_debug;
	uint32_t stl_dbg;
	uint32_t stl_mask;
	uint32_t stl_pile;
	uint32_t stl_ovif;
	uint32_t stl_stif;
	uint32_t fatal_ap;
} mipi_host_param_t;
#define MIPI_HOST_PARAMS_NUM	((uint32_t)(sizeof(struct mipi_host_param_s)/sizeof(uint32_t)))

/* run params name strings, see: struct mipi_host_param_s */
#define MIPI_HOST_PARAM_STRINGS { \
	"nocheck", \
	"notimeout", \
	"wait_ms", \
	"dbg_value", \
	"adv_value", \
	"need_stop_check", \
	"stop_check_instart", \
	"cut_through", \
	"mem_flush", \
	"data_ids_1", \
	"data_ids_2", \
	"data_ids_vc1", \
	"data_ids_vc2", \
	"ipi_16bit", \
	"ipi_force", \
	"ipi_limit", \
	"ipi_overst", \
	"pkt2pkt_time", \
	"snrclk_en", \
	"snrclk_freq", \
	"vcext_en", \
	"error_diag", \
	"ipi1_dt", \
	"ipi2_dt", \
	"ipi3_dt", \
	"ipi4_dt", \
	"cfg_nocheck", \
	"drop_func", \
	"drop_mask", \
	"irq_cnt", \
	"irq_debug", \
	"stl_dbg", \
	"stl_mask", \
	"stl_pile", \
	"stl_ovif", \
	"stl_stif", \
	"fatal_ap", \
}
#endif

/**
 * struct mipi_host_ipi_reset_s - define the struct to reset ipi
 * @NO{S10E03C01}
 *
 * @mask:		bit mask of each ipi to operate(1) or ignore(0).
 * @enable:		set 0 to enable ipi, set 1 to disable ipi.
 */
typedef struct mipi_host_ipi_reset_s {
	uint16_t mask;
	uint16_t enable;
} mipi_host_ipi_reset_t;

/**
 * struct mipi_host_ipi_info_s - define the struct to get/set ipi info
 * @NO{S10E03C01}
 *
 * @index:		ipi channel index, Range: 0:3.
 * @vc:			vc index config of ipi channel, Range: 0:3.
 * @fatal:		for get: the value of ipi fatal register.
 *              for set: the bit mask of set operate:
 *					b15: 0-all set, 1-masked set;
 *					b0: mod; b1: vc; b2: datatype;
 *					b4: hsa; b5: hbp; b6: hsd; b7: adv;
 * @datatype:	datatype of ipi channel, Range: 0x00:0x3F.
 * @hsa:		time of ipi horizontal synchronism active.
 * @hbp:		time of ipi horizontal back period.
 * @hsd:		time of ipi horizontal sync porch delay period.
 * @adv:		advanced features configuration of ipi.
 * @mode:		mode configuration of ipi.
 */
typedef struct mipi_host_ipi_info_s {
	uint8_t index;
	uint8_t vc;
	uint16_t fatal;
	uint16_t datatype;
	uint16_t hsa;
	uint16_t hbp;
	uint16_t hsd;
	uint32_t adv;
	uint32_t mode;
} mipi_host_ipi_info_t;

/**
 * struct mipi_host_reg_s - define the struct to operate register of mipi host
 * @NO{S10E03C01}
 *
 * @offset:		the address offset of register to operate.
 * @value:		the value of register to read or write.
 */
typedef struct mipi_host_reg_s {
	uint32_t offset;
	uint32_t value;
} mipi_host_reg_t;

#define MIPIHOSTIOC_MAGIC 'v'
#define MIPIHOSTIOC_INIT             _IOW(MIPIHOSTIOC_MAGIC, 0, mipi_host_cfg_t)
#define MIPIHOSTIOC_DEINIT           _IO(MIPIHOSTIOC_MAGIC,  1)
#define MIPIHOSTIOC_START            _IO(MIPIHOSTIOC_MAGIC,  2)
#define MIPIHOSTIOC_STOP             _IO(MIPIHOSTIOC_MAGIC,  3)
#define MIPIHOSTIOC_SNRCLK_SET_EN    _IOW(MIPIHOSTIOC_MAGIC, 4, uint32_t)
#define MIPIHOSTIOC_SNRCLK_SET_FREQ  _IOW(MIPIHOSTIOC_MAGIC, 5, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 6, uint32_t)
#define MIPIHOSTIOC_PRE_START_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 7, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_RESULT  _IOW(MIPIHOSTIOC_MAGIC, 8, uint32_t)
#define MIPIHOSTIOC_PRE_START_RESULT _IOW(MIPIHOSTIOC_MAGIC, 9, uint32_t)
#define MIPIHOSTIOC_IPI_RESET        _IOW(MIPIHOSTIOC_MAGIC, 10, mipi_host_ipi_reset_t)
#define MIPIHOSTIOC_IPI_GET_INFO     _IOR(MIPIHOSTIOC_MAGIC, 11, mipi_host_ipi_info_t)
#define MIPIHOSTIOC_IPI_SET_INFO     _IOW(MIPIHOSTIOC_MAGIC, 12, mipi_host_ipi_info_t)
#define MIPIHOSTIOC_GET_PARAM        _IOR(MIPIHOSTIOC_MAGIC, 13, mipi_host_param_t)
#define MIPIHOSTIOC_SET_PARAM        _IOW(MIPIHOSTIOC_MAGIC, 14, mipi_host_param_t)

#define MIPIHOSTIOC_READ		     _IOWR(MIPIHOSTIOC_MAGIC, 16, mipi_host_reg_t)
#define MIPIHOSTIOC_WRITE		     _IOW(MIPIHOSTIOC_MAGIC, 17, mipi_host_reg_t)

#endif /*__HOBOT_MIPI_HOST_H__*/
