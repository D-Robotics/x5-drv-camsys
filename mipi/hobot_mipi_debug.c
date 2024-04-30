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
 * @file hobot_mipi_debug.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "hobot_mipi_csi.h"

static char *class;
module_param(class, charp, 0644); /* PRQA S 0605,0636,4501 */ /* module_param macro */

#define MIPI_CSI_DBG_IOC_MAX	(32)
#define MIPI_CSI_DBG_INT_MAX	(32)

typedef struct mipi_csidbg_call_s {
	uint32_t count_all;
	uint32_t error_all;
	uint32_t count;
	uint32_t error;
	uint64_t in_ns;
	uint64_t out_ns;
} mipi_csidbg_call_t;

typedef struct mipi_csidbg_dev_s {
	struct mipi_csidbg_call_s open;
	struct mipi_csidbg_call_s close;
	struct mipi_csidbg_call_s ioctl[MIPI_CSI_DBG_IOC_MAX];
	struct mipi_csidbg_call_s intcb[MIPI_CSI_DBG_INT_MAX];
	int32_t open_cnt;
} mipi_csidbg_dev_t;

typedef struct mipi_csidbg_s {
	struct mipi_csidbg_dev_s *rxs;
	struct mipi_csidbg_dev_s *txs;
	struct mipi_csidbg_dev_s *phy;
	int32_t rx_num;
	uint32_t rx_mask;
	int32_t tx_num;
	uint32_t tx_mask;
	struct class *csi_class;
	struct class *p_class;
	int32_t class_cnt;
} mipi_csidbg_t;

#define MIPI_CSI_DBG_OPEN_BASE	(0)
#define MIPI_CSI_DBG_CLOSE_BASE	(1)
#define MIPI_CSI_DBG_IOC_BASE	(2)
#define MIPI_CSI_DBG_INT_BASE	(MIPI_CSI_DBG_IOC_BASE + MIPI_CSI_DBG_IOC_MAX)
#define MIPI_CSI_DBG_CALL_NUM	(MIPI_CSI_DBG_INT_BASE + MIPI_CSI_DBG_INT_MAX)
static const char *g_cdbg_rxcalls [MIPI_CSI_DBG_CALL_NUM] = {
	[MIPI_CSI_DBG_OPEN_BASE] = "open",
	[MIPI_CSI_DBG_CLOSE_BASE] = "close",
	[MIPI_CSI_DBG_IOC_BASE + 0] = "init",
	[MIPI_CSI_DBG_IOC_BASE + 1] = "deinit",
	[MIPI_CSI_DBG_IOC_BASE + 2] = "start",
	[MIPI_CSI_DBG_IOC_BASE + 3] = "stop",
	[MIPI_CSI_DBG_IOC_BASE + 4] = "snrclk_set_en",
	[MIPI_CSI_DBG_IOC_BASE + 5] = "snrclk_set_freq",
	[MIPI_CSI_DBG_IOC_BASE + 6] = "pre_init_request",
	[MIPI_CSI_DBG_IOC_BASE + 7] = "pre_start_request",
	[MIPI_CSI_DBG_IOC_BASE + 8] = "pre_init_result",
	[MIPI_CSI_DBG_IOC_BASE + 9] = "pre_start_result",
	[MIPI_CSI_DBG_IOC_BASE + 10] = "ipi_reset",
	[MIPI_CSI_DBG_IOC_BASE + 11] = "ipi_get_info",
	[MIPI_CSI_DBG_IOC_BASE + 12] = "ipi_set_info",
	[MIPI_CSI_DBG_IOC_BASE + 13] = "ipi_get_param",
	[MIPI_CSI_DBG_IOC_BASE + 14] = "ipi_set_param",
	[MIPI_CSI_DBG_IOC_BASE + 16] = "read",
	[MIPI_CSI_DBG_IOC_BASE + 17] = "write",
	[MIPI_CSI_DBG_INT_BASE + 0] = "st_main",
	[MIPI_CSI_DBG_INT_BASE + 1] = "phy_fatal",
	[MIPI_CSI_DBG_INT_BASE + 2] = "pkt_fatal",
	[MIPI_CSI_DBG_INT_BASE + 3] = "frm_fatal",
	[MIPI_CSI_DBG_INT_BASE + 4] = "bndry_frm_fatal",
	[MIPI_CSI_DBG_INT_BASE + 5] = "seq_frm_fatal",
	[MIPI_CSI_DBG_INT_BASE + 6] = "crc_frm_fatal",
	[MIPI_CSI_DBG_INT_BASE + 7] = "pld_frm_fatal",
	[MIPI_CSI_DBG_INT_BASE + 8] = "data_id",
	[MIPI_CSI_DBG_INT_BASE + 9] = "ecc_corrected",
	[MIPI_CSI_DBG_INT_BASE + 10] = "phy",
	[MIPI_CSI_DBG_INT_BASE + 11] = "pkt",
	[MIPI_CSI_DBG_INT_BASE + 12] = "line",
	[MIPI_CSI_DBG_INT_BASE + 13] = "ipi",
	[MIPI_CSI_DBG_INT_BASE + 14] = "ipi2",
	[MIPI_CSI_DBG_INT_BASE + 15] = "ipi3",
	[MIPI_CSI_DBG_INT_BASE + 16] = "ipi4",
	[MIPI_CSI_DBG_INT_BASE + 17] = "ap_generic",
	[MIPI_CSI_DBG_INT_BASE + 18] = "ap_ipi",
	[MIPI_CSI_DBG_INT_BASE + 19] = "ap_ipi2",
	[MIPI_CSI_DBG_INT_BASE + 20] = "ap_ipi3",
	[MIPI_CSI_DBG_INT_BASE + 21] = "ap_ipi4",
};
static const char *g_cdbg_txcalls [MIPI_CSI_DBG_CALL_NUM] = {
	[MIPI_CSI_DBG_OPEN_BASE] = "open",
	[MIPI_CSI_DBG_CLOSE_BASE] = "close",
	[MIPI_CSI_DBG_IOC_BASE + 0] = "init",
	[MIPI_CSI_DBG_IOC_BASE + 1] = "deinit",
	[MIPI_CSI_DBG_IOC_BASE + 2] = "start",
	[MIPI_CSI_DBG_IOC_BASE + 3] = "stop",
	[MIPI_CSI_DBG_IOC_BASE + 4] = "ipi_get_info",
	[MIPI_CSI_DBG_IOC_BASE + 5] = "ipi_set_info",
	[MIPI_CSI_DBG_IOC_BASE + 6] = "ipi_get_param",
	[MIPI_CSI_DBG_IOC_BASE + 7] = "ipi_set_param",
	[MIPI_CSI_DBG_IOC_BASE + 16] = "read",
	[MIPI_CSI_DBG_IOC_BASE + 17] = "write",
	[MIPI_CSI_DBG_INT_BASE + 0] = "st_main",
	[MIPI_CSI_DBG_INT_BASE + 1] = "vpg",
	[MIPI_CSI_DBG_INT_BASE + 2] = "ipi",
	[MIPI_CSI_DBG_INT_BASE + 3] = "idi",
	[MIPI_CSI_DBG_INT_BASE + 4] = "phy",
	[MIPI_CSI_DBG_INT_BASE + 5] = "mt_ipi",
	[MIPI_CSI_DBG_INT_BASE + 6] = "idi_vcx",
	[MIPI_CSI_DBG_INT_BASE + 7] = "idi_vcx2",
	[MIPI_CSI_DBG_INT_BASE + 8] = "diag0",
	[MIPI_CSI_DBG_INT_BASE + 9] = "diag_ipi",
	[MIPI_CSI_DBG_INT_BASE + 10] = "diag_idi",
	[MIPI_CSI_DBG_INT_BASE + 11] = "diag_ambaapbintf",
	[MIPI_CSI_DBG_INT_BASE + 12] = "diag_phy_if_ctrl",
	[MIPI_CSI_DBG_INT_BASE + 13] = "diag_reg_bank",
	[MIPI_CSI_DBG_INT_BASE + 14] = "diag_ipi_fifoctrl",
	[MIPI_CSI_DBG_INT_BASE + 15] = "diag_idi_fifoctrl",
	[MIPI_CSI_DBG_INT_BASE + 16] = "diag_pkt_builder",
	[MIPI_CSI_DBG_INT_BASE + 17] = "diag_err_handler",
	[MIPI_CSI_DBG_INT_BASE + 18] = "diag_sync",
	[MIPI_CSI_DBG_INT_BASE + 19] = "diag_pkt_if",
	[MIPI_CSI_DBG_INT_BASE + 20] = "diag_ecf",
	[MIPI_CSI_DBG_INT_BASE + 21] = "diag_cmu",
	[MIPI_CSI_DBG_INT_BASE + 22] = "diag_mt_ipi_ctrl",
	[MIPI_CSI_DBG_INT_BASE + 23] = "diag_ipi2",
	[MIPI_CSI_DBG_INT_BASE + 24] = "diag_ipi2_fifoctrl",
	[MIPI_CSI_DBG_INT_BASE + 25] = "diag_ipi3",
	[MIPI_CSI_DBG_INT_BASE + 26] = "diag_ipi3_fifoctrl",
	[MIPI_CSI_DBG_INT_BASE + 27] = "diag_ipi4",
	[MIPI_CSI_DBG_INT_BASE + 28] = "diag_ipi4_fifoctrl",
};
static struct mipi_csidbg_s g_cdbg = {};

static void *hobot_mipi_csi_class_get(void)
{
	struct class *p_class = NULL;
	const char *cname = (class != NULL) ? class : MIPI_MODULE_CLASS_NAME;

	if (g_cdbg.csi_class != NULL)
		return g_cdbg.csi_class;

	if (g_cdbg.p_class == NULL) {
		p_class = class_create(THIS_MODULE, cname);
		if (IS_ERR((void *)p_class)) {
			p_class = NULL;
			mipi_err(NULL, "[%s] class %s error %d\n", __func__,
					cname, (int32_t)PTR_ERR((void *)p_class));
		}
		g_cdbg.class_cnt = 0;
		g_cdbg.p_class = p_class;
		g_cdbg.class_cnt ++;
	} else {
		p_class = g_cdbg.p_class;
		g_cdbg.class_cnt ++;
	}
	return p_class;
}

static void hobot_mipi_csi_class_put(void)
{
	if (g_cdbg.csi_class != NULL)
		return;

	if (g_cdbg.p_class == NULL)
		return;

	if (g_cdbg.class_cnt == 0)
		return;

	g_cdbg.class_cnt --;
	if (g_cdbg.class_cnt == 0) {
		class_destroy(g_cdbg.p_class);
		g_cdbg.p_class = NULL;
	}
}

static void *hobot_mipi_csi_class(int32_t op)
{
	if (op)
		return hobot_mipi_csi_class_get();

	hobot_mipi_csi_class_put();
	return NULL;
}

extern int32_t hobot_mipi_host_debug_remove(uint32_t mask);
extern int32_t hobot_mipi_host_debug_probe(uint32_t mask, const struct mipi_sub_ops_s *ops,
			const struct mipi_dbg_ops_s *dops);
static const struct mipi_dbg_ops_s mipi_csi_rx_debug_ops;

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug hook call info clear
 *
 * @param[in] call: mipi host(rx) call node point
 * @param[in] clear_all: need clear all info?
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_debug_call_clear(struct mipi_csidbg_call_s *call, int32_t clear_all)
{
	if (clear_all == 0) {
		call->count = 0U;
		call->error = 0U;
		call->in_ns = 0UL;
		call->out_ns = 0UL;
		return;
	}

	memset(call, 0, sizeof(struct mipi_csidbg_call_s));
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug hook call: open
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] call: mipi host(rx) call node point
 * @param[in] name: mipi host(rx) call name string
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_debug_call(mipi_dbg_hook_e type, struct mipi_csidbg_call_s *call, const char *name)
{
	if (call == NULL)
		return;

	switch (type) {
	case MIPI_CSI_DBG_HOOKCALL:
		call->count ++;
		call->count_all ++;
		call->in_ns = osal_time_get_ns();
		call->out_ns = osal_time_get_ns();
		break;
	case MIPI_CSI_DBG_HOOKPRE:
		call->count ++;
		call->count_all ++;
		call->in_ns = osal_time_get_ns();
		break;
	case MIPI_CSI_DBG_HOOKPOST:
		call->out_ns = osal_time_get_ns();
		break;
	case MIPI_CSI_DBG_HOOKERR:
	default:
		call->error ++;
		call->error_all ++;
		call->out_ns = osal_time_get_ns();
		break;
	}
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug function setup
 *
 * @param[in] mask: mipi host(rx) device valid number bit mask
 * @param[in] ops: interrupt count type
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_rx_debug_setup(uint32_t mask, const mipi_sub_ops_t *ops)
{
	int32_t ret, i, size = 0, num = 0;
	struct mipi_csidbg_dev_s *rxs = NULL;

	if (mask > 0) {
		for (i = 0; i < MIPI_CSI_RXNUM; i++) {
			if ((mask & (0x1U << i)) != 0U) {
				num++;
				size = i +1;
			}
		}
		rxs = (struct mipi_csidbg_dev_s *)osal_kmalloc(sizeof(struct mipi_csidbg_dev_s) * size, GFP_KERNEL);
		if (IS_ERR(rxs)) {
			rxs = NULL;
			return -ENOMEM;
		}

		ret = hobot_mipi_host_debug_probe(mask, ops, &mipi_csi_rx_debug_ops);
		if (ret < 0) {
			osal_kfree(rxs);
			return ret;
		}
		(void)memset((void *)rxs, 0, sizeof(struct mipi_csidbg_dev_s) * size);
		g_cdbg.rx_num = num;
		g_cdbg.rx_mask = mask;
		g_cdbg.rxs = rxs;
	} else {
		ret = hobot_mipi_host_debug_remove(g_cdbg.rx_mask);

		if (g_cdbg.rxs != NULL) {
			osal_kfree(g_cdbg.rxs);
			g_cdbg.rxs = NULL;
		}
		g_cdbg.rx_num = 0;
		g_cdbg.rx_mask = 0U;
	}

	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug hook call: open
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi host(rx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_rx_debug_open(mipi_dbg_hook_e type, int32_t index)
{
	int32_t i;
	struct mipi_csidbg_dev_s *rx_dbg;

	if (index >= g_cdbg.rx_num)
		return -ENODEV;
	rx_dbg = &g_cdbg.rxs[index];

	if (rx_dbg->open_cnt == 0) {
		mipi_csi_debug_call_clear(&rx_dbg->open, 0);
		mipi_csi_debug_call_clear(&rx_dbg->close, 0);
		for(i = 0; i < MIPI_CSI_DBG_IOC_MAX; i++)
			mipi_csi_debug_call_clear(&rx_dbg->ioctl[i], 0);
		for(i = 0; i < MIPI_CSI_DBG_INT_MAX; i++)
			mipi_csi_debug_call_clear(&rx_dbg->intcb[i], 0);
	}

	mipi_csi_debug_call(type, &rx_dbg->open,
			g_cdbg_rxcalls[MIPI_CSI_DBG_OPEN_BASE]);
	if (type == MIPI_CSI_DBG_HOOKPOST)
		rx_dbg->open_cnt ++;
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug hook call: close
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi host(rx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_rx_debug_close(mipi_dbg_hook_e type, int32_t index)
{
	struct mipi_csidbg_dev_s *rx_dbg;

	if (index >= g_cdbg.rx_num)
		return -ENODEV;
	rx_dbg = &g_cdbg.rxs[index];

	mipi_csi_debug_call(type, &rx_dbg->close,
			g_cdbg_rxcalls[MIPI_CSI_DBG_CLOSE_BASE]);
	if ((rx_dbg->open_cnt > 0) && (type == MIPI_CSI_DBG_HOOKPOST))
		rx_dbg->open_cnt --;
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) debug hook call: ioctl
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi host(rx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_rx_debug_ioctl(mipi_dbg_hook_e type, int32_t index, uint32_t cmd, mipi_ioc_arg_t arg)
{
	int32_t iocnr = _IOC_NR(cmd);
	if ((index >= g_cdbg.rx_num) || (iocnr >= MIPI_CSI_DBG_IOC_MAX))
		return -ENODEV;

	mipi_csi_debug_call(type, &g_cdbg.rxs[index].ioctl[iocnr],
			g_cdbg_rxcalls[MIPI_CSI_DBG_IOC_BASE + iocnr]);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) interrupt debug function
 *
 * @param[in] index: mipi host(rx) port index
 * @param[in] icnt: interrupt count type
 * @param[in] subirq: interrupt sub error status
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_rx_debug_int(int32_t index, uint32_t icnt, uint32_t subirq)
{
	if ((index >= g_cdbg.rx_num) || (icnt >= MIPI_CSI_DBG_INT_MAX))
		return;

	mipi_csi_debug_call(MIPI_CSI_DBG_HOOKCALL, &g_cdbg.rxs[index].ioctl[0],
			g_cdbg_rxcalls[MIPI_CSI_DBG_INT_BASE + 0]);
	mipi_csi_debug_call(MIPI_CSI_DBG_HOOKCALL, &g_cdbg.rxs[index].ioctl[icnt],
			g_cdbg_rxcalls[MIPI_CSI_DBG_INT_BASE + icnt]);
}

/*
 * @var mipi_csi_rx_debug_ops
 * mipi csi host(rx) debug ops struct.
 */
static const struct mipi_dbg_ops_s mipi_csi_rx_debug_ops = {
	.setup = mipi_csi_rx_debug_setup,
	.open = mipi_csi_rx_debug_open,
	.close = mipi_csi_rx_debug_close,
	.ioctl = mipi_csi_rx_debug_ioctl,
	.int_cb = mipi_csi_rx_debug_int,
	.class = hobot_mipi_csi_class,
};


extern int32_t hobot_mipi_phy_debug_remove(void);
extern int32_t hobot_mipi_phy_debug_probe(const struct mipi_phy_ops_s *ops,
			const struct mipi_phy_dbg_ops_s *dops);
static const struct mipi_phy_dbg_ops_s mipi_csi_phy_debug_ops;
/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi phy debug function setup
 *
 * @param[in] ops: phy driver ops callback
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_phy_debug_setup(const mipi_phy_ops_t *ops)
{
	int32_t ret;
	struct mipi_csidbg_dev_s *phy = NULL;

	if (ops != NULL) {
		phy = (struct mipi_csidbg_dev_s *)osal_kmalloc(sizeof(struct mipi_csidbg_dev_s), GFP_KERNEL);
		if (IS_ERR(phy)) {
			phy = NULL;
			return -ENOMEM;
		}

		ret = hobot_mipi_phy_debug_probe(ops, &mipi_csi_phy_debug_ops);
		if (ret < 0) {
			osal_kfree(phy);
			return ret;
		}
		(void)memset((void *)phy, 0, sizeof(struct mipi_csidbg_dev_s));
		g_cdbg.phy = phy;
	} else {
		ret = hobot_mipi_phy_debug_remove();

		if (g_cdbg.phy != NULL) {
			osal_kfree(g_cdbg.phy);
			g_cdbg.phy = NULL;
		}
	}

	return ret;
}

/*
 * @var mipi_csi_phy_debug_ops
 * mipi csi phy debug ops struct.
 */
static const struct mipi_phy_dbg_ops_s mipi_csi_phy_debug_ops = {
	.setup = mipi_csi_phy_debug_setup,
	.class = hobot_mipi_csi_class,
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) debug function setup
 *
 * @param[in] mask: mipi dev(tx) device valid number bit mask
 * @param[in] ops: interrupt count type
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_tx_debug_setup(uint32_t mask, const mipi_sub_ops_t *ops)
{
	int32_t i, size = 0, num = 0;
	struct mipi_csidbg_dev_s *txs = NULL;

	if (mask > 0) {
		for (i = 0; i < MIPI_CSI_RXNUM; i++) {
			if ((mask & (0x1U << i)) != 0U) {
				num++;
				size = i +1;
			}
		}
		txs = (struct mipi_csidbg_dev_s *)osal_kmalloc(sizeof(struct mipi_csidbg_dev_s) * size, GFP_KERNEL);
		if (IS_ERR(txs)) {
			txs = NULL;
			return -ENOMEM;
		}

		(void)memset((void *)txs, 0, sizeof(struct mipi_csidbg_dev_s) * size);
		g_cdbg.tx_num = num;
		g_cdbg.tx_mask = mask;
		g_cdbg.txs = txs;
	} else {
		if (g_cdbg.txs != NULL) {
			osal_kfree(g_cdbg.txs);
			g_cdbg.txs = NULL;
		}
		g_cdbg.tx_num = 0;
		g_cdbg.tx_mask = 0U;
	}

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) debug hook call: open
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi dev(tx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_tx_debug_open(mipi_dbg_hook_e type, int32_t index)
{
	int32_t i;
	struct mipi_csidbg_dev_s *tx_dbg;

	if (index >= g_cdbg.tx_num)
		return -ENODEV;
	tx_dbg = &g_cdbg.txs[index];

	if (tx_dbg->open_cnt == 0) {
		mipi_csi_debug_call_clear(&tx_dbg->open, 0);
		mipi_csi_debug_call_clear(&tx_dbg->close, 0);
		for(i = 0; i < MIPI_CSI_DBG_IOC_MAX; i++)
			mipi_csi_debug_call_clear(&tx_dbg->ioctl[i], 0);
		for(i = 0; i < MIPI_CSI_DBG_INT_MAX; i++)
			mipi_csi_debug_call_clear(&tx_dbg->intcb[i], 0);
	}

	mipi_csi_debug_call(type, &tx_dbg->open,
			g_cdbg_txcalls[MIPI_CSI_DBG_OPEN_BASE]);
	if (type == MIPI_CSI_DBG_HOOKPOST)
		tx_dbg->open_cnt ++;
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) debug hook call: close
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi dev(tx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_tx_debug_close(mipi_dbg_hook_e type, int32_t index)
{
	struct mipi_csidbg_dev_s *tx_dbg;

	if (index >= g_cdbg.tx_num)
		return -ENODEV;
	tx_dbg = &g_cdbg.txs[index];

	mipi_csi_debug_call(type, &tx_dbg->close,
			g_cdbg_txcalls[MIPI_CSI_DBG_CLOSE_BASE]);
	if ((tx_dbg->open_cnt > 0) && (type == MIPI_CSI_DBG_HOOKPOST))
		tx_dbg->open_cnt --;
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) debug hook call: ioctl
 *
 * @param[in] type: debug hook call type: pre/post/err
 * @param[in] index: mipi dev(tx) port index
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_csi_tx_debug_ioctl(mipi_dbg_hook_e type, int32_t index, uint32_t cmd, mipi_ioc_arg_t arg)
{
	int32_t iocnr = _IOC_NR(cmd);
	if ((index >= g_cdbg.tx_num) || (iocnr >= MIPI_CSI_DBG_IOC_MAX))
		return -ENODEV;

	mipi_csi_debug_call(type, &g_cdbg.txs[index].ioctl[iocnr],
			g_cdbg_txcalls[MIPI_CSI_DBG_IOC_BASE + iocnr]);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) interrupt debug function
 *
 * @param[in] index: mipi dev(tx) port index
 * @param[in] icnt: interrupt count type
 * @param[in] subirq: interrupt sub error status
 *
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_tx_debug_int(int32_t index, uint32_t icnt, uint32_t subirq)
{
	if ((index >= g_cdbg.tx_num) || (icnt >= MIPI_CSI_DBG_INT_MAX))
		return;

	mipi_csi_debug_call(MIPI_CSI_DBG_HOOKCALL, &g_cdbg.txs[index].intcb[0],
			g_cdbg_txcalls[MIPI_CSI_DBG_INT_BASE + 0]);
	mipi_csi_debug_call(MIPI_CSI_DBG_HOOKCALL, &g_cdbg.txs[index].intcb[icnt],
			g_cdbg_txcalls[MIPI_CSI_DBG_INT_BASE + icnt]);
}

/*
 * @var mipi_csi_tx_debug_ops
 * mipi csi dev(tx) debug ops struct.
 */
static const struct mipi_dbg_ops_s mipi_csi_tx_debug_ops = {
	.setup = mipi_csi_tx_debug_setup,
	.open = mipi_csi_tx_debug_open,
	.close = mipi_csi_tx_debug_close,
	.ioctl = mipi_csi_tx_debug_ioctl,
	.int_cb = mipi_csi_tx_debug_int,
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi debug driver init
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t __init hobot_mipi_csi_debug_init(void)
{
	int32_t ret;

	/* prepare class */
	g_cdbg.csi_class = (struct class *)hobot_mipi_csi_debug_class();

	ret = hobot_mipi_phy_debug_setup(&mipi_csi_phy_debug_ops);
	if (ret < 0)
		return ret;

	ret = hobot_mipi_csi_debug_setup(&mipi_csi_rx_debug_ops, &mipi_csi_tx_debug_ops);
	if (ret < 0) {
		hobot_mipi_phy_debug_setup(NULL);
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi debug driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_mipi_csi_debug_exit(void)
{
	hobot_mipi_csi_debug_setup(NULL, NULL);
	hobot_mipi_phy_debug_setup(NULL);
}

late_initcall_sync(hobot_mipi_csi_debug_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_mipi_csi_debug_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT MIPI DEBUG Driver"); /* PRQA S ALL */ /* linux macro */
