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
 * @file hobot_mipi_csi.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#include <linux/init.h>
#include <linux/module.h>

#include "hobot_mipi_csi.h"
#include "hobot_mipi_host_ops.h"
#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_phy.h"

#include "hobot_vin_common.h"
#include "hobot_dev_vin_node.h"

/**
 * @var g_csi
 * covert the global struct of csi driver
 */
static struct mipi_csi_s g_csi = {};

/**
 * @def mipi_rx_op
 * covert mipi csi rx device operation
 */
#define mipi_rx_op(ops, ...) \
	({ \
		int32_t ret; \
		if ((g_csi.rx_ops == NULL) || (g_csi.rx_ops->ops == NULL)) \
			ret = -EFAULT; \
		if ((g_csi.rx_dbg_ops != NULL) && (g_csi.rx_dbg_ops->ops != NULL)) \
			(void)g_csi.rx_dbg_ops->ops(MIPI_CSI_DBG_HOOKPRE, __VA_ARGS__); \
		ret = g_csi.rx_ops->ops(__VA_ARGS__); \
		if ((g_csi.rx_dbg_ops != NULL) && (g_csi.rx_dbg_ops->ops != NULL)) \
			(void)g_csi.rx_dbg_ops->ops(((ret < 0) ? MIPI_CSI_DBG_HOOKERR : MIPI_CSI_DBG_HOOKPOST), \
						__VA_ARGS__); \
		ret; \
       	})

/**
 * @def mipi_tx_op
 * covert mipi csi tx device operation
 */
#define mipi_tx_op(ops, ...) \
	({ \
		int32_t ret,  dbg = 0; \
		if ((g_csi.tx_ops == NULL) || (g_csi.tx_ops->ops == NULL)) \
			ret = -EFAULT; \
		if ((g_csi.tx_dbg_ops != NULL) && (g_csi.tx_dbg_ops->ops != NULL)) \
			dbg = 1; \
		if (dbg != 0) \
			(void)g_csi.tx_dbg_ops->ops(MIPI_CSI_DBG_HOOKPRE, __VA_ARGS__); \
		ret = g_csi.tx_ops->ops(__VA_ARGS__); \
		if (dbg != 0) \
			(void)g_csi.tx_dbg_ops->ops(((ret < 0) ? MIPI_CSI_DBG_HOOKERR : MIPI_CSI_DBG_HOOKPOST), \
						__VA_ARGS__); \
		ret; \
       	})

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief get mipi csi device struct by index
 *
 * @param[in] index: the mipi csi index
 *
 * @return !0:mipi csi device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct mipi_csi_device_s* mipi_get_by_id(int32_t index)
{
	if ((g_csi.rx_mask & (0x1U << index)) == 0U)
		return NULL;
	return &g_csi.csi[index];
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi csi device index covert from vctx
 *
 * @param[in] vctx: vin contex for vcon
 *
 * @return !0:mipi csi device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct mipi_csi_device_s* mipi_get_by_vctx(struct vio_video_ctx *vctx)
{
	uint32_t index;
	struct mipi_csi_device_s *mipi;

	if ((vctx == NULL) || (vctx->device == NULL))
		return NULL;

	index = ((struct j6_vin_node_dev *)(vctx->device))->hw_id;
	mipi = mipi_get_by_id(index);

	return mipi;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi extra attr value change
 *
 * @param[in] mask: attr valid mask to update
 * @param[in] size: attr size at all
 * @param[in] src: source point of attr_ex struct
 * @param[in] dst: dest point of attr_ex struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_change_attr_ex(uint64_t mask, uint32_t size, void *src, void *dst)
{
	uint32_t i;
	uint32_t *psrc = (uint32_t *)src;
	uint32_t *pdst = (uint32_t *)dst;

	for (i = 0U; i < size; i++) {
		if ((mask & (0x1UL << i)) != 0UL)
			pdst[i] = psrc[i];
	}
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device extra attr set to driver really (should call in mutex)
 *
 * @param[in] csi: mipi csi device struct
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
static int32_t mipi_csi_update_attr_ex(struct mipi_csi_device_s *csi)
{
	int32_t ret = 0;
	int32_t host = csi->index, dev = csi->config.tx_index;
	struct mipi_attr_s *cfg = &csi->config;
	struct mipi_attr_ex_s *ex = &csi->param;


	if (csi->open_cnt == 0) {
		return ret;
	}
	if (ex->rx_ex_mask != 0UL) {
		struct mipi_host_param_s rx_attr_ex;

		if (cfg->rx_enable == 0) {
			return -EPERM;
		}
		ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_GET_PARAM,
			(mipi_ioc_arg_t)(&rx_attr_ex));
		if (ret < 0) {
			return ret;
		}
		mipi_csi_change_attr_ex(ex->rx_ex_mask, MIPI_HOST_PARAMS_NUM,
			&ex->rx_attr_ex, &rx_attr_ex);
		ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_SET_PARAM,
			(mipi_ioc_arg_t)(&rx_attr_ex));
		if (ret < 0) {
			return ret;
		}
		memcpy(&ex->rx_attr_ex, &rx_attr_ex, sizeof(struct mipi_host_param_s));
		ex->rx_ex_mask = 0UL;
	}
	if (ex->tx_ex_mask != 0UL) {
		struct mipi_dev_param_s tx_attr_ex;

		if (cfg->tx_enable == 0)  {
			return -EPERM;
		}
		ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_GET_PARAM,
			(mipi_ioc_arg_t)(&tx_attr_ex));
		if (ret < 0) {
			return ret;
		}
		mipi_csi_change_attr_ex(ex->tx_ex_mask, MIPI_DEV_PARAMS_NUM,
			&ex->tx_attr_ex, &tx_attr_ex);
		ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_SET_PARAM,
			(mipi_ioc_arg_t)(&tx_attr_ex));
		if (ret < 0) {
			return ret;
		}
		memcpy(&ex->tx_attr_ex, &tx_attr_ex, sizeof(struct mipi_dev_param_s));
		ex->tx_ex_mask = 0UL;
	}
	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device attach operation as init
 *
 * @param[in] csi: mipi csi device struct
 * @param[in] cfg: mipi csi attach attr struct
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
static int32_t mipi_csi_attach(struct mipi_csi_device_s *csi, struct mipi_attr_s *cfg)
{
	int32_t ret, tx_opened = 0, tx_inited = 0, rx_inited = 0;
	struct mipi_attr_s *config = &csi->config;
	struct mipi_attr_s *c_rx = cfg, *c_tx = cfg;
	int32_t rx_en_bak = config->rx_enable;
	int32_t tx_en_bak = config->tx_enable;
	int32_t tx_id_bak = config->tx_index;
	int32_t host = csi->index, dev;

	/* use attr prepare set ? */
	if ((cfg->rx_enable == 0) && (config->rx_enable != 0))
		c_rx = config;
	if ((cfg->tx_enable == 0) && (config->tx_enable != 0))
		c_tx = config;

	dev = c_tx->tx_index;
	if ((c_tx->tx_enable != 0) && ((config->attach & MIPI_CSI_TX_ATTACHED) == 0)) {
		/* tx open first */
		if (g_csi.tx[dev] != NULL)
			return -EBUSY;
		ret = mipi_tx_op(open, dev);
		if (ret < 0)
			goto attach_err;
		tx_opened = 1;
	}

	/* param updata to hw */
	config->rx_enable = c_rx->rx_enable;
	config->tx_enable = c_tx->tx_enable;
	config->tx_index = c_tx->tx_index;
	ret = mipi_csi_update_attr_ex(csi);
	config->rx_enable = rx_en_bak;
	config->tx_enable = tx_en_bak;
	config->tx_index = tx_id_bak;
	if (ret < 0)
		goto attach_err;

	/* init hw */
	if ((c_tx->tx_enable != 0) && (c_tx->tx_attr.vpg != 0U)) {
		/* tx vpg init */
		if ((config->attach & MIPI_CSI_TX_ATTACHED) == 0) {
			ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_INIT, (mipi_ioc_arg_t)&c_tx->tx_attr);
			if (ret < 0)
				goto attach_err;
			tx_inited = 1;
		}
	}

	if (c_rx->rx_enable != 0) {
		/* rx init */
		if ((config->attach & MIPI_CSI_RX_ATTACHED) == 0) {
			ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_INIT, (mipi_ioc_arg_t)&c_rx->rx_attr);
			if (ret < 0)
				goto attach_err;
			rx_inited = 1;
		}
	}

	if ((c_tx->tx_enable != 0) && (c_tx->tx_attr.vpg == 0U)) {
		/* tx bypass init */
		if ((config->attach & MIPI_CSI_TX_ATTACHED) == 0) {
			ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_INIT, (mipi_ioc_arg_t)&c_tx->tx_attr);
			if (ret < 0)
				goto attach_err;
			tx_inited = 1;
		}
	}

	/* init done */
	if (rx_inited) {
		config->attach |= MIPI_CSI_RX_ATTACHED;
		if (c_rx != config) {
			config->rx_enable = c_rx->rx_enable;
			memcpy(&config->rx_attr, &c_rx->rx_attr, sizeof(struct mipi_host_cfg_s));
		}
	}
	if (tx_inited) {
		config->attach |= MIPI_CSI_TX_ATTACHED;
		if (c_tx != config) {
			config->tx_enable = c_tx->tx_enable;
			config->tx_index = dev;
			memcpy(&config->tx_attr, &c_tx->tx_attr, sizeof(struct mipi_dev_cfg_s));
		}
		g_csi.tx[dev] = csi;
	}
	return ret;

attach_err:
	if (rx_inited)
		mipi_rx_op(ioctl, host, MIPIHOSTIOC_DEINIT, 0UL);
	if (tx_inited)
		mipi_tx_op(ioctl, dev, MIPIDEVIOC_DEINIT, 0UL);
	if (tx_opened)
		mipi_tx_op(close, dev);

	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device detach operation as deinit
 *
 * @param[in] csi: mipi csi device struct
 * @param[in] cfg: mipi csi detach attr struct
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
static int32_t mipi_csi_detach(struct mipi_csi_device_s *csi, struct mipi_attr_s *cfg)
{
	struct mipi_attr_s *config = &csi->config;
	int32_t host = csi->index, dev = config->tx_index;

	/* deinit hw */
	if ((cfg->tx_enable != 0) && ((config->attach & MIPI_CSI_TX_ATTACHED) != 0)) {
		/* tx deinit */
		mipi_tx_op(ioctl, dev, MIPIDEVIOC_DEINIT, 0UL);
		mipi_tx_op(close, dev);
		config->attach &= ~MIPI_CSI_TX_ATTACHED;
		config->tx_enable = 0;
		g_csi.tx[dev] = NULL;
	}

	if ((cfg->rx_enable != 0) && ((config->attach & MIPI_CSI_RX_ATTACHED) != 0)) {
		/* rx deinit */
		mipi_rx_op(ioctl, host, MIPIHOSTIOC_DEINIT, 0UL);
		config->attach &= ~MIPI_CSI_RX_ATTACHED;
		config->rx_enable = 0;
	}
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device open, prepare to work
 *
 * @param[in] vctx: vin contex for mipi csi
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
static int32_t mipi_open(struct vio_video_ctx *vctx)
{
	int32_t ret, host;
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);

	if (csi == NULL)
		return -ENODEV;
	host = csi->index;

	osal_mutex_lock(&csi->mutex);
	ret = mipi_rx_op(open, host);
	if (ret == 0) {
		if (csi->open_cnt == 0) {
			memset(&csi->config, 0, sizeof(csi->config));
		}
		csi->open_cnt ++;
	}
	osal_mutex_unlock(&csi->mutex);
	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device close, done to idle
 *
 * @param[in] vctx: vin contex for mipi csi
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
static int32_t mipi_close(struct vio_video_ctx *vctx)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg;
	int32_t host;

	if (csi == NULL)
		return -ENODEV;
	cfg = &csi->config;
	host = csi->index;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt > 0) {
		mipi_rx_op(close, host);
		csi->open_cnt--;
		if (csi->open_cnt == 0) {
			mipi_csi_detach(csi, cfg);
			memset(&csi->param, 0, sizeof(csi->param));
		}
	}
	osal_mutex_unlock(&csi->mutex);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device extra attr get for params
 *
 * @param[in] vctx: vin contex for mipi csi
 * @param[in] arg: extra attr struct: mipi_attr_ex_t
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
static int32_t mipi_get_attr_ex(struct vio_video_ctx *vctx, void *arg)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_ex_s *ex = (struct mipi_attr_ex_s *)arg;

	if (csi == NULL)
		return -ENODEV;
	if (ex == NULL)
		return -EINVAL;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	memcpy(ex, &csi->param, sizeof(struct mipi_attr_ex_s));
	osal_mutex_unlock(&csi->mutex);

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device extra attr set for params
 *
 * @param[in] vctx: vin contex for mipi csi
 * @param[in] arg: extra attr struct: mipi_attr_ex_t
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
static int32_t mipi_set_attr_ex(struct vio_video_ctx *vctx, void *arg)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct vin_attr_ex_s *vin_ex = (struct vin_attr_ex_s*)arg;
	struct mipi_attr_ex_s *ex;
        struct mclk_attr_ex_s *mclk_ex = { 0 };
	mipi_host_ipi_reset_t ipi_reset = { 0 };
	struct mipi_attr_s *cfg;
	int32_t ret = 0, host, dev;
	uint32_t bypass_enable;
        uint32_t mclk_freq = 0;
        uint32_t mclk_enable = 0;

	if (csi == NULL)
		return -ENODEV;
	if (vin_ex == NULL)
		return -EINVAL;

	switch (vin_ex->ex_attr_type) {
	case VIN_STATIC_MIPI_ATTR:
		ex = &vin_ex->mipi_ex_attr;
		if (ex->attr_valid == 0)
			return 0;

		osal_mutex_lock(&csi->mutex);
		if (ex->rx_ex_mask != 0UL) {
			mipi_csi_change_attr_ex(ex->rx_ex_mask, MIPI_HOST_PARAMS_NUM,
					&ex->rx_attr_ex, &csi->param.rx_attr_ex);
			csi->param.rx_ex_mask |= ex->rx_ex_mask;
		}
		if (ex->tx_ex_mask != 0UL) {
			mipi_csi_change_attr_ex(ex->tx_ex_mask, MIPI_DEV_PARAMS_NUM,
					&ex->tx_attr_ex, &csi->param.tx_attr_ex);
			csi->param.tx_ex_mask |= ex->tx_ex_mask;
		}
		ret = mipi_csi_update_attr_ex(csi);
		osal_mutex_unlock(&csi->mutex);
		break;
	case VIN_DYNAMIC_IPI_RESET:
		cfg = &csi->config;
		host = csi->index;
		ipi_reset.enable = MIPI_CSI_IPI_RESET_ENABLE(vin_ex->ipi_reset);
		ipi_reset.mask = MIPI_CSI_IPI_RESET_MASK(vin_ex->ipi_reset);

		osal_mutex_lock(&csi->mutex);
		if (csi->open_cnt == 0) {
			osal_mutex_unlock(&csi->mutex);
			return -EPERM;
		}
		if ((cfg->rx_enable != 0) && ((cfg->attach & MIPI_CSI_RX_ATTACHED) != 0))
			ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_IPI_RESET, (mipi_ioc_arg_t)&ipi_reset);
		else
			ret = -EACCES;
		osal_mutex_unlock(&csi->mutex);
		break;
	case VIN_DYNAMIC_BYPASS_ENABLE:
		cfg = &csi->config;
		dev = cfg->tx_index;
		bypass_enable = vin_ex->bypass_enable;

		osal_mutex_lock(&csi->mutex);
		if (csi->open_cnt == 0) {
			osal_mutex_unlock(&csi->mutex);
			return -EPERM;
		}
		if ((cfg->tx_enable != 0) && ((cfg->attach & MIPI_CSI_TX_ATTACHED) != 0))
			ret = mipi_rx_op(ioctl, dev, MIPIDEVIOC_SET_BYPASS, (mipi_ioc_arg_t)&bypass_enable);
		else
			ret = -EACCES;
		osal_mutex_unlock(&csi->mutex);

		break;
        case VIN_STATIC_MCLK_ATTR:
                if (vctx->id == VNODE_ID_SRC) {  //only for input
                        host = csi->index;
                        mclk_ex = &vin_ex->mclk_ex_attr;
                        if (mclk_ex == NULL) {
                                return -EINVAL;
                        }
                        mclk_freq = mclk_ex->mclk_freq;

                        osal_mutex_lock(&csi->mutex);
                        // set freq also will enable or disable clk
                        ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_SNRCLK_SET_FREQ, (mipi_ioc_arg_t)&mclk_freq);
                        if (ret != 0) {
                                osal_mutex_unlock(&csi->mutex);
                                ret = -EACCES;
                                break;
                        }

                        // set freq should select pinctrl by MIPIHOSTIOC_SNRCLK_SET_EN
                        // MIPIHOSTIOC_SNRCLK_SET_EN only select pinctrl, and will not enable or disable clk again
                        if (mclk_freq != 0) {
                                mclk_enable = 1;
                        } else {
                                mclk_enable = 0;
                        }
                        ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_SNRCLK_SET_EN,(mipi_ioc_arg_t)&mclk_enable);
                        if (ret != 0) {
                                osal_mutex_unlock(&csi->mutex);
                                ret = -EACCES;
                                break;
                        }

                        osal_mutex_unlock(&csi->mutex);
                }
                break;
	default:
		ret = -ERANGE;
	}

	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device internal attr set for init
 *
 * @param[in] vctx: vin contex for mipi csi
 * @param[in] arg: init attr struct: mipi_attr_t
 *
 * @return 0:Susscess <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_set_internal_attr(struct vio_video_ctx *vctx, void *arg)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg = (struct mipi_attr_s *)arg;
	int32_t ret;

	if (csi == NULL)
		return -ENODEV;
	if (cfg == NULL)
		return -EINVAL;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	if (cfg->attach != 0) {
		/* attach: init */
		ret = mipi_csi_attach(csi, cfg);
	} else {
		/* detach: deinit */
		ret = mipi_csi_detach(csi, cfg);
	}
	osal_mutex_unlock(&csi->mutex);
	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device attr set
 *
 * @param[in] vctx: vin contex for mipi csi
 * @param[out] arg: the current mipi attr struct: mipi_attr_t
 *
 * @return 0:Susscess <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_set_attr(struct vio_video_ctx *vctx, void* arg)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg = (struct mipi_attr_s *)arg;
	struct mipi_attr_s *config;

	if (csi == NULL)
		return -ENODEV;
	if (cfg == NULL)
		return -EINVAL;
	config = &csi->config;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	if (cfg->rx_enable != 0) {
		if ((config->attach & MIPI_CSI_RX_ATTACHED) != 0) {
			osal_mutex_unlock(&csi->mutex);
			return -EBUSY;
		}
		config->rx_enable = cfg->rx_enable;
		memcpy(&config->rx_attr, &cfg->rx_attr, sizeof(struct mipi_host_cfg_s));
	}
	if (cfg->tx_enable != 0) {
		if ((config->attach & MIPI_CSI_TX_ATTACHED) != 0) {
			osal_mutex_unlock(&csi->mutex);
			return -EBUSY;
		}
		config->tx_enable = cfg->tx_enable;
		config->tx_index = cfg->tx_index;
		memcpy(&config->tx_attr, &cfg->tx_attr, sizeof(struct mipi_dev_cfg_s));
	}
	osal_mutex_unlock(&csi->mutex);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device attr get
 *
 * @param[in] vctx: vin contex for mipi csi
 * @param[out] arg: the current mipi attr struct: mipi_attr_t
 *
 * @return 0:Susscess <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t mipi_get_attr(struct vio_video_ctx *vctx, void *arg)
{
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg = (struct mipi_attr_s *)arg;

	if (csi == NULL)
		return -ENODEV;
	if (cfg == NULL)
		return -EINVAL;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	memcpy(cfg, &csi->config, sizeof(struct mipi_attr_s));
	osal_mutex_unlock(&csi->mutex);
	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device start, check status and working
 *
 * @param[in] vctx: vin contex for mipi csi
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
static int32_t mipi_start(struct vio_video_ctx *vctx)
{
	int32_t ret, started = 0, host, dev;
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg;

	if (csi == NULL)
		return -ENODEV;
	cfg = &csi->config;
	host = csi->index;
	dev = cfg->tx_index;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	if (cfg->tx_enable != 0) {
		if ((cfg->attach & MIPI_CSI_TX_ATTACHED) != 0)
			ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_START, 0UL);
		else
			ret = -EACCES;
		if (ret < 0) {
			osal_mutex_unlock(&csi->mutex);
			return ret;
		}
		started = 1;
	}
	if (cfg->rx_enable != 0) {
		if ((cfg->attach & MIPI_CSI_RX_ATTACHED) != 0)
			ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_START, 0UL);
		else
			ret = -EACCES;
		if (ret < 0) {
			if (started != 0)
				mipi_tx_op(ioctl, dev, MIPIDEVIOC_STOP, 0UL);
			osal_mutex_unlock(&csi->mutex);
			return ret;
		}
		started = 1;
	}
	osal_mutex_unlock(&csi->mutex);
	return (started == 0) ? (-ENODEV) : 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi device stop, stop working
 *
 * @param[in] vctx: vin contex for mipi csi
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
static int32_t mipi_stop(struct vio_video_ctx *vctx)
{
	int32_t ret, stoped = 0, host, dev;
	struct mipi_csi_device_s *csi = mipi_get_by_vctx(vctx);
	struct mipi_attr_s *cfg;

	if (csi == NULL)
		return -ENODEV;
	cfg = &csi->config;
	host = csi->index;
	dev = cfg->tx_index;

	osal_mutex_lock(&csi->mutex);
	if (csi->open_cnt == 0) {
		osal_mutex_unlock(&csi->mutex);
		return -EPERM;
	}
	if (cfg->tx_enable != 0) {
		if ((cfg->attach & MIPI_CSI_TX_ATTACHED) != 0)
			ret = mipi_tx_op(ioctl, dev, MIPIDEVIOC_STOP, 0UL);
		else
			ret = -EACCES;
		if (ret == 0)
			stoped = 1;
	}
	if (cfg->rx_enable != 0) {
		if ((cfg->attach & MIPI_CSI_RX_ATTACHED) != 0)
			ret = mipi_rx_op(ioctl, host, MIPIHOSTIOC_STOP, 0UL);
		else
			ret = -EACCES;
		if (ret == 0)
			stoped = 1;
	}
	osal_mutex_unlock(&csi->mutex);
	return (stoped == 0) ? (-ENODEV) : 0;
}

/**
 * mipi csi driver ops for vin_node
 */
static struct vin_common_ops mipi_ops = {
	.open = mipi_open,
	.close = mipi_close,
	.video_start = mipi_start,
	.video_stop = mipi_stop,
	.video_set_attr = mipi_set_attr,
	.video_get_attr = mipi_get_attr,
        .video_set_internal_attr = mipi_set_internal_attr,
	.video_get_attr_ex = mipi_get_attr_ex,
	.video_set_attr_ex = mipi_set_attr_ex,
	.video_error_callback = NULL,
};

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) frame drop callbcak function
 *
 * @param[in] index: host(rx) port index
 * @param[in] ipi_mask: hosr(rx) ipi mask to drop
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_rx_frame_drop(int32_t index, uint32_t ipi_mask)
{

}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi host(rx) interrupt callback function
 *
 * @param[in] index: host(rx) port index
 * @param[in] icnt: interrupt count index
 * @param[in] subirq: interrupt sub error mask
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_rx_interrupt(int32_t index, uint32_t icnt, uint32_t subirq)
{
	if ((g_csi.rx_dbg_ops != NULL) && (g_csi.rx_dbg_ops->int_cb != NULL))
		g_csi.rx_dbg_ops->int_cb(index, icnt, subirq);
}

#ifdef CONFIG_HOBOT_MIPI_DEV
/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi dev(tx) interrupt callback function
 *
 * @param[in] index: dev(tx) port index
 * @param[in] icnt: interrupt count index
 * @param[in] subirq: interrupt sub error mask
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void mipi_csi_tx_interrupt(int32_t index, uint32_t icnt, uint32_t subirq)
{
	if ((g_csi.tx_dbg_ops != NULL) && (g_csi.tx_dbg_ops->int_cb != NULL))
		g_csi.tx_dbg_ops->int_cb(index, icnt, subirq);
}
#endif

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi class get for debug device create
 *
 * @return NULL:invalid !NULL:valid
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void* hobot_mipi_csi_debug_class(void)
{
	return ((void *)vin_node_get_class());
}
EXPORT_SYMBOL_GPL(hobot_mipi_csi_debug_class); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi debug setup to register callbcak function of error interrupt
 *
 * @param[in] rx_dbg_ops: rx debug callback struct for setup
 * @param[in] tx_dbg_ops: tx debug callback struct for setup
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_csi_debug_setup(const mipi_dbg_ops_t *rx_dbg_ops, const mipi_dbg_ops_t *tx_dbg_ops)
{
	int32_t ret;

	if ((rx_dbg_ops != NULL) && (g_csi.rx_dbg_ops != NULL))
		return -EBUSY;
	if ((tx_dbg_ops != NULL) && (g_csi.tx_dbg_ops != NULL))
		return -EBUSY;
	if ((rx_dbg_ops != NULL) && (rx_dbg_ops->setup != NULL)) {
		ret = rx_dbg_ops->setup(g_csi.rx_mask, g_csi.rx_ops);
		if (ret < 0)
			return ret;
	} else if ((g_csi.rx_dbg_ops != NULL) && (g_csi.rx_dbg_ops->setup != NULL)) {
		g_csi.rx_dbg_ops->setup(0, NULL);
	}
	if ((tx_dbg_ops != NULL) && (tx_dbg_ops->setup != NULL)) {
		ret = tx_dbg_ops->setup(g_csi.tx_mask, g_csi.tx_ops);
		if (ret < 0) {
			if ((rx_dbg_ops != NULL) && (rx_dbg_ops->setup != NULL))
				rx_dbg_ops->setup(0, NULL);
			return ret;
		}
	} else if ((g_csi.tx_dbg_ops != NULL) && (g_csi.tx_dbg_ops->setup != NULL)) {
		g_csi.tx_dbg_ops->setup(0, NULL);
	}
	g_csi.rx_dbg_ops = rx_dbg_ops;
	g_csi.tx_dbg_ops = tx_dbg_ops;
	return 0;
}
EXPORT_SYMBOL_GPL(hobot_mipi_csi_debug_setup); /* PRQA S 0307 */ /* EXPORT_SYMBOL_GPL macro */

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi driver init
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
static int32_t __init hobot_mipi_csi_module_init(void)
{
	int32_t ret, i;

#ifdef CONFIG_HOBOT_MIPI_HOST
	ret = hobot_mipi_host_setup(&g_csi.rx_ops);
	if (ret < 0) {
		return ret;
	}
	g_csi.rx_mask = (uint32_t)ret;
	g_csi.rx_num = 0;
	for (i = 0; i < MIPI_CSI_RXNUM; i++) {
		if ((g_csi.rx_mask & (0x1U << i)) != 0U) {
			g_csi.rx_num++;
			g_csi.csi[i].index = i;
			osal_mutex_init(&g_csi.csi[i].mutex);
		}
	}
#endif

#ifdef CONFIG_HOBOT_MIPI_DEV
	ret = hobot_mipi_dev_setup(&g_csi.tx_ops);
	if (ret < 0) {
#ifdef CONFIG_HOBOT_MIPI_HOST
		hobot_mipi_host_setup(NULL);
#endif
		g_csi.rx_num = 0;
		g_csi.rx_mask = 0U;
		g_csi.rx_ops = NULL;
		return ret;
	}
	g_csi.tx_mask = (uint32_t)ret;
	g_csi.tx_num = 0;
	for (i = 0; i < MIPI_CSI_TXNUM; i++) {
		if ((g_csi.tx_mask & (0x1U << i)) != 0U)
			g_csi.tx_num++;
	}
#endif

	vin_register_device_node(VIN_MIPI, &mipi_ops);

#ifdef CONFIG_HOBOT_MIPI_HOST
	if ((g_csi.rx_ops != NULL) && (g_csi.rx_ops->setcb != NULL)) {
		for (i = 0; i < MIPI_CSI_RXNUM; i++) {
			if ((g_csi.rx_mask & (0x1U << i)) != 0U)
				g_csi.rx_ops->setcb(i, mipi_csi_rx_frame_drop, mipi_csi_rx_interrupt);
		}
	}
#endif
#ifdef CONFIG_HOBOT_MIPI_DEV
	if ((g_csi.tx_ops != NULL) && (g_csi.tx_ops->setcb != NULL)) {
		for (i = 0; i < MIPI_CSI_TXNUM; i++) {
			if ((g_csi.tx_mask & (0x1U << i)) != 0U)
				g_csi.tx_ops->setcb(i, NULL, mipi_csi_tx_interrupt);
		}
	}
#endif
	mipi_info(NULL, "init %d rx %d tx done\n", g_csi.rx_num, g_csi.tx_num);

	return 0;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi csi driver exit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void __exit hobot_mipi_csi_module_exit(void)
{
	int32_t i, rx_num = 0, tx_num = 0;

	vin_unregister_device_node(VIN_MIPI);

#ifdef CONFIG_HOBOT_MIPI_HOST
	if ((g_csi.rx_ops != NULL) && (g_csi.rx_ops->setcb != NULL)) {
		for (i = 0; i < MIPI_CSI_RXNUM; i++) {
			if ((g_csi.rx_mask & (0x1U << i)) != 0U)
				g_csi.rx_ops->setcb(i, NULL, NULL);
		}
	}
	(void)hobot_mipi_host_setup(NULL);
	for (i = 0; i < MIPI_CSI_RXNUM; i++) {
		g_csi.csi[i].index = 0;
	}
	rx_num = g_csi.rx_num;
	g_csi.rx_num = 0;
	g_csi.rx_mask = 0U;
	g_csi.rx_ops = NULL;
#endif

#ifdef CONFIG_HOBOT_MIPI_DEV
	if ((g_csi.tx_ops != NULL) && (g_csi.tx_ops->setcb != NULL)) {
		for (i = 0; i < MIPI_CSI_TXNUM; i++) {
			if ((g_csi.tx_mask & (0x1U << i)) != 0U)
				g_csi.tx_ops->setcb(i, NULL, NULL);
		}
	}
	(void)hobot_mipi_dev_setup(NULL);
	tx_num = g_csi.tx_num;
	g_csi.tx_num = 0;
	g_csi.tx_mask = 0U;
	g_csi.tx_ops = NULL;
#endif

	mipi_info(NULL, "exit %d rx %d tx done\n", rx_num, tx_num);
}

late_initcall_sync(hobot_mipi_csi_module_init); /* PRQA S 0605 */ /* late_initcall_sync macro */
module_exit(hobot_mipi_csi_module_exit); /* PRQA S 0605 */ /* module_exit macro */
MODULE_LICENSE("GPL"); /* PRQA S ALL */ /* linux macro */
MODULE_AUTHOR("Lan Mingang <mingang.lan@horizon.ai>"); /* PRQA S ALL */ /* linux macro */
MODULE_DESCRIPTION("HOBOT MIPI CSI Driver"); /* PRQA S ALL */ /* linux macro */
