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
 * @file hobot_mipi_dev.c
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#define osdev_pre	"vin mipi-tx"

#ifdef MIPI_DEV_IDE_ENABLE
/* mipi-csi-tx in ide */
#include "hb_mipi_csi_device_ops.h"
#endif

#include "hobot_mipi_csi.h"
#include "hobot_mipi_dev.h"
#include "hobot_mipi_osal.h"

/* mipi dev user struct */
struct mipi_dev_user_s {
	osal_mutex_t      open_mutex;
	osal_mutex_t      mutex;
	uint32_t          open_cnt;
	uint32_t          init_cnt;
	uint32_t          start_cnt;
};

/**
 * @struct mipi_ddev_s
 * mipi csi device operation struct
 * @NO{S10E03C01}
 */
struct mipi_ddev_s {
	int32_t port;
	struct os_dev               osdev;
	struct mipi_dev_user_s      user;
#ifdef MIPI_DEV_IDE_ENABLE
	struct device	            tx_dev;
	struct mipi_csi_dev_priv_s *priv;
#endif
};

/**
 * @var g_ddev
 * covert the global struct of csi tx device
 */
static struct mipi_ddev_s g_ddev[MIPI_DEV_MAX_NUM];

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device setup init all
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
static int32_t hobot_mipi_dev_setup_init(void)
{
	struct mipi_ddev_s *ddev;
	int32_t i;

	memset (&g_ddev, 0, sizeof(g_ddev));

	for (i = 0; i < ARRAY_SIZE(g_ddev); i++) {
		ddev = &g_ddev[i];
		ddev->port = i;
		ddev->osdev.devno = i;
		osal_mutex_init(&ddev->user.open_mutex);
	}

	return 0;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device setup exit all
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void hobot_mipi_dev_setup_exit(void)
{
#ifdef MIPI_DEV_IDE_ENABLE
	struct mipi_ddev_s *ddev;
	int32_t i;

	for (i = 0; i < ARRAY_SIZE(g_ddev); i++) {
		ddev = &g_ddev[i];
		if (ddev->priv != NULL)  {
			(void)mipi_csi_dev_unregister(ddev->priv);
			ddev->priv = NULL;
		}
	}
#endif
	return;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device sturct get
 *
 * @param[in] port: mipi dev(tx) device port index
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
static struct mipi_ddev_s *hobot_mipi_dev_ddev(int32_t port)
{
	if (port >= ARRAY_SIZE(g_ddev))
		return NULL;
	return &g_ddev[port];
}


#ifdef MIPI_DEV_IDE_ENABLE
/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device operation init do
 *
 * @param[in] ddev: mipi dev(tx) device struct
 * @param[in] cfg: mipi dev(tx) ioctl cmd arg
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
static int32_t hobot_mipi_dev_ops_init_do(struct mipi_ddev_s *ddev, struct mipi_dev_cfg_s *cfg)
{
	struct os_dev *dev = &ddev->osdev;
	struct mipi_csi_dev_priv_s *priv = ddev->priv;
	struct mipi_csi_dev_cfg_t *tx_cfg;
	int32_t ret = 0, i;

	if (cfg == NULL)
		return -EINVAL;
	if ((priv == NULL) || (priv->ops.init == NULL)) {
		mipi_err(dev, "priv %p ops init %p invalid\n",
			priv, priv->ops.init);
		return -ENODEV;
	}
	tx_cfg = &priv->cfg;

	/* config set */
	tx_cfg->enable = 1U;
	tx_cfg->lanes = cfg->lane;
	tx_cfg->fps = cfg->fps;
	tx_cfg->datatype = cfg->datatype;
	tx_cfg->bpp = mipi_datatype2bpp(cfg->datatype);
	// tx_cfg->mclk = cfg->mclk;
	tx_cfg->mipiclk = cfg->mipiclk;
	tx_cfg->width = cfg->width;
	tx_cfg->height = cfg->height;
	tx_cfg->linelenth = cfg->linelenth;
	tx_cfg->framelenth = cfg->framelenth;
	tx_cfg->settle = cfg->settle;
	tx_cfg->vpg = cfg->vpg;
	tx_cfg->ipi_lines = cfg->ipi_lines;
	tx_cfg->channel_num = cfg->channel_num;
	for (i = 0; i < MIPIDEV_CHANNEL_NUM; i++)
		tx_cfg->channel_sel[i] = cfg->channel_sel[i];
	tx_cfg->lpclk_mode = 1U;
	tx_cfg->vpg_mode = 0U;
	tx_cfg->vpg_hsyncpkt_en = 0U;

	ret = priv->ops.init(ddev->port, priv->regs, NULL, tx_cfg);
	if (ret < 0) {
		mipi_err(dev, "csi tx init failed %d\n", ret);
	}

	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device operation start do
 *
 * @param[in] ddev: mipi dev(tx) device struct
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
static int32_t hobot_mipi_dev_ops_start_do(struct mipi_ddev_s *ddev)
{
	struct os_dev *dev = &ddev->osdev;
	struct mipi_csi_dev_priv_s *priv = ddev->priv;
	int32_t ret = 0;

	if ((priv == NULL) || (priv->ops.start == NULL)) {
		mipi_err(dev, "priv %p ops start %p invalid\n",
			priv, priv->ops.start);
		return -ENODEV;
	}

	ret = priv->ops.start(ddev->port, priv->regs, &priv->cfg);
	if (ret < 0) {
		mipi_err(dev, "csi tx start failed %d\n", ret);
	}

	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device operation stop do
 *
 * @param[in] ddev: mipi dev(tx) device struct
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
static int32_t hobot_mipi_dev_ops_stop_do(struct mipi_ddev_s *ddev)
{
	struct os_dev *dev = &ddev->osdev;
	struct mipi_csi_dev_priv_s *priv = ddev->priv;
	int32_t ret = 0;

	if ((priv == NULL) || (priv->ops.stop == NULL)) {
		mipi_err(dev, "priv %p ops stop %p invalid\n",
			priv, priv->ops.stop);
		return -ENODEV;
	}

	ret = priv->ops.stop(ddev->port, priv->regs);
	if (ret < 0) {
		mipi_err(dev, "csi tx stop failed %d\n", ret);
	}

	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device operation deinit do
 *
 * @param[in] ddev: mipi dev(tx) device struct
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
static int32_t hobot_mipi_dev_ops_deinit_do(struct mipi_ddev_s *ddev)
{
	struct os_dev *dev = &ddev->osdev;
	struct mipi_csi_dev_priv_s *priv = ddev->priv;
	int32_t ret = 0;

	if ((priv == NULL) || (priv->ops.deinit == NULL)) {
		mipi_err(dev, "priv %p ops deinit %p invalid\n",
			priv, priv->ops.deinit);
		return -ENODEV;
	}

	ret = priv->ops.deinit(ddev->port, priv->regs);
	if (ret < 0) {
		mipi_err(dev, "csi tx deinit failed %d\n", ret);
	}

	return ret;
}
#endif

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device open do
 *
 * @param[in] port: mipi dev(tx) device struct
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
static int32_t hobot_mipi_dev_open_do(struct mipi_ddev_s *ddev)
{
	struct mipi_dev_user_s *user = &ddev->user;
	struct os_dev *dev = &ddev->osdev;
	int32_t ret = 0;

	osal_mutex_lock(&user->open_mutex);
	mipi_debug(dev, "open as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
		osal_mutex_init(&user->mutex);
		user->init_cnt = 0U;
		user->start_cnt = 0U;
#ifdef MIPI_DEV_IDE_ENABLE
		ddev->priv = mipi_csi_dev_register(ddev->port, &ddev->tx_dev);
		if (IS_ERR_OR_NULL(ddev->priv)) {
			ddev->priv = NULL;
			mipi_err(dev, "csi dev register failed\n");
				ret = -EBUSY;
		}
#else
		mipi_info(dev, "dev%d dummy open %d\n", ddev->port, user->open_cnt);
#endif
		if (ret == 0) {
			osal_mutex_init(&user->mutex);
			user->init_cnt = 0U;
			user->start_cnt = 0U;
		}
	}
	if (ret == 0)
		user->open_cnt++;
	osal_mutex_unlock(&user->open_mutex);

	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device close do
 *
 * @param[in] ddev: mipi dev(tx) device struct
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
static int32_t hobot_mipi_dev_close_do(struct mipi_ddev_s *ddev)
{
	struct mipi_dev_user_s *user = &ddev->user;
	struct os_dev *dev = &ddev->osdev;
	int32_t ret = 0;

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt == 0U) {
		osal_mutex_unlock(&user->open_mutex);
		return 0;
	}
	user->open_cnt--;
	mipi_debug(dev, "close as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
#ifdef MIPI_DEV_IDE_ENABLE
		if (user->start_cnt != 0U) {
			(void)hobot_mipi_dev_ops_stop_do(ddev);
		}
		if (user->init_cnt != 0U) {
			(void)hobot_mipi_dev_ops_deinit_do(ddev);
		}
		(void)mipi_csi_dev_unregister(ddev->priv);
		ddev->priv = NULL;
#else
		mipi_info(dev, "dev%d dummy close %d\n", ddev->port, user->open_cnt);
#endif
	}
	osal_mutex_unlock(&user->open_mutex);

	return ret;
}


/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device ioctl do
 *
 * @param[in] ddev: mipi dev(tx) device struct
 * @param[in] cmd: mipi dev(tx) ioctl cmd
 * @param[in] arg: mipi dev(tx) ioctl cmd arg
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
static mipi_ioc_ret_t hobot_mipi_dev_ioctl_do(struct mipi_ddev_s *ddev, uint32_t cmd, mipi_ioc_arg_t arg)
{
	struct mipi_dev_user_s *user = &ddev->user;
	struct os_dev *dev = &ddev->osdev;
	int32_t ret = 0;
#ifdef MIPI_DEV_IDE_ENABLE
	mipi_dev_cfg_t mipi_dev_cfg;
#endif

	if (user->open_cnt == 0U) {
		mipi_err(dev, "dev%d not open before ioctl 0x%x\n", ddev->port, cmd);
		return -EACCES;
	}

	if ((char)(_IOC_TYPE(cmd)) != MIPIDEVIOC_MAGIC) {
		mipi_err(dev, "dev%d ioc 0x%x maigc not match\n", ddev->port, cmd);
		return -ENOTTY;
	}

#ifdef MIPI_DEV_IDE_ENABLE
	switch (cmd) {
	case MIPIDEVIOC_INIT:
		if (arg == 0U) {
			mipi_err(dev, "cmd init, arg NULL\n");
			return -EINVAL;
		}
		if (mipi_copy_from_app((void *)&mipi_dev_cfg,
			(void __user *)arg, sizeof(mipi_dev_cfg_t)) != 0U) {
			mipi_err(dev, "init error, config %px from user error\n", (void __user *)arg);
			return -EINVAL;
		}
		osal_mutex_lock(&user->mutex);
		mipi_info(dev, "init cmd: %u %s\n", user->init_cnt,
			(user->init_cnt != 0U) ? "drop" : "real");
		if (user->init_cnt == 0U)
			ret = hobot_mipi_dev_ops_init_do(ddev, &mipi_dev_cfg);
		if (ret == 0)
			user->init_cnt++;
		osal_mutex_unlock(&user->mutex);
		break;
	case MIPIDEVIOC_DEINIT:
		osal_mutex_lock(&user->mutex);
		if (user->init_cnt > 0U)
			user->init_cnt--;
		mipi_info(dev, "deinit cmd: %u %s\n", user->init_cnt,
			(user->init_cnt != 0U) ? "drop" : "real");
		if (user->init_cnt == 0U)
			(void)hobot_mipi_dev_ops_deinit_do(ddev);
		osal_mutex_unlock(&user->mutex);
		break;
	case MIPIDEVIOC_START:
		osal_mutex_lock(&user->mutex);
		mipi_info(dev, "start cmd: %u %s\n", user->start_cnt,
			(user->start_cnt != 0U) ? "drop" : "real");
		if (user->start_cnt == 0U)
			ret = hobot_mipi_dev_ops_start_do(ddev);
		if (ret == 0)
			user->start_cnt++;
		osal_mutex_unlock(&user->mutex);
		break;
	case MIPIDEVIOC_STOP:
		osal_mutex_lock(&user->mutex);
		if (user->start_cnt > 0U)
			user->start_cnt--;
		mipi_info(dev, "stop cmd: %u %s\n", user->start_cnt,
			(user->start_cnt != 0U) ? "drop" : "real");
		if (user->start_cnt == 0U)
			(void)hobot_mipi_dev_ops_stop_do(ddev);
		osal_mutex_unlock(&user->mutex);
		break;
	case MIPIDEVIOC_SET_BYPASS:
		osal_mutex_lock(&user->mutex);
		osal_mutex_unlock(&user->mutex);
		break;
	default:
		mipi_err(dev, "dev%d ioctl 0x%x - 0x%lx not support\n", ddev->port, cmd, arg);
		ret = -ERANGE;
		break;
	}
#else
	mipi_info(dev, "dev%d dummy ioctl 0x%x - 0x%lx\n", ddev->port, cmd, arg);
#endif

	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device setcb do
 *
 * @param[in] ddev: mipi dev(tx) device struct
 * @param[in] drop_cb: mipi dev(tx) device drop callback func
 * @param[in] int_cb: mipi dev(tx) device interrupt callback func
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
static int32_t hobot_mipi_dev_setcb_do(struct mipi_ddev_s *ddev, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb)
{
	int32_t ret = 0;

#ifdef MIPI_DEV_IDE_ENABLE
#else
	mipi_info(NULL, "dev%d dummy setcb\n", ddev->port);
#endif
	return ret;
}

/**
 * @NO{S10E03C01}
 * @ASIL{B}
 * @brief mipi dev(tx) device sys do
 *
 * @param[in] type: mipi dev(tx) sys type
 * @param[in] sub: mipi dev(tx) sys sub type
 * @param[in] ddev: mipi dev(tx) device struct
 * @param[in] name: mipi dev(tx) sys name string
 * @param[in] name: mipi dev(tx) sys buffer
 * @param[in] name: mipi dev(tx) sys buffer size
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
static int32_t hobot_mipi_dev_sys_do(int32_t type, int32_t sub, struct mipi_ddev_s *ddev,
			       const char *name, char *buf, int32_t count)
{
	int32_t ret = 0;

#ifdef MIPI_DEV_IDE_ENABLE
#else
	mipi_info(NULL, "dev%d dummy sys %d - %d - %s\n", ddev->port, type, sub, (name) ? name : "NULL");
#endif
	return ret;
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) device open
 *
 * @param[in] index: mipi dev(tx) device index
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
static int32_t mipi_dev_open(int32_t index)
{
	struct mipi_ddev_s *ddev = hobot_mipi_dev_ddev(index);

	if (ddev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_dev_open_do(ddev);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) device close
 *
 * @param[in] index: mipi dev(tx) device index
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
static int32_t mipi_dev_close(int32_t index)
{
	struct mipi_ddev_s *ddev = hobot_mipi_dev_ddev(index);

	if (ddev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_dev_close_do(ddev);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) device ioctl operation
 *
 * @param[in] index: mipi dev(tx) device index
 * @param[in] cmd: ioctl command
 * @param[in] arg: ioctl arg
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
static int32_t mipi_dev_ioctl(int32_t index, uint32_t cmd, unsigned long arg)
{
	struct mipi_ddev_s *ddev = hobot_mipi_dev_ddev(index);

	if (ddev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_dev_ioctl_do(ddev, cmd, arg);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) device callback set
 *
 * @param[in] index: mipi dev(tx) device index
 * @param[in] drop_cb: frame drop callbcak
 * @param[in] int_cb: interrupt callback for error
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
static int32_t mipi_dev_setcb(int32_t index, MIPI_DROP_CB drop_cb, MIPI_INT_CB int_cb)
{
	struct mipi_ddev_s *ddev = hobot_mipi_dev_ddev(index);

	if (ddev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_dev_setcb_do(ddev, drop_cb, int_cb);
}

/**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) device sys operation
 *
 * @param[in] index: mipi dev(tx) device index
 * @param[in] type: sys operation type
 * @param[in] sub: sys operation sub type show or stroe
 * @param[in] name: sys node name
 * @param[in] buf: info to show or store
 * @param[in] count: buf size

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
static int32_t mipi_dev_sys(int32_t index, int32_t type, int32_t sub, const char *name, char *buf, int32_t count)
{
	struct mipi_ddev_s *ddev = hobot_mipi_dev_ddev(index);

	if (ddev == NULL) {
		return -EEXIST;
	}

	return hobot_mipi_dev_sys_do(type, sub, ddev, name, buf, count);
}

/**
 * @brief mipi dev driver ops
 */
static const struct mipi_sub_ops_s mipi_dev_ops = {
	.open = mipi_dev_open,
	.close = mipi_dev_close,
	.ioctl = mipi_dev_ioctl,
	.setcb = mipi_dev_setcb,
	.sys = mipi_dev_sys,
	.osdev = NULL,
};

#ifdef CONFIG_HOBOT_MIPI_DEV
 /**
 * @NO{S10E03C01I}
 * @ASIL{B}
 * @brief mipi dev(tx) driver setup to probe devices
 *
 * @param[in] tx_ops: tx ops struct to store for csi driver
 *
 * @return >=0:Success and return dev device valid mask, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_mipi_dev_setup(const struct mipi_sub_ops_s **tx_ops)
{
	int32_t ret = 0, i;


	if (tx_ops != NULL) {
		ret = hobot_mipi_dev_setup_init();
		if (ret == 0) {
			for (i = 0; i < MIPI_DEV_MAX_NUM; i++) {
				if (hobot_mipi_dev_ddev(i) != NULL)
					ret |= (0x1U << i);
			}
			*tx_ops = &mipi_dev_ops;
		}
	} else {
		hobot_mipi_dev_setup_exit();
	}

	return ret;
}
#endif
