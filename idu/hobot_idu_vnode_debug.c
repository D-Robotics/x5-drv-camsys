/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_debug.c
 *
 * Copyright (C) 2020 horizon
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>

#include "hobot_idu_vnode_debug.h"

struct hobot_idu_debug_priv *g_debug_priv[IDU_DEV_NUM] ={NULL, };

static int store_config_log_enable(void *data, u64 val)
{
	struct hobot_idu_debug_priv *debug = (struct hobot_idu_debug_priv *)data;

	debug->config_log = val;

	return 0;
}
static int show_config_log_enable(void *data, u64 *val)
{
	struct hobot_idu_debug_priv *debug = (struct hobot_idu_debug_priv *)data;

	*val = debug->config_log;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(config_log_enable, show_config_log_enable, store_config_log_enable, "%llu\n");

struct dentry *hobot_idu_vnode_create_debugfs(struct hobot_idu_debug_priv *debug_priv, uint8_t id)
{
	struct dentry *debugfs = NULL;
	char	devname[16] = {0, };

	snprintf(devname, sizeof(devname), "%s%d", "idu_vnode", id);
	debugfs = debugfs_create_dir(devname, NULL);
	if (!debugfs) {
		vio_err("%s create debugfs fail\n", devname);
		return NULL;
	}
	debugfs_create_file("config_log_enable", S_IALLUGO, debugfs, debug_priv, &config_log_enable);
	g_debug_priv[id] = debug_priv;
	g_debug_priv[id]->id = id;

	return debugfs;
}