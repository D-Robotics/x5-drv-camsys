/*
 * hobot-drivers/camsys/idu/hobot_idu_vnode_dev.h
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
#ifndef HOBOT_IDU_VNODE_DEBUG_H
#define HOBOT_IDU_VNODE_DEBUG_H
#include "hobot_idu_vnode_dev.h"

extern struct hobot_idu_debug_priv *g_debug_priv[IDU_DEV_NUM];

struct dentry *hobot_idu_vnode_create_debugfs(struct hobot_idu_debug_priv *debug_priv, uint8_t id);

#endif //HOBOT_IDU_VNODE_DEBUG_H