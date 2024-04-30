// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>

#include "vio_node_api.h"

#include "cam_online_ops.h"

static const struct cam_online_ops *cam_online_ops_for_all[MODULE_NUM];

int add_online_ops(u32 id, const struct cam_online_ops *ops)
{
	if (id < MODULE_NUM) {
		cam_online_ops_for_all[id] = ops;
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(add_online_ops);

const struct cam_online_ops *get_online_ops(u32 id)
{
	const struct cam_online_ops *ops = NULL;

	if (id < MODULE_NUM)
		ops = cam_online_ops_for_all[id];
	return ops;
}
EXPORT_SYMBOL(get_online_ops);

static int __init cam_online_ops_init_module(void)
{
	return 0;
}

static void __exit cam_online_ops_exit_module(void)
{
}

module_init(cam_online_ops_init_module);
module_exit(cam_online_ops_exit_module);

MODULE_DESCRIPTION("VeriSilicon Camera Control Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VS-CAM-CTRL");
