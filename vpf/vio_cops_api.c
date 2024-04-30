/**
 * @file: vio_cops_api.c
 * @
 * @NO{S090E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)    "[VIO cops]:" fmt

#include "vio_node_api.h"
#include "vio_cops_api.h"
#include "hobot_vpf_manager.h"

s32 vio_register_callback_ops(struct vio_callback_ops *vio_cops, u32 module, enum cops_type ctype)
{
	struct vio_callback_ops *vcops;
	struct hobot_vpf_dev *vpf_dev;

	if (vio_cops == NULL) {
		vio_err("%s: vio_cops is NULL\n", __func__);
		return -EINVAL;
	}

	if (module >= MODULE_NUM || ctype >= COPS_NUM) {
		vio_err("%s: wrong input parameters module(%d) and ctype(%d)", __func__, module, ctype);
		return -EINVAL;
	}

	vpf_dev = vpf_get_drvdata();
	vcops = &vpf_dev->vio_cops[module][ctype];

	osal_mutex_lock(&vpf_dev->mlock);
	vcops->cops = vio_cops->cops;
	osal_mutex_unlock(&vpf_dev->mlock);

	return 0;
}
EXPORT_SYMBOL(vio_register_callback_ops);/*PRQA S 0605,0307*/

void vio_unregister_callback_ops(u32 module, enum cops_type ctype)
{
	struct vio_callback_ops *vio_cops;
	struct hobot_vpf_dev *vpf_dev;

	if (module >= MODULE_NUM || ctype >= COPS_NUM) {
		vio_err("%s: wrong input parameters module(%d) and ctype(%d)", __func__, module, ctype);
		return;
	}

	vpf_dev = vpf_get_drvdata();
	vio_cops = &vpf_dev->vio_cops[module][ctype];

	osal_mutex_lock(&vpf_dev->mlock);
	vio_cops->cops = vio_cops->empty_ops;
	osal_mutex_unlock(&vpf_dev->mlock);
}
EXPORT_SYMBOL(vio_unregister_callback_ops);/*PRQA S 0605,0307*/

void *vio_get_callback_ops(void *empty_ops, u32 module, enum cops_type ctype)
{
	struct vio_callback_ops *vio_cops;
	struct hobot_vpf_dev *vpf_dev;

	if (empty_ops == NULL) {
		vio_err("%s: empty_ops is NULL\n", __func__);
		return NULL;
	}

	if (module >= MODULE_NUM || ctype >= COPS_NUM) {
		vio_err("%s: wrong input parameters module(%d) and ctype(%d)", __func__, module, ctype);
		return NULL;
	}

	vpf_dev = vpf_get_drvdata();
	vio_cops = &vpf_dev->vio_cops[module][ctype];

	osal_mutex_lock(&vpf_dev->mlock);
	if ((vio_cops->cops == NULL) || (vio_cops->cops == vio_cops->empty_ops))
		vio_cops->cops = empty_ops;
	vio_cops->empty_ops = empty_ops;
	osal_mutex_unlock(&vpf_dev->mlock);

	return vio_cops;
}
EXPORT_SYMBOL(vio_get_callback_ops);/*PRQA S 0605,0307*/
