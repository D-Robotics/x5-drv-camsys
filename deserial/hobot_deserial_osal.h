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

#ifndef __HOBOT_DESERIAL_OSAL_H__
#define __HOBOT_DESERIAL_OSAL_H__ /* PRQA S 0603 */ /* header file macro */

#include "osal.h"

#include <linux/miscdevice.h>

typedef struct os_dev {
	dev_t              devno;
	int32_t            minor_id;
	struct miscdevice  miscdev;
} os_dev_t;

/**
 * @def osdev_pre
 * deserial print pre string macro
 */
#define osdev_pre	"vin deserial"
/**
 * @def des_pr
 * deserial print macro: support debug,info,err,...
 */
#define des_pr(level_pr, dev, fmt, ...) do { \
		if ((dev) == NULL) \
			level_pr(osdev_pre ": " fmt, ##__VA_ARGS__); \
		else \
			level_pr(osdev_pre "%u: " fmt, ((((os_dev_t *)(dev))->devno) & 0xFFU), ##__VA_ARGS__); \
	} while(0)
#define des_debug(dev, fmt, ...)	des_pr(osal_pr_debug, dev, fmt, ##__VA_ARGS__)
#define des_info(dev, fmt, ...)		des_pr(osal_pr_info, dev, fmt, ##__VA_ARGS__)
#define des_warn(dev, fmt, ...)		des_pr(osal_pr_warn, dev, fmt, ##__VA_ARGS__)
#define des_err(dev, fmt, ...)		des_pr(osal_pr_err, dev, fmt, ##__VA_ARGS__)

#endif // __HOBOT_DESERIAL_OSAL_H__
