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
 * @file hobot_sensor_osal.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef __HOBOT_SENSOR_OSAL_H__
#define __HOBOT_SENSOR_OSAL_H__ /* PRQA S 0603 */ /* header file macro */

#include "osal.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>

typedef struct os_dev {
	dev_t              devno;
	int32_t            minor_id;
	struct miscdevice  miscdev;
	struct i2c_client *client;
	struct i2c_board_info board_info;
} os_dev_t;

/**
 * @def osdev_pre
 * sensor print pre string macro
 */
#define osdev_pre	"vin sensor"
/**
 * @def des_pr
 * senerial print macro: support debug,info,err,...
 */
#define sen_pr(level_pr, dev, fmt, ...) do { \
		if ((dev) == NULL) \
			level_pr(osdev_pre ": " fmt, ##__VA_ARGS__); \
		else \
			level_pr(osdev_pre "%u: " fmt, ((((os_dev_t *)(dev))->devno) & 0xFFU), ##__VA_ARGS__); \
	} while(0)
#define sen_debug(dev, fmt, ...)	sen_pr(osal_pr_debug, dev, fmt, ##__VA_ARGS__)
#define sen_info(dev, fmt, ...)		sen_pr(osal_pr_info, dev, fmt, ##__VA_ARGS__)
#define sen_warn(dev, fmt, ...)		sen_pr(osal_pr_warn, dev, fmt, ##__VA_ARGS__)
#define sen_err(dev, fmt, ...)		sen_pr(osal_pr_err, dev, fmt, ##__VA_ARGS__)

#endif // __HOBOT_SENSOR_OSAL_H__
