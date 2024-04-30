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
 * @file hobot_vin_vcon_osal.h
 *
 * @NO{S10E01C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_VIN_VCON_OSAL_H__
#define __HOBOT_VIN_VCON_OSAL_H__ /* PRQA S 0603 */ /* header file macro */

#include "osal.h"

typedef struct os_dev {
	dev_t              devno;
} os_dev_t;

/**
 * @def osdev_pre
 * vcon print pre string macro
 */
#define osdev_pre	"vin vcon"
/**
 * @def vcon_pr
 * vcon print macro: support debug,info,err,...
 */
#define vcon_pr(level_pr, dev, fmt, ...) do { \
		if ((dev) == NULL) \
			level_pr(osdev_pre ": " fmt, ##__VA_ARGS__); \
		else \
			level_pr(osdev_pre "%u: " fmt, ((((os_dev_t *)(dev))->devno) & 0xFFU), ##__VA_ARGS__); \
	} while(0)
#define vcon_debug(dev, fmt, ...)	vcon_pr(osal_pr_debug, dev, fmt, ##__VA_ARGS__)
#define vcon_info(dev, fmt, ...)	vcon_pr(osal_pr_info, dev, fmt, ##__VA_ARGS__)
#define vcon_err(dev, fmt, ...)		vcon_pr(osal_pr_err, dev, fmt, ##__VA_ARGS__)

#endif // __HOBOT_VIN_VCON_OSAL_H__
