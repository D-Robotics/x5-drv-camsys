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
 * @file hobot_mipi_osal.h
 *
 * @NO{S10E03C01}
 * @ASIL{B}
 */

#ifndef __HOBOT_MIPI_OSAL_H__
#define __HOBOT_MIPI_OSAL_H__ /* PRQA S 0603 */ /* header file macro */

#include "osal.h"

/* pr with dev */
#include <linux/device.h>
#include <linux/cdev.h>

/**
 * @struct os_dev
 * os device struct
 * @NO{S10E03C02}
 */
typedef struct os_dev {
	dev_t              devno;
	struct cdev        cdev;
	struct device     *dev;
} os_dev_t;

/**
 * @def osdev_pre
 * mipi print pre string macro
 */
#ifndef osdev_pre
#define osdev_pre	"vin mipi"
#endif
/**
 * @def mipi_pr
 * mipi print macro: support debug,info,err,...
 */
#define mipi_pr(level_pr, dev, fmt, ...) do { \
		if ((dev) == NULL) \
			level_pr(osdev_pre ": " fmt, ##__VA_ARGS__); \
		else \
			level_pr(osdev_pre "%u: " fmt, ((((os_dev_t *)(dev))->devno) & 0xFFU), ##__VA_ARGS__); \
	} while(0)
#define mipi_debug(dev, fmt, ...)	mipi_pr(osal_pr_debug, dev, fmt, ##__VA_ARGS__)
#define mipi_info(dev, fmt, ...)	mipi_pr(osal_pr_info, dev, fmt, ##__VA_ARGS__)
#define mipi_err(dev, fmt, ...)		mipi_pr(osal_pr_err, dev, fmt, ##__VA_ARGS__)

#endif //__HOBOT_MIPI_OSAL_H__
