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
 * @file hobot_sensor_ver.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef __HOBOT_SENSOR_VER_H__
#define __HOBOT_SENSOR_VER_H__

/**
 * @struct sensor_version_info_s
 * sensor driver version info struct
 * @NO{S10E02C08}
 */
typedef struct sensor_version_info_s {
	uint32_t major;/**< the major version number >*/
	uint32_t minor;/**< the minor version number >*/
} sensor_version_info_t;

#endif /* __HOBOT_SENSOR_VER_H__ */
