/*    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

/**
 * @file camera_i2c.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_I2C_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_I2C_H_

#include "camera_dev.h"

#define CAMERA_I2C_BYTE_MAX	(100)

int32_t camera_i2c_open(struct sensor_device_s *sen, uint32_t bus_num, char *sensor_name, uint32_t sensor_addr);
int32_t camera_i2c_release(struct sensor_device_s *sen);
int32_t camera_i2c_isdummy(struct sensor_device_s *sen);
int32_t camera_i2c_read(struct sensor_device_s *sen, uint32_t reg_addr,
		uint32_t bit_width, char *buf, uint32_t count);
int32_t camera_i2c_write(struct sensor_device_s *sen, uint32_t reg_addr,
		uint32_t bit_width, const char *buf, uint32_t count);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_I2C_H_

