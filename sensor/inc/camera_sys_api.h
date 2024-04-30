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
 * @file camera_sys_api.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_SYS_API_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_SYS_API_H_

#define NORMAL_M	1u
#define DOL2_M	2u
#define DOL3_M	3u
#define DOL4_M	4u
#define PWL_M	5u

int32_t camera_sys_priv_set(uint32_t port, sensor_priv_t *priv_param);
int32_t camera_sys_get_param(uint32_t port, sensor_data_t *sensor_data);
int32_t camera_sys_tuning_set(uint32_t port, sensor_tuning_data_t *tuning_pram);
int32_t camera_sys_get_tuning_param(uint32_t port, sensor_tuning_data_t *tuning_pram);
void camera_sys_lut_free(uint32_t port);
int32_t	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain);
int32_t camera_sys_alloc_dgain(uint32_t port, uint32_t *a_gain);
int32_t camera_sys_alloc_intergration_time(uint32_t port,
		uint32_t *intergration_time);
int32_t camera_sys_sensor_write(uint32_t port, uint32_t address,
		uint32_t w_data);
int32_t camera_sys_sensor_read(uint32_t port, uint32_t address,
		uint32_t *r_data);
int32_t camera_sys_stream_on(uint32_t port);
int32_t camera_sys_stream_off(uint32_t port);
int32_t camera_sys_priv_awb_set(uint32_t port, sensor_priv_t *priv_param);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SYS_API_H_
