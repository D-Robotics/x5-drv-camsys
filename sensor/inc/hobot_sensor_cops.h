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
 * @file hobot_sensor_cops.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef __HOBOT_SENSOR_COPS_H__
#define __HOBOT_SENSOR_COPS_H__

#include "hobot_vin_vcon_sub.h"
#include "common_camera.h"

/**
 * @def SENSOR_COP_MODULE
 * the module id of sensor drivers ops to used
 */
#define SENSOR_COP_MODULE	VCON_COP_MODULE
/**
 * @def SENSOR_VCON_COP_TYPE
 * the type id of sensor drivers for vcon ops to used
 */
#define SENSOR_VCON_COP_TYPE	VCON_COPT_SENSOR
/**
 * @def SENSOR_ISP_COP_TYPE
 * the type id of sensor drivers for vcon ops to used
 */
#define SENSOR_ISP_COP_TYPE	COPS_4
/**
 * @def SENSOR_CIM_COP_TYPE
 * the type id of sensor drivers for vcon ops to used
 */
#define SENSOR_CIM_COP_TYPE	COPS_5
/**
 * @def SENSOR_ISI_COP_TYPE
 * the type id of sensor drivers for isi isp sensor ops to used
 */
#define SENSOR_ISI_COP_TYPE	COPS_8

#endif /* __HOBOT_SENSOR_COPS_H__ */
