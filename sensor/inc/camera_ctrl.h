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
 * @file camera_ctrl.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_CTRL_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_CTRL_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>

#include "inc/camera_subdev.h"
#include "hobot_sensor_ver.h"

#define SENSOR_CTRL_VER_MAJOR	(1u)
#define SENSOR_CTRL_VER_MINOR	(0u)

#define CTRL_DEVNAME_LEN  100
#define CTRL_DATA_FLAG (0x00)
#define CTRL_FS_FLAG (0x01)
#define CTRL_BUF_SELECT_FLAG (0x04)
#define CTRL_BUF_PING (0x00)
#define CTRL_WAKE_UP_FLAG ((1u << CTRL_DATA_FLAG) | (1u << CTRL_FS_FLAG))

/**
 * @struct sensor_ctrl_info_s
 * camera sensor control info struct
 * @NO{S10E02C08}
 */
typedef struct sensor_ctrl_info_s {
	uint32_t port;
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint32_t grgain;
	uint32_t gbgain;
	uint32_t af_pos;
	uint32_t zoom_pos;
	uint32_t mode;
	uint32_t color_temper;
	uint32_t id;
	uint32_t reserverd[7];
} sensor_ctrl_info_t;

/**
 * @struct sensor_ctrl_result_s
 * camera sensor control result struct
 * @NO{S10E02C08}
 */
typedef struct sensor_ctrl_result_s {
	uint32_t port;
	uint32_t id;
	uint32_t ops;
	int32_t result;
} sensor_ctrl_result_t;

#define CAMERA_CTRL_IOC_MAGIC    'x'
#define SENSOR_CTRL_INFO_SYNC	_IOWR((uint32_t)CAMERA_CTRL_IOC_MAGIC, 40, sensor_ctrl_info_t)
#define SENSOR_CTRL_RESULT	_IOW((uint32_t)CAMERA_CTRL_IOC_MAGIC, 41, sensor_ctrl_result_t)
#define SENSOR_CTRL_GET_VERSION _IOR((uint32_t)CAMERA_CTRL_IOC_MAGIC, 48, sensor_version_info_t)

/**
 * @struct _camera_ctrlmod_s
 * camera sensor control device struct
 * @NO{S10E02C08}
 */
typedef struct _camera_ctrlmod_s {
        char name[CTRL_DEVNAME_LEN];
        uint32_t user_num;
        int32_t dev_minor_id;
        struct miscdevice camera_chardev;
        // wait queue
        uint32_t port_flag[CAMERA_TOTAL_NUMBER];
} camera_ctrlmod_s;

void set_sensor_aexp_info(uint32_t chn, void *ptr);
int camera_ctrldev_init(void);
void camera_ctrldev_exit(void);
void sensor_ctrl_wakeup(uint32_t chn);
void sensor_ctrl_wakeup_flag(uint32_t chn);

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
