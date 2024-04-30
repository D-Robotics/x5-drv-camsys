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
 * @file hobot_sensor_ops.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef __HOBOT_SENSOR_OPS_H__
#define __HOBOT_SENSOR_OPS_H__

#include "hobot_sensor_osal.h"
#include "hobot_sensor_ver.h"
#include "hobot_vin_vcon_sub.h"
#include "camera_dev.h"

#define SENSOR_VER_MAJOR	(1u)
#define SENSOR_VER_MINOR	(0u)

/**
 * @def SENSOR_NUM_MAX
 * sensor max number
 */
#define SENSOR_NUM_MAX	(32)

/**
 * @def SENSOR_DEV_NAME
 * sensor device name string
 */
#define SENSOR_DEV_NAME	"port_"

#define SENSOR_IOC_MAGIC      'x'
#define SENSOR_TUNING_PARAM   _IOW(SENSOR_IOC_MAGIC, 0, sensor_tuning_data_t)
#define SENSOR_OPEN_CNT       _IOR(SENSOR_IOC_MAGIC, 1, int32_t)
#define SENSOR_SET_START_CNT  _IOW(SENSOR_IOC_MAGIC, 2, int32_t)
#define SENSOR_GET_START_CNT  _IOR(SENSOR_IOC_MAGIC, 3, int32_t)
#define SENSOR_USER_LOCK      _IOW(SENSOR_IOC_MAGIC, 4, int32_t)
#define SENSOR_USER_UNLOCK    _IOW(SENSOR_IOC_MAGIC, 5, int32_t)
#define SENSOR_AE_SHARE       _IOW(SENSOR_IOC_MAGIC, 6, int32_t)

#define SENSOR_SET_INIT_CNT   _IOW(SENSOR_IOC_MAGIC, 8, int32_t)
#define SENSOR_GET_INIT_CNT   _IOR(SENSOR_IOC_MAGIC, 9, int32_t)
#define SENSOR_INPUT_PARAM    _IOW(SENSOR_IOC_MAGIC, 10, sensor_input_param_t)
#define SENSOR_SET_INTRINSIC_PARAM _IOW(SENSOR_IOC_MAGIC, 11, cam_usr_info_t)
#define SENSOR_GET_INTRINSIC_PARAM _IOR(SENSOR_IOC_MAGIC, 12, cam_usr_info_t)

#define SENSOR_INIT_REQ       _IOW(SENSOR_IOC_MAGIC, 16, int32_t)
#define SENSOR_INIT_RESULT    _IOW(SENSOR_IOC_MAGIC, 17, int32_t)
#define SENSOR_DEINIT_REQ     _IOW(SENSOR_IOC_MAGIC, 18, int32_t)
#define SENSOR_START          _IOW(SENSOR_IOC_MAGIC, 19, int32_t)
#define SENSOR_STOP           _IOW(SENSOR_IOC_MAGIC, 20, int32_t)
#define SENSOR_EVENT_GET      _IOR(SENSOR_IOC_MAGIC, 21, sensor_event_info_t)
#define SENSOR_EVENT_PUT      _IOW(SENSOR_IOC_MAGIC, 22, int32_t)
#define SENSOR_UPDATE_AE_INFO _IOW(SENSOR_IOC_MAGIC, 23, sensor_ae_info_t)
#define SENSOR_GET_VERSION    _IOR(SENSOR_IOC_MAGIC, 24, sensor_version_info_t)

#if CAMERA_TOTAL_NUMBER > SENSOR_NUM_MAX
#error CAMERA_TOTAL_NUMBER over SENSOR_NUM_MAX error
#else
#define SENSOR_NUM_DEFAULT	CAMERA_TOTAL_NUMBER
#endif

#define SENSOR_NS2S(ns)		((ns) / 1000000000U)
#define SENSOR_NS2SNS(ns)	((ns) % 1000000000U)
#define SENSOR_NS2MS(ns)	((ns) / 1000000U)
#define SENSOR_NS2MSNS(ns)	((ns) % 1000000U)
#define SENSOR_NS2FPKS(ns)	((uint32_t)(1000000000000ULL / (ns)))

/**
 * @struct sensor_s
 * sensor driver struct
 * @NO{S10E02C08}
 */
struct sensor_s {
	struct sensor_version_info_s ver;
	struct sensor_device_s *sen;
	struct sensor_device_s *sen_flow[SENSOR_NUM_MAX];
	int32_t sen_num;
	vcon_cb vcon_event_cb;
};

/* inter apis */
extern struct sensor_s* sensor_global(void);
extern struct sensor_device_s* sensor_dev_get(int32_t port);
extern struct os_dev* sensor_osdev_get(int32_t port);
extern struct sensor_device_s* sensor_dev_get_by_flow(int32_t flow_id);
extern int32_t sensor_device_open(struct sensor_device_s *sen);
extern int32_t sensor_device_close(struct sensor_device_s *sen);
extern int32_t sensor_device_ioctl(struct sensor_device_s *sen, uint32_t cmd, unsigned long arg);
extern int32_t sensor_device_status_info_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_cfg_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_iparam_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_regs_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_user_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_frame_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_fps_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_status_fps_record_show(struct sensor_device_s *sen, char *buf);
extern int32_t sensor_device_param_show(struct sensor_device_s *sen, const char *name, char *buf);
extern int32_t sensor_device_param_store(struct sensor_device_s *sen, const char *name, const char *buf, int32_t count);
extern int32_t sensor_streaming_do(struct sensor_device_s *sen, int32_t state);
extern int32_t sensor_device_init(struct sensor_device_s *sen);
extern void sensor_device_exit(struct sensor_device_s *sen);
extern int32_t sensor_driver_init(void);
extern void sensor_driver_exit(void);

#endif /* __HOBOT_SENSOR_OPS_H__ */
