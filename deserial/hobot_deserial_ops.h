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
 * @file hobot_deserial_ops.h
 *
 * @NO{S10E02C11}
 * @ASIL{B}
 */

#ifndef __HOBOT_DESERIAL_OPS_H__
#define __HOBOT_DESERIAL_OPS_H__

#include "hobot_deserial_osal.h"
#include "hobot_vin_vcon_sub.h"
#include "hobot_camera_reg.h"

#define DESERIAL_VER_MAJOR	(1u)
#define DESERIAL_VER_MINOR	(0u)

/**
 * @def DESERIAL_NUM_MAX
 * deserial max number
 */
#define DESERIAL_NUM_MAX	(12)

/**
 * @def DESERIAL_DEV_NAME
 * deserial device name string
 */
#define DESERIAL_DEV_NAME	"deserial_"

#ifdef CONFIG_HOBOT_DESERIAL_NUM
#define DESERIAL_NUM_DEFAULT	CONFIG_HOBOT_DESERIAL_NUM
#else
#define DESERIAL_NUM_DEFAULT	(6)
#endif
/**
 * @def DESERIAL_DEV_NAME_LEN
 * deserial dev name string length
 */
#define DESERIAL_DEV_NAME_LEN	(32)
/**
 * @def DESERIAL_LINK_NUM_MAX
 * deserial link number maaxl
 */
#define DESERIAL_LINK_NUM_MAX	(4)
/**
 * @def DESERIAL_NAME_LEN_MA
 * deserial chip name string length
 */
#define DESERIAL_NAME_LEN_MAX	(32)
/**
 * @def DESERIAL_GPIO_NUM_MAX
 * deserial gpio number max
 */
#define DESERIAL_GPIO_NUM_MAX	(32)
/**
 * @def DESERIAL_PORT_DESPLEN
 * deserial port description string length
 */
#define DESERIAL_PORT_DESPLEN	(64)

#define DES_DEBUG	LOGLEVEL_DEBUG
#define DES_INFO	LOGLEVEL_INFO
#define DES_ERR		LOGLEVEL_ERR

/**
 * @def DESERIAL_COP_MODULE
 * the module id of deserial drivers ops to used
 */
#define DESERIAL_COP_MODULE	VCON_COP_MODULE
/**
 * @def DESERIAL_VCON_COP_TYPE
 * the type id of deserial drivers for vcon ops to used
 */
#define DESERIAL_VCON_COP_TYPE	VCON_COPT_DESERIAL
/**
 * @def DESERIAL_COPS_MAGIC
 * the type id of sensor drivers for vcon ops to used
 */
#define DESERIAL_COPS_MAGIC	(0x40446573)

/**
 * @struct deserial_version_info_s
 * deserial driver version info struct
 * @NO{S10E02C11}
 */
typedef struct deserial_version_info_s {
	uint32_t major;/**< the major version number >*/
	uint32_t minor;/**< the minor version number >*/
} deserial_version_info_t;

/**
 * @struct deseial_pre_state_e
 * deserial device pre operation state
 * @NO{S10E02C11}
 */
enum deseial_pre_state_e {
	DES_PRE_STATE_DEFAULT = 0,
	DES_PRE_STATE_INITING,
	DES_PRE_STATE_INITED,
	DES_PRE_STATE_STARTING,
	DES_PRE_STATE_STARTED,
	DES_PRE_STATE_MAX,
};

/**
 * @def DESERIAL_PRE_STATE_NAMES
 * deserial pre state name strings
 */
#define DESERIAL_PRE_STATE_NAMES { \
	"default", \
	"initing", \
	"inited", \
	"starting", \
	"started", \
}

/**
 * @struct deseial_op_state_e
 * deserial device user operation state
 * @NO{S10E02C11}
 */
enum deseial_op_state_e {
	DES_OP_STATE_DEFAULT = 0,
	DES_OP_STATE_WAIT,
	DES_OP_STATE_DOING_NOWAIT,
	DES_OP_STATE_DOING,
	DES_OP_STATE_DONE,
	DES_OP_STATE_ERROR,
	DES_OP_STATE_FINISH,
	DES_OP_STATE_CANCEL,
	DES_OP_STATE_MAX,
};

/**
 * @def DESERIAL_OP_STATE_NAMES
 * deserial op state name strings
 */
#define DESERIAL_OP_STATE_NAMES { \
	"default", \
	"wait", \
	"doing_nowait", \
	"doing", \
	"done", \
	"error", \
	"finish", \
	"cancel", \
}

/**
 * @struct deserial_info_data_s
 * deserial device info data struct
 * @NO{S10E02C11}
 */
typedef struct deserial_info_data_s {
	uint32_t  index;
	char      deserial_name[DESERIAL_NAME_LEN_MAX];
	uint32_t  deserial_addr;
	char      poc_name[DESERIAL_NAME_LEN_MAX];
	uint32_t  poc_addr;
	uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_addr;
	uint32_t  chip_id;
	int32_t  gpios[DESERIAL_GPIO_NUM_MAX];
	uint32_t  gpio_enable;
	uint32_t  gpio_level;
	uint32_t  poc_map;
	uint32_t  link_map;
	int32_t sensor_index[DESERIAL_LINK_NUM_MAX];
	char link_desp[DESERIAL_LINK_NUM_MAX][DESERIAL_PORT_DESPLEN];
	camera_reg_t init;
	camera_reg_t start;
	camera_reg_t stop;
	camera_reg_t deinit;
} deserial_info_data_t;

/**
 * @struct deserial_op_state_e
 * deserial device user operation state
 * @NO{S10E02C11}
 */
typedef enum deserial_op_type_e {
	DES_OP_TYPE_INVALID = 0,
	DES_OP_TYPE_STREAM,
	DES_OP_TYPE_MAX,
} deserial_op_type_t;

/**
 * @struct deserial_op_info_s
 * deserial device operation info struct
 * @NO{S10E02C11}
 */
typedef struct deserial_op_info_s {
	int32_t type;
	int32_t data;
	int32_t reserved[2];
} deserial_op_info_t;

#define DESERIAL_IOC_MAGIC      's'
#define DESERIAL_DATA_INIT      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 0, deserial_info_data_t)
#define DESERIAL_INIT_REQ       _IOW((uint32_t)DESERIAL_IOC_MAGIC, 1, int32_t)
#define DESERIAL_INIT_RESULT    _IOW((uint32_t)DESERIAL_IOC_MAGIC, 2, int32_t)
#define DESERIAL_DEINIT_REQ     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 3, int32_t)
#define DESERIAL_START_REQ      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 4, int32_t)
#define DESERIAL_START_RESULT   _IOW((uint32_t)DESERIAL_IOC_MAGIC, 5, int32_t)
#define DESERIAL_STOP_REQ       _IOW((uint32_t)DESERIAL_IOC_MAGIC, 6, int32_t)
#define DESERIAL_STREAM_GET     _IOR((uint32_t)DESERIAL_IOC_MAGIC, 7, deserial_op_info_t)
#define DESERIAL_STREAM_PUT     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 8, int32_t)
#define DESERIAL_STREAM_ON      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 9, int32_t)
#define DESERIAL_STREAM_OFF     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 10, int32_t)
#define DESERIAL_GET_VERSION    _IOR((uint32_t)DESERIAL_IOC_MAGIC, 11, deserial_version_info_t)
#define DESERIAL_STATE_CHECK    _IOR((uint32_t)DESERIAL_IOC_MAGIC, 12, uint32_t)
#define DESERIAL_STATE_CONFIRM  _IOW((uint32_t)DESERIAL_IOC_MAGIC, 13, uint32_t)
#define DESERIAL_STATE_CLEAR    _IOW((uint32_t)DESERIAL_IOC_MAGIC, 14, uint32_t)

/**
 * @struct deserial_user_s
 * deserial device user struct for operation
 * @NO{S10E02C11}
 */
struct deserial_user_s {
	osal_mutex_t open_mutex;
	osal_mutex_t mutex;
	uint32_t open_cnt;
	uint32_t init_cnt;
	uint32_t start_cnt;
	uint32_t data_init;
	uint32_t pre_state;
	uint32_t op_state;
	uint32_t op_wait;
	bool pre_done;
	osal_waitqueue_t pre_wq;
	osal_waitqueue_t opu_wq;
	osal_waitqueue_t opk_wq;
	deserial_op_info_t *op_info;
	deserial_op_info_t op_nowait;
};

/**
 * @struct deserial_miscdev_s
 * deserial misc device struct for hw operation
 * @NO{S10E02C11}
 */
struct deserial_miscdev_s {
	char name[DESERIAL_DEV_NAME_LEN];
	int32_t bus_num;
	int32_t addr;
	uint32_t devflag;
	void *client;
	// struct miscdevice miscdev;
	// struct i2c_client *client;
	// struct i2c_board_info board_info;
};

/**
 * @struct deserial_link_s
 * deserial device link struct for operation
 * @NO{S10E02C11}
 */
struct deserial_link_s {
	int32_t attach;
	int32_t linked;
	int32_t flow_id;
	int32_t sensor_id;
	int32_t poc_id;
	int32_t init_by_pid;
	char *desp;
};

/**
 * @struct deserial_param_s
 * deserial device param struct for debug
 * @NO{S10E02C11}
 */
struct deserial_param_s {
	/* must be int32_t */
	int32_t op_timeout_ms;
	int32_t op_retry_max;
};

/**
 * @def DESERIAL_PARAM_NAMES
 * the deserial param name string array
 */
#define DESERIAL_PARAM_NAMES { \
	"op_timeout_ms", \
	"op_retry_max", \
}

/**
 * @def DESERIAL_PARAM_OPEN_RESET_MASK
 * the deserial param mask which should reset when open first:
 */
#define DESERIAL_PARAM_OPEN_RESET_MASK	(0x0)

#define DESERIAL_PARAM_OP_TIMEOOUT_MS_DEFAULT	(-1) // (500)
#define DESERIAL_PARAM_OP_RETRY_MAX_DEFAULT	(3)

/**
 * @struct deserial_device_s
 * deserial device struct
 * @NO{S10E02C11}
 */
struct deserial_device_s {
	int32_t index;
	int32_t attach_link;
	struct os_dev osdev;
	struct deserial_user_s user;
	struct deserial_miscdev_s mdev;
	struct deserial_param_s param;
	deserial_info_data_t deserial_info;
	struct deserial_link_s link[DESERIAL_LINK_NUM_MAX];
};

/**
 * @struct deserial_s
 * deserial driver struct
 * @NO{S10E02C11}
 */
struct deserial_s {
	struct deserial_version_info_s ver;
	struct deserial_device_s *des;
	int32_t des_num;
	vcon_cb vcon_event_cb;
};

/* inter  apis */
extern struct deserial_s* deserial_global(void);
extern int32_t deserial_device_open(struct deserial_device_s *des);
extern int32_t deserial_device_close(struct deserial_device_s *des);
extern int32_t deserial_device_ioctl(struct deserial_device_s *des, uint32_t cmd, unsigned long arg);
extern int32_t deserial_device_status_info_show(struct deserial_device_s *des, char *buf);
extern int32_t deserial_device_status_cfg_show(struct deserial_device_s *des, char *buf);
extern int32_t deserial_device_status_regs_show(struct deserial_device_s *des, char *buf);
extern int32_t deserial_device_status_user_show(struct deserial_device_s *des, char *buf);
extern int32_t deserial_device_param_show(struct deserial_device_s *des, const char *name, char *buf);
extern int32_t deserial_device_param_store(struct deserial_device_s *des, const char *name, const char *buf, int32_t count);
extern int32_t deserial_device_init(struct deserial_device_s *des);
extern void deserial_device_exit(struct deserial_device_s *des);
extern int32_t deserial_driver_init(void);
extern void deserial_driver_exit(void);

#endif /* __HOBOT_DESERIAL_OPS_H__ */
