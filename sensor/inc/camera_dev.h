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
 * @file camera_dev.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_DEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_DEV_H_

#include "hobot_sensor_osal.h"
#include "camera_subdev.h"

/**
 * @def SENSOR_DEV_NAME_LEN
 * sensor dev name string length
 */
#define SENSOR_DEV_NAME_LEN	(32)
/**
 * @def SENSOR_NAME_LEN_MA
 * sensor chip name string length
 */
#define SENSOR_NAME_LEN_MAX	(32)

#define LEN_VENDOR_STR			20
#define LEN_SENSOR_NAME_STR		20
#define LEN_FOV_STR				10
#define LEN_PATTERN_STR			10
#define LEN_CALB_NAME_STR		100
#define LEN_HW_VER_STR			20

/**
 * @struct _sensor_pre_state_t
 * sensor device pre operation lock state
 * @NO{S10E02C08}
 */
typedef enum _sensor_pre_state_t {
	SENSOR_PRE_STATE_LOCK = 0,
	SENSOR_PRE_STATE_UNLOCK,
} sensor_pre_state_t;

/**
 * @struct sensor_pre_state_e
 * sensor device pre operation state
 * @NO{S10E02C08}
 */
enum sensor_pre_state_e {
	SEN_PRE_STATE_DEFAULT = 0,
	SEN_PRE_STATE_INITING,
	SEN_PRE_STATE_INITED,
	SEN_PRE_STATE_STARTING,
	SEN_PRE_STATE_STARTED,
	SEN_PRE_STATE_MAX,
};

/**
 * @def SENSOR_PRE_STATE_NAMES
 * sensor pre state name strings
 */
#define SENSOR_PRE_STATE_NAMES { \
	"default", \
	"initing", \
	"inited", \
	"starting", \
	"started", \
}

/**
 * @struct sensor_ev_state_e
 * sensor device user event operation state
 * @NO{S10E02C08}
 */
enum sensor_ev_state_e {
	SEN_EV_STATE_DEFAULT = 0,
	SEN_EV_STATE_WAIT,
	SEN_EV_STATE_DOING_NOWAIT,
	SEN_EV_STATE_DOING,
	SEN_EV_STATE_DONE,
	SEN_EV_STATE_ERROR,
	SEN_EV_STATE_FINISH,
	SEN_EV_STATE_CANCEL,
	SEN_EV_STATE_MAX,
};

/**
 * @def SENSOR_EV_STATE_NAMES
 * sensor event op state name strings
 */
#define SENSOR_EV_STATE_NAMES { \
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
 * @struct sensor_event_type_e
 * sensor device user event type enum
 * @NO{S10E02C08}
 */
typedef enum sensor_event_type_e {
        SEN_EVENT_TYPE_INVALID = 0,
        SEN_EVENT_TYPE_STREAM,
        SEN_EVENT_TYPE_MAX,
} sensor_event_type_t;

/**
 * @struct sensor_event_info_s
 * sensor device event info struct
 * @NO{S10E02C08}
 */
typedef struct sensor_event_info_s {
        int32_t type;
        int32_t data;
        int32_t reserved[2];
} sensor_event_info_t;

/**
 * @struct sensor_input_param_s
 * sensor device input info param struct
 * @NO{S10E02C08}
 */
typedef struct sensor_input_param_s {
	int32_t  ts_compensate;
	int32_t  reserved[3];
} sensor_input_param_t;

/**
 * @struct sensor_ae_info_s
 * camera device ae info struct
 */
typedef struct sensor_ae_info_s {
	uint32_t line;
	uint32_t again;
	uint32_t dgain;
} sensor_ae_info_t;

/**
 * @struct sensor_parameter_s
 * sensor device basic param struct
 */
typedef struct sensor_parameter_s {
	uint32_t frame_length;
	uint32_t line_length;
	uint32_t width;
	uint32_t height;
	float    fps;
	uint32_t pclk;
	uint32_t exp_num;
	uint32_t lines_per_second;
	char     version[10];
	char     vendor[LEN_VENDOR_STR];
	char     sensor_name[LEN_SENSOR_NAME_STR];
	char     fov[LEN_FOV_STR];
	char     bayer_pattern[LEN_PATTERN_STR];
	char     calb_name[LEN_CALB_NAME_STR];
	char     sensor_hw_version[LEN_HW_VER_STR];  // reserved
	char     reserved[64];
} sensor_parameter_t;

/**
 * @struct sensor_intrinsic_parameter_s
 * sensor device intrinasic param struct
 */
typedef struct sensor_intrinsic_parameter_s {
	uint8_t major_version;
	uint8_t minor_version;
	uint16_t vendor_id;
	uint16_t module_id;
	uint32_t module_serial;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t cam_type;
	uint8_t module_falg;
	uint8_t efl_flag;
	uint8_t cod_flag;
	uint8_t pp_flag;
	uint8_t distortion_flag;
	uint16_t image_height;
	uint16_t image_width;
	uint32_t crc32_1;
	uint32_t crc_group1;
	uint8_t distort_params;
	uint8_t distort_model_type;
	uint8_t serial_num[40];
	double pp_x;
	double pp_y;
	double cam_skew;
	double focal_u;
	double focal_v;
	double center_u;
	double center_v;
	double hfov;
	double k1;
	double k2;
	double p1;
	double p2;
	double k3;
	double k4;
	double k5;
	double k6;
	double k7;
	double k8;
	double k9;
	double k10;
	double k11;
	double k12;
	double k13;
	double k14;
	double focal_u_2;
	double focal_v_2;
	double center_u_2;
	double center_v_2;
	double k1_2;
	double k2_2;
	double k3_2;
	double k4_2;
} sensor_intrinsic_parameter_t;

/**
 * @struct cam_parameter_s
 * sensor device param struct
 * @NO{S10E02C08}
 */
typedef struct cam_parameter_s {
	sensor_parameter_t sns_param;
	sensor_intrinsic_parameter_t sns_intrinsic_param;
} cam_parameter_t;

/**
 * @struct cam_usr_info_s
 * sensor device param struct for user
 * @NO{S10E02C08}
 */
typedef struct cam_usr_info_s {
	int32_t state;
	cam_parameter_t iparam;
} cam_usr_info_t;

/**
 * @struct sensor_user_s
 * sensor device user struct for operation
 * @NO{S10E02C08}
 */
struct sensor_user_s {
	osal_mutex_t open_mutex;
	osal_mutex_t mutex;
	uint32_t open_cnt;
	uint32_t init_cnt;
	uint32_t start_cnt;
	uint32_t data_init;
	uint32_t pre_state;
	uint32_t ev_state;
	uint32_t ev_wait;
	uint32_t stop_wait;
	uint32_t devflag;
	bool pre_done;
	osal_waitqueue_t pre_wq;
	osal_waitqueue_t evu_wq;
	osal_waitqueue_t evk_wq;
	osal_waitqueue_t stop_wq;
	osal_waitqueue_t usr_info_wq;
	sensor_event_info_t *ev_info;
	sensor_event_info_t ev_nowait;
};

/**
 * @struct sensor_miscdev_s
 * sensor misc device struct for hw operation
 * @NO{S10E02C08}
 */
struct sensor_miscdev_s {
	char name[SENSOR_DEV_NAME_LEN];
	// int32_t minor_id;
	int32_t bus_num;
	int32_t addr;
	/* uint32_t devflag; */
	/* uint32_t writing_flag; */
	/* uint32_t reading_flag; */
	/* uint32_t stop_flag; */
	uint32_t dummy_sensor;
	osal_mutex_t bus_mutex;
	// struct miscdevice miscdev;
	// struct i2c_client *client;
	// struct i2c_board_info board_info;
	void *client;
	char *client_name;
};

/**
 * @struct sensor_link_s
 * sensor device link struct for operation
 * @NO{S10E02C08}
 */
struct sensor_link_s {
	int32_t attach;
	int32_t flow_id;
	int32_t ctx_id;
};

/**
 * @struct sensor_param_s
 * sensor device param struct for debug
 * @NO{S10E02C08}
 */
struct sensor_param_s {
	/* must be int32_t */
	int32_t i2c_debug;
	int32_t frame_debug;
	int32_t stop_wait_ms;
	int32_t ctrl_mode;
	int32_t ae_share_flag;
	int32_t ae_event_flag;
	int32_t ts_compensate;
	int32_t iparam_timeout_ms;
	int32_t ctrl_timeout_ms;
	int32_t ev_timeout_ms;
	int32_t ev_retry_max;
};

/**
 * @def SENSOR_PARAM_NAMES
 * the sensor param name string array
 */
#define SENSOR_PARAM_NAMES { \
	"i2c_debug", \
	"frame_debug", \
	"stop_wait_ms", \
	"ctrl_mode", \
	"ae_share_flag", \
	"ae_event_flag", \
	"ts_compensate", \
	"iparam_timeout_ms", \
	"ctrl_timeout_ms", \
	"ev_timeout_ms", \
	"ev_retry_max", \
}

/**
 * @def SENSOR_PARAM_OPEN_RESET_MASK
 * the sensor param mask which should reset when open first:
 *  ctrl_mode, ts_compensate
 */
#define SENSOR_PARAM_OPEN_RESET_MASK	(0x48)

#define SENSOR_FRAME_DEBUG_TIME_EVENT	(0x1)
#define SENSOR_FRAME_DEBUG_TIME_2A	(0x2)
#define SENSOR_FRAME_DEBUG_FPS_RECORD	(0x4)
#define SENSOR_FRAME_DEBUG_FPS_RECMIS	(0x8)
#define SENSOR_FRAME_DEBUG_CFG_SHOW	(0x10)
#define SENSOR_FRAME_DEBUG_ALL_DISABLE	(0x0)
#define SENSOR_FRAME_DEBUG_ALL_ENABLE	(0x1F)
#define SENSOR_FRAME_DEBUG_ALL_RECORD	(0xF)

/**
 * @enumct sensor_ctrl_mode_e
 * sensor device ctrl mode enum
 * @NO{S10E02C08}
 */
typedef enum sensor_ctrl_mode_e {
	SENSOR_CTRLM_AUTO,
	SENSOR_CTRLM_USER,
	SENSOR_CTRLM_DRIVER,
	SENSOR_CTRLM_INVALID,
} sensor_ctrl_mode_t;

/**
 * @def SENSOR_CTRLM_NAMES
 * the sensor ctrl mode name string array
 */
#define SENSOR_CTRLM_NAMES { \
	"auto", \
	"user", \
	"driver", \
	"invalid", \
}

#define SENSOR_PARAM_FRAME_DEBUG_DEFAULT	(SENSOR_FRAME_DEBUG_ALL_RECORD)
#define SENSOR_PARAM_STOP_WAIT_MS_DEFAULT	(100)
#define SENSOR_PARAM_CATL_MODE_DEFAULT		(SENSOR_CTRLM_AUTO)
#define SENSOR_PARAM_AE_EVENT_FLAG_DEFAULT	(SENSOR_FRAME_START)
#define SENSOR_PARAM_CTRL_TIMEOOUT_MS_DEFAULT	(200)
#define SENSOR_PARAM_EV_TIMEOOUT_MS_DEFAULT	(-1) // (500)
#define SENSOR_PARAM_EV_RETRY_MAX_DEFAULT	(3)

/**
 * @struct sensor_frame_record_s
 * sensor device frame record struct for debug
 * @NO{S10E02C08}
 */
struct sensor_frame_record_s {
	uint32_t warn;
	uint32_t count;
	uint32_t count_max;
	uint32_t count_min;
	uint32_t diff_ns_min;
	uint32_t diff_ns_max;
	uint64_t diff_ns_all;
	uint64_t ts_ns_last;
};

/**
 * @enumct sensor_frame_2a_step_e
 * sensor device frame 2a operation step enum
 * @NO{S10E02C08}
 */
typedef enum sensor_frame_2a_step_e {
	SENSOR_F2AS_UPDATE,
	SENSOR_F2AS_TRIGGER,
	SENSOR_F2AS_DONE,
	SENSOR_F2AS_UPDATE_WARN,
	SENSOR_F2AS_TRIGGER_WARN,
	SENSOR_F2AS_DONE_WARN,
	SENSOR_F2AS_INVALID,
} sensor_frame_2a_step_t;

/**
 * @struct sensor_frame_s
 * sensor device frame info struct for debug
 * @NO{S10E02C08}
 */
struct sensor_frame_s {
	struct sensor_frame_record_s fs;
	struct sensor_frame_record_s fe;
	struct sensor_frame_record_s fs_fe;
	struct sensor_frame_record_s update_2a;
	struct sensor_frame_record_s start_2a;
	struct sensor_frame_record_s trig_2a;
	struct sensor_frame_record_s done_2a;
	uint32_t event;
};

/**
 * @enum sensor_frame_type_e
 * sensor device frame type enum
 * @NO{S10E02C08}
 */
typedef enum sensor_frame_type_e {
	SENSOR_FTYPE_FS,
	SENSOR_FTYPE_FE,
	SENSOR_FTYPE_FDONE,
	SENSOR_FTYPE_2A_UPD,
	SENSOR_FTYPE_2A_START,
	SENSOR_FTYPE_2A_DONE,
	SENSOR_FTYPE_2A_TRIG,
	SENSOR_FTYPE_MAX,
} sensor_frame_type_t;

#define SENSOR_FPS_RECORD_MAX	(2)
#define SENSOR_FPS_RECMIS_MAX	(100)

/**
 * @struct sensor_fps_record_s
 * sensor device fps record struct for debug
 * @NO{S10E02C08}
 */
struct sensor_fps_record_s {
	uint32_t count;
	uint32_t start;
	uint64_t ts_s;
};

/**
 * @struct sensor_frame_info_s
 * sensor device frame info struct for debug
 * @NO{S10E02C08}
 */
struct sensor_fps_s {
	uint32_t fps_target;
	uint32_t rec_index;
	uint32_t mis_index;
	uint32_t mis_count;
	uint64_t ts_ns_first;
	uint64_t ts_ns_last;
	struct sensor_fps_record_s record[SENSOR_FPS_RECORD_MAX];
	struct sensor_fps_record_s recmis[SENSOR_FPS_RECMIS_MAX];
};

/**
 * @struct sensor_device_s
 * sensor device struct
 * @NO{S10E02C08}
 */
struct sensor_device_s {
	int32_t port;
	struct os_dev osdev;
	struct sensor_user_s user;
	struct sensor_miscdev_s mdev;
	struct sensor_link_s link;
	struct sensor_frame_s frame;
	struct sensor_fps_s fps;
	struct sensor_param_s param;
	struct cam_usr_info_s user_info;
	struct sensor_tuning_data_s camera_param;
};

/**
 * @struct sensor_ctrl_ops
 * camera sensor gain operation functions
 * @NO{S10E02C08}
 */
struct sensor_ctrl_ops {
	char ctrl_name[20];
	void (*camera_gain_control)(uint32_t port, sensor_priv_t *priv_param,
			uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line);
	void (*camera_line_control)(uint32_t port, sensor_priv_t *priv_param,
			uint32_t *a_line);
	void (*camera_alloc_again)(uint32_t port, uint32_t *a_gain);
	void (*camera_alloc_dgain)(uint32_t port, uint32_t *a_gain);
};

/**
 * @struct _x3_camera_i2c_t
 * camera sensor i2c operation struct
 * @NO{S10E02C08}
 */
typedef struct _x3_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x3_camera_i2c_t;

/* internal apis */
extern void camera_dev_param_init(struct sensor_device_s *sen);
extern int32_t camera_dev_open(struct sensor_device_s *sen);
extern int32_t camera_dev_release(struct sensor_device_s *sen);

extern int32_t sensor_stream_reg_do(struct sensor_device_s *sen, int32_t on);
extern int32_t sensor_frame_end_stop_wait(struct sensor_device_s *sen);
extern uint32_t sensor_frame_count_get(struct sensor_device_s *sen, sensor_frame_type_t type);
extern void sensor_frame_event_record(struct sensor_device_s *sen, enum _sensor_frame_event_e event);
extern void sensor_frame_2a_record(struct sensor_device_s *sen, int32_t status);
extern int32_t sensor_frame_2a_check(struct sensor_device_s *sen, uint32_t id);
extern int32_t sensor_ctrl_mode_set(struct sensor_device_s *sen, int32_t ctrl_mode);
extern int32_t sensor_ctrl_mode_get(struct sensor_device_s *sen);

extern int32_t camera_tuning_update_param(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_set_ae_share(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_set_input_param(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_init_req(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_init_result(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_deinit_req(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_start(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_stop(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_event_get(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_event_put(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_update_ae_info(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_set_intrinsic_param(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_get_intrinsic_param(struct sensor_device_s *sen, unsigned long arg);
extern int32_t camera_isi_sensor_chn_number(void);
#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_DEV_H_
