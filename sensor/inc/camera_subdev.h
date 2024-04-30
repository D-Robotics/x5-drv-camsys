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
 * @file camera_subdev.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_SUBDEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_SUBDEV_H_

#include "inc/common_camera.h"

#ifdef CONFIG_HOBOT_SENSOR_NUM
#define CAMERA_TOTAL_NUMBER  	CONFIG_HOBOT_SENSOR_NUM
#else
#define CAMERA_TOTAL_NUMBER	(24)
#endif
#define V4L2_CAMERA_NAME "camera"
#define CAMERA_SENSOR_NAME  20u
#define FIRMWARE_CONTEXT_NUMBER 16u
#define HARDWARE_BUF_NUMBER 4u
#define SENSOR_WRITE_ENABLE 1u
#define SENSOR_WRITE_DISABLE 0u

#define I2C_BUS  0u
#define SPI_BUS  1u
#define SENSOR_ADDR_IGNORE 0xFFu

#define CALL_CISOPS(s, op, args...) (((s)->cis_ops->op) ? ((s)->cis_ops->op(args)) : 0)

/**
 * @struct sensor_priv
 * sensor private info struct
 * @NO{S10E02C08}
 */
typedef struct sensor_priv {
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
	uint32_t temper;
	uint8_t  mode;
	uint32_t id;
} sensor_priv_t;

/**
 * @struct sensor_data
 * sensor common info data struct
 * @NO{S10E02C08}
 */
typedef struct sensor_data {
	uint32_t  tuning_type;
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
	uint32_t  data_width;		// Bits per pixel.
	uint32_t  bayer_start;		// RGGB pattern start (R/Gr/Gb/B).
	uint32_t  bayer_pattern;	// CFA pattern type (RGGB/RCCC/RIrGB/RGIrB).
	uint32_t  exposure_max_bit_width; // pwl mode bits
}sensor_data_t;

enum led_type {
	INVALID,
	ON_OFF,
	LINEAR,
	LUT,
	LED_TYPE_MAX
};
/**
 * @struct sensor_led_s
 * sensor control operation arg
 * @NO{S10E02C08}
 */
typedef struct sensor_led_s {
	uint32_t type; //led type : 0 off/on ; 1 linear
	uint32_t pos_value;
	uint32_t neg_value;
	uint32_t reg_addr; // linear ratio
	uint32_t *led_lut;
} sensor_led_t;

/**
 * @struct sensor_arg
 * sensor control operation arg
 * @NO{S10E02C08}
 */
struct sensor_arg {
	uint32_t port;
	sensor_priv_t *sensor_priv;
	sensor_data_t *sensor_data;
	uint32_t *a_gain;
	uint32_t *d_gain;
	uint32_t  address;
	uint32_t  w_data;
	uint32_t  *r_data;
	uint32_t *integration_time;
};

/**
 * @struct ctrlp_s
 * sensor contrl line struct
 * @NO{S10E02C08}
 */
typedef struct ctrlp_s {
	int32_t ratio;
	uint32_t offset;
	uint32_t max;
	uint32_t min;
} ctrlp_t;

/**
 * @struct dol3_s
 * sensor dol3 control info struct
 * @NO{S10E02C08}
 */
typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

/**
 * @struct dol2_s
 * sensor dol2 control info struct
 * @NO{S10E02C08}
 */
typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;

/**
 * @struct normal_s
 * sensor normal(linear) control info struct
 * @NO{S10E02C08}
 */
typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;

/**
 * @struct pwl_s
 * sensor pwd(hdr) control info struct
 * @NO{S10E02C08}
 */
typedef struct pwl_s {
        uint32_t param_hold;
        uint32_t param_hold_length;
        uint32_t l_s_mode; //0 not use, 1 long; 2 short
        uint32_t line_num;
        ctrlp_t  line_p;
        ctrlp_t  line_p_ext[4];
        uint32_t line;
        uint32_t line_ext[4];
        uint32_t line_length;
        uint32_t line_length_ext[4];
        uint32_t again_control_num;
        uint32_t again_control[4];
        uint32_t again_control_length[4];
        uint32_t dgain_control_num;
        uint32_t dgain_control[4];
        uint32_t dgain_control_length[4];
        uint32_t *again_lut;
        uint32_t *dgain_lut;
}pwl_t;

/**
 * @struct stream_ctrl_s
 * sensor streaming operation register struct
 * @NO{S10E02C08}
 */
typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;


/**
 * @struct sensor_awb_ctrl_s
 * sensor awb control operation struct
 * @NO{S10E02C08}
 */
typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
	uint32_t apply_lut_gain;
} sensor_awb_ctrl_t;

/**
 * @struct sensor_tuning_data_s
 * sensro tuning data struct
 * @NO{S10E02C08}
 */
typedef struct sensor_tuning_data_s {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
	uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
	sensor_led_t led_info;
}sensor_tuning_data_t;

/**
 * @enumt camera_IOCTL
 * sensro ioctl for isp operaiton callback
 * @NO{S10E02C08}
 */
enum camera_IOCTL {
	SENSOR_INVALID = 0,
	SENSOR_UPDATE = 1,
	SENSOR_WRITE,
	SENSOR_READ,
	SENSOR_GET_PARAM,
	SENSOR_STREAM_ON,
	SENSOR_STREAM_OFF,
	SENSOR_ALLOC_ANALOG_GAIN,
	SENSOR_ALLOC_DIGITAL_GAIN,
	SENSOR_ALLOC_INTEGRATION_TIME,
	SENSOR_AWB_UPDATE
};

/**
 * @struct sensor_event_header_s
 * sensro event handle struct for control
 * @NO{S10E02C08}
 */
typedef struct sensor_event_header_s{
        osal_list_head_t list_free;
        osal_list_head_t list_busy;
        spinlock_t lock;
        struct work_struct updata_work;
	uint32_t ctx_id;
} sensor_event_header_t;

/**
 * @struct sensor_event_node_s
 * sensro event node struct for control
 * @NO{S10E02C08}
 */
typedef struct sensor_event_node_s{
        osal_list_head_t  list_node;
        uint32_t port;
        uint32_t cmd;
        sensor_priv_t priv_param;
} sensor_event_node_t;

extern struct sensor_isp_ops_s sensor_isp_ops;
extern struct sensor_cim_ops_s sensor_cim_ops;
extern struct sensor_isi_ops_s sensor_isi_ops;

int32_t camera_subdev_init(void);
void camera_subdev_exit(void);
int common_init(uint8_t chn, uint8_t mode);
void common_exit(uint8_t chn);
extern void wake_up_release_work(uint32_t pipeline);
#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_SENSOR_INC_CAMERA_SUBDEV_H_
