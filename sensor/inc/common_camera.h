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
 * @file common_camera.h
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#ifndef COMMON_CAMERA_SUBDEV_H
#define COMMON_CAMERA_SUBDEV_H

#define FIRMWARE_CAMERA_NUMBER 16u

/**
 * @struct enum_bayer_rggb_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rggb_start_e {
	BAYER_RGGB_START_RGGB = 0,
	BAYER_START_R = 0,
	BAYER_RGGB_START_GRBG = 1,
	BAYER_START_GR = 1,
	BAYER_RGGB_START_GBRG = 2,
	BAYER_START_GB = 2,
	BAYER_RGGB_START_BGGR = 3,
	BAYER_START_B = 3,

	BAYER_RGGB_START_BUTT
} bayer_rggb_start_e;

/**
 * @struct enum_bayer_rccc_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rccc_start_e {
	BAYER_RCCC_START_RCCC = 0,
	BAYER_RCCC_START_CRCC = 1,
	BAYER_RCCC_START_CCRC = 2,
	BAYER_RCCC_START_CCCR = 3,

	BAYER_RCCC_START_BUTT
} bayer_rccc_start_e;

/**
 * @struct enum_bayer_rccb_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rccb_start_e {
	BAYER_RCCB_START_RCCB = 0,
	BAYER_RCCB_START_CRBC = 1,
	BAYER_RCCB_START_CBRC = 2,
	BAYER_RCCB_START_BCCR = 3,

	BAYER_RCCB_START_BUTT
} bayer_rccb_start_e;

/**
 * @struct enum_bayer_rccg_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rccg_start_e {
	BAYER_RCCG_START_RCCG = 0,
	BAYER_RCCG_START_CRGC = 1,
	BAYER_RCCG_START_CGRC = 2,
	BAYER_RCCG_START_GCCR = 3,

	BAYER_RCCG_START_BUTT
} bayer_rccg_start_e;

/**
 * @struct enum_bayer_rgbir_2x2_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rgbir_2x2_start_e {
	BAYER_RGBIR_2x2_START_RGBIR = 0,
	BAYER_RGBIR_2x2_START_GRIRB = 1,
	BAYER_RGBIR_2x2_START_BIRRG = 2,
	BAYER_RGBIR_2x2_START_IRBGR = 3,

	BAYER_RGBIR_2x2_START_BUTT
} bayer_rgbir_2x2_start_e;

/**
 * @struct enum_bayer_rgbir_4x4_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rgbir_4x4_statr_e {
	BAYER_RGBIR_4x4_START_GRIRG = 0,//BAYER_RGGB_START_GBRG
	BAYER_RGBIR_4x4_START_RGGIR = 1,//BAYER_RGGB_START_BGGR
	BAYER_RGBIR_4x4_START_GBIRG = 2,//BAYER_RGGB_START_GBRG
	BAYER_RGBIR_4x4_START_BGGIR = 3,//BAYER_RGGB_START_BGGR
	BAYER_RGBIR_4x4_START_IRGGB = 4,//BAYER_RGGB_START_RGGB
	BAYER_RGBIR_4x4_START_GIRBG = 5,//BAYER_RGGB_START_GRBG
	BAYER_RGBIR_4x4_START_IRGGR = 6,//BAYER_RGGB_START_RGGB
	BAYER_RGBIR_4x4_START_GIRRG = 7,//BAYER_RGGB_START_GRBG

	BAYER_RGBIR_4x4_START_BUTT
} bayer_rgbir_4x4_start_e;

const static uint32_t rgbir_4x4_to_rggb[8] = {BAYER_RGGB_START_GBRG,BAYER_RGGB_START_BGGR,
									BAYER_RGGB_START_GBRG,BAYER_RGGB_START_BGGR,
									BAYER_RGGB_START_RGGB,BAYER_RGGB_START_GRBG,
									BAYER_RGGB_START_RGGB,BAYER_RGGB_START_GRBG};

/**
 * @struct enum_bayer_ryycy_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_ryycy_start_e {
	BAYER_RYYCy_START_RYYCy = 0,
	BAYER_RYYCy_START_YRCyY = 1,
	BAYER_RYYCy_START_YCyRY = 2,
	BAYER_RYYCy_START_CyYYR = 3,

	BAYER_RYYCy_START_BUTT
} bayer_ryycy_start_e;

/**
 * @struct enum_bayer_ryyb_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_ryyb_start_e {
	BAYER_RYYB_START_RYYB = 0,
	BAYER_RYYB_START_YRBY = 1,
	BAYER_RYYB_START_YBRY = 2,
	BAYER_RYYB_START_BYYR = 3,

	BAYER_RYYB_START_BUTT
} bayer_ryyb_start_e;

/**
 * @struct enum_bayer_rgbw_start_e
 * sensor bayer pattern start type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_rgbw_start_e {
	BAYER_RGBW_START_RGBW = 0,
	BAYER_RGBW_START_GRWB = 1,
	BAYER_RGBW_START_BWRG = 2,
	BAYER_RGBW_START_WBGR = 3,

	BAYER_RGBW_START_BUTT
} bayer_rgbw_start_e;

typedef union union_bayer_start_u {
	bayer_rggb_start_e rggb;
	bayer_rccc_start_e rccc;
	bayer_rccb_start_e rccb;
	bayer_rccg_start_e rccg;
	bayer_rgbir_2x2_start_e rgbir_2x2;
	bayer_rgbir_4x4_start_e rgbir_4x4;
	bayer_ryycy_start_e ryycy;
	bayer_ryyb_start_e ryyb;
	bayer_rgbw_start_e rgbw;
} bayer_start_u;

/**
 * @struct enum_bayer_pattern_e
 * sensor bayer pattern type
 * @NO{S10E02C08}
 */
typedef enum enum_bayer_pattern_e {
	BAYER_PATTERN_RGGB = 0,
	BAYER_PATTERN_RCCC = 1,
	BAYER_PATTERN_RCCB = 2,
	BAYER_PATTERN_RCCG = 3,
	BAYER_PATTERN_CCCC = 4,
	BAYER_PATTERN_RGBIR_2X2 = 5,
	BAYER_PATTERN_GRBIR_4X4 = 6,
	BAYER_PATTERN_RYYCy = 7,
	BAYER_PATTERN_RYYB = 8,
	BAYER_PATTERN_RGBW = 9,

	BAYER_PATTERN_BUTT
} bayer_pattern_e;

/**
 * @struct _setting_param_t
 * sensor params struct which transfer to isp by callback
 * @NO{S10E02C08}
 */
struct _setting_param_t {
    uint32_t lines_per_second;
    uint32_t analog_gain_max;
    uint32_t digital_gain_max;
    uint32_t exposure_time_max;
    uint32_t exposure_time_min;
    uint32_t exposure_time_limit;
	uint32_t exposure_time_long_max;
    uint16_t active_width;
    uint16_t active_height;
	uint32_t fps;
	uint8_t data_width;				// Bits per pixel.
	bayer_start_u bayer_start;		// RGGB pattern start (R/Gr/Gb/B).
	bayer_pattern_e bayer_pattern;	// CFA pattern type (RGGB/RCCC/RIrGB/RGIrB).
	uint8_t exposure_max_bit_width;
};

struct sensor_priv_old {
	uint32_t gain_buf[4];
	uint32_t dgain_buf[4];
	uint32_t line_buf[4];
};


/**
 * @enum enum_sensor_frame_event_e
 * sensor frame event type for cim cops
 * @NO{S10E02C08}
 */
typedef enum _sensor_frame_event_e {
	SENSOR_FRAME_START = 0,
	SENSOR_FRAME_END,

	SENSOR_FRAME_EVENT_MAX,
} sensor_frame_event_e;

/**
 * @struct sensor_cim_ops_s
 * sensor operation functions sturct for cim callback
 * @NO{S10E02C08}
 */
struct sensor_cim_ops_s {
	int32_t (* sensor_get_para)(uint32_t chn, struct _setting_param_t *user_para);
	void (*sensor_frame_event)(int32_t flow_id, enum _sensor_frame_event_e event);
	int32_t (*sensor_get_ts_compensate)(int32_t flow_id);
	void *reserved_func[4];
	uint32_t end_magic;
};

#define SENSOR_COPS_MAGIC		(0x4053656E)
#define SENSOR_OPS_END_MAGIC		(0x2653656E)
#define SENSOR_OPS_CHECK(p)		(((p) != NULL) && ((p)->end_magic == SENSOR_OPS_END_MAGIC))
#define SENSOR_OPS_CHECK_STRICT(p)	(SENSOR_OPERATIONS_CHECK(p) && ((p)->reserved_func[0] == NULL))

/**
 * @struct SENSOR_MODE
 * sensor mode as linear,dol2,dol3,dol4,pwl
 * @NO{S10E02C08}
 */
enum SENSOR_MODE {
	SENSOR_LINEAR = 1,
	SENSOR_DOL2,
	SENSOR_DOL3,
	SENSOR_DOL4,
	SENSOR_PWL,
};


struct isi_sensor_base_info_s {
        char sensor_name[20u];
        uint32_t  chn;
        uint32_t  sensor_addr;
        uint32_t  bus_num;
        uint32_t  bus_type;
        uint32_t  reg_width;
        uint32_t  mode;
};

struct isi_sensor_again_info_s {
        uint32_t again_num;
        uint32_t again_buf[4];
};

struct isi_sensor_dgain_info_s {
        uint32_t dgain_num;
        uint32_t dgain_buf[4];
};

struct isi_sensor_line_info_s {
        uint32_t line_num;
        uint32_t line_buf[4];
};

struct isi_sensor_awb_info_s {
        uint32_t rgain;
        uint32_t bgain;
        uint32_t grgain;
        uint32_t gbgain;
        uint32_t temper;
};

/**
 * @struct sensor_isi_ops_s
 * sensor operation functions sturct for isp callback
 */
struct sensor_isi_ops_s {
        int32_t (* sensor_alloc_analog_gain) (uint32_t chn, int32_t *gain_ptr, uint32_t gain_num);
        int32_t (* sensor_alloc_digital_gain) (uint32_t chn, int32_t *gain_ptr, uint32_t gain_num);
        int32_t (* sensor_alloc_integration_time) (uint32_t chn, uint32_t *int_time, uint32_t *int_time_M, uint32_t *int_time_L );
        int32_t (* sensor_get_analog_gain) (uint32_t chn, struct isi_sensor_again_info_s *user_again);
        int32_t (* sensor_get_digital_gain) (uint32_t chn, struct isi_sensor_dgain_info_s *user_dgain);
        int32_t (* sensor_get_integration_time) (uint32_t chn, struct isi_sensor_line_info_s *user_line);
        int32_t (* sensor_update)  (uint32_t chn, int32_t effect);
        uint32_t (* read_register) (uint32_t chn, uint32_t address);
        int32_t (* write_register) (uint32_t chn, uint32_t address, uint32_t data);
        int32_t (* sensor_get_para) (uint32_t chn, struct _setting_param_t *user_para);
        int32_t (* sensor_get_base_info) (uint32_t chn, struct isi_sensor_base_info_s *user_para);
        int32_t (* sensor_awb_para) (uint32_t chn, uint32_t rgain, uint32_t bgain,
                        uint32_t grgain, uint32_t gbgain, uint32_t temper);
        int32_t (* sensor_get_awb_para) (uint32_t chn, struct isi_sensor_awb_info_s *user_awb);
        void *reserved_func[7];
        uint32_t end_magic;
};

/**
 * @struct sensor_isp_ops_s
 * sensor operation functions sturct for isp callback
 * @NO{S10E02C08}
 */
struct sensor_isp_ops_s {
	int32_t (* sensor_alloc_analog_gain) (uint32_t chn, int32_t *gain_ptr);
	int32_t (* sensor_alloc_digital_gain) (uint32_t chn, int32_t *gain_ptr);
	int32_t (* sensor_alloc_integration_time) (uint32_t chn, uint32_t *line_ptr);
	int32_t (* sensor_update) (uint32_t chn, struct sensor_priv_old *updata, int32_t effect);
	uint32_t (* read_register) (uint32_t chn, uint32_t address);
	int32_t (* write_register) (uint32_t chn, uint32_t address, uint32_t data);
	int32_t (* sensor_get_para)(uint32_t chn, struct _setting_param_t *user_para);
        int32_t (* sensor_get_base_info) (uint32_t chn, struct isi_sensor_base_info_s *user_para);
	int32_t (* sensor_awb_para)(uint32_t chn, uint32_t rgain, uint32_t bgain,
			uint32_t grgain, uint32_t gbgain, uint32_t temper);
	int32_t (* sensor_set_led) (uint32_t chn, int32_t brightness);
	void *reserved_func[6];
	uint32_t end_magic;
};

#endif // COMMON_CAMERA_SUBDEV_H

