/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _HBN_ISP_API_H_
#define _HBN_ISP_API_H_
#include <linux/types.h>

typedef enum isp_ctrl_id {
	ctrl_id_module_ctrl,
	ctrl_id_exposure_attr,
	ctrl_id_hdr_exposure_attr,
	ctrl_id_awb_attr,
	ctrl_id_awb_get_gain_by_temper,
	ctrl_id_color_process_attr,
	ctrl_id_ae_zone_weight_attr,
	ctrl_id_af_zone_weight_attr,
	ctrl_id_ae_statistics,
	ctrl_id_awb_statistics,
	ctrl_id_af_statistics,
	ctrl_id_ae_exposure_table,
	ctrl_id_sensor_param,
} isp_ctrl_id_e;

/* common define */
typedef enum enum_isp_mode {
	HBN_ISP_MODE_AUTO = 0,
	HBN_ISP_MODE_MANUAL,
	HBN_ISP_MODE_BUTT
} hbn_isp_mode_e;

typedef struct isp_param_range {
	__u32 min;
	__u32 max;
} hbn_isp_param_range_t;

typedef struct isp_zone_weight_s {
	__u32 x;
	__u32 y;
	__u32 h;
	__u32 w;
	__u32 weight;
} hbn_isp_zone_weight_t;

/* module control */
typedef enum enum_isp_module_version {
	HBN_ISP_MODULE_V0 = 0,
	HBN_ISP_MODULE_V1,
	HBN_ISP_MODULE_BUTT
} hbn_isp_module_version_e;

typedef union tag_isp_module_ctrl_u {
	__u32 u32Key;
	struct {
		__u32 bitWdrBlc : 1; /* RW;[0] */
		__u32 bitWdrDgain : 1; /* RW;[1] */
		__u32 bitWdrWbGain : 1; /* RW;[2] */
		__u32 bitFrameStitch : 1; /* RW;[3] */
		__u32 bitDecompare : 1; /* RW;[4] */
		__u32 bitWb : 1; /* RW;[5] */
		__u32 bitIspDgain : 1; /* RW;[6] */
		__u32 bitBlc : 1; /* RW;[7] */
		__u32 bitGe : 1; /* RW;[8] */
		__u32 bitDdpc : 1; /* RW;[9] */
		__u32 bitSdpc : 1; /* RW;[10] */
		__u32 bitMeshShading: 1; /* RW;[11] */
		__u32 bitIsp2Dnr : 1; /* RW;[12] */
		__u32 bitIsp3Dnr : 1; /* RW;[13] */
		__u32 bitWdr : 1; /* RW;[14] */
		__u32 bitDemosaic : 1; /* RW;[15] */
		__u32 bitCac : 1; /* RW;[16] */
		__u32 bitDepurle : 1; /* RW;[17] */
		__u32 bitCcm : 1; /* RW;[18] */
		__u32 bitGamma : 1; /* RW;[19] */
		__u32 bitCa : 1; /* RW;[20] */
		__u32 bitCnr : 1; /* RW;[21] */
		__u32 bitEe : 1; /* RW;[22] */
		__u32 bitCsc : 1; /* RW;[23] */
		__u32 bitColorProcess : 1; /* RW;[24] */
		__u32 bitRgbir : 1; /* RW;[25] */
		__u32 bitRadialShading : 1; /* RW;[26] */
		__u32 bitYnr : 1; /* RW;[27] */
	};
} isp_module_ctrl_u;

typedef struct isp_module_ctrl_s {
	hbn_isp_module_version_e version;
	isp_module_ctrl_u module;
} hbn_isp_module_ctrl_t;

/* exposure attribute */
typedef enum enum_isp_exposure_version {
	HBN_ISP_EXP_V0 = 0,
	HBN_ISP_EXP_V1,
	HBN_ISP_EXP_V2,
} hbn_isp_exposure_version_e;

typedef enum enum_isp_auto_exposure_mode {
	HBN_ISP_AUTO_EXP_MODE_ADAPTIVE = 0,
	HBN_ISP_AUTO_EXP_MODE_FIX,
} hbn_isp_auto_exposure_mode_e;

typedef struct isp_exposure_auto_attr_s {
	hbn_isp_param_range_t exp_time_range; // 使用s单位
	hbn_isp_param_range_t again_range;
	hbn_isp_param_range_t dgain_range;
	hbn_isp_param_range_t isp_dgain_range;
	__u32 speed_over;          // 暗到亮速度
	__u32 speed_under;         // 亮到暗速度
	__u32 tolerance;           // 偏差容忍度
	__u32 target;              // 目标亮度值
	__u32 anti_flicker_status; // 抗频闪状态
	__u32 flicker_freq;        // flicker 频率
	hbn_isp_auto_exposure_mode_e mode;
} hbn_isp_exposure_auto_attr_t;

typedef struct isp_exposure_manual_attr_s {
	__u32 exp_time; // 使用s单位
	__u32 again;
	__u32 dgain;
	__u32 ispgain;
	__u32 ae_exp;
	__u32 cur_lux; // 环境照度
} hbn_isp_exposure_manual_attr_t;

typedef struct hbn_isp_exposure_attr_s {
	hbn_isp_exposure_version_e version; //版本号
	hbn_isp_mode_e mode;
	hbn_isp_exposure_auto_attr_t auto_attr;
	hbn_isp_exposure_manual_attr_t manual_attr;
} hbn_isp_exposure_attr_t;

/* HDR exposure attribute */
typedef enum enum_isp_hdr_exposure_version {
	HBN_ISP_HDR_EXP_V0 = 0,
	HBN_ISP_HDR_EXP_V1,
	HBN_ISP_HDR_EXP_V2,
} hbn_isp_hdr_exposure_version_e;

typedef struct isp_hdr_exposure_auto_attr_s {
	__u32 exp_ratio[4];
	hbn_isp_param_range_t exp_ratio_range;
} hbn_isp_hdr_exposure_auto_attr_t;

typedef struct isp_hdr_exposure_manual_attr_s {
	__u32 exp_time; // 使用s单位
	__u32 again;
	__u32 dgain;
	__u32 ispgain;
	__u32 ae_exp;
} hbn_isp_hdr_exposure_manual_attr_t;

typedef struct isp_hdr_exposure_attr_s {
	hbn_isp_hdr_exposure_version_e version;
	hbn_isp_mode_e mode;
	hbn_isp_hdr_exposure_auto_attr_t auto_attr;
	hbn_isp_hdr_exposure_manual_attr_t manual_attr;
} hbn_isp_hdr_exposure_attr_t;

/* AWB attribute */
typedef enum enum_isp_awb_version {
	HBN_ISP_WB_V0 = 0,
	HBN_ISP_WB_V1,
} hbn_isp_awb_version_e;

/* AWB gain parameter */
typedef struct hbn_isp_awb_gain_s {
	__u32 rgain;
	__u32 grgain;
	__u32 gbgain;
	__u32 bgain;
} hbn_isp_awb_gain_t;

typedef struct hbn_isp_awb_auto_attr_s {
	__u32 speed;
	__u32 tolerance;
	__u32 static_wb[4]; // 归一化参数
	__u32 rg_strength;  // r通道强度（设置白平衡偏好r通道）
	__u32 bg_strength;  // b通道强度（设置白平衡偏好b通道）
	hbn_isp_awb_gain_t gain;
	__u32 temper;
} hbn_isp_awb_auto_attr_t;

typedef struct hbn_isp_awb_manual_attr_s {
	hbn_isp_awb_gain_t gain;
	__u32 temper;
} hbn_isp_awb_manual_attr_t;

typedef struct hbn_isp_awb_attr_s {
	hbn_isp_awb_version_e version;
	hbn_isp_mode_e mode;
	hbn_isp_awb_auto_attr_t auto_attr;
	hbn_isp_awb_manual_attr_t manual_attr;
} hbn_isp_awb_attr_t;

/* color process attribute */
typedef enum enum_isp_color_process_version {
	HBN_ISP_COLOR_PROCESS_V0 = 0,
	HBN_ISP_COLOR_PROCESS_V1,
} hbn_isp_color_process_version_e;

typedef struct hbn_isp_color_process_auto_attr_s {
	__u32 total_size;
	__u32 gain[20];
	__u32 bright[20];
	__u32 contrast[20];
	__u32 saturation[20];
	__u32 hue[20];
} hbn_isp_color_process_auto_attr_t;

typedef struct hbn_isp_color_process_manual_attr_s {
	__u32 total_size;
	__u32 bright;
	__u32 contrast;
	__u32 saturation;
	__u32 hue;
} hbn_isp_color_process_manual_attr_t;

typedef struct hbn_isp_color_process_attr_s {
	hbn_isp_color_process_version_e version;
	hbn_isp_mode_e mode;
	hbn_isp_color_process_auto_attr_t auto_attr;
	hbn_isp_color_process_manual_attr_t manual_attr;
} hbn_isp_color_process_attr_t;

/* AE 1024-zone weight */
typedef enum enum_isp_ae_zone_weight_version {
	HBN_ISP_AE_ZONE_WEIGHT_A = 0,
	HBN_ISP_AE_ZONE_WEIGHT_B,
} hbn_isp_ae_zone_weight_version_e;

#define HBN_ISP_GRID_ITEMS  (32 * 32)  /**< number of grid items */

typedef struct isp_ae_zone_weight_func_a_attr_s {
	__u32 total_size;
	hbn_isp_zone_weight_t weight[HBN_ISP_GRID_ITEMS];
} isp_ae_zone_weight_func_a_attr_t;

typedef struct hbn_isp_ae_zone_weight_attr_s {
	hbn_isp_ae_zone_weight_version_e version;
	union {
		isp_ae_zone_weight_func_a_attr_t func_a_attr;
	} attr;
} hbn_isp_ae_zone_weight_attr_t;

/* AF 225-zone weight */
typedef enum enum_isp_af_zone_weight_version_e {
	HBN_ISP_AF_ZONE_WEIGHT_A = 0,
	HBN_ISP_AF_ZONE_WEIGHT_B,
} hbn_isp_af_zone_weight_version_e;

typedef struct isp_af_zone_weight_func_a_attr_s {
	__u32 total_size;
	hbn_isp_zone_weight_t weight[15 * 15];
} isp_af_zone_weight_func_a_attr_t;

typedef struct hbn_isp_af_zone_weight_attr_s {
	hbn_isp_af_zone_weight_version_e version;
	union {
		isp_af_zone_weight_func_a_attr_t func_a_attr;
	} attr;
} hbn_isp_af_zone_weight_attr_t;

/* statistics */
#define HBN_ISP_PIXEL_CHANNEL 4   /**< number of pixel channel */
#define HBN_ISP_AFM_BLOCK_NUM 225

typedef struct hbn_isp_ae_statistics_s {
	__u32 expStat[HBN_ISP_GRID_ITEMS * HBN_ISP_PIXEL_CHANNEL];
	__u32 datatype;
	__u32 frame_id;
	__u64 timestamps;
} hbn_isp_ae_statistics_t;

typedef struct hbn_isp_awb_statistics_s {
	__u32 awbStat[HBN_ISP_GRID_ITEMS * HBN_ISP_PIXEL_CHANNEL];
	__u32 datatype;
	__u32 frame_id;
	__u64 timestamps;
} hbn_isp_awb_statistics_t;

typedef struct hbn_isp_af_statistics_s {
	__u32 sharpnessLowPass[HBN_ISP_AFM_BLOCK_NUM];
	__u32 sharpnessHighPass[HBN_ISP_AFM_BLOCK_NUM];
	__u32 histLowData[HBN_ISP_AFM_BLOCK_NUM];
	__u32 histHighData[HBN_ISP_AFM_BLOCK_NUM];
	__u32 frame_id;	// 当前对应frame id(备用)
} hbn_isp_af_statistics_t;

/* exposure table */
#define CAMDEV_AE_EXP_TABLE_NUM	8

typedef struct hbn_isp_table_s {
	__u32 exposure_time;/**< AE exposure time */
	__u32 again;        /**< AE simulated again */
	__u32 dgain;        /**< AE digital gain */
	__u32 isp_gain;     /**< AE isp gain */
} hbn_isp_table_t;

typedef struct hbn_isp_exposure_table_s {
	hbn_isp_table_t exp_table[CAMDEV_AE_EXP_TABLE_NUM]; /**< Exposure table */
	__u8 valid_num; /**< The valid number of exposure table */
} hbn_isp_exposure_table_t;

typedef struct hbn_isp_sensor_param_s {
	__u32 lines_per_second;
	__u32 again_max;
	__u32 dgain_max;
	__u32 exp_time_max;
	__u32 exp_time_min;
} hbn_isp_sensor_param_t;

#endif /* _HBN_ISP_API_H_ */
