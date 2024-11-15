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
	ctrl_id_exposure_roi,
	ctrl_id_2dnr_attr,
	ctrl_id_3dnr_attr,
} isp_ctrl_id_e;

/* common define */
#define HBN_ISP_AUTO_LEVEL_MAX 20

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

typedef struct hbn_windows_s {
	__u32 h_offset;		/**< Horizontal start offset */
	__u32 v_offset;		/**< Vertical start offset */
	__u32 width;		/**< Width */
	__u32 height;		/**< Height */
} hbn_windows_t;

typedef struct hbn_isp_roi_s {
	hbn_windows_t window;	/**< ROT window */
	__u32 weight;		/**< Weight */
} hbn_isp_roi_t;

/* module control */
typedef enum enum_isp_module_version {
	HBN_ISP_MODULE_V0 = 0,
	HBN_ISP_MODULE_V1,
	HBN_ISP_MODULE_BUTT
} hbn_isp_module_version_e;

typedef union tag_isp_module_ctrl_u {
	__u32 u32Key;
	struct {
		__u32 bit_ccm : 1; /* RW;[0] */
		__u32 bit_cnr : 1; /* RW;[1] */
		__u32 bit_cproc : 1; /* RW;[2] */
		__u32 bit_dg : 1; /* RW;[3] */
		__u32 bit_demosaic : 1; /* RW;[4] */
		__u32 bit_dpcc : 1; /* RW;[5] */
		__u32 bit_2dnr : 1; /* RW;[6] */
		__u32 bit_3dnr : 1; /* RW;[7] */
		__u32 bit_ee : 1; /* RW;[8] */
		__u32 bit_lsc : 1; /* RW;[9] */
		__u32 bit_lut3d : 1; /* RW;[10] */
		__u32 bit_wdr: 1; /* RW;[11] */
		__u32 bit_ynr : 1; /* RW;[12] */
		__u32 bit_ge : 1; /* RW;[13] */
		__u32 bit_wb : 1; /* RW;[14] */
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
	__u32 frame_id;
	__u64 timestamps;
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
#define HBN_ISP_EXP_TABLE_NUM	8

typedef struct hbn_isp_table_s {
	__u32 exposure_time;/**< AE exposure time */
	__u32 again;        /**< AE simulated again */
	__u32 dgain;        /**< AE digital gain */
	__u32 isp_gain;     /**< AE isp gain */
} hbn_isp_table_t;

typedef struct hbn_isp_exposure_table_s {
	hbn_isp_table_t exp_table[HBN_ISP_EXP_TABLE_NUM]; /**< Exposure table */
	__u8 valid_num; /**< The valid number of exposure table */
} hbn_isp_exposure_table_t;

typedef struct hbn_isp_sensor_param_s {
	__u32 lines_per_second;
	__u32 again_max;
	__u32 dgain_max;
	__u32 exp_time_max;
	__u32 exp_time_min;
} hbn_isp_sensor_param_t;

/* exposure roi */
#define HBN_ISP_ROI_WINDOWS_MAX 25

typedef struct hbn_isp_exposure_roi_s {
	__u32 roi_num;			/**< Number of ROI window */
	__u32 roi_weight;		/**< The weight of ROI; */
	hbn_isp_roi_t roi_window[HBN_ISP_ROI_WINDOWS_MAX];	/**< ROI windows */
} hbn_isp_exposure_roi_t;

/* 2dnr attr */
#define HBN_ISP_2DNR_CURVE_SIZE 12
#define HBN_ISP_2DNR_MOTION_SIZE 2
#define HBN_ISP_2DNR_STATIC_X_NUM 2
#define HBN_ISP_2DNR_STATIC_Y_NUM 3
#define HBN_ISP_2DNR_MOVING_X_NUM 2
#define HBN_ISP_2DNR_MOVING_Y_NUM 3
#define HBN_ISP_2DNR_SIGMA_NUM 3

typedef struct hbn_isp_2dnr_curve_s {
	__u16 ary_x[HBN_ISP_2DNR_CURVE_SIZE];	/**< Luma curve in X axis */
	__u16 ary_y[HBN_ISP_2DNR_CURVE_SIZE];	/**< Luma curve in Y axis */
	__u16 ary_px[HBN_ISP_2DNR_CURVE_SIZE];	/**< Luma curve of delta X */
	__u32 interp_mode;			/**< 2DNR internal mode */
} hbn_isp_2dnr_curve_t;

typedef struct hbn_isp_2dnr_motion_config_s {
	__u16 motion_anchor_x[HBN_ISP_2DNR_MOTION_SIZE];	/**< Motion anchor in X axis */
	hbn_isp_2dnr_curve_t curve_cfg;				/**< 2DNR curve configuration parameters */
} hbn_isp_2dnr_motion_config_t;

typedef struct hbn_isp_2dnr_manual_attr_s {
	__u32 blend_static;		/**< Weight of spatial NR result in final output for static pixels. */
	__u32 blend_motion;		/**< Weight of spatial NR result in final output for 100% moving pixels. */
	__u32 blend_slope;		/**< Merge slope. Larger values mean more NLM image weight. */
	__u32 vst_factor;		/**< VST factor */
	__u32 sigma_scale[HBN_ISP_2DNR_SIGMA_NUM];	/**< The scale of sigma */
	__u32 sigma_factor_mul[HBN_ISP_2DNR_SIGMA_NUM];	/**< Sigma factor multiplication */
	__u16 sigma_factor_motion_max;			/**< Maximum sigma factor motion */
	__u16 sigma_factor_motion_min;			/**< Minimum sigma factor motion */
	__u16 sigma_offset;				/**< Sigma square */
	__u16 static_detail_thresh[HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail threshold */
	__u16 static_detail_boost_thresh[HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail boost threshold */
	__u32 static_detail_boost[HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail boost */
	__u16 static_detail_clip_thresh[HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail clip threshold */
	__u16 moving_detail_thresh[HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail threshold */
	__u16 moving_detail_boost_thresh[HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail boost threshold */
	__u32 moving_detail_boost[HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail boost */
	__u16 moving_detail_clip_thresh[HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail clip threshold */
	__u32 static_factor[HBN_ISP_2DNR_SIGMA_NUM];	/**< Static factor */
	hbn_isp_2dnr_curve_t luma_curve_cfg;		/**< Luma curve configuration*/
	hbn_isp_2dnr_curve_t lsc_comp_curve_cfg;	/**< LSC comparison curve configuration*/
	hbn_isp_2dnr_motion_config_t motion_cfg;	/**< Motion configuration*/
} hbn_isp_2dnr_manual_attr_t;

typedef struct hbn_isp_2dnr_auto_attr_s {
	__u8 auto_level;			/**< The auto level */
	__u32 gain[HBN_ISP_AUTO_LEVEL_MAX];	/**< 2DNR gain */
	__u32 vst_factor[HBN_ISP_AUTO_LEVEL_MAX];	/**< VST factor */
	__u32 blend_static[HBN_ISP_AUTO_LEVEL_MAX];	/**< Weight of spatial NR result in final output for static pixels. */
	__u32 blend_motion[HBN_ISP_AUTO_LEVEL_MAX];	/**< Weight of spatial NR result in final output for 100% moving pixels. */
	__u32 blend_slope[HBN_ISP_AUTO_LEVEL_MAX];	/**< Merge slope. Larger values mean more NLM image weight. */
	__u16 sigma_offset[HBN_ISP_AUTO_LEVEL_MAX];	/**< Sigma offset */
	__u16 luma_curve_y[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_CURVE_SIZE];		/**< Luma curve in Y axis */
	__u16 lsc_comp_curve_y[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_CURVE_SIZE];	/**< LSC comparison curve in Y axis */
	__u16 motion_fac_curve_y[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_CURVE_SIZE];	/**< Motion factor curve in Y axis */
	__u16 motion_anchor_x[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_MOTION_SIZE];	/**< Motion anchor in X axis */
	__u16 static_detail_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail threshold */
	__u16 static_detail_boost_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail boost threshold */
	__u32 static_detail_boost[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail boost */
	__u16 static_detail_clip_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_STATIC_X_NUM][HBN_ISP_2DNR_STATIC_Y_NUM];	/**< Static detail clip threshold */
	__u16 moving_detail_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail threshold */
	__u16 moving_detail_boost_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail boost threshold */
	__u32 moving_detail_boost[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail boost */
	__u16 moving_detail_clip_thresh[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_MOVING_X_NUM][HBN_ISP_2DNR_MOVING_Y_NUM];	/**< Moving detail clip threshold */
	__u32 sigma_scale[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_SIGMA_NUM];		/**< The scale of sigma */
	__u32 static_factor[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_SIGMA_NUM];		/**< Static factor */
	__u32 sigma_factor_mul[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_2DNR_SIGMA_NUM];		/**< Sigma factor multiplication */
	__u16 sigma_factor_motion_max[HBN_ISP_AUTO_LEVEL_MAX];				/**< Maximum sigma factor motion */
} hbn_isp_2dnr_auto_attr_t;

typedef struct hbn_isp_2dnr_attr_s {
	hbn_isp_mode_e mode;			/**< The run mode: 0--manual, 1--auto */
	hbn_isp_2dnr_manual_attr_t manual_attr;	/**< 2DNR manual configuration*/
	hbn_isp_2dnr_auto_attr_t auto_attr;	/**< 2DNR auto configuration*/
} hbn_isp_2dnr_attr_t;

/* 3dnr attr */
#define HBN_ISP_3DNR_BLS_EXP_NUM 4		/**< The number of BLS explosure */
#define HBN_ISP_3DNR_THR_LUMA_CURVE_NUM 12	/**< The number of luma curve threshold */

typedef enum hbn_isp_3dnr_range_dilate_e {
	HBN_ISP_3DNR_RANGE_3 = 3,    /**< Dilate range:3 */
	HBN_ISP_3DNR_RANGE_4 = 6,    /**< Dilate range:6 */
} hbn_isp_3dnr_range_dilate_t;

typedef struct hbn_isp_3dnr_noise_model_s {
	__u8 input_bits;	/**< Input bits */
	__u16 fix_curve_start;	/**< Start to fix curve */
	__u64 noisemodel_a;	/**< Noise model A */
	__u64 noisemodel_b;	/**< Noise model B */
	__u32 bls_exp[HBN_ISP_3DNR_BLS_EXP_NUM];	/**< BLS exposure */
} hbn_isp_3dnr_noise_model_t;

typedef struct hbn_isp_3dnr_manual_attr_s {
	__u64 vst_factor;	/**< VST factor */
	__u8 tnr_strength;	/**< The threshold of strength */
	__u8 tnr_strength2;	/**< The threshold of strength2 */
	__u8 filter_len;	/**< IIR filter length of reference frame */
	__u8 filter_len2;	/**< IIR filter length of motion frame */
	__u64 motion_smooth_factor;	/**< Motion smooth factor */
	hbn_isp_3dnr_range_dilate_t range_h;	/**< Set motion detection window size in horizontal direction. */
	__u8 sad_weight;	/**< Set weight of motion difference(SAD+mean). */
	__u32 diff_type;	/**< The type of difference */
	__u8 sqr_diff_factor;	/**< Square difference factor */
	__u8 motion_smooth_lvl;	/**< Motion smooth level */
	hbn_isp_3dnr_range_dilate_t dilate_h;	/**< Set motion dilation window size in horizontal direction. */
	__u16 noise_level;	/**< Noise calibration data */
	__u16 thr_motion_slope;	/**< The threshold of gap between static and 100% moving. */
	__u16 tnr_luma_curve_x[HBN_ISP_3DNR_THR_LUMA_CURVE_NUM];	/**< The threshold of luma curve in X axis */
	__u16 tnr_luma_curve_y[HBN_ISP_3DNR_THR_LUMA_CURVE_NUM];	/**< The threshold of luma curve in Y axis */
	__u16 tnr_motion_slop_y[HBN_ISP_3DNR_THR_LUMA_CURVE_NUM];	/**< The threshold of motion slop in Y axis */
	hbn_isp_3dnr_noise_model_t noise_cfg;			/**< 3DNR noise model configuration */
} hbn_isp_3dnr_manual_attr_t;

typedef struct hbn_isp_3dnr_auto_attr_s {
	__u8 auto_level;	/**< 3DNR auto level */
	__u32 nm_k;		/**< Noise model K */
	__u32 nm_p;		/**< Noise model P */
	__u32 gains[HBN_ISP_AUTO_LEVEL_MAX];			/**<  3DNR gains */
	__u16 fix_curve_start[HBN_ISP_AUTO_LEVEL_MAX];	/**< Fix curve start */
	__u64 noisemodel_a[HBN_ISP_AUTO_LEVEL_MAX];		/**<Noise model A range:[0, 1000]*/
	__u64 noisemodel_b[HBN_ISP_AUTO_LEVEL_MAX];		/**<Noise model B range:[0, 1000]*/
	__u32 bls_exp[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_3DNR_BLS_EXP_NUM];	/**< BLS exposure */
	__u8 tnr_strength[HBN_ISP_AUTO_LEVEL_MAX];		/**< TNR strength */
	__u8 tnr_strength2[HBN_ISP_AUTO_LEVEL_MAX];		/**< TNR strength2  */
	__u8 filter_len[HBN_ISP_AUTO_LEVEL_MAX];		/**<[0, 1024]*/
	__u8 filter_len2[HBN_ISP_AUTO_LEVEL_MAX];		/**<[0, 1024]*/
	__u64 motion_smooth_factor[HBN_ISP_AUTO_LEVEL_MAX];	/**< Motion smooth factor */
	hbn_isp_3dnr_range_dilate_t range_h[HBN_ISP_AUTO_LEVEL_MAX];	/**< Set motion detection window size in horizontal direction. */
	__u8 sad_weight[HBN_ISP_AUTO_LEVEL_MAX];		/**< Set weight of motion difference(SAD+mean). */
	__u8 sqr_diff_factor[HBN_ISP_AUTO_LEVEL_MAX];	/**< Square difference factor */
	__u8 motion_smooth_lvl[HBN_ISP_AUTO_LEVEL_MAX];	/**< Motion smooth level */
	__u32 motion_dilate_en[HBN_ISP_AUTO_LEVEL_MAX];	/**< Motion dilate enable */
	hbn_isp_3dnr_range_dilate_t dilate_h[HBN_ISP_AUTO_LEVEL_MAX];		/**< Set motion dilation window size in horizontal direction. */
	__u16 tnr_luma_curve_y[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_3DNR_THR_LUMA_CURVE_NUM];	/**< Threshold of luma curve on Y axis */
	__u16 tnr_motion_slop_y[HBN_ISP_AUTO_LEVEL_MAX][HBN_ISP_3DNR_THR_LUMA_CURVE_NUM];	/**< Threshold of motion slop on Y axis */
} hbn_isp_3dnr_auto_attr_t;

typedef struct hbn_isp_3dnr_attr_s {
	hbn_isp_mode_e mode;			/**< The run mode: 0--manual, 1--auto */
	hbn_isp_3dnr_manual_attr_t manual_attr;	/**< 3DNR current configuration */
	hbn_isp_3dnr_auto_attr_t auto_attr;	/**< 3DNR SNR configuration */
} hbn_isp_3dnr_attr_t;

#endif /* _HBN_ISP_API_H_ */
