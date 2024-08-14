// SPDX-License-Identifier: GPL-2.0-only

#ifndef __V4L2_USR_API_H__
#define __V4L2_USR_API_H__

#include <linux/types.h>
#include <linux/videodev2.h>

#define V4L2_CID_DR_BASE        (V4L2_CID_USER_BASE + 0x11a0)
#define V4L2_CID_DR_EXPOSURE    (V4L2_CID_DR_BASE + 0x0001)
#define V4L2_CID_DR_AWB         (V4L2_CID_DR_BASE + 0x0002)

typedef struct isp_param_range {
    float min;
    float max;
} hbn_isp_param_range_t;

typedef enum enum_isp_mode {
    HBN_ISP_MODE_AUTO = 0,
    HBN_ISP_MODE_MANUAL,
    HBN_ISP_MODE_BUTT
} hbn_isp_mode_e;

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
    float speed_over;          // 暗到亮速度
    float speed_under;         // 亮到暗速度
    float tolerance;           // 偏差容忍度
    float target;              // 目标亮度值
    __u32 anti_flicker_status; // 抗频闪状态
    float flicker_freq;        // flicker 频率
    hbn_isp_auto_exposure_mode_e mode;
} hbn_isp_exposure_auto_attr_t;

typedef struct isp_exposure_manual_attr_s {
    float exp_time; // 使用s单位
    float again;
    float dgain;
    float ispgain;
    __u32 ae_exp;
    __u32 cur_lux; // 环境照度
} hbn_isp_exposure_manual_attr_t;

/* isp exposure attribute */
typedef struct hbn_isp_exposure_attr_s {
    hbn_isp_exposure_version_e version; //版本号
    hbn_isp_mode_e mode;
    hbn_isp_exposure_auto_attr_t auto_attr;
    hbn_isp_exposure_manual_attr_t manual_attr;
} hbn_isp_exposure_attr_t;

typedef enum enum_isp_awb_version {
    HBN_ISP_WB_V0 = 0,
    HBN_ISP_WB_V1,
} hbn_isp_awb_version_e;

typedef struct hbn_isp_awb_gain_s {
    float rgain;
    float grgain;
    float gbgain;
    float bgain;
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

/* isp awb attribute */
typedef struct hbn_isp_awb_attr_s {
    hbn_isp_awb_version_e version;
    hbn_isp_mode_e mode;
    hbn_isp_awb_auto_attr_t auto_attr;
    hbn_isp_awb_manual_attr_t manual_attr;
} hbn_isp_awb_attr_t;

#endif /* __V4L2_USR_API_H__ */
