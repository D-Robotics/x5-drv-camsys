/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_ISP_H_
#define _UAPI_LINUX_ISP_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"
#include "mem_helper_uapi.h"

#define ISP_UID(n) cam_fourcc('i', 's', 'p', (n) + '0')

#define ISP_MCM_CLOCK (CAM_CLOCK_EXT_BASE + 1)

#define ISP_MSG_UNIT_TEST       (0x1 << 0)
#define ISP_MSG_GET_FUNC        (0x2 << 0)
#define ISP_MSG_GET_VI_INFO     (0x3 << 0)
#define ISP_MSG_GET_FRAME_INFO  (0x4 << 0)
#define ISP_MSG_GET_METADATA    (0x5 << 0)
#define ISP_MSG_QRY_METADATA    (0x6 << 0)
#define ISP_MSG_RESET_SCH       (0x7 << 0)
#define ISP_MSG_SET_GAMMA_FE_BE (0x8 << 0)
#define ISP_MSG_SET_RGBGAMMA    (0x9 << 0)
#define ISP_MSG_SET_WDR5_HIST       (0xa << 0)
#define ISP_MSG_SET_WDR5_SHIFT      (0xb << 0)
#define ISP_MSG_SET_WDR5_SHIFT0     (0xc << 0)
#define ISP_MSG_SET_WDR5_GAMMAPRE   (0xd << 0)
#define ISP_MSG_SET_WDR5_GAMMADOWN  (0xe << 0)
#define ISP_MSG_SET_WDR5_ENTROPY    (0xf << 0)
#define ISP_MSG_SET_WDR5_DISTANCE   (0x10 << 0)
#define ISP_MSG_SET_WDR5_DIFFERENCE (0x11 << 0)
#define ISP_MSG_SET_WDR5_FACTOR     (0x12 << 0)
#define ISP_MSG_SET_WDR5_LEVEL      (0x13 << 0)
#define ISP_MSG_SET_WDR5_SAT_SHIFT  (0x14 << 0)
#define ISP_MSG_ACK_CTRL            (0x15 << 0)
#define ISP_MSG_ACK_CTRL_EXT        (0x16 << 0)

#define ISP_MSG_IRQ_MIS    (0x1 << 8)
#define ISP_MSG_MCM_SCH    (0x2 << 8)
#define ISP_MSG_TUNE_EN    (0x3 << 8)
#define ISP_MSG_FRAME_DONE (0x4 << 8)

#define ISP_CTRL_DATA_LENGTH (128)
#define ISP_CTRL_FEBE_NUM    (129)

struct isp_format {
	struct cam_format ifmt;
	struct cam_format ofmt;
	struct cam_rect icrop;
};

enum isp_irq_num {
	ISP_IRQ_MIS = 0,
	MI_IRQ_MIS,
	FE_IRQ_MIS,
};

struct mi_mis_group {
	__u32 miv2_mis;
	__u32 miv2_mis1;
	__u32 miv2_mis2;
	__u32 miv2_mis3;
	__u32 mi_mis_hdr1;
};

union isp_irq_stat {
	__u32 isp_mis;
	struct mi_mis_group mi_mis;
	__u32 fe_mis;
};

struct isp_ctrl {
	__u32 ctrl_id;
	__u8 ctrl_data[ISP_CTRL_DATA_LENGTH];
	__u32 size; // must no more than ISP_CTRL_DATA_LENGTH!
	__u8 dir;
	__u64 timestamp;
};

struct isp_ctrl_ext {
	__u32 ctrl_id;
	struct mem_buf buf;
	__u32 size;
	__u8 dir;
	__u64 timestamp;
};

struct isp_gamma_febe_ctrl {
	__u32 compress[ISP_CTRL_FEBE_NUM];
	__u32 expand[ISP_CTRL_FEBE_NUM];
	__u8  flag;
};

struct isp_rgbgamma_data {
	__u32 rgbgc_r_px[64];
	__u32 rgbgc_r_datax[63];
	__u32 rgbgc_r_datay[64];
	__u32 rgbgc_g_px[64];
	__u32 rgbgc_g_datax[63];
	__u32 rgbgc_g_datay[64];
	__u32 rgbgc_b_px[64];
	__u32 rgbgc_b_datax[63];
	__u32 rgbgc_b_datay[64];
	__u8  flag;
};

struct isp_wdr5_data {
	__u32 lut_histogram_write_data[65];
	__u32 lut_shift_write_data[65];
	__u32 lut_shift0_write_data[65];
	__u32 lut_gammapre_write_data[65];
	__u32 lut_gammadown_write_data[65];
	__u32 lut_entropy_write_data[65];

	__u32 lut_distance_weight_write_data[65];
	__u32 lut_difference_weight_write_data[65];
	__u32 lut_flat_factor_write_data[272];
	__u8 lut_flat_level_write_data[68];
	__u32 lut_sat_shift_write_data[18];

	__u8 histogram_w_data_changed;
	__u8 lut_shift_w_data_changed;
	__u8 lut_shift0_w_data_changed;
	__u8 gammapre_w_data_changed;
	__u8 gammadown_w_data_changed;
	__u8 entropy_w_data_changed;

	__u8 lut_distance_weight_w_data_changed;
	__u8 difference_weight_w_data_changed;
	__u8 flat_factor_w_data_changed;
	__u8 flat_level_w_data_changed;
	__u8 sat_shift_w_data_changed;
};

enum isp_work_mode {
	ISP_MODE_INVALID = 0,
	ISP_STRM_MODE,
	ISP_MCM_MODE,
	ISP_RDMA_MODE,
	ISP_MODE_MAX,
};

struct isp_func {
	__u32 work_mode;
	__u8 tile_en;
	union {
		struct {
			__u16 online;
			__u16 stream_idx;
		} mcm;
	};
	__u8 hdr_sram;
};

struct isp_buf {
	__u32 valid;
	struct cam_format fmt;
	struct mem_buf mem;
};

struct isp_mcm_sch {
	__u32 id;
	__u32 hdr_en;
	__u32 tile_en;
	__u32 online_mcm;
	struct isp_buf rdma_buf;
	struct isp_buf mp_buf;
};

struct isp_vi_info {
	__u8 sensor_id;
	__u8 hdr_en;
};

struct isp_frame_info {
	__u32 frame_id;
	__u64 time_stamp;
};

struct isp_metadata {
	struct mem_buf buf;
	__u8 last;
};

enum group_type {
	V4L_GROUP = 0,
	CUSTOM_GROUP,
};

struct isp_msg {
	__u32 id;
	__u32 inst;
	__u32 group; /* enum group_type */
	union {
		struct cam_reg reg;
		struct cam_input in;
		struct cam_format_cap fcap;
		struct {
			__u32 num;
			union isp_irq_stat stat;
		} irq;
		struct isp_format fmt;
		__u32 state; /* enum cam_state */
		__u32 unit_test;
		struct isp_ctrl ctrl;
		struct isp_ctrl_ext ctrl_ext;
		struct isp_func func;
		struct isp_mcm_sch sch;
		struct cam_clk clk;
		struct cam_log log;
		struct isp_vi_info vinfo;
		struct isp_frame_info frame_info;
		struct isp_metadata meta;
		__u32 meta_enabled;
		__u32 tune_enabled;
		struct isp_gamma_febe_ctrl febe_ctrl;
		struct sen_ctrl sen_ctrl;
		struct iommu_map_buf map_buf;
	};
};

#endif /* _UAPI_LINUX_ISP_H_ */
