/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_ISP_H_
#define _UAPI_LINUX_ISP_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"
#include "mem_helper_uapi.h"

#define ISP_UID(n) cam_fourcc('i', 's', 'p', (n) + '0')

#define ISP_MCM_CLOCK (CAM_CLOCK_EXT_BASE + 1)

#define ISP_MSG_UNIT_TEST      (0x1 << 0)
#define ISP_MSG_GET_FUNC       (0x2 << 0)
#define ISP_MSG_GET_VI_INFO    (0x3 << 0)
#define ISP_MSG_GET_FRAME_INFO (0x4 << 0)

#define ISP_MSG_IRQ_MIS    (0x1 << 8)
#define ISP_MSG_MCM_SCH    (0x2 << 8)
#define ISP_MSG_TUNE_EN    (0x3 << 8)
#define ISP_MSG_FRAME_DONE (0x4 << 8)

#define ISP_CTRL_DATA_LENGTH (128)

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
};

struct isp_ctrl_ext {
	__u32 ctrl_id;
	struct mem_buf buf;
	__u32 size;
	__u8 dir;
};

enum isp_work_mode {
	ISP_STRM_MODE,
	ISP_MCM_MODE,
	ISP_RDMA_MODE,
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

struct isp_msg {
	__u32 id;
	__u32 inst;
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
		__u32 tune_enabled;
	};
};

#endif /* _UAPI_LINUX_ISP_H_ */
