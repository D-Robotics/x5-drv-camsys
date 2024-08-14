/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_VSE_H_
#define _UAPI_LINUX_VSE_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"
#include "mem_helper_uapi.h"

#define LUT_NUM 64
#define BIN_LEVEL_NUM (3)
#define VSE_OSD_MAX   (4)
#define VSE_HIST_MAX  (8)

#define VSE_UID(n) cam_fourcc('v', 's', 'e', (n) + '0')

#define VSE_MSG_ALLOC_CMD_BUF (0x1)
#define VSE_MSG_SET_OSD_BUF   (0x2)

#define VSE_MSG_IRQ_STAT         (0x1 << 8)
#define VSE_MSG_SRC_ALT          (0x2 << 8)
#define VSE_MSG_MCM_SCH          (0x3 << 8)
#define VSE_MSG_CASCADE_UPDATE   (0x4 << 8)
#define VSE_MSG_SKIP_FRAME       (0x5 << 8)
#define VSE_MSG_OSD_INFO         (0x6 << 8)
#define VSE_MSG_OSD_SCH          (0x7 << 8)
#define VSE_MSG_LOAD_LUT         (0x8 << 8)
#define VSE_MSG_MODE_CHANGED     (0x9 << 8)
#define VSE_MSG_HIST_UPDATE      (0x10 << 8)
#define VSE_MSG_BIN_LEVEL_UPDATE (0x11 << 8)

#define VSE_OUT_CHNL_MAX (6)

struct vse_fmt_cap {
	__u32 format;
	__u8 index;
	__u8 res_num;
	struct cam_res_cap res[VSE_OUT_CHNL_MAX];
};

enum scale_type {
	SCALE_BYPASS,
	SCALE_DOWN,
	SCALE_UP,
};

struct vse_format {
	__u32 io_type; /* enum cam_io_type */
	union {
		struct {
			struct cam_format fmt;
		} in;
		struct {
			struct cam_format fmt;
			struct cam_rect icrop;
			__u32 scale_type; /* enum scale_type */
		} out;
	};
};

struct vse_hist_info {
    __u8  histId;
    __u8  histEnable;
    __u32 histHsize;
    __u32 histVsize;
    __u32 histStartX;
    __u32 histStartY;
};

struct vse_osd_info {
	__u8 roiId;
	__u8 roiEnable;
	__u32 roiHsize;
	__u32 roiVsize;
	__u32 roiStartX;
	__u32 roiStartY;
};

struct vse_lut_tbl {
	__u32 alpha_data[LUT_NUM];
	__u32 rgb_data[LUT_NUM];
};

struct vse_cmd_buf {
	__u16 ready;
	__u16 num;
	struct {
		__u32 offset;
		__u32 value;
	} regs[0];
};

struct vse_osd_buf {
	struct mem_buf buf;
	__u8 id;
};

struct vse_sch {
	__u32 id;
	__u32 source;
	__u32 ochn_en_mask;
	__u32 work_mode;
	struct mem_buf rdma_buf;
	struct mem_buf mp_buf[VSE_OUT_CHNL_MAX];
};

struct vse_cascade {
	__u8 cascade_id;
	__u8 cascade_enable;
};

enum vse_src {
	VSE_SRC_STRM0,
	VSE_SRC_STRM1,
	VSE_SRC_STRM2,
	VSE_SRC_STRM3,
	VSE_SRC_RESV,
	VSE_SRC_RDMA,
	VSE_SRC_MAX,
};

enum vse_work_mode {
	VSE_MCM_MODE,
	VSE_SCM_MODE,
};

struct vse_msg {
	__u32 id;
	__u32 inst;
	__u32 channel;
	union {
		struct cam_reg reg;
		struct cam_input in;
		struct vse_fmt_cap fcap;
		struct {
			__u32 num;
			__u32 stat;
		} irq;
		struct vse_hist_info hist_info[VSE_HIST_MAX];
		__u8   bin_level[BIN_LEVEL_NUM];
		struct vse_format fmt;
		struct vse_osd_info osd_info;
		struct vse_lut_tbl lut_tbl;
		struct mem_buf cmd;
		struct vse_osd_buf osd;
		__u32 state;  /* enum cam_state */
		__u32 source; /* enum vse_src */
		__u32 mode;   /* enum vse_work_mode */
		struct vse_sch sch;
		struct vse_cascade cascade;
		struct cam_clk clk;
		struct cam_log log;
	};
};

#endif /* _UAPI_LINUX_VSE_H_ */
