/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_SIF_H_
#define _UAPI_LINUX_SIF_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"

#define SIF_UID(n) cam_fourcc('s', 'i', 'f', (n) + '0')
#define SIF_MSG_SET_CFG (0x1)

struct time_stamp_ctrl {
	__u8 time_stamp_en;
	__u8 trigger_mode; // Timestamp trigger source selelct
	__u8 ts_trigger_src;
	__u8 pps_trigger_src;
};

struct frame_id_ctrl {
	__u8 frame_id_en;
	__u16 initial_frame_id;
};

struct embedded_data_ctrl {
	__u8 ebd_en;
	__u8 ipi_mode; // 0:ipi16, 1:ipi48.
	__u8 ebd_halt_en;
	__u8 ebd_post;
	__u8 ebd_halt_len; // control minimal axi_clk cycles of ipi_halt
	__u16 ebd_size;    // Total EBD bytes.
	__u16 ebd_hsize;   // EBD HSIZE within one frame(actual value - 1, shall align with 16Bytes)
	__u16 ebd_vsize;   // EBD VSIZE within one frame(actual value - 1, shall align with 16Bytes)
	__u16 ebd_stride;  // EBD horizontal stride within one frame(actual value - 1) , byte unit
	__u16 ebd_addr;    // base addr for EBD data
};

struct sif_cfg {
	__u8 yuv_conv;   // set true to enable yuv conversion from nv16 to nv12.
	__u8 fps_ctrl;   // control fps, 0 is original fps, n=1-15, 1/(n+1) fps.
	__u8 sram_merge; // 0:do not merge ram
	__u8 isp_ctrl;   // 0:do not disable data to isp.
	__u8 hdr_mode;   // 0: do not use hdr. 1: two frames merge. 2: three frames merge.
	struct time_stamp_ctrl ts_ctrl;
	struct frame_id_ctrl f_id_ctrl;
	struct embedded_data_ctrl ebd_ctrl;
};

struct sif_msg {
	__u32 id;
	__u32 inst;
	union {
		struct cam_input in;
		struct cam_format_cap fcap;
		struct cam_format fmt;
		struct sif_cfg cfg;
		__u32 state; /* enum cam_state */
		struct cam_clk clk;
	};
};

#endif /* _UAPI_LINUX_SIF_H_ */
