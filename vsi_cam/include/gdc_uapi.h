/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_GDC_H_
#define _UAPI_LINUX_GDC_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"
#include "mem_helper_uapi.h"

#define GDC_UID(n)              cam_fourcc('g', 'd', 'c', (n) + '0')

#define GDC_MSG_SET_CFG_BUF     (0x1)

#define GDC_MSG_IRQ_STAT        (0x1 << 8)

struct gdc_format {
	struct cam_format ifmt;
	struct cam_format ofmt;
};

struct gdc_msg {
	__u32 id;
	__u32 inst;
	union {
		struct cam_reg reg;
		struct cam_input in;
		struct cam_format_cap fcap;
		struct {
			__u32 num;
			__u32 stat;
		} irq;
		struct gdc_format fmt;
		struct mem_buf cfg_buf;
		__u32 state; /* enum cam_state */
		struct cam_clk clk;
	};
};

#endif /* _UAPI_LINUX_GDC_H_ */
