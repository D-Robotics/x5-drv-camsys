/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_DEV_H_
#define _CAM_DEV_H_

#include <linux/interrupt.h>

struct cam_res_cap;
struct cam_format;

struct mem_res {
	const char *name;
	void __iomem *base;
};

struct irq_res {
	const char *name;
	int no;
	irq_handler_t handler;
	void *arg;
};

struct clk_res {
	const char *name;
	struct clk *clk;
};

struct rst_res {
	const char *name;
	struct reset_control *rst;
};

struct cam_dt {
	u32 id, num_insts;
	struct mem_res *mems;
	struct irq_res *irqs;
	struct clk_res *clks;
	struct rst_res *rsts;
};

int parse_cam_dt(struct platform_device *pdev, struct cam_dt *dt, void *arg);
bool check_framesize(struct cam_res_cap *cap, u32 size, struct cam_format *fmt);
u32 get_framebuf_size(struct cam_format *fmt);
struct csiw_device *get_csi_wrapper_device(struct platform_device *pdev);
struct cam_ctrl_device *get_cam_ctrl_device(struct platform_device *pdev);

#endif /* _CAM_DEV_H_ */
