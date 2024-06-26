// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[vse_drv]: %s: " fmt, __func__
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "cam_ctrl.h"
#include "cam_dev.h"
#include "cam_ctx.h"
#include "dw230_vse_regs.h"
#include "isc.h"
#include "vse_uapi.h"

#include "vse.h"

#define REFCNT_INIT_VAL (1)

#ifdef EN_CHK_FMT
static bool check_iformat(struct vse_instance *ins, struct cam_format *fmt)
{
	return true;
}

static bool check_oformat(struct vse_instance *ins, u32 chnl,
			 struct cam_format *fmt)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ins->fmt_cap); i++) {
		struct vse_format_cap *cap = &ins->fmt_cap[i];

		if (!cap->format)
			return false;
		if (cap->format == fmt->format)
			return check_framesize(cap->res[chnl], ARRAY_SIZE(cap->res[chnl]), fmt);
	}
	return false;
}
#endif

int vse_post(struct vse_device *vse, struct vse_msg *msg, bool sync)
{
	struct isc_post_param param = {
		.msg = msg,
		.msg_len = sizeof(*msg),
		.lock = &vse->isc_lock,
		.sync = sync,
	};

	if (vse->isc)
		return isc_post(vse->isc, &param);
	return -EINVAL;
}

int vse_set_source(struct vse_device *vse, u32 inst, u32 source)
{
	struct vse_instance *ins;
	struct vse_msg msg;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];

	ins->sch.source = source;
	ins->source = source;
	msg.id = VSE_MSG_SRC_ALT;
	msg.inst = inst;
	msg.channel = -1;
	msg.source = source;
	return vse_post(vse, &msg, false);
}

int vse_set_cascade(struct vse_device *vse, u32 inst, u32 cas_id, bool en_cas)
{
	struct vse_msg msg;

	msg.id = VSE_MSG_CASCADE_UPDATE;
	msg.inst = inst;
	msg.channel = -1;
	msg.cascade.cascade_id     = cas_id;
	msg.cascade.cascade_enable = en_cas;

	return vse_post(vse, &msg, true);
}

int vse_set_fps_rate(struct vse_device *vse, u32 inst, u32 chnl,
		     struct vse_fps_rate *fps)
{
	struct vse_instance *ins;

	if (!vse || !fps)
		return -EINVAL;

	if (inst >= vse->num_insts || chnl >= VSE_OUT_CHNL_MAX)
		return -EINVAL;

	ins = &vse->insts[inst];

	ins->fps[chnl].src = fps->src;
	ins->fps[chnl].dst = fps->dst;
	return 0;
}

int vse_server_trigger(struct vse_device *vse, u32 inst)
{
	struct vse_instance *ins;
	struct vse_msg msg;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];

	msg.id = VSE_MSG_MCM_SCH;
	msg.inst = inst;
	msg.channel = -1;
	memcpy(&msg.sch, &ins->sch, sizeof(msg.sch));
	msg.sch.work_mode = vse->mode;
	return vse_post(vse, &msg, false);
}

int vse_set_iformat(struct vse_device *vse, u32 inst, struct cam_format *fmt)
{
	struct vse_instance *ins;
	struct vse_msg msg;
	int rc;

	if (!vse || !fmt)
		return -EINVAL;

	if (inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];

#ifdef EN_CHK_FMT
	if (!check_iformat(ins, fmt))
		return -EINVAL;
#endif

	memset(&msg, 0, sizeof(msg));

	msg.id = CAM_MSG_FORMAT_CHANGED;
	msg.inst = inst;
	msg.channel = -1;
	msg.fmt.io_type = CAM_INPUT;
	memcpy(&msg.fmt.in.fmt, fmt, sizeof(msg.fmt.in.fmt));
	rc = vse_post(vse, &msg, true);
	if (rc < 0)
		return rc;

	memcpy(&ins->ifmt, fmt, sizeof(ins->ifmt));
	return 0;
}

int vse_set_oformat(struct vse_device *vse, u32 inst, u32 chnl, struct cam_format *fmt,
		    struct cam_rect *crop, bool chn_en)
{
	struct vse_instance *ins;
	struct vse_msg msg;
	int rc;

	if (!vse || !fmt || !crop)
		return -EINVAL;

	if (inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];

	if (chn_en)
		ins->fps[chnl].dst = ins->fps[chnl].src;
	else
		ins->fps[chnl].dst = 0;

#ifdef EN_CHK_FMT
	if (!check_oformat(ins, chnl, fmt))
		return -EINVAL;
#endif

	memset(&msg, 0, sizeof(msg));

	msg.id = CAM_MSG_FORMAT_CHANGED;
	msg.inst = inst;
	msg.channel = chnl;
	msg.fmt.io_type = CAM_OUTPUT;
	msg.fmt.out.scale_type = (chnl == 5 ? SCALE_UP : SCALE_DOWN);
	memcpy(&msg.fmt.out.fmt, fmt, sizeof(msg.fmt.out.fmt));
	memcpy(&msg.fmt.out.icrop, crop, sizeof(msg.fmt.out.icrop));
	rc = vse_post(vse, &msg, true);
	if (rc < 0)
		return rc;

	memcpy(&ins->ofmt[chnl], fmt, sizeof(ins->ofmt[chnl]));
	memcpy(&ins->crop[chnl], crop, sizeof(ins->crop[chnl]));
	return 0;
}

void vse_set_rdma_buffer(struct vse_device *vse, phys_addr_t phys_addr, struct cam_format *fmt)
{
	vse_write(vse, 0xcfc, phys_addr);
	vse_write(vse, 0xd00, phys_addr + fmt->stride * fmt->height);
	vse_write(vse, 0xd04, 0);
	vse_write(vse, 0xcf0, fmt->width);
	vse_write(vse, 0xcf4, fmt->height);
	vse_write(vse, 0xcf8, fmt->stride);
}

void vse_set_stitch_buffer(struct vse_device *vse, u32 chnl, struct vse_stitching *stitch,
			   phys_addr_t phys_addr, struct cam_format *fmt)
{
	u32 base;
	u32 y_size, y_offs, cb_offs;

	y_size = fmt->stride * fmt->height * stitch->vfactor;
	y_offs = fmt->stride * fmt->height * stitch->x + fmt->width * stitch->y;
	cb_offs = fmt->stride * fmt->height / 2 * stitch->x + fmt->width * stitch->y;
	base = VSE_MIn_BASE(chnl);

	vse_write(vse, VSE_MIn_Y_ADDR(base), phys_addr); /* share one buffer */
	vse_write(vse, VSE_MIn_Y_SIZE(base), y_size);
	vse_write(vse, VSE_MIn_Y_OFFS(base), y_offs);
	vse_write(vse, VSE_MIn_CB_ADDR(base), phys_addr + y_size);
	vse_write(vse, VSE_MIn_CB_SIZE(base), fmt->stride * fmt->height);
	vse_write(vse, VSE_MIn_CB_OFFS(base), cb_offs);
	vse_write(vse, VSE_MIn_CR_ADDR(base), 0x00000000);
	vse_write(vse, VSE_MIn_CR_SIZE(base), 0x00000000);
	vse_write(vse, VSE_MIn_CR_OFFS(base), 0x00000000);

	vse_write(vse, VSE_MIn_CTRL(base), 0x38);
}

void vse_set_mi_buffer(struct vse_device *vse, u32 chnl,
		       phys_addr_t phys_addr, struct cam_format *fmt)
{
	u32 base = VSE_MIn_BASE(chnl);

	vse_write(vse, VSE_MIn_Y_ADDR(base), phys_addr);
	vse_write(vse, VSE_MIn_Y_SIZE(base), fmt->stride * fmt->height);
	vse_write(vse, VSE_MIn_Y_OFFS(base), 0x00000000);
	vse_write(vse, VSE_MIn_CB_ADDR(base), phys_addr + fmt->stride * fmt->height);
	vse_write(vse, VSE_MIn_CB_SIZE(base), fmt->stride * fmt->height / 2);
	vse_write(vse, VSE_MIn_CB_OFFS(base), 0x00000000);
	vse_write(vse, VSE_MIn_CR_ADDR(base), 0x00000000);
	vse_write(vse, VSE_MIn_CR_SIZE(base), 0x00000000);
	vse_write(vse, VSE_MIn_CR_OFFS(base), 0x00000000);

	vse_write(vse, VSE_MIn_CTRL(base), 0x38);
}

#ifdef WITH_LEGACY_VSE
static void vse_start(struct vse_device *vse, struct cam_format *fmt)
{
	__u8 chnl;
	__u32 base;

	vse_write(vse, 0xd40, 0xffffffff);
	vse_write(vse, 0xcec, (fmt->width << 16));
	vse_write(vse, 0x304, 0x0000403f);
	vse_write(vse, 0x304, 0x0000c03f);
	vse_write(vse, 0xce8, (1 << 2) | 1);

	for (chnl = 0; chnl < VSE_OUT_CHNL_MAX; chnl++) {
		if (chnl == 3) /* exclude path3 in all scenarios */
			continue;
		base = VSE_OSDn_BASE(chnl);
		vse_write(vse, VSE_OSDn_CTRL(base), 0x0000047f);
	}
}

void vse_set_cmd(struct vse_device *vse, u32 inst)
{
	struct vse_irq_ctx *ctx;
	struct vse_instance *ins;
	phys_addr_t phys_addr = {0};
	u32 i;

	if(!vse)
		return;

	vse->error = 0;
	vse->is_completed = false;
	ins = &vse->insts[inst];
	ctx = &ins->ctx;

	if (ins->cmd_buf_va && ins->cmd_buf_va->ready) {
		for (i = 0; i < ins->cmd_buf_va->num; i++)
			vse_write(vse, ins->cmd_buf_va->regs[i].offset, ins->cmd_buf_va->regs[i].value);
	}

	if (ctx->sink_buf) {
		phys_addr = get_phys_addr(ctx->sink_buf, 0);
		vse_set_rdma_buffer(vse, phys_addr, &ins->ifmt);
	}

	ins->sch.ochn_en_mask = 0;
	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		struct vse_stitching *stitch = &ctx->stitches[i];

		if (stitch->enabled && ctx->src_buf[stitch->left_top]) {
			phys_addr = get_phys_addr(ctx->src_buf[stitch->left_top], 0);
			vse_set_stitch_buffer(vse, i, stitch, phys_addr, &ins->ofmt[i]);
		}
	}

	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (ctx->src_buf[i] && !ctx->stitches[i].enabled) {
			phys_addr = get_phys_addr(ctx->src_buf[i], 0);
			vse_set_mi_buffer(vse, i, phys_addr, &ins->ofmt[i]);
		}
	}
	vse_start(vse, &ins->ifmt);
}
#else
void vse_set_cmd(struct vse_device *vse, u32 inst)
{
	struct vse_irq_ctx *ctx;
	struct vse_instance *ins;
	phys_addr_t phys_addr = {0};
	u32 i;

	if(!vse)
		return;

	vse->error = 0;
	vse->is_completed = false;
	ins = &vse->insts[inst];
	ctx = &ins->ctx;

	if (ctx->sink_buf) {
		phys_addr = get_phys_addr(ctx->sink_buf, 0);
		ins->sch.rdma_buf.addr = phys_addr;
	}
	ins->sch.ochn_en_mask = 0;
	for (i = 0; i < VSE_OUT_CHNL_MAX; i++) {
		if (ctx->src_buf[i]) {
			phys_addr = get_phys_addr(ctx->src_buf[i], 0);
			pr_debug("vse chn%d mp addr:%x\n", i, (u32)phys_addr);
			ins->sch.mp_buf[i].addr = phys_addr;
			ins->sch.ochn_en_mask |= BIT(i);
			// set osd cfg for each channel
			cam_osd_set_cfg(ctx->src_ctx[i], i);
		}
	}

	vse_server_trigger(vse, inst);
	pr_debug("inst %d\n", inst);
}
#endif

int vse_set_state(struct vse_device *vse, u32 inst, int enable, u32 cur_cnt, u32 total_cnt)
{
	struct vse_instance *ins;
	struct vse_msg msg;
	int rc;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	if (cur_cnt < total_cnt)
		return 0;

	ins = &vse->insts[inst];
	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	msg.channel = -1;
	if (enable)
		msg.state = CAM_STATE_STARTED;
	else
		msg.state = CAM_STATE_STOPPED;
	rc = vse_post(vse, &msg, true);
	if (rc < 0)
		return rc;
	ins->state = msg.state;
	return 0;
}

int vse_set_osd_info(struct vse_device *vse, u32 inst, u32 chnl, struct vse_osd_info *info) {
	struct vse_msg msg;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	msg.id = VSE_MSG_OSD_INFO;
	msg.inst = inst;
	msg.channel = chnl;
	memcpy(&msg.osd_info, info, sizeof(struct vse_osd_info));

	return vse_post(vse, &msg, false);
}

int vse_set_osd_buf(struct vse_device *vse, u32 inst, u32 chnl, struct vse_osd_buf *osd_buf) {
	struct vse_msg msg;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	msg.id = VSE_MSG_OSD_SCH;
	msg.inst = inst;
	msg.channel = chnl;
	memcpy(&msg.osd, osd_buf, sizeof(struct vse_osd_buf));

	return vse_post(vse, &msg, false);
}

int vse_set_osd_lut(struct vse_device *vse, u32 inst, u32 chnl, struct vse_lut_tbl *lut_tbl) {
	struct vse_msg msg;

	if (!vse || inst >= vse->num_insts)
		return -EINVAL;

	msg.id = VSE_MSG_LOAD_LUT;
	msg.inst = inst;
	msg.channel = chnl;
	memcpy(&msg.lut_tbl, lut_tbl, sizeof(struct vse_lut_tbl));

	return vse_post(vse, &msg, false);
}

int vse_set_src_ctx(struct vse_device *vse, u32 inst, u32 chnl, struct cam_ctx *ctx)
{
	struct vse_instance *ins;
	unsigned long flags;

	if (!vse || inst >= vse->num_insts || chnl >= VSE_OUT_CHNL_MAX)
		return -EINVAL;

	ins = &vse->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ins->ctx.src_ctx[chnl] = ctx;
	spin_unlock_irqrestore(&ins->lock, flags);
	return 0;
}

int vse_set_ctx(struct vse_device *vse, u32 inst, struct vse_irq_ctx *ctx)
{
	struct vse_instance *ins;
	unsigned long flags;

	if (!vse || !ctx)
		return -EINVAL;

	if (inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ins->ctx = *ctx;
	ins->ctx.fps = ins->fps;
	memcpy(ins->ctx.cur_fps, ins->fps, sizeof(ins->ctx.cur_fps));
	spin_unlock_irqrestore(&ins->lock, flags);
	return 0;
}

int vse_add_job(struct vse_device *vse, u32 inst)
{
	struct irq_job job = { inst };
	struct vse_irq_ctx *ctx;
	int rc;
	u32 id;

	if (vse->mode != VSE_SCM_MODE) {
		pr_debug("add job inst:%d\n", inst);
		rc = push_job(vse->jq, &job);
		if (rc < 0) {
			dev_err(vse->dev, "failed to push a job(err=%d)\n", rc);
			return rc;
		}
	}

	if (vse->error) {
		ctx = get_next_irq_ctx(vse);
		if(ctx) {
			id = vse->next_irq_ctx;
			vse_set_cmd(vse, id);
		}
	}
	return 0;
}

static void vse_bound(struct isc_handle *isc, void *arg)
{
	struct vse_device *vse = (struct vse_device *)arg;
	unsigned long flags;

	if (vse) {
		vse->error = 1;
		spin_lock_irqsave(&vse->isc_lock, flags);
		if (!vse->isc) {
			isc_get(isc);
			vse->isc = isc;
		}
		spin_unlock_irqrestore(&vse->isc_lock, flags);
	}
}

static void vse_unbind(void *arg)
{
	struct vse_device *vse = (struct vse_device *)arg;
	unsigned long flags;

	if (vse) {
		spin_lock_irqsave(&vse->isc_lock, flags);
		if (vse->isc) {
			isc_put(vse->isc);
			vse->isc = NULL;
		}
		spin_unlock_irqrestore(&vse->isc_lock, flags);
	}
}

static struct isc_notifier_ops vse_notifier_ops = {
	.bound = vse_bound,
	.unbind = vse_unbind,
	.got = vse_msg_handler,
};

int vse_open(struct vse_device *vse, u32 inst)
{
	bool en_clk = false;
	int rc = 0;

	if (!vse)
		return -EINVAL;

	mutex_lock(&vse->open_lock);
	if (refcount_read(&vse->open_cnt) == REFCNT_INIT_VAL)
		en_clk = true;
	refcount_inc(&vse->open_cnt);
	mutex_unlock(&vse->open_lock);

	if (en_clk)
		rc = vse_runtime_resume(vse->dev);
	return rc;
}

int vse_close(struct vse_device *vse, u32 inst)
{
	struct vse_instance *ins;
	struct vse_msg msg;
	bool dis_clk = false;
	int rc;

	if (!vse)
		return -EINVAL;

	if (inst >= vse->num_insts)
		return -EINVAL;

	ins = &vse->insts[inst];
	memset(&ins->ifmt, 0, sizeof(ins->ifmt));
	memset(ins->ofmt, 0, sizeof(ins->ofmt));
	memset(ins->crop, 0, sizeof(ins->crop));
	memset(&ins->osd, 0, sizeof(ins->osd));
	ins->error = 1;
	memset(ins->fps, 0, sizeof(ins->fps));
	if (ins->cmd_buf_va) {
		dma_free_coherent(vse->dev, ins->cmd_buf.size, ins->cmd_buf_va, ins->cmd_buf.addr);
		ins->cmd_buf_va = NULL;
	}

	msg.id = CAM_MSG_STATE_CHANGED;
	msg.inst = inst;
	msg.channel = -1;
	msg.state = CAM_STATE_CLOSED;
	rc = vse_post(vse, &msg, true);
	if (rc < 0)
		dev_err(vse->dev, "failed to call vse_post (err=%d)\n", rc);

	mutex_lock(&vse->open_lock);
	if (refcount_read(&vse->open_cnt) > REFCNT_INIT_VAL) {
		refcount_dec(&vse->open_cnt);
		if (refcount_read(&vse->open_cnt) == REFCNT_INIT_VAL)
			dis_clk = true;
	}
	mutex_unlock(&vse->open_lock);

	if (!dis_clk)
		return 0;

	reset_job_queue(vse->jq);

	vse_reset(vse);
	vse->is_completed = true;
	vse->error = 1;
	return vse_runtime_suspend(vse->dev);
}

int vse_probe(struct platform_device *pdev, struct vse_device *vse)
{
	struct device *dev = &pdev->dev;
	int rc;
	struct mem_res vse_mems[] = {
		{ "reg", NULL },
		{},
	};
	struct irq_res vse_irqs[] = {
		{ "fe", -1, vse_fe_irq_handler, vse },
		{ "vse", -1, vse_irq_handler, vse },
		{},
	};
	struct clk_res vse_clks[] = {
		{ "core", NULL },
		{ "axi", NULL },
		{ "ups", NULL },
		{ "gdc_axi", NULL },
		{ "gdc_hclk", NULL },
		{},
	};
	struct rst_res vse_rsts[] = {
		{ "rst", NULL },
		{},
	};
	struct cam_dt vse_dt = {
		.id = 0,
		.num_insts = 0,
		.mems = vse_mems,
		.irqs = vse_irqs,
		.clks = vse_clks,
		.rsts = vse_rsts,
	};
	u32 i;

	if (!vse)
		return -EINVAL;

	rc = parse_cam_dt(pdev, &vse_dt, vse);
	if (rc < 0) {
		dev_err(dev, "failed to call parse_cam_dt (err=%d)\n", rc);
		return rc;
	}

	vse->dev = dev;
	vse->id = vse_dt.id;
	vse->num_insts = vse_dt.num_insts;
	vse->base = vse_dt.mems[0].base;
	vse->core = vse_dt.clks[0].clk;
	vse->axi = vse_dt.clks[1].clk;
	vse->ups = vse_dt.clks[2].clk;
	vse->gdc_axi = vse_dt.clks[3].clk;
	vse->gdc_hclk = vse_dt.clks[4].clk;
	vse->rst = vse_dt.rsts[0].rst;
	spin_lock_init(&vse->isc_lock);
	mutex_init(&vse->open_lock);
	refcount_set(&vse->open_cnt, REFCNT_INIT_VAL);

	vse->error = 1;
	vse->is_completed = true;

	vse->insts = devm_kcalloc(dev, vse_dt.num_insts,
				  sizeof(*vse->insts), GFP_KERNEL);
	if (!vse->insts)
		return -ENOMEM;

	vse->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(vse->ctrl_dev))
		return PTR_ERR(vse->ctrl_dev);

	vse->jq = create_job_queue(32);
	if (!vse->jq) {
		dev_err(dev, "failed to call create_job_queue\n");
		return -ENOMEM;
	}

	rc = isc_register(VSE_UID(vse->id), &vse_notifier_ops, vse);
	if (rc < 0) {
		dev_err(dev, "failed to call isc_register (err=%d)\n", rc);
		destroy_job_queue(vse->jq);
		return rc;
	}

	for (i = 0; i < vse_dt.num_insts; i++)
		spin_lock_init(&vse->insts[i].lock);

	dev_dbg(dev, "VS VSE driver (base) probed done\n");
	return 0;
}

int vse_remove(struct platform_device *pdev, struct vse_device *vse)
{
	int i, rc;

	rc = isc_unregister(VSE_UID(vse->id));
	if (rc < 0)
		dev_err(&pdev->dev, "failed to call isc_unregister (err=%d)\n", rc);

	destroy_job_queue(vse->jq);

	for (i = 0; i < vse->num_insts; i++) {
		struct vse_instance *ins = &vse->insts[i];

		if (ins->cmd_buf_va)
			dma_free_coherent(vse->dev, ins->cmd_buf.size, ins->cmd_buf_va, ins->cmd_buf.addr);
	}
	put_cam_ctrl_device(vse->ctrl_dev);
	dev_dbg(&pdev->dev, "VS VSE driver (base) removed\n");
	return rc;
}

void vse_reset(struct vse_device *vse)
{
	if (vse->rst) {
		reset_control_assert(vse->rst);
		udelay(2);
		reset_control_deassert(vse->rst);
	}
}

#ifdef CONFIG_DEBUG_FS
static ssize_t vse_debugfs_write(struct file *f, const char __user *buf,
				 size_t size, loff_t *pos)
{
	struct vse_device *vse = f->f_inode->i_private;
	char cmd[64], *str = cmd, *token;
	struct vse_msg msg;
	int rc;

	if (!size || size >= sizeof(cmd))
		return -EINVAL;

	rc = strncpy_from_user(cmd, buf, size);
	if (rc < 0)
		return rc;

	cmd[size] = '\0';
	memset(&msg, 0, sizeof(msg));
	msg.id = CAM_MSG_LOG_STATE_CHANGED;
	token = strsep(&str, " ");
	while (token) {
		msg.log.level = simple_strtoul(token, NULL, 10);
		pr_debug("%s [%d,%d]\n", __func__, msg.log.id, msg.log.level);
		rc = vse_post(vse, &msg, true);
		if (rc < 0) {
			pr_err("failed to post log state changed msg (err=%d)\n", rc);
			break;
		}
		msg.log.id++;
		token = strsep(&str, " ");
	}
	return size;
}

static const struct file_operations vse_debugfs_fops = {
	.owner  = THIS_MODULE,
	.write  = vse_debugfs_write,
	.llseek = seq_lseek,
};

void vse_debugfs_init(struct vse_device *vse)
{
	if (!vse->debugfs_dir)
		vse->debugfs_dir = debugfs_create_dir("vse", NULL);
	if (!vse->debugfs_dir)
		return;
	if (!vse->debugfs_file)
		vse->debugfs_file = debugfs_create_file
				("log", 0222, vse->debugfs_dir, vse,
				&vse_debugfs_fops);
}

void vse_debugfs_remo(struct vse_device *vse)
{
	if (vse->debugfs_dir) {
		debugfs_remove_recursive(vse->debugfs_dir);
		vse->debugfs_dir = NULL;
		vse->debugfs_file = NULL;
	}
}
#endif

#ifdef CONFIG_PM_SLEEP
int vse_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

int vse_system_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

#ifdef CONFIG_PM
int vse_runtime_suspend(struct device *dev)
{
	struct vse_device *vse = dev_get_drvdata(dev);

	if (vse->axi)
		clk_disable_unprepare(vse->axi);
	if (vse->gdc_axi)
		clk_disable_unprepare(vse->gdc_axi);
	if (vse->gdc_hclk)
		clk_disable_unprepare(vse->gdc_hclk);
	if (vse->core)
		clk_disable_unprepare(vse->core);
	if (vse->ups)
		clk_disable_unprepare(vse->ups);
	return 0;
}

int vse_runtime_resume(struct device *dev)
{
	struct vse_device *vse = dev_get_drvdata(dev);
	int rc;

	if (vse->gdc_hclk) {
		rc = clk_prepare_enable(vse->gdc_hclk);
		if (rc)
			return rc;
	}
	if (vse->gdc_axi) {
		rc = clk_prepare_enable(vse->gdc_axi);
		if (rc)
			goto _gdc_axi_err;
	}
	if (vse->axi) {
		rc = clk_prepare_enable(vse->axi);
		if (rc)
			goto _axi_err;
	}
	if (vse->core) {
		rc = clk_prepare_enable(vse->core);
		if (rc)
			goto _core_err;
	}
	if (vse->ups) {
		rc = clk_prepare_enable(vse->ups);
		if (rc)
			goto _ups_err;
	}
	return 0;
_ups_err:
	if (vse->core)
		clk_disable_unprepare(vse->core);
_core_err:
	if (vse->axi)
		clk_disable_unprepare(vse->axi);
_axi_err:
	if (vse->gdc_axi)
		clk_disable_unprepare(vse->gdc_axi);
_gdc_axi_err:
	if (vse->gdc_hclk)
		clk_disable_unprepare(vse->gdc_hclk);
	return rc;
}
#endif
