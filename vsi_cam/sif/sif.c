// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/mfd/syscon.h>
#include <linux/reset.h>
#include <linux/debugfs.h>

#include "cam_ctrl.h"
#include "cam_dev.h"
#include "isc.h"
#include "sif_uapi.h"
#include "sif_regs.h"

#include "sif.h"

void sif_post(struct sif_device *sif, void *msg, u32 len)
{
	struct isc_post_param param = {
		.msg = msg,
		.msg_len = len,
		.lock = &sif->isc_lock,
		.sync = false,
	};

	if (sif->isc)
		isc_post(sif->isc, &param);
}

#ifdef EN_CHK_FMT
static bool check_format(struct sif_instance *ins, struct cam_format *fmt)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ins->fmt_cap); i++) {
		struct sif_format_cap *cap = &ins->fmt_cap[i];

		if (!cap->format)
			return false;
		if (cap->format == fmt->format)
			return check_framesize(cap->res, ARRAY_SIZE(cap->res),
					       fmt);
	}
	return false;
}
#endif

static void sif_set_emb_data(struct sif_device *sif, u32 inst)
{
	struct sif_cfg *sif_cfg = &sif->insts[inst].sif_cfg;
	u32 h_size = ALIGN(sif_cfg->ebd_ctrl.ebd_hsize, 16);
	u32 v_size = sif_cfg->ebd_ctrl.ebd_vsize;

	if (!sif_cfg->ebd_ctrl.ebd_en)
		return;

	sif_write(sif, SIF_IPI_EBD_CTRL(inst), 0xf335);

	if (sif_cfg->ebd_ctrl.ebd_post) {
		sif_write(sif, SIF_IPI_EBD_SIZE_POST(inst),
				  SIF_EBD_HSIZE(h_size) | SIF_EBD_VSIZE(v_size));
		sif_write(sif, SIF_IPI_EBD_TOTAL_BYTE(inst),
				  SIF_EBD_POST_TOTAL(h_size * v_size));
		sif_write(sif, SIF_IPI_EBD_HSTRIDE(inst), SIF_EBD_POST_HSTRIDE(h_size));
	} else {
		sif_write(sif, SIF_IPI_EBD_SIZE_PRE(inst),
				  SIF_EBD_HSIZE(h_size) | SIF_EBD_VSIZE(v_size));
		sif_write(sif, SIF_IPI_EBD_TOTAL_BYTE(inst),
		  		  SIF_EBD_PRE_TOTAL(h_size * v_size));
		sif_write(sif, SIF_IPI_EBD_HSTRIDE(inst), SIF_EBD_PRE_HSTRIDE(h_size));
	}
}

static void sif_set_ipi_feature(struct sif_device *sif, u32 inst)
{
	u32 val;
	u8 ipi_trigger_source, pps_trigger_source;
	struct sif_instance *ins;

	ins = &sif->insts[inst];

	if (ins->sif_cfg.f_id_ctrl.frame_id_en) {
		sif_write(sif, SIF_IPI_FRAME_ID_INITAL(inst),
			  ins->sif_cfg.f_id_ctrl.initial_frame_id);
		sif_write(sif, SIF_IPI_FRAME_ID_CTRL(inst), 0x3);
	}

	ipi_trigger_source = ins->sif_cfg.ts_ctrl.ts_trigger_src ?
			     (ins->sif_cfg.ts_ctrl.ts_trigger_src - 1) : sif->ipi_trigger_src;
	pps_trigger_source = ins->sif_cfg.ts_ctrl.pps_trigger_src ?
			     (ins->sif_cfg.ts_ctrl.pps_trigger_src - 1) : sif->pps_trigger_src;

	if (ins->sif_cfg.ts_ctrl.time_stamp_en) {
		if (ins->sif_cfg.ts_ctrl.trigger_mode & IPI_VSYNC ||
			ins->sif_cfg.ts_ctrl.trigger_mode & IPI_TRIGGER) {
			sif_write(sif, SIF_IPI_TIMESTAMP_CTRL(inst), ipi_trigger_source << 1);
			val = sif_read(sif, SIF_IPI_TIMESTAMP_CTRL(inst));
			/*
			 * BIT 0 - time_stamp_en
			 * BIT 6 - select trigger event both edge
			 * */
			sif_write(sif, SIF_IPI_TIMESTAMP_CTRL(inst), val | BIT(0) | BIT(6));
		}

		if (ins->sif_cfg.ts_ctrl.trigger_mode &  PPS0_TRIGGER) {
			sif_write(sif, SIF_PPS_TIMESTAMP_CTRL(0), BIT(7) | pps_trigger_source << 1);
			sif_write(sif, SIF_PPS_TIMEOUT_INTVAL(0), 0xffffffff);
			sif_write(sif, SIF_PPS_IRQ_EN, BIT(1) | BIT(0));
			val = sif_read(sif, SIF_PPS_TIMESTAMP_CTRL(0));
			sif_write(sif, SIF_PPS_TIMESTAMP_CTRL(0), val | BIT(0));
		}

		if (ins->sif_cfg.ts_ctrl.trigger_mode &  PPS1_TRIGGER) {
			sif_write(sif, SIF_PPS_TIMESTAMP_CTRL(1), BIT(7) | pps_trigger_source << 1);
			sif_write(sif, SIF_PPS_TIMEOUT_INTVAL(1), 0xffffffff);
			sif_write(sif, SIF_PPS_IRQ_EN, BIT(3) | BIT(2));
			val = sif_read(sif, SIF_PPS_TIMESTAMP_CTRL(1));
			sif_write(sif, SIF_PPS_TIMESTAMP_CTRL(1), val | BIT(0));
		}
	}

	if (ins->sif_cfg.hdr_mode) {
	    sif_write(sif, SIF_HDR_CTRL, 0x90 | ins->sif_cfg.hdr_mode);
	    sif_write(sif, SIF_HDR_EN, 0x1);
	}
}

static void sif_set_ex_feature(struct sif_device *sif, u32 inst)
{
	u32 val;
	unsigned long flags;
	struct sif_instance *ins;

	ins = &sif->insts[inst];
	spin_lock_irqsave(&sif->cfg_reg_lock, flags);
	if (ins->sif_cfg.fps_ctrl) {
		val = sif_read(sif, SIF_DMA_FPS_CTRL);
		val |= ins->sif_cfg.fps_ctrl << (inst * 4);
		sif_write(sif, SIF_DMA_FPS_CTRL, val);
	}
	if (ins->sif_cfg.yuv_conv) {
		val = sif_read(sif, SIF_YUV_CONV_CTRL);
		val |= BIT(inst);
		sif_write(sif, SIF_YUV_CONV_CTRL, val);
	}
	if (ins->sif_cfg.sram_merge) {
		val = sif_read(sif, SIF_IPI_RAM_CTRL);
		val |= BIT(ins->sif_cfg.sram_merge - 1);
		sif_write(sif, SIF_IPI_RAM_CTRL, val);
	}
	spin_unlock_irqrestore(&sif->cfg_reg_lock, flags);
}

static int sif_set_ipi_fmt(struct sif_device *sif, u32 inst, struct cam_format *fmt)
{
	u32 val, reg_val;

	val = sif_read(sif, SIF_DMA_CTL);

	val |= SIF_WR_LIMIT_ENABLE | SIF_WR_LIMIT_OUTSTAND(8);

	val &= SIF_FMT_CLEAR[inst];
	switch (fmt->format) {
	case CAM_FMT_RAW8:
		val |= SIF_FMT_RAW8_IPI[inst];
		fmt->stride = fmt->width;
		break;
	case CAM_FMT_RAW10:
		val |= SIF_FMT_RAW10_IPI[inst];
		fmt->stride = fmt->width * 2;
		break;
	case CAM_FMT_RAW12:
		val |= SIF_FMT_RAW12_IPI[inst];
		fmt->stride = fmt->width * 2;
		break;
	case CAM_FMT_YUYV:
		val |= SIF_FMT_YUYV_IPI[inst];
		fmt->stride = fmt->width * 2;
		reg_val = sif_read(sif, SIF_ISP_CTRL);
		sif_write(sif, SIF_ISP_CTRL, reg_val | BIT(inst));
		break;
	case CAM_FMT_NV12:
		val |= SIF_FMT_NV12_IPI[inst];
		fmt->stride = fmt->width;
		reg_val = sif_read(sif, SIF_ISP_CTRL);
		sif_write(sif, SIF_ISP_CTRL, reg_val | BIT(inst));
		break;
	case CAM_FMT_NV16:
		val |= SIF_FMT_YUY422SP_IPI[inst];
		fmt->stride = fmt->width;
		reg_val = sif_read(sif, SIF_ISP_CTRL);
		sif_write(sif, SIF_ISP_CTRL, reg_val | BIT(inst));
		break;
	case CAM_FMT_RGB888X:
		val |= SIF_FMT_RGB888_IPI[inst];
		fmt->stride = fmt->width * 4;
		reg_val = sif_read(sif, SIF_ISP_CTRL);
		sif_write(sif, SIF_ISP_CTRL, reg_val | BIT(inst));
		break;
	default:
		dev_err(sif->dev, "unsupported format 0x%x.\n", fmt->format);
		return -EINVAL;
	}

	sif_write(sif, SIF_DMA_CTL, val);
	sif_write(sif, SIF_IMG_OUT_BLENTH, SIF_BURST_LENGTH);

	sif_write(sif, SIF_IPI_PIX_HSTRIDE(inst), fmt->stride - 1);
	sif_write(sif, SIF_IPI_PIX_HSIZE(inst), fmt->width - 1);
	sif_write(sif, SIF_IPI_PIX_VSIZE(inst), fmt->height - 1);

	return 0;
}

static int sif_config(struct sif_device *sif, u32 inst, struct cam_format *fmt,
		      enum sif_channel_type channel_type)
{
	int ret;

	if (channel_type == IN_CHANNEL || channel_type == BOTH_CHANNEL)
		sif_set_ipi_feature(sif, inst);

	if (channel_type == EX_FEAT_CHANNEL || channel_type == BOTH_CHANNEL)
		sif_set_ex_feature(sif, inst);

	if (channel_type == OUT_CHANNEL_MAIN || channel_type == BOTH_CHANNEL) {
		ret = sif_set_ipi_fmt(sif, inst, fmt);
		if (ret)
			return ret;
	}
	if (channel_type == OUT_CHANNEL_EMB || channel_type == BOTH_CHANNEL)
		sif_set_emb_data(sif, inst);

	return 0;
}

int sif_set_format(struct sif_device *sif, u32 inst, struct cam_format *fmt,
		   bool post, enum sif_channel_type channel_type)
{
	struct sif_instance *ins;
	int rc = 0;

	if (!sif || !fmt || inst >= sif->num_insts || inst + sif->ipi_channel_num > 4)
		return -EINVAL;

	ins = &sif->insts[inst];
#ifdef EN_CHK_FMT
	if (!check_format(ins, fmt))
		return -EINVAL;
#endif

	if (post && inst == sif->ipi_base) {
		struct sif_msg msg;

		msg.id = CAM_MSG_FORMAT_CHANGED;
		msg.inst = inst;
		memcpy(&msg.fmt, fmt, sizeof(msg.fmt));
		sif_post(sif, &msg, sizeof(msg));
	}

	rc = sif_config(sif, inst, fmt, channel_type);
	if (rc < 0)
		return rc;
	memcpy(&ins->fmt, fmt, sizeof(ins->fmt));
	return 0;
}

static void sif_start_ipi(struct sif_device *dev, u32 inst)
{
	struct sif_instance *sif;
	struct sif_irq_ctx *ctx;
	phys_addr_t p_addr = 0;
	phys_addr_t p_uv_addr = 0;
	unsigned long flags;
	u32 val;
	u32 irq_val;

	sif = &dev->insts[inst];
	spin_lock_irqsave(&sif->lock, flags);
	ctx = &sif->ctx;
	if (ctx->buf_ctx)
		ctx->buf = cam_dqbuf_irq(ctx->buf_ctx, true);
	if (ctx->buf) {
		p_addr = get_phys_addr(ctx->buf, 0);
		if (sif->fmt.format == CAM_FMT_NV12 || sif->fmt.format == CAM_FMT_NV16
			|| (dev->ipi_channel_num != 1  && inst == sif->ipi_base))
			p_uv_addr = p_addr + (sif->fmt.stride * sif->fmt.height);
	}
	spin_unlock_irqrestore(&sif->lock, flags);
	if (p_addr) {
		if (sif->fmt.format == CAM_FMT_NV12 || sif->fmt.format == CAM_FMT_NV16) {
			sif_write(dev, SIF_IPI_BADDR_Y(inst), p_addr);
			if (p_uv_addr)
				sif_write(dev, SIF_IPI_BADDR_UV(inst), p_uv_addr);
		} else if (dev->ipi_channel_num != 1 && inst == sif->ipi_base) {
			sif_write(dev, SIF_IPI_BADDR_Y(inst), p_addr);
			if (p_uv_addr)
				sif_write(dev, SIF_IPI_BADDR_Y(inst + 1), p_uv_addr);
		} else {
			sif_write(dev, SIF_IPI_BADDR_Y(inst), p_addr);
		}
	}

	sif->hsize_err_count_pre = 0;
	sif->vsize_err_count_pre = 0;
	spin_lock_irqsave(&dev->cfg_reg_lock, flags);

	irq_val = sif_read(dev, SIF_IPI_IRQ_EN(inst));
	irq_val |= SIF_IRQ_FS | SIF_IPI_FRAME_END_EN | SIF_IRQ_OF | SIF_FRAME_SIZE_ERROR_EN | SIF_PIXEL_BUF_AFULL_IRQ_EN;
	val = sif_read(dev, SIF_DMA_CTL);
	val |= SIF_DMA_CONFIG_IPI[inst];
	if (sif->ctx.buf_ctx || inst != sif->ipi_base) {
		val |= SIF_ENABLE_IPI[inst];
		irq_val |= SIF_IRQ_DONE;
	}
	sif_write(dev, SIF_DMA_CTL, val);

	if (sif->ctx.emb_buf_ctx)
		irq_val |= SIF_IRQ_EBD_DMA_DONE;

	sif_write(dev, SIF_IPI_IRQ_CLR(inst), irq_val);
	sif_write(dev, SIF_IPI_IRQ_EN(inst), irq_val);

	spin_unlock_irqrestore(&dev->cfg_reg_lock, flags);
}

static void sif_stop_ipi(struct sif_device *dev, u32 inst)
{
	struct sif_instance *sif;
	unsigned long flags;
	u32 val;
	u32 irq_val;

	sif = &dev->insts[inst];
	spin_lock_irqsave(&dev->cfg_reg_lock, flags);
	irq_val = sif_read(dev, SIF_IPI_IRQ_EN(inst));
	irq_val &= ~(SIF_IRQ_FS | SIF_IPI_FRAME_END_EN | SIF_IRQ_OF | SIF_FRAME_SIZE_ERROR_EN | SIF_PIXEL_BUF_AFULL_IRQ_EN);
	if (sif->ctx.buf_ctx || inst != sif->ipi_base) {
		val = sif_read(dev, SIF_DMA_CTL);
		val &= ~SIF_ENABLE_IPI[inst];
		sif_write(dev, SIF_DMA_CTL, val);
		irq_val &= ~(SIF_IRQ_DONE);
	}

	if (sif->ctx.emb_buf_ctx)
		irq_val &= ~SIF_IRQ_EBD_DMA_DONE;

	sif_write(dev, SIF_IPI_IRQ_EN(inst), irq_val);
	spin_unlock_irqrestore(&dev->cfg_reg_lock, flags);
}

int sif_reset_ipi(struct sif_device *sif, u32 inst)
{
	u32 val;
	int retrycnt = 100;

	sif_write(sif, SIF_IPI_RESET, BIT(inst));
	do {
		val = sif_read(sif, SIF_IPI_RESET);
	} while ((val != (BIT(inst) << 4)) && (--retrycnt));

	if (retrycnt > 0) {
		dev_info(sif->dev, "sif ipi rest done\n");
		return 0;
	}

	if (retrycnt == 0 && (val != (BIT(inst) << 4))) {
		dev_info(sif->dev, "sif ipi rest failed\n");
		return -1;
	}

	return 0;
}

int sif_set_state(struct sif_device *sif, u32 inst, int enable, bool post)
{
	struct sif_instance *ins;
	enum cam_state state;
	unsigned long flags;
	int i;

	if (!sif || inst >= sif->num_insts || inst + sif->ipi_channel_num > 4 )
		return -EINVAL;

	for (i = 0; i < sif->ipi_channel_num; i++) {
		ins = &sif->insts[inst];
		if (enable) {
			ins->last_frame_done = 0;
			ins->frame_interval = 0;
			ins->frame_count = 0;
			state = CAM_STATE_STARTED;
			sif_start_ipi(sif, inst);
		} else {
			state = CAM_STATE_STOPPED;
		}
		ins->size_err_cnt = 0;
		spin_lock_irqsave(&ins->lock, flags);
		ins->state = state;
		spin_unlock_irqrestore(&ins->lock, flags);

		if (post && inst == sif->ipi_base) {
			struct sif_msg msg;

			msg.id = CAM_MSG_STATE_CHANGED;
			msg.inst = inst;
			msg.state = state;
			sif_post(sif, &msg, sizeof(msg));
		}

		if (!enable)
			sif_stop_ipi(sif, inst);
		inst++;
	}

	return 0;
}

int sif_set_ctx(struct sif_device *sif, u32 inst, struct sif_irq_ctx *ctx)
{
	struct sif_instance *ins;
	unsigned long flags;

	if (!sif || !ctx || inst >= sif->num_insts)
		return -EINVAL;

	ins = &sif->insts[inst];
	spin_lock_irqsave(&ins->lock, flags);
	ins->ctx = *ctx;
	spin_unlock_irqrestore(&ins->lock, flags);
	return 0;
}

static void sif_bound(struct isc_handle *isc, void *arg)
{
	struct sif_device *sif = (struct sif_device *)arg;
	unsigned long flags;

	if (sif) {
		spin_lock_irqsave(&sif->isc_lock, flags);
		if (!sif->isc) {
			isc_get(isc);
			sif->isc = isc;
		}
		spin_unlock_irqrestore(&sif->isc_lock, flags);
	}
}

static void sif_unbind(void *arg)
{
	struct sif_device *sif = (struct sif_device *)arg;
	unsigned long flags;

	if (sif) {
		spin_lock_irqsave(&sif->isc_lock, flags);
		if (sif->isc) {
			isc_put(sif->isc);
			sif->isc = NULL;
		}
		spin_unlock_irqrestore(&sif->isc_lock, flags);
	}
}

static struct isc_notifier_ops sif_notifier_ops = {
	.bound = sif_bound,
	.unbind = sif_unbind,
	.got = sif_msg_handler,
};

int sif_probe(struct platform_device *pdev, struct sif_device *sif)
{
	struct device *dev = &pdev->dev;
	int rc;
	struct mem_res sif_mems[] = {
		{ "reg", NULL },
		{},
	};
	struct irq_res sif_irqs[] = {
		{ "sif", -1, sif_irq_handler, sif },
		{},
	};
	struct clk_res sif_clks[] = {
		{ "axi", NULL },
		{ "pclk", NULL },
		{},
	};
	struct rst_res sif_rsts[] = {
		{ "rst", NULL },
		{},
	};
	struct cam_dt sif_dt = {
		.id = 0,
		.num_insts = 0,
		.mems = sif_mems,
		.irqs = sif_irqs,
		.clks = sif_clks,
		.rsts = sif_rsts,
	};
	u32 i;

	if (!sif)
		return -EINVAL;

	rc = parse_cam_dt(pdev, &sif_dt, sif);
	if (rc < 0) {
		dev_err(dev, "failed to call parse_cam_dt (err=%d)\n", rc);
		return rc;
	}

	sif->dev = dev;
	sif->id = sif_dt.id;
	sif->num_insts = sif_dt.num_insts;
	sif->base = sif_dt.mems[0].base;
	sif->axi = sif_dt.clks[0].clk;
	sif->pclk = sif_dt.clks[1].clk;
	sif->rst = sif_dt.rsts[0].rst;
	spin_lock_init(&sif->isc_lock);
	spin_lock_init(&sif->cfg_reg_lock);

	sif->insts = devm_kcalloc(dev, sif_dt.num_insts, sizeof(*sif->insts),
				  GFP_KERNEL);
	if (!sif->insts)
		return -ENOMEM;

	sif->ctrl_dev = get_cam_ctrl_device(pdev);
	if (IS_ERR(sif->ctrl_dev))
		return PTR_ERR(sif->ctrl_dev);

	rc = of_property_read_u32(pdev->dev.of_node, "timestamp-clk", &sif->timestamp_clk);
	if (rc) {
		dev_err(&pdev->dev, "couldn't get timestamp-clk value\n");
		return rc;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "ipi-trigger-src", &sif->ipi_trigger_src);
	if (rc) {
		dev_err(&pdev->dev, "couldn't get ipi-trigger-src value\n");
		return rc;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "pps-trigger-src", &sif->pps_trigger_src);
	if (rc) {
		dev_err(&pdev->dev, "couldn't get pps-trigger-src value\n");
		return rc;
	}

	rc = isc_register(SIF_UID(sif->id), &sif_notifier_ops, sif);
	if (rc < 0) {
		dev_err(dev, "failed to call isc_register (err=%d)\n", rc);
		return rc;
	}

	for (i = 0; i < sif_dt.num_insts; i++)
		spin_lock_init(&sif->insts[i].lock);

	dev_dbg(dev, "VS SIF driver #%d (base) probed done\n", sif->id);
	return 0;
}

int sif_remove(struct platform_device *pdev, struct sif_device *sif)
{
	int rc;

	rc = isc_unregister(SIF_UID(sif->id));
	if (rc < 0)
		dev_err(&pdev->dev, "failed to call isc_unregister (err=%d)\n",
			rc);

	put_cam_ctrl_device(sif->ctrl_dev);
	dev_dbg(&pdev->dev, "VS SIF driver #%d (base) removed\n", sif->id);
	return 0;
}

void sif_reset(struct sif_device *sif)
{
	if (sif->rst) {
		reset_control_assert(sif->rst);
		udelay(2);
		reset_control_deassert(sif->rst);
	}
}

#ifdef CONFIG_DEBUG_FS
static ssize_t sif_debugfs_fps_read(struct file *f, char __user *buf,
				    size_t size, loff_t *pos)
{
	struct sif_device *sif = f->f_inode->i_private;
	struct sif_instance *ins;
	char *output = NULL;
	size_t output_size = 0;
	size_t output_len = 0;
	ssize_t all_bytes_read = 0;
	ssize_t targets_read;
	u32 i, fps;

	output_size = 40 * sif->num_insts;
	output = kmalloc(output_size, GFP_KERNEL);
	if (!output)
		return -ENOMEM;

	output_len += snprintf(output + output_len, output_size - output_len,
			       "sif[%d]: ", sif->id);
	for (i = 0; i < sif->num_insts; i++) {
		ins = &sif->insts[i];
		if (ins->frame_interval)
			fps = 100000 * (ins->frame_count - 1) / ins->frame_interval;
		else
			fps = 0;
		output_len += snprintf(output + output_len, output_size - output_len,
				       "ipi[%d] fps:%d  ", i, fps / 100);
	}
	output_len += snprintf(output + output_len, output_size - output_len, "\n");

	all_bytes_read = output_len;

	if (*pos >= all_bytes_read)
		return 0;

	targets_read = min(size, (size_t)(all_bytes_read - *pos));
	if (copy_to_user(buf, output + *pos, targets_read))
		return -EFAULT;

	*pos += targets_read;
	kfree(output);
	return targets_read;
}

static const struct file_operations sif_debugfs_fps_fops = {
	.owner  = THIS_MODULE,
	.read  = sif_debugfs_fps_read,
	.llseek = seq_lseek,
};

void sif_debugfs_init(struct sif_device *sif)
{
	char sub_dir_name[10];

	snprintf(sub_dir_name, sizeof(sub_dir_name), "sif%d", sif->id);

	if (!sif->debugfs_dir)
		sif->debugfs_dir = debugfs_create_dir(sub_dir_name, NULL);
	if (!sif->debugfs_dir)
		return;
	if (!sif->debugfs_fps_file)
		sif->debugfs_fps_file = debugfs_create_file
				("fps", 0444, sif->debugfs_dir, sif,
				&sif_debugfs_fps_fops);
}

void sif_debugfs_remo(struct sif_device *sif)
{
	if (sif->debugfs_dir) {
		debugfs_remove_recursive(sif->debugfs_dir);
		sif->debugfs_dir = NULL;
		sif->debugfs_fps_file = NULL;
	}
}
#endif

#ifdef CONFIG_PM_SLEEP
int sif_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

int sif_system_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

#ifdef CONFIG_PM
int sif_runtime_suspend(struct device *dev)
{
	struct sif_device *sif = dev_get_drvdata(dev);

	if (sif->axi)
		clk_disable_unprepare(sif->axi);
	if (sif->pclk)
		clk_disable_unprepare(sif->pclk);
	return 0;
}

int sif_runtime_resume(struct device *dev)
{
	struct sif_device *sif = dev_get_drvdata(dev);
	int rc;

	if (sif->axi) {
		rc = clk_prepare_enable(sif->axi);
		if (rc)
			return rc;
	}
	if (sif->pclk) {
		rc = clk_prepare_enable(sif->pclk);
		if (rc) {
			clk_disable_unprepare(sif->axi);
			return rc;
		}
	}
	return 0;
}
#endif
