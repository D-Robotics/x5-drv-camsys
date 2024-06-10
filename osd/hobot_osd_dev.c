/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/bug.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <asm/atomic.h>

#include "hobot_osd_dev.h"
#include "vse_drv.h"
#include <linux/module.h>

struct osd_color_map g_osd_color = {
	.color_map_update = 0,
	.color_map = {
		0x8080FF, 0x808000, 0x808082, 0x6BFF1D, 0x152C96, 0xFF554C,
		0x01ABB3, 0xEBD469, 0x9501E2, 0xC241A2, 0x8D6432, 0xC7B89B,
		0x73D012, 0x3D4B5E, 0xD06530, 0x808050
	}
};

uint32_t g_osd_fps[OSD_CHN_MAX][VIO_MAX_STREAM] = {0, };
uint32_t g_osd_idx[OSD_CHN_MAX][VIO_MAX_STREAM] = {0, };
uint32_t g_osd_fps_lasttime[OSD_CHN_MAX][VIO_MAX_STREAM] = {0, };

struct osd_dev *g_osd_dev = NULL;

static struct vio_osd_info *g_osd_info[6][VIO_MAX_STREAM];

void osd_set_info(uint32_t chn_id, uint32_t ctx_id, struct vio_osd_info *info)
{
	g_osd_info[chn_id][ctx_id] = info;
}

struct vio_osd_info* osd_get_info(uint32_t chn_id, uint32_t ctx_id)
{
	return g_osd_info[chn_id][ctx_id];
}

static void osd_frame_process(struct vio_osd_info *osd_info)
{
	struct osd_subdev *osd_subdev;
	struct osd_dev *osd_dev;
	struct vse_nat_instance *vse_ctx = NULL;
	struct vio_framemgr *framemgr;
	struct vio_frame *frame;
	u64 flags = 0;
	int32_t ret = 0;

	osd_dev = g_osd_dev;

	osd_subdev = &osd_dev->subdev[osd_info->chn_id][osd_info->ctx_id];
	if (osd_dev->task_state != OSD_TASK_START) {
		osd_debug("task request stop, exit\n");
		goto exit;
	}

	vse_ctx = container_of(osd_info, struct vse_nat_instance, osd_info);
	if (!vse_ctx) {
		osd_err("get vse ctx fail\n");
		goto exit;
	}

	framemgr = vse_ctx->vdev.cur_fmgr;
	vio_e_barrier_irqs(framemgr, flags);
	frame = peek_frame(framemgr, FS_PROCESS);
	vio_x_barrier_irqr(framemgr, flags);

	ret = osd_queue_push(&osd_subdev->queue, frame);
	if (ret) {
		osd_err("push queue failed\n");
		goto exit;
	}
	atomic_inc(&osd_subdev->osd_info->frame_count);
	if (!kthread_queue_work(&osd_dev->worker, &osd_subdev->work)) {
		osd_err("kthread_queue_work failed\n");
		goto exit;
	}

	return;
exit:
	osd_subdev->osd_info->return_frame(osd_subdev->osd_info, frame);
}

static void osd_hw_set_roi_addr_config(struct osd_subdev *subdev)
{
	struct osd_bind *bind = NULL, *temp;
	uint32_t hw_cnt = 0;
	struct vse_osd_cfg *osd_hw_cfg;

	if (subdev->osd_hw_cfg == NULL)
		return;

	spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
	osd_hw_cfg = subdev->osd_hw_cfg;
	list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
		if (hw_cnt >= OSD_HW_PROC_NUM)
			break;

		if (bind->proc_info.proc_type == OSD_PROC_HW_VGA8) {
			osd_hw_cfg->osd_box[hw_cnt].osd_en = bind->proc_info.show_en;
			osd_hw_cfg->osd_box[hw_cnt].overlay_mode = bind->proc_info.invert_en;
			osd_hw_cfg->osd_box[hw_cnt].start_x = (uint16_t)bind->proc_info.start_x;
			osd_hw_cfg->osd_box[hw_cnt].start_y = (uint16_t)bind->proc_info.start_y;
			osd_hw_cfg->osd_box[hw_cnt].width = (uint16_t)bind->proc_info.width;
			osd_hw_cfg->osd_box[hw_cnt].height = (uint16_t)bind->proc_info.height;

			if (osd_hw_cfg->osd_buf[hw_cnt] != 0)
				osd_single_buffer_dec_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);

			osd_hw_cfg->osd_buf[hw_cnt] = (uint32_t)bind->proc_info.src_paddr;
			osd_single_buffer_inc_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);
			osd_debug("[CHN%d][CTX%d] hw_cnt: %d, osd_en: %d, mode: %d "
				"x: %d y: %d, w: %d h: %d, paddr: 0x%x\n",
				subdev->chn_id, subdev->ctx_id, hw_cnt,
				osd_hw_cfg->osd_box[hw_cnt].osd_en,
				osd_hw_cfg->osd_box[hw_cnt].overlay_mode,
				osd_hw_cfg->osd_box[hw_cnt].start_x,
				osd_hw_cfg->osd_box[hw_cnt].start_y,
				osd_hw_cfg->osd_box[hw_cnt].width,
				osd_hw_cfg->osd_box[hw_cnt].height,
				osd_hw_cfg->osd_buf[hw_cnt]);
			hw_cnt++;
		}
	}

	if (hw_cnt > 0) {
		subdev->osd_hw_limit_y = osd_hw_cfg->osd_box[hw_cnt - 1].start_y +
					osd_hw_cfg->osd_box[hw_cnt - 1].height;
	} else {
		subdev->osd_hw_limit_y = 0;
	}

	for (; hw_cnt < OSD_HW_PROC_NUM; hw_cnt++) {
		memset(&osd_hw_cfg->osd_box[hw_cnt], 0, sizeof(struct osd_box));

		if (osd_hw_cfg->osd_buf[hw_cnt] != 0)
			osd_single_buffer_dec_by_paddr(osd_hw_cfg->osd_buf[hw_cnt]);

		osd_hw_cfg->osd_buf[hw_cnt] = 0;
	}

	osd_hw_cfg->osd_box_update = 1;
	osd_hw_cfg->osd_buf_update = 1;
	atomic_set(&subdev->osd_hw_need_update, 0);
	spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

// static void osd_process_set_pym_addr(struct osd_process_info *proc_info,
//     struct mp_vio_frame *frame, struct pym_subdev *pym_subdev)
// {
//     uint32_t buf_layer;
//     struct special_buffer *pym_spec;

//     buf_layer = proc_info->buf_layer;
//     pym_spec = &frame->common_frame.frameinfo.spec;
//     if (buf_layer < MAX_PYM_DS_COUNT) {
//         proc_info->image_width =
//             ALIGN(pym_subdev->pym_cfg.stds_box[buf_layer].tgt_width, 16);
//         proc_info->image_height = pym_subdev->pym_cfg.stds_box[buf_layer].tgt_height;

//         if ((buf_layer == 0) &&
//             (test_bit(PYM_DMA_INPUT, &pym_subdev->pym_dev->state))) {
//             proc_info->tar_y_addr = __va(frame->common_frame.frameinfo.addr[0]);
//             proc_info->tar_uv_addr = __va(frame->common_frame.frameinfo.addr[1]);
//         } else {
//             proc_info->tar_y_addr = __va(pym_spec->ds_y_addr[buf_layer]);
//             proc_info->tar_uv_addr = __va(pym_spec->ds_uv_addr[buf_layer]);
//         }
//     } else {
//         buf_layer = buf_layer - MAX_PYM_DS_COUNT;
//         proc_info->image_width =
//             ALIGN(pym_subdev->pym_cfg.stus_box[buf_layer].tgt_width, 16);
//         proc_info->image_height = pym_subdev->pym_cfg.stus_box[buf_layer].tgt_height;

//         proc_info->tar_y_addr = __va(pym_spec->us_y_addr[buf_layer]);
//         proc_info->tar_uv_addr = __va(pym_spec->us_uv_addr[buf_layer]);
//     }
// }

static void osd_hw_set_color_map(struct osd_subdev *subdev)
{
	struct vse_osd_cfg  *osd_hw_cfg;

	if ((subdev->chn_id >= OSD_CHN_MAX) || (subdev->osd_hw_cfg == NULL))
		return;

	spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
	osd_hw_cfg = subdev->osd_hw_cfg;
	memcpy(osd_hw_cfg->color_map.color_map, g_osd_color.color_map,
		MAX_OSD_COLOR_NUM * sizeof(uint32_t));
	osd_hw_cfg->color_map.color_map_update = 1;
	g_osd_color.color_map_update = 0;
	spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

static void osd_set_process_bind_info(struct osd_process_info *process_info,
					struct osd_bind_info *bind_info)
{
	process_info->show_en = bind_info->show_en;
	process_info->invert_en = bind_info->invert_en;
	process_info->osd_level = bind_info->osd_level;
	process_info->start_x = bind_info->start_point.x;
	process_info->start_y = bind_info->start_point.y;
	process_info->buf_layer = bind_info->buf_layer;

	if (process_info->proc_type >= OSD_PROC_RECT) {
		process_info->fill_color = bind_info->handle_info.fill_color;
		process_info->width = bind_info->handle_info.size.w;
		process_info->height = bind_info->handle_info.size.h;
		process_info->polygon_buf = bind_info->handle_info.polygon_buf;
	}

	if (process_info->proc_type == OSD_PROC_HW_VGA8)
		atomic_set(&process_info->subdev->osd_hw_need_update, 1);
}

static void osd_set_process_info_workfunc(struct kthread_work *work)
{
	int32_t i, j;
	struct osd_bind *bind = NULL, *temp;
	struct osd_subdev *subdev;
	struct osd_dev *osd_dev;
	int32_t handle_id;
	struct osd_handle *handle;

	osd_dev = container_of(work, struct osd_dev, work);

	for (i = 0; i < OSD_CHN_MAX; i++) {
		for (j = 0; j < VIO_MAX_STREAM; j++) {
			subdev = &osd_dev->subdev[i][j];

			if (g_osd_color.color_map_update == 1) {
				osd_hw_set_color_map(subdev);
				g_osd_color.color_map_update = 0;
			}

			mutex_lock(&subdev->bind_mutex);
			list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
				mutex_lock(&bind->proc_info.proc_mutex);
				handle_id = bind->bind_info.handle_id;
				mutex_lock(&osd_dev->osd_list_mutex);
				handle = osd_find_handle_node(osd_dev, handle_id);
				if (handle == NULL) {
					osd_err("[H%d][CHN%d] attached, but handle was destroyed!\n",
						handle_id, subdev->chn_id);
				} else {
					osd_set_process_handle_info(&bind->proc_info, handle);
				}
				mutex_unlock(&osd_dev->osd_list_mutex);

				if (atomic_read(&bind->need_update) > 0) {
					osd_set_process_bind_info(&bind->proc_info, &bind->bind_info);
					atomic_set(&bind->need_update, 0);
				}

				mutex_unlock(&bind->proc_info.proc_mutex);
			}
			if (atomic_read(&subdev->osd_hw_need_update) > 0) {
				osd_hw_set_roi_addr_config(subdev);
			}
			mutex_unlock(&subdev->bind_mutex);
		}
	}
}

static struct file_operations osd_cdev_fops = {
	.owner = THIS_MODULE,
	.open = hb_osd_open,
	.write = hb_osd_write,
	.read = hb_osd_read,
	.release = hb_osd_close,
	.unlocked_ioctl = hb_osd_ioctl,
	.compat_ioctl = hb_osd_ioctl,
};

static int32_t hb_osd_suspend(struct device *dev)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;

	osd_dev = dev_get_drvdata(dev);
	if (atomic_read(&osd_dev->open_cnt) > 0) {
		osd_stop_worker(osd_dev);
	}

	osd_info("done\n");

	return ret;
}

static int32_t hb_osd_resume(struct device *dev)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;

	osd_dev = dev_get_drvdata(dev);
	if (atomic_read(&osd_dev->open_cnt) > 0) {
		ret = osd_start_worker(g_osd_dev);
	}

	osd_info("done\n");

	return ret;
}

static int32_t hb_osd_runtime_suspend(struct device *dev)
{
	int32_t ret = 0;

	osd_info("done\n");

	return ret;
}

static int32_t hb_osd_runtime_resume(struct device *dev)
{
	int32_t ret = 0;

	osd_info("done\n");

	return ret;
}

static const struct dev_pm_ops hb_osd_pm_ops = {
	.suspend = hb_osd_suspend,
	.resume = hb_osd_resume,
	.runtime_suspend = hb_osd_runtime_suspend,
	.runtime_resume = hb_osd_runtime_resume,
};

static int32_t osd_device_node_init(struct osd_dev *osd)
{
	int32_t ret = 0;
	struct device *dev;

	ret = alloc_chrdev_region(&osd->devno, 0, OSD_MAX_DEVICE, OSD_NAM);
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev osd", ret);
		goto err_req_cdev;
	}

	cdev_init(&osd->cdev, &osd_cdev_fops);
	osd->cdev.owner = THIS_MODULE;
	ret = cdev_add(&osd->cdev, osd->devno, OSD_MAX_DEVICE);
	if (ret) {
		pr_err("Error %d while adding osd cdev", ret);
		goto err;
	}

	osd->class = class_create(THIS_MODULE, OSD_NAM);
	dev = device_create(osd->class, NULL, MKDEV(MAJOR(osd->devno), 0), NULL, OSD_NAM);
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		pr_err("osd device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(osd->class);
err_req_cdev:
	unregister_chrdev_region(osd->devno, OSD_MAX_DEVICE);
	return ret;
}

static ssize_t osd_handle_show(struct device *dev,
                	struct device_attribute *attr, char* buf)
{
	uint32_t offset = 0;
	int32_t len;
	struct osd_dev *osd_dev;
	struct osd_handle *handle, *temp;

	osd_dev = dev_get_drvdata(dev);

	mutex_lock(&osd_dev->osd_list_mutex);
	len = snprintf(&buf[offset], PAGE_SIZE - offset,
		"******************osd handle info******************\n");
	offset += len;

	list_for_each_entry_safe(handle, temp, &osd_dev->osd_list, node) {
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"[H%d] info: bind_cnt:%d ref_cnt:%d fill_color:%d bg_trans:%d "
			"proc_type:%d size:%dx%d \n",
			handle->info.handle_id,
			atomic_read(&handle->bind_cnt), atomic_read(&handle->ref_cnt),
			handle->info.fill_color,
			handle->info.yuv_bg_transparent, handle->info.proc_type,
			handle->info.size.w, handle->info.size.h);
		offset += len;
		len = snprintf(&buf[offset], PAGE_SIZE - offset,
			"[H%d] buffer: size: %dx%d \n",
			handle->info.handle_id,
			handle->buffer.size.w, handle->buffer.size.h);
		offset += len;
		if (handle->buffer.buf[0].state != OSD_BUF_NULL) {
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"    buf[0]: state:%d pixel format:%d length:%ld paddr:0x%llx "
				"vaddr:%p ref_count:%d \n",
				handle->buffer.buf[0].state, handle->buffer.buf[0].pixel_fmt,
				handle->buffer.buf[0].length,
				handle->buffer.buf[0].paddr, handle->buffer.buf[0].vaddr,
				atomic_read(&handle->buffer.buf[0].ref_count));
			offset += len;
		}
		if (handle->buffer.buf[1].state != OSD_BUF_NULL) {
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"    buf[1]: state:%d pixel format:%d length:%ld paddr:0x%llx "
				"vaddr:%p ref_count:%d \n",
				handle->buffer.buf[1].state, handle->buffer.buf[1].pixel_fmt,
				handle->buffer.buf[1].length,
				handle->buffer.buf[1].paddr, handle->buffer.buf[1].vaddr,
				atomic_read(&handle->buffer.buf[1].ref_count));
			offset += len;
		}
		if (handle->buffer.vga_buf[0].state != OSD_BUF_NULL) {
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"    vga_buf[0]: state:%d pixel format:%d length:%ld paddr:0x%llx "
				"vaddr:%p ref_count:%d \n",
				handle->buffer.vga_buf[0].state, handle->buffer.vga_buf[0].pixel_fmt,
				handle->buffer.vga_buf[0].length,
				handle->buffer.vga_buf[0].paddr, handle->buffer.vga_buf[0].vaddr,
				atomic_read(&handle->buffer.vga_buf[0].ref_count));
			offset += len;
		}
		if (handle->buffer.vga_buf[1].state != OSD_BUF_NULL) {
			len = snprintf(&buf[offset], PAGE_SIZE - offset,
				"    vga_buf[1]: state:%d pixel format:%d length:%ld paddr:0x%llx "
				"vaddr:%p ref_count:%d \n",
				handle->buffer.vga_buf[1].state, handle->buffer.vga_buf[1].pixel_fmt,
				handle->buffer.vga_buf[1].length,
				handle->buffer.vga_buf[1].paddr, handle->buffer.vga_buf[1].vaddr,
				atomic_read(&handle->buffer.vga_buf[1].ref_count));
			offset += len;
		}
		len = snprintf(&buf[offset], PAGE_SIZE - offset, "****************\n");
		offset += len;
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	return offset;
}

static DEVICE_ATTR(handle_info, S_IRUGO, osd_handle_show, NULL);

static ssize_t osd_bind_show(struct device *dev,
                struct device_attribute *attr, char* buf)
{
	uint32_t offset = 0;
	// int32_t i, j, m, len;
	// struct osd_dev *osd_dev;
	// struct osd_subdev *subdev;
	// struct osd_bind *bind, *temp;

	// osd_dev = dev_get_drvdata(dev);

	// len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 		"******************osd bind info******************\n");
	// offset += len;
	// for (i = 0; i < OSD_CHN_MAX; i++) {
	// 	for (j = 0; j < VIO_MAX_STREAM; j++) {
	// 		subdev = &osd_dev->subdev[i][j];

	// 		mutex_lock(&subdev->bind_mutex);
	// 		list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
	// 			len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 				"[S%d][V%d][H%d]: show:%d invert:%d level:%d "
	// 				"buf_layer:%d start: (%d, %d) bind_cnt:%d\n",
	// 				bind->bind_info.chn_id, bind->bind_info.ctx_id,
	// 				bind->bind_info.handle_id, bind->bind_info.show_en,
	// 				bind->bind_info.invert_en, bind->bind_info.osd_level,
	// 				bind->bind_info.buf_layer, bind->bind_info.start_point.x,
	// 				bind->bind_info.start_point.y, atomic_read(&bind->ref_cnt));
	// 			offset += len;
	// 			if (bind->bind_info.handle_info.proc_type == OSD_PROC_RECT) {
	// 				len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 					"rect: size:%dx%d fill_color:%d\n",
	// 					bind->bind_info.handle_info.size.w,
	// 					bind->bind_info.handle_info.size.h,
	// 					bind->bind_info.handle_info.fill_color);
	// 				offset += len;
	// 			}
	// 			if (bind->bind_info.handle_info.proc_type == OSD_PROC_POLYGON) {
	// 				len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 					"polygon: side num:%d fill_color:%d point:(%d, %d) \n"
	// 					"(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) "
	// 					"\n(%d, %d) (%d, %d) (%d, %d) buffer:%p\n",
	// 					bind->bind_info.side_num,
	// 					bind->bind_info.handle_info.fill_color,
	// 					bind->bind_info.point[0].x, bind->bind_info.point[0].y,
	// 					bind->bind_info.point[1].x, bind->bind_info.point[1].y,
	// 					bind->bind_info.point[2].x, bind->bind_info.point[2].y,
	// 					bind->bind_info.point[3].x, bind->bind_info.point[3].y,
	// 					bind->bind_info.point[4].x, bind->bind_info.point[4].y,
	// 					bind->bind_info.point[5].x, bind->bind_info.point[5].y,
	// 					bind->bind_info.point[6].x, bind->bind_info.point[6].y,
	// 					bind->bind_info.point[7].x, bind->bind_info.point[7].y,
	// 					bind->bind_info.point[8].x, bind->bind_info.point[8].y,
	// 					bind->bind_info.point[9].x, bind->bind_info.point[9].y,
	// 					bind->bind_info.polygon_buf);
	// 				offset += len;
	// 			}
	// 			if (bind->bind_info.handle_info.proc_type == OSD_PROC_MOSAIC) {
	// 				len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 					"mosaic: size:%dx%d\n",
	// 					bind->bind_info.handle_info.size.w,
	// 					bind->bind_info.handle_info.size.h);
	// 				offset += len;
	// 			}
	// 			len = snprintf(&buf[offset], PAGE_SIZE - offset, "******************\n");
	// 			offset += len;
	// 		}
	// 		mutex_unlock(&subdev->bind_mutex);

	// 		if (subdev->osd_hw_cfg != NULL) {
	// 			spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
	// 			for (m = 0; m < OSD_HW_PROC_NUM; m++) {
	// 				if (subdev->osd_hw_cfg->osd_box[m].osd_en == 0) {
	// 					continue;
	// 				}
	// 				len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 					"[V%d][%d]: hw en:%d mode:%d start:(%d, %d) size:(%d, %d)\n",
	// 					subdev->chn_id, m,
	// 					subdev->osd_hw_cfg->osd_box[m].osd_en,
	// 					subdev->osd_hw_cfg->osd_box[m].overlay_mode,
	// 					subdev->osd_hw_cfg->osd_box[m].start_x,
	// 					subdev->osd_hw_cfg->osd_box[m].start_y,
	// 					subdev->osd_hw_cfg->osd_box[m].width,
	// 					subdev->osd_hw_cfg->osd_box[m].height);
	// 					offset += len;
	// 					len = snprintf(&buf[offset], PAGE_SIZE - offset,
	// 					"******************\n");
	// 				offset += len;
	// 			}
	// 			spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
	// 		}
	// 	}
	// }
	return offset;
}

static DEVICE_ATTR(bind_info, S_IRUGO, osd_bind_show, NULL);

static ssize_t osd_fps_show(struct device *dev,
			struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	int32_t i, j, len;
	char *osd_chn[OSD_CHN_MAX] = OSD_CHN_NAME;

	for (i = 0; i < OSD_CHN_MAX; i++) {
		for (j = 0; j < VIO_MAX_STREAM; j++) {
			if (g_osd_fps[i][j] > 0) {
				len = snprintf(&buf[offset], PAGE_SIZE - offset,
					"osd pipe %d:  %s output fps %d\n",
					i, osd_chn[j], g_osd_fps[i][j]);
				offset += len;
				g_osd_fps[i][j] = 0;
			}
		}
	}
	return offset;
}

static DEVICE_ATTR(fps, S_IRUGO, osd_fps_show, NULL);

static int32_t osd_sysfs_create(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_handle_info)) {
		pr_err("create handle info failed\n");
		return -ENOMEM;
	}
	if (device_create_file(dev, &dev_attr_bind_info)) {
		pr_err("create bind info failed\n");
		return -ENOMEM;
	}
	if (device_create_file(dev, &dev_attr_fps)) {
		pr_err("create fps failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void osd_sysfs_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_handle_info);
	device_remove_file(dev, &dev_attr_bind_info);
	device_remove_file(dev, &dev_attr_fps);
}

struct osd_interface_ops osd_cb_ops = {
	.frame_process = osd_frame_process,
	.osd_set_info = osd_set_info,
};

DECLARE_VIO_CALLBACK_OPS(osd_cops, OSD_COPS_MAGIC, &osd_cb_ops);

static int32_t osd_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	struct osd_dev *osd;
	struct ion_device *hb_ion_dev;

	BUG_ON(!pdev);

	osd = kzalloc(sizeof(struct osd_dev), GFP_ATOMIC);
	if (!osd) {
		osd_err("osd is NULL\n");
		return -ENOMEM;
	}

	ret = osd_device_node_init(osd);
	if (ret < 0) {
		osd_err("osd_device_node_init fail\n");
		ret = -ENOMEM;
		goto _exit_create_file;
	}

	ret = osd_sysfs_create(&pdev->dev);
	if (ret < 0) {
		osd_err("osd_sysfs_create fail\n");
		ret = -ENOMEM;
		goto _exit_create_file;
	}

	hb_ion_dev = hobot_ion_get_ion_device();
	if (hb_ion_dev == NULL) {
		osd_err("hb_ion_dev is null.\n");
		ret = -EFAULT;
		goto _exit_create_file;
	}

	osd->ion_client = ion_client_create(hb_ion_dev, "osd_driver_ion");
	if (IS_ERR(osd->ion_client)) {
		osd_err("ion client create failed.");
		goto _exit_create_file;
	}

	platform_set_drvdata(pdev, osd);

	kthread_init_worker(&osd->worker);
	kthread_init_work(&osd->work, osd_set_process_info_workfunc);
	INIT_LIST_HEAD(&osd->osd_list);
	mutex_init(&osd->osd_mutex);
	mutex_init(&osd->osd_list_mutex);
	atomic_set(&osd->open_cnt, 0);
	g_osd_dev = osd;

	ret = vio_register_callback_ops(&cb_osd_cops, VSE_MODULE, COPS_0);
	if (ret < 0) {
		pr_err("osd cops register failed\n");
		return ret;
	}

	osd_info("done\n");

	return 0;
_exit_create_file:
	kfree(osd);

	osd_err("fail\n");

	return ret;
}

static int32_t hobot_osd_remove(struct platform_device *pdev)
{
	int32_t ret = 0;
	struct osd_dev *osd;

	BUG_ON(!pdev);

	osd = platform_get_drvdata(pdev);

	ion_client_destroy(osd->ion_client);
	osd_sysfs_remove(&pdev->dev);

	mutex_destroy(&osd->osd_mutex);
	mutex_destroy(&osd->osd_list_mutex);
	device_destroy(osd->class, osd->devno);

	class_destroy(osd->class);
	cdev_del(&osd->cdev);
	unregister_chrdev_region(osd->devno, OSD_MAX_DEVICE);

	kfree(osd);

	pr_err("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_osd_match[] = {
	{
		.compatible = "hobot,osd",
	},
	{},
};

MODULE_DEVICE_TABLE(of, hobot_osd_match);

static struct platform_driver hobot_osd_driver = {
	.probe = osd_probe,
	.remove = hobot_osd_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &hb_osd_pm_ops,
		.of_match_table = hobot_osd_match,
	}
};

#else
static struct platform_device_id hobot_osd_driver_ids[] = {
	{
		.name = MODULE_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, hobot_osd_driver_ids);

static struct platform_driver hobot_osd_driver = {
	.probe = osd_probe,
	.remove = __devexit_p(hobot_osd_remove),
	.id_table = hobot_osd_driver_ids,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &hb_osd_pm_ops,
	}
};
#endif

static int32_t __init hobot_osd_init(void)
{
	int32_t ret = platform_driver_register(&hobot_osd_driver);
	if (ret)
		pr_err("platform_driver_register failed: %d\n", ret);

	return ret;
}
late_initcall(hobot_osd_init);

static void __exit hobot_osd_exit(void)
{
	platform_driver_unregister(&hobot_osd_driver);
}
module_exit(hobot_osd_exit);

MODULE_AUTHOR("Li Ming <ming01.li@horizon.cc>");
MODULE_DESCRIPTION("HOBOT OSD driver");
MODULE_LICENSE("GPL v2");
