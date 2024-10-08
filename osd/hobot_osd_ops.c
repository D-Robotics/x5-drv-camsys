/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "hobot_osd_ops.h"
#include "vse_drv.h"

extern struct osd_color_map g_osd_color;
extern uint32_t g_osd_fps[OSD_CHN_MAX][VIO_MAX_STREAM];
extern uint32_t g_osd_idx[OSD_CHN_MAX][VIO_MAX_STREAM];
extern uint32_t g_osd_fps_lasttime[OSD_CHN_MAX][VIO_MAX_STREAM];

void osd_queue_init(struct osd_frame_queue *queue)
{
	int32_t i;

	if (!queue) {
		pr_err("input queue null\n");
		return;
	}

	INIT_LIST_HEAD(&queue->incoming_list);
	INIT_LIST_HEAD(&queue->process_list);
	INIT_LIST_HEAD(&queue->request_list);
	spin_lock_init(&queue->lock);

	queue->frames = (struct osd_frame *)kzalloc(sizeof(struct osd_frame) * OSD_QUEUE_SIZE, GFP_ATOMIC);

	for (i = 0; i < OSD_QUEUE_SIZE; i++) {
		list_add_tail(&queue->frames[i].node, &queue->request_list);
	}
}

void osd_queue_destroy(struct osd_frame_queue *queue)
{
	unsigned long flags;

	if (!queue) {
		pr_err("input queue null\n");
		return;
	}

	spin_lock_irqsave(&queue->lock, flags);
	kfree(queue->frames);
	INIT_LIST_HEAD(&queue->incoming_list);
	INIT_LIST_HEAD(&queue->process_list);
	INIT_LIST_HEAD(&queue->request_list);
	spin_unlock_irqrestore(&queue->lock, flags);
}

int32_t osd_queue_push(struct osd_frame_queue *queue, struct vio_frame *vio_frame)
{
	unsigned long flags;
	int32_t rc = 0;
	struct osd_frame *osd_frame;

	if (!queue || !vio_frame) {
		pr_err("input queue null\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);

	osd_frame = list_first_entry_or_null(&queue->request_list, struct osd_frame, node);
	if (!osd_frame) {
		pr_err("no request frame found\n");
		rc = -EBUSY;
		goto exit;
	}
	osd_frame->frame = vio_frame;

	list_del(&osd_frame->node);
	list_add_tail(&osd_frame->node, &queue->incoming_list);

exit:
	spin_unlock_irqrestore(&queue->lock, flags);
	return rc;
}

struct vio_frame *osd_queue_pop(struct osd_frame_queue *queue)
{
	unsigned long flags;
	struct osd_frame *osd_frame;
	struct vio_frame *rf = NULL;

	if (!queue) {
		pr_err("input queue null\n");
		return NULL;
	}

	spin_lock_irqsave(&queue->lock, flags);

	osd_frame = list_first_entry_or_null(&queue->incoming_list, struct osd_frame, node);
	if (!osd_frame)
		goto exit;

	rf = osd_frame->frame;

	list_del(&osd_frame->node);
	list_add_tail(&osd_frame->node, &queue->process_list);

exit:
	spin_unlock_irqrestore(&queue->lock, flags);
	return rf;
}

int32_t osd_queue_done(struct osd_frame_queue *queue)
{
	unsigned long flags;
	struct osd_frame *osd_frame;
	int32_t rc = 0;

	if (!queue) {
		pr_err("input queue null\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);

	osd_frame = list_first_entry_or_null(&queue->process_list, struct osd_frame, node);
	if (!osd_frame) {
		pr_err("no process frame found\n");
		rc = -EINVAL;
		goto exit;
	}

	list_del(&osd_frame->node);
	list_add_tail(&osd_frame->node, &queue->request_list);

exit:
	spin_unlock_irqrestore(&queue->lock, flags);
	return rc;
}

struct osd_handle *osd_find_handle_node(struct osd_dev *osd_dev, int32_t id)
{
	struct osd_handle *handle = NULL, *temp;

	if (!list_empty(&osd_dev->osd_list)) {
		list_for_each_entry_safe(handle, temp, &osd_dev->osd_list, node) {
			if (handle->info.handle_id == id) {
				return handle;
			}
		}
	}

	return NULL;
}

static struct osd_bind *osd_find_bind_node(struct osd_subdev *osd_subdev, int32_t id,
					int32_t buf_layer)
{
	struct osd_bind *bind = NULL, *temp;

	if (!list_empty(&osd_subdev->bind_list)) {
		list_for_each_entry_safe(bind, temp, &osd_subdev->bind_list, node) {
			if (bind->bind_info.handle_id == id) {
				// if ((osd_subdev->chn_id == OSD_PYM_OUT) &&
				// 	(bind->bind_info.buf_layer != buf_layer)) {
				// 	continue;
				// }
				return bind;
			}
		}
	}
	return NULL;
}

static int32_t osd_handle_info_check(struct osd_handle_info *handle_info)
{
	osd_debug("[H%d]: fill_color: %d yuv_bg_transparent: %d proc_type: %d size(%dx%d)\n",
		handle_info->handle_id, handle_info->fill_color,
		handle_info->yuv_bg_transparent, handle_info->proc_type,
		handle_info->size.w, handle_info->size.h);
	//todo: Add polygon check and print

	if (handle_info->handle_id >= OSD_HANDLE_MAX) {
		osd_err("handle id: %d exceed %d!\n", handle_info->handle_id, OSD_HANDLE_MAX);
		return -EINVAL;
	}

	if (handle_info->proc_type >= OSD_PROC_MAX_TYPE) {
		osd_err("handle id: %d proc_type %d exceed %d!\n",
			handle_info->handle_id, handle_info->proc_type, OSD_PROC_MAX_TYPE);
		return -EINVAL;
	}

	return 0;
}

static void osd_print_bind_info(struct osd_bind_info *bind_info)
{
	// osd_debug("[S%d][V%d][H%d]: show:%d invert:%d level:%d buf_layer:%d start: (%d, %d)\n",
	// 	bind_info->chn_id, bind_info->ctx_id, bind_info->handle_id,
	// 	bind_info->show_en, bind_info->invert_en, bind_info->osd_level,
	// 	bind_info->buf_layer, bind_info->start_point.x, bind_info->start_point.y);
	// osd_debug("polygon: side num:%d point:(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) "
	// 	"(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d) buffer:%p\n",
	// 	bind_info->side_num, bind_info->point[0].x, bind_info->point[0].y,
	// 	bind_info->point[1].x, bind_info->point[1].y,
	// 	bind_info->point[2].x, bind_info->point[2].y,
	// 	bind_info->point[3].x, bind_info->point[3].y,
	// 	bind_info->point[4].x, bind_info->point[4].y,
	// 	bind_info->point[5].x, bind_info->point[5].y,
	// 	bind_info->point[6].x, bind_info->point[6].y,
	// 	bind_info->point[7].x, bind_info->point[7].y,
	// 	bind_info->point[8].x, bind_info->point[8].y,
	// 	bind_info->point[9].x, bind_info->point[9].y,
	// 	bind_info->polygon_buf);
}

static int32_t handle_find_buffer(struct osd_handle *handle, enum osd_buf_state state,
				struct osd_single_buffer **single_buf, uint8_t is_vga)
{
	int32_t i = 0;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if (is_vga == 0) {
			if (handle->buffer.buf[i].state == state) {
				*single_buf = &handle->buffer.buf[i];
				return i;
			}
		} else {
			if (handle->buffer.vga_buf[i].state == state) {
				*single_buf = &handle->buffer.vga_buf[i];
				return i;
			}
		}
	}

	return -1;
}

static void handle_update_buffer(struct osd_handle *handle, int32_t index)
{
	int32_t i = 0;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if (handle->buffer.buf[i].state == OSD_BUF_PROCESS) {
			handle->buffer.buf[i].state = OSD_BUF_CREATE;
		}
		if (handle->buffer.vga_buf[i].state == OSD_BUF_PROCESS) {
			handle->buffer.vga_buf[i].state = OSD_BUF_CREATE;
		}
		if (i == index) {
			if (handle->buffer.buf[i].state != OSD_BUF_NULL) {
				handle->buffer.buf[i].state = OSD_BUF_PROCESS;
			}
			if (handle->buffer.vga_buf[i].state != OSD_BUF_NULL) {
				handle->buffer.vga_buf[i].state = OSD_BUF_PROCESS;
			}
			osd_debug("[H%d] update buffer index: %d paddr: 0x%llx vaddr: %p\n",
				handle->info.handle_id, i, handle->buffer.buf[i].paddr,
				handle->buffer.buf[i].vaddr);
		}
	}
}

static enum osd_process_type osd_hw_check_limit(struct osd_subdev *subdev,
						struct osd_handle *handle, struct osd_bind *bind)
{
	/* HW in X5 donot limit y */
	if ((subdev->osd_hw_cfg == NULL) ||
		// 1 ||
		subdev->chn_id == OSD_VSE_DS4 ||
		(bind->bind_info.show_en == 0) || (bind->bind_info.osd_level > 0)||
		// (bind->bind_info.start_point.y < subdev->osd_hw_limit_y) ||
		(atomic_read(&subdev->osd_hw_cnt) >= OSD_HW_PROC_NUM)) {
		return OSD_PROC_VGA8;
	} else {
		subdev->osd_hw_limit_y = bind->bind_info.start_point.y + handle->info.size.h;
		atomic_inc(&subdev->osd_hw_cnt);
		return OSD_PROC_HW_VGA8;
	}
}

static void osd_sw_set_process_flag(struct osd_subdev *subdev)
{
	struct osd_bind *bind = NULL, *temp;
	struct vio_osd_info *osd_info;

	osd_info = subdev->osd_info;
	if (!osd_info) {
		osd_err("get osd info fail\n");
		return ;
	}

	if (!list_empty(&subdev->bind_list)) {
		list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
			if (bind && bind->proc_info.proc_type != OSD_PROC_HW_VGA8) {
				atomic_set(&osd_info->need_sw_osd, 1);
				return;
			} else {
				if (!bind) {
					osd_err("bind null\n");
				}
			}
		}
	}

	atomic_set(&osd_info->need_sw_osd, 0);
}

static void osd_hw_set_sta_config(struct osd_subdev *subdev)
{
	struct vse_osd_cfg  *osd_hw_cfg;

	if (subdev->osd_hw_cfg == NULL)
		return;

	spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
	osd_hw_cfg = subdev->osd_hw_cfg;
	memcpy(osd_hw_cfg->osd_sta, subdev->osd_sta.sta_box, MAX_STA_NUM * sizeof(struct osd_sta_box));
	osd_hw_cfg->osd_sta_update = 1;
	memcpy(osd_hw_cfg->osd_sta_level, subdev->osd_sta.sta_level, MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
	osd_hw_cfg->osd_sta_level_update = 1;
	spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
}

void osd_process_addr_inc(struct osd_process_info *proc_info)
{
	switch (proc_info->proc_type)
	{
	case OSD_PROC_VGA8:
		osd_single_buffer_inc_by_addr(proc_info->src_vga_addr);
		break;
	case OSD_PROC_NV12:
		osd_single_buffer_inc_by_addr(proc_info->src_addr);
		break;
	default:
		break;
	}
}

void osd_process_addr_dec(struct osd_process_info *proc_info)
{
	switch (proc_info->proc_type)
	{
	case OSD_PROC_VGA8:
		osd_single_buffer_dec_by_addr(proc_info->src_vga_addr);
		break;
	case OSD_PROC_NV12:
		osd_single_buffer_dec_by_addr(proc_info->src_addr);
		break;
	default:
		break;
	}
}

/* Ioctl ops */
static int32_t osd_create_handle(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_handle *handle, *tmp_handle;
	struct osd_dev *osd_dev;
	void *polygon_buf = NULL;

	osd_dev = osd_ctx->osd_dev;

	handle = kzalloc(sizeof(struct osd_handle), GFP_ATOMIC);
	if (handle == NULL) {
		osd_err("kzalloc failed!\n");
		return -ENOMEM;
	}

	if (copy_from_user((void *)&handle->info, (void __user *)arg, sizeof(struct osd_handle_info))) {
		osd_err("copy_from_user failed!\n");
		ret = -EFAULT;
		goto exit_free;
	}

	if (osd_handle_info_check(&handle->info) < 0) {
		ret = -EINVAL;
		goto exit_free;
	}

	if (handle->info.proc_type <= OSD_PROC_NV12) {
		handle->buffer.size.w = handle->info.size.w;
		handle->buffer.size.h = handle->info.size.h;
		handle->buffer.buf[0].pixel_fmt = (handle->info.proc_type == OSD_PROC_VGA8) ?
						OSD_PIXEL_FORMAT_VGA8 : OSD_PIXEL_FORMAT_NV12;
		handle->buffer.buf[1].pixel_fmt = handle->buffer.buf[0].pixel_fmt;

		ret = osd_buffer_create(osd_dev->ion_client, &handle->buffer);
		if (ret < 0) {
			osd_err("[H%d] alloc buffer size %dx%d failed\n",
				handle->info.handle_id, handle->buffer.size.w, handle->buffer.size.h);
			goto exit_free;
		}
		osd_single_buffer_fill(&handle->buffer.buf[0],
			(handle->info.proc_type == OSD_PROC_VGA8) ?
			handle->info.fill_color :
			g_osd_color.color_map[handle->info.fill_color],
			handle->info.alpha);
		osd_single_buffer_flush(&handle->buffer.buf[0]);
		osd_single_buffer_fill(&handle->buffer.buf[1],
			(handle->info.proc_type == OSD_PROC_VGA8) ?
			handle->info.fill_color :
			g_osd_color.color_map[handle->info.fill_color],
			handle->info.alpha);
		osd_single_buffer_flush(&handle->buffer.buf[1]);
	}

	if (handle->info.proc_type == OSD_PROC_POLYGON) {
		polygon_buf = kzalloc(2 * handle->info.size.h * sizeof(uint32_t), GFP_ATOMIC);
		if (polygon_buf == NULL) {
			osd_err("kzalloc polygon buffer is fail\n");
			ret = -ENOMEM;
			goto exit_destroy;
		}
		ret = copy_from_user(polygon_buf, (void __user *)handle->info.polygon_buf,
					2 * handle->info.size.h * sizeof(uint32_t));
		if (ret) {
			osd_err("copy_from_user failed!\n");
			goto exit_free_polygon;
		}
		handle->info.polygon_buf = (uint32_t *)polygon_buf;
	}

	atomic_set(&handle->bind_cnt, 0);
	atomic_set(&handle->ref_cnt, 1);

	mutex_lock(&osd_dev->osd_list_mutex);
	tmp_handle = osd_find_handle_node(osd_dev, handle->info.handle_id);
	if (tmp_handle != NULL) {
		if (tmp_handle->info.proc_type != handle->info.proc_type) {
			osd_err("[H%d] proc_type: %d %d not match\n",
				tmp_handle->info.handle_id,
				tmp_handle->info.proc_type, handle->info.proc_type);
			ret = -EINVAL;
		} else {
			atomic_inc(&tmp_handle->ref_cnt);
			osd_info("[H%d] already create, count: %d\n",
				handle->info.handle_id,	atomic_read(&tmp_handle->ref_cnt));
		}
		mutex_unlock(&osd_dev->osd_list_mutex);
		goto exit_free_polygon;
	}
	list_add_tail(&handle->node, &osd_dev->osd_list);
	mutex_unlock(&osd_dev->osd_list_mutex);

	// osd_info("[H%d] done\n", handle->info.handle_id);

	return ret;
exit_free_polygon:
	if (polygon_buf != NULL)
		kfree(polygon_buf);
exit_destroy:
	if (handle != NULL) {
		osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
	}
exit_free:
	kfree(handle);

	return ret;
}

static int32_t osd_destroy_handle(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	struct osd_handle *handle;
	struct osd_dev *osd_dev;
	int32_t id = 0;
	int32_t ret = 0;

	osd_dev = osd_ctx->osd_dev;

	ret = get_user(id, (int32_t __user *) arg);
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	// ensure update hw paddr work is done and osd_buffer_destroy can get newest buffer count
	kthread_flush_work(&osd_dev->work);

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", id);
		return -EINVAL;
	}

	if (atomic_read(&handle->ref_cnt) <= 1) {
		if (atomic_read(&handle->bind_cnt) > 0) {
			mutex_unlock(&osd_dev->osd_list_mutex);
			osd_err("[H%d] need detach first\n", id);
			return -EFAULT;
		}

		list_del(&handle->node);
		osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);

		if (handle->info.polygon_buf != NULL)
			kfree(handle->info.polygon_buf);
		kfree(handle);
	} else {
		atomic_dec(&handle->ref_cnt);
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	// osd_info("[H%d] done\n", id);

	return ret;
}

static int32_t osd_get_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_handle_info handle_info;
	struct osd_handle *handle;
	struct osd_dev *osd_dev;

	osd_dev = osd_ctx->osd_dev;

	ret = copy_from_user((void *)&handle_info, (void __user *)arg, sizeof(struct osd_handle_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return ret;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", handle_info.handle_id);
		return -EINVAL;
	}
	ret = copy_to_user((void __user *)arg, (void *)&handle->info, sizeof(struct osd_handle_info));
	if (ret) {
		osd_err("[H%d] copy_to_user failed\n", handle_info.handle_id);
		mutex_unlock(&osd_dev->osd_list_mutex);
		return ret;
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	osd_debug("[H%d] done\n", handle_info.handle_id);

	return ret;
}

static int32_t osd_polygon_check_change(struct osd_handle_info *old_info,
					struct osd_handle_info *new_info)
{
	int32_t i;

	if (old_info->side_num != new_info->side_num) {
		return 1;
	}

	for (i = 0; i < old_info->side_num; i++) {
		if (old_info->point[i].x != new_info->point[i].x)
			return 1;
		if (old_info->point[i].y != new_info->point[i].y)
			return 1;
	}

	return 0;
}

static int32_t osd_set_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_handle_info handle_info;
	struct osd_handle *handle = NULL, *tmp_handle;
	struct osd_dev *osd_dev;
	int32_t need_replace = 0;
	int32_t need_replace_polygon = 0;
	void *polygon_buf = NULL;
	void *polygon_buf_free = NULL;

	osd_dev = osd_ctx->osd_dev;

	ret = copy_from_user((void *)&handle_info, (void __user *)arg, sizeof(struct osd_handle_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	if (osd_handle_info_check(&handle_info) < 0)
		return -EINVAL;

	tmp_handle = kzalloc(sizeof(struct osd_handle), GFP_ATOMIC);
	if (tmp_handle == NULL) {
		osd_err("kzalloc failed\n");
		return -ENOMEM;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", handle_info.handle_id);
		ret = -EINVAL;
		goto exit_free;
	}

	memcpy(&tmp_handle->info, &handle->info, sizeof(struct osd_handle_info));
	atomic_set(&tmp_handle->bind_cnt, atomic_read(&handle->bind_cnt));
	mutex_unlock(&osd_dev->osd_list_mutex);

	if (tmp_handle->info.proc_type != handle_info.proc_type) {
		osd_err("[H%d] proc_type: %d %d not match\n",
			tmp_handle->info.handle_id,
			tmp_handle->info.proc_type, handle_info.proc_type);
		ret = -EINVAL;
		goto exit_free;
	}
	tmp_handle->info.fill_color = handle_info.fill_color;
	tmp_handle->info.proc_type = handle_info.proc_type;
	tmp_handle->info.yuv_bg_transparent = handle_info.yuv_bg_transparent;
	tmp_handle->info.size.w = handle_info.size.w;
	tmp_handle->info.size.h = handle_info.size.h;

	if (handle_info.proc_type <= OSD_PROC_NV12) {
		if ((tmp_handle->info.size.w != handle_info.size.w) ||
			(tmp_handle->info.size.h != handle_info.size.h)) {
			if (atomic_read(&tmp_handle->bind_cnt) == 0) {
				// tmp_handle->info.proc_type = handle_info.proc_type;
				// tmp_handle->info.size.w = handle_info.size.w;
				// tmp_handle->info.size.h = handle_info.size.h;

				//todo: new func: create and fill
				tmp_handle->buffer.size.w = handle_info.size.w;
				tmp_handle->buffer.size.h = handle_info.size.h;
				tmp_handle->buffer.buf[0].pixel_fmt =
					(handle_info.proc_type == OSD_PROC_VGA8) ?
					OSD_PIXEL_FORMAT_VGA8 : OSD_PIXEL_FORMAT_NV12;
				tmp_handle->buffer.buf[1].pixel_fmt =
					(handle_info.proc_type == OSD_PROC_VGA8) ?
					OSD_PIXEL_FORMAT_VGA8 : OSD_PIXEL_FORMAT_NV12;
				ret = osd_buffer_create(osd_dev->ion_client, &tmp_handle->buffer);
				if (ret < 0) {
					goto exit_free;
				}
				osd_single_buffer_fill(&tmp_handle->buffer.buf[0],
					(handle_info.proc_type == OSD_PROC_VGA8) ?
					handle_info.fill_color :
					g_osd_color.color_map[handle_info.fill_color],
					handle_info.alpha);
				osd_single_buffer_flush(&tmp_handle->buffer.buf[0]);
				osd_single_buffer_fill(&tmp_handle->buffer.buf[1],
					(handle_info.proc_type == OSD_PROC_VGA8) ?
					handle_info.fill_color :
					g_osd_color.color_map[handle_info.fill_color],
					handle_info.alpha);
				osd_single_buffer_flush(&tmp_handle->buffer.buf[1]);
				need_replace = 1;
			} else {
				osd_err("[H%d] already bind, cannot modify size\n",
					tmp_handle->info.handle_id);
				ret = -EINVAL;
				goto exit_free;
			}
		}
		// tmp_handle->info.fill_color = handle_info.fill_color;
		// tmp_handle->info.yuv_bg_transparent = handle_info.yuv_bg_transparent;
	}

	if (handle->info.proc_type == OSD_PROC_POLYGON) {
		if (atomic_read(&tmp_handle->bind_cnt) == 0) {
			// polygon_buf_temp = tmp_handle->info.polygon_buf;
			// memcpy(&tmp_handle->info, &handle_info, sizeof(struct osd_handle_info));
			// tmp_handle->info.polygon_buf = polygon_buf_temp;
			if (osd_polygon_check_change(&tmp_handle->info, &handle_info)) {
				tmp_handle->info.side_num = handle_info.side_num;
				memcpy(&tmp_handle->info.point[0], &handle_info.point[0], OSD_POLYGON_MAX_SIDE * sizeof(struct osd_point));
				polygon_buf_free = tmp_handle->info.polygon_buf;
				polygon_buf = kzalloc(2 * handle->info.size.h * sizeof(uint32_t), GFP_ATOMIC);
				if (polygon_buf == NULL) {
					osd_err("kzalloc polygon buffer is fail\n");
					ret = -ENOMEM;
					goto exit_buffer_destroy;
				}
				tmp_handle->info.polygon_buf = (uint32_t *)polygon_buf;
				need_replace_polygon = 1;
			}
			ret = copy_from_user(tmp_handle->info.polygon_buf, (void __user *)handle_info.polygon_buf,
						2 * handle_info.size.h * sizeof(uint32_t));
			if (ret) {
				osd_err("copy_from_user failed!\n");
				goto exit_free_polygon;
			}
		} else {
			osd_err("[H%d] already bind, cannot modify size\n",
				tmp_handle->info.handle_id);
			ret = -EINVAL;
			goto exit_buffer_destroy;
		}
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, handle_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", handle_info.handle_id);
		ret = -EINVAL;
		goto exit_buffer_destroy;
	}

	if (need_replace == 1 || need_replace_polygon == 1) {
		list_replace(&handle->node, &tmp_handle->node);

		osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
		kfree(handle);
		handle = NULL;
		if (polygon_buf_free)
			kfree(polygon_buf_free);
	} else {
		memcpy(&handle->info, &tmp_handle->info, sizeof(struct osd_handle_info));
		if (atomic_read(&handle->bind_cnt) > 0) {
			kthread_queue_work(&osd_dev->worker, &osd_dev->work);
		}
		kfree(tmp_handle);
		tmp_handle = NULL;
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	osd_debug("[H%d] done\n", handle_info.handle_id);

	return ret;
exit_free_polygon:
	if (polygon_buf_free)
		kfree(polygon_buf_free);
exit_buffer_destroy:
	if (tmp_handle != NULL) {
		osd_buffer_destroy(osd_dev->ion_client, &tmp_handle->buffer);
	}
exit_free:
	if (tmp_handle != NULL) {
		kfree(tmp_handle);
		tmp_handle = NULL;
	}

 	return ret;
}

static int32_t osd_get_buffer(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_buffer_info buffer_info;
	struct osd_handle *handle = NULL;
	struct osd_dev *osd_dev;
	struct osd_single_buffer *single_buf;

	osd_dev = osd_ctx->osd_dev;

	ret = copy_from_user((void *)&buffer_info, (void __user *)arg, sizeof(struct osd_buffer_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, buffer_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", buffer_info.handle_id);
		return -EINVAL;
	}

	if (handle->info.proc_type <= OSD_PROC_NV12) {
		buffer_info.proc_type = handle->info.proc_type;
		buffer_info.size.w = handle->buffer.size.w;
		buffer_info.size.h = handle->buffer.size.h;
		if (buffer_info.index < 0) {
			buffer_info.index = handle_find_buffer(handle, OSD_BUF_CREATE, &single_buf, 0);
			if (buffer_info.index < 0) {
				mutex_unlock(&osd_dev->osd_list_mutex);
				osd_err("[H%d] osd find buffer: %d failed\n",
					buffer_info.handle_id, OSD_BUF_CREATE);
				return -ENOBUFS;
			}
		} else {
			single_buf = &handle->buffer.buf[!!buffer_info.index];
		}
		buffer_info.paddr = single_buf->paddr;
		buffer_info.vaddr = single_buf->vaddr;
		buffer_info.share_id = single_buf->share_id;
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	ret = copy_to_user((void __user *)arg, (void *)&buffer_info, sizeof(struct osd_buffer_info));
	if (ret) {
		osd_err("copy_to_user failed\n");
		return -EFAULT;
	}

	// osd_info("[H%d] done\n", buffer_info.handle_id);

	return ret;
}

static int32_t osd_set_buffer(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_buffer_info buffer_info;
	struct osd_handle *handle;
	struct osd_dev *osd_dev;
	struct osd_single_buffer *single_buf;

	osd_dev = osd_ctx->osd_dev;

	ret = copy_from_user((void *)&buffer_info, (void __user *)arg, sizeof(struct osd_buffer_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, buffer_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		osd_err("[H%d] handle was null\n", buffer_info.handle_id);
		return -EINVAL;
	}

	if (handle->info.proc_type <= OSD_PROC_NV12) {
		if (buffer_info.index == -1) {
			buffer_info.index = handle_find_buffer(handle, OSD_BUF_CREATE, &single_buf, 0);
			if (buffer_info.index < 0) {
				mutex_unlock(&osd_dev->osd_list_mutex);
				osd_err("[H%d] find buffer: %d failed\n",
					buffer_info.handle_id, OSD_BUF_CREATE);
				return -EINVAL;
			}
		}

		osd_single_buffer_flush(&handle->buffer.buf[buffer_info.index]);
		if (handle->buffer.vga_buf[buffer_info.index].state != OSD_BUF_NULL) {
			osd_vga4_to_sw(g_osd_color.color_map,
					handle->buffer.buf[buffer_info.index].vaddr,
					handle->buffer.vga_buf[buffer_info.index].vaddr,
					handle->buffer.size.w, handle->buffer.size.h);
			osd_single_buffer_flush(&handle->buffer.vga_buf[buffer_info.index]);
		}

		handle_update_buffer(handle, buffer_info.index);
		if (atomic_read(&handle->bind_cnt) > 0) {
			kthread_queue_work(&osd_dev->worker, &osd_dev->work);
		}
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	// osd_info("[H%d] done\n", buffer_info.handle_id);

	return ret;
}

static int32_t osd_attach(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_bind *bind, *tmp_bind;
	struct osd_handle *handle;
	struct osd_subdev *subdev;
	int32_t buf_index = -1;
	struct osd_single_buffer *single_buf, *vga_buf;

	bind = kzalloc(sizeof(struct osd_bind), GFP_ATOMIC);
	if (bind == NULL) {
		osd_err("kzalloc failed\n");
		return -ENOMEM;
	}

	ret = copy_from_user((void *)&bind->bind_info, (void __user *)arg, sizeof(struct osd_bind_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		goto exit_free_bind;
	}

	if (osd_handle_info_check(&bind->bind_info.handle_info) < 0) {
		ret = -EINVAL;
		goto exit_free_bind;
	}

	mutex_init(&bind->proc_info.proc_mutex);
	atomic_set(&bind->ref_cnt, 1);
	osd_print_bind_info(&bind->bind_info);

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[bind->bind_info.chn_id][bind->bind_info.ctx_id];

	mutex_lock(&subdev->bind_mutex);
	tmp_bind = osd_find_bind_node(subdev, bind->bind_info.handle_id,
					bind->bind_info.buf_layer);
	if (tmp_bind != NULL) {
		atomic_inc(&tmp_bind->ref_cnt);
		osd_info("[H%d][CHN%d][CTX%d] already attach, count: %d\n",
			bind->bind_info.handle_id, bind->bind_info.chn_id,
			bind->bind_info.ctx_id, atomic_read(&tmp_bind->ref_cnt));
		mutex_unlock(&subdev->bind_mutex);
		ret = -EINVAL;
		goto exit_free_bind;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, bind->bind_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		mutex_unlock(&subdev->bind_mutex);
		osd_err("[H%d][CHN%d][CTX%d] handle was null!\n",
			bind->bind_info.handle_id, bind->bind_info.chn_id,
			bind->bind_info.ctx_id);
		ret = -EINVAL;
		goto exit_free_bind;
	}

	mutex_lock(&bind->proc_info.proc_mutex);
	bind->proc_info.proc_type = (handle->info.proc_type >= OSD_PROC_RECT) ?
				bind->bind_info.handle_info.proc_type : handle->info.proc_type;
	if (bind->proc_info.proc_type == OSD_PROC_VGA8) {
		bind->proc_info.proc_type = osd_hw_check_limit(subdev, handle, bind);
	}
	bind->proc_info.subdev = subdev;
	mutex_unlock(&bind->proc_info.proc_mutex);

	if (bind->proc_info.proc_type == OSD_PROC_VGA8) {
		if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
			(handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
			handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
			handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
			ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
			if (ret < 0) {
				mutex_unlock(&osd_dev->osd_list_mutex);
				mutex_unlock(&subdev->bind_mutex);
				goto exit_free_bind;
			}
			buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &single_buf, 0);
			if (buf_index < 0) {
				mutex_unlock(&osd_dev->osd_list_mutex);
				mutex_unlock(&subdev->bind_mutex);
				goto exit_free_bind;
			}
			vga_buf = &handle->buffer.vga_buf[buf_index];
			osd_vga4_to_sw(g_osd_color.color_map, single_buf->vaddr, vga_buf->vaddr,
					handle->buffer.size.w, handle->buffer.size.h);
			osd_single_buffer_flush(single_buf);
		}
	}

	atomic_inc(&handle->bind_cnt);
	mutex_unlock(&osd_dev->osd_list_mutex);

	if ((bind->proc_info.proc_type != OSD_PROC_HW_VGA8)
		&& (bind->bind_info.osd_level == 0)) {
		// hw level: 0, sw level: 1-3
		bind->bind_info.osd_level = 1;
	}
	atomic_set(&bind->need_update, 1);

	list_add_tail(&bind->node, &subdev->bind_list);
	if (bind->proc_info.proc_type == OSD_PROC_HW_VGA8) {
		atomic_set(&subdev->osd_hw_need_update, 1);
	}

	kthread_queue_work(&osd_dev->worker, &osd_dev->work);
	osd_sw_set_process_flag(subdev);
	mutex_unlock(&subdev->bind_mutex);

	// osd_info("[H%d][CHN%d][CTX%d] done\n",
	// 	bind->bind_info.handle_id, bind->bind_info.chn_id,
	// 	bind->bind_info.ctx_id);

	return ret;

exit_free_bind:
	if (bind != NULL)
		kfree(bind);

	return ret;
}

static int32_t osd_detach(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_bind_info bind_info;
	struct osd_bind *bind;
	struct osd_handle *handle;
	struct osd_subdev *subdev;

	ret = copy_from_user((void *)&bind_info, (void __user *)arg, sizeof(struct osd_bind_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return ret;
	}

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[bind_info.chn_id][bind_info.ctx_id];

	mutex_lock(&subdev->bind_mutex);
	bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
	if (bind == NULL) {
		mutex_unlock(&subdev->bind_mutex);
		osd_err("[H%d][CHN%d][CTX%d] bind was null!\n",
			bind_info.handle_id, bind_info.chn_id,
			bind_info.ctx_id);
		return -EINVAL;
	}

	if (atomic_dec_return(&bind->ref_cnt) == 0) {
		list_del(&bind->node);
		if (bind->proc_info.proc_type == OSD_PROC_HW_VGA8) {
			atomic_set(&subdev->osd_hw_need_update, 1);
			atomic_dec(&subdev->osd_hw_cnt);
			kthread_queue_work(&osd_dev->worker, &osd_dev->work);
		}
		osd_sw_set_process_flag(subdev);
		mutex_unlock(&subdev->bind_mutex);

		mutex_lock(&osd_dev->osd_list_mutex);
		handle = osd_find_handle_node(osd_dev, bind_info.handle_id);
		if (handle == NULL) {
			osd_err("[H%d][CHN%d][CTX%d] handle was null!\n",
				bind_info.handle_id, bind_info.chn_id,
				bind_info.ctx_id);
		} else {
			atomic_dec(&handle->bind_cnt);
		}
		mutex_unlock(&osd_dev->osd_list_mutex);

		kfree(bind);
		bind = NULL;
	} else {
		mutex_unlock(&subdev->bind_mutex);
	}

	// osd_info("[H%d][CHN%d][CTX%d] done\n", bind_info.handle_id,
	// 	bind_info.chn_id, bind_info.ctx_id);

	return ret;
}

static int32_t osd_get_bind_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_bind_info bind_info;
	struct osd_bind *bind;
	struct osd_subdev *subdev;

	ret = copy_from_user((void *)&bind_info, (void __user *)arg, sizeof(struct osd_bind_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return ret;
	}

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[bind_info.chn_id][bind_info.ctx_id];

	mutex_lock(&subdev->bind_mutex);
	bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
	if (bind == NULL) {
		mutex_unlock(&subdev->bind_mutex);
		osd_err("[H%d][CHN%d][CTX%d] bind was null!\n",
			bind_info.handle_id, bind_info.chn_id,
			bind_info.ctx_id);
		return -EINVAL;
	}

	ret = copy_to_user((void __user *)arg, (void *)&bind->bind_info, sizeof(struct osd_bind_info));
	if (ret) {
		osd_err("copy_to_user failed\n");
		return ret;
	}
	mutex_unlock(&subdev->bind_mutex);

	// osd_info("[H%d][CHN%d][CTX%d] done\n", bind_info.handle_id,
	// 	bind_info.chn_id, bind_info.ctx_id);

	return ret;
}

static int32_t osd_vga4_check_need_sw_process(struct osd_bind_info *old_info,
						struct osd_bind_info *new_info)
{
	if (new_info->show_en == 0)
		return 1;
	if (new_info->osd_level > 0)
		return 1;
	if (old_info->start_point.y != new_info->start_point.y)
		return 1;

	return 0;
}

static int32_t osd_set_bind_attr(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_bind *bind;
	struct osd_bind_info bind_info;
	struct osd_handle *handle;
	struct osd_subdev *subdev;
	int32_t buf_index = -1;
	struct osd_single_buffer *single_buf, *vga_buf;
	// uint32_t *polygon_buf = NULL;

	ret = copy_from_user((void *)&bind_info, (void __user *)arg, sizeof(struct osd_bind_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	osd_print_bind_info(&bind_info);
	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[bind_info.chn_id][bind_info.ctx_id];

	mutex_lock(&subdev->bind_mutex);
	bind = osd_find_bind_node(subdev, bind_info.handle_id, bind_info.buf_layer);
	if (bind == NULL) {
		mutex_unlock(&subdev->bind_mutex);
		osd_err("[H%d][CHN%d][CTX%d] bind was null!\n",
			bind_info.handle_id, bind_info.chn_id,
			bind_info.ctx_id);
		return -EINVAL;
	}

	mutex_lock(&osd_dev->osd_list_mutex);
	handle = osd_find_handle_node(osd_dev, bind_info.handle_id);
	if (handle == NULL) {
		mutex_unlock(&osd_dev->osd_list_mutex);
		mutex_unlock(&subdev->bind_mutex);
		osd_err("[H%d][CHN%d][CTX%d] handle was null!\n",
			bind_info.chn_id, bind_info.ctx_id,
			bind_info.handle_id);
		return -EINVAL;
	}
	// todo: move to set attr
	// if ((bind->bind_info.handle_info.proc_type == OSD_PROC_POLYGON) &&
	// 	(osd_polygon_check_change(&bind->bind_info, &bind_info))) {
	// 	polygon_buf = kzalloc(2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t), GFP_ATOMIC);
	// 	if (polygon_buf == NULL) {
	// 		mutex_unlock(&osd_dev->osd_list_mutex);
	// 		mutex_unlock(&subdev->bind_mutex);
	// 		osd_err("kzalloc failed\n");
	// 		return -ENOMEM;
	// 	}
	// 	ret = copy_from_user((void *)polygon_buf,
	// 			(void __user *)bind_info.polygon_buf,
	// 			2 * bind->bind_info.handle_info.size.h * sizeof(uint32_t));
	// 	if (ret) {
	// 		mutex_unlock(&osd_dev->osd_list_mutex);
	// 		mutex_unlock(&subdev->bind_mutex);
	// 		osd_err("copy_from_user failed\n");
	// 		goto exit_free;
	// 	}
	// }
	if ((bind->proc_info.proc_type == OSD_PROC_HW_VGA8) &&
		(osd_vga4_check_need_sw_process(&bind->bind_info, &bind_info))) {
		if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
			(handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
			handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
			handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
			ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
			if (ret < 0) {
				mutex_unlock(&osd_dev->osd_list_mutex);
				mutex_unlock(&subdev->bind_mutex);
				return ret;
			}
			buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &single_buf, 0);
			if (buf_index < 0) {
				osd_buffer_destroy_vga(osd_dev->ion_client, &handle->buffer);
				mutex_unlock(&osd_dev->osd_list_mutex);
				mutex_unlock(&subdev->bind_mutex);
				return -EFAULT;
			}
			vga_buf = &handle->buffer.vga_buf[buf_index];
			osd_vga4_to_sw(g_osd_color.color_map, single_buf->vaddr, vga_buf->vaddr,
					handle->buffer.size.w, handle->buffer.size.h);
			osd_single_buffer_flush(vga_buf);
		}

		mutex_lock(&bind->proc_info.proc_mutex);
		bind->proc_info.proc_type = OSD_PROC_VGA8;
		mutex_unlock(&bind->proc_info.proc_mutex);
		atomic_set(&subdev->osd_hw_need_update, 1);
		atomic_dec(&subdev->osd_hw_cnt);
	}
	mutex_unlock(&osd_dev->osd_list_mutex);

	// if (polygon_buf != NULL) {
	// 	if (bind->bind_info.polygon_buf != NULL)
	// 		kfree(bind->bind_info.polygon_buf);
	// } else {
	// 	polygon_buf = bind->bind_info.polygon_buf;
	// }

	if ((bind->proc_info.proc_type != OSD_PROC_HW_VGA8)
		&& (bind->bind_info.osd_level == 0)) {
		// hw level: 0, sw level: 1-3
		bind->bind_info.osd_level = 1;
	}

	memcpy(&bind->bind_info, &bind_info, sizeof(struct osd_bind_info));
	// bind->bind_info.polygon_buf = polygon_buf;
	atomic_set(&bind->need_update, 1);
	kthread_queue_work(&osd_dev->worker, &osd_dev->work);
	mutex_unlock(&subdev->bind_mutex);

	// osd_info("[H%d][CHN%d][CTX%d] done\n", bind_info.handle_id,
	// 	bind_info.chn_id, bind_info.ctx_id);

	return ret;
	// if (polygon_buf != NULL) {
	// 	kfree(polygon_buf);
	// 	polygon_buf = NULL;
	// }
}

static int32_t osd_set_sta(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret, i;
	struct osd_dev *osd_dev;
	struct osd_sta_info sta_info;
	struct osd_subdev *subdev;

	ret = copy_from_user((void *)&sta_info, (void __user *)arg, sizeof(struct osd_sta_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[sta_info.chn_id][sta_info.ctx_id];

	mutex_lock(&subdev->sta_mutex);
	subdev->osd_sta.enable_index = MAX_STA_NUM;
	for (i = MAX_STA_NUM - 1; i >= 0; i--) {
		if (sta_info.sta_box[i].sta_en) {
			subdev->osd_sta.enable_index = i;
			break;
		}
	}

	subdev->osd_sta.buf_layer = sta_info.buf_layer;
	memcpy(subdev->osd_sta.sta_level, sta_info.sta_level, MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
	memcpy(subdev->osd_sta.sta_box, sta_info.sta_box, MAX_STA_NUM * sizeof(struct osd_sta_box));
	subdev->osd_sta.sta_state = OSD_STA_REQUEST;

	if (sta_info.chn_id != OSD_VSE_DS4) {
		osd_hw_set_sta_config(subdev);
		subdev->osd_sta.sta_state = OSD_STA_PROCESS;
	} else {
		atomic_set(&subdev->osd_info->need_sw_osd, 1);
	}
	mutex_unlock(&subdev->sta_mutex);

	osd_debug("[CHN%d][CTX%d] done.\n", sta_info.chn_id, sta_info.ctx_id);

	return ret;
}

static int32_t osd_set_sta_level(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_sta_info sta_info;
	struct osd_subdev *subdev;

	ret = copy_from_user((void *)&sta_info, (void __user *)arg, sizeof(struct osd_sta_info));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[sta_info.chn_id][sta_info.ctx_id];

	mutex_lock(&subdev->sta_mutex);
	memcpy(subdev->osd_sta.sta_level, sta_info.sta_level, MAX_OSD_STA_LEVEL_NUM * sizeof(uint8_t));
	mutex_unlock(&subdev->sta_mutex);

	pr_info("[S%d][C%d] %s done\n", sta_info.chn_id, sta_info.ctx_id, __func__);

	return ret;
}

static int32_t osd_get_sta_bin_value(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret, i;
	struct osd_dev *osd_dev;
	struct osd_sta_bin_info sta_bin_info;
	struct osd_subdev *subdev;
	uint32_t enable_index;
	struct vse_nat_instance *vse_ctx = NULL;

	ret = copy_from_user((void *)&sta_bin_info, (void __user *)arg, sizeof(struct osd_sta_bin_info));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return ret;
	}

	osd_dev = osd_ctx->osd_dev;
	subdev = &osd_dev->subdev[sta_bin_info.chn_id][sta_bin_info.ctx_id];
	enable_index = subdev->osd_sta.enable_index;

	if (subdev->osd_sta.sta_state == OSD_STA_NULL) {
		osd_err("[CHN%d][CTX%d] need set sta first, now state:%d\n",
			sta_bin_info.chn_id, sta_bin_info.ctx_id, subdev->osd_sta.sta_state);
		return -EFAULT;
	}
	if (enable_index == MAX_STA_NUM) {
		osd_warn("[CHN%d][CTX%d] no enable sta\n", sta_bin_info.chn_id, sta_bin_info.ctx_id);
		goto exit_sta_null;
	}

	if (subdev->osd_hw_cfg)
		vse_ctx = container_of(subdev->osd_hw_cfg, struct vse_nat_instance, osd_hw_cfg);

	for (i = 0; i <= OSD_STA_WAIT_CNT; i++) {
		if (sta_bin_info.chn_id != OSD_VSE_DS4) {
			if (subdev->osd_info && subdev->osd_info->get_sta_val) {
				ret = subdev->osd_info->get_sta_val(vse_ctx, sta_bin_info.chn_id, subdev->osd_sta.sta_value);
				if (ret == -EBUSY) {
					osd_debug("sta has not been updated\n");
					msleep(10);
					continue;
				}else if (ret < 0) {
					osd_err("get sta fail!\n");
					goto exit_sta_null;
				}
				osd_debug("get bin value done\n");
				break;
			} else {
				osd_err("get sta null!\n");
				goto exit_sta_null;
			}
		} else {
			kthread_flush_work(&subdev->work);
		}
		if ((subdev->osd_sta.sta_value[enable_index][0] != 0) ||
			(subdev->osd_sta.sta_value[enable_index][1] != 0) ||
			(subdev->osd_sta.sta_value[enable_index][2] != 0) ||
			(subdev->osd_sta.sta_value[enable_index][3] != 0)) {
			break;
		}
		msleep(10);
	}
	if (i == OSD_STA_WAIT_CNT) {
		osd_err("[CHN%d][CTX%d] timeout sta bin was null, now enable_index: %d\n",
			sta_bin_info.chn_id, sta_bin_info.ctx_id, enable_index);
		ret = -ETIMEDOUT;
		goto exit_sta_null;
	}

	mutex_lock(&subdev->sta_mutex);
	memcpy(sta_bin_info.sta_value, (void *)subdev->osd_sta.sta_value, MAX_STA_NUM * MAX_STA_BIN_NUM * sizeof(uint16_t));
	memset((void *)subdev->osd_sta.sta_value, 0, MAX_STA_NUM * MAX_STA_BIN_NUM * sizeof(uint16_t));
	subdev->osd_sta.sta_state = OSD_STA_NULL;
	mutex_unlock(&subdev->sta_mutex);

	mutex_lock(&subdev->bind_mutex);
	osd_sw_set_process_flag(subdev);
	mutex_unlock(&subdev->bind_mutex);

	ret = copy_to_user((void __user *)arg, (void *)&sta_bin_info, sizeof(struct osd_sta_bin_info));
	if (ret) {
		osd_err("copy_to_user failed\n");
		return ret;
	}

	osd_debug("[CHN%d][CTX%d] done\n", sta_bin_info.chn_id, sta_bin_info.ctx_id);

	return ret;

exit_sta_null:
	subdev->osd_sta.sta_state = OSD_STA_NULL;

	mutex_lock(&subdev->bind_mutex);
	osd_sw_set_process_flag(subdev);
	mutex_unlock(&subdev->bind_mutex);

	return ret;
}

static int32_t osd_set_color_map(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_dev *osd_dev;
	struct osd_color_map osd_color;

	ret = copy_from_user((void *)&osd_color, (void __user *)arg, sizeof(struct osd_color_map));
	if (ret) {
		osd_err("copy_from_user failed\n");
		return -EFAULT;
	}

	osd_dev = osd_ctx->osd_dev;

	if (osd_color.color_map_update) {
		memcpy(g_osd_color.color_map, osd_color.color_map, MAX_OSD_COLOR_NUM * sizeof(uint32_t));
		g_osd_color.color_map_update = 1;
		kthread_queue_work(&osd_dev->worker, &osd_dev->work);
	}

	osd_info("done\n");

	return ret;
}

void osd_set_process_handle_info(struct osd_process_info *process_info,
				struct osd_handle *handle)
{
	struct osd_single_buffer *single_buf;
	int32_t index;

	if (process_info->proc_type <= OSD_PROC_NV12) {
		// rect, polygon, mosaic info will be in bind_info
		process_info->fill_color = handle->info.fill_color;
		process_info->width = handle->info.size.w;
		process_info->height = handle->info.size.h;
	}

	switch (process_info->proc_type) {
	case OSD_PROC_HW_VGA8:
	case OSD_PROC_NV12:
		if (process_info->proc_type == OSD_PROC_HW_VGA8) {
			if (process_info->subdev != NULL)
				atomic_set(&process_info->subdev->osd_hw_need_update, 1);
		}

		index = handle_find_buffer(handle, OSD_BUF_PROCESS, &single_buf, 0);
		if (index >= 0) {
			process_info->src_addr = single_buf->vaddr;
			process_info->src_paddr = single_buf->paddr;
		} else {
			osd_err("[H%d] find process buffer failed\n", handle->info.handle_id);
		}

		if (process_info->proc_type == OSD_PROC_NV12)
			process_info->yuv_bg_transparent = handle->info.yuv_bg_transparent;
		break;
	case OSD_PROC_VGA8:
		index = handle_find_buffer(handle, OSD_BUF_PROCESS, &single_buf, 1);
		if (index >= 0) {
			process_info->src_vga_addr = single_buf->vaddr;
		} else {
			osd_err("[H%d] find process buffer failed\n", handle->info.handle_id);
		}
		break;
	// case OSD_PROC_POLYGON:
	// 	process_info->polygon_buf = handle->info.polygon_buf;
	// 	break;
	default:
		break;
	}
}

static int32_t osd_proc_buf(struct osd_video_ctx *osd_ctx, unsigned long arg)
{
	int32_t ret = 0;
	struct osd_proc_buf_info proc_buf;
	struct osd_handle *handle;
	struct osd_dev *osd_dev;
	struct osd_process_info proc_info = {0};
	struct osd_single_buffer *single_buf, *vga_buf;
	int32_t buf_index = -1;

	osd_dev = osd_ctx->osd_dev;

	ret = copy_from_user((void *)&proc_buf, (void __user *)arg, sizeof(struct osd_proc_buf_info));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	proc_info.proc_type = proc_buf.proc_type;
	switch (proc_buf.proc_type)
	{
	case OSD_PROC_NV12:
	case OSD_PROC_VGA8:
		mutex_lock(&osd_dev->osd_list_mutex);
		handle = osd_find_handle_node(osd_dev, proc_buf.handle_id);
		if (handle == NULL) {
			mutex_unlock(&osd_dev->osd_list_mutex);
			pr_err("[H%d] %s handle was null!\n", proc_buf.handle_id, __func__);
			ret = -EINVAL;
			goto exit;
		}

		if (proc_buf.proc_type == OSD_PROC_NV12) {
			proc_info.yuv_bg_transparent = proc_buf.yuv_bg_transparent;
		} else if (proc_buf.proc_type == OSD_PROC_VGA8) {
			if ((handle->buffer.vga_buf[0].state == OSD_BUF_NULL) ||
				(handle->buffer.vga_buf[1].state == OSD_BUF_NULL)) {
				handle->buffer.size.w = handle->info.size.w;
				handle->buffer.size.h = handle->info.size.h;
				handle->buffer.vga_buf[0].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
				handle->buffer.vga_buf[1].pixel_fmt = OSD_PIXEL_FORMAT_SW_VGA4;
				ret = osd_buffer_create_vga(osd_dev->ion_client, &handle->buffer);
				if (ret < 0) {
					mutex_unlock(&osd_dev->osd_list_mutex);
					goto exit;
				}
				buf_index = handle_find_buffer(handle, OSD_BUF_PROCESS, &single_buf, 0);
				if (buf_index < 0) {
					osd_buffer_destroy_vga(osd_dev->ion_client, &handle->buffer);
					mutex_unlock(&osd_dev->osd_list_mutex);
					goto exit;
				}
				vga_buf = &handle->buffer.vga_buf[buf_index];
				osd_vga4_to_sw(g_osd_color.color_map, single_buf->vaddr, vga_buf->vaddr,
						handle->buffer.size.w, handle->buffer.size.h);
				osd_single_buffer_flush(vga_buf);
			}
		}

		osd_set_process_handle_info(&proc_info, handle);
		mutex_unlock(&osd_dev->osd_list_mutex);
		break;
	case OSD_PROC_POLYGON:
	case OSD_PROC_RECT:
	case OSD_PROC_MOSAIC:
		if (proc_buf.proc_type == OSD_PROC_POLYGON) {
			proc_info.polygon_buf = kzalloc(2 * proc_buf.size.h * sizeof(uint32_t), GFP_ATOMIC);
			if (proc_info.polygon_buf == NULL) {
				pr_err("osd kzalloc polygon buffer is failed\n");
				ret = -ENOMEM;
				goto exit;
			}
			ret = (int32_t)copy_from_user((void *)proc_info.polygon_buf,
				(void __user *)proc_buf.polygon_buf,
				2 * proc_buf.size.h * sizeof(uint32_t));
			if (ret) {
				pr_err("%s: copy_from_user failed\n", __func__);
				ret = -EFAULT;
				goto exit;
			}
		}
		if (proc_buf.proc_type == OSD_PROC_RECT || proc_buf.proc_type == OSD_PROC_POLYGON) {
			proc_info.fill_color = proc_buf.fill_color;
		}
		proc_info.width = proc_buf.size.w;
		proc_info.height = proc_buf.size.h;
		break;
	default:
		pr_err("[H%d] %s Invalid type:%d\n", proc_buf.handle_id, __func__, proc_buf.proc_type);
		ret = -EINVAL;
		goto exit;
	}

	proc_info.invert_en = proc_buf.invert_en;
	proc_info.start_x = proc_buf.start_point.x;
	proc_info.start_y = proc_buf.start_point.y;
	proc_info.image_width = proc_buf.image_width;
	proc_info.image_height = proc_buf.image_height;
	proc_info.tar_y_addr = __va(proc_buf.y_paddr);
	proc_info.tar_uv_addr = __va(proc_buf.uv_paddr);
	proc_info.subdev = NULL;

	osd_process_addr_inc(&proc_info);
	osd_run_process(&proc_info);
	osd_process_addr_dec(&proc_info);

	ion_dcache_invalid(proc_buf.y_paddr, proc_buf.image_width * proc_buf.image_height);
	ion_dcache_invalid(proc_buf.uv_paddr, proc_buf.image_width * proc_buf.image_height / 2);
	ion_dcache_flush(proc_buf.y_paddr, proc_buf.image_width * proc_buf.image_height);
	ion_dcache_flush(proc_buf.uv_paddr, proc_buf.image_width * proc_buf.image_height / 2);

	if (proc_buf.proc_type == OSD_PROC_POLYGON) {
		kfree(proc_info.polygon_buf);
		proc_info.polygon_buf = NULL;
	}

	pr_debug("%s handle:%d done\n", __func__, proc_buf.handle_id);

exit:
	return ret;
}

long hb_osd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct osd_video_ctx *osd_ctx;
	int32_t ret = 0;

	osd_ctx = file->private_data;

	if (_IOC_TYPE(cmd) != OSD_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case OSD_IOC_CREATE_HANDLE:
		ret = osd_create_handle(osd_ctx, arg);
		break;
	case OSD_IOC_DESTROY_HANDLE:
		ret = osd_destroy_handle(osd_ctx, arg);
		break;
	case OSD_IOC_GET_ATTR:
		ret = osd_get_attr(osd_ctx, arg);
		break;
	case OSD_IOC_SET_ATTR:
		ret = osd_set_attr(osd_ctx, arg);
		break;
	case OSD_IOC_GET_BUF:
		ret = osd_get_buffer(osd_ctx, arg);
		break;
	case OSD_IOC_SET_BUF:
		ret = osd_set_buffer(osd_ctx, arg);
		break;
	case OSD_IOC_ATTACH:
		ret = osd_attach(osd_ctx, arg);
		break;
	case OSD_IOC_DETACH:
		ret = osd_detach(osd_ctx, arg);
		break;
	case OSD_IOC_GET_BIND_ATTR:
		ret = osd_get_bind_attr(osd_ctx, arg);
		break;
	case OSD_IOC_SET_BIND_ATTR:
		ret = osd_set_bind_attr(osd_ctx, arg);
		break;
	case OSD_IOC_STA:
		ret = osd_set_sta(osd_ctx, arg);
		break;
	case OSD_IOC_STA_LEVEL:
		ret = osd_set_sta_level(osd_ctx, arg);
		break;
	case OSD_IOC_STA_BIN:
		ret = osd_get_sta_bin_value(osd_ctx, arg);
		break;
	case OSD_IOC_COLOR_MAP:
		ret = osd_set_color_map(osd_ctx, arg);
		break;
	case OSD_IOC_PROC_BUF:
		ret = osd_proc_buf(osd_ctx, arg);
		break;
	default:
		pr_err("wrong ioctl cmd: %x for osd\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

int32_t osd_start_worker(struct osd_dev *osd_dev)
{
	int32_t ret = 0;
	struct sched_attr param = {0};

	mutex_lock(&osd_dev->osd_mutex);

	osd_dev->task = kthread_run(kthread_worker_fn, &osd_dev->worker, "osd_work");
	if (IS_ERR(osd_dev->task)) {
		mutex_unlock(&osd_dev->osd_mutex);
		osd_err("failed to create buffer task, err: %ld\n", PTR_ERR(osd_dev->task));
		ret = (int32_t)PTR_ERR(osd_dev->task);
		goto exit;
	}

	param.sched_priority = OSD_TASK_PRIORITY;
	param.sched_policy = SCHED_FIFO;
	ret = sched_setattr_nocheck(osd_dev->task, &param);
	if (ret) {
		mutex_unlock(&osd_dev->osd_mutex);
		pr_err("sched_setattr_nocheck is fail(%d)", ret);
		goto exit;
	}

	osd_dev->task_state = OSD_TASK_START;
	mutex_unlock(&osd_dev->osd_mutex);

	return ret;
exit:
	osd_dev->task = NULL;
	return ret;
}

void osd_stop_worker(struct osd_dev *osd_dev)
{
	struct task_struct *task;

	mutex_lock(&osd_dev->osd_mutex);
	osd_dev->task_state = OSD_TASK_REQUEST_STOP;
	task = osd_dev->task;
	osd_dev->task = NULL;
	kthread_flush_worker(&osd_dev->worker);
	kthread_stop(task);
	osd_dev->task = NULL;
	osd_dev->task_state = OSD_TASK_STOP;
	mutex_unlock(&osd_dev->osd_mutex);
}

static void osd_process_workfunc_done(struct osd_subdev *subdev)
{
	struct osd_dev *osd_dev;
	int32_t chn_id, ctx_id;
	osal_time_t tmp_tv;

	osd_dev = subdev->osd_dev;
	chn_id = subdev->osd_info->chn_id;
	ctx_id = subdev->osd_info->ctx_id;

	osal_time_get(&tmp_tv);
	g_osd_idx[chn_id][ctx_id]++;
	if (tmp_tv.tv_sec > g_osd_fps_lasttime[chn_id][ctx_id]) {
		g_osd_fps[chn_id][ctx_id] = g_osd_idx[chn_id][ctx_id];
		g_osd_fps_lasttime[chn_id][ctx_id] = tmp_tv.tv_sec;
		g_osd_idx[chn_id][ctx_id] = 0;
	}

	osd_queue_done(&subdev->queue);

	if (atomic_dec_return(&subdev->osd_info->frame_count) > 0)
		kthread_queue_work(&osd_dev->worker, &subdev->work);
}

static void osd_frame_process_workfunc(struct kthread_work *work)
{
	int32_t i;
	struct osd_bind *bind, *temp;
	struct osd_subdev *subdev;
	struct osd_dev *osd_dev;
	struct vio_frame *vio_frame;
	struct osd_process_info *process_info;

	subdev = container_of(work, struct osd_subdev, work);
	osd_dev = subdev->osd_dev;

	vio_frame = osd_queue_pop(&subdev->queue);
	if (vio_frame == NULL) {
		osd_err("pop frame fail\n");
		goto exit;
	}

	// osd_debug("frame id: %d, index: %d\n", vio_frame->frameinfo.frameid.frame_id,
	// 	vio_frame->frameinfo.bufferindex);

	mutex_lock(&subdev->sta_mutex);
	if (subdev->osd_sta.sta_state == OSD_STA_REQUEST) {
		subdev->osd_sta.sta_state = OSD_STA_PROCESS;
		for (i = 0; i < MAX_STA_NUM; i++) {
			if (osd_dev->task_state == OSD_TASK_REQUEST_STOP ||
				osd_dev->task_state == OSD_TASK_STOP) {
				osd_debug("task request stop, exit\n");
				break;
			}
			if (subdev->osd_sta.sta_box[i].sta_en) {
				// it means sw process sta
				process_info = &subdev->osd_sta.sta_proc[i];
				process_info->subdev = subdev;
				process_info->width = subdev->osd_sta.sta_box[i].width;
				process_info->height = subdev->osd_sta.sta_box[i].height;
				process_info->start_x = subdev->osd_sta.sta_box[i].start_x;
				process_info->start_y = subdev->osd_sta.sta_box[i].start_y;
				process_info->sta_level = subdev->osd_sta.sta_level;
				process_info->sta_bin_value = (uint16_t *)subdev->osd_sta.sta_value[i];

				process_info->frame_id = vio_frame->frameinfo.frameid.frame_id;
				process_info->buffer_index = vio_frame->frameinfo.bufferindex;
				process_info->image_width = vio_frame->vbuf.group_info.info[0].buf_attr.width;
				process_info->image_height = vio_frame->vbuf.group_info.info[0].buf_attr.height;
				process_info->tar_y_addr = __va(vio_frame->vbuf.group_info.info[0].paddr[0]);
				process_info->tar_uv_addr = __va(vio_frame->vbuf.group_info.info[0].paddr[1]);
				process_info->proc_type = OSD_PROC_STA;

				osd_run_process(process_info);
			}
			subdev->osd_sta.sta_state = OSD_STA_DONE;
		}
		memset(subdev->osd_sta.sta_box, 0, MAX_STA_NUM * sizeof(struct osd_sta_box));
	}
	mutex_unlock(&subdev->sta_mutex);

	mutex_lock(&subdev->bind_mutex);
	for (i = 0; i < OSD_LEVEL_NUM; i++) {
		if (osd_dev->task_state == OSD_TASK_REQUEST_STOP ||
			osd_dev->task_state == OSD_TASK_STOP) {
			osd_debug("task request stop, exit\n");
			break;
		}

		list_for_each_entry_safe(bind, temp, &subdev->bind_list, node) {
			mutex_lock(&bind->proc_info.proc_mutex);
			if ((bind->proc_info.show_en == 0) || (bind->proc_info.osd_level != i)) {
				mutex_unlock(&bind->proc_info.proc_mutex);
				continue;
			}

			bind->proc_info.frame_id = vio_frame->frameinfo.frameid.frame_id;
			bind->proc_info.buffer_index = vio_frame->frameinfo.bufferindex;
			bind->proc_info.image_width = vio_frame->vbuf.group_info.info[0].buf_attr.width;
			bind->proc_info.image_height = vio_frame->vbuf.group_info.info[0].buf_attr.height;
			bind->proc_info.tar_y_addr = __va(vio_frame->vbuf.group_info.info[0].paddr[0]);
			bind->proc_info.tar_uv_addr = __va(vio_frame->vbuf.group_info.info[0].paddr[1]);

			if (bind->proc_info.proc_type != OSD_PROC_HW_VGA8) {
				osd_process_addr_inc(&bind->proc_info);
				osd_run_process(&bind->proc_info);
				osd_process_addr_dec(&bind->proc_info);
			}
			mutex_unlock(&bind->proc_info.proc_mutex);
		}
	}
	mutex_unlock(&subdev->bind_mutex);

	osd_process_workfunc_done(subdev);

exit:
	if (!subdev->osd_info && !subdev->osd_info->return_frame) {
		osd_err("return frame func null\n");
		return;
	}
	subdev->osd_info->return_frame(subdev->osd_info, (struct vio_frame *)vio_frame);
}

static int32_t osd_subdev_init(struct osd_dev *osd)
{
	int32_t i = 0, j = 0;
	struct osd_subdev *subdev;
	struct vse_nat_instance *vse_ctx = NULL;

	for (i = 0; i < OSD_CHN_MAX; i++) {
		for (j = 0; j < VIO_MAX_STREAM; j++) {
			subdev = &osd->subdev[i][j];
			subdev->osd_dev = osd;
			subdev->chn_id = i;
			subdev->ctx_id = j;
			osd_queue_init(&subdev->queue);

			INIT_LIST_HEAD(&subdev->bind_list);
			INIT_LIST_HEAD(&subdev->input_frame_list);
			mutex_init(&subdev->bind_mutex);
			mutex_init(&subdev->sta_mutex);
			spin_lock_init(&subdev->frame_slock);
			atomic_set(&subdev->osd_hw_need_update, 0);
			atomic_set(&subdev->osd_hw_cnt, 0);
			kthread_init_work(&subdev->work, osd_frame_process_workfunc);

			subdev->osd_info = osd_get_info(i, j);
			if (subdev->osd_info) {
				vse_ctx = container_of(subdev->osd_info, struct vse_nat_instance, osd_info);
				if (!vse_ctx) {
					pr_err("get vse ctx fail\n");
					return -1;
				}
				subdev->osd_hw_cfg = &vse_ctx->osd_hw_cfg;
			} else {
				pr_err("chn %d ctx %d get osd info fail\n", i, j);
			}
		}
	}

	return 0;
}

static void osd_subdev_clear(struct osd_dev *osd_dev)
{
	int32_t i = 0, j = 0, m = 0;
	struct osd_subdev *subdev;
	struct osd_bind *bind;
	struct vio_frame *frame;
	unsigned long flags;

	for (i = 0; i < OSD_CHN_MAX; i++) {
		for (j = 0; j < VIO_MAX_STREAM; j++) {
			subdev = &osd_dev->subdev[i][j];

			if (subdev->osd_info != NULL) {
				atomic_set(&subdev->osd_info->need_sw_osd, 0);
				atomic_set(&subdev->osd_info->frame_count, 0);
			}
			osd_queue_destroy(&subdev->queue);

			if (subdev->osd_hw_cfg != NULL) {
				spin_lock(&subdev->osd_hw_cfg->osd_cfg_slock);
				for (m = 0; m < OSD_HW_PROC_NUM; m++) {
					memset(&subdev->osd_hw_cfg->osd_box[m], 0, sizeof(struct osd_box));
					if (subdev->osd_hw_cfg->osd_buf[m] != 0) {
						osd_single_buffer_dec_by_paddr(subdev->osd_hw_cfg->osd_buf[m]);
						subdev->osd_hw_cfg->osd_buf[m] = 0;
					}
				}
				for (m = 0; m < MAX_STA_NUM; m++) {
					memset(&subdev->osd_hw_cfg->osd_sta[m], 0, sizeof(struct osd_sta_box));
				}
				for (m = 0; m < MAX_OSD_STA_LEVEL_NUM; m++) {
					subdev->osd_hw_cfg->osd_sta_level[m] = 0;
				}
				subdev->osd_hw_cfg->osd_box_update = 1;
				subdev->osd_hw_cfg->osd_buf_update = 1;
				subdev->osd_hw_cfg->osd_sta_update = 1;
				subdev->osd_hw_cfg->osd_sta_level_update = 1;
				subdev->osd_hw_limit_y = 0;
				atomic_set(&subdev->osd_hw_cnt, 0);
				atomic_set(&subdev->osd_hw_need_update, 0);
				spin_unlock(&subdev->osd_hw_cfg->osd_cfg_slock);
			}

			mutex_lock(&subdev->bind_mutex);
			while (!list_empty(&subdev->bind_list)) {
				bind = list_first_entry(&subdev->bind_list, struct osd_bind, node);
				list_del(&bind->node);
				if (bind->proc_info.polygon_buf != NULL) {
					kfree(bind->proc_info.polygon_buf);
					bind->proc_info.polygon_buf = NULL;
				}
				kfree(bind);
			}
			mutex_unlock(&subdev->bind_mutex);

			mutex_lock(&subdev->sta_mutex);
			memset(&subdev->osd_sta, 0, sizeof(struct osd_sta));
			mutex_unlock(&subdev->sta_mutex);

			spin_lock_irqsave(&subdev->frame_slock, flags);
			while (!list_empty(&subdev->queue.incoming_list)) {
				frame = osd_queue_pop(&subdev->queue);
				if (frame) {
					subdev->osd_info->return_frame(subdev->osd_info, frame);
				} else {
					osd_err("pop fail\n");
					break;
				}
			}
			spin_unlock_irqrestore(&subdev->frame_slock, flags);

			g_osd_fps[i][j] = 0;
		}
	}
}

int32_t hb_osd_open(struct inode *inode, struct file *file)
{
	struct osd_video_ctx *osd_ctx;
	struct osd_dev *osd_dev;
	int32_t ret = 0;

	osd_dev = container_of(inode->i_cdev, struct osd_dev, cdev);
	osd_ctx = kzalloc(sizeof(struct osd_video_ctx), GFP_ATOMIC);
	if (osd_ctx == NULL) {
		osd_err("kzalloc failed\n");
		return -ENOMEM;
	}

	osd_ctx->osd_dev = osd_dev;
	file->private_data = osd_ctx;

	if (atomic_inc_return(&osd_dev->open_cnt) == 1) {
		ret = osd_subdev_init(osd_dev);
		if (ret < 0)
			goto exit_free;

		ret = osd_start_worker(osd_dev);
		if (ret < 0)
			goto exit_free;
	}

	osd_debug("open done, open count: %d\n", atomic_read(&osd_dev->open_cnt));

	return ret;
exit_free:
	kfree(osd_ctx);
	atomic_dec(&osd_dev->open_cnt);
	osd_dev->task = NULL;

	return ret;
}

ssize_t hb_osd_write(struct file *file, const char __user * buf,
			size_t count, loff_t * ppos)
{
	return 0;
}

ssize_t hb_osd_read(struct file *file, char __user * buf, size_t size,
			loff_t * ppos)
{
	return 0;
}

static void osd_handle_clear(struct osd_dev *osd_dev)
{
	struct osd_handle *handle;

	mutex_lock(&osd_dev->osd_list_mutex);
	while (!list_empty(&osd_dev->osd_list)) {
		handle = list_first_entry(&osd_dev->osd_list, struct osd_handle, node);
		list_del(&handle->node);

		osd_buffer_destroy(osd_dev->ion_client, &handle->buffer);
		kfree(handle);
	}
	mutex_unlock(&osd_dev->osd_list_mutex);
}

int32_t hb_osd_close(struct inode *inode, struct file *file)
{
	struct osd_video_ctx *osd_ctx;
	struct osd_dev *osd_dev;
	int32_t ret = 0;

	osd_ctx = file->private_data;
	if (osd_ctx == NULL) {
		pr_err("osd ctx was null\n");
		goto exit;
	}

	osd_dev = osd_ctx->osd_dev;

	if (atomic_dec_return(&osd_dev->open_cnt) == 0) {
		osd_stop_worker(osd_dev);

		osd_subdev_clear(osd_dev);
		osd_handle_clear(osd_dev);
	}

	if (osd_ctx) {
		kfree(osd_ctx);
		osd_ctx = NULL;
	}

	osd_debug("close done, open count: %d\n", atomic_read(&osd_dev->open_cnt));
exit:
	return ret;
}
