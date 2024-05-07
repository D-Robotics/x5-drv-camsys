/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt) "[hobot_osd](%s): " fmt, __func__

#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include "hobot_osd_mem.h"
#include "vio_config.h"
#include "hobot_osd_dev.h"

struct list_head s_osd_buf_list = LIST_HEAD_INIT(s_osd_buf_list);
spinlock_t s_osd_buf_slock = __SPIN_LOCK_UNLOCKED(s_osd_buf_slock);

int32_t osd_ion_alloc(struct ion_client *client, struct osd_single_buffer *buf)
{
	int32_t ret = 0;

	buf->ion_handle = ion_alloc(client, buf->length, PAGE_SIZE, ION_HEAP_CARVEOUT_MASK,
		(OSD_ION_TYPE << 16) | ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC);
	if (IS_ERR(buf->ion_handle)) {
		pr_err("osd ion alloc failed\n");
		return -ENOMEM;
	}
	ret = ion_phys(client, buf->ion_handle->id, &buf->paddr, &buf->length);
	if (ret) {
		pr_err("osd ion get phys addr failed\n");
		goto _exit;
	}
	buf->vaddr = ion_map_kernel(client, buf->ion_handle);
	if (IS_ERR(buf->vaddr)) {
		pr_err("osd ion map failed\n");
		goto _exit;
	}

	pr_debug("osd alloc buffer paddr:0x%llx vaddr:%p length:%ld\n",
		buf->paddr, buf->vaddr, buf->length);

	return 0;
_exit:
	ion_free(client, buf->ion_handle);
	buf->ion_handle = NULL;
	buf->paddr = 0;
	buf->vaddr = NULL;

	return -ENOMEM;
}

void osd_single_buffer_inc(struct osd_single_buffer *buf)
{
	atomic_inc(&buf->ref_count);
}

void osd_single_buffer_inc_by_addr(uint8_t *vaddr)
{
	struct osd_single_buffer *buf, *temp;

	spin_lock(&s_osd_buf_slock);
	list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
		if (buf->vaddr == vaddr) {
			atomic_inc(&buf->ref_count);
		}
	}
	spin_unlock(&s_osd_buf_slock);
}

void osd_single_buffer_inc_by_paddr(uint64_t paddr)
{
	struct osd_single_buffer *buf, *temp;

	spin_lock(&s_osd_buf_slock);
	list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
		if (buf->paddr == paddr) {
			atomic_inc(&buf->ref_count);
			break;
		}
	}
	spin_unlock(&s_osd_buf_slock);
}

void osd_single_buffer_dec(struct osd_single_buffer *buf)
{
	atomic_dec(&buf->ref_count);
}

void osd_single_buffer_dec_by_addr(uint8_t *vaddr)
{
	struct osd_single_buffer *buf, *temp;

	spin_lock(&s_osd_buf_slock);
	list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
		if (buf->vaddr == vaddr) {
			atomic_dec(&buf->ref_count);
		}
	}
	spin_unlock(&s_osd_buf_slock);
}

void osd_single_buffer_dec_by_paddr(uint64_t paddr)
{
	struct osd_single_buffer *buf, *temp;

	spin_lock(&s_osd_buf_slock);
	list_for_each_entry_safe(buf, temp, &s_osd_buf_list, node) {
		if (buf->paddr == paddr) {
			atomic_dec(&buf->ref_count);
			break;
		}
	}
	spin_unlock(&s_osd_buf_slock);
}

int32_t osd_single_buffer_create(struct ion_client *client, struct osd_single_buffer *buf)
{
	if (osd_ion_alloc(client, buf) < 0) {
		return -ENOMEM;
	}

	buf->state = OSD_BUF_CREATE;
	atomic_set(&buf->ref_count, 0);

	spin_lock(&s_osd_buf_slock);
	list_add_tail(&buf->node, &s_osd_buf_list);
	spin_unlock(&s_osd_buf_slock);

	pr_debug("osd create buffer format:%d, paddr:0x%llx addr:%p length:%ld\n",
		buf->pixel_fmt, buf->paddr, buf->vaddr, buf->length);

	return 0;
}

void osd_single_buffer_destroy(struct ion_client *client, struct osd_single_buffer *buf)
{
	int32_t i;

	for (i = OSD_WAIT_CNT; i > 0; i--) {
		// wait for no thread use this buffer
		if (atomic_read(&buf->ref_count) == 0) {
			break;
		}
		msleep(10);
		if (!i) {
			pr_err("osd one buffer paddr:0x%llx desroyed, but ref count is %d\n",
			buf->paddr, atomic_read(&buf->ref_count));
		}
	}
	spin_lock(&s_osd_buf_slock);
	list_del(&buf->node);
	spin_unlock(&s_osd_buf_slock);

	if (buf->ion_handle != NULL) {
		pr_debug("osd destroy buffer format:%d, paddr:0x%llx addr:%p length:%ld\n",
			buf->pixel_fmt, buf->paddr, buf->vaddr, buf->length);
		ion_free(client, buf->ion_handle);
		buf->ion_handle = NULL;
	}
	buf->paddr = 0;
	buf->vaddr = NULL;
	buf->length = 0;
	buf->pixel_fmt = OSD_PIXEL_FORMAT_NULL;
	buf->state = OSD_BUF_NULL;
}

void ion_dcache_invalid(phys_addr_t paddr, size_t size)
{
	// dma_sync_single_for_cpu(NULL, paddr, size, DMA_FROM_DEVICE);
	// __inval_dcache_area(phys_to_virt(paddr), size);
}

void ion_dcache_flush(phys_addr_t paddr, size_t size)
{
	// dma_sync_single_for_device(NULL, paddr, size, DMA_TO_DEVICE);
	// __flush_dcache_area(phys_to_virt(paddr), size);
}

void osd_single_buffer_flush(struct osd_single_buffer *single_buffer)
{
	ion_dcache_invalid(single_buffer->paddr, single_buffer->length);
	ion_dcache_flush(single_buffer->paddr, single_buffer->length);
}

void osd_single_buffer_fill(struct osd_single_buffer *single_buffer, uint32_t color)
{
	uint32_t size = 0;
	uint8_t y_color = 0, u_color = 0, v_color = 0;
	uint8_t *addr = NULL;
	int32_t i = 0;

	if (single_buffer->pixel_fmt == OSD_PIXEL_FORMAT_VGA4) {
		memset(single_buffer->vaddr, (color << 4) | color, single_buffer->length);
	} else if (single_buffer->pixel_fmt == OSD_PIXEL_FORMAT_NV12) {
		y_color = (color >> 16) & 0xff;
		u_color = (color >> 8) & 0xff;
		v_color = color & 0xff;
		size = single_buffer->length * 2 / 3; // get (w * h) size
		addr = single_buffer->vaddr + size;
		memset(single_buffer->vaddr, y_color, size);
		for (i = 0; i < size / 2; i += 2) {
			addr[0] = u_color;
			addr[1] = v_color;
			addr += 2;
		}
	}
}

static uint32_t osd_get_buffer_length(uint32_t width, uint32_t height,
					enum osd_pixel_format pixel_fmt)
{
	if (pixel_fmt == OSD_PIXEL_FORMAT_VGA4) {
		return (width * height / 2);
	} else if (pixel_fmt == OSD_PIXEL_FORMAT_NV12) {
		return width * height * 3 / 2;
	} else if (pixel_fmt == OSD_PIXEL_FORMAT_SW_VGA4) {
		return width * height * 3;
	} else if (pixel_fmt == OSD_PIXEL_FORMAT_POLYGON) {
		return (2 * height * sizeof(uint32_t));
	} else {
		return 0;
	}
}

int32_t osd_buffer_create(struct ion_client *client, struct osd_buffer *osd_buffer)
{
	int32_t i;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if ((osd_buffer->buf[i].state == OSD_BUF_NULL) && (osd_buffer->buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
			osd_buffer->buf[i].length = osd_get_buffer_length(osd_buffer->size.w, osd_buffer->size.h, osd_buffer->buf[i].pixel_fmt);
			if ((osd_single_buffer_create(client, &osd_buffer->buf[i])) < 0) {
				goto _exit;
			}
			osd_buffer->buf[i].state = OSD_BUF_CREATE + i;
		}
		if ((osd_buffer->vga_buf[i].state == OSD_BUF_NULL) && (osd_buffer->vga_buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
			osd_buffer->vga_buf[i].length = osd_get_buffer_length(osd_buffer->size.w, osd_buffer->size.h, osd_buffer->vga_buf[i].pixel_fmt);
			if ((osd_single_buffer_create(client, &osd_buffer->vga_buf[i])) < 0) {
				goto _exit;
			}
			osd_buffer->vga_buf[i].state = OSD_BUF_CREATE;
		}
	}

	return 0;
_exit:
	osd_buffer_destroy(client, osd_buffer);
	return -1;
}

int32_t osd_buffer_create_vga(struct ion_client *client, struct osd_buffer *osd_buffer)
{
	size_t length;
	int32_t ret = 0, i;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if ((osd_buffer->vga_buf[i].state == OSD_BUF_NULL) &&
			(osd_buffer->vga_buf[i].pixel_fmt != OSD_PIXEL_FORMAT_NULL)) {
			length = osd_get_buffer_length(osd_buffer->size.w, osd_buffer->size.h,
			osd_buffer->vga_buf[i].pixel_fmt);
			osd_buffer->vga_buf[i].length = length;
			ret = osd_single_buffer_create(client, &osd_buffer->vga_buf[i]);
			if (ret < 0) {
				goto exit;
			}
			osd_buffer->vga_buf[i].state = osd_buffer->buf[i].state;
		}
	}

	return ret;

exit:
	osd_buffer_destroy_vga(client, osd_buffer);

	return ret;
}

void osd_buffer_destroy(struct ion_client *client, struct osd_buffer *osd_buffer)
{
	int32_t i;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if (osd_buffer->buf[i].state != OSD_BUF_NULL) {
			osd_single_buffer_destroy(client, &osd_buffer->buf[i]);
		}
		if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
			osd_single_buffer_destroy(client, &osd_buffer->vga_buf[i]);
		}
	}
}

void osd_buffer_destroy_vga(struct ion_client *client, struct osd_buffer *osd_buffer)
{
	int32_t i;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
			osd_single_buffer_destroy(client, &osd_buffer->vga_buf[i]);
		}
	}
}

void osd_buffer_flush(struct osd_buffer *osd_buffer)
{
	int32_t i = 0;

	for (i = 0; i < OSD_PP_BUF; i++) {
		if (osd_buffer->buf[i].state != OSD_BUF_NULL) {
			osd_single_buffer_flush(&osd_buffer->buf[i]);
		}
		if (osd_buffer->vga_buf[i].state != OSD_BUF_NULL) {
			osd_single_buffer_flush(&osd_buffer->vga_buf[i]);
		}
	}
}

struct vio_frame *osd_get_frame(struct list_head *list, int32_t frame_index)
{
	struct vio_frame *frame, *temp;

	list_for_each_entry_safe(frame, temp, list, list) {
		if (frame->frameinfo.bufferindex == frame_index) {
			list_del(&frame->list);
			frame->state = FS_INVALID;
			return frame;
		}
	}

	return NULL;
}

// void osd_put_input_frame(struct list_head *list, struct vio_frame *frame)
// {
// 	list_add_tail(&frame->list, list);
// }

// struct vio_frame *osd_get_input_frame(struct list_head *list)
// {
// 	struct vio_frame *frame;

// 	frame = list_first_entry(list, struct vio_frame, list);
// 	list_del(&frame->list);

// 	return frame;
// }
