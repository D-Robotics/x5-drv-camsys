/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_MEM_H__
#define __HOBOT_OSD_MEM_H__

#include <linux/ion.h>
#include <linux/list.h>
#include <linux/types.h>

#include "osd_config.h"
#include "vio_framemgr.h"

#define OSD_PP_BUF 2
#define OSD_WAIT_CNT 30
#define OSD_ION_TYPE 20

enum osd_buf_state {
	OSD_BUF_NULL,
	OSD_BUF_CREATE,
	OSD_BUF_PROCESS,
	OSD_BUF_USER,
};

enum osd_pixel_format {
	OSD_PIXEL_FORMAT_NULL,
	OSD_PIXEL_FORMAT_VGA4,
	OSD_PIXEL_FORMAT_NV12,
	OSD_PIXEL_FORMAT_SW_VGA4,
	OSD_PIXEL_FORMAT_POLYGON,
};

struct osd_single_buffer {
	struct list_head node;
	uint32_t share_id;
	// int32_t fd;
	ssize_t length;
	uint8_t *vaddr;
	uint64_t paddr;
	struct ion_handle *ion_handle;
	enum osd_buf_state state;
	enum osd_pixel_format pixel_fmt;
	atomic_t ref_count;
};

struct osd_buffer {
	struct osd_size size;
	struct osd_single_buffer buf[OSD_PP_BUF];
	struct osd_single_buffer vga_buf[OSD_PP_BUF];
};

void ion_dcache_invalid(phys_addr_t paddr, size_t size);
void ion_dcache_flush(phys_addr_t paddr, size_t size);
void osd_single_buffer_inc(struct osd_single_buffer *one_buffer);
void osd_single_buffer_inc_by_addr(uint8_t *vaddr);
void osd_single_buffer_inc_by_paddr(uint64_t paddr);
void osd_single_buffer_dec(struct osd_single_buffer *one_buffer);
void osd_single_buffer_dec_by_addr(uint8_t *vaddr);
void osd_single_buffer_dec_by_paddr(uint64_t paddr);
int32_t osd_single_buffer_create(struct ion_client *client, struct osd_single_buffer *one_buffer);
void osd_single_buffer_destroy(struct ion_client *client, struct osd_single_buffer *buf);
void osd_single_buffer_flush(struct osd_single_buffer *single_buffer);
void osd_single_buffer_fill(struct osd_single_buffer *one_buffer, uint32_t color);

int32_t osd_buffer_create(struct ion_client *client, struct osd_buffer *osd_buffer);
int32_t osd_buffer_create_vga(struct ion_client *client, struct osd_buffer *osd_buffer);
void osd_buffer_destroy(struct ion_client *client, struct osd_buffer *osd_buffer);
void osd_buffer_destroy_vga(struct ion_client *client, struct osd_buffer *osd_buffer);
void osd_buffer_flush(struct osd_buffer *osd_buffer);

struct vio_frame *osd_get_frame(struct list_head *list, int32_t frame_index);
void osd_put_input_frame(struct list_head *list, struct vio_frame *frame);
struct vio_frame *osd_get_input_frame(struct list_head *list);

#endif // __HOBOT_OSD_MEM_H__
