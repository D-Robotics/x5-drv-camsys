/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_OPS_H__
#define __HOBOT_OSD_OPS_H__

#include "hobot_osd_dev.h"

#define OSD_QUEUE_SIZE 6
#define OSD_STA_WAIT_CNT 30

struct osd_frame_queue;
struct osd_dev;
struct osd_process_info;
struct osd_handle;


void osd_queue_init(struct osd_frame_queue *queue);
void osd_queue_destroy(struct osd_frame_queue *queue);
int32_t osd_queue_push(struct osd_frame_queue *queue, struct vio_frame *vio_frame);
struct vio_frame *osd_queue_pop(struct osd_frame_queue *queue);
int32_t osd_queue_done(struct osd_frame_queue *queue);

struct osd_handle *osd_find_handle_node(struct osd_dev *osd_dev, int32_t id);
void osd_set_process_handle_info(struct osd_process_info *process_info, struct osd_handle *handle);

long hb_osd_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int32_t hb_osd_open(struct inode *inode, struct file *file);

ssize_t hb_osd_write(struct file *file, const char __user * buf, size_t count, loff_t * ppos);
ssize_t hb_osd_read(struct file *file, char __user * buf, size_t size, loff_t * ppos);
int32_t hb_osd_close(struct inode *inode, struct file *file);

void osd_process_addr_inc(struct osd_process_info *proc_info);
void osd_process_addr_dec(struct osd_process_info *proc_info);

int32_t osd_start_worker(struct osd_dev *osd_dev);
void osd_stop_worker(struct osd_dev *osd_dev);

#endif // __HOBOT_OSD_OPS_H__
