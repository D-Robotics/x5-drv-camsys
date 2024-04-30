/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VID_DRV_H_
#define _VID_DRV_H_

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>

#define VID_DEV_NAME        "vs-video"

struct vid_device {
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier sd_notifier;
	struct list_head video_device_list;
};

#endif /* _VID_DRV_H_ */
