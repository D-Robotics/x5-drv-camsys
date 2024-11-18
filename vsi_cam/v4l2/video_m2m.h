/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VIDEO_M2M_H_
#define _VIDEO_M2M_H_

#include "vid_drv.h"

struct vid_m2m_video_device {
	struct video_device video;
	struct list_head entry;
};

struct vid_m2m_video_device *create_m2m_video_device(struct vid_device *vdev, u32 id);
void destroy_m2m_video_device(struct vid_m2m_video_device *dev);

#endif /* _VIDEO_M2M_H_ */
