/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VIDEO_H_
#define _VIDEO_H_

#include "vid_drv.h"

int create_default_links(struct vid_device *vdev);
void destroy_links(struct vid_device *vdev);
int vid_subdev_set_cap(struct vid_device *vdev);

#endif /* _VIDEO_H_ */
