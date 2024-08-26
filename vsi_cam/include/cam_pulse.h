/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CAM_PULSE_H_
#define _CAM_PULSE_H_

struct cam_pulse_device;

int start_cam_pulse_gen(struct cam_pulse_device *dev);
int stop_cam_pulse_gen(struct cam_pulse_device *dev);
void put_cam_pulse_device(struct cam_pulse_device *dev);

#endif /* _CAM_PULSE_H_ */
