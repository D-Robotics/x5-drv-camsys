
/**
 * @file: vio_debug_dev.h
 * @
 * @NO{S09E05C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef VIO_DEBUG_DEV_H
#define VIO_DEBUG_DEV_H

s32 vio_debug_create(struct device *dev);
void vio_debug_destroy(struct device *dev);
s32 vpf_create_debug_file(struct hobot_vpf_dev *vpf_dev);
void vpf_destroy_debug_file(struct hobot_vpf_dev *vpf_dev);

#endif