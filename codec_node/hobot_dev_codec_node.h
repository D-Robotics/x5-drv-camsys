/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef HOBOT_DEV_CODEC_NODE_H
#define HOBOT_DEV_CODEC_NODE_H

#include <linux/cdev.h>
#include <linux/interrupt.h>
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_node_api.h"
#include "hobot_codec_common.h"
#include "codec_node_config.h"

/**
 * @struct codec_node_subdev
 * @brief VIN_NODE sub-device definition.
 * @NO{S10E01C01}
 */
struct codec_node_subdev {
     /**
      * @var codec_node_subdev::vdev
      * Vio's sub-device abstraction.
      * range:N/A; default: N/A
      */
     struct vio_subdev vdev;

     /**
      * @var codec_node_subdev::*codec_node_dev
      * pointer to codec_node_dev.
      * range:N/A; default: N/A
      */
     struct j6_codec_node_dev *codec_node_dev;

     /**
      * @var codec_node_subdev::codec_node_attr_s
      * Initialization parameters of the codec_node.
      * range:N/A; default: N/A
      */
     struct codec_node_attr_s  codec_attr;

     /**
      * @var codec_node_subdev::user_bind
      * Initialization parameters of the codec_node.
      * range:N/A; default: N/A
      */
     u32 user_bind;
};

/**
 * @struct j6_codec_node_dev
 * @brief CODEC_NODE device definition.
 * @NO{S10E01C01}
 */
struct j6_codec_node_dev {
     /**
      * @var j6_codec_node_dev::channel_idx_bitmap
      * codec_node's channel idx bitmap.
      * range:N/A; default: N/A
      */
     DECLARE_BITMAP(channel_idx_bitmap, CODEC_MAX_SUBDEV);

     /**
      * @var j6_codec_node_dev::pdev
      * codec_node's platform_device
      * range:N/A; default: N/A
      */
     struct platform_device *pdev;

     /**
      * @var j6_codec_node_dev::hw_id
      * codec_node's hardware id, not used.
      * range:N/A; default: N/A
      */
     u32 hw_id;

     /**
      * @var j6_codec_node_dev::vps_device
      * VPF device node.
      * range:N/A; default: N/A
      */
     struct vpf_device codec_device[2];

     /**
      * @var j6_codec_node_dev::open_cnt
      * The number of times codec_node was opened.
      * range:N/A; default: N/A
      */
     atomic_t open_cnt;

     /**
      * @var j6_codec_node_dev::mlock
      * mutex lock.
      * range:N/A; default: N/A
      */     
     osal_mutex_t mlock;

     /**
      * @var j6_codec_node_dev::src_subdev
      * Abstraction of src nodes
      * range:N/A; default: N/A
      */  
     struct codec_node_subdev subdev[CODEC_MAX_CHANNEL][2];

     /**
      * @var j6_vin_node_dev::vnode
      * One vnode along the way
      * range:N/A; default: N/A
      */
     struct vio_node  vnode[CODEC_MAX_CHANNEL];
};

#endif