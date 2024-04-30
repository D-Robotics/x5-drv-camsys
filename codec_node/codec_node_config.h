/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef CODEC_NODE_CONFIG_H
#define CODEC_NODE_CONFIG_H

#include "hobot_codec_common.h"

typedef struct codec_node_attr_s {
	uint32_t input_width;
	uint32_t input_stride;
	uint32_t input_height;
	uint32_t output_width;
	uint32_t output_stride;
	uint32_t output_height;
	uint32_t buf_num;
	uint32_t fb_buf_num;
	uint32_t channel_idx;
} codec_node_attr_t;

typedef struct codec_attr_ex_s {
	uint32_t  trigger_source;
	uint32_t  ipi_reset;
	uint32_t  bypass_enable;
} codec_attr_ex_t;

typedef struct codec_ochn_attr_s {
	uint32_t  ddr_en;
} codec_ochn_attr_t;

typedef struct codec_ochn_buff_attr_s {
	uint32_t buffers_num;
	int64_t   flags;
} codec_ochn_buffer_attr_t;

typedef struct codec_ichn_attr_s {
	uint32_t  format;
	uint32_t  width;
	uint32_t  height;
} codec_ichn_attr_t;

#endif /*CODEC_NODE_CONFIGs_H*/
