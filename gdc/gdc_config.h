/**
 * @file: gdc_config.h
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */

#ifndef HOBOT_GDC_CONFIG_H
#define HOBOT_GDC_CONFIG_H

#include <linux/types.h>

#define ACAMERA_GDC_MAX_INPUT 3

typedef struct gdc_attr_s {
	uint64_t config_addr;   /* gdc bin address */
	uint32_t config_size;
	uint8_t div_width;      /* use in dividing UV dimensions parameters */
	uint8_t div_height;     /* use in dividing UV dimensions parameters */
	uint32_t total_planes;
	int32_t binary_ion_id;  /* share id for config binary physical addr */
	uint64_t binary_offset; /* config binary physical addr offset */
} gdc_attr_t;

typedef struct gdc_ichn_attr_s {
	uint32_t input_width;
	uint32_t input_height;
	uint32_t input_stride;
} gdc_ichn_attr_t;

typedef struct gdc_ochn_attr_s {
	uint32_t output_width;
	uint32_t output_height;
	uint32_t output_stride;
} gdc_ochn_attr_t;

// each configuration addresses and size
typedef struct gdc_config {
	u64 config_addr;   //gdc config address
	u32 config_size;   //gdc config size in 32bit
	u32 input_width;  //gdc input width resolution
	u32 input_height; //gdc input height resolution
	u32 input_stride;  //gdc input stride (pixel)
	u32 output_width;  //gdc output width resolution
	u32 output_height; //gdc output height resolution
	u32 output_stride;  //gdc output stride (pixel)
	u8  div_width;     //use in dividing UV dimensions; actually a shift right
	u8  div_height;    //use in dividing UV dimensions; actually a shift right
	u32 total_planes;
	u8 sequential_mode; //sequential processing(not used)
} gdc_config_t;

// overall gdc settings and state
typedef struct gdc_settings {
	gdc_config_t gdc_config; //array of gdc configuration and sizes
	s32 binary_ion_id; //share id for config binary physical addr;
	u64 binary_offset;
	u32 reserved[8];
} gdc_settings_t;

#endif
