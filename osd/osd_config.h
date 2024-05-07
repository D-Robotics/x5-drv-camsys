/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_OSD_CONFIG_H__
#define __HOBOT_OSD_CONFIG_H__

#include <linux/types.h>

#define OSD_POLYGON_MAX_SIDE 10

struct osd_point {
	uint32_t x;
	uint32_t y;
};

struct osd_size {
	uint32_t w;
	uint32_t h;
};

struct osd_line {
	struct osd_point p1;
	struct osd_point p2;
	double k;/*PRQA S ALL*/
	double b;/*PRQA S ALL*/
	uint8_t k_flag;     // 1:parallel with x;  2:parallel with y
};

struct osd_rect {
	struct osd_point point;
	struct osd_size size;
};

struct osd_polygon {
	// uint8_t show_en;
	// uint8_t invert_en;
	// uint32_t color;
	uint32_t side_num;
	struct osd_point point[OSD_POLYGON_MAX_SIDE];  // point of vertex
	struct osd_line line[OSD_POLYGON_MAX_SIDE];

	// struct osd_point start_point;
	// struct osd_size range;
	struct osd_point *start;  // start and end of every line
	struct osd_point *end;
};

#endif //
