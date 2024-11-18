// SPDX-License-Identifier: GPL-2.0-only
#ifndef _VIDEO_LINK_H_
#define _VIDEO_LINK_H_

#include "utils.h"

struct entity_link {
	const char *src_name;
	u16 src_pad;
	const char *sink_name;
	u16 sink_pad;
	u32 flags;
};

/* sifx4 output */
static struct entity_link links0[] = {
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* [0,1,2,3] sif online isp x4 */
/* [4,5] sif raw offline isp */
static struct entity_link links1[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "0-1", 0, ISP_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED }, // sif offline isp
	{ ISP_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "1-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "2-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "3-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
};

/* sif online isp online vse */
static struct entity_link links2[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP capture data
	{ ISP_DEV_NAME "0-1", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP capture data
	{ ISP_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP capture data
	{ ISP_DEV_NAME "0-3", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP capture data
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "1-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "2-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
	{ SIF_DEV_NAME "3-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF capture data
};

/* sif offline vse */
static struct entity_link links3[] = {
	{ SIF_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-1", 0, VSE_DEV_NAME "0-5", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-5", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* sif offline isp offline vse 6 channel */
static struct entity_link links4[] = {
	{ SIF_DEV_NAME "0-1", 0, ISP_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-4", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* vse stand alone 6 channel */
static struct entity_link links5[] = {
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 2, "video-m2m", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* sif online isp online vse 6 channel */
static struct entity_link links6[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 1, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 5, "video", 0, MEDIA_LNK_FL_ENABLED },

};

/* 1v sif online isp gdc */
/* + 1v sif offline isp gdc */
static struct entity_link links7[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-4", 0, GDC_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* 2v sif yuv offline gdc */
static struct entity_link links8[] = {
	{ SIF_DEV_NAME "0-1", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-1", 0, GDC_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* 4v sif online isp gdc vse */
static struct entity_link links9[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, GDC_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, GDC_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, GDC_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-5", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-6", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-7", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-5", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-6", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-7", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* 4v sif online isp online vse gdc */
static struct entity_link links10[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-0", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-1", 0, GDC_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-2", 0, GDC_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-3", 0, GDC_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-2", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-3", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* 4v sif online isp offline vse gdc */
static struct entity_link links11[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "1-0", 0, ISP_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "2-0", 0, ISP_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "3-0", 0, ISP_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-0", 0, VSE_DEV_NAME "0-4", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-1", 0, VSE_DEV_NAME "0-5", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-2", 0, VSE_DEV_NAME "0-6", 0, MEDIA_LNK_FL_ENABLED },
	{ ISP_DEV_NAME "0-3", 0, VSE_DEV_NAME "0-7", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 0, GDC_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-5", 0, GDC_DEV_NAME "0-1", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-6", 0, GDC_DEV_NAME "0-2", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-7", 0, GDC_DEV_NAME "0-3", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-0", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-1", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-2", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ GDC_DEV_NAME "0-3", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* SIF/ISP as front capture device connect to VSE mem2mem device */
static struct entity_link links12[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF as front
	{ ISP_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP as front
	{ VSE_DEV_NAME "0-4", 0, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 1, "video-m2m", 0, MEDIA_LNK_FL_ENABLED }, // m2m master
	{ VSE_DEV_NAME "0-4", 2, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 3, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 4, "video", 0, MEDIA_LNK_FL_ENABLED },
	{ VSE_DEV_NAME "0-4", 5, "video", 0, MEDIA_LNK_FL_ENABLED },
};

/* gdc stand alone */
static struct entity_link links13[] = {
	{ GDC_DEV_NAME "0-0", 0, "video-m2m", 0, MEDIA_LNK_FL_ENABLED },
};

/* SIF/ISP as front capture device connect to GDC mem2mem device */
static struct entity_link links14[] = {
	{ SIF_DEV_NAME "0-0", 0, ISP_DEV_NAME "0-0", 0, MEDIA_LNK_FL_ENABLED },
	{ SIF_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // SIF as front
	{ ISP_DEV_NAME "0-0", 1, "video", 0, MEDIA_LNK_FL_ENABLED }, // ISP as front
	{ GDC_DEV_NAME "0-0", 0, "video-m2m", 0, MEDIA_LNK_FL_ENABLED },
};

static struct entity_link *links[] = {
	links0,
	links1,
	links2,
	links3,
	links4,
	links5,
	links6,
	links7,
	links8,
	links9,
	links10,
	links11,
	links12,
	links13,
	links14,
};

static u32 links_size[] = {
	ARRAY_SIZE(links0),
	ARRAY_SIZE(links1),
	ARRAY_SIZE(links2),
	ARRAY_SIZE(links3),
	ARRAY_SIZE(links4),
	ARRAY_SIZE(links5),
	ARRAY_SIZE(links6),
	ARRAY_SIZE(links7),
	ARRAY_SIZE(links8),
	ARRAY_SIZE(links9),
	ARRAY_SIZE(links10),
	ARRAY_SIZE(links11),
	ARRAY_SIZE(links12),
	ARRAY_SIZE(links13),
	ARRAY_SIZE(links14),
};

#endif /* _VIDEO_LINK_H_ */
