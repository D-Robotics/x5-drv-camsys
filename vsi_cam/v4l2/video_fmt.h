/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VIDEO_FMT_H_
#define _VIDEO_FMT_H_

#define MIN_W   (32)
#define MIN_H   (32)
#define MAX_W   (8192)
#define MAX_H   (8192)
#define ALIGN_W (1)
#define ALIGN_H (1)

struct video_fmt {
	u32 pixelformat;
	u32 mbus_code;
	u16 bit_depth;
	u16 bpp;
};

static struct video_fmt formats[] = {
	{ V4L2_PIX_FMT_SBGGR8,  MEDIA_BUS_FMT_SBGGR8_1X8,   8,  1 },
	{ V4L2_PIX_FMT_SGBRG8,  MEDIA_BUS_FMT_SGBRG8_1X8,   8,  1 },
	{ V4L2_PIX_FMT_SGRBG8,  MEDIA_BUS_FMT_SGRBG8_1X8,   8,  1 },
	{ V4L2_PIX_FMT_SRGGB8,  MEDIA_BUS_FMT_SRGGB8_1X8,   8,  1 },
	{ V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, 16, 2 },
	{ V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, 16, 2 },
	{ V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, 16, 2 },
	{ V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, 16, 2 },
	{ V4L2_PIX_FMT_SBGGR12, MEDIA_BUS_FMT_SBGGR12_1X12, 16, 2 },
	{ V4L2_PIX_FMT_SGBRG12, MEDIA_BUS_FMT_SGBRG12_1X12, 16, 2 },
	{ V4L2_PIX_FMT_SGRBG12, MEDIA_BUS_FMT_SGRBG12_1X12, 16, 2 },
	{ V4L2_PIX_FMT_SRGGB12, MEDIA_BUS_FMT_SRGGB12_1X12, 16, 2 },
	{ V4L2_PIX_FMT_YUYV,    MEDIA_BUS_FMT_YUYV8_2X8,    16, 2 },
	{ V4L2_PIX_FMT_NV12,    MEDIA_BUS_FMT_YUYV8_1_5X8,  12, 1 },
	{ V4L2_PIX_FMT_NV16,    MEDIA_BUS_FMT_YUYV8_2X8,    16, 1 },
	{ V4L2_PIX_FMT_RGB32,   MEDIA_BUS_FMT_RGB888_1X32_PADHI, 32, 4 },
};

static struct video_fmt *get_fmt_by_pixelformat(u32 pixelformat)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (formats[i].pixelformat == pixelformat)
			return &formats[i];
	return NULL;
}

#endif /* _VIDEO_FMT_H_ */
