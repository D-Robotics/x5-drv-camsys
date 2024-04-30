/**
 * @file: hobot_gdc_ops.h
 * @
 * @NO{S09E03C01}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */

#ifndef HOBOT_GDC_J5_OPS_API
#define HOBOT_GDC_J5_OPS_API

#define GDC_PROCESS_TIMEOUT		(100) /* < FTTI 132ms */
#define GDC_LAYER_INDEX 37u

#define INTR_GDC_BUSY  ((u32)1 << 0)
#define	INTR_GDC_ERROR ((u32)1 << 1)
#define	INTR_GDC_CONF_ERROR ((u32)1 << 8)
#define	INTR_GDC_USER_ABORT ((u32)1 << 9)
#define	INTR_GDC_AXI_READER_ERROR ((u32)1 << 10)
#define	INTR_GDC_AXI_WRITER_ERROR ((u32)1 << 11)
#define	INTR_GDC_UNALIGNED_ACCESS ((u32)1 << 12)
#define	INTR_GDC_INCOMPATIBLE_CONF ((u32)1 << 13)

#define CONFIG_SIZE_OFFSET 4u
#define YUV_PLANE_0 0u
#define YUV_PLANE_1 1u
#define YUV_PLANE_2 2u
#define WAIT_TIMEOUT 5u

#define GDC_MAX_WIDTH  3840u
#define GDC_MAX_HEIGHT 3840u

s32 gdc_get_version(struct vio_version_info *version);
s32 gdc_subdev_open(struct vio_video_ctx *vctx);
s32 gdc_subdev_close(struct vio_video_ctx *vctx, u32 rst_en);
s32 gdc_force_stop(struct vio_video_ctx *vctx);
void gdc_handle_interrupt(struct hobot_gdc_dev *gdc, u32 gdc_status);
void gdc_frame_work(struct vio_node *vnode);
s32 gdc_allow_bind(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online_mode);
s32 gdc_setting_check(gdc_settings_t *gdc_cfg);
s32 gdc_iommu_map(struct gdc_subdev *subdev);
void gdc_iommu_ummap(struct gdc_subdev *subdev);
void gdc_attr_trans_to_settings(gdc_attr_t *gdc_attr, gdc_ichn_attr_t *ichn_attr,
                    gdc_ochn_attr_t *ochn_attr, gdc_settings_t *gdc_setting);
void gdc_settings_trans_to_attr(gdc_attr_t *gdc_attr, gdc_ichn_attr_t *ichn_attr,
					gdc_ochn_attr_t *ochn_attr, gdc_settings_t *gdc_setting);
#endif
