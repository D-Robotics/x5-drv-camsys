# SPDX-License-Identifier: GPL-2.0-only

# origin
# ccflags-y += -I$(srctree)/$(src)/include
# ccflags-y += -I$(srctree)/$(src)/../hbn/vpf
# ccflags-y += -I$(srctree)/$(src)/../hbn/vin_node
# ccflags-y += -I$(srctree)/$(src)/../hbn/osal/linux/inc/
# ccflags-y += -I$(srctree)/$(src)/native
# new
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/osd/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/mipi/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/include
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/native
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/native/vse
ccflags-y += -D X5_CHIP

# isc
ifneq ($(CONFIG_ION_HOBOT),)
  vs_isc-objs += isc/isc.o isc/ion_mem_helper.o
else
  vs_isc-objs += isc/isc.o isc/mem_helper.o
endif
# obj-$(CONFIG_VIDEO_VS_SW_ISC) += vs_isc.o
obj-m += vs_isc.o

# cam sys control
vs_cam_ctrl-objs += base/cam_ctrl.o
obj-$(CONFIG_VIDEO_VS_CAM_CTRL) += vs_cam_ctrl.o

# csi wrapper
vs_csi_wrapper-objs += csi/csi_wrapper.o
obj-$(CONFIG_VIDEO_VS_CSI_WRAPPER) += vs_csi_wrapper.o

cam_common = base/cam_dev.o base/job_queue.o

# v4l2
vs_vid_v4l-objs += v4l2/vid_drv.o v4l2/video.o v4l2/cam_buf.o
obj-$(CONFIG_VIDEO_VS_VID_V4L) += vs_vid_v4l.o

vs_isp_v4l-objs += $(cam_common)
vs_isp_v4l-objs += isp/isp.o
vs_isp_v4l-objs += isp/isp_handler.o
vs_isp_v4l-objs += v4l2/isp_drv.o v4l2/cam_buf.o v4l2/utils.o

obj-$(CONFIG_VIDEO_VS_ISP_V4L) += vs_isp_v4l.o

vs_vse_v4l-objs += $(cam_common)
vs_vse_v4l-objs += vse/vse.o
vs_vse_v4l-objs += vse/vse_handler.o
vs_vse_v4l-objs += v4l2/vse_drv.o v4l2/cam_buf.o v4l2/utils.o

obj-$(CONFIG_VIDEO_VS_VSE_V4L) += vs_vse_v4l.o

vs_sif_v4l-objs += $(cam_common)
vs_sif_v4l-objs += sif/sif.o
vs_sif_v4l-objs += sif/sif_handler.o
vs_sif_v4l-objs += v4l2/sif_drv.o v4l2/cam_buf.o v4l2/utils.o

obj-$(CONFIG_VIDEO_VS_SIF_V4L) += vs_sif_v4l.o

vs_csi2_snps_v4l-objs += base/cam_dev.o
vs_csi2_snps_v4l-objs += csi/csi.o
vs_csi2_snps_v4l-objs += csi/csi_handler.o
vs_csi2_snps_v4l-objs += v4l2/csi_drv.o v4l2/csi_idi_drv.o v4l2/utils.o

obj-$(CONFIG_VIDEO_VS_CSI2_SNPS_V4L) += vs_csi2_snps_v4l.o

vs_gdc_arm_v4l-objs += $(cam_common)
vs_gdc_arm_v4l-objs += gdc/gdc.o
vs_gdc_arm_v4l-objs += gdc/gdc_handler.o
vs_gdc_arm_v4l-objs += v4l2/gdc_drv.o v4l2/cam_buf.o v4l2/utils.o

obj-$(CONFIG_VIDEO_VS_GDC_ARM_V4L) += vs_gdc_arm_v4l.o

# native
vs_isp_nat-objs += $(cam_common)
vs_isp_nat-objs += isp/isp.o
vs_isp_nat-objs += isp/isp_handler.o
vs_isp_nat-objs += native/isp/isp_drv.o native/cam_buf.o

obj-$(CONFIG_VIDEO_VS_ISP_NAT) += vs_isp_nat.o

vs_vse_nat-objs += $(cam_common)
vs_vse_nat-objs += vse/vse.o
vs_vse_nat-objs += vse/vse_handler.o
vs_vse_nat-objs += native/vse/vse_drv.o native/cam_buf.o

obj-$(CONFIG_VIDEO_VS_VSE_NAT) += vs_vse_nat.o

vs_sif_nat-objs += $(cam_common)
vs_sif_nat-objs += sif/sif.o
vs_sif_nat-objs += sif/sif_handler.o
vs_sif_nat-objs += native/cam_buf.o
vs_sif_nat-objs += native/sif/hobot_dev_cim.o
vs_sif_nat-objs += native/sif/hobot_cim_ops.o

obj-$(CONFIG_VIDEO_VS_SIF_NAT) += vs_sif_nat.o

vs_csi2_snps_nat-objs += base/cam_dev.o
vs_csi2_snps_nat-objs += csi/csi.o
vs_csi2_snps_nat-objs += csi/csi_handler.o
vs_csi2_snps_nat-objs += native/csi_drv.o

obj-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT) += vs_csi2_snps_nat.o
ccflags-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT_HBN_ADPT) += -I$(srctree)/$(src)/../hbn/vin_node
ccflags-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT_HBN_ADPT) += -I$(srctree)/$(src)/../hbn/vpf
ccflags-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT_HBN_ADPT) += -I$(srctree)/$(src)/../hbn/osal/linux/inc
ccflags-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT_HBN_ADPT) += -I$(srctree)/$(src)/../hbn/mipi
ccflags-$(CONFIG_VIDEO_VS_CSI2_SNPS_NAT_HBN_ADPT) += -I$(srctree)/$(src)/native

vs_online_ops-objs += native/cam_online_ops.o
obj-m += vs_online_ops.o

GCOV_PROFILE := $(CONFIG_GCOV_KERNEL)
