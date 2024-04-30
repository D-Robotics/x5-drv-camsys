#
# Makefile for the Linux Kernel IPS specific device drivers.
#
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
subdir-ccflags-y += -I$(srctree)/drivers/staging/android/ion/
subdir-ccflags-y += -I$(srctree)/drivers/smmu/
subdir-ccflags-y += -I$(srctree)/drivers/iommu/
subdir-ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/sensor/
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/native/sif
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/include
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/idu/

obj-$(CONFIG_HOBOT_VIO_JPLUS) += vpf/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += cam_subsys/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += vin_node/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += vcon/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += gdc/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += sensor/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += deserial/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += lpwm/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += vtrace/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += codec_node/

obj-$(CONFIG_HOBOT_MIPI_CSI) += mipi/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += idu/
obj-$(CONFIG_HOBOT_VSI_CAM) += vsi_cam/
obj-$(CONFIG_HOBOT_VIO_JPLUS) += osd/
obj-y += isi_sensor/