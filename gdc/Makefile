#
# Makefile for the Linux Kernel GDC specific device drivers.
#
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y += -I$(srctree)/drivers/staging/android/ion/
ccflags-y += -I$(srctree)/drivers/smmu/


obj-$(CONFIG_HOBOT_GDC_JPLUS) += hobot_gdc.o
hobot_gdc-objs := hobot_gdc_hw_reg.o hobot_gdc_ops.o hobot_dev_gdc.o
ccflags-y += -I$(INC_DIR)/vpf/
ccflags-y += -I$(INC_DIR)/vsi_cam/include/

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

ccflags-y += -D X5_CHIP



