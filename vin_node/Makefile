#
# Makefile for the Linux Kernel VIN_NODE specific device drivers.
#


INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys

ccflags-y +=  -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/


obj-$(CONFIG_HOBOT_VIN_NODE) += hobot_vin_vnode.o
hobot_vin_vnode-objs := hobot_vin_node_ops.o hobot_dev_vin_node.o

ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/j6_cim/
ccflags-y += -DX5_CHIP

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif


