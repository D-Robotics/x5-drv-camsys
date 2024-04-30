#
# Makefile for the Linux Kernel VIN_NODE specific device drivers.
#

ifeq ($(OUTPUT_BUILD_DIR),)
#J5
INC_DIR := $(srctree)/drivers/media/platform/hobot
OUT_DIR := ${OUT_BUILD_KERNEL_DIR}
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/osal/linux/include/
else
#j6
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel
ccflags-y +=  -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/
endif

obj-$(CONFIG_HOBOT_VIN_NODE) += hobot_vin_vnode.o
hobot_vin_vnode-objs := hobot_vin_node_ops.o hobot_dev_vin_node.o

ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/j6_cim/
ccflags-y += -DX5_CHIP

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C ${OUTPUT_BUILD_DIR}/kernel M=$(PWD) modules

clean: FORCE
	make -C ${OUTPUT_BUILD_DIR}/kernel M=$(PWD) clean

FORCE:

