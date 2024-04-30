#
# Makefile for the Linux Kernel CODEC_NODE specific device drivers.
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
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/
endif

obj-$(CONFIG_HOBOT_CODEC_NODE) += hobot_codec_vnode.o
hobot_codec_vnode-objs := hobot_codec_node_ops.o hobot_dev_codec_node.o

ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/codec_node/

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C ${OUTPUT_BUILD_DIR}/kernel M=$(PWD) modules

clean: FORCE
	make -C ${OUTPUT_BUILD_DIR}/kernel M=$(PWD) clean

FORCE:
