#
# Makefile for the Linux Kernel CODEC_NODE specific device drivers.
#


INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys

ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/

obj-$(CONFIG_HOBOT_CODEC_NODE) += hobot_codec_vnode.o
hobot_codec_vnode-objs := hobot_codec_node_ops.o hobot_dev_codec_node.o

ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/codec_node/

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

