#
# Makefile for the Linux Kernel Deserial specific device drivers.
#

obj-$(CONFIG_HOBOT_DESERIAL) += hobot_deserial.o

hobot_deserial-objs += hobot_deserial_dev.o
hobot_deserial-objs += hobot_deserial_ops.o

INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
ccflags-y += -I$(INC_DIR)/../osal/linux/inc
ccflags-y += -I$(INC_DIR)/vpf
ccflags-y += -I$(INC_DIR)/vcon
ccflags-y += -D _LINUX_KERNEL_MODE

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

