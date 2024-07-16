#
# Makefile for the Linux Kernel VCON specific device drivers.
#

obj-$(CONFIG_HOBOT_VCON) += hobot_vin_vcon.o

hobot_vin_vcon-objs += hobot_vin_vcon_dev.o
hobot_vin_vcon-objs += hobot_vin_vcon_ops.o


INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
ccflags-y += -I$(INC_DIR)/../osal/linux/inc


ccflags-y += -I$(INC_DIR)/vpf
ccflags-y += -I$(INC_DIR)/vin_node
ccflags-y += -I$(INC_DIR)/j6_cim

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif


