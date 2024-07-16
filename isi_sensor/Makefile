#
# Makefile for the Linux Kernel IPS specific device drivers.
#
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys

obj-$(CONFIG_HOBOT_VIO_COMMON) += hobot_isi_sensor.o
hobot_isi_sensor-objs := hobot_isi_dev_sensor.o

ccflags-y += -I$(INC_DIR)/sensor/
ccflags-y += -I$(INC_DIR)/sensor/inc/
ccflags-y += -I$(INC_DIR)/vpf/

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif