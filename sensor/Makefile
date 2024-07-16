#
# Makefile for the Linux Kernel Sensor specific device drivers.
#

obj-$(CONFIG_HOBOT_SENSOR) += hobot_sensor.o


INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
ccflags-y += -I$(INC_DIR)/../osal/linux/inc


common_dir := $(INC_DIR)/sensor
srcfiles := $(subst $(common_dir)/,,$(wildcard $(common_dir)/src/*.c $(common_dir)/src/*/*.c))
hobot_sensor-objs += $(patsubst %.c,%.o,$(srcfiles))

ccflags-y += -I$(INC_DIR)/sensor
ccflags-y += -I$(INC_DIR)/sensor/inc
ccflags-y += -I$(INC_DIR)/vpf
ccflags-y += -I$(INC_DIR)/vcon

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

