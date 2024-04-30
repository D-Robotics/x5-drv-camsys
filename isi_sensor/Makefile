#
# Makefile for the Linux Kernel IPS specific device drivers.
#
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel

obj-$(CONFIG_HOBOT_VIO_COMMON) += hobot_isi_sensor.o
hobot_isi_sensor-objs := hobot_isi_dev_sensor.o

ccflags-y += -I$(INC_DIR)/sensor/
ccflags-y += -I$(INC_DIR)/sensor/inc/
ccflags-y += -I$(INC_DIR)/vpf/

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C $(OUT_DIR) M=$(PWD) modules -j${N}

clean: FORCE
	make -C $(OUT_DIR) M=$(PWD) clean -j${N}

FORCE: