#
# Makefile for the Linux Kernel Deserial specific device drivers.
#

obj-$(CONFIG_HOBOT_DESERIAL) += hobot_deserial.o

hobot_deserial-objs += hobot_deserial_dev.o
hobot_deserial-objs += hobot_deserial_ops.o

ifeq ($(OUTPUT_BUILD_DIR),)
#J5
INC_DIR := $(srctree)/drivers/media/platform/hobot
OUT_DIR := ${OUT_BUILD_KERNEL_DIR}
ccflags-y += -I$(INC_DIR)/osal/linux/inc
else
#j6
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel
ccflags-y += -I$(INC_DIR)/../osal/linux/inc
endif

ccflags-y += -I$(INC_DIR)/vpf
ccflags-y += -I$(INC_DIR)/vcon

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C ${OUT_DIR} M=$(PWD) modules -j${N}

clean: FORCE
	make -C ${OUT_DIR} M=$(PWD) clean -j${N}

FORCE:

