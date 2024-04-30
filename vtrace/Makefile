ifeq ($(OUTPUT_BUILD_DIR),)
#J5
CONFIG_HOBOT_VTRACE=m
OUT_DIR := ${OUT_BUILD_KERNEL_DIR}
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/vin_node/
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/osal/linux/inc/
else
# j6
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
endif

obj-$(CONFIG_HOBOT_VTRACE) += hobot_vtrace.o

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C ${OUT_DIR} M=$(PWD) modules

clean: FORCE
	make -C ${OUT_DIR} M=$(PWD) clean

FORCE: