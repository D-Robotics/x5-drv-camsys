#
# Makefile for the Linux Kernel VPF specific device drivers.
#
ifeq ($(OUTPUT_BUILD_DIR),)
#J5
INC_DIR := $(srctree)/drivers/media/platform/hobot
OUT_DIR := ${OUT_BUILD_KERNEL_DIR}
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/osal/linux/inc/
else
#j6
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel
ccflags-y +=  -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/
endif

obj-$(CONFIG_HOBOT_VIO_COMMON) += hobot_vio_common.o
hobot_vio_common-objs := hobot_vpf_manager.o hobot_vpf_ops.o vio_cops_api.o vio_framemgr.o vio_mem.o vio_hw_common_api.o vio_debug_api.o vio_debug_dev.o vio_node_api.o vio_video_api.o vio_chain_api.o vio_metadata_api.o
ccflags-y += -I$(INC_DIR)/sensor/inc/

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

ccflags-y += -D X5_CHIP

all: FORCE
	make -C $(OUT_DIR) M=$(PWD) modules -j${N}

clean: FORCE
	make -C $(OUT_DIR) M=$(PWD) clean -j${N}

FORCE:

