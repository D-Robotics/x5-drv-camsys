#
# Makefile for the Linux Kernel PYM specific device drivers.
#
ifeq ($(OUTPUT_BUILD_DIR),)
#J5
INC_DIR := $(srctree)/drivers/media/platform/hobot
OUT_DIR := ${OUT_BUILD_KERNEL_DIR}
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
else
#j6
INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/
endif

obj-$(CONFIG_HOBOT_CAMSYS) += hobot_camsys.o
hobot_camsys-objs := hobot_camsys_ops.o hobot_dev_camsys.o hobot_camsys_hw_reg.o
ccflags-y += -I$(INC_DIR)/vpf/

ifneq ($(CONFIG_HOBOT_CAMSYS_STL),)
hobot_camsys-objs += hobot_camsys_stl.o
endif

ccflags-y += -D _LINUX_KERNEL_MODE
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

all: FORCE
	make -C $(OUT_DIR) M=$(PWD) modules -j${N}

clean: FORCE
	make -C $(OUT_DIR) M=$(PWD) clean -j${N}

FORCE:


