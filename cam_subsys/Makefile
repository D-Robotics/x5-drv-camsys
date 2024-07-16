#
# Makefile for the Linux Kernel PYM specific device drivers.
#

INC_DIR := $(srctree)/drivers/media/platform/horizon/camsys
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y +=  -I$(srctree)/drivers/staging/android/ion/
ccflags-y +=  -I$(srctree)/drivers/smmu/


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

