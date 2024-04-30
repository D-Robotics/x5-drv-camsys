obj-$(CONFIG_HOBOT_LPWM) += hobot_lpwm.o
hobot_lpwm-objs := hobot_lpwm_dev.o hobot_lpwm_hw_reg.o hobot_lpwm_ops.o

obj-$(CONFIG_HOBOT_STL_LPWM) += hobot_lpwm_stl.o

ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/j6_cim/
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/
ccflags-y += -I$(srctree)/drivers/staging/android/ion/
ccflags-y += -I$(srctree)/drivers/smmu/

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel

all:
	make -C $(OUT_DIR) M=$(PWD) modules

clean:
	make -C $(OUT_DIR) M=$(PWD) clean