ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/native/vse
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/include
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vsi_cam/native


obj-$(CONFIG_HOBOT_OSD) += hobot_osd.o
# obj-m += hobot_osd.o
hobot_osd-objs := hobot_osd_dev.o hobot_osd_process.o hobot_osd_mem.o hobot_osd_ops.o

ccflags-y += -D _LINUX_KERNEL_MODE

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

OUT_DIR := ${OUTPUT_BUILD_DIR}/kernel

all: FORCE
	make -C $(OUT_DIR) M=$(PWD) modules

clean: FORCE
	make -C $(OUT_DIR) M=$(PWD) clean

FORCE:
