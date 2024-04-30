ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/


obj-$(CONFIG_HOBOT_OSD) += hobot_osd.o
# obj-m += hobot_osd.o
hobot_osd-objs := hobot_dev_osd.o hobot_osd_process.o hobot_osd_mem.o

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
