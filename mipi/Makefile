#
# Makefile for the Linux Kernel MIPI specific device drivers.
#

obj-$(CONFIG_HOBOT_MIPI_CSI_DRV) += hobot_mipicsi.o
obj-$(CONFIG_HOBOT_MIPI_CSI_STL) += hobot_mipistl.o
obj-$(CONFIG_HOBOT_MIPI_PHY) += hobot_mipiphy.o
obj-$(CONFIG_HOBOT_MIPI_DEBUG) += hobot_mipidbg.o

hobot_mipicsi-objs += hobot_mipi_csi.o
ifneq ($(CONFIG_HOBOT_MIPI_HOST),)
hobot_mipicsi-objs += hobot_mipi_host.o
hobot_mipicsi-objs += hobot_mipi_host_ops.o
endif
ifneq ($(CONFIG_HOBOT_MIPI_DEV),)
hobot_mipicsi-objs += hobot_mipi_dev.o
endif

hobot_mipiphy-objs += hobot_mipi_phy.o
hobot_mipiphy-objs += hobot_mipi_phy_ops.o

hobot_mipistl-objs += hobot_mipi_csi_stl.o
ifneq ($(CONFIG_HOBOT_MIPI_CSI_HOST_STL),)
hobot_mipistl-objs += hobot_mipi_host_stl.o
endif
ifneq ($(CONFIG_HOBOT_MIPI_CSI_ERM_STL),)
hobot_mipistl-objs += hobot_mipi_erm_stl.o
endif

hobot_mipidbg-objs += hobot_mipi_debug.o
hobot_mipidbg-objs += hobot_mipi_debug_hdev.o
hobot_mipidbg-objs += hobot_mipi_debug_pdev.o

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

# ccflags-y += -DMIPI_DEV_IDE_ENABLE
ccflags-y += -I$(INC_DIR)/../ide/mipi-csi-tx
ccflags-y += -I$(INC_DIR)/../ide/mipi-tx-dphy
endif

ccflags-y += -I$(INC_DIR)/vpf
ccflags-y += -I$(INC_DIR)/vin_node
ccflags-y += -I$(INC_DIR)/j6_cim

ccflags-y += -D _LINUX_KERNEL_MODE
ccflags-y += -DX5_CHIP
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

ccflags-y += -D X5_CHIP

all: FORCE
	make -C ${OUT_DIR} M=$(PWD) modules -j${N}

clean: FORCE
	make -C ${OUT_DIR} M=$(PWD) clean -j${N}

FORCE:

