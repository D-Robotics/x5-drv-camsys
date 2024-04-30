#
# Makefile for the Linux Kernel IPS specific device drivers.
#
KBUILD_EXTRA_SYMBOLS += $(srctree)/Module.symvers
export KBUILD_EXTRA_SYMBOLS

ifneq ($(CONFIG_HOBOT_CHIP_J6X),)
ccflags-y += -I$(srctree)/../hobot-drivers/osal/linux/inc/
ccflags-y += -I$(srctree)/../hobot-drivers/ide/idu
ccflags-y += -I$(srctree)/../hobot-drivers/ide/mipi-dsi/
ccflags-y += -I$(srctree)/../hobot-drivers/ide/mipi-dsi/hardware
ccflags-y += -I$(srctree)/../hobot-drivers/ide/mipi-csi-tx/
ccflags-y += -I$(srctree)/../hobot-drivers/ide/mipi-tx-dphy/
ccflags-y += -I$(srctree)/../hobot-drivers/ide/idu/hardware/
ccflags-y += -I$(srctree)/../hobot-drivers/ide/ide
ccflags-y += -I$(srctree)/../hobot-drivers/ide/ide/hardware

endif

ifneq ($(CONFIG_HOBOT_CHIP_J5),)
ccflags-y += -I$(srctree)/drivers/media/platform/hobot/osal/linux/inc
ccflags-y += -I$(srctree)/drivers/gpu/drm/hobot/idu
ccflags-y += -I$(srctree)/drivers/gpu/drm/hobot/mipi-csi-tx/
ccflags-y += -I$(srctree)/drivers/gpu/drm/hobot/mipi-tx-dphy/
ccflags-y += -I$(srctree)/drivers/gpu/drm/hobot/idu/hardware/
endif

ifneq ($(CONFIG_HOBOT_IDU_VNODE),)
hobot_idu_vnode-y:= hobot_idu_vnode_dev.o hobot_idu_vnode_ops.o hobot_idu_vnode_debug.o
obj-$(CONFIG_HOBOT_IDU_VNODE) += hobot_idu_vnode.o
endif

KDIR=${srctree}
default:
	CROSS_COMPILE=${CROSS_COMPILE} make ARCH=${ARCH} -C $(KDIR) M=$(PWD) modules

clean:
	CROSS_COMPILE=${CROSS_COMPILE} make ARCH=${ARCH} -C $(KDIR) M=$(PWD) clean

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif