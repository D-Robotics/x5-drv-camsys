
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vpf/
ccflags-y += -I$(srctree)/drivers/media/platform/horizon/camsys/vin_node/
ccflags-y += -I$(srctree)/drivers/osal/linux/inc/


obj-$(CONFIG_HOBOT_VTRACE) += hobot_vtrace.o

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
