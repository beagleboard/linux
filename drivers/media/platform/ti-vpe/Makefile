obj-$(CONFIG_VIDEO_TI_VPE) += ti-vpe.o
obj-$(CONFIG_VIDEO_TI_VPE) += ti-vpdma.o
obj-$(CONFIG_VIDEO_TI_VPE) += ti-sc.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-vip.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-vpdma.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-sc.o

ti-vpe-y := vpe.o csc.o
ti-vpdma-y := vpdma.o
ti-sc-y := sc.o
ti-vip-y := vip.o

ccflags-$(CONFIG_VIDEO_TI_VPE_DEBUG) += -DDEBUG

obj-$(CONFIG_VIDEO_TI_CAL) += ti-cal.o

ti-cal-y := cal.o
