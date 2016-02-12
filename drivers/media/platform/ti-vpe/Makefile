obj-$(CONFIG_VIDEO_TI_VPE) += ti-vpe.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-vip.o
obj-$(CONFIG_VIDEO_TI_VPE) += ti-vpdma.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-vpdma.o
obj-$(CONFIG_VIDEO_TI_VPE) += ti-sc.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-sc.o

ifdef CONFIG_VIDEO_TI_VPE_DEBUG
CFLAGS_vpe.o := -DDEBUG
CFLAGS_csc.o := -DDEBUG
endif

ti-vpe-y := vpe.o csc.o
ti-vip-y := vip.o
ti-vpdma-y := vpdma.o
ti-sc-y := sc.o

obj-$(CONFIG_VIDEO_TI_CAL) += ti-cal.o

ti-cal-y := cal.o
