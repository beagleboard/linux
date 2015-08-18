obj-$(CONFIG_VIDEO_TI_VPE) += ti-vpe.o
obj-$(CONFIG_VIDEO_TI_VIP) += ti-vip.o
obj-$(CONFIG_VIDEO_TI_VPDMA_HELPER) += ti-vpdma.o

ifdef CONFIG_VIDEO_TI_VPE_DEBUG
CFLAGS_vpe.o := -DDEBUG
CFLAGS_csc.o := -DDEBUG
CFLAGS_sc.o := -DDEBUG
endif

ti-vpe-y := vpe.o sc.o csc.o
ti-vip-y := vip.o
ti-vpdma-y := vpdma.o

obj-$(CONFIG_VIDEO_TI_CAL) += ti-cal.o

ti-cal-y := cal.o
