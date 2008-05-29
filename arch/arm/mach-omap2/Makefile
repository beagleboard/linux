#
# Makefile for the linux kernel.
#

# Common support
obj-y := irq.o id.o io.o memory.o control.o prcm.o clock.o mux.o \
		devices.o serial.o gpmc.o timer-gp.o powerdomain.o \
		clockdomain.o

obj-$(CONFIG_OMAP_MCBSP) += mcbsp.o

# Functions loaded to SRAM
obj-$(CONFIG_ARCH_OMAP2420)		+= sram242x.o
obj-$(CONFIG_ARCH_OMAP2430)		+= sram243x.o
obj-$(CONFIG_ARCH_OMAP3)		+= sram34xx.o

# Power Management
obj-$(CONFIG_PM) += pm.o

ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_ARCH_OMAP2)		+= pm24xx.o
obj-$(CONFIG_ARCH_OMAP2420)		+= sleep242x.o
obj-$(CONFIG_ARCH_OMAP2430)		+= sleep243x.o
obj-$(CONFIG_ARCH_OMAP3)		+= pm34xx.o sleep34xx.o
obj-$(CONFIG_PM_DEBUG)			+= pm-debug.o
endif

# Clock framework
obj-$(CONFIG_ARCH_OMAP2)		+= clock24xx.o
obj-$(CONFIG_ARCH_OMAP3)		+= clock34xx.o

# DSP
obj-$(CONFIG_OMAP_MMU_FWK)	+= mmu_mach.o
obj-$(CONFIG_OMAP_MBOX_FWK)	+= mailbox_mach.o
mailbox_mach-objs		:= mailbox.o
mmu_mach-objs			:= mmu.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o board-h4-mmc.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= board-2430sdp.o \
					   board-2430sdp-flash.o \
					   hsmmc.o \
					   usb-musb.o \
					   usb-ehci.o
obj-$(CONFIG_MACH_OMAP_2430OSK)		+= board-2430osk.o
obj-$(CONFIG_MACH_OMAP_3430SDP)		+= board-3430sdp.o \
					   hsmmc.o \
					   usb-musb.o \
					   usb-ehci.o \
					   board-3430sdp-flash.o
obj-$(CONFIG_MACH_OMAP3EVM)		+= board-omap3evm.o \
					   hsmmc.o \
					   usb-musb.o usb-ehci.o \
					   board-omap3evm-flash.o
obj-$(CONFIG_MACH_OMAP3_BEAGLE)		+= board-omap3beagle.o \
					   usb-musb.o usb-ehci.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_LDP)		+= board-ldp.o \
					   hsmmc.o \
					   usb-musb.o
obj-$(CONFIG_MACH_OMAP_APOLLON)		+= board-apollon.o \
					   board-apollon-mmc.o	\
					   board-apollon-keys.o
obj-$(CONFIG_MACH_NOKIA_N800)		+= board-n800.o board-n800-flash.o \
					   board-n800-mmc.o board-n800-bt.o \
					   board-n800-audio.o board-n800-usb.o \
					   board-n800-dsp.o \
					   board-n800-camera.o
obj-$(CONFIG_MACH_NOKIA_N810)		+= board-n810.o

# TUSB 6010 chips
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o

