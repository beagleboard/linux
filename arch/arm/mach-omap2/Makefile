#
# Makefile for the linux kernel.
#

# Common support
obj-y := irq.o id.o io.o sram-fn.o memory.o prcm.o clock.o mux.o devices.o \
	 serial.o gpmc.o

obj-$(CONFIG_OMAP_MPU_TIMER)		+= timer-gp.o

# Power Management
obj-$(CONFIG_PM) += pm.o sleep.o

# DSP
obj-$(CONFIG_OMAP_MMU_FWK)	+= mmu_mach.o
obj-$(CONFIG_OMAP_MBOX_FWK)	+= mailbox_mach.o
mailbox_mach-objs		:= mailbox.o
mmu_mach-objs			:= mmu.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= board-2430sdp.o \
					   board-2430sdp-flash.o \
					   board-2430sdp-usb.o
obj-$(CONFIG_MACH_OMAP_APOLLON)		+= board-apollon.o \
					   board-apollon-keys.o
obj-$(CONFIG_MACH_NOKIA_N800)		+= board-n800.o board-n800-flash.o \
					   board-n800-mmc.o board-n800-bt.o \
					   board-n800-audio.o board-n800-usb.o \
					   board-n800-dsp.o board-n800-pm.o

# TUSB 6010 chips
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o

