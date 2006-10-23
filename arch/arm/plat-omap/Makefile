#
# Makefile for the linux kernel.
#

# Common support
obj-y := common.o sram.o sram-fn.o clock.o devices.o dma.o mux.o gpio.o \
	 usb.o fb.o
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_OMAP_32K_TIMER)	+= timer32k.o

# OCPI interconnect support for 1710, 1610 and 5912
obj-$(CONFIG_ARCH_OMAP16XX) += ocpi.o

# STI support
obj-$(CONFIG_OMAP_STI) += sti/

obj-$(CONFIG_OMAP_MCBSP) += mcbsp.o

obj-$(CONFIG_CPU_FREQ) += cpu-omap.o
obj-$(CONFIG_OMAP_DM_TIMER) += dmtimer.o
obj-$(CONFIG_OMAP_BOOT_REASON) += bootreason.o
obj-$(CONFIG_OMAP_COMPONENT_VERSION) += component-version.o
obj-$(CONFIG_OMAP_GPIO_SWITCH) += gpio-switch.o

# DSP subsystem
obj-y += dsp/
obj-$(CONFIG_OMAP_DSP) += mailbox.o
