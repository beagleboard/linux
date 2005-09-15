#
# Makefile for the linux kernel.
#

# Common support
obj-y := irq.o io.o mux.o serial.o

obj-$(CONFIG_OMAP_MPU_TIMER)		+= timer-gp.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o

