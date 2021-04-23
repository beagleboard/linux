ccflags-y += -I$(srctree)/drivers/xenomai/net/stack/include

obj-$(CONFIG_XENO_DRIVERS_NET_DRV_TULIP) += rt_tulip.o

rt_tulip-y := \
	tulip_core.o \
	eeprom.o \
	interrupt.o \
	media.o \
	21142.o \
	pnic.o \
	pnic2.o
