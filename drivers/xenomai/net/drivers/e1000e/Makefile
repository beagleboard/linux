ccflags-y += -I$(srctree)/drivers/xenomai/net/stack/include

obj-$(CONFIG_XENO_DRIVERS_NET_DRV_E1000E) += rt_e1000e.o

rt_e1000e-y := \
	82571.o \
	80003es2lan.o \
	ich8lan.o \
	lib.o \
	netdev.o \
	param.o \
	phy.o
