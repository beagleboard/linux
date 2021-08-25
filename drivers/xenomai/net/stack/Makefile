ccflags-y += -I$(srctree)/drivers/xenomai/net/stack/include -I$(srctree)/kernel/

obj-$(CONFIG_XENO_DRIVERS_NET_RTIPV4) += ipv4/

obj-$(CONFIG_XENO_DRIVERS_NET_RTPACKET) += packet/

obj-$(CONFIG_XENO_DRIVERS_NET_RTMAC) += rtmac/

obj-$(CONFIG_XENO_DRIVERS_NET_RTCFG) += rtcfg/

obj-$(CONFIG_XENO_DRIVERS_NET) += rtnet.o

rtnet-y :=  \
	corectl.o \
	iovec.o \
	rtdev.o \
	rtdev_mgr.o \
	rtnet_chrdev.o \
	rtnet_module.o \
	rtnet_rtpc.o \
	rtskb.o \
	socket.o \
	stack_mgr.o \
	eth.o

rtnet-$(CONFIG_XENO_DRIVERS_NET_RTWLAN) += rtwlan.o
