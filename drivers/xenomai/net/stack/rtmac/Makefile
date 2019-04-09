ccflags-y += -Idrivers/xenomai/net/stack/include

obj-$(CONFIG_XENO_DRIVERS_NET_NOMAC) += nomac/

obj-$(CONFIG_XENO_DRIVERS_NET_TDMA) += tdma/

obj-$(CONFIG_XENO_DRIVERS_NET_RTMAC) += rtmac.o

rtmac-y := \
	rtmac_disc.o \
	rtmac_module.o \
	rtmac_proc.o \
	rtmac_proto.o \
	rtmac_syms.o \
	rtmac_vnic.o
