ccflags-y += -I$(srctree)/drivers/xenomai/net/stack/include

obj-$(CONFIG_XENO_DRIVERS_NET_RTIPV4) += rtipv4.o

rtipv4-y := \
	route.o \
	protocol.o \
	arp.o \
	af_inet.o \
	ip_input.o \
	ip_sock.o \
	ip_output.o \
	ip_fragment.o

obj-$(CONFIG_XENO_DRIVERS_NET_RTIPV4_UDP) += udp/

obj-$(CONFIG_XENO_DRIVERS_NET_RTIPV4_TCP) += tcp/

rtipv4-$(CONFIG_XENO_DRIVERS_NET_RTIPV4_ICMP) += icmp.o
