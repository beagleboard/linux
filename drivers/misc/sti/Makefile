ifeq ($(CONFIG_ARCH_OMAP3),y)
obj-$(CONFIG_OMAP_STI)	+= sdti.o
else
obj-$(CONFIG_OMAP_STI)	+= sti.o sti-fifo.o
endif

obj-$(CONFIG_NET)	+= sti-netlink.o
obj-$(CONFIG_OMAP_STI_CONSOLE)	+= sti-console.o
