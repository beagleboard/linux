#
# Makefile for the OMAP touchscreen input driver
#

obj-$(CONFIG_TOUCHSCREEN_OMAP) += omapts.o

objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_H2) += ts_hx.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_H3) += ts_hx.o
objs-$(CONFIG_ARCH_OMAP15XX)$(CONFIG_MACH_OMAP_INNOVATOR) += ts_inn1510.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_OSK) += ts_osk.o

omapts-objs := omap_ts.o $(objs-yy)
