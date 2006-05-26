#
# Makefile for the OMAP touchscreen input driver
#

obj-$(CONFIG_TOUCHSCREEN_OMAP) += omapts.o

objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_H2) += ts_hx.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_H3) += ts_hx.o

omapts-objs := omap_ts.o $(objs-yy)
