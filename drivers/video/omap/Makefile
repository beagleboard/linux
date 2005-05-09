#
# Makefile for the new OMAP framebuffer device driver
#

obj-$(CONFIG_FB_OMAP) += omapfb.o

objs-yy := omapfb_main.o
objs-y$(CONFIG_FB_OMAP_INTERNAL_LCDC) += omap_lcdc.o
objs-y$(CONFIG_FB_OMAP_EXTERNAL_LCDC) += sossi.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_H3) += lcd_h3.o
objs-y$(CONFIG_MACH_OMAP_H2) += lcd_h2.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_INNOVATOR) += lcd_inn1610.o
objs-$(CONFIG_ARCH_OMAP1510)$(CONFIG_MACH_OMAP_INNOVATOR) += lcd_inn1510.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_OSK) += lcd_osk.o
objs-$(CONFIG_ARCH_OMAP730)$(CONFIG_MACH_OMAP_PERSEUS2) += lcd_p2.o

omapfb-objs := $(objs-yy)

