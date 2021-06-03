ccflags-$(CONFIG_XENO_DRIVERS_GPIO_DEBUG) := -DDEBUG

obj-$(CONFIG_XENO_DRIVERS_GPIO_BCM2835) += xeno-gpio-bcm2835.o
obj-$(CONFIG_XENO_DRIVERS_GPIO_MXC) += xeno-gpio-mxc.o
obj-$(CONFIG_XENO_DRIVERS_GPIO_SUN8I_H3) += xeno-gpio-sun8i-h3.o
obj-$(CONFIG_XENO_DRIVERS_GPIO_ZYNQ7000) += xeno-gpio-zynq7000.o
obj-$(CONFIG_XENO_DRIVERS_GPIO_XILINX) += xeno-gpio-xilinx.o
obj-$(CONFIG_XENO_DRIVERS_GPIO_OMAP) += xeno-gpio-omap.o
obj-$(CONFIG_XENO_DRIVERS_GPIO) += gpio-core.o

xeno-gpio-bcm2835-y := gpio-bcm2835.o
xeno-gpio-mxc-y := gpio-mxc.o
xeno-gpio-sun8i-h3-y := gpio-sun8i-h3.o
xeno-gpio-zynq7000-y := gpio-zynq7000.o
xeno-gpio-xilinx-y := gpio-xilinx.o
xeno-gpio-omap-y := gpio-omap.o
