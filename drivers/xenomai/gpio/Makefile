
ccflags-$(CONFIG_XENO_DRIVERS_GPIO_DEBUG) := -DDEBUG

obj-$(CONFIG_XENO_DRIVERS_GPIO) += xeno_gpio.o

xeno_gpio-y := gpio-core.o

xeno_gpio-$(CONFIG_XENO_DRIVERS_GPIO_BCM2835) += gpio-bcm2835.o
xeno_gpio-$(CONFIG_XENO_DRIVERS_GPIO_MXC) += gpio-mxc.o
xeno_gpio-$(CONFIG_XENO_DRIVERS_GPIO_SUN8I_H3) += gpio-sun8i-h3.o
