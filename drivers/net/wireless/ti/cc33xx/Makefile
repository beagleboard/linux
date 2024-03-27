# SPDX-License-Identifier: GPL-2.0

cc33xx-objs		= main.o cmd.o io.o event.o tx.o rx.o ps.o acx.o \
			  boot.o init.o debugfs.o scan.o sysfs.o

cc33xx_spi-objs 	= spi.o
cc33xx_sdio-objs	= sdio.o

cc33xx-$(CONFIG_NL80211_TESTMODE)	+= testmode.o
obj-$(CONFIG_CC33XX)				+= cc33xx.o
obj-$(CONFIG_CC33XX_SPI)			+= cc33xx_spi.o
obj-$(CONFIG_CC33XX_SDIO)			+= cc33xx_sdio.o
