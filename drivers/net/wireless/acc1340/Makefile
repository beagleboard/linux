acc1340_core-y := \
		fwio.o \
		txrx.o \
		main.o \
		queue.o \
		hwio.o \
		bh.o \
		wsm.o \
		sta.o \
		scan.o \
		debug.o
acc1340_core-$(CONFIG_PM)	+= pm.o

# CFLAGS_sta.o += -DDEBUG

acc1340_wlan_sdio-y := acc1340_sdio.o
acc1340_wlan_spi-y := acc1340_spi.o

obj-$(CONFIG_ACC1340) += acc1340_core.o
obj-$(CONFIG_ACC1340_WLAN_SDIO) += acc1340_wlan_sdio.o
obj-$(CONFIG_ACC1340_WLAN_SPI) += acc1340_wlan_spi.o
