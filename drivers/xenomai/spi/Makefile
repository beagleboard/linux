
ccflags-$(CONFIG_XENO_DRIVERS_SPI_DEBUG) := -DDEBUG

obj-$(CONFIG_XENO_DRIVERS_SPI) += xeno_spi.o

xeno_spi-y := spi-master.o spi-device.o

obj-$(CONFIG_XENO_DRIVERS_SPI_BCM2835) += xeno_spi_bcm2835.o

xeno_spi_bcm2835-y := spi-bcm2835.o
