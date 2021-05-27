
ccflags-$(CONFIG_XENO_DRIVERS_SPI_DEBUG) := -DDEBUG

obj-$(CONFIG_XENO_DRIVERS_SPI) += xeno_spi.o

xeno_spi-y := spi-master.o spi-device.o

obj-$(CONFIG_XENO_DRIVERS_SPI_BCM2835) += xeno_spi_bcm2835.o
obj-$(CONFIG_XENO_DRIVERS_SPI_SUN6I) += xeno_spi_sun6i.o
obj-$(CONFIG_XENO_DRIVERS_SPI_OMAP2_MCSPI_RT) += xeno_spi_omap2_mcspi_rt.o

xeno_spi_bcm2835-y := spi-bcm2835.o
xeno_spi_sun6i-y := spi-sun6i.o
xeno_spi_omap2_mcspi_rt-y := spi-omap2-mcspi-rt.o
