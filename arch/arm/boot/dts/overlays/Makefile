# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_OMAP2PLUS) += \
	AM335X-PRU-UIO-00A0.dtbo	\
	AM57XX-PRU-UIO-00A0.dtbo	\
	BB-ADC-00A0.dtbo	\
	BB-BBBW-WL1835-00A0.dtbo	\
	BB-BBGG-WL1835-00A0.dtbo	\
	BB-BBGW-WL1835-00A0.dtbo	\
	BB-BONE-4D4C-01-00A1.dtbo	\
	BB-BONE-4D5R-01-00A1.dtbo	\
	BB-BONE-eMMC1-01-00A0.dtbo	\
	BB-BONE-LCD4-01-00A1.dtbo	\
	BB-BONE-NH7C-01-A0.dtbo	\
	BB-CAPE-DISP-CT4-00A0.dtbo	\
	BB-HDMI-TDA998x-00A0.dtbo	\
	BB-I2C1-MCP7940X-00A0.dtbo	\
	BB-I2C1-RTC-DS3231.dtbo	\
	BB-I2C1-RTC-PCF8563.dtbo	\
	BB-I2C2-BME680.dtbo	\
	BB-I2C2-MPU6050.dtbo	\
	BB-LCD-ADAFRUIT-24-SPI1-00A0.dtbo	\
	BB-NHDMI-TDA998x-00A0.dtbo	\
	BB-SPIDEV0-00A0.dtbo	\
	BB-SPIDEV1-00A0.dtbo	\
	BB-W1-P9.12-00A0.dtbo	\
	BB-UART1-00A0.dtbo	\
	BB-UART2-00A0.dtbo	\
	BB-UART4-00A0.dtbo	\
	BBORG_COMMS-00A2.dtbo	\
	BBORG_FAN-A000.dtbo	\
	BBORG_RELAY-00A2.dtbo	\
	BONE-ADC.dtbo	\
	M-BB-BBG-00A0.dtbo	\
	M-BB-BBGG-00A0.dtbo	\
	PB-MIKROBUS-0.dtbo	\
	PB-MIKROBUS-1.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
