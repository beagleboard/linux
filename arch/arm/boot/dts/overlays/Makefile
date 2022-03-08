# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_OMAP2PLUS) += \
	BB-ADC-00A0.dtbo	\
	BB-BBBW-WL1835-00A0.dtbo	\
	BB-BBGG-WL1835-00A0.dtbo	\
	BB-BBGW-WL1835-00A0.dtbo	\
	BB-BONE-4D5R-01-00A1.dtbo	\
	BB-BONE-eMMC1-01-00A0.dtbo	\
	BB-BONE-LCD4-01-00A1.dtbo	\
	BB-BONE-NH7C-01-A0.dtbo	\
	BB-CAPE-DISP-CT4-00A0.dtbo	\
	BB-HDMI-TDA998x-00A0.dtbo	\
	BB-I2C2-BME680.dtbo	\
	BB-SPIDEV0-00A0.dtbo	\
	BB-SPIDEV1-00A0.dtbo	\
	BB-W1-P9.12-00A0.dtbo	\
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

always		:= $(dtbo-y)
clean-files	:= *.dtbo
