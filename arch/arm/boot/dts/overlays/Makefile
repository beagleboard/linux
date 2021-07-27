# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_OMAP2PLUS) += \
	BB-ADC-00A0.dtbo	\
	BB-BBBW-WL1835-00A0.dtbo	\
	BB-BBGW-WL1835-00A0.dtbo	\
	BB-BONE-eMMC1-01-00A0.dtbo	\
	BB-HDMI-TDA998x-00A0.dtbo	\
	BBORG_FAN-A000.dtbo	\
	M-BB-BBG-00A0.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
