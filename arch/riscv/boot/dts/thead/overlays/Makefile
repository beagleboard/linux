# Overlays for the CONFIG_SOC_THEAD platform

dtbo-$(CONFIG_SOC_THEAD) += \
	BBORG_LOAD-00A2.dtbo	\
	BBORG_RELAY-00A2.dtbo	\
	BONE-LED_P8_03.dtbo	\
	BONE-LED_P9_11.dtbo	\
	BVA-MIKROBUS-0.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
