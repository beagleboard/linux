# Overlays for the BeagleBone platform

dtbo-$(CONFIG_SOC_AM33XX) += \
	BB-SPIDEV0-00A0.dtbo \
	BB-SPIDEV1-00A0.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtbo
