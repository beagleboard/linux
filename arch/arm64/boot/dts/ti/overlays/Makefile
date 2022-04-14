# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_K3) += \
	robotics-cape.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
