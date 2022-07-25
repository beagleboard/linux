# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_K3) += \
	robotics-cape.dtbo \
	k3-j721e-beagleboneai64-RPi-7inch-panel.dtbo \
	J721E-PRU-UIO-00A0.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
