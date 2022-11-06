# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_K3) += \
	BB-I2C2-MPU6050.dtbo \
	BBAI64-DSI-RPi-7inch-panel.dtbo \
	BBAI64-P8_37-ehrpwm5_a.dtbo \
	BBAI64-P9_25-ehrpwm4_b.dtbo \
	BONE-I2C1.dtbo \
	BONE-I2C2.dtbo \
	BONE-I2C3.dtbo \
	BONE-FAN.dtbo \
	BONE-PWM0.dtbo \
	BONE-PWM1.dtbo \
	BONE-PWM2.dtbo \
	BONE-SPI0_0.dtbo \
	BONE-SPI0_1.dtbo \
	BONE-UART1.dtbo \
	BONE-USB0-host.dtbo \
	robotics-cape.dtbo \
	k3-j721e-beagleboneai64-RPi-7inch-panel.dtbo \
	J721E-PRU-UIO-00A0.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
