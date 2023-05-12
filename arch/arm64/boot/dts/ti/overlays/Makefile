# Overlays for the BeagleBone platform

dtbo-$(CONFIG_ARCH_K3) += \
	BBAI64-CSI0-imx219.dtbo \
	BBAI64-CSI1-imx219.dtbo \
	BBAI64-DSI-RPi-7inch-panel.dtbo \
	BBAI64-P8_37-ehrpwm5_a.dtbo \
	BBAI64-P9_25-ehrpwm4_b.dtbo \
	BB-I2C2-MPU6050.dtbo \
	BBORG_LOAD-00A2.dtbo \
	BBORG_RELAY-00A2.dtbo \
	BBORG_SERVO-00A2.dtbo \
	BONE-FAN.dtbo \
	BONE-I2C1.dtbo \
	BONE-I2C2.dtbo \
	BONE-I2C3.dtbo \
	BONE-LED_P8_03.dtbo \
	BONE-LED_P9_11.dtbo \
	BONE-PWM0.dtbo \
	BONE-PWM1.dtbo \
	BONE-PWM2.dtbo \
	BONE-SPI0_0.dtbo \
	BONE-SPI0_1.dtbo \
	BONE-SPI1_0.dtbo \
	BONE-UART1.dtbo \
	BONE-USB0-host.dtbo \
	J721E-PRU-UIO-00A0.dtbo \
	k3-j721e-beagleboneai64-RPi-7inch-panel.dtbo \
	robotics-cape.dtbo \
	k3-am625-beagleplay-bcfserial-no-firmware.dtbo \
	k3-am625-beagleplay-csi2-ov5640.dtbo \
	k3-am625-beagleplay-lt-lcd185.dtbo


targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
