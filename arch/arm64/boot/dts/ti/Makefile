# SPDX-License-Identifier: GPL-2.0
#
# Make file to build device tree binaries for boards based on
# Texas Instruments Inc processors
#
# Copyright (C) 2016-2021 Texas Instruments Incorporated - https://www.ti.com/
#
# Entries are grouped as per SoC present on the board. Groups are sorted
# alphabetically.

# Boards with AM62x SoC
dtb-$(CONFIG_ARCH_K3) += k3-am625-beagleplay.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-beagleplay-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-beagleplay-csi2-tevi-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-beagleplay-lincolntech-lcd185-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-phyboard-lyra-rdk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-ecap-capture.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-lincolntech-lcd185-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-m2-cc3301.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-microtips-mf101hie-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-rpi-hdr-ehrpwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-nonwifi-dahlia.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-nonwifi-dev.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-nonwifi-mallow.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-nonwifi-yavia.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-wifi-dahlia.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-wifi-dev.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-wifi-mallow.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am625-verdin-wifi-yavia.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am62x-phyboard-lyra-gpio-fan.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-e3-max-opp.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62-lp-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am62-lp-sk-lincolntech-lcd185-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62-lp-sk-microtips-mf101hie-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am625-sk-mcspi-loopback.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62-lp-sk-nand.dtbo

# Boards with AM62Ax SoC
k3-am62a7-sk-ub954-evm-ov2312-dtbs := k3-am62a7-sk.dtb \
				      k3-am62a7-sk-ub954-evm.dtbo \
				      k3-fpdlink-ov2312-0-0.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-csi2-ox05b1s.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-ethernet-dc01.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-m2-cc3301.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-rpi-hdr-ehrpwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62a7-sk-ub954-evm-ov2312.dtb

# Boards with AM62Px SoC
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-dsi-rpi-7inch-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-dss-shared-mode.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-eqep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-ethfw.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-m2-cc3301.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-mcan.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-microtips-mf070zima-lcd3.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-microtips-mf101hie-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-microtips-mf103hie-lcd2.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-pwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62p5-sk-rpi-hdr-ehrpwm.dtbo

# Common overlays for SK-AM62* family of boards
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-csi2-tevi-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-csi2-imx219.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-csi2-v3link-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-dmtimer-pwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-eqep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-fastboot-disable-hdmi.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-hdmi-audio.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-mcan.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-microtips-mf103hie-lcd2.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-lpm-wkup-sources.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am62x-sk-pwm.dtbo

# Boards with AM64x SoC
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm-nand.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm-icssg1-dualemac.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm-icssg1-dualemac-mii.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm-pcie0-ep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-evm-sk-cpsw3g-pps.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-phyboard-electra-rdk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am642-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am642-sk-pwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am642-tqma64xxl-mbax4xxl.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am64-tqma64xxl-mbax4xxl-sdcard.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am64-tqma64xxl-mbax4xxl-wlan.dtbo

# Boards with AM65x SoC
k3-am654-gp-evm-dtbs := k3-am654-base-board.dtb \
	k3-am654-base-board-rocktech-rk101-panel.dtbo \
	k3-am654-pcie-usb3.dtbo
k3-am654-evm-dtbs := k3-am654-base-board.dtb k3-am654-icssg2.dtbo
k3-am654-idk-dtbs := k3-am654-evm.dtb k3-am654-idk.dtbo k3-am654-pcie-usb2.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am6528-iot2050-basic.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am6528-iot2050-basic-pg2.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am6548-iot2050-advanced.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am6548-iot2050-advanced-m2.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am6548-iot2050-advanced-pg2.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am654-base-board.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am654-base-board-toshiba-tc358867-evm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am654-gp-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am654-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am654-idk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am654-base-board-rocktech-rk101-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am654-pcie-usb2.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am654-pcie-usb3.dtbo

# Boards with J7200 SoC
k3-j7200-evm-dtbs := k3-j7200-common-proc-board.dtb k3-j7200-evm-quad-port-eth-exp.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j7200-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j7200-evm-ethfw.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j7200-evm-mcspi-loopback.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j7200-evm-pcie1-ep.dtbo

# Boards with J721e SoC
k3-j721e-evm-dtbs := k3-j721e-common-proc-board.dtb k3-j721e-evm-quad-port-eth-exp.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-dsi-rpi-7inch-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-microtips-mf070zima-lcd3.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-common-proc-board-infotainment.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-ethfw.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-gesi-exp-board.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-pcie0-ep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-pcie1-ep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-evm-ina2xx.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j721e-sk-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-sk-csi2-dual-imx219.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-sk-fpdlink-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-sk-rpi-hdr-ehrpwm.dtbo

# Boards with J721s2 SoC
dtb-$(CONFIG_ARCH_K3) += k3-am68-sk-base-board.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am68-sk-bb-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am68-sk-rpi-hdr-ehrpwm.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am68-sk-v3link-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-common-proc-board.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-evm-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-evm-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-evm-gesi-exp-board.dtbo
k3-j721s2-evm-dtbs := k3-j721s2-common-proc-board.dtb k3-j721s2-evm-gesi-exp-board.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j721s2-evm-pcie1-ep.dtbo

# Boards with J722s SoC
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-csi2-quad-rpi-cam-imx219.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-csi2-quad-tevi-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-dsi-rpi-7inch-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-fpdlink-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-microtips-mf101hie-panel.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-v3link-fusion.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j722s-evm-pwm.dtbo

# Boards with J784s4 SoC
dtb-$(CONFIG_ARCH_K3) += k3-am69-sk.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am69-sk-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am69-sk-fpdlink-fusion-auxport.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm-csi2-ov5640.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm-ethfw.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm-pcie0-pcie1-ep.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm-quad-port-eth-exp1.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j784s4-evm-usxgmii-exp1-exp2.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am69-sk-rpi-hdr-ehrpwm.dtbo

# Boards with J742S2 SoC
dtb-$(CONFIG_ARCH_K3) += k3-j742s2-evm.dtb
dtb-$(CONFIG_ARCH_K3) += k3-j742s2-evm-ethfw.dtbo

# Build time test only, enabled by CONFIG_OF_ALL_DTBS
k3-am625-beagleplay-csi2-ov5640-dtbs := k3-am625-beagleplay.dtb \
	k3-am625-beagleplay-csi2-ov5640.dtbo
k3-am625-beagleplay-csi2-tevi-ov5640-dtbs := k3-am625-beagleplay.dtb \
	k3-am625-beagleplay-csi2-tevi-ov5640.dtbo
k3-am625-beagleplay-lincolntech-lcd185-panel-dtbs := k3-am625-beagleplay.dtb \
	k3-am625-beagleplay-lincolntech-lcd185-panel.dtbo
k3-am625-phyboard-lyra-gpio-fan-dtbs := k3-am625-phyboard-lyra-rdk.dtb \
	k3-am62x-phyboard-lyra-gpio-fan.dtbo
k3-am625-sk-csi2-imx219-dtbs := k3-am625-sk.dtb \
	k3-am62x-sk-csi2-imx219.dtbo
k3-am625-sk-csi2-ov5640-dtbs := k3-am625-sk.dtb \
	k3-am62x-sk-csi2-ov5640.dtbo
k3-am625-sk-csi2-tevi-ov5640-dtbs := k3-am625-sk.dtb \
	k3-am62x-sk-csi2-tevi-ov5640.dtbo
k3-am625-sk-fastboot-disable-hdmi-dtbs := k3-am625-sk.dtb \
	k3-am62x-sk-fastboot-disable-hdmi.dtbo
k3-am625-sk-hdmi-audio-dtbs := k3-am625-sk.dtb k3-am62x-sk-hdmi-audio.dtbo
k3-am625-sk-lincolntech-lcd185-panel-dtbs := k3-am625-sk.dtb \
	k3-am625-sk-lincolntech-lcd185-panel.dtbo
k3-am625-sk-microtips-mf101hie-panel-dtbs := k3-am625-sk.dtb \
	k3-am625-sk-microtips-mf101hie-panel.dtbo
k3-am625-sk-microtips-mf103hie-lcd2-dtbs := k3-am625-sk.dtb \
	k3-am62x-sk-microtips-mf103hie-lcd2.dtbo
k3-am62-lp-sk-fastboot-disable-hdmi-dtbs := k3-am62-lp-sk.dtb \
	k3-am62x-sk-fastboot-disable-hdmi.dtbo
k3-am62-lp-sk-hdmi-audio-dtbs := k3-am62-lp-sk.dtb k3-am62x-sk-hdmi-audio.dtbo
k3-am62-lp-sk-lincolntech-lcd185-panel-dtbs := k3-am62-lp-sk.dtb \
	k3-am62-lp-sk-lincolntech-lcd185-panel.dtbo
k3-am62-lp-sk-microtips-mf101hie-panel-dtbs := k3-am62-lp-sk.dtb \
	k3-am62-lp-sk-microtips-mf101hie-panel.dtbo
k3-am62-lp-sk-microtips-mf103hie-lcd2-dtbs := k3-am62-lp-sk.dtb \
	k3-am62x-sk-microtips-mf103hie-lcd2.dtbo
k3-am62a7-sk-csi2-imx219-dtbs := k3-am62a7-sk.dtb \
	k3-am62x-sk-csi2-imx219.dtbo
k3-am62a7-sk-csi2-ov5640-dtbs := k3-am62a7-sk.dtb \
	k3-am62x-sk-csi2-ov5640.dtbo
k3-am62a7-sk-csi2-tevi-ov5640-dtbs := k3-am62a7-sk.dtb \
	k3-am62x-sk-csi2-tevi-ov5640.dtbo
k3-am62a7-sk-ethernet-dc01-dtbs := k3-am62a7-sk.dtb \
	k3-am62a7-sk-ethernet-dc01.dtbo
k3-am62a7-sk-fastboot-disable-hdmi-dtbs := k3-am62a7-sk.dtb \
	k3-am62x-sk-fastboot-disable-hdmi.dtbo
k3-am62a7-sk-hdmi-audio-dtbs := k3-am62a7-sk.dtb k3-am62x-sk-hdmi-audio.dtbo
k3-am62p5-sk-csi2-imx219-dtbs := k3-am62p5-sk.dtb \
	k3-am62x-sk-csi2-imx219.dtbo
k3-am62p5-sk-csi2-ov5640-dtbs := k3-am62p5-sk.dtb \
	k3-am62x-sk-csi2-ov5640.dtbo
k3-am62p5-sk-csi2-tevi-ov5640-dtbs := k3-am62p5-sk.dtb \
	k3-am62x-sk-csi2-tevi-ov5640.dtbo
k3-am62p5-sk-fastboot-disable-hdmi-dtbs := k3-am62p5-sk.dtb \
	k3-am62x-sk-fastboot-disable-hdmi.dtbo
k3-am62p5-sk-dsi-rpi-7inch-panel-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-dsi-rpi-7inch-panel.dtbo
k3-am62p5-sk-dss-shared-mode-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-dss-shared-mode.dtbo
k3-am62p5-sk-ethfw-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-ethfw.dtbo
k3-am62p5-sk-microtips-mf070zima-lcd3-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-microtips-mf070zima-lcd3.dtbo
k3-am62p5-sk-microtips-mf101hie-panel-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-microtips-mf101hie-panel.dtbo
k3-am62p5-sk-microtips-mf103hie-lcd2-dtbs := k3-am62p5-sk.dtb \
	k3-am62p5-sk-microtips-mf103hie-lcd2.dtbo
k3-am642-evm-icssg1-dualemac-dtbs := \
	k3-am642-evm.dtb k3-am642-evm-icssg1-dualemac.dtbo
k3-am642-evm-icssg1-dualemac-mii-dtbs := \
	k3-am642-evm.dtb k3-am642-evm-icssg1-dualemac-mii.dtbo
k3-am642-evm-pcie0-ep-dtbs := \
	k3-am642-evm.dtb k3-am642-evm-pcie0-ep.dtbo
k3-am642-evm-cpsw3g-pps.dtbs := \
	k3-am642-evm.dtb k3-am642-evm-sk-cpsw3g-pps.dtbo
k3-am642-sk-cpsw3g-pps.dtbs := \
	k3-am642-sk.dtb k3-am642-evm-sk-cpsw3g-pps.dtbo
k3-am642-tqma64xxl-mbax4xxl-sdcard-dtbs := \
	k3-am642-tqma64xxl-mbax4xxl.dtb k3-am64-tqma64xxl-mbax4xxl-sdcard.dtbo
k3-am642-tqma64xxl-mbax4xxl-wlan-dtbs := \
	k3-am642-tqma64xxl-mbax4xxl.dtb k3-am64-tqma64xxl-mbax4xxl-wlan.dtbo
k3-am654-base-board-toshiba-tc358867-evm-dtbs := k3-am654-base-board.dtb \
	k3-am654-base-board-toshiba-tc358867-evm.dtbo
k3-am68-sk-base-board-csi2-dual-imx219-dtbs := k3-am68-sk-base-board.dtb \
	k3-j721e-sk-csi2-dual-imx219.dtbo
k3-am69-sk-csi2-dual-imx219-dtbs := k3-am69-sk.dtb \
	k3-j721e-sk-csi2-dual-imx219.dtbo
k3-j7200-evm-ethfw-dtbs := k3-j7200-common-proc-board.dtb \
	k3-j7200-evm-ethfw.dtbo
k3-j7200-evm-mcspi-loopback-dtbs := k3-j7200-common-proc-board.dtb \
	k3-j7200-evm-mcspi-loopback.dtbo
k3-j7200-evm-pcie1-ep-dtbs := k3-j7200-common-proc-board.dtb \
	k3-j7200-evm-pcie1-ep.dtbo
k3-j721e-beagleboneai64-dsi-rpi-7inch-panel-dtbs := k3-j721e-beagleboneai64.dtb \
	k3-j721e-beagleboneai64-dsi-rpi-7inch-panel.dtbo
k3-j721e-beagleboneai64-microtips-mf070zima-lcd3-dtbs := k3-j721e-beagleboneai64.dtb \
	k3-j721e-beagleboneai64-microtips-mf070zima-lcd3.dtbo
k3-j721e-common-proc-board-infotainment-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j721e-common-proc-board-infotainment.dtbo
k3-j721e-evm-ethfw-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j721e-evm-ethfw.dtbo
k3-j721e-evm-mcspi-loopback-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j7200-evm-mcspi-loopback.dtbo
k3-j721e-evm-pcie0-ep-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j721e-evm-pcie0-ep.dtbo
k3-j721e-evm-pcie1-ep-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j721e-evm-pcie1-ep.dtbo
k3-j721e-evm-ina2xx-dtbs := k3-j721e-common-proc-board.dtb \
	k3-j721e-evm-ina2xx.dtbo
k3-j721e-sk-csi2-dual-imx219-dtbs := k3-j721e-sk.dtb \
	k3-j721e-sk-csi2-dual-imx219.dtbo
k3-j721s2-evm-mcspi-loopback-dtbs := k3-j721s2-common-proc-board.dtb \
	k3-j7200-evm-mcspi-loopback.dtbo
k3-j721s2-evm-pcie1-ep-dtbs := k3-j721s2-common-proc-board.dtb \
	k3-j721s2-evm-pcie1-ep.dtbo
k3-j722s-evm-dsi-rpi-7inch-panel-dtbs := k3-j722s-evm.dtb \
	k3-j722s-evm-dsi-rpi-7inch-panel.dtbo
k3-j722s-evm-microtips-mf101hie-panel-dtbs := k3-j722s-evm.dtb \
	k3-j722s-evm-microtips-mf101hie-panel.dtbo
k3-j742s2-evm-ethfw-dtbs := k3-j742s2-evm.dtb \
	k3-j742s2-evm-ethfw.dtbo
k3-j784s4-evm-ethfw-dtbs := k3-j784s4-evm.dtb \
	k3-j784s4-evm-ethfw.dtbo
k3-j784s4-evm-mcspi-loopback-dtbs := k3-j784s4-evm.dtb \
	k3-j7200-evm-mcspi-loopback.dtbo
k3-j784s4-evm-pcie0-pcie1-ep-dtbs := k3-j784s4-evm.dtb \
	k3-j784s4-evm-pcie0-pcie1-ep.dtbo
k3-j784s4-evm-quad-port-eth-exp1-dtbs := k3-j784s4-evm.dtb \
	k3-j784s4-evm-quad-port-eth-exp1.dtbo
k3-j784s4-evm-usxgmii-exp1-exp2-dtbs := k3-j784s4-evm.dtb \
	k3-j784s4-evm-usxgmii-exp1-exp2.dtbo
dtb- += k3-am625-beagleplay-csi2-ov5640.dtb \
	k3-am625-beagleplay-csi2-tevi-ov5640.dtb \
	k3-am625-beagleplay-lincolntech-lcd185-panel.dtb \
	k3-am625-sk-csi2-imx219.dtb \
	k3-am625-sk-csi2-ov5640.dtb \
	k3-am625-sk-csi2-tevi-ov5640.dtb \
	k3-am625-sk-fastboot-disable-hdmi.dtb \
	k3-am625-sk-hdmi-audio.dtb \
	k3-am625-sk-lincolntech-lcd185-panel.dtb \
	k3-am625-sk-microtips-mf101hie-panel.dtb \
	k3-am625-sk-microtips-mf103hie-lcd2.dtb \
	k3-am62-lp-sk-fastboot-disable-hdmi.dtb \
	k3-am62-lp-sk-hdmi-audio.dtb \
	k3-am62-lp-sk-lincolntech-lcd185-panel.dtb \
	k3-am62-lp-sk-microtips-mf101hie-panel.dtb \
	k3-am62-lp-sk-microtips-mf103hie-lcd2.dtb \
	k3-am62a7-sk-csi2-imx219.dtb \
	k3-am62a7-sk-csi2-ov5640.dtb \
	k3-am62a7-sk-fastboot-disable-hdmi.dtb \
	k3-am62a7-sk-hdmi-audio.dtb \
	k3-am62p5-sk-csi2-imx219.dtb \
	k3-am62p5-sk-csi2-ov5640.dtb \
	k3-am62p5-sk-csi2-tevi-ov5640.dtb \
	k3-am62p5-sk-dsi-rpi-7inch-panel.dtb \
	k3-am62p5-sk-dss-shared-mode.dtb \
	k3-am62p5-sk-ethfw.dtb \
	k3-am62p5-sk-fastboot-disable-hdmi.dtb \
	k3-am62p5-sk-microtips-mf070zima-lcd3.dtb \
	k3-am62p5-sk-microtips-mf101hie-panel.dtb \
	k3-am62p5-sk-microtips-mf103hie-lcd2.dtb \
	k3-am642-evm-icssg1-dualemac.dtb \
	k3-am642-evm-icssg1-dualemac-mii.dtb \
	k3-am642-evm-pcie0-ep.dtb \
	k3-am642-evm-cpsw3g-pps.dtb \
	k3-am642-sk-cpsw3g-pps.dtb \
	k3-am642-tqma64xxl-mbax4xxl-sdcard.dtb \
	k3-am642-tqma64xxl-mbax4xxl-wlan.dtb \
	k3-am654-base-board-toshiba-tc358867-evm.dtb \
	k3-am68-sk-base-board-csi2-dual-imx219-dtbs \
	k3-am69-sk-csi2-dual-imx219-dtbs \
	k3-j7200-evm-ethfw.dtb \
	k3-j7200-evm-mcspi-loopback.dtb \
	k3-j7200-evm-pcie1-ep.dtbo \
	k3-j721e-beagleboneai64-dsi-rpi-7inch-panel.dtb \
	k3-j721e-beagleboneai64-microtips-mf070zima-lcd3.dtb \
	k3-j721e-common-proc-board-infotainment.dtb \
	k3-j721e-evm-ethfw.dtb \
	k3-j721e-evm-mcspi-loopback.dtb \
	k3-j721e-evm-pcie0-ep.dtb \
	k3-j721e-evm-pcie1-ep.dtb \
	k3-j721e-evm-ina2xx.dtb \
	k3-j721e-sk-csi2-dual-imx219-dtbs \
	k3-j721s2-evm-pcie1-ep.dtb \
	k3-j721s2-evm-mcspi-loopback.dtb \
	k3-j722s-evm-dsi-rpi-7inch-panel.dtb \
	k3-j722s-evm-microtips-mf101hie-panel.dtb \
	k3-j742s2-evm-ethfw.dtb \
	k3-j784s4-evm-ethfw.dtb \
	k3-j784s4-evm-mcspi-loopback.dtb \
	k3-j784s4-evm-pcie0-pcie1-ep.dtb \
	k3-j784s4-evm-quad-port-eth-exp1.dtb \
	k3-j784s4-evm-usxgmii-exp1-exp2.dtb

# FPDLink Sensors
dtb-$(CONFIG_ARCH_K3) += k3-fpdlink-imx390-rcm-0-0.dtbo \
			 k3-fpdlink-imx390-rcm-0-1.dtbo \
			 k3-fpdlink-imx390-rcm-0-2.dtbo \
			 k3-fpdlink-imx390-rcm-0-3.dtbo \
			 k3-fpdlink-imx390-rcm-1-0.dtbo \
			 k3-fpdlink-imx390-rcm-1-1.dtbo \
			 k3-fpdlink-imx390-rcm-1-2.dtbo \
			 k3-fpdlink-imx390-rcm-1-3.dtbo \
			 k3-fpdlink-imx390-rcm-2-0.dtbo \
			 k3-fpdlink-imx390-rcm-2-1.dtbo \
			 k3-fpdlink-imx390-rcm-2-2.dtbo \
			 k3-fpdlink-imx390-rcm-2-3.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-fpdlink-ov2312-0-0.dtbo \
			 k3-fpdlink-ov2312-0-1.dtbo \
			 k3-fpdlink-ov2312-0-2.dtbo \
			 k3-fpdlink-ov2312-0-3.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-v3link-imx219-0-0.dtbo \
			 k3-v3link-imx219-0-1.dtbo \
			 k3-v3link-imx219-0-2.dtbo \
			 k3-v3link-imx219-0-3.dtbo \
			 k3-v3link-imx219-1-0.dtbo \
			 k3-v3link-imx219-1-1.dtbo \
			 k3-v3link-imx219-1-2.dtbo \
			 k3-v3link-imx219-1-3.dtbo \
			 k3-v3link-ov5640-0-0.dtbo \
			 k3-v3link-ov5640-0-1.dtbo \
			 k3-v3link-ov5640-0-2.dtbo \
			 k3-v3link-ov5640-0-3.dtbo \
			 k3-v3link-tevi-ov5640-0-0.dtbo \
			 k3-v3link-tevi-ov5640-0-1.dtbo \
			 k3-v3link-tevi-ov5640-0-2.dtbo \
			 k3-v3link-tevi-ov5640-0-3.dtbo

dtb-$(CONFIG_ARCH_K3) += k3-am625-pocketbeagle2.dtb
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai.dtb
dtb-$(CONFIG_ARCH_K3) += BONE-I2C1.dtbo
dtb-$(CONFIG_ARCH_K3) += BONE-I2C2.dtbo
dtb-$(CONFIG_ARCH_K3) += BONE-I2C3.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-i2c1-400000.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-i2c1-ads1115.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-i2c1-ssd1306.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-mikroe-eth.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-mikroe-microsd.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-ecap0-gpio12.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-ecap1-gpio16.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-ecap1-gpio21.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-ecap2-gpio17.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-ecap2-gpio18.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio12.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio14.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio15.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio15-gpio12.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio15-gpio14.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio5.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio5-gpio12.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm0-gpio5-gpio14.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio13.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio20.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio21.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio21-gpio13.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio21-gpio20.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio6.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio6-gpio13.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-pwm-epwm1-gpio6-gpio20.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-spi0-1cs.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-spi0-2cs.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-spidev0.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-am67a-beagley-ai-uart-ttyama0.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-BBORG_MOTOR.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm0-p8_13.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm0-p8_13-p8_19.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm0-p8_19.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm2-p9_14.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm2-p9_14-p9_16.dtbo
dtb-$(CONFIG_ARCH_K3) += k3-j721e-beagleboneai64-pwm-epwm2-p9_16.dtbo

# Enable support for device-tree overlays
DTC_FLAGS_k3-am625-beagleplay += -@
DTC_FLAGS_k3-am625-sk += -@
DTC_FLAGS_k3-am62-lp-sk += -@
DTC_FLAGS_k3-am62x-sk-csi2-v3link-fusion += -@
DTC_FLAGS_k3-am62a7-sk-ub954-evm += -@
DTC_FLAGS_k3-am62a7-sk += -@
DTC_FLAGS_k3-am62p5-sk += -@
DTC_FLAGS_k3-am642-evm += -@
DTC_FLAGS_k3-am642-sk += -@
DTC_FLAGS_k3-am642-tqma64xxl-mbax4xxl += -@
DTC_FLAGS_k3-am6548-iot2050-advanced-m2 += -@
DTC_FLAGS_k3-am62p5-sk += -@
DTC_FLAGS_k3-am68-sk-base-board += -@
DTC_FLAGS_k3-am68-sk-v3link-fusion += -@
DTC_FLAGS_k3-am69-sk += -@
DTC_FLAGS_k3-am69-sk-fpdlink-fusion-auxport += -@
DTC_FLAGS_k3-j7200-common-proc-board += -@
DTC_FLAGS_k3-j721e-beagleboneai64 += -@
DTC_FLAGS_k3-j721e-evm-fusion += -@
DTC_FLAGS_k3-j721e-common-proc-board += -@
DTC_FLAGS_k3-j721e-evm-pcie0-ep += -@
DTC_FLAGS_k3-j721e-sk += -@
DTC_FLAGS_k3-j721e-sk-fpdlink-fusion += -@
DTC_FLAGS_k3-j721s2-common-proc-board += -@
DTC_FLAGS_k3-j721s2-evm-fusion += -@
DTC_FLAGS_k3-j722s-evm += -@
DTC_FLAGS_k3-j722s-evm-fpdlink-fusion += -@
DTC_FLAGS_k3-j722s-evm-v3link-fusion += -@
DTC_FLAGS_k3-j784s4-evm += -@
DTC_FLAGS_k3-j742s2-evm += -@
DTC_FLAGS_k3-j742s2-evm-ethfw += -@
DTC_FLAGS_k3-am625-pocketbeagle2 += -@
DTC_FLAGS_k3-am67a-beagley-ai += -@
DTC_FLAGS_k3-j721e-beagleboneai64 += -@
