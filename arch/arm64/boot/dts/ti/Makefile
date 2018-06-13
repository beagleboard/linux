# SPDX-License-Identifier: GPL-2.0
#
# Make file to build device tree binaries for boards based on
# Texas Instruments Inc processors
#
# Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
#

DTC_FLAGS += -@

dtb-$(CONFIG_ARCH_K3_AM6_SOC) += k3-am654-base-board.dtb \
	k3-am654-idk.dtbo \
	k3-am654-gp.dtbo \
	k3-am654-pcie-usb3.dtbo \
	k3-am654-evm-csi2-ov490.dtbo \
	k3-am654-evm-oldi-lcd1evm.dtbo \
	k3-am654-pcie-usb2.dtbo

$(obj)/%.dtbo: $(src)/%.dtso FORCE
	$(call if_changed_dep,dtc)

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
