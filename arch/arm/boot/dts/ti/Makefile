# SPDX-License-Identifier: GPL-2.0
#
# Makefile to build device tree overlay binaries for boards based on
# Texas Instruments Inc processors
#
# Copyright (C) 2017-2021 Texas Instruments Incorporated - https://www.ti.com/
#

DTC_FLAGS += -@

# base dtbs with symbols enabled
dtb-$(CONFIG_SOC_DRA7XX) += \
	am571x-idk.dtb \
	am572x-idk.dtb \
	am574x-idk.dtb \
	am57xx-beagle-x15.dtb \
	am57xx-beagle-x15-revb1.dtb \
	am57xx-beagle-x15-revc.dtb \
	dra71-evm.dtb \
	dra72-evm.dtb \
	dra72-evm-revc.dtb \
	dra76-evm.dtb \
	dra7-evm.dtb

# overlays
dtb-$(CONFIG_SOC_DRA7XX) += \
	am57xx-evm-common.dtbo \
	am57xx-evm-reva3.dtbo \
	lcd-osd101t2045.dtbo \
	lcd-osd101t2587.dtbo \
	dra72-evm-touchscreen.dtbo \
	dra74-evm-touchscreen.dtbo \
	dra7x-evm-osd-lcd-common.dtbo \
	am571x-idk-touchscreen.dtbo \
	am572x-idk-touchscreen.dtbo \
	am57xx-idk-osd-lcd-common.dtbo \
	dra71-evm-lcd-auo-g101evn01.0.dtbo \
	dra76-evm-tfp410.dtbo \
	dra71-evm-nand.dtb \
	ov10635.dtbo \
	ov2659-am571x.dtbo \
	ov2659-am572x.dtbo

$(obj)/%.dtb: $(src)/../%.dts FORCE
	$(call if_changed_dep,dtc,dtb)

$(obj)/%.dt.yaml: $(src)/../%.dts $(DTC) $(DT_TMP_SCHEMA) FORCE
	$(call if_changed_rule,dtc,yaml)

clean-files	:= *.dtb *.dtbo

always-y	:= $(dtb-y)
