# SPDX-License-Identifier: GPL-2.0
#
# Make file to build device tree binaries for boards based on
# Texas Instruments Inc processors
#
# Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
#

DTC_FLAGS += -@

dtb-$(CONFIG_ARCH_K3_AM6_SOC) += k3-am654-base-board.dtb \
	k3-am654-gp.dtbo \
	k3-am654-evm-oldi-lcd1evm.dtbo

dtb-$(CONFIG_ARCH_K3_AM6_SOC) += $(shell grep -s incbin $(srctree)/$(src)/*.its | grep dtb | cut -d "\"" -f 2)

$(obj)/%.dtbo: $(src)/%.dtso FORCE
	$(call if_changed_dep,dtc)

$(addprefix $(obj)/,$(dtb-merge-y)): $(addprefix $(obj)/,$(dtb-y))
	@$(srctree)/scripts/dtb-merge $(srctree) $(objtree) $@ $(objtree)/scripts/dtc/fdtoverlay $(src)

always		:= $(dtb-merge-y)
