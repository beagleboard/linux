DTC_FLAGS += -@

dtb-y += am57xx-evm.dtbo

ifneq ($(DTC),)
  DTC_PATH=$(dir $(DTC))
else
  DTC_PATH=$(objtree)/scripts/dtc
endif

$(obj)/%.dtb: $(src)/../%.dts FORCE
	$(call if_changed_dep,dtc,dtb)

$(obj)/%.dtbo: $(src)/%.dtso FORCE
	$(call if_changed_dep,dtc,dtb)

clean-files	:= *.dtb *.dtbo

always		:= $(dtb-y)
