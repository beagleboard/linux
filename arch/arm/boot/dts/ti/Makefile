DTC_FLAGS += -@ -Wno-reg_format -Wno-avoid_default_addr_size
DTC_FLAGS_MKIMAGE = -I dts -O dtb -p 500 -Wno-unit_address_vs_reg \
	-i arch/arm/boot/dts/ti

ifneq ($(DTC),)
  DTC_PATH=$(dir $(DTC))
else
  DTC_PATH=$(srctree)/scripts/dtc
endif

quiet_cmd_mkfit = MKIMAGE -f $@
cmd_mkfit = PATH="$(DTC_PATH):$(PATH)" $(MKIMAGE) -D "$(DTC_FLAGS_MKIMAGE)" -f $< $@ ; \
	echo "$(notdir $@): $(srctree)/$(src)/$(patsubst %.itb,%.its,$(notdir $@)) \\" > $(depfile) ; \
	cat $< | grep incbin | cut -d "\"" -f 2 | awk '{ printf " $(src)/%s \\\n", $$1 }' | sed '$$s/ \\$$//' >> $(depfile)

%.dtb: ../%.dts FORCE
	$(call if_changed_dep,dtc)

%.dtbo: %.dtso FORCE
	$(call if_changed_dep,dtc)

%.itb: %.its FORCE
	$(call if_changed_dep,mkfit)

dtb-$(CONFIG_SOC_DRA7XX) += \
	am57xx-beagle-x15.dtb \
	am57xx-beagle-x15-revc.dtb \
	am57xx-evm.dtbo \
	am57xx-evm.itb

$(src)/am57xx-evm.itb: $(obj)/am57xx-evm.dtbo $(obj)/am57xx-beagle-x15.dtb \
	$(obj)/am57xx-beagle-x15-revc.dtb $(obj)/../../zImage

clean-files	:= *.dtb *.dtbo *.itb

always		:= $(dtb-y)
