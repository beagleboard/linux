DTC_FLAGS += -@ -Wno-reg_format -Wno-avoid_default_addr_size
DTC_FLAGS_MKIMAGE = -I dts -O dtb -p 500 -Wno-unit_address_vs_reg \
	-i arch/arm/boot/dts/ti

quiet_cmd_mkfit = MKIMAGE -f $@
cmd_mkfit = $(MKIMAGE) -D "$(DTC_FLAGS_MKIMAGE)" -f $< $@ ; \
	echo "$(notdir $@): $(srctree)/$(src)/$(patsubst %.itb,%.its,$(notdir $@)) \\" > $(depfile) ; \
	cat $< | grep incbin | cut -d "\"" -f 2 | awk '{ printf " $(src)/%s \\\n", $$1 }' | sed '$$s/ \\$$//' >> $(depfile)

%.dtb: ../%.dts FORCE
	$(call if_changed_dep,dtc)

%.dtbo: %.dtso FORCE
	$(call if_changed_dep,dtc)

%.itb: %.its FORCE
	$(call if_changed_dep,mkfit)

clean-files	:= *.dtb *.dtbo *.itb

always		:= $(dtb-y)
