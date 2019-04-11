DTC_FLAGS += -@ -Wno-reg_format -Wno-avoid_default_addr_size
DTC_FLAGS_MKIMAGE = -I dts -O dtb -p 500 -Wno-unit_address_vs_reg \
	-i arch/arm/boot/dts/ti

-include $(obj)/.itb-deps.d

ifneq ($(DTC),)
  DTC_PATH=$(dir $(DTC))
else
  DTC_PATH=$(objtree)/scripts/dtc
endif

quiet_cmd_mkfit = MKIMAGE -f $@
cmd_mkfit = PATH="$(DTC_PATH):$(PATH)" $(MKIMAGE) -D "$(DTC_FLAGS_MKIMAGE)" -f $< $@

define gen_itb_deps
	@if [ -d $(obj) ] ; then \
		echo >> $(obj)/.itb-deps.d ; \
		echo "$(src)/$(patsubst %.its,%.itb,$(notdir $(1))): $(srctree)/$(src)/$(notdir $(1)) \\" >> $(obj)/.itb-deps.d ; \
		cat $(1) | grep incbin | cut -d "\"" -f 2 | awk '{ printf " $(src)/%s \\\n", $$1 }' | sed '$$s/ \\$$//' >> $(obj)/.itb-deps.d ; \
	fi

endef

$(obj)/.itb-deps.d:
	@if [ -d  $(obj) ] ; then \
		echo "$(src)/.itb-deps.d: $(src)/Makefile $(wildcard $(srctree)/$(src)/*.its)" > $(obj)/.itb-deps.d ; \
		echo >> $(obj)/.itb-deps.d ; \
	fi
	$(foreach f,$(wildcard $(srctree)/$(src)/*.its),$(call gen_itb_deps,$f))

$(obj)/%.dtb: $(src)/../%.dts FORCE
	$(call if_changed_dep,dtc)

$(obj)/%.dtbo: $(src)/%.dtso FORCE
	$(call if_changed_dep,dtc)

$(obj)/%.itb: $(src)/%.its FORCE
	$(call cmd,mkfit)

ifeq ($(BUILD_ITBS),y)
dtb-$(CONFIG_SOC_DRA7XX) += \
	am57xx-evm.itb \
	am57xx-idk.itb \
	dra71-evm.itb \
	dra72-evm.itb \
	dra74-evm.itb \
	dra76-evm.itb
else
dtb-$(CONFIG_SOC_DRA7XX) += $(shell grep incbin $(srctree)/$(src)/*.its | grep dtb | cut -d "\"" -f 2)
endif

clean-files	:= *.dtb *.dtbo *.itb

always		:= $(dtb-y)
