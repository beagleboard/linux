cmd_arch/arm/boot/dts/am335x-evm.dtb := gcc -E -Wp,-MMD,arch/arm/boot/dts/.am335x-evm.dtb.d.pre.tmp -nostdinc -I./scripts/dtc/include-prefixes -undef -D__DTS__ -x assembler-with-cpp -o arch/arm/boot/dts/.am335x-evm.dtb.dts.tmp arch/arm/boot/dts/am335x-evm.dts ; ./scripts/dtc/dtc -O dtb -o arch/arm/boot/dts/am335x-evm.dtb -b 0 -iarch/arm/boot/dts/ -i./scripts/dtc/include-prefixes -@ -Wno-interrupt_provider -Wno-unit_address_vs_reg -Wno-unit_address_format -Wno-gpios_property -Wno-avoid_unnecessary_addr_size -Wno-alias_paths -Wno-graph_child_address -Wno-simple_bus_reg -Wno-unique_unit_address -Wno-pci_device_reg  -d arch/arm/boot/dts/.am335x-evm.dtb.d.dtc.tmp arch/arm/boot/dts/.am335x-evm.dtb.dts.tmp ; cat arch/arm/boot/dts/.am335x-evm.dtb.d.pre.tmp arch/arm/boot/dts/.am335x-evm.dtb.d.dtc.tmp > arch/arm/boot/dts/.am335x-evm.dtb.d

source_arch/arm/boot/dts/am335x-evm.dtb := arch/arm/boot/dts/am335x-evm.dts

deps_arch/arm/boot/dts/am335x-evm.dtb := \
  arch/arm/boot/dts/am33xx.dtsi \
  scripts/dtc/include-prefixes/dt-bindings/bus/ti-sysc.h \
  scripts/dtc/include-prefixes/dt-bindings/gpio/gpio.h \
  scripts/dtc/include-prefixes/dt-bindings/pinctrl/am33xx.h \
  scripts/dtc/include-prefixes/dt-bindings/pinctrl/omap.h \
  scripts/dtc/include-prefixes/dt-bindings/clock/am3.h \
  arch/arm/boot/dts/am33xx-l4.dtsi \
  arch/arm/boot/dts/am33xx-clocks.dtsi \
  scripts/dtc/include-prefixes/dt-bindings/interrupt-controller/irq.h \
  arch/arm/boot/dts/tps65910.dtsi \

arch/arm/boot/dts/am335x-evm.dtb: $(deps_arch/arm/boot/dts/am335x-evm.dtb)

$(deps_arch/arm/boot/dts/am335x-evm.dtb):
