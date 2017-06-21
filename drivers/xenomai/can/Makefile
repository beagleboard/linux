
ccflags-y += -Idrivers/xenomai/can

obj-$(CONFIG_XENO_DRIVERS_CAN) += xeno_can.o mscan/ sja1000/
obj-$(CONFIG_XENO_DRIVERS_CAN_FLEXCAN) += xeno_can_flexcan.o
obj-$(CONFIG_XENO_DRIVERS_CAN_VIRT) += xeno_can_virt.o

xeno_can-y := rtcan_dev.o rtcan_socket.o rtcan_module.o rtcan_raw.o rtcan_raw_dev.o rtcan_raw_filter.o
xeno_can_virt-y := rtcan_virt.o
xeno_can_flexcan-y := rtcan_flexcan.o
