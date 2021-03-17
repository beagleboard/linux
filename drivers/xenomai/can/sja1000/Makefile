ccflags-y += -I$(srctree)/drivers/xenomai/can -I$(srctree)/drivers/xenomai/can/sja1000

obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000) += xeno_can_sja1000.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_PEAK_PCI) += xeno_can_peak_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_PEAK_DNG) += xeno_can_peak_dng.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_PLX_PCI) += xeno_can_plx_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_IXXAT_PCI) += xeno_can_ixxat_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_ADV_PCI) += xeno_can_adv_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_EMS_PCI) += xeno_can_ems_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_ESD_PCI) += xeno_can_esd_pci.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_ISA) += xeno_can_isa.o
obj-$(CONFIG_XENO_DRIVERS_CAN_SJA1000_MEM) += xeno_can_mem.o

xeno_can_sja1000-y := rtcan_sja1000.o
xeno_can_sja1000-$(CONFIG_FS_PROCFS) += rtcan_sja1000_proc.o
xeno_can_peak_pci-y := rtcan_peak_pci.o
xeno_can_peak_dng-y := rtcan_peak_dng.o
xeno_can_plx_pci-y := rtcan_plx_pci.o
xeno_can_ixxat_pci-y := rtcan_ixxat_pci.o
xeno_can_adv_pci-y := rtcan_adv_pci.o
xeno_can_ems_pci-y := rtcan_ems_pci.o
xeno_can_esd_pci-y := rtcan_esd_pci.o
xeno_can_isa-y := rtcan_isa.o
xeno_can_mem-y := rtcan_mem.o
