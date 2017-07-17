obj-$(CONFIG_PCIE_DW) += pcie-designware.o
obj-$(CONFIG_PCIE_DW_HOST) += pcie-designware-host.o
obj-$(CONFIG_PCIE_DW_EP) += pcie-designware-ep.o
obj-$(CONFIG_PCIE_DW_PLAT) += pcie-designware-plat.o
ifneq ($(filter y,$(CONFIG_PCI_DRA7XX_HOST) $(CONFIG_PCI_DRA7XX_EP)),)
        obj-$(CONFIG_PCI_DRA7XX) += pci-dra7xx.o
endif
obj-$(CONFIG_PCI_EXYNOS) += pci-exynos.o
obj-$(CONFIG_PCI_IMX6) += pci-imx6.o
obj-$(CONFIG_PCIE_SPEAR13XX) += pcie-spear13xx.o
ifneq ($(filter y,$(CONFIG_PCI_KEYSTONE_HOST) $(CONFIG_PCI_KEYSTONE_EP)),)
	obj-$(CONFIG_PCI_KEYSTONE) += pci-keystone-dw.o pci-keystone.o
endif
obj-$(CONFIG_PCI_LAYERSCAPE) += pci-layerscape.o
obj-$(CONFIG_PCI_HISI) += pcie-hisi.o
obj-$(CONFIG_PCIE_QCOM) += pcie-qcom.o
obj-$(CONFIG_PCIE_ARMADA_8K) += pcie-armada8k.o
obj-$(CONFIG_PCIE_ARTPEC6) += pcie-artpec6.o
