#
# Makefile for nvmem drivers.
#

obj-$(CONFIG_NVMEM)		+= nvmem_core.o
nvmem_core-y			:= core.o
nvmem_core-y			+= nvmem-mmio.o

# Devices
obj-$(CONFIG_QCOM_QFPROM)	+= nvmem_qfprom.o
nvmem_qfprom-y			:= qfprom.o
obj-$(CONFIG_NVMEM_SUNXI_SID)	+= nvmem-sunxi-sid.o
nvmem-sunxi-sid-y		:= sunxi-sid.o
