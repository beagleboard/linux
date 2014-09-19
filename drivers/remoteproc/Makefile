#
# Generic framework for controlling remote processors
#

obj-$(CONFIG_REMOTEPROC)		+= remoteproc.o
remoteproc-y				:= remoteproc_core.o
remoteproc-y				+= remoteproc_debugfs.o
remoteproc-y				+= remoteproc_virtio.o
remoteproc-y				+= remoteproc_elf_loader.o
obj-$(CONFIG_OMAP_REMOTEPROC)		+= omap_remoteproc.o
obj-$(CONFIG_STE_MODEM_RPROC)	 	+= ste_modem_rproc.o
obj-$(CONFIG_PRU_RPROC)		 	+= pru_rproc.o
obj-$(CONFIG_PRU_RPROC_BEAGLELOGIC)	+= beaglelogic.o
obj-$(CONFIG_PRU_RPROC_PRU_SPEAK)	+= pru_speak.o
