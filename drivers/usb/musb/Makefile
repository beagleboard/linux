#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

obj-$(CONFIG_USB_MUSB_HDRC) += musb_hdrc.o

musb_hdrc-y := musb_core.o

musb_hdrc-y					+= musb_gadget_ep0.o musb_gadget.o
musb_hdrc-y					+= musb_virthub.o musb_host.o
musb_hdrc-$(CONFIG_DEBUG_FS)			+= musb_debugfs.o
musb_hdrc-$(CONFIG_PROC_FS)			+= musb_procfs.o
# Hardware Glue Layer
obj-$(CONFIG_USB_MUSB_OMAP2PLUS_GLUE)		+= omap2430.o
obj-$(CONFIG_USB_MUSB_AM35X_GLUE)		+= am35x.o
obj-$(CONFIG_USB_MUSB_TI81XX_GLUE)		+= ti81xx.o
obj-$(CONFIG_USB_MUSB_TUSB6010_GLUE)		+= tusb6010.o
obj-$(CONFIG_USB_MUSB_DAVINCI_GLUE)			+= davinci.o
obj-$(CONFIG_USB_MUSB_DA8XX_GLUE)			+= da8xx.o
obj-$(CONFIG_USB_MUSB_BLACKFIN_GLUE)		+= blackfin.o
obj-$(CONFIG_USB_MUSB_UX500_GLUE)			+= ux500.o

# the kconfig must guarantee that only one of the
# possible I/O schemes will be enabled at a time ...
# PIO only, or DMA (several potential schemes).
# though PIO is always there to back up DMA, and for ep0

obj-$(CONFIG_USB_INVENTRA_DMA_HW)		+= musbhsdma.o
obj-$(CONFIG_USB_TI_CPPI_DMA_HW)		+= cppi_dma.o
obj-$(CONFIG_USB_TI_CPPI41_DMA_HW)		+= cppi41dma.o
cppi41dma-y					+= cppi41.o cppi41_dma.o
obj-$(CONFIG_USB_TUSB_OMAP_DMA_HW)		+= tusb6010_omap.o
obj-$(CONFIG_USB_UX500_DMA_HW)			+= ux500_dma.o
