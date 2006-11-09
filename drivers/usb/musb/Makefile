#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

musb_hdrc-objs := plat_uds.o

obj-$(CONFIG_USB_MUSB_HDRC)	+= musb_hdrc.o

ifeq ($(CONFIG_ARCH_DAVINCI),y)
	musb_hdrc-objs	+= davinci.o
endif

ifeq ($(CONFIG_USB_TUSB6010),y)
	musb_hdrc-objs	+= tusb6010.o
endif

ifeq ($(CONFIG_ARCH_OMAP243X),y)
	musb_hdrc-objs	+= omap2430.o
endif

ifeq ($(CONFIG_USB_GADGET_MUSB_HDRC),y)
	musb_hdrc-objs		+= g_ep0.o musb_gadget.o
endif

ifeq ($(CONFIG_USB_MUSB_HDRC_HCD),y)
	musb_hdrc-objs		+= virthub.o musb_host.o
endif

# the kconfig must guarantee that only one of the
# possible I/O schemes will be enabled at a time ...
# PIO (INVENTRA_FIFO), or DMA (several potential schemes).
# though PIO is always there to back up DMA, and for ep0

ifneq ($(CONFIG_USB_INVENTRA_FIFO),y)

  ifeq ($(CONFIG_USB_INVENTRA_DMA),y)
    musb_hdrc-objs		+= musbhsdma.o

  else
    ifeq ($(CONFIG_USB_TI_CPPI_DMA),y)
      musb_hdrc-objs		+= cppi_dma.o

    else
      ifeq ($(CONFIG_USB_TUSB_OMAP_DMA),y)
        musb_hdrc-objs		+= tusb6010_omap.o

      endif
    endif
  endif
endif


################################################################################

# FIXME remove all these extra "-DMUSB_* things, stick to CONFIG_*

ifeq ($(CONFIG_USB_INVENTRA_MUSB_HAS_AHB_ID),y)
	EXTRA_CFLAGS += -DMUSB_AHB_ID
endif

# Debugging

MUSB_DEBUG:=$(CONFIG_USB_INVENTRA_HCD_LOGGING)

ifeq ("$(strip $(MUSB_DEBUG))","")
    ifdef CONFIG_USB_DEBUG
	MUSB_DEBUG:=1
    else
	MUSB_DEBUG:=0
    endif
endif

ifneq ($(MUSB_DEBUG),0)
    EXTRA_CFLAGS += -DDEBUG

    ifeq ($(CONFIG_PROC_FS),y)
	musb_hdrc-objs		+= musb_procfs.o
    endif

endif

EXTRA_CFLAGS += -DMUSB_DEBUG=$(MUSB_DEBUG)
