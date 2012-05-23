/*
 * Texas Instruments TI81XX "usb platform glue layer"
 *
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on the DaVinci "glue layer" code.
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include "cppi41.h"
#include "ti81xx.h"

#include "musb_core.h"
#include "cppi41_dma.h"

#ifdef CONFIG_PM
struct ti81xx_usbss_regs {
	u32	sysconfig;

	u32	irq_en_set;

#ifdef CONFIG_USB_TI_CPPI41_DMA
	u32	irq_dma_th_tx0[4];
	u32	irq_dma_th_rx0[4];
	u32	irq_dma_th_tx1[4];
	u32	irq_dma_th_rx1[4];
	u32	irq_dma_en[2];

	u32	irq_frame_th_tx0[4];
	u32	irq_frame_th_rx0[4];
	u32	irq_frame_th_tx1[4];
	u32	irq_frame_th_rx1[4];
	u32	irq_frame_en[2];
#endif
};

struct ti81xx_usb_regs {
	u32	control;

	u32	irq_en_set[2];

#ifdef CONFIG_USB_TI_CPPI41_DMA
	u32	tx_mode;
	u32	rx_mode;
	u32	grndis_size[15];
	u32	auto_req;
	u32	teardn;
	u32	th_xdma_idle;
#endif
	u32	srp_fix;
	u32	phy_utmi;
	u32	mgc_utmi_loopback;
	u32	mode;
};
#endif

struct ti81xx_glue {
	struct device *dev;
	struct resource *mem_pa;	/* usbss memory resource */
	void *mem_va;			/* ioremapped virtual address */
	struct platform_device *musb[2];/* child musb pdevs */
	u8	irq;			/* usbss irq */
	u8	first;			/* ignore first call of resume */

#ifdef CONFIG_PM
	struct ti81xx_usbss_regs usbss_regs;
	struct ti81xx_usb_regs usb_regs[2];
#endif
};

static u64 musb_dmamask = DMA_BIT_MASK(32);
static void *usbss_virt_base;
static u8 usbss_init_done;
struct musb *gmusb[2];

u8 usbid_sw_ctrl;
#undef USB_TI81XX_DEBUG

#ifdef USB_TI81XX_DEBUG
#define	dprintk(x, ...) printk(x, ## __VA_ARGS__)
#else
#define dprintk(x, ...)
#endif

#ifdef CONFIG_USB_TI_CPPI41_DMA
static irqreturn_t cppi41dma_Interrupt(int irq, void *hci);
static u8 cppi41_init_done;
static void *cppi41_dma_base;
#define CPPI41_ADDR(offs) ((void *)((u32)cppi41_dma_base + (offs - 0x2000)))
#endif

extern void omap_ctrl_writel(u32 val, u16 offset);
extern u32 omap_ctrl_readl(u16 offset);

static inline u32 usbss_read(u32 offset)
{
	if (!usbss_init_done)
		return 0;
	return readl(usbss_virt_base + offset);
}

static inline void usbss_write(u32 offset, u32 data)
{
	if (!usbss_init_done)
		return ;
	writel(data, usbss_virt_base + offset);
}

static void usbotg_ss_init(void)
{
	if (!usbss_init_done) {
		usbss_init_done = 1;

		/* clear any USBSS interrupts */
		usbss_write(USBSS_IRQ_EOI, 0);
		usbss_write(USBSS_IRQ_STATUS, usbss_read(USBSS_IRQ_STATUS));
	}
}
static void usbotg_ss_uninit(void)
{
	if (usbss_init_done) {
		usbss_init_done = 0;
		usbss_virt_base = 0;
	}
}
void set_frame_threshold(struct musb *musb, u8 is_tx, u8 epnum, u8 value, u8 en_intr)
{
	u32     base, reg_val, frame_intr = 0, frame_base = 0;
	u32     offs = epnum/4*4;
	u8      indx = (epnum % 4) * 8;

	if (is_tx)
		base = musb->id ? USBSS_IRQ_FRAME_THRESHOLD_TX1 :
				USBSS_IRQ_FRAME_THRESHOLD_TX0;
	else
		base = musb->id ? USBSS_IRQ_FRAME_THRESHOLD_RX1 :
				USBSS_IRQ_FRAME_THRESHOLD_RX0;

	reg_val = usbss_read(base + offs);
	reg_val &= ~(0xFF << indx);
	reg_val |= (value << indx);
	usbss_write(base + offs, reg_val);

	if (en_intr) {
		frame_base = musb->id ? USBSS_IRQ_FRAME_ENABLE_1 :
			USBSS_IRQ_FRAME_ENABLE_0;
		frame_intr = musb->id ? usbss_read(USBSS_IRQ_FRAME_ENABLE_0) :
			usbss_read(USBSS_IRQ_FRAME_ENABLE_1);
		frame_intr |= is_tx ? (1 << epnum) : (1 << (16 + epnum));
		usbss_write(frame_base, frame_intr);
		dev_dbg(musb->controller, "%s: framebase=%x, frame_intr=%x\n",
			is_tx ? "tx" : "rx", frame_base, frame_intr);
	}
}

void set_dma_threshold(struct musb *musb, u8 is_tx, u8 epnum, u8 value)
{
	u32     base, reg_val;
	u32     offs = epnum/4*4;
	u8      indx = (epnum % 4) * 8;

	if (musb->id == 0)
		base = is_tx ? USBSS_IRQ_DMA_THRESHOLD_TX0 :
				USBSS_IRQ_DMA_THRESHOLD_RX0;
	else
		base = is_tx ? USBSS_IRQ_DMA_THRESHOLD_TX1 :
				USBSS_IRQ_DMA_THRESHOLD_RX1;

	reg_val = usbss_read(base + offs);
	reg_val &= ~(0xFF << indx);
	reg_val |= (value << indx);
	dev_dbg(musb->controller, "base=%x, offs=%x, indx=%d, reg_val = (%x)%x\n",
		base, offs, indx, reg_val, usbss_read(base + offs));
	usbss_write(base + offs, reg_val);
}

/* ti81xx specific read/write functions */
u16 ti81xx_musb_readw(const void __iomem *addr, unsigned offset)
{
	u32 tmp;
	u16 val;

	tmp = readl(addr + (offset & ~3));

	switch (offset & 0x3) {
	case 0:
		val = (tmp & 0xffff);
		break;
	case 1:
		val = (tmp >> 8) & 0xffff;
		break;
	case 2:
	case 3:
	default:
		val = (tmp >> 16) & 0xffff;
		break;
	}
	return val;
}

void ti81xx_musb_writew(void __iomem *addr, unsigned offset, u16 data)
{
	__raw_writew(data, addr + offset);
}

u8 ti81xx_musb_readb(const void __iomem *addr, unsigned offset)
{
	u32 tmp;
	u8 val;

	tmp = readl(addr + (offset & ~3));

	switch (offset & 0x3) {
	case 0:
		val = tmp & 0xff;
		break;
	case 1:
		val = (tmp >> 8);
		break;
	case 2:
		val = (tmp >> 16);
		break;
	case 3:
	default:
		val = (tmp >> 24);
		break;
	}
	return val;
}
void ti81xx_musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	__raw_writeb(data, addr + offset);
}

#ifdef CONFIG_USB_TI_CPPI41_DMA
/*
 * CPPI 4.1 resources used for USB OTG controller module:
 *
 tx/rx completion queues for usb0 */
static u16 tx_comp_q[] = {93, 94, 95, 96, 97,
				98, 99, 100, 101, 102,
				103, 104, 105, 106, 107 };

static u16 rx_comp_q[] = {109, 110, 111, 112, 113,
				114, 115, 116, 117, 118,
				119, 120, 121, 122, 123 };

/* tx/rx completion queues for usb1 */
static u16 tx_comp_q1[] = {125, 126, 127, 128, 129,
				 130, 131, 132, 133, 134,
				 135, 136, 137, 138, 139 };

static u16 rx_comp_q1[] = {141, 142, 143, 144, 145,
				 146, 147, 148, 149, 150,
				 151, 152, 153, 154, 155 };

/* Fair scheduling */
u32 dma_sched_table[] = {
	0x81018000, 0x83038202, 0x85058404, 0x87078606,
	0x89098808, 0x8b0b8a0a, 0x8d0d8c0c, 0x8f0f8e0e,
	0x91119010, 0x93139212, 0x95159414, 0x97179616,
	0x99199818, 0x9b1b9a1a, 0x9d1d9c1c, 0x00009e1e,
};

/* cppi41 dma tx channel info */
static const struct cppi41_tx_ch tx_ch_info[] = {
	[0] = {
		.port_num	= 1,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 32} , {0, 33} }
	},
	[1] = {
		.port_num	= 2,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 34} , {0, 35} }
	},
	[2] = {
		.port_num	= 3,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 36} , {0, 37} }
	},
	[3] = {
		.port_num	= 4,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 38} , {0, 39} }
	},
	[4] = {
		.port_num	= 5,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 40} , {0, 41} }
	},
	[5] = {
		.port_num	= 6,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 42} , {0, 43} }
	},
	[6] = {
		.port_num	= 7,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 44} , {0, 45} }
	},
	[7] = {
		.port_num	= 8,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 46} , {0, 47} }
	},
	[8] = {
		.port_num	= 9,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 48} , {0, 49} }
	},
	[9] = {
		.port_num	= 10,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 50} , {0, 51} }
	},
	[10] = {
		.port_num	= 11,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 52} , {0, 53} }
	},
	[11] = {
		.port_num	= 12,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 54} , {0, 55} }
	},
	[12] = {
		.port_num	= 13,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 56} , {0, 57} }
	},
	[13] = {
		.port_num	= 14,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 58} , {0, 59} }
	},
	[14] = {
		.port_num	= 15,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 60} , {0, 61} }
	},
	[15] = {
		.port_num	= 1,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 62} , {0, 63} }
	},
	[16] = {
		.port_num	= 2,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 64} , {0, 65} }
	},
	[17] = {
		.port_num	= 3,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 66} , {0, 67} }
	},
	[18] = {
		.port_num	= 4,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 68} , {0, 69} }
	},
	[19] = {
		.port_num	= 5,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 70} , {0, 71} }
	},
	[20] = {
		.port_num	= 6,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 72} , {0, 73} }
	},
	[21] = {
		.port_num	= 7,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 74} , {0, 75} }
	},
	[22] = {
		.port_num	= 8,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 76} , {0, 77} }
	},
	[23] = {
		.port_num	= 9,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 78} , {0, 79} }
	},
	[24] = {
		.port_num	= 10,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 80} , {0, 81} }
	},
	[25] = {
		.port_num	= 11,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 82} , {0, 83} }
	},
	[26] = {
		.port_num	= 12,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 84} , {0, 85} }
	},
	[27] = {
		.port_num	= 13,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 86} , {0, 87} }
	},
	[28] = {
		.port_num	= 14,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 88} , {0, 89} }
	},
	[29] = {
		.port_num	= 15,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 90} , {0, 91} }
	}
};

/* Queues 0 to 66 are pre-assigned, others are spare */
static const u32 assigned_queues[] = {	0xffffffff, /* queue 0..31 */
					0xffffffff, /* queue 32..63 */
					0xffffffff, /* queue 64..95 */
					0xffffffff, /* queue 96..127 */
					0x0fffffff  /* queue 128..155 */
					};

int __devinit cppi41_init(u8 id, u8 irq, int num_instances)
{
	struct usb_cppi41_info *cppi_info = &usb_cppi41_info[id];
	u16 numch, blknum, order;
	u32 i;

	/* init cppi info structure  */
	cppi_info->dma_block = 0;
	for (i = 0 ; i < USB_CPPI41_NUM_CH ; i++)
		cppi_info->ep_dma_ch[i] = i + (15 * id);

	cppi_info->q_mgr = 0;
	cppi_info->num_tx_comp_q = 15;
	cppi_info->num_rx_comp_q = 15;
	cppi_info->tx_comp_q = id ? tx_comp_q1 : tx_comp_q;
	cppi_info->rx_comp_q = id ? rx_comp_q1 : rx_comp_q;
	cppi_info->bd_intr_ctrl = 1;
	cppi_info->version = usbss_read(USBSS_REVISION);

	if (cppi41_init_done)
		return 0;

	blknum = cppi_info->dma_block;

	/* Queue manager information */
	cppi41_queue_mgr[0].num_queue = 159;
	cppi41_queue_mgr[0].queue_types = CPPI41_FREE_DESC_BUF_QUEUE |
						CPPI41_UNASSIGNED_QUEUE;
	cppi41_queue_mgr[0].base_fdbq_num = 0;
	cppi41_queue_mgr[0].assigned = assigned_queues;

	/* init DMA block */
	cppi41_dma_block[0].num_tx_ch = 30;
	cppi41_dma_block[0].num_rx_ch = 30;
	cppi41_dma_block[0].tx_ch_info = tx_ch_info;

	/* initilize cppi41 dma & Qmgr address */
	cppi41_dma_base = ioremap(TI81XX_USB_CPPIDMA_BASE,
					TI81XX_USB_CPPIDMA_LEN);

	cppi41_queue_mgr[0].q_mgr_rgn_base = CPPI41_ADDR(QMGR_RGN_OFFS);
	cppi41_queue_mgr[0].desc_mem_rgn_base = CPPI41_ADDR(QMRG_DESCRGN_OFFS);
	cppi41_queue_mgr[0].q_mgmt_rgn_base = CPPI41_ADDR(QMGR_REG_OFFS);
	cppi41_queue_mgr[0].q_stat_rgn_base = CPPI41_ADDR(QMGR_STAT_OFFS);
	cppi41_dma_block[0].global_ctrl_base = CPPI41_ADDR(DMA_GLBCTRL_OFFS);
	cppi41_dma_block[0].ch_ctrl_stat_base = CPPI41_ADDR(DMA_CHCTRL_OFFS);
	cppi41_dma_block[0].sched_ctrl_base = CPPI41_ADDR(DMA_SCHED_OFFS);
	cppi41_dma_block[0].sched_table_base = CPPI41_ADDR(DMA_SCHEDTBL_OFFS);

	/* Initialize for Linking RAM region 0 alone */
	cppi41_queue_mgr_init(cppi_info->q_mgr, 0, 0x3fff);

	numch =  USB_CPPI41_NUM_CH * 2 * num_instances;
	cppi41_dma_block[0].num_max_ch = numch;

	order = get_count_order(numch);

	/* TODO: check two teardown desc per channel (5 or 7 ?)*/
	if (order < 5)
		order = 5;

	cppi41_dma_block_init(blknum, cppi_info->q_mgr, order,
			dma_sched_table, numch);

	/* attach to the IRQ */
	if (request_irq(irq, cppi41dma_Interrupt, 0, "cppi41_dma", 0))
		printk(KERN_INFO "request_irq %d failed!\n", irq);
	else
		printk(KERN_INFO "registerd cppi-dma Intr @ IRQ %d\n", irq);

	cppi41_init_done = 1;

	printk(KERN_INFO "Cppi41 Init Done Qmgr-base(%p) dma-base(%p)\n",
		cppi41_queue_mgr[0].q_mgr_rgn_base,
		cppi41_dma_block[0].global_ctrl_base);

	/* enable all usbss the interrupts */
	usbss_write(USBSS_IRQ_EOI, 0);
	usbss_write(USBSS_IRQ_ENABLE_SET, USBSS_INTR_FLAGS);
	usbss_write(USBSS_IRQ_DMA_ENABLE_0, 0xFFFeFFFe);

	printk(KERN_INFO "Cppi41 Init Done\n");

	return 0;
}

void cppi41_free(void)
{
	u32 numch, blknum, order;
	struct usb_cppi41_info *cppi_info = &usb_cppi41_info[0];

	if (!cppi41_init_done)
		return ;

	numch = cppi41_dma_block[0].num_max_ch;
	order = get_count_order(numch);
	blknum = cppi_info->dma_block;

	cppi41_dma_block_uninit(blknum, cppi_info->q_mgr, order,
			dma_sched_table, numch);
	cppi41_queue_mgr_uninit(cppi_info->q_mgr);

	iounmap(cppi41_dma_base);
	cppi41_dma_base = 0;
	cppi41_init_done = 0;
}

int cppi41_disable_sched_rx(void)
{
	cppi41_dma_sched_tbl_init(0, 0, dma_sched_table, 30);
	return 0;
}

int cppi41_enable_sched_rx(void)
{
	cppi41_dma_sched_tbl_init(0, 0, dma_sched_table, 30);
	return 0;
}
#endif /* CONFIG_USB_TI_CPPI41_DMA */

/*
 * Because we don't set CTRL.UINT, it's "important" to:
 *	- not read/write INTRUSB/INTRUSBE (except during
 *	  initial setup, as a workaround);
 *	- use INTSET/INTCLR instead.
 */

/**
 * ti81xx_musb_enable - enable interrupts
 */
void ti81xx_musb_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask, coremask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & USB_TX_EP_MASK) << USB_INTR_TX_SHIFT) |
	       ((musb->epmask & USB_RX_EP_MASK) << USB_INTR_RX_SHIFT);
	coremask = (0x01ff << USB_INTR_USB_SHIFT);

	coremask &= ~MUSB_INTR_SOF;

	musb_writel(reg_base, USB_EP_INTR_SET_REG, epmask);
	musb_writel(reg_base, USB_CORE_INTR_SET_REG, coremask);
	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	if (is_otg_enabled(musb))
		musb_writel(reg_base, USB_CORE_INTR_SET_REG,
			    USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT);
}

/**
 * ti81xx_musb_disable - disable HDRC and flush interrupts
 */
void ti81xx_musb_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

	musb_writel(reg_base, USB_CORE_INTR_CLEAR_REG, USB_INTR_USB_MASK);
	musb_writel(reg_base, USB_EP_INTR_CLEAR_REG,
			 USB_TX_INTR_MASK | USB_RX_INTR_MASK);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	musb_writel(reg_base, USB_IRQ_EOI, 0);
}

#define	POLL_SECONDS	2

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/* We poll because DaVinci's won't expose several OTG-critical
	* status change events (from the transceiver) otherwise.
	 */
	devctl = musb_readb(mregs, MUSB_DEVCTL);
	dev_dbg(musb->controller, "Poll devctl %02x (%s)\n", devctl,
			otg_state_string(musb->xceiv->state));

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_HM) {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		/*
		 * Wait till VBUS falls below SessionEnd (~0.2 V); the 1.3
		 * RTL seems to mis-handle session "start" otherwise (or in
		 * our case "recover"), in routine "VBUS was valid by the time
		 * VBUSERR got reported during enumeration" cases.
		 */
		if (devctl & MUSB_DEVCTL_VBUS) {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			break;
		}
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, USB_CORE_INTR_SET_REG,
			    MUSB_INTR_VBUSERROR << USB_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		if (!is_peripheral_enabled(musb))
			break;

		/*
		 * There's no ID-changed IRQ, so we have no good way to tell
		 * when to switch to the A-Default state machine (by setting
		 * the DEVCTL.SESSION flag).
		 *
		 * Workaround:  whenever we're in B_IDLE, try setting the
		 * session flag every few seconds.  If it works, ID was
		 * grounded and we're now in the A-Default state machine.
		 *
		 * NOTE: setting the session flag is _supposed_ to trigger
		 * SRP but clearly it doesn't.
		 */
		devctl = musb_readb(mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_HM) {
			musb->xceiv->state = OTG_STATE_A_IDLE;
		} else {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl |
				MUSB_DEVCTL_SESSION);
		}
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

void ti81xx_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	if (!is_otg_enabled(musb))
		return;

	if (timeout == 0)
		timeout = jiffies + msecs_to_jiffies(3);

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || (musb->a_wait_bcon == 0 &&
				musb->xceiv->state == OTG_STATE_A_WAIT_BCON)) {
		dev_dbg(musb->controller, "%s active, deleting timer\n",
			otg_state_string(musb->xceiv->state));
		del_timer(&musb->otg_workaround);
		musb->last_timer = jiffies;
		return;
	}

	if (time_after(musb->last_timer, timeout) &&
					timer_pending(&musb->otg_workaround)) {
		dev_dbg(musb->controller, "Longer idle timer already pending, ignoring...\n");
		return;
	}
	musb->last_timer = timeout;

	dev_dbg(musb->controller, "%s inactive, starting idle timer for %u ms\n",
	    otg_state_string(musb->xceiv->state),
		jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb->otg_workaround, timeout);
}

#ifdef CONFIG_USB_TI_CPPI41_DMA
static irqreturn_t cppi41dma_Interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	u32 intr_status;
	irqreturn_t ret = IRQ_NONE;
	u32 q_cmpl_status_0, q_cmpl_status_1, q_cmpl_status_2;
	u32 usb0_tx_intr, usb0_rx_intr;
	u32 usb1_tx_intr, usb1_rx_intr;
	void *q_mgr_base = cppi41_queue_mgr[0].q_mgr_rgn_base;
	unsigned long flags;

	musb = hci;
	/*
	 * CPPI 4.1 interrupts share the same IRQ and the EOI register but
	 * don't get reflected in the interrupt source/mask registers.
	 */
	/*
	 * Check for the interrupts from Tx/Rx completion queues; they
	 * are level-triggered and will stay asserted until the queues
	 * are emptied.  We're using the queue pending register 0 as a
	 * substitute for the interrupt status register and reading it
	 * directly for speed.
	 */
	intr_status = usbss_read(USBSS_IRQ_STATUS);

	if (intr_status)
		usbss_write(USBSS_IRQ_STATUS, intr_status);
	else
		printk(KERN_DEBUG "spurious usbss intr\n");

	q_cmpl_status_0 = musb_readl(q_mgr_base, CPPI41_QSTATUS_REG2);
	q_cmpl_status_1 = musb_readl(q_mgr_base, CPPI41_QSTATUS_REG3);
	q_cmpl_status_2 = musb_readl(q_mgr_base, CPPI41_QSTATUS_REG4);

	/* USB0 tx/rx completion */
	/* usb0 tx completion interrupt for ep1..15 */
	usb0_tx_intr = (q_cmpl_status_0 >> 29) |
			((q_cmpl_status_1 & 0xFFF) << 3);
	usb0_rx_intr = ((q_cmpl_status_1 & 0x07FFe000) >> 13);

	usb1_tx_intr = (q_cmpl_status_1 >> 29) |
			((q_cmpl_status_2 & 0xFFF) << 3);
	usb1_rx_intr = ((q_cmpl_status_2 & 0x0fffe000) >> 13);

	/* get proper musb handle based usb0/usb1 ctrl-id */

	dev_dbg(musb->controller, "CPPI 4.1 IRQ: Tx %x, Rx %x\n", usb0_tx_intr,
				usb0_rx_intr);
	if (gmusb[0] && (usb0_tx_intr || usb0_rx_intr)) {
		spin_lock_irqsave(&gmusb[0]->lock, flags);
		cppi41_completion(gmusb[0], usb0_rx_intr,
					usb0_tx_intr);
		spin_unlock_irqrestore(&gmusb[0]->lock, flags);
		ret = IRQ_HANDLED;
	}

	dev_dbg(musb->controller, "CPPI 4.1 IRQ: Tx %x, Rx %x\n", usb1_tx_intr,
		usb1_rx_intr);
	if (gmusb[1] && (usb1_rx_intr || usb1_tx_intr)) {
		spin_lock_irqsave(&gmusb[1]->lock, flags);
		cppi41_completion(gmusb[1], usb1_rx_intr,
			usb1_tx_intr);
		spin_unlock_irqrestore(&gmusb[1]->lock, flags);
		ret = IRQ_HANDLED;
	}
	usbss_write(USBSS_IRQ_EOI, 0);

	return ret;
}
#endif

int musb_simulate_babble(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	void __iomem *mbase = musb->mregs;
	u8 reg;

	/* during babble condition musb controller
	 * remove the session
	 */
	reg = musb_readb(mbase, MUSB_DEVCTL);
	reg &= ~MUSB_DEVCTL_SESSION;
	musb_writeb(mbase, MUSB_DEVCTL, reg);
	mdelay(100);

	/* generate s/w babble interrupt */
	musb_writel(reg_base, USB_IRQ_STATUS_RAW_1, MUSB_INTR_BABBLE);
	return 0;
}
EXPORT_SYMBOL(musb_simulate_babble);

void musb_babble_workaround(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);
	while ((musb_readl(reg_base, USB_CTRL_REG) & 0x1))
		cpu_relax();

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(musb->id, 0);
	udelay(100);

	musb_platform_set_mode(musb, MUSB_HOST);
	udelay(100);

	/* enable the usbphy */
	if (data->set_phy_power)
		data->set_phy_power(musb->id, 1);
	mdelay(100);

	/* re-setup the endpoint fifo addresses */
	ep_config_from_table(musb);
	musb_start(musb);
}

static void evm_deferred_musb_restart(struct work_struct *work)
{
	struct musb *musb =
		container_of(work, struct musb, work);

	ERR("deferred musb restart musbid(%d)\n", musb->id);
	musb_babble_workaround(musb);
}

static irqreturn_t ti81xx_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 pend1 = 0, pend2 = 0;
	u32 epintr, usbintr;
	u8  is_babble = 0;

	spin_lock_irqsave(&musb->lock, flags);

	/* Acknowledge and handle non-CPPI interrupts */
	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, USB_EP_INTR_STATUS_REG);
	musb->int_rx = (epintr & USB_RX_INTR_MASK) >> USB_INTR_RX_SHIFT;
	musb->int_tx = (epintr & USB_TX_INTR_MASK) >> USB_INTR_TX_SHIFT;
	if (epintr)
		musb_writel(reg_base, USB_EP_INTR_STATUS_REG, epintr);

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, USB_CORE_INTR_STATUS_REG);
	if (!usbintr && !epintr) {
		dev_dbg(musb->controller, "sprious interrupt\n");
		goto eoi;
	}

	if (usbintr)
		musb_writel(reg_base, USB_CORE_INTR_STATUS_REG, usbintr);
	musb->int_usb =	(usbintr & USB_INTR_USB_MASK) >> USB_INTR_USB_SHIFT;

	dev_dbg(musb->controller, "usbintr (%x) epintr(%x)\n", usbintr, epintr);
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * AM3517's missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if ((usbintr & MUSB_INTR_BABBLE) && is_otg_enabled(musb)
		&& (musb->xceiv->state == OTG_STATE_A_HOST))
		is_babble = 1;
	else if ((usbintr & MUSB_INTR_BABBLE) && !is_otg_enabled(musb)
		&& is_host_enabled(musb))
			is_babble = 1;

	if (is_babble) {
		if (musb->enable_babble_work)
			musb->int_usb |= MUSB_INTR_DISCONNECT;

		ERR("CAUTION: musb%d: Babble Interrupt Occured\n", musb->id);
		ERR("Please issue long reset to make usb functional !!\n");
	}

	if (usbintr & (USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT)) {
		int drvvbus = musb_readl(reg_base, USB_STAT_REG);
		void __iomem *mregs = musb->mregs;
		u8 devctl = musb_readb(mregs, MUSB_DEVCTL);
		int err;

		err = is_host_enabled(musb) && (musb->int_usb &
						MUSB_INTR_VBUSERROR);
		if (err) {
			/*
			 * The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround, but newer RTL from Mentor
			 * seems to allow a better one: "re"-starting sessions
			 * without waiting for VBUS to stop registering in
			 * devctl.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;
			musb->xceiv->state = OTG_STATE_A_WAIT_VFALL;
			mod_timer(&musb->otg_workaround,
						jiffies + POLL_SECONDS * HZ);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (is_host_enabled(musb) && drvvbus) {
			musb->is_active = 1;
			MUSB_HST_MODE(musb);
			musb->xceiv->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			del_timer(&musb->otg_workaround);
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv->default_a = 0;
			musb->xceiv->state = OTG_STATE_B_IDLE;
		}

		/* NOTE: this must complete power-on within 100 ms. */
		dev_dbg(musb->controller, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				otg_state_string(musb->xceiv->state),
				err ? " ERROR" : "",
				devctl);
		ret = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		ret |= musb_interrupt(musb);

 eoi:
	/* EOI needs to be written for the IRQ to be re-asserted. */
	if (ret == IRQ_HANDLED || epintr || usbintr) {
		/* write EOI */
		musb_writel(reg_base, USB_IRQ_EOI, 1);
	}

	/* Poll for ID change */
	if (is_otg_enabled(musb) && musb->xceiv->state == OTG_STATE_B_IDLE)
		mod_timer(&musb->otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	if (ret != IRQ_HANDLED) {
		if (epintr || usbintr)
			/*
			 * We sometimes get unhandled IRQs in the peripheral
			 * mode from EP0 and SOF...
			 */
			dev_dbg(musb->controller, "Unhandled USB IRQ %08x-%08x\n",
					 epintr, usbintr);
		else if (printk_ratelimit())
			/*
			 * We've seen series of spurious interrupts in the
			 * peripheral mode after USB reset and then after some
			 * time a real interrupt storm starting...
			 */
			dev_dbg(musb->controller, "Spurious IRQ, CPPI 4.1 status %08x, %08x\n",
					 pend1, pend2);
	}

	if (is_babble) {
		if (!musb->enable_babble_work) {
			musb_writeb(musb->mregs, MUSB_DEVCTL,
				musb_readb(musb->mregs, MUSB_DEVCTL) |
				MUSB_DEVCTL_SESSION);
		} else {
			ERR("Babble: devtcl(%x)Restarting musb....\n",
				 musb_readb(musb->mregs, MUSB_DEVCTL));
			schedule_work(&musb->work);
		}
	}
	return ret;
}
int ti81xx_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 regval;

	/* TODO: implement this using CONF0 */
	if (musb_mode == MUSB_HOST) {
		regval = musb_readl(reg_base, USB_MODE_REG);

		regval &= ~USBMODE_USBID_HIGH;
		if (usbid_sw_ctrl && cpu_is_ti816x())
			regval |= USBMODE_USBID_MUXSEL;

		musb_writel(reg_base, USB_MODE_REG, regval);
		musb_writel(musb->ctrl_base, USB_PHY_UTMI_REG, 0x02);
		dev_dbg(musb->controller, "host: value of mode reg=%x regval(%x)\n",
			musb_readl(reg_base, USB_MODE_REG), regval);
	} else if (musb_mode == MUSB_PERIPHERAL) {
		/* TODO commmented writing 8 to USB_MODE_REG device
			mode is not working */
		regval = musb_readl(reg_base, USB_MODE_REG);

		regval |= USBMODE_USBID_HIGH;
		if (usbid_sw_ctrl && cpu_is_ti816x())
			regval |= USBMODE_USBID_MUXSEL;

		musb_writel(reg_base, USB_MODE_REG, regval);
		dev_dbg(musb->controller, "device: value of mode reg=%x regval(%x)\n",
			musb_readl(reg_base, USB_MODE_REG), regval);
	} else if (musb_mode == MUSB_OTG) {
		musb_writel(musb->ctrl_base, USB_PHY_UTMI_REG, 0x02);
	} else
		return -EIO;

	return 0;
}

int ti81xx_musb_init(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	u32 rev;
	u8 mode;

	if (musb->id < 2)
		gmusb[musb->id] = musb;

	usb_nop_xceiv_register(musb->id);

	musb->xceiv = otg_get_transceiver(musb->id);
	if (!musb->xceiv)
		return -ENODEV;

	/* mentor is at offset of 0x400 in am3517/ti81xx */
	musb->mregs += USB_MENTOR_CORE_OFFSET;

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev)
		return -ENODEV;

	pr_info("MUSB%d controller's USBSS revision = %08x\n", musb->id, rev);

	if (is_host_enabled(musb))
		setup_timer(&musb->otg_workaround, otg_timer,
					(unsigned long) musb);

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);

	/* wait till reset bit clears */
	while ((musb_readl(reg_base, USB_CTRL_REG) & 0x1))
		cpu_relax();

	/* Start the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(musb->id, 1);

	musb->a_wait_bcon = A_WAIT_BCON_TIMEOUT;
	musb->isr = ti81xx_interrupt;

	if (cpu_is_ti816x())
		usbid_sw_ctrl = 1;

	if (is_otg_enabled(musb)) {
		/* if usb-id contolled through software for ti816x then
		 * configure the usb0 in peripheral mode and usb1 in
		 * host mode
		*/
		if (usbid_sw_ctrl && cpu_is_ti816x())
			mode = musb->id ? MUSB_HOST : MUSB_PERIPHERAL;
		else
			mode = MUSB_OTG;
	} else
		/* set musb controller to host mode */
		mode = is_host_enabled(musb) ? MUSB_HOST : MUSB_PERIPHERAL;

	/* set musb controller to host mode */
	musb_platform_set_mode(musb, mode);

	/* enable babble workaround */
	INIT_WORK(&musb->work, evm_deferred_musb_restart);
	musb->enable_babble_work = 0;

	musb_writel(reg_base, USB_IRQ_EOI, 0);

	return 0;
}

/* TI81xx supports only 32bit read operation */
void ti81xx_musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	void __iomem *fifo = hw_ep->fifo;
	u32		val;
	int		i;

	/* Read for 32bit-aligned destination address */
	if (likely((0x03 & (unsigned long) dst) == 0) && len >= 4) {
		readsl(fifo, dst, len >> 2);
		dst += len & ~0x03;
		len &= 0x03;
	}
	/*
	 * Now read the remaining 1 to 3 byte or complete length if
	 * unaligned address.
	 */
	if (len > 4) {
		for (i = 0; i < (len >> 2); i++) {
			*(u32 *) dst = musb_readl(fifo, 0);
			dst += 4;
		}
		len &= 0x03;
	}
	if (len > 0) {
		val = musb_readl(fifo, 0);
		memcpy(dst, &val, len);
	}
}

int ti81xx_musb_exit(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	if (is_host_enabled(musb))
		del_timer_sync(&musb->otg_workaround);

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(musb->id, 0);

	otg_put_transceiver(musb->xceiv);
	usb_nop_xceiv_unregister(musb->id);

	return 0;
}

static struct musb_platform_ops ti81xx_ops = {
	.fifo_mode	= 4,
	.flags		= MUSB_GLUE_EP_ADDR_FLAT_MAPPING | MUSB_GLUE_DMA_CPPI41,
	.init		= ti81xx_musb_init,
	.exit		= ti81xx_musb_exit,

	.enable		= ti81xx_musb_enable,
	.disable	= ti81xx_musb_disable,

	.try_idle	= ti81xx_musb_try_idle,
	.set_mode	= ti81xx_musb_set_mode,

	.read_fifo      = ti81xx_musb_read_fifo,
	.write_fifo     = musb_write_fifo,

	.dma_controller_create	= cppi41_dma_controller_create,
	.dma_controller_destroy	= cppi41_dma_controller_destroy,
	.simulate_babble_intr	= musb_simulate_babble,
};

static void __devexit ti81xx_delete_musb_pdev(struct ti81xx_glue *glue, u8 id)
{
	platform_device_del(glue->musb[id]);
	platform_device_put(glue->musb[id]);
}

static int __devinit ti81xx_create_musb_pdev(struct ti81xx_glue *glue, u8 id)
{
	struct device *dev = glue->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct musb_hdrc_platform_data  *pdata = dev->platform_data;
	struct omap_musb_board_data *bdata = pdata->board_data;
	struct platform_device	*musb;
	struct resource *res;
	struct resource	resources[2];
	char res_name[10];
	int ret = 0;

	/* get memory resource */
	sprintf(res_name, "musb%d", id);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!res) {
		dev_err(dev, "%s get mem resource failed\n", res_name);
		ret = -ENODEV;
		goto err0;
	}
	res->parent = NULL;
	resources[0] = *res;

	/* get irq resource */
	sprintf(res_name, "musb%d-irq", id);
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res_name);
	if (!res) {
		dev_err(dev, "%s get irq resource failed\n", res_name);
		ret = -ENODEV;
		goto err0;
	}
	res->parent = NULL;
	resources[1] = *res;

	/* allocate the child platform device */
	musb = platform_device_alloc("musb-hdrc", id);
	if (!musb) {
		dev_err(dev, "failed to allocate musb device\n");
		goto err0;
	}

	musb->id			= id;
	musb->dev.parent		= dev;
	musb->dev.dma_mask		= &musb_dmamask;
	musb->dev.coherent_dma_mask	= musb_dmamask;

	glue->musb[id]			= musb;

	pdata->platform_ops		= &ti81xx_ops;

	ret = platform_device_add_resources(musb, resources, 2);
	if (ret) {
		dev_err(dev, "failed to add resources\n");
		goto err1;
	}

	if (id == 0)
		pdata->mode = bdata->mode & USB0PORT_MODEMASK;
	else
		pdata->mode = (bdata->mode & USB1PORT_MODEMASK)
					>> USB1PORT_MODESHIFT;

	dev_info(dev, "musb%d, board_mode=0x%x, plat_mode=0x%x\n",
					id, bdata->mode, pdata->mode);

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(dev, "failed to add platform_data\n");
		goto err1;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(dev, "failed to register musb device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(musb);
err0:
	return ret;
}

static int __init ti81xx_probe(struct platform_device *pdev)
{
	struct ti81xx_glue *glue;
	struct device *dev = &pdev->dev;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int ret = 0, i;
	struct resource *res;

	/* allocate glue */
	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "unable to allocate glue memory\n");
		ret = -ENOMEM;
		goto err0;
	}

	/* get memory resource */
	glue->mem_pa = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!glue->mem_pa) {
		dev_err(&pdev->dev, "failed to get usbss mem resourse\n");
		ret = -ENODEV;
		goto err1;
	}

	/* get memory resource */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "usbss-irq");
	if (!res) {
		dev_err(&pdev->dev, "failed to get usbss irq resourse\n");
		ret = -ENODEV;
		goto err1;
	}
	glue->irq = res->start;

	/* iomap for usbss mem space */
	glue->mem_va =
		ioremap(glue->mem_pa->start, resource_size(glue->mem_pa));
	if (!glue->mem_va) {
		dev_err(&pdev->dev, "usbss ioremap failed\n");
		ret = -ENOMEM;
		goto err1;
	}
	usbss_virt_base = glue->mem_va;

	glue->first = 1;
	glue->dev = &pdev->dev;
	platform_set_drvdata(pdev, glue);

	/* enable clocks */
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync FAILED");
		goto err2;
	}

	/* usb subsystem init */
	usbotg_ss_init();

	/* clear any USBSS interrupts */
	writel(0, glue->mem_va + USBSS_IRQ_EOI);
	writel(readl(glue->mem_va + USBSS_IRQ_STATUS),
					glue->mem_va + USBSS_IRQ_STATUS);

	/* create the child platform device for mulitple instances of musb */
	for (i = 0; i <= data->instances; ++i) {
#ifdef CONFIG_USB_TI_CPPI41_DMA
		/* initialize the cppi41dma init */
		cppi41_init(i, glue->irq, data->instances+1);
#endif
		ret = ti81xx_create_musb_pdev(glue, i);
		if (ret != 0)
			goto err3;
	}

	return 0;

err3:
	pm_runtime_put_sync(&pdev->dev);
err2:
	pm_runtime_disable(&pdev->dev);
	iounmap(glue->mem_va);
err1:
	kfree(glue);
err0:
	return ret;
}

static int __exit ti81xx_remove(struct platform_device *pdev)
{
	struct ti81xx_glue *glue = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/* delete the child platform device for mulitple instances of musb */
	for (i = 0; i <= data->instances; ++i)
		ti81xx_delete_musb_pdev(glue, i);

#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_free();
#endif
	/* iounmap */
	iounmap(glue->mem_va);
	usbotg_ss_uninit();

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(glue);

	return 0;
}

#ifdef CONFIG_PM
static void ti81xx_save_context(struct ti81xx_glue *glue)
{
	struct ti81xx_usbss_regs *usbss = &glue->usbss_regs;
	u8 i, j;

	/* save USBSS register */
	usbss->irq_en_set = usbss_read(USBSS_IRQ_ENABLE_SET);

#ifdef CONFIG_USB_TI_CPPI41_DMA
	for (i = 0 ; i < 4 ; i++) {
		usbss->irq_dma_th_tx0[i] =
			usbss_read(USBSS_IRQ_DMA_THRESHOLD_TX0 + (0x4 * i));
		usbss->irq_dma_th_rx0[i] =
			usbss_read(USBSS_IRQ_DMA_THRESHOLD_RX0 + (0x4 * i));
		usbss->irq_dma_th_tx1[i] =
			usbss_read(USBSS_IRQ_DMA_THRESHOLD_TX1 + (0x4 * i));
		usbss->irq_dma_th_rx1[i] =
			usbss_read(USBSS_IRQ_DMA_THRESHOLD_RX1 + (0x4 * i));

		usbss->irq_frame_th_tx0[i] =
			usbss_read(USBSS_IRQ_FRAME_THRESHOLD_TX0 + (0x4 * i));
		usbss->irq_frame_th_rx0[i] =
			usbss_read(USBSS_IRQ_FRAME_THRESHOLD_RX0 + (0x4 * i));
		usbss->irq_frame_th_tx1[i] =
			usbss_read(USBSS_IRQ_FRAME_THRESHOLD_TX1 + (0x4 * i));
		usbss->irq_frame_th_rx1[i] =
			usbss_read(USBSS_IRQ_FRAME_THRESHOLD_RX1 + (0x4 * i));
	}
	for (i = 0 ; i < 2 ; i++) {
		usbss->irq_dma_en[i] =
			usbss_read(USBSS_IRQ_DMA_ENABLE_0 + (0x4 * i));
		usbss->irq_frame_en[i] =
			usbss_read(USBSS_IRQ_FRAME_ENABLE_0 + (0x4 * i));
	}
#endif
	/* save usbX register */
	for (i = 0 ; i < 2 ; i++) {
		struct ti81xx_usb_regs *usb = &glue->usb_regs[i];
		struct musb *musb = platform_get_drvdata(glue->musb[i]);
		void __iomem *cbase = musb->ctrl_base;

		/* disable the timers */
		if (timer_pending(&musb->otg_workaround) &&
					is_host_enabled(musb)) {
			del_timer_sync(&musb->otg_workaround);
			musb->en_otgw_timer = 1;
		}

		if (timer_pending(&musb->otg_workaround) &&
					is_otg_enabled(musb)) {
			del_timer_sync(&musb->otg_timer);
			musb->en_otg_timer = 1;
		}

		musb_save_context(musb);
		usb->control = musb_readl(cbase, USB_CTRL_REG);

		for (j = 0 ; j < 2 ; j++)
			usb->irq_en_set[j] = musb_readl(cbase,
					USB_IRQ_ENABLE_SET_0 + (0x4 * j));
#ifdef CONFIG_USB_TI_CPPI41_DMA
		usb->tx_mode = musb_readl(cbase, USB_TX_MODE_REG);
		usb->rx_mode = musb_readl(cbase, USB_RX_MODE_REG);

		for (j = 0 ; j < 15 ; j++)
			usb->grndis_size[j] = musb_readl(cbase,
					USB_GENERIC_RNDIS_EP_SIZE_REG(j + 1));

		usb->auto_req = musb_readl(cbase, TI81XX_USB_AUTOREQ_REG);
		usb->teardn = musb_readl(cbase, TI81XX_USB_TEARDOWN_REG);
		usb->th_xdma_idle = musb_readl(cbase, USB_TH_XDMA_IDLE_REG);
#endif
		usb->srp_fix = musb_readl(cbase, USB_SRP_FIX_TIME_REG);
		usb->phy_utmi = musb_readl(cbase, USB_PHY_UTMI_REG);
		usb->mgc_utmi_loopback = musb_readl(cbase, USB_PHY_UTMI_LB_REG);
		usb->mode = musb_readl(cbase, USB_MODE_REG);
	}
#ifdef CONFIG_USB_TI_CPPI41_DMA
	/* save CPPI4.1 DMA register for dma block 0 */
	cppi41_save_context(0);
#endif
}
static void ti81xx_restore_context(struct ti81xx_glue *glue)
{
	struct ti81xx_usbss_regs *usbss = &glue->usbss_regs;
	u8 i, j;

	/* restore USBSS register */
	usbss_write(USBSS_IRQ_ENABLE_SET, usbss->irq_en_set);

#ifdef CONFIG_USB_TI_CPPI41_DMA
	for (i = 0 ; i < 4 ; i++) {
		usbss_write(USBSS_IRQ_DMA_THRESHOLD_TX0 + (0x4 * i),
				usbss->irq_dma_th_tx0[i]);
		usbss_write(USBSS_IRQ_DMA_THRESHOLD_RX0 + (0x4 * i),
				usbss->irq_dma_th_rx0[i]);
		usbss_write(USBSS_IRQ_DMA_THRESHOLD_TX1 + (0x4 * i),
				usbss->irq_dma_th_tx1[i]);
		usbss_write(USBSS_IRQ_DMA_THRESHOLD_RX1 + (0x4 * i),
				usbss->irq_dma_th_rx1[i]);

		usbss_write(USBSS_IRQ_FRAME_THRESHOLD_TX0 + (0x4 * i),
				usbss->irq_frame_th_tx0[i]);
		usbss_write(USBSS_IRQ_FRAME_THRESHOLD_RX0 + (0x4 * i),
				usbss->irq_frame_th_rx0[i]);
		usbss_write(USBSS_IRQ_FRAME_THRESHOLD_TX1 + (0x4 * i),
				usbss->irq_frame_th_tx1[i]);
		usbss_write(USBSS_IRQ_FRAME_THRESHOLD_RX1 + (0x4 * i),
				usbss->irq_frame_th_rx1[i]);
	}
	for (i = 0 ; i < 2 ; i++) {
		usbss_write(USBSS_IRQ_DMA_ENABLE_0 + (0x4 * i),
				usbss->irq_dma_en[i]);
		usbss_write(USBSS_IRQ_FRAME_ENABLE_0 + (0x4 * i),
				usbss->irq_frame_en[i]);
	}
#endif
	/* restore usbX register */
	for (i = 0 ; i < 2 ; i++) {
		struct ti81xx_usb_regs *usb = &glue->usb_regs[i];
		struct musb *musb = platform_get_drvdata(glue->musb[i]);
		void __iomem *cbase = musb->ctrl_base;

		musb_restore_context(musb);
		musb_writel(cbase, USB_CTRL_REG, usb->control);

		for (j = 0 ; j < 2 ; j++)
			musb_writel(cbase, USB_IRQ_ENABLE_SET_0 + (0x4 * j),
					usb->irq_en_set[j]);

#ifdef CONFIG_USB_TI_CPPI41_DMA
		musb_writel(cbase, USB_TX_MODE_REG, usb->tx_mode);
		musb_writel(cbase, USB_RX_MODE_REG, usb->rx_mode);

		for (j = 0 ; j < 15 ; j++)
			musb_writel(cbase, USB_GENERIC_RNDIS_EP_SIZE_REG(j + 1),
					usb->grndis_size[j]);

		musb_writel(cbase, TI81XX_USB_AUTOREQ_REG, usb->auto_req);
		musb_writel(cbase, TI81XX_USB_TEARDOWN_REG, usb->teardn);
		musb_writel(cbase, USB_TH_XDMA_IDLE_REG, usb->th_xdma_idle);
#endif
		musb_writel(cbase, USB_SRP_FIX_TIME_REG, usb->srp_fix);
		musb_writel(cbase, USB_PHY_UTMI_REG, usb->phy_utmi);
		musb_writel(cbase, USB_PHY_UTMI_LB_REG, usb->mgc_utmi_loopback);
		musb_writel(cbase, USB_MODE_REG, usb->mode);

		/* reenable the timers */
		if (musb->en_otgw_timer && is_host_enabled(musb)) {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			musb->en_otgw_timer = 0;
		}
		if (musb->en_otg_timer && is_otg_enabled(musb)) {
			mod_timer(&musb->otg_timer,
					jiffies + POLL_SECONDS * HZ);
			musb->en_otg_timer = 0;
		}
	}
#ifdef CONFIG_USB_TI_CPPI41_DMA
	/* restore CPPI4.1 DMA register for dma block 0 */
	cppi41_restore_context(0, dma_sched_table);
#endif
	/* controller needs delay for successful resume */
	msleep(200);
}
static int ti81xx_runtime_suspend(struct device *dev)
{
	struct ti81xx_glue *glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/* save wrappers and cppi4.1 dma register */
	ti81xx_save_context(glue);

	/* Shutdown the on-chip PHY and its PLL. */
	for (i = 0; i <= data->instances; ++i) {
		if (data->set_phy_power)
			data->set_phy_power(i, 0);
	}

	return 0;
}

static int ti81xx_runtime_resume(struct device *dev)
{
	struct ti81xx_glue *glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/*
	 * ignore first call of resume as all registers are not yet
	 * initialized
	 */
	if (glue->first) {
		glue->first = 0;
		return 0;
	}

	/* Start the on-chip PHY and its PLL. */
	for (i = 0; i <= data->instances; ++i) {
		if (data->set_phy_power)
			data->set_phy_power(i, 1);
	}

	/* restore wrappers and cppi4.1 dma register */
	ti81xx_restore_context(glue);

	return 0;
}

static const struct dev_pm_ops ti81xx_pm_ops = {
	.runtime_suspend = ti81xx_runtime_suspend,
	.runtime_resume	= ti81xx_runtime_resume,
};

#define DEV_PM_OPS	(&ti81xx_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver ti81xx_musb_driver = {
	.remove         = __exit_p(ti81xx_remove),
	.driver         = {
		.name   = "musb-ti81xx",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_DESCRIPTION("AM35x MUSB Glue Layer");
MODULE_LICENSE("GPL v2");

static int __init ti81xx_glue_init(void)
{
	return platform_driver_probe(&ti81xx_musb_driver, ti81xx_probe);
}
subsys_initcall(ti81xx_glue_init);

static void __exit ti81xx_glue_exit(void)
{
#ifdef CONFIG_USB_TI_CPPI41_DMA
	/* free the usbss irq */
	free_irq(TI81XX_IRQ_USBSS, 0);
#endif

	/* disable the interrupts */
	usbss_write(USBSS_IRQ_EOI, 0);
	usbss_write(USBSS_IRQ_ENABLE_SET, 0);
	usbss_write(USBSS_IRQ_DMA_ENABLE_0, 0);

	/* unregister platform driver */
	platform_driver_unregister(&ti81xx_musb_driver);
}
module_exit(ti81xx_glue_exit);
