/*
 * Texas Instruments AM35x "glue layer"
 *
 * Copyright (c) 2010, by Texas Instruments
 *
 * Based on the DA8xx "glue layer" code.
 * Copyright (c) 2008-2009, MontaVista Software, Inc. <source@mvista.com>
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
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <plat/usb.h>

#include "musb_core.h"
#include "cppi41.h"
#include "cppi41_dma.h"

/*
 * AM35x specific definitions
 */
/* USB 2.0 OTG module registers */
#define USB_REVISION_REG	0x00
#define USB_CTRL_REG		0x04
#define USB_STAT_REG		0x08
#define USB_EMULATION_REG	0x0c
/* 0x10 Reserved */
#define USB_SRP_FIX_TIME_REG	0x18
#define EP_INTR_SRC_REG		0x20
#define EP_INTR_SRC_SET_REG	0x24
#define EP_INTR_SRC_CLEAR_REG	0x28
#define EP_INTR_MASK_REG	0x2c
#define EP_INTR_MASK_SET_REG	0x30
#define EP_INTR_MASK_CLEAR_REG	0x34
#define EP_INTR_SRC_MASKED_REG	0x38
#define CORE_INTR_SRC_REG	0x40
#define CORE_INTR_SRC_SET_REG	0x44
#define CORE_INTR_SRC_CLEAR_REG	0x48
#define CORE_INTR_MASK_REG	0x4c
#define CORE_INTR_MASK_SET_REG	0x50
#define CORE_INTR_MASK_CLEAR_REG 0x54
#define CORE_INTR_SRC_MASKED_REG 0x58
/* 0x5c Reserved */
#define USB_END_OF_INTR_REG	0x60

/* Control register bits */
#define AM35X_SOFT_RESET_MASK	1

/* USB interrupt register bits */
#define AM35X_INTR_USB_SHIFT	16
#define AM35X_INTR_USB_MASK	(0x1ff << AM35X_INTR_USB_SHIFT)
#define AM35X_INTR_DRVVBUS	0x100
#define AM35X_INTR_RX_SHIFT	16
#define AM35X_INTR_TX_SHIFT	0
#define AM35X_TX_EP_MASK	0xffff		/* EP0 + 15 Tx EPs */
#define AM35X_RX_EP_MASK	0xfffe		/* 15 Rx EPs */
#define AM35X_TX_INTR_MASK	(AM35X_TX_EP_MASK << AM35X_INTR_TX_SHIFT)
#define AM35X_RX_INTR_MASK	(AM35X_RX_EP_MASK << AM35X_INTR_RX_SHIFT)

/* CPPI 4.1 queue manager registers */
#define QMGR_PEND0_REG		0x4090
#define QMGR_PEND1_REG		0x4094
#define QMGR_PEND2_REG		0x4098

#define USB_MENTOR_CORE_OFFSET	0x400

#ifdef CONFIG_USB_TI_CPPI41_DMA
#define CPPI41_QMGR_REG0SIZE	0x3fff

/*
 * CPPI 4.1 resources used for USB OTG controller module:
 *
 * USB   DMA  DMA  QMgr  Tx     Src
 *       Tx   Rx         QNum   Port
 * ---------------------------------
 * EP0   0    0    0     16,17  1
 * ---------------------------------
 * EP1   1    1    0     18,19  2
 * ---------------------------------
 * EP2   2    2    0     20,21  3
 * ---------------------------------
 * EP3   3    3    0     22,23  4
 * ---------------------------------
 */

static u16 tx_comp_q[] = {63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
				63, 63};
static u16 rx_comp_q[] = {65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65,
				65, 65};

/* Fair scheduling */
u32 dma_sched_table[] = {
	0x81018000, 0x83038202, 0x85058404, 0x87078606,
	0x89098808, 0x8b0b8a0a, 0x8d0d8c0c, 0x00008e0e
};

/* DMA block configuration */
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
	}
};

/* Queues 0 to 66 are pre-assigned, others are spare */
static const u32 assigned_queues[] = { 0xffffffff, 0xffffffff, 0x7 };

int __devinit cppi41_init(struct musb *musb)
{
	struct usb_cppi41_info *cppi_info = &usb_cppi41_info[musb->id];
	u16 numch, blknum, order, i;

	/* init cppi info structure  */
	cppi_info->dma_block = 0;
	for (i = 0 ; i < USB_CPPI41_NUM_CH ; i++)
		cppi_info->ep_dma_ch[i] = i;

	cppi_info->q_mgr = 0;
	cppi_info->num_tx_comp_q = 15;
	cppi_info->num_rx_comp_q = 15;
	cppi_info->tx_comp_q = tx_comp_q;
	cppi_info->rx_comp_q = rx_comp_q;
	cppi_info->bd_intr_ctrl = 0; /* am35x dont support bd interrupt */

	blknum = cppi_info->dma_block;

	/* Queue manager information */
	cppi41_queue_mgr[0].num_queue = 96;
	cppi41_queue_mgr[0].queue_types = CPPI41_FREE_DESC_BUF_QUEUE |
						CPPI41_UNASSIGNED_QUEUE;
	cppi41_queue_mgr[0].base_fdbq_num = 0;
	cppi41_queue_mgr[0].assigned = assigned_queues;

	/* init mappings */
	cppi41_queue_mgr[0].q_mgr_rgn_base = musb->ctrl_base + 0x4000;
	cppi41_queue_mgr[0].desc_mem_rgn_base = musb->ctrl_base + 0x5000;
	cppi41_queue_mgr[0].q_mgmt_rgn_base = musb->ctrl_base + 0x6000;
	cppi41_queue_mgr[0].q_stat_rgn_base = musb->ctrl_base + 0x6800;

	/* init DMA block */
	cppi41_dma_block[0].num_tx_ch = 15;
	cppi41_dma_block[0].num_rx_ch = 15;
	cppi41_dma_block[0].tx_ch_info = tx_ch_info;

	cppi41_dma_block[0].global_ctrl_base = musb->ctrl_base + 0x1000;
	cppi41_dma_block[0].ch_ctrl_stat_base = musb->ctrl_base + 0x1800;
	cppi41_dma_block[0].sched_ctrl_base = musb->ctrl_base + 0x2000;
	cppi41_dma_block[0].sched_table_base = musb->ctrl_base + 0x2800;

	/* Initialize for Linking RAM region 0 alone */
	cppi41_queue_mgr_init(cppi_info->q_mgr, 0, CPPI41_QMGR_REG0SIZE);

	numch =  USB_CPPI41_NUM_CH * 2;
	order = get_count_order(numch);

	/* TODO: check two teardown desc per channel (5 or 7 ?)*/
	if (order < 5)
		order = 5;

	cppi41_dma_block_init(blknum, cppi_info->q_mgr, order,
			dma_sched_table, numch);
	return 0;
}
void cppi41_free(void)
{
	u32 numch, blknum, order;
	struct usb_cppi41_info *cppi_info = &usb_cppi41_info[0];

	numch =  USB_CPPI41_NUM_CH * 2;
	order = get_count_order(numch);
	blknum = cppi_info->dma_block;

	cppi41_dma_block_uninit(blknum, cppi_info->q_mgr, order,
			dma_sched_table, numch);
	cppi41_queue_mgr_uninit(cppi_info->q_mgr);
}
#endif /* CONFIG_USB_TI_CPPI41_DMA */

struct am35x_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct clk		*phy_clk;
	struct clk		*clk;
};
#define glue_to_musb(g)		platform_get_drvdata(g->musb)

/*
 * am35x_musb_enable - enable interrupts
 */
static void am35x_musb_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & AM35X_TX_EP_MASK) << AM35X_INTR_TX_SHIFT) |
	       ((musb->epmask & AM35X_RX_EP_MASK) << AM35X_INTR_RX_SHIFT);

	musb_writel(reg_base, EP_INTR_MASK_SET_REG, epmask);
	musb_writel(reg_base, CORE_INTR_MASK_SET_REG, AM35X_INTR_USB_MASK);

	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	if (is_otg_enabled(musb))
		musb_writel(reg_base, CORE_INTR_SRC_SET_REG,
			    AM35X_INTR_DRVVBUS << AM35X_INTR_USB_SHIFT);
}

/*
 * am35x_musb_disable - disable HDRC and flush interrupts
 */
static void am35x_musb_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

	musb_writel(reg_base, CORE_INTR_MASK_CLEAR_REG, AM35X_INTR_USB_MASK);
	musb_writel(reg_base, EP_INTR_MASK_CLEAR_REG,
			 AM35X_TX_INTR_MASK | AM35X_RX_INTR_MASK);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
}

#define portstate(stmt)		stmt

static void am35x_musb_set_vbus(struct musb *musb, int is_on)
{
	WARN_ON(is_on && is_peripheral_active(musb));
}

#define	POLL_SECONDS	2

static struct timer_list otg_workaround;

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/*
	 * We poll because AM35x's won't expose several OTG-critical
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
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, CORE_INTR_SRC_SET_REG,
			    MUSB_INTR_VBUSERROR << AM35X_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		if (!is_peripheral_enabled(musb))
			break;

		devctl = musb_readb(mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE)
			mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);
		else
			musb->xceiv->state = OTG_STATE_A_IDLE;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

static void am35x_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	static unsigned long last_timer;

	if (!is_otg_enabled(musb))
		return;

	if (timeout == 0)
		timeout = jiffies + msecs_to_jiffies(3);

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || (musb->a_wait_bcon == 0 &&
				musb->xceiv->state == OTG_STATE_A_WAIT_BCON)) {
		dev_dbg(musb->controller, "%s active, deleting timer\n",
			otg_state_string(musb->xceiv->state));
		del_timer(&otg_workaround);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout) && timer_pending(&otg_workaround)) {
		dev_dbg(musb->controller, "Longer idle timer already pending, ignoring...\n");
		return;
	}
	last_timer = timeout;

	dev_dbg(musb->controller, "%s inactive, starting idle timer for %u ms\n",
		otg_state_string(musb->xceiv->state),
		jiffies_to_msecs(timeout - jiffies));
	mod_timer(&otg_workaround, timeout);
}

static irqreturn_t am35x_musb_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 pend1 = 0, pend2 = 0, tx, rx;
	u32 epintr, usbintr;

	spin_lock_irqsave(&musb->lock, flags);

	/*
	 * CPPI 4.1 interrupts share the same IRQ and the EOI register but
	 * don't get reflected in the interrupt source/mask registers.
	 */
	if (is_cppi41_enabled(musb)) {
		/*
		 * Check for the interrupts from Tx/Rx completion queues; they
		 * are level-triggered and will stay asserted until the queues
		 * are emptied.  We're using the queue pending register 0 as a
		 * substitute for the interrupt status register and reading it
		 * directly for speed.
		 */
		pend1 = musb_readl(reg_base, QMGR_PEND1_REG);
		pend2 = musb_readl(reg_base, QMGR_PEND2_REG);

		/* AM3517 uses 63,64,65 and 66 queues as completion queue */
		if ((pend1 & (1 << 31)) || (pend2 & (7 << 0))) {
			tx = (pend1 >> 31)  | ((pend2 & 1) ? (1 << 1) : 0);
			rx = (pend2 >> 1) & 0x3;

			dev_dbg(musb->controller, "CPPI 4.1 IRQ: Tx %x, Rx %x\n", tx, rx);
			cppi41_completion(musb, rx, tx);
			ret = IRQ_HANDLED;
		}
	}
	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, EP_INTR_SRC_MASKED_REG);

	if (epintr) {
		musb_writel(reg_base, EP_INTR_SRC_CLEAR_REG, epintr);

		musb->int_rx =
			(epintr & AM35X_RX_INTR_MASK) >> AM35X_INTR_RX_SHIFT;
		musb->int_tx =
			(epintr & AM35X_TX_INTR_MASK) >> AM35X_INTR_TX_SHIFT;
	}

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, CORE_INTR_SRC_MASKED_REG);
	if (!usbintr && !epintr)
		goto eoi;

	if (usbintr) {
		musb_writel(reg_base, CORE_INTR_SRC_CLEAR_REG, usbintr);

		musb->int_usb =
			(usbintr & AM35X_INTR_USB_MASK) >> AM35X_INTR_USB_SHIFT;
	}
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * AM35x's missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if (usbintr & (AM35X_INTR_DRVVBUS << AM35X_INTR_USB_SHIFT)) {
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
			mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (is_host_enabled(musb) && drvvbus) {
			MUSB_HST_MODE(musb);
			musb->xceiv->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			portstate(musb->port1_status |= USB_PORT_STAT_POWER);
			del_timer(&otg_workaround);
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv->default_a = 0;
			musb->xceiv->state = OTG_STATE_B_IDLE;
			portstate(musb->port1_status &= ~USB_PORT_STAT_POWER);
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
		/* clear level interrupt */
		if (data->clear_irq)
			data->clear_irq();
		/* write EOI */
		musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
	}

	/* Poll for ID change */
	if (is_otg_enabled(musb) && musb->xceiv->state == OTG_STATE_B_IDLE)
		mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;
}

static int am35x_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int     retval = 0;

	if (data->set_mode)
		data->set_mode(musb_mode);
	else
		retval = -EIO;

	return retval;
}

static int am35x_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	void __iomem *reg_base = musb->ctrl_base;
	u32 rev;

	musb->mregs += USB_MENTOR_CORE_OFFSET;

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev)
		return -ENODEV;

	usb_nop_xceiv_register(musb->id);
	musb->xceiv = otg_get_transceiver(musb->id);
	if (!musb->xceiv)
		return -ENODEV;

	if (is_host_enabled(musb))
		setup_timer(&otg_workaround, otg_timer, (unsigned long) musb);

	/* Reset the musb */
	if (data->reset)
		data->reset();

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, AM35X_SOFT_RESET_MASK);

	/* Start the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0, 1);

	msleep(5);

#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_init(musb);
#endif

	musb->isr = am35x_musb_interrupt;

	/* clear level interrupt */
	if (data->clear_irq)
		data->clear_irq();

	return 0;
}

static int am35x_musb_exit(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	if (is_host_enabled(musb))
		del_timer_sync(&otg_workaround);

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0, 0);

	otg_put_transceiver(musb->xceiv);
	usb_nop_xceiv_unregister(musb->id);

	return 0;
}

/* AM35x supports only 32bit read operation */
static void am35x_musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
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

static const struct musb_platform_ops am35x_ops = {
	.fifo_mode	= 4,
	.flags		= MUSB_GLUE_EP_ADDR_FLAT_MAPPING | MUSB_GLUE_DMA_CPPI41,
	.init		= am35x_musb_init,
	.exit		= am35x_musb_exit,

	.enable		= am35x_musb_enable,
	.disable	= am35x_musb_disable,

	.set_mode	= am35x_musb_set_mode,
	.try_idle	= am35x_musb_try_idle,

	.set_vbus	= am35x_musb_set_vbus,

	.read_fifo  = am35x_musb_read_fifo,
	.write_fifo = musb_write_fifo,

	.dma_controller_create = cppi41_dma_controller_create,
	.dma_controller_destroy = cppi41_dma_controller_destroy,
};

static u64 am35x_dmamask = DMA_BIT_MASK(32);

static int __init am35x_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct am35x_glue		*glue;

	struct clk			*phy_clk;
	struct clk			*clk;

	int				ret = -ENOMEM;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", pdev->id);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err1;
	}

	dev_set_name(&pdev->dev, "musb-am35x");
	phy_clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(phy_clk)) {
		dev_err(&pdev->dev, "failed to get PHY clock\n");
		ret = PTR_ERR(phy_clk);
		goto err2;
	}

	clk = clk_get(&pdev->dev, "ick");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err3;
	}

	ret = clk_enable(phy_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable PHY clock\n");
		goto err4;
	}

	ret = clk_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err5;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &am35x_dmamask;
	musb->dev.coherent_dma_mask	= am35x_dmamask;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->phy_clk			= phy_clk;
	glue->clk			= clk;

	pdata->platform_ops		= &am35x_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err6;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err6;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err6;
	}

	return 0;

err6:
	clk_disable(clk);

err5:
	clk_disable(phy_clk);

err4:
	clk_put(clk);

err3:
	clk_put(phy_clk);

err2:
	platform_device_put(musb);

err1:
	kfree(glue);

err0:
	return ret;
}

static int __exit am35x_remove(struct platform_device *pdev)
{
	struct am35x_glue	*glue = platform_get_drvdata(pdev);

	platform_device_del(glue->musb);
	/*platform_device_put(glue->musb);*/
	clk_disable(glue->clk);
	clk_disable(glue->phy_clk);
	clk_put(glue->clk);
	clk_put(glue->phy_clk);
	kfree(glue);

	return 0;
}

#ifdef CONFIG_PM
static int am35x_suspend(struct device *dev)
{
	struct am35x_glue	*glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0, 0);

	clk_disable(glue->phy_clk);
	clk_disable(glue->clk);

	return 0;
}

static int am35x_resume(struct device *dev)
{
	struct am35x_glue	*glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int			ret;

	/* Start the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0, 1);

	ret = clk_enable(glue->phy_clk);
	if (ret) {
		dev_err(dev, "failed to enable PHY clock\n");
		return ret;
	}

	ret = clk_enable(glue->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	return 0;
}

static struct dev_pm_ops am35x_pm_ops = {
	.suspend	= am35x_suspend,
	.resume		= am35x_resume,
};

#define DEV_PM_OPS	&am35x_pm_ops
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver am35x_driver = {
	.remove		= __exit_p(am35x_remove),
	.driver		= {
		.name	= "musb-am35x",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_DESCRIPTION("AM35x MUSB Glue Layer");
MODULE_AUTHOR("Ajay Kumar Gupta <ajay.gupta@ti.com>");
MODULE_LICENSE("GPL v2");

static int __init am35x_init(void)
{
	return platform_driver_probe(&am35x_driver, am35x_probe);
}
subsys_initcall(am35x_init);

static void __exit am35x_exit(void)
{
#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_free();
#endif
	platform_driver_unregister(&am35x_driver);
}
module_exit(am35x_exit);
