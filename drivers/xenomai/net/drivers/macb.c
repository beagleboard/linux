/*
 * Cadence MACB/GEM Ethernet Controller driver
 *
 * Copyright (C) 2004-2006 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * RTnet porting by Cristiano Mantovani & Stefano Banzi (Marposs SpA).
 * Copyright (C) 2014 Gilles Chanteperdrix <gch@xenomai.org>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/macb.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/pinctrl/consumer.h>

#include <rtdev.h>
#include <rtnet.h>
#include <rtnet_port.h>
#include <rtskb.h>

#include "rt_macb.h"

#define MACB_RX_BUFFER_SIZE	128
#define RX_BUFFER_MULTIPLE	64  /* bytes */
#define RX_RING_SIZE		512 /* must be power of 2 */
#define RX_RING_BYTES		(sizeof(struct macb_dma_desc) * RX_RING_SIZE)

#define TX_RING_SIZE		128 /* must be power of 2 */
#define TX_RING_BYTES		(sizeof(struct macb_dma_desc) * TX_RING_SIZE)

/* level of occupied TX descriptors under which we wake up TX process */
#define MACB_TX_WAKEUP_THRESH	(3 * TX_RING_SIZE / 4)

#define MACB_RX_INT_FLAGS	(MACB_BIT(RCOMP) | MACB_BIT(RXUBR)	\
				 | MACB_BIT(ISR_ROVR))
#define MACB_TX_ERR_FLAGS	(MACB_BIT(ISR_TUND)			\
					| MACB_BIT(ISR_RLE)		\
					| MACB_BIT(TXERR))
#define MACB_TX_INT_FLAGS	(MACB_TX_ERR_FLAGS | MACB_BIT(TCOMP))

/*
 * Graceful stop timeouts in us. We should allow up to
 * 1 frame time (10 Mbits/s, full-duplex, ignoring collisions)
 */
#define MACB_HALT_TIMEOUT	1230

/* Ring buffer accessors */
static unsigned int macb_tx_ring_wrap(unsigned int index)
{
	return index & (TX_RING_SIZE - 1);
}

static struct macb_dma_desc *macb_tx_desc(struct macb *bp, unsigned int index)
{
	return &bp->tx_ring[macb_tx_ring_wrap(index)];
}

static struct macb_tx_skb *macb_tx_skb(struct macb *bp, unsigned int index)
{
	return &bp->tx_skb[macb_tx_ring_wrap(index)];
}

static unsigned int macb_rx_ring_wrap(unsigned int index)
{
	return index & (RX_RING_SIZE - 1);
}

static struct macb_dma_desc *macb_rx_desc(struct macb *bp, unsigned int index)
{
	return &bp->rx_ring[macb_rx_ring_wrap(index)];
}

static void *macb_rx_buffer(struct macb *bp, unsigned int index)
{
	return bp->rx_buffers + bp->rx_buffer_size * macb_rx_ring_wrap(index);
}

void rtmacb_set_hwaddr(struct macb *bp)
{
	u32 bottom;
	u16 top;

	bottom = cpu_to_le32(*((u32 *)bp->dev->dev_addr));
	macb_or_gem_writel(bp, SA1B, bottom);
	top = cpu_to_le16(*((u16 *)(bp->dev->dev_addr + 4)));
	macb_or_gem_writel(bp, SA1T, top);

	/* Clear unused address register sets */
	macb_or_gem_writel(bp, SA2B, 0);
	macb_or_gem_writel(bp, SA2T, 0);
	macb_or_gem_writel(bp, SA3B, 0);
	macb_or_gem_writel(bp, SA3T, 0);
	macb_or_gem_writel(bp, SA4B, 0);
	macb_or_gem_writel(bp, SA4T, 0);
}
EXPORT_SYMBOL_GPL(rtmacb_set_hwaddr);

void rtmacb_get_hwaddr(struct macb *bp)
{
	struct macb_platform_data *pdata;
	u32 bottom;
	u16 top;
	u8 addr[6];
	int i;

	pdata = dev_get_platdata(&bp->pdev->dev);

	/* Check all 4 address register for vaild address */
	for (i = 0; i < 4; i++) {
		bottom = macb_or_gem_readl(bp, SA1B + i * 8);
		top = macb_or_gem_readl(bp, SA1T + i * 8);

		if (pdata && pdata->rev_eth_addr) {
			addr[5] = bottom & 0xff;
			addr[4] = (bottom >> 8) & 0xff;
			addr[3] = (bottom >> 16) & 0xff;
			addr[2] = (bottom >> 24) & 0xff;
			addr[1] = top & 0xff;
			addr[0] = (top & 0xff00) >> 8;
		} else {
			addr[0] = bottom & 0xff;
			addr[1] = (bottom >> 8) & 0xff;
			addr[2] = (bottom >> 16) & 0xff;
			addr[3] = (bottom >> 24) & 0xff;
			addr[4] = top & 0xff;
			addr[5] = (top >> 8) & 0xff;
		}

		if (is_valid_ether_addr(addr)) {
			memcpy(bp->dev->dev_addr, addr, sizeof(addr));
			return;
		}
	}
}
EXPORT_SYMBOL_GPL(rtmacb_get_hwaddr);

static int macb_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct macb *bp = bus->priv;
	int value;

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_READ)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)));

	/* wait for end of transfer */
	while (!MACB_BFEXT(IDLE, macb_readl(bp, NSR)))
		cpu_relax();

	value = MACB_BFEXT(DATA, macb_readl(bp, MAN));

	return value;
}

static int macb_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct macb *bp = bus->priv;

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_WRITE)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)
			      | MACB_BF(DATA, value)));

	/* wait for end of transfer */
	while (!MACB_BFEXT(IDLE, macb_readl(bp, NSR)))
		cpu_relax();

	return 0;
}

/**
 * macb_set_tx_clk() - Set a clock to a new frequency
 * @clk		Pointer to the clock to change
 * @rate	New frequency in Hz
 * @dev		Pointer to the struct rtnet_device
 */
static void macb_set_tx_clk(struct clk *clk, int speed, struct rtnet_device *dev)
{
	long ferr, rate, rate_rounded;

	switch (speed) {
	case SPEED_10:
		rate = 2500000;
		break;
	case SPEED_100:
		rate = 25000000;
		break;
	case SPEED_1000:
		rate = 125000000;
		break;
	default:
		return;
	}

	rate_rounded = clk_round_rate(clk, rate);
	if (rate_rounded < 0)
		return;

	/* RGMII allows 50 ppm frequency error. Test and warn if this limit
	 * is not satisfied.
	 */
	ferr = abs(rate_rounded - rate);
	ferr = DIV_ROUND_UP(ferr, rate / 100000);
	if (ferr > 5)
		rtdev_warn(dev, "unable to generate target frequency: %ld Hz\n",
				rate);

	if (clk_set_rate(clk, rate_rounded))
		rtdev_err(dev, "adjusting tx_clk failed.\n");
}

struct macb_dummy_netdev_priv {
	struct rtnet_device *rtdev;
};

static void macb_handle_link_change(struct net_device *nrt_dev)
{
	struct macb_dummy_netdev_priv *p = netdev_priv(nrt_dev);
	struct rtnet_device *dev = p->rtdev;
	struct macb *bp = rtnetdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;
	unsigned long flags;

	int status_change = 0;

	rtdm_lock_get_irqsave(&bp->lock, flags);

	if (phydev->link) {
		if ((bp->speed != phydev->speed) ||
		    (bp->duplex != phydev->duplex)) {
			u32 reg;

			reg = macb_readl(bp, NCFGR);
			reg &= ~(MACB_BIT(SPD) | MACB_BIT(FD));
			if (macb_is_gem(bp))
				reg &= ~GEM_BIT(GBE);

			if (phydev->duplex)
				reg |= MACB_BIT(FD);
			if (phydev->speed == SPEED_100)
				reg |= MACB_BIT(SPD);
			if (phydev->speed == SPEED_1000)
				reg |= GEM_BIT(GBE);

			macb_or_gem_writel(bp, NCFGR, reg);

			bp->speed = phydev->speed;
			bp->duplex = phydev->duplex;
			status_change = 1;
		}
	}

	if (phydev->link != bp->link) {
		if (!phydev->link) {
			bp->speed = 0;
			bp->duplex = -1;
		}
		bp->link = phydev->link;

		status_change = 1;
	}

	rtdm_lock_put_irqrestore(&bp->lock, flags);

	if (!IS_ERR(bp->tx_clk))
		macb_set_tx_clk(bp->tx_clk, phydev->speed, dev);

	if (status_change) {
		if (phydev->link) {
			rtnetif_carrier_on(dev);
			rtdev_info(dev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			rtnetif_carrier_off(dev);
			rtdev_info(dev, "link down\n");
		}
	}
}

/* based on au1000_eth. c*/
static int macb_mii_probe(struct rtnet_device *dev)
{
	struct macb *bp = rtnetdev_priv(dev);
	struct macb_dummy_netdev_priv *p;
	struct macb_platform_data *pdata;
	struct phy_device *phydev;
	struct net_device *dummy;
	int phy_irq;
	int ret;

	phydev = phy_find_first(bp->mii_bus);
	if (!phydev) {
		rtdev_err(dev, "no PHY found\n");
		return -ENXIO;
	}

	pdata = dev_get_platdata(&bp->pdev->dev);
	if (pdata && gpio_is_valid(pdata->phy_irq_pin)) {
		ret = devm_gpio_request(&bp->pdev->dev, pdata->phy_irq_pin, "phy int");
		if (!ret) {
			phy_irq = gpio_to_irq(pdata->phy_irq_pin);
			phydev->irq = (phy_irq < 0) ? PHY_POLL : phy_irq;
		}
	}

	dummy = alloc_etherdev(sizeof(*p));
	p = netdev_priv(dummy);
	p->rtdev = dev;
	bp->phy_phony_net_device = dummy;

	/* attach the mac to the phy */
	ret = phy_connect_direct(dummy, phydev, &macb_handle_link_change,
				 bp->phy_interface);
	if (ret) {
		rtdev_err(dev, "Could not attach to PHY\n");
		return ret;
	}

	/* mask with MAC supported features */
	if (macb_is_gem(bp))
		phydev->supported &= PHY_GBIT_FEATURES;
	else
		phydev->supported &= PHY_BASIC_FEATURES;

	phydev->advertising = phydev->supported;

	bp->link = 0;
	bp->speed = 0;
	bp->duplex = -1;
	bp->phy_dev = phydev;

	return 0;
}

int rtmacb_mii_init(struct macb *bp)
{
	struct macb_platform_data *pdata;
	struct device_node *np;
	int err = -ENXIO, i;

	/* Enable management port */
	macb_writel(bp, NCR, MACB_BIT(MPE));

	bp->mii_bus = mdiobus_alloc();
	if (bp->mii_bus == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	bp->mii_bus->name = "MACB_mii_bus";
	bp->mii_bus->read = &macb_mdio_read;
	bp->mii_bus->write = &macb_mdio_write;
	snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		bp->pdev->name, bp->pdev->id);
	bp->mii_bus->priv = bp;
	bp->mii_bus->parent = &bp->pdev->dev;
	pdata = dev_get_platdata(&bp->pdev->dev);

	bp->mii_bus->irq = kmalloc(sizeof(int)*PHY_MAX_ADDR, GFP_KERNEL);
	if (!bp->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	np = bp->pdev->dev.of_node;
	if (np) {
		/* try dt phy registration */
		err = of_mdiobus_register(bp->mii_bus, np);

		/* fallback to standard phy registration if no phy were
		   found during dt phy registration */
		if (!err && !phy_find_first(bp->mii_bus)) {
			for (i = 0; i < PHY_MAX_ADDR; i++) {
				struct phy_device *phydev;

				phydev = mdiobus_scan(bp->mii_bus, i);
				if (IS_ERR(phydev)) {
					err = PTR_ERR(phydev);
					break;
				}
			}

			if (err)
				goto err_out_unregister_bus;
		}
	} else {
		for (i = 0; i < PHY_MAX_ADDR; i++)
			bp->mii_bus->irq[i] = PHY_POLL;

		if (pdata)
			bp->mii_bus->phy_mask = pdata->phy_mask;

		err = mdiobus_register(bp->mii_bus);
	}

	if (err)
		goto err_out_free_mdio_irq;

	err = macb_mii_probe(bp->dev);
	if (err)
		goto err_out_unregister_bus;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(bp->mii_bus);
err_out_free_mdio_irq:
	kfree(bp->mii_bus->irq);
err_out_free_mdiobus:
	mdiobus_free(bp->mii_bus);
err_out:
	return err;
}
EXPORT_SYMBOL_GPL(rtmacb_mii_init);

static void macb_update_stats(struct macb *bp)
{
	u32 __iomem *reg = bp->regs + MACB_PFR;
	u32 *p = &bp->hw_stats.macb.rx_pause_frames;
	u32 *end = &bp->hw_stats.macb.tx_pause_frames + 1;

	WARN_ON((unsigned long)(end - p - 1) != (MACB_TPF - MACB_PFR) / 4);

	for(; p < end; p++, reg++)
		*p += __raw_readl(reg);
}

static int macb_halt_tx(struct macb *bp)
{
	unsigned long	halt_time, timeout;
	u32		status;

	macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(THALT));

	timeout = jiffies + usecs_to_jiffies(MACB_HALT_TIMEOUT);
	do {
		halt_time = jiffies;
		status = macb_readl(bp, TSR);
		if (!(status & MACB_BIT(TGO)))
			return 0;

		usleep_range(10, 250);
	} while (time_before(halt_time, timeout));

	return -ETIMEDOUT;
}

static void macb_tx_error_task(struct work_struct *work)
{
	struct macb	*bp = container_of(work, struct macb, tx_error_task);
	struct macb_tx_skb	*tx_skb;
	struct rtskb		*skb;
	unsigned int		tail;

	rtdev_vdbg(bp->dev, "macb_tx_error_task: t = %u, h = %u\n",
		    bp->tx_tail, bp->tx_head);

	/* Make sure nobody is trying to queue up new packets */
	rtnetif_stop_queue(bp->dev);

	/*
	 * Stop transmission now
	 * (in case we have just queued new packets)
	 */
	if (macb_halt_tx(bp))
		/* Just complain for now, reinitializing TX path can be good */
		rtdev_err(bp->dev, "BUG: halt tx timed out\n");

	/* No need for the lock here as nobody will interrupt us anymore */

	/*
	 * Treat frames in TX queue including the ones that caused the error.
	 * Free transmit buffers in upper layer.
	 */
	for (tail = bp->tx_tail; tail != bp->tx_head; tail++) {
		struct macb_dma_desc	*desc;
		u32			ctrl;

		desc = macb_tx_desc(bp, tail);
		ctrl = desc->ctrl;
		tx_skb = macb_tx_skb(bp, tail);
		skb = tx_skb->skb;

		if (ctrl & MACB_BIT(TX_USED)) {
			rtdev_vdbg(bp->dev, "txerr skb %u (data %p) TX complete\n",
				    macb_tx_ring_wrap(tail), skb->data);
			bp->stats.tx_packets++;
			bp->stats.tx_bytes += skb->len;
		} else {
			/*
			 * "Buffers exhausted mid-frame" errors may only happen
			 * if the driver is buggy, so complain loudly about those.
			 * Statistics are updated by hardware.
			 */
			if (ctrl & MACB_BIT(TX_BUF_EXHAUSTED))
				rtdev_err(bp->dev,
					   "BUG: TX buffers exhausted mid-frame\n");

			desc->ctrl = ctrl | MACB_BIT(TX_USED);
		}

		dma_unmap_single(&bp->pdev->dev, tx_skb->mapping, skb->len,
				 DMA_TO_DEVICE);
		tx_skb->skb = NULL;
		dev_kfree_rtskb(skb);
	}

	/* Make descriptor updates visible to hardware */
	wmb();

	/* Reinitialize the TX desc queue */
	macb_writel(bp, TBQP, bp->tx_ring_dma);
	/* Make TX ring reflect state of hardware */
	bp->tx_head = bp->tx_tail = 0;

	/* Now we are ready to start transmission again */
	rtnetif_wake_queue(bp->dev);

	/* Housework before enabling TX IRQ */
	macb_writel(bp, TSR, macb_readl(bp, TSR));
	macb_writel(bp, IER, MACB_TX_INT_FLAGS);
}

static void macb_tx_interrupt(struct macb *bp)
{
	unsigned int tail;
	unsigned int head;
	u32 status;

	status = macb_readl(bp, TSR);
	macb_writel(bp, TSR, status);

	if (bp->caps & MACB_CAPS_ISR_CLEAR_ON_WRITE)
		macb_writel(bp, ISR, MACB_BIT(TCOMP));

	rtdev_vdbg(bp->dev, "macb_tx_interrupt status = 0x%03lx\n",
		(unsigned long)status);

	head = bp->tx_head;
	for (tail = bp->tx_tail; tail != head; tail++) {
		struct macb_tx_skb	*tx_skb;
		struct rtskb		*skb;
		struct macb_dma_desc	*desc;
		u32			ctrl;

		desc = macb_tx_desc(bp, tail);

		/* Make hw descriptor updates visible to CPU */
		rmb();

		ctrl = desc->ctrl;

		if (!(ctrl & MACB_BIT(TX_USED)))
			break;

		tx_skb = macb_tx_skb(bp, tail);
		skb = tx_skb->skb;

		rtdev_vdbg(bp->dev, "skb %u (data %p) TX complete\n",
			macb_tx_ring_wrap(tail), skb->data);
		dma_unmap_single(&bp->pdev->dev, tx_skb->mapping, skb->len,
				 DMA_TO_DEVICE);
		bp->stats.tx_packets++;
		bp->stats.tx_bytes += skb->len;
		tx_skb->skb = NULL;
		dev_kfree_rtskb(skb);
	}

	bp->tx_tail = tail;
	if (rtnetif_queue_stopped(bp->dev)
			&& CIRC_CNT(bp->tx_head, bp->tx_tail,
				    TX_RING_SIZE) <= MACB_TX_WAKEUP_THRESH)
		rtnetif_wake_queue(bp->dev);
}

static void gem_rx_refill(struct macb *bp)
{
	unsigned int		entry;
	struct rtskb		*skb;
	dma_addr_t		paddr;

	while (CIRC_SPACE(bp->rx_prepared_head, bp->rx_tail, RX_RING_SIZE) > 0) {
		entry = macb_rx_ring_wrap(bp->rx_prepared_head);

		/* Make hw descriptor updates visible to CPU */
		rmb();

		bp->rx_prepared_head++;

		if (bp->rx_skbuff[entry] == NULL) {
			/* allocate rtskb for this free entry in ring */
			skb = rtnetdev_alloc_rtskb(bp->dev, bp->rx_buffer_size);
			if (unlikely(skb == NULL)) {
				rtdev_err(bp->dev,
					   "Unable to allocate sk_buff\n");
				break;
			}

			/* now fill corresponding descriptor entry */
			paddr = dma_map_single(&bp->pdev->dev, skb->data,
					       bp->rx_buffer_size, DMA_FROM_DEVICE);
			if (dma_mapping_error(&bp->pdev->dev, paddr)) {
				dev_kfree_rtskb(skb);
				break;
			}

			bp->rx_skbuff[entry] = skb;

			if (entry == RX_RING_SIZE - 1)
				paddr |= MACB_BIT(RX_WRAP);
			bp->rx_ring[entry].addr = paddr;
			bp->rx_ring[entry].ctrl = 0;

			/* properly align Ethernet header */
			rtskb_reserve(skb, NET_IP_ALIGN);
		}
	}

	/* Make descriptor updates visible to hardware */
	wmb();

	rtdev_vdbg(bp->dev, "rx ring: prepared head %d, tail %d\n",
		   bp->rx_prepared_head, bp->rx_tail);
}

/* Mark DMA descriptors from begin up to and not including end as unused */
static void discard_partial_frame(struct macb *bp, unsigned int begin,
				  unsigned int end)
{
	unsigned int frag;

	for (frag = begin; frag != end; frag++) {
		struct macb_dma_desc *desc = macb_rx_desc(bp, frag);
		desc->addr &= ~MACB_BIT(RX_USED);
	}

	/* Make descriptor updates visible to hardware */
	wmb();

	/*
	 * When this happens, the hardware stats registers for
	 * whatever caused this is updated, so we don't have to record
	 * anything.
	 */
}

static int gem_rx(struct macb *bp, int budget, nanosecs_abs_t *time_stamp)
{
	unsigned int		len;
	unsigned int		entry;
	struct rtskb		*skb;
	struct macb_dma_desc	*desc;
	int			count = 0;

	while (count < budget) {
		u32 addr, ctrl;

		entry = macb_rx_ring_wrap(bp->rx_tail);
		desc = &bp->rx_ring[entry];

		/* Make hw descriptor updates visible to CPU */
		rmb();

		addr = desc->addr;
		ctrl = desc->ctrl;

		if (!(addr & MACB_BIT(RX_USED)))
			break;

		bp->rx_tail++;
		count++;

		if (!(ctrl & MACB_BIT(RX_SOF) && ctrl & MACB_BIT(RX_EOF))) {
			rtdev_err(bp->dev,
				   "not whole frame pointed by descriptor\n");
			bp->stats.rx_dropped++;
			break;
		}
		skb = bp->rx_skbuff[entry];
		if (unlikely(!skb)) {
			rtdev_err(bp->dev,
				   "inconsistent Rx descriptor chain\n");
			bp->stats.rx_dropped++;
			break;
		}
		skb->time_stamp = *time_stamp;
		/* now everything is ready for receiving packet */
		bp->rx_skbuff[entry] = NULL;
		len = MACB_BFEXT(RX_FRMLEN, ctrl);

		rtdev_vdbg(bp->dev, "gem_rx %u (len %u)\n", entry, len);

		rtskb_put(skb, len);
		addr = MACB_BF(RX_WADDR, MACB_BFEXT(RX_WADDR, addr));
		dma_unmap_single(&bp->pdev->dev, addr,
				 bp->rx_buffer_size, DMA_FROM_DEVICE);

		skb->protocol = rt_eth_type_trans(skb, bp->dev);
		rtskb_checksum_none_assert(skb);

		bp->stats.rx_packets++;
		bp->stats.rx_bytes += skb->len;

#if defined(DEBUG) && defined(VERBOSE_DEBUG)
		rtdev_vdbg(bp->dev, "received skb of length %u, csum: %08x\n",
			    skb->len, skb->csum);
		print_hex_dump(KERN_DEBUG, " mac: ", DUMP_PREFIX_ADDRESS, 16, 1,
			       skb->mac_header, 16, true);
		print_hex_dump(KERN_DEBUG, "data: ", DUMP_PREFIX_ADDRESS, 16, 1,
			       skb->data, 32, true);
#endif

		rtnetif_rx(skb);
	}

	gem_rx_refill(bp);

	return count;
}

static int macb_rx_frame(struct macb *bp, unsigned int first_frag,
			unsigned int last_frag, nanosecs_abs_t *time_stamp)
{
	unsigned int len;
	unsigned int frag;
	unsigned int offset;
	struct rtskb *skb;
	struct macb_dma_desc *desc;

	desc = macb_rx_desc(bp, last_frag);
	len = MACB_BFEXT(RX_FRMLEN, desc->ctrl);

	rtdev_vdbg(bp->dev, "macb_rx_frame frags %u - %u (len %u)\n",
		macb_rx_ring_wrap(first_frag),
		macb_rx_ring_wrap(last_frag), len);

	/*
	 * The ethernet header starts NET_IP_ALIGN bytes into the
	 * first buffer. Since the header is 14 bytes, this makes the
	 * payload word-aligned.
	 *
	 * Instead of calling skb_reserve(NET_IP_ALIGN), we just copy
	 * the two padding bytes into the skb so that we avoid hitting
	 * the slowpath in memcpy(), and pull them off afterwards.
	 */
	skb = rtnetdev_alloc_rtskb(bp->dev, len + NET_IP_ALIGN);
	if (!skb) {
		rtdev_notice(bp->dev, "Low memory, packet dropped.\n");
		bp->stats.rx_dropped++;
		for (frag = first_frag; ; frag++) {
			desc = macb_rx_desc(bp, frag);
			desc->addr &= ~MACB_BIT(RX_USED);
			if (frag == last_frag)
				break;
		}

		/* Make descriptor updates visible to hardware */
		wmb();

		return 1;
	}

	offset = 0;
	len += NET_IP_ALIGN;
	skb->time_stamp = *time_stamp;
	rtskb_checksum_none_assert(skb);
	rtskb_put(skb, len);

	for (frag = first_frag; ; frag++) {
		unsigned int frag_len = bp->rx_buffer_size;

		if (offset + frag_len > len) {
			BUG_ON(frag != last_frag);
			frag_len = len - offset;
		}
		memcpy(skb->data + offset, macb_rx_buffer(bp, frag), frag_len);
		offset += bp->rx_buffer_size;
		desc = macb_rx_desc(bp, frag);
		desc->addr &= ~MACB_BIT(RX_USED);

		if (frag == last_frag)
			break;
	}

	/* Make descriptor updates visible to hardware */
	wmb();

	__rtskb_pull(skb, NET_IP_ALIGN);
	skb->protocol = rt_eth_type_trans(skb, bp->dev);

	bp->stats.rx_packets++;
	bp->stats.rx_bytes += skb->len;
	rtdev_vdbg(bp->dev, "received skb of length %u, csum: %08x\n",
		   skb->len, skb->csum);
	rtnetif_rx(skb);

	return 0;
}

static int macb_rx(struct macb *bp, int budget, nanosecs_abs_t *time_stamp)
{
	int received = 0;
	unsigned int tail;
	int first_frag = -1;

	for (tail = bp->rx_tail; budget > 0; tail++) {
		struct macb_dma_desc *desc = macb_rx_desc(bp, tail);
		u32 addr, ctrl;

		/* Make hw descriptor updates visible to CPU */
		rmb();

		addr = desc->addr;
		ctrl = desc->ctrl;

		if (!(addr & MACB_BIT(RX_USED)))
			break;

		if (ctrl & MACB_BIT(RX_SOF)) {
			if (first_frag != -1)
				discard_partial_frame(bp, first_frag, tail);
			first_frag = tail;
		}

		if (ctrl & MACB_BIT(RX_EOF)) {
			int dropped;
			BUG_ON(first_frag == -1);

			dropped = macb_rx_frame(bp, first_frag, tail, time_stamp);
			first_frag = -1;
			if (!dropped) {
				received++;
				budget--;
			}
		}
	}

	if (first_frag != -1)
		bp->rx_tail = first_frag;
	else
		bp->rx_tail = tail;

	return received;
}

static int macb_interrupt(rtdm_irq_t *irq_handle)
{
	void *dev_id = rtdm_irq_get_arg(irq_handle, void);
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtnet_device *dev = dev_id;
	struct macb *bp = rtnetdev_priv(dev);
	unsigned received = 0;
	u32 status;

	status = macb_readl(bp, ISR);

	if (unlikely(!status))
		return RTDM_IRQ_NONE;

	rtdm_lock_get(&bp->lock);

	while (status) {
		/* close possible race with dev_close */
		if (unlikely(!rtnetif_running(dev))) {
			macb_writel(bp, IDR, -1);
			break;
		}

		rtdev_vdbg(bp->dev, "isr = 0x%08lx\n", (unsigned long)status);

		if (status & MACB_RX_INT_FLAGS)
			received += bp->macbgem_ops.mog_rx(bp, 100 - received,
							&time_stamp);

		if (unlikely(status & (MACB_TX_ERR_FLAGS))) {
			macb_writel(bp, IDR, MACB_TX_INT_FLAGS);
			rtdm_schedule_nrt_work(&bp->tx_error_task);

			if (bp->caps & MACB_CAPS_ISR_CLEAR_ON_WRITE)
				macb_writel(bp, ISR, MACB_TX_ERR_FLAGS);

			break;
		}

		if (status & MACB_BIT(TCOMP))
			macb_tx_interrupt(bp);

		/*
		 * Link change detection isn't possible with RMII, so we'll
		 * add that if/when we get our hands on a full-blown MII PHY.
		 */

		if (status & MACB_BIT(ISR_ROVR)) {
			/* We missed at least one packet */
			if (macb_is_gem(bp))
				bp->hw_stats.gem.rx_overruns++;
			else
				bp->hw_stats.macb.rx_overruns++;

			if (bp->caps & MACB_CAPS_ISR_CLEAR_ON_WRITE)
				macb_writel(bp, ISR, MACB_BIT(ISR_ROVR));
		}

		if (status & MACB_BIT(HRESP)) {
			/*
			 * TODO: Reset the hardware, and maybe move the
			 * rtdev_err to a lower-priority context as well
			 * (work queue?)
			 */
			rtdev_err(dev, "DMA bus error: HRESP not OK\n");

			if (bp->caps & MACB_CAPS_ISR_CLEAR_ON_WRITE)
				macb_writel(bp, ISR, MACB_BIT(HRESP));
		}

		status = macb_readl(bp, ISR);
	}

	rtdm_lock_put(&bp->lock);

	if (received)
		rt_mark_stack_mgr(dev);

	return RTDM_IRQ_HANDLED;
}

static int macb_start_xmit(struct rtskb *skb, struct rtnet_device *dev)
{
	struct macb *bp = rtnetdev_priv(dev);
	dma_addr_t mapping;
	unsigned int len, entry;
	struct macb_dma_desc *desc;
	struct macb_tx_skb *tx_skb;
	u32 ctrl;
	unsigned long flags;

#if defined(DEBUG) && defined(VERBOSE_DEBUG)
	rtdev_vdbg(bp->dev,
		   "start_xmit: len %u head %p data %p tail %p end %p\n",
		   skb->len, skb->head, skb->data,
		   rtskb_tail_pointer(skb), rtskb_end_pointer(skb));
	print_hex_dump(KERN_DEBUG, "data: ", DUMP_PREFIX_OFFSET, 16, 1,
		       skb->data, 16, true);
#endif

	len = skb->len;
	rtdm_lock_get_irqsave(&bp->lock, flags);

	/* This is a hard error, log it. */
	if (CIRC_SPACE(bp->tx_head, bp->tx_tail, TX_RING_SIZE) < 1) {
		rtnetif_stop_queue(dev);
		rtdm_lock_put_irqrestore(&bp->lock, flags);
		rtdev_err(bp->dev, "BUG! Tx Ring full when queue awake!\n");
		rtdev_dbg(bp->dev, "tx_head = %u, tx_tail = %u\n",
			   bp->tx_head, bp->tx_tail);
		return RTDEV_TX_BUSY;
	}

	entry = macb_tx_ring_wrap(bp->tx_head);
	rtdev_vdbg(bp->dev, "Allocated ring entry %u\n", entry);
	mapping = dma_map_single(&bp->pdev->dev, skb->data,
				 len, DMA_TO_DEVICE);
	if (dma_mapping_error(&bp->pdev->dev, mapping)) {
		dev_kfree_rtskb(skb);
		goto unlock;
	}

	bp->tx_head++;
	tx_skb = &bp->tx_skb[entry];
	tx_skb->skb = skb;
	tx_skb->mapping = mapping;
	rtdev_vdbg(bp->dev, "Mapped skb data %p to DMA addr %08lx\n",
		   skb->data, (unsigned long)mapping);

	ctrl = MACB_BF(TX_FRMLEN, len);
	ctrl |= MACB_BIT(TX_LAST);
	if (entry == (TX_RING_SIZE - 1))
		ctrl |= MACB_BIT(TX_WRAP);

	desc = &bp->tx_ring[entry];
	desc->addr = mapping;
	desc->ctrl = ctrl;

	/* Make newly initialized descriptor visible to hardware */
	wmb();

	rtskb_tx_timestamp(skb);

	macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(TSTART));

	if (CIRC_SPACE(bp->tx_head, bp->tx_tail, TX_RING_SIZE) < 1)
		rtnetif_stop_queue(dev);

unlock:
	rtdm_lock_put_irqrestore(&bp->lock, flags);

	return RTDEV_TX_OK;
}

static void macb_init_rx_buffer_size(struct macb *bp, size_t size)
{
	if (!macb_is_gem(bp)) {
		bp->rx_buffer_size = MACB_RX_BUFFER_SIZE;
	} else {
		bp->rx_buffer_size = size;

		if (bp->rx_buffer_size % RX_BUFFER_MULTIPLE) {
			rtdev_dbg(bp->dev,
				    "RX buffer must be multiple of %d bytes, expanding\n",
				    RX_BUFFER_MULTIPLE);
			bp->rx_buffer_size =
				roundup(bp->rx_buffer_size, RX_BUFFER_MULTIPLE);
		}
	}

	rtdev_dbg(bp->dev, "mtu [%u] rx_buffer_size [%Zu]\n",
		   bp->dev->mtu, bp->rx_buffer_size);
}

static void gem_free_rx_buffers(struct macb *bp)
{
	struct rtskb		*skb;
	struct macb_dma_desc	*desc;
	dma_addr_t		addr;
	int i;

	if (!bp->rx_skbuff)
		return;

	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = bp->rx_skbuff[i];

		if (skb == NULL)
			continue;

		desc = &bp->rx_ring[i];
		addr = MACB_BF(RX_WADDR, MACB_BFEXT(RX_WADDR, desc->addr));
		dma_unmap_single(&bp->pdev->dev, addr, bp->rx_buffer_size,
				 DMA_FROM_DEVICE);
		dev_kfree_rtskb(skb);
		skb = NULL;
	}

	kfree(bp->rx_skbuff);
	bp->rx_skbuff = NULL;
}

static void macb_free_rx_buffers(struct macb *bp)
{
	if (bp->rx_buffers) {
		dma_free_coherent(&bp->pdev->dev,
				  RX_RING_SIZE * bp->rx_buffer_size,
				  bp->rx_buffers, bp->rx_buffers_dma);
		bp->rx_buffers = NULL;
	}
}

static void macb_free_consistent(struct macb *bp)
{
	if (bp->tx_skb) {
		kfree(bp->tx_skb);
		bp->tx_skb = NULL;
	}
	bp->macbgem_ops.mog_free_rx_buffers(bp);
	if (bp->rx_ring) {
		dma_free_coherent(&bp->pdev->dev, RX_RING_BYTES,
				  bp->rx_ring, bp->rx_ring_dma);
		bp->rx_ring = NULL;
	}
	if (bp->tx_ring) {
		dma_free_coherent(&bp->pdev->dev, TX_RING_BYTES,
				  bp->tx_ring, bp->tx_ring_dma);
		bp->tx_ring = NULL;
	}
}

static int gem_alloc_rx_buffers(struct macb *bp)
{
	int size;

	size = RX_RING_SIZE * sizeof(struct rtskb *);
	bp->rx_skbuff = kzalloc(size, GFP_KERNEL);
	if (!bp->rx_skbuff)
		return -ENOMEM;
	else
		rtdev_dbg(bp->dev,
			   "Allocated %d RX struct rtskb entries at %p\n",
			   RX_RING_SIZE, bp->rx_skbuff);
	return 0;
}

static int macb_alloc_rx_buffers(struct macb *bp)
{
	int size;

	size = RX_RING_SIZE * bp->rx_buffer_size;
	bp->rx_buffers = dma_alloc_coherent(&bp->pdev->dev, size,
					    &bp->rx_buffers_dma, GFP_KERNEL);
	if (!bp->rx_buffers)
		return -ENOMEM;
	else
		rtdev_dbg(bp->dev,
			   "Allocated RX buffers of %d bytes at %08lx (mapped %p)\n",
			   size, (unsigned long)bp->rx_buffers_dma, bp->rx_buffers);
	return 0;
}

static int macb_alloc_consistent(struct macb *bp)
{
	int size;

	size = TX_RING_SIZE * sizeof(struct macb_tx_skb);
	bp->tx_skb = kmalloc(size, GFP_KERNEL);
	if (!bp->tx_skb)
		goto out_err;

	size = RX_RING_BYTES;
	bp->rx_ring = dma_alloc_coherent(&bp->pdev->dev, size,
					 &bp->rx_ring_dma, GFP_KERNEL);
	if (!bp->rx_ring)
		goto out_err;
	rtdev_dbg(bp->dev,
		   "Allocated RX ring of %d bytes at %08lx (mapped %p)\n",
		   size, (unsigned long)bp->rx_ring_dma, bp->rx_ring);

	size = TX_RING_BYTES;
	bp->tx_ring = dma_alloc_coherent(&bp->pdev->dev, size,
					 &bp->tx_ring_dma, GFP_KERNEL);
	if (!bp->tx_ring)
		goto out_err;
	rtdev_dbg(bp->dev,
		   "Allocated TX ring of %d bytes at %08lx (mapped %p)\n",
		   size, (unsigned long)bp->tx_ring_dma, bp->tx_ring);

	if (bp->macbgem_ops.mog_alloc_rx_buffers(bp))
		goto out_err;

	return 0;

out_err:
	macb_free_consistent(bp);
	return -ENOMEM;
}

static void gem_init_rings(struct macb *bp)
{
	int i;

	for (i = 0; i < TX_RING_SIZE; i++) {
		bp->tx_ring[i].addr = 0;
		bp->tx_ring[i].ctrl = MACB_BIT(TX_USED);
	}
	bp->tx_ring[TX_RING_SIZE - 1].ctrl |= MACB_BIT(TX_WRAP);

	bp->rx_tail = bp->rx_prepared_head = bp->tx_head = bp->tx_tail = 0;

	gem_rx_refill(bp);
}

static void macb_init_rings(struct macb *bp)
{
	int i;
	dma_addr_t addr;

	addr = bp->rx_buffers_dma;
	for (i = 0; i < RX_RING_SIZE; i++) {
		bp->rx_ring[i].addr = addr;
		bp->rx_ring[i].ctrl = 0;
		addr += bp->rx_buffer_size;
	}
	bp->rx_ring[RX_RING_SIZE - 1].addr |= MACB_BIT(RX_WRAP);

	for (i = 0; i < TX_RING_SIZE; i++) {
		bp->tx_ring[i].addr = 0;
		bp->tx_ring[i].ctrl = MACB_BIT(TX_USED);
	}
	bp->tx_ring[TX_RING_SIZE - 1].ctrl |= MACB_BIT(TX_WRAP);

	bp->rx_tail = bp->tx_head = bp->tx_tail = 0;
}

static void macb_reset_hw(struct macb *bp)
{
	/*
	 * Disable RX and TX (XXX: Should we halt the transmission
	 * more gracefully?)
	 */
	macb_writel(bp, NCR, 0);

	/* Clear the stats registers (XXX: Update stats first?) */
	macb_writel(bp, NCR, MACB_BIT(CLRSTAT));

	/* Clear all status flags */
	macb_writel(bp, TSR, -1);
	macb_writel(bp, RSR, -1);

	/* Disable all interrupts */
	macb_writel(bp, IDR, -1);
	macb_readl(bp, ISR);
}

static u32 gem_mdc_clk_div(struct macb *bp)
{
	u32 config;
	unsigned long pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= 20000000)
		config = GEM_BF(CLK, GEM_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = GEM_BF(CLK, GEM_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = GEM_BF(CLK, GEM_CLK_DIV32);
	else if (pclk_hz <= 120000000)
		config = GEM_BF(CLK, GEM_CLK_DIV48);
	else if (pclk_hz <= 160000000)
		config = GEM_BF(CLK, GEM_CLK_DIV64);
	else
		config = GEM_BF(CLK, GEM_CLK_DIV96);

	return config;
}

static u32 macb_mdc_clk_div(struct macb *bp)
{
	u32 config;
	unsigned long pclk_hz;

	if (macb_is_gem(bp))
		return gem_mdc_clk_div(bp);

	pclk_hz = clk_get_rate(bp->pclk);
	if (pclk_hz <= 20000000)
		config = MACB_BF(CLK, MACB_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = MACB_BF(CLK, MACB_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = MACB_BF(CLK, MACB_CLK_DIV32);
	else
		config = MACB_BF(CLK, MACB_CLK_DIV64);

	return config;
}

/*
 * Get the DMA bus width field of the network configuration register that we
 * should program.  We find the width from decoding the design configuration
 * register to find the maximum supported data bus width.
 */
static u32 macb_dbw(struct macb *bp)
{
	if (!macb_is_gem(bp))
		return 0;

	switch (GEM_BFEXT(DBWDEF, gem_readl(bp, DCFG1))) {
	case 4:
		return GEM_BF(DBW, GEM_DBW128);
	case 2:
		return GEM_BF(DBW, GEM_DBW64);
	case 1:
	default:
		return GEM_BF(DBW, GEM_DBW32);
	}
}

/*
 * Configure the receive DMA engine
 * - use the correct receive buffer size
 * - set the possibility to use INCR16 bursts
 *   (if not supported by FIFO, it will fallback to default)
 * - set both rx/tx packet buffers to full memory size
 * These are configurable parameters for GEM.
 */
static void macb_configure_dma(struct macb *bp)
{
	u32 dmacfg;

	if (macb_is_gem(bp)) {
		dmacfg = gem_readl(bp, DMACFG) & ~GEM_BF(RXBS, -1L);
		dmacfg |= GEM_BF(RXBS, bp->rx_buffer_size / RX_BUFFER_MULTIPLE);
		dmacfg |= GEM_BF(FBLDO, 16);
		dmacfg |= GEM_BIT(TXPBMS) | GEM_BF(RXBMS, -1L);
		dmacfg &= ~GEM_BIT(ENDIA);
		gem_writel(bp, DMACFG, dmacfg);
	}
}

/*
 * Configure peripheral capacities according to integration options used
 */
static void macb_configure_caps(struct macb *bp)
{
	if (macb_is_gem(bp)) {
		if (GEM_BFEXT(IRQCOR, gem_readl(bp, DCFG1)) == 0)
			bp->caps |= MACB_CAPS_ISR_CLEAR_ON_WRITE;
	}
}

static void macb_init_hw(struct macb *bp)
{
	u32 config;

	macb_reset_hw(bp);
	rtmacb_set_hwaddr(bp);

	config = macb_mdc_clk_div(bp);
	config |= MACB_BF(RBOF, NET_IP_ALIGN);	/* Make eth data aligned */
	config |= MACB_BIT(PAE);		/* PAuse Enable */
	config |= MACB_BIT(DRFCS);		/* Discard Rx FCS */
	if (bp->dev->flags & IFF_PROMISC)
		config |= MACB_BIT(CAF);	/* Copy All Frames */
	if (!(bp->dev->flags & IFF_BROADCAST))
		config |= MACB_BIT(NBC);	/* No BroadCast */
	config |= macb_dbw(bp);
	macb_writel(bp, NCFGR, config);
	bp->speed = SPEED_10;
	bp->duplex = DUPLEX_HALF;

	macb_configure_dma(bp);
	macb_configure_caps(bp);

	/* Initialize TX and RX buffers */
	macb_writel(bp, RBQP, bp->rx_ring_dma);
	macb_writel(bp, TBQP, bp->tx_ring_dma);

	/* Enable TX and RX */
	macb_writel(bp, NCR, MACB_BIT(RE) | MACB_BIT(TE) | MACB_BIT(MPE));

	/* Enable interrupts */
	macb_writel(bp, IER, (MACB_RX_INT_FLAGS
			      | MACB_TX_INT_FLAGS
			      | MACB_BIT(HRESP)));

}

static int macb_open(struct rtnet_device *dev)
{
	struct macb *bp = rtnetdev_priv(dev);
	size_t bufsz = dev->mtu + ETH_HLEN + ETH_FCS_LEN + NET_IP_ALIGN;
	int err;

	rt_stack_connect(dev, &STACK_manager);

	rtdev_dbg(bp->dev, "open\n");

	/* carrier starts down */
	rtnetif_carrier_off(dev);

	/* if the phy is not yet register, retry later*/
	if (!bp->phy_dev)
		return -EAGAIN;

	/* RX buffers initialization */
	macb_init_rx_buffer_size(bp, bufsz);

	err = macb_alloc_consistent(bp);
	if (err) {
		rtdev_err(dev, "Unable to allocate DMA memory (error %d)\n",
			   err);
		return err;
	}

	bp->macbgem_ops.mog_init_rings(bp);
	macb_init_hw(bp);

	/* schedule a link state check */
	phy_start(bp->phy_dev);

	rtnetif_start_queue(dev);

	return 0;
}

static int macb_close(struct rtnet_device *dev)
{
	struct macb *bp = rtnetdev_priv(dev);
	unsigned long flags;

	rtnetif_stop_queue(dev);

	if (bp->phy_dev)
		phy_stop(bp->phy_dev);

	rtdm_lock_get_irqsave(&bp->lock, flags);
	macb_reset_hw(bp);
	rtnetif_carrier_off(dev);
	rtdm_lock_put_irqrestore(&bp->lock, flags);

	macb_free_consistent(bp);

	rt_stack_disconnect(dev);

	return 0;
}

static void gem_update_stats(struct macb *bp)
{
	u32 __iomem *reg = bp->regs + GEM_OTX;
	u32 *p = &bp->hw_stats.gem.tx_octets_31_0;
	u32 *end = &bp->hw_stats.gem.rx_udp_checksum_errors + 1;

	for (; p < end; p++, reg++)
		*p += __raw_readl(reg);
}

static struct net_device_stats *gem_get_stats(struct macb *bp)
{
	struct gem_stats *hwstat = &bp->hw_stats.gem;
	struct net_device_stats *nstat = &bp->stats;

	gem_update_stats(bp);

	nstat->rx_errors = (hwstat->rx_frame_check_sequence_errors +
			    hwstat->rx_alignment_errors +
			    hwstat->rx_resource_errors +
			    hwstat->rx_overruns +
			    hwstat->rx_oversize_frames +
			    hwstat->rx_jabbers +
			    hwstat->rx_undersized_frames +
			    hwstat->rx_length_field_frame_errors);
	nstat->tx_errors = (hwstat->tx_late_collisions +
			    hwstat->tx_excessive_collisions +
			    hwstat->tx_underrun +
			    hwstat->tx_carrier_sense_errors);
	nstat->multicast = hwstat->rx_multicast_frames;
	nstat->collisions = (hwstat->tx_single_collision_frames +
			     hwstat->tx_multiple_collision_frames +
			     hwstat->tx_excessive_collisions);
	nstat->rx_length_errors = (hwstat->rx_oversize_frames +
				   hwstat->rx_jabbers +
				   hwstat->rx_undersized_frames +
				   hwstat->rx_length_field_frame_errors);
	nstat->rx_over_errors = hwstat->rx_resource_errors;
	nstat->rx_crc_errors = hwstat->rx_frame_check_sequence_errors;
	nstat->rx_frame_errors = hwstat->rx_alignment_errors;
	nstat->rx_fifo_errors = hwstat->rx_overruns;
	nstat->tx_aborted_errors = hwstat->tx_excessive_collisions;
	nstat->tx_carrier_errors = hwstat->tx_carrier_sense_errors;
	nstat->tx_fifo_errors = hwstat->tx_underrun;

	return nstat;
}

struct net_device_stats *rtmacb_get_stats(struct rtnet_device *dev)
{
	struct macb *bp = rtnetdev_priv(dev);
	struct net_device_stats *nstat = &bp->stats;
	struct macb_stats *hwstat = &bp->hw_stats.macb;

	if (macb_is_gem(bp))
		return gem_get_stats(bp);

	/* read stats from hardware */
	macb_update_stats(bp);

	/* Convert HW stats into netdevice stats */
	nstat->rx_errors = (hwstat->rx_fcs_errors +
			    hwstat->rx_align_errors +
			    hwstat->rx_resource_errors +
			    hwstat->rx_overruns +
			    hwstat->rx_oversize_pkts +
			    hwstat->rx_jabbers +
			    hwstat->rx_undersize_pkts +
			    hwstat->sqe_test_errors +
			    hwstat->rx_length_mismatch);
	nstat->tx_errors = (hwstat->tx_late_cols +
			    hwstat->tx_excessive_cols +
			    hwstat->tx_underruns +
			    hwstat->tx_carrier_errors);
	nstat->collisions = (hwstat->tx_single_cols +
			     hwstat->tx_multiple_cols +
			     hwstat->tx_excessive_cols);
	nstat->rx_length_errors = (hwstat->rx_oversize_pkts +
				   hwstat->rx_jabbers +
				   hwstat->rx_undersize_pkts +
				   hwstat->rx_length_mismatch);
	nstat->rx_over_errors = hwstat->rx_resource_errors +
				   hwstat->rx_overruns;
	nstat->rx_crc_errors = hwstat->rx_fcs_errors;
	nstat->rx_frame_errors = hwstat->rx_align_errors;
	nstat->rx_fifo_errors = hwstat->rx_overruns;
	/* XXX: What does "missed" mean? */
	nstat->tx_aborted_errors = hwstat->tx_excessive_cols;
	nstat->tx_carrier_errors = hwstat->tx_carrier_errors;
	nstat->tx_fifo_errors = hwstat->tx_underruns;
	/* Don't know about heartbeat or window errors... */

	return nstat;
}
EXPORT_SYMBOL_GPL(rtmacb_get_stats);

int rtmacb_ioctl(struct rtnet_device *dev, unsigned cmd, void *rq)
{
	struct macb *bp = rtnetdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!rtnetif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	return phy_mii_ioctl(phydev, rq, cmd);
}
EXPORT_SYMBOL_GPL(rtmacb_ioctl);

#if defined(CONFIG_OF)
static const struct of_device_id macb_dt_ids[] = {
	{ .compatible = "cdns,at32ap7000-macb" },
	{ .compatible = "cdns,at91sam9260-macb" },
	{ .compatible = "cdns,macb" },
	{ .compatible = "cdns,pc302-gem" },
	{ .compatible = "cdns,gem" },
	{ .compatible = "atmel,sama5d3-gem" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, macb_dt_ids);
#endif

static int __init macb_probe(struct platform_device *pdev)
{
	struct macb_platform_data *pdata;
	struct resource *regs;
	struct rtnet_device *dev;
	struct macb *bp;
	struct phy_device *phydev;
	u32 config;
	int err = -ENXIO;
	struct pinctrl *pinctrl;
	const char *mac;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "no mmio resource defined\n");
		goto err_out;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		err = PTR_ERR(pinctrl);
		if (err == -EPROBE_DEFER)
			goto err_out;

		dev_warn(&pdev->dev, "No pinctrl provided\n");
	}

	err = -ENOMEM;
	dev = rt_alloc_etherdev(sizeof(*bp), RX_RING_SIZE * 2 + TX_RING_SIZE);
	if (!dev)
		goto err_out;

	rtdev_alloc_name(dev, "rteth%d");
	rt_rtdev_connect(dev, &RTDEV_manager);
	dev->vers = RTDEV_VERS_2_0;

	/* TODO: Actually, we have some interesting features... */
	dev->features |= 0;

	bp = rtnetdev_priv(dev);
	bp->pdev = pdev;
	bp->dev = dev;

	rtdm_lock_init(&bp->lock);
	INIT_WORK(&bp->tx_error_task, macb_tx_error_task);

	bp->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(bp->pclk)) {
		err = PTR_ERR(bp->pclk);
		dev_err(&pdev->dev, "failed to get macb_clk (%u)\n", err);
		goto err_out_free_dev;
	}

	bp->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(bp->hclk)) {
		err = PTR_ERR(bp->hclk);
		dev_err(&pdev->dev, "failed to get hclk (%u)\n", err);
		goto err_out_free_dev;
	}

	bp->tx_clk = devm_clk_get(&pdev->dev, "tx_clk");

	err = clk_prepare_enable(bp->pclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pclk (%u)\n", err);
		goto err_out_free_dev;
	}

	err = clk_prepare_enable(bp->hclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable hclk (%u)\n", err);
		goto err_out_disable_pclk;
	}

	if (!IS_ERR(bp->tx_clk)) {
		err = clk_prepare_enable(bp->tx_clk);
		if (err) {
			dev_err(&pdev->dev, "failed to enable tx_clk (%u)\n",
					err);
			goto err_out_disable_hclk;
		}
	}

	bp->regs = devm_ioremap(&pdev->dev, regs->start, resource_size(regs));
	if (!bp->regs) {
		dev_err(&pdev->dev, "failed to map registers, aborting.\n");
		err = -ENOMEM;
		goto err_out_disable_clocks;
	}

	dev->irq = platform_get_irq(pdev, 0);
	rt_stack_connect(dev, &STACK_manager);

	err = rtdm_irq_request(&bp->irq_handle, dev->irq, macb_interrupt, 0,
			dev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "Unable to request IRQ %d (error %d)\n",
			dev->irq, err);
		goto err_out_disable_clocks;
	}

	dev->open = macb_open;
	dev->stop = macb_close;
	dev->hard_start_xmit = macb_start_xmit;
	dev->do_ioctl = rtmacb_ioctl;
	dev->get_stats = rtmacb_get_stats;

	dev->base_addr = regs->start;

	/* setup appropriated routines according to adapter type */
	if (macb_is_gem(bp)) {
		bp->macbgem_ops.mog_alloc_rx_buffers = gem_alloc_rx_buffers;
		bp->macbgem_ops.mog_free_rx_buffers = gem_free_rx_buffers;
		bp->macbgem_ops.mog_init_rings = gem_init_rings;
		bp->macbgem_ops.mog_rx = gem_rx;
	} else {
		bp->macbgem_ops.mog_alloc_rx_buffers = macb_alloc_rx_buffers;
		bp->macbgem_ops.mog_free_rx_buffers = macb_free_rx_buffers;
		bp->macbgem_ops.mog_init_rings = macb_init_rings;
		bp->macbgem_ops.mog_rx = macb_rx;
	}

	/* Set MII management clock divider */
	config = macb_mdc_clk_div(bp);
	config |= macb_dbw(bp);
	macb_writel(bp, NCFGR, config);

	mac = of_get_mac_address(pdev->dev.of_node);
	if (mac)
		memcpy(bp->dev->dev_addr, mac, ETH_ALEN);
	else
		rtmacb_get_hwaddr(bp);

	err = of_get_phy_mode(pdev->dev.of_node);
	if (err < 0) {
		pdata = dev_get_platdata(&pdev->dev);
		if (pdata && pdata->is_rmii)
			bp->phy_interface = PHY_INTERFACE_MODE_RMII;
		else
			bp->phy_interface = PHY_INTERFACE_MODE_MII;
	} else {
		bp->phy_interface = err;
	}

	if (bp->phy_interface == PHY_INTERFACE_MODE_RGMII)
		macb_or_gem_writel(bp, USRIO, GEM_BIT(RGMII));
	else if (bp->phy_interface == PHY_INTERFACE_MODE_RMII)
#if defined(CONFIG_ARCH_AT91)
		macb_or_gem_writel(bp, USRIO, (MACB_BIT(RMII) |
					       MACB_BIT(CLKEN)));
#else
		macb_or_gem_writel(bp, USRIO, 0);
#endif
	else
#if defined(CONFIG_ARCH_AT91)
		macb_or_gem_writel(bp, USRIO, MACB_BIT(CLKEN));
#else
		macb_or_gem_writel(bp, USRIO, MACB_BIT(MII));
#endif

	err = rt_register_rtnetdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Cannot register net device, aborting.\n");
		goto err_out_irq_free;
	}

	err = rtmacb_mii_init(bp);
	if (err)
		goto err_out_unregister_netdev;

	platform_set_drvdata(pdev, dev);

	rtnetif_carrier_off(dev);

	rtdev_info(dev, "Cadence %s at 0x%08lx irq %d (%pM)\n",
		    macb_is_gem(bp) ? "GEM" : "MACB", dev->base_addr,
		    dev->irq, dev->dev_addr);

	phydev = bp->phy_dev;
	rtdev_info(dev, "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	return 0;

err_out_unregister_netdev:
	rt_unregister_rtnetdev(dev);
err_out_irq_free:
	rtdm_irq_free(&bp->irq_handle);
err_out_disable_clocks:
	if (!IS_ERR(bp->tx_clk))
		clk_disable_unprepare(bp->tx_clk);
err_out_disable_hclk:
	clk_disable_unprepare(bp->hclk);
err_out_disable_pclk:
	clk_disable_unprepare(bp->pclk);
err_out_free_dev:
	rtdev_free(dev);
err_out:
	return err;
}

static int __exit macb_remove(struct platform_device *pdev)
{
	struct rtnet_device *dev;
	struct macb *bp;

	dev = platform_get_drvdata(pdev);

	if (dev) {
		bp = rtnetdev_priv(dev);
		if (bp->phy_dev)
			phy_disconnect(bp->phy_dev);
		mdiobus_unregister(bp->mii_bus);
		if (bp->phy_phony_net_device)
			free_netdev(bp->phy_phony_net_device);
		kfree(bp->mii_bus->irq);
		rt_rtdev_disconnect(dev);
		rtdm_irq_free(&bp->irq_handle);
		mdiobus_free(bp->mii_bus);
		rt_unregister_rtnetdev(dev);
		if (!IS_ERR(bp->tx_clk))
			clk_disable_unprepare(bp->tx_clk);
		clk_disable_unprepare(bp->hclk);
		clk_disable_unprepare(bp->pclk);
		rtdev_free(dev);
	}

	return 0;
}

static struct platform_driver macb_driver = {
	.remove		= __exit_p(macb_remove),
	.driver		= {
		.name		= "macb",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(macb_dt_ids),
	},
};

static bool found;
static int __init macb_driver_init(void)
{
	found = platform_driver_probe(&macb_driver, macb_probe) == 0;
	return 0;
}
module_init(macb_driver_init);

static void __exit macb_driver_exit(void)
{
	if (found)
		platform_driver_unregister(&macb_driver);
}
module_exit(macb_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cadence MACB/GEM Ethernet driver");
MODULE_AUTHOR("Haavard Skinnemoen (Atmel)");
MODULE_ALIAS("platform:macb");
