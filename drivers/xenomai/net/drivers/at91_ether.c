/*
 * Ethernet driver for the Atmel AT91RM9200 (Thunder)
 *
 *  Copyright (C) 2003 SAN People (Pty) Ltd
 *
 * Based on an earlier Atmel EMAC macrocell driver by Atmel and Lineo Inc.
 * Initial version by Rick Bronson 01/11/2003
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * RTnet port:
 * Copyright (C) 2014 Gilles Chanteperdrix <gch@xenomai.org>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/platform_data/macb.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gfp.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>

#include <rtdev.h>
#include <rtdm/net.h>
#include <rtnet_port.h>
#include <rtskb.h>
#include "rt_macb.h"

/* 1518 rounded up */
#define MAX_RBUFF_SZ	0x600
/* max number of receive buffers */
#define MAX_RX_DESCR	9

/* Initialize and start the Receiver and Transmit subsystems */
static int at91ether_start(struct rtnet_device *dev)
{
	struct macb *lp = rtnetdev_priv(dev);
	dma_addr_t addr;
	u32 ctl;
	int i;

	lp->rx_ring = dma_alloc_coherent(&lp->pdev->dev,
					 (MAX_RX_DESCR *
					  sizeof(struct macb_dma_desc)),
					 &lp->rx_ring_dma, GFP_KERNEL);
	if (!lp->rx_ring)
		return -ENOMEM;

	lp->rx_buffers = dma_alloc_coherent(&lp->pdev->dev,
					    MAX_RX_DESCR * MAX_RBUFF_SZ,
					    &lp->rx_buffers_dma, GFP_KERNEL);
	if (!lp->rx_buffers) {
		dma_free_coherent(&lp->pdev->dev,
				  MAX_RX_DESCR * sizeof(struct macb_dma_desc),
				  lp->rx_ring, lp->rx_ring_dma);
		lp->rx_ring = NULL;
		return -ENOMEM;
	}

	addr = lp->rx_buffers_dma;
	for (i = 0; i < MAX_RX_DESCR; i++) {
		lp->rx_ring[i].addr = addr;
		lp->rx_ring[i].ctrl = 0;
		addr += MAX_RBUFF_SZ;
	}

	/* Set the Wrap bit on the last descriptor */
	lp->rx_ring[MAX_RX_DESCR - 1].addr |= MACB_BIT(RX_WRAP);

	/* Reset buffer index */
	lp->rx_tail = 0;

	/* Program address of descriptor list in Rx Buffer Queue register */
	macb_writel(lp, RBQP, lp->rx_ring_dma);

	/* Enable Receive and Transmit */
	ctl = macb_readl(lp, NCR);
	macb_writel(lp, NCR, ctl | MACB_BIT(RE) | MACB_BIT(TE));

	return 0;
}

/* Open the ethernet interface */
static int at91ether_open(struct rtnet_device *dev)
{
	struct macb *lp = rtnetdev_priv(dev);
	u32 ctl;
	int ret;

	rt_stack_connect(dev, &STACK_manager);

	/* Clear internal statistics */
	ctl = macb_readl(lp, NCR);
	macb_writel(lp, NCR, ctl | MACB_BIT(CLRSTAT));

	rtmacb_set_hwaddr(lp);

	ret = at91ether_start(dev);
	if (ret)
		return ret;

	/* Enable MAC interrupts */
	macb_writel(lp, IER, MACB_BIT(RCOMP)	|
			     MACB_BIT(RXUBR)	|
			     MACB_BIT(ISR_TUND)	|
			     MACB_BIT(ISR_RLE)	|
			     MACB_BIT(TCOMP)	|
			     MACB_BIT(ISR_ROVR)	|
			     MACB_BIT(HRESP));

	/* schedule a link state check */
	phy_start(lp->phy_dev);

	rtnetif_start_queue(dev);

	return 0;
}

/* Close the interface */
static int at91ether_close(struct rtnet_device *dev)
{
	struct macb *lp = rtnetdev_priv(dev);
	u32 ctl;

	/* Disable Receiver and Transmitter */
	ctl = macb_readl(lp, NCR);
	macb_writel(lp, NCR, ctl & ~(MACB_BIT(TE) | MACB_BIT(RE)));

	/* Disable MAC interrupts */
	macb_writel(lp, IDR, MACB_BIT(RCOMP)	|
			     MACB_BIT(RXUBR)	|
			     MACB_BIT(ISR_TUND)	|
			     MACB_BIT(ISR_RLE)	|
			     MACB_BIT(TCOMP)	|
			     MACB_BIT(ISR_ROVR) |
			     MACB_BIT(HRESP));

	rtnetif_stop_queue(dev);

	dma_free_coherent(&lp->pdev->dev,
				MAX_RX_DESCR * sizeof(struct macb_dma_desc),
				lp->rx_ring, lp->rx_ring_dma);
	lp->rx_ring = NULL;

	dma_free_coherent(&lp->pdev->dev,
				MAX_RX_DESCR * MAX_RBUFF_SZ,
				lp->rx_buffers, lp->rx_buffers_dma);
	lp->rx_buffers = NULL;

	rt_stack_disconnect(dev);

	return 0;
}

/* Transmit packet */
static int at91ether_start_xmit(struct rtskb *skb, struct rtnet_device *dev)
{
	struct macb *lp = rtnetdev_priv(dev);

	if (macb_readl(lp, TSR) & MACB_BIT(RM9200_BNQ)) {
		rtnetif_stop_queue(dev);

		/* Store packet information (to free when Tx completed) */
		lp->skb = skb;
		lp->skb_length = skb->len;
		lp->skb_physaddr = dma_map_single(NULL, skb->data, skb->len,
							DMA_TO_DEVICE);

		/* Set address of the data in the Transmit Address register */
		macb_writel(lp, TAR, lp->skb_physaddr);
		/* Set length of the packet in the Transmit Control register */
		macb_writel(lp, TCR, skb->len);

	} else {
		rtdev_err(dev, "%s called, but device is busy!\n", __func__);
		return RTDEV_TX_BUSY;
	}

	return RTDEV_TX_OK;
}

/* Extract received frame from buffer descriptors and sent to upper layers.
 * (Called from interrupt context)
 */
static bool at91ether_rx(struct rtnet_device *dev, nanosecs_abs_t *time_stamp)
{
	struct macb *lp = rtnetdev_priv(dev);
	unsigned char *p_recv;
	struct rtskb *skb;
	unsigned int pktlen;
	bool ret = false;

	while (lp->rx_ring[lp->rx_tail].addr & MACB_BIT(RX_USED)) {
		p_recv = lp->rx_buffers + lp->rx_tail * MAX_RBUFF_SZ;
		pktlen = MACB_BF(RX_FRMLEN, lp->rx_ring[lp->rx_tail].ctrl);
		skb = rtnetdev_alloc_rtskb(dev, pktlen + 2);
		if (skb) {
			rtskb_reserve(skb, 2);
			memcpy(rtskb_put(skb, pktlen), p_recv, pktlen);

			skb->protocol = rt_eth_type_trans(skb, dev);
			lp->stats.rx_packets++;
			lp->stats.rx_bytes += pktlen;
			ret = true;
			skb->time_stamp = *time_stamp;
			rtnetif_rx(skb);
		} else {
			lp->stats.rx_dropped++;
		}

		if (lp->rx_ring[lp->rx_tail].ctrl & MACB_BIT(RX_MHASH_MATCH))
			lp->stats.multicast++;

		/* reset ownership bit */
		lp->rx_ring[lp->rx_tail].addr &= ~MACB_BIT(RX_USED);

		/* wrap after last buffer */
		if (lp->rx_tail == MAX_RX_DESCR - 1)
			lp->rx_tail = 0;
		else
			lp->rx_tail++;
	}

	return ret;
}

/* MAC interrupt handler */
static int at91ether_interrupt(rtdm_irq_t *irq_handle)
{
	void *dev_id = rtdm_irq_get_arg(irq_handle, void);
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtnet_device *dev = dev_id;
	struct macb *lp = rtnetdev_priv(dev);
	u32 intstatus, ctl;

	/* MAC Interrupt Status register indicates what interrupts are pending.
	 * It is automatically cleared once read.
	 */
	intstatus = macb_readl(lp, ISR);

	/* Receive complete */
	if ((intstatus & MACB_BIT(RCOMP)) && at91ether_rx(dev, &time_stamp))
		rt_mark_stack_mgr(dev);

	/* Transmit complete */
	if (intstatus & MACB_BIT(TCOMP)) {
		/* The TCOM bit is set even if the transmission failed */
		if (intstatus & (MACB_BIT(ISR_TUND) | MACB_BIT(ISR_RLE)))
			lp->stats.tx_errors++;

		if (lp->skb) {
			dev_kfree_rtskb(lp->skb);
			lp->skb = NULL;
			dma_unmap_single(NULL, lp->skb_physaddr, lp->skb_length, DMA_TO_DEVICE);
			lp->stats.tx_packets++;
			lp->stats.tx_bytes += lp->skb_length;
		}
		rtnetif_wake_queue(dev);
	}

	/* Work-around for EMAC Errata section 41.3.1 */
	if (intstatus & MACB_BIT(RXUBR)) {
		ctl = macb_readl(lp, NCR);
		macb_writel(lp, NCR, ctl & ~MACB_BIT(RE));
		macb_writel(lp, NCR, ctl | MACB_BIT(RE));
	}

	if (intstatus & MACB_BIT(ISR_ROVR))
		rtdev_err(dev, "ROVR error\n");

	return RTDM_IRQ_HANDLED;
}

#if defined(CONFIG_OF)
static const struct of_device_id at91ether_dt_ids[] = {
	{ .compatible = "cdns,at91rm9200-emac" },
	{ .compatible = "cdns,emac" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, at91ether_dt_ids);
#endif

/* Detect MAC & PHY and perform ethernet interface initialization */
static int __init at91ether_probe(struct platform_device *pdev)
{
	struct macb_platform_data *board_data = dev_get_platdata(&pdev->dev);
	struct resource *regs;
	struct rtnet_device *dev;
	struct phy_device *phydev;
	struct macb *lp;
	int res;
	u32 reg;
	const char *mac;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENOENT;

	dev = rt_alloc_etherdev(sizeof(struct macb), MAX_RX_DESCR * 2 + 2);
	if (!dev)
		return -ENOMEM;

	rtdev_alloc_name(dev, "rteth%d");
	rt_rtdev_connect(dev, &RTDEV_manager);
	dev->vers = RTDEV_VERS_2_0;
	dev->sysbind = &pdev->dev;

	lp = rtnetdev_priv(dev);
	lp->pdev = pdev;
	lp->dev = dev;
	rtdm_lock_init(&lp->lock);

	/* physical base address */
	dev->base_addr = regs->start;
	lp->regs = devm_ioremap(&pdev->dev, regs->start, resource_size(regs));
	if (!lp->regs) {
		res = -ENOMEM;
		goto err_free_dev;
	}

	/* Clock */
	lp->pclk = devm_clk_get(&pdev->dev, "ether_clk");
	if (IS_ERR(lp->pclk)) {
		res = PTR_ERR(lp->pclk);
		goto err_free_dev;
	}
	clk_enable(lp->pclk);

	lp->hclk = ERR_PTR(-ENOENT);
	lp->tx_clk = ERR_PTR(-ENOENT);

	/* Install the interrupt handler */
	dev->irq = platform_get_irq(pdev, 0);
	res = rtdm_irq_request(&lp->irq_handle, dev->irq, at91ether_interrupt, 0, dev->name, dev);
	if (res)
		goto err_disable_clock;

	dev->open = at91ether_open;
	dev->stop = at91ether_close;
	dev->hard_start_xmit = at91ether_start_xmit;
	dev->do_ioctl = rtmacb_ioctl;
	dev->get_stats = rtmacb_get_stats;

	platform_set_drvdata(pdev, dev);

	mac = of_get_mac_address(pdev->dev.of_node);
	if (mac)
		memcpy(lp->dev->dev_addr, mac, ETH_ALEN);
	else
		rtmacb_get_hwaddr(lp);

	res = of_get_phy_mode(pdev->dev.of_node);
	if (res < 0) {
		if (board_data && board_data->is_rmii)
			lp->phy_interface = PHY_INTERFACE_MODE_RMII;
		else
			lp->phy_interface = PHY_INTERFACE_MODE_MII;
	} else {
		lp->phy_interface = res;
	}

	macb_writel(lp, NCR, 0);

	reg = MACB_BF(CLK, MACB_CLK_DIV32) | MACB_BIT(BIG);
	if (lp->phy_interface == PHY_INTERFACE_MODE_RMII)
		reg |= MACB_BIT(RM9200_RMII);

	macb_writel(lp, NCFGR, reg);

	/* Register the network interface */
	res = rt_register_rtnetdev(dev);
	if (res)
		goto err_irq_free;

	res = rtmacb_mii_init(lp);
	if (res)
		goto err_out_unregister_netdev;

	/* will be enabled in open() */
	rtnetif_carrier_off(dev);

	phydev = lp->phy_dev;
	rtdev_info(dev, "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
				phydev->drv->name, dev_name(&phydev->dev),
				phydev->irq);

	/* Display ethernet banner */
	rtdev_info(dev, "AT91 ethernet at 0x%08lx int=%d (%pM)\n",
				dev->base_addr, dev->irq, dev->dev_addr);

	return 0;

err_out_unregister_netdev:
	rt_unregister_rtnetdev(dev);
err_irq_free:
	rtdm_irq_free(&lp->irq_handle);
err_disable_clock:
	clk_disable(lp->pclk);
err_free_dev:
	rtdev_free(dev);
	return res;
}

static int at91ether_remove(struct platform_device *pdev)
{
	struct rtnet_device *dev = platform_get_drvdata(pdev);
	struct macb *lp = rtnetdev_priv(dev);

	if (lp->phy_dev)
		phy_disconnect(lp->phy_dev);

	mdiobus_unregister(lp->mii_bus);
	if (lp->phy_phony_net_device)
		free_netdev(lp->phy_phony_net_device);
	kfree(lp->mii_bus->irq);
	rt_rtdev_disconnect(dev);
	rtdm_irq_free(&lp->irq_handle);
	mdiobus_free(lp->mii_bus);
	rt_unregister_rtnetdev(dev);
	clk_disable(lp->pclk);
	rtdev_free(dev);

	return 0;
}

static struct platform_driver at91ether_driver = {
	.remove		= at91ether_remove,
	.driver		= {
		.name	= "at91_ether",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(at91ether_dt_ids),
	},
};

module_platform_driver_probe(at91ether_driver, at91ether_probe);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AT91RM9200 EMAC Ethernet driver");
MODULE_AUTHOR("Andrew Victor");
MODULE_ALIAS("platform:at91_ether");
