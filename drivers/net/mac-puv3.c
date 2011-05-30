/*
 * Code specific to PKUnity SoC and UniCore ISA
 *
 *	Maintained by GUAN Xue-tao <gxt@mprc.pku.edu.cn>
 *	Copyright (C) 2001-2010 Guan Xuetao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/bug.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/cache.h>
#include <linux/io.h>

#include <mach/hardware.h>

MODULE_DESCRIPTION("PKUNITY-3 SoC Ethernet driver");
MODULE_LICENSE("GPL v2");

/**********************************************************************
 *  Globals
 ********************************************************************* */
static const char umal_string[] = "PKUnity-v3-UMAL";
static const char umal_mdio_string[] = "umal-mdio";

/**********************************************************************
 *  Simple types
 ********************************************************************* */
enum umal_speed {
	umal_speed_none = 0,
	umal_speed_10 = SPEED_10,
	umal_speed_100 = SPEED_100,
	umal_speed_1000 = SPEED_1000,
};

enum umal_duplex {
	umal_duplex_none = -1,
	umal_duplex_half = DUPLEX_HALF,
	umal_duplex_full = DUPLEX_FULL,
};

enum umal_fc {
	umal_fc_none,
	umal_fc_disabled,
	umal_fc_frame,
	umal_fc_collision,
	umal_fc_carrier,
};

enum umal_state {
	umal_state_uninit,
	umal_state_off,
	umal_state_on,
	umal_state_broken,
};

/**********************************************************************
 *  Macros
 ********************************************************************* */
#define UMALDMA_NEXTBUF(d, f) ((((d)->f+1) == (d)->umaldma_dscrtable_end) ? \
			  (d)->umaldma_dscrtable : (d)->f+1)

#define DMA_RX			0
#define DMA_TX			1

#define UMAL_MAX_TXDESCR	256
#define UMAL_MAX_RXDESCR	256

#define ETHER_ADDR_LEN		6
#define ENET_PACKET_SIZE	1518

/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT		(2*HZ)

/**********************************************************************
 *  DMA Descriptor structure
 ********************************************************************* */
struct umaldmadscr {
	dma_addr_t   PacketStartAddr;
	int          PacketSize;
	dma_addr_t   NextDescriptor;
	struct umaldmadscr *NextDescriptor_Virt;
};

/**********************************************************************
 *  DMA Controller structure
 ********************************************************************* */
struct umaldma {
	/*
	 * This stuff is used to identify the channel and the registers
	 * associated with it.
	 */
	/* back pointer to associated MAC */
	struct umal_softc	*umaldma_eth;
	/* direction (1=transmit) */
	int			umaldma_txdir;
	/* total # of descriptors in ring */
	int			umaldma_maxdescr;
	/*
	 * This stuff is for maintenance of the ring
	 */
	/* base of descriptor table */
	struct umaldmadscr	*umaldma_dscrtable;
	void			*umaldma_dscrtable_unaligned;
	/* and also the phys addr */
	dma_addr_t		umaldma_dscrtable_phys;
	/* and also the phys addr */
	dma_addr_t		umaldma_dscrtable_phys_unaligned;
	/* end of descriptor table */
	struct umaldmadscr	*umaldma_dscrtable_end;
	/* context table, one per descr */
	struct sk_buff		**umaldma_ctxtable;
	/* next dscr for sw to add */
	struct umaldmadscr	*umaldma_addptr;
	/* next dscr for sw to remove */
	struct umaldmadscr	*umaldma_remptr;
};

/**********************************************************************
 *  Ethernet softc structure
 ********************************************************************* */
struct umal_softc {
	/* Linux-specific things */
	struct net_device	*umal_dev;	/* pointer to linux device */
	int			umal_idx;
	struct phy_device	*phy_dev;	/* the associated PHY device */
	struct mii_bus		*mii_bus;	/* the MII bus */
	int			phy_irq[PHY_MAX_ADDR];
	spinlock_t		umal_lock;	/* spin lock */
	int			umal_devflags;	/* current device flags */

	/* Controller-specific things */
	enum umal_state		umal_state;	/* current state */
	unsigned char		umal_hwaddr[ETHER_ADDR_LEN];

	enum umal_speed		umal_speed;	/* current speed */
	enum umal_duplex	umal_duplex;	/* current duplex */
	enum umal_fc		umal_fc;	/* cur. flow control setting */
	int			umal_pause;	/* current pause setting */
	int			umal_link;	/* current link state */

	struct umaldma		umal_rxdma;	/* rx dma channel */
	struct umaldma		umal_txdma;	/* tx dma channel */
};

/**********************************************************************
 *  Prototypes
 ********************************************************************* */
static int umal_mii_reset(struct mii_bus *bus);
static int umal_mii_read(struct mii_bus *bus, int phyaddr, int regidx);
static int umal_mii_write(struct mii_bus *bus, int phyaddr, int regidx,
		u16 val);
static int umal_mii_probe(struct net_device *dev);

static void umaldma_initctx(struct umaldma *d, struct umal_softc *s,
		int rxtx, int maxdescr);
static void umaldma_uninitctx(struct umaldma *d);
static void umaldma_channel_start(struct umaldma *d, int rxtx);
static void umaldma_channel_stop(struct umaldma *d);
static int umaldma_add_rcvbuffer(struct umal_softc *sc, struct umaldma *d,
		struct sk_buff *m);
static int umaldma_add_txbuffer(struct umaldma *d, struct sk_buff *m);
static void umaldma_emptyring(struct umaldma *d);
static void umaldma_fillring(struct umal_softc *sc, struct umaldma *d);
static int umaldma_rx_process(struct umal_softc *sc, struct umaldma *d,
		int work_to_do, int poll);
static void umaldma_tx_process(struct umal_softc *sc, struct umaldma *d,
		int poll);

static int umal_initctx(struct umal_softc *s);
static void umal_uninitctx(struct umal_softc *s);
static void umal_channel_start(struct umal_softc *s);
static void umal_channel_stop(struct umal_softc *s);
static enum umal_state umal_set_channel_state(struct umal_softc *,
		enum umal_state);

static int umal_init(struct platform_device *pldev, long long base);
static int umal_open(struct net_device *dev);
static int umal_close(struct net_device *dev);
static int umal_mii_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
static irqreturn_t umal_intr(int irq, void *dev_instance);
static void umal_clr_intr(struct net_device *dev);
static int umal_start_tx(struct sk_buff *skb, struct net_device *dev);
static void umal_tx_timeout(struct net_device *dev);
static void umal_set_rx_mode(struct net_device *dev);
static void umal_promiscuous_mode(struct umal_softc *sc, int onoff);
static void umal_setmulti(struct umal_softc *sc);
static int umal_set_speed(struct umal_softc *s, enum umal_speed speed);
static int umal_set_duplex(struct umal_softc *s, enum umal_duplex duplex,
		enum umal_fc fc);
static int umal_change_mtu(struct net_device *_dev, int new_mtu);
static void umal_miipoll(struct net_device *dev);

/**********************************************************************
 *  MII Bus functions for PAL (phy abstraction layer)
 ********************************************************************* */

/**********************************************************************
 *  UMAL_MII_RESET(bus)
 *
 *  Reset MII bus.
 *
 *  Input parameters:
 *	   bus     - MDIO bus handle
 *
 *  Return value:
 *	   0 if ok
 ********************************************************************* */
static int umal_mii_reset(struct mii_bus *bus)
{
	/* reset the MII management */
	writel(UMAL_MIICFG_RESET, UMAL_MIICFG);
	/* enable the MII management */
	writel(0, UMAL_MIICFG);
	/* source clock division = 28 */
	writel(readl(UMAL_MIICFG) | 0x7, UMAL_MIICFG);

	return 0;
}

/**********************************************************************
 *  UMAL_MII_READ(bus, phyaddr, regidx)
 *  Read a PHY register.
 *
 *  Input parameters:
 *	   bus     - MDIO bus handle
 *	   phyaddr - PHY's address
 *	   regnum  - index of register to read
 *
 *  Return value:
 *	   value read, or 0xffff if an error occurred.
 ********************************************************************* */
static int umal_mii_read(struct mii_bus *bus, int phyaddr, int regidx)
{
	int tmp = 0;

	writel((phyaddr<<8) | regidx, UMAL_MIIADDR);
	writel(UMAL_MIICMD_READ, UMAL_MIICMD);

	tmp = readl(UMAL_MIIIDCT);
	while (tmp & UMAL_MIIIDCT_BUSY)
		tmp = readl(UMAL_MIIIDCT);

	if (tmp & UMAL_MIIIDCT_NOTVALID)
		return 0xffff;

	writel(0, UMAL_MIICMD);

	tmp = readl(UMAL_MIISTATUS);
	return tmp;
}

/**********************************************************************
 *  UMAL_MII_WRITE(bus, phyaddr, regidx, regval)
 *
 *  Write a value to a PHY register.
 *
 *  Input parameters:
 *	   bus     - MDIO bus handle
 *	   phyaddr - PHY to use
 *	   regidx  - register within the PHY
 *	   regval  - data to write to register
 *
 *  Return value:
 *	   0 if ok
 ********************************************************************* */
static int umal_mii_write(struct mii_bus *bus, int phyaddr, int regidx, u16 val)
{
	int tmp = 0;

	writel((phyaddr<<8) | regidx, UMAL_MIIADDR);
	writel(val, UMAL_MIICTRL);

	tmp = readl(UMAL_MIIIDCT);
	while (tmp & UMAL_MIIIDCT_BUSY)
		tmp = readl(UMAL_MIIIDCT);

	return 0;
}

/**********************************************************************
 *  UMAL_MII_PROBE(dev)
 *
 *  Write a value to a PHY register.
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   0 if ok
 ********************************************************************* */
static int umal_mii_probe(struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);
	struct phy_device *phy_dev;
	int i;

	for (i = 0; i < PHY_MAX_ADDR; i++) {
		phy_dev = sc->mii_bus->phy_map[i];
		if (phy_dev)
			break;
	}
	if (!phy_dev) {
		printk(KERN_ERR "%s: no PHY found\n", dev->name);
		return -ENXIO;
	}

	phy_dev = phy_connect(dev, dev_name(&phy_dev->dev), &umal_miipoll, 0,
			      PHY_INTERFACE_MODE_MII);
	if (IS_ERR(phy_dev)) {
		printk(KERN_ERR "%s: could not attach to PHY\n", dev->name);
		return PTR_ERR(phy_dev);
	}

	/* Remove any features not supported by the controller */
	phy_dev->supported &= SUPPORTED_10baseT_Half |
			      SUPPORTED_10baseT_Full |
			      SUPPORTED_100baseT_Half |
			      SUPPORTED_100baseT_Full |
			      SUPPORTED_Autoneg |
			      SUPPORTED_MII;
	phy_dev->advertising = phy_dev->supported;

	pr_info("%s: attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		dev->name, phy_dev->drv->name,
		dev_name(&phy_dev->dev), phy_dev->irq);

	sc->phy_dev = phy_dev;

	return 0;
}

/**********************************************************************
 *  UMAL DMA functions
 ********************************************************************* */

/**********************************************************************
 *  UMALDMA_INITCTX(d,s,txrx,maxdescr)
 *
 *  Initialize a DMA channel context.  Since there are potentially
 *  eight DMA channels per MAC, it's nice to do this in a standard
 *  way.
 *
 *  Input parameters:
 *	   d - struct umaldma (DMA channel context)
 *	   s - struct umal_softc (pointer to a MAC)
 *	   txrx - Identifies DMA_TX or DMA_RX for channel direction
 *	   maxdescr - number of descriptors
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_initctx(struct umaldma *d, struct umal_softc *s, int rxtx,
		int maxdescr)
{
	struct umaldmadscr *dscr_item;
	int idx;

	/*
	 * Save away interesting stuff in the structure
	 */
	d->umaldma_eth   = s;
	d->umaldma_txdir = rxtx;
	d->umaldma_maxdescr = maxdescr;

	/*
	 * Allocate memory for the ring
	 */
	d->umaldma_dscrtable_unaligned = dma_alloc_coherent(NULL,
					sizeof(*d->umaldma_dscrtable),
					&d->umaldma_dscrtable_phys_unaligned,
					GFP_KERNEL | GFP_DMA);
	dma_cache_sync(NULL, d->umaldma_dscrtable_unaligned,
			sizeof(*d->umaldma_dscrtable), DMA_BIDIRECTIONAL);

	/*
	 * The descriptor table must be aligned to at least 16 bytes or the
	 * MAC will corrupt it.
	 */
	d->umaldma_dscrtable = (struct umaldmadscr *)
			ALIGN((unsigned long)d->umaldma_dscrtable_unaligned,
			sizeof(*d->umaldma_dscrtable));
	d->umaldma_dscrtable_end = d->umaldma_dscrtable + d->umaldma_maxdescr;
	d->umaldma_dscrtable_phys = ALIGN((unsigned long)
				d->umaldma_dscrtable_phys_unaligned,
				sizeof(*d->umaldma_dscrtable));

	for (idx = 0; idx < d->umaldma_maxdescr; idx++) {
		dscr_item                      = d->umaldma_dscrtable + idx;
		dscr_item->PacketStartAddr     = 0;
		dscr_item->PacketSize          = UMAL_DESC_PACKETSIZE_EMPTY;
		dscr_item->NextDescriptor      = (dma_addr_t)(
				(struct umaldmadscr *)d->umaldma_dscrtable_phys
				+ (idx+1));
		dscr_item->NextDescriptor_Virt = d->umaldma_dscrtable + (idx+1);
	}
	dscr_item                      = d->umaldma_dscrtable +
					(d->umaldma_maxdescr - 1);
	dscr_item->NextDescriptor      = d->umaldma_dscrtable_phys;
	dscr_item->NextDescriptor_Virt = d->umaldma_dscrtable;

	d->umaldma_addptr = d->umaldma_dscrtable;
	d->umaldma_remptr = d->umaldma_dscrtable;

	/*
	 * And context table
	 */
	d->umaldma_ctxtable = kcalloc(d->umaldma_maxdescr,
				    sizeof(*d->umaldma_ctxtable), GFP_KERNEL);
}

/**********************************************************************
 *  UMALDMA_UNINITCTX(d)
 *
 *  Uninitialize a DMA channel context.
 *
 *  Input parameters:
 *	   d - struct umaldma (DMA channel context)
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_uninitctx(struct umaldma *d)
{
	if (d->umaldma_dscrtable_unaligned) {
		dma_free_coherent(NULL, sizeof(*d->umaldma_dscrtable),
				d->umaldma_dscrtable_unaligned,
				d->umaldma_dscrtable_phys_unaligned);
		d->umaldma_dscrtable_unaligned = d->umaldma_dscrtable = NULL;
		d->umaldma_dscrtable_phys_unaligned = 0;
		d->umaldma_dscrtable_phys = 0;
	}

	kfree(d->umaldma_ctxtable);
	d->umaldma_ctxtable = NULL;
}

/**********************************************************************
 *  UMALDMA_CHANNEL_START(d)
 *
 *  Open a DMA channel.
 *
 *  Input parameters:
 *	   d - DMA channel to init (context must be previously init'd)
 *	   rxtx - DMA_RX or DMA_TX depending on what type of channel
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_channel_start(struct umaldma *d, int rxtx)
{
	/*
	 * Turn on the DMA channel
	 */
	if (rxtx == DMA_TX) {
		writel(d->umaldma_dscrtable_phys, UMAL_DMATxDescriptor);
		writel(UMAL_DMA_Enable, UMAL_DMATxCtrl);
	} else {
		writel(d->umaldma_dscrtable_phys, UMAL_DMARxDescriptor);
		writel(UMAL_DMA_Enable, UMAL_DMARxCtrl);
	}
}

/**********************************************************************
 *  UMALDMA_CHANNEL_STOP(d)
 *
 *  Close DMA channel.
 *
 *  Input parameters:
 *	   d - DMA channel to init (context must be previously init'd
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_channel_stop(struct umaldma *d)
{
	/*
	 * Turn off the DMA channel
	 */
	if (d->umaldma_txdir == DMA_TX) {
		writel(0, UMAL_DMATxCtrl);
		writel(0, UMAL_DMATxDescriptor);
	} else {
		writel(0, UMAL_DMARxCtrl);
		writel(0, UMAL_DMARxDescriptor);
	}

	/*
	 * Zero ring pointers
	 */
	d->umaldma_addptr = d->umaldma_dscrtable;
	d->umaldma_remptr = d->umaldma_dscrtable;
}

/**********************************************************************
 *  UMALDMA_ADD_RCVBUFFER(d,sb)
 *
 *  Add a buffer to the specified DMA channel.
 *  For receive channels, this queues a buffer for inbound packets.
 *
 *  Input parameters:
 *	   sc - softc structure
 *	    d - DMA channel descriptor
 *	   sb - sk_buff to add, or NULL if we should allocate one
 *
 *  Return value:
 *	   0 if buffer could not be added (ring is full)
 *	   1 if buffer added successfully
 ********************************************************************* */
static int umaldma_add_rcvbuffer(struct umal_softc *sc, struct umaldma *d,
		struct sk_buff *sb)
{
	struct net_device *dev = sc->umal_dev;
	struct umaldmadscr *dsc;
	struct umaldmadscr *nextdsc;
	struct sk_buff *sb_new = NULL;

	/* get pointer to our current place in the ring */
	dsc = d->umaldma_addptr;
	nextdsc = UMALDMA_NEXTBUF(d, umaldma_addptr);

	/*
	 * figure out if the ring is full - if the next descriptor
	 * is the same as the one that we're going to remove from
	 * the ring, the ring is full
	 */
	if (nextdsc == d->umaldma_remptr)
		return -ENOSPC;

	/*
	 * Allocate a sk_buff if we don't already have one.
	 * If we do have an sk_buff, reset it so that it's empty.
	 *
	 * Note: sk_buffs don't seem to be guaranteed to have any sort
	 * of alignment when they are allocated.  Therefore, allocate enough
	 * extra space to make sure that:
	 *
	 *    1. the data does not start in the middle of a cache line.
	 *    2. The data does not end in the middle of a cache line
	 *    3. The buffer can be aligned such that the IP addresses are
	 *       naturally aligned.
	 *
	 *  Remember, the SOCs MAC writes whole cache lines at a time,
	 *  without reading the old contents first.  So, if the sk_buff's
	 *  data portion starts in the middle of a cache line, the SOC
	 *  DMA will trash the beginning (and ending) portions.
	 */
	if (sb == NULL) {
		sb_new = netdev_alloc_skb(dev, ENET_PACKET_SIZE +
		       SMP_CACHE_BYTES * 2 + NET_IP_ALIGN);
		if (sb_new == NULL) {
			pr_info("%s: sk_buff allocation failed\n",
			       d->umaldma_eth->umal_dev->name);
			return -ENOBUFS;
		}
		skb_reserve(sb_new, 2);
	} else {
		sb_new = sb;
		/*
		 * nothing special to reinit buffer, it's already aligned
		 * and sb->data already points to a good place.
		 */
	}

	/* fill in the descriptor */
	dsc->PacketStartAddr = virt_to_phys(sb_new->data);
	dsc->PacketSize      = UMAL_DESC_PACKETSIZE_EMPTY;

	/* fill in the context */
	d->umaldma_ctxtable[dsc-d->umaldma_dscrtable] = sb_new;

	/* point at next packet */
	d->umaldma_addptr = nextdsc;

	return 0;
}

/**********************************************************************
 *  UMALDMA_ADD_TXBUFFER(d,sb)
 *
 *  Add a transmit buffer to the specified DMA channel, causing a
 *  transmit to start.
 *
 *  Input parameters:
 *	   d - DMA channel descriptor
 *	   sb - sk_buff to add
 *
 *  Return value:
 *	   0 transmit queued successfully
 *	   otherwise error code
 ********************************************************************* */
static int umaldma_add_txbuffer(struct umaldma *d, struct sk_buff *sb)
{
	struct umaldmadscr *dsc;
	struct umaldmadscr *nextdsc;

	/* get pointer to our current place in the ring */
	dsc = d->umaldma_addptr;
	nextdsc = UMALDMA_NEXTBUF(d, umaldma_addptr);

	/*
	 * figure out if the ring is full - if the next descriptor
	 * is the same as the one that we're going to remove from
	 * the ring, the ring is full
	 */
	if (nextdsc == d->umaldma_remptr)
		return -ENOSPC;

	/*
	 * fill in the descriptor.  Note that the number of cache
	 * blocks in the descriptor is the number of blocks
	 * *spanned*, so we need to add in the offset (if any)
	 * while doing the calculation.
	 */
	dsc->PacketStartAddr = virt_to_phys(sb->data);
	dsc->PacketSize      = sb->len | UMAL_DESC_PACKETSIZE_NONEMPTY;

	dma_map_single(NULL, sb->data, sb->len, DMA_BIDIRECTIONAL);
	dma_cache_sync(NULL, sb->data, sb->len, DMA_BIDIRECTIONAL);

	/* fill in the context */
	d->umaldma_ctxtable[dsc-d->umaldma_dscrtable] = sb;

	/* point at next packet */
	d->umaldma_addptr = nextdsc;

	return 0;
}

/**********************************************************************
 *  UMALDMA_EMPTYRING(d)
 *
 *  Free all allocated sk_buffs on the specified DMA channel;
 *
 *  Input parameters:
 *	   d  - DMA channel
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_emptyring(struct umaldma *d)
{
	int idx;
	struct sk_buff *sb;

	for (idx = 0; idx < d->umaldma_maxdescr; idx++) {
		sb = d->umaldma_ctxtable[idx];
		if (sb) {
			dev_kfree_skb(sb);
			d->umaldma_ctxtable[idx] = NULL;
		}
	}
}

/**********************************************************************
 *  UMALDMA_FILLRING(sc,d)
 *
 *  Fill the specified DMA channel (must be receive channel) with sk_buffs
 *
 *  Input parameters:
 *	   sc - softc structure
 *	    d - DMA channel
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umaldma_fillring(struct umal_softc *sc, struct umaldma *d)
{
	int idx;
	for (idx = 0; idx < UMAL_MAX_RXDESCR - 1; idx++) {
		if (umaldma_add_rcvbuffer(sc, d, NULL) != 0)
			break;
	}
}

/**********************************************************************
 *  UMALDMA_RX_PROCESS(sc,d,work_to_do,poll)
 *
 *  Process "completed" receive buffers on the specified DMA channel.
 *
 *  Input parameters:
 *            sc - softc structure
 *	       d - DMA channel context
 *    work_to_do - no. of packets to process before enabling interrupt
 *                 again (for NAPI)
 *          poll - 1: using polling (for NAPI)
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static int umaldma_rx_process(struct umal_softc *sc, struct umaldma *d,
		int work_to_do, int poll)
{
	struct net_device *dev = sc->umal_dev;
	int curidx;
	int hwidx;
	struct umaldmadscr *dsc;
	struct sk_buff *sb;
	int len;
	int work_done = 0;
	int dropped = 0;
	unsigned int int_status;

	if (!netif_device_present(dev))
		return 0;

	int_status = readl(UMAL_DMAInterrupt);

	if (int_status & INT_RX_BUS_ERR) {
		writel(CLR_RX_BUS_ERR, UMAL_DMARxStatus);
		writel(readl(UMAL_DMARxCtrl) | UMAL_DMA_Enable, UMAL_DMARxCtrl);
	}

	if (int_status & INT_RX_OVERFLOW) {
		writel(CLR_RX_OVERFLOW, UMAL_DMARxStatus);
		writel(readl(UMAL_DMARxCtrl) | UMAL_DMA_Enable, UMAL_DMARxCtrl);
	}

	if (!(int_status & INT_RX_PKT))
		return 0;

	while (work_to_do-- > 0) {
		/*
		 * figure out where we are (as an index) and where
		 * the hardware is (also as an index)
		 *
		 * This could be done faster if (for example) the
		 * descriptor table was page-aligned and contiguous in
		 * both virtual and physical memory -- you could then
		 * just compare the low-order bits of the virtual
		 * address (sbdma_remptr) and the physical address
		 * (sbdma_curdscr CSR)
		 */
		dsc = d->umaldma_remptr;
		curidx = dsc - d->umaldma_dscrtable;

		hwidx = (struct umaldmadscr *) readl(UMAL_DMARxDescriptor) -
			(struct umaldmadscr *)d->umaldma_dscrtable_phys;

		writel(CLR_RX_PKT, UMAL_DMARxStatus);

		/*
		 * If they're the same, that means we've processed all
		 * of the descriptors up to (but not including) the one
		 * that the hardware is working on right now.
		 */
		if (curidx == hwidx)
			goto done;

		/*
		 * Otherwise, get the packet's sk_buff ptr back
		 */
		sb = d->umaldma_ctxtable[curidx];
		len = dsc->PacketSize & 0xfff;
		d->umaldma_ctxtable[curidx] = NULL;

		/* .. and advance to the next buffer.  */
		d->umaldma_remptr = UMALDMA_NEXTBUF(d, umaldma_remptr);

		/*
		 * Check packet status.  If good, process it.
		 * If not, silently drop it and put it back on the
		 * receive ring.
		 */
		if (likely(!(dsc->PacketSize & UMAL_DESC_PACKETSIZE_EMPTY))) {
			/*
			 * Add a new buffer to replace the old one.
			 * If we fail to allocate a buffer, we're going
			 * to drop this packet and put it right back on
			 * the receive ring.
			 */
			if (unlikely(umaldma_add_rcvbuffer(sc, d, NULL)
					== -ENOBUFS)) {
				dev->stats.rx_dropped++;
				/* Re-add old buffer */
				umaldma_add_rcvbuffer(sc, d, sb);
				/* No point in continuing */
				printk(KERN_ERR "dropped packet (1)\n");
				d->umaldma_remptr = UMALDMA_NEXTBUF(d,
						umaldma_remptr);
				goto done;
			} else {
				/* Set length into the packet */
				skb_put(sb, len + 4);

				/*
				 * Buffer has been replaced on the
				 * receive ring.  Pass the buffer to
				 * the kernel
				 */
				sb->protocol = eth_type_trans(sb,
					d->umaldma_eth->umal_dev);
				/*
				 * Check hw IPv4/TCP checksum
				 * if supported
				 */
				skb_checksum_none_assert(sb);

				if (poll)
					dropped = netif_receive_skb(sb);
				else
					dropped = netif_rx(sb);

				if (dropped == NET_RX_DROP) {
					dev->stats.rx_dropped++;
					d->umaldma_remptr = UMALDMA_NEXTBUF(d,
							umaldma_remptr);
					goto done;
				} else {
					dev->stats.rx_bytes += len;
					dev->stats.rx_packets++;
				}
			}
		} else {
			/*
			 * Packet was mangled somehow.  Just drop it and
			 * put it back on the receive ring.
			 */
			dev->stats.rx_errors++;
			umaldma_add_rcvbuffer(sc, d, sb);
		}
		work_done++;
	}
done:
	return work_done;
}

/**********************************************************************
 *  UMALDMA_TX_PROCESS(sc,d,poll)
 *
 *  Process "completed" transmit buffers on the specified DMA channel.
 *  This is normally called within the interrupt service routine.
 *  Note that this isn't really ideal for priority channels, since
 *  it processes all of the packets on a given channel before
 *  returning.
 *
 *  Input parameters:
 *      sc - softc structure
 *	 d - DMA channel context
 *    poll - 1: using polling (for NAPI)
 *
 *  Return value:
 *	   nothing
 **********************************************************************/
static void umaldma_tx_process(struct umal_softc *sc, struct umaldma *d,
		int poll)
{
	struct net_device *dev = sc->umal_dev;
	int curidx;
	int hwidx;
	struct umaldmadscr *dsc;
	struct sk_buff *sb;
	unsigned long flags;
	int packets_handled = 0;
	unsigned int int_status;

	spin_lock_irqsave(&(sc->umal_lock), flags);

	if (!netif_device_present(dev))
		return;

	int_status = readl(UMAL_DMAInterrupt);

	if (int_status & INT_TX_BUS_ERR)
		writel(CLR_TX_BUS_ERR, UMAL_DMATxStatus);

	if (int_status & INT_TX_UNDERRUN)
		writel(CLR_TX_UNDERRUN, UMAL_DMATxStatus);

	if (int_status & INT_TX_PKT) {
		hwidx = (struct umaldmadscr *)readl(UMAL_DMATxDescriptor) -
			(struct umaldmadscr *)d->umaldma_dscrtable_phys;

		if (d->umaldma_remptr == d->umaldma_addptr)
			goto end_unlock;

		for (;;) {
			/*
			 * figure out where we are (as an index) and where
			 * the hardware is (also as an index)
			 *
			 * This could be done faster if (for example) the
			 * descriptor table was page-aligned and contiguous in
			 * both virtual and physical memory -- you could then
			 * just compare the low-order bits of the virtual
			 * address (sbdma_remptr) and the physical address
			 * (sbdma_curdscr CSR)
			 */
			curidx = d->umaldma_remptr - d->umaldma_dscrtable;

			/*
			 * If they're the same, that means we've processed all
			 * of the descriptors up to (but not including) the one
			 * that the hardware is working on right now.
			 */
			if (curidx == hwidx)
				break;

			/* Otherwise, get the packet's sk_buff ptr back */
			dsc = &(d->umaldma_dscrtable[curidx]);
			sb = d->umaldma_ctxtable[curidx];

			/* Stats */
			dev->stats.tx_bytes += sb->len;
			dev->stats.tx_packets++;

			/* for transmits, we just free buffers.  */
			dev_kfree_skb_irq(sb);
			d->umaldma_ctxtable[curidx] = NULL;
			writel(CLR_TX_PKT, UMAL_DMATxStatus);

			/* .. and advance to the next buffer.  */
			d->umaldma_remptr = UMALDMA_NEXTBUF(d, umaldma_remptr);
			packets_handled++;
		}

		/*
		 * Decide if we should wake up the protocol or not.
		 * Other drivers seem to do this when we reach a low
		 * watermark on the transmit queue.
		 */
		if (packets_handled)
			netif_wake_queue(d->umaldma_eth->umal_dev);
	}

end_unlock:
	spin_unlock_irqrestore(&(sc->umal_lock), flags);
}

/**********************************************************************
 *  UMAL Channel functions
 ********************************************************************* */

/**********************************************************************
 *  UMAL_INITCTX(s)
 *
 *  Initialize an Ethernet context structure - this is called
 *  once per MAC. Memory is allocated here, so don't
 *  call it again from inside the ioctl routines that bring the
 *  interface up/down
 *
 *  Input parameters:
 *	   s - umal context structure
 *
 *  Return value:
 *	   0
 ********************************************************************* */
static int umal_initctx(struct umal_softc *s)
{
	/*
	 * Initialize the DMA channels.
	 * Note: Only do this _once_, as it allocates memory from the kernel!
	 */
	umaldma_initctx(&(s->umal_txdma), s, DMA_TX, UMAL_MAX_TXDESCR);
	umaldma_initctx(&(s->umal_rxdma), s, DMA_RX, UMAL_MAX_RXDESCR);

	/* initial state is OFF */
	s->umal_state = umal_state_off;

	return 0;
}

/**********************************************************************
 *  UMAL_UNINITCTX(s)
 *
 *  Initialize an Ethernet context structure.
 *  Memory is allocated here, so don't call it again from inside the
 *  ioctl routines that bring the interface up/down
 *
 *  Input parameters:
 *	   s - umal context structure
 *
 *  Return value:
 *	   Nothing
 ********************************************************************* */
static void umal_uninitctx(struct umal_softc *s)
{
	umaldma_uninitctx(&(s->umal_txdma));
	umaldma_uninitctx(&(s->umal_rxdma));
}

/**********************************************************************
 *  UMAL_CHANNEL_START(s)
 *
 *  Start packet processing on this MAC.
 *
 *  Input parameters:
 *	   s - umal context structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_channel_start(struct umal_softc *s)
{
	/*
	 * Don't do this if running
	 */
	if (s->umal_state == umal_state_on)
		return;

	/* don't accept any packets, disable all interrupts */
	umaldma_channel_stop(&(s->umal_rxdma));
	umaldma_channel_stop(&(s->umal_txdma));

	writel(0, UMAL_DMAIntrMask);
	umal_clr_intr(s->umal_dev);

	/*
	 * Program the hardware address.  It goes into the hardware-address
	 * register as well as the first filter register.
	 */
	writel(s->umal_hwaddr[0]<<24 | s->umal_hwaddr[1]<<16 |
			s->umal_hwaddr[2]<<8 | s->umal_hwaddr[3], UMAL_STADDR1);
	writel(s->umal_hwaddr[4]<<24 |  s->umal_hwaddr[5]<<16, UMAL_STADDR2);

	/* Configure the speed, duplex, and flow control */
	umal_set_speed(s, s->umal_speed);
	umal_set_duplex(s, s->umal_duplex, s->umal_fc);

	/* Program multicast addresses */
	umal_setmulti(s);

	/* If channel was in promiscuous mode before, turn that on */
	if (s->umal_devflags & IFF_PROMISC)
		umal_promiscuous_mode(s, 1);

	/* Fill the receive ring */
	umaldma_fillring(s, &(s->umal_rxdma));

	umaldma_channel_start(&(s->umal_rxdma), DMA_RX);
	umaldma_channel_start(&(s->umal_txdma), DMA_TX);

	s->umal_state = umal_state_on;

	/* Initialize DMA channels (rings should be ok now) */
	writel(INT_RX_BUS_ERR | INT_RX_OVERFLOW |
			INT_RX_PKT        | INT_TX_BUS_ERR |
			INT_TX_UNDERRUN   | INT_TX_PKT |
			UMAL_DMAIntrMask_ENABLEHALFWORD, UMAL_DMAIntrMask);

	/* we're running now.  */
	writel(readl(UMAL_CFG1) | UMAL_CFG1_RXENABLE | UMAL_CFG1_TXENABLE,
			UMAL_CFG1);
}

/**********************************************************************
 *  UMAL_CHANNEL_STOP(s)
 *
 *  Stop packet processing on this MAC.
 *
 *  Input parameters:
 *	   s - umal context structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_channel_stop(struct umal_softc *s)
{
	/* don't do this if already stopped */
	if (s->umal_state == umal_state_off)
		return;

	/* don't accept any packets, disable all interrupts */
	writel(0, UMAL_DMAIntrMask);
	umal_clr_intr(s->umal_dev);

	/* turn off receiver and transmitter */
	writel(UMAL_CFG1_RESET, UMAL_CFG1);	/* reset MAC */
	writel(0, UMAL_CFG1);

	/* We're stopped now. */
	s->umal_state = umal_state_off;

	/* Stop DMA channels (rings should be ok now) */
	umaldma_channel_stop(&(s->umal_rxdma));
	umaldma_channel_stop(&(s->umal_txdma));

	/* Empty the receive and transmit rings */
	umaldma_emptyring(&(s->umal_rxdma));
	umaldma_emptyring(&(s->umal_txdma));
}

/**********************************************************************
 *  UMAL_SET_CHANNEL_STATE(s,state)
 *
 *  Set the channel's state ON or OFF
 *
 *  Input parameters:
 *	   s - umal context structure
 *	   state - new state
 *
 *  Return value:
 *	   old state
 ********************************************************************* */
static enum umal_state umal_set_channel_state(struct umal_softc *s,
		enum umal_state state)
{
	enum umal_state oldstate = s->umal_state;

	/* If same as previous state, return */
	if (state == oldstate)
		return oldstate;

	/* If new state is ON, turn channel on */
	if (state == umal_state_on)
		umal_channel_start(s);
	else
		umal_channel_stop(s);

	/* Return previous state */
	return oldstate;
}

/**********************************************************************
 *  UMAL functions
 ********************************************************************* */
static const struct net_device_ops umal_netdev_ops = {
	.ndo_open		= umal_open,
	.ndo_stop		= umal_close,
	.ndo_start_xmit		= umal_start_tx,
	.ndo_set_multicast_list	= umal_set_rx_mode,
	.ndo_tx_timeout		= umal_tx_timeout,
	.ndo_do_ioctl		= umal_mii_ioctl,
	.ndo_change_mtu		= umal_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

/**********************************************************************
 *  UMAL_INIT(dev)
 *
 *  Attach routine - init hardware and hook ourselves into linux
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   0 if ok
 ********************************************************************* */
static int umal_init(struct platform_device *pldev, long long base)
{
	struct net_device *dev = dev_get_drvdata(&pldev->dev);
	int idx = pldev->id;
	struct umal_softc *sc = netdev_priv(dev);
	unsigned char *eaddr;
	int i;
	int err;

	sc->umal_dev = dev;
	sc->umal_idx = idx;

	eaddr = sc->umal_hwaddr;

#ifdef CONFIG_CMDLINE_FORCE
	eaddr[0] = 0x00;
	eaddr[1] = 0x25;
	eaddr[2] = 0x9b;
	eaddr[3] = 0xff;
	eaddr[4] = 0x00;
	eaddr[5] = 0x00;
#endif

	for (i = 0; i < 6; i++)
		dev->dev_addr[i] = eaddr[i];

	/* Set up Linux device callins */
	spin_lock_init(&(sc->umal_lock));

	dev->netdev_ops = &umal_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->irq = IRQ_UMAL;

	sc->mii_bus = mdiobus_alloc();
	if (sc->mii_bus == NULL) {
		err = -ENOMEM;
		goto uninit_ctx;
	}

	sc->mii_bus->name = umal_mdio_string;
	snprintf(sc->mii_bus->id, MII_BUS_ID_SIZE, "%x", idx);
	sc->mii_bus->priv = sc;
	sc->mii_bus->read = umal_mii_read;
	sc->mii_bus->write = umal_mii_write;
	sc->mii_bus->reset = umal_mii_reset;
	sc->mii_bus->irq = sc->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		sc->mii_bus->irq[i] = PHY_POLL;

	sc->mii_bus->parent = &pldev->dev;

	/* Probe PHY address */
	err = mdiobus_register(sc->mii_bus);
	if (err) {
		printk(KERN_ERR "%s: unable to register MDIO bus\n",
		       dev->name);
		goto free_mdio;
	}
	dev_set_drvdata(&pldev->dev, sc->mii_bus);

	err = register_netdev(dev);
	if (err) {
		printk(KERN_ERR "%s.%d: unable to register netdev\n",
		       umal_string, idx);
		goto unreg_mdio;
	}

	pr_info("%s.%d: registered as %s\n", umal_string, idx, dev->name);

	/*
	 * Initialize context (get pointers to registers and stuff), then
	 * allocate the memory for the descriptor tables.
	 */
	dev->dev.coherent_dma_mask = 0xFFFFFFFF;
	umal_initctx(sc);

	/*
	 * Display Ethernet address (this is called during the config
	 * process so we need to finish off the config message that
	 * was being displayed)
	 */
	pr_info("%s: UMAL Ethernet at 0x%08Lx, address: %pM\n",
	       dev->name, base, eaddr);

	return 0;
unreg_mdio:
	mdiobus_unregister(sc->mii_bus);
	dev_set_drvdata(&pldev->dev, NULL);
free_mdio:
	mdiobus_free(sc->mii_bus);
uninit_ctx:
	umal_uninitctx(sc);
	return err;
}

/**********************************************************************
 *  UMAL_OPEN(dev)
 *
 *  Open umal device
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   0 if ok
 *	   otherwise error
 ********************************************************************* */
static int umal_open(struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);
	int err;

	sc->umal_speed = umal_speed_none;
	sc->umal_duplex = umal_duplex_none;
	sc->umal_fc = umal_fc_none;
	sc->umal_pause = -1;
	sc->umal_link = 0;

	/* reset mac and interface */
	writel(UMAL_CFG1_RESET, UMAL_CFG1);	/* reset MAC */
	writel(0, UMAL_CFG1);			/* clear the reset bit of MAC */
	writel(UMAL_IFCTRL_RESET, UMAL_IFCTRL);	/* reset the MAC Interface */
	writel(0, UMAL_DMAIntrMask);

	/* Attach to the PHY */
	err = umal_mii_probe(dev);
	if (err)
		goto out_unregister;

	/* Turn on the channel */
	phy_start(sc->phy_dev);

	/* config fifo */
	writel(0x000000ff, UMAL_FIFOCFG0);	/* reset FIFO */
	writel(0x0fff0fff, UMAL_FIFOCFG1);
	writel(0x0aaa0555, UMAL_FIFOCFG2);
	writel(0x02800fff, UMAL_FIFOCFG3);
	writel(0x00000070, UMAL_FIFOCFG4);
	writel(0x0007ff8f, UMAL_FIFOCFG5);
	writel(0x0000ff00, UMAL_FIFOCFG0);

	writel(0x7016, UMAL_CFG2);

	/*
	 * map/route interrupt (clear status first, in case something
	 * weird is pending; we haven't initialized the mac registers
	 * yet)
	 */
	umal_clr_intr(dev);

	err = request_irq(dev->irq, umal_intr, IRQF_SHARED, dev->name, dev);
	if (err) {
		printk(KERN_ERR "%s: unable to get IRQ %d\n", dev->name,
		       dev->irq);
		goto out_err;
	}

	umal_set_channel_state(sc, umal_state_on);

	netif_start_queue(dev);

	umal_set_rx_mode(dev);

	return 0;

out_unregister:
	free_irq(dev->irq, dev);
out_err:
	return err;
}

/**********************************************************************
 *  UMAL_CLSOE(dev)
 *
 *  Close umal device
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   0 if ok
 ********************************************************************* */
static int umal_close(struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);

	phy_stop(sc->phy_dev);

	umal_set_channel_state(sc, umal_state_off);

	netif_stop_queue(dev);

	phy_disconnect(sc->phy_dev);
	sc->phy_dev = NULL;

	free_irq(dev->irq, dev);

	umaldma_emptyring(&(sc->umal_rxdma));
	umaldma_emptyring(&(sc->umal_txdma));

	return 0;
}

/**********************************************************************
 *  UMAL_MII_IOCTL(dev,rq,cmd)
 *
 *  Umal device ioctrl routine
 *
 *  Input parameters:
 *	   dev - net_device structure
 *	   rq  - interface request structure
 *	   cmd - ioctrl command
 *
 *  Return value:
 *	   ioctrl command result
 ********************************************************************* */
static int umal_mii_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct umal_softc *sc = netdev_priv(dev);

	if (!netif_running(dev) || !sc->phy_dev)
		return -EINVAL;

	return phy_mii_ioctl(sc->phy_dev, rq, cmd);
}

/**********************************************************************
 *  UMAL_INTR(irq,dev_instance)
 *
 *  Interrupt handler for MAC interrupts
 *
 *  Input parameters:
 *	   irq - irq number
 *	   dev_instance - net_device structure
 *
 *  Return value:
 *	   irq handling result
 ********************************************************************* */
static irqreturn_t umal_intr(int irq, void *dev_instance)
{
	struct net_device *dev = (struct net_device *) dev_instance;
	struct umal_softc *sc = netdev_priv(dev);
	uint64_t isr;
	int handled = 0;

	/*
	 * Read the ISR (this clears the bits in the real
	 * register, except for counter addr)
	 */
	isr = readl(UMAL_DMAInterrupt);
	if (isr == 0)
		return IRQ_RETVAL(0);

	if (sc->umal_state != umal_state_on) {
		umal_clr_intr(dev);
		return IRQ_RETVAL(0);
	}
	handled = 1;

	/* Transmits on channel 0 */
	if (isr & INT_TX_MASK)
		umaldma_tx_process(sc, &(sc->umal_txdma), 0);
	if (isr & INT_RX_MASK)
		umaldma_rx_process(sc, &(sc->umal_rxdma),
				 UMAL_MAX_RXDESCR * 2, 0);

	return IRQ_RETVAL(handled);
}

/**********************************************************************
 *  UMAL_CLR_INTR(dev)
 *
 *  Clear all interrupt of umal
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_clr_intr(struct net_device *dev)
{
	unsigned int int_status;

	if (!netif_device_present(dev))
		return;
	int_status = readl(UMAL_DMAInterrupt);
	if (int_status & INT_RX_PKT)
		writel(CLR_RX_PKT, UMAL_DMARxStatus);

	if (int_status & INT_RX_BUS_ERR)
		writel(CLR_RX_BUS_ERR, UMAL_DMARxStatus);

	if (int_status & INT_RX_OVERFLOW)
		writel(CLR_RX_OVERFLOW, UMAL_DMARxStatus);

	if (int_status & INT_TX_PKT)
		writel(CLR_TX_PKT, UMAL_DMATxStatus);

	if (int_status & INT_TX_BUS_ERR)
		writel(CLR_TX_BUS_ERR, UMAL_DMATxStatus);

	if (int_status & INT_TX_UNDERRUN)
		writel(CLR_TX_UNDERRUN, UMAL_DMATxStatus);
}

/**********************************************************************
 *  UMAL_START_TX(skb,dev)
 *
 *  Start output on the specified interface.  Basically, we
 *  queue as many buffers as we can until the ring fills up, or
 *  we run off the end of the queue, whichever comes first.
 *
 *  Input parameters:
 *	   skb - sk_buff structure
 *	   dev - net_device structure
 *
 *  Return value:
 *	   0 if ok
 *	   otherwise error
 ********************************************************************* */
static int umal_start_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);
	unsigned long flags;

	/* lock eth irq */
	spin_lock_irqsave(&sc->umal_lock, flags);

	/*
	 * Put the buffer on the transmit ring.  If we
	 * don't have room, stop the queue.
	 */
	if (umaldma_add_txbuffer(&(sc->umal_txdma), skb)) {
		/* XXX: save skb that we could not send */
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&sc->umal_lock, flags);

		return NETDEV_TX_BUSY;
	}

	writel(readl(UMAL_CFG1) | UMAL_CFG1_TXENABLE, UMAL_CFG1);
	writel(readl(UMAL_DMATxCtrl) | UMAL_DMA_Enable, UMAL_DMATxCtrl);

	spin_unlock_irqrestore(&sc->umal_lock, flags);

	return 0;
}

/**********************************************************************
 *  UMAL_TX_TIMEOUT(dev)
 *
 *  Tx timeout, update statistic structure
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_tx_timeout(struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&sc->umal_lock, flags);

	dev->trans_start = jiffies; /* prevent tx timeout */
	dev->stats.tx_errors++;

	spin_unlock_irqrestore(&sc->umal_lock, flags);

	printk(KERN_WARNING "%s: Transmit timed out\n", dev->name);
}

/**********************************************************************
 *  UMAL_SET_RX_MODE(dev)
 *
 *  Set promiscuous mode and multicast list
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_set_rx_mode(struct net_device *dev)
{
	unsigned long flags;
	struct umal_softc *sc = netdev_priv(dev);

	spin_lock_irqsave(&sc->umal_lock, flags);
	if ((dev->flags ^ sc->umal_devflags) & IFF_PROMISC) {
		/* Promiscuous changed.  */
		if (dev->flags & IFF_PROMISC)
			umal_promiscuous_mode(sc, 1);
		else
			umal_promiscuous_mode(sc, 0);
	}
	spin_unlock_irqrestore(&sc->umal_lock, flags);

	/* Program the multicasts.  Do this every time.  */
	umal_setmulti(sc);
}

/**********************************************************************
 *  UMAL_PROMISCUOUS_MODE(sc,onoff)
 *
 *  Turn on or off promiscuous mode
 *
 *  Input parameters:
 *	   sc - softc
 *      onoff - 1 to turn on, 0 to turn off
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_promiscuous_mode(struct umal_softc *sc, int onoff)
{
	if (onoff) {
		writel(readl(UMAL_FIFOCFG4) & ~0x40000, UMAL_FIFOCFG4);
		writel(readl(UMAL_FIFOCFG5) | 0x40000, UMAL_FIFOCFG5);
	} else {
		writel(readl(UMAL_FIFOCFG4) | 0x40000, UMAL_FIFOCFG4);
		writel(readl(UMAL_FIFOCFG5) & ~0x40000, UMAL_FIFOCFG5);
	}
}

/**********************************************************************
 *  UMAL_SETMULTI(sc)
 *
 *  Reprogram the multicast table into the hardware, given
 *  the list of multicasts associated with the interface
 *  structure.
 *
 *  Input parameters:
 *	   sc - softc
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_setmulti(struct umal_softc *sc)
{
}

/**********************************************************************
 *  UMAL_SET_SPEED(s,speed)
 *
 *  Configure LAN speed for the specified MAC.
 *  Warning: must be called when MAC is off!
 *
 *  Input parameters:
 *	   s - sbmac structure
 *	   speed - speed to set MAC to (see enum sbmac_speed)
 *
 *  Return value:
 *	   1 if successful
 *	   0 indicates invalid parameters
 ********************************************************************* */
static int umal_set_speed(struct umal_softc *s, enum umal_speed speed)
{
	unsigned int cfg;

	/* Save new current values */
	s->umal_speed = speed;

	if (s->umal_state == umal_state_on)
		return 0;	/* save for next restart */

	/* Read current register values */
	cfg = readl(UMAL_CFG2);

	/* Mask out the stuff we want to change */
	cfg &= ~(UMAL_CFG2_MODEMASK);

	/* Now add in the new bits */
	switch (speed) {
	case umal_speed_10:
	case umal_speed_100:
		cfg |= UMAL_CFG2_NIBBLEMODE;
		break;

	default:
		return 0;
	}

	/* Send the bits back to the hardware */
	writel(cfg, UMAL_CFG2);

	return 1;
}

/**********************************************************************
 *  UMAL_SET_DUPLEX(s,duplex,fc)
 *
 *  Set Ethernet duplex and flow control options for this MAC
 *  Warning: must be called when MAC is off!
 *
 *  Input parameters:
 *	   s - umal structure
 *	   duplex - duplex setting (see enum sbmac_duplex)
 *	   fc - flow control setting (see enum sbmac_fc)
 *
 *  Return value:
 *	   1 if ok
 *	   0 if an invalid parameter combination was specified
 ********************************************************************* */
static int umal_set_duplex(struct umal_softc *s, enum umal_duplex duplex,
		enum umal_fc fc)
{
	unsigned int cfg1, cfg2;
	int err = 0;

	/* Save new current values */
	s->umal_duplex = duplex;
	s->umal_fc = fc;

	if (s->umal_state == umal_state_on)
		return 0;	/* save for next restart */

	/* Read current register values */
	cfg1 = readl(UMAL_CFG1);
	cfg2 = readl(UMAL_CFG2);

	/* Mask off the stuff we're about to change */
	cfg1 &= ~(UMAL_CFG1_TXFLOWCTL | UMAL_CFG1_RXFLOWCTL);
	cfg2 &= ~(UMAL_CFG2_FULLDUPLEX);

	err = 0;
	switch (duplex) {
	case umal_duplex_half:
		break;

	case umal_duplex_full:
		cfg2 |= UMAL_CFG2_FULLDUPLEX;
		break;

	default:
		err = 1;
	}
	if (!err)
		writel(cfg2, UMAL_CFG2);

	err = 0;
	switch (fc) {
	case umal_fc_disabled:
		break;

	case umal_fc_collision:
		break;

	case umal_fc_carrier:
		break;

	case umal_fc_frame:
		cfg1 |= UMAL_CFG1_TXFLOWCTL | UMAL_CFG1_RXFLOWCTL;
		break;

	default:
		err = 1;
	}

	if (!err)
		writel(cfg1, UMAL_CFG1);

	/* Send the bits back to the hardware */
	return 1;
}

/**********************************************************************
 *  UMAL_CHANGE_MTU(dev,new_mtu)
 *
 *  Change MTU value
 *
 *  Input parameters:
 *	   dev - net_device structure
 *	   new_mtu - new mtu value
 *
 *  Return value:
 *	   1 if ok
 ********************************************************************* */
static int umal_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu > ENET_PACKET_SIZE)
		return -EINVAL;

	dev->mtu = new_mtu;

	pr_info("changing the mtu to %d\n", new_mtu);

	return 0;
}

/**********************************************************************
 *  UMAL_MIIPOLL(dev)
 *
 *  Phy statemachine call back routine for link change
 *
 *  Input parameters:
 *	   dev - net_device structure
 *
 *  Return value:
 *	   nothing
 ********************************************************************* */
static void umal_miipoll(struct net_device *dev)
{
	struct umal_softc *sc = netdev_priv(dev);
	struct phy_device *phy_dev = sc->phy_dev;
	unsigned long flags;
	enum umal_fc fc;
	int link_chg, speed_chg, duplex_chg, pause_chg, fc_chg;

	link_chg = (sc->umal_link != phy_dev->link);
	speed_chg = (sc->umal_speed != phy_dev->speed);
	duplex_chg = (sc->umal_duplex != phy_dev->duplex);
	pause_chg = (sc->umal_pause != phy_dev->pause);

	if (!link_chg && !speed_chg && !duplex_chg && !pause_chg)
		return;

	if (!phy_dev->link) {
		if (link_chg) {
			sc->umal_link = phy_dev->link;
			sc->umal_speed = umal_speed_none;
			sc->umal_duplex = umal_duplex_none;
			sc->umal_fc = umal_fc_disabled;
			sc->umal_pause = -1;
			pr_info("%s: link unavailable\n", dev->name);
		}
		return;
	}

	if (phy_dev->duplex == DUPLEX_FULL) {
		if (phy_dev->pause)
			fc = umal_fc_frame;
		else
			fc = umal_fc_disabled;
	} else
		fc = umal_fc_collision;
	fc_chg = (sc->umal_fc != fc);

	pr_info("%s: link available: %dbase-%cD\n", dev->name, phy_dev->speed,
		phy_dev->duplex == DUPLEX_FULL ? 'F' : 'H');

	spin_lock_irqsave(&sc->umal_lock, flags);

	sc->umal_speed = phy_dev->speed;
	sc->umal_duplex = phy_dev->duplex;
	sc->umal_fc = fc;
	sc->umal_pause = phy_dev->pause;
	sc->umal_link = phy_dev->link;

	if ((speed_chg || duplex_chg || fc_chg) &&
		sc->umal_state != umal_state_off) {
		/* something changed, restart the channel */
		umal_channel_stop(sc);
		umal_channel_start(sc);
	}

	spin_unlock_irqrestore(&sc->umal_lock, flags);
}

static int umal_probe(struct platform_device *pldev)
{
	struct net_device *dev;
	struct umal_softc *sc;
	struct resource *res;
	int err;

	res = platform_get_resource(pldev, IORESOURCE_MEM, 0);
	BUG_ON(!res);

	/* Okay.  Initialize this MAC.  */
	dev = alloc_etherdev(sizeof(struct umal_softc));
	if (!dev) {
		printk(KERN_ERR "%s: unable to allocate etherdev\n",
		       dev_name(&pldev->dev));
		err = -ENOMEM;
		goto out_out;
	}

	dev_set_drvdata(&pldev->dev, dev);
	SET_NETDEV_DEV(dev, &pldev->dev);

	sc = netdev_priv(dev);

	err = umal_init(pldev, res->start);
	if (err)
		goto out_kfree;

	return 0;

out_kfree:
	free_netdev(dev);

out_out:
	return err;
}

static int __exit umal_remove(struct platform_device *pldev)
{
	struct net_device *dev = dev_get_drvdata(&pldev->dev);
	struct umal_softc *sc = netdev_priv(dev);

	unregister_netdev(dev);
	umal_uninitctx(sc);
	mdiobus_unregister(sc->mii_bus);
	free_netdev(dev);

	return 0;
}

#if CONFIG_PM
static void umal_reset(struct net_device *ndev)
{
	writel(UMAL_CFG1_RESET, UMAL_CFG1);	/* reset MAC */
	writel(0, UMAL_CFG1);			/* clear the reset bit of MAC */
	writel(UMAL_IFCTRL_RESET, UMAL_IFCTRL); /* reset the MAC Interface */
	writel(1, UMAL_DMAIntrMask);

	writel(0x000000ff, UMAL_FIFOCFG0);	/* reset FIFO */
	writel(0x0fff0fff, UMAL_FIFOCFG1);
	writel(0x0aaa0555, UMAL_FIFOCFG2);
	writel(0x02800fff, UMAL_FIFOCFG3);
	writel(0x00000070, UMAL_FIFOCFG4);
	writel(0x0007ff8f, UMAL_FIFOCFG5);
	writel(0x0000ff00, UMAL_FIFOCFG0);

	writel(0x7016, UMAL_CFG2);
}

static void umal_shutdown(struct net_device *ndev)
{
	/* XXX: something should be cleared */
	return;
}

static int umal_suspend(struct platform_device *pldev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pldev);

	if (netif_running(ndev)) {
		netif_device_detach(ndev);
		umal_shutdown(ndev);
	}
	return 0;
}

static int umal_resume(struct platform_device *pldev)
{
	struct net_device *dev = platform_get_drvdata(pldev);

	if (netif_running(dev)) {
		umal_reset(dev);
		netif_device_attach(dev);
	}
	return 0;
}
#else
static int umal_suspend(struct platform_device *pldev, pm_message_t state) { }
static int umal_resume(struct platform_device *pldev) { }
#endif
static struct platform_driver umal_driver = {
	.probe = umal_probe,
	.remove = __exit_p(umal_remove),
	.driver = {
		.name = umal_string,
		.owner  = THIS_MODULE,
	},
	.suspend = umal_suspend,
	.resume	 = umal_resume,
};

static int __init umal_init_module(void)
{
	return platform_driver_register(&umal_driver);
}

static void __exit umal_cleanup_module(void)
{
	platform_driver_unregister(&umal_driver);
}

module_init(umal_init_module);
module_exit(umal_cleanup_module);
