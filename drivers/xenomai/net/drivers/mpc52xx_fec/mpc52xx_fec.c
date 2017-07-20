/*
 * arch/ppc/5xxx_io/fec.c
 *
 * Driver for the MPC5200 Fast Ethernet Controller
 * Support for MPC5100 FEC has been removed, contact the author if you need it
 *
 * Author: Dale Farnsworth <dfarnsworth@mvista.com>
 *
 * 2003 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 *
 * Ported to RTnet from "linuxppc_2_4_devel/arch/ppc/5xxx_io/fec.c".
 * Copyright (c) 2008 Wolfgang Grandegger <wg@denx.de>
 */

/* #define PARANOID_CHECKS*/
/* #define MUST_ALIGN_TRANSMIT_DATA*/
#define MUST_UNALIGN_RECEIVE_DATA
/* #define EXIT_ISR_AT_MEMORY_SQUEEZE*/
/* #define DISPLAY_WARNINGS*/

#ifdef ORIGINAL_CODE
static const char *version = "fec.c v0.2\n";
#endif /* ORIGINAL_CODE */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/crc32.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <asm/delay.h>
#include <rtnet_port.h>
#include "rt_mpc52xx_fec.h"
#ifdef CONFIG_UBOOT
#include <asm/ppcboot.h>
#endif

#ifdef CONFIG_XENO_DRIVERS_NET_FASTROUTE
#error "Fast Routing on MPC5200 ethernet not supported"
#endif

MODULE_AUTHOR("Maintainer: Wolfgang Grandegger <wg@denx.de>");
MODULE_DESCRIPTION("RTnet driver for MPC52xx FEC");
MODULE_LICENSE("GPL");

static unsigned int rx_pool_size =  0;
MODULE_PARM(rx_pool_size, "i");
MODULE_PARM_DESC(rx_pool_size, "Receive buffer pool size");

#define printk(fmt,args...)	rtdm_printk (fmt ,##args)

static struct rtnet_device *mpc5xxx_fec_dev;
static int mpc5xxx_fec_interrupt(rtdm_irq_t *irq_handle);
static int mpc5xxx_fec_receive_interrupt(rtdm_irq_t *irq_handle);
static int mpc5xxx_fec_transmit_interrupt(rtdm_irq_t *irq_handle);
static struct net_device_stats *mpc5xxx_fec_get_stats(struct rtnet_device *dev);
#ifdef ORIGINAL_CODE
static void mpc5xxx_fec_set_multicast_list(struct rtnet_device *dev);
#endif /* ORIGINAL_CODE */
static void mpc5xxx_fec_reinit(struct rtnet_device* dev);
static int mpc5xxx_fec_setup(struct rtnet_device *dev, int reinit);
static int mpc5xxx_fec_cleanup(struct rtnet_device *dev, int reinit);

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
static void mpc5xxx_fec_mii(struct rtnet_device *dev);
#ifdef ORIGINAL_CODE
static int mpc5xxx_fec_ioctl(struct rtnet_device *, struct ifreq *rq, int cmd);
static int mpc5xxx_netdev_ethtool_ioctl(struct rtnet_device *dev, void *useraddr);
#endif /* ORIGINAL_CODE */
static void mdio_timer_callback(unsigned long data);
static void mii_display_status(struct rtnet_device *dev);
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET
static void mpc5xxx_mdio_callback(uint regval, struct rtnet_device *dev, uint data);
static int mpc5xxx_mdio_read(struct rtnet_device *dev, int phy_id, int location);
#endif

static void mpc5xxx_fec_update_stat(struct rtnet_device *);

/* MII processing.  We keep this as simple as possible.  Requests are
 * placed on the list (if there is room).  When the request is finished
 * by the MII, an optional function may be called.
 */
typedef struct mii_list {
	uint    mii_regval;
	void    (*mii_func)(uint val, struct rtnet_device *dev, uint data);
	struct  mii_list *mii_next;
	uint    mii_data;
} mii_list_t;

#define		NMII	20
mii_list_t      mii_cmds[NMII];
mii_list_t      *mii_free;
mii_list_t      *mii_head;
mii_list_t      *mii_tail;

typedef struct mdio_read_data {
	u16 regval;
	struct task_struct *sleeping_task;
} mdio_read_data_t;

static int mii_queue(struct rtnet_device *dev, int request,
		void (*func)(uint, struct rtnet_device *, uint), uint data);

/* Make MII read/write commands for the FEC.
 * */
#define mk_mii_read(REG)	(0x60020000 | ((REG & 0x1f) << 18))
#define mk_mii_write(REG, VAL)	(0x50020000 | ((REG & 0x1f) << 18) | \
							(VAL & 0xffff))
#define mk_mii_end	0

/* Register definitions for the PHY.
*/

#define MII_REG_CR	 0	/* Control Register */
#define MII_REG_SR	 1	/* Status Register */
#define MII_REG_PHYIR1	 2	/* PHY Identification Register 1 */
#define MII_REG_PHYIR2	 3	/* PHY Identification Register 2 */
#define MII_REG_ANAR	 4	/* A-N Advertisement Register */
#define MII_REG_ANLPAR	 5	/* A-N Link Partner Ability Register */
#define MII_REG_ANER	 6	/* A-N Expansion Register */
#define MII_REG_ANNPTR	 7	/* A-N Next Page Transmit Register */
#define MII_REG_ANLPRNPR 8	/* A-N Link Partner Received Next Page Reg. */

/* values for phy_status */

#define PHY_CONF_ANE	0x0001	/* 1 auto-negotiation enabled */
#define PHY_CONF_LOOP	0x0002	/* 1 loopback mode enabled */
#define PHY_CONF_SPMASK	0x00f0	/* mask for speed */
#define PHY_CONF_10HDX	0x0010	/* 10 Mbit half duplex supported */
#define PHY_CONF_10FDX	0x0020	/* 10 Mbit full duplex supported */
#define PHY_CONF_100HDX	0x0040	/* 100 Mbit half duplex supported */
#define PHY_CONF_100FDX	0x0080	/* 100 Mbit full duplex supported */

#define PHY_STAT_LINK	0x0100	/* 1 up - 0 down */
#define PHY_STAT_FAULT	0x0200	/* 1 remote fault */
#define PHY_STAT_ANC	0x0400	/* 1 auto-negotiation complete	*/
#define PHY_STAT_SPMASK	0xf000	/* mask for speed */
#define PHY_STAT_10HDX	0x1000	/* 10 Mbit half duplex selected	*/
#define PHY_STAT_10FDX	0x2000	/* 10 Mbit full duplex selected	*/
#define PHY_STAT_100HDX	0x4000	/* 100 Mbit half duplex selected */
#define PHY_STAT_100FDX	0x8000	/* 100 Mbit full duplex selected */

#endif	/* CONFIG_XENO_DRIVERS_NET_USE_MDIO */

u8 mpc5xxx_fec_mac_addr[6];
u8 null_mac[6];

#ifdef ORIGINAL_CODE
static void mpc5xxx_fec_tx_timeout(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;

	priv->stats.tx_errors++;

	if (!priv->tx_full)
		rtnetif_wake_queue(dev);
}
#endif /* ORIGINAL_CODE */

static void
mpc5xxx_fec_set_paddr(struct rtnet_device *dev, u8 *mac)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;

	out_be32(&fec->paddr1, (mac[0]<<24) | (mac[1]<<16)
			| (mac[2]<<8) | (mac[3]<<0));
	out_be32(&fec->paddr2, (mac[4]<<24) | (mac[5]<<16) | 0x8808);
}

#ifdef ORIGINAL_CODE
static int
mpc5xxx_fec_set_mac_address(struct rtnet_device *dev, void *addr)
{
	struct sockaddr *sock = (struct sockaddr *)addr;

	mpc5xxx_fec_set_paddr(dev, sock->sa_data);
	return 0;
}
#endif /* ORIGINAL_CODE */

/* This function is called to start or restart the FEC during a link
 * change.  This happens on fifo errors or when switching between half
 * and full duplex.
 */
static void
mpc5xxx_fec_restart(struct rtnet_device *dev, int duplex)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	u32 rcntrl;
	u32 tcntrl;
	int i;

#if MPC5xxx_FEC_DEBUG > 1
	printk("mpc5xxx_fec_restart\n");
#endif
	out_be32(&fec->rfifo_status, in_be32(&fec->rfifo_status) & 0x700000);
	out_be32(&fec->tfifo_status, in_be32(&fec->tfifo_status) & 0x700000);
	out_be32(&fec->reset_cntrl, 0x1000000);

	/* Whack a reset.  We should wait for this. */
	out_be32(&fec->ecntrl, MPC5xxx_FEC_ECNTRL_RESET);
	for (i = 0; i < MPC5xxx_FEC_RESET_DELAY; ++i) {
		if ((in_be32(&fec->ecntrl) & MPC5xxx_FEC_ECNTRL_RESET) == 0)
			break;
		udelay(1);
	}
	if (i == MPC5xxx_FEC_RESET_DELAY)
		printk ("FEC Reset timeout!\n");

	/* Set station address. */
	out_be32(&fec->paddr1, *(u32 *)&dev->dev_addr[0]);
	out_be32(&fec->paddr2,
		((*(u16 *)&dev->dev_addr[4]) << 16) | 0x8808);

#ifdef ORIGINAL_CODE
	mpc5xxx_fec_set_multicast_list(dev);
#endif /* ORIGINAL_CODE */

	rcntrl = MPC5xxx_FEC_RECV_BUFFER_SIZE << 16;	/* max frame length */
	rcntrl |= MPC5xxx_FEC_RCNTRL_FCE;
#ifdef	CONFIG_XENO_DRIVERS_NET_USE_MDIO
	rcntrl |= MPC5xxx_FEC_RCNTRL_MII_MODE;
#endif
	if (duplex)
		tcntrl = MPC5xxx_FEC_TCNTRL_FDEN;		/* FD enable */
	else {
		rcntrl |= MPC5xxx_FEC_RCNTRL_DRT;
		tcntrl = 0;
	}
	out_be32(&fec->r_cntrl, rcntrl);
	out_be32(&fec->x_cntrl, tcntrl);

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
	/* Set MII speed. */
	out_be32(&fec->mii_speed, priv->phy_speed);
#endif

	priv->full_duplex = duplex;
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
	priv->duplex_change = 0;
#endif
#if MPC5xxx_FEC_DEBUG > 4
	printk("%s: duplex set to %d\n", dev->name, priv->full_duplex);
#endif

	/* Clear any outstanding interrupt. */
	out_be32(&fec->ievent, 0xffffffff);	/* clear intr events */

	/* Enable interrupts we wish to service.
	*/
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
	out_be32(&fec->imask, 0xf0fe0000);	/* enable all intr but tfint */
#else
	out_be32(&fec->imask, 0xf07e0000);	/* enable all intr but tfint */
#endif

	/* And last, enable the transmit and receive processing.
	*/
	out_be32(&fec->ecntrl, MPC5xxx_FEC_ECNTRL_ETHER_EN);
	out_be32(&fec->r_des_active, 0x01000000);

	/* The tx ring is no longer full. */
	if (priv->tx_full)
	{
		priv->tx_full = 0;
		rtnetif_wake_queue(dev);
	}
}

#ifdef	CONFIG_XENO_DRIVERS_NET_USE_MDIO
static void
mpc5xxx_fec_mii(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	mii_list_t	*mip;
	uint		mii_reg;

	mii_reg = in_be32(&fec->mii_data);

	if ((mip = mii_head) == NULL) {
		printk("MII and no head!\n");
		return;
	}
#if MPC5xxx_FEC_DEBUG > 4
	printk("mpc5xxx_fec_mii %08x %08x %08x\n",
		mii_reg, (u32)mip->mii_func, mip->mii_data);
#endif

	if (mip->mii_func != NULL)
		(*(mip->mii_func))(mii_reg, dev, mip->mii_data);

	mii_head = mip->mii_next;
	mip->mii_next = mii_free;
	mii_free = mip;

	if ((mip = mii_head) != NULL)
		out_be32(&fec->mii_data, mip->mii_regval);
}

static int
mii_queue(struct rtnet_device *dev, int regval, void (*func)(uint, struct rtnet_device *, uint), uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	rtdm_lockctx_t	context;
	mii_list_t	*mip;
	int		retval;

#if MPC5xxx_FEC_DEBUG > 4
	printk("mii_queue: %08x %08x %08x\n", regval, (u32)func, data);
#endif

	/* Add PHY address to register command.
	*/
	regval |= priv->phy_addr << 23;

	retval = 0;

	rtdm_lock_get_irqsave(&priv->lock, context);

	if ((mip = mii_free) != NULL) {
		mii_free = mip->mii_next;
		mip->mii_regval = regval;
		mip->mii_func = func;
		mip->mii_next = NULL;
		mip->mii_data = data;
		if (mii_head) {
			mii_tail->mii_next = mip;
			mii_tail = mip;
		} else {
			mii_head = mii_tail = mip;
			out_be32(&fec->mii_data, regval);
		}
	} else
		retval = 1;

	rtdm_lock_put_irqrestore(&priv->lock, context);

	return retval;
}

static void mii_do_cmd(struct rtnet_device *dev, const phy_cmd_t *c)
{
	int k;

	if (!c)
		return;

	for (k = 0; (c+k)->mii_data != mk_mii_end; k++)
		mii_queue(dev, (c+k)->mii_data, (c+k)->funct, 0);
}

static void mii_parse_sr(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	s &= ~(PHY_STAT_LINK | PHY_STAT_FAULT | PHY_STAT_ANC);

	if (mii_reg & 0x0004)
		s |= PHY_STAT_LINK;
	if (mii_reg & 0x0010)
		s |= PHY_STAT_FAULT;
	if (mii_reg & 0x0020)
		s |= PHY_STAT_ANC;

	priv->phy_status = s;
	priv->link = (s & PHY_STAT_LINK) ? 1 : 0;
}

static void mii_parse_cr(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	s &= ~(PHY_CONF_ANE | PHY_CONF_LOOP);

	if (mii_reg & 0x1000)
		s |= PHY_CONF_ANE;
	if (mii_reg & 0x4000)
		s |= PHY_CONF_LOOP;

	priv->phy_status = s;
}

static void mii_parse_anar(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	s &= ~(PHY_CONF_SPMASK);

	if (mii_reg & 0x0020)
		s |= PHY_CONF_10HDX;
	if (mii_reg & 0x0040)
		s |= PHY_CONF_10FDX;
	if (mii_reg & 0x0080)
		s |= PHY_CONF_100HDX;
	if (mii_reg & 0x0100)
		s |= PHY_CONF_100FDX;

	priv->phy_status = s;
}

/* ------------------------------------------------------------------------- */
/* Generic PHY support.  Should work for all PHYs, but does not support link
 * change interrupts.
 */
#ifdef CONFIG_XENO_DRIVERS_NET_FEC_GENERIC_PHY

static phy_info_t phy_info_generic = {
	0x00000000, /* 0-->match any PHY */
	"GENERIC",

	(const phy_cmd_t []) {  /* config */
		/* advertise only half-duplex capabilities */
		{ mk_mii_write(MII_ADVERTISE, MII_ADVERTISE_HALF),
			mii_parse_anar },

		/* enable auto-negotiation */
		{ mk_mii_write(MII_BMCR, BMCR_ANENABLE), mii_parse_cr },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {  /* startup */
		/* restart auto-negotiation */
		{ mk_mii_write(MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART)),
			NULL },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) { /* ack_int */
		/* We don't actually use the ack_int table with a generic
		 * PHY, but putting a reference to mii_parse_sr here keeps
		 * us from getting a compiler warning about unused static
		 * functions in the case where we only compile in generic
		 * PHY support.
		 */
		{ mk_mii_read(MII_BMSR), mii_parse_sr },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {  /* shutdown */
		{ mk_mii_end, }
	},
};
#endif	/* CONFIG_XENO_DRIVERS_NET_FEC_GENERIC_PHY */

/* ------------------------------------------------------------------------- */
/* The Level one LXT971 is used on some of my custom boards		     */

#ifdef CONFIG_XENO_DRIVERS_NET_FEC_LXT971

/* register definitions for the 971 */

#define MII_LXT971_PCR	16	/* Port Control Register	*/
#define MII_LXT971_SR2	17	/* Status Register 2		*/
#define MII_LXT971_IER	18	/* Interrupt Enable Register	*/
#define MII_LXT971_ISR	19	/* Interrupt Status Register	*/
#define MII_LXT971_LCR	20	/* LED Control Register		*/
#define MII_LXT971_TCR	30	/* Transmit Control Register	*/

static void mii_parse_lxt971_sr2(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	s &= ~(PHY_STAT_SPMASK);

	if (mii_reg & 0x4000) {
		if (mii_reg & 0x0200)
			s |= PHY_STAT_100FDX;
		else
			s |= PHY_STAT_100HDX;
	}
	else {
		if (mii_reg & 0x0200)
			s |= PHY_STAT_10FDX;
		else
			s |= PHY_STAT_10HDX;
	}
	if (mii_reg & 0x0008)
		s |= PHY_STAT_FAULT;

	/* Record the new full_duplex value only if the link is up
	 * (so we don't bother restarting the driver on duplex
	 * changes when the link is down).
	 */
	if (priv->link) {
		int prev_duplex = priv->full_duplex;
		priv->full_duplex = ((mii_reg & 0x0200) != 0);
		if (priv->full_duplex != prev_duplex) {
			/* trigger a restart with changed duplex */
			priv->duplex_change = 1;
#if MPC5xxx_FEC_DEBUG > 1
			printk("%s: duplex change: %s\n",
			       dev->name, priv->full_duplex ? "full" : "half");
#endif
		}
	}
	priv->phy_status = s;
}

static phy_info_t phy_info_lxt971 = {
	0x0001378e,
	"LXT971",

	(const phy_cmd_t []) {	/* config */
#ifdef MPC5100_FIX10HDX
		{ mk_mii_write(MII_REG_ANAR, 0x021), NULL }, /* 10 Mbps, HD */
#else
/*		{ mk_mii_write(MII_REG_ANAR, 0x0A1), NULL }, *//*  10/100, HD */
		{ mk_mii_write(MII_REG_ANAR, 0x01E1), NULL }, /* 10/100, FD */
#endif
		{ mk_mii_read(MII_REG_CR), mii_parse_cr },
		{ mk_mii_read(MII_REG_ANAR), mii_parse_anar },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {	/* startup - enable interrupts */
		{ mk_mii_write(MII_LXT971_IER, 0x00f2), NULL },
		{ mk_mii_write(MII_REG_CR, 0x1200), NULL }, /* autonegotiate */

		/* Somehow does the 971 tell me that the link is down
		 * the first read after power-up.
		 * read here to get a valid value in ack_int */

		{ mk_mii_read(MII_REG_SR), mii_parse_sr },
#if defined(CONFIG_UC101)
		{ mk_mii_write(MII_LXT971_LCR, 0x4122), NULL }, /* LED settings */
#endif
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) { /* ack_int */
		/* find out the current status */

		{ mk_mii_read(MII_REG_SR), mii_parse_sr },
		{ mk_mii_read(MII_LXT971_SR2), mii_parse_lxt971_sr2 },

		/* we only need to read ISR to acknowledge */

		{ mk_mii_read(MII_LXT971_ISR), NULL },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {	/* shutdown - disable interrupts */
		{ mk_mii_write(MII_LXT971_IER, 0x0000), NULL },
		{ mk_mii_end, }
	},
};

#endif /* CONFIG_XENO_DRIVERS_NET_FEC_LXT971 */

/* ----------------------------------------------------------------- */
/* The National Semiconductor DP83847 is used on a INKA 4X0 board    */
/* ----------------------------------------------------------------- */

#ifdef CONFIG_XENO_DRIVERS_NET_FEC_DP83847

/* Register definitions */
#define MII_DP83847_PHYSTS 0x10  /* PHY Status Register */

static void mii_parse_dp83847_physts(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	s &= ~(PHY_STAT_SPMASK);

	if (mii_reg & 0x2) {
		if (mii_reg & 0x4)
			s |= PHY_STAT_10FDX;
		else
			s |= PHY_STAT_10HDX;
	}
	else {
		if (mii_reg & 0x4)
			s |= PHY_STAT_100FDX;
		else
			s |= PHY_STAT_100HDX;
	}
	if (mii_reg & 0x40)
		s |= PHY_STAT_FAULT;

	priv->full_duplex = ((mii_reg & 0x4) != 0);

	priv->phy_status = s;
}

static phy_info_t phy_info_dp83847 = {
	0x020005c3,
	"DP83847",

	(const phy_cmd_t []) {  /* config */
		{ mk_mii_write(MII_REG_ANAR, 0x01E1), NULL  }, /* Auto-Negociation Register Control set to    */
							       /* auto-negociate 10/100MBps, Half/Full duplex */
		{ mk_mii_read(MII_REG_CR),   mii_parse_cr   },
		{ mk_mii_read(MII_REG_ANAR), mii_parse_anar },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {  /* startup */
		{ mk_mii_write(MII_REG_CR, 0x1200), NULL }, /* Enable and Restart Auto-Negotiation */
		{ mk_mii_read(MII_REG_SR), mii_parse_sr },
		{ mk_mii_read(MII_REG_CR), mii_parse_cr   },
		{ mk_mii_read(MII_DP83847_PHYSTS), mii_parse_dp83847_physts },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) { /* ack_int */
		{ mk_mii_read(MII_REG_SR), mii_parse_sr },
		{ mk_mii_read(MII_REG_CR), mii_parse_cr   },
		{ mk_mii_read(MII_DP83847_PHYSTS), mii_parse_dp83847_physts },
		{ mk_mii_end, }
	},
	(const phy_cmd_t []) {  /* shutdown - disable interrupts */
		{ mk_mii_end, }
	}
};

#endif /* CONFIG_XENO_DRIVERS_NET_FEC_DP83847 */

static phy_info_t *phy_info[] = {

#ifdef CONFIG_XENO_DRIVERS_NET_FEC_LXT971
	&phy_info_lxt971,
#endif /* CONFIG_XENO_DRIVERS_NET_FEC_LXT971 */

#ifdef CONFIG_XENO_DRIVERS_NET_FEC_DP83847
	&phy_info_dp83847,
#endif /* CONFIG_XENO_DRIVERS_NET_FEC_DP83847 */

#ifdef CONFIG_XENO_DRIVERS_NET_FEC_GENERIC_PHY
	/* Generic PHY support.  This must be the last PHY in the table.
	 * It will be used to support any PHY that doesn't match a previous
	 * entry in the table.
	 */
	&phy_info_generic,
#endif /* CONFIG_XENO_DRIVERS_NET_FEC_GENERIC_PHY */

	NULL
};

static void mii_display_config(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint s = priv->phy_status;

	printk("%s: config: auto-negotiation ", dev->name);

	if (s & PHY_CONF_ANE)
		printk("on");
	else
		printk("off");

	if (s & PHY_CONF_100FDX)
		printk(", 100FDX");
	if (s & PHY_CONF_100HDX)
		printk(", 100HDX");
	if (s & PHY_CONF_10FDX)
		printk(", 10FDX");
	if (s & PHY_CONF_10HDX)
		printk(", 10HDX");
	if (!(s & PHY_CONF_SPMASK))
		printk(", No speed/duplex selected?");

	if (s & PHY_CONF_LOOP)
		printk(", loopback enabled");

	printk(".\n");

	priv->sequence_done = 1;
}

static void mii_queue_config(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;

	priv->phy_task.routine = (void *)mii_display_config;
	priv->phy_task.data = dev;
	schedule_task(&priv->phy_task);
}


phy_cmd_t phy_cmd_config[] = { { mk_mii_read(MII_REG_CR), mii_queue_config },
			       { mk_mii_end, } };


/* Read remainder of PHY ID.
*/
static void
mii_discover_phy3(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	int	i;

	priv->phy_id |= (mii_reg & 0xffff);

	for (i = 0; phy_info[i]; i++) {
		if (phy_info[i]->id == (priv->phy_id >> 4) || !phy_info[i]->id)
			break;
		if (phy_info[i]->id == 0)	/* check generic entry */
			break;
	}

	if (!phy_info[i])
		panic("%s: PHY id 0x%08x is not supported!\n",
			dev->name, priv->phy_id);

	priv->phy = phy_info[i];
	priv->phy_id_done = 1;

	printk("%s: Phy @ 0x%x, type %s (0x%08x)\n",
		dev->name, priv->phy_addr, priv->phy->name, priv->phy_id);
#if defined(CONFIG_UC101)
	mii_do_cmd(dev, priv->phy->startup);
#endif
}

/* Scan all of the MII PHY addresses looking for someone to respond
 * with a valid ID.  This usually happens quickly.
 */
static void
mii_discover_phy(uint mii_reg, struct rtnet_device *dev, uint data)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	uint	phytype;

#if MPC5xxx_FEC_DEBUG > 4
	printk("mii_discover_phy\n");
#endif

	if ((phytype = (mii_reg & 0xffff)) != 0xffff) {
		/* Got first part of ID, now get remainder.
		*/
		priv->phy_id = phytype << 16;
		mii_queue(dev, mk_mii_read(MII_REG_PHYIR2), mii_discover_phy3, 0);
	} else {
		priv->phy_addr++;
		if (priv->phy_addr < 32)
			mii_queue(dev, mk_mii_read(MII_REG_PHYIR1),
							mii_discover_phy, 0);
		else
			printk("fec: No PHY device found.\n");
	}
}

static void
mpc5xxx_fec_link_up(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)(dev->priv);

	printk("mpc5xxx_fec_link_up: link_up=%d\n", priv->link_up);
#ifdef ORIGINAL_CODE
	priv->link_up = 0;
#endif /* ORIGINAL_CODE */
	mii_display_status(dev);
	if (priv->duplex_change) {
#if MPC5xxx_FEC_DEBUG > 1
		printk("%s: restarting with %s duplex...\n",
		       dev->name, priv->full_duplex ? "full" : "half");
#endif
		mpc5xxx_fec_restart(dev, priv->full_duplex);
	}
}

/*
 * Execute the ack_int command set and schedules next timer call back.
 */
static void mdio_timer_callback(unsigned long data)
{
	struct rtnet_device *dev = (struct rtnet_device *)data;
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)(dev->priv);
	mii_do_cmd(dev, priv->phy->ack_int);

	if (priv->link_up) {
#ifdef ORIGINAL_CODE
		priv->link_up_task.routine = (void *)mpc5xxx_fec_link_up;
		priv->link_up_task.data = dev;
		schedule_task(&priv->link_up_task);
#else
		mpc5xxx_fec_link_up(dev);
		return;
#endif /* ORIGINAL_CODE */
	}
	/* Reschedule in 1 second */
	priv->phy_timer_list.expires = jiffies + (1000 * HZ / 1000);
	add_timer(&priv->phy_timer_list);
}

/*
 * Displays the current status of the PHY.
 */
static void mii_display_status(struct rtnet_device *dev)
{
    struct mpc5xxx_fec_priv *priv = dev->priv;
    uint s = priv->phy_status;

    printk("%s: status: ", dev->name);

    if (!priv->link) {
	printk("link down");
    } else {
	printk("link up");

	switch(s & PHY_STAT_SPMASK) {
	case PHY_STAT_100FDX: printk(", 100 Mbps Full Duplex"); break;
	case PHY_STAT_100HDX: printk(", 100 Mbps Half Duplex"); break;
	case PHY_STAT_10FDX:  printk(", 10 Mbps Full Duplex");  break;
	case PHY_STAT_10HDX:  printk(", 10 Mbps Half Duplex");  break;
	default:
	    printk(", Unknown speed/duplex");
	}

	if (s & PHY_STAT_ANC)
	    printk(", auto-negotiation complete");
    }

    if (s & PHY_STAT_FAULT)
	printk(", remote fault");

    printk(".\n");
}
#endif	/* CONFIG_XENO_DRIVERS_NET_USE_MDIO */


#define RFIFO_DATA	0xf0003184
#define TFIFO_DATA	0xf00031a4

/*
 * Initialize FEC receive task.
 * Returns task number of FEC receive task.
 * Returns -1 on failure
 */
int
mpc5xxx_fec_rx_task_setup(int num_bufs, int maxbufsize)
{
	static TaskSetupParamSet_t params;
	int tasknum;

	params.NumBD = num_bufs;
	params.Size.MaxBuf = maxbufsize;
	params.StartAddrSrc = RFIFO_DATA;
	params.IncrSrc = 0;
	params.SzSrc = 4;
	params.IncrDst = 4;
	params.SzDst = 4;

	tasknum = TaskSetup(TASK_FEC_RX, &params);

	/* clear pending interrupt bits */
	TaskIntClear(tasknum);

	return tasknum;
}

/*
 * Initialize FEC transmit task.
 * Returns task number of FEC transmit task.
 * Returns -1 on failure
 */
int
mpc5xxx_fec_tx_task_setup(int num_bufs)
{
	static TaskSetupParamSet_t params;
	int tasknum;

	params.NumBD = num_bufs;
	params.IncrSrc = 4;
	params.SzSrc = 4;
	params.StartAddrDst = TFIFO_DATA;
	params.IncrDst = 0;
	params.SzDst = 4;

	tasknum = TaskSetup(TASK_FEC_TX, &params);

	/* clear pending interrupt bits */
	TaskIntClear(tasknum);

	return tasknum;
}



#ifdef PARANOID_CHECKS
static volatile int tx_fifo_cnt, tx_fifo_ipos, tx_fifo_opos;
static volatile int rx_fifo_opos;
#endif

static struct rtskb *tx_fifo_skb[MPC5xxx_FEC_TBD_NUM];
static struct rtskb *rx_fifo_skb[MPC5xxx_FEC_RBD_NUM];
static BDIdx mpc5xxx_bdi_tx = 0;


static int
mpc5xxx_fec_setup(struct rtnet_device *dev, int reinit)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_xlb *xlb = (struct mpc5xxx_xlb *)MPC5xxx_XLB;
	struct rtskb *skb;
	int i;
	struct mpc5xxx_rbuf *rbuf;
	struct mpc5xxx_fec *fec = priv->fec;
	u32 u32_value;
	u16 u16_value;

#if MPC5xxx_FEC_DEBUG > 1
	printk("mpc5xxx_fec_setup\n");
#endif

	mpc5xxx_fec_set_paddr(dev, dev->dev_addr);

	/*
	 * Initialize receive queue
	 */
	priv->r_tasknum = mpc5xxx_fec_rx_task_setup(MPC5xxx_FEC_RBD_NUM,
						    MPC5xxx_FEC_RECV_BUFFER_SIZE_BC);
	TaskBDReset(priv->r_tasknum);
	for(i=0;i<MPC5xxx_FEC_RBD_NUM;i++) {
		BDIdx bdi_a;
		if(!reinit) {
			skb = dev_alloc_rtskb(sizeof *rbuf, dev);
			if (skb == 0)
				goto eagain;
#ifdef MUST_UNALIGN_RECEIVE_DATA
			rtskb_reserve(skb,2);
#endif
			rbuf = (struct mpc5xxx_rbuf *)rtskb_put(skb, sizeof *rbuf);
			rx_fifo_skb[i]=skb;
		}
		else {
			skb=rx_fifo_skb[i];
			rbuf = (struct mpc5xxx_rbuf *)skb->data;
		}
		bdi_a = TaskBDAssign(priv->r_tasknum,
					(void*)virt_to_phys((void *)&rbuf->data),
					0, sizeof *rbuf, MPC5xxx_FEC_RBD_INIT);
		if(bdi_a<0)
			panic("mpc5xxx_fec_setup: error while TaskBDAssign, err=%i\n",(int)bdi_a);
	}
#ifdef PARANOID_CHECKS
	rx_fifo_opos = 0;
#endif

	/*
	 * Initialize transmit queue
	 */
	if(!reinit) {
		priv->t_tasknum = mpc5xxx_fec_tx_task_setup(MPC5xxx_FEC_TBD_NUM);
		TaskBDReset(priv->t_tasknum);
		mpc5xxx_bdi_tx = 0;
		for(i=0;i<MPC5xxx_FEC_TBD_NUM;i++) tx_fifo_skb[i]=0;
#ifdef PARANOID_CHECKS
		tx_fifo_cnt = tx_fifo_ipos = tx_fifo_opos = 0;
#endif

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
		if (reinit) {
			if (!priv->sequence_done) {
				if (!priv->phy) {
					printk("mpc5xxx_fec_setup: PHY not configured\n");
					return -ENODEV; /* No PHY we understand */
				}

				mii_do_cmd(dev, priv->phy->config);
				mii_do_cmd(dev, phy_cmd_config); /* display configuration */
				while(!priv->sequence_done)
					schedule();

				mii_do_cmd(dev, priv->phy->startup);
			}
		}
#endif /* CONFIG_XENO_DRIVERS_NET_USE_MDIO */

		dev->irq = MPC5xxx_FEC_IRQ;
		priv->r_irq = MPC5xxx_SDMA_IRQ_BASE + priv->r_tasknum;
		priv->t_irq = MPC5xxx_SDMA_IRQ_BASE + priv->t_tasknum;

		if ((i = rtdm_irq_request(&priv->irq_handle, dev->irq,
					  mpc5xxx_fec_interrupt, 0,
					  "rteth_err", dev))) {
			printk(KERN_ERR "FEC interrupt allocation failed\n");
			return i;
		}

		if ((i = rtdm_irq_request(&priv->r_irq_handle, priv->r_irq,
					  mpc5xxx_fec_receive_interrupt, 0,
					  "rteth_recv", dev))) {
			printk(KERN_ERR "FEC receive task interrupt allocation failed\n");
			return i;
		}

		if ((i = rtdm_irq_request(&priv->t_irq_handle, priv->t_irq,
					  mpc5xxx_fec_transmit_interrupt, 0,
					  "rteth_xmit", dev))) {
			printk(KERN_ERR "FEC transmit task interrupt allocation failed\n");
			return i;
		}

		rt_stack_connect(dev, &STACK_manager);

		u32_value = in_be32(&priv->gpio->port_config);
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
		u32_value |= 0x00050000;	/* 100MBit with MD	*/
#else
		u32_value |= 0x00020000;	/* 10MBit with 7-wire	*/
#endif
		out_be32(&priv->gpio->port_config, u32_value);

	}

	out_be32(&fec->op_pause, 0x00010020);	/* change to 0xffff0020 ??? */
	out_be32(&fec->rfifo_cntrl, 0x0f240000);
	out_be32(&fec->rfifo_alarm, 0x0000030c);
	out_be32(&fec->tfifo_cntrl, 0x0f240000);
	out_be32(&fec->tfifo_alarm, 0x00000100);
	out_be32(&fec->x_wmrk, 0x3);		/* xmit fifo watermark = 256 */
	out_be32(&fec->xmit_fsm, 0x03000000);	/* enable crc generation */
	out_be32(&fec->iaddr1, 0x00000000);	/* No individual filter */
	out_be32(&fec->iaddr2, 0x00000000);	/* No individual filter */

#ifdef CONFIG_MPC5200
	/* Disable COMM Bus Prefetch */
	u16_value = in_be16(&priv->sdma->PtdCntrl);
	u16_value |= 1;
	out_be16(&priv->sdma->PtdCntrl, u16_value);

	/* Disable (or enable?) BestComm XLB address snooping */
	out_be32(&xlb->config, in_be32(&xlb->config) | MPC5200B_XLB_CONF_BSDIS);
#endif

	if(!reinit) {
#if !defined(CONFIG_XENO_DRIVERS_NET_USE_MDIO)
		mpc5xxx_fec_restart (dev, 0);	/* always use half duplex mode only */
#else
#ifdef CONFIG_UBOOT
		extern unsigned char __res[];
		bd_t *bd = (bd_t *)__res;
#define MPC5xxx_IPBFREQ bd->bi_ipbfreq
#else
#define MPC5xxx_IPBFREQ CONFIG_PPC_5xxx_IPBFREQ
#endif

		for (i=0; i<NMII-1; i++)
			mii_cmds[i].mii_next = &mii_cmds[i+1];
		mii_free = mii_cmds;

		priv->phy_speed = (((MPC5xxx_IPBFREQ >> 20) / 5) << 1);

		/*mpc5xxx_fec_restart (dev, 0);*/ /* half duplex, negotiate speed */
		mpc5xxx_fec_restart (dev, 1);	/* full duplex, negotiate speed */

		/* Queue up command to detect the PHY and initialize the
		 * remainder of the interface.
		 */
		priv->phy_id_done = 0;
		priv->phy_addr = 0;
		mii_queue(dev, mk_mii_read(MII_REG_PHYIR1), mii_discover_phy, 0);

		priv->old_status = 0;

		/*
		 * Read MIB counters in order to reset them,
		 * then zero all the stats fields in memory
		 */
		mpc5xxx_fec_update_stat(dev);

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
		if (reinit) {
			if (!priv->sequence_done) {
				if (!priv->phy) {
					printk("mpc5xxx_fec_open: PHY not configured\n");
					return -ENODEV;		/* No PHY we understand */
				}

				mii_do_cmd(dev, priv->phy->config);
				mii_do_cmd(dev, phy_cmd_config);  /* display configuration */
				while(!priv->sequence_done)
					schedule();

				mii_do_cmd(dev, priv->phy->startup);

				/*
				 * Currently, MII link interrupts are not supported,
				 * so start the 100 msec timer to monitor the link up event.
				 */
				init_timer(&priv->phy_timer_list);

				priv->phy_timer_list.expires = jiffies + (100 * HZ / 1000);
				priv->phy_timer_list.data = (unsigned long)dev;
				priv->phy_timer_list.function = mdio_timer_callback;
				add_timer(&priv->phy_timer_list);

				printk("%s: Waiting for the link to be up...\n", dev->name);
				while (priv->link == 0) {
					schedule();
				}
				mii_display_status(dev);
				if (priv->full_duplex == 0) { /* FD is not negotiated, restart the fec in HD */
					mpc5xxx_fec_restart(dev, 0);
				}
			}
		}
#endif /* CONFIG_XENO_DRIVERS_NET_USE_MDIO */
#endif
	}
	else {
		mpc5xxx_fec_restart (dev, 0);
	}

	rtnetif_start_queue(dev);

	TaskStart(priv->r_tasknum, TASK_AUTOSTART_ENABLE,
		  priv->r_tasknum, TASK_INTERRUPT_ENABLE);

	if(reinit) {
		TaskStart(priv->t_tasknum, TASK_AUTOSTART_ENABLE,
			  priv->t_tasknum, TASK_INTERRUPT_ENABLE);
	}

	return 0;

eagain:
	printk("mpc5xxx_fec_setup: failed\n");
	for (i=0; i<MPC5xxx_FEC_RBD_NUM; i++) {
		skb = rx_fifo_skb[i];
		if (skb == 0)
			break;
		dev_kfree_rtskb(skb);
	}
	TaskBDReset(priv->r_tasknum);

	return -EAGAIN;
}

static int
mpc5xxx_fec_open(struct rtnet_device *dev)
{
	return mpc5xxx_fec_setup(dev,0);
}

/* This will only be invoked if your driver is _not_ in XOFF state.
 * What this means is that you need not check it, and that this
 * invariant will hold if you make sure that the netif_*_queue()
 * calls are done at the proper times.
 */
static int
mpc5xxx_fec_hard_start_xmit(struct rtskb *skb, struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	rtdm_lockctx_t context;
	int pad;
	short length;
	BDIdx bdi_a;

#if MPC5xxx_FEC_DEBUG > 4
	printk("mpc5xxx_fec_hard_start_xmit:\n");
	printk("dev %08x, priv %08x, skb %08x\n",
			(u32)dev, (u32)priv, (u32)skb);
#endif
#if MPC5xxx_FEC_DEBUG > 0
	if (fec_start_status(&priv->t_queue) & MPC5xxx_FEC_TBD_TFD)
		panic("MPC5xxx transmit queue overrun\n");
#endif

	length = skb->len;
#ifdef	MUST_ALIGN_TRANSMIT_DATA
	pad = (int)skb->data & 3;
	if (pad) {
		void *old_data = skb->data;
		rtskb_push(skb, pad);
		memcpy(skb->data, old_data, length);
		rtskb_trim(skb, length);
	}
#endif
	/* Zero out up to the minimum length ethernet packet size,
	 * so we don't inadvertently expose sensitive data
	 */
	pad = ETH_ZLEN - skb->len;
	if (pad > 0) {
		skb = rtskb_padto(skb, ETH_ZLEN);
		if (skb == 0) {
			printk("rtskb_padto failed\n");
			return 0;
		}
		length += pad;
	}

	flush_dcache_range((u32)skb->data, (u32)skb->data + length);

	rtdm_lock_get_irqsave(&priv->lock, context);

	bdi_a = TaskBDAssign(priv->t_tasknum,(void*)virt_to_phys((void *)skb->data),
			     NULL,length,MPC5xxx_FEC_TBD_INIT);

#ifdef PARANOID_CHECKS
	/* check for other errors during assignment*/
	if((bdi_a<0)||(bdi_a>=MPC5xxx_FEC_TBD_NUM))
		panic("mpc5xxx_fec_hard_start_xmit: error while TaskBDAssign, err=%i\n",(int)bdi_a);

	/* sanity check: bdi must always equal tx_fifo_ipos*/
	if(bdi_a!=tx_fifo_ipos)
		panic("bdi_a!=tx_fifo_ipos: %i, %i\n",(int)bdi_a,tx_fifo_ipos);

	tx_fifo_cnt++;
	tx_fifo_ipos++;
	if(tx_fifo_ipos==MPC5xxx_FEC_TBD_NUM) tx_fifo_ipos=0;

	/* check number of BDs in use*/
	if(TaskBDInUse(priv->t_tasknum)!=tx_fifo_cnt)
		panic("TaskBDInUse != tx_fifo_cnt: %i %i\n",TaskBDInUse(priv->t_tasknum),tx_fifo_cnt);
#endif

	tx_fifo_skb[bdi_a]=skb;

#ifdef ORIGINAL_CODE
	dev->trans_start = jiffies;
#endif /* ORIGINAL_CODE */

	/* Get and patch time stamp just before the transmission */
	if (skb->xmit_stamp)
		*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() + *skb->xmit_stamp);

	TaskStart(priv->t_tasknum, TASK_AUTOSTART_ENABLE, priv->t_tasknum, TASK_INTERRUPT_ENABLE);

	if(TaskBDInUse(priv->t_tasknum)==MPC5xxx_FEC_TBD_NUM) {
		priv->tx_full = 1;
		rtnetif_stop_queue(dev);
	}
	rtdm_lock_put_irqrestore(&priv->lock, context);

	return 0;
}

/* This handles SDMA transmit task interrupts
 */
static int
mpc5xxx_fec_transmit_interrupt(rtdm_irq_t *irq_handle)
{
	struct rtnet_device *dev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	BDIdx bdi_r;

	rtdm_lock_get(&priv->lock);

	while(TaskBDInUse(priv->t_tasknum)) {

		/* relase BD*/
		bdi_r = TaskBDRelease(priv->t_tasknum);

		/* we are done if we can't release any more BDs*/
		if(bdi_r==TASK_ERR_BD_BUSY) break;
		/* if(bdi_r<0) break;*/

#ifdef PARANOID_CHECKS
		/* check for other errors during release*/
		if((bdi_r<0)||(bdi_r>=MPC5xxx_FEC_TBD_NUM))
			panic("mpc5xxx_fec_transmit_interrupt: error while TaskBDRelease, err=%i\n",(int)bdi_r);

		tx_fifo_cnt--;
		tx_fifo_opos++;
		if(tx_fifo_opos==MPC5xxx_FEC_TBD_NUM) tx_fifo_opos=0;

		/* sanity check: bdi_r must always equal tx_fifo_opos*/
		if(bdi_r!=tx_fifo_opos) {
			panic("bdi_r!=tx_fifo_opos: %i, %i\n",(int)bdi_r,tx_fifo_opos);
		}

		/* check number of BDs in use*/
		if(TaskBDInUse(priv->t_tasknum)!=tx_fifo_cnt)
			panic("TaskBDInUse != tx_fifo_cnt: %i %i\n",TaskBDInUse(priv->t_tasknum),tx_fifo_cnt);
#endif

		if((tx_fifo_skb[mpc5xxx_bdi_tx])==0)
			panic("skb confusion in tx\n");

		dev_kfree_rtskb(tx_fifo_skb[mpc5xxx_bdi_tx]);
		tx_fifo_skb[mpc5xxx_bdi_tx]=0;

		mpc5xxx_bdi_tx = bdi_r;

		if(TaskBDInUse(priv->t_tasknum)<MPC5xxx_FEC_TBD_NUM/2)
			priv->tx_full = 0;

	}

	if (rtnetif_queue_stopped(dev) && !priv->tx_full)
		rtnetif_wake_queue(dev);

	rtdm_lock_put(&priv->lock);

	return RTDM_IRQ_HANDLED;
}

static BDIdx mpc5xxx_bdi_rx = 0;

static int
mpc5xxx_fec_receive_interrupt(rtdm_irq_t *irq_handle)
{
	struct rtnet_device *dev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct rtskb *skb;
	struct rtskb *nskb;
	struct mpc5xxx_rbuf *rbuf;
	struct mpc5xxx_rbuf *nrbuf;
	u32 status;
	int length;
	BDIdx bdi_a, bdi_r;
	int discard = 0;
	int dropped = 0;
	int packets = 0;
	nanosecs_abs_t time_stamp = rtdm_clock_read();

	while(1) {

		/* release BD*/
		bdi_r = TaskBDRelease(priv->r_tasknum);

		/* we are done if we can't release any more BDs*/
		if(bdi_r==TASK_ERR_BD_BUSY) break;

#ifdef PARANOID_CHECKS
		/* check for other errors during release*/
		if((bdi_r<0)||(bdi_r>=MPC5xxx_FEC_RBD_NUM))
			panic("mpc5xxx_fec_receive_interrupt: error while TaskBDRelease, err=%i\n",(int)bdi_r);

		rx_fifo_opos++;
		if(rx_fifo_opos==MPC5xxx_FEC_RBD_NUM) rx_fifo_opos=0;

		if(bdi_r != rx_fifo_opos)
			panic("bdi_r != rx_fifo_opos: %i, %i\n",bdi_r, rx_fifo_opos);
#endif

		/* get BD status in order to determine length*/
		status = TaskGetBD(priv->r_tasknum,mpc5xxx_bdi_rx)->Status;

		/* determine packet length and pointer to socket buffer / actual data*/
		skb = rx_fifo_skb[mpc5xxx_bdi_rx];
		length = (status & 0xffff) - 4;
		rbuf = (struct mpc5xxx_rbuf *)skb->data;

#ifndef EXIT_ISR_AT_MEMORY_SQUEEZE
		/* in case of a memory squeeze, we just drop all packets, because*/
		/* subsequent allocations will also fail.*/
		if(discard!=3) {
#endif

			/* check for frame errors*/
			if(status&0x00370000) {
				/* frame error, drop */
#ifdef DISPLAY_WARNINGS
				if(status&MPC5xxx_FEC_FRAME_LG)
					printk("%s: Frame length error, dropping packet (status=0x%08x)\n",dev->name,status);
				if(status&MPC5xxx_FEC_FRAME_NO)
					printk("%s: Non-octet aligned frame error, dropping packet (status=0x%08x)\n",dev->name,status);
				if(status&MPC5xxx_FEC_FRAME_CR)
					printk("%s: Frame CRC error, dropping packet (status=0x%08x)\n",dev->name,status);
				if(status&MPC5xxx_FEC_FRAME_OV)
					printk("%s: FIFO overrun error, dropping packet (status=0x%08x)\n",dev->name,status);
				if(status&MPC5xxx_FEC_FRAME_TR)
					printk("%s: Frame truncated error, dropping packet (status=0x%08x)\n",dev->name,status);
#endif
				discard=1;
			}
			else if (length>(MPC5xxx_FEC_RECV_BUFFER_SIZE-4)) {
				/* packet too big, drop */
#ifdef DISPLAY_WARNINGS
				printk("%s: Frame too big, dropping packet (length=%i)\n",dev->name,length);
#endif
				discard=2;
			}
			else {
				/* allocate replacement skb */
				nskb = dev_alloc_rtskb(sizeof *nrbuf, dev);
				if (nskb == NULL) {
					/* memory squeeze, drop */
					discard=3;
					dropped++;
				}
				else {
					discard=0;
				}
			}

#ifndef EXIT_ISR_AT_MEMORY_SQUEEZE
		}
		else {
			dropped++;
		}
#endif

		if (discard) {
			priv->stats.rx_dropped++;
			nrbuf = (struct mpc5xxx_rbuf *)skb->data;
		}
		else {
#ifdef MUST_UNALIGN_RECEIVE_DATA
			rtskb_reserve(nskb,2);
#endif
			nrbuf = (struct mpc5xxx_rbuf *)rtskb_put(nskb, sizeof *nrbuf);

			/* only invalidate the number of bytes in dcache actually received*/
#ifdef MUST_UNALIGN_RECEIVE_DATA
			invalidate_dcache_range((u32)rbuf - 2, (u32)rbuf + length);
#else
			invalidate_dcache_range((u32)rbuf, (u32)rbuf + length);
#endif
			rtskb_trim(skb, length);
			skb->protocol = rt_eth_type_trans(skb, dev);
			skb->time_stamp = time_stamp;
			rtnetif_rx(skb);
			packets++;
#ifdef ORIGINAL_CODE
			dev->last_rx = jiffies;
#endif /* ORIGINAL_CODE */
			rx_fifo_skb[mpc5xxx_bdi_rx] = nskb;
		}

		/* Assign new socket buffer to BD*/
		bdi_a = TaskBDAssign(priv->r_tasknum, (void*)virt_to_phys((void *)&nrbuf->data),
				     0, sizeof *nrbuf, MPC5xxx_FEC_RBD_INIT);

#ifdef PARANOID_CHECKS
		/* check for errors during assignment*/
		if((bdi_a<0)||(bdi_r>=MPC5xxx_FEC_RBD_NUM))
			panic("mpc5xxx_fec_receive_interrupt: error while TaskBDAssign, err=%i\n",(int)bdi_a);

		/* check if Assign/Release sequence numbers are ok*/
		if(((bdi_a+1)%MPC5xxx_FEC_RBD_NUM) != bdi_r)
			panic("bdi_a+1 != bdi_r: %i %i\n",(int)((bdi_a+1)%MPC5xxx_FEC_RBD_NUM),(int)bdi_r);
#endif

		mpc5xxx_bdi_rx = bdi_r;

#ifdef EXIT_ISR_AT_MEMORY_SQUEEZE
		/* if we couldn't get memory for a new socket buffer, then it doesn't*/
		/* make sense to proceed.*/
		if (discard==3)
			break;
#endif

	}

#ifdef DISPLAY_WARNINGS
	if(dropped) {
		printk("%s: Memory squeeze, dropped %i packets\n",dev->name,dropped);
	}
#endif
	TaskStart(priv->r_tasknum, TASK_AUTOSTART_ENABLE, priv->r_tasknum, TASK_INTERRUPT_ENABLE);

	if (packets > 0)
		rt_mark_stack_mgr(dev);
	return RTDM_IRQ_HANDLED;
}


static void
mpc5xxx_fec_reinit(struct rtnet_device *dev)
{
	int retval;
	printk("mpc5xxx_fec_reinit\n");
	mpc5xxx_fec_cleanup(dev,1);
	retval=mpc5xxx_fec_setup(dev,1);
	if(retval) panic("reinit failed\n");
}


static int
mpc5xxx_fec_interrupt(rtdm_irq_t *irq_handle)
{
	struct rtnet_device *dev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	int ievent;

#if MPC5xxx_FEC_DEBUG > 4
	printk("mpc5xxx_fec_interrupt:\n");
#endif

	ievent = in_be32(&fec->ievent);
	out_be32(&fec->ievent, ievent);		/* clear pending events */

	if (ievent & (MPC5xxx_FEC_IEVENT_RFIFO_ERROR |
		      MPC5xxx_FEC_IEVENT_XFIFO_ERROR)) {
		if (ievent & MPC5xxx_FEC_IEVENT_RFIFO_ERROR)
			printk(KERN_WARNING "MPC5xxx_FEC_IEVENT_RFIFO_ERROR\n");
		if (ievent & MPC5xxx_FEC_IEVENT_XFIFO_ERROR)
			printk(KERN_WARNING "MPC5xxx_FEC_IEVENT_XFIFO_ERROR\n");
		mpc5xxx_fec_reinit(dev);
	}
	else if (ievent & MPC5xxx_FEC_IEVENT_MII) {
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
		mpc5xxx_fec_mii(dev);
#else
		printk("%s[%d] %s: unexpected MPC5xxx_FEC_IEVENT_MII\n",
			__FILE__, __LINE__, __FUNCTION__);
#endif /* CONFIG_XENO_DRIVERS_NET_USE_MDIO */
	}

	return RTDM_IRQ_HANDLED;
}

static int
mpc5xxx_fec_cleanup(struct rtnet_device *dev, int reinit)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	unsigned long timeout;
	int i;

	priv->open_time = 0;
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO
	priv->sequence_done = 0;
#endif

	rtnetif_stop_queue(dev);

	/* Wait for rx queue to drain */
	if(!reinit) {
		timeout = jiffies + 2*HZ;
		while (TaskBDInUse(priv->t_tasknum) && (jiffies < timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10);
		}
	}

	/* Disable FEC interrupts */
	out_be32(&fec->imask, 0x0);

	/* Stop FEC */
	out_be32(&fec->ecntrl, in_be32(&fec->ecntrl) & ~0x2);

	/* Disable the rx and tx queues. */
	TaskStop(priv->r_tasknum);
	TaskStop(priv->t_tasknum);

	/* Release irqs */
	if(!reinit) {
		rtdm_irq_disable(&priv->irq_handle);
		rtdm_irq_disable(&priv->r_irq_handle);
		rtdm_irq_disable(&priv->t_irq_handle);
		rtdm_irq_free(&priv->irq_handle);
		rtdm_irq_free(&priv->r_irq_handle);
		rtdm_irq_free(&priv->t_irq_handle);
		rt_stack_disconnect(dev);
	}

	/* Free rx Buffers */
	if(!reinit) {
		for (i=0; i<MPC5xxx_FEC_RBD_NUM; i++) {
			dev_kfree_rtskb(rx_fifo_skb[i]);
		}
	}

	mpc5xxx_fec_get_stats(dev);

	return 0;
}

static int
mpc5xxx_fec_close(struct rtnet_device *dev)
{
	int ret = mpc5xxx_fec_cleanup(dev,0);
	return ret;
}

/*
 * Get the current statistics.
 * This may be called with the card open or closed.
 */
static struct net_device_stats *mpc5xxx_fec_get_stats(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct net_device_stats *stats = &priv->stats;
	struct mpc5xxx_fec *fec = priv->fec;

	stats->rx_bytes = in_be32(&fec->rmon_r_octets);
	stats->rx_packets = in_be32(&fec->rmon_r_packets);
	stats->rx_errors = stats->rx_packets - (
					in_be32(&fec->ieee_r_frame_ok) +
					in_be32(&fec->rmon_r_mc_pkt));
	stats->tx_bytes = in_be32(&fec->rmon_t_octets);
	stats->tx_packets = in_be32(&fec->rmon_t_packets);
	stats->tx_errors = stats->tx_packets - (
					in_be32(&fec->ieee_t_frame_ok) +
					in_be32(&fec->rmon_t_col) +
					in_be32(&fec->ieee_t_1col) +
					in_be32(&fec->ieee_t_mcol) +
					in_be32(&fec->ieee_t_def));
	stats->multicast = in_be32(&fec->rmon_r_mc_pkt);
	stats->collisions = in_be32(&fec->rmon_t_col);

	/* detailed rx_errors: */
	stats->rx_length_errors = in_be32(&fec->rmon_r_undersize)
			+ in_be32(&fec->rmon_r_oversize)
			+ in_be32(&fec->rmon_r_frag)
			+ in_be32(&fec->rmon_r_jab);
	stats->rx_over_errors = in_be32(&fec->r_macerr);
	stats->rx_crc_errors = in_be32(&fec->ieee_r_crc);
	stats->rx_frame_errors = in_be32(&fec->ieee_r_align);
	stats->rx_fifo_errors = in_be32(&fec->rmon_r_drop);
	stats->rx_missed_errors = in_be32(&fec->rmon_r_drop);

	/* detailed tx_errors: */
	stats->tx_aborted_errors = 0;
	stats->tx_carrier_errors = in_be32(&fec->ieee_t_cserr);
	stats->tx_fifo_errors = in_be32(&fec->rmon_t_drop) +
				in_be32(&fec->ieee_t_macerr);
	stats->tx_heartbeat_errors = in_be32(&fec->ieee_t_sqe);
	stats->tx_window_errors = in_be32(&fec->ieee_t_lcol);

	return stats;
}

static void
mpc5xxx_fec_update_stat(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct net_device_stats *stats = &priv->stats;
	struct mpc5xxx_fec *fec = priv->fec;

	out_be32(&fec->mib_control, MPC5xxx_FEC_MIB_DISABLE);
	memset_io(&fec->rmon_t_drop, 0,
			(u32)&fec->reserved10 - (u32)&fec->rmon_t_drop);
	out_be32(&fec->mib_control, 0);
	memset(stats, 0, sizeof *stats);
	mpc5xxx_fec_get_stats(dev);
}

#ifdef ORIGINAL_CODE
/*
 * Set or clear the multicast filter for this adaptor.
 */
static void
mpc5xxx_fec_set_multicast_list(struct rtnet_device *dev)
{
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;
	struct mpc5xxx_fec *fec = priv->fec;
	u32 u32_value;

	if (dev->flags & IFF_PROMISC) {
		printk("%s: Promiscuous mode enabled.\n", dev->name);
		u32_value = in_be32(&fec->r_cntrl);
		u32_value |= MPC5xxx_FEC_RCNTRL_PROM;
		out_be32(&fec->r_cntrl, u32_value);
	}
	else if (dev->flags & IFF_ALLMULTI) {
		u32_value = in_be32(&fec->r_cntrl);
		u32_value &= ~MPC5xxx_FEC_RCNTRL_PROM;
		out_be32(&fec->r_cntrl, u32_value);
		out_be32(&fec->gaddr1, 0xffffffff);
		out_be32(&fec->gaddr2, 0xffffffff);
	}
	else {
		u32 crc;
		int i;
		struct dev_mc_list *dmi;
		u32 gaddr1 = 0x00000000;
		u32 gaddr2 = 0x00000000;

		dmi = dev->mc_list;
		for (i=0; i<dev->mc_count; i++) {
			crc = ether_crc_le(6, dmi->dmi_addr) >> 26;
			if (crc >= 32)
				gaddr1 |= 1 << (crc-32);
			else
				gaddr2 |= 1 << crc;
			dmi = dmi->next;
		}
		out_be32(&fec->gaddr1, gaddr1);
		out_be32(&fec->gaddr2, gaddr2);
	}
}
#endif /* ORIGINAL_CODE */

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET
static void mpc5xxx_mdio_callback(uint regval, struct rtnet_device *dev, uint data)
{
	mdio_read_data_t* mrd = (mdio_read_data_t *)data;
	mrd->regval = 0xFFFF & regval;
	wake_up_process(mrd->sleeping_task);
}

static int mpc5xxx_mdio_read(struct rtnet_device *dev, int phy_id, int location)
{
	uint retval;
	mdio_read_data_t* mrd = (mdio_read_data_t *)kmalloc(sizeof(*mrd),
			GFP_KERNEL);

	mrd->sleeping_task = current;
	set_current_state(TASK_INTERRUPTIBLE);
	mii_queue(dev, mk_mii_read(location),
		mpc5xxx_mdio_callback, (unsigned int) mrd);
	schedule();

	retval = mrd->regval;

	kfree(mrd);

	return retval;
}
#endif /* CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET */

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET_XXX
static void mpc5xxx_mdio_write(struct rtnet_device *dev, int phy_id, int location, int value)
{
	mii_queue(dev, mk_mii_write(location, value), NULL, 0);
}
#endif /* CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET */
#endif	/* CONFIG_XENO_DRIVERS_NET_USE_MDIO */

#ifdef ORIGINAL_CODE
static int
mpc5xxx_netdev_ethtool_ioctl(struct rtnet_device *dev, void *useraddr)
{
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET_XXX
	struct mpc5xxx_fec_priv *private = (struct mpc5xxx_fec_priv *)dev->priv;
#endif
	u32 ethcmd;

	if (copy_from_user(&ethcmd, useraddr, sizeof ethcmd))
		return -EFAULT;

	switch (ethcmd) {

		/* Get driver info */
	case ETHTOOL_GDRVINFO:{
			struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
			strncpy(info.driver, "gt64260",
				sizeof info.driver - 1);
			strncpy(info.version, version,
				sizeof info.version - 1);
			if (copy_to_user(useraddr, &info, sizeof info))
				return -EFAULT;
			return 0;
		}
		/* get settings */
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET_XXX
	case ETHTOOL_GSET:{
			struct ethtool_cmd ecmd = { ETHTOOL_GSET };
			spin_lock_irq(&private->lock);
			mii_ethtool_gset(&private->mii_if, &ecmd);
			spin_unlock_irq(&private->lock);
			if (copy_to_user(useraddr, &ecmd, sizeof ecmd))
				return -EFAULT;
			return 0;
		}
		/* set settings */
	case ETHTOOL_SSET:{
			int r;
			struct ethtool_cmd ecmd;
			if (copy_from_user(&ecmd, useraddr, sizeof ecmd))
				return -EFAULT;
			spin_lock_irq(&private->lock);
			r = mii_ethtool_sset(&private->mii_if, &ecmd);
			spin_unlock_irq(&private->lock);
			return r;
		}
		/* restart autonegotiation */
	case ETHTOOL_NWAY_RST:{
			return mii_nway_restart(&private->mii_if);
		}
		/* get link status */
	case ETHTOOL_GLINK:{
			struct ethtool_value edata = { ETHTOOL_GLINK };
			edata.data = mii_link_ok(&private->mii_if);
			if (copy_to_user(useraddr, &edata, sizeof edata))
				return -EFAULT;
			return 0;
		}
#endif
		/* get message-level */
	case ETHTOOL_GMSGLVL:{
			struct ethtool_value edata = { ETHTOOL_GMSGLVL };
			edata.data = 0;	/* XXX */
			if (copy_to_user(useraddr, &edata, sizeof edata))
				return -EFAULT;
			return 0;
		}
		/* set message-level */
	case ETHTOOL_SMSGLVL:{
			struct ethtool_value edata;
			if (copy_from_user(&edata, useraddr, sizeof edata))
				return -EFAULT;
/* debug = edata.data; *//* XXX */
			return 0;
		}
	}
	return -EOPNOTSUPP;
}

static int
mpc5xxx_fec_ioctl(struct rtnet_device *dev, struct ifreq *rq, int cmd)
{
#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET_XXX
	struct mii_ioctl_data *data = (struct mii_ioctl_data *) &rq->ifr_data;
	int phy = dev->base_addr & 0x1f;
#endif
	int retval;

	switch (cmd) {
	case SIOCETHTOOL:
		retval = mpc5xxx_netdev_ethtool_ioctl(
					dev, (void *) rq->ifr_data);
		break;

#ifdef CONFIG_XENO_DRIVERS_NET_USE_MDIO_NOT_YET_XXX
	case SIOCGMIIPHY:	/* Get address of MII PHY in use. */
	case SIOCDEVPRIVATE:	/* for binary compat, remove in 2.5 */
		data->phy_id = phy;
		/* Fall through */

	case SIOCGMIIREG:	/* Read MII PHY register. */
	case SIOCDEVPRIVATE + 1:	/* for binary compat, remove in 2.5 */
		data->val_out =
			mpc5xxx_mdio_read(dev, data->phy_id&0x1f,
				data->reg_num&0x1f);
		retval = 0;
		break;

	case SIOCSMIIREG:	/* Write MII PHY register. */
	case SIOCDEVPRIVATE + 2:	/* for binary compat, remove in 2.5 */
		if (!capable(CAP_NET_ADMIN)) {
			retval = -EPERM;
		} else {
			mpc5xxx_mdio_write(dev, data->phy_id & 0x1f,
				data->reg_num & 0x1f, data->val_in);
			retval = 0;
		}
		break;
#endif

	default:
		retval = -EOPNOTSUPP;
		break;
	}
	return retval;
}

static void __init
mpc5xxx_fec_str2mac(char *str, unsigned char *mac)
{
	int i;
	u64 val64;

	val64 = simple_strtoull(str, NULL, 16);

	for (i = 0; i < 6; i++)
		mac[5-i] = val64 >> (i*8);
}

static int __init
mpc5xxx_fec_mac_setup(char *mac_address)
{
	mpc5xxx_fec_str2mac(mac_address, mpc5xxx_fec_mac_addr);
	return 0;
}

__setup("mpc5xxx_mac=", mpc5xxx_fec_mac_setup);
#endif /* ORIGINAL_CODE */

static int __init
mpc5xxx_fec_init(void)
{
	struct mpc5xxx_fec *fec;
	struct rtnet_device *dev;
	struct mpc5xxx_fec_priv *priv;
	int err = 0;

#if MPC5xxx_FEC_DEBUG > 1
	printk("mpc5xxx_fec_init\n");
#endif

	if (!rx_pool_size)
		rx_pool_size = MPC5xxx_FEC_RBD_NUM * 2;

	dev = rt_alloc_etherdev(sizeof(*priv), rx_pool_size + MPC5xxx_FEC_TBD_NUM);
	if (!dev)
		return -EIO;
	rtdev_alloc_name(dev, "rteth%d");
	memset(dev->priv, 0, sizeof(*priv));
	rt_rtdev_connect(dev, &RTDEV_manager);
	dev->vers = RTDEV_VERS_2_0;


	mpc5xxx_fec_dev = dev;
	priv = (struct mpc5xxx_fec_priv *)dev->priv;
#if MPC5xxx_FEC_DEBUG > 1
	printk("fec_priv %08x\n", (u32)priv);
#endif
	priv->fec = fec = (struct mpc5xxx_fec *)MPC5xxx_FEC;
	priv->gpio = (struct mpc5xxx_gpio *)MPC5xxx_GPIO;
	priv->sdma = (struct mpc5xxx_sdma *)MPC5xxx_SDMA;

	rtdm_lock_init(&priv->lock);
	dev->open		= mpc5xxx_fec_open;
	dev->stop		= mpc5xxx_fec_close;
	dev->hard_start_xmit	= mpc5xxx_fec_hard_start_xmit;
	//FIXME dev->hard_header	= &rt_eth_header;
	dev->get_stats		= mpc5xxx_fec_get_stats;
#ifdef ORIGINAL_CODE
	dev->do_ioctl		= mpc5xxx_fec_ioctl;
	dev->set_mac_address	= mpc5xxx_fec_set_mac_address;
	dev->set_multicast_list = mpc5xxx_fec_set_multicast_list;

	dev->tx_timeout		= mpc5xxx_fec_tx_timeout;
	dev->watchdog_timeo	= MPC5xxx_FEC_WATCHDOG_TIMEOUT;
#endif /* ORIGINAL_CODE */
	dev->flags &= ~IFF_RUNNING;

	if ((err = rt_register_rtnetdev(dev)))
		goto abort;

#ifdef CONFIG_XENO_DRIVERS_NET_FASTROUTE
	dev->accept_fastpath = mpc5xxx_fec_accept_fastpath;
#endif
	if (memcmp(mpc5xxx_fec_mac_addr, null_mac, 6) != 0)
		memcpy(dev->dev_addr, mpc5xxx_fec_mac_addr, 6);
	else {
		*(u32 *)&dev->dev_addr[0] = in_be32(&fec->paddr1);
		*(u16 *)&dev->dev_addr[4] = in_be16((u16*)&fec->paddr2);
	}

	/*
	 * Read MIB counters in order to reset them,
	 * then zero all the stats fields in memory
	 */
	mpc5xxx_fec_update_stat(dev);

	return 0;

abort:
	rtdev_free(dev);

	return err;
}

static void __exit
mpc5xxx_fec_uninit(void)
{
	struct rtnet_device *dev = mpc5xxx_fec_dev;
	struct mpc5xxx_fec_priv *priv = (struct mpc5xxx_fec_priv *)dev->priv;

	rt_stack_disconnect(dev);
	rt_unregister_rtnetdev(dev);
	rt_rtdev_disconnect(dev);
	printk("%s: unloaded\n", dev->name);
	rtdev_free(dev);
	dev->priv = NULL;
}

static int __init
mpc5xxx_fec_module_init(void)
{
	return mpc5xxx_fec_init();
}

static void __exit
mpc5xxx_fec_module_exit(void)
{
	mpc5xxx_fec_uninit();
}

module_init(mpc5xxx_fec_module_init);
module_exit(mpc5xxx_fec_module_exit);
