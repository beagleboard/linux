/*
 * flexcan.c - FLEXCAN RTCAN controller driver
 *
 * Copyright (c) 2012 Wolfgang Grandegger <wg@denx.de>
 *
 * Derived from the Linux-CAN driver flexcan.c:
 *
 * Copyright (c) 2005-2006 Varma Electronics Oy
 * Copyright (c) 2009 Sascha Hauer, Pengutronix
 * Copyright (c) 2010 Marc Kleine-Budde, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <asm/unaligned.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include "rtcan_dev.h"
#include "rtcan_raw.h"
#include "rtcan_internal.h"

#define DEV_NAME	"rtcan%d"
#define DRV_NAME	"flexcan"

enum flexcan_ip_version {
	FLEXCAN_VER_3_0_0,
	FLEXCAN_VER_3_0_4,
	FLEXCAN_VER_10_0_12,
};

/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT		(8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS		BIT(31)
#define FLEXCAN_MCR_FRZ			BIT(30)
#define FLEXCAN_MCR_FEN			BIT(29)
#define FLEXCAN_MCR_HALT		BIT(28)
#define FLEXCAN_MCR_NOT_RDY		BIT(27)
#define FLEXCAN_MCR_WAK_MSK		BIT(26)
#define FLEXCAN_MCR_SOFTRST		BIT(25)
#define FLEXCAN_MCR_FRZ_ACK		BIT(24)
#define FLEXCAN_MCR_SUPV		BIT(23)
#define FLEXCAN_MCR_SLF_WAK		BIT(22)
#define FLEXCAN_MCR_WRN_EN		BIT(21)
#define FLEXCAN_MCR_LPM_ACK		BIT(20)
#define FLEXCAN_MCR_WAK_SRC		BIT(19)
#define FLEXCAN_MCR_DOZE		BIT(18)
#define FLEXCAN_MCR_SRX_DIS		BIT(17)
#define FLEXCAN_MCR_BCC			BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0xf)
#define FLEXCAN_MCR_IDAM_A		(0 << 8)
#define FLEXCAN_MCR_IDAM_B		(1 << 8)
#define FLEXCAN_MCR_IDAM_C		(2 << 8)
#define FLEXCAN_MCR_IDAM_D		(3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)		(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK		BIT(15)
#define FLEXCAN_CTRL_ERR_MSK		BIT(14)
#define FLEXCAN_CTRL_CLK_SRC		BIT(13)
#define FLEXCAN_CTRL_LPB		BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK		BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK		BIT(10)
#define FLEXCAN_CTRL_SMP		BIT(7)
#define FLEXCAN_CTRL_BOFF_REC		BIT(6)
#define FLEXCAN_CTRL_TSYN		BIT(5)
#define FLEXCAN_CTRL_LBUF		BIT(4)
#define FLEXCAN_CTRL_LOM		BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)		((x) & 0x07)
#define FLEXCAN_CTRL_ERR_BUS		(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
	(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | \
	 FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL \
	(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT		BIT(17)
#define FLEXCAN_ESR_RWRN_INT		BIT(16)
#define FLEXCAN_ESR_BIT1_ERR		BIT(15)
#define FLEXCAN_ESR_BIT0_ERR		BIT(14)
#define FLEXCAN_ESR_ACK_ERR		BIT(13)
#define FLEXCAN_ESR_CRC_ERR		BIT(12)
#define FLEXCAN_ESR_FRM_ERR		BIT(11)
#define FLEXCAN_ESR_STF_ERR		BIT(10)
#define FLEXCAN_ESR_TX_WRN		BIT(9)
#define FLEXCAN_ESR_RX_WRN		BIT(8)
#define FLEXCAN_ESR_IDLE		BIT(7)
#define FLEXCAN_ESR_TXRX		BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT	(4)
#define FLEXCAN_ESR_FLT_CONF_MASK	(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE	(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT		BIT(2)
#define FLEXCAN_ESR_ERR_INT		BIT(1)
#define FLEXCAN_ESR_WAK_INT		BIT(0)
#define FLEXCAN_ESR_ERR_BUS \
	(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | \
	 FLEXCAN_ESR_ACK_ERR | FLEXCAN_ESR_CRC_ERR | \
	 FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL \
	(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | \
	 FLEXCAN_ESR_BOFF_INT | FLEXCAN_ESR_ERR_INT)

/* FLEXCAN interrupt flag register (IFLAG) bits */
#define FLEXCAN_TX_BUF_ID		8
#define FLEXCAN_IFLAG_BUF(x)		BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)
#define FLEXCAN_IFLAG_DEFAULT \
	(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE | \
	 FLEXCAN_IFLAG_BUF(FLEXCAN_TX_BUF_ID))

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CNT_CODE(x)		(((x) & 0xf) << 24)
#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_MB_CODE_MASK		(0xf0ffffff)

/*
 * FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch-  [TR]WRN_INT
 *                                Filter?   connected?
 *   MX25  FlexCAN2  03.00.00.00     no         no
 *   MX28  FlexCAN2  03.00.04.00    yes        yes
 *   MX35  FlexCAN2  03.00.00.00     no         no
 *   MX53  FlexCAN2  03.00.00.00    yes         no
 *   MX6s  FlexCAN3  10.00.12.00    yes        yes
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_HAS_V10_FEATURES	BIT(1) /* For core version >= 10 */
#define FLEXCAN_HAS_BROKEN_ERR_STATE	BIT(2) /* [TR]WRN_INT not connected */

/* Structure of the message buffer */
struct flexcan_mb {
	u32 can_ctrl;
	u32 can_id;
	u32 data[2];
};

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 */
	u32 timer;		/* 0x08 */
	u32 _reserved1;		/* 0x0c */
	u32 rxgmask;		/* 0x10 */
	u32 rx14mask;		/* 0x14 */
	u32 rx15mask;		/* 0x18 */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	u32 crl2;		/* 0x34 */
	u32 esr2;		/* 0x38 */
	u32 _reserved2[2];
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 _reserved3[12];
	struct flexcan_mb cantxfg[64];
};

struct flexcan_devtype_data {
	u32 features;	/* hardware controller features */
};

struct flexcan_priv {
	struct rtcan_device *dev;

	int irq;
	void __iomem *base;
	u32 reg_esr;
	u32 reg_ctrl_default;

	struct can_bittime bit_time;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;
	struct clk *clk_ipg;
	struct clk *clk_per;
};

static struct flexcan_devtype_data fsl_p1010_devtype_data = {
	.features = FLEXCAN_HAS_BROKEN_ERR_STATE,
};

static struct flexcan_devtype_data fsl_imx28_devtype_data;

static struct flexcan_devtype_data fsl_imx6q_devtype_data = {
	.features = FLEXCAN_HAS_V10_FEATURES,
};

static char *flexcan_ctrl_name = "FLEXCAN";

static struct can_bittiming_const flexcan_bittiming_const = {
	.name = "flexcan",
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

/*
 * Abstract off the read/write for arm versus ppc.
 */
#if defined(__BIG_ENDIAN)
static inline u32 flexcan_read(void __iomem *addr)
{
	return in_be32(addr);
}

static inline void flexcan_write(u32 val, void __iomem *addr)
{
	out_be32(addr, val);
}
#else
static inline u32 flexcan_read(void __iomem *addr)
{
	return readl(addr);
}

static inline void flexcan_write(u32 val, void __iomem *addr)
{
	writel(val, addr);
}
#endif

static inline void flexcan_clk_enable(struct flexcan_priv *priv)
{
	clk_prepare_enable(priv->clk_ipg);
	clk_prepare_enable(priv->clk_per);
}
static inline void flexcan_clk_disable(struct flexcan_priv *priv)
{
	clk_disable_unprepare(priv->clk_ipg);
	clk_disable_unprepare(priv->clk_per);
}

/*
 * Switch transceiver on or off
 */
static int flexcan_transceiver_switch(const struct flexcan_priv *priv, int on)
{
	if (priv->reg_xceiver == NULL)
		return 0;

	if (on)
		return regulator_enable(priv->reg_xceiver);

	return regulator_disable(priv->reg_xceiver);
}

static inline void flexcan_chip_enable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	flexcan_write(reg, &regs->mcr);

	udelay(10);
}

static inline void flexcan_chip_disable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	flexcan_write(reg, &regs->mcr);
}

static int flexcan_start_xmit(struct rtcan_device *dev, struct can_frame *cf)
{
	const struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	u32 can_id;
	u32 ctrl;

	/* If DLC exceeds 8 bytes adjust it to 8 (for the payload) */
	if (cf->can_dlc > 8)
		cf->can_dlc = 8;

	ctrl = FLEXCAN_MB_CNT_CODE(0xc) | (cf->can_dlc << 16);

	if (cf->can_id & CAN_EFF_FLAG) {
		can_id = cf->can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} else {
		can_id = (cf->can_id & CAN_SFF_MASK) << 18;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ctrl |= FLEXCAN_MB_CNT_RTR;

	if (cf->can_dlc > 0) {
		u32 data = be32_to_cpup((__be32 *)&cf->data[0]);
		flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
	}
	if (cf->can_dlc > 3) {
		u32 data = be32_to_cpup((__be32 *)&cf->data[4]);
		flexcan_write(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[1]);
	}

	flexcan_write(can_id, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_id);
	flexcan_write(ctrl, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);

	return 0;
}

static void flexcan_do_bus_err(struct rtcan_device *dev,
			       struct rtcan_rb_frame *cf, u32 reg_esr)
{
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	if (reg_esr & FLEXCAN_ESR_BIT1_ERR)
		cf->data[2] |= CAN_ERR_PROT_BIT1;
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR)
		cf->data[2] |= CAN_ERR_PROT_BIT0;
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] |= CAN_ERR_PROT_LOC_ACK;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR)
		cf->data[2] |= CAN_ERR_PROT_FORM;
	if (reg_esr & FLEXCAN_ESR_STF_ERR)
		cf->data[2] |= CAN_ERR_PROT_STUFF;
}

static can_state_t flexcan_get_state(struct rtcan_device *dev, u32 reg_esr)
{
	int flt;

	flt = reg_esr & FLEXCAN_ESR_FLT_CONF_MASK;
	if (likely(flt == FLEXCAN_ESR_FLT_CONF_ACTIVE)) {
		if (likely(!(reg_esr & (FLEXCAN_ESR_TX_WRN |
					FLEXCAN_ESR_RX_WRN))))
			return CAN_STATE_ERROR_ACTIVE;
		else
			return CAN_STATE_ERROR_WARNING;
	} else if (unlikely(flt == FLEXCAN_ESR_FLT_CONF_PASSIVE)) {
		return CAN_STATE_ERROR_PASSIVE;
	} else {
		return CAN_STATE_BUS_OFF;
	}
}

static void flexcan_do_state(struct rtcan_device *dev,
			     struct rtcan_rb_frame *cf,
			     can_state_t new_state)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg = flexcan_read(&regs->ecr);
	u8 txerr = reg & 0xff;
	u8 rxerr = (reg >> 8) & 0xff;

	switch (dev->state) {
	case CAN_STATE_ERROR_ACTIVE:
		/*
		 * from: ERROR_ACTIVE
		 * to  : ERROR_WARNING, ERROR_PASSIVE, BUS_OFF
		 * =>  : there was a warning int
		 */
		if (new_state >= CAN_STATE_ERROR_WARNING &&
		    new_state <= CAN_STATE_BUS_OFF) {
			rtcandev_dbg(dev, "Error Warning IRQ\n");

			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		}
	case CAN_STATE_ERROR_WARNING:	/* fallthrough */
		/*
		 * from: ERROR_ACTIVE, ERROR_WARNING
		 * to  : ERROR_PASSIVE, BUS_OFF
		 * =>  : error passive int
		 */
		if (new_state >= CAN_STATE_ERROR_PASSIVE &&
		    new_state <= CAN_STATE_BUS_OFF) {
			rtcandev_dbg(dev, "Error Passive IRQ\n");

			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
		break;
	case CAN_STATE_BUS_OFF:
		rtcandev_err(dev, "BUG! "
			     "hardware recovered automatically from BUS_OFF\n");
		break;
	default:
		break;
	}

	/* process state changes depending on the new state */
	switch (new_state) {
	case CAN_STATE_ERROR_ACTIVE:
		rtcandev_dbg(dev, "Error Active\n");
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] = CAN_ERR_PROT_ACTIVE;
		break;
	case CAN_STATE_BUS_OFF:
		cf->can_id |= CAN_ERR_BUSOFF;
		/* Wake up waiting senders */
		rtdm_sem_destroy(&dev->tx_sem);
		break;
	default:
		break;
	}
}

static void flexcan_rx_interrupt(struct rtcan_device *dev,
				 struct rtcan_skb *skb)
{
	const struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	struct flexcan_mb __iomem *mb = &regs->cantxfg[0];
	struct rtcan_rb_frame *cf = &skb->rb_frame;
	u32 reg_ctrl, reg_id;

	reg_ctrl = flexcan_read(&mb->can_ctrl);
	reg_id = flexcan_read(&mb->can_id);
	if (reg_ctrl & FLEXCAN_MB_CNT_IDE)
		cf->can_id = ((reg_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = (reg_id >> 18) & CAN_SFF_MASK;

	cf->can_dlc = (reg_ctrl >> 16) & 0xf;
	/* If DLC exceeds 8 bytes adjust it to 8 (for the payload size) */
	if (cf->can_dlc > 8)
		cf->can_dlc = 8;

	if (reg_ctrl & FLEXCAN_MB_CNT_RTR) {
		cf->can_id |= CAN_RTR_FLAG;
		skb->rb_frame_size = EMPTY_RB_FRAME_SIZE;
	} else {
		skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + cf->can_dlc;
		put_unaligned_be32(flexcan_read(&mb->data[0]), cf->data + 0);
		put_unaligned_be32(flexcan_read(&mb->data[1]), cf->data + 4);
	}

	/* Store the interface index */
	cf->can_ifindex = dev->ifindex;

	/* mark as read */
	flexcan_write(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
	flexcan_read(&regs->timer);
}

static void flexcan_err_interrupt(struct rtcan_device *dev,
				  struct rtcan_skb *skb,
				  u32 reg_iflag1, u32 reg_esr,
				  can_state_t new_state)
{
	const struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	struct rtcan_rb_frame *cf = &skb->rb_frame;

	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + CAN_ERR_DLC;

	cf->can_id = CAN_ERR_FLAG;
	cf->can_dlc = CAN_ERR_DLC;
	memset(&cf->data[0], 0, cf->can_dlc);

	/* State changed? */
	if (new_state != dev->state) {
		flexcan_do_state(dev, cf, new_state);
		dev->state = new_state;
	}

	/* FIFO overflow? */
	if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
		flexcan_write(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW, &regs->iflag1);
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	}

	/* Bus errors? */
	if (reg_esr & FLEXCAN_ESR_ERR_BUS)
		flexcan_do_bus_err(dev, cf, reg_esr);
}

static int flexcan_interrupt(rtdm_irq_t *irq_handle)
{
	struct rtcan_device *dev = rtdm_irq_get_arg(irq_handle, void);
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	struct rtcan_skb skb;
	u32 reg_iflag1, reg_esr;
	int recv_lock_free = 1;
	int ret = RTDM_IRQ_NONE;
	can_state_t new_state;

	rtdm_lock_get(&dev->device_lock);

	reg_iflag1 = flexcan_read(&regs->iflag1);
	reg_esr = flexcan_read(&regs->esr);
	/* ACK all bus error and state change IRQ sources */
	if (reg_esr & FLEXCAN_ESR_ALL_INT)
		flexcan_write(reg_esr & FLEXCAN_ESR_ALL_INT, &regs->esr);

	/* transmission complete interrupt */
	if (reg_iflag1 & (1 << FLEXCAN_TX_BUF_ID)) {
		flexcan_write((1 << FLEXCAN_TX_BUF_ID), &regs->iflag1);

		/* Wake up a sender */
		rtdm_sem_up(&dev->tx_sem);
		if (rtcan_loopback_pending(dev)) {
			if (recv_lock_free) {
				recv_lock_free = 0;
				rtdm_lock_get(&rtcan_recv_list_lock);
				rtdm_lock_get(&rtcan_socket_lock);
			}
			rtcan_loopback(dev);
		}
		ret = RTDM_IRQ_HANDLED;
	}

	/* RX interrupt? */
	while (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
		flexcan_rx_interrupt(dev, &skb);
		reg_iflag1 = flexcan_read(&regs->iflag1);

		/* Take more locks. Ensure that they are taken and
		 * released only once in the IRQ handler. */
		/* WARNING: Nested locks are dangerous! But they are
		 * nested only in this routine so a deadlock should
		 * not be possible. */
		if (recv_lock_free) {
			recv_lock_free = 0;
			rtdm_lock_get(&rtcan_recv_list_lock);
			rtdm_lock_get(&rtcan_socket_lock);
		}

		/* Pass received frame out to the sockets */
		rtcan_rcv(dev, &skb);
		ret = RTDM_IRQ_HANDLED;
	}

	/* Error or state change Interrupt? */
	new_state = flexcan_get_state(dev, reg_esr);
	if ((new_state != dev->state) ||
	     (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) ||
	     (reg_esr & FLEXCAN_ESR_ERR_BUS)) {
		/* Check error condition and fill error frame */
		flexcan_err_interrupt(dev, &skb, reg_iflag1, reg_esr,
				      new_state);

		if (recv_lock_free) {
			recv_lock_free = 0;
			rtdm_lock_get(&rtcan_recv_list_lock);
			rtdm_lock_get(&rtcan_socket_lock);
		}

		/* Pass error frame out to the sockets */
		rtcan_rcv(dev, &skb);
		ret = RTDM_IRQ_HANDLED;
	}

	if (!recv_lock_free) {
		rtdm_lock_put(&rtcan_socket_lock);
		rtdm_lock_put(&rtcan_recv_list_lock);
	}
	rtdm_lock_put(&dev->device_lock);

	return ret;
}

static int flexcan_save_bit_time(struct rtcan_device *dev,
				 struct can_bittime *bt,
				 rtdm_lockctx_t *lock_ctx)
{
	struct flexcan_priv *priv = rtcan_priv(dev);

	memcpy(&priv->bit_time, bt, sizeof(*bt));

	return 0;
}

static int flexcan_set_bit_time(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	struct can_bittime *bt = &priv->bit_time;
	u32 reg;

	if (bt->type != CAN_BITTIME_STD)
		return -EINVAL;

	reg = flexcan_read(&regs->ctrl);
	reg &= ~(FLEXCAN_CTRL_PRESDIV(0xff) |
		 FLEXCAN_CTRL_RJW(0x3) |
		 FLEXCAN_CTRL_PSEG1(0x7) |
		 FLEXCAN_CTRL_PSEG2(0x7) |
		 FLEXCAN_CTRL_PROPSEG(0x7) |
		 FLEXCAN_CTRL_LPB |
		 FLEXCAN_CTRL_SMP |
		 FLEXCAN_CTRL_LOM);

	reg |= FLEXCAN_CTRL_PRESDIV(bt->std.brp - 1) |
		FLEXCAN_CTRL_PSEG1(bt->std.phase_seg1 - 1) |
		FLEXCAN_CTRL_PSEG2(bt->std.phase_seg2 - 1) |
		FLEXCAN_CTRL_RJW(bt->std.sjw - 1) |
		FLEXCAN_CTRL_PROPSEG(bt->std.prop_seg - 1);

	if (dev->ctrl_mode & CAN_CTRLMODE_LOOPBACK)
		reg |= FLEXCAN_CTRL_LPB;
	if (dev->ctrl_mode & CAN_CTRLMODE_LISTENONLY)
		reg |= FLEXCAN_CTRL_LOM;
	if (dev->ctrl_mode & CAN_CTRLMODE_3_SAMPLES)
		reg |= FLEXCAN_CTRL_SMP;

	rtcandev_info(dev, "writing ctrl=0x%08x\n", reg);
	flexcan_write(reg, &regs->ctrl);

	/* print chip status */
	rtcandev_dbg(dev, "%s: mcr=0x%08x ctrl=0x%08x\n", __func__,
		     flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));

	return 0;
}

/*
 * flexcan_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int flexcan_chip_start(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	unsigned int i;
	int err;
	u32 reg_mcr, reg_ctrl;

	/* enable module */
	flexcan_chip_enable(priv);

	/* soft reset */
	flexcan_write(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	udelay(10);

	reg_mcr = flexcan_read(&regs->mcr);
	if (reg_mcr & FLEXCAN_MCR_SOFTRST) {
		rtcandev_err(dev,
			     "Failed to softreset can module (mcr=0x%08x)\n",
			     reg_mcr);
		err = -ENODEV;
		goto out;
	}

	flexcan_set_bit_time(dev);

	/*
	 * MCR
	 *
	 * enable freeze
	 * enable fifo
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * choose format C
	 * disable local echo
	 *
	 */
	reg_mcr = flexcan_read(&regs->mcr);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_FEN | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_SUPV | FLEXCAN_MCR_WRN_EN |
		FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_SRX_DIS;
	rtcandev_dbg(dev, "%s: writing mcr=0x%08x", __func__, reg_mcr);
	flexcan_write(reg_mcr, &regs->mcr);

	/*
	 * CTRL
	 *
	 * disable timer sync feature
	 *
	 * disable auto busoff recovery
	 * transmit lowest buffer first
	 *
	 * enable tx and rx warning interrupt
	 * enable bus off interrupt
	 * (== FLEXCAN_CTRL_ERR_STATE)
	 */
	reg_ctrl = flexcan_read(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF |
		FLEXCAN_CTRL_ERR_STATE;
	/*
	 * enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->features & FLEXCAN_HAS_BROKEN_ERR_STATE)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	rtcandev_dbg(dev, "%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	flexcan_write(reg_ctrl, &regs->ctrl);

	for (i = 0; i < ARRAY_SIZE(regs->cantxfg); i++) {
		flexcan_write(0, &regs->cantxfg[i].can_ctrl);
		flexcan_write(0, &regs->cantxfg[i].can_id);
		flexcan_write(0, &regs->cantxfg[i].data[0]);
		flexcan_write(0, &regs->cantxfg[i].data[1]);

		/* put MB into rx queue */
		flexcan_write(FLEXCAN_MB_CNT_CODE(0x4),
			&regs->cantxfg[i].can_ctrl);
	}

	/* acceptance mask/acceptance code (accept everything) */
	flexcan_write(0x0, &regs->rxgmask);
	flexcan_write(0x0, &regs->rx14mask);
	flexcan_write(0x0, &regs->rx15mask);

	if (priv->devtype_data->features & FLEXCAN_HAS_V10_FEATURES)
		flexcan_write(0x0, &regs->rxfgmask);

	flexcan_transceiver_switch(priv, 1);

	/* synchronize with the can bus */
	reg_mcr = flexcan_read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_HALT;
	flexcan_write(reg_mcr, &regs->mcr);

	dev->state = CAN_STATE_ERROR_ACTIVE;

	/* enable FIFO interrupts */
	flexcan_write(FLEXCAN_IFLAG_DEFAULT, &regs->imask1);

	/* print chip status */
	rtcandev_dbg(dev, "%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		     flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));

	return 0;

out:
	flexcan_chip_disable(priv);
	return err;
}

/*
 * flexcan_chip_stop
 *
 * this functions is entered with clocks enabled
 *
 */
static void flexcan_chip_stop(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg;

	/* Disable all interrupts */
	flexcan_write(0, &regs->imask1);

	/* Disable + halt module */
	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS | FLEXCAN_MCR_HALT;
	flexcan_write(reg, &regs->mcr);

	flexcan_transceiver_switch(priv, 0);
	dev->state = CAN_STATE_STOPPED;

	return;
}

static int flexcan_mode_stop(struct rtcan_device *dev, rtdm_lockctx_t *lock_ctx)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	can_state_t state;

	state = dev->state;
	/* If controller is not operating anyway, go out */
	if (!CAN_STATE_OPERATING(state))
		goto out;

	/*
	 * Drop the device lock early, we should not need it and we
	 * may not hold it for calling the regular kernel
	 * infrastructure.
	 */
	rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);

	flexcan_chip_stop(dev);

	/* Wake up waiting senders */
	rtdm_sem_destroy(&dev->tx_sem);

	rtdm_irq_free(&dev->irq_handle);

	flexcan_clk_disable(priv);

	rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
out:
	return 0;
}

static int flexcan_mode_start(struct rtcan_device *dev,
			      rtdm_lockctx_t *lock_ctx)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	int err = 0;

	rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);

	switch (dev->state) {

	case CAN_STATE_ACTIVE:
	case CAN_STATE_BUS_WARNING:
	case CAN_STATE_BUS_PASSIVE:
		break;

	case CAN_STATE_STOPPED:
		flexcan_clk_enable(priv);

		/* Register IRQ handler and pass device structure as arg */
		err = rtdm_irq_request(&dev->irq_handle, priv->irq,
				       flexcan_interrupt, 0, DRV_NAME,
				       (void *)dev);
		if (err) {
			rtcandev_err(dev, "couldn't request irq %d\n",
				     priv->irq);
			goto out_clk_disable;
		}

		/* start chip and queuing */
		err = flexcan_chip_start(dev);
		if (err)
			goto out_irq_free;

		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);

		break;

	case CAN_STATE_BUS_OFF:
		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);
		/* start chip and queuing */
		err = flexcan_chip_start(dev);
		if (err)
			goto out;
		break;

	case CAN_STATE_SLEEPING:
	default:
		/* Never reached, but we don't want nasty compiler warnings ... */
		err = 0;
		break;
	}

	goto out;

out_irq_free:
	rtdm_irq_free(&dev->irq_handle);
out_clk_disable:
	flexcan_clk_disable(priv);
out:
	rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);

	return err;
}

int flexcan_set_mode(struct rtcan_device *dev, can_mode_t mode,
		     rtdm_lockctx_t *lock_ctx)
{
	switch (mode) {

	case CAN_MODE_STOP:
		return flexcan_mode_stop(dev, lock_ctx);

	case CAN_MODE_START:
		return flexcan_mode_start(dev, lock_ctx);

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int register_flexcandev(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->base;
	u32 reg, err;

	flexcan_clk_enable(priv);

	/* select "bus clock", chip must be disabled */
	flexcan_chip_disable(priv);
	reg = flexcan_read(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	flexcan_write(reg, &regs->ctrl);

	flexcan_chip_enable(priv);

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	flexcan_write(reg, &regs->mcr);

	/*
	 * Currently we only support newer versions of this core
	 * featuring a RX FIFO. Older cores found on some Coldfire
	 * derivates are not yet supported.
	 */
	reg = flexcan_read(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		rtcandev_err(dev,
			     "Could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out_chip_disable;
	}

	err = rtcan_dev_register(dev);
	if (err)
		goto out_chip_disable;

	return 0;

out_chip_disable:
	/* disable core and turn off clocks */
	flexcan_chip_disable(priv);
	flexcan_clk_disable(priv);

	return err;
}

static void unregister_flexcandev(struct rtcan_device *dev)
{
	flexcan_mode_stop(dev, NULL);
	rtcan_dev_unregister(dev);
}

static struct of_device_id flexcan_of_match[] = {
	{ .compatible = "fsl,p1010-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,imx28-flexcan", .data = &fsl_imx28_devtype_data, },
	{ .compatible = "fsl,imx6q-flexcan", .data = &fsl_imx6q_devtype_data, },
	{ /* sentinel */ },
};

static int flexcan_probe(struct platform_device *pdev)
{
	const struct flexcan_devtype_data *devtype_data;
	const struct of_device_id *of_id = NULL;
	struct flexcan_priv *priv;
	struct rtcan_device *dev;
	resource_size_t mem_size;
	struct pinctrl *pinctrl;
	struct resource *mem;
	void __iomem *base;
	u32 clock_freq = 0;
	int err, irq;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		return PTR_ERR(pinctrl);

	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node,
						"clock-frequency", &clock_freq);
		of_id = of_match_device(flexcan_of_match, &pdev->dev);
	}

	if (of_id)
		devtype_data = of_id->data;
	else
		devtype_data = (typeof(devtype_data))
			platform_get_device_id(pdev)->driver_data;

	if (devtype_data == NULL) {
		dev_err(&pdev->dev, "no feature set defined");
		return -ENODEV;
	}

	dev = rtcan_dev_alloc(sizeof(struct flexcan_priv), 0);
	if (dev == NULL)
		return -ENOMEM;

	priv = rtcan_priv(dev);

	if (!clock_freq) {
		priv->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(priv->clk_ipg)) {
			dev_err(&pdev->dev, "no ipg clock defined\n");
			return PTR_ERR(priv->clk_ipg);
		}

		priv->clk_per = devm_clk_get(&pdev->dev, "per");
		if (IS_ERR(priv->clk_per)) {
			clk_put(priv->clk_ipg);
			dev_err(&pdev->dev, "no per clock defined\n");
			return PTR_ERR(priv->clk_per);
		}
		clock_freq = clk_get_rate(priv->clk_per);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq <= 0) {
		err = -ENODEV;
		goto out_get;
	}

	mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, mem_size, pdev->name)) {
		err = -EBUSY;
		goto out_get;
	}

	base = ioremap(mem->start, mem_size);
	if (!base) {
		err = -ENOMEM;
		goto out_map;
	}

	priv->base = base;
	priv->irq = irq;
	priv->dev = dev;
	priv->devtype_data = devtype_data;
	priv->reg_xceiver = devm_regulator_get(&pdev->dev, "xceiver");
	if (IS_ERR(priv->reg_xceiver))
		priv->reg_xceiver = NULL;

	dev_set_drvdata(&pdev->dev, dev);

	dev->ctrl_name = flexcan_ctrl_name;
	dev->board_name = flexcan_ctrl_name;
	dev->base_addr = (unsigned long)base;
	dev->can_sys_clock = clock_freq;
	dev->hard_start_xmit = flexcan_start_xmit;
	dev->do_set_mode = flexcan_set_mode;
	dev->do_set_bit_time = flexcan_save_bit_time;
	dev->bittiming_const = &flexcan_bittiming_const;
	dev->state = CAN_STATE_STOPPED;

	/* Give device an interface name */
	strncpy(dev->name, DEV_NAME, IFNAMSIZ);

	/* Register RTDM device */
	err = register_flexcandev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering RTCAN device failed\n");
		goto out_register;
	}

	dev_info(&pdev->dev,
		 "RTCAN device registered (reg_base=%p, irq=%d, clock=%d)\n",
		 priv->base, priv->irq, dev->can_sys_clock);

	return 0;

out_register:
	iounmap(base);
out_map:
	release_mem_region(mem->start, mem_size);
out_get:
	rtcan_dev_free(dev);

	return err;
}

static int flexcan_remove(struct platform_device *pdev)
{
	struct rtcan_device *dev = platform_get_drvdata(pdev);
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct resource *mem;

	unregister_flexcandev(dev);
	platform_set_drvdata(pdev, NULL);
	iounmap(priv->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	rtcan_dev_free(dev);

	return 0;
}

static struct platform_device_id flexcan_id_table[] = {
	{ .name = "imx25-flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ .name = "imx28-flexcan", .driver_data = (kernel_ulong_t)&fsl_imx28_devtype_data, },
	{ .name = "imx35-flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ .name = "imx53-flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ .name = "imx6q-flexcan", .driver_data = (kernel_ulong_t)&fsl_imx6q_devtype_data, },
	{ .name = "flexcan",       .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};

static struct platform_driver flexcan_driver = {
	.driver = {
		/* For legacy platform support */
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = flexcan_of_match,
	},
	.id_table = flexcan_id_table,
	.probe = flexcan_probe,
	.remove = flexcan_remove,
};

module_platform_driver(flexcan_driver);

MODULE_AUTHOR("Wolfgang Grandegger <wg@denx.de>, "
	      "Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RT-CAN port driver for flexcan based chip");
