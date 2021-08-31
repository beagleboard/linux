/*
 * RTDM-based FLEXCAN CAN controller driver
 *
 * Rebased on linux 4.14.58 flexcan driver:
 * Copyright (c) 2018 Philippe Gerum <rpm@xenomai.org>
 *
 * Original port to RTDM:
 * Copyright (c) 2012 Wolfgang Grandegger <wg@denx.de>
 *
 * Copyright (c) 2005-2006 Varma Electronics Oy
 * Copyright (c) 2009 Sascha Hauer, Pengutronix
 * Copyright (c) 2010-2017 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
 * Copyright (c) 2014 David Jander, Protonic Holland
 *
 * Based on code originally by Andrey Volkov <avolkov@varma-el.com>
 *
 * LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <rtdm/driver.h>
#include <rtdm/can.h>
#include "rtcan_dev.h"
#include "rtcan_raw.h"
#include "rtcan_internal.h"
#include <asm/unaligned.h>

#define DRV_NAME	"flexcan"
#define DEV_NAME	"rtcan%d"

#define CAN_MAX_DLC 8
#define get_can_dlc(i)		(min_t(__u8, (i), CAN_MAX_DLC))

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
#define FLEXCAN_MCR_IRMQ		BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
/* MCR_MAXMB: maximum used MBs is MAXMB + 1 */
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0x7f)
#define FLEXCAN_MCR_IDAM_A		(0x0 << 8)
#define FLEXCAN_MCR_IDAM_B		(0x1 << 8)
#define FLEXCAN_MCR_IDAM_C		(0x2 << 8)
#define FLEXCAN_MCR_IDAM_D		(0x3 << 8)

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

/* FLEXCAN control register 2 (CTRL2) bits */
#define FLEXCAN_CTRL2_ECRWRE		BIT(29)
#define FLEXCAN_CTRL2_WRMFRZ		BIT(28)
#define FLEXCAN_CTRL2_RFFN(x)		(((x) & 0x0f) << 24)
#define FLEXCAN_CTRL2_TASD(x)		(((x) & 0x1f) << 19)
#define FLEXCAN_CTRL2_MRP		BIT(18)
#define FLEXCAN_CTRL2_RRS		BIT(17)
#define FLEXCAN_CTRL2_EACEN		BIT(16)

/* FLEXCAN memory error control register (MECR) bits */
#define FLEXCAN_MECR_ECRWRDIS		BIT(31)
#define FLEXCAN_MECR_HANCEI_MSK		BIT(19)
#define FLEXCAN_MECR_FANCEI_MSK		BIT(18)
#define FLEXCAN_MECR_CEI_MSK		BIT(16)
#define FLEXCAN_MECR_HAERRIE		BIT(15)
#define FLEXCAN_MECR_FAERRIE		BIT(14)
#define FLEXCAN_MECR_EXTERRIE		BIT(13)
#define FLEXCAN_MECR_RERRDIS		BIT(9)
#define FLEXCAN_MECR_ECCDIS		BIT(8)
#define FLEXCAN_MECR_NCEFAFRZ		BIT(7)

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
/* Errata ERR005829 step7: Reserve first valid MB */
#define FLEXCAN_TX_MB_RESERVED_OFF_FIFO	8
#define FLEXCAN_TX_MB_OFF_FIFO		9
#define FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP	0
#define FLEXCAN_TX_MB_OFF_TIMESTAMP		1
#define FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST	(FLEXCAN_TX_MB_OFF_TIMESTAMP + 1)
#define FLEXCAN_RX_MB_OFF_TIMESTAMP_LAST	63
#define FLEXCAN_RX_MB_TIMESTAMP_COUNT	(FLEXCAN_RX_MB_OFF_TIMESTAMP_LAST -	\
					 FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST + 1)
#define FLEXCAN_IFLAG_MB(x)		BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CODE_MASK		(0xf << 24)
#define FLEXCAN_MB_CODE_RX_BUSY_BIT	(0x1 << 24)
#define FLEXCAN_MB_CODE_RX_INACTIVE	(0x0 << 24)
#define FLEXCAN_MB_CODE_RX_EMPTY	(0x4 << 24)
#define FLEXCAN_MB_CODE_RX_FULL		(0x2 << 24)
#define FLEXCAN_MB_CODE_RX_OVERRUN	(0x6 << 24)
#define FLEXCAN_MB_CODE_RX_RANSWER	(0xa << 24)

#define FLEXCAN_MB_CODE_TX_INACTIVE	(0x8 << 24)
#define FLEXCAN_MB_CODE_TX_ABORT	(0x9 << 24)
#define FLEXCAN_MB_CODE_TX_DATA		(0xc << 24)
#define FLEXCAN_MB_CODE_TX_TANSWER	(0xe << 24)

#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_TIMEOUT_US		(50)

/* FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch- [TR]WRN_INT IRQ Err Memory err RTR re-
 *                                Filter? connected?  Passive detection  ception in MB
 *   MX25  FlexCAN2  03.00.00.00     no        no         ?       no        no
 *   MX28  FlexCAN2  03.00.04.00    yes       yes        no       no        no
 *   MX35  FlexCAN2  03.00.00.00     no        no         ?       no        no
 *   MX53  FlexCAN2  03.00.00.00    yes        no        no       no        no
 *   MX6s  FlexCAN3  10.00.12.00    yes       yes        no       no       yes
 *   VF610 FlexCAN3  ?               no       yes        no      yes       yes?
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_QUIRK_BROKEN_WERR_STATE	BIT(1) /* [TR]WRN_INT not connected */
#define FLEXCAN_QUIRK_DISABLE_RXFG	BIT(2) /* Disable RX FIFO Global mask */
#define FLEXCAN_QUIRK_ENABLE_EACEN_RRS	BIT(3) /* Enable EACEN and RRS bit in ctrl2 */
#define FLEXCAN_QUIRK_DISABLE_MECR	BIT(4) /* Disable Memory error detection */
#define FLEXCAN_QUIRK_USE_OFF_TIMESTAMP	BIT(5) /* Use timestamp based offloading */
#define FLEXCAN_QUIRK_BROKEN_PERR_STATE	BIT(6) /* No interrupt for error passive */

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
	union {			/* 0x34 */
		u32 gfwr_mx28;	/* MX28, MX53 */
		u32 ctrl2;	/* MX6, VF610 */
	};
	u32 esr2;		/* 0x38 */
	u32 imeur;		/* 0x3c */
	u32 lrfr;		/* 0x40 */
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 _reserved3[12];	/* 0x50 */
	struct flexcan_mb mb[64];	/* 0x80 */
	/* FIFO-mode:
	 *			MB
	 * 0x080...0x08f	0	RX message buffer
	 * 0x090...0x0df	1-5	reserverd
	 * 0x0e0...0x0ff	6-7	8 entry ID table
	 *				(mx25, mx28, mx35, mx53)
	 * 0x0e0...0x2df	6-7..37	8..128 entry ID table
	 *				size conf'ed via ctrl2::RFFN
	 *				(mx6, vf610)
	 */
	u32 _reserved4[256];	/* 0x480 */
	u32 rximr[64];		/* 0x880 */
	u32 _reserved5[24];	/* 0x980 */
	u32 gfwr_mx6;		/* 0x9e0 - MX6 */
	u32 _reserved6[63];	/* 0x9e4 */
	u32 mecr;		/* 0xae0 */
	u32 erriar;		/* 0xae4 */
	u32 erridpr;		/* 0xae8 */
	u32 errippr;		/* 0xaec */
	u32 rerrar;		/* 0xaf0 */
	u32 rerrdr;		/* 0xaf4 */
	u32 rerrsynr;		/* 0xaf8 */
	u32 errsr;		/* 0xafc */
};

struct flexcan_devtype_data {
	u32 quirks;		/* quirks needed for different IP cores */
};

struct flexcan_timestamped_frame {
	struct rtcan_skb skb;
	u32 timestamp;
	struct list_head next;
};

struct flexcan_priv {
	unsigned int irq;
	unsigned int mb_first;
	unsigned int mb_last;
	struct can_bittime bittiming;
	struct flexcan_timestamped_frame *ts_frames;

	struct flexcan_regs __iomem *regs;
	struct flexcan_mb __iomem *tx_mb;
	struct flexcan_mb __iomem *tx_mb_reserved;
	u8 tx_mb_idx;
	u32 reg_ctrl_default;
	u32 reg_imask1_default;
	u32 reg_imask2_default;

	struct clk *clk_ipg;
	struct clk *clk_per;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;

	unsigned long bus_errors;
};

static const struct flexcan_devtype_data fsl_p1010_devtype_data = {
	.quirks = FLEXCAN_QUIRK_BROKEN_WERR_STATE |
		FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_imx28_devtype_data = {
	.quirks = FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_imx6q_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
	FLEXCAN_QUIRK_USE_OFF_TIMESTAMP | FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_vf610_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_DISABLE_MECR | FLEXCAN_QUIRK_USE_OFF_TIMESTAMP |
		FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct can_bittiming_const flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

/* Abstract off the read/write for arm versus ppc. This
 * assumes that PPC uses big-endian registers and everything
 * else uses little-endian registers, independent of CPU
 * endianness.
 */
#if defined(CONFIG_PPC)
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

static inline void flexcan_error_irq_enable(const struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl = (priv->reg_ctrl_default | FLEXCAN_CTRL_ERR_MSK);

	flexcan_write(reg_ctrl, &regs->ctrl);
}

static inline void flexcan_error_irq_disable(const struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl = (priv->reg_ctrl_default & ~FLEXCAN_CTRL_ERR_MSK);

	flexcan_write(reg_ctrl, &regs->ctrl);
}

static inline int flexcan_transceiver_enable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_enable(priv->reg_xceiver);
}

static inline int flexcan_transceiver_disable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_disable(priv->reg_xceiver);
}

static int flexcan_chip_enable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	flexcan_write(reg, &regs->mcr);

	while (timeout-- && (flexcan_read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (flexcan_read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_disable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	flexcan_write(reg, &regs->mcr);

	while (timeout-- && !(flexcan_read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (!(flexcan_read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_freeze(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = 1000 * 1000 * 10 / dev->baudrate;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_HALT;
	flexcan_write(reg, &regs->mcr);

	while (timeout-- && !(flexcan_read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(100);

	if (!(flexcan_read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_unfreeze(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = flexcan_read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_HALT;
	flexcan_write(reg, &regs->mcr);

	while (timeout-- && (flexcan_read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(10);

	if (flexcan_read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_softreset(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	flexcan_write(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	while (timeout-- && (flexcan_read(&regs->mcr) & FLEXCAN_MCR_SOFTRST))
		udelay(10);

	if (flexcan_read(&regs->mcr) & FLEXCAN_MCR_SOFTRST)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_start_xmit(struct rtcan_device *dev, struct can_frame *cf)
{
	const struct flexcan_priv *priv = rtcan_priv(dev);
	u32 can_id, data, ctrl;

	ctrl = FLEXCAN_MB_CODE_TX_DATA | (cf->can_dlc << 16);
	if (cf->can_id & CAN_EFF_FLAG) {
		can_id = cf->can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} else {
		can_id = (cf->can_id & CAN_SFF_MASK) << 18;
	}

	if (cf->can_id & CAN_RTR_FLAG)
		ctrl |= FLEXCAN_MB_CNT_RTR;

	if (cf->can_dlc > CAN_MAX_DLC)
		cf->can_dlc = CAN_MAX_DLC;

	if (cf->can_dlc > 0) {
		data = be32_to_cpup((__be32 *)&cf->data[0]);
		flexcan_write(data, &priv->tx_mb->data[0]);
	}
	if (cf->can_dlc > 4) {
		data = be32_to_cpup((__be32 *)&cf->data[4]);
		flexcan_write(data, &priv->tx_mb->data[1]);
	}

	flexcan_write(can_id, &priv->tx_mb->can_id);
	flexcan_write(ctrl, &priv->tx_mb->can_ctrl);

	/* Errata ERR005829 step8:
	 * Write twice INACTIVE(0x8) code to first MB.
	 */
	flexcan_write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb_reserved->can_ctrl);
	flexcan_write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb_reserved->can_ctrl);

	return 0;
}

static void init_err_skb(struct rtcan_skb *skb)
{
	struct rtcan_rb_frame *cf = &skb->rb_frame;

	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + CAN_ERR_DLC;
	cf->can_id = CAN_ERR_FLAG;
	cf->can_dlc = CAN_ERR_DLC;
	memset(&cf->data[0], 0, cf->can_dlc);
}

static void flexcan_irq_bus_err(struct rtcan_device *dev,
				u32 reg_esr, struct rtcan_skb *skb)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct rtcan_rb_frame *cf = &skb->rb_frame;

	init_err_skb(skb);

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
		rtcandev_dbg(dev, "BIT1_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
	}
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
		rtcandev_dbg(dev, "BIT0_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
	}
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		rtcandev_dbg(dev, "ACK_ERR irq\n");
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		rtcandev_dbg(dev, "CRC_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
		rtcandev_dbg(dev, "FRM_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
	}
	if (reg_esr & FLEXCAN_ESR_STF_ERR) {
		rtcandev_dbg(dev, "STF_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
	}

	priv->bus_errors++;
}

struct berr_counter {
	u16 txerr;
	u16 rxerr;
};

static void flexcan_change_state(struct rtcan_device *dev,
				 struct rtcan_rb_frame *cf,
				 struct berr_counter *bec,
				 can_state_t new_state)
{
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
			cf->data[1] = (bec->txerr > bec->rxerr) ?
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
			cf->data[1] = (bec->txerr > bec->rxerr) ?
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

	dev->state = new_state;
}

static bool flexcan_irq_state(struct rtcan_device *dev, u32 reg_esr,
			      struct rtcan_skb *skb)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	enum CAN_STATE new_state, rx_state, tx_state;
	struct rtcan_rb_frame *cf = &skb->rb_frame;
	struct berr_counter bec;
	u32 reg;
	int flt;

	reg = flexcan_read(&regs->ecr);
	bec.txerr = (reg >> 0) & 0xff;
	bec.rxerr = (reg >> 8) & 0xff;

	flt = reg_esr & FLEXCAN_ESR_FLT_CONF_MASK;
	if (likely(flt == FLEXCAN_ESR_FLT_CONF_ACTIVE)) {
		tx_state = unlikely(reg_esr & FLEXCAN_ESR_TX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		rx_state = unlikely(reg_esr & FLEXCAN_ESR_RX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		new_state = max(tx_state, rx_state);
	} else
		new_state = flt == FLEXCAN_ESR_FLT_CONF_PASSIVE ?
			CAN_STATE_ERROR_PASSIVE : CAN_STATE_BUS_OFF;

	/* state hasn't changed */
	if (likely(new_state == dev->state))
		return false;

	init_err_skb(skb);
	
	flexcan_change_state(dev, cf, &bec, new_state);

	return true;
}

static unsigned int flexcan_mailbox_read(struct rtcan_device *dev,
					 struct rtcan_skb *skb,
					 u32 *timestamp, unsigned int n)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	struct flexcan_mb __iomem *mb = &regs->mb[n];
	u32 reg_ctrl, reg_id, reg_iflag1, code;
	struct rtcan_rb_frame *cf = &skb->rb_frame;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		do {
			reg_ctrl = flexcan_read(&mb->can_ctrl);
		} while (reg_ctrl & FLEXCAN_MB_CODE_RX_BUSY_BIT);

		/* is this MB empty? */
		code = reg_ctrl & FLEXCAN_MB_CODE_MASK;
		if ((code != FLEXCAN_MB_CODE_RX_FULL) &&
		    (code != FLEXCAN_MB_CODE_RX_OVERRUN))
			return 0;
	} else {
		reg_iflag1 = flexcan_read(&regs->iflag1);
		if (!(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE))
			return 0;

		reg_ctrl = flexcan_read(&mb->can_ctrl);
	}

	/* increase timstamp to full 32 bit */
	*timestamp = reg_ctrl << 16;

	cf->can_dlc = get_can_dlc((reg_ctrl >> 16) & 0xf);
	reg_id = flexcan_read(&mb->can_id);
	if (reg_ctrl & FLEXCAN_MB_CNT_IDE)
		cf->can_id = ((reg_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = (reg_id >> 18) & CAN_SFF_MASK;

	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE;

	if (reg_ctrl & FLEXCAN_MB_CNT_RTR)
		cf->can_id |= CAN_RTR_FLAG;
	else
		skb->rb_frame_size += cf->can_dlc;

	put_unaligned_be32(flexcan_read(&mb->data[0]), cf->data + 0);
	put_unaligned_be32(flexcan_read(&mb->data[1]), cf->data + 4);

	cf->can_ifindex = dev->ifindex;

	/* mark as read */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		/* Clear IRQ */
		if (n < 32)
			flexcan_write(BIT(n), &regs->iflag1);
		else
			flexcan_write(BIT(n - 32), &regs->iflag2);
	} else {
		flexcan_write(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
		flexcan_read(&regs->timer);
	}

	return 1;
}

static inline bool flexcan_rx_le(struct flexcan_priv *priv, unsigned int a, unsigned int b)
{
	if (priv->mb_first < priv->mb_last)
		return a <= b;

	return a >= b;
}

static inline unsigned int flexcan_rx_inc(struct flexcan_priv *priv, unsigned int *val)
{
	if (priv->mb_first < priv->mb_last)
		return (*val)++;

	return (*val)--;
}

static int flexcan_mailbox_read_timestamp(struct rtcan_device *dev, u64 pending)
{
	struct flexcan_timestamped_frame *new, *pos, *tmp;
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct list_head q, *head;
	int i, count = 0;

	INIT_LIST_HEAD(&q);

	for (i = priv->mb_first;
	     flexcan_rx_le(priv, i, priv->mb_last);
	     flexcan_rx_inc(priv, &i)) {
		if (!(pending & BIT_ULL(i)))
			continue;

		new = priv->ts_frames + (i - priv->mb_first);
		if (!flexcan_mailbox_read(dev, &new->skb, &new->timestamp, i))
			break;

		head = &q;
		if (list_empty(&q))
			goto add;

		list_for_each_entry_reverse(pos, &q, next) {
			/*
			 * Substract two u32 and return result as int,
			 * to keep difference steady around the u32
			 * overflow.
			 */
			if (((int)(new->timestamp - pos->timestamp)) >= 0) {
				head = &pos->next;
				break;
			}
		}
	add:
		list_add(&new->next, head);
		count++;
	}

	if (list_empty(&q))
		return 0;

	list_for_each_entry_safe(pos, tmp, &q, next)
		rtcan_rcv(dev, &pos->skb);
	
	return count;
}

static void flexcan_mailbox_read_fifo(struct rtcan_device *dev)
{
	struct rtcan_skb skb;
	u32 timestamp;
	
	for (;;) {
		if (!flexcan_mailbox_read(dev, &skb, &timestamp, 0))
			break;
		rtcan_rcv(dev, &skb);
	}
}

static inline u64 flexcan_read_reg_iflag_rx(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 iflag1, iflag2;

	iflag2 = flexcan_read(&regs->iflag2) & priv->reg_imask2_default;
	iflag1 = flexcan_read(&regs->iflag1) & priv->reg_imask1_default &
		~FLEXCAN_IFLAG_MB(priv->tx_mb_idx);

	return (u64)iflag2 << 32 | iflag1;
}

static int flexcan_do_rx(struct rtcan_device *dev, u32 reg_iflag1)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	struct rtcan_skb skb;
	struct rtcan_rb_frame *cf = &skb.rb_frame;
	bool input = false;
	u64 reg;
	int ret;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		while ((reg = flexcan_read_reg_iflag_rx(priv))) {
			input = true;
			ret = flexcan_mailbox_read_timestamp(dev, reg);
			if (!ret)
				break;
		}
	} else {
		if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
			flexcan_write(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW, &regs->iflag1);
			init_err_skb(&skb);
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
			input = true;
		} else  if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
			flexcan_mailbox_read_fifo(dev);
			input = true;
		}
	}

	return input;
}

static int flexcan_irq(rtdm_irq_t *irq_handle)
{
	struct rtcan_device *dev = rtdm_irq_get_arg(irq_handle, void);
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_iflag1, reg_esr;
	struct rtcan_skb skb;
	int handled;

	rtdm_lock_get(&dev->device_lock);
	rtdm_lock_get(&rtcan_recv_list_lock);
	rtdm_lock_get(&rtcan_socket_lock);

	reg_iflag1 = flexcan_read(&regs->iflag1);

	/* reception interrupt */
	if (flexcan_do_rx(dev, reg_iflag1))
		handled = RTDM_IRQ_HANDLED;

	/* transmission complete interrupt */
	if (reg_iflag1 & FLEXCAN_IFLAG_MB(priv->tx_mb_idx)) {
		/* after sending a RTR frame MB is in RX mode */
		flexcan_write(FLEXCAN_MB_CODE_TX_INACTIVE,
			      &priv->tx_mb->can_ctrl);
		flexcan_write(FLEXCAN_IFLAG_MB(priv->tx_mb_idx), &regs->iflag1);
		rtdm_sem_up(&dev->tx_sem);
		if (rtcan_loopback_pending(dev))
			rtcan_loopback(dev);
		handled = RTDM_IRQ_HANDLED;
	}

	reg_esr = flexcan_read(&regs->esr);

	/* ACK all bus error and state change IRQ sources */
	if (reg_esr & FLEXCAN_ESR_ALL_INT) {
		flexcan_write(reg_esr & FLEXCAN_ESR_ALL_INT, &regs->esr);
		handled = RTDM_IRQ_HANDLED;
	}

	/* state change interrupt or broken error state quirk fix is enabled */
	if (reg_esr & FLEXCAN_ESR_ERR_STATE)
		handled = RTDM_IRQ_HANDLED;
	else if (priv->devtype_data->quirks & (FLEXCAN_QUIRK_BROKEN_WERR_STATE |
					       FLEXCAN_QUIRK_BROKEN_PERR_STATE))
		goto esr_err;
	
	if (reg_esr & FLEXCAN_ESR_ERR_STATE) {
	esr_err:
		if (flexcan_irq_state(dev, reg_esr, &skb)) {
			rtcan_rcv(dev, &skb);
		}
	}

	/* bus error IRQ - report unconditionally */
	if (reg_esr & FLEXCAN_ESR_ERR_BUS) {
		flexcan_irq_bus_err(dev, reg_esr, &skb);
		rtcan_rcv(dev, &skb);
		handled = RTDM_IRQ_HANDLED;
	}

	rtdm_lock_put(&rtcan_socket_lock);
	rtdm_lock_put(&rtcan_recv_list_lock);
	rtdm_lock_put(&dev->device_lock);

	return handled;
}

static void flexcan_set_bittiming(struct rtcan_device *dev)
{
	const struct flexcan_priv *priv = rtcan_priv(dev);
	const struct can_bittime *bt = &priv->bittiming;
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg;

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

	rtcandev_dbg(dev, "writing ctrl=0x%08x\n", reg);
	flexcan_write(reg, &regs->ctrl);

	/* print chip status */
	rtcandev_dbg(dev, "%s: mcr=0x%08x ctrl=0x%08x\n", __func__,
		   flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));
}

/* flexcan_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int flexcan_chip_start(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr, reg_ctrl, reg_ctrl2, reg_mecr;
	int err, i;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	/* enable module */
	err = flexcan_chip_enable(priv);
	if (err)
		goto out_disable_per;

	/* soft reset */
	err = flexcan_chip_softreset(priv);
	if (err)
		goto out_chip_disable;

	flexcan_set_bittiming(dev);

	/* MCR
	 *
	 * enable freeze
	 * enable fifo
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * disable local echo
	 * enable individual RX masking
	 * choose format C
	 * set max mailbox number
	 */
	reg_mcr = flexcan_read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_MAXMB(0xff);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV |
		FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_SRX_DIS | FLEXCAN_MCR_IRMQ |
		FLEXCAN_MCR_IDAM_C;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		reg_mcr &= ~FLEXCAN_MCR_FEN;
		reg_mcr |= FLEXCAN_MCR_MAXMB(priv->mb_last);
	} else {
		reg_mcr |= FLEXCAN_MCR_FEN |
			FLEXCAN_MCR_MAXMB(priv->tx_mb_idx);
	}
	rtcandev_dbg(dev, "%s: writing mcr=0x%08x", __func__, reg_mcr);
	flexcan_write(reg_mcr, &regs->mcr);

	/* CTRL
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

	/* enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_BROKEN_WERR_STATE)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;
	else
		reg_ctrl &= ~FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	/* leave interrupts disabled for now */
	reg_ctrl &= ~FLEXCAN_CTRL_ERR_ALL;
	rtcandev_dbg(dev, "%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	flexcan_write(reg_ctrl, &regs->ctrl);

	if ((priv->devtype_data->quirks & FLEXCAN_QUIRK_ENABLE_EACEN_RRS)) {
		reg_ctrl2 = flexcan_read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_EACEN | FLEXCAN_CTRL2_RRS;
		flexcan_write(reg_ctrl2, &regs->ctrl2);
	}

	/* clear and invalidate all mailboxes first */
	for (i = priv->tx_mb_idx; i < ARRAY_SIZE(regs->mb); i++) {
		flexcan_write(FLEXCAN_MB_CODE_RX_INACTIVE,
			      &regs->mb[i].can_ctrl);
	}

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		for (i = priv->mb_first; i <= priv->mb_last; i++)
			flexcan_write(FLEXCAN_MB_CODE_RX_EMPTY,
				      &regs->mb[i].can_ctrl);
	}

	/* Errata ERR005829: mark first TX mailbox as INACTIVE */
	flexcan_write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb_reserved->can_ctrl);

	/* mark TX mailbox as INACTIVE */
	flexcan_write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb->can_ctrl);

	/* acceptance mask/acceptance code (accept everything) */
	flexcan_write(0x0, &regs->rxgmask);
	flexcan_write(0x0, &regs->rx14mask);
	flexcan_write(0x0, &regs->rx15mask);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		flexcan_write(0x0, &regs->rxfgmask);

	/* clear acceptance filters */
	for (i = 0; i < ARRAY_SIZE(regs->mb); i++)
		flexcan_write(0, &regs->rximr[i]);

	/* On Vybrid, disable memory error detection interrupts
	 * and freeze mode.
	 * This also works around errata e5295 which generates
	 * false positive memory errors and put the device in
	 * freeze mode.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_MECR) {
		/* Follow the protocol as described in "Detection
		 * and Correction of Memory Errors" to write to
		 * MECR register
		 */
		reg_ctrl2 = flexcan_read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_ECRWRE;
		flexcan_write(reg_ctrl2, &regs->ctrl2);

		reg_mecr = flexcan_read(&regs->mecr);
		reg_mecr &= ~FLEXCAN_MECR_ECRWRDIS;
		flexcan_write(reg_mecr, &regs->mecr);
		reg_mecr &= ~(FLEXCAN_MECR_NCEFAFRZ | FLEXCAN_MECR_HANCEI_MSK |
			      FLEXCAN_MECR_FANCEI_MSK);
		flexcan_write(reg_mecr, &regs->mecr);
	}

	err = flexcan_transceiver_enable(priv);
	if (err)
		goto out_chip_disable;

	/* synchronize with the can bus */
	err = flexcan_chip_unfreeze(priv);
	if (err)
		goto out_transceiver_disable;

	dev->state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts atomically */
	rtdm_irq_disable(&dev->irq_handle);
	flexcan_write(priv->reg_ctrl_default, &regs->ctrl);
	flexcan_write(priv->reg_imask1_default, &regs->imask1);
	flexcan_write(priv->reg_imask2_default, &regs->imask2);
	rtdm_irq_enable(&dev->irq_handle);

	/* print chip status */
	rtcandev_dbg(dev, "%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		   flexcan_read(&regs->mcr), flexcan_read(&regs->ctrl));

	return 0;

 out_transceiver_disable:
	flexcan_transceiver_disable(priv);
 out_chip_disable:
	flexcan_chip_disable(priv);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

/* flexcan_chip_stop
 *
 * this functions is entered with clocks enabled
 */
static void flexcan_chip_stop(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;

	/* freeze + disable module */
	flexcan_chip_freeze(dev);
	flexcan_chip_disable(priv);

	/* Disable all interrupts */
	flexcan_write(0, &regs->imask2);
	flexcan_write(0, &regs->imask1);
	flexcan_write(priv->reg_ctrl_default & ~FLEXCAN_CTRL_ERR_ALL,
		      &regs->ctrl);

	flexcan_transceiver_disable(priv);

	clk_disable_unprepare(priv->clk_per);
	clk_disable_unprepare(priv->clk_ipg);
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
		/* Register IRQ handler and pass device structure as arg */
		err = rtdm_irq_request(&dev->irq_handle, priv->irq,
				       flexcan_irq, 0, DRV_NAME,
				       dev);
		if (err) {
			rtcandev_err(dev, "couldn't request irq %d\n",
				     priv->irq);
			goto out;
		}

		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);

		/* start chip and queuing */
		err = flexcan_chip_start(dev);
		if (err) {
			rtdm_irq_free(&dev->irq_handle);
			rtdm_sem_destroy(&dev->tx_sem);
			goto out;
		}
		break;

	case CAN_STATE_BUS_OFF:
		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);
		/* start chip and queuing */
		err = flexcan_chip_start(dev);
		if (err) {
			rtdm_sem_destroy(&dev->tx_sem);
			goto out;
		}
		break;

	case CAN_STATE_SLEEPING:
	default:
		err = 0;
		break;
	}

out:
	rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);

	return err;
}

static int flexcan_mode_stop(struct rtcan_device *dev,
			     rtdm_lockctx_t *lock_ctx)
{
	if (!CAN_STATE_OPERATING(dev->state))
		return 0;

	dev->state = CAN_STATE_STOPPED;

	rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);

	flexcan_chip_stop(dev);
	rtdm_irq_free(&dev->irq_handle);
	rtdm_sem_destroy(&dev->tx_sem);

	rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);

	return 0;
}

static int flexcan_set_mode(struct rtcan_device *dev, can_mode_t mode,
			    rtdm_lockctx_t *lock_ctx)
{
	if (mode == CAN_MODE_START)
		return flexcan_mode_start(dev, lock_ctx);

	if (mode == CAN_MODE_STOP)
		return flexcan_mode_stop(dev, lock_ctx);

	return -EOPNOTSUPP;
}

static int flexcan_copy_bittiming(struct rtcan_device *dev,
				  struct can_bittime *bt,
				  rtdm_lockctx_t *lock_ctx)
{
	struct flexcan_priv *priv = rtcan_priv(dev);

	memcpy(&priv->bittiming, bt, sizeof(*bt));

	return 0;
}

static int register_flexcandev(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg, err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	/* select "bus clock", chip must be disabled */
	err = flexcan_chip_disable(priv);
	if (err)
		goto out_disable_per;
	reg = flexcan_read(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	flexcan_write(reg, &regs->ctrl);

	err = flexcan_chip_enable(priv);
	if (err)
		goto out_chip_disable;

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = flexcan_read(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	flexcan_write(reg, &regs->mcr);

	/* Currently we only support newer versions of this core
	 * featuring a RX hardware FIFO (although this driver doesn't
	 * make use of it on some cores). Older cores, found on some
	 * Coldfire derivates are not tested.
	 */
	reg = flexcan_read(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		rtcandev_err(dev, "Could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out_chip_disable;
	}

	err = rtcan_dev_register(dev);

	/* disable core and turn off clocks */
 out_chip_disable:
	flexcan_chip_disable(priv);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static void unregister_flexcandev(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);

	rtcan_dev_unregister(dev);
	if (priv->ts_frames)
		kfree(priv->ts_frames);
}

static const struct of_device_id flexcan_of_match[] = {
	{ .compatible = "fsl,imx6q-flexcan", .data = &fsl_imx6q_devtype_data, },
	{ .compatible = "fsl,imx28-flexcan", .data = &fsl_imx28_devtype_data, },
	{ .compatible = "fsl,p1010-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,vf610-flexcan", .data = &fsl_vf610_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, flexcan_of_match);

static const struct platform_device_id flexcan_id_table[] = {
	{ .name = "flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, flexcan_id_table);

static int flexcan_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	const struct flexcan_devtype_data *devtype_data;
	struct rtcan_device *dev;
	struct flexcan_priv *priv;
	struct regulator *reg_xceiver;
	struct resource *mem;
	struct clk *clk_ipg = NULL, *clk_per = NULL;
	struct flexcan_regs __iomem *regs;
	int err, irq;
	u32 clock_freq = 0;

	reg_xceiver = devm_regulator_get(&pdev->dev, "xceiver");
	if (PTR_ERR(reg_xceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (IS_ERR(reg_xceiver))
		reg_xceiver = NULL;

	if (pdev->dev.of_node)
		of_property_read_u32(pdev->dev.of_node,
				     "clock-frequency", &clock_freq);

	if (!clock_freq) {
		clk_ipg = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(clk_ipg)) {
			dev_err(&pdev->dev, "no ipg clock defined\n");
			return PTR_ERR(clk_ipg);
		}

		clk_per = devm_clk_get(&pdev->dev, "per");
		if (IS_ERR(clk_per)) {
			dev_err(&pdev->dev, "no per clock defined\n");
			return PTR_ERR(clk_per);
		}
		clock_freq = clk_get_rate(clk_per);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	of_id = of_match_device(flexcan_of_match, &pdev->dev);
	if (of_id) {
		devtype_data = of_id->data;
	} else if (platform_get_device_id(pdev)->driver_data) {
		devtype_data = (struct flexcan_devtype_data *)
			platform_get_device_id(pdev)->driver_data;
	} else {
		return -ENODEV;
	}

	dev = rtcan_dev_alloc(sizeof(struct flexcan_priv), 0);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);

	priv = rtcan_priv(dev);
	priv->regs = regs;
	priv->irq = irq;
	priv->clk_ipg = clk_ipg;
	priv->clk_per = clk_per;
	priv->devtype_data = devtype_data;
	priv->reg_xceiver = reg_xceiver;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		priv->tx_mb_idx = FLEXCAN_TX_MB_OFF_TIMESTAMP;
		priv->tx_mb_reserved = &regs->mb[FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP];
	} else {
		priv->tx_mb_idx = FLEXCAN_TX_MB_OFF_FIFO;
		priv->tx_mb_reserved = &regs->mb[FLEXCAN_TX_MB_RESERVED_OFF_FIFO];
	}
	priv->tx_mb = &regs->mb[priv->tx_mb_idx];

	priv->reg_imask1_default = FLEXCAN_IFLAG_MB(priv->tx_mb_idx);
	priv->reg_imask2_default = 0;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		u64 imask;

		priv->mb_first = FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST;
		priv->mb_last = FLEXCAN_RX_MB_OFF_TIMESTAMP_LAST;
		priv->ts_frames = kzalloc(sizeof(*priv->ts_frames) *
					  FLEXCAN_RX_MB_TIMESTAMP_COUNT, GFP_KERNEL);
		if (priv->ts_frames == NULL) {
			err = -ENOMEM;
			goto failed_fralloc;
		}

		imask = GENMASK_ULL(priv->mb_last, priv->mb_first);
		priv->reg_imask1_default |= imask;
		priv->reg_imask2_default |= imask >> 32;
	} else {
		priv->reg_imask1_default |= FLEXCAN_IFLAG_RX_FIFO_OVERFLOW |
			FLEXCAN_IFLAG_RX_FIFO_AVAILABLE;
		priv->ts_frames = NULL;
	}

	dev->ctrl_name = "FLEXCAN";
	dev->board_name = "FLEXCAN";
	dev->base_addr = (unsigned long)regs;
	dev->can_sys_clock = clock_freq;
	dev->hard_start_xmit = flexcan_start_xmit;
	dev->do_set_mode = flexcan_set_mode;
	dev->do_set_bit_time = flexcan_copy_bittiming;
	dev->bittiming_const = &flexcan_bittiming_const;
	dev->state = CAN_STATE_STOPPED;
	strncpy(dev->name, DEV_NAME, IFNAMSIZ);
	
	err = register_flexcandev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed\n");
		goto failed_register;
	}

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d)\n",
		 priv->regs, priv->irq);

	return 0;

 failed_register:
	if (priv->ts_frames)
		kfree(priv->ts_frames);
 failed_fralloc:
	rtcan_dev_free(dev);
	return err;
}

static int flexcan_remove(struct platform_device *pdev)
{
	struct rtcan_device *dev = platform_get_drvdata(pdev);

	unregister_flexcandev(dev);
	rtcan_dev_free(dev);

	return 0;
}

static struct platform_driver flexcan_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = flexcan_of_match,
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.id_table = flexcan_id_table,
};

module_platform_driver(flexcan_driver);

MODULE_AUTHOR("Wolfgang Grandegger <wg@denx.de>, "
	      "Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RT-CAN port driver for flexcan based chip");
