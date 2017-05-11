/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Based on linux-2.4.25/include/asm-ppc/mpc5xxx.h
 * Prototypes, etc. for the Motorola MPC5xxx embedded cpu chips
 *
 * Author: Dale Farnsworth <dfarnsworth@mvista.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __RTCAN_MSCAN_REGS_H_
#define __RTCAN_MSCAN_REGS_H_

#include <linux/version.h>
#include <linux/of_platform.h>
#include <asm/mpc52xx.h>

static inline void __iomem *mpc5xxx_gpio_find_and_map(void)
{
	struct device_node *ofn;
	ofn = of_find_compatible_node(NULL, NULL, "mpc5200-gpio");
	if (!ofn)
		ofn = of_find_compatible_node(NULL, NULL, "fsl,mpc5200-gpio");
	return ofn ? of_iomap(ofn, 0) : NULL;
}

#define MPC5xxx_GPIO	mpc5xxx_gpio_find_and_map()
#define mpc5xxx_gpio	mpc52xx_gpio

#define mpc5xxx_get_of_node(ofdev) (ofdev)->dev.of_node

#define MSCAN_CAN1_ADDR	(MSCAN_MBAR + 0x0900) /* MSCAN Module 1 */
#define MSCAN_CAN2_ADDR	(MSCAN_MBAR + 0x0980) /* MSCAN Module 2 */
#define MSCAN_SIZE	0x80

/* MSCAN control register 0 (CANCTL0) bits */
#define MSCAN_RXFRM	0x80
#define MSCAN_RXACT	0x40
#define MSCAN_CSWAI	0x20
#define MSCAN_SYNCH	0x10
#define MSCAN_TIME	0x08
#define MSCAN_WUPE	0x04
#define MSCAN_SLPRQ	0x02
#define MSCAN_INITRQ	0x01

/* MSCAN control register 1 (CANCTL1) bits */
#define MSCAN_CANE	0x80
#define MSCAN_CLKSRC	0x40
#define MSCAN_LOOPB	0x20
#define MSCAN_LISTEN	0x10
#define MSCAN_WUPM	0x04
#define MSCAN_SLPAK	0x02
#define MSCAN_INITAK	0x01

/* MSCAN receiver flag register (CANRFLG) bits */
#define MSCAN_WUPIF	0x80
#define MSCAN_CSCIF	0x40
#define MSCAN_RSTAT1	0x20
#define MSCAN_RSTAT0	0x10
#define MSCAN_TSTAT1	0x08
#define MSCAN_TSTAT0	0x04
#define MSCAN_OVRIF	0x02
#define MSCAN_RXF	0x01

/* MSCAN receiver interrupt enable register (CANRIER) bits */
#define MSCAN_WUPIE	0x80
#define MSCAN_CSCIE	0x40
#define MSCAN_RSTATE1	0x20
#define MSCAN_RSTATE0	0x10
#define MSCAN_TSTATE1	0x08
#define MSCAN_TSTATE0	0x04
#define MSCAN_OVRIE	0x02
#define MSCAN_RXFIE	0x01

/* MSCAN transmitter flag register (CANTFLG) bits */
#define MSCAN_TXE2	0x04
#define MSCAN_TXE1	0x02
#define MSCAN_TXE0	0x01
#define MSCAN_TXE	(MSCAN_TXE2 | MSCAN_TXE1 | MSCAN_TXE0)

/* MSCAN transmitter interrupt enable register (CANTIER) bits */
#define MSCAN_TXIE2	0x04
#define MSCAN_TXIE1	0x02
#define MSCAN_TXIE0	0x01
#define MSCAN_TXIE	(MSCAN_TXIE2 | MSCAN_TXIE1 | MSCAN_TXIE0)

/* MSCAN transmitter message abort request (CANTARQ) bits */
#define MSCAN_ABTRQ2	0x04
#define MSCAN_ABTRQ1	0x02
#define MSCAN_ABTRQ0	0x01

/* MSCAN transmitter message abort ack (CANTAAK) bits */
#define MSCAN_ABTAK2	0x04
#define MSCAN_ABTAK1	0x02
#define MSCAN_ABTAK0	0x01

/* MSCAN transmit buffer selection (CANTBSEL) bits */
#define MSCAN_TX2	0x04
#define MSCAN_TX1	0x02
#define MSCAN_TX0	0x01

/* MSCAN ID acceptance control register (CANIDAC) bits */
#define MSCAN_IDAM1	0x20
#define MSCAN_IDAM0	0x10
#define MSCAN_IDHIT2	0x04
#define MSCAN_IDHIT1	0x02
#define MSCAN_IDHIT0	0x01

struct mscan_msgbuf {
	volatile u8  idr[0x8];		/* 0x00 */
	volatile u8  dsr[0x10];		/* 0x08 */
	volatile u8  dlr;		/* 0x18 */
	volatile u8  tbpr;		/* 0x19 */	/* This register is not applicable for receive buffers */
	volatile u16 rsrv1;		/* 0x1A */
	volatile u8  tsrh;		/* 0x1C */
	volatile u8  tsrl;		/* 0x1D */
	volatile u16 rsrv2;		/* 0x1E */
};

struct mscan_regs {
	volatile u8  canctl0;		/* MSCAN + 0x00 */
	volatile u8  canctl1;		/* MSCAN + 0x01 */
	volatile u16 rsrv1;		/* MSCAN + 0x02 */
	volatile u8  canbtr0;		/* MSCAN + 0x04 */
	volatile u8  canbtr1;		/* MSCAN + 0x05 */
	volatile u16 rsrv2;		/* MSCAN + 0x06 */
	volatile u8  canrflg;		/* MSCAN + 0x08 */
	volatile u8  canrier;		/* MSCAN + 0x09 */
	volatile u16 rsrv3;		/* MSCAN + 0x0A */
	volatile u8  cantflg;		/* MSCAN + 0x0C */
	volatile u8  cantier;		/* MSCAN + 0x0D */
	volatile u16 rsrv4;		/* MSCAN + 0x0E */
	volatile u8  cantarq;		/* MSCAN + 0x10 */
	volatile u8  cantaak;		/* MSCAN + 0x11 */
	volatile u16 rsrv5;		/* MSCAN + 0x12 */
	volatile u8  cantbsel;		/* MSCAN + 0x14 */
	volatile u8  canidac;		/* MSCAN + 0x15 */
	volatile u16 rsrv6[3];		/* MSCAN + 0x16 */
	volatile u8  canrxerr;		/* MSCAN + 0x1C */
	volatile u8  cantxerr;		/* MSCAN + 0x1D */
	volatile u16 rsrv7;		/* MSCAN + 0x1E */
	volatile u8  canidar0;		/* MSCAN + 0x20 */
	volatile u8  canidar1;		/* MSCAN + 0x21 */
	volatile u16 rsrv8;		/* MSCAN + 0x22 */
	volatile u8  canidar2;		/* MSCAN + 0x24 */
	volatile u8  canidar3;		/* MSCAN + 0x25 */
	volatile u16 rsrv9;		/* MSCAN + 0x26 */
	volatile u8  canidmr0;		/* MSCAN + 0x28 */
	volatile u8  canidmr1;		/* MSCAN + 0x29 */
	volatile u16 rsrv10;		/* MSCAN + 0x2A */
	volatile u8  canidmr2;		/* MSCAN + 0x2C */
	volatile u8  canidmr3;		/* MSCAN + 0x2D */
	volatile u16 rsrv11;		/* MSCAN + 0x2E */
	volatile u8  canidar4;		/* MSCAN + 0x30 */
	volatile u8  canidar5;		/* MSCAN + 0x31 */
	volatile u16 rsrv12;		/* MSCAN + 0x32 */
	volatile u8  canidar6;		/* MSCAN + 0x34 */
	volatile u8  canidar7;		/* MSCAN + 0x35 */
	volatile u16 rsrv13;		/* MSCAN + 0x36 */
	volatile u8  canidmr4;		/* MSCAN + 0x38 */
	volatile u8  canidmr5;		/* MSCAN + 0x39 */
	volatile u16 rsrv14;		/* MSCAN + 0x3A */
	volatile u8  canidmr6;		/* MSCAN + 0x3C */
	volatile u8  canidmr7;		/* MSCAN + 0x3D */
	volatile u16 rsrv15;		/* MSCAN + 0x3E */

	struct mscan_msgbuf canrxfg;	/* MSCAN + 0x40 */    /* Foreground receive buffer */
	struct mscan_msgbuf cantxfg;	/* MSCAN + 0x60 */    /* Foreground transmit buffer */
};

/* Clock source selection
 */
#define MSCAN_CLKSRC_BUS	0
#define MSCAN_CLKSRC_XTAL	MSCAN_CLKSRC
#define MSCAN_CLKSRC_IPS	MSCAN_CLKSRC

/* Message type access macros.
 */
#define MSCAN_BUF_STD_RTR	0x10
#define MSCAN_BUF_EXT_RTR	0x01
#define MSCAN_BUF_EXTENDED	0x08

#define MSCAN_IDAM1		0x20
/* Value for the interrupt enable register */
#define MSCAN_RIER		(MSCAN_OVRIE |		\
				 MSCAN_RXFIE |		\
				 MSCAN_WUPIF |		\
				 MSCAN_CSCIE |		\
				 MSCAN_RSTATE0 |	\
				 MSCAN_RSTATE1 |	\
				 MSCAN_TSTATE0 |	\
				 MSCAN_TSTATE1)

#define BTR0_BRP_MASK		0x3f
#define BTR0_SJW_SHIFT		6
#define BTR0_SJW_MASK		(0x3 << BTR0_SJW_SHIFT)

#define BTR1_TSEG1_MASK		0xf
#define BTR1_TSEG2_SHIFT	4
#define BTR1_TSEG2_MASK		(0x7 << BTR1_TSEG2_SHIFT)
#define BTR1_SAM_SHIFT		7

#define BTR0_SET_BRP(brp)	(((brp) - 1) & BTR0_BRP_MASK)
#define BTR0_SET_SJW(sjw)	((((sjw) - 1) << BTR0_SJW_SHIFT) & \
				 BTR0_SJW_MASK)

#define BTR1_SET_TSEG1(tseg1)	(((tseg1) - 1) & BTR1_TSEG1_MASK)
#define BTR1_SET_TSEG2(tseg2)	((((tseg2) - 1) << BTR1_TSEG2_SHIFT) & \
				 BTR1_TSEG2_MASK)
#define BTR1_SET_SAM(sam)	(((sam) & 1) << BTR1_SAM_SHIFT)

#endif /* __RTCAN_MSCAN_REGS_H_ */
