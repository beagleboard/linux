/*
 * CAN bus driver for Bosch D_CAN controller
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Borrowed from C_CAN driver
 * Copyright (C) 2010 ST Microelectronics
 * - Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the C_CAN driver originally written by:
 * Copyright (C) 2007
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 *
 * Bosch D_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch D_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/media/en/pdf/ipmodules_1/can/
 * d_can_users_manual_111.pdf
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/list.h>
#include <linux/io.h>

#include <linux/platform_device.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#include "d_can.h"

/* TI D_CAN module registers */
#define D_CAN_CTL		0x0	/* CAN control register */
#define D_CAN_ES		0x4	/* Error and status */
#define D_CAN_PARITYERR_EOI	0x4	/* Parity error EOI */
#define D_CAN_ERRC		0x8	/* Error counter */
#define D_CAN_BTR		0xC	/* Bit timing */
#define D_CAN_INT		0x10	/* Interrupt register */
#define D_CAN_TEST		0x14	/* Test register */
#define D_CAN_PERR		0x1C	/* Parity Error Code */
#define D_CAN_ABOTR		0x80	/* Auto-Bus-On Time */
#define D_CAN_TXRQ_X		0x84	/* Transmission Request X */
#define D_CAN_TXRQ(n)		(0x88 + ((n) * 4)) /* Transmission request */
#define D_CAN_NWDAT_X		0x98	/* New data X register */
#define D_CAN_NWDAT(n)		(0x9C + ((n) * 4)) /* New data */
#define D_CAN_INTPND_X		0xAC	/* Interrupt Pending X */
#define D_CAN_INTPND(n)		(0xB0 + ((n) * 4)) /* Interrupt Pending */
#define D_CAN_MSGVAL_X		0xC0		/* Message Valid X */
#define D_CAN_MSGVAL(n)		(0xC4 + ((n) * 4)) /* Message Valid */
#define D_CAN_INTMUX(n)		(0xD8 + ((n) * 4)) /* Interrupt Multiplexer */
#define D_CAN_IFCMD(n)		(0x100 + ((n) * 0x20)) /* Command */
#define D_CAN_IFMSK(n)		(0x104 + ((n) * 0x20)) /* Mask */
#define D_CAN_IFARB(n)		(0x108 + ((n) * 0x20)) /* Arbitration */
#define D_CAN_IFMCTL(n)		(0x10c + ((n) * 0x20)) /* Message ctl */
#define D_CAN_IFDATA(n)		(0x110 + ((n) * 0x20)) /* DATA A */
#define D_CAN_IFDATB(n)		(0x114 + ((n) * 0x20)) /* DATA B */
#define D_CAN_IF3OBS		0x140	/* IF3 Observation */
#define D_CAN_IF3UPD(n)		(0x160 + ((n) * 4)) /* Update enable */
#define D_CAN_TIOC		0x1E0	/* CAN TX IO Control */
#define D_CAN_RIOC		0x1E4	/* CAN RX IO Control */

/* Control register Bit fields */
#define D_CAN_CTL_WUBA		BIT(26)	/* Automatic wake-up on bus activity */
#define D_CAN_CTL_PDR		BIT(24)	/* Request for local low power mode */
#define D_CAN_CTL_DE3		BIT(20)	/* Enable DMA request line for IF3 */
#define D_CAN_CTL_DE2		BIT(19)	/* Enable DMA request line for IF2 */
#define D_CAN_CTL_DE1		BIT(18)	/* Enable DMA request line for IF1 */
#define D_CAN_CTL_IE1		BIT(17)	/* Interrupt line 1 enable */
#define D_CAN_CTL_INITDBG	BIT(16)	/* Init state for debug access */
#define D_CAN_CTL_SWR		BIT(15)	/* S/W reset enable */
#define D_CAN_CTL_PMD		(0xF << 10)	/* Parity on/off */
#define D_CAN_CTL_ABO		BIT(9)	/* Auto bus on enable */
#define D_CAN_CTL_IDS		BIT(8)	/* Interruption debug support enable */
#define D_CAN_CTL_TEST		BIT(7)	/* Test mode enable */
#define D_CAN_CTL_CCE		BIT(6)	/* Configuration change enable */
#define D_CAN_CTL_DISABLE_AR	BIT(5)	/* Disable automatic retransmission */
#define D_CAN_CTL_ENABLE_AR	(0 << 5)
#define D_CAN_CTL_EIE		BIT(3)	/* Error interrupt enable */
#define D_CAN_CTL_SIE		BIT(2)	/* Status change int enable */
#define D_CAN_CTL_IE0		BIT(1)	/* Interrupt line 0 enable */
#define D_CAN_CTL_INIT		BIT(0)	/* D_CAN initialization mode */

/* D_CAN Error and Status and Parity Error EOI reg bit fields */
#define D_CAN_ES_PDA		BIT(10)	/* Local power-down ACK */
#define D_CAN_ES_WUP		BIT(9)	/* Wkae up pending */
#define D_CAN_ES_PER		BIT(8)	/* Parity error detected */
#define D_CAN_ES_BOFF		BIT(7)	/* Bus off state */
#define D_CAN_ES_EWARN		BIT(6)	/* Warning state */
#define D_CAN_ES_EPASS		BIT(5)	/* Error passive state */
#define D_CAN_ES_RXOK		BIT(4)	/* Received a msg successfully */
#define D_CAN_ES_TXOK		BIT(3)	/* Transmitted a msg successfully */
#define D_CAN_ES_LEC_MASK	0x7	/* Last error code */

/* Parity error reg bit fields */
#define D_CAN_PEEOI		BIT(8)	/* EOI indication for parity error */

/* Error counter reg bit fields */
#define D_CAN_ERRC_RP_SHIFT	15
#define D_CAN_ERRC_RP_MASK	BIT(15)		/* Receive error passive */
#define D_CAN_ERRC_REC_SHIFT	8
#define D_CAN_ERRC_REC_MASK	(0x7F << 8)	/* Receive err counter */
#define D_CAN_ERRC_TEC_SHIFT	0
#define D_CAN_ERRC_TEC_MASK	(0xFF << 0)	/* Transmit err counter */

/* Bit timing reg bit fields */
#define D_CAN_BTR_BRPE_SHIFT	16
#define D_CAN_BTR_BRPE_MASK	(0xF << 16)	/* Baud rate prescaler ext */
#define D_CAN_BTR_TSEG2_SHIFT	12
#define D_CAN_BTR_TSEG2_MASK	(0x7 << 12)	/* Time seg after smpl point */
#define D_CAN_BTR_TSEG1_SHIFT	8
#define D_CAN_BTR_TSEG1_MASK	(0xF << 8)	/* Time seg before smpl point */
#define D_CAN_BTR_SJW_SHIFT	6
#define D_CAN_BTR_SJW_MASK	(0x3 << 6)	/* Syncronization jump width */
#define D_CAN_BTR_BRP_SHIFT	0
#define D_CAN_BTR_BRP_MASK	(0x3F << 0)	/* Baud rate prescaler */

/* D_CAN Test register bit fields */
#define D_CAN_TEST_RDA		BIT(9)	/* RAM direct access enable */
#define D_CAN_TEST_EXL		BIT(8)	/* External loopback mode */
#define D_CAN_TEST_RX		BIT(7)	/* Monitors the reveive pin */
#define D_CAN_TEST_TX		(0x3 << 5)	/* Control of CAN_TX pin */
#define D_CAN_TEST_LBACK	BIT(4)	/* Loopback mode */
#define D_CAN_TEST_SILENT	BIT(3)	/* Silent mdoe */

/* D_CAN Parity error reg bit fields */
#define D_CAN_PERR_WN_MASK	(0x7 << 8)	/* Parity error word nuber */
#define D_CAN_PERR_MN_MASK	0xFF		/* Parity error msg object */

/* D_CAN X registers bit fields */
#define D_CAN_BIT_FIELD(n)	(0x3 << (2 * n)) /* X reg's bit field 1 mask */

/* D_CAN IF command reg bit fields */
#define D_CAN_IF_CMD_WR		BIT(23)	/* Write/read */
#define D_CAN_IF_CMD_MASK	BIT(22)	/* Access to mask bits */
#define D_CAN_IF_CMD_ARB	BIT(21)	/* Access to arbitration bits */
#define D_CAN_IF_CMD_CONTROL	BIT(20)	/* Acess to control bits */
#define D_CAN_IF_CMD_CIP	BIT(19)	/* Clear int pending */
#define D_CAN_IF_CMD_TXRQST	BIT(18)	/* Access transmission request */
#define D_CAN_IF_CMD_DATAA	BIT(17)	/* Access to Data Bytes 0-3 */
#define D_CAN_IF_CMD_DATAB	BIT(16)	/* Access to Data Bytes 4-7 */
#define D_CAN_IF_CMD_BUSY	BIT(15)	/* Busy flag */
#define D_CAN_IF_CMD_DAM	BIT(14)	/* Activation of DMA */
#define D_CAN_IF_CMD_MN_MASK	0xFF	/* No. of msg's used for DMA T/F */
#define D_CAN_IF_CMD_ALL	(D_CAN_IF_CMD_MASK | D_CAN_IF_CMD_ARB | \
				D_CAN_IF_CMD_CONTROL | D_CAN_IF_CMD_TXRQST | \
				D_CAN_IF_CMD_DATAA | D_CAN_IF_CMD_DATAB)

/* D_CAN IF mask reg bit fields */
#define D_CAN_IF_MASK_MX	BIT(31)	/* Mask Extended Identifier */
#define D_CAN_IF_MASK_MD	BIT(30)	/* Mask Message direction */

/* D_CAN IF Arbitration */
#define D_CAN_IF_ARB_MSGVAL	BIT(31)	/* Message Vaild */
#define D_CAN_IF_ARB_MSGXTD	BIT(30)	/* Extended Identifier 0-11 1-29 */
#define D_CAN_IF_ARB_DIR_XMIT	BIT(29) /* Message direction 0-R 1-T */

/* D_CAN IF Message control */
#define D_CAN_IF_MCTL_NEWDAT	BIT(15)	/* New data available */
#define D_CAN_IF_MCTL_MSGLST	BIT(14)	/* Message lost, only for receive */
#define D_CAN_IF_MCTL_CLR_MSGLST (0 << 14)
#define D_CAN_IF_MCTL_INTPND	BIT(13)	/* Interrupt pending */
#define D_CAN_IF_MCTL_UMASK	BIT(12)	/* Use acceptance mask */
#define D_CAN_IF_MCTL_TXIE	BIT(11)	/* Transmit int enable */
#define D_CAN_IF_MCTL_RXIE	BIT(10)	/* Receive int enable */
#define D_CAN_IF_MCTL_RMTEN	BIT(9)	/* Remote enable */
#define D_CAN_IF_MCTL_TXRQST	BIT(8)	/* Transmit request */
#define D_CAN_IF_MCTL_EOB	BIT(7)	/* Data frames */
#define D_CAN_IF_MCTL_DLC_MASK	0xF	/* Data length code */

/* D_CAN IF3 Observation reg bit fields */
#define D_CAN_IF3OBS_UP		BIT(15)	/* Update data status */
#define D_CAN_IF3OBS_SDB	BIT(12)	/* DataB read out status */
#define D_CAN_IF3OBS_SDA	BIT(11)	/* DataA read out status */
#define D_CAN_IF3OBS_SC		BIT(10)	/* Contol bits read out status */
#define D_CAN_IF3OBS_SA		BIT(9)	/* Arbitration read out status */
#define D_CAN_IF3OBS_SM		BIT(8)	/* Mask bits read out status */
#define D_CAN_IF3OBS_DB		BIT(4)	/* Data B read observation */
#define D_CAN_IF3OBS_DA		BIT(3)	/* Data A read observation */
#define D_CAN_IF3OBS_CTL	BIT(2)	/* Control read observation */
#define D_CAN_IF3OBS_ARB	BIT(1)	/* Arbitration data read observation */
#define D_CAN_IF3OBS_MASK	BIT(0)	/* Mask data read observation */

/* D_CAN TX I/O reg bit fields */
#define D_CAN_TIOC_PU		BIT(18)	/* CAN_TX pull up/down select */
#define D_CAN_TIOC_PD		BIT(17)	/* CAN_TX pull disable */
#define D_CAN_TIOC_OD		BIT(16)	/* CAN_TX open drain enable */
#define D_CAN_TIOC_FUNC		BIT(3)	/* CAN_TX function */
#define D_CAN_TIOC_DIR		BIT(2)	/* CAN_TX data direction */
#define D_CAN_TIOC_OUT		BIT(1)	/* CAN_TX data out write */
#define D_CAN_TIOC_IN		BIT(0)	/* CAN_TX data in */

/* D_CAN RX I/O reg bit fields */
#define D_CAN_RIOC_PU		BIT(18)	/* CAN_RX pull up/down select */
#define D_CAN_RIOC_PD		BIT(17)	/* CAN_RX pull disable */
#define D_CAN_RIOC_OD		BIT(16)	/* CAN_RX open drain enable */
#define D_CAN_RIOC_FUNC		BIT(3)	/* CAN_RX function */
#define D_CAN_RIOC_DIR		BIT(2)	/* CAN_RX data direction */
#define D_CAN_RIOC_OUT		BIT(1)	/* CAN_RX data out write */
#define D_CAN_RTIOC_IN		BIT(0)	/* CAN_RX data in */

#define D_CAN_SET_REG		0xFFFFFFFF

#define D_CAN_CANMID_IDE	BIT(31)	/* Extended frame format */
#define D_CAN_CANMID_AME	BIT(30)	/* Acceptance mask enable */
#define D_CAN_CANMID_AAM	BIT(29)	/* Auto answer mode */

/*
 * IF register masks:
 */
#define IFX_WRITE_IDR(x)		((x) & 0x1FFFFFFF)

#define IFX_CMD_BITS(x)			((x) & 0xFFFFFF00)
#define IFX_CMD_MSG_NUMBER(x)		((x) & 0xFF)

/* Message objects split */
#define D_CAN_NUM_MSG_OBJECTS		64
#define D_CAN_NUM_RX_MSG_OBJECTS	32
#define D_CAN_NUM_TX_MSG_OBJECTS	32

#define D_CAN_MSG_OBJ_RX_FIRST		1
#define D_CAN_MSG_OBJ_RX_LAST		(D_CAN_MSG_OBJ_RX_FIRST + \
					D_CAN_NUM_RX_MSG_OBJECTS - 1)

#define D_CAN_MSG_OBJ_TX_FIRST		(D_CAN_MSG_OBJ_RX_LAST + 1)
#define D_CAN_MSG_OBJ_TX_LAST		(D_CAN_MSG_OBJ_TX_FIRST + \
					D_CAN_NUM_TX_MSG_OBJECTS - 1)

#define D_CAN_MSG_OBJ_RX_SPLIT		17
#define D_CAN_MSG_OBJ_RX_LOW_LAST	(D_CAN_MSG_OBJ_RX_SPLIT - 1)

#define D_CAN_NEXT_MSG_OBJ_MASK		(D_CAN_NUM_TX_MSG_OBJECTS - 1)

/* status interrupt */
#define STATUS_INTERRUPT		0x8000

/* global interrupt masks */
#define ENABLE_ALL_INTERRUPTS		1
#define DISABLE_ALL_INTERRUPTS		0

/* minimum timeout for checking BUSY status */
#define MIN_TIMEOUT_VALUE		6

/* Wait for ~1 sec for INIT bit */
#define D_CAN_WAIT_COUNT		1000

#define D_CAN_IF_RX_NUM			0
#define D_CAN_IF_TX_NUM			1

#define D_CAN_GET_XREG_NUM(priv, reg)	(__ffs(d_can_read(priv, reg))/4)

/* CAN Bittiming constants as per D_CAN specs */
static struct can_bittiming_const d_can_bittiming_const = {
	.name = D_CAN_DRV_NAME,
	.tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,	/* 6-bit BRP field + 4-bit BRPE field*/
	.brp_inc = 1,
};

/* d_can last error code (lec) values */
enum d_can_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
	LEC_UNUSED,
};

/*
 * d_can error types:
 * Bus errors (BUS_OFF, ERROR_WARNING, ERROR_PASSIVE) are supported
 */
enum d_can_bus_error_types {
	D_CAN_NO_ERROR = 0,
	D_CAN_BUS_OFF,
	D_CAN_ERROR_WARNING,
	D_CAN_ERROR_PASSIVE,
};

static inline void d_can_write(struct d_can_priv *priv, u32 reg, u32 val)
{
	__raw_writel(val, priv->base + reg);
}

static inline u32 d_can_read(struct d_can_priv *priv, int reg)
{
	return __raw_readl(priv->base + reg);
}

static inline void d_can_set_bit(struct d_can_priv *priv, int reg,
	u32 bit_mask)
{
	d_can_write(priv, reg, d_can_read(priv, reg) | bit_mask);
}

static inline u32 d_can_get_bit(struct d_can_priv *priv, int reg,
	u32 bit_mask)
{
	return (d_can_read(priv, reg) & bit_mask) ? 1 : 0;
}

static inline void d_can_clear_bit(struct d_can_priv *priv, int reg,
	u32 bit_mask)
{
	d_can_write(priv, reg, d_can_read(priv, reg) & ~bit_mask);
}

static inline int get_tx_next_msg_obj(const struct d_can_priv *priv)
{
	return (priv->tx_next & D_CAN_NEXT_MSG_OBJ_MASK) +
			D_CAN_MSG_OBJ_TX_FIRST;
}

static inline int get_tx_echo_msg_obj(const struct d_can_priv *priv)
{
	return (priv->tx_echo & D_CAN_NEXT_MSG_OBJ_MASK) +
			D_CAN_MSG_OBJ_TX_FIRST;
}

/*
 * API for enabling and disabling the multiple interrupts
 * of the DCAN module like error interrupt, status interrupt
 * error enable/disable for instance zero and one and etc.
 */
static void d_can_interrupts(struct d_can_priv *priv, int enable)
{
	unsigned int cntrl_save = d_can_read(priv, D_CAN_CTL);

	if (enable)
		cntrl_save |= (D_CAN_CTL_IE1 | D_CAN_CTL_EIE |
				D_CAN_CTL_IE0);
	else
		cntrl_save &= ~(D_CAN_CTL_IE1 | D_CAN_CTL_SIE |
				D_CAN_CTL_EIE | D_CAN_CTL_IE0);

	d_can_write(priv, D_CAN_CTL, cntrl_save);
}

static inline int d_can_msg_obj_is_busy(struct d_can_priv *priv, int iface)
{
	int count = MIN_TIMEOUT_VALUE;

	while (count && (d_can_read(priv, D_CAN_IFCMD(iface)) &
				D_CAN_IF_CMD_BUSY)) {
		count--;
		udelay(1);
	}

	if (!count)
		return 1;

	return 0;
}

static inline void d_can_object_get(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFCMD(iface), IFX_CMD_BITS(mask) |
					IFX_CMD_MSG_NUMBER(objno));

	/*
	 * As per specs, after writing the message object number in the
	 * IF command register the transfer b/w interface register and
	 * message RAM must be complete in 12 CAN-CLK period.
	 */
	if (d_can_msg_obj_is_busy(priv, iface))
		netdev_err(dev, "timed out in object get\n");
}

static inline void d_can_object_put(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFCMD(iface), D_CAN_IF_CMD_WR |
		IFX_CMD_BITS(mask) | IFX_CMD_MSG_NUMBER(objno));

	/*
	 * As per specs, after writing the message object number in the
	 * IF command register the transfer b/w interface register and
	 * message RAM must be complete in 12 CAN-CLK period.
	 */
	if (d_can_msg_obj_is_busy(priv, iface))
		netdev_err(dev, "timed out in object put\n");
}

static void d_can_write_msg_object(struct net_device *dev,
			int iface, struct can_frame *frame, int objno)
{
	int i;
	unsigned int id;
	u32 dataA = 0;
	u32 dataB = 0;
	u32 flags = 0;
	struct d_can_priv *priv = netdev_priv(dev);

	if (!(frame->can_id & CAN_RTR_FLAG))
		flags |= D_CAN_IF_ARB_DIR_XMIT;

	if (frame->can_id & CAN_EFF_FLAG) {
		id = frame->can_id & CAN_EFF_MASK;
		flags |= D_CAN_IF_ARB_MSGXTD;
	} else
		id = ((frame->can_id & CAN_SFF_MASK) << 18);

	flags |= D_CAN_IF_ARB_MSGVAL;
	d_can_write(priv, D_CAN_IFARB(iface), IFX_WRITE_IDR(id) | flags);

	for (i = 0; i < frame->can_dlc; i++) {
		if (frame->can_dlc <= 4)
			dataA |= (frame->data[i] << (8 * i));
		else {
			if (i < 4)
				dataA |= (frame->data[i] << (8 * i));
			else
				dataB |= (frame->data[i] << (8 * (i - 4)));
		}
	}

	/* DATA write to Message object registers DATAA and DATAB */
	if (frame->can_dlc <= 4)
		d_can_write(priv, D_CAN_IFDATA(iface), dataA);
	else {
		d_can_write(priv, D_CAN_IFDATB(iface), dataB);
		d_can_write(priv, D_CAN_IFDATA(iface), dataA);
	}

	/* enable TX interrupt for this message object */
	d_can_write(priv, D_CAN_IFMCTL(iface),
			D_CAN_IF_MCTL_TXIE | D_CAN_IF_MCTL_EOB |
			D_CAN_IF_MCTL_TXRQST | D_CAN_IF_MCTL_NEWDAT |
			frame->can_dlc);

	/* Put message data into message RAM */
	d_can_object_put(dev, iface, objno, D_CAN_IF_CMD_ALL);
}

/*
 * Mark that this particular message object is received and clearing
 * the interrupt pending register value.
 */
static inline void d_can_mark_rx_msg_obj(struct net_device *dev,
				int iface, int ctrl_mask, int obj)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFMCTL(iface), ctrl_mask
		& ~(D_CAN_IF_MCTL_MSGLST | D_CAN_IF_MCTL_INTPND));

	d_can_object_put(dev, iface, obj, D_CAN_IF_CMD_CONTROL);
}

static inline void d_can_activate_all_lower_rx_msg_objs(struct net_device *dev,
				int iface, int ctrl_mask)
{
	int i;
	struct d_can_priv *priv = netdev_priv(dev);

	for (i = D_CAN_MSG_OBJ_RX_FIRST; i <= D_CAN_MSG_OBJ_RX_LOW_LAST; i++) {
		d_can_write(priv, D_CAN_IFMCTL(iface),
				ctrl_mask & ~(D_CAN_IF_MCTL_MSGLST |
				D_CAN_IF_MCTL_INTPND | D_CAN_IF_MCTL_NEWDAT));
		d_can_object_put(dev, iface, i, D_CAN_IF_CMD_CONTROL);
	}
}

static inline void d_can_activate_rx_msg_obj(struct net_device *dev,
						int iface, int ctrl_mask,
						int obj)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFMCTL(iface),
			ctrl_mask & ~(D_CAN_IF_MCTL_MSGLST |
			D_CAN_IF_MCTL_INTPND | D_CAN_IF_MCTL_NEWDAT));
	d_can_object_put(dev, iface, obj, D_CAN_IF_CMD_CONTROL);
}

static void d_can_handle_lost_msg_obj(struct net_device *dev,
					int iface, int objno)
{
	struct d_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	netdev_err(dev, "msg lost in buffer %d\n", objno);

	d_can_object_get(dev, iface, objno, D_CAN_IF_CMD_ALL &
					~D_CAN_IF_CMD_TXRQST);

	d_can_write(priv, D_CAN_IFMCTL(iface), D_CAN_IF_MCTL_CLR_MSGLST);

	d_can_object_put(dev, iface, objno, D_CAN_IF_CMD_CONTROL);

	/* create an error msg */
	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	stats->rx_errors++;
	stats->rx_over_errors++;

	netif_receive_skb(skb);
}

static int d_can_read_msg_object(struct net_device *dev, int iface, int ctrl)
{
	int i;
	u32 dataA = 0;
	u32 dataB = 0;
	unsigned int arb_val;
	unsigned int mctl_val;
	struct d_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	skb = alloc_can_skb(dev, &frame);
	if (!skb) {
		stats->rx_dropped++;
		return -ENOMEM;
	}

	frame->can_dlc = get_can_dlc(ctrl & 0x0F);

	arb_val = d_can_read(priv, D_CAN_IFARB(iface));
	mctl_val = d_can_read(priv, D_CAN_IFMCTL(iface));

	if (arb_val & D_CAN_IF_ARB_MSGXTD)
		frame->can_id = (arb_val & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		frame->can_id = (arb_val >> 18) & CAN_SFF_MASK;

	if (mctl_val & D_CAN_IF_MCTL_RMTEN)
		frame->can_id |= CAN_RTR_FLAG;
	else {
		dataA = d_can_read(priv, D_CAN_IFDATA(iface));
		dataB = d_can_read(priv, D_CAN_IFDATB(iface));
		for (i = 0; i < frame->can_dlc; i++) {
			/* Writing MO higher 4 data bytes to skb */
			if (frame->can_dlc <= 4)
				frame->data[i] = dataA >> (8 * i);
			else {
				if (i < 4)
					frame->data[i] = dataA >> (8 * i);
				else
					frame->data[i] = dataB >> (8 * (i-4));
			}
		}
	}

	netif_receive_skb(skb);

	stats->rx_packets++;
	stats->rx_bytes += frame->can_dlc;

	return 0;
}

static void d_can_setup_receive_object(struct net_device *dev, int iface,
					int objno, unsigned int mask,
					unsigned int id, unsigned int mcont)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFMSK(iface), IFX_WRITE_IDR(mask));
	d_can_write(priv, D_CAN_IFARB(iface), IFX_WRITE_IDR(id) |
			D_CAN_IF_ARB_MSGVAL);
	d_can_write(priv, D_CAN_IFMCTL(iface), mcont);

	d_can_object_put(dev, iface, objno, D_CAN_IF_CMD_ALL &
					~D_CAN_IF_CMD_TXRQST);

	netdev_dbg(dev, "obj no:%d, msgval:0x%08x\n", objno, d_can_read(priv,
		D_CAN_MSGVAL(D_CAN_GET_XREG_NUM(priv, D_CAN_MSGVAL_X))));
}

static void d_can_inval_msg_object(struct net_device *dev, int iface, int objno)
{
	struct d_can_priv *priv = netdev_priv(dev);

	d_can_write(priv, D_CAN_IFARB(iface), 0);
	d_can_write(priv, D_CAN_IFMCTL(iface), 0);

	d_can_object_put(dev, iface, objno, D_CAN_IF_CMD_ARB |
					D_CAN_IF_CMD_CONTROL);

	netdev_dbg(dev, "obj no:%d, msgval:0x%08x\n", objno, d_can_read(priv,
		D_CAN_MSGVAL(D_CAN_GET_XREG_NUM(priv, D_CAN_MSGVAL_X))));
}

static inline int d_can_is_next_tx_obj_busy(struct d_can_priv *priv, int objno)
{
	u32 txrq_x_reg_val = D_CAN_GET_XREG_NUM(priv, D_CAN_TXRQ_X);

	/*
	 * as transmission request register's bit n-1 corresponds to
	 * message object n, we need to handle the same properly.
	 */
	if (d_can_read(priv, D_CAN_TXRQ(txrq_x_reg_val)) &
			(1 << (objno - D_CAN_MSG_OBJ_TX_FIRST)))
		return 1;

	return 0;
}

static netdev_tx_t d_can_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	u32 msg_obj_no;
	struct d_can_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	msg_obj_no = get_tx_next_msg_obj(priv);

	/* prepare message object for transmission */
	d_can_write_msg_object(dev, D_CAN_IF_TX_NUM, frame, msg_obj_no);
	can_put_echo_skb(skb, dev, msg_obj_no - D_CAN_MSG_OBJ_TX_FIRST);

	/*
	 * we have to stop the queue in case of a wrap around or
	 * if the next TX message object is still in use
	 */
	priv->tx_next++;
	if (d_can_is_next_tx_obj_busy(priv, get_tx_next_msg_obj(priv)) ||
		((priv->tx_next & D_CAN_NEXT_MSG_OBJ_MASK) == 0))
		netif_stop_queue(dev);

	return NETDEV_TX_OK;
}

static int d_can_set_bittiming(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u32 can_btc;

	can_btc = ((bt->phase_seg2 - 1) & 0x7) << D_CAN_BTR_TSEG2_SHIFT;
	can_btc |= ((bt->phase_seg1 + bt->prop_seg - 1)
			& 0xF) << D_CAN_BTR_TSEG1_SHIFT;

	can_btc |= ((bt->sjw - 1) & 0x3) << D_CAN_BTR_SJW_SHIFT;

	/* Ten bits contains the BRP, 6 bits for BRP and upper 4 bits for brpe*/
	can_btc |= ((bt->brp - 1) & 0x3F) << D_CAN_BTR_BRP_SHIFT;
	can_btc |= ((((bt->brp - 1) >> 6) & 0xF) << D_CAN_BTR_BRPE_SHIFT);

	d_can_write(priv, D_CAN_BTR, can_btc);

	netdev_info(dev, "setting CAN BT = %#x\n", can_btc);

	return 0;
}

/*
 * Configure D_CAN message objects for Tx and Rx purposes:
 * D_CAN provides a total of 64 message objects that can be configured
 * either for Tx or Rx purposes. In this driver first 32 message objects
 * are used as a reception FIFO and the reception FIFO is signified by the
 * EoB bit being SET. The remaining 32 message objects are kept aside for
 * Tx purposes. See user guide document for further details on configuring
 * message objects.
 */
static void d_can_configure_msg_objects(struct net_device *dev)
{
	unsigned int i;

	/* first invalidate all message objects */
	for (i = D_CAN_MSG_OBJ_RX_FIRST; i <= D_CAN_NUM_MSG_OBJECTS; i++)
		d_can_inval_msg_object(dev, D_CAN_IF_RX_NUM, i);

	/* setup receive message objects */
	for (i = D_CAN_MSG_OBJ_RX_FIRST; i < D_CAN_MSG_OBJ_RX_LAST; i++)
		d_can_setup_receive_object(dev, D_CAN_IF_RX_NUM, i, 0, 0,
			(D_CAN_IF_MCTL_RXIE | D_CAN_IF_MCTL_UMASK) &
			~D_CAN_IF_MCTL_EOB);

	/* Last object EoB bit should be 1 for terminate */
	d_can_setup_receive_object(dev, D_CAN_IF_RX_NUM, D_CAN_MSG_OBJ_RX_LAST,
			0, 0, D_CAN_IF_MCTL_RXIE | D_CAN_IF_MCTL_UMASK |
			D_CAN_IF_MCTL_EOB);
}

static void d_can_test_mode(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);

	/* Test mode is enabled in this step & the specific TEST bits
	 * are enabled accordingly */
	d_can_write(priv, D_CAN_CTL, D_CAN_CTL_EIE |
			D_CAN_CTL_IE1 |	D_CAN_CTL_IE0 | D_CAN_CTL_TEST);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* silent mode : bus-monitoring mode */
		d_can_write(priv, D_CAN_TEST, D_CAN_TEST_SILENT);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* loopback mode : useful for self-test function */
		d_can_write(priv, D_CAN_TEST, D_CAN_TEST_LBACK);
	} else {
		/* loopback + silent mode : useful for hot self-test */
		d_can_write(priv, D_CAN_TEST, D_CAN_TEST_LBACK |
				D_CAN_TEST_SILENT);
	}
}

/*
 * Configure D_CAN chip:
 * - enable/disable auto-retransmission
 * - set operating mode
 * - configure message objects
 */
static void d_can_init(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);
	u32 cnt;

	netdev_dbg(dev, "resetting d_can ...\n");
	d_can_set_bit(priv, D_CAN_CTL, D_CAN_CTL_SWR);

	/* Enter initialization mode by setting the Init bit */
	d_can_set_bit(priv, D_CAN_CTL, D_CAN_CTL_INIT);

	/* enable automatic retransmission */
	d_can_set_bit(priv, D_CAN_CTL, D_CAN_CTL_ENABLE_AR);

	/* Set the Configure Change Enable ( CCE) bit */
	d_can_set_bit(priv, D_CAN_CTL, D_CAN_CTL_CCE);

	/* Wait for the Init bit to get set */
	cnt = D_CAN_WAIT_COUNT;
	while (!d_can_get_bit(priv, D_CAN_CTL, D_CAN_CTL_INIT) && cnt != 0) {
		--cnt;
		udelay(10);
	}

	/* set bittiming params */
	d_can_set_bittiming(dev);

	d_can_clear_bit(priv, D_CAN_CTL, D_CAN_CTL_INIT | D_CAN_CTL_CCE);

	/* Wait for the Init bit to get clear */
	cnt = D_CAN_WAIT_COUNT;
	while (d_can_get_bit(priv, D_CAN_CTL, D_CAN_CTL_INIT) && cnt != 0) {
		--cnt;
		udelay(10);
	}

	if (priv->can.ctrlmode & (CAN_CTRLMODE_LOOPBACK |
				CAN_CTRLMODE_LISTENONLY))
		d_can_test_mode(dev);
	else
		/* normal mode*/
		d_can_write(priv, D_CAN_CTL, D_CAN_CTL_EIE | D_CAN_CTL_IE1 |
							D_CAN_CTL_IE0);

	/* Enable TX and RX I/O Control pins */
	d_can_write(priv, D_CAN_TIOC, D_CAN_TIOC_FUNC);
	d_can_write(priv, D_CAN_RIOC, D_CAN_RIOC_FUNC);

	/* configure message objects */
	d_can_configure_msg_objects(dev);

	/* set a LEC value so that we can check for updates later */
	d_can_write(priv, D_CAN_ES, LEC_UNUSED);
}

static void d_can_start(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);

	/* basic d_can initialization */
	d_can_init(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* reset tx helper pointers */
	priv->tx_next = priv->tx_echo = 0;

	/* enable status change, error and module interrupts */
	d_can_interrupts(priv, ENABLE_ALL_INTERRUPTS);
}

static void d_can_stop(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts */
	d_can_interrupts(priv, DISABLE_ALL_INTERRUPTS);

	/* set the state as STOPPED */
	priv->can.state = CAN_STATE_STOPPED;
}

static int d_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		d_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int d_can_get_berr_counter(const struct net_device *dev,
					struct can_berr_counter *bec)
{
	unsigned int reg_err_counter;
	struct d_can_priv *priv = netdev_priv(dev);

	reg_err_counter = d_can_read(priv, D_CAN_ERRC);
	bec->rxerr = (reg_err_counter & D_CAN_ERRC_REC_MASK) >>
				D_CAN_ERRC_REC_SHIFT;
	bec->txerr = reg_err_counter & D_CAN_ERRC_TEC_MASK;

	return 0;
}

/*
 * theory of operation:
 *
 * priv->tx_echo holds the number of the oldest can_frame put for
 * transmission into the hardware, but not yet ACKed by the CAN tx
 * complete IRQ.
 *
 * We iterate from priv->tx_echo to priv->tx_next and check if the
 * packet has been transmitted, echo it back to the CAN framework.
 * If we discover a not yet transmitted package, stop looking for more.
 */
static void d_can_do_tx(struct net_device *dev)
{
	u32 msg_obj_no;
	struct d_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 txrq_x_reg_val;
	u32 txrq_reg_val;

	for (/* nix */; (priv->tx_next - priv->tx_echo) > 0; priv->tx_echo++) {
		msg_obj_no = get_tx_echo_msg_obj(priv);
		txrq_x_reg_val = D_CAN_GET_XREG_NUM(priv, D_CAN_TXRQ_X);
		txrq_reg_val = d_can_read(priv, D_CAN_TXRQ(txrq_x_reg_val));
		if (!(txrq_reg_val & (1 << (msg_obj_no -
						D_CAN_MSG_OBJ_TX_FIRST)))) {
			can_get_echo_skb(dev,
					msg_obj_no - D_CAN_MSG_OBJ_TX_FIRST);
			stats->tx_bytes += d_can_read(priv,
					D_CAN_IFMCTL(D_CAN_IF_TX_NUM))
					& D_CAN_IF_MCTL_DLC_MASK;
			stats->tx_packets++;
			d_can_inval_msg_object(dev, D_CAN_IF_TX_NUM,
					msg_obj_no);
		} else
			break;
	}

	/* restart queue if wrap-up or if queue stalled on last pkt */
	if (((priv->tx_next & D_CAN_NEXT_MSG_OBJ_MASK) != 0)
		|| ((priv->tx_echo & D_CAN_NEXT_MSG_OBJ_MASK) == 0))
		netif_wake_queue(dev);
}

/*
 * theory of operation:
 *
 * d_can core saves a received CAN message into the first free message
 * object it finds free (starting with the lowest). Bits NEWDAT and
 * INTPND are set for this message object indicating that a new message
 * has arrived. To work-around this issue, we keep two groups of message
 * objects whose partitioning is defined by D_CAN_MSG_OBJ_RX_SPLIT.
 *
 * To ensure in-order frame reception we use the following
 * approach while re-activating a message object to receive further
 * frames:
 * - if the current message object number is lower than
 *   D_CAN_MSG_RX_LOW_LAST, do not clear the NEWDAT bit while clearing
 *   the INTPND bit.
 * - if the current message object number is equal to
 *   D_CAN_MSG_RX_LOW_LAST then clear the NEWDAT bit of all lower
 *   receive message objects.
 * - if the current message object number is greater than
 *   D_CAN_MSG_RX_LOW_LAST then clear the NEWDAT bit of
 *   only this message object.
 */
static int d_can_do_rx_poll(struct net_device *dev, int quota)
{
	struct d_can_priv *priv = netdev_priv(dev);
	unsigned int msg_obj, mctrl_reg_val;
	u32 num_rx_pkts = 0;
	u32 intpnd_x_reg_val;
	u32 intpnd_reg_val;

	for (msg_obj = D_CAN_MSG_OBJ_RX_FIRST; msg_obj <= D_CAN_MSG_OBJ_RX_LAST
				&& quota > 0; msg_obj++) {

		intpnd_x_reg_val = D_CAN_GET_XREG_NUM(priv, D_CAN_INTPND_X);
		intpnd_reg_val = d_can_read(priv,
					D_CAN_INTPND(intpnd_x_reg_val));

		/*
		 * as interrupt pending register's bit n-1 corresponds to
		 * message object n, we need to handle the same properly.
		 */
		if (intpnd_reg_val & (1 << (msg_obj - 1))) {

			d_can_object_get(dev, D_CAN_IF_RX_NUM, msg_obj,
					D_CAN_IF_CMD_ALL &
					~D_CAN_IF_CMD_TXRQST);

			mctrl_reg_val = d_can_read(priv,
					D_CAN_IFMCTL(D_CAN_IF_RX_NUM));

			if (!(mctrl_reg_val & D_CAN_IF_MCTL_NEWDAT))
				continue;

			/* read the data from the message object */
			d_can_read_msg_object(dev, D_CAN_IF_RX_NUM,
						mctrl_reg_val);

			if (mctrl_reg_val & D_CAN_IF_MCTL_EOB)
				d_can_setup_receive_object(dev, D_CAN_IF_RX_NUM,
					D_CAN_MSG_OBJ_RX_LAST, 0, 0,
					D_CAN_IF_MCTL_RXIE | D_CAN_IF_MCTL_UMASK
					| D_CAN_IF_MCTL_EOB);

			if (mctrl_reg_val & D_CAN_IF_MCTL_MSGLST) {
				d_can_handle_lost_msg_obj(dev, D_CAN_IF_RX_NUM,
					msg_obj);
				num_rx_pkts++;
				quota--;
				continue;
			}

			if (msg_obj < D_CAN_MSG_OBJ_RX_LOW_LAST)
				d_can_mark_rx_msg_obj(dev, D_CAN_IF_RX_NUM,
						mctrl_reg_val, msg_obj);
			else if (msg_obj > D_CAN_MSG_OBJ_RX_LOW_LAST)
				/* activate this msg obj */
				d_can_activate_rx_msg_obj(dev, D_CAN_IF_RX_NUM,
						mctrl_reg_val, msg_obj);
			else if (msg_obj == D_CAN_MSG_OBJ_RX_LOW_LAST)
				/* activate all lower message objects */
				d_can_activate_all_lower_rx_msg_objs(dev,
						D_CAN_IF_RX_NUM, mctrl_reg_val);

			num_rx_pkts++;
			quota--;
		}
	}

	return num_rx_pkts;
}

static inline int d_can_has_handle_berr(struct d_can_priv *priv)
{
	return (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
		(priv->current_status & LEC_UNUSED);
}

static int d_can_handle_state_change(struct net_device *dev,
				enum d_can_bus_error_types error_type)
{
	unsigned int reg_err_counter;
	unsigned int rx_err_passive;
	struct d_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	d_can_get_berr_counter(dev, &bec);
	reg_err_counter = d_can_read(priv, D_CAN_ERRC);
	rx_err_passive = (reg_err_counter & D_CAN_ERRC_RP_MASK) >>
				D_CAN_ERRC_RP_SHIFT;

	switch (error_type) {
	case D_CAN_ERROR_WARNING:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			CAN_ERR_CRTL_TX_WARNING :
			CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;

		break;
	case D_CAN_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		if (rx_err_passive)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case D_CAN_BUS_OFF:
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		/*
		 * disable all interrupts in bus-off mode to ensure that
		 * the CPU is not hogged down
		 */
		d_can_interrupts(priv, DISABLE_ALL_INTERRUPTS);
		can_bus_off(dev);
		break;
	default:
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int d_can_handle_bus_err(struct net_device *dev,
				enum d_can_lec_type lec_type)
{
	struct d_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	/*
	 * early exit if no lec update or no error.
	 * no lec update means that no CAN bus event has been detected
	 * since CPU wrote 0x7 value to status reg.
	 */
	if (lec_type == LEC_UNUSED || lec_type == LEC_NO_ERROR)
		return 0;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/*
	 * check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */

	/* common for all type of bus errors */
	priv->can.can_stats.bus_error++;
	stats->rx_errors++;
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	cf->data[2] |= CAN_ERR_PROT_UNSPEC;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_dbg(dev, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_dbg(dev, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_dbg(dev, "ack error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_ACK |
				CAN_ERR_PROT_LOC_ACK_DEL);
		break;
	case LEC_BIT1_ERROR:
		netdev_dbg(dev, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_dbg(dev, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_dbg(dev, "CRC error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_CRC_SEQ |
				CAN_ERR_PROT_LOC_CRC_DEL);
		break;
	default:
		break;
	}

	/* set a LEC value so that we can check for updates later */
	d_can_write(priv, D_CAN_ES, LEC_UNUSED);

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int d_can_poll(struct napi_struct *napi, int quota)
{
	int lec_type = 0;
	int work_done = 0;
	struct net_device *dev = napi->dev;
	struct d_can_priv *priv = netdev_priv(dev);

	if (!priv->irqstatus)
		goto end;

	/* status events have the highest priority */
	if (priv->irqstatus == STATUS_INTERRUPT) {
		priv->current_status = d_can_read(priv, D_CAN_ES);

		/* handle Tx/Rx events */
		if (priv->current_status & D_CAN_ES_TXOK)
			d_can_write(priv, D_CAN_ES,
					priv->current_status & ~D_CAN_ES_TXOK);

		if (priv->current_status & D_CAN_ES_RXOK)
			d_can_write(priv, D_CAN_ES,
					priv->current_status & ~D_CAN_ES_RXOK);

		/* handle state changes */
		if ((priv->current_status & D_CAN_ES_EWARN) &&
				(!(priv->last_status & D_CAN_ES_EWARN))) {
			netdev_dbg(dev, "entered error warning state\n");
			work_done += d_can_handle_state_change(dev,
						D_CAN_ERROR_WARNING);
		}
		if ((priv->current_status & D_CAN_ES_EPASS) &&
				(!(priv->last_status & D_CAN_ES_EPASS))) {
			netdev_dbg(dev, "entered error passive state\n");
			work_done += d_can_handle_state_change(dev,
						D_CAN_ERROR_PASSIVE);
		}
		if ((priv->current_status & D_CAN_ES_BOFF) &&
				(!(priv->last_status & D_CAN_ES_BOFF))) {
			netdev_dbg(dev, "entered bus off state\n");
			work_done += d_can_handle_state_change(dev,
						D_CAN_BUS_OFF);
		}

		/* handle bus recovery events */
		if ((!(priv->current_status & D_CAN_ES_BOFF)) &&
				(priv->last_status & D_CAN_ES_BOFF)) {
			netdev_dbg(dev, "left bus off state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}
		if ((!(priv->current_status & D_CAN_ES_EPASS)) &&
				(priv->last_status & D_CAN_ES_EPASS)) {
			netdev_dbg(dev, "left error passive state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}

		priv->last_status = priv->current_status;

		/* handle lec errors on the bus */
		lec_type = d_can_has_handle_berr(priv);
		if (lec_type)
			work_done += d_can_handle_bus_err(dev, lec_type);
	} else if ((priv->irqstatus >= D_CAN_MSG_OBJ_RX_FIRST) &&
			(priv->irqstatus <= D_CAN_MSG_OBJ_RX_LAST)) {
		/* handle events corresponding to receive message objects */
		work_done += d_can_do_rx_poll(dev, (quota - work_done));
	} else if ((priv->irqstatus >= D_CAN_MSG_OBJ_TX_FIRST) &&
			(priv->irqstatus <= D_CAN_MSG_OBJ_TX_LAST)) {
		/* handle events corresponding to transmit message objects */
		d_can_do_tx(dev);
	}

end:
	if (work_done < quota) {
		napi_complete(napi);
		/* enable all IRQs */
		d_can_interrupts(priv, ENABLE_ALL_INTERRUPTS);
	}

	return work_done;
}

static irqreturn_t d_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct d_can_priv *priv = netdev_priv(dev);

	priv->irqstatus = d_can_read(priv, D_CAN_INT);
	if (!priv->irqstatus)
		return IRQ_NONE;

	/* disable all interrupts and schedule the NAPI */
	d_can_interrupts(priv, DISABLE_ALL_INTERRUPTS);
	napi_schedule(&priv->napi);

	return IRQ_HANDLED;
}

static int d_can_open(struct net_device *ndev)
{
	int err;
	struct d_can_priv *priv = netdev_priv(ndev);

	/* Open common can device */
	err = open_candev(ndev);
	if (err) {
		netdev_err(ndev, "open_candev() failed %d\n", err);
		return err;
	}

	/* register interrupt handler for Message Object (MO)
	 * and Error + status change (ES) */
	err = request_irq(ndev->irq, &d_can_isr, IRQF_SHARED, ndev->name,
				ndev);
	if (err) {
		netdev_err(ndev, "failed to request MO_ES interrupt\n");
		goto exit_close_candev;
	}

	/* register interrupt handler for only Message Object */
	err = request_irq(priv->irq_obj, &d_can_isr, IRQF_SHARED, ndev->name,
				ndev);
	if (err) {
		netdev_err(ndev, "failed to request MO interrupt\n");
		goto exit_free_irq;
	}

	/* start the d_can controller */
	d_can_start(ndev);

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	priv->opened = true;
	return 0;
exit_free_irq:
	free_irq(ndev->irq, ndev);
exit_close_candev:
	close_candev(ndev);

	return err;
}

static int d_can_close(struct net_device *ndev)
{
	struct d_can_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	d_can_stop(ndev);
	free_irq(ndev->irq, ndev);
	free_irq(priv->irq_obj, ndev);
	close_candev(ndev);
	priv->opened = false;

	return 0;
}

void d_can_reset_ram(struct d_can_priv *d_can, unsigned int instance,
					unsigned int enable)
{
	if (d_can->ram_init)
		d_can->ram_init(instance, enable);

	/* Give some time delay for DCAN RAM initialization */
	udelay(1);
}
EXPORT_SYMBOL_GPL(d_can_reset_ram);

struct net_device *alloc_d_can_dev(int num_objs)
{
	struct net_device *dev;
	struct d_can_priv *priv;

	dev = alloc_candev(sizeof(struct d_can_priv), num_objs/2);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	netif_napi_add(dev, &priv->napi, d_can_poll, num_objs/2);

	priv->dev = dev;
	priv->can.bittiming_const = &d_can_bittiming_const;
	priv->can.do_set_mode = d_can_set_mode;
	priv->can.do_get_berr_counter = d_can_get_berr_counter;
	priv->can.ctrlmode_supported = (CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_BERR_REPORTING |
					CAN_CTRLMODE_3_SAMPLES);

	return dev;
}
EXPORT_SYMBOL_GPL(alloc_d_can_dev);

#ifdef CONFIG_PM
int d_can_power_down(struct d_can_priv *d_can)
{
	unsigned long time_out;
	struct net_device *ndev = platform_get_drvdata(d_can->pdev);

	d_can_set_bit(d_can, D_CAN_CTL, D_CAN_CTL_PDR);

	/* Wait for the PDA bit to get set */
	time_out = jiffies + msecs_to_jiffies(D_CAN_WAIT_COUNT);
	while (!d_can_get_bit(d_can, D_CAN_ES, D_CAN_ES_PDA) &&
				time_after(time_out, jiffies))
		cpu_relax();

	if (time_after(jiffies, time_out))
		return -ETIMEDOUT;

	if (d_can->opened)
		d_can_stop(ndev);

	return 0;
}
EXPORT_SYMBOL_GPL(d_can_power_down);

int d_can_power_up(struct d_can_priv *d_can)
{
	unsigned long time_out;
	struct net_device *ndev = platform_get_drvdata(d_can->pdev);

	d_can_clear_bit(d_can, D_CAN_CTL, D_CAN_CTL_PDR);
	d_can_clear_bit(d_can, D_CAN_CTL, D_CAN_CTL_INIT);

	/* Wait for the PDA bit to get clear */
	time_out = jiffies + msecs_to_jiffies(D_CAN_WAIT_COUNT);
	while (d_can_get_bit(d_can, D_CAN_ES, D_CAN_ES_PDA) &&
				time_after(time_out, jiffies))
		cpu_relax();

	if (time_after(jiffies, time_out))
		return -ETIMEDOUT;

	if (d_can->opened)
		d_can_start(ndev);

	return 0;
}
EXPORT_SYMBOL_GPL(d_can_power_up);
#else
#define d_can_power_down NULL
#define d_can_power_up NULL
#endif

void free_d_can_dev(struct net_device *dev)
{
	free_candev(dev);
}
EXPORT_SYMBOL_GPL(free_d_can_dev);

static const struct net_device_ops d_can_netdev_ops = {
	.ndo_open = d_can_open,
	.ndo_stop = d_can_close,
	.ndo_start_xmit = d_can_start_xmit,
};

int register_d_can_dev(struct net_device *dev)
{
	/* we support local echo */
	dev->flags |= IFF_ECHO;
	dev->netdev_ops = &d_can_netdev_ops;

	return register_candev(dev);
}
EXPORT_SYMBOL_GPL(register_d_can_dev);

void unregister_d_can_dev(struct net_device *dev)
{
	struct d_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts */
	d_can_interrupts(priv, DISABLE_ALL_INTERRUPTS);

	unregister_candev(dev);
}
EXPORT_SYMBOL_GPL(unregister_d_can_dev);

MODULE_AUTHOR("AnilKumar Ch <anilkumar@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(D_CAN_VERSION);
MODULE_DESCRIPTION(D_CAN_DRV_DESC);
