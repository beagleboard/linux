/*
 * Copyright (C) 2006-2010 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Derived from the PCAN project file driver/src/pcan_mpc5200.c:
 *
 * Copyright (c) 2003 Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Copyright (c) 2005 Felix Daners, Plugit AG, felix.daners@plugit.ch
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

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include "rtcan_dev.h"
#include "rtcan_raw.h"
#include "rtcan_internal.h"
#include "rtcan_mscan_regs.h"
#include "rtcan_mscan.h"

#define MSCAN_SET_MODE_RETRIES	255

#ifndef CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD
static struct can_bittiming_const mscan_bittiming_const = {
	.name = "mscan",
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};
#endif

/**
 *  Reception Interrupt handler
 *
 *  Inline function first called within @ref rtcan_mscan_interrupt when an RX
 *  interrupt was detected. Here the HW registers are read out and composed
 *  to a struct rtcan_skb.
 *
 *  @param[out] skb  Pointer to an instance of struct rtcan_skb which will be
 *                   filled with received CAN message
 *  @param[in]  dev  Device ID
 */
static inline void rtcan_mscan_rx_interrupt(struct rtcan_device *dev,
					    struct rtcan_skb *skb)
{
	int i;
	unsigned char size;
	struct rtcan_rb_frame *frame = &skb->rb_frame;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE;

	frame->can_dlc = in_8(&regs->canrxfg.dlr) & 0x0F;

	/* If DLC exceeds 8 bytes adjust it to 8 (for the payload size) */
	size = (frame->can_dlc > 8) ? 8 : frame->can_dlc;

	if (in_8(&regs->canrxfg.idr[1]) & MSCAN_BUF_EXTENDED) {
		frame->can_id = ((in_8(&regs->canrxfg.idr[0]) << 21) |
				 ((in_8(&regs->canrxfg.idr[1]) & 0xE0) << 13) |
				 ((in_8(&regs->canrxfg.idr[1]) & 0x07) << 15) |
				 (in_8(&regs->canrxfg.idr[4]) << 7) |
				 (in_8(&regs->canrxfg.idr[5]) >> 1));

		frame->can_id |= CAN_EFF_FLAG;

		if ((in_8(&regs->canrxfg.idr[5]) & MSCAN_BUF_EXT_RTR)) {
			frame->can_id |= CAN_RTR_FLAG;
		} else {
			for (i = 0; i < size; i++)
				frame->data[i] =
					in_8(&regs->canrxfg.dsr[i +
								(i / 2) * 2]);
			skb->rb_frame_size += size;
		}

	} else {
		frame->can_id = ((in_8(&regs->canrxfg.idr[0]) << 3) |
				 (in_8(&regs->canrxfg.idr[1]) >> 5));

		if ((in_8(&regs->canrxfg.idr[1]) & MSCAN_BUF_STD_RTR)) {
			frame->can_id |= CAN_RTR_FLAG;
		} else {
			for (i = 0; i < size; i++)
				frame->data[i] =
					in_8(&regs->canrxfg.dsr[i +
								(i / 2) * 2]);
			skb->rb_frame_size += size;
		}
	}


	/* Store the interface index */
	frame->can_ifindex = dev->ifindex;
}

static can_state_t mscan_stat_map[4] = {
	CAN_STATE_ACTIVE,
	CAN_STATE_BUS_WARNING,
	CAN_STATE_BUS_PASSIVE,
	CAN_STATE_BUS_OFF
};

static inline void rtcan_mscan_err_interrupt(struct rtcan_device *dev,
					     struct rtcan_skb *skb,
					     int r_status)
{
	u8 rstat, tstat;
	struct rtcan_rb_frame *frame = &skb->rb_frame;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + CAN_ERR_DLC;

	frame->can_id = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;

	memset(&frame->data[0], 0, frame->can_dlc);

	if ((r_status & MSCAN_OVRIF)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

	} else if ((r_status & (MSCAN_CSCIF))) {

		rstat = (r_status & (MSCAN_TSTAT0 |
				     MSCAN_TSTAT1)) >> 2 & 0x3;
		tstat = (r_status & (MSCAN_RSTAT0 |
				     MSCAN_RSTAT1)) >> 4 & 0x3;
		dev->state = mscan_stat_map[max(rstat, tstat)];

		switch (dev->state) {
		case CAN_STATE_BUS_OFF:
			/* Bus-off condition */
			frame->can_id |= CAN_ERR_BUSOFF;
			dev->state = CAN_STATE_BUS_OFF;
			/* Disable receiver interrupts */
			out_8(&regs->canrier, 0);
			/* Wake up waiting senders */
			rtdm_sem_destroy(&dev->tx_sem);
			break;

		case CAN_STATE_BUS_PASSIVE:
			frame->can_id |= CAN_ERR_CRTL;
			if (tstat > rstat)
				frame->data[1] = CAN_ERR_CRTL_TX_PASSIVE;
			else
				frame->data[1] = CAN_ERR_CRTL_RX_PASSIVE;
			break;

		case CAN_STATE_BUS_WARNING:
			frame->can_id |= CAN_ERR_CRTL;
			if (tstat > rstat)
				frame->data[1] = CAN_ERR_CRTL_TX_WARNING;
			else
				frame->data[1] = CAN_ERR_CRTL_RX_WARNING;
			break;

		default:
			break;

		}
	}
	/* Store the interface index */
	frame->can_ifindex = dev->ifindex;
}

/** Interrupt handler */
static int rtcan_mscan_interrupt(rtdm_irq_t *irq_handle)
{
	struct rtcan_skb skb;
	struct rtcan_device *dev;
	struct mscan_regs *regs;
	u8 canrflg;
	int recv_lock_free = 1;
	int ret = RTDM_IRQ_NONE;


	dev = (struct rtcan_device *)rtdm_irq_get_arg(irq_handle, void);
	regs = (struct mscan_regs *)dev->base_addr;

	rtdm_lock_get(&dev->device_lock);

	canrflg = in_8(&regs->canrflg);

	ret = RTDM_IRQ_HANDLED;

	/* Transmit Interrupt? */
	if ((in_8(&regs->cantier) & MSCAN_TXIE0) &&
	    (in_8(&regs->cantflg) & MSCAN_TXE0)) {
		out_8(&regs->cantier, 0);
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
	}

	/* Wakeup interrupt?  */
	if ((canrflg & MSCAN_WUPIF)) {
		rtdm_printk("WUPIF interrupt\n");
	}

	/* Receive Interrupt? */
	if ((canrflg & MSCAN_RXF)) {

		/* Read out HW registers */
		rtcan_mscan_rx_interrupt(dev, &skb);

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
	}

	/* Error Interrupt? */
	if ((canrflg & (MSCAN_CSCIF | MSCAN_OVRIF))) {
		/* Check error condition and fill error frame */
		rtcan_mscan_err_interrupt(dev, &skb, canrflg);

		if (recv_lock_free) {
			recv_lock_free = 0;
			rtdm_lock_get(&rtcan_recv_list_lock);
			rtdm_lock_get(&rtcan_socket_lock);
		}

		/* Pass error frame out to the sockets */
		rtcan_rcv(dev, &skb);
	}

	/* Acknowledge the handled interrupt within the controller.
	 * Only do so for the receiver interrupts.
	 */
	if (canrflg)
		out_8(&regs->canrflg, canrflg);

	if (!recv_lock_free) {
		rtdm_lock_put(&rtcan_socket_lock);
		rtdm_lock_put(&rtcan_recv_list_lock);
	}
	rtdm_lock_put(&dev->device_lock);

	return ret;
}

/**
 *   Set controller into reset mode. Called from @ref rtcan_mscan_ioctl
 *   (main usage), init_module and cleanup_module.
 *
 *   @param dev_id   Device ID
 *   @param lock_ctx Pointer to saved IRQ context (if stored before calling
 *                   this function). Only evaluated if @c locked is true.
 *   @param locked   Boolean value indicating if function was called in an
 *                   spin locked and IRQ disabled context
 *
 *   @return 0 on success, otherwise:
 *   - -EAGAIN: Reset mode bit could not be verified after setting it.
 *              See also note.
 *
 *   @note According to the MSCAN specification, it is necessary to check
 *   the reset mode bit in PeliCAN mode after having set it. So we do. But if
 *   using a ISA card like the PHYTEC eNET card this should not be necessary
 *   because the CAN controller clock of this card (16 MHz) is twice as high
 *   as the ISA bus clock.
 */
static int rtcan_mscan_mode_stop(struct rtcan_device *dev,
				 rtdm_lockctx_t *lock_ctx)
{
	int ret = 0;
	int rinit = 0;
	can_state_t state;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;
	u8 reg;

	state = dev->state;
	/* If controller is not operating anyway, go out */
	if (!CAN_STATE_OPERATING(state))
		goto out;

	/* Switch to sleep mode */
	setbits8(&regs->canctl0, MSCAN_SLPRQ);
	reg = in_8(&regs->canctl1);
	while (!(reg & MSCAN_SLPAK) &&
	        (rinit < MSCAN_SET_MODE_RETRIES)) {
		if (likely(lock_ctx != NULL))
			rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
		/* Busy sleep 1 microsecond */
		rtdm_task_busy_sleep(1000);
		if (likely(lock_ctx != NULL))
			rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
		rinit++;
		reg = in_8(&regs->canctl1);
	}
	/*
	 * The mscan controller will fail to enter sleep mode,
	 * while there are irregular activities on bus, like
	 * somebody keeps retransmitting. This behavior is
	 * undocumented and seems to differ between mscan built
	 * in mpc5200b and mpc5200. We proceed in that case,
	 * since otherwise the slprq will be kept set and the
	 * controller will get stuck. NOTE: INITRQ or CSWAI
	 * will abort all active transmit actions, if still
	 * any, at once.
	 */
	if (rinit >= MSCAN_SET_MODE_RETRIES)
		rtdm_printk("rtcan_mscan: device failed to enter sleep mode. "
				"We proceed anyhow.\n");
	else
		dev->state = CAN_STATE_SLEEPING;

	rinit = 0;
	setbits8(&regs->canctl0, MSCAN_INITRQ);

	reg = in_8(&regs->canctl1);
	while (!(reg & MSCAN_INITAK) &&
	        (rinit < MSCAN_SET_MODE_RETRIES)) {
		if (likely(lock_ctx != NULL))
			rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
		/* Busy sleep 1 microsecond */
		rtdm_task_busy_sleep(1000);
		if (likely(lock_ctx != NULL))
			rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
		rinit++;
		reg = in_8(&regs->canctl1);
	}
	if (rinit >= MSCAN_SET_MODE_RETRIES)
		ret = -ENODEV;

	/* Volatile state could have changed while we slept busy. */
	dev->state = CAN_STATE_STOPPED;
	/* Wake up waiting senders */
	rtdm_sem_destroy(&dev->tx_sem);

out:
	return ret;
}

/**
 *   Set controller into operating mode.
 *
 *   Called from @ref rtcan_mscan_ioctl in spin locked and IRQ disabled
 *   context.
 *
 *   @param dev_id   Device ID
 *   @param lock_ctx Pointer to saved IRQ context (only used when coming
 *                   from @ref CAN_STATE_SLEEPING, see also note)
 *
 *   @return 0 on success, otherwise:
 *   - -EINVAL: No Baud rate set before request to set start mode
 *
 *   @note If coming from @c CAN_STATE_SLEEPING, the controller must wait
 *         some time to avoid bus errors. Measured on an PHYTEC eNET card,
 *         this time was 110 microseconds.
 */
static int rtcan_mscan_mode_start(struct rtcan_device *dev,
				  rtdm_lockctx_t *lock_ctx)
{
	int ret = 0, retries = 0;
	can_state_t state;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	/* We won't forget that state in the device structure is volatile and
	 * access to it will not be optimized by the compiler. So ... */
	state = dev->state;

	switch (state) {
	case CAN_STATE_ACTIVE:
	case CAN_STATE_BUS_WARNING:
	case CAN_STATE_BUS_PASSIVE:
		break;

	case CAN_STATE_SLEEPING:
	case CAN_STATE_STOPPED:
		/* Set error active state */
		state = CAN_STATE_ACTIVE;
		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);

		if ((dev->ctrl_mode & CAN_CTRLMODE_LISTENONLY)) {
			setbits8(&regs->canctl1, MSCAN_LISTEN);
		} else {
			clrbits8(&regs->canctl1, MSCAN_LISTEN);
		}
		if ((dev->ctrl_mode & CAN_CTRLMODE_LOOPBACK)) {
			setbits8(&regs->canctl1, MSCAN_LOOPB);
		} else {
			clrbits8(&regs->canctl1, MSCAN_LOOPB);
		}

		/* Switch to normal mode */
		clrbits8(&regs->canctl0, MSCAN_INITRQ);
		clrbits8(&regs->canctl0, MSCAN_SLPRQ);
		while ((in_8(&regs->canctl1) & MSCAN_INITAK) ||
		       (in_8(&regs->canctl1) & MSCAN_SLPAK)) {
			if (likely(lock_ctx != NULL))
				rtdm_lock_put_irqrestore(&dev->device_lock,
							 *lock_ctx);
			/* Busy sleep 1 microsecond */
			rtdm_task_busy_sleep(1000);
			if (likely(lock_ctx != NULL))
				rtdm_lock_get_irqsave(&dev->device_lock,
						      *lock_ctx);
			retries++;
		}
		/* Enable interrupts */
		setbits8(&regs->canrier, MSCAN_RIER);

		break;

	case CAN_STATE_BUS_OFF:
		/* Trigger bus-off recovery */
		out_8(&regs->canrier, MSCAN_RIER);
		/* Set up sender "mutex" */
		rtdm_sem_init(&dev->tx_sem, 1);
		/* Set error active state */
		state = CAN_STATE_ACTIVE;

		break;

	default:
		/* Never reached, but we don't want nasty compiler warnings */
		break;
	}
	/* Store new state in device structure (or old state) */
	dev->state = state;

	return ret;
}

static int rtcan_mscan_set_bit_time(struct rtcan_device *dev,
				    struct can_bittime *bit_time,
				    rtdm_lockctx_t *lock_ctx)
{
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;
	u8 btr0, btr1;

	switch (bit_time->type) {
	case CAN_BITTIME_BTR:
		btr0 = bit_time->btr.btr0;
		btr1 = bit_time->btr.btr1;
		break;

	case CAN_BITTIME_STD:
		btr0 = (BTR0_SET_BRP(bit_time->std.brp) |
			BTR0_SET_SJW(bit_time->std.sjw));
		btr1 = (BTR1_SET_TSEG1(bit_time->std.prop_seg +
				       bit_time->std.phase_seg1) |
			BTR1_SET_TSEG2(bit_time->std.phase_seg2) |
			BTR1_SET_SAM(bit_time->std.sam));
		break;

	default:
		return -EINVAL;
	}

	out_8(&regs->canbtr0, btr0);
	out_8(&regs->canbtr1, btr1);

	rtdm_printk("%s: btr0=0x%02x btr1=0x%02x\n", dev->name, btr0, btr1);

	return 0;
}

static int rtcan_mscan_set_mode(struct rtcan_device *dev,
				can_mode_t mode,
				rtdm_lockctx_t *lock_ctx)
{
	int ret = 0, retries = 0;
	can_state_t state;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	switch (mode) {

	case CAN_MODE_STOP:
		ret = rtcan_mscan_mode_stop(dev, lock_ctx);
		break;

	case CAN_MODE_START:
		ret = rtcan_mscan_mode_start(dev, lock_ctx);
		break;

	case CAN_MODE_SLEEP:

		state = dev->state;

		/* Controller must operate, otherwise go out */
		if (!CAN_STATE_OPERATING(state)) {
			ret = -ENETDOWN;
			goto mode_sleep_out;
		}

		/* Is controller sleeping yet? If yes, go out */
		if (state == CAN_STATE_SLEEPING)
			goto mode_sleep_out;

		/* Remember into which state to return when we
		 * wake up */
		dev->state_before_sleep = state;
		state = CAN_STATE_SLEEPING;

		/* Let's take a nap. (Now I REALLY understand
		 * the meaning of interrupts ...) */
		out_8(&regs->canrier, 0);
		out_8(&regs->cantier, 0);
		setbits8(&regs->canctl0,
			 MSCAN_SLPRQ /*| MSCAN_INITRQ*/ | MSCAN_WUPE);
		while (!(in_8(&regs->canctl1) & MSCAN_SLPAK)) {
			rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
			/* Busy sleep 1 microsecond */
			rtdm_task_busy_sleep(1000);
			rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
			if (retries++ >= 1000)
				break;
		}
		rtdm_printk("Fallen asleep after %d tries.\n", retries);
		clrbits8(&regs->canctl0, MSCAN_INITRQ);
		while ((in_8(&regs->canctl1) & MSCAN_INITAK)) {
			rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
			/* Busy sleep 1 microsecond */
			rtdm_task_busy_sleep(1000);
			rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
			if (retries++ >= 1000)
				break;
		}
		rtdm_printk("Back to normal after %d tries.\n", retries);
		out_8(&regs->canrier, MSCAN_WUPIE);

	mode_sleep_out:
		dev->state = state;
		break;

	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}

/**
 *  Start a transmission to a MSCAN
 *
 *  Inline function called within @ref rtcan_mscan_sendmsg.
 *  This is the completion of a send call when hardware access is granted.
 *  Spinlock is taken before calling this function.
 *
 *  @param[in] frame  Pointer to CAN frame which is about to be sent
 *  @param[in] dev Device ID
 */
static int rtcan_mscan_start_xmit(struct rtcan_device *dev, can_frame_t *frame)
{
	int             i, id;
	/* "Real" size of the payload */
	unsigned char   size;
	/* Content of frame information register */
	unsigned char   dlc;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	/* Is TX buffer empty? */
	if (!(in_8(&regs->cantflg) & MSCAN_TXE0)) {
		rtdm_printk("rtcan_mscan_start_xmit: TX buffer not empty");
		return -EIO;
	}
	/* Select the buffer we've found. */
	out_8(&regs->cantbsel, MSCAN_TXE0);

	/* Get DLC and ID */
	dlc = frame->can_dlc;

	/* If DLC exceeds 8 bytes adjust it to 8 (for the payload) */
	size = (dlc > 8) ? 8 : dlc;

	id = frame->can_id;
	if (frame->can_id & CAN_EFF_FLAG) {
		out_8(&regs->cantxfg.idr[0], (id & 0x1fe00000) >> 21);
		out_8(&regs->cantxfg.idr[1], ((id & 0x001c0000) >> 13) |
		      ((id & 0x00038000) >> 15) |
		      0x18); /* set SRR and IDE bits */

		out_8(&regs->cantxfg.idr[4], (id & 0x00007f80) >> 7);
		out_8(&regs->cantxfg.idr[5], (id & 0x0000007f) << 1);

		/* RTR? */
		if (frame->can_id & CAN_RTR_FLAG)
			setbits8(&regs->cantxfg.idr[5], 0x1);
		else {
			clrbits8(&regs->cantxfg.idr[5], 0x1);
			/* No RTR, write data bytes */
			for (i = 0; i < size; i++)
				out_8(&regs->cantxfg.dsr[i + (i / 2) * 2],
				      frame->data[i]);
		}

	} else {
		/* Send standard frame */

		out_8(&regs->cantxfg.idr[0], (id & 0x000007f8) >> 3);
		out_8(&regs->cantxfg.idr[1], (id & 0x00000007) << 5);

		/* RTR? */
		if (frame->can_id & CAN_RTR_FLAG)
			setbits8(&regs->cantxfg.idr[1], 0x10);
		else {
			clrbits8(&regs->cantxfg.idr[1], 0x10);
			/* No RTR, write data bytes */
			for (i = 0; i < size; i++)
				out_8(&regs->cantxfg.dsr[i + (i / 2) * 2],
				      frame->data[i]);
		}
	}

	out_8(&regs->cantxfg.dlr, frame->can_dlc);
	out_8(&regs->cantxfg.tbpr, 0);	/* all messages have the same prio */

	/* Trigger transmission. */
	out_8(&regs->cantflg, MSCAN_TXE0);

	/* Enable interrupt. */
	setbits8(&regs->cantier, MSCAN_TXIE0);

	return 0;
}

/**
 *  MSCAN Chip configuration
 *
 *  Called during @ref init_module. Here, the configuration registers which
 *  must be set only once are written with the right values. The controller
 *  is left in reset mode and goes into operating mode not until the IOCTL
 *  for starting it is triggered.
 *
 *  @param[in] dev Device ID of the controller to be configured
 */
static inline void __init mscan_chip_config(struct mscan_regs *regs,
					    int mscan_clksrc)
{
	/* Choose IP bus as clock source.
	 */
	if (mscan_clksrc)
		setbits8(&regs->canctl1, MSCAN_CLKSRC);
	clrbits8(&regs->canctl1, MSCAN_LISTEN);

	/* Configure MSCAN to accept all incoming messages.
	 */
	out_8(&regs->canidar0, 0x00);
	out_8(&regs->canidar1, 0x00);
	out_8(&regs->canidar2, 0x00);
	out_8(&regs->canidar3, 0x00);
	out_8(&regs->canidmr0, 0xFF);
	out_8(&regs->canidmr1, 0xFF);
	out_8(&regs->canidmr2, 0xFF);
	out_8(&regs->canidmr3, 0xFF);
	out_8(&regs->canidar4, 0x00);
	out_8(&regs->canidar5, 0x00);
	out_8(&regs->canidar6, 0x00);
	out_8(&regs->canidar7, 0x00);
	out_8(&regs->canidmr4, 0xFF);
	out_8(&regs->canidmr5, 0xFF);
	out_8(&regs->canidmr6, 0xFF);
	out_8(&regs->canidmr7, 0xFF);
	clrbits8(&regs->canidac, MSCAN_IDAM0 | MSCAN_IDAM1);
}

/**
 *  MSCAN Chip registration
 *
 *  Called during @ref init_module.
 *
 *  @param[in] dev Device ID of the controller to be registered
 *  @param[in] mscan_clksrc clock source to be used
 */
int rtcan_mscan_register(struct rtcan_device *dev, int irq, int mscan_clksrc)
{
	int ret;
	struct mscan_regs *regs;

	regs = (struct mscan_regs *)dev->base_addr;

	/* Enable MSCAN module. */
	setbits8(&regs->canctl1, MSCAN_CANE);
	udelay(100);

	/* Set dummy state for following call */
	dev->state = CAN_STATE_ACTIVE;

	/* Enter reset mode */
	rtcan_mscan_mode_stop(dev, NULL);

	/* Give device an interface name (so that programs using this driver
	   don't need to know the device ID) */

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	dev->hard_start_xmit = rtcan_mscan_start_xmit;
	dev->do_set_mode = rtcan_mscan_set_mode;
	dev->do_set_bit_time = rtcan_mscan_set_bit_time;
#ifndef CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD
	dev->bittiming_const = &mscan_bittiming_const;
#endif

	/* Register IRQ handler and pass device structure as arg */
	ret = rtdm_irq_request(&dev->irq_handle, irq, rtcan_mscan_interrupt,
			       0, RTCAN_DRV_NAME, (void *)dev);
	if (ret) {
		printk("ERROR! rtdm_irq_request for IRQ %d failed\n", irq);
		goto out_can_disable;
	}

	mscan_chip_config(regs, mscan_clksrc);

	/* Register RTDM device */
	ret = rtcan_dev_register(dev);
	if (ret) {
		printk(KERN_ERR
		       "ERROR while trying to register RTCAN device!\n");
		goto out_irq_free;
	}

	rtcan_mscan_create_proc(dev);

	return 0;

out_irq_free:
	rtdm_irq_free(&dev->irq_handle);

out_can_disable:
	/* Disable MSCAN module. */
	clrbits8(&regs->canctl1, MSCAN_CANE);

	return ret;
}

/**
 *  MSCAN Chip deregistration
 *
 *  Called during @ref cleanup_module
 *
 *  @param[in] dev Device ID of the controller to be registered
 */
int rtcan_mscan_unregister(struct rtcan_device *dev)
{
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	printk("Unregistering %s device %s\n", RTCAN_DRV_NAME, dev->name);

	rtcan_mscan_mode_stop(dev, NULL);
	rtdm_irq_free(&dev->irq_handle);
	rtcan_mscan_remove_proc(dev);
	rtcan_dev_unregister(dev);

	/* Disable MSCAN module. */
	clrbits8(&regs->canctl1, MSCAN_CANE);

	return 0;
}
