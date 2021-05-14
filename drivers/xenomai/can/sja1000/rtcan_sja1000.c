/*
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * Parts of this software are based on the following:
 *
 * - RTAI CAN device driver for SJA1000 controllers by Jan Kiszka
 *
 * - linux-can.patch, a CAN socket framework for Linux,
 *   Copyright (C) 2004, 2005, Robert Schwebel, Benedikt Spranger,
 *   Marc Kleine-Budde, Sascha Hauer, Pengutronix
 *
 * - RTnet (www.rtnet.org)
 *
 * - serial device driver and profile included in Xenomai (RTDM),
 *   Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>.
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

#include <rtdm/driver.h>
#include <rtdm/can.h>

#include <rtcan_socket.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_list.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>


#define BTR0_BRP_MASK	0x3f
#define BTR0_SJW_SHIFT	6
#define BTR0_SJW_MASK	(0x3 << BTR0_SJW_SHIFT)

#define BTR1_TSEG1_MASK  0xf
#define BTR1_TSEG2_SHIFT 4
#define BTR1_TSEG2_MASK  (0x7 << BTR1_TSEG2_SHIFT)
#define BTR1_SAM_SHIFT   7

#define BTR0_SET_BRP(brp)     (((brp) - 1) & BTR0_BRP_MASK)
#define BTR0_SET_SJW(sjw)     ((((sjw) - 1) << BTR0_SJW_SHIFT) & BTR0_SJW_MASK)

#define BTR1_SET_TSEG1(tseg1) (((tseg1) - 1) & BTR1_TSEG1_MASK)
#define BTR1_SET_TSEG2(tseg2) ((((tseg2) - 1) << BTR1_TSEG2_SHIFT) & BTR1_TSEG2_MASK)
#define BTR1_SET_SAM(sam)     (((sam) & 1) << BTR1_SAM_SHIFT)

/* Value for the interrupt enable register */
#define SJA1000_IER                 SJA_IER_RIE | SJA_IER_TIE | \
				    SJA_IER_EIE | SJA_IER_WUIE | \
				    SJA_IER_EPIE | SJA_IER_BEIE | \
				    SJA_IER_ALIE | SJA_IER_DOIE

static char *sja_ctrl_name = "SJA1000";

#define STATE_OPERATING(state) \
    ((state) != CAN_STATE_STOPPED && (state) != CAN_STATE_BUS_OFF)

#define STATE_RESET(state) \
    ((state) == CAN_STATE_STOPPED || (state) == CAN_STATE_BUS_OFF)


MODULE_AUTHOR("Sebastian.Smolorz@stud.uni-hannover.de");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RT-Socket-CAN driver for SJA1000");
MODULE_SUPPORTED_DEVICE("SJA1000 CAN controller");

#ifndef CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD
static struct can_bittiming_const sja1000_bittiming_const = {
	.name = "sja1000",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};
#endif

static inline void rtcan_sja_rx_interrupt(struct rtcan_device *dev,
					  struct rtcan_skb *skb)
{
    int i;
    /* "Real" size of the payload */
    u8 size;
    /* Content of frame information register */
    u8 fir;
    /* Ring buffer frame within skb */
    struct rtcan_rb_frame *frame = &skb->rb_frame;
    struct rtcan_sja1000 *chip = dev->priv;

    /* Read out frame information register */
    fir = chip->read_reg(dev, SJA_FIR);

    /* Extract data length code */
    frame->can_dlc = fir & SJA_FIR_DLC_MASK;

    /* If DLC exceeds 8 bytes adjust it to 8 (for the payload size) */
    size = (frame->can_dlc > 8) ? 8 : frame->can_dlc;


    if (fir & SJA_FIR_EFF) {
	/* Extended frame */
	frame->can_id = CAN_EFF_FLAG;

	/* Read ID */
	frame->can_id |= chip->read_reg(dev, SJA_ID1) << 21;
	frame->can_id |= chip->read_reg(dev, SJA_ID2) << 13;
	frame->can_id |= chip->read_reg(dev, SJA_ID3) << 5;
	frame->can_id |= chip->read_reg(dev, SJA_ID4) >> 3;

	if (!(fir & SJA_FIR_RTR)) {
	    /* No RTR, read data bytes */
	    for (i = 0; i < size; i++)
		frame->data[i] = chip->read_reg(dev,
						SJA_DATA_EFF(i));
	}

    } else {
	/* Standard frame */

	/* Read ID */
	frame->can_id  = chip->read_reg(dev, SJA_ID1) << 3;
	frame->can_id |= chip->read_reg(dev, SJA_ID2) >> 5;

	if (!(fir & SJA_FIR_RTR)) {
	    /* No RTR, read data bytes */
	    for (i = 0; i < size; i++)
		frame->data[i] = chip->read_reg(dev, SJA_DATA_SFF(i));
	}
    }

    /* Release Receive Buffer */
    chip->write_reg(dev, SJA_CMR, SJA_CMR_RRB);


    /* RTR? */
    if (fir & SJA_FIR_RTR) {
	frame->can_id |= CAN_RTR_FLAG;
	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE;
    } else
	skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + size;

    /* Store the interface index */
    frame->can_ifindex = dev->ifindex;
}


static inline void rtcan_sja_err_interrupt(struct rtcan_device *dev,
					   struct rtcan_sja1000 *chip,
					   struct rtcan_skb *skb,
					   u8 irq_source)
{
    struct rtcan_rb_frame *frame = &skb->rb_frame;
    can_state_t state = dev->state;
    u8 status, txerr, rxerr;

    status = chip->read_reg(dev, SJA_SR);
    txerr = chip->read_reg(dev, SJA_TXERR);
    rxerr = chip->read_reg(dev, SJA_RXERR);

    skb->rb_frame_size = EMPTY_RB_FRAME_SIZE + CAN_ERR_DLC;

    frame->can_id = CAN_ERR_FLAG;
    frame->can_dlc = CAN_ERR_DLC;

    memset(&frame->data[0], 0, frame->can_dlc);

    /* Data overrun interrupt? */
    if (irq_source & SJA_IR_DOI) {
	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
    }

    /* Arbitratio lost interrupt? */
    if (irq_source & SJA_IR_ALI) {
	frame->can_id |= CAN_ERR_LOSTARB;
	frame->data[0] = chip->read_reg(dev, SJA_ALC)  & 0x1f;
    }

    /* Bus error interrupt? */
    if (irq_source & SJA_IR_BEI) {
	u8 ecc = chip->read_reg(dev, SJA_ECC);

	frame->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	switch (ecc & SJA_ECC_ERR_MASK) {
	case SJA_ECC_ERR_BIT:
	    frame->data[2] |= CAN_ERR_PROT_BIT;
	    break;
	case SJA_ECC_ERR_FORM:
	    frame->data[2] |= CAN_ERR_PROT_FORM;
	    break;
	case SJA_ECC_ERR_STUFF:
	    frame->data[2] |= CAN_ERR_PROT_STUFF;
	    break;
	default:
	    frame->data[2] |= CAN_ERR_PROT_UNSPEC;
	    frame->data[3] = ecc & SJA_ECC_SEG_MASK;
	    break;
	}
	/* Error occured during transmission? */
	if ((ecc & SJA_ECC_DIR) == 0)
	    frame->data[2] |= CAN_ERR_PROT_TX;
    }

    /* Error passive interrupt? */
    if (unlikely(irq_source & SJA_IR_EPI)) {
	if (state == CAN_STATE_BUS_WARNING) {
	    state = CAN_STATE_BUS_PASSIVE;
	} else {
	    state = CAN_STATE_BUS_WARNING;
	}
    }

    /* Error warning interrupt? */
    if (irq_source & SJA_IR_EI) {

	/* Test bus status (bus-off condition) */
	if (status & SJA_SR_BS) {
	    /* Bus-off */
	    state = CAN_STATE_BUS_OFF;
	    frame->can_id |= CAN_ERR_BUSOFF;
	    /* Only allow error warning interrupts
	       (otherwise an EPI would arise during bus-off
	       recovery) */
	    chip->write_reg(dev, SJA_IER, SJA_IER_EIE);
	    /* Wake up waiting senders */
	    rtdm_sem_destroy(&dev->tx_sem);
	}

	/* Test error status (error warning limit) */
	else if (status & SJA_SR_ES)
	    /* error warning limit reached */
	    state = CAN_STATE_BUS_WARNING;

	/* Re-entrance into error active state from bus-warn? */
	else if (state == CAN_STATE_BUS_WARNING)
	    state = CAN_STATE_ACTIVE;

	else
	    /* Bus-off recovery complete, enable all interrupts again */
	    chip->write_reg(dev, SJA_IER, SJA1000_IER);
    }

    if (state != dev->state &&
	(state == CAN_STATE_BUS_WARNING || state == CAN_STATE_BUS_PASSIVE)) {
	frame->can_id |= CAN_ERR_PROT;
	if (txerr > rxerr)
	    frame->data[1] = CAN_ERR_CRTL_TX_WARNING;
	else
	    frame->data[1] = CAN_ERR_CRTL_RX_WARNING;
    }

    dev->state = state;
    frame->can_ifindex = dev->ifindex;
}

static int rtcan_sja_interrupt(rtdm_irq_t *irq_handle)
{
    struct rtcan_device *dev;
    struct rtcan_sja1000 *chip;
    struct rtcan_skb skb;
    int recv_lock_free = 1;
    int irq_count = 0;
    int ret = RTDM_IRQ_NONE;
    u8 irq_source;


    /* Get the ID of the device which registered this IRQ. */
    dev = (struct rtcan_device *)rtdm_irq_get_arg(irq_handle, void);
    chip = (struct rtcan_sja1000 *)dev->priv;

    /* Take spinlock protecting HW register access and device structures. */
    rtdm_lock_get(&dev->device_lock);

    /* Loop as long as the device reports an event */
    while ((irq_source = chip->read_reg(dev, SJA_IR))) {
	ret = RTDM_IRQ_HANDLED;
	irq_count++;

	/* Now look up which interrupts appeared */

	/* Wake-up interrupt? */
	if (irq_source & SJA_IR_WUI)
	    dev->state = dev->state_before_sleep;

	/* Error Interrupt? */
	if (irq_source & (SJA_IR_EI | SJA_IR_DOI | SJA_IR_EPI |
			  SJA_IR_ALI | SJA_IR_BEI)) {

	    /* Check error condition and fill error frame */
	    if (!((irq_source & SJA_IR_BEI) && (chip->bus_err_on-- < 2))) {
		rtcan_sja_err_interrupt(dev, chip, &skb, irq_source);

		if (recv_lock_free) {
		    recv_lock_free = 0;
		    rtdm_lock_get(&rtcan_recv_list_lock);
		    rtdm_lock_get(&rtcan_socket_lock);
		}
		/* Pass error frame out to the sockets */
		rtcan_rcv(dev, &skb);
	    }
	}

	/* Transmit Interrupt? */
	if (irq_source & SJA_IR_TI) {
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

	/* Receive Interrupt? */
	if (irq_source & SJA_IR_RI) {

	    /* Read out HW registers */
	    rtcan_sja_rx_interrupt(dev, &skb);

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
    }

    if (chip->irq_ack)
	chip->irq_ack(dev);

    /* Release spinlocks */
    if (!recv_lock_free) {
	rtdm_lock_put(&rtcan_socket_lock);
	rtdm_lock_put(&rtcan_recv_list_lock);
    }
    rtdm_lock_put(&dev->device_lock);

    return ret;
}



/*
 * Inline function to decide if controller is operating
 *
 * Catch the very unlikely case that setting stop mode
 * returned without success before this call but in the
 * meantime the controller went into reset mode.
 */
static inline int rtcan_sja_is_operating(struct rtcan_device *dev,
					 can_state_t *state)
{
    int is_operating = STATE_OPERATING(*state);
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;

    if (unlikely(is_operating && chip->read_reg(dev, SJA_MOD) & SJA_MOD_RM)) {
	*state = CAN_STATE_STOPPED;
	is_operating = 0;
	/* Disable the controller's interrupts */
	chip->write_reg(dev, SJA_IER, 0x00);
	/* Wake up waiting senders */
	rtdm_sem_destroy(&dev->tx_sem);
    }

    return is_operating;
}


/*
 * Set controller into reset mode.
 *
 * According to the SJA1000 specification, it is necessary to check the
 * reset mode bit in PeliCAN mode after having set it. So we do. But if
 * using a ISA card like the PHYTEC eNET card this should not be necessary
 * because the CAN controller clock of this card (16 MHz) is twice as high
 * as the ISA bus clock.
 */
static int rtcan_sja_mode_stop(struct rtcan_device *dev,
			       rtdm_lockctx_t *lock_ctx)
{
    int ret = 0;
    /* Max. 50 loops busy sleep. If the controller is stopped while in
     * sleep mode 20-40 loops are needed (tested on PHYTEC eNET). */
    int wait_loop = 50;
    can_state_t state;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;

    state = dev->state;
    /* If controller is not operating anyway, go out */
    if (STATE_RESET(state))
	goto out;

    /* Disable the controller's interrupts */
    chip->write_reg(dev, SJA_IER, 0x00);

    /* Set reset mode bit */
    chip->write_reg(dev, SJA_MOD, SJA_MOD_RM);

    /* Read reset mode bit, multiple tests */
    do {
	if (chip->read_reg(dev, SJA_MOD) & SJA_MOD_RM)
	    break;

	if (lock_ctx)
	    rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
	/* Busy sleep 1 microsecond */
	rtdm_task_busy_sleep(1000);
	if (lock_ctx)
	    rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);
    } while(--wait_loop);


    if (wait_loop) {
	/* Volatile state could have changed while we slept busy. */
	dev->state = CAN_STATE_STOPPED;
	/* Wake up waiting senders */
	rtdm_sem_destroy(&dev->tx_sem);
    } else {
	ret = -EAGAIN;
	/* Enable interrupts again as we did not succeed */
	chip->write_reg(dev, SJA_IER, SJA1000_IER);
    }

 out:
    return ret;
}



/*
 * Set controller into operating mode.
 *
 * If coming from CAN_STATE_SLEEPING, the controller must wait
 * some time to avoid bus errors. Measured on an PHYTEC eNET card,
 * this time was 110 microseconds.
 */
static int rtcan_sja_mode_start(struct rtcan_device *dev,
				rtdm_lockctx_t *lock_ctx)
{
    int ret = 0;
    u8 mod_reg;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;

    /* We won't forget that state in the device structure is volatile and
     * access to it will not be optimized by the compiler. So ... */

    mod_reg = 0;
    if (dev->ctrl_mode & CAN_CTRLMODE_LISTENONLY)
	mod_reg |= SJA_MOD_LOM;
    if (dev->ctrl_mode & CAN_CTRLMODE_LOOPBACK)
	mod_reg |= SJA_MOD_STM;

    switch (dev->state) {

    case CAN_STATE_ACTIVE:
    case CAN_STATE_BUS_WARNING:
    case CAN_STATE_BUS_PASSIVE:
	break;

    case CAN_STATE_STOPPED:
	/* Clear error counters */
	chip->write_reg(dev, SJA_RXERR , 0);
	chip->write_reg(dev, SJA_TXERR , 0);
	/* Clear error code capture (i.e. read it) */
	chip->read_reg(dev, SJA_ECC);
	/* Set error active state */
	dev->state = CAN_STATE_ACTIVE;
	/* Set up sender "mutex" */
	rtdm_sem_init(&dev->tx_sem, 1);
	/* Enable interrupts */
	chip->write_reg(dev, SJA_IER, SJA1000_IER);

	/* Clear reset mode bit in SJA1000 */
	chip->write_reg(dev, SJA_MOD, mod_reg);

	break;

    case CAN_STATE_SLEEPING:
	/* Trigger Wake-up interrupt */
	chip->write_reg(dev, SJA_MOD, mod_reg);

	/* Ok, coming from sleep mode is problematic. We have to wait
	 * for the SJA1000 to get on both feet again. */
	rtdm_lock_put_irqrestore(&dev->device_lock, *lock_ctx);
	rtdm_task_busy_sleep(110000);
	rtdm_lock_get_irqsave(&dev->device_lock, *lock_ctx);

	/* Meanwhile, the Wake-up interrupt was serviced and has set the
	 * right state. As we don't want to set it back jump out. */
	goto out;

	break;

    case CAN_STATE_BUS_OFF:
	/* Trigger bus-off recovery */
	chip->write_reg(dev, SJA_MOD, mod_reg);
	/* Set up sender "mutex" */
	rtdm_sem_init(&dev->tx_sem, 1);
	/* Set error active state */
	dev->state = CAN_STATE_ACTIVE;

	break;

    default:
	/* Never reached, but we don't want nasty compiler warnings ... */
	break;
    }

 out:
    return ret;
}

can_state_t rtcan_sja_get_state(struct rtcan_device *dev)
{
    can_state_t state = dev->state;
    rtcan_sja_is_operating(dev, &state);
    return state;
}

int rtcan_sja_set_mode(struct rtcan_device *dev,
		       can_mode_t mode,
		       rtdm_lockctx_t *lock_ctx)
{
    int ret = 0;
    can_state_t state;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000*)dev->priv;

    switch (mode) {

    case CAN_MODE_STOP:
	ret = rtcan_sja_mode_stop(dev, lock_ctx);
	break;

    case CAN_MODE_START:
	ret = rtcan_sja_mode_start(dev, lock_ctx);
	break;

    case CAN_MODE_SLEEP:

	state = dev->state;

	/* Controller must operate, otherwise go out */
	if (!rtcan_sja_is_operating(dev, &state)) {
	    ret = -ENETDOWN;
	    goto mode_sleep_out;
	}

	/* Is controller sleeping yet? If yes, go out */
	if (state == CAN_STATE_SLEEPING)
	    goto mode_sleep_out;

	/* Remember into which state to return when we
	 * wake up */
	dev->state_before_sleep = state;

	/* Let's take a nap. (Now I REALLY understand
	 * the meaning of interrupts ...) */
	state = CAN_STATE_SLEEPING;
	chip->write_reg(dev, SJA_MOD,
			chip->read_reg(dev, SJA_MOD) | SJA_MOD_SM);

    mode_sleep_out:
	dev->state = state;
	break;

    default:
	ret = -EOPNOTSUPP;
	break;
    }

    return ret;
}

int rtcan_sja_set_bit_time(struct rtcan_device *dev,
			   struct can_bittime *bit_time,
			   rtdm_lockctx_t *lock_ctx)
{
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;
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

    printk("%s: btr0=%#x btr1=%#x\n", __func__, btr0, btr1);
    chip->write_reg(dev, SJA_BTR0, btr0);
    chip->write_reg(dev, SJA_BTR1, btr1);

    return 0;
}

void rtcan_sja_enable_bus_err(struct rtcan_device *dev)
{
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;

    if (chip->bus_err_on < 2) {
	if (chip->bus_err_on < 1)
	    chip->read_reg(dev, SJA_ECC);
	chip->bus_err_on = 2;
    }
}

/*
 *  Start a transmission to a SJA1000 device
 */
static int rtcan_sja_start_xmit(struct rtcan_device *dev,
				can_frame_t *frame)
{
    int             i;
    /* "Real" size of the payload */
    u8   size;
    /* Content of frame information register */
    u8   fir;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;

    /* Get DLC */
    fir  = frame->can_dlc;

    /* If DLC exceeds 8 bytes adjust it to 8 (for the payload) */
    size = (fir > 8) ? 8 : fir;


    if (frame->can_id & CAN_EFF_FLAG) {
	/* Send extended frame */
	fir |= SJA_FIR_EFF;

	/* Write ID */
	chip->write_reg(dev, SJA_ID1, frame->can_id >> 21);
	chip->write_reg(dev, SJA_ID2, frame->can_id >> 13);
	chip->write_reg(dev, SJA_ID3, frame->can_id >> 5);
	chip->write_reg(dev, SJA_ID4, frame->can_id << 3);

	/* RTR? */
	if (frame->can_id & CAN_RTR_FLAG)
	    fir |= SJA_FIR_RTR;

	else {
	    /* No RTR, write data bytes */
	    for (i = 0; i < size; i++)
		chip->write_reg(dev, SJA_DATA_EFF(i),
				frame->data[i]);
	}

    } else {
	/* Send standard frame */

	/* Write ID */
	chip->write_reg(dev, SJA_ID1, frame->can_id >> 3);
	chip->write_reg(dev, SJA_ID2, frame->can_id << 5);

	/* RTR? */
	if (frame->can_id & CAN_RTR_FLAG)
	    fir |= SJA_FIR_RTR;

	else {
	    /* No RTR, write data bytes */
	    for (i = 0; i < size; i++)
		chip->write_reg(dev, SJA_DATA_SFF(i),
				frame->data[i]);
	}
    }


    /* Write frame information register */
    chip->write_reg(dev, SJA_FIR, fir);

    /* Push the 'send' button */
    if (dev->ctrl_mode & CAN_CTRLMODE_LOOPBACK)
	chip->write_reg(dev, SJA_CMR, SJA_CMR_SRR);
    else
	chip->write_reg(dev, SJA_CMR, SJA_CMR_TR);

    return 0;
}



/*
 *  SJA1000 chip configuration
 */
static void sja1000_chip_config(struct rtcan_device *dev)
{
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000* )dev->priv;

    chip->write_reg(dev, SJA_CDR, chip->cdr);
    chip->write_reg(dev, SJA_OCR, chip->ocr);

    chip->write_reg(dev, SJA_AMR0, 0xFF);
    chip->write_reg(dev, SJA_AMR1, 0xFF);
    chip->write_reg(dev, SJA_AMR2, 0xFF);
    chip->write_reg(dev, SJA_AMR3, 0xFF);
}


int rtcan_sja1000_register(struct rtcan_device *dev)
{
    int                         ret;
    struct rtcan_sja1000 *chip = dev->priv;

    if (chip == NULL)
	return -EINVAL;

    /* Set dummy state for following call */
    dev->state = CAN_STATE_ACTIVE;
    /* Enter reset mode */
    rtcan_sja_mode_stop(dev, NULL);

    if ((chip->read_reg(dev, SJA_SR) &
	 (SJA_SR_RBS | SJA_SR_DOS | SJA_SR_TBS)) != SJA_SR_TBS) {
	printk("ERROR! No SJA1000 device found!\n");
	return -ENODEV;
    }

    dev->ctrl_name = sja_ctrl_name;

    dev->hard_start_xmit = rtcan_sja_start_xmit;
    dev->do_set_mode = rtcan_sja_set_mode;
    dev->do_get_state = rtcan_sja_get_state;
    dev->do_set_bit_time = rtcan_sja_set_bit_time;
    dev->do_enable_bus_err = rtcan_sja_enable_bus_err;
#ifndef CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD
    dev->bittiming_const = &sja1000_bittiming_const;
#endif

    chip->bus_err_on = 1;

    ret = rtdm_irq_request(&dev->irq_handle,
			   chip->irq_num, rtcan_sja_interrupt,
			   chip->irq_flags, sja_ctrl_name, dev);
    if (ret) {
	printk(KERN_ERR "ERROR %d: IRQ %d is %s!\n",
	       ret, chip->irq_num, ret == -EBUSY ?
	       "busy, check shared interrupt support" : "invalid");
	return ret;
    }

    sja1000_chip_config(dev);

    /* Register RTDM device */
    ret = rtcan_dev_register(dev);
    if (ret) {
	    printk(KERN_ERR
		   "ERROR %d while trying to register RTCAN device!\n", ret);
	goto out_irq_free;
    }

    rtcan_sja_create_proc(dev);

    return 0;

 out_irq_free:
    rtdm_irq_free(&dev->irq_handle);

    return ret;
}


/* Cleanup module */
void rtcan_sja1000_unregister(struct rtcan_device *dev)
{
    printk("Unregistering SJA1000 device %s\n", dev->name);

    rtdm_irq_disable(&dev->irq_handle);
    rtcan_sja_mode_stop(dev, NULL);
    rtdm_irq_free(&dev->irq_handle);
    rtcan_sja_remove_proc(dev);
    rtcan_dev_unregister(dev);
}

int __init rtcan_sja_init(void)
{
	if (!rtdm_available())
		return -ENOSYS;

	printk("RTCAN SJA1000 driver initialized\n");
	return 0;
}


void __exit rtcan_sja_exit(void)
{
	printk("%s removed\n", sja_ctrl_name);
}

module_init(rtcan_sja_init);
module_exit(rtcan_sja_exit);

EXPORT_SYMBOL_GPL(rtcan_sja1000_register);
EXPORT_SYMBOL_GPL(rtcan_sja1000_unregister);
