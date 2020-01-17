/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * TI PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef __LINUX_PRUSS_INTC_H
#define __LINUX_PRUSS_INTC_H

#include <linux/interrupt.h>

#if IS_ENABLED(CONFIG_TI_PRUSS_INTC)

/**
 * pruss_intc_trigger() - trigger a PRU system event
 * @irq: linux IRQ number associated with a PRU system event
 *
 * Trigger an interrupt by signalling a specific PRU system event.
 * This can be used by PRUSS client users to raise/send an event to
 * a PRU or any other core that is listening on the host interrupt
 * mapped to that specific PRU system event. The @irq variable is the
 * Linux IRQ number associated with a specific PRU system event that
 * a client user/application uses. The interrupt mappings for this is
 * provided by the PRUSS INTC irqchip instance.
 *
 * Returns 0 on success, or an error value upon failure.
 */
static inline int pruss_intc_trigger(unsigned int irq)
{
	return irq_set_irqchip_state(irq, IRQCHIP_STATE_PENDING, true);
}

#else

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_TI_PRUSS_INTC */

#endif /* __LINUX_PRUSS_INTC_H */
