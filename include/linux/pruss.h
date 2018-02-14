/* SPDX-License-Identifier: GPL-2.0 */
/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

int pruss_intc_trigger(unsigned int irq);

#else

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
