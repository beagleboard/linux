/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
