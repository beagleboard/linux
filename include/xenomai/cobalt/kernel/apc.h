/*
 * Copyright (C) 2012 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_APC_H
#define _COBALT_KERNEL_APC_H

#include <linux/ipipe.h>
#include <asm/xenomai/machine.h>

/**
 * @addtogroup cobalt_core_apc
 * @{
 */

int xnapc_alloc(const char *name,
		void (*handler)(void *cookie),
		void *cookie);

void xnapc_free(int apc);

static inline void __xnapc_schedule(int apc)
{
	unsigned long *p = &raw_cpu_ptr(&cobalt_machine_cpudata)->apc_pending;

	if (!__test_and_set_bit(apc, p))
		ipipe_post_irq_root(cobalt_pipeline.apc_virq);
}

/**
 * @fn static inline int xnapc_schedule(int apc)
 *
 * @brief Schedule an APC invocation.
 *
 * This service marks the APC as pending for the Linux domain, so that
 * its handler will be called as soon as possible, when the Linux
 * domain gets back in control.
 *
 * When posted from the Linux domain, the APC handler is fired as soon
 * as the interrupt mask is explicitly cleared by some kernel
 * code. When posted from the Xenomai domain, the APC handler is
 * fired as soon as the Linux domain is resumed, i.e. after Xenomai has
 * completed all its pending duties.
 *
 * @param apc The APC id. to schedule.
 *
 * This service can be called from:
 *
 * - Any domain context, albeit the usual calling place is from the
 * Xenomai domain.
 */
static inline void xnapc_schedule(int apc)
{
	unsigned long flags;

	flags = ipipe_test_and_stall_head() & 1;
	__xnapc_schedule(apc);
	ipipe_restore_head(flags);
}

void apc_dispatch(unsigned int virq, void *arg);

/** @} */

#endif /* !_COBALT_KERNEL_APC_H */
