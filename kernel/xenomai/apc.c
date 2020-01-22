/*
 * Copyright (C) 2007,2012 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/spinlock.h>
#include <linux/ipipe.h>
#include <cobalt/kernel/apc.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_apc Asynchronous Procedure Calls
 *
 * Services for scheduling function calls in the Linux domain
 *
 * APC is the acronym for Asynchronous Procedure Call, a mean by which
 * activities from the Xenomai domain can schedule deferred
 * invocations of handlers to be run into the Linux domain, as soon as
 * possible when the Linux kernel gets back in control.
 *
 * Up to BITS_PER_LONG APC slots can be active at any point in time.
 *
 * APC support is built upon the interrupt pipeline's virtual
 * interrupt support.
 *
 * @{
 */
static IPIPE_DEFINE_SPINLOCK(apc_lock);

void apc_dispatch(unsigned int virq, void *arg)
{
	void (*handler)(void *), *cookie;
	unsigned long *p;
	int apc;

	/*
	 * CAUTION: The APC dispatch loop is not protected against a
	 * handler becoming unavailable while processing the pending
	 * queue; the software must make sure to uninstall all APCs
	 * before eventually unloading any module that may contain APC
	 * handlers. We keep the handler affinity with the poster's
	 * CPU, so that the handler is invoked on the same CPU than
	 * the code which called xnapc_schedule().
	 */
	raw_spin_lock(&apc_lock);

	/* This is atomic linux context (non-threaded IRQ). */
	p = &raw_cpu_ptr(&cobalt_machine_cpudata)->apc_pending;
	while (*p) {
		apc = ffnz(*p);
		clear_bit(apc, p);
		handler = cobalt_pipeline.apc_table[apc].handler;
		cookie = cobalt_pipeline.apc_table[apc].cookie;
		raw_cpu_ptr(&cobalt_machine_cpudata)->apc_shots[apc]++;
		raw_spin_unlock(&apc_lock);
		handler(cookie);
		raw_spin_lock(&apc_lock);
	}

	raw_spin_unlock(&apc_lock);
}

/**
 * @fn int xnapc_alloc(const char *name,void (*handler)(void *cookie),void *cookie)
 *
 * @brief Allocate an APC slot.
 *
 * APC is the acronym for Asynchronous Procedure Call, a mean by which
 * activities from the Xenomai domain can schedule deferred
 * invocations of handlers to be run into the Linux domain, as soon as
 * possible when the Linux kernel gets back in control. Up to
 * BITS_PER_LONG APC slots can be active at any point in time. APC
 * support is built upon the interrupt pipeline's virtual interrupt
 * support.
 *
 * Any Linux kernel service which is callable from a regular Linux
 * interrupt handler is in essence available to APC handlers.
 *
 * @param name is a symbolic name identifying the APC which will get
 * reported through the /proc/xenomai/apc interface. Passing NULL to
 * create an anonymous APC is allowed.
 *
 * @param handler The address of the fault handler to call upon
 * exception condition. The handle will be passed the @a cookie value
 * unmodified.
 *
 * @param cookie A user-defined opaque pointer the APC handler
 * receives as its sole argument.
 *
 * @return a valid APC identifier is returned upon success, or a
 * negative error code otherwise:
 *
 * - -EINVAL is returned if @a handler is invalid.
 *
 * - -EBUSY is returned if no more APC slots are available.
 *
 * @coretags{unrestricted}
 */
int xnapc_alloc(const char *name,
		void (*handler)(void *cookie), void *cookie)
{
	unsigned long flags;
	int apc;

	if (handler == NULL)
		return -EINVAL;

	raw_spin_lock_irqsave(&apc_lock, flags);

	if (cobalt_pipeline.apc_map == ~0) {
		apc = -EBUSY;
		goto out;
	}

	apc = ffz(cobalt_pipeline.apc_map);
	__set_bit(apc, &cobalt_pipeline.apc_map);
	cobalt_pipeline.apc_table[apc].handler = handler;
	cobalt_pipeline.apc_table[apc].cookie = cookie;
	cobalt_pipeline.apc_table[apc].name = name;
out:
	raw_spin_unlock_irqrestore(&apc_lock, flags);

	return apc;
}
EXPORT_SYMBOL_GPL(xnapc_alloc);

/**
 * @fn int xnapc_free(int apc)
 *
 * @brief Releases an APC slot.
 *
 * This service deallocates an APC slot obtained by xnapc_alloc().
 *
 * @param apc The APC id. to release, as returned by a successful call
 * to the xnapc_alloc() service.
 *
 * @coretags{unrestricted}
 */
void xnapc_free(int apc)
{
	BUG_ON(apc < 0 || apc >= BITS_PER_LONG);
	clear_bit(apc, &cobalt_pipeline.apc_map);
	smp_mb__after_atomic();
}
EXPORT_SYMBOL_GPL(xnapc_free);

/** @} */
