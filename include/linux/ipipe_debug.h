/* -*- linux-c -*-
 * include/linux/ipipe_debug.h
 *
 * Copyright (C) 2012 Philippe Gerum <rpm@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __LINUX_IPIPE_DEBUG_H
#define __LINUX_IPIPE_DEBUG_H

#include <linux/ipipe_domain.h>

#ifdef CONFIG_IPIPE_DEBUG_CONTEXT

#include <asm/bug.h>

static inline int ipipe_disable_context_check(void)
{
	return xchg(raw_cpu_ptr(&ipipe_percpu.context_check), 0);
}

static inline void ipipe_restore_context_check(int old_state)
{
	__this_cpu_write(ipipe_percpu.context_check, old_state);
}

static inline void ipipe_context_check_off(void)
{
	int cpu;
	for_each_online_cpu(cpu)
		per_cpu(ipipe_percpu, cpu).context_check = 0;
}

static inline void ipipe_save_context_nmi(void)
{
	int state = ipipe_disable_context_check();
	__this_cpu_write(ipipe_percpu.context_check_saved, state);
}

static inline void ipipe_restore_context_nmi(void)
{
	ipipe_restore_context_check(__this_cpu_read(ipipe_percpu.context_check_saved));
}

#else	/* !CONFIG_IPIPE_DEBUG_CONTEXT */

static inline int ipipe_disable_context_check(void)
{
	return 0;
}

static inline void ipipe_restore_context_check(int old_state) { }

static inline void ipipe_context_check_off(void) { }

static inline void ipipe_save_context_nmi(void) { }

static inline void ipipe_restore_context_nmi(void) { }

#endif	/* !CONFIG_IPIPE_DEBUG_CONTEXT */

#ifdef CONFIG_IPIPE_DEBUG

#define ipipe_check_irqoff()					\
	do {							\
		if (WARN_ON_ONCE(!hard_irqs_disabled()))	\
			hard_local_irq_disable();		\
	} while (0)

#else /* !CONFIG_IPIPE_DEBUG */

static inline void ipipe_check_irqoff(void) { }

#endif /* !CONFIG_IPIPE_DEBUG */

#ifdef CONFIG_IPIPE_DEBUG_INTERNAL
#define IPIPE_WARN(c)		WARN_ON(c)
#define IPIPE_WARN_ONCE(c)	WARN_ON_ONCE(c)
#define IPIPE_BUG_ON(c)		BUG_ON(c)
#else
#define IPIPE_WARN(c)		do { (void)(c); } while (0)
#define IPIPE_WARN_ONCE(c)	do { (void)(c); } while (0)
#define IPIPE_BUG_ON(c)		do { (void)(c); } while (0)
#endif

#endif /* !__LINUX_IPIPE_DEBUG_H */
