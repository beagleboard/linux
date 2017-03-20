/*
 * Copyright (C) 2006 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_ASSERT_H
#define _COBALT_KERNEL_ASSERT_H

#include <linux/kconfig.h>
#include <cobalt/kernel/trace.h>
#include <cobalt/kernel/ancillaries.h>

#define XENO_INFO	KERN_INFO    "[Xenomai] "
#define XENO_WARNING	KERN_WARNING "[Xenomai] "
#define XENO_ERR	KERN_ERR     "[Xenomai] "

#define XENO_DEBUG(__subsys)				\
	IS_ENABLED(CONFIG_XENO_OPT_DEBUG_##__subsys)
#define XENO_ASSERT(__subsys, __cond)			\
	(!WARN_ON(XENO_DEBUG(__subsys) && !(__cond)))
#define XENO_BUG(__subsys)				\
	BUG_ON(XENO_DEBUG(__subsys))
#define XENO_BUG_ON(__subsys, __cond)			\
	BUG_ON(XENO_DEBUG(__subsys) && (__cond))
#define XENO_WARN(__subsys, __cond, __fmt...)		\
	WARN(XENO_DEBUG(__subsys) && (__cond), __fmt)
#define XENO_WARN_ON(__subsys, __cond)			\
	WARN_ON(XENO_DEBUG(__subsys) && (__cond))
#define XENO_WARN_ON_ONCE(__subsys, __cond)		\
	WARN_ON_ONCE(XENO_DEBUG(__subsys) && (__cond))
#ifdef CONFIG_SMP
#define XENO_BUG_ON_SMP(__subsys, __cond)		\
	XENO_BUG_ON(__subsys, __cond)
#define XENO_WARN_ON_SMP(__subsys, __cond)		\
	XENO_WARN_ON(__subsys, __cond)
#define XENO_WARN_ON_ONCE_SMP(__subsys, __cond)		\
	XENO_WARN_ON_ONCE(__subsys, __cond)
#else
#define XENO_BUG_ON_SMP(__subsys, __cond)		\
	do { } while (0)
#define XENO_WARN_ON_SMP(__subsys, __cond)		\
	do { } while (0)
#define XENO_WARN_ON_ONCE_SMP(__subsys, __cond)		\
	do { } while (0)
#endif

#define primary_mode_only()	XENO_BUG_ON(CONTEXT, ipipe_root_p)
#define secondary_mode_only()	XENO_BUG_ON(CONTEXT, !ipipe_root_p)
#define interrupt_only()	XENO_BUG_ON(CONTEXT, !xnsched_interrupt_p())
#define realtime_cpu_only()	XENO_BUG_ON(CONTEXT, !xnsched_supported_cpu(ipipe_processor_id()))
#define thread_only()		XENO_BUG_ON(CONTEXT, xnsched_interrupt_p())
#define irqoff_only()		XENO_BUG_ON(CONTEXT, hard_irqs_disabled() == 0)
#if XENO_DEBUG(LOCKING)
#define atomic_only()		XENO_BUG_ON(CONTEXT, (xnlock_is_owner(&nklock) && hard_irqs_disabled()) == 0)
#define preemptible_only()	XENO_BUG_ON(CONTEXT, xnlock_is_owner(&nklock) || hard_irqs_disabled())
#else
#define atomic_only()		XENO_BUG_ON(CONTEXT, hard_irqs_disabled() == 0)
#define preemptible_only()	XENO_BUG_ON(CONTEXT, hard_irqs_disabled() != 0)
#endif

#endif /* !_COBALT_KERNEL_ASSERT_H */
