/* -*- linux-c -*-
 * include/asm-generic/ipipe.h
 *
 * Copyright (C) 2002-2017 Philippe Gerum.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 */
#ifndef __ASM_GENERIC_IPIPE_H
#define __ASM_GENERIC_IPIPE_H

#ifdef CONFIG_IPIPE

#if defined(CONFIG_DEBUG_ATOMIC_SLEEP) || defined(CONFIG_PROVE_LOCKING) || \
	defined(CONFIG_PREEMPT_VOLUNTARY) || defined(CONFIG_IPIPE_DEBUG_CONTEXT)
void __ipipe_uaccess_might_fault(void);
#else
#define __ipipe_uaccess_might_fault() might_fault()
#endif

#define hard_cond_local_irq_enable()		hard_local_irq_enable()
#define hard_cond_local_irq_disable()		hard_local_irq_disable()
#define hard_cond_local_irq_save()		hard_local_irq_save()
#define hard_cond_local_irq_restore(flags)	hard_local_irq_restore(flags)

#ifdef CONFIG_IPIPE_DEBUG_CONTEXT
void ipipe_root_only(void);
#else /* !CONFIG_IPIPE_DEBUG_CONTEXT */
static inline void ipipe_root_only(void) { }
#endif /* !CONFIG_IPIPE_DEBUG_CONTEXT */

void ipipe_stall_root(void);

void ipipe_unstall_root(void);

unsigned long ipipe_test_and_stall_root(void);

unsigned long ipipe_test_root(void);

void ipipe_restore_root(unsigned long x);

#else  /* !CONFIG_IPIPE */

#define hard_local_irq_save_notrace()		\
	({					\
		unsigned long __flags;		\
		raw_local_irq_save(__flags);	\
		__flags;			\
	})

#define hard_local_irq_restore_notrace(__flags)	\
	raw_local_irq_restore(__flags)

#define hard_local_irq_enable_notrace()	\
	raw_local_irq_enable()

#define hard_local_irq_disable_notrace()	\
	raw_local_irq_disable()

#define hard_local_irq_save()			\
	({					\
		unsigned long __flags;		\
		local_irq_save(__flags);	\
		__flags;			\
	})
#define hard_local_irq_restore(__flags)	local_irq_restore(__flags)
#define hard_local_irq_enable()		local_irq_enable()
#define hard_local_irq_disable()	local_irq_disable()
#define hard_irqs_disabled()		irqs_disabled()

#define hard_cond_local_irq_enable()		do { } while(0)
#define hard_cond_local_irq_disable()		do { } while(0)
#define hard_cond_local_irq_save()		0
#define hard_cond_local_irq_restore(__flags)	do { (void)(__flags); } while(0)

#define __ipipe_uaccess_might_fault()		might_fault()

static inline void ipipe_root_only(void) { }

#endif /* !CONFIG_IPIPE */

#if defined(CONFIG_SMP) && defined(CONFIG_IPIPE)
#define hard_smp_local_irq_save()		hard_local_irq_save()
#define hard_smp_local_irq_restore(__flags)	hard_local_irq_restore(__flags)
#else /* !CONFIG_SMP */
#define hard_smp_local_irq_save()		0
#define hard_smp_local_irq_restore(__flags)	do { (void)(__flags); } while(0)
#endif /* CONFIG_SMP */

#endif
