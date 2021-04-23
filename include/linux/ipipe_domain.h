/*   -*- linux-c -*-
 *   include/linux/ipipe_domain.h
 *
 *   Copyright (C) 2007-2012 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __LINUX_IPIPE_DOMAIN_H
#define __LINUX_IPIPE_DOMAIN_H

#ifdef CONFIG_IPIPE

#include <linux/mutex.h>
#include <linux/percpu.h>
#include <asm/ptrace.h>
#include <asm/hw_irq.h>
#include <asm/ipipe_base.h>

struct task_struct;
struct mm_struct;
struct irq_desc;
struct ipipe_vm_notifier;

#define __bpl_up(x)		(((x)+(BITS_PER_LONG-1)) & ~(BITS_PER_LONG-1))
/* Number of virtual IRQs (must be a multiple of BITS_PER_LONG) */
#define IPIPE_NR_VIRQS		BITS_PER_LONG
/* First virtual IRQ # (must be aligned on BITS_PER_LONG) */
#define IPIPE_VIRQ_BASE		__bpl_up(IPIPE_NR_XIRQS)
/* Total number of IRQ slots */
#define IPIPE_NR_IRQS		(IPIPE_VIRQ_BASE+IPIPE_NR_VIRQS)

#define IPIPE_IRQ_MAPSZ		(IPIPE_NR_IRQS / BITS_PER_LONG)
#define IPIPE_IRQ_1MAPSZ	BITS_PER_LONG
#if IPIPE_IRQ_MAPSZ > BITS_PER_LONG * BITS_PER_LONG
/*
 * We need a 4-level mapping, up to 16M IRQs (64bit long, MAXSMP
 * defines 512K IRQs).
 */
#define __IPIPE_IRQMAP_LEVELS	4
#define IPIPE_IRQ_2MAPSZ	(BITS_PER_LONG * BITS_PER_LONG)
#elif IPIPE_IRQ_MAPSZ > BITS_PER_LONG
/*
 * 3-level mapping. Up to 256K IRQs (64 bit long).
 */
#define __IPIPE_IRQMAP_LEVELS	3
#else
/*
 * 2-level mapping is enough. Up to 4K IRQs (64 bit long).
 */
#define __IPIPE_IRQMAP_LEVELS	2
#endif

/* Per-cpu pipeline status */
#define IPIPE_STALL_FLAG	0 /* interrupts (virtually) disabled. */
#define IPIPE_STALL_MASK	(1L << IPIPE_STALL_FLAG)

/* Interrupt control bits */
#define IPIPE_HANDLE_FLAG	0
#define IPIPE_STICKY_FLAG	1
#define IPIPE_LOCK_FLAG		2
#define IPIPE_HANDLE_MASK	(1 << IPIPE_HANDLE_FLAG)
#define IPIPE_STICKY_MASK	(1 << IPIPE_STICKY_FLAG)
#define IPIPE_LOCK_MASK		(1 << IPIPE_LOCK_FLAG)

#define __IPIPE_SYSCALL_P  0
#define __IPIPE_TRAP_P     1
#define __IPIPE_KEVENT_P   2
#define __IPIPE_SYSCALL_E (1 << __IPIPE_SYSCALL_P)
#define __IPIPE_TRAP_E	  (1 << __IPIPE_TRAP_P)
#define __IPIPE_KEVENT_E  (1 << __IPIPE_KEVENT_P)
#define __IPIPE_ALL_E	   0x7
#define __IPIPE_SYSCALL_R (8 << __IPIPE_SYSCALL_P)
#define __IPIPE_TRAP_R	  (8 << __IPIPE_TRAP_P)
#define __IPIPE_KEVENT_R  (8 << __IPIPE_KEVENT_P)
#define __IPIPE_SHIFT_R	   3
#define __IPIPE_ALL_R	  (__IPIPE_ALL_E << __IPIPE_SHIFT_R)

#define IPIPE_KEVT_SCHEDULE	0
#define IPIPE_KEVT_SIGWAKE	1
#define IPIPE_KEVT_SETSCHED	2
#define IPIPE_KEVT_SETAFFINITY	3
#define IPIPE_KEVT_EXIT		4
#define IPIPE_KEVT_CLEANUP	5
#define IPIPE_KEVT_HOSTRT	6
#define IPIPE_KEVT_CLOCKFREQ	7
#define IPIPE_KEVT_USERINTRET	8
#define IPIPE_KEVT_PTRESUME	9

typedef void (*ipipe_irq_ackfn_t)(struct irq_desc *desc);

typedef void (*ipipe_irq_handler_t)(unsigned int irq,
				    void *cookie);

struct ipipe_domain {
	int context_offset;
	struct ipipe_irqdesc {
		unsigned long control;
		ipipe_irq_ackfn_t ackfn;
		ipipe_irq_handler_t handler;
		void *cookie;
	} ____cacheline_aligned irqs[IPIPE_NR_IRQS];
	const char *name;
	struct mutex mutex;
};

static inline void *
__ipipe_irq_cookie(struct ipipe_domain *ipd, unsigned int irq)
{
	return ipd->irqs[irq].cookie;
}

static inline ipipe_irq_handler_t
__ipipe_irq_handler(struct ipipe_domain *ipd, unsigned int irq)
{
	return ipd->irqs[irq].handler;
}

extern struct ipipe_domain ipipe_root;

#define ipipe_root_domain (&ipipe_root)

extern struct ipipe_domain *ipipe_head_domain;

struct ipipe_percpu_domain_data {
	unsigned long status;	/* <= Must be first in struct. */
	unsigned long irqpend_0map;
#if __IPIPE_IRQMAP_LEVELS >= 3
	unsigned long irqpend_1map[IPIPE_IRQ_1MAPSZ];
#if __IPIPE_IRQMAP_LEVELS >= 4
	unsigned long irqpend_2map[IPIPE_IRQ_2MAPSZ];
#endif
#endif
	unsigned long irqpend_map[IPIPE_IRQ_MAPSZ];
	unsigned long irqheld_map[IPIPE_IRQ_MAPSZ];
	unsigned long irqall[IPIPE_NR_IRQS];
	struct ipipe_domain *domain;
	int coflags;
};

struct ipipe_percpu_data {
	struct ipipe_percpu_domain_data root;
	struct ipipe_percpu_domain_data head;
	struct ipipe_percpu_domain_data *curr;
	struct pt_regs tick_regs;
	int hrtimer_irq;
	struct task_struct *task_hijacked;
	struct task_struct *rqlock_owner;
	struct ipipe_vm_notifier *vm_notifier;
	unsigned long nmi_state;
	struct mm_struct *active_mm;
#ifdef CONFIG_IPIPE_DEBUG_CONTEXT
	int context_check;
	int context_check_saved;
#endif
};

/*
 * CAREFUL: all accessors based on __ipipe_raw_cpu_ptr() you may find
 * in this file should be used only while hw interrupts are off, to
 * prevent from CPU migration regardless of the running domain.
 */
DECLARE_PER_CPU(struct ipipe_percpu_data, ipipe_percpu);

static inline struct ipipe_percpu_domain_data *
__context_of(struct ipipe_percpu_data *p, struct ipipe_domain *ipd)
{
	return (void *)p + ipd->context_offset;
}

/**
 * ipipe_percpu_context - return the address of the pipeline context
 * data for a domain on a given CPU.
 *
 * NOTE: this is the slowest accessor, use it carefully. Prefer
 * ipipe_this_cpu_context() for requests targeted at the current
 * CPU. Additionally, if the target domain is known at build time,
 * consider ipipe_this_cpu_{root, head}_context().
 */
static inline struct ipipe_percpu_domain_data *
ipipe_percpu_context(struct ipipe_domain *ipd, int cpu)
{
	return __context_of(&per_cpu(ipipe_percpu, cpu), ipd);
}

/**
 * ipipe_this_cpu_context - return the address of the pipeline context
 * data for a domain on the current CPU. hw IRQs must be off.
 *
 * NOTE: this accessor is a bit faster, but since we don't know which
 * one of "root" or "head" ipd refers to, we still need to compute the
 * context address from its offset.
 */
static inline struct ipipe_percpu_domain_data *
ipipe_this_cpu_context(struct ipipe_domain *ipd)
{
	return __context_of(__ipipe_raw_cpu_ptr(&ipipe_percpu), ipd);
}

/**
 * ipipe_this_cpu_root_context - return the address of the pipeline
 * context data for the root domain on the current CPU. hw IRQs must
 * be off.
 *
 * NOTE: this accessor is recommended when the domain we refer to is
 * known at build time to be the root one.
 */
static inline struct ipipe_percpu_domain_data *
ipipe_this_cpu_root_context(void)
{
	return __ipipe_raw_cpu_ptr(&ipipe_percpu.root);
}

/**
 * ipipe_this_cpu_head_context - return the address of the pipeline
 * context data for the registered head domain on the current CPU. hw
 * IRQs must be off.
 *
 * NOTE: this accessor is recommended when the domain we refer to is
 * known at build time to be the registered head domain. This address
 * is always different from the context data of the root domain in
 * absence of registered head domain. To get the address of the
 * context data for the domain leading the pipeline at the time of the
 * call (which may be root in absence of registered head domain), use
 * ipipe_this_cpu_leading_context() instead.
 */
static inline struct ipipe_percpu_domain_data *
ipipe_this_cpu_head_context(void)
{
	return __ipipe_raw_cpu_ptr(&ipipe_percpu.head);
}

/**
 * ipipe_this_cpu_leading_context - return the address of the pipeline
 * context data for the domain leading the pipeline on the current
 * CPU. hw IRQs must be off.
 *
 * NOTE: this accessor is required when either root or a registered
 * head domain may be the final target of this call, depending on
 * whether the high priority domain was installed via
 * ipipe_register_head().
 */
static inline struct ipipe_percpu_domain_data *
ipipe_this_cpu_leading_context(void)
{
	return ipipe_this_cpu_context(ipipe_head_domain);
}

/**
 * __ipipe_get_current_context() - return the address of the pipeline
 * context data of the domain running on the current CPU. hw IRQs must
 * be off.
 */
static inline struct ipipe_percpu_domain_data *__ipipe_get_current_context(void)
{
	return __ipipe_raw_cpu_read(ipipe_percpu.curr);
}

#define __ipipe_current_context __ipipe_get_current_context()

/**
 * __ipipe_set_current_context() - switch the current CPU to the
 * specified domain context.  hw IRQs must be off.
 *
 * NOTE: this is the only way to change the current domain for the
 * current CPU. Don't bypass.
 */
static inline
void __ipipe_set_current_context(struct ipipe_percpu_domain_data *pd)
{
	struct ipipe_percpu_data *p;
	p = __ipipe_raw_cpu_ptr(&ipipe_percpu);
	p->curr = pd;
}

/**
 * __ipipe_set_current_domain() - switch the current CPU to the
 * specified domain. This is equivalent to calling
 * __ipipe_set_current_context() with the context data of that
 * domain. hw IRQs must be off.
 */
static inline void __ipipe_set_current_domain(struct ipipe_domain *ipd)
{
	struct ipipe_percpu_data *p;
	p = __ipipe_raw_cpu_ptr(&ipipe_percpu);
	p->curr = __context_of(p, ipd);
}

static inline struct ipipe_percpu_domain_data *ipipe_current_context(void)
{
	struct ipipe_percpu_domain_data *pd;
	unsigned long flags;

	flags = hard_smp_local_irq_save();
	pd = __ipipe_get_current_context();
	hard_smp_local_irq_restore(flags);

	return pd;
}

static inline struct ipipe_domain *__ipipe_get_current_domain(void)
{
	return __ipipe_get_current_context()->domain;
}

#define __ipipe_current_domain	__ipipe_get_current_domain()

/**
 * __ipipe_get_current_domain() - return the address of the pipeline
 * domain running on the current CPU. hw IRQs must be off.
 */
static inline struct ipipe_domain *ipipe_get_current_domain(void)
{
	struct ipipe_domain *ipd;
	unsigned long flags;

	flags = hard_smp_local_irq_save();
	ipd = __ipipe_get_current_domain();
	hard_smp_local_irq_restore(flags);

	return ipd;
}

#define ipipe_current_domain	ipipe_get_current_domain()

#define __ipipe_root_p	(__ipipe_current_domain == ipipe_root_domain)
#define ipipe_root_p	(ipipe_current_domain == ipipe_root_domain)

#ifdef CONFIG_SMP
#define __ipipe_root_status	(ipipe_this_cpu_root_context()->status)
#else
extern unsigned long __ipipe_root_status;
#endif

#define __ipipe_head_status	(ipipe_this_cpu_head_context()->status)

/**
 * __ipipe_ipending_p() - Whether we have interrupts pending
 * (i.e. logged) for the given domain context on the current CPU. hw
 * IRQs must be off.
 */
static inline int __ipipe_ipending_p(struct ipipe_percpu_domain_data *pd)
{
	return pd->irqpend_0map != 0;
}

static inline unsigned long
__ipipe_cpudata_irq_hits(struct ipipe_domain *ipd, int cpu, unsigned int irq)
{
	return ipipe_percpu_context(ipd, cpu)->irqall[irq];
}

#endif /* CONFIG_IPIPE */

#endif	/* !__LINUX_IPIPE_DOMAIN_H */
