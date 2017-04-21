/* -*- linux-c -*-
 * include/linux/ipipe_compat.h
 *
 * Copyright (C) 2012 Philippe Gerum.
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

#ifndef __LINUX_IPIPE_COMPAT_H
#define __LINUX_IPIPE_COMPAT_H

#ifndef __LINUX_IPIPE_H
#error "Do not include this file directly, use linux/ipipe.h instead"
#endif

#ifdef CONFIG_IPIPE_LEGACY

#define IPIPE_HEAD_PRIORITY	(-1)
#define IPIPE_ROOT_PRIO		100
#define IPIPE_ROOT_ID		0
#define IPIPE_ROOT_NPTDKEYS	4

/* Legacy pipeline status bit */
#define IPIPE_NOSTACK_FLAG	1 /* running on foreign stack. */
#define IPIPE_NOSTACK_MASK	(1L << IPIPE_NOSTACK_FLAG)

/* Legacy interrupt control bits */
#define IPIPE_DUMMY_FLAG	31
#define IPIPE_WIRED_FLAG	IPIPE_HANDLE_FLAG
#define IPIPE_WIRED_MASK	(1 << IPIPE_WIRED_FLAG)
#define IPIPE_PASS_FLAG		IPIPE_DUMMY_FLAG
#define IPIPE_PASS_MASK		(1 << IPIPE_PASS_FLAG)
#define IPIPE_DYNAMIC_FLAG	IPIPE_HANDLE_FLAG
#define IPIPE_DYNAMIC_MASK	(1 << IPIPE_DYNAMIC_FLAG)
#define IPIPE_SYSTEM_FLAG	IPIPE_DUMMY_FLAG
#define IPIPE_SYSTEM_MASK	(1 << IPIPE_SYSTEM_FLAG)
#define IPIPE_EXCLUSIVE_FLAG	IPIPE_DUMMY_FLAG
#define IPIPE_EXCLUSIVE_MASK	(1 << IPIPE_EXCLUSIVE_FLAG)

#define IPIPE_NR_CPUS		NR_CPUS

#define IPIPE_EVENT_SELF        0x80000000
#define IPIPE_EVENT_RETURN	IPIPE_TRAP_MAYDAY

#define TASK_ATOMICSWITCH	TASK_HARDENING

struct ipipe_domain_attr {
	unsigned int domid;
	const char *name;
	int priority;
	void (*entry) (void);
	void *pdd;
};

void ipipe_init_attr(struct ipipe_domain_attr *attr);

int ipipe_register_domain(struct ipipe_domain *ipd,
			  struct ipipe_domain_attr *attr);

int ipipe_unregister_domain(struct ipipe_domain *ipd);

int ipipe_alloc_ptdkey(void);

int ipipe_free_ptdkey(int key);

int ipipe_set_ptd(int key, void *value);

void *ipipe_get_ptd(int key);

int ipipe_virtualize_irq(struct ipipe_domain *ipd,
			 unsigned int irq,
			 ipipe_irq_handler_t handler,
			 void *cookie,
			 ipipe_irq_ackfn_t ackfn,
			 unsigned int modemask);

ipipe_event_handler_t ipipe_catch_event(struct ipipe_domain *ipd,
					unsigned int event,
					ipipe_event_handler_t handler);

int ipipe_setscheduler_root(struct task_struct *p,
			    int policy,
			    int prio);

static inline void ipipe_check_context(struct ipipe_domain *border_ipd)
{
	ipipe_root_only();
}

static inline void ipipe_set_printk_sync(struct ipipe_domain *ipd)
{
	ipipe_prepare_panic();
}

static inline void __ipipe_propagate_irq(unsigned int irq)
{
	ipipe_post_irq_root(irq);
}

static inline void __ipipe_schedule_irq_head(unsigned int irq)
{
	ipipe_post_irq_head(irq);
}

static inline void __ipipe_schedule_irq_root(unsigned int irq)
{
	ipipe_post_irq_root(irq);
}

static inline int ipipe_trigger_irq(unsigned int irq)
{
	ipipe_raise_irq(irq);
	return 1;
}

static inline void ipipe_stall_pipeline_from(struct ipipe_domain *ipd)
{
	if (ipd != ipipe_root_domain)
		ipipe_stall_head();
	else
		ipipe_stall_root();
}

static inline
unsigned long ipipe_test_and_stall_pipeline_from(struct ipipe_domain *ipd)
{
	if (ipd != ipipe_root_domain)
		return ipipe_test_and_stall_head();

	return ipipe_test_and_stall_root();
}

static inline
void ipipe_unstall_pipeline_from(struct ipipe_domain *ipd)
{
	if (ipd != ipipe_root_domain)
		ipipe_unstall_head();
	else
		ipipe_unstall_root();
}

static inline
void ipipe_restore_pipeline_from(struct ipipe_domain *ipd,
				 unsigned long x)
{
	if (ipd != ipipe_root_domain)
		ipipe_restore_head(x);
	else
		ipipe_restore_root(x);
}

static inline
unsigned long ipipe_test_pipeline_from(struct ipipe_domain *ipd)
{
	return test_bit(IPIPE_STALL_FLAG, &ipipe_this_cpu_context(ipd)->status);
}

static inline void ipipe_stall_pipeline_head(void)
{
	ipipe_stall_head();
}

static inline unsigned long ipipe_test_and_stall_pipeline_head(void)
{
	return ipipe_test_and_stall_head();
}

static inline void ipipe_unstall_pipeline_head(void)
{
	ipipe_unstall_head();
}

static inline void ipipe_restore_pipeline_head(unsigned long x)
{
	ipipe_restore_head(x);
}

static inline int ipipe_disable_ondemand_mappings(struct task_struct *p)
{
	return __ipipe_disable_ondemand_mappings(p);
}

static inline int ipipe_reenter_root(struct task_struct *prev,
				     int policy,
				     int prio)
{
	__ipipe_reenter_root();
	return 0;
}

static inline void ipipe_root_preempt_notify(void)
{
	ipipe_notify_root_preemption();
}

#define ipipe_return_notify(p)	ipipe_raise_mayday(p)

/*
 * Keep the following as a macro, so that client code could check for
 * the support of the invariant pipeline head optimization.
 */
#define __ipipe_pipeline_head() ipipe_head_domain

static inline int irqs_disabled_hw(void)
{
	return hard_irqs_disabled();
}

static inline void local_irq_disable_hw(void)
{
	hard_local_irq_disable();
}

static inline void local_irq_enable_hw(void)
{
	hard_local_irq_enable();
}

#define local_irq_save_hw(flags)			\
	do {						\
		(flags) = hard_local_irq_save();	\
	} while (0)

static inline void local_irq_restore_hw(unsigned long flags)
{
	hard_local_irq_restore(flags);
}

#define local_save_flags_hw(flags)			\
	do {						\
		(flags) = hard_local_save_flags();	\
	} while (0)

#define local_irq_save_hw_smp(flags)			\
	do {						\
		(flags) = hard_smp_local_irq_save();	\
	} while (0)
#define local_irq_restore_hw_smp(flags)   hard_smp_local_irq_restore(flags)

#define local_irq_save_hw_cond(flags)			\
	do {						\
		(flags) = hard_cond_local_irq_save();	\
	} while (0)
#define local_irq_restore_hw_cond(flags)  hard_cond_local_irq_restore(flags)

static inline void ipipe_set_foreign_stack(struct ipipe_domain *ipd)
{
	/* Must be called hw interrupts off. */
	__set_bit(IPIPE_NOSTACK_FLAG, &ipipe_this_cpu_context(ipd)->status);
}

static inline void ipipe_clear_foreign_stack(struct ipipe_domain *ipd)
{
	/* Must be called hw interrupts off. */
	__clear_bit(IPIPE_NOSTACK_FLAG, &ipipe_this_cpu_context(ipd)->status);
}

static inline int ipipe_test_foreign_stack(void)
{
	/* Must be called hw interrupts off. */
	return test_bit(IPIPE_NOSTACK_FLAG, &__ipipe_current_context->status);
}

#ifndef ipipe_safe_current
#define ipipe_safe_current()						\
	({								\
		struct task_struct *__p__;				\
		unsigned long __flags__;				\
		__flags__ = hard_smp_local_irq_save();			\
		__p__ = ipipe_test_foreign_stack() ? &init_task : current; \
		hard_smp_local_irq_restore(__flags__);			\
		__p__;							\
	})
#endif

void __ipipe_legacy_init_stage(struct ipipe_domain *ipd);

/*
 * These values have no real meaning from a versioning POV, however
 * they are guaranteed to look more recent than any legacy patch
 * release ever published in the past.
 */
#define IPIPE_MAJOR_NUMBER  3
#define IPIPE_MINOR_NUMBER  0
#define IPIPE_PATCH_NUMBER  0

#define __IPIPE_FEATURE_REQUEST_TICKDEV		1
#define __IPIPE_FEATURE_FASTPEND_IRQ		1
#define __IPIPE_FEATURE_TRACE_EVENT		1
#define __IPIPE_FEATURE_ENABLE_NOTIFIER		1
#define __IPIPE_FEATURE_PREPARE_PANIC		1
#define __IPIPE_FEATURE_SYSINFO_V2		1
#define __IPIPE_FEATURE_PIC_MUTE		1
#ifdef CONFIG_IPIPE_HAVE_VM_NOTIFIER
#define __IPIPE_FEATURE_ROOTPREEMPT_NOTIFIER	1
#endif

#else  /* !CONFIG_IPIPE_LEGACY */

static inline void __ipipe_legacy_init_stage(struct ipipe_domain *ipd)
{
}

#endif /* !CONFIG_IPIPE_LEGACY */

#endif	/* !__LINUX_IPIPE_COMPAT_H */
