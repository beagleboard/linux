/* -*- linux-c -*-
 * include/linux/ipipe_base.h
 *
 * Copyright (C) 2002-2014 Philippe Gerum.
 *               2007 Jan Kiszka.
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

#ifndef __LINUX_IPIPE_BASE_H
#define __LINUX_IPIPE_BASE_H

struct kvm_vcpu;
struct ipipe_vm_notifier;
struct irq_desc;

#ifdef CONFIG_IPIPE

#define IPIPE_CORE_APIREV  CONFIG_IPIPE_CORE_APIREV

#include <linux/ipipe_domain.h>
#include <linux/compiler.h>
#include <linux/linkage.h>
#include <asm/ipipe_base.h>

struct pt_regs;
struct ipipe_domain;

struct ipipe_trap_data {
	int exception;
	struct pt_regs *regs;
};

struct ipipe_vm_notifier {
	void (*handler)(struct ipipe_vm_notifier *nfy);
};

static inline int ipipe_virtual_irq_p(unsigned int irq)
{
	return irq >= IPIPE_VIRQ_BASE && irq < IPIPE_NR_IRQS;
}

void __ipipe_init_early(void);

void __ipipe_init(void);

#ifdef CONFIG_PROC_FS
void __ipipe_init_proc(void);
#ifdef CONFIG_IPIPE_TRACE
void __ipipe_init_tracer(void);
#else /* !CONFIG_IPIPE_TRACE */
static inline void __ipipe_init_tracer(void) { }
#endif /* CONFIG_IPIPE_TRACE */
#else	/* !CONFIG_PROC_FS */
static inline void __ipipe_init_proc(void) { }
#endif	/* CONFIG_PROC_FS */

void __ipipe_restore_root_nosync(unsigned long x);

#define IPIPE_IRQF_NOACK    0x1
#define IPIPE_IRQF_NOSYNC   0x2

void __ipipe_dispatch_irq(unsigned int irq, int flags);

void __ipipe_do_sync_stage(void);

void __ipipe_do_sync_pipeline(struct ipipe_domain *top);

void __ipipe_lock_irq(unsigned int irq);

void __ipipe_unlock_irq(unsigned int irq);

void __ipipe_do_critical_sync(unsigned int irq, void *cookie);

void __ipipe_ack_edge_irq(struct irq_desc *desc);

void __ipipe_nop_irq(struct irq_desc *desc);

static inline void __ipipe_idle(void)
{
	ipipe_unstall_root();
}

#ifndef __ipipe_sync_check
#define __ipipe_sync_check	1
#endif

static inline void __ipipe_sync_stage(void)
{
	if (likely(__ipipe_sync_check))
		__ipipe_do_sync_stage();
}

#ifndef __ipipe_run_irqtail
#define __ipipe_run_irqtail(irq) do { } while(0)
#endif

int __ipipe_log_printk(const char *fmt, va_list args);
void __ipipe_flush_printk(unsigned int irq, void *cookie);

#define __ipipe_get_cpu(flags)	({ (flags) = hard_preempt_disable(); ipipe_processor_id(); })
#define __ipipe_put_cpu(flags)	hard_preempt_enable(flags)

int __ipipe_notify_kevent(int event, void *data);

#define __ipipe_report_sigwake(p)					\
	do {								\
		if (ipipe_notifier_enabled_p(p))			\
			__ipipe_notify_kevent(IPIPE_KEVT_SIGWAKE, p);	\
	} while (0)

struct ipipe_cpu_migration_data {
	struct task_struct *task;
	int dest_cpu;
};

#define __ipipe_report_setaffinity(__p, __dest_cpu)			\
	do {								\
		struct ipipe_cpu_migration_data d = {			\
			.task = (__p),					\
			.dest_cpu = (__dest_cpu),			\
		};							\
		if (ipipe_notifier_enabled_p(__p))			\
			__ipipe_notify_kevent(IPIPE_KEVT_SETAFFINITY, &d); \
	} while (0)

#define __ipipe_report_exit(p)						\
	do {								\
		if (ipipe_notifier_enabled_p(p))			\
			__ipipe_notify_kevent(IPIPE_KEVT_EXIT, p);	\
	} while (0)

#define __ipipe_report_setsched(p)					\
	do {								\
		if (ipipe_notifier_enabled_p(p))			\
			__ipipe_notify_kevent(IPIPE_KEVT_SETSCHED, p); \
	} while (0)

#define __ipipe_report_schedule(prev, next)				\
do {									\
	if (ipipe_notifier_enabled_p(next) ||				\
	    ipipe_notifier_enabled_p(prev)) {				\
		__this_cpu_write(ipipe_percpu.rqlock_owner, prev);	\
		__ipipe_notify_kevent(IPIPE_KEVT_SCHEDULE, next);	\
	}								\
} while (0)

#define __ipipe_report_cleanup(mm)					\
	__ipipe_notify_kevent(IPIPE_KEVT_CLEANUP, mm)

#define __ipipe_report_clockfreq_update(freq)				\
	__ipipe_notify_kevent(IPIPE_KEVT_CLOCKFREQ, &(freq))

int __ipipe_notify_syscall(struct pt_regs *regs);

int __ipipe_notify_trap(int exception, struct pt_regs *regs);

#define __ipipe_report_trap(exception, regs)				\
	__ipipe_notify_trap(exception, regs)

void __ipipe_call_mayday(struct pt_regs *regs);

void __ipipe_notify_vm_preemption(void);

#define __ipipe_serial_debug(__fmt, __args...)	raw_printk(__fmt, ##__args)

#else /* !CONFIG_IPIPE */

struct task_struct;
struct mm_struct;

static inline void __ipipe_init_early(void) { }

static inline void __ipipe_init(void) { }

static inline void __ipipe_init_proc(void) { }

static inline void __ipipe_idle(void) { }

static inline void __ipipe_report_sigwake(struct task_struct *p) { }

static inline void __ipipe_report_setaffinity(struct task_struct *p,
					      int dest_cpu) { }

static inline void __ipipe_report_setsched(struct task_struct *p) { }

static inline void __ipipe_report_exit(struct task_struct *p) { }

static inline void __ipipe_report_cleanup(struct mm_struct *mm) { }

#define __ipipe_report_trap(exception, regs)  0

#define hard_preempt_disable()		({ preempt_disable(); 0; })
#define hard_preempt_enable(flags)	({ preempt_enable(); (void)(flags); })

#define __ipipe_get_cpu(flags)		({ (void)(flags); get_cpu(); })
#define __ipipe_put_cpu(flags)		\
	do {				\
		(void)(flags);		\
		put_cpu();		\
	} while (0)

#define __ipipe_root_tick_p(regs)	1

#define ipipe_handle_domain_irq(__domain, __hwirq, __regs)	\
	handle_domain_irq(__domain, __hwirq, __regs)

#define ipipe_handle_demuxed_irq(irq)		generic_handle_irq(irq)

#define __ipipe_enter_vm(vmf)	do { } while (0)

static inline void __ipipe_exit_vm(void) { }

static inline void __ipipe_notify_vm_preemption(void) { }

#define __ipipe_serial_debug(__fmt, __args...)	do { } while (0)

#endif	/* !CONFIG_IPIPE */

#ifdef CONFIG_IPIPE_WANT_PTE_PINNING
void __ipipe_pin_mapping_globally(unsigned long start,
				  unsigned long end);
#else
static inline void __ipipe_pin_mapping_globally(unsigned long start,
						unsigned long end)
{ }
#endif

#endif	/* !__LINUX_IPIPE_BASE_H */
