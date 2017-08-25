/* -*- linux-c -*-
 * arch/arm/include/asm/ipipe.h
 *
 * Copyright (C) 2002-2005 Philippe Gerum.
 * Copyright (C) 2005 Stelian Pop.
 * Copyright (C) 2006-2008 Gilles Chanteperdrix.
 * Copyright (C) 2010 Philippe Gerum (SMP port).
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

#ifndef __ARM_IPIPE_H
#define __ARM_IPIPE_H

#include <linux/irqdomain.h>

#ifdef CONFIG_IPIPE

#include <linux/jump_label.h>
#include <linux/ipipe_trace.h>
#include <linux/ipipe_debug.h>

#define IPIPE_CORE_RELEASE	2

struct ipipe_domain;

#define IPIPE_TSC_TYPE_NONE			0
#define IPIPE_TSC_TYPE_FREERUNNING		1
#define IPIPE_TSC_TYPE_DECREMENTER		2
#define IPIPE_TSC_TYPE_FREERUNNING_COUNTDOWN	3
#define IPIPE_TSC_TYPE_FREERUNNING_TWICE	4
#define IPIPE_TSC_TYPE_FREERUNNING_ARCH		5

/* tscinfo, exported to user-space */
struct __ipipe_tscinfo {
	unsigned type;
	unsigned freq;
	unsigned long counter_vaddr;
	union {
		struct {
			unsigned long counter_paddr;
			unsigned long long mask;
		};
		struct {
			unsigned *counter; /* Hw counter physical address */
			unsigned long long mask; /* Significant bits in the hw counter. */
			unsigned long long *tsc; /* 64 bits tsc value. */
		} fr;
		struct {
			unsigned *counter; /* Hw counter physical address */
			unsigned long long mask; /* Significant bits in the hw counter. */
			unsigned *last_cnt; /* Counter value when updating
						tsc value. */
			unsigned long long *tsc; /* 64 bits tsc value. */
		} dec;
	} u;
};

struct ipipe_arch_sysinfo {
	struct __ipipe_tscinfo tsc;
};


/* arch specific stuff */

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info);

static inline void __ipipe_mach_update_tsc(void) {}

static inline notrace unsigned long long __ipipe_mach_get_tsc(void)
{
	return arch_counter_get_cntvct();
}

#define __ipipe_tsc_get() __ipipe_mach_get_tsc()
void __ipipe_tsc_register(struct __ipipe_tscinfo *info);
static inline void __ipipe_tsc_update(void) {}
#ifndef __ipipe_hrclock_freq
extern unsigned long __ipipe_hrtimer_freq;
#define __ipipe_hrclock_freq __ipipe_hrtimer_freq
#endif /* !__ipipe_mach_hrclock_freq */

#ifdef CONFIG_IPIPE_DEBUG_INTERNAL
extern void (*__ipipe_mach_hrtimer_debug)(unsigned irq);
#endif /* CONFIG_IPIPE_DEBUG_INTERNAL */

#define ipipe_mm_switch_protect(__flags)		\
	do {						\
		(__flags) = hard_cond_local_irq_save();	\
	} while (0)

#define ipipe_mm_switch_unprotect(__flags)	\
	hard_cond_local_irq_restore(__flags)

#define ipipe_read_tsc(t)	do { t = __ipipe_tsc_get(); } while(0)
#define __ipipe_read_timebase()	__ipipe_tsc_get()

#define ipipe_tsc2ns(t) \
({ \
	unsigned long long delta = (t)*1000; \
	do_div(delta, __ipipe_hrclock_freq / 1000000 + 1); \
	(unsigned long)delta; \
})
#define ipipe_tsc2us(t) \
({ \
	unsigned long long delta = (t); \
	do_div(delta, __ipipe_hrclock_freq / 1000000 + 1); \
	(unsigned long)delta; \
})

static inline const char *ipipe_clock_name(void)
{
	return "ipipe_tsc";
}

/* Private interface -- Internal use only */

#define __ipipe_enable_irq(irq)		enable_irq(irq)
#define __ipipe_disable_irq(irq)	disable_irq(irq)

/* PIC muting */
struct ipipe_mach_pic_muter {
	void (*enable_irqdesc)(struct ipipe_domain *ipd, unsigned irq);
	void (*disable_irqdesc)(struct ipipe_domain *ipd, unsigned irq);
	void (*mute)(void);
	void (*unmute)(void);
};

extern struct ipipe_mach_pic_muter ipipe_pic_muter;

void ipipe_pic_muter_register(struct ipipe_mach_pic_muter *muter);

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned irq);

void __ipipe_disable_irqdesc(struct ipipe_domain *ipd, unsigned irq);

static inline void ipipe_mute_pic(void)
{
	if (ipipe_pic_muter.mute)
		ipipe_pic_muter.mute();
}

static inline void ipipe_unmute_pic(void)
{
	if (ipipe_pic_muter.unmute)
		ipipe_pic_muter.unmute();
}

#define ipipe_notify_root_preemption() do { } while(0)

#ifdef CONFIG_SMP
void __ipipe_early_core_setup(void);
void __ipipe_hook_critical_ipi(struct ipipe_domain *ipd);
void __ipipe_root_localtimer(unsigned int irq, void *cookie);
void __ipipe_grab_ipi(unsigned svc, struct pt_regs *regs);
void __ipipe_ipis_alloc(void);
void __ipipe_ipis_request(void);

static inline void ipipe_handle_multi_ipi(int irq, struct pt_regs *regs)
{
	__ipipe_grab_ipi(irq, regs);
}

#ifdef CONFIG_SMP_ON_UP
extern struct static_key __ipipe_smp_key;
#define ipipe_smp_p (static_key_true(&__ipipe_smp_key))
#endif /* SMP_ON_UP */
#else /* !CONFIG_SMP */
#define __ipipe_early_core_setup()	do { } while(0)
#define __ipipe_hook_critical_ipi(ipd)	do { } while(0)
#endif /* !CONFIG_SMP */
#ifndef __ipipe_mach_init_platform
#define __ipipe_mach_init_platform()	do { } while(0)
#endif

void __ipipe_enable_pipeline(void);

void __ipipe_do_critical_sync(unsigned irq, void *cookie);

void __ipipe_grab_irq(int irq, struct pt_regs *regs);

void __ipipe_exit_irq(struct pt_regs *regs);

static inline
int ipipe_handle_domain_irq(struct irq_domain *domain,
			    unsigned int hwirq, struct pt_regs *regs)
{
	unsigned int irq;

	irq = irq_find_mapping(domain, hwirq);
	__ipipe_grab_irq(irq, regs);

	return 0;
}

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
	int __r;

	/* zero input is not valid */
	IPIPE_WARN(ul == 0);

	__asm__ ("rbit\t%0, %1\n"
	         "clz\t%0, %0\n"
	        : "=r" (__r) : "r"(ul) : "cc");

	return __r;
}

#define __ipipe_syscall_watched_p(p, sc)				\
	(ipipe_notifier_enabled_p(p) || (unsigned long)sc >= __NR_syscalls)

#define __ipipe_root_tick_p(regs) (!arch_irqs_disabled_flags(regs->pstate))

#else /* !CONFIG_IPIPE */

#define __ipipe_tsc_update()	do { } while(0)

#define hard_smp_processor_id()		smp_processor_id()

#define ipipe_mm_switch_protect(flags) \
	do {					\
		(void) (flags);			\
	} while(0)

#define ipipe_mm_switch_unprotect(flags)	\
	do {					\
		(void) (flags);			\
	} while(0)

#ifdef CONFIG_SMP
static inline void ipipe_handle_multi_ipi(int irq, struct pt_regs *regs)
{
	handle_IPI(irq, regs);
}
#endif /* CONFIG_SMP */

static inline
int ipipe_handle_domain_irq(struct irq_domain *domain,
			    unsigned int hwirq, struct pt_regs *regs)
{
	return handle_domain_irq(domain, hwirq, regs);
}

#endif /* CONFIG_IPIPE */

#endif	/* !__ARM_IPIPE_H */
