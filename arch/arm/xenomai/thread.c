/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
 *
 * ARM port
 *   Copyright (C) 2005 Stelian Pop
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

#include <linux/sched.h>
#include <linux/ipipe.h>
#include <linux/mm.h>
#include <linux/jump_label.h>
#include <asm/mmu_context.h>
#include <cobalt/kernel/thread.h>

struct static_key __xeno_vfp_key = STATIC_KEY_INIT_TRUE;

asmlinkage void __asm_thread_switch(struct thread_info *out,
				    struct thread_info *in);

asmlinkage void __asm_thread_trampoline(void);

#if defined(CONFIG_XENO_ARCH_FPU) && defined(CONFIG_VFP)

static unsigned int vfp_checked;
static DEFINE_MUTEX(vfp_check_lock);

asmlinkage void __asm_vfp_save(union vfp_state *vfp, unsigned int fpexc);

asmlinkage void __asm_vfp_load(union vfp_state *vfp, unsigned int cpu);

#define do_vfp_fmrx(_vfp_)						\
	({								\
		u32 __v;						\
		asm volatile("mrc p10, 7, %0, " __stringify(_vfp_)	\
			     ", cr0, 0 @ fmrx %0, " #_vfp_:		\
			     "=r" (__v));				\
		__v;							\
	})

#define do_vfp_fmxr(_vfp_,_var_)				\
	asm volatile("mcr p10, 7, %0, " __stringify(_vfp_)	\
		     ", cr0, 0 @ fmxr " #_vfp_ ", %0":		\
		     /* */ : "r" (_var_))

extern union vfp_state *vfp_current_hw_state[NR_CPUS];

static inline union vfp_state *get_fpu_owner(void)
{
	union vfp_state *vfp_owner;
	unsigned int cpu;
#ifdef CONFIG_SMP
	unsigned int fpexc;
#endif

#if __LINUX_ARM_ARCH__ <= 6
	if (!static_key_true(&__xeno_vfp_key))
		return NULL;
#endif

#ifdef CONFIG_SMP
	fpexc = do_vfp_fmrx(FPEXC);
	if (!(fpexc & FPEXC_EN))
		return NULL;
#endif

	cpu = ipipe_processor_id();
	vfp_owner = vfp_current_hw_state[cpu];
	if (!vfp_owner)
		return NULL;

#ifdef CONFIG_SMP
	if (vfp_owner->hard.cpu != cpu)
		return NULL;
#endif /* SMP */

	return vfp_owner;
}

#define do_disable_vfp(fpexc)					\
	do_vfp_fmxr(FPEXC, fpexc & ~FPEXC_EN)

#define XNARCH_VFP_ANY_EXC						\
	(FPEXC_EX|FPEXC_DEX|FPEXC_FP2V|FPEXC_VV|FPEXC_TRAP_MASK)

#define do_enable_vfp()							\
	({								\
		unsigned _fpexc = do_vfp_fmrx(FPEXC) | FPEXC_EN;	\
		do_vfp_fmxr(FPEXC, _fpexc & ~XNARCH_VFP_ANY_EXC);	\
		_fpexc;							\
	})

int xnarch_fault_fpu_p(struct ipipe_trap_data *d)
{
	/* This function does the same thing to decode the faulting instruct as
	   "call_fpe" in arch/arm/entry-armv.S */
	static unsigned copro_to_exc[16] = {
		IPIPE_TRAP_UNDEFINSTR,
		/* FPE */
		IPIPE_TRAP_FPU, IPIPE_TRAP_FPU,
		IPIPE_TRAP_UNDEFINSTR,
#ifdef CONFIG_CRUNCH
		IPIPE_TRAP_FPU, IPIPE_TRAP_FPU, IPIPE_TRAP_FPU,
#else /* !CONFIG_CRUNCH */
		IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR,
#endif /* !CONFIG_CRUNCH */
		IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR,
#ifdef CONFIG_VFP
		IPIPE_TRAP_VFP, IPIPE_TRAP_VFP,
#else /* !CONFIG_VFP */
		IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR,
#endif /* !CONFIG_VFP */
		IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR,
		IPIPE_TRAP_UNDEFINSTR, IPIPE_TRAP_UNDEFINSTR,
	};
	unsigned instr, exc, cp;
	char *pc;

	if (d->exception == IPIPE_TRAP_FPU)
		return 1;

	if (d->exception == IPIPE_TRAP_VFP)
		goto trap_vfp;

	if (d->exception != IPIPE_TRAP_UNDEFINSTR)
		return 0;

	pc = (char *) xnarch_fault_pc(d);
	if (unlikely(thumb_mode(d->regs))) {
		unsigned short thumbh, thumbl;

#if defined(CONFIG_ARM_THUMB) && __LINUX_ARM_ARCH__ >= 6 && defined(CONFIG_CPU_V7)
#if __LINUX_ARM_ARCH__ < 7
		if (cpu_architecture() < CPU_ARCH_ARMv7)
#else
		if (0)
#endif /* arch < 7 */
#endif /* thumb && arch >= 6 && cpu_v7 */
			return 0;

		thumbh = *(unsigned short *) pc;
		thumbl = *((unsigned short *) pc + 1);

		if ((thumbh & 0x0000f800) < 0x0000e800)
			return 0;
		instr = (thumbh << 16) | thumbl;

#ifdef CONFIG_NEON
		if ((instr & 0xef000000) == 0xef000000
		    || (instr & 0xff100000) == 0xf9000000)
			goto trap_vfp;
#endif
	} else {
		instr = *(unsigned *) pc;

#ifdef CONFIG_NEON
		if ((instr & 0xfe000000) == 0xf2000000
		    || (instr & 0xff100000) == 0xf4000000)
			goto trap_vfp;
#endif
	}

	if ((instr & 0x0c000000) != 0x0c000000)
		return 0;

	cp = (instr & 0x00000f00) >> 8;
#ifdef CONFIG_IWMMXT
	/* We need something equivalent to _TIF_USING_IWMMXT for Xenomai kernel
	   threads */
	if (cp <= 1) {
		d->exception = IPIPE_TRAP_FPU;
		return 1;
	}
#endif

	exc = copro_to_exc[cp];
	if (exc == IPIPE_TRAP_VFP) {
	  trap_vfp:
		/* If an exception is pending, the VFP fault is not really an
		   "FPU unavailable" fault, so we return undefinstr in that
		   case, the nucleus will let linux handle the fault. */
		exc = do_vfp_fmrx(FPEXC);
		if (exc & (FPEXC_EX|FPEXC_DEX)
		    || ((exc & FPEXC_EN) && do_vfp_fmrx(FPSCR) & FPSCR_IXE))
			exc = IPIPE_TRAP_UNDEFINSTR;
		else
			exc = IPIPE_TRAP_VFP;
	}

	d->exception = exc;
	return exc != IPIPE_TRAP_UNDEFINSTR;
}

void xnarch_leave_root(struct xnthread *root)
{
	struct xnarchtcb *rootcb = xnthread_archtcb(root);
	rootcb->fpup = get_fpu_owner();
}

void xnarch_switch_fpu(struct xnthread *from, struct xnthread *to)
{
	union vfp_state *const from_fpup = from ? from->tcb.fpup : NULL;
	unsigned cpu = ipipe_processor_id();
	
	if (xnthread_test_state(to, XNROOT) == 0) {
		union vfp_state *const to_fpup = to->tcb.fpup;
		unsigned fpexc = do_enable_vfp();

		if (from_fpup == to_fpup)
			return;

		if (from_fpup)
			__asm_vfp_save(from_fpup, fpexc);

		__asm_vfp_load(to_fpup, cpu);
       } else {
		/*
		 * We are restoring the Linux current thread. The FPU
		 * can be disabled, so that a fault will occur if the
		 * newly switched thread uses the FPU, to allow the
		 * kernel handler to pick the correct FPU context, and
		 * save in the same move the last used RT context.
		 */
		vfp_current_hw_state[cpu] = from_fpup;
#ifdef CONFIG_SMP
		/*
		 * On SMP, since returning to FPU disabled mode means
		 * that we have to save fpu, avoid doing it if
		 * current FPU context belongs to the task we are
		 * switching to.
		 */
		if (from_fpup) {
			union vfp_state *const current_task_fpup =
				&to->tcb.core.tip->vfpstate;
			const unsigned fpdis = do_vfp_fmrx(FPEXC);
			const unsigned fpen = fpdis | FPEXC_EN;

			do_vfp_fmxr(FPEXC, fpen & ~XNARCH_VFP_ANY_EXC);
			if (from_fpup == current_task_fpup)
				return;
			
			__asm_vfp_save(from_fpup, fpen);
			do_vfp_fmxr(FPEXC, fpdis);
		}
#endif
	}
}

int xnarch_handle_fpu_fault(struct xnthread *from, 
			struct xnthread *to, struct ipipe_trap_data *d)
{
	if (xnthread_test_state(to, XNFPU))
		/* FPU is already enabled, probably an exception */
               return 0;

#if __LINUX_ARM_ARCH__ <= 6
	if (!static_key_true(&__xeno_vfp_key))
		/* VFP instruction emitted, on a cpu without VFP, this
		   is an error */
		return 0;
#endif

	xnlock_get(&nklock);
	xnthread_set_state(to, XNFPU);
	xnlock_put(&nklock);

	xnarch_switch_fpu(from, to);

	/* Retry faulting instruction */
	d->regs->ARM_pc = xnarch_fault_pc(d);
	return 1;
}

void xnarch_init_shadow_tcb(struct xnthread *thread)
{
	struct xnarchtcb *tcb = xnthread_archtcb(thread);

	tcb->fpup = &task_thread_info(tcb->core.host_task)->vfpstate;

	if (vfp_checked == 0) {
		mutex_lock(&vfp_check_lock);
		if (vfp_checked == 0) {
			if ((elf_hwcap & HWCAP_VFP) == 0)
				static_key_slow_dec(&__xeno_vfp_key);
			vfp_checked = 1;
		}
		mutex_unlock(&vfp_check_lock);
	}

	/* XNFPU is set upon first FPU fault */
	xnthread_clear_state(thread, XNFPU);
}

void xnarch_init_root_tcb(struct xnthread *thread)
{
	struct xnarchtcb *tcb = &thread->tcb;
	tcb->fpup = NULL;
}

#endif /* CONFIG_XENO_ARCH_FPU && CONFIG_VFP*/

void xnarch_switch_to(struct xnthread *out, struct xnthread *in)
{
	struct xnarchtcb *out_tcb = &out->tcb, *in_tcb = &in->tcb;
	struct mm_struct *prev_mm, *next_mm;
	struct task_struct *next;

	next = in_tcb->core.host_task;
	prev_mm = out_tcb->core.active_mm;

	next_mm = in_tcb->core.mm;
	if (next_mm == NULL) {
		in_tcb->core.active_mm = prev_mm;
		enter_lazy_tlb(prev_mm, next);
	} else {
		ipipe_switch_mm_head(prev_mm, next_mm, next);
		/*
		 * We might be switching back to the root thread,
		 * which we preempted earlier, shortly after "current"
		 * dropped its mm context in the do_exit() path
		 * (next->mm == NULL). In that particular case, the
		 * kernel expects a lazy TLB state for leaving the mm.
		 */
		if (next->mm == NULL)
			enter_lazy_tlb(prev_mm, next);
	}

	__asm_thread_switch(out_tcb->core.tip, in_tcb->core.tip);
}
