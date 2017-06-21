#ifndef _X86_IRQFLAGS_H_
#define _X86_IRQFLAGS_H_

#include <asm/processor-flags.h>

#ifndef __ASSEMBLY__

#include <linux/ipipe_base.h>
#include <linux/ipipe_trace.h>
#include <linux/compiler.h>

/*
 * Interrupt control:
 */

static inline unsigned long native_save_fl(void)
{
	unsigned long flags;

	/*
	 * "=rm" is safe here, because "pop" adjusts the stack before
	 * it evaluates its effective address -- this is part of the
	 * documented behavior of the "pop" instruction.
	 */
	asm volatile("# __raw_save_flags\n\t"
		     "pushf ; pop %0"
		     : "=rm" (flags)
		     : /* no input */
		     : "memory");

	return flags;
}

static inline void native_restore_fl(unsigned long flags)
{
	asm volatile("push %0 ; popf"
		     : /* no output */
		     :"g" (flags)
		     :"memory", "cc");
}

static inline void native_irq_disable(void)
{
	asm volatile("cli": : :"memory");
}

static inline void native_irq_enable(void)
{
	asm volatile("sti": : :"memory");
}

static inline void native_safe_halt(void)
{
	asm volatile("sti; hlt": : :"memory");
}

static inline void native_halt(void)
{
	asm volatile("hlt": : :"memory");
}

static inline int native_irqs_disabled(void)
{
	unsigned long flags = native_save_fl();

	return !(flags & X86_EFLAGS_IF);
}

#endif

#ifdef CONFIG_PARAVIRT
#include <asm/paravirt.h>
#define HARD_COND_ENABLE_INTERRUPTS
#define HARD_COND_DISABLE_INTERRUPTS
#else
#ifndef __ASSEMBLY__
#include <linux/types.h>

static inline notrace unsigned long arch_local_save_flags(void)
{
#ifdef CONFIG_IPIPE
	unsigned long flags;

	flags = (!ipipe_test_root()) << 9;
	barrier();
	return flags;
#else
	return native_save_fl();
#endif
}

static inline notrace void arch_local_irq_restore(unsigned long flags)
{
#ifdef CONFIG_IPIPE
	barrier();
	ipipe_restore_root(!(flags & X86_EFLAGS_IF));
#else
	native_restore_fl(flags);
#endif
}

static inline notrace void arch_local_irq_disable(void)
{
#ifdef CONFIG_IPIPE
	ipipe_stall_root();
	barrier();
#else
	native_irq_disable();
#endif
}

static inline notrace void arch_local_irq_enable(void)
{
#ifdef CONFIG_IPIPE
	barrier();
	ipipe_unstall_root();
#else
	native_irq_enable();
#endif
}

/*
 * Used in the idle loop; sti takes one instruction cycle
 * to complete:
 */
static inline void arch_safe_halt(void)
{
#ifdef CONFIG_IPIPE
	barrier();
	__ipipe_halt_root(0);
#else
	native_safe_halt();
#endif
}

/*
 * Used when interrupts are already enabled or to
 * shutdown the processor:
 */
static inline void halt(void)
{
	native_halt();
}

/* Merge virtual+real interrupt mask bits into a single word. */
static inline unsigned long arch_mangle_irq_bits(int virt, unsigned long real)
{
	return (real & ~(1L << 31)) | ((unsigned long)(virt != 0) << 31);
}

/* Converse operation of arch_mangle_irq_bits() */
static inline int arch_demangle_irq_bits(unsigned long *x)
{
	int virt = (*x & (1L << 31)) != 0;
	*x &= ~(1L << 31);
	return virt;
}

/*
 * For spinlocks, etc:
 */
static inline notrace unsigned long arch_local_irq_save(void)
{
	unsigned long flags = arch_local_save_flags();
	arch_local_irq_disable();
	return flags;
}
#else

#define ENABLE_INTERRUPTS(x)	sti
#define DISABLE_INTERRUPTS(x)	cli

#ifdef CONFIG_IPIPE
#define HARD_COND_ENABLE_INTERRUPTS	sti
#define HARD_COND_DISABLE_INTERRUPTS	cli
#else /* !CONFIG_IPIPE */
#define HARD_COND_ENABLE_INTERRUPTS
#define HARD_COND_DISABLE_INTERRUPTS
#endif /* !CONFIG_IPIPE */

#ifdef CONFIG_X86_64
#define SWAPGS	swapgs
/*
 * Currently paravirt can't handle swapgs nicely when we
 * don't have a stack we can rely on (such as a user space
 * stack).  So we either find a way around these or just fault
 * and emulate if a guest tries to call swapgs directly.
 *
 * Either way, this is a good way to document that we don't
 * have a reliable stack. x86_64 only.
 */
#define SWAPGS_UNSAFE_STACK	swapgs

#define PARAVIRT_ADJUST_EXCEPTION_FRAME	/*  */

#define INTERRUPT_RETURN	jmp native_iret
#define USERGS_SYSRET64				\
	swapgs;					\
	sysretq;
#define USERGS_SYSRET32				\
	swapgs;					\
	sysretl

#else
#define INTERRUPT_RETURN		iret
#define ENABLE_INTERRUPTS_SYSEXIT	sti; sysexit
#define GET_CR0_INTO_EAX		movl %cr0, %eax
#endif


#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PARAVIRT */

#ifndef __ASSEMBLY__
static inline int arch_irqs_disabled_flags(unsigned long flags)
{
	return !(flags & X86_EFLAGS_IF);
}

static inline int arch_irqs_disabled(void)
{
	unsigned long flags = arch_local_save_flags();

	return arch_irqs_disabled_flags(flags);
}

static inline unsigned long hard_local_irq_save_notrace(void)
{
	unsigned long flags;

	flags = native_save_fl();
	native_irq_disable();

	return flags;
}

static inline void hard_local_irq_restore_notrace(unsigned long flags)
{
	native_restore_fl(flags);
}

static inline void hard_local_irq_disable_notrace(void)
{
	native_irq_disable();
}

static inline void hard_local_irq_enable_notrace(void)
{
	native_irq_enable();
}

static inline int hard_irqs_disabled(void)
{
	return native_irqs_disabled();
}

#define hard_irqs_disabled_flags(flags)	arch_irqs_disabled_flags(flags)

#ifdef CONFIG_IPIPE_TRACE_IRQSOFF

static inline void hard_local_irq_disable(void)
{
	if (!native_irqs_disabled()) {
		native_irq_disable();
		ipipe_trace_begin(0x80000000);
	}
}

static inline void hard_local_irq_enable(void)
{
	if (native_irqs_disabled()) {
		ipipe_trace_end(0x80000000);
		native_irq_enable();
	}
}

static inline unsigned long hard_local_irq_save(void)
{
	unsigned long flags;

	flags = native_save_fl();
	if (flags & X86_EFLAGS_IF) {
		native_irq_disable();
		ipipe_trace_begin(0x80000001);
	}

	return flags;
}

static inline void hard_local_irq_restore(unsigned long flags)
{
	if (flags & X86_EFLAGS_IF)
		ipipe_trace_end(0x80000001);

	native_restore_fl(flags);
}

#else /* !CONFIG_IPIPE_TRACE_IRQSOFF */

static inline unsigned long hard_local_irq_save(void)
{
	return hard_local_irq_save_notrace();
}

static inline void hard_local_irq_restore(unsigned long flags)
{
	hard_local_irq_restore_notrace(flags);
}

static inline void hard_local_irq_enable(void)
{
	hard_local_irq_enable_notrace();
}

static inline void hard_local_irq_disable(void)
{
	hard_local_irq_disable_notrace();
}

#endif /* CONFIG_IPIPE_TRACE_IRQSOFF */

static inline unsigned long hard_local_save_flags(void)
{
	return native_save_fl();
}

#ifndef CONFIG_IPIPE
#define hard_cond_local_irq_enable()		do { } while(0)
#define hard_cond_local_irq_disable()		do { } while(0)
#define hard_cond_local_irq_save()		0
#define hard_cond_local_irq_restore(flags)	do { (void)(flags); } while(0)
#endif

#if defined(CONFIG_SMP) && defined(CONFIG_IPIPE)
#define hard_smp_local_irq_save()		hard_local_irq_save()
#define hard_smp_local_irq_restore(flags)	hard_local_irq_restore(flags)
#else /* !CONFIG_SMP */
#define hard_smp_local_irq_save()		0
#define hard_smp_local_irq_restore(flags)	do { (void)(flags); } while(0)
#endif /* CONFIG_SMP */

#endif /* !__ASSEMBLY__ */

#ifdef __ASSEMBLY__
#ifdef CONFIG_TRACE_IRQFLAGS
#  define TRACE_IRQS_ON		call trace_hardirqs_on_thunk;
#  define TRACE_IRQS_OFF	call trace_hardirqs_off_thunk;
#else
#  define TRACE_IRQS_ON
#  define TRACE_IRQS_OFF
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
#  ifdef CONFIG_X86_64
#    define LOCKDEP_SYS_EXIT	call lockdep_sys_exit_thunk
#    define LOCKDEP_SYS_EXIT_IRQ \
	TRACE_IRQS_ON; \
	sti; \
	call lockdep_sys_exit_thunk; \
	cli; \
	TRACE_IRQS_OFF;

#  else
#    define LOCKDEP_SYS_EXIT			\
	pushl %eax;				\
	pushl %ecx;				\
	pushl %edx;				\
	pushfl;					\
	sti;					\
	call lockdep_sys_exit;			\
	popfl;					\
	popl %edx;				\
	popl %ecx;				\
	popl %eax;

#    define LOCKDEP_SYS_EXIT_IRQ
#  endif
#else
#  define LOCKDEP_SYS_EXIT
#  define LOCKDEP_SYS_EXIT_IRQ
#endif

#endif /* __ASSEMBLY__ */
#endif
