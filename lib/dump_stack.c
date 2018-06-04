/*
 * Provide a default dump_stack() function for architectures
 * which don't implement their own.
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/atomic.h>
#include <linux/ipipe.h>

static void __dump_stack(void)
{
	dump_stack_print_info(KERN_DEFAULT);
	show_stack(NULL, NULL);
}

/**
 * dump_stack - dump the current task information and its stack trace
 *
 * Architectures can override this implementation by implementing its own.
 */
#ifdef CONFIG_SMP
static atomic_t dump_lock = ATOMIC_INIT(-1);

static unsigned long disable_local_irqs(void)
{
	unsigned long flags = 0; /* only to trick the UMR detection */

	/*
	 * We neither need nor want to disable root stage IRQs over
	 * the head stage, where CPU migration can't
	 * happen. Conversely, we neither need nor want to disable
	 * hard IRQs from the head stage, so that latency won't
	 * skyrocket as a result of dumping the stack backtrace.
	 */
	if (ipipe_root_p)
		local_irq_save(flags);

	return flags;
}

static void restore_local_irqs(unsigned long flags)
{
	if (ipipe_root_p)
		local_irq_restore(flags);
}

asmlinkage __visible void dump_stack(void)
{
	unsigned long flags;
	int was_locked;
	int old;
	int cpu;

	/*
	 * Permit this cpu to perform nested stack dumps while serialising
	 * against other CPUs
	 */
retry:
	flags = disable_local_irqs();
	cpu = smp_processor_id();
	old = atomic_cmpxchg(&dump_lock, -1, cpu);
	if (old == -1) {
		was_locked = 0;
	} else if (old == cpu) {
		was_locked = 1;
	} else {
		restore_local_irqs(flags);
		cpu_relax();
		goto retry;
	}

	__dump_stack();

	if (!was_locked)
		atomic_set(&dump_lock, -1);

	restore_local_irqs(flags);
}
#else
asmlinkage __visible void dump_stack(void)
{
	__dump_stack();
}
#endif
EXPORT_SYMBOL(dump_stack);
