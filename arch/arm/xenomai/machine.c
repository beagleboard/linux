/**
 *   Copyright (C) 2005 Stelian Pop
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 */

#include <linux/mm.h>
#include <linux/ipipe_tickdev.h>
#include <cobalt/kernel/arith.h>
#include <asm/cacheflush.h>
#include <asm/xenomai/machine.h>

#define CALIBRATION_LOOPS 10

static void mach_arm_prefault(struct vm_area_struct *vma)
{
	unsigned long addr;
	unsigned int flags;

	if ((vma->vm_flags & VM_MAYREAD)) {
		flags = (vma->vm_flags & VM_MAYWRITE) ? FAULT_FLAG_WRITE : 0;
		for (addr = vma->vm_start;
		     addr != vma->vm_end; addr += PAGE_SIZE)
			handle_mm_fault(vma->vm_mm, vma, addr, flags);
	}
}

static unsigned long mach_arm_calibrate(void)
{
	unsigned long long start, end, sum = 0, sum_sq = 0;
	volatile unsigned const_delay = 0xffffffff;
	unsigned long result, flags, tsc_lat;
	unsigned int delay = const_delay;
	long long diff;
	int i, j;

	flags = ipipe_critical_enter(NULL);

	/*
	 * Hw interrupts off, other CPUs quiesced, no migration
	 * possible. We can now fiddle with the timer chip (per-cpu
	 * local or global, ipipe_timer_set() will handle this
	 * transparently).
	 */
	ipipe_read_tsc(start);
	barrier();
	ipipe_read_tsc(end);
	tsc_lat = end - start;
	barrier();

	for (i = 0; i < CALIBRATION_LOOPS; i++) {
		flush_cache_all();
		for (j = 0; j < CALIBRATION_LOOPS; j++) {
			ipipe_read_tsc(start);
			barrier();
			ipipe_timer_set(delay);
			barrier();
			ipipe_read_tsc(end);
			diff = end - start - tsc_lat;
			if (diff > 0) {
				sum += diff;
				sum_sq += diff * diff;
			}
		}
	}

	ipipe_critical_exit(flags);

	/* Use average + standard deviation as timer programming latency. */
	do_div(sum, CALIBRATION_LOOPS * CALIBRATION_LOOPS);
	do_div(sum_sq, CALIBRATION_LOOPS * CALIBRATION_LOOPS);
	result = sum + int_sqrt(sum_sq - sum * sum) + 1;
	/*
	 * Reset the max trace, since it contains the calibration time
	 * now.
	 */
	ipipe_trace_max_reset();

	return result;
}

static const char *const fault_labels[] = {
	[IPIPE_TRAP_ACCESS] = "Data or instruction access",
	[IPIPE_TRAP_SECTION] = "Section fault",
	[IPIPE_TRAP_DABT] = "Generic data abort",
	[IPIPE_TRAP_UNKNOWN] = "Unknown exception",
	[IPIPE_TRAP_BREAK] = "Instruction breakpoint",
	[IPIPE_TRAP_FPU] = "Floating point exception",
	[IPIPE_TRAP_VFP] = "VFP Floating point exception",
	[IPIPE_TRAP_UNDEFINSTR] = "Undefined instruction",
#ifdef IPIPE_TRAP_ALIGNMENT
	[IPIPE_TRAP_ALIGNMENT] = "Unaligned access exception",
#endif /* IPIPE_TRAP_ALIGNMENT */
	[IPIPE_NR_FAULTS] = NULL
};

struct cobalt_machine cobalt_machine = {
	.name = "arm",
	.init = NULL,
	.late_init = NULL,
	.cleanup = NULL,
	.calibrate = mach_arm_calibrate,
	.prefault = mach_arm_prefault,
	.fault_labels = fault_labels,
};
