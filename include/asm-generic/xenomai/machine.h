/**
 *   Copyright &copy; 2012 Philippe Gerum.
 *
 *   Xenomai is free software; you can redistribute it and/or modify
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
#ifndef _COBALT_ASM_GENERIC_MACHINE_H
#define _COBALT_ASM_GENERIC_MACHINE_H

#include <linux/ipipe.h>
#include <linux/percpu.h>
#include <asm/byteorder.h>
#include <asm/xenomai/wrappers.h>

struct vm_area_struct;

struct cobalt_machine {
	const char *name;
	int (*init)(void);
	int (*late_init)(void);
	void (*cleanup)(void);
	void (*prefault)(struct vm_area_struct *vma);
	unsigned long (*calibrate)(void);
	const char *const *fault_labels;
};

extern struct cobalt_machine cobalt_machine;

struct cobalt_machine_cpudata {
	unsigned long apc_pending;
	unsigned long apc_shots[BITS_PER_LONG];
	unsigned int faults[IPIPE_NR_FAULTS];
};

DECLARE_PER_CPU(struct cobalt_machine_cpudata, cobalt_machine_cpudata);

struct cobalt_pipeline {
	struct ipipe_domain domain;
	unsigned long timer_freq;
	unsigned long clock_freq;
	unsigned int apc_virq;
	unsigned long apc_map;
	unsigned int escalate_virq;
	struct {
		void (*handler)(void *cookie);
		void *cookie;
		const char *name;
	} apc_table[BITS_PER_LONG];
#ifdef CONFIG_SMP
	cpumask_t supported_cpus;
#endif
};

extern struct cobalt_pipeline cobalt_pipeline;

static inline unsigned long xnarch_timer_calibrate(void)
{
	return cobalt_machine.calibrate();
}

#ifndef xnarch_cache_aliasing
#define xnarch_cache_aliasing()  0
#endif

#endif /* !_COBALT_ASM_GENERIC_MACHINE_H */
