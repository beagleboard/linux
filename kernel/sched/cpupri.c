// SPDX-License-Identifier: GPL-2.0-only
/*
 *  kernel/sched/cpupri.c
 *
 *  CPU priority management
 *
 *  Copyright (C) 2007-2008 Novell
 *
 *  Author: Gregory Haskins <ghaskins@novell.com>
 *
 *  This code tracks the priority of each CPU so that global migration
 *  decisions are easy to calculate.  Each CPU can be in a state as follows:
 *
 *                 (INVALID), IDLE, NORMAL, RT1, ... RT99
 *
 *  going from the lowest priority to the highest.  CPUs in the INVALID state
 *  are not eligible for routing.  The system maintains this state with
 *  a 2 dimensional bitmap (the first for priority class, the second for CPUs
 *  in that class).  Therefore a typical application without affinity
 *  restrictions can find a suitable CPU with O(1) complexity (e.g. two bit
 *  searches).  For tasks with affinity restrictions, the algorithm has a
 *  worst case complexity of O(min(102, nr_domcpus)), though the scenario that
 *  yields the worst case search is fairly contrived.
 */
#include "sched.h"

/* Convert between a 140 based task->prio, and our 102 based cpupri */
static int convert_prio(int prio)
{
	int cpupri;

	if (prio == CPUPRI_INVALID)
		cpupri = CPUPRI_INVALID;
	else if (prio == MAX_PRIO)
		cpupri = CPUPRI_IDLE;
	else if (prio >= MAX_RT_PRIO)
		cpupri = CPUPRI_NORMAL;
	else
		cpupri = MAX_RT_PRIO - prio + 1;

	return cpupri;
}

#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
/**
 * drop_nopreempt_cpus - remove likely nonpreemptible cpus from the mask
 * @lowest_mask: mask with selected CPUs (non-NULL)
 */
static void
drop_nopreempt_cpus(struct cpumask *lowest_mask)
{
	unsigned int cpu = cpumask_first(lowest_mask);
	while (cpu < nr_cpu_ids) {
		/* unlocked access */
		struct task_struct *task = READ_ONCE(cpu_rq(cpu)->curr);
		if (task_may_not_preempt(task, cpu)) {
			cpumask_clear_cpu(cpu, lowest_mask);
		}
		cpu = cpumask_next(cpu, lowest_mask);
	}
}
#endif

static inline int __cpupri_find(struct cpupri *cp, struct task_struct *p,
				struct cpumask *lowest_mask, int idx,
				bool drop_nopreempts)
{
	struct cpupri_vec *vec  = &cp->pri_to_cpu[idx];
	int skip = 0;

	if (!atomic_read(&(vec)->count))
		skip = 1;
	/*
	 * When looking at the vector, we need to read the counter,
	 * do a memory barrier, then read the mask.
	 *
	 * Note: This is still all racey, but we can deal with it.
	 *  Ideally, we only want to look at masks that are set.
	 *
	 *  If a mask is not set, then the only thing wrong is that we
	 *  did a little more work than necessary.
	 *
	 *  If we read a zero count but the mask is set, because of the
	 *  memory barriers, that can only happen when the highest prio
	 *  task for a run queue has left the run queue, in which case,
	 *  it will be followed by a pull. If the task we are processing
	 *  fails to find a proper place to go, that pull request will
	 *  pull this task if the run queue is running at a lower
	 *  priority.
	 */
	smp_rmb();

	/* Need to do the rmb for every iteration */
	if (skip)
		return 0;

	if (cpumask_any_and(p->cpus_ptr, vec->mask) >= nr_cpu_ids)
		return 0;

	if (lowest_mask) {
		cpumask_and(lowest_mask, p->cpus_ptr, vec->mask);
		cpumask_and(lowest_mask, lowest_mask, cpu_active_mask);

#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
		if (drop_nopreempts)
			drop_nopreempt_cpus(lowest_mask);
#endif

		/*
		 * We have to ensure that we have at least one bit
		 * still set in the array, since the map could have
		 * been concurrently emptied between the first and
		 * second reads of vec->mask.  If we hit this
		 * condition, simply act as though we never hit this
		 * priority level and continue on.
		 */
		if (cpumask_empty(lowest_mask))
			return 0;
	}

	return 1;
}

int cpupri_find(struct cpupri *cp, struct task_struct *p,
		struct cpumask *lowest_mask)
{
	return cpupri_find_fitness(cp, p, lowest_mask, NULL);
}

/**
 * cpupri_find_fitness - find the best (lowest-pri) CPU in the system
 * @cp: The cpupri context
 * @p: The task
 * @lowest_mask: A mask to fill in with selected CPUs (or NULL)
 * @fitness_fn: A pointer to a function to do custom checks whether the CPU
 *              fits a specific criteria so that we only return those CPUs.
 *
 * Note: This function returns the recommended CPUs as calculated during the
 * current invocation.  By the time the call returns, the CPUs may have in
 * fact changed priorities any number of times.  While not ideal, it is not
 * an issue of correctness since the normal rebalancer logic will correct
 * any discrepancies created by racing against the uncertainty of the current
 * priority configuration.
 *
 * Return: (int)bool - CPUs were found
 */
int cpupri_find_fitness(struct cpupri *cp, struct task_struct *p,
		struct cpumask *lowest_mask,
		bool (*fitness_fn)(struct task_struct *p, int cpu))
{
	int task_pri = convert_prio(p->prio);
	int idx, cpu;
	bool drop_nopreempts = task_pri <= MAX_RT_PRIO;

	BUG_ON(task_pri >= CPUPRI_NR_PRIORITIES);

#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
retry:
#endif
	for (idx = 0; idx < task_pri; idx++) {

		if (!__cpupri_find(cp, p, lowest_mask, idx, drop_nopreempts))
			continue;

		if (!lowest_mask || !fitness_fn)
			return 1;

		/* Ensure the capacity of the CPUs fit the task */
		for_each_cpu(cpu, lowest_mask) {
			if (!fitness_fn(p, cpu))
				cpumask_clear_cpu(cpu, lowest_mask);
		}

		/*
		 * If no CPU at the current priority can fit the task
		 * continue looking
		 */
		if (cpumask_empty(lowest_mask))
			continue;

		return 1;
	}

	/*
	 * If we can't find any non-preemptible cpu's, retry so we can
	 * find the lowest priority target and avoid priority inversion.
	 */
#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
	if (drop_nopreempts) {
		drop_nopreempts = false;
		goto retry;
	}
#endif

	/*
	 * If we failed to find a fitting lowest_mask, kick off a new search
	 * but without taking into account any fitness criteria this time.
	 *
	 * This rule favours honouring priority over fitting the task in the
	 * correct CPU (Capacity Awareness being the only user now).
	 * The idea is that if a higher priority task can run, then it should
	 * run even if this ends up being on unfitting CPU.
	 *
	 * The cost of this trade-off is not entirely clear and will probably
	 * be good for some workloads and bad for others.
	 *
	 * The main idea here is that if some CPUs were overcommitted, we try
	 * to spread which is what the scheduler traditionally did. Sys admins
	 * must do proper RT planning to avoid overloading the system if they
	 * really care.
	 */
	if (fitness_fn)
		return cpupri_find(cp, p, lowest_mask);

	return 0;
}
EXPORT_SYMBOL_GPL(cpupri_find_fitness);

/**
 * cpupri_set - update the CPU priority setting
 * @cp: The cpupri context
 * @cpu: The target CPU
 * @newpri: The priority (INVALID-RT99) to assign to this CPU
 *
 * Note: Assumes cpu_rq(cpu)->lock is locked
 *
 * Returns: (void)
 */
void cpupri_set(struct cpupri *cp, int cpu, int newpri)
{
	int *currpri = &cp->cpu_to_pri[cpu];
	int oldpri = *currpri;
	int do_mb = 0;

	newpri = convert_prio(newpri);

	BUG_ON(newpri >= CPUPRI_NR_PRIORITIES);

	if (newpri == oldpri)
		return;

	/*
	 * If the CPU was currently mapped to a different value, we
	 * need to map it to the new value then remove the old value.
	 * Note, we must add the new value first, otherwise we risk the
	 * cpu being missed by the priority loop in cpupri_find.
	 */
	if (likely(newpri != CPUPRI_INVALID)) {
		struct cpupri_vec *vec = &cp->pri_to_cpu[newpri];

		cpumask_set_cpu(cpu, vec->mask);
		/*
		 * When adding a new vector, we update the mask first,
		 * do a write memory barrier, and then update the count, to
		 * make sure the vector is visible when count is set.
		 */
		smp_mb__before_atomic();
		atomic_inc(&(vec)->count);
		do_mb = 1;
	}
	if (likely(oldpri != CPUPRI_INVALID)) {
		struct cpupri_vec *vec  = &cp->pri_to_cpu[oldpri];

		/*
		 * Because the order of modification of the vec->count
		 * is important, we must make sure that the update
		 * of the new prio is seen before we decrement the
		 * old prio. This makes sure that the loop sees
		 * one or the other when we raise the priority of
		 * the run queue. We don't care about when we lower the
		 * priority, as that will trigger an rt pull anyway.
		 *
		 * We only need to do a memory barrier if we updated
		 * the new priority vec.
		 */
		if (do_mb)
			smp_mb__after_atomic();

		/*
		 * When removing from the vector, we decrement the counter first
		 * do a memory barrier and then clear the mask.
		 */
		atomic_dec(&(vec)->count);
		smp_mb__after_atomic();
		cpumask_clear_cpu(cpu, vec->mask);
	}

	*currpri = newpri;
}

/**
 * cpupri_init - initialize the cpupri structure
 * @cp: The cpupri context
 *
 * Return: -ENOMEM on memory allocation failure.
 */
int cpupri_init(struct cpupri *cp)
{
	int i;

	for (i = 0; i < CPUPRI_NR_PRIORITIES; i++) {
		struct cpupri_vec *vec = &cp->pri_to_cpu[i];

		atomic_set(&vec->count, 0);
		if (!zalloc_cpumask_var(&vec->mask, GFP_KERNEL))
			goto cleanup;
	}

	cp->cpu_to_pri = kcalloc(nr_cpu_ids, sizeof(int), GFP_KERNEL);
	if (!cp->cpu_to_pri)
		goto cleanup;

	for_each_possible_cpu(i)
		cp->cpu_to_pri[i] = CPUPRI_INVALID;

	return 0;

cleanup:
	for (i--; i >= 0; i--)
		free_cpumask_var(cp->pri_to_cpu[i].mask);
	return -ENOMEM;
}

/**
 * cpupri_cleanup - clean up the cpupri structure
 * @cp: The cpupri context
 */
void cpupri_cleanup(struct cpupri *cp)
{
	int i;

	kfree(cp->cpu_to_pri);
	for (i = 0; i < CPUPRI_NR_PRIORITIES; i++)
		free_cpumask_var(cp->pri_to_cpu[i].mask);
}

#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
/*
 * cpupri_check_rt - check if CPU has a RT task
 * should be called from rcu-sched read section.
 */
bool cpupri_check_rt(void)
{
	int cpu = raw_smp_processor_id();

	return cpu_rq(cpu)->rd->cpupri.cpu_to_pri[cpu] > CPUPRI_NORMAL;
}
#endif
