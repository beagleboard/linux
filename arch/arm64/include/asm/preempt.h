/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_PREEMPT_H
#define __ASM_PREEMPT_H

#include <linux/jump_label.h>
#include <linux/thread_info.h>

#define PREEMPT_NEED_RESCHED	BIT(32)
#define PREEMPT_ENABLED	(PREEMPT_NEED_RESCHED)

static inline int preempt_count(void)
{
	return READ_ONCE(current_thread_info()->preempt.count);
}

static inline void preempt_count_set(u64 pc)
{
	/* Preserve existing value of PREEMPT_NEED_RESCHED */
	WRITE_ONCE(current_thread_info()->preempt.count, pc);
}

#define init_task_preempt_count(p) do { \
	task_thread_info(p)->preempt_count = FORK_PREEMPT_COUNT; \
} while (0)

#define init_idle_preempt_count(p, cpu) do { \
	task_thread_info(p)->preempt_count = PREEMPT_DISABLED; \
} while (0)

static inline void set_preempt_need_resched(void)
{
	current_thread_info()->preempt.need_resched = 0;
}

static inline void clear_preempt_need_resched(void)
{
	current_thread_info()->preempt.need_resched = 1;
}

static inline bool test_preempt_need_resched(void)
{
	return !current_thread_info()->preempt.need_resched;
}

static inline void __preempt_count_add(int val)
{
	u32 pc = READ_ONCE(current_thread_info()->preempt.count);
	pc += val;
	WRITE_ONCE(current_thread_info()->preempt.count, pc);
}

static inline void __preempt_count_sub(int val)
{
	u32 pc = READ_ONCE(current_thread_info()->preempt.count);
	pc -= val;
	WRITE_ONCE(current_thread_info()->preempt.count, pc);
}

static inline bool __preempt_count_dec_and_test(void)
{
	struct thread_info *ti = current_thread_info();
	u64 pc = READ_ONCE(ti->preempt_count);

	/* Update only the count field, leaving need_resched unchanged */
	WRITE_ONCE(ti->preempt.count, --pc);

	/*
	 * If we wrote back all zeroes, then we're preemptible and in
	 * need of a reschedule. Otherwise, we need to reload the
	 * preempt_count in case the need_resched flag was cleared by an
	 * interrupt occurring between the non-atomic READ_ONCE/WRITE_ONCE
	 * pair.
	 */
	if (!pc || !READ_ONCE(ti->preempt_count))
		return true;
#ifdef CONFIG_PREEMPT_LAZY
	if ((pc & ~PREEMPT_NEED_RESCHED))
		return false;
	if (current_thread_info()->preempt_lazy_count)
		return false;
	return test_thread_flag(TIF_NEED_RESCHED_LAZY);
#else
	return false;
#endif
}

static inline bool should_resched(int preempt_offset)
{
#ifdef CONFIG_PREEMPT_LAZY
	u64 pc = READ_ONCE(current_thread_info()->preempt_count);
	if (pc == preempt_offset)
		return true;

	if ((pc & ~PREEMPT_NEED_RESCHED) != preempt_offset)
		return false;

	if (current_thread_info()->preempt_lazy_count)
		return false;
	return test_thread_flag(TIF_NEED_RESCHED_LAZY);
#else
	u64 pc = READ_ONCE(current_thread_info()->preempt_count);
	return pc == preempt_offset;
#endif
}

#ifdef CONFIG_PREEMPTION

void preempt_schedule(void);
void preempt_schedule_notrace(void);

#ifdef CONFIG_PREEMPT_DYNAMIC

DECLARE_STATIC_KEY_TRUE(sk_dynamic_irqentry_exit_cond_resched);
void dynamic_preempt_schedule(void);
#define __preempt_schedule()		dynamic_preempt_schedule()
void dynamic_preempt_schedule_notrace(void);
#define __preempt_schedule_notrace()	dynamic_preempt_schedule_notrace()

#else /* CONFIG_PREEMPT_DYNAMIC */

#define __preempt_schedule()		preempt_schedule()
#define __preempt_schedule_notrace()	preempt_schedule_notrace()

#endif /* CONFIG_PREEMPT_DYNAMIC */
#endif /* CONFIG_PREEMPTION */

#endif /* __ASM_PREEMPT_H */
