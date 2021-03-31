/*
 * Copyright (C) 2001-2008,2012 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2004,2005 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
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
#ifndef _COBALT_KERNEL_LOCK_H
#define _COBALT_KERNEL_LOCK_H

#include <linux/ipipe.h>
#include <linux/percpu.h>
#include <cobalt/kernel/assert.h>

/**
 * @addtogroup cobalt_core_lock
 *
 * @{
 */
typedef unsigned long spl_t;

/**
 * Hard disable interrupts on the local processor, saving previous state.
 *
 * @param[out] x An unsigned long integer context variable
 */
#define splhigh(x)  ((x) = ipipe_test_and_stall_head() & 1)
#ifdef CONFIG_SMP
/**
 * Restore the saved hard interrupt state on the local processor.
 *
 * @param[in] x The context variable previously updated by splhigh()
 */
#define splexit(x)  ipipe_restore_head(x & 1)
#else /* !CONFIG_SMP */
#define splexit(x)  ipipe_restore_head(x)
#endif /* !CONFIG_SMP */
/**
 * Hard disable interrupts on the local processor.
 */
#define splmax()    ipipe_stall_head()
/**
 * Hard enable interrupts on the local processor.
 */
#define splnone()   ipipe_unstall_head()
/**
 * Test hard interrupt state on the local processor.
 *
 * @return Zero if the local processor currently accepts interrupts,
 * non-zero otherwise.
 */
#define spltest()   ipipe_test_head()

#ifdef CONFIG_XENO_OPT_DEBUG_LOCKING

struct xnlock {
	unsigned owner;
	arch_spinlock_t alock;
	const char *file;
	const char *function;
	unsigned int line;
	int cpu;
	unsigned long long spin_time;
	unsigned long long lock_date;
};

struct xnlockinfo {
	unsigned long long spin_time;
	unsigned long long lock_time;
	const char *file;
	const char *function;
	unsigned int line;
};

#define XNARCH_LOCK_UNLOCKED (struct xnlock) {	\
	~0,					\
	__ARCH_SPIN_LOCK_UNLOCKED,		\
	NULL,					\
	NULL,					\
	0,					\
	-1,					\
	0LL,					\
	0LL,					\
}

#define XNLOCK_DBG_CONTEXT		, __FILE__, __LINE__, __FUNCTION__
#define XNLOCK_DBG_CONTEXT_ARGS					\
	, const char *file, int line, const char *function
#define XNLOCK_DBG_PASS_CONTEXT		, file, line, function

void xnlock_dbg_prepare_acquire(unsigned long long *start);
void xnlock_dbg_prepare_spin(unsigned int *spin_limit);
void xnlock_dbg_acquired(struct xnlock *lock, int cpu,
			 unsigned long long *start,
			 const char *file, int line,
			 const char *function);
int xnlock_dbg_release(struct xnlock *lock,
			 const char *file, int line,
			 const char *function);

DECLARE_PER_CPU(struct xnlockinfo, xnlock_stats);

#else /* !CONFIG_XENO_OPT_DEBUG_LOCKING */

struct xnlock {
	unsigned owner;
	arch_spinlock_t alock;
};

#define XNARCH_LOCK_UNLOCKED			\
	(struct xnlock) {			\
		~0,				\
		__ARCH_SPIN_LOCK_UNLOCKED,	\
	}

#define XNLOCK_DBG_CONTEXT
#define XNLOCK_DBG_CONTEXT_ARGS
#define XNLOCK_DBG_PASS_CONTEXT

static inline
void xnlock_dbg_prepare_acquire(unsigned long long *start)
{
}

static inline
void xnlock_dbg_prepare_spin(unsigned int *spin_limit)
{
}

static inline void
xnlock_dbg_acquired(struct xnlock *lock, int cpu,
		    unsigned long long *start)
{
}

static inline int xnlock_dbg_release(struct xnlock *lock)
{
	return 0;
}

#endif /* !CONFIG_XENO_OPT_DEBUG_LOCKING */

#if defined(CONFIG_SMP) || defined(CONFIG_XENO_OPT_DEBUG_LOCKING)

#define xnlock_get(lock)		__xnlock_get(lock  XNLOCK_DBG_CONTEXT)
#define xnlock_put(lock)		__xnlock_put(lock  XNLOCK_DBG_CONTEXT)
#define xnlock_get_irqsave(lock,x) \
	((x) = __xnlock_get_irqsave(lock  XNLOCK_DBG_CONTEXT))
#define xnlock_put_irqrestore(lock,x) \
	__xnlock_put_irqrestore(lock,x  XNLOCK_DBG_CONTEXT)
#define xnlock_clear_irqoff(lock)	xnlock_put_irqrestore(lock, 1)
#define xnlock_clear_irqon(lock)	xnlock_put_irqrestore(lock, 0)

static inline void xnlock_init (struct xnlock *lock)
{
	*lock = XNARCH_LOCK_UNLOCKED;
}

#define DECLARE_XNLOCK(lock)		struct xnlock lock
#define DECLARE_EXTERN_XNLOCK(lock)	extern struct xnlock lock
#define DEFINE_XNLOCK(lock)		struct xnlock lock = XNARCH_LOCK_UNLOCKED
#define DEFINE_PRIVATE_XNLOCK(lock)	static DEFINE_XNLOCK(lock)

static inline int ____xnlock_get(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	int cpu = ipipe_processor_id();
	unsigned long long start;

	if (lock->owner == cpu)
		return 2;

	xnlock_dbg_prepare_acquire(&start);

	arch_spin_lock(&lock->alock);
	lock->owner = cpu;

	xnlock_dbg_acquired(lock, cpu, &start /*, */ XNLOCK_DBG_PASS_CONTEXT);

	return 0;
}

static inline void ____xnlock_put(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	if (xnlock_dbg_release(lock /*, */ XNLOCK_DBG_PASS_CONTEXT))
		return;

	lock->owner = ~0U;
	arch_spin_unlock(&lock->alock);
}

#ifndef CONFIG_XENO_ARCH_OUTOFLINE_XNLOCK
#define ___xnlock_get ____xnlock_get
#define ___xnlock_put ____xnlock_put
#else /* out of line xnlock */
int ___xnlock_get(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS);

void ___xnlock_put(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS);
#endif /* out of line xnlock */

#ifdef CONFIG_XENO_OPT_DEBUG_LOCKING
/* Disable UP-over-SMP kernel optimization in debug mode. */
#define __locking_active__  1
#else
#define __locking_active__  ipipe_smp_p
#endif

static inline spl_t
__xnlock_get_irqsave(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	unsigned long flags;

	splhigh(flags);

	if (__locking_active__)
		flags |= ___xnlock_get(lock /*, */ XNLOCK_DBG_PASS_CONTEXT);

	return flags;
}

static inline void __xnlock_put_irqrestore(struct xnlock *lock, spl_t flags
					   /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	/* Only release the lock if we didn't take it recursively. */
	if (__locking_active__ && !(flags & 2))
		___xnlock_put(lock /*, */ XNLOCK_DBG_PASS_CONTEXT);

	splexit(flags & 1);
}

static inline int xnlock_is_owner(struct xnlock *lock)
{
	if (__locking_active__)
		return lock->owner == ipipe_processor_id();

	return 1;
}

static inline int __xnlock_get(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	if (__locking_active__)
		return ___xnlock_get(lock /* , */ XNLOCK_DBG_PASS_CONTEXT);

	return 0;
}

static inline void __xnlock_put(struct xnlock *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	if (__locking_active__)
		___xnlock_put(lock /*, */ XNLOCK_DBG_PASS_CONTEXT);
}

#undef __locking_active__

#else /* !(CONFIG_SMP || CONFIG_XENO_OPT_DEBUG_LOCKING) */

#define xnlock_init(lock)		do { } while(0)
#define xnlock_get(lock)		do { } while(0)
#define xnlock_put(lock)		do { } while(0)
#define xnlock_get_irqsave(lock,x)	splhigh(x)
#define xnlock_put_irqrestore(lock,x)	splexit(x)
#define xnlock_clear_irqoff(lock)	splmax()
#define xnlock_clear_irqon(lock)	splnone()
#define xnlock_is_owner(lock)		1

#define DECLARE_XNLOCK(lock)
#define DECLARE_EXTERN_XNLOCK(lock)
#define DEFINE_XNLOCK(lock)
#define DEFINE_PRIVATE_XNLOCK(lock)

#endif /* !(CONFIG_SMP || CONFIG_XENO_OPT_DEBUG_LOCKING) */

DECLARE_EXTERN_XNLOCK(nklock);

/** @} */

#endif /* !_COBALT_KERNEL_LOCK_H */
