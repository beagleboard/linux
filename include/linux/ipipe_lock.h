/*   -*- linux-c -*-
 *   include/linux/ipipe_lock.h
 *
 *   Copyright (C) 2009 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
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

#ifndef __LINUX_IPIPE_LOCK_H
#define __LINUX_IPIPE_LOCK_H

#include <asm-generic/ipipe.h>

typedef struct {
	arch_spinlock_t arch_lock;
} __ipipe_spinlock_t;

#define ipipe_spinlock(lock)	((__ipipe_spinlock_t *)(lock))
#define ipipe_spinlock_p(lock)							\
	__builtin_types_compatible_p(typeof(lock), __ipipe_spinlock_t *) ||	\
	__builtin_types_compatible_p(typeof(lock), __ipipe_spinlock_t [])

#define std_spinlock_raw(lock)	((raw_spinlock_t *)(lock))
#define std_spinlock_raw_p(lock)					\
	__builtin_types_compatible_p(typeof(lock), raw_spinlock_t *) ||	\
	__builtin_types_compatible_p(typeof(lock), raw_spinlock_t [])

#ifdef CONFIG_PREEMPT_RT_FULL

#define PICK_SPINLOCK_IRQSAVE(lock, flags)				\
	do {								\
		if (ipipe_spinlock_p(lock))				\
			(flags) = __ipipe_spin_lock_irqsave(ipipe_spinlock(lock)); \
		else if (std_spinlock_raw_p(lock))				\
			__real_raw_spin_lock_irqsave(std_spinlock_raw(lock), flags); \
		else __bad_lock_type();					\
	} while (0)

#define PICK_SPINTRYLOCK_IRQSAVE(lock, flags)				\
	({								\
		int __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = __ipipe_spin_trylock_irqsave(ipipe_spinlock(lock), &(flags)); \
		else if (std_spinlock_raw_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irqsave(std_spinlock_raw(lock), flags); \
		else __bad_lock_type();					\
		__ret__;						\
	 })

#define PICK_SPINTRYLOCK_IRQ(lock)					\
	({								\
		int __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = __ipipe_spin_trylock_irq(ipipe_spinlock(lock)); \
		else if (std_spinlock_raw_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irq(std_spinlock_raw(lock)); \
		else __bad_lock_type();					\
		__ret__;						\
	 })

#define PICK_SPINUNLOCK_IRQRESTORE(lock, flags)				\
	do {								\
		if (ipipe_spinlock_p(lock))				\
			__ipipe_spin_unlock_irqrestore(ipipe_spinlock(lock), flags); \
		else if (std_spinlock_raw_p(lock)) {			\
			__ipipe_spin_unlock_debug(flags);		\
			__real_raw_spin_unlock_irqrestore(std_spinlock_raw(lock), flags); \
		} else __bad_lock_type();				\
	} while (0)

#define PICK_SPINOP(op, lock)						\
	({								\
		if (ipipe_spinlock_p(lock))				\
			arch_spin##op(&ipipe_spinlock(lock)->arch_lock); \
		else if (std_spinlock_raw_p(lock))			\
			__real_raw_spin##op(std_spinlock_raw(lock));	\
		else __bad_lock_type();					\
		(void)0;						\
	})

#define PICK_SPINOP_RET(op, lock, type)					\
	({								\
		type __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = arch_spin##op(&ipipe_spinlock(lock)->arch_lock); \
		else if (std_spinlock_raw_p(lock))			\
			__ret__ = __real_raw_spin##op(std_spinlock_raw(lock)); \
		else { __ret__ = -1; __bad_lock_type(); }		\
		__ret__;						\
	})

#else /* !CONFIG_PREEMPT_RT_FULL */

#define std_spinlock(lock)	((spinlock_t *)(lock))
#define std_spinlock_p(lock)						\
	__builtin_types_compatible_p(typeof(lock), spinlock_t *) ||	\
	__builtin_types_compatible_p(typeof(lock), spinlock_t [])

#define PICK_SPINLOCK_IRQSAVE(lock, flags)				\
	do {								\
		if (ipipe_spinlock_p(lock))				\
			(flags) = __ipipe_spin_lock_irqsave(ipipe_spinlock(lock)); \
		else if (std_spinlock_raw_p(lock))				\
			__real_raw_spin_lock_irqsave(std_spinlock_raw(lock), flags); \
		else if (std_spinlock_p(lock))				\
			__real_raw_spin_lock_irqsave(&std_spinlock(lock)->rlock, flags); \
		else __bad_lock_type();					\
	} while (0)

#define PICK_SPINTRYLOCK_IRQSAVE(lock, flags)				\
	({								\
		int __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = __ipipe_spin_trylock_irqsave(ipipe_spinlock(lock), &(flags)); \
		else if (std_spinlock_raw_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irqsave(std_spinlock_raw(lock), flags); \
		else if (std_spinlock_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irqsave(&std_spinlock(lock)->rlock, flags); \
		else __bad_lock_type();					\
		__ret__;						\
	 })

#define PICK_SPINTRYLOCK_IRQ(lock)					\
	({								\
		int __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = __ipipe_spin_trylock_irq(ipipe_spinlock(lock)); \
		else if (std_spinlock_raw_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irq(std_spinlock_raw(lock)); \
		else if (std_spinlock_p(lock))				\
			__ret__ = __real_raw_spin_trylock_irq(&std_spinlock(lock)->rlock); \
		else __bad_lock_type();					\
		__ret__;						\
	 })

#define PICK_SPINUNLOCK_IRQRESTORE(lock, flags)				\
	do {								\
		if (ipipe_spinlock_p(lock))				\
			__ipipe_spin_unlock_irqrestore(ipipe_spinlock(lock), flags); \
		else {							\
			__ipipe_spin_unlock_debug(flags);		\
			if (std_spinlock_raw_p(lock))			\
				__real_raw_spin_unlock_irqrestore(std_spinlock_raw(lock), flags); \
			else if (std_spinlock_p(lock))			\
				__real_raw_spin_unlock_irqrestore(&std_spinlock(lock)->rlock, flags); \
		}							\
	} while (0)

#define PICK_SPINOP(op, lock)						\
	({								\
		if (ipipe_spinlock_p(lock))				\
			arch_spin##op(&ipipe_spinlock(lock)->arch_lock); \
		else if (std_spinlock_raw_p(lock))			\
			__real_raw_spin##op(std_spinlock_raw(lock));	\
		else if (std_spinlock_p(lock))				\
			__real_raw_spin##op(&std_spinlock(lock)->rlock); \
		else __bad_lock_type();					\
		(void)0;						\
	})

#define PICK_SPINOP_RET(op, lock, type)					\
	({								\
		type __ret__;						\
		if (ipipe_spinlock_p(lock))				\
			__ret__ = arch_spin##op(&ipipe_spinlock(lock)->arch_lock); \
		else if (std_spinlock_raw_p(lock))			\
			__ret__ = __real_raw_spin##op(std_spinlock_raw(lock)); \
		else if (std_spinlock_p(lock))				\
			__ret__ = __real_raw_spin##op(&std_spinlock(lock)->rlock); \
		else { __ret__ = -1; __bad_lock_type(); }		\
		__ret__;						\
	})

#endif /* !CONFIG_PREEMPT_RT_FULL */

#define arch_spin_lock_init(lock)					\
	do {								\
		IPIPE_DEFINE_SPINLOCK(__lock__);			\
		*((ipipe_spinlock_t *)lock) = __lock__;			\
	} while (0)

#define arch_spin_lock_irq(lock)					\
	do {								\
		hard_local_irq_disable();				\
		arch_spin_lock(lock);					\
	} while (0)

#define arch_spin_unlock_irq(lock)					\
	do {								\
		arch_spin_unlock(lock);					\
		hard_local_irq_enable();				\
	} while (0)

typedef struct {
	arch_rwlock_t arch_lock;
} __ipipe_rwlock_t;

#define ipipe_rwlock_p(lock)						\
	__builtin_types_compatible_p(typeof(lock), __ipipe_rwlock_t *)

#define std_rwlock_p(lock)						\
	__builtin_types_compatible_p(typeof(lock), rwlock_t *)

#define ipipe_rwlock(lock)	((__ipipe_rwlock_t *)(lock))
#define std_rwlock(lock)	((rwlock_t *)(lock))

#define PICK_RWOP(op, lock)						\
	do {								\
		if (ipipe_rwlock_p(lock))				\
			arch##op(&ipipe_rwlock(lock)->arch_lock);	\
		else if (std_rwlock_p(lock))				\
			_raw##op(std_rwlock(lock));			\
		else __bad_lock_type();					\
	} while (0)

extern int __bad_lock_type(void);

#ifdef CONFIG_IPIPE

#define ipipe_spinlock_t		__ipipe_spinlock_t
#define IPIPE_DEFINE_RAW_SPINLOCK(x)	ipipe_spinlock_t x = IPIPE_SPIN_LOCK_UNLOCKED
#define IPIPE_DECLARE_RAW_SPINLOCK(x)	extern ipipe_spinlock_t x
#define IPIPE_DEFINE_SPINLOCK(x)	IPIPE_DEFINE_RAW_SPINLOCK(x)
#define IPIPE_DECLARE_SPINLOCK(x)	IPIPE_DECLARE_RAW_SPINLOCK(x)

#define IPIPE_SPIN_LOCK_UNLOCKED					\
	(__ipipe_spinlock_t) {	.arch_lock = __ARCH_SPIN_LOCK_UNLOCKED }

#define spin_lock_irqsave_cond(lock, flags) \
	spin_lock_irqsave(lock, flags)

#define spin_unlock_irqrestore_cond(lock, flags) \
	spin_unlock_irqrestore(lock, flags)

#define raw_spin_lock_irqsave_cond(lock, flags) \
	raw_spin_lock_irqsave(lock, flags)

#define raw_spin_unlock_irqrestore_cond(lock, flags) \
	raw_spin_unlock_irqrestore(lock, flags)

void __ipipe_spin_lock_irq(ipipe_spinlock_t *lock);

int __ipipe_spin_trylock_irq(ipipe_spinlock_t *lock);

void __ipipe_spin_unlock_irq(ipipe_spinlock_t *lock);

unsigned long __ipipe_spin_lock_irqsave(ipipe_spinlock_t *lock);

int __ipipe_spin_trylock_irqsave(ipipe_spinlock_t *lock,
				 unsigned long *x);

void __ipipe_spin_unlock_irqrestore(ipipe_spinlock_t *lock,
				    unsigned long x);

void __ipipe_spin_unlock_irqbegin(ipipe_spinlock_t *lock);

void __ipipe_spin_unlock_irqcomplete(unsigned long x);

#if defined(CONFIG_IPIPE_DEBUG_INTERNAL) && defined(CONFIG_SMP)
void __ipipe_spin_unlock_debug(unsigned long flags);
#else
#define __ipipe_spin_unlock_debug(flags)  do { } while (0)
#endif

#define ipipe_rwlock_t			__ipipe_rwlock_t
#define IPIPE_DEFINE_RWLOCK(x)		ipipe_rwlock_t x = IPIPE_RW_LOCK_UNLOCKED
#define IPIPE_DECLARE_RWLOCK(x)		extern ipipe_rwlock_t x

#define IPIPE_RW_LOCK_UNLOCKED	\
	(__ipipe_rwlock_t) { .arch_lock = __ARCH_RW_LOCK_UNLOCKED }

#else /* !CONFIG_IPIPE */

#define ipipe_spinlock_t		spinlock_t
#define IPIPE_DEFINE_SPINLOCK(x)	DEFINE_SPINLOCK(x)
#define IPIPE_DECLARE_SPINLOCK(x)	extern spinlock_t x
#define IPIPE_SPIN_LOCK_UNLOCKED	__SPIN_LOCK_UNLOCKED(unknown)
#define IPIPE_DEFINE_RAW_SPINLOCK(x)	DEFINE_RAW_SPINLOCK(x)
#define IPIPE_DECLARE_RAW_SPINLOCK(x)	extern raw_spinlock_t x

#define spin_lock_irqsave_cond(lock, flags)		\
	do {						\
		(void)(flags);				\
		spin_lock(lock);			\
	} while(0)

#define spin_unlock_irqrestore_cond(lock, flags)	\
	spin_unlock(lock)

#define raw_spin_lock_irqsave_cond(lock, flags) \
	do {					\
		(void)(flags);			\
		raw_spin_lock(lock);		\
	} while(0)

#define raw_spin_unlock_irqrestore_cond(lock, flags) \
	raw_spin_unlock(lock)

#define __ipipe_spin_lock_irq(lock)		do { } while (0)
#define __ipipe_spin_unlock_irq(lock)		do { } while (0)
#define __ipipe_spin_lock_irqsave(lock)		0
#define __ipipe_spin_trylock_irq(lock)		1
#define __ipipe_spin_trylock_irqsave(lock, x)	({ (void)(x); 1; })
#define __ipipe_spin_unlock_irqrestore(lock, x)	do { (void)(x); } while (0)
#define __ipipe_spin_unlock_irqbegin(lock)	spin_unlock(lock)
#define __ipipe_spin_unlock_irqcomplete(x)	do { (void)(x); } while (0)
#define __ipipe_spin_unlock_debug(flags)	do { } while (0)

#define ipipe_rwlock_t			rwlock_t
#define IPIPE_DEFINE_RWLOCK(x)		DEFINE_RWLOCK(x)
#define IPIPE_DECLARE_RWLOCK(x)		extern rwlock_t x
#define IPIPE_RW_LOCK_UNLOCKED		RW_LOCK_UNLOCKED

#endif /* !CONFIG_IPIPE */

#endif /* !__LINUX_IPIPE_LOCK_H */
