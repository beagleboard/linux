/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_POSIX_EXTENSION_H
#define _COBALT_POSIX_EXTENSION_H

#include <linux/time.h>
#include <linux/list.h>

#ifdef CONFIG_XENO_OPT_COBALT_EXTENSION

#include <cobalt/kernel/thread.h>

struct cobalt_timer;
struct cobalt_sigpending;
struct cobalt_extref;
struct siginfo;
struct xnsched_class;
union xnsched_policy_param;

struct cobalt_extension {
	struct xnthread_personality core;
	struct {
		struct cobalt_thread *
		(*timer_init)(struct cobalt_extref *reftimer, /* nklocked, IRQs off. */
			      const struct sigevent *__restrict__ evp);
		int (*timer_settime)(struct cobalt_extref *reftimer, /* nklocked, IRQs off. */
				     const struct itimerspec *__restrict__ value,
				     int flags);
		int (*timer_gettime)(struct cobalt_extref *reftimer, /* nklocked, IRQs off. */
				     struct itimerspec *__restrict__ value);
		int (*timer_delete)(struct cobalt_extref *reftimer); /* nklocked, IRQs off. */
		int (*timer_cleanup)(struct cobalt_extref *reftimer); /* nklocked, IRQs off. */
		int (*signal_deliver)(struct cobalt_extref *refthread,
				      struct siginfo *si,
				      struct cobalt_sigpending *sigp);
		int (*signal_queue)(struct cobalt_extref *refthread,
				    struct cobalt_sigpending *sigp);
		int (*signal_copyinfo)(struct cobalt_extref *refthread,
				       void __user *u_si,
				       const struct siginfo *si,
				       int overrun);
		int (*signal_copyinfo_compat)(struct cobalt_extref *refthread,
					      void __user *u_si,
					      const struct siginfo *si,
					      int overrun);
		int (*sched_yield)(struct cobalt_extref *curref);
		int (*thread_setsched)(struct cobalt_extref *refthread, /* nklocked, IRQs off. */
				       struct xnsched_class *sched_class,
				       union xnsched_policy_param *param);
	} ops;
};

struct cobalt_extref {
	struct cobalt_extension *extension;
	struct list_head next;
	void *private;
};

static inline void cobalt_set_extref(struct cobalt_extref *ref,
				     struct cobalt_extension *ext,
				     void *priv)
{
	ref->extension = ext;
	ref->private = priv;
}

/**
 * All macros return non-zero if some thread-level extension code was
 * called, leaving the output value into __ret. Otherwise, the __ret
 * value is undefined.
 */
#define cobalt_initcall_extension(__extfn, __extref, __owner, __ret, __args...) \
	({									\
		int __val = 0;							\
		if ((__owner) && (__owner)->extref.extension) {			\
			(__extref)->extension = (__owner)->extref.extension;	\
			if ((__extref)->extension->ops.__extfn) {		\
				(__ret) = (__extref)->extension->ops.		\
					__extfn(__extref, ##__args );		\
				__val = 1;					\
			}							\
		} else								\
			(__extref)->extension = NULL;				\
		__val;								\
	})
		
#define cobalt_call_extension(__extfn, __extref, __ret, __args...)	\
	({								\
		int __val = 0;						\
		if ((__extref)->extension &&				\
		    (__extref)->extension->ops.__extfn) {		\
			(__ret) = (__extref)->extension->ops.		\
				__extfn(__extref, ##__args );		\
			__val = 1;					\
		}							\
		__val;							\
	})

#else /* !CONFIG_XENO_OPT_COBALT_EXTENSION */

struct cobalt_extension;

struct cobalt_extref {
};

static inline void cobalt_set_extref(struct cobalt_extref *ref,
				     struct cobalt_extension *ext,
				     void *priv)
{
}

#define cobalt_initcall_extension(__extfn, __extref, __owner, __ret, __args...)	\
	({ (void)(__owner); (void)(__ret); 0; })

#define cobalt_call_extension(__extfn, __extref, __ret, __args...)	\
	({ (void)(__ret); 0; })

#endif /* !CONFIG_XENO_OPT_COBALT_EXTENSION */

#endif /* !_COBALT_POSIX_EXTENSION_H */
