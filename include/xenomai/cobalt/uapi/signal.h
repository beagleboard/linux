/*
 * Copyright (C) 2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_SIGNAL_H
#define _COBALT_UAPI_SIGNAL_H

/*
 * Those are pseudo-signals only available with pthread_kill() to
 * suspend/resume/unblock threads synchronously, force them out of
 * primary mode or even demote them to the SCHED_OTHER class via the
 * low-level nucleus interface. Can't block those signals, queue them,
 * or even set them in a sigset. Those are nasty, strictly anti-POSIX
 * things; we do provide them nevertheless only because we are mean
 * people doing harmful code for no valid reason. Can't go against
 * your nature, right?  Nah... (this said, don't blame us for POSIX,
 * we are not _that_ mean).
 */
#define SIGSUSP (SIGRTMAX + 1)
#define SIGRESM (SIGRTMAX + 2)
#define SIGRELS (SIGRTMAX + 3)
#define SIGKICK (SIGRTMAX + 4)
#define SIGDEMT (SIGRTMAX + 5)

/*
 * Regular POSIX signals with specific handling by Xenomai.
 */
#define SIGSHADOW			SIGWINCH
#define sigshadow_action(code)		((code) & 0xff)
#define sigshadow_arg(code)		(((code) >> 8) & 0xff)
#define sigshadow_int(action, arg)	((action) | ((arg) << 8))

/* SIGSHADOW action codes. */
#define SIGSHADOW_ACTION_HARDEN		1
#define SIGSHADOW_ACTION_BACKTRACE	2
#define SIGSHADOW_ACTION_HOME		3
#define SIGSHADOW_BACKTRACE_DEPTH	16

#define SIGDEBUG			SIGXCPU
#define sigdebug_code(si)		((si)->si_value.sival_int)
#define sigdebug_reason(si)		(sigdebug_code(si) & 0xff)
#define sigdebug_marker			0xfccf0000
#define sigdebug_marked(si)		\
	((sigdebug_code(si) & 0xffff0000) == sigdebug_marker)

/* Possible values of sigdebug_reason() */
#define SIGDEBUG_UNDEFINED		0
#define SIGDEBUG_MIGRATE_SIGNAL		1
#define SIGDEBUG_MIGRATE_SYSCALL	2
#define SIGDEBUG_MIGRATE_FAULT		3
#define SIGDEBUG_MIGRATE_PRIOINV	4
#define SIGDEBUG_NOMLOCK		5
#define SIGDEBUG_WATCHDOG		6
#define SIGDEBUG_RESCNT_IMBALANCE	7
#define SIGDEBUG_LOCK_BREAK		8
#define SIGDEBUG_MUTEX_SLEEP		9

#define COBALT_DELAYMAX			2147483647U

/*
 * Internal accessors to extra siginfo/sigevent fields, extending some
 * existing base field. The extra data should be grouped in a
 * dedicated struct type. The extra space is taken from the padding
 * area available from the original structure definitions.
 *
 * e.g. getting the address of the following extension to
 * _sifields._rt from siginfo_t,
 *
 * struct bar {
 *    int foo;
 * };
 *
 * would be noted as:
 *
 * siginfo_t si;
 * struct bar *p = __cobalt_si_extra(&si, _rt, struct bar);
 *
 * This code is shared between kernel and user space. Proper
 * definitions of siginfo_t and sigevent_t should have been read prior
 * to including this file.
 *
 * CAUTION: this macro does not handle alignment issues for the extra
 * data. The extra type definition should take care of this.
 */
#ifdef __OPTIMIZE__
extern void *__siginfo_overflow(void);
static inline
const void *__check_si_overflow(size_t fldsz, size_t extrasz, const void *p)
{
	siginfo_t *si __attribute__((unused));

	if (fldsz + extrasz <= sizeof(si->_sifields))
		return p;

	return __siginfo_overflow();
}
#define __cobalt_si_extra(__si, __basefield, __type)				\
	((__type *)__check_si_overflow(sizeof(__si->_sifields.__basefield),	\
	       sizeof(__type), &(__si->_sifields.__basefield) + 1))
#else
#define __cobalt_si_extra(__si, __basefield, __type)				\
	((__type *)((&__si->_sifields.__basefield) + 1))
#endif

/* Same approach, this time for extending sigevent_t. */

#ifdef __OPTIMIZE__
extern void *__sigevent_overflow(void);
static inline
const void *__check_sev_overflow(size_t fldsz, size_t extrasz, const void *p)
{
	sigevent_t *sev __attribute__((unused));

	if (fldsz + extrasz <= sizeof(sev->_sigev_un))
		return p;

	return __sigevent_overflow();
}
#define __cobalt_sev_extra(__sev, __basefield, __type)				\
	((__type *)__check_sev_overflow(sizeof(__sev->_sigev_un.__basefield),	\
	       sizeof(__type), &(__sev->_sigev_un.__basefield) + 1))
#else
#define __cobalt_sev_extra(__sev, __basefield, __type)				\
	((__type *)((&__sev->_sigev_un.__basefield) + 1))
#endif

#endif /* !_COBALT_UAPI_SIGNAL_H */
