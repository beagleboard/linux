/*
 * Copyright (C) 2006 Jan Kiszka <jan.kiszka@web.de>.
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
#ifndef _COBALT_KERNEL_TRACE_H
#define _COBALT_KERNEL_TRACE_H

#include <linux/types.h>
#include <linux/ipipe_trace.h>
#include <cobalt/uapi/kernel/trace.h>

static inline int xntrace_max_begin(unsigned long v)
{
	ipipe_trace_begin(v);
	return 0;
}

static inline int xntrace_max_end(unsigned long v)
{
	ipipe_trace_end(v);
	return 0;
}

static inline int xntrace_max_reset(void)
{
	ipipe_trace_max_reset();
	return 0;
}

static inline int xntrace_user_start(void)
{
	return ipipe_trace_frozen_reset();
}

static inline int xntrace_user_stop(unsigned long v)
{
	ipipe_trace_freeze(v);
	return 0;
}

static inline int xntrace_user_freeze(unsigned long v, int once)
{
	int ret = 0;

	if (!once)
		ret = ipipe_trace_frozen_reset();

	ipipe_trace_freeze(v);

	return ret;
}

static inline int xntrace_special(unsigned char id, unsigned long v)
{
	ipipe_trace_special(id, v);
	return 0;
}

static inline int xntrace_special_u64(unsigned char id,
				      unsigned long long v)
{
	ipipe_trace_special(id, (unsigned long)(v >> 32));
	ipipe_trace_special(id, (unsigned long)(v & 0xFFFFFFFF));
	return 0;
}

static inline int xntrace_pid(pid_t pid, short prio)
{
	ipipe_trace_pid(pid, prio);
	return 0;
}

static inline int xntrace_tick(unsigned long delay_ticks)
{
	ipipe_trace_event(0, delay_ticks);
	return 0;
}

static inline int xntrace_panic_freeze(void)
{
	ipipe_trace_panic_freeze();
	return 0;
}

static inline int xntrace_panic_dump(void)
{
	ipipe_trace_panic_dump();
	return 0;
}

#endif /* !_COBALT_KERNEL_TRACE_H */
