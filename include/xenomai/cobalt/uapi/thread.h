/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_UAPI_THREAD_H
#define _COBALT_UAPI_THREAD_H

#include <cobalt/uapi/kernel/thread.h>

#define PTHREAD_WARNSW             XNWARN
#define PTHREAD_LOCK_SCHED         XNLOCK
#define PTHREAD_DISABLE_LOCKBREAK  XNTRAPLB
#define PTHREAD_CONFORMING     0

struct cobalt_mutexattr {
	int type : 3;
	int protocol : 3;
	int pshared : 1;
	int __pad : 1;
};

struct cobalt_condattr {
	int clock : 7;
	int pshared : 1;
};

struct cobalt_threadstat {
	__u64 xtime;
	__u64 timeout;
	__u64 msw;
	__u64 csw;
	__u64 xsc;
	__u32 status;
	__u32 pf;
	int cpu;
	int cprio;
	char name[XNOBJECT_NAME_LEN];
	char personality[XNOBJECT_NAME_LEN];
};

#endif /* !_COBALT_UAPI_THREAD_H */
