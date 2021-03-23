/*
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
#ifndef _COBALT_UAPI_KERNEL_THREAD_H
#define _COBALT_UAPI_KERNEL_THREAD_H

#include <cobalt/uapi/kernel/types.h>

/**
 * @ingroup cobalt_core_thread
 * @defgroup cobalt_core_thread_states Thread state flags
 * @brief Bits reporting permanent or transient states of threads
 * @{
 */

/* State flags (shared) */

#define XNSUSP    0x00000001 /**< Suspended. */
#define XNPEND    0x00000002 /**< Sleep-wait for a resource. */
#define XNDELAY   0x00000004 /**< Delayed */
#define XNREADY   0x00000008 /**< Linked to the ready queue. */
#define XNDORMANT 0x00000010 /**< Not started yet */
#define XNZOMBIE  0x00000020 /**< Zombie thread in deletion process */
#define XNMAPPED  0x00000040 /**< Thread is mapped to a linux task */
#define XNRELAX   0x00000080 /**< Relaxed shadow thread (blocking bit) */
#define XNMIGRATE 0x00000100 /**< Thread is currently migrating to another CPU. */
#define XNHELD    0x00000200 /**< Thread is held to process emergency. */
#define XNBOOST   0x00000400 /**< PI/PP boost undergoing */
#define XNSSTEP   0x00000800 /**< Single-stepped by debugger */
#define XNLOCK    0x00001000 /**< Scheduler lock control (pseudo-bit, not in ->state) */
#define XNRRB     0x00002000 /**< Undergoes a round-robin scheduling */
#define XNWARN    0x00004000 /**< Issue SIGDEBUG on error detection */
#define XNFPU     0x00008000 /**< Thread uses FPU */
#define XNROOT    0x00010000 /**< Root thread (that is, Linux/IDLE) */
#define XNWEAK    0x00020000 /**< Non real-time shadow (from the WEAK class) */
#define XNUSER    0x00040000 /**< Shadow thread running in userland */
#define XNJOINED  0x00080000 /**< Another thread waits for joining this thread */
#define XNTRAPLB  0x00100000 /**< Trap lock break (i.e. may not sleep with sched lock) */
#define XNDEBUG   0x00200000 /**< User-level debugging enabled */
#define XNDBGSTOP 0x00400000 /**< Stopped for synchronous debugging */

/** @} */

/**
 * @ingroup cobalt_core_thread
 * @defgroup cobalt_core_thread_info Thread information flags
 * @brief Bits reporting events notified to threads
 * @{
 */

/* Information flags (shared) */

#define XNTIMEO   0x00000001 /**< Woken up due to a timeout condition */
#define XNRMID    0x00000002 /**< Pending on a removed resource */
#define XNBREAK   0x00000004 /**< Forcibly awaken from a wait state */
#define XNKICKED  0x00000008 /**< Forced out of primary mode */
#define XNWAKEN   0x00000010 /**< Thread waken up upon resource availability */
#define XNROBBED  0x00000020 /**< Robbed from resource ownership */
#define XNCANCELD 0x00000040 /**< Cancellation request is pending */
#define XNPIALERT 0x00000080 /**< Priority inversion alert (SIGDEBUG sent) */
#define XNSCHEDP  0x00000100 /**< schedparam propagation is pending */
#define XNCONTHI  0x00000200 /**< Continue in primary mode after debugging */

/* Local information flags (private to current thread) */

#define XNMOVED   0x00000001 /**< CPU migration in primary mode occurred */
#define XNLBALERT 0x00000002 /**< Scheduler lock break alert (SIGDEBUG sent) */
#define XNDESCENT 0x00000004 /**< Adaptive transitioning to secondary mode */
#define XNSYSRST  0x00000008 /**< Thread awaiting syscall restart after signal */
#define XNHICCUP  0x00000010 /**< Just left from ptracing */

/** @} */

/*
 * Must follow strictly the declaration order of the state flags
 * defined above. Status symbols are defined as follows:
 *
 * 'S' -> Forcibly suspended.
 * 'w'/'W' -> Waiting for a resource, with or without timeout.
 * 'D' -> Delayed (without any other wait condition).
 * 'R' -> Runnable.
 * 'U' -> Unstarted or dormant.
 * 'X' -> Relaxed shadow.
 * 'H' -> Held in emergency.
 * 'b' -> Priority boost undergoing.
 * 'T' -> Ptraced and stopped.
 * 'l' -> Locks scheduler.
 * 'r' -> Undergoes round-robin.
 * 't' -> Runtime mode errors notified.
 * 'L' -> Lock breaks trapped.
 * 's' -> Ptraced, stopped synchronously.
 */
#define XNTHREAD_STATE_LABELS  "SWDRU..X.HbTlrt.....L.s"

struct xnthread_user_window {
	__u32 state;
	__u32 info;
	__u32 grant_value;
	__u32 pp_pending;
};

#endif /* !_COBALT_UAPI_KERNEL_THREAD_H */
