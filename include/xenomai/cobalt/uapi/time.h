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
#ifndef _COBALT_UAPI_TIME_H
#define _COBALT_UAPI_TIME_H

#ifndef CLOCK_MONOTONIC_RAW
#define CLOCK_MONOTONIC_RAW  4
#endif

/*
 * Additional clock ids we manage are supposed not to collide with any
 * of the POSIX and Linux kernel definitions so that no ambiguities
 * arise when porting applications in both directions.
 *
 * 0  .. 31   regular POSIX/linux clock ids.
 * 32 .. 63   statically reserved Cobalt clocks
 * 64 .. 127  dynamically registered Cobalt clocks (external)
 *
 * CAUTION: clock ids must fit within a 7bit value, see
 * include/cobalt/uapi/thread.h (e.g. cobalt_condattr).
 */
#define __COBALT_CLOCK_STATIC(nr)	((clockid_t)(nr + 32))

#define CLOCK_HOST_REALTIME  __COBALT_CLOCK_STATIC(0)

#define COBALT_MAX_EXTCLOCKS  64

#define __COBALT_CLOCK_EXT(nr)		((clockid_t)(nr) | (1 << 6))
#define __COBALT_CLOCK_EXT_P(id)	((int)(id) >= 64 && (int)(id) < 128)
#define __COBALT_CLOCK_EXT_INDEX(id)	((int)(id) & ~(1 << 6))

/*
 * Additional timerfd defines
 *
 * when passing TFD_WAKEUP to timer_settime, any timer expiration
 * unblocks the thread having issued timer_settime.
 */
#define TFD_WAKEUP	(1 << 2)

#endif /* !_COBALT_UAPI_TIME_H */
