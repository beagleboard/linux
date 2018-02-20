/*
 * Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_TESTING_H
#define _COBALT_RTDM_TESTING_H

#include <rtdm/rtdm.h>
#include <rtdm/uapi/testing.h>

#ifdef CONFIG_XENO_ARCH_SYS3264

#include <rtdm/compat.h>

struct compat_rttst_overall_bench_res {
	struct rttst_bench_res result;
	compat_uptr_t histogram_avg;
	compat_uptr_t histogram_min;
	compat_uptr_t histogram_max;
};

#define RTTST_RTIOC_TMBENCH_STOP_COMPAT \
	_IOWR(RTIOC_TYPE_TESTING, 0x11, struct compat_rttst_overall_bench_res)

#endif	/* CONFIG_XENO_ARCH_SYS3264 */

#endif /* !_COBALT_RTDM_TESTING_H */
