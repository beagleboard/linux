/**
 * @file
 * Real-Time Driver Model for Xenomai, testing device profile header
 *
 * @note Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>
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
 *
 * @ingroup rttesting
 */
#ifndef _RTDM_UAPI_TESTING_H
#define _RTDM_UAPI_TESTING_H

#include <linux/types.h>

#define RTTST_PROFILE_VER		2

typedef struct rttst_bench_res {
	__s32 avg;
	__s32 min;
	__s32 max;
	__s32 overruns;
	__s32 test_loops;
} rttst_bench_res_t;

typedef struct rttst_interm_bench_res {
	struct rttst_bench_res last;
	struct rttst_bench_res overall;
} rttst_interm_bench_res_t;

typedef struct rttst_overall_bench_res {
	struct rttst_bench_res result;
	__s32 *histogram_avg;
	__s32 *histogram_min;
	__s32 *histogram_max;
} rttst_overall_bench_res_t;

#define RTTST_TMBENCH_INVALID		-1 /* internal use only */
#define RTTST_TMBENCH_TASK		0
#define RTTST_TMBENCH_HANDLER		1

typedef struct rttst_tmbench_config {
	int mode;
	int priority;
	__u64 period;
	int warmup_loops;
	int histogram_size;
	int histogram_bucketsize;
	int freeze_max;
} rttst_tmbench_config_t;

struct rttst_swtest_task {
	unsigned int index;
	unsigned int flags;
};

/* Possible values for struct rttst_swtest_task::flags. */
#define RTTST_SWTEST_FPU		0x1
#define RTTST_SWTEST_USE_FPU		0x2 /* Only for kernel-space tasks. */
#define RTTST_SWTEST_FREEZE		0x4 /* Only for kernel-space tasks. */

struct rttst_swtest_dir {
	unsigned int from;
	unsigned int to;
};

struct rttst_swtest_error {
	struct rttst_swtest_dir last_switch;
	unsigned int fp_val;
};

#define RTTST_RTDM_NORMAL_CLOSE		0
#define RTTST_RTDM_DEFER_CLOSE_CONTEXT	1

#define RTTST_RTDM_MAGIC_PRIMARY	0xfefbfefb
#define RTTST_RTDM_MAGIC_SECONDARY	0xa5b9a5b9

#define RTTST_HEAPCHECK_ZEROOVRD   1
#define RTTST_HEAPCHECK_SHUFFLE    2
#define RTTST_HEAPCHECK_PATTERN    4
#define RTTST_HEAPCHECK_HOT        8

struct rttst_heap_parms {
	__u64 heap_size;
	__u64 block_size;
	int flags;
	int nrstats;
};

struct rttst_heap_stats {
	__u64 heap_size;
	__u64 user_size;
	__u64 block_size;
	__s64 alloc_avg_ns;
	__s64 alloc_max_ns;
	__s64 free_avg_ns;
	__s64 free_max_ns;
	__u64 maximum_free;
	__u64 largest_free;
	int nrblocks;
	int flags;
};

struct rttst_heap_stathdr {
	int nrstats;
	struct rttst_heap_stats *buf;
};

#define RTIOC_TYPE_TESTING		RTDM_CLASS_TESTING

/*!
 * @name Sub-Classes of RTDM_CLASS_TESTING
 * @{ */
/** subclass name: "timerbench" */
#define RTDM_SUBCLASS_TIMERBENCH	0
/** subclass name: "irqbench" */
#define RTDM_SUBCLASS_IRQBENCH		1
/** subclass name: "switchtest" */
#define RTDM_SUBCLASS_SWITCHTEST	2
/** subclase name: "rtdm" */
#define RTDM_SUBCLASS_RTDMTEST		3
/** subclase name: "heapcheck" */
#define RTDM_SUBCLASS_HEAPCHECK		4
/** @} */

/*!
 * @anchor TSTIOCTLs @name IOCTLs
 * Testing device IOCTLs
 * @{ */
#define RTTST_RTIOC_INTERM_BENCH_RES \
	_IOWR(RTIOC_TYPE_TESTING, 0x00, struct rttst_interm_bench_res)

#define RTTST_RTIOC_TMBENCH_START \
	_IOW(RTIOC_TYPE_TESTING, 0x10, struct rttst_tmbench_config)

#define RTTST_RTIOC_TMBENCH_STOP \
	_IOWR(RTIOC_TYPE_TESTING, 0x11, struct rttst_overall_bench_res)

#define RTTST_RTIOC_SWTEST_SET_TASKS_COUNT \
	_IOW(RTIOC_TYPE_TESTING, 0x30, __u32)

#define RTTST_RTIOC_SWTEST_SET_CPU \
	_IOW(RTIOC_TYPE_TESTING, 0x31, __u32)

#define RTTST_RTIOC_SWTEST_REGISTER_UTASK \
	_IOW(RTIOC_TYPE_TESTING, 0x32, struct rttst_swtest_task)

#define RTTST_RTIOC_SWTEST_CREATE_KTASK \
	_IOWR(RTIOC_TYPE_TESTING, 0x33, struct rttst_swtest_task)

#define RTTST_RTIOC_SWTEST_PEND \
	_IOR(RTIOC_TYPE_TESTING, 0x34, struct rttst_swtest_task)

#define RTTST_RTIOC_SWTEST_SWITCH_TO \
	_IOR(RTIOC_TYPE_TESTING, 0x35, struct rttst_swtest_dir)

#define RTTST_RTIOC_SWTEST_GET_SWITCHES_COUNT \
	_IOR(RTIOC_TYPE_TESTING, 0x36, __u32)

#define RTTST_RTIOC_SWTEST_GET_LAST_ERROR \
	_IOR(RTIOC_TYPE_TESTING, 0x37, struct rttst_swtest_error)

#define RTTST_RTIOC_SWTEST_SET_PAUSE \
	_IOW(RTIOC_TYPE_TESTING, 0x38, __u32)

#define RTTST_RTIOC_RTDM_DEFER_CLOSE \
	_IOW(RTIOC_TYPE_TESTING, 0x40, __u32)

#define RTTST_RTIOC_RTDM_ACTOR_GET_CPU \
	_IOR(RTIOC_TYPE_TESTING, 0x41, __u32)
  
#define RTTST_RTIOC_RTDM_PING_PRIMARY \
	_IOR(RTIOC_TYPE_TESTING, 0x42, __u32)
  
#define RTTST_RTIOC_RTDM_PING_SECONDARY \
	_IOR(RTIOC_TYPE_TESTING, 0x43, __u32)

#define RTTST_RTIOC_HEAP_CHECK \
	_IOR(RTIOC_TYPE_TESTING, 0x44, struct rttst_heap_parms)

#define RTTST_RTIOC_HEAP_STAT_COLLECT \
	_IOR(RTIOC_TYPE_TESTING, 0x45, int)

/** @} */

#endif /* !_RTDM_UAPI_TESTING_H */
