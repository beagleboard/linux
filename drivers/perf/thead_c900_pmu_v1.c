// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018 Hangzhou C-SKY Microsystems co.,ltd. */

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <asm/sbi.h>

#define RISCV_MAX_COUNTERS	32
#define RISCV_OP_UNSUPP		(-EOPNOTSUPP)

#define RISCV_PMU_CYCLE		0
#define RISCV_PMU_TIME		1
#define RISCV_PMU_INSTRET	2
#define RISCV_PMU_L1ICAC	3	/* ICache Access */
#define RISCV_PMU_L1ICMC	4	/* ICache Miss */
#define RISCV_PMU_IUTLBMC	5	/* I-UTLB Miss */
#define RISCV_PMU_DUTLBMC	6	/* D-UTLB Miss */
#define RISCV_PMU_JTLBMC	7	/* JTLB Miss Counter */

#define RISCV_PMU_CBMC		8	/* Cond-br-mispredict */
#define RISCV_PMU_CBIC		9	/* Cond-br-instruction */
#define RISCV_PMU_IBMC		10	/* Indirect Branch Mispredict */
#define RISCV_PMU_IBIC		11	/* Indirect Branch Instruction */
#define RISCV_PMU_LSUSFC	12	/* LSU Spec Fail */
#define RISCV_PMU_STC		13	/* Store Instruction */

#define RISCV_PMU_L1DCRAC	14	/* L1 DCache Read Access */
#define RISCV_PMU_L1DCRMC	15	/* L1 DCache Read Miss */
#define RISCV_PMU_L1DCWAC	16	/* L1 DCache Write Access */
#define RISCV_PMU_L1DCWMC	17	/* L1 DCache Write Miss */

#define RISCV_PMU_L2CRAC	18	/* L2 Cache Read Access */
#define RISCV_PMU_L2CRMC	19	/* L2 Cache Read Miss */
#define RISCV_PMU_L2CWAC	20	/* L2 Cache Write Access */
#define RISCV_PMU_L2CWMC	21	/* L2 Cache Write Miss */

#define RISCV_PMU_RFLFC		22	/* RF Launch Fail */
#define RISCV_PMU_RFRLFC	23	/* RF Reg Launch Fail */
#define RISCV_PMU_RFIC		24	/* RF Instruction */

#define RISCV_PMU_LSUC4SC	25	/* LSU Cross 4K Stall */
#define RISCV_PMU_LSUOSC	26	/* LSU Other Stall */
#define RISCV_PMU_LSUSQDC	27	/* LSU SQ Discard */
#define RISCV_PMU_LSUSQDDC	28	/* LSU SQ Data Discard */

#define SCOUNTERINTEN		0x5c4
#define SCOUNTEROF		0x5c5
#define SCOUNTERBASE		0x5e0

#define WRITE_COUNTER(idx, value) \
		csr_write(SCOUNTERBASE + idx, value)

/* The events for a given PMU register set. */
struct pmu_hw_events {
	/*
	 * The events that are active on the PMU for the given index.
	 */
	struct perf_event *events[RISCV_MAX_COUNTERS];

	/*
	 * A 1 bit for an index indicates that the counter is being used for
	 * an event. A 0 means that the counter can be used.
	 */
	unsigned long used_mask[BITS_TO_LONGS(RISCV_MAX_COUNTERS)];
};

static struct riscv_pmu_t {
	struct pmu		      pmu;
	struct pmu_hw_events __percpu   *hw_events;
	struct platform_device	  *plat_device;
	u64			     max_period;
} riscv_pmu;

static int riscv_pmu_irq;

/*
 * Hardware & cache maps and their methods
 */

static const int riscv_hw_event_map[] = {
	[PERF_COUNT_HW_CPU_CYCLES]			= RISCV_PMU_CYCLE,
	[PERF_COUNT_HW_INSTRUCTIONS]			= RISCV_PMU_INSTRET,

	[PERF_COUNT_HW_CACHE_REFERENCES]		= RISCV_PMU_L1ICAC,
	[PERF_COUNT_HW_CACHE_MISSES]			= RISCV_PMU_L1ICMC,

	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS]		= RISCV_PMU_CBIC,
	[PERF_COUNT_HW_BRANCH_MISSES]			= RISCV_PMU_CBMC,

	[PERF_COUNT_HW_BUS_CYCLES]			= RISCV_PMU_IBMC,
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND]		= RISCV_PMU_IBIC,
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND]		= RISCV_PMU_LSUSFC,
	[PERF_COUNT_HW_REF_CPU_CYCLES]			= RISCV_PMU_STC,
};

#define C(x) PERF_COUNT_HW_CACHE_##x
static const int riscv_cache_event_map[PERF_COUNT_HW_CACHE_MAX]
[PERF_COUNT_HW_CACHE_OP_MAX]
[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
	[C(L1D)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_PMU_L1DCRAC,
			[C(RESULT_MISS)] = RISCV_PMU_L1DCRMC,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_PMU_L1DCWAC,
			[C(RESULT_MISS)] = RISCV_PMU_L1DCWMC,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(L1I)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(LL)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_PMU_L2CRAC,
			[C(RESULT_MISS)] = RISCV_PMU_L2CRMC,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_PMU_L2CWAC,
			[C(RESULT_MISS)] = RISCV_PMU_L2CWMC,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(DTLB)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(ITLB)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(BPU)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
	[C(NODE)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = RISCV_OP_UNSUPP,
			[C(RESULT_MISS)] = RISCV_OP_UNSUPP,
		},
	},
};

/*
 * Low-level functions: reading/writing counters
 */
static inline u64 read_counter(int idx)
{
	u64 val = 0;

	switch (idx) {
	case RISCV_PMU_CYCLE:
		val = csr_read(cycle);
		break;
	case RISCV_PMU_INSTRET:
		val = csr_read(instret);
		break;
	case RISCV_PMU_L1ICAC:
		val = csr_read(hpmcounter3);
		break;
	case RISCV_PMU_L1ICMC:
		val = csr_read(hpmcounter4);
		break;
	case RISCV_PMU_IUTLBMC:
		val = csr_read(hpmcounter5);
		break;
	case RISCV_PMU_DUTLBMC:
		val = csr_read(hpmcounter6);
		break;
	case RISCV_PMU_JTLBMC:
		val = csr_read(hpmcounter7);
		break;
	case RISCV_PMU_CBMC:
		val = csr_read(hpmcounter8);
		break;
	case RISCV_PMU_CBIC:
		val = csr_read(hpmcounter9);
		break;
	case RISCV_PMU_IBMC:
		val = csr_read(hpmcounter10);
		break;
	case RISCV_PMU_IBIC:
		val = csr_read(hpmcounter11);
		break;
	case RISCV_PMU_LSUSFC:
		val = csr_read(hpmcounter12);
		break;
	case RISCV_PMU_STC:
		val = csr_read(hpmcounter13);
		break;
	case RISCV_PMU_L1DCRAC:
		val = csr_read(hpmcounter14);
		break;
	case RISCV_PMU_L1DCRMC:
		val = csr_read(hpmcounter15);
		break;
	case RISCV_PMU_L1DCWAC:
		val = csr_read(hpmcounter16);
		break;
	case RISCV_PMU_L1DCWMC:
		val = csr_read(hpmcounter17);
		break;
	case RISCV_PMU_L2CRAC:
		val = csr_read(hpmcounter18);
		break;
	case RISCV_PMU_L2CRMC:
		val = csr_read(hpmcounter19);
		break;
	case RISCV_PMU_L2CWAC:
		val = csr_read(hpmcounter20);
		break;
	case RISCV_PMU_L2CWMC:
		val = csr_read(hpmcounter21);
		break;
	case RISCV_PMU_RFLFC:
		val = csr_read(hpmcounter22);
		break;
	case RISCV_PMU_RFRLFC:
		val = csr_read(hpmcounter23);
		break;
	case RISCV_PMU_RFIC:
		val = csr_read(hpmcounter24);
		break;
	case RISCV_PMU_LSUC4SC:
		val = csr_read(hpmcounter25);
		break;
	case RISCV_PMU_LSUOSC:
		val = csr_read(hpmcounter26);
		break;
	case RISCV_PMU_LSUSQDC:
		val = csr_read(hpmcounter27);
		break;
	case RISCV_PMU_LSUSQDDC:
		val = csr_read(hpmcounter28);
		break;
	default:
		WARN_ON_ONCE(idx < 0 ||	idx > RISCV_MAX_COUNTERS);
		return -EINVAL;
	}

	return val;
}

static inline void write_counter(int idx, u64 value)
{
	switch (idx) {
	case RISCV_PMU_CYCLE:
		WRITE_COUNTER(RISCV_PMU_CYCLE, value);
		break;
	case RISCV_PMU_INSTRET:
		WRITE_COUNTER(RISCV_PMU_INSTRET, value);
		break;
	case RISCV_PMU_L1ICAC:
		WRITE_COUNTER(RISCV_PMU_L1ICAC, value);
		break;
	case RISCV_PMU_L1ICMC:
		WRITE_COUNTER(RISCV_PMU_L1ICMC, value);
		break;
	case RISCV_PMU_IUTLBMC:
		WRITE_COUNTER(RISCV_PMU_IUTLBMC, value);
		break;
	case RISCV_PMU_DUTLBMC:
		WRITE_COUNTER(RISCV_PMU_DUTLBMC, value);
		break;
	case RISCV_PMU_JTLBMC:
		WRITE_COUNTER(RISCV_PMU_JTLBMC, value);
		break;
	case RISCV_PMU_CBMC:
		WRITE_COUNTER(RISCV_PMU_CBMC, value);
		break;
	case RISCV_PMU_CBIC:
		WRITE_COUNTER(RISCV_PMU_CBIC, value);
		break;
	case RISCV_PMU_IBMC:
		WRITE_COUNTER(RISCV_PMU_IBMC, value);
		break;
	case RISCV_PMU_IBIC:
		WRITE_COUNTER(RISCV_PMU_IBIC, value);
		break;
	case RISCV_PMU_LSUSFC:
		WRITE_COUNTER(RISCV_PMU_LSUSFC, value);
		break;
	case RISCV_PMU_STC:
		WRITE_COUNTER(RISCV_PMU_STC, value);
		break;
	case RISCV_PMU_L1DCRAC:
		WRITE_COUNTER(RISCV_PMU_L1DCRAC, value);
		break;
	case RISCV_PMU_L1DCRMC:
		WRITE_COUNTER(RISCV_PMU_L1DCRMC, value);
		break;
	case RISCV_PMU_L1DCWAC:
		WRITE_COUNTER(RISCV_PMU_L1DCWAC, value);
		break;
	case RISCV_PMU_L1DCWMC:
		WRITE_COUNTER(RISCV_PMU_L1DCWMC, value);
		break;
	case RISCV_PMU_L2CRAC:
		WRITE_COUNTER(RISCV_PMU_L2CRAC, value);
		break;
	case RISCV_PMU_L2CRMC:
		WRITE_COUNTER(RISCV_PMU_L2CRMC, value);
		break;
	case RISCV_PMU_L2CWAC:
		WRITE_COUNTER(RISCV_PMU_L2CWAC, value);
		break;
	case RISCV_PMU_L2CWMC:
		WRITE_COUNTER(RISCV_PMU_L2CWMC, value);
		break;
	case RISCV_PMU_RFLFC:
		WRITE_COUNTER(RISCV_PMU_RFLFC, value);
		break;
	case RISCV_PMU_RFRLFC:
		WRITE_COUNTER(RISCV_PMU_RFRLFC, value);
		break;
	case RISCV_PMU_RFIC:
		WRITE_COUNTER(RISCV_PMU_RFIC, value);
		break;
	case RISCV_PMU_LSUC4SC:
		WRITE_COUNTER(RISCV_PMU_LSUC4SC, value);
		break;
	case RISCV_PMU_LSUOSC:
		WRITE_COUNTER(RISCV_PMU_LSUOSC, value);
		break;
	case RISCV_PMU_LSUSQDC:
		WRITE_COUNTER(RISCV_PMU_LSUSQDC, value);
		break;
	case RISCV_PMU_LSUSQDDC:
		WRITE_COUNTER(RISCV_PMU_LSUSQDDC, value);
		break;
	default:
		WARN_ON_ONCE(idx < 0 ||	idx > RISCV_MAX_COUNTERS);
	}
}

static int riscv_pmu_event_is_frequent(int idx)
{
	return  idx >= RISCV_PMU_CYCLE &&
		idx <= RISCV_PMU_L1DCWMC;
}

static int riscv_pmu_event_set_period(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	s64 left = local64_read(&hwc->period_left);
	s64 period = hwc->sample_period;
	int ret = 0;

	if (period < 4096 && period != 0 &&
	    riscv_pmu_event_is_frequent(hwc->idx)) {
		hwc->sample_period = period = 4096;
	}

	if (unlikely(left <= -period)) {
		left = period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	if (unlikely(left <= 0)) {
		left += period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	if (left < 0)
		left = riscv_pmu.max_period;

	/*
	 * The hw event starts counting from this event offset,
	 * mark it to be able to extract future "deltas":
	 */
	local64_set(&hwc->prev_count, (u64)(-left));
	csr_write(SCOUNTEROF, csr_read(SCOUNTEROF) & ~BIT(hwc->idx));
	write_counter(hwc->idx, (u64)(-left));

	perf_event_update_userpage(event);

	return ret;
}

static void riscv_perf_event_update(struct perf_event *event,
				   struct hw_perf_event *hwc)
{
	uint64_t prev_raw_count = local64_read(&hwc->prev_count);
	/*
	 * Sign extend count value to 64bit, otherwise delta calculation
	 * would be incorrect when overflow occurs.
	 */
	uint64_t new_raw_count = read_counter(hwc->idx);
	int64_t delta = new_raw_count - prev_raw_count;

	/*
	 * We aren't afraid of hwc->prev_count changing beneath our feet
	 * because there's no way for us to re-enter this function anytime.
	 */
	local64_set(&hwc->prev_count, new_raw_count);
	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);
}

static void riscv_pmu_read(struct perf_event *event)
{
	riscv_perf_event_update(event, &event->hw);
}

static int riscv_pmu_cache_event(u64 config)
{
	unsigned int cache_type, cache_op, cache_result;

	cache_type      = (config >>  0) & 0xff;
	cache_op	= (config >>  8) & 0xff;
	cache_result    = (config >> 16) & 0xff;

	if (cache_type >= PERF_COUNT_HW_CACHE_MAX)
		return -EINVAL;
	if (cache_op >= PERF_COUNT_HW_CACHE_OP_MAX)
		return -EINVAL;
	if (cache_result >= PERF_COUNT_HW_CACHE_RESULT_MAX)
		return -EINVAL;

	return riscv_cache_event_map[cache_type][cache_op][cache_result];
}

static int riscv_pmu_event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int ret;

	switch (event->attr.type) {
	case PERF_TYPE_HARDWARE:
		if (event->attr.config >= PERF_COUNT_HW_MAX)
			return -ENOENT;
		ret = riscv_hw_event_map[event->attr.config];
		if (ret == RISCV_OP_UNSUPP)
			return -ENOENT;
		hwc->idx = ret;
		break;
	case PERF_TYPE_HW_CACHE:
		ret = riscv_pmu_cache_event(event->attr.config);
		if (ret == RISCV_OP_UNSUPP)
			return -ENOENT;
		hwc->idx = ret;
		break;
	case PERF_TYPE_RAW:
		if (event->attr.config < 0 || event->attr.config >
		    RISCV_MAX_COUNTERS)
			return -ENOENT;
		hwc->idx = event->attr.config;
		break;
	default:
		return -ENOENT;
	}

	return 0;
}

static void riscv_pmu_enable(struct pmu *pmu)
{
}

/* stops all counters */
static void riscv_pmu_disable(struct pmu *pmu)
{
}

static void riscv_pmu_start(struct perf_event *event, int flags)
{
	unsigned long flg;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	if (WARN_ON_ONCE(idx == -1))
		return;

	if (flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(hwc->state & PERF_HES_UPTODATE));

	hwc->state = 0;

	riscv_pmu_event_set_period(event);

	local_irq_save(flg);

	csr_write(SCOUNTERINTEN, BIT(idx) | csr_read(SCOUNTERINTEN));

	local_irq_restore(flg);
}

static void riscv_pmu_stop_event(struct perf_event *event)
{
	unsigned long flg;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	local_irq_save(flg);

	csr_write(SCOUNTERINTEN, ~BIT(idx) & csr_read(SCOUNTERINTEN));

	local_irq_restore(flg);
}

static void riscv_pmu_stop(struct perf_event *event, int flags)
{
	if (!(event->hw.state & PERF_HES_STOPPED)) {
		riscv_pmu_stop_event(event);
		event->hw.state |= PERF_HES_STOPPED;
	}

	if ((flags & PERF_EF_UPDATE) &&
	    !(event->hw.state & PERF_HES_UPTODATE)) {
		riscv_perf_event_update(event, &event->hw);
		event->hw.state |= PERF_HES_UPTODATE;
	}
}

static void riscv_pmu_del(struct perf_event *event, int flags)
{
	struct pmu_hw_events *hw_events = this_cpu_ptr(riscv_pmu.hw_events);
	struct hw_perf_event *hwc = &event->hw;

	riscv_pmu_stop(event, PERF_EF_UPDATE);

	hw_events->events[hwc->idx] = NULL;

	perf_event_update_userpage(event);
}

/* allocate hardware counter and optionally start counting */
static int riscv_pmu_add(struct perf_event *event, int flags)
{
	struct pmu_hw_events *hw_events = this_cpu_ptr(riscv_pmu.hw_events);
	struct hw_perf_event *hwc = &event->hw;

	hw_events->events[hwc->idx] = event;

	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		riscv_pmu_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);

	return 0;
}

static irqreturn_t riscv_pmu_handle_irq(int irq, void *dev_id)
{
	struct perf_sample_data data;
	struct pmu_hw_events *cpuc = this_cpu_ptr(riscv_pmu.hw_events);
	struct pt_regs *regs;
	int idx;

	/*
	 * Did an overflow occur?
	 */
	if (!csr_read(SCOUNTEROF))
		return IRQ_NONE;

	/*
	 * Handle the counter(s) overflow(s)
	 */
	regs = get_irq_regs();


	for (idx = 0; idx < RISCV_MAX_COUNTERS; ++idx) {
		struct perf_event *event = cpuc->events[idx];
		struct hw_perf_event *hwc;

		/* Ignore if we don't have an event. */
		if (!event)
			continue;
		/*
		 * We have a single interrupt for all counters. Check that
		 * each counter has overflowed before we process it.
		 */
		if (!(csr_read(SCOUNTEROF) & BIT(idx)))
			continue;

		hwc = &event->hw;
		riscv_perf_event_update(event, &event->hw);
		perf_sample_data_init(&data, 0, hwc->last_period);
		riscv_pmu_event_set_period(event);

		if (perf_event_overflow(event, &data, regs))
			riscv_pmu_stop_event(event);
	}

	/*
	 * Handle the pending perf events.
	 *
	 * Note: this call *must* be run with interrupts disabled. For
	 * platforms that can have the PMU interrupts raised as an NMI, this
	 * will not work.
	 */
	irq_work_run();

	return IRQ_HANDLED;
}

static int riscv_pmu_request_irq(irq_handler_t handler)
{
	int err, irqs;
	struct platform_device *pmu_device = riscv_pmu.plat_device;

	if (!pmu_device)
		return -ENODEV;

	irqs = min(pmu_device->num_resources, num_possible_cpus());
	if (irqs < 1) {
		pr_err("no irqs for PMUs defined\n");
		return -ENODEV;
	}

	riscv_pmu_irq = platform_get_irq(pmu_device, 0);
	if (riscv_pmu_irq < 0)
		return -ENODEV;
	err = request_percpu_irq(riscv_pmu_irq, handler, "c9xx-pmu-v1", &riscv_pmu);
	if (err) {
		pr_err("unable to request IRQ%d for c9xx PMU counters\n",
		       riscv_pmu_irq);
		return err;
	}

	return 0;
}

static void riscv_pmu_free_irq(void)
{
	int irq;
	struct platform_device *pmu_device = riscv_pmu.plat_device;

	irq = platform_get_irq(pmu_device, 0);
	if (irq >= 0)
		free_percpu_irq(irq, this_cpu_ptr(riscv_pmu.hw_events));
}

static int init_hw_perf_events(void)
{
	riscv_pmu.hw_events = alloc_percpu_gfp(struct pmu_hw_events,
					      GFP_KERNEL);
	if (!riscv_pmu.hw_events) {
		pr_info("failed to allocate per-cpu PMU data.\n");
		return -ENOMEM;
	}

	riscv_pmu.pmu = (struct pmu) {
		.pmu_enable     = riscv_pmu_enable,
		.pmu_disable    = riscv_pmu_disable,
		.event_init     = riscv_pmu_event_init,
		.add	    = riscv_pmu_add,
		.del	    = riscv_pmu_del,
		.start	  = riscv_pmu_start,
		.stop	   = riscv_pmu_stop,
		.read	   = riscv_pmu_read,
	};

	return 0;
}

static int riscv_pmu_starting_cpu(unsigned int cpu)
{
	sbi_ecall(0x09000001, 0, 1, 0, 0, 0, 0, 0);

	enable_percpu_irq(riscv_pmu_irq, 0);
	return 0;
}

static int riscv_pmu_dying_cpu(unsigned int cpu)
{
	disable_percpu_irq(riscv_pmu_irq);
	return 0;
}

static int riscv_pmu_device_probe(struct platform_device *pdev,
			  const struct of_device_id *of_table)
{
	int ret;

	ret = init_hw_perf_events();
	if (ret) {
		pr_notice("[perf] failed to probe PMU!\n");
		return ret;
	}
	riscv_pmu.max_period = ULONG_MAX;
	riscv_pmu.plat_device = pdev;

	ret = riscv_pmu_request_irq(riscv_pmu_handle_irq);
	if (ret) {
		riscv_pmu.pmu.capabilities |= PERF_PMU_CAP_NO_INTERRUPT;
		pr_notice("[perf] PMU request irq fail!\n");
	}

	ret = cpuhp_setup_state(CPUHP_AP_PERF_ONLINE + 1, "perf riscv:online",
				riscv_pmu_starting_cpu,
				riscv_pmu_dying_cpu);
	if (ret) {
		riscv_pmu_free_irq();
		free_percpu(riscv_pmu.hw_events);
		return ret;
	}

	ret = perf_pmu_register(&riscv_pmu.pmu, "thead_xt_pmu", PERF_TYPE_RAW);
	if (ret) {
		riscv_pmu_free_irq();
		free_percpu(riscv_pmu.hw_events);
	}

	pr_notice("[perf] T-HEAD C900 PMU v1 probed\n");

	return ret;
}

const static struct of_device_id riscv_pmu_of_device_ids[] = {
	{.compatible = "riscv,thead_xt_pmu"},
	{.compatible = "riscv,c910_pmu"},
	{},
};

static int riscv_pmu_dev_probe(struct platform_device *pdev)
{
	return riscv_pmu_device_probe(pdev, riscv_pmu_of_device_ids);
}

static struct platform_driver riscv_pmu_driver = {
	.driver = {
		   .name = "thead_xt_pmu_v1",
		   .of_match_table = riscv_pmu_of_device_ids,
		   },
	.probe = riscv_pmu_dev_probe,
};

int __init riscv_pmu_v1_probe(void)
{
	return platform_driver_register(&riscv_pmu_driver);
}
device_initcall(riscv_pmu_v1_probe);
