// SPDX-License-Identifier: GPL-2.0

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <asm/sbi.h>

#define MAX_COUNTERS		32

/*
 * Counter Enable register
 */
#define SCOUNTEREN		0x106

/*
 * Counter inhibit register
 */
#define SCOUNTINHIBIT		0x5c8

/*
 * 11: PMDS - rw, Performance monitor disable S-mode counting
 * 10: PMDU - rw, Performance monitor disable U-mode counting
 */
#define SXSTATUS		0x5c0

/*
 * Overflow interrupt enable & status register
 */
#define SCOUNTERINTEN		0x5c4
#define SCOUNTEROF		0x5c5

/*
 * 63: TS - rw, status of trigger
 * 13: PMDM - ro, Performance monitor disable machine mode counting
 * 11: PMDS - rw, Performance monitor disable supervisor mode counting
 * 10: PMDU - rw, Performance monitor disable user mode counting
 * 0-1: TME - Trigger Mode, 2'b00 trigger disabled, 2'b01 triger enable
 */
#define SHPMCR			0x5c9

/*
 * Start/End trigger register
 */
#define SHPMSP			0x5ca
#define SHPMEP			0x5cb

#define SHPMCOUNTER0		0x5e0

struct pmu_hw_events {
	struct perf_event	*events[MAX_COUNTERS];
	DECLARE_BITMAP(used_mask, MAX_COUNTERS);
};

static struct thead_pmu_t {
	struct pmu			pmu;
	struct pmu_hw_events __percpu	*hw_events;
	struct platform_device		*plat_device;
	unsigned long			max_period;
	int				irq;
} thead_pmu;

/*
 * Hardware events
 */
#define EVENT_NONE				0
#define EVENT_L1_ICACHE_ACCESS			1
#define EVENT_L1_ICACHE_MISS			2
#define EVENT_ITLB_MISS				3
#define EVENT_DTLB_MISS				4
#define EVENT_JTLB_MISS				5
#define EVENT_BRANCH_MISS			6
#define EVENT_BRANCH				7
#define EVENT_INDIRECT_BRANCH_MISS		8
#define EVENT_INDIRECT_BRANCH			9
#define EVENT_LSU_SPEC_FAIL			10
#define EVENT_STORE_INSTRUCTION			11
#define EVENT_L1_DCACHE_LOAD_ACCESS		12
#define EVENT_L1_DCACHE_LOAD_MISS		13
#define EVENT_L1_DCACHE_STORE_ACCESS		14
#define EVENT_L1_DCACHE_STORE_MISS		15
#define EVENT_L2_LOAD_ACCESS			16
#define EVENT_L2_LOAD_MISS			17
#define EVENT_L2_STORE_ACCESS			18
#define EVENT_L2_STORE_MISS			19
#define EVENT_RF_LAUNCH_FAIL			20
#define EVENT_RF_REG_LAUNCH_FAIL		21
#define EVENT_RF_INSTRUCTION			22
#define EVENT_LSU_CROSS_4K_STALL		23
#define EVENT_LSU_OTHER_STALL			24
#define EVENT_LSU_SQ_DISCARD			25
#define EVENT_LSU_SQ_DATA_DISCARD		26
#define EVENT_IFU_BRANCH_TARGET_MISPRED		27
#define EVENT_IFU_BRANCH_TARGET_INSTRUCTION	28
#define EVENT_ALU_INSTRUCTION			29
#define EVENT_LDST_INSTRUCTION			30
#define EVENT_VECTOR_SIMD_INSTRUCTION		31
#define EVENT_CSR_INSTRUCTION			32
#define EVENT_SYNC_INSTRUCTION			33
#define EVENT_LDST_UNALIGNED_ACCESS		34
#define EVENT_INTERRUPT_NUMBER			35
#define EVENT_INTERRUPT_OFF_CYCLE		36
#define EVENT_ENVIRONMENT_CALL			37
#define EVENT_LONG_JUMP				38
#define EVENT_STALLED_CYCLES_FRONTEND		39
#define EVENT_STALLED_CYCLES_BACKEND		40
#define EVENT_SYNC_STALL			41
#define EVENT_FLOAT_POINT_INSTRUCTION		42

static const int hw_event_map[] = {
	[PERF_COUNT_HW_CPU_CYCLES]			= -EOPNOTSUPP,
	[PERF_COUNT_HW_INSTRUCTIONS]			= -EOPNOTSUPP,
	[PERF_COUNT_HW_CACHE_REFERENCES]		= EVENT_L1_ICACHE_ACCESS,
	[PERF_COUNT_HW_CACHE_MISSES]			= EVENT_L1_ICACHE_MISS,
	[PERF_COUNT_HW_BRANCH_INSTRUCTIONS]		= EVENT_BRANCH,
	[PERF_COUNT_HW_BRANCH_MISSES]			= EVENT_BRANCH_MISS,
	[PERF_COUNT_HW_BUS_CYCLES]			= -EOPNOTSUPP,
	[PERF_COUNT_HW_STALLED_CYCLES_FRONTEND]		= EVENT_STALLED_CYCLES_FRONTEND,
	[PERF_COUNT_HW_STALLED_CYCLES_BACKEND]		= EVENT_STALLED_CYCLES_BACKEND,
	[PERF_COUNT_HW_REF_CPU_CYCLES]			= -EOPNOTSUPP,
};

#define C(x) PERF_COUNT_HW_CACHE_##x
static const int thead_cache_event_map[PERF_COUNT_HW_CACHE_MAX]
[PERF_COUNT_HW_CACHE_OP_MAX]
[PERF_COUNT_HW_CACHE_RESULT_MAX] = {
	[C(L1D)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_DCACHE_LOAD_ACCESS,
			[C(RESULT_MISS)] = EVENT_L1_DCACHE_LOAD_MISS,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_DCACHE_STORE_ACCESS,
			[C(RESULT_MISS)] = EVENT_L1_DCACHE_STORE_MISS,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(L1I)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_ICACHE_ACCESS,
			[C(RESULT_MISS)] = EVENT_L1_ICACHE_MISS,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(LL)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = EVENT_L2_LOAD_ACCESS,
			[C(RESULT_MISS)] = EVENT_L2_LOAD_MISS,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = EVENT_L2_STORE_ACCESS,
			[C(RESULT_MISS)] = EVENT_L2_STORE_MISS,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(DTLB)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_DCACHE_LOAD_ACCESS,
			[C(RESULT_MISS)] = EVENT_DTLB_MISS,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_DCACHE_STORE_ACCESS,
			[C(RESULT_MISS)] = EVENT_DTLB_MISS,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(ITLB)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = EVENT_L1_ICACHE_ACCESS,
			[C(RESULT_MISS)] = EVENT_ITLB_MISS,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(BPU)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
	[C(NODE)] = {
		[C(OP_READ)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_WRITE)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
		[C(OP_PREFETCH)] = {
			[C(RESULT_ACCESS)] = -EOPNOTSUPP,
			[C(RESULT_MISS)] = -EOPNOTSUPP,
		},
	},
};

#define RW_COUNTER(idx, value, is_write) \
	if (is_write) { \
		csr_write(SHPMCOUNTER0 + idx, value); \
		return 0; \
	} else \
		return csr_read(SHPMCOUNTER0 + idx);

static inline u64 rw_counter(int idx, u64 value, bool is_write)
{
	switch (idx) {
	case 0:
		RW_COUNTER(0, value, is_write);
	case 1:
		return -EINVAL;
	case 2:
		RW_COUNTER(2, value, is_write);
	case 3:
		RW_COUNTER(3, value, is_write);
	case 4:
		RW_COUNTER(4, value, is_write);
	case 5:
		RW_COUNTER(5, value, is_write);
	case 6:
		RW_COUNTER(6, value, is_write);
	case 7:
		RW_COUNTER(7, value, is_write);
	case 8:
		RW_COUNTER(8, value, is_write);
	case 9:
		RW_COUNTER(9, value, is_write);
	case 10:
		RW_COUNTER(10, value, is_write);
	case 11:
		RW_COUNTER(11, value, is_write);
	case 12:
		RW_COUNTER(12, value, is_write);
	case 13:
		RW_COUNTER(13, value, is_write);
	case 14:
		RW_COUNTER(14, value, is_write);
	case 15:
		RW_COUNTER(15, value, is_write);
	case 16:
		RW_COUNTER(16, value, is_write);
	case 17:
		RW_COUNTER(17, value, is_write);
	case 18:
		RW_COUNTER(18, value, is_write);
	case 19:
		RW_COUNTER(19, value, is_write);
	case 20:
		RW_COUNTER(20, value, is_write);
	case 21:
		RW_COUNTER(21, value, is_write);
	case 22:
		RW_COUNTER(22, value, is_write);
	case 23:
		RW_COUNTER(23, value, is_write);
	case 24:
		RW_COUNTER(24, value, is_write);
	case 25:
		RW_COUNTER(25, value, is_write);
	case 26:
		RW_COUNTER(26, value, is_write);
	case 27:
		RW_COUNTER(27, value, is_write);
	case 28:
		RW_COUNTER(28, value, is_write);
	case 29:
		RW_COUNTER(29, value, is_write);
	case 30:
		RW_COUNTER(30, value, is_write);
	case 31:
		RW_COUNTER(31, value, is_write);
	}

	return -EINVAL;
}

static int thead_pmu_event_set_period(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	s64 left = local64_read(&hwc->period_left);
	s64 period = hwc->sample_period;
	int ret = 0;

	if (period < 4096 && period != 0 && (
	    hwc->idx == 0 || hwc->idx == 2 ||
	    hwc->config_base == EVENT_L1_ICACHE_ACCESS ||
	    hwc->config_base == EVENT_L1_DCACHE_LOAD_ACCESS ||
	    hwc->config_base == EVENT_L1_DCACHE_STORE_ACCESS ||
	    hwc->config_base == EVENT_L2_LOAD_ACCESS ||
	    hwc->config_base == EVENT_L2_STORE_ACCESS))
		hwc->sample_period = period = 4096;

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
		left = thead_pmu.max_period;

	local64_set(&hwc->prev_count, (u64)(-left));

	csr_clear(SCOUNTEROF, BIT(hwc->idx));

	rw_counter(hwc->idx, (u64)(-left), true);

	perf_event_update_userpage(event);

	return ret;
}

static void thead_perf_event_update(struct perf_event *event,
				   struct hw_perf_event *hwc)
{
	uint64_t prev_raw_count = local64_read(&hwc->prev_count);
	/*
	 * Sign extend count value to 64bit, otherwise delta calculation
	 * would be incorrect when overflow occurs.
	 */
	uint64_t new_raw_count = rw_counter(hwc->idx, 0, false);
	int64_t delta = new_raw_count - prev_raw_count;

	/*
	 * We aren't afraid of hwc->prev_count changing beneath our feet
	 * because there's no way for us to re-enter this function anytime.
	 */
	local64_set(&hwc->prev_count, new_raw_count);
	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);
}

static void thead_pmu_read(struct perf_event *event)
{
	thead_perf_event_update(event, &event->hw);
}

static int thead_pmu_cache_event(u64 config)
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

	return thead_cache_event_map[cache_type][cache_op][cache_result];
}

static int thead_pmu_event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int event_id;

	hwc->idx		= -1;
	hwc->config_base	= 0;
	hwc->config		= 0;
	hwc->event_base		= 0;

	switch (event->attr.type) {
	case PERF_TYPE_HARDWARE:
		if (event->attr.config >= PERF_COUNT_HW_MAX)
			return -ENOENT;

		event_id = hw_event_map[event->attr.config];

		if (event->attr.config == PERF_COUNT_HW_CPU_CYCLES ||
		    event->attr.config == PERF_COUNT_HW_BUS_CYCLES ||
		    event->attr.config == PERF_COUNT_HW_REF_CPU_CYCLES) {
			hwc->idx = 0;
			hwc->config_base = EVENT_NONE;
			break;
		}

		if (event->attr.config == PERF_COUNT_HW_INSTRUCTIONS) {
			hwc->idx = 2;
			hwc->config_base = EVENT_NONE;
			break;
		}

		break;
	case PERF_TYPE_HW_CACHE:
		event_id = thead_pmu_cache_event(event->attr.config);
		if (event_id == -EOPNOTSUPP)
			return -ENOENT;

		hwc->config_base = event_id;
		break;
	case PERF_TYPE_RAW:
		event_id = event->attr.config;
		if (event_id < 0)
			return -ENOENT;

		hwc->config_base = event_id;
		break;
	default:
		return -ENOENT;
	}

	return 0;
}

static void thead_pmu_enable(struct pmu *pmu)
{
}

static void thead_pmu_disable(struct pmu *pmu)
{
}

static void thead_pmu_start(struct perf_event *event, int flags)
{
	unsigned long flg;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	if (WARN_ON_ONCE(idx == -1))
		return;

	if (flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(hwc->state & PERF_HES_UPTODATE));

	hwc->state = 0;

	thead_pmu_event_set_period(event);

	local_irq_save(flg);

	csr_set(SCOUNTERINTEN, BIT(idx));

	local_irq_restore(flg);
}

static void thead_pmu_stop_event(struct perf_event *event)
{
	unsigned long flg;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	local_irq_save(flg);

	csr_clear(SCOUNTERINTEN, BIT(idx));

	local_irq_restore(flg);
}

static void thead_pmu_stop(struct perf_event *event, int flags)
{
	if (!(event->hw.state & PERF_HES_STOPPED)) {
		thead_pmu_stop_event(event);
		event->hw.state |= PERF_HES_STOPPED;
	}

	if ((flags & PERF_EF_UPDATE) &&
	    !(event->hw.state & PERF_HES_UPTODATE)) {
		thead_perf_event_update(event, &event->hw);
		event->hw.state |= PERF_HES_UPTODATE;
	}
}

static void thead_pmu_del(struct perf_event *event, int flags)
{
	struct pmu_hw_events *hw_events = this_cpu_ptr(thead_pmu.hw_events);
	struct hw_perf_event *hwc = &event->hw;
	unsigned long *used_mask = hw_events->used_mask;

	thead_pmu_stop(event, PERF_EF_UPDATE);

	hw_events->events[hwc->idx] = NULL;

	if (hwc->config_base != EVENT_NONE) {
		clear_bit(hwc->idx, used_mask);
		hwc->idx = -1;
	}

	perf_event_update_userpage(event);
}

/* allocate hardware counter and optionally start counting */
static int thead_pmu_add(struct perf_event *event, int flags)
{
	struct pmu_hw_events *hw_events = this_cpu_ptr(thead_pmu.hw_events);
	struct hw_perf_event *hwc = &event->hw;
	unsigned long *used_mask = hw_events->used_mask;
	int idx;

	if (hwc->config_base != EVENT_NONE) {
		set_bit(0, used_mask);
		set_bit(1, used_mask);
		set_bit(2, used_mask);

		idx = find_first_zero_bit(used_mask, MAX_COUNTERS);
		if (idx == MAX_COUNTERS)
			return -EAGAIN;

		set_bit(idx, used_mask);

		sbi_ecall(0x09000001, 0, 2, idx, hwc->config_base, 0, 0, 0);

		hwc->idx = idx;
	}

	hw_events->events[hwc->idx] = event;

	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		thead_pmu_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);

	return 0;
}

static irqreturn_t thead_pmu_handle_irq(int irq, void *dev_id)
{
	struct perf_sample_data data;
	struct pmu_hw_events *cpuc = this_cpu_ptr(thead_pmu.hw_events);
	struct pt_regs *regs;
	int idx;

	/*
	 * Did an overflow occur?
	 */
	if (!csr_read(SCOUNTEROF))
		return IRQ_NONE;

	csr_write(SCOUNTINHIBIT, UINT_MAX);

	/*
	 * Handle the counter(s) overflow(s)
	 */
	regs = get_irq_regs();

	for (idx = 0; idx < MAX_COUNTERS; ++idx) {
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
		thead_perf_event_update(event, &event->hw);
		perf_sample_data_init(&data, 0, hwc->last_period);
		thead_pmu_event_set_period(event);

		if (perf_event_overflow(event, &data, regs))
			thead_pmu_stop_event(event);
	}

	csr_write(SCOUNTINHIBIT, 0);

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

static int thead_pmu_request_irq(irq_handler_t handler)
{
	int err, irqs;
	struct platform_device *pmu_device = thead_pmu.plat_device;

	if (!pmu_device)
		return -ENODEV;

	irqs = min(pmu_device->num_resources, num_possible_cpus());
	if (irqs < 1) {
		pr_err("no irqs for PMUs defined\n");
		return -ENODEV;
	}

	thead_pmu.irq = platform_get_irq(pmu_device, 0);
	if (thead_pmu.irq < 0)
		return -ENODEV;
	err = request_percpu_irq(thead_pmu.irq, handler, "c9xx-pmu-v1", &thead_pmu);
	if (err) {
		pr_err("unable to request IRQ%d for c9xx PMU counters\n",
		       thead_pmu.irq);
		return err;
	}

	return 0;
}

static void thead_pmu_free_irq(void)
{
	int irq;
	struct platform_device *pmu_device = thead_pmu.plat_device;

	irq = platform_get_irq(pmu_device, 0);
	if (irq >= 0)
		free_percpu_irq(irq, this_cpu_ptr(thead_pmu.hw_events));
}

static int init_hw_perf_events(void)
{
	thead_pmu.hw_events = alloc_percpu_gfp(struct pmu_hw_events,
					      GFP_KERNEL);
	if (!thead_pmu.hw_events) {
		pr_info("failed to allocate per-cpu PMU data.\n");
		return -ENOMEM;
	}

	thead_pmu.pmu = (struct pmu) {
		.pmu_enable	= thead_pmu_enable,
		.pmu_disable	= thead_pmu_disable,
		.event_init	= thead_pmu_event_init,
		.add		= thead_pmu_add,
		.del		= thead_pmu_del,
		.start		= thead_pmu_start,
		.stop		= thead_pmu_stop,
		.read		= thead_pmu_read,
	};

	return 0;
}

static int thead_pmu_starting_cpu(unsigned int cpu)
{
	sbi_ecall(0x09000001, 0, 1, 0, 0, 0, 0, 0);

	enable_percpu_irq(thead_pmu.irq, 0);
	return 0;
}

static int thead_pmu_dying_cpu(unsigned int cpu)
{
	disable_percpu_irq(thead_pmu.irq);
	return 0;
}

static int thead_pmu_device_probe(struct platform_device *pdev,
			  const struct of_device_id *of_table)
{
	int ret;

	ret = init_hw_perf_events();
	if (ret) {
		pr_notice("[perf] failed to probe PMU!\n");
		return ret;
	}
	thead_pmu.max_period = ULONG_MAX;
	thead_pmu.plat_device = pdev;

	ret = thead_pmu_request_irq(thead_pmu_handle_irq);
	if (ret) {
		thead_pmu.pmu.capabilities |= PERF_PMU_CAP_NO_INTERRUPT;
		pr_notice("[perf] PMU request irq fail!\n");
	}

	ret = cpuhp_setup_state(CPUHP_AP_PERF_ONLINE + 1, "perf thead:online",
				thead_pmu_starting_cpu,
				thead_pmu_dying_cpu);
	if (ret) {
		thead_pmu_free_irq();
		free_percpu(thead_pmu.hw_events);
		return ret;
	}

	ret = perf_pmu_register(&thead_pmu.pmu, "thead_xt_pmu", PERF_TYPE_RAW);
	if (ret) {
		thead_pmu_free_irq();
		free_percpu(thead_pmu.hw_events);
	}

	pr_notice("[perf] T-HEAD C900 PMU probed\n");

	return ret;
}

const static struct of_device_id thead_pmu_of_device_ids[] = {
	{.compatible = "thead,c900_pmu"},
	{},
};

static int thead_pmu_dev_probe(struct platform_device *pdev)
{
	return thead_pmu_device_probe(pdev, thead_pmu_of_device_ids);
}

static struct platform_driver thead_pmu_driver = {
	.driver = {
		.name = "thead_c900_pmu",
		.of_match_table = thead_pmu_of_device_ids,
	},
	.probe = thead_pmu_dev_probe,
};

int __init thead_c900_pmu_probe(void)
{
	return platform_driver_register(&thead_pmu_driver);
}
device_initcall(thead_c900_pmu_probe);
