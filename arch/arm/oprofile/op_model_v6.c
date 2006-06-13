/**
 * @file op_model_v6.c
 * ARM11 Performance Monitor Driver
 *
 * Based on op_model_xscale.c
 *
 * @remark Copyright 2000-2004 Deepak Saxena <dsaxena@mvista.com>
 * @remark Copyright 2000-2004 MontaVista Software Inc
 * @remark Copyright 2004 Dave Jiang <dave.jiang@intel.com>
 * @remark Copyright 2004 Intel Corporation
 * @remark Copyright 2004 Zwane Mwaikambo <zwane@arm.linux.org.uk>
 * @remark Copyright 2004 OProfile Authors
 *
 * @remark Read the file COPYING
 *
 * @author Tony Lindgren <tony@atomide.com>
 */

/* #define DEBUG */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/system.h>

#include "op_counter.h"
#include "op_arm_model.h"

#define	PMU_ENABLE	0x001	/* Enable counters */
#define PMN_RESET	0x002	/* Reset event counters */
#define	CCNT_RESET	0x004	/* Reset clock counter */
#define	PMU_RESET	(CCNT_RESET | PMN_RESET)
#define PMU_CNT64	0x008	/* Make CCNT count every 64th cycle */

/*
 * Different types of events that can be counted by the XScale PMU
 * as used by Oprofile userspace. Here primarily for documentation
 * purposes.
 */

#define EVT_ICACHE_MISS			0x00
#define	EVT_ICACHE_NO_DELIVER		0x01
#define	EVT_DATA_STALL			0x02
#define	EVT_ITLB_MISS			0x03
#define	EVT_DTLB_MISS			0x04
#define	EVT_BRANCH			0x05
#define	EVT_BRANCH_MISS			0x06
#define	EVT_INSTRUCTION			0x07
#define	EVT_DCACHE_FULL_STALL		0x08
#define	EVT_DCACHE_FULL_STALL_CONTIG	0x09
#define	EVT_DCACHE_ACCESS		0x0A
#define	EVT_DCACHE_MISS			0x0B
#define	EVT_DCACE_WRITE_BACK		0x0C
#define	EVT_PC_CHANGED			0x0D
#define	EVT_BCU_REQUEST			0x10
#define	EVT_BCU_FULL			0x11
#define	EVT_BCU_DRAIN			0x12
#define	EVT_BCU_ECC_NO_ELOG		0x14
#define	EVT_BCU_1_BIT_ERR		0x15
#define	EVT_RMW				0x16
/* EVT_CCNT is not hardware defined */
#define EVT_CCNT			0xFE
#define EVT_UNUSED			0xFF

struct pmu_counter {
	volatile unsigned long ovf;
	unsigned long reset_counter;
};

enum { CCNT, PMN0, PMN1, PMN2, PMN3, MAX_COUNTERS };

static struct pmu_counter results[MAX_COUNTERS];

enum { PMU_ARM11 };

struct pmu_type {
	int id;
	char *name;
	int num_counters;
	int interrupt;
	unsigned int int_enable;
	unsigned int cnt_ovf[MAX_COUNTERS];
	unsigned int int_mask[MAX_COUNTERS];
};

static struct pmu_type pmu_parms[] = {
	{
		.id		= PMU_ARM11,
		.name		= "arm/arm11",
		.num_counters	= 3,
		.interrupt	= -1,
		.int_mask	= { [PMN0] = 0x10, [PMN1] = 0x20,
				    [CCNT] = 0x40 },
		.cnt_ovf	= { [CCNT] = 0x400, [PMN0] = 0x100,
				    [PMN1] = 0x200},
	},
};

static struct pmu_type *pmu;

static void write_pmnc(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c15, c12, 0" : : "r" (val));
}

static u32 read_pmnc(void)
{
	u32 val;

	__asm__ __volatile__ ("mrc p15, 0, %0, c15, c12, 0" : "=r" (val));

	return val;
}

static u32 read_counter(int counter)
{
	u32 val = 0;

	switch (counter) {
	case CCNT:
		__asm__ __volatile__ ("mrc p15, 0, %0, c15, c12, 1" : "=r" (val));
		break;
	case PMN0:
		__asm__ __volatile__ ("mrc p15, 0, %0, c15, c12, 2" : "=r" (val));
		break;
	case PMN1:
		__asm__ __volatile__ ("mrc p15, 0, %0, c15, c12, 3" : "=r" (val));
		break;
	}

	return val;
}

static void write_counter(int counter, u32 val)
{
	switch (counter) {
	case CCNT:
		__asm__ __volatile__ ("mcr p15, 0, %0, c15, c12, 1" : : "r" (val));
		break;
	case PMN0:
		__asm__ __volatile__ ("mcr p15, 0, %0, c15, c12, 2" : : "r" (val));
		break;
	case PMN1:
		__asm__ __volatile__ ("mcr p15, 0, %0, c15, c12, 3" : : "r" (val));
		break;
	}
}

static int arm11_setup_ctrs(void)
{
	u32 pmnc;
	int i;

	for (i = CCNT; i < MAX_COUNTERS; i++) {
		if (counter_config[i].enabled)
			continue;

		counter_config[i].event = EVT_UNUSED;
	}

	pmnc = (counter_config[PMN1].event << 20) | (counter_config[PMN0].event << 12);
	pr_debug("arm11_setup_ctrs: pmnc: %#08x\n", pmnc);
	write_pmnc(pmnc);

	for (i = CCNT; i < MAX_COUNTERS; i++) {
		if (counter_config[i].event == EVT_UNUSED) {
			counter_config[i].event = 0;
			pmu->int_enable &= ~pmu->int_mask[i];
			continue;
		}

		results[i].reset_counter = counter_config[i].count;
		write_counter(i, -(u32)counter_config[i].count);
		pmu->int_enable |= pmu->int_mask[i];
		pr_debug("arm11_setup_ctrs: counter%d %#08x from %#08lx\n", i,
			read_counter(i), counter_config[i].count);
	}

	return 0;
}

static void inline __arm11_check_ctrs(void)
{
	int i;
	u32 pmnc = read_pmnc();

	/* Write the value back to clear the overflow flags. Overflow */
	/* flags remain in pmnc for use below */
	write_pmnc(pmnc & ~PMU_ENABLE);

	for (i = CCNT; i <= PMN1; i++) {
		if (!(pmu->int_mask[i] & pmu->int_enable))
			continue;

		if (pmnc & pmu->cnt_ovf[i])
			results[i].ovf++;
	}
}

static irqreturn_t arm11_pmu_interrupt(int irq, void *arg, struct pt_regs *regs)
{
	int i;
	u32 pmnc;

	__arm11_check_ctrs();

	for (i = CCNT; i < MAX_COUNTERS; i++) {
		if (!results[i].ovf)
			continue;

		write_counter(i, -(u32)results[i].reset_counter);
		oprofile_add_sample(regs, i);
		results[i].ovf--;
	}

	pmnc = read_pmnc() | PMU_ENABLE;
	write_pmnc(pmnc);

	return IRQ_HANDLED;
}

static void arm11_pmu_stop(void)
{
	u32 pmnc = read_pmnc();

	pmnc &= ~PMU_ENABLE;
	write_pmnc(pmnc);

	if (pmu->interrupt >= 0)
		free_irq(pmu->interrupt, results);
}

static int arm11_pmu_start(void)
{
	int ret;
	u32 pmnc = read_pmnc();

	if (pmu->interrupt >= 0) {
		ret = request_irq(pmu->interrupt, arm11_pmu_interrupt, SA_INTERRUPT,
				  "ARM11 PMU", (void *)results);
		if (ret < 0) {
			printk(KERN_ERR "oprofile: unable to request IRQ%d for ARM11 PMU\n",
			       pmu->interrupt);
			return ret;
		}

		pmnc |= pmu->int_enable;
	}

	printk("XXX enabling PMNC: %08x\n", pmnc);
	pmnc |= PMU_ENABLE;
	write_pmnc(pmnc);
	pr_debug("arm11_pmu_start: pmnc: %#08x mask: %08x\n", pmnc, pmu->int_enable);
	return 0;
}

static int arm11_detect_pmu(void)
{
	pmu = &pmu_parms[PMU_ARM11];

	op_arm11_spec.name = pmu->name;
	op_arm11_spec.num_counters = pmu->num_counters;
	pr_debug("arm11_detect_pmu: detected %s PMU\n", pmu->name);

	return 0;
}

struct op_arm_model_spec op_arm11_spec = {
	.init		= arm11_detect_pmu,
	.setup_ctrs	= arm11_setup_ctrs,
	.start		= arm11_pmu_start,
	.stop		= arm11_pmu_stop,
};

static int __init op_arm11_init(void)
{
	int ret;

	ret = arm11_detect_pmu();
	if (ret == 0)
		ret = arm11_pmu_start();

	return ret;
}
subsys_initcall(op_arm11_init);
