/*
 * linux/arch/arm/mach-at91/at91_ipipe.c
 *
 * Copyright (C) 2007,2014 Gilles Chanteperdrix <gch@xenomai.org>
 *
 * Adaptation to AT91SAM926x:
 * Copyright (C) 2007 Gregory CLEMENT, Adeneo
 *
 * Adaptation to AT91SAM9G45:
 * Copyright (C) 2011 Gregory CLEMENT, Free Electrons
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/stringify.h>
#include <linux/ipipe.h>
#include <linux/atmel_tc.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/ipipe_tickdev.h>
#include <linux/platform_device.h>
#include "at91_ipipe.h"

#define AT91_SLOW_CLOCK 32768
#define TCNXCNS(timer,v) ((v) << ((timer)<<1))
#define AT91_TC_REG_MASK (0xffff)

#define at91_tc_read(reg) \
	__raw_readl(at91_tc_base + ATMEL_TC_REG(CONFIG_IPIPE_AT91_TC % 3, reg))

#define at91_tc_write(reg, value) \
	__raw_writel(value, at91_tc_base + ATMEL_TC_REG(CONFIG_IPIPE_AT91_TC % 3, reg))

#define read_CV() at91_tc_read(CV)
#define read_RC() at91_tc_read(RC)
#define write_RC(value) at91_tc_write(RC, value)

static void __iomem *at91_tc_base;
static unsigned max_delta_ticks;

static int at91_tc_set_16(unsigned long evt, void *timer);

/*
 * IRQ handler for the timer.
 */
static void at91_tc_ack(void)
{
	at91_tc_read(SR);
}

static void at91_tc_request(struct ipipe_timer *timer, int steal)
{
	/* Enable CPCS interrupt. */
	at91_tc_write(IER, ATMEL_TC_CPCS);
}

static void at91_tc_release(struct ipipe_timer *timer)
{
	/* Disable all interrupts. */
	at91_tc_write(IDR, ~0ul);
}

static struct ipipe_timer at91_itimer = {
	.request        = at91_tc_request,
	.set            = at91_tc_set_16,
	.ack            = at91_tc_ack,
	.release        = at91_tc_release,

	.name		= "at91_tc" __stringify(CONFIG_IPIPE_AT91_TC),
	.rating		= 250,
};

/*
 * Reprogram the timer
 */
static int at91_tc_set_16(unsigned long evt, void *timer)
{
	unsigned short next_tick;

	if (evt > max_delta_ticks)
		evt = max_delta_ticks;

	__ipipe_tsc_update();

	next_tick = read_CV() + evt;
	write_RC(next_tick);
	if (evt >= AT91_TC_REG_MASK / 2
	    || (short)(next_tick - read_CV()) > 0)
		return 0;

	at91_itimer.min_delay_ticks = evt;
	return -ETIME;
}

static int at91_tc_set_32(unsigned long evt, void *timer)
{
	unsigned long next_tick;

	next_tick = read_CV() + evt;
	write_RC(next_tick);
	if (evt >= 0x7fffffff
	    || (long)(next_tick - read_CV()) > 0)
		return 0;

	at91_itimer.min_delay_ticks = evt;
	return -ETIME;
}

static struct __ipipe_tscinfo tsc_info = {
	.type = IPIPE_TSC_TYPE_FREERUNNING,
	.u = {
		{
			.mask = AT91_TC_REG_MASK,
		},
	},
};

static int __init at91_ipipe_init(void)
{
	unsigned master_freq, divided_freq = 0;
	unsigned divisor, width32, target;
	unsigned long at91_tc_pbase = 0;
	unsigned long long wrap_ns;
	unsigned index, block;
	struct atmel_tc *tc;
	struct resource	*r;
	int tc_timer_clock;
	unsigned short v;
	int ret;

	index = CONFIG_IPIPE_AT91_TC % 3;
	block = CONFIG_IPIPE_AT91_TC / 3;

	tc = atmel_tc_alloc(block);
	if (tc == NULL) {
		printk(KERN_ERR "I-pipe: could not reserve TC block %d\n",
			block);
		return -ENODEV;
	}
	at91_tc_base = tc->regs;
	r = platform_get_resource(tc->pdev, IORESOURCE_MEM, 0);
	at91_tc_pbase = r->start;
	at91_itimer.irq = tc->irq[index];

	ret = clk_prepare_enable(tc->clk[index]);
	if (ret < 0)
		goto err_free_tc;

	master_freq = clk_get_rate(tc->clk[index]);

	width32 = tc->tcb_config && tc->tcb_config->counter_width == 32;
	target = width32 ? 5000000 : 1000000;

	/* Find the first frequency above 1 or 5 MHz */
	for (tc_timer_clock = ARRAY_SIZE(atmel_tc_divisors) - 1;
	     tc_timer_clock >= 0; tc_timer_clock--) {
		divisor = atmel_tc_divisors[tc_timer_clock];
		divided_freq =
			(divisor ? master_freq / divisor : AT91_SLOW_CLOCK);
		if (divided_freq > target)
			break;
	}

	if (divided_freq < target)
		printk(KERN_INFO "AT91 I-pipe warning: could not find a"
			" frequency greater than %dMHz\n", target / 1000000);

	if (width32) {
		at91_itimer.set = at91_tc_set_32;
		tsc_info.u.mask = 0xffffffffU;
		wrap_ns = 0;
	} else {
		wrap_ns = (unsigned long long)(AT91_TC_REG_MASK + 1);
		wrap_ns *= NSEC_PER_SEC;
		do_div(wrap_ns, divided_freq);

		/*
		 * Add a 1ms margin. It means that when an interrupt
		 * occurs, update_tsc must be called within
		 * 1ms. update_tsc is called through set_dec.
		 */
		wrap_ns -= 1000000;
	}

	printk(KERN_INFO "AT91 I-pipe timer: using TC%d, div: %u, "
		"freq: %u.%06u MHz\n",
		CONFIG_IPIPE_AT91_TC, divisor,
	       divided_freq / 1000000, divided_freq % 1000000);

	/* Disable the channel */
	at91_tc_write(CCR, ATMEL_TC_CLKDIS);

	/* Disable all interrupts. */
	at91_tc_write(IDR, ~0ul);

	/* No Sync. */
	at91_tc_write(BCR, 0);

	/* program NO signal on XCN */
	v = __raw_readl(at91_tc_base + ATMEL_TC_BMR);
	v &= ~TCNXCNS(index, 3);
	v |= TCNXCNS(index, 1); /* AT91_TC_TCNXCNS_NONE */
	__raw_writel(v, at91_tc_base + ATMEL_TC_BMR);

	/* Use the clock selected as input clock. */
	at91_tc_write(CMR, tc_timer_clock);

	/* Load the TC register C. */
	write_RC(0xffff);

	/* Enable the channel. */
	at91_tc_write(CCR, ATMEL_TC_CLKEN | ATMEL_TC_SWTRG);

	at91_itimer.freq = divided_freq;
	at91_itimer.min_delay_ticks = ipipe_timer_ns2ticks(&at91_itimer, 2000);
	if (wrap_ns)
		max_delta_ticks = ipipe_timer_ns2ticks(&at91_itimer, wrap_ns);
	ipipe_timer_register(&at91_itimer);

	tsc_info.counter_vaddr =
		(unsigned long)(at91_tc_base + ATMEL_TC_REG(index, CV));
	tsc_info.u.counter_paddr = (at91_tc_pbase + ATMEL_TC_REG(index, CV));
	tsc_info.freq = divided_freq;
	__ipipe_tsc_register(&tsc_info);

	return 0;

err_free_tc:
	atmel_tc_free(tc);
	return ret;
}
subsys_initcall(at91_ipipe_init);
