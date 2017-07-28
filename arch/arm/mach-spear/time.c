/*
 * arch/arm/plat-spear/time.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim<shiraz.linux.kernel@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <linux/ipipe.h>
#include <linux/ipipe_tickdev.h>
#include <asm/mach/time.h>
#include "generic.h"

/*
 * We would use TIMER0 and TIMER1 as clockevent and clocksource.
 * Timer0 and Timer1 both belong to same gpt block in cpu subbsystem. Further
 * they share same functional clock. Any change in one's functional clock will
 * also affect other timer.
 */

#define CLKEVT	0	/* gpt0, channel0 as clockevent */
#define CLKSRC	1	/* gpt0, channel1 as clocksource */

/* Register offsets, x is channel number */
#define CR(x)		((x) * 0x80 + 0x80)
#define IR(x)		((x) * 0x80 + 0x84)
#define LOAD(x)		((x) * 0x80 + 0x88)
#define COUNT(x)	((x) * 0x80 + 0x8C)

/* Reg bit definitions */
#define CTRL_INT_ENABLE		0x0100
#define CTRL_ENABLE		0x0020
#define CTRL_ONE_SHOT		0x0010

#define CTRL_PRESCALER1		0x0
#define CTRL_PRESCALER2		0x1
#define CTRL_PRESCALER4		0x2
#define CTRL_PRESCALER8		0x3
#define CTRL_PRESCALER16	0x4
#define CTRL_PRESCALER32	0x5
#define CTRL_PRESCALER64	0x6
#define CTRL_PRESCALER128	0x7
#define CTRL_PRESCALER256	0x8

#define INT_STATUS		0x1

/*
 * Minimum clocksource/clockevent timer range in seconds
 */
#define SPEAR_MIN_RANGE 4

static __iomem void *gpt_base;
static struct clk *gpt_clk;
static unsigned long gpt_phys_base;

static void spear_timer_ack(void);
static int clockevent_next_event(unsigned long evt,
				 struct clock_event_device *clk_event_dev);

#ifdef CONFIG_IPIPE
static unsigned prescale, max_delta_ticks;

static struct __ipipe_tscinfo __maybe_unused tsc_info = {
	.type = IPIPE_TSC_TYPE_FREERUNNING_TWICE,
	.u = {
		{
			.mask = 0x0000ffff,
		},
	},
};
#endif /* CONFIG_IPIPE */

static void __init spear_clocksource_init(void)
{
	u32 tick_rate;
	u16 val;

	tick_rate = clk_get_rate(gpt_clk);
#ifndef CONFIG_IPIPE
	/* program the prescaler (/256)*/
	writew(CTRL_PRESCALER256, gpt_base + CR(CLKSRC));

	/* find out actual clock driving Timer */
	tick_rate >>= CTRL_PRESCALER256;
#else /* CONFIG_IPIPE */
	writew(prescale, gpt_base + CR(CLKSRC));
	tick_rate >>= prescale;
#endif /* CONFIG_IPIPE */

	writew(0xFFFF, gpt_base + LOAD(CLKSRC));

	val = readw(gpt_base + CR(CLKSRC));
	val &= ~CTRL_ONE_SHOT;	/* autoreload mode */
	val |= CTRL_ENABLE ;
	writew(val, gpt_base + CR(CLKSRC));

	/* register the clocksource */
	clocksource_mmio_init(gpt_base + COUNT(CLKSRC), "tmr1", tick_rate,
		200, 16, clocksource_mmio_readw_up);

#ifdef CONFIG_IPIPE
	tsc_info.u.counter_paddr = gpt_phys_base + COUNT(CLKSRC),
	tsc_info.counter_vaddr = (unsigned long)(gpt_base + COUNT(CLKSRC));
	tsc_info.freq = tick_rate;
	__ipipe_tsc_register(&tsc_info);
#endif /* CONFIG_IPIPE */
}

static inline void timer_shutdown(struct clock_event_device *evt)
{
	u16 val = readw(gpt_base + CR(CLKEVT));

	/* stop the timer */
	val &= ~CTRL_ENABLE;
	writew(val, gpt_base + CR(CLKEVT));
}

static int spear_shutdown(struct clock_event_device *evt)
{
	timer_shutdown(evt);

	return 0;
}

static int spear_set_oneshot(struct clock_event_device *evt)
{
	u16 val;

	/* stop the timer */
	timer_shutdown(evt);

	val = readw(gpt_base + CR(CLKEVT));
	val |= CTRL_ONE_SHOT;
	writew(val, gpt_base + CR(CLKEVT));

	return 0;
}

static int spear_set_periodic(struct clock_event_device *evt)
{
	u32 period;
	u16 val;

	/* stop the timer */
	timer_shutdown(evt);

	period = clk_get_rate(gpt_clk) / HZ;
#ifndef CONFIG_IPIPE
	period >>= CTRL_PRESCALER16;
#else /* !CONFIG_IPIPE */
	period >>= prescale;
#endif /* !CONFIG_IPIPE */
	writew(period, gpt_base + LOAD(CLKEVT));

	val = readw(gpt_base + CR(CLKEVT));
	val &= ~CTRL_ONE_SHOT;
	val |= CTRL_ENABLE | CTRL_INT_ENABLE;
	writew(val, gpt_base + CR(CLKEVT));

	return 0;
}

#ifdef CONFIG_IPIPE
static struct ipipe_timer spear_itimer = {
	.ack = spear_timer_ack,
};
#endif /* CONFIG_IPIPE */

static struct clock_event_device clkevt = {
	.name = "tmr0",
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown = spear_shutdown,
	.set_state_periodic = spear_set_periodic,
	.set_state_oneshot = spear_set_oneshot,
	.tick_resume = spear_shutdown,
	.set_next_event = clockevent_next_event,
	.shift = 0,	/* to be computed */
#ifdef CONFIG_IPIPE
	.ipipe_timer = &spear_itimer,
#endif /* CONFIG_IPIPE */
};

static int clockevent_next_event(unsigned long cycles,
				 struct clock_event_device *clk_event_dev)
{
	u16 val = readw(gpt_base + CR(CLKEVT));

#ifdef CONFIG_IPIPE
	if (cycles > max_delta_ticks)
		cycles = max_delta_ticks;
#endif

	if (val & CTRL_ENABLE)
		writew(val & ~CTRL_ENABLE, gpt_base + CR(CLKEVT));

	writew(cycles, gpt_base + LOAD(CLKEVT));

	val |= CTRL_ENABLE | CTRL_INT_ENABLE;
	writew(val, gpt_base + CR(CLKEVT));

	return 0;
}

static void spear_timer_ack(void)
{
	writew(INT_STATUS, gpt_base + IR(CLKEVT));
}

static irqreturn_t spear_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clkevt;

	if (!clockevent_ipipe_stolen(evt))
		spear_timer_ack();

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction spear_timer_irq = {
	.name = "timer",
	.flags = IRQF_TIMER,
	.handler = spear_timer_interrupt
};

static void __init spear_clockevent_init(int irq)
{
	u32 tick_rate;

	tick_rate = clk_get_rate(gpt_clk);
#ifndef CONFIG_IPIPE
	/* program the prescaler (/16)*/
	writew(CTRL_PRESCALER16, gpt_base + CR(CLKEVT));

	/* find out actual clock driving Timer */
	tick_rate >>= CTRL_PRESCALER16;
#else /* CONFIG_IPIPE */
	/* Find the prescaler giving a precision under 1us */
	for (prescale = CTRL_PRESCALER256; prescale != 0xffff; prescale--)
		if ((tick_rate >> prescale) >= 1000000)
			break;

	spear_itimer.irq = irq;

	writew(prescale, gpt_base + CR(CLKEVT));
	tick_rate >>= prescale;

	max_delta_ticks = 0xffff - tick_rate / 1000;
#endif /* CONFIG_IPIPE */

	clkevt.cpumask = cpumask_of(0);

	clockevents_config_and_register(&clkevt, tick_rate, 3, 0xfff0);

	setup_irq(irq, &spear_timer_irq);
}

static const struct of_device_id const timer_of_match[] __initconst = {
	{ .compatible = "st,spear-timer", },
	{ },
};

void __init spear_setup_of_timer(void)
{
	struct device_node *np;
	struct resource res;
	int irq, ret;

	np = of_find_matching_node(NULL, timer_of_match);
	if (!np) {
		pr_err("%s: No timer passed via DT\n", __func__);
		return;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%s: No irq passed for timer via DT\n", __func__);
		return;
	}

	gpt_base = of_iomap(np, 0);
	if (!gpt_base) {
		pr_err("%s: of iomap failed\n", __func__);
		return;
	}

	if (of_address_to_resource(np, 0, &res))
		res.start = 0;
	gpt_phys_base = res.start;

	gpt_clk = clk_get_sys("gpt0", NULL);
	if (!gpt_clk) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		goto err_iomap;
	}

	ret = clk_prepare_enable(gpt_clk);
	if (ret < 0) {
		pr_err("%s:couldn't prepare-enable gpt clock\n", __func__);
		goto err_prepare_enable_clk;
	}

	spear_clockevent_init(irq);
	spear_clocksource_init();

	return;

err_prepare_enable_clk:
	clk_put(gpt_clk);
err_iomap:
	iounmap(gpt_base);
}
