/*
 * linux/arch/arm/mach-omap1/time.c
 *
 * OMAP Timers
 *
 * Copyright (C) 2004 Nokia Corporation
 * Partial timer rewrite and additional dynamic tick timer support by
 * Tony Lindgen <tony@atomide.com> and
 * Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *
 * MPU timer code based on the older MPU timer code for OMAP
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: Greg Lonnon <glonnon@ridgerun.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clocksource.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>


#define OMAP_MPU_TIMER_BASE		OMAP_MPU_TIMER1_BASE
#define OMAP_MPU_TIMER_OFFSET		0x100

/* cycles to nsec conversions taken from arch/i386/kernel/timers/timer_tsc.c,
 * converted to use kHz by Kevin Hilman */
/* convert from cycles(64bits) => nanoseconds (64bits)
 *  basic equation:
 *		ns = cycles / (freq / ns_per_sec)
 *		ns = cycles * (ns_per_sec / freq)
 *		ns = cycles * (10^9 / (cpu_khz * 10^3))
 *		ns = cycles * (10^6 / cpu_khz)
 *
 *	Then we use scaling math (suggested by george at mvista.com) to get:
 *		ns = cycles * (10^6 * SC / cpu_khz / SC
 *		ns = cycles * cyc2ns_scale / SC
 *
 *	And since SC is a constant power of two, we can convert the div
 *  into a shift.
 *			-johnstul at us.ibm.com "math is hard, lets go shopping!"
 */
static unsigned long cyc2ns_scale;
#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */

static inline void set_cyc2ns_scale(unsigned long cpu_khz)
{
	cyc2ns_scale = (1000000 << CYC2NS_SCALE_FACTOR)/cpu_khz;
}

static inline unsigned long long cycles_2_ns(unsigned long long cyc)
{
	return (cyc * cyc2ns_scale) >> CYC2NS_SCALE_FACTOR;
}


typedef struct {
	u32 cntl;			/* CNTL_TIMER, R/W */
	u32 load_tim;			/* LOAD_TIM,   W */
	u32 read_tim;			/* READ_TIM,   R */
} omap_mpu_timer_regs_t;

#define omap_mpu_timer_base(n)						\
((volatile omap_mpu_timer_regs_t*)IO_ADDRESS(OMAP_MPU_TIMER_BASE +	\
				 (n)*OMAP_MPU_TIMER_OFFSET))

static inline unsigned long omap_mpu_timer_read(int nr)
{
	volatile omap_mpu_timer_regs_t* timer = omap_mpu_timer_base(nr);
	return timer->read_tim;
}

static inline void omap_mpu_timer_start(int nr, unsigned long load_val)
{
	volatile omap_mpu_timer_regs_t* timer = omap_mpu_timer_base(nr);

	timer->cntl = MPU_TIMER_CLOCK_ENABLE;
	udelay(1);
	timer->load_tim = load_val;
        udelay(1);
	timer->cntl = (MPU_TIMER_CLOCK_ENABLE | MPU_TIMER_AR | MPU_TIMER_ST);
}

/*
 * ---------------------------------------------------------------------------
 * MPU timer 1 ... count down to zero, interrupt, reload
 * ---------------------------------------------------------------------------
 */
static irqreturn_t omap_mpu_timer1_interrupt(int irq, void *dev_id)
{
	write_seqlock(&xtime_lock);
	/* NOTE:  no lost-tick detection/handling! */
	timer_tick();
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static struct irqaction omap_mpu_timer1_irq = {
	.name		= "mpu_timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= omap_mpu_timer1_interrupt,
};

static __init void omap_init_mpu_timer(unsigned long rate)
{
	set_cyc2ns_scale(rate / 1000);

	setup_irq(INT_TIMER1, &omap_mpu_timer1_irq);
	omap_mpu_timer_start(0, (rate / HZ) - 1);
}

/*
 * ---------------------------------------------------------------------------
 * MPU timer 2 ... free running 32-bit clock source and scheduler clock
 * ---------------------------------------------------------------------------
 */

static unsigned long omap_mpu_timer2_overflows;

static irqreturn_t omap_mpu_timer2_interrupt(int irq, void *dev_id)
{
	omap_mpu_timer2_overflows++;
	return IRQ_HANDLED;
}

static struct irqaction omap_mpu_timer2_irq = {
	.name		= "mpu_timer2",
	.flags		= IRQF_DISABLED,
	.handler	= omap_mpu_timer2_interrupt,
};

static cycle_t mpu_read(void)
{
	return ~omap_mpu_timer_read(1);
}

static struct clocksource clocksource_mpu = {
	.name		= "mpu_timer2",
	.rating		= 300,
	.read		= mpu_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 24,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init omap_init_clocksource(unsigned long rate)
{
	static char err[] __initdata = KERN_ERR
			"%s: can't register clocksource!\n";

	clocksource_mpu.mult
		= clocksource_khz2mult(rate/1000, clocksource_mpu.shift);

	setup_irq(INT_TIMER2, &omap_mpu_timer2_irq);
	omap_mpu_timer_start(1, ~0);

	if (clocksource_register(&clocksource_mpu))
		printk(err, clocksource_mpu.name);
}


/*
 * Scheduler clock - returns current time in nanosec units.
 */
unsigned long long sched_clock(void)
{
	unsigned long ticks = 0 - omap_mpu_timer_read(1);
	unsigned long long ticks64;

	ticks64 = omap_mpu_timer2_overflows;
	ticks64 <<= 32;
	ticks64 |= ticks;

	return cycles_2_ns(ticks64);
}

/*
 * ---------------------------------------------------------------------------
 * Timer initialization
 * ---------------------------------------------------------------------------
 */
static void __init omap_timer_init(void)
{
 	struct clk	*ck_ref = clk_get(NULL, "ck_ref");
	unsigned long	rate;

	BUG_ON(IS_ERR(ck_ref));

	rate = clk_get_rate(ck_ref);
	clk_put(ck_ref);

	/* PTV = 0 */
	rate /= 2;

	omap_init_mpu_timer(rate);
	omap_init_clocksource(rate);
}

struct sys_timer omap_timer = {
	.init		= omap_timer_init,
};
