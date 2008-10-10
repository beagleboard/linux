/*
 * twl4030-irq.c - TWL4030/TPS659x0 irq support
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kthread.h>

#include <linux/i2c/twl4030.h>


static inline void activate_irq(int irq)
{
#ifdef CONFIG_ARM
	/* ARM requires an extra step to clear IRQ_NOREQUEST, which it
	 * sets on behalf of every irq_chip.  Also sets IRQ_NOPROBE.
	 */
	set_irq_flags(irq, IRQF_VALID);
#else
	/* same effect on other architectures */
	set_irq_noprobe(irq);
#endif
}

/* PIH register offsets */
#define REG_PIH_ISR_P1			0x01
#define REG_PIH_ISR_P2			0x02
#define REG_PIH_SIR			0x03	/* for testing */

/*----------------------------------------------------------------------*/

/**
 * struct twl4030_mod_iregs - TWL module IMR/ISR regs to mask/clear at init
 * @mod_no: TWL4030 module number (e.g., TWL4030_MODULE_GPIO)
 * @sih_ctrl: address of module SIH_CTRL register
 * @reg_cnt: number of IMR/ISR regs
 * @imrs: pointer to array of TWL module interrupt mask register indices
 * @isrs: pointer to array of TWL module interrupt status register indices
 *
 * Ties together TWL4030 modules and lists of IMR/ISR registers to mask/clear
 * during twl_init_irq().
 */
struct twl4030_mod_iregs {
	const u8 mod_no;
	const u8 sih_ctrl;
	const u8 reg_cnt;
	const u8 *imrs;
	const u8 *isrs;
};

/* TWL4030 INT module interrupt mask registers */
static const u8 __initconst twl4030_int_imr_regs[] = {
	TWL4030_INT_PWR_IMR1,
	TWL4030_INT_PWR_IMR2,
};

/* TWL4030 INT module interrupt status registers */
static const u8 __initconst twl4030_int_isr_regs[] = {
	TWL4030_INT_PWR_ISR1,
	TWL4030_INT_PWR_ISR2,
};

/* TWL4030 INTERRUPTS module interrupt mask registers */
static const u8 __initconst twl4030_interrupts_imr_regs[] = {
	TWL4030_INTERRUPTS_BCIIMR1A,
	TWL4030_INTERRUPTS_BCIIMR1B,
	TWL4030_INTERRUPTS_BCIIMR2A,
	TWL4030_INTERRUPTS_BCIIMR2B,
};

/* TWL4030 INTERRUPTS module interrupt status registers */
static const u8 __initconst twl4030_interrupts_isr_regs[] = {
	TWL4030_INTERRUPTS_BCIISR1A,
	TWL4030_INTERRUPTS_BCIISR1B,
	TWL4030_INTERRUPTS_BCIISR2A,
	TWL4030_INTERRUPTS_BCIISR2B,
};

/* TWL4030 MADC module interrupt mask registers */
static const u8 __initconst twl4030_madc_imr_regs[] = {
	TWL4030_MADC_IMR1,
	TWL4030_MADC_IMR2,
};

/* TWL4030 MADC module interrupt status registers */
static const u8 __initconst twl4030_madc_isr_regs[] = {
	TWL4030_MADC_ISR1,
	TWL4030_MADC_ISR2,
};

/* TWL4030 keypad module interrupt mask registers */
static const u8 __initconst twl4030_keypad_imr_regs[] = {
	TWL4030_KEYPAD_KEYP_IMR1,
	TWL4030_KEYPAD_KEYP_IMR2,
};

/* TWL4030 keypad module interrupt status registers */
static const u8 __initconst twl4030_keypad_isr_regs[] = {
	TWL4030_KEYPAD_KEYP_ISR1,
	TWL4030_KEYPAD_KEYP_ISR2,
};

/* TWL4030 GPIO module interrupt mask registers */
static const u8 __initconst twl4030_gpio_imr_regs[] = {
	REG_GPIO_IMR1A,
	REG_GPIO_IMR1B,
	REG_GPIO_IMR2A,
	REG_GPIO_IMR2B,
	REG_GPIO_IMR3A,
	REG_GPIO_IMR3B,
};

/* TWL4030 GPIO module interrupt status registers */
static const u8 __initconst twl4030_gpio_isr_regs[] = {
	REG_GPIO_ISR1A,
	REG_GPIO_ISR1B,
	REG_GPIO_ISR2A,
	REG_GPIO_ISR2B,
	REG_GPIO_ISR3A,
	REG_GPIO_ISR3B,
};

/* TWL4030 modules that have IMR/ISR registers that must be masked/cleared */
static const struct twl4030_mod_iregs __initconst twl4030_mod_regs[] = {
	{
		.mod_no	  = TWL4030_MODULE_INT,
		.sih_ctrl = TWL4030_INT_PWR_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_int_imr_regs),
		.imrs	  = twl4030_int_imr_regs,
		.isrs	  = twl4030_int_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_INTERRUPTS,
		.sih_ctrl = TWL4030_INTERRUPTS_BCISIHCTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_interrupts_imr_regs),
		.imrs	  = twl4030_interrupts_imr_regs,
		.isrs	  = twl4030_interrupts_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_MADC,
		.sih_ctrl = TWL4030_MADC_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_madc_imr_regs),
		.imrs	  = twl4030_madc_imr_regs,
		.isrs	  = twl4030_madc_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_KEYPAD,
		.sih_ctrl = TWL4030_KEYPAD_KEYP_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_keypad_imr_regs),
		.imrs	  = twl4030_keypad_imr_regs,
		.isrs	  = twl4030_keypad_isr_regs,
	},
	{
		.mod_no	  = TWL4030_MODULE_GPIO,
		.sih_ctrl = REG_GPIO_SIH_CTRL,
		.reg_cnt  = ARRAY_SIZE(twl4030_gpio_imr_regs),
		.imrs	  = twl4030_gpio_imr_regs,
		.isrs	  = twl4030_gpio_isr_regs,
	},
};

/*----------------------------------------------------------------------*/

static unsigned twl4030_irq_base;

static struct completion irq_event;

/*
 * This thread processes interrupts reported by the Primary Interrupt Handler.
 */
static int twl4030_irq_thread(void *data)
{
	long irq = (long)data;
	irq_desc_t *desc = irq_desc + irq;
	static unsigned i2c_errors;
	const static unsigned max_i2c_errors = 100;

	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		int ret;
		int module_irq;
		u8 pih_isr;

		/* Wait for IRQ, then read PIH irq status (also blocking) */
		wait_for_completion_interruptible(&irq_event);

		ret = twl4030_i2c_read_u8(TWL4030_MODULE_PIH, &pih_isr,
					  REG_PIH_ISR_P1);
		if (ret) {
			pr_warning("twl4030: I2C error %d reading PIH ISR\n",
					ret);
			if (++i2c_errors >= max_i2c_errors) {
				printk(KERN_ERR "Maximum I2C error count"
						" exceeded.  Terminating %s.\n",
						__func__);
				break;
			}
			complete(&irq_event);
			continue;
		}

		/* these handlers deal with the relevant SIH irq status */
		local_irq_disable();
		for (module_irq = twl4030_irq_base;
				pih_isr;
				pih_isr >>= 1, module_irq++) {
			if (pih_isr & 0x1) {
				irq_desc_t *d = irq_desc + module_irq;

				/* These can't be masked ... always warn
				 * if we get any surprises.
				 */
				if (d->status & IRQ_DISABLED)
					note_interrupt(module_irq, d,
							IRQ_NONE);
				else
					d->handle_irq(module_irq, d);
			}
		}
		local_irq_enable();

		desc->chip->unmask(irq);
	}

	return 0;
}

/*
 * handle_twl4030_pih() is the desc->handle method for the twl4030 interrupt.
 * This is a chained interrupt, so there is no desc->action method for it.
 * Now we need to query the interrupt controller in the twl4030 to determine
 * which module is generating the interrupt request.  However, we can't do i2c
 * transactions in interrupt context, so we must defer that work to a kernel
 * thread.  All we do here is acknowledge and mask the interrupt and wakeup
 * the kernel thread.
 */
static void handle_twl4030_pih(unsigned int irq, irq_desc_t *desc)
{
	/* Acknowledge, clear *AND* mask the interrupt... */
	desc->chip->ack(irq);
	complete(&irq_event);
}

static struct task_struct *start_twl4030_irq_thread(long irq)
{
	struct task_struct *thread;

	init_completion(&irq_event);
	thread = kthread_run(twl4030_irq_thread, (void *)irq, "twl4030-irq");
	if (!thread)
		pr_err("twl4030: could not create irq %ld thread!\n", irq);

	return thread;
}

/*----------------------------------------------------------------------*/

/**
 * twl4030_i2c_clear_isr - clear TWL4030 SIH ISR regs via read + write
 * @mod_no: TWL4030 module number
 * @reg: register index to clear
 * @cor: value of the <module>_SIH_CTRL.COR bit (1 or 0)
 *
 * Either reads (cor == 1) or writes (cor == 0) to a TWL4030 interrupt
 * status register to ensure that any prior interrupts are cleared.
 * Returns the status from the I2C read operation.
 */
static int __init twl4030_i2c_clear_isr(u8 mod_no, u8 reg, u8 cor)
{
	u8 tmp;

	return (cor) ? twl4030_i2c_read_u8(mod_no, &tmp, reg) :
		twl4030_i2c_write_u8(mod_no, 0xff, reg);
}

/**
 * twl4030_read_cor_bit - are TWL module ISRs cleared by reads or writes?
 * @mod_no: TWL4030 module number
 * @reg: register index to clear
 *
 * Returns 1 if the TWL4030 SIH interrupt status registers (ISRs) for
 * the specified TWL module are cleared by reads, or 0 if cleared by
 * writes.
 */
static int twl4030_read_cor_bit(u8 mod_no, u8 reg)
{
	u8 tmp = 0;

	WARN_ON(twl4030_i2c_read_u8(mod_no, &tmp, reg) < 0);

	tmp &= TWL4030_SIH_CTRL_COR_MASK;
	tmp >>= __ffs(TWL4030_SIH_CTRL_COR_MASK);

	return tmp;
}

/**
 * twl4030_mask_clear_intrs - mask and clear all TWL4030 interrupts
 * @t: pointer to twl4030_mod_iregs array
 * @t_sz: ARRAY_SIZE(t) (starting at 1)
 *
 * Mask all TWL4030 interrupt mask registers (IMRs) and clear all
 * interrupt status registers (ISRs).  No return value, but will WARN if
 * any I2C operations fail.
 */
static void __init twl4030_mask_clear_intrs(const struct twl4030_mod_iregs *t,
					    const u8 t_sz)
{
	int i, j;

	/*
	 * N.B. - further efficiency is possible here.  Eight I2C
	 * operations on BCI and GPIO modules are avoidable if I2C
	 * burst read/write transactions were implemented.  Would
	 * probably save about 1ms of boot time and a small amount of
	 * power.
	 */
	for (i = 0; i < t_sz; i++) {
		const struct twl4030_mod_iregs tmr = t[i];
		int cor;

		/* Are ISRs cleared by reads or writes? */
		cor = twl4030_read_cor_bit(tmr.mod_no, tmr.sih_ctrl);

		for (j = 0; j < tmr.reg_cnt; j++) {

			/* Mask interrupts at the TWL4030 */
			WARN_ON(twl4030_i2c_write_u8(tmr.mod_no, 0xff,
						     tmr.imrs[j]) < 0);

			/* Clear TWL4030 ISRs */
			WARN_ON(twl4030_i2c_clear_isr(tmr.mod_no,
						      tmr.isrs[j], cor) < 0);
		}
	}
}


int twl_init_irq(int irq_num, unsigned irq_base, unsigned irq_end)
{
	static struct irq_chip	twl4030_irq_chip;

	int			i;

	/*
	 * Mask and clear all TWL4030 interrupts since initially we do
	 * not have any TWL4030 module interrupt handlers present
	 */
	twl4030_mask_clear_intrs(twl4030_mod_regs,
				 ARRAY_SIZE(twl4030_mod_regs));

	twl4030_irq_base = irq_base;

	/* install an irq handler for each of the SIH modules;
	 * clone dummy irq_chip since PIH can't *do* anything
	 */
	twl4030_irq_chip = dummy_irq_chip;
	twl4030_irq_chip.name = "twl4030";

	for (i = irq_base; i < irq_end; i++) {
		set_irq_chip_and_handler(i, &twl4030_irq_chip,
				handle_simple_irq);
		activate_irq(i);
	}

	/* install an irq handler to demultiplex the TWL4030 interrupt */
	set_irq_data(irq_num, start_twl4030_irq_thread(irq_num));
	set_irq_chained_handler(irq_num, handle_twl4030_pih);

	return 0;
}

int twl_exit_irq(void)
{
	/* FIXME undo twl_init_irq() */
	if (twl4030_irq_base) {
		pr_err("twl4030: can't yet clean up IRQs?\n");
		return -ENOSYS;
	}
	return 0;
}
