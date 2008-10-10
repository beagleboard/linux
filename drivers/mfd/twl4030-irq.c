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


/*
 * TWL4030 IRQ handling has two stages in hardware, and thus in software.
 * The Primary Interrupt Handler (PIH) stage exposes status bits saying
 * which Secondary Interrupt Handler (SIH) stage is raising an interrupt.
 * SIH modules are more traditional IRQ components, which support per-IRQ
 * enable/disable and trigger controls; they do most of the work.
 *
 * These chips are designed to support IRQ handling from two different
 * I2C masters.  Each has a dedicated IRQ line, and dedicated IRQ status
 * and mask registers in the PIH and SIH modules.
 *
 * We set up IRQs starting at a platform-specified base, always starting
 * with PIH and the SIH for PWR_INT and then usually adding GPIO:
 *	base + 0  .. base + 7	PIH
 *	base + 8  .. base + 15	SIH for PWR_INT
 *	base + 16 .. base + 33	SIH for GPIO
 */

/* PIH register offsets */
#define REG_PIH_ISR_P1			0x01
#define REG_PIH_ISR_P2			0x02
#define REG_PIH_SIR			0x03	/* for testing */


/* Linux could (eventually) use either IRQ line */
static int irq_line;

struct sih {
	char	name[8];
	u8	module;			/* module id */
	u8	control_offset;		/* for SIH_CTRL */
	bool	set_cor;

	u8	bits;			/* valid in isr/imr */
	u8	bytes_ixr;		/* bytelen of ISR/IMR/SIR */

	u8	edr_offset;
	u8	bytes_edr;		/* bytelen of EDR */

	/* SIR ignored -- set interrupt, for testing only */
	struct irq_data {
		u8	isr_offset;
		u8	imr_offset;
	} mask[2];
	/* + 2 bytes padding */
};

#define SIH_INITIALIZER(modname, nbits) \
	.module		= TWL4030_MODULE_ ## modname, \
	.control_offset = TWL4030_ ## modname ## _SIH_CTRL, \
	.bits		= nbits, \
	.bytes_ixr	= DIV_ROUND_UP(nbits, 8), \
	.edr_offset	= TWL4030_ ## modname ## _EDR, \
	.bytes_edr	= DIV_ROUND_UP((2*(nbits)), 8), \
	.mask = { { \
		.isr_offset	= TWL4030_ ## modname ## _ISR1, \
		.imr_offset	= TWL4030_ ## modname ## _IMR1, \
	}, \
	{ \
		.isr_offset	= TWL4030_ ## modname ## _ISR2, \
		.imr_offset	= TWL4030_ ## modname ## _IMR2, \
	}, },

/* register naming policies are inconsistent ... */
#define TWL4030_INT_PWR_EDR		TWL4030_INT_PWR_EDR1
#define TWL4030_MODULE_KEYPAD_KEYP	TWL4030_MODULE_KEYPAD
#define TWL4030_MODULE_INT_PWR		TWL4030_MODULE_INT


/* Order in this table matches order in PIH_ISR.  That is,
 * BIT(n) in PIH_ISR is sih_modules[n].
 */
static const struct sih sih_modules[6] = {
	[0] = {
		.name		= "gpio",
		.module		= TWL4030_MODULE_GPIO,
		.control_offset	= REG_GPIO_SIH_CTRL,
		.set_cor	= true,
		.bits		= TWL4030_GPIO_MAX,
		.bytes_ixr	= 3,
		/* Note: *all* of these IRQs default to no-trigger */
		.edr_offset	= REG_GPIO_EDR1,
		.bytes_edr	= 5,
		.mask = { {
			.isr_offset	= REG_GPIO_ISR1A,
			.imr_offset	= REG_GPIO_IMR1A,
		}, {
			.isr_offset	= REG_GPIO_ISR1B,
			.imr_offset	= REG_GPIO_IMR1B,
		}, },
	},
	[1] = {
		.name		= "keypad",
		.set_cor	= true,
		SIH_INITIALIZER(KEYPAD_KEYP, 4)
	},
	[2] = {
		.name		= "bci",
		.module		= TWL4030_MODULE_INTERRUPTS,
		.control_offset	= TWL4030_INTERRUPTS_BCISIHCTRL,
		.bits		= 12,
		.bytes_ixr	= 2,
		.edr_offset	= TWL4030_INTERRUPTS_BCIEDR1,
		/* Note: most of these IRQs default to no-trigger */
		.bytes_edr	= 3,
		.mask = { {
			.isr_offset	= TWL4030_INTERRUPTS_BCIISR1A,
			.imr_offset	= TWL4030_INTERRUPTS_BCIIMR1A,
		}, {
			.isr_offset	= TWL4030_INTERRUPTS_BCIISR1B,
			.imr_offset	= TWL4030_INTERRUPTS_BCIIMR1B,
		}, },
	},
	[3] = {
		.name		= "madc",
		SIH_INITIALIZER(MADC, 4)
	},
	[4] = {
		/* USB doesn't use the same SIH organization */
		.name		= "usb",
	},
	[5] = {
		.name		= "power",
		.set_cor	= true,
		SIH_INITIALIZER(INT_PWR, 8)
	},
		/* there are no SIH modules #6 or #7 ... */
};

#undef TWL4030_MODULE_KEYPAD_KEYP
#undef TWL4030_MODULE_INT_PWR
#undef TWL4030_INT_PWR_EDR

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

/*
 * twl4030_init_sih_modules() ... start from a known state where no
 * IRQs will be coming in, and where we can quickly enable them then
 * handle them as they arrive.  Mask all IRQs: maybe init SIH_CTRL.
 *
 * NOTE:  we don't touch EDR registers here; they stay with hardware
 * defaults or whatever the last value was.  Note that when both EDR
 * bits for an IRQ are clear, that's as if its IMR bit is set...
 */
static int twl4030_init_sih_modules(unsigned line)
{
	const struct sih *sih;
	u8 buf[4];
	int i;
	int status;

	/* line 0 == int1_n signal; line 1 == int2_n signal */
	if (line > 1)
		return -EINVAL;

	irq_line = line;

	/* disable all interrupts on our line */
	memset(buf, 0xff, sizeof buf);
	sih = sih_modules;
	for (i = 0; i < ARRAY_SIZE(sih_modules); i++, sih++) {

		/* skip USB -- it's funky */
		if (!sih->bytes_ixr)
			continue;

		status = twl4030_i2c_write(sih->module, buf,
				sih->mask[line].imr_offset, sih->bytes_ixr);
		if (status < 0)
			pr_err("twl4030: err %d initializing %s %s\n",
					status, sih->name, "IMR");

		/* Maybe disable "exclusive" mode; buffer second pending irq;
		 * set Clear-On-Read (COR) bit.
		 *
		 * NOTE that sometimes COR polarity is documented as being
		 * inverted:  for MADC and BCI, COR=1 means "clear on write".
		 * And for PWR_INT it's not documented...
		 */
		if (sih->set_cor) {
			status = twl4030_i2c_write_u8(sih->module,
					TWL4030_SIH_CTRL_COR_MASK,
					sih->control_offset);
			if (status < 0)
				pr_err("twl4030: err %d initializing %s %s\n",
						status, sih->name, "SIH_CTRL");
		}
	}

	sih = sih_modules;
	for (i = 0; i < ARRAY_SIZE(sih_modules); i++, sih++) {
		u8 rxbuf[4];
		int j;

		/* skip USB */
		if (!sih->bytes_ixr)
			continue;

		/* Clear pending interrupt status.  Either the read was
		 * enough, or we need to write those bits.  Repeat, in
		 * case an IRQ is pending (PENDDIS=0) ... that's not
		 * uncommon with PWR_INT.PWRON.
		 */
		for (j = 0; j < 2; j++) {
			status = twl4030_i2c_read(sih->module, rxbuf,
				sih->mask[line].isr_offset, sih->bytes_ixr);
			if (status < 0)
				pr_err("twl4030: err %d initializing %s %s\n",
					status, sih->name, "ISR");

			if (!sih->set_cor)
				status = twl4030_i2c_write(sih->module, buf,
					sih->mask[line].isr_offset,
					sih->bytes_ixr);
			/* else COR=1 means read sufficed.
			 * (for most SIH modules...)
			 */
		}
	}

	return 0;
}

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

/* FIXME pass in which interrupt line we'll use ... */
#define twl_irq_line	0

int twl_init_irq(int irq_num, unsigned irq_base, unsigned irq_end)
{
	static struct irq_chip	twl4030_irq_chip;

	int			status;
	int			i;

	/*
	 * Mask and clear all TWL4030 interrupts since initially we do
	 * not have any TWL4030 module interrupt handlers present
	 */
	status = twl4030_init_sih_modules(twl_irq_line);
	if (status < 0)
		return status;

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

	return status;
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
