/*
 * twl4030_gpio.c -- access to GPIOs on TWL4030/TPS659x0 chips
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Code re-arranged and cleaned up by:
 *	Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * Initial Code:
 *	Andy Lowe / Nishanth Menon
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

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-gpio.h>

#include <mach/irqs.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>
#include <mach/mux.h>


/* REVISIT when these symbols vanish elsewhere, remove them here too */
/* #undef TWL4030_GPIO_IRQ_BASE */
/* #undef TWL4030_GPIO_IRQ_END */

static struct gpio_chip twl_gpiochip;
static int twl4030_gpio_irq_base;
static int twl4030_gpio_irq_end;

#ifdef MODULE
#define is_module()	true
#else
#define is_module()	false
#endif

/* BitField Definitions */

/* Data banks : 3 banks for 8 gpios each */
#define DATA_BANK_MAX				8
#define GET_GPIO_DATA_BANK(x)			((x)/DATA_BANK_MAX)
#define GET_GPIO_DATA_OFF(x)			((x)%DATA_BANK_MAX)

/* GPIODATADIR Fields each block 0-7 */
#define BIT_GPIODATADIR_GPIOxDIR(x)		(x)
#define MASK_GPIODATADIR_GPIOxDIR(x)		(0x01 << (x))

/* GPIODATAIN Fields each block 0-7 */
#define BIT_GPIODATAIN_GPIOxIN(x)		(x)
#define MASK_GPIODATAIN_GPIOxIN(x)		(0x01 << (x))

/* GPIODATAOUT Fields each block 0-7 */
#define BIT_GPIODATAOUT_GPIOxOUT(x)		(x)
#define MASK_GPIODATAOUT_GPIOxOUT(x)		(0x01 << (x))

/* CLEARGPIODATAOUT Fields */
#define BIT_CLEARGPIODATAOUT_GPIOxOUT(x)	(x)
#define MASK_CLEARGPIODATAOUT_GPIOxOUT(x)	(0x01 << (x))

/* SETGPIODATAOUT Fields */
#define BIT_SETGPIODATAOUT_GPIOxOUT(x)		(x)
#define MASK_SETGPIODATAOUT_GPIOxOUT(x)		(0x01 << (x))

/* GPIO_DEBEN Fields */
#define BIT_GPIO_DEBEN_GPIOxDEB(x)		(x)
#define MASK_GPIO_DEBEN_GPIOxDEB(x)		(0x01 << (x))

/* GPIO_ISR1A Fields */
#define BIT_GPIO_ISR_GPIOxISR(x)		(x)
#define MASK_GPIO_ISR_GPIOxISR(x)		(0x01 << (x))

/* GPIO_IMR1A Fields */
#define BIT_GPIO_IMR1A_GPIOxIMR(x)		(x)
#define MASK_GPIO_IMR1A_GPIOxIMR(x)		(0x01 << (x))

/* GPIO_SIR1 Fields */
#define BIT_GPIO_SIR1_GPIOxSIR(x)		(x)
#define MASK_GPIO_SIR1_GPIO0SIR			(0x01 << (x))


/* Control banks : 5 banks for 4 gpios each */
#define DATA_CTL_MAX			4
#define GET_GPIO_CTL_BANK(x)		((x)/DATA_CTL_MAX)
#define GET_GPIO_CTL_OFF(x)		((x)%DATA_CTL_MAX)
#define GPIO_BANK_MAX			GET_GPIO_CTL_BANK(TWL4030_GPIO_MAX)

/* GPIOPUPDCTRx Fields 5 banks of 4 gpios each */
#define BIT_GPIOPUPDCTR1_GPIOxPD(x)	(2 * (x))
#define MASK_GPIOPUPDCTR1_GPIOxPD(x)	(0x01 << (2 * (x)))
#define BIT_GPIOPUPDCTR1_GPIOxPU(x)	((x) + 1)
#define MASK_GPIOPUPDCTR1_GPIOxPU(x)	(0x01 << (((2 * (x)) + 1)))

/* GPIO_EDR1 Fields */
#define BIT_GPIO_EDR1_GPIOxFALLING(x)	(2 * (x))
#define MASK_GPIO_EDR1_GPIOxFALLING(x)	(0x01 << (2 * (x)))
#define BIT_GPIO_EDR1_GPIOxRISING(x)	((x) + 1)
#define MASK_GPIO_EDR1_GPIOxRISING(x)	(0x01 << (((2 * (x)) + 1)))

/* GPIO_SIH_CTRL Fields */
#define BIT_GPIO_SIH_CTRL_EXCLEN	(0x000)
#define MASK_GPIO_SIH_CTRL_EXCLEN	(0x00000001)
#define BIT_GPIO_SIH_CTRL_PENDDIS	(0x001)
#define MASK_GPIO_SIH_CTRL_PENDDIS	(0x00000002)
#define BIT_GPIO_SIH_CTRL_COR		(0x002)
#define MASK_GPIO_SIH_CTRL_COR		(0x00000004)

/* GPIO_CTRL Fields */
#define BIT_GPIO_CTRL_GPIO0CD1		(0x000)
#define MASK_GPIO_CTRL_GPIO0CD1		(0x00000001)
#define BIT_GPIO_CTRL_GPIO1CD2		(0x001)
#define MASK_GPIO_CTRL_GPIO1CD2		(0x00000002)
#define BIT_GPIO_CTRL_GPIO_ON		(0x002)
#define MASK_GPIO_CTRL_GPIO_ON		(0x00000004)

/* Mask for GPIO registers when aggregated into a 32-bit integer */
#define GPIO_32_MASK			0x0003ffff

/* Data structures */
static DEFINE_MUTEX(gpio_lock);

/* store usage of each GPIO. - each bit represents one GPIO */
static unsigned int gpio_usage_count;

/* shadow the imr register */
static unsigned int gpio_imr_shadow;

/* bitmask of pending requests to unmask gpio interrupts */
static unsigned int gpio_pending_unmask;

/* pointer to gpio unmask thread struct */
static struct task_struct *gpio_unmask_thread;

/*
 * Helper functions to read and write the GPIO ISR and IMR registers as
 * 32-bit integers. Functions return 0 on success, non-zero otherwise.
 * The caller must hold gpio_lock.
 */

static int gpio_read_isr(unsigned int *isr)
{
	int ret;

	*isr = 0;
	ret = twl4030_i2c_read(TWL4030_MODULE_GPIO, (u8 *) isr,
			REG_GPIO_ISR1A, 3);
	le32_to_cpup(isr);
	*isr &= GPIO_32_MASK;

	return ret;
}

static int gpio_write_isr(unsigned int isr)
{
	isr &= GPIO_32_MASK;
	/*
	 * The buffer passed to the twl4030_i2c_write() routine must have an
	 * extra byte at the beginning reserved for its internal use.
	 */
	isr <<= 8;
	isr = cpu_to_le32(isr);
	return twl4030_i2c_write(TWL4030_MODULE_GPIO, (u8 *) &isr,
				REG_GPIO_ISR1A, 3);
}

static int gpio_write_imr(unsigned int imr)
{
	imr &= GPIO_32_MASK;
	/*
	 * The buffer passed to the twl4030_i2c_write() routine must have an
	 * extra byte at the beginning reserved for its internal use.
	 */
	imr <<= 8;
	imr = cpu_to_le32(imr);
	return twl4030_i2c_write(TWL4030_MODULE_GPIO, (u8 *) &imr,
				REG_GPIO_IMR1A, 3);
}

/*
 * These routines are analagous to the irqchip methods, but they are designed
 * to be called from thread context with cpu interrupts enabled and with no
 * locked spinlocks.  We call these routines from our custom IRQ handler
 * instead of the usual irqchip methods.
 */
static void twl4030_gpio_mask_and_ack(unsigned int irq)
{
	int gpio = irq - twl4030_gpio_irq_base;

	mutex_lock(&gpio_lock);
	/* mask */
	gpio_imr_shadow |= (1 << gpio);
	gpio_write_imr(gpio_imr_shadow);
	/* ack */
	gpio_write_isr(1 << gpio);
	mutex_unlock(&gpio_lock);
}

static void twl4030_gpio_unmask(unsigned int irq)
{
	int gpio = irq - twl4030_gpio_irq_base;

	mutex_lock(&gpio_lock);
	gpio_imr_shadow &= ~(1 << gpio);
	gpio_write_imr(gpio_imr_shadow);
	mutex_unlock(&gpio_lock);
}

/*
 * These are the irqchip methods for the TWL4030 GPIO interrupts.
 * Our IRQ handle method doesn't call these, but they will be called by
 * other routines such as setup_irq() and enable_irq().  They are called
 * with cpu interrupts disabled and with a lock on the irq_controller_lock
 * spinlock.  This complicates matters, because accessing the TWL4030 GPIO
 * interrupt controller requires I2C bus transactions that can't be initiated
 * in this context.  Our solution is to defer accessing the interrupt
 * controller to a kernel thread.  We only need to support the unmask method.
 */

static void twl4030_gpio_mask_and_ack_irqchip(unsigned int irq) {}
static void twl4030_gpio_mask_irqchip(unsigned int irq) {}

static void twl4030_gpio_unmask_irqchip(unsigned int irq)
{
	int gpio = irq - twl4030_gpio_irq_base;

	gpio_pending_unmask |= (1 << gpio);
	if (gpio_unmask_thread && gpio_unmask_thread->state != TASK_RUNNING)
		wake_up_process(gpio_unmask_thread);
}

static struct irq_chip twl4030_gpio_irq_chip = {
	.name	= "twl4030",
	.ack	= twl4030_gpio_mask_and_ack_irqchip,
	.mask	= twl4030_gpio_mask_irqchip,
	.unmask	= twl4030_gpio_unmask_irqchip,
};

/*
 * These are the irqchip methods for the TWL4030 PIH GPIO module interrupt.
 * The PIH module doesn't have interrupt masking capability, so these
 * methods are NULL.
 */
static void twl4030_gpio_module_ack(unsigned int irq) {}
static void twl4030_gpio_module_mask(unsigned int irq) {}
static void twl4030_gpio_module_unmask(unsigned int irq) {}
static struct irq_chip twl4030_gpio_module_irq_chip = {
	.ack	= twl4030_gpio_module_ack,
	.mask	= twl4030_gpio_module_mask,
	.unmask	= twl4030_gpio_module_unmask,
};

/*
 * To configure TWL4030 GPIO module registers
 */
static inline int gpio_twl4030_write(u8 address, u8 data)
{
	int ret = 0;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, address);
	return ret;
}

/*
 * To read a TWL4030 GPIO module register
 */
static inline int gpio_twl4030_read(u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, address);
	if (ret >= 0)
		ret = data;
	return ret;
}

/*
 * twl4030 GPIO request function
 */
int twl4030_request_gpio(int gpio)
{
	int ret = 0;

	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	ret = gpio_request(twl_gpiochip.base + gpio, NULL);
	if (ret < 0)
		return ret;

	mutex_lock(&gpio_lock);
	if (gpio_usage_count & (0x1 << gpio)) {
		ret = -EBUSY;
	} else {
		/* First time usage? - switch on GPIO module */
		if (!gpio_usage_count) {
			ret = gpio_twl4030_write(REG_GPIO_CTRL,
					MASK_GPIO_CTRL_GPIO_ON);
			ret = gpio_twl4030_write(REG_GPIO_SIH_CTRL, 0x00);
		}
		if (!ret)
			gpio_usage_count |= (0x1 << gpio);
		else
			gpio_free(twl_gpiochip.base + gpio);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_request_gpio);

/*
 * TWL4030 GPIO free module
 */
int twl4030_free_gpio(int gpio)
{
	int ret = 0;

	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	mutex_lock(&gpio_lock);

	if ((gpio_usage_count & (0x1 << gpio)) == 0) {
		ret = -EPERM;
	} else {
		gpio_usage_count &= ~(0x1 << gpio);
		gpio_free(twl_gpiochip.base + gpio);
	}

	/* Last time usage? - switch off GPIO module */
	if (ret == 0 && !gpio_usage_count)
		ret = gpio_twl4030_write(REG_GPIO_CTRL, 0x0);

	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_free_gpio);

/*
 * Set direction for TWL4030 GPIO
 */
static int twl4030_set_gpio_direction(int gpio, int is_input)
{
	u8 d_bnk = GET_GPIO_DATA_BANK(gpio);
	u8 d_msk = MASK_GPIODATADIR_GPIOxDIR(GET_GPIO_DATA_OFF(gpio));
	u8 reg = 0;
	u8 base = REG_GPIODATADIR1 + d_bnk;
	int ret = 0;

	if (unlikely(!(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (is_input)
			reg = (u8) ((ret) & ~(d_msk));
		else
			reg = (u8) ((ret) | (d_msk));

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}

/*
 * To enable/disable GPIO pin on TWL4030
 */
static int twl4030_set_gpio_dataout(int gpio, int enable)
{
	u8 d_bnk = GET_GPIO_DATA_BANK(gpio);
	u8 d_msk = MASK_GPIODATAOUT_GPIOxOUT(GET_GPIO_DATA_OFF(gpio));
	u8 base = 0;
	int ret = 0;

	if (unlikely(!(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	if (enable)
		base = REG_SETGPIODATAOUT1 + d_bnk;
	else
		base = REG_CLEARGPIODATAOUT1 + d_bnk;

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_write(base, d_msk);
	mutex_unlock(&gpio_lock);
	return ret;
}

/*
 * To get the status of a GPIO pin on TWL4030
 */
int twl4030_get_gpio_datain(int gpio)
{
	u8 d_bnk = GET_GPIO_DATA_BANK(gpio);
	u8 d_off = BIT_GPIODATAIN_GPIOxIN(GET_GPIO_DATA_OFF(gpio));
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	base = REG_GPIODATAIN1 + d_bnk;
	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	mutex_unlock(&gpio_lock);
	if (ret > 0)
		ret = (ret >> d_off) & 0x1;

	return ret;
}
EXPORT_SYMBOL(twl4030_get_gpio_datain);

#if 0
/*
 * Configure PULL type for a GPIO pin on TWL4030
 */
int twl4030_set_gpio_pull(int gpio, int pull_dircn)
{
	u8 c_bnk = GET_GPIO_CTL_BANK(gpio);
	u8 c_off = GET_GPIO_CTL_OFF(gpio);
	u8 c_msk = 0;
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)	||
		!(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	base = REG_GPIOPUPDCTR1 + c_bnk;
	if (pull_dircn == TWL4030_GPIO_PULL_DOWN)
		c_msk = MASK_GPIOPUPDCTR1_GPIOxPD(c_off);
	else if (pull_dircn == TWL4030_GPIO_PULL_UP)
		c_msk = MASK_GPIOPUPDCTR1_GPIOxPU(c_off);

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		/* clear the previous up/down values */
		reg = (u8) (ret);
		reg &= ~(MASK_GPIOPUPDCTR1_GPIOxPU(c_off) |
			MASK_GPIOPUPDCTR1_GPIOxPD(c_off));
		reg |= c_msk;
		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
#endif

/*
 * Configure Edge control for a GPIO pin on TWL4030
 *
 * FIXME this should just be the irq_chip.set_type() method
 */
int twl4030_set_gpio_edge_ctrl(int gpio, int edge)
{
	u8 c_bnk = GET_GPIO_CTL_BANK(gpio);
	u8 c_off = GET_GPIO_CTL_OFF(gpio);
	u8 c_msk = 0;
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	base = REG_GPIO_EDR1 + c_bnk;

	if (edge & TWL4030_GPIO_EDGE_RISING)
		c_msk |= MASK_GPIO_EDR1_GPIOxRISING(c_off);

	if (edge & TWL4030_GPIO_EDGE_FALLING)
		c_msk |= MASK_GPIO_EDR1_GPIOxFALLING(c_off);

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		/* clear the previous rising/falling values */
		reg =
		(u8) (ret &
			~(MASK_GPIO_EDR1_GPIOxFALLING(c_off) |
			MASK_GPIO_EDR1_GPIOxRISING(c_off)));
		reg |= c_msk;
		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_set_gpio_edge_ctrl);

/*
 * Configure debounce timing value for a GPIO pin on TWL4030
 */
int twl4030_set_gpio_debounce(int gpio, int enable)
{
	u8 d_bnk = GET_GPIO_DATA_BANK(gpio);
	u8 d_msk = MASK_GPIO_DEBEN_GPIOxDEB(GET_GPIO_DATA_OFF(gpio));
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & (0x1 << gpio))))
		return -EPERM;

	base = REG_GPIO_DEBEN1 + d_bnk;
	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (enable)
			reg = (u8) ((ret) | (d_msk));
		else
			reg = (u8) ((ret) & ~(d_msk));

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_set_gpio_debounce);

#if 0
/*
 * Configure Card detect for GPIO pin on TWL4030
 */
int twl4030_set_gpio_card_detect(int gpio, int enable)
{
	u8 reg = 0;
	u8 msk = (1 << gpio);
	int ret = 0;

	/* Only GPIO 0 or 1 can be used for CD feature.. */
	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & (0x1 << gpio))
		|| (gpio >= TWL4030_GPIO_MAX_CD))) {
		return -EPERM;
	}

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(REG_GPIO_CTRL);
	if (ret >= 0) {
		if (enable)
			reg = (u8) (ret | msk);
		else
			reg = (u8) (ret & ~msk);

		ret = gpio_twl4030_write(REG_GPIO_CTRL, reg);
	}
	mutex_unlock(&gpio_lock);
	return (ret);
}
#endif

/* MODULE FUNCTIONS */

/*
 * gpio_unmask_thread() runs as a kernel thread.  It is awakened by the unmask
 * method for the GPIO interrupts.  It unmasks all of the GPIO interrupts
 * specified in the gpio_pending_unmask bitmask.  We have to do the unmasking
 * in a kernel thread rather than directly in the unmask method because of the
 * need to access the TWL4030 via the I2C bus.  Note that we don't need to be
 * concerned about race conditions where the request to unmask a GPIO interrupt
 * has already been cancelled before this thread does the unmasking.  If a GPIO
 * interrupt is improperly unmasked, then the IRQ handler for it will mask it
 * when an interrupt occurs.
 */
static int twl4030_gpio_unmask_thread(void *data)
{
	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		int irq;
		unsigned int gpio_unmask;

		local_irq_disable();
		gpio_unmask = gpio_pending_unmask;
		gpio_pending_unmask = 0;
		local_irq_enable();

		for (irq = twl4030_gpio_irq_base; 0 != gpio_unmask;
				gpio_unmask >>= 1, irq++) {
			if (gpio_unmask & 0x1)
				twl4030_gpio_unmask(irq);
		}

		local_irq_disable();
		if (!gpio_pending_unmask)
			set_current_state(TASK_INTERRUPTIBLE);
		local_irq_enable();

		schedule();
	}
	set_current_state(TASK_RUNNING);
	return 0;
}

/*
 * do_twl4030_gpio_irq() is the desc->handle method for each of the twl4030
 * gpio interrupts.  It executes in kernel thread context.
 * On entry, cpu interrupts are enabled.
 */
static void do_twl4030_gpio_irq(unsigned int irq, irq_desc_t *desc)
{
	struct irqaction *action;
	const unsigned int cpu = smp_processor_id();

	desc->status |= IRQ_LEVEL;

	/*
	 * Acknowledge, clear _AND_ disable the interrupt.
	 */
	twl4030_gpio_mask_and_ack(irq);

	if (!desc->depth) {
		kstat_cpu(cpu).irqs[irq]++;

		action = desc->action;
		if (action) {
			int ret;
			int status = 0;
			int retval = 0;
			do {
				/* Call the ISR with cpu interrupts enabled. */
				ret = action->handler(irq, action->dev_id);
				if (ret == IRQ_HANDLED)
					status |= action->flags;
				retval |= ret;
				action = action->next;
			} while (action);

			if (retval != IRQ_HANDLED)
				printk(KERN_ERR "ISR for TWL4030 GPIO"
					" irq %d can't handle interrupt\n",
					irq);

			if (!desc->depth)
				twl4030_gpio_unmask(irq);
		}
	}
}

/*
 * do_twl4030_gpio_module_irq() is the desc->handle method for the twl4030 gpio
 * module interrupt.  It executes in kernel thread context.
 * This is a chained interrupt, so there is no desc->action method for it.
 * We query the gpio module interrupt controller in the twl4030 to determine
 * which gpio lines are generating interrupt requests, and then call the
 * desc->handle method for each gpio that needs service.
 * On entry, cpu interrupts are disabled.
 */
static void do_twl4030_gpio_module_irq(unsigned int irq, irq_desc_t *desc)
{
	const unsigned int cpu = smp_processor_id();

	desc->status |= IRQ_LEVEL;
	/*
	* The desc->handle method would normally call the desc->chip->ack
	* method here, but we won't bother since our ack method is NULL.
	*/
	if (!desc->depth) {
		int gpio_irq;
		unsigned int gpio_isr;

		kstat_cpu(cpu).irqs[irq]++;
		local_irq_enable();

		mutex_lock(&gpio_lock);
		if (gpio_read_isr(&gpio_isr))
			gpio_isr = 0;
		mutex_unlock(&gpio_lock);

		for (gpio_irq = twl4030_gpio_irq_base; 0 != gpio_isr;
			gpio_isr >>= 1, gpio_irq++) {
			if (gpio_isr & 0x1) {
				irq_desc_t *d = irq_desc + gpio_irq;
				d->handle_irq(gpio_irq, d);
			}
		}

		local_irq_disable();
		/*
		 * Here is where we should call the unmask method, but again we
		 * won't bother since it is NULL.
		 */
	}
}

/*----------------------------------------------------------------------*/

static int twl_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return twl4030_set_gpio_direction(offset, 1);
}

static int twl_get(struct gpio_chip *chip, unsigned offset)
{
	int status = twl4030_get_gpio_datain(offset);

	return (status < 0) ? 0 : status;
}

static int twl_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
	return twl4030_set_gpio_direction(offset, 0);
}

static void twl_set(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
}

static struct gpio_chip twl_gpiochip = {
	.label			= "twl4030",
	.owner			= THIS_MODULE,
	.direction_input	= twl_direction_in,
	.get			= twl_get,
	.direction_output	= twl_direction_out,
	.set			= twl_set,
	.can_sleep		= 1,
};

/*----------------------------------------------------------------------*/

static int gpio_twl4030_remove(struct platform_device *pdev);

static int __devinit gpio_twl4030_probe(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	int irq = 0;

	/* All GPIO interrupts are initially masked */
	gpio_pending_unmask = 0;
	gpio_imr_shadow = GPIO_32_MASK;
	ret = gpio_write_imr(gpio_imr_shadow);

	twl4030_gpio_irq_base = pdata->irq_base;
	twl4030_gpio_irq_end = pdata->irq_end;

	if ((twl4030_gpio_irq_end - twl4030_gpio_irq_base) > 0) {
		if (is_module()) {
			dev_err(&pdev->dev,
				"can't dispatch IRQs from modules\n");
			goto no_irqs;
		}
		if (twl4030_gpio_irq_end > NR_IRQS) {
			dev_err(&pdev->dev,
				"last IRQ is too large: %d\n",
				twl4030_gpio_irq_end);
			return -EINVAL;
		}
	} else {
		dev_notice(&pdev->dev,
			"no IRQs being dispatched\n");
		goto no_irqs;
	}

	if (!ret) {
		/*
		 * Create a kernel thread to handle deferred unmasking of gpio
		 * interrupts.
		 */
		gpio_unmask_thread = kthread_create(twl4030_gpio_unmask_thread,
			NULL, "twl4030 gpio");
		if (!gpio_unmask_thread) {
			dev_err(&pdev->dev,
				"could not create twl4030 gpio unmask"
				" thread!\n");
			ret = -ENOMEM;
		}
	}

	if (!ret) {
		/* install an irq handler for each of the gpio interrupts */
		for (irq = twl4030_gpio_irq_base; irq < twl4030_gpio_irq_end;
				irq++) {
			set_irq_chip(irq, &twl4030_gpio_irq_chip);
			set_irq_handler(irq, do_twl4030_gpio_irq);
			set_irq_flags(irq, IRQF_VALID);
		}

		/* gpio module IRQ */
		irq = platform_get_irq(pdev, 0);

		/*
		 * Install an irq handler to demultiplex the gpio module
		 * interrupt.
		 */
		set_irq_chip(irq, &twl4030_gpio_module_irq_chip);
		set_irq_chained_handler(irq, do_twl4030_gpio_module_irq);
		wake_up_process(gpio_unmask_thread);

		dev_info(&pdev->dev, "IRQ %d chains IRQs %d..%d\n", irq,
			twl4030_gpio_irq_base, twl4030_gpio_irq_end - 1);
	}

no_irqs:
	if (!ret) {
		twl_gpiochip.base = pdata->gpio_base;
		twl_gpiochip.ngpio = TWL4030_GPIO_MAX;
		twl_gpiochip.dev = &pdev->dev;

		ret = gpiochip_add(&twl_gpiochip);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"could not register gpiochip, %d\n",
					ret);
			twl_gpiochip.ngpio = 0;
			gpio_twl4030_remove(pdev);
		} else if (pdata->setup) {
			int status;

			status = pdata->setup(&pdev->dev,
					pdata->gpio_base, TWL4030_GPIO_MAX);
			if (status)
				dev_dbg(&pdev->dev, "setup --> %d\n", status);
		}
	}

	return ret;
}

static int __devexit gpio_twl4030_remove(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int status;
	int irq;

	if (pdata->teardown) {
		status = pdata->teardown(&pdev->dev,
				pdata->gpio_base, TWL4030_GPIO_MAX);
		if (status) {
			dev_dbg(&pdev->dev, "teardown --> %d\n", status);
			return status;
		}
	}

	status = gpiochip_remove(&twl_gpiochip);
	if (status < 0)
		return status;

	if (is_module() || (twl4030_gpio_irq_end - twl4030_gpio_irq_base) <= 0)
		return 0;

	/* uninstall the gpio demultiplexing interrupt handler */
	irq = platform_get_irq(pdev, 0);
	set_irq_handler(irq, NULL);
	set_irq_flags(irq, 0);

	/* uninstall the irq handler for each of the gpio interrupts */
	for (irq = twl4030_gpio_irq_base; irq < twl4030_gpio_irq_end; irq++) {
		set_irq_handler(irq, NULL);
		set_irq_flags(irq, 0);
	}

	/* stop the gpio unmask kernel thread */
	if (gpio_unmask_thread) {
		kthread_stop(gpio_unmask_thread);
		gpio_unmask_thread = NULL;
	}

	return 0;
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:twl4030_gpio");

static struct platform_driver gpio_twl4030_driver = {
	.driver.name	= "twl4030_gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= gpio_twl4030_probe,
	.remove		= __devexit_p(gpio_twl4030_remove),
};

static int __init gpio_twl4030_init(void)
{
	return platform_driver_register(&gpio_twl4030_driver);
}
subsys_initcall(gpio_twl4030_init);

static void __exit gpio_twl4030_exit(void)
{
	platform_driver_unregister(&gpio_twl4030_driver);
}
module_exit(gpio_twl4030_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("GPIO interface for TWL4030");
MODULE_LICENSE("GPL");
