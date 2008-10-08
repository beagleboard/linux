/*
 * twl4030-pwrirq.c - handle power interrupts from TWL3040
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/random.h>
#include <linux/kthread.h>
#include <linux/i2c/twl4030.h>


static DEFINE_SPINLOCK(pwr_lock);
static u8 twl4030_pwrirq_mask;

static struct task_struct *twl4030_pwrirq_unmask_thread;

static void twl4030_pwrirq_ack(unsigned int irq) {}

static void twl4030_pwrirq_disableint(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	twl4030_pwrirq_mask |= 1 << (irq - TWL4030_PWR_IRQ_BASE);
	if (twl4030_pwrirq_unmask_thread
			&& twl4030_pwrirq_unmask_thread->state != TASK_RUNNING)
		wake_up_process(twl4030_pwrirq_unmask_thread);
	spin_unlock_irqrestore(&pwr_lock, flags);
}

static void twl4030_pwrirq_enableint(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);
	twl4030_pwrirq_mask &= ~(1 << (irq - TWL4030_PWR_IRQ_BASE));
	if (twl4030_pwrirq_unmask_thread
			&& twl4030_pwrirq_unmask_thread->state != TASK_RUNNING)
		wake_up_process(twl4030_pwrirq_unmask_thread);
	spin_unlock_irqrestore(&pwr_lock, flags);
}

static struct irq_chip twl4030_pwrirq_chip = {
	.name	= "twl4030-pwr",
	.ack	= twl4030_pwrirq_ack,
	.mask	= twl4030_pwrirq_disableint,
	.unmask	= twl4030_pwrirq_enableint,
};

static void do_twl4030_pwrirq(unsigned int irq, irq_desc_t *desc)
{
	const unsigned int cpu = smp_processor_id();

	desc->status |= IRQ_LEVEL;

	desc->chip->ack(irq);

	if (!desc->depth) {
		int ret;
		int module_irq;
		u8 pwr_isr;

		kstat_cpu(cpu).irqs[irq]++;

		local_irq_enable();
		ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &pwr_isr,
					  TWL4030_INT_PWR_ISR1);
		local_irq_disable();
		if (ret) {
			printk(KERN_WARNING
				"I2C error %d while reading TWL4030"
				"INT PWR_ISR1 register\n", ret);
			return;
		}

		for (module_irq = TWL4030_PWR_IRQ_BASE; pwr_isr != 0;
			module_irq++, pwr_isr >>= 1) {
			if (pwr_isr & 1)
				generic_handle_irq(module_irq);
		}

		desc->chip->unmask(irq);
	}
}

static int twl4030_pwrirq_thread(void *data)
{
	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		u8 local_mask;

		spin_lock_irq(&pwr_lock);
		local_mask = twl4030_pwrirq_mask;
		spin_unlock_irq(&pwr_lock);

		twl4030_i2c_write_u8(TWL4030_MODULE_INT, local_mask,
				     TWL4030_INT_PWR_IMR1);

		spin_lock_irq(&pwr_lock);
		if (local_mask == twl4030_pwrirq_mask)
			set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irq(&pwr_lock);

		schedule();
	}
	set_current_state(TASK_RUNNING);
	return 0;
}

static int __init twl4030_pwrirq_init(void)
{
	int i, err;

	twl4030_pwrirq_mask = 0xff;

	err = twl4030_i2c_write_u8(TWL4030_MODULE_INT, twl4030_pwrirq_mask,
					TWL4030_INT_PWR_IMR1);
	if (err)
		return err;

	/* Enable clear on read */

	err = twl4030_i2c_write_u8(TWL4030_MODULE_INT,
				TWL4030_SIH_CTRL_COR_MASK,
				TWL4030_INT_PWR_SIH_CTRL);
	if (err)
		return err;

	twl4030_pwrirq_unmask_thread = kthread_create(twl4030_pwrirq_thread,
		NULL, "twl4030 pwrirq");
	if (!twl4030_pwrirq_unmask_thread) {
		printk(KERN_ERR
			"%s: could not create twl4030 pwrirq unmask thread!\n",
				__func__);
		return -ENOMEM;
	}

	for (i = TWL4030_PWR_IRQ_BASE; i < TWL4030_PWR_IRQ_END; i++) {
		set_irq_chip_and_handler(i, &twl4030_pwrirq_chip,
				handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(TWL4030_MODIRQ_PWR, do_twl4030_pwrirq);

	return 0;
}
subsys_initcall(twl4030_pwrirq_init);

static void __exit twl4030_pwrirq_exit(void)
{

	int i;

	/* FIXME the irqs are left enabled; trouble when they arrive... */

	set_irq_handler(TWL4030_MODIRQ_PWR, NULL);
	set_irq_flags(TWL4030_MODIRQ_PWR, 0);

	for (i = TWL4030_PWR_IRQ_BASE; i < TWL4030_PWR_IRQ_END; i++) {
		set_irq_handler(i, NULL);
		set_irq_flags(i, 0);
	}

	if (twl4030_pwrirq_unmask_thread) {
		kthread_stop(twl4030_pwrirq_unmask_thread);
		twl4030_pwrirq_unmask_thread = NULL;
	}
}
module_exit(twl4030_pwrirq_exit);
