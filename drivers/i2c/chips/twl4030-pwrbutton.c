/**
 * drivers/i2c/chips/twl4030-pwrbutton.c
 *
 * Driver for sending triton2 power button event to input-layer
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c/twl4030.h>

#define PWR_ISR1 0
#define PWR_IMR1 1
#define PWR_PWRON_IRQ (1<<0)

#define PWR_EDR1 5
#define PWR_PWRON_RISING (1<<1)
#define PWR_PWRON_FALLING  (1<<0)
#define PWR_PWRON_BOTH (PWR_PWRON_RISING | PWR_PWRON_FALLING)

#define PWR_SIH_CTRL 7

#define STS_HW_CONDITIONS 0xf

static struct input_dev *powerbutton_dev;

static irqreturn_t powerbutton_irq(int irq, void *dev_id)
{
	int err;
	u8 value;

	err = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value,
				  STS_HW_CONDITIONS);
	if (!err)  {
		input_report_key(powerbutton_dev, KEY_POWER,
				 value & PWR_PWRON_IRQ);
	} else {
		pr_err("twl4030: i2c error %d while reading TWL4030"
			" PM_MASTER STS_HW_CONDITIONS register\n", err);
	}

	return IRQ_HANDLED;
}

static int __init twl4030_pwrbutton_init(void)
{
	int err = 0;
	u8 value;

	if (request_irq(TWL4030_PWRIRQ_PWRBTN, powerbutton_irq, 0,
			"PwrButton", NULL) < 0) {
		printk(KERN_ERR "Unable to allocate IRQ for power button\n");
		err = -EBUSY;
		goto out;
	}

	powerbutton_dev = input_allocate_device();
	if (!powerbutton_dev) {
		printk(KERN_ERR
			"Unable to allocate input device for power button\n");
		err = -ENOMEM;
		goto free_irq_and_out;
	}

	powerbutton_dev->evbit[0] = BIT_MASK(EV_KEY);
	powerbutton_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	powerbutton_dev->name = "triton2-pwrbutton";

	err = input_register_device(powerbutton_dev);
	if (err) {
		input_free_device(powerbutton_dev);
		goto free_irq_and_out;
	}

	err = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &value, PWR_IMR1);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					" INT PWR_IMR1 register\n", err);

		goto free_input_dev;
	}

	err = twl4030_i2c_write_u8(TWL4030_MODULE_INT,
				   value & (~PWR_PWRON_IRQ), PWR_IMR1);
	if (err) {
		printk(KERN_WARNING "I2C error %d while writing TWL4030"
				    " INT PWR_IMR1 register\n", err);
		goto free_input_dev;
	}

	err = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &value, PWR_EDR1);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					" INT PWR_EDR1 register\n", err);
		goto free_input_dev;
	}

	err = twl4030_i2c_write_u8(TWL4030_MODULE_INT,
				   value | PWR_PWRON_BOTH, PWR_EDR1);

	if (err) {
		printk(KERN_WARNING "I2C error %d while writing TWL4030"
					" INT PWR_EDR1 register\n", err);
		goto free_input_dev;
	}

	printk(KERN_INFO "triton2 power button driver initialized\n");

	return 0;


free_input_dev:
	input_unregister_device(powerbutton_dev);
free_irq_and_out:
	free_irq(TWL4030_PWRIRQ_PWRBTN, NULL);
out:
	return err;
}

static void __exit twl4030_pwrbutton_exit(void)
{
	free_irq(TWL4030_PWRIRQ_PWRBTN, NULL);
	input_unregister_device(powerbutton_dev);
	input_free_device(powerbutton_dev);

}

module_init(twl4030_pwrbutton_init);
module_exit(twl4030_pwrbutton_exit);

MODULE_ALIAS("i2c:twl4030-pwrbutton");
MODULE_DESCRIPTION("Triton2 Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver");

