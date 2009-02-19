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
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>

#define PWR_PWRON_IRQ (1 << 0)

#define STS_HW_CONDITIONS 0xf

static struct input_dev *powerbutton_dev;
static struct device *dbg_dev;

static irqreturn_t powerbutton_irq(int irq, void *dev_id)
{
	int err;
	u8 value;

#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	err = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value,
				  STS_HW_CONDITIONS);
	if (!err)  {
		input_report_key(powerbutton_dev, KEY_POWER,
				 value & PWR_PWRON_IRQ);
	} else {
		dev_err(dbg_dev, "twl4030: i2c error %d while reading TWL4030"
			" PM_MASTER STS_HW_CONDITIONS register\n", err);
	}

	return IRQ_HANDLED;
}

static int __devinit twl4030_pwrbutton_probe(struct platform_device *pdev)
{
	int err = 0;
	int irq = platform_get_irq(pdev, 0);

	dbg_dev = &pdev->dev;

	/* PWRBTN == PWRON */
	err = request_irq(irq, powerbutton_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl4030-pwrbutton", NULL);
	if (err < 0) {
		dev_dbg(&pdev->dev, "Can't get IRQ for power button: %d\n", err);
		goto out;
	}

	powerbutton_dev = input_allocate_device();
	if (!powerbutton_dev) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		err = -ENOMEM;
		goto free_irq_and_out;
	}

	powerbutton_dev->evbit[0] = BIT_MASK(EV_KEY);
	powerbutton_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	powerbutton_dev->name = "triton2-pwrbutton";

	err = input_register_device(powerbutton_dev);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_input_dev;
	}

	dev_info(&pdev->dev, "triton2 power button driver initialized\n");

	return 0;


free_input_dev:
	input_free_device(powerbutton_dev);
free_irq_and_out:
	free_irq(TWL4030_PWRIRQ_PWRBTN, NULL);
out:
	return err;
}

static int __devexit twl4030_pwrbutton_remove(struct platform_device *pdev)
{
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, NULL);
	input_unregister_device(powerbutton_dev);
	input_free_device(powerbutton_dev);

	return 0;
}

struct platform_driver twl4030_pwrbutton_driver = {
	.probe		= twl4030_pwrbutton_probe,
	.remove		= twl4030_pwrbutton_remove,
	.driver		= {
		.name	= "twl4030-pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_pwrbutton_init(void)
{
	return platform_driver_register(&twl4030_pwrbutton_driver);
}
module_init(twl4030_pwrbutton_init);

static void __exit twl4030_pwrbutton_exit(void)
{
	platform_driver_unregister(&twl4030_pwrbutton_driver);
}
module_exit(twl4030_pwrbutton_exit);

MODULE_DESCRIPTION("Triton2 Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver");

