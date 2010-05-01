/**
 * drivers/cbus/retu-pwrbutton.c
 *
 * Driver for sending retu power button event to input-layer
 *
 * Copyright (C) 2004-2010 Nokia Corporation
 *
 * Written by
 *	Ari Saastamoinen <ari.saastamoinen@elektrobit.com>
 *	Juha Yrjola <juha.yrjola@solidboot.com>
 *
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
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
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retu.h"

#define RETU_STATUS_PWRONX	(1 << 5)

#define PWRBTN_DELAY		20
#define PWRBTN_UP		0
#define PWRBTN_PRESSED		1

struct retu_pwrbutton {
	struct input_dev	*idev;
	struct timer_list	timer;

	int			state;
	int			irq;
};

static void retubutton_timer_func(unsigned long arg)
{
	struct retu_pwrbutton *pwr = (struct retu_pwrbutton *) arg;
	int state;

	if (retu_read_reg(RETU_REG_STATUS) & RETU_STATUS_PWRONX)
		state = PWRBTN_UP;
	else
		state = PWRBTN_PRESSED;

	if (pwr->state != state) {
		input_report_key(pwr->idev, KEY_POWER, state);
		input_sync(pwr->idev);
		pwr->state = state;
	}
}

/**
 * Interrupt function is called whenever power button key is pressed
 * or released.
 */
static void retubutton_irq(unsigned long arg)
{
	struct retu_pwrbutton *pwr = (struct retu_pwrbutton *) arg;

	retu_ack_irq(RETU_INT_PWR);
	mod_timer(&pwr->timer, jiffies + msecs_to_jiffies(PWRBTN_DELAY));
}

/**
 * Init function.
 * Allocates interrupt for power button and registers itself to input layer.
 */
static int __devinit retubutton_probe(struct platform_device *pdev)
{
	struct retu_pwrbutton		*pwr;
	int				ret = 0;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (!pwr) {
		dev_err(&pdev->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err0;
	}

	pwr->irq = RETU_INT_PWR;
	platform_set_drvdata(pdev, pwr);
	setup_timer(&pwr->timer, retubutton_timer_func, (unsigned long) pwr);

	ret = retu_request_irq(pwr->irq, retubutton_irq, (unsigned long) pwr,
			"PwrOnX");
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot allocate irq\n");
		goto err1;
	}

	pwr->idev = input_allocate_device();
	if (!pwr->idev) {
		dev_err(&pdev->dev, "can't allocate input device\n");
		ret = -ENOMEM;
		goto err2;
	}

	pwr->idev->evbit[0] = BIT_MASK(EV_KEY);
	pwr->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->idev->name = "retu-pwrbutton";

	ret = input_register_device(pwr->idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err3;
	}

	return 0;

err3:
	input_free_device(pwr->idev);

err2:
	retu_free_irq(pwr->irq);

err1:
	kfree(pwr);

err0:
	return ret;
}

/**
 * Cleanup function which is called when driver is unloaded
 */
static int __devexit retubutton_remove(struct platform_device *pdev)
{
	struct retu_pwrbutton		*pwr = platform_get_drvdata(pdev);

	retu_free_irq(pwr->irq);
	del_timer_sync(&pwr->timer);
	input_unregister_device(pwr->idev);
	input_free_device(pwr->idev);
	kfree(pwr);

	return 0;
}

static struct platform_driver retu_pwrbutton_driver = {
	.probe		= retubutton_probe,
	.remove		= __devexit_p(retubutton_remove),
	.driver		= {
		.name	= "retu-pwrbutton",
	},
};

static int __init retubutton_init(void)
{
	return platform_driver_register(&retu_pwrbutton_driver);
}
module_init(retubutton_init);

static void __exit retubutton_exit(void)
{
	platform_driver_unregister(&retu_pwrbutton_driver);
}
module_exit(retubutton_exit);

MODULE_DESCRIPTION("Retu Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ari Saastamoinen");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

