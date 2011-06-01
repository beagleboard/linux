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
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retu.h"

#define RETU_STATUS_PWRONX	(1 << 5)

#define PWRBTN_DELAY		20
#define PWRBTN_UP		0
#define PWRBTN_PRESSED		1

struct retu_pwrbutton {
	struct input_dev	*idev;
	struct device		*dev;

	int			state;
	int			irq;
};

static irqreturn_t retubutton_irq(int irq, void *_pwr)
{
	struct retu_pwrbutton *pwr = _pwr;
	int state;

	if (retu_read_reg(pwr->dev, RETU_REG_STATUS) & RETU_STATUS_PWRONX)
		state = PWRBTN_UP;
	else
		state = PWRBTN_PRESSED;

	if (pwr->state != state) {
		input_report_key(pwr->idev, KEY_POWER, state);
		input_sync(pwr->idev);
		pwr->state = state;
	}

	return IRQ_HANDLED;
}

static int __init retubutton_probe(struct platform_device *pdev)
{
	struct retu_pwrbutton		*pwr;
	int				ret = 0;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (!pwr) {
		dev_err(&pdev->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err0;
	}

	pwr->dev = &pdev->dev;
	pwr->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, pwr);

	ret = request_threaded_irq(pwr->irq, NULL, retubutton_irq, 0,
			"retu-pwrbutton", pwr);
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
	free_irq(pwr->irq, pwr);

err1:
	kfree(pwr);

err0:
	return ret;
}

static int __exit retubutton_remove(struct platform_device *pdev)
{
	struct retu_pwrbutton		*pwr = platform_get_drvdata(pdev);

	free_irq(pwr->irq, pwr);
	input_unregister_device(pwr->idev);
	input_free_device(pwr->idev);
	kfree(pwr);

	return 0;
}

static struct platform_driver retu_pwrbutton_driver = {
	.remove		= __exit_p(retubutton_remove),
	.driver		= {
		.name	= "retu-pwrbutton",
	},
};

static int __init retubutton_init(void)
{
	return platform_driver_probe(&retu_pwrbutton_driver, retubutton_probe);
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

