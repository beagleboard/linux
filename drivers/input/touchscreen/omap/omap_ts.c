/*
 * input/touchscreen/omap/omap_ts.c
 *
 * touchscreen input device driver for various TI OMAP boards
 * Copyright (c) 2002 MontaVista Software Inc.
 * Copyright (c) 2004 Texas Instruments, Inc.
 * Cleanup and modularization 2004 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Assembled using driver code copyright the companies above.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * History:
 * 12/12/2004    Srinath Modified and intergrated code for H2 and H3
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>

//#define DEBUG

#include "omap_ts.h"

#define OMAP_TS_NAME	"omap_ts"

static struct ts_device *__initdata ts_devs[] = {
#if defined(CONFIG_MACH_OMAP_H2) || defined(CONFIG_MACH_OMAP_H3)
	&hx_ts,
#endif
};

static struct omap_ts_t ts_omap;

static int omap_ts_read(void)
{
	u16 data[4] = { 0, 0, 0, 0 };

	ts_omap.dev->read(data);

	input_report_abs(ts_omap.inputdevice, ABS_X, data[0]);
	input_report_abs(ts_omap.inputdevice, ABS_Y, data[1]);
	input_report_abs(ts_omap.inputdevice, ABS_PRESSURE, data[2]);
	input_sync(ts_omap.inputdevice);

	DEBUG_TS("omap_ts_read: read x=%d,y=%d,p=%d\n", data[0], data[1],
		 data[2]);

	return 0;
}

static void omap_ts_timer(unsigned long data)
{
	unsigned long flags;

	spin_lock_irqsave(&ts_omap.lock, flags);

	if (!ts_omap.dev->penup()) {
		if (!ts_omap.touched) {
			DEBUG_TS("omap_ts_timer: pen down\n");
			input_report_key(ts_omap.inputdevice, BTN_TOUCH, 1);
		}
		ts_omap.touched = 1;
		omap_ts_read();
		ts_omap.ts_timer.expires = jiffies + HZ / 100;
		add_timer(&(ts_omap.ts_timer));
	} else {
		if (ts_omap.touched) {
			DEBUG_TS("omap_ts_timer: pen up\n");
			ts_omap.touched = 0;
			input_report_abs(ts_omap.inputdevice, ABS_X, 0);
			input_report_abs(ts_omap.inputdevice, ABS_Y, 0);
			input_report_abs(ts_omap.inputdevice, ABS_PRESSURE,
					 0);
			input_sync(ts_omap.inputdevice);
			input_report_key(ts_omap.inputdevice, BTN_TOUCH, 0);
		}
		if (!ts_omap.irq_enabled) {
			ts_omap.irq_enabled = 1;
			enable_irq(ts_omap.irq);
		}
	}

	spin_unlock_irqrestore(&ts_omap.lock, flags);
}

static irqreturn_t omap_ts_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	spin_lock(&ts_omap.lock);

	if (ts_omap.irq_enabled) {
		ts_omap.irq_enabled = 0;
		disable_irq(irq);
	}
	// restart acquire
	mod_timer(&ts_omap.ts_timer, jiffies + HZ / 100);

	spin_unlock(&ts_omap.lock);

	return IRQ_HANDLED;
}

static int __init omap_ts_probe(struct platform_device *pdev)
{
	int i;
	int status = -ENODEV;

	memset(&ts_omap, 0, sizeof(ts_omap));

	ts_omap.inputdevice = input_allocate_device();
	if (!ts_omap.inputdevice) {
		return -ENOMEM;
	}

	spin_lock_init(&ts_omap.lock);

	for (i = 0; i < ARRAY_SIZE(ts_devs); i++) {
		if (!ts_devs[i] || !ts_devs[i]->probe)
			continue;
		status = ts_devs[i]->probe(&ts_omap);
		if (status == 0) {
			ts_omap.dev = ts_devs[i];
			break;
		}
	}

	if (status != 0) {
	    	input_free_device(ts_omap.inputdevice);
		return status;
	}

	// Init acquisition timer function
	init_timer(&ts_omap.ts_timer);
	ts_omap.ts_timer.function = omap_ts_timer;

	/* request irq */
	if (ts_omap.irq != -1) {
		if (request_irq(ts_omap.irq, omap_ts_handler,
				SA_SAMPLE_RANDOM | ts_omap.irq_type,
				OMAP_TS_NAME, &ts_omap)) {
			printk(KERN_ERR
	  "omap_ts.c: Could not allocate touchscreen IRQ!\n");
			ts_omap.irq = -1;
			ts_omap.dev->remove();
			input_free_device(ts_omap.inputdevice);
			return -EINVAL;
		}
		ts_omap.irq_enabled = 1;
	} else {
		printk(KERN_ERR "omap_ts.c: No touchscreen IRQ assigned!\n");
		ts_omap.dev->remove();
		input_free_device(ts_omap.inputdevice);
		return -EINVAL;
	}

	ts_omap.inputdevice->name = OMAP_TS_NAME;
	ts_omap.inputdevice->dev = &pdev->dev;
	ts_omap.inputdevice->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	ts_omap.inputdevice->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
	ts_omap.inputdevice->absbit[0] =
	    BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_register_device(ts_omap.inputdevice);

	ts_omap.dev->enable();

	printk("OMAP touchscreen driver initialized\n");

	return 0;
}

static int omap_ts_remove(struct platform_device *pdev)
{
	ts_omap.dev->disable();
	input_unregister_device(ts_omap.inputdevice);
	if (ts_omap.irq != -1)
		free_irq(ts_omap.irq, &ts_omap);

	ts_omap.dev->remove();

	return 0;
}

static int omap_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	ts_omap.dev->disable();
	return 0;
}

static int omap_ts_resume(struct platform_device *pdev)
{
	ts_omap.dev->enable();
	return 0;
}

static void omap_ts_device_release(struct device *dev)
{
	/* Nothing */
}
static struct platform_driver omap_ts_driver = {
	.probe 		= omap_ts_probe,
	.remove 	= omap_ts_remove,
	.suspend 	= omap_ts_suspend,
	.resume 	= omap_ts_resume,
	.driver = {
		.name	= OMAP_TS_NAME,
	},
};

static struct platform_device omap_ts_device = {
	.name 		= OMAP_TS_NAME,
	.id 		= -1,
	.dev = {
		.release 	= omap_ts_device_release,
	},
};

static int __init omap_ts_init(void)
{
	int ret;

	if (machine_is_omap_osk() || machine_is_omap_innovator())
		return -ENODEV;

	ret = platform_device_register(&omap_ts_device);
	if (ret != 0)
		return -ENODEV;

	ret = platform_driver_register(&omap_ts_driver);
	if (ret != 0) {
		platform_device_unregister(&omap_ts_device);
		return -ENODEV;
	}

	return 0;
}

static void __exit omap_ts_exit(void)
{
	platform_driver_unregister(&omap_ts_driver);
	platform_device_unregister(&omap_ts_device);
}

module_init(omap_ts_init);
module_exit(omap_ts_exit);

MODULE_LICENSE("GPL");
