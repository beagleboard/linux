/**
 * drivers/cbus/retu-pwrbutton.c
 *
 * Driver for sending retu power button event to input-layer
 *
 * Copyright (C) 2004 Nokia Corporation
 *
 * Written by Ari Saastamoinen <ari.saastamoinen@elektrobit.com>
 *
 * Contact Juha Yrjölä <juha.yrjola@nokia.com>
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

#include "retu.h"

#define RETU_STATUS_PWRONX	(1 << 5)

#define PWRBTN_DELAY		20
#define PWRBTN_UP		0
#define PWRBTN_PRESSED		1

static int pwrbtn_state;
static struct input_dev *pwrbtn_dev;
static struct timer_list pwrbtn_timer;

static void retubutton_timer_func(unsigned long arg)
{
	int state;

	if (retu_read_reg(RETU_REG_STATUS) & RETU_STATUS_PWRONX)
		state = PWRBTN_UP;
	else
		state = PWRBTN_PRESSED;

	if (pwrbtn_state != state) {
		input_report_key(pwrbtn_dev, KEY_POWER, state);
		pwrbtn_state = state;
	}
}

/**
 * Interrupt function is called whenever power button key is pressed
 * or released.
 */
static void retubutton_irq(unsigned long arg)
{
	retu_ack_irq(RETU_INT_PWR);
	mod_timer(&pwrbtn_timer, jiffies + msecs_to_jiffies(PWRBTN_DELAY));
}

/**
 * Init function.
 * Allocates interrupt for power button and registers itself to input layer.
 */
static int __init retubutton_init(void)
{
	int irq;

	printk(KERN_INFO "Retu power button driver initialized\n");
	irq = RETU_INT_PWR;

	init_timer(&pwrbtn_timer);
	pwrbtn_timer.function = retubutton_timer_func;

	if (retu_request_irq(irq, &retubutton_irq, 0, "PwrOnX") < 0) {
		printk(KERN_ERR "%s@%s: Cannot allocate irq\n",
		       __FUNCTION__, __FILE__);
		return -EBUSY;
	}

	pwrbtn_dev = input_allocate_device();
	if (!pwrbtn_dev)
		return -ENOMEM;

	pwrbtn_dev->evbit[0] = BIT_MASK(EV_KEY);
	pwrbtn_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwrbtn_dev->name = "retu-pwrbutton";

	return input_register_device(pwrbtn_dev);
}

/**
 * Cleanup function which is called when driver is unloaded
 */
static void __exit retubutton_exit(void)
{
	retu_free_irq(RETU_INT_PWR);
	del_timer_sync(&pwrbtn_timer);
	input_unregister_device(pwrbtn_dev);
}

module_init(retubutton_init);
module_exit(retubutton_exit);

MODULE_DESCRIPTION("Retu Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ari Saastamoinen");
