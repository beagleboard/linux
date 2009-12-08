/**
 * drivers/cbus/tahvo.c
 *
 * Support functions for Tahvo ASIC
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>,
 *	      David Weinehall <david.weinehall@nokia.com>, and
 *	      Mikko Ylinen <mikko.k.ylinen@nokia.com>
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
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>

#include <mach/mux.h>
#include <mach/board.h>
#include <mach/board-nokia.h>

#include "cbus.h"
#include "tahvo.h"

#define TAHVO_ID		0x02
#define PFX			"tahvo: "

static int tahvo_initialized;
static int tahvo_irq_pin;
static int tahvo_is_betty;

static struct tasklet_struct tahvo_tasklet;
spinlock_t tahvo_lock = SPIN_LOCK_UNLOCKED;

static struct completion device_release;

struct tahvo_irq_handler_desc {
	int (*func)(unsigned long);
	unsigned long arg;
	char name[8];
};

static struct tahvo_irq_handler_desc tahvo_irq_handlers[MAX_TAHVO_IRQ_HANDLERS];

/**
 * tahvo_read_reg - Read a value from a register in Tahvo
 * @reg: the register to read from
 *
 * This function returns the contents of the specified register
 */
int tahvo_read_reg(int reg)
{
	BUG_ON(!tahvo_initialized);
	return cbus_read_reg(cbus_host, TAHVO_ID, reg);
}

/**
 * tahvo_write_reg - Write a value to a register in Tahvo
 * @reg: the register to write to
 * @reg: the value to write to the register
 *
 * This function writes a value to the specified register
 */
void tahvo_write_reg(int reg, u16 val)
{
	BUG_ON(!tahvo_initialized);
	cbus_write_reg(cbus_host, TAHVO_ID, reg, val);
}

/**
 * tahvo_set_clear_reg_bits - set and clear register bits atomically
 * @reg: the register to write to
 * @bits: the bits to set
 *
 * This function sets and clears the specified Tahvo register bits atomically
 */
void tahvo_set_clear_reg_bits(int reg, u16 set, u16 clear)
{
	unsigned long flags;
	u16 w;

	spin_lock_irqsave(&tahvo_lock, flags);
	w = tahvo_read_reg(reg);
	w &= ~clear;
	w |= set;
	tahvo_write_reg(reg, w);
	spin_unlock_irqrestore(&tahvo_lock, flags);
}

/*
 * Disable given TAHVO interrupt
 */
void tahvo_disable_irq(int id)
{
	unsigned long flags;
	u16 mask;

	spin_lock_irqsave(&tahvo_lock, flags);
	mask = tahvo_read_reg(TAHVO_REG_IMR);
	mask |= 1 << id;
	tahvo_write_reg(TAHVO_REG_IMR, mask);
	spin_unlock_irqrestore(&tahvo_lock, flags);
}

/*
 * Enable given TAHVO interrupt
 */
void tahvo_enable_irq(int id)
{
	unsigned long flags;
	u16 mask;

	spin_lock_irqsave(&tahvo_lock, flags);
	mask = tahvo_read_reg(TAHVO_REG_IMR);
	mask &= ~(1 << id);
	tahvo_write_reg(TAHVO_REG_IMR, mask);
	spin_unlock_irqrestore(&tahvo_lock, flags);
}

/*
 * Acknowledge given TAHVO interrupt
 */
void tahvo_ack_irq(int id)
{
	tahvo_write_reg(TAHVO_REG_IDR, 1 << id);
}

static int tahvo_7bit_backlight;

int tahvo_get_backlight_level(void)
{
	int mask;

	if (tahvo_7bit_backlight)
		mask = 0x7f;
	else
		mask = 0x0f;
	return tahvo_read_reg(TAHVO_REG_LEDPWMR) & mask;
}

int tahvo_get_max_backlight_level(void)
{
	if (tahvo_7bit_backlight)
		return 0x7f;
	else
		return 0x0f;
}

void tahvo_set_backlight_level(int level)
{
	int max_level;

	max_level = tahvo_get_max_backlight_level();
	if (level > max_level)
		level = max_level;
	tahvo_write_reg(TAHVO_REG_LEDPWMR, level);
}

/*
 * TAHVO interrupt handler. Only schedules the tasklet.
 */
static irqreturn_t tahvo_irq_handler(int irq, void *dev_id)
{
	tasklet_schedule(&tahvo_tasklet);
	return IRQ_HANDLED;
}

/*
 * Tasklet handler
 */
static void tahvo_tasklet_handler(unsigned long data)
{
	struct tahvo_irq_handler_desc *hnd;
	u16 id;
	u16 im;
	int i;

	for (;;) {
		id = tahvo_read_reg(TAHVO_REG_IDR);
		im = ~tahvo_read_reg(TAHVO_REG_IMR);
		id &= im;

		if (!id)
			break;

		for (i = 0; id != 0; i++, id >>= 1) {
			if (!(id & 1))
				continue;
			hnd = &tahvo_irq_handlers[i];
			if (hnd->func == NULL) {
				/* Spurious tahvo interrupt - just ack it */
				printk(KERN_INFO "Spurious Tahvo interrupt "
						 "(id %d)\n", i);
				tahvo_disable_irq(i);
				tahvo_ack_irq(i);
				continue;
			}
			hnd->func(hnd->arg);
			/*
			 * Don't acknowledge the interrupt here
			 * It must be done explicitly
			 */
		}
	}
}

/*
 * Register the handler for a given TAHVO interrupt source.
 */
int tahvo_request_irq(int id, void *irq_handler, unsigned long arg, char *name)
{
	struct tahvo_irq_handler_desc *hnd;

	if (irq_handler == NULL || id >= MAX_TAHVO_IRQ_HANDLERS ||
	    name == NULL) {
		printk(KERN_ERR PFX "Invalid arguments to %s\n",
		       __FUNCTION__);
		return -EINVAL;
	}
	hnd = &tahvo_irq_handlers[id];
	if (hnd->func != NULL) {
		printk(KERN_ERR PFX "IRQ %d already reserved\n", id);
		return -EBUSY;
	}
	printk(KERN_INFO PFX "Registering interrupt %d for device %s\n",
	       id, name);
	hnd->func = irq_handler;
	hnd->arg = arg;
	strlcpy(hnd->name, name, sizeof(hnd->name));

	tahvo_ack_irq(id);
	tahvo_enable_irq(id);

	return 0;
}

/*
 * Unregister the handler for a given TAHVO interrupt source.
 */
void tahvo_free_irq(int id)
{
	struct tahvo_irq_handler_desc *hnd;

	if (id >= MAX_TAHVO_IRQ_HANDLERS) {
		printk(KERN_ERR PFX "Invalid argument to %s\n",
		       __FUNCTION__);
		return;
	}
	hnd = &tahvo_irq_handlers[id];
	if (hnd->func == NULL) {
		printk(KERN_ERR PFX "IRQ %d already freed\n", id);
		return;
	}

	tahvo_disable_irq(id);
	hnd->func = NULL;
}

/**
 * tahvo_probe - Probe for Tahvo ASIC
 * @dev: the Tahvo device
 *
 * Probe for the Tahvo ASIC and allocate memory
 * for its device-struct if found
 */
static int __devinit tahvo_probe(struct device *dev)
{
	const struct omap_em_asic_bb5_config * em_asic_config;
	int rev, id, ret;

	/* Prepare tasklet */
	tasklet_init(&tahvo_tasklet, tahvo_tasklet_handler, 0);

	em_asic_config = omap_get_config(OMAP_TAG_EM_ASIC_BB5,
					 struct omap_em_asic_bb5_config);
	if (em_asic_config == NULL) {
		printk(KERN_ERR PFX "Unable to retrieve config data\n");
		return -ENODATA;
	}

	tahvo_initialized = 1;

	rev = tahvo_read_reg(TAHVO_REG_ASICR);

	id = (rev >> 8) & 0xff;
	if (id == 0x03) {
		if ((rev & 0xff) >= 0x50)
			tahvo_7bit_backlight = 1;
	} else if (id == 0x0b) {
		tahvo_is_betty = 1;
		tahvo_7bit_backlight = 1;
	} else {
		printk(KERN_ERR "Tahvo/Betty chip not found");
		return -ENODEV;
	}

	printk(KERN_INFO "%s v%d.%d found\n", tahvo_is_betty ? "Betty" : "Tahvo",
	       (rev >> 4) & 0x0f, rev & 0x0f);

	tahvo_irq_pin = em_asic_config->tahvo_irq_gpio;

	if ((ret = gpio_request(tahvo_irq_pin, "TAHVO irq")) < 0) {
		printk(KERN_ERR PFX "Unable to reserve IRQ GPIO\n");
		return ret;
	}

	/* Set the pin as input */
	gpio_direction_input(tahvo_irq_pin);

	/* Rising edge triggers the IRQ */
	set_irq_type(gpio_to_irq(tahvo_irq_pin), IRQ_TYPE_EDGE_RISING);

	/* Mask all TAHVO interrupts */
	tahvo_write_reg(TAHVO_REG_IMR, 0xffff);

	ret = request_irq(gpio_to_irq(tahvo_irq_pin), tahvo_irq_handler, 0,
			  "tahvo", 0);
	if (ret < 0) {
		printk(KERN_ERR PFX "Unable to register IRQ handler\n");
		gpio_free(tahvo_irq_pin);
		return ret;
	}
#ifdef CONFIG_CBUS_TAHVO_USER
	/* Initialize user-space interface */
	if (tahvo_user_init() < 0) {
		printk(KERN_ERR "Unable to initialize driver\n");
		free_irq(gpio_to_irq(tahvo_irq_pin), 0);
		gpio_free(tahvo_irq_pin);
		return ret;
	}
#endif
	return 0;
}

static int tahvo_remove(struct device *dev)
{
#ifdef CONFIG_CBUS_TAHVO_USER
	tahvo_user_cleanup();
#endif
	/* Mask all TAHVO interrupts */
	tahvo_write_reg(TAHVO_REG_IMR, 0xffff);
	free_irq(gpio_to_irq(tahvo_irq_pin), 0);
	gpio_free(tahvo_irq_pin);
	tasklet_kill(&tahvo_tasklet);

	return 0;
}

static void tahvo_device_release(struct device *dev)
{
	complete(&device_release);
}

static struct device_driver tahvo_driver = {
	.name		= "tahvo",
	.bus		= &platform_bus_type,
	.probe		= tahvo_probe,
	.remove		= tahvo_remove,
};

static struct platform_device tahvo_device = {
	.name		= "tahvo",
	.id		= -1,
	.dev = {
		.release = tahvo_device_release,
	}
};

/**
 * tahvo_init - initialise Tahvo driver
 *
 * Initialise the Tahvo driver and return 0 if everything worked ok
 */
static int __init tahvo_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Tahvo/Betty driver initialising\n");

	init_completion(&device_release);

	if ((ret = driver_register(&tahvo_driver)) < 0)
		return ret;

	if ((ret = platform_device_register(&tahvo_device)) < 0) {
		driver_unregister(&tahvo_driver);
		return ret;
	}
	return 0;
}

/*
 * Cleanup
 */
static void __exit tahvo_exit(void)
{
	platform_device_unregister(&tahvo_device);
	driver_unregister(&tahvo_driver);
	wait_for_completion(&device_release);
}

EXPORT_SYMBOL(tahvo_request_irq);
EXPORT_SYMBOL(tahvo_free_irq);
EXPORT_SYMBOL(tahvo_enable_irq);
EXPORT_SYMBOL(tahvo_disable_irq);
EXPORT_SYMBOL(tahvo_ack_irq);
EXPORT_SYMBOL(tahvo_read_reg);
EXPORT_SYMBOL(tahvo_write_reg);
EXPORT_SYMBOL(tahvo_get_backlight_level);
EXPORT_SYMBOL(tahvo_get_max_backlight_level);
EXPORT_SYMBOL(tahvo_set_backlight_level);

subsys_initcall(tahvo_init);
module_exit(tahvo_exit);

MODULE_DESCRIPTION("Tahvo ASIC control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä, David Weinehall, and Mikko Ylinen");
