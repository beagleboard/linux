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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include "cbus.h"
#include "tahvo.h"

#define TAHVO_ID		0x02
#define PFX			"tahvo: "

struct tahvo {
	/* device lock */
	struct mutex	mutex;
	struct device	*dev;

	unsigned int	is_betty;
	unsigned int	wide_backlight;
};

static struct tahvo *the_tahvo;

struct tahvo_irq_handler_desc {
	int (*func)(unsigned long);
	unsigned long arg;
	char name[8];
};

static struct tahvo_irq_handler_desc tahvo_irq_handlers[MAX_TAHVO_IRQ_HANDLERS];

int tahvo_get_status(void)
{
	return the_tahvo != NULL;
}
EXPORT_SYMBOL(tahvo_get_status);

/**
 * tahvo_read_reg - Read a value from a register in Tahvo
 * @reg: the register to read from
 *
 * This function returns the contents of the specified register
 */
int tahvo_read_reg(unsigned reg)
{
	struct tahvo		*tahvo = the_tahvo;

	return cbus_read_reg(tahvo->dev, TAHVO_ID, reg);
}
EXPORT_SYMBOL(tahvo_read_reg);

/**
 * tahvo_write_reg - Write a value to a register in Tahvo
 * @reg: the register to write to
 * @reg: the value to write to the register
 *
 * This function writes a value to the specified register
 */
void tahvo_write_reg(unsigned reg, u16 val)
{
	struct tahvo		*tahvo = the_tahvo;

	cbus_write_reg(tahvo->dev, TAHVO_ID, reg, val);
}
EXPORT_SYMBOL(tahvo_write_reg);

/**
 * tahvo_set_clear_reg_bits - set and clear register bits atomically
 * @reg: the register to write to
 * @bits: the bits to set
 *
 * This function sets and clears the specified Tahvo register bits atomically
 */
void tahvo_set_clear_reg_bits(unsigned reg, u16 set, u16 clear)
{
	struct tahvo		*tahvo = the_tahvo;
	u16			w;

	mutex_lock(&tahvo->mutex);
	w = tahvo_read_reg(reg);
	w &= ~clear;
	w |= set;
	tahvo_write_reg(reg, w);
	mutex_unlock(&tahvo->mutex);
}

void tahvo_disable_irq(int id)
{
	struct tahvo		*tahvo = the_tahvo;
	u16			mask;

	mutex_lock(&tahvo->mutex);
	mask = tahvo_read_reg(TAHVO_REG_IMR);
	mask |= 1 << id;
	tahvo_write_reg(TAHVO_REG_IMR, mask);
	mutex_unlock(&tahvo->mutex);
}
EXPORT_SYMBOL(tahvo_disable_irq);

void tahvo_enable_irq(int id)
{
	struct tahvo		*tahvo = the_tahvo;
	u16			mask;

	mutex_lock(&tahvo->mutex);
	mask = tahvo_read_reg(TAHVO_REG_IMR);
	mask &= ~(1 << id);
	tahvo_write_reg(TAHVO_REG_IMR, mask);
	mutex_unlock(&tahvo->mutex);
}
EXPORT_SYMBOL(tahvo_enable_irq);

void tahvo_ack_irq(int id)
{
	tahvo_write_reg(TAHVO_REG_IDR, 1 << id);
}
EXPORT_SYMBOL(tahvo_ack_irq);

int tahvo_get_backlight_level(void)
{
	struct tahvo		*tahvo = the_tahvo;
	int			mask;

	if (tahvo->wide_backlight)
		mask = 0x7f;
	else
		mask = 0x0f;
	return tahvo_read_reg(TAHVO_REG_LEDPWMR) & mask;
}
EXPORT_SYMBOL(tahvo_get_backlight_level);

int tahvo_get_max_backlight_level(void)
{
	struct tahvo		*tahvo = the_tahvo;

	if (tahvo->wide_backlight)
		return 0x7f;
	else
		return 0x0f;
}
EXPORT_SYMBOL(tahvo_get_max_backlight_level);

void tahvo_set_backlight_level(int level)
{
	int			max_level;

	max_level = tahvo_get_max_backlight_level();
	if (level > max_level)
		level = max_level;
	tahvo_write_reg(TAHVO_REG_LEDPWMR, level);
}
EXPORT_SYMBOL(tahvo_set_backlight_level);

static irqreturn_t tahvo_irq_handler(int irq, void *dev_id)
{
	struct tahvo_irq_handler_desc *hnd;

	struct tahvo		*tahvo = the_tahvo;
	u16			id;
	u16			im;
	int			i;

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
				dev_err(tahvo->dev, "Spurious interrupt "
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

	return IRQ_HANDLED;
}

/*
 * Register the handler for a given TAHVO interrupt source.
 */
int tahvo_request_irq(int id, void *irq_handler, unsigned long arg, char *name)
{
	struct tahvo_irq_handler_desc *hnd;
	struct tahvo		*tahvo = the_tahvo;

	if (irq_handler == NULL || id >= MAX_TAHVO_IRQ_HANDLERS ||
	    name == NULL) {
		dev_err(tahvo->dev, "Invalid arguments to %s\n",
		       __FUNCTION__);
		return -EINVAL;
	}
	hnd = &tahvo_irq_handlers[id];
	if (hnd->func != NULL) {
		dev_err(tahvo->dev, "IRQ %d already reserved\n", id);
		return -EBUSY;
	}
	dev_dbg(tahvo->dev, "Registering interrupt %d for device %s\n",
	       id, name);
	hnd->func = irq_handler;
	hnd->arg = arg;
	strlcpy(hnd->name, name, sizeof(hnd->name));

	tahvo_ack_irq(id);
	tahvo_enable_irq(id);

	return 0;
}
EXPORT_SYMBOL(tahvo_request_irq);

/*
 * Unregister the handler for a given TAHVO interrupt source.
 */
void tahvo_free_irq(int id)
{
	struct tahvo_irq_handler_desc *hnd;
	struct tahvo		*tahvo = the_tahvo;

	if (id >= MAX_TAHVO_IRQ_HANDLERS) {
		dev_err(tahvo->dev, "Invalid argument to %s\n",
		       __FUNCTION__);
		return;
	}
	hnd = &tahvo_irq_handlers[id];
	if (hnd->func == NULL) {
		dev_err(tahvo->dev, "IRQ %d already freed\n", id);
		return;
	}

	tahvo_disable_irq(id);
	hnd->func = NULL;
}
EXPORT_SYMBOL(tahvo_free_irq);

static int __devinit tahvo_probe(struct platform_device *pdev)
{
	struct tahvo		*tahvo;
	int			rev;
	int			ret;
	int			irq;
	int			id;

	tahvo = kzalloc(sizeof(*tahvo), GFP_KERNEL);
	if (!tahvo) {
		dev_err(&pdev->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err0;
	}

	the_tahvo = tahvo;
	platform_set_drvdata(pdev, tahvo);

	mutex_init(&tahvo->mutex);
	tahvo->dev = &pdev->dev;

	rev = tahvo_read_reg(TAHVO_REG_ASICR);

	id = (rev >> 8) & 0xff;

	switch (id) {
	case 0x03:
		if ((rev & 0xff) >= 0x50)
			tahvo->wide_backlight = true;
		break;
	case 0x0b:
		tahvo->is_betty = true;
		tahvo->wide_backlight = true;
		break;
	default:
		dev_err(&pdev->dev, "Tahvo/Betty chip not found");
		ret = -ENODEV;
		goto err1;
	}

	dev_err(&pdev->dev, "%s v%d.%d found\n",
			tahvo->is_betty ? "Betty" : "Tahvo",
			(rev >> 4) & 0x0f, rev & 0x0f);

	irq = platform_get_irq(pdev, 0);

	/* Mask all TAHVO interrupts */
	tahvo_write_reg(TAHVO_REG_IMR, 0xffff);

	ret = request_threaded_irq(irq, NULL, tahvo_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"tahvo", 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register IRQ handler\n");
		goto err1;
	}

	return 0;

err1:
	kfree(tahvo);

err0:
	return ret;
}

static int __devexit tahvo_remove(struct platform_device *pdev)
{
	struct tahvo		*tahvo = platform_get_drvdata(pdev);
	int			irq;

	irq = platform_get_irq(pdev, 0);

	/* Mask all TAHVO interrupts */
	tahvo_write_reg(TAHVO_REG_IMR, 0xffff);
	free_irq(irq, 0);
	kfree(tahvo);
	the_tahvo = NULL;

	return 0;
}

static struct platform_driver tahvo_driver = {
	.remove		= __devexit_p(tahvo_remove),
	.driver		= {
		.name	= "tahvo",
	},
};

static int __init tahvo_init(void)
{
	return platform_driver_probe(&tahvo_driver, tahvo_probe);
}

static void __exit tahvo_exit(void)
{
	platform_driver_unregister(&tahvo_driver);
}

subsys_initcall(tahvo_init);
module_exit(tahvo_exit);

MODULE_DESCRIPTION("Tahvo ASIC control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
MODULE_AUTHOR("David Weinehall");
MODULE_AUTHOR("Mikko Ylinen");

