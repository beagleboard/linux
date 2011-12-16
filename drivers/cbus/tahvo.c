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
#include <linux/platform_data/cbus.h>
#include <linux/mutex.h>

#include "cbus.h"
#include "tahvo.h"

struct tahvo {
	/* device lock */
	struct mutex	mutex;
	struct device	*dev;

	struct irq_chip	irq_chip;

	int		irq_base;
	int		irq_end;
	int		irq;

	int		mask;

	unsigned int	mask_pending:1;
	unsigned int	is_betty:1;
};

/**
 * __tahvo_read_reg - Reads a value from a register in Tahvo
 * @tahvo: pointer to tahvo structure
 * @reg: the register address to read from
 */
static int __tahvo_read_reg(struct tahvo *tahvo, unsigned reg)
{
	return cbus_read_reg(tahvo->dev, CBUS_TAHVO_DEVICE_ID, reg);
}

/**
 * __tahvo_write_reg - Writes a value to a register in Tahvo
 * @tahvo: pointer to tahvo structure
 * @reg: register address to write to
 * @val: the value to be written to @reg
 */
static void __tahvo_write_reg(struct tahvo *tahvo, unsigned reg, u16 val)
{
	cbus_write_reg(tahvo->dev, CBUS_TAHVO_DEVICE_ID, reg, val);
}

/**
 * tahvo_read_reg - Read a value from a register in Tahvo
 * @child: device pointer from the calling child
 * @reg: the register to read from
 *
 * This function returns the contents of the specified register
 */
int tahvo_read_reg(struct device *child, unsigned reg)
{
	struct tahvo		*tahvo = dev_get_drvdata(child->parent);

	return __tahvo_read_reg(tahvo, reg);
}
EXPORT_SYMBOL(tahvo_read_reg);

/**
 * tahvo_write_reg - Write a value to a register in Tahvo
 * @child: device pointer from the calling child
 * @reg: the register to write to
 * @val : the value to write to the register
 *
 * This function writes a value to the specified register
 */
void tahvo_write_reg(struct device *child, unsigned reg, u16 val)
{
	struct tahvo		*tahvo = dev_get_drvdata(child->parent);

	mutex_lock(&tahvo->mutex);
	__tahvo_write_reg(tahvo, reg, val);
	mutex_unlock(&tahvo->mutex);
}
EXPORT_SYMBOL(tahvo_write_reg);

/**
 * tahvo_set_clear_reg_bits - set and clear register bits atomically
 * @child: device pointer from the calling child
 * @reg: the register to write to
 * @bits: the bits to set
 *
 * This function sets and clears the specified Tahvo register bits atomically
 */
void tahvo_set_clear_reg_bits(struct device *child, unsigned reg, u16 set,
		u16 clear)
{
	struct tahvo		*tahvo = dev_get_drvdata(child->parent);
	u16			w;

	mutex_lock(&tahvo->mutex);
	w = __tahvo_read_reg(tahvo, reg);
	w &= ~clear;
	w |= set;
	__tahvo_write_reg(tahvo, reg, w);
	mutex_unlock(&tahvo->mutex);
}

static irqreturn_t tahvo_irq_handler(int irq, void *_tahvo)
{
	struct tahvo		*tahvo = _tahvo;
	u16			id;
	u16			im;

	mutex_lock(&tahvo->mutex);
	id = __tahvo_read_reg(tahvo, TAHVO_REG_IDR);
	im = __tahvo_read_reg(tahvo, TAHVO_REG_IMR);
	id &= ~im;
	__tahvo_write_reg(tahvo, TAHVO_REG_IDR, id);
	mutex_unlock(&tahvo->mutex);

	if (!id) {
		dev_vdbg(tahvo->dev, "No IRQ, spurious ?\n");
		return IRQ_NONE;
	}

	while (id) {
		unsigned long	pending = __ffs(id);
		unsigned int	irq;

		id &= ~BIT(pending);
		irq = pending + tahvo->irq_base;
		handle_nested_irq(irq);
	}

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------- */

static void tahvo_irq_bus_lock(struct irq_data *data)
{
	struct tahvo		*tahvo = irq_data_get_irq_chip_data(data);

	mutex_lock(&tahvo->mutex);
}

static void tahvo_irq_bus_sync_unlock(struct irq_data *data)
{
	struct tahvo		*tahvo = irq_data_get_irq_chip_data(data);

	if (tahvo->mask_pending) {
		__tahvo_write_reg(tahvo, TAHVO_REG_IMR, tahvo->mask);
		tahvo->mask_pending = false;
	}

	mutex_unlock(&tahvo->mutex);
}

static void tahvo_irq_mask(struct irq_data *data)
{
	struct tahvo		*tahvo = irq_data_get_irq_chip_data(data);
	int			irq = data->irq;

	tahvo->mask |= (1 << (irq - tahvo->irq_base));
	tahvo->mask_pending = true;
}

static void tahvo_irq_unmask(struct irq_data *data)
{
	struct tahvo		*tahvo = irq_data_get_irq_chip_data(data);
	int			irq = data->irq;

	tahvo->mask &= ~(1 << (irq - tahvo->irq_base));
	tahvo->mask_pending = true;
}

static inline void tahvo_irq_setup(int irq)
{
#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif
}

static void tahvo_irq_init(struct tahvo *tahvo)
{
	int			base = tahvo->irq_base;
	int			end = tahvo->irq_end;
	int			irq;

	for (irq = base; irq < end; irq++) {
		irq_set_chip_data(irq, tahvo);
		irq_set_chip(irq, &tahvo->irq_chip);
		irq_set_nested_thread(irq, 1);
		tahvo_irq_setup(irq);
	}
}

/* -------------------------------------------------------------------------- */

static struct resource generic_resources[] = {
	{
		.start		= -EINVAL,	/* fixed later */
		.flags		= IORESOURCE_IRQ,
	},
};

static struct device *tahvo_allocate_child(const char *name,
		struct device *parent, int irq)
{
	struct platform_device	*pdev;
	int			ret;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		dev_dbg(parent, "can't allocate %s\n", name);
		goto err0;
	}

	pdev->dev.parent = parent;

	if (irq > 0) {
		generic_resources[0].start = irq;

		ret = platform_device_add_resources(pdev, generic_resources,
				ARRAY_SIZE(generic_resources));
		if (ret < 0) {
			dev_dbg(parent, "can't add resources to %s\n", name);
			goto err1;
		}
	}

	ret = platform_device_add(pdev);
	if (ret < 0) {
		dev_dbg(parent, "can't add %s\n", name);
		goto err1;
	}

	return &pdev->dev;

err1:
	platform_device_put(pdev);

err0:
	return NULL;
}

static int tahvo_allocate_children(struct device *parent, int irq_base)
{
	struct device		*child;

	child = tahvo_allocate_child("tahvo-usb", parent,
			irq_base + TAHVO_INT_VBUSON);
	if (!child)
		return -ENOMEM;

	child = tahvo_allocate_child("tahvo-pwm", parent, -1);
	if (!child)
		return -ENOMEM;

	return 0;
}

static int __devinit tahvo_probe(struct platform_device *pdev)
{
	struct irq_chip		*chip;
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

	irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, tahvo);

	mutex_init(&tahvo->mutex);

	ret = irq_alloc_descs(-1, 0, MAX_TAHVO_IRQ_HANDLERS, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to allocate IRQ descs\n");
		goto err1;
	}

	chip = &tahvo->irq_chip;

	chip->name	= "tahvo",
	chip->irq_bus_lock = tahvo_irq_bus_lock,
	chip->irq_bus_sync_unlock = tahvo_irq_bus_sync_unlock,
	chip->irq_mask	= tahvo_irq_mask,
	chip->irq_unmask = tahvo_irq_unmask,

	tahvo->irq_base	= ret;
	tahvo->irq_end	= ret + MAX_TAHVO_IRQ_HANDLERS;
	tahvo->dev	= &pdev->dev;
	tahvo->irq	= irq;

	tahvo_irq_init(tahvo);

	rev = __tahvo_read_reg(tahvo, TAHVO_REG_ASICR);

	id = (rev >> 8) & 0xff;

	if (id == 0x0b)
		tahvo->is_betty = true;

	ret = tahvo_allocate_children(&pdev->dev, tahvo->irq_base);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to allocate children\n");
		goto err2;
	}

	dev_err(&pdev->dev, "%s v%d.%d found\n",
			tahvo->is_betty ? "Betty" : "Tahvo",
			(rev >> 4) & 0x0f, rev & 0x0f);

	/* Mask all TAHVO interrupts */
	tahvo->mask = 0xffff;
	__tahvo_write_reg(tahvo, TAHVO_REG_IMR, tahvo->mask);

	ret = request_threaded_irq(irq, NULL, tahvo_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"tahvo", tahvo);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register IRQ handler\n");
		goto err2;
	}

	return 0;

err2:
	irq_free_descs(tahvo->irq_base, MAX_TAHVO_IRQ_HANDLERS);

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

	free_irq(irq, 0);
	irq_free_descs(tahvo->irq_base, MAX_TAHVO_IRQ_HANDLERS);
	kfree(tahvo);

	return 0;
}


static const struct of_device_id tahvo_match_table[] __devinitconst = {
	{
		.compatible = "nokia,tahvo",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tahvo_match);

static struct platform_driver tahvo_driver = {
	.probe		= tahvo_probe,
	.remove		= __devexit_p(tahvo_remove),
	.driver		= {
		.name	= "tahvo",
		.of_match_table = tahvo_match_table,
	},
};

module_platform_driver(tahvo_driver);

MODULE_ALIAS("platform:tahvo");
MODULE_DESCRIPTION("Tahvo ASIC control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
MODULE_AUTHOR("David Weinehall");
MODULE_AUTHOR("Mikko Ylinen");

