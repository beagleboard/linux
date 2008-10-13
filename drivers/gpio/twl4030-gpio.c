/*
 * twl4030_gpio.c -- access to GPIOs on TWL4030/TPS659x0 chips
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Code re-arranged and cleaned up by:
 *	Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * Initial Code:
 *	Andy Lowe / Nishanth Menon
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/i2c/twl4030.h>


/*
 * The GPIO "subchip" supports 18 GPIOs which can be configured as
 * inputs or outputs, with pullups or pulldowns on each pin.  Each
 * GPIO can trigger interrupts on either or both edges.
 *
 * GPIO interrupts can be fed to either of two IRQ lines; this is
 * intended to support multiple hosts.
 *
 * There are also two LED pins used sometimes as output-only GPIOs.
 *
 * FIXME code currently only handles the first IRQ line.
 */


static struct gpio_chip twl_gpiochip;
static int twl4030_gpio_irq_base;

/* genirq interfaces are not available to modules */
#ifdef MODULE
#define is_module()	true
#else
#define is_module()	false
#endif

/* GPIO_CTRL Fields */
#define MASK_GPIO_CTRL_GPIO0CD1		BIT(0)
#define MASK_GPIO_CTRL_GPIO1CD2		BIT(1)
#define MASK_GPIO_CTRL_GPIO_ON		BIT(2)

/* Mask for GPIO registers when aggregated into a 32-bit integer */
#define GPIO_32_MASK			0x0003ffff

/* Data structures */
static DEFINE_MUTEX(gpio_lock);

/* store usage of each GPIO. - each bit represents one GPIO */
static unsigned int gpio_usage_count;

/*----------------------------------------------------------------------*/

/*
 * To configure TWL4030 GPIO module registers
 */
static inline int gpio_twl4030_write(u8 address, u8 data)
{
	return twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, address);
}

/*
 * To read a TWL4030 GPIO module register
 */
static inline int gpio_twl4030_read(u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, address);
	return (ret < 0) ? ret : data;
}

/*
 * twl4030 GPIO request function
 */
int twl4030_request_gpio(int gpio)
{
	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	return gpio_request(twl_gpiochip.base + gpio, NULL);
}
EXPORT_SYMBOL(twl4030_request_gpio);

/*
 * TWL4030 GPIO free module
 */
int twl4030_free_gpio(int gpio)
{
	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	gpio_free(twl_gpiochip.base + gpio);
	return 0;
}
EXPORT_SYMBOL(twl4030_free_gpio);

static int twl4030_set_gpio_direction(int gpio, int is_input)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 reg = 0;
	u8 base = REG_GPIODATADIR1 + d_bnk;
	int ret = 0;

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (is_input)
			reg = ret & ~d_msk;
		else
			reg = ret | d_msk;

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}

static int twl4030_set_gpio_dataout(int gpio, int enable)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 base = 0;

	if (enable)
		base = REG_SETGPIODATAOUT1 + d_bnk;
	else
		base = REG_CLEARGPIODATAOUT1 + d_bnk;

	return gpio_twl4030_write(base, d_msk);
}

int twl4030_get_gpio_datain(int gpio)
{
	u8 d_bnk = gpio >> 3;
	u8 d_off = gpio & 0x7;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))))
		return -EPERM;

	base = REG_GPIODATAIN1 + d_bnk;
	ret = gpio_twl4030_read(base);
	if (ret > 0)
		ret = (ret >> d_off) & 0x1;

	return ret;
}
EXPORT_SYMBOL(twl4030_get_gpio_datain);

/*
 * Configure debounce timing value for a GPIO pin on TWL4030
 */
int twl4030_set_gpio_debounce(int gpio, int enable)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))))
		return -EPERM;

	base = REG_GPIO_DEBEN1 + d_bnk;
	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (enable)
			reg = ret | d_msk;
		else
			reg = ret & ~d_msk;

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_set_gpio_debounce);

#if 0
/*
 * Configure Card detect for GPIO pin on TWL4030
 *
 * This means:  VMMC1 or VMMC2 is enabled or disabled based
 * on the status of GPIO-0 or GPIO-1 pins (respectively).
 */
int twl4030_set_gpio_card_detect(int gpio, int enable)
{
	u8 reg = 0;
	u8 msk = (1 << gpio);
	int ret = 0;

	/* Only GPIO 0 or 1 can be used for CD feature.. */
	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))
		|| (gpio >= TWL4030_GPIO_MAX_CD))) {
		return -EPERM;
	}

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(REG_GPIO_CTRL);
	if (ret >= 0) {
		if (enable)
			reg = (u8) (ret | msk);
		else
			reg = (u8) (ret & ~msk);

		ret = gpio_twl4030_write(REG_GPIO_CTRL, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
#endif

/*----------------------------------------------------------------------*/

static int twl_request(struct gpio_chip *chip, unsigned offset)
{
	int status = 0;

	mutex_lock(&gpio_lock);

	/* on first use, turn GPIO module "on" */
	if (!gpio_usage_count)
		status = gpio_twl4030_write(REG_GPIO_CTRL,
				MASK_GPIO_CTRL_GPIO_ON);

	if (!status)
		gpio_usage_count |= (0x1 << offset);

done:
	mutex_unlock(&gpio_lock);
	return status;
}

static void twl_free(struct gpio_chip *chip, unsigned offset)
{
	mutex_lock(&gpio_lock);

	gpio_usage_count &= ~BIT(offset);

	/* on last use, switch off GPIO module */
	if (!gpio_usage_count)
		gpio_twl4030_write(REG_GPIO_CTRL, 0x0);

	mutex_unlock(&gpio_lock);
}

static int twl_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return twl4030_set_gpio_direction(offset, 1);
}

static int twl_get(struct gpio_chip *chip, unsigned offset)
{
	int status = twl4030_get_gpio_datain(offset);

	return (status < 0) ? 0 : status;
}

static int twl_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
	return twl4030_set_gpio_direction(offset, 0);
}

static void twl_set(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
}

static int twl_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return twl4030_gpio_irq_base
		? (twl4030_gpio_irq_base + offset)
		: -EINVAL;
}

static struct gpio_chip twl_gpiochip = {
	.label			= "twl4030",
	.owner			= THIS_MODULE,
	.request		= twl_request,
	.free			= twl_free,
	.direction_input	= twl_direction_in,
	.get			= twl_get,
	.direction_output	= twl_direction_out,
	.set			= twl_set,
	.to_irq			= twl_to_irq,
	.can_sleep		= 1,
};

/*----------------------------------------------------------------------*/

static int __devinit gpio_twl4030_pulls(u32 ups, u32 downs)
{
	u8		message[6];
	unsigned	i, gpio_bit;

	/* For most pins, a pulldown was enabled by default.
	 * We should have data that's specific to this board.
	 */
	for (gpio_bit = 1, i = 1; i < 6; i++) {
		u8		bit_mask;
		unsigned	j;

		for (bit_mask = 0, j = 0; j < 8; j += 2, gpio_bit <<= 1) {
			if (ups & gpio_bit)
				bit_mask |= 1 << (j + 1);
			else if (downs & gpio_bit)
				bit_mask |= 1 << (j + 0);
		}
		message[i] = bit_mask;
	}

	return twl4030_i2c_write(TWL4030_MODULE_GPIO, message,
				REG_GPIOPUPDCTR1, 5);
}

static int gpio_twl4030_remove(struct platform_device *pdev);

static int __devinit gpio_twl4030_probe(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	/* maybe setup IRQs */
	if (pdata->irq_base) {
		if (is_module()) {
			dev_err(&pdev->dev,
				"can't dispatch IRQs from modules\n");
			goto no_irqs;
		}
		ret = twl4030_sih_setup(TWL4030_MODULE_GPIO);
		if (ret < 0)
			return ret;
		WARN_ON(ret != pdata->irq_base);
		twl4030_gpio_irq_base = ret;
	}

no_irqs:
	/*
	 * NOTE:  boards may waste power if they don't set pullups
	 * and pulldowns correctly ... default for non-ULPI pins is
	 * pulldown, and some other pins may have external pullups
	 * or pulldowns.  Careful!
	 */
	ret = gpio_twl4030_pulls(pdata->pullups, pdata->pulldowns);
	if (ret)
		dev_dbg(&pdev->dev, "pullups %.05x %.05x --> %d\n",
				pdata->pullups, pdata->pulldowns,
				ret);

	twl_gpiochip.base = pdata->gpio_base;
	twl_gpiochip.ngpio = TWL4030_GPIO_MAX;
	twl_gpiochip.dev = &pdev->dev;

	ret = gpiochip_add(&twl_gpiochip);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"could not register gpiochip, %d\n",
				ret);
		twl_gpiochip.ngpio = 0;
		gpio_twl4030_remove(pdev);
	} else if (pdata->setup) {
		int status;

		status = pdata->setup(&pdev->dev,
				pdata->gpio_base, TWL4030_GPIO_MAX);
		if (status)
			dev_dbg(&pdev->dev, "setup --> %d\n", status);
	}

	return ret;
}

static int __devexit gpio_twl4030_remove(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int status;

	if (pdata->teardown) {
		status = pdata->teardown(&pdev->dev,
				pdata->gpio_base, TWL4030_GPIO_MAX);
		if (status) {
			dev_dbg(&pdev->dev, "teardown --> %d\n", status);
			return status;
		}
	}

	status = gpiochip_remove(&twl_gpiochip);
	if (status < 0)
		return status;

	if (is_module())
		return 0;

	/* REVISIT no support yet for deregistering all the IRQs */
	WARN_ON(1);
	return -EIO;
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:twl4030_gpio");

static struct platform_driver gpio_twl4030_driver = {
	.driver.name	= "twl4030_gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= gpio_twl4030_probe,
	.remove		= __devexit_p(gpio_twl4030_remove),
};

static int __init gpio_twl4030_init(void)
{
	return platform_driver_register(&gpio_twl4030_driver);
}
subsys_initcall(gpio_twl4030_init);

static void __exit gpio_twl4030_exit(void)
{
	platform_driver_unregister(&gpio_twl4030_driver);
}
module_exit(gpio_twl4030_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("GPIO interface for TWL4030");
MODULE_LICENSE("GPL");
