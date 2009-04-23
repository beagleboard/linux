/*
 * drivers/cbus/cbus.c
 *
 * Support functions for CBUS serial protocol
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>

#include <mach/board.h>
#include <mach/board-nokia.h>

#include <asm/io.h>

#include "cbus.h"

struct cbus_host *cbus_host = NULL;

#ifdef CONFIG_ARCH_OMAP1
/* We use our own MPUIO functions to get closer to 1MHz bus speed */

static inline void cbus_set_gpio_direction(u32 base, int mpuio, int is_input)
{
	u16 w;

	mpuio &= 0x0f;
	w = __raw_readw(base + OMAP_MPUIO_IO_CNTL);
	if (is_input)
		w |= 1 << mpuio;
	else
		w &= ~(1 << mpuio);
	__raw_writew(w, base + OMAP_MPUIO_IO_CNTL);

}

static inline void cbus_set_gpio_dataout(u32 base, int mpuio, int enable)
{
	u16 w;

	mpuio &= 0x0f;
	w = __raw_readw(base + OMAP_MPUIO_OUTPUT);
	if (enable)
		w |= 1 << mpuio;
	else
		w &= ~(1 << mpuio);
	__raw_writew(w, base + OMAP_MPUIO_OUTPUT);
}

static inline int cbus_get_gpio_datain(u32 base, int mpuio)
{
	mpuio &= 0x0f;

	return (__raw_readw(base + OMAP_MPUIO_INPUT_LATCH) & (1 << mpuio)) != 0;
}

static void cbus_send_bit(struct cbus_host *host, u32 base, int bit,
			  int set_to_input)
{
	cbus_set_gpio_dataout(base, host->dat_gpio, bit ? 1 : 0);
	cbus_set_gpio_dataout(base, host->clk_gpio, 1);

	/* The data bit is read on the rising edge of CLK */
	if (set_to_input)
		cbus_set_gpio_direction(base, host->dat_gpio, 1);

	cbus_set_gpio_dataout(base, host->clk_gpio, 0);
}

static u8 cbus_receive_bit(struct cbus_host *host, u32 base)
{
	u8 ret;

	cbus_set_gpio_dataout(base, host->clk_gpio, 1);
	ret = cbus_get_gpio_datain(base, host->dat_gpio);
	cbus_set_gpio_dataout(base, host->clk_gpio, 0);

	return ret;
}

#define cbus_output(base, gpio, val)	cbus_set_gpio_direction(base, gpio, 0)

#else

#define cbus_output(base, gpio, val)	gpio_direction_output(gpio, val)
#define cbus_set_gpio_dataout(base, gpio, enable) gpio_set_value(gpio, enable)
#define cbus_get_gpio_datain(base, int, gpio) gpio_get_value(gpio)

static void _cbus_send_bit(struct cbus_host *host, int bit, int set_to_input)
{
	gpio_set_value(host->dat_gpio, bit ? 1 : 0);
	gpio_set_value(host->clk_gpio, 1);

	/* The data bit is read on the rising edge of CLK */
	if (set_to_input)
		gpio_direction_input(host->dat_gpio);

	gpio_set_value(host->clk_gpio, 0);
}

static u8 _cbus_receive_bit(struct cbus_host *host)
{
	u8 ret;

	gpio_set_value(host->clk_gpio, 1);
	ret = gpio_get_value(host->dat_gpio);
	gpio_set_value(host->clk_gpio, 0);

	return ret;
}

#define cbus_send_bit(host, base, bit, set_to_input) _cbus_send_bit(host, bit, set_to_input)
#define cbus_receive_bit(host, base) _cbus_receive_bit(host)

#endif

static int cbus_transfer(struct cbus_host *host, int dev, int reg, int data)
{
	int i;
	int is_read = 0;
	unsigned long flags;
	u32 base;

#ifdef CONFIG_ARCH_OMAP1
	base = OMAP1_IO_ADDRESS(OMAP_MPUIO_BASE);
#else
	base = 0;
#endif

	if (data < 0)
		is_read = 1;

	/* We don't want interrupts disturbing our transfer */
	spin_lock_irqsave(&host->lock, flags);

	/* Reset state and start of transfer, SEL stays down during transfer */
	cbus_set_gpio_dataout(base, host->sel_gpio, 0);

	/* Set the DAT pin to output */
	cbus_output(base, host->dat_gpio, 1);

	/* Send the device address */
	for (i = 3; i > 0; i--)
		cbus_send_bit(host, base, dev & (1 << (i - 1)), 0);

	/* Send the rw flag */
	cbus_send_bit(host, base, is_read, 0);

	/* Send the register address */
	for (i = 5; i > 0; i--) {
		int set_to_input = 0;

		if (is_read && i == 1)
			set_to_input = 1;

		cbus_send_bit(host, base, reg & (1 << (i - 1)), set_to_input);
	}

	if (!is_read) {
		for (i = 16; i > 0; i--)
			cbus_send_bit(host, base, data & (1 << (i - 1)), 0);
	} else {
		cbus_set_gpio_dataout(base, host->clk_gpio, 1);
		data = 0;

		for (i = 16; i > 0; i--) {
			u8 bit = cbus_receive_bit(host, base);

			if (bit)
				data |= 1 << (i - 1);
		}
	}

	/* Indicate end of transfer, SEL goes up until next transfer */
	cbus_set_gpio_dataout(base, host->sel_gpio, 1);
	cbus_set_gpio_dataout(base, host->clk_gpio, 1);
	cbus_set_gpio_dataout(base, host->clk_gpio, 0);

	spin_unlock_irqrestore(&host->lock, flags);

	return is_read ? data : 0;
}

/*
 * Read a given register from the device
 */
int cbus_read_reg(struct cbus_host *host, int dev, int reg)
{
	return cbus_host ? cbus_transfer(host, dev, reg, -1) : -ENODEV;
}

/*
 * Write to a given register of the device
 */
int cbus_write_reg(struct cbus_host *host, int dev, int reg, u16 val)
{
	return cbus_host ? cbus_transfer(host, dev, reg, (int)val) : -ENODEV;
}

int __init cbus_bus_init(void)
{
	const struct omap_cbus_config * cbus_config;
	struct cbus_host *chost;
	int ret;

	chost = kmalloc(sizeof (*chost), GFP_KERNEL);
	if (chost == NULL)
		return -ENOMEM;

	memset(chost, 0, sizeof (*chost));

	spin_lock_init(&chost->lock);

	cbus_config = omap_get_config(OMAP_TAG_CBUS, struct omap_cbus_config);

	if (cbus_config == NULL) {
		printk(KERN_ERR "cbus: Unable to retrieve config data\n");
		return -ENODATA;
	}

	chost->clk_gpio = cbus_config->clk_gpio;
	chost->dat_gpio = cbus_config->dat_gpio;
	chost->sel_gpio = cbus_config->sel_gpio;

#ifdef CONFIG_ARCH_OMAP1
	if (!OMAP_GPIO_IS_MPUIO(chost->clk_gpio) ||
	    !OMAP_GPIO_IS_MPUIO(chost->dat_gpio) ||
	    !OMAP_GPIO_IS_MPUIO(chost->sel_gpio)) {
		printk(KERN_ERR "cbus: Only MPUIO pins supported\n");
		ret = -ENODEV;
		goto exit1;
	}
#endif

	if ((ret = gpio_request(chost->clk_gpio, "CBUS clk")) < 0)
		goto exit1;

	if ((ret = gpio_request(chost->dat_gpio, "CBUS data")) < 0)
		goto exit2;

	if ((ret = gpio_request(chost->sel_gpio, "CBUS sel")) < 0)
		goto exit3;

	gpio_direction_output(chost->clk_gpio, 0);
	gpio_direction_input(chost->dat_gpio);
	gpio_direction_output(chost->sel_gpio, 1);

	gpio_set_value(chost->clk_gpio, 1);
	gpio_set_value(chost->clk_gpio, 0);

	cbus_host = chost;

	return 0;
exit3:
	gpio_free(chost->dat_gpio);
exit2:
	gpio_free(chost->clk_gpio);
exit1:
	kfree(chost);
	return ret;
}

subsys_initcall(cbus_bus_init);

EXPORT_SYMBOL(cbus_host);
EXPORT_SYMBOL(cbus_read_reg);
EXPORT_SYMBOL(cbus_write_reg);

MODULE_DESCRIPTION("CBUS serial protocol");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä, David Weinehall, and Mikko Ylinen");
