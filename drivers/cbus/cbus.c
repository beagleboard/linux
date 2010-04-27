/*
 * drivers/cbus/cbus.c
 *
 * Support functions for CBUS serial protocol
 *
 * Copyright (C) 2004-2010 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>,
 *	      David Weinehall <david.weinehall@nokia.com>, and
 *	      Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * Several updates and cleanups by Felipe Balbi <felipe.balbi@nokia.com>
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <plat/board.h>
#include <plat/cbus.h>

#include "cbus.h"

#define CBUS_XFER_READ		1
#define CBUS_XFER_WRITE		0

struct cbus_host {
	/* host lock */
	spinlock_t	lock;

	struct device	*dev;

	int		clk_gpio;
	int		dat_gpio;
	int		sel_gpio;
};

static struct cbus_host *cbus_host;

static int cbus_send_bit(struct cbus_host *host, int bit, int set_to_input)
{
	int ret = 0;

	gpio_set_value(host->dat_gpio, bit ? 1 : 0);
	gpio_set_value(host->clk_gpio, 1);

	/* The data bit is read on the rising edge of CLK */
	if (set_to_input)
		ret = gpio_direction_input(host->dat_gpio);

	gpio_set_value(host->clk_gpio, 0);

	return ret;
}

static int cbus_receive_bit(struct cbus_host *host)
{
	int ret;

	gpio_set_value(host->clk_gpio, 1);
	ret = gpio_get_value(host->dat_gpio);
	if (ret < 0)
		goto out;
	gpio_set_value(host->clk_gpio, 0);

out:
	return ret;
}

static int cbus_transfer(struct cbus_host *host, unsigned rw, unsigned dev,
		unsigned reg, unsigned data)
{
	unsigned long flags;
	int ret = 0;
	int i;

	/* We don't want interrupts disturbing our transfer */
	spin_lock_irqsave(&host->lock, flags);

	/* Reset state and start of transfer, SEL stays down during transfer */
	gpio_set_value(host->sel_gpio, 0);

	/* Set the DAT pin to output */
	gpio_direction_output(host->dat_gpio, 1);

	/* Send the device address */
	for (i = 3; i > 0; i--) {
		ret = cbus_send_bit(host, dev & (1 << (i - 1)), 0);
		if (ret < 0) {
			dev_dbg(host->dev, "failed sending device addr\n");
			goto out;
		}
	}

	/* Send the rw flag */
	ret = cbus_send_bit(host, rw, 0);
	if (ret < 0) {
		dev_dbg(host->dev, "failed sending read/write flag\n");
		goto out;
	}

	/* Send the register address */
	for (i = 5; i > 0; i--) {
		int set_to_input = 0;

		if (rw && i == 1)
			set_to_input = 1;

		ret = cbus_send_bit(host, reg & (1 << (i - 1)), set_to_input);
		if (ret < 0) {
			dev_dbg(host->dev, "failed sending register addr\n");
			goto out;
		}
	}

	if (!rw) {
		for (i = 16; i > 0; i--) {
			ret = cbus_send_bit(host, data & (1 << (i - 1)), 0);
			if (ret < 0) {
				dev_dbg(host->dev, "failed sending data\n");
				goto out;
			}
		}
	} else {
		gpio_set_value(host->clk_gpio, 1);

		for (i = 16; i > 0; i--) {
			u8 bit = cbus_receive_bit(host);

			if (bit < 0) {
				dev_dbg(host->dev, "failed receiving data\n");
				goto out;
			}

			if (bit)
				data |= 1 << (i - 1);
		}

		/* return the data received */
		ret = data;
	}

	/* Indicate end of transfer, SEL goes up until next transfer */
	gpio_set_value(host->sel_gpio, 1);
	gpio_set_value(host->clk_gpio, 1);
	gpio_set_value(host->clk_gpio, 0);

out:
	spin_unlock_irqrestore(&host->lock, flags);

	return ret;
}

/*
 * Read a given register from the device
 */
int cbus_read_reg(unsigned dev, unsigned reg)
{
	return cbus_transfer(cbus_host, CBUS_XFER_READ, dev, reg, 0);
}
EXPORT_SYMBOL(cbus_read_reg);

/*
 * Write to a given register of the device
 */
int cbus_write_reg(unsigned dev, unsigned reg, unsigned val)
{
	return cbus_transfer(cbus_host, CBUS_XFER_WRITE, dev, reg, val);
}
EXPORT_SYMBOL(cbus_write_reg);

static int __init cbus_bus_probe(struct platform_device *pdev)
{
	struct cbus_host *chost;
	struct cbus_host_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	chost = kzalloc(sizeof(*chost), GFP_KERNEL);
	if (chost == NULL)
		return -ENOMEM;

	spin_lock_init(&chost->lock);

	chost->clk_gpio = pdata->clk_gpio;
	chost->dat_gpio = pdata->dat_gpio;
	chost->sel_gpio = pdata->sel_gpio;
	chost->dev = &pdev->dev;

	ret = gpio_request(chost->clk_gpio, "CBUS clk");
	if (ret < 0)
		goto exit1;

	ret = gpio_request(chost->dat_gpio, "CBUS data");
	if (ret < 0)
		goto exit2;

	ret = gpio_request(chost->sel_gpio, "CBUS sel");
	if (ret < 0)
		goto exit3;

	gpio_direction_output(chost->clk_gpio, 0);
	gpio_direction_input(chost->dat_gpio);
	gpio_direction_output(chost->sel_gpio, 1);

	gpio_set_value(chost->clk_gpio, 1);
	gpio_set_value(chost->clk_gpio, 0);

	platform_set_drvdata(pdev, chost);

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

static void __exit cbus_bus_remove(struct platform_device *pdev)
{
	struct cbus_host	*chost = platform_get_drvdata(pdev);

	gpio_free(chost->sel_gpio);
	gpio_free(chost->dat_gpio);
	gpio_free(chost->clk_gpio);

	kfree(chost);
	cbus_host = NULL;
}

static struct platform_driver cbus_driver = {
	.remove		= __exit_p(cbus_bus_remove),
	.driver		= {
		.name	= "cbus",
	},
};

static int __init cbus_bus_init(void)
{
	return platform_driver_probe(&cbus_driver, cbus_bus_probe);
}
subsys_initcall(cbus_bus_init);

static void __exit cbus_bus_exit(void)
{
	platform_driver_unregister(&cbus_driver);
}
module_exit(cbus_bus_exit);

MODULE_DESCRIPTION("CBUS serial protocol");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
MODULE_AUTHOR("David Weinehall");
MODULE_AUTHOR("Mikko Ylinen");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

