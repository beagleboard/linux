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

#include <linux/export.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/cbus.h>

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

/**
 * cbus_send_bit - sends one bit over the bus
 * @host: the host we're using
 * @bit: one bit of information to send
 * @input: whether to set data pin as input after sending
 */
static int cbus_send_bit(struct cbus_host *host, unsigned bit,
		unsigned input)
{
	int ret = 0;

	gpio_set_value(host->dat_gpio, bit ? 1 : 0);
	gpio_set_value(host->clk_gpio, 1);

	/* The data bit is read on the rising edge of CLK */
	if (input)
		ret = gpio_direction_input(host->dat_gpio);

	gpio_set_value(host->clk_gpio, 0);

	return ret;
}

/**
 * cbus_send_data - sends @len amount of data over the bus
 * @host: the host we're using
 * @data: the data to send
 * @len: size of the transfer
 * @input: whether to set data pin as input after sending
 */
static int cbus_send_data(struct cbus_host *host, unsigned data, unsigned len,
		unsigned input)
{
	int ret = 0;
	int i;

	for (i = len; i > 0; i--) {
		ret = cbus_send_bit(host, data & (1 << (i - 1)),
				input && (i == 1));
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

/**
 * cbus_receive_bit - receives one bit from the bus
 * @host: the host we're using
 */
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

/**
 * cbus_receive_data - receives @len data from the bus
 * @host: the host we're using
 * @len: the length of data to receive
 */
static int cbus_receive_data(struct cbus_host *host, unsigned len)
{
	int ret = 0;
	int i;

	for (i = 16; i > 0; i--) {
		int bit = cbus_receive_bit(host);

		if (bit < 0)
			goto out;

		if (bit)
			ret |= 1 << (i - 1);
	}

out:
	return ret;
}

/**
 * cbus_transfer - transfers data over the bus
 * @host: the host we're using
 * @rw: read/write flag
 * @dev: device address
 * @reg: register address
 * @data: if @rw == 0 data to send otherwise 0
 */
static int cbus_transfer(struct cbus_host *host, unsigned rw, unsigned dev,
		unsigned reg, unsigned data)
{
	unsigned long flags;
	int input = 0;
	int ret = 0;

	/* We don't want interrupts disturbing our transfer */
	spin_lock_irqsave(&host->lock, flags);

	/* Reset state and start of transfer, SEL stays down during transfer */
	gpio_set_value(host->sel_gpio, 0);

	/* Set the DAT pin to output */
	gpio_direction_output(host->dat_gpio, 1);

	/* Send the device address */
	ret = cbus_send_data(host, dev, 3, 0);
	if (ret < 0) {
		dev_dbg(host->dev, "failed sending device addr\n");
		goto out;
	}

	/* Send the rw flag */
	ret = cbus_send_bit(host, rw, 0);
	if (ret < 0) {
		dev_dbg(host->dev, "failed sending read/write flag\n");
		goto out;
	}

	/* Send the register address */
	if (rw)
		input = true;

	ret = cbus_send_data(host, reg, 5, input);
	if (ret < 0) {
		dev_dbg(host->dev, "failed sending register addr\n");
		goto out;
	}

	if (!rw) {
		ret = cbus_send_data(host, data, 16, 0);
		if (ret < 0) {
			dev_dbg(host->dev, "failed sending data\n");
			goto out;
		}
	} else {
		gpio_set_value(host->clk_gpio, 1);

		ret = cbus_receive_data(host, 16);
		if (ret < 0) {
			dev_dbg(host->dev, "failed receiving data\n");
			goto out;
		}
	}

	/* Indicate end of transfer, SEL goes up until next transfer */
	gpio_set_value(host->sel_gpio, 1);
	gpio_set_value(host->clk_gpio, 1);
	gpio_set_value(host->clk_gpio, 0);

out:
	spin_unlock_irqrestore(&host->lock, flags);

	return ret;
}

/**
 * cbus_read_reg - reads a given register from the device
 * @child: the child device
 * @dev: device address
 * @reg: register address
 */
int cbus_read_reg(struct device *child, unsigned dev, unsigned reg)
{
	struct cbus_host	*host = dev_get_drvdata(child->parent);

	return cbus_transfer(host, CBUS_XFER_READ, dev, reg, 0);
}
EXPORT_SYMBOL(cbus_read_reg);

/**
 * cbus_write_reg - writes to a given register of the device
 * @child: the child device
 * @dev: device address
 * @reg: register address
 * @val: data to be written to @reg
 */
int cbus_write_reg(struct device *child, unsigned dev, unsigned reg,
		unsigned val)
{
	struct cbus_host	*host = dev_get_drvdata(child->parent);

	return cbus_transfer(host, CBUS_XFER_WRITE, dev, reg, val);
}
EXPORT_SYMBOL(cbus_write_reg);

static int __devinit cbus_bus_probe(struct platform_device *pdev)
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

	return 0;
exit3:
	gpio_free(chost->dat_gpio);
exit2:
	gpio_free(chost->clk_gpio);
exit1:
	kfree(chost);

	return ret;
}

static int __devexit cbus_bus_remove(struct platform_device *pdev)
{
	struct cbus_host	*chost = platform_get_drvdata(pdev);

	gpio_free(chost->sel_gpio);
	gpio_free(chost->dat_gpio);
	gpio_free(chost->clk_gpio);

	kfree(chost);

	return 0;
}

static struct platform_driver cbus_driver = {
	.probe		= cbus_bus_probe,
	.remove		= __devexit_p(cbus_bus_remove),
	.driver		= {
		.name	= "cbus",
	},
};

module_platform_driver(cbus_driver);

MODULE_DESCRIPTION("CBUS serial protocol");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
MODULE_AUTHOR("David Weinehall");
MODULE_AUTHOR("Mikko Ylinen");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

