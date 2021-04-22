// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Serial Port driver for PRUSS UART on TI platforms
 *
 *  Copyright (C) 2020-2021 by Texas Instruments Incorporated - http://www.ti.com/
 *  Author: Bin Liu <b-liu@ti.com>
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pruss.h>
#include <linux/remoteproc.h>
#include "8250.h"

#define DEFAULT_CLK_SPEED	192000000

/* extra registers */
#define PRUSS_UART_PEREMU_MGMT	12
#define PRUSS_UART_TX_EN	BIT(14)
#define PRUSS_UART_RX_EN	BIT(13)
#define PRUSS_UART_FREE_RUN	BIT(0)

#define PRUSS_UART_MDR			13
#define PRUSS_UART_MDR_OSM_SEL_MASK	BIT(0)
#define PRUSS_UART_MDR_16X_MODE		0
#define PRUSS_UART_MDR_13X_MODE		1

struct pruss8250_info {
	int type;
	int line;
};

static inline void uart_writel(struct uart_port *p, u32 offset, int value)
{
	writel(value, p->membase + (offset << p->regshift));
}

static int pruss8250_startup(struct uart_port *port)
{
	int ret;

	uart_writel(port, PRUSS_UART_PEREMU_MGMT, 0);

	ret = serial8250_do_startup(port);
	if (!ret)
		uart_writel(port, PRUSS_UART_PEREMU_MGMT, PRUSS_UART_TX_EN |
							  PRUSS_UART_RX_EN |
							  PRUSS_UART_FREE_RUN);
	return ret;
}

static unsigned int pruss8250_get_divisor(struct uart_port *port,
					  unsigned int baud,
					  unsigned int *frac)
{
	unsigned int uartclk = port->uartclk;
	unsigned int div_13, div_16;
	unsigned int abs_d13, abs_d16;
	u16 quot;

	/* Old custom speed handling */
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST) {
		quot = port->custom_divisor & UART_DIV_MAX;
		if (port->custom_divisor & (1 << 16))
			*frac = PRUSS_UART_MDR_13X_MODE;
		else
			*frac = PRUSS_UART_MDR_16X_MODE;

		return quot;
	}

	div_13 = DIV_ROUND_CLOSEST(uartclk, 13 * baud);
	div_16 = DIV_ROUND_CLOSEST(uartclk, 16 * baud);
	div_13 = div_13 ? : 1;
	div_16 = div_16 ? : 1;

	abs_d13 = abs(baud - uartclk / 13 / div_13);
	abs_d16 = abs(baud - uartclk / 16 / div_16);

	if (abs_d13 >= abs_d16) {
		*frac = PRUSS_UART_MDR_16X_MODE;
		quot = div_16;
	} else {
		*frac = PRUSS_UART_MDR_13X_MODE;
		quot = div_13;
	}

	return quot;
}

static void pruss8250_set_divisor(struct uart_port *port, unsigned int baud,
				  unsigned int quot, unsigned int quot_frac)
{
	serial8250_do_set_divisor(port, baud, quot, quot_frac);
	/*
	 * quot_frac holds the MDR over-sampling mode
	 * which is set in pruss8250_get_divisor()
	 */
	quot_frac &= PRUSS_UART_MDR_OSM_SEL_MASK;
	serial_port_out(port, PRUSS_UART_MDR, quot_frac);
}

static int pruss8250_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct uart_8250_port port8250;
	struct uart_port *up = &port8250.port;
	struct pruss8250_info *info;
	struct resource resource;
	unsigned int port_type;
	struct clk *clk;
	int ret;

	port_type = (unsigned long)of_device_get_match_data(&pdev->dev);
	if (port_type == PORT_UNKNOWN)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	memset(&port8250, 0, sizeof(port8250));

	ret = of_address_to_resource(np, 0, &resource);
	if (ret) {
		dev_err(&pdev->dev, "invalid address\n");
		return ret;
	}

	ret = of_alias_get_id(np, "serial");
	if (ret > 0)
		up->line = ret;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		up->uartclk = DEFAULT_CLK_SPEED;
	} else {
		up->uartclk = clk_get_rate(clk);
		devm_clk_put(&pdev->dev, clk);
	}

	up->dev = &pdev->dev;
	up->mapbase = resource.start;
	up->mapsize = resource_size(&resource);
	up->type = port_type;
	up->iotype = UPIO_MEM;
	up->regshift = 2;
	up->flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF | UPF_FIXED_PORT |
		    UPF_FIXED_TYPE | UPF_IOREMAP;
	up->irqflags |= IRQF_SHARED;
	up->startup = pruss8250_startup;
	up->rs485_config = serial8250_em485_config;
	up->get_divisor = pruss8250_get_divisor;
	up->set_divisor = pruss8250_set_divisor;

	ret = of_irq_get(np, 0);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "missing irq\n");
		return ret;
	}

	up->irq = ret;
	spin_lock_init(&port8250.port.lock);
	port8250.capabilities = UART_CAP_FIFO | UART_CAP_AFE;

	ret = serial8250_register_8250_port(&port8250);
	if (ret < 0)
		goto err_dispose;

	info->type = port_type;
	info->line = ret;
	platform_set_drvdata(pdev, info);

	return 0;

err_dispose:
	irq_dispose_mapping(port8250.port.irq);
	return ret;
}

static int pruss8250_remove(struct platform_device *pdev)
{
	struct pruss8250_info *info = platform_get_drvdata(pdev);

	serial8250_unregister_port(info->line);
	return 0;
}

static const struct of_device_id pruss8250_table[] = {
	{ .compatible = "ti,pruss-uart", .data = (void *)PORT_16550A, },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, pruss8250_table);

static struct platform_driver pruss8250_driver = {
	.driver = {
		.name = "pruss8250",
		.of_match_table = pruss8250_table,
	},
	.probe = pruss8250_probe,
	.remove = pruss8250_remove,
};

module_platform_driver(pruss8250_driver);

MODULE_AUTHOR("Bin Liu <b-liu@ti.com");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Serial Port driver for PRUSS UART on TI platforms");
