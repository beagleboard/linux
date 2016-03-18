/*
 * TI Keystone 16550 UART "driver"
 *
 * This isn't a full driver; it just provides for special initialization
 * that keystone UARTs need. Everything else is just using the standard
 * 8250 support.
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of_platform.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

#include "8250.h"

/*
 * Texas Instruments Keystone registers
 */
#define UART_KEYSTONE_PWREMU		0x0c		/* Power and Emulation */

/*
 * Keystone PWREMU register definitions
 */
#define UART_KEYSTONE_PWREMU_FREE	(1 << 0)	/* Free-running enable */
#define UART_KEYSTONE_PWREMU_URRST	(1 << 13)	/* Receiver reset and enable */
#define UART_KEYSTONE_PWREMU_UTRST	(1 << 14)	/* Transmitter reset and enable */

int keystone_serial8250_init(struct uart_port *port)
{
	unsigned long flags;

	if (!of_device_is_compatible(port->dev->of_node, "ti,keystone-uart"))
		return 0;

	spin_lock_irqsave(&port->lock, flags);

	serial_port_out(port, UART_KEYSTONE_PWREMU,
			 UART_KEYSTONE_PWREMU_FREE  |
			 UART_KEYSTONE_PWREMU_URRST |
			 UART_KEYSTONE_PWREMU_UTRST);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}
EXPORT_SYMBOL(keystone_serial8250_init);
