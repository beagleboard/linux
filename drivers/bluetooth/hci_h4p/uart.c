/*
 * This file is part of hci_h4p bluetooth driver
 *
 * Copyright (C) 2005, 2006 Nokia Corporation.
 *
 * Contact: Ville Tervo <ville.tervo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>

#include "hci_h4p.h"

inline void hci_h4p_outb(struct hci_h4p_info *info, unsigned int offset, u8 val)
{
	outb(val, info->uart_base + (offset << 2));
}

inline u8 hci_h4p_inb(struct hci_h4p_info *info, unsigned int offset)
{
	return inb(info->uart_base + (offset << 2));
}

void hci_h4p_set_rts(struct hci_h4p_info *info, int active)
{
	u8 b;

	b = hci_h4p_inb(info, UART_MCR);
	if (active)
		b |= UART_MCR_RTS;
	else
		b &= ~UART_MCR_RTS;
	hci_h4p_outb(info, UART_MCR, b);
}

int hci_h4p_wait_for_cts(struct hci_h4p_info *info, int active,
			 int timeout_ms)
{
	int okay;
	unsigned long timeout;

	okay = 0;
	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	for (;;) {
		int state;

		state = hci_h4p_inb(info, UART_MSR) & UART_MSR_CTS;
		if (active) {
			if (state)
				return 0;
		} else {
			if (!state)
				return 0;
		}
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}
}

void __hci_h4p_set_auto_ctsrts(struct hci_h4p_info *info, int on, u8 which)
{
	u8 lcr, b;

	lcr = hci_h4p_inb(info, UART_LCR);
	hci_h4p_outb(info, UART_LCR, 0xbf);
	b = hci_h4p_inb(info, UART_EFR);
	if (on)
		b |= which;
	else
		b &= ~which;
	hci_h4p_outb(info, UART_EFR, b);
	hci_h4p_outb(info, UART_LCR, lcr);
}

void hci_h4p_set_auto_ctsrts(struct hci_h4p_info *info, int on, u8 which)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	__hci_h4p_set_auto_ctsrts(info, on, which);
	spin_unlock_irqrestore(&info->lock, flags);
}

void hci_h4p_change_speed(struct hci_h4p_info *info, unsigned long speed)
{
	unsigned int divisor;
	u8 lcr, mdr1;

	NBT_DBG("Setting speed %lu\n", speed);

	if (speed >= 460800) {
		divisor = UART_CLOCK / 13 / speed;
		mdr1 = 3;
	} else {
		divisor = UART_CLOCK / 16 / speed;
		mdr1 = 0;
	}

	hci_h4p_outb(info, UART_OMAP_MDR1, 7); /* Make sure UART mode is disabled */
	lcr = hci_h4p_inb(info, UART_LCR);
	hci_h4p_outb(info, UART_LCR, UART_LCR_DLAB);     /* Set DLAB */
	hci_h4p_outb(info, UART_DLL, divisor & 0xff);    /* Set speed */
	hci_h4p_outb(info, UART_DLM, divisor >> 8);
	hci_h4p_outb(info, UART_LCR, lcr);
	hci_h4p_outb(info, UART_OMAP_MDR1, mdr1); /* Make sure UART mode is enabled */
}

int hci_h4p_reset_uart(struct hci_h4p_info *info)
{
	int count = 0;

	/* Reset the  UART */
	hci_h4p_outb(info, UART_OMAP_SYSC, UART_SYSC_OMAP_RESET);
	while (!(hci_h4p_inb(info, UART_OMAP_SYSS) & UART_SYSS_RESETDONE)) {
		if (count++ > 100) {
			dev_err(info->dev, "hci_h4p: UART reset timeout\n");
			return -ENODEV;
		}
		udelay(1);
	}

	return 0;
}

int hci_h4p_init_uart(struct hci_h4p_info *info)
{
	int err;

	err = hci_h4p_reset_uart(info);
	if (err < 0)
		return err;

	/* Enable and setup FIFO */
	hci_h4p_outb(info, UART_LCR, UART_LCR_WLEN8);
	hci_h4p_outb(info, UART_OMAP_MDR1, 0x00); /* Make sure UART mode is enabled */
	hci_h4p_outb(info, UART_OMAP_SCR, 0x80);
	hci_h4p_outb(info, UART_EFR, UART_EFR_ECB);
	hci_h4p_outb(info, UART_MCR, UART_MCR_TCRTLR);
	hci_h4p_outb(info, UART_TI752_TLR, 0x1f);
	hci_h4p_outb(info, UART_TI752_TCR, 0xef);
	hci_h4p_outb(info, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR |
		     UART_FCR_CLEAR_XMIT | UART_FCR_R_TRIG_00);
	hci_h4p_outb(info, UART_IER, UART_IER_RDI);

	return 0;
}
