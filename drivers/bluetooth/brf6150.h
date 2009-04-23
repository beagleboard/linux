/*
 *  linux/drivers/bluetooth/brf6150/brf6150.h
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Ville Tervo <ville.tervo@nokia.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. 
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <mach/board.h>

#ifndef __DRIVERS_BLUETOOTH_BRF6150_H
#define __DRIVERS_BLUETOOTH_BRF6150_H

#define UART_SYSC_OMAP_RESET	0x02
#define UART_SYSS_RESETDONE	0x01
#define UART_OMAP_SCR_EMPTY_THR	0x08
#define UART_OMAP_SCR_WAKEUP	0x10
#define UART_OMAP_SSR_WAKEUP	0x02
#define UART_OMAP_SSR_TXFULL	0x01

struct brf6150_info {
	struct hci_dev *hdev;
	spinlock_t lock;

	struct clk *uart_ck;
	unsigned long uart_base;
	unsigned int irq;

	struct sk_buff_head txq;
	struct sk_buff *rx_skb;
	const struct omap_bluetooth_config *btinfo;
	const struct firmware *fw_entry;
	int fw_pos;
	int fw_error;
	struct completion fw_completion;
	struct completion init_completion;
	struct tasklet_struct tx_task;
	long rx_count;
	unsigned long garbage_bytes;
	unsigned long rx_state;
	int pm_enabled;
	int rx_pm_enabled;
	int tx_pm_enabled;
	struct timer_list pm_timer;
};

#define BT_DEVICE "nokia_btuart"
#define BT_DRIVER "nokia_btuart"

#define MAX_BAUD_RATE		921600
#define UART_CLOCK		48000000
#define BT_INIT_DIVIDER		320
#define BT_BAUDRATE_DIVIDER	384000000
#define BT_SYSCLK_DIV		1000
#define INIT_SPEED		120000

#define H4_TYPE_SIZE		1

/* H4+ packet types */
#define H4_CMD_PKT		0x01
#define H4_ACL_PKT		0x02
#define H4_SCO_PKT		0x03
#define H4_EVT_PKT		0x04
#define H4_NEG_PKT		0x06
#define H4_ALIVE_PKT		0x07

/* TX states */
#define WAIT_FOR_PKT_TYPE	1
#define WAIT_FOR_HEADER		2
#define WAIT_FOR_DATA		3

struct hci_fw_event {
	struct hci_event_hdr hev;
	struct hci_ev_cmd_complete cmd;
	__u8 status;
} __attribute__ ((packed));

#endif /* __DRIVERS_BLUETOOTH_BRF6150_H */
