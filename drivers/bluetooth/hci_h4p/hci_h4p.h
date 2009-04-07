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

#include <mach/board.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

#ifndef __DRIVERS_BLUETOOTH_HCI_H4P_H
#define __DRIVERS_BLUETOOTH_HCI_H4P_H

#define UART_SYSC_OMAP_RESET	0x03
#define UART_SYSS_RESETDONE	0x01
#define UART_OMAP_SCR_EMPTY_THR	0x08
#define UART_OMAP_SCR_WAKEUP	0x10
#define UART_OMAP_SSR_WAKEUP	0x02
#define UART_OMAP_SSR_TXFULL	0x01

#if 0
#define NBT_DBG(fmt, arg...)  printk("%s: " fmt "" , __FUNCTION__ , ## arg)
#else
#define NBT_DBG(...)
#endif

#if 0
#define NBT_DBG_FW(fmt, arg...)  printk("%s: " fmt "" , __FUNCTION__ , ## arg)
#else
#define NBT_DBG_FW(...)
#endif

#if 0
#define NBT_DBG_POWER(fmt, arg...)  printk("%s: " fmt "" , __FUNCTION__ , ## arg)
#else
#define NBT_DBG_POWER(...)
#endif

#if 0
#define NBT_DBG_TRANSFER(fmt, arg...)  printk("%s: " fmt "" , __FUNCTION__ , ## arg)
#else
#define NBT_DBG_TRANSFER(...)
#endif

#if 0
#define NBT_DBG_TRANSFER_NF(fmt, arg...)  printk(fmt "" , ## arg)
#else
#define NBT_DBG_TRANSFER_NF(...)
#endif

#if 0
#define NBT_DBG_DMA(fmt, arg...)  printk("%s: " fmt "" , __FUNCTION__ , ## arg)
#else
#define NBT_DBG_DMA(...)
#endif

struct hci_h4p_info {
	struct hci_dev *hdev;
	spinlock_t lock;

	void __iomem *uart_base;
	unsigned long uart_phys_base;
	int irq;
	struct device *dev;
	u8 bdaddr[6];
	u8 chip_type;
	u8 bt_wakeup_gpio;
	u8 host_wakeup_gpio;
	u8 reset_gpio;
	u8 bt_sysclk;


	struct sk_buff_head fw_queue;
	struct sk_buff *alive_cmd_skb;
	struct completion init_completion;
	struct completion fw_completion;
	int fw_error;
	int init_error;

	struct sk_buff_head txq;
	struct tasklet_struct tx_task;

	struct sk_buff *rx_skb;
	long rx_count;
	unsigned long rx_state;
	unsigned long garbage_bytes;
	struct tasklet_struct rx_task;

	int pm_enabled;
	int tx_pm_enabled;
	int rx_pm_enabled;
	struct timer_list tx_pm_timer;
	struct timer_list rx_pm_timer;

	int tx_clocks_en;
	int rx_clocks_en;
	spinlock_t clocks_lock;
	struct clk *uart_iclk;
	struct clk *uart_fclk;
};

#define MAX_BAUD_RATE		921600
#define BC4_MAX_BAUD_RATE	3692300
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
	u8 status;
} __attribute__ ((packed));

struct hci_bc4_set_bdaddr {
	u8 type;
	struct hci_command_hdr cmd_hdr;
} __attribute__ ((packed));

int hci_h4p_send_alive_packet(struct hci_h4p_info *info);

void hci_h4p_bc4_parse_fw_event(struct hci_h4p_info *info,
				struct sk_buff *skb);
int hci_h4p_bc4_send_fw(struct hci_h4p_info *info,
			struct sk_buff_head *fw_queue);

void hci_h4p_brf6150_parse_fw_event(struct hci_h4p_info *info,
				    struct sk_buff *skb);
int hci_h4p_brf6150_send_fw(struct hci_h4p_info *info,
			    struct sk_buff_head *fw_queue);

int hci_h4p_read_fw(struct hci_h4p_info *info, struct sk_buff_head *fw_queue);
int hci_h4p_send_fw(struct hci_h4p_info *info, struct sk_buff_head *fw_queue);
void hci_h4p_parse_fw_event(struct hci_h4p_info *info, struct sk_buff *skb);

int hci_h4p_sysfs_create_files(struct device *dev);

void hci_h4p_outb(struct hci_h4p_info *info, unsigned int offset, u8 val);
u8 hci_h4p_inb(struct hci_h4p_info *info, unsigned int offset);
void hci_h4p_set_rts(struct hci_h4p_info *info, int active);
int hci_h4p_wait_for_cts(struct hci_h4p_info *info, int active, int timeout_ms);
void __hci_h4p_set_auto_ctsrts(struct hci_h4p_info *info, int on, u8 which);
void hci_h4p_set_auto_ctsrts(struct hci_h4p_info *info, int on, u8 which);
void hci_h4p_change_speed(struct hci_h4p_info *info, unsigned long speed);
int hci_h4p_reset_uart(struct hci_h4p_info *info);
int hci_h4p_init_uart(struct hci_h4p_info *info);

#endif /* __DRIVERS_BLUETOOTH_HCI_H4P_H */
