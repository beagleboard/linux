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

#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/serial_reg.h>

#include "hci_h4p.h"

void hci_h4p_bc4_parse_fw_event(struct hci_h4p_info *info, struct sk_buff *skb)
{
	/* Check if this is fw packet */
	if (skb->data[0] != 0xff) {
		hci_recv_frame(skb);
		return;
	}

	if (skb->data[11] || skb->data[12]) {
		dev_err(info->dev, "Firmware sending command failed\n");
		info->fw_error = -EPROTO;
	}

	kfree_skb(skb);
	complete(&info->fw_completion);
}

int hci_h4p_bc4_send_fw(struct hci_h4p_info *info,
			struct sk_buff_head *fw_queue)
{
	struct sk_buff *skb;
	unsigned int offset;
	int retries, count, i;

	info->fw_error = 0;

	NBT_DBG_FW("Sending firmware\n");
	skb = skb_dequeue(fw_queue);

	if (!skb)
		return -ENOMSG;

	/* Check if this is bd_address packet */
	if (skb->data[15] == 0x01 && skb->data[16] == 0x00) {
		offset = 21;
		skb->data[offset + 1] = 0x00;
		skb->data[offset + 5] = 0x00;
		skb->data[offset + 7] = info->bdaddr[0];
		skb->data[offset + 6] = info->bdaddr[1];
		skb->data[offset + 4] = info->bdaddr[2];
		skb->data[offset + 0] = info->bdaddr[3];
		skb->data[offset + 3] = info->bdaddr[4];
		skb->data[offset + 2] = info->bdaddr[5];
	}

	for (i = 0; i < 6; i++) {
		if (info->bdaddr[i] != 0x00)
			break;
	}

	if (i > 5) {
		dev_info(info->dev, "Valid bluetooth address not found.\n");
		kfree_skb(skb);
		return -ENODEV;
	}

	for (count = 1; ; count++) {
		NBT_DBG_FW("Sending firmware command %d\n", count);
		init_completion(&info->fw_completion);
		skb_queue_tail(&info->txq, skb);
		tasklet_schedule(&info->tx_task);

		skb = skb_dequeue(fw_queue);
		if (!skb)
			break;

		if (!wait_for_completion_timeout(&info->fw_completion,
						 msecs_to_jiffies(1000))) {
			dev_err(info->dev, "No reply to fw command\n");
			return -ETIMEDOUT;
		}

		if (info->fw_error) {
			dev_err(info->dev, "FW error\n");
			return -EPROTO;
		}
	};

	/* Wait for chip warm reset */
	retries = 100;
	while ((!skb_queue_empty(&info->txq) ||
	       !(hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT)) &&
	       retries--) {
		msleep(10);
	}
	if (!retries) {
		dev_err(info->dev, "Transmitter not empty\n");
		return -ETIMEDOUT;
	}

	hci_h4p_change_speed(info, BC4_MAX_BAUD_RATE);

	if (hci_h4p_wait_for_cts(info, 1, 100)) {
		dev_err(info->dev, "cts didn't go down after final speed change\n");
		return -ETIMEDOUT;
	}

	retries = 100;
	do {
		init_completion(&info->init_completion);
		hci_h4p_send_alive_packet(info);
		retries--;
	} while (!wait_for_completion_timeout(&info->init_completion, 100) &&
		 retries > 0);

	if (!retries) {
		dev_err(info->dev, "No alive reply after speed change\n");
		return -ETIMEDOUT;
	}

	return 0;
}
