/*
 *  linux/drivers/bluetooth/brf6150/brf6150.c
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

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/serial_reg.h>
#include <linux/skbuff.h>
#include <linux/firmware.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/irqs.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

#include "brf6150.h"

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

#define PM_TIMEOUT (2000)

static void brf6150_device_release(struct device *dev);
static struct brf6150_info *exit_info;

static struct platform_device brf6150_device = {
	.name		= BT_DEVICE,
	.id		= -1,
	.num_resources	= 0,
	.dev = {
		.release = brf6150_device_release,
	}
};

static struct device_driver brf6150_driver = {
	.name		= BT_DRIVER,
	.bus		= &platform_bus_type,
};

static inline void brf6150_outb(struct brf6150_info *info, unsigned int offset, u8 val)
{
	outb(val, info->uart_base + (offset << 2));
}

static inline u8 brf6150_inb(struct brf6150_info *info, unsigned int offset)
{
	return inb(info->uart_base + (offset << 2));
}

static void brf6150_set_rts(struct brf6150_info *info, int active)
{
	u8 b;

	b = brf6150_inb(info, UART_MCR);
	if (active)
		b |= UART_MCR_RTS;
	else
		b &= ~UART_MCR_RTS;
	brf6150_outb(info, UART_MCR, b);
}

static void brf6150_wait_for_cts(struct brf6150_info *info, int active,
				 int timeout_ms)
{
	int okay;
	unsigned long timeout;

	okay = 0;
	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	for (;;) {
		int state;

		state = brf6150_inb(info, UART_MSR) & UART_MSR_CTS;
		if (active) {
			if (state)
				break;
		} else {
			if (!state)
				break;
		}
		if (jiffies > timeout)
			break;
	}
}

static inline void brf6150_set_auto_ctsrts(struct brf6150_info *info, int on)
{
	u8 lcr, b;

	lcr = brf6150_inb(info, UART_LCR);
	brf6150_outb(info, UART_LCR, 0xbf);
	b = brf6150_inb(info, UART_EFR);
	if (on)
		b |= UART_EFR_CTS | UART_EFR_RTS;
	else
		b &= ~(UART_EFR_CTS | UART_EFR_RTS);
	brf6150_outb(info, UART_EFR, b);
	brf6150_outb(info, UART_LCR, lcr);
}

static inline void brf6150_enable_pm_rx(struct brf6150_info *info)
{
	if (info->pm_enabled) {
		info->rx_pm_enabled = 1;
	}
}

static inline void brf6150_disable_pm_rx(struct brf6150_info *info)
{
	if (info->pm_enabled) {
		info->rx_pm_enabled = 0;
	}
}

static void brf6150_enable_pm_tx(struct brf6150_info *info)
{
	if (info->pm_enabled) {
		mod_timer(&info->pm_timer, jiffies + msecs_to_jiffies(PM_TIMEOUT));
		info->tx_pm_enabled = 1;
	}
}

static void brf6150_disable_pm_tx(struct brf6150_info *info)
{
	if (info->pm_enabled) {
		info->tx_pm_enabled = 0;
		gpio_set_value(info->btinfo->bt_wakeup_gpio, 1);
	}
	if (gpio_get_value(info->btinfo->host_wakeup_gpio))
		tasklet_schedule(&info->tx_task);
}

static void brf6150_pm_timer(unsigned long data)
{
	struct brf6150_info *info;

	info = (struct brf6150_info *)data;
	if (info->tx_pm_enabled && info->rx_pm_enabled && !test_bit(HCI_INQUIRY, &info->hdev->flags))
		gpio_set_value(info->btinfo->bt_wakeup_gpio, 0);
	else
		mod_timer(&info->pm_timer, jiffies + msecs_to_jiffies(PM_TIMEOUT));
}

static int brf6150_change_speed(struct brf6150_info *info, unsigned long speed)
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

	brf6150_outb(info, UART_OMAP_MDR1, 7); /* Make sure UART mode is disabled */
	lcr = brf6150_inb(info, UART_LCR);
	brf6150_outb(info, UART_LCR, UART_LCR_DLAB);     /* Set DLAB */
	brf6150_outb(info, UART_DLL, divisor & 0xff);    /* Set speed */
	brf6150_outb(info, UART_DLM, divisor >> 8);
	brf6150_outb(info, UART_LCR, lcr);
	brf6150_outb(info, UART_OMAP_MDR1, mdr1); /* Make sure UART mode is enabled */

	return 0;
}

/* Firmware handling */
static int brf6150_open_firmware(struct brf6150_info *info)
{
	int err;

	info->fw_pos = 0;
	err = request_firmware(&info->fw_entry, "brf6150fw.bin", &brf6150_device.dev);

	return err;
}

static struct sk_buff *brf6150_read_fw_cmd(struct brf6150_info *info, int how)
{
	struct sk_buff *skb;
	unsigned int cmd_len;

	if (info->fw_pos >= info->fw_entry->size) {
		return NULL;
	}

	cmd_len = info->fw_entry->data[info->fw_pos++];
	if (!cmd_len)
		return NULL;

	if (info->fw_pos + cmd_len > info->fw_entry->size) {
		printk(KERN_WARNING "Corrupted firmware image\n");
		return NULL;
	}

	skb = bt_skb_alloc(cmd_len, how);
	if (!skb) {
		printk(KERN_WARNING "Cannot reserve memory for buffer\n");
		return NULL;
	}
	memcpy(skb_put(skb, cmd_len), &info->fw_entry->data[info->fw_pos], cmd_len);

	info->fw_pos += cmd_len;

	return skb;
}

static int brf6150_close_firmware(struct brf6150_info *info)
{
	release_firmware(info->fw_entry);
	return 0;
}

static int brf6150_send_alive_packet(struct brf6150_info *info)
{
	struct sk_buff *skb;

	NBT_DBG("Sending alive packet\n");
	skb = brf6150_read_fw_cmd(info, GFP_ATOMIC);
	if (!skb) {
		printk(KERN_WARNING "Cannot read alive command");
		return -1;
	}

	clk_enable(info->uart_ck);
	skb_queue_tail(&info->txq, skb);
	tasklet_schedule(&info->tx_task);

	NBT_DBG("Alive packet sent\n");
	return 0;
}

static void brf6150_alive_packet(struct brf6150_info *info, struct sk_buff *skb)
{
	NBT_DBG("Received alive packet\n");
	if (skb->data[1] == 0xCC) {
		complete(&info->init_completion);
	}

	kfree_skb(skb);
}

static int brf6150_send_negotiation(struct brf6150_info *info)
{
	struct sk_buff *skb;
	NBT_DBG("Sending negotiation..\n");

	brf6150_change_speed(info, INIT_SPEED);

	skb = brf6150_read_fw_cmd(info, GFP_KERNEL);

	if (!skb) {
		printk(KERN_WARNING "Cannot read negoatiation message");
		return -1;
	}

	clk_enable(info->uart_ck);
	skb_queue_tail(&info->txq, skb);
	tasklet_schedule(&info->tx_task);


	NBT_DBG("Negotiation sent\n");
	return 0;
}

static void brf6150_negotiation_packet(struct brf6150_info *info,
				       struct sk_buff *skb)
{
	if (skb->data[1] == 0x20) {
		/* Change to operational settings */
		brf6150_set_rts(info, 0);
		brf6150_wait_for_cts(info, 0, 100);
		brf6150_change_speed(info, MAX_BAUD_RATE);
		brf6150_set_rts(info, 1);
		brf6150_wait_for_cts(info, 1, 100);
		brf6150_set_auto_ctsrts(info, 1);
		brf6150_send_alive_packet(info);
	} else {
		printk(KERN_WARNING "Could not negotiate brf6150 settings\n");
	}
	kfree_skb(skb);
}

static int brf6150_get_hdr_len(u8 pkt_type)
{
	long retval;

	switch (pkt_type) {
	case H4_EVT_PKT:
		retval = HCI_EVENT_HDR_SIZE;
		break;
	case H4_ACL_PKT:
		retval = HCI_ACL_HDR_SIZE;
		break;
	case H4_SCO_PKT:
		retval = HCI_SCO_HDR_SIZE;
		break;
	case H4_NEG_PKT:
		retval = 9;
		break;
	case H4_ALIVE_PKT:
		retval = 3;
		break;
	default:
		printk(KERN_ERR "brf6150: Unknown H4 packet");
		retval = -1;
		break;
	}

	return retval;
}

static unsigned int brf6150_get_data_len(struct brf6150_info *info,
					 struct sk_buff *skb)
{
	long retval = -1;
	struct hci_event_hdr *evt_hdr;
	struct hci_acl_hdr *acl_hdr;
	struct hci_sco_hdr *sco_hdr;

	switch (bt_cb(skb)->pkt_type) {
	case H4_EVT_PKT:
		evt_hdr = (struct hci_event_hdr *)skb->data;
		retval = evt_hdr->plen;
		break;
	case H4_ACL_PKT:
		acl_hdr = (struct hci_acl_hdr *)skb->data;
		retval = le16_to_cpu(acl_hdr->dlen);
		break;
	case H4_SCO_PKT:
		sco_hdr = (struct hci_sco_hdr *)skb->data;
		retval = sco_hdr->dlen;
		break;
	case H4_NEG_PKT:
		retval = 0;
		break;
	case H4_ALIVE_PKT:
		retval = 0;
		break;
	}

	return retval;
}

static void brf6150_parse_fw_event(struct brf6150_info *info)
{
	struct hci_fw_event *ev;

	if (bt_cb(info->rx_skb)->pkt_type != H4_EVT_PKT) {
		printk(KERN_WARNING "Got non event fw packet.\n");
		info->fw_error = 1;
		return;
	}

	ev = (struct hci_fw_event *)info->rx_skb->data;
	if (ev->hev.evt != HCI_EV_CMD_COMPLETE) {
		printk(KERN_WARNING "Got non cmd complete fw event\n");
		info->fw_error = 1;
		return;
	}

	if (ev->status != 0) {
		printk(KERN_WARNING "Got error status from fw command\n");
		info->fw_error = 1;
		return;
	}

	complete(&info->fw_completion);
}

static inline void brf6150_recv_frame(struct brf6150_info *info,
				      struct sk_buff *skb)
{
	if (unlikely(!test_bit(HCI_RUNNING, &info->hdev->flags))) {
		NBT_DBG("fw_event\n");
		brf6150_parse_fw_event(info);
		kfree_skb(skb);
	} else {
		hci_recv_frame(skb);
		if (!(brf6150_inb(info, UART_LSR) & UART_LSR_DR))
			brf6150_enable_pm_rx(info);
		NBT_DBG("Frame sent to upper layer\n");
	}

}

static inline void brf6150_rx(struct brf6150_info *info)
{
	u8 byte;

	NBT_DBG_TRANSFER("rx_tasklet woke up\ndata ");

	while (brf6150_inb(info, UART_LSR) & UART_LSR_DR) {
		if (info->rx_skb == NULL) {
			info->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
			if (!info->rx_skb) {
				printk(KERN_WARNING "brf6150: Can't allocate memory for new packet\n");
				return;
			}
			info->rx_state = WAIT_FOR_PKT_TYPE;
			info->rx_skb->dev = (void *)info->hdev;
			brf6150_disable_pm_rx(info);
			clk_enable(info->uart_ck);
		}

		byte = brf6150_inb(info, UART_RX);
		if (info->garbage_bytes) {
			info->garbage_bytes--;
			info->hdev->stat.err_rx++;
			continue;
		}
		info->hdev->stat.byte_rx++;
		NBT_DBG_TRANSFER_NF("0x%.2x  ", byte);
		switch (info->rx_state) {
		case WAIT_FOR_PKT_TYPE:
			bt_cb(info->rx_skb)->pkt_type = byte;
			info->rx_count = brf6150_get_hdr_len(byte);
			if (info->rx_count >= 0) {
				info->rx_state = WAIT_FOR_HEADER;
			} else {
				info->hdev->stat.err_rx++;
				kfree_skb(info->rx_skb);
				info->rx_skb = NULL;
				clk_disable(info->uart_ck);
			}
			break;
		case WAIT_FOR_HEADER:
			info->rx_count--;
			*skb_put(info->rx_skb, 1) = byte;
			if (info->rx_count == 0) {
				info->rx_count = brf6150_get_data_len(info, info->rx_skb);
				if (info->rx_count > skb_tailroom(info->rx_skb)) {
					printk(KERN_WARNING "brf6150: Frame is %ld bytes too long.\n",
					       info->rx_count - skb_tailroom(info->rx_skb));
					info->rx_skb = NULL;
					info->garbage_bytes = info->rx_count - skb_tailroom(info->rx_skb);
					clk_disable(info->uart_ck);
					break;
				}
				info->rx_state = WAIT_FOR_DATA;
				if (bt_cb(info->rx_skb)->pkt_type == H4_NEG_PKT) {
					brf6150_negotiation_packet(info, info->rx_skb);
					info->rx_skb = NULL;
					clk_disable(info->uart_ck);
					return;
				}
				if (bt_cb(info->rx_skb)->pkt_type == H4_ALIVE_PKT) {
					brf6150_alive_packet(info, info->rx_skb);
					info->rx_skb = NULL;
					clk_disable(info->uart_ck);
					return;
				}
			}
			break;
		case WAIT_FOR_DATA:
			info->rx_count--;
			*skb_put(info->rx_skb, 1) = byte;
			if (info->rx_count == 0) {
				brf6150_recv_frame(info, info->rx_skb);
				info->rx_skb = NULL;
				clk_disable(info->uart_ck);
			}
			break;
		default:
			WARN_ON(1);
			break;
		}
	}

	NBT_DBG_TRANSFER_NF("\n");
}

static void brf6150_tx_tasklet(unsigned long data)
{
	unsigned int sent = 0;
	unsigned long flags;
	struct sk_buff *skb;
	struct brf6150_info *info = (struct brf6150_info *)data;

	NBT_DBG_TRANSFER("tx_tasklet woke up\n data ");

	skb = skb_dequeue(&info->txq);
	if (!skb) {
		/* No data in buffer */
		brf6150_enable_pm_tx(info);
		return;
	}

	/* Copy data to tx fifo */
	while (!(brf6150_inb(info, UART_OMAP_SSR) & UART_OMAP_SSR_TXFULL) &&
	       (sent < skb->len)) {
		NBT_DBG_TRANSFER_NF("0x%.2x ", skb->data[sent]);
		brf6150_outb(info, UART_TX, skb->data[sent]);
		sent++;
	}

	info->hdev->stat.byte_tx += sent;
	NBT_DBG_TRANSFER_NF("\n");
	if (skb->len == sent) {
		kfree_skb(skb);
		clk_disable(info->uart_ck);
	} else {
		skb_pull(skb, sent);
		skb_queue_head(&info->txq, skb);
	}

	spin_lock_irqsave(&info->lock, flags);
	brf6150_outb(info, UART_IER, brf6150_inb(info, UART_IER) | UART_IER_THRI);
	spin_unlock_irqrestore(&info->lock, flags);
}

static irqreturn_t brf6150_interrupt(int irq, void *data)
{
	struct brf6150_info *info = (struct brf6150_info *)data;
	u8 iir, msr;
	int ret;
	unsigned long flags;

	ret = IRQ_NONE;

	clk_enable(info->uart_ck);
	iir = brf6150_inb(info, UART_IIR);
	if (iir & UART_IIR_NO_INT) {
		printk("Interrupt but no reason irq 0x%.2x\n", iir);
		clk_disable(info->uart_ck);
		return IRQ_HANDLED;
	}

	NBT_DBG("In interrupt handler iir 0x%.2x\n", iir);

	iir &= UART_IIR_ID;

	if (iir == UART_IIR_MSI) {
		msr = brf6150_inb(info, UART_MSR);
		ret = IRQ_HANDLED;
	}
	if (iir == UART_IIR_RLSI) {
		brf6150_inb(info, UART_RX);
		brf6150_inb(info, UART_LSR);
		ret = IRQ_HANDLED;
	}

	if (iir == UART_IIR_RDI) {
		brf6150_rx(info);
		ret = IRQ_HANDLED;
	}

	if (iir == UART_IIR_THRI) {
		spin_lock_irqsave(&info->lock, flags);
		brf6150_outb(info, UART_IER, brf6150_inb(info, UART_IER) & ~UART_IER_THRI);
		spin_unlock_irqrestore(&info->lock, flags);
		tasklet_schedule(&info->tx_task);
		ret = IRQ_HANDLED;
	}

	clk_disable(info->uart_ck);
	return ret;
}

static irqreturn_t brf6150_wakeup_interrupt(int irq, void *dev_inst)
{
	struct brf6150_info *info = dev_inst;
	int should_wakeup;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	should_wakeup = gpio_get_value(info->btinfo->host_wakeup_gpio);
	NBT_DBG_POWER("gpio interrupt %d\n", should_wakeup);
	if (should_wakeup) {
		clk_enable(info->uart_ck);
		brf6150_set_auto_ctsrts(info, 1);
		brf6150_rx(info);
		tasklet_schedule(&info->tx_task);
	} else {
		brf6150_set_auto_ctsrts(info, 0);
		brf6150_set_rts(info, 0);
		clk_disable(info->uart_ck);
	}

	spin_unlock_irqrestore(&info->lock, flags);
	return IRQ_HANDLED;
}

static int brf6150_init_uart(struct brf6150_info *info)
{
	int count = 0;

	/* Reset the  UART */
	brf6150_outb(info, UART_OMAP_SYSC, UART_SYSC_OMAP_RESET);
	while (!(brf6150_inb(info, UART_OMAP_SYSS) & UART_SYSS_RESETDONE)) {
		if (count++ > 100) {
			printk(KERN_ERR "brf6150: UART reset timeout\n");
			return -1;
		}
		udelay(1);
	}

	/* Enable and setup FIFO */
	brf6150_outb(info, UART_LCR, UART_LCR_WLEN8);
	brf6150_outb(info, UART_OMAP_MDR1, 0x00); /* Make sure UART mode is enabled */
	brf6150_outb(info, UART_OMAP_SCR, 0x00);
	brf6150_outb(info, UART_EFR, brf6150_inb(info, UART_EFR) | UART_EFR_ECB);
	brf6150_outb(info, UART_MCR, brf6150_inb(info, UART_MCR) | UART_MCR_TCRTLR);
	brf6150_outb(info, UART_TI752_TLR, 0xff);
	brf6150_outb(info, UART_TI752_TCR, 0x1f);
	brf6150_outb(info, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	brf6150_outb(info, UART_IER, UART_IER_RDI);

	return 0;
}

static int brf6150_reset(struct brf6150_info *info)
{
	gpio_set_value(info->btinfo->bt_wakeup_gpio, 0);
	gpio_set_value(info->btinfo->reset_gpio, 0);
	current->state = TASK_UNINTERRUPTIBLE;
	schedule_timeout(msecs_to_jiffies(10));
	gpio_set_value(info->btinfo->bt_wakeup_gpio, 1);
	current->state = TASK_UNINTERRUPTIBLE;
	schedule_timeout(msecs_to_jiffies(100));
	gpio_set_value(info->btinfo->reset_gpio, 1);
	current->state = TASK_UNINTERRUPTIBLE;
	schedule_timeout(msecs_to_jiffies(100));

	return 0;
}

static int brf6150_send_firmware(struct brf6150_info *info)
{
	struct sk_buff *skb;

	init_completion(&info->fw_completion);
	info->fw_error = 0;

	while ((skb = brf6150_read_fw_cmd(info, GFP_KERNEL)) != NULL) {
		clk_enable(info->uart_ck);
		skb_queue_tail(&info->txq, skb);
		tasklet_schedule(&info->tx_task);

		if (!wait_for_completion_timeout(&info->fw_completion, HZ)) {
			return -1;
		}

		if (info->fw_error) {
			return -1;
		}
	}
	NBT_DBG_FW("Firmware sent\n");

	return 0;

}

/* hci callback functions */
static int brf6150_hci_flush(struct hci_dev *hdev)
{
	struct brf6150_info *info;
	info = hdev->driver_data;

	skb_queue_purge(&info->txq);

	return 0;
}

static int brf6150_hci_open(struct hci_dev *hdev)
{
	struct brf6150_info *info;
	int err;

	info = hdev->driver_data;

	if (test_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	if (brf6150_open_firmware(info) < 0) {
		printk("Cannot open firmware\n");
		return -1;
	}

	info->rx_state = WAIT_FOR_PKT_TYPE;
	info->rx_count = 0;
	info->garbage_bytes = 0;
	info->rx_skb = NULL;
	info->pm_enabled = 0;
	set_irq_type(gpio_to_irq(info->btinfo->host_wakeup_gpio), IRQ_TYPE_NONE);
	init_completion(&info->fw_completion);

	clk_enable(info->uart_ck);

	brf6150_init_uart(info);
	brf6150_set_auto_ctsrts(info, 0);
	brf6150_set_rts(info, 0);
	brf6150_reset(info);
	brf6150_wait_for_cts(info, 1, 10);
	brf6150_set_rts(info, 1);
	if (brf6150_send_negotiation(info)) {
		brf6150_close_firmware(info);
		return -1;
	}

	if (!wait_for_completion_interruptible_timeout(&info->init_completion, HZ)) {
		brf6150_close_firmware(info);
		clk_disable(info->uart_ck);
		clear_bit(HCI_RUNNING, &hdev->flags);
		return -1;
	}
	brf6150_set_auto_ctsrts(info, 1);

	err = brf6150_send_firmware(info);
	brf6150_close_firmware(info);
	if (err < 0)
		printk(KERN_ERR "brf6150: Sending firmware failed. Bluetooth won't work properly\n");

	set_irq_type(gpio_to_irq(info->btinfo->host_wakeup_gpio), IRQ_TYPE_EDGE_BOTH);
	info->pm_enabled = 1;
	set_bit(HCI_RUNNING, &hdev->flags);
	return 0;
}

static int brf6150_hci_close(struct hci_dev *hdev)
{
	struct brf6150_info *info = hdev->driver_data;
	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	brf6150_hci_flush(hdev);
	clk_disable(info->uart_ck);
	del_timer_sync(&info->pm_timer);
	gpio_set_value(info->btinfo->bt_wakeup_gpio, 0);
	set_irq_type(gpio_to_irq(info->btinfo->host_wakeup_gpio), IRQ_TYPE_NONE);

	return 0;
}

static void brf6150_hci_destruct(struct hci_dev *hdev)
{
}

static int brf6150_hci_send_frame(struct sk_buff *skb)
{
	struct brf6150_info *info;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;

	if (!hdev) {
		printk(KERN_WARNING "brf6150: Frame for unknown device\n");
		return -ENODEV;
	}

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		printk(KERN_WARNING "brf6150: Frame for non-running device\n");
		return -EIO;
	}

	info = hdev->driver_data;

	switch (bt_cb(skb)->pkt_type) {
		case HCI_COMMAND_PKT:
			hdev->stat.cmd_tx++;
			break;
		case HCI_ACLDATA_PKT:
			hdev->stat.acl_tx++;
			break;
		case HCI_SCODATA_PKT:
			hdev->stat.sco_tx++;
			break;
	};

	/* Push frame type to skb */
	clk_enable(info->uart_ck);
	*skb_push(skb, 1) = bt_cb(skb)->pkt_type;
	skb_queue_tail(&info->txq, skb);

	brf6150_disable_pm_tx(info);

	return 0;
}

static int brf6150_hci_ioctl(struct hci_dev *hdev, unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static void brf6150_device_release(struct device *dev)
{
}

static int brf6150_register_hdev(struct brf6150_info *info)
{
	struct hci_dev *hdev;

	/* Initialize and register HCI device */

	hdev = hci_alloc_dev();
	if (!hdev) {
		printk(KERN_WARNING "brf6150: Can't allocate memory for device\n");
		return -ENOMEM;
	}
	info->hdev = hdev;

	hdev->type = HCI_UART;
	hdev->driver_data = info;

	hdev->open = brf6150_hci_open;
	hdev->close = brf6150_hci_close;
	hdev->destruct = brf6150_hci_destruct;
	hdev->flush = brf6150_hci_flush;
	hdev->send = brf6150_hci_send_frame;
	hdev->destruct = brf6150_hci_destruct;
	hdev->ioctl = brf6150_hci_ioctl;

	hdev->owner = THIS_MODULE;

	if (hci_register_dev(hdev) < 0) {
		printk(KERN_WARNING "brf6150: Can't register HCI device %s.\n", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static int __init brf6150_init(void)
{
	struct brf6150_info *info;
	int irq, err;

	info = kmalloc(sizeof(struct brf6150_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	memset(info, 0, sizeof(struct brf6150_info));

	brf6150_device.dev.driver_data = info;
	init_completion(&info->init_completion);
	init_completion(&info->fw_completion);
	info->pm_enabled = 0;
	info->rx_pm_enabled = 0;
	info->tx_pm_enabled = 0;
	info->garbage_bytes = 0;
	tasklet_init(&info->tx_task, brf6150_tx_tasklet, (unsigned long)info);
	spin_lock_init(&info->lock);
	skb_queue_head_init(&info->txq);
	init_timer(&info->pm_timer);
	info->pm_timer.function = brf6150_pm_timer;
	info->pm_timer.data = (unsigned long)info;
	exit_info = NULL;

	info->btinfo = omap_get_config(OMAP_TAG_NOKIA_BT, struct omap_bluetooth_config);
	if (info->btinfo == NULL)
		return -1;

	NBT_DBG("RESET gpio: %d\n", info->btinfo->reset_gpio);
	NBT_DBG("BTWU gpio: %d\n", info->btinfo->bt_wakeup_gpio);
	NBT_DBG("HOSTWU gpio: %d\n", info->btinfo->host_wakeup_gpio);
	NBT_DBG("Uart: %d\n", info->btinfo->bt_uart);
	NBT_DBG("sysclk: %d\n", info->btinfo->bt_sysclk);

	err = gpio_request(info->btinfo->reset_gpio, "BT reset");
	if (err < 0)
	{
		printk(KERN_WARNING "Cannot get GPIO line %d", 
		       info->btinfo->reset_gpio);
		kfree(info);
		return err;
	}

	err = gpio_request(info->btinfo->bt_wakeup_gpio, "BT wakeup");
	if (err < 0)
	{
		printk(KERN_WARNING "Cannot get GPIO line 0x%d",
		       info->btinfo->bt_wakeup_gpio);
		gpio_free(info->btinfo->reset_gpio);
		kfree(info);
		return err;
	}

	err = gpio_request(info->btinfo->host_wakeup_gpio, "BT host wakeup");
	if (err < 0)
	{
		printk(KERN_WARNING "Cannot get GPIO line %d",
		       info->btinfo->host_wakeup_gpio);
		gpio_free(info->btinfo->reset_gpio);
		gpio_free(info->btinfo->bt_wakeup_gpio);
		kfree(info);
		return err;
	}

	gpio_direction_output(info->btinfo->reset_gpio, 0);
	gpio_direction_output(info->btinfo->bt_wakeup_gpio, 0);
	gpio_direction_input(info->btinfo->host_wakeup_gpio);
	set_irq_type(gpio_to_irq(info->btinfo->host_wakeup_gpio), IRQ_TYPE_NONE);

	switch (info->btinfo->bt_uart) {
	case 1:
		irq = INT_UART1;
		info->uart_ck = clk_get(NULL, "uart1_ck");
		/* FIXME: Use platform_get_resource for the port */
		info->uart_base = ioremap(OMAP_UART1_BASE, 0x16);
		if (!info->uart_base)
			goto cleanup;
		break;
	case 2:
		irq = INT_UART2;
		info->uart_ck = clk_get(NULL, "uart2_ck");
		/* FIXME: Use platform_get_resource for the port */
		info->uart_base = ioremap(OMAP_UART2_BASE, 0x16);
		if (!info->uart_base)
			goto cleanup;
		break;
	case 3:
		irq = INT_UART3;
		info->uart_ck = clk_get(NULL, "uart3_ck");
		/* FIXME: Use platform_get_resource for the port */
		info->uart_base = ioremap(OMAP_UART3_BASE, 0x16);
		if (!info->uart_base)
			goto cleanup;
		break;
	default:
		printk(KERN_ERR "No uart defined\n");
		goto cleanup;
	}

	info->irq = irq;
	err = request_irq(irq, brf6150_interrupt, 0, "brf6150", (void *)info);
	if (err < 0) {
		printk(KERN_ERR "brf6150: unable to get IRQ %d\n", irq);
		goto cleanup;
	}

	err = request_irq(gpio_to_irq(info->btinfo->host_wakeup_gpio),
			brf6150_wakeup_interrupt, 0, "brf6150_wkup", (void *)info);
	if (err < 0) {
		printk(KERN_ERR "brf6150: unable to get wakeup IRQ %d\n",
				gpio_to_irq(info->btinfo->host_wakeup_gpio));
		free_irq(irq, (void *)info);
		goto cleanup;
	}

	/* Register with LDM */
	if (platform_device_register(&brf6150_device)) {
		printk(KERN_ERR "failed to register brf6150 device\n");
		err = -ENODEV;
		goto cleanup_irq;
	}
	/* Register the driver with LDM */
	if (driver_register(&brf6150_driver)) {
		printk(KERN_WARNING "failed to register brf6150 driver\n");
		platform_device_unregister(&brf6150_device);
		err = -ENODEV;
		goto cleanup_irq;
	}

	if (brf6150_register_hdev(info) < 0) {
		printk(KERN_WARNING "failed to register brf6150 hci device\n");
		platform_device_unregister(&brf6150_device);
		driver_unregister(&brf6150_driver);
		goto cleanup_irq;
	}

	exit_info = info;
	return 0;

cleanup_irq:
	free_irq(irq, (void *)info);
	free_irq(gpio_to_irq(info->btinfo->host_wakeup_gpio), (void *)info);
cleanup:
	gpio_free(info->btinfo->reset_gpio);
	gpio_free(info->btinfo->bt_wakeup_gpio);
	gpio_free(info->btinfo->host_wakeup_gpio);
	kfree(info);

	return err;
}

static void __exit brf6150_exit(void)
{
	brf6150_hci_close(exit_info->hdev);
	hci_free_dev(exit_info->hdev);
	gpio_free(exit_info->btinfo->reset_gpio);
	gpio_free(exit_info->btinfo->bt_wakeup_gpio);
	gpio_free(exit_info->btinfo->host_wakeup_gpio);
	free_irq(exit_info->irq, (void *)exit_info);
	free_irq(gpio_to_irq(exit_info->btinfo->host_wakeup_gpio), (void *)exit_info);
	kfree(exit_info);
}

module_init(brf6150_init);
module_exit(brf6150_exit);

MODULE_DESCRIPTION("brf6150 hci driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ville Tervo <ville.tervo@nokia.com>");
