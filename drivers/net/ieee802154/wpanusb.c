// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the WPANUSB IEEE 802.15.4 dongle
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * The driver implements SoftMAC 802.15.4 protocol based on atusb
 * driver for ATUSB IEEE 802.15.4 dongle.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/usb.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#define DEBUG
#include "wpanusb.h"

#define WPANUSB_NUM_RX_URBS	4	/* allow for a bit of local latency */
#define WPANUSB_ALLOC_DELAY_MS	100	/* delay after failed allocation */

#define VENDOR_OUT		(USB_TYPE_VENDOR | USB_DIR_OUT)
#define VENDOR_IN		(USB_TYPE_VENDOR | USB_DIR_IN)

#define WPANUSB_VALID_CHANNELS	(0x07FFFFFF)

struct wpanusb {
	struct ieee802154_hw *hw;
	struct usb_device *udev;
	int shutdown;			/* non-zero if shutting down */

	/* RX variables */
	struct delayed_work work;	/* memory allocations */
	struct usb_anchor idle_urbs;	/* URBs waiting to be submitted */
	struct usb_anchor rx_urbs;	/* URBs waiting for reception */

	/* TX variables */
	struct usb_ctrlrequest tx_dr;
	struct urb *tx_urb;
	struct sk_buff *tx_skb;
	u8 tx_ack_seq;			/* current TX ACK sequence number */
};

/* ----- USB commands without data ----------------------------------------- */

static int wpanusb_control_send(struct wpanusb *wpanusb, unsigned int pipe,
				u8 request, void *data, u16 size)
{
	struct usb_device *udev = wpanusb->udev;

	return usb_control_msg(udev, pipe, request, VENDOR_OUT,
			       0, 0, data, size, 1000);
}

static int wpanusb_control_recv(struct wpanusb *wpanusb, u8 request, void *data, u16 size)
{
	struct usb_device *udev = wpanusb->udev;

	usb_control_msg(udev, usb_sndctrlpipe(udev, 0), request, VENDOR_OUT,
			       0, 0, NULL, 0, 1000);

	return usb_control_msg(udev, usb_rcvbulkpipe(udev, 1), request, VENDOR_IN,
			       0, 0, data, size, 1000);
}

/* ----- skb allocation ---------------------------------------------------- */

#define MAX_PSDU	127
#define MAX_RX_XFER	(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */

#define SKB_WPANUSB(skb)	(*(struct wpanusb **)(skb)->cb)

static void wpanusb_bulk_complete(struct urb *urb);

static int wpanusb_submit_rx_urb(struct wpanusb *wpanusb, struct urb *urb)
{
	struct usb_device *udev = wpanusb->udev;
	struct sk_buff *skb = urb->context;
	int ret;

	if (!skb) {
		skb = alloc_skb(MAX_RX_XFER, GFP_KERNEL);
		if (!skb) {
			dev_warn_ratelimited(&udev->dev,
					     "can't allocate skb\n");
			return -ENOMEM;
		}
		skb_put(skb, MAX_RX_XFER);
		SKB_WPANUSB(skb) = wpanusb;
	}

	usb_fill_bulk_urb(urb, udev, usb_rcvbulkpipe(udev, 1),
			  skb->data, MAX_RX_XFER, wpanusb_bulk_complete, skb);
	usb_anchor_urb(urb, &wpanusb->rx_urbs);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		usb_unanchor_urb(urb);
		kfree_skb(skb);
		urb->context = NULL;
	}

	return ret;
}

static void wpanusb_work_urbs(struct work_struct *work)
{
	struct wpanusb *wpanusb =
		container_of(to_delayed_work(work), struct wpanusb, work);
	struct usb_device *udev = wpanusb->udev;
	struct urb *urb;
	int ret;

	if (wpanusb->shutdown)
		return;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			return;

		ret = wpanusb_submit_rx_urb(wpanusb, urb);
	} while (!ret);

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	dev_warn_ratelimited(&udev->dev, "can't allocate/submit URB (%d)\n",
			     ret);
	schedule_delayed_work(&wpanusb->work,
			      msecs_to_jiffies(WPANUSB_ALLOC_DELAY_MS) + 1);
}

/* ----- Asynchronous USB -------------------------------------------------- */

static void wpanusb_tx_done(struct wpanusb *wpanusb, uint8_t seq)
{
	struct usb_device *udev = wpanusb->udev;
	u8 expect = wpanusb->tx_ack_seq;

	dev_dbg(&udev->dev, "seq 0x%02x expect 0x%02x\n", seq, expect);

	if (seq == expect) {
		ieee802154_xmit_complete(wpanusb->hw, wpanusb->tx_skb, false);
	} else {
		dev_dbg(&udev->dev, "unknown ack %u\n", seq);

		ieee802154_wake_queue(wpanusb->hw);
		if (wpanusb->tx_skb)
			dev_kfree_skb_irq(wpanusb->tx_skb);
	}
}

static void wpanusb_process_urb(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);
	u8 len, lqi;

	if (!urb->actual_length) {
		dev_dbg(&udev->dev, "zero-sized URB ?\n");
		return;
	}

	len = *skb->data;

	dev_dbg(&udev->dev, "urb %p urb len %u pkt len %u", urb,
		urb->actual_length, len);

	/* Handle ACK */
	if (urb->actual_length == 1) {
		wpanusb_tx_done(wpanusb, len);
		return;
	}

	if (len + 1 > urb->actual_length - 1) {
		dev_dbg(&udev->dev, "frame len %d+1 > URB %u-1\n",
			len, urb->actual_length);
		return;
	}

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_dbg(&udev->dev, "frame corrupted\n");
		return;
	}

	print_hex_dump_bytes("> ", DUMP_PREFIX_OFFSET, skb->data,
			     urb->actual_length);

	/* Get LQI at the end of the packet */
	lqi = skb->data[len + 1];
	dev_dbg(&udev->dev, "rx len %d lqi 0x%02x\n", len, lqi);
	skb_pull(skb, 1);	/* remove length */
	skb_trim(skb, len);	/* remove LQI */
	ieee802154_rx_irqsafe(wpanusb->hw, skb, lqi);
	urb->context = NULL;	/* skb is gone */
}

static void wpanusb_bulk_complete(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);

	dev_dbg(&udev->dev, "status %d len %d\n",
		urb->status, urb->actual_length);

	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			urb->context = NULL;
			return;
		}

		dev_dbg(&udev->dev, "URB error %d\n", urb->status);
	} else {
		wpanusb_process_urb(urb);
	}

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	if (!wpanusb->shutdown)
		schedule_delayed_work(&wpanusb->work, 0);
}

/* ----- URB allocation/deallocation --------------------------------------- */

static void wpanusb_free_urbs(struct wpanusb *wpanusb)
{
	struct urb *urb;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			break;
		kfree_skb(urb->context);
		usb_free_urb(urb);
	} while (true);
}

static int wpanusb_alloc_urbs(struct wpanusb *wpanusb, unsigned int n)
{
	struct urb *urb;

	while (n--) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			wpanusb_free_urbs(wpanusb);
			return -ENOMEM;
		}
		usb_anchor_urb(urb, &wpanusb->idle_urbs);
	}

	return 0;
}

/* ----- IEEE 802.15.4 interface operations -------------------------------- */

static void wpanusb_xmit_complete(struct urb *urb)
{
	dev_dbg(&urb->dev->dev, "urb transmit completed");
}

static int wpanusb_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret = 0;

	dev_dbg(&udev->dev, "len %u", skb->len);

	/* ack_seq range is 0x01 - 0xff */
	wpanusb->tx_ack_seq++;
	if (!wpanusb->tx_ack_seq)
		wpanusb->tx_ack_seq++;

	wpanusb->tx_skb = skb;
	wpanusb->tx_dr.wIndex = cpu_to_le16(wpanusb->tx_ack_seq);
	wpanusb->tx_dr.wLength = cpu_to_le16(skb->len);

	usb_fill_control_urb(wpanusb->tx_urb, udev,
			     usb_sndctrlpipe(udev, 0),
			     (unsigned char *)&wpanusb->tx_dr, skb->data,
			     skb->len, wpanusb_xmit_complete, NULL);
	ret = usb_submit_urb(wpanusb->tx_urb, GFP_ATOMIC);

	dev_dbg(&udev->dev, "%s: ret %d len %u seq %u\n", __func__, ret,
		skb->len, wpanusb->tx_ack_seq);

	return ret;
}

static int wpanusb_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_channel *req;
	int ret;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->page = page;
	req->channel = channel;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_CHANNEL, req, sizeof(*req));
	kfree(req);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed set channel, ret %d", ret);
		return ret;
	}

	dev_dbg(&udev->dev, "set page %u channel %u", page, channel);

	return 0;
}

static int wpanusb_ed(struct ieee802154_hw *hw, u8 *level)
{
	WARN_ON(!level);

	*level = 0xbe;

	return 0;
}

static int wpanusb_set_hw_addr_filt(struct ieee802154_hw *hw,
				    struct ieee802154_hw_addr_filt *filt,
				    unsigned long changed)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret = 0;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		struct set_short_addr *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		req->short_addr = filt->short_addr;

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
					   SET_SHORT_ADDR, req, sizeof(*req));
		kfree(req);
		if (ret < 0) {
			dev_err(&udev->dev, "Failed to set short_addr, ret %d",
				ret);
			return ret;
		}

		dev_dbg(&udev->dev, "short addr changed to 0x%04x",
			le16_to_cpu(filt->short_addr));
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		struct set_pan_id *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		req->pan_id = filt->pan_id;

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
					   SET_PAN_ID, req, sizeof(*req));
		kfree(req);
		if (ret < 0) {
			dev_err(&udev->dev, "Failed to set pan_id, ret %d",
				ret);
			return ret;
		}

		dev_dbg(&udev->dev, "pan id changed to 0x%04x",
			le16_to_cpu(filt->pan_id));
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		struct set_ieee_addr *req;

		req = kmalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		memcpy(&req->ieee_addr, &filt->ieee_addr,
		       sizeof(req->ieee_addr));

		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
					   SET_IEEE_ADDR, req, sizeof(*req));
		kfree(req);
		if (ret < 0) {
			dev_err(&udev->dev, "Failed to set ieee_addr, ret %d",
				ret);
			return ret;
		}

		dev_dbg(&udev->dev, "IEEE addr changed");
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		dev_dbg(&udev->dev, "panc changed");

		dev_err(&udev->dev, "Not handled AFILT_PANC_CHANGED");
	}

	return ret;
}

static int wpanusb_set_extended_addr(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	unsigned char *buffer;
	__le64 extended_addr;
	int ret = 0;
	u64 addr;

	buffer = kmalloc(IEEE802154_EXTENDED_ADDR_LEN, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0), GET_EXTENDED_ADDR, buffer,
					IEEE802154_EXTENDED_ADDR_LEN);
	if (ret < 0) {
		dev_err(&udev->dev, "failed to fetch extended address, random address set\n");
		ieee802154_random_extended_addr(&wpanusb->hw->phy->perm_extended_addr);
		kfree(buffer);
		return ret;
	}

	memcpy(&extended_addr, buffer, IEEE802154_EXTENDED_ADDR_LEN);
	/* Check if read address is not empty and the unicast bit is set correctly */
	if (!ieee802154_is_valid_extended_unicast_addr(extended_addr)) {
		dev_info(&udev->dev, "no permanent extended address found, random address set\n");
		ieee802154_random_extended_addr(&wpanusb->hw->phy->perm_extended_addr);
	} else {
		wpanusb->hw->phy->perm_extended_addr = extended_addr;
		addr = swab64((__force u64)wpanusb->hw->phy->perm_extended_addr);
		dev_info(&udev->dev, "Read permanent extended address %8phC from device\n", &addr);
	}

	kfree(buffer);
	return ret;
}

/* FIXME: these need to come as capabilities from the device */
static const s32 wpanusb_powers[] = {
	300, 280, 230, 180, 130, 70, 0, -100, -200, -300, -400, -500, -700,
	-900, -1200, -1700,
};

static int wpanusb_get_device_capabilities(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	unsigned char *buffer;
	uint32_t valid_channels;
	int ret = 0;

	buffer = kmalloc(IEEE802154_EXTENDED_ADDR_LEN, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0), GET_EXTENDED_ADDR, buffer,
					IEEE802154_EXTENDED_ADDR_LEN);
	if (ret < 0) {
		dev_err(&udev->dev, "failed to fetch extended address, random address set\n");
		ieee802154_random_extended_addr(&wpanusb->hw->phy->perm_extended_addr);
		kfree(buffer);
		return ret;
	}

	buffer = kmalloc(sizeof(valid_channels), GFP_NOIO);
	if (!buffer)
		return -ENOMEM;
	ret = wpanusb_control_recv(wpanusb, GET_SUPPORTED_CHANNELS, buffer,	sizeof(valid_channels));
	if (ret != sizeof(uint32_t)) {
		dev_err(&udev->dev, "failed to fetch supported channels\n");
		kfree(buffer);
		return ret;
	}
	valid_channels = *(uint32_t *)buffer;
	if (!valid_channels) {
		dev_err(&udev->dev, "failed to fetch valid channels, setting default valid channels\n");
		valid_channels = WPANUSB_VALID_CHANNELS;
	}

	/* FIXME: these need to come from device capabilities */
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT;

	/* FIXME: these need to come from device capabilities */
	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	/* Set default and supported channels */
	hw->phy->current_page = 0;
	hw->phy->current_channel = ffs(valid_channels) - 1; //set to lowest valid channel
	hw->phy->supported.channels[0] = valid_channels;

	/* FIXME: these need to come from device capabilities */
	hw->phy->supported.tx_powers = wpanusb_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(wpanusb_powers);
	hw->phy->transmit_power = hw->phy->supported.tx_powers[0];

	kfree(buffer);
	return ret;
}

static int wpanusb_start(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret;

	schedule_delayed_work(&wpanusb->work, 0);

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   START, NULL, 0);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to start ieee802154");
		usb_kill_anchored_urbs(&wpanusb->idle_urbs);
	}

	return ret;
}

static void wpanusb_stop(struct ieee802154_hw *hw)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret;

	dev_dbg(&udev->dev, "stop");

	usb_kill_anchored_urbs(&wpanusb->idle_urbs);

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   STOP, NULL, 0);
	if (ret < 0)
		dev_err(&udev->dev, "Failed to stop ieee802154");
}

static int wpanusb_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;

	dev_err(&udev->dev, "%s: Not handled, mbm %d", __func__, mbm);

	return -ENOTSUPP;
}

static int wpanusb_set_cca_mode(struct ieee802154_hw *hw,
				const struct wpan_phy_cca *cca)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;

	dev_err(&udev->dev, "%s: Not handled, mode %u opt %u",
		__func__, cca->mode, cca->opt);

	switch (cca->mode) {
	case NL802154_CCA_ENERGY:
		break;
	case NL802154_CCA_CARRIER:
		break;
	case NL802154_CCA_ENERGY_CARRIER:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wpanusb_set_lbt(struct ieee802154_hw *hw, bool on)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret = 0;

	if (on)
		ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_LBT, NULL, 0);

	return ret;
}

static int wpanusb_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret;

	/* FIXME pass retries onwards to device */
	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_FRAME_RETRIES, NULL, 0);

	return ret;
}

static int wpanusb_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;

	dev_err(&udev->dev, "%s: Not handled, mbm %d", __func__, mbm);

	return 0;
}

static int wpanusb_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
				   u8 max_be, u8 retries)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;

	dev_err(&udev->dev, "%s: Not handled, min_be %u max_be %u retr %u",
		__func__, min_be, max_be, retries);

	return 0;
}

static int wpanusb_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;

	dev_err(&udev->dev, "%s: Not handled, on %d", __func__, on);

	return 0;
}

static const struct ieee802154_ops wpanusb_ops = {
	.owner			= THIS_MODULE,
	.xmit_async		= wpanusb_xmit,
	.ed			= wpanusb_ed,
	.set_channel		= wpanusb_channel,
	.start			= wpanusb_start,
	.stop			= wpanusb_stop,
	.set_hw_addr_filt	= wpanusb_set_hw_addr_filt,
	.set_txpower		= wpanusb_set_txpower,
	.set_lbt		= wpanusb_set_lbt,
	.set_cca_mode		= wpanusb_set_cca_mode,
	.set_cca_ed_level	= wpanusb_set_cca_ed_level,
	.set_csma_params	= wpanusb_set_csma_params,
	.set_frame_retries	= wpanusb_set_frame_retries,
	.set_promiscuous_mode	= wpanusb_set_promiscuous_mode,
};

/* ----- Setup ------------------------------------------------------------- */

static int wpanusb_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct ieee802154_hw *hw;
	struct wpanusb *wpanusb;
	int ret;

	hw = ieee802154_alloc_hw(sizeof(struct wpanusb), &wpanusb_ops);
	if (!hw)
		return -ENOMEM;

	wpanusb = hw->priv;
	wpanusb->hw = hw;
	wpanusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, wpanusb);

	wpanusb->shutdown = 0;
	INIT_DELAYED_WORK(&wpanusb->work, wpanusb_work_urbs);
	init_usb_anchor(&wpanusb->idle_urbs);
	init_usb_anchor(&wpanusb->rx_urbs);

	ret = wpanusb_alloc_urbs(wpanusb, WPANUSB_NUM_RX_URBS);
	if (ret)
		goto fail;

	wpanusb->tx_dr.bRequestType = VENDOR_OUT;
	wpanusb->tx_dr.bRequest = TX;
	wpanusb->tx_dr.wValue = cpu_to_le16(0);

	wpanusb->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wpanusb->tx_urb)
		goto fail;

	hw->parent = &udev->dev;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0), RESET,
				   NULL, 0);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to RESET ieee802154");
		goto fail;
	}

	ret = wpanusb_get_device_capabilities(hw);

	if (ret < 0) {
		dev_err(&udev->dev, "Failed to get device capabilities");
		goto fail;
	}

	ret = wpanusb_set_extended_addr(hw);

	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set permanent address");
		goto fail;
	}

	ret = ieee802154_register_hw(hw);
	if (ret) {
		dev_err(&udev->dev, "Failed to register ieee802154");
		goto fail;
	}

	dev_dbg(&udev->dev, "ieee802154 ready to go");

	return 0;

fail:
	dev_err(&udev->dev, "Failed ieee802154 probe");
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);
	usb_put_dev(udev);
	ieee802154_free_hw(hw);

	return ret;
}

static void wpanusb_disconnect(struct usb_interface *interface)
{
	struct wpanusb *wpanusb = usb_get_intfdata(interface);

	wpanusb->shutdown = 1;
	cancel_delayed_work_sync(&wpanusb->work);

	usb_kill_anchored_urbs(&wpanusb->rx_urbs);
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);

	ieee802154_unregister_hw(wpanusb->hw);

	ieee802154_free_hw(wpanusb->hw);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(wpanusb->udev);
}

/* The devices we work with */
static const struct usb_device_id wpanusb_device_table[] = {
	{
		USB_DEVICE_AND_INTERFACE_INFO(WPANUSB_VENDOR_ID,
					      WPANUSB_PRODUCT_ID,
					      USB_CLASS_VENDOR_SPEC,
					      0, 0),
                USB_DEVICE_AND_INTERFACE_INFO(BEAGLECONNECT_VENDOR_ID,
                                                BEAGLECONNECT_PRODUCT_ID,
                                                USB_CLASS_VENDOR_SPEC,
                                                0, 0)
	},
	/* end with null element */
	{}
};
MODULE_DEVICE_TABLE(usb, wpanusb_device_table);

static struct usb_driver wpanusb_driver = {
	.name		= "wpanusb",
	.probe		= wpanusb_probe,
	.disconnect	= wpanusb_disconnect,
	.id_table	= wpanusb_device_table,
};
module_usb_driver(wpanusb_driver);

MODULE_AUTHOR("Andrei Emeltchenko <andrei.emeltchenko@intel.com>");
MODULE_DESCRIPTION("WPANUSB IEEE 802.15.4 over USB Driver");
MODULE_LICENSE("GPL");
