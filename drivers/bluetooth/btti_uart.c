// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  TI Bluetooth HCI UART driver
 *  Copyright (C) 2023 Texas Instruments
 *
 *  Acknowledgements:
 *  This file is based on btuart.c, which was written by Marcel Holtmann.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <asm/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "h4_recv.h"

#define VERSION "0.84"

struct btti_uart_vnd {
	const struct h4_recv_pkt *recv_pkts;
	int recv_pkts_cnt;
	unsigned int manufacturer;
};

enum state{
	STATE_PROBING = 0,
	STATE_HW_OFF,
	STATE_HW_ON,
	STATE_HW_READY,
	STATE_REMOVED
};

static const char *sm_state_to_string(enum state state)
{
	switch (state){
	case STATE_PROBING:
		return "STATE_PROBING";
	case STATE_HW_OFF:
		return "STATE_HW_OFF";
	case STATE_HW_ON:
		return "STATE_HW_ON";
	case STATE_HW_READY:
		return "STATE_HW_READY";
	case STATE_REMOVED:
		return "STATE_REMOVED";
	}

	return "Illegal state value";
};

enum sm_event{
	EVENT_PROBE_DONE,
	EVENT_REMOVE,
	EVENT_REGULATOR_ENABLE,
	EVENT_REGULATOR_DISABLE,
	EVENT_HCI_WAKEUP_FRAME_RECEIVED,
};

static const char *event_to_string(enum sm_event event)
{
	switch (event){
	case EVENT_PROBE_DONE:
		return "EVENT_PROBE_DONE";
	case EVENT_REMOVE:
		return "EVENT_REMOVE";
	case EVENT_REGULATOR_ENABLE:
		return "EVENT_REGULATOR_ENABLE";
	case EVENT_REGULATOR_DISABLE:
		return "EVENT_REGULATOR_DISABLE";
	case EVENT_HCI_WAKEUP_FRAME_RECEIVED:
		return "EVENT_HCI_WAKEUP_FRAME_RECEIVED";
	}

	return "Illegal event value";
};

struct btti_uart_dev {
	struct hci_dev *hdev;
	struct serdev_device *serdev;
	struct regulator *reg;
	struct notifier_block nb;

	struct work_struct tx_work;
	struct sk_buff_head txq;

	struct work_struct btti_uart_sm_work;
	struct llist_head sm_event_list;
	enum state sm_state;

	struct sk_buff *rx_skb;

	const struct btti_uart_vnd *vnd;

	struct gpio_desc	*host_wakeup;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_runtime;
	struct pinctrl_state *pins_sleep;
};

struct event_node{
	struct llist_node node;
	enum sm_event event;
};

static int serial_open(struct btti_uart_dev *bdev)
{
	struct serdev_device *serdev = bdev->serdev;
	bool disable_flow_control = false;
	u32 max_speed = 3000000;
	int ret=0;

	ret = serdev_device_open(serdev);
	if (ret){
		dev_err(&serdev->dev, "Cannot open serial port (%d)", ret);
		return ret;
	}

	of_property_read_u32(serdev->dev.of_node, "max-speed", &max_speed);
	disable_flow_control = of_property_read_bool(serdev->dev.of_node, "disable-flow-control");

	serdev_device_set_baudrate(serdev, max_speed);
	if (disable_flow_control)
		serdev_device_set_flow_control(serdev, false);

	if (bdev->host_wakeup)
		pm_runtime_enable(&serdev->dev);

	return ret;
}

static void serial_close(struct btti_uart_dev *bdev)
{
	struct serdev_device *serdev = bdev->serdev;

	if (bdev->host_wakeup)
		pm_runtime_disable(&serdev->dev);

	serdev_device_close(serdev);
}

static int btti_uart_open(struct hci_dev *hdev)
{
	return 0;
}

static int btti_uart_close(struct hci_dev *hdev)
{
	return 0;
}

static int btti_uart_flush(struct hci_dev *hdev)
{
	struct btti_uart_dev *bdev = hci_get_drvdata(hdev);

	/* Flush any pending characters */
	serdev_device_write_flush(bdev->serdev);
	skb_queue_purge(&bdev->txq);

	cancel_work_sync(&bdev->tx_work);

	kfree_skb(bdev->rx_skb);
	bdev->rx_skb = NULL;

	return 0;
}

static int btti_uart_setup(struct hci_dev *hdev)
{
	return 0;
}

static int btti_uart_tx_wakeup(struct btti_uart_dev *bdev)
{
	schedule_work(&bdev->tx_work);
	return 0;
}

static int btti_uart_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btti_uart_dev *bdev = hci_get_drvdata(hdev);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
	skb_queue_tail(&bdev->txq, skb);

	btti_uart_tx_wakeup(bdev);
	return 0;
}

static int btti_uart_register_hci_device(struct btti_uart_dev *bdev)
{
	struct serdev_device *serdev = bdev->serdev;
	struct hci_dev *hdev;
	int ret;

	hdev = hci_alloc_dev();
	if (!hdev) {
		dev_err(&serdev->dev, "Can't allocate HCI device");
		return -ENOMEM;
	}

	hci_set_drvdata(hdev, bdev);
	hdev->bus = HCI_UART;
	hdev->open  = btti_uart_open;
	hdev->close = btti_uart_close;
	hdev->flush = btti_uart_flush;
	hdev->setup = btti_uart_setup;
	hdev->send  = btti_uart_send_frame;
	SET_HCIDEV_DEV(hdev, &serdev->dev);

	ret = hci_register_dev(hdev);
	if (ret){
		dev_err(&serdev->dev, "Can't register HCI device (%d)", ret);
		hci_free_dev(hdev);
		hdev = NULL;
	}

	bdev->hdev = hdev;
	return ret;
}

static void btti_uart_unregister_device(struct btti_uart_dev *bdev)
{
	struct hci_dev *hdev = bdev->hdev;

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);

	bdev->hdev = NULL;
}

static void btti_uart_tx_work(struct work_struct *work)
{
	struct btti_uart_dev *bdev = container_of(work, struct btti_uart_dev,
					          tx_work);
	struct serdev_device *serdev = bdev->serdev;
	struct hci_dev *hdev = bdev->hdev;

	while (1) {
		struct sk_buff *skb = skb_dequeue(&bdev->txq);
		int len;

		if (!skb)
			break;

		len = serdev_device_write_buf(serdev, skb->data,
					      skb->len);
		hdev->stat.byte_tx += len;

		skb_pull(skb, len);
		if (skb->len > 0) {
			skb_queue_head(&bdev->txq, skb);
			break;
		}

		switch (hci_skb_pkt_type(skb)) {
		case HCI_COMMAND_PKT:
			hdev->stat.cmd_tx++;
			break;
		case HCI_ACLDATA_PKT:
			hdev->stat.acl_tx++;
			break;
		case HCI_SCODATA_PKT:
			hdev->stat.sco_tx++;
			break;
		}

		kfree_skb(skb);
	}
}

static void unexpected_event(struct serdev_device *serdev,
			     enum state state, enum sm_event event)
{
	dev_err(&serdev->dev, "Unexpected event %s at state %s",
		event_to_string(event), sm_state_to_string(state));
	WARN_ON(1);
}

static void btti_uart_sm_post_event(struct btti_uart_dev *bdev, enum sm_event event)
{
	struct event_node *event_node;

	event_node = kzalloc(sizeof(*event_node), GFP_KERNEL);
	if (unlikely(!event_node)){
		dev_err(&bdev->serdev->dev, "Event allocation failure");
		return;
	}

	event_node->event = event;

	llist_add(&event_node->node, &bdev->sm_event_list);
	schedule_work(&bdev->btti_uart_sm_work);
}

static void sm_process(struct btti_uart_dev *bdev, enum sm_event event)
{
	struct serdev_device *serdev = bdev->serdev;
	enum state next_state = bdev->sm_state;

	switch (bdev->sm_state){
	case STATE_PROBING:
		switch (event){
		case EVENT_PROBE_DONE:
			if (regulator_is_enabled(bdev->reg))
				btti_uart_sm_post_event(bdev, EVENT_REGULATOR_ENABLE);

			next_state = STATE_HW_OFF;
			break;

		default:
			unexpected_event(serdev, bdev->sm_state, event);
		}
		break;

	case STATE_HW_OFF:
		switch(event){
		case EVENT_REGULATOR_ENABLE:
			if (0 == serial_open(bdev))
				next_state = STATE_HW_ON;
			else
				dev_err(&serdev->dev, "Could not open serial port");
			break;

		case EVENT_REMOVE:
			next_state = STATE_REMOVED;
			break;

		case EVENT_REGULATOR_DISABLE:
			break;

		default:
			unexpected_event(serdev, bdev->sm_state, event);
		}
		break;

	case STATE_HW_ON:
		switch(event){
		case EVENT_REMOVE:
		case EVENT_REGULATOR_DISABLE:
			serial_close(bdev);
			next_state = (event == EVENT_REMOVE) ?
						STATE_REMOVED: STATE_HW_OFF;
			break;

		case EVENT_HCI_WAKEUP_FRAME_RECEIVED:
			if (0 == btti_uart_register_hci_device(bdev))
				next_state = STATE_HW_READY;
			break;

		case EVENT_REGULATOR_ENABLE:
			break;

		default:
			unexpected_event(serdev, bdev->sm_state, event);
		}
		break;

	case STATE_HW_READY:
		switch(event){
		case EVENT_REMOVE:
		case EVENT_REGULATOR_DISABLE:
			btti_uart_unregister_device(bdev);
			serial_close(bdev);
			next_state = (event == EVENT_REMOVE) ?
						STATE_REMOVED: STATE_HW_OFF;
			break;

		case EVENT_REGULATOR_ENABLE:
			break;

		default:
			unexpected_event(serdev, bdev->sm_state, event);
		}
		break;

	case STATE_REMOVED:
		switch(event){
		case EVENT_REGULATOR_DISABLE:
		case EVENT_REGULATOR_ENABLE:
		case EVENT_HCI_WAKEUP_FRAME_RECEIVED:
			break;

		default:
			unexpected_event(serdev, bdev->sm_state, event);
		}
		break;
	}

	dev_dbg(&serdev->dev,
		"SM: Got %s, moving from %s to %s",
		event_to_string(event), sm_state_to_string(bdev->sm_state),
		sm_state_to_string(next_state));

	bdev->sm_state = next_state;
}

inline static struct llist_node* get_event_list(struct btti_uart_dev *bdev)
{
	struct llist_node* node;

	node = llist_del_all(&bdev->sm_event_list);
	if (!node)
		return NULL;

	return llist_reverse_order(node);
}

static void btti_uart_sm_work(struct work_struct *work)
{
	struct btti_uart_dev *bdev;
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;

	bdev = container_of(work, struct btti_uart_dev, btti_uart_sm_work);
	event_list = get_event_list(bdev);

	llist_for_each_entry_safe(event_node, tmp, event_list, node){
		sm_process(bdev, event_node->event);
		kfree(event_node);
	}
}

static int btti_uart_wakeup_event_match(struct serdev_device *serdev,
					const u8 *data, size_t count)
{
	struct btti_uart_dev *bdev = serdev_device_get_drvdata(serdev);
	const u8 hw_wakeup_evt[] = {HCI_VENDOR_PKT, 0xff, 0x02, 0x04, 0x2a, 0x00, 0x00};

	if (count < sizeof hw_wakeup_evt)
		return 0; // Reject data until all bytes have been received

	if (count > sizeof hw_wakeup_evt)
		goto error;

	if (0 != memcmp(&hw_wakeup_evt, data, sizeof hw_wakeup_evt))
		goto error;

	btti_uart_sm_post_event(bdev, EVENT_HCI_WAKEUP_FRAME_RECEIVED);
	return count;

error:
	dev_err(&serdev->dev, "Unexpected wakeup pattern");
	print_hex_dump(KERN_DEBUG, "Pattern:",
		       DUMP_PREFIX_NONE, 16, 1, data, count, true);
	return count;
}

static int btti_uart_receive_buf(struct serdev_device *serdev, const u8 *data,
			      size_t count)
{
	struct btti_uart_dev *bdev = serdev_device_get_drvdata(serdev);
	const struct btti_uart_vnd *vnd = bdev->vnd;

	if (unlikely(!bdev->hdev)){
		// Accept only wakeup event until driver is registered with HCI
		return btti_uart_wakeup_event_match(serdev, data, count);
	}

	bdev->rx_skb = h4_recv_buf(bdev->hdev, bdev->rx_skb, data, count,
				   vnd->recv_pkts, vnd->recv_pkts_cnt);
	if (IS_ERR(bdev->rx_skb)) {
		int err = PTR_ERR(bdev->rx_skb);
		dev_err(&serdev->dev, "Frame reassembly failed (%d)", err);
		bdev->rx_skb = NULL;
		print_hex_dump( KERN_DEBUG, "Frame:", DUMP_PREFIX_NONE,
				16, 1, data, count, true);
		return 0;
	}

	bdev->hdev->stat.byte_rx += count;

	return count;
}

static void btti_uart_write_wakeup(struct serdev_device *serdev)
{
	struct btti_uart_dev *bdev = serdev_device_get_drvdata(serdev);

	btti_uart_tx_wakeup(bdev);
}

static const struct serdev_device_ops btti_uart_client_ops = {
	.receive_buf = btti_uart_receive_buf,
	.write_wakeup = btti_uart_write_wakeup,
};

static const struct h4_recv_pkt default_recv_pkts[] = {
	{ H4_RECV_ACL,      .recv = hci_recv_frame },
	{ H4_RECV_SCO,      .recv = hci_recv_frame },
	{ H4_RECV_EVENT,    .recv = hci_recv_frame },
};

static const struct btti_uart_vnd default_vnd = {
	.recv_pkts	= default_recv_pkts,
	.recv_pkts_cnt	= ARRAY_SIZE(default_recv_pkts),
};

static const struct btti_uart_vnd ti_vnd = {
	.recv_pkts	= default_recv_pkts,
	.recv_pkts_cnt	= ARRAY_SIZE(default_recv_pkts),
	.manufacturer	= 13,
};

static int btti_uart_regulator_event(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct btti_uart_dev *bdev = container_of(nb, struct btti_uart_dev, nb);

	if (event & REGULATOR_EVENT_DISABLE) {
		btti_uart_sm_post_event(bdev, EVENT_REGULATOR_DISABLE);
	}

	if (event & REGULATOR_EVENT_ENABLE) {
		btti_uart_sm_post_event(bdev, EVENT_REGULATOR_ENABLE);
	}

	return NOTIFY_OK;
}

static irqreturn_t host_wake_irq(int irq, void *data)
{
	struct btti_uart_dev *bdev = data;
	struct serdev_device *serdev = bdev->serdev;

	dev_info(&serdev->dev, "CC33xx wake IRQ");

	pm_wakeup_event(&serdev->dev, 0);
	pm_system_wakeup();

	return IRQ_HANDLED;
}

static void btti_uart_host_wake_init(struct serdev_device *serdev)
{
	struct btti_uart_dev *bdev = serdev_device_get_drvdata(serdev);
	int ret;

	if (!bdev->pinctrl)
		goto out_err;

	bdev->pins_sleep = pinctrl_lookup_state(bdev->pinctrl, "sleep");
	if (IS_ERR(bdev->pins_sleep))
		bdev->pins_sleep = NULL;

	bdev->host_wakeup = devm_gpiod_get_optional(&serdev->dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(bdev->host_wakeup))
		goto out_err;

	if (device_init_wakeup(&serdev->dev, true) != 0)
		goto out_err;

	ret = devm_request_irq(&serdev->dev, gpiod_to_irq(bdev->host_wakeup),
		host_wake_irq, IRQF_TRIGGER_RISING, "btti_host_wake", bdev);
	if (ret)
		goto out_err;

	ret = enable_irq_wake(gpiod_to_irq(bdev->host_wakeup));
	if (ret < 0)
		goto out_err_disable_wake;

	disable_irq(gpiod_to_irq(bdev->host_wakeup));

	dev_info(&serdev->dev, "Host wakeup enabled");
	return;


out_err_disable_wake:
	device_init_wakeup(&serdev->dev, false);
out_err:
	bdev->host_wakeup = NULL;
	bdev->pins_sleep = NULL;
	dev_info(&serdev->dev, "Host wakeup NOT enabled");
	return;
}

static int btti_uart_probe(struct serdev_device *serdev)
{
	struct btti_uart_dev *bdev;
	int ret;

	bdev = devm_kzalloc(&serdev->dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;

	/* Request the vendor specific data and callbacks */
	bdev->vnd = device_get_match_data(&serdev->dev);
	if (!bdev->vnd)
		bdev->vnd = &default_vnd;

	bdev->serdev = serdev;
	serdev_device_set_drvdata(serdev, bdev);

	/* Using the optional get regulator API as normal get returns a dummy
	if the regulator is not found. */
	bdev->reg = devm_regulator_get_optional(&serdev->dev, "cc33xx");
	if (PTR_ERR(bdev->reg) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (IS_ERR(bdev->reg)) {
		dev_err(&serdev->dev, "can't get regulator");
		return PTR_ERR(bdev->reg);
	}

	bdev->pinctrl = devm_pinctrl_get(&serdev->dev);
	if (!IS_ERR(bdev->pinctrl)){
		bdev->pins_runtime = pinctrl_lookup_state(bdev->pinctrl, "default");
		if (IS_ERR(bdev->pins_runtime)){
			dev_err(&serdev->dev, "can't lookup default pin state");
			return PTR_ERR(bdev->pins_runtime);
		}
	} else {
		bdev->pinctrl = NULL;
		bdev->pins_runtime = NULL;
	}

	btti_uart_host_wake_init(serdev);

	if (bdev->pins_runtime)
		pinctrl_select_state(bdev->pinctrl, bdev->pins_runtime);

	bdev->nb.notifier_call = btti_uart_regulator_event;

	ret = regulator_register_notifier(bdev->reg, &bdev->nb);
	if (ret != 0) {
		dev_err(&serdev->dev,
			"Failed to register regulator notifier (%d)", ret);
		return ret;
	}

	serdev_device_set_client_ops(serdev, &btti_uart_client_ops);

	INIT_WORK(&bdev->tx_work, btti_uart_tx_work);
	skb_queue_head_init(&bdev->txq);

	INIT_WORK(&bdev->btti_uart_sm_work, btti_uart_sm_work);
	init_llist_head(&bdev->sm_event_list);

	btti_uart_sm_post_event(bdev, EVENT_PROBE_DONE);

	return 0;
}

static void btti_uart_remove(struct serdev_device *serdev)
{
	struct btti_uart_dev *bdev = serdev_device_get_drvdata(serdev);

	if (bdev->host_wakeup){
		device_init_wakeup(&serdev->dev, false);
		disable_irq_wake(gpiod_to_irq(bdev->host_wakeup));
	}

	regulator_unregister_notifier(bdev->reg, &bdev->nb);
	btti_uart_sm_post_event(bdev, EVENT_REMOVE);
	flush_work(&bdev->btti_uart_sm_work);
}

static int btti_suspend_device(struct device *dev)
{
	struct btti_uart_dev *bdev = dev_get_drvdata(dev);
	struct serdev_device *serdev = bdev->serdev;
	int ret;

	if (bdev->pins_sleep){
		ret = pinctrl_select_state(bdev->pinctrl, bdev->pins_sleep);
		if (ret < 0)
			goto out_err;
	}

	enable_irq(gpiod_to_irq(bdev->host_wakeup));

	ret = serdev_device_set_rts(serdev, false);
		if (ret < 0)
			goto out_err;

	return 0;

out_err:
	if (bdev->pins_runtime)
		pinctrl_select_state(bdev->pinctrl, bdev->pins_runtime);

	serdev_device_set_rts(serdev, true);

	return ret;
}

static int btti_resume_device(struct device *dev)
{
	struct btti_uart_dev *bdev = dev_get_drvdata(dev);
	struct serdev_device *serdev = bdev->serdev;
	int ret;

	disable_irq_nosync(gpiod_to_irq(bdev->host_wakeup));

	if (bdev->pins_runtime){
		ret = pinctrl_select_state(bdev->pinctrl, bdev->pins_runtime);
		if (ret < 0)
			return ret;
	}

	return serdev_device_set_rts(serdev, true);
}

#ifdef CONFIG_OF
static const struct of_device_id btti_uart_of_match_table[] = {
	{ .compatible = "ti,cc33xx-bt", .data = &ti_vnd },
	{ }
};
MODULE_DEVICE_TABLE(of, btti_uart_of_match_table);
#endif

static const struct dev_pm_ops btti_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(btti_suspend_device, btti_resume_device)
};

static struct serdev_device_driver btti_uart_driver = {
	.probe = btti_uart_probe,
	.remove = btti_uart_remove,
	.driver = {
		.name = "btti",
		.of_match_table = of_match_ptr(btti_uart_of_match_table),
		.pm = &btti_pm_ops
	},
};

module_serdev_device_driver(btti_uart_driver);

MODULE_DESCRIPTION("TI Bluetooth UART driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
