/*
 * This file is part of TI BLE over SDIO
 *
 * Copyright (C) 2022 Texas Instruments
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
 */


#include <linux/module.h>
#include <linux/of.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>

#include "btti_drv.h"
#include "btti_sdio.h"

#define VERSION "ti_1.0"

#define BTTI_SDIO_AUTOSUSPEND_DELAY	8000

static int btti_hci_is_ble_enabled(struct btti_private *private_data);


/*
 * This function is called by the interrupt handler.
 * It wakes up the worker thread.to make the RX reception.
 */
void btti_hci_irq_handler(struct btti_private *private_data)
{
	ulong flags;

	spin_lock_irqsave(&private_data->irq_cnt_lock, flags);
	private_data->hci_adapter->num_of_interrupt++;
    spin_unlock_irqrestore(&private_data->irq_cnt_lock, flags);

    //  wakeup to btti_service_work_thread
	wake_up_interruptible(&private_data->work_thread.wait_queue);
}
EXPORT_SYMBOL_GPL(btti_hci_irq_handler);


int btti_debugfs_if_prepare_command(u8 cmd_type,\
		struct btti_private *private_data)
{
	int ret = 0;
	switch(cmd_type){
		case CMD_TYPE_BLE_ENABLE:
			BT_INFO("[bt sdio hci] "\
					"btti_debugfs_if_prepare_command "\
					" CMD_TYPE_BLE_ENABLE ");
			ret = btti_hci_is_ble_enabled(private_data);
		break;
		default:
			break;
	}
	return ret;
}

static int btti_hci_tx_pkt(struct btti_private *private_data,\
		struct sk_buff *skb)
{
	int ret = 0;

	if (!skb || !skb->data)
		return -EINVAL;

	if (!skb->len || ((skb->len + BT_SDIO_HEADER_LEN) > BT_SDIO_UPLD_SIZE))
	{
		BT_ERR("[bt sdio hci] TX Error: Bad skb length %d : %d",
						skb->len, BT_SDIO_UPLD_SIZE);
		return -EINVAL;
	}

	skb_push(skb, BT_SDIO_HEADER_LEN);

	/* header type: byte[3]
	 * HCI_COMMAND = 1, ACL_DATA = 2, SCO_DATA = 3, 0xFE = Vendor
	 * header length: byte[2][1][0]
	 */

	skb->data[0] = (skb->len & 0x0000ff);
	skb->data[1] = (skb->len & 0x00ff00) >> 8;
	skb->data[2] = (skb->len & 0xff0000) >> 16;
	skb->data[3] = hci_skb_pkt_type(skb);


	BT_DBG("[bt sdio hci] TX buff 0x: %*ph", skb->len, skb->data);


	if (private_data->card_tx_packet_funcp)
		ret = private_data->card_tx_packet_funcp(private_data,\
				skb);//btti_sdio_tx_packet

	return ret;
}

static void btti_hci_init_hci_adapter(struct btti_private *private_data)
{

	skb_queue_head_init(&private_data->hci_adapter->tx_queue);
	private_data->hci_adapter->enable_autosuspend = true;
}

static void btti_hci_free_hci_adapter(struct btti_private *private_data)
{
	skb_queue_purge(&private_data->hci_adapter->tx_queue);
	kfree(private_data->hci_adapter);

	private_data->hci_adapter = NULL;
}




//it is on purpose without lock ,
//worse case card_ble_verify_if_ble_enable_funcp will be called more than once
static int btti_hci_is_ble_enabled(struct btti_private *private_data)
{
        //if ble_enabled -- return 0
	return( private_data->hci_adapter->ble_enable);
}


static int btti_hci_if_open(struct hci_dev *hdev)
{
	BT_DBG("[bt sdio hci] btti_hci_if_open");
	return 0;
}

static int btti_hci_if_close(struct hci_dev *hdev)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	BT_DBG("[bt sdio hci] btti_hci_if_close");

	pm_runtime_disable(&sdiodev->func->dev);
	skb_queue_purge(&private_data->hci_adapter->tx_queue);

	return 0;
}

static int btti_hci_if_flush(struct hci_dev *hdev)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);
	BT_DBG("[bt sdio hci] btti_hci_if_flush");

	skb_queue_purge(&private_data->hci_adapter->tx_queue);

	return 0;
}
static int btti_hci_if_tx_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);

	BT_INFO("[bt sdio hci] TX from HCI received ,type=%d,"\
			" opcode: 0x%x len=%d ble_enable=%d",
			hci_skb_pkt_type(skb), hci_skb_opcode(skb),\
			skb->len,private_data->hci_adapter->ble_enable);

	if (private_data->hci_adapter->is_suspending\
			|| private_data->hci_adapter->is_suspended) {
		BT_ERR("[bt sdio hci] %s:"\
				" Device is suspending or suspended", __func__);
		goto fail;
	}

	//check of ble is enabled
	if(!btti_hci_is_ble_enabled(private_data)){
		BT_INFO("[bt sdio hci] ble is not enabled");
		goto fail;
	}


	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		if(hdev){
			hdev->stat.cmd_tx++;
		}
		break;

	case HCI_ACLDATA_PKT:
		if(hdev){
			hdev->stat.acl_tx++;
		}
		break;

	case HCI_SCODATA_PKT:
		BT_WARN("[bt sdio hci] HCI_SCODATA_PKT not supported");
		if(hdev){
			hdev->stat.sco_tx++;
		}
		break;

	default:
		BT_ERR("[bt sdio hci] unknown packet type :%d",\
				hci_skb_pkt_type(skb));
		return -EILSEQ;

	}


	skb_queue_tail(&private_data->hci_adapter->tx_queue, skb);

	if (!private_data->hci_adapter->is_suspended){
		wake_up_interruptible(&private_data->work_thread.wait_queue);
	}else{
		skb_dequeue_tail(&private_data->hci_adapter->tx_queue);
		BT_INFO("[bt sdio hci] Device is suspending");
		goto fail;
	}

	return 0;
fail:
    //in case of error HCI frees the skb
	return -EIO;
}



static int btti_hci_if_setup(struct hci_dev *hdev)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;

	BT_INFO("[bt sdio hci] btti_hci_if_setup");
	pm_runtime_set_autosuspend_delay(&sdiodev->func->dev,
			BTTI_SDIO_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&sdiodev->func->dev);

	pm_runtime_set_active(&sdiodev->func->dev);

	/* Default forbid runtime auto suspend
	 */
	pm_runtime_forbid(&sdiodev->func->dev);
	pm_runtime_enable(&sdiodev->func->dev);


	//auto suspend is allowed
	if (private_data->hci_adapter\
			&& private_data->hci_adapter->enable_autosuspend){
		BT_DBG("[bt sdio hci] btti_hci_if_setup , enable auto suspend");
		pm_runtime_allow(&sdiodev->func->dev);
	}
	return 0;
}

/*
 * This function handles the event generated by firmware, rx data
 * received from firmware, and tx data sent from kernel.
 * work_thread.wait_queue
 */
static int btti_service_work_thread(void *data)
{
	struct btti_worker_thread *thread = data;
	struct btti_private *private_data = thread->private_data;
	struct btti_hci_adapter *hci_adapter = private_data->hci_adapter;
	wait_queue_entry_t wait;
	struct sk_buff *skb;
	ulong flags;

	BT_INFO("[bt sdio hci] work thread is started");

	init_waitqueue_entry(&wait, current);

	for (;;) {
		add_wait_queue(&thread->wait_queue, &wait);

		if (kthread_should_stop() || private_data->sdio_dev_removed) {
			BT_DBG("[bt sdio hci]work_thread: "\
					" thread is need to stopped");
			break;
		}


		set_current_state(TASK_INTERRUPTIBLE);

		//if no RX and queue is empty or ble is not enabled
		if ((!private_data->hci_adapter->ble_enable) ||
				((!hci_adapter->num_of_interrupt) &&\
						skb_queue_empty\
						(&hci_adapter->tx_queue))) {
			BT_INFO("[bt sdio hci] work thread is sleeping...");
			schedule();
		}

		set_current_state(TASK_RUNNING);

		remove_wait_queue(&thread->wait_queue, &wait);

		BT_DBG("[bt sdio hci] work thread woke up");

		if (kthread_should_stop() || private_data->sdio_dev_removed) {
			BT_INFO("[bt sdio hci] work_thread:"\
					" break from main thread");
			break;
		}

		//handle the RX, process the interrupt function
		spin_lock_irqsave(&private_data->irq_cnt_lock, flags);
		if (hci_adapter->num_of_interrupt) {
			hci_adapter->num_of_interrupt = 0;
			spin_unlock_irqrestore(&private_data->irq_cnt_lock,\
					flags);
			if(private_data->card_process_rx_funcp){
				private_data->card_process_rx_funcp\
				(private_data);//call to btti_sdio_process_rx
			}
		} else {
			spin_unlock_irqrestore(&private_data->irq_cnt_lock,\
					flags);
		}

		if (private_data->hci_adapter->is_suspended)
		{
			BT_INFO("[bt sdio hci] work thread not available,"\
					" is_suspended:%d",
					private_data->hci_adapter->is_suspended);
			continue;
		}
		//handle the TX
		skb = skb_dequeue(&hci_adapter->tx_queue);
		if (skb) {
			if (btti_hci_tx_pkt(private_data, skb)){
				//handle tx packet
				BT_ERR("[bt sdio hci] TX , error send packet");
			}
		}
	}

	return 0;
}

int btti_hci_register_hdev(struct btti_private *private_data)

{
	struct hci_dev *hdev = NULL;
	struct btti_sdio_dev *sdiodev = private_data->btti_dev.sdiodev;
	int ret;
	BT_INFO("[bt sdio hci] btti_hci_register_hdev");

	hdev = hci_alloc_dev();
	if (!hdev) {
		BT_ERR("[bt sdio hci] Can not allocate HCI device");
		goto err_hdev;
	}

	private_data->btti_dev.hcidev = hdev;
	hci_set_drvdata(hdev, private_data);

	hdev->bus   = HCI_SDIO;
	hdev->open  = btti_hci_if_open;
	hdev->close = btti_hci_if_close;
	hdev->flush = btti_hci_if_flush;
	hdev->send  = btti_hci_if_tx_frame;
	hdev->setup = btti_hci_if_setup;
	SET_HCIDEV_DEV(hdev, &sdiodev->func->dev);

	set_bit(HCI_QUIRK_NON_PERSISTENT_SETUP, &hdev->quirks);

	hdev->dev_type = HCI_PRIMARY;

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		BT_ERR("[bt sdio hci] Can not register HCI device");
		goto err_hci_register_dev;
	}

#ifdef CONFIG_DEBUG_FS
	btti_debugfs_init(hdev);
#endif

	return 0;

err_hci_register_dev:
	hci_free_dev(hdev);

err_hdev:
	/* Stop the thread servicing the interrupts */
	kthread_stop(private_data->work_thread.task);

	btti_hci_free_hci_adapter(private_data);
	kfree(private_data);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(btti_hci_register_hdev);

struct btti_private *btti_hci_add_sdio_dev(void *sdiodev)
{
	struct btti_private *private_data;
	BT_INFO("[bt sdio hci] btti_hci_add_sdio_dev");


	private_data = kzalloc(sizeof(*private_data), GFP_KERNEL);
	if (!private_data) {
		BT_ERR("[bt sdio hci] Can not allocate private_data");
		goto err_priv;
	}

	private_data->hci_adapter = \
			kzalloc(sizeof(*private_data->hci_adapter), GFP_KERNEL);
	if (!private_data->hci_adapter) {
		BT_ERR("[bt sdio hci] Allocate buffer"\
				" for btti_hci_adapter failed!");
		goto err_hci_adapter;
	}

	btti_hci_init_hci_adapter(private_data);

	BT_INFO("[bt sdio hci] Starting work thread...");
	private_data->work_thread.private_data = private_data;
	spin_lock_init(&private_data->irq_cnt_lock);

	init_waitqueue_head(&private_data->work_thread.wait_queue);
	private_data->work_thread.task = kthread_run(btti_service_work_thread,
				&private_data->work_thread,\
					"btti_main_service");
	if (IS_ERR(private_data->work_thread.task))
		goto err_thread;

	private_data->btti_dev.sdiodev = sdiodev;
	return private_data;

err_thread:
	btti_hci_free_hci_adapter(private_data);

err_hci_adapter:
	kfree(private_data);

err_priv:
	return NULL;
}
EXPORT_SYMBOL_GPL(btti_hci_add_sdio_dev);

int btti_hci_remove_sdio_dev(struct btti_private *private_data)
{
	struct hci_dev *hdev;

	BT_INFO("[bt sdio hci] remove sdio dev");

	hdev = private_data->btti_dev.hcidev;

	if(private_data->work_thread.task)
		kthread_stop(private_data->work_thread.task);


	if(hdev)
	{
		BT_INFO("[bt sdio hci] unregister hci");
#ifdef CONFIG_DEBUG_FS
		btti_debugfs_remove(hdev);
#endif
		hci_unregister_dev(hdev);
		hci_free_dev(hdev);
	}

	private_data->btti_dev.hcidev = NULL;

	btti_hci_free_hci_adapter(private_data);

	kfree(private_data);

	return 0;
}
EXPORT_SYMBOL_GPL(btti_hci_remove_sdio_dev);

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("Texas Instruments Bluetooth driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL v2");
