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


#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <net/bluetooth/bluetooth.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_irq.h>


#define BT_SDIO_HEADER_LEN			4
#define BT_SDIO_UPLD_SIZE			(251+BT_SDIO_HEADER_LEN)

#define CMD_TYPE_BLE_ENABLE         1



struct btti_worker_thread {
	struct task_struct *task;
	wait_queue_head_t wait_queue;
	void *private_data;
};

struct btti_device {
	void *sdiodev; //see btti_sdio_dev
	struct hci_dev *hcidev;
	u8 dev_type;
};

struct btti_hci_adapter {
	u32 num_of_interrupt;
	struct sk_buff_head tx_queue;
	u8 ble_enable;
	u8 enable_autosuspend;
	bool is_suspended;
	bool is_suspending;
};

struct btti_private {
	struct btti_device btti_dev;
	struct btti_hci_adapter *hci_adapter;
	struct btti_worker_thread work_thread;
	int (*card_tx_packet_funcp)\
			(struct btti_private *private_data,struct sk_buff *skb);
	int (*card_power_up_firmware_funcp)(struct btti_private *private_data);
	int (*card_power_dn_firmware_funcp)(struct btti_private *private_data);
	int (*card_process_rx_funcp)(struct btti_private *private_data);
	spinlock_t irq_cnt_lock;		/* spinlock used by driver */
#ifdef CONFIG_DEBUG_FS
	void *debugfs_dir_vals;
#endif
	bool sdio_dev_removed;
};

/* Prototype of global function */
int btti_hci_register_hdev(struct btti_private *private_data);
struct btti_private *btti_hci_add_sdio_dev(void *sdiodev);
int btti_hci_remove_sdio_dev(struct btti_private *private_data);
void btti_hci_irq_handler(struct btti_private *private_data);

#ifdef CONFIG_DEBUG_FS
void btti_debugfs_init(struct hci_dev *hdev);
void btti_debugfs_remove(struct hci_dev *hdev);
#endif
