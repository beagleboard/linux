/*
 * Copyright (C) 2006 Jan Kiszka <jan.kiszka@web.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


#include <linux/module.h>
#include <rtdm/driver.h>
#include <rtdm/can.h>
#include "rtcan_dev.h"
#include "rtcan_raw.h"

#define RTCAN_DEV_NAME          "rtcan%d"
#define RTCAN_DRV_NAME          "VIRT"
#define RTCAN_MAX_VIRT_DEVS     8

#define VIRT_TX_BUFS            1

static char *virt_ctlr_name  = "<virtual>";
static char *virt_board_name = "<virtual>";

MODULE_AUTHOR("Jan Kiszka <jan.kiszka@web.de>");
MODULE_DESCRIPTION("Virtual RT-Socket-CAN driver");
MODULE_LICENSE("GPL");

static unsigned int devices = 2;

module_param(devices, uint, 0400);
MODULE_PARM_DESC(devices, "Number of devices on the virtual bus");

static struct rtcan_device *rtcan_virt_devs[RTCAN_MAX_VIRT_DEVS];


static int rtcan_virt_start_xmit(struct rtcan_device *tx_dev,
				 can_frame_t *tx_frame)
{
	int i;
	struct rtcan_device *rx_dev;
	struct rtcan_skb skb;
	struct rtcan_rb_frame *rx_frame = &skb.rb_frame;
	rtdm_lockctx_t lock_ctx;

	/* we can transmit immediately again */
	rtdm_sem_up(&tx_dev->tx_sem);

	skb.rb_frame_size = EMPTY_RB_FRAME_SIZE;

	rx_frame->can_dlc = tx_frame->can_dlc;
	rx_frame->can_id  = tx_frame->can_id;

	if (!(tx_frame->can_id & CAN_RTR_FLAG)) {
		memcpy(rx_frame->data, tx_frame->data, tx_frame->can_dlc);
		skb.rb_frame_size += tx_frame->can_dlc;
	}

	rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);
	rtdm_lock_get(&rtcan_socket_lock);


	/* Deliver to all other devices on the virtual bus */
	for (i = 0; i < devices; i++) {
		rx_dev = rtcan_virt_devs[i];
		if (rx_dev->state == CAN_STATE_ACTIVE) {
			if (tx_dev != rx_dev) {
				rx_frame->can_ifindex = rx_dev->ifindex;
				rtcan_rcv(rx_dev, &skb);
			} else if (rtcan_loopback_pending(tx_dev))
				rtcan_loopback(tx_dev);
		}
	}
	rtdm_lock_put(&rtcan_socket_lock);
	rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);

	return 0;
}


static int rtcan_virt_set_mode(struct rtcan_device *dev, can_mode_t mode,
			       rtdm_lockctx_t *lock_ctx)
{
	int err = 0;

	switch (mode) {
	case CAN_MODE_STOP:
		dev->state = CAN_STATE_STOPPED;
		/* Wake up waiting senders */
		rtdm_sem_destroy(&dev->tx_sem);
		break;

	case CAN_MODE_START:
		rtdm_sem_init(&dev->tx_sem, VIRT_TX_BUFS);
		dev->state = CAN_STATE_ACTIVE;
		break;

	default:
		err = -EOPNOTSUPP;
	}

	return err;
}


static int __init rtcan_virt_init_one(int idx)
{
	struct rtcan_device *dev;
	int err;

	if ((dev = rtcan_dev_alloc(0, 0)) == NULL)
		return -ENOMEM;

	dev->ctrl_name = virt_ctlr_name;
	dev->board_name = virt_board_name;

	rtcan_virt_set_mode(dev, CAN_MODE_STOP, NULL);

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	dev->hard_start_xmit = rtcan_virt_start_xmit;
	dev->do_set_mode = rtcan_virt_set_mode;

	/* Register RTDM device */
	err = rtcan_dev_register(dev);
	if (err) {
	    printk(KERN_ERR "ERROR %d while trying to register RTCAN device!\n", err);
		goto error_out;
	}

	/* Remember initialized devices */
	rtcan_virt_devs[idx] = dev;

	printk("%s: %s driver loaded\n", dev->name, RTCAN_DRV_NAME);

	return 0;

 error_out:
	rtcan_dev_free(dev);
	return err;
}


/** Init module */
static int __init rtcan_virt_init(void)
{
	int i, err = 0;

	if (!rtdm_available())
		return -ENOSYS;

	for (i = 0; i < devices; i++) {
		err = rtcan_virt_init_one(i);
		if (err) {
			while (--i >= 0) {
				struct rtcan_device *dev = rtcan_virt_devs[i];

				rtcan_dev_unregister(dev);
				rtcan_dev_free(dev);
			}
			break;
		}
	}

	return err;
}


/** Cleanup module */
static void __exit rtcan_virt_exit(void)
{
	int i;
	struct rtcan_device *dev;

	for (i = 0; i < devices; i++) {
		dev = rtcan_virt_devs[i];

		printk("Unloading %s device %s\n", RTCAN_DRV_NAME, dev->name);

		rtcan_virt_set_mode(dev, CAN_MODE_STOP, NULL);
		rtcan_dev_unregister(dev);
		rtcan_dev_free(dev);
	}
}

module_init(rtcan_virt_init);
module_exit(rtcan_virt_exit);
