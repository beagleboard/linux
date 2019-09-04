/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from RTnet project file stack/rtcan_module.c:
 *
 * Copyright (C) 2002      Ulrich Marx <marx@kammer.uni-hannover.de>
 *               2003-2006 Jan Kiszka <jan.kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <rtdm/driver.h>
#include <rtdm/can.h>
#include <rtcan_version.h>
#include <rtcan_internal.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>

MODULE_LICENSE("GPL");


const char rtcan_rtdm_provider_name[] =
    "(C) 2006 RT-Socket-CAN Development Team";


#ifdef CONFIG_PROC_FS

struct proc_dir_entry *rtcan_proc_root;

static void rtcan_dev_get_ctrlmode_name(can_ctrlmode_t ctrlmode,
					char* name, int max_len)
{
    *name = '\0';
    if (ctrlmode & CAN_CTRLMODE_LISTENONLY)
	strncat(name, "listen-only ", max_len);
    if (ctrlmode & CAN_CTRLMODE_LOOPBACK)
	strncat(name, "loopback ", max_len);
}

static char *rtcan_state_names[] = {
    "active", "warning", "passive" , "bus-off",
    "scanning", "stopped", "sleeping"
};

static void rtcan_dev_get_state_name(can_state_t state,
				     char* name, int max_len)
{
    if (state >= CAN_STATE_ACTIVE &&
	state <= CAN_STATE_SLEEPING)
	strncpy(name, rtcan_state_names[state], max_len);
    else
	strncpy(name, "unknown", max_len);
}

static void rtcan_dev_get_baudrate_name(can_baudrate_t baudrate,
					char* name, int max_len)
{
    switch (baudrate) {
    case CAN_BAUDRATE_UNCONFIGURED:
	strncpy(name, "undefined", max_len);
	break;
    case CAN_BAUDRATE_UNKNOWN:
	strncpy(name, "unknown", max_len);
	break;
    default:
	ksformat(name, max_len, "%d", baudrate);
	break;
    }
}

static void rtcan_dev_get_bittime_name(struct can_bittime *bit_time,
				       char* name, int max_len)
{
    switch (bit_time->type) {
    case CAN_BITTIME_STD:
	ksformat(name, max_len,
		 "brp=%d prop_seg=%d phase_seg1=%d "
		 "phase_seg2=%d sjw=%d sam=%d",
		 bit_time->std.brp,
		 bit_time->std.prop_seg,
		 bit_time->std.phase_seg1,
		 bit_time->std.phase_seg2,
		 bit_time->std.sjw,
		 bit_time->std.sam);
	break;
    case CAN_BITTIME_BTR:
	ksformat(name, max_len, "btr0=0x%02x btr1=0x%02x",
		 bit_time->btr.btr0, bit_time->btr.btr1);
	break;
    default:
	strncpy(name, "unknown", max_len);
	break;
    }
}

static void rtcan_get_timeout_name(nanosecs_rel_t timeout,
				   char* name, int max_len)
{
    if (timeout == RTDM_TIMEOUT_INFINITE)
	strncpy(name, "infinite", max_len);
    else
	ksformat(name, max_len, "%lld", (long long)timeout);
}

static int rtcan_read_proc_devices(struct seq_file *p, void *data)
{
    int i;
    struct rtcan_device *dev;
    char state_name[20], baudrate_name[20];

    if (down_interruptible(&rtcan_devices_nrt_lock))
	return -ERESTARTSYS;

    /* Name___________ _Baudrate State___ _TX_Counts _TX_Counts ____Errors
     * rtcan0             125000 stopped  1234567890 1234567890 1234567890
     * rtcan1          undefined warning  1234567890 1234567890 1234567890
     * rtcan2          undefined scanning 1234567890 1234567890 1234567890
     */
    seq_printf(p, "Name___________ _Baudrate State___ TX_Counter RX_Counter "
		  "____Errors\n");

    for (i = 1; i <= RTCAN_MAX_DEVICES; i++) {
	if ((dev = rtcan_dev_get_by_index(i)) != NULL) {
	    rtcan_dev_get_state_name(dev->state,
				     state_name, sizeof(state_name));
	    rtcan_dev_get_baudrate_name(dev->baudrate,
					baudrate_name, sizeof(baudrate_name));
	    seq_printf(p, "%-15s %9s %-8s %10d %10d %10d\n",
		       dev->name, baudrate_name, state_name, dev->tx_count,
		       dev->rx_count, dev->err_count);
	    rtcan_dev_dereference(dev);
	}
    }

    up(&rtcan_devices_nrt_lock);

    return 0;
}

static int rtcan_proc_devices_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_read_proc_devices, NULL);
}

static const struct file_operations rtcan_proc_devices_ops = {
	.open		= rtcan_proc_devices_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rtcan_read_proc_sockets(struct seq_file *p, void *data)
{
    struct rtcan_socket *sock;
    struct rtdm_fd *fd;
    struct rtcan_device *dev;
    char name[IFNAMSIZ] = "not-bound";
    char rx_timeout[20], tx_timeout[20];
    rtdm_lockctx_t lock_ctx;
    int ifindex;

    if (down_interruptible(&rtcan_devices_nrt_lock))
	return -ERESTARTSYS;

    /* Name___________ Filter ErrMask RX_Timeout TX_Timeout RX_BufFull TX_Lo
     * rtcan0               1 0x00010 1234567890 1234567890 1234567890 12345
     */
    seq_printf(p, "Name___________ Filter ErrMask RX_Timeout_ns "
		  "TX_Timeout_ns RX_BufFull TX_Lo\n");

    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);

    list_for_each_entry(sock, &rtcan_socket_list, socket_list) {
	fd = rtcan_socket_to_fd(sock);
	if (rtcan_sock_is_bound(sock)) {
	    ifindex = atomic_read(&sock->ifindex);
	    if (ifindex) {
		dev = rtcan_dev_get_by_index(ifindex);
		if (dev) {
		    strncpy(name, dev->name, IFNAMSIZ);
		    rtcan_dev_dereference(dev);
		}
	    } else
		ksformat(name, sizeof(name), "%d", ifindex);
	}
	rtcan_get_timeout_name(sock->tx_timeout,
			       tx_timeout, sizeof(tx_timeout));
	rtcan_get_timeout_name(sock->rx_timeout,
			       rx_timeout, sizeof(rx_timeout));
	seq_printf(p, "%-15s %6d 0x%05x %13s %13s %10d %5d\n",
		   name, sock->flistlen, sock->err_mask,
		   rx_timeout, tx_timeout, sock->rx_buf_full,
		   rtcan_loopback_enabled(sock));
    }

    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);

    up(&rtcan_devices_nrt_lock);

    return 0;
}

static int rtcan_proc_sockets_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_read_proc_sockets, NULL);
}

static const struct file_operations rtcan_proc_sockets_ops = {
	.open		= rtcan_proc_sockets_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int rtcan_read_proc_info(struct seq_file *p, void *data)
{
    struct rtcan_device *dev = p->private;
    char state_name[20], baudrate_name[20];
    char ctrlmode_name[80], bittime_name[80];

    if (down_interruptible(&rtcan_devices_nrt_lock))
	return -ERESTARTSYS;

    rtcan_dev_get_state_name(dev->state,
			     state_name, sizeof(state_name));
    rtcan_dev_get_ctrlmode_name(dev->ctrl_mode,
				ctrlmode_name, sizeof(ctrlmode_name));
    rtcan_dev_get_baudrate_name(dev->baudrate,
				baudrate_name, sizeof(baudrate_name));
    rtcan_dev_get_bittime_name(&dev->bit_time,
			       bittime_name, sizeof(bittime_name));

    seq_printf(p, "Device     %s\n", dev->name);
    seq_printf(p, "Controller %s\n", dev->ctrl_name);
    seq_printf(p, "Board      %s\n", dev->board_name);
    seq_printf(p, "Clock-Hz   %d\n", dev->can_sys_clock);
    seq_printf(p, "Baudrate   %s\n", baudrate_name);
    seq_printf(p, "Bit-time   %s\n", bittime_name);
    seq_printf(p, "Ctrl-Mode  %s\n", ctrlmode_name);
    seq_printf(p, "State      %s\n", state_name);
    seq_printf(p, "TX-Counter %d\n", dev->tx_count);
    seq_printf(p, "RX-Counter %d\n", dev->rx_count);
    seq_printf(p, "Errors     %d\n", dev->err_count);
#ifdef RTCAN_USE_REFCOUNT
    seq_printf(p, "Refcount   %d\n", atomic_read(&dev->refcount));
#endif

    up(&rtcan_devices_nrt_lock);

    return 0;
}

static int rtcan_proc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_read_proc_info, PDE_DATA(inode));
}

static const struct file_operations rtcan_proc_info_ops = {
	.open		= rtcan_proc_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};



static int rtcan_read_proc_filter(struct seq_file *p, void *data)
{
    struct rtcan_device *dev = p->private;
    struct rtcan_recv *recv_listener = dev->recv_list;
    struct rtdm_fd *fd;
    rtdm_lockctx_t lock_ctx;

    /*  __CAN_ID__ _CAN_Mask_ Inv MatchCount
     *  0x12345678 0x12345678  no 1234567890
     */

    seq_printf(p, "__CAN_ID__ _CAN_Mask_ Inv MatchCount\n");

    rtdm_lock_get_irqsave(&rtcan_recv_list_lock, lock_ctx);

    /* Loop over the reception list of the device */
    while (recv_listener != NULL) {
	fd = rtcan_socket_to_fd(recv_listener->sock);

	seq_printf(p, "0x%08x 0x%08x %s %10d\n",
		   recv_listener->can_filter.can_id,
		   recv_listener->can_filter.can_mask & ~CAN_INV_FILTER,
		   (recv_listener->can_filter.can_mask & CAN_INV_FILTER) ?
			"yes" : " no",
		   recv_listener->match_count);

	recv_listener = recv_listener->next;
    }

    rtdm_lock_put_irqrestore(&rtcan_recv_list_lock, lock_ctx);

    return 0;
}

static int rtcan_proc_filter_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_read_proc_filter, PDE_DATA(inode));
}

static const struct file_operations rtcan_proc_filter_ops = {
	.open		= rtcan_proc_filter_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};



static int rtcan_read_proc_version(struct seq_file *p, void *data)
{
	seq_printf(p, "RT-Socket-CAN %d.%d.%d\n",
		   RTCAN_MAJOR_VER, RTCAN_MINOR_VER, RTCAN_BUGFIX_VER);

	return 0;
}

static int rtcan_proc_version_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_read_proc_version, NULL);
}

static const struct file_operations rtcan_proc_version_ops = {
	.open		= rtcan_proc_version_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


void rtcan_dev_remove_proc(struct rtcan_device* dev)
{
    if (!dev->proc_root)
	return;

    remove_proc_entry("info", dev->proc_root);
    remove_proc_entry("filters", dev->proc_root);
    remove_proc_entry(dev->name, rtcan_proc_root);

    dev->proc_root = NULL;
}

int rtcan_dev_create_proc(struct rtcan_device* dev)
{
    if (!rtcan_proc_root)
	return -EINVAL;

    dev->proc_root = proc_mkdir(dev->name, rtcan_proc_root);
    if (!dev->proc_root) {
	printk("%s: unable to create /proc device entries\n", dev->name);
	return -1;
    }

    proc_create_data("info", S_IFREG | S_IRUGO | S_IWUSR, dev->proc_root,
		     &rtcan_proc_info_ops, dev);
    proc_create_data("filters", S_IFREG | S_IRUGO | S_IWUSR, dev->proc_root,
		     &rtcan_proc_filter_ops, dev);
    return 0;

}


static int rtcan_proc_register(void)
{
    rtcan_proc_root = proc_mkdir("rtcan", NULL);
    if (!rtcan_proc_root) {
	printk("rtcan: unable to initialize /proc entries\n");
	return -1;
    }

    proc_create("devices", S_IFREG | S_IRUGO | S_IWUSR, rtcan_proc_root,
		&rtcan_proc_devices_ops);
    proc_create("version", S_IFREG | S_IRUGO | S_IWUSR, rtcan_proc_root,
		&rtcan_proc_version_ops);
    proc_create("sockets", S_IFREG | S_IRUGO | S_IWUSR, rtcan_proc_root,
		&rtcan_proc_sockets_ops);
    return 0;
}



static void rtcan_proc_unregister(void)
{
    remove_proc_entry("devices", rtcan_proc_root);
    remove_proc_entry("version", rtcan_proc_root);
    remove_proc_entry("sockets", rtcan_proc_root);
    remove_proc_entry("rtcan", 0);
}
#endif  /* CONFIG_PROC_FS */



int __init rtcan_init(void)
{
    int err = 0;

    if (!realtime_core_enabled())
	    return 0;

    printk("RT-Socket-CAN %d.%d.%d - %s\n",
	   RTCAN_MAJOR_VER, RTCAN_MINOR_VER, RTCAN_BUGFIX_VER,
	   rtcan_rtdm_provider_name);

    if ((err = rtcan_raw_proto_register()) != 0)
	goto out;

#ifdef CONFIG_PROC_FS
    if ((err = rtcan_proc_register()) != 0)
	goto out;
#endif

 out:
    return err;
}


void __exit rtcan_exit(void)
{
    if (!realtime_core_enabled())
	    return;

    rtcan_raw_proto_unregister();
#ifdef CONFIG_PROC_FS
    rtcan_proc_unregister();
#endif

    printk("rtcan: unloaded\n");
}


module_init(rtcan_init);
module_exit(rtcan_exit);
