/***
 *
 *  stack/rtnet_chrdev.c - implements char device for management interface
 *
 *  Copyright (C) 1999       Lineo, Inc
 *                1999, 2002 David A. Schleef <ds@schleef.org>
 *                2002       Ulrich Marx <marx@fet.uni-hannover.de>
 *                2003-2005  Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/if_arp.h>
#include <linux/kmod.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>

#include <rtnet_chrdev.h>
#include <rtnet_internal.h>
#include <ipv4/route.h>


static DEFINE_SPINLOCK(ioctl_handler_lock);
static LIST_HEAD(ioctl_handlers);

static long rtnet_ioctl(struct file *file,
			unsigned int request, unsigned long arg)
{
    struct rtnet_ioctl_head head;
    struct rtnet_device     *rtdev = NULL;
    struct rtnet_ioctls     *ioctls;
    struct list_head        *entry;
    int                     ret;


    if (!capable(CAP_SYS_ADMIN))
	return -EPERM;

    ret = copy_from_user(&head, (void *)arg, sizeof(head));
    if (ret != 0)
	return -EFAULT;

    spin_lock(&ioctl_handler_lock);

    list_for_each(entry, &ioctl_handlers) {
	ioctls = list_entry(entry, struct rtnet_ioctls, entry);

	if (ioctls->ioctl_type == _IOC_TYPE(request)) {
	    atomic_inc(&ioctls->ref_count);

	    spin_unlock(&ioctl_handler_lock);

	    if ((_IOC_NR(request) & RTNET_IOC_NODEV_PARAM) == 0) {
		rtdev = rtdev_get_by_name(head.if_name);
		if (!rtdev) {
		    atomic_dec(&ioctls->ref_count);
		    return -ENODEV;
		}
	    }

	    ret = ioctls->handler(rtdev, request, arg);

	    if (rtdev)
		rtdev_dereference(rtdev);
	    atomic_dec(&ioctls->ref_count);

	    return ret;
	}
    }

    spin_unlock(&ioctl_handler_lock);

    return -ENOTTY;
}



static int rtnet_core_ioctl(struct rtnet_device *rtdev, unsigned int request,
			    unsigned long arg)
{
    struct rtnet_core_cmd   cmd;
    struct list_head        *entry;
    struct rtdev_event_hook *hook;
    int                     ret;
    rtdm_lockctx_t          context;


    ret = copy_from_user(&cmd, (void *)arg, sizeof(cmd));
    if (ret != 0)
	return -EFAULT;

    switch (request) {
	case IOC_RT_IFUP:
	    if (mutex_lock_interruptible(&rtdev->nrt_lock))
		return -ERESTARTSYS;

	    /* We cannot change the promisc flag or the hardware address if
	       the device is already up. */
	    if ((rtdev->flags & IFF_UP) &&
		(((cmd.args.up.set_dev_flags | cmd.args.up.clear_dev_flags) &
		  IFF_PROMISC) ||
		 (cmd.args.up.dev_addr_type != ARPHRD_VOID))) {
		ret = -EBUSY;
		goto up_out;
	    }

	    rtdev->flags |= cmd.args.up.set_dev_flags;
	    rtdev->flags &= ~cmd.args.up.clear_dev_flags;

	    if (cmd.args.up.dev_addr_type != ARPHRD_VOID) {
		if (cmd.args.up.dev_addr_type != rtdev->type) {
		    ret = -EINVAL;
		    goto up_out;
		}
		memcpy(rtdev->dev_addr, cmd.args.up.dev_addr, MAX_ADDR_LEN);
	    }

	    set_bit(PRIV_FLAG_UP, &rtdev->priv_flags);

	    ret = rtdev_open(rtdev);    /* also == 0 if rtdev is already up */

	    if (ret == 0) {
		mutex_lock(&rtnet_devices_nrt_lock);

		list_for_each(entry, &event_hook_list) {
		    hook = list_entry(entry, struct rtdev_event_hook, entry);
		    if (hook->ifup)
			hook->ifup(rtdev, &cmd);
		}

		mutex_unlock(&rtnet_devices_nrt_lock);
	    } else
		clear_bit(PRIV_FLAG_UP, &rtdev->priv_flags);

	  up_out:
	    mutex_unlock(&rtdev->nrt_lock);
	    break;

	case IOC_RT_IFDOWN:
	    if (mutex_lock_interruptible(&rtdev->nrt_lock))
		return -ERESTARTSYS;

	    /* spin lock required for sync with routing code */
	    rtdm_lock_get_irqsave(&rtdev->rtdev_lock, context);

	    if (test_bit(PRIV_FLAG_ADDING_ROUTE, &rtdev->priv_flags)) {
		rtdm_lock_put_irqrestore(&rtdev->rtdev_lock, context);

		mutex_unlock(&rtdev->nrt_lock);
		return -EBUSY;
	    }
	    clear_bit(PRIV_FLAG_UP, &rtdev->priv_flags);

	    rtdm_lock_put_irqrestore(&rtdev->rtdev_lock, context);

	    ret = 0;
	    if (rtdev->mac_detach != NULL)
		ret = rtdev->mac_detach(rtdev);

	    if (ret == 0) {
		mutex_lock(&rtnet_devices_nrt_lock);

		list_for_each(entry, &event_hook_list) {
		    hook = list_entry(entry, struct rtdev_event_hook, entry);
		    if (hook->ifdown)
			hook->ifdown(rtdev);
		}

		mutex_unlock(&rtnet_devices_nrt_lock);

		ret = rtdev_close(rtdev);
	    }

	    mutex_unlock(&rtdev->nrt_lock);
	    break;

	case IOC_RT_IFINFO:
	    if (cmd.args.info.ifindex > 0)
		rtdev = rtdev_get_by_index(cmd.args.info.ifindex);
	    else
		rtdev = rtdev_get_by_name(cmd.head.if_name);
	    if (rtdev == NULL)
		return -ENODEV;

	    if (mutex_lock_interruptible(&rtdev->nrt_lock)) {
		rtdev_dereference(rtdev);
		return -ERESTARTSYS;
	    }

	    memcpy(cmd.head.if_name, rtdev->name, IFNAMSIZ);
	    cmd.args.info.ifindex      = rtdev->ifindex;
	    cmd.args.info.type         = rtdev->type;
	    cmd.args.info.ip_addr      = rtdev->local_ip;
	    cmd.args.info.broadcast_ip = rtdev->broadcast_ip;
	    cmd.args.info.mtu          = rtdev->mtu;
	    cmd.args.info.flags        = rtdev->flags;
            if ((cmd.args.info.flags & IFF_UP)
		    && (rtdev->link_state
			    & (RTNET_LINK_STATE_PRESENT
				    | RTNET_LINK_STATE_NOCARRIER))
                    == RTNET_LINK_STATE_PRESENT)
                    cmd.args.info.flags |= IFF_RUNNING;

	    memcpy(cmd.args.info.dev_addr, rtdev->dev_addr, MAX_ADDR_LEN);

	    mutex_unlock(&rtdev->nrt_lock);

	    rtdev_dereference(rtdev);

	    if (copy_to_user((void *)arg, &cmd, sizeof(cmd)) != 0)
		return -EFAULT;
	    break;

	default:
	    ret = -ENOTTY;
    }

    return ret;
}



int rtnet_register_ioctls(struct rtnet_ioctls *ioctls)
{
    struct list_head    *entry;
    struct rtnet_ioctls *registered_ioctls;


    RTNET_ASSERT(ioctls->handler != NULL, return -EINVAL;);

    spin_lock(&ioctl_handler_lock);

    list_for_each(entry, &ioctl_handlers) {
	registered_ioctls = list_entry(entry, struct rtnet_ioctls, entry);
	if (registered_ioctls->ioctl_type == ioctls->ioctl_type) {
	    spin_unlock(&ioctl_handler_lock);
	    return -EEXIST;
	}
    }

    list_add_tail(&ioctls->entry, &ioctl_handlers);
    atomic_set(&ioctls->ref_count, 0);

    spin_unlock(&ioctl_handler_lock);

    return 0;
}



void rtnet_unregister_ioctls(struct rtnet_ioctls *ioctls)
{
    spin_lock(&ioctl_handler_lock);

    while (atomic_read(&ioctls->ref_count) != 0) {
	spin_unlock(&ioctl_handler_lock);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(1*HZ); /* wait a second */

	spin_lock(&ioctl_handler_lock);
    }

    list_del(&ioctls->entry);

    spin_unlock(&ioctl_handler_lock);
}



static struct file_operations rtnet_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = rtnet_ioctl,
};

static struct miscdevice rtnet_chr_misc_dev = {
    .minor= RTNET_MINOR,
    .name = "rtnet",
    .fops = &rtnet_fops,
};

static struct rtnet_ioctls core_ioctls = {
    .service_name = "RTnet Core",
    .ioctl_type =   RTNET_IOC_TYPE_CORE,
    .handler =      rtnet_core_ioctl
};



/**
 * rtnet_chrdev_init -
 *
 */
int __init rtnet_chrdev_init(void)
{
    int err;

    err = misc_register(&rtnet_chr_misc_dev);
    if (err) {
	printk("RTnet: unable to register rtnet management device/class "
	       "(error %d)\n", err);
	return err;
    }

    rtnet_register_ioctls(&core_ioctls);
    return 0;
}



/**
 * rtnet_chrdev_release -
 *
 */
void rtnet_chrdev_release(void)
{
    misc_deregister(&rtnet_chr_misc_dev);
}


EXPORT_SYMBOL_GPL(rtnet_register_ioctls);
EXPORT_SYMBOL_GPL(rtnet_unregister_ioctls);
