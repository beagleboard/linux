/*
 * linux/net/ipv4/netfilter/ipt_IDLETIMER.c
 *
 * Netfilter module to trigger a timer when packet matches.
 * After timer expires a kevent will be sent.
 *
 * Copyright (C) 2004 Nokia Corporation. All rights reserved.
 * Written by Timo Teras <ext-timo.teras@nokia.com>
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
 */

#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/netfilter.h>
#include <linux/rtnetlink.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter_ipv4/ipt_IDLETIMER.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

#if 0
#define DEBUGP(format, args...) printk("%s:%s:" format, \
                                       __FILE__, __FUNCTION__ , ## args)
#else
#define DEBUGP(format, args...)
#endif

/*
 * Internal timer management.
 */
static ssize_t utimer_attr_show(struct device *, struct device_attribute *attr, char *buf);
static ssize_t utimer_attr_store(struct device *, struct device_attribute *attr,
				 const char *buf, size_t count);

struct utimer_t {
	char name[IFNAMSIZ];
	struct list_head entry;
	struct timer_list timer;
	struct work_struct work;
};

static LIST_HEAD(active_utimer_head);
static DEFINE_SPINLOCK(list_lock);
static DEVICE_ATTR(idletimer, 0644, utimer_attr_show, utimer_attr_store);

static void utimer_delete(struct utimer_t *timer)
{
	DEBUGP("Deleting timer '%s'\n", timer->name);

	list_del(&timer->entry);
	del_timer_sync(&timer->timer);
	kfree(timer);
}

static void utimer_work(struct work_struct *work)
{
	struct utimer_t *timer = container_of(work, struct utimer_t, work);
	struct net_device *netdev;

	netdev = dev_get_by_name(&init_net, timer->name);

	if (netdev != NULL) {
		sysfs_notify(&netdev->dev.kobj, NULL,
			     "idletimer");
		dev_put(netdev);
	}
}

static void utimer_expired(unsigned long data)
{
	struct utimer_t *timer = (struct utimer_t *) data;

	DEBUGP("Timer '%s' expired\n", timer->name);

	spin_lock_bh(&list_lock);
	utimer_delete(timer);
	spin_unlock_bh(&list_lock);

	schedule_work(&timer->work);
}

static struct utimer_t *utimer_create(const char *name)
{
	struct utimer_t *timer;

	timer = kmalloc(sizeof(struct utimer_t), GFP_ATOMIC);
	if (timer == NULL)
		return NULL;

	list_add(&timer->entry, &active_utimer_head);
	strlcpy(timer->name, name, sizeof(timer->name));

	init_timer(&timer->timer);
	timer->timer.function = utimer_expired;
	timer->timer.data = (unsigned long) timer;

	INIT_WORK(&timer->work, utimer_work);

	DEBUGP("Created timer '%s'\n", timer->name);

	return timer;
}

static struct utimer_t *__utimer_find(const char *name)
{
	struct utimer_t *entry;

	list_for_each_entry(entry, &active_utimer_head, entry) {
		if (strcmp(name, entry->name) == 0) {
			return entry;
		}
	}

	return NULL;
}

static void utimer_modify(const char *name,
			  unsigned long expires)
{
	struct utimer_t *timer;

	DEBUGP("Modifying timer '%s'\n", name);
	spin_lock_bh(&list_lock);
	timer = __utimer_find(name);
	if (timer == NULL)
		timer = utimer_create(name);
	mod_timer(&timer->timer, expires);
	spin_unlock_bh(&list_lock);
}

static ssize_t utimer_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct utimer_t *timer;
	unsigned long expires = 0;
	struct net_device *netdev = container_of(dev, struct net_device, dev);

	spin_lock_bh(&list_lock);
	timer = __utimer_find(netdev->name);
	if (timer)
		expires = timer->timer.expires;
	spin_unlock_bh(&list_lock);

	if (expires)
		return sprintf(buf, "%lu\n", (expires-jiffies) / HZ);

	return sprintf(buf, "0\n");
}

static ssize_t utimer_attr_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int expires;
	struct net_device *netdev = container_of(dev, struct net_device, dev);

	if (sscanf(buf, "%d", &expires) == 1) {
		if (expires > 0)
			utimer_modify(netdev->name,
				      jiffies+HZ*(unsigned long)expires);
	}

	return count;
}

static int utimer_notifier_call(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	int ret = NOTIFY_DONE;

	switch (event) {
	case NETDEV_UP:
		DEBUGP("NETDEV_UP: %s\n", dev->name);
		ret = device_create_file(&dev->dev,
					 &dev_attr_idletimer);
		break;
	case NETDEV_DOWN:
		DEBUGP("NETDEV_DOWN: %s\n", dev->name);
		device_remove_file(&dev->dev,
					 &dev_attr_idletimer);
		break;
	}

	return ret;
}

static struct notifier_block utimer_notifier_block = {
	.notifier_call	= utimer_notifier_call,
};


static int utimer_init(void)
{
        return register_netdevice_notifier(&utimer_notifier_block);
}

static void utimer_fini(void)
{
	struct utimer_t *entry, *next;
	struct net_device *dev;

	list_for_each_entry_safe(entry, next, &active_utimer_head, entry)
		utimer_delete(entry);

	rtnl_lock();
	unregister_netdevice_notifier(&utimer_notifier_block);
	for_each_netdev(&init_net, dev)
		utimer_notifier_call(&utimer_notifier_block,
				     NETDEV_DOWN, dev);
	rtnl_unlock();
}

/*
 * The actual iptables plugin.
 */
static unsigned int ipt_idletimer_target(struct sk_buff *pskb,
					 const struct net_device *in,
					 const struct net_device *out,
					 unsigned int hooknum,
					 const struct xt_target *xttarget,
					 const void *targinfo)
{
	struct ipt_idletimer_info *target = (struct ipt_idletimer_info*) targinfo;
	unsigned long expires;

	expires = jiffies + HZ*target->timeout;

	if (in != NULL)
		utimer_modify(in->name, expires);

	if (out != NULL)
		utimer_modify(out->name, expires);

	return XT_CONTINUE;
}

static bool ipt_idletimer_checkentry(const char *tablename,
				    const void *e,
				    const struct xt_target *target,
				    void *targinfo,
				    unsigned int hookmask)
{
	struct ipt_idletimer_info *info =
		(struct ipt_idletimer_info *) targinfo;

	if (info->timeout == 0) {
		DEBUGP("timeout value is zero\n");
		return 0;
	}

	return true;
}

static struct xt_target ipt_idletimer = {
	.name		= "IDLETIMER",
	.target		= ipt_idletimer_target,
	.checkentry	= ipt_idletimer_checkentry,
	.me		= THIS_MODULE,
	.targetsize     = sizeof(struct ipt_idletimer_info),
};

static int __init init(void)
{
	int ret;

	ret = utimer_init();
	if (ret)
		return ret;

	if (xt_register_target(&ipt_idletimer)) {
		utimer_fini();
		return -EINVAL;
	}

	return 0;
}

static void __exit fini(void)
{
	xt_unregister_target(&ipt_idletimer);
	utimer_fini();
}

module_init(init);
module_exit(fini);

MODULE_AUTHOR("Timo Teras <ext-timo.teras@nokia.com>");
MODULE_DESCRIPTION("iptables idletimer target module");
MODULE_LICENSE("GPL");
