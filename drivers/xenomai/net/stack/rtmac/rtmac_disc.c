/***
 *
 *  rtmac_disc.c
 *
 *  rtmac - real-time networking media access control subsystem
 *  Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003, 2004 Jan Kiszka <Jan.Kiszka@web.de>
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/mutex.h>

#include <rtnet_internal.h>
#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_proc.h>
#include <rtmac/rtmac_vnic.h>



static DEFINE_MUTEX(disc_list_lock);
static LIST_HEAD(disc_list);



/***
 *  rtmac_disc_attach
 *
 *  @rtdev       attaches a discipline to a device
 *  @disc        discipline to attach
 *
 *  0            success
 *  -EBUSY       other discipline active
 *  -ENOMEM      could not allocate memory
 *
 *  Note: must be called with rtdev->nrt_lock acquired
 */
int rtmac_disc_attach(struct rtnet_device *rtdev, struct rtmac_disc *disc)
{
    int                 ret;
    struct rtmac_priv   *priv;


    RTNET_ASSERT(rtdev != NULL, return -EINVAL;);
    RTNET_ASSERT(disc != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->attach != NULL, return -EINVAL;);

    if (rtdev->mac_disc) {
	printk("RTmac: another discipline for rtdev '%s' active.\n", rtdev->name);
	return -EBUSY;
    }

    if (rtdev->flags & IFF_LOOPBACK)
	return -EINVAL;

    if (!try_module_get(disc->owner))
	return -EIDRM;

    if (!rtdev_reference(rtdev)) {
	ret = -EIDRM;
	goto err_module_put;
    }

    /* alloc memory */
    priv = kmalloc(sizeof(struct rtmac_priv) + disc->priv_size, GFP_KERNEL);
    if (!priv) {
	printk("RTmac: kmalloc returned NULL for rtmac!\n");
	return -ENOMEM;
    }
    priv->orig_start_xmit = rtdev->start_xmit;

    /* call attach function of discipline */
    ret = disc->attach(rtdev, priv->disc_priv);
    if (ret < 0)
	goto err_kfree_priv;

    /* now attach RTmac to device */
    rtdev->mac_disc = disc;
    rtdev->mac_priv = priv;
    rtdev->start_xmit = disc->rt_packet_tx;
    if (disc->get_mtu)
	rtdev->get_mtu = disc->get_mtu;
    rtdev->mac_detach = rtmac_disc_detach;

    /* create the VNIC */
    ret = rtmac_vnic_add(rtdev, disc->vnic_xmit);
    if (ret < 0) {
	printk("RTmac: Warning, VNIC creation failed for rtdev %s.\n", rtdev->name);
	goto err_disc_detach;
    }

    return 0;

  err_disc_detach:
    disc->detach(rtdev, priv->disc_priv);
  err_kfree_priv:
    kfree(priv);
    rtdev_dereference(rtdev);
  err_module_put:
    module_put(disc->owner);
    return ret;
}



/***
 *  rtmac_disc_detach
 *
 *  @rtdev       detaches a discipline from a device
 *
 *  0            success
 *  -1           discipline has no detach function
 *  -EINVAL      called with rtdev=NULL
 *  -ENODEV      no discipline active on dev
 *
 *  Note: must be called with rtdev->nrt_lock acquired
 */
int rtmac_disc_detach(struct rtnet_device *rtdev)
{
    int                 ret;
    struct rtmac_disc   *disc;
    struct rtmac_priv   *priv;


    RTNET_ASSERT(rtdev != NULL, return -EINVAL;);

    disc = rtdev->mac_disc;
    if (!disc)
	return -ENODEV;

    RTNET_ASSERT(disc->detach != NULL, return -EINVAL;);

    priv = rtdev->mac_priv;
    RTNET_ASSERT(priv != NULL, return -EINVAL;);

    ret = rtmac_vnic_unregister(rtdev);
    if (ret < 0)
	return ret;

    /* call release function of discipline */
    ret = disc->detach(rtdev, priv->disc_priv);
    if (ret < 0)
	return ret;

    rtmac_vnic_cleanup(rtdev);

    /* restore start_xmit and get_mtu */
    rtdev->start_xmit = priv->orig_start_xmit;
    rtdev->get_mtu    = rt_hard_mtu;

    /* remove pointers from rtdev */
    rtdev->mac_disc   = NULL;
    rtdev->mac_priv   = NULL;
    rtdev->mac_detach = NULL;

    rtdev_dereference(rtdev);

    kfree(priv);

    module_put(disc->owner);

    return 0;
}



static struct rtmac_disc *rtmac_get_disc_by_name(const char *name)
{
    struct list_head    *disc;


    mutex_lock(&disc_list_lock);

    list_for_each(disc, &disc_list) {
	if (strcmp(((struct rtmac_disc *)disc)->name, name) == 0) {
	    mutex_unlock(&disc_list_lock);
	    return (struct rtmac_disc *)disc;
	}
    }

    mutex_unlock(&disc_list_lock);

    return NULL;
}



int __rtmac_disc_register(struct rtmac_disc *disc, struct module *module)
{
    int ret;


    RTNET_ASSERT(disc != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->name != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->rt_packet_tx != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->nrt_packet_tx != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->attach != NULL, return -EINVAL;);
    RTNET_ASSERT(disc->detach != NULL, return -EINVAL;);

    disc->owner = module;

    if (rtmac_get_disc_by_name(disc->name) != NULL)
    {
	printk("RTmac: discipline '%s' already registered!\n", disc->name);
	return -EBUSY;
    }

    ret = rtnet_register_ioctls(&disc->ioctls);
    if (ret < 0)
	return ret;

#ifdef CONFIG_XENO_OPT_VFILE
    ret = rtmac_disc_proc_register(disc);
    if (ret < 0) {
	rtnet_unregister_ioctls(&disc->ioctls);
	return ret;
    }
#endif /* CONFIG_XENO_OPT_VFILE */

    mutex_lock(&disc_list_lock);

    list_add(&disc->list, &disc_list);

    mutex_unlock(&disc_list_lock);

    return 0;
}



void rtmac_disc_deregister(struct rtmac_disc *disc)
{
    RTNET_ASSERT(disc != NULL, return;);

    mutex_lock(&disc_list_lock);

    list_del(&disc->list);

    mutex_unlock(&disc_list_lock);

    rtnet_unregister_ioctls(&disc->ioctls);

#ifdef CONFIG_XENO_OPT_VFILE
    rtmac_disc_proc_unregister(disc);
#endif /* CONFIG_XENO_OPT_VFILE */
}



#ifdef CONFIG_XENO_OPT_VFILE
int rtnet_rtmac_disciplines_show(struct xnvfile_regular_iterator *it, void *d)
{
    struct rtmac_disc    *disc;
    int err;

    err = mutex_lock_interruptible(&disc_list_lock);
    if (err < 0)
	return err;

    xnvfile_printf(it, "Name\t\tID\n");

    list_for_each_entry(disc, &disc_list, list)
	xnvfile_printf(it, "%-15s %04X\n",disc->name, ntohs(disc->disc_type));

    mutex_unlock(&disc_list_lock);

    return 0;
}
#endif /* CONFIG_XENO_OPT_VFILE */
