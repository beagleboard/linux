/***
 *
 *  rtmac/nomac/nomac_module.c
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002       Marc Kleine-Budde <kleine-budde@gmx.de>,
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

#include <linux/init.h>
#include <linux/module.h>

#include <rtdm/driver.h>
#include <rtmac/rtmac_vnic.h>
#include <rtmac/nomac/nomac.h>
#include <rtmac/nomac/nomac_dev.h>
#include <rtmac/nomac/nomac_ioctl.h>
#include <rtmac/nomac/nomac_proto.h>

#ifdef CONFIG_XENO_OPT_VFILE
LIST_HEAD(nomac_devices);
DEFINE_MUTEX(nomac_nrt_lock);

int nomac_proc_read(struct xnvfile_regular_iterator *it, void *data)
{
	struct nomac_priv *entry;

	mutex_lock(&nomac_nrt_lock);

	xnvfile_printf(it, "Interface       API Device      State\n");

	list_for_each_entry (entry, &nomac_devices, list_entry)
		xnvfile_printf(it, "%-15s %-15s Attached\n", entry->rtdev->name,
			       entry->api_device.name);

	mutex_unlock(&nomac_nrt_lock);

	return 0;
}
#endif /* CONFIG_XENO_OPT_VFILE */

int nomac_attach(struct rtnet_device *rtdev, void *priv)
{
	struct nomac_priv *nomac = (struct nomac_priv *)priv;
	int ret;

	nomac->magic = NOMAC_MAGIC;
	nomac->rtdev = rtdev;

	/* ... */

	ret = nomac_dev_init(rtdev, nomac);
	if (ret < 0)
		return ret;

#ifdef CONFIG_XENO_OPT_VFILE
	mutex_lock(&nomac_nrt_lock);
	list_add(&nomac->list_entry, &nomac_devices);
	mutex_unlock(&nomac_nrt_lock);
#endif /* CONFIG_XENO_OPT_VFILE */

	return 0;
}

int nomac_detach(struct rtnet_device *rtdev, void *priv)
{
	struct nomac_priv *nomac = (struct nomac_priv *)priv;

	nomac_dev_release(nomac);

	/* ... */
#ifdef CONFIG_XENO_OPT_VFILE
	mutex_lock(&nomac_nrt_lock);
	list_del(&nomac->list_entry);
	mutex_unlock(&nomac_nrt_lock);
#endif /* CONFIG_XENO_OPT_VFILE */

	return 0;
}

#ifdef CONFIG_XENO_OPT_VFILE
struct rtmac_proc_entry nomac_proc_entries[] = {
	{ name: "nomac", handler: nomac_proc_read },
};
#endif /* CONFIG_XENO_OPT_VFILE */

struct rtmac_disc nomac_disc = {
	name: "NoMAC",
	priv_size: sizeof(struct nomac_priv),
	disc_type: __constant_htons(RTMAC_TYPE_NOMAC),

	packet_rx: nomac_packet_rx,
	rt_packet_tx: nomac_rt_packet_tx,
	nrt_packet_tx: nomac_nrt_packet_tx,

	get_mtu: NULL,

	vnic_xmit: RTMAC_DEFAULT_VNIC,

	attach: nomac_attach,
	detach: nomac_detach,

	ioctls: {
		service_name: "RTmac/NoMAC",
		ioctl_type: RTNET_IOC_TYPE_RTMAC_NOMAC,
		handler: nomac_ioctl
	},

#ifdef CONFIG_XENO_OPT_VFILE
	proc_entries: nomac_proc_entries,
	nr_proc_entries: ARRAY_SIZE(nomac_proc_entries),
#endif /* CONFIG_XENO_OPT_VFILE */
};

int __init nomac_init(void)
{
	int ret;

	printk("RTmac/NoMAC: init void media access control mechanism\n");

	ret = nomac_proto_init();
	if (ret < 0)
		return ret;

	ret = rtmac_disc_register(&nomac_disc);
	if (ret < 0) {
		nomac_proto_cleanup();
		return ret;
	}

	return 0;
}

void nomac_release(void)
{
	rtmac_disc_deregister(&nomac_disc);
	nomac_proto_cleanup();

	printk("RTmac/NoMAC: unloaded\n");
}

module_init(nomac_init);
module_exit(nomac_release);

MODULE_AUTHOR("Jan Kiszka");
MODULE_LICENSE("GPL");
