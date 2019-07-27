/***
 *
 *	rtcfg/rtcfg_proc.c
 *
 *	Real-Time Configuration Distribution Protocol
 *
 *	Copyright (C) 2004 Jan Kiszka <jan.kiszka@web.de>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <rtdev.h>
#include <rtnet_internal.h>
#include <rtnet_port.h>
#include <rtcfg/rtcfg_conn_event.h>
#include <rtcfg/rtcfg_event.h>
#include <rtcfg/rtcfg_frame.h>


#ifdef CONFIG_XENO_OPT_VFILE
DEFINE_MUTEX(nrt_proc_lock);
static struct xnvfile_directory rtcfg_proc_root;

static int rtnet_rtcfg_proc_lock_get(struct xnvfile *vfile)
{
	return mutex_lock_interruptible(&nrt_proc_lock);
}

static void rtnet_rtcfg_proc_lock_put(struct xnvfile *vfile)
{
	return mutex_unlock(&nrt_proc_lock);
}

static struct xnvfile_lock_ops rtnet_rtcfg_proc_lock_ops = {
	.get = rtnet_rtcfg_proc_lock_get,
	.put = rtnet_rtcfg_proc_lock_put,
};

int rtnet_rtcfg_dev_state_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct rtcfg_device *rtcfg_dev = xnvfile_priv(it->vfile);
	const char *state_name[] = {
		"OFF", "SERVER_RUNNING", "CLIENT_0", "CLIENT_1",
		"CLIENT_ANNOUNCED", "CLIENT_ALL_KNOWN",
		"CLIENT_ALL_FRAMES", "CLIENT_2", "CLIENT_READY"
	};

	xnvfile_printf(it, "state:\t\t\t%d (%s)\n"
				"flags:\t\t\t%08lX\n"
				"other stations:\t\t%d\n"
				"stations found:\t\t%d\n"
				"stations ready:\t\t%d\n",
				rtcfg_dev->state, state_name[rtcfg_dev->state],
				rtcfg_dev->flags, rtcfg_dev->other_stations,
				rtcfg_dev->stations_found,
				rtcfg_dev->stations_ready);

	if (rtcfg_dev->state == RTCFG_MAIN_SERVER_RUNNING) {
		xnvfile_printf(it, "configured clients:\t%d\n"
					"burstrate:\t\t%d\n"
					"heartbeat period:\t%d ms\n",
					rtcfg_dev->spec.srv.clients_configured,
					rtcfg_dev->burstrate, rtcfg_dev->spec.srv.heartbeat);
	} else if (rtcfg_dev->state != RTCFG_MAIN_OFF) {
		xnvfile_printf(it, "address type:\t\t%d\n"
					"server address:\t\t%02X:%02X:%02X:%02X:%02X:%02X\n"
					"stage 2 config:\t\t%d/%d\n",
					rtcfg_dev->spec.clt.addr_type,
					rtcfg_dev->spec.clt.srv_mac_addr[0],
					rtcfg_dev->spec.clt.srv_mac_addr[1],
					rtcfg_dev->spec.clt.srv_mac_addr[2],
					rtcfg_dev->spec.clt.srv_mac_addr[3],
					rtcfg_dev->spec.clt.srv_mac_addr[4],
					rtcfg_dev->spec.clt.srv_mac_addr[5],
					rtcfg_dev->spec.clt.cfg_offs,
					rtcfg_dev->spec.clt.cfg_len);
	}

	return 0;
}

static struct xnvfile_regular_ops rtnet_rtcfg_dev_state_vfile_ops = {
	.show = rtnet_rtcfg_dev_state_show,
};

int rtnet_rtcfg_dev_stations_show(struct xnvfile_regular_iterator *it, void *d)
{
	struct rtcfg_device *rtcfg_dev = xnvfile_priv(it->vfile);
	struct rtcfg_connection *conn;
	struct rtcfg_station *station;
	int i;

	if (rtcfg_dev->state == RTCFG_MAIN_SERVER_RUNNING) {
		list_for_each_entry(conn, &rtcfg_dev->spec.srv.conn_list, entry) {
			if ((conn->state != RTCFG_CONN_SEARCHING) &&
				(conn->state != RTCFG_CONN_DEAD))
				xnvfile_printf(it, "%02X:%02X:%02X:%02X:%02X:%02X\t%02X\n",
							conn->mac_addr[0], conn->mac_addr[1],
							conn->mac_addr[2], conn->mac_addr[3],
							conn->mac_addr[4], conn->mac_addr[5],
							conn->flags);
		}
	} else if (rtcfg_dev->spec.clt.station_addr_list) {
		for (i = 0; i < rtcfg_dev->stations_found; i++) {
			station = &rtcfg_dev->spec.clt.station_addr_list[i];

			xnvfile_printf(it, "%02X:%02X:%02X:%02X:%02X:%02X\t%02X\n",
						station->mac_addr[0], station->mac_addr[1],
						station->mac_addr[2], station->mac_addr[3],
						station->mac_addr[4], station->mac_addr[5],
						station->flags);
		}
	}

	return 0;
}

static struct xnvfile_regular_ops rtnet_rtcfg_dev_stations_vfile_ops = {
	.show = rtnet_rtcfg_dev_stations_show,
};

int
rtnet_rtcfg_dev_conn_state_show(struct xnvfile_regular_iterator *it, void *d)
{
	struct rtcfg_connection *conn = xnvfile_priv(it->vfile);
	char *state_name[] =
		{ "SEARCHING", "STAGE_1", "STAGE_2", "READY", "DEAD" };

	xnvfile_printf(it, "state:\t\t\t%d (%s)\n"
				"flags:\t\t\t%02X\n"
				"stage 1 size:\t\t%zd\n"
				"stage 2 filename:\t%s\n"
				"stage 2 size:\t\t%zd\n"
				"stage 2 offset:\t\t%d\n"
				"burstrate:\t\t%d\n"
				"mac address:\t\t%02X:%02X:%02X:%02X:%02X:%02X\n",
				conn->state, state_name[conn->state], conn->flags,
				conn->stage1_size,
				(conn->stage2_file)? conn->stage2_file->name: "-",
				(conn->stage2_file)? conn->stage2_file->size: 0,
				conn->cfg_offs, conn->burstrate,
				conn->mac_addr[0], conn->mac_addr[1],
				conn->mac_addr[2], conn->mac_addr[3],
				conn->mac_addr[4], conn->mac_addr[5]);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	if ((conn->addr_type & RTCFG_ADDR_MASK) == RTCFG_ADDR_IP)
		xnvfile_printf(it, "ip:\t\t\t%u.%u.%u.%u\n",
					NIPQUAD(conn->addr.ip_addr));
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

	return 0;
}

static struct xnvfile_regular_ops rtnet_rtcfg_dev_conn_state_vfile_ops = {
	.show = rtnet_rtcfg_dev_conn_state_show,
};

void rtcfg_update_conn_proc_entries(int ifindex)
{
	struct rtcfg_device		*dev = &device[ifindex];
	struct rtcfg_connection *conn;
	char					name_buf[64];

	if (dev->state != RTCFG_MAIN_SERVER_RUNNING)
		return;

	list_for_each_entry(conn, &dev->spec.srv.conn_list, entry) {
		switch (conn->addr_type & RTCFG_ADDR_MASK) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
		case RTCFG_ADDR_IP:
			snprintf(name_buf, 64, "CLIENT_%u.%u.%u.%u",
					NIPQUAD(conn->addr.ip_addr));
			break;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

		default: /* RTCFG_ADDR_MAC */
			snprintf(name_buf, 64,
					"CLIENT_%02X%02X%02X%02X%02X%02X",
					conn->mac_addr[0], conn->mac_addr[1],
					conn->mac_addr[2], conn->mac_addr[3],
					conn->mac_addr[4], conn->mac_addr[5]);
			break;
		}
		memset(&conn->proc_entry, '\0', sizeof(conn->proc_entry));
		conn->proc_entry.entry.lockops = &rtnet_rtcfg_proc_lock_ops;
		conn->proc_entry.ops = &rtnet_rtcfg_dev_conn_state_vfile_ops;
		xnvfile_priv(&conn->proc_entry) = conn;

		xnvfile_init_regular(name_buf, &conn->proc_entry, &dev->proc_entry);
	}
}



void rtcfg_remove_conn_proc_entries(int ifindex)
{
	struct rtcfg_device		*dev = &device[ifindex];
	struct rtcfg_connection *conn;


	if (dev->state != RTCFG_MAIN_SERVER_RUNNING)
		return;

	list_for_each_entry(conn, &dev->spec.srv.conn_list, entry)
		xnvfile_destroy_regular(&conn->proc_entry);
}



void rtcfg_new_rtdev(struct rtnet_device *rtdev)
{
	struct rtcfg_device *dev = &device[rtdev->ifindex];
	int err;


	mutex_lock(&nrt_proc_lock);

	memset(&dev->proc_entry, '\0', sizeof(dev->proc_entry));
	err = xnvfile_init_dir(rtdev->name, &dev->proc_entry, &rtcfg_proc_root);
	if (err < 0)
		goto error1;

	memset(&dev->proc_state_vfile, '\0', sizeof(dev->proc_state_vfile));
	dev->proc_state_vfile.entry.lockops = &rtnet_rtcfg_proc_lock_ops;
	dev->proc_state_vfile.ops = &rtnet_rtcfg_dev_state_vfile_ops;
	xnvfile_priv(&dev->proc_state_vfile) = dev;

	err = xnvfile_init_regular("state",
							&dev->proc_state_vfile, &dev->proc_entry);
	if (err < 0)
		goto error2;

	memset(&dev->proc_stations_vfile, '\0', sizeof(dev->proc_stations_vfile));
	dev->proc_stations_vfile.entry.lockops = &rtnet_rtcfg_proc_lock_ops;
	dev->proc_stations_vfile.ops = &rtnet_rtcfg_dev_stations_vfile_ops;
	xnvfile_priv(&dev->proc_stations_vfile) = dev;

	err = xnvfile_init_regular("stations_list",
							&dev->proc_stations_vfile, &dev->proc_entry);
	if (err < 0)
		goto error3;

	mutex_unlock(&nrt_proc_lock);

	return;

  error3:
	xnvfile_destroy_regular(&dev->proc_state_vfile);
  error2:
	xnvfile_destroy_dir(&dev->proc_entry);
  error1:
	dev->proc_entry.entry.pde = NULL;
	mutex_unlock(&nrt_proc_lock);
}



void rtcfg_remove_rtdev(struct rtnet_device *rtdev)
{
	struct rtcfg_device *dev = &device[rtdev->ifindex];


	// To-Do: issue down command

	mutex_lock(&nrt_proc_lock);

	if (dev->proc_entry.entry.pde) {
		rtcfg_remove_conn_proc_entries(rtdev->ifindex);

		xnvfile_destroy_regular(&dev->proc_stations_vfile);
		xnvfile_destroy_regular(&dev->proc_state_vfile);
		xnvfile_destroy_dir(&dev->proc_entry);
		dev->proc_entry.entry.pde = NULL;
	}

	mutex_unlock(&nrt_proc_lock);
}



static struct rtdev_event_hook rtdev_hook = {
	.register_device =	rtcfg_new_rtdev,
	.unregister_device =rtcfg_remove_rtdev,
	.ifup =				NULL,
	.ifdown =			NULL
};



int rtcfg_init_proc(void)
{
	struct rtnet_device *rtdev;
	int					i, err;

	err = xnvfile_init_dir("rtcfg", &rtcfg_proc_root, &rtnet_proc_root);
	if (err < 0)
		goto err1;

	for (i = 0; i < MAX_RT_DEVICES; i++) {
		rtdev = rtdev_get_by_index(i);
		if (rtdev) {
			rtcfg_new_rtdev(rtdev);
			rtdev_dereference(rtdev);
		}
	}

	rtdev_add_event_hook(&rtdev_hook);
	return 0;

  err1:
	printk("RTcfg: unable to initialise /proc entries\n");
	return err;
}



void rtcfg_cleanup_proc(void)
{
	struct rtnet_device *rtdev;
	int					i;


	rtdev_del_event_hook(&rtdev_hook);

	for (i = 0; i < MAX_RT_DEVICES; i++) {
		rtdev = rtdev_get_by_index(i);
		if (rtdev) {
			rtcfg_remove_rtdev(rtdev);
			rtdev_dereference(rtdev);
		}
	}

	xnvfile_destroy_dir(&rtcfg_proc_root);
}

#endif /* CONFIG_XENO_OPT_VFILE */
