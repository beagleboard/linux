/***
 *
 *  rtcfg/rtcfg_timer.c
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2003-2005 Jan Kiszka <jan.kiszka@web.de>
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

#include <linux/kernel.h>
#include <linux/list.h>

#include <rtdev.h>
#include <rtcfg/rtcfg.h>
#include <rtcfg/rtcfg_conn_event.h>
#include <rtcfg/rtcfg_event.h>
#include <rtcfg/rtcfg_frame.h>
#include <rtcfg/rtcfg_timer.h>

void rtcfg_timer(rtdm_timer_t *t)
{
	struct rtcfg_device *rtcfg_dev =
		container_of(t, struct rtcfg_device, timer);

	set_bit(FLAG_TIMER_PENDING, &rtcfg_dev->flags);
	rtcfg_thread_signal();
}

void rtcfg_timer_run_one(int ifindex)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct list_head *entry;
	struct rtcfg_connection *conn;
	int last_stage_1 = -1;
	int burst_credit;
	int index;
	int ret, shutdown;

	shutdown = test_and_clear_bit(FLAG_TIMER_SHUTDOWN, &rtcfg_dev->flags);

	if (!test_and_clear_bit(FLAG_TIMER_PENDING, &rtcfg_dev->flags) ||
	    shutdown)
		return;

	rtdm_mutex_lock(&rtcfg_dev->dev_mutex);

	if (rtcfg_dev->state == RTCFG_MAIN_SERVER_RUNNING) {
		index = 0;
		burst_credit = rtcfg_dev->burstrate;

		list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
			conn = list_entry(entry, struct rtcfg_connection,
					  entry);

			if ((conn->state == RTCFG_CONN_SEARCHING) ||
			    (conn->state == RTCFG_CONN_DEAD)) {
				if ((burst_credit > 0) &&
				    (index > last_stage_1)) {
					if ((ret = rtcfg_send_stage_1(conn)) <
					    0) {
						RTCFG_DEBUG(
							2,
							"RTcfg: error %d while sending "
							"stage 1 frame\n",
							ret);
					}
					burst_credit--;
					last_stage_1 = index;
				}
			} else {
				/* skip connection in history */
				if (last_stage_1 == (index - 1))
					last_stage_1 = index;

				rtcfg_do_conn_event(conn, RTCFG_TIMER, NULL);
			}
			index++;
		}

		/* handle pointer overrun of the last stage 1 transmission */
		if (last_stage_1 == (index - 1))
			last_stage_1 = -1;
	} else if (rtcfg_dev->state == RTCFG_MAIN_CLIENT_READY)
		rtcfg_send_heartbeat(ifindex);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
}

void rtcfg_timer_run(void)
{
	int ifindex;

	for (ifindex = 0; ifindex < MAX_RT_DEVICES; ifindex++)
		rtcfg_timer_run_one(ifindex);
}
