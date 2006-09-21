/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
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
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ioctl.h"

static DECLARE_WAIT_QUEUE_HEAD(read_wait_q);
static unsigned int change_cnt;

void dsp_twch_touch(void)
{
	change_cnt++;
	wake_up_interruptible(&read_wait_q);
}

/*
 * @count: represents the device counts of the user's interst
 */
static ssize_t dsp_twch_read(struct file *file, char __user *buf, size_t count,
			     loff_t *ppos)
{
	long taskstat[TASKDEV_MAX];
	int devcount = count / sizeof(long);
	int i;

	if (dsp_cfgstat_get_stat() != CFGSTAT_READY) {
		printk(KERN_ERR "omapdsp: dsp has not been configured.\n");
		return -EINVAL;
	}

	if (change_cnt == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&read_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (change_cnt == 0)	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&read_wait_q, &wait);

		/* unconfigured while waiting ;-( */
		if (dsp_cfgstat_get_stat() != CFGSTAT_READY)
			return -EINVAL;
	}

	if (devcount > TASKDEV_MAX)
		devcount = TASKDEV_MAX;

	count = devcount * sizeof(long);
	change_cnt = 0;
	for (i = 0; i < devcount; i++) {
		/*
		 * once the device state is read, the 'STALE' bit will be set
		 * so that the Dynamic Loader can distinguish the new request
		 * from the old one.
		 */
		taskstat[i] = taskdev_state_stale(i);
	}

	if (copy_to_user(buf, taskstat, count))
		return -EFAULT;

	return count;
}

static unsigned int dsp_twch_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &read_wait_q, wait);
	if (change_cnt)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int dsp_twch_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case TWCH_IOCTL_MKDEV:
		{
			char name[TNM_LEN];
			if (copy_from_user(name, (void __user *)arg, TNM_LEN))
				return -EFAULT;
			name[TNM_LEN-1] = '\0';
			ret = dsp_mkdev(name);
			break;
		}

	case TWCH_IOCTL_RMDEV:
		{
			char name[TNM_LEN];
			if (copy_from_user(name, (void __user *)arg, TNM_LEN))
				return -EFAULT;
			name[TNM_LEN-1] = '\0';
			ret = dsp_rmdev(name);
			break;
		}

	case TWCH_IOCTL_TADD:
		{
			struct omap_dsp_taddinfo ti;
			if (copy_from_user(&ti, (void __user *)arg, sizeof(ti)))
				return -EFAULT;
			ret = dsp_tadd_minor(ti.minor, ti.taskadr);
			break;
		}

	case TWCH_IOCTL_TDEL:
		ret = dsp_tdel_minor(arg);
		break;

	case TWCH_IOCTL_TKILL:
		ret = dsp_tkill_minor(arg);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

struct file_operations dsp_twch_fops = {
	.owner = THIS_MODULE,
	.read  = dsp_twch_read,
	.poll  = dsp_twch_poll,
	.ioctl = dsp_twch_ioctl,
};

void dsp_twch_start(void)
{
	change_cnt = 1;		/* first read will not wait */
}

void dsp_twch_stop(void)
{
	wake_up_interruptible(&read_wait_q);
}
