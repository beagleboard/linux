/*
 * linux/arch/arm/mach-omap/dsp/taskwatch.c
 *
 * OMAP DSP task watch device driver
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 200%/05/16:  DSP Gateway version 3.3
 */

#include <linux/module.h>
#include <linux/major.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/dsp.h>
#include "dsp.h"

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
static ssize_t dsp_twch_read(struct file *file, char *buf, size_t count,
			     loff_t *ppos)
{
	long taskstat[TASKDEV_MAX];
	int devcount = count / sizeof(long);
	int i;

	if (!dsp_is_ready()) {
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
		if (dsp_is_ready())
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
	static DECLARE_MUTEX(ioctl_sem);
	int ret;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	switch (cmd) {
	case OMAP_DSP_TWCH_IOCTL_MKDEV:
		{
			char name[OMAP_DSP_TNM_LEN];
			if (copy_from_user(name, (void *)arg, OMAP_DSP_TNM_LEN)) {
				ret = -EFAULT;
				goto up_out;
			}
			name[OMAP_DSP_TNM_LEN-1] = '\0';
			ret = dsp_mkdev(name);
			break;
		}

	case OMAP_DSP_TWCH_IOCTL_RMDEV:
		{
			char name[OMAP_DSP_TNM_LEN];
			if (copy_from_user(name, (void *)arg, OMAP_DSP_TNM_LEN)) {
				ret = -EFAULT;
				goto up_out;
			}
			name[OMAP_DSP_TNM_LEN-1] = '\0';
			ret = dsp_rmdev(name);
			break;
		}

	case OMAP_DSP_TWCH_IOCTL_TADD:
		{
			struct omap_dsp_taddinfo ti;
			if (copy_from_user(&ti, (void *)arg, sizeof(ti))) {
				ret = -EFAULT;
				goto up_out;
			}
			ret = dsp_tadd(ti.minor, ti.taskadr);
			break;
		}

	case OMAP_DSP_TWCH_IOCTL_TDEL:
		ret = dsp_tdel(arg);
		break;

	case OMAP_DSP_TWCH_IOCTL_TKILL:
		ret = dsp_tkill(arg);
		break;

	default:
		ret = -ENOIOCTLCMD;
	}

up_out:
	up(&ioctl_sem);
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
