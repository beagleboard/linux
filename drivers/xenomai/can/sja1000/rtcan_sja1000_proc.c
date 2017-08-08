/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
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
#include <linux/ioport.h>
#include <linux/delay.h>

#include <rtdm/driver.h>

#include <rtcan_dev.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>

#ifdef CONFIG_XENO_DRIVERS_CAN_DEBUG

static int rtcan_sja_proc_regs(struct seq_file *p, void *data)
{
    struct rtcan_device *dev = (struct rtcan_device *)data;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;
    int i;

    seq_printf(p, "SJA1000 registers");
    for (i = 0; i < 0x20; i++) {
	if ((i % 0x10) == 0)
	    seq_printf(p, "\n%02x:", i);
	seq_printf(p, " %02x", chip->read_reg(dev, i));
    }
    seq_printf(p, "\n");
    return 0;
}

static int rtcan_sja_proc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_sja_proc_regs, PDE_DATA(inode));
}

static const struct file_operations rtcan_sja_proc_regs_ops = {
	.open		= rtcan_sja_proc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int rtcan_sja_create_proc(struct rtcan_device* dev)
{
    if (!dev->proc_root)
	return -EINVAL;

    proc_create_data("registers", S_IFREG | S_IRUGO | S_IWUSR, dev->proc_root,
		     &rtcan_sja_proc_regs_ops, dev);
    return 0;
}

void rtcan_sja_remove_proc(struct rtcan_device* dev)
{
    if (!dev->proc_root)
	return;

    remove_proc_entry("registers", dev->proc_root);
}

#else /* !CONFIG_XENO_DRIVERS_CAN_DEBUG */

void rtcan_sja_remove_proc(struct rtcan_device* dev)
{
}

int rtcan_sja_create_proc(struct rtcan_device* dev)
{
    return 0;
}
#endif	/* CONFIG_XENO_DRIVERS_CAN_DEBUG */
