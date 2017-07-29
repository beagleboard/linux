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

/* CAN device profile */
#include "rtcan_dev.h"
#include "rtcan_internal.h"
#include "rtcan_mscan_regs.h"

#define MSCAN_REG_ARGS(reg) \
	"%-8s 0x%02x\n", #reg, (int)(in_8(&regs->reg)) & 0xff

#ifdef CONFIG_XENO_DRIVERS_CAN_DEBUG

static int rtcan_mscan_proc_regs(struct seq_file *p, void *data)
{
	struct rtcan_device *dev = (struct rtcan_device *)data;
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;
#ifdef MPC5xxx_GPIO
	struct mpc5xxx_gpio *gpio = (struct mpc5xxx_gpio *)MPC5xxx_GPIO;
	u32 port_config;
#endif
	u8 canctl0, canctl1;

	seq_printf(p, "MSCAN registers at %p\n", regs);

	canctl0 = in_8(&regs->canctl0);
	seq_printf(p, "canctl0  0x%02x%s%s%s%s%s%s%s%s\n",
		   canctl0,
		   (canctl0 & MSCAN_RXFRM) ? " rxfrm" :"",
		   (canctl0 & MSCAN_RXACT) ? " rxact" :"",
		   (canctl0 & MSCAN_CSWAI) ? " cswai" :"",
		   (canctl0 & MSCAN_SYNCH) ? " synch" :"",
		   (canctl0 & MSCAN_TIME)  ? " time"  :"",
		   (canctl0 & MSCAN_WUPE)  ? " wupe"  :"",
		   (canctl0 & MSCAN_SLPRQ) ? " slprq" :"",
		   (canctl0 & MSCAN_INITRQ)? " initrq":"" );
	canctl1 = in_8(&regs->canctl1);
	seq_printf(p, "canctl1  0x%02x%s%s%s%s%s%s%s\n",
		   canctl1,
		   (canctl1 & MSCAN_CANE)  ? " cane"  :"",
		   (canctl1 & MSCAN_CLKSRC)? " clksrc":"",
		   (canctl1 & MSCAN_LOOPB) ? " loopb" :"",
		   (canctl1 & MSCAN_LISTEN)? " listen":"",
		   (canctl1 & MSCAN_WUPM)  ? " wump"  :"",
		   (canctl1 & MSCAN_SLPAK) ? " slpak" :"",
		   (canctl1 & MSCAN_INITAK)? " initak":"");
	seq_printf(p, MSCAN_REG_ARGS(canbtr0 ));
	seq_printf(p, MSCAN_REG_ARGS(canbtr1 ));
	seq_printf(p, MSCAN_REG_ARGS(canrflg ));
	seq_printf(p, MSCAN_REG_ARGS(canrier ));
	seq_printf(p, MSCAN_REG_ARGS(cantflg ));
	seq_printf(p, MSCAN_REG_ARGS(cantier ));
	seq_printf(p, MSCAN_REG_ARGS(cantarq ));
	seq_printf(p, MSCAN_REG_ARGS(cantaak ));
	seq_printf(p, MSCAN_REG_ARGS(cantbsel));
	seq_printf(p, MSCAN_REG_ARGS(canidac ));
	seq_printf(p, MSCAN_REG_ARGS(canrxerr));
	seq_printf(p, MSCAN_REG_ARGS(cantxerr));
	seq_printf(p, MSCAN_REG_ARGS(canidar0));
	seq_printf(p, MSCAN_REG_ARGS(canidar1));
	seq_printf(p, MSCAN_REG_ARGS(canidar2));
	seq_printf(p, MSCAN_REG_ARGS(canidar3));
	seq_printf(p, MSCAN_REG_ARGS(canidmr0));
	seq_printf(p, MSCAN_REG_ARGS(canidmr1));
	seq_printf(p, MSCAN_REG_ARGS(canidmr2));
	seq_printf(p, MSCAN_REG_ARGS(canidmr3));
	seq_printf(p, MSCAN_REG_ARGS(canidar4));
	seq_printf(p, MSCAN_REG_ARGS(canidar5));
	seq_printf(p, MSCAN_REG_ARGS(canidar6));
	seq_printf(p, MSCAN_REG_ARGS(canidar7));
	seq_printf(p, MSCAN_REG_ARGS(canidmr4));
	seq_printf(p, MSCAN_REG_ARGS(canidmr5));
	seq_printf(p, MSCAN_REG_ARGS(canidmr6));
	seq_printf(p, MSCAN_REG_ARGS(canidmr7));

#ifdef MPC5xxx_GPIO
	seq_printf(p, "GPIO registers\n");
	port_config = in_be32(&gpio->port_config);
	seq_printf(p, "port_config 0x%08x %s\n", port_config,
		   (port_config & 0x10000000 ?
			"CAN1 on I2C1, CAN2 on TMR0/1 pins" :
			(port_config & 0x70) == 0x10 ?
				"CAN1/2 on PSC2 pins" :
				"MSCAN1/2 not routed"));
#endif

	return 0;
}

static int rtcan_mscan_proc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtcan_mscan_proc_regs, PDE_DATA(inode));
}

static const struct file_operations rtcan_mscan_proc_regs_ops = {
	.open		= rtcan_mscan_proc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int rtcan_mscan_create_proc(struct rtcan_device* dev)
{
	if (!dev->proc_root)
		return -EINVAL;

	proc_create_data("registers", S_IFREG | S_IRUGO | S_IWUSR,
			 dev->proc_root, &rtcan_mscan_proc_regs_ops, dev);
	return 0;
}

void rtcan_mscan_remove_proc(struct rtcan_device* dev)
{
	if (!dev->proc_root)
		return;

	remove_proc_entry("registers", dev->proc_root);
}

#else /* !CONFIG_XENO_DRIVERS_CAN_DEBUG */

void rtcan_mscan_remove_proc(struct rtcan_device* dev)
{
}

int rtcan_mscan_create_proc(struct rtcan_device* dev)
{
	return 0;
}
#endif	/* CONFIG_XENO_DRIVERS_CAN_DEBUG */
