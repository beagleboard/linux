/*
 * Copyright (C) 2006 Matthias Fuchs <matthias.fuchs@esd-electronics.com>,
 *                    Jan Kiszka <jan.kiszka@web.de>
 *
 * RTCAN driver for memory mapped SJA1000 CAN controller
 * This code has been tested on esd's CPCI405/EPPC405 PPC405 systems.
 *
 * This driver is derived from the rtcan-isa driver by
 * Wolfgang Grandegger and Sebastian Smolorz.
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; eitherer version 2 of the License, or
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

#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "sja1000-mem"

#define RTCAN_MEM_MAX_DEV 4

static char *mem_board_name = "mem mapped";

MODULE_AUTHOR("Matthias Fuchs <matthias.fuchs@esd-electronics.com>");
MODULE_DESCRIPTION("RTCAN driver for memory mapped SJA1000 controller");
MODULE_SUPPORTED_DEVICE("mem mapped");
MODULE_LICENSE("GPL");

static u32 mem[RTCAN_MEM_MAX_DEV];
static int irq[RTCAN_MEM_MAX_DEV];
static u32 can_clock[RTCAN_MEM_MAX_DEV];
static u8 ocr[RTCAN_MEM_MAX_DEV];
static u8 cdr[RTCAN_MEM_MAX_DEV];

module_param_array(mem, uint, NULL, 0444);
module_param_array(irq, int, NULL, 0444);
module_param_array(can_clock, uint, NULL, 0444);
module_param_array(ocr, byte, NULL, 0444);
module_param_array(cdr, byte, NULL, 0444);

MODULE_PARM_DESC(mem, "The io-memory address");
MODULE_PARM_DESC(irq, "The interrupt number");
MODULE_PARM_DESC(can_clock, "External clock frequency (default 16 MHz)");
MODULE_PARM_DESC(ocr, "Value of output control register (default 0x1a)");
MODULE_PARM_DESC(cdr, "Value of clock divider register (default 0xc8");

#define RTCAN_MEM_RANGE 0x80

struct rtcan_mem
{
	volatile void __iomem *vmem;
};

static struct rtcan_device *rtcan_mem_devs[RTCAN_MEM_MAX_DEV];

static u8 rtcan_mem_readreg(struct rtcan_device *dev, int reg)
{
	struct rtcan_mem *board = (struct rtcan_mem *)dev->board_priv;
	return readb(board->vmem + reg);
}

static void rtcan_mem_writereg(struct rtcan_device *dev, int reg, u8 val)
{
	struct rtcan_mem *board = (struct rtcan_mem *)dev->board_priv;
	writeb(val, board->vmem + reg);
}

int __init rtcan_mem_init_one(int idx)
{
	struct rtcan_device *dev;
	struct rtcan_sja1000 *chip;
	struct rtcan_mem *board;
	int ret;

	if ((dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
				   sizeof(struct rtcan_mem))) == NULL)
		return -ENOMEM;

	chip = (struct rtcan_sja1000 *)dev->priv;
	board = (struct rtcan_mem *)dev->board_priv;

	dev->board_name = mem_board_name;

	chip->irq_num = irq[idx];
	chip->irq_flags = RTDM_IRQTYPE_SHARED;
	chip->read_reg = rtcan_mem_readreg;
	chip->write_reg = rtcan_mem_writereg;

	if (!request_mem_region(mem[idx], RTCAN_MEM_RANGE, RTCAN_DRV_NAME)) {
		ret = -EBUSY;
		goto out_dev_free;
	}

	/* ioremap io memory */
	if (!(board->vmem = ioremap(mem[idx], RTCAN_MEM_RANGE))) {
		ret = -EBUSY;
		goto out_release_mem;
	}

	/* Clock frequency in Hz */
	if (can_clock[idx])
		dev->can_sys_clock = can_clock[idx] / 2;
	else
		dev->can_sys_clock = 8000000; /* 16/2 MHz */

	/* Output control register */
	if (ocr[idx])
		chip->ocr = ocr[idx];
	else
		chip->ocr = SJA_OCR_MODE_NORMAL | SJA_OCR_TX0_PUSHPULL;

	if (cdr[idx])
		chip->cdr = cdr[idx];
	else
		chip->cdr = SJA_CDR_CAN_MODE | SJA_CDR_CLK_OFF | SJA_CDR_CBP;

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	ret = rtcan_sja1000_register(dev);
	if (ret) {
		printk(KERN_ERR "ERROR %d while trying to register SJA1000 "
		       "device!\n", ret);
		goto out_iounmap;
	}

	rtcan_mem_devs[idx] = dev;
	return 0;

 out_iounmap:
	iounmap((void *)board->vmem);

 out_release_mem:
	release_mem_region(mem[idx], RTCAN_MEM_RANGE);

 out_dev_free:
	rtcan_dev_free(dev);

	return ret;
}

static void rtcan_mem_exit(void);

/** Init module */
static int __init rtcan_mem_init(void)
{
	int i, err;
	int devices = 0;

	if (!rtdm_available())
		return -ENOSYS;

	for (i = 0; i < RTCAN_MEM_MAX_DEV && mem[i] != 0; i++) {
		err = rtcan_mem_init_one(i);
		if (err) {
			rtcan_mem_exit();
			return err;
		}
		devices++;
	}
	if (devices)
		return 0;

	printk(KERN_ERR "ERROR! No devices specified! "
	       "Use mem=<port1>[,...] irq=<irq1>[,...]\n");
	return -EINVAL;
}


/** Cleanup module */
static void rtcan_mem_exit(void)
{
	int i;
	struct rtcan_device *dev;
	volatile void __iomem *vmem;

	for (i = 0; i < RTCAN_MEM_MAX_DEV; i++) {
		dev = rtcan_mem_devs[i];
		if (!dev)
			continue;
		vmem = ((struct rtcan_mem *)dev->board_priv)->vmem;
		rtcan_sja1000_unregister(dev);
		iounmap((void *)vmem);
		release_mem_region(mem[i], RTCAN_MEM_RANGE);
		rtcan_dev_free(dev);
	}
}

module_init(rtcan_mem_init);
module_exit(rtcan_mem_exit);
