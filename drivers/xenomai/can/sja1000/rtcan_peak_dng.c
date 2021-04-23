/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from the PCAN project file driver/src/pcan_dongle.c:
 *
 * Copyright (C) 2001-2006  PEAK System-Technik GmbH
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
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/pnp.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "PEAK-Dongle"

#define RTCAN_PEAK_DNG_MAX_DEV 1

static char *dongle_board_name = "PEAK-Dongle";

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("RTCAN board driver for PEAK-Dongle");
MODULE_SUPPORTED_DEVICE("PEAK-Dongle CAN controller");
MODULE_LICENSE("GPL");

static char   *type[RTCAN_PEAK_DNG_MAX_DEV];
static ushort io[RTCAN_PEAK_DNG_MAX_DEV];
static char   irq[RTCAN_PEAK_DNG_MAX_DEV];

module_param_array(type, charp,  NULL, 0444);
module_param_array(io,   ushort, NULL, 0444);
module_param_array(irq,  byte,   NULL, 0444);

MODULE_PARM_DESC(type, "The type of interface (sp, epp)");
MODULE_PARM_DESC(io,   "The io-port address");
MODULE_PARM_DESC(irq,  "The interrupt number");

#define DONGLE_TYPE_SP  0
#define DONGLE_TYPE_EPP 1

#define DNG_PORT_SIZE            4  /* the address range of the dongle-port */
#define ECR_PORT_SIZE            1  /* size of the associated ECR register */

struct rtcan_peak_dng
{
    u16  ioport;
    u16  ecr;      /* ECR register in case of EPP */
    u8   old_data; /* the overwritten contents of the port registers */
    u8   old_ctrl;
    u8   old_ecr;
    u8   type;
};

static struct rtcan_device *rtcan_peak_dng_devs[RTCAN_PEAK_DNG_MAX_DEV];

static u16 dng_ports[] = {0x378, 0x278, 0x3bc, 0x2bc};
static u8  dng_irqs[]  = {7, 5, 7, 5};

static unsigned char nibble_decode[32] =
{
    0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
    0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7
};

/* Enable and disable irqs */
static inline void rtcan_parport_disable_irq(u32 port)
{
    u32 pc = port + 2;
    outb(inb(pc) & ~0x10, pc);
}

static inline void rtcan_parport_enable_irq(u32 port)
{
    u32 pc = port + 2;
    outb(inb(pc) | 0x10, pc);
}

/* Functions for SP port */
static u8 rtcan_peak_dng_sp_readreg(struct rtcan_device *dev, int port)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;
    u32 pa = dng->ioport;
    u32 pb = pa + 1;
    u32 pc = pb + 1;
    u8  b0, b1 ;
    u8  irq_enable = inb(pc) & 0x10; /* don't influence irq_enable */

    outb((0x0B ^ 0x0D) | irq_enable, pc);
    outb((port & 0x1F) | 0x80, pa);
    outb((0x0B ^ 0x0C) | irq_enable, pc);
    b1=nibble_decode[inb(pb)>>3];
    outb(0x40, pa);
    b0=nibble_decode[inb(pb)>>3];
    outb((0x0B ^ 0x0D) | irq_enable, pc);

    return  (b1 << 4) | b0 ;
}

static void rtcan_peak_dng_writereg(struct rtcan_device *dev, int port, u8 data)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;
    u32 pa = dng->ioport;
    u32 pc = pa + 2;
    u8  irq_enable = inb(pc) & 0x10; /* don't influence irq_enable */

    outb((0x0B ^ 0x0D) | irq_enable, pc);
    outb(port & 0x1F, pa);
    outb((0x0B ^ 0x0C) | irq_enable, pc);
    outb(data, pa);
    outb((0x0B ^ 0x0D) | irq_enable, pc);
}

/* Functions for EPP port */
static u8 rtcan_peak_dng_epp_readreg(struct rtcan_device *dev, int port)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;
    u32 pa = dng->ioport;
    u32 pc = pa + 2;
    u8  val;
    u8  irq_enable = inb(pc) & 0x10; /* don't influence irq_enable */

    outb((0x0B ^ 0x0F) | irq_enable, pc);
    outb((port & 0x1F) | 0x80, pa);
    outb((0x0B ^ 0x2E) | irq_enable, pc);
    val = inb(pa);
    outb((0x0B ^ 0x0F) | irq_enable, pc);

    return val;
}


/* to switch epp on or restore register */
static void dongle_set_ecr(u16 port, struct rtcan_peak_dng *dng)
{
    u32 ecr = dng->ecr;

    dng->old_ecr = inb(ecr);
    outb((dng->old_ecr & 0x1F) | 0x20, ecr);

    if (dng->old_ecr == 0xff)
	printk(KERN_DEBUG "%s: realy ECP mode configured?\n", RTCAN_DRV_NAME);
}

static void dongle_restore_ecr(u16 port, struct rtcan_peak_dng *dng)
{
    u32 ecr = dng->ecr;

    outb(dng->old_ecr, ecr);

    printk(KERN_DEBUG "%s: restore ECR\n", RTCAN_DRV_NAME);
}

static inline void rtcan_peak_dng_enable(struct rtcan_device *dev)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;
    u32 port = dng->ioport;

    /* save old port contents */
    dng->old_data = inb(port);
    dng->old_ctrl = inb(port + 2);

    /* switch to epp mode if possible */
    if (dng->type == DONGLE_TYPE_EPP)
	dongle_set_ecr(port, dng);

    rtcan_parport_enable_irq(port);
}

static inline void rtcan_peak_dng_disable(struct rtcan_device *dev)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;
    u32 port = dng->ioport;

    rtcan_parport_disable_irq(port);

    if (dng->type == DONGLE_TYPE_EPP)
	dongle_restore_ecr(port, dng);

    /* restore port state */
    outb(dng->old_data, port);
    outb(dng->old_ctrl, port + 2);
}

/** Init module */
int __init rtcan_peak_dng_init_one(int idx)
{
    int ret, dtype;
    struct rtcan_device *dev;
    struct rtcan_sja1000 *sja;
    struct rtcan_peak_dng *dng;

    if (strncmp(type[idx], "sp", 2) == 0)
	dtype = DONGLE_TYPE_SP;
    else if (strncmp(type[idx], "epp", 3) == 0)
	dtype = DONGLE_TYPE_EPP;
    else {
	printk("%s: type %s is invalid, use \"sp\" or \"epp\".",
	       RTCAN_DRV_NAME, type[idx]);
	return -EINVAL;
    }

    if ((dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
			       sizeof(struct rtcan_peak_dng))) == NULL)
	return -ENOMEM;

    sja = (struct rtcan_sja1000 *)dev->priv;
    dng = (struct rtcan_peak_dng *)dev->board_priv;

    dev->board_name = dongle_board_name;

    if (io[idx])
	dng->ioport = io[idx];
    else
	dng->ioport = dng_ports[idx];

    if (irq[idx])
	sja->irq_num = irq[idx];
    else
	sja->irq_num = dng_irqs[idx];
    sja->irq_flags = 0;

    if (dtype == DONGLE_TYPE_SP) {
	sja->read_reg = rtcan_peak_dng_sp_readreg;
	sja->write_reg = rtcan_peak_dng_writereg;
	dng->ecr = 0; /* set to anything */
    } else {
	sja->read_reg = rtcan_peak_dng_epp_readreg;
	sja->write_reg = rtcan_peak_dng_writereg;
	dng->ecr = dng->ioport + 0x402;
    }

    /* Check and request I/O ports */
    if (!request_region(dng->ioport, DNG_PORT_SIZE, RTCAN_DRV_NAME)) {
	ret = -EBUSY;
	goto out_dev_free;
    }

    if (dng->type == DONGLE_TYPE_EPP) {
	if (!request_region(dng->ecr, ECR_PORT_SIZE, RTCAN_DRV_NAME)) {
	    ret = -EBUSY;
	    goto out_free_region;
	}
    }

    /* Clock frequency in Hz */
    dev->can_sys_clock = 8000000;	/* 16/2 MHz */

    /* Output control register */
    sja->ocr = SJA_OCR_MODE_NORMAL | SJA_OCR_TX0_PUSHPULL;

    sja->cdr = SJA_CDR_CAN_MODE;

    strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

    rtcan_peak_dng_enable(dev);

    /* Register RTDM device */
    ret = rtcan_sja1000_register(dev);
    if (ret) {
	printk(KERN_ERR "ERROR while trying to register SJA1000 device %d!\n",
	       ret);
	goto out_free_region2;
    }

    rtcan_peak_dng_devs[idx] = dev;
    return 0;

 out_free_region2:
    if (dng->type == DONGLE_TYPE_EPP)
	release_region(dng->ecr, ECR_PORT_SIZE);

 out_free_region:
    release_region(dng->ioport, DNG_PORT_SIZE);

 out_dev_free:
    rtcan_dev_free(dev);

    return ret;
}

void rtcan_peak_dng_exit_one(struct rtcan_device *dev)
{
    struct rtcan_peak_dng *dng = (struct rtcan_peak_dng *)dev->board_priv;

    rtcan_sja1000_unregister(dev);
    rtcan_peak_dng_disable(dev);
    if (dng->type == DONGLE_TYPE_EPP)
	release_region(dng->ecr, ECR_PORT_SIZE);
    release_region(dng->ioport, DNG_PORT_SIZE);
    rtcan_dev_free(dev);
}

static const struct pnp_device_id rtcan_peak_dng_pnp_tbl[] = {
    /* Standard LPT Printer Port */
    {.id = "PNP0400", .driver_data = 0},
    /* ECP Printer Port */
    {.id = "PNP0401", .driver_data = 0},
    { }
};

static int rtcan_peak_dng_pnp_probe(struct pnp_dev *dev,
				    const struct pnp_device_id *id)
{
    return 0;
}

static struct pnp_driver rtcan_peak_dng_pnp_driver = {
    .name     = RTCAN_DRV_NAME,
    .id_table = rtcan_peak_dng_pnp_tbl,
    .probe    = rtcan_peak_dng_pnp_probe,
};

static int pnp_registered;

/** Cleanup module */
static void rtcan_peak_dng_exit(void)
{
    int i;
    struct rtcan_device *dev;

    for (i = 0, dev = rtcan_peak_dng_devs[i];
	 i < RTCAN_PEAK_DNG_MAX_DEV && dev != NULL;
	 i++)
	rtcan_peak_dng_exit_one(dev);

    if (pnp_registered)
	pnp_unregister_driver(&rtcan_peak_dng_pnp_driver);
}

/** Init module */
static int __init rtcan_peak_dng_init(void)
{
    int i, ret = -EINVAL, done = 0;

    if (!rtdm_available())
	return -ENOSYS;

    if (pnp_register_driver(&rtcan_peak_dng_pnp_driver) == 0)
	pnp_registered = 1;

    for (i = 0;
	 i < RTCAN_PEAK_DNG_MAX_DEV && type[i] != 0;
	 i++) {

	if ((ret = rtcan_peak_dng_init_one(i)) != 0) {
	    printk(KERN_ERR "%s: Init failed with %d\n", RTCAN_DRV_NAME, ret);
	    goto cleanup;
	}
	done++;
    }
    if (done)
	return 0;

    printk(KERN_ERR "%s: Please specify type=epp or type=sp\n",
	   RTCAN_DRV_NAME);

cleanup:
    rtcan_peak_dng_exit();
    return ret;
}

module_init(rtcan_peak_dng_init);
module_exit(rtcan_peak_dng_exit);
