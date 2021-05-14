/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
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
#include <linux/pci.h>
#include <asm/io.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME "rtcan%d"
#define RTCAN_DRV_NAME "IXXAT-PCI-CAN"

static char *ixxat_pci_board_name = "IXXAT-PCI";

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("RTCAN board driver for IXXAT-PCI cards");
MODULE_SUPPORTED_DEVICE("IXXAT-PCI card CAN controller");
MODULE_LICENSE("GPL");

struct rtcan_ixxat_pci
{
    struct pci_dev *pci_dev;
    struct rtcan_device *slave_dev;
    int conf_addr;
    void __iomem *base_addr;
};

#define IXXAT_PCI_CAN_SYS_CLOCK (16000000 / 2)

#define CHANNEL_SINGLE 0 /* this is a single channel device */
#define CHANNEL_MASTER 1 /* multi channel device, this device is master */
#define CHANNEL_SLAVE  2 /* multi channel device, this is slave */

#define CHANNEL_OFFSET       0x200
#define CHANNEL_MASTER_RESET 0x110
#define CHANNEL_SLAVE_RESET  (CHANNEL_MASTER_RESET + CHANNEL_OFFSET)

#define IXXAT_INTCSR_OFFSET  0x4c /* Offset in PLX9050 conf registers */
#define IXXAT_INTCSR_SLAVE   0x41 /* LINT1 and PCI interrupt enabled */
#define IXXAT_INTCSR_MASTER  0x08 /* LINT2 enabled */
#define IXXAT_SJA_MOD_MASK   0xa1 /* Mask for reading dual/single channel */

/* PCI vender, device and sub-device ID */
#define IXXAT_PCI_VENDOR_ID  0x10b5
#define IXXAT_PCI_DEVICE_ID  0x9050
#define IXXAT_PCI_SUB_SYS_ID 0x2540

#define IXXAT_CONF_PORT_SIZE 0x0080
#define IXXAT_BASE_PORT_SIZE 0x0400

static struct pci_device_id ixxat_pci_tbl[] = {
	{IXXAT_PCI_VENDOR_ID, IXXAT_PCI_DEVICE_ID,
	 IXXAT_PCI_VENDOR_ID, IXXAT_PCI_SUB_SYS_ID, 0, 0, 0},
	{ }
};
MODULE_DEVICE_TABLE (pci, ixxat_pci_tbl);


static u8 rtcan_ixxat_pci_read_reg(struct rtcan_device *dev, int port)
{
    struct rtcan_ixxat_pci *board = (struct rtcan_ixxat_pci *)dev->board_priv;
    return readb(board->base_addr + port);
}

static void rtcan_ixxat_pci_write_reg(struct rtcan_device *dev, int port, u8 data)
{
    struct rtcan_ixxat_pci *board = (struct rtcan_ixxat_pci *)dev->board_priv;
    writeb(data, board->base_addr + port);
}

static void rtcan_ixxat_pci_del_chan(struct rtcan_device *dev)
{
    struct rtcan_ixxat_pci *board;
    u8 intcsr;

    if (!dev)
	return;

    board = (struct rtcan_ixxat_pci *)dev->board_priv;

    printk("Removing %s %s device %s\n",
	   ixxat_pci_board_name, dev->ctrl_name, dev->name);

    rtcan_sja1000_unregister(dev);

    /* Disable PCI interrupts */
    intcsr = inb(board->conf_addr + IXXAT_INTCSR_OFFSET);
    if (board->slave_dev) {
	intcsr &= ~IXXAT_INTCSR_MASTER;
	outb(intcsr, board->conf_addr + IXXAT_INTCSR_OFFSET);
	writeb(0x1, board->base_addr + CHANNEL_MASTER_RESET);
	iounmap(board->base_addr);
    } else {
	intcsr &= ~IXXAT_INTCSR_SLAVE;
	outb(intcsr, board->conf_addr + IXXAT_INTCSR_OFFSET);
	writeb(0x1, board->base_addr + CHANNEL_SLAVE_RESET );
    }
    rtcan_dev_free(dev);
}

static int rtcan_ixxat_pci_add_chan(struct pci_dev *pdev,
				    int channel,
				    struct rtcan_device **master_dev,
				    int conf_addr,
				    void __iomem *base_addr)
{
    struct rtcan_device *dev;
    struct rtcan_sja1000 *chip;
    struct rtcan_ixxat_pci *board;
    u8 intcsr;
    int ret;

    dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
			  sizeof(struct rtcan_ixxat_pci));
    if (dev == NULL)
	return -ENOMEM;

    chip = (struct rtcan_sja1000 *)dev->priv;
    board = (struct rtcan_ixxat_pci *)dev->board_priv;

    board->pci_dev = pdev;
    board->conf_addr = conf_addr;
    board->base_addr = base_addr;

    if (channel == CHANNEL_SLAVE) {
	struct rtcan_ixxat_pci *master_board =
	    (struct rtcan_ixxat_pci *)(*master_dev)->board_priv;
	master_board->slave_dev = dev;
    }

    dev->board_name = ixxat_pci_board_name;

    chip->read_reg = rtcan_ixxat_pci_read_reg;
    chip->write_reg = rtcan_ixxat_pci_write_reg;

    /* Clock frequency in Hz */
    dev->can_sys_clock = IXXAT_PCI_CAN_SYS_CLOCK;

    /* Output control register */
    chip->ocr = (SJA_OCR_MODE_NORMAL | SJA_OCR_TX0_INVERT |
		 SJA_OCR_TX0_PUSHPULL | SJA_OCR_TX1_PUSHPULL);

    /* Clock divider register */
    chip->cdr = SJA_CDR_CAN_MODE;

    strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

    /* Enable PCI interrupts */
    intcsr = inb(board->conf_addr + IXXAT_INTCSR_OFFSET);
    if (channel == CHANNEL_SLAVE)
	intcsr |= IXXAT_INTCSR_SLAVE;
    else
	intcsr |= IXXAT_INTCSR_MASTER;
    outb(intcsr, board->conf_addr + IXXAT_INTCSR_OFFSET);

    /* Register and setup interrupt handling */
    chip->irq_flags = RTDM_IRQTYPE_SHARED;
    chip->irq_num = pdev->irq;

    RTCAN_DBG("%s: base_addr=0x%p conf_addr=%#x irq=%d ocr=%#x cdr=%#x\n",
	      RTCAN_DRV_NAME, board->base_addr, board->conf_addr,
	      chip->irq_num, chip->ocr, chip->cdr);

    /* Register SJA1000 device */
    ret = rtcan_sja1000_register(dev);
    if (ret) {
	printk(KERN_ERR "ERROR %d while trying to register SJA1000 device!\n",
	       ret);
	goto failure;
    }

    if (channel != CHANNEL_SLAVE)
	*master_dev = dev;

    return 0;

 failure:
    rtcan_dev_free(dev);
    return ret;
}

static int ixxat_pci_init_one(struct pci_dev *pdev,
			      const struct pci_device_id *ent)
{
    int ret, channel, conf_addr;
    unsigned long addr;
    void __iomem *base_addr;
    struct rtcan_device *master_dev = NULL;

    if (!rtdm_available())
	return -ENODEV;

    if ((ret = pci_enable_device (pdev)))
	goto failure;

    if ((ret = pci_request_regions(pdev, RTCAN_DRV_NAME)))
	goto failure;

    RTCAN_DBG("%s: Initializing device %04x:%04x:%04x\n",
	      RTCAN_DRV_NAME, pdev->vendor, pdev->device,
	      pdev->subsystem_device);

    /* Enable memory and I/O space */
    if ((ret = pci_write_config_word(pdev, 0x04, 0x3)))
	goto failure_release_pci;

    conf_addr = pci_resource_start(pdev, 1);

    addr = pci_resource_start(pdev, 2);
    base_addr = ioremap(addr, IXXAT_BASE_PORT_SIZE);
    if (base_addr == 0) {
	ret = -ENODEV;
	goto failure_release_pci;
    }

    /* Check if second channel is available after reset */
    writeb(0x1, base_addr + CHANNEL_MASTER_RESET);
    writeb(0x1, base_addr + CHANNEL_SLAVE_RESET);
    udelay(100);
    if ( (readb(base_addr + CHANNEL_OFFSET + SJA_MOD) & IXXAT_SJA_MOD_MASK ) != 0x21 ||
	readb(base_addr + CHANNEL_OFFSET + SJA_SR ) != 0x0c ||
	readb(base_addr + CHANNEL_OFFSET + SJA_IR ) != 0xe0)
	channel = CHANNEL_SINGLE;
    else
	channel = CHANNEL_MASTER;

    if ((ret = rtcan_ixxat_pci_add_chan(pdev, channel, &master_dev,
					conf_addr, base_addr)))
	goto failure_iounmap;

    if (channel != CHANNEL_SINGLE) {
	channel = CHANNEL_SLAVE;
	if ((ret = rtcan_ixxat_pci_add_chan(pdev, channel,
					    &master_dev, conf_addr,
					    base_addr + CHANNEL_OFFSET)))
	    goto failure_iounmap;
    }

    pci_set_drvdata(pdev, master_dev);
    return 0;

failure_iounmap:
    if (master_dev)
	rtcan_ixxat_pci_del_chan(master_dev);
    iounmap(base_addr);

failure_release_pci:
    pci_release_regions(pdev);

failure:
    return ret;
}

static void ixxat_pci_remove_one(struct pci_dev *pdev)
{
    struct rtcan_device *dev = pci_get_drvdata(pdev);
    struct rtcan_ixxat_pci *board = (struct rtcan_ixxat_pci *)dev->board_priv;

    if (board->slave_dev)
	rtcan_ixxat_pci_del_chan(board->slave_dev);
    rtcan_ixxat_pci_del_chan(dev);

    pci_release_regions(pdev);
    pci_disable_device(pdev);
    pci_set_drvdata(pdev, NULL);
}

static struct pci_driver rtcan_ixxat_pci_driver = {
	.name = RTCAN_DRV_NAME,
	.id_table = ixxat_pci_tbl,
	.probe = ixxat_pci_init_one,
	.remove = ixxat_pci_remove_one,
};

module_pci_driver(rtcan_ixxat_pci_driver);
