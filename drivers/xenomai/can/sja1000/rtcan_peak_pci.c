/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from the PCAN project file driver/src/pcan_pci.c:
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
#include <linux/delay.h>
#include <linux/pci.h>
#include <asm/io.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "PEAK-PCI-CAN"

static char *peak_pci_board_name = "PEAK-PCI";

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("RTCAN board driver for PEAK-PCI cards");
MODULE_SUPPORTED_DEVICE("PEAK-PCI card CAN controller");
MODULE_LICENSE("GPL");

struct rtcan_peak_pci
{
    struct pci_dev *pci_dev;
    struct rtcan_device *slave_dev;
    int channel;
    volatile void __iomem *base_addr;
    volatile void __iomem *conf_addr;
};

#define PEAK_PCI_CAN_SYS_CLOCK (16000000 / 2)

#define PELICAN_SINGLE  (SJA_CDR_CAN_MODE | SJA_CDR_CBP | 0x07 | SJA_CDR_CLK_OFF)
#define PELICAN_MASTER  (SJA_CDR_CAN_MODE | SJA_CDR_CBP | 0x07            )
#define PELICAN_DEFAULT (SJA_CDR_CAN_MODE                                 )

#define CHANNEL_SINGLE 0 /* this is a single channel device */
#define CHANNEL_MASTER 1 /* multi channel device, this device is master */
#define CHANNEL_SLAVE  2 /* multi channel device, this is slave */

// important PITA registers
#define PITA_ICR         0x00        // interrupt control register
#define PITA_GPIOICR     0x18        // general purpose IO interface control register
#define PITA_MISC        0x1C        // miscellanoes register

#define PEAK_PCI_VENDOR_ID      0x001C  // the PCI device and vendor IDs
#define PEAK_PCI_DEVICE_ID      0x0001  // Device ID for PCI and older PCIe cards
#define PEAK_PCIE_DEVICE_ID     0x0003  // Device ID for newer PCIe cards (IPEH-003027)
#define PEAK_CPCI_DEVICE_ID     0x0004  // for nextgen cPCI slot cards
#define PEAK_MPCI_DEVICE_ID     0x0005  // for nextgen miniPCI slot cards
#define PEAK_PC_104P_DEVICE_ID  0x0006  // PCAN-PC/104+ cards
#define PEAK_PCI_104E_DEVICE_ID 0x0007  // PCAN-PCI/104 Express cards
#define PEAK_MPCIE_DEVICE_ID    0x0008  // The miniPCIe slot cards
#define PEAK_PCIE_OEM_ID        0x0009  // PCAN-PCI Express OEM

#define PCI_CONFIG_PORT_SIZE 0x1000  // size of the config io-memory
#define PCI_PORT_SIZE        0x0400  // size of a channel io-memory

static struct pci_device_id peak_pci_tbl[] = {
	{PEAK_PCI_VENDOR_ID, PEAK_PCI_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_PCIE_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_MPCI_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_MPCIE_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_PC_104P_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_PCI_104E_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_CPCI_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{PEAK_PCI_VENDOR_ID, PEAK_PCIE_OEM_ID, PCI_ANY_ID, PCI_ANY_ID,},
	{ }
};
MODULE_DEVICE_TABLE (pci, peak_pci_tbl);


static u8 rtcan_peak_pci_read_reg(struct rtcan_device *dev, int port)
{
    struct rtcan_peak_pci *board = (struct rtcan_peak_pci *)dev->board_priv;
    return readb(board->base_addr + ((unsigned long)port << 2));
}

static void rtcan_peak_pci_write_reg(struct rtcan_device *dev, int port, u8 data)
{
    struct rtcan_peak_pci *board = (struct rtcan_peak_pci *)dev->board_priv;
    writeb(data, board->base_addr + ((unsigned long)port << 2));
}

static void rtcan_peak_pci_irq_ack(struct rtcan_device *dev)
{
    struct rtcan_peak_pci *board = (struct rtcan_peak_pci *)dev->board_priv;
    u16 pita_icr_low;

    /* Select and clear in Pita stored interrupt */
    pita_icr_low = readw(board->conf_addr + PITA_ICR);
    if (board->channel == CHANNEL_SLAVE) {
	if (pita_icr_low & 0x0001)
	    writew(0x0001, board->conf_addr + PITA_ICR);
    }
    else {
	if (pita_icr_low & 0x0002)
	    writew(0x0002, board->conf_addr + PITA_ICR);
    }
}

static void rtcan_peak_pci_del_chan(struct rtcan_device *dev,
				    int init_step)
{
    struct rtcan_peak_pci *board;
    u16 pita_icr_high;

    if (!dev)
	return;

    board = (struct rtcan_peak_pci *)dev->board_priv;

    switch (init_step) {
    case 0:			/* Full cleanup */
	printk("Removing %s %s device %s\n",
	       peak_pci_board_name, dev->ctrl_name, dev->name);
	rtcan_sja1000_unregister(dev);
	/* fallthrough */
    case 5:
	pita_icr_high = readw(board->conf_addr + PITA_ICR + 2);
	if (board->channel == CHANNEL_SLAVE) {
	    pita_icr_high &= ~0x0001;
	} else {
	    pita_icr_high &= ~0x0002;
	}
	writew(pita_icr_high, board->conf_addr + PITA_ICR + 2);
	/* fallthrough */
    case 4:
	iounmap((void *)board->base_addr);
	/* fallthrough */
    case 3:
	if (board->channel != CHANNEL_SLAVE)
	    iounmap((void *)board->conf_addr);
	/* fallthrough */
    case 2:
	rtcan_dev_free(dev);
	/* fallthrough */
    case 1:
	break;
    }

}

static int rtcan_peak_pci_add_chan(struct pci_dev *pdev, int channel,
				   struct rtcan_device **master_dev)
{
    struct rtcan_device *dev;
    struct rtcan_sja1000 *chip;
    struct rtcan_peak_pci *board;
    u16 pita_icr_high;
    unsigned long addr;
    int ret, init_step = 1;

    dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
			  sizeof(struct rtcan_peak_pci));
    if (dev == NULL)
	return -ENOMEM;
    init_step = 2;

    chip = (struct rtcan_sja1000 *)dev->priv;
    board = (struct rtcan_peak_pci *)dev->board_priv;

    board->pci_dev = pdev;
    board->channel = channel;

    if (channel != CHANNEL_SLAVE) {

	addr = pci_resource_start(pdev, 0);
	board->conf_addr = ioremap(addr, PCI_CONFIG_PORT_SIZE);
	if (board->conf_addr == 0) {
	    ret = -ENODEV;
	    goto failure;
	}
	init_step = 3;

	/* Set GPIO control register */
	writew(0x0005, board->conf_addr + PITA_GPIOICR + 2);

	if (channel == CHANNEL_MASTER)
	    writeb(0x00, board->conf_addr + PITA_GPIOICR); /* enable both */
	else
	    writeb(0x04, board->conf_addr + PITA_GPIOICR); /* enable single */

	writeb(0x05, board->conf_addr + PITA_MISC + 3);  /* toggle reset */
	mdelay(5);
	writeb(0x04, board->conf_addr + PITA_MISC + 3);  /* leave parport mux mode */
    } else {
	struct rtcan_peak_pci *master_board =
	    (struct rtcan_peak_pci *)(*master_dev)->board_priv;
	master_board->slave_dev = dev;
	board->conf_addr = master_board->conf_addr;
    }

    addr = pci_resource_start(pdev, 1);
    if (channel == CHANNEL_SLAVE)
	addr += 0x400;

    board->base_addr = ioremap(addr, PCI_PORT_SIZE);
    if (board->base_addr == 0) {
	ret = -ENODEV;
	goto failure;
    }
    init_step = 4;

    dev->board_name = peak_pci_board_name;

    chip->read_reg = rtcan_peak_pci_read_reg;
    chip->write_reg = rtcan_peak_pci_write_reg;
    chip->irq_ack = rtcan_peak_pci_irq_ack;

    /* Clock frequency in Hz */
    dev->can_sys_clock = PEAK_PCI_CAN_SYS_CLOCK;

    /* Output control register */
    chip->ocr = SJA_OCR_MODE_NORMAL | SJA_OCR_TX0_PUSHPULL;

    /* Clock divider register */
    if (channel == CHANNEL_MASTER)
	chip->cdr = PELICAN_MASTER;
    else
	chip->cdr = PELICAN_SINGLE;

    strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

    /* Register and setup interrupt handling */
    chip->irq_flags = RTDM_IRQTYPE_SHARED;
    chip->irq_num = pdev->irq;
    pita_icr_high = readw(board->conf_addr + PITA_ICR + 2);
    if (channel == CHANNEL_SLAVE) {
	pita_icr_high |= 0x0001;
    } else {
	pita_icr_high |= 0x0002;
    }
    writew(pita_icr_high, board->conf_addr + PITA_ICR + 2);
    init_step = 5;

    printk("%s: base_addr=%p conf_addr=%p irq=%d\n", RTCAN_DRV_NAME,
	   board->base_addr, board->conf_addr, chip->irq_num);

    /* Register SJA1000 device */
    ret = rtcan_sja1000_register(dev);
    if (ret) {
	printk(KERN_ERR
	       "ERROR %d while trying to register SJA1000 device!\n", ret);
	goto failure;
    }

    if (channel != CHANNEL_SLAVE)
	*master_dev = dev;

    return 0;

 failure:
    rtcan_peak_pci_del_chan(dev, init_step);
    return ret;
}

static int peak_pci_init_one(struct pci_dev *pdev,
			     const struct pci_device_id *ent)
{
    int ret;
    u16 sub_sys_id;
    struct rtcan_device *master_dev = NULL;

    if (!rtdm_available())
	return -ENODEV;

    printk("%s: initializing device %04x:%04x\n",
	   RTCAN_DRV_NAME,  pdev->vendor, pdev->device);

    if ((ret = pci_enable_device (pdev)))
	goto failure;

    if ((ret = pci_request_regions(pdev, RTCAN_DRV_NAME)))
	goto failure;

    if ((ret = pci_read_config_word(pdev, 0x2e, &sub_sys_id)))
	goto failure_cleanup;

    /* Enable memory space */
    if ((ret = pci_write_config_word(pdev, 0x04, 2)))
	goto failure_cleanup;

    if ((ret = pci_write_config_word(pdev, 0x44, 0)))
	goto failure_cleanup;

    if (sub_sys_id > 3) {
	if ((ret = rtcan_peak_pci_add_chan(pdev, CHANNEL_MASTER,
					   &master_dev)))
	    goto failure_cleanup;
	if ((ret = rtcan_peak_pci_add_chan(pdev, CHANNEL_SLAVE,
					   &master_dev)))
	    goto failure_cleanup;
    } else {
	if ((ret = rtcan_peak_pci_add_chan(pdev, CHANNEL_SINGLE,
					   &master_dev)))
	    goto failure_cleanup;
    }

    pci_set_drvdata(pdev, master_dev);
    return 0;

 failure_cleanup:
    if (master_dev)
	rtcan_peak_pci_del_chan(master_dev, 0);

    pci_release_regions(pdev);

 failure:
    return ret;

}

static void peak_pci_remove_one(struct pci_dev *pdev)
{
    struct rtcan_device *dev = pci_get_drvdata(pdev);
    struct rtcan_peak_pci *board = (struct rtcan_peak_pci *)dev->board_priv;

    if (board->slave_dev)
	rtcan_peak_pci_del_chan(board->slave_dev, 0);
    rtcan_peak_pci_del_chan(dev, 0);

    pci_release_regions(pdev);
    pci_disable_device(pdev);
    pci_set_drvdata(pdev, NULL);
}

static struct pci_driver rtcan_peak_pci_driver = {
	.name		= RTCAN_DRV_NAME,
	.id_table	= peak_pci_tbl,
	.probe		= peak_pci_init_one,
	.remove		= peak_pci_remove_one,
};

module_pci_driver(rtcan_peak_pci_driver);
