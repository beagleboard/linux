/*
 * Copyright (C) 2009 Sebastian Smolorz <sesmo@gmx.net>
 *
 * This driver is based on the Socket-CAN driver esd_pci.c,
 * Copyright (C) 2007 Wolfgang Grandegger <wg@grandegger.com>
 * Copyright (C) 2008 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 * Copyright (C) 2009 Matthias Fuchs <matthias.fuchs@esd.eu>, esd gmbh
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
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
#define RTCAN_DRV_NAME "ESD-PCI-CAN"

static char *esd_pci_board_name = "ESD-PCI";

MODULE_AUTHOR("Sebastian Smolorz <sesmo@gmx.net");
MODULE_DESCRIPTION("RTCAN board driver for esd PCI/PMC/CPCI/PCIe/PCI104 " \
		   "CAN cards");
MODULE_SUPPORTED_DEVICE("esd CAN-PCI/200, CAN-PCI/266, CAN-PMC266, " \
			"CAN-PCIe/2000, CAN-CPCI/200, CAN-PCI104");
MODULE_LICENSE("GPL v2");

struct rtcan_esd_pci {
	struct pci_dev *pci_dev;
	struct rtcan_device *slave_dev;
	void __iomem *conf_addr;
	void __iomem *base_addr;
};

#define ESD_PCI_CAN_CLOCK	(16000000 / 2)

#define ESD_PCI_OCR		(SJA_OCR_TX0_PUSHPULL | SJA_OCR_TX1_PUSHPULL | \
				 SJA_OCR_TX1_INVERT | SJA_OCR_MODE_CLOCK)
#define ESD_PCI_CDR		(SJA_CDR_CLK_OFF | SJA_CDR_CBP | \
				 SJA_CDR_CAN_MODE)

#define CHANNEL_SINGLE 0 /* this is a single channel device */
#define CHANNEL_MASTER 1 /* multi channel device, this device is master */
#define CHANNEL_SLAVE  2 /* multi channel device, this is slave */

#define CHANNEL_OFFSET		0x100

#define INTCSR_OFFSET		0x4c /* Offset in PLX9050 conf registers */
#define INTCSR_LINTI1		(1 << 0)
#define INTCSR_PCI		(1 << 6)

#define INTCSR9056_OFFSET	0x68 /* Offset in PLX9056 conf registers */
#define INTCSR9056_LINTI	(1 << 11)
#define INTCSR9056_PCI		(1 << 8)

#ifndef PCI_DEVICE_ID_PLX_9056
# define PCI_DEVICE_ID_PLX_9056 0x9056
#endif

/* PCI subsystem IDs of esd's SJA1000 based CAN cards */

/* CAN-PCI/200: PCI, 33MHz only, bridge: PLX9050 */
#define ESD_PCI_SUB_SYS_ID_PCI200	0x0004

/* CAN-PCI/266: PCI, 33/66MHz, bridge: PLX9056 */
#define ESD_PCI_SUB_SYS_ID_PCI266	0x0009

/* CAN-PMC/266: PMC module, 33/66MHz, bridge: PLX9056 */
#define ESD_PCI_SUB_SYS_ID_PMC266	0x000e

/* CAN-CPCI/200: Compact PCI, 33MHz only, bridge: PLX9030 */
#define ESD_PCI_SUB_SYS_ID_CPCI200	0x010b

/* CAN-PCIE/2000: PCI Express 1x, bridge: PEX8311 = PEX8111 + PLX9056 */
#define ESD_PCI_SUB_SYS_ID_PCIE2000	0x0200

/* CAN-PCI/104: PCI104 module, 33MHz only, bridge: PLX9030 */
#define ESD_PCI_SUB_SYS_ID_PCI104200	0x0501

static struct pci_device_id esd_pci_tbl[] = {
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9050,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_PCI200},
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9056,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_PCI266},
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9056,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_PMC266},
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_CPCI200},
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9056,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_PCIE2000},
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030,
	 PCI_VENDOR_ID_ESDGMBH, ESD_PCI_SUB_SYS_ID_PCI104200},
	{0,}
};

#define ESD_PCI_BASE_SIZE  0x200

MODULE_DEVICE_TABLE(pci, esd_pci_tbl);


static u8 rtcan_esd_pci_read_reg(struct rtcan_device *dev, int port)
{
	struct rtcan_esd_pci *board = (struct rtcan_esd_pci *)dev->board_priv;
	return readb(board->base_addr + port);
}

static void rtcan_esd_pci_write_reg(struct rtcan_device *dev, int port, u8 val)
{
	struct rtcan_esd_pci *board = (struct rtcan_esd_pci *)dev->board_priv;
	writeb(val, board->base_addr + port);
}

static void rtcan_esd_pci_del_chan(struct rtcan_device *dev)
{
	struct rtcan_esd_pci *board;

	if (!dev)
		return;

	board = (struct rtcan_esd_pci *)dev->board_priv;

	printk("Removing %s %s device %s\n",
		esd_pci_board_name, dev->ctrl_name, dev->name);

	rtcan_sja1000_unregister(dev);

	rtcan_dev_free(dev);
}

static int rtcan_esd_pci_add_chan(struct pci_dev *pdev, int channel,
				  struct rtcan_device **master_dev,
				  void __iomem *conf_addr,
				  void __iomem *base_addr)
{
	struct rtcan_device *dev;
	struct rtcan_sja1000 *chip;
	struct rtcan_esd_pci *board;
	int ret;

	dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
			      sizeof(struct rtcan_esd_pci));
	if (dev == NULL)
		return -ENOMEM;

	chip = (struct rtcan_sja1000 *)dev->priv;
	board = (struct rtcan_esd_pci *)dev->board_priv;

	board->pci_dev = pdev;
	board->conf_addr = conf_addr;
	board->base_addr = base_addr;

	if (channel == CHANNEL_SLAVE) {
		struct rtcan_esd_pci *master_board =
			(struct rtcan_esd_pci *)(*master_dev)->board_priv;
		master_board->slave_dev = dev;
	}

	dev->board_name = esd_pci_board_name;

	chip->read_reg = rtcan_esd_pci_read_reg;
	chip->write_reg = rtcan_esd_pci_write_reg;

	dev->can_sys_clock = ESD_PCI_CAN_CLOCK;

	chip->ocr = ESD_PCI_OCR;
	chip->cdr = ESD_PCI_CDR;

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	chip->irq_flags = RTDM_IRQTYPE_SHARED;
	chip->irq_num = pdev->irq;

	RTCAN_DBG("%s: base_addr=0x%p conf_addr=0x%p irq=%d ocr=%#x cdr=%#x\n",
		  RTCAN_DRV_NAME, board->base_addr, board->conf_addr,
		  chip->irq_num, chip->ocr, chip->cdr);

	/* Register SJA1000 device */
	ret = rtcan_sja1000_register(dev);
	if (ret) {
		printk(KERN_ERR "ERROR %d while trying to register SJA1000 "
				"device!\n", ret);
		goto failure;
	}

	if (channel != CHANNEL_SLAVE)
		*master_dev = dev;

	return 0;


failure:
	rtcan_dev_free(dev);
	return ret;
}

static int esd_pci_init_one(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	int ret, channel;
	void __iomem *base_addr;
	void __iomem *conf_addr;
	struct rtcan_device *master_dev = NULL;

	if (!rtdm_available())
		return -ENODEV;

	if ((ret = pci_enable_device (pdev)))
		goto failure;

	if ((ret = pci_request_regions(pdev, RTCAN_DRV_NAME)))
		goto failure;

	RTCAN_DBG("%s: Initializing device %04x:%04x %04x:%04x\n",
		 RTCAN_DRV_NAME, pdev->vendor, pdev->device,
		 pdev->subsystem_vendor, pdev->subsystem_device);

	conf_addr = pci_iomap(pdev, 0, ESD_PCI_BASE_SIZE);
	if (conf_addr == NULL) {
		ret = -ENODEV;
		goto failure_release_pci;
	}

	base_addr = pci_iomap(pdev, 2, ESD_PCI_BASE_SIZE);
	if (base_addr == NULL) {
		ret = -ENODEV;
		goto failure_iounmap_conf;
	}

	/* Check if second channel is available */
	writeb(SJA_MOD_RM, base_addr + CHANNEL_OFFSET + SJA_MOD);
	writeb(SJA_CDR_CBP, base_addr + CHANNEL_OFFSET + SJA_CDR);
	writeb(SJA_MOD_RM, base_addr + CHANNEL_OFFSET + SJA_MOD);
	if (readb(base_addr + CHANNEL_OFFSET + SJA_MOD) == 0x21) {
		writeb(SJA_MOD_SM | SJA_MOD_AFM | SJA_MOD_STM | SJA_MOD_LOM |
		       SJA_MOD_RM, base_addr + CHANNEL_OFFSET + SJA_MOD);
		if (readb(base_addr + CHANNEL_OFFSET + SJA_MOD) == 0x3f)
			channel = CHANNEL_MASTER;
		else {
			writeb(SJA_MOD_RM,
				base_addr + CHANNEL_OFFSET + SJA_MOD);
			channel = CHANNEL_SINGLE;
		}
	} else {
		writeb(SJA_MOD_RM, base_addr + CHANNEL_OFFSET + SJA_MOD);
		channel = CHANNEL_SINGLE;
	}

	if ((ret = rtcan_esd_pci_add_chan(pdev, channel, &master_dev,
						conf_addr, base_addr)))
		goto failure_iounmap_base;

	if (channel != CHANNEL_SINGLE) {
		channel = CHANNEL_SLAVE;
		if ((ret = rtcan_esd_pci_add_chan(pdev, channel, &master_dev,
				      conf_addr, base_addr + CHANNEL_OFFSET)))
			goto failure_iounmap_base;
	}

	if ((pdev->device == PCI_DEVICE_ID_PLX_9050) ||
	    (pdev->device == PCI_DEVICE_ID_PLX_9030)) {
		/* Enable interrupts in PLX9050 */
		writel(INTCSR_LINTI1 | INTCSR_PCI, conf_addr + INTCSR_OFFSET);
	} else {
		/* Enable interrupts in PLX9056*/
		writel(INTCSR9056_LINTI | INTCSR9056_PCI,
					conf_addr + INTCSR9056_OFFSET);
	}

	pci_set_drvdata(pdev, master_dev);

	return 0;


failure_iounmap_base:
	if (master_dev)
		rtcan_esd_pci_del_chan(master_dev);
	pci_iounmap(pdev, base_addr);

failure_iounmap_conf:
	pci_iounmap(pdev, conf_addr);

failure_release_pci:
	pci_release_regions(pdev);

failure:
	return ret;
}

static void esd_pci_remove_one(struct pci_dev *pdev)
{
	struct rtcan_device *dev = pci_get_drvdata(pdev);
	struct rtcan_esd_pci *board = (struct rtcan_esd_pci *)dev->board_priv;

	if ((pdev->device == PCI_DEVICE_ID_PLX_9050) ||
	    (pdev->device == PCI_DEVICE_ID_PLX_9030)) {
		/* Disable interrupts in PLX9050*/
		writel(0, board->conf_addr + INTCSR_OFFSET);
	} else {
		/* Disable interrupts in PLX9056*/
		writel(0, board->conf_addr + INTCSR9056_OFFSET);
	}

	if (board->slave_dev)
		rtcan_esd_pci_del_chan(board->slave_dev);
	rtcan_esd_pci_del_chan(dev);


	pci_iounmap(pdev, board->base_addr);
	pci_iounmap(pdev, board->conf_addr);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver rtcan_esd_pci_driver = {
	.name = RTCAN_DRV_NAME,
	.id_table = esd_pci_tbl,
	.probe = esd_pci_init_one,
	.remove = esd_pci_remove_one,
};

module_pci_driver(rtcan_esd_pci_driver);
