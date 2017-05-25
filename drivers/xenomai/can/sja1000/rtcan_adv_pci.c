/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Copyright (C) 2012 Thierry Bultel <thierry.bultel@basystemes.fr>
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
#include <linux/io.h>

#include <rtdm/driver.h>

#define ADV_PCI_BASE_SIZE	0x80

/* CAN device profile */
#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME "rtcan%d"
#define RTCAN_DRV_NAME "ADV-PCI-CAN"

static char *adv_pci_board_name = "ADV-PCI";

MODULE_AUTHOR("Thierry Bultel <thierry.bultel@basystemes.fr>");
MODULE_DESCRIPTION("RTCAN board driver for Advantech PCI cards");
MODULE_SUPPORTED_DEVICE("ADV-PCI card CAN controller");
MODULE_LICENSE("GPL");

struct rtcan_adv_pci {
	struct pci_dev *pci_dev;
	struct rtcan_device *slave_dev;
	void __iomem *conf_addr;
	void __iomem *base_addr;
};

/*
 * According to the datasheet,
 * internal clock is 1/2 of the external oscillator frequency
 * which is 16 MHz
 */
#define ADV_PCI_CAN_CLOCK (16000000 / 2)

/*
 * Output control register
  Depends on the board configuration
 */

#define ADV_PCI_OCR (SJA_OCR_MODE_NORMAL	|\
		     SJA_OCR_TX0_PUSHPULL	|\
		     SJA_OCR_TX1_PUSHPULL	|\
		     SJA_OCR_TX1_INVERT)

/*
 * In the CDR register, you should set CBP to 1.
 */
#define ADV_PCI_CDR (SJA_CDR_CBP | SJA_CDR_CAN_MODE)

#define ADV_PCI_VENDOR_ID 0x13fe

#define CHANNEL_SINGLE 0 /* this is a single channel device */
#define CHANNEL_MASTER 1 /* multi channel device, this device is master */
#define CHANNEL_SLAVE  2 /* multi channel device, this is slave */

#define ADV_PCI_DEVICE(device_id)\
	{ ADV_PCI_VENDOR_ID, device_id, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 }

static const struct pci_device_id adv_pci_tbl[] = {
	ADV_PCI_DEVICE(0x1680),
	ADV_PCI_DEVICE(0x3680),
	ADV_PCI_DEVICE(0x2052),
	ADV_PCI_DEVICE(0x1681),
	ADV_PCI_DEVICE(0xc001),
	ADV_PCI_DEVICE(0xc002),
	ADV_PCI_DEVICE(0xc004),
	ADV_PCI_DEVICE(0xc101),
	ADV_PCI_DEVICE(0xc102),
	ADV_PCI_DEVICE(0xc104),
	/* required last entry */
	{ }
};

MODULE_DEVICE_TABLE(pci, adv_pci_tbl);

static u8 rtcan_adv_pci_read_reg(struct rtcan_device *dev, int port)
{
	struct rtcan_adv_pci *board = (struct rtcan_adv_pci *)dev->board_priv;

	return ioread8(board->base_addr + port);
}

static void rtcan_adv_pci_write_reg(struct rtcan_device *dev, int port, u8 data)
{
	struct rtcan_adv_pci *board = (struct rtcan_adv_pci *)dev->board_priv;

	iowrite8(data, board->base_addr + port);
}

static void rtcan_adv_pci_del_chan(struct pci_dev *pdev,
				   struct rtcan_device *dev)
{
	struct rtcan_adv_pci *board;

	if (!dev)
		return;

	board = (struct rtcan_adv_pci *)dev->board_priv;

	rtcan_sja1000_unregister(dev);

	pci_iounmap(pdev, board->base_addr);

	rtcan_dev_free(dev);
}


static int rtcan_adv_pci_add_chan(struct pci_dev *pdev,
				  int channel,
				  unsigned int bar,
				  unsigned int offset,
				  struct rtcan_device **master_dev)
{
	struct rtcan_device *dev;
	struct rtcan_sja1000 *chip;
	struct rtcan_adv_pci *board;
	void __iomem *base_addr;
	int ret;

	dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
			      sizeof(struct rtcan_adv_pci));
	if (dev == NULL)
		return -ENOMEM;

	chip = (struct rtcan_sja1000 *)dev->priv;
	board = (struct rtcan_adv_pci *)dev->board_priv;

	if (channel == CHANNEL_SLAVE) {
		struct rtcan_adv_pci *master_board =
			(struct rtcan_adv_pci *)(*master_dev)->board_priv;
		master_board->slave_dev = dev;

		if (offset)
			base_addr = master_board->base_addr+offset;
		else
			base_addr = pci_iomap(pdev, bar, ADV_PCI_BASE_SIZE);
			if (!base_addr) {
				ret = -EIO;
				goto failure;
			}
	} else {
		base_addr = pci_iomap(pdev, bar, ADV_PCI_BASE_SIZE) + offset;
		if (!base_addr) {
			ret = -EIO;
			goto failure;
		}
	}

	board->pci_dev = pdev;
	board->conf_addr = NULL;
	board->base_addr = base_addr;

	dev->board_name = adv_pci_board_name;

	chip->read_reg = rtcan_adv_pci_read_reg;
	chip->write_reg = rtcan_adv_pci_write_reg;

	/* Clock frequency in Hz */
	dev->can_sys_clock = ADV_PCI_CAN_CLOCK;

	/* Output control register */
	chip->ocr = ADV_PCI_OCR;

	/* Clock divider register */
	chip->cdr = ADV_PCI_CDR;

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	/* Make sure SJA1000 is in reset mode */
	chip->write_reg(dev, SJA_MOD, SJA_MOD_RM);
	/* Set PeliCAN mode */
	chip->write_reg(dev, SJA_CDR, SJA_CDR_CAN_MODE);

	/* check if mode is set */
	ret = chip->read_reg(dev, SJA_CDR);
	if (ret != SJA_CDR_CAN_MODE) {
		ret = -EIO;
		goto failure_iounmap;
	}

	/* Register and setup interrupt handling */
	chip->irq_flags = RTDM_IRQTYPE_SHARED;
	chip->irq_num = pdev->irq;

	RTCAN_DBG("%s: base_addr=%p conf_addr=%p irq=%d ocr=%#x cdr=%#x\n",
		   RTCAN_DRV_NAME, board->base_addr, board->conf_addr,
		   chip->irq_num, chip->ocr, chip->cdr);

	/* Register SJA1000 device */
	ret = rtcan_sja1000_register(dev);
	if (ret) {
		printk(KERN_ERR "ERROR %d while trying to register SJA1000 device!\n",
		       ret);
		goto failure_iounmap;
	}

	if (channel != CHANNEL_SLAVE)
		*master_dev = dev;

	return 0;

failure_iounmap:
	if (channel != CHANNEL_SLAVE || !offset)
		pci_iounmap(pdev, base_addr);
failure:
	rtcan_dev_free(dev);

	return ret;
}

static int adv_pci_init_one(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	int ret, channel;
	unsigned int nb_ports = 0;
	unsigned int bar = 0;
	unsigned int bar_flag = 0;
	unsigned int offset = 0;
	unsigned int ix;

	struct rtcan_device *master_dev = NULL;

	dev_info(&pdev->dev, "RTCAN Registering card");

	ret = pci_enable_device(pdev);
	if (ret)
		goto failure;

	dev_info(&pdev->dev, "RTCAN detected Advantech PCI card at slot #%i\n",
		 PCI_SLOT(pdev->devfn));

	ret = pci_request_regions(pdev, RTCAN_DRV_NAME);
	if (ret)
		goto failure_device;

	switch (pdev->device) {
	case 0xc001:
	case 0xc002:
	case 0xc004:
	case 0xc101:
	case 0xc102:
	case 0xc104:
		nb_ports = pdev->device & 0x7;
		offset = 0x100;
		bar = 0;
		break;
	case 0x1680:
	case 0x2052:
		nb_ports = 2;
		bar = 2;
		bar_flag = 1;
		break;
	case 0x1681:
		nb_ports = 1;
		bar = 2;
		bar_flag = 1;
		break;
	default:
		goto failure_regions;
	}

	if (nb_ports > 1)
		channel = CHANNEL_MASTER;
	else
		channel = CHANNEL_SINGLE;

	RTCAN_DBG("%s: Initializing device %04x:%04x:%04x\n",
		   RTCAN_DRV_NAME,
		   pdev->vendor,
		   pdev->device,
		   pdev->subsystem_device);

	ret = rtcan_adv_pci_add_chan(pdev, channel, bar, offset, &master_dev);
	if (ret)
		goto failure_iounmap;

	/* register slave channel, if any */

	for (ix = 1; ix < nb_ports; ix++) {
		ret = rtcan_adv_pci_add_chan(pdev,
					     CHANNEL_SLAVE,
					     bar + (bar_flag ? ix : 0),
					     offset * ix,
					     &master_dev);
		if (ret)
			goto failure_iounmap;
	}

	pci_set_drvdata(pdev, master_dev);

	return 0;

failure_iounmap:
	if (master_dev)
		rtcan_adv_pci_del_chan(pdev, master_dev);

failure_regions:
	pci_release_regions(pdev);

failure_device:
	pci_disable_device(pdev);

failure:
	return ret;
}

static void adv_pci_remove_one(struct pci_dev *pdev)
{
	struct rtcan_device *dev = pci_get_drvdata(pdev);
	struct rtcan_adv_pci *board = (struct rtcan_adv_pci *)dev->board_priv;

	if (board->slave_dev)
		rtcan_adv_pci_del_chan(pdev, board->slave_dev);

	rtcan_adv_pci_del_chan(pdev, dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver rtcan_adv_pci_driver = {
	.name = RTCAN_DRV_NAME,
	.id_table = adv_pci_tbl,
	.probe = adv_pci_init_one,
	.remove = adv_pci_remove_one,
};

static int __init rtcan_adv_pci_init(void)
{
	if (!realtime_core_enabled())
		return 0;

	return pci_register_driver(&rtcan_adv_pci_driver);
}

static void __exit rtcan_adv_pci_exit(void)
{
	if (realtime_core_enabled())
		pci_unregister_driver(&rtcan_adv_pci_driver);
}

module_init(rtcan_adv_pci_init);
module_exit(rtcan_adv_pci_exit);
