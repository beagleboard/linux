/*
 * Copyright (C) 2007, 2016 Wolfgang Grandegger <wg@grandegger.com>
 * Copyright (C) 2008 Markus Plessing <plessing@ems-wuensche.com>
 * Copyright (C) 2008 Sebastian Haas <haas@ems-wuensche.com>
 *
 * Derived from Linux CAN SJA1000 PCI driver "ems_pci".
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
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/io.h>

#include <rtdm/driver.h>

/* CAN device profile */
#include <rtdm/can.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "EMS-CPC-PCI-CAN"

static char *ems_pci_board_name = "EMS-CPC-PCI";

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("RTCAN board driver for EMS CPC-PCI/PCIe/104P CAN cards");
MODULE_SUPPORTED_DEVICE("EMS CPC-PCI/PCIe/104P CAN card");
MODULE_LICENSE("GPL v2");

#define EMS_PCI_V1_MAX_CHAN 2
#define EMS_PCI_V2_MAX_CHAN 4
#define EMS_PCI_MAX_CHAN    EMS_PCI_V2_MAX_CHAN

struct ems_pci_card {
	int version;
	int channels;

	struct pci_dev *pci_dev;
	struct rtcan_device *rtcan_dev[EMS_PCI_MAX_CHAN];

	void __iomem *conf_addr;
	void __iomem *base_addr;
};

#define EMS_PCI_CAN_CLOCK (16000000 / 2)

/*
 * Register definitions and descriptions are from LinCAN 0.3.3.
 *
 * PSB4610 PITA-2 bridge control registers
 */
#define PITA2_ICR           0x00	/* Interrupt Control Register */
#define PITA2_ICR_INT0      0x00000002	/* [RC] INT0 Active/Clear */
#define PITA2_ICR_INT0_EN   0x00020000	/* [RW] Enable INT0 */

#define PITA2_MISC          0x1c	/* Miscellaneous Register */
#define PITA2_MISC_CONFIG   0x04000000	/* Multiplexed parallel interface */

/*
 * Register definitions for the PLX 9030
 */
#define PLX_ICSR            0x4c   /* Interrupt Control/Status register */
#define PLX_ICSR_LINTI1_ENA 0x0001 /* LINTi1 Enable */
#define PLX_ICSR_PCIINT_ENA 0x0040 /* PCI Interrupt Enable */
#define PLX_ICSR_LINTI1_CLR 0x0400 /* Local Edge Triggerable Interrupt Clear */
#define PLX_ICSR_ENA_CLR    (PLX_ICSR_LINTI1_ENA | PLX_ICSR_PCIINT_ENA | \
			     PLX_ICSR_LINTI1_CLR)

/*
 * The board configuration is probably following:
 * RX1 is connected to ground.
 * TX1 is not connected.
 * CLKO is not connected.
 * Setting the OCR register to 0xDA is a good idea.
 * This means normal output mode, push-pull and the correct polarity.
 */
#define EMS_PCI_OCR         (SJA_OCR_TX0_PUSHPULL | SJA_OCR_TX1_PUSHPULL)

/*
 * In the CDR register, you should set CBP to 1.
 * You will probably also want to set the clock divider value to 7
 * (meaning direct oscillator output) because the second SJA1000 chip
 * is driven by the first one CLKOUT output.
 */
#define EMS_PCI_CDR             (SJA_CDR_CBP | SJA_CDR_CLKOUT_MASK)

#define EMS_PCI_V1_BASE_BAR     1
#define EMS_PCI_V1_CONF_SIZE    4096 /* size of PITA control area */
#define EMS_PCI_V2_BASE_BAR     2
#define EMS_PCI_V2_CONF_SIZE    128 /* size of PLX control area */
#define EMS_PCI_CAN_BASE_OFFSET 0x400 /* offset where the controllers starts */
#define EMS_PCI_CAN_CTRL_SIZE   0x200 /* memory size for each controller */

#define EMS_PCI_BASE_SIZE  4096 /* size of controller area */

static const struct pci_device_id ems_pci_tbl[] = {
	/* CPC-PCI v1 */
	{PCI_VENDOR_ID_SIEMENS, 0x2104, PCI_ANY_ID, PCI_ANY_ID,},
	/* CPC-PCI v2 */
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030, PCI_VENDOR_ID_PLX, 0x4000},
	/* CPC-104P v2 */
	{PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9030, PCI_VENDOR_ID_PLX, 0x4002},
	{0,}
};
MODULE_DEVICE_TABLE(pci, ems_pci_tbl);

/*
 * Helper to read internal registers from card logic (not CAN)
 */
static u8 ems_pci_v1_readb(struct ems_pci_card *card, unsigned int port)
{
	return readb((void __iomem *)card->base_addr + (port * 4));
}

static u8 ems_pci_v1_read_reg(struct rtcan_device *dev, int port)
{
	return readb((void __iomem *)dev->base_addr + (port * 4));
}

static void ems_pci_v1_write_reg(struct rtcan_device *dev,
				 int port, u8 val)
{
	writeb(val, (void __iomem *)dev->base_addr + (port * 4));
}

static void ems_pci_v1_post_irq(struct rtcan_device *dev)
{
	struct ems_pci_card *card = (struct ems_pci_card *)dev->board_priv;

	/* reset int flag of pita */
	writel(PITA2_ICR_INT0_EN | PITA2_ICR_INT0,
	       card->conf_addr + PITA2_ICR);
}

static u8 ems_pci_v2_read_reg(struct rtcan_device *dev, int port)
{
	return readb((void __iomem *)dev->base_addr + port);
}

static void ems_pci_v2_write_reg(struct rtcan_device *dev,
				 int port, u8 val)
{
	writeb(val, (void __iomem *)dev->base_addr + port);
}

static void ems_pci_v2_post_irq(struct rtcan_device *dev)
{
	struct ems_pci_card *card = (struct ems_pci_card *)dev->board_priv;

	writel(PLX_ICSR_ENA_CLR, card->conf_addr + PLX_ICSR);
}

/*
 * Check if a CAN controller is present at the specified location
 * by trying to set 'em into the PeliCAN mode
 */
static inline int ems_pci_check_chan(struct rtcan_device *dev)
{
	struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;
	unsigned char res;

	/* Make sure SJA1000 is in reset mode */
	chip->write_reg(dev, SJA_MOD, 1);

	chip->write_reg(dev, SJA_CDR, SJA_CDR_CAN_MODE);

	/* read reset-values */
	res = chip->read_reg(dev, SJA_CDR);

	if (res == SJA_CDR_CAN_MODE)
		return 1;

	return 0;
}

static void ems_pci_del_card(struct pci_dev *pdev)
{
	struct ems_pci_card *card = pci_get_drvdata(pdev);
	struct rtcan_device *dev;
	int i = 0;

	for (i = 0; i < card->channels; i++) {
		dev = card->rtcan_dev[i];

		if (!dev)
			continue;

		dev_info(&pdev->dev, "Removing %s.\n", dev->name);
		rtcan_sja1000_unregister(dev);
		rtcan_dev_free(dev);
	}

	if (card->base_addr != NULL)
		pci_iounmap(card->pci_dev, card->base_addr);

	if (card->conf_addr != NULL)
		pci_iounmap(card->pci_dev, card->conf_addr);

	kfree(card);

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static void ems_pci_card_reset(struct ems_pci_card *card)
{
	/* Request board reset */
	writeb(0, card->base_addr);
}

/*
 * Probe PCI device for EMS CAN signature and register each available
 * CAN channel to RTCAN subsystem.
 */
static int ems_pci_add_card(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	struct rtcan_sja1000 *chip;
	struct rtcan_device *dev;
	struct ems_pci_card *card;
	int max_chan, conf_size, base_bar;
	int err, i;

	/* Enabling PCI device */
	if (pci_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "Enabling PCI device failed\n");
		return -ENODEV;
	}

	/* Allocating card structures to hold addresses, ... */
	card = kzalloc(sizeof(*card), GFP_KERNEL);
	if (card == NULL) {
		pci_disable_device(pdev);
		return -ENOMEM;
	}

	pci_set_drvdata(pdev, card);

	card->pci_dev = pdev;

	card->channels = 0;

	if (pdev->vendor == PCI_VENDOR_ID_PLX) {
		card->version = 2; /* CPC-PCI v2 */
		max_chan = EMS_PCI_V2_MAX_CHAN;
		base_bar = EMS_PCI_V2_BASE_BAR;
		conf_size = EMS_PCI_V2_CONF_SIZE;
	} else {
		card->version = 1; /* CPC-PCI v1 */
		max_chan = EMS_PCI_V1_MAX_CHAN;
		base_bar = EMS_PCI_V1_BASE_BAR;
		conf_size = EMS_PCI_V1_CONF_SIZE;
	}

	/* Remap configuration space and controller memory area */
	card->conf_addr = pci_iomap(pdev, 0, conf_size);
	if (card->conf_addr == NULL) {
		err = -ENOMEM;
		goto failure_cleanup;
	}

	card->base_addr = pci_iomap(pdev, base_bar, EMS_PCI_BASE_SIZE);
	if (card->base_addr == NULL) {
		err = -ENOMEM;
		goto failure_cleanup;
	}

	if (card->version == 1) {
		/* Configure PITA-2 parallel interface (enable MUX) */
		writel(PITA2_MISC_CONFIG, card->conf_addr + PITA2_MISC);

		/* Check for unique EMS CAN signature */
		if (ems_pci_v1_readb(card, 0) != 0x55 ||
		    ems_pci_v1_readb(card, 1) != 0xAA ||
		    ems_pci_v1_readb(card, 2) != 0x01 ||
		    ems_pci_v1_readb(card, 3) != 0xCB ||
		    ems_pci_v1_readb(card, 4) != 0x11) {
			dev_err(&pdev->dev,
				"Not EMS Dr. Thomas Wuensche interface\n");
			err = -ENODEV;
			goto failure_cleanup;
		}
	}

	ems_pci_card_reset(card);

	for (i = 0; i < max_chan; i++) {
		dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000), 0);
		if (!dev) {
			err = -ENOMEM;
			goto failure_cleanup;
		}

		strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);
		dev->board_name = ems_pci_board_name;
		dev->board_priv = card;

		card->rtcan_dev[i] = dev;
		chip = card->rtcan_dev[i]->priv;
		chip->irq_flags = RTDM_IRQTYPE_SHARED;
		chip->irq_num = pdev->irq;

		dev->base_addr = (unsigned long)card->base_addr +
			EMS_PCI_CAN_BASE_OFFSET + (i * EMS_PCI_CAN_CTRL_SIZE);
		if (card->version == 1) {
			chip->read_reg  = ems_pci_v1_read_reg;
			chip->write_reg = ems_pci_v1_write_reg;
			chip->irq_ack = ems_pci_v1_post_irq;
		} else {
			chip->read_reg  = ems_pci_v2_read_reg;
			chip->write_reg = ems_pci_v2_write_reg;
			chip->irq_ack = ems_pci_v2_post_irq;
		}

		/* Check if channel is present */
		if (ems_pci_check_chan(dev)) {
			dev->can_sys_clock = EMS_PCI_CAN_CLOCK;
			chip->ocr = EMS_PCI_OCR | SJA_OCR_MODE_NORMAL;
			chip->cdr = EMS_PCI_CDR | SJA_CDR_CAN_MODE;

			if (card->version == 1)
				/* reset int flag of pita */
				writel(PITA2_ICR_INT0_EN | PITA2_ICR_INT0,
				       card->conf_addr + PITA2_ICR);
			else
				/* enable IRQ in PLX 9030 */
				writel(PLX_ICSR_ENA_CLR,
				       card->conf_addr + PLX_ICSR);

			/* Register SJA1000 device */
			err = rtcan_sja1000_register(dev);
			if (err) {
				dev_err(&pdev->dev, "Registering device failed "
					"(err=%d)\n", err);
				rtcan_dev_free(dev);
				goto failure_cleanup;
			}

			card->channels++;

			dev_info(&pdev->dev, "Channel #%d at 0x%p, irq %d "
				 "registered as %s\n", i + 1,
				 (void* __iomem)dev->base_addr, chip->irq_num,
				 dev->name);
		} else {
			dev_err(&pdev->dev, "Channel #%d not detected\n",
				i + 1);
			rtcan_dev_free(dev);
		}
	}

	if (!card->channels) {
		err = -ENODEV;
		goto failure_cleanup;
	}

	return 0;

failure_cleanup:
	dev_err(&pdev->dev, "Error: %d. Cleaning Up.\n", err);

	ems_pci_del_card(pdev);

	return err;
}

static struct pci_driver ems_pci_driver = {
	.name = RTCAN_DRV_NAME,
	.id_table = ems_pci_tbl,
	.probe = ems_pci_add_card,
	.remove = ems_pci_del_card,
};

static int __init ems_pci_init(void)
{
	if (!realtime_core_enabled())
		return 0;

	return pci_register_driver(&ems_pci_driver);
}

static void __exit ems_pci_exit(void)
{
	if (realtime_core_enabled())
		pci_unregister_driver(&ems_pci_driver);
}

module_init(ems_pci_init);
module_exit(ems_pci_exit);
