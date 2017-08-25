/*
 * Copyright (C) 2006-2007 Jan Kiszka <jan.kiszka@web.de>.
 * Copyright (C) 2011 Stefan Kisdaroczi <kisda@hispeed.ch>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#if defined(CONFIG_XENO_DRIVERS_16550A_PCI)

#include <linux/pci.h>

struct rt_16550_pci_board {
	char *name;
	resource_size_t resource_base_addr;
	unsigned int nports;
	unsigned int port_ofs;
	unsigned long irqtype;
	unsigned int baud_base;
	int tx_fifo;
};

#if defined(CONFIG_XENO_DRIVERS_16550A_PCI_MOXA)

#define PCI_DEVICE_ID_CP112UL	0x1120
#define PCI_DEVICE_ID_CP114UL	0x1143
#define PCI_DEVICE_ID_CP138U	0x1380

static const struct rt_16550_pci_board rt_16550_moxa_c104 = {
	.name = "Moxa C104H/PCI",
	.resource_base_addr = 2,
	.nports = 4,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_c168 = {
	.name = "Moxa C168H/PCI",
	.resource_base_addr = 2,
	.nports = 8,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp114 = {
	.name = "Moxa CP-114",
	.resource_base_addr = 2,
	.nports = 4,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp132 = {
	.name = "Moxa CP-132",
	.resource_base_addr = 2,
	.nports = 2,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp102u = {
	.name = "Moxa CP-102U",
	.resource_base_addr = 2,
	.nports = 2,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp102ul = {
	.name = "Moxa CP-102UL",
	.resource_base_addr = 2,
	.nports = 2,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp104u = {
	.name = "Moxa CP-104U",
	.resource_base_addr = 2,
	.nports = 4,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp112ul = {
	.name = "Moxa CP-112UL",
	.resource_base_addr = 2,
	.nports = 2,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp114ul = {
	.name = "Moxa CP-114UL",
	.resource_base_addr = 2,
	.nports = 4,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp118u = {
	.name = "Moxa CP-118U",
	.resource_base_addr = 2,
	.nports = 8,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp132u = {
	.name = "Moxa CP-132U",
	.resource_base_addr = 2,
	.nports = 2,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp134u = {
	.name = "Moxa CP-134U",
	.resource_base_addr = 2,
	.nports = 4,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp138u = {
	.name = "Moxa CP-138U",
	.resource_base_addr = 2,
	.nports = 8,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};

static const struct rt_16550_pci_board rt_16550_moxa_cp168u = {
	.name = "Moxa CP-168U",
	.resource_base_addr = 2,
	.nports = 8,
	.port_ofs = 8,
	.baud_base = 921600,
	.tx_fifo = 16,
	.irqtype = RTDM_IRQTYPE_SHARED,
};
#endif

const struct pci_device_id rt_16550_pci_table[] = {
#if defined(CONFIG_XENO_DRIVERS_16550A_PCI_MOXA)
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_C104),
	 .driver_data = (unsigned long)&rt_16550_moxa_c104},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_C168),
	 .driver_data = (unsigned long)&rt_16550_moxa_c168},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP114),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp114},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP132),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp132},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP102U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp102u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP102UL),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp102ul},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP104U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp104u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_CP112UL),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp112ul},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_CP114UL),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp114ul},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP118U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp118u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP132U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp132u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP134U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp134u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_CP138U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp138u},
	{PCI_VDEVICE(MOXA, PCI_DEVICE_ID_MOXA_CP168U),
	 .driver_data = (unsigned long)&rt_16550_moxa_cp168u},
#endif
	{ }
};

static int rt_16550_pci_probe(struct pci_dev *pdev,
			      const struct pci_device_id *ent)
{
	struct rt_16550_pci_board *board;
	int err;
	int i;
	int port = 0;
	int base_addr;
	int max_devices = 0;

	if (!ent->driver_data)
		return -ENODEV;

	board = (struct rt_16550_pci_board *)ent->driver_data;

	for (i = 0; i < MAX_DEVICES; i++)
		if (!rt_16550_addr_param(i))
			max_devices++;

	if (board->nports > max_devices)
		return -ENODEV;

	if ((err = pci_enable_device(pdev)))
		return err;

	base_addr = pci_resource_start(pdev, board->resource_base_addr);

	for (i = 0; i < MAX_DEVICES; i++) {
		if ((port < board->nports) && (!rt_16550_addr_param(i))) {
			io[i] = base_addr + port * board->port_ofs;
			irq[i] = pdev->irq;
			irqtype[i] = board->irqtype;
			baud_base[i] = board->baud_base;
			tx_fifo[i] = board->tx_fifo;
			port++;
		}
	}

	return 0;
}

static void rt_16550_pci_remove(struct pci_dev *pdev) {
	pci_disable_device( pdev );
};

static struct pci_driver rt_16550_pci_driver = {
	.name     = RT_16550_DRIVER_NAME,
	.id_table = rt_16550_pci_table,
	.probe    = rt_16550_pci_probe,
	.remove   = rt_16550_pci_remove
};

static int pci_registered;

static inline void rt_16550_pci_init(void)
{
	if (pci_register_driver(&rt_16550_pci_driver) == 0)
		pci_registered = 1;
}

static inline void rt_16550_pci_cleanup(void)
{
	if (pci_registered)
		pci_unregister_driver(&rt_16550_pci_driver);
}

#else /* Linux < 2.6.0 || !CONFIG_PCI || !(..._16550A_PCI */

#define rt_16550_pci_init()	do { } while (0)
#define rt_16550_pci_cleanup()	do { } while (0)

#endif /* Linux < 2.6.0 || !CONFIG_PCI || !(..._16550A_PCI */
