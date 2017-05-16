/*
	drivers/net/tulip/21142.c

	Maintained by Jeff Garzik <jgarzik@mandrakesoft.com>
	Copyright 2000,2001  The Linux Kernel Team
	Written/copyright 1994-2001 by Donald Becker.

	This software may be used and distributed according to the terms
	of the GNU General Public License, incorporated herein by reference.

	Please refer to Documentation/DocBook/tulip.{pdf,ps,html}
	for more information on this driver, or visit the project
	Web page at http://sourceforge.net/projects/tulip/

*/
/* Ported to RTnet by Wittawat Yamwong <wittawat@web.de> */

#include "tulip.h"
#include <linux/pci.h>
#include <linux/delay.h>

u16 t21142_csr14[] = { 0xFFFF, 0x0705, 0x0705, 0x0000, 0x7F3D, };


void t21142_start_nway(/*RTnet*/struct rtnet_device *rtdev)
{
	struct tulip_private *tp = (struct tulip_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	int csr14 = ((tp->sym_advertise & 0x0780) << 9)  |
		((tp->sym_advertise & 0x0020) << 1) | 0xffbf;

	rtdev->if_port = 0;
	tp->nway = tp->mediasense = 1;
	tp->nwayset = tp->lpar = 0;
	if (tulip_debug > 1)
		printk(KERN_DEBUG "%s: Restarting 21143 autonegotiation, csr14=%8.8x.\n",
			   rtdev->name, csr14);
	outl(0x0001, ioaddr + CSR13);
	udelay(100);
	outl(csr14, ioaddr + CSR14);
	tp->csr6 = 0x82420000 | (tp->sym_advertise & 0x0040 ? FullDuplex : 0);
	outl(tp->csr6, ioaddr + CSR6);
	if (tp->mtable  &&  tp->mtable->csr15dir) {
		outl(tp->mtable->csr15dir, ioaddr + CSR15);
		outl(tp->mtable->csr15val, ioaddr + CSR15);
	} else
		outw(0x0008, ioaddr + CSR15);
	outl(0x1301, ioaddr + CSR12); 		/* Trigger NWAY. */
}


