/*
	drivers/net/tulip/pnic2.c

	Maintained by Jeff Garzik <jgarzik@mandrakesoft.com>
	Copyright 2000,2001  The Linux Kernel Team
	Written/copyright 1994-2001 by Donald Becker.
        Modified to hep support PNIC_II by Kevin B. Hendricks

	This software may be used and distributed according to the terms
	of the GNU General Public License, incorporated herein by reference.

	Please refer to Documentation/DocBook/tulip.{pdf,ps,html}
	for more information on this driver, or visit the project
	Web page at http://sourceforge.net/projects/tulip/

*/
/* Ported to RTnet by Wittawat Yamwong <wittawat@web.de> */


/* Understanding the PNIC_II - everything is this file is based
 * on the PNIC_II_PDF datasheet which is sorely lacking in detail
 *
 * As I understand things, here are the registers and bits that
 * explain the masks and constants used in this file that are
 * either different from the 21142/3 or important for basic operation.
 *
 *
 * CSR 6  (mask = 0xfe3bd1fd of bits not to change)
 * -----
 * Bit 24    - SCR
 * Bit 23    - PCS
 * Bit 22    - TTM (Trasmit Threshold Mode)
 * Bit 18    - Port Select
 * Bit 13    - Start - 1, Stop - 0 Transmissions
 * Bit 11:10 - Loop Back Operation Mode
 * Bit 9     - Full Duplex mode (Advertise 10BaseT-FD is CSR14<7> is set)
 * Bit 1     - Start - 1, Stop - 0 Receive
 *
 *
 * CSR 14  (mask = 0xfff0ee39 of bits not to change)
 * ------
 * Bit 19    - PAUSE-Pause
 * Bit 18    - Advertise T4
 * Bit 17    - Advertise 100baseTx-FD
 * Bit 16    - Advertise 100baseTx-HD
 * Bit 12    - LTE - Link Test Enable
 * Bit 7     - ANE - Auto Negotiate Enable
 * Bit 6     - HDE - Advertise 10baseT-HD
 * Bit 2     - Reset to Power down - kept as 1 for normal operation
 * Bit 1     -  Loop Back enable for 10baseT MCC
 *
 *
 * CSR 12
 * ------
 * Bit 25    - Partner can do T4
 * Bit 24    - Partner can do 100baseTx-FD
 * Bit 23    - Partner can do 100baseTx-HD
 * Bit 22    - Partner can do 10baseT-FD
 * Bit 21    - Partner can do 10baseT-HD
 * Bit 15    - LPN is 1 if all above bits are valid other wise 0
 * Bit 14:12 - autonegotiation state (write 001 to start autonegotiate)
 * Bit 3     - Autopolarity state
 * Bit 2     - LS10B - link state of 10baseT 0 - good, 1 - failed
 * Bit 1     - LS100B - link state of 100baseT 0 - good, 1- faild
 *
 *
 * Data Port Selection Info
 *-------------------------
 *
 * CSR14<7>   CSR6<18>    CSR6<22>    CSR6<23>    CSR6<24>   MODE/PORT
 *   1           0           0 (X)       0 (X)       1        NWAY
 *   0           0           1           0 (X)       0        10baseT
 *   0           1           0           1           1 (X)    100baseT
 *
 *
 */



#include "tulip.h"
#include <linux/pci.h>
#include <linux/delay.h>


void pnic2_start_nway(/*RTnet*/struct rtnet_device *rtdev)
{
	struct tulip_private *tp = (struct tulip_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
        int csr14;
        int csr12;

        /* set up what to advertise during the negotiation */

        /* load in csr14  and mask off bits not to touch
         * comment at top of file explains mask value
         */
	csr14 = (inl(ioaddr + CSR14) & 0xfff0ee39);

        /* bit 17 - advetise 100baseTx-FD */
        if (tp->sym_advertise & 0x0100) csr14 |= 0x00020000;

        /* bit 16 - advertise 100baseTx-HD */
        if (tp->sym_advertise & 0x0080) csr14 |= 0x00010000;

        /* bit 6 - advertise 10baseT-HD */
        if (tp->sym_advertise & 0x0020) csr14 |= 0x00000040;

        /* Now set bit 12 Link Test Enable, Bit 7 Autonegotiation Enable
         * and bit 0 Don't PowerDown 10baseT
         */
        csr14 |= 0x00001184;

	if (tulip_debug > 1)
		printk(KERN_DEBUG "%s: Restarting PNIC2 autonegotiation, "
                      "csr14=%8.8x.\n", rtdev->name, csr14);

        /* tell pnic2_lnk_change we are doing an nway negotiation */
	rtdev->if_port = 0;
	tp->nway = tp->mediasense = 1;
	tp->nwayset = tp->lpar = 0;

        /* now we have to set up csr6 for NWAY state */

	tp->csr6 = inl(ioaddr + CSR6);
	if (tulip_debug > 1)
		printk(KERN_DEBUG "%s: On Entry to Nway, "
                      "csr6=%8.8x.\n", rtdev->name, tp->csr6);

        /* mask off any bits not to touch
         * comment at top of file explains mask value
         */
	tp->csr6 = tp->csr6 & 0xfe3bd1fd;

        /* don't forget that bit 9 is also used for advertising */
        /* advertise 10baseT-FD for the negotiation (bit 9) */
        if (tp->sym_advertise & 0x0040) tp->csr6 |= 0x00000200;

        /* set bit 24 for nway negotiation mode ...
         * see Data Port Selection comment at top of file
         * and "Stop" - reset both Transmit (bit 13) and Receive (bit 1)
         */
        tp->csr6 |= 0x01000000;
	outl(csr14, ioaddr + CSR14);
	outl(tp->csr6, ioaddr + CSR6);
        udelay(100);

        /* all set up so now force the negotiation to begin */

        /* read in current values and mask off all but the
	 * Autonegotiation bits 14:12.  Writing a 001 to those bits
         * should start the autonegotiation
         */
        csr12 = (inl(ioaddr + CSR12) & 0xffff8fff);
        csr12 |= 0x1000;
	outl(csr12, ioaddr + CSR12);
}


