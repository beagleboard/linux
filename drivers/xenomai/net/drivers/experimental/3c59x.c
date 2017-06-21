#warning  *********************************************************************
#warning  This driver is probably not real-time safe! Under certain conditions
#warning  it can cause interrupt locks of up to 1 second (issue_and_wait). We
#warning  need a rewrite of critical parts, but we are lacking the knowledge
#warning  about the hardware details (e.g. how long does a normal delay take =>
#warning  apply this value and throw an error message on timeouts).
#warning  *********************************************************************

/* EtherLinkXL.c: A 3Com EtherLink PCI III/XL ethernet driver for linux / RTnet. */
/*
  RTnet porting 2002 by Mathias Koehrer (mathias_koehrer@yahoo.de)
  -- Support only for PCI boards, EISA stuff ignored...

  Originally written 1996-1999 by Donald Becker.

  This software may be used and distributed according to the terms
  of the GNU General Public License, incorporated herein by reference.

  This driver is for the 3Com "Vortex" and "Boomerang" series ethercards.
  Members of the series include Fast EtherLink 3c590/3c592/3c595/3c597
  and the EtherLink XL 3c900 and 3c905 cards.

  Problem reports and questions should be directed to
  vortex@scyld.com

  The author may be reached as becker@scyld.com, or C/O
  Scyld Computing Corporation
  410 Severn Ave., Suite 210
  Annapolis MD 21403

  Linux Kernel Additions:

  0.99H+lk0.9 - David S. Miller - softnet, PCI DMA updates
  0.99H+lk1.0 - Jeff Garzik <jgarzik@mandrakesoft.com>
  Remove compatibility defines for kernel versions < 2.2.x.
  Update for new 2.3.x module interface
  LK1.1.2 (March 19, 2000)
  * New PCI interface (jgarzik)

  LK1.1.3 25 April 2000, Andrew Morton <andrewm@uow.edu.au>
  - Merged with 3c575_cb.c
  - Don't set RxComplete in boomerang interrupt enable reg
  - spinlock in vortex_timer to protect mdio functions
  - disable local interrupts around call to vortex_interrupt in
  vortex_tx_timeout() (So vortex_interrupt can use spin_lock())
  - Select window 3 in vortex_timer()'s write to Wn3_MAC_Ctrl
  - In vortex_start_xmit(), move the lock to _after_ we've altered
  vp->cur_tx and vp->tx_full.  This defeats the race between
  vortex_start_xmit() and vortex_interrupt which was identified
  by Bogdan Costescu.
  - Merged back support for six new cards from various sources
  - Set vortex_have_pci if pci_module_init returns zero (fixes cardbus
  insertion oops)
  - Tell it that 3c905C has NWAY for 100bT autoneg
  - Fix handling of SetStatusEnd in 'Too much work..' code, as
  per 2.3.99's 3c575_cb (Dave Hinds).
  - Split ISR into two for vortex & boomerang
  - Fix MOD_INC/DEC races
  - Handle resource allocation failures.
  - Fix 3CCFE575CT LED polarity
  - Make tx_interrupt_mitigation the default

  LK1.1.4 25 April 2000, Andrew Morton <andrewm@uow.edu.au>
  - Add extra TxReset to vortex_up() to fix 575_cb hotplug initialisation probs.
  - Put vortex_info_tbl into __devinitdata
  - In the vortex_error StatsFull HACK, disable stats in vp->intr_enable as well
  as in the hardware.
  - Increased the loop counter in issue_and_wait from 2,000 to 4,000.

  LK1.1.5 28 April 2000, andrewm
  - Added powerpc defines (John Daniel <jdaniel@etresoft.com> said these work...)
  - Some extra diagnostics
  - In vortex_error(), reset the Tx on maxCollisions.  Otherwise most
  chips usually get a Tx timeout.
  - Added extra_reset module parm
  - Replaced some inline timer manip with mod_timer
  (Franois romieu <Francois.Romieu@nic.fr>)
  - In vortex_up(), don't make Wn3_config initialisation dependent upon has_nway
  (this came across from 3c575_cb).

  LK1.1.6 06 Jun 2000, andrewm
  - Backed out the PPC defines.
  - Use del_timer_sync(), mod_timer().
  - Fix wrapped ulong comparison in boomerang_rx()
  - Add IS_TORNADO, use it to suppress 3c905C checksum error msg
  (Donald Becker, I Lee Hetherington <ilh@sls.lcs.mit.edu>)
  - Replace union wn3_config with BFINS/BFEXT manipulation for
  sparc64 (Pete Zaitcev, Peter Jones)
  - In vortex_error, do_tx_reset and vortex_tx_timeout(Vortex):
  do a netif_wake_queue() to better recover from errors. (Anders Pedersen,
  Donald Becker)
  - Print a warning on out-of-memory (rate limited to 1 per 10 secs)
  - Added two more Cardbus 575 NICs: 5b57 and 6564 (Paul Wagland)

  LK1.1.7 2 Jul 2000 andrewm
  - Better handling of shared IRQs
  - Reset the transmitter on a Tx reclaim error
  - Fixed crash under OOM during vortex_open() (Mark Hemment)
  - Fix Rx cessation problem during OOM (help from Mark Hemment)
  - The spinlocks around the mdio access were blocking interrupts for 300uS.
  Fix all this to use spin_lock_bh() within mdio_read/write
  - Only write to TxFreeThreshold if it's a boomerang - other NICs don't
  have one.
  - Added 802.3x MAC-layer flow control support

  LK1.1.8 13 Aug 2000 andrewm
  - Ignore request_region() return value - already reserved if Cardbus.
  - Merged some additional Cardbus flags from Don's 0.99Qk
  - Some fixes for 3c556 (Fred Maciel)
  - Fix for EISA initialisation (Jan Rekorajski)
  - Renamed MII_XCVR_PWR and EEPROM_230 to align with 3c575_cb and D. Becker's drivers
  - Fixed MII_XCVR_PWR for 3CCFE575CT
  - Added INVERT_LED_PWR, used it.
  - Backed out the extra_reset stuff

  LK1.1.9 12 Sep 2000 andrewm
  - Backed out the tx_reset_resume flags.  It was a no-op.
  - In vortex_error, don't reset the Tx on txReclaim errors
  - In vortex_error, don't reset the Tx on maxCollisions errors.
  Hence backed out all the DownListPtr logic here.
  - In vortex_error, give Tornado cards a partial TxReset on
  maxCollisions (David Hinds).	Defined MAX_COLLISION_RESET for this.
  - Redid some driver flags and device names based on pcmcia_cs-3.1.20.
  - Fixed a bug where, if vp->tx_full is set when the interface
  is downed, it remains set when the interface is upped.  Bad
  things happen.

  LK1.1.10 17 Sep 2000 andrewm
  - Added EEPROM_8BIT for 3c555 (Fred Maciel)
  - Added experimental support for the 3c556B Laptop Hurricane (Louis Gerbarg)
  - Add HAS_NWAY to "3c900 Cyclone 10Mbps TPO"

  LK1.1.11 13 Nov 2000 andrewm
  - Dump MOD_INC/DEC_USE_COUNT, use SET_MODULE_OWNER

  LK1.1.12 1 Jan 2001 andrewm (2.4.0-pre1)
  - Call pci_enable_device before we request our IRQ (Tobias Ringstrom)
  - Add 3c590 PCI latency timer hack to vortex_probe1 (from 0.99Ra)
  - Added extended issue_and_wait for the 3c905CX.
  - Look for an MII on PHY index 24 first (3c905CX oddity).
  - Add HAS_NWAY to 3cSOHO100-TX (Brett Frankenberger)
  - Don't free skbs we don't own on oom path in vortex_open().

  LK1.1.13 27 Jan 2001
  - Added explicit `medialock' flag so we can truly
  lock the media type down with `options'.
  - "check ioremap return and some tidbits" (Arnaldo Carvalho de Melo <acme@conectiva.com.br>)
  - Added and used EEPROM_NORESET for 3c556B PM resumes.
  - Fixed leakage of vp->rx_ring.
  - Break out separate HAS_HWCKSM device capability flag.
  - Kill vp->tx_full (ANK)
  - Merge zerocopy fragment handling (ANK?)

  LK1.1.14 15 Feb 2001
  - Enable WOL.  Can be turned on with `enable_wol' module option.
  - EISA and PCI initialisation fixes (jgarzik, Manfred Spraul)
  - If a device's internalconfig register reports it has NWAY,
  use it, even if autoselect is enabled.

  LK1.1.15 6 June 2001 akpm
  - Prevent double counting of received bytes (Lars Christensen)
  - Add ethtool support (jgarzik)
  - Add module parm descriptions (Andrzej M. Krzysztofowicz)
  - Implemented alloc_etherdev() API
  - Special-case the 'Tx error 82' message.

  LK1.1.16 18 July 2001 akpm
  - Make NETIF_F_SG dependent upon nr_free_highpages(), not on CONFIG_HIGHMEM
  - Lessen verbosity of bootup messages
  - Fix WOL - use new PM API functions.
  - Use netif_running() instead of vp->open in suspend/resume.
  - Don't reset the interface logic on open/close/rmmod.  It upsets
  autonegotiation, and hence DHCP (from 0.99T).
  - Back out EEPROM_NORESET flag because of the above (we do it for all
  NICs).
  - Correct 3c982 identification string
  - Rename wait_for_completion() to issue_and_wait() to avoid completion.h
  clash.

  - See http://www.uow.edu.au/~andrewm/linux/#3c59x-2.3 for more details.
  - Also see Documentation/networking/vortex.txt
*/

/*
 * FIXME: This driver _could_ support MTU changing, but doesn't.  See Don's hamachi.c implementation
 * as well as other drivers
 *
 * NOTE: If you make 'vortex_debug' a constant (#define vortex_debug 0) the driver shrinks by 2k
 * due to dead code elimination.  There will be some performance benefits from this due to
 * elimination of all the tests and reduced cache footprint.
 */


#define DRV_NAME	"3c59x"
#define DRV_VERSION	"LK1.1.16"
#define DRV_RELDATE	"19 July 2001"



/* A few values that may be tweaked. */
/* Keep the ring sizes a power of two for efficiency. */
#define TX_RING_SIZE	16
#define RX_RING_SIZE	8 /*** RTnet ***/
#define PKT_BUF_SZ		1536			/* Size of each temporary Rx buffer.*/

/* "Knobs" that adjust features and parameters. */
/* Set the copy breakpoint for the copy-only-tiny-frames scheme.
   Setting to > 1512 effectively disables this feature. */
/*** RTnet ***/
/*** RTnet ***/
/* Allow setting MTU to a larger size, bypassing the normal ethernet setup. */
static const int mtu = 1500;
/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static int max_interrupt_work = 32;
/* Tx timeout interval (millisecs) */
// *** RTnet ***
//static int watchdog = 5000;
// *** RTnet ***

/* Allow aggregation of Tx interrupts.	Saves CPU load at the cost
 * of possible Tx stalls if the system is blocking interrupts
 * somewhere else.  Undefine this to disable.
 */
#define tx_interrupt_mitigation 1

/* Put out somewhat more debugging messages. (0: no msg, 1 minimal .. 6). */
#define vortex_debug debug
#ifdef VORTEX_DEBUG
static int vortex_debug = VORTEX_DEBUG;
#else
static int vortex_debug = 1;
#endif

#ifndef __OPTIMIZE__
#error You must compile this file with the correct options!
#error See the last lines of the source file.
#error You must compile this driver with "-O".
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/mii.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <asm/irq.h>			/* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>

// *** RTnet ***
#include <rtnet_port.h>

static int cards = INT_MAX;
module_param(cards, int, 0444);
MODULE_PARM_DESC(cards, "number of cards to be supported");
// *** RTnet ***

/* Kernel compatibility defines, some common to David Hinds' PCMCIA package.
   This is only in the support-all-kernels source code. */

#define RUN_AT(x) (jiffies + (x))

#include <linux/delay.h>

// *** RTnet - no power management ***
#undef pci_set_power_state
#define pci_set_power_state null_set_power_state
static inline int null_set_power_state(struct pci_dev *dev, int state)
{
	return 0;
}
// *** RTnet ***


static char version[] =
	DRV_NAME " for RTnet : Donald Becker and others. www.scyld.com/network/vortex.html\n";

MODULE_AUTHOR("Donald Becker <becker@scyld.com>");
MODULE_DESCRIPTION("3Com 3c59x/3c9xx ethernet driver for RTnet "
		DRV_VERSION " " DRV_RELDATE);
MODULE_LICENSE("GPL");

/* Operational parameter that usually are not changed. */

/* The Vortex size is twice that of the original EtherLinkIII series: the
   runtime register window, window 1, is now always mapped in.
   The Boomerang size is twice as large as the Vortex -- it has additional
   bus master control registers. */
#define VORTEX_TOTAL_SIZE 0x20
#define BOOMERANG_TOTAL_SIZE 0x40

/* Set iff a MII transceiver on any interface requires mdio preamble.
   This only set with the original DP83840 on older 3c905 boards, so the extra
   code size of a per-interface flag is not worthwhile. */
static char mii_preamble_required;

#define PFX DRV_NAME ": "



/*
  Theory of Operation

  I. Board Compatibility

  This device driver is designed for the 3Com FastEtherLink and FastEtherLink
  XL, 3Com's PCI to 10/100baseT adapters.  It also works with the 10Mbs
  versions of the FastEtherLink cards.	The supported product IDs are
  3c590, 3c592, 3c595, 3c597, 3c900, 3c905

  The related ISA 3c515 is supported with a separate driver, 3c515.c, included
  with the kernel source or available from
  cesdis.gsfc.nasa.gov:/pub/linux/drivers/3c515.html

  II. Board-specific settings

  PCI bus devices are configured by the system at boot time, so no jumpers
  need to be set on the board.	The system BIOS should be set to assign the
  PCI INTA signal to an otherwise unused system IRQ line.

  The EEPROM settings for media type and forced-full-duplex are observed.
  The EEPROM media type should be left at the default "autoselect" unless using
  10base2 or AUI connections which cannot be reliably detected.

  III. Driver operation

  The 3c59x series use an interface that's very similar to the previous 3c5x9
  series.  The primary interface is two programmed-I/O FIFOs, with an
  alternate single-contiguous-region bus-master transfer (see next).

  The 3c900 "Boomerang" series uses a full-bus-master interface with separate
  lists of transmit and receive descriptors, similar to the AMD LANCE/PCnet,
  DEC Tulip and Intel Speedo3.	The first chip version retains a compatible
  programmed-I/O interface that has been removed in 'B' and subsequent board
  revisions.

  One extension that is advertised in a very large font is that the adapters
  are capable of being bus masters.  On the Vortex chip this capability was
  only for a single contiguous region making it far less useful than the full
  bus master capability.  There is a significant performance impact of taking
  an extra interrupt or polling for the completion of each transfer, as well
  as difficulty sharing the single transfer engine between the transmit and
  receive threads.  Using DMA transfers is a win only with large blocks or
  with the flawed versions of the Intel Orion motherboard PCI controller.

  The Boomerang chip's full-bus-master interface is useful, and has the
  currently-unused advantages over other similar chips that queued transmit
  packets may be reordered and receive buffer groups are associated with a
  single frame.

  With full-bus-master support, this driver uses a "RX_COPYBREAK" scheme.
  Rather than a fixed intermediate receive buffer, this scheme allocates
  full-sized skbuffs as receive buffers.  The value RX_COPYBREAK is used as
  the copying breakpoint: it is chosen to trade-off the memory wasted by
  passing the full-sized skbuff to the queue layer for all frames vs. the
  copying cost of copying a frame to a correctly-sized skbuff.

  IIIC. Synchronization
  The driver runs as two independent, single-threaded flows of control.  One
  is the send-packet routine, which enforces single-threaded use by the
  dev->tbusy flag.  The other thread is the interrupt handler, which is single
  threaded by the hardware and other software.

  IV. Notes

  Thanks to Cameron Spitzer and Terry Murphy of 3Com for providing development
  3c590, 3c595, and 3c900 boards.
  The name "Vortex" is the internal 3Com project name for the PCI ASIC, and
  the EISA version is called "Demon".  According to Terry these names come
  from rides at the local amusement park.

  The new chips support both ethernet (1.5K) and FDDI (4.5K) packet sizes!
  This driver only supports ethernet packets because of the skbuff allocation
  limit of 4K.
*/

/* This table drives the PCI probe routines.  It's mostly boilerplate in all
   of the drivers, and will likely be provided by some future kernel.
*/
enum pci_flags_bit {
	PCI_USES_IO=1, PCI_USES_MEM=2, PCI_USES_MASTER=4,
	PCI_ADDR0=0x10<<0, PCI_ADDR1=0x10<<1, PCI_ADDR2=0x10<<2, PCI_ADDR3=0x10<<3,
};

enum {	IS_VORTEX=1, IS_BOOMERANG=2, IS_CYCLONE=4, IS_TORNADO=8,
	EEPROM_8BIT=0x10,	/* AKPM: Uses 0x230 as the base bitmaps for EEPROM reads */
	HAS_PWR_CTRL=0x20, HAS_MII=0x40, HAS_NWAY=0x80, HAS_CB_FNS=0x100,
	INVERT_MII_PWR=0x200, INVERT_LED_PWR=0x400, MAX_COLLISION_RESET=0x800,
	EEPROM_OFFSET=0x1000, HAS_HWCKSM=0x2000 };

enum vortex_chips {
	CH_3C590 = 0,
	CH_3C592,
	CH_3C597,
	CH_3C595_1,
	CH_3C595_2,

	CH_3C595_3,
	CH_3C900_1,
	CH_3C900_2,
	CH_3C900_3,
	CH_3C900_4,

	CH_3C900_5,
	CH_3C900B_FL,
	CH_3C905_1,
	CH_3C905_2,
	CH_3C905B_1,

	CH_3C905B_2,
	CH_3C905B_FX,
	CH_3C905C,
	CH_3C980,
	CH_3C9805,

	CH_3CSOHO100_TX,
	CH_3C555,
	CH_3C556,
	CH_3C556B,
	CH_3C575,

	CH_3C575_1,
	CH_3CCFE575,
	CH_3CCFE575CT,
	CH_3CCFE656,
	CH_3CCFEM656,

	CH_3CCFEM656_1,
	CH_3C450,
};


/* note: this array directly indexed by above enums, and MUST
 * be kept in sync with both the enums above, and the PCI device
 * table below
 */
static struct vortex_chip_info {
	const char *name;
	int flags;
	int drv_flags;
	int io_size;
} vortex_info_tbl[] = {
#define EISA_TBL_OFFSET	0		/* Offset of this entry for vortex_eisa_init */
	{"3c590 Vortex 10Mbps",
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },
	{"3c592 EISA 10Mbps Demon/Vortex",					/* AKPM: from Don's 3c59x_cb.c 0.49H */
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },
	{"3c597 EISA Fast Demon/Vortex",					/* AKPM: from Don's 3c59x_cb.c 0.49H */
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },
	{"3c595 Vortex 100baseTx",
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },
	{"3c595 Vortex 100baseT4",
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },

	{"3c595 Vortex 100base-MII",
	 PCI_USES_IO|PCI_USES_MASTER, IS_VORTEX, 32, },
	{"3c900 Boomerang 10baseT",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG, 64, },
	{"3c900 Boomerang 10Mbps Combo",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG, 64, },
	{"3c900 Cyclone 10Mbps TPO",						/* AKPM: from Don's 0.99M */
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_HWCKSM, 128, },
	{"3c900 Cyclone 10Mbps Combo",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },

	{"3c900 Cyclone 10Mbps TPC",						/* AKPM: from Don's 0.99M */
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },
	{"3c900B-FL Cyclone 10base-FL",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },
	{"3c905 Boomerang 100baseTx",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG|HAS_MII, 64, },
	{"3c905 Boomerang 100baseT4",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG|HAS_MII, 64, },
	{"3c905B Cyclone 100baseTx",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_HWCKSM, 128, },

	{"3c905B Cyclone 10/100/BNC",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_HWCKSM, 128, },
	{"3c905B-FX Cyclone 100baseFx",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },
	{"3c905C Tornado",
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|HAS_HWCKSM, 128, },
	{"3c980 Cyclone",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },
	{"3c982 Dual Port Server Cyclone",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_HWCKSM, 128, },

	{"3cSOHO100-TX Hurricane",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_HWCKSM, 128, },
	{"3c555 Laptop Hurricane",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|EEPROM_8BIT|HAS_HWCKSM, 128, },
	{"3c556 Laptop Tornado",
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|EEPROM_8BIT|HAS_CB_FNS|INVERT_MII_PWR|
	 HAS_HWCKSM, 128, },
	{"3c556B Laptop Hurricane",
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|EEPROM_OFFSET|HAS_CB_FNS|INVERT_MII_PWR|
	 HAS_HWCKSM, 128, },
	{"3c575 [Megahertz] 10/100 LAN	CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG|HAS_MII|EEPROM_8BIT, 128, },

	{"3c575 Boomerang CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_BOOMERANG|HAS_MII|EEPROM_8BIT, 128, },
	{"3CCFE575BT Cyclone CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_CB_FNS|EEPROM_8BIT|
	 INVERT_LED_PWR|HAS_HWCKSM, 128, },
	{"3CCFE575CT Tornado CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|HAS_CB_FNS|EEPROM_8BIT|INVERT_MII_PWR|
	 MAX_COLLISION_RESET|HAS_HWCKSM, 128, },
	{"3CCFE656 Cyclone CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_CB_FNS|EEPROM_8BIT|INVERT_MII_PWR|
	 INVERT_LED_PWR|HAS_HWCKSM, 128, },
	{"3CCFEM656B Cyclone+Winmodem CardBus",
	 PCI_USES_IO|PCI_USES_MASTER, IS_CYCLONE|HAS_NWAY|HAS_CB_FNS|EEPROM_8BIT|INVERT_MII_PWR|
	 INVERT_LED_PWR|HAS_HWCKSM, 128, },

	{"3CXFEM656C Tornado+Winmodem CardBus",			/* From pcmcia-cs-3.1.5 */
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|HAS_CB_FNS|EEPROM_8BIT|INVERT_MII_PWR|
	 MAX_COLLISION_RESET|HAS_HWCKSM, 128, },
	{"3c450 HomePNA Tornado",						/* AKPM: from Don's 0.99Q */
	 PCI_USES_IO|PCI_USES_MASTER, IS_TORNADO|HAS_NWAY|HAS_HWCKSM, 128, },
	{0,}, /* 0 terminated list. */
};


static struct pci_device_id vortex_pci_tbl[] = {
	{ 0x10B7, 0x5900, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C590 },
	{ 0x10B7, 0x5920, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C592 },
	{ 0x10B7, 0x5970, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C597 },
	{ 0x10B7, 0x5950, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C595_1 },
	{ 0x10B7, 0x5951, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C595_2 },

	{ 0x10B7, 0x5952, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C595_3 },
	{ 0x10B7, 0x9000, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900_1 },
	{ 0x10B7, 0x9001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900_2 },
	{ 0x10B7, 0x9004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900_3 },
	{ 0x10B7, 0x9005, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900_4 },

	{ 0x10B7, 0x9006, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900_5 },
	{ 0x10B7, 0x900A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C900B_FL },
	{ 0x10B7, 0x9050, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905_1 },
	{ 0x10B7, 0x9051, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905_2 },
	{ 0x10B7, 0x9055, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905B_1 },

	{ 0x10B7, 0x9058, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905B_2 },
	{ 0x10B7, 0x905A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905B_FX },
	{ 0x10B7, 0x9200, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C905C },
	{ 0x10B7, 0x9800, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C980 },
	{ 0x10B7, 0x9805, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C9805 },

	{ 0x10B7, 0x7646, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CSOHO100_TX },
	{ 0x10B7, 0x5055, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C555 },
	{ 0x10B7, 0x6055, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C556 },
	{ 0x10B7, 0x6056, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C556B },
	{ 0x10B7, 0x5b57, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C575 },

	{ 0x10B7, 0x5057, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C575_1 },
	{ 0x10B7, 0x5157, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CCFE575 },
	{ 0x10B7, 0x5257, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CCFE575CT },
	{ 0x10B7, 0x6560, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CCFE656 },
	{ 0x10B7, 0x6562, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CCFEM656 },

	{ 0x10B7, 0x6564, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3CCFEM656_1 },
	{ 0x10B7, 0x4500, PCI_ANY_ID, PCI_ANY_ID, 0, 0, CH_3C450 },
	{0,}						/* 0 terminated list. */
};
MODULE_DEVICE_TABLE(pci, vortex_pci_tbl);


/* Operational definitions.
   These are not used by other compilation units and thus are not
   exported in a ".h" file.

   First the windows.  There are eight register windows, with the command
   and status registers available in each.
*/
#define EL3WINDOW(win_num) outw(SelectWindow + (win_num), ioaddr + EL3_CMD)
#define EL3_CMD 0x0e
#define EL3_STATUS 0x0e

/* The top five bits written to EL3_CMD are a command, the lower
   11 bits are the parameter, if applicable.
   Note that 11 parameters bits was fine for ethernet, but the new chip
   can handle FDDI length frames (~4500 octets) and now parameters count
   32-bit 'Dwords' rather than octets. */

enum vortex_cmd {
	TotalReset = 0<<11, SelectWindow = 1<<11, StartCoax = 2<<11,
	RxDisable = 3<<11, RxEnable = 4<<11, RxReset = 5<<11,
	UpStall = 6<<11, UpUnstall = (6<<11)+1,
	DownStall = (6<<11)+2, DownUnstall = (6<<11)+3,
	RxDiscard = 8<<11, TxEnable = 9<<11, TxDisable = 10<<11, TxReset = 11<<11,
	FakeIntr = 12<<11, AckIntr = 13<<11, SetIntrEnb = 14<<11,
	SetStatusEnb = 15<<11, SetRxFilter = 16<<11, SetRxThreshold = 17<<11,
	SetTxThreshold = 18<<11, SetTxStart = 19<<11,
	StartDMAUp = 20<<11, StartDMADown = (20<<11)+1, StatsEnable = 21<<11,
	StatsDisable = 22<<11, StopCoax = 23<<11, SetFilterBit = 25<<11,};

/* The SetRxFilter command accepts the following classes: */
enum RxFilter {
	RxStation = 1, RxMulticast = 2, RxBroadcast = 4, RxProm = 8 };

/* Bits in the general status register. */
enum vortex_status {
	IntLatch = 0x0001, HostError = 0x0002, TxComplete = 0x0004,
	TxAvailable = 0x0008, RxComplete = 0x0010, RxEarly = 0x0020,
	IntReq = 0x0040, StatsFull = 0x0080,
	DMADone = 1<<8, DownComplete = 1<<9, UpComplete = 1<<10,
	DMAInProgress = 1<<11,			/* DMA controller is still busy.*/
	CmdInProgress = 1<<12,			/* EL3_CMD is still busy.*/
};

/* Register window 1 offsets, the window used in normal operation.
   On the Vortex this window is always mapped at offsets 0x10-0x1f. */
enum Window1 {
	TX_FIFO = 0x10,  RX_FIFO = 0x10,  RxErrors = 0x14,
	RxStatus = 0x18,  Timer=0x1A, TxStatus = 0x1B,
	TxFree = 0x1C, /* Remaining free bytes in Tx buffer. */
};
enum Window0 {
	Wn0EepromCmd = 10,		/* Window 0: EEPROM command register. */
	Wn0EepromData = 12,		/* Window 0: EEPROM results register. */
	IntrStatus=0x0E,		/* Valid in all windows. */
};
enum Win0_EEPROM_bits {
	EEPROM_Read = 0x80, EEPROM_WRITE = 0x40, EEPROM_ERASE = 0xC0,
	EEPROM_EWENB = 0x30,		/* Enable erasing/writing for 10 msec. */
	EEPROM_EWDIS = 0x00,		/* Disable EWENB before 10 msec timeout. */
};
/* EEPROM locations. */
enum eeprom_offset {
	PhysAddr01=0, PhysAddr23=1, PhysAddr45=2, ModelID=3,
	EtherLink3ID=7, IFXcvrIO=8, IRQLine=9,
	NodeAddr01=10, NodeAddr23=11, NodeAddr45=12,
	DriverTune=13, Checksum=15};

enum Window2 {			/* Window 2. */
	Wn2_ResetOptions=12,
};
enum Window3 {			/* Window 3: MAC/config bits. */
	Wn3_Config=0, Wn3_MAC_Ctrl=6, Wn3_Options=8,
};

#define BFEXT(value, offset, bitcount)					\
	((((unsigned long)(value)) >> (offset)) & ((1 << (bitcount)) - 1))

#define BFINS(lhs, rhs, offset, bitcount)				\
	(((lhs) & ~((((1 << (bitcount)) - 1)) << (offset))) |		\
		(((rhs) & ((1 << (bitcount)) - 1)) << (offset)))

#define RAM_SIZE(v)		BFEXT(v, 0, 3)
#define RAM_WIDTH(v)	BFEXT(v, 3, 1)
#define RAM_SPEED(v)	BFEXT(v, 4, 2)
#define ROM_SIZE(v)		BFEXT(v, 6, 2)
#define RAM_SPLIT(v)	BFEXT(v, 16, 2)
#define XCVR(v)			BFEXT(v, 20, 4)
#define AUTOSELECT(v)	BFEXT(v, 24, 1)

enum Window4 {		/* Window 4: Xcvr/media bits. */
	Wn4_FIFODiag = 4, Wn4_NetDiag = 6, Wn4_PhysicalMgmt=8, Wn4_Media = 10,
};
enum Win4_Media_bits {
	Media_SQE = 0x0008,		/* Enable SQE error counting for AUI. */
	Media_10TP = 0x00C0,	/* Enable link beat and jabber for 10baseT. */
	Media_Lnk = 0x0080,		/* Enable just link beat for 100TX/100FX. */
	Media_LnkBeat = 0x0800,
};
enum Window7 {					/* Window 7: Bus Master control. */
	Wn7_MasterAddr = 0, Wn7_MasterLen = 6, Wn7_MasterStatus = 12,
};
/* Boomerang bus master control registers. */
enum MasterCtrl {
	PktStatus = 0x20, DownListPtr = 0x24, FragAddr = 0x28, FragLen = 0x2c,
	TxFreeThreshold = 0x2f, UpPktStatus = 0x30, UpListPtr = 0x38,
};

/* The Rx and Tx descriptor lists.
   Caution Alpha hackers: these types are 32 bits!  Note also the 8 byte
   alignment contraint on tx_ring[] and rx_ring[]. */
#define LAST_FRAG	0x80000000			/* Last Addr/Len pair in descriptor. */
#define DN_COMPLETE	0x00010000			/* This packet has been downloaded */
struct boom_rx_desc {
	u32 next;					/* Last entry points to 0.   */
	s32 status;
	u32 addr;					/* Up to 63 addr/len pairs possible. */
	s32 length;					/* Set LAST_FRAG to indicate last pair. */
};
/* Values for the Rx status entry. */
enum rx_desc_status {
	RxDComplete=0x00008000, RxDError=0x4000,
	/* See boomerang_rx() for actual error bits */
	IPChksumErr=1<<25, TCPChksumErr=1<<26, UDPChksumErr=1<<27,
	IPChksumValid=1<<29, TCPChksumValid=1<<30, UDPChksumValid=1<<31,
};

// *** RTnet ***
//#ifdef MAX_SKB_FRAGS
//#define DO_ZEROCOPY 1
//#else
#define DO_ZEROCOPY 0
//#endif

struct boom_tx_desc {
	u32 next;					/* Last entry points to 0.   */
	s32 status;					/* bits 0:12 length, others see below.	*/
#if DO_ZEROCOPY
	struct {
		u32 addr;
		s32 length;
	} frag[1+MAX_SKB_FRAGS];
#else
	u32 addr;
	s32 length;
#endif
};

/* Values for the Tx status entry. */
enum tx_desc_status {
	CRCDisable=0x2000, TxDComplete=0x8000,
	AddIPChksum=0x02000000, AddTCPChksum=0x04000000, AddUDPChksum=0x08000000,
	TxIntrUploaded=0x80000000,		/* IRQ when in FIFO, but maybe not sent. */
};

/* Chip features we care about in vp->capabilities, read from the EEPROM. */
enum ChipCaps { CapBusMaster=0x20, CapPwrMgmt=0x2000 };

struct vortex_private {
	/* The Rx and Tx rings should be quad-word-aligned. */
	struct boom_rx_desc* rx_ring;
	struct boom_tx_desc* tx_ring;
	dma_addr_t rx_ring_dma;
	dma_addr_t tx_ring_dma;
	/* The addresses of transmit- and receive-in-place skbuffs. */

	// *** RTnet ***
	struct rtskb *tx_skbuff[TX_RING_SIZE];
	struct rtskb *rx_skbuff[RX_RING_SIZE];
	// *** RTnet ***

	struct rtnet_device *next_module;		/* NULL if PCI device */
	unsigned int cur_rx, cur_tx;		/* The next free ring entry */
	unsigned int dirty_rx, dirty_tx;	/* The ring entries to be free()ed. */
	struct net_device_stats stats;
	struct rtskb *tx_skb;				/* Packet being eaten by bus master ctrl.  */
	dma_addr_t tx_skb_dma;				/* Allocated DMA address for bus master ctrl DMA.   */

	/* PCI configuration space information. */
	struct pci_dev *pdev;
	char *cb_fn_base;					/* CardBus function status addr space. */

	/* Some values here only for performance evaluation and path-coverage */
	int rx_nocopy, rx_copy, queued_packet, rx_csumhits;
	int card_idx;

	/* The remainder are related to chip state, mostly media selection. */
	struct timer_list timer;			/* Media selection timer. */
	struct timer_list rx_oom_timer;		/* Rx skb allocation retry timer */
	int options;						/* User-settable misc. driver options. */
	unsigned int media_override:4,		/* Passed-in media type. */
		default_media:4,				/* Read from the EEPROM/Wn3_Config. */
		full_duplex:1, force_fd:1, autoselect:1,
		bus_master:1,					/* Vortex can only do a fragment bus-m. */
		full_bus_master_tx:1, full_bus_master_rx:2, /* Boomerang  */
		flow_ctrl:1,					/* Use 802.3x flow control (PAUSE only) */
		partner_flow_ctrl:1,			/* Partner supports flow control */
		has_nway:1,
		enable_wol:1,					/* Wake-on-LAN is enabled */
		pm_state_valid:1,				/* power_state[] has sane contents */
		open:1,
		medialock:1,
		must_free_region:1;				/* Flag: if zero, Cardbus owns the I/O region */
	int drv_flags;
	u16 status_enable;
	u16 intr_enable;
	u16 available_media;				/* From Wn3_Options. */
	u16 capabilities, info1, info2;		/* Various, from EEPROM. */
	u16 advertising;					/* NWay media advertisement */
	unsigned char phys[2];				/* MII device addresses. */
	u16 deferred;						/* Resend these interrupts when we
								 * bale from the ISR */
	u16 io_size;						/* Size of PCI region (for release_region) */
	rtdm_lock_t lock;					/* Serialise access to device & its vortex_private */
	spinlock_t mdio_lock;				/* Serialise access to mdio hardware */
	u32 power_state[16];
	rtdm_irq_t irq_handle;
};

/* The action to take with a media selection timer tick.
   Note that we deviate from the 3Com order by checking 10base2 before AUI.
*/
enum xcvr_types {
	XCVR_10baseT=0, XCVR_AUI, XCVR_10baseTOnly, XCVR_10base2, XCVR_100baseTx,
	XCVR_100baseFx, XCVR_MII=6, XCVR_NWAY=8, XCVR_ExtMII=9, XCVR_Default=10,
};

static struct media_table {
	char *name;
	unsigned int media_bits:16,		/* Bits to set in Wn4_Media register. */
		mask:8,						/* The transceiver-present bit in Wn3_Config.*/
		next:8;						/* The media type to try next. */
	int wait;						/* Time before we check media status. */
} media_tbl[] = {
	{	"10baseT",   Media_10TP,0x08, XCVR_10base2, (14*HZ)/10},
	{ "10Mbs AUI", Media_SQE, 0x20, XCVR_Default, (1*HZ)/10},
	{ "undefined", 0,			0x80, XCVR_10baseT, 10000},
	{ "10base2",   0,			0x10, XCVR_AUI,		(1*HZ)/10},
	{ "100baseTX", Media_Lnk, 0x02, XCVR_100baseFx, (14*HZ)/10},
	{ "100baseFX", Media_Lnk, 0x04, XCVR_MII,		(14*HZ)/10},
	{ "MII",               0,			0x41, XCVR_10baseT, 3*HZ },
	{ "undefined", 0,			0x01, XCVR_10baseT, 10000},
	{ "Autonegotiate", 0,		0x41, XCVR_10baseT, 3*HZ},
	{ "MII-External",      0,		0x41, XCVR_10baseT, 3*HZ },
	{ "Default",   0,			0xFF, XCVR_10baseT, 10000},
};

static int vortex_probe1(struct pci_dev *pdev, long ioaddr, int irq,
			int chip_idx, int card_idx);
static void vortex_up(struct rtnet_device *rtdev);
static void vortex_down(struct rtnet_device *rtdev);
static int vortex_open(struct rtnet_device *rtdev);
static void mdio_sync(long ioaddr, int bits);
static int mdio_read(struct rtnet_device *rtdev, int phy_id, int location);
static void mdio_write(struct rtnet_device *vp, int phy_id, int location, int value);

// *** RTnet ***
//static void vortex_timer(unsigned long arg);
//static void rx_oom_timer(unsigned long arg);
// *** RTnet ***

static int vortex_start_xmit(struct rtskb *skb, struct rtnet_device *rtdev);
static int boomerang_start_xmit(struct rtskb *skb, struct rtnet_device *rtdev);
static int vortex_rx(struct rtnet_device *rtdev, int *packets, nanosecs_abs_t *time_stamp);
static int boomerang_rx(struct rtnet_device *rtdev, int *packets, nanosecs_abs_t *time_stamp);
static int vortex_interrupt(rtdm_irq_t *irq_handle);
static int boomerang_interrupt(rtdm_irq_t *irq_handle);
static int vortex_close(struct rtnet_device *rtdev);
static void dump_tx_ring(struct rtnet_device *rtdev);

static void update_stats(long ioaddr, struct rtnet_device *dev);
static struct net_device_stats *vortex_get_stats(struct rtnet_device *rtdev);

static void set_rx_mode(struct rtnet_device *rtdev);

// *** RTnet ***
//static int vortex_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
//static void vortex_tx_timeout(struct net_device *dev);
// *** RTnet ***

static void acpi_set_WOL(struct rtnet_device *rtdev);

/* This driver uses 'options' to pass the media type, full-duplex flag, etc. */
/* Option count limit only -- unlimited interfaces are supported. */
#define MAX_UNITS 8
static int options[MAX_UNITS] = { -1, -1, -1, -1, -1, -1, -1, -1,};
static int full_duplex[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int hw_checksums[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int flow_ctrl[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int enable_wol[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};

module_param(debug, int, 0444);
module_param_array(options, int, NULL, 0444);
module_param_array(full_duplex, int, NULL, 0444);
module_param_array(hw_checksums, int, NULL, 0444);
module_param_array(flow_ctrl, int, NULL, 0444);
module_param_array(enable_wol, int, NULL, 0444);
/*** RTnet ***
     MODULE_PARM(rx_copybreak, "i");
     *** RTnet ***/
module_param(max_interrupt_work, int, 0444);
/*** RTnet ***
     MODULE_PARM(compaq_ioaddr, "i");
     MODULE_PARM(compaq_irq, "i");
     MODULE_PARM(compaq_device_id, "i");
     MODULE_PARM(watchdog, "i");
     *** RTnet ***/
MODULE_PARM_DESC(debug, "3c59x debug level (0-6)");
MODULE_PARM_DESC(options, "3c59x: Bits 0-3: media type, bit 4: bus mastering, bit 9: full duplex");
MODULE_PARM_DESC(full_duplex, "3c59x full duplex setting(s) (1)");
MODULE_PARM_DESC(hw_checksums, "3c59x Hardware checksum checking by adapter(s) (0-1)");
MODULE_PARM_DESC(flow_ctrl, "3c59x 802.3x flow control usage (PAUSE only) (0-1)");
MODULE_PARM_DESC(enable_wol, "3c59x: Turn on Wake-on-LAN for adapter(s) (0-1)");
/*** RTnet ***
     MODULE_PARM_DESC(rx_copybreak, "3c59x copy breakpoint for copy-only-tiny-frames");
     *** RTnet ***/
MODULE_PARM_DESC(max_interrupt_work, "3c59x maximum events handled per interrupt");
/*** RTnet ***
     MODULE_PARM_DESC(compaq_ioaddr, "3c59x PCI I/O base address (Compaq BIOS problem workaround)");
     MODULE_PARM_DESC(compaq_irq, "3c59x PCI IRQ number (Compaq BIOS problem workaround)");
     MODULE_PARM_DESC(compaq_device_id, "3c59x PCI device ID (Compaq BIOS problem workaround)");
     MODULE_PARM_DESC(watchdog, "3c59x transmit timeout in milliseconds");
     *** RTnet ***/

/* #define dev_alloc_skb dev_alloc_skb_debug */

/* A list of all installed Vortex EISA devices, for removing the driver module. */
static struct rtnet_device *root_vortex_eisa_dev;

/* Variables to work-around the Compaq PCI BIOS32 problem. */
// *** RTnet ***
//static int compaq_ioaddr, compaq_irq, compaq_device_id = 0x5900;
// *** RTnet ***

static int vortex_cards_found;

#ifdef CONFIG_PM

#endif /* CONFIG_PM */

/* returns count found (>= 0), or negative on error */

/* returns count (>= 0), or negative on error */
static int vortex_init_one (struct pci_dev *pdev,
			const struct pci_device_id *ent)
{
	int rc;

	if( vortex_cards_found >= cards )
		return -ENODEV;

	/* wake up and enable device */
	if (pci_enable_device (pdev)) {
		rc = -EIO;
	} else {
		rc = vortex_probe1 (pdev, pci_resource_start (pdev, 0), pdev->irq,
				ent->driver_data, vortex_cards_found);
		if (rc == 0)
			vortex_cards_found++;
	}
	return rc;
}

/*
 * Start up the PCI device which is described by *pdev.
 * Return 0 on success.
 *
 * NOTE: pdev can be NULL, for the case of an EISA driver
 */
static int vortex_probe1(struct pci_dev *pdev,
			long ioaddr, int irq,
			int chip_idx, int card_idx)
{
	// *** RTnet ***
	struct rtnet_device *rtdev = NULL;
	// *** RTnet ***

	struct vortex_private *vp;
	int option;
	unsigned int eeprom[0x40], checksum = 0;		/* EEPROM contents */
	int i, step;
	static int printed_version;
	int retval, print_info;
	struct vortex_chip_info * const vci = &vortex_info_tbl[chip_idx];
	const char *print_name;



	if (!printed_version) {
		printk (version);
		printed_version = 1;
	}

	print_name = pdev ? pci_name(pdev) : "3c59x";

	// *** RTnet ***
	rtdev = rt_alloc_etherdev(sizeof(*vp), RX_RING_SIZE * 2 + TX_RING_SIZE);
	retval = -ENOMEM;
	if (!rtdev) {
		printk (KERN_ERR PFX "unable to allocate etherdev, aborting\n");
		goto out;
	}
	rtdev_alloc_name(rtdev, "rteth%d");
	memset(rtdev->priv, 0, sizeof(*vp));
	rt_rtdev_connect(rtdev, &RTDEV_manager);
	rtdev->vers = RTDEV_VERS_2_0;
	// *** RTnet ***

	vp = rtdev->priv;

	/* The lower four bits are the media type. */
	if (rtdev->mem_start) {
		/*
		 * The 'options' param is passed in as the third arg to the
		 * LILO 'ether=' argument for non-modular use
		 */
		option = rtdev->mem_start;
	}
	else if (card_idx < MAX_UNITS)
		option = options[card_idx];
	else
		option = -1;

	if (option > 0) {
		if (option & 0x8000)
			vortex_debug = 7;
		if (option & 0x4000)
			vortex_debug = 2;
		if (option & 0x0400)
			vp->enable_wol = 1;
	}

	print_info = (vortex_debug > 1);
	if (print_info)
		printk (KERN_INFO "See Documentation/networking/vortex.txt\n");

	printk(KERN_INFO "%s: 3Com %s %s at 0x%lx. Vers " DRV_VERSION "\n",
		print_name,
		pdev ? "PCI" : "EISA",
		vci->name,
		ioaddr);

	rtdev->base_addr = ioaddr;
	rtdev->irq = irq;
	rtdev->mtu = mtu;
	vp->drv_flags = vci->drv_flags;
	vp->has_nway = (vci->drv_flags & HAS_NWAY) ? 1 : 0;
	vp->io_size = vci->io_size;
	vp->card_idx = card_idx;

	/* module list only for EISA devices */
	if (pdev == NULL) {
		vp->next_module = root_vortex_eisa_dev;
		root_vortex_eisa_dev = rtdev;
	}

	/* PCI-only startup logic */
	if (pdev) {
		/* EISA resources already marked, so only PCI needs to do this here */
		/* Ignore return value, because Cardbus drivers already allocate for us */
		if (!request_region(ioaddr, vci->io_size, print_name))
			printk(KERN_INFO "rt_3c50x: request region failed\n");
		else
			vp->must_free_region = 1;

		/* enable bus-mastering if necessary */
		if (vci->flags & PCI_USES_MASTER)
			pci_set_master (pdev);

		if (vci->drv_flags & IS_VORTEX) {
			u8 pci_latency;
			u8 new_latency = 248;

			/* Check the PCI latency value.  On the 3c590 series the latency timer
			   must be set to the maximum value to avoid data corruption that occurs
			   when the timer expires during a transfer.  This bug exists the Vortex
			   chip only. */
			pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &pci_latency);
			if (pci_latency < new_latency) {
				printk(KERN_INFO "%s: Overriding PCI latency"
					" timer (CFLT) setting of %d, new value is %d.\n",
					print_name, pci_latency, new_latency);
				pci_write_config_byte(pdev, PCI_LATENCY_TIMER, new_latency);
			}
		}
	}

	rtdm_lock_init(&vp->lock);
	spin_lock_init(&vp->mdio_lock);
	vp->pdev = pdev;

	/* Makes sure rings are at least 16 byte aligned. */
	vp->rx_ring = pci_alloc_consistent(pdev, sizeof(struct boom_rx_desc) * RX_RING_SIZE
					+ sizeof(struct boom_tx_desc) * TX_RING_SIZE,
					&vp->rx_ring_dma);
	retval = -ENOMEM;
	if (vp->rx_ring == 0)
		goto free_region;

	vp->tx_ring = (struct boom_tx_desc *)(vp->rx_ring + RX_RING_SIZE);
	vp->tx_ring_dma = vp->rx_ring_dma + sizeof(struct boom_rx_desc) * RX_RING_SIZE;

	/* if we are a PCI driver, we store info in pdev->driver_data
	 * instead of a module list */
	if (pdev)
		pci_set_drvdata(pdev, rtdev);

	vp->media_override = 7;
	if (option >= 0) {
		vp->media_override = ((option & 7) == 2)  ?  0	:  option & 15;
		if (vp->media_override != 7)
			vp->medialock = 1;
		vp->full_duplex = (option & 0x200) ? 1 : 0;
		vp->bus_master = (option & 16) ? 1 : 0;
	}

	if (card_idx < MAX_UNITS) {
		if (full_duplex[card_idx] > 0)
			vp->full_duplex = 1;
		if (flow_ctrl[card_idx] > 0)
			vp->flow_ctrl = 1;
		if (enable_wol[card_idx] > 0)
			vp->enable_wol = 1;
	}

	vp->force_fd = vp->full_duplex;
	vp->options = option;

	/* Read the station address from the EEPROM. */
	EL3WINDOW(0);
	{
		int base;

		if (vci->drv_flags & EEPROM_8BIT)
			base = 0x230;
		else if (vci->drv_flags & EEPROM_OFFSET)
			base = EEPROM_Read + 0x30;
		else
			base = EEPROM_Read;

		for (i = 0; i < 0x40; i++) {
			int timer;
			outw(base + i, ioaddr + Wn0EepromCmd);
			/* Pause for at least 162 us. for the read to take place. */
			for (timer = 10; timer >= 0; timer--) {
				udelay(162);
				if ((inw(ioaddr + Wn0EepromCmd) & 0x8000) == 0)
					break;
			}
			eeprom[i] = inw(ioaddr + Wn0EepromData);
		}
	}
	for (i = 0; i < 0x18; i++)
		checksum ^= eeprom[i];
	checksum = (checksum ^ (checksum >> 8)) & 0xff;
	if (checksum != 0x00) {		/* Grrr, needless incompatible change 3Com. */
		while (i < 0x21)
			checksum ^= eeprom[i++];
		checksum = (checksum ^ (checksum >> 8)) & 0xff;
	}
	if ((checksum != 0x00) && !(vci->drv_flags & IS_TORNADO))
		printk(" ***INVALID CHECKSUM %4.4x*** ", checksum);

	for (i = 0; i < 3; i++)
		((u16 *)rtdev->dev_addr)[i] = htons(eeprom[i + 10]);
	if (print_info) {
		for (i = 0; i < 6; i++)
			printk("%c%2.2x", i ? ':' : ' ', rtdev->dev_addr[i]);
	}
	EL3WINDOW(2);
	for (i = 0; i < 6; i++)
		outb(rtdev->dev_addr[i], ioaddr + i);

#ifdef __sparc__
	if (print_info)
		printk(", IRQ %s\n", __irq_itoa(rtdev->irq));
#else
	if (print_info)
		printk(", IRQ %d\n", rtdev->irq);
	/* Tell them about an invalid IRQ. */
	if (rtdev->irq <= 0 || rtdev->irq >= NR_IRQS)
		printk(KERN_WARNING " *** Warning: IRQ %d is unlikely to work! ***\n",
			rtdev->irq);
#endif

	EL3WINDOW(4);
	step = (inb(ioaddr + Wn4_NetDiag) & 0x1e) >> 1;
	if (print_info) {
		printk(KERN_INFO "  product code %02x%02x rev %02x.%d date %02d-"
			"%02d-%02d\n", eeprom[6]&0xff, eeprom[6]>>8, eeprom[0x14],
			step, (eeprom[4]>>5) & 15, eeprom[4] & 31, eeprom[4]>>9);
	}


	if (pdev && vci->drv_flags & HAS_CB_FNS) {
		unsigned long fn_st_addr;			/* Cardbus function status space */
		unsigned short n;

		fn_st_addr = pci_resource_start (pdev, 2);
		if (fn_st_addr) {
			vp->cb_fn_base = ioremap(fn_st_addr, 128);
			retval = -ENOMEM;
			if (!vp->cb_fn_base)
				goto free_ring;
		}
		if (print_info) {
			printk(KERN_INFO "%s: CardBus functions mapped %8.8lx->%p\n",
				print_name, fn_st_addr, vp->cb_fn_base);
		}
		EL3WINDOW(2);

		n = inw(ioaddr + Wn2_ResetOptions) & ~0x4010;
		if (vp->drv_flags & INVERT_LED_PWR)
			n |= 0x10;
		if (vp->drv_flags & INVERT_MII_PWR)
			n |= 0x4000;
		outw(n, ioaddr + Wn2_ResetOptions);
	}

	/* Extract our information from the EEPROM data. */
	vp->info1 = eeprom[13];
	vp->info2 = eeprom[15];
	vp->capabilities = eeprom[16];

	if (vp->info1 & 0x8000) {
		vp->full_duplex = 1;
		if (print_info)
			printk(KERN_INFO "Full duplex capable\n");
	}

	{
		static const char * ram_split[] = {"5:3", "3:1", "1:1", "3:5"};
		unsigned int config;
		EL3WINDOW(3);
		vp->available_media = inw(ioaddr + Wn3_Options);
		if ((vp->available_media & 0xff) == 0)		/* Broken 3c916 */
			vp->available_media = 0x40;
		config = inl(ioaddr + Wn3_Config);
		if (print_info) {
			printk(KERN_DEBUG "  Internal config register is %4.4x, "
				"transceivers %#x.\n", config, inw(ioaddr + Wn3_Options));
			printk(KERN_INFO "  %dK %s-wide RAM %s Rx:Tx split, %s%s interface.\n",
				8 << RAM_SIZE(config),
				RAM_WIDTH(config) ? "word" : "byte",
				ram_split[RAM_SPLIT(config)],
				AUTOSELECT(config) ? "autoselect/" : "",
				XCVR(config) > XCVR_ExtMII ? "<invalid transceiver>" :
				media_tbl[XCVR(config)].name);
		}
		vp->default_media = XCVR(config);
		if (vp->default_media == XCVR_NWAY)
			vp->has_nway = 1;
		vp->autoselect = AUTOSELECT(config);
	}

	if (vp->media_override != 7) {
		printk(KERN_INFO "%s:  Media override to transceiver type %d (%s).\n",
			print_name, vp->media_override,
			media_tbl[vp->media_override].name);
		rtdev->if_port = vp->media_override;
	} else
		rtdev->if_port = vp->default_media;

	if (rtdev->if_port == XCVR_MII || rtdev->if_port == XCVR_NWAY) {
		int phy, phy_idx = 0;
		EL3WINDOW(4);
		mii_preamble_required++;
		mii_preamble_required++;
		mdio_read(rtdev, 24, 1);
		for (phy = 0; phy < 32 && phy_idx < 1; phy++) {
			int mii_status, phyx;

			/*
			 * For the 3c905CX we look at index 24 first, because it bogusly
			 * reports an external PHY at all indices
			 */
			if (phy == 0)
				phyx = 24;
			else if (phy <= 24)
				phyx = phy - 1;
			else
				phyx = phy;
			mii_status = mdio_read(rtdev, phyx, 1);
			if (mii_status	&&  mii_status != 0xffff) {
				vp->phys[phy_idx++] = phyx;
				if (print_info) {
					printk(KERN_INFO "  MII transceiver found at address %d,"
						" status %4x.\n", phyx, mii_status);
				}
				if ((mii_status & 0x0040) == 0)
					mii_preamble_required++;
			}
		}
		mii_preamble_required--;
		if (phy_idx == 0) {
			printk(KERN_WARNING"  ***WARNING*** No MII transceivers found!\n");
			vp->phys[0] = 24;
		} else {
			vp->advertising = mdio_read(rtdev, vp->phys[0], 4);
			if (vp->full_duplex) {
				/* Only advertise the FD media types. */
				vp->advertising &= ~0x02A0;
				mdio_write(rtdev, vp->phys[0], 4, vp->advertising);
			}
		}
	}

	if (vp->capabilities & CapBusMaster) {
		vp->full_bus_master_tx = 1;
		if (print_info) {
			printk(KERN_INFO "  Enabling bus-master transmits and %s receives.\n",
				(vp->info2 & 1) ? "early" : "whole-frame" );
		}
		vp->full_bus_master_rx = (vp->info2 & 1) ? 1 : 2;
		vp->bus_master = 0;		/* AKPM: vortex only */
	}

	// *** RTnet ***
	/* The 3c59x-specific entries in the device structure. */
	rtdev->open = vortex_open;
	if (vp->full_bus_master_tx) {
		rtdev->hard_start_xmit = boomerang_start_xmit;
		/* Actually, it still should work with iommu. */
		rtdev->features |= NETIF_F_SG;
		if (((hw_checksums[card_idx] == -1) && (vp->drv_flags & HAS_HWCKSM)) ||
			(hw_checksums[card_idx] == 1)) {
			rtdev->features |= NETIF_F_IP_CSUM;
		}
	} else {
		rtdev->hard_start_xmit = vortex_start_xmit;
	}
	rtdev->get_stats = vortex_get_stats;

	if (print_info) {
		printk(KERN_INFO "%s: scatter/gather %sabled. h/w checksums %sabled\n",
			print_name,
			(rtdev->features & NETIF_F_SG) ? "en":"dis",
			(rtdev->features & NETIF_F_IP_CSUM) ? "en":"dis");
	}

	rtdev->stop = vortex_close;
	retval = rt_register_rtnetdev(rtdev);
	if (retval) {
		printk(KERN_ERR "rt_3c59x: rtnet device registration failed %d\n",retval);
		goto free_ring;
	}
	return 0;

	// *** RTnet ***

  free_ring:
	pci_free_consistent(pdev,
			sizeof(struct boom_rx_desc) * RX_RING_SIZE
			+ sizeof(struct boom_tx_desc) * TX_RING_SIZE,
			vp->rx_ring,
			vp->rx_ring_dma);
  free_region:
	if (vp->must_free_region)
		release_region(ioaddr, vci->io_size);
	rtdev_free (rtdev);
	printk(KERN_ERR PFX "vortex_probe1 fails.  Returns %d\n", retval);
  out:
	return retval;
}

static void
issue_and_wait(struct rtnet_device *rtdev, int cmd)
{
	int i;

	outw(cmd, rtdev->base_addr + EL3_CMD);
	for (i = 0; i < 2000; i++) {
		if (!(inw(rtdev->base_addr + EL3_STATUS) & CmdInProgress))
			return;
	}

	/* OK, that didn't work.  Do it the slow way.  One second */
	for (i = 0; i < 100000; i++) {
		if (!(inw(rtdev->base_addr + EL3_STATUS) & CmdInProgress)) {
			if (vortex_debug > 1)
				rtdm_printk(KERN_INFO "%s: command 0x%04x took %d usecs\n",
					rtdev->name, cmd, i * 10);
			return;
		}
		udelay(10);
	}
	rtdm_printk(KERN_ERR "%s: command 0x%04x did not complete! Status=0x%x\n",
		rtdev->name, cmd, inw(rtdev->base_addr + EL3_STATUS));
}

static void
vortex_up(struct rtnet_device *rtdev)
{
	long ioaddr = rtdev->base_addr;
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	unsigned int config;
	int i;

	if (vp->pdev && vp->enable_wol) {
		pci_set_power_state(vp->pdev, 0);	/* Go active */
		pci_restore_state(vp->pdev, vp->power_state);
	}

	/* Before initializing select the active media port. */
	EL3WINDOW(3);
	config = inl(ioaddr + Wn3_Config);

	if (vp->media_override != 7) {
		printk(KERN_INFO "%s: Media override to transceiver %d (%s).\n",
			rtdev->name, vp->media_override,
			media_tbl[vp->media_override].name);
		rtdev->if_port = vp->media_override;
	} else if (vp->autoselect) {
		if (vp->has_nway) {
			if (vortex_debug > 1)
				printk(KERN_INFO "%s: using NWAY device table, not %d\n",
					rtdev->name, rtdev->if_port);
			rtdev->if_port = XCVR_NWAY;
		} else {
			/* Find first available media type, starting with 100baseTx. */
			rtdev->if_port = XCVR_100baseTx;
			while (! (vp->available_media & media_tbl[rtdev->if_port].mask))
				rtdev->if_port = media_tbl[rtdev->if_port].next;
			if (vortex_debug > 1)
				printk(KERN_INFO "%s: first available media type: %s\n",
					rtdev->name, media_tbl[rtdev->if_port].name);
		}
	} else {
		rtdev->if_port = vp->default_media;
		if (vortex_debug > 1)
			printk(KERN_INFO "%s: using default media %s\n",
				rtdev->name, media_tbl[rtdev->if_port].name);
	}

	init_timer(&vp->timer);
	vp->timer.expires = RUN_AT(media_tbl[rtdev->if_port].wait);
	vp->timer.data = (unsigned long)rtdev;
	// *** RTnet  vp->timer.function = vortex_timer;		/* timer handler */
	// *** RTnet  add_timer(&vp->timer);

	init_timer(&vp->rx_oom_timer);
	vp->rx_oom_timer.data = (unsigned long)rtdev;
	// **** RTnet *** vp->rx_oom_timer.function = rx_oom_timer;

	if (vortex_debug > 1)
		printk(KERN_DEBUG "%s: Initial media type %s.\n",
			rtdev->name, media_tbl[rtdev->if_port].name);

	vp->full_duplex = vp->force_fd;
	config = BFINS(config, rtdev->if_port, 20, 4);
	if (vortex_debug > 6)
		printk(KERN_DEBUG "vortex_up(): writing 0x%x to InternalConfig\n", config);
	outl(config, ioaddr + Wn3_Config);

	if (rtdev->if_port == XCVR_MII || rtdev->if_port == XCVR_NWAY) {
		int mii_reg1, mii_reg5;
		EL3WINDOW(4);
		/* Read BMSR (reg1) only to clear old status. */
		mii_reg1 = mdio_read(rtdev, vp->phys[0], 1);
		mii_reg5 = mdio_read(rtdev, vp->phys[0], 5);
		if (mii_reg5 == 0xffff	||  mii_reg5 == 0x0000)
			;					/* No MII device or no link partner report */
		else if ((mii_reg5 & 0x0100) != 0	/* 100baseTx-FD */
			|| (mii_reg5 & 0x00C0) == 0x0040) /* 10T-FD, but not 100-HD */
			vp->full_duplex = 1;
		vp->partner_flow_ctrl = ((mii_reg5 & 0x0400) != 0);
		if (vortex_debug > 1)
			printk(KERN_INFO "%s: MII #%d status %4.4x, link partner capability %4.4x,"
				" info1 %04x, setting %s-duplex.\n",
				rtdev->name, vp->phys[0],
				mii_reg1, mii_reg5,
				vp->info1, ((vp->info1 & 0x8000) || vp->full_duplex) ? "full" : "half");
		EL3WINDOW(3);
	}

	/* Set the full-duplex bit. */
	outw(	((vp->info1 & 0x8000) || vp->full_duplex ? 0x20 : 0) |
		(rtdev->mtu > 1500 ? 0x40 : 0) |
		((vp->full_duplex && vp->flow_ctrl && vp->partner_flow_ctrl) ? 0x100 : 0),
		ioaddr + Wn3_MAC_Ctrl);

	if (vortex_debug > 1) {
		printk(KERN_DEBUG "%s: vortex_up() InternalConfig %8.8x.\n",
			rtdev->name, config);
	}

	issue_and_wait(rtdev, TxReset);
	/*
	 * Don't reset the PHY - that upsets autonegotiation during DHCP operations.
	 */
	issue_and_wait(rtdev, RxReset|0x04);

	outw(SetStatusEnb | 0x00, ioaddr + EL3_CMD);

	if (vortex_debug > 1) {
		EL3WINDOW(4);
		printk(KERN_DEBUG "%s: vortex_up() irq %d media status %4.4x.\n",
			rtdev->name, rtdev->irq, inw(ioaddr + Wn4_Media));
	}

	/* Set the station address and mask in window 2 each time opened. */
	EL3WINDOW(2);
	for (i = 0; i < 6; i++)
		outb(rtdev->dev_addr[i], ioaddr + i);
	for (; i < 12; i+=2)
		outw(0, ioaddr + i);

	if (vp->cb_fn_base) {
		unsigned short n = inw(ioaddr + Wn2_ResetOptions) & ~0x4010;
		if (vp->drv_flags & INVERT_LED_PWR)
			n |= 0x10;
		if (vp->drv_flags & INVERT_MII_PWR)
			n |= 0x4000;
		outw(n, ioaddr + Wn2_ResetOptions);
	}

	if (rtdev->if_port == XCVR_10base2)
		/* Start the thinnet transceiver. We should really wait 50ms...*/
		outw(StartCoax, ioaddr + EL3_CMD);
	if (rtdev->if_port != XCVR_NWAY) {
		EL3WINDOW(4);
		outw((inw(ioaddr + Wn4_Media) & ~(Media_10TP|Media_SQE)) |
			media_tbl[rtdev->if_port].media_bits, ioaddr + Wn4_Media);
	}

	/* Switch to the stats window, and clear all stats by reading. */
	outw(StatsDisable, ioaddr + EL3_CMD);
	EL3WINDOW(6);
	for (i = 0; i < 10; i++)
		inb(ioaddr + i);
	inw(ioaddr + 10);
	inw(ioaddr + 12);
	/* New: On the Vortex we must also clear the BadSSD counter. */
	EL3WINDOW(4);
	inb(ioaddr + 12);
	/* ..and on the Boomerang we enable the extra statistics bits. */
	outw(0x0040, ioaddr + Wn4_NetDiag);

	/* Switch to register set 7 for normal use. */
	EL3WINDOW(7);

	if (vp->full_bus_master_rx) { /* Boomerang bus master. */
		vp->cur_rx = vp->dirty_rx = 0;
		/* Initialize the RxEarly register as recommended. */
		outw(SetRxThreshold + (1536>>2), ioaddr + EL3_CMD);
		outl(0x0020, ioaddr + PktStatus);
		outl(vp->rx_ring_dma, ioaddr + UpListPtr);
	}
	if (vp->full_bus_master_tx) {		/* Boomerang bus master Tx. */
		vp->cur_tx = vp->dirty_tx = 0;
		if (vp->drv_flags & IS_BOOMERANG)
			outb(PKT_BUF_SZ>>8, ioaddr + TxFreeThreshold); /* Room for a packet. */
		/* Clear the Rx, Tx rings. */
		for (i = 0; i < RX_RING_SIZE; i++)	/* AKPM: this is done in vortex_open, too */
			vp->rx_ring[i].status = 0;
		for (i = 0; i < TX_RING_SIZE; i++)
			vp->tx_skbuff[i] = 0;
		outl(0, ioaddr + DownListPtr);
	}
	/* Set receiver mode: presumably accept b-case and phys addr only. */
	set_rx_mode(rtdev);
	outw(StatsEnable, ioaddr + EL3_CMD); /* Turn on statistics. */

//	issue_and_wait(dev, SetTxStart|0x07ff);
	outw(RxEnable, ioaddr + EL3_CMD); /* Enable the receiver. */
	outw(TxEnable, ioaddr + EL3_CMD); /* Enable transmitter. */
	/* Allow status bits to be seen. */
	vp->status_enable = SetStatusEnb | HostError|IntReq|StatsFull|TxComplete|
		(vp->full_bus_master_tx ? DownComplete : TxAvailable) |
		(vp->full_bus_master_rx ? UpComplete : RxComplete) |
		(vp->bus_master ? DMADone : 0);
	vp->intr_enable = SetIntrEnb | IntLatch | TxAvailable |
		(vp->full_bus_master_rx ? 0 : RxComplete) |
		StatsFull | HostError | TxComplete | IntReq
		| (vp->bus_master ? DMADone : 0) | UpComplete | DownComplete;
	outw(vp->status_enable, ioaddr + EL3_CMD);
	/* Ack all pending events, and set active indicator mask. */
	outw(AckIntr | IntLatch | TxAvailable | RxEarly | IntReq,
		ioaddr + EL3_CMD);
	outw(vp->intr_enable, ioaddr + EL3_CMD);
	if (vp->cb_fn_base)			/* The PCMCIA people are idiots.  */
		writel(0x8000, vp->cb_fn_base + 4);
	rtnetif_start_queue (rtdev);
}

static int
vortex_open(struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	int i;
	int retval;

	// *** RTnet ***
	rt_stack_connect(rtdev, &STACK_manager);

	if ((retval = rtdm_irq_request(&vp->irq_handle, rtdev->irq,
						(vp->full_bus_master_rx ? boomerang_interrupt : vortex_interrupt),
						0, "rt_3c59x", rtdev))) {
		printk(KERN_ERR "%s: Could not reserve IRQ %d\n", rtdev->name, rtdev->irq);
		goto out;
	}
	// *** RTnet ***

	if (vp->full_bus_master_rx) { /* Boomerang bus master. */
		if (vortex_debug > 2)
			printk(KERN_DEBUG "%s:	Filling in the Rx ring.\n", rtdev->name);
		for (i = 0; i < RX_RING_SIZE; i++) {
			struct rtskb *skb; // *** RTnet
			vp->rx_ring[i].next = cpu_to_le32(vp->rx_ring_dma + sizeof(struct boom_rx_desc) * (i+1));
			vp->rx_ring[i].status = 0;	/* Clear complete bit. */
			vp->rx_ring[i].length = cpu_to_le32(PKT_BUF_SZ | LAST_FRAG);
			skb = rtnetdev_alloc_rtskb(rtdev, PKT_BUF_SZ);
			vp->rx_skbuff[i] = skb;
			if (skb == NULL)
				break;			/* Bad news!  */
			// *** RTnet ***
			rtskb_reserve(skb, 2);	/* Align IP on 16 byte boundaries */
			vp->rx_ring[i].addr = cpu_to_le32(pci_map_single(vp->pdev,
													skb->tail, PKT_BUF_SZ, PCI_DMA_FROMDEVICE));
			// *** RTnet ***
		}
		if (i != RX_RING_SIZE) {
			int j;
			printk(KERN_EMERG "%s: no memory for rx ring\n", rtdev->name);
			for (j = 0; j < i; j++) {
				if (vp->rx_skbuff[j]) {
					dev_kfree_rtskb(vp->rx_skbuff[j]);
					vp->rx_skbuff[j] = 0;
				}
			}
			retval = -ENOMEM;
			goto out_free_irq;
		}
		/* Wrap the ring. */
		vp->rx_ring[i-1].next = cpu_to_le32(vp->rx_ring_dma);
	}

	vortex_up(rtdev);
	return 0;

  out_free_irq:

	// *** RTnet ***
	if ( (i=rtdm_irq_free(&vp->irq_handle))<0 )
		return i;
	rt_stack_disconnect(rtdev);
	// *** RTnet ***
  out:
	if (vortex_debug > 1)
		printk(KERN_ERR "%s: vortex_open() fails: returning %d\n", rtdev->name, retval);
	return retval;
}

/*
 * Handle uncommon interrupt sources.  This is a separate routine to minimize
 * the cache impact.
 */
static void
vortex_error(struct rtnet_device *rtdev, int status, nanosecs_abs_t *time_stamp)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	int do_tx_reset = 0, reset_mask = 0;
	unsigned char tx_status = 0;
	int packets=0;

	if (vortex_debug > 2) {
		rtdm_printk(KERN_ERR "%s: vortex_error(), status=0x%x\n", rtdev->name, status);
	}

	if (status & TxComplete) {			/* Really "TxError" for us. */
		tx_status = inb(ioaddr + TxStatus);
		/* Presumably a tx-timeout. We must merely re-enable. */
		if (vortex_debug > 2
			|| (tx_status != 0x88 && vortex_debug > 0)) {
			rtdm_printk(KERN_ERR "%s: Transmit error, Tx status register %2.2x.\n",
				rtdev->name, tx_status);
			if (tx_status == 0x82) {
				rtdm_printk(KERN_ERR "Probably a duplex mismatch.  See "
					"Documentation/networking/vortex.txt\n");
			}
			dump_tx_ring(rtdev);
		}
		if (tx_status & 0x14)  vp->stats.tx_fifo_errors++;
		if (tx_status & 0x38)  vp->stats.tx_aborted_errors++;
		outb(0, ioaddr + TxStatus);
		if (tx_status & 0x30) {			/* txJabber or txUnderrun */
			do_tx_reset = 1;
		} else if ((tx_status & 0x08) && (vp->drv_flags & MAX_COLLISION_RESET)) {	/* maxCollisions */
			do_tx_reset = 1;
			reset_mask = 0x0108;		/* Reset interface logic, but not download logic */
		} else {						/* Merely re-enable the transmitter. */
			outw(TxEnable, ioaddr + EL3_CMD);
		}
	}

	if (status & RxEarly) {				/* Rx early is unused. */
		vortex_rx(rtdev, &packets, time_stamp);
		outw(AckIntr | RxEarly, ioaddr + EL3_CMD);
	}
	if (status & StatsFull) {			/* Empty statistics. */
		static int DoneDidThat;
		if (vortex_debug > 4)
			rtdm_printk(KERN_DEBUG "%s: Updating stats.\n", rtdev->name);
		// *** RTnet *** update_stats(ioaddr, dev);
		/* HACK: Disable statistics as an interrupt source. */
		/* This occurs when we have the wrong media type! */
		if (DoneDidThat == 0  &&
			inw(ioaddr + EL3_STATUS) & StatsFull) {
			rtdm_printk(KERN_WARNING "%s: Updating statistics failed, disabling "
				"stats as an interrupt source.\n", rtdev->name);
			EL3WINDOW(5);
			outw(SetIntrEnb | (inw(ioaddr + 10) & ~StatsFull), ioaddr + EL3_CMD);
			vp->intr_enable &= ~StatsFull;
			EL3WINDOW(7);
			DoneDidThat++;
		}
	}
	if (status & IntReq) {		/* Restore all interrupt sources.  */
		outw(vp->status_enable, ioaddr + EL3_CMD);
		outw(vp->intr_enable, ioaddr + EL3_CMD);
	}
	if (status & HostError) {
		u16 fifo_diag;
		EL3WINDOW(4);
		fifo_diag = inw(ioaddr + Wn4_FIFODiag);
		rtdm_printk(KERN_ERR "%s: Host error, FIFO diagnostic register %4.4x.\n",
			rtdev->name, fifo_diag);
		/* Adapter failure requires Tx/Rx reset and reinit. */
		if (vp->full_bus_master_tx) {
			int bus_status = inl(ioaddr + PktStatus);
			/* 0x80000000 PCI master abort. */
			/* 0x40000000 PCI target abort. */
			if (vortex_debug)
				rtdm_printk(KERN_ERR "%s: PCI bus error, bus status %8.8x\n", rtdev->name, bus_status);

			/* In this case, blow the card away */
			vortex_down(rtdev);
			issue_and_wait(rtdev, TotalReset | 0xff);
			vortex_up(rtdev);		/* AKPM: bug.  vortex_up() assumes that the rx ring is full. It may not be. */
		} else if (fifo_diag & 0x0400)
			do_tx_reset = 1;
		if (fifo_diag & 0x3000) {
			/* Reset Rx fifo and upload logic */
			issue_and_wait(rtdev, RxReset|0x07);
			/* Set the Rx filter to the current state. */
			set_rx_mode(rtdev);
			outw(RxEnable, ioaddr + EL3_CMD); /* Re-enable the receiver. */
			outw(AckIntr | HostError, ioaddr + EL3_CMD);
		}
	}

	if (do_tx_reset) {
		issue_and_wait(rtdev, TxReset|reset_mask);
		outw(TxEnable, ioaddr + EL3_CMD);
		if (!vp->full_bus_master_tx)
			rtnetif_wake_queue(rtdev);
	}
}

static int
vortex_start_xmit(struct rtskb *skb, struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	rtdm_lockctx_t context;

	/* Put out the doubleword header... */
	outl(skb->len, ioaddr + TX_FIFO);
	if (vp->bus_master) {
		/* Set the bus-master controller to transfer the packet. */
		int len = (skb->len + 3) & ~3;
		outl(	vp->tx_skb_dma = pci_map_single(vp->pdev, skb->data,
							len, PCI_DMA_TODEVICE),
			ioaddr + Wn7_MasterAddr);
		outw(len, ioaddr + Wn7_MasterLen);
		vp->tx_skb = skb;

		rtdm_lock_irqsave(context);
		if (unlikely(skb->xmit_stamp != NULL))
			*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() +
						*skb->xmit_stamp);
		outw(StartDMADown, ioaddr + EL3_CMD);
		rtdm_lock_irqrestore(context);

		/* rtnetif_wake_queue() will be called at the DMADone interrupt. */
	} else {
		rtdm_printk("rt_3x59x: UNSUPPORTED CODE PATH (device is lacking DMA support)!\n");
		/* ... and the packet rounded to a doubleword. */
		outsl(ioaddr + TX_FIFO, skb->data, (skb->len + 3) >> 2);
		dev_kfree_rtskb (skb);
		if (inw(ioaddr + TxFree) > 1536) {
			rtnetif_start_queue (rtdev);	/* AKPM: redundant? */
		} else {
			/* Interrupt us when the FIFO has room for max-sized packet. */
			rtnetif_stop_queue(rtdev);
			outw(SetTxThreshold + (1536>>2), ioaddr + EL3_CMD);
		}
	}

	//rtdev->trans_start = jiffies;

	/* Clear the Tx status stack. */
	{
		int tx_status;
		int i = 32;

		while (--i > 0	&&	(tx_status = inb(ioaddr + TxStatus)) > 0) {
			if (tx_status & 0x3C) {		/* A Tx-disabling error occurred.  */
				if (vortex_debug > 2)
					printk(KERN_DEBUG "%s: Tx error, status %2.2x.\n",
						rtdev->name, tx_status);
				if (tx_status & 0x04) vp->stats.tx_fifo_errors++;
				if (tx_status & 0x38) vp->stats.tx_aborted_errors++;
				if (tx_status & 0x30) {
					issue_and_wait(rtdev, TxReset);
				}
				outw(TxEnable, ioaddr + EL3_CMD);
			}
			outb(0x00, ioaddr + TxStatus); /* Pop the status stack. */
		}
	}
	return 0;
}

static int
boomerang_start_xmit(struct rtskb *skb, struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	/* Calculate the next Tx descriptor entry. */
	int entry = vp->cur_tx % TX_RING_SIZE;
	struct boom_tx_desc *prev_entry = &vp->tx_ring[(vp->cur_tx-1) % TX_RING_SIZE];
	rtdm_lockctx_t context;

	if (vortex_debug > 6) {
		rtdm_printk(KERN_DEBUG "boomerang_start_xmit()\n");
		if (vortex_debug > 3)
			rtdm_printk(KERN_DEBUG "%s: Trying to send a packet, Tx index %d.\n",
				rtdev->name, vp->cur_tx);
	}

	if (vp->cur_tx - vp->dirty_tx >= TX_RING_SIZE) {
		if (vortex_debug > 0)
			rtdm_printk(KERN_WARNING "%s: BUG! Tx Ring full, refusing to send buffer.\n",
				rtdev->name);
		rtnetif_stop_queue(rtdev);
		return 1;
	}

	vp->tx_skbuff[entry] = skb;

	vp->tx_ring[entry].next = 0;
#if DO_ZEROCOPY
	if (skb->ip_summed != CHECKSUM_HW)
		vp->tx_ring[entry].status = cpu_to_le32(skb->len | TxIntrUploaded);
	else
		vp->tx_ring[entry].status = cpu_to_le32(skb->len | TxIntrUploaded | AddTCPChksum);

	if (!skb_shinfo(skb)->nr_frags) {
		{
//            int j;
//            for (j=0; j<skb->len; j++)
//            {
//                rtdm_printk("%02x ", skb->data[j]);
//            }

		}
		vp->tx_ring[entry].frag[0].addr = cpu_to_le32(pci_map_single(vp->pdev,
														skb->data, skb->len, PCI_DMA_TODEVICE));
		vp->tx_ring[entry].frag[0].length = cpu_to_le32(skb->len | LAST_FRAG);
	} else {
		int i;

		vp->tx_ring[entry].frag[0].addr = cpu_to_le32(pci_map_single(vp->pdev,
														skb->data, skb->len, PCI_DMA_TODEVICE));
		vp->tx_ring[entry].frag[0].length = cpu_to_le32(skb->len);

		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

			vp->tx_ring[entry].frag[i+1].addr =
				cpu_to_le32(pci_map_single(vp->pdev, // *** RTnet: page mapping correct? Or is this code never used?
								(void*)page_address(frag->page) + frag->page_offset,
								frag->size, PCI_DMA_TODEVICE));

			if (i == skb_shinfo(skb)->nr_frags-1)
				vp->tx_ring[entry].frag[i+1].length = cpu_to_le32(frag->size|LAST_FRAG);
			else
				vp->tx_ring[entry].frag[i+1].length = cpu_to_le32(frag->size);
		}
	}
#else
	vp->tx_ring[entry].addr = cpu_to_le32(pci_map_single(vp->pdev,
											skb->data, skb->len, PCI_DMA_TODEVICE));
	vp->tx_ring[entry].length = cpu_to_le32(skb->len | LAST_FRAG);
	vp->tx_ring[entry].status = cpu_to_le32(skb->len | TxIntrUploaded);
#endif

	// *** RTnet ***
	rtdm_irq_disable(&vp->irq_handle);
	rtdm_lock_get(&vp->lock);
	// *** RTnet ***

	/* Wait for the stall to complete. */
	issue_and_wait(rtdev, DownStall);

	rtdm_lock_irqsave(context);
	if (unlikely(skb->xmit_stamp != NULL))
		*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() + *skb->xmit_stamp);

	prev_entry->next = cpu_to_le32(vp->tx_ring_dma + entry * sizeof(struct boom_tx_desc));
	if (inl(ioaddr + DownListPtr) == 0) {
		outl(vp->tx_ring_dma + entry * sizeof(struct boom_tx_desc), ioaddr + DownListPtr);
		vp->queued_packet++;
	}

	vp->cur_tx++;
	if (vp->cur_tx - vp->dirty_tx > TX_RING_SIZE - 1) {
		rtnetif_stop_queue (rtdev);
	} else {					/* Clear previous interrupt enable. */
#if defined(tx_interrupt_mitigation)
		/* Dubious. If in boomeang_interrupt "faster" cyclone ifdef
		 * were selected, this would corrupt DN_COMPLETE. No?
		 */
		prev_entry->status &= cpu_to_le32(~TxIntrUploaded);
#endif
	}
	outw(DownUnstall, ioaddr + EL3_CMD);
	rtdm_lock_put_irqrestore(&vp->lock, context);
	rtdm_irq_enable(&vp->irq_handle);
	//rtdev->trans_start = jiffies;
	return 0;
}

/* The interrupt handler does all of the Rx thread work and cleans up
   after the Tx thread. */

/*
 * This is the ISR for the vortex series chips.
 * full_bus_master_tx == 0 && full_bus_master_rx == 0
 */

static int vortex_interrupt(rtdm_irq_t *irq_handle)
{
	// *** RTnet ***
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtnet_device *rtdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	int packets = 0;
	// *** RTnet ***

	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr;
	int status;
	int work_done = max_interrupt_work;

	ioaddr = rtdev->base_addr;
	rtdm_lock_get(&vp->lock);

	status = inw(ioaddr + EL3_STATUS);

	if (vortex_debug > 6)
		printk("vortex_interrupt(). status=0x%4x\n", status);

	if ((status & IntLatch) == 0)
		goto handler_exit;		/* No interrupt: shared IRQs cause this */

	if (status & IntReq) {
		status |= vp->deferred;
		vp->deferred = 0;
	}

	if (status == 0xffff)		/* h/w no longer present (hotplug)? */
		goto handler_exit;

	if (vortex_debug > 4)
		rtdm_printk(KERN_DEBUG "%s: interrupt, status %4.4x, latency %d ticks.\n",
			rtdev->name, status, inb(ioaddr + Timer));

	do {
		if (vortex_debug > 5)
			rtdm_printk(KERN_DEBUG "%s: In interrupt loop, status %4.4x.\n",
				rtdev->name, status);
		if (status & RxComplete)
			vortex_rx(rtdev, &packets, &time_stamp);

		if (status & TxAvailable) {
			if (vortex_debug > 5)
				rtdm_printk(KERN_DEBUG "	TX room bit was handled.\n");
			/* There's room in the FIFO for a full-sized packet. */
			outw(AckIntr | TxAvailable, ioaddr + EL3_CMD);
			rtnetif_wake_queue (rtdev);
		}

		if (status & DMADone) {
			if (inw(ioaddr + Wn7_MasterStatus) & 0x1000) {
				outw(0x1000, ioaddr + Wn7_MasterStatus); /* Ack the event. */
				pci_unmap_single(vp->pdev, vp->tx_skb_dma, (vp->tx_skb->len + 3) & ~3, PCI_DMA_TODEVICE);
				dev_kfree_rtskb(vp->tx_skb); /* Release the transferred buffer */
				if (inw(ioaddr + TxFree) > 1536) {
					/*
					 * AKPM: FIXME: I don't think we need this.  If the queue was stopped due to
					 * insufficient FIFO room, the TxAvailable test will succeed and call
					 * rtnetif_wake_queue()
					 */
					rtnetif_wake_queue(rtdev);
				} else { /* Interrupt when FIFO has room for max-sized packet. */
					outw(SetTxThreshold + (1536>>2), ioaddr + EL3_CMD);
					rtnetif_stop_queue(rtdev);
				}
			}
		}
		/* Check for all uncommon interrupts at once. */
		if (status & (HostError | RxEarly | StatsFull | TxComplete | IntReq)) {
			if (status == 0xffff)
				break;
			vortex_error(rtdev, status, &time_stamp);
		}

		if (--work_done < 0) {
			rtdm_printk(KERN_WARNING "%s: Too much work in interrupt, status "
				"%4.4x.\n", rtdev->name, status);
			/* Disable all pending interrupts. */
			do {
				vp->deferred |= status;
				outw(SetStatusEnb | (~vp->deferred & vp->status_enable),
					ioaddr + EL3_CMD);
				outw(AckIntr | (vp->deferred & 0x7ff), ioaddr + EL3_CMD);
			} while ((status = inw(ioaddr + EL3_CMD)) & IntLatch);
			/* The timer will reenable interrupts. */
			mod_timer(&vp->timer, jiffies + 1*HZ);
			break;
		}
		/* Acknowledge the IRQ. */
		outw(AckIntr | IntReq | IntLatch, ioaddr + EL3_CMD);
	} while ((status = inw(ioaddr + EL3_STATUS)) & (IntLatch | RxComplete));

	if (vortex_debug > 4)
		rtdm_printk(KERN_DEBUG "%s: exiting interrupt, status %4.4x.\n",
			rtdev->name, status);
  handler_exit:
	rtdm_lock_put(&vp->lock);
	if (packets > 0)
		rt_mark_stack_mgr(rtdev);

	return RTDM_IRQ_HANDLED;
}

/*
 * This is the ISR for the boomerang series chips.
 * full_bus_master_tx == 1 && full_bus_master_rx == 1
 */

static int boomerang_interrupt(rtdm_irq_t *irq_handle)
{
	// *** RTnet ***
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtnet_device *rtdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	int packets = 0;
	// *** RTnet ***

	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr;
	int status;
	int work_done = max_interrupt_work;

	ioaddr = rtdev->base_addr;

	/*
	 * It seems dopey to put the spinlock this early, but we could race against vortex_tx_timeout
	 * and boomerang_start_xmit
	 */
	rtdm_lock_get(&vp->lock);

	status = inw(ioaddr + EL3_STATUS);

	if (vortex_debug > 6)
		rtdm_printk(KERN_DEBUG "boomerang_interrupt. status=0x%4x\n", status);

	if ((status & IntLatch) == 0)
		goto handler_exit;		/* No interrupt: shared IRQs can cause this */

	if (status == 0xffff) {		/* h/w no longer present (hotplug)? */
		if (vortex_debug > 1)
			rtdm_printk(KERN_DEBUG "boomerang_interrupt(1): status = 0xffff\n");
		goto handler_exit;
	}

	if (status & IntReq) {
		status |= vp->deferred;
		vp->deferred = 0;
	}

	if (vortex_debug > 4)
		rtdm_printk(KERN_DEBUG "%s: interrupt, status %4.4x, latency %d ticks.\n",
			rtdev->name, status, inb(ioaddr + Timer));
	do {
		if (vortex_debug > 5)
			rtdm_printk(KERN_DEBUG "%s: In interrupt loop, status %4.4x.\n",
				rtdev->name, status);
		if (status & UpComplete) {
			outw(AckIntr | UpComplete, ioaddr + EL3_CMD);
			if (vortex_debug > 5)
				rtdm_printk(KERN_DEBUG "boomerang_interrupt->boomerang_rx\n");
			boomerang_rx(rtdev, &packets, &time_stamp);
		}

		if (status & DownComplete) {
			unsigned int dirty_tx = vp->dirty_tx;

			outw(AckIntr | DownComplete, ioaddr + EL3_CMD);
			while (vp->cur_tx - dirty_tx > 0) {
				int entry = dirty_tx % TX_RING_SIZE;
				if (inl(ioaddr + DownListPtr) ==
					vp->tx_ring_dma + entry * sizeof(struct boom_tx_desc))
					break;			/* It still hasn't been processed. */

				if (vp->tx_skbuff[entry]) {
					struct rtskb *skb = vp->tx_skbuff[entry];
#if DO_ZEROCOPY
					int i;
					for (i=0; i<=skb_shinfo(skb)->nr_frags; i++)
						pci_unmap_single(vp->pdev,
								le32_to_cpu(vp->tx_ring[entry].frag[i].addr),
								le32_to_cpu(vp->tx_ring[entry].frag[i].length)&0xFFF,
								PCI_DMA_TODEVICE);
#else
					pci_unmap_single(vp->pdev,
							le32_to_cpu(vp->tx_ring[entry].addr), skb->len, PCI_DMA_TODEVICE);
#endif
					dev_kfree_rtskb(skb);
					vp->tx_skbuff[entry] = 0;
				} else {
					rtdm_printk(KERN_DEBUG "boomerang_interrupt: no skb!\n");
				}
				/* vp->stats.tx_packets++;  Counted below. */
				dirty_tx++;
			}
			vp->dirty_tx = dirty_tx;
			if (vp->cur_tx - dirty_tx <= TX_RING_SIZE - 1) {
				if (vortex_debug > 6)
					rtdm_printk(KERN_DEBUG "boomerang_interrupt: wake queue\n");
				rtnetif_wake_queue (rtdev);
			}
		}

		/* Check for all uncommon interrupts at once. */
		if (status & (HostError | RxEarly | StatsFull | TxComplete | IntReq))
			vortex_error(rtdev, status, &time_stamp);

		if (--work_done < 0) {
			rtdm_printk(KERN_WARNING "%s: Too much work in interrupt, status "
				"%4.4x.\n", rtdev->name, status);
			/* Disable all pending interrupts. */
			do {
				vp->deferred |= status;
				outw(SetStatusEnb | (~vp->deferred & vp->status_enable),
					ioaddr + EL3_CMD);
				outw(AckIntr | (vp->deferred & 0x7ff), ioaddr + EL3_CMD);
			} while ((status = inw(ioaddr + EL3_CMD)) & IntLatch);
			/* The timer will reenable interrupts. */
			mod_timer(&vp->timer, jiffies + 1*HZ);
			break;
		}
		/* Acknowledge the IRQ. */
		outw(AckIntr | IntReq | IntLatch, ioaddr + EL3_CMD);
		if (vp->cb_fn_base)			/* The PCMCIA people are idiots.  */
			writel(0x8000, vp->cb_fn_base + 4);

	} while ((status = inw(ioaddr + EL3_STATUS)) & IntLatch);

	if (vortex_debug > 4)
		rtdm_printk(KERN_DEBUG "%s: exiting interrupt, status %4.4x.\n",
			rtdev->name, status);
  handler_exit:
	rtdm_lock_put(&vp->lock);
	if (packets > 0)
		rt_mark_stack_mgr(rtdev);

	return RTDM_IRQ_HANDLED;
}

static int vortex_rx(struct rtnet_device *rtdev, int *packets, nanosecs_abs_t *time_stamp)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	int i;
	short rx_status;

	if (vortex_debug > 5)
		printk(KERN_DEBUG "vortex_rx(): status %4.4x, rx_status %4.4x.\n",
			inw(ioaddr+EL3_STATUS), inw(ioaddr+RxStatus));
	while ((rx_status = inw(ioaddr + RxStatus)) > 0) {
		if (rx_status & 0x4000) { /* Error, update stats. */
			unsigned char rx_error = inb(ioaddr + RxErrors);
			if (vortex_debug > 2)
				printk(KERN_DEBUG " Rx error: status %2.2x.\n", rx_error);
			vp->stats.rx_errors++;
			if (rx_error & 0x01)  vp->stats.rx_over_errors++;
			if (rx_error & 0x02)  vp->stats.rx_length_errors++;
			if (rx_error & 0x04)  vp->stats.rx_frame_errors++;
			if (rx_error & 0x08)  vp->stats.rx_crc_errors++;
			if (rx_error & 0x10)  vp->stats.rx_length_errors++;
		} else {
			/* The packet length: up to 4.5K!. */
			int pkt_len = rx_status & 0x1fff;
			struct rtskb *skb;

			skb = rtnetdev_alloc_rtskb(rtdev, pkt_len + 5);
			if (vortex_debug > 4)
				printk(KERN_DEBUG "Receiving packet size %d status %4.4x.\n",
					pkt_len, rx_status);
			if (skb != NULL) {
				rtskb_reserve(skb, 2);	/* Align IP on 16 byte boundaries */
				/* 'skb_put()' points to the start of sk_buff data area. */
				if (vp->bus_master &&
					! (inw(ioaddr + Wn7_MasterStatus) & 0x8000)) {
					dma_addr_t dma = pci_map_single(vp->pdev,
									rtskb_put(skb, pkt_len),
									pkt_len, PCI_DMA_FROMDEVICE);
					outl(dma, ioaddr + Wn7_MasterAddr);
					outw((skb->len + 3) & ~3, ioaddr + Wn7_MasterLen);
					outw(StartDMAUp, ioaddr + EL3_CMD);
					while (inw(ioaddr + Wn7_MasterStatus) & 0x8000)
						;
					pci_unmap_single(vp->pdev, dma, pkt_len, PCI_DMA_FROMDEVICE);
				} else {
					insl(ioaddr + RX_FIFO, rtskb_put(skb, pkt_len),
						(pkt_len + 3) >> 2);
				}
				outw(RxDiscard, ioaddr + EL3_CMD); /* Pop top Rx packet. */
				skb->protocol = rt_eth_type_trans(skb, rtdev);
				skb->time_stamp = *time_stamp;
				rtnetif_rx(skb);
				//rtdev->last_rx = jiffies;
				vp->stats.rx_packets++;
				(*packets)++;

				/* Wait a limited time to go to next packet. */
				for (i = 200; i >= 0; i--)
					if ( ! (inw(ioaddr + EL3_STATUS) & CmdInProgress))
						break;
				continue;
			} else if (vortex_debug > 0)
				printk(KERN_NOTICE "%s: No memory to allocate a sk_buff of "
					"size %d.\n", rtdev->name, pkt_len);
		}
		vp->stats.rx_dropped++;
		issue_and_wait(rtdev, RxDiscard);
	}

	return 0;
}

static int
boomerang_rx(struct rtnet_device *rtdev, int *packets, nanosecs_abs_t *time_stamp)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	int entry = vp->cur_rx % RX_RING_SIZE;
	long ioaddr = rtdev->base_addr;
	int rx_status;
	int rx_work_limit = vp->dirty_rx + RX_RING_SIZE - vp->cur_rx;


	if (vortex_debug > 5)
		rtdm_printk(KERN_DEBUG "boomerang_rx(): status %4.4x\n", inw(ioaddr+EL3_STATUS));

	while ((rx_status = le32_to_cpu(vp->rx_ring[entry].status)) & RxDComplete){
		if (--rx_work_limit < 0)
			break;
		if (rx_status & RxDError) { /* Error, update stats. */
			unsigned char rx_error = rx_status >> 16;
			if (vortex_debug > 2)
				rtdm_printk(KERN_DEBUG " Rx error: status %2.2x.\n", rx_error);
			vp->stats.rx_errors++;
			if (rx_error & 0x01)  vp->stats.rx_over_errors++;
			if (rx_error & 0x02)  vp->stats.rx_length_errors++;
			if (rx_error & 0x04)  vp->stats.rx_frame_errors++;
			if (rx_error & 0x08)  vp->stats.rx_crc_errors++;
			if (rx_error & 0x10)  vp->stats.rx_length_errors++;
		} else {
			/* The packet length: up to 4.5K!. */
			int pkt_len = rx_status & 0x1fff;
			struct rtskb *skb;
			dma_addr_t dma = le32_to_cpu(vp->rx_ring[entry].addr);

			if (vortex_debug > 4)
				rtdm_printk(KERN_DEBUG "Receiving packet size %d status %4.4x.\n",
					pkt_len, rx_status);

			/* Check if the packet is long enough to just accept without
			   copying to a properly sized skbuff. */
			{
/*** RTnet ***/
				/* Pass up the skbuff already on the Rx ring. */
				skb = vp->rx_skbuff[entry];
				vp->rx_skbuff[entry] = NULL;
				rtskb_put(skb, pkt_len);
				pci_unmap_single(vp->pdev, dma, PKT_BUF_SZ, PCI_DMA_FROMDEVICE);
				vp->rx_nocopy++;
			}
			skb->protocol = rt_eth_type_trans(skb, rtdev);
			skb->time_stamp = *time_stamp;
			{					/* Use hardware checksum info. */
				int csum_bits = rx_status & 0xee000000;
				if (csum_bits &&
					(csum_bits == (IPChksumValid | TCPChksumValid) ||
						csum_bits == (IPChksumValid | UDPChksumValid))) {
					skb->ip_summed = CHECKSUM_UNNECESSARY;
					vp->rx_csumhits++;
				}
			}
			rtnetif_rx(skb);
			//rtdev->last_rx = jiffies;
			vp->stats.rx_packets++;
			(*packets)++;
		}
		entry = (++vp->cur_rx) % RX_RING_SIZE;
	}
	/* Refill the Rx ring buffers. */
	for (; vp->cur_rx - vp->dirty_rx > 0; vp->dirty_rx++) {
		struct rtskb *skb;
		entry = vp->dirty_rx % RX_RING_SIZE;
		if (vp->rx_skbuff[entry] == NULL) {
			skb = rtnetdev_alloc_rtskb(rtdev, PKT_BUF_SZ);
			if (skb == NULL) {
				static unsigned long last_jif;
				if ((jiffies - last_jif) > 10 * HZ) {
					rtdm_printk(KERN_WARNING "%s: memory shortage\n", rtdev->name);
					last_jif = jiffies;
				}
				if ((vp->cur_rx - vp->dirty_rx) == RX_RING_SIZE)
					{
						// *** RTnet *** mod_timer(&vp->rx_oom_timer, RUN_AT(HZ * 1));
						;
					}
				break;			/* Bad news!  */
			}
			rtskb_reserve(skb, 2);	/* Align IP on 16 byte boundaries */
			vp->rx_ring[entry].addr = cpu_to_le32(pci_map_single(vp->pdev,
													skb->tail, PKT_BUF_SZ, PCI_DMA_FROMDEVICE));
			vp->rx_skbuff[entry] = skb;
		}
		vp->rx_ring[entry].status = 0;	/* Clear complete bit. */
		outw(UpUnstall, ioaddr + EL3_CMD);
	}
	return 0;
}

/*
 * If we've hit a total OOM refilling the Rx ring we poll once a second
 * for some memory.  Otherwise there is no way to restart the rx process.
 */
static void
vortex_down(struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;

	rtnetif_stop_queue (rtdev);

	del_timer_sync(&vp->rx_oom_timer);
	del_timer_sync(&vp->timer);

	/* Turn off statistics ASAP.  We update vp->stats below. */
	outw(StatsDisable, ioaddr + EL3_CMD);

	/* Disable the receiver and transmitter. */
	outw(RxDisable, ioaddr + EL3_CMD);
	outw(TxDisable, ioaddr + EL3_CMD);

	if (rtdev->if_port == XCVR_10base2)
		/* Turn off thinnet power.  Green! */
		outw(StopCoax, ioaddr + EL3_CMD);

	outw(SetIntrEnb | 0x0000, ioaddr + EL3_CMD);

	// *** RTnet ***  update_stats(ioaddr, dev);
	if (vp->full_bus_master_rx)
		outl(0, ioaddr + UpListPtr);
	if (vp->full_bus_master_tx)
		outl(0, ioaddr + DownListPtr);

	if (vp->pdev && vp->enable_wol) {
		pci_save_state(vp->pdev, vp->power_state);
		acpi_set_WOL(rtdev);
	}
}

static int
vortex_close(struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	int i;

	// rtnet_device is always present after vortex_open was called.
	//if (netif_device_present(dev))
	//	vortex_down(dev);
	vortex_down(rtdev);

	if (vortex_debug > 1) {
		printk(KERN_DEBUG"%s: vortex_close() status %4.4x, Tx status %2.2x.\n",
			rtdev->name, inw(ioaddr + EL3_STATUS), inb(ioaddr + TxStatus));
		printk(KERN_DEBUG "%s: vortex close stats: rx_nocopy %d rx_copy %d"
			" tx_queued %d Rx pre-checksummed %d.\n",
			rtdev->name, vp->rx_nocopy, vp->rx_copy, vp->queued_packet, vp->rx_csumhits);
	}

#if DO_ZEROCOPY
	if (	vp->rx_csumhits &&
		((vp->drv_flags & HAS_HWCKSM) == 0) &&
		(hw_checksums[vp->card_idx] == -1)) {
		printk(KERN_WARNING "%s supports hardware checksums, and we're not using them!\n", rtdev->name);
		printk(KERN_WARNING "Please see http://www.uow.edu.au/~andrewm/zerocopy.html\n");
	}
#endif

	// *** RTnet ***
	if ( (i=rtdm_irq_free(&vp->irq_handle))<0 )
		return i;

	rt_stack_disconnect(rtdev);

	// *** RTnet ***

	if (vp->full_bus_master_rx) { /* Free Boomerang bus master Rx buffers. */
		for (i = 0; i < RX_RING_SIZE; i++)
			if (vp->rx_skbuff[i]) {
				pci_unmap_single(	vp->pdev, le32_to_cpu(vp->rx_ring[i].addr),
						PKT_BUF_SZ, PCI_DMA_FROMDEVICE);
				dev_kfree_rtskb(vp->rx_skbuff[i]);
				vp->rx_skbuff[i] = 0;
			}
	}
	if (vp->full_bus_master_tx) { /* Free Boomerang bus master Tx buffers. */
		for (i = 0; i < TX_RING_SIZE; i++) {
			if (vp->tx_skbuff[i]) {
				struct rtskb *skb = vp->tx_skbuff[i];
#if DO_ZEROCOPY
				int k;

				for (k=0; k<=skb_shinfo(skb)->nr_frags; k++)
					pci_unmap_single(vp->pdev,
							le32_to_cpu(vp->tx_ring[i].frag[k].addr),
							le32_to_cpu(vp->tx_ring[i].frag[k].length)&0xFFF,
							PCI_DMA_TODEVICE);
#else
				pci_unmap_single(vp->pdev, le32_to_cpu(vp->tx_ring[i].addr), skb->len, PCI_DMA_TODEVICE);
#endif
				dev_kfree_rtskb(skb);
				vp->tx_skbuff[i] = 0;
			}
		}
	}

	return 0;
}

static void
dump_tx_ring(struct rtnet_device *rtdev)
{
	if (vortex_debug > 0) {
		struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
		long ioaddr = rtdev->base_addr;

		if (vp->full_bus_master_tx) {
			int i;
			int stalled = inl(ioaddr + PktStatus) & 0x04;	/* Possible racy. But it's only debug stuff */

			rtdm_printk(KERN_ERR "	Flags; bus-master %d, dirty %d(%d) current %d(%d)\n",
				vp->full_bus_master_tx,
				vp->dirty_tx, vp->dirty_tx % TX_RING_SIZE,
				vp->cur_tx, vp->cur_tx % TX_RING_SIZE);
			rtdm_printk(KERN_ERR "	Transmit list %8.8x vs. %p.\n",
				inl(ioaddr + DownListPtr),
				&vp->tx_ring[vp->dirty_tx % TX_RING_SIZE]);
			issue_and_wait(rtdev, DownStall);
			for (i = 0; i < TX_RING_SIZE; i++) {
				rtdm_printk(KERN_ERR "	%d: @%p  length %8.8x status %8.8x\n", i,
					&vp->tx_ring[i],
#if DO_ZEROCOPY
					le32_to_cpu(vp->tx_ring[i].frag[0].length),
#else
					le32_to_cpu(vp->tx_ring[i].length),
#endif
					le32_to_cpu(vp->tx_ring[i].status));
			}
			if (!stalled)
				outw(DownUnstall, ioaddr + EL3_CMD);
		}
	}
}

static struct net_device_stats *vortex_get_stats(struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	rtdm_lockctx_t flags;

	if (rtnetif_device_present(rtdev)) {	/* AKPM: Used to be netif_running */
		rtdm_lock_get_irqsave (&vp->lock, flags);
		update_stats(rtdev->base_addr, rtdev);
		rtdm_lock_put_irqrestore (&vp->lock, flags);
	}
	return &vp->stats;
}

/*  Update statistics.
    Unlike with the EL3 we need not worry about interrupts changing
    the window setting from underneath us, but we must still guard
    against a race condition with a StatsUpdate interrupt updating the
    table.  This is done by checking that the ASM (!) code generated uses
    atomic updates with '+='.
*/
static void update_stats(long ioaddr, struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	int old_window = inw(ioaddr + EL3_CMD);

	if (old_window == 0xffff)	/* Chip suspended or ejected. */
		return;
	/* Unlike the 3c5x9 we need not turn off stats updates while reading. */
	/* Switch to the stats window, and read everything. */
	EL3WINDOW(6);
	vp->stats.tx_carrier_errors		+= inb(ioaddr + 0);
	vp->stats.tx_heartbeat_errors	+= inb(ioaddr + 1);
	/* Multiple collisions. */		inb(ioaddr + 2);
	vp->stats.collisions			+= inb(ioaddr + 3);
	vp->stats.tx_window_errors		+= inb(ioaddr + 4);
	vp->stats.rx_fifo_errors		+= inb(ioaddr + 5);
	vp->stats.tx_packets			+= inb(ioaddr + 6);
	vp->stats.tx_packets			+= (inb(ioaddr + 9)&0x30) << 4;
	/* Rx packets	*/				inb(ioaddr + 7);   /* Must read to clear */
	/* Tx deferrals */				inb(ioaddr + 8);
	/* Don't bother with register 9, an extension of registers 6&7.
	   If we do use the 6&7 values the atomic update assumption above
	   is invalid. */
	vp->stats.rx_bytes += inw(ioaddr + 10);
	vp->stats.tx_bytes += inw(ioaddr + 12);
	/* New: On the Vortex we must also clear the BadSSD counter. */
	EL3WINDOW(4);
	inb(ioaddr + 12);

	{
		u8 up = inb(ioaddr + 13);
		vp->stats.rx_bytes += (up & 0x0f) << 16;
		vp->stats.tx_bytes += (up & 0xf0) << 12;
	}

	EL3WINDOW(old_window >> 13);
	return;
}

/* Pre-Cyclone chips have no documented multicast filter, so the only
   multicast setting is to receive all multicast frames.  At least
   the chip has a very clean way to set the mode, unlike many others. */
static void set_rx_mode(struct rtnet_device *rtdev)
{
	long ioaddr = rtdev->base_addr;
	int new_mode;

	if (rtdev->flags & IFF_PROMISC) {
		if (vortex_debug > 0)
			printk(KERN_NOTICE "%s: Setting promiscuous mode.\n", rtdev->name);
		new_mode = SetRxFilter|RxStation|RxMulticast|RxBroadcast|RxProm;
	} else	if (rtdev->flags & IFF_ALLMULTI) {
		new_mode = SetRxFilter|RxStation|RxMulticast|RxBroadcast;
	} else
		new_mode = SetRxFilter | RxStation | RxBroadcast;

	outw(new_mode, ioaddr + EL3_CMD);
}

/* MII transceiver control section.
   Read and write the MII registers using software-generated serial
   MDIO protocol.  See the MII specifications or DP83840A data sheet
   for details. */

/* The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues. */
#define mdio_delay() inl(mdio_addr)

#define MDIO_SHIFT_CLK	0x01
#define MDIO_DIR_WRITE	0x04
#define MDIO_DATA_WRITE0 (0x00 | MDIO_DIR_WRITE)
#define MDIO_DATA_WRITE1 (0x02 | MDIO_DIR_WRITE)
#define MDIO_DATA_READ	0x02
#define MDIO_ENB_IN		0x00

/* Generate the preamble required for initial synchronization and
   a few older transceivers. */
static void mdio_sync(long ioaddr, int bits)
{
	long mdio_addr = ioaddr + Wn4_PhysicalMgmt;

	/* Establish sync by sending at least 32 logic ones. */
	while (-- bits >= 0) {
		outw(MDIO_DATA_WRITE1, mdio_addr);
		mdio_delay();
		outw(MDIO_DATA_WRITE1 | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
}

static int mdio_read(struct rtnet_device *rtdev, int phy_id, int location)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	int i;
	long ioaddr = rtdev->base_addr;
	int read_cmd = (0xf6 << 10) | (phy_id << 5) | location;
	unsigned int retval = 0;
	long mdio_addr = ioaddr + Wn4_PhysicalMgmt;

	spin_lock_bh(&vp->mdio_lock);

	if (mii_preamble_required)
		mdio_sync(ioaddr, 32);

	/* Shift the read command bits out. */
	for (i = 14; i >= 0; i--) {
		int dataval = (read_cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		outw(dataval, mdio_addr);
		mdio_delay();
		outw(dataval | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Read the two transition, 16 data, and wire-idle bits. */
	for (i = 19; i > 0; i--) {
		outw(MDIO_ENB_IN, mdio_addr);
		mdio_delay();
		retval = (retval << 1) | ((inw(mdio_addr) & MDIO_DATA_READ) ? 1 : 0);
		outw(MDIO_ENB_IN | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	spin_unlock_bh(&vp->mdio_lock);
	return retval & 0x20000 ? 0xffff : retval>>1 & 0xffff;
}

static void mdio_write(struct rtnet_device *rtdev, int phy_id, int location, int value)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	int write_cmd = 0x50020000 | (phy_id << 23) | (location << 18) | value;
	long mdio_addr = ioaddr + Wn4_PhysicalMgmt;
	int i;

	spin_lock_bh(&vp->mdio_lock);

	if (mii_preamble_required)
		mdio_sync(ioaddr, 32);

	/* Shift the command bits out. */
	for (i = 31; i >= 0; i--) {
		int dataval = (write_cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		outw(dataval, mdio_addr);
		mdio_delay();
		outw(dataval | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Leave the interface idle. */
	for (i = 1; i >= 0; i--) {
		outw(MDIO_ENB_IN, mdio_addr);
		mdio_delay();
		outw(MDIO_ENB_IN | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	spin_unlock_bh(&vp->mdio_lock);
	return;
}

/* ACPI: Advanced Configuration and Power Interface. */
/* Set Wake-On-LAN mode and put the board into D3 (power-down) state. */
static void acpi_set_WOL(struct rtnet_device *rtdev)
{
	struct vortex_private *vp = (struct vortex_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;

	/* Power up on: 1==Downloaded Filter, 2==Magic Packets, 4==Link Status. */
	EL3WINDOW(7);
	outw(2, ioaddr + 0x0c);
	/* The RxFilter must accept the WOL frames. */
	outw(SetRxFilter|RxStation|RxMulticast|RxBroadcast, ioaddr + EL3_CMD);
	outw(RxEnable, ioaddr + EL3_CMD);

	/* Change the power state to D3; RxEnable doesn't take effect. */
	pci_enable_wake(vp->pdev, 0, 1);
	pci_set_power_state(vp->pdev, 3);
}


static void vortex_remove_one (struct pci_dev *pdev)
{
	struct vortex_private *vp;
	// *** RTnet ***
	struct rtnet_device *rtdev = pci_get_drvdata (pdev);



	if (!rtdev) {
		printk("vortex_remove_one called for EISA device!\n");
		BUG();
	}

	vp = rtdev->priv;

	/* AKPM: FIXME: we should have
	 *	if (vp->cb_fn_base) iounmap(vp->cb_fn_base);
	 * here
	 */
	rt_unregister_rtnetdev(rtdev);
	/* Should really use issue_and_wait() here */
	outw(TotalReset|0x14, rtdev->base_addr + EL3_CMD);

	if (vp->pdev && vp->enable_wol) {
		pci_set_power_state(vp->pdev, 0);	/* Go active */
		if (vp->pm_state_valid)
			pci_restore_state(vp->pdev, vp->power_state);
	}

	pci_free_consistent(pdev,
			sizeof(struct boom_rx_desc) * RX_RING_SIZE
			+ sizeof(struct boom_tx_desc) * TX_RING_SIZE,
			vp->rx_ring,
			vp->rx_ring_dma);
	if (vp->must_free_region)
		release_region(rtdev->base_addr, vp->io_size);
	// *** RTnet ***
	rtdev_free(rtdev);
	// *** RTnet ***
}


static struct pci_driver vortex_driver = {
  name:		"3c59x_rt",
  probe:		vortex_init_one,
  remove:		vortex_remove_one,
  id_table:	vortex_pci_tbl,
#ifdef CONFIG_PM
  suspend:	NULL,
  resume:		NULL,
#endif
};


static int vortex_have_pci;


static int __init vortex_init (void)
{
	int pci_rc;

	pci_rc = pci_register_driver(&vortex_driver);

	if (pci_rc == 0)
		vortex_have_pci = 1;

	return (vortex_have_pci) ? 0 : -ENODEV;
}


static void __exit vortex_cleanup (void)
{
	if (vortex_have_pci)
		pci_unregister_driver (&vortex_driver);
}

module_init(vortex_init);
module_exit(vortex_cleanup);
