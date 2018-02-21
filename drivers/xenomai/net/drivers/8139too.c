/***
 * rt_8139too.c - Realtime driver for
 * for more information, look to end of file or '8139too.c'
 *
 * Copyright (C) 2002      Ulrich Marx <marx@kammer.uni-hannover.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

 /*
  * This Version was modified by Fabian Koch
  * It includes a different implementation of the 'cards' module parameter
  * we are using an array of integers to determine which cards to use
  * for RTnet (e.g. cards=0,1,0)
  *
  * Thanks to Jan Kiszka for this idea
  */

#define DRV_NAME            "rt_8139too"
#define DRV_VERSION         "0.9.24-rt0.7"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/ethtool.h>
#include <linux/rtnetlink.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <asm/io.h>
#include <asm/uaccess.h>

/* *** RTnet *** */
#include <rtnet_port.h>

#define MAX_UNITS               8
#define DEFAULT_RX_POOL_SIZE    16

static int cards[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = 1 };
static int media[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = -1 };
static unsigned int rx_pool_size = DEFAULT_RX_POOL_SIZE;
module_param_array(cards, int, NULL, 0444);
module_param_array(media, int, NULL, 0444);
module_param(rx_pool_size, uint, 0444);
MODULE_PARM_DESC(cards, "array of cards to be supported (e.g. 1,0,1)");
MODULE_PARM_DESC(media, "8139too: Bits 4+9: force full duplex, bit 5: 100Mbps");
MODULE_PARM_DESC(rx_pool_size, "number of receive buffers");

/* *** RTnet *** */


#define RTL8139_DRIVER_NAME   DRV_NAME " Fast Ethernet driver " DRV_VERSION
#define PFX DRV_NAME ": "

/* enable PIO instead of MMIO, if CONFIG_8139TOO_PIO is selected */
/* *** RTnet ***
#ifdef CONFIG_8139TOO_PIO
#define USE_IO_OPS 1
#endif
 *** RTnet *** */

/* Size of the in-memory receive ring. */
#define RX_BUF_LEN_IDX        2        /* 0==8K, 1==16K, 2==32K, 3==64K */
#define RX_BUF_LEN        (8192 << RX_BUF_LEN_IDX)
#define RX_BUF_PAD        16
#define RX_BUF_WRAP_PAD 2048 /* spare padding to handle lack of packet wrap */
#define RX_BUF_TOT_LEN        (RX_BUF_LEN + RX_BUF_PAD + RX_BUF_WRAP_PAD)

/* Number of Tx descriptor registers. */
#define NUM_TX_DESC        4

/* max supported ethernet frame size -- must be at least (rtdev->mtu+14+4).*/
#define MAX_ETH_FRAME_SIZE        1536

/* Size of the Tx bounce buffers -- must be at least (rtdev->mtu+14+4). */
#define TX_BUF_SIZE        MAX_ETH_FRAME_SIZE
#define TX_BUF_TOT_LEN        (TX_BUF_SIZE * NUM_TX_DESC)

/* PCI Tuning Parameters
   Threshold is bytes transferred to chip before transmission starts. */
#define TX_FIFO_THRESH 256        /* In bytes, rounded down to 32 byte units. */

/* The following settings are log_2(bytes)-4:  0 == 16 bytes .. 6==1024, 7==end of packet. */
#define RX_FIFO_THRESH        7        /* Rx buffer level before first PCI xfer.  */
#define RX_DMA_BURST        7        /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST        6        /* Maximum PCI burst, '6' is 1024 */
#define TX_RETRY        8        /* 0-15.  retries = 16 + (TX_RETRY * 16) */

/* Operational parameters that usually are not changed. */
/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT  (6*HZ)


enum {
	HAS_MII_XCVR = 0x010000,
	HAS_CHIP_XCVR = 0x020000,
	HAS_LNK_CHNG = 0x040000,
};

#define RTL_MIN_IO_SIZE 0x80
#define RTL8139B_IO_SIZE 256

#define RTL8129_CAPS        HAS_MII_XCVR
#define RTL8139_CAPS        HAS_CHIP_XCVR|HAS_LNK_CHNG

typedef enum {
	RTL8139 = 0,
	RTL8139_CB,
	SMC1211TX,
	/*MPX5030,*/
	DELTA8139,
	ADDTRON8139,
	DFE538TX,
	DFE690TXD,
	FE2000VX,
	ALLIED8139,
	RTL8129,
} board_t;


/* indexed by board_t, above */
static struct {
	const char *name;
	u32 hw_flags;
} board_info[] = {
	{ "RealTek RTL8139", RTL8139_CAPS },
	{ "RealTek RTL8129", RTL8129_CAPS },
};


static struct pci_device_id rtl8139_pci_tbl[] = {
	{0x10ec, 0x8139, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x10ec, 0x8138, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1113, 0x1211, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1500, 0x1360, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x4033, 0x1360, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1186, 0x1300, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1186, 0x1340, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x13d1, 0xab06, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1259, 0xa117, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1259, 0xa11e, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x14ea, 0xab06, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x14ea, 0xab07, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x11db, 0x1234, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1432, 0x9130, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x02ac, 0x1012, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x018a, 0x0106, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x126c, 0x1211, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x1743, 0x8139, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
	{0x021b, 0x8139, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },

#ifdef CONFIG_SH_SECUREEDGE5410
	/* Bogus 8139 silicon reports 8129 without external PROM :-( */
	{0x10ec, 0x8129, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8139 },
#endif
#ifdef CONFIG_8139TOO_8129
	{0x10ec, 0x8129, PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8129 },
#endif

	/* some crazy cards report invalid vendor ids like
	 * 0x0001 here.  The other ids are valid and constant,
	 * so we simply don't match on the main vendor id.
	 */
	{PCI_ANY_ID, 0x8139, 0x10ec, 0x8139, 0, 0, RTL8139 },
	{PCI_ANY_ID, 0x8139, 0x1186, 0x1300, 0, 0, RTL8139 },
	{PCI_ANY_ID, 0x8139, 0x13d1, 0xab06, 0, 0, RTL8139 },

	{0,}
};
MODULE_DEVICE_TABLE (pci, rtl8139_pci_tbl);

/* The rest of these values should never change. */

/* Symbolic offsets to registers. */
enum RTL8139_registers {
	MAC0 = 0,                /* Ethernet hardware address. */
	MAR0 = 8,                /* Multicast filter. */
	TxStatus0 = 0x10,        /* Transmit status (Four 32bit registers). */
	TxAddr0 = 0x20,                /* Tx descriptors (also four 32bit). */
	RxBuf = 0x30,
	ChipCmd = 0x37,
	RxBufPtr = 0x38,
	RxBufAddr = 0x3A,
	IntrMask = 0x3C,
	IntrStatus = 0x3E,
	TxConfig = 0x40,
	ChipVersion = 0x43,
	RxConfig = 0x44,
	Timer = 0x48,                /* A general-purpose counter. */
	RxMissed = 0x4C,        /* 24 bits valid, write clears. */
	Cfg9346 = 0x50,
	Config0 = 0x51,
	Config1 = 0x52,
	FlashReg = 0x54,
	MediaStatus = 0x58,
	Config3 = 0x59,
	Config4 = 0x5A,                /* absent on RTL-8139A */
	HltClk = 0x5B,
	MultiIntr = 0x5C,
	TxSummary = 0x60,
	BasicModeCtrl = 0x62,
	BasicModeStatus = 0x64,
	NWayAdvert = 0x66,
	NWayLPAR = 0x68,
	NWayExpansion = 0x6A,
	/* Undocumented registers, but required for proper operation. */
	FIFOTMS = 0x70,                /* FIFO Control and test. */
	CSCR = 0x74,                /* Chip Status and Configuration Register. */
	PARA78 = 0x78,
	PARA7c = 0x7c,                /* Magic transceiver parameter register. */
	Config5 = 0xD8,                /* absent on RTL-8139A */
};

enum ClearBitMasks {
	MultiIntrClear = 0xF000,
	ChipCmdClear = 0xE2,
	Config1Clear = (1<<7)|(1<<6)|(1<<3)|(1<<2)|(1<<1),
};

enum ChipCmdBits {
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,
};

/* Interrupt register bits, using my own meaningful names. */
enum IntrStatusBits {
	PCIErr = 0x8000,
	PCSTimeout = 0x4000,
	RxFIFOOver = 0x40,
	RxUnderrun = 0x20,
	RxOverflow = 0x10,
	TxErr = 0x08,
	TxOK = 0x04,
	RxErr = 0x02,
	RxOK = 0x01,

	RxAckBits = RxFIFOOver | RxOverflow | RxOK,
};

enum TxStatusBits {
	TxHostOwns = 0x2000,
	TxUnderrun = 0x4000,
	TxStatOK = 0x8000,
	TxOutOfWindow = 0x20000000,
	TxAborted = 0x40000000,
	TxCarrierLost = 0x80000000,
};
enum RxStatusBits {
	RxMulticast = 0x8000,
	RxPhysical = 0x4000,
	RxBroadcast = 0x2000,
	RxBadSymbol = 0x0020,
	RxRunt = 0x0010,
	RxTooLong = 0x0008,
	RxCRCErr = 0x0004,
	RxBadAlign = 0x0002,
	RxStatusOK = 0x0001,
};

/* Bits in RxConfig. */
enum rx_mode_bits {
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,
};

/* Bits in TxConfig. */
enum tx_config_bits {

	/* Interframe Gap Time. Only TxIFG96 doesn't violate IEEE 802.3 */
	TxIFGShift = 24,
	TxIFG84 = (0 << TxIFGShift),    /* 8.4us / 840ns (10 / 100Mbps) */
	TxIFG88 = (1 << TxIFGShift),    /* 8.8us / 880ns (10 / 100Mbps) */
	TxIFG92 = (2 << TxIFGShift),    /* 9.2us / 920ns (10 / 100Mbps) */
	TxIFG96 = (3 << TxIFGShift),    /* 9.6us / 960ns (10 / 100Mbps) */

	TxLoopBack = (1 << 18) | (1 << 17), /* enable loopback test mode */
	TxCRC = (1 << 16),        /* DISABLE appending CRC to end of Tx packets */
	TxClearAbt = (1 << 0),        /* Clear abort (WO) */
	TxDMAShift = 8,                /* DMA burst value (0-7) is shifted this many bits */
	TxRetryShift = 4,        /* TXRR value (0-15) is shifted this many bits */

	TxVersionMask = 0x7C800000, /* mask out version bits 30-26, 23 */
};

/* Bits in Config1 */
enum Config1Bits {
	Cfg1_PM_Enable = 0x01,
	Cfg1_VPD_Enable = 0x02,
	Cfg1_PIO = 0x04,
	Cfg1_MMIO = 0x08,
	LWAKE = 0x10,                /* not on 8139, 8139A */
	Cfg1_Driver_Load = 0x20,
	Cfg1_LED0 = 0x40,
	Cfg1_LED1 = 0x80,
	SLEEP = (1 << 1),        /* only on 8139, 8139A */
	PWRDN = (1 << 0),        /* only on 8139, 8139A */
};

/* Bits in Config3 */
enum Config3Bits {
	Cfg3_FBtBEn    = (1 << 0), /* 1 = Fast Back to Back */
	Cfg3_FuncRegEn = (1 << 1), /* 1 = enable CardBus Function registers */
	Cfg3_CLKRUN_En = (1 << 2), /* 1 = enable CLKRUN */
	Cfg3_CardB_En  = (1 << 3), /* 1 = enable CardBus registers */
	Cfg3_LinkUp    = (1 << 4), /* 1 = wake up on link up */
	Cfg3_Magic     = (1 << 5), /* 1 = wake up on Magic Packet (tm) */
	Cfg3_PARM_En   = (1 << 6), /* 0 = software can set twister parameters */
	Cfg3_GNTSel    = (1 << 7), /* 1 = delay 1 clock from PCI GNT signal */
};

/* Bits in Config4 */
enum Config4Bits {
	LWPTN = (1 << 2),        /* not on 8139, 8139A */
};

/* Bits in Config5 */
enum Config5Bits {
	Cfg5_PME_STS     = (1 << 0), /* 1 = PCI reset resets PME_Status */
	Cfg5_LANWake     = (1 << 1), /* 1 = enable LANWake signal */
	Cfg5_LDPS        = (1 << 2), /* 0 = save power when link is down */
	Cfg5_FIFOAddrPtr = (1 << 3), /* Realtek internal SRAM testing */
	Cfg5_UWF         = (1 << 4), /* 1 = accept unicast wakeup frame */
	Cfg5_MWF         = (1 << 5), /* 1 = accept multicast wakeup frame */
	Cfg5_BWF         = (1 << 6), /* 1 = accept broadcast wakeup frame */
};

enum RxConfigBits {
	/* rx fifo threshold */
	RxCfgFIFOShift = 13,
	RxCfgFIFONone = (7 << RxCfgFIFOShift),

	/* Max DMA burst */
	RxCfgDMAShift = 8,
	RxCfgDMAUnlimited = (7 << RxCfgDMAShift),

	/* rx ring buffer length */
	RxCfgRcv8K = 0,
	RxCfgRcv16K = (1 << 11),
	RxCfgRcv32K = (1 << 12),
	RxCfgRcv64K = (1 << 11) | (1 << 12),

	/* Disable packet wrap at end of Rx buffer */
	RxNoWrap = (1 << 7),
};


/* Twister tuning parameters from RealTek.
   Completely undocumented, but required to tune bad links. */
enum CSCRBits {
	CSCR_LinkOKBit = 0x0400,
	CSCR_LinkChangeBit = 0x0800,
	CSCR_LinkStatusBits = 0x0f000,
	CSCR_LinkDownOffCmd = 0x003c0,
	CSCR_LinkDownCmd = 0x0f3c0,
};


enum Cfg9346Bits {
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,
};


#define PARA78_default        0x78fa8388
#define PARA7c_default        0xcb38de43        /* param[0][3] */
#define PARA7c_xxx                0xcb38de43
/*static const unsigned long param[4][4] = {
	{0xcb39de43, 0xcb39ce43, 0xfb38de03, 0xcb38de43},
	{0xcb39de43, 0xcb39ce43, 0xcb39ce83, 0xcb39ce83},
	{0xcb39de43, 0xcb39ce43, 0xcb39ce83, 0xcb39ce83},
	{0xbb39de43, 0xbb39ce43, 0xbb39ce83, 0xbb39ce83}
};*/

typedef enum {
	CH_8139 = 0,
	CH_8139_K,
	CH_8139A,
	CH_8139B,
	CH_8130,
	CH_8139C,
} chip_t;

enum chip_flags {
	HasHltClk = (1 << 0),
	HasLWake = (1 << 1),
};


/* directly indexed by chip_t, above */
const static struct {
	const char *name;
	u8 version; /* from RTL8139C docs */
	u32 flags;
} rtl_chip_info[] = {
	{ "RTL-8139",
	  0x40,
	  HasHltClk,
	},

	{ "RTL-8139 rev K",
	  0x60,
	  HasHltClk,
	},

	{ "RTL-8139A",
	  0x70,
	  HasHltClk, /* XXX undocumented? */
	},

	{ "RTL-8139A rev G",
	  0x72,
	  HasHltClk, /* XXX undocumented? */
	},

	{ "RTL-8139B",
	  0x78,
	  HasLWake,
	},

	{ "RTL-8130",
	  0x7C,
	  HasLWake,
	},

	{ "RTL-8139C",
	  0x74,
	  HasLWake,
	},

	{ "RTL-8100",
	  0x7A,
	  HasLWake,
	 },

	{ "RTL-8100B/8139D",
	  0x75,
	  HasHltClk /* XXX undocumented? */
	  | HasLWake,
	},

	{ "RTL-8101",
	  0x77,
	  HasLWake,
	},
};

struct rtl_extra_stats {
	unsigned long early_rx;
	unsigned long tx_buf_mapped;
	unsigned long tx_timeouts;
	unsigned long rx_lost_in_ring;
};

struct rtl8139_private {
	void *mmio_addr;
	int drv_flags;
	struct pci_dev *pci_dev;
	struct net_device_stats stats;
	unsigned char *rx_ring;
	unsigned int cur_rx;        /* Index into the Rx buffer of next Rx pkt. */
	unsigned int tx_flag;
	unsigned long cur_tx;
	unsigned long dirty_tx;
	unsigned char *tx_buf[NUM_TX_DESC];        /* Tx bounce buffers */
	unsigned char *tx_bufs;        /* Tx bounce buffer region. */
	dma_addr_t rx_ring_dma;
	dma_addr_t tx_bufs_dma;
	signed char phys[4];                /* MII device addresses. */
	char twistie, twist_row, twist_col;        /* Twister tune state. */
	unsigned int default_port:4;        /* Last rtdev->if_port value. */
	unsigned int medialock:1;        /* Don't sense media type. */
	rtdm_lock_t lock;
	chip_t chipset;
	pid_t thr_pid;
	u32 rx_config;
	struct rtl_extra_stats xstats;
	int time_to_die;
	struct mii_if_info mii;
	rtdm_irq_t irq_handle;
};

MODULE_AUTHOR ("Jeff Garzik <jgarzik@mandrakesoft.com>");
MODULE_DESCRIPTION ("RealTek RTL-8139 Fast Ethernet driver");
MODULE_LICENSE("GPL");

static int read_eeprom (void *ioaddr, int location, int addr_len);
static int mdio_read (struct rtnet_device *rtdev, int phy_id, int location);
static void mdio_write (struct rtnet_device *rtdev, int phy_id, int location, int val);


static int rtl8139_open (struct rtnet_device *rtdev);
static int rtl8139_close (struct rtnet_device *rtdev);
static int rtl8139_interrupt (rtdm_irq_t *irq_handle);
static int rtl8139_start_xmit (struct rtskb *skb, struct rtnet_device *rtdev);

static int rtl8139_ioctl(struct rtnet_device *rtdev, unsigned int request, void *cmd);
static struct net_device_stats *rtl8139_get_stats(struct rtnet_device*rtdev);

static void rtl8139_init_ring (struct rtnet_device *rtdev);
static void rtl8139_set_rx_mode (struct rtnet_device *rtdev);
static void __set_rx_mode (struct rtnet_device *rtdev);
static void rtl8139_hw_start (struct rtnet_device *rtdev);

#ifdef USE_IO_OPS

#define RTL_R8(reg)                inb (((unsigned long)ioaddr) + (reg))
#define RTL_R16(reg)                inw (((unsigned long)ioaddr) + (reg))
#define RTL_R32(reg)                inl (((unsigned long)ioaddr) + (reg))
#define RTL_W8(reg, val8)        outb ((val8), ((unsigned long)ioaddr) + (reg))
#define RTL_W16(reg, val16)        outw ((val16), ((unsigned long)ioaddr) + (reg))
#define RTL_W32(reg, val32)        outl ((val32), ((unsigned long)ioaddr) + (reg))
#define RTL_W8_F                RTL_W8
#define RTL_W16_F                RTL_W16
#define RTL_W32_F                RTL_W32
#undef readb
#undef readw
#undef readl
#undef writeb
#undef writew
#undef writel
#define readb(addr) inb((unsigned long)(addr))
#define readw(addr) inw((unsigned long)(addr))
#define readl(addr) inl((unsigned long)(addr))
#define writeb(val,addr) outb((val),(unsigned long)(addr))
#define writew(val,addr) outw((val),(unsigned long)(addr))
#define writel(val,addr) outl((val),(unsigned long)(addr))

#else

/* write MMIO register, with flush */
/* Flush avoids rtl8139 bug w/ posted MMIO writes */
#define RTL_W8_F(reg, val8)        do { writeb ((val8), ioaddr + (reg)); readb (ioaddr + (reg)); } while (0)
#define RTL_W16_F(reg, val16)        do { writew ((val16), ioaddr + (reg)); readw (ioaddr + (reg)); } while (0)
#define RTL_W32_F(reg, val32)        do { writel ((val32), ioaddr + (reg)); readl (ioaddr + (reg)); } while (0)


#define MMIO_FLUSH_AUDIT_COMPLETE 1
#if MMIO_FLUSH_AUDIT_COMPLETE

/* write MMIO register */
#define RTL_W8(reg, val8)        writeb ((val8), ioaddr + (reg))
#define RTL_W16(reg, val16)        writew ((val16), ioaddr + (reg))
#define RTL_W32(reg, val32)        writel ((val32), ioaddr + (reg))

#else

/* write MMIO register, then flush */
#define RTL_W8                RTL_W8_F
#define RTL_W16                RTL_W16_F
#define RTL_W32                RTL_W32_F

#endif /* MMIO_FLUSH_AUDIT_COMPLETE */

/* read MMIO register */
#define RTL_R8(reg)                readb (ioaddr + (reg))
#define RTL_R16(reg)                readw (ioaddr + (reg))
#define RTL_R32(reg)                readl (ioaddr + (reg))

#endif /* USE_IO_OPS */


static const u16 rtl8139_intr_mask =
	PCIErr | PCSTimeout | RxUnderrun | RxOverflow | RxFIFOOver |
	TxErr | TxOK | RxErr | RxOK;

static const unsigned int rtl8139_rx_config =
	RxCfgRcv32K | RxNoWrap |
	(RX_FIFO_THRESH << RxCfgFIFOShift) |
	(RX_DMA_BURST << RxCfgDMAShift);

static const unsigned int rtl8139_tx_config =
	TxIFG96 | (TX_DMA_BURST << TxDMAShift) | (TX_RETRY << TxRetryShift);




static void rtl8139_chip_reset (void *ioaddr)
{
	int i;

	/* Soft reset the chip. */
	RTL_W8 (ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--) {
		barrier();
		if ((RTL_R8 (ChipCmd) & CmdReset) == 0)
			break;
		udelay (10);
	}
}


static int rtl8139_init_board (struct pci_dev *pdev,
					 struct rtnet_device **dev_out)
{
	void *ioaddr;
	struct rtnet_device *rtdev;
	struct rtl8139_private *tp;
	u8 tmp8;
	int rc;
	unsigned int i;
#ifdef USE_IO_OPS
	u32 pio_start, pio_end, pio_flags, pio_len;
#endif
	unsigned long mmio_start, mmio_flags, mmio_len;
	u32 tmp;


	*dev_out = NULL;

	/* dev and rtdev->priv zeroed in alloc_etherdev */
	rtdev=rt_alloc_etherdev(sizeof (struct rtl8139_private),
				rx_pool_size + NUM_TX_DESC);
	if (rtdev==NULL) {
		rtdm_printk (KERN_ERR PFX "%s: Unable to alloc new net device\n", pci_name(pdev));
		return -ENOMEM;
	}
	rtdev_alloc_name(rtdev, "rteth%d");

	rt_rtdev_connect(rtdev, &RTDEV_manager);

	rtdev->vers = RTDEV_VERS_2_0;
	tp = rtdev->priv;
	tp->pci_dev = pdev;

	/* enable device (incl. PCI PM wakeup and hotplug setup) */
	rc = pci_enable_device (pdev);
	if (rc)
		goto err_out;

	rc = pci_request_regions (pdev, "rtnet8139too");
	if (rc)
		goto err_out;

	/* enable PCI bus-mastering */
	pci_set_master (pdev);

	mmio_start = pci_resource_start (pdev, 1);
	mmio_flags = pci_resource_flags (pdev, 1);
	mmio_len = pci_resource_len (pdev, 1);

	/* set this immediately, we need to know before
	 * we talk to the chip directly */
#ifdef USE_IO_OPS
	pio_start = pci_resource_start (pdev, 0);
	pio_end = pci_resource_end (pdev, 0);
	pio_flags = pci_resource_flags (pdev, 0);
	pio_len = pci_resource_len (pdev, 0);

	/* make sure PCI base addr 0 is PIO */
	if (!(pio_flags & IORESOURCE_IO)) {
		rtdm_printk (KERN_ERR PFX "%s: region #0 not a PIO resource, aborting\n", pci_name(pdev));
		rc = -ENODEV;
		goto err_out;
	}
	/* check for weird/broken PCI region reporting */
	if (pio_len < RTL_MIN_IO_SIZE) {
		rtdm_printk (KERN_ERR PFX "%s: Invalid PCI I/O region size(s), aborting\n", pci_name(pdev));
		rc = -ENODEV;
		goto err_out;
	}
#else
	/* make sure PCI base addr 1 is MMIO */
	if (!(mmio_flags & IORESOURCE_MEM)) {
		rtdm_printk(KERN_ERR PFX "%s: region #1 not an MMIO resource, aborting\n", pci_name(pdev));
		rc = -ENODEV;
		goto err_out;
	}
	if (mmio_len < RTL_MIN_IO_SIZE) {
		rtdm_printk(KERN_ERR PFX "%s: Invalid PCI mem region size(s), aborting\n", pci_name(pdev));
		rc = -ENODEV;
		goto err_out;
	}
#endif

#ifdef USE_IO_OPS
	ioaddr = (void *) pio_start;
	rtdev->base_addr = pio_start;
	tp->mmio_addr = ioaddr;
#else
	/* ioremap MMIO region */
	ioaddr = ioremap (mmio_start, mmio_len);
	if (ioaddr == NULL) {
		rtdm_printk(KERN_ERR PFX "%s: cannot remap MMIO, aborting\n", pci_name(pdev));
		rc = -EIO;
		goto err_out;
	}
	rtdev->base_addr = (long) ioaddr;
	tp->mmio_addr = ioaddr;
#endif /* USE_IO_OPS */

	/* Bring old chips out of low-power mode. */
	RTL_W8 (HltClk, 'R');

	/* check for missing/broken hardware */
	if (RTL_R32 (TxConfig) == 0xFFFFFFFF) {
		rtdm_printk(KERN_ERR PFX "%s: Chip not responding, ignoring board\n", pci_name(pdev));
		rc = -EIO;
		goto err_out;
	}

	/* identify chip attached to board */
	tmp = RTL_R8 (ChipVersion);
	for (i = 0; i < ARRAY_SIZE (rtl_chip_info); i++)
		if (tmp == rtl_chip_info[i].version) {
			tp->chipset = i;
			goto match;
		}

	rtdm_printk("rt8139too: unknown chip version, assuming RTL-8139\n");
	rtdm_printk("rt8139too: TxConfig = 0x%08x\n", RTL_R32 (TxConfig));

	tp->chipset = 0;

match:
	if (tp->chipset >= CH_8139B) {
		u8 new_tmp8 = tmp8 = RTL_R8 (Config1);
		if ((rtl_chip_info[tp->chipset].flags & HasLWake) &&
		    (tmp8 & LWAKE))
			new_tmp8 &= ~LWAKE;
		new_tmp8 |= Cfg1_PM_Enable;
		if (new_tmp8 != tmp8) {
			RTL_W8 (Cfg9346, Cfg9346_Unlock);
			RTL_W8 (Config1, tmp8);
			RTL_W8 (Cfg9346, Cfg9346_Lock);
		}
		if (rtl_chip_info[tp->chipset].flags & HasLWake) {
			tmp8 = RTL_R8 (Config4);
			if (tmp8 & LWPTN) {
				RTL_W8 (Cfg9346, Cfg9346_Unlock);
				RTL_W8 (Config4, tmp8 & ~LWPTN);
				RTL_W8 (Cfg9346, Cfg9346_Lock);
			}
		}
	} else {
		tmp8 = RTL_R8 (Config1);
		tmp8 &= ~(SLEEP | PWRDN);
		RTL_W8 (Config1, tmp8);
	}

	rtl8139_chip_reset (ioaddr);

	*dev_out = rtdev;
	return 0;

err_out:
#ifndef USE_IO_OPS
	if (tp->mmio_addr) iounmap (tp->mmio_addr);
#endif /* !USE_IO_OPS */
	/* it's ok to call this even if we have no regions to free */
	pci_release_regions (pdev);
	rtdev_free(rtdev);
	pci_set_drvdata (pdev, NULL);

	return rc;
}




static int rtl8139_init_one (struct pci_dev *pdev,
				       const struct pci_device_id *ent)
{
	struct rtnet_device *rtdev = NULL;
	struct rtl8139_private *tp;
	int i, addr_len;
	int option;
	void *ioaddr;
	static int board_idx = -1;

	board_idx++;

	if( cards[board_idx] == 0)
		return -ENODEV;

	/* when we're built into the kernel, the driver version message
	 * is only printed if at least one 8139 board has been found
	 */
#ifndef MODULE
	{
		static int printed_version;
		if (!printed_version++)
			rtdm_printk (KERN_INFO RTL8139_DRIVER_NAME "\n");
	}
#endif

	if ((i=rtl8139_init_board (pdev, &rtdev)) < 0)
		return i;


	tp = rtdev->priv;
	ioaddr = tp->mmio_addr;

	addr_len = read_eeprom (ioaddr, 0, 8) == 0x8129 ? 8 : 6;
	for (i = 0; i < 3; i++)
		((u16 *) (rtdev->dev_addr))[i] =
		    le16_to_cpu (read_eeprom (ioaddr, i + 7, addr_len));

	/* The Rtl8139-specific entries in the device structure. */
	rtdev->open = rtl8139_open;
	rtdev->stop = rtl8139_close;
	rtdev->hard_header = &rt_eth_header;
	rtdev->hard_start_xmit = rtl8139_start_xmit;
	rtdev->do_ioctl = rtl8139_ioctl;
	rtdev->get_stats = rtl8139_get_stats;

	/*rtdev->set_multicast_list = rtl8139_set_rx_mode; */
	rtdev->features |= NETIF_F_SG|NETIF_F_HW_CSUM;

	rtdev->irq = pdev->irq;

	/* rtdev->priv/tp zeroed and aligned in init_etherdev */
	tp = rtdev->priv;

	/* note: tp->chipset set in rtl8139_init_board */
	tp->drv_flags = board_info[ent->driver_data].hw_flags;
	tp->mmio_addr = ioaddr;
	rtdm_lock_init (&tp->lock);

	if ( (i=rt_register_rtnetdev(rtdev)) )
		goto err_out;

	pci_set_drvdata (pdev, rtdev);

	tp->phys[0] = 32;

	/* The lower four bits are the media type. */
	option = (board_idx >= MAX_UNITS) ? 0 : media[board_idx];
	if (option > 0) {
		tp->mii.full_duplex = (option & 0x210) ? 1 : 0;
		tp->default_port = option & 0xFF;
		if (tp->default_port)
			tp->medialock = 1;
	}
	if (tp->default_port) {
		rtdm_printk(KERN_INFO "  Forcing %dMbps %s-duplex operation.\n",
			    (option & 0x20 ? 100 : 10),
			    (option & 0x10 ? "full" : "half"));
		mdio_write(rtdev, tp->phys[0], 0,
				   ((option & 0x20) ? 0x2000 : 0) |         /* 100Mbps? */
				   ((option & 0x10) ? 0x0100 : 0)); /* Full duplex? */
	}


	/* Put the chip into low-power mode. */
	if (rtl_chip_info[tp->chipset].flags & HasHltClk)
		RTL_W8 (HltClk, 'H');        /* 'R' would leave the clock running. */

	return 0;


err_out:
#ifndef USE_IO_OPS
	if (tp->mmio_addr) iounmap (tp->mmio_addr);
#endif /* !USE_IO_OPS */
	/* it's ok to call this even if we have no regions to free */
	pci_release_regions (pdev);
	rtdev_free(rtdev);
	pci_set_drvdata (pdev, NULL);

	return i;
}


static void rtl8139_remove_one (struct pci_dev *pdev)
{
	struct rtnet_device *rtdev = pci_get_drvdata(pdev);

#ifndef USE_IO_OPS
	struct rtl8139_private *tp = rtdev->priv;

	if (tp->mmio_addr)
		iounmap (tp->mmio_addr);
#endif /* !USE_IO_OPS */

	/* it's ok to call this even if we have no regions to free */
	rt_unregister_rtnetdev(rtdev);
	rt_rtdev_disconnect(rtdev);

	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);

	rtdev_free(rtdev);
}


/* Serial EEPROM section. */

/*  EEPROM_Ctrl bits. */
#define EE_SHIFT_CLK        0x04        /* EEPROM shift clock. */
#define EE_CS                        0x08        /* EEPROM chip select. */
#define EE_DATA_WRITE        0x02        /* EEPROM chip data in. */
#define EE_WRITE_0                0x00
#define EE_WRITE_1                0x02
#define EE_DATA_READ        0x01        /* EEPROM chip data out. */
#define EE_ENB                        (0x80 | EE_CS)

/* Delay between EEPROM clock transitions.
   No extra delay is needed with 33Mhz PCI, but 66Mhz may change this.
 */

#define eeprom_delay()        readl(ee_addr)

/* The EEPROM commands include the alway-set leading bit. */
#define EE_WRITE_CMD        (5)
#define EE_READ_CMD                (6)
#define EE_ERASE_CMD        (7)

static int read_eeprom (void *ioaddr, int location, int addr_len)
{
	int i;
	unsigned retval = 0;
	void *ee_addr = ioaddr + Cfg9346;
	int read_cmd = location | (EE_READ_CMD << addr_len);

	writeb (EE_ENB & ~EE_CS, ee_addr);
	writeb (EE_ENB, ee_addr);
	eeprom_delay ();

	/* Shift the read command bits out. */
	for (i = 4 + addr_len; i >= 0; i--) {
		int dataval = (read_cmd & (1 << i)) ? EE_DATA_WRITE : 0;
		writeb (EE_ENB | dataval, ee_addr);
		eeprom_delay ();
		writeb (EE_ENB | dataval | EE_SHIFT_CLK, ee_addr);
		eeprom_delay ();
	}
	writeb (EE_ENB, ee_addr);
	eeprom_delay ();

	for (i = 16; i > 0; i--) {
		writeb (EE_ENB | EE_SHIFT_CLK, ee_addr);
		eeprom_delay ();
		retval =
		    (retval << 1) | ((readb (ee_addr) & EE_DATA_READ) ? 1 :
				     0);
		writeb (EE_ENB, ee_addr);
		eeprom_delay ();
	}

	/* Terminate the EEPROM access. */
	writeb (~EE_CS, ee_addr);
	eeprom_delay ();

	return retval;
}

/* MII serial management: mostly bogus for now. */
/* Read and write the MII management registers using software-generated
   serial MDIO protocol.
   The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues. */
#define MDIO_DIR                0x80
#define MDIO_DATA_OUT        0x04
#define MDIO_DATA_IN        0x02
#define MDIO_CLK                0x01
#define MDIO_WRITE0 (MDIO_DIR)
#define MDIO_WRITE1 (MDIO_DIR | MDIO_DATA_OUT)

#define mdio_delay(mdio_addr)        readb(mdio_addr)



static char mii_2_8139_map[8] = {
	BasicModeCtrl,
	BasicModeStatus,
	0,
	0,
	NWayAdvert,
	NWayLPAR,
	NWayExpansion,
	0
};

#ifdef CONFIG_8139TOO_8129
/* Syncronize the MII management interface by shifting 32 one bits out. */
static void mdio_sync (void *mdio_addr)
{
	int i;

	for (i = 32; i >= 0; i--) {
		writeb (MDIO_WRITE1, mdio_addr);
		mdio_delay (mdio_addr);
		writeb (MDIO_WRITE1 | MDIO_CLK, mdio_addr);
		mdio_delay (mdio_addr);
	}
}
#endif


static int mdio_read (struct rtnet_device *rtdev, int phy_id, int location)
{
	struct rtl8139_private *tp = rtdev->priv;
	int retval = 0;
#ifdef CONFIG_8139TOO_8129
	void *mdio_addr = tp->mmio_addr + Config4;
	int mii_cmd = (0xf6 << 10) | (phy_id << 5) | location;
	int i;
#endif

	if (phy_id > 31) {        /* Really a 8139.  Use internal registers. */
		return location < 8 && mii_2_8139_map[location] ?
		    readw (tp->mmio_addr + mii_2_8139_map[location]) : 0;
	}

#ifdef CONFIG_8139TOO_8129
	mdio_sync (mdio_addr);
	/* Shift the read command bits out. */
	for (i = 15; i >= 0; i--) {
		int dataval = (mii_cmd & (1 << i)) ? MDIO_DATA_OUT : 0;

		writeb (MDIO_DIR | dataval, mdio_addr);
		mdio_delay (mdio_addr);
		writeb (MDIO_DIR | dataval | MDIO_CLK, mdio_addr);
		mdio_delay (mdio_addr);
	}

	/* Read the two transition, 16 data, and wire-idle bits. */
	for (i = 19; i > 0; i--) {
		writeb (0, mdio_addr);
		mdio_delay (mdio_addr);
		retval = (retval << 1) | ((readb (mdio_addr) & MDIO_DATA_IN) ? 1 : 0);
		writeb (MDIO_CLK, mdio_addr);
		mdio_delay (mdio_addr);
	}
#endif

	return (retval >> 1) & 0xffff;
}


static void mdio_write (struct rtnet_device *rtdev, int phy_id, int location,
			int value)
{
	struct rtl8139_private *tp = rtdev->priv;
#ifdef CONFIG_8139TOO_8129
	void *mdio_addr = tp->mmio_addr + Config4;
	int mii_cmd = (0x5002 << 16) | (phy_id << 23) | (location << 18) | value;
	int i;
#endif

	if (phy_id > 31) {        /* Really a 8139.  Use internal registers. */
		void *ioaddr = tp->mmio_addr;
		if (location == 0) {
			RTL_W8 (Cfg9346, Cfg9346_Unlock);
			RTL_W16 (BasicModeCtrl, value);
			RTL_W8 (Cfg9346, Cfg9346_Lock);
		} else if (location < 8 && mii_2_8139_map[location])
			RTL_W16 (mii_2_8139_map[location], value);
		return;
	}

#ifdef CONFIG_8139TOO_8129
	mdio_sync (mdio_addr);

	/* Shift the command bits out. */
	for (i = 31; i >= 0; i--) {
		int dataval =
		    (mii_cmd & (1 << i)) ? MDIO_WRITE1 : MDIO_WRITE0;
		writeb (dataval, mdio_addr);
		mdio_delay (mdio_addr);
		writeb (dataval | MDIO_CLK, mdio_addr);
		mdio_delay (mdio_addr);
	}
	/* Clear out extra bits. */
	for (i = 2; i > 0; i--) {
		writeb (0, mdio_addr);
		mdio_delay (mdio_addr);
		writeb (MDIO_CLK, mdio_addr);
		mdio_delay (mdio_addr);
	}
#endif
}

static int rtl8139_open (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	int retval;

	rt_stack_connect(rtdev, &STACK_manager);

	retval = rtdm_irq_request(&tp->irq_handle, rtdev->irq,
				  rtl8139_interrupt, RTDM_IRQTYPE_SHARED,
				  rtdev->name, rtdev);
	if (retval)
		return retval;

	tp->tx_bufs = pci_alloc_consistent(tp->pci_dev, TX_BUF_TOT_LEN, &tp->tx_bufs_dma);
	tp->rx_ring = pci_alloc_consistent(tp->pci_dev, RX_BUF_TOT_LEN, &tp->rx_ring_dma);

	if (tp->tx_bufs == NULL || tp->rx_ring == NULL) {
		rtdm_irq_free(&tp->irq_handle);
		if (tp->tx_bufs)
			pci_free_consistent(tp->pci_dev, TX_BUF_TOT_LEN, tp->tx_bufs, tp->tx_bufs_dma);
		if (tp->rx_ring)
			pci_free_consistent(tp->pci_dev, RX_BUF_TOT_LEN, tp->rx_ring, tp->rx_ring_dma);

		return -ENOMEM;
	}
	/* FIXME: create wrapper for duplex_lock vs. force_media
	   tp->mii.full_duplex = tp->mii.duplex_lock; */
	tp->tx_flag = (TX_FIFO_THRESH << 11) & 0x003f0000;
	tp->twistie = 1;
	tp->time_to_die = 0;

	rtl8139_init_ring (rtdev);
	rtl8139_hw_start (rtdev);

	return 0;
}


static void rtl_check_media (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	u16 mii_lpa;

	if (tp->phys[0] < 0)
		return;

	mii_lpa = mdio_read(rtdev, tp->phys[0], MII_LPA);
	if (mii_lpa == 0xffff)
		return;

	tp->mii.full_duplex = (mii_lpa & LPA_100FULL) == LPA_100FULL ||
		(mii_lpa & 0x00C0) == LPA_10FULL;
}


/* Start the hardware at open or resume. */
static void rtl8139_hw_start (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	void *ioaddr = tp->mmio_addr;
	u32 i;
	u8 tmp;

	/* Bring old chips out of low-power mode. */
	if (rtl_chip_info[tp->chipset].flags & HasHltClk)
		RTL_W8 (HltClk, 'R');

	rtl8139_chip_reset(ioaddr);

	/* unlock Config[01234] and BMCR register writes */
	RTL_W8_F (Cfg9346, Cfg9346_Unlock);
	/* Restore our idea of the MAC address. */
	RTL_W32_F (MAC0 + 0, cpu_to_le32 (*(u32 *) (rtdev->dev_addr + 0)));
	RTL_W32_F (MAC0 + 4, cpu_to_le32 (*(u32 *) (rtdev->dev_addr + 4)));

	tp->cur_rx = 0;

	/* init Rx ring buffer DMA address */
	RTL_W32_F (RxBuf, tp->rx_ring_dma);

	/* Must enable Tx/Rx before setting transfer thresholds! */
	RTL_W8 (ChipCmd, CmdRxEnb | CmdTxEnb);

	tp->rx_config = rtl8139_rx_config | AcceptBroadcast | AcceptMyPhys;
	RTL_W32 (RxConfig, tp->rx_config);

	/* Check this value: the documentation for IFG contradicts ifself. */
	RTL_W32 (TxConfig, rtl8139_tx_config);

	rtl_check_media (rtdev);

	if (tp->chipset >= CH_8139B) {
		/* Disable magic packet scanning, which is enabled
		 * when PM is enabled in Config1.  It can be reenabled
		 * via ETHTOOL_SWOL if desired.  */
		RTL_W8 (Config3, RTL_R8 (Config3) & ~Cfg3_Magic);
	}

	/* Lock Config[01234] and BMCR register writes */
	RTL_W8 (Cfg9346, Cfg9346_Lock);

	/* init Tx buffer DMA addresses */
	for (i = 0; i < NUM_TX_DESC; i++)
		RTL_W32_F (TxAddr0 + (i * 4), tp->tx_bufs_dma + (tp->tx_buf[i] - tp->tx_bufs));

	RTL_W32 (RxMissed, 0);

	rtl8139_set_rx_mode (rtdev);

	/* no early-rx interrupts */
	RTL_W16 (MultiIntr, RTL_R16 (MultiIntr) & MultiIntrClear);

	/* make sure RxTx has started */
	tmp = RTL_R8 (ChipCmd);
	if ((!(tmp & CmdRxEnb)) || (!(tmp & CmdTxEnb)))
		RTL_W8 (ChipCmd, CmdRxEnb | CmdTxEnb);

	/* Enable all known interrupts by setting the interrupt mask. */
	RTL_W16 (IntrMask, rtl8139_intr_mask);

	rtnetif_start_queue (rtdev);
}


/* Initialize the Rx and Tx rings, along with various 'dev' bits. */
static void rtl8139_init_ring (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	int i;

	tp->cur_rx = 0;
	tp->cur_tx = 0;
	tp->dirty_tx = 0;

	for (i = 0; i < NUM_TX_DESC; i++)
		tp->tx_buf[i] = &tp->tx_bufs[i * TX_BUF_SIZE];
}


static void rtl8139_tx_clear (struct rtl8139_private *tp)
{
	tp->cur_tx = 0;
	tp->dirty_tx = 0;

	/* XXX account for unsent Tx packets in tp->stats.tx_dropped */
}



static int rtl8139_start_xmit (struct rtskb *skb, struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;

	void *ioaddr = tp->mmio_addr;
	unsigned int entry;
	unsigned int len = skb->len;
	rtdm_lockctx_t context;

	/* Calculate the next Tx descriptor entry. */
	entry = tp->cur_tx % NUM_TX_DESC;

	if (likely(len < TX_BUF_SIZE)) {
		if (unlikely(skb->xmit_stamp != NULL)) {
			rtdm_lock_irqsave(context);
			*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() +
						       *skb->xmit_stamp);
			/* typically, we are only copying a few bytes here */
			rtskb_copy_and_csum_dev(skb, tp->tx_buf[entry]);
		} else {
			/* copy larger packets outside the lock */
			rtskb_copy_and_csum_dev(skb, tp->tx_buf[entry]);
			rtdm_lock_irqsave(context);
		}
	} else {
		dev_kfree_rtskb(skb);
		tp->stats.tx_dropped++;
		return 0;
	}


	/* Note: the chip doesn't have auto-pad! */
	rtdm_lock_get(&tp->lock);
	RTL_W32_F (TxStatus0 + (entry * sizeof (u32)), tp->tx_flag | max(len, (unsigned int)ETH_ZLEN));
	tp->cur_tx++;
	wmb();
	if ((tp->cur_tx - NUM_TX_DESC) == tp->dirty_tx)
		rtnetif_stop_queue (rtdev);
	rtdm_lock_put_irqrestore(&tp->lock, context);

	dev_kfree_rtskb(skb);

#ifdef DEBUG
	rtdm_printk ("%s: Queued Tx packet size %u to slot %d.\n", rtdev->name, len, entry);
#endif
	return 0;
}

static int rtl8139_ioctl(struct rtnet_device *rtdev, unsigned int request, void *arg)
{
    struct rtl8139_private *tp = rtdev->priv;
    void *ioaddr = tp->mmio_addr;
    int nReturn = 0;
    struct ifreq *ifr = arg;
    struct ethtool_value *value;

    switch (request) {
	case SIOCETHTOOL:
	    /* TODO: user-safe parameter access, most probably one layer higher */
	    value = (struct ethtool_value *)ifr->ifr_data;
	    if (value->cmd == ETHTOOL_GLINK)
	    {
		if (RTL_R16(CSCR) & CSCR_LinkOKBit)
		    value->data = 1;
		else
		    value->data = 0;
	    }
	    break;

	default:
	    nReturn = -EOPNOTSUPP;
	    break;
    }
    return nReturn;
}

static struct net_device_stats *rtl8139_get_stats(struct rtnet_device*rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	return &tp->stats;
}

static void rtl8139_tx_interrupt (struct rtnet_device *rtdev,
				  struct rtl8139_private *tp,
				  void *ioaddr)
{
	unsigned long dirty_tx, tx_left;

	dirty_tx = tp->dirty_tx;
	tx_left = tp->cur_tx - dirty_tx;

	while (tx_left > 0) {
		int entry = dirty_tx % NUM_TX_DESC;
		int txstatus;

		txstatus = RTL_R32 (TxStatus0 + (entry * sizeof (u32)));

		if (!(txstatus & (TxStatOK | TxUnderrun | TxAborted)))
			break;        /* It still hasn't been Txed */

		/* Note: TxCarrierLost is always asserted at 100mbps. */
		if (txstatus & (TxOutOfWindow | TxAborted)) {
			/* There was an major error, log it. */
			rtdm_printk("%s: Transmit error, Tx status %8.8x.\n",
				    rtdev->name, txstatus);
			tp->stats.tx_errors++;
			if (txstatus & TxAborted) {
				tp->stats.tx_aborted_errors++;
				RTL_W32 (TxConfig, TxClearAbt);
				RTL_W16 (IntrStatus, TxErr);
				wmb();
			}
			if (txstatus & TxCarrierLost)
				tp->stats.tx_carrier_errors++;
			if (txstatus & TxOutOfWindow)
				tp->stats.tx_window_errors++;
#ifdef ETHER_STATS
			if ((txstatus & 0x0f000000) == 0x0f000000)
				tp->stats.collisions16++;
#endif
		} else {
			if (txstatus & TxUnderrun) {
				/* Add 64 to the Tx FIFO threshold. */
				if (tp->tx_flag < 0x00300000)
					tp->tx_flag += 0x00020000;
				tp->stats.tx_fifo_errors++;
			}
			tp->stats.collisions += (txstatus >> 24) & 15;
			tp->stats.tx_bytes += txstatus & 0x7ff;
			tp->stats.tx_packets++;
		}

		dirty_tx++;
		tx_left--;
	}

	/* only wake the queue if we did work, and the queue is stopped */
	if (tp->dirty_tx != dirty_tx) {
		tp->dirty_tx = dirty_tx;
		mb();
		if (rtnetif_queue_stopped (rtdev))
			rtnetif_wake_queue (rtdev);
	}
}


/* TODO: clean this up!  Rx reset need not be this intensive */
static void rtl8139_rx_err
(u32 rx_status, struct rtnet_device *rtdev, struct rtl8139_private *tp, void *ioaddr)
{
/*        u8 tmp8;
#ifndef CONFIG_8139_NEW_RX_RESET
	int tmp_work;
#endif */

	/* RTnet-TODO: We really need an error manager to handle such issues... */
	rtdm_printk("%s: FATAL - Ethernet frame had errors, status %8.8x.\n",
		    rtdev->name, rx_status);
}


static void rtl8139_rx_interrupt (struct rtnet_device *rtdev,
				  struct rtl8139_private *tp, void *ioaddr,
				  nanosecs_abs_t *time_stamp)
{
	unsigned char *rx_ring;
	u16 cur_rx;

	rx_ring = tp->rx_ring;
	cur_rx = tp->cur_rx;

	while ((RTL_R8 (ChipCmd) & RxBufEmpty) == 0) {
		int ring_offset = cur_rx % RX_BUF_LEN;
		u32 rx_status;
		unsigned int rx_size;
		unsigned int pkt_size;
		struct rtskb *skb;

		rmb();

		/* read size+status of next frame from DMA ring buffer */
		rx_status = le32_to_cpu (*(u32 *) (rx_ring + ring_offset));
		rx_size = rx_status >> 16;
		pkt_size = rx_size - 4;

		/* Packet copy from FIFO still in progress.
		 * Theoretically, this should never happen
		 * since EarlyRx is disabled.
		 */
		if (rx_size == 0xfff0) {
			tp->xstats.early_rx++;
			break;
		}

		/* If Rx err or invalid rx_size/rx_status received
		 * (which happens if we get lost in the ring),
		 * Rx process gets reset, so we abort any further
		 * Rx processing.
		 */
		if ((rx_size > (MAX_ETH_FRAME_SIZE+4)) ||
		    (rx_size < 8) ||
		    (!(rx_status & RxStatusOK))) {
			rtl8139_rx_err (rx_status, rtdev, tp, ioaddr);
			return;
		}

		/* Malloc up new buffer, compatible with net-2e. */
		/* Omit the four octet CRC from the length. */

		/* TODO: consider allocating skb's outside of
		 * interrupt context, both to speed interrupt processing,
		 * and also to reduce the chances of having to
		 * drop packets here under memory pressure.
		 */

		skb = rtnetdev_alloc_rtskb(rtdev, pkt_size + 2);
		if (skb) {
			skb->time_stamp = *time_stamp;
			rtskb_reserve (skb, 2);        /* 16 byte align the IP fields. */


			/* eth_copy_and_sum (skb, &rx_ring[ring_offset + 4], pkt_size, 0); */
			memcpy (skb->data, &rx_ring[ring_offset + 4], pkt_size);
			rtskb_put (skb, pkt_size);
			skb->protocol = rt_eth_type_trans (skb, rtdev);
			rtnetif_rx (skb);
			tp->stats.rx_bytes += pkt_size;
			tp->stats.rx_packets++;
		} else {
			rtdm_printk (KERN_WARNING"%s: Memory squeeze, dropping packet.\n", rtdev->name);
			tp->stats.rx_dropped++;
		}

		cur_rx = (cur_rx + rx_size + 4 + 3) & ~3;
		RTL_W16 (RxBufPtr, cur_rx - 16);

		if (RTL_R16 (IntrStatus) & RxAckBits)
			RTL_W16_F (IntrStatus, RxAckBits);
	}

	tp->cur_rx = cur_rx;
}


static void rtl8139_weird_interrupt (struct rtnet_device *rtdev,
				     struct rtl8139_private *tp,
				     void *ioaddr,
				     int status, int link_changed)
{
	rtdm_printk ("%s: Abnormal interrupt, status %8.8x.\n",
		      rtdev->name, status);

	/* Update the error count. */
	tp->stats.rx_missed_errors += RTL_R32 (RxMissed);
	RTL_W32 (RxMissed, 0);

	if ((status & RxUnderrun) && link_changed && (tp->drv_flags & HAS_LNK_CHNG)) {
		/* Really link-change on new chips. */
		status &= ~RxUnderrun;
	}

	/* XXX along with rtl8139_rx_err, are we double-counting errors? */
	if (status &
	    (RxUnderrun | RxOverflow | RxErr | RxFIFOOver))
		tp->stats.rx_errors++;

	if (status & PCSTimeout)
		tp->stats.rx_length_errors++;

	if (status & (RxUnderrun | RxFIFOOver))
		tp->stats.rx_fifo_errors++;

	if (status & PCIErr) {
		u16 pci_cmd_status;
		pci_read_config_word (tp->pci_dev, PCI_STATUS, &pci_cmd_status);
		pci_write_config_word (tp->pci_dev, PCI_STATUS, pci_cmd_status);

		rtdm_printk (KERN_ERR "%s: PCI Bus error %4.4x.\n", rtdev->name, pci_cmd_status);
	}
}

/* The interrupt handler does all of the Rx thread work and cleans up
   after the Tx thread. */
static int rtl8139_interrupt(rtdm_irq_t *irq_handle)
{
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtnet_device *rtdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct rtl8139_private *tp = rtdev->priv;
	void *ioaddr = tp->mmio_addr;
	int ackstat;
	int status;
	int link_changed = 0; /* avoid bogus "uninit" warning */
	int saved_status = 0;
	int ret = RTDM_IRQ_NONE;

	rtdm_lock_get(&tp->lock);

	status = RTL_R16(IntrStatus);

	/* h/w no longer present (hotplug?) or major error, bail */
	if (unlikely(status == 0xFFFF) || unlikely(!(status & rtl8139_intr_mask)))
		goto out;

	ret = RTDM_IRQ_HANDLED;

	/* close possible race with dev_close */
	if (unlikely(!rtnetif_running(rtdev))) {
		RTL_W16(IntrMask, 0);
		goto out;
	}

	/* Acknowledge all of the current interrupt sources ASAP, but
	   first get an additional status bit from CSCR. */
	if (unlikely(status & RxUnderrun))
		link_changed = RTL_R16(CSCR) & CSCR_LinkChangeBit;

	/* The chip takes special action when we clear RxAckBits,
	 * so we clear them later in rtl8139_rx_interrupt
	 */
	ackstat = status & ~(RxAckBits | TxErr);
	if (ackstat)
		RTL_W16(IntrStatus, ackstat);

	if (status & RxAckBits) {
		saved_status |= RxAckBits;
		rtl8139_rx_interrupt(rtdev, tp, ioaddr, &time_stamp);
	}

	/* Check uncommon events with one test. */
	if (unlikely(status & (PCIErr | PCSTimeout | RxUnderrun | RxErr)))
		rtl8139_weird_interrupt(rtdev, tp, ioaddr, status, link_changed);

	if (status & (TxOK |TxErr)) {
		rtl8139_tx_interrupt(rtdev, tp, ioaddr);
		if (status & TxErr) {
			RTL_W16(IntrStatus, TxErr);
			saved_status |= TxErr;
		}
	}
 out:
	rtdm_lock_put(&tp->lock);

	if (saved_status & RxAckBits)
		rt_mark_stack_mgr(rtdev);

	if (saved_status & TxErr)
		rtnetif_err_tx(rtdev);

	return ret;
}


static int rtl8139_close (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	void *ioaddr = tp->mmio_addr;
	rtdm_lockctx_t context;

	printk ("%s: Shutting down ethercard, status was 0x%4.4x.\n", rtdev->name, RTL_R16 (IntrStatus));

	rtnetif_stop_queue (rtdev);

	rtdm_lock_get_irqsave (&tp->lock, context);
	/* Stop the chip's Tx and Rx DMA processes. */
	RTL_W8 (ChipCmd, 0);
	/* Disable interrupts by clearing the interrupt mask. */
	RTL_W16 (IntrMask, 0);
	/* Update the error counts. */
	tp->stats.rx_missed_errors += RTL_R32 (RxMissed);
	RTL_W32 (RxMissed, 0);
	rtdm_lock_put_irqrestore (&tp->lock, context);

	rtdm_irq_free(&tp->irq_handle);

	rt_stack_disconnect(rtdev);

	rtl8139_tx_clear (tp);

	pci_free_consistent(tp->pci_dev, RX_BUF_TOT_LEN, tp->rx_ring, tp->rx_ring_dma);
	pci_free_consistent(tp->pci_dev, TX_BUF_TOT_LEN, tp->tx_bufs, tp->tx_bufs_dma);
	tp->rx_ring = NULL;
	tp->tx_bufs = NULL;

	/* Green! Put the chip in low-power mode. */
	RTL_W8 (Cfg9346, Cfg9346_Unlock);

	if (rtl_chip_info[tp->chipset].flags & HasHltClk)
		RTL_W8 (HltClk, 'H');        /* 'R' would leave the clock running. */

	return 0;
}



/* Set or clear the multicast filter for this adaptor.
   This routine is not state sensitive and need not be SMP locked. */
static void __set_rx_mode (struct rtnet_device *rtdev)
{
	struct rtl8139_private *tp = rtdev->priv;
	void *ioaddr = tp->mmio_addr;
	u32 mc_filter[2];        /* Multicast hash filter */
	int rx_mode;
	u32 tmp;

#ifdef DEBUG
	rtdm_printk ("%s:   rtl8139_set_rx_mode(%4.4x) done -- Rx config %8.8lx.\n",
			rtdev->name, rtdev->flags, RTL_R32 (RxConfig));
#endif

	/* Note: do not reorder, GCC is clever about common statements. */
	if (rtdev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		/*printk (KERN_NOTICE "%s: Promiscuous mode enabled.\n", rtdev->name);*/
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else if (rtdev->flags & IFF_ALLMULTI) {
		/* Too many to filter perfectly -- accept all multicasts. */
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
		rx_mode = AcceptBroadcast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0;
	}

	/* We can safely update without stopping the chip. */
	tmp = rtl8139_rx_config | rx_mode;
	if (tp->rx_config != tmp) {
		RTL_W32_F (RxConfig, tmp);
		tp->rx_config = tmp;
	}
	RTL_W32_F (MAR0 + 0, mc_filter[0]);
	RTL_W32_F (MAR0 + 4, mc_filter[1]);
}

static void rtl8139_set_rx_mode (struct rtnet_device *rtdev)
{
	rtdm_lockctx_t context;
	struct rtl8139_private *tp = rtdev->priv;

	rtdm_lock_get_irqsave (&tp->lock, context);
	__set_rx_mode(rtdev);
	rtdm_lock_put_irqrestore (&tp->lock, context);
}

static struct pci_driver rtl8139_pci_driver = {
	name:                   DRV_NAME,
	id_table:               rtl8139_pci_tbl,
	probe:                  rtl8139_init_one,
	remove:                 rtl8139_remove_one,
	suspend:                NULL,
	resume:                 NULL,
};


static int __init rtl8139_init_module (void)
{
	/* when we're a module, we always print a version message,
	 * even if no 8139 board is found.
	 */

#ifdef MODULE
	printk (KERN_INFO RTL8139_DRIVER_NAME "\n");
#endif

	return pci_register_driver (&rtl8139_pci_driver);
}


static void __exit rtl8139_cleanup_module (void)
{
	pci_unregister_driver (&rtl8139_pci_driver);
}


module_init(rtl8139_init_module);
module_exit(rtl8139_cleanup_module);
