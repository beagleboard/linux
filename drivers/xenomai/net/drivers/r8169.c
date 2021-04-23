/*
=========================================================================
 r8169.c: A RealTek RTL8169s/8110s Gigabit Ethernet driver for Linux kernel 2.4.x.
 --------------------------------------------------------------------

 History:
 Feb  4 2002	- created initially by ShuChen <shuchen@realtek.com.tw>.
 May 20 2002	- Add link status force-mode and TBI mode support.
=========================================================================

RTL8169_VERSION "1.1"	<2002/10/4>

	The bit4:0 of MII register 4 is called "selector field", and have to be
	00001b to indicate support of IEEE std 802.3 during NWay process of
	exchanging Link Code Word (FLP).

RTL8169_VERSION "1.2"	<2003/6/17>
	Update driver module name.
	Modify ISR.
	Add chip mcfg.

RTL8169_VERSION "1.3"	<2003/6/20>
	Add chip pcfg.
	Add priv->phy_timer_t, rtl8169_phy_timer_t_handler()
	Add rtl8169_hw_PHY_config()
	Add rtl8169_hw_PHY_reset()

RTL8169_VERSION "1.4"	<2003/7/14>
	Add tx_bytes, rx_bytes.

RTL8169_VERSION "1.5"	<2003/7/18>
	Set 0x0000 to PHY at offset 0x0b.
	Modify chip mcfg, pcfg
	Force media for multiple card.
RTL8169_VERSION "1.6"	<2003/8/25>
	Modify receive data buffer.

RTL8169_VERSION "1.7"	<2003/9/18>
	Add Jumbo Frame support.

RTL8169_VERSION "1.8"	<2003/10/21>
	Performance and CPU Utilizaion Enhancement.

RTL8169_VERSION "1.9"	<2003/12/29>
	Enable Tx/Rx flow control.

RTL8169_VERSION "2.0"	<2004/03/26>
	Beta version.
	Support for linux 2.6.x

RTL8169_VERSION "2.1"	<2004/07/05>
	Modify parameters.

RTL8169_VERSION "2.2"	<2004/08/09>
	Add.pci_dma_sync_single.
	Add pci_alloc_consistent()/pci_free_consistent().
	Revise parameters.
	Recognize our interrupt for linux 2.6.x.
*/

/*
 * Ported to RTnet by Klaus Keppler <klaus.keppler@gmx.de>
 * All RTnet porting stuff may be used and distributed according to the
 * terms of the GNU General Public License (GPL).
 *
 * Version 2.2-04 <2005/08/22>
 *    Initial release of this driver, based on RTL8169 driver v2.2
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/version.h>

#include <linux/timer.h>
#include <linux/init.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
#include <linux/pci-aspm.h>
#endif

#include <rtnet_port.h>	/*** RTnet ***/

#define RTL8169_VERSION "2.2-04"
#define MODULENAME "rt_r8169"
#define RTL8169_DRIVER_NAME   MODULENAME " RTnet Gigabit Ethernet driver " RTL8169_VERSION
#define PFX MODULENAME ": "

//#define RTL8169_DEBUG
#undef RTL8169_JUMBO_FRAME_SUPPORT	/*** RTnet: no not enable! ***/
#undef	RTL8169_HW_FLOW_CONTROL_SUPPORT


#undef RTL8169_IOCTL_SUPPORT	/*** RTnet: do not enable! ***/
#undef RTL8169_DYNAMIC_CONTROL
#undef RTL8169_USE_IO


#ifdef RTL8169_DEBUG
	#define assert(expr) \
		if(!(expr)) { printk( "Assertion failed! %s,%s,%s,line=%d\n", #expr,__FILE__,__FUNCTION__,__LINE__); }
	/*** RTnet / <kk>: rt_assert must be used instead of assert() within interrupt context! ***/
	#define rt_assert(expr) \
		if(!(expr)) { rtdm_printk( "Assertion failed! %s,%s,%s,line=%d\n", #expr,__FILE__,__FUNCTION__,__LINE__); }
	/*** RTnet / <kk>: RT_DBG_PRINT must be used instead of DBG_PRINT() within interrupt context! ***/
	#define DBG_PRINT( fmt, args...)   printk("r8169: " fmt, ## args);
	#define RT_DBG_PRINT( fmt, args...)   rtdm_printk("r8169: " fmt, ## args);
#else
	#define assert(expr) do {} while (0)
	#define rt_assert(expr) do {} while (0)
	#define DBG_PRINT( fmt, args...)   ;
	#define RT_DBG_PRINT( fmt, args...)   ;
#endif	// end of #ifdef RTL8169_DEBUG

/* media options */
#define MAX_UNITS 8
static int media[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};

/*** RTnet ***/
static int cards[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = 1 };
module_param_array(cards, int, NULL, 0444);
MODULE_PARM_DESC(cards, "array of cards to be supported (e.g. 1,0,1)");
/*** /RTnet ***/

/* <kk> Enable debugging output */
#define DEBUG_RX_SYNC 1
#define DEBUG_RX_OTHER 2
#define DEBUG_TX_SYNC 4
#define DEBUG_TX_OTHER 8
#define DEBUG_RUN 16
static int local_debug = -1;
static int r8169_debug = -1;
module_param_named(debug, local_debug, int, 0444);
MODULE_PARM_DESC(debug, MODULENAME " debug level (bit mask, see docs!)");


/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static int max_interrupt_work = 20;

/* MAC address length*/
#define MAC_ADDR_LEN        6

#define RX_FIFO_THRESH      7       /* 7 means NO threshold, Rx buffer level before first PCI xfer.  */
#define RX_DMA_BURST        7       /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST        7       /* Maximum PCI burst, '6' is 1024 */
#define ETTh                0x3F    /* 0x3F means NO threshold */

#define ETH_HDR_LEN         14
#define DEFAULT_MTU         1500
#define DEFAULT_RX_BUF_LEN  1536


#ifdef RTL8169_JUMBO_FRAME_SUPPORT
#define MAX_JUMBO_FRAME_MTU	( 10000 )
#define MAX_RX_SKBDATA_SIZE	( MAX_JUMBO_FRAME_MTU + ETH_HDR_LEN )
#else
#define MAX_RX_SKBDATA_SIZE 1600
#endif //end #ifdef RTL8169_JUMBO_FRAME_SUPPORT


#define InterFrameGap       0x03    /* 3 means InterFrameGap = the shortest one */

//#define NUM_TX_DESC         64	/* Number of Tx descriptor registers*/
//#define NUM_RX_DESC         64	/* Number of Rx descriptor registers*/

#define TX_RING_SIZE          16	/*** RTnet ***/
#define NUM_TX_DESC TX_RING_SIZE	/* Number of Tx descriptor registers*/	/*** RTnet ***/
#define RX_RING_SIZE           8	/*** RTnet ***/
#define NUM_RX_DESC RX_RING_SIZE	/* Number of Rx descriptor registers*/	/*** RTnet ***/

#define RTL_MIN_IO_SIZE     0x80
#define TX_TIMEOUT          (6*HZ)
//#define RTL8169_TIMER_EXPIRE_TIME 100 //100	/*** RTnet ***/


#ifdef RTL8169_USE_IO
#define RTL_W8(reg, val8)   outb ((val8), ioaddr + (reg))
#define RTL_W16(reg, val16) outw ((val16), ioaddr + (reg))
#define RTL_W32(reg, val32) outl ((val32), ioaddr + (reg))
#define RTL_R8(reg)         inb (ioaddr + (reg))
#define RTL_R16(reg)        inw (ioaddr + (reg))
#define RTL_R32(reg)        ((unsigned long) inl (ioaddr + (reg)))
#else
/* write/read MMIO register */
#define RTL_W8(reg, val8)   writeb ((val8), (void *)ioaddr + (reg))
#define RTL_W16(reg, val16) writew ((val16), (void *)ioaddr + (reg))
#define RTL_W32(reg, val32) writel ((val32), (void *)ioaddr + (reg))
#define RTL_R8(reg)         readb ((void *)ioaddr + (reg))
#define RTL_R16(reg)        readw ((void *)ioaddr + (reg))
#define RTL_R32(reg)        ((unsigned long) readl ((void *)ioaddr + (reg)))
#endif

#define MCFG_METHOD_1		0x01
#define MCFG_METHOD_2		0x02
#define MCFG_METHOD_3		0x03
#define MCFG_METHOD_4		0x04

#define PCFG_METHOD_1		0x01	//PHY Reg 0x03 bit0-3 == 0x0000
#define PCFG_METHOD_2		0x02	//PHY Reg 0x03 bit0-3 == 0x0001
#define PCFG_METHOD_3		0x03	//PHY Reg 0x03 bit0-3 == 0x0002


#ifdef RTL8169_DYNAMIC_CONTROL
#include "r8169_callback.h"
#endif  //end #ifdef RTL8169_DYNAMIC_CONTROL


const static struct {
	const char *name;
	u8 mcfg;                 /* depend on RTL8169 docs */
	u32 RxConfigMask;       /* should clear the bits supported by this chip */
} rtl_chip_info[] = {
	{ "RTL8169",  MCFG_METHOD_1,  0xff7e1880 },
	{ "RTL8169s/8110s",  MCFG_METHOD_2,  0xff7e1880 },
	{ "RTL8169s/8110s",  MCFG_METHOD_3,  0xff7e1880 },
};


static struct pci_device_id rtl8169_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8136), 0, 0, 2 },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8167), 0, 0, 1 },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8169), 0, 0, 1 },
	{ PCI_DEVICE(PCI_VENDOR_ID_DLINK,	0x4300), 0, 0, 1 },	/* <kk> D-Link DGE-528T */
	{0,},
};


MODULE_DEVICE_TABLE (pci, rtl8169_pci_tbl);


enum RTL8169_registers {
	MAC0 = 0x0,
	MAR0 = 0x8,
	TxDescStartAddr	= 0x20,
	TxHDescStartAddr= 0x28,
	FLASH	= 0x30,
	ERSR	= 0x36,
	ChipCmd	= 0x37,
	TxPoll	= 0x38,
	IntrMask = 0x3C,
	IntrStatus = 0x3E,
	TxConfig = 0x40,
	RxConfig = 0x44,
	RxMissed = 0x4C,
	Cfg9346 = 0x50,
	Config0	= 0x51,
	Config1	= 0x52,
	Config2	= 0x53,
	Config3	= 0x54,
	Config4	= 0x55,
	Config5	= 0x56,
	MultiIntr = 0x5C,
	PHYAR	= 0x60,
	TBICSR	= 0x64,
	TBI_ANAR = 0x68,
	TBI_LPAR = 0x6A,
	PHYstatus = 0x6C,
	RxMaxSize = 0xDA,
	CPlusCmd = 0xE0,
	RxDescStartAddr	= 0xE4,
	ETThReg	= 0xEC,
	FuncEvent	= 0xF0,
	FuncEventMask	= 0xF4,
	FuncPresetState	= 0xF8,
	FuncForceEvent	= 0xFC,
};

enum RTL8169_register_content {
	/*InterruptStatusBits*/
	SYSErr          = 0x8000,
	PCSTimeout	= 0x4000,
	SWInt		= 0x0100,
	TxDescUnavail	= 0x80,
	RxFIFOOver      = 0x40,
	LinkChg         = 0x20,
	RxOverflow      = 0x10,
	TxErr   = 0x08,
	TxOK    = 0x04,
	RxErr   = 0x02,
	RxOK    = 0x01,

	/*RxStatusDesc*/
	RxRES = 0x00200000,
	RxCRC = 0x00080000,
	RxRUNT= 0x00100000,
	RxRWT = 0x00400000,

	/*ChipCmdBits*/
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,

	/*Cfg9346Bits*/
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,

	/*rx_mode_bits*/
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,

	/*RxConfigBits*/
	RxCfgFIFOShift = 13,
	RxCfgDMAShift = 8,

	/*TxConfigBits*/
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,

	/* Config2 register */
	MSIEnable	= (1 << 5),

	/*rtl8169_PHYstatus*/
	TBI_Enable	= 0x80,
	TxFlowCtrl	= 0x40,
	RxFlowCtrl	= 0x20,
	_1000bpsF	= 0x10,
	_100bps		= 0x08,
	_10bps		= 0x04,
	LinkStatus	= 0x02,
	FullDup		= 0x01,

	/*GIGABIT_PHY_registers*/
	PHY_CTRL_REG = 0,
	PHY_STAT_REG = 1,
	PHY_AUTO_NEGO_REG = 4,
	PHY_1000_CTRL_REG = 9,

	/*GIGABIT_PHY_REG_BIT*/
	PHY_Restart_Auto_Nego	= 0x0200,
	PHY_Enable_Auto_Nego	= 0x1000,

	//PHY_STAT_REG = 1;
	PHY_Auto_Neco_Comp	= 0x0020,

	//PHY_AUTO_NEGO_REG = 4;
	PHY_Cap_10_Half		= 0x0020,
	PHY_Cap_10_Full		= 0x0040,
	PHY_Cap_100_Half	= 0x0080,
	PHY_Cap_100_Full	= 0x0100,

	//PHY_1000_CTRL_REG = 9;
	PHY_Cap_1000_Full	= 0x0200,
	PHY_Cap_1000_Half	= 0x0100,

	PHY_Cap_PAUSE		= 0x0400,
	PHY_Cap_ASYM_PAUSE	= 0x0800,

	PHY_Cap_Null		= 0x0,

	/*_MediaType*/
	_10_Half	= 0x01,
	_10_Full	= 0x02,
	_100_Half	= 0x04,
	_100_Full	= 0x08,
	_1000_Full	= 0x10,

	/*_TBICSRBit*/
	TBILinkOK       = 0x02000000,
};



enum _DescStatusBit {
	OWNbit	= 0x80000000,
	EORbit	= 0x40000000,
	FSbit	= 0x20000000,
	LSbit	= 0x10000000,
};


struct TxDesc {
	u32		status;
	u32		vlan_tag;
	u32		buf_addr;
	u32		buf_Haddr;
};

struct RxDesc {
	u32		status;
	u32		vlan_tag;
	u32		buf_addr;
	u32		buf_Haddr;
};


typedef struct timer_list rt_timer_t;

enum rtl8169_features {
	RTL_FEATURE_WOL		= (1 << 0),
	RTL_FEATURE_MSI		= (1 << 1),
	RTL_FEATURE_GMII	= (1 << 2),
};


struct rtl8169_private {
	unsigned long ioaddr;                /* memory map physical address*/
	struct pci_dev *pci_dev;                /* Index of PCI device  */
	struct net_device_stats stats;          /* statistics of net device */
	rtdm_lock_t lock;                       /* spin lock flag */	/*** RTnet ***/
	int chipset;
	int mcfg;
	int pcfg;
/*	rt_timer_t r8169_timer; */	/*** RTnet ***/
/*	unsigned long expire_time;	*/	/*** RTnet ***/

	unsigned long phy_link_down_cnt;
	unsigned long cur_rx;                   /* Index into the Rx descriptor buffer of next Rx pkt. */
	unsigned long cur_tx;                   /* Index into the Tx descriptor buffer of next Rx pkt. */
	unsigned long dirty_tx;
	struct	TxDesc	*TxDescArray;           /* Index of 256-alignment Tx Descriptor buffer */
	struct	RxDesc	*RxDescArray;           /* Index of 256-alignment Rx Descriptor buffer */
	struct	rtskb	*Tx_skbuff[NUM_TX_DESC];/* Index of Transmit data buffer */	/*** RTnet ***/
	struct	rtskb	*Rx_skbuff[NUM_RX_DESC];/* Receive data buffer */			/*** RTnet ***/
	unsigned char   drvinit_fail;

	dma_addr_t txdesc_array_dma_addr[NUM_TX_DESC];
	dma_addr_t rxdesc_array_dma_addr[NUM_RX_DESC];
	dma_addr_t rx_skbuff_dma_addr[NUM_RX_DESC];

	void *txdesc_space;
	dma_addr_t txdesc_phy_dma_addr;
	int sizeof_txdesc_space;

	void *rxdesc_space;
	dma_addr_t rxdesc_phy_dma_addr;
	int sizeof_rxdesc_space;

	int curr_mtu_size;
	int tx_pkt_len;
	int rx_pkt_len;

	int hw_rx_pkt_len;

	int rx_buf_size;	/*** RTnet / <kk> ***/

#ifdef RTL8169_DYNAMIC_CONTROL
	struct r8169_cb_t rt;
#endif //end #ifdef RTL8169_DYNAMIC_CONTROL

	unsigned char   linkstatus;
	rtdm_irq_t irq_handle;			/*** RTnet ***/

	unsigned features;
};


MODULE_AUTHOR ("Realtek, modified for RTnet by Klaus.Keppler@gmx.de");
MODULE_DESCRIPTION ("RealTek RTL-8169 Gigabit Ethernet driver");
module_param_array(media, int, NULL, 0444);
MODULE_LICENSE("GPL");


static int rtl8169_open (struct rtnet_device *rtdev);
static int rtl8169_start_xmit (struct rtskb *skb, struct rtnet_device *rtdev);

static int rtl8169_interrupt(rtdm_irq_t *irq_handle);

static void rtl8169_init_ring (struct rtnet_device *rtdev);
static void rtl8169_hw_start (struct rtnet_device *rtdev);
static int rtl8169_close (struct rtnet_device *rtdev);
static inline u32 ether_crc (int length, unsigned char *data);
static void rtl8169_set_rx_mode (struct rtnet_device *rtdev);
/* static void rtl8169_tx_timeout (struct net_device *dev); */	/*** RTnet ***/
static struct net_device_stats *rtl8169_get_stats(struct rtnet_device *netdev);

#ifdef RTL8169_JUMBO_FRAME_SUPPORT
static int rtl8169_change_mtu(struct net_device *dev, int new_mtu);
#endif //end #ifdef RTL8169_JUMBO_FRAME_SUPPORT

static void rtl8169_hw_PHY_config (struct rtnet_device *rtdev);
/* static void rtl8169_hw_PHY_reset(struct net_device *dev); */	/*** RTnet ***/
static const u16 rtl8169_intr_mask = LinkChg | RxOverflow | RxFIFOOver | TxErr | TxOK | RxErr | RxOK | SYSErr;	/*** <kk> added SYSErr ***/
static const unsigned int rtl8169_rx_config = (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift) | 0x0000000E;

/*** <kk> these functions are backported from Linux-2.6.12's r8169.c driver ***/
static void rtl8169_irq_mask_and_ack(unsigned long ioaddr);
/* static void rtl8169_asic_down(unsigned long ioaddr); */ /*** RTnet ***/
static void rtl8169_pcierr_interrupt(struct rtnet_device *rtdev);

#define RTL8169_WRITE_GMII_REG_BIT( ioaddr, reg, bitnum, bitval )\
{ \
	int val; \
	if( bitval == 1 ){ val = ( RTL8169_READ_GMII_REG( ioaddr, reg ) | (bitval<<bitnum) ) & 0xffff ; } \
	else{ val = ( RTL8169_READ_GMII_REG( ioaddr, reg ) & (~(0x0001<<bitnum)) ) & 0xffff ; } \
	RTL8169_WRITE_GMII_REG( ioaddr, reg, val ); \
}



#ifdef RTL8169_DEBUG
unsigned alloc_rxskb_cnt = 0;
#define RTL8169_ALLOC_RXSKB(bufsize)    dev_alloc_skb(bufsize); alloc_rxskb_cnt ++ ;
#define RTL8169_FREE_RXSKB(skb)         kfree_skb(skb); alloc_rxskb_cnt -- ;
#define RTL8169_NETIF_RX(skb)           netif_rx(skb); alloc_rxskb_cnt -- ;
#else
#define RTL8169_ALLOC_RXSKB(bufsize)    dev_alloc_skb(bufsize);
#define RTL8169_FREE_RXSKB(skb)         kfree_skb(skb);
#define RTL8169_NETIF_RX(skb)           netif_rx(skb);
#endif //end #ifdef RTL8169_DEBUG


//=================================================================
//	PHYAR
//	bit		Symbol
//	31		Flag
//	30-21	reserved
//	20-16	5-bit GMII/MII register address
//	15-0	16-bit GMII/MII register data
//=================================================================
void RTL8169_WRITE_GMII_REG( unsigned long ioaddr, int RegAddr, int value )
{
	int	i;

	RTL_W32 ( PHYAR, 0x80000000 | (RegAddr&0xFF)<<16 | value);
	udelay(1000);

	for( i = 2000; i > 0 ; i -- ){
		// Check if the RTL8169 has completed writing to the specified MII register
		if( ! (RTL_R32(PHYAR)&0x80000000) ){
			break;
		}
		else{
			udelay(100);
		}// end of if( ! (RTL_R32(PHYAR)&0x80000000) )
	}// end of for() loop
}
//=================================================================
int RTL8169_READ_GMII_REG( unsigned long ioaddr, int RegAddr )
{
	int i, value = -1;

	RTL_W32 ( PHYAR, 0x0 | (RegAddr&0xFF)<<16 );
	udelay(1000);

	for( i = 2000; i > 0 ; i -- ){
		// Check if the RTL8169 has completed retrieving data from the specified MII register
		if( RTL_R32(PHYAR) & 0x80000000 ){
			value = (int)( RTL_R32(PHYAR)&0xFFFF );
			break;
		}
		else{
			udelay(100);
		}// end of if( RTL_R32(PHYAR) & 0x80000000 )
	}// end of for() loop
	return value;
}


#ifdef RTL8169_IOCTL_SUPPORT
#include "r8169_ioctl.c"
#endif //end #ifdef RTL8169_IOCTL_SUPPORT


#ifdef RTL8169_DYNAMIC_CONTROL
#include "r8169_callback.c"
#endif



//======================================================================================================
//======================================================================================================
static int rtl8169_init_board ( struct pci_dev *pdev, struct rtnet_device **dev_out, unsigned long *ioaddr_out, int region)
{
	unsigned long ioaddr = 0;
	struct rtnet_device *rtdev;
	struct rtl8169_private *priv;
	int rc, i;
	unsigned long mmio_start, mmio_end, mmio_flags, mmio_len;


	assert (pdev != NULL);
	assert (ioaddr_out != NULL);

	*ioaddr_out = 0;
	*dev_out = NULL;

	/*** RTnet ***/
	rtdev = rt_alloc_etherdev(sizeof(struct rtl8169_private),
				RX_RING_SIZE * 2 + TX_RING_SIZE);
	if (rtdev == NULL) {
		printk (KERN_ERR PFX "unable to alloc new ethernet\n");
		return -ENOMEM;
	}
	rtdev_alloc_name(rtdev, "rteth%d");
	rt_rtdev_connect(rtdev, &RTDEV_manager);
	rtdev->vers = RTDEV_VERS_2_0;
	rtdev->sysbind = &pdev->dev;
	/*** /RTnet ***/

	priv = rtdev->priv;

	/* disable ASPM completely as that cause random device stop working
	 * problems as well as full system hangs for some PCIe devices users */
	pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
				     PCIE_LINK_STATE_CLKPM);

	// enable device (incl. PCI PM wakeup and hotplug setup)
	rc = pci_enable_device (pdev);
	if (rc)
		goto err_out;

	if (pci_set_mwi(pdev) < 0)
		printk("R8169: Mem-Wr-Inval unavailable\n");

	mmio_start = pci_resource_start (pdev, region);
	mmio_end = pci_resource_end (pdev, region);
	mmio_flags = pci_resource_flags (pdev, region);
	mmio_len = pci_resource_len (pdev, region);

	// make sure PCI base addr 1 is MMIO
	if (!(mmio_flags & IORESOURCE_MEM)) {
		printk (KERN_ERR PFX "region #%d not an MMIO resource, aborting\n", region);
		rc = -ENODEV;
		goto err_out;
	}

	// check for weird/broken PCI region reporting
	if ( mmio_len < RTL_MIN_IO_SIZE ) {
		printk (KERN_ERR PFX "Invalid PCI region size(s), aborting\n");
		rc = -ENODEV;
		goto err_out;
	}


	rc = pci_request_regions (pdev, rtdev->name);
	if (rc)
		goto err_out;

	// enable PCI bus-mastering
	pci_set_master (pdev);

#ifdef RTL8169_USE_IO
	ioaddr = pci_resource_start(pdev, 0);
#else
	// ioremap MMIO region
	ioaddr = (unsigned long)ioremap (mmio_start, mmio_len);
	if (ioaddr == 0) {
		printk (KERN_ERR PFX "cannot remap MMIO, aborting\n");
		rc = -EIO;
		goto err_out_free_res;
	}
#endif

	// Soft reset the chip.
	RTL_W8 ( ChipCmd, CmdReset);

	// Check that the chip has finished the reset.
	for (i = 1000; i > 0; i--){
		if ( (RTL_R8(ChipCmd) & CmdReset) == 0){
			break;
		}
		else{
			udelay (10);
		}
	}

	{
		u8 cfg2 = RTL_R8(Config2) & ~MSIEnable;
		if (region) {
			if (pci_enable_msi(pdev))
				printk("R8169: no MSI, Back to INTx.\n");
			else {
				cfg2 |= MSIEnable;
				priv->features |= RTL_FEATURE_MSI;
			}
		}
		RTL_W8(Config2, cfg2);
	}

	// identify config method
	{
		unsigned long val32 = (RTL_R32(TxConfig)&0x7c800000);

		if( val32 == (0x1<<28) ){
			priv->mcfg = MCFG_METHOD_4;
		}
		else if( val32 == (0x1<<26) ){
			priv->mcfg = MCFG_METHOD_3;
		}
		else if( val32 == (0x1<<23) ){
			priv->mcfg = MCFG_METHOD_2;
		}
		else if( val32 == 0x00000000 ){
			priv->mcfg = MCFG_METHOD_1;
		}
		else{
			priv->mcfg = MCFG_METHOD_1;
		}
	}

	{
		unsigned char val8 = (unsigned char)(RTL8169_READ_GMII_REG(ioaddr,3)&0x000f);
		if( val8 == 0x00 ){
			priv->pcfg = PCFG_METHOD_1;
		}
		else if( val8 == 0x01 ){
			priv->pcfg = PCFG_METHOD_2;
		}
		else if( val8 == 0x02 ){
			priv->pcfg = PCFG_METHOD_3;
		}
		else{
			priv->pcfg = PCFG_METHOD_3;
		}
	}


	for (i = ARRAY_SIZE (rtl_chip_info) - 1; i >= 0; i--){
		if (priv->mcfg == rtl_chip_info[i].mcfg) {
			priv->chipset = i;
			goto match;
		}
	}

	//if unknown chip, assume array element #0, original RTL-8169 in this case
	printk (KERN_DEBUG PFX "PCI device %s: unknown chip version, assuming RTL-8169\n", pci_name(pdev));
	priv->chipset = 0;

match:
	*ioaddr_out = ioaddr;
	*dev_out = rtdev;
	return 0;

#ifndef RTL8169_USE_IO
err_out_free_res:
#endif
	pci_release_regions (pdev);	/*** <kk> moved outside of #ifdev ***/

err_out:
	/*** RTnet ***/
	rt_rtdev_disconnect(rtdev);
	rtdev_free(rtdev);
	/*** /RTnet ***/
	return rc;
}







//======================================================================================================
static int rtl8169_init_one (struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct rtnet_device *rtdev = NULL;	/*** RTnet ***/
	struct rtl8169_private *priv = NULL;
	unsigned long ioaddr = 0;
	static int board_idx = -1;
	int region = ent->driver_data;
	int i;
	int option = -1, Cap10_100 = 0, Cap1000 = 0;


	assert (pdev != NULL);
	assert (ent != NULL);

	board_idx++;

	/*** RTnet ***/
	if (board_idx >= MAX_UNITS) {
		return -ENODEV;
	}
	if (cards[board_idx] == 0)
		return -ENODEV;
	/*** RTnet ***/

	i = rtl8169_init_board (pdev, &rtdev, &ioaddr, region);
	if (i < 0) {
		return i;
	}

	priv = rtdev->priv;

	assert (ioaddr != 0);
	assert (rtdev != NULL);
	assert (priv != NULL);

	// Get MAC address //
	for (i = 0; i < MAC_ADDR_LEN ; i++){
		rtdev->dev_addr[i] = RTL_R8( MAC0 + i );
	}

	rtdev->open		= rtl8169_open;
	rtdev->hard_start_xmit  = rtl8169_start_xmit;
	rtdev->get_stats        = rtl8169_get_stats;
	rtdev->stop             = rtl8169_close;
	/* dev->tx_timeout      = rtl8169_tx_timeout; */			/*** RTnet ***/
	/* dev->set_multicast_list = rtl8169_set_rx_mode; */	/*** RTnet ***/
	/* dev->watchdog_timeo  = TX_TIMEOUT; */				/*** RTnet ***/
	rtdev->irq              = pdev->irq;
	rtdev->base_addr                = (unsigned long) ioaddr;

#ifdef RTL8169_JUMBO_FRAME_SUPPORT
	rtdev->change_mtu		= rtl8169_change_mtu;
#endif //end #ifdef RTL8169_JUMBO_FRAME_SUPPORT

#ifdef RTL8169_IOCTL_SUPPORT
	rtdev->do_ioctl                 = rtl8169_ioctl;
#endif //end #ifdef RTL8169_IOCTL_SUPPORT

#ifdef RTL8169_DYNAMIC_CONTROL
	priv->rt.dev = rtdev;
#endif //end #ifdef RTL8169_DYNAMIC_CONTROL

	priv = rtdev->priv;				// private data //
	priv->pci_dev   = pdev;
	priv->ioaddr    = ioaddr;

//#ifdef RTL8169_JUMBO_FRAME_SUPPORT
	priv->curr_mtu_size = rtdev->mtu;
	priv->tx_pkt_len = rtdev->mtu + ETH_HDR_LEN;
	priv->rx_pkt_len = rtdev->mtu + ETH_HDR_LEN;
	priv->hw_rx_pkt_len = priv->rx_pkt_len + 8;
//#endif //end #ifdef RTL8169_JUMBO_FRAME_SUPPORT

	DBG_PRINT("-------------------------- \n");
	DBG_PRINT("dev->mtu = %d \n", rtdev->mtu);
	DBG_PRINT("priv->curr_mtu_size = %d \n", priv->curr_mtu_size);
	DBG_PRINT("priv->tx_pkt_len = %d \n", priv->tx_pkt_len);
	DBG_PRINT("priv->rx_pkt_len = %d \n", priv->rx_pkt_len);
	DBG_PRINT("priv->hw_rx_pkt_len = %d \n", priv->hw_rx_pkt_len);
	DBG_PRINT("-------------------------- \n");

	rtdm_lock_init(&priv->lock);	/*** RTnet ***/

	/*** RTnet ***/
	if (rt_register_rtnetdev(rtdev) < 0) {
		/* clean up... */
		pci_release_regions (pdev);
		rt_rtdev_disconnect(rtdev);
		rtdev_free(rtdev);
		return -ENODEV;
	}
	/*** /RTnet ***/

	pci_set_drvdata(pdev, rtdev);     //      pdev->driver_data = data;

	printk (KERN_DEBUG "%s: Identified chip type is '%s'.\n", rtdev->name, rtl_chip_info[priv->chipset].name);
	printk (KERN_INFO "%s: %s at 0x%lx, "
				"%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
				"IRQ %d\n",
				rtdev->name,
				RTL8169_DRIVER_NAME,
				rtdev->base_addr,
				rtdev->dev_addr[0], rtdev->dev_addr[1],
				rtdev->dev_addr[2], rtdev->dev_addr[3],
				rtdev->dev_addr[4], rtdev->dev_addr[5],
				rtdev->irq);

	// Config PHY
	rtl8169_hw_PHY_config(rtdev);

	DBG_PRINT("Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
	RTL_W8( 0x82, 0x01 );

	if( priv->mcfg < MCFG_METHOD_3 ){
		DBG_PRINT("Set PCI Latency=0x40\n");
		pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0x40);
	}

	if( priv->mcfg == MCFG_METHOD_2 ){
		DBG_PRINT("Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
		RTL_W8( 0x82, 0x01 );
		DBG_PRINT("Set PHY Reg 0x0bh = 0x00h\n");
		RTL8169_WRITE_GMII_REG( ioaddr, 0x0b, 0x0000 );	//w 0x0b 15 0 0
	}

	// if TBI is not endbled
	if( !(RTL_R8(PHYstatus) & TBI_Enable) ){
		int	val = RTL8169_READ_GMII_REG( ioaddr, PHY_AUTO_NEGO_REG );

#ifdef RTL8169_HW_FLOW_CONTROL_SUPPORT
		val |= PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE ;
#endif //end #define RTL8169_HW_FLOW_CONTROL_SUPPORT

		option = (board_idx >= MAX_UNITS) ? 0 : media[board_idx];
		// Force RTL8169 in 10/100/1000 Full/Half mode.
		if( option > 0 ){
			printk(KERN_INFO "%s: Force-mode Enabled. \n", rtdev->name);
			Cap10_100 = 0;
			Cap1000 = 0;
			switch( option ){
				case _10_Half:
						Cap10_100 = PHY_Cap_10_Half;
						Cap1000 = PHY_Cap_Null;
						break;
				case _10_Full:
						Cap10_100 = PHY_Cap_10_Full | PHY_Cap_10_Half;
						Cap1000 = PHY_Cap_Null;
						break;
				case _100_Half:
						Cap10_100 = PHY_Cap_100_Half | PHY_Cap_10_Full | PHY_Cap_10_Half;
						Cap1000 = PHY_Cap_Null;
						break;
				case _100_Full:
						Cap10_100 = PHY_Cap_100_Full | PHY_Cap_100_Half | PHY_Cap_10_Full | PHY_Cap_10_Half;
						Cap1000 = PHY_Cap_Null;
						break;
				case _1000_Full:
						Cap10_100 = PHY_Cap_100_Full | PHY_Cap_100_Half | PHY_Cap_10_Full | PHY_Cap_10_Half;
						Cap1000 = PHY_Cap_1000_Full;
						break;
				default:
						break;
			}
			RTL8169_WRITE_GMII_REG( ioaddr, PHY_AUTO_NEGO_REG, Cap10_100 | ( val&0xC1F ) );	//leave PHY_AUTO_NEGO_REG bit4:0 unchanged
			RTL8169_WRITE_GMII_REG( ioaddr, PHY_1000_CTRL_REG, Cap1000 );
		}
		else{
			printk(KERN_INFO "%s: Auto-negotiation Enabled.\n", rtdev->name);

			// enable 10/100 Full/Half Mode, leave PHY_AUTO_NEGO_REG bit4:0 unchanged
			RTL8169_WRITE_GMII_REG( ioaddr, PHY_AUTO_NEGO_REG,
				PHY_Cap_10_Half | PHY_Cap_10_Full | PHY_Cap_100_Half | PHY_Cap_100_Full | ( val&0xC1F ) );

			// enable 1000 Full Mode
//			RTL8169_WRITE_GMII_REG( ioaddr, PHY_1000_CTRL_REG, PHY_Cap_1000_Full );
			RTL8169_WRITE_GMII_REG( ioaddr, PHY_1000_CTRL_REG, PHY_Cap_1000_Full | PHY_Cap_1000_Half);	//rtl8168

		}// end of if( option > 0 )

		// Enable auto-negotiation and restart auto-nigotiation
		RTL8169_WRITE_GMII_REG( ioaddr, PHY_CTRL_REG, PHY_Enable_Auto_Nego | PHY_Restart_Auto_Nego );
		udelay(100);

		// wait for auto-negotiation process
		for( i = 10000; i > 0; i-- ){
			//check if auto-negotiation complete
			if( RTL8169_READ_GMII_REG(ioaddr, PHY_STAT_REG) & PHY_Auto_Neco_Comp ){
				udelay(100);
				option = RTL_R8(PHYstatus);
				if( option & _1000bpsF ){
					printk(KERN_INFO "%s: 1000Mbps Full-duplex operation.\n", rtdev->name);
				}
				else{
					printk(KERN_INFO "%s: %sMbps %s-duplex operation.\n", rtdev->name,
							(option & _100bps) ? "100" : "10", (option & FullDup) ? "Full" : "Half" );
				}
				break;
			}
			else{
				udelay(100);
			}// end of if( RTL8169_READ_GMII_REG(ioaddr, 1) & 0x20 )
		}// end for-loop to wait for auto-negotiation process

		option = RTL_R8(PHYstatus);
		if( option & _1000bpsF ){
			priv->linkstatus = _1000_Full;
		}
		else{
			if(option & _100bps){
				priv->linkstatus = (option & FullDup) ? _100_Full : _100_Half;
			}
	    else{
				priv->linkstatus = (option & FullDup) ? _10_Full : _10_Half;
			}
		}
		DBG_PRINT("priv->linkstatus = 0x%02x\n", priv->linkstatus);

	}// end of TBI is not enabled
	else{
		udelay(100);
		DBG_PRINT("1000Mbps Full-duplex operation, TBI Link %s!\n",(RTL_R32(TBICSR) & TBILinkOK) ? "OK" : "Failed" );

	}// end of TBI is not enabled

	return 0;
}







//======================================================================================================
static void rtl8169_remove_one (struct pci_dev *pdev)
{
	struct rtnet_device *rtdev = pci_get_drvdata(pdev);
	struct rtl8169_private *priv = rtdev->priv;;

	assert (rtdev != NULL);

	/*** RTnet ***/
	rt_unregister_rtnetdev(rtdev);
	rt_rtdev_disconnect(rtdev);
	/*** /RTnet ***/

	if (priv->features & RTL_FEATURE_MSI)
		pci_disable_msi(pdev);

#ifdef RTL8169_USE_IO
#else
	iounmap ((void *)(rtdev->base_addr));
#endif
	pci_release_regions(pdev);

	rtdev_free(rtdev);	/*** RTnet ***/

	pci_disable_device(pdev);	/*** <kk> Disable device now :-) ***/

	pci_set_drvdata(pdev, NULL);
}







//======================================================================================================
static int rtl8169_open (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	struct pci_dev *pdev = priv->pci_dev;
	int retval;
//	u8 diff;
//	u32 TxPhyAddr, RxPhyAddr;

	if( priv->drvinit_fail == 1 ){
		printk("%s: Gigabit driver open failed.\n", rtdev->name );
		return -ENOMEM;
	}

	/*** RTnet ***/
	rt_stack_connect(rtdev, &STACK_manager);

	retval = rtdm_irq_request(&priv->irq_handle, rtdev->irq, rtl8169_interrupt, 0, "rt_r8169", rtdev);
	/*** /RTnet ***/

	// retval = request_irq (dev->irq, rtl8169_interrupt, SA_SHIRQ, dev->name, dev);
	if (retval) {
		return retval;
	}


	//2004-05-11
	// Allocate tx/rx descriptor space
	priv->sizeof_txdesc_space = NUM_TX_DESC * sizeof(struct TxDesc)+256;
	priv->txdesc_space = pci_alloc_consistent( pdev, priv->sizeof_txdesc_space, &priv->txdesc_phy_dma_addr );
	if( priv->txdesc_space == NULL ){
		printk("%s: Gigabit driver alloc txdesc_space failed.\n", rtdev->name );
		return -ENOMEM;
	}
	priv->sizeof_rxdesc_space = NUM_RX_DESC * sizeof(struct RxDesc)+256;
	priv->rxdesc_space = pci_alloc_consistent( pdev, priv->sizeof_rxdesc_space, &priv->rxdesc_phy_dma_addr );
	if( priv->rxdesc_space == NULL ){
		printk("%s: Gigabit driver alloc rxdesc_space failed.\n", rtdev->name );
		return -ENOMEM;
	}

	if(priv->txdesc_phy_dma_addr & 0xff){
		printk("%s: Gigabit driver txdesc_phy_dma_addr is not 256-bytes-aligned.\n", rtdev->name );
	}
	if(priv->rxdesc_phy_dma_addr & 0xff){
		printk("%s: Gigabit driver rxdesc_phy_dma_addr is not 256-bytes-aligned.\n", rtdev->name );
	}
	// Set tx/rx descriptor space
	priv->TxDescArray = (struct TxDesc *)priv->txdesc_space;
	priv->RxDescArray = (struct RxDesc *)priv->rxdesc_space;

	{
		int i;
		struct rtskb *skb = NULL;	/*** RTnet ***/
		priv->rx_buf_size = (rtdev->mtu <= 1500 ? DEFAULT_RX_BUF_LEN : rtdev->mtu + 32);	/*** RTnet / <kk> ***/

		for(i=0;i<NUM_RX_DESC;i++){
			//skb = RTL8169_ALLOC_RXSKB(MAX_RX_SKBDATA_SIZE);	/*** <kk> ***/
			skb = rtnetdev_alloc_rtskb(rtdev, priv->rx_buf_size); /*** RTnet ***/;
			if( skb != NULL ) {
				rtskb_reserve (skb, 2);	// 16 byte align the IP fields. //
				priv->Rx_skbuff[i] = skb;
			}
			else{
				printk("%s: Gigabit driver failed to allocate skbuff.\n", rtdev->name);
				priv->drvinit_fail = 1;
			}
		}
	}


	//////////////////////////////////////////////////////////////////////////////
	rtl8169_init_ring(rtdev);
	rtl8169_hw_start(rtdev);

	// ------------------------------------------------------

	//DBG_PRINT("%s: %s() alloc_rxskb_cnt = %d\n", dev->name, __FUNCTION__, alloc_rxskb_cnt );	/*** <kk> won't work anymore... ***/

	return 0;

}//end of rtl8169_open (struct net_device *dev)








//======================================================================================================



//======================================================================================================
static void rtl8169_hw_PHY_config (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	void *ioaddr = (void*)priv->ioaddr;

	DBG_PRINT("priv->mcfg=%d, priv->pcfg=%d\n",priv->mcfg,priv->pcfg);

	if( priv->mcfg == MCFG_METHOD_4 ){
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1F, 0x0001 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1b, 0x841e );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x0e, 0x7bfb );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x09, 0x273a );

		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1F, 0x0002 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0x90D0 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1F, 0x0000 );
	}else if((priv->mcfg == MCFG_METHOD_2)||(priv->mcfg == MCFG_METHOD_3)){
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1F, 0x0001 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x15, 0x1000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x18, 0x65C7 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x0000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x03, 0x00A1 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x02, 0x0008 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0x1020 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x00, 0x1000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x0800 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x0000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x7000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x03, 0xFF41 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x02, 0xDE60 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0x0140 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x00, 0x0077 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x7800 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x7000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xA000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x03, 0xDF01 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x02, 0xDF20 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0xFF95 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x00, 0xFA00 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xA800 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xA000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xB000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x03, 0xFF41 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x02, 0xDE20 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0x0140 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x00, 0x00BB );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xB800 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xB000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xF000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x03, 0xDF01 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x02, 0xDF20 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x01, 0xFF95 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x00, 0xBF00 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xF800 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0xF000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x04, 0x0000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x1F, 0x0000 );
		RTL8169_WRITE_GMII_REG( (unsigned long)ioaddr, 0x0B, 0x0000 );
	}
	else{
		DBG_PRINT("priv->mcfg=%d. Discard hw PHY config.\n",priv->mcfg);
	}
}










//======================================================================================================
static void rtl8169_hw_start (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	unsigned long ioaddr = priv->ioaddr;
	u32 i;


	/* Soft reset the chip. */
	RTL_W8 ( ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--){
		if ((RTL_R8( ChipCmd ) & CmdReset) == 0) break;
		else udelay (10);
	}

	RTL_W8 ( Cfg9346, Cfg9346_Unlock);
	RTL_W8 ( ChipCmd, CmdTxEnb | CmdRxEnb);
	RTL_W8 ( ETThReg, ETTh);

	// For gigabit rtl8169
	RTL_W16	( RxMaxSize, (unsigned short)priv->hw_rx_pkt_len );

	// Set Rx Config register
	i = rtl8169_rx_config | ( RTL_R32( RxConfig ) & rtl_chip_info[priv->chipset].RxConfigMask);
	RTL_W32 ( RxConfig, i);


	/* Set DMA burst size and Interframe Gap Time */
	RTL_W32 ( TxConfig, (TX_DMA_BURST << TxDMAShift) | (InterFrameGap << TxInterFrameGapShift) );



	RTL_W16( CPlusCmd, RTL_R16(CPlusCmd) );

	if(	priv->mcfg == MCFG_METHOD_2 ||
		priv->mcfg == MCFG_METHOD_3)
	{
		RTL_W16( CPlusCmd, (RTL_R16(CPlusCmd)|(1<<14)|(1<<3)) );
		DBG_PRINT("Set MAC Reg C+CR Offset 0xE0: bit-3 and bit-14\n");
	}
	else
	{
		RTL_W16( CPlusCmd, (RTL_R16(CPlusCmd)|(1<<3)) );
		DBG_PRINT("Set MAC Reg C+CR Offset 0xE0: bit-3.\n");
	}

	{
		//RTL_W16(0xE2, 0x1517);
		//RTL_W16(0xE2, 0x152a);
		//RTL_W16(0xE2, 0x282a);
		RTL_W16(0xE2, 0x0000);		/* 0xE2 = IntrMitigate */
	}

	priv->cur_rx = 0;

	RTL_W32 ( TxDescStartAddr, priv->txdesc_phy_dma_addr);
	RTL_W32 ( TxDescStartAddr + 4, 0x00);
	RTL_W32 ( RxDescStartAddr, priv->rxdesc_phy_dma_addr);
	RTL_W32 ( RxDescStartAddr + 4, 0x00);

	RTL_W8 ( Cfg9346, Cfg9346_Lock );
	udelay (10);

	RTL_W32 ( RxMissed, 0 );

	rtl8169_set_rx_mode (rtdev);

	/* no early-rx interrupts */
	RTL_W16 ( MultiIntr, RTL_R16(MultiIntr) & 0xF000);

	/* enable all known interrupts by setting the interrupt mask */
	RTL_W16 ( IntrMask, rtl8169_intr_mask);

	rtnetif_start_queue (rtdev);	/*** RTnet ***/

}//end of rtl8169_hw_start (struct net_device *dev)







//======================================================================================================
static void rtl8169_init_ring (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	struct pci_dev *pdev = priv->pci_dev;
	int i;
	struct rtskb	*skb;


	priv->cur_rx = 0;
	priv->cur_tx = 0;
	priv->dirty_tx = 0;
	memset(priv->TxDescArray, 0x0, NUM_TX_DESC*sizeof(struct TxDesc));
	memset(priv->RxDescArray, 0x0, NUM_RX_DESC*sizeof(struct RxDesc));


	for (i=0 ; i<NUM_TX_DESC ; i++){
		priv->Tx_skbuff[i]=NULL;
		priv->txdesc_array_dma_addr[i] = pci_map_single(pdev, &priv->TxDescArray[i], sizeof(struct TxDesc), PCI_DMA_TODEVICE);
	}

	for (i=0; i <NUM_RX_DESC; i++) {
		if(i==(NUM_RX_DESC-1)){
			priv->RxDescArray[i].status = cpu_to_le32((OWNbit | EORbit) | (unsigned long)priv->hw_rx_pkt_len);
		}
		else{
			priv->RxDescArray[i].status = cpu_to_le32(OWNbit | (unsigned long)priv->hw_rx_pkt_len);
		}

		{//-----------------------------------------------------------------------
			skb = priv->Rx_skbuff[i];
			priv->rx_skbuff_dma_addr[i] = pci_map_single(pdev, skb->data, priv->rx_buf_size /* MAX_RX_SKBDATA_SIZE */, PCI_DMA_FROMDEVICE);	/*** <kk> ***/

			if( skb != NULL ){
				priv->RxDescArray[i].buf_addr = cpu_to_le32(priv->rx_skbuff_dma_addr[i]);
				priv->RxDescArray[i].buf_Haddr = 0;
			}
			else{
				DBG_PRINT("%s: %s() Rx_skbuff == NULL\n", rtdev->name, __FUNCTION__);
				priv->drvinit_fail = 1;
			}
		}//-----------------------------------------------------------------------
		priv->rxdesc_array_dma_addr[i] = pci_map_single(pdev, &priv->RxDescArray[i], sizeof(struct RxDesc), PCI_DMA_TODEVICE);
		pci_dma_sync_single_for_device(pdev, priv->rxdesc_array_dma_addr[i], sizeof(struct RxDesc), PCI_DMA_TODEVICE);
	}
}







//======================================================================================================
static void rtl8169_tx_clear (struct rtl8169_private *priv)
{
	int i;

	priv->cur_tx = 0;
	for ( i = 0 ; i < NUM_TX_DESC ; i++ ){
		if ( priv->Tx_skbuff[i] != NULL ) {
			dev_kfree_rtskb ( priv->Tx_skbuff[i] );
			priv->Tx_skbuff[i] = NULL;
			priv->stats.tx_dropped++;
		}
	}
}







//======================================================================================================






//======================================================================================================
static int rtl8169_start_xmit (struct rtskb *skb, struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	unsigned long ioaddr = priv->ioaddr;
	struct pci_dev *pdev = priv->pci_dev;
	int entry = priv->cur_tx % NUM_TX_DESC;
	// int buf_len = 60;
	dma_addr_t txbuf_dma_addr;
	rtdm_lockctx_t context;	/*** RTnet ***/
	u32 status, len;		/* <kk> */

	rtdm_lock_get_irqsave(&priv->lock, context);	/*** RTnet ***/

	status = le32_to_cpu(priv->TxDescArray[entry].status);

	if( (status & OWNbit)==0 ){

		priv->Tx_skbuff[entry] = skb;

		len = skb->len;
		if (len < ETH_ZLEN) {
			skb = rtskb_padto(skb, ETH_ZLEN);
			if (skb == NULL) {
				/* Error... */
				rtdm_printk("%s: Error -- rtskb_padto returned NULL; out of memory?\n", rtdev->name);
			}
			len = ETH_ZLEN;
		}

		txbuf_dma_addr = pci_map_single(pdev, skb->data, len, PCI_DMA_TODEVICE);

		priv->TxDescArray[entry].buf_addr = cpu_to_le32(txbuf_dma_addr);

		/* <kk> print TX frame debug informations? */
		while (r8169_debug & (DEBUG_TX_SYNC | DEBUG_TX_OTHER)) {
			unsigned short proto = 0;

			/* get ethernet protocol id */
			if (skb->len < 14) break;	/* packet too small! */
			if (skb->len > 12) proto = be16_to_cpu(*((unsigned short *)(skb->data + 12)));

			if (proto == 0x9021 && !(r8169_debug & DEBUG_TX_SYNC)) {
				/* don't show TDMA Sync frames for better debugging, so look at RTmac frame type... */
				unsigned short type;

				if (skb->len < 16) break;	/* packet too small! */
				type = be16_to_cpu(*((unsigned short *)(skb->data + 14)));

				if (type == 0x0001) {
					/* TDMA-Frame; get Message ID */
					unsigned short tdma_version;

					if (skb->len < 20) break;	/* packet too small! */
					tdma_version = be16_to_cpu(*((unsigned short *)(skb->data + 18)));

					if (tdma_version == 0x0201) {
						unsigned short tdma_id;

						if (skb->len < 22) break;	/* packet too small! */
						tdma_id = be16_to_cpu(*((unsigned short *)(skb->data + 20)));

						if (tdma_id == 0x0000 && !(r8169_debug & DEBUG_TX_SYNC)) {
							/* TDMA sync frame found, but not allowed to print it */
							break;
						}
					}
				}

			}

			/* print frame informations */
			RT_DBG_PRINT("%s: TX len = %d, skb->len = %d, eth_proto=%04x\n", __FUNCTION__, len, skb->len, proto);

			break;	/* leave loop */
		}

		if( len > priv->tx_pkt_len ){
			rtdm_printk("%s: Error -- Tx packet size(%d) > mtu(%d)+14\n", rtdev->name, len, rtdev->mtu);
			len = priv->tx_pkt_len;
		}

		/*** RTnet ***/
		/* get and patch time stamp just before the transmission */
		if (skb->xmit_stamp)
			*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() + *skb->xmit_stamp);
		/*** /RTnet ***/

		if( entry != (NUM_TX_DESC-1) ){
			status = (OWNbit | FSbit | LSbit) | len;
		}
		else{
			status = (OWNbit | EORbit | FSbit | LSbit) | len;
		}
		priv->TxDescArray[entry].status = cpu_to_le32(status);

		pci_dma_sync_single_for_device(pdev, priv->txdesc_array_dma_addr[entry], sizeof(struct TxDesc), PCI_DMA_TODEVICE);

		RTL_W8 ( TxPoll, 0x40);		//set polling bit

		//rtdev->trans_start = jiffies;

		priv->stats.tx_bytes += len;
		priv->cur_tx++;
	}//end of if( (priv->TxDescArray[entry].status & 0x80000000)==0 )

	rtdm_lock_put_irqrestore(&priv->lock, context);	/*** RTnet ***/

	if ( (priv->cur_tx - NUM_TX_DESC) == priv->dirty_tx ){
		if (r8169_debug & DEBUG_RUN) rtdm_printk(KERN_DEBUG "%s: stopping rtnetif queue", __FUNCTION__);
		rtnetif_stop_queue (rtdev);
	}
	else{
		if (rtnetif_queue_stopped (rtdev)){
			if (r8169_debug & DEBUG_RUN) rtdm_printk(KERN_DEBUG "%s: waking rtnetif queue", __FUNCTION__);
			rtnetif_wake_queue (rtdev);
		}
	}

	return 0;
}







//======================================================================================================
/* This routine is logically part of the interrupt handler, but isolated
   for clarity. */
static void rtl8169_tx_interrupt (struct rtnet_device *rtdev, struct rtl8169_private *priv, unsigned long ioaddr)
{
	unsigned long dirty_tx, tx_left=0;
	//int entry = priv->cur_tx % NUM_TX_DESC;	/* <kk> */
	int txloop_cnt = 0;

	rt_assert (rtdev != NULL);
	rt_assert (priv != NULL);
	rt_assert (ioaddr != 0);

	rtdm_lock_get(&priv->lock); /*** RTnet ***/

	dirty_tx = priv->dirty_tx;
	smp_rmb();	/*** <kk> ***/
	tx_left = priv->cur_tx - dirty_tx;

	while( (tx_left > 0) && (txloop_cnt < max_interrupt_work) ){
		unsigned int entry = dirty_tx % NUM_TX_DESC;	/* <kk> */
		if( (le32_to_cpu(priv->TxDescArray[entry].status) & OWNbit) == 0 ){

#ifdef RTL8169_DYNAMIC_CONTROL
			r8169_callback_tx(&(priv->rt), 1, priv->Tx_skbuff[dirty_tx % NUM_TX_DESC]->len);
#endif //end #ifdef RTL8169_DYNAMIC_CONTROL

			if (priv->txdesc_array_dma_addr[entry])
				pci_unmap_single(priv->pci_dev, priv->txdesc_array_dma_addr[entry], priv->Tx_skbuff[entry]->len, PCI_DMA_TODEVICE);	/*** ##KK## ***/
			dev_kfree_rtskb( priv->Tx_skbuff[entry] );	/*** RTnet; previously: dev_kfree_skb_irq() - luckily we're within an IRQ ***/
			priv->Tx_skbuff[entry] = NULL;
			priv->stats.tx_packets++;
			dirty_tx++;
			tx_left--;
			entry++;
		}
		txloop_cnt ++;
	}

	if (priv->dirty_tx != dirty_tx) {
		priv->dirty_tx = dirty_tx;
		smp_wmb();	/*** <kk> ***/
		if (rtnetif_queue_stopped (rtdev))
			rtnetif_wake_queue (rtdev);
	}

	rtdm_lock_put(&priv->lock); /*** RTnet ***/

}






//======================================================================================================
/* This routine is logically part of the interrupt handler, but isolated
   for clarity. */
static void rtl8169_rx_interrupt (struct rtnet_device *rtdev, struct rtl8169_private *priv, unsigned long ioaddr, nanosecs_abs_t *time_stamp)
{
	struct pci_dev *pdev = priv->pci_dev;
	int cur_rx;
	int pkt_size = 0 ;
	int rxdesc_cnt = 0;
	/* int ret; */	/*** RTnet ***/
	struct rtskb *n_skb = NULL;
	struct rtskb *cur_skb;
	struct rtskb *rx_skb;
	struct	RxDesc	*rxdesc;

	rt_assert (rtdev != NULL);
	rt_assert (priv != NULL);
	rt_assert (ioaddr != 0);


	cur_rx = priv->cur_rx;

	rxdesc = &priv->RxDescArray[cur_rx];
	pci_dma_sync_single_for_cpu(pdev, priv->rxdesc_array_dma_addr[cur_rx], sizeof(struct RxDesc), PCI_DMA_FROMDEVICE);

	while ( ((le32_to_cpu(rxdesc->status) & OWNbit)== 0) && (rxdesc_cnt < max_interrupt_work) ){

	    rxdesc_cnt++;

	    if( le32_to_cpu(rxdesc->status) & RxRES ){
			rtdm_printk(KERN_INFO "%s: Rx ERROR!!!\n", rtdev->name);
			priv->stats.rx_errors++;
			if ( le32_to_cpu(rxdesc->status) & (RxRWT|RxRUNT) )
				priv->stats.rx_length_errors++;
			if ( le32_to_cpu(rxdesc->status) & RxCRC) {
				/* in the rt_via-rhine.c there's a lock around the incrementation... we'll do that also here <kk> */
				rtdm_lock_get(&priv->lock); /*** RTnet ***/
				priv->stats.rx_crc_errors++;
				rtdm_lock_put(&priv->lock); /*** RTnet ***/
			}
	    }
	    else{
			pkt_size=(int)(le32_to_cpu(rxdesc->status) & 0x00001FFF)-4;

			if( pkt_size > priv->rx_pkt_len ){
				rtdm_printk("%s: Error -- Rx packet size(%d) > mtu(%d)+14\n", rtdev->name, pkt_size, rtdev->mtu);
				pkt_size = priv->rx_pkt_len;
			}

			{// -----------------------------------------------------
				rx_skb = priv->Rx_skbuff[cur_rx];
				// n_skb = RTL8169_ALLOC_RXSKB(MAX_RX_SKBDATA_SIZE);	/*** <kk> ***/
				n_skb = rtnetdev_alloc_rtskb(rtdev, priv->rx_buf_size);	/*** RTnet ***/
				if( n_skb != NULL ) {
					rtskb_reserve (n_skb, 2);	// 16 byte align the IP fields. //

					// Indicate rx_skb
					if( rx_skb != NULL ){
						pci_dma_sync_single_for_cpu(pdev, priv->rx_skbuff_dma_addr[cur_rx], sizeof(struct RxDesc), PCI_DMA_FROMDEVICE);

						rtskb_put ( rx_skb, pkt_size );
						rx_skb->protocol = rt_eth_type_trans ( rx_skb, rtdev );
						rx_skb->time_stamp = *time_stamp;	/*** RTnet ***/
						//ret = RTL8169_NETIF_RX (rx_skb);
						rtnetif_rx(rx_skb);	/*** RTnet ***/

//						dev->last_rx = jiffies;
						priv->stats.rx_bytes += pkt_size;
						priv->stats.rx_packets++;

#ifdef RTL8169_DYNAMIC_CONTROL
						r8169_callback_rx( &(priv->rt), 1, pkt_size);
#endif //end #ifdef RTL8169_DYNAMIC_CONTROL

					}//end if( rx_skb != NULL )

					priv->Rx_skbuff[cur_rx] = n_skb;
				}
				else{
					RT_DBG_PRINT("%s: Allocate n_skb failed! (priv->rx_buf_size = %d)\n",__FUNCTION__, priv->rx_buf_size );
					priv->Rx_skbuff[cur_rx] = rx_skb;
				}


				// Update rx descriptor
				if( cur_rx == (NUM_RX_DESC-1) ){
					priv->RxDescArray[cur_rx].status  = cpu_to_le32((OWNbit | EORbit) | (unsigned long)priv->hw_rx_pkt_len);
				}
				else{
					priv->RxDescArray[cur_rx].status  = cpu_to_le32(OWNbit | (unsigned long)priv->hw_rx_pkt_len);
				}

				cur_skb = priv->Rx_skbuff[cur_rx];

				if( cur_skb != NULL ){
					priv->rx_skbuff_dma_addr[cur_rx] = pci_map_single(pdev, cur_skb->data, priv->rx_buf_size /* <kk> MAX_RX_SKBDATA_SIZE */, PCI_DMA_FROMDEVICE);
					rxdesc->buf_addr = cpu_to_le32(priv->rx_skbuff_dma_addr[cur_rx]);
				}
				else{
					RT_DBG_PRINT("%s: %s() cur_skb == NULL\n", rtdev->name, __FUNCTION__);
				}

			}//------------------------------------------------------------

	    }// end of if( priv->RxDescArray[cur_rx].status & RxRES )

	    cur_rx = (cur_rx +1) % NUM_RX_DESC;
	    rxdesc = &priv->RxDescArray[cur_rx];
	    pci_dma_sync_single_for_cpu(pdev, priv->rxdesc_array_dma_addr[cur_rx], sizeof(struct RxDesc), PCI_DMA_FROMDEVICE);

	}// end of while ( (priv->RxDescArray[cur_rx].status & 0x80000000)== 0)

	if( rxdesc_cnt >= max_interrupt_work ){
		RT_DBG_PRINT("%s: Too much work at Rx interrupt.\n", rtdev->name);
	}

	priv->cur_rx = cur_rx;
}








//======================================================================================================
/* The interrupt handler does all of the Rx thread work and cleans up after the Tx thread. */
static int rtl8169_interrupt(rtdm_irq_t *irq_handle)
{
	/* struct net_device *dev = (struct net_device *) dev_instance; */	/*** RTnet ***/
	struct rtnet_device *rtdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device); /*** RTnet ***/
	struct rtl8169_private *priv = rtdev->priv;
	int boguscnt = max_interrupt_work;
	unsigned long ioaddr = priv->ioaddr;
	int status = 0;
	unsigned int old_packet_cnt = priv->stats.rx_packets; /*** RTnet ***/
	nanosecs_abs_t time_stamp = rtdm_clock_read(); /*** RTnet ***/

	int interrupt_handled = RTDM_IRQ_NONE; /*** <kk> ***/

	do {
		status = RTL_R16(IntrStatus);	/* read interrupt status */

		if ((status == 0xFFFF) || (!status)) {
			break;						/* hotplug/major error/no more work/shared irq */
		}


		interrupt_handled = RTDM_IRQ_HANDLED;

/*		if (unlikely(!rtnetif_running(rtdev))) {
			rtl8169_asic_down(ioaddr);
			goto out;
		}
*/

		/* Acknowledge interrupts */
		RTL_W16(IntrStatus, 0xffff);

		if (!(status & rtl8169_intr_mask)) {
			break;
		}

		if (unlikely(status & SYSErr)) {
			RT_DBG_PRINT("PCI error...!? %i\n", __LINE__);
			rtl8169_pcierr_interrupt(rtdev);
			break;
		}

		/*** RTnet / <kk> (Linux-2.6.12-Backport) ***/
		if (unlikely(status & LinkChg)) {
			rtdm_lock_get(&priv->lock);
			if (RTL_R8(PHYstatus) & LinkStatus)	/*** <kk> only supporting XMII, not yet TBI ***/
				rtnetif_carrier_on(rtdev);
			else
				rtnetif_carrier_off(rtdev);
			rtdm_lock_put(&priv->lock);
		}

		// Rx interrupt
		if (status & (RxOK | RxOverflow | RxFIFOOver)) {
			rtl8169_rx_interrupt (rtdev, priv, ioaddr, &time_stamp);
		}

		// Tx interrupt
		if (status & (TxOK | TxErr)) {
			rtl8169_tx_interrupt (rtdev, priv, ioaddr);
		}

		boguscnt--;
	} while (boguscnt > 0);

	if (boguscnt <= 0) {
		rtdm_printk(KERN_WARNING "%s: Too much work at interrupt!\n", rtdev->name);
		RTL_W16( IntrStatus, 0xffff);	/* Clear all interrupt sources */
	}

//out:

	if (old_packet_cnt != priv->stats.rx_packets)
		rt_mark_stack_mgr(rtdev);
	return interrupt_handled;
}







//======================================================================================================
static int rtl8169_close (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	unsigned long ioaddr = priv->ioaddr;
	int i;
	rtdm_lockctx_t context;	/*** RTnet, for rtdm_lock_get_irqsave ***/

	// -----------------------------------------
	/* rtl8169_delete_timer( &(priv->r8169_timer) ); */	/*** RTnet ***/


	rtdm_lock_get_irqsave (&priv->lock, context);	/*** RTnet ***/

	rtnetif_stop_queue (rtdev);		/*** RTnet / <kk>: moved behind spin_lock! ***/

	/* Stop the chip's Tx and Rx processes. */
	RTL_W8 ( ChipCmd, 0x00);

	/* Disable interrupts by clearing the interrupt mask. */
	RTL_W16 ( IntrMask, 0x0000);

	/* Update the error counts. */
	priv->stats.rx_missed_errors += RTL_R32(RxMissed);
	RTL_W32( RxMissed, 0);

	rtdm_lock_put_irqrestore(&priv->lock, context);	/*** RTnet ***/

	/*** RTnet ***/
	if ( (i=rtdm_irq_free(&priv->irq_handle))<0 )
		return i;

	rt_stack_disconnect(rtdev);
	/*** /RTnet ***/

	rtl8169_tx_clear (priv);

	//2004-05-11
	if(priv->txdesc_space != NULL){
		pci_free_consistent(
				priv->pci_dev,
				priv->sizeof_txdesc_space,
				priv->txdesc_space,
				priv->txdesc_phy_dma_addr
		);
		priv->txdesc_space = NULL;
	}

	if(priv->rxdesc_space != NULL){
		pci_free_consistent(
				priv->pci_dev,
				priv->sizeof_rxdesc_space,
				priv->rxdesc_space,
				priv->rxdesc_phy_dma_addr
		);
		priv->rxdesc_space = NULL;
	}

	priv->TxDescArray = NULL;
	priv->RxDescArray = NULL;

	{//-----------------------------------------------------------------------------
		for(i=0;i<NUM_RX_DESC;i++){
			if( priv->Rx_skbuff[i] != NULL ) {
				//RTL8169_FREE_RXSKB ( priv->Rx_skbuff[i] );	/*** <kk> ***/
				dev_kfree_rtskb(priv->Rx_skbuff[i]);	/*** RTnet ***/
			}
		}
	}//-----------------------------------------------------------------------------

	//DBG_PRINT("%s: %s() alloc_rxskb_cnt = %d\n", dev->name, __FUNCTION__, alloc_rxskb_cnt );	/*** <kk> won't work anymore ***/

	return 0;
}







//======================================================================================================
static unsigned const ethernet_polynomial = 0x04c11db7U;
static inline u32 ether_crc (int length, unsigned char *data)
{
	int crc = -1;

	while (--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^ ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
	}

	return crc;
}








//======================================================================================================
static void rtl8169_set_rx_mode (struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	unsigned long ioaddr = priv->ioaddr;
	rtdm_lockctx_t context;
	u32 mc_filter[2];	/* Multicast hash filter */
	int rx_mode;
	u32 tmp=0;


	if (rtdev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		printk (KERN_NOTICE "%s: Promiscuous mode enabled.\n", rtdev->name);
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else if (rtdev->flags & IFF_ALLMULTI) {
		/* Too many to filter perfectly -- accept all multicasts. */
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0;
	}

	rtdm_lock_get_irqsave(&priv->lock, context);			/*** RTnet ***/

	tmp = rtl8169_rx_config | rx_mode | (RTL_R32(RxConfig) & rtl_chip_info[priv->chipset].RxConfigMask);

	RTL_W32 ( RxConfig, tmp);
	RTL_W32 ( MAR0 + 0, mc_filter[0]);
	RTL_W32 ( MAR0 + 4, mc_filter[1]);

	rtdm_lock_put_irqrestore(&priv->lock, context);	/*** RTnet ***/

}//end of rtl8169_set_rx_mode (struct net_device *dev)







//================================================================================
static struct net_device_stats *rtl8169_get_stats(struct rtnet_device *rtdev)

{
    struct rtl8169_private *priv = rtdev->priv;

    return &priv->stats;
}







//================================================================================
static struct pci_driver rtl8169_pci_driver = {
	name:		MODULENAME,
	id_table:	rtl8169_pci_tbl,
	probe:		rtl8169_init_one,
	remove:		rtl8169_remove_one,
	suspend:	NULL,
	resume:		NULL,
};





//======================================================================================================
static int __init rtl8169_init_module (void)
{
	/* <kk> Enable debugging output... */
	if (local_debug > 0) {
		r8169_debug = local_debug;
	}
	if (r8169_debug & DEBUG_RUN) printk("Initializing " MODULENAME " driver");
	return pci_register_driver (&rtl8169_pci_driver);
}




//======================================================================================================
static void __exit rtl8169_cleanup_module (void)
{
	pci_unregister_driver (&rtl8169_pci_driver);
}


#ifdef RTL8169_JUMBO_FRAME_SUPPORT
static int rtl8169_change_mtu(struct net_device *dev, int new_mtu)
{
	struct rtl8169_private *priv = dev->priv;
	unsigned long ioaddr = priv->ioaddr;

	if( new_mtu > MAX_JUMBO_FRAME_MTU ){
		printk("%s: Error -- new_mtu(%d) > MAX_JUMBO_FRAME_MTU(%d).\n", dev->name, new_mtu, MAX_JUMBO_FRAME_MTU);
		return -1;
	}

	dev->mtu = new_mtu;

	priv->curr_mtu_size = new_mtu;
	priv->tx_pkt_len = new_mtu + ETH_HDR_LEN;
	priv->rx_pkt_len = new_mtu + ETH_HDR_LEN;
	priv->hw_rx_pkt_len = priv->rx_pkt_len + 8;

	RTL_W8 ( Cfg9346, Cfg9346_Unlock);
	RTL_W16	( RxMaxSize, (unsigned short)priv->hw_rx_pkt_len );
	RTL_W8 ( Cfg9346, Cfg9346_Lock);

	DBG_PRINT("-------------------------- \n");
	DBG_PRINT("dev->mtu = %d \n", dev->mtu);
	DBG_PRINT("priv->curr_mtu_size = %d \n", priv->curr_mtu_size);
	DBG_PRINT("priv->rx_pkt_len = %d \n", priv->rx_pkt_len);
	DBG_PRINT("priv->tx_pkt_len = %d \n", priv->tx_pkt_len);
	DBG_PRINT("RTL_W16( RxMaxSize, %d )\n", priv->hw_rx_pkt_len);
	DBG_PRINT("-------------------------- \n");

	rtl8169_close (dev);
	rtl8169_open (dev);

	return 0;
}
#endif //end #ifdef RTL8169_JUMBO_FRAME_SUPPORT



/*** <kk> these functions are backported from Linux-2.6.12's r8169.c driver ***/
static void rtl8169_irq_mask_and_ack(unsigned long ioaddr)
{
	RTL_W16(IntrMask, 0x0000);

	RTL_W16(IntrStatus, 0xffff);
}

static void rtl8169_pcierr_interrupt(struct rtnet_device *rtdev)
{
	struct rtl8169_private *priv = rtdev->priv;
	struct pci_dev *pdev = priv->pci_dev;
	unsigned long ioaddr = priv->ioaddr;
	u16 pci_status, pci_cmd;

	pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);
	pci_read_config_word(pdev, PCI_STATUS, &pci_status);

	rtdm_printk(KERN_ERR PFX "%s: PCI error (cmd = 0x%04x, status = 0x%04x).\n",
	       rtdev->name, pci_cmd, pci_status);

	/*
	 * The recovery sequence below admits a very elaborated explanation:
	 * - it seems to work;
	 * - I did not see what else could be done.
	 *
	 * Feel free to adjust to your needs.
	 */
	pci_write_config_word(pdev, PCI_COMMAND,
			      pci_cmd | PCI_COMMAND_SERR | PCI_COMMAND_PARITY);

	pci_write_config_word(pdev, PCI_STATUS,
		pci_status & (PCI_STATUS_DETECTED_PARITY |
		PCI_STATUS_SIG_SYSTEM_ERROR | PCI_STATUS_REC_MASTER_ABORT |
		PCI_STATUS_REC_TARGET_ABORT | PCI_STATUS_SIG_TARGET_ABORT));

	/* The infamous DAC f*ckup only happens at boot time */
	/*** <kk> ***
	if ((priv->cp_cmd & PCIDAC) && !priv->dirty_rx && !priv->cur_rx) {
		rtdm_printk(KERN_INFO PFX "%s: disabling PCI DAC.\n", rtdev->name);
		priv->cp_cmd &= ~PCIDAC;
		RTL_W16(CPlusCmd, priv->cp_cmd);
		rtdev->features &= ~NETIF_F_HIGHDMA;
		rtl8169_schedule_work(rtdev, rtl8169_reinit_task);
	}
	 *** /RTnet ***/

	/* Disable interrupts */
	rtl8169_irq_mask_and_ack(ioaddr);

	/* Reset the chipset */
	RTL_W8(ChipCmd, CmdReset);

	/* PCI commit */
	RTL_R8(ChipCmd);

}






//======================================================================================================
module_init(rtl8169_init_module);
module_exit(rtl8169_cleanup_module);
