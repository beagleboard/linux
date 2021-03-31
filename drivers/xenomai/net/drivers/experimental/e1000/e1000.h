/*******************************************************************************

  Intel PRO/1000 Linux driver
  Copyright(c) 1999 - 2008 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _E1000_H_
#define _E1000_H_

#include "kcompat.h"

#include "e1000_api.h"

#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

#define INTEL_E1000_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

struct e1000_adapter;

#define E1000_DBG(args...)

#define E1000_ERR(args...) printk(KERN_ERR "e1000: " args)

#define PFX "e1000: "
#define DPRINTK(nlevel, klevel, fmt, args...) \
	(void)((NETIF_MSG_##nlevel & adapter->msg_enable) && \
	printk(KERN_##klevel PFX "%s: %s: " fmt, adapter->netdev->name, \
		__FUNCTION__ , ## args))

#define E1000_MAX_INTR 10

/* TX/RX descriptor defines */
#define E1000_DEFAULT_TXD                  256
#define E1000_MAX_TXD                      256
#define E1000_MIN_TXD                       80
#define E1000_MAX_82544_TXD               4096

#define E1000_DEFAULT_RXD                  256
#define E1000_MAX_RXD                      256

#define E1000_MIN_RXD                       80
#define E1000_MAX_82544_RXD               4096

#define E1000_MIN_ITR_USECS                 10 /* 100000 irq/sec */
#define E1000_MAX_ITR_USECS              10000 /* 100    irq/sec */

#ifdef CONFIG_E1000_MQ
#define E1000_MAX_TX_QUEUES                  4
#endif

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define E1000_RXBUFFER_128   128    /* Used for packet split */
#define E1000_RXBUFFER_256   256    /* Used for packet split */
#define E1000_RXBUFFER_512   512
#define E1000_RXBUFFER_1024  1024
#define E1000_RXBUFFER_2048  2048
#define E1000_RXBUFFER_4096  4096
#define E1000_RXBUFFER_8192  8192
#define E1000_RXBUFFER_16384 16384

/* SmartSpeed delimiters */
#define E1000_SMARTSPEED_DOWNSHIFT 3
#define E1000_SMARTSPEED_MAX       15

/* Packet Buffer allocations */
#define E1000_PBA_BYTES_SHIFT 0xA
#define E1000_TX_HEAD_ADDR_SHIFT 7
#define E1000_PBA_TX_MASK 0xFFFF0000

/* Early Receive defines */
#define E1000_ERT_2048 0x100

#define E1000_FC_PAUSE_TIME 0x0680 /* 858 usec */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16	/* Must be power of 2 */

#define AUTO_ALL_MODES            0
#define E1000_EEPROM_82544_APM    0x0004
#define E1000_EEPROM_APME         0x0400

#ifndef E1000_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define E1000_MASTER_SLAVE	e1000_ms_hw_default
#endif

#ifdef NETIF_F_HW_VLAN_TX
#define E1000_MNG_VLAN_NONE -1
#endif
/* Number of packet split data buffers (not including the header buffer) */
#define PS_PAGE_BUFFERS MAX_PS_BUFFERS-1

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct e1000_buffer {
	struct rtskb *skb;
	dma_addr_t dma;
	unsigned long time_stamp;
	u16 length;
	u16 next_to_watch;
};

struct e1000_rx_buffer {
	struct rtskb *skb;
	dma_addr_t dma;
	struct page *page;
};

#ifdef CONFIG_E1000_MQ
struct e1000_queue_stats {
	u64 packets;
	u64 bytes;
};
#endif

struct e1000_ps_page { struct page *ps_page[PS_PAGE_BUFFERS]; };
struct e1000_ps_page_dma { u64 ps_page_dma[PS_PAGE_BUFFERS]; };

struct e1000_tx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct e1000_buffer *buffer_info;

#ifdef CONFIG_E1000_MQ
	/* for tx ring cleanup - needed for multiqueue */
	spinlock_t tx_queue_lock;
#endif
	rtdm_lock_t tx_lock;
	u16 tdh;
	u16 tdt;
#ifdef CONFIG_E1000_MQ
	struct e1000_queue_stats tx_stats;
#endif
	bool last_tx_tso;
};

struct e1000_rx_ring {
	struct e1000_adapter *adapter; /* back link */
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
#ifdef CONFIG_E1000_NAPI
	struct napi_struct napi;
#endif
	/* array of buffer information structs */
	struct e1000_rx_buffer *buffer_info;
	/* arrays of page information for packet split */
	struct e1000_ps_page *ps_page;
	struct e1000_ps_page_dma *ps_page_dma;
	struct sk_buff *rx_skb_top;

	/* cpu for rx queue */
	int cpu;

	u16 rdh;
	u16 rdt;
#ifdef CONFIG_E1000_MQ
	struct e1000_queue_stats rx_stats;
#endif
};

#define E1000_DESC_UNUSED(R) \
	((((R)->next_to_clean > (R)->next_to_use) ? 0 : (R)->count) + \
	(R)->next_to_clean - (R)->next_to_use - 1)

#define E1000_RX_DESC_PS(R, i)	    \
	(&(((union e1000_rx_desc_packet_split *)((R).desc))[i]))
#define E1000_RX_DESC_EXT(R, i)	    \
	(&(((union e1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)

#ifdef SIOCGMIIPHY
/* PHY register snapshot values */
struct e1000_phy_regs {
	u16 bmcr;		/* basic mode control register    */
	u16 bmsr;		/* basic mode status register     */
	u16 advertise;		/* auto-negotiation advertisement */
	u16 lpa;		/* link partner ability register  */
	u16 expansion;		/* auto-negotiation expansion reg */
	u16 ctrl1000;		/* 1000BASE-T control register    */
	u16 stat1000;		/* 1000BASE-T status register     */
	u16 estatus;		/* extended status register       */
};
#endif

/* board specific private data structure */

struct e1000_adapter {
#ifdef NETIF_F_HW_VLAN_TX
	struct vlan_group *vlgrp;
	u16 mng_vlan_id;
#endif
	u32 bd_number;
	u32 rx_buffer_len;
	u32 wol;
	u32 smartspeed;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;
	rtdm_lock_t  stats_lock;
#ifdef CONFIG_E1000_NAPI
	spinlock_t tx_queue_lock;
#endif
	atomic_t irq_sem;
	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;
	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
	u16 tx_itr;
	u16 rx_itr;

	bool fc_autoneg;

#ifdef ETHTOOL_PHYS_ID
	struct timer_list blink_timer;
	unsigned long led_status;
#endif

	/* TX */
	struct e1000_tx_ring *tx_ring;      /* One per active queue */
#ifdef CONFIG_E1000_MQ
	struct e1000_tx_ring **cpu_tx_ring; /* per-cpu */
#endif
	unsigned int restart_queue;
	unsigned long tx_queue_len;
	u32 txd_cmd;
	u32 tx_int_delay;
	u32 tx_abs_int_delay;
	u32 gotc;
	u64 gotc_old;
	u64 tpt_old;
	u64 colc_old;
	u32 tx_timeout_count;
	u32 tx_fifo_head;
	u32 tx_head_addr;
	u32 tx_fifo_size;
	u8 tx_timeout_factor;
	atomic_t tx_fifo_stall;
	bool pcix_82544;
	bool detect_tx_hung;

	/* RX */
#ifdef CONFIG_E1000_NAPI
	bool (*clean_rx) (struct e1000_adapter *adapter,
			       struct e1000_rx_ring *rx_ring,
			       int *work_done, int work_to_do);
#else
	bool (*clean_rx) (struct e1000_adapter *adapter,
			       struct e1000_rx_ring *rx_ring,
			       nanosecs_abs_t *time_stamp);
#endif
	void (*alloc_rx_buf) (struct e1000_adapter *adapter,
			      struct e1000_rx_ring *rx_ring,
				int cleaned_count);
	struct e1000_rx_ring *rx_ring;      /* One per active queue */
#ifdef CONFIG_E1000_NAPI
	//struct napi_struct napi;
#endif
	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u64 rx_hdr_split;
	u32 alloc_rx_buff_failed;
	u32 rx_int_delay;
	u32 rx_abs_int_delay;
	bool rx_csum;
	unsigned int rx_ps_pages;
	u32 gorc;
	u64 gorc_old;
	u16 rx_ps_bsize0;
	u32 max_frame_size;
	u32 min_frame_size;


	/* OS defined structs */
	struct rtnet_device *netdev;
	struct pci_dev *pdev;
	struct net_device_stats net_stats;

	rtdm_irq_t irq_handle;
	char  data_received;

	/* structs defined in e1000_hw.h */
	struct e1000_hw hw;
	struct e1000_hw_stats stats;
	struct e1000_phy_info phy_info;
	struct e1000_phy_stats phy_stats;

#ifdef SIOCGMIIPHY
	/* Snapshot of PHY registers */
	struct e1000_phy_regs phy_regs;
#endif

#ifdef ETHTOOL_TEST
	u32 test_icr;
	struct e1000_tx_ring test_tx_ring;
	struct e1000_rx_ring test_rx_ring;
#endif


	int msg_enable;
	/* to not mess up cache alignment, always add to the bottom */
	unsigned long state;
	u32 eeprom_wol;

	u32 *config_space;

	/* hardware capability, feature, and workaround flags */
	unsigned int flags;

	struct work_struct reset_task;
	struct delayed_work watchdog_task;
	struct delayed_work fifo_stall_task;
	struct delayed_work phy_info_task;
};

#define E1000_FLAG_HAS_SMBUS                (1 << 0)
#define E1000_FLAG_HAS_MANC2H               (1 << 1)
#define E1000_FLAG_HAS_MSI                  (1 << 2)
#define E1000_FLAG_MSI_ENABLED              (1 << 3)
#define E1000_FLAG_HAS_INTR_MODERATION      (1 << 4)
#define E1000_FLAG_RX_NEEDS_RESTART         (1 << 5)
#define E1000_FLAG_BAD_TX_CARRIER_STATS_FD  (1 << 6)
#define E1000_FLAG_INT_ASSERT_AUTO_MASK     (1 << 7)
#define E1000_FLAG_QUAD_PORT_A              (1 << 8)
#define E1000_FLAG_SMART_POWER_DOWN         (1 << 9)
#ifdef NETIF_F_TSO
#define E1000_FLAG_HAS_TSO                  (1 << 10)
#ifdef NETIF_F_TSO6
#define E1000_FLAG_HAS_TSO6                 (1 << 11)
#endif
#define E1000_FLAG_TSO_FORCE                (1 << 12)
#endif
#define E1000_FLAG_RX_RESTART_NOW           (1 << 13)

enum e1000_state_t {
	__E1000_TESTING,
	__E1000_RESETTING,
	__E1000_DOWN
};

extern char e1000_driver_name[];
extern const char e1000_driver_version[];

extern void e1000_power_up_phy(struct e1000_hw *hw);

extern void e1000_set_ethtool_ops(struct net_device *netdev);
extern void e1000_check_options(struct e1000_adapter *adapter);

extern int e1000_up(struct e1000_adapter *adapter);
extern void e1000_down(struct e1000_adapter *adapter);
extern void e1000_reinit_locked(struct e1000_adapter *adapter);
extern void e1000_reset(struct e1000_adapter *adapter);
extern int e1000_set_spd_dplx(struct e1000_adapter *adapter, u16 spddplx);
extern int e1000_setup_all_rx_resources(struct e1000_adapter *adapter);
extern int e1000_setup_all_tx_resources(struct e1000_adapter *adapter);
extern void e1000_free_all_rx_resources(struct e1000_adapter *adapter);
extern void e1000_free_all_tx_resources(struct e1000_adapter *adapter);
extern void e1000_update_stats(struct e1000_adapter *adapter);
#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *ifr);
#endif

#endif /* _E1000_H_ */
