/* Intel(R) Gigabit Ethernet Linux driver
 * Copyright(c) 2007-2015 Intel Corporation.
 * RTnet port   2009 Vladimir Zapolskiy <vladimir.zapolskiy@siemens.com>
 * Copyright(c) 2015 Gilles Chanteperdrix <gch@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/netdevice.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/net_tstamp.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/sctp.h>
#include <linux/if_ether.h>
#include <linux/aer.h>
#include <linux/prefetch.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include "igb.h"

#include <rtnet_port.h>

// RTNET redefines
#ifdef  NETIF_F_TSO
#undef  NETIF_F_TSO
#define NETIF_F_TSO 0
#endif

#ifdef  NETIF_F_TSO6
#undef  NETIF_F_TSO6
#define NETIF_F_TSO6 0
#endif

#ifdef  NETIF_F_HW_VLAN_TX
#undef  NETIF_F_HW_VLAN_TX
#define NETIF_F_HW_VLAN_TX 0
#endif

#ifdef  NETIF_F_HW_VLAN_RX
#undef  NETIF_F_HW_VLAN_RX
#define NETIF_F_HW_VLAN_RX 0
#endif

#ifdef  NETIF_F_HW_VLAN_FILTER
#undef  NETIF_F_HW_VLAN_FILTER
#define NETIF_F_HW_VLAN_FILTER 0
#endif

#ifdef  IGB_MAX_TX_QUEUES
#undef  IGB_MAX_TX_QUEUES
#define IGB_MAX_TX_QUEUES 1
#endif

#ifdef  IGB_MAX_RX_QUEUES
#undef  IGB_MAX_RX_QUEUES
#define IGB_MAX_RX_QUEUES 1
#endif

#ifdef CONFIG_IGB_NAPI
#undef CONFIG_IGB_NAPI
#endif

#ifdef IGB_HAVE_TX_TIMEOUT
#undef IGB_HAVE_TX_TIMEOUT
#endif

#ifdef ETHTOOL_GPERMADDR
#undef ETHTOOL_GPERMADDR
#endif

#ifdef CONFIG_PM
#undef CONFIG_PM
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
#undef CONFIG_NET_POLL_CONTROLLER
#endif

#ifdef MAX_SKB_FRAGS
#undef MAX_SKB_FRAGS
#define MAX_SKB_FRAGS 1
#endif

#ifdef IGB_FRAMES_SUPPORT
#undef IGB_FRAMES_SUPPORT
#endif

#define MAJ 5
#define MIN 2
#define BUILD 18
#define DRV_VERSION __stringify(MAJ) "." __stringify(MIN) "." \
__stringify(BUILD) "-k"
char igb_driver_name[] = "rt_igb";
char igb_driver_version[] = DRV_VERSION;
static const char igb_driver_string[] =
				"Intel(R) Gigabit Ethernet Network Driver";
static const char igb_copyright[] =
				"Copyright (c) 2007-2014 Intel Corporation.";

static const struct e1000_info *igb_info_tbl[] = {
	[board_82575] = &e1000_82575_info,
};

#define MAX_UNITS 8
static int InterruptThrottle = 0;
module_param(InterruptThrottle, uint, 0);
MODULE_PARM_DESC(InterruptThrottle, "Throttle interrupts (boolean, false by default)");

static const struct pci_device_id igb_pci_tbl[] = {
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I354_BACKPLANE_1GBPS) },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I354_SGMII) },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I354_BACKPLANE_2_5GBPS) },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I211_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_FIBER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_SGMII), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_COPPER_FLASHLESS), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I210_SERDES_FLASHLESS), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I350_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I350_FIBER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I350_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_I350_SGMII), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_FIBER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_QUAD_FIBER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_SGMII), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82580_COPPER_DUAL), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_DH89XXCC_SGMII), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_DH89XXCC_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_DH89XXCC_BACKPLANE), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_DH89XXCC_SFP), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_NS), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_NS_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_FIBER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_SERDES_QUAD), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_QUAD_COPPER_ET2), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82576_QUAD_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82575EB_COPPER), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82575EB_FIBER_SERDES), board_82575 },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_82575GB_QUAD_COPPER), board_82575 },
	/* required last entry */
	{0, }
};

MODULE_DEVICE_TABLE(pci, igb_pci_tbl);

static int igb_setup_all_tx_resources(struct igb_adapter *);
static int igb_setup_all_rx_resources(struct igb_adapter *);
static void igb_free_all_tx_resources(struct igb_adapter *);
static void igb_free_all_rx_resources(struct igb_adapter *);
static void igb_setup_mrqc(struct igb_adapter *);
static int igb_probe(struct pci_dev *, const struct pci_device_id *);
static void igb_remove(struct pci_dev *pdev);
static int igb_sw_init(struct igb_adapter *);
static int igb_open(struct rtnet_device *);
static int igb_close(struct rtnet_device *);
static void igb_configure(struct igb_adapter *);
static void igb_configure_tx(struct igb_adapter *);
static void igb_configure_rx(struct igb_adapter *);
static void igb_clean_all_tx_rings(struct igb_adapter *);
static void igb_clean_all_rx_rings(struct igb_adapter *);
static void igb_clean_tx_ring(struct igb_ring *);
static void igb_clean_rx_ring(struct igb_ring *);
static void igb_set_rx_mode(struct rtnet_device *);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static void igb_update_phy_info(struct timer_list *);
static void igb_watchdog(struct timer_list *);
#else
static void igb_update_phy_info(unsigned long);
static void igb_watchdog(unsigned long);
#endif
static void igb_watchdog_task(struct work_struct *);
static netdev_tx_t igb_xmit_frame(struct rtskb *skb, struct rtnet_device *);
static struct net_device_stats *igb_get_stats(struct rtnet_device *);
static int igb_intr(rtdm_irq_t *irq_handle);
static int igb_intr_msi(rtdm_irq_t *irq_handle);
static void igb_nrtsig_watchdog(rtdm_nrtsig_t *sig, void *data);
static irqreturn_t igb_msix_other(int irq, void *);
static int igb_msix_ring(rtdm_irq_t *irq_handle);
static void igb_poll(struct igb_q_vector *);
static bool igb_clean_tx_irq(struct igb_q_vector *);
static bool igb_clean_rx_irq(struct igb_q_vector *, int);
static int igb_ioctl(struct rtnet_device *, struct ifreq *ifr, int cmd);
static void igb_reset_task(struct work_struct *);
static void igb_vlan_mode(struct rtnet_device *netdev,
			  netdev_features_t features);
static int igb_vlan_rx_add_vid(struct rtnet_device *, __be16, u16);
static void igb_restore_vlan(struct igb_adapter *);
static void igb_rar_set_qsel(struct igb_adapter *, u8 *, u32 , u8);

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
static int igb_suspend(struct device *);
#endif
static int igb_resume(struct device *);
static int igb_runtime_suspend(struct device *dev);
static int igb_runtime_resume(struct device *dev);
static int igb_runtime_idle(struct device *dev);
static const struct dev_pm_ops igb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(igb_suspend, igb_resume)
	SET_RUNTIME_PM_OPS(igb_runtime_suspend, igb_runtime_resume,
			igb_runtime_idle)
};
#endif
static void igb_shutdown(struct pci_dev *);
static int igb_pci_sriov_configure(struct pci_dev *dev, int num_vfs);
#ifdef CONFIG_NET_POLL_CONTROLLER
/* for netdump / net console */
static void igb_netpoll(struct rtnet_device *);
#endif

static pci_ers_result_t igb_io_error_detected(struct pci_dev *,
		     pci_channel_state_t);
static pci_ers_result_t igb_io_slot_reset(struct pci_dev *);
static void igb_io_resume(struct pci_dev *);

static const struct pci_error_handlers igb_err_handler = {
	.error_detected = igb_io_error_detected,
	.slot_reset = igb_io_slot_reset,
	.resume = igb_io_resume,
};

static void igb_init_dmac(struct igb_adapter *adapter, u32 pba);

static struct pci_driver igb_driver = {
	.name     = igb_driver_name,
	.id_table = igb_pci_tbl,
	.probe    = igb_probe,
	.remove   = igb_remove,
#ifdef CONFIG_PM
	.driver.pm = &igb_pm_ops,
#endif
	.shutdown = igb_shutdown,
	.sriov_configure = igb_pci_sriov_configure,
	.err_handler = &igb_err_handler
};

MODULE_AUTHOR("Intel Corporation, <e1000-devel@lists.sourceforge.net>");
MODULE_DESCRIPTION("Intel(R) Gigabit Ethernet Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)
static int local_debug = -1;
module_param_named(debug, local_debug, int, 0);
MODULE_PARM_DESC(debug, "debug level (0=none,...,16=all)");

struct igb_reg_info {
	u32 ofs;
	char *name;
};

static const struct igb_reg_info igb_reg_info_tbl[] = {

	/* General Registers */
	{E1000_CTRL, "CTRL"},
	{E1000_STATUS, "STATUS"},
	{E1000_CTRL_EXT, "CTRL_EXT"},

	/* Interrupt Registers */
	{E1000_ICR, "ICR"},

	/* RX Registers */
	{E1000_RCTL, "RCTL"},
	{E1000_RDLEN(0), "RDLEN"},
	{E1000_RDH(0), "RDH"},
	{E1000_RDT(0), "RDT"},
	{E1000_RXDCTL(0), "RXDCTL"},
	{E1000_RDBAL(0), "RDBAL"},
	{E1000_RDBAH(0), "RDBAH"},

	/* TX Registers */
	{E1000_TCTL, "TCTL"},
	{E1000_TDBAL(0), "TDBAL"},
	{E1000_TDBAH(0), "TDBAH"},
	{E1000_TDLEN(0), "TDLEN"},
	{E1000_TDH(0), "TDH"},
	{E1000_TDT(0), "TDT"},
	{E1000_TXDCTL(0), "TXDCTL"},
	{E1000_TDFH, "TDFH"},
	{E1000_TDFT, "TDFT"},
	{E1000_TDFHS, "TDFHS"},
	{E1000_TDFPC, "TDFPC"},

	/* List Terminator */
	{}
};

/* igb_regdump - register printout routine */
static void igb_regdump(struct e1000_hw *hw, struct igb_reg_info *reginfo)
{
	int n = 0;
	char rname[16];
	u32 regs[8];

	switch (reginfo->ofs) {
	case E1000_RDLEN(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDLEN(n));
		break;
	case E1000_RDH(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDH(n));
		break;
	case E1000_RDT(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDT(n));
		break;
	case E1000_RXDCTL(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RXDCTL(n));
		break;
	case E1000_RDBAL(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDBAL(n));
		break;
	case E1000_RDBAH(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDBAH(n));
		break;
	case E1000_TDBAL(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_RDBAL(n));
		break;
	case E1000_TDBAH(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_TDBAH(n));
		break;
	case E1000_TDLEN(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_TDLEN(n));
		break;
	case E1000_TDH(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_TDH(n));
		break;
	case E1000_TDT(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_TDT(n));
		break;
	case E1000_TXDCTL(0):
		for (n = 0; n < 4; n++)
			regs[n] = rd32(E1000_TXDCTL(n));
		break;
	default:
		pr_info("%-15s %08x\n", reginfo->name, rd32(reginfo->ofs));
		return;
	}

	snprintf(rname, 16, "%s%s", reginfo->name, "[0-3]");
	pr_info("%-15s %08x %08x %08x %08x\n", rname, regs[0], regs[1],
		regs[2], regs[3]);
}

/* igb_dump - Print registers, Tx-rings and Rx-rings */
static void igb_dump(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	struct igb_reg_info *reginfo;
	struct igb_ring *tx_ring;
	union e1000_adv_tx_desc *tx_desc;
	struct my_u0 { u64 a; u64 b; } *u0;
	struct igb_ring *rx_ring;
	union e1000_adv_rx_desc *rx_desc;
	u32 staterr;
	u16 i, n;

	/* Print netdevice Info */
	if (netdev) {
		dev_info(&adapter->pdev->dev, "Net device Info\n");
		pr_info("Device Name\n");
		pr_info("%s\n", netdev->name);
	}

	/* Print Registers */
	dev_info(&adapter->pdev->dev, "Register Dump\n");
	pr_info(" Register Name   Value\n");
	for (reginfo = (struct igb_reg_info *)igb_reg_info_tbl;
	     reginfo->name; reginfo++) {
		igb_regdump(hw, reginfo);
	}

	/* Print TX Ring Summary */
	if (!netdev || !rtnetif_running(netdev))
		goto exit;

	dev_info(&adapter->pdev->dev, "TX Rings Summary\n");
	pr_info("Queue [NTU] [NTC] [bi(ntc)->dma  ] leng ntw timestamp\n");
	for (n = 0; n < adapter->num_tx_queues; n++) {
		struct igb_tx_buffer *buffer_info;
		tx_ring = adapter->tx_ring[n];
		buffer_info = &tx_ring->tx_buffer_info[tx_ring->next_to_clean];
		pr_info(" %5d %5X %5X %p %016llX\n",
			n, tx_ring->next_to_use, tx_ring->next_to_clean,
			buffer_info->next_to_watch,
			(u64)buffer_info->time_stamp);
	}

	dev_info(&adapter->pdev->dev, "TX Rings Dump\n");

	/* Transmit Descriptor Formats
	 *
	 * Advanced Transmit Descriptor
	 *   +--------------------------------------------------------------+
	 * 0 |         Buffer Address [63:0]                                |
	 *   +--------------------------------------------------------------+
	 * 8 | PAYLEN  | PORTS  |CC|IDX | STA | DCMD  |DTYP|MAC|RSV| DTALEN |
	 *   +--------------------------------------------------------------+
	 *   63      46 45    40 39 38 36 35 32 31   24             15       0
	 */

	for (n = 0; n < adapter->num_tx_queues; n++) {
		tx_ring = adapter->tx_ring[n];
		pr_info("------------------------------------\n");
		pr_info("TX QUEUE INDEX = %d\n", tx_ring->queue_index);
		pr_info("------------------------------------\n");
		pr_info("T [desc]     [address 63:0  ] [PlPOCIStDDM Ln] "
			"[bi->dma       ] leng  ntw timestamp        "
			"bi->skb\n");

		for (i = 0; tx_ring->desc && (i < tx_ring->count); i++) {
			const char *next_desc;
			struct igb_tx_buffer *buffer_info;
			tx_desc = IGB_TX_DESC(tx_ring, i);
			buffer_info = &tx_ring->tx_buffer_info[i];
			u0 = (struct my_u0 *)tx_desc;
			if (i == tx_ring->next_to_use &&
			    i == tx_ring->next_to_clean)
				next_desc = " NTC/U";
			else if (i == tx_ring->next_to_use)
				next_desc = " NTU";
			else if (i == tx_ring->next_to_clean)
				next_desc = " NTC";
			else
				next_desc = "";

			pr_info("T [0x%03X]    %016llX %016llX"
				"  %p %016llX %p%s\n", i,
				le64_to_cpu(u0->a),
				le64_to_cpu(u0->b),
				buffer_info->next_to_watch,
				(u64)buffer_info->time_stamp,
				buffer_info->skb, next_desc);

			if (buffer_info->skb)
				print_hex_dump(KERN_INFO, "",
					DUMP_PREFIX_ADDRESS,
					16, 1, buffer_info->skb->data,
					14,
					true);
		}
	}

	/* Print RX Rings Summary */
	dev_info(&adapter->pdev->dev, "RX Rings Summary\n");
	pr_info("Queue [NTU] [NTC]\n");
	for (n = 0; n < adapter->num_rx_queues; n++) {
		rx_ring = adapter->rx_ring[n];
		pr_info(" %5d %5X %5X\n",
			n, rx_ring->next_to_use, rx_ring->next_to_clean);
	}

	/* Print RX Rings */
	dev_info(&adapter->pdev->dev, "RX Rings Dump\n");

	/* Advanced Receive Descriptor (Read) Format
	 *    63                                           1        0
	 *    +-----------------------------------------------------+
	 *  0 |       Packet Buffer Address [63:1]           |A0/NSE|
	 *    +----------------------------------------------+------+
	 *  8 |       Header Buffer Address [63:1]           |  DD  |
	 *    +-----------------------------------------------------+
	 *
	 *
	 * Advanced Receive Descriptor (Write-Back) Format
	 *
	 *   63       48 47    32 31  30      21 20 17 16   4 3     0
	 *   +------------------------------------------------------+
	 * 0 | Packet     IP     |SPH| HDR_LEN   | RSV|Packet|  RSS |
	 *   | Checksum   Ident  |   |           |    | Type | Type |
	 *   +------------------------------------------------------+
	 * 8 | VLAN Tag | Length | Extended Error | Extended Status |
	 *   +------------------------------------------------------+
	 *   63       48 47    32 31            20 19               0
	 */

	for (n = 0; n < adapter->num_rx_queues; n++) {
		rx_ring = adapter->rx_ring[n];
		pr_info("------------------------------------\n");
		pr_info("RX QUEUE INDEX = %d\n", rx_ring->queue_index);
		pr_info("------------------------------------\n");
		pr_info("R  [desc]      [ PktBuf     A0] [  HeadBuf   DD] "
			"[bi->dma       ] [bi->skb] <-- Adv Rx Read format\n");
		pr_info("RWB[desc]      [PcsmIpSHl PtRs] [vl er S cks ln] -----"
			"----------- [bi->skb] <-- Adv Rx Write-Back format\n");

		for (i = 0; i < rx_ring->count; i++) {
			const char *next_desc;
			struct igb_rx_buffer *buffer_info;
			buffer_info = &rx_ring->rx_buffer_info[i];
			rx_desc = IGB_RX_DESC(rx_ring, i);
			u0 = (struct my_u0 *)rx_desc;
			staterr = le32_to_cpu(rx_desc->wb.upper.status_error);

			if (i == rx_ring->next_to_use)
				next_desc = " NTU";
			else if (i == rx_ring->next_to_clean)
				next_desc = " NTC";
			else
				next_desc = "";

			if (staterr & E1000_RXD_STAT_DD) {
				/* Descriptor Done */
				pr_info("%s[0x%03X]     %016llX %016llX ---------------- %s\n",
					"RWB", i,
					le64_to_cpu(u0->a),
					le64_to_cpu(u0->b),
					next_desc);
			} else {
				pr_info("%s[0x%03X]     %016llX %016llX %016llX %s\n",
					"R  ", i,
					le64_to_cpu(u0->a),
					le64_to_cpu(u0->b),
					(u64)buffer_info->dma,
					next_desc);

			}
		}
	}

exit:
	return;
}

/**
 *  igb_get_hw_dev - return device
 *  @hw: pointer to hardware structure
 *
 *  used by hardware layer to print debugging information
 **/
struct rtnet_device *igb_get_hw_dev(struct e1000_hw *hw)
{
	struct igb_adapter *adapter = hw->back;
	return adapter->netdev;
}

/**
 *  igb_init_module - Driver Registration Routine
 *
 *  igb_init_module is the first routine called when the driver is
 *  loaded. All it does is register with the PCI subsystem.
 **/
static int __init igb_init_module(void)
{
	int ret;

	pr_info("%s - version %s\n",
	       igb_driver_string, igb_driver_version);
	pr_info("%s\n", igb_copyright);

	ret = pci_register_driver(&igb_driver);
	return ret;
}

module_init(igb_init_module);

/**
 *  igb_exit_module - Driver Exit Cleanup Routine
 *
 *  igb_exit_module is called just before the driver is removed
 *  from memory.
 **/
static void __exit igb_exit_module(void)
{
	pci_unregister_driver(&igb_driver);
}

module_exit(igb_exit_module);

#define Q_IDX_82576(i) (((i & 0x1) << 3) + (i >> 1))
/**
 *  igb_cache_ring_register - Descriptor ring to register mapping
 *  @adapter: board private structure to initialize
 *
 *  Once we know the feature-set enabled for the device, we'll cache
 *  the register offset the descriptor ring is assigned to.
 **/
static void igb_cache_ring_register(struct igb_adapter *adapter)
{
	int i = 0, j = 0;
	u32 rbase_offset = 0;

	switch (adapter->hw.mac.type) {
	case e1000_82576:
		/* The queues are allocated for virtualization such that VF 0
		 * is allocated queues 0 and 8, VF 1 queues 1 and 9, etc.
		 * In order to avoid collision we start at the first free queue
		 * and continue consuming queues in the same sequence
		 */
		/* Fall through */
	case e1000_82575:
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_i210:
	case e1000_i211:
		/* Fall through */
	default:
		for (; i < adapter->num_rx_queues; i++)
			adapter->rx_ring[i]->reg_idx = rbase_offset + i;
		for (; j < adapter->num_tx_queues; j++)
			adapter->tx_ring[j]->reg_idx = rbase_offset + j;
		break;
	}
}

u32 igb_rd32(struct e1000_hw *hw, u32 reg)
{
	struct igb_adapter *igb = container_of(hw, struct igb_adapter, hw);
	u8 __iomem *hw_addr = READ_ONCE(hw->hw_addr);
	u32 value = 0;

	if (E1000_REMOVED(hw_addr))
		return ~value;

	value = readl(&hw_addr[reg]);

	/* reads should not return all F's */
	if (!(~value) && (!reg || !(~readl(hw_addr)))) {
		struct rtnet_device *netdev = igb->netdev;
		hw->hw_addr = NULL;
		rtnetif_device_detach(netdev);
		rtdev_err(netdev, "PCIe link lost, device now detached\n");
	}

	return value;
}

/**
 *  igb_write_ivar - configure ivar for given MSI-X vector
 *  @hw: pointer to the HW structure
 *  @msix_vector: vector number we are allocating to a given ring
 *  @index: row index of IVAR register to write within IVAR table
 *  @offset: column offset of in IVAR, should be multiple of 8
 *
 *  This function is intended to handle the writing of the IVAR register
 *  for adapters 82576 and newer.  The IVAR table consists of 2 columns,
 *  each containing an cause allocation for an Rx and Tx ring, and a
 *  variable number of rows depending on the number of queues supported.
 **/
static void igb_write_ivar(struct e1000_hw *hw, int msix_vector,
			   int index, int offset)
{
	u32 ivar = array_rd32(E1000_IVAR0, index);

	/* clear any bits that are currently set */
	ivar &= ~((u32)0xFF << offset);

	/* write vector and valid bit */
	ivar |= (msix_vector | E1000_IVAR_VALID) << offset;

	array_wr32(E1000_IVAR0, index, ivar);
}

#define IGB_N0_QUEUE -1
static void igb_assign_vector(struct igb_q_vector *q_vector, int msix_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;
	int rx_queue = IGB_N0_QUEUE;
	int tx_queue = IGB_N0_QUEUE;
	u32 msixbm = 0;

	if (q_vector->rx.ring)
		rx_queue = q_vector->rx.ring->reg_idx;
	if (q_vector->tx.ring)
		tx_queue = q_vector->tx.ring->reg_idx;

	switch (hw->mac.type) {
	case e1000_82575:
		/* The 82575 assigns vectors using a bitmask, which matches the
		 * bitmask for the EICR/EIMS/EIMC registers.  To assign one
		 * or more queues to a vector, we write the appropriate bits
		 * into the MSIXBM register for that vector.
		 */
		if (rx_queue > IGB_N0_QUEUE)
			msixbm = E1000_EICR_RX_QUEUE0 << rx_queue;
		if (tx_queue > IGB_N0_QUEUE)
			msixbm |= E1000_EICR_TX_QUEUE0 << tx_queue;
		if (!(adapter->flags & IGB_FLAG_HAS_MSIX) && msix_vector == 0)
			msixbm |= E1000_EIMS_OTHER;
		array_wr32(E1000_MSIXBM(0), msix_vector, msixbm);
		q_vector->eims_value = msixbm;
		break;
	case e1000_82576:
		/* 82576 uses a table that essentially consists of 2 columns
		 * with 8 rows.  The ordering is column-major so we use the
		 * lower 3 bits as the row index, and the 4th bit as the
		 * column offset.
		 */
		if (rx_queue > IGB_N0_QUEUE)
			igb_write_ivar(hw, msix_vector,
				       rx_queue & 0x7,
				       (rx_queue & 0x8) << 1);
		if (tx_queue > IGB_N0_QUEUE)
			igb_write_ivar(hw, msix_vector,
				       tx_queue & 0x7,
				       ((tx_queue & 0x8) << 1) + 8);
		q_vector->eims_value = 1 << msix_vector;
		break;
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_i210:
	case e1000_i211:
		/* On 82580 and newer adapters the scheme is similar to 82576
		 * however instead of ordering column-major we have things
		 * ordered row-major.  So we traverse the table by using
		 * bit 0 as the column offset, and the remaining bits as the
		 * row index.
		 */
		if (rx_queue > IGB_N0_QUEUE)
			igb_write_ivar(hw, msix_vector,
				       rx_queue >> 1,
				       (rx_queue & 0x1) << 4);
		if (tx_queue > IGB_N0_QUEUE)
			igb_write_ivar(hw, msix_vector,
				       tx_queue >> 1,
				       ((tx_queue & 0x1) << 4) + 8);
		q_vector->eims_value = 1 << msix_vector;
		break;
	default:
		BUG();
		break;
	}

	/* add q_vector eims value to global eims_enable_mask */
	adapter->eims_enable_mask |= q_vector->eims_value;

	/* configure q_vector to set itr on first interrupt */
	q_vector->set_itr = 1;
}

/**
 *  igb_configure_msix - Configure MSI-X hardware
 *  @adapter: board private structure to initialize
 *
 *  igb_configure_msix sets up the hardware to properly
 *  generate MSI-X interrupts.
 **/
static void igb_configure_msix(struct igb_adapter *adapter)
{
	u32 tmp;
	int i, vector = 0;
	struct e1000_hw *hw = &adapter->hw;

	adapter->eims_enable_mask = 0;

	/* set vector for other causes, i.e. link changes */
	switch (hw->mac.type) {
	case e1000_82575:
		tmp = rd32(E1000_CTRL_EXT);
		/* enable MSI-X PBA support*/
		tmp |= E1000_CTRL_EXT_PBA_CLR;

		/* Auto-Mask interrupts upon ICR read. */
		tmp |= E1000_CTRL_EXT_EIAME;
		tmp |= E1000_CTRL_EXT_IRCA;

		wr32(E1000_CTRL_EXT, tmp);

		/* enable msix_other interrupt */
		array_wr32(E1000_MSIXBM(0), vector++, E1000_EIMS_OTHER);
		adapter->eims_other = E1000_EIMS_OTHER;

		break;

	case e1000_82576:
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_i210:
	case e1000_i211:
		/* Turn on MSI-X capability first, or our settings
		 * won't stick.  And it will take days to debug.
		 */
		wr32(E1000_GPIE, E1000_GPIE_MSIX_MODE |
		     E1000_GPIE_PBA | E1000_GPIE_EIAME |
		     E1000_GPIE_NSICR);

		/* enable msix_other interrupt */
		adapter->eims_other = 1 << vector;
		tmp = (vector++ | E1000_IVAR_VALID) << 8;

		wr32(E1000_IVAR_MISC, tmp);
		break;
	default:
		/* do nothing, since nothing else supports MSI-X */
		break;
	} /* switch (hw->mac.type) */

	adapter->eims_enable_mask |= adapter->eims_other;

	for (i = 0; i < adapter->num_q_vectors; i++)
		igb_assign_vector(adapter->q_vector[i], vector++);

	wrfl();
}

/**
 *  igb_request_msix - Initialize MSI-X interrupts
 *  @adapter: board private structure to initialize
 *
 *  igb_request_msix allocates MSI-X vectors and requests interrupts from the
 *  kernel.
 **/
static int igb_request_msix(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	int i, err = 0, vector = 0, free_vector = 0;

	err = request_irq(adapter->msix_entries[vector].vector,
			  igb_msix_other, 0, netdev->name, adapter);
	if (err)
		goto err_out;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct igb_q_vector *q_vector = adapter->q_vector[i];

		vector++;

		q_vector->itr_register = hw->hw_addr + E1000_EITR(vector);

		if (q_vector->rx.ring && q_vector->tx.ring)
			sprintf(q_vector->name, "%s-TxRx-%u", netdev->name,
				q_vector->rx.ring->queue_index);
		else if (q_vector->tx.ring)
			sprintf(q_vector->name, "%s-tx-%u", netdev->name,
				q_vector->tx.ring->queue_index);
		else if (q_vector->rx.ring)
			sprintf(q_vector->name, "%s-rx-%u", netdev->name,
				q_vector->rx.ring->queue_index);
		else
			sprintf(q_vector->name, "%s-unused", netdev->name);

		err = rtdm_irq_request(&adapter->msix_irq_handle[vector],
				adapter->msix_entries[vector].vector,
				igb_msix_ring, 0, q_vector->name, q_vector);
		if (err)
			goto err_free;
	}

	igb_configure_msix(adapter);
	return 0;

err_free:
	/* free already assigned IRQs */
	free_irq(adapter->msix_entries[free_vector++].vector, adapter);

	vector--;
	for (i = 0; i < vector; i++)
		rtdm_irq_free(&adapter->msix_irq_handle[free_vector++]);
err_out:
	return err;
}

/**
 *  igb_free_q_vector - Free memory allocated for specific interrupt vector
 *  @adapter: board private structure to initialize
 *  @v_idx: Index of vector to be freed
 *
 *  This function frees the memory allocated to the q_vector.
 **/
static void igb_free_q_vector(struct igb_adapter *adapter, int v_idx)
{
	struct igb_q_vector *q_vector = adapter->q_vector[v_idx];

	adapter->q_vector[v_idx] = NULL;

	/* igb_get_stats64() might access the rings on this vector,
	 * we must wait a grace period before freeing it.
	 */
	if (q_vector)
		kfree_rcu(q_vector, rcu);
}

/**
 *  igb_reset_q_vector - Reset config for interrupt vector
 *  @adapter: board private structure to initialize
 *  @v_idx: Index of vector to be reset
 *
 *  If NAPI is enabled it will delete any references to the
 *  NAPI struct. This is preparation for igb_free_q_vector.
 **/
static void igb_reset_q_vector(struct igb_adapter *adapter, int v_idx)
{
	struct igb_q_vector *q_vector = adapter->q_vector[v_idx];

	/* Coming from igb_set_interrupt_capability, the vectors are not yet
	 * allocated. So, q_vector is NULL so we should stop here.
	 */
	if (!q_vector)
		return;

	if (q_vector->tx.ring)
		adapter->tx_ring[q_vector->tx.ring->queue_index] = NULL;

	if (q_vector->rx.ring)
		adapter->rx_ring[q_vector->rx.ring->queue_index] = NULL;
}

static void igb_reset_interrupt_capability(struct igb_adapter *adapter)
{
	int v_idx = adapter->num_q_vectors;

	if (adapter->flags & IGB_FLAG_HAS_MSIX)
		pci_disable_msix(adapter->pdev);
	else if (adapter->flags & IGB_FLAG_HAS_MSI)
		pci_disable_msi(adapter->pdev);

	while (v_idx--)
		igb_reset_q_vector(adapter, v_idx);
}

/**
 *  igb_free_q_vectors - Free memory allocated for interrupt vectors
 *  @adapter: board private structure to initialize
 *
 *  This function frees the memory allocated to the q_vectors.  In addition if
 *  NAPI is enabled it will delete any references to the NAPI struct prior
 *  to freeing the q_vector.
 **/
static void igb_free_q_vectors(struct igb_adapter *adapter)
{
	int v_idx = adapter->num_q_vectors;

	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--) {
		igb_reset_q_vector(adapter, v_idx);
		igb_free_q_vector(adapter, v_idx);
	}
}

/**
 *  igb_clear_interrupt_scheme - reset the device to a state of no interrupts
 *  @adapter: board private structure to initialize
 *
 *  This function resets the device so that it has 0 Rx queues, Tx queues, and
 *  MSI-X interrupts allocated.
 */
static void igb_clear_interrupt_scheme(struct igb_adapter *adapter)
{
	igb_free_q_vectors(adapter);
	igb_reset_interrupt_capability(adapter);
}

/**
 *  igb_set_interrupt_capability - set MSI or MSI-X if supported
 *  @adapter: board private structure to initialize
 *  @msix: boolean value of MSIX capability
 *
 *  Attempt to configure interrupts using the best available
 *  capabilities of the hardware and kernel.
 **/
static void igb_set_interrupt_capability(struct igb_adapter *adapter, bool msix)
{
	int err;
	int numvecs, i;

	if (!msix)
		goto msi_only;
	adapter->flags |= IGB_FLAG_HAS_MSIX;

	/* Number of supported queues. */
	adapter->num_rx_queues = adapter->rss_queues;
	adapter->num_tx_queues = adapter->rss_queues;

	/* start with one vector for every Rx queue */
	numvecs = adapter->num_rx_queues;

	/* if Tx handler is separate add 1 for every Tx queue */
	if (!(adapter->flags & IGB_FLAG_QUEUE_PAIRS))
		numvecs += adapter->num_tx_queues;

	/* store the number of vectors reserved for queues */
	adapter->num_q_vectors = numvecs;

	/* add 1 vector for link status interrupts */
	numvecs++;
	for (i = 0; i < numvecs; i++)
		adapter->msix_entries[i].entry = i;

	err = pci_enable_msix_range(adapter->pdev,
				    adapter->msix_entries,
				    numvecs,
				    numvecs);
	if (err > 0)
		return;

	igb_reset_interrupt_capability(adapter);

	/* If we can't do MSI-X, try MSI */
msi_only:
	adapter->flags &= ~IGB_FLAG_HAS_MSIX;
	adapter->rss_queues = 1;
	adapter->flags |= IGB_FLAG_QUEUE_PAIRS;
	adapter->num_rx_queues = 1;
	adapter->num_tx_queues = 1;
	adapter->num_q_vectors = 1;
	if (!pci_enable_msi(adapter->pdev))
		adapter->flags |= IGB_FLAG_HAS_MSI;
}

static void igb_add_ring(struct igb_ring *ring,
			 struct igb_ring_container *head)
{
	head->ring = ring;
	head->count++;
}

/**
 *  igb_alloc_q_vector - Allocate memory for a single interrupt vector
 *  @adapter: board private structure to initialize
 *  @v_count: q_vectors allocated on adapter, used for ring interleaving
 *  @v_idx: index of vector in adapter struct
 *  @txr_count: total number of Tx rings to allocate
 *  @txr_idx: index of first Tx ring to allocate
 *  @rxr_count: total number of Rx rings to allocate
 *  @rxr_idx: index of first Rx ring to allocate
 *
 *  We allocate one q_vector.  If allocation fails we return -ENOMEM.
 **/
static int igb_alloc_q_vector(struct igb_adapter *adapter,
			      int v_count, int v_idx,
			      int txr_count, int txr_idx,
			      int rxr_count, int rxr_idx)
{
	struct igb_q_vector *q_vector;
	struct igb_ring *ring;
	int ring_count, size;

	/* igb only supports 1 Tx and/or 1 Rx queue per vector */
	if (txr_count > 1 || rxr_count > 1)
		return -ENOMEM;

	ring_count = txr_count + rxr_count;
	size = sizeof(struct igb_q_vector) +
	       (sizeof(struct igb_ring) * ring_count);

	/* allocate q_vector and rings */
	q_vector = adapter->q_vector[v_idx];
	if (!q_vector)
		q_vector = kzalloc(size, GFP_KERNEL);
	else
		memset(q_vector, 0, size);
	if (!q_vector)
		return -ENOMEM;

	/* tie q_vector and adapter together */
	adapter->q_vector[v_idx] = q_vector;
	q_vector->adapter = adapter;

	/* initialize work limits */
	q_vector->tx.work_limit = adapter->tx_work_limit;

	/* initialize ITR configuration */
	q_vector->itr_register = adapter->hw.hw_addr + E1000_EITR(0);
	q_vector->itr_val = IGB_START_ITR;

	/* initialize pointer to rings */
	ring = q_vector->ring;

	/* intialize ITR */
	if (rxr_count) {
		/* rx or rx/tx vector */
		if (!adapter->rx_itr_setting || adapter->rx_itr_setting > 3)
			q_vector->itr_val = adapter->rx_itr_setting;
	} else {
		/* tx only vector */
		if (!adapter->tx_itr_setting || adapter->tx_itr_setting > 3)
			q_vector->itr_val = adapter->tx_itr_setting;
	}

	if (txr_count) {
		/* assign generic ring traits */
		ring->dev = &adapter->pdev->dev;
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Tx values */
		igb_add_ring(ring, &q_vector->tx);

		/* For 82575, context index must be unique per ring. */
		if (adapter->hw.mac.type == e1000_82575)
			set_bit(IGB_RING_FLAG_TX_CTX_IDX, &ring->flags);

		/* apply Tx specific ring traits */
		ring->count = adapter->tx_ring_count;
		ring->queue_index = txr_idx;

		/* assign ring to adapter */
		adapter->tx_ring[txr_idx] = ring;

		/* push pointer to next ring */
		ring++;
	}

	if (rxr_count) {
		/* assign generic ring traits */
		ring->dev = &adapter->pdev->dev;
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Rx values */
		igb_add_ring(ring, &q_vector->rx);

		/* set flag indicating ring supports SCTP checksum offload */
		if (adapter->hw.mac.type >= e1000_82576)
			set_bit(IGB_RING_FLAG_RX_SCTP_CSUM, &ring->flags);

		/* On i350, i354, i210, and i211, loopback VLAN packets
		 * have the tag byte-swapped.
		 */
		if (adapter->hw.mac.type >= e1000_i350)
			set_bit(IGB_RING_FLAG_RX_LB_VLAN_BSWAP, &ring->flags);

		/* apply Rx specific ring traits */
		ring->count = adapter->rx_ring_count;
		ring->queue_index = rxr_idx;

		/* assign ring to adapter */
		adapter->rx_ring[rxr_idx] = ring;
	}

	return 0;
}


/**
 *  igb_alloc_q_vectors - Allocate memory for interrupt vectors
 *  @adapter: board private structure to initialize
 *
 *  We allocate one q_vector per queue interrupt.  If allocation fails we
 *  return -ENOMEM.
 **/
static int igb_alloc_q_vectors(struct igb_adapter *adapter)
{
	int q_vectors = adapter->num_q_vectors;
	int rxr_remaining = adapter->num_rx_queues;
	int txr_remaining = adapter->num_tx_queues;
	int rxr_idx = 0, txr_idx = 0, v_idx = 0;
	int err;

	if (q_vectors >= (rxr_remaining + txr_remaining)) {
		for (; rxr_remaining; v_idx++) {
			err = igb_alloc_q_vector(adapter, q_vectors, v_idx,
						 0, 0, 1, rxr_idx);

			if (err)
				goto err_out;

			/* update counts and index */
			rxr_remaining--;
			rxr_idx++;
		}
	}

	for (; v_idx < q_vectors; v_idx++) {
		int rqpv = DIV_ROUND_UP(rxr_remaining, q_vectors - v_idx);
		int tqpv = DIV_ROUND_UP(txr_remaining, q_vectors - v_idx);

		err = igb_alloc_q_vector(adapter, q_vectors, v_idx,
					 tqpv, txr_idx, rqpv, rxr_idx);

		if (err)
			goto err_out;

		/* update counts and index */
		rxr_remaining -= rqpv;
		txr_remaining -= tqpv;
		rxr_idx++;
		txr_idx++;
	}

	return 0;

err_out:
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--)
		igb_free_q_vector(adapter, v_idx);

	return -ENOMEM;
}

/**
 *  igb_init_interrupt_scheme - initialize interrupts, allocate queues/vectors
 *  @adapter: board private structure to initialize
 *  @msix: boolean value of MSIX capability
 *
 *  This function initializes the interrupts and allocates all of the queues.
 **/
static int igb_init_interrupt_scheme(struct igb_adapter *adapter, bool msix)
{
	struct pci_dev *pdev = adapter->pdev;
	int err;

	igb_set_interrupt_capability(adapter, msix);

	err = igb_alloc_q_vectors(adapter);
	if (err) {
		dev_err(&pdev->dev, "Unable to allocate memory for vectors\n");
		goto err_alloc_q_vectors;
	}

	igb_cache_ring_register(adapter);

	return 0;

err_alloc_q_vectors:
	igb_reset_interrupt_capability(adapter);
	return err;
}

/**
 *  igb_request_irq - initialize interrupts
 *  @adapter: board private structure to initialize
 *
 *  Attempts to configure interrupts using the best available
 *  capabilities of the hardware and kernel.
 **/
static int igb_request_irq(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	int err = 0;

	rt_stack_connect(netdev, &STACK_manager);

	if (adapter->flags & IGB_FLAG_HAS_MSIX) {
		err = igb_request_msix(adapter);
		if (!err)
			goto request_done;
		/* fall back to MSI */
		igb_free_all_tx_resources(adapter);
		igb_free_all_rx_resources(adapter);

		igb_clear_interrupt_scheme(adapter);
		err = igb_init_interrupt_scheme(adapter, false);
		if (err)
			goto request_done;

		igb_setup_all_tx_resources(adapter);
		igb_setup_all_rx_resources(adapter);
		igb_configure(adapter);
	}

	igb_assign_vector(adapter->q_vector[0], 0);

	if (adapter->flags & IGB_FLAG_HAS_MSI) {
		err = rtdm_irq_request(&adapter->irq_handle,
				pdev->irq, igb_intr_msi, 0,
				netdev->name, adapter);
		if (!err)
			goto request_done;

		/* fall back to legacy interrupts */
		igb_reset_interrupt_capability(adapter);
		adapter->flags &= ~IGB_FLAG_HAS_MSI;
	}

	err = rtdm_irq_request(&adapter->irq_handle,
			pdev->irq, igb_intr, IRQF_SHARED,
			netdev->name, adapter);

	if (err)
		dev_err(&pdev->dev, "Error %d getting interrupt\n",
			err);

request_done:
	return err;
}

static void igb_free_irq(struct igb_adapter *adapter)
{
	if (adapter->flags & IGB_FLAG_HAS_MSIX) {
		int vector = 0, i;

		free_irq(adapter->msix_entries[vector++].vector, adapter);

		for (i = 0; i < adapter->num_q_vectors; i++)
			rtdm_irq_free(&adapter->msix_irq_handle[vector++]);
	} else {
		rtdm_irq_free(&adapter->irq_handle);
	}
}

/**
 *  igb_irq_disable - Mask off interrupt generation on the NIC
 *  @adapter: board private structure
 **/
static void igb_irq_disable(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	/* we need to be careful when disabling interrupts.  The VFs are also
	 * mapped into these registers and so clearing the bits can cause
	 * issues on the VF drivers so we only need to clear what we set
	 */
	if (adapter->flags & IGB_FLAG_HAS_MSIX) {
		u32 regval = rd32(E1000_EIAM);

		wr32(E1000_EIAM, regval & ~adapter->eims_enable_mask);
		wr32(E1000_EIMC, adapter->eims_enable_mask);
		regval = rd32(E1000_EIAC);
		wr32(E1000_EIAC, regval & ~adapter->eims_enable_mask);
	}

	wr32(E1000_IAM, 0);
	wr32(E1000_IMC, ~0);
	wrfl();

	msleep(10);
}

/**
 *  igb_irq_enable - Enable default interrupt generation settings
 *  @adapter: board private structure
 **/
static void igb_irq_enable(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	if (adapter->flags & IGB_FLAG_HAS_MSIX) {
		u32 ims = E1000_IMS_LSC | E1000_IMS_DOUTSYNC | E1000_IMS_DRSTA;
		u32 regval = rd32(E1000_EIAC);

		wr32(E1000_EIAC, regval | adapter->eims_enable_mask);
		regval = rd32(E1000_EIAM);
		wr32(E1000_EIAM, regval | adapter->eims_enable_mask);
		wr32(E1000_EIMS, adapter->eims_enable_mask);
		wr32(E1000_IMS, ims);
	} else {
		wr32(E1000_IMS, IMS_ENABLE_MASK |
				E1000_IMS_DRSTA);
		wr32(E1000_IAM, IMS_ENABLE_MASK |
				E1000_IMS_DRSTA);
	}
}

static void igb_update_mng_vlan(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 vid = adapter->hw.mng_cookie.vlan_id;
	u16 old_vid = adapter->mng_vlan_id;

	if (hw->mng_cookie.status & E1000_MNG_DHCP_COOKIE_STATUS_VLAN) {
		/* add VID to filter table */
		igb_vfta_set(hw, vid, true);
		adapter->mng_vlan_id = vid;
	} else {
		adapter->mng_vlan_id = IGB_MNG_VLAN_NONE;
	}

	if ((old_vid != (u16)IGB_MNG_VLAN_NONE) &&
	    (vid != old_vid) &&
	    !test_bit(old_vid, adapter->active_vlans)) {
		/* remove VID from filter table */
		igb_vfta_set(hw, old_vid, false);
	}
}

/**
 *  igb_release_hw_control - release control of the h/w to f/w
 *  @adapter: address of board private structure
 *
 *  igb_release_hw_control resets CTRL_EXT:DRV_LOAD bit.
 *  For ASF and Pass Through versions of f/w this means that the
 *  driver is no longer loaded.
 **/
static void igb_release_hw_control(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_ext;

	/* Let firmware take over control of h/w */
	ctrl_ext = rd32(E1000_CTRL_EXT);
	wr32(E1000_CTRL_EXT,
			ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
}

/**
 *  igb_get_hw_control - get control of the h/w from f/w
 *  @adapter: address of board private structure
 *
 *  igb_get_hw_control sets CTRL_EXT:DRV_LOAD bit.
 *  For ASF and Pass Through versions of f/w this means that
 *  the driver is loaded.
 **/
static void igb_get_hw_control(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_ext;

	/* Let firmware know the driver has taken over */
	ctrl_ext = rd32(E1000_CTRL_EXT);
	wr32(E1000_CTRL_EXT,
			ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
}

/**
 *  igb_configure - configure the hardware for RX and TX
 *  @adapter: private board structure
 **/
static void igb_configure(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	int i;

	igb_get_hw_control(adapter);
	igb_set_rx_mode(netdev);

	igb_restore_vlan(adapter);

	igb_setup_tctl(adapter);
	igb_setup_mrqc(adapter);
	igb_setup_rctl(adapter);

	igb_configure_tx(adapter);
	igb_configure_rx(adapter);

	igb_rx_fifo_flush_82575(&adapter->hw);

	/* call igb_desc_unused which always leaves
	 * at least 1 descriptor unused to make sure
	 * next_to_use != next_to_clean
	 */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct igb_ring *ring = adapter->rx_ring[i];
		igb_alloc_rx_buffers(ring, igb_desc_unused(ring));
	}
}

/**
 *  igb_power_up_link - Power up the phy/serdes link
 *  @adapter: address of board private structure
 **/
void igb_power_up_link(struct igb_adapter *adapter)
{
	igb_reset_phy(&adapter->hw);

	if (adapter->hw.phy.media_type == e1000_media_type_copper)
		igb_power_up_phy_copper(&adapter->hw);
	else
		igb_power_up_serdes_link_82575(&adapter->hw);

	igb_setup_link(&adapter->hw);
}

/**
 *  igb_power_down_link - Power down the phy/serdes link
 *  @adapter: address of board private structure
 */
static void igb_power_down_link(struct igb_adapter *adapter)
{
	if (adapter->hw.phy.media_type == e1000_media_type_copper)
		igb_power_down_phy_copper_82575(&adapter->hw);
	else
		igb_shutdown_serdes_link_82575(&adapter->hw);
}

/**
 * Detect and switch function for Media Auto Sense
 * @adapter: address of the board private structure
 **/
static void igb_check_swap_media(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_ext, connsw;
	bool swap_now = false;

	ctrl_ext = rd32(E1000_CTRL_EXT);
	connsw = rd32(E1000_CONNSW);

	/* need to live swap if current media is copper and we have fiber/serdes
	 * to go to.
	 */

	if ((hw->phy.media_type == e1000_media_type_copper) &&
	    (!(connsw & E1000_CONNSW_AUTOSENSE_EN))) {
		swap_now = true;
	} else if (!(connsw & E1000_CONNSW_SERDESD)) {
		/* copper signal takes time to appear */
		if (adapter->copper_tries < 4) {
			adapter->copper_tries++;
			connsw |= E1000_CONNSW_AUTOSENSE_CONF;
			wr32(E1000_CONNSW, connsw);
			return;
		} else {
			adapter->copper_tries = 0;
			if ((connsw & E1000_CONNSW_PHYSD) &&
			    (!(connsw & E1000_CONNSW_PHY_PDN))) {
				swap_now = true;
				connsw &= ~E1000_CONNSW_AUTOSENSE_CONF;
				wr32(E1000_CONNSW, connsw);
			}
		}
	}

	if (!swap_now)
		return;

	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		rtdev_info(adapter->netdev,
			"MAS: changing media to fiber/serdes\n");
		ctrl_ext |=
			E1000_CTRL_EXT_LINK_MODE_PCIE_SERDES;
		adapter->flags |= IGB_FLAG_MEDIA_RESET;
		adapter->copper_tries = 0;
		break;
	case e1000_media_type_internal_serdes:
	case e1000_media_type_fiber:
		rtdev_info(adapter->netdev,
			"MAS: changing media to copper\n");
		ctrl_ext &=
			~E1000_CTRL_EXT_LINK_MODE_PCIE_SERDES;
		adapter->flags |= IGB_FLAG_MEDIA_RESET;
		break;
	default:
		/* shouldn't get here during regular operation */
		rtdev_err(adapter->netdev,
			"AMS: Invalid media type found, returning\n");
		break;
	}
	wr32(E1000_CTRL_EXT, ctrl_ext);
}

/**
 *  igb_up - Open the interface and prepare it to handle traffic
 *  @adapter: board private structure
 **/
int igb_up(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	/* hardware has been reset, we need to reload some things */
	igb_configure(adapter);

	clear_bit(__IGB_DOWN, &adapter->state);

	if (adapter->flags & IGB_FLAG_HAS_MSIX)
		igb_configure_msix(adapter);
	else
		igb_assign_vector(adapter->q_vector[0], 0);

	/* Clear any pending interrupts. */
	rd32(E1000_ICR);
	igb_irq_enable(adapter);

	rtnetif_start_queue(adapter->netdev);

	/* start the watchdog. */
	hw->mac.get_link_status = 1;
	schedule_work(&adapter->watchdog_task);

	if ((adapter->flags & IGB_FLAG_EEE) &&
	    (!hw->dev_spec._82575.eee_disable))
		adapter->eee_advert = MDIO_EEE_100TX | MDIO_EEE_1000T;

	return 0;
}

void igb_down(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	u32 tctl, rctl;

	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer
	 */
	set_bit(__IGB_DOWN, &adapter->state);

	/* disable receives in the hardware */
	rctl = rd32(E1000_RCTL);
	wr32(E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */

	rtnetif_stop_queue(netdev);

	/* disable transmits in the hardware */
	tctl = rd32(E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	wr32(E1000_TCTL, tctl);
	/* flush both disables and wait for them to finish */
	wrfl();
	usleep_range(10000, 11000);

	igb_irq_disable(adapter);

	adapter->flags &= ~IGB_FLAG_NEED_LINK_UPDATE;

	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_info_timer);

	/* record the stats before reset*/
	spin_lock(&adapter->stats64_lock);
	igb_update_stats(adapter);
	spin_unlock(&adapter->stats64_lock);

	rtnetif_carrier_off(netdev);
	adapter->link_speed = 0;
	adapter->link_duplex = 0;

	if (!pci_channel_offline(adapter->pdev))
		igb_reset(adapter);
	igb_clean_all_tx_rings(adapter);
	igb_clean_all_rx_rings(adapter);
}

void igb_reinit_locked(struct igb_adapter *adapter)
{
	WARN_ON(in_interrupt());
	while (test_and_set_bit(__IGB_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	igb_down(adapter);
	igb_up(adapter);
	clear_bit(__IGB_RESETTING, &adapter->state);
}

/** igb_enable_mas - Media Autosense re-enable after swap
 *
 * @adapter: adapter struct
 **/
static void igb_enable_mas(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 connsw = rd32(E1000_CONNSW);

	/* configure for SerDes media detect */
	if ((hw->phy.media_type == e1000_media_type_copper) &&
	    (!(connsw & E1000_CONNSW_SERDESD))) {
		connsw |= E1000_CONNSW_ENRGSRC;
		connsw |= E1000_CONNSW_AUTOSENSE_EN;
		wr32(E1000_CONNSW, connsw);
		wrfl();
	}
}

void igb_reset(struct igb_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_mac_info *mac = &hw->mac;
	struct e1000_fc_info *fc = &hw->fc;
	u32 pba = 0, tx_space, min_tx_space, min_rx_space, hwm;

	/* Repartition Pba for greater than 9k mtu
	 * To take effect CTRL.RST is required.
	 */
	switch (mac->type) {
	case e1000_i350:
	case e1000_i354:
	case e1000_82580:
		pba = rd32(E1000_RXPBS);
		pba = igb_rxpbs_adjust_82580(pba);
		break;
	case e1000_82576:
		pba = rd32(E1000_RXPBS);
		pba &= E1000_RXPBS_SIZE_MASK_82576;
		break;
	case e1000_82575:
	case e1000_i210:
	case e1000_i211:
	default:
		pba = E1000_PBA_34K;
		break;
	}

	if ((adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) &&
	    (mac->type < e1000_82576)) {
		/* adjust PBA for jumbo frames */
		wr32(E1000_PBA, pba);

		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB.
		 */
		pba = rd32(E1000_PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the Tx fifo also stores 16 bytes of information about the Tx
		 * but don't include ethernet FCS because hardware appends it
		 */
		min_tx_space = (adapter->max_frame_size +
				sizeof(union e1000_adv_tx_desc) -
				ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;

		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation
		 */
		if (tx_space < min_tx_space &&
		    ((min_tx_space - tx_space) < pba)) {
			pba = pba - (min_tx_space - tx_space);

			/* if short on Rx space, Rx wins and must trump Tx
			 * adjustment
			 */
			if (pba < min_rx_space)
				pba = min_rx_space;
		}
		wr32(E1000_PBA, pba);
	}

	/* flow control settings */
	/* The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, or
	 * - the full Rx FIFO size minus one full frame
	 */
	hwm = min(((pba << 10) * 9 / 10),
			((pba << 10) - 2 * adapter->max_frame_size));

	fc->high_water = hwm & 0xFFFFFFF0;	/* 16-byte granularity */
	fc->low_water = fc->high_water - 16;
	fc->pause_time = 0xFFFF;
	fc->send_xon = 1;
	fc->current_mode = fc->requested_mode;

	/* Allow time for pending master requests to run */
	hw->mac.ops.reset_hw(hw);
	wr32(E1000_WUC, 0);

	if (adapter->flags & IGB_FLAG_MEDIA_RESET) {
		/* need to resetup here after media swap */
		adapter->ei.get_invariants(hw);
		adapter->flags &= ~IGB_FLAG_MEDIA_RESET;
	}
	if ((mac->type == e1000_82575) &&
	    (adapter->flags & IGB_FLAG_MAS_ENABLE)) {
		igb_enable_mas(adapter);
	}
	if (hw->mac.ops.init_hw(hw))
		dev_err(&pdev->dev, "Hardware Error\n");

	/* Flow control settings reset on hardware reset, so guarantee flow
	 * control is off when forcing speed.
	 */
	if (!hw->mac.autoneg)
		igb_force_mac_fc(hw);

	igb_init_dmac(adapter, pba);
#ifdef CONFIG_IGB_HWMON
	/* Re-initialize the thermal sensor on i350 devices. */
	if (!test_bit(__IGB_DOWN, &adapter->state)) {
		if (mac->type == e1000_i350 && hw->bus.func == 0) {
			/* If present, re-initialize the external thermal sensor
			 * interface.
			 */
			if (adapter->ets)
				mac->ops.init_thermal_sensor_thresh(hw);
		}
	}
#endif
	/* Re-establish EEE setting */
	if (hw->phy.media_type == e1000_media_type_copper) {
		switch (mac->type) {
		case e1000_i350:
		case e1000_i210:
		case e1000_i211:
			igb_set_eee_i350(hw, true, true);
			break;
		case e1000_i354:
			igb_set_eee_i354(hw, true, true);
			break;
		default:
			break;
		}
	}
	if (!rtnetif_running(adapter->netdev))
		igb_power_down_link(adapter);

	igb_update_mng_vlan(adapter);

	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	wr32(E1000_VET, ETHERNET_IEEE_VLAN_TYPE);

	igb_get_phy_info(hw);
}


/**
 * igb_set_fw_version - Configure version string for ethtool
 * @adapter: adapter struct
 **/
void igb_set_fw_version(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_fw_version fw;

	igb_get_fw_version(hw, &fw);

	switch (hw->mac.type) {
	case e1000_i210:
	case e1000_i211:
		if (!(igb_get_flash_presence_i210(hw))) {
			snprintf(adapter->fw_version,
				 sizeof(adapter->fw_version),
				 "%2d.%2d-%d",
				 fw.invm_major, fw.invm_minor,
				 fw.invm_img_type);
			break;
		}
		/* fall through */
	default:
		/* if option is rom valid, display its version too */
		if (fw.or_valid) {
			snprintf(adapter->fw_version,
				 sizeof(adapter->fw_version),
				 "%d.%d, 0x%08x, %d.%d.%d",
				 fw.eep_major, fw.eep_minor, fw.etrack_id,
				 fw.or_major, fw.or_build, fw.or_patch);
		/* no option rom */
		} else if (fw.etrack_id != 0X0000) {
			snprintf(adapter->fw_version,
			    sizeof(adapter->fw_version),
			    "%d.%d, 0x%08x",
			    fw.eep_major, fw.eep_minor, fw.etrack_id);
		} else {
		snprintf(adapter->fw_version,
		    sizeof(adapter->fw_version),
		    "%d.%d.%d",
		    fw.eep_major, fw.eep_minor, fw.eep_build);
		}
		break;
	}
}

/**
 * igb_init_mas - init Media Autosense feature if enabled in the NVM
 *
 * @adapter: adapter struct
 **/
static void igb_init_mas(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u16 eeprom_data;

	hw->nvm.ops.read(hw, NVM_COMPAT, 1, &eeprom_data);
	switch (hw->bus.func) {
	case E1000_FUNC_0:
		if (eeprom_data & IGB_MAS_ENABLE_0) {
			adapter->flags |= IGB_FLAG_MAS_ENABLE;
			rtdev_info(adapter->netdev,
				"MAS: Enabling Media Autosense for port %d\n",
				hw->bus.func);
		}
		break;
	case E1000_FUNC_1:
		if (eeprom_data & IGB_MAS_ENABLE_1) {
			adapter->flags |= IGB_FLAG_MAS_ENABLE;
			rtdev_info(adapter->netdev,
				"MAS: Enabling Media Autosense for port %d\n",
				hw->bus.func);
		}
		break;
	case E1000_FUNC_2:
		if (eeprom_data & IGB_MAS_ENABLE_2) {
			adapter->flags |= IGB_FLAG_MAS_ENABLE;
			rtdev_info(adapter->netdev,
				"MAS: Enabling Media Autosense for port %d\n",
				hw->bus.func);
		}
		break;
	case E1000_FUNC_3:
		if (eeprom_data & IGB_MAS_ENABLE_3) {
			adapter->flags |= IGB_FLAG_MAS_ENABLE;
			rtdev_info(adapter->netdev,
				"MAS: Enabling Media Autosense for port %d\n",
				hw->bus.func);
		}
		break;
	default:
		/* Shouldn't get here */
		rtdev_err(adapter->netdev,
			"MAS: Invalid port configuration, returning\n");
		break;
	}
}

static dma_addr_t igb_map_rtskb(struct rtnet_device *netdev,
				struct rtskb *skb)
{
	struct igb_adapter *adapter = netdev->priv;
	struct device *dev = &adapter->pdev->dev;
	dma_addr_t addr;

	addr = dma_map_single(dev, skb->buf_start, RTSKB_SIZE,
			      DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, addr)) {
		dev_err(dev, "DMA map failed\n");
		return RTSKB_UNMAPPED;
	}
	return addr;
}

static void igb_unmap_rtskb(struct rtnet_device *netdev,
			      struct rtskb *skb)
{
	struct igb_adapter *adapter = netdev->priv;
	struct device *dev = &adapter->pdev->dev;

	dma_unmap_single(dev, skb->buf_dma_addr, RTSKB_SIZE,
			 DMA_BIDIRECTIONAL);
}

/**
 *  igb_probe - Device Initialization Routine
 *  @pdev: PCI device information struct
 *  @ent: entry in igb_pci_tbl
 *
 *  Returns 0 on success, negative on failure
 *
 *  igb_probe initializes an adapter identified by a pci_dev structure.
 *  The OS initialization, configuring of the adapter private structure,
 *  and a hardware reset occur.
 **/
static int igb_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct rtnet_device *netdev;
	struct igb_adapter *adapter;
	struct e1000_hw *hw;
	u16 eeprom_data = 0;
	s32 ret_val;
	static int global_quad_port_a; /* global quad port a indication */
	const struct e1000_info *ei = igb_info_tbl[ent->driver_data];
	int err, pci_using_dac;
	u8 part_str[E1000_PBANUM_LENGTH];

	/* Catch broken hardware that put the wrong VF device ID in
	 * the PCIe SR-IOV capability.
	 */
	if (pdev->is_virtfn) {
		WARN(1, KERN_ERR "%s (%hx:%hx) should not be a VF!\n",
			pci_name(pdev), pdev->vendor, pdev->device);
		return -EINVAL;
	}

	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

	pci_using_dac = 0;
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (!err) {
		pci_using_dac = 1;
	} else {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"No usable DMA configuration, aborting\n");
			goto err_dma;
		}
	}

	err = pci_request_selected_regions(pdev, pci_select_bars(pdev,
					   IORESOURCE_MEM),
					   igb_driver_name);
	if (err)
		goto err_pci_reg;

	pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);
	pci_save_state(pdev);

	err = -ENOMEM;
	netdev = rt_alloc_etherdev(sizeof(*adapter),
				2 * IGB_DEFAULT_RXD + IGB_DEFAULT_TXD);
	if (!netdev)
		goto err_alloc_etherdev;

	rtdev_alloc_name(netdev, "rteth%d");
	rt_rtdev_connect(netdev, &RTDEV_manager);

	netdev->vers = RTDEV_VERS_2_0;
	netdev->sysbind = &pdev->dev;

	pci_set_drvdata(pdev, netdev);
	adapter = rtnetdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	hw = &adapter->hw;
	hw->back = adapter;

	err = -EIO;
	hw->hw_addr = pci_iomap(pdev, 0, 0);
	if (!hw->hw_addr)
		goto err_ioremap;

	netdev->open = igb_open;
	netdev->stop = igb_close;
	netdev->hard_start_xmit = igb_xmit_frame;
	netdev->get_stats = igb_get_stats;
	netdev->map_rtskb = igb_map_rtskb;
	netdev->unmap_rtskb = igb_unmap_rtskb;
	netdev->do_ioctl = igb_ioctl;
#if 0
	netdev->set_multicast_list = igb_set_multi;
	netdev->set_mac_address = igb_set_mac;
	netdev->change_mtu = igb_change_mtu;

	// No ethtool support for now
	igb_set_ethtool_ops(netdev);
	netdev->watchdog_timeo = 5 * HZ;
#endif

	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	netdev->mem_start = pci_resource_start(pdev, 0);
	netdev->mem_end = pci_resource_end(pdev, 0);

	/* PCI config space info */
	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->revision_id = pdev->revision;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_device_id = pdev->subsystem_device;

	/* Copy the default MAC, PHY and NVM function pointers */
	memcpy(&hw->mac.ops, ei->mac_ops, sizeof(hw->mac.ops));
	memcpy(&hw->phy.ops, ei->phy_ops, sizeof(hw->phy.ops));
	memcpy(&hw->nvm.ops, ei->nvm_ops, sizeof(hw->nvm.ops));
	/* Initialize skew-specific constants */
	err = ei->get_invariants(hw);
	if (err)
		goto err_sw_init;

	/* setup the private structure */
	err = igb_sw_init(adapter);
	if (err)
		goto err_sw_init;

	igb_get_bus_info_pcie(hw);

	hw->phy.autoneg_wait_to_complete = false;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = false;
		hw->phy.ms_type = e1000_ms_hw_default;
	}

	if (igb_check_reset_block(hw))
		dev_info(&pdev->dev,
			"PHY reset is blocked due to SOL/IDER session.\n");

	/* features is initialized to 0 in allocation, it might have bits
	 * set by igb_sw_init so we should use an or instead of an
	 * assignment.
	 */
	netdev->features |= NETIF_F_SG |
			    NETIF_F_IP_CSUM |
			    NETIF_F_IPV6_CSUM |
			    NETIF_F_TSO |
			    NETIF_F_TSO6 |
			    NETIF_F_RXHASH |
			    NETIF_F_RXCSUM |
			    NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_HW_VLAN_CTAG_TX;

#if 0
	/* set this bit last since it cannot be part of hw_features */
	netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
#endif

	netdev->priv_flags |= IFF_SUPP_NOFCS;

	if (pci_using_dac)
		netdev->features |= NETIF_F_HIGHDMA;

	netdev->priv_flags |= IFF_UNICAST_FLT;

	adapter->en_mng_pt = igb_enable_mng_pass_thru(hw);

	/* before reading the NVM, reset the controller to put the device in a
	 * known good starting state
	 */
	hw->mac.ops.reset_hw(hw);

	/* make sure the NVM is good , i211/i210 parts can have special NVM
	 * that doesn't contain a checksum
	 */
	switch (hw->mac.type) {
	case e1000_i210:
	case e1000_i211:
		if (igb_get_flash_presence_i210(hw)) {
			if (hw->nvm.ops.validate(hw) < 0) {
				dev_err(&pdev->dev,
					"The NVM Checksum Is Not Valid\n");
				err = -EIO;
				goto err_eeprom;
			}
		}
		break;
	default:
		if (hw->nvm.ops.validate(hw) < 0) {
			dev_err(&pdev->dev, "The NVM Checksum Is Not Valid\n");
			err = -EIO;
			goto err_eeprom;
		}
		break;
	}

	/* copy the MAC address out of the NVM */
	if (hw->mac.ops.read_mac_addr(hw))
		dev_err(&pdev->dev, "NVM Read Error\n");

	memcpy(netdev->dev_addr, hw->mac.addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		dev_err(&pdev->dev, "Invalid MAC Address\n");
		err = -EIO;
		goto err_eeprom;
	}

	/* get firmware version for ethtool -i */
	igb_set_fw_version(adapter);

	/* configure RXPBSIZE and TXPBSIZE */
	if (hw->mac.type == e1000_i210) {
		wr32(E1000_RXPBS, I210_RXPBSIZE_DEFAULT);
		wr32(E1000_TXPBS, I210_TXPBSIZE_DEFAULT);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
	timer_setup(&adapter->watchdog_timer, igb_watchdog, 0);
	timer_setup(&adapter->phy_info_timer, igb_update_phy_info, 0);
#else /* < 4.14 */
	setup_timer(&adapter->watchdog_timer, igb_watchdog,
		    (unsigned long) adapter);
	setup_timer(&adapter->phy_info_timer, igb_update_phy_info,
		    (unsigned long) adapter);
#endif /* < 4.14 */

	INIT_WORK(&adapter->reset_task, igb_reset_task);
	INIT_WORK(&adapter->watchdog_task, igb_watchdog_task);
	rtdm_nrtsig_init(&adapter->watchdog_nrtsig,
			igb_nrtsig_watchdog, adapter);

	/* Initialize link properties that are user-changeable */
	adapter->fc_autoneg = true;
	hw->mac.autoneg = true;
	hw->phy.autoneg_advertised = 0x2f;

	hw->fc.requested_mode = e1000_fc_default;
	hw->fc.current_mode = e1000_fc_default;

	igb_validate_mdi_setting(hw);

	/* By default, support wake on port A */
	if (hw->bus.func == 0)
		adapter->flags |= IGB_FLAG_WOL_SUPPORTED;

	/* Check the NVM for wake support on non-port A ports */
	if (hw->mac.type >= e1000_82580)
		hw->nvm.ops.read(hw, NVM_INIT_CONTROL3_PORT_A +
				 NVM_82580_LAN_FUNC_OFFSET(hw->bus.func), 1,
				 &eeprom_data);
	else if (hw->bus.func == 1)
		hw->nvm.ops.read(hw, NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);

	if (eeprom_data & IGB_EEPROM_APME)
		adapter->flags |= IGB_FLAG_WOL_SUPPORTED;

	/* now that we have the eeprom settings, apply the special cases where
	 * the eeprom may be wrong or the board simply won't support wake on
	 * lan on a particular port
	 */
	switch (pdev->device) {
	case E1000_DEV_ID_82575GB_QUAD_COPPER:
		adapter->flags &= ~IGB_FLAG_WOL_SUPPORTED;
		break;
	case E1000_DEV_ID_82575EB_FIBER_SERDES:
	case E1000_DEV_ID_82576_FIBER:
	case E1000_DEV_ID_82576_SERDES:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting
		 */
		if (rd32(E1000_STATUS) & E1000_STATUS_FUNC_1)
			adapter->flags &= ~IGB_FLAG_WOL_SUPPORTED;
		break;
	case E1000_DEV_ID_82576_QUAD_COPPER:
	case E1000_DEV_ID_82576_QUAD_COPPER_ET2:
		/* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->flags &= ~IGB_FLAG_WOL_SUPPORTED;
		else
			adapter->flags |= IGB_FLAG_QUAD_PORT_A;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
		break;
	default:
		/* If the device can't wake, don't set software support */
		if (!device_can_wakeup(&adapter->pdev->dev))
			adapter->flags &= ~IGB_FLAG_WOL_SUPPORTED;
	}

	/* initialize the wol settings based on the eeprom settings */
	if (adapter->flags & IGB_FLAG_WOL_SUPPORTED)
		adapter->wol |= E1000_WUFC_MAG;

	/* Some vendors want WoL disabled by default, but still supported */
	if ((hw->mac.type == e1000_i350) &&
	    (pdev->subsystem_vendor == PCI_VENDOR_ID_HP)) {
		adapter->flags |= IGB_FLAG_WOL_SUPPORTED;
		adapter->wol = 0;
	}

	device_set_wakeup_enable(&adapter->pdev->dev,
				 adapter->flags & IGB_FLAG_WOL_SUPPORTED);

	/* reset the hardware with the new settings */
	igb_reset(adapter);

	/* let the f/w know that the h/w is now under the control of the
	 * driver.
	 */
	igb_get_hw_control(adapter);

	strcpy(netdev->name, "rteth%d");
	err = rt_register_rtnetdev(netdev);
	if (err)
		goto err_release_hw_control;

	/* carrier off reporting is important to ethtool even BEFORE open */
	rtnetif_carrier_off(netdev);

#ifdef CONFIG_IGB_HWMON
	/* Initialize the thermal sensor on i350 devices. */
	if (hw->mac.type == e1000_i350 && hw->bus.func == 0) {
		u16 ets_word;

		/* Read the NVM to determine if this i350 device supports an
		 * external thermal sensor.
		 */
		hw->nvm.ops.read(hw, NVM_ETS_CFG, 1, &ets_word);
		if (ets_word != 0x0000 && ets_word != 0xFFFF)
			adapter->ets = true;
		else
			adapter->ets = false;
		if (igb_sysfs_init(adapter))
			dev_err(&pdev->dev,
				"failed to allocate sysfs resources\n");
	} else {
		adapter->ets = false;
	}
#endif
	/* Check if Media Autosense is enabled */
	adapter->ei = *ei;
	if (hw->dev_spec._82575.mas_capable)
		igb_init_mas(adapter);

	dev_info(&pdev->dev, "Intel(R) Gigabit Ethernet Network Connection\n");
	/* print bus type/speed/width info, not applicable to i354 */
	if (hw->mac.type != e1000_i354) {
		dev_info(&pdev->dev, "%s: (PCIe:%s:%s) %pM\n",
			 netdev->name,
			 ((hw->bus.speed == e1000_bus_speed_2500) ? "2.5Gb/s" :
			  (hw->bus.speed == e1000_bus_speed_5000) ? "5.0Gb/s" :
			   "unknown"),
			 ((hw->bus.width == e1000_bus_width_pcie_x4) ?
			  "Width x4" :
			  (hw->bus.width == e1000_bus_width_pcie_x2) ?
			  "Width x2" :
			  (hw->bus.width == e1000_bus_width_pcie_x1) ?
			  "Width x1" : "unknown"), netdev->dev_addr);
	}

	if ((hw->mac.type >= e1000_i210 ||
	     igb_get_flash_presence_i210(hw))) {
		ret_val = igb_read_part_string(hw, part_str,
					       E1000_PBANUM_LENGTH);
	} else {
		ret_val = -E1000_ERR_INVM_VALUE_NOT_FOUND;
	}

	if (ret_val)
		strcpy(part_str, "Unknown");
	dev_info(&pdev->dev, "%s: PBA No: %s\n", netdev->name, part_str);
	dev_info(&pdev->dev,
		"Using %s interrupts. %d rx queue(s), %d tx queue(s)\n",
		(adapter->flags & IGB_FLAG_HAS_MSIX) ? "MSI-X" :
		(adapter->flags & IGB_FLAG_HAS_MSI) ? "MSI" : "legacy",
		adapter->num_rx_queues, adapter->num_tx_queues);
	if (hw->phy.media_type == e1000_media_type_copper) {
		switch (hw->mac.type) {
		case e1000_i350:
		case e1000_i210:
		case e1000_i211:
			/* Enable EEE for internal copper PHY devices */
			err = igb_set_eee_i350(hw, true, true);
			if ((!err) &&
			    (!hw->dev_spec._82575.eee_disable)) {
				adapter->eee_advert =
					MDIO_EEE_100TX | MDIO_EEE_1000T;
				adapter->flags |= IGB_FLAG_EEE;
			}
			break;
		case e1000_i354:
			if ((rd32(E1000_CTRL_EXT) &
			    E1000_CTRL_EXT_LINK_MODE_SGMII)) {
				err = igb_set_eee_i354(hw, true, true);
				if ((!err) &&
					(!hw->dev_spec._82575.eee_disable)) {
					adapter->eee_advert =
					   MDIO_EEE_100TX | MDIO_EEE_1000T;
					adapter->flags |= IGB_FLAG_EEE;
				}
			}
			break;
		default:
			break;
		}
	}
	pm_runtime_put_noidle(&pdev->dev);
	return 0;

err_release_hw_control:
	igb_release_hw_control(adapter);
	memset(&adapter->i2c_adap, 0, sizeof(adapter->i2c_adap));
err_eeprom:
	if (!igb_check_reset_block(hw))
		igb_reset_phy(hw);

	if (hw->flash_address)
		iounmap(hw->flash_address);
err_sw_init:
	igb_clear_interrupt_scheme(adapter);
	pci_iounmap(pdev, hw->hw_addr);
err_ioremap:
	rtdev_free(netdev);
err_alloc_etherdev:
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

/**
 *  igb_remove_i2c - Cleanup  I2C interface
 *  @adapter: pointer to adapter structure
 **/
static void igb_remove_i2c(struct igb_adapter *adapter)
{
	/* free the adapter bus structure */
	i2c_del_adapter(&adapter->i2c_adap);
}

/**
 *  igb_remove - Device Removal Routine
 *  @pdev: PCI device information struct
 *
 *  igb_remove is called by the PCI subsystem to alert the driver
 *  that it should release a PCI device.  The could be caused by a
 *  Hot-Plug event, or because the driver is going to be removed from
 *  memory.
 **/
static void igb_remove(struct pci_dev *pdev)
{
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	rtdev_down(netdev);
	igb_down(adapter);

	pm_runtime_get_noresume(&pdev->dev);
#ifdef CONFIG_IGB_HWMON
	igb_sysfs_exit(adapter);
#endif
	igb_remove_i2c(adapter);
	/* The watchdog timer may be rescheduled, so explicitly
	 * disable watchdog from being rescheduled.
	 */
	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_info_timer);

	cancel_work_sync(&adapter->reset_task);
	cancel_work_sync(&adapter->watchdog_task);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	igb_release_hw_control(adapter);

	rt_rtdev_disconnect(netdev);
	rt_unregister_rtnetdev(netdev);

	igb_clear_interrupt_scheme(adapter);

	pci_iounmap(pdev, hw->hw_addr);
	if (hw->flash_address)
		iounmap(hw->flash_address);
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));

	kfree(adapter->shadow_vfta);
	rtdev_free(netdev);

	pci_disable_pcie_error_reporting(pdev);

	pci_disable_device(pdev);
}

/**
 *  igb_probe_vfs - Initialize vf data storage and add VFs to pci config space
 *  @adapter: board private structure to initialize
 *
 *  This function initializes the vf specific data storage and then attempts to
 *  allocate the VFs.  The reason for ordering it this way is because it is much
 *  mor expensive time wise to disable SR-IOV than it is to allocate and free
 *  the memory for the VFs.
 **/
static void igb_probe_vfs(struct igb_adapter *adapter)
{
}

static void igb_init_queue_configuration(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 max_rss_queues;

	max_rss_queues = 1;
	adapter->rss_queues = max_rss_queues;

	/* Determine if we need to pair queues. */
	switch (hw->mac.type) {
	case e1000_82575:
	case e1000_i211:
		/* Device supports enough interrupts without queue pairing. */
		break;
	case e1000_82576:
		/* If VFs are going to be allocated with RSS queues then we
		 * should pair the queues in order to conserve interrupts due
		 * to limited supply.
		 */
		/* fall through */
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_i210:
	default:
		/* If rss_queues > half of max_rss_queues, pair the queues in
		 * order to conserve interrupts due to limited supply.
		 */
		if (adapter->rss_queues > (max_rss_queues / 2))
			adapter->flags |= IGB_FLAG_QUEUE_PAIRS;
		break;
	}
}

/**
 *  igb_sw_init - Initialize general software structures (struct igb_adapter)
 *  @adapter: board private structure to initialize
 *
 *  igb_sw_init initializes the Adapter private data structure.
 *  Fields are initialized based on PCI device information and
 *  OS network device settings (MTU size).
 **/
static int igb_sw_init(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;

	pci_read_config_word(pdev, PCI_COMMAND, &hw->bus.pci_cmd_word);

	/* set default ring sizes */
	adapter->tx_ring_count = IGB_DEFAULT_TXD;
	adapter->rx_ring_count = IGB_DEFAULT_RXD;

	/* set default ITR values */
	if (InterruptThrottle) {
		adapter->rx_itr_setting = IGB_DEFAULT_ITR;
		adapter->tx_itr_setting = IGB_DEFAULT_ITR;
	} else {
		adapter->rx_itr_setting = IGB_MIN_ITR_USECS;
		adapter->tx_itr_setting = IGB_MIN_ITR_USECS;
	}

	/* set default work limits */
	adapter->tx_work_limit = IGB_DEFAULT_TX_WORK;

	adapter->max_frame_size = netdev->mtu + ETH_HLEN + ETH_FCS_LEN +
				  VLAN_HLEN;
	adapter->min_frame_size = ETH_ZLEN + ETH_FCS_LEN;

	spin_lock_init(&adapter->stats64_lock);

	igb_init_queue_configuration(adapter);

	/* Setup and initialize a copy of the hw vlan table array */
	adapter->shadow_vfta = kcalloc(E1000_VLAN_FILTER_TBL_SIZE, sizeof(u32),
				       GFP_ATOMIC);

	/* This call may decrease the number of queues */
	if (igb_init_interrupt_scheme(adapter, true)) {
		dev_err(&pdev->dev, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	igb_probe_vfs(adapter);

	/* Explicitly disable IRQ since the NIC can be in any state. */
	igb_irq_disable(adapter);

	if (hw->mac.type >= e1000_i350)
		adapter->flags &= ~IGB_FLAG_DMAC;

	set_bit(__IGB_DOWN, &adapter->state);
	return 0;
}

/**
 *  igb_open - Called when a network interface is made active
 *  @netdev: network interface device structure
 *
 *  Returns 0 on success, negative value on failure
 *
 *  The open entry point is called when a network interface is made
 *  active by the system (IFF_UP).  At this point all resources needed
 *  for transmit and receive operations are allocated, the interrupt
 *  handler is registered with the OS, the watchdog timer is started,
 *  and the stack is notified that the interface is ready.
 **/
static int __igb_open(struct rtnet_device *netdev, bool resuming)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	int err;

	/* disallow open during test */
	if (test_bit(__IGB_TESTING, &adapter->state)) {
		WARN_ON(resuming);
		return -EBUSY;
	}

	if (!resuming)
		pm_runtime_get_sync(&pdev->dev);

	rtnetif_carrier_off(netdev);

	/* allocate transmit descriptors */
	err = igb_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = igb_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;

	igb_power_up_link(adapter);

	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.
	 */
	igb_configure(adapter);

	err = igb_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* From here on the code is the same as igb_up() */
	clear_bit(__IGB_DOWN, &adapter->state);

	/* Clear any pending interrupts. */
	rd32(E1000_ICR);

	igb_irq_enable(adapter);

	rtnetif_start_queue(netdev);

	if (!resuming)
		pm_runtime_put(&pdev->dev);

	/* start the watchdog. */
	hw->mac.get_link_status = 1;
	schedule_work(&adapter->watchdog_task);

	return 0;

err_req_irq:
	igb_release_hw_control(adapter);
	igb_power_down_link(adapter);
	igb_free_all_rx_resources(adapter);
err_setup_rx:
	igb_free_all_tx_resources(adapter);
err_setup_tx:
	igb_reset(adapter);
	if (!resuming)
		pm_runtime_put(&pdev->dev);

	return err;
}

static int igb_open(struct rtnet_device *netdev)
{
	return __igb_open(netdev, false);
}

/**
 *  igb_close - Disables a network interface
 *  @netdev: network interface device structure
 *
 *  Returns 0, this is not allowed to fail
 *
 *  The close entry point is called when an interface is de-activated
 *  by the OS.  The hardware is still under the driver's control, but
 *  needs to be disabled.  A global MAC reset is issued to stop the
 *  hardware, and all transmit and receive resources are freed.
 **/
static int __igb_close(struct rtnet_device *netdev, bool suspending)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct pci_dev *pdev = adapter->pdev;

	WARN_ON(test_bit(__IGB_RESETTING, &adapter->state));

	if (!suspending)
		pm_runtime_get_sync(&pdev->dev);

	igb_down(adapter);
	igb_free_irq(adapter);

	rt_stack_disconnect(netdev);

	igb_free_all_tx_resources(adapter);
	igb_free_all_rx_resources(adapter);

	if (!suspending)
		pm_runtime_put_sync(&pdev->dev);
	return 0;
}

static int igb_close(struct rtnet_device *netdev)
{
	return __igb_close(netdev, false);
}

/**
 *  igb_setup_tx_resources - allocate Tx resources (Descriptors)
 *  @tx_ring: tx descriptor ring (for a specific queue) to setup
 *
 *  Return 0 on success, negative on failure
 **/
int igb_setup_tx_resources(struct igb_ring *tx_ring)
{
	struct device *dev = tx_ring->dev;
	int size;

	size = sizeof(struct igb_tx_buffer) * tx_ring->count;

	tx_ring->tx_buffer_info = vzalloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;

	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(union e1000_adv_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	tx_ring->desc = dma_alloc_coherent(dev, tx_ring->size,
					   &tx_ring->dma, GFP_KERNEL);
	if (!tx_ring->desc)
		goto err;

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Tx descriptor ring\n");
	return -ENOMEM;
}

/**
 *  igb_setup_all_tx_resources - wrapper to allocate Tx resources
 *				 (Descriptors) for all queues
 *  @adapter: board private structure
 *
 *  Return 0 on success, negative on failure
 **/
static int igb_setup_all_tx_resources(struct igb_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	int i, err = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		err = igb_setup_tx_resources(adapter->tx_ring[i]);
		if (err) {
			dev_err(&pdev->dev,
				"Allocation for Tx Queue %u failed\n", i);
			for (i--; i >= 0; i--)
				igb_free_tx_resources(adapter->tx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 *  igb_setup_tctl - configure the transmit control registers
 *  @adapter: Board private structure
 **/
void igb_setup_tctl(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 tctl;

	/* disable queue 0 which is enabled by default on 82575 and 82576 */
	wr32(E1000_TXDCTL(0), 0);

	/* Program the Transmit Control Register */
	tctl = rd32(E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
		(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);

	igb_config_collision_dist(hw);

	/* Enable transmits */
	tctl |= E1000_TCTL_EN;

	wr32(E1000_TCTL, tctl);
}

/**
 *  igb_configure_tx_ring - Configure transmit ring after Reset
 *  @adapter: board private structure
 *  @ring: tx ring to configure
 *
 *  Configure a transmit ring after a reset.
 **/
void igb_configure_tx_ring(struct igb_adapter *adapter,
			   struct igb_ring *ring)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 txdctl = 0;
	u64 tdba = ring->dma;
	int reg_idx = ring->reg_idx;

	/* disable the queue */
	wr32(E1000_TXDCTL(reg_idx), 0);
	wrfl();
	mdelay(10);

	wr32(E1000_TDLEN(reg_idx),
	     ring->count * sizeof(union e1000_adv_tx_desc));
	wr32(E1000_TDBAL(reg_idx),
	     tdba & 0x00000000ffffffffULL);
	wr32(E1000_TDBAH(reg_idx), tdba >> 32);

	ring->tail = hw->hw_addr + E1000_TDT(reg_idx);
	wr32(E1000_TDH(reg_idx), 0);
	writel(0, ring->tail);

	txdctl |= IGB_TX_PTHRESH;
	txdctl |= IGB_TX_HTHRESH << 8;
	txdctl |= IGB_TX_WTHRESH << 16;

	txdctl |= E1000_TXDCTL_QUEUE_ENABLE;
	wr32(E1000_TXDCTL(reg_idx), txdctl);
}

/**
 *  igb_configure_tx - Configure transmit Unit after Reset
 *  @adapter: board private structure
 *
 *  Configure the Tx unit of the MAC after a reset.
 **/
static void igb_configure_tx(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		igb_configure_tx_ring(adapter, adapter->tx_ring[i]);
}

/**
 *  igb_setup_rx_resources - allocate Rx resources (Descriptors)
 *  @rx_ring: Rx descriptor ring (for a specific queue) to setup
 *
 *  Returns 0 on success, negative on failure
 **/
int igb_setup_rx_resources(struct igb_ring *rx_ring)
{
	struct device *dev = rx_ring->dev;
	int size;

	size = sizeof(struct igb_rx_buffer) * rx_ring->count;

	rx_ring->rx_buffer_info = vzalloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(union e1000_adv_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);

	rx_ring->desc = dma_alloc_coherent(dev, rx_ring->size,
					   &rx_ring->dma, GFP_KERNEL);
	if (!rx_ring->desc)
		goto err;

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;

err:
	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");
	return -ENOMEM;
}

/**
 *  igb_setup_all_rx_resources - wrapper to allocate Rx resources
 *				 (Descriptors) for all queues
 *  @adapter: board private structure
 *
 *  Return 0 on success, negative on failure
 **/
static int igb_setup_all_rx_resources(struct igb_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	int i, err = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		err = igb_setup_rx_resources(adapter->rx_ring[i]);
		if (err) {
			dev_err(&pdev->dev,
				"Allocation for Rx Queue %u failed\n", i);
			for (i--; i >= 0; i--)
				igb_free_rx_resources(adapter->rx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 *  igb_setup_mrqc - configure the multiple receive queue control registers
 *  @adapter: Board private structure
 **/
static void igb_setup_mrqc(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mrqc, rxcsum;
	u32 j, num_rx_queues;
	u32 rss_key[10];

	get_random_bytes(rss_key, sizeof(rss_key));
	for (j = 0; j < 10; j++)
		wr32(E1000_RSSRK(j), rss_key[j]);

	num_rx_queues = adapter->rss_queues;

	switch (hw->mac.type) {
	case e1000_82576:
		/* 82576 supports 2 RSS queues for SR-IOV */
		break;
	default:
		break;
	}

	if (adapter->rss_indir_tbl_init != num_rx_queues) {
		for (j = 0; j < IGB_RETA_SIZE; j++)
			adapter->rss_indir_tbl[j] =
			(j * num_rx_queues) / IGB_RETA_SIZE;
		adapter->rss_indir_tbl_init = num_rx_queues;
	}

	/* Disable raw packet checksumming so that RSS hash is placed in
	 * descriptor on writeback.  No need to enable TCP/UDP/IP checksum
	 * offloads as they are enabled by default
	 */
	rxcsum = rd32(E1000_RXCSUM);
	rxcsum |= E1000_RXCSUM_PCSD;

	if (adapter->hw.mac.type >= e1000_82576)
		/* Enable Receive Checksum Offload for SCTP */
		rxcsum |= E1000_RXCSUM_CRCOFL;

	/* Don't need to set TUOFL or IPOFL, they default to 1 */
	wr32(E1000_RXCSUM, rxcsum);

	/* Generate RSS hash based on packet types, TCP/UDP
	 * port numbers and/or IPv4/v6 src and dst addresses
	 */
	mrqc = E1000_MRQC_RSS_FIELD_IPV4 |
	       E1000_MRQC_RSS_FIELD_IPV4_TCP |
	       E1000_MRQC_RSS_FIELD_IPV6 |
	       E1000_MRQC_RSS_FIELD_IPV6_TCP |
	       E1000_MRQC_RSS_FIELD_IPV6_TCP_EX;

	if (adapter->flags & IGB_FLAG_RSS_FIELD_IPV4_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4_UDP;
	if (adapter->flags & IGB_FLAG_RSS_FIELD_IPV6_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_UDP;

	/* If VMDq is enabled then we set the appropriate mode for that, else
	 * we default to RSS so that an RSS hash is calculated per packet even
	 * if we are only using one queue
	 */
	if (hw->mac.type != e1000_i211)
		mrqc |= E1000_MRQC_ENABLE_RSS_4Q;

	wr32(E1000_MRQC, mrqc);
}

/**
 *  igb_setup_rctl - configure the receive control registers
 *  @adapter: Board private structure
 **/
void igb_setup_rctl(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;

	rctl = rd32(E1000_RCTL);

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl &= ~(E1000_RCTL_LBM_TCVR | E1000_RCTL_LBM_MAC);

	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* enable stripping of CRC. It's unlikely this will break BMC
	 * redirection as it did with e1000. Newer features require
	 * that the HW strips the CRC.
	 */
	rctl |= E1000_RCTL_SECRC;

	/* disable store bad packets and clear size bits. */
	rctl &= ~(E1000_RCTL_SBP | E1000_RCTL_SZ_256);

	/* enable LPE to prevent packets larger than max_frame_size */
	rctl |= E1000_RCTL_LPE;

	/* disable queue 0 to prevent tail write w/o re-config */
	wr32(E1000_RXDCTL(0), 0);

	/* This is useful for sniffing bad packets. */
	if (adapter->netdev->features & NETIF_F_RXALL) {
		/* UPE and MPE will be handled by normal PROMISC logic
		 * in e1000e_set_rx_mode
		 */
		rctl |= (E1000_RCTL_SBP | /* Receive bad packets */
			 E1000_RCTL_BAM | /* RX All Bcast Pkts */
			 E1000_RCTL_PMCF); /* RX All MAC Ctrl Pkts */

		rctl &= ~(E1000_RCTL_VFE | /* Disable VLAN filter */
			  E1000_RCTL_DPF | /* Allow filtered pause */
			  E1000_RCTL_CFIEN); /* Dis VLAN CFIEN Filter */
		/* Do not mess with E1000_CTRL_VME, it affects transmit as well,
		 * and that breaks VLANs.
		 */
	}

	wr32(E1000_RCTL, rctl);
}

/**
 *  igb_rlpml_set - set maximum receive packet size
 *  @adapter: board private structure
 *
 *  Configure maximum receivable packet size.
 **/
static void igb_rlpml_set(struct igb_adapter *adapter)
{
	u32 max_frame_size = adapter->max_frame_size;
	struct e1000_hw *hw = &adapter->hw;

	wr32(E1000_RLPML, max_frame_size);
}

static inline void igb_set_vmolr(struct igb_adapter *adapter,
				 int vfn, bool aupe)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 vmolr;

	/* This register exists only on 82576 and newer so if we are older then
	 * we should exit and do nothing
	 */
	if (hw->mac.type < e1000_82576)
		return;

	vmolr = rd32(E1000_VMOLR(vfn));
	vmolr |= E1000_VMOLR_STRVLAN; /* Strip vlan tags */
	if (hw->mac.type == e1000_i350) {
		u32 dvmolr;

		dvmolr = rd32(E1000_DVMOLR(vfn));
		dvmolr |= E1000_DVMOLR_STRVLAN;
		wr32(E1000_DVMOLR(vfn), dvmolr);
	}
	if (aupe)
		vmolr |= E1000_VMOLR_AUPE; /* Accept untagged packets */
	else
		vmolr &= ~(E1000_VMOLR_AUPE); /* Tagged packets ONLY */

	/* clear all bits that might not be set */
	vmolr &= ~(E1000_VMOLR_BAM | E1000_VMOLR_RSSE);

	if (adapter->rss_queues > 1)
		vmolr |= E1000_VMOLR_RSSE; /* enable RSS */
	/* for VMDq only allow the VFs and pool 0 to accept broadcast and
	 * multicast packets
	 */
	vmolr |= E1000_VMOLR_BAM; /* Accept broadcast */

	wr32(E1000_VMOLR(vfn), vmolr);
}

/**
 *  igb_configure_rx_ring - Configure a receive ring after Reset
 *  @adapter: board private structure
 *  @ring: receive ring to be configured
 *
 *  Configure the Rx unit of the MAC after a reset.
 **/
void igb_configure_rx_ring(struct igb_adapter *adapter,
			   struct igb_ring *ring)
{
	struct e1000_hw *hw = &adapter->hw;
	u64 rdba = ring->dma;
	int reg_idx = ring->reg_idx;
	u32 srrctl = 0, rxdctl = 0;

	ring->rx_buffer_len = max_t(u32, adapter->max_frame_size,
				MAXIMUM_ETHERNET_VLAN_SIZE);

	/* disable the queue */
	wr32(E1000_RXDCTL(reg_idx), 0);

	/* Set DMA base address registers */
	wr32(E1000_RDBAL(reg_idx),
	     rdba & 0x00000000ffffffffULL);
	wr32(E1000_RDBAH(reg_idx), rdba >> 32);
	wr32(E1000_RDLEN(reg_idx),
	     ring->count * sizeof(union e1000_adv_rx_desc));

	/* initialize head and tail */
	ring->tail = hw->hw_addr + E1000_RDT(reg_idx);
	wr32(E1000_RDH(reg_idx), 0);
	writel(0, ring->tail);

	/* set descriptor configuration */
	srrctl = IGB_RX_HDR_LEN << E1000_SRRCTL_BSIZEHDRSIZE_SHIFT;
	srrctl |= IGB_RX_BUFSZ >> E1000_SRRCTL_BSIZEPKT_SHIFT;
	srrctl |= E1000_SRRCTL_DESCTYPE_ADV_ONEBUF;
	if (hw->mac.type >= e1000_82580)
		srrctl |= E1000_SRRCTL_TIMESTAMP;
	/* Only set Drop Enable if we are supporting multiple queues */
	if (adapter->num_rx_queues > 1)
		srrctl |= E1000_SRRCTL_DROP_EN;

	wr32(E1000_SRRCTL(reg_idx), srrctl);

	/* set filtering for VMDQ pools */
	igb_set_vmolr(adapter, reg_idx & 0x7, true);

	rxdctl |= IGB_RX_PTHRESH;
	rxdctl |= IGB_RX_HTHRESH << 8;
	rxdctl |= IGB_RX_WTHRESH << 16;

	/* enable receive descriptor fetching */
	rxdctl |= E1000_RXDCTL_QUEUE_ENABLE;
	wr32(E1000_RXDCTL(reg_idx), rxdctl);
}

/**
 *  igb_configure_rx - Configure receive Unit after Reset
 *  @adapter: board private structure
 *
 *  Configure the Rx unit of the MAC after a reset.
 **/
static void igb_configure_rx(struct igb_adapter *adapter)
{
	int i;

	/* set the correct pool for the PF default MAC address in entry 0 */
	igb_rar_set_qsel(adapter, adapter->hw.mac.addr, 0, 0);

	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < adapter->num_rx_queues; i++)
		igb_configure_rx_ring(adapter, adapter->rx_ring[i]);
}

/**
 *  igb_free_tx_resources - Free Tx Resources per Queue
 *  @tx_ring: Tx descriptor ring for a specific queue
 *
 *  Free all transmit software resources
 **/
void igb_free_tx_resources(struct igb_ring *tx_ring)
{
	igb_clean_tx_ring(tx_ring);

	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

	dma_free_coherent(tx_ring->dev, tx_ring->size,
			  tx_ring->desc, tx_ring->dma);

	tx_ring->desc = NULL;
}

/**
 *  igb_free_all_tx_resources - Free Tx Resources for All Queues
 *  @adapter: board private structure
 *
 *  Free all transmit software resources
 **/
static void igb_free_all_tx_resources(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		if (adapter->tx_ring[i])
			igb_free_tx_resources(adapter->tx_ring[i]);
}

void igb_unmap_and_free_tx_resource(struct igb_ring *ring,
				    struct igb_tx_buffer *tx_buffer)
{
	if (tx_buffer->skb) {
		kfree_rtskb(tx_buffer->skb);
		tx_buffer->skb = NULL;
	}
	tx_buffer->next_to_watch = NULL;
	/* buffer_info must be completely set up in the transmit path */
}

/**
 *  igb_clean_tx_ring - Free Tx Buffers
 *  @tx_ring: ring to be cleaned
 **/
static void igb_clean_tx_ring(struct igb_ring *tx_ring)
{
	struct igb_tx_buffer *buffer_info;
	unsigned long size;
	u16 i;

	if (!tx_ring->tx_buffer_info)
		return;
	/* Free all the Tx ring sk_buffs */

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->tx_buffer_info[i];
		igb_unmap_and_free_tx_resource(tx_ring, buffer_info);
	}

	size = sizeof(struct igb_tx_buffer) * tx_ring->count;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

/**
 *  igb_clean_all_tx_rings - Free Tx Buffers for all queues
 *  @adapter: board private structure
 **/
static void igb_clean_all_tx_rings(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		if (adapter->tx_ring[i])
			igb_clean_tx_ring(adapter->tx_ring[i]);
}

/**
 *  igb_free_rx_resources - Free Rx Resources
 *  @rx_ring: ring to clean the resources from
 *
 *  Free all receive software resources
 **/
void igb_free_rx_resources(struct igb_ring *rx_ring)
{
	igb_clean_rx_ring(rx_ring);

	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->size,
			  rx_ring->desc, rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 *  igb_free_all_rx_resources - Free Rx Resources for All Queues
 *  @adapter: board private structure
 *
 *  Free all receive software resources
 **/
static void igb_free_all_rx_resources(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		if (adapter->rx_ring[i])
			igb_free_rx_resources(adapter->rx_ring[i]);
}

/**
 *  igb_clean_rx_ring - Free Rx Buffers per Queue
 *  @rx_ring: ring to free buffers from
 **/
static void igb_clean_rx_ring(struct igb_ring *rx_ring)
{
	unsigned long size;
	u16 i;

	if (!rx_ring->rx_buffer_info)
		return;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		struct igb_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];

		if (buffer_info->dma)
			buffer_info->dma = 0;

		if (buffer_info->skb) {
			kfree_rtskb(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct igb_rx_buffer) * rx_ring->count;
	memset(rx_ring->rx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

/**
 *  igb_clean_all_rx_rings - Free Rx Buffers for all queues
 *  @adapter: board private structure
 **/
static void igb_clean_all_rx_rings(struct igb_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		if (adapter->rx_ring[i])
			igb_clean_rx_ring(adapter->rx_ring[i]);
}

/**
 *  igb_write_mc_addr_list - write multicast addresses to MTA
 *  @netdev: network interface device structure
 *
 *  Writes multicast address list to the MTA hash table.
 *  Returns: -ENOMEM on failure
 *           0 on no addresses written
 *           X on writing X addresses to MTA
 **/
static int igb_write_mc_addr_list(struct rtnet_device *netdev)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
#if 0
	struct netdev_hw_addr *ha;
	u8  *mta_list;
	int i;
	if (netdev_mc_empty(netdev)) {
		/* nothing to program, so clear mc list */
		igb_update_mc_addr_list(hw, NULL, 0);
		igb_restore_vf_multicasts(adapter);
		return 0;
	}

	mta_list = kzalloc(netdev_mc_count(netdev) * 6, GFP_ATOMIC);
	if (!mta_list)
		return -ENOMEM;

	/* The shared function expects a packed array of only addresses. */
	i = 0;
	netdev_for_each_mc_addr(ha, netdev)
		memcpy(mta_list + (i++ * ETH_ALEN), ha->addr, ETH_ALEN);

	igb_update_mc_addr_list(hw, mta_list, i);
	kfree(mta_list);

	return netdev_mc_count(netdev);
#else
	igb_update_mc_addr_list(hw, NULL, 0);
	return 0;
#endif
}

/**
 *  igb_write_uc_addr_list - write unicast addresses to RAR table
 *  @netdev: network interface device structure
 *
 *  Writes unicast address list to the RAR table.
 *  Returns: -ENOMEM on failure/insufficient address space
 *           0 on no addresses written
 *           X on writing X addresses to the RAR table
 **/
static int igb_write_uc_addr_list(struct rtnet_device *netdev)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	unsigned int vfn = 0;
	unsigned int rar_entries = hw->mac.rar_entry_count - (vfn + 1);
	int count = 0;

	/* write the addresses in reverse order to avoid write combining */
	for (; rar_entries > 0 ; rar_entries--) {
		wr32(E1000_RAH(rar_entries), 0);
		wr32(E1000_RAL(rar_entries), 0);
	}
	wrfl();

	return count;
}

/**
 *  igb_set_rx_mode - Secondary Unicast, Multicast and Promiscuous mode set
 *  @netdev: network interface device structure
 *
 *  The set_rx_mode entry point is called whenever the unicast or multicast
 *  address lists or the network interface flags are updated.  This routine is
 *  responsible for configuring the hardware for proper unicast, multicast,
 *  promiscuous mode, and all-multi behavior.
 **/
static void igb_set_rx_mode(struct rtnet_device *netdev)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	unsigned int vfn = 0;
	u32 rctl, vmolr = 0;
	int count;

	/* Check for Promiscuous and All Multicast modes */
	rctl = rd32(E1000_RCTL);

	/* clear the effected bits */
	rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE | E1000_RCTL_VFE);

	if (netdev->flags & IFF_PROMISC) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		vmolr |= (E1000_VMOLR_ROPE | E1000_VMOLR_MPME);
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			rctl |= E1000_RCTL_MPE;
			vmolr |= E1000_VMOLR_MPME;
		} else {
			/* Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscuous mode so
			 * that we can at least receive multicast traffic
			 */
			count = igb_write_mc_addr_list(netdev);
			if (count < 0) {
				rctl |= E1000_RCTL_MPE;
				vmolr |= E1000_VMOLR_MPME;
			} else if (count) {
				vmolr |= E1000_VMOLR_ROMPE;
			}
		}
		/* Write addresses to available RAR registers, if there is not
		 * sufficient space to store all the addresses then enable
		 * unicast promiscuous mode
		 */
		count = igb_write_uc_addr_list(netdev);
		if (count < 0) {
			rctl |= E1000_RCTL_UPE;
			vmolr |= E1000_VMOLR_ROPE;
		}
		rctl |= E1000_RCTL_VFE;
	}
	wr32(E1000_RCTL, rctl);

	/* In order to support SR-IOV and eventually VMDq it is necessary to set
	 * the VMOLR to enable the appropriate modes.  Without this workaround
	 * we will have issues with VLAN tag stripping not being done for frames
	 * that are only arriving because we are the default pool
	 */
	if ((hw->mac.type < e1000_82576) || (hw->mac.type > e1000_i350))
		return;

	vmolr |= rd32(E1000_VMOLR(vfn)) &
		 ~(E1000_VMOLR_ROPE | E1000_VMOLR_MPME | E1000_VMOLR_ROMPE);
	wr32(E1000_VMOLR(vfn), vmolr);
}

static void igb_check_wvbr(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 wvbr = 0;

	switch (hw->mac.type) {
	case e1000_82576:
	case e1000_i350:
		wvbr = rd32(E1000_WVBR);
		if (!wvbr)
			return;
		break;
	default:
		break;
	}

	adapter->wvbr |= wvbr;
}

#define IGB_STAGGERED_QUEUE_OFFSET 8

/* Need to wait a few seconds after link up to get diagnostic information from
 * the phy
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static void igb_update_phy_info(struct timer_list *t)
{
	struct igb_adapter *adapter = from_timer(adapter, t, phy_info_timer);
#else /* < 4.14 */
static void igb_update_phy_info(unsigned long data)
{
	struct igb_adapter *adapter = (struct igb_adapter *) data;
#endif /* < 4.14 */
	igb_get_phy_info(&adapter->hw);
}

/**
 *  igb_has_link - check shared code for link and determine up/down
 *  @adapter: pointer to driver private info
 **/
bool igb_has_link(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = false;

	/* get_link_status is set on LSC (link status) interrupt or
	 * rx sequence error interrupt.  get_link_status will stay
	 * false until the e1000_check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (!hw->mac.get_link_status)
			return true;
		/* fallthrough */
	case e1000_media_type_internal_serdes:
		hw->mac.ops.check_for_link(hw);
		link_active = !hw->mac.get_link_status;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	if (((hw->mac.type == e1000_i210) ||
	     (hw->mac.type == e1000_i211)) &&
	     (hw->phy.id == I210_I_PHY_ID)) {
		if (!rtnetif_carrier_ok(adapter->netdev)) {
			adapter->flags &= ~IGB_FLAG_NEED_LINK_UPDATE;
		} else if (!(adapter->flags & IGB_FLAG_NEED_LINK_UPDATE)) {
			adapter->flags |= IGB_FLAG_NEED_LINK_UPDATE;
			adapter->link_check_timeout = jiffies;
		}
	}

	return link_active;
}

static bool igb_thermal_sensor_event(struct e1000_hw *hw, u32 event)
{
	bool ret = false;
	u32 ctrl_ext, thstat;

	/* check for thermal sensor event on i350 copper only */
	if (hw->mac.type == e1000_i350) {
		thstat = rd32(E1000_THSTAT);
		ctrl_ext = rd32(E1000_CTRL_EXT);

		if ((hw->phy.media_type == e1000_media_type_copper) &&
		    !(ctrl_ext & E1000_CTRL_EXT_LINK_MODE_SGMII))
			ret = !!(thstat & event);
	}

	return ret;
}

/**
 *  igb_check_lvmmc - check for malformed packets received
 *  and indicated in LVMMC register
 *  @adapter: pointer to adapter
 **/
static void igb_check_lvmmc(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 lvmmc;

	lvmmc = rd32(E1000_LVMMC);
	if (lvmmc) {
		if (unlikely(net_ratelimit())) {
			rtdev_warn(adapter->netdev,
				    "malformed Tx packet detected and dropped, LVMMC:0x%08x\n",
				    lvmmc);
		}
	}
}

/**
 *  igb_watchdog - Timer Call-back
 *  @data: pointer to adapter cast into an unsigned long
 **/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static void igb_watchdog(struct timer_list *t)
{
	struct igb_adapter *adapter = from_timer(adapter, t, watchdog_timer);
#else /* < 4.14 */
static void igb_watchdog(unsigned long data)
{
	struct igb_adapter *adapter = (struct igb_adapter *)data;
#endif /* < 4.14 */
	/* Do the rest outside of interrupt context */
	schedule_work(&adapter->watchdog_task);
}

static void igb_watchdog_task(struct work_struct *work)
{
	struct igb_adapter *adapter = container_of(work,
						   struct igb_adapter,
						   watchdog_task);
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_phy_info *phy = &hw->phy;
	struct rtnet_device *netdev = adapter->netdev;
	u32 link;
	int i;
	u32 connsw;

	link = igb_has_link(adapter);

	if (adapter->flags & IGB_FLAG_NEED_LINK_UPDATE) {
		if (time_after(jiffies, (adapter->link_check_timeout + HZ)))
			adapter->flags &= ~IGB_FLAG_NEED_LINK_UPDATE;
		else
			link = false;
	}

	/* Force link down if we have fiber to swap to */
	if (adapter->flags & IGB_FLAG_MAS_ENABLE) {
		if (hw->phy.media_type == e1000_media_type_copper) {
			connsw = rd32(E1000_CONNSW);
			if (!(connsw & E1000_CONNSW_AUTOSENSE_EN))
				link = 0;
		}
	}
	if (link) {
		/* Perform a reset if the media type changed. */
		if (hw->dev_spec._82575.media_changed) {
			hw->dev_spec._82575.media_changed = false;
			adapter->flags |= IGB_FLAG_MEDIA_RESET;
			igb_reset(adapter);
		}
		/* Cancel scheduled suspend requests. */
		pm_runtime_resume(adapter->pdev->dev.parent);

		if (!rtnetif_carrier_ok(netdev)) {
			u32 ctrl;

			hw->mac.ops.get_speed_and_duplex(hw,
							 &adapter->link_speed,
							 &adapter->link_duplex);

			ctrl = rd32(E1000_CTRL);
			/* Links status message must follow this format */
			rtdev_info(netdev,
			       "igb: %s NIC Link is Up %d Mbps %s Duplex, Flow Control: %s\n",
			       netdev->name,
			       adapter->link_speed,
			       adapter->link_duplex == FULL_DUPLEX ?
			       "Full" : "Half",
			       (ctrl & E1000_CTRL_TFCE) &&
			       (ctrl & E1000_CTRL_RFCE) ? "RX/TX" :
			       (ctrl & E1000_CTRL_RFCE) ?  "RX" :
			       (ctrl & E1000_CTRL_TFCE) ?  "TX" : "None");

			/* disable EEE if enabled */
			if ((adapter->flags & IGB_FLAG_EEE) &&
				(adapter->link_duplex == HALF_DUPLEX)) {
				dev_info(&adapter->pdev->dev,
				"EEE Disabled: unsupported at half duplex. Re-enable using ethtool when at full duplex.\n");
				adapter->hw.dev_spec._82575.eee_disable = true;
				adapter->flags &= ~IGB_FLAG_EEE;
			}

			/* check if SmartSpeed worked */
			igb_check_downshift(hw);
			if (phy->speed_downgraded)
				rtdev_warn(netdev, "Link Speed was downgraded by SmartSpeed\n");

			/* check for thermal sensor event */
			if (igb_thermal_sensor_event(hw,
			    E1000_THSTAT_LINK_THROTTLE))
				rtdev_info(netdev, "The network adapter link speed was downshifted because it overheated\n");

			/* adjust timeout factor according to speed/duplex */
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
			case SPEED_10:
				adapter->tx_timeout_factor = 14;
				break;
			case SPEED_100:
				/* maybe add some timeout factor ? */
				break;
			}

			rtnetif_carrier_on(netdev);

			/* link state has changed, schedule phy info update */
			if (!test_bit(__IGB_DOWN, &adapter->state))
				mod_timer(&adapter->phy_info_timer,
					  round_jiffies(jiffies + 2 * HZ));
		}
	} else {
		if (rtnetif_carrier_ok(netdev)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;

			/* check for thermal sensor event */
			if (igb_thermal_sensor_event(hw,
			    E1000_THSTAT_PWR_DOWN)) {
				rtdev_err(netdev, "The network adapter was stopped because it overheated\n");
			}

			/* Links status message must follow this format */
			rtdev_info(netdev, "igb: %s NIC Link is Down\n",
			       netdev->name);
			rtnetif_carrier_off(netdev);

			/* link state has changed, schedule phy info update */
			if (!test_bit(__IGB_DOWN, &adapter->state))
				mod_timer(&adapter->phy_info_timer,
					  round_jiffies(jiffies + 2 * HZ));

			/* link is down, time to check for alternate media */
			if (adapter->flags & IGB_FLAG_MAS_ENABLE) {
				igb_check_swap_media(adapter);
				if (adapter->flags & IGB_FLAG_MEDIA_RESET) {
					schedule_work(&adapter->reset_task);
					/* return immediately */
					return;
				}
			}
			pm_schedule_suspend(adapter->pdev->dev.parent,
					    MSEC_PER_SEC * 5);

		/* also check for alternate media here */
		} else if (!rtnetif_carrier_ok(netdev) &&
			   (adapter->flags & IGB_FLAG_MAS_ENABLE)) {
			igb_check_swap_media(adapter);
			if (adapter->flags & IGB_FLAG_MEDIA_RESET) {
				schedule_work(&adapter->reset_task);
				/* return immediately */
				return;
			}
		}
	}

	spin_lock(&adapter->stats64_lock);
	igb_update_stats(adapter);
	spin_unlock(&adapter->stats64_lock);

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igb_ring *tx_ring = adapter->tx_ring[i];
		if (!rtnetif_carrier_ok(netdev)) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context).
			 */
			if (igb_desc_unused(tx_ring) + 1 < tx_ring->count) {
				adapter->tx_timeout_count++;
				schedule_work(&adapter->reset_task);
				/* return immediately since reset is imminent */
				return;
			}
		}

		/* Force detection of hung controller every watchdog period */
		set_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags);
	}

	/* Cause software interrupt to ensure Rx ring is cleaned */
	if (adapter->flags & IGB_FLAG_HAS_MSIX) {
		u32 eics = 0;

		for (i = 0; i < adapter->num_q_vectors; i++)
			eics |= adapter->q_vector[i]->eims_value;
		wr32(E1000_EICS, eics);
	} else {
		wr32(E1000_ICS, E1000_ICS_RXDMT0);
	}

	/* Check LVMMC register on i350/i354 only */
	if ((adapter->hw.mac.type == e1000_i350) ||
	    (adapter->hw.mac.type == e1000_i354))
		igb_check_lvmmc(adapter);

	/* Reset the timer */
	if (!test_bit(__IGB_DOWN, &adapter->state)) {
		if (adapter->flags & IGB_FLAG_NEED_LINK_UPDATE)
			mod_timer(&adapter->watchdog_timer,
				  round_jiffies(jiffies +  HZ));
		else
			mod_timer(&adapter->watchdog_timer,
				  round_jiffies(jiffies + 2 * HZ));
	}
}

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

/**
 *  igb_update_ring_itr - update the dynamic ITR value based on packet size
 *  @q_vector: pointer to q_vector
 *
 *  Stores a new ITR value based on strictly on packet size.  This
 *  algorithm is less sophisticated than that used in igb_update_itr,
 *  due to the difficulty of synchronizing statistics across multiple
 *  receive rings.  The divisors and thresholds used by this function
 *  were determined based on theoretical maximum wire speed and testing
 *  data, in order to minimize response time while increasing bulk
 *  throughput.
 *  This functionality is controlled by ethtool's coalescing settings.
 *  NOTE:  This function is called only when operating in a multiqueue
 *         receive environment.
 **/
static void igb_update_ring_itr(struct igb_q_vector *q_vector)
{
	int new_val = q_vector->itr_val;
	int avg_wire_size = 0;
	struct igb_adapter *adapter = q_vector->adapter;
	unsigned int packets;

	if (!InterruptThrottle)
		return;

	/* For non-gigabit speeds, just fix the interrupt rate at 4000
	 * ints/sec - ITR timer value of 120 ticks.
	 */
	if (adapter->link_speed != SPEED_1000) {
		new_val = IGB_4K_ITR;
		goto set_itr_val;
	}

	packets = q_vector->rx.total_packets;
	if (packets)
		avg_wire_size = q_vector->rx.total_bytes / packets;

	packets = q_vector->tx.total_packets;
	if (packets)
		avg_wire_size = max_t(u32, avg_wire_size,
				      q_vector->tx.total_bytes / packets);

	/* if avg_wire_size isn't set no work was done */
	if (!avg_wire_size)
		goto clear_counts;

	/* Add 24 bytes to size to account for CRC, preamble, and gap */
	avg_wire_size += 24;

	/* Don't starve jumbo frames */
	avg_wire_size = min(avg_wire_size, 3000);

	/* Give a little boost to mid-size frames */
	if ((avg_wire_size > 300) && (avg_wire_size < 1200))
		new_val = avg_wire_size / 3;
	else
		new_val = avg_wire_size / 2;

	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (new_val < IGB_20K_ITR &&
	    ((q_vector->rx.ring && adapter->rx_itr_setting == 3) ||
	     (!q_vector->rx.ring && adapter->tx_itr_setting == 3)))
		new_val = IGB_20K_ITR;

set_itr_val:
	if (new_val != q_vector->itr_val) {
		q_vector->itr_val = new_val;
		q_vector->set_itr = 1;
	}
clear_counts:
	q_vector->rx.total_bytes = 0;
	q_vector->rx.total_packets = 0;
	q_vector->tx.total_bytes = 0;
	q_vector->tx.total_packets = 0;
}

/**
 *  igb_update_itr - update the dynamic ITR value based on statistics
 *  @q_vector: pointer to q_vector
 *  @ring_container: ring info to update the itr for
 *
 *  Stores a new ITR value based on packets and byte
 *  counts during the last interrupt.  The advantage of per interrupt
 *  computation is faster updates and more accurate ITR for the current
 *  traffic pattern.  Constants in this function were computed
 *  based on theoretical maximum wire speed and thresholds were set based
 *  on testing data as well as attempting to minimize response time
 *  while increasing bulk throughput.
 *  This functionality is controlled by ethtool's coalescing settings.
 *  NOTE:  These calculations are only valid when operating in a single-
 *         queue environment.
 **/
static void igb_update_itr(struct igb_q_vector *q_vector,
			   struct igb_ring_container *ring_container)
{
	unsigned int packets = ring_container->total_packets;
	unsigned int bytes = ring_container->total_bytes;
	u8 itrval = ring_container->itr;

	/* no packets, exit with status unchanged */
	if (packets == 0)
		return;

	switch (itrval) {
	case lowest_latency:
		/* handle TSO and jumbo frames */
		if (bytes/packets > 8000)
			itrval = bulk_latency;
		else if ((packets < 5) && (bytes > 512))
			itrval = low_latency;
		break;
	case low_latency:  /* 50 usec aka 20000 ints/s */
		if (bytes > 10000) {
			/* this if handles the TSO accounting */
			if (bytes/packets > 8000)
				itrval = bulk_latency;
			else if ((packets < 10) || ((bytes/packets) > 1200))
				itrval = bulk_latency;
			else if ((packets > 35))
				itrval = lowest_latency;
		} else if (bytes/packets > 2000) {
			itrval = bulk_latency;
		} else if (packets <= 2 && bytes < 512) {
			itrval = lowest_latency;
		}
		break;
	case bulk_latency: /* 250 usec aka 4000 ints/s */
		if (bytes > 25000) {
			if (packets > 35)
				itrval = low_latency;
		} else if (bytes < 1500) {
			itrval = low_latency;
		}
		break;
	}

	/* clear work counters since we have the values we need */
	ring_container->total_bytes = 0;
	ring_container->total_packets = 0;

	/* write updated itr to ring container */
	ring_container->itr = itrval;
}

static void igb_set_itr(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	u32 new_itr = q_vector->itr_val;
	u8 current_itr = 0;

	if (!InterruptThrottle)
		return;

	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (adapter->link_speed != SPEED_1000) {
		current_itr = 0;
		new_itr = IGB_4K_ITR;
		goto set_itr_now;
	}

	igb_update_itr(q_vector, &q_vector->tx);
	igb_update_itr(q_vector, &q_vector->rx);

	current_itr = max(q_vector->rx.itr, q_vector->tx.itr);

	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (current_itr == lowest_latency &&
	    ((q_vector->rx.ring && adapter->rx_itr_setting == 3) ||
	     (!q_vector->rx.ring && adapter->tx_itr_setting == 3)))
		current_itr = low_latency;

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = IGB_70K_ITR; /* 70,000 ints/sec */
		break;
	case low_latency:
		new_itr = IGB_20K_ITR; /* 20,000 ints/sec */
		break;
	case bulk_latency:
		new_itr = IGB_4K_ITR;  /* 4,000 ints/sec */
		break;
	default:
		break;
	}

set_itr_now:
	if (new_itr != q_vector->itr_val) {
		/* this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing
		 */
		new_itr = new_itr > q_vector->itr_val ?
			  max((new_itr * q_vector->itr_val) /
			  (new_itr + (q_vector->itr_val >> 2)),
			  new_itr) : new_itr;
		/* Don't write the value here; it resets the adapter's
		 * internal timer, and causes us to delay far longer than
		 * we should between interrupts.  Instead, we write the ITR
		 * value at the beginning of the next interrupt so the timing
		 * ends up being correct.
		 */
		q_vector->itr_val = new_itr;
		q_vector->set_itr = 1;
	}
}


#define IGB_SET_FLAG(_input, _flag, _result) \
	((_flag <= _result) ? \
	 ((u32)(_input & _flag) * (_result / _flag)) : \
	 ((u32)(_input & _flag) / (_flag / _result)))

static u32 igb_tx_cmd_type(struct rtskb *skb, u32 tx_flags)
{
	/* set type for advanced descriptor with frame checksum insertion */
	u32 cmd_type = E1000_ADVTXD_DTYP_DATA |
		       E1000_ADVTXD_DCMD_DEXT |
		       E1000_ADVTXD_DCMD_IFCS;

	return cmd_type;
}

static void igb_tx_olinfo_status(struct igb_ring *tx_ring,
				 union e1000_adv_tx_desc *tx_desc,
				 u32 tx_flags, unsigned int paylen)
{
	u32 olinfo_status = paylen << E1000_ADVTXD_PAYLEN_SHIFT;

	/* 82575 requires a unique index per ring */
	if (test_bit(IGB_RING_FLAG_TX_CTX_IDX, &tx_ring->flags))
		olinfo_status |= tx_ring->reg_idx << 4;

	tx_desc->read.olinfo_status = cpu_to_le32(olinfo_status);
}

static int __igb_maybe_stop_tx(struct igb_ring *tx_ring, const u16 size)
{
	struct rtnet_device *netdev = tx_ring->netdev;

	rtnetif_stop_queue(netdev);

	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it.
	 */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available.
	 */
	if (igb_desc_unused(tx_ring) < size)
		return -EBUSY;

	/* A reprieve! */
	rtnetif_wake_queue(netdev);

	tx_ring->tx_stats.restart_queue2++;

	return 0;
}

static inline int igb_maybe_stop_tx(struct igb_ring *tx_ring, const u16 size)
{
	if (igb_desc_unused(tx_ring) >= size)
		return 0;
	return __igb_maybe_stop_tx(tx_ring, size);
}

static void igb_tx_map(struct igb_ring *tx_ring,
		       struct igb_tx_buffer *first,
		       const u8 hdr_len)
{
	struct rtskb *skb = first->skb;
	struct igb_tx_buffer *tx_buffer;
	union e1000_adv_tx_desc *tx_desc;
	dma_addr_t dma;
	unsigned int size;
	u32 tx_flags = first->tx_flags;
	u32 cmd_type = igb_tx_cmd_type(skb, tx_flags);
	u16 i = tx_ring->next_to_use;

	/* first descriptor is also last, set RS and EOP bits */
	cmd_type |= IGB_TXD_DCMD;
	tx_desc = IGB_TX_DESC(tx_ring, i);

	igb_tx_olinfo_status(tx_ring, tx_desc, tx_flags, skb->len - hdr_len);

	size = skb->len;

	dma = rtskb_data_dma_addr(skb, 0);

	tx_buffer = first;

	tx_desc->read.buffer_addr = cpu_to_le64(dma);
	tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type ^ size);

	/* set the timestamp */
	first->time_stamp = jiffies;
	first->next_to_watch = tx_desc;

	i++;
	tx_desc++;
	if (i == tx_ring->count) {
		tx_desc = IGB_TX_DESC(tx_ring, 0);
		i = 0;
	}

	/* Force memory writes to complete before letting h/w know there
	 * are new descriptors to fetch.  (Only applicable for weak-ordered
	 * memory model archs, such as IA-64).
	 *
	 * We also need this memory barrier to make certain all of the
	 * status bits have been updated before next_to_watch is written.
	 */
	wmb();

	if (skb->xmit_stamp)
		*skb->xmit_stamp =
			cpu_to_be64(rtdm_clock_read() + *skb->xmit_stamp);
	/* set next_to_watch value indicating a packet is present */
	tx_ring->next_to_use = i;

	/* Make sure there is space in the ring for the next send. */
	igb_maybe_stop_tx(tx_ring, DESC_NEEDED);

	writel(i, tx_ring->tail);

	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems
	 */
	mmiowb();

	return;
}

netdev_tx_t igb_xmit_frame_ring(struct rtskb *skb,
				struct igb_ring *tx_ring)
{
	struct igb_tx_buffer *first;
	u32 tx_flags = 0;
	u16 count = 2;
	u8 hdr_len = 0;

	/* need: 1 descriptor per page * PAGE_SIZE/IGB_MAX_DATA_PER_TXD,
	 *       + 1 desc for skb_headlen/IGB_MAX_DATA_PER_TXD,
	 *       + 2 desc gap to keep tail from touching head,
	 *       + 1 desc for context descriptor,
	 * otherwise try next time
	 */
	if (igb_maybe_stop_tx(tx_ring, count + 3)) {
		/* this is a hard error */
		return NETDEV_TX_BUSY;
	}

	if (skb->protocol == htons(ETH_P_IP))
		tx_flags |= IGB_TX_FLAGS_IPV4;

	/* record the location of the first descriptor for this packet */
	first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];
	first->skb = skb;
	first->bytecount = skb->len;
	first->gso_segs = 1;

	/* record initial flags and protocol */
	first->tx_flags = tx_flags;
	first->protocol = skb->protocol;

	igb_tx_map(tx_ring, first, hdr_len);

	return NETDEV_TX_OK;
}

static inline struct igb_ring *igb_tx_queue_mapping(struct igb_adapter *adapter,
						    struct rtskb *skb)
{
	return adapter->tx_ring[0];
}

static netdev_tx_t igb_xmit_frame(struct rtskb *skb,
				  struct rtnet_device *netdev)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);

	if (test_bit(__IGB_DOWN, &adapter->state)) {
		kfree_rtskb(skb);
		return NETDEV_TX_OK;
	}

	if (skb->len <= 0) {
		kfree_rtskb(skb);
		return NETDEV_TX_OK;
	}

	/* The minimum packet size with TCTL.PSP set is 17 so pad the skb
	 * in order to meet this minimum size requirement.
	 */
	if (skb->len < 17) {
		skb = rtskb_padto(skb, 17);
		if (!skb)
			return NETDEV_TX_OK;
	}

	return igb_xmit_frame_ring(skb, igb_tx_queue_mapping(adapter, skb));
}

static void igb_reset_task(struct work_struct *work)
{
	struct igb_adapter *adapter;
	adapter = container_of(work, struct igb_adapter, reset_task);

	igb_dump(adapter);
	rtdev_err(adapter->netdev, "Reset adapter\n");
	igb_reinit_locked(adapter);
}

/**
 * igb_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats *
igb_get_stats(struct rtnet_device *netdev)
{
	struct igb_adapter *adapter = netdev->priv;

	/* only return the current stats */
	return &adapter->net_stats;
}

/**
 *  igb_update_stats - Update the board statistics counters
 *  @adapter: board private structure
 **/
void igb_update_stats(struct igb_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	struct net_device_stats *net_stats;
	u32 reg, mpc;
	int i;
	u64 bytes, packets;

	/* Prevent stats update while adapter is being reset, or if the pci
	 * connection is down.
	 */
	if (adapter->link_speed == 0)
		return;
	if (pci_channel_offline(pdev))
		return;

	net_stats = &adapter->net_stats;
	bytes = 0;
	packets = 0;

	rcu_read_lock();
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct igb_ring *ring = adapter->rx_ring[i];
		u32 rqdpc = rd32(E1000_RQDPC(i));
		if (hw->mac.type >= e1000_i210)
			wr32(E1000_RQDPC(i), 0);

		if (rqdpc) {
			ring->rx_stats.drops += rqdpc;
			net_stats->rx_fifo_errors += rqdpc;
		}

		bytes += ring->rx_stats.bytes;
		packets += ring->rx_stats.packets;
	}

	net_stats->rx_bytes = bytes;
	net_stats->rx_packets = packets;

	bytes = 0;
	packets = 0;
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igb_ring *ring = adapter->tx_ring[i];
		bytes += ring->tx_stats.bytes;
		packets += ring->tx_stats.packets;
	}
	net_stats->tx_bytes = bytes;
	net_stats->tx_packets = packets;
	rcu_read_unlock();

	/* read stats registers */
	adapter->stats.crcerrs += rd32(E1000_CRCERRS);
	adapter->stats.gprc += rd32(E1000_GPRC);
	adapter->stats.gorc += rd32(E1000_GORCL);
	rd32(E1000_GORCH); /* clear GORCL */
	adapter->stats.bprc += rd32(E1000_BPRC);
	adapter->stats.mprc += rd32(E1000_MPRC);
	adapter->stats.roc += rd32(E1000_ROC);

	adapter->stats.prc64 += rd32(E1000_PRC64);
	adapter->stats.prc127 += rd32(E1000_PRC127);
	adapter->stats.prc255 += rd32(E1000_PRC255);
	adapter->stats.prc511 += rd32(E1000_PRC511);
	adapter->stats.prc1023 += rd32(E1000_PRC1023);
	adapter->stats.prc1522 += rd32(E1000_PRC1522);
	adapter->stats.symerrs += rd32(E1000_SYMERRS);
	adapter->stats.sec += rd32(E1000_SEC);

	mpc = rd32(E1000_MPC);
	adapter->stats.mpc += mpc;
	net_stats->rx_fifo_errors += mpc;
	adapter->stats.scc += rd32(E1000_SCC);
	adapter->stats.ecol += rd32(E1000_ECOL);
	adapter->stats.mcc += rd32(E1000_MCC);
	adapter->stats.latecol += rd32(E1000_LATECOL);
	adapter->stats.dc += rd32(E1000_DC);
	adapter->stats.rlec += rd32(E1000_RLEC);
	adapter->stats.xonrxc += rd32(E1000_XONRXC);
	adapter->stats.xontxc += rd32(E1000_XONTXC);
	adapter->stats.xoffrxc += rd32(E1000_XOFFRXC);
	adapter->stats.xofftxc += rd32(E1000_XOFFTXC);
	adapter->stats.fcruc += rd32(E1000_FCRUC);
	adapter->stats.gptc += rd32(E1000_GPTC);
	adapter->stats.gotc += rd32(E1000_GOTCL);
	rd32(E1000_GOTCH); /* clear GOTCL */
	adapter->stats.rnbc += rd32(E1000_RNBC);
	adapter->stats.ruc += rd32(E1000_RUC);
	adapter->stats.rfc += rd32(E1000_RFC);
	adapter->stats.rjc += rd32(E1000_RJC);
	adapter->stats.tor += rd32(E1000_TORH);
	adapter->stats.tot += rd32(E1000_TOTH);
	adapter->stats.tpr += rd32(E1000_TPR);

	adapter->stats.ptc64 += rd32(E1000_PTC64);
	adapter->stats.ptc127 += rd32(E1000_PTC127);
	adapter->stats.ptc255 += rd32(E1000_PTC255);
	adapter->stats.ptc511 += rd32(E1000_PTC511);
	adapter->stats.ptc1023 += rd32(E1000_PTC1023);
	adapter->stats.ptc1522 += rd32(E1000_PTC1522);

	adapter->stats.mptc += rd32(E1000_MPTC);
	adapter->stats.bptc += rd32(E1000_BPTC);

	adapter->stats.tpt += rd32(E1000_TPT);
	adapter->stats.colc += rd32(E1000_COLC);

	adapter->stats.algnerrc += rd32(E1000_ALGNERRC);
	/* read internal phy specific stats */
	reg = rd32(E1000_CTRL_EXT);
	if (!(reg & E1000_CTRL_EXT_LINK_MODE_MASK)) {
		adapter->stats.rxerrc += rd32(E1000_RXERRC);

		/* this stat has invalid values on i210/i211 */
		if ((hw->mac.type != e1000_i210) &&
		    (hw->mac.type != e1000_i211))
			adapter->stats.tncrs += rd32(E1000_TNCRS);
	}

	adapter->stats.tsctc += rd32(E1000_TSCTC);
	adapter->stats.tsctfc += rd32(E1000_TSCTFC);

	adapter->stats.iac += rd32(E1000_IAC);
	adapter->stats.icrxoc += rd32(E1000_ICRXOC);
	adapter->stats.icrxptc += rd32(E1000_ICRXPTC);
	adapter->stats.icrxatc += rd32(E1000_ICRXATC);
	adapter->stats.ictxptc += rd32(E1000_ICTXPTC);
	adapter->stats.ictxatc += rd32(E1000_ICTXATC);
	adapter->stats.ictxqec += rd32(E1000_ICTXQEC);
	adapter->stats.ictxqmtc += rd32(E1000_ICTXQMTC);
	adapter->stats.icrxdmtc += rd32(E1000_ICRXDMTC);

	/* Fill out the OS statistics structure */
	net_stats->multicast = adapter->stats.mprc;
	net_stats->collisions = adapter->stats.colc;

	/* Rx Errors */

	/* RLEC on some newer hardware can be incorrect so build
	 * our own version based on RUC and ROC
	 */
	net_stats->rx_errors = adapter->stats.rxerrc +
		adapter->stats.crcerrs + adapter->stats.algnerrc +
		adapter->stats.ruc + adapter->stats.roc +
		adapter->stats.cexterr;
	net_stats->rx_length_errors = adapter->stats.ruc +
				      adapter->stats.roc;
	net_stats->rx_crc_errors = adapter->stats.crcerrs;
	net_stats->rx_frame_errors = adapter->stats.algnerrc;
	net_stats->rx_missed_errors = adapter->stats.mpc;

	/* Tx Errors */
	net_stats->tx_errors = adapter->stats.ecol +
			       adapter->stats.latecol;
	net_stats->tx_aborted_errors = adapter->stats.ecol;
	net_stats->tx_window_errors = adapter->stats.latecol;
	net_stats->tx_carrier_errors = adapter->stats.tncrs;

	/* Tx Dropped needs to be maintained elsewhere */

	/* Management Stats */
	adapter->stats.mgptc += rd32(E1000_MGTPTC);
	adapter->stats.mgprc += rd32(E1000_MGTPRC);
	adapter->stats.mgpdc += rd32(E1000_MGTPDC);

	/* OS2BMC Stats */
	reg = rd32(E1000_MANC);
	if (reg & E1000_MANC_EN_BMC2OS) {
		adapter->stats.o2bgptc += rd32(E1000_O2BGPTC);
		adapter->stats.o2bspc += rd32(E1000_O2BSPC);
		adapter->stats.b2ospc += rd32(E1000_B2OSPC);
		adapter->stats.b2ogprc += rd32(E1000_B2OGPRC);
	}
}

static void igb_nrtsig_watchdog(rtdm_nrtsig_t *sig, void *data)
{
	struct igb_adapter *adapter = data;
	mod_timer(&adapter->watchdog_timer, jiffies + 1);
}

static void igb_other_handler(struct igb_adapter *adapter, u32 icr, bool root)
{
	struct e1000_hw *hw = &adapter->hw;

	if (icr & E1000_ICR_DRSTA)
		rtdm_schedule_nrt_work(&adapter->reset_task);

	if (icr & E1000_ICR_DOUTSYNC) {
		/* HW is reporting DMA is out of sync */
		adapter->stats.doosync++;
		/* The DMA Out of Sync is also indication of a spoof event
		 * in IOV mode. Check the Wrong VM Behavior register to
		 * see if it is really a spoof event.
		 */
		igb_check_wvbr(adapter);
	}

	if (icr & E1000_ICR_LSC) {
		hw->mac.get_link_status = 1;
		/* guard against interrupt when we're going down */
		if (!test_bit(__IGB_DOWN, &adapter->state)) {
			if (root)
				mod_timer(&adapter->watchdog_timer,
					jiffies + 1);
			else
				rtdm_nrtsig_pend(&adapter->watchdog_nrtsig);
		}
	}
}

static irqreturn_t igb_msix_other(int irq, void *data)
{
	struct igb_adapter *adapter = data;
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = rd32(E1000_ICR);
	/* reading ICR causes bit 31 of EICR to be cleared */

	igb_other_handler(adapter, icr, true);

	wr32(E1000_EIMS, adapter->eims_other);

	return IRQ_HANDLED;
}

static void igb_write_itr(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	u32 itr_val = (q_vector->itr_val + 0x3) & 0x7FFC;

	if (!q_vector->set_itr)
		return;

	if (!itr_val)
		itr_val = 0x4;

	if (adapter->hw.mac.type == e1000_82575)
		itr_val |= itr_val << 16;
	else
		itr_val |= E1000_EITR_CNT_IGNR;

	writel(itr_val, q_vector->itr_register);
	q_vector->set_itr = 0;
}

static int igb_msix_ring(rtdm_irq_t *ih)
{
	struct igb_q_vector *q_vector =
		rtdm_irq_get_arg(ih, struct igb_q_vector);

	/* Write the ITR value calculated from the previous interrupt. */
	igb_write_itr(q_vector);

	igb_poll(q_vector);

	return RTDM_IRQ_HANDLED;
}


/**
 *  igb_intr_msi - Interrupt Handler
 *  @irq: interrupt number
 *  @data: pointer to a network interface device structure
 **/
static int igb_intr_msi(rtdm_irq_t *ih)
{
	struct igb_adapter *adapter =
		rtdm_irq_get_arg(ih, struct igb_adapter);
	struct igb_q_vector *q_vector = adapter->q_vector[0];
	struct e1000_hw *hw = &adapter->hw;
	u32 icr = rd32(E1000_ICR);

	igb_write_itr(q_vector);

	igb_other_handler(adapter, icr, false);

	igb_poll(q_vector);

	return RTDM_IRQ_HANDLED;
}

/**
 *  igb_intr - Legacy Interrupt Handler
 *  @irq: interrupt number
 *  @data: pointer to a network interface device structure
 **/
static int igb_intr(rtdm_irq_t *ih)
{
	struct igb_adapter *adapter =
		rtdm_irq_get_arg(ih, struct igb_adapter);
	struct igb_q_vector *q_vector = adapter->q_vector[0];
	struct e1000_hw *hw = &adapter->hw;
	/* Interrupt Auto-Mask...upon reading ICR, interrupts are masked.  No
	 * need for the IMC write
	 */
	u32 icr = rd32(E1000_ICR);

	/* IMS will not auto-mask if INT_ASSERTED is not set, and if it is
	 * not set, then the adapter didn't send an interrupt
	 */
	if (!(icr & E1000_ICR_INT_ASSERTED))
		return IRQ_NONE;

	igb_write_itr(q_vector);

	igb_other_handler(adapter, icr, false);

	igb_poll(q_vector);

	return RTDM_IRQ_HANDLED;
}

static void igb_ring_irq_enable(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct e1000_hw *hw = &adapter->hw;

	if ((q_vector->rx.ring && (adapter->rx_itr_setting & 3)) ||
	    (!q_vector->rx.ring && (adapter->tx_itr_setting & 3))) {
		if (adapter->num_q_vectors == 1)
			igb_set_itr(q_vector);
		else
			igb_update_ring_itr(q_vector);
	}

	if (!test_bit(__IGB_DOWN, &adapter->state)) {
		if (adapter->flags & IGB_FLAG_HAS_MSIX)
			wr32(E1000_EIMS, q_vector->eims_value);
		else
			igb_irq_enable(adapter);
	}
}

/**
 *  igb_poll - NAPI Rx polling callback
 *  @napi: napi polling structure
 *  @budget: count of how many packets we should handle
 **/
static void igb_poll(struct igb_q_vector *q_vector)
{
	if (q_vector->tx.ring)
		igb_clean_tx_irq(q_vector);

	if (q_vector->rx.ring)
		igb_clean_rx_irq(q_vector, 64);

	igb_ring_irq_enable(q_vector);
}

/**
 *  igb_clean_tx_irq - Reclaim resources after transmit completes
 *  @q_vector: pointer to q_vector containing needed info
 *
 *  returns true if ring is completely cleaned
 **/
static bool igb_clean_tx_irq(struct igb_q_vector *q_vector)
{
	struct igb_adapter *adapter = q_vector->adapter;
	struct igb_ring *tx_ring = q_vector->tx.ring;
	struct igb_tx_buffer *tx_buffer;
	union e1000_adv_tx_desc *tx_desc;
	unsigned int total_bytes = 0, total_packets = 0;
	unsigned int budget = q_vector->tx.work_limit;
	unsigned int i = tx_ring->next_to_clean;

	if (test_bit(__IGB_DOWN, &adapter->state))
		return true;

	tx_buffer = &tx_ring->tx_buffer_info[i];
	tx_desc = IGB_TX_DESC(tx_ring, i);
	i -= tx_ring->count;

	do {
		union e1000_adv_tx_desc *eop_desc = tx_buffer->next_to_watch;

		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* prevent any other reads prior to eop_desc */
		read_barrier_depends();

		/* if DD is not set pending work has not been completed */
		if (!(eop_desc->wb.status & cpu_to_le32(E1000_TXD_STAT_DD)))
			break;

		/* clear next_to_watch to prevent false hangs */
		tx_buffer->next_to_watch = NULL;

		/* update the statistics for this packet */
		total_bytes += tx_buffer->bytecount;
		total_packets += tx_buffer->gso_segs;

		/* free the skb */
		kfree_rtskb(tx_buffer->skb);

		/* clear tx_buffer data */
		tx_buffer->skb = NULL;

		/* clear last DMA location and unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= tx_ring->count;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = IGB_TX_DESC(tx_ring, 0);
			}
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		tx_desc++;
		i++;
		if (unlikely(!i)) {
			i -= tx_ring->count;
			tx_buffer = tx_ring->tx_buffer_info;
			tx_desc = IGB_TX_DESC(tx_ring, 0);
		}

		/* issue prefetch for next Tx descriptor */
		prefetch(tx_desc);

		/* update budget accounting */
		budget--;
	} while (likely(budget));

	i += tx_ring->count;
	tx_ring->next_to_clean = i;
	tx_ring->tx_stats.bytes += total_bytes;
	tx_ring->tx_stats.packets += total_packets;
	q_vector->tx.total_bytes += total_bytes;
	q_vector->tx.total_packets += total_packets;

	if (test_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags)) {
		struct e1000_hw *hw = &adapter->hw;

		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i
		 */
		clear_bit(IGB_RING_FLAG_TX_DETECT_HANG, &tx_ring->flags);
		if (tx_buffer->next_to_watch &&
		    time_after(jiffies, tx_buffer->time_stamp +
			       (adapter->tx_timeout_factor * HZ)) &&
		    !(rd32(E1000_STATUS) & E1000_STATUS_TXOFF)) {

			/* detected Tx unit hang */
			dev_err(tx_ring->dev,
				"Detected Tx Unit Hang\n"
				"  Tx Queue             <%d>\n"
				"  TDH                  <%x>\n"
				"  TDT                  <%x>\n"
				"  next_to_use          <%x>\n"
				"  next_to_clean        <%x>\n"
				"buffer_info[next_to_clean]\n"
				"  time_stamp           <%lx>\n"
				"  next_to_watch        <%p>\n"
				"  jiffies              <%lx>\n"
				"  desc.status          <%x>\n",
				tx_ring->queue_index,
				rd32(E1000_TDH(tx_ring->reg_idx)),
				readl(tx_ring->tail),
				tx_ring->next_to_use,
				tx_ring->next_to_clean,
				tx_buffer->time_stamp,
				tx_buffer->next_to_watch,
				jiffies,
				tx_buffer->next_to_watch->wb.status);
			rtnetif_stop_queue(tx_ring->netdev);

			/* we are about to reset, no point in enabling stuff */
			return true;
		}
	}

#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (unlikely(total_packets &&
	    rtnetif_carrier_ok(tx_ring->netdev) &&
	    igb_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
		if (rtnetif_queue_stopped(tx_ring->netdev) &&
		    !(test_bit(__IGB_DOWN, &adapter->state))) {
			rtnetif_wake_queue(tx_ring->netdev);

			tx_ring->tx_stats.restart_queue++;
		}
	}

	return !!budget;
}

static struct rtskb *igb_fetch_rx_buffer(struct igb_ring *rx_ring,
					   union e1000_adv_rx_desc *rx_desc)
{
	struct igb_rx_buffer *rx_buffer;
	struct rtskb *skb;

	rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
	skb = rx_buffer->skb;
	prefetchw(skb->data);

	/* pull the header of the skb in */
	rtskb_put(skb, le16_to_cpu(rx_desc->wb.upper.length));
	rx_buffer->skb = NULL;
	rx_buffer->dma = 0;

	return skb;
}

static inline void igb_rx_checksum(struct igb_ring *ring,
				   union e1000_adv_rx_desc *rx_desc,
				   struct rtskb *skb)
{
	skb->ip_summed = CHECKSUM_NONE;

	/* Ignore Checksum bit is set */
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_IXSM))
		return;

	/* Rx checksum disabled via ethtool */
	if (!(ring->netdev->features & NETIF_F_RXCSUM))
		return;

	/* TCP/UDP checksum error bit is set */
	if (igb_test_staterr(rx_desc,
			     E1000_RXDEXT_STATERR_TCPE |
			     E1000_RXDEXT_STATERR_IPE)) {
		/* work around errata with sctp packets where the TCPE aka
		 * L4E bit is set incorrectly on 64 byte (60 byte w/o crc)
		 * packets, (aka let the stack check the crc32c)
		 */
		if (!((skb->len == 60) &&
		      test_bit(IGB_RING_FLAG_RX_SCTP_CSUM, &ring->flags))) {
			ring->rx_stats.csum_err++;
		}
		/* let the stack verify checksum errors */
		return;
	}
	/* It must be a TCP or UDP packet with a valid checksum */
	if (igb_test_staterr(rx_desc, E1000_RXD_STAT_TCPCS |
				      E1000_RXD_STAT_UDPCS))
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	dev_dbg(ring->dev, "cksum success: bits %08X\n",
		le32_to_cpu(rx_desc->wb.upper.status_error));
}

/**
 *  igb_is_non_eop - process handling of non-EOP buffers
 *  @rx_ring: Rx ring being processed
 *  @rx_desc: Rx descriptor for current buffer
 *  @skb: current socket buffer containing buffer in progress
 *
 *  This function updates next to clean.  If the buffer is an EOP buffer
 *  this function exits returning false, otherwise it will place the
 *  sk_buff in the next buffer to be chained and return true indicating
 *  that this is in fact a non-EOP buffer.
 **/
static bool igb_is_non_eop(struct igb_ring *rx_ring,
			   union e1000_adv_rx_desc *rx_desc)
{
	u32 ntc = rx_ring->next_to_clean + 1;

	/* fetch, update, and store next to clean */
	ntc = (ntc < rx_ring->count) ? ntc : 0;
	rx_ring->next_to_clean = ntc;

	prefetch(IGB_RX_DESC(rx_ring, ntc));

	if (likely(igb_test_staterr(rx_desc, E1000_RXD_STAT_EOP)))
		return false;

	return true;
}

/**
 *  igb_cleanup_headers - Correct corrupted or empty headers
 *  @rx_ring: rx descriptor ring packet is being transacted on
 *  @rx_desc: pointer to the EOP Rx descriptor
 *  @skb: pointer to current skb being fixed
 *
 *  Address the case where we are pulling data in on pages only
 *  and as such no data is present in the skb header.
 *
 *  In addition if skb is not at least 60 bytes we need to pad it so that
 *  it is large enough to qualify as a valid Ethernet frame.
 *
 *  Returns true if an error was encountered and skb was freed.
 **/
static bool igb_cleanup_headers(struct igb_ring *rx_ring,
				union e1000_adv_rx_desc *rx_desc,
				struct rtskb *skb)
{
	if (unlikely((igb_test_staterr(rx_desc,
				       E1000_RXDEXT_ERR_FRAME_ERR_MASK)))) {
		struct rtnet_device *netdev = rx_ring->netdev;
		if (!(netdev->features & NETIF_F_RXALL)) {
			kfree_rtskb(skb);
			return true;
		}
	}

	return false;
}

/**
 *  igb_process_skb_fields - Populate skb header fields from Rx descriptor
 *  @rx_ring: rx descriptor ring packet is being transacted on
 *  @rx_desc: pointer to the EOP Rx descriptor
 *  @skb: pointer to current skb being populated
 *
 *  This function checks the ring, descriptor, and packet information in
 *  order to populate the hash, checksum, VLAN, timestamp, protocol, and
 *  other fields within the skb.
 **/
static void igb_process_skb_fields(struct igb_ring *rx_ring,
				   union e1000_adv_rx_desc *rx_desc,
				   struct rtskb *skb)
{
	igb_rx_checksum(rx_ring, rx_desc, skb);

	skb->protocol = rt_eth_type_trans(skb, rx_ring->netdev);
}

static bool igb_clean_rx_irq(struct igb_q_vector *q_vector, const int budget)
{
	struct igb_ring *rx_ring = q_vector->rx.ring;
	unsigned int total_bytes = 0, total_packets = 0;
	u16 cleaned_count = igb_desc_unused(rx_ring);
	nanosecs_abs_t time_stamp = rtdm_clock_read();
	struct rtskb *skb;

	while (likely(total_packets < budget)) {
		union e1000_adv_rx_desc *rx_desc;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= IGB_RX_BUFFER_WRITE) {
			igb_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		rx_desc = IGB_RX_DESC(rx_ring, rx_ring->next_to_clean);

		if (!rx_desc->wb.upper.status_error)
			break;

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back
		 */
		rmb();

		/* retrieve a buffer from the ring */
		skb = igb_fetch_rx_buffer(rx_ring, rx_desc);
		skb->time_stamp = time_stamp;

		cleaned_count++;

		/* fetch next buffer in frame if non-eop */
		if (igb_is_non_eop(rx_ring, rx_desc)) {
			kfree_rtskb(skb);
			continue;
		}

		/* verify the packet layout is correct */
		if (igb_cleanup_headers(rx_ring, rx_desc, skb))
			continue;

		/* probably a little skewed due to removing CRC */
		total_bytes += skb->len;

		/* populate checksum, timestamp, VLAN, and protocol */
		igb_process_skb_fields(rx_ring, rx_desc, skb);

		rtnetif_rx(skb);

		/* reset skb pointer */
		skb = NULL;

		/* update budget accounting */
		total_packets++;
	}

	rx_ring->rx_stats.packets += total_packets;
	rx_ring->rx_stats.bytes += total_bytes;
	q_vector->rx.total_packets += total_packets;
	q_vector->rx.total_bytes += total_bytes;

	if (cleaned_count)
		igb_alloc_rx_buffers(rx_ring, cleaned_count);

	if (total_packets)
		rt_mark_stack_mgr(q_vector->adapter->netdev);

	return total_packets < budget;
}

static bool igb_alloc_mapped_skb(struct igb_ring *rx_ring,
				 struct igb_rx_buffer *bi)
{
	struct igb_adapter *adapter = rx_ring->q_vector->adapter;
	struct rtskb *skb = bi->skb;
	dma_addr_t dma = bi->dma;

	if (dma)
		return true;

	if (likely(!skb)) {
		skb = rtnetdev_alloc_rtskb(adapter->netdev,
					rx_ring->rx_buffer_len + NET_IP_ALIGN);
		if (!skb) {
			rx_ring->rx_stats.alloc_failed++;
			return false;
		}

		rtskb_reserve(skb, NET_IP_ALIGN);
		skb->rtdev = adapter->netdev;

		bi->skb = skb;
		bi->dma = rtskb_data_dma_addr(skb, 0);
	}

	return true;
}

/**
 *  igb_alloc_rx_buffers - Replace used receive buffers; packet split
 *  @adapter: address of board private structure
 **/
void igb_alloc_rx_buffers(struct igb_ring *rx_ring, u16 cleaned_count)
{
	union e1000_adv_rx_desc *rx_desc;
	struct igb_rx_buffer *bi;
	u16 i = rx_ring->next_to_use;

	/* nothing to do */
	if (!cleaned_count)
		return;

	rx_desc = IGB_RX_DESC(rx_ring, i);
	bi = &rx_ring->rx_buffer_info[i];
	i -= rx_ring->count;

	do {
		if (!igb_alloc_mapped_skb(rx_ring, bi))
			break;

		/* Refresh the desc even if buffer_addrs didn't change
		 * because each write-back erases this info.
		 */
		rx_desc->read.pkt_addr = cpu_to_le64(bi->dma);

		rx_desc++;
		bi++;
		i++;
		if (unlikely(!i)) {
			rx_desc = IGB_RX_DESC(rx_ring, 0);
			bi = rx_ring->rx_buffer_info;
			i -= rx_ring->count;
		}

		/* clear the status bits for the next_to_use descriptor */
		rx_desc->wb.upper.status_error = 0;

		cleaned_count--;
	} while (cleaned_count);

	i += rx_ring->count;

	if (rx_ring->next_to_use != i) {
		/* record the next descriptor to use */
		rx_ring->next_to_use = i;

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64).
		 */
		wmb();
		writel(i, rx_ring->tail);
	}
}

/**
 * igb_mii_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int igb_mii_ioctl(struct rtnet_device *netdev, struct ifreq *ifr, int cmd)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);

	if (adapter->hw.phy.media_type != e1000_media_type_copper)
		return -EOPNOTSUPP;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = adapter->hw.phy.addr;
		break;
	case SIOCGMIIREG:
		if (igb_read_phy_reg(&adapter->hw, data->reg_num & 0x1F,
				     &data->val_out))
			return -EIO;
		break;
	case SIOCSMIIREG:
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/**
 * igb_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int igb_ioctl(struct rtnet_device *netdev, struct ifreq *ifr, int cmd)
{
	if (rtdm_in_rt_context())
		return -ENOSYS;
	
	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return igb_mii_ioctl(netdev, ifr, cmd);

	default:
		return -EOPNOTSUPP;
	}
}

void igb_read_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = hw->back;

	pci_read_config_word(adapter->pdev, reg, value);
}

void igb_write_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = hw->back;

	pci_write_config_word(adapter->pdev, reg, *value);
}

s32 igb_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = hw->back;

	if (pcie_capability_read_word(adapter->pdev, reg, value))
		return -E1000_ERR_CONFIG;

	return 0;
}

s32 igb_write_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct igb_adapter *adapter = hw->back;

	if (pcie_capability_write_word(adapter->pdev, reg, *value))
		return -E1000_ERR_CONFIG;

	return 0;
}

static void igb_vlan_mode(struct rtnet_device *netdev, netdev_features_t features)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl;

	/* disable VLAN tag insert/strip */
	ctrl = rd32(E1000_CTRL);
	ctrl &= ~E1000_CTRL_VME;
	wr32(E1000_CTRL, ctrl);

	igb_rlpml_set(adapter);
}

static int igb_vlan_rx_add_vid(struct rtnet_device *netdev,
			       __be16 proto, u16 vid)
{
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	/* add the filter since PF can receive vlans w/o entry in vlvf */
	igb_vfta_set(hw, vid, true);

	set_bit(vid, adapter->active_vlans);

	return 0;
}

static void igb_restore_vlan(struct igb_adapter *adapter)
{
	u16 vid;

	igb_vlan_mode(adapter->netdev, adapter->netdev->features);

	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID)
		igb_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), vid);
}

static int __igb_shutdown(struct pci_dev *pdev, bool *enable_wake,
			  bool runtime)
{
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl, rctl, status;
	u32 wufc = runtime ? E1000_WUFC_LNKC : adapter->wol;
#ifdef CONFIG_PM
	int retval = 0;
#endif

	rtnetif_device_detach(netdev);

	if (rtnetif_running(netdev))
		__igb_close(netdev, true);

	igb_clear_interrupt_scheme(adapter);

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;
#endif

	status = rd32(E1000_STATUS);
	if (status & E1000_STATUS_LU)
		wufc &= ~E1000_WUFC_LNKC;

	if (wufc) {
		igb_setup_rctl(adapter);
		igb_set_rx_mode(netdev);

		/* turn on all-multi mode if wake on multicast is enabled */
		if (wufc & E1000_WUFC_MC) {
			rctl = rd32(E1000_RCTL);
			rctl |= E1000_RCTL_MPE;
			wr32(E1000_RCTL, rctl);
		}

		ctrl = rd32(E1000_CTRL);
		/* advertise wake from D3Cold */
		#define E1000_CTRL_ADVD3WUC 0x00100000
		/* phy power management enable */
		#define E1000_CTRL_EN_PHY_PWR_MGMT 0x00200000
		ctrl |= E1000_CTRL_ADVD3WUC;
		wr32(E1000_CTRL, ctrl);

		/* Allow time for pending master requests to run */
		igb_disable_pcie_master(hw);

		wr32(E1000_WUC, E1000_WUC_PME_EN);
		wr32(E1000_WUFC, wufc);
	} else {
		wr32(E1000_WUC, 0);
		wr32(E1000_WUFC, 0);
	}

	*enable_wake = wufc || adapter->en_mng_pt;
	if (!*enable_wake)
		igb_power_down_link(adapter);
	else
		igb_power_up_link(adapter);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	igb_release_hw_control(adapter);

	pci_disable_device(pdev);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
static int igb_suspend(struct device *dev)
{
	int retval;
	bool wake;
	struct pci_dev *pdev = to_pci_dev(dev);

	retval = __igb_shutdown(pdev, &wake, 0);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int igb_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	if (!pci_device_is_present(pdev))
		return -ENODEV;
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"igb: Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	if (igb_init_interrupt_scheme(adapter, true)) {
		dev_err(&pdev->dev, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	igb_reset(adapter);

	/* let the f/w know that the h/w is now under the control of the
	 * driver.
	 */
	igb_get_hw_control(adapter);

	wr32(E1000_WUS, ~0);

	if (netdev->flags & IFF_UP) {
		rtnl_lock();
		err = __igb_open(netdev, true);
		rtnl_unlock();
		if (err)
			return err;
	}

	rtnetif_device_attach(netdev);
	return 0;
}

static int igb_runtime_idle(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);

	if (!igb_has_link(adapter))
		pm_schedule_suspend(dev, MSEC_PER_SEC * 5);

	return -EBUSY;
}

static int igb_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int retval;
	bool wake;

	retval = __igb_shutdown(pdev, &wake, 1);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}

static int igb_runtime_resume(struct device *dev)
{
	return igb_resume(dev);
}
#endif /* CONFIG_PM */

static void igb_shutdown(struct pci_dev *pdev)
{
	bool wake;

	__igb_shutdown(pdev, &wake, 0);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}

static int igb_pci_sriov_configure(struct pci_dev *dev, int num_vfs)
{
	return 0;
}

/**
 *  igb_io_error_detected - called when PCI error is detected
 *  @pdev: Pointer to PCI device
 *  @state: The current pci connection state
 *
 *  This function is called after a PCI bus error affecting
 *  this device has been detected.
 **/
static pci_ers_result_t igb_io_error_detected(struct pci_dev *pdev,
					      pci_channel_state_t state)
{
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);

	rtnetif_device_detach(netdev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (rtnetif_running(netdev))
		igb_down(adapter);
	pci_disable_device(pdev);

	/* Request a slot slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 *  igb_io_slot_reset - called after the pci bus has been reset.
 *  @pdev: Pointer to PCI device
 *
 *  Restart the card from scratch, as if from a cold-boot. Implementation
 *  resembles the first-half of the igb_resume routine.
 **/
static pci_ers_result_t igb_io_slot_reset(struct pci_dev *pdev)
{
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	pci_ers_result_t result;
	int err;

	if (pci_enable_device_mem(pdev)) {
		dev_err(&pdev->dev,
			"Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pci_set_master(pdev);
		pci_restore_state(pdev);
		pci_save_state(pdev);

		pci_enable_wake(pdev, PCI_D3hot, 0);
		pci_enable_wake(pdev, PCI_D3cold, 0);

		igb_reset(adapter);
		wr32(E1000_WUS, ~0);
		result = PCI_ERS_RESULT_RECOVERED;
	}

	err = pci_cleanup_aer_uncorrect_error_status(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"pci_cleanup_aer_uncorrect_error_status failed 0x%0x\n",
			err);
		/* non-fatal, continue */
	}

	return result;
}

/**
 *  igb_io_resume - called when traffic can start flowing again.
 *  @pdev: Pointer to PCI device
 *
 *  This callback is called when the error recovery driver tells us that
 *  its OK to resume normal operation. Implementation resembles the
 *  second-half of the igb_resume routine.
 */
static void igb_io_resume(struct pci_dev *pdev)
{
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct igb_adapter *adapter = rtnetdev_priv(netdev);

	if (rtnetif_running(netdev)) {
		if (igb_up(adapter)) {
			dev_err(&pdev->dev, "igb_up failed after reset\n");
			return;
		}
	}

	rtnetif_device_attach(netdev);

	/* let the f/w know that the h/w is now under the control of the
	 * driver.
	 */
	igb_get_hw_control(adapter);
}

static void igb_rar_set_qsel(struct igb_adapter *adapter, u8 *addr, u32 index,
			     u8 qsel)
{
	u32 rar_low, rar_high;
	struct e1000_hw *hw = &adapter->hw;

	/* HW expects these in little endian so we reverse the byte order
	 * from network order (big endian) to little endian
	 */
	rar_low = ((u32) addr[0] | ((u32) addr[1] << 8) |
		   ((u32) addr[2] << 16) | ((u32) addr[3] << 24));
	rar_high = ((u32) addr[4] | ((u32) addr[5] << 8));

	/* Indicate to hardware the Address is Valid. */
	rar_high |= E1000_RAH_AV;

	if (hw->mac.type == e1000_82575)
		rar_high |= E1000_RAH_POOL_1 * qsel;
	else
		rar_high |= E1000_RAH_POOL_1 << qsel;

	wr32(E1000_RAL(index), rar_low);
	wrfl();
	wr32(E1000_RAH(index), rar_high);
	wrfl();
}

static void igb_init_dmac(struct igb_adapter *adapter, u32 pba)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 dmac_thr;
	u16 hwm;

	if (hw->mac.type > e1000_82580) {
		if (adapter->flags & IGB_FLAG_DMAC) {
			u32 reg;

			/* force threshold to 0. */
			wr32(E1000_DMCTXTH, 0);

			/* DMA Coalescing high water mark needs to be greater
			 * than the Rx threshold. Set hwm to PBA - max frame
			 * size in 16B units, capping it at PBA - 6KB.
			 */
			hwm = 64 * pba - adapter->max_frame_size / 16;
			if (hwm < 64 * (pba - 6))
				hwm = 64 * (pba - 6);
			reg = rd32(E1000_FCRTC);
			reg &= ~E1000_FCRTC_RTH_COAL_MASK;
			reg |= ((hwm << E1000_FCRTC_RTH_COAL_SHIFT)
				& E1000_FCRTC_RTH_COAL_MASK);
			wr32(E1000_FCRTC, reg);

			/* Set the DMA Coalescing Rx threshold to PBA - 2 * max
			 * frame size, capping it at PBA - 10KB.
			 */
			dmac_thr = pba - adapter->max_frame_size / 512;
			if (dmac_thr < pba - 10)
				dmac_thr = pba - 10;
			reg = rd32(E1000_DMACR);
			reg &= ~E1000_DMACR_DMACTHR_MASK;
			reg |= ((dmac_thr << E1000_DMACR_DMACTHR_SHIFT)
				& E1000_DMACR_DMACTHR_MASK);

			/* transition to L0x or L1 if available..*/
			reg |= (E1000_DMACR_DMAC_EN | E1000_DMACR_DMAC_LX_MASK);

			/* watchdog timer= +-1000 usec in 32usec intervals */
			reg |= (1000 >> 5);

			/* Disable BMC-to-OS Watchdog Enable */
			if (hw->mac.type != e1000_i354)
				reg &= ~E1000_DMACR_DC_BMC2OSW_EN;

			wr32(E1000_DMACR, reg);

			/* no lower threshold to disable
			 * coalescing(smart fifb)-UTRESH=0
			 */
			wr32(E1000_DMCRTRH, 0);

			reg = (IGB_DMCTLX_DCFLUSH_DIS | 0x4);

			wr32(E1000_DMCTLX, reg);

			/* free space in tx packet buffer to wake from
			 * DMA coal
			 */
			wr32(E1000_DMCTXTH, (IGB_MIN_TXPBSIZE -
			     (IGB_TX_BUF_4096 + adapter->max_frame_size)) >> 6);

			/* make low power state decision controlled
			 * by DMA coal
			 */
			reg = rd32(E1000_PCIEMISC);
			reg &= ~E1000_PCIEMISC_LX_DECISION;
			wr32(E1000_PCIEMISC, reg);
		} /* endif adapter->dmac is not disabled */
	} else if (hw->mac.type == e1000_82580) {
		u32 reg = rd32(E1000_PCIEMISC);

		wr32(E1000_PCIEMISC, reg & ~E1000_PCIEMISC_LX_DECISION);
		wr32(E1000_DMACR, 0);
	}
}

/**
 *  igb_read_i2c_byte - Reads 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to read
 *  @dev_addr: device address
 *  @data: value read
 *
 *  Performs byte read operation over I2C interface at
 *  a specified device address.
 **/
s32 igb_read_i2c_byte(struct e1000_hw *hw, u8 byte_offset,
		      u8 dev_addr, u8 *data)
{
	struct igb_adapter *adapter = container_of(hw, struct igb_adapter, hw);
	struct i2c_client *this_client = adapter->i2c_client;
	s32 status;
	u16 swfw_mask = 0;

	if (!this_client)
		return E1000_ERR_I2C;

	swfw_mask = E1000_SWFW_PHY0_SM;

	if (hw->mac.ops.acquire_swfw_sync(hw, swfw_mask))
		return E1000_ERR_SWFW_SYNC;

	status = i2c_smbus_read_byte_data(this_client, byte_offset);
	hw->mac.ops.release_swfw_sync(hw, swfw_mask);

	if (status < 0)
		return E1000_ERR_I2C;
	else {
		*data = status;
		return 0;
	}
}

/**
 *  igb_write_i2c_byte - Writes 8 bit word over I2C
 *  @hw: pointer to hardware structure
 *  @byte_offset: byte offset to write
 *  @dev_addr: device address
 *  @data: value to write
 *
 *  Performs byte write operation over I2C interface at
 *  a specified device address.
 **/
s32 igb_write_i2c_byte(struct e1000_hw *hw, u8 byte_offset,
		       u8 dev_addr, u8 data)
{
	struct igb_adapter *adapter = container_of(hw, struct igb_adapter, hw);
	struct i2c_client *this_client = adapter->i2c_client;
	s32 status;
	u16 swfw_mask = E1000_SWFW_PHY0_SM;

	if (!this_client)
		return E1000_ERR_I2C;

	if (hw->mac.ops.acquire_swfw_sync(hw, swfw_mask))
		return E1000_ERR_SWFW_SYNC;
	status = i2c_smbus_write_byte_data(this_client, byte_offset, data);
	hw->mac.ops.release_swfw_sync(hw, swfw_mask);

	if (status)
		return E1000_ERR_I2C;
	else
		return 0;

}

int igb_reinit_queues(struct igb_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	int err = 0;

	if (rtnetif_running(netdev))
		igb_close(netdev);

	igb_reset_interrupt_capability(adapter);

	if (igb_init_interrupt_scheme(adapter, true)) {
		dev_err(&pdev->dev, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	if (rtnetif_running(netdev))
		err = igb_open(netdev);

	return err;
}
/* igb_main.c */
