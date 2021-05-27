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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/netdevice.h>
#include <linux/tcp.h>
#include <linux/ipv6.h>


// RTNET defines...
#ifdef NETIF_F_TSO
#undef NETIF_F_TSO
#endif

#ifdef NETIF_F_TSO6
#undef NETIF_F_TSO6
#endif

#ifdef NETIF_F_HW_VLAN_TX
#undef NETIF_F_HW_VLAN_TX
#endif

#ifdef CONFIG_E1000_NAPI
#undef CONFIG_E1000_NAPI
#endif

#ifdef MAX_SKB_FRAGS
#undef MAX_SKB_FRAGS
#endif

#ifndef CONFIG_E1000_DISABLE_PACKET_SPLIT
#define CONFIG_E1000_DISABLE_PACKET_SPLIT
#endif

#ifdef CONFIG_E1000_MQ
#undef CONFIG_E1000_MQ
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
#undef CONFIG_NET_POLL_CONTROLLER
#endif

#ifdef CONFIG_PM
#undef CONFIG_PM
#endif

#ifdef HAVE_PCI_ERS
#error "STOP it here"
#undef HAVE_PCI_ERS
#endif

#ifdef USE_REBOOT_NOTIFIER
#undef USE_REBOOT_NOTIFIER
#endif

#ifdef HAVE_TX_TIMEOUT
#undef HAVE_TX_TIMEOUT
#endif


#ifdef NETIF_F_TSO
#include <net/checksum.h>
#ifdef NETIF_F_TSO6
#include <net/ip6_checksum.h>
#endif
#endif
#ifdef SIOCGMIIPHY
#include <linux/mii.h>
#endif
#ifdef SIOCETHTOOL
#include <linux/ethtool.h>
#endif
#ifdef NETIF_F_HW_VLAN_TX
#include <linux/if_vlan.h>
#endif
#ifdef CONFIG_E1000_MQ
#include <linux/cpu.h>
#include <linux/smp.h>
#endif

#include "e1000.h"

#ifdef HAVE_PCI_ERS
#error "STOP it here"
#endif



char e1000_driver_name[MODULE_NAME_LEN] = "rt_e1000";
static char e1000_driver_string[] = "Intel(R) PRO/1000 Network Driver";

#ifdef CONFIG_E1000_NAPI
#define DRV_NAPI "-NAPI"
#else
#define DRV_NAPI
#endif


#define DRV_DEBUG

#define DRV_HW_PERF

/*
 * Port to rtnet based on e1000 driver version 7.6.15.5 (22-Sep-2008 Mathias Koehrer)
 *
 * */

#define DRV_VERSION "7.6.15.5" DRV_NAPI DRV_DEBUG DRV_HW_PERF " ported to RTnet"
const char e1000_driver_version[] = DRV_VERSION;
static const char e1000_copyright[] = "Copyright (c) 1999-2008 Intel Corporation.";

// RTNET wrappers
#define kmalloc(a,b) rtdm_malloc(a)
#define vmalloc(a) rtdm_malloc(a)
#define kfree(a) rtdm_free(a)
#define vfree(a) rtdm_free(a)
#define skb_reserve(a,b) rtskb_reserve(a,b)
#define net_device rtnet_device
#define sk_buff rtskb
#define netdev_priv(a) a->priv
// ----------------------



/* e1000_pci_tbl - PCI Device ID Table
 *
 * Last entry must be all 0s
 *
 * Macro expands to...
 *   {PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}
 */

#define PCI_ID_LIST_PCI  \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82542), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82543GC_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82543GC_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82544EI_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82544EI_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82544GC_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82544GC_LOM), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82540EM), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82545EM_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546EB_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82545EM_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546EB_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541EI), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541ER_LOM), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82540EM_LOM), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82540EP_LOM), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82540EP), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541EI_MOBILE), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82547EI), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82547EI_MOBILE), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546EB_QUAD_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82540EP_LP), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82545GM_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82545GM_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82545GM_SERDES), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82547GI), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541GI), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541GI_MOBILE), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541ER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_FIBER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_SERDES), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82541GI_LF), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_PCIE), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_QUAD_COPPER), \
	  INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3)

#define PCI_ID_LIST_PCIE  \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IGP_M_AMT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IGP_AMT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IGP_C), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IFE), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IGP_M), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_COPPER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_FIBER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_SERDES), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82572EI_COPPER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82572EI_FIBER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82572EI_SERDES), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82573E), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82573E_IAMT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_80003ES2LAN_COPPER_DPT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_80003ES2LAN_SERDES_DPT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82573L), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_QUAD_COPPER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_QUAD_FIBER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_SERDES_DUAL), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_SERDES_QUAD), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82572EI), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_80003ES2LAN_COPPER_SPT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_80003ES2LAN_SERDES_SPT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571EB_QUAD_COPPER_LP), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_82571PT_QUAD_COPPER), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IFE_GT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH8_IFE_G), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH9_IGP_AMT), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH9_IGP_C), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH9_IFE), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH9_IFE_G), \
	 INTEL_E1000_ETHERNET_DEVICE(E1000_DEV_ID_ICH9_IFE_GT)




static struct pci_device_id e1000_pci_tbl[] = {
    PCI_ID_LIST_PCI,
    PCI_ID_LIST_PCIE,
	/* required last entry */
	{0,}
};
MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);

static struct pci_device_id e1000_pcipure_tbl[] = {
    PCI_ID_LIST_PCI,
	/* required last entry */
	{0,}
};

static struct pci_device_id e1000_pcie_tbl[] = {
    PCI_ID_LIST_PCIE,
	/* required last entry */
	{0,}
};



static int e1000_setup_tx_resources(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring);
static int e1000_setup_rx_resources(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring);
static void e1000_free_tx_resources(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring);
static void e1000_free_rx_resources(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring);

static int e1000_init_module(void);
static void e1000_exit_module(void);
static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void e1000_remove(struct pci_dev *pdev);
static int e1000_alloc_queues(struct e1000_adapter *adapter);
#ifdef CONFIG_E1000_MQ
static void e1000_setup_queue_mapping(struct e1000_adapter *adapter);
#endif
static int e1000_sw_init(struct e1000_adapter *adapter);
static int e1000_open(struct net_device *netdev);
static int e1000_close(struct net_device *netdev);
static void e1000_configure(struct e1000_adapter *adapter);
static void e1000_configure_tx(struct e1000_adapter *adapter);
static void e1000_configure_rx(struct e1000_adapter *adapter);
static void e1000_setup_rctl(struct e1000_adapter *adapter);
static void e1000_clean_all_tx_rings(struct e1000_adapter *adapter);
static void e1000_clean_all_rx_rings(struct e1000_adapter *adapter);
static void e1000_clean_tx_ring(struct e1000_adapter *adapter,
				struct e1000_tx_ring *tx_ring);
static void e1000_clean_rx_ring(struct e1000_adapter *adapter,
				struct e1000_rx_ring *rx_ring);
static void e1000_set_multi(struct net_device *netdev);
static void e1000_update_phy_info_task(struct work_struct *work);
static void e1000_watchdog_task(struct work_struct *work);
static void e1000_82547_tx_fifo_stall_task(struct work_struct *work);
static int e1000_xmit_frame_ring(struct sk_buff *skb, struct net_device *netdev,
				 struct e1000_tx_ring *tx_ring);
static int e1000_xmit_frame(struct sk_buff *skb, struct net_device *netdev);
#ifdef CONFIG_E1000_MQ
static int e1000_subqueue_xmit_frame(struct sk_buff *skb,
				     struct net_device *netdev, int queue);
#endif
static void e1000_phy_read_status(struct e1000_adapter *adapter);
#if 0
static struct net_device_stats * e1000_get_stats(struct net_device *netdev);
static int e1000_change_mtu(struct net_device *netdev, int new_mtu);
static int e1000_set_mac(struct net_device *netdev, void *p);
#endif
static int  e1000_intr(rtdm_irq_t *irq_handle);
static int e1000_intr_msi(rtdm_irq_t *irq_handle);
static bool e1000_clean_tx_irq(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring);
#ifdef CONFIG_E1000_NAPI
static int e1000_poll(struct napi_struct *napi, int budget);
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring,
				    int *work_done, int work_to_do);
static bool e1000_clean_rx_irq_ps(struct e1000_adapter *adapter,
				       struct e1000_rx_ring *rx_ring,
				       int *work_done, int work_to_do);
static bool e1000_clean_jumbo_rx_irq(struct e1000_adapter *adapter,
					  struct e1000_rx_ring *rx_ring,
					  int *work_done, int work_to_do);
static void e1000_alloc_jumbo_rx_buffers(struct e1000_adapter *adapter,
					 struct e1000_rx_ring *rx_ring,
					 int cleaned_count);
#else
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring,
					 nanosecs_abs_t *time_stamp);
static bool e1000_clean_rx_irq_ps(struct e1000_adapter *adapter,
				       struct e1000_rx_ring *rx_ring,
					 nanosecs_abs_t *time_stamp);
#endif
static void e1000_alloc_rx_buffers(struct e1000_adapter *adapter,
				   struct e1000_rx_ring *rx_ring,
				   int cleaned_count);
static void e1000_alloc_rx_buffers_ps(struct e1000_adapter *adapter,
				      struct e1000_rx_ring *rx_ring,
				      int cleaned_count);
#if 0
static int e1000_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
#ifdef SIOCGMIIPHY
static int e1000_mii_ioctl(struct net_device *netdev, struct ifreq *ifr,
			   int cmd);
static void e1000_enter_82542_rst(struct e1000_adapter *adapter);
static void e1000_leave_82542_rst(struct e1000_adapter *adapter);
static void e1000_tx_timeout(struct net_device *dev);
#endif
#endif
static void e1000_reset_task(struct work_struct *work);
static void e1000_smartspeed(struct e1000_adapter *adapter);
static int e1000_82547_fifo_workaround(struct e1000_adapter *adapter,
				       struct sk_buff *skb);

#ifdef NETIF_F_HW_VLAN_TX
static void e1000_vlan_rx_register(struct net_device *netdev,
				   struct vlan_group *grp);
static void e1000_vlan_rx_add_vid(struct net_device *netdev, u16 vid);
static void e1000_vlan_rx_kill_vid(struct net_device *netdev, u16 vid);
static void e1000_restore_vlan(struct e1000_adapter *adapter);
#endif

// static int e1000_suspend(struct pci_dev *pdev, pm_message_t state);
#ifdef CONFIG_PM
static int e1000_resume(struct pci_dev *pdev);
#endif
#ifndef USE_REBOOT_NOTIFIER
// static void e1000_shutdown(struct pci_dev *pdev);
#else
static int e1000_notify_reboot(struct notifier_block *, unsigned long event,
			       void *ptr);
static struct notifier_block e1000_notifier_reboot = {
	.notifier_call	= e1000_notify_reboot,
	.next		= NULL,
	.priority	= 0
};
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
/* for netdump / net console */
static void e1000_netpoll (struct net_device *netdev);
#endif

#define COPYBREAK_DEFAULT 256
static unsigned int copybreak __read_mostly = COPYBREAK_DEFAULT;
module_param(copybreak, uint, 0644);
MODULE_PARM_DESC(copybreak,
	"Maximum size of packet that is copied to a new buffer on receive");


#ifdef HAVE_PCI_ERS
static pci_ers_result_t e1000_io_error_detected(struct pci_dev *pdev,
		     pci_channel_state_t state);
static pci_ers_result_t e1000_io_slot_reset(struct pci_dev *pdev);
static void e1000_io_resume(struct pci_dev *pdev);

static struct pci_error_handlers e1000_err_handler = {
	.error_detected = e1000_io_error_detected,
	.slot_reset = e1000_io_slot_reset,
	.resume = e1000_io_resume,
};
#endif

static struct pci_driver e1000_driver = {
	.name     = e1000_driver_name,
	.id_table = e1000_pci_tbl,
	.probe    = e1000_probe,
	.remove   = e1000_remove,
#ifdef HAVE_PCI_ERS
	.err_handler = &e1000_err_handler
#endif
};

MODULE_AUTHOR("Intel Corporation, <linux.nics@intel.com>");
MODULE_DESCRIPTION("Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

#define MAX_UNITS 8
static int cards[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = 1 };
module_param_array(cards, int, NULL, 0444);
MODULE_PARM_DESC(cards, "array of cards to be supported (eg. 1,0,1)");


static int local_debug = NETIF_MSG_DRV | NETIF_MSG_PROBE;
module_param(local_debug, int, 0);
MODULE_PARM_DESC(local_debug, "Debug level (0=none,...,16=all)");

/* The parameter 'pciif' might be used to use this driver for
 * PCI or PCIe only NICs.
 * This allows to reflect the situation that newer Linux kernels
 * have two different (non real time) drivers for the e1000:
 * e1000 for PCI only
 * e1000e for PCIe only
 *
 * Using the 'pciif' parameter allows to load the driver
 *  modprobe rt_e1000 pciif=pci
 * to use it as PCI only
 * and a
 *  modprobe rt_e1000 -o rt_e1000e pciif=pcie
 * allows to load a second instance of this driver named 'rt_e1000e'
 *
 * If the 'pciif' paramter is not specified, all (PCI and PCIe) e1000
 * NICs will be used.
 * */
static char *pciif = "all";
module_param(pciif, charp, 0);
MODULE_PARM_DESC(pciif, "PCI Interface: 'all' (default), 'pci', 'pcie'");


//#define register_netdev(a) rt_register_rtnetdev(a)
//#define unregister_netdev(a) rt_unregister_rtnetdev(a)
//#define free_netdev(a) rtdev_free(a)
//#define netif_stop_queue(a) rtnetif_stop_queue(a)

/**
 * e1000_init_module - Driver Registration Routine
 *
 * e1000_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/
static int __init e1000_init_module(void)
{
	int ret;
    strcpy(e1000_driver_name, THIS_MODULE->name);
	printk(KERN_INFO "%s - %s version %s (pciif: %s)\n",
	       e1000_driver_string, e1000_driver_name, e1000_driver_version, pciif);

	printk(KERN_INFO "%s\n", e1000_copyright);


    if (0 == strcmp(pciif, "pcie"))
    {
	// PCIe only
	    e1000_driver.id_table = e1000_pcie_tbl;
    }
    else if (0 == strcmp(pciif, "pci"))
    {
	// PCI only
	    e1000_driver.id_table = e1000_pcipure_tbl;
    }

	ret = pci_register_driver(&e1000_driver);
#ifdef USE_REBOOT_NOTIFIER
	if (ret >= 0) {
		register_reboot_notifier(&e1000_notifier_reboot);
	}
#endif
	if (copybreak != COPYBREAK_DEFAULT) {
		if (copybreak == 0)
			printk(KERN_INFO "e1000: copybreak disabled\n");
		else
			printk(KERN_INFO "e1000: copybreak enabled for "
			       "packets <= %u bytes\n", copybreak);
	}
	return ret;
}

module_init(e1000_init_module);

/**
 * e1000_exit_module - Driver Exit Cleanup Routine
 *
 * e1000_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit e1000_exit_module(void)
{
#ifdef USE_REBOOT_NOTIFIER
	unregister_reboot_notifier(&e1000_notifier_reboot);
#endif
	pci_unregister_driver(&e1000_driver);
}

module_exit(e1000_exit_module);

static int e1000_request_irq(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err = 0;

	if (adapter->flags & E1000_FLAG_HAS_MSI) {
		err = pci_enable_msi(adapter->pdev);
		if (!err)
			adapter->flags |= E1000_FLAG_MSI_ENABLED;
	}
    rt_stack_connect(netdev, &STACK_manager);
	if (adapter->flags & E1000_FLAG_MSI_ENABLED) {
		err = rtdm_irq_request(&adapter->irq_handle, adapter->pdev->irq, e1000_intr_msi,
				  0, netdev->name, netdev);
		if (!err) {
			return err;
		} else {
			adapter->flags &= ~E1000_FLAG_MSI_ENABLED;
			pci_disable_msi(adapter->pdev);
		}
	}
	err = rtdm_irq_request(&adapter->irq_handle, adapter->pdev->irq,
			       e1000_intr, RTDM_IRQTYPE_SHARED, netdev->name,
			       netdev);
	if (err)
		DPRINTK(PROBE, ERR, "Unable to allocate interrupt Error: %d\n",
			err);

	return err;
}

static void e1000_free_irq(struct e1000_adapter *adapter)
{
	// struct net_device *netdev = adapter->netdev;

	rtdm_irq_free(&adapter->irq_handle);

	if (adapter->flags & E1000_FLAG_MSI_ENABLED) {
		pci_disable_msi(adapter->pdev);
		adapter->flags &= ~E1000_FLAG_MSI_ENABLED;
	}
}

/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static void e1000_irq_disable(struct e1000_adapter *adapter)
{
	atomic_inc(&adapter->irq_sem);
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, ~0);
	E1000_WRITE_FLUSH(&adapter->hw);
	synchronize_irq(adapter->pdev->irq);
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

static void e1000_irq_enable(struct e1000_adapter *adapter)
{
	if (likely(atomic_dec_and_test(&adapter->irq_sem))) {
		E1000_WRITE_REG(&adapter->hw, E1000_IMS, IMS_ENABLE_MASK);
		E1000_WRITE_FLUSH(&adapter->hw);
	}
}
#ifdef NETIF_F_HW_VLAN_TX

static void e1000_update_mng_vlan(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	u16 vid = adapter->hw.mng_cookie.vlan_id;
	u16 old_vid = adapter->mng_vlan_id;
	if (adapter->vlgrp) {
		if (!vlan_group_get_device(adapter->vlgrp, vid)) {
			if (adapter->hw.mng_cookie.status &
				E1000_MNG_DHCP_COOKIE_STATUS_VLAN) {
				e1000_vlan_rx_add_vid(netdev, vid);
				adapter->mng_vlan_id = vid;
			} else {
				adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
			}

			if ((old_vid != (u16)E1000_MNG_VLAN_NONE) &&
					(vid != old_vid) &&
			    !vlan_group_get_device(adapter->vlgrp, old_vid))
				e1000_vlan_rx_kill_vid(netdev, old_vid);
		} else {
			adapter->mng_vlan_id = vid;
		}
	}
}
#endif

/**
 * e1000_release_hw_control - release control of the h/w to f/w
 * @adapter: address of board private structure
 *
 * e1000_release_hw_control resets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded. For AMT version (only with 82573) i
 * of the f/w this means that the network i/f is closed.
 *
 **/
static void e1000_release_hw_control(struct e1000_adapter *adapter)
{
	u32 ctrl_ext;
	u32 swsm;

	/* Let firmware taken over control of h/w */
	switch (adapter->hw.mac.type) {
	case e1000_82573:
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
				swsm & ~E1000_SWSM_DRV_LOAD);
		break;
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
	case e1000_ich8lan:
	case e1000_ich9lan:
		ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
				ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
		break;
	default:
		break;
	}
}

/**
 * e1000_get_hw_control - get control of the h/w from f/w
 * @adapter: address of board private structure
 *
 * e1000_get_hw_control sets {CTRL_EXT|SWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded. For AMT version (only with 82573)
 * of the f/w this means that the network i/f is open.
 *
 **/
static void e1000_get_hw_control(struct e1000_adapter *adapter)
{
	u32 ctrl_ext;
	u32 swsm;

	/* Let firmware know the driver has taken over */
	switch (adapter->hw.mac.type) {
	case e1000_82573:
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
				swsm | E1000_SWSM_DRV_LOAD);
		break;
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
	case e1000_ich8lan:
	case e1000_ich9lan:
		ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
				ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
		break;
	default:
		break;
	}
}

static void e1000_init_manageability(struct e1000_adapter *adapter)
{
}

static void e1000_release_manageability(struct e1000_adapter *adapter)
{
}

/**
 * e1000_configure - configure the hardware for RX and TX
 * @adapter: private board structure
 **/
static void e1000_configure(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int i;

	e1000_set_multi(netdev);

#ifdef NETIF_F_HW_VLAN_TX
	e1000_restore_vlan(adapter);
#endif
	e1000_init_manageability(adapter);

	e1000_configure_tx(adapter);
	e1000_setup_rctl(adapter);
	e1000_configure_rx(adapter);
	/* call E1000_DESC_UNUSED which always leaves
	 * at least 1 descriptor unused to make sure
	 * next_to_use != next_to_clean */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct e1000_rx_ring *ring = &adapter->rx_ring[i];
		adapter->alloc_rx_buf(adapter, ring,
				      E1000_DESC_UNUSED(ring));
	}

#ifdef CONFIG_E1000_MQ
	e1000_setup_queue_mapping(adapter);
#endif

	// adapter->tx_queue_len = netdev->tx_queue_len;
}

static void e1000_napi_enable_all(struct e1000_adapter *adapter)
{
#ifdef CONFIG_E1000_NAPI
	int i;
	for (i = 0; i < adapter->num_rx_queues; i++)
		napi_enable(&adapter->rx_ring[i].napi);
#endif
}

static void e1000_napi_disable_all(struct e1000_adapter *adapter)
{
#ifdef CONFIG_E1000_NAPI
	int i;
	for (i = 0; i < adapter->num_rx_queues; i++)
		napi_disable(&adapter->rx_ring[i].napi);
#endif
}

int e1000_up(struct e1000_adapter *adapter)
{
	/* hardware has been reset, we need to reload some things */
	e1000_configure(adapter);

	clear_bit(__E1000_DOWN, &adapter->state);

	e1000_napi_enable_all(adapter);

	e1000_irq_enable(adapter);

	/* fire a link change interrupt to start the watchdog */
	// E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_LSC);
	return 0;
}

static void e1000_down_and_stop(struct e1000_adapter *adapter)
{
	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer */
	set_bit(__E1000_DOWN, &adapter->state);

	cancel_work_sync(&adapter->reset_task);
	cancel_delayed_work_sync(&adapter->watchdog_task);
	cancel_delayed_work_sync(&adapter->phy_info_task);
	cancel_delayed_work_sync(&adapter->fifo_stall_task);
}

void e1000_down(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	u32 tctl, rctl;

	e1000_down_and_stop(adapter);

	/* disable receives in the hardware */
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */

#ifdef NETIF_F_LLTX
	rtnetif_stop_queue(netdev);
#else
	rtnetif_tx_disable(netdev);
#endif

	/* disable transmits in the hardware */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);
	/* flush both disables and wait for them to finish */
	E1000_WRITE_FLUSH(&adapter->hw);
	msleep(10);

	e1000_napi_disable_all(adapter);

	e1000_irq_disable(adapter);

	// netdev->tx_queue_len = adapter->tx_queue_len;
	rtnetif_carrier_off(netdev);
	adapter->link_speed = 0;
	adapter->link_duplex = 0;

	e1000_reset(adapter);
	e1000_clean_all_tx_rings(adapter);
	e1000_clean_all_rx_rings(adapter);
}

void e1000_reinit_locked(struct e1000_adapter *adapter)
{
	WARN_ON(in_interrupt());
	while (test_and_set_bit(__E1000_RESETTING, &adapter->state))
		msleep(1);
	e1000_down(adapter);
	e1000_up(adapter);
	clear_bit(__E1000_RESETTING, &adapter->state);
}

void e1000_reset(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_fc_info *fc = &adapter->hw.fc;
	u32 pba = 0, tx_space, min_tx_space, min_rx_space;
	bool legacy_pba_adjust = FALSE;
	u16 hwm;

	/* Repartition Pba for greater than 9k mtu
	 * To take effect CTRL.RST is required.
	 */

	switch (mac->type) {
	case e1000_82542:
	case e1000_82543:
	case e1000_82544:
	case e1000_82540:
	case e1000_82541:
	case e1000_82541_rev_2:
		legacy_pba_adjust = TRUE;
		pba = E1000_PBA_48K;
		break;
	case e1000_82545:
	case e1000_82545_rev_3:
	case e1000_82546:
	case e1000_82546_rev_3:
		pba = E1000_PBA_48K;
		break;
	case e1000_82547:
	case e1000_82547_rev_2:
		legacy_pba_adjust = TRUE;
		pba = E1000_PBA_30K;
		break;
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		pba = E1000_PBA_38K;
		break;
	case e1000_82573:
		pba = E1000_PBA_20K;
		break;
	case e1000_ich8lan:
		pba = E1000_PBA_8K;
		break;
	case e1000_ich9lan:
#define E1000_PBA_10K 0x000A
		pba = E1000_PBA_10K;
		break;
	case e1000_undefined:
	case e1000_num_macs:
		break;
	}

	if (legacy_pba_adjust == TRUE) {
		if (adapter->max_frame_size > E1000_RXBUFFER_8192)
			pba -= 8; /* allocate more FIFO for Tx */

		if (mac->type == e1000_82547) {
			adapter->tx_fifo_head = 0;
			adapter->tx_head_addr = pba << E1000_TX_HEAD_ADDR_SHIFT;
			adapter->tx_fifo_size =
				(E1000_PBA_40K - pba) << E1000_PBA_BYTES_SHIFT;
			atomic_set(&adapter->tx_fifo_stall, 0);
		}
	} else if (adapter->max_frame_size > ETH_FRAME_LEN + ETHERNET_FCS_SIZE) {
		/* adjust PBA for jumbo frames */
		E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);

		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB. */
		pba = E1000_READ_REG(&adapter->hw, E1000_PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the tx fifo also stores 16 bytes of information about the tx
		 * but don't include ethernet FCS because hardware appends it */
		min_tx_space = (adapter->max_frame_size +
				sizeof(struct e1000_tx_desc) -
				ETHERNET_FCS_SIZE) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;

		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation */
		if (tx_space < min_tx_space &&
		    ((min_tx_space - tx_space) < pba)) {
			pba = pba - (min_tx_space - tx_space);

			/* PCI/PCIx hardware has PBA alignment constraints */
			switch (mac->type) {
			case e1000_82545 ... e1000_82546_rev_3:
				pba &= ~(E1000_PBA_8K - 1);
				break;
			default:
				break;
			}

			/* if short on rx space, rx wins and must trump tx
			 * adjustment or use Early Receive if available */
			if (pba < min_rx_space) {
				switch (mac->type) {
				case e1000_82573:
				case e1000_ich9lan:
					/* ERT enabled in e1000_configure_rx */
					break;
				default:
					pba = min_rx_space;
					break;
				}
			}
		}
	}

	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);

	/* flow control settings */
	/* The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus the early receive size (for parts
	 *   with ERT support assuming ERT set to E1000_ERT_2048), or
	 * - the full Rx FIFO size minus one full frame */
	hwm = min(((pba << 10) * 9 / 10),
		  ((mac->type == e1000_82573 || mac->type == e1000_ich9lan) ?
		      (u16)((pba << 10) - (E1000_ERT_2048 << 3)) :
		      ((pba << 10) - adapter->max_frame_size)));

	fc->high_water = hwm & 0xFFF8;	/* 8-byte granularity */
	fc->low_water = fc->high_water - 8;

	if (mac->type == e1000_80003es2lan)
		fc->pause_time = 0xFFFF;
	else
		fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = 1;
	fc->type = fc->original_type;

	/* Allow time for pending master requests to run */
	e1000_reset_hw(&adapter->hw);

	/* For 82573 and ICHx if AMT is enabled, let the firmware know
	 * that the network interface is in control */
	if (((adapter->hw.mac.type == e1000_82573) ||
	     (adapter->hw.mac.type == e1000_ich8lan) ||
	     (adapter->hw.mac.type == e1000_ich9lan)) &&
	    e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	if (mac->type >= e1000_82544)
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, 0);

	if (e1000_init_hw(&adapter->hw))
		DPRINTK(PROBE, ERR, "Hardware Error\n");
#ifdef NETIF_F_HW_VLAN_TX
	e1000_update_mng_vlan(adapter);
#endif
	/* if (adapter->hwflags & HWFLAGS_PHY_PWR_BIT) { */
	if (mac->type >= e1000_82544 &&
	    mac->type <= e1000_82547_rev_2 &&
	    mac->autoneg == 1 &&
	    adapter->hw.phy.autoneg_advertised == ADVERTISE_1000_FULL) {
		u32 ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
		/* clear phy power management bit if we are in gig only mode,
		 * which if enabled will attempt negotiation to 100Mb, which
		 * can cause a loss of link at power off or driver unload */
		ctrl &= ~E1000_CTRL_SWDPIN3;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	}

#if defined(CONFIG_PPC64) || defined(CONFIG_PPC)
#define E1000_GCR_DISABLE_TIMEOUT_MECHANISM 0x80000000
	if (adapter->hw.mac.type == e1000_82571) {
		/* work around pSeries hardware by disabling timeouts */
		u32 gcr = E1000_READ_REG(&adapter->hw, E1000_GCR);
		gcr |= E1000_GCR_DISABLE_TIMEOUT_MECHANISM;
		E1000_WRITE_REG(&adapter->hw, E1000_GCR, gcr);
	}
#endif

	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERNET_IEEE_VLAN_TYPE);

	e1000_reset_adaptive(&adapter->hw);
	e1000_get_phy_info(&adapter->hw);

	if (!(adapter->flags & E1000_FLAG_SMART_POWER_DOWN) &&
	    (mac->type == e1000_82571 || mac->type == e1000_82572)) {
		u16 phy_data = 0;
		/* speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed */
		e1000_read_phy_reg(&adapter->hw, IGP02E1000_PHY_POWER_MGMT,
				   &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1000_write_phy_reg(&adapter->hw, IGP02E1000_PHY_POWER_MGMT,
				    phy_data);
	}

	e1000_release_manageability(adapter);
}

/**
 * e1000_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in e1000_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * e1000_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int e1000_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct e1000_adapter *adapter;

	static int cards_found = 0;
	static int global_quad_port_a = 0; /* global ksp3 port a indication */
	int i, err, pci_using_dac;
	u16 eeprom_data = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;

    if (cards[cards_found++] == 0)
    {
	return -ENODEV;
    }

	if ((err = pci_enable_device(pdev)))
		return err;

	if (!(err = pci_set_dma_mask(pdev, DMA_64BIT_MASK)) &&
	    !(err = pci_set_consistent_dma_mask(pdev, DMA_64BIT_MASK))) {
		pci_using_dac = 1;
	} else {
		if ((err = pci_set_dma_mask(pdev, DMA_32BIT_MASK)) &&
		    (err = pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK))) {
			E1000_ERR("No usable DMA configuration, aborting\n");
			goto err_dma;
		}
		pci_using_dac = 0;
	}

	if ((err = pci_request_regions(pdev, e1000_driver_name)))
		goto err_pci_reg;

	pci_set_master(pdev);

	err = -ENOMEM;
#ifdef CONFIG_E1000_MQ
	netdev = rt_alloc_etherdev(sizeof(struct e1000_adapter) +
							(sizeof(struct net_device_subqueue) *
								E1000_MAX_TX_QUEUES), 16);
#else
	netdev = rt_alloc_etherdev(sizeof(struct e1000_adapter),
				2 * E1000_DEFAULT_RXD + E1000_DEFAULT_TXD);
#endif
	if (!netdev)
		goto err_alloc_etherdev;

    memset(netdev->priv, 0, sizeof(struct e1000_adapter));
    rt_rtdev_connect(netdev, &RTDEV_manager);

	// SET_NETDEV_DEV(netdev, &pdev->dev);
    netdev->vers = RTDEV_VERS_2_0;

	pci_set_drvdata(pdev, netdev);
	adapter = netdev->priv;
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->hw.back = adapter;
	adapter->msg_enable = (1 << local_debug) - 1;

	err = -EIO;
	adapter->hw.hw_addr = ioremap(pci_resource_start(pdev, BAR_0),
				      pci_resource_len(pdev, BAR_0));
	if (!adapter->hw.hw_addr)
		goto err_ioremap;

	for (i = BAR_1; i <= BAR_5; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		if (pci_resource_flags(pdev, i) & IORESOURCE_IO) {
			adapter->hw.io_base = pci_resource_start(pdev, i);
			break;
		}
	}

	netdev->open = &e1000_open;
	netdev->stop = &e1000_close;
	netdev->hard_start_xmit = &e1000_xmit_frame;
#ifdef CONFIG_E1000_MQ
	netdev->hard_start_subqueue_xmit = &e1000_subqueue_xmit_frame;
#endif
#ifdef HAVE_TX_TIMEOUT
	netdev->tx_timeout = &e1000_tx_timeout;
	netdev->watchdog_timeo = 5 * HZ;
#endif
#ifdef NETIF_F_HW_VLAN_TX
	netdev->vlan_rx_register = e1000_vlan_rx_register;
	netdev->vlan_rx_add_vid = e1000_vlan_rx_add_vid;
	netdev->vlan_rx_kill_vid = e1000_vlan_rx_kill_vid;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	netdev->poll_controller = e1000_netpoll;
#endif
	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	adapter->bd_number = cards_found;

	/* setup the private structure */
	if ((err = e1000_sw_init(adapter)))
		goto err_sw_init;

	err = -EIO;
	/* Flash BAR mapping must happen after e1000_sw_init
	 * because it depends on mac.type */
	if (((adapter->hw.mac.type == e1000_ich8lan) ||
	     (adapter->hw.mac.type == e1000_ich9lan)) &&
	   (pci_resource_flags(pdev, 1) & IORESOURCE_MEM)) {
		adapter->hw.flash_address = ioremap(pci_resource_start(pdev, 1),
						    pci_resource_len(pdev, 1));
		if (!adapter->hw.flash_address)
			goto err_flashmap;
	}

	if ((err = e1000_init_mac_params(&adapter->hw)))
		goto err_hw_init;

	if ((err = e1000_init_nvm_params(&adapter->hw)))
		goto err_hw_init;

	if ((err = e1000_init_phy_params(&adapter->hw)))
		goto err_hw_init;

	e1000_get_bus_info(&adapter->hw);

	e1000_init_script_state_82541(&adapter->hw, TRUE);
	e1000_set_tbi_compatibility_82543(&adapter->hw, TRUE);

	adapter->hw.phy.autoneg_wait_to_complete = FALSE;
	adapter->hw.mac.adaptive_ifs = FALSE;

	/* Copper options */

	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		adapter->hw.phy.mdix = AUTO_ALL_MODES;
		adapter->hw.phy.disable_polarity_correction = FALSE;
		adapter->hw.phy.ms_type = E1000_MASTER_SLAVE;
	}

	if (e1000_check_reset_block(&adapter->hw))
		DPRINTK(PROBE, INFO, "PHY reset is blocked due to SOL/IDER session.\n");

#ifdef MAX_SKB_FRAGS
	if (adapter->hw.mac.type >= e1000_82543) {
#ifdef NETIF_F_HW_VLAN_TX
		netdev->features = NETIF_F_SG |
				   NETIF_F_HW_CSUM |
				   NETIF_F_HW_VLAN_TX |
				   NETIF_F_HW_VLAN_RX |
				   NETIF_F_HW_VLAN_FILTER;
		if ((adapter->hw.mac.type == e1000_ich8lan) ||
		    (adapter->hw.mac.type == e1000_ich9lan))
			netdev->features &= ~NETIF_F_HW_VLAN_FILTER;
#else
		netdev->features = NETIF_F_SG | NETIF_F_HW_CSUM;
#endif
	}

#ifdef NETIF_F_TSO
	if ((adapter->hw.mac.type >= e1000_82544) &&
	   (adapter->hw.mac.type != e1000_82547)) {
		adapter->flags |= E1000_FLAG_HAS_TSO;
		netdev->features |= NETIF_F_TSO;
	}

#ifdef NETIF_F_TSO6
	if (adapter->hw.mac.type > e1000_82547_rev_2) {
		adapter->flags |= E1000_FLAG_HAS_TSO6;
		netdev->features |= NETIF_F_TSO6;
	}
#endif
#endif
	if (pci_using_dac)
		netdev->features |= NETIF_F_HIGHDMA;

#endif
#ifdef NETIF_F_LLTX
	netdev->features |= NETIF_F_LLTX;
#endif

	/* Hardware features, flags and workarounds */
	if (adapter->hw.mac.type >= e1000_82571) {
		adapter->flags |= E1000_FLAG_INT_ASSERT_AUTO_MASK;
		adapter->flags |= E1000_FLAG_HAS_MSI;
		adapter->flags |= E1000_FLAG_HAS_MANC2H;
	}

	if (adapter->hw.mac.type >= e1000_82540) {
		adapter->flags |= E1000_FLAG_HAS_SMBUS;
		adapter->flags |= E1000_FLAG_HAS_INTR_MODERATION;
	}

	if (adapter->hw.mac.type == e1000_82543)
		adapter->flags |= E1000_FLAG_BAD_TX_CARRIER_STATS_FD;

	/* In rare occasions, ESB2 systems would end up started without
	 * the RX unit being turned on. */
	if (adapter->hw.mac.type == e1000_80003es2lan)
		adapter->flags |= E1000_FLAG_RX_NEEDS_RESTART;

	adapter->en_mng_pt = e1000_enable_mng_pass_thru(&adapter->hw);

	/* before reading the NVM, reset the controller to
	 * put the device in a known good starting state */

	e1000_reset_hw(&adapter->hw);

	/* make sure we don't intercept ARP packets until we're up */
	e1000_release_manageability(adapter);

	/* make sure the NVM is good */

	if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
		DPRINTK(PROBE, ERR, "The NVM Checksum Is Not Valid\n");
		err = -EIO;
		goto err_eeprom;
	}

	/* copy the MAC address out of the NVM */

	if (e1000_read_mac_addr(&adapter->hw))
		DPRINTK(PROBE, ERR, "NVM Read Error\n");
	memcpy(netdev->dev_addr, adapter->hw.mac.addr, netdev->addr_len);
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, adapter->hw.mac.addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->perm_addr)) {
#else
	if (!is_valid_ether_addr(netdev->dev_addr)) {
#endif
		DPRINTK(PROBE, ERR, "Invalid MAC Address\n");
		err = -EIO;
		goto err_eeprom;
	}

	INIT_DELAYED_WORK(&adapter->watchdog_task, e1000_watchdog_task);
	INIT_DELAYED_WORK(&adapter->fifo_stall_task,
			  e1000_82547_tx_fifo_stall_task);
	INIT_DELAYED_WORK(&adapter->phy_info_task, e1000_update_phy_info_task);
	INIT_WORK(&adapter->reset_task, e1000_reset_task);

	e1000_check_options(adapter);

	/* Initial Wake on LAN setting
	 * If APM wake is enabled in the EEPROM,
	 * enable the ACPI Magic Packet filter
	 */

	switch (adapter->hw.mac.type) {
	case e1000_82542:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_nvm(&adapter->hw,
			NVM_INIT_CONTROL2_REG, 1, &eeprom_data);
		eeprom_apme_mask = E1000_EEPROM_82544_APM;
		break;
	case e1000_ich8lan:
	case e1000_ich9lan:
		/* APME bit in EEPROM is mapped to WUC.APME */
		eeprom_data = E1000_READ_REG(&adapter->hw, E1000_WUC);
		eeprom_apme_mask = E1000_WUC_APME;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
	case e1000_82571:
	case e1000_80003es2lan:
		if (adapter->hw.bus.func == 1) {
			e1000_read_nvm(&adapter->hw,
				NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		}
		/* Fall Through */
	default:
		e1000_read_nvm(&adapter->hw,
			NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & eeprom_apme_mask)
		adapter->eeprom_wol |= E1000_WUFC_MAG;

	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port */
	switch (pdev->device) {
	case E1000_DEV_ID_82546GB_PCIE:
	case E1000_DEV_ID_82571EB_SERDES_QUAD:
		adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546GB_FIBER:
	case E1000_DEV_ID_82571EB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting */
		if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		    E1000_STATUS_FUNC_1)
			adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
	case E1000_DEV_ID_82571EB_QUAD_COPPER:
	case E1000_DEV_ID_82571EB_QUAD_FIBER:
	case E1000_DEV_ID_82571EB_QUAD_COPPER_LP:
	case E1000_DEV_ID_82571PT_QUAD_COPPER:
		/* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->eeprom_wol = 0;
		else
			adapter->flags |= E1000_FLAG_QUAD_PORT_A;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
		break;
	}

	/* initialize the wol settings based on the eeprom settings */
	adapter->wol = adapter->eeprom_wol;

	/* print bus type/speed/width info */
	{
	struct e1000_hw *hw = &adapter->hw;
	DPRINTK(PROBE, INFO, "(PCI%s:%s:%s) ",
		((hw->bus.type == e1000_bus_type_pcix) ? "-X" :
		 (hw->bus.type == e1000_bus_type_pci_express ? " Express":"")),
		((hw->bus.speed == e1000_bus_speed_2500) ? "2.5Gb/s" :
		 (hw->bus.speed == e1000_bus_speed_133) ? "133MHz" :
		 (hw->bus.speed == e1000_bus_speed_120) ? "120MHz" :
		 (hw->bus.speed == e1000_bus_speed_100) ? "100MHz" :
		 (hw->bus.speed == e1000_bus_speed_66) ? "66MHz" : "33MHz"),
		((hw->bus.width == e1000_bus_width_64) ? "64-bit" :
		 (hw->bus.width == e1000_bus_width_pcie_x4) ? "Width x4" :
		 (hw->bus.width == e1000_bus_width_pcie_x1) ? "Width x1" :
		 "32-bit"));
	}

	for (i = 0; i < 6; i++)
		printk("%2.2x%c", netdev->dev_addr[i], i == 5 ? '\n' : ':');

	/* reset the hardware with the new settings */
	e1000_reset(adapter);

	/* If the controller is 82573 or ICH and f/w is AMT, do not set
	 * DRV_LOAD until the interface is up.  For all other cases,
	 * let the f/w know that the h/w is now under the control
	 * of the driver. */
	if (((adapter->hw.mac.type != e1000_82573) &&
	     (adapter->hw.mac.type != e1000_ich8lan) &&
	     (adapter->hw.mac.type != e1000_ich9lan)) ||
	    !e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	/* tell the stack to leave us alone until e1000_open() is called */
	rtnetif_carrier_off(netdev);
	rtnetif_stop_queue(netdev);

	strcpy(netdev->name, "rteth%d");
	err = rt_register_rtnetdev(netdev);
	if (err)
		goto err_register;

	DPRINTK(PROBE, INFO, "Intel(R) PRO/1000 Network Connection\n");

	cards_found++;
	return 0;

err_register:
err_hw_init:
	e1000_release_hw_control(adapter);
err_eeprom:
	if (!e1000_check_reset_block(&adapter->hw))
		e1000_phy_hw_reset(&adapter->hw);

	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);

	e1000_remove_device(&adapter->hw);
err_flashmap:
	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);
err_sw_init:
	iounmap(adapter->hw.hw_addr);
err_ioremap:
	rtdev_free(netdev);
err_alloc_etherdev:
	pci_release_regions(pdev);
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

/**
 * e1000_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * e1000_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void e1000_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	e1000_down_and_stop(adapter);

	e1000_release_manageability(adapter);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant. */
	e1000_release_hw_control(adapter);

	rt_unregister_rtnetdev(netdev);

	if (!e1000_check_reset_block(&adapter->hw))
		e1000_phy_hw_reset(&adapter->hw);

	e1000_remove_device(&adapter->hw);

	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);

	iounmap(adapter->hw.hw_addr);
	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);
	pci_release_regions(pdev);

	rtdev_free(netdev);

	pci_disable_device(pdev);
}

/**
 * e1000_sw_init - Initialize general software structures (struct e1000_adapter)
 * @adapter: board private structure to initialize
 *
 * e1000_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int e1000_sw_init(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
#ifdef CONFIG_E1000_NAPI
	int i;
#endif

	/* PCI config space info */

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_device_id = pdev->subsystem_device;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &hw->revision_id);

	pci_read_config_word(pdev, PCI_COMMAND, &hw->bus.pci_cmd_word);

	adapter->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;
	adapter->rx_ps_bsize0 = E1000_RXBUFFER_128;
	adapter->max_frame_size = netdev->mtu + ETH_HLEN + ETHERNET_FCS_SIZE;
	adapter->min_frame_size = ETH_ZLEN + ETHERNET_FCS_SIZE;

	/* Initialize the hardware-specific values */
	if (e1000_setup_init_funcs(hw, FALSE)) {
		DPRINTK(PROBE, ERR, "Hardware Initialization Failure\n");
		return -EIO;
	}

#ifdef CONFIG_E1000_MQ
	/* Number of supported queues.
	 * TODO: It's assumed num_rx_queues >= num_tx_queues, since multi-rx
	 * queues are much more interesting.  Is it worth coding for the
	 * possibility (however improbable) of num_tx_queues > num_rx_queues?
	 */
	switch (hw->mac.type) {
	case e1000_82571:
	case e1000_82572:
	case e1000_82573:
	case e1000_80003es2lan:
		adapter->num_tx_queues = 2;
		adapter->num_rx_queues = 2;
		break;
	case e1000_ich8lan:
	case e1000_ich9lan:
		if ((adapter->hw.device_id == E1000_DEV_ID_ICH8_IGP_AMT) ||
		    (adapter->hw.device_id == E1000_DEV_ID_ICH8_IGP_M_AMT) ||
		    (adapter->hw.device_id == E1000_DEV_ID_ICH9_IGP_AMT)) {
			adapter->num_tx_queues = 2;
			adapter->num_rx_queues = 2;
			break;
		}
		/* Fall through - remaining ICH SKUs do not support MQ */
	default:
		/* All hardware before 82571 only have 1 queue each for Rx/Tx.
		 * However, the 82571 family does not have MSI-X, so multi-
		 * queue isn't enabled.
		 * It'd be wise not to mess with this default case. :) */
		adapter->num_tx_queues = 1;
		adapter->num_rx_queues = 1;
		netdev->egress_subqueue_count = 0;
		break;
	}
	adapter->num_rx_queues = min(adapter->num_rx_queues, num_online_cpus());
	adapter->num_tx_queues = min(adapter->num_tx_queues, num_online_cpus());

	if ((adapter->num_tx_queues > 1) || (adapter->num_rx_queues > 1)) {
		netdev->egress_subqueue = (struct net_device_subqueue *)
					   ((void *)adapter +
					    sizeof(struct e1000_adapter));
		netdev->egress_subqueue_count = adapter->num_tx_queues;
		DPRINTK(DRV, INFO, "Multiqueue Enabled: RX queues = %u, "
			"TX queues = %u\n", adapter->num_rx_queues,
			adapter->num_tx_queues);
	}
#else
	adapter->num_tx_queues = 1;
	adapter->num_rx_queues = 1;
#endif

	if (e1000_alloc_queues(adapter)) {
		DPRINTK(PROBE, ERR, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

#ifdef CONFIG_E1000_NAPI
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct e1000_rx_ring *rx_ring = &adapter->rx_ring[i];
		netif_napi_add(adapter->netdev, &rx_ring->napi, e1000_poll, 64);
	}
	rtdm_lock_init(&adapter->tx_queue_lock);
#ifdef CONFIG_E1000_MQ
	for (i = 0; i < adapter->num_tx_queues; i++)
		rtdm_lock_init(&adapter->tx_ring[i].tx_queue_lock);
#endif
#endif

	/* Explicitly disable IRQ since the NIC can be in any state. */
	atomic_set(&adapter->irq_sem, 0);
	e1000_irq_disable(adapter);

	rtdm_lock_init(&adapter->stats_lock);

	set_bit(__E1000_DOWN, &adapter->state);
	return 0;
}

/**
 * e1000_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 **/
static int e1000_alloc_queues(struct e1000_adapter *adapter)
{
	adapter->tx_ring = kcalloc(adapter->num_tx_queues,
				   sizeof(struct e1000_tx_ring), GFP_KERNEL);
	if (!adapter->tx_ring)
		return -ENOMEM;

	adapter->rx_ring = kcalloc(adapter->num_rx_queues,
				   sizeof(struct e1000_rx_ring), GFP_KERNEL);
	if (!adapter->rx_ring) {
		kfree(adapter->tx_ring);
		return -ENOMEM;
	}

#ifdef CONFIG_E1000_MQ
	adapter->cpu_tx_ring = alloc_percpu(struct e1000_tx_ring *);
#endif

	return E1000_SUCCESS;
}

#ifdef CONFIG_E1000_MQ
static void e1000_setup_queue_mapping(struct e1000_adapter *adapter)
{
	int i, cpu;

	lock_cpu_hotplug();
	i = 0;
	for_each_online_cpu(cpu) {
		*per_cpu_ptr(adapter->cpu_tx_ring, cpu) =
			     &adapter->tx_ring[i % adapter->num_tx_queues];
		i++;
	}
	unlock_cpu_hotplug();
}
#endif

/**
 * e1000_intr_msi_test - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t e1000_intr_msi_test(int irq, void *data)
{
	struct net_device *netdev = data;
	struct e1000_adapter *adapter = netdev_priv(netdev);

	u32 icr = E1000_READ_REG(&adapter->hw, E1000_ICR);
	DPRINTK(HW,INFO, "icr is %08X\n", icr);
	if (icr & E1000_ICR_RXSEQ) {
		adapter->flags |= E1000_FLAG_HAS_MSI;
		wmb();
	}

	return IRQ_HANDLED;
}

/**
 * e1000_test_msi_interrupt - Returns 0 for successful test
 * @adapter: board private struct
 *
 * code flow taken from tg3.c
 **/
static int e1000_test_msi_interrupt(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err;

	/* poll_enable hasn't been called yet, so don't need disable */
	/* clear any pending events */
	E1000_READ_REG(&adapter->hw, E1000_ICR);

	/* free the real vector and request a test handler */
	e1000_free_irq(adapter);

	err = pci_enable_msi(adapter->pdev);
	err = request_irq(adapter->pdev->irq, &e1000_intr_msi_test, 0,
			  netdev->name, netdev);
	if (err) {
		pci_disable_msi(adapter->pdev);
		goto msi_test_failed;
	}

	/* our temporary test variable */
	adapter->flags &= ~E1000_FLAG_HAS_MSI;
	wmb();

	e1000_irq_enable(adapter);

	/* fire an unusual interrupt on the test handler */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_RXSEQ);
	E1000_WRITE_FLUSH(&adapter->hw);
	msleep(50);

	e1000_irq_disable(adapter);

	rmb();
	if (!(adapter->flags & E1000_FLAG_HAS_MSI)) {
		adapter->flags |= E1000_FLAG_HAS_MSI;
		err = -EIO;
		DPRINTK(HW, INFO, "MSI interrupt test failed!\n");
	}

	free_irq(adapter->pdev->irq, netdev);
	pci_disable_msi(adapter->pdev);

	if (err == -EIO)
		goto msi_test_failed;

	/* okay so the test worked, restore settings */
	DPRINTK(HW, INFO, "MSI interrupt test succeeded!\n");
msi_test_failed:
	/* restore the original vector, even if it failed */
	e1000_request_irq(adapter);
	return err;
}

/**
 * e1000_test_msi - Returns 0 if MSI test succeeds and INTx mode is restored
 * @adapter: board private struct
 *
 * code flow taken from tg3.c, called with e1000 interrupts disabled.
 **/
static int e1000_test_msi(struct e1000_adapter *adapter)
{
	int err;
	u16 pci_cmd;

	if (!(adapter->flags & E1000_FLAG_MSI_ENABLED) ||
	    !(adapter->flags & E1000_FLAG_HAS_MSI))
		return 0;

	/* disable SERR in case the MSI write causes a master abort */
	pci_read_config_word(adapter->pdev, PCI_COMMAND, &pci_cmd);
	pci_write_config_word(adapter->pdev, PCI_COMMAND,
			      pci_cmd & ~PCI_COMMAND_SERR);

	err = e1000_test_msi_interrupt(adapter);

	/* restore previous setting of command word */
	pci_write_config_word(adapter->pdev, PCI_COMMAND, pci_cmd);

	/* success ! */
	if (!err)
		return 0;

	/* EIO means MSI test failed */
	if (err != -EIO)
		return err;

	/* back to INTx mode */
	DPRINTK(PROBE, WARNING, "MSI interrupt test failed, using legacy "
		"interrupt.\n");

	e1000_free_irq(adapter);
	adapter->flags &= ~E1000_FLAG_HAS_MSI;

	err = e1000_request_irq(adapter);

	return err;
}

/**
 * e1000_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int e1000_open(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int err;
	/* disallow open during test */
	if (test_bit(__E1000_TESTING, &adapter->state))
		return -EBUSY;

	/* allocate transmit descriptors */
	err = e1000_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = e1000_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;

	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		e1000_power_up_phy(&adapter->hw);
		e1000_setup_link(&adapter->hw);
	}

#ifdef NETIF_F_HW_VLAN_TX
	adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
	if ((adapter->hw.mng_cookie.status &
	     E1000_MNG_DHCP_COOKIE_STATUS_VLAN)) {
		e1000_update_mng_vlan(adapter);
	}
#endif

	/* For 82573 and ICHx if AMT is enabled, let the firmware know
	 * that the network interface is now open */
	if (((adapter->hw.mac.type == e1000_82573) ||
	     (adapter->hw.mac.type == e1000_ich8lan) ||
	     (adapter->hw.mac.type == e1000_ich9lan)) &&
	    e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.  */
	e1000_configure(adapter);


	err = e1000_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* work around PCIe errata with MSI interrupts causing some chipsets to
	 * ignore e1000 MSI messages, which means we need to test our MSI
	 * interrupt now */
	err = e1000_test_msi(adapter);
	if (err) {
		DPRINTK(PROBE, ERR, "Interrupt allocation failed\n");
		goto err_req_irq;
	}

	/* From here on the code is the same as e1000_up() */
	clear_bit(__E1000_DOWN, &adapter->state);

	e1000_napi_enable_all(adapter);

	schedule_delayed_work(&adapter->watchdog_task, 1);
	e1000_irq_enable(adapter);

	/* fire a link status change interrupt to start the watchdog */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_LSC);

	return E1000_SUCCESS;

err_req_irq:
	e1000_release_hw_control(adapter);
	/* Power down the PHY so no link is implied when interface is down *
	 * The PHY cannot be powered down if any of the following is TRUE *
	 * (a) WoL is enabled
	 * (b) AMT is active
	 * (c) SoL/IDER session is active */
	if (!adapter->wol && adapter->hw.mac.type >= e1000_82540 &&
	   adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_down_phy(&adapter->hw);
	e1000_free_all_rx_resources(adapter);
err_setup_rx:
	e1000_free_all_tx_resources(adapter);
err_setup_tx:
	e1000_reset(adapter);

	return err;
}

/**
 * e1000_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int e1000_close(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	WARN_ON(test_bit(__E1000_RESETTING, &adapter->state));
	e1000_down(adapter);
	/* Power down the PHY so no link is implied when interface is down *
	 * The PHY cannot be powered down if any of the following is TRUE *
	 * (a) WoL is enabled
	 * (b) AMT is active
	 * (c) SoL/IDER session is active */
	if (!adapter->wol && adapter->hw.mac.type >= e1000_82540 &&
	   adapter->hw.phy.media_type == e1000_media_type_copper)
		e1000_power_down_phy(&adapter->hw);
	e1000_free_irq(adapter);

	e1000_free_all_tx_resources(adapter);
	e1000_free_all_rx_resources(adapter);

#ifdef NETIF_F_HW_VLAN_TX
	/* kill manageability vlan ID if supported, but not if a vlan with
	 * the same ID is registered on the host OS (let 8021q kill it) */
	if ((adapter->hw.mng_cookie.status &
			  E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
	     !(adapter->vlgrp &&
	       vlan_group_get_device(adapter->vlgrp, adapter->mng_vlan_id))) {
		e1000_vlan_rx_kill_vid(netdev, adapter->mng_vlan_id);
	}
#endif

	/* For 82573 and ICHx if AMT is enabled, let the firmware know
	 * that the network interface is now closed */
	if (((adapter->hw.mac.type == e1000_82573) ||
	     (adapter->hw.mac.type == e1000_ich8lan) ||
	     (adapter->hw.mac.type == e1000_ich9lan)) &&
	    e1000_check_mng_mode(&adapter->hw))
		e1000_release_hw_control(adapter);

	return 0;
}

/**
 * e1000_check_64k_bound - check that memory doesn't cross 64kB boundary
 * @adapter: address of board private structure
 * @start: address of beginning of memory
 * @len: length of memory
 **/
static bool e1000_check_64k_bound(struct e1000_adapter *adapter,
				       void *start, unsigned long len)
{
	unsigned long begin = (unsigned long) start;
	unsigned long end = begin + len;

	/* First rev 82545 and 82546 need to not allow any memory
	 * write location to cross 64k boundary due to errata 23 */
	if (adapter->hw.mac.type == e1000_82545 ||
	    adapter->hw.mac.type == e1000_82546) {
		return ((begin ^ (end - 1)) >> 16) != 0 ? FALSE : TRUE;
	}

	return TRUE;
}

/**
 * e1000_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 * @tx_ring:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
static int e1000_setup_tx_resources(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring)
{
	struct pci_dev *pdev = adapter->pdev;
	int size;

	size = sizeof(struct e1000_buffer) * tx_ring->count;
	tx_ring->buffer_info = vmalloc(size);
	if (!tx_ring->buffer_info) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the transmit descriptor ring\n");
		return -ENOMEM;
	}
	memset(tx_ring->buffer_info, 0, size);

	/* round up to nearest 4K */

	tx_ring->size = tx_ring->count * sizeof(struct e1000_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	tx_ring->desc = pci_alloc_consistent(pdev, tx_ring->size,
					     &tx_ring->dma);
	if (!tx_ring->desc) {
setup_tx_desc_die:
		vfree(tx_ring->buffer_info);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the transmit descriptor ring\n");
		return -ENOMEM;
	}

	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, tx_ring->desc, tx_ring->size)) {
		void *olddesc = tx_ring->desc;
		dma_addr_t olddma = tx_ring->dma;
		DPRINTK(TX_ERR, ERR, "tx_ring align check failed: %u bytes "
				     "at %p\n", tx_ring->size, tx_ring->desc);
		/* Try again, without freeing the previous */
		tx_ring->desc = pci_alloc_consistent(pdev, tx_ring->size,
						     &tx_ring->dma);
		/* Failed allocation, critical failure */
		if (!tx_ring->desc) {
			pci_free_consistent(pdev, tx_ring->size, olddesc,
					    olddma);
			goto setup_tx_desc_die;
		}

		if (!e1000_check_64k_bound(adapter, tx_ring->desc,
					   tx_ring->size)) {
			/* give up */
			pci_free_consistent(pdev, tx_ring->size, tx_ring->desc,
					    tx_ring->dma);
			pci_free_consistent(pdev, tx_ring->size, olddesc,
					    olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate aligned memory "
				"for the transmit descriptor ring\n");
			vfree(tx_ring->buffer_info);
			return -ENOMEM;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, tx_ring->size, olddesc,
					    olddma);
		}
	}
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	rtdm_lock_init(&tx_ring->tx_lock);

	return 0;
}

/**
 * e1000_setup_all_tx_resources - wrapper to allocate Tx resources
 * @adapter: board private structure
 *
 * this allocates tx resources for all queues, return 0 on success, negative
 * on failure
 **/
int e1000_setup_all_tx_resources(struct e1000_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		err = e1000_setup_tx_resources(adapter, &adapter->tx_ring[i]);
		if (err) {
			DPRINTK(PROBE, ERR,
				"Allocation for Tx Queue %u failed\n", i);
			for (i-- ; i >= 0; i--)
				e1000_free_tx_resources(adapter,
							&adapter->tx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 * e1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void e1000_configure_tx(struct e1000_adapter *adapter)
{
	u64 tdba;
	struct e1000_hw *hw = &adapter->hw;
	u32 tdlen, tctl, tipg, tarc;
	u32 ipgr1, ipgr2;
	int i;

	/* Setup the HW Tx Head and Tail descriptor pointers */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		tdba = adapter->tx_ring[i].dma;
		tdlen = adapter->tx_ring[i].count * sizeof(struct e1000_tx_desc);
		E1000_WRITE_REG(hw, E1000_TDBAL(i), (tdba & 0x00000000ffffffffULL));
		E1000_WRITE_REG(hw, E1000_TDBAH(i), (tdba >> 32));
		E1000_WRITE_REG(hw, E1000_TDLEN(i), tdlen);
		E1000_WRITE_REG(hw, E1000_TDH(i), 0);
		E1000_WRITE_REG(hw, E1000_TDT(i), 0);
		adapter->tx_ring[i].tdh = E1000_REGISTER(hw, E1000_TDH(i));
		adapter->tx_ring[i].tdt = E1000_REGISTER(hw, E1000_TDT(i));
	}


	/* Set the default values for the Tx Inter Packet Gap timer */
	if (adapter->hw.mac.type <= e1000_82547_rev_2 &&
	    (hw->phy.media_type == e1000_media_type_fiber ||
	     hw->phy.media_type == e1000_media_type_internal_serdes))
		tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
	else
		tipg = DEFAULT_82543_TIPG_IPGT_COPPER;

	switch (hw->mac.type) {
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		ipgr1 = DEFAULT_82542_TIPG_IPGR1;
		ipgr2 = DEFAULT_82542_TIPG_IPGR2;
		break;
	case e1000_80003es2lan:
		ipgr1 = DEFAULT_82543_TIPG_IPGR1;
		ipgr2 = DEFAULT_80003ES2LAN_TIPG_IPGR2;
		break;
	default:
		ipgr1 = DEFAULT_82543_TIPG_IPGR1;
		ipgr2 = DEFAULT_82543_TIPG_IPGR2;
		break;
	}
	tipg |= ipgr1 << E1000_TIPG_IPGR1_SHIFT;
	tipg |= ipgr2 << E1000_TIPG_IPGR2_SHIFT;
	E1000_WRITE_REG(hw, E1000_TIPG, tipg);

	/* Set the Tx Interrupt Delay register */

	E1000_WRITE_REG(hw, E1000_TIDV, adapter->tx_int_delay);
	if (adapter->flags & E1000_FLAG_HAS_INTR_MODERATION)
		E1000_WRITE_REG(hw, E1000_TADV, adapter->tx_abs_int_delay);

	/* Program the Transmit Control Register */

	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
		(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);

	if (hw->mac.type == e1000_82571 || hw->mac.type == e1000_82572) {
		tarc = E1000_READ_REG(hw, E1000_TARC(0));
		/* set the speed mode bit, we'll clear it if we're not at
		 * gigabit link later */
#define SPEED_MODE_BIT (1 << 21)
		tarc |= SPEED_MODE_BIT;
		E1000_WRITE_REG(hw, E1000_TARC(0), tarc);
	} else if (hw->mac.type == e1000_80003es2lan) {
		tarc = E1000_READ_REG(hw, E1000_TARC(0));
		tarc |= 1;
		E1000_WRITE_REG(hw, E1000_TARC(0), tarc);
		tarc = E1000_READ_REG(hw, E1000_TARC(1));
		tarc |= 1;
		E1000_WRITE_REG(hw, E1000_TARC(1), tarc);
	}

	e1000_config_collision_dist(hw);

	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter->txd_cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS;

	/* only set IDE if we are delaying interrupts using the timers */
	if (adapter->tx_int_delay)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;

	if (hw->mac.type < e1000_82543)
		adapter->txd_cmd |= E1000_TXD_CMD_RPS;
	else
		adapter->txd_cmd |= E1000_TXD_CMD_RS;

	/* Cache if we're 82544 running in PCI-X because we'll
	 * need this to apply a workaround later in the send path. */
	if (hw->mac.type == e1000_82544 &&
	    hw->bus.type == e1000_bus_type_pcix)
		adapter->pcix_82544 = 1;

	E1000_WRITE_REG(hw, E1000_TCTL, tctl);

}

/**
 * e1000_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: board private structure
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_setup_rx_resources(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring)
{
	struct pci_dev *pdev = adapter->pdev;
	int size, desc_len;

	size = sizeof(struct e1000_rx_buffer) * rx_ring->count;
	rx_ring->buffer_info = vmalloc(size);
	if (!rx_ring->buffer_info) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(rx_ring->buffer_info, 0, size);

	rx_ring->ps_page = kcalloc(rx_ring->count, sizeof(struct e1000_ps_page),
				   GFP_KERNEL);
	if (!rx_ring->ps_page) {
		vfree(rx_ring->buffer_info);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}

	rx_ring->ps_page_dma = kcalloc(rx_ring->count,
				       sizeof(struct e1000_ps_page_dma),
				       GFP_KERNEL);
	if (!rx_ring->ps_page_dma) {
		vfree(rx_ring->buffer_info);
		kfree(rx_ring->ps_page);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}

	if (adapter->hw.mac.type <= e1000_82547_rev_2)
		desc_len = sizeof(struct e1000_rx_desc);
	else
		desc_len = sizeof(union e1000_rx_desc_packet_split);

	/* Round up to nearest 4K */

	rx_ring->size = rx_ring->count * desc_len;
	rx_ring->size = ALIGN(rx_ring->size, 4096);

	rx_ring->desc = pci_alloc_consistent(pdev, rx_ring->size,
					     &rx_ring->dma);

	if (!rx_ring->desc) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
setup_rx_desc_die:
		vfree(rx_ring->buffer_info);
		kfree(rx_ring->ps_page);
		kfree(rx_ring->ps_page_dma);
		return -ENOMEM;
	}

	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, rx_ring->desc, rx_ring->size)) {
		void *olddesc = rx_ring->desc;
		dma_addr_t olddma = rx_ring->dma;
		DPRINTK(RX_ERR, ERR, "rx_ring align check failed: %u bytes "
				     "at %p\n", rx_ring->size, rx_ring->desc);
		/* Try again, without freeing the previous */
		rx_ring->desc = pci_alloc_consistent(pdev, rx_ring->size,
						     &rx_ring->dma);
		/* Failed allocation, critical failure */
		if (!rx_ring->desc) {
			pci_free_consistent(pdev, rx_ring->size, olddesc,
					    olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate memory "
				"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		}

		if (!e1000_check_64k_bound(adapter, rx_ring->desc,
					   rx_ring->size)) {
			/* give up */
			pci_free_consistent(pdev, rx_ring->size, rx_ring->desc,
					    rx_ring->dma);
			pci_free_consistent(pdev, rx_ring->size, olddesc,
					    olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate aligned memory "
				"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, rx_ring->size, olddesc,
					    olddma);
		}
	}
	memset(rx_ring->desc, 0, rx_ring->size);

	/* set up ring defaults */
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	rx_ring->rx_skb_top = NULL;
	rx_ring->adapter = adapter;

	return 0;
}

/**
 * e1000_setup_all_rx_resources - wrapper to allocate Rx resources
 * @adapter: board private structure
 *
 * this allocates rx resources for all queues, return 0 on success, negative
 * on failure
 **/
int e1000_setup_all_rx_resources(struct e1000_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		err = e1000_setup_rx_resources(adapter, &adapter->rx_ring[i]);
		if (err) {
			DPRINTK(PROBE, ERR,
				"Allocation for Rx Queue %u failed\n", i);
			for (i-- ; i >= 0; i--)
				e1000_free_rx_resources(adapter,
							&adapter->rx_ring[i]);
			break;
		}
	}

	return err;
}

#define PAGE_USE_COUNT(S) (((S) >> PAGE_SHIFT) + \
			(((S) & (PAGE_SIZE - 1)) ? 1 : 0))
/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
static void e1000_setup_rctl(struct e1000_adapter *adapter)
{
	u32 rctl, rfctl;
	u32 psrctl = 0;
#ifndef CONFIG_E1000_DISABLE_PACKET_SPLIT
	u32 pages = 0;
#endif

	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);

	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
		E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
		(adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* disable the stripping of CRC because it breaks
	 * BMC firmware connected over SMBUS
	if (adapter->hw.mac.type > e1000_82543)
		rctl |= E1000_RCTL_SECRC;
	*/

	if (e1000_tbi_sbp_enabled_82543(&adapter->hw))
		rctl |= E1000_RCTL_SBP;
	else
		rctl &= ~E1000_RCTL_SBP;

	if (adapter->netdev->mtu <= ETH_DATA_LEN)
		rctl &= ~E1000_RCTL_LPE;
	else
		rctl |= E1000_RCTL_LPE;

	/* Setup buffer sizes */
	rctl &= ~E1000_RCTL_SZ_4096;
	rctl |= E1000_RCTL_BSEX;
	switch (adapter->rx_buffer_len) {
		case E1000_RXBUFFER_256:
			rctl |= E1000_RCTL_SZ_256;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case E1000_RXBUFFER_512:
			rctl |= E1000_RCTL_SZ_512;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case E1000_RXBUFFER_1024:
			rctl |= E1000_RCTL_SZ_1024;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case E1000_RXBUFFER_2048:
		default:
			rctl |= E1000_RCTL_SZ_2048;
			rctl &= ~E1000_RCTL_BSEX;
			break;
		case E1000_RXBUFFER_4096:
			rctl |= E1000_RCTL_SZ_4096;
			break;
		case E1000_RXBUFFER_8192:
			rctl |= E1000_RCTL_SZ_8192;
			break;
		case E1000_RXBUFFER_16384:
			rctl |= E1000_RCTL_SZ_16384;
			break;
	}

#ifndef CONFIG_E1000_DISABLE_PACKET_SPLIT
	/* 82571 and greater support packet-split where the protocol
	 * header is placed in skb->data and the packet data is
	 * placed in pages hanging off of skb_shinfo(skb)->nr_frags.
	 * In the case of a non-split, skb->data is linearly filled,
	 * followed by the page buffers.  Therefore, skb->data is
	 * sized to hold the largest protocol header.
	 */
	/* allocations using alloc_page take too long for regular MTU
	 * so only enable packet split for jumbo frames */
	pages = PAGE_USE_COUNT(adapter->netdev->mtu);
	if ((adapter->hw.mac.type >= e1000_82571) && (pages <= 3) &&
	    PAGE_SIZE <= 16384 && (rctl & E1000_RCTL_LPE))
		adapter->rx_ps_pages = pages;
	else
		adapter->rx_ps_pages = 0;
#endif

	if (adapter->rx_ps_pages) {
		/* Configure extra packet-split registers */
		rfctl = E1000_READ_REG(&adapter->hw, E1000_RFCTL);
		rfctl |= E1000_RFCTL_EXTEN;
		/* disable packet split support for IPv6 extension headers,
		 * because some malformed IPv6 headers can hang the RX */
		rfctl |= (E1000_RFCTL_IPV6_EX_DIS |
			  E1000_RFCTL_NEW_IPV6_EXT_DIS);

		E1000_WRITE_REG(&adapter->hw, E1000_RFCTL, rfctl);

		/* disable the stripping of CRC because it breaks
		 * BMC firmware connected over SMBUS */
		rctl |= E1000_RCTL_DTYP_PS /* | E1000_RCTL_SECRC */;

		psrctl |= adapter->rx_ps_bsize0 >>
			E1000_PSRCTL_BSIZE0_SHIFT;

		switch (adapter->rx_ps_pages) {
		case 3:
			psrctl |= PAGE_SIZE <<
				E1000_PSRCTL_BSIZE3_SHIFT;
		case 2:
			psrctl |= PAGE_SIZE <<
				E1000_PSRCTL_BSIZE2_SHIFT;
		case 1:
			psrctl |= PAGE_SIZE >>
				E1000_PSRCTL_BSIZE1_SHIFT;
			break;
		}

		E1000_WRITE_REG(&adapter->hw, E1000_PSRCTL, psrctl);
	}

	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	adapter->flags &= ~E1000_FLAG_RX_RESTART_NOW;
}

/**
 * e1000_configure_rx - Configure 8254x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void e1000_configure_rx(struct e1000_adapter *adapter)
{
	u64 rdba;
	struct e1000_hw *hw = &adapter->hw;
	u32 rdlen, rctl, rxcsum, ctrl_ext;
	int i;

	if (adapter->rx_ps_pages) {
		/* this is a 32 byte descriptor */
		rdlen = adapter->rx_ring[0].count *
			sizeof(union e1000_rx_desc_packet_split);
		adapter->clean_rx = e1000_clean_rx_irq_ps;
		adapter->alloc_rx_buf = e1000_alloc_rx_buffers_ps;
#ifdef CONFIG_E1000_NAPI
	} else if (adapter->netdev->mtu > MAXIMUM_ETHERNET_VLAN_SIZE) {
		rdlen = adapter->rx_ring[0].count *
			sizeof(struct e1000_rx_desc);
		adapter->clean_rx = e1000_clean_jumbo_rx_irq;
		adapter->alloc_rx_buf = e1000_alloc_jumbo_rx_buffers;
#endif
	} else {
		rdlen = adapter->rx_ring[0].count *
			sizeof(struct e1000_rx_desc);
		adapter->clean_rx = e1000_clean_rx_irq;
		adapter->alloc_rx_buf = e1000_alloc_rx_buffers;
	}

	/* disable receives while setting up the descriptors */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	E1000_WRITE_FLUSH(hw);
	mdelay(10);

	/* set the Receive Delay Timer Register */
	E1000_WRITE_REG(hw, E1000_RDTR, adapter->rx_int_delay);

	if (adapter->flags & E1000_FLAG_HAS_INTR_MODERATION) {
		E1000_WRITE_REG(hw, E1000_RADV, adapter->rx_abs_int_delay);
		if (adapter->itr_setting != 0)
			E1000_WRITE_REG(hw, E1000_ITR,
				1000000000 / (adapter->itr * 256));
	}

	if (hw->mac.type >= e1000_82571) {
		ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
		/* Reset delay timers after every interrupt */
		ctrl_ext |= E1000_CTRL_EXT_INT_TIMER_CLR;
#ifdef CONFIG_E1000_NAPI
		/* Auto-Mask interrupts upon ICR access */
		ctrl_ext |= E1000_CTRL_EXT_IAME;
		E1000_WRITE_REG(hw, E1000_IAM, 0xffffffff);
#endif
		E1000_WRITE_REG(hw, E1000_CTRL_EXT, ctrl_ext);
		E1000_WRITE_FLUSH(hw);
	}

	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rdba = adapter->rx_ring[i].dma;
		E1000_WRITE_REG(hw, E1000_RDBAL(i), (rdba & 0x00000000ffffffffULL));
		E1000_WRITE_REG(hw, E1000_RDBAH(i), (rdba >> 32));
		E1000_WRITE_REG(hw, E1000_RDLEN(i), rdlen);
		E1000_WRITE_REG(hw, E1000_RDH(i), 0);
		E1000_WRITE_REG(hw, E1000_RDT(i), 0);
		adapter->rx_ring[i].rdh = E1000_REGISTER(hw, E1000_RDH(i));
		adapter->rx_ring[i].rdt = E1000_REGISTER(hw, E1000_RDT(i));
	}

#ifdef CONFIG_E1000_MQ
	if (adapter->num_rx_queues > 1) {
		u32 random[10];
		u32 reta, mrqc;
		int i;

		get_random_bytes(&random[0], 40);

		switch (adapter->num_rx_queues) {
		default:
			reta = 0x00800080;
			mrqc = E1000_MRQC_ENABLE_RSS_2Q;
			break;
		}

		/* Fill out redirection table */
		for (i = 0; i < 32; i++)
			E1000_WRITE_REG_ARRAY(hw, E1000_RETA, i, reta);
		/* Fill out hash function seeds */
		for (i = 0; i < 10; i++)
			E1000_WRITE_REG_ARRAY(hw, E1000_RSSRK, i, random[i]);

		mrqc |= (E1000_MRQC_RSS_FIELD_IPV4 |
			 E1000_MRQC_RSS_FIELD_IPV4_TCP);

		E1000_WRITE_REG(hw, E1000_MRQC, mrqc);

		/* Multiqueue and packet checksumming are mutually exclusive. */
		rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
		rxcsum |= E1000_RXCSUM_PCSD;
		E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);
	} else if (hw->mac.type >= e1000_82543) {
#else
	if (hw->mac.type >= e1000_82543) {
#endif /* CONFIG_E1000_MQ */
		/* Enable 82543 Receive Checksum Offload for TCP and UDP */
		rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
		if (adapter->rx_csum == TRUE) {
			rxcsum |= E1000_RXCSUM_TUOFL;

			/* Enable 82571 IPv4 payload checksum for UDP fragments
			 * Must be used in conjunction with packet-split. */
			if ((hw->mac.type >= e1000_82571) &&
			    (adapter->rx_ps_pages)) {
				rxcsum |= E1000_RXCSUM_IPPCSE;
			}
		} else {
			rxcsum &= ~E1000_RXCSUM_TUOFL;
			/* don't need to clear IPPCSE as it defaults to 0 */
		}
		E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);
	}

	/* Enable early receives on supported devices, only takes effect when
	 * packet size is equal or larger than the specified value (in 8 byte
	 * units), e.g. using jumbo frames when setting to E1000_ERT_2048 */
	if ((hw->mac.type == e1000_82573 || hw->mac.type == e1000_ich9lan) &&
	    (adapter->netdev->mtu > ETH_DATA_LEN))
		E1000_WRITE_REG(hw, E1000_ERT, E1000_ERT_2048);

	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
}

/**
 * e1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
static void e1000_free_tx_resources(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring)
{
	struct pci_dev *pdev = adapter->pdev;

	e1000_clean_tx_ring(adapter, tx_ring);

	vfree(tx_ring->buffer_info);
	tx_ring->buffer_info = NULL;

	pci_free_consistent(pdev, tx_ring->size, tx_ring->desc, tx_ring->dma);

	tx_ring->desc = NULL;
}

/**
 * e1000_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void e1000_free_all_tx_resources(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		e1000_free_tx_resources(adapter, &adapter->tx_ring[i]);
}

static void e1000_unmap_and_free_tx_resource(struct e1000_adapter *adapter,
					     struct e1000_buffer *buffer_info)
{
	if (buffer_info->dma) {
		pci_unmap_page(adapter->pdev,
				buffer_info->dma,
				buffer_info->length,
				PCI_DMA_TODEVICE);
		buffer_info->dma = 0;
	}
	if (buffer_info->skb) {
		kfree_rtskb(buffer_info->skb);
		buffer_info->skb = NULL;
	}
	/* buffer_info must be completely set up in the transmit path */
}

/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 * @tx_ring: ring to be cleaned
 **/
static void e1000_clean_tx_ring(struct e1000_adapter *adapter,
				struct e1000_tx_ring *tx_ring)
{
	struct e1000_buffer *buffer_info;
	unsigned long size;
	unsigned int i;

	/* Free all the Tx ring sk_buffs */

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_info[i];
		e1000_unmap_and_free_tx_resource(adapter, buffer_info);
	}

	size = sizeof(struct e1000_buffer) * tx_ring->count;
	memset(tx_ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */

	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	tx_ring->last_tx_tso = 0;

	writel(0, adapter->hw.hw_addr + tx_ring->tdh);
	writel(0, adapter->hw.hw_addr + tx_ring->tdt);
}

/**
 * e1000_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void e1000_clean_all_tx_rings(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		e1000_clean_tx_ring(adapter, &adapter->tx_ring[i]);
}

/**
 * e1000_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
static void e1000_free_rx_resources(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring)
{
	struct pci_dev *pdev = adapter->pdev;

	e1000_clean_rx_ring(adapter, rx_ring);

	vfree(rx_ring->buffer_info);
	rx_ring->buffer_info = NULL;
	kfree(rx_ring->ps_page);
	rx_ring->ps_page = NULL;
	kfree(rx_ring->ps_page_dma);
	rx_ring->ps_page_dma = NULL;

	pci_free_consistent(pdev, rx_ring->size, rx_ring->desc, rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 * e1000_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
void e1000_free_all_rx_resources(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		e1000_free_rx_resources(adapter, &adapter->rx_ring[i]);
}

/**
 * e1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 * @rx_ring: ring to free buffers from
 **/
static void e1000_clean_rx_ring(struct e1000_adapter *adapter,
				struct e1000_rx_ring *rx_ring)
{
	struct e1000_rx_buffer *buffer_info;
	struct e1000_ps_page *ps_page;
	struct e1000_ps_page_dma *ps_page_dma;
	struct pci_dev *pdev = adapter->pdev;
	unsigned long size;
	unsigned int i, j;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		if (buffer_info->dma &&
		    adapter->clean_rx == e1000_clean_rx_irq) {
			pci_unmap_single(pdev, buffer_info->dma,
					 adapter->rx_buffer_len,
					 PCI_DMA_FROMDEVICE);
#ifdef CONFIG_E1000_NAPI
		} else if (buffer_info->dma &&
			   adapter->clean_rx == e1000_clean_jumbo_rx_irq) {
			pci_unmap_page(pdev, buffer_info->dma, PAGE_SIZE,
				       PCI_DMA_FROMDEVICE);
#endif
		} else if (buffer_info->dma &&
			   adapter->clean_rx == e1000_clean_rx_irq_ps) {
			pci_unmap_single(pdev, buffer_info->dma,
					 adapter->rx_ps_bsize0,
					 PCI_DMA_FROMDEVICE);
		}
		buffer_info->dma = 0;
		if (buffer_info->page) {
			put_page(buffer_info->page);
			buffer_info->page = NULL;
		}
		if (buffer_info->skb) {
			kfree_rtskb(buffer_info->skb);
			buffer_info->skb = NULL;
		}
		ps_page = &rx_ring->ps_page[i];
		ps_page_dma = &rx_ring->ps_page_dma[i];
		for (j = 0; j < adapter->rx_ps_pages; j++) {
			if (!ps_page->ps_page[j]) break;
			pci_unmap_page(pdev,
				       ps_page_dma->ps_page_dma[j],
				       PAGE_SIZE, PCI_DMA_FROMDEVICE);
			ps_page_dma->ps_page_dma[j] = 0;
			put_page(ps_page->ps_page[j]);
			ps_page->ps_page[j] = NULL;
		}
	}

#ifdef CONFIG_E1000_NAPI
	/* there also may be some cached data from a chained receive */
	if (rx_ring->rx_skb_top) {
		kfree_rtskb(rx_ring->rx_skb_top);
		rx_ring->rx_skb_top = NULL;
	}
#endif

	size = sizeof(struct e1000_rx_buffer) * rx_ring->count;
	memset(rx_ring->buffer_info, 0, size);
	size = sizeof(struct e1000_ps_page) * rx_ring->count;
	memset(rx_ring->ps_page, 0, size);
	size = sizeof(struct e1000_ps_page_dma) * rx_ring->count;
	memset(rx_ring->ps_page_dma, 0, size);

	/* Zero out the descriptor ring */

	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	writel(0, adapter->hw.hw_addr + rx_ring->rdh);
	writel(0, adapter->hw.hw_addr + rx_ring->rdt);
}

/**
 * e1000_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void e1000_clean_all_rx_rings(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		e1000_clean_rx_ring(adapter, &adapter->rx_ring[i]);
}

/* The 82542 2.0 (revision 2) needs to have the receive unit in reset
 * and memory write and invalidate disabled for certain operations
 */
#if 0
static void e1000_enter_82542_rst(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	u32 rctl;

	if (adapter->hw.mac.type != e1000_82542)
		return;
	if (adapter->hw.revision_id != E1000_REVISION_2)
		return;

	e1000_pci_clear_mwi(&adapter->hw);

	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	rctl |= E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);

	if (rtnetif_running(netdev))
		e1000_clean_all_rx_rings(adapter);
}

static void e1000_leave_82542_rst(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	u32 rctl;

	if (adapter->hw.mac.type != e1000_82542)
		return;
	if (adapter->hw.revision_id != E1000_REVISION_2)
		return;

	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	rctl &= ~E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);

	if (adapter->hw.bus.pci_cmd_word & PCI_COMMAND_INVALIDATE)
		e1000_pci_set_mwi(&adapter->hw);

	if (rtnetif_running(netdev)) {
		/* No need to loop, because 82542 supports only 1 queue */
		struct e1000_rx_ring *ring = &adapter->rx_ring[0];
		e1000_configure_rx(adapter);
		adapter->alloc_rx_buf(adapter, ring, E1000_DESC_UNUSED(ring));
	}
}

/**
 * e1000_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_set_mac(struct net_device *netdev, void *p)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* 82542 2.0 needs to be in reset to write receive address registers */

	if (adapter->hw.mac.type == e1000_82542)
		e1000_enter_82542_rst(adapter);

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(adapter->hw.mac.addr, addr->sa_data, netdev->addr_len);

	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/* With 82571 controllers, LAA may be overwritten (with the default)
	 * due to controller reset from the other port. */
	if (adapter->hw.mac.type == e1000_82571) {
		/* activate the work around */
		e1000_set_laa_state_82571(&adapter->hw, TRUE);

		/* Hold a copy of the LAA in RAR[14] This is done so that
		 * between the time RAR[0] gets clobbered  and the time it
		 * gets fixed (in e1000_watchdog), the actual LAA is in one
		 * of the RARs and no incoming packets directed to this port
		 * are dropped. Eventually the LAA will be in RAR[0] and
		 * RAR[14] */
		e1000_rar_set(&adapter->hw,
			      adapter->hw.mac.addr,
			      adapter->hw.mac.rar_entry_count - 1);
	}

	if (adapter->hw.mac.type == e1000_82542)
		e1000_leave_82542_rst(adapter);

	return 0;
}
#endif

/**
 * e1000_set_multi - Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/
static void e1000_set_multi(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;

	/* Check for Promiscuous and All Multicast modes */

	rctl = E1000_READ_REG(hw, E1000_RCTL);

	if (netdev->flags & IFF_PROMISC) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
	} else if (netdev->flags & IFF_ALLMULTI) {
		rctl |= E1000_RCTL_MPE;
		rctl &= ~E1000_RCTL_UPE;
	} else {
		rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
	}

	E1000_WRITE_REG(hw, E1000_RCTL, rctl);
}

/* Need to wait a few seconds after link up to get diagnostic information from
 * the phy */
static void e1000_update_phy_info_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     phy_info_task.work);
	e1000_get_phy_info(&adapter->hw);
}

/**
 * e1000_82547_tx_fifo_stall_task - task to complete work
 * @work: work struct contained inside adapter struct
 **/
static void e1000_82547_tx_fifo_stall_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     fifo_stall_task.work);
	struct net_device *netdev = adapter->netdev;
	u32 tctl;

	if (atomic_read(&adapter->tx_fifo_stall)) {
		if ((E1000_READ_REG(&adapter->hw, E1000_TDT(0)) ==
		    E1000_READ_REG(&adapter->hw, E1000_TDH(0))) &&
		   (E1000_READ_REG(&adapter->hw, E1000_TDFT) ==
		    E1000_READ_REG(&adapter->hw, E1000_TDFH)) &&
		   (E1000_READ_REG(&adapter->hw, E1000_TDFTS) ==
		    E1000_READ_REG(&adapter->hw, E1000_TDFHS))) {
			tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
			E1000_WRITE_REG(&adapter->hw, E1000_TCTL,
					tctl & ~E1000_TCTL_EN);
			E1000_WRITE_REG(&adapter->hw, E1000_TDFT,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, E1000_TDFH,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, E1000_TDFTS,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, E1000_TDFHS,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);
			E1000_WRITE_FLUSH(&adapter->hw);

			adapter->tx_fifo_head = 0;
			atomic_set(&adapter->tx_fifo_stall, 0);
			rtnetif_wake_queue(netdev);
		} else if (!test_bit(__E1000_DOWN, &adapter->state))
			schedule_delayed_work(&adapter->fifo_stall_task, 1);
	}
}

static bool e1000_has_link(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = FALSE;
	s32 ret_val = 0;

	/* get_link_status is set on LSC (link status) interrupt or
	 * rx sequence error interrupt.  get_link_status will stay
	 * false until the e1000_check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			ret_val = e1000_check_for_link(hw);
			link_active = !hw->mac.get_link_status;
		} else {
			link_active = TRUE;
		}
		break;
	case e1000_media_type_fiber:
		ret_val = e1000_check_for_link(hw);
		link_active = !!(E1000_READ_REG(hw, E1000_STATUS) &
				 E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		ret_val = e1000_check_for_link(hw);
		link_active = adapter->hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (E1000_READ_REG(&adapter->hw, E1000_CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		DPRINTK(LINK, INFO,
			"Gigabit has been disabled, downgrading speed\n");
	}

	return link_active;
}

static void e1000_enable_receives(struct e1000_adapter *adapter)
{
	/* make sure the receive unit is started */
	if ((adapter->flags & E1000_FLAG_RX_NEEDS_RESTART) &&
	    (adapter->flags & E1000_FLAG_RX_RESTART_NOW)) {
		struct e1000_hw *hw = &adapter->hw;
		u32 rctl = E1000_READ_REG(hw, E1000_RCTL);
		E1000_WRITE_REG(hw, E1000_RCTL, rctl | E1000_RCTL_EN);
		adapter->flags &= ~E1000_FLAG_RX_RESTART_NOW;
	}
}

static void e1000_watchdog_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     watchdog_task.work);

	struct net_device *netdev = adapter->netdev;
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_tx_ring *tx_ring;
	u32 link, tctl;
	int i, tx_pending = 0;

	link = e1000_has_link(adapter);
	if ((rtnetif_carrier_ok(netdev)) && link) {
		e1000_enable_receives(adapter);
		goto link_up;
	}

	if (mac->type == e1000_82573) {
		e1000_enable_tx_pkt_filtering(&adapter->hw);
#ifdef NETIF_F_HW_VLAN_TX
		if (adapter->mng_vlan_id != adapter->hw.mng_cookie.vlan_id)
			e1000_update_mng_vlan(adapter);
#endif
	}

	if (link) {
		if (!rtnetif_carrier_ok(netdev)) {
			u32 ctrl;
			bool txb2b = 1;
#ifdef SIOCGMIIPHY
			/* update snapshot of PHY registers on LSC */
			e1000_phy_read_status(adapter);
#endif
			e1000_get_speed_and_duplex(&adapter->hw,
						   &adapter->link_speed,
						   &adapter->link_duplex);

			ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
			DPRINTK(LINK, INFO, "NIC Link is Up %d Mbps %s, "
				"Flow Control: %s\n",
				adapter->link_speed,
				adapter->link_duplex == FULL_DUPLEX ?
				"Full Duplex" : "Half Duplex",
				((ctrl & E1000_CTRL_TFCE) && (ctrl &
				E1000_CTRL_RFCE)) ? "RX/TX" : ((ctrl &
				E1000_CTRL_RFCE) ? "RX" : ((ctrl &
				E1000_CTRL_TFCE) ? "TX" : "None" )));

			/* tweak tx_queue_len according to speed/duplex
			 * and adjust the timeout factor */
			//netdev->tx_queue_len = adapter->tx_queue_len;
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
			case SPEED_10:
				txb2b = 0;
				//netdev->tx_queue_len = 10;
				adapter->tx_timeout_factor = 16;
				break;
			case SPEED_100:
				txb2b = 0;
				//netdev->tx_queue_len = 100;
				/* maybe add some timeout factor ? */
				break;
			}

			if ((mac->type == e1000_82571 ||
			     mac->type == e1000_82572) &&
			    txb2b == 0) {
				u32 tarc0;
				tarc0 = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
				tarc0 &= ~SPEED_MODE_BIT;
				E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc0);
			}

#ifdef NETIF_F_TSO
			/* disable TSO for pcie and 10/100 speeds, to avoid
			 * some hardware issues */
			if (!(adapter->flags & E1000_FLAG_TSO_FORCE) &&
			    adapter->hw.bus.type == e1000_bus_type_pci_express){
				switch (adapter->link_speed) {
				case SPEED_10:
				case SPEED_100:
					DPRINTK(PROBE,INFO,
					"10/100 speed: disabling TSO\n");
					netdev->features &= ~NETIF_F_TSO;
#ifdef NETIF_F_TSO6
					netdev->features &= ~NETIF_F_TSO6;
#endif
					break;
				case SPEED_1000:
					netdev->features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
					netdev->features |= NETIF_F_TSO6;
#endif
					break;
				default:
					/* oops */
					break;
				}
			}
#endif

			/* enable transmits in the hardware, need to do this
			 * after setting TARC0 */
			tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
			tctl |= E1000_TCTL_EN;
			E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

			rtnetif_carrier_on(netdev);
			rtnetif_wake_queue(netdev);
#ifdef CONFIG_E1000_MQ
			if (netif_is_multiqueue(netdev))
				for (i = 0; i < adapter->num_tx_queues; i++)
					netif_wake_subqueue(netdev, i);
#endif

			if (!test_bit(__E1000_DOWN, &adapter->state))
				schedule_delayed_work(&adapter->phy_info_task,
						      2 * HZ);
			adapter->smartspeed = 0;
		}
	} else {
		if (rtnetif_carrier_ok(netdev)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			DPRINTK(LINK, INFO, "NIC Link is Down\n");
			rtnetif_carrier_off(netdev);
			rtnetif_stop_queue(netdev);
			if (!test_bit(__E1000_DOWN, &adapter->state))
				schedule_delayed_work(&adapter->phy_info_task,
						      2 * HZ);

			/* 80003ES2LAN workaround--
			 * For packet buffer work-around on link down event;
			 * disable receives in the ISR and
			 * reset device here in the watchdog
			 */
			if (adapter->flags & E1000_FLAG_RX_NEEDS_RESTART)
				/* reset device */
				schedule_work(&adapter->reset_task);
		}

		e1000_smartspeed(adapter);
	}

link_up:
	e1000_update_stats(adapter);

	mac->tx_packet_delta = adapter->stats.tpt - adapter->tpt_old;
	adapter->tpt_old = adapter->stats.tpt;
	mac->collision_delta = adapter->stats.colc - adapter->colc_old;
	adapter->colc_old = adapter->stats.colc;

	adapter->gorc = adapter->stats.gorc - adapter->gorc_old;
	adapter->gorc_old = adapter->stats.gorc;
	adapter->gotc = adapter->stats.gotc - adapter->gotc_old;
	adapter->gotc_old = adapter->stats.gotc;

	e1000_update_adaptive(&adapter->hw);

	if (!rtnetif_carrier_ok(netdev)) {
		for (i = 0 ; i < adapter->num_tx_queues ; i++) {
			tx_ring = &adapter->tx_ring[i];
			tx_pending |= (E1000_DESC_UNUSED(tx_ring) + 1 <
							       tx_ring->count);
		}
		if (tx_pending) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context). */
			adapter->tx_timeout_count++;
			schedule_work(&adapter->reset_task);
		}
	}

	/* Cause software interrupt to ensure rx ring is cleaned */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_RXDMT0);

	/* Force detection of hung controller every watchdog period */
	adapter->detect_tx_hung = TRUE;

	/* With 82571 controllers, LAA may be overwritten due to controller
	 * reset from the other port. Set the appropriate LAA in RAR[0] */
	if (e1000_get_laa_state_82571(&adapter->hw) == TRUE)
		e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/* Reschedule the task */
	if (!test_bit(__E1000_DOWN, &adapter->state))
		schedule_delayed_work(&adapter->watchdog_task, 2 * HZ);
}

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

/**
 * e1000_update_itr - update the dynamic ITR value based on statistics
 * @adapter: pointer to adapter
 * @itr_setting: current adapter->itr
 * @packets: the number of packets during this measurement interval
 * @bytes: the number of bytes during this measurement interval
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 *      this functionality is controlled by the InterruptThrottleRate module
 *      parameter (see e1000_param.c)
 **/
#if 0
static unsigned int e1000_update_itr(struct e1000_adapter *adapter,
				     u16 itr_setting, int packets,
				     int bytes)
{
	unsigned int retval = itr_setting;

	if (unlikely(!(adapter->flags & E1000_FLAG_HAS_INTR_MODERATION)))
		goto update_itr_done;

	if (packets == 0)
		goto update_itr_done;

	switch (itr_setting) {
	case lowest_latency:
		/* handle TSO and jumbo frames */
		if (bytes/packets > 8000)
			retval = bulk_latency;
		else if ((packets < 5) && (bytes > 512)) {
			retval = low_latency;
		}
		break;
	case low_latency:  /* 50 usec aka 20000 ints/s */
		if (bytes > 10000) {
			/* this if handles the TSO accounting */
			if (bytes/packets > 8000) {
				retval = bulk_latency;
			} else if ((packets < 10) || ((bytes/packets) > 1200)) {
				retval = bulk_latency;
			} else if ((packets > 35)) {
				retval = lowest_latency;
			}
		} else if (bytes/packets > 2000) {
			retval = bulk_latency;
		} else if (packets <= 2 && bytes < 512) {
			retval = lowest_latency;
		}
		break;
	case bulk_latency: /* 250 usec aka 4000 ints/s */
		if (bytes > 25000) {
			if (packets > 35) {
				retval = low_latency;
			}
		} else if (bytes < 6000) {
			retval = low_latency;
		}
		break;
	}

update_itr_done:
	return retval;
}
#endif

static void e1000_set_itr(struct e1000_adapter *adapter)
{
}

#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16

static int e1000_tso(struct e1000_adapter *adapter,
		     struct e1000_tx_ring *tx_ring, struct sk_buff *skb)
{
#ifdef NETIF_F_TSO
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	u32 cmd_length = 0;
	u16 ipcse = 0, tucse, mss;
	u8 ipcss, ipcso, tucss, tucso, hdr_len;
	int err;

	if (skb_is_gso(skb)) {
		if (skb_header_cloned(skb)) {
			err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
			if (err)
				return err;
		}

		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		mss = skb_shinfo(skb)->gso_size;
		if (skb->protocol == htons(ETH_P_IP)) {
			struct iphdr *iph = ip_hdr(skb);
			iph->tot_len = 0;
			iph->check = 0;
			tcp_hdr(skb)->check = ~csum_tcpudp_magic(iph->saddr,
								 iph->daddr, 0,
								 IPPROTO_TCP,
								 0);
			cmd_length = E1000_TXD_CMD_IP;
			ipcse = skb_transport_offset(skb) - 1;
#ifdef NETIF_F_TSO6
		} else if (skb_shinfo(skb)->gso_type == SKB_GSO_TCPV6) {
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check =
				~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
						 &ipv6_hdr(skb)->daddr,
						 0, IPPROTO_TCP, 0);
			ipcse = 0;
#endif
		}
		ipcss = skb_network_offset(skb);
		ipcso = (void *)&(ip_hdr(skb)->check) - (void *)skb->data;
		tucss = skb_transport_offset(skb);
		tucso = (void *)&(tcp_hdr(skb)->check) - (void *)skb->data;
		tucse = 0;

		cmd_length |= (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE |
			       E1000_TXD_CMD_TCP | (skb->len - (hdr_len)));

		i = tx_ring->next_to_use;
		context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];

		context_desc->lower_setup.ip_fields.ipcss  = ipcss;
		context_desc->lower_setup.ip_fields.ipcso  = ipcso;
		context_desc->lower_setup.ip_fields.ipcse  = cpu_to_le16(ipcse);
		context_desc->upper_setup.tcp_fields.tucss = tucss;
		context_desc->upper_setup.tcp_fields.tucso = tucso;
		context_desc->upper_setup.tcp_fields.tucse = cpu_to_le16(tucse);
		context_desc->tcp_seg_setup.fields.mss     = cpu_to_le16(mss);
		context_desc->tcp_seg_setup.fields.hdr_len = hdr_len;
		context_desc->cmd_and_length = cpu_to_le32(cmd_length);

		buffer_info->time_stamp = jiffies;
		buffer_info->next_to_watch = i;

		if (++i == tx_ring->count) i = 0;
		tx_ring->next_to_use = i;

		return TRUE;
	}
#endif

	return FALSE;
}

static bool e1000_tx_csum(struct e1000_adapter *adapter,
			       struct e1000_tx_ring *tx_ring,
			       struct sk_buff *skb)
{
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	// u8 css;
	u32 cmd_len = E1000_TXD_CMD_DEXT;

	if (unlikely(skb->ip_summed != CHECKSUM_PARTIAL))
		return FALSE;

	switch (skb->protocol) {
	case __constant_htons(ETH_P_IP):
		break;
	default:
		if (unlikely(net_ratelimit())) {
			DPRINTK(PROBE, WARNING, "checksum_partial proto=%x!\n",
				skb->protocol);
		}
		break;
	}

	// css = skb_transport_offset(skb);

	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);

	context_desc->lower_setup.ip_config = 0;
	context_desc->cmd_and_length = cpu_to_le32(cmd_len);

	buffer_info->time_stamp = jiffies;
	buffer_info->next_to_watch = i;

	if (unlikely(++i == tx_ring->count)) i = 0;
	tx_ring->next_to_use = i;

	return TRUE;
}

#define E1000_MAX_TXD_PWR	12
#define E1000_MAX_DATA_PER_TXD	(1<<E1000_MAX_TXD_PWR)

static int e1000_tx_map(struct e1000_adapter *adapter,
			struct e1000_tx_ring *tx_ring,
			struct sk_buff *skb, unsigned int first,
			unsigned int max_per_txd, unsigned int nr_frags,
			unsigned int mss)
{
	struct e1000_buffer *buffer_info;
	unsigned int len = skb->len;
	unsigned int offset = 0, size, count = 0, i;
#ifdef MAX_SKB_FRAGS
	unsigned int f;
	len -= skb->data_len;
#endif

	i = tx_ring->next_to_use;

	while (len) {
		buffer_info = &tx_ring->buffer_info[i];
		size = min(len, max_per_txd);
#ifdef NETIF_F_TSO
		/* Workaround for Controller erratum --
		 * descriptor for non-tso packet in a linear SKB that follows a
		 * tso gets written back prematurely before the data is fully
		 * DMA'd to the controller */
		if (tx_ring->last_tx_tso && !skb_is_gso(skb)) {
			tx_ring->last_tx_tso = 0;
			if (!skb->data_len)
				size -= 4;
		}

		/* Workaround for premature desc write-backs
		 * in TSO mode.  Append 4-byte sentinel desc */
		if (unlikely(mss && !nr_frags && size == len && size > 8))
			size -= 4;
#endif
		/* work-around for errata 10 and it applies
		 * to all controllers in PCI-X mode
		 * The fix is to make sure that the first descriptor of a
		 * packet is smaller than 2048 - 16 - 16 (or 2016) bytes
		 */
		if (unlikely((adapter->hw.bus.type == e1000_bus_type_pcix) &&
				(size > 2015) && count == 0))
			size = 2015;

		/* Workaround for potential 82544 hang in PCI-X.  Avoid
		 * terminating buffers within evenly-aligned dwords. */
		if (unlikely(adapter->pcix_82544 &&
		   !((unsigned long)(skb->data + offset + size - 1) & 4) &&
		   size > 4))
			size -= 4;

		buffer_info->length = size;
		/* set time_stamp *before* dma to help avoid a possible race */
		buffer_info->time_stamp = jiffies;
		buffer_info->dma =
			pci_map_single(adapter->pdev,
				skb->data + offset,
				size,
				PCI_DMA_TODEVICE);
		buffer_info->next_to_watch = i;

		len -= size;
		offset += size;
		count++;
		if (unlikely(++i == tx_ring->count)) i = 0;
	}

#ifdef MAX_SKB_FRAGS
	for (f = 0; f < nr_frags; f++) {
		struct skb_frag_struct *frag;

		frag = &skb_shinfo(skb)->frags[f];
		len = frag->size;
		offset = frag->page_offset;

		while (len) {
			buffer_info = &tx_ring->buffer_info[i];
			size = min(len, max_per_txd);
#ifdef NETIF_F_TSO
			/* Workaround for premature desc write-backs
			 * in TSO mode.  Append 4-byte sentinel desc */
			if (unlikely(mss && f == (nr_frags-1) && size == len && size > 8))
				size -= 4;
#endif
			/* Workaround for potential 82544 hang in PCI-X.
			 * Avoid terminating buffers within evenly-aligned
			 * dwords. */
			if (unlikely(adapter->pcix_82544 &&
			   !((unsigned long)(frag->page+offset+size-1) & 4) &&
			   size > 4))
				size -= 4;

			buffer_info->length = size;
			buffer_info->time_stamp = jiffies;
			buffer_info->dma =
				pci_map_page(adapter->pdev,
					frag->page,
					offset,
					size,
					PCI_DMA_TODEVICE);
			buffer_info->next_to_watch = i;

			len -= size;
			offset += size;
			count++;
			if (unlikely(++i == tx_ring->count)) i = 0;
		}
	}
#endif

	i = (i == 0) ? tx_ring->count - 1 : i - 1;
	tx_ring->buffer_info[i].skb = skb;
	tx_ring->buffer_info[first].next_to_watch = i;

	return count;
}

static void e1000_tx_queue(struct e1000_adapter *adapter,
			   struct e1000_tx_ring *tx_ring,
			   int tx_flags, int count, nanosecs_abs_t *xmit_stamp)
{
	struct e1000_tx_desc *tx_desc = NULL;
	struct e1000_buffer *buffer_info;
	u32 txd_upper = 0, txd_lower = E1000_TXD_CMD_IFCS;
	unsigned int i;
    rtdm_lockctx_t context;

	if (likely(tx_flags & E1000_TX_FLAGS_TSO)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D |
			     E1000_TXD_CMD_TSE;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;

		if (likely(tx_flags & E1000_TX_FLAGS_IPV4))
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
	}

	if (likely(tx_flags & E1000_TX_FLAGS_CSUM)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}

	if (unlikely(tx_flags & E1000_TX_FLAGS_VLAN)) {
		txd_lower |= E1000_TXD_CMD_VLE;
		txd_upper |= (tx_flags & E1000_TX_FLAGS_VLAN_MASK);
	}

	i = tx_ring->next_to_use;

	while (count--) {
		buffer_info = &tx_ring->buffer_info[i];
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		tx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);
		tx_desc->lower.data =
			cpu_to_le32(txd_lower | buffer_info->length);
		tx_desc->upper.data = cpu_to_le32(txd_upper);
		if (unlikely(++i == tx_ring->count)) i = 0;
	}

	tx_desc->lower.data |= cpu_to_le32(adapter->txd_cmd);

    rtdm_lock_irqsave(context);

    if (xmit_stamp)
	*xmit_stamp = cpu_to_be64(rtdm_clock_read() + *xmit_stamp);

	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64). */
	wmb();

	tx_ring->next_to_use = i;
	writel(i, adapter->hw.hw_addr + tx_ring->tdt);

    rtdm_lock_irqrestore(context);
	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems */
	mmiowb();
}

#define E1000_FIFO_HDR			0x10
#define E1000_82547_PAD_LEN		0x3E0

/**
 * 82547 workaround to avoid controller hang in half-duplex environment.
 * The workaround is to avoid queuing a large packet that would span
 * the internal Tx FIFO ring boundary by notifying the stack to resend
 * the packet at a later time.  This gives the Tx FIFO an opportunity to
 * flush all packets.  When that occurs, we reset the Tx FIFO pointers
 * to the beginning of the Tx FIFO.
 **/
static int e1000_82547_fifo_workaround(struct e1000_adapter *adapter,
				       struct sk_buff *skb)
{
	u32 fifo_space = adapter->tx_fifo_size - adapter->tx_fifo_head;
	u32 skb_fifo_len = skb->len + E1000_FIFO_HDR;

	skb_fifo_len = ALIGN(skb_fifo_len, E1000_FIFO_HDR);

	if (adapter->link_duplex != HALF_DUPLEX)
		goto no_fifo_stall_required;

	if (atomic_read(&adapter->tx_fifo_stall))
		return 1;

	if (skb_fifo_len >= (E1000_82547_PAD_LEN + fifo_space)) {
		atomic_set(&adapter->tx_fifo_stall, 1);
		return 1;
	}

no_fifo_stall_required:
	adapter->tx_fifo_head += skb_fifo_len;
	if (adapter->tx_fifo_head >= adapter->tx_fifo_size)
		adapter->tx_fifo_head -= adapter->tx_fifo_size;
	return 0;
}

#define MINIMUM_DHCP_PACKET_SIZE 282
static int e1000_transfer_dhcp_info(struct e1000_adapter *adapter,
				    struct sk_buff *skb)
{
	struct e1000_hw *hw =  &adapter->hw;
	u16 length, offset;
#ifdef NETIF_F_HW_VLAN_TX
	if (vlan_tx_tag_present(skb)) {
		if (!((vlan_tx_tag_get(skb) == adapter->hw.mng_cookie.vlan_id)
		    && (adapter->hw.mng_cookie.status &
			E1000_MNG_DHCP_COOKIE_STATUS_VLAN)))
			return 0;
	}
#endif
	if (skb->len > MINIMUM_DHCP_PACKET_SIZE) {
		struct ethhdr *eth = (struct ethhdr *) skb->data;
		if ((htons(ETH_P_IP) == eth->h_proto)) {
			const struct iphdr *ip =
				(struct iphdr *)((u8 *)skb->data+14);
			if (IPPROTO_UDP == ip->protocol) {
				struct udphdr *udp =
					(struct udphdr *)((u8 *)ip +
						(ip->ihl << 2));
				if (ntohs(udp->dest) == 67) {
					offset = (u8 *)udp + 8 - skb->data;
					length = skb->len - offset;

					return e1000_mng_write_dhcp_info(hw,
							(u8 *)udp + 8,
							length);
				}
			}
		}
	}
	return 0;
}

static int __e1000_maybe_stop_tx(struct net_device *netdev,
				 struct e1000_tx_ring *tx_ring, int size)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	rtnetif_stop_queue(netdev);
	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it. */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (likely(E1000_DESC_UNUSED(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! */
	rtnetif_start_queue(netdev);
	++adapter->restart_queue;
	return 0;
}

static int e1000_maybe_stop_tx(struct net_device *netdev,
			       struct e1000_tx_ring *tx_ring, int size)
{
	if (likely(E1000_DESC_UNUSED(tx_ring) >= size))
		return 0;
	return __e1000_maybe_stop_tx(netdev, tx_ring, size);
}

#define TXD_USE_COUNT(S, X) (((S) >> (X)) + 1 )
static int e1000_xmit_frame_ring(struct sk_buff *skb,
				 struct net_device *netdev,
				 struct e1000_tx_ring *tx_ring)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	unsigned int first, max_per_txd = E1000_MAX_DATA_PER_TXD;
	unsigned int max_txd_pwr = E1000_MAX_TXD_PWR;
	unsigned int tx_flags = 0;
	unsigned int len = skb->len;
	unsigned long irq_flags;
	unsigned int nr_frags = 0;
	unsigned int mss = 0;
	int count = 0;
	int tso;
#ifdef MAX_SKB_FRAGS
	unsigned int f;
	len -= skb->data_len;
#endif

	if (test_bit(__E1000_DOWN, &adapter->state)) {
		kfree_rtskb(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb->len <= 0)) {
		kfree_rtskb(skb);
		return NETDEV_TX_OK;
	}


	/* 82571 and newer doesn't need the workaround that limited descriptor
	 * length to 4kB */
	if (adapter->hw.mac.type >= e1000_82571)
		max_per_txd = 8192;

#ifdef NETIF_F_TSO
	mss = skb_shinfo(skb)->gso_size;
	/* The controller does a simple calculation to
	 * make sure there is enough room in the FIFO before
	 * initiating the DMA for each buffer.  The calc is:
	 * 4 = ceil(buffer len/mss).  To make sure we don't
	 * overrun the FIFO, adjust the max buffer len if mss
	 * drops. */
	if (mss) {
		u8 hdr_len;
		max_per_txd = min(mss << 2, max_per_txd);
		max_txd_pwr = fls(max_per_txd) - 1;

		/* TSO Workaround for 82571/2/3 Controllers -- if skb->data
		* points to just header, pull a few bytes of payload from
		* frags into skb->data */
		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		if (skb->data_len && (hdr_len == (skb->len - skb->data_len))) {
			switch (adapter->hw.mac.type) {
				unsigned int pull_size;
			case e1000_82544:
				/* Make sure we have room to chop off 4 bytes,
				 * and that the end alignment will work out to
				 * this hardware's requirements
				 * NOTE: this is a TSO only workaround
				 * if end byte alignment not correct move us
				 * into the next dword */
				if ((unsigned long)(skb_tail_pointer(skb) - 1) & 4)
					break;
				/* fall through */
			case e1000_82571:
			case e1000_82572:
			case e1000_82573:
			case e1000_ich8lan:
			case e1000_ich9lan:
				pull_size = min((unsigned int)4, skb->data_len);
				if (!__pskb_pull_tail(skb, pull_size)) {
					DPRINTK(DRV, ERR,
						"__pskb_pull_tail failed.\n");
					kfree_rtskb(skb);
					return NETDEV_TX_OK;
				}
				len = skb->len - skb->data_len;
				break;
			default:
				/* do nothing */
				break;
			}
		}
	}

	/* reserve a descriptor for the offload context */
	if ((mss) || (skb->ip_summed == CHECKSUM_PARTIAL))
		count++;
	count++;
#else
	if (skb->ip_summed == CHECKSUM_PARTIAL)
		count++;
#endif

#ifdef NETIF_F_TSO
	/* Controller Erratum workaround */
	if (!skb->data_len && tx_ring->last_tx_tso && !skb_is_gso(skb))
		count++;
#endif

	count += TXD_USE_COUNT(len, max_txd_pwr);

	if (adapter->pcix_82544)
		count++;

	/* work-around for errata 10 and it applies to all controllers
	 * in PCI-X mode, so add one more descriptor to the count
	 */
	if (unlikely((adapter->hw.bus.type == e1000_bus_type_pcix) &&
			(len > 2015)))
		count++;

#ifdef MAX_SKB_FRAGS
	nr_frags = skb_shinfo(skb)->nr_frags;
	for (f = 0; f < nr_frags; f++)
		count += TXD_USE_COUNT(skb_shinfo(skb)->frags[f].size,
				       max_txd_pwr);
	if (adapter->pcix_82544)
		count += nr_frags;

#endif

	if (adapter->hw.mac.tx_pkt_filtering &&
	    (adapter->hw.mac.type == e1000_82573))
		e1000_transfer_dhcp_info(adapter, skb);

	rtdm_lock_get_irqsave(&tx_ring->tx_lock, irq_flags);

	/* need: count + 2 desc gap to keep tail from touching
	 * head, otherwise try next time */
	if (unlikely(e1000_maybe_stop_tx(netdev, tx_ring, count + 2))) {
		rtdm_lock_put_irqrestore(&tx_ring->tx_lock, irq_flags);
		rtdm_printk("FATAL: rt_e1000 ran into tail close to head situation!\n");
		return NETDEV_TX_BUSY;
	}

	if (unlikely(adapter->hw.mac.type == e1000_82547)) {
		if (unlikely(e1000_82547_fifo_workaround(adapter, skb))) {
			rtnetif_stop_queue(netdev);
			rtdm_lock_put_irqrestore(&tx_ring->tx_lock, irq_flags);
			if (!test_bit(__E1000_DOWN, &adapter->state))
				schedule_delayed_work(&adapter->fifo_stall_task,
						      1);
		    rtdm_printk("FATAL: rt_e1000 ran into tail 82547 controller bug!\n");
			return NETDEV_TX_BUSY;
		}
	}

#ifndef NETIF_F_LLTX
	rtdm_lock_put_irqrestore(&tx_ring->tx_lock, irq_flags);

#endif
#ifdef NETIF_F_HW_VLAN_TX
	if (unlikely(adapter->vlgrp && vlan_tx_tag_present(skb))) {
		tx_flags |= E1000_TX_FLAGS_VLAN;
		tx_flags |= (vlan_tx_tag_get(skb) << E1000_TX_FLAGS_VLAN_SHIFT);
	}
#endif

	first = tx_ring->next_to_use;

	tso = e1000_tso(adapter, tx_ring, skb);
	if (tso < 0) {
		kfree_rtskb(skb);
#ifdef NETIF_F_LLTX
		rtdm_lock_put_irqrestore(&tx_ring->tx_lock, irq_flags);
#endif
		return NETDEV_TX_OK;
	}

	if (likely(tso)) {
		tx_ring->last_tx_tso = 1;
		tx_flags |= E1000_TX_FLAGS_TSO;
	} else if (likely(e1000_tx_csum(adapter, tx_ring, skb)))
		tx_flags |= E1000_TX_FLAGS_CSUM;

	/* Old method was to assume IPv4 packet by default if TSO was enabled.
	 * 82571 hardware supports TSO capabilities for IPv6 as well...
	 * no longer assume, we must. */
	if (likely(skb->protocol == htons(ETH_P_IP)))
		tx_flags |= E1000_TX_FLAGS_IPV4;

	e1000_tx_queue(adapter, tx_ring, tx_flags,
		       e1000_tx_map(adapter, tx_ring, skb, first,
				    max_per_txd, nr_frags, mss),
		   skb->xmit_stamp);

	// netdev->trans_start = jiffies;

	/* Make sure there is space in the ring for the next send. */
	// e1000_maybe_stop_tx(netdev, tx_ring, MAX_SKB_FRAGS + 2);

#ifdef NETIF_F_LLTX
	rtdm_lock_put_irqrestore(&tx_ring->tx_lock, irq_flags);
#endif
	return NETDEV_TX_OK;
}

static int e1000_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_tx_ring *tx_ring = adapter->tx_ring;

	/* This goes back to the question of how to logically map a tx queue
	 * to a flow.  Right now, performance is impacted slightly negatively
	 * if using multiple tx queues.  If the stack breaks away from a
	 * single qdisc implementation, we can look at this again. */
	return (e1000_xmit_frame_ring(skb, netdev, tx_ring));
}

#ifdef CONFIG_E1000_MQ
static int e1000_subqueue_xmit_frame(struct sk_buff *skb,
				     struct net_device *netdev, int queue)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_tx_ring *tx_ring = &adapter->tx_ring[queue];

	return (e1000_xmit_frame_ring(skb, netdev, tx_ring));
}
#endif


/**
 * e1000_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
#if 0
static void e1000_tx_timeout(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	/* Do the reset outside of interrupt context */
	adapter->tx_timeout_count++;
	schedule_work(&adapter->reset_task);
}
#endif

static void e1000_reset_task(struct work_struct *work)
{
	struct e1000_adapter *adapter;
	adapter = container_of(work, struct e1000_adapter, reset_task);

	e1000_reinit_locked(adapter);
}

#if 0
/**
 * e1000_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats * e1000_get_stats(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	/* only return the current stats */
	return &adapter->net_stats;
}

/**
 * e1000_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int max_frame = new_mtu + ETH_HLEN + ETHERNET_FCS_SIZE;
	u16 eeprom_data = 0;

	if ((max_frame < ETH_ZLEN + ETHERNET_FCS_SIZE) ||
	    (max_frame > MAX_JUMBO_FRAME_SIZE)) {
		DPRINTK(PROBE, ERR, "Invalid MTU setting\n");
		return -EINVAL;
	}

	/* Adapter-specific max frame size limits. */
	switch (adapter->hw.mac.type) {
	case e1000_undefined:
	case e1000_82542:
	case e1000_ich8lan:
		if (max_frame > ETH_FRAME_LEN + ETHERNET_FCS_SIZE) {
			DPRINTK(PROBE, ERR, "Jumbo Frames not supported.\n");
			return -EINVAL;
		}
		break;
	case e1000_82573:
		/* Jumbo Frames not supported if:
		 * - this is not an 82573L device
		 * - ASPM is enabled in any way (0x1A bits 3:2) */
		e1000_read_nvm(&adapter->hw, NVM_INIT_3GIO_3, 1, &eeprom_data);
		if ((adapter->hw.device_id != E1000_DEV_ID_82573L) ||
		    (eeprom_data & NVM_WORD1A_ASPM_MASK)) {
			if (max_frame > ETH_FRAME_LEN + ETHERNET_FCS_SIZE) {
				DPRINTK(PROBE, ERR,
					"Jumbo Frames not supported.\n");
				return -EINVAL;
			}
			break;
		}
		/* ERT will be enabled later to enable wire speed receives */

		/* fall through to get support */
	case e1000_ich9lan:
		if ((adapter->hw.phy.type == e1000_phy_ife) &&
		    (max_frame > ETH_FRAME_LEN + ETHERNET_FCS_SIZE)) {
			DPRINTK(PROBE, ERR, "Jumbo Frames not supported.\n");
			return -EINVAL;
		}
		/* fall through to get support */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
#define MAX_STD_JUMBO_FRAME_SIZE 9234
		if (max_frame > MAX_STD_JUMBO_FRAME_SIZE) {
			DPRINTK(PROBE, ERR, "MTU > 9216 not supported.\n");
			return -EINVAL;
		}
		break;
	default:
		/* Capable of supporting up to MAX_JUMBO_FRAME_SIZE limit. */
		break;
	}

	while (test_and_set_bit(__E1000_RESETTING, &adapter->state))
		msleep(1);
	/* e1000_down has a dependency on max_frame_size */
	adapter->max_frame_size = max_frame;
	if (rtnetif_running(netdev))
		e1000_down(adapter);

	/* NOTE: netdev_alloc_skb reserves 16 bytes, and typically NET_IP_ALIGN
	 * means we reserve 2 more, this pushes us to allocate from the next
	 * larger slab size.
	 * i.e. RXBUFFER_2048 --> size-4096 slab
	 *  however with the new *_jumbo_rx* routines, jumbo receives will use
	 *  fragmented skbs */

	if (max_frame <= E1000_RXBUFFER_256)
		adapter->rx_buffer_len = E1000_RXBUFFER_256;
	else if (max_frame <= E1000_RXBUFFER_512)
		adapter->rx_buffer_len = E1000_RXBUFFER_512;
	else if (max_frame <= E1000_RXBUFFER_1024)
		adapter->rx_buffer_len = E1000_RXBUFFER_1024;
	else if (max_frame <= E1000_RXBUFFER_2048)
		adapter->rx_buffer_len = E1000_RXBUFFER_2048;
#ifdef CONFIG_E1000_NAPI
	else
		adapter->rx_buffer_len = E1000_RXBUFFER_4096;
#else
	else if (max_frame <= E1000_RXBUFFER_4096)
		adapter->rx_buffer_len = E1000_RXBUFFER_4096;
	else if (max_frame <= E1000_RXBUFFER_8192)
		adapter->rx_buffer_len = E1000_RXBUFFER_8192;
	else if (max_frame <= E1000_RXBUFFER_16384)
		adapter->rx_buffer_len = E1000_RXBUFFER_16384;
#endif

	/* adjust allocation if LPE protects us, and we aren't using SBP */
	if (!e1000_tbi_sbp_enabled_82543(&adapter->hw) &&
	    ((max_frame == ETH_FRAME_LEN + ETHERNET_FCS_SIZE) ||
	     (max_frame == MAXIMUM_ETHERNET_VLAN_SIZE)))
		adapter->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;

	DPRINTK(PROBE, INFO, "changing MTU from %d to %d\n",
		netdev->mtu, new_mtu);
	netdev->mtu = new_mtu;

	if (rtnetif_running(netdev))
		e1000_up(adapter);
	else
		e1000_reset(adapter);

	clear_bit(__E1000_RESETTING, &adapter->state);

	return 0;
}
#endif

/**
 * e1000_update_stats - Update the board statistics counters
 * @adapter: board private structure
 **/
void e1000_update_stats(struct e1000_adapter *adapter)
{
}
#ifdef SIOCGMIIPHY

/**
 * e1000_phy_read_status - Update the PHY register status snapshot
 * @adapter: board private structure
 **/
static void e1000_phy_read_status(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_phy_regs *phy = &adapter->phy_regs;
	int ret_val = E1000_SUCCESS;
	unsigned long irq_flags;


	rtdm_lock_get_irqsave(&adapter->stats_lock, irq_flags);

	if (E1000_READ_REG(hw, E1000_STATUS)& E1000_STATUS_LU) {
		ret_val = e1000_read_phy_reg(hw, PHY_CONTROL, &phy->bmcr);
		ret_val |= e1000_read_phy_reg(hw, PHY_STATUS, &phy->bmsr);
		ret_val |= e1000_read_phy_reg(hw, PHY_AUTONEG_ADV,
					      &phy->advertise);
		ret_val |= e1000_read_phy_reg(hw, PHY_LP_ABILITY, &phy->lpa);
		ret_val |= e1000_read_phy_reg(hw, PHY_AUTONEG_EXP,
					      &phy->expansion);
		ret_val |= e1000_read_phy_reg(hw, PHY_1000T_CTRL,
					      &phy->ctrl1000);
		ret_val |= e1000_read_phy_reg(hw, PHY_1000T_STATUS,
					      &phy->stat1000);
		ret_val |= e1000_read_phy_reg(hw, PHY_EXT_STATUS,
					      &phy->estatus);
		if (ret_val)
			DPRINTK(DRV, WARNING, "Error reading PHY register\n");
	} else {
		/* Do not read PHY registers if link is not up
		 * Set values to typical power-on defaults */
		phy->bmcr = (BMCR_SPEED1000 | BMCR_ANENABLE | BMCR_FULLDPLX);
		phy->bmsr = (BMSR_100FULL | BMSR_100HALF | BMSR_10FULL |
			     BMSR_10HALF | BMSR_ESTATEN | BMSR_ANEGCAPABLE |
			     BMSR_ERCAP);
		phy->advertise = (ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
				  ADVERTISE_ALL | ADVERTISE_CSMA);
		phy->lpa = 0;
		phy->expansion = EXPANSION_ENABLENPAGE;
		phy->ctrl1000 = ADVERTISE_1000FULL;
		phy->stat1000 = 0;
		phy->estatus = (ESTATUS_1000_TFULL | ESTATUS_1000_THALF);
	}

	rtdm_lock_put_irqrestore(&adapter->stats_lock, irq_flags);
}
#endif


/**
 * e1000_intr_msi - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static int e1000_intr_msi(rtdm_irq_t *irq_handle)
{
    struct rtnet_device *netdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
#ifndef CONFIG_E1000_NAPI
	int i, j;
	int rx_cleaned, tx_cleaned;
#endif
	u32 icr = E1000_READ_REG(hw, E1000_ICR);
    nanosecs_abs_t time_stamp = rtdm_clock_read();



#ifdef CONFIG_E1000_NAPI
	/* read ICR disables interrupts using IAM, so keep up with our
	 * enable/disable accounting */
	atomic_inc(&adapter->irq_sem);
#endif
	if (icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		hw->mac.get_link_status = 1;
		/* ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers */
		if ((hw->mac.type == e1000_ich8lan) &&
		    (hw->phy.type == e1000_phy_igp_3) &&
		    (!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)))
			e1000_gig_downshift_workaround_ich8lan(hw);

		/* 80003ES2LAN workaround-- For packet buffer work-around on
		 * link down event; disable receives here in the ISR and reset
		 * adapter in watchdog */
		if (rtnetif_carrier_ok(netdev) &&
		    (adapter->flags & E1000_FLAG_RX_NEEDS_RESTART)) {
			/* disable receives */
			u32 rctl = E1000_READ_REG(hw, E1000_RCTL);
			E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
			adapter->flags |= E1000_FLAG_RX_RESTART_NOW;
		}
		/* guard against interrupt when we're going down */
		//if (!test_bit(__E1000_DOWN, &adapter->state))
		//	mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

#ifdef CONFIG_E1000_NAPI
	/* XXX only using ring 0 for napi */
	if (likely(netif_rx_schedule_prep(netdev, &adapter->rx_ring[0].napi))) {
		adapter->total_tx_bytes = 0;
		adapter->total_tx_packets = 0;
		adapter->total_rx_bytes = 0;
		adapter->total_rx_packets = 0;
		__netif_rx_schedule(netdev, &adapter->rx_ring[0].napi);
	} else {
		atomic_dec(&adapter->irq_sem);
	}
#else
	adapter->total_tx_bytes = 0;
	adapter->total_rx_bytes = 0;
	adapter->total_tx_packets = 0;
	adapter->total_rx_packets = 0;
    adapter->data_received = 0;

	for (i = 0; i < E1000_MAX_INTR; i++) {
		rx_cleaned = 0;
		for (j = 0; j < adapter->num_rx_queues; j++)
			rx_cleaned |= adapter->clean_rx(adapter,
							&adapter->rx_ring[j], &time_stamp);

		tx_cleaned = 0;
		for (j = 0 ; j < adapter->num_tx_queues ; j++)
			tx_cleaned |= e1000_clean_tx_irq(adapter,
							 &adapter->tx_ring[j]);

		if (!rx_cleaned && !tx_cleaned)
			break;
	}

	if (likely(adapter->itr_setting & 3))
		e1000_set_itr(adapter);
#endif

	if (adapter->data_received)
		rt_mark_stack_mgr(netdev);

	return RTDM_IRQ_HANDLED;
}

/**
 * e1000_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static int e1000_intr(rtdm_irq_t *irq_handle)
{
	struct rtnet_device *netdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl, icr = E1000_READ_REG(hw, E1000_ICR);
#ifndef CONFIG_E1000_NAPI
	int i, j;
	int rx_cleaned, tx_cleaned;
#endif
    nanosecs_abs_t time_stamp = rtdm_clock_read();
	if (unlikely(!icr))
		return RTDM_IRQ_NONE;  /* Not our interrupt */

#ifdef CONFIG_E1000_NAPI
	/* IMS will not auto-mask if INT_ASSERTED is not set, and if it is
	 * not set, then the adapter didn't send an interrupt */
	if ((adapter->flags & E1000_FLAG_INT_ASSERT_AUTO_MASK) &&
	    !(icr & E1000_ICR_INT_ASSERTED))
		return IRQ_NONE;

	/* Interrupt Auto-Mask...upon reading ICR,
	 * interrupts are masked.  No need for the
	 * IMC write, but it does mean we should
	 * account for it ASAP. */
	if (likely(hw->mac.type >= e1000_82571))
		atomic_inc(&adapter->irq_sem);
#endif

	if (unlikely(icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC))) {
		hw->mac.get_link_status = 1;
		/* ICH8 workaround-- Call gig speed drop workaround on cable
		 * disconnect (LSC) before accessing any PHY registers */
		if ((hw->mac.type == e1000_ich8lan) &&
		    (hw->phy.type == e1000_phy_igp_3) &&
		    (!(E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)))
			e1000_gig_downshift_workaround_ich8lan(hw);

		/* 80003ES2LAN workaround--
		 * For packet buffer work-around on link down event;
		 * disable receives here in the ISR and
		 * reset adapter in watchdog
		 */
		if (rtnetif_carrier_ok(netdev) &&
		    (adapter->flags & E1000_FLAG_RX_NEEDS_RESTART)) {
			/* disable receives */
			rctl = E1000_READ_REG(hw, E1000_RCTL);
			E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
			adapter->flags |= E1000_FLAG_RX_RESTART_NOW;
		}
		/* guard against interrupt when we're going down */
		//if (!test_bit(__E1000_DOWN, &adapter->state))
		//	mod_timer(&adapter->watchdog_timer, jiffies + 1);
	}

#ifdef CONFIG_E1000_NAPI
	if (hw->mac.type < e1000_82571) {
		/* disable interrupts, without the synchronize_irq bit */
		atomic_inc(&adapter->irq_sem);
		E1000_WRITE_REG(hw, E1000_IMC, ~0);
		E1000_WRITE_FLUSH(hw);
	}
	/* XXX only using ring 0 for napi */
	if (likely(netif_rx_schedule_prep(netdev, &adapter->rx_ring[0].napi))) {
		adapter->total_tx_bytes = 0;
		adapter->total_tx_packets = 0;
		adapter->total_rx_bytes = 0;
		adapter->total_rx_packets = 0;
		__netif_rx_schedule(netdev, &adapter->rx_ring[0].napi);
	} else {
		atomic_dec(&adapter->irq_sem);
	}
#else
	/* Writing IMC and IMS is needed for 82547.
	 * Due to Hub Link bus being occupied, an interrupt
	 * de-assertion message is not able to be sent.
	 * When an interrupt assertion message is generated later,
	 * two messages are re-ordered and sent out.
	 * That causes APIC to think 82547 is in de-assertion
	 * state, while 82547 is in assertion state, resulting
	 * in dead lock. Writing IMC forces 82547 into
	 * de-assertion state.
	 */
	if (hw->mac.type == e1000_82547 || hw->mac.type == e1000_82547_rev_2) {
		atomic_inc(&adapter->irq_sem);
		E1000_WRITE_REG(hw, E1000_IMC, ~0);
	}

    adapter->data_received = 0;
	adapter->total_tx_bytes = 0;
	adapter->total_rx_bytes = 0;
	adapter->total_tx_packets = 0;
	adapter->total_rx_packets = 0;

	for (i = 0; i < E1000_MAX_INTR; i++) {
		rx_cleaned = 0;
		for (j = 0; j < adapter->num_rx_queues; j++)
			rx_cleaned |= adapter->clean_rx(adapter,
							&adapter->rx_ring[j], &time_stamp);

		tx_cleaned = 0;
		for (j = 0 ; j < adapter->num_tx_queues ; j++)
			tx_cleaned |= e1000_clean_tx_irq(adapter,
							 &adapter->tx_ring[j]);

		if (!rx_cleaned && !tx_cleaned)
			break;
	}

	if (likely(adapter->itr_setting & 3))
		e1000_set_itr(adapter);

	if (hw->mac.type == e1000_82547 || hw->mac.type == e1000_82547_rev_2)
		e1000_irq_enable(adapter);

#endif

	if (adapter->data_received)
		rt_mark_stack_mgr(netdev);
	return RTDM_IRQ_HANDLED;
}

#ifdef CONFIG_E1000_NAPI
/**
 * e1000_poll - NAPI Rx polling callback
 * @napi: struct associated with this polling callback
 * @budget: amount of packets driver is allowed to process this poll
 **/
static int e1000_poll(struct napi_struct *napi, int budget)
{
	struct e1000_rx_ring *rx_ring = container_of(napi, struct e1000_rx_ring,
						     napi);
	struct e1000_adapter *adapter = rx_ring->adapter;
	struct net_device *netdev = adapter->netdev;
	int tx_clean_complete = 1, work_done = 0;
	int i;

	/* FIXME: i think this code is un-necessary when using base netdev */
	/* Keep link state information with original netdev */
	if (!rtnetif_carrier_ok(netdev))
		goto quit_polling;

	/* e1000_poll is called per-cpu.  This lock protects
	 * tx_ring[i] from being cleaned by multiple cpus
	 * simultaneously.  A failure obtaining the lock means
	 * tx_ring[i] is currently being cleaned anyway. */
	for (i = 0; i < adapter->num_tx_queues; i++) {
#ifdef CONFIG_E1000_MQ
		if (spin_trylock(&adapter->tx_ring[i].tx_queue_lock)) {
			tx_clean_complete &= e1000_clean_tx_irq(adapter,
							&adapter->tx_ring[i]);
			spin_unlock(&adapter->tx_ring[i].tx_queue_lock);
		}
#else
		if (spin_trylock(&adapter->tx_queue_lock)) {
			tx_clean_complete &= e1000_clean_tx_irq(adapter,
							&adapter->tx_ring[i]);
			spin_unlock(&adapter->tx_queue_lock);
		}
#endif
	}

	for (i = 0; i < adapter->num_rx_queues; i++) {
		adapter->clean_rx(adapter, &adapter->rx_ring[i],
				  &work_done, budget);
	}

	/* If no Tx and not enough Rx work done, exit the polling mode */
	if ((tx_clean_complete && (work_done == 0)) ||
	   !rtnetif_running(netdev)) {
quit_polling:
		if (likely(adapter->itr_setting & 3))
			e1000_set_itr(adapter);
		netif_rx_complete(netdev, napi);
		if (test_bit(__E1000_DOWN, &adapter->state))
			atomic_dec(&adapter->irq_sem);
		else
			e1000_irq_enable(adapter);
		return 0;
	}

	/* need to make sure the stack is aware of a tx-only poll loop */
	if (!tx_clean_complete)
		work_done = budget;

	return work_done;
}

#endif
/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
static bool e1000_clean_tx_irq(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
#ifdef CONFIG_E1000_NAPI
	unsigned int count = 0;
#endif
	bool cleaned = FALSE;
	bool retval = TRUE;
	unsigned int total_tx_bytes=0, total_tx_packets=0;


	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);

	while (eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) {
		for (cleaned = FALSE; !cleaned; ) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);

#ifdef CONFIG_E1000_MQ
			tx_ring->tx_stats.bytes += buffer_info->length;
#endif
			if (cleaned) {
				struct sk_buff *skb = buffer_info->skb;
#ifdef NETIF_F_TSO
				unsigned int segs, bytecount;
				segs = skb_shinfo(skb)->gso_segs ?: 1;
				/* multiply data chunks by size of headers */
				bytecount = ((segs - 1) * skb_headlen(skb)) +
					    skb->len;
				total_tx_packets += segs;
				total_tx_bytes += bytecount;
#else
				total_tx_packets++;
				total_tx_bytes += skb->len;
#endif
			}
			e1000_unmap_and_free_tx_resource(adapter, buffer_info);
			tx_desc->upper.data = 0;

			if (unlikely(++i == tx_ring->count)) i = 0;
		}

#ifdef CONFIG_E1000_MQ
		tx_ring->tx_stats.packets++;
#endif
		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
#ifdef CONFIG_E1000_NAPI
#define E1000_TX_WEIGHT 64
		/* weight of a sort for tx, to avoid endless transmit cleanup */
		if (count++ == E1000_TX_WEIGHT) {
			retval = FALSE;
			break;
		}
#endif
	}

	tx_ring->next_to_clean = i;

#define TX_WAKE_THRESHOLD 32
	if (unlikely(cleaned && rtnetif_carrier_ok(netdev) &&
		     E1000_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();

		if (rtnetif_queue_stopped(netdev) &&
		    !(test_bit(__E1000_DOWN, &adapter->state))) {
			rtnetif_wake_queue(netdev);
			++adapter->restart_queue;
		}
	}

	if (adapter->detect_tx_hung) {
		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i */
		adapter->detect_tx_hung = FALSE;
		if (tx_ring->buffer_info[eop].dma &&
		    time_after(jiffies, tx_ring->buffer_info[eop].time_stamp +
			       (adapter->tx_timeout_factor * HZ))
		    && !(E1000_READ_REG(&adapter->hw, E1000_STATUS) &
			 E1000_STATUS_TXOFF)) {

			/* detected Tx unit hang */
			DPRINTK(DRV, ERR, "Detected Tx Unit Hang\n"
					"  Tx Queue             <%lu>\n"
					"  TDH                  <%x>\n"
					"  TDT                  <%x>\n"
					"  next_to_use          <%x>\n"
					"  next_to_clean        <%x>\n"
					"buffer_info[next_to_clean]\n"
					"  time_stamp           <%lx>\n"
					"  next_to_watch        <%x>\n"
					"  jiffies              <%lx>\n"
					"  next_to_watch.status <%x>\n",
				(unsigned long)((tx_ring - adapter->tx_ring) /
					sizeof(struct e1000_tx_ring)),
				readl(adapter->hw.hw_addr + tx_ring->tdh),
				readl(adapter->hw.hw_addr + tx_ring->tdt),
				tx_ring->next_to_use,
				tx_ring->next_to_clean,
				tx_ring->buffer_info[eop].time_stamp,
				eop,
				jiffies,
				eop_desc->upper.fields.status);
			rtnetif_stop_queue(netdev);
		}
	}
	adapter->total_tx_bytes += total_tx_bytes;
	adapter->total_tx_packets += total_tx_packets;
	adapter->net_stats.tx_bytes += total_tx_bytes;
	adapter->net_stats.tx_packets += total_tx_packets;
	return retval;
}

/**
 * e1000_rx_checksum - Receive Checksum Offload for 82543
 * @adapter:     board private structure
 * @status_err:  receive descriptor status and error fields
 * @csum:        receive descriptor csum field
 * @sk_buff:     socket buffer with received data
 **/
static void e1000_rx_checksum(struct e1000_adapter *adapter, u32 status_err,
			      u32 csum, struct sk_buff *skb)
{
	u16 status = (u16)status_err;
	u8 errors = (u8)(status_err >> 24);
	skb->ip_summed = CHECKSUM_NONE;

	/* 82543 or newer only */
	if (unlikely(adapter->hw.mac.type < e1000_82543)) return;
	/* Ignore Checksum bit is set */
	if (unlikely(status & E1000_RXD_STAT_IXSM)) return;
	/* TCP/UDP checksum error bit is set */
	if (unlikely(errors & E1000_RXD_ERR_TCPE)) {
		/* let the stack verify checksum errors */
		adapter->hw_csum_err++;
		return;
	}
	/* TCP/UDP Checksum has not been calculated */
	if (adapter->hw.mac.type <= e1000_82547_rev_2) {
		if (!(status & E1000_RXD_STAT_TCPCS))
			return;
	} else {
		if (!(status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)))
			return;
	}
	/* It must be a TCP or UDP packet with a valid checksum */
	if (likely(status & E1000_RXD_STAT_TCPCS)) {
		/* TCP checksum is good */
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	} else if (adapter->hw.mac.type > e1000_82547_rev_2) {
		/* IP fragment with UDP payload */
		/* Hardware complements the payload checksum, so we undo it
		 * and then put the value in host order for further stack use.
		 */
		csum = ntohl(csum ^ 0xFFFF);
		skb->csum = csum;
		skb->ip_summed = CHECKSUM_COMPLETE;
	}
	adapter->hw_csum_good++;
}

/**
 * e1000_receive_skb - helper function to handle rx indications
 * @adapter: board private structure
 * @status: descriptor status field as written by hardware
 * @vlan: descriptor vlan field as written by hardware (no le/be conversion)
 * @skb: pointer to sk_buff to be indicated to stack
 **/
static void e1000_receive_skb(struct e1000_adapter *adapter, u8 status,
			      u16 vlan, struct sk_buff *skb)
{
#ifdef CONFIG_E1000_NAPI
#ifdef NETIF_F_HW_VLAN_TX
	if (unlikely(adapter->vlgrp && (status & E1000_RXD_STAT_VP))) {
		vlan_hwaccel_receive_skb(skb, adapter->vlgrp,
					 le16_to_cpu(vlan) &
					 E1000_RXD_SPC_VLAN_MASK);
	} else {
		netif_receive_skb(skb);
	}
#else
	netif_receive_skb(skb);
#endif
#else /* CONFIG_E1000_NAPI */
#ifdef NETIF_F_HW_VLAN_TX
	if (unlikely(adapter->vlgrp && (status & E1000_RXD_STAT_VP))) {
		vlan_hwaccel_rx(skb, adapter->vlgrp,
				le16_to_cpu(vlan) & E1000_RXD_SPC_VLAN_MASK);
	} else {
		netif_rx(skb);
	}
#else
	rtnetif_rx(skb);
#endif
#endif /* CONFIG_E1000_NAPI */
}

#ifdef CONFIG_E1000_NAPI
/* NOTE: these new jumbo frame routines rely on NAPI because of the
 * pskb_may_pull call, which eventually must call kmap_atomic which you cannot
 * call from hard irq context */

/**
 * e1000_consume_page - helper function
 **/
static void e1000_consume_page(struct e1000_rx_buffer *bi, struct sk_buff *skb,
			       u16 length)
{
	bi->page = NULL;
	skb->len += length;
	skb->data_len += length;
	skb->truesize += length;
}

/**
 * e1000_clean_jumbo_rx_irq - Send received data up the network stack; legacy
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
static bool e1000_clean_jumbo_rx_irq(struct e1000_adapter *adapter,
					  struct e1000_rx_ring *rx_ring,
					  int *work_done, int work_to_do)
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc, *next_rxd;
	struct e1000_rx_buffer *buffer_info, *next_buffer;
	unsigned long irq_flags;
	u32 length;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = FALSE;
	unsigned int total_rx_bytes=0, total_rx_packets=0;

	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];

	while (rx_desc->status & E1000_RXD_STAT_DD) {
		struct sk_buff *skb;
		u8 status;

		if (*work_done >= work_to_do)
			break;
		(*work_done)++;

		status = rx_desc->status;
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];

		cleaned = TRUE;
		cleaned_count++;
		pci_unmap_page(pdev,
			       buffer_info->dma,
			       PAGE_SIZE,
			       PCI_DMA_FROMDEVICE);
		buffer_info->dma = 0;

		length = le16_to_cpu(rx_desc->length);

		/* errors is only valid for DD + EOP descriptors */
		if (unlikely((status & E1000_RXD_STAT_EOP) &&
		    (rx_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK))) {
			u8 last_byte = *(skb->data + length - 1);
			if (TBI_ACCEPT(&adapter->hw, status,
				      rx_desc->errors, length, last_byte,
				      adapter->min_frame_size,
				      adapter->max_frame_size)) {
				rtdm_lock_get_irqsave(&adapter->stats_lock,
						  irq_flags);
				e1000_tbi_adjust_stats_82543(&adapter->hw,
						      &adapter->stats,
						      length, skb->data,
						      adapter->max_frame_size);
				rtdm_lock_put_irqrestore(&adapter->stats_lock,
						       irq_flags);
				length--;
			} else {
				/* recycle both page and skb */
				buffer_info->skb = skb;
				/* an error means any chain goes out the window
				 * too */
				if (rx_ring->rx_skb_top)
					kfree_rtskb(rx_ring->rx_skb_top);
				rx_ring->rx_skb_top = NULL;
				goto next_desc;
			}
		}

#define rxtop rx_ring->rx_skb_top
		if (!(status & E1000_RXD_STAT_EOP)) {
			/* this descriptor is only the beginning (or middle) */
			if (!rxtop) {
				/* this is the beginning of a chain */
				rxtop = skb;
				skb_fill_page_desc(rxtop, 0, buffer_info->page,
						   0, length);
			} else {
				/* this is the middle of a chain */
				skb_fill_page_desc(rxtop,
				    skb_shinfo(rxtop)->nr_frags,
				    buffer_info->page, 0, length);
				/* re-use the skb, only consumed the page */
				buffer_info->skb = skb;
			}
			e1000_consume_page(buffer_info, rxtop, length);
			goto next_desc;
		} else {
			if (rxtop) {
				/* end of the chain */
				skb_fill_page_desc(rxtop,
				    skb_shinfo(rxtop)->nr_frags,
				    buffer_info->page, 0, length);
				/* re-use the current skb, we only consumed the
				 * page */
				buffer_info->skb = skb;
				skb = rxtop;
				rxtop = NULL;
				e1000_consume_page(buffer_info, skb, length);
			} else {
				/* no chain, got EOP, this buf is the packet
				 * copybreak to save the put_page/alloc_page */
				if (length <= copybreak &&
				    skb_tailroom(skb) >= length) {
					u8 *vaddr;
					vaddr = kmap_atomic(buffer_info->page,
							   KM_SKB_DATA_SOFTIRQ);
					memcpy(skb_tail_pointer(skb), vaddr, length);
					kunmap_atomic(vaddr,
						      KM_SKB_DATA_SOFTIRQ);
					/* re-use the page, so don't erase
					 * buffer_info->page */
					rtskb_put(skb, length);
				} else {
					skb_fill_page_desc(skb, 0,
							   buffer_info->page, 0,
							   length);
					e1000_consume_page(buffer_info, skb,
							   length);
				}
			}
		}

		/* Receive Checksum Offload XXX recompute due to CRC strip? */
		e1000_rx_checksum(adapter,
				  (u32)(status) |
				  ((u32)(rx_desc->errors) << 24),
				  le16_to_cpu(rx_desc->csum), skb);

		pskb_trim(skb, skb->len - 4);

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;
		total_rx_packets++;

		/* eth type trans needs skb->data to point to something */
		if (!pskb_may_pull(skb, ETH_HLEN)) {
			DPRINTK(DRV, ERR, "__pskb_pull_tail failed.\n");
			kfree_rtskb(skb);
			goto next_desc;
		}

		skb->protocol = rt_eth_type_trans(skb, netdev);

		e1000_receive_skb(adapter, status, rx_desc->special, skb);
	adapter->data_received = 1; // Set flag for the main interrupt routine

		netdev->last_rx = jiffies;
#ifdef CONFIG_E1000_MQ
		rx_ring->rx_stats.packets++;
		rx_ring->rx_stats.bytes += length;
#endif

next_desc:
		rx_desc->status = 0;

		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= E1000_RX_BUFFER_WRITE)) {
			adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	rx_ring->next_to_clean = i;

	cleaned_count = E1000_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);

	adapter->total_rx_packets += total_rx_packets;
	adapter->total_rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_packets += total_rx_packets;
	return cleaned;
}
#endif /* NAPI */


/**
 * e1000_clean_rx_irq - Send received data up the network stack; legacy
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
#ifdef CONFIG_E1000_NAPI
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring,
				    int *work_done, int work_to_do)
#else
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring,
				    nanosecs_abs_t *time_stamp)
#endif
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc, *next_rxd;
	struct e1000_rx_buffer *buffer_info, *next_buffer;
	u32 length;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = FALSE;
	unsigned int total_rx_bytes=0, total_rx_packets=0;

	// rtdm_printk("<2> e1000_clean_rx_irq %i\n", __LINE__);

	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];

	while (rx_desc->status & E1000_RXD_STAT_DD) {
		struct sk_buff *skb;
		u8 status;

#ifdef CONFIG_E1000_NAPI
		if (*work_done >= work_to_do)
			break;
		(*work_done)++;
#endif
		status = rx_desc->status;
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		prefetch(skb->data - NET_IP_ALIGN);

		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];

		cleaned = TRUE;
		cleaned_count++;
		pci_unmap_single(pdev,
				 buffer_info->dma,
				 adapter->rx_buffer_len,
				 PCI_DMA_FROMDEVICE);
		buffer_info->dma = 0;

		length = le16_to_cpu(rx_desc->length);

		/* !EOP means multiple descriptors were used to store a single
		 * packet, also make sure the frame isn't just CRC only */
		if (unlikely(!(status & E1000_RXD_STAT_EOP) || (length <= 4))) {
			/* All receives must fit into a single buffer */
			E1000_DBG("%s: Receive packet consumed multiple"
				  " buffers\n", netdev->name);
			/* recycle */
			buffer_info->skb = skb;
			goto next_desc;
		}

		if (unlikely(rx_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK)) {
			u8 last_byte = *(skb->data + length - 1);
			if (TBI_ACCEPT(&adapter->hw, status,
				      rx_desc->errors, length, last_byte,
				      adapter->min_frame_size,
				      adapter->max_frame_size)) {
				length--;
			} else {
				/* recycle */
				buffer_info->skb = skb;
				goto next_desc;
			}
		}

		/* adjust length to remove Ethernet CRC, this must be
		 * done after the TBI_ACCEPT workaround above */
		length -= 4;

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += length;
		total_rx_packets++;

		rtskb_put(skb, length);

		/* Receive Checksum Offload */
		e1000_rx_checksum(adapter,
				  (u32)(status) |
				  ((u32)(rx_desc->errors) << 24),
				  le16_to_cpu(rx_desc->csum), skb);

		skb->protocol = rt_eth_type_trans(skb, netdev);
	skb->time_stamp = *time_stamp;

		e1000_receive_skb(adapter, status, rx_desc->special, skb);
	adapter->data_received = 1; // Set flag for the main interrupt routine

		// netdev->last_rx = jiffies;
#ifdef CONFIG_E1000_MQ
		rx_ring->rx_stats.packets++;
		rx_ring->rx_stats.bytes += length;
#endif

next_desc:
		rx_desc->status = 0;

		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= E1000_RX_BUFFER_WRITE)) {
			adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	rx_ring->next_to_clean = i;

	cleaned_count = E1000_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);

	adapter->total_rx_packets += total_rx_packets;
	adapter->total_rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_packets += total_rx_packets;
	return cleaned;
}

/**
 * e1000_clean_rx_irq_ps - Send received data up the network stack; packet split
 * @adapter: board private structure
 *
 * the return value indicates whether actual cleaning was done, there
 * is no guarantee that everything was cleaned
 **/
#ifdef CONFIG_E1000_NAPI
static bool e1000_clean_rx_irq_ps(struct e1000_adapter *adapter,
				       struct e1000_rx_ring *rx_ring,
				       int *work_done, int work_to_do)
#else
static bool e1000_clean_rx_irq_ps(struct e1000_adapter *adapter,
				       struct e1000_rx_ring *rx_ring,
				       nanosecs_abs_t *time_stamp)
#endif
{
#ifdef CONFIG_E1000_DISABLE_PACKET_SPLIT
    return true;

#else

	union e1000_rx_desc_packet_split *rx_desc, *next_rxd;
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_buffer *buffer_info, *next_buffer;
	struct e1000_ps_page *ps_page;
	struct e1000_ps_page_dma *ps_page_dma;
	struct sk_buff *skb;
	unsigned int i, j;
	u32 length, staterr;
	int cleaned_count = 0;
	bool cleaned = FALSE;
	unsigned int total_rx_bytes=0, total_rx_packets=0;

	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC_PS(*rx_ring, i);
	staterr = le32_to_cpu(rx_desc->wb.middle.status_error);
	buffer_info = &rx_ring->buffer_info[i];

	while (staterr & E1000_RXD_STAT_DD) {
		ps_page = &rx_ring->ps_page[i];
		ps_page_dma = &rx_ring->ps_page_dma[i];
#ifdef CONFIG_E1000_NAPI
		if (unlikely(*work_done >= work_to_do))
			break;
		(*work_done)++;
#endif
		skb = buffer_info->skb;

		/* in the packet split case this is header only */
		prefetch(skb->data - NET_IP_ALIGN);

		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC_PS(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];

		cleaned = TRUE;
		cleaned_count++;
		pci_unmap_single(pdev, buffer_info->dma,
				 adapter->rx_ps_bsize0,
				 PCI_DMA_FROMDEVICE);
		buffer_info->dma = 0;

		if (unlikely(!(staterr & E1000_RXD_STAT_EOP))) {
			E1000_DBG("%s: Packet Split buffers didn't pick up"
				  " the full packet\n", netdev->name);
			dev_kfree_skb_irq(skb);
			goto next_desc;
		}

		if (unlikely(staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK)) {
			dev_kfree_skb_irq(skb);
			goto next_desc;
		}

		length = le16_to_cpu(rx_desc->wb.middle.length0);

		if (unlikely(!length)) {
			E1000_DBG("%s: Last part of the packet spanning"
				  " multiple descriptors\n", netdev->name);
			dev_kfree_skb_irq(skb);
			goto next_desc;
		}

		/* Good Receive */
		rtskb_put(skb, length);
#ifdef CONFIG_E1000_MQ
		rx_ring->rx_stats.packets++;
		rx_ring->rx_stats.bytes += skb->len;
#endif

#ifdef CONFIG_E1000_NAPI
		{
		/* this looks ugly, but it seems compiler issues make it
		   more efficient than reusing j */
		int l1 = le16_to_cpu(rx_desc->wb.upper.length[0]);

		/* page alloc/put takes too long and effects small packet
		 * throughput, so unsplit small packets and save the alloc/put
		 * only valid in softirq (napi) context to call kmap_* */
		if (l1 && (l1 <= copybreak) &&
		    ((length + l1) <= adapter->rx_ps_bsize0)) {
			u8 *vaddr;
			/* there is no documentation about how to call
			 * kmap_atomic, so we can't hold the mapping
			 * very long */
			pci_dma_sync_single_for_cpu(pdev,
				ps_page_dma->ps_page_dma[0],
				PAGE_SIZE,
				PCI_DMA_FROMDEVICE);
			vaddr = kmap_atomic(ps_page->ps_page[0],
					    KM_SKB_DATA_SOFTIRQ);
			memcpy(skb_tail_pointer(skb), vaddr, l1);
			kunmap_atomic(vaddr, KM_SKB_DATA_SOFTIRQ);
			pci_dma_sync_single_for_device(pdev,
				ps_page_dma->ps_page_dma[0],
				PAGE_SIZE, PCI_DMA_FROMDEVICE);
			/* remove the CRC */
			l1 -= 4;
			rtskb_put(skb, l1);
			goto copydone;
		} /* if */
		}
#endif

		for (j = 0; j < adapter->rx_ps_pages; j++) {
			if (!(length= le16_to_cpu(rx_desc->wb.upper.length[j])))
				break;
			pci_unmap_page(pdev, ps_page_dma->ps_page_dma[j],
					PAGE_SIZE, PCI_DMA_FROMDEVICE);
			ps_page_dma->ps_page_dma[j] = 0;
			skb_fill_page_desc(skb, j, ps_page->ps_page[j], 0,
					   length);
			ps_page->ps_page[j] = NULL;
			skb->len += length;
			skb->data_len += length;
			skb->truesize += length;
		}

		/* strip the ethernet crc, problem is we're using pages now so
		 * this whole operation can get a little cpu intensive */
		pskb_trim(skb, skb->len - 4);

#ifdef CONFIG_E1000_NAPI
copydone:
#endif
		total_rx_bytes += skb->len;
		total_rx_packets++;

		e1000_rx_checksum(adapter, staterr,
				  le16_to_cpu(rx_desc->wb.lower.hi_dword.csum_ip.csum), skb);
		skb->protocol = rt_eth_type_trans(skb, netdev);

		if (likely(rx_desc->wb.upper.header_status &
			   cpu_to_le16(E1000_RXDPS_HDRSTAT_HDRSP)))
			adapter->rx_hdr_split++;

		e1000_receive_skb(adapter, staterr, rx_desc->wb.middle.vlan,
				  skb);
		netdev->last_rx = jiffies;

next_desc:
		rx_desc->wb.middle.status_error &= cpu_to_le32(~0xFF);
		buffer_info->skb = NULL;

		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= E1000_RX_BUFFER_WRITE)) {
			adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;

		staterr = le32_to_cpu(rx_desc->wb.middle.status_error);
	}
	rx_ring->next_to_clean = i;

	cleaned_count = E1000_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		adapter->alloc_rx_buf(adapter, rx_ring, cleaned_count);

	adapter->total_rx_packets += total_rx_packets;
	adapter->total_rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_bytes += total_rx_bytes;
	adapter->net_stats.rx_packets += total_rx_packets;
	return cleaned;
#endif
}

#ifdef CONFIG_E1000_NAPI
/**
 * e1000_alloc_jumbo_rx_buffers - Replace used jumbo receive buffers
 * @adapter: address of board private structure
 * @rx_ring: pointer to receive ring structure
 * @cleaned_count: number of buffers to allocate this pass
 **/
static void e1000_alloc_jumbo_rx_buffers(struct e1000_adapter *adapter,
					 struct e1000_rx_ring *rx_ring,
					 int cleaned_count)
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc;
	struct e1000_rx_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i;
	unsigned int bufsz = 256 -
			     16 /*for skb_reserve */ -
			     NET_IP_ALIGN;

	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];

	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {
			skb_trim(skb, 0);
			goto check_page;
		}

		skb = rtnetdev_alloc_rtskb(netdev, bufsz);
		if (unlikely(!skb)) {
			/* Better luck next round */
			adapter->alloc_rx_buff_failed++;
			break;
		}

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
			struct sk_buff *oldskb = skb;
			DPRINTK(PROBE, ERR, "skb align check failed: %u bytes "
					     "at %p\n", bufsz, skb->data);
			/* Try again, without freeing the previous */
			skb = rtnetdev_alloc_rtskb(netdev, bufsz);
			/* Failed allocation, critical failure */
			if (!skb) {
				kfree_rtskb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break;
			}

			if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
				/* give up */
				kfree_rtskb(skb);
				kfree_rtskb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break; /* while !buffer_info->skb */
			}

			/* Use new allocation */
			kfree_rtskb(oldskb);
		}
		/* Make buffer alignment 2 beyond a 16 byte boundary
		 * this will result in a 16 byte aligned IP header after
		 * the 14 byte MAC header is removed
		 */
		skb_reserve(skb, NET_IP_ALIGN);

		buffer_info->skb = skb;
check_page:
		/* allocate a new page if necessary */
		if (!buffer_info->page) {
			buffer_info->page = alloc_page(GFP_ATOMIC);
			if (unlikely(!buffer_info->page)) {
				adapter->alloc_rx_buff_failed++;
				break;
			}
		}

		if (!buffer_info->dma)
			buffer_info->dma = pci_map_page(pdev,
							buffer_info->page, 0,
							PAGE_SIZE,
							PCI_DMA_FROMDEVICE);

		rx_desc = E1000_RX_DESC(*rx_ring, i);
		rx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);

		if (unlikely(++i == rx_ring->count))
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}

	if (likely(rx_ring->next_to_use != i)) {
		rx_ring->next_to_use = i;
		if (unlikely(i-- == 0))
			i = (rx_ring->count - 1);

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64). */
		wmb();
		writel(i, adapter->hw.hw_addr + rx_ring->rdt);
	}
}
#endif /* NAPI */

/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/
static void e1000_alloc_rx_buffers(struct e1000_adapter *adapter,
				   struct e1000_rx_ring *rx_ring,
				   int cleaned_count)
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc;
	struct e1000_rx_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i;
	unsigned int bufsz = adapter->rx_buffer_len + NET_IP_ALIGN;

	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];

	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {
			rtskb_trim(skb, 0);
			goto map_skb;
		}

		skb = rtnetdev_alloc_rtskb(netdev, bufsz);
		if (unlikely(!skb)) {
			/* Better luck next round */
			adapter->alloc_rx_buff_failed++;
			break;
		}

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
			struct sk_buff *oldskb = skb;
			DPRINTK(RX_ERR, ERR, "skb align check failed: %u bytes "
					     "at %p\n", bufsz, skb->data);
			/* Try again, without freeing the previous */
			skb = rtnetdev_alloc_rtskb(netdev, bufsz);
			/* Failed allocation, critical failure */
			if (!skb) {
				kfree_rtskb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break;
			}

			if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
				/* give up */
				kfree_rtskb(skb);
				kfree_rtskb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break; /* while !buffer_info->skb */
			}

			/* Use new allocation */
			kfree_rtskb(oldskb);
		}
		/* Make buffer alignment 2 beyond a 16 byte boundary
		 * this will result in a 16 byte aligned IP header after
		 * the 14 byte MAC header is removed
		 */
		skb_reserve(skb, NET_IP_ALIGN);

		buffer_info->skb = skb;
map_skb:
		buffer_info->dma = pci_map_single(pdev,
						  skb->data,
						  adapter->rx_buffer_len,
						  PCI_DMA_FROMDEVICE);

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!e1000_check_64k_bound(adapter,
					(void *)(unsigned long)buffer_info->dma,
					adapter->rx_buffer_len)) {
			DPRINTK(RX_ERR, ERR,
				"dma align check failed: %u bytes at %p\n",
				adapter->rx_buffer_len,
				(void *)(unsigned long)buffer_info->dma);
			kfree_rtskb(skb);
			buffer_info->skb = NULL;

			pci_unmap_single(pdev, buffer_info->dma,
					 adapter->rx_buffer_len,
					 PCI_DMA_FROMDEVICE);
			buffer_info->dma = 0;

			adapter->alloc_rx_buff_failed++;
			break; /* while !buffer_info->skb */
		}
		rx_desc = E1000_RX_DESC(*rx_ring, i);
		rx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);

		if (unlikely(++i == rx_ring->count))
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}

	if (likely(rx_ring->next_to_use != i)) {
		rx_ring->next_to_use = i;
		if (unlikely(i-- == 0))
			i = (rx_ring->count - 1);

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64). */
		wmb();
		writel(i, adapter->hw.hw_addr + rx_ring->rdt);
	}
}

/**
 * e1000_alloc_rx_buffers_ps - Replace used receive buffers; packet split
 * @adapter: address of board private structure
 **/
static void e1000_alloc_rx_buffers_ps(struct e1000_adapter *adapter,
				      struct e1000_rx_ring *rx_ring,
				      int cleaned_count)
{
}

/**
 * e1000_smartspeed - Workaround for SmartSpeed on 82541 and 82547 controllers.
 * @adapter:
 **/
static void e1000_smartspeed(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_phy_info *phy = &adapter->hw.phy;
	u16 phy_status;
	u16 phy_ctrl;

	if ((phy->type != e1000_phy_igp) || !mac->autoneg ||
	    !(phy->autoneg_advertised & ADVERTISE_1000_FULL))
		return;

	if (adapter->smartspeed == 0) {
		/* If Master/Slave config fault is asserted twice,
		 * we assume back-to-back */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT)) return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT)) return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_ctrl);
		if (phy_ctrl & CR_1000T_MS_ENABLE) {
			phy_ctrl &= ~CR_1000T_MS_ENABLE;
			e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL,
					    phy_ctrl);
			adapter->smartspeed++;
			if (!e1000_phy_setup_autoneg(&adapter->hw) &&
			   !e1000_read_phy_reg(&adapter->hw, PHY_CONTROL,
					       &phy_ctrl)) {
				phy_ctrl |= (MII_CR_AUTO_NEG_EN |
					     MII_CR_RESTART_AUTO_NEG);
				e1000_write_phy_reg(&adapter->hw, PHY_CONTROL,
						    phy_ctrl);
			}
		}
		return;
	} else if (adapter->smartspeed == E1000_SMARTSPEED_DOWNSHIFT) {
		/* If still no link, perhaps using 2/3 pair cable */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_ctrl);
		phy_ctrl |= CR_1000T_MS_ENABLE;
		e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL, phy_ctrl);
		if (!e1000_phy_setup_autoneg(&adapter->hw) &&
		   !e1000_read_phy_reg(&adapter->hw, PHY_CONTROL, &phy_ctrl)) {
			phy_ctrl |= (MII_CR_AUTO_NEG_EN |
				     MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(&adapter->hw, PHY_CONTROL, phy_ctrl);
		}
	}
	/* Restart process after E1000_SMARTSPEED_MAX iterations */
	if (adapter->smartspeed++ == E1000_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}

/**
 * e1000_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
#if 0
static int e1000_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
#ifdef SIOCGMIIPHY
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return e1000_mii_ioctl(netdev, ifr, cmd);
#endif
#ifdef ETHTOOL_OPS_COMPAT
	case SIOCETHTOOL:
		return ethtool_ioctl(ifr);
#endif
	default:
		return -EOPNOTSUPP;
	}
}

#ifdef SIOCGMIIPHY
/**
 * e1000_mii_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int e1000_mii_ioctl(struct net_device *netdev, struct ifreq *ifr,
			   int cmd)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);

	if (adapter->hw.phy.media_type != e1000_media_type_copper)
		return -EOPNOTSUPP;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = adapter->hw.phy.addr;
		break;
	case SIOCGMIIREG:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		switch (data->reg_num & 0x1F) {
		case MII_BMCR:
			data->val_out = adapter->phy_regs.bmcr;
			break;
		case MII_BMSR:
			data->val_out = adapter->phy_regs.bmsr;
			break;
		case MII_PHYSID1:
			data->val_out = (adapter->hw.phy.id >> 16);
			break;
		case MII_PHYSID2:
			data->val_out = (adapter->hw.phy.id & 0xFFFF);
			break;
		case MII_ADVERTISE:
			data->val_out = adapter->phy_regs.advertise;
			break;
		case MII_LPA:
			data->val_out = adapter->phy_regs.lpa;
			break;
		case MII_EXPANSION:
			data->val_out = adapter->phy_regs.expansion;
			break;
		case MII_CTRL1000:
			data->val_out = adapter->phy_regs.ctrl1000;
			break;
		case MII_STAT1000:
			data->val_out = adapter->phy_regs.stat1000;
			break;
		case MII_ESTATUS:
			data->val_out = adapter->phy_regs.estatus;
			break;
		default:
			return -EIO;
		}
		break;
	case SIOCSMIIREG:
	default:
		return -EOPNOTSUPP;
	}
	return E1000_SUCCESS;
}
#endif
#endif

void e1000_pci_set_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = hw->back;
	int ret_val = pci_set_mwi(adapter->pdev);

	if (ret_val)
		DPRINTK(PROBE, ERR, "Error in setting MWI\n");
}

void e1000_pci_clear_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = hw->back;

	pci_clear_mwi(adapter->pdev);
}

void e1000_read_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = hw->back;

	pci_read_config_word(adapter->pdev, reg, value);
}

void e1000_write_pci_cfg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = hw->back;

	pci_write_config_word(adapter->pdev, reg, *value);
}

s32 e1000_read_pcie_cap_reg(struct e1000_hw *hw, u32 reg, u16 *value)
{
	struct e1000_adapter *adapter = hw->back;
	u16 cap_offset;

	cap_offset = pci_find_capability(adapter->pdev, PCI_CAP_ID_EXP);
	if (!cap_offset)
		return -E1000_ERR_CONFIG;

	pci_read_config_word(adapter->pdev, cap_offset + reg, value);

	return E1000_SUCCESS;
}

#ifdef NETIF_F_HW_VLAN_TX
static void e1000_vlan_rx_register(struct net_device *netdev,
				   struct vlan_group *grp)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	u32 ctrl, rctl;

	e1000_irq_disable(adapter);
	adapter->vlgrp = grp;

	if (grp) {
		/* enable VLAN tag insert/strip */
		ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
		ctrl |= E1000_CTRL_VME;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);

		if ((adapter->hw.mac.type != e1000_ich8lan) &&
		    (adapter->hw.mac.type != e1000_ich9lan)) {
			/* enable VLAN receive filtering */
			rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
			rctl |= E1000_RCTL_VFE;
			rctl &= ~E1000_RCTL_CFIEN;
			E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
			e1000_update_mng_vlan(adapter);
		}
	} else {
		/* disable VLAN tag insert/strip */
		ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
		ctrl &= ~E1000_CTRL_VME;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);

		if ((adapter->hw.mac.type != e1000_ich8lan) &&
		    (adapter->hw.mac.type != e1000_ich9lan)) {
			/* disable VLAN filtering */
			rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
			rctl &= ~E1000_RCTL_VFE;
			E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
			if (adapter->mng_vlan_id !=
			    (u16)E1000_MNG_VLAN_NONE) {
				e1000_vlan_rx_kill_vid(netdev,
						       adapter->mng_vlan_id);
				adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
			}
		}
	}

	e1000_irq_enable(adapter);
}

static void e1000_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	u32 vfta, index;
	struct net_device *v_netdev;

	if ((adapter->hw.mng_cookie.status &
	     E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
	    (vid == adapter->mng_vlan_id))
		return;
	/* add VID to filter table */
	index = (vid >> 5) & 0x7F;
	vfta = E1000_READ_REG_ARRAY(&adapter->hw, E1000_VFTA, index);
	vfta |= (1 << (vid & 0x1F));
	e1000_write_vfta(&adapter->hw, index, vfta);
	/* Copy feature flags from netdev to the vlan netdev for this vid.
	 * This allows things like TSO to bubble down to our vlan device.
	 */
	v_netdev = vlan_group_get_device(adapter->vlgrp, vid);
	v_netdev->features |= adapter->netdev->features;
	vlan_group_set_device(adapter->vlgrp, vid, v_netdev);
}

static void e1000_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	u32 vfta, index;

	e1000_irq_disable(adapter);
	vlan_group_set_device(adapter->vlgrp, vid, NULL);
	e1000_irq_enable(adapter);

	if ((adapter->hw.mng_cookie.status &
	     E1000_MNG_DHCP_COOKIE_STATUS_VLAN) &&
	    (vid == adapter->mng_vlan_id)) {
		/* release control to f/w */
		e1000_release_hw_control(adapter);
		return;
	}

	/* remove VID from filter table */
	index = (vid >> 5) & 0x7F;
	vfta = E1000_READ_REG_ARRAY(&adapter->hw, E1000_VFTA, index);
	vfta &= ~(1 << (vid & 0x1F));
	e1000_write_vfta(&adapter->hw, index, vfta);
}

static void e1000_restore_vlan(struct e1000_adapter *adapter)
{
	e1000_vlan_rx_register(adapter->netdev, adapter->vlgrp);

	if (adapter->vlgrp) {
		u16 vid;
		for (vid = 0; vid < VLAN_N_VID; vid++) {
			if (!vlan_group_get_device(adapter->vlgrp, vid))
				continue;
			e1000_vlan_rx_add_vid(adapter->netdev, vid);
		}
	}
}
#endif

int e1000_set_spd_dplx(struct e1000_adapter *adapter, u16 spddplx)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;

	mac->autoneg = 0;

	/* Fiber NICs only allow 1000 gbps Full duplex */
	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) &&
		spddplx != (SPEED_1000 + DUPLEX_FULL)) {
		DPRINTK(PROBE, ERR, "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}

	switch (spddplx) {
	case SPEED_10 + DUPLEX_HALF:
		mac->forced_speed_duplex = ADVERTISE_10_HALF;
		break;
	case SPEED_10 + DUPLEX_FULL:
		mac->forced_speed_duplex = ADVERTISE_10_FULL;
		break;
	case SPEED_100 + DUPLEX_HALF:
		mac->forced_speed_duplex = ADVERTISE_100_HALF;
		break;
	case SPEED_100 + DUPLEX_FULL:
		mac->forced_speed_duplex = ADVERTISE_100_FULL;
		break;
	case SPEED_1000 + DUPLEX_FULL:
		mac->autoneg = 1;
		adapter->hw.phy.autoneg_advertised = ADVERTISE_1000_FULL;
		break;
	case SPEED_1000 + DUPLEX_HALF: /* not supported */
	default:
		DPRINTK(PROBE, ERR, "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}
	return 0;
}

#ifdef USE_REBOOT_NOTIFIER
/* only want to do this for 2.4 kernels? */
static int e1000_notify_reboot(struct notifier_block *nb,
			       unsigned long event, void *p)
{
	struct pci_dev *pdev = NULL;

	switch (event) {
	case SYS_DOWN:
	case SYS_HALT:
	case SYS_POWER_OFF:
		while ((pdev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, pdev))) {
			if (pci_dev_driver(pdev) == &e1000_driver)
				e1000_suspend(pdev, PMSG_SUSPEND);
		}
	}
	return NOTIFY_DONE;
}
#endif

#ifdef CONFIG_PM
static int e1000_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	if ((err = pci_enable_device(pdev))) {
		printk(KERN_ERR "e1000: Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	if (rtnetif_running(netdev) && (err = e1000_request_irq(adapter)))
		return err;

	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		e1000_power_up_phy(&adapter->hw);
		e1000_setup_link(&adapter->hw);
	}
	e1000_reset(adapter);
	E1000_WRITE_REG(&adapter->hw, E1000_WUS, ~0);

	e1000_init_manageability(adapter);

	if (rtnetif_running(netdev))
		e1000_up(adapter);

	netif_device_attach(netdev);

	/* If the controller is 82573 or ICHx and f/w is AMT, do not set
	 * DRV_LOAD until the interface is up.  For all other cases,
	 * let the f/w know that the h/w is now under the control
	 * of the driver. */
	if (((adapter->hw.mac.type != e1000_82573) &&
	     (adapter->hw.mac.type != e1000_ich8lan) &&
	     (adapter->hw.mac.type != e1000_ich9lan)) ||
	    !e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	return 0;
}
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void e1000_netpoll(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int i;

	disable_irq(adapter->pdev->irq);
	e1000_intr(adapter->pdev->irq, netdev);

	for (i = 0; i < adapter->num_tx_queues ; i++ )
		e1000_clean_tx_irq(adapter, &adapter->tx_ring[i]);
#ifndef CONFIG_E1000_NAPI
	for (i = 0; i < adapter->num_rx_queues ; i++ )
		adapter->clean_rx(adapter, &adapter->rx_ring[i], NULL);
#endif
	enable_irq(adapter->pdev->irq);
}
#endif

#ifdef HAVE_PCI_ERS
/**
 * e1000_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t e1000_io_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev->priv;

	netif_device_detach(netdev);

	if (rtnetif_running(netdev))
		e1000_down(adapter);
	pci_disable_device(pdev);

	/* Request a slot slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * e1000_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot. Implementation
 * resembles the first-half of the e1000_resume routine.
 */
static pci_ers_result_t e1000_io_slot_reset(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev->priv;

	if (pci_enable_device(pdev)) {
		printk(KERN_ERR "e1000: Cannot re-enable PCI device after reset.\n");
		return PCI_ERS_RESULT_DISCONNECT;
	}
	pci_set_master(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	e1000_reset(adapter);
	E1000_WRITE_REG(&adapter->hw, E1000_WUS, ~0);

	return PCI_ERS_RESULT_RECOVERED;
}

/**
 * e1000_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation. Implementation resembles the
 * second-half of the e1000_resume routine.
 */
static void e1000_io_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev->priv;

	e1000_init_manageability(adapter);

	if (rtnetif_running(netdev)) {
		if (e1000_up(adapter)) {
			printk("e1000: can't bring device back up after reset\n");
			return;
		}
	}

	netif_device_attach(netdev);

	/* If the controller is 82573 or ICHx and f/w is AMT, do not set
	 * DRV_LOAD until the interface is up.  For all other cases,
	 * let the f/w know that the h/w is now under the control
	 * of the driver. */
	if (((adapter->hw.mac.type != e1000_82573) &&
	     (adapter->hw.mac.type != e1000_ich8lan) &&
	     (adapter->hw.mac.type != e1000_ich9lan)) ||
	    !e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

}
#endif /* HAVE_PCI_ERS */

s32 e1000_alloc_zeroed_dev_spec_struct(struct e1000_hw *hw, u32 size)
{
	hw->dev_spec = kmalloc(size, GFP_KERNEL);

	if (!hw->dev_spec)
		return -ENOMEM;

	memset(hw->dev_spec, 0, size);

	return E1000_SUCCESS;
}

void e1000_free_dev_spec_struct(struct e1000_hw *hw)
{
	if (!hw->dev_spec)
		return;

	kfree(hw->dev_spec);
}

/* vim: set ts=4: */
/* e1000_main.c */
