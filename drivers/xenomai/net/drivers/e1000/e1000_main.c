/*******************************************************************************


  Copyright(c) 1999 - 2006 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/

#include "e1000.h"

/* Change Log
 *
 * Port to rtnet (0.9.3) by Mathias Koehrer. Base version: e1000-7.1.9
 *             8-Aug-2006
 *
 * 7.0.36      10-Mar-2006
 *   o fixups for compilation issues on older kernels
 * 7.0.35      3-Mar-2006
 * 7.0.34
 *   o Major performance fixes by understanding relationship of rx_buffer_len
 *     to window size growth.  _ps and legacy receive paths changed
 *   o merge with kernel changes
 *   o legacy receive path went back to single descriptor model for jumbos
 * 7.0.33      3-Feb-2006
 *   o Added another fix for the pass false carrier bit
 * 7.0.32      24-Jan-2006
 *   o Need to rebuild with noew version number for the pass false carrier
 *     fix in e1000_hw.c
 * 7.0.30      18-Jan-2006
 *   o fixup for tso workaround to disable it for pci-x
 *   o fix mem leak on 82542
 *   o fixes for 10 Mb/s connections and incorrect stats
 * 7.0.28      01/06/2006
 *   o hardware workaround to only set "speed mode" bit for 1G link.
 * 7.0.26      12/23/2005
 *   o wake on lan support modified for device ID 10B5
 *   o fix dhcp + vlan issue not making it to the iAMT firmware
 * 7.0.24      12/9/2005
 *   o New hardware support for the Gigabit NIC embedded in the south bridge
 *   o Fixes to the recycling logic (skb->tail) from IBM LTC
 * 6.3.7	11/18/2005
 *   o Honor eeprom setting for enabling/disabling Wake On Lan
 * 6.3.5	11/17/2005
 *   o Fix memory leak in rx ring handling for PCI Express adapters
 * 6.3.4	11/8/05
 *   o Patch from Jesper Juhl to remove redundant NULL checks for kfree
 * 6.3.2	9/20/05
 *   o Render logic that sets/resets DRV_LOAD as inline functions to
 *     avoid code replication. If f/w is AMT then set DRV_LOAD only when
 *     network interface is open.
 *   o Handle DRV_LOAD set/reset in cases where AMT uses VLANs.
 *   o Adjust PBA partioning for Jumbo frames using MTU size and not
 *     rx_buffer_len
 * 6.3.1	9/19/05
 *   o Use adapter->tx_timeout_factor in Tx Hung Detect logic
 *      (e1000_clean_tx_irq)
 *   o Support for 8086:10B5 device (Quad Port)
 */

char e1000_driver_name[] = "rt_e1000";
static char e1000_driver_string[] = "Intel(R) PRO/1000 Network Driver";
#ifndef CONFIG_E1000_NAPI
#define DRIVERNAPI
#else
#define DRIVERNAPI "-NAPI"
#endif
#define DRV_VERSION "7.1.9"DRIVERNAPI
char e1000_driver_version[] = DRV_VERSION;
static char e1000_copyright[] = "Copyright (c) 1999-2006 Intel Corporation.";

/* e1000_pci_tbl - PCI Device ID Table
 *
 * Last entry must be all 0s
 *
 * Macro expands to...
 *   {PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}
 */
static struct pci_device_id e1000_pci_tbl[] = {
	INTEL_E1000_ETHERNET_DEVICE(0x1000),
	INTEL_E1000_ETHERNET_DEVICE(0x1001),
	INTEL_E1000_ETHERNET_DEVICE(0x1004),
	INTEL_E1000_ETHERNET_DEVICE(0x1008),
	INTEL_E1000_ETHERNET_DEVICE(0x1009),
	INTEL_E1000_ETHERNET_DEVICE(0x100C),
	INTEL_E1000_ETHERNET_DEVICE(0x100D),
	INTEL_E1000_ETHERNET_DEVICE(0x100E),
	INTEL_E1000_ETHERNET_DEVICE(0x100F),
	INTEL_E1000_ETHERNET_DEVICE(0x1010),
	INTEL_E1000_ETHERNET_DEVICE(0x1011),
	INTEL_E1000_ETHERNET_DEVICE(0x1012),
	INTEL_E1000_ETHERNET_DEVICE(0x1013),
	INTEL_E1000_ETHERNET_DEVICE(0x1014),
	INTEL_E1000_ETHERNET_DEVICE(0x1015),
	INTEL_E1000_ETHERNET_DEVICE(0x1016),
	INTEL_E1000_ETHERNET_DEVICE(0x1017),
	INTEL_E1000_ETHERNET_DEVICE(0x1018),
	INTEL_E1000_ETHERNET_DEVICE(0x1019),
	INTEL_E1000_ETHERNET_DEVICE(0x101A),
	INTEL_E1000_ETHERNET_DEVICE(0x101D),
	INTEL_E1000_ETHERNET_DEVICE(0x101E),
	INTEL_E1000_ETHERNET_DEVICE(0x1026),
	INTEL_E1000_ETHERNET_DEVICE(0x1027),
	INTEL_E1000_ETHERNET_DEVICE(0x1028),
	INTEL_E1000_ETHERNET_DEVICE(0x1049),
	INTEL_E1000_ETHERNET_DEVICE(0x104A),
	INTEL_E1000_ETHERNET_DEVICE(0x104B),
	INTEL_E1000_ETHERNET_DEVICE(0x104C),
	INTEL_E1000_ETHERNET_DEVICE(0x104D),
	INTEL_E1000_ETHERNET_DEVICE(0x105E),
	INTEL_E1000_ETHERNET_DEVICE(0x105F),
	INTEL_E1000_ETHERNET_DEVICE(0x1060),
	INTEL_E1000_ETHERNET_DEVICE(0x1075),
	INTEL_E1000_ETHERNET_DEVICE(0x1076),
	INTEL_E1000_ETHERNET_DEVICE(0x1077),
	INTEL_E1000_ETHERNET_DEVICE(0x1078),
	INTEL_E1000_ETHERNET_DEVICE(0x1079),
	INTEL_E1000_ETHERNET_DEVICE(0x107A),
	INTEL_E1000_ETHERNET_DEVICE(0x107B),
	INTEL_E1000_ETHERNET_DEVICE(0x107C),
	INTEL_E1000_ETHERNET_DEVICE(0x107D),
	INTEL_E1000_ETHERNET_DEVICE(0x107E),
	INTEL_E1000_ETHERNET_DEVICE(0x107F),
	INTEL_E1000_ETHERNET_DEVICE(0x108A),
	INTEL_E1000_ETHERNET_DEVICE(0x108B),
	INTEL_E1000_ETHERNET_DEVICE(0x108C),
	INTEL_E1000_ETHERNET_DEVICE(0x1096),
	INTEL_E1000_ETHERNET_DEVICE(0x1098),
	INTEL_E1000_ETHERNET_DEVICE(0x1099),
	INTEL_E1000_ETHERNET_DEVICE(0x109A),
	INTEL_E1000_ETHERNET_DEVICE(0x10A4),
	INTEL_E1000_ETHERNET_DEVICE(0x10B5),
	INTEL_E1000_ETHERNET_DEVICE(0x10B9),
	INTEL_E1000_ETHERNET_DEVICE(0x10BA),
	INTEL_E1000_ETHERNET_DEVICE(0x10BB),
	INTEL_E1000_ETHERNET_DEVICE(0x10BC),
	INTEL_E1000_ETHERNET_DEVICE(0x10C4),
	INTEL_E1000_ETHERNET_DEVICE(0x10C5),
	/* required last entry */
	{0,}
};

MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);

int e1000_up(struct e1000_adapter *adapter);
void e1000_down(struct e1000_adapter *adapter);
void e1000_reinit_locked(struct e1000_adapter *adapter);
void e1000_reset(struct e1000_adapter *adapter);
int e1000_set_spd_dplx(struct e1000_adapter *adapter, uint16_t spddplx);
int e1000_setup_all_tx_resources(struct e1000_adapter *adapter);
int e1000_setup_all_rx_resources(struct e1000_adapter *adapter);
void e1000_free_all_tx_resources(struct e1000_adapter *adapter);
void e1000_free_all_rx_resources(struct e1000_adapter *adapter);
static int e1000_setup_tx_resources(struct e1000_adapter *adapter,
			     struct e1000_tx_ring *txdr);
static int e1000_setup_rx_resources(struct e1000_adapter *adapter,
			     struct e1000_rx_ring *rxdr);
static void e1000_free_tx_resources(struct e1000_adapter *adapter,
			     struct e1000_tx_ring *tx_ring);
static void e1000_free_rx_resources(struct e1000_adapter *adapter,
			     struct e1000_rx_ring *rx_ring);

static int e1000_init_module(void);
static void e1000_exit_module(void);
static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void e1000_remove(struct pci_dev *pdev);
static int e1000_alloc_queues(struct e1000_adapter *adapter);
static int e1000_sw_init(struct e1000_adapter *adapter);
static int e1000_open(struct rtnet_device *netdev);
static int e1000_close(struct rtnet_device *netdev);
static void e1000_configure_tx(struct e1000_adapter *adapter);
static void e1000_configure_rx(struct e1000_adapter *adapter);
static void e1000_setup_rctl(struct e1000_adapter *adapter);
static void e1000_clean_all_tx_rings(struct e1000_adapter *adapter);
static void e1000_clean_all_rx_rings(struct e1000_adapter *adapter);
static void e1000_clean_tx_ring(struct e1000_adapter *adapter,
				struct e1000_tx_ring *tx_ring);
static void e1000_clean_rx_ring(struct e1000_adapter *adapter,
				struct e1000_rx_ring *rx_ring);
static void e1000_set_multi(struct rtnet_device *netdev);
static void e1000_update_phy_info_task(struct work_struct *work);
static void e1000_watchdog(struct work_struct *work);
static void e1000_82547_tx_fifo_stall_task(struct work_struct *work);
static int e1000_xmit_frame(struct rtskb *skb, struct rtnet_device *netdev);
static int e1000_intr(rtdm_irq_t *irq_handle);
static boolean_t e1000_clean_tx_irq(struct e1000_adapter *adapter,
				    struct e1000_tx_ring *tx_ring);
static boolean_t e1000_clean_rx_irq(struct e1000_adapter *adapter,
				    struct e1000_rx_ring *rx_ring,
				    nanosecs_abs_t *time_stamp);
static void e1000_alloc_rx_buffers(struct e1000_adapter *adapter,
				   struct e1000_rx_ring *rx_ring,
				   int cleaned_count);
#ifdef SIOCGMIIPHY
#endif
void e1000_set_ethtool_ops(struct rtnet_device *netdev);
#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *ifr);
#endif
static void e1000_enter_82542_rst(struct e1000_adapter *adapter);
static void e1000_leave_82542_rst(struct e1000_adapter *adapter);
static void e1000_smartspeed(struct e1000_adapter *adapter);
static int e1000_82547_fifo_workaround(struct e1000_adapter *adapter,
				       struct rtskb *skb);





/* Exported from other modules */

extern void e1000_check_options(struct e1000_adapter *adapter);

static struct pci_driver e1000_driver = {
	.name     = e1000_driver_name,
	.id_table = e1000_pci_tbl,
	.probe    = e1000_probe,
	.remove   = e1000_remove,
};

MODULE_AUTHOR("Intel Corporation, <linux.nics@intel.com>");
MODULE_DESCRIPTION("Intel(R) PRO/1000 Network Driver for rtnet");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

static int local_debug = NETIF_MSG_DRV | NETIF_MSG_PROBE;
module_param_named(debug, local_debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");


#define MAX_UNITS           8

static int cards[MAX_UNITS] = { [0 ... (MAX_UNITS-1)] = 1 };
module_param_array(cards, int, NULL, 0444);
MODULE_PARM_DESC(cards, "array of cards to be supported (eg. 1,0,1)");


#define kmalloc(a,b) rtdm_malloc(a)
#define vmalloc(a) rtdm_malloc(a)
#define kfree(a) rtdm_free(a)
#define vfree(a) rtdm_free(a)


/**
 * e1000_init_module - Driver Registration Routine
 *
 * e1000_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/

static int __init
e1000_init_module(void)
{
	int ret;
	printk(KERN_INFO "%s - version %s\n",
	       e1000_driver_string, e1000_driver_version);

	printk(KERN_INFO "%s\n", e1000_copyright);

	ret = pci_register_driver(&e1000_driver);
	return ret;
}

module_init(e1000_init_module);

/**
 * e1000_exit_module - Driver Exit Cleanup Routine
 *
 * e1000_exit_module is called just before the driver is removed
 * from memory.
 **/

static void __exit
e1000_exit_module(void)
{
	pci_unregister_driver(&e1000_driver);
}

module_exit(e1000_exit_module);

static int e1000_request_irq(struct e1000_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	int flags, err = 0;

	flags = RTDM_IRQTYPE_SHARED;
#ifdef CONFIG_PCI_MSI
	if (adapter->hw.mac_type > e1000_82547_rev_2) {
		adapter->have_msi = TRUE;
		if ((err = pci_enable_msi(adapter->pdev))) {
			DPRINTK(PROBE, ERR,
			 "Unable to allocate MSI interrupt Error: %d\n", err);
			adapter->have_msi = FALSE;
		}
	}
	if (adapter->have_msi)
		flags = 0;
#endif
	rt_stack_connect(netdev, &STACK_manager);
	if ((err = rtdm_irq_request(&adapter->irq_handle, adapter->pdev->irq,
				    e1000_intr, flags, netdev->name, netdev)))
		DPRINTK(PROBE, ERR,
		    "Unable to allocate interrupt Error: %d\n", err);

	return err;
}

static void e1000_free_irq(struct e1000_adapter *adapter)
{
	// struct rtnet_device *netdev = adapter->netdev;

	rtdm_irq_free(&adapter->irq_handle);

#ifdef CONFIG_PCI_MSI
	if (adapter->have_msi)
		pci_disable_msi(adapter->pdev);
#endif
}

/**
 * e1000_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/

static void
e1000_irq_disable(struct e1000_adapter *adapter)
{
	atomic_inc(&adapter->irq_sem);
	E1000_WRITE_REG(&adapter->hw, IMC, ~0);
	E1000_WRITE_FLUSH(&adapter->hw);
	synchronize_irq(adapter->pdev->irq);
}

/**
 * e1000_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

static void
e1000_irq_enable(struct e1000_adapter *adapter)
{
	if (likely(atomic_dec_and_test(&adapter->irq_sem))) {
		E1000_WRITE_REG(&adapter->hw, IMS, IMS_ENABLE_MASK);
		E1000_WRITE_FLUSH(&adapter->hw);
	}
}

/**
 * e1000_release_hw_control - release control of the h/w to f/w
 * @adapter: address of board private structure
 *
 * e1000_release_hw_control resets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded. For AMT version (only with 82573) i
 * of the f/w this means that the netowrk i/f is closed.
 *
 **/

static void
e1000_release_hw_control(struct e1000_adapter *adapter)
{
	uint32_t ctrl_ext;
	uint32_t swsm;
	uint32_t extcnf;

	/* Let firmware taken over control of h/w */
	switch (adapter->hw.mac_type) {
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		ctrl_ext = E1000_READ_REG(&adapter->hw, CTRL_EXT);
		E1000_WRITE_REG(&adapter->hw, CTRL_EXT,
				ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
		break;
	case e1000_82573:
		swsm = E1000_READ_REG(&adapter->hw, SWSM);
		E1000_WRITE_REG(&adapter->hw, SWSM,
				swsm & ~E1000_SWSM_DRV_LOAD);
		break;
	case e1000_ich8lan:
		extcnf = E1000_READ_REG(&adapter->hw, CTRL_EXT);
		E1000_WRITE_REG(&adapter->hw, CTRL_EXT,
				extcnf & ~E1000_CTRL_EXT_DRV_LOAD);
		break;
	default:
		break;
	}
}

/**
 * e1000_get_hw_control - get control of the h/w from f/w
 * @adapter: address of board private structure
 *
 * e1000_get_hw_control sets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded. For AMT version (only with 82573)
 * of the f/w this means that the netowrk i/f is open.
 *
 **/

static void
e1000_get_hw_control(struct e1000_adapter *adapter)
{
	uint32_t ctrl_ext;
	uint32_t swsm;
	uint32_t extcnf;
	/* Let firmware know the driver has taken over */
	switch (adapter->hw.mac_type) {
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		ctrl_ext = E1000_READ_REG(&adapter->hw, CTRL_EXT);
		E1000_WRITE_REG(&adapter->hw, CTRL_EXT,
				ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
		break;
	case e1000_82573:
		swsm = E1000_READ_REG(&adapter->hw, SWSM);
		E1000_WRITE_REG(&adapter->hw, SWSM,
				swsm | E1000_SWSM_DRV_LOAD);
		break;
	case e1000_ich8lan:
		extcnf = E1000_READ_REG(&adapter->hw, EXTCNF_CTRL);
		E1000_WRITE_REG(&adapter->hw, EXTCNF_CTRL,
				extcnf | E1000_EXTCNF_CTRL_SWFLAG);
		break;
	default:
		break;
	}
}

int
e1000_up(struct e1000_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	int i;

	/* hardware has been reset, we need to reload some things */

	e1000_set_multi(netdev);


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

	// TODO makoehre adapter->tx_queue_len = netdev->tx_queue_len;

	schedule_delayed_work(&adapter->watchdog_task, 1);

	e1000_irq_enable(adapter);

	return 0;
}

/**
 * e1000_power_up_phy - restore link in case the phy was powered down
 * @adapter: address of board private structure
 *
 * The phy may be powered down to save power and turn off link when the
 * driver is unloaded and wake on lan is not enabled (among others)
 * *** this routine MUST be followed by a call to e1000_reset ***
 *
 **/

static void e1000_power_up_phy(struct e1000_adapter *adapter)
{
	uint16_t mii_reg = 0;

	/* Just clear the power down bit to wake the phy back up */
	if (adapter->hw.media_type == e1000_media_type_copper) {
		/* according to the manual, the phy will retain its
		 * settings across a power-down/up cycle */
		e1000_read_phy_reg(&adapter->hw, PHY_CTRL, &mii_reg);
		mii_reg &= ~MII_CR_POWER_DOWN;
		e1000_write_phy_reg(&adapter->hw, PHY_CTRL, mii_reg);
	}
}

static void e1000_power_down_phy(struct e1000_adapter *adapter)
{
	boolean_t mng_mode_enabled = (adapter->hw.mac_type >= e1000_82571) &&
				      e1000_check_mng_mode(&adapter->hw);
	/* Power down the PHY so no link is implied when interface is down *
	 * The PHY cannot be powered down if any of the following is TRUE *
	 * (a) WoL is enabled
	 * (b) AMT is active
	 * (c) SoL/IDER session is active */
	if (!adapter->wol && adapter->hw.mac_type >= e1000_82540 &&
	   adapter->hw.mac_type != e1000_ich8lan &&
	   adapter->hw.media_type == e1000_media_type_copper &&
	   !(E1000_READ_REG(&adapter->hw, MANC) & E1000_MANC_SMBUS_EN) &&
	   !mng_mode_enabled &&
	   !e1000_check_phy_reset_block(&adapter->hw)) {
		uint16_t mii_reg = 0;
		e1000_read_phy_reg(&adapter->hw, PHY_CTRL, &mii_reg);
		mii_reg |= MII_CR_POWER_DOWN;
		e1000_write_phy_reg(&adapter->hw, PHY_CTRL, mii_reg);
		mdelay(1);
	}
}

static void e1000_down_and_stop(struct e1000_adapter *adapter)
{
	cancel_work_sync(&adapter->reset_task);
	cancel_delayed_work_sync(&adapter->watchdog_task);
	cancel_delayed_work_sync(&adapter->phy_info_task);
	cancel_delayed_work_sync(&adapter->fifo_stall_task);
}

void
e1000_down(struct e1000_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;

	e1000_irq_disable(adapter);

	e1000_down_and_stop(adapter);

	// TODO makoehre     netdev->tx_queue_len = adapter->tx_queue_len;
	adapter->link_speed = 0;
	adapter->link_duplex = 0;
	rtnetif_carrier_off(netdev);
	rtnetif_stop_queue(netdev);

	e1000_reset(adapter);
	e1000_clean_all_tx_rings(adapter);
	e1000_clean_all_rx_rings(adapter);
}

void
e1000_reinit_locked(struct e1000_adapter *adapter)
{
	WARN_ON(in_interrupt());
	if (test_and_set_bit(__E1000_RESETTING, &adapter->flags))
		msleep(1);
	e1000_down(adapter);
	e1000_up(adapter);
	clear_bit(__E1000_RESETTING, &adapter->flags);
}

void
e1000_reset(struct e1000_adapter *adapter)
{
	uint32_t pba;
	uint16_t fc_high_water_mark = E1000_FC_HIGH_DIFF;

	/* Repartition Pba for greater than 9k mtu
	 * To take effect CTRL.RST is required.
	 */

	switch (adapter->hw.mac_type) {
	case e1000_82547:
	case e1000_82547_rev_2:
		pba = E1000_PBA_30K;
		break;
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		pba = E1000_PBA_38K;
		break;
	case e1000_82573:
		pba = E1000_PBA_12K;
		break;
	case e1000_ich8lan:
		pba = E1000_PBA_8K;
		break;
	default:
		pba = E1000_PBA_48K;
		break;
	}

	if ((adapter->hw.mac_type != e1000_82573) &&
	   (adapter->netdev->mtu > E1000_RXBUFFER_8192))
		pba -= 8; /* allocate more FIFO for Tx */


	if (adapter->hw.mac_type == e1000_82547) {
		adapter->tx_fifo_head = 0;
		adapter->tx_head_addr = pba << E1000_TX_HEAD_ADDR_SHIFT;
		adapter->tx_fifo_size =
			(E1000_PBA_40K - pba) << E1000_PBA_BYTES_SHIFT;
		atomic_set(&adapter->tx_fifo_stall, 0);
	}

	E1000_WRITE_REG(&adapter->hw, PBA, pba);

	/* flow control settings */
	/* Set the FC high water mark to 90% of the FIFO size.
	 * Required to clear last 3 LSB */
	fc_high_water_mark = ((pba * 9216)/10) & 0xFFF8;
	/* We can't use 90% on small FIFOs because the remainder
	 * would be less than 1 full frame.  In this case, we size
	 * it to allow at least a full frame above the high water
	 *  mark. */
	if (pba < E1000_PBA_16K)
		fc_high_water_mark = (pba * 1024) - 1600;

	adapter->hw.fc_high_water = fc_high_water_mark;
	adapter->hw.fc_low_water = fc_high_water_mark - 8;
	if (adapter->hw.mac_type == e1000_80003es2lan)
		adapter->hw.fc_pause_time = 0xFFFF;
	else
		adapter->hw.fc_pause_time = E1000_FC_PAUSE_TIME;
	adapter->hw.fc_send_xon = 1;
	adapter->hw.fc = adapter->hw.original_fc;

	/* Allow time for pending master requests to run */
	e1000_reset_hw(&adapter->hw);
	if (adapter->hw.mac_type >= e1000_82544)
		E1000_WRITE_REG(&adapter->hw, WUC, 0);
	if (e1000_init_hw(&adapter->hw))
		DPRINTK(PROBE, ERR, "Hardware Error\n");
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	E1000_WRITE_REG(&adapter->hw, VET, ETHERNET_IEEE_VLAN_TYPE);

	E1000_WRITE_REG(&adapter->hw, AIT, 0);  // Set adaptive interframe spacing to zero

	// e1000_reset_adaptive(&adapter->hw);
	e1000_phy_get_info(&adapter->hw, &adapter->phy_info);

	if (!adapter->smart_power_down &&
	    (adapter->hw.mac_type == e1000_82571 ||
	     adapter->hw.mac_type == e1000_82572)) {
		uint16_t phy_data = 0;
		/* speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed */
		e1000_read_phy_reg(&adapter->hw, IGP02E1000_PHY_POWER_MGMT,
				   &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1000_write_phy_reg(&adapter->hw, IGP02E1000_PHY_POWER_MGMT,
				    phy_data);
	}

}

static void
e1000_reset_task(struct work_struct *work)
{
	struct e1000_adapter *adapter =
		container_of(work, struct e1000_adapter, reset_task);

	e1000_reinit_locked(adapter);
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
	struct rtnet_device *netdev;
	struct e1000_adapter *adapter;
	unsigned long mmio_start, mmio_len;
	unsigned long flash_start, flash_len;

	static int cards_found = 0;
	static int e1000_ksp3_port_a = 0; /* global ksp3 port a indication */
	int i, err;
	uint16_t eeprom_data;
	uint16_t eeprom_apme_mask = E1000_EEPROM_APME;

	if (cards[cards_found++] == 0)
	{
	    return -ENODEV;
	}

	if ((err = pci_enable_device(pdev)))
		return err;

	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) ||
	    (err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64)))) {
		if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) &&
		    (err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32)))) {
			E1000_ERR("No usable DMA configuration, aborting\n");
			return err;
		}
	}

	if ((err = pci_request_regions(pdev, e1000_driver_name)))
		return err;

	pci_set_master(pdev);

	netdev = rt_alloc_etherdev(sizeof(struct e1000_adapter), 48);
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}
	memset(netdev->priv, 0, sizeof(struct e1000_adapter));

	rt_rtdev_connect(netdev, &RTDEV_manager);


	// SET_NETDEV_DEV(netdev, &pdev->dev);
	netdev->vers = RTDEV_VERS_2_0;
	netdev->sysbind = &pdev->dev;

	pci_set_drvdata(pdev, netdev);
	adapter = netdev->priv;
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->hw.back = adapter;
	adapter->msg_enable = (1 << local_debug) - 1;

	mmio_start = pci_resource_start(pdev, BAR_0);
	mmio_len = pci_resource_len(pdev, BAR_0);

	adapter->hw.hw_addr = ioremap(mmio_start, mmio_len);
	if (!adapter->hw.hw_addr) {
		err = -EIO;
		goto err_ioremap;
	}

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
	// netdev->get_stats = &e1000_get_stats;
	// netdev->set_multicast_list = &e1000_set_multi;
	// netdev->set_mac_address = &e1000_set_mac;
	// netdev->change_mtu = &e1000_change_mtu;
	// netdev->do_ioctl = &e1000_ioctl;
	// e1000_set_ethtool_ops(netdev);
	strcpy(netdev->name, pci_name(pdev));

	netdev->mem_start = mmio_start;
	netdev->mem_end = mmio_start + mmio_len;
	netdev->base_addr = adapter->hw.io_base;

	adapter->bd_number = cards_found - 1;

	/* setup the private structure */

	if ((err = e1000_sw_init(adapter)))
		goto err_sw_init;

	/* Flash BAR mapping must happen after e1000_sw_init
	 * because it depends on mac_type */
	if ((adapter->hw.mac_type == e1000_ich8lan) &&
	   (pci_resource_flags(pdev, 1) & IORESOURCE_MEM)) {
		flash_start = pci_resource_start(pdev, 1);
		flash_len = pci_resource_len(pdev, 1);
		adapter->hw.flash_address = ioremap(flash_start, flash_len);
		if (!adapter->hw.flash_address) {
			err = -EIO;
			goto err_flashmap;
		}
	}

	if ((err = e1000_check_phy_reset_block(&adapter->hw)))
		DPRINTK(PROBE, INFO, "PHY reset is blocked due to SOL/IDER session.\n");

	/* if ksp3, indicate if it's port a being setup */
	if (pdev->device == E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3 &&
			e1000_ksp3_port_a == 0)
		adapter->ksp3_port_a = 1;
	e1000_ksp3_port_a++;
	/* Reset for multiple KP3 adapters */
	if (e1000_ksp3_port_a == 4)
		e1000_ksp3_port_a = 0;

	netdev->features |= NETIF_F_LLTX;

	adapter->en_mng_pt = e1000_enable_mng_pass_thru(&adapter->hw);

	/* initialize eeprom parameters */

	if (e1000_init_eeprom_params(&adapter->hw)) {
		E1000_ERR("EEPROM initialization failed\n");
		return -EIO;
	}

	/* before reading the EEPROM, reset the controller to
	 * put the device in a known good starting state */

	e1000_reset_hw(&adapter->hw);

	/* make sure the EEPROM is good */

	if (e1000_validate_eeprom_checksum(&adapter->hw) < 0) {
		DPRINTK(PROBE, ERR, "The EEPROM Checksum Is Not Valid\n");
		err = -EIO;
		goto err_eeprom;
	}

	/* copy the MAC address out of the EEPROM */

	if (e1000_read_mac_addr(&adapter->hw))
		DPRINTK(PROBE, ERR, "EEPROM Read Error\n");
	memcpy(netdev->dev_addr, adapter->hw.mac_addr, netdev->addr_len);
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, adapter->hw.mac_addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->perm_addr)) {
#else
	if (!is_valid_ether_addr(netdev->dev_addr)) {
#endif
		DPRINTK(PROBE, ERR, "Invalid MAC Address\n");
		err = -EIO;
		goto err_eeprom;
	}

	e1000_read_part_num(&adapter->hw, &(adapter->part_num));

	e1000_get_bus_info(&adapter->hw);

	INIT_DELAYED_WORK(&adapter->watchdog_task, e1000_watchdog);
	INIT_DELAYED_WORK(&adapter->fifo_stall_task,
			  e1000_82547_tx_fifo_stall_task);
	INIT_DELAYED_WORK(&adapter->phy_info_task, e1000_update_phy_info_task);
	INIT_WORK(&adapter->reset_task,
		(void (*)(struct work_struct *))e1000_reset_task);

	/* we're going to reset, so assume we have no link for now */

	rtnetif_carrier_off(netdev);
	rtnetif_stop_queue(netdev);

	e1000_check_options(adapter);

	/* Initial Wake on LAN setting
	 * If APM wake is enabled in the EEPROM,
	 * enable the ACPI Magic Packet filter
	 */

	switch (adapter->hw.mac_type) {
	case e1000_82542_rev2_0:
	case e1000_82542_rev2_1:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_eeprom(&adapter->hw,
			EEPROM_INIT_CONTROL2_REG, 1, &eeprom_data);
		eeprom_apme_mask = E1000_EEPROM_82544_APM;
		break;
	case e1000_ich8lan:
		e1000_read_eeprom(&adapter->hw,
			EEPROM_INIT_CONTROL1_REG, 1, &eeprom_data);
		eeprom_apme_mask = E1000_EEPROM_ICH8_APME;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
	case e1000_82571:
	case e1000_80003es2lan:
		if (E1000_READ_REG(&adapter->hw, STATUS) & E1000_STATUS_FUNC_1){
			e1000_read_eeprom(&adapter->hw,
				EEPROM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		}
		/* Fall Through */
	default:
		e1000_read_eeprom(&adapter->hw,
			EEPROM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & eeprom_apme_mask)
		adapter->wol |= E1000_WUFC_MAG;

	/* print bus type/speed/width info */
	{
	struct e1000_hw *hw = &adapter->hw;
	DPRINTK(PROBE, INFO, "(PCI%s:%s:%s) ",
		((hw->bus_type == e1000_bus_type_pcix) ? "-X" :
		 (hw->bus_type == e1000_bus_type_pci_express ? " Express":"")),
		((hw->bus_speed == e1000_bus_speed_2500) ? "2.5Gb/s" :
		 (hw->bus_speed == e1000_bus_speed_133) ? "133MHz" :
		 (hw->bus_speed == e1000_bus_speed_120) ? "120MHz" :
		 (hw->bus_speed == e1000_bus_speed_100) ? "100MHz" :
		 (hw->bus_speed == e1000_bus_speed_66) ? "66MHz" : "33MHz"),
		((hw->bus_width == e1000_bus_width_64) ? "64-bit" :
		 (hw->bus_width == e1000_bus_width_pciex_4) ? "Width x4" :
		 (hw->bus_width == e1000_bus_width_pciex_1) ? "Width x1" :
		 "32-bit"));
	}

	printk(KERN_INFO "e1000: hw ");
	for (i = 0; i < 6; i++)
		printk(KERN_CONT "%2.2x%c", netdev->dev_addr[i], i == 5 ? '\n' : ':');

	/* reset the hardware with the new settings */
	e1000_reset(adapter);

	/* If the controller is 82573 and f/w is AMT, do not set
	 * DRV_LOAD until the interface is up.  For all other cases,
	 * let the f/w know that the h/w is now under the control
	 * of the driver. */
	if (adapter->hw.mac_type != e1000_82573 ||
	    !e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	strcpy(netdev->name, "rteth%d");
	if ((err = rt_register_rtnetdev(netdev)))
		goto err_register;

	DPRINTK(PROBE, INFO, "Intel(R) PRO/1000 Network Connection\n");

	return 0;

err_register:
	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);
err_flashmap:
err_sw_init:
err_eeprom:
	iounmap(adapter->hw.hw_addr);
err_ioremap:
	rtdev_free(netdev);
err_alloc_etherdev:
	pci_release_regions(pdev);
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
	struct rtnet_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev->priv;
	uint32_t manc;

	e1000_down_and_stop(adapter);

	if (adapter->hw.mac_type >= e1000_82540 &&
	   adapter->hw.mac_type != e1000_ich8lan &&
	   adapter->hw.media_type == e1000_media_type_copper) {
		manc = E1000_READ_REG(&adapter->hw, MANC);
		if (manc & E1000_MANC_SMBUS_EN) {
			manc |= E1000_MANC_ARP_EN;
			E1000_WRITE_REG(&adapter->hw, MANC, manc);
		}
	}

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant. */
	e1000_release_hw_control(adapter);

	rt_unregister_rtnetdev(netdev);

	if (!e1000_check_phy_reset_block(&adapter->hw))
		e1000_phy_hw_reset(&adapter->hw);

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
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;

	/* PCI config space info */

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_id = pdev->subsystem_device;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &hw->revision_id);

	pci_read_config_word(pdev, PCI_COMMAND, &hw->pci_cmd_word);

	adapter->rx_buffer_len = MAXIMUM_ETHERNET_FRAME_SIZE;
	adapter->rx_ps_bsize0 = E1000_RXBUFFER_128;
	hw->max_frame_size = netdev->mtu +
			     ENET_HEADER_SIZE + ETHERNET_FCS_SIZE;
	hw->min_frame_size = MINIMUM_ETHERNET_FRAME_SIZE;

	/* identify the MAC */

	if (e1000_set_mac_type(hw)) {
		DPRINTK(PROBE, ERR, "Unknown MAC Type\n");
		return -EIO;
	}

	switch (hw->mac_type) {
	default:
		break;
	case e1000_82541:
	case e1000_82547:
	case e1000_82541_rev_2:
	case e1000_82547_rev_2:
		hw->phy_init_script = 1;
		break;
	}

	e1000_set_media_type(hw);

	hw->wait_autoneg_complete = FALSE;
	hw->tbi_compatibility_en = TRUE;
	hw->adaptive_ifs = FALSE;

	/* Copper options */

	if (hw->media_type == e1000_media_type_copper) {
		hw->mdix = AUTO_ALL_MODES;
		hw->disable_polarity_correction = FALSE;
		hw->master_slave = E1000_MASTER_SLAVE;
	}

	adapter->num_tx_queues = 1;
	adapter->num_rx_queues = 1;


	if (e1000_alloc_queues(adapter)) {
		DPRINTK(PROBE, ERR, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	atomic_set(&adapter->irq_sem, 1);

	return 0;
}

/**
 * e1000_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 * We allocate one ring per queue at run-time since we don't know the
 * number of queues at compile-time.  The polling_netdev array is
 * intended for Multiqueue, but should work fine with a single queue.
 **/

static int e1000_alloc_queues(struct e1000_adapter *adapter)
{
	int size;

	size = sizeof(struct e1000_tx_ring) * adapter->num_tx_queues;
	adapter->tx_ring = kmalloc(size, GFP_KERNEL);
	if (!adapter->tx_ring)
		return -ENOMEM;
	memset(adapter->tx_ring, 0, size);

	size = sizeof(struct e1000_rx_ring) * adapter->num_rx_queues;
	adapter->rx_ring = kmalloc(size, GFP_KERNEL);
	if (!adapter->rx_ring) {
		kfree(adapter->tx_ring);
		return -ENOMEM;
	}
	memset(adapter->rx_ring, 0, size);


	return E1000_SUCCESS;
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

static int
e1000_open(struct rtnet_device *netdev)
{
	struct e1000_adapter *adapter = netdev->priv;
	int err;

	/* disallow open during test */
	if (test_bit(__E1000_DRIVER_TESTING, &adapter->flags))
		return -EBUSY;

	/* allocate transmit descriptors */

	if ((err = e1000_setup_all_tx_resources(adapter)))
		goto err_setup_tx;

	/* allocate receive descriptors */

	if ((err = e1000_setup_all_rx_resources(adapter)))
		goto err_setup_rx;

	err = e1000_request_irq(adapter);
	if (err)
		goto err_up;

	e1000_power_up_phy(adapter);

	if ((err = e1000_up(adapter)))
		goto err_up;

	/* If AMT is enabled, let the firmware know that the network
	 * interface is now open */
	if (adapter->hw.mac_type == e1000_82573 &&
	    e1000_check_mng_mode(&adapter->hw))
		e1000_get_hw_control(adapter);

	/* Wait for the hardware to come up */
	msleep(3000);

	return E1000_SUCCESS;

err_up:
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

static int
e1000_close(struct rtnet_device *netdev)
{
	struct e1000_adapter *adapter = netdev->priv;

	WARN_ON(test_bit(__E1000_RESETTING, &adapter->flags));
	e1000_down(adapter);
	e1000_power_down_phy(adapter);
	e1000_free_irq(adapter);

	e1000_free_all_tx_resources(adapter);
	e1000_free_all_rx_resources(adapter);


	/* If AMT is enabled, let the firmware know that the network
	 * interface is now closed */
	if (adapter->hw.mac_type == e1000_82573 &&
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
static boolean_t
e1000_check_64k_bound(struct e1000_adapter *adapter,
		      void *start, unsigned long len)
{
	unsigned long begin = (unsigned long) start;
	unsigned long end = begin + len;

	/* First rev 82545 and 82546 need to not allow any memory
	 * write location to cross 64k boundary due to errata 23 */
	if (adapter->hw.mac_type == e1000_82545 ||
	    adapter->hw.mac_type == e1000_82546) {
		return ((begin ^ (end - 1)) >> 16) != 0 ? FALSE : TRUE;
	}

	return TRUE;
}

/**
 * e1000_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 * @txdr:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/

static int
e1000_setup_tx_resources(struct e1000_adapter *adapter,
			 struct e1000_tx_ring *txdr)
{
	struct pci_dev *pdev = adapter->pdev;
	int size;

	size = sizeof(struct e1000_buffer) * txdr->count;
	txdr->buffer_info = vmalloc(size);
	if (!txdr->buffer_info) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the transmit descriptor ring\n");
		return -ENOMEM;
	}
	memset(txdr->buffer_info, 0, size);

	/* round up to nearest 4K */

	txdr->size = txdr->count * sizeof(struct e1000_tx_desc);
	E1000_ROUNDUP(txdr->size, 4096);

	txdr->desc = pci_alloc_consistent(pdev, txdr->size, &txdr->dma);
	if (!txdr->desc) {
setup_tx_desc_die:
		vfree(txdr->buffer_info);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the transmit descriptor ring\n");
		return -ENOMEM;
	}

	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, txdr->desc, txdr->size)) {
		void *olddesc = txdr->desc;
		dma_addr_t olddma = txdr->dma;
		DPRINTK(TX_ERR, ERR, "txdr align check failed: %u bytes "
				     "at %p\n", txdr->size, txdr->desc);
		/* Try again, without freeing the previous */
		txdr->desc = pci_alloc_consistent(pdev, txdr->size, &txdr->dma);
		/* Failed allocation, critical failure */
		if (!txdr->desc) {
			pci_free_consistent(pdev, txdr->size, olddesc, olddma);
			goto setup_tx_desc_die;
		}

		if (!e1000_check_64k_bound(adapter, txdr->desc, txdr->size)) {
			/* give up */
			pci_free_consistent(pdev, txdr->size, txdr->desc,
					    txdr->dma);
			pci_free_consistent(pdev, txdr->size, olddesc, olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate aligned memory "
				"for the transmit descriptor ring\n");
			vfree(txdr->buffer_info);
			return -ENOMEM;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, txdr->size, olddesc, olddma);
		}
	}
	memset(txdr->desc, 0, txdr->size);

	txdr->next_to_use = 0;
	txdr->next_to_clean = 0;
	rtdm_lock_init(&txdr->tx_lock);

	return 0;
}

/**
 * e1000_setup_all_tx_resources - wrapper to allocate Tx resources
 *				  (Descriptors) for all queues
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/

int
e1000_setup_all_tx_resources(struct e1000_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		err = e1000_setup_tx_resources(adapter, &adapter->tx_ring[i]);
		if (err) {
			DPRINTK(PROBE, ERR,
				"Allocation for Tx Queue %u failed\n", i);
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

static void
e1000_configure_tx(struct e1000_adapter *adapter)
{
	uint64_t tdba;
	struct e1000_hw *hw = &adapter->hw;
	uint32_t tdlen, tctl, tipg, tarc;
	uint32_t ipgr1, ipgr2;

	/* Setup the HW Tx Head and Tail descriptor pointers */

	switch (adapter->num_tx_queues) {
	case 1:
	default:
		tdba = adapter->tx_ring[0].dma;
		tdlen = adapter->tx_ring[0].count *
			sizeof(struct e1000_tx_desc);
		E1000_WRITE_REG(hw, TDLEN, tdlen);
		E1000_WRITE_REG(hw, TDBAH, (tdba >> 32));
		E1000_WRITE_REG(hw, TDBAL, (tdba & 0x00000000ffffffffULL));
		E1000_WRITE_REG(hw, TDT, 0);
		E1000_WRITE_REG(hw, TDH, 0);
		adapter->tx_ring[0].tdh = E1000_TDH;
		adapter->tx_ring[0].tdt = E1000_TDT;
		break;
	}

	/* Set the default values for the Tx Inter Packet Gap timer */

	if (hw->media_type == e1000_media_type_fiber ||
	    hw->media_type == e1000_media_type_internal_serdes)
		tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
	else
		tipg = DEFAULT_82543_TIPG_IPGT_COPPER;

	switch (hw->mac_type) {
	case e1000_82542_rev2_0:
	case e1000_82542_rev2_1:
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
	E1000_WRITE_REG(hw, TIPG, tipg);

	/* Set the Tx Interrupt Delay register */

	E1000_WRITE_REG(hw, TIDV, adapter->tx_int_delay);
	if (hw->mac_type >= e1000_82540)
		E1000_WRITE_REG(hw, TADV, adapter->tx_abs_int_delay);

	/* Program the Transmit Control Register */

	tctl = E1000_READ_REG(hw, TCTL);

	tctl &= ~E1000_TCTL_CT;
	tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC |
		(E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);

#ifdef DISABLE_MULR
	/* disable Multiple Reads for debugging */
	tctl &= ~E1000_TCTL_MULR;
#endif

	if (hw->mac_type == e1000_82571 || hw->mac_type == e1000_82572) {
		tarc = E1000_READ_REG(hw, TARC0);
		tarc |= ((1 << 25) | (1 << 21));
		E1000_WRITE_REG(hw, TARC0, tarc);
		tarc = E1000_READ_REG(hw, TARC1);
		tarc |= (1 << 25);
		if (tctl & E1000_TCTL_MULR)
			tarc &= ~(1 << 28);
		else
			tarc |= (1 << 28);
		E1000_WRITE_REG(hw, TARC1, tarc);
	} else if (hw->mac_type == e1000_80003es2lan) {
		tarc = E1000_READ_REG(hw, TARC0);
		tarc |= 1;
		if (hw->media_type == e1000_media_type_internal_serdes)
			tarc |= (1 << 20);
		E1000_WRITE_REG(hw, TARC0, tarc);
		tarc = E1000_READ_REG(hw, TARC1);
		tarc |= 1;
		E1000_WRITE_REG(hw, TARC1, tarc);
	}

	e1000_config_collision_dist(hw);

	/* Setup Transmit Descriptor Settings for eop descriptor */
	adapter->txd_cmd = E1000_TXD_CMD_IDE | E1000_TXD_CMD_EOP |
		E1000_TXD_CMD_IFCS;

	if (hw->mac_type < e1000_82543)
		adapter->txd_cmd |= E1000_TXD_CMD_RPS;
	else
		adapter->txd_cmd |= E1000_TXD_CMD_RS;

	/* Cache if we're 82544 running in PCI-X because we'll
	 * need this to apply a workaround later in the send path. */
	if (hw->mac_type == e1000_82544 &&
	    hw->bus_type == e1000_bus_type_pcix)
		adapter->pcix_82544 = 1;

	E1000_WRITE_REG(hw, TCTL, tctl);

}

/**
 * e1000_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: board private structure
 * @rxdr:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/

static int
e1000_setup_rx_resources(struct e1000_adapter *adapter,
			 struct e1000_rx_ring *rxdr)
{
	struct pci_dev *pdev = adapter->pdev;
	int size, desc_len;

	size = sizeof(struct e1000_buffer) * rxdr->count;
	rxdr->buffer_info = vmalloc(size);
	if (!rxdr->buffer_info) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(rxdr->buffer_info, 0, size);

	size = sizeof(struct e1000_ps_page) * rxdr->count;
	rxdr->ps_page = kmalloc(size, GFP_KERNEL);
	if (!rxdr->ps_page) {
		vfree(rxdr->buffer_info);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(rxdr->ps_page, 0, size);

	size = sizeof(struct e1000_ps_page_dma) * rxdr->count;
	rxdr->ps_page_dma = kmalloc(size, GFP_KERNEL);
	if (!rxdr->ps_page_dma) {
		vfree(rxdr->buffer_info);
		kfree(rxdr->ps_page);
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(rxdr->ps_page_dma, 0, size);

	if (adapter->hw.mac_type <= e1000_82547_rev_2)
		desc_len = sizeof(struct e1000_rx_desc);
	else
		desc_len = sizeof(union e1000_rx_desc_packet_split);

	/* Round up to nearest 4K */

	rxdr->size = rxdr->count * desc_len;
	E1000_ROUNDUP(rxdr->size, 4096);

	rxdr->desc = pci_alloc_consistent(pdev, rxdr->size, &rxdr->dma);

	if (!rxdr->desc) {
		DPRINTK(PROBE, ERR,
		"Unable to allocate memory for the receive descriptor ring\n");
setup_rx_desc_die:
		vfree(rxdr->buffer_info);
		kfree(rxdr->ps_page);
		kfree(rxdr->ps_page_dma);
		return -ENOMEM;
	}

	/* Fix for errata 23, can't cross 64kB boundary */
	if (!e1000_check_64k_bound(adapter, rxdr->desc, rxdr->size)) {
		void *olddesc = rxdr->desc;
		dma_addr_t olddma = rxdr->dma;
		DPRINTK(RX_ERR, ERR, "rxdr align check failed: %u bytes "
				     "at %p\n", rxdr->size, rxdr->desc);
		/* Try again, without freeing the previous */
		rxdr->desc = pci_alloc_consistent(pdev, rxdr->size, &rxdr->dma);
		/* Failed allocation, critical failure */
		if (!rxdr->desc) {
			pci_free_consistent(pdev, rxdr->size, olddesc, olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate memory "
				"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		}

		if (!e1000_check_64k_bound(adapter, rxdr->desc, rxdr->size)) {
			/* give up */
			pci_free_consistent(pdev, rxdr->size, rxdr->desc,
					    rxdr->dma);
			pci_free_consistent(pdev, rxdr->size, olddesc, olddma);
			DPRINTK(PROBE, ERR,
				"Unable to allocate aligned memory "
				"for the receive descriptor ring\n");
			goto setup_rx_desc_die;
		} else {
			/* Free old allocation, new allocation was successful */
			pci_free_consistent(pdev, rxdr->size, olddesc, olddma);
		}
	}
	memset(rxdr->desc, 0, rxdr->size);

	rxdr->next_to_clean = 0;
	rxdr->next_to_use = 0;

	return 0;
}

/**
 * e1000_setup_all_rx_resources - wrapper to allocate Rx resources
 *				  (Descriptors) for all queues
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/

int
e1000_setup_all_rx_resources(struct e1000_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		err = e1000_setup_rx_resources(adapter, &adapter->rx_ring[i]);
		if (err) {
			DPRINTK(PROBE, ERR,
				"Allocation for Rx Queue %u failed\n", i);
			break;
		}
	}

	return err;
}

/**
 * e1000_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 **/
#define PAGE_USE_COUNT(S) (((S) >> PAGE_SHIFT) + \
			(((S) & (PAGE_SIZE - 1)) ? 1 : 0))
static void
e1000_setup_rctl(struct e1000_adapter *adapter)
{
	uint32_t rctl;
#ifndef CONFIG_E1000_DISABLE_PACKET_SPLIT
	uint32_t pages = 0;
#endif

	rctl = E1000_READ_REG(&adapter->hw, RCTL);

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);

	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
		E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
		(adapter->hw.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* FIXME: disable the stripping of CRC because it breaks
	 * BMC firmware connected over SMBUS
	if (adapter->hw.mac_type > e1000_82543)
		rctl |= E1000_RCTL_SECRC;
	*/

	if (adapter->hw.tbi_compatibility_on == 1)
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

	adapter->rx_ps_pages = 0;

	E1000_WRITE_REG(&adapter->hw, RCTL, rctl);
}

/**
 * e1000_configure_rx - Configure 8254x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/

static void
e1000_configure_rx(struct e1000_adapter *adapter)
{
	uint64_t rdba;
	struct e1000_hw *hw = &adapter->hw;
	uint32_t rdlen, rctl, rxcsum, ctrl_ext;

	{
		rdlen = adapter->rx_ring[0].count *
			sizeof(struct e1000_rx_desc);
		adapter->clean_rx = NULL; /* unused */
		adapter->alloc_rx_buf = e1000_alloc_rx_buffers;
	}

	/* disable receives while setting up the descriptors */
	rctl = E1000_READ_REG(hw, RCTL);
	E1000_WRITE_REG(hw, RCTL, rctl & ~E1000_RCTL_EN);

	/* set the Receive Delay Timer Register */
	E1000_WRITE_REG(hw, RDTR, adapter->rx_int_delay);

	if (hw->mac_type >= e1000_82540) {
		E1000_WRITE_REG(hw, RADV, adapter->rx_abs_int_delay);
		if (adapter->itr > 1)
			E1000_WRITE_REG(hw, ITR,
				1000000000 / (adapter->itr * 256));
	}

	if (hw->mac_type >= e1000_82571) {
		ctrl_ext = E1000_READ_REG(hw, CTRL_EXT);
		/* Reset delay timers after every interrupt */
		ctrl_ext |= E1000_CTRL_EXT_INT_TIMER_CLR;
		E1000_WRITE_REG(hw, CTRL_EXT, ctrl_ext);
		E1000_WRITE_REG(hw, IAM, ~0);
		E1000_WRITE_FLUSH(hw);
	}

	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring */
	switch (adapter->num_rx_queues) {
	case 1:
	default:
		rdba = adapter->rx_ring[0].dma;
		E1000_WRITE_REG(hw, RDLEN, rdlen);
		E1000_WRITE_REG(hw, RDBAH, (rdba >> 32));
		E1000_WRITE_REG(hw, RDBAL, (rdba & 0x00000000ffffffffULL));
		E1000_WRITE_REG(hw, RDT, 0);
		E1000_WRITE_REG(hw, RDH, 0);
		adapter->rx_ring[0].rdh = E1000_RDH;
		adapter->rx_ring[0].rdt = E1000_RDT;
		break;
	}

	/* Enable 82543 Receive Checksum Offload for TCP and UDP */
	if (hw->mac_type >= e1000_82543) {
		rxcsum = E1000_READ_REG(hw, RXCSUM);
		if (adapter->rx_csum == TRUE) {
			rxcsum |= E1000_RXCSUM_TUOFL;

		} else {
			rxcsum &= ~E1000_RXCSUM_TUOFL;
			/* don't need to clear IPPCSE as it defaults to 0 */
		}
		E1000_WRITE_REG(hw, RXCSUM, rxcsum);
	}


	/* Enable Receives */
	E1000_WRITE_REG(hw, RCTL, rctl);
}

/**
 * e1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/

static void
e1000_free_tx_resources(struct e1000_adapter *adapter,
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

void
e1000_free_all_tx_resources(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		e1000_free_tx_resources(adapter, &adapter->tx_ring[i]);
}

static void
e1000_unmap_and_free_tx_resource(struct e1000_adapter *adapter,
			struct e1000_buffer *buffer_info)
{
	if (buffer_info->dma) {
		pci_unmap_page(adapter->pdev,
				buffer_info->dma,
				buffer_info->length,
				PCI_DMA_TODEVICE);
	}
	if (buffer_info->skb)
		kfree_rtskb(buffer_info->skb);
	memset(buffer_info, 0, sizeof(struct e1000_buffer));
}

/**
 * e1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 * @tx_ring: ring to be cleaned
 **/

static void
e1000_clean_tx_ring(struct e1000_adapter *adapter,
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

static void
e1000_clean_all_tx_rings(struct e1000_adapter *adapter)
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

static void
e1000_free_rx_resources(struct e1000_adapter *adapter,
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

void
e1000_free_all_rx_resources(struct e1000_adapter *adapter)
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

static void
e1000_clean_rx_ring(struct e1000_adapter *adapter,
		    struct e1000_rx_ring *rx_ring)
{
	struct e1000_buffer *buffer_info;
	struct pci_dev *pdev = adapter->pdev;
	unsigned long size;
	unsigned int i;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		if (buffer_info->skb) {
			pci_unmap_single(pdev,
					 buffer_info->dma,
					 buffer_info->length,
					 PCI_DMA_FROMDEVICE);

			kfree_rtskb(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct e1000_buffer) * rx_ring->count;
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

static void
e1000_clean_all_rx_rings(struct e1000_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		e1000_clean_rx_ring(adapter, &adapter->rx_ring[i]);
}

/* The 82542 2.0 (revision 2) needs to have the receive unit in reset
 * and memory write and invalidate disabled for certain operations
 */
static void
e1000_enter_82542_rst(struct e1000_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	uint32_t rctl;

	e1000_pci_clear_mwi(&adapter->hw);

	rctl = E1000_READ_REG(&adapter->hw, RCTL);
	rctl |= E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);

	if (rtnetif_running(netdev))
		e1000_clean_all_rx_rings(adapter);
}

static void
e1000_leave_82542_rst(struct e1000_adapter *adapter)
{
	struct rtnet_device *netdev = adapter->netdev;
	uint32_t rctl;

	rctl = E1000_READ_REG(&adapter->hw, RCTL);
	rctl &= ~E1000_RCTL_RST;
	E1000_WRITE_REG(&adapter->hw, RCTL, rctl);
	E1000_WRITE_FLUSH(&adapter->hw);
	mdelay(5);

	if (adapter->hw.pci_cmd_word & PCI_COMMAND_INVALIDATE)
		e1000_pci_set_mwi(&adapter->hw);

	if (rtnetif_running(netdev)) {
		/* No need to loop, because 82542 supports only 1 queue */
		struct e1000_rx_ring *ring = &adapter->rx_ring[0];
		e1000_configure_rx(adapter);
		adapter->alloc_rx_buf(adapter, ring, E1000_DESC_UNUSED(ring));
	}
}

/**
 * e1000_set_multi - Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/

static void
e1000_set_multi(struct rtnet_device *netdev)
{
	struct e1000_adapter *adapter = netdev->priv;
	struct e1000_hw *hw = &adapter->hw;
	uint32_t rctl;
	int i, rar_entries = E1000_RAR_ENTRIES;
	int mta_reg_count = (hw->mac_type == e1000_ich8lan) ?
				E1000_NUM_MTA_REGISTERS_ICH8LAN :
				E1000_NUM_MTA_REGISTERS;

	if (adapter->hw.mac_type == e1000_ich8lan)
		rar_entries = E1000_RAR_ENTRIES_ICH8LAN;

	/* reserve RAR[14] for LAA over-write work-around */
	if (adapter->hw.mac_type == e1000_82571)
		rar_entries--;

	/* Check for Promiscuous and All Multicast modes */

	rctl = E1000_READ_REG(hw, RCTL);

	if (netdev->flags & IFF_PROMISC) {
		rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
	} else if (netdev->flags & IFF_ALLMULTI) {
		rctl |= E1000_RCTL_MPE;
		rctl &= ~E1000_RCTL_UPE;
	} else {
		rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
	}

	E1000_WRITE_REG(hw, RCTL, rctl);

	/* 82542 2.0 needs to be in reset to write receive address registers */

	if (hw->mac_type == e1000_82542_rev2_0)
		e1000_enter_82542_rst(adapter);

	/* load the first 14 multicast address into the exact filters 1-14
	 * RAR 0 is used for the station MAC adddress
	 * if there are not 14 addresses, go ahead and clear the filters
	 * -- with 82571 controllers only 0-13 entries are filled here
	 */

	for (i = 1; i < rar_entries; i++) {
		E1000_WRITE_REG_ARRAY(hw, RA, i << 1, 0);
		E1000_WRITE_FLUSH(hw);
		E1000_WRITE_REG_ARRAY(hw, RA, (i << 1) + 1, 0);
		E1000_WRITE_FLUSH(hw);
	}

	/* clear the old settings from the multicast hash table */

	for (i = 0; i < mta_reg_count; i++) {
		E1000_WRITE_REG_ARRAY(hw, MTA, i, 0);
		E1000_WRITE_FLUSH(hw);
	}

	if (hw->mac_type == e1000_82542_rev2_0)
		e1000_leave_82542_rst(adapter);
}

/**
 * e1000_update_phy_info_task - get phy info
 * @work: work struct contained inside adapter struct
 *
 * Need to wait a few seconds after link up to get diagnostic information from
 * the phy
 */
static void e1000_update_phy_info_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     phy_info_task.work);
	e1000_phy_get_info(&adapter->hw, &adapter->phy_info);
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
	struct rtnet_device *netdev = adapter->netdev;
	uint32_t tctl;

	if (atomic_read(&adapter->tx_fifo_stall)) {
		if ((E1000_READ_REG(&adapter->hw, TDT) ==
		    E1000_READ_REG(&adapter->hw, TDH)) &&
		   (E1000_READ_REG(&adapter->hw, TDFT) ==
		    E1000_READ_REG(&adapter->hw, TDFH)) &&
		   (E1000_READ_REG(&adapter->hw, TDFTS) ==
		    E1000_READ_REG(&adapter->hw, TDFHS))) {
			tctl = E1000_READ_REG(&adapter->hw, TCTL);
			E1000_WRITE_REG(&adapter->hw, TCTL,
					tctl & ~E1000_TCTL_EN);
			E1000_WRITE_REG(&adapter->hw, TDFT,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, TDFH,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, TDFTS,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, TDFHS,
					adapter->tx_head_addr);
			E1000_WRITE_REG(&adapter->hw, TCTL, tctl);
			E1000_WRITE_FLUSH(&adapter->hw);

			adapter->tx_fifo_head = 0;
			atomic_set(&adapter->tx_fifo_stall, 0);
			rtnetif_wake_queue(netdev);
		} else {
			schedule_delayed_work(&adapter->fifo_stall_task, 1);
		}
	}
}

/**
 * e1000_watchdog - work function
 * @work: work struct contained inside adapter struct
 **/
static void e1000_watchdog(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     watchdog_task.work);
	struct rtnet_device *netdev = adapter->netdev;
	struct e1000_tx_ring *txdr = adapter->tx_ring;
	uint32_t link, tctl;
	int32_t ret_val;

	ret_val = e1000_check_for_link(&adapter->hw);
	if ((ret_val == E1000_ERR_PHY) &&
	    (adapter->hw.phy_type == e1000_phy_igp_3) &&
	    (E1000_READ_REG(&adapter->hw, CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kumeran_lock_loss_workaround() */
		DPRINTK(LINK, INFO,
			"Gigabit has been disabled, downgrading speed\n");
	}
	if (adapter->hw.mac_type == e1000_82573) {
		e1000_enable_tx_pkt_filtering(&adapter->hw);
	}

	if ((adapter->hw.media_type == e1000_media_type_internal_serdes) &&
	   !(E1000_READ_REG(&adapter->hw, TXCW) & E1000_TXCW_ANE))
		link = !adapter->hw.serdes_link_down;
	else
		link = E1000_READ_REG(&adapter->hw, STATUS) & E1000_STATUS_LU;

	if (link) {
		if (!rtnetif_carrier_ok(netdev)) {
			boolean_t txb2b = 1;
			e1000_get_speed_and_duplex(&adapter->hw,
						   &adapter->link_speed,
						   &adapter->link_duplex);

			DPRINTK(LINK, INFO, "NIC Link is Up %d Mbps %s\n",
			       adapter->link_speed,
			       adapter->link_duplex == FULL_DUPLEX ?
			       "Full Duplex" : "Half Duplex");

			/* tweak tx_queue_len according to speed/duplex
			 * and adjust the timeout factor */
			// TODO makoehre netdev->tx_queue_len = adapter->tx_queue_len;
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
			case SPEED_10:
				txb2b = 0;
				// TODO makoehre netdev->tx_queue_len = 10;
				adapter->tx_timeout_factor = 8;
				break;
			case SPEED_100:
				txb2b = 0;
				// TODO makoehre netdev->tx_queue_len = 100;
				/* maybe add some timeout factor ? */
				break;
			}

			if ((adapter->hw.mac_type == e1000_82571 ||
			     adapter->hw.mac_type == e1000_82572) &&
			    txb2b == 0) {
#define SPEED_MODE_BIT (1 << 21)
				uint32_t tarc0;
				tarc0 = E1000_READ_REG(&adapter->hw, TARC0);
				tarc0 &= ~SPEED_MODE_BIT;
				E1000_WRITE_REG(&adapter->hw, TARC0, tarc0);
			}


			/* enable transmits in the hardware, need to do this
			 * after setting TARC0 */
			tctl = E1000_READ_REG(&adapter->hw, TCTL);
			tctl |= E1000_TCTL_EN;
			E1000_WRITE_REG(&adapter->hw, TCTL, tctl);

			rtnetif_carrier_on(netdev);
			rtnetif_wake_queue(netdev);
			schedule_delayed_work(&adapter->phy_info_task, 2 * HZ);
			adapter->smartspeed = 0;
		}
	} else {
		if (rtnetif_carrier_ok(netdev)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			DPRINTK(LINK, INFO, "NIC Link is Down\n");
			rtnetif_carrier_off(netdev);
			rtnetif_stop_queue(netdev);
			schedule_delayed_work(&adapter->phy_info_task, 2 * HZ);

			/* 80003ES2LAN workaround--
			 * For packet buffer work-around on link down event;
			 * disable receives in the ISR and
			 * reset device here in the watchdog
			 */
			if (adapter->hw.mac_type == e1000_80003es2lan)
				/* reset device */
				schedule_work(&adapter->reset_task);
		}

		e1000_smartspeed(adapter);
	}


	adapter->hw.tx_packet_delta = adapter->stats.tpt - adapter->tpt_old;
	adapter->tpt_old = adapter->stats.tpt;
	adapter->hw.collision_delta = adapter->stats.colc - adapter->colc_old;
	adapter->colc_old = adapter->stats.colc;

	adapter->gorcl = adapter->stats.gorcl - adapter->gorcl_old;
	adapter->gorcl_old = adapter->stats.gorcl;
	adapter->gotcl = adapter->stats.gotcl - adapter->gotcl_old;
	adapter->gotcl_old = adapter->stats.gotcl;

	// e1000_update_adaptive(&adapter->hw);

	if (!rtnetif_carrier_ok(netdev)) {
		if (E1000_DESC_UNUSED(txdr) + 1 < txdr->count) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context). */
			adapter->tx_timeout_count++;
			schedule_work(&adapter->reset_task);
		}
	}

	/* Dynamic mode for Interrupt Throttle Rate (ITR) */
	if (adapter->hw.mac_type >= e1000_82540 && adapter->itr == 1) {
		/* Symmetric Tx/Rx gets a reduced ITR=2000; Total
		 * asymmetrical Tx or Rx gets ITR=8000; everyone
		 * else is between 2000-8000. */
		uint32_t goc = (adapter->gotcl + adapter->gorcl) / 10000;
		uint32_t dif = (adapter->gotcl > adapter->gorcl ?
			adapter->gotcl - adapter->gorcl :
			adapter->gorcl - adapter->gotcl) / 10000;
		uint32_t itr = goc > 0 ? (dif * 6000 / goc + 2000) : 8000;
		E1000_WRITE_REG(&adapter->hw, ITR, 1000000000 / (itr * 256));
	}

	/* Cause software interrupt to ensure rx ring is cleaned */
	E1000_WRITE_REG(&adapter->hw, ICS, E1000_ICS_RXDMT0);

	/* Force detection of hung controller every watchdog period */
	adapter->detect_tx_hung = TRUE;

	/* With 82571 controllers, LAA may be overwritten due to controller
	 * reset from the other port. Set the appropriate LAA in RAR[0] */
	if (adapter->hw.mac_type == e1000_82571 && adapter->hw.laa_is_present)
		e1000_rar_set(&adapter->hw, adapter->hw.mac_addr, 0);

	/* Reschedule the task */
	schedule_delayed_work(&adapter->watchdog_task, 2 * HZ);
}

#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16


static boolean_t
e1000_tx_csum(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring,
	      struct rtskb *skb)
{
	struct e1000_context_desc *context_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i;
	uint8_t css;

	if (likely(skb->ip_summed == CHECKSUM_PARTIAL)) {
		css = skb->h.raw - skb->data;

		i = tx_ring->next_to_use;
		buffer_info = &tx_ring->buffer_info[i];
		context_desc = E1000_CONTEXT_DESC(*tx_ring, i);

		context_desc->upper_setup.tcp_fields.tucss = css;
		context_desc->upper_setup.tcp_fields.tucso = css + skb->csum;
		context_desc->upper_setup.tcp_fields.tucse = 0;
		context_desc->tcp_seg_setup.data = 0;
		context_desc->cmd_and_length = cpu_to_le32(E1000_TXD_CMD_DEXT);

		buffer_info->time_stamp = jiffies;

		if (unlikely(++i == tx_ring->count)) i = 0;
		tx_ring->next_to_use = i;

		return TRUE;
	}

	return FALSE;
}

#define E1000_MAX_TXD_PWR	12
#define E1000_MAX_DATA_PER_TXD	(1<<E1000_MAX_TXD_PWR)

static int
e1000_tx_map(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring,
	     struct rtskb *skb, unsigned int first, unsigned int max_per_txd,
	     unsigned int nr_frags, unsigned int mss)
{
	struct e1000_buffer *buffer_info;
	unsigned int len = skb->len;
	unsigned int offset = 0, size, count = 0, i;

	i = tx_ring->next_to_use;

	while (len) {
		buffer_info = &tx_ring->buffer_info[i];
		size = min(len, max_per_txd);
		/* work-around for errata 10 and it applies
		 * to all controllers in PCI-X mode
		 * The fix is to make sure that the first descriptor of a
		 * packet is smaller than 2048 - 16 - 16 (or 2016) bytes
		 */
		if (unlikely((adapter->hw.bus_type == e1000_bus_type_pcix) &&
				(size > 2015) && count == 0))
			size = 2015;

		/* Workaround for potential 82544 hang in PCI-X.  Avoid
		 * terminating buffers within evenly-aligned dwords. */
		if (unlikely(adapter->pcix_82544 &&
		   !((unsigned long)(skb->data + offset + size - 1) & 4) &&
		   size > 4))
			size -= 4;

		buffer_info->length = size;
		buffer_info->dma =
			pci_map_single(adapter->pdev,
				skb->data + offset,
				size,
				PCI_DMA_TODEVICE);
		buffer_info->time_stamp = jiffies;

		len -= size;
		offset += size;
		count++;
		if (unlikely(++i == tx_ring->count)) i = 0;
	}


	i = (i == 0) ? tx_ring->count - 1 : i - 1;
	tx_ring->buffer_info[i].skb = skb;
	tx_ring->buffer_info[first].next_to_watch = i;

	return count;
}

static void
e1000_tx_queue(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring,
	       int tx_flags, int count, nanosecs_abs_t *xmit_stamp)
{
	struct e1000_tx_desc *tx_desc = NULL;
	struct e1000_buffer *buffer_info;
	uint32_t txd_upper = 0, txd_lower = E1000_TXD_CMD_IFCS;
	unsigned int i;


	if (likely(tx_flags & E1000_TX_FLAGS_CSUM)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
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

	if (xmit_stamp)
		*xmit_stamp = cpu_to_be64(rtdm_clock_read() + *xmit_stamp);

	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64). */
	wmb();

	tx_ring->next_to_use = i;
	writel(i, adapter->hw.hw_addr + tx_ring->tdt);
}

/**
 * 82547 workaround to avoid controller hang in half-duplex environment.
 * The workaround is to avoid queuing a large packet that would span
 * the internal Tx FIFO ring boundary by notifying the stack to resend
 * the packet at a later time.  This gives the Tx FIFO an opportunity to
 * flush all packets.  When that occurs, we reset the Tx FIFO pointers
 * to the beginning of the Tx FIFO.
 **/

#define E1000_FIFO_HDR			0x10
#define E1000_82547_PAD_LEN		0x3E0

static int
e1000_82547_fifo_workaround(struct e1000_adapter *adapter, struct rtskb *skb)
{
	uint32_t fifo_space = adapter->tx_fifo_size - adapter->tx_fifo_head;
	uint32_t skb_fifo_len = skb->len + E1000_FIFO_HDR;

	E1000_ROUNDUP(skb_fifo_len, E1000_FIFO_HDR);

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
static int
e1000_transfer_dhcp_info(struct e1000_adapter *adapter, struct rtskb *skb)
{
	struct e1000_hw *hw =  &adapter->hw;
	uint16_t length, offset;
	if (skb->len > MINIMUM_DHCP_PACKET_SIZE) {
		struct ethhdr *eth = (struct ethhdr *) skb->data;
		if ((htons(ETH_P_IP) == eth->h_proto)) {
			const struct iphdr *ip =
				(struct iphdr *)((uint8_t *)skb->data+14);
			if (IPPROTO_UDP == ip->protocol) {
				struct udphdr *udp =
					(struct udphdr *)((uint8_t *)ip +
						(ip->ihl << 2));
				if (ntohs(udp->dest) == 67) {
					offset = (uint8_t *)udp + 8 - skb->data;
					length = skb->len - offset;

					return e1000_mng_write_dhcp_info(hw,
							(uint8_t *)udp + 8,
							length);
				}
			}
		}
	}
	return 0;
}

#define TXD_USE_COUNT(S, X) (((S) >> (X)) + 1 )
static int
e1000_xmit_frame(struct rtskb *skb, struct rtnet_device *netdev)
{
	struct e1000_adapter *adapter = netdev->priv;
	struct e1000_tx_ring *tx_ring;
	unsigned int first, max_per_txd = E1000_MAX_DATA_PER_TXD;
	unsigned int max_txd_pwr = E1000_MAX_TXD_PWR;
	unsigned int tx_flags = 0;
	unsigned int len = skb->len;
	rtdm_lockctx_t context;
	unsigned int nr_frags = 0;
	unsigned int mss = 0;
	int count = 0;

	/* This goes back to the question of how to logically map a tx queue
	 * to a flow.  Right now, performance is impacted slightly negatively
	 * if using multiple tx queues.  If the stack breaks away from a
	 * single qdisc implementation, we can look at this again. */
	tx_ring = adapter->tx_ring;

	if (unlikely(skb->len <= 0)) {
		kfree_rtskb(skb);
		return NETDEV_TX_OK;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		count++;


	count += TXD_USE_COUNT(len, max_txd_pwr);

	if (adapter->pcix_82544)
		count++;

	/* work-around for errata 10 and it applies to all controllers
	 * in PCI-X mode, so add one more descriptor to the count
	 */
	if (unlikely((adapter->hw.bus_type == e1000_bus_type_pcix) &&
			(len > 2015)))
		count++;


	if (adapter->hw.tx_pkt_filtering &&
	    (adapter->hw.mac_type == e1000_82573))
		e1000_transfer_dhcp_info(adapter, skb);

	rtdm_lock_get_irqsave(&tx_ring->tx_lock, context);

	/* need: count + 2 desc gap to keep tail from touching
	 * head, otherwise try next time */
	if (unlikely(E1000_DESC_UNUSED(tx_ring) < count + 2)) {
		rtnetif_stop_queue(netdev);
		rtdm_lock_put_irqrestore(&tx_ring->tx_lock, context);
		rtdm_printk("FATAL: rt_e1000 ran into tail close to head situation!\n");
		return NETDEV_TX_BUSY;
	}

	if (unlikely(adapter->hw.mac_type == e1000_82547)) {
		if (unlikely(e1000_82547_fifo_workaround(adapter, skb))) {
			rtnetif_stop_queue(netdev);
			rtdm_lock_put_irqrestore(&tx_ring->tx_lock, context);

			/* FIXME: warn the user earlier, i.e. on startup if
			   half-duplex is detected! */
			rtdm_printk("FATAL: rt_e1000 ran into 82547 "
				    "controller bug!\n");
			return NETDEV_TX_BUSY;
		}
	}

	first = tx_ring->next_to_use;

	if (likely(e1000_tx_csum(adapter, tx_ring, skb)))
		tx_flags |= E1000_TX_FLAGS_CSUM;

	e1000_tx_queue(adapter, tx_ring, tx_flags,
		       e1000_tx_map(adapter, tx_ring, skb, first,
				    max_per_txd, nr_frags, mss),
		       skb->xmit_stamp);

	rtdm_lock_put_irqrestore(&tx_ring->tx_lock, context);

	return NETDEV_TX_OK;
}

/**
 * e1000_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 * @pt_regs: CPU registers structure
 **/

static int
e1000_intr(rtdm_irq_t *irq_handle)
    /* int irq, void *data, struct pt_regs *regs) */
{

	struct rtnet_device *netdev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
	struct e1000_adapter *adapter = netdev->priv;
	struct e1000_hw *hw = &adapter->hw;
	uint32_t rctl, icr = E1000_READ_REG(hw, ICR);
	int i;
	nanosecs_abs_t time_stamp = rtdm_clock_read();

	if (unlikely(!icr)) {
		return RTDM_IRQ_NONE;  /* Not our interrupt */
	}
	if (unlikely(icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC))) {
		hw->get_link_status = 1;
		/* 80003ES2LAN workaround--
		 * For packet buffer work-around on link down event;
		 * disable receives here in the ISR and
		 * reset adapter in watchdog
		 */
		if (rtnetif_carrier_ok(netdev) &&
		    (adapter->hw.mac_type == e1000_80003es2lan)) {
			/* disable receives */
			rctl = E1000_READ_REG(hw, RCTL);
			E1000_WRITE_REG(hw, RCTL, rctl & ~E1000_RCTL_EN);
		}
		/* FIXME: we need to handle this via some yet-to-be-invented
		   error manager (Linux botton-half and/or kthread)
		mod_timer(&adapter->watchdog_timer, jiffies);*/
	}

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
	if (hw->mac_type == e1000_82547 || hw->mac_type == e1000_82547_rev_2) {
		atomic_inc(&adapter->irq_sem);
		E1000_WRITE_REG(hw, IMC, ~0);
	}

	adapter->data_received = 0;

	for (i = 0; i < E1000_MAX_INTR; i++)
		if (unlikely(!e1000_clean_rx_irq(adapter, adapter->rx_ring,
						 &time_stamp) &
		   !e1000_clean_tx_irq(adapter, adapter->tx_ring)))
			break;

	if (hw->mac_type == e1000_82547 || hw->mac_type == e1000_82547_rev_2)
		e1000_irq_enable(adapter);


	if (adapter->data_received)
		rt_mark_stack_mgr(netdev);
	return RTDM_IRQ_HANDLED;
}

/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 **/

static boolean_t
e1000_clean_tx_irq(struct e1000_adapter *adapter,
		   struct e1000_tx_ring *tx_ring)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct e1000_buffer *buffer_info;
	unsigned int i, eop;
	boolean_t cleaned = FALSE;

	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);

	while (eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) {
		for (cleaned = FALSE; !cleaned; ) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);

			e1000_unmap_and_free_tx_resource(adapter, buffer_info);
			memset(tx_desc, 0, sizeof(struct e1000_tx_desc));

			if (unlikely(++i == tx_ring->count)) i = 0;
		}


		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
	}

	tx_ring->next_to_clean = i;

#define TX_WAKE_THRESHOLD 32
	if (unlikely(cleaned && rtnetif_queue_stopped(netdev) &&
		     rtnetif_carrier_ok(netdev))) {
		rtdm_lock_get(&tx_ring->tx_lock);
		if (rtnetif_queue_stopped(netdev) &&
		    (E1000_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD))
			rtnetif_wake_queue(netdev);
		rtdm_lock_put(&tx_ring->tx_lock);
	}

	if (adapter->detect_tx_hung) {
		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i */
		adapter->detect_tx_hung = FALSE;
		if (tx_ring->buffer_info[eop].dma &&
		    time_after(jiffies, tx_ring->buffer_info[eop].time_stamp +
			       (adapter->tx_timeout_factor * HZ))
		    && !(E1000_READ_REG(&adapter->hw, STATUS) &
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
	return cleaned;
}

/**
 * e1000_rx_checksum - Receive Checksum Offload for 82543
 * @adapter:     board private structure
 * @status_err:  receive descriptor status and error fields
 * @csum:        receive descriptor csum field
 * @sk_buff:     socket buffer with received data
 **/

static void
e1000_rx_checksum(struct e1000_adapter *adapter,
		  uint32_t status_err, uint32_t csum,
		  struct rtskb *skb)
{
	uint16_t status = (uint16_t)status_err;
	uint8_t errors = (uint8_t)(status_err >> 24);
	skb->ip_summed = CHECKSUM_NONE;

	/* 82543 or newer only */
	if (unlikely(adapter->hw.mac_type < e1000_82543)) return;
	/* Ignore Checksum bit is set */
	if (unlikely(status & E1000_RXD_STAT_IXSM)) return;
	/* TCP/UDP checksum error bit is set */
	if (unlikely(errors & E1000_RXD_ERR_TCPE)) {
		/* let the stack verify checksum errors */
		adapter->hw_csum_err++;
		return;
	}
	/* TCP/UDP Checksum has not been calculated */
	if (adapter->hw.mac_type <= e1000_82547_rev_2) {
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
	} else if (adapter->hw.mac_type > e1000_82547_rev_2) {
		/* IP fragment with UDP payload */
		/* Hardware complements the payload checksum, so we undo it
		 * and then put the value in host order for further stack use.
		 */
		csum = ntohl(csum ^ 0xFFFF);
		skb->csum = csum;
		skb->ip_summed = CHECKSUM_PARTIAL;
	}
	adapter->hw_csum_good++;
}

/**
 * e1000_clean_rx_irq - Send received data up the network stack; legacy
 * @adapter: board private structure
 **/

static boolean_t
e1000_clean_rx_irq(struct e1000_adapter *adapter,
		   struct e1000_rx_ring *rx_ring,
		   nanosecs_abs_t *time_stamp)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc, *next_rxd;
	struct e1000_buffer *buffer_info, *next_buffer;
	uint32_t length;
	uint8_t last_byte;
	unsigned int i;
	int cleaned_count = 0;
	boolean_t cleaned = FALSE;

	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];

	while (rx_desc->status & E1000_RXD_STAT_DD) {
		struct rtskb *skb, *next_skb;
		u8 status;

		status = rx_desc->status;
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		prefetch(skb->data - NET_IP_ALIGN);

		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];
		next_skb = next_buffer->skb;
		prefetch(next_skb->data - NET_IP_ALIGN);

		cleaned = TRUE;
		cleaned_count++;
		pci_unmap_single(pdev,
				 buffer_info->dma,
				 buffer_info->length,
				 PCI_DMA_FROMDEVICE);

		length = le16_to_cpu(rx_desc->length);

		if (unlikely(!(status & E1000_RXD_STAT_EOP))) {
			/* All receives must fit into a single buffer */
			E1000_DBG("%s: Receive packet consumed multiple"
				  " buffers\n", netdev->name);
			/* recycle */
			buffer_info->skb = skb;
			goto next_desc;
		}

		if (unlikely(rx_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK)) {
			last_byte = *(skb->data + length - 1);
			if (TBI_ACCEPT(&adapter->hw, status,
				      rx_desc->errors, length, last_byte)) {
				length--;
			} else {
				/* recycle */
				buffer_info->skb = skb;
				goto next_desc;
			}
		}

		/* code added for copybreak, this should improve
		 * performance for small packets with large amounts
		 * of reassembly being done in the stack */
		rtskb_put(skb, length);

		/* end copybreak code */

		/* Receive Checksum Offload */
		e1000_rx_checksum(adapter,
				  (uint32_t)(status) |
				  ((uint32_t)(rx_desc->errors) << 24),
				  le16_to_cpu(rx_desc->csum), skb);

		skb->protocol = rt_eth_type_trans(skb, netdev);
		skb->time_stamp = *time_stamp;
		rtnetif_rx(skb);
		adapter->data_received = 1; // Set flag for the main interrupt routine

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

	return cleaned;
}

/**
 * e1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/

static void
e1000_alloc_rx_buffers(struct e1000_adapter *adapter,
		       struct e1000_rx_ring *rx_ring,
		       int cleaned_count)
{
	struct rtnet_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct e1000_rx_desc *rx_desc;
	struct e1000_buffer *buffer_info;
	struct rtskb *skb;
	unsigned int i;
	unsigned int bufsz = adapter->rx_buffer_len + NET_IP_ALIGN;

	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];

	while (cleaned_count--) {
		if (!(skb = buffer_info->skb))
			skb = rtnetdev_alloc_rtskb(netdev, bufsz);
		else {
			rtskb_trim(skb, 0);
			goto map_skb;
		}

		if (unlikely(!skb)) {
			/* Better luck next round */
			adapter->alloc_rx_buff_failed++;
			break;
		}

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
			struct rtskb *oldskb = skb;
			DPRINTK(RX_ERR, ERR, "skb align check failed: %u bytes "
					     "at %p\n", bufsz, skb->data);
			/* Try again, without freeing the previous */
			skb = rtnetdev_alloc_rtskb(netdev, bufsz);
			/* Failed allocation, critical failure */
			if (!skb) {
				kfree_rtskb(oldskb);
				break;
			}

			if (!e1000_check_64k_bound(adapter, skb->data, bufsz)) {
				/* give up */
				kfree_rtskb(skb);
				kfree_rtskb(oldskb);
				break; /* while !buffer_info->skb */
			} else {
				/* Use new allocation */
				kfree_rtskb(oldskb);
			}
		}
		/* Make buffer alignment 2 beyond a 16 byte boundary
		 * this will result in a 16 byte aligned IP header after
		 * the 14 byte MAC header is removed
		 */
		rtskb_reserve(skb, NET_IP_ALIGN);

		buffer_info->skb = skb;
		buffer_info->length = adapter->rx_buffer_len;
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
 * e1000_smartspeed - Workaround for SmartSpeed on 82541 and 82547 controllers.
 * @adapter:
 **/

static void
e1000_smartspeed(struct e1000_adapter *adapter)
{
	uint16_t phy_status;
	uint16_t phy_ctrl;

	if ((adapter->hw.phy_type != e1000_phy_igp) || !adapter->hw.autoneg ||
	   !(adapter->hw.autoneg_advertised & ADVERTISE_1000_FULL))
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
			   !e1000_read_phy_reg(&adapter->hw, PHY_CTRL,
					       &phy_ctrl)) {
				phy_ctrl |= (MII_CR_AUTO_NEG_EN |
					     MII_CR_RESTART_AUTO_NEG);
				e1000_write_phy_reg(&adapter->hw, PHY_CTRL,
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
		   !e1000_read_phy_reg(&adapter->hw, PHY_CTRL, &phy_ctrl)) {
			phy_ctrl |= (MII_CR_AUTO_NEG_EN |
				     MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(&adapter->hw, PHY_CTRL, phy_ctrl);
		}
	}
	/* Restart process after E1000_SMARTSPEED_MAX iterations */
	if (adapter->smartspeed++ == E1000_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}



void
e1000_pci_set_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = hw->back;
#ifdef HAVE_PCI_SET_MWI
	int ret_val = pci_set_mwi(adapter->pdev);

	if (ret_val)
		DPRINTK(PROBE, ERR, "Error in setting MWI\n");
#else
	pci_write_config_word(adapter->pdev, PCI_COMMAND,
			      adapter->hw.pci_cmd_word |
			      PCI_COMMAND_INVALIDATE);
#endif
}

void
e1000_pci_clear_mwi(struct e1000_hw *hw)
{
	struct e1000_adapter *adapter = hw->back;

#ifdef HAVE_PCI_SET_MWI
	pci_clear_mwi(adapter->pdev);
#else
	pci_write_config_word(adapter->pdev, PCI_COMMAND,
			      adapter->hw.pci_cmd_word &
			      ~PCI_COMMAND_INVALIDATE);
#endif
}

void
e1000_read_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *value)
{
	struct e1000_adapter *adapter = hw->back;

	pci_read_config_word(adapter->pdev, reg, value);
}

void
e1000_write_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *value)
{
	struct e1000_adapter *adapter = hw->back;

	pci_write_config_word(adapter->pdev, reg, *value);
}

uint32_t
e1000_io_read(struct e1000_hw *hw, unsigned long port)
{
	return inl(port);
}

void
e1000_io_write(struct e1000_hw *hw, unsigned long port, uint32_t value)
{
	outl(value, port);
}


int
e1000_set_spd_dplx(struct e1000_adapter *adapter, uint16_t spddplx)
{
	adapter->hw.autoneg = 0;

	/* Fiber NICs only allow 1000 gbps Full duplex */
	if ((adapter->hw.media_type == e1000_media_type_fiber) &&
		spddplx != (SPEED_1000 + DUPLEX_FULL)) {
		DPRINTK(PROBE, ERR, "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}

	switch (spddplx) {
	case SPEED_10 + DUPLEX_HALF:
		adapter->hw.forced_speed_duplex = e1000_10_half;
		break;
	case SPEED_10 + DUPLEX_FULL:
		adapter->hw.forced_speed_duplex = e1000_10_full;
		break;
	case SPEED_100 + DUPLEX_HALF:
		adapter->hw.forced_speed_duplex = e1000_100_half;
		break;
	case SPEED_100 + DUPLEX_FULL:
		adapter->hw.forced_speed_duplex = e1000_100_full;
		break;
	case SPEED_1000 + DUPLEX_FULL:
		adapter->hw.autoneg = 1;
		adapter->hw.autoneg_advertised = ADVERTISE_1000_FULL;
		break;
	case SPEED_1000 + DUPLEX_HALF: /* not supported */
	default:
		DPRINTK(PROBE, ERR, "Unsupported Speed/Duplex configuration\n");
		return -EINVAL;
	}
	return 0;
}
