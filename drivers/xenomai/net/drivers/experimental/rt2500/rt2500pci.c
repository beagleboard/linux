/* rt2500pci.c
 *
 * Copyright (C) 2004 - 2005 rt2x00-2.0.0-b3 SourceForge Project
 *			     <http://rt2x00.serialmonkey.com>
 *               2006        rtnet adaption by Daniel Gregorek
 *                           <dxg@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * Module: rt_rt2500pci
 * Abstract: rt2500pci device specific routines.
 * Supported chipsets: RT2560.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "rt2x00.h"
#include "rt2500pci.h"

#include <rtnet_port.h>

#ifdef DRV_NAME
#undef DRV_NAME
#define DRV_NAME			"rt_rt2500pci"
#endif /* DRV_NAME */

/* handler for direct register access from core module */
static int rt2x00_dev_register_access(struct _rt2x00_core * core,
				      int request,
				      u32 address,
				      u32 * value) {

    struct _rt2x00_pci * rt2x00pci = rt2x00_priv(core);
    u8 u8_value;

    switch(request) {
    case IOC_RTWLAN_REGREAD:
	rt2x00_register_read(rt2x00pci, address, value);
	break;
    case IOC_RTWLAN_REGWRITE:
	rt2x00_register_write(rt2x00pci, address, *value);
	break;
    case IOC_RTWLAN_BBPREAD:
	rt2x00_bbp_regread(rt2x00pci, address, &u8_value);
	*value = u8_value;
	break;
    case IOC_RTWLAN_BBPWRITE:
	rt2x00_bbp_regwrite(rt2x00pci, address, *value);
	break;
    default:
	return -1;
    }

    return 0;
}


/*
 * Interrupt routines.
 * rt2x00_interrupt_txdone processes all transmitted packetss results.
 * rt2x00_interrupt_rxdone processes all received rx packets.
 */
static void rt2x00_interrupt_txdone(struct _data_ring * ring) {

    struct rtwlan_device * rtwlan_dev     = rtnetdev_priv(ring->core->rtnet_dev);
    struct _txd		 *txd         = NULL;
    u8			tx_result     = 0x00;
    /*    u8			retry_count = 0x00; */


    do{
	txd = DESC_ADDR_DONE(ring);

	if(rt2x00_get_field32(txd->word0, TXD_W0_OWNER_NIC)
	   || !rt2x00_get_field32(txd->word0, TXD_W0_VALID))
	    break;

	if(ring->ring_type == RING_TX){
	    tx_result = rt2x00_get_field32(txd->word0, TXD_W0_RESULT);
	    /*	    retry_count = rt2x00_get_field32(txd->word0, TXD_W0_RETRY_COUNT); */

	    switch(tx_result) {
	    case TX_SUCCESS:
		rtwlan_dev->stats.tx_packets++;
		break;
	    case TX_SUCCESS_RETRY:
		rtwlan_dev->stats.tx_retry++;
		break;
	    case TX_FAIL_RETRY:
		DEBUG("TX_FAIL_RETRY.\n");
		break;
	    case TX_FAIL_INVALID:
		DEBUG("TX_FAIL_INVALID.\n");
		break;
	    case TX_FAIL_OTHER:
		DEBUG("TX_FAIL_OTHER.\n");
		break;
	    default:
		DEBUG("Unknown tx result.\n");
	    }
	}

	rt2x00_set_field32(&txd->word0, TXD_W0_VALID, 0);

	rt2x00_ring_index_done_inc(ring);
    }while(!rt2x00_ring_empty(ring));

}


static void rt2x00_interrupt_rxdone(struct _data_ring * ring, nanosecs_abs_t *time_stamp) {

    struct _rt2x00_pci	 * rt2x00pci  = rt2x00_priv(ring->core);
    struct rtnet_device  * rtnet_dev  = ring->core->rtnet_dev;
    struct rtwlan_device * rtwlan_dev     = rtnetdev_priv(rtnet_dev);
    struct _rxd		 * rxd = NULL;
    struct rtskb         * rtskb;
    void		 * data = NULL;
    u16			   size = 0x0000;
    /*    u16                    rssi = 0x0000; */

    while(1){

	rxd = DESC_ADDR(ring);
	data = DATA_ADDR(ring);

	if(rt2x00_get_field32(rxd->word0, RXD_W0_OWNER_NIC))
	    break;

	size = rt2x00_get_field32(rxd->word0, RXD_W0_DATABYTE_COUNT);
	/*	rssi = rt2x00_get_field32(rxd->word2, RXD_W2_RSSI); */

	/* prepare rtskb */
	rtskb = rtnetdev_alloc_rtskb(rtnet_dev, size + NET_IP_ALIGN);
	if(!rtskb){
	    ERROR("Couldn't allocate rtskb, packet dropped.\n");
	    break;
	}
	rtskb->time_stamp = *time_stamp;
	rtskb_reserve(rtskb, NET_IP_ALIGN);

	memcpy(rtskb->data, data, size);
	rtskb_put(rtskb, size);

	/* give incoming frame to rtwlan stack */
	rtwlan_rx(rtskb, rtnet_dev);

	rtwlan_dev->stats.rx_packets++;

	rt2x00_set_field32(&rxd->word0, RXD_W0_OWNER_NIC, 1);
	rt2x00_ring_index_inc(&rt2x00pci->rx);
    }
}

int rt2x00_interrupt(rtdm_irq_t *irq_handle) {

    nanosecs_abs_t time_stamp = rtdm_clock_read();

    struct rtnet_device   * rtnet_dev = rtdm_irq_get_arg(irq_handle, struct rtnet_device);
    struct rtwlan_device  * rtwlan_dev    = rtnetdev_priv(rtnet_dev);
    struct _rt2x00_core   * core      = rtwlan_priv(rtwlan_dev);
    struct _rt2x00_pci	  * rt2x00pci = rt2x00_priv(core);
    unsigned int old_packet_cnt       = rtwlan_dev->stats.rx_packets;
    u32			reg           = 0x00000000;

    rtdm_lock_get(&rt2x00pci->lock);

    rt2x00_register_read(rt2x00pci, CSR7, &reg);
    rt2x00_register_write(rt2x00pci, CSR7, reg);

    if(!reg) {
	rtdm_lock_put(&rt2x00pci->lock);
	return RTDM_IRQ_NONE;
    }

    if(rt2x00_get_field32(reg, CSR7_TBCN_EXPIRE))		/* Beacon timer expired interrupt. */
	DEBUG("Beacon timer expired.\n");
    if(rt2x00_get_field32(reg, CSR7_RXDONE))		/* Rx ring done interrupt. */
	rt2x00_interrupt_rxdone(&rt2x00pci->rx, &time_stamp);
    if(rt2x00_get_field32(reg, CSR7_TXDONE_ATIMRING))	/* Atim ring transmit done interrupt. */
	DEBUG("AtimTxDone.\n");
    if(rt2x00_get_field32(reg, CSR7_TXDONE_PRIORING))	/* Priority ring transmit done interrupt. */
	DEBUG("PrioTxDone.\n");
    if(rt2x00_get_field32(reg, CSR7_TXDONE_TXRING))	/* Tx ring transmit done interrupt. */
	rt2x00_interrupt_txdone(&rt2x00pci->tx);

    rtdm_lock_put(&rt2x00pci->lock);

    if (old_packet_cnt != rtwlan_dev->stats.rx_packets)
	rt_mark_stack_mgr(rtnet_dev);

    return RTDM_IRQ_HANDLED;
}

void rt2x00_init_eeprom(struct _rt2x00_pci * rt2x00pci, struct _rt2x00_config * config) {

    u32		reg = 0x00000000;
    u16		eeprom = 0x0000;

    /*
     * 1 - Detect EEPROM width.
     */
    rt2x00_register_read(rt2x00pci, CSR21, &reg);
    rt2x00pci->eeprom_width = rt2x00_get_field32(reg, CSR21_TYPE_93C46) ? EEPROM_WIDTH_93c46 : EEPROM_WIDTH_93c66;

    /*
     * 2 - Identify rf chipset.
     */
    eeprom = rt2x00_eeprom_read_word(rt2x00pci, EEPROM_ANTENNA);
    set_chip(&rt2x00pci->chip, RT2560, rt2x00_get_field16(eeprom, EEPROM_ANTENNA_RF_TYPE));

    /*
     * 3 - Identify default antenna configuration.
     */
    config->antenna_tx = rt2x00_get_field16(eeprom, EEPROM_ANTENNA_TX_DEFAULT);
    config->antenna_rx = rt2x00_get_field16(eeprom, EEPROM_ANTENNA_RX_DEFAULT);

    DEBUG("antenna_tx=%d antenna_rx=%d\n", config->antenna_tx, config->antenna_rx);

    /*
     * 4 - Read BBP data from EEPROM and store in private structure.
     */
    memset(&rt2x00pci->eeprom, 0x00, sizeof(rt2x00pci->eeprom));
    for(eeprom = 0; eeprom < EEPROM_BBP_SIZE; eeprom++)
	rt2x00pci->eeprom[eeprom] = rt2x00_eeprom_read_word(rt2x00pci, EEPROM_BBP_START + eeprom);

}

void rt2x00_dev_read_mac(struct _rt2x00_pci * rt2x00pci, struct rtnet_device * rtnet_dev) {

    u32			reg[2];

    memset(&reg, 0x00, sizeof(reg));

    rt2x00_register_multiread(rt2x00pci, CSR3, &reg[0], sizeof(reg));

    rtnet_dev->dev_addr[0] = rt2x00_get_field32(reg[0], CSR3_BYTE0);
    rtnet_dev->dev_addr[1] = rt2x00_get_field32(reg[0], CSR3_BYTE1);
    rtnet_dev->dev_addr[2] = rt2x00_get_field32(reg[0], CSR3_BYTE2);
    rtnet_dev->dev_addr[3] = rt2x00_get_field32(reg[0], CSR3_BYTE3);
    rtnet_dev->dev_addr[4] = rt2x00_get_field32(reg[1], CSR4_BYTE4);
    rtnet_dev->dev_addr[5] = rt2x00_get_field32(reg[1], CSR4_BYTE5);

    rtnet_dev->addr_len = 6;
}

int rt2x00_dev_probe(struct _rt2x00_core * core, void * priv) {

    struct pci_dev	*pci_dev = (struct pci_dev*)priv;
    struct _rt2x00_pci	*rt2x00pci = core->priv;

    memset(rt2x00pci, 0x00, sizeof(*rt2x00pci));

    if(unlikely(!pci_dev)){
	ERROR("invalid priv pointer.\n");
	return -ENODEV;
    }
    rt2x00pci->pci_dev = pci_dev;

    rt2x00pci->rx.data_addr = NULL;
    rt2x00pci->tx.data_addr = NULL;

    rt2x00pci->csr_addr = ioremap(pci_resource_start(pci_dev, 0), pci_resource_len(pci_dev, 0));
    if(!rt2x00pci->csr_addr){
	ERROR("ioremap failed.\n");
	return -ENOMEM;
    }

    rt2x00_init_eeprom(rt2x00pci, &core->config);
    rt2x00_dev_read_mac(rt2x00pci, core->rtnet_dev);

    return 0;
}

int rt2x00_dev_remove(struct _rt2x00_core * core) {

    struct _rt2x00_pci	*rt2x00pci = rt2x00_priv(core);

    if(rt2x00pci->csr_addr){
	iounmap(rt2x00pci->csr_addr);
	rt2x00pci->csr_addr = NULL;
    }

    return 0;
}

/*
 * rt2x00_clear_ring
 * During the initialization some of the descriptor variables are filled in.
 * The default value of the owner variable is different between the types of the descriptor,
 * DMA ring entries that receive packets are owned by the device untill a packet is received.
 * DMA ring entries that are used to transmit a packet are owned by the module untill the device,
 * for these rings the valid bit is set to 0 to indicate it is ready for use.
 * should transmit the packet that particular DMA ring entry.
 * The BUFFER_ADDRESS variable is used to link a descriptor to a packet data block.
 */
static void
rt2x00_clear_ring(struct _rt2x00_pci *rt2x00pci, struct _data_ring *ring) {

    struct _rxd		*rxd = NULL;
    struct _txd		*txd = NULL;
    dma_addr_t		data_dma = ring->data_dma + (ring->max_entries * ring->desc_size);
    u8			counter = 0x00;

    memset(ring->data_addr, 0x00, ring->mem_size);

    for(; counter < ring->max_entries; counter++){
	if(ring->ring_type == RING_RX){
	    rxd = (struct _rxd*)__DESC_ADDR(ring, counter);

	    rt2x00_set_field32(&rxd->word1, RXD_W1_BUFFER_ADDRESS, data_dma);
	    rt2x00_set_field32(&rxd->word0, RXD_W0_OWNER_NIC, 1);
	}else{
	    txd = (struct _txd*)__DESC_ADDR(ring, counter);

	    rt2x00_set_field32(&txd->word1, TXD_W1_BUFFER_ADDRESS, data_dma);
	    rt2x00_set_field32(&txd->word0, TXD_W0_VALID, 0);
	    rt2x00_set_field32(&txd->word0, TXD_W0_OWNER_NIC, 0);
	}

	data_dma += ring->entry_size;
    }

    rt2x00_ring_clear_index(ring);
}

/*
 * rt2x00_init_ring_register
 * The registers should be updated with the descriptor size and the
 * number of entries of each ring.
 * The address of the first entry of the descriptor ring is written to the register
 * corresponding to the ring.
 */
static void rt2x00_init_ring_register(struct _rt2x00_pci *rt2x00pci) {

    u32			reg = 0x00000000;

    /* Initialize ring register for RX/TX */

    rt2x00_set_field32(&reg, TXCSR2_TXD_SIZE, rt2x00pci->tx.desc_size);
    rt2x00_set_field32(&reg, TXCSR2_NUM_TXD, rt2x00pci->tx.max_entries);
    rt2x00_register_write(rt2x00pci, TXCSR2, reg);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, TXCSR3_TX_RING_REGISTER, rt2x00pci->tx.data_dma);
    rt2x00_register_write(rt2x00pci, TXCSR3, reg);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, RXCSR1_RXD_SIZE, rt2x00pci->rx.desc_size);
    rt2x00_set_field32(&reg, RXCSR1_NUM_RXD, rt2x00pci->rx.max_entries);
    rt2x00_register_write(rt2x00pci, RXCSR1, reg);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, RXCSR2_RX_RING_REGISTER, rt2x00pci->rx.data_dma);
    rt2x00_register_write(rt2x00pci, RXCSR2, reg);
}

static int
rt2x00_init_registers(struct _rt2x00_pci *rt2x00pci) {

    u32		reg = 0x00000000;

    DEBUG("Start.\n");

    rt2x00_register_write(rt2x00pci, PWRCSR0, cpu_to_le32(0x3f3b3100));

    rt2x00_register_write(rt2x00pci, PSCSR0, cpu_to_le32(0x00020002));
    rt2x00_register_write(rt2x00pci, PSCSR1, cpu_to_le32(0x00000002));
    rt2x00_register_write(rt2x00pci, PSCSR2, cpu_to_le32(0x00020002));
    rt2x00_register_write(rt2x00pci, PSCSR3, cpu_to_le32(0x00000002));

    rt2x00_register_read(rt2x00pci, TIMECSR, &reg);
    rt2x00_set_field32(&reg, TIMECSR_US_COUNT, 33);
    rt2x00_set_field32(&reg, TIMECSR_US_64_COUNT, 63);
    rt2x00_set_field32(&reg, TIMECSR_BEACON_EXPECT, 0);
    rt2x00_register_write(rt2x00pci, TIMECSR, reg);

    rt2x00_register_read(rt2x00pci, CSR9, &reg);
    rt2x00_set_field32(&reg, CSR9_MAX_FRAME_UNIT, (rt2x00pci->rx.entry_size / 128));
    rt2x00_register_write(rt2x00pci, CSR9, reg);

    rt2x00_register_write(rt2x00pci, CNT3, cpu_to_le32(0x3f080000));

    rt2x00_register_read(rt2x00pci, RXCSR0, &reg);
    rt2x00_set_field32(&reg, RXCSR0_DISABLE_RX, 0);
    rt2x00_set_field32(&reg, RXCSR0_DROP_CONTROL, 0);
    rt2x00_register_write(rt2x00pci, RXCSR0, reg);

    rt2x00_register_write(rt2x00pci, MACCSR0, cpu_to_le32(0x00213223));

    rt2x00_register_read(rt2x00pci, MACCSR1, &reg);
    rt2x00_set_field32(&reg, MACCSR1_AUTO_TXBBP, 1);
    rt2x00_set_field32(&reg, MACCSR1_AUTO_RXBBP, 1);
    rt2x00_register_write(rt2x00pci, MACCSR1, reg);

    rt2x00_register_read(rt2x00pci, MACCSR2, &reg);
    rt2x00_set_field32(&reg, MACCSR2_DELAY, 64);
    rt2x00_register_write(rt2x00pci, MACCSR2, reg);

    rt2x00_register_read(rt2x00pci, RXCSR3, &reg);
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID0, 47);		/* Signal. */
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID0_VALID, 1);
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID1, 51);		/* Rssi. */
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID1_VALID, 1);
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID2, 42);		/* OFDM Rate. */
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID2_VALID, 1);
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID3, 51);		/* OFDM. */
    rt2x00_set_field32(&reg, RXCSR3_BBP_ID3_VALID, 1);
    rt2x00_register_write(rt2x00pci, RXCSR3, reg);

    rt2x00_register_read(rt2x00pci, RALINKCSR, &reg);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_DATA0, 17);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_ID0, 26);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_VALID0, 1);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_DATA1, 0);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_ID1, 26);
    rt2x00_set_field32(&reg, RALINKCSR_AR_BBP_VALID1, 1);
    rt2x00_register_write(rt2x00pci, RALINKCSR, reg);

    rt2x00_register_write(rt2x00pci, BBPCSR1, cpu_to_le32(0x82188200));

    rt2x00_register_write(rt2x00pci, TXACKCSR0, cpu_to_le32(0x00000020));

    rt2x00_register_write(rt2x00pci, ARTCSR0, cpu_to_le32(0x7038140a));
    rt2x00_register_write(rt2x00pci, ARTCSR1, cpu_to_le32(0x1d21252d));
    rt2x00_register_write(rt2x00pci, ARTCSR2, cpu_to_le32(0x1919191d));

    /* disable Beacon timer */
    rt2x00_register_write(rt2x00pci, CSR14, 0x0);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, LEDCSR_ON_PERIOD, 30);
    rt2x00_set_field32(&reg, LEDCSR_OFF_PERIOD, 70);
    rt2x00_set_field32(&reg, LEDCSR_LINK, 0);
    rt2x00_set_field32(&reg, LEDCSR_ACTIVITY, 0);
    rt2x00_register_write(rt2x00pci, LEDCSR, reg);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, CSR1_SOFT_RESET, 1);
    rt2x00_register_write(rt2x00pci, CSR1, reg);

    reg = 0x00000000;
    rt2x00_set_field32(&reg, CSR1_HOST_READY, 1);
    rt2x00_register_write(rt2x00pci, CSR1, reg);

    /*
     * We must clear the FCS and FIFI error count.
     * These registers are cleared on read, so we may pass a useless variable to store the value.
     */
    rt2x00_register_read(rt2x00pci, CNT0, &reg);
    rt2x00_register_read(rt2x00pci, CNT4, &reg);

    return 0;
}

static void
rt2x00_init_write_mac(struct _rt2x00_pci *rt2x00pci, struct rtnet_device *rtnet_dev) {

    u32			reg[2];

    memset(&reg, 0x00, sizeof(reg));

    rt2x00_set_field32(&reg[0], CSR3_BYTE0, rtnet_dev->dev_addr[0]);
    rt2x00_set_field32(&reg[0], CSR3_BYTE1, rtnet_dev->dev_addr[1]);
    rt2x00_set_field32(&reg[0], CSR3_BYTE2, rtnet_dev->dev_addr[2]);
    rt2x00_set_field32(&reg[0], CSR3_BYTE3, rtnet_dev->dev_addr[3]);
    rt2x00_set_field32(&reg[1], CSR4_BYTE4, rtnet_dev->dev_addr[4]);
    rt2x00_set_field32(&reg[1], CSR4_BYTE5, rtnet_dev->dev_addr[5]);

    rt2x00_register_multiwrite(rt2x00pci, CSR3, &reg[0], sizeof(reg));
}


static int
rt2x00_init_bbp(struct _rt2x00_pci *rt2x00pci) {

    u8		reg_id = 0x00;
    u8		value = 0x00;
    u8		counter = 0x00;

    for(counter = 0x00; counter < REGISTER_BUSY_COUNT; counter++){
	rt2x00_bbp_regread(rt2x00pci, 0x00, &value);
	if((value != 0xff) && (value != 0x00))
	    goto continue_csr_init;
	NOTICE("Waiting for BBP register.\n");
    }

    ERROR("hardware problem, BBP register access failed, aborting.\n");
    return -EACCES;

  continue_csr_init:
    rt2x00_bbp_regwrite(rt2x00pci, 3, 0x02);
    rt2x00_bbp_regwrite(rt2x00pci, 4, 0x19);
    rt2x00_bbp_regwrite(rt2x00pci, 14, 0x1c);
    rt2x00_bbp_regwrite(rt2x00pci, 15, 0x30);
    rt2x00_bbp_regwrite(rt2x00pci, 16, 0xac);
    rt2x00_bbp_regwrite(rt2x00pci, 17, 0x48);
    rt2x00_bbp_regwrite(rt2x00pci, 18, 0x18);
    rt2x00_bbp_regwrite(rt2x00pci, 19, 0xff);
    rt2x00_bbp_regwrite(rt2x00pci, 20, 0x1e);
    rt2x00_bbp_regwrite(rt2x00pci, 21, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 22, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 23, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 24, 0x70);
    rt2x00_bbp_regwrite(rt2x00pci, 25, 0x40);
    rt2x00_bbp_regwrite(rt2x00pci, 26, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 27, 0x23);
    rt2x00_bbp_regwrite(rt2x00pci, 30, 0x10);
    rt2x00_bbp_regwrite(rt2x00pci, 31, 0x2b);
    rt2x00_bbp_regwrite(rt2x00pci, 32, 0xb9);
    rt2x00_bbp_regwrite(rt2x00pci, 34, 0x12);
    rt2x00_bbp_regwrite(rt2x00pci, 35, 0x50);
    rt2x00_bbp_regwrite(rt2x00pci, 39, 0xc4);
    rt2x00_bbp_regwrite(rt2x00pci, 40, 0x02);
    rt2x00_bbp_regwrite(rt2x00pci, 41, 0x60);
    rt2x00_bbp_regwrite(rt2x00pci, 53, 0x10);
    rt2x00_bbp_regwrite(rt2x00pci, 54, 0x18);
    rt2x00_bbp_regwrite(rt2x00pci, 56, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 57, 0x10);
    rt2x00_bbp_regwrite(rt2x00pci, 58, 0x08);
    rt2x00_bbp_regwrite(rt2x00pci, 61, 0x6d);
    rt2x00_bbp_regwrite(rt2x00pci, 62, 0x10);

    DEBUG("Start reading EEPROM contents...\n");
    for(counter = 0; counter < EEPROM_BBP_SIZE; counter++){
	if(rt2x00pci->eeprom[counter] != 0xffff && rt2x00pci->eeprom[counter] != 0x0000){
	    reg_id = rt2x00_get_field16(rt2x00pci->eeprom[counter], EEPROM_BBP_REG_ID);
	    value = rt2x00_get_field16(rt2x00pci->eeprom[counter], EEPROM_BBP_VALUE);
	    DEBUG("BBP reg_id: 0x%02x, value: 0x%02x.\n", reg_id, value);
	    rt2x00_bbp_regwrite(rt2x00pci, reg_id, value);
	}
    }
    DEBUG("...End of EEPROM contents.\n");

    return 0;
}

/*
 * Device radio routines.
 * When the radio is switched on or off, the TX and RX
 * should always be reset using the TXCSR0 and RXCSR0 registers.
 * The radio itself is switched on and off using the PWRCSR0 register.
 */

static int rt2x00_dev_radio_on(struct _rt2x00_core * core) {

    struct _rt2x00_pci	*rt2x00pci = rt2x00_priv(core);
    u32			reg = 0x00000000;
    int retval;

    if(rt2x00_pci_alloc_rings(core))
	goto exit_fail;

    rt2x00_clear_ring(rt2x00pci, &rt2x00pci->rx);
    rt2x00_clear_ring(rt2x00pci, &rt2x00pci->tx);

    rt2x00_init_ring_register(rt2x00pci);

    if(rt2x00_init_registers(rt2x00pci))
	goto exit_fail;

    rt2x00_init_write_mac(rt2x00pci, core->rtnet_dev);

    if(rt2x00_init_bbp(rt2x00pci))
	goto exit_fail;

    /*
     * Clear interrupts.
     */
    rt2x00_register_read(rt2x00pci, CSR7, &reg);
    rt2x00_register_write(rt2x00pci, CSR7, reg);

    /* Register rtdm-irq */
    retval = rtdm_irq_request(&rt2x00pci->irq_handle,
			      core->rtnet_dev->irq,
			      rt2x00_interrupt, 0,
			      core->rtnet_dev->name,
			      core->rtnet_dev);

    /*
     * Enable interrupts.
     */
    rt2x00_register_read(rt2x00pci, CSR8, &reg);
    rt2x00_set_field32(&reg, CSR8_TBCN_EXPIRE, 0);
    rt2x00_set_field32(&reg, CSR8_TXDONE_TXRING, 0);
    rt2x00_set_field32(&reg, CSR8_TXDONE_ATIMRING, 0);
    rt2x00_set_field32(&reg, CSR8_TXDONE_PRIORING, 0);
    rt2x00_set_field32(&reg, CSR8_RXDONE, 0);
    rt2x00_register_write(rt2x00pci, CSR8, reg);

    return 0;

  exit_fail:
    rt2x00_pci_free_rings(core);

    return -ENOMEM;
}

static int rt2x00_dev_radio_off(struct _rt2x00_core * core) {

    struct _rt2x00_pci	*rt2x00pci = rt2x00_priv(core);
    u32			reg = 0x00000000;
    int retval=0;

    rt2x00_register_write(rt2x00pci, PWRCSR0, cpu_to_le32(0x00000000));

    rt2x00_register_read(rt2x00pci, TXCSR0, &reg);
    rt2x00_set_field32(&reg, TXCSR0_ABORT, 1);
    rt2x00_register_write(rt2x00pci, TXCSR0, reg);

    rt2x00_register_read(rt2x00pci, RXCSR0, &reg);
    rt2x00_set_field32(&reg, RXCSR0_DISABLE_RX, 1);
    rt2x00_register_write(rt2x00pci, RXCSR0, reg);

    rt2x00_register_read(rt2x00pci, LEDCSR, &reg);
    rt2x00_set_field32(&reg, LEDCSR_LINK, 0);
    rt2x00_register_write(rt2x00pci, LEDCSR, reg);

    rt2x00_register_read(rt2x00pci, CSR8, &reg);
    rt2x00_set_field32(&reg, CSR8_TBCN_EXPIRE, 1);
    rt2x00_set_field32(&reg, CSR8_TXDONE_TXRING, 1);
    rt2x00_set_field32(&reg, CSR8_TXDONE_ATIMRING, 1);
    rt2x00_set_field32(&reg, CSR8_TXDONE_PRIORING, 1);
    rt2x00_set_field32(&reg, CSR8_RXDONE, 1);
    rt2x00_register_write(rt2x00pci, CSR8, reg);

    rt2x00_pci_free_rings(core);

    if((retval=rtdm_irq_free(&rt2x00pci->irq_handle)) != 0)
	ERROR("rtdm_irq_free=%d\n", retval);

    rt_stack_disconnect(core->rtnet_dev);

    return retval;
}


/*
 * Configuration handlers.
 */

static void
rt2x00_dev_update_autoresp(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32 reg = 0;

    DEBUG("Start.\n");

    rt2x00_register_read(rt2x00pci, TXCSR1, &reg);

    if(config->config_flags & CONFIG_AUTORESP)
	rt2x00_set_field32(&reg, TXCSR1_AUTORESPONDER , 1);
    else
	rt2x00_set_field32(&reg, TXCSR1_AUTORESPONDER , 0);

    rt2x00_register_write(rt2x00pci, TXCSR1, reg);
}

static void
rt2x00_dev_update_bbpsens(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    rt2x00_bbp_regwrite(rt2x00pci, 0x11, config->bbpsens);
}

static void
rt2x00_dev_update_bssid(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg[2];

    memset(&reg, 0x00, sizeof(reg));

    rt2x00_set_field32(&reg[0], CSR5_BYTE0, config->bssid[0]);
    rt2x00_set_field32(&reg[0], CSR5_BYTE1, config->bssid[1]);
    rt2x00_set_field32(&reg[0], CSR5_BYTE2, config->bssid[2]);
    rt2x00_set_field32(&reg[0], CSR5_BYTE3, config->bssid[3]);
    rt2x00_set_field32(&reg[1], CSR6_BYTE4, config->bssid[4]);
    rt2x00_set_field32(&reg[1], CSR6_BYTE5, config->bssid[5]);

    rt2x00_register_multiwrite(rt2x00pci, CSR5, &reg[0], sizeof(reg));
}

static void
rt2x00_dev_update_packet_filter(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg = 0x00000000;

    DEBUG("Start.\n");

    rt2x00_register_read(rt2x00pci, RXCSR0, &reg);

    rt2x00_set_field32(&reg, RXCSR0_DROP_TODS, 0);
    rt2x00_set_field32(&reg, RXCSR0_DROP_NOT_TO_ME, 1);
    rt2x00_set_field32(&reg, RXCSR0_DROP_CRC, 1);
    rt2x00_set_field32(&reg, RXCSR0_DROP_PHYSICAL, 1);
    rt2x00_set_field32(&reg, RXCSR0_DROP_CONTROL, 1);
    rt2x00_set_field32(&reg, RXCSR0_DROP_VERSION_ERROR, 1);
    rt2x00_set_field32(&reg, RXCSR0_DROP_NOT_TO_ME, 1);

    /*
     * This looks like a bug, but for an unknown reason the register seems to swap the bits !!!
     */
    if(config->config_flags & CONFIG_DROP_BCAST)
	rt2x00_set_field32(&reg, RXCSR0_DROP_MCAST, 1);
    else
	rt2x00_set_field32(&reg, RXCSR0_DROP_MCAST, 0);

    if(config->config_flags & CONFIG_DROP_MCAST)
	rt2x00_set_field32(&reg, RXCSR0_DROP_BCAST, 1);
    else
	rt2x00_set_field32(&reg, RXCSR0_DROP_BCAST, 0);

    rt2x00_register_write(rt2x00pci, RXCSR0, reg);

}

static void
rt2x00_dev_update_channel(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u8			txpower = rt2x00_get_txpower(&rt2x00pci->chip, config->txpower);
    u32			reg = 0x00000000;

    if(rt2x00_get_rf_value(&rt2x00pci->chip, config->channel, &rt2x00pci->channel)){
	ERROR("RF values for chip %04x and channel %d not found.\n", rt2x00_get_rf(&rt2x00pci->chip), config->channel);
	return;
    }

    /*
     * Set TXpower.
     */
    rt2x00_set_field32(&rt2x00pci->channel.rf3, RF3_TXPOWER, txpower);

    /*
     * For RT2525 we should first set the channel to half band higher.
     */
    if(rt2x00_rf(&rt2x00pci->chip, RF2525)){
	rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf1);
	rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf2 + cpu_to_le32(0x00000020));
	rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf3);
	if(rt2x00pci->channel.rf4)
	    rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf4);
    }

    rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf1);
    rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf2);
    rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf3);
    if(rt2x00pci->channel.rf4)
	rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf4);

    /*
     * Channel 14 requires the Japan filter bit to be set.
     */
    rt2x00_bbp_regwrite(rt2x00pci, 70, (config->channel == 14) ? 0x4e : 0x46);

    msleep(1);

    /*
     * Clear false CRC during channel switch.
     */
    rt2x00_register_read(rt2x00pci, CNT0, &reg);

    DEBUG("Switching to channel %d. RF1: 0x%08x, RF2: 0x%08x, RF3: 0x%08x, RF4: 0x%08x.\n",
	 config->channel, rt2x00pci->channel.rf1, rt2x00pci->channel.rf2,
	 rt2x00pci->channel.rf3, rt2x00pci->channel.rf4);
}

static void
rt2x00_dev_update_rate(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			value = 0x00000000;
    u32			reg = 0x00000000;
    u8			counter = 0x00;

    DEBUG("Start.\n");

    rt2x00_register_read(rt2x00pci, TXCSR1, &reg);

    value = config->sifs + (2 * config->slot_time) + config->plcp
	+ get_preamble(config)
	+ get_duration(ACK_SIZE, capabilities.bitrate[0]);
    rt2x00_set_field32(&reg, TXCSR1_ACK_TIMEOUT, value);

    value = config->sifs + config->plcp
	+ get_preamble(config)
	+ get_duration(ACK_SIZE, capabilities.bitrate[0]);
    rt2x00_set_field32(&reg, TXCSR1_ACK_CONSUME_TIME, value);

    rt2x00_set_field32(&reg, TXCSR1_TSF_OFFSET, 0x18);
    rt2x00_set_field32(&reg, TXCSR1_AUTORESPONDER, 1);

    rt2x00_register_write(rt2x00pci, TXCSR1, reg);

    reg = 0x00000000;
    for(counter = 0; counter < 12; counter++){
	reg |= cpu_to_le32(0x00000001 << counter);
	if(capabilities.bitrate[counter] == config->bitrate)
	    break;
    }

    rt2x00_register_write(rt2x00pci, ARCSR1, reg);
}

static void
rt2x00_dev_update_txpower(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u8			txpower = rt2x00_get_txpower(&rt2x00pci->chip, config->txpower);

    DEBUG("Start.\n");

    rt2x00_set_field32(&rt2x00pci->channel.rf3, RF3_TXPOWER, txpower);
    rt2x00_rf_regwrite(rt2x00pci, rt2x00pci->channel.rf3);
}

static void
rt2x00_dev_update_antenna(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg;
    u8			reg_rx;
    u8			reg_tx;

    rt2x00_register_read(rt2x00pci, BBPCSR1, &reg);
    rt2x00_bbp_regread(rt2x00pci, 14, &reg_rx);
    rt2x00_bbp_regread(rt2x00pci, 2, &reg_tx);

    /* TX antenna select */
    if(config->antenna_tx == 1) {
	/* Antenna A */
	reg_tx = (reg_tx & 0xfc) | 0x00;
	reg    = (reg    & 0xfffcfffc) | 0x00;
    }
    else if(config->antenna_tx == 2) {
	/* Antenna B */
	reg_tx = (reg_tx & 0xfc) | 0x02;
	reg    = (reg    & 0xfffcfffc) | 0x00020002;
    }
    else {
	/* Diversity */
	reg_tx = (reg_tx & 0xfc) | 0x02;
	reg    = (reg    & 0xfffcfffc) | 0x00020002;
    }

    /* RX antenna select */
    if(config->antenna_rx == 1)
	reg_rx = (reg_rx & 0xfc) | 0x00;
    else if(config->antenna_rx == 2)
	reg_rx = (reg_rx & 0xfc) | 0x02;
    else
	reg_rx = (reg_rx & 0xfc) | 0x02;

    /*
     * RT2525E and RT5222 need to flip I/Q
     */
    if(rt2x00_rf(&rt2x00pci->chip, RF5222)){

	reg |=  0x00040004;
	reg_tx |= 0x04;
    }
    else if(rt2x00_rf(&rt2x00pci->chip, RF2525E)) {

	reg |=  0x00040004;
	reg_tx |= 0x04;
	reg_rx |= 0xfb;
    }

    rt2x00_register_write(rt2x00pci, BBPCSR1, reg);
    rt2x00_bbp_regwrite(rt2x00pci, 14, reg_rx);
    rt2x00_bbp_regwrite(rt2x00pci, 2, reg_tx);

}

static void
rt2x00_dev_update_duration(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg = 0x00000000;

    DEBUG("Start.\n");

    rt2x00_register_read(rt2x00pci, CSR11, &reg);
    rt2x00_set_field32(&reg, CSR11_CWMIN, 5);		/* 2^5 = 32. */
    rt2x00_set_field32(&reg, CSR11_CWMAX, 10);		/* 2^10 = 1024. */
    rt2x00_set_field32(&reg, CSR11_SLOT_TIME, config->slot_time);
    rt2x00_set_field32(&reg, CSR11_CW_SELECT, 1);
    rt2x00_register_write(rt2x00pci, CSR11, reg);

    rt2x00_register_read(rt2x00pci, CSR18, &reg);
    rt2x00_set_field32(&reg, CSR18_SIFS, config->sifs);
    rt2x00_set_field32(&reg, CSR18_PIFS, config->sifs + config->slot_time);
    rt2x00_register_write(rt2x00pci, CSR18, reg);

    rt2x00_register_read(rt2x00pci, CSR19, &reg);
    rt2x00_set_field32(&reg, CSR19_DIFS, config->sifs + (2 * config->slot_time));
    rt2x00_set_field32(&reg, CSR19_EIFS, config->sifs + get_duration((IEEE80211_HEADER + ACK_SIZE), capabilities.bitrate[0]));
    rt2x00_register_write(rt2x00pci, CSR19, reg);
}

static void
rt2x00_dev_update_retry(struct _rt2x00_pci * rt2x00pci, struct _rt2x00_config * config) {

    u32			reg = 0x00000000;

    rt2x00_register_read(rt2x00pci, CSR11, &reg);
    rt2x00_set_field32(&reg, CSR11_LONG_RETRY, config->long_retry);
    rt2x00_set_field32(&reg, CSR11_SHORT_RETRY, config->short_retry);
    rt2x00_register_write(rt2x00pci, CSR11, reg);
}

static void
rt2x00_dev_update_preamble(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg[4];
    u32			preamble = 0x00000000;

    memset(&reg, 0x00, sizeof(reg));

    reg[0] = cpu_to_le32(0x00700400 | preamble);	/* ARCSR2 */
    reg[1] = cpu_to_le32(0x00380401 | preamble);	/* ARCSR3 */
    reg[2] = cpu_to_le32(0x00150402 | preamble);	/* ARCSR4 */
    reg[3] = cpu_to_le32(0x000b8403 | preamble);	/* ARCSR5 */

    rt2x00_register_multiwrite(rt2x00pci, ARCSR2, &reg[0], sizeof(reg));
}

static void
rt2x00_dev_update_led(struct _rt2x00_pci *rt2x00pci, struct _rt2x00_config *config) {

    u32			reg = 0x00000000;

    rt2x00_register_read(rt2x00pci, LEDCSR, &reg);
    rt2x00_set_field32(&reg, LEDCSR_LINK, config->led_status ? 1 : 0);
    rt2x00_register_write(rt2x00pci, LEDCSR, reg);
}

static int
rt2x00_dev_update_config(struct _rt2x00_core *core, u16 update_flags) {

    struct _rt2x00_pci	*rt2x00pci = rt2x00_priv(core);

    DEBUG("Start.\n");

    if(update_flags & UPDATE_BSSID)
	rt2x00_dev_update_bssid(rt2x00pci, &core->config);

    if(update_flags & UPDATE_PACKET_FILTER)
	rt2x00_dev_update_packet_filter(rt2x00pci, &core->config);

    if(update_flags & UPDATE_CHANNEL)
	rt2x00_dev_update_channel(rt2x00pci, &core->config);

    if(update_flags & UPDATE_BITRATE)
	rt2x00_dev_update_rate(rt2x00pci, &core->config);

    if(update_flags & UPDATE_TXPOWER)
	rt2x00_dev_update_txpower(rt2x00pci, &core->config);

    if(update_flags & UPDATE_ANTENNA)
	rt2x00_dev_update_antenna(rt2x00pci, &core->config);

    if(update_flags & UPDATE_DURATION)
	rt2x00_dev_update_duration(rt2x00pci, &core->config);

    if(update_flags & UPDATE_RETRY)
	rt2x00_dev_update_retry(rt2x00pci, &core->config);

    if(update_flags & UPDATE_PREAMBLE)
	rt2x00_dev_update_preamble(rt2x00pci, &core->config);

    if(update_flags & UPDATE_LED_STATUS)
	rt2x00_dev_update_led(rt2x00pci, &core->config);

    if(update_flags & UPDATE_AUTORESP)
	rt2x00_dev_update_autoresp(rt2x00pci, &core->config);

    if(update_flags & UPDATE_BBPSENS)
	rt2x00_dev_update_bbpsens(rt2x00pci, &core->config);

    DEBUG("Exit.\n");

    return 0;
}


/*
 * Transmission routines.
 * rt2x00_write_tx_desc will write the txd descriptor.
 * rt2x00_dev_xmit_packet will copy the packets to the appropriate DMA ring.
 */

/*
 * PLCP_SIGNAL, PLCP_SERVICE, PLCP_LENGTH_LOW and PLCP_LENGTH_HIGH are BBP registers.
 * For RT2460 devices we need, besides the value we want to write,
 * also set the busy bit (0x8000) and the register number (0x0f00).
 * The value we want to write is stored in 0x00ff.
 * For PLCP_SIGNAL we can optionally enable SHORT_PREAMBLE.
 * For PLCP_SERVICE we can set the length extension bit according to
 * 802.11b standard 18.2.3.5.
 */
static void rt2x00_write_tx_desc(struct _rt2x00_pci *rt2x00pci, struct _txd *txd, u32 packet_size, u16 rate, u16 xmit_flags) {

    u32		residual = 0x00000000;
    u32         duration = 0x00000000;
    u16		signal = 0x0000;
    u16		service = 0x0000;
    u16		length_low = 0x0000;
    u16		length_high = 0x0000;

    rt2x00_set_field32(&txd->word0, TXD_W0_VALID, 1);
    rt2x00_set_field32(&txd->word0, TXD_W0_DATABYTE_COUNT, packet_size);
    rt2x00_set_field32(&txd->word0, TXD_W0_ACK, (xmit_flags & XMIT_ACK) ? 1 : 0);
    rt2x00_set_field32(&txd->word0, TXD_W0_RETRY_MODE, (xmit_flags & XMIT_LONG_RETRY) ? 1 : 0);
    rt2x00_set_field32(&txd->word0, TXD_W0_TIMESTAMP, (xmit_flags & XMIT_TIMESTAMP) ? 1 : 0);
    rt2x00_set_field32(&txd->word0, TXD_W0_MORE_FRAG, (xmit_flags & XMIT_MORE_FRAGS) ? 1 : 0);
    rt2x00_set_field32(&txd->word0, TXD_W0_MORE_FRAG, (xmit_flags & XMIT_RTS) ? 1 : 0);
    rt2x00_set_field32(&txd->word10, TXD_W10_RTS, (xmit_flags & XMIT_RTS) ? 1 : 0);
    rt2x00_set_field32(&txd->word0, TXD_W0_OFDM, (xmit_flags & XMIT_OFDM) ? 1 : 0);

    packet_size += 4;

    if(xmit_flags & XMIT_OFDM) {

	/*
	 * convert length to microseconds.
	 */
	length_high = (packet_size >> 6) & 0x3f;
	length_low = (packet_size & 0x3f);
    } else {

	residual = get_duration_res(packet_size, rate);
	duration = get_duration(packet_size, rate);

	if(residual != 0)
	    duration++;

	length_high = duration >> 8;
	length_low = duration & 0xff;
    }

    signal |= 0x8500 | rt2x00_get_plcp(rate);
    if(xmit_flags & XMIT_SHORT_PREAMBLE)
	signal |= 0x0008;

    service |= 0x0600 | 0x0004;
    if(residual <= (8 % 11))
	service |= 0x0080;

    rt2x00_set_field32(&txd->word3, TXD_W3_PLCP_SIGNAL, signal);
    rt2x00_set_field32(&txd->word3, TXD_W3_PLCP_SERVICE, service);
    rt2x00_set_field32(&txd->word3, TXD_W3_PLCP_LENGTH_LOW, length_low);
    rt2x00_set_field32(&txd->word3, TXD_W3_PLCP_LENGTH_HIGH, length_high);

    /* set XMIT_IFS to XMIT_IFS_NONE */
    rt2x00_set_field32(&txd->word0, TXD_W0_IFS, XMIT_IFS_NONE);

    /* highest priority */
    rt2x00_set_field32(&txd->word2, TXD_W2_CWMIN, 1);
    rt2x00_set_field32(&txd->word2, TXD_W2_CWMAX, 2);
    rt2x00_set_field32(&txd->word2, TXD_W2_AIFS, 1);

    /*
     * set this last, after this the device can start transmitting the packet.
     */
    rt2x00_set_field32(&txd->word0, TXD_W0_OWNER_NIC, 1);
}

static int rt2x00_dev_xmit_packet(struct _rt2x00_core * core,
				  struct rtskb *rtskb,
				  u16 rate,
				  u16 xmit_flags) {

    struct _rt2x00_pci	*rt2x00pci = rt2x00_priv(core);
    struct _data_ring	*ring = NULL;
    struct _txd		*txd = NULL;
    void		*data = NULL;
    u32			reg = 0x00000000;
    rtdm_lockctx_t context;

    rtdm_lock_get_irqsave(&rt2x00pci->lock, context);

    /* load tx-control register */
    rt2x00_register_read(rt2x00pci, TXCSR0, &reg);

    /* select tx-descriptor ring and prepare xmit */
    ring = &rt2x00pci->tx;
    rt2x00_set_field32(&reg, TXCSR0_KICK_TX, 1);

    txd = DESC_ADDR(ring);
    data = DATA_ADDR(ring);

    if(rt2x00_get_field32(txd->word0, TXD_W0_OWNER_NIC)
       || rt2x00_get_field32(txd->word0, TXD_W0_VALID)) {
	rtdm_lock_put_irqrestore(&rt2x00pci->lock, context);
	return -ENOMEM;
    }

    /* get and patch time stamp just before the transmission */
    if (rtskb->xmit_stamp)
	*rtskb->xmit_stamp = cpu_to_be64(rtdm_clock_read() + *rtskb->xmit_stamp);

    /* copy rtskb to dma */
    memcpy(data, rtskb->data, rtskb->len);

    rt2x00_write_tx_desc(rt2x00pci, txd, rtskb->len, rate, xmit_flags);
    rt2x00_ring_index_inc(ring);

    /* let the device do the rest ... */
    rt2x00_register_write(rt2x00pci, TXCSR0, reg);

    rtdm_lock_put_irqrestore(&rt2x00pci->lock, context);

    return 0;
}


/*
 * PCI device handlers for usage by core module.
 */
static struct _rt2x00_dev_handler rt2x00_pci_handler = {

    .dev_module		 = THIS_MODULE,
    .dev_probe		 = rt2x00_dev_probe,
    .dev_remove		 = rt2x00_dev_remove,
    .dev_radio_on	 = rt2x00_dev_radio_on,
    .dev_radio_off	 = rt2x00_dev_radio_off,
    .dev_update_config   = rt2x00_dev_update_config,
    .dev_register_access = rt2x00_dev_register_access,
    .dev_xmit_packet     = rt2x00_dev_xmit_packet,
};

int rt2x00_pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *id) {

    struct rtnet_device	*rtnet_dev = NULL;
    int			status = 0x00000000;

    DEBUG("start.\n");

    if(id->driver_data != RT2560){
	ERROR("detected device not supported.\n");
	status = -ENODEV;
	goto exit;
    }

    if(pci_enable_device(pci_dev)){
	ERROR("enable device failed.\n");
	status = -EIO;
	goto exit;
    }

    pci_set_master(pci_dev);

    if(pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64))
       && pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32))){
	ERROR("PCI DMA not supported\n");
	status = -EIO;
	goto exit_disable_device;
    }

    if(pci_request_regions(pci_dev, pci_name(pci_dev))){
	ERROR("PCI request regions failed.\n");
	status = -EBUSY;
	goto exit_disable_device;
    }
    INFO("pci_dev->irq=%d\n", pci_dev->irq);

    rtnet_dev = rt2x00_core_probe(&rt2x00_pci_handler, pci_dev, sizeof(struct _rt2x00_pci));

    if(!rtnet_dev){
	ERROR("rtnet_device allocation failed.\n");
	status = -ENOMEM;
	goto exit_release_regions;
    }

    rtnet_dev->irq = pci_dev->irq;

    pci_set_drvdata(pci_dev, rtnet_dev);

    return 0;

  exit_release_regions:
    pci_release_regions(pci_dev);

  exit_disable_device:
    if(status != -EBUSY)
	pci_disable_device(pci_dev);

  exit:
    return status;
}

static void rt2x00_pci_remove(struct pci_dev *pci_dev) {

    struct rtnet_device	*rtnet_dev = pci_get_drvdata(pci_dev);

    rt2x00_core_remove(rtnet_dev);
    pci_set_drvdata(pci_dev, NULL);
    pci_release_regions(pci_dev);
    pci_disable_device(pci_dev);
}

/*
 * RT2500 PCI module information.
 */
char version[] = DRV_NAME " - " DRV_VERSION;

struct pci_device_id rt2x00_device_pci_tbl[] = {
    { PCI_DEVICE(0x1814, 0x0201), .driver_data = RT2560},	/* Ralink 802.11g */
    {0,}
};

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION("RTnet rt2500 PCI WLAN driver (PCI Module)");
MODULE_LICENSE("GPL");

struct pci_driver rt2x00_pci_driver =
{
    .name	= DRV_NAME,
    .id_table	= rt2x00_device_pci_tbl,
    .probe	= rt2x00_pci_probe,
    .remove	= rt2x00_pci_remove,
};


static int __init rt2x00_pci_init(void) {
    rtdm_printk(KERN_INFO "Loading module: %s\n", version);
    return pci_register_driver(&rt2x00_pci_driver);
}

static void __exit rt2x00_pci_exit(void) {
    rtdm_printk(KERN_INFO "Unloading module: %s\n", version);
    pci_unregister_driver(&rt2x00_pci_driver);
}

module_init(rt2x00_pci_init);
module_exit(rt2x00_pci_exit);
