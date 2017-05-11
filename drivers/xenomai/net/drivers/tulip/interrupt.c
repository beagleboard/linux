/*
	drivers/net/tulip/interrupt.c

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
#include <linux/etherdevice.h>
#include <linux/pci.h>


int tulip_rx_copybreak;
unsigned int tulip_max_interrupt_work;

#ifdef CONFIG_NET_HW_FLOWCONTROL

#define MIT_SIZE 15
unsigned int mit_table[MIT_SIZE+1] =
{
	/*  CRS11 21143 hardware Mitigation Control Interrupt
	    We use only RX mitigation we other techniques for
	    TX intr. mitigation.

	   31    Cycle Size (timer control)
	   30:27 TX timer in 16 * Cycle size
	   26:24 TX No pkts before Int.
	   23:20 RX timer in Cycle size
	   19:17 RX No pkts before Int.
	   16       Continues Mode (CM)
	*/

	0x0,             /* IM disabled */
	0x80150000,      /* RX time = 1, RX pkts = 2, CM = 1 */
	0x80150000,
	0x80270000,
	0x80370000,
	0x80490000,
	0x80590000,
	0x80690000,
	0x807B0000,
	0x808B0000,
	0x809D0000,
	0x80AD0000,
	0x80BD0000,
	0x80CF0000,
	0x80DF0000,
//       0x80FF0000      /* RX time = 16, RX pkts = 7, CM = 1 */
	0x80F10000      /* RX time = 16, RX pkts = 0, CM = 1 */
};
#endif


int tulip_refill_rx(/*RTnet*/struct rtnet_device *rtdev)
{
	struct tulip_private *tp = (struct tulip_private *)rtdev->priv;
	int entry;
	int refilled = 0;

	/* Refill the Rx ring buffers. */
	for (; tp->cur_rx - tp->dirty_rx > 0; tp->dirty_rx++) {
		entry = tp->dirty_rx % RX_RING_SIZE;
		if (tp->rx_buffers[entry].skb == NULL) {
			struct /*RTnet*/rtskb *skb;
			dma_addr_t mapping;

			skb = tp->rx_buffers[entry].skb = /*RTnet*/rtnetdev_alloc_rtskb(rtdev, PKT_BUF_SZ);
			if (skb == NULL)
				break;

			mapping = pci_map_single(tp->pdev, skb->tail, PKT_BUF_SZ,
						 PCI_DMA_FROMDEVICE);
			tp->rx_buffers[entry].mapping = mapping;

			tp->rx_ring[entry].buffer1 = cpu_to_le32(mapping);
			refilled++;
		}
		tp->rx_ring[entry].status = cpu_to_le32(DescOwned);
	}
	if(tp->chip_id == LC82C168) {
		if(((inl(rtdev->base_addr + CSR5)>>17)&0x07) == 4) {
			/* Rx stopped due to out of buffers,
			 * restart it
			 */
			outl(0x01, rtdev->base_addr + CSR2);
		}
	}
	return refilled;
}


static int tulip_rx(/*RTnet*/struct rtnet_device *rtdev, nanosecs_abs_t *time_stamp)
{
	struct tulip_private *tp = (struct tulip_private *)rtdev->priv;
	int entry = tp->cur_rx % RX_RING_SIZE;
	int rx_work_limit = tp->dirty_rx + RX_RING_SIZE - tp->cur_rx;
	int received = 0;

	if (tulip_debug > 4)
		/*RTnet*/rtdm_printk(KERN_DEBUG " In tulip_rx(), entry %d %8.8x.\n", entry,
			   tp->rx_ring[entry].status);
	/* If we own the next entry, it is a new packet. Send it up. */
	while ( ! (tp->rx_ring[entry].status & cpu_to_le32(DescOwned))) {
		s32 status = le32_to_cpu(tp->rx_ring[entry].status);

		if (tulip_debug > 5)
			/*RTnet*/rtdm_printk(KERN_DEBUG "%s: In tulip_rx(), entry %d %8.8x.\n",
				   rtdev->name, entry, status);
		if (--rx_work_limit < 0)
			break;
		if ((status & 0x38008300) != 0x0300) {
			if ((status & 0x38000300) != 0x0300) {
				/* Ingore earlier buffers. */
				if ((status & 0xffff) != 0x7fff) {
					if (tulip_debug > 1)
						/*RTnet*/rtdm_printk(KERN_WARNING "%s: Oversized Ethernet frame "
							   "spanned multiple buffers, status %8.8x!\n",
							   rtdev->name, status);
					tp->stats.rx_length_errors++;
				}
			} else if (status & RxDescFatalErr) {
				/* There was a fatal error. */
				if (tulip_debug > 2)
					/*RTnet*/rtdm_printk(KERN_DEBUG "%s: Receive error, Rx status %8.8x.\n",
						   rtdev->name, status);
				tp->stats.rx_errors++; /* end of a packet.*/
				if (status & 0x0890) tp->stats.rx_length_errors++;
				if (status & 0x0004) tp->stats.rx_frame_errors++;
				if (status & 0x0002) tp->stats.rx_crc_errors++;
				if (status & 0x0001) tp->stats.rx_fifo_errors++;
			}
		} else {
			/* Omit the four octet CRC from the length. */
			short pkt_len = ((status >> 16) & 0x7ff) - 4;
			struct /*RTnet*/rtskb *skb;

#ifndef final_version
			if (pkt_len > 1518) {
				/*RTnet*/rtdm_printk(KERN_WARNING "%s: Bogus packet size of %d (%#x).\n",
					   rtdev->name, pkt_len, pkt_len);
				pkt_len = 1518;
				tp->stats.rx_length_errors++;
			}
#endif

			{
				unsigned char *temp = /*RTnet*/rtskb_put(skb = tp->rx_buffers[entry].skb, pkt_len);

#ifndef final_version
				if (tp->rx_buffers[entry].mapping !=
				    le32_to_cpu(tp->rx_ring[entry].buffer1)) {
					/*RTnet*/rtdm_printk(KERN_ERR "%s: Internal fault: The skbuff addresses "
					       "do not match in tulip_rx: %08x vs. %08llx ? / %p.\n",
					       rtdev->name,
					       le32_to_cpu(tp->rx_ring[entry].buffer1),
					       (unsigned long long)tp->rx_buffers[entry].mapping,
					       temp);/*RTnet*/
				}
#endif

				pci_unmap_single(tp->pdev, tp->rx_buffers[entry].mapping,
						 PKT_BUF_SZ, PCI_DMA_FROMDEVICE);

				tp->rx_buffers[entry].skb = NULL;
				tp->rx_buffers[entry].mapping = 0;
			}
			skb->protocol = /*RTnet*/rt_eth_type_trans(skb, rtdev);
			skb->time_stamp = *time_stamp;
			/*RTnet*/rtnetif_rx(skb);

			tp->stats.rx_packets++;
			tp->stats.rx_bytes += pkt_len;
		}
		received++;
		entry = (++tp->cur_rx) % RX_RING_SIZE;
	}
	return received;
}

/* The interrupt handler does all of the Rx thread work and cleans up
   after the Tx thread. */
int tulip_interrupt(rtdm_irq_t *irq_handle)
{
	nanosecs_abs_t time_stamp = rtdm_clock_read();/*RTnet*/
	struct rtnet_device *rtdev =
	    rtdm_irq_get_arg(irq_handle, struct rtnet_device);/*RTnet*/
	struct tulip_private *tp = (struct tulip_private *)rtdev->priv;
	long ioaddr = rtdev->base_addr;
	unsigned int csr5;
	int entry;
	int missed;
	int rx = 0;
	int tx = 0;
	int oi = 0;
	int maxrx = RX_RING_SIZE;
	int maxtx = TX_RING_SIZE;
	int maxoi = TX_RING_SIZE;
	unsigned int work_count = tulip_max_interrupt_work;

	/* Let's see whether the interrupt really is for us */
	csr5 = inl(ioaddr + CSR5);

	if ((csr5 & (NormalIntr|AbnormalIntr)) == 0) {
		rtdm_printk("%s: unexpected IRQ!\n",rtdev->name);
		return RTDM_IRQ_NONE;
	}

	tp->nir++;

	do {
		/* Acknowledge all of the current interrupt sources ASAP. */
		outl(csr5 & 0x0001ffff, ioaddr + CSR5);

		if (tulip_debug > 4)
			/*RTnet*/rtdm_printk(KERN_DEBUG "%s: interrupt  csr5=%#8.8x new csr5=%#8.8x.\n",
				   rtdev->name, csr5, inl(rtdev->base_addr + CSR5));

		if (csr5 & (RxIntr | RxNoBuf)) {
			rx += tulip_rx(rtdev, &time_stamp);
			tulip_refill_rx(rtdev);
		}

		if (csr5 & (TxNoBuf | TxDied | TxIntr | TimerInt)) {
			unsigned int dirty_tx;

			rtdm_lock_get(&tp->lock);

			for (dirty_tx = tp->dirty_tx; tp->cur_tx - dirty_tx > 0;
				 dirty_tx++) {
				int entry = dirty_tx % TX_RING_SIZE;
				int status = le32_to_cpu(tp->tx_ring[entry].status);

				if (status < 0)
					break;			/* It still has not been Txed */

				/* Check for Rx filter setup frames. */
				if (tp->tx_buffers[entry].skb == NULL) {
					/* test because dummy frames not mapped */
					if (tp->tx_buffers[entry].mapping)
						pci_unmap_single(tp->pdev,
							 tp->tx_buffers[entry].mapping,
							 sizeof(tp->setup_frame),
							 PCI_DMA_TODEVICE);
					continue;
				}

				if (status & 0x8000) {
					/* There was an major error, log it. */
#ifndef final_version
					if (tulip_debug > 1)
						/*RTnet*/rtdm_printk(KERN_DEBUG "%s: Transmit error, Tx status %8.8x.\n",
							   rtdev->name, status);
#endif
					tp->stats.tx_errors++;
					if (status & 0x4104) tp->stats.tx_aborted_errors++;
					if (status & 0x0C00) tp->stats.tx_carrier_errors++;
					if (status & 0x0200) tp->stats.tx_window_errors++;
					if (status & 0x0002) tp->stats.tx_fifo_errors++;
					if ((status & 0x0080) && tp->full_duplex == 0)
						tp->stats.tx_heartbeat_errors++;
				} else {
					tp->stats.tx_bytes +=
						tp->tx_buffers[entry].skb->len;
					tp->stats.collisions += (status >> 3) & 15;
					tp->stats.tx_packets++;
				}

				pci_unmap_single(tp->pdev, tp->tx_buffers[entry].mapping,
						 tp->tx_buffers[entry].skb->len,
						 PCI_DMA_TODEVICE);

				/* Free the original skb. */
				/*RTnet*/dev_kfree_rtskb(tp->tx_buffers[entry].skb);
				tp->tx_buffers[entry].skb = NULL;
				tp->tx_buffers[entry].mapping = 0;
				tx++;
				rtnetif_tx(rtdev);
			}

#ifndef final_version
			if (tp->cur_tx - dirty_tx > TX_RING_SIZE) {
				/*RTnet*/rtdm_printk(KERN_ERR "%s: Out-of-sync dirty pointer, %d vs. %d.\n",
					   rtdev->name, dirty_tx, tp->cur_tx);
				dirty_tx += TX_RING_SIZE;
			}
#endif

			if (tp->cur_tx - dirty_tx < TX_RING_SIZE - 2)
				/*RTnet*/rtnetif_wake_queue(rtdev);

			tp->dirty_tx = dirty_tx;
			if (csr5 & TxDied) {
				if (tulip_debug > 2)
					/*RTnet*/rtdm_printk(KERN_WARNING "%s: The transmitter stopped."
						   "  CSR5 is %x, CSR6 %x, new CSR6 %x.\n",
						   rtdev->name, csr5, inl(ioaddr + CSR6), tp->csr6);
				tulip_restart_rxtx(tp);
			}
			rtdm_lock_put(&tp->lock);
		}

		/* Log errors. */
		if (csr5 & AbnormalIntr) {	/* Abnormal error summary bit. */
			if (csr5 == 0xffffffff)
				break;
			/*RTnet*/rtdm_printk(KERN_ERR "%s: Error detected, "
			    "device may not work any more (csr5=%08x)!\n", rtdev->name, csr5);
			/* Clear all error sources, included undocumented ones! */
			outl(0x0800f7ba, ioaddr + CSR5);
			oi++;
		}
		if (csr5 & TimerInt) {

			if (tulip_debug > 2)
				/*RTnet*/rtdm_printk(KERN_ERR "%s: Re-enabling interrupts, %8.8x.\n",
					   rtdev->name, csr5);
			outl(tulip_tbl[tp->chip_id].valid_intrs, ioaddr + CSR7);
			tp->ttimer = 0;
			oi++;
		}
		if (tx > maxtx || rx > maxrx || oi > maxoi) {
			if (tulip_debug > 1)
				/*RTnet*/rtdm_printk(KERN_WARNING "%s: Too much work during an interrupt, "
					   "csr5=0x%8.8x. (%lu) (%d,%d,%d)\n", rtdev->name, csr5, tp->nir, tx, rx, oi);

		       /* Acknowledge all interrupt sources. */
			outl(0x8001ffff, ioaddr + CSR5);
			if (tp->flags & HAS_INTR_MITIGATION) {
		     /* Josip Loncaric at ICASE did extensive experimentation
			to develop a good interrupt mitigation setting.*/
				outl(0x8b240000, ioaddr + CSR11);
			} else if (tp->chip_id == LC82C168) {
				/* the LC82C168 doesn't have a hw timer.*/
				outl(0x00, ioaddr + CSR7);
			} else {
			  /* Mask all interrupting sources, set timer to
				re-enable. */
			}
			break;
		}

		work_count--;
		if (work_count == 0)
			break;

		csr5 = inl(ioaddr + CSR5);
	} while ((csr5 & (NormalIntr|AbnormalIntr)) != 0);

	tulip_refill_rx(rtdev);

	/* check if the card is in suspend mode */
	entry = tp->dirty_rx % RX_RING_SIZE;
	if (tp->rx_buffers[entry].skb == NULL) {
		if (tulip_debug > 1)
			/*RTnet*/rtdm_printk(KERN_WARNING "%s: in rx suspend mode: (%lu) (tp->cur_rx = %u, ttimer = %d, rx = %d) go/stay in suspend mode\n", rtdev->name, tp->nir, tp->cur_rx, tp->ttimer, rx);
		if (tp->chip_id == LC82C168)
			outl(0x00, ioaddr + CSR7);
		else {
			if (tp->ttimer == 0 || (inl(ioaddr + CSR11) & 0xffff) == 0) {
				if (tulip_debug > 1)
					/*RTnet*/rtdm_printk(KERN_WARNING "%s: in rx suspend mode: (%lu) set timer\n", rtdev->name, tp->nir);
				outl(tulip_tbl[tp->chip_id].valid_intrs | TimerInt,
					ioaddr + CSR7);
				outl(TimerInt, ioaddr + CSR5);
				outl(12, ioaddr + CSR11);
				tp->ttimer = 1;
			}
		}
	}

	if ((missed = inl(ioaddr + CSR8) & 0x1ffff)) {
		tp->stats.rx_dropped += missed & 0x10000 ? 0x10000 : missed;
	}

	if (tulip_debug > 4)
		/*RTnet*/rtdm_printk(KERN_DEBUG "%s: exiting interrupt, csr5=%#4.4x.\n",
			   rtdev->name, inl(ioaddr + CSR5));
	if (rx)
		rt_mark_stack_mgr(rtdev);
	return RTDM_IRQ_HANDLED;
}
