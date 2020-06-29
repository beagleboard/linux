// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments PRUETH hsr/prp Link Redunancy Entity (LRE) Driver.
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/spinlock_types.h>

#include "icss_lre_firmware.h"
#include "prueth.h"
#include "prueth_lre.h"
#include "prueth_switch.h"

void prueth_lre_config_check_flags(struct prueth *prueth)
{
	void __iomem *dram1 = prueth->mem[PRUETH_MEM_DRAM1].va;

	/* HSR/PRP: initialize check table when first port is up */
	if (prueth->emac_configured)
		return;

	prueth->tbl_check_mask = (ICSS_LRE_HOST_TIMER_NODE_TABLE_CHECK_BIT |
				  ICSS_LRE_HOST_TIMER_HOST_TABLE_CHECK_BIT);
	if (PRUETH_IS_HSR(prueth))
		prueth->tbl_check_mask |=
			ICSS_LRE_HOST_TIMER_PORT_TABLE_CHECK_BITS;
	writel(prueth->tbl_check_mask, dram1 + ICSS_LRE_HOST_TIMER_CHECK_FLAGS);
}

/* A group of PCPs are mapped to a Queue. This is the size of firmware
 * array in shared memory
 */
#define PCP_GROUP_TO_QUEUE_MAP_SIZE	4

/* PRU firmware default PCP to priority Queue map for ingress & egress
 *
 * At ingress to Host
 * ==================
 * byte 0 => PRU 1, PCP 0-3 => Q3
 * byte 1 => PRU 1, PCP 4-7 => Q2
 * byte 2 => PRU 0, PCP 0-3 => Q1
 * byte 3 => PRU 0, PCP 4-7 => Q0
 *
 * At egress to wire/network on PRU-0 and PRU-1
 * ============================================
 * byte 0 => Host, PCP 0-3 => Q3
 * byte 1 => Host, PCP 4-7 => Q2
 *
 * PRU-0
 * -----
 * byte 2 => PRU-1, PCP 0-3 => Q1
 * byte 3 => PRU-1, PCP 4-7 => Q0
 *
 * PRU-1
 * -----
 * byte 2 => PRU-0, PCP 0-3 => Q1
 * byte 3 => PRU-0, PCP 4-7 => Q0
 *
 * queue names below are named 1 based. i.e PRUETH_QUEUE1 is Q0,
 * PRUETH_QUEUE2 is Q1 and so forth. Firmware convention is that
 * a lower queue number has higher priority than a higher queue
 * number.
 */
static u8 fw_pcp_default_priority_queue_map[PCP_GROUP_TO_QUEUE_MAP_SIZE] = {
	/* port 2 or PRU 1 */
	PRUETH_QUEUE4, PRUETH_QUEUE3,
	/* port 1 or PRU 0 */
	PRUETH_QUEUE2, PRUETH_QUEUE1,
};

static void prueth_lre_pcp_queue_map_config(struct prueth *prueth)
{
	void __iomem *sram  = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	memcpy_toio(sram + ICSS_LRE_QUEUE_2_PCP_MAP_OFFSET,
		    &fw_pcp_default_priority_queue_map[0],
		    PCP_GROUP_TO_QUEUE_MAP_SIZE);
}

static void prueth_lre_host_table_init(struct prueth *prueth)
{
	void __iomem *dram0 = prueth->mem[PRUETH_MEM_DRAM0].va;
	void __iomem *dram1 = prueth->mem[PRUETH_MEM_DRAM1].va;

	memset_io(dram0 + ICSS_LRE_DUPLICATE_HOST_TABLE, 0,
		  ICSS_LRE_DUPLICATE_HOST_TABLE_DMEM_SIZE);

	writel(ICSS_LRE_DUPLICATE_HOST_TABLE_SIZE_INIT,
	       dram1 + ICSS_LRE_DUPLICATE_HOST_TABLE_SIZE);

	writel(ICSS_LRE_TABLE_CHECK_RESOLUTION_10_MS,
	       dram1 + ICSS_LRE_DUPLI_HOST_CHECK_RESO);

	writel(ICSS_LRE_MASTER_SLAVE_BUSY_BITS_CLEAR,
	       dram1 + ICSS_LRE_HOST_DUPLICATE_ARBITRATION);
}

#define IND_BINOFS(x)		nt->index_array->index_tbl[x].bin_offset
#define BIN_NODEOFS(x)		nt->bin_array->bin_tbl[x].node_tbl_offset

static void _prueth_lre_init_node_table(struct prueth *prueth)
{
	struct node_tbl *nt = prueth->nt;
	int j;

	const struct prueth_fw_offsets *fw_offsets = prueth->fw_offsets;

	nt->nt_array = prueth->mem[fw_offsets->nt_array_loc].va +
		       fw_offsets->nt_array_offset;
	memset_io(nt->nt_array, 0, sizeof(struct node_tbl_t) *
		  fw_offsets->nt_array_max_entries);

	nt->bin_array = prueth->mem[fw_offsets->bin_array_loc].va +
			fw_offsets->bin_array_offset;
	memset_io(nt->bin_array, 0, sizeof(struct bin_tbl_t) *
		  fw_offsets->bin_array_max_entries);

	nt->index_array = prueth->mem[fw_offsets->index_array_loc].va +
			  fw_offsets->index_array_offset;
	memset_io(nt->index_array, 0, sizeof(struct node_index_tbl_t) *
		  fw_offsets->index_array_max_entries);

	nt->nt_info = prueth->mem[fw_offsets->nt_array_loc].va +
		      fw_offsets->nt_array_offset +
		      (sizeof(struct node_tbl_t) *
		       fw_offsets->nt_array_max_entries);
	memset_io(nt->nt_info, 0, sizeof(struct node_tbl_info_t));

	nt->nt_lre_cnt =
		prueth->mem[PRUETH_MEM_SHARED_RAM].va + ICSS_LRE_CNT_NODES;
	memset_io(nt->nt_lre_cnt, 0, sizeof(struct node_tbl_lre_cnt_t));

	nt->nt_array_max_entries = fw_offsets->nt_array_max_entries;
	nt->bin_array_max_entries = fw_offsets->bin_array_max_entries;
	nt->index_array_max_entries = fw_offsets->index_array_max_entries;
	nt->hash_mask = fw_offsets->hash_mask;

	for (j = 0; j < fw_offsets->index_array_max_entries; j++)
		IND_BINOFS(j) = fw_offsets->bin_array_max_entries;
	for (j = 0; j < fw_offsets->bin_array_max_entries; j++)
		BIN_NODEOFS(j) = fw_offsets->nt_array_max_entries;
	for (j = 0; j < fw_offsets->nt_array_max_entries; j++)
		nt->nt_array->node_tbl[j].entry_state = ICSS_LRE_NODE_FREE;
}

static void prueth_lre_port_table_init(struct prueth *prueth)
{
	void __iomem *dram1 = prueth->mem[PRUETH_MEM_DRAM1].va;

	if (PRUETH_IS_HSR(prueth)) {
		memset_io(dram1 + ICSS_LRE_DUPLICATE_PORT_TABLE_PRU0, 0,
			  ICSS_LRE_DUPLICATE_PORT_TABLE_DMEM_SIZE);
		memset_io(dram1 + ICSS_LRE_DUPLICATE_PORT_TABLE_PRU1, 0,
			  ICSS_LRE_DUPLICATE_PORT_TABLE_DMEM_SIZE);

		writel(ICSS_LRE_DUPLICATE_PORT_TABLE_SIZE_INIT,
		       dram1 + ICSS_LRE_DUPLICATE_PORT_TABLE_SIZE);
	} else {
		writel(0, dram1 + ICSS_LRE_DUPLICATE_PORT_TABLE_SIZE);
	}

	writel(ICSS_LRE_TABLE_CHECK_RESOLUTION_10_MS,
	       dram1 + ICSS_LRE_DUPLI_PORT_CHECK_RESO);
}

static void prueth_lre_init(struct prueth *prueth)
{
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	memset_io(sram + ICSS_LRE_START, 0, ICSS_LRE_STATS_DMEM_SIZE);

	writel(ICSS_LRE_IEC62439_CONST_DUPLICATE_DISCARD,
	       sram + ICSS_LRE_DUPLICATE_DISCARD);
	writel(ICSS_LRE_IEC62439_CONST_TRANSP_RECEPTION_REMOVE_RCT,
	       sram + ICSS_LRE_TRANSPARENT_RECEPTION);
	prueth->prp_tr_mode = IEC62439_3_TR_REMOVE_RCT;
}

static void prueth_lre_dbg_init(struct prueth *prueth)
{
	void __iomem *dram0 = prueth->mem[PRUETH_MEM_DRAM0].va;

	memset_io(dram0 + ICSS_LRE_DBG_START, 0,
		  ICSS_LRE_DEBUG_COUNTER_DMEM_SIZE);
}

static void prueth_lre_protocol_init(struct prueth *prueth)
{
	void __iomem *dram0 = prueth->mem[PRUETH_MEM_DRAM0].va;
	void __iomem *dram1 = prueth->mem[PRUETH_MEM_DRAM1].va;

	if (PRUETH_IS_HSR(prueth))
		writew(prueth->hsr_mode, dram0 + ICSS_LRE_HSR_MODE);

	writel(ICSS_LRE_DUPLICATE_FORGET_TIME_400_MS,
	       dram1 + ICSS_LRE_DUPLI_FORGET_TIME);
	writel(ICSS_LRE_SUP_ADDRESS_INIT_OCTETS_HIGH,
	       dram1 + ICSS_LRE_SUP_ADDR);
	writel(ICSS_LRE_SUP_ADDRESS_INIT_OCTETS_LOW,
	       dram1 + ICSS_LRE_SUP_ADDR_LOW);
}

static void prueth_lre_config_packet_timestamping(struct prueth *prueth)
{
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	writeb(1, sram + ICSS_LRE_PRIORITY_INTRS_STATUS_OFFSET);
	writeb(1, sram + ICSS_LRE_TIMESTAMP_PKTS_STATUS_OFFSET);
}

void prueth_lre_config(struct prueth *prueth)
{
	if (PRUETH_IS_HSR(prueth))
		prueth->hsr_mode = ICSS_LRE_MODEH;

	prueth_lre_pcp_queue_map_config(prueth);
	prueth_lre_host_table_init(prueth);
	prueth_lre_port_table_init(prueth);
	prueth_lre_init(prueth);
	prueth_lre_dbg_init(prueth);
	prueth_lre_protocol_init(prueth);
	/* for HSR/PRP LRE driver order the frames based on
	 * packet timestamp.
	 */
	prueth_lre_config_packet_timestamping(prueth);
}

static int prueth_lre_emac_rx_packets(struct prueth_emac *emac,
				      int quota, u8 qid1, u8 qid2)
{
	struct prueth *prueth = emac->prueth;
	void *ocmc_ram = (__force void *)prueth->mem[PRUETH_MEM_OCMC].va;
	u16 bd_rd_ptr, bd_wr_ptr, update_rd_ptr, bd_rd_ptr_o, bd_wr_ptr_o;
	void __iomem *shared_ram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	struct prueth_queue_desc __iomem *queue_desc, *queue_desc_o;
	struct net_device_stats *ndevstats = &emac->ndev->stats;
	int ret, used = 0, port, port0_q_empty, port1_q_empty;
	unsigned int emac_max_pktlen = PRUETH_MAX_PKTLEN_LRE;
	const struct prueth_queue_info *rxqueue, *rxqueue_o;
	u8 overflow_cnt, overflow_cnt_o, status, status_o;
	struct prueth_queue_desc __iomem *queue_desc_p;
	struct prueth_packet_info pkt_info, pkt_info_o;
	const struct prueth_queue_info *rxqueue_p;
	struct prueth_packet_info *pkt_info_p;
	struct net_device_stats *ndevstats_o;
	struct net_device_stats *ndevstats_p;
	u32 rd_buf_desc, rd_buf_desc_o;
	struct prueth_emac *other_emac;
	u16 *bd_rd_ptr_p, *bd_wr_ptr_p;
	struct prueth_emac *emac_p;
	u32 pkt_ts, pkt_ts_o;
	u32 iep_wrap;

	other_emac = prueth->emac[(emac->port_id ^ 0x3) - 1];
	ndevstats_o = &other_emac->ndev->stats;

	/* use the correct wrap value based on ICSSM version */
	iep_wrap = prueth->fw_offsets->iep_wrap;
	/* search host queues for packets */
	queue_desc = emac->rx_queue_descs + qid1;
	queue_desc_o = other_emac->rx_queue_descs + qid2;

	rxqueue = &sw_queue_infos[PRUETH_PORT_HOST][qid1];
	rxqueue_o = &sw_queue_infos[PRUETH_PORT_HOST][qid2];

	status = readb(&queue_desc->status);
	status_o = readb(&queue_desc_o->status);

	overflow_cnt = readb(&queue_desc->overflow_cnt);
	overflow_cnt_o = readb(&queue_desc_o->overflow_cnt);

	if (overflow_cnt > 0) {
		emac->ndev->stats.rx_over_errors += overflow_cnt;
		/* reset to zero */
		writeb(0, &queue_desc->overflow_cnt);
	}
	if (overflow_cnt_o > 0) {
		other_emac->ndev->stats.rx_over_errors += overflow_cnt_o;

		/* reset to zero */
		writeb(0, &queue_desc_o->overflow_cnt);
	}

	bd_rd_ptr = readw(&queue_desc->rd_ptr);
	bd_wr_ptr = readw(&queue_desc->wr_ptr);

	bd_rd_ptr_o = readw(&queue_desc_o->rd_ptr);
	bd_wr_ptr_o = readw(&queue_desc_o->wr_ptr);

	port0_q_empty = (bd_rd_ptr == bd_wr_ptr) ? 1 : 0;
	port1_q_empty = (bd_rd_ptr_o == bd_wr_ptr_o) ? 1 : 0;

	/* while packets are available in this queue */
	while (!port0_q_empty || !port1_q_empty) {
		/* get packet info from the read buffer descriptor */
		rd_buf_desc = readl(shared_ram + bd_rd_ptr);
		rd_buf_desc_o = readl(shared_ram + bd_rd_ptr_o);

		parse_packet_info(prueth, rd_buf_desc, &pkt_info);
		parse_packet_info(prueth, rd_buf_desc_o, &pkt_info_o);

		pkt_ts = readl(ocmc_ram + ICSS_LRE_TIMESTAMP_ARRAY_OFFSET +
			       bd_rd_ptr - SRAM_START_OFFSET);
		pkt_ts_o = readl(ocmc_ram + ICSS_LRE_TIMESTAMP_ARRAY_OFFSET +
				 bd_rd_ptr_o - SRAM_START_OFFSET);

		if (!port0_q_empty && !port1_q_empty) {
			/* Packets in both port queues */
			/* Calculate diff b/n timestamps and account for
			 * wraparound
			 */
			if (pkt_ts > pkt_ts_o)
				port = (pkt_ts - pkt_ts_o) > (iep_wrap / 2) ?
					0 : 1;
			else
				port = (pkt_ts_o - pkt_ts) > (iep_wrap / 2) ?
					1 : 0;

		} else if (!port0_q_empty) {
			/* Packet(s) in port0 queue only */
			port = 0;
		} else {
			/* Packet(s) in port1 queue only */
			port = 1;
		}

		/* Select correct data structures for queue/packet selected */
		if (port == 0) {
			pkt_info_p = &pkt_info;
			bd_wr_ptr_p = &bd_wr_ptr;
			bd_rd_ptr_p = &bd_rd_ptr;
			emac_p = emac;
			ndevstats_p = ndevstats;
			rxqueue_p = rxqueue;
			queue_desc_p = queue_desc;
		} else {
			pkt_info_p = &pkt_info_o;
			bd_wr_ptr_p = &bd_wr_ptr_o;
			bd_rd_ptr_p = &bd_rd_ptr_o;
			emac_p = other_emac;
			ndevstats_p = ndevstats_o;
			rxqueue_p = rxqueue_o;
			queue_desc_p = queue_desc_o;
		}

		if ((*pkt_info_p).length <= 0) {
			/* a packet length of zero will cause us to
			 * never move the read pointer ahead, locking
			 * the driver, so we manually have to move it
			 * to the write pointer, discarding all
			 * remaining packets in this queue. This should
			 * never happen.
			 */
			update_rd_ptr = *bd_wr_ptr_p;
			ndevstats_p->rx_length_errors++;
		} else if ((*pkt_info_p).length > emac_max_pktlen) {
			/* if the packet is too large we skip it but we
			 * still need to move the read pointer ahead
			 * and assume something is wrong with the read
			 * pointer as the firmware should be filtering
			 * these packets
			 */
			update_rd_ptr = *bd_wr_ptr_p;
			ndevstats_p->rx_length_errors++;
		} else {
			update_rd_ptr = *bd_rd_ptr_p;
			ret = emac_rx_packet(emac_p, &update_rd_ptr,
					     *pkt_info_p, rxqueue_p);
			if (ret)
				return ret;

			used++;
		}

		/* after reading the buffer descriptor we clear it
		 * to prevent improperly moved read pointer errors
		 * from simply looking like old packets.
		 */

		/* update read pointer in queue descriptor */
		if (port == 0) {
			writel(0, shared_ram + bd_rd_ptr);
			writew(update_rd_ptr, &queue_desc->rd_ptr);
			bd_rd_ptr = update_rd_ptr;
		} else {
			writel(0, shared_ram + bd_rd_ptr_o);
			writew(update_rd_ptr, &queue_desc_o->rd_ptr);
			bd_rd_ptr_o = update_rd_ptr;
		}

		port0_q_empty = (bd_rd_ptr == bd_wr_ptr) ? 1 : 0;
		port1_q_empty = (bd_rd_ptr_o == bd_wr_ptr_o) ? 1 : 0;

		/* all we have room for? */
		if (used >= quota)
			return used;
	}

	return used;
}

int prueth_lre_napi_poll_lpq(struct napi_struct *napi, int budget)
{
	struct prueth *prueth = container_of(napi, struct prueth, napi_lpq);
	struct net_device *ndev = prueth->lp->ndev;
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 qid1 = PRUETH_QUEUE2, qid2 = PRUETH_QUEUE4;
	int num_rx_packets;

	num_rx_packets = prueth_lre_emac_rx_packets(emac, budget, qid1, qid2);
	if (num_rx_packets < budget)
		emac_finish_napi(emac, napi, prueth->rx_lpq_irq);

	return num_rx_packets;
}

int prueth_lre_napi_poll_hpq(struct napi_struct *napi, int budget)
{
	struct prueth *prueth = container_of(napi, struct prueth, napi_hpq);
	struct net_device *ndev = prueth->hp->ndev;
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 qid1 = PRUETH_QUEUE1, qid2 = PRUETH_QUEUE3;
	int num_rx_packets;

	num_rx_packets = prueth_lre_emac_rx_packets(emac, budget, qid1, qid2);
	if (num_rx_packets < budget)
		emac_finish_napi(emac, napi, prueth->rx_hpq_irq);

	return num_rx_packets;
}

irqreturn_t prueth_lre_emac_rx_hardirq(int irq, void *dev_id)
{
	struct prueth_ndev_priority *ndev_prio =
		(struct prueth_ndev_priority *)dev_id;
	struct net_device *ndev = ndev_prio->ndev;
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	/* disable Rx system event */
	if (ndev_prio->priority == 1) {
		disable_irq_nosync(prueth->rx_lpq_irq);
		napi_schedule(&prueth->napi_lpq);
	} else {
		disable_irq_nosync(prueth->rx_hpq_irq);
		napi_schedule(&prueth->napi_hpq);
	}

	return IRQ_HANDLED;
}

int prueth_lre_request_irqs(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int ret;

	/* HSR/PRP. Request irq when first port is initialized */
	if (prueth->emac_configured)
		return 0;

	ret = request_irq(prueth->rx_hpq_irq, prueth_lre_emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH, "eth_hp_int", prueth->hp);
	if (ret) {
		netdev_err(emac->ndev, "unable to request RX HPQ IRQ\n");
		return ret;
	}

	ret = request_irq(prueth->rx_lpq_irq, prueth_lre_emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH, "eth_lp_int", prueth->lp);
	if (ret) {
		netdev_err(emac->ndev, "unable to request RX LPQ IRQ\n");
		free_irq(prueth->rx_hpq_irq, prueth->hp);
		return ret;
	}

	return ret;
}

void prueth_lre_free_irqs(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;

	/* HSR/PRP: free irqs when last port is down */
	if (prueth->emac_configured)
		return;

	free_irq(prueth->rx_lpq_irq, prueth->lp);
	free_irq(prueth->rx_hpq_irq, prueth->hp);
}

void prueth_lre_free_memory(struct prueth *prueth)
{
	/* HSR/PRP: initialize node table when first port is up */
	if (prueth->emac_configured)
		return;

	kfree(prueth->nt);
	prueth->nt = NULL;
}

int prueth_lre_init_node_table(struct prueth *prueth)
{
	/* HSR/PRP: initialize node table when first port is up */
	if (prueth->emac_configured)
		return 0;

	/* initialize for node table handling in driver for HSR/PRP */
	prueth->nt = kmalloc(sizeof(*prueth->nt), GFP_KERNEL);
	if (!prueth->nt)
		return -ENOMEM;

	_prueth_lre_init_node_table(prueth);

	return 0;
}
