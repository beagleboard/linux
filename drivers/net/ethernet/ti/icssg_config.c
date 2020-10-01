// SPDX-License-Identifier: GPL-2.0
/* ICSSG Ethernet driver
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#include <linux/regmap.h>
#include "icssg_config.h"
#include "icssg_prueth.h"
#include "icssg_switch_map.h"
#include "icss_mii_rt.h"

/* TX IPG Values to be set for 100M and 1G link speeds.  These values are
 * in ocp_clk cycles. So need change if ocp_clk is changed for a specific
 * h/w design.
 */

/* SR1.0 IPG is in core_clk cycles */
#define MII_RT_TX_IPG_100M_SR1	0x166
#define MII_RT_TX_IPG_1G_SR1	0x18

/* SR2.0 IPG is in rgmii_clk (125MHz) clock cycles + 1 */
#define MII_RT_TX_IPG_100M	0xb2	/* FIXME: cross check */
#define MII_RT_TX_IPG_1G	0xb

#define	ICSSG_QUEUES_MAX		64
#define	ICSSG_QUEUE_OFFSET		0xd00
#define	ICSSG_QUEUE_PEEK_OFFSET		0xe00
#define	ICSSG_QUEUE_CNT_OFFSET		0xe40
#define	ICSSG_QUEUE_RESET_OFFSET	0xf40

#define	ICSSG_NUM_TX_QUEUES	8

#define	RECYCLE_Q_SLICE0	16
#define	RECYCLE_Q_SLICE1	17

#define	ICSSG_NUM_OTHER_QUEUES	5	/* port, host and special queues */

#define	PORT_HI_Q_SLICE0	32
#define	PORT_LO_Q_SLICE0	33
#define	HOST_HI_Q_SLICE0	34
#define	HOST_LO_Q_SLICE0	35
#define	HOST_SPL_Q_SLICE0	40	/* Special Queue */

#define	PORT_HI_Q_SLICE1	36
#define	PORT_LO_Q_SLICE1	37
#define	HOST_HI_Q_SLICE1	38
#define	HOST_LO_Q_SLICE1	39
#define	HOST_SPL_Q_SLICE1	41	/* Special Queue */

#define MII_RXCFG_DEFAULT	(PRUSS_MII_RT_RXCFG_RX_ENABLE | \
				 PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS | \
				 PRUSS_MII_RT_RXCFG_RX_L2_EN | \
				 PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS)

#define MII_TXCFG_DEFAULT	(PRUSS_MII_RT_TXCFG_TX_ENABLE | \
				 PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE | \
				 PRUSS_MII_RT_TXCFG_TX_32_MODE_EN | \
				 PRUSS_MII_RT_TXCFG_TX_IPG_WIRE_CLK_EN)

#define ICSSG_CFG_DEFAULT	(ICSSG_CFG_TX_L1_EN | \
				 ICSSG_CFG_TX_L2_EN | ICSSG_CFG_RX_L2_G_EN | \
				 ICSSG_CFG_TX_PRU_EN | /* SR2.0 only */ \
				 ICSSG_CFG_SGMII_MODE)

struct map {
	int queue;
	u32 pd_addr_start;
	u32 flags;
	bool special;
};

struct map hwq_map[2][ICSSG_NUM_OTHER_QUEUES] = {
	{
		{ PORT_HI_Q_SLICE0, PORT_DESC0_HI, 0x200000, 0 },
		{ PORT_LO_Q_SLICE0, PORT_DESC0_LO, 0, 0 },
		{ HOST_HI_Q_SLICE0, HOST_DESC0_HI, 0x200000, 0 },
		{ HOST_LO_Q_SLICE0, HOST_DESC0_LO, 0, 0 },
		{ HOST_SPL_Q_SLICE0, HOST_SPPD0, 0x400000, 1 },
	},
	{
		{ PORT_HI_Q_SLICE1, PORT_DESC1_HI, 0xa00000, 0 },
		{ PORT_LO_Q_SLICE1, PORT_DESC1_LO, 0x800000, 0 },
		{ HOST_HI_Q_SLICE1, HOST_DESC1_HI, 0xa00000, 0 },
		{ HOST_LO_Q_SLICE1, HOST_DESC1_LO, 0x800000, 0 },
		{ HOST_SPL_Q_SLICE1, HOST_SPPD1, 0xc00000, 1 },
	},
};

static void icssg_config_mii_init(struct prueth *prueth, int mii)
{
	struct regmap *mii_rt = prueth->mii_rt;
	u32 rxcfg_reg, txcfg_reg, pcnt_reg;
	u32 rxcfg, txcfg;

	rxcfg_reg = (mii == ICSS_MII0) ? PRUSS_MII_RT_RXCFG0 :
				       PRUSS_MII_RT_RXCFG1;
	txcfg_reg = (mii == ICSS_MII0) ? PRUSS_MII_RT_TXCFG0 :
				       PRUSS_MII_RT_TXCFG1;
	pcnt_reg = (mii == ICSS_MII0) ? PRUSS_MII_RT_RX_PCNT0 :
				       PRUSS_MII_RT_RX_PCNT1;

	icssg_config_ipg(prueth, SPEED_1000, mii);

	rxcfg = MII_RXCFG_DEFAULT;
	txcfg = MII_TXCFG_DEFAULT;

	if (mii == ICSS_MII1) {
		rxcfg |= PRUSS_MII_RT_RXCFG_RX_MUX_SEL;
		txcfg |= PRUSS_MII_RT_TXCFG_TX_MUX_SEL;
	}

	regmap_write(mii_rt, rxcfg_reg, rxcfg);
	regmap_write(mii_rt, txcfg_reg, txcfg);
	regmap_write(mii_rt, pcnt_reg, 0x1);
}

static void icssg_config_rgmii_init(struct prueth *prueth, int slice)
{
	void __iomem *smem = prueth->shram.va;
	struct regmap *miig_rt = prueth->miig_rt;
	int queue = 0, i, j;
	u8 pd[ICSSG_SPECIAL_PD_SIZE];
	u32 *pdword;
	u32 mii_mode;

	mii_mode = MII_MODE_RGMII << ICSSG_CFG_MII0_MODE_SHIFT;
	mii_mode |= MII_MODE_RGMII << ICSSG_CFG_MII1_MODE_SHIFT;
	regmap_write(miig_rt, ICSSG_CFG_OFFSET, ICSSG_CFG_DEFAULT | mii_mode);

	icssg_update_rgmii_cfg(miig_rt, true, true, slice);
	/* reset hwqueues */
	if (slice)
		queue = ICSSG_NUM_TX_QUEUES;

	for (i = 0; i < ICSSG_NUM_TX_QUEUES; i++) {
		regmap_write(miig_rt, ICSSG_QUEUE_RESET_OFFSET, queue);
		queue++;
	}

	queue = slice ? RECYCLE_Q_SLICE1 : RECYCLE_Q_SLICE0;
	regmap_write(miig_rt, ICSSG_QUEUE_RESET_OFFSET, queue);

	for (i = 0; i < ICSSG_NUM_OTHER_QUEUES; i++) {
		regmap_write(miig_rt, ICSSG_QUEUE_RESET_OFFSET,
			     hwq_map[slice][i].queue);
	}

	/* initialize packet descriptors in SMEM */
	/* push pakcet descriptors to hwqueues */

	pdword = (u32 *)pd;
	for (j = 0; j < ICSSG_NUM_OTHER_QUEUES; j++) {
		struct map *mp;
		int pd_size, num_pds;
		u32 pdaddr;

		mp = &hwq_map[slice][j];
		if (mp->special) {
			pd_size = ICSSG_SPECIAL_PD_SIZE;
			num_pds = ICSSG_NUM_SPECIAL_PDS;
		} else	{
			pd_size = ICSSG_NORMAL_PD_SIZE;
			num_pds = ICSSG_NUM_NORMAL_PDS;
		}

		for (i = 0; i < num_pds; i++) {
			memset(pd, 0, pd_size);

			pdword[0] &= cpu_to_le32(ICSSG_FLAG_MASK);
			pdword[0] |= cpu_to_le32(mp->flags);
			pdaddr = mp->pd_addr_start + i * pd_size;

			memcpy_toio(smem + pdaddr, pd, pd_size);
			queue = mp->queue;
			regmap_write(miig_rt, ICSSG_QUEUE_OFFSET + 4 * queue,
				     pdaddr);
		}
	}
}

void icssg_config_ipg(struct prueth *prueth, int speed, int mii)
{
	switch (speed) {
	case SPEED_1000:
		icssg_mii_update_ipg(prueth->mii_rt, mii, prueth->is_sr1 ?
				     MII_RT_TX_IPG_1G_SR1 : MII_RT_TX_IPG_1G);
		break;
	case SPEED_100:
		icssg_mii_update_ipg(prueth->mii_rt, mii, prueth->is_sr1 ?
				     MII_RT_TX_IPG_100M_SR1 : MII_RT_TX_IPG_100M);
		break;
	default:
		/* Other links speeds not supported */
		pr_err("Unsupported link speed\n");
		return;
	}
}

/* SR1: Set buffer sizes for the pools. There are 8 internal queues
 * implemented in firmware, but only 4 tx channels/threads in the Egress
 * direction to firmware. Need a high priority queue for management
 * messages since they shouldn't be blocked even during high traffic
 * situation. So use Q0-Q2 as data queues and Q3 as management queue
 * in the max case. However for ease of configuration, use the max
 * data queue + 1 for management message if we are not using max
 * case.
 *
 * Allocate 4 MTU buffers per data queue.  Firmware requires
 * pool sizes to be set for internal queues. Set the upper 5 queue
 * pool size to min size of 128 bytes since there are only 3 tx
 * data channels and management queue requires only minimum buffer.
 * i.e lower queues are used by driver and highest priority queue
 * from that is used for management message.
 */

static int emac_egress_buf_pool_size[] = {
	PRUETH_EMAC_BUF_POOL_SIZE_SR1, PRUETH_EMAC_BUF_POOL_SIZE_SR1,
	PRUETH_EMAC_BUF_POOL_SIZE_SR1, PRUETH_EMAC_BUF_POOL_MIN_SIZE_SR1,
	PRUETH_EMAC_BUF_POOL_MIN_SIZE_SR1, PRUETH_EMAC_BUF_POOL_MIN_SIZE_SR1,
	PRUETH_EMAC_BUF_POOL_MIN_SIZE_SR1, PRUETH_EMAC_BUF_POOL_MIN_SIZE_SR1};

void icssg_config_sr1(struct prueth *prueth, struct prueth_emac *emac,
		      int slice)
{
	void __iomem *va;
	struct icssg_config_sr1 *config;
	int i, index;

	va = prueth->shram.va + slice * ICSSG_CONFIG_OFFSET_SLICE1;
	config = &prueth->config[slice];
	memset(config, 0, sizeof(*config));
	config->addr_lo = cpu_to_le32(lower_32_bits(prueth->msmcram.pa));
	config->addr_hi = cpu_to_le32(upper_32_bits(prueth->msmcram.pa));
	config->num_tx_threads = 0;
	config->rx_flow_id = emac->rx_flow_id_base; /* flow id for host port */
	config->rx_mgr_flow_id = emac->rx_mgm_flow_id_base; /* for mgm ch */

	for (i = PRUETH_EMAC_BUF_POOL_START_SR1; i < PRUETH_NUM_BUF_POOLS_SR1;
	     i++) {
		index =  i - PRUETH_EMAC_BUF_POOL_START_SR1;
		config->tx_buf_sz[i] =
			cpu_to_le32(emac_egress_buf_pool_size[index]);
	}

	memcpy_toio(va, &prueth->config[slice], sizeof(prueth->config[slice]));
}

static void emac_r30_cmd_init(struct prueth_emac *emac)
{
	int i;
	struct icssg_r30_cmd *p;

	p = emac->dram.va + MGR_R30_CMD_OFFSET;

	for (i = 0; i < 4; i++)
		writel(EMAC_NONE, &p->cmd[i]);
}

static int emac_r30_is_done(struct prueth_emac *emac)
{
	const struct icssg_r30_cmd *p;
	int i;
	u32 cmd;

	p = emac->dram.va + MGR_R30_CMD_OFFSET;

	for (i = 0; i < 4; i++) {
		cmd = readl(&p->cmd[i]);
		if (cmd != EMAC_NONE)
			return 0;
	}

	return 1;
}

int icssg_config_sr2(struct prueth *prueth, struct prueth_emac *emac, int slice)
{
	void *config = emac->dram.va + ICSSG_CONFIG_OFFSET;
	u8 *cfg_byte_ptr = config;
	struct icssg_flow_cfg *flow_cfg;
	struct icssg_buffer_pool_cfg *bpool_cfg;
	struct icssg_rxq_ctx *rxq_ctx;
	int i;
	u32 addr;

	rxq_ctx = emac->dram.va + HOST_RX_Q_PRE_CONTEXT_OFFSET;
	memset_io(config, 0, TAS_GATE_MASK_LIST0);
	icssg_config_rgmii_init(prueth, slice);
	icssg_config_mii_init(prueth, slice);

	/* set GPI mode */
	pruss_cfg_gpimode(prueth->pruss, prueth->pru[slice],
			  PRUSS_GPI_MODE_MII);

	/* set C28 to 0x100 */
	pru_rproc_set_ctable(prueth->pru[slice], PRU_C28, 0x100 << 8);
	pru_rproc_set_ctable(prueth->rtu[slice], PRU_C28, 0x100 << 8);
	pru_rproc_set_ctable(prueth->txpru[slice], PRU_C28, 0x100 << 8);

	bpool_cfg = emac->dram.va + BUFFER_POOL_0_ADDR_OFFSET;

	flow_cfg = config + PSI_L_REGULAR_FLOW_ID_BASE_OFFSET;
	flow_cfg->rx_base_flow = cpu_to_le32(emac->rx_flow_id_base);
	flow_cfg->mgm_base_flow = 0;
	*(cfg_byte_ptr + SPL_PKT_DEFAULT_PRIORITY) = 0;
	*(cfg_byte_ptr + QUEUE_NUM_UNTAGGED) = 0x0;

	/* Layout to have 64KB aligned buffer pool
	 * |BPOOL0|BPOOL1|RX_CTX0|RX_CTX1|
	 */

	addr = lower_32_bits(prueth->msmcram.pa);
	if (slice)
		addr += PRUETH_NUM_BUF_POOLS_SR2 * PRUETH_EMAC_BUF_POOL_SIZE_SR2;

	if (addr % SZ_64K) {
		dev_warn(prueth->dev, "buffer pool needs to be 64KB aligned\n");
		return -EINVAL;
	}

	/* workaround for f/w bug. bpool 0 needs to be initilalized */
	bpool_cfg[0].addr = cpu_to_le32(addr);
	bpool_cfg[0].len = 0;

	for (i = PRUETH_EMAC_BUF_POOL_START_SR2;
	     i < (PRUETH_EMAC_BUF_POOL_START_SR2 + PRUETH_NUM_BUF_POOLS_SR2);
	     i++) {
		bpool_cfg[i].addr = cpu_to_le32(addr);
		bpool_cfg[i].len = cpu_to_le32(PRUETH_EMAC_BUF_POOL_SIZE_SR2);
		addr += PRUETH_EMAC_BUF_POOL_SIZE_SR2;
	}

	addr += PRUETH_NUM_BUF_POOLS_SR2 * PRUETH_EMAC_BUF_POOL_SIZE_SR2;
	if (slice)
		addr += PRUETH_EMAC_RX_CTX_BUF_SIZE;

	for (i = 0; i < 3; i++)
		rxq_ctx->start[i] = cpu_to_le32(addr);

	addr += PRUETH_EMAC_RX_CTX_BUF_SIZE;
	rxq_ctx->end = cpu_to_le32(addr);

	emac_r30_cmd_init(emac);
	return 0;
}

/* commands to program ICSSG R30 registers */
/* FIXME: fix hex magic numbers with macros */
static struct icssg_r30_cmd emac_r32_bitmask[] = {
	{{0xffff0004, 0xffff0100, 0xffff0100, EMAC_NONE}},	/* EMAC_PORT_DISABLE */
	{{0xfffb0040, 0xfeff0200, 0xfeff0200, EMAC_NONE}},	/* EMAC_PORT_BLOCK */
	{{0xffbb0000, 0xfcff0000, 0xdcff0000, EMAC_NONE}},	/* EMAC_PORT_FORWARD */
	{{0xffbb0000, 0xfcff0000, 0xfcff2000, EMAC_NONE}},	/* EMAC_PORT_FORWARD_WO_LEARNING */
	{{0xffff0001, EMAC_NONE,  EMAC_NONE, EMAC_NONE}},	/* ACCEPT ALL */
	{{0xfffe0002, EMAC_NONE,  EMAC_NONE, EMAC_NONE}},	/* ACCEPT TAGGED */
	{{0xfffc0000, EMAC_NONE,  EMAC_NONE, EMAC_NONE}},	/* ACCEPT UNTAGGED and PRIO */
	{{EMAC_NONE,  0xffff0020, EMAC_NONE, EMAC_NONE}},	/* TAS Trigger List change */
	{{EMAC_NONE,  0xdfff1000, EMAC_NONE, EMAC_NONE}},	/* TAS set state ENABLE*/
	{{EMAC_NONE,  0xefff2000, EMAC_NONE, EMAC_NONE}},	/* TAS set state RESET*/
	{{EMAC_NONE,  0xcfff0000, EMAC_NONE, EMAC_NONE}},	/* TAS set state DISABLE*/
	{{EMAC_NONE,  EMAC_NONE,  0xffff0400, EMAC_NONE}},	/* UC flooding ENABLE*/
	{{EMAC_NONE,  EMAC_NONE,  0xfbff0000, EMAC_NONE}},	/* UC flooding DISABLE*/
	{{EMAC_NONE,  EMAC_NONE,  0xffff0800, EMAC_NONE}},	/* MC flooding ENABLE*/
	{{EMAC_NONE,  EMAC_NONE,  0xf7ff0000, EMAC_NONE}},	/* MC flooding DISABLE*/
	{{EMAC_NONE,  0xffff4000, EMAC_NONE, EMAC_NONE}},	/* Preemption on Tx ENABLE*/
	{{EMAC_NONE,  0xbfff0000, EMAC_NONE, EMAC_NONE}}	/* Preemption on Tx DISABLE*/
};

int emac_set_port_state(struct prueth_emac *emac,
			enum icssg_port_state_cmd cmd)
{
	struct icssg_r30_cmd *p;
	int ret = -ETIMEDOUT;
	int timeout = 10;
	int i;

	p = emac->dram.va + MGR_R30_CMD_OFFSET;

	if (cmd >= ICSSG_EMAC_PORT_MAX_COMMANDS) {
		netdev_err(emac->ndev, "invalid port command\n");
		return -EINVAL;
	}

	/* only one command at a time allowed to firmware */
	mutex_lock(&emac->cmd_lock);

	for (i = 0; i < 4; i++)
		writel(emac_r32_bitmask[cmd].cmd[i], &p->cmd[i]);

	/* wait for done */
	while (timeout) {
		if (emac_r30_is_done(emac)) {
			ret = 0;
			break;
		}

		usleep_range(1000, 2000);
		timeout--;
	}

	if (ret == -ETIMEDOUT)
		netdev_err(emac->ndev, "timeout waiting for command done\n");

	mutex_unlock(&emac->cmd_lock);

	return ret;
}
