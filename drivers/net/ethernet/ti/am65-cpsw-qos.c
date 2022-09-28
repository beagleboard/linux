// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet QoS submodule
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * quality of service module includes:
 * Enhanced Scheduler Traffic (EST - P802.1Qbv/D2.2)
 * Interspersed Express Traffic (IET - P802.3br/D2.0)
 */

#include <linux/bitfield.h>
#include <linux/pm_runtime.h>
#include <linux/time.h>
#include <linux/math64.h>

#include "am65-cpsw-nuss.h"
#include "am65-cpsw-qos.h"
#include "am65-cpts.h"

#define AM65_CPSW_REG_CTL			0x004
#define AM65_CPSW_REG_FREQ			0x05c
#define AM65_CPSW_PN_REG_CTL			0x004
#define AM65_CPSW_PN_REG_MAX_BLKS		0x008
#define AM65_CPSW_PN_REG_TX_PRI_MAP		0x018
#define AM65_CPSW_PN_REG_RX_PRI_MAP		0x020
#define AM65_CPSW_PN_REG_IET_CTRL		0x040
#define AM65_CPSW_PN_REG_IET_STATUS		0x044
#define AM65_CPSW_PN_REG_IET_VERIFY		0x048
#define AM65_CPSW_PN_REG_FIFO_STATUS		0x050
#define AM65_CPSW_PN_REG_EST_CTL		0x060
#define AM65_CPSW_PN_REG_PRI_CIR(pri)		(0x140 + 4 * (pri))
#define AM65_CPSW_PN_REG_PRI_EIR(pri)		(0x160 + 4 * (pri))

#define AM64_CPSW_PN_CUT_THRU			0x3C0
#define AM64_CPSW_PN_SPEED			0x3C4

/* AM65_CPSW_REG_CTL register fields */
#define AM65_CPSW_CTL_IET_EN			BIT(17)
#define AM65_CPSW_CTL_EST_EN			BIT(18)
#define AM64_CPSW_CTL_CUT_THRU_EN		BIT(19)

/* AM65_CPSW_PN_REG_CTL register fields */
#define AM65_CPSW_PN_CTL_IET_PORT_EN		BIT(16)
#define AM65_CPSW_PN_CTL_EST_PORT_EN		BIT(17)

/* AM65_CPSW_PN_REG_EST_CTL register fields */
#define AM65_CPSW_PN_EST_ONEBUF			BIT(0)
#define AM65_CPSW_PN_EST_BUFSEL			BIT(1)
#define AM65_CPSW_PN_EST_TS_EN			BIT(2)
#define AM65_CPSW_PN_EST_TS_FIRST		BIT(3)
#define AM65_CPSW_PN_EST_ONEPRI			BIT(4)
#define AM65_CPSW_PN_EST_TS_PRI_MSK		GENMASK(7, 5)

/* AM65_CPSW_PN_REG_IET_CTRL register fields */
#define AM65_CPSW_PN_IET_MAC_PENABLE		BIT(0)
#define AM65_CPSW_PN_IET_MAC_DISABLEVERIFY	BIT(2)
#define AM65_CPSW_PN_IET_MAC_LINKFAIL		BIT(3)
#define AM65_CPSW_PN_IET_MAC_MAC_ADDFRAGSIZE_MASK	GENMASK(10, 8)
#define AM65_CPSW_PN_IET_MAC_MAC_ADDFRAGSIZE_OFFSET	8
#define AM65_CPSW_PN_IET_PREMPT_MASK		GENMASK(23, 16)
#define AM65_CPSW_PN_IET_PREMPT_OFFSET		16

/* AM65_CPSW_PN_REG_IET_STATUS register fields */
#define AM65_CPSW_PN_MAC_VERIFIED		BIT(0)
#define AM65_CPSW_PN_MAC_VERIFY_FAIL		BIT(1)
#define AM65_CPSW_PN_MAC_RESPOND_ERR		BIT(2)
#define AM65_CPSW_PN_MAC_VERIFY_ERR		BIT(3)

/* AM65_CPSW_PN_REG_IET_VERIFY register fields */
/* 10 msec converted to NSEC */
#define AM65_CPSW_IET_VERIFY_CNT_MS		(10)
#define AM65_CPSW_IET_VERIFY_CNT_NS		(AM65_CPSW_IET_VERIFY_CNT_MS * \
						 NSEC_PER_MSEC)

/* AM65_CPSW_PN_REG_FIFO_STATUS register fields */
#define AM65_CPSW_PN_FST_TX_PRI_ACTIVE_MSK	GENMASK(7, 0)
#define AM65_CPSW_PN_FST_TX_E_MAC_ALLOW_MSK	GENMASK(15, 8)
#define AM65_CPSW_PN_FST_EST_CNT_ERR		BIT(16)
#define AM65_CPSW_PN_FST_EST_ADD_ERR		BIT(17)
#define AM65_CPSW_PN_FST_EST_BUFACT		BIT(18)

/* EST FETCH COMMAND RAM */
#define AM65_CPSW_FETCH_RAM_CMD_NUM		0x80
#define AM65_CPSW_FETCH_CNT_MSK			GENMASK(21, 8)
#define AM65_CPSW_FETCH_CNT_MAX			(AM65_CPSW_FETCH_CNT_MSK >> 8)
#define AM65_CPSW_FETCH_CNT_OFFSET		8
#define AM65_CPSW_FETCH_ALLOW_MSK		GENMASK(7, 0)
#define AM65_CPSW_FETCH_ALLOW_MAX		AM65_CPSW_FETCH_ALLOW_MSK

/* Cut-Thru AM64_CPSW_PN_CUT_THRU */
#define  AM64_PN_CUT_THRU_TX_PRI		GENMASK(7, 0)
#define  AM64_PN_CUT_THRU_RX_PRI		GENMASK(15, 8)

/* Cut-Thru AM64_CPSW_PN_SPEED */
#define  AM64_PN_SPEED_VAL			GENMASK(3, 0)
#define  AM64_PN_SPEED_AUTO_EN			BIT(8)
#define  AM64_PN_AUTO_SPEED			GENMASK(15, 12)

/* AM65_CPSW_PN_REG_MAX_BLKS fields for IET and No IET cases */
/* 7 blocks for pn_rx_max_blks, 13 for pn_tx_max_blks*/
#define AM65_CPSW_PN_TX_RX_MAX_BLKS_IET		0xD07
#define AM65_CPSW_PN_TX_RX_MAX_BLKS_DEFAULT	0x1004

enum timer_act {
	TACT_PROG,		/* need program timer */
	TACT_NEED_STOP,		/* need stop first */
	TACT_SKIP_PROG,		/* just buffer can be updated */
};

/* number of traffic classes (fifos) per port */
#define AM65_CPSW_PN_TC_NUM			8
#define AM65_CPSW_PN_TX_PRI_MAP_DEF			0x76543210

static int am65_cpsw_mqprio_setup(struct net_device *ndev, void *type_data);

/* Fetch command count it's number of bytes in Gigabit mode or nibbles in
 * 10/100Mb mode. So, having speed and time in ns, recalculate ns to number of
 * bytes/nibbles that can be sent while transmission on given speed.
 */
static int am65_est_cmd_ns_to_cnt(u64 ns, int link_speed)
{
	u64 temp;

	temp = ns * link_speed;
	if (link_speed < SPEED_1000)
		temp <<= 1;

	return DIV_ROUND_UP(temp, 8 * 1000);
}

/* IET */

static void am65_cpsw_iet_enable(struct am65_cpsw_common *common)
{
	int common_enable = 0;
	u32 val;
	int i;

	for (i = 0; i < common->port_num; i++)
		common_enable |= !!common->ports[i].qos.iet.mask;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (common_enable)
		val |= AM65_CPSW_CTL_IET_EN;
	else
		val &= ~AM65_CPSW_CTL_IET_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->iet_enabled = common_enable;
}

static void am65_cpsw_port_iet_enable(struct am65_cpsw_port *port,
				      u32 mask)
{
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (mask)
		val |= AM65_CPSW_PN_CTL_IET_PORT_EN;
	else
		val &= ~AM65_CPSW_PN_CTL_IET_PORT_EN;

	writel(val, port->port_base + AM65_CPSW_PN_REG_CTL);
	port->qos.iet.mask = mask;
}

static int am65_cpsw_iet_verify(struct am65_cpsw_port *port)
{
	int try;
	u32 val;

	netdev_info(port->ndev, "Starting IET/FPE MAC Verify\n");
	/* Set verify timeout depending on link speed. It is 10 msec
	 * in wireside clocks
	 */
	val = am65_est_cmd_ns_to_cnt(AM65_CPSW_IET_VERIFY_CNT_NS,
				     port->qos.link_speed);
	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_VERIFY);

	/* By experiment, keep this about 20 * 50 msec = 1000 msec.
	 * Usually succeeds in one try. But at times it takes more
	 * attempts especially at initial boot. Try for 20 times
	 * before give up
	 */
	try = 20;
	do {
		/* Enable IET Preemption for the port and
		 * reset LINKFAIL bit to start Verify.
		 */
		writel(AM65_CPSW_PN_IET_MAC_PENABLE,
		       port->port_base + AM65_CPSW_PN_REG_IET_CTRL);

		/* Takes 10 msec to complete this in h/w assuming other
		 * side is already ready. However since both side might
		 * take variable setup/config time, need to Wait for
		 * additional time. Chose 50 msec through trials
		 */
		msleep(50);

		val = readl(port->port_base + AM65_CPSW_PN_REG_IET_STATUS);
		if (val & AM65_CPSW_PN_MAC_VERIFIED)
			break;

		if (val & AM65_CPSW_PN_MAC_VERIFY_FAIL) {
			netdev_dbg(port->ndev,
				   "IET MAC verify failed, trying again");
			/* Reset the verify state machine by writing 1
			 * to LINKFAIL
			 */
			writel(AM65_CPSW_PN_IET_MAC_LINKFAIL,
			       port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
		}

		if (val & AM65_CPSW_PN_MAC_RESPOND_ERR) {
			netdev_err(port->ndev, "IET MAC respond error");
			return -ENODEV;
		}

		if (val & AM65_CPSW_PN_MAC_VERIFY_ERR) {
			netdev_err(port->ndev, "IET MAC verify error");
			return -ENODEV;
		}

	} while (try-- > 0);

	if (try <= 0) {
		netdev_err(port->ndev, "IET MAC Verify/Response timeout");
		return -ENODEV;
	}

	netdev_info(port->ndev, "IET/FPE MAC Verify Success\n");
	return 0;
}

static void am65_cpsw_iet_config_mac_preempt(struct am65_cpsw_port *port,
					     bool enable, bool force)
{
	struct am65_cpsw_iet *iet = &port->qos.iet;
	u32 val;

	/* Enable pre-emption queues and force mode if no mac verify */
	val = 0;
	if (enable) {
		if (!force) {
			/* AM65_CPSW_PN_IET_MAC_PENABLE already
			 * set as part of MAC Verify. So read
			 * modify
			 */
			val = readl(port->port_base +
				    AM65_CPSW_PN_REG_IET_CTRL);
		} else {
			val |= AM65_CPSW_PN_IET_MAC_PENABLE;
			val |= AM65_CPSW_PN_IET_MAC_DISABLEVERIFY;
		}
		val |= ((iet->fpe_mask_configured <<
			AM65_CPSW_PN_IET_PREMPT_OFFSET) &
			AM65_CPSW_PN_IET_PREMPT_MASK);
		val |= ((iet->addfragsize <<
			AM65_CPSW_PN_IET_MAC_MAC_ADDFRAGSIZE_OFFSET) &
			AM65_CPSW_PN_IET_MAC_MAC_ADDFRAGSIZE_MASK);
	}
	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
	iet->fpe_enabled = enable;
}

static void am65_cpsw_iet_set(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_iet *iet = &port->qos.iet;

	/* For IET, Change MAX_BLKS */
	writel(AM65_CPSW_PN_TX_RX_MAX_BLKS_IET,
	       port->port_base + AM65_CPSW_PN_REG_MAX_BLKS);

	am65_cpsw_port_iet_enable(port, iet->fpe_mask_configured);
	am65_cpsw_iet_enable(common);
}

static int am65_cpsw_iet_fpe_enable(struct am65_cpsw_port *port, bool verify)
{
	int ret;

	if (verify) {
		ret = am65_cpsw_iet_verify(port);
		if (ret)
			return ret;
	}

	am65_cpsw_iet_config_mac_preempt(port, true, !verify);

	return 0;
}

void am65_cpsw_qos_iet_init(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_iet *iet = &port->qos.iet;

	/* Enable IET FPE only if user has enabled priv flag for iet frame
	 * preemption.
	 */
	if (!iet->fpe_configured) {
		iet->fpe_mask_configured = 0;
		return;
	}
	/* Use highest priority queue as express queue and others
	 * as preemptible queues.
	 */
	iet->fpe_mask_configured = GENMASK(common->tx_ch_num - 2, 0);

	/* Init work queue for IET MAC verify process */
	iet->ndev = ndev;

	am65_cpsw_iet_set(ndev);
}

static void am65_cpsw_iet_fpe_disable(struct am65_cpsw_port *port)
{
	struct am65_cpsw_iet *iet = &port->qos.iet;

	am65_cpsw_iet_config_mac_preempt(port, false,
					 !iet->mac_verify_configured);
}

void am65_cpsw_qos_iet_cleanup(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);

	/* restore MAX_BLKS to default */
	writel(AM65_CPSW_PN_TX_RX_MAX_BLKS_DEFAULT,
	       port->port_base + AM65_CPSW_PN_REG_MAX_BLKS);

	am65_cpsw_iet_fpe_disable(port);
	am65_cpsw_port_iet_enable(port, 0);
	am65_cpsw_iet_enable(common);
}

static int am65_cpsw_port_est_enabled(struct am65_cpsw_port *port)
{
	return port->qos.est_oper || port->qos.est_admin;
}

static void am65_cpsw_est_enable(struct am65_cpsw_common *common, int enable)
{
	u32 val;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (enable)
		val |= AM65_CPSW_CTL_EST_EN;
	else
		val &= ~AM65_CPSW_CTL_EST_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->est_enabled = enable;
}

static void am65_cpsw_port_est_enable(struct am65_cpsw_port *port, int enable)
{
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (enable)
		val |= AM65_CPSW_PN_CTL_EST_PORT_EN;
	else
		val &= ~AM65_CPSW_PN_CTL_EST_PORT_EN;

	writel(val, port->port_base + AM65_CPSW_PN_REG_CTL);
}

/* target new EST RAM buffer, actual toggle happens after cycle completion */
static void am65_cpsw_port_est_assign_buf_num(struct net_device *ndev,
					      int buf_num)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	if (buf_num)
		val |= AM65_CPSW_PN_EST_BUFSEL;
	else
		val &= ~AM65_CPSW_PN_EST_BUFSEL;

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);
}

/* am65_cpsw_port_est_is_swapped() - Indicate if h/w is transitioned
 * admin -> oper or not
 *
 * Return true if already transitioned. i.e oper is equal to admin and buf
 * numbers match (est_oper->buf match with est_admin->buf).
 * false if before transition. i.e oper is not equal to admin, (i.e a
 * previous admin command is waiting to be transitioned to oper state
 * and est_oper->buf not match with est_oper->buf).
 */
static int am65_cpsw_port_est_is_swapped(struct net_device *ndev, int *oper,
					 int *admin)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_FIFO_STATUS);
	*oper = !!(val & AM65_CPSW_PN_FST_EST_BUFACT);

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	*admin = !!(val & AM65_CPSW_PN_EST_BUFSEL);

	return *admin == *oper;
}

/* am65_cpsw_port_est_get_free_buf_num() - Get free buffer number for
 * Admin to program the new schedule.
 *
 * Logic as follows:-
 * If oper is same as admin, return the other buffer (!oper) as the admin
 * buffer.  If oper is not the same, driver let the current oper to continue
 * as it is in the process of transitioning from admin -> oper. So keep the
 * oper by selecting the same oper buffer by writing to EST_BUFSEL bit in
 * EST CTL register. In the second iteration they will match and code returns.
 * The actual buffer to write command is selected later before it is ready
 * to update the schedule.
 */
static int am65_cpsw_port_est_get_free_buf_num(struct net_device *ndev)
{
	int oper, admin;
	int roll = 2;

	while (roll--) {
		if (am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
			return !oper;

		/* admin is not set, so hinder transition as it's not allowed
		 * to touch memory in-flight, by targeting same oper buf.
		 */
		am65_cpsw_port_est_assign_buf_num(ndev, oper);

		dev_info(&ndev->dev,
			 "Prev. EST admin cycle is in transit %d -> %d\n",
			 oper, admin);
	}

	return admin;
}

static void am65_cpsw_admin_to_oper(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (port->qos.est_oper)
		devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = port->qos.est_admin;
	port->qos.est_admin = NULL;
}

static void am65_cpsw_port_est_get_buf_num(struct net_device *ndev,
					   struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	val &= ~AM65_CPSW_PN_EST_ONEBUF;
	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);

	est_new->buf = am65_cpsw_port_est_get_free_buf_num(ndev);

	/* rolled buf num means changed buf while configuring */
	if (port->qos.est_oper && port->qos.est_admin &&
	    est_new->buf == port->qos.est_oper->buf)
		am65_cpsw_admin_to_oper(ndev);
}

static void am65_cpsw_est_set(struct net_device *ndev, int enable)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	int common_enable = 0;
	int i;

	am65_cpsw_port_est_enable(port, enable);

	for (i = 0; i < common->port_num; i++)
		common_enable |= am65_cpsw_port_est_enabled(&common->ports[i]);

	common_enable |= enable;
	am65_cpsw_est_enable(common, common_enable);
}

/* This update is supposed to be used in any routine before getting real state
 * of admin -> oper transition, particularly it's supposed to be used in some
 * generic routine for providing real state to Taprio Qdisc.
 */
static void am65_cpsw_est_update_state(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int oper, admin;

	if (!port->qos.est_admin)
		return;

	if (!am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
		return;

	am65_cpsw_admin_to_oper(ndev);
}

static void __iomem *am65_cpsw_est_set_sched_cmds(void __iomem *addr,
						  int fetch_cnt,
						  int fetch_allow)
{
	u32 prio_mask, cmd_fetch_cnt, cmd;

	do {
		if (fetch_cnt > AM65_CPSW_FETCH_CNT_MAX) {
			fetch_cnt -= AM65_CPSW_FETCH_CNT_MAX;
			cmd_fetch_cnt = AM65_CPSW_FETCH_CNT_MAX;
		} else {
			cmd_fetch_cnt = fetch_cnt;
			/* fetch count can't be less than 16? */
			if (cmd_fetch_cnt && cmd_fetch_cnt < 16)
				cmd_fetch_cnt = 16;

			fetch_cnt = 0;
		}

		prio_mask = fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK;
		cmd = (cmd_fetch_cnt << AM65_CPSW_FETCH_CNT_OFFSET) | prio_mask;

		writel(cmd, addr);
		addr += 4;
	} while (fetch_cnt);

	return addr;
}

static int am65_cpsw_est_calc_cmd_num(struct net_device *ndev,
				      struct tc_taprio_qopt_offload *taprio,
				      int link_speed)
{
	int i, cmd_cnt, cmd_sum = 0;
	u32 fetch_cnt;

	for (i = 0; i < taprio->num_entries; i++) {
		if (taprio->entries[i].command != TC_TAPRIO_CMD_SET_GATES) {
			dev_err(&ndev->dev, "Only SET command is supported");
			return -EINVAL;
		}

		fetch_cnt = am65_est_cmd_ns_to_cnt(taprio->entries[i].interval,
						   link_speed);

		cmd_cnt = DIV_ROUND_UP(fetch_cnt, AM65_CPSW_FETCH_CNT_MAX);
		if (!cmd_cnt)
			cmd_cnt++;

		cmd_sum += cmd_cnt;

		if (!fetch_cnt)
			break;
	}

	return cmd_sum;
}

static int am65_cpsw_est_check_scheds(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int cmd_num;

	cmd_num = am65_cpsw_est_calc_cmd_num(ndev, &est_new->taprio,
					     port->qos.link_speed);
	if (cmd_num < 0)
		return cmd_num;

	if (cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM / 2) {
		dev_err(&ndev->dev, "No fetch RAM");
		return -ENOMEM;
	}

	return 0;
}

static void am65_cpsw_est_set_sched_list(struct net_device *ndev,
					 struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 fetch_cnt, fetch_allow, all_fetch_allow = 0;
	void __iomem *ram_addr, *max_ram_addr;
	struct tc_taprio_sched_entry *entry;
	int i, ram_size;

	ram_addr = port->fetch_ram_base;
	ram_size = AM65_CPSW_FETCH_RAM_CMD_NUM * 2;
	ram_addr += est_new->buf * ram_size;

	max_ram_addr = ram_size + ram_addr;
	for (i = 0; i < est_new->taprio.num_entries; i++) {
		entry = &est_new->taprio.entries[i];

		fetch_cnt = am65_est_cmd_ns_to_cnt(entry->interval,
						   port->qos.link_speed);
		fetch_allow = entry->gate_mask;
		if (fetch_allow > AM65_CPSW_FETCH_ALLOW_MAX)
			dev_dbg(&ndev->dev, "fetch_allow > 8 bits: %d\n",
				fetch_allow);

		ram_addr = am65_cpsw_est_set_sched_cmds(ram_addr, fetch_cnt,
							fetch_allow);

		if (!fetch_cnt && i < est_new->taprio.num_entries - 1) {
			dev_info(&ndev->dev,
				 "next scheds after %d have no impact", i + 1);
			break;
		}

		all_fetch_allow |= fetch_allow;
	}

	/* end cmd, enabling non-timed queues for potential over cycle time */
	if (ram_addr < max_ram_addr)
		writel(~all_fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK, ram_addr);
}

/*
 * Enable ESTf periodic output, set cycle start time and interval.
 */
static int am65_cpsw_timer_set(struct net_device *ndev,
			       struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpts *cpts = common->cpts;
	struct am65_cpts_estf_cfg cfg;

	cfg.ns_period = est_new->taprio.cycle_time;
	cfg.ns_start = est_new->taprio.base_time;

	return am65_cpts_estf_enable(cpts, port->port_id - 1, &cfg);
}

static void am65_cpsw_timer_stop(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;

	am65_cpts_estf_disable(cpts, port->port_id - 1);
}

static enum timer_act am65_cpsw_timer_act(struct net_device *ndev,
					  struct am65_cpsw_est *est_new)
{
	struct tc_taprio_qopt_offload *taprio_oper, *taprio_new;
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;
	u64 cur_time;
	s64 diff;

	if (!port->qos.est_oper)
		return TACT_PROG;

	taprio_new = &est_new->taprio;
	taprio_oper = &port->qos.est_oper->taprio;

	if (taprio_new->cycle_time != taprio_oper->cycle_time)
		return TACT_NEED_STOP;

	/* in order to avoid timer reset get base_time form oper taprio */
	if (!taprio_new->base_time && taprio_oper)
		taprio_new->base_time = taprio_oper->base_time;

	if (taprio_new->base_time == taprio_oper->base_time)
		return TACT_SKIP_PROG;

	/* base times are cycle synchronized */
	diff = taprio_new->base_time - taprio_oper->base_time;
	diff = diff < 0 ? -diff : diff;
	if (diff % taprio_new->cycle_time)
		return TACT_NEED_STOP;

	cur_time = am65_cpts_ns_gettime(cpts);
	if (taprio_new->base_time <= cur_time + taprio_new->cycle_time)
		return TACT_SKIP_PROG;

	/* TODO: Admin schedule at future time is not currently supported */
	return TACT_NEED_STOP;
}

static void am65_cpsw_stop_est(struct net_device *ndev)
{
	am65_cpsw_est_set(ndev, 0);
	am65_cpsw_timer_stop(ndev);
}

static void am65_cpsw_purge_est(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	am65_cpsw_stop_est(ndev);

	if (port->qos.est_admin)
		devm_kfree(&ndev->dev, port->qos.est_admin);

	if (port->qos.est_oper)
		devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = NULL;
	port->qos.est_admin = NULL;
}

static int am65_cpsw_configure_taprio(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct am65_cpts *cpts = common->cpts;
	int ret = 0, tact = TACT_PROG;
	u64 cur_time, n;

	am65_cpsw_est_update_state(ndev);

	if (!est_new->taprio.enable) {
		am65_cpsw_stop_est(ndev);
		return ret;
	}

	ret = am65_cpsw_est_check_scheds(ndev, est_new);
	if (ret < 0)
		return ret;

	tact = am65_cpsw_timer_act(ndev, est_new);
	if (tact == TACT_NEED_STOP) {
		dev_err(&ndev->dev,
			"Can't toggle estf timer, stop taprio first");
		return -EINVAL;
	}

	if (tact == TACT_PROG)
		am65_cpsw_timer_stop(ndev);

	am65_cpsw_port_est_get_buf_num(ndev, est_new);
	am65_cpsw_est_set_sched_list(ndev, est_new);
	am65_cpsw_port_est_assign_buf_num(ndev, est_new->buf);

	/* If the base-time is in the past, start schedule from the time:
	 * base_time + (N*cycle_time)
	 * where N is the smallest possible integer such that the above
	 * time is in the future.
	 */
	cur_time = am65_cpts_ns_gettime(cpts);
	if (est_new->taprio.base_time < cur_time) {
		n = div64_u64(cur_time - est_new->taprio.base_time, est_new->taprio.cycle_time);
		est_new->taprio.base_time += (n + 1) * est_new->taprio.cycle_time;
	}

	am65_cpsw_est_set(ndev, est_new->taprio.enable);

	if (tact == TACT_PROG) {
		ret = am65_cpsw_timer_set(ndev, est_new);
		if (ret) {
			dev_err(&ndev->dev, "Failed to set cycle time");
			return ret;
		}
	}

	return ret;
}

static void am65_cpsw_cp_taprio(struct tc_taprio_qopt_offload *from,
				struct tc_taprio_qopt_offload *to)
{
	int i;

	*to = *from;
	for (i = 0; i < from->num_entries; i++)
		to->entries[i] = from->entries[i];
}

static int am65_cpsw_set_taprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct tc_taprio_qopt_offload *taprio = type_data;
	struct am65_cpsw_est *est_new;
	int ret = 0;

	if (taprio->cycle_time_extension) {
		dev_err(&ndev->dev, "Failed to set cycle time extension");
		return -EOPNOTSUPP;
	}

	est_new = devm_kzalloc(&ndev->dev,
			       struct_size(est_new, taprio.entries, taprio->num_entries),
			       GFP_KERNEL);
	if (!est_new)
		return -ENOMEM;

	am65_cpsw_cp_taprio(taprio, &est_new->taprio);
	ret = am65_cpsw_configure_taprio(ndev, est_new);
	if (!ret) {
		if (taprio->enable) {
			if (port->qos.est_admin)
				devm_kfree(&ndev->dev, port->qos.est_admin);

			port->qos.est_admin = est_new;
		} else {
			devm_kfree(&ndev->dev, est_new);
			am65_cpsw_purge_est(ndev);
		}
	} else {
		devm_kfree(&ndev->dev, est_new);
	}

	return ret;
}

static void am65_cpsw_est_link_up(struct net_device *ndev, int link_speed)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	ktime_t cur_time;
	s64 delta;

	port->qos.link_speed = link_speed;
	if (!am65_cpsw_port_est_enabled(port))
		return;

	if (port->qos.link_down_time) {
		cur_time = ktime_get();
		delta = ktime_us_delta(cur_time, port->qos.link_down_time);
		if (delta > USEC_PER_SEC) {
			dev_err(&ndev->dev,
				"Link has been lost too long, stopping TAS");
			goto purge_est;
		}
	}

	return;

purge_est:
	am65_cpsw_purge_est(ndev);
}

static int am65_cpsw_setup_taprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;

	if (!IS_ENABLED(CONFIG_TI_AM65_CPSW_TAS))
		return -ENODEV;

	if (!netif_running(ndev)) {
		dev_err(&ndev->dev, "interface is down, link speed unknown\n");
		return -ENETDOWN;
	}

	if (common->pf_p0_rx_ptype_rrobin) {
		dev_err(&ndev->dev,
			"p0-rx-ptype-rrobin flag conflicts with taprio qdisc\n");
		return -EINVAL;
	}

	if (port->qos.link_speed == SPEED_UNKNOWN)
		return -ENOLINK;

	return am65_cpsw_set_taprio(ndev, type_data);
}

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data)
{
	switch (type) {
	case TC_SETUP_QDISC_TAPRIO:
		return am65_cpsw_setup_taprio(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return am65_cpsw_mqprio_setup(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static void am65_cpsw_iet_link_up(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_iet *iet = &port->qos.iet;

	if (!iet->fpe_configured)
		return;

	/* Schedule MAC Verify and enable IET FPE if configured */
	if (iet->mac_verify_configured) {
		am65_cpsw_iet_fpe_enable(port, true);
	} else {
		/* Force IET FPE here */
		netdev_info(ndev, "IET Enable Force mode\n");
		am65_cpsw_iet_fpe_enable(port, false);
	}
}

static void am65_cpsw_cut_thru_link_up(struct am65_cpsw_port *port);
static void am65_cpsw_tx_pn_shaper_link_up(struct am65_cpsw_port *port);

void am65_cpsw_qos_link_up(struct net_device *ndev, int link_speed, int duplex)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	port->qos.link_speed = link_speed;
	port->qos.duplex = duplex;
	am65_cpsw_iet_link_up(ndev);
	am65_cpsw_cut_thru_link_up(port);
	am65_cpsw_tx_pn_shaper_link_up(port);

	if (!IS_ENABLED(CONFIG_TI_AM65_CPSW_TAS))
		return;
	am65_cpsw_est_link_up(ndev, link_speed);
	port->qos.link_down_time = 0;
}

void am65_cpsw_qos_link_down(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	am65_cpsw_iet_fpe_disable(port);

	if (!IS_ENABLED(CONFIG_TI_AM65_CPSW_TAS))
		return;

	if (!port->qos.link_down_time)
		port->qos.link_down_time = ktime_get();

	port->qos.link_speed = SPEED_UNKNOWN;
}

static void am65_cpsw_cut_thru_dump(struct am65_cpsw_port *port)
{
	struct am65_cpsw_common *common = port->common;
	u32 contro, cut_thru, speed;

	contro = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	cut_thru = readl(port->port_base + AM64_CPSW_PN_CUT_THRU);
	speed = readl(port->port_base + AM64_CPSW_PN_SPEED);
	dev_dbg(common->dev, "Port%u: cut_thru dump control:%08x cut_thru:%08x hwspeed:%08x\n",
		port->port_id, contro, cut_thru, speed);
}

static void am65_cpsw_cut_thru_enable(struct am65_cpsw_common *common)
{
	u32 val;

	if (common->cut_thru_enabled) {
		common->cut_thru_enabled++;
		return;
	}

	/* Populate CPSW VBUS freq for auto speed detection */
	writel(common->bus_freq / 1000000,
	       common->cpsw_base + AM65_CPSW_REG_FREQ);

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	val |= AM64_CPSW_CTL_CUT_THRU_EN;
	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->cut_thru_enabled++;
}

void am65_cpsw_qos_cut_thru_init(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;

	/* Enable cut_thr only if user has enabled priv flag */
	if (!cut_thru->enable)
		return;

	if (common->is_emac_mode) {
		cut_thru->enable = false;
		dev_info(common->dev, "Disable cut-thru, need Switch mode\n");
		return;
	}

	am65_cpsw_cut_thru_enable(common);

	/* en auto speed */
	writel(AM64_PN_SPEED_AUTO_EN, port->port_base + AM64_CPSW_PN_SPEED);
	dev_info(common->dev, "Init cut_thru\n");
	am65_cpsw_cut_thru_dump(port);
}

static void am65_cpsw_cut_thru_disable(struct am65_cpsw_common *common)
{
	u32 val;

	if (--common->cut_thru_enabled)
		return;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	val &= ~AM64_CPSW_CTL_CUT_THRU_EN;
	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
}

void am65_cpsw_qos_cut_thru_cleanup(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;

	if (!cut_thru->enable)
		return;

	writel(0, port->port_base + AM64_CPSW_PN_CUT_THRU);
	writel(0, port->port_base + AM64_CPSW_PN_SPEED);

	am65_cpsw_cut_thru_disable(common);
	dev_info(common->dev, "Cleanup cut_thru\n");
	am65_cpsw_cut_thru_dump(port);
}

static u32 am65_cpsw_cut_thru_speed2hw(int link_speed)
{
	switch (link_speed) {
	case SPEED_10:
		return 1;
	case SPEED_100:
		return 2;
	case SPEED_1000:
		return 3;
	default:
		return 0;
	}
}

static void am65_cpsw_cut_thru_link_up(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;
	u32 val, speed;

	if (!cut_thru->enable)
		return;

	writel(AM64_PN_SPEED_AUTO_EN, port->port_base + AM64_CPSW_PN_SPEED);
	/* barrier */
	readl(port->port_base + AM64_CPSW_PN_SPEED);
	/* HW need 15us in 10/100 mode and 3us in 1G mode auto speed detection
	 * add delay with some margin
	 */
	usleep_range(40, 50);
	val = readl(port->port_base + AM64_CPSW_PN_SPEED);
	speed = FIELD_GET(AM64_PN_AUTO_SPEED, val);
	if (!speed) {
		dev_warn(common->dev,
			 "Port%u: cut_thru no speed auto detected switch to manual\n",
			 port->port_id);
		speed = am65_cpsw_cut_thru_speed2hw(port->qos.link_speed);
		if (!speed) {
			dev_err(common->dev,
				"Port%u: cut_thru speed configuration failed\n",
				port->port_id);
			return;
		}
		val = FIELD_PREP(AM64_PN_SPEED_VAL, speed);
		writel(val, port->port_base + AM64_CPSW_PN_SPEED);
	}

	val = FIELD_PREP(AM64_PN_CUT_THRU_TX_PRI, cut_thru->tx_pri_mask) |
	      FIELD_PREP(AM64_PN_CUT_THRU_RX_PRI, cut_thru->rx_pri_mask);

	if (port->qos.duplex) {
		writel(val, port->port_base + AM64_CPSW_PN_CUT_THRU);
		dev_info(common->dev, "Port%u: Enable cut_thru rx:%08x tx:%08x hwspeed:%u (%08x)\n",
			 port->port_id,
			 cut_thru->rx_pri_mask, cut_thru->tx_pri_mask,
			 speed, val);
	} else {
		writel(0, port->port_base + AM64_CPSW_PN_CUT_THRU);
		dev_info(common->dev, "Port%u: Disable cut_thru duplex=%d\n",
			 port->port_id, port->qos.duplex);
	}
	am65_cpsw_cut_thru_dump(port);
}

static u32
am65_cpsw_qos_tx_rate_calc(u32 rate_mbps, unsigned long bus_freq)
{
	u32 ir;

	bus_freq /= 1000000;
	ir = DIV_ROUND_UP(((u64)rate_mbps * 32768),  bus_freq);
	return ir;
}

static void
am65_cpsw_qos_tx_p0_rate_apply(struct am65_cpsw_common *common,
			       int tx_ch, u32 rate_mbps)
{
	struct am65_cpsw_host *host = am65_common_get_host(common);
	u32 ch_cir;
	int i;

	ch_cir = am65_cpsw_qos_tx_rate_calc(rate_mbps, common->bus_freq);
	writel(ch_cir, host->port_base + AM65_CPSW_PN_REG_PRI_CIR(tx_ch));

	/* update rates for every port tx queues */
	for (i = 0; i < common->port_num; i++) {
		struct net_device *ndev = common->ports[i].ndev;

		if (!ndev)
			continue;
		netdev_get_tx_queue(ndev, tx_ch)->tx_maxrate = rate_mbps;
	}
}

int am65_cpsw_qos_ndo_tx_p0_set_maxrate(struct net_device *ndev,
					int queue, u32 rate_mbps)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_tx_chn *tx_chn;
	u32 ch_rate, tx_ch_rate_msk_new;
	u32 ch_msk = 0;
	int ret;

	dev_dbg(common->dev, "apply TX%d rate limiting %uMbps tx_rate_msk%x\n",
		queue, rate_mbps, common->tx_ch_rate_msk);

	if (common->pf_p0_rx_ptype_rrobin) {
		dev_err(common->dev, "TX Rate Limiting failed - rrobin mode\n");
		return -EINVAL;
	}

	ch_rate = netdev_get_tx_queue(ndev, queue)->tx_maxrate;
	if (ch_rate == rate_mbps)
		return 0;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}
	ret = 0;

	tx_ch_rate_msk_new = common->tx_ch_rate_msk;
	if (rate_mbps && !(tx_ch_rate_msk_new & BIT(queue))) {
		tx_ch_rate_msk_new |= BIT(queue);
		ch_msk = GENMASK(common->tx_ch_num - 1, queue);
		ch_msk = tx_ch_rate_msk_new ^ ch_msk;
	} else if (!rate_mbps) {
		tx_ch_rate_msk_new &= ~BIT(queue);
		ch_msk = queue ? GENMASK(queue - 1, 0) : 0;
		ch_msk = tx_ch_rate_msk_new & ch_msk;
	}

	if (ch_msk) {
		dev_err(common->dev, "TX rate limiting has to be enabled sequentially hi->lo tx_rate_msk:%x tx_rate_msk_new:%x\n",
			common->tx_ch_rate_msk, tx_ch_rate_msk_new);
		ret = -EINVAL;
		goto exit_put;
	}

	tx_chn = &common->tx_chns[queue];
	tx_chn->rate_mbps = rate_mbps;
	common->tx_ch_rate_msk = tx_ch_rate_msk_new;

	if (!common->usage_count)
		/* will be applied on next netif up */
		goto exit_put;

	am65_cpsw_qos_tx_p0_rate_apply(common, queue, rate_mbps);

exit_put:
	pm_runtime_put(common->dev);
	return ret;
}

void am65_cpsw_qos_tx_p0_rate_init(struct am65_cpsw_common *common)
{
	struct am65_cpsw_host *host = am65_common_get_host(common);
	int tx_ch;

	for (tx_ch = 0; tx_ch < common->tx_ch_num; tx_ch++) {
		struct am65_cpsw_tx_chn *tx_chn = &common->tx_chns[tx_ch];
		u32 ch_cir;

		if (!tx_chn->rate_mbps)
			continue;

		ch_cir = am65_cpsw_qos_tx_rate_calc(tx_chn->rate_mbps,
						    common->bus_freq);
		writel(ch_cir,
		       host->port_base + AM65_CPSW_PN_REG_PRI_CIR(tx_ch));
	}
}

static void am65_cpsw_tx_pn_shaper_apply(struct am65_cpsw_port *port)
{
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct am65_cpsw_common *common = port->common;
	struct tc_mqprio_qopt_offload *mqprio;
	bool shaper_en;
	u32 rate_mbps;
	int i;

	mqprio = &p_mqprio->mqprio_hw;
	shaper_en = p_mqprio->shaper_en && !p_mqprio->shaper_susp;

	for (i = 0; i < mqprio->qopt.num_tc; i++) {
		rate_mbps = 0;
		if (shaper_en) {
			rate_mbps = mqprio->min_rate[i] * 8 / 1000000;
			rate_mbps = am65_cpsw_qos_tx_rate_calc(rate_mbps,
							       common->bus_freq);
		}

		writel(rate_mbps,
		       port->port_base + AM65_CPSW_PN_REG_PRI_CIR(i));
	}

	for (i = 0; i < mqprio->qopt.num_tc; i++) {
		rate_mbps = 0;
		if (shaper_en && mqprio->max_rate[i]) {
			rate_mbps = mqprio->max_rate[i] - mqprio->min_rate[i];
			rate_mbps = rate_mbps * 8 / 1000000;
			rate_mbps = am65_cpsw_qos_tx_rate_calc(rate_mbps,
							       common->bus_freq);
		}

		writel(rate_mbps,
		       port->port_base + AM65_CPSW_PN_REG_PRI_EIR(i));
	}
}

static void am65_cpsw_tx_pn_shaper_link_up(struct am65_cpsw_port *port)
{
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct am65_cpsw_common *common = port->common;
	bool shaper_susp = false;

	if (!p_mqprio->enable || !p_mqprio->shaper_en)
		return;

	if (p_mqprio->max_rate_total > port->qos.link_speed)
		shaper_susp = true;

	if (p_mqprio->shaper_susp == shaper_susp)
		return;

	if (shaper_susp)
		dev_info(common->dev,
			 "Port%u: total shaper tx rate > link speed - suspend shaper\n",
			 port->port_id);
	else
		dev_info(common->dev,
			 "Port%u: link recover - resume shaper\n",
			 port->port_id);

	p_mqprio->shaper_susp = shaper_susp;
	am65_cpsw_tx_pn_shaper_apply(port);
}

void am65_cpsw_qos_mqprio_init(struct am65_cpsw_port *port)
{
	struct am65_cpsw_host *host = am65_common_get_host(port->common);
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct tc_mqprio_qopt_offload *mqprio = &p_mqprio->mqprio_hw;
	int i, fifo, rx_prio_map;

	rx_prio_map = readl(host->port_base + AM65_CPSW_PN_REG_RX_PRI_MAP);

	if (p_mqprio->enable) {
		for (i = 0; i < AM65_CPSW_PN_TC_NUM; i++) {
			fifo = mqprio->qopt.prio_tc_map[i];
			p_mqprio->tx_prio_map |= fifo << (4 * i);
		}

		netdev_set_num_tc(port->ndev, mqprio->qopt.num_tc);
		for (i = 0; i < mqprio->qopt.num_tc; i++) {
			netdev_set_tc_queue(port->ndev, i,
					    mqprio->qopt.count[i],
					    mqprio->qopt.offset[i]);
			if (!i) {
				p_mqprio->tc0_q = mqprio->qopt.offset[i];
				rx_prio_map &= ~(0x7 << (4 * p_mqprio->tc0_q));
			}
		}
	} else {
		/* restore default configuration */
		netdev_reset_tc(port->ndev);
		p_mqprio->tx_prio_map = AM65_CPSW_PN_TX_PRI_MAP_DEF;
		rx_prio_map |= p_mqprio->tc0_q << (4 * p_mqprio->tc0_q);
		p_mqprio->tc0_q = 0;
	}

	writel(p_mqprio->tx_prio_map,
	       port->port_base + AM65_CPSW_PN_REG_TX_PRI_MAP);
	writel(rx_prio_map,
	       host->port_base + AM65_CPSW_PN_REG_RX_PRI_MAP);

	am65_cpsw_tx_pn_shaper_apply(port);
}

static int am65_cpsw_mqprio_verify(struct am65_cpsw_port *port,
				   struct tc_mqprio_qopt_offload *mqprio)
{
	int i;

	for (i = 0; i < mqprio->qopt.num_tc; i++) {
		unsigned int last = mqprio->qopt.offset[i] +
				    mqprio->qopt.count[i];

		if (mqprio->qopt.offset[i] >= port->ndev->real_num_tx_queues ||
		    !mqprio->qopt.count[i] ||
		    last >  port->ndev->real_num_tx_queues)
			return -EINVAL;
	}

	return 0;
}

static int am65_cpsw_mqprio_verify_shaper(struct am65_cpsw_port *port,
					  struct tc_mqprio_qopt_offload *mqprio,
					  u64 *max_rate)
{
	struct am65_cpsw_common *common = port->common;
	bool has_min_rate, has_max_rate;
	u64 min_rate_total = 0, max_rate_total = 0;
	u32 min_rate_msk = 0, max_rate_msk = 0;
	int num_tc, i;

	has_min_rate = !!(mqprio->flags & TC_MQPRIO_F_MIN_RATE);
	has_max_rate = !!(mqprio->flags & TC_MQPRIO_F_MAX_RATE);

	if (!has_min_rate && has_max_rate)
		return -EOPNOTSUPP;

	if (!has_min_rate)
		return 0;

	num_tc = mqprio->qopt.num_tc;

	for (i = num_tc - 1; i >= 0; i--) {
		u32 ch_msk;

		if (mqprio->min_rate[i])
			min_rate_msk |= BIT(i);
		min_rate_total +=  mqprio->min_rate[i];

		if (has_max_rate) {
			if (mqprio->max_rate[i])
				max_rate_msk |= BIT(i);
			max_rate_total +=  mqprio->max_rate[i];

			if (!mqprio->min_rate[i] && mqprio->max_rate[i]) {
				dev_err(common->dev, "TX tc%d rate max>0 but min=0\n",
					i);
				return -EINVAL;
			}

			if (mqprio->max_rate[i] &&
			    mqprio->max_rate[i] < mqprio->min_rate[i]) {
				dev_err(common->dev, "TX tc%d rate min(%llu)>max(%llu)\n",
					i, mqprio->min_rate[i],
					mqprio->max_rate[i]);
				return -EINVAL;
			}
		}

		ch_msk = GENMASK(num_tc - 1, i);
		if ((min_rate_msk & BIT(i)) && (min_rate_msk ^ ch_msk)) {
			dev_err(common->dev, "TX Min rate limiting has to be enabled sequentially hi->lo tx_rate_msk%x\n",
				min_rate_msk);
			return -EINVAL;
		}

		if ((max_rate_msk & BIT(i)) && (max_rate_msk ^ ch_msk)) {
			dev_err(common->dev, "TX max rate limiting has to be enabled sequentially hi->lo tx_rate_msk%x\n",
				max_rate_msk);
			return -EINVAL;
		}
	}
	min_rate_total *= 8;
	min_rate_total /= 1000 * 1000;
	max_rate_total *= 8;
	max_rate_total /= 1000 * 1000;

	if (port->qos.link_speed != SPEED_UNKNOWN) {
		if (min_rate_total > port->qos.link_speed) {
			dev_err(common->dev, "TX rate min exceed %llu link speed %d\n",
				min_rate_total, port->qos.link_speed);
			return -EINVAL;
		}

		if (max_rate_total > port->qos.link_speed) {
			dev_err(common->dev, "TX rate max exceed %llu link speed %d\n",
				max_rate_total, port->qos.link_speed);
			return -EINVAL;
		}
	}

	*max_rate = max_t(u64, min_rate_total, max_rate_total);

	return 0;
}

static int am65_cpsw_mqprio_setup(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct tc_mqprio_qopt_offload *mqprio = type_data;
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	bool has_min_rate;
	int num_tc, ret;
	u64 max_rate;

	if (!mqprio->qopt.hw)
		goto skip_check;

	if (mqprio->mode != TC_MQPRIO_MODE_CHANNEL)
		return -EOPNOTSUPP;

	num_tc = mqprio->qopt.num_tc;
	if (num_tc > AM65_CPSW_PN_TC_NUM)
		return -ERANGE;

	if ((mqprio->flags & TC_MQPRIO_F_SHAPER) &&
	    mqprio->shaper != TC_MQPRIO_SHAPER_BW_RATE)
		return -EOPNOTSUPP;

	ret = am65_cpsw_mqprio_verify(port, mqprio);
	if (ret)
		return ret;

	ret = am65_cpsw_mqprio_verify_shaper(port, mqprio, &max_rate);
	if (ret)
		return ret;

skip_check:
	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	if (mqprio->qopt.hw) {
		memcpy(&p_mqprio->mqprio_hw, mqprio, sizeof(*mqprio));
		has_min_rate = !!(mqprio->flags & TC_MQPRIO_F_MIN_RATE);
		p_mqprio->enable = 1;
		p_mqprio->shaper_en = has_min_rate;
		p_mqprio->shaper_susp = !has_min_rate;
		p_mqprio->max_rate_total = max_rate;
		p_mqprio->tx_prio_map = 0;
	} else {
		unsigned int tc0_q = p_mqprio->tc0_q;

		memset(p_mqprio, 0, sizeof(*p_mqprio));
		p_mqprio->mqprio_hw.qopt.num_tc = AM65_CPSW_PN_TC_NUM;
		p_mqprio->tc0_q = tc0_q;
	}

	if (!netif_running(ndev))
		goto exit_put;

	am65_cpsw_qos_mqprio_init(port);

exit_put:
	pm_runtime_put(common->dev);
	return 0;
}
