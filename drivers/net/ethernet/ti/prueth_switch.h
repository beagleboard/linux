/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __NET_TI_PRUETH_SWITCH_H
#define __NET_TI_PRUETH_SWITCH_H

#include "prueth.h"
#include "prueth_fdb_tbl.h"

/* PRU Ethernet Type - Ethernet functionality (protocol
 * implemented) provided by the PRU firmware being loaded.
 */
enum pruss_ethtype {
	PRUSS_ETHTYPE_EMAC = 0,
	PRUSS_ETHTYPE_SWITCH = 3,
};

#define PRUETH_IS_EMAC(p)	((p)->eth_type == PRUSS_ETHTYPE_EMAC)
#define PRUETH_IS_SWITCH(p)	((p)->eth_type == PRUSS_ETHTYPE_SWITCH)

struct prueth_col_rx_context_info {
	u16 buffer_offset;
	u16 buffer_offset2;
	u16 queue_desc_offset;
	u16 buffer_desc_offset;
	u16 buffer_desc_end;
} __packed;

struct prueth_col_tx_context_info {
	u16 buffer_offset;
	u16 buffer_offset2;
	u16 buffer_offset_end;
} __packed;

static inline enum prueth_port other_port_id(enum prueth_port port_id)
{
	enum prueth_port other_port_id =
		(port_id == PRUETH_PORT_MII0) ? PRUETH_PORT_MII1 :
						PRUETH_PORT_MII0;
	return other_port_id;
}

static inline
void prueth_sw_port_set_stp_state(struct prueth *prueth,
				  enum prueth_port port, u8 state)
{
	struct fdb_tbl *t = prueth->fdb_tbl;

	writeb(state, port - 1 ?
		&t->port2_stp_cfg->state : &t->port1_stp_cfg->state);
}

int prueth_sw_hostconfig(struct prueth *prueth);
int prueth_sw_emac_config(struct prueth_emac *emac);
void prueth_sw_fdb_tbl_init(struct prueth *prueth);
int prueth_sw_learn_fdb(struct prueth_emac *emac, u8 *src_mac);
int prueth_sw_boot_prus(struct prueth *prueth, struct net_device *ndev);
int prueth_sw_shutdown_prus(struct prueth_emac *emac, struct net_device *ndev);
int prueth_sw_register_notifiers(struct prueth *prueth);
void prueth_sw_unregister_notifiers(struct prueth *prueth);
bool prueth_sw_port_dev_check(const struct net_device *ndev);

extern const struct prueth_queue_info sw_queue_infos[][4];

#endif /* __NET_TI_PRUETH_SWITCH_H */
