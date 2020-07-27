/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSS Enhanced Capture (eCAP) Driver
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include "prueth.h"

#ifndef __NET_TI_PRUETH_ECAP_H
#define __NET_TI_PRUETH_ECAP_H

/* SRAM offsets for firmware pacing timer configuration */
struct rx_int_pacing_offsets {
	u32 rx_int_pacing_ctrl;
	u32 rx_int_pacing_exp;
	u32 rx_int_pacing_prev;
};

struct prueth_ecap {
	struct device *dev;
	void __iomem *base;
	struct device_node *client_np;
	struct rx_int_pacing_offsets int_pacing_offsets[PRUETH_NUM_MACS];
	u32 timeout[PRUETH_NUM_MACS];
	void (*init)(struct prueth_emac *emac);
	int (*get_coalesce)(struct prueth_emac *emac,
			    u32 *use_adaptive_rx_coalesce,
			    u32 *rx_coalesce_usecs);
	int (*set_coalesce)(struct prueth_emac *emac,
			    u32 use_adaptive_rx_coalesce,
			    u32 rx_coalesce_usecs);
};

#if IS_ENABLED(CONFIG_TI_PRUETH_ECAP)
struct prueth_ecap *prueth_ecap_get(struct device_node *np);
void prueth_ecap_put(struct prueth_ecap *ecap);
#else
static inline struct prueth_ecap *prueth_ecap_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}

static inline void prueth_ecap_put(struct prueth_ecap *ecap)
{};
#endif

#endif /* __NET_TI_PRUETH_ECAP_H */
