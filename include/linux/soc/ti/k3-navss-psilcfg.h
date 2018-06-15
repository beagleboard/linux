/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#ifndef __SOC_TI_K3_NAVSS_PSILCFG_H__
#define __SOC_TI_K3_NAVSS_PSILCFG_H__

struct k3_nav_psil_entry;

struct device_node *of_k3_nav_psil_get_by_phandle(struct device_node *np,
						  const char *property);
struct k3_nav_psil_entry *k3_nav_psil_request_link(
					struct device_node *psilcfg_node,
					u32 src_thread, u32 dst_thread);
int k3_nav_psil_release_link(struct k3_nav_psil_entry *link);

#endif /* __SOC_TI_K3_NAVSS_PSILCFG_H__ */
