/*
 * am335x specific cpsw dt fixups
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>

#include "soc.h"
#include "control.h"

/**
 * am33xx_dt_cpsw_set_mac_from_efuse - Add mac-address property using
 * ethernet hwaddr from efuse
 * @np:		Pointer to the cpsw slave to set mac address of
 * @idx:	Mac address index to use from efuse
 */
static void am33xx_dt_cpsw_set_mac_from_efuse(struct device_node *np, int idx)
{
	struct property *prop;
	u32 lo, hi;
	u8 *mac;

	switch (idx) {
	case 0:
		lo = omap_ctrl_readl(AM33XX_CONTROL_MAC_ID0_LOW);
		hi = omap_ctrl_readl(AM33XX_CONTROL_MAC_ID0_HIGH);
		break;

	case 1:
		lo = omap_ctrl_readl(AM33XX_CONTROL_MAC_ID1_LOW);
		hi = omap_ctrl_readl(AM33XX_CONTROL_MAC_ID1_HIGH);
		break;

	default:
		pr_err("cpsw.%d: too many slaves found\n", idx);
		return;
	}

	prop = kzalloc(sizeof(*prop) + ETH_ALEN, GFP_KERNEL);
	if (!prop)
		return;

	prop->value = prop + 1;
	prop->length = ETH_ALEN;
	prop->name = kstrdup("mac-address", GFP_KERNEL);
	if (!prop->name) {
		kfree(prop);
		return;
	}

	mac = prop->value;

	mac[0] = hi;
	mac[1] = hi >> 8;
	mac[2] = hi >> 16;
	mac[3] = hi >> 24;
	mac[4] = lo;
	mac[5] = lo >> 8;

	of_update_property(np, prop);

	pr_info("cpsw.%d: No hwaddr in dt. Using %pM from efuse\n", idx, mac);
}

static int __init am33xx_dt_cpsw_mac_fixup(void)
{
	struct device_node *np, *slave;
	int idx = 0;

	if (!soc_is_am33xx())
		return -ENODEV;

	for_each_compatible_node(np, NULL, "ti,cpsw")
		for_each_node_by_name(slave, "slave") {
			if (!of_get_mac_address(slave))
				am33xx_dt_cpsw_set_mac_from_efuse(slave, idx);
			idx++;
		}

	return 0;
}
arch_initcall(am33xx_dt_cpsw_mac_fixup);
