/*
 * Texas Instruments N-Port Ethernet Switch Address Lookup Engine
 *
 * Copyright (C) 2012 Texas Instruments
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/etherdevice.h>

#include "cpsw_ale.h"

#define BITMASK(bits)		(BIT(bits) - 1)
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS	DIV_ROUND_UP(ALE_ENTRY_BITS, 32)

#define ALE_VERSION_MAJOR(rev, mask) (((rev) >> 8) & (mask))
#define ALE_VERSION_MINOR(rev)	(rev & 0xff)
#define ALE_VERSION_1R3		0x0103
#define ALE_VERSION_1R4		0x0104
#define ALE_VERSION_9R3		0x0903

/* ALE Registers */
#define ALE_IDVER		0x00
#define ALE_STATUS		0x04
#define ALE_CONTROL		0x08
#define ALE_PRESCALE		0x10
#define ALE_UNKNOWNVLAN		0x18
#define ALE_TABLE_CONTROL	0x20
#define ALE_TABLE		0x34
#define ALE_PORTCTL		0x40

/* ALE NetCP NU switch specific Registers */
#define ALE_UNKNOWNVLAN_MEMBER			0x90
#define ALE_UNKNOWNVLAN_UNREG_MCAST_FLOOD	0x94
#define ALE_UNKNOWNVLAN_REG_MCAST_FLOOD		0x98
#define ALE_UNKNOWNVLAN_FORCE_UNTAG_EGRESS	0x9C
#define ALE_VLAN_MASK_MUX(reg)			(0xc0 + (0x4 * reg))

#define ALE_TABLE_WRITE		BIT(31)

#define ALE_TYPE_FREE			0
#define ALE_TYPE_ADDR			1
#define ALE_TYPE_VLAN			2
#define ALE_TYPE_VLAN_ADDR		3

#define ALE_UCAST_PERSISTANT		0
#define ALE_UCAST_UNTOUCHED		1
#define ALE_UCAST_OUI			2
#define ALE_UCAST_TOUCHED		3

#define ALE_TBL_ENTRY_SHOW_LEN		160
#define ALE_RAW_TBL_ENTRY_SHOW_LEN	32
#define ALE_TABLE_SIZE_MULTIPLIER	1024
#define ALE_STATUS_SIZE_MASK		0x1f
#define ALE_TABLE_SIZE_DEFAULT		64

/* ALE Table store VLAN command param indices */
enum {
	ALE_VP_VID,
	ALE_VP_FORCE_UT_EGR,
	ALE_VP_REG_FLD,
	ALE_VP_UNREG_FLD,
	ALE_VP_M_LIST,
	ALE_VP_NUM,
};

/* ALE Table store UCAST command param indices */
enum {
	ALE_UP_PORT,
	ALE_UP_BLOCK,
	ALE_UP_SECURE,
	ALE_UP_AGEABLE,
	ALE_UP_ADDR,
	ALE_UP_VID,
	ALE_UP_NUM,
};

/* ALE Table store MCAST command param indices */
enum {
	ALE_MP_PORT_MASK,
	ALE_MP_SUPER,
	ALE_MP_FW_ST,
	ALE_MP_ADDR,
	ALE_MP_VID,
	ALE_MP_NUM
};

static inline int cpsw_ale_get_field(u32 *ale_entry, u32 start, u32 bits)
{
	int idx;

	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	return (ale_entry[idx] >> start) & BITMASK(bits);
}

static inline void cpsw_ale_set_field(u32 *ale_entry, u32 start, u32 bits,
				      u32 value)
{
	int idx;

	value &= BITMASK(bits);
	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	ale_entry[idx] &= ~(BITMASK(bits) << start);
	ale_entry[idx] |=  (value << start);
}

#define DEFINE_ALE_FIELD(name, start, bits)				\
static inline int cpsw_ale_get_##name(u32 *ale_entry)			\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value)	\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

#define DEFINE_ALE_FIELD1(name, start)					\
static inline int cpsw_ale_get_##name(u32 *ale_entry, u32 bits)		\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value,	\
		u32 bits)						\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

DEFINE_ALE_FIELD(entry_type,		60,	2)
DEFINE_ALE_FIELD(vlan_id,		48,	12)
DEFINE_ALE_FIELD(mcast_state,		62,	2)
DEFINE_ALE_FIELD1(port_mask,		66)
DEFINE_ALE_FIELD(super,			65,	1)
DEFINE_ALE_FIELD(ucast_type,		62,     2)
DEFINE_ALE_FIELD1(port_num,		66)
DEFINE_ALE_FIELD(blocked,		65,     1)
DEFINE_ALE_FIELD(secure,		64,     1)
DEFINE_ALE_FIELD1(vlan_untag_force,	24)
DEFINE_ALE_FIELD1(vlan_reg_mcast,	16)
DEFINE_ALE_FIELD1(vlan_unreg_mcast,	8)
DEFINE_ALE_FIELD1(vlan_member_list,	0)
DEFINE_ALE_FIELD(mcast,			40,	1)
/* ALE NetCP nu switch specific */
DEFINE_ALE_FIELD(vlan_unreg_mcast_idx,	20,	3)
DEFINE_ALE_FIELD(vlan_reg_mcast_idx,	44,	3)

/* The MAC address field in the ALE entry cannot be macroized as above */
static inline void cpsw_ale_get_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		addr[i] = cpsw_ale_get_field(ale_entry, 40 - 8*i, 8);
}

static inline void cpsw_ale_set_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		cpsw_ale_set_field(ale_entry, 40 - 8*i, 8, addr[i]);
}

static int cpsw_ale_read(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->params.ale_entries);

	__raw_writel(idx, ale->params.ale_regs + ALE_TABLE_CONTROL);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		ale_entry[i] = __raw_readl(ale->params.ale_regs +
					   ALE_TABLE + 4 * i);

	return idx;
}

static int cpsw_ale_write(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->params.ale_entries);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		__raw_writel(ale_entry[i], ale->params.ale_regs +
			     ALE_TABLE + 4 * i);

	__raw_writel(idx | ALE_TABLE_WRITE, ale->params.ale_regs +
		     ALE_TABLE_CONTROL);

	return idx;
}

static int cpsw_ale_match_addr(struct cpsw_ale *ale, u8 *addr, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		u8 entry_addr[6];

		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		if (cpsw_ale_get_vlan_id(ale_entry) != vid)
			continue;
		cpsw_ale_get_addr(ale_entry, entry_addr);
		if (ether_addr_equal(entry_addr, addr))
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_vlan(struct cpsw_ale *ale, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_VLAN)
			continue;
		if (cpsw_ale_get_vlan_id(ale_entry) == vid)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_free(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type == ALE_TYPE_FREE)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_find_ageable(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		if (cpsw_ale_get_mcast(ale_entry))
			continue;
		type = cpsw_ale_get_ucast_type(ale_entry);
		if (type != ALE_UCAST_PERSISTANT &&
		    type != ALE_UCAST_OUI)
			return idx;
	}
	return -ENOENT;
}

static void cpsw_ale_flush_mcast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int mask;

	mask = cpsw_ale_get_port_mask(ale_entry,
				      ale->port_mask_bits);
	if ((mask & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	mask &= ~port_mask;

	/* free if only remaining port is host port */
	if (mask)
		cpsw_ale_set_port_mask(ale_entry, mask,
				       ale->port_mask_bits);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
}

int cpsw_ale_flush_multicast(struct cpsw_ale *ale, int port_mask, int vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int ret, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		ret = cpsw_ale_get_entry_type(ale_entry);
		if (ret != ALE_TYPE_ADDR && ret != ALE_TYPE_VLAN_ADDR)
			continue;

		/* if vid passed is -1 then remove all multicast entry from
		 * the table irrespective of vlan id, if a valid vlan id is
		 * passed then remove only multicast added to that vlan id.
		 * if vlan id doesn't match then move on to next entry.
		 */
		if (vid != -1 && cpsw_ale_get_vlan_id(ale_entry) != vid)
			continue;

		if (cpsw_ale_get_mcast(ale_entry)) {
			u8 addr[6];

			cpsw_ale_get_addr(ale_entry, addr);
			if (!is_broadcast_ether_addr(addr))
				cpsw_ale_flush_mcast(ale, ale_entry, port_mask);
		}

		cpsw_ale_write(ale, idx, ale_entry);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_flush_multicast);

static inline void cpsw_ale_set_vlan_entry_type(u32 *ale_entry,
						int flags, u16 vid)
{
	if (flags & ALE_VLAN) {
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_VLAN_ADDR);
		cpsw_ale_set_vlan_id(ale_entry, vid);
	} else {
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	}
}

int cpsw_ale_add_ucast(struct cpsw_ale *ale, u8 *addr, int port,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_vlan_entry_type(ale_entry, flags, vid);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_PERSISTANT);
	cpsw_ale_set_secure(ale_entry, (flags & ALE_SECURE) ? 1 : 0);
	cpsw_ale_set_blocked(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_port_num(ale_entry, port, ale->port_num_bits);

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_add_ucast);

int cpsw_ale_del_ucast(struct cpsw_ale *ale, u8 *addr, int port,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_del_ucast);

int cpsw_ale_add_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask,
		       int flags, u16 vid, int mcast_state)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_vlan_entry_type(ale_entry, flags, vid);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_super(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_mcast_state(ale_entry, mcast_state);

	mask = cpsw_ale_get_port_mask(ale_entry,
				      ale->port_mask_bits);
	port_mask |= mask;
	cpsw_ale_set_port_mask(ale_entry, port_mask,
			       ale->port_mask_bits);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_add_mcast);

int cpsw_ale_del_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		return -EINVAL;

	cpsw_ale_read(ale, idx, ale_entry);

	if (port_mask)
		cpsw_ale_set_port_mask(ale_entry, port_mask,
				       ale->port_mask_bits);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_del_mcast);

/* ALE NetCP NU switch specific vlan functions */
static void cpsw_ale_set_vlan_mcast(struct cpsw_ale *ale, u32 *ale_entry,
				    int reg_mcast, int unreg_mcast)
{
	int idx;

	/* Set VLAN registered multicast flood mask */
	idx = cpsw_ale_get_vlan_reg_mcast_idx(ale_entry);
	writel(reg_mcast, ale->params.ale_regs + ALE_VLAN_MASK_MUX(idx));

	/* Set VLAN unregistered multicast flood mask */
	idx = cpsw_ale_get_vlan_unreg_mcast_idx(ale_entry);
	writel(unreg_mcast, ale->params.ale_regs + ALE_VLAN_MASK_MUX(idx));
}

static void cpsw_ale_get_vlan_mcast(struct cpsw_ale *ale, u32 *ale_entry,
				    int *reg_mcast, int *unreg_mcast)
{
	int idx;

	/* Get VLAN registered multicast flood mask */
	idx = cpsw_ale_get_vlan_reg_mcast_idx(ale_entry);
	*reg_mcast = __raw_readl(ale->params.ale_regs +
				 ALE_VLAN_MASK_MUX(idx));

	/* Get VLAN unregistered multicast flood mask */
	idx = cpsw_ale_get_vlan_unreg_mcast_idx(ale_entry);
	*unreg_mcast = __raw_readl(ale->params.ale_regs +
				   ALE_VLAN_MASK_MUX(idx));
}

int cpsw_ale_add_vlan(struct cpsw_ale *ale, u16 vid, int port, int untag,
		      int reg_mcast, int unreg_mcast)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_VLAN);
	cpsw_ale_set_vlan_id(ale_entry, vid);

	cpsw_ale_set_vlan_untag_force(ale_entry, untag, ale->vlan_field_bits);
	if (!ale->params.nu_switch_ale) {
		cpsw_ale_set_vlan_reg_mcast(ale_entry, reg_mcast,
					    ale->vlan_field_bits);
		cpsw_ale_set_vlan_unreg_mcast(ale_entry, unreg_mcast,
					      ale->vlan_field_bits);
	} else {
		cpsw_ale_set_vlan_mcast(ale, ale_entry, reg_mcast, unreg_mcast);
	}
	cpsw_ale_set_vlan_member_list(ale_entry, port, ale->vlan_field_bits);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_add_vlan);

int cpsw_ale_del_vlan(struct cpsw_ale *ale, u16 vid, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_read(ale, idx, ale_entry);

	if (port_mask)
		cpsw_ale_set_vlan_member_list(ale_entry, port_mask,
					      ale->vlan_field_bits);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_del_vlan);

void cpsw_ale_set_allmulti(struct cpsw_ale *ale, int allmulti)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;
	int unreg_mcast = 0;

	/* Only bother doing the work if the setting is actually changing */
	if (ale->allmulti == allmulti)
		return;

	/* Remember the new setting to check against next time */
	ale->allmulti = allmulti;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_VLAN)
			continue;

		unreg_mcast =
			cpsw_ale_get_vlan_unreg_mcast(ale_entry,
						      ale->vlan_field_bits);
		if (allmulti)
			unreg_mcast |= 1;
		else
			unreg_mcast &= ~1;
		cpsw_ale_set_vlan_unreg_mcast(ale_entry, unreg_mcast,
					      ale->vlan_field_bits);
		cpsw_ale_write(ale, idx, ale_entry);
	}
}
EXPORT_SYMBOL_GPL(cpsw_ale_set_allmulti);

struct ale_control_info {
	const char	*name;
	int		offset, port_offset;
	int		shift, port_shift;
	int		bits;
};

static struct ale_control_info ale_controls[ALE_NUM_CONTROLS] = {
	[ALE_VERSION]		= {
		.name		= "version",
		.offset		= ALE_IDVER,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 32,
	},
	[ALE_ENABLE]		= {
		.name		= "enable",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 31,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_CLEAR]		= {
		.name		= "clear",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 30,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_AGEOUT]		= {
		.name		= "ageout",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 29,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_P0_UNI_FLOOD]	= {
		.name		= "port0_unicast_flood",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 8,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_VLAN_NOLEARN]	= {
		.name		= "vlan_nolearn",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 7,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_NO_PORT_VLAN]	= {
		.name		= "no_port_vlan",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 6,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_OUI_DENY]		= {
		.name		= "oui_deny",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 5,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_BYPASS]		= {
		.name		= "bypass",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 4,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_RATE_LIMIT_TX]	= {
		.name		= "rate_limit_tx",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 3,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_VLAN_AWARE]	= {
		.name		= "vlan_aware",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 2,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_AUTH_ENABLE]	= {
		.name		= "auth_enable",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 1,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_RATE_LIMIT]	= {
		.name		= "rate_limit",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_STATE]	= {
		.name		= "port_state",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 2,
	},
	[ALE_PORT_DROP_UNTAGGED] = {
		.name		= "drop_untagged",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 2,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_DROP_UNKNOWN_VLAN] = {
		.name		= "drop_unknown",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 3,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_NOLEARN]	= {
		.name		= "nolearn",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 4,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_NO_SA_UPDATE]	= {
		.name		= "no_source_update",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 5,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_MCAST_LIMIT]	= {
		.name		= "mcast_limit",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 16,
		.port_shift	= 0,
		.bits		= 8,
	},
	[ALE_PORT_BCAST_LIMIT]	= {
		.name		= "bcast_limit",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 24,
		.port_shift	= 0,
		.bits		= 8,
	},
	/* Fields below has individual registers on NetCP NU switch */
	[ALE_PORT_UNKNOWN_VLAN_MEMBER] = {
		.name		= "unknown_vlan_member",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNKNOWN_MCAST_FLOOD] = {
		.name		= "unknown_mcast_flood",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 8,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD] = {
		.name		= "unknown_reg_flood",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 16,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNTAGGED_EGRESS] = {
		.name		= "untagged_egress",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 24,
		.port_shift	= 0,
		.bits		= 6,
	},
};

int cpsw_ale_control_set(struct cpsw_ale *ale, int port, int control,
			 int value)
{
	const struct ale_control_info *info;
	int offset, shift;
	u32 tmp, mask;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	info = &ale_controls[control];
	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->params.ale_ports)
		return -EINVAL;

	mask = BITMASK(info->bits);
	if (value & ~mask)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->params.ale_regs + offset);
	tmp = (tmp & ~(mask << shift)) | (value << shift);
	__raw_writel(tmp, ale->params.ale_regs + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_control_set);

int cpsw_ale_control_get(struct cpsw_ale *ale, int port, int control)
{
	const struct ale_control_info *info;
	int offset, shift;
	u32 tmp;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	info = &ale_controls[control];
	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->params.ale_ports)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->params.ale_regs + offset) >> shift;
	return tmp & BITMASK(info->bits);
}
EXPORT_SYMBOL_GPL(cpsw_ale_control_get);

static int cpsw_ale_dump_mcast(struct cpsw_ale *ale, u32 *ale_entry, char *buf,
			       int len)
{
	static const char * const str_mcast_state[] = {"f", "blf", "lf", "f"};
	int mcast_state = cpsw_ale_get_mcast_state(ale_entry);
	int port_mask   = cpsw_ale_get_port_mask(ale_entry,
						 ale->port_mask_bits);
	int super       = cpsw_ale_get_super(ale_entry);
	int outlen = 0;

	outlen += snprintf(buf + outlen, len - outlen,
			   "mcstate: %s(%d), ", str_mcast_state[mcast_state],
			   mcast_state);
	outlen += snprintf(buf + outlen, len - outlen,
			   "port mask: %x, %ssuper\n", port_mask,
			   super ? "" : "no ");
	return outlen;
}

static int cpsw_ale_dump_ucast(struct cpsw_ale *ale,
			       u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	static const char * const str_ucast_type[] = {"persistent", "untouched",
							"oui", "touched"};
	int ucast_type  = cpsw_ale_get_ucast_type(ale_entry);
	int port_num    = cpsw_ale_get_port_num(ale_entry,
						ale->port_num_bits);
	int secure      = cpsw_ale_get_secure(ale_entry);
	int blocked     = cpsw_ale_get_blocked(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "uctype: %s(%d)", str_ucast_type[ucast_type],
			   ucast_type);
	if (ucast_type == ALE_UCAST_OUI)
		outlen += snprintf(buf + outlen, len - outlen, "\n");
	else
		outlen += snprintf(buf + outlen, len - outlen,
				", port: %d%s%s\n", port_num,
				secure ? ", Secure" : "",
				blocked ? ", Blocked" : "");
	return outlen;
}

static int cpsw_ale_dump_vlan(struct cpsw_ale *ale, u32 *ale_entry,
			      char *buf, int len)
{
	int outlen = 0, reg_mc_fld, unreg_mc_fld;
	int force_utag_egress	=
		cpsw_ale_get_vlan_untag_force(ale_entry,
					      ale->vlan_field_bits);
	int mem_list	=
		cpsw_ale_get_vlan_member_list(ale_entry, ale->vlan_field_bits);

	if (!ale->params.nu_switch_ale) {
		reg_mc_fld =
			cpsw_ale_get_vlan_reg_mcast(ale_entry,
						    ale->vlan_field_bits);
		unreg_mc_fld =
			cpsw_ale_get_vlan_unreg_mcast(ale_entry,
						      ale->vlan_field_bits);
	} else
		cpsw_ale_get_vlan_mcast(ale, ale_entry, &reg_mc_fld,
					&unreg_mc_fld);

	outlen += snprintf(buf + outlen, len - outlen,
			   "force_untag_egress: %02x, ", force_utag_egress);
	outlen += snprintf(buf + outlen, len - outlen,
			   "reg_fld: %02x, ", reg_mc_fld);
	outlen += snprintf(buf + outlen, len - outlen,
			   "unreg_fld: %02x, ", unreg_mc_fld);
	outlen += snprintf(buf + outlen, len - outlen,
			   "mem_list: %02x\n", mem_list);
	return outlen;
}

static int cpsw_ale_dump_entry(struct cpsw_ale *ale, int idx, u32 *ale_entry,
			       char *buf, int len)
{
	static const char * const str_type[] = {"free", "addr",
						"vlan", "vlan+addr"};
	int type, outlen = 0;
	u8 addr[6];

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (len < ALE_TBL_ENTRY_SHOW_LEN)
		return outlen;

	if (idx >= 0) {
		outlen += snprintf(buf + outlen, len - outlen,
				   "index %d, ", idx);
	}

	outlen += snprintf(buf + outlen, len - outlen, "raw: %08x %08x %08x, ",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	outlen += snprintf(buf + outlen, len - outlen,
			   "type: %s(%d), ", str_type[type], type);

	if (type != ALE_TYPE_VLAN) {
		cpsw_ale_get_addr(ale_entry, addr);
		outlen += snprintf(buf + outlen, len - outlen,
			   "addr: %02x:%02x:%02x:%02x:%02x:%02x, ",
			   (addr)[0], (addr)[1], (addr)[2],
			   (addr)[3], (addr)[4], (addr)[5]);
	}

	if (type == ALE_TYPE_VLAN || type == ALE_TYPE_VLAN_ADDR) {
		outlen += snprintf(buf + outlen, len - outlen, "vlan: %d, ",
				   cpsw_ale_get_vlan_id(ale_entry));
	}

	if (type == ALE_TYPE_VLAN)
		outlen += cpsw_ale_dump_vlan(ale, ale_entry,
				buf + outlen, len - outlen);
	else
		outlen += cpsw_ale_get_mcast(ale_entry) ?
		  cpsw_ale_dump_mcast(ale, ale_entry, buf + outlen,
				      len - outlen) :
		  cpsw_ale_dump_ucast(ale, ale_entry, buf + outlen,
				      len - outlen);

	return outlen;
}

static ssize_t cpsw_ale_control_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, port, len = 0, max_control = ARRAY_SIZE(ale_controls);
	struct cpsw_ale *ale = control_attr_to_ale(attr);
	struct cpsw_ale_params *params = &ale->params;
	const struct ale_control_info *info;
	const char *fmt = "%s=%d\n";
	u32 reg;

	for (i = 0, info = ale_controls; i < max_control; i++, info++) {
		if (i == ALE_VERSION) {
			reg = cpsw_ale_control_get(ale, 0, i);
			len +=
			snprintf(buf + len, SZ_4K - len,
				 "%s=(ALE_ID=0x%04x) Rev %d.%d\n",
				 info->name,
				 (reg & 0xffff0000) >> 16,
				 ALE_VERSION_MAJOR(reg,
						   params->major_ver_mask),
						   ALE_VERSION_MINOR(reg));
			continue;
		}

		/* global controls */
		if (info->port_shift == 0 &&  info->port_offset == 0) {
			if ((i >= ALE_PORT_UNKNOWN_VLAN_MEMBER) &&
			    (i <= ALE_PORT_UNTAGGED_EGRESS))
				fmt = "%s=0x%x\n";

			len += snprintf(buf + len, SZ_4K - len,
					fmt, info->name,
					cpsw_ale_control_get(ale, 0, i));
			continue;
		}

		/* port specific controls */
		for (port = 0; port < ale->params.ale_ports; port++) {
			len += snprintf(buf + len, SZ_4K - len,
					"%s.%d=%d\n", info->name, port,
					cpsw_ale_control_get(ale, port, i));
		}
	}

	return len;
}

static ssize_t cpsw_ale_control_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int port = 0, value, len, ret, control, max_control = ALE_NUM_CONTROLS;
	struct cpsw_ale *ale = control_attr_to_ale(attr);
	char ctrl_str[33], tmp_str[9];
	unsigned long end;

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		++buf;
		len = strcspn(buf, "=");
		if (len >= 8)
			return -ENOMEM;
		strncpy(tmp_str, buf, len);
		tmp_str[len] = '\0';
		if (kstrtoul(tmp_str, 0, &end))
			return -EINVAL;
		port = (int)end;
		buf += len;
	}

	if (*buf != '=')
		return -EINVAL;

	if (kstrtoul(buf + 1, 0, &end))
		return -EINVAL;

	value = (int)end;

	for (control = 0; control < max_control; control++)
		if (strcmp(ctrl_str, ale_controls[control].name) == 0)
			break;

	if (control >= max_control)
		return -ENOENT;

	dev_dbg(ale->params.dev, "processing command %s.%d=%d\n",
		ale_controls[control].name, port, value);

	ret = cpsw_ale_control_set(ale, port, control, value);
	if (ret < 0)
		return ret;

	return count;
}
DEVICE_ATTR(ale_control, S_IRUGO | S_IWUSR,
	    cpsw_ale_control_show, cpsw_ale_control_store);

static ssize_t cpsw_ale_table_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int not_shown = 0, total_outlen = 0, type, shown = 0;
	struct cpsw_ale *ale = table_attr_to_ale(attr);
	int len = SZ_4K, outlen = 0, idx, start;
	u32 ale_entry[ALE_ENTRY_WORDS];

	start = ale->show_next;

	for (idx = start; (idx < ale->params.ale_entries) &&
	     (len > total_outlen); idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen = cpsw_ale_dump_entry(ale, idx, ale_entry,
					     buf + total_outlen,
					     len - total_outlen);
		if (outlen == 0) {
			type = cpsw_ale_get_entry_type(ale_entry);
			if (type != ALE_TYPE_FREE) {
				++not_shown;
				break;
			}
		} else {
			total_outlen += outlen;
			++shown;
		}
	}

	/* update next show index */
	if (idx >= ale->params.ale_entries)
		ale->show_next = 0;
	else
		ale->show_next = idx;

	if (len > total_outlen + 32)
		total_outlen += snprintf(buf + total_outlen, len - total_outlen,
				"[%d..%d]: %d entries%s\n", start, idx - 1,
				shown, not_shown ? ", +" : "");

	return total_outlen;
}

struct ale_table_param {
	const char *name;
	union	{
		int	val;
		u8	addr[6];
	};
};

struct ale_table_cmd {
	const char *name;
	int (*process)(struct cpsw_ale *ale,
		       const u8 *params_str, size_t str_len);
};

static struct ale_table_param vlan_params[] = {
	[ALE_VP_VID]		= { .name = "vid", },
	[ALE_VP_FORCE_UT_EGR]	= { .name = "force_untag_egress", },
	[ALE_VP_REG_FLD]	= { .name = "reg_fld_mask", },
	[ALE_VP_UNREG_FLD]	= { .name = "unreg_fld_mask", },
	[ALE_VP_M_LIST]		= { .name = "mem_list", },
};

static struct ale_table_param vlan_ucast_params[] = {
	[ALE_UP_PORT]		= { .name = "port", },
	[ALE_UP_BLOCK]		= { .name = "block", },
	[ALE_UP_SECURE]		= { .name = "secure", },
	[ALE_UP_AGEABLE]	= { .name = "ageable", },
	[ALE_UP_ADDR]		= { .name = "addr", },
	[ALE_UP_VID]		= { .name = "vid", },
};

static struct ale_table_param vlan_mcast_params[] = {
	[ALE_MP_PORT_MASK]	= { .name = "port_mask", },
	[ALE_MP_SUPER]		= { .name = "supervisory", },
	[ALE_MP_FW_ST]		= { .name = "mc_fw_st", },
	[ALE_MP_ADDR]		= { .name = "addr", },
	[ALE_MP_VID]		= { .name = "vid", },
};

static struct ale_table_param oui_params[] = {
	{ .name	= "addr", },
};

void cpsw_ale_table_store_init_params(
	struct ale_table_param *params, int param_num)
{
	int i;

	for (i = 0; i < param_num; i++)
		memset(params[i].addr, 0, 6);
}

static int cpsw_ale_table_store_get_params(struct cpsw_ale *ale,
					   struct ale_table_param *params,
					   int param_num, const u8 *params_str,
					   size_t str_len)
{
	char param_name[33], val_str[33];
	size_t tmp_len = str_len;
	int len, i, n, addr_len;
	unsigned int iaddr[6];
	unsigned long end;

	while (tmp_len > 0) {
		len = strcspn(params_str, "=");
		if (len >= 32)
			return -ENOMEM;

		strncpy(param_name, params_str, len);
		param_name[len] = '\0';
		params_str += len;
		tmp_len -= len;

		if (*params_str != '=')
			return -EINVAL;

		++params_str;
		--tmp_len;

		len = strcspn(params_str, ".");
		if (len >= 32)
			return -ENOMEM;

		strncpy(val_str, params_str, len);
		val_str[len] = '\0';
		params_str += len;
		tmp_len -= len;

		if (*params_str == '.') {
			++params_str;
			--tmp_len;
		}

		for (n = 0; n < param_num; n++) {
			if (strcmp(param_name, params[n].name) != 0)
				continue;

			if (strcmp(param_name, "addr") == 0) {
				addr_len =
					sscanf(val_str,
					       "%02x:%02x:%02x:%02x:%02x:%02x",
					       &iaddr[0], &iaddr[1], &iaddr[2],
					       &iaddr[3], &iaddr[4], &iaddr[5]);
				if (addr_len != 6 && addr_len != 3)
					return -EINVAL;

				for (i = 0; i < addr_len; i++)
					params[n].addr[i] = iaddr[i];

				break;
			}

			if (kstrtoul(val_str, 0, &end))
				return -EINVAL;

			params[n].val = (int)end;
			break;
		}

		if (n >= param_num)
			return -EINVAL;
	}

	return str_len;
}

static int cpsw_ale_table_store_vlan(struct cpsw_ale *ale, const u8 *params_str,
				     size_t str_len)
{
	int ret;

	cpsw_ale_table_store_init_params(vlan_params, ALE_VP_NUM);
	vlan_params[ALE_VP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale, vlan_params, ALE_VP_NUM,
					      params_str, str_len);
	if (ret < 0)
		return ret;

	ret = cpsw_ale_add_vlan(ale, vlan_params[ALE_VP_VID].val,
				vlan_params[ALE_VP_M_LIST].val,
				vlan_params[ALE_VP_FORCE_UT_EGR].val,
				vlan_params[ALE_VP_REG_FLD].val,
				vlan_params[ALE_VP_UNREG_FLD].val);
	if (ret < 0)
		return ret;
	return str_len;
}

static int
cpsw_ale_table_store_vlan_ucast(struct cpsw_ale *ale, const u8 *params_str,
				size_t str_len, int has_vid)
{
	int ret, flags = 0;

	cpsw_ale_table_store_init_params(vlan_ucast_params, ALE_UP_NUM);
	vlan_ucast_params[ALE_UP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale, vlan_ucast_params,
					      ALE_UP_NUM, params_str, str_len);

	if (ret < 0)
		return ret;

	if (!has_vid && vlan_ucast_params[ALE_UP_VID].val >= 0)
		return -EINVAL;

	if (vlan_ucast_params[ALE_UP_BLOCK].val)
		flags |= ALE_BLOCKED;

	if (vlan_ucast_params[ALE_UP_SECURE].val)
		flags |= ALE_SECURE;

	ret = cpsw_ale_add_ucast(ale, vlan_ucast_params[ALE_UP_ADDR].addr,
				 vlan_ucast_params[ALE_UP_PORT].val, flags,
				 vlan_ucast_params[ALE_UP_VID].val);
	if (ret < 0)
		return ret;

	return str_len;
}

static int cpsw_ale_table_store_u_proc(struct cpsw_ale *ale,
				       const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_ucast(ale, params_str, str_len, 0);
}

static int cpsw_ale_table_store_vu_proc(struct cpsw_ale *ale,
					const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_ucast(ale, params_str, str_len, 1);
}

static int cpsw_ale_table_store_vlan_mcast(struct cpsw_ale *ale,
					   const u8 *params_str,
					   size_t str_len, int has_vid)
{
	int ret;

	cpsw_ale_table_store_init_params(vlan_mcast_params, ALE_MP_NUM);
	vlan_mcast_params[ALE_MP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale, vlan_mcast_params,
					      ALE_MP_NUM, params_str, str_len);
	if (ret < 0)
		return ret;

	if (!has_vid && vlan_mcast_params[ALE_MP_VID].val >= 0)
		return -EINVAL;

	ret = cpsw_ale_add_mcast(ale, vlan_mcast_params[ALE_MP_ADDR].addr,
				 vlan_mcast_params[ALE_MP_PORT_MASK].val,
				 vlan_mcast_params[ALE_MP_SUPER].val,
				 vlan_mcast_params[ALE_MP_FW_ST].val,
				 vlan_mcast_params[ALE_MP_VID].val);
	if (ret < 0)
		return ret;

	return str_len;
}

static int cpsw_ale_table_store_m_proc(struct cpsw_ale *ale,
				       const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_mcast(ale, params_str, str_len, 0);
}

static int cpsw_ale_table_store_vm_proc(struct cpsw_ale *ale,
					const u8 *params_str,
					size_t str_len)
{
	return  cpsw_ale_table_store_vlan_mcast(ale, params_str, str_len, 1);
}

static int cpsw_ale_add_oui(struct cpsw_ale *ale, u8 *addr)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_OUI);

	idx = cpsw_ale_match_addr(ale, addr, -1);
	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

static int cpsw_ale_table_store_oui(struct cpsw_ale *ale, const u8 *params_str,
				    size_t str_len)
{
	int ret;

	cpsw_ale_table_store_init_params(oui_params, 1);

	ret = cpsw_ale_table_store_get_params(ale, oui_params, 1, params_str,
					      str_len);
	if (ret < 0)
		return ret;

	/* Clear out the don't cares */
	oui_params[0].addr[3] = 0;
	oui_params[0].addr[4] = 0;
	oui_params[0].addr[5] = 0;

	ret = cpsw_ale_add_oui(ale, oui_params[0].addr);
	if (ret < 0)
		return ret;

	return str_len;
}

static int cpsw_ale_table_store_del(struct cpsw_ale *ale, int idx)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type;

	dev_dbg(ale->params.dev, "deleting entry[%d] ...\n", idx);

	if (idx >= ale->params.ale_entries)
		return -EINVAL;

	cpsw_ale_read(ale, idx, ale_entry);

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return -EINVAL;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

static struct ale_table_cmd ale_table_cmds[] = {
	{
		.name		= "v",
		.process	= cpsw_ale_table_store_vlan,
	},
	{
		.name		= "m",
		.process	= cpsw_ale_table_store_m_proc,
	},
	{
		.name		= "vm",
		.process	= cpsw_ale_table_store_vm_proc,
	},
	{
		.name		= "u",
		.process	= cpsw_ale_table_store_u_proc,
	},
	{
		.name		= "vu",
		.process	= cpsw_ale_table_store_vu_proc,
	},
	{
		.name		= "o",
		.process	= cpsw_ale_table_store_oui,
	},
};

static ssize_t cpsw_ale_table_store_proc(struct cpsw_ale *ale,
					 const char *buf, size_t count)
{
	int len, i, tmp_count = count, ret = -EINVAL;
	char ctrl_str[33];
	unsigned long end;

	len = strcspn(buf, ".:");
	if (len >= 5)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';

	/* skip to param beginning */
	buf += len;
	tmp_count -= len;

	if (*buf == ':') {
		/* delete cmd */
		if (kstrtoul(ctrl_str, 0, &end))
			return -EINVAL;
		ret = cpsw_ale_table_store_del(ale, end);
		if (ret != 0)
			return ret;
		else
			return count;
	}

	if (len >= 3)
		return -ENOMEM;

	if (*buf != '.')
		return -EINVAL;

	++buf;
	--tmp_count;

	for (i = 0; i < ARRAY_SIZE(ale_table_cmds); i++) {
		if (strcmp(ale_table_cmds[i].name, ctrl_str) == 0) {
			ret = ale_table_cmds[i].process(ale, buf, tmp_count);
			break;
		}
	}

	if (ret < 0)
		return ret;
	else
		return count;
}

static ssize_t cpsw_ale_table_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	return cpsw_ale_table_store_proc(ale, buf, count);
}
DEVICE_ATTR(ale_table, S_IRUGO | S_IWUSR,
	    cpsw_ale_table_show, cpsw_ale_table_store);

static int cpsw_ale_dump_entry_raw(int idx, u32 *ale_entry, char *buf, int len)
{
	int type, outlen = 0;

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (len < ALE_RAW_TBL_ENTRY_SHOW_LEN)
		return outlen;

	if (idx >= 0)
		outlen += snprintf(buf + outlen, len - outlen,
				   "%d: ", idx);

	outlen += snprintf(buf + outlen, len - outlen, "%02x %08x %08x\n",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	return outlen;
}

static ssize_t cpsw_ale_table_raw_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct cpsw_ale *ale = table_raw_attr_to_ale(attr);
	int not_shown = 0, total_outlen = 0, shown = 0;
	int outlen = 0, idx, start, type;
	u32 ale_entry[ALE_ENTRY_WORDS];

	start = ale->raw_show_next;

	for (idx = start; (idx < ale->params.ale_entries) &&
	     (total_outlen < PAGE_SIZE); idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen = cpsw_ale_dump_entry_raw(idx, ale_entry,
						 buf + total_outlen,
						 PAGE_SIZE - total_outlen);
		if (outlen == 0) {
			type = cpsw_ale_get_entry_type(ale_entry);
			if (type != ALE_TYPE_FREE) {
				++not_shown;
				break;
			}
		} else {
			total_outlen += outlen;
			++shown;
		}
	}

	/* update next show index */
	if (idx >= ale->params.ale_entries)
		ale->raw_show_next = 0;
	else
		ale->raw_show_next = idx;

	if ((total_outlen + 32) < PAGE_SIZE)
		total_outlen += snprintf(buf + total_outlen,
					 PAGE_SIZE - total_outlen,
					 "[%d..%d]: %d entries%s\n",
					 start, idx - 1, shown, not_shown ?
					 ", +" : "");

	return total_outlen;
}

static ssize_t cpsw_ale_table_raw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cpsw_ale *ale = table_raw_attr_to_ale(attr);
	unsigned long end;

	if (kstrtoul(buf, 0, &end) == 0) {
		/* set start-show-index command */
		ale->raw_show_next = (int)end;
		if (ale->raw_show_next >= ale->params.ale_entries)
			ale->raw_show_next = 0;
		return count;
	}

	/* add or delete command */
	return cpsw_ale_table_store_proc(ale, buf, count);
}
DEVICE_ATTR(ale_table_raw, S_IRUGO | S_IWUSR,
	    cpsw_ale_table_raw_show, cpsw_ale_table_raw_store);

static void cpsw_ale_timer(unsigned long arg)
{
	struct cpsw_ale *ale = (struct cpsw_ale *)arg;

	cpsw_ale_control_set(ale, 0, ALE_AGEOUT, 1);

	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

void cpsw_ale_start(struct cpsw_ale *ale)
{
	u32 rev, ale_entries;
	int ret, i;

	rev = __raw_readl(ale->params.ale_regs + ALE_IDVER);
	if (!ale->params.major_ver_mask)
		ale->params.major_ver_mask = 0xff;
	ale->version =
		(ALE_VERSION_MAJOR(rev, ale->params.major_ver_mask) << 8) |
		 ALE_VERSION_MINOR(rev);
	dev_info(ale->params.dev, "initialized cpsw ale version %d.%d\n",
		 ALE_VERSION_MAJOR(rev, ale->params.major_ver_mask),
		 ALE_VERSION_MINOR(rev));

	if (!ale->params.ale_entries) {
		ale_entries =
			__raw_readl(ale->params.ale_regs + ALE_STATUS) &
				    ALE_STATUS_SIZE_MASK;
		/* ALE available on newer NetCP switches has introduced
		 * a register, ALE_STATUS, to indicate the size of ALE
		 * table which shows the size as a multiple of 1024 entries.
		 * For these, params.ale_entries will be set to zero. So
		 * read the register and update the value of ale_entries.
		 * ALE table on NetCP lite, is much smaller and is indicated
		 * by a value of zero in ALE_STATUS. So use a default value
		 * of ALE_TABLE_SIZE_DEFAULT for this. Caller is expected
		 * to set the value of ale_entries for all other versions
		 * of ALE.
		 */
		if (!ale_entries)
			ale_entries = ALE_TABLE_SIZE_DEFAULT;
		else
			ale_entries *= ALE_TABLE_SIZE_MULTIPLIER;
		ale->params.ale_entries = ale_entries;
	}
	dev_info(ale->params.dev,
		 "ALE Table size %ld\n", ale->params.ale_entries);

	/* set default bits for existing h/w */
	ale->port_mask_bits = 3;
	ale->port_num_bits = 2;
	ale->vlan_field_bits = 3;

	/* Set defaults override for ALE on NetCP NU switch and for version
	 * 1R3
	 */
	if (ale->params.nu_switch_ale) {
		/* Separate registers for unknown vlan configuration.
		 * Also there are N bits, where N is number of ale
		 * ports and shift value should be 0
		 */
		ale_controls[ALE_PORT_UNKNOWN_VLAN_MEMBER].bits =
					ale->params.ale_ports;
		ale_controls[ALE_PORT_UNKNOWN_VLAN_MEMBER].offset =
					ALE_UNKNOWNVLAN_MEMBER;
		ale_controls[ALE_PORT_UNKNOWN_MCAST_FLOOD].bits =
					ale->params.ale_ports;
		ale_controls[ALE_PORT_UNKNOWN_MCAST_FLOOD].shift = 0;
		ale_controls[ALE_PORT_UNKNOWN_MCAST_FLOOD].offset =
					ALE_UNKNOWNVLAN_UNREG_MCAST_FLOOD;
		ale_controls[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD].bits =
					ale->params.ale_ports;
		ale_controls[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD].shift = 0;
		ale_controls[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD].offset =
					ALE_UNKNOWNVLAN_REG_MCAST_FLOOD;
		ale_controls[ALE_PORT_UNTAGGED_EGRESS].bits =
					ale->params.ale_ports;
		ale_controls[ALE_PORT_UNTAGGED_EGRESS].shift = 0;
		ale_controls[ALE_PORT_UNTAGGED_EGRESS].offset =
					ALE_UNKNOWNVLAN_FORCE_UNTAG_EGRESS;
		ale->port_mask_bits = ale->params.ale_ports;
		ale->port_num_bits = ale->params.ale_ports - 1;
		ale->vlan_field_bits = ale->params.ale_ports;
	} else if (ale->version == ALE_VERSION_1R3) {
		ale->port_mask_bits = ale->params.ale_ports;
		ale->port_num_bits = 3;
		ale->vlan_field_bits = ale->params.ale_ports;
	}

	if ((ale->version == ALE_VERSION_1R3) ||
	    (ale->params.nu_switch_ale) ||
	    (ale->version == ALE_VERSION_9R3)) {
		/* disable forwarding on all ports */
		for (i = 0; i < ale->params.ale_ports; ++i)
			cpsw_ale_control_set(ale, i, ALE_PORT_STATE,
					     ALE_PORT_STATE_DISABLE);

		ale->ale_control_attr = dev_attr_ale_control;
		sysfs_attr_init(&ale->ale_control_attr.attr);
		ret = device_create_file(ale->params.dev,
					 &ale->ale_control_attr);
		WARN_ON(ret < 0);

		ale->ale_table_attr = dev_attr_ale_table;
		sysfs_attr_init(&ale->ale_table_attr.attr);
		ret = device_create_file(ale->params.dev, &ale->ale_table_attr);
		WARN_ON(ret < 0);

		ale->ale_table_raw_attr = dev_attr_ale_table_raw;
		sysfs_attr_init(&ale->ale_table_raw_attr.attr);
		ret = device_create_file(ale->params.dev,
					 &ale->ale_table_raw_attr);
		WARN_ON(ret < 0);
	}
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 1);
	cpsw_ale_control_set(ale, 0, ALE_CLEAR, 1);

	init_timer(&ale->timer);
	ale->timer.data	    = (unsigned long)ale;
	ale->timer.function = cpsw_ale_timer;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}
EXPORT_SYMBOL_GPL(cpsw_ale_start);

void cpsw_ale_stop(struct cpsw_ale *ale)
{
	del_timer_sync(&ale->timer);
	if ((ale->version == ALE_VERSION_1R3) ||
	    (ale->params.nu_switch_ale) ||
	    (ale->version == ALE_VERSION_9R3)) {
		device_remove_file(ale->params.dev, &ale->ale_table_attr);
		device_remove_file(ale->params.dev, &ale->ale_control_attr);
		device_remove_file(ale->params.dev, &ale->ale_table_raw_attr);
	}
}
EXPORT_SYMBOL_GPL(cpsw_ale_stop);

struct cpsw_ale *cpsw_ale_create(struct cpsw_ale_params *params)
{
	struct cpsw_ale *ale;

	ale = kzalloc(sizeof(*ale), GFP_KERNEL);
	if (!ale)
		return NULL;

	ale->params = *params;
	ale->ageout = ale->params.ale_ageout * HZ;

	return ale;
}
EXPORT_SYMBOL_GPL(cpsw_ale_create);

int cpsw_ale_destroy(struct cpsw_ale *ale)
{
	if (!ale)
		return -EINVAL;
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 0);
	kfree(ale);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_destroy);

void cpsw_ale_dump(struct cpsw_ale *ale, u32 *data)
{
	int i;

	for (i = 0; i < ale->params.ale_entries; i++) {
		cpsw_ale_read(ale, i, data);
		data += ALE_ENTRY_WORDS;
	}
}
EXPORT_SYMBOL_GPL(cpsw_ale_dump);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI CPSW ALE driver");
MODULE_AUTHOR("Texas Instruments");
