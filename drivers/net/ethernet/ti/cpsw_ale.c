/*
 * Texas Instruments 3-Port Ethernet Switch Address Lookup Engine
 *
 * Copyright (C) 2010 Texas Instruments
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
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/module.h>

#include "cpsw_ale.h"

#define BITMASK(bits)		(BIT(bits) - 1)
#define ADDR_FMT_STR		"%02x:%02x:%02x:%02x:%02x:%02x"
#define ADDR_FMT_ARGS(addr)	(addr)[0], (addr)[1], (addr)[2], \
				(addr)[3], (addr)[4], (addr)[5]
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS		DIV_ROUND_UP(ALE_ENTRY_BITS, 32)

/* ALE Registers */
#define ALE_IDVER		0x00
#define ALE_CONTROL		0x08
#define ALE_PRESCALE		0x10
#define ALE_UNKNOWNVLAN		0x18
#define ALE_TABLE_CONTROL	0x20
#define ALE_TABLE		0x34
#define ALE_PORTCTL		0x40

#define ALE_TABLE_WRITE		BIT(31)

#define ALE_TYPE_FREE			0
#define ALE_TYPE_ADDR			1
#define ALE_TYPE_VLAN			2
#define ALE_TYPE_VLAN_ADDR		3

#define ALE_UCAST_PERSISTANT		0
#define ALE_UCAST_UNTOUCHED		1
#define ALE_UCAST_OUI			2
#define ALE_UCAST_TOUCHED		3

#define ALE_MCAST_FWD			0
#define ALE_MCAST_BLOCK_LEARN_FWD	1
#define ALE_MCAST_FWD_LEARN		2
#define ALE_MCAST_FWD_2			3

/* the following remap params into members of cpsw_ale */
#define ale_regs	params.ale_regs
#define ale_entries	params.ale_entries
#define ale_ports	params.ale_ports

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

DEFINE_ALE_FIELD(entry_type,		60,	2)
DEFINE_ALE_FIELD(vlan_id,		48,	12)
DEFINE_ALE_FIELD(mcast_state,		62,	2)
DEFINE_ALE_FIELD(port_mask,		66,     3)
DEFINE_ALE_FIELD(super,			65,	1)
DEFINE_ALE_FIELD(ucast_type,		62,     2)
DEFINE_ALE_FIELD(port_num,		66,     2)
DEFINE_ALE_FIELD(blocked,		65,     1)
DEFINE_ALE_FIELD(secure,		64,     1)
DEFINE_ALE_FIELD(vlan_untag_force,	24,	3)
DEFINE_ALE_FIELD(vlan_reg_mcast,	16,	3)
DEFINE_ALE_FIELD(vlan_unreg_mcast,	8,	3)
DEFINE_ALE_FIELD(vlan_member_list,	0,	3)
DEFINE_ALE_FIELD(mcast,			40,     1)

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

	WARN_ON(idx > ale->ale_entries);

	__raw_writel(idx, ale->ale_regs + ALE_TABLE_CONTROL);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		ale_entry[i] = __raw_readl(ale->ale_regs + ALE_TABLE + 4 * i);

	return idx;
}

static int cpsw_ale_write(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->ale_entries);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		__raw_writel(ale_entry[i], ale->ale_regs + ALE_TABLE + 4 * i);

	__raw_writel(idx | ALE_TABLE_WRITE, ale->ale_regs + ALE_TABLE_CONTROL);

	return idx;
}

static int cpsw_ale_match_addr(struct cpsw_ale *ale, u8* addr)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		u8 entry_addr[6];

		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		cpsw_ale_get_addr(ale_entry, entry_addr);
		if (memcmp(entry_addr, addr, 6) == 0)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_free(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
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

	for (idx = 0; idx < ale->ale_entries; idx++) {
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

	mask = cpsw_ale_get_port_mask(ale_entry);
	if ((mask & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	mask &= ~port_mask;

	/* free if only remaining port is host port */
	if (mask == BIT(ale->ale_ports))
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	else
		cpsw_ale_set_port_mask(ale_entry, mask);
}

static void cpsw_ale_flush_ucast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int port;

	port = cpsw_ale_get_port_num(ale_entry);
	if ((BIT(port) & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
}

int cpsw_ale_flush(struct cpsw_ale *ale, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int ret, idx;

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		ret = cpsw_ale_get_entry_type(ale_entry);
		if (ret != ALE_TYPE_ADDR && ret != ALE_TYPE_VLAN_ADDR)
			continue;

		if (cpsw_ale_get_mcast(ale_entry))
			cpsw_ale_flush_mcast(ale, ale_entry, port_mask);
		else
			cpsw_ale_flush_ucast(ale, ale_entry, port_mask);

		cpsw_ale_write(ale, idx, ale_entry);
	}
	return 0;
}

static int cpsw_ale_dump_mcast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	static const char *str_mcast_state[] = {"f", "blf", "lf", "f"};
	int mcast_state = cpsw_ale_get_mcast_state(ale_entry);
	int port_mask   = cpsw_ale_get_port_mask(ale_entry);
	int super       = cpsw_ale_get_super(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "mcstate: %s(%d), ", str_mcast_state[mcast_state],
			   mcast_state);
	outlen += snprintf(buf + outlen, len - outlen,
			   "port mask: %x, %ssuper\n", port_mask,
			   super ? "" : "no ");
	return outlen;
}

static int cpsw_ale_dump_ucast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	static const char *str_ucast_type[] = {"persistant", "untouched",
					       "oui", "touched"};
	int ucast_type  = cpsw_ale_get_ucast_type(ale_entry);
	int port_num    = cpsw_ale_get_port_num(ale_entry);
	int secure      = cpsw_ale_get_secure(ale_entry);
	int blocked     = cpsw_ale_get_blocked(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "uctype: %s(%d), ", str_ucast_type[ucast_type],
			   ucast_type);
	outlen += snprintf(buf + outlen, len - outlen,
			   "port: %d%s%s\n", port_num, secure ? ", Secure" : "",
			   blocked ? ", Blocked" : "");
	return outlen;
}

static int cpsw_ale_dump_entry(int idx, u32 *ale_entry, char *buf, int len)
{
	int type, outlen = 0;
	u8 addr[6];
	static const char *str_type[] = {"free", "addr", "vlan", "vlan+addr"};

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (idx >= 0) {
		outlen += snprintf(buf + outlen, len - outlen,
				   "index %d, ", idx);
	}

	outlen += snprintf(buf + outlen, len - outlen, "raw: %08x %08x %08x, ",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	outlen += snprintf(buf + outlen, len - outlen,
			   "type: %s(%d), ", str_type[type], type);

	cpsw_ale_get_addr(ale_entry, addr);
	outlen += snprintf(buf + outlen, len - outlen,
			   "addr: " ADDR_FMT_STR ", ", ADDR_FMT_ARGS(addr));

	if (type == ALE_TYPE_VLAN || type == ALE_TYPE_VLAN_ADDR) {
		outlen += snprintf(buf + outlen, len - outlen, "vlan: %d, ",
				   cpsw_ale_get_vlan_id(ale_entry));
	}

	outlen += cpsw_ale_get_mcast(ale_entry) ?
		  cpsw_ale_dump_mcast(ale_entry, buf + outlen, len - outlen) :
		  cpsw_ale_dump_ucast(ale_entry, buf + outlen, len - outlen);

	return outlen;
}

int cpsw_ale_add_ucast(struct cpsw_ale *ale, u8 *addr, int port, int flags)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_PERSISTANT);
	cpsw_ale_set_secure(ale_entry, (flags & ALE_SECURE) ? 1 : 0);
	cpsw_ale_set_blocked(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_port_num(ale_entry, port);

	idx = cpsw_ale_match_addr(ale, addr);
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

int cpsw_ale_del_ucast(struct cpsw_ale *ale, u8 *addr, int port)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_del_ucast);

int cpsw_ale_add_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_mcast_state(ale_entry, ALE_MCAST_FWD_2);

	mask = cpsw_ale_get_port_mask(ale_entry);
	port_mask |= mask;
	cpsw_ale_set_port_mask(ale_entry, port_mask);

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

int cpsw_ale_del_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr);
	if (idx < 0)
		return -EINVAL;

	cpsw_ale_read(ale, idx, ale_entry);
	mask = cpsw_ale_get_port_mask(ale_entry);
	port_mask = mask & ~port_mask;

	if (port_mask == BIT(ale->ale_ports))
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	else
		cpsw_ale_set_port_mask(ale_entry, port_mask);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_del_mcast);

struct ale_control_info {
	const char	*name;
	int		offset, port_offset;
	int		shift, port_shift;
	int		bits;
};

#define CTRL_GLOBAL(name, bit)		{#name, ALE_CONTROL, 0, bit, 0, 1}
#define CTRL_UNK(name, bit)		{#name, ALE_UNKNOWNVLAN, 0, bit, 1, 1}
#define CTRL_PORTCTL(name, start, bits)	{#name, ALE_PORTCTL, 4, start, 0, bits}

static struct ale_control_info ale_controls[] = {
	[ALE_ENABLE]		= CTRL_GLOBAL(enable, 31),
	[ALE_CLEAR]		= CTRL_GLOBAL(clear, 30),
	[ALE_AGEOUT]		= CTRL_GLOBAL(ageout, 29),
	[ALE_VLAN_NOLEARN]	= CTRL_GLOBAL(vlan_nolearn, 7),
	[ALE_NO_PORT_VLAN]	= CTRL_GLOBAL(no_port_vlan, 6),
	[ALE_OUI_DENY]		= CTRL_GLOBAL(oui_deny, 5),
	[ALE_BYPASS]		= CTRL_GLOBAL(bypass, 4),
	[ALE_RATE_LIMIT_TX]	= CTRL_GLOBAL(rate_limit_tx, 3),
	[ALE_VLAN_AWARE]	= CTRL_GLOBAL(vlan_aware, 2),
	[ALE_AUTH_ENABLE]	= CTRL_GLOBAL(auth_enable, 1),
	[ALE_RATE_LIMIT]	= CTRL_GLOBAL(rate_limit, 0),

	[ALE_PORT_STATE]	     = CTRL_PORTCTL(port_state, 0, 2),
	[ALE_PORT_DROP_UNTAGGED]     = CTRL_PORTCTL(drop_untagged, 2, 1),
	[ALE_PORT_DROP_UNKNOWN_VLAN] = CTRL_PORTCTL(drop_unknown, 3, 1),
	[ALE_PORT_NOLEARN]	     = CTRL_PORTCTL(nolearn, 4, 1),
	[ALE_PORT_MCAST_LIMIT]	     = CTRL_PORTCTL(mcast_limit, 16, 8),
	[ALE_PORT_BCAST_LIMIT]	     = CTRL_PORTCTL(bcast_limit, 24, 8),

	[ALE_PORT_UNKNOWN_VLAN_MEMBER]	   = CTRL_UNK(unknown_vlan_member, 0),
	[ALE_PORT_UNKNOWN_MCAST_FLOOD]	   = CTRL_UNK(unknown_mcast_flood, 8),
	[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD] = CTRL_UNK(unknown_reg_flood, 16),
	[ALE_PORT_UNTAGGED_EGRESS]	   = CTRL_UNK(untagged_egress, 24),
};

int cpsw_ale_control_set(struct cpsw_ale *ale, int port, int control,
			 int value)
{
	struct ale_control_info *info = &ale_controls[control];
	int offset, shift;
	u32 tmp, mask;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->ale_ports)
		return -EINVAL;

	mask = BITMASK(info->bits);
	if (value & ~mask)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->ale_regs + offset);
	tmp = (tmp & ~(mask << shift)) | (value << shift);
	__raw_writel(tmp, ale->ale_regs + offset);

	{
		volatile u32 dly = 10000;
		while (dly--)
			;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_control_set);

int cpsw_ale_control_get(struct cpsw_ale *ale, int port, int control)
{
	struct ale_control_info *info = &ale_controls[control];
	int offset, shift;
	u32 tmp;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->ale_ports)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->ale_regs + offset) >> shift;
	return tmp & BITMASK(info->bits);
}
EXPORT_SYMBOL_GPL(cpsw_ale_control_get);

static ssize_t cpsw_ale_control_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, port, len = 0;
	struct ale_control_info *info;
	struct cpsw_ale *ale = control_attr_to_ale(attr);

	for (i = 0, info = ale_controls; i < ALE_NUM_CONTROLS; i++, info++) {
		/* global controls */
		if (info->port_shift == 0 &&  info->port_offset == 0) {
			len += snprintf(buf + len, SZ_4K - len,
					"%s=%d\n", info->name,
					cpsw_ale_control_get(ale, 0, i));
			continue;
		}
		/* port specific controls */
		for (port = 0; port < ale->ale_ports; port++) {
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
	char ctrl_str[33], *end;
	int port = 0, value, len, ret, control;
	struct cpsw_ale *ale = control_attr_to_ale(attr);

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;
	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		port = simple_strtoul(buf + 1, &end, 0);
		buf = end;
	}

	if (*buf != '=')
		return -EINVAL;

	value = simple_strtoul(buf + 1, NULL, 0);

	for (control = 0; control < ALE_NUM_CONTROLS; control++)
		if (strcmp(ctrl_str, ale_controls[control].name) == 0)
			break;

	if (control >= ALE_NUM_CONTROLS)
		return -ENOENT;

	dev_dbg(ale->params.dev, "processing command %s.%d=%d\n",
		ale_controls[control].name, port, value);

	ret = cpsw_ale_control_set(ale, port, control, value);
	if (ret < 0)
		return ret;
	return count;
}

DEVICE_ATTR(ale_control, S_IRUGO | S_IWUSR, cpsw_ale_control_show,
	    cpsw_ale_control_store);

static ssize_t cpsw_ale_table_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = SZ_4K, outlen = 0, idx;
	u32 ale_entry[ALE_ENTRY_WORDS];
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	for (idx = 0; idx < ale->ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen += cpsw_ale_dump_entry(idx, ale_entry, buf + outlen,
					      len - outlen);
	}
	return outlen;
}
DEVICE_ATTR(ale_table, S_IRUGO, cpsw_ale_table_show, NULL);

static void cpsw_ale_timer(unsigned long arg)
{
	struct cpsw_ale *ale = (struct cpsw_ale *)arg;

	cpsw_ale_control_set(ale, 0, ALE_AGEOUT, 1);

	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

int cpsw_ale_set_ageout(struct cpsw_ale *ale, int ageout)
{
	del_timer_sync(&ale->timer);
	ale->ageout = ageout * HZ;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
	return 0;
}

void cpsw_ale_start(struct cpsw_ale *ale)
{
	u32 rev;
	int ret;

	rev = __raw_readl(ale->ale_regs + ALE_IDVER);
	dev_dbg(ale->params.dev, "initialized cpsw ale revision %d.%d\n",
		(rev >> 8) & 0xff, rev & 0xff);
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 1);
	cpsw_ale_control_set(ale, 0, ALE_CLEAR, 1);

	ale->ale_control_attr = dev_attr_ale_control;
	sysfs_attr_init(&ale->ale_control_attr.attr);
	ret = device_create_file(ale->params.dev, &ale->ale_control_attr);
	WARN_ON(ret < 0);

	ale->ale_table_attr = dev_attr_ale_table;
	sysfs_attr_init(&ale->ale_table_attr.attr);
	ret = device_create_file(ale->params.dev, &ale->ale_table_attr);
	WARN_ON(ret < 0);

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
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 0);
	del_timer_sync(&ale->timer);
	device_remove_file(ale->params.dev, &ale->ale_table_attr);
	device_remove_file(ale->params.dev, &ale->ale_control_attr);
}
EXPORT_SYMBOL_GPL(cpsw_ale_stop);

struct cpsw_ale *cpsw_ale_create(struct cpsw_ale_params *params)
{
	struct cpsw_ale *ale;
	int ret;

	ret = -ENOMEM;
	ale = kzalloc(sizeof(*ale), GFP_KERNEL);
	if (WARN_ON(!ale))
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
	cpsw_ale_stop(ale);
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 0);
	kfree(ale);
	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_ale_destroy);

MODULE_DESCRIPTION("Ethernet Switch Address Lookup Engine driver");
MODULE_AUTHOR("Chandan Nath <chandan.nath@ti.com>");
MODULE_LICENSE("GPL");
