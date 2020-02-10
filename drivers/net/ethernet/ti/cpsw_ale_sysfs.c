// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments netcp ethss ale sysfs interface
 *
 * Copyright (C) 2016-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 */
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS		DIV_ROUND_UP(ALE_ENTRY_BITS, 32)
#define ALE_TBL_ENTRY_SHOW_LEN		160
#define ALE_RAW_TBL_ENTRY_SHOW_LEN	32
#define ALE_VERSION_9R3		0x0903
#define ALE_TBL_ENTRY_SHOW_LEN		160
#define ALE_RAW_TBL_ENTRY_SHOW_LEN	32

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
	} else {
		cpsw_ale_get_vlan_mcast(ale, ale_entry, &reg_mc_fld,
					&unreg_mc_fld);
	}

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

static ssize_t ale_control_show(struct device *dev,
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
			if (i >= ALE_PORT_UNKNOWN_VLAN_MEMBER &&
			    i <= ALE_PORT_UNTAGGED_EGRESS)
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

static ssize_t ale_control_store(struct device *dev,
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
DEVICE_ATTR_RW(ale_control);

static ssize_t ale_table_show(struct device *dev,
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

static void cpsw_ale_table_store_init_params(struct ale_table_param *params,
					     int param_num)
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

static ssize_t ale_table_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	return cpsw_ale_table_store_proc(ale, buf, count);
}
DEVICE_ATTR_RW(ale_table);

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

static ssize_t ale_table_raw_show(struct device *dev,
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

static ssize_t ale_table_raw_store(struct device *dev,
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
DEVICE_ATTR_RW(ale_table_raw);

static void cpsw_ale_create_sysfs_entries(struct cpsw_ale *ale)
{
	int ret, i;

	if (ale->version != ALE_VERSION_1R3 &&
	    !ale->params.nu_switch_ale &&
	    ale->version != ALE_VERSION_9R3)
		return;

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

static void cpsw_ale_remove_sysfs_entries(struct cpsw_ale *ale)
{
	if (ale->version != ALE_VERSION_1R3 &&
	    !ale->params.nu_switch_ale &&
	    ale->version != ALE_VERSION_9R3)
		return;

	device_remove_file(ale->params.dev, &ale->ale_table_attr);
	device_remove_file(ale->params.dev, &ale->ale_control_attr);
	device_remove_file(ale->params.dev, &ale->ale_table_raw_attr);
}
