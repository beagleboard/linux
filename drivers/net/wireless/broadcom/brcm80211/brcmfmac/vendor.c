// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */

#include <linux/vmalloc.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

#include <brcmu_wifi.h>
#include "fwil_types.h"
#include "core.h"
#include "p2p.h"
#include "debug.h"
#include "cfg80211.h"
#include "vendor.h"
#include "fwil.h"

static int brcmf_cfg80211_vndr_cmds_dcmd_handler(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	const struct brcmf_vndr_dcmd_hdr *cmdhdr = data;
	struct sk_buff *reply;
	unsigned int payload, ret_len;
	void *dcmd_buf = NULL, *wr_pointer;
	u16 msglen, maxmsglen = PAGE_SIZE - 0x100;
	int ret;

	if (len < sizeof(*cmdhdr)) {
		brcmf_err("vendor command too short: %d\n", len);
		return -EINVAL;
	}

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	brcmf_dbg(TRACE, "ifidx=%d, cmd=%d\n", ifp->ifidx, cmdhdr->cmd);

	if (cmdhdr->offset > len) {
		brcmf_err("bad buffer offset %d > %d\n", cmdhdr->offset, len);
		return -EINVAL;
	}

	len -= cmdhdr->offset;
	ret_len = cmdhdr->len;
	if (ret_len > 0 || len > 0) {
		if (len > BRCMF_DCMD_MAXLEN) {
			brcmf_err("oversize input buffer %d\n", len);
			len = BRCMF_DCMD_MAXLEN;
		}
		if (ret_len > BRCMF_DCMD_MAXLEN) {
			brcmf_err("oversize return buffer %d\n", ret_len);
			ret_len = BRCMF_DCMD_MAXLEN;
		}
		payload = max_t(unsigned int, ret_len, len) + 1;
		dcmd_buf = vzalloc(payload);
		if (NULL == dcmd_buf)
			return -ENOMEM;

		memcpy(dcmd_buf, (void *)cmdhdr + cmdhdr->offset, len);
		*(char *)(dcmd_buf + len)  = '\0';
	}

	if (cmdhdr->cmd == BRCMF_C_SET_AP) {
		if (*(int *)(dcmd_buf) == 1) {
			ifp->vif->wdev.iftype = NL80211_IFTYPE_AP;
			brcmf_net_setcarrier(ifp, true);
		} else {
			ifp->vif->wdev.iftype = NL80211_IFTYPE_STATION;
		}
		brcmf_cfg80211_update_proto_addr_mode(&vif->wdev);
	}

	if (cmdhdr->set)
		ret = brcmf_fil_cmd_data_set(ifp, cmdhdr->cmd, dcmd_buf,
					     ret_len);
	else
		ret = brcmf_fil_cmd_data_get(ifp, cmdhdr->cmd, dcmd_buf,
					     ret_len);
	if (ret != 0)
		goto exit;

	wr_pointer = dcmd_buf;
	while (ret_len > 0) {
		msglen = ret_len > maxmsglen ? maxmsglen : ret_len;
		ret_len -= msglen;
		payload = msglen + sizeof(msglen);
		reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, payload);
		if (NULL == reply) {
			ret = -ENOMEM;
			break;
		}

		if (nla_put(reply, BRCMF_NLATTR_DATA, msglen, wr_pointer) ||
		    nla_put_u16(reply, BRCMF_NLATTR_LEN, msglen)) {
			kfree_skb(reply);
			ret = -ENOBUFS;
			break;
		}

		ret = cfg80211_vendor_cmd_reply(reply);
		if (ret)
			break;

		wr_pointer += msglen;
	}

exit:
	vfree(dcmd_buf);

	return ret;
}

static int brcmf_cfg80211_vndr_cmds_int_get(struct brcmf_if *ifp,
					    u32 cmd, struct wiphy *wiphy)
{
	struct sk_buff *reply;
	int get_value = 0;
	int ret;

	ret = brcmf_fil_cmd_int_get(ifp, cmd, &get_value);
	if (ret)
		brcmf_err("Command %u get failure. Error :  %d\n", cmd, ret);

	reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(int));
	nla_put_nohdr(reply, sizeof(int), &get_value);
	ret = cfg80211_vendor_cmd_reply(reply);
	if (ret)
		brcmf_err("Command %u failure. Error : %d\n", cmd, ret);
	return ret;
}

static int brcmf_cfg80211_vndr_cmds_int_set(struct brcmf_if *ifp, int val, u32 cmd)
{
	int ret;

	ret = brcmf_fil_cmd_int_set(ifp, cmd, val);
	if (ret < 0)
		brcmf_err("Command %u set failure. Error : %d\n", cmd, ret);
	return ret;
}

static int brcmf_cfg80211_vndr_cmds_frameburst(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data, int len)
{
	int ret;
	int val = *(int *)data;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0x0 || val == 0x1)
		ret = brcmf_cfg80211_vndr_cmds_int_set(ifp, val,
						       BRCMF_C_SET_FAKEFRAG);
	else if (val == 0xff)
		ret = brcmf_cfg80211_vndr_cmds_int_get(ifp,
						       BRCMF_C_GET_FAKEFRAG,
						       wiphy);
	else
		brcmf_err("Invalid Input\n");

	return ret;
}

s32
brcmf_wiphy_phy_temp_evt_handler(struct brcmf_if *ifp,
				 const struct brcmf_event_msg *e, void *data)

{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct sk_buff *skb;
	struct nlattr *phy_temp_data;
	u32 version, temp, tempdelta;
	struct brcmf_phy_temp_evt *phy_temp_evt;

	phy_temp_evt = (struct brcmf_phy_temp_evt *)data;

	version = le32_to_cpu(phy_temp_evt->version);
	temp = le32_to_cpu(phy_temp_evt->temp);
	tempdelta = le32_to_cpu(phy_temp_evt->tempdelta);

	skb = cfg80211_vendor_event_alloc(wiphy, NULL,
					  sizeof(*phy_temp_evt),
					  BRCMF_VNDR_EVTS_PHY_TEMP,
					  GFP_KERNEL);

	if (!skb) {
		brcmf_dbg(EVENT, "NO MEM: can't allocate skb for vendor PHY_TEMP_EVENT\n");
		return -ENOMEM;
	}

	phy_temp_data = nla_nest_start(skb, NL80211_ATTR_VENDOR_EVENTS);
	if (!phy_temp_data) {
		nla_nest_cancel(skb, phy_temp_data);
		kfree_skb(skb);
		brcmf_dbg(EVENT, "skb could not nest vendor attributes\n");
		return -EMSGSIZE;
	}

	if (nla_put_u32(skb, BRCMF_NLATTR_VERS, version) ||
	    nla_put_u32(skb, BRCMF_NLATTR_PHY_TEMP, temp) ||
	    nla_put_u32(skb, BRCMF_NLATTR_PHY_TEMPDELTA, tempdelta)) {
		kfree_skb(skb);
		brcmf_dbg(EVENT, "NO ROOM in skb for vendor PHY_TEMP_EVENT\n");
		return -EMSGSIZE;
	}

	nla_nest_end(skb, phy_temp_data);

	cfg80211_vendor_event(skb, GFP_KERNEL);
	return 0;
}

const struct wiphy_vendor_command brcmf_vendor_cmds[] = {
	{
		{
			.vendor_id = BROADCOM_OUI,
			.subcmd = BRCMF_VNDR_CMDS_DCMD
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.policy = VENDOR_CMD_RAW_DATA,
		.doit = brcmf_cfg80211_vndr_cmds_dcmd_handler
	},
	{
		{
			.vendor_id = BROADCOM_OUI,
			.subcmd = BRCMF_VNDR_CMDS_FRAMEBURST
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.policy = VENDOR_CMD_RAW_DATA,
		.doit = brcmf_cfg80211_vndr_cmds_frameburst
	},
};

const struct nl80211_vendor_cmd_info brcmf_vendor_events[] = {
	{
		.vendor_id = BROADCOM_OUI,
		.subcmd = BRCMF_VNDR_EVTS_PHY_TEMP,
	},
};
