// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <net/genetlink.h>

#include "cc33xx.h"
#include "acx.h"
#include "io.h"
#include "testmode.h"

#define CC33XX_TM_MAX_DATA_LENGTH 1024

enum cc33xx_tm_commands {
	CC33XX_TM_CMD_UNSPEC,
	CC33XX_TM_CMD_TEST,
	CC33XX_TM_CMD_INTERROGATE,
	CC33XX_TM_CMD_CONFIGURE,
	CC33XX_TM_CMD_NVS_PUSH,		/* Not in use. Keep to not break ABI */
	CC33XX_TM_CMD_SET_PLT_MODE,
	CC33XX_TM_CMD_RECOVER,		/* Not in use. Keep to not break ABI */
	CC33XX_TM_CMD_GET_MAC,

	__CC33XX_TM_CMD_AFTER_LAST
};

enum cc33xx_tm_attrs {
	CC33XX_TM_ATTR_UNSPEC,
	CC33XX_TM_ATTR_CMD_ID,
	CC33XX_TM_ATTR_ANSWER,
	CC33XX_TM_ATTR_DATA,
	CC33XX_TM_ATTR_IE_ID,
	CC33XX_TM_ATTR_PLT_MODE,

	__CC33XX_TM_ATTR_AFTER_LAST
};

#define CC33XX_TM_ATTR_MAX (__CC33XX_TM_ATTR_AFTER_LAST - 1)

static struct nla_policy cc33xx_tm_policy[CC33XX_TM_ATTR_MAX + 1] = {
	[CC33XX_TM_ATTR_CMD_ID] =	{ .type = NLA_U32 },
	[CC33XX_TM_ATTR_ANSWER] =	{ .type = NLA_U8 },
	[CC33XX_TM_ATTR_DATA] =		{ .type = NLA_BINARY,
					  .len = CC33XX_TM_MAX_DATA_LENGTH },
	[CC33XX_TM_ATTR_IE_ID] =	{ .type = NLA_U32 },
	[CC33XX_TM_ATTR_PLT_MODE] =	{ .type = NLA_U32 },
};

static int cc33xx_tm_cmd_test(struct cc33xx *cc, struct nlattr *tb[])
{
	int ret, len;
	u16 buf_len;
	struct sk_buff *skb;
	void *buf;
	u8 answer = 0;

	cc33xx_debug(DEBUG_TESTMODE, "testmode cmd test");

	if (!tb[CC33XX_TM_ATTR_DATA])
		return -EINVAL;

	buf = nla_data(tb[CC33XX_TM_ATTR_DATA]);
	buf_len = nla_len(tb[CC33XX_TM_ATTR_DATA]);

	if (tb[CC33XX_TM_ATTR_ANSWER])
		answer = nla_get_u8(tb[CC33XX_TM_ATTR_ANSWER]);

	if (buf_len > sizeof(struct cc33xx_command))
		return -EMSGSIZE;

	mutex_lock(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		ret = -EINVAL;
		goto out;
	}

	ret = cc33xx_cmd_test(cc, buf, buf_len, answer);
	if (ret < 0) {
		cc33xx_warning("testmode cmd test failed: %d", ret);
		goto out;
	}

	if (answer) {
		/* If we got bip calibration answer print radio status */
		struct cc33xx_cmd_cal_p2g *params =
			(struct cc33xx_cmd_cal_p2g *)buf;
		s16 radio_status = (s16)le16_to_cpu(params->radio_status);

		if (params->test.id == TEST_CMD_P2G_CAL && radio_status < 0)
			cc33xx_warning("testmode cmd: radio status=%d",
				       radio_status);
		else
			cc33xx_info("testmode cmd: radio status=%d",
				    radio_status);

		len = nla_total_size(buf_len);
		skb = cfg80211_testmode_alloc_reply_skb(cc->hw->wiphy, len);
		if (!skb) {
			ret = -ENOMEM;
			goto out;
		}

		if (nla_put(skb, CC33XX_TM_ATTR_DATA, buf_len, buf)) {
			kfree_skb(skb);
			ret = -EMSGSIZE;
			goto out;
		}

		ret = cfg80211_testmode_reply(skb);
	}

out:
	mutex_unlock(&cc->mutex);

	return ret;
}

static int cc33xx_tm_cmd_interrogate(struct cc33xx *cc, struct nlattr *tb[])
{
	int ret;
	struct cc33xx_command *cmd;
	struct sk_buff *skb;
	u8 ie_id;

	cc33xx_debug(DEBUG_TESTMODE, "testmode cmd interrogate");

	if (!tb[CC33XX_TM_ATTR_IE_ID])
		return -EINVAL;

	ie_id = nla_get_u8(tb[CC33XX_TM_ATTR_IE_ID]);

	cc33xx_debug(DEBUG_TESTMODE, "testmode cmd interrogate id %d", ie_id);

	mutex_lock(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		ret = -EINVAL;
		goto out;
	}

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_debug_inter(cc, ie_id, cmd,
				     sizeof(struct acx_header), sizeof(*cmd));
	if (ret < 0) {
		cc33xx_warning("testmode cmd interrogate failed: %d", ret);
		goto out_free;
	}

	skb = cfg80211_testmode_alloc_reply_skb(cc->hw->wiphy, sizeof(*cmd));
	if (!skb) {
		ret = -ENOMEM;
		goto out_free;
	}

	if (nla_put(skb, CC33XX_TM_ATTR_DATA, sizeof(*cmd), cmd)) {
		kfree_skb(skb);
		ret = -EMSGSIZE;
		goto out_free;
	}

	ret = cfg80211_testmode_reply(skb);
	if (ret < 0)
		goto out_free;

out_free:
	kfree(cmd);

out:
	mutex_unlock(&cc->mutex);

	return ret;
}

static int cc33xx_tm_cmd_configure(struct cc33xx *cc, struct nlattr *tb[])
{
	int ret;
	u16 buf_len;
	void *buf;
	u8 ie_id;

	cc33xx_debug(DEBUG_TESTMODE, "testmode cmd configure");

	if (!tb[CC33XX_TM_ATTR_DATA])
		return -EINVAL;
	if (!tb[CC33XX_TM_ATTR_IE_ID])
		return -EINVAL;

	ie_id = nla_get_u8(tb[CC33XX_TM_ATTR_IE_ID]);
	buf = nla_data(tb[CC33XX_TM_ATTR_DATA]);
	buf_len = nla_len(tb[CC33XX_TM_ATTR_DATA]);

	if (buf_len > sizeof(struct cc33xx_command))
		return -EMSGSIZE;

	mutex_lock(&cc->mutex);
	ret = cc33xx_cmd_debug(cc, ie_id, buf, buf_len);
	mutex_unlock(&cc->mutex);

	if (ret < 0) {
		cc33xx_warning("testmode cmd configure failed: %d", ret);
		return ret;
	}

	return 0;
}

static int cc33xx_tm_detect_fem(struct cc33xx *cc, struct nlattr *tb[])
{
	/* return FEM type */
	int ret, len;
	struct sk_buff *skb;

	ret = cc33xx_plt_start(cc, PLT_FEM_DETECT);
	if (ret < 0)
		goto out;

	mutex_lock(&cc->mutex);

	len = nla_total_size(sizeof(cc->fem_manuf));
	skb = cfg80211_testmode_alloc_reply_skb(cc->hw->wiphy, len);
	if (!skb) {
		ret = -ENOMEM;
		goto out_mutex;
	}

	if (nla_put(skb, CC33XX_TM_ATTR_DATA, sizeof(cc->fem_manuf),
		    &cc->fem_manuf)) {
		kfree_skb(skb);
		ret = -EMSGSIZE;
		goto out_mutex;
	}

	ret = cfg80211_testmode_reply(skb);

out_mutex:
	mutex_unlock(&cc->mutex);

	/* We always stop plt after DETECT mode */
	cc33xx_plt_stop(cc);
out:
	return ret;
}

static int cc33xx_tm_cmd_set_plt_mode(struct cc33xx *cc, struct nlattr *tb[])
{
	u32 val;
	int ret;

	cc33xx_debug(DEBUG_TESTMODE, "testmode cmd set plt mode");

	if (!tb[CC33XX_TM_ATTR_PLT_MODE])
		return -EINVAL;

	val = nla_get_u32(tb[CC33XX_TM_ATTR_PLT_MODE]);

	switch (val) {
	case PLT_OFF:
		ret = cc33xx_plt_stop(cc);
		break;
	case PLT_ON:
	case PLT_CHIP_AWAKE:
		ret = cc33xx_plt_start(cc, val);
		break;
	case PLT_FEM_DETECT:
		ret = cc33xx_tm_detect_fem(cc, tb);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int cc33xx_tm_cmd_get_mac(struct cc33xx *cc, struct nlattr *tb[])
{
	struct sk_buff *skb;
	u8 zero_mac[ETH_ALEN] = {0};
	int ret = 0;

	mutex_lock(&cc->mutex);

	if (!cc->plt) {
		ret = -EINVAL;
		goto out;
	}

	if (memcmp(zero_mac, cc->efuse_mac_address, ETH_ALEN) == 0) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	skb = cfg80211_testmode_alloc_reply_skb(cc->hw->wiphy, ETH_ALEN);
	if (!skb) {
		ret = -ENOMEM;
		goto out;
	}

	if (nla_put(skb, CC33XX_TM_ATTR_DATA,
		    ETH_ALEN, cc->efuse_mac_address)) {
		kfree_skb(skb);
		ret = -EMSGSIZE;
		goto out;
	}

	ret = cfg80211_testmode_reply(skb);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&cc->mutex);
	return ret;
}

int cc33xx_tm_cmd(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  void *data, int len)
{
	struct cc33xx *cc = hw->priv;
	struct nlattr *tb[CC33XX_TM_ATTR_MAX + 1];
	u32 nla_cmd;
	int err;

	err = nla_parse_deprecated(tb, CC33XX_TM_ATTR_MAX, data, len,
				   cc33xx_tm_policy, NULL);
	if (err)
		return err;

	if (!tb[CC33XX_TM_ATTR_CMD_ID])
		return -EINVAL;

	nla_cmd = nla_get_u32(tb[CC33XX_TM_ATTR_CMD_ID]);

	/* Only SET_PLT_MODE is allowed in case of mode PLT_CHIP_AWAKE */
	if (cc->plt_mode == PLT_CHIP_AWAKE &&
	    nla_cmd != CC33XX_TM_CMD_SET_PLT_MODE)
		return -EOPNOTSUPP;

	switch (nla_cmd) {
	case CC33XX_TM_CMD_TEST:
		return cc33xx_tm_cmd_test(cc, tb);
	case CC33XX_TM_CMD_INTERROGATE:
		return cc33xx_tm_cmd_interrogate(cc, tb);
	case CC33XX_TM_CMD_CONFIGURE:
		return cc33xx_tm_cmd_configure(cc, tb);
	case CC33XX_TM_CMD_SET_PLT_MODE:
		return cc33xx_tm_cmd_set_plt_mode(cc, tb);
	case CC33XX_TM_CMD_GET_MAC:
		return cc33xx_tm_cmd_get_mac(cc, tb);
	default:
		return -EOPNOTSUPP;
	}
}
