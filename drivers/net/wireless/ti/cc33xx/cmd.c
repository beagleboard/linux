// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "wlcore.h"
#include "debug.h"
#include "io.h"
#include "acx.h"
#include "cc33xx_80211.h"
#include "cmd.h"
#include "event.h"
#include "tx.h"

#define CC33XX_REBOOT_TIMEOUT_MSEC		100

#define CC33XX_CMD_FAST_POLL_COUNT       50
#define CC33XX_WAIT_EVENT_FAST_POLL_COUNT 20
static inline void init_cmd_header(
	struct cc33xx_cmd_header* header,
	size_t cmd_len,
	u16 id)
{
	header->NAB_header.len = cpu_to_le16(cmd_len);
	BUG_ON(header->NAB_header.len != cmd_len);

	header->NAB_header.sync_pattern = cpu_to_le32(HOST_SYNC_PATTERN);
	header->NAB_header.opcode = cpu_to_le16(id);
}
int cc33xx_set_max_buffer_size(struct cc33xx *wl, BufferSize_e max_buffer_size)
{
	switch(max_buffer_size)
	{
		case INI_MAX_BUFFER_SIZE:
			wl->max_cmd_size = CC33XX_INI_CMD_MAX_SIZE + sizeof(struct cc33xx_cmd_ini_params_download)+ sizeof(u32);//INI FILE PAYLOAD SIZE + INI CMD PARAM + INT
			break;

		case CMD_MAX_BUFFER_SIZE:
			wl->max_cmd_size = CC33XX_CMD_MAX_SIZE;
			break;
		default:
			cc33xx_warning("max_buffer_size invalid, not changing buffer size");
			break;
	}
	return 0;

}

static int send_buffer(struct cc33xx *wl, int cmd_box_addr,
			       void *buf, size_t len)
{
	size_t max_cmd_size_align;
	memcpy(wl->cmd_buf, buf, len);

	memset(wl->cmd_buf + len, 0, (CC33XX_CMD_BUFFER_SIZE) - len);

	max_cmd_size_align = __ALIGN_MASK(wl->max_cmd_size,CC33XX_BUS_BLOCK_SIZE*2 - 1);

	return wlcore_write(wl, cmd_box_addr, wl->cmd_buf,
				(max_cmd_size_align), true);
}

/*
 * send command to firmware
 *
 * @wl: wl struct
 * @id: command id
 * @buf: buffer containing the command, must work with dma
 * @len: length of the buffer
 * return the cmd status code on success.
 */
static int __wlcore_cmd_send(struct cc33xx *wl, u16 id, void *buf,
			     size_t len, size_t res_len, bool sync)
{
	struct cc33xx_cmd_header *cmd;
	unsigned long timeout;
	int ret;

	if (id >= CMD_LAST_SUPPORTED_COMMAND)
	{
	    cc33xx_debug(DEBUG_CMD, "command ID: %d, blocked",id);
	    return(CMD_STATUS_SUCCESS);
	}

	if (unlikely(wl->state == WLCORE_STATE_RESTARTING &&
		     id != CMD_STOP_FWLOGGER))
		return -EIO;

	if (WARN_ON_ONCE(len < sizeof(*cmd)))
		return -EIO;

	BUG_ON(len > wl->max_cmd_size);

	cmd = buf;
	cmd->id = cpu_to_le16(id);
	cmd->status = 0;

	WARN_ON(len % 4 != 0);
	WARN_ON(test_bit(CC33XX_FLAG_IN_ELP, &wl->flags));
	init_cmd_header(cmd, len, id);
	init_completion (&wl->command_complete);
	ret = send_buffer(wl, NAB_DATA_ADDR, buf, len);
	if (ret < 0)
		return ret;

	if (unlikely(!sync))
		return CMD_STATUS_SUCCESS;
	timeout = msecs_to_jiffies(CC33XX_COMMAND_TIMEOUT);
	ret = wait_for_completion_timeout(
		&wl->command_complete, timeout);
	if (ret < 1)
	{
		cc33xx_debug(DEBUG_CMD, "Command T.O");
		return -EIO;
	}

	switch(id){
	case CMD_INTERROGATE:
	case CMD_DEBUG_READ:
	case CMD_TEST_MODE:
	case CMD_BM_READ_DEVICE_INFO:
		cc33xx_debug(DEBUG_CMD,
			"Response len %d, allocated buffer len %d",
			wl->result_length,
			res_len);

		if (!res_len)
			break; // Response should be discarded

		if (WARN_ON(wl->result_length > res_len)){
			cc33xx_error("Error, insufficient response buffer");
			break;
		}

		memcpy(buf + sizeof(struct NAB_header),
			wl->command_result,
			wl->result_length);

		break;

	default:
		break;
	}

	return CMD_STATUS_SUCCESS;
}

/*
 * send command to fw and return cmd status on success
 * valid_rets contains a bitmap of allowed error codes
 */
static int wlcore_cmd_send_failsafe(struct cc33xx *wl, u16 id, void *buf,
				    size_t len, size_t res_len,
				    unsigned long valid_rets)
{
	int ret = __wlcore_cmd_send(wl, id, buf, len, res_len, true);


	cc33xx_debug(DEBUG_TESTMODE, "CMD# %d, len=%d", id, len);

	if (ret < 0)
		goto fail;

	/* success is always a valid status */
	valid_rets |= BIT(CMD_STATUS_SUCCESS);

	if (ret >= MAX_COMMAND_STATUS ||
	    !test_bit(ret, &valid_rets)) {
		cc33xx_error("command execute failure %d", ret);
		ret = -EIO;
		//goto fail;
	}
	return ret;
fail:
	cc33xx_queue_recovery_work(wl);
	return ret;
}

/*
 * wrapper for wlcore_cmd_send that accept only CMD_STATUS_SUCCESS
 * return 0 on success.
 */
int cc33xx_cmd_send(struct cc33xx *wl, u16 id, void *buf, size_t len,
		    size_t res_len)
{
	int ret;
	/* Support id */
	switch((enum cc33xx_cmd)id)
	{
		case CMD_EMPTY:
		case CMD_CHANNEL_SWITCH          :
		case CMD_STOP_CHANNEL_SWICTH     :
		case CMD_START_DHCP_MGMT_SEQ     :
		case CMD_STOP_DHCP_MGMT_SEQ      :
		case CMD_START_SECURITY_MGMT_SEQ :
		case CMD_STOP_SECURITY_MGMT_SEQ  :
		case CMD_START_ARP_MGMT_SEQ      :
		case CMD_STOP_ARP_MGMT_SEQ       :
		case CMD_START_DNS_MGMT_SEQ      :
		case CMD_STOP_DNS_MGMT_SEQ       :
		case CMD_SEND_DEAUTH_DISASSOC    :
		case CMD_SCHED_STATE_EVENT	 :
		{
			return 0;
		}break;
		default:
		{
			if ( (enum cc33xx_cmd)id >= CMD_LAST_SUPPORTED_COMMAND)
				return 0;
			goto send;
		}
	}
send:
	ret = wlcore_cmd_send_failsafe(wl, id, buf, len, res_len, 0);
	if (ret < 0)
		return ret;
	return 0;
}

int cc33xx_count_role_set_bits(unsigned long role_map)
{
	int count = 0;
	// if device bit is set ( BIT_2 = ROLE_DEVICE)
	// since role device is not counted
	// remove it from map
	role_map &= ~BIT(2);

	while (role_map != 0)
	{
		count += role_map & 1;
		role_map >>= 1;
	}
	return count;
}

int cc33xx_cmd_role_enable(struct cc33xx *wl, u8 *addr, u8 role_type,
			   u8 *role_id)
{
	struct cc33xx_cmd_role_enable *cmd;

	int ret;
	unsigned long role_count;


	struct cc33xx_cmd_complete_role_enable *command_complete =
		  (struct cc33xx_cmd_complete_role_enable *)&wl->command_result;

	role_count = *wl->roles_map;
        ret = cc33xx_count_role_set_bits(role_count);
	cc33xx_debug(DEBUG_CMD, "cmd roles enabled: bitmap before: %ld, ret=%d", role_count, ret);

	//do not enable more than 2 roles at once, exception is device role
	if ((ret >= 2) && (CC33XX_ROLE_DEVICE != role_type))
	{
		cc33xx_debug(DEBUG_CMD, "cmd role enable: 2 roles already have beed allocated");
		cc33xx_error("failed to initiate cmd role enable");
		ret = -EBUSY;
		goto out;
	}
	cc33xx_debug(DEBUG_CMD, "cmd role enable, role type %d, addr = %pM", role_type, addr);

	if (WARN_ON(*role_id != CC33XX_INVALID_ROLE_ID))
		return -EBUSY;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(cmd->mac_address, addr, ETH_ALEN);
	cmd->role_type = role_type;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_ENABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role enable");
		goto out_free;
	}
	cc33xx_debug(DEBUG_CMD, "complete role_id = %d",
						      command_complete->role_id);
	__set_bit(command_complete->role_id, wl->roles_map);
	*role_id = command_complete->role_id;

out_free:
	kfree(cmd);
out:
	return ret;
}

int cc33xx_cmd_role_disable(struct cc33xx *wl, u8 *role_id)
{
	struct cc33xx_cmd_role_disable *cmd;
	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd role disable");

	if (WARN_ON(*role_id == CC33XX_INVALID_ROLE_ID))
		return -ENOENT;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}
	cmd->role_id = *role_id;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_DISABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role disable");
		goto out_free;
	}

	__clear_bit(*role_id, wl->roles_map);
	*role_id = CC33XX_INVALID_ROLE_ID;

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_set_link(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 link)
{
	unsigned long flags;

	/* these bits are used by op_tx */
	spin_lock_irqsave(&wl->wl_lock, flags);
	__set_bit(link, wl->links_map);
	__set_bit(link, wlvif->links_map);
	spin_unlock_irqrestore(&wl->wl_lock, flags);
	wl->links[link].wlvif = wlvif;

	/*
	 * Take saved value for total freed packets from wlvif, in case this is
	 * recovery/resume
	 */
	if (wlvif->bss_type != BSS_TYPE_AP_BSS)
		wl->links[link].total_freed_pkts = wlvif->total_freed_pkts;

	wl->active_link_count++;
	return 0;
}

void cc33xx_clear_link(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 *hlid)
{
	unsigned long flags;

	if (*hlid == CC33XX_INVALID_LINK_ID)
		return;

	/* these bits are used by op_tx */
	spin_lock_irqsave(&wl->wl_lock, flags);
	__clear_bit(*hlid, wl->links_map);
	__clear_bit(*hlid, wlvif->links_map);
	spin_unlock_irqrestore(&wl->wl_lock, flags);

	wl->links[*hlid].prev_freed_pkts = 0;
	wl->links[*hlid].ba_bitmap = 0;
	eth_zero_addr(wl->links[*hlid].addr);

	/*
	 * At this point op_tx() will not add more packets to the queues. We
	 * can purge them.
	 */
	cc33xx_tx_reset_link_queues(wl, *hlid);
	wl->links[*hlid].wlvif = NULL;

	if (wlvif->bss_type == BSS_TYPE_AP_BSS &&
	    *hlid == wlvif->ap.bcast_hlid) {
		u32 sqn_padding = CC33XX_TX_SQN_POST_RECOVERY_PADDING;
		/*
		 * save the total freed packets in the wlvif, in case this is
		 * recovery or suspend
		 */
		wlvif->total_freed_pkts = wl->links[*hlid].total_freed_pkts;

		/*
		 * increment the initial seq number on recovery to account for
		 * transmitted packets that we haven't yet got in the FW status
		 */
		if (wlvif->encryption_type == KEY_GEM)
			sqn_padding = CC33XX_TX_SQN_POST_RECOVERY_PADDING_GEM;

		if (test_bit(CC33XX_FLAG_RECOVERY_IN_PROGRESS, &wl->flags))
			wlvif->total_freed_pkts += sqn_padding;
	}

	wl->links[*hlid].total_freed_pkts = 0;

	*hlid = CC33XX_INVALID_LINK_ID;
	wl->active_link_count--;
	WARN_ON_ONCE(wl->active_link_count < 0);
}

u8 wlcore_get_native_channel_type(u8 nl_channel_type)
{
	switch (nl_channel_type) {
	case NL80211_CHAN_NO_HT:
		return WLCORE_CHAN_NO_HT;
	case NL80211_CHAN_HT20:
		return WLCORE_CHAN_HT20;
	case NL80211_CHAN_HT40MINUS:
		return WLCORE_CHAN_HT40MINUS;
	case NL80211_CHAN_HT40PLUS:
		return WLCORE_CHAN_HT40PLUS;
	default:
		WARN_ON(1);
		return WLCORE_CHAN_NO_HT;
	}
}

int cc33xx_cmd_role_start_dev(struct cc33xx *wl,
				     struct cc33xx_vif *wlvif,
				     enum nl80211_band band,
				     int channel)
{
	struct cc33xx_cmd_role_start *cmd;
	int ret;

	struct cc33xx_cmd_complete_role_start *command_complete =
	          (struct cc33xx_cmd_complete_role_start *)&wl->command_result;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role start dev %d", wlvif->dev_role_id);

	cmd->role_id = wlvif->dev_role_id;
	cmd->role_type = CC33XX_ROLE_DEVICE;
	if (band == NL80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;
	cmd->channel = channel;
	cmd->channel_type = wlcore_get_native_channel_type(wlvif->channel_type);

	ret = cc33xx_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role device start");
		goto err_hlid;
	}

	wlvif->dev_hlid = command_complete->sta.hlid;
	wl->links[wlvif->dev_hlid].allocated_pkts = 0;
	wl->session_ids[wlvif->dev_hlid] = command_complete->sta.session;
	cc33xx_debug(DEBUG_CMD, "role start: roleid=%d, hlid=%d, session=%d ",
             wlvif->dev_role_id, command_complete->sta.hlid, command_complete->sta.session);
	ret = cc33xx_set_link(wl, wlvif, wlvif->dev_hlid);
	goto out_free;

err_hlid:
	/* clear links on error */
	cc33xx_clear_link(wl, wlvif, &wlvif->dev_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_role_stop_transceiver(struct cc33xx *wl)
{
	struct cc33xx_cmd_role_stop *cmd;
	int ret;

	if (unlikely((wl->state != WLCORE_STATE_ON) || (!wl->plt))) {
		ret = -EINVAL;
		goto out;
	}

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}
	cc33xx_debug(DEBUG_CMD, "cmd role stop transceiver");

	cmd->role_id = wl->plt_role_id;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("transceiver - failed to initiate cmd role stop");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}


int cc33xx_cmd_plt_disable(struct cc33xx *wl)
{
	struct cc33xx_cmd_PLT_disable *cmd;
	int ret;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_send(wl, CMD_PLT_DISABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("transceiver: failed to disable Transceiver mode");
		goto out_free;
	}
	else
	{
		cc33xx_debug(DEBUG_CMD, "Succeed to disable Transceiver mode");
	}

	out_free:
	kfree(cmd);

	out:
	return ret;
}

static int cc333xx_cmd_role_stop_dev(struct cc33xx *wl,
				    struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd;
	int ret;

	if (WARN_ON(wlvif->dev_hlid == CC33XX_INVALID_LINK_ID))
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role stop dev");

	cmd->role_id = wlvif->dev_role_id;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role stop");
		goto out_free;
	}

	cc33xx_clear_link(wl, wlvif, &wlvif->dev_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}


int cc33xx_cmd_plt_enable(struct cc33xx *wl, u8 role_id)
{
	struct cc33xx_cmd_PLT_enable *cmd;
	int32_t ret;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_send(wl, CMD_PLT_ENABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("Failed to send CMD_PLT_ENABLE");
		goto out_free;
	}
	cc33xx_debug(DEBUG_CMD, "Success to send CMD_PLT_ENABLE");

out_free:
	kfree(cmd);
out:
	return ret;

}

int cc33xx_cmd_role_start_transceiver(struct cc33xx *wl, u8 role_id)
{
	struct cc33xx_cmd_role_start *cmd;
	int32_t ret;
	u8 role_type = ROLE_TRANSCEIVER;

	/* Default values */
	u8 band = NL80211_BAND_2GHZ;
	u8 channel = 6;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

    cmd->role_type = role_type;
	cmd->role_id = role_id;
	cmd->channel = channel;
	cmd->band = band;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role start PLT");
		goto out_free;
	}
	cc33xx_debug(DEBUG_CMD, "cmd role start PLT. Role ID number: %u", role_id);

out_free:
	kfree(cmd);
out:
	return ret;
}

int cc33xx_cmd_role_start_sta(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	struct cc33xx_cmd_role_start *cmd;

	u32 supported_rates;
	int ret;

	struct cc33xx_cmd_complete_role_start *command_complete =
	          (struct cc33xx_cmd_complete_role_start *)&wl->command_result;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role start sta %d", wlvif->role_id);

	cmd->role_id = wlvif->role_id;
	cmd->role_type = CC33XX_ROLE_STA;
	if (wlvif->band == NL80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;
	cmd->channel = wlvif->channel;
	if (wlvif->band == NL80211_BAND_5GHZ)
		cmd->sta.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set & ~CONF_TX_CCK_RATES);
	else
		cmd->sta.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set);
	cmd->sta.beacon_interval = cpu_to_le16(wlvif->beacon_int);
	cmd->sta.ssid_type = CC33XX_SSID_TYPE_ANY;
	cmd->sta.ssid_len = wlvif->ssid_len;
	memcpy(cmd->sta.ssid, wlvif->ssid, wlvif->ssid_len);
	memcpy(cmd->sta.bssid, vif->bss_conf.bssid, ETH_ALEN);

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES | wlvif->rate_set;
	if (wlvif->band == NL80211_BAND_5GHZ)
		supported_rates &= ~CONF_TX_CCK_RATES;

	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;

	cmd->sta.local_rates = cpu_to_le32(supported_rates);

	cmd->channel_type = wlcore_get_native_channel_type(wlvif->channel_type);

	/*
	 * We don't have the correct remote rates in this stage.  The
	 * rates will be reconfigured later, after association, if the
	 * firmware supports ACX_PEER_CAP.  Otherwise, there's nothing
	 * we can do, so use all supported_rates here.
	 */
	cmd->sta.remote_rates = cpu_to_le32(supported_rates);

	ret = cc33xx_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role start sta");
		goto err_hlid;
	}

	wlvif->sta.role_chan_type = wlvif->channel_type;

	wlvif->sta.hlid = command_complete->sta.hlid;
	wl->links[wlvif->sta.hlid].allocated_pkts = 0;
	wl->session_ids[wlvif->sta.hlid] = command_complete->sta.session;
	cc33xx_debug(DEBUG_CMD, "role start: roleid=%d, hlid=%d, session=%d "
             "basic_rate_set: 0x%x, remote_rates: 0x%x",
             wlvif->role_id, command_complete->sta.hlid, command_complete->sta.session,
             wlvif->basic_rate_set, wlvif->rate_set);
	ret = cc33xx_set_link(wl, wlvif, wlvif->sta.hlid);

	goto out_free;

err_hlid:

	cc33xx_clear_link(wl, wlvif, &wlvif->sta.hlid);

out_free:
	kfree(cmd);
out:
	return ret;
}

/* use this function to stop ibss as well */
int cc33xx_cmd_role_stop_sta(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd;
	int ret;

	if (WARN_ON(wlvif->sta.hlid == CC33XX_INVALID_LINK_ID))
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role stop sta %d", wlvif->role_id);

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role stop sta");
		goto out_free;
	}

	cc33xx_clear_link(wl, wlvif, &wlvif->sta.hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_role_start_ap(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_start *cmd;
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	struct ieee80211_bss_conf *bss_conf = &vif->bss_conf;
	u32 supported_rates;
	int ret;

	struct cc33xx_cmd_complete_role_start *command_complete =
	          (struct cc33xx_cmd_complete_role_start *)&wl->command_result;


	cc33xx_debug(DEBUG_CMD, "cmd role start ap %d", wlvif->role_id);
	cc33xx_debug(DEBUG_CMD, "cmd role start ap basic rateset:  0x%x ", wlvif->basic_rate_set);

	/* If MESH --> ssid_len is always 0 */
	if (!ieee80211_vif_is_mesh(vif)) {
		/* trying to use hidden SSID with an old hostapd version */
		if (wlvif->ssid_len == 0 && !bss_conf->hidden_ssid) {
			cc33xx_error("got a null SSID from beacon/bss");
			ret = -EINVAL;
			goto out;
		}
	}

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->role_id = wlvif->role_id;
	cmd->role_type = CC33XX_ROLE_AP;
	cmd->ap.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set);
	cmd->ap.beacon_interval = cpu_to_le16(wlvif->beacon_int);
	cmd->ap.dtim_interval = bss_conf->dtim_period;
	cmd->ap.wmm = wlvif->wmm_enabled;
	cmd->channel = wlvif->channel;
	cmd->channel_type = wlcore_get_native_channel_type(wlvif->channel_type);

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES ;
	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;

	cc33xx_debug(DEBUG_CMD, "cmd role start ap with supported_rates 0x%08x",
		     supported_rates);

	cmd->ap.local_rates = cpu_to_le32(supported_rates);

	switch (wlvif->band) {
	case NL80211_BAND_2GHZ:
		cmd->band = WLCORE_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		cmd->band = WLCORE_BAND_5GHZ;
		break;
	default:
		cc33xx_warning("ap start - unknown band: %d", (int)wlvif->band);
		cmd->band = WLCORE_BAND_2_4GHZ;
		break;
	}

	ret = cc33xx_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role start ap");
		goto out_free_bcast;
	}

	wlvif->ap.global_hlid = command_complete->ap.global_hlid;
	wlvif->ap.bcast_hlid  = command_complete->ap.broadcast_hlid;
	wl->session_ids[wlvif->ap.global_hlid] = command_complete->ap.global_session_id;
	wl->session_ids[wlvif->ap.bcast_hlid] = command_complete->ap.bcast_session_id;

	cc33xx_debug(DEBUG_CMD, "role start: roleid=%d, global_hlid=%d, "
	     "broadcast_hlid=%d global_session_id=%d, bcast_session_id=%d "
             "basic_rate_set: 0x%x, remote_rates: 0x%x",
             wlvif->role_id, command_complete->ap.global_hlid,
	     command_complete->ap.broadcast_hlid, command_complete->ap.global_session_id,
	     command_complete->ap.bcast_session_id, wlvif->basic_rate_set, wlvif->rate_set);

	ret = cc33xx_set_link(wl, wlvif, wlvif->ap.global_hlid);
	ret = cc33xx_set_link(wl, wlvif, wlvif->ap.bcast_hlid );

	goto out_free;

out_free_bcast:

	cc33xx_clear_link(wl, wlvif, &wlvif->ap.bcast_hlid);
	cc33xx_clear_link(wl, wlvif, &wlvif->ap.global_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_role_stop_ap(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_role_stop *cmd;
	int ret;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role stop ap %d", wlvif->role_id);

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(wl, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role stop ap");
		goto out_free;
	}


	cc33xx_clear_link(wl, wlvif, &wlvif->ap.bcast_hlid);
	cc33xx_clear_link(wl, wlvif, &wlvif->ap.global_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_role_start_ibss(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
//TODO: RazB - need to implemntation role ibss
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	struct cc33xx_cmd_role_start *cmd;
	struct ieee80211_bss_conf *bss_conf = &vif->bss_conf;
	int ret;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role start ibss %d", wlvif->role_id);

	cmd->role_id = wlvif->role_id;
	cmd->role_type = CC33XX_ROLE_IBSS;
	if (wlvif->band == NL80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;
	cmd->channel = wlvif->channel;
	cmd->ibss.basic_rate_set = cpu_to_le32(wlvif->basic_rate_set);
	cmd->ibss.beacon_interval = cpu_to_le16(wlvif->beacon_int);
	cmd->ibss.dtim_interval = bss_conf->dtim_period;
	cmd->ibss.ssid_type = CC33XX_SSID_TYPE_ANY;
	cmd->ibss.ssid_len = wlvif->ssid_len;
	memcpy(cmd->ibss.ssid, wlvif->ssid, wlvif->ssid_len);
	memcpy(cmd->ibss.bssid, vif->bss_conf.bssid, ETH_ALEN);
	cmd->sta.local_rates = cpu_to_le32(wlvif->rate_set);

	if (wlvif->sta.hlid == CC33XX_INVALID_LINK_ID) {
		ret = cc33xx_set_link(wl, wlvif, wlvif->sta.hlid);
		if (ret)
			goto out_free;
	}
	cmd->ibss.hlid = wlvif->sta.hlid;
	cmd->ibss.remote_rates = cpu_to_le32(wlvif->rate_set);

	ret = cc33xx_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd role enable");
		goto err_hlid;
	}

	goto out_free;

err_hlid:
	/* clear links on error. */
	cc33xx_clear_link(wl, wlvif, &wlvif->sta.hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}


/**
 * send test command to firmware
 *
 * @wl: wl struct
 * @buf: buffer containing the command, with all headers, must work with dma
 * @len: length of the buffer
 * @answer: is answer needed
 */
int cc33xx_cmd_test(struct cc33xx *wl, void *buf, size_t buf_len, u8 answer)
{
	int ret;
	size_t res_len = 0;

	cc33xx_debug(DEBUG_CMD, "cmd test");

	if (answer)
		res_len = buf_len;

	ret = cc33xx_cmd_send(wl, CMD_TEST_MODE, buf, buf_len, res_len);

	if (ret < 0) {
		cc33xx_warning("TEST command failed");
		return ret;
	}

	return ret;
}

/**
 * read acx from firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer for the response, including all headers, must work with dma
 * @len: length of buf
 */
int cc33xx_cmd_interrogate(struct cc33xx *wl, u16 id, void *buf,
			   size_t cmd_len, size_t res_len)
{
	struct acx_header *acx = buf;
	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd interrogate");

	acx->id = cpu_to_le16(id);

	/* response payload length, does not include any headers */
	acx->len = cpu_to_le16(res_len - sizeof(*acx));

	ret = cc33xx_cmd_send(wl, CMD_INTERROGATE, acx, cmd_len, res_len);
	if (ret < 0)
		cc33xx_error("INTERROGATE command failed");

	return ret;
}

/**
 * read debug acx from firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer for the response, including all headers, must work with dma
 * @len: length of buf
 */
int cc33xx_cmd_debug_inter(struct cc33xx *wl, u16 id, void *buf,
			   size_t cmd_len, size_t res_len)
{
	struct acx_header *acx = buf;
	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd debug interrogate");

	acx->id = cpu_to_le16(id);

	/* response payload length, does not include any headers */
	acx->len = cpu_to_le16(res_len - sizeof(*acx));

	ret = cc33xx_cmd_send(wl, CMD_DEBUG_READ, acx, cmd_len, res_len);
	if (ret < 0)
		cc33xx_error("CMD_DEBUG_READ command failed");

	return ret;
}

/**
 * write acx value to firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer containing acx, including all headers, must work with dma
 * @len: length of buf
 * @valid_rets: bitmap of valid cmd status codes (i.e. return values).
 * return the cmd status on success.
 */
int wlcore_cmd_configure_failsafe(struct cc33xx *wl, u16 id, void *buf,
				  size_t len, unsigned long valid_rets)
{
	struct acx_header *acx = buf;
	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd configure (%d), TSFL %x",
		id, wl->core_status->tsf);

	if (WARN_ON_ONCE(len < sizeof(*acx)))
		return -EIO;

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = wlcore_cmd_send_failsafe(wl, CMD_CONFIGURE, acx, len, 0,
				       valid_rets);
	if (ret < 0) {
		cc33xx_warning("CONFIGURE command NOK");
		return ret;
	}

	return ret;
}

/*
 * wrapper for wlcore_cmd_configure that accepts only success status.
 * return 0 on success
 */
int cc33xx_cmd_configure(struct cc33xx *wl, u16 id, void *buf, size_t len)
{
	int ret = wlcore_cmd_configure_failsafe(wl, id, buf, len, 0);

	if (ret < 0)
		return ret;
	return 0;
}


/**
 * write acx value to firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer containing debug, including all headers, must work with dma
 * @len: length of buf
 * @valid_rets: bitmap of valid cmd status codes (i.e. return values).
 * return the cmd status on success.
 */
int wlcore_cmd_debug_failsafe(struct cc33xx *wl, u16 id, void *buf,
				  size_t len, unsigned long valid_rets)
{
	struct debug_header *acx = buf;
	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd debug (%d)", id);

	if (WARN_ON_ONCE(len < sizeof(*acx)))
		return -EIO;

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = wlcore_cmd_send_failsafe(wl, CMD_DEBUG, acx, len, 0,
				       valid_rets);
	if (ret < 0) {
		cc33xx_warning("CONFIGURE command NOK");
		return ret;
	}

	return ret;
}

/*
 * wrapper for wlcore_cmd_debug that accepts only success status.
 * return 0 on success
 */
int cc33xx_cmd_debug(struct cc33xx *wl, u16 id, void *buf, size_t len)
{
	int ret = wlcore_cmd_debug_failsafe(wl, id, buf, len, 0);

	if (ret < 0)
		return ret;
	return 0;
}

int cc33xx_cmd_ps_mode(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		       u8 ps_mode, u16 auto_ps_timeout)
{
	struct cc33xx_cmd_ps_params *ps_params = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd set ps mode");

	ps_params = kzalloc(sizeof(*ps_params), GFP_KERNEL);
	if (!ps_params) {
		ret = -ENOMEM;
		goto out;
	}

	ps_params->role_id = wlvif->role_id;
	ps_params->ps_mode = ps_mode;
	ps_params->auto_ps_timeout = cpu_to_le16(auto_ps_timeout);

	ret = cc33xx_cmd_send(wl, CMD_SET_PS_MODE, ps_params,
			      sizeof(*ps_params), 0);
	if (ret < 0) {
		cc33xx_error("cmd set_ps_mode failed");
		goto out;
	}

out:
	kfree(ps_params);
	return ret;
}

int cc33xx_cmd_template_set(struct cc33xx *wl, u8 role_id,
			    u16 template_id, void *buf, size_t buf_len,
			    int index, u32 rates)
{
	struct cc33xx_cmd_template_set *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd template_set %d (role %d)",
		     template_id, role_id);

	WARN_ON(buf_len > CC33XX_CMD_TEMPL_MAX_SIZE);
	buf_len = min_t(size_t, buf_len, CC33XX_CMD_TEMPL_MAX_SIZE);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	/* during initialization wlvif is NULL */
	cmd->role_id = role_id;
	cmd->len = cpu_to_le16(buf_len);
	cmd->template_type = template_id;
	cmd->enabled_rates = cpu_to_le32(rates);
	cmd->short_retry_limit = wl->conf.host_conf.tx.tmpl_short_retry_limit;
	cmd->long_retry_limit = wl->conf.host_conf.tx.tmpl_long_retry_limit;
	cmd->index = index;

	if (buf)
		memcpy(cmd->template_data, buf, buf_len);

	ret = cc33xx_cmd_send(wl, CMD_SET_TEMPLATE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_warning("cmd set_template failed: %d", ret);
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_build_null_data(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct sk_buff *skb = NULL;
	int size;
	void *ptr;
	int ret = -ENOMEM;


	if (wlvif->bss_type == BSS_TYPE_IBSS) {
		size = sizeof(struct cc33xx_null_data_template);
		ptr = NULL;
	} else {
		skb = ieee80211_nullfunc_get(wl->hw,
					     cc33xx_wlvif_to_vif(wlvif),
					     -1, false);
		if (!skb)
			goto out;
		size = skb->len;
		ptr = skb->data;
	}

	ret = cc33xx_cmd_template_set(wl, wlvif->role_id,
				      CMD_TEMPL_NULL_DATA, ptr, size, 0,
				      wlvif->basic_rate);

out:
	dev_kfree_skb(skb);
	if (ret)
		cc33xx_warning("cmd buld null data failed %d", ret);

	return ret;

}

struct sk_buff *cc33xx_cmd_build_ap_probe_req(struct cc33xx *wl,
					      struct cc33xx_vif *wlvif,
					      struct sk_buff *skb)
{
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	int ret;
	u32 rate;

	if (!skb)
		skb = ieee80211_ap_probereq_get(wl->hw, vif);
	if (!skb)
		goto out;

	cc33xx_debug(DEBUG_SCAN, "set ap probe request template");

	rate = cc33xx_tx_min_rate_get(wl, wlvif->bitrate_masks[wlvif->band]);
	if (wlvif->band == NL80211_BAND_2GHZ)
		ret = cc33xx_cmd_template_set(wl, wlvif->role_id,
					      CMD_TEMPL_CFG_PROBE_REQ_2_4,
					      skb->data, skb->len, 0, rate);
	else
		ret = cc33xx_cmd_template_set(wl, wlvif->role_id,
					      CMD_TEMPL_CFG_PROBE_REQ_5,
					      skb->data, skb->len, 0, rate);

	if (ret < 0)
		cc33xx_error("Unable to set ap probe request template.");

out:
	return skb;
}

int cc33xx_cmd_build_arp_rsp(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	int ret, extra = 0;
	u16 fc;
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
	struct sk_buff *skb;
	struct cc33xx_arp_rsp_template *tmpl;
	struct ieee80211_hdr_3addr *hdr;
	struct arphdr *arp_hdr;

	skb = dev_alloc_skb(sizeof(*hdr) + sizeof(__le16) + sizeof(*tmpl) +
			    CC33XX_EXTRA_SPACE_MAX);
	if (!skb) {
		cc33xx_error("failed to allocate buffer for arp rsp template");
		return -ENOMEM;
	}

	skb_reserve(skb, sizeof(*hdr) + CC33XX_EXTRA_SPACE_MAX);

	tmpl = skb_put_zero(skb, sizeof(*tmpl));

	/* llc layer */
	memcpy(tmpl->llc_hdr, rfc1042_header, sizeof(rfc1042_header));
	tmpl->llc_type = cpu_to_be16(ETH_P_ARP);

	/* arp header */
	arp_hdr = &tmpl->arp_hdr;
	arp_hdr->ar_hrd = cpu_to_be16(ARPHRD_ETHER);
	arp_hdr->ar_pro = cpu_to_be16(ETH_P_IP);
	arp_hdr->ar_hln = ETH_ALEN;
	arp_hdr->ar_pln = 4;
	arp_hdr->ar_op = cpu_to_be16(ARPOP_REPLY);

	/* arp payload */
	memcpy(tmpl->sender_hw, vif->addr, ETH_ALEN);
	tmpl->sender_ip = wlvif->ip_addr;

	/* encryption space */
	switch (wlvif->encryption_type) {
	case KEY_TKIP:
		if (wl->quirks & WLCORE_QUIRK_TKIP_HEADER_SPACE)
			extra = CC33XX_EXTRA_SPACE_TKIP;
		break;
	case KEY_AES:
	case KEY_GCMP_256:
		extra = CC33XX_EXTRA_SPACE_AES;
		break;
	case KEY_NONE:
	case KEY_WEP:
	case KEY_GEM:
		extra = 0;
		break;
	default:
		cc33xx_warning("Unknown encryption type: %d",
			       wlvif->encryption_type);
		ret = -EINVAL;
		goto out;
	}

	if (extra) {
		u8 *space = skb_push(skb, extra);
		memset(space, 0, extra);
	}

	/* QoS header - BE */
	if (wlvif->sta.qos)
		memset(skb_push(skb, sizeof(__le16)), 0, sizeof(__le16));

	/* mac80211 header */
	hdr = skb_push(skb, sizeof(*hdr));
	memset(hdr, 0, sizeof(*hdr));
	fc = IEEE80211_FTYPE_DATA | IEEE80211_FCTL_TODS;
	if (wlvif->sta.qos)
		fc |= IEEE80211_STYPE_QOS_DATA;
	else
		fc |= IEEE80211_STYPE_DATA;
	if (wlvif->encryption_type != KEY_NONE)
		fc |= IEEE80211_FCTL_PROTECTED;

	hdr->frame_control = cpu_to_le16(fc);
	memcpy(hdr->addr1, vif->bss_conf.bssid, ETH_ALEN);
	memcpy(hdr->addr2, vif->addr, ETH_ALEN);
	eth_broadcast_addr(hdr->addr3);

	ret = cc33xx_cmd_template_set(wl, wlvif->role_id, CMD_TEMPL_ARP_RSP,
				      skb->data, skb->len, 0,
				      wlvif->basic_rate);
out:
	dev_kfree_skb(skb);
	return ret;
}

int cc33xx_build_qos_null_data(struct cc33xx *wl, struct ieee80211_vif *vif)
{
	struct cc33xx_vif *wlvif = cc33xx_vif_to_data(vif);
	struct ieee80211_qos_hdr template;

	memset(&template, 0, sizeof(template));

	memcpy(template.addr1, vif->bss_conf.bssid, ETH_ALEN);
	memcpy(template.addr2, vif->addr, ETH_ALEN);
	memcpy(template.addr3, vif->bss_conf.bssid, ETH_ALEN);

	template.frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA |
					     IEEE80211_STYPE_QOS_NULLFUNC |
					     IEEE80211_FCTL_TODS);

	/* FIXME: not sure what priority to use here */
	template.qos_ctrl = cpu_to_le16(0);

	return cc33xx_cmd_template_set(wl, wlvif->role_id,
				       CMD_TEMPL_QOS_NULL_DATA, &template,
				       sizeof(template), 0,
				       wlvif->basic_rate);
}

int cc33xx_cmd_set_default_wep_key(struct cc33xx *wl, u8 id, u8 hlid)
{
	struct cc33xx_cmd_set_keys *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd set_default_wep_key %d", id);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->hlid = hlid;
	cmd->key_id = id;
	cmd->lid_key_type = WEP_DEFAULT_LID_TYPE;
	cmd->key_action = cpu_to_le16(KEY_SET_ID);
	cmd->key_type = KEY_WEP;

	ret = cc33xx_cmd_send(wl, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_warning("cmd set_default_wep_key failed: %d", ret);
		goto out;
	}

out:
	kfree(cmd);

	return ret;
}

int cc33xx_cmd_set_sta_key(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		       u16 action, u8 id, u8 key_type,
		       u8 key_size, const u8 *key, const u8 *addr,
		       u32 tx_seq_32, u16 tx_seq_16)
{
	struct cc33xx_cmd_set_keys *cmd;
	int ret = 0;

	/* hlid might have already been deleted */
	if (wlvif->sta.hlid == CC33XX_INVALID_LINK_ID)
		return 0;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->hlid = wlvif->sta.hlid;

	if (key_type == KEY_WEP)
		cmd->lid_key_type = WEP_DEFAULT_LID_TYPE;
	else if (is_broadcast_ether_addr(addr))
		cmd->lid_key_type = BROADCAST_LID_TYPE;
	else
		cmd->lid_key_type = UNICAST_LID_TYPE;

	cmd->key_action = cpu_to_le16(action);
	cmd->key_size = key_size;
	cmd->key_type = key_type;

	cmd->ac_seq_num16[0] = cpu_to_le16(tx_seq_16);
	cmd->ac_seq_num32[0] = cpu_to_le32(tx_seq_32);

	cmd->key_id = id;

	if (key_type == KEY_TKIP) {
		/*
		 * We get the key in the following form:
		 * TKIP (16 bytes) - TX MIC (8 bytes) - RX MIC (8 bytes)
		 * but the target is expecting:
		 * TKIP - RX MIC - TX MIC
		 */
		memcpy(cmd->key, key, 16);
		memcpy(cmd->key + 16, key + 24, 8);
		memcpy(cmd->key + 24, key + 16, 8);

	} else {
		memcpy(cmd->key, key, key_size);
	}

	cc33xx_dump(DEBUG_CRYPT, "TARGET KEY: ", cmd, sizeof(*cmd));

	ret = cc33xx_cmd_send(wl, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_warning("could not set keys");
		goto out;
	}

out:
	kfree(cmd);

	return ret;
}

/*
 * TODO: merge with sta/ibss into 1 set_key function.
 * note there are slight diffs
 */
int cc33xx_cmd_set_ap_key(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			  u16 action, u8 id, u8 key_type,
			  u8 key_size, const u8 *key, u8 hlid, u32 tx_seq_32,
			  u16 tx_seq_16)
{
	struct cc33xx_cmd_set_keys *cmd;
	int ret = 0;
	u8 lid_type;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	if (hlid == wlvif->ap.bcast_hlid) {
		if (key_type == KEY_WEP)
			lid_type = WEP_DEFAULT_LID_TYPE;
		else
			lid_type = BROADCAST_LID_TYPE;
	} else {
		lid_type = UNICAST_LID_TYPE;
	}

	cc33xx_debug(DEBUG_CRYPT, "ap key action: %d id: %d lid: %d type: %d"
		     " hlid: %d", (int)action, (int)id, (int)lid_type,
		     (int)key_type, (int)hlid);

	cmd->lid_key_type = lid_type;
	cmd->hlid = hlid;
	cmd->key_action = cpu_to_le16(action);
	cmd->key_size = key_size;
	cmd->key_type = key_type;
	cmd->key_id = id;
	cmd->ac_seq_num16[0] = cpu_to_le16(tx_seq_16);
	cmd->ac_seq_num32[0] = cpu_to_le32(tx_seq_32);

	if (key_type == KEY_TKIP) {
		/*
		 * We get the key in the following form:
		 * TKIP (16 bytes) - TX MIC (8 bytes) - RX MIC (8 bytes)
		 * but the target is expecting:
		 * TKIP - RX MIC - TX MIC
		 */
		memcpy(cmd->key, key, 16);
		memcpy(cmd->key + 16, key + 24, 8);
		memcpy(cmd->key + 24, key + 16, 8);
	} else {
		memcpy(cmd->key, key, key_size);
	}

	cc33xx_dump(DEBUG_CRYPT, "TARGET AP KEY: ", cmd, sizeof(*cmd));

	ret = cc33xx_cmd_send(wl, CMD_SET_KEYS, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_warning("could not set ap keys");
		goto out;
	}

out:
	kfree(cmd);
	return ret;
}

int cc33xx_cmd_set_peer_state(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			      u8 hlid)
{
	struct cc33xx_cmd_set_peer_state *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd set peer state (hlid=%d)", hlid);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->hlid = hlid;
	cmd->state = CC33XX_CMD_STA_STATE_CONNECTED;

	ret = cc33xx_cmd_send(wl, CMD_SET_LINK_CONNECTION_STATE, cmd, sizeof(*cmd), 0);
    if (ret < 0) {
        cc33xx_error("failed to send set peer state command");
        goto out_free;
    }

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_add_peer(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			struct ieee80211_sta *sta, u8 *hlid, u8 is_connected)
{
	struct cc33xx_cmd_add_peer *cmd;

	struct cc33xx_cmd_complete_add_peer *command_complete =
	          (struct cc33xx_cmd_complete_add_peer *)&wl->command_result;


	int i, ret;
	u32 sta_rates;

	//cc33xx_debug(DEBUG_CMD, "cmd add peer %d", (int)hlid);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}


	cc33xx_debug(DEBUG_CMD, "cmd add peer is ap %d", is_connected);
	cmd->is_connected = is_connected;
	cmd->role_id = wlvif->role_id;
	cmd->role_type = CC33XX_ROLE_AP;
	cmd->link_type = 1;

	memcpy(cmd->addr, sta->addr, ETH_ALEN);
	cmd->bss_index = CC33XX_AP_BSS_INDEX;
	cmd->aid = cpu_to_le16(sta->aid);
	cmd->sp_len = sta->max_sp;
	cmd->wmm = sta->wme ? 1 : 0;

	for (i = 0; i < NUM_ACCESS_CATEGORIES_COPY; i++)
		if (sta->wme && (sta->uapsd_queues & BIT(i)))
			cmd->psd_type[NUM_ACCESS_CATEGORIES_COPY-1-i] =
					CC33XX_PSD_UPSD_TRIGGER;
		else
			cmd->psd_type[NUM_ACCESS_CATEGORIES_COPY-1-i] =
					CC33XX_PSD_LEGACY;


	sta_rates = sta->deflink.supp_rates[wlvif->band];
	if (sta->deflink.ht_cap.ht_supported)
		sta_rates |=
			(sta->deflink.ht_cap.mcs.rx_mask[0] << HW_HT_RATES_OFFSET) |
			(sta->deflink.ht_cap.mcs.rx_mask[1] << HW_MIMO_RATES_OFFSET);

	cmd->supported_rates =
		cpu_to_le32(cc33xx_tx_enabled_rates_get(wl, sta_rates,
							wlvif->band));

	if (!cmd->supported_rates) {
		cc33xx_debug(DEBUG_CMD,
			     "peer has no supported rates yet, configuring basic rates: 0x%x",
			     wlvif->basic_rate_set);
		cmd->supported_rates = cpu_to_le32(wlvif->basic_rate_set);
	}

	cc33xx_debug(DEBUG_CMD, "new peer rates=0x%x queues=0x%x",
		     cmd->supported_rates, sta->uapsd_queues);

	if(sta->deflink.ht_cap.ht_supported)
	{
		cmd->ht_capabilities = cpu_to_le32(sta->deflink.ht_cap.cap);
		cmd->ht_capabilities|= cpu_to_le32(CC33XX_HT_CAP_HT_OPERATION);
		cmd->ampdu_params = sta->deflink.ht_cap.ampdu_factor | sta->deflink.ht_cap.ampdu_density;

	}
	cmd->has_he= sta->deflink.he_cap.has_he;
	cmd->mfp= sta->mfp;
	ret = cc33xx_cmd_send(wl, CMD_ADD_PEER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd add peer");
		goto out_free;
	}

	if(NULL != hlid) {
		if (command_complete->header.status == CMD_STATUS_SUCCESS) {
			*hlid = command_complete->hlid;
			wl->links[*hlid].allocated_pkts = 0;
			wl->session_ids[*hlid] = command_complete->session_id;
			cc33xx_debug(DEBUG_CMD, "new peer hlid=%d session_ids=%d",
			command_complete->hlid, command_complete->session_id);
		} else {
			ret = -EMLINK;
		}
	} else {
		cc33xx_debug(DEBUG_CMD, "update peer done !");
	}
out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_remove_peer(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			   u8 hlid)
{
	struct cc33xx_cmd_remove_peer *cmd;
	int ret;
	bool timeout = false;

	cc33xx_debug(DEBUG_CMD, "cmd remove peer %d", (int)hlid);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->hlid = hlid;

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(wl, CMD_REMOVE_PEER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to initiate cmd remove peer");
		goto out_free;
	}

	ret = cc33xx_wait_for_event(wl,
		WLCORE_EVENT_PEER_REMOVE_COMPLETE,
		&timeout);

	/*
	 * We are ok with a timeout here. The event is sometimes not sent
	 * due to a firmware bug. In case of another error (like SDIO timeout)
	 * queue a recovery.
	 */
	if (ret < 0)
		cc33xx_queue_recovery_work(wl);

out_free:
	kfree(cmd);

out:
	return ret;
}

static int wlcore_get_reg_conf_ch_idx(enum nl80211_band band, u16 ch)
{
	/*
	 * map the given band/channel to the respective predefined
	 * bit expected by the fw
	 */
	switch (band) {
	case NL80211_BAND_2GHZ:
		/* channels 1..14 are mapped to 0..13 */
		if (ch >= 1 && ch <= 14)
			return ch - 1;
		break;
	case NL80211_BAND_5GHZ:
		switch (ch) {
		case 8 ... 16:
			/* channels 8,12,16 are mapped to 18,19,20 */
			return 18 + (ch-8)/4;
		case 34 ... 48:
			/* channels 34,36..48 are mapped to 21..28 */
			return 21 + (ch-34)/2;
		case 52 ... 64:
			/* channels 52,56..64 are mapped to 29..32 */
			return 29 + (ch-52)/4;
		case 100 ... 140:
			/* channels 100,104..140 are mapped to 33..43 */
			return 33 + (ch-100)/4;
		case 149 ... 165:
			/* channels 149,153..165 are mapped to 44..48 */
			return 44 + (ch-149)/4;
		default:
			break;
		}
		break;
	default:
		break;
	}

	cc33xx_error("%s: unknown band/channel: %d/%d", __func__, band, ch);
	return -1;
}

void wlcore_set_pending_regdomain_ch(struct cc33xx *wl, u16 channel,
				     enum nl80211_band band)
{
	int ch_bit_idx = 0;

	if (!(wl->quirks & WLCORE_QUIRK_REGDOMAIN_CONF))
		return;

	ch_bit_idx = wlcore_get_reg_conf_ch_idx(band, channel);

	if (ch_bit_idx >= 0 && ch_bit_idx <= CC33XX_MAX_CHANNELS)
		__set_bit_le(ch_bit_idx, (long *)wl->reg_ch_conf_pending);
}

int wlcore_cmd_regdomain_config_locked(struct cc33xx *wl)
{
	struct cc33xx_cmd_regdomain_dfs_config *cmd = NULL;
	int ret = 0, i, b, ch_bit_idx;
	__le32 tmp_ch_bitmap[2] __aligned(sizeof(unsigned long));
	struct wiphy *wiphy = wl->hw->wiphy;
	struct ieee80211_supported_band *band;
	bool timeout = false;
	return 0;

	if (!(wl->quirks & WLCORE_QUIRK_REGDOMAIN_CONF))
		return 0;

	cc33xx_debug(DEBUG_CMD, "cmd reg domain config");

	memcpy(tmp_ch_bitmap, wl->reg_ch_conf_pending, sizeof(tmp_ch_bitmap));

	for (b = NL80211_BAND_2GHZ; b <= NL80211_BAND_5GHZ; b++) {
		band = wiphy->bands[b];
		for (i = 0; i < band->n_channels; i++) {
			struct ieee80211_channel *channel = &band->channels[i];
			u16 ch = channel->hw_value;
			u32 flags = channel->flags;

			if (flags & (IEEE80211_CHAN_DISABLED |
				     IEEE80211_CHAN_NO_IR))
				continue;

			if ((flags & IEEE80211_CHAN_RADAR) &&
			    channel->dfs_state != NL80211_DFS_AVAILABLE)
				continue;

			ch_bit_idx = wlcore_get_reg_conf_ch_idx(b, ch);
			if (ch_bit_idx < 0)
				continue;

			__set_bit_le(ch_bit_idx, (long *)tmp_ch_bitmap);
		}
	}

	if (!memcmp(tmp_ch_bitmap, wl->reg_ch_conf_last, sizeof(tmp_ch_bitmap)))
		goto out;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->ch_bit_map1 = tmp_ch_bitmap[0];
	cmd->ch_bit_map2 = tmp_ch_bitmap[1];
	cmd->dfs_region = wl->dfs_region;

	cc33xx_debug(DEBUG_CMD,
		     "cmd reg domain bitmap1: 0x%08x, bitmap2: 0x%08x",
		     cmd->ch_bit_map1, cmd->ch_bit_map2);

	ret = cc33xx_cmd_send(wl, CMD_DFS_CHANNEL_CONFIG, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send reg domain dfs config");
		goto out;
	}

	ret = cc33xx_wait_for_event(wl,
		WLCORE_EVENT_DFS_CONFIG_COMPLETE,
		&timeout);

	if (ret < 0 || timeout) {
		cc33xx_error("reg domain conf %serror",
			     timeout ? "completion " : "");
		ret = timeout ? -ETIMEDOUT : ret;
		goto out;
	}

	memcpy(wl->reg_ch_conf_last, tmp_ch_bitmap, sizeof(tmp_ch_bitmap));
	memset(wl->reg_ch_conf_pending, 0, sizeof(wl->reg_ch_conf_pending));

out:
	kfree(cmd);
	return ret;
}

int cc33xx_cmd_config_fwlog(struct cc33xx *wl)
{
	struct cc33xx_cmd_config_fwlog *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd config firmware logger");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->logger_mode = wl->conf.host_conf.fwlog.mode;
	cmd->log_severity = wl->conf.host_conf.fwlog.severity;
	cmd->timestamp = wl->conf.host_conf.fwlog.timestamp;
	cmd->output = wl->conf.host_conf.fwlog.output;
	cmd->threshold = wl->conf.host_conf.fwlog.threshold;

	ret = cc33xx_cmd_send(wl, CMD_CONFIG_FWLOGGER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send config firmware logger command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_start_fwlog(struct cc33xx *wl)
{
	struct cc33xx_cmd_start_fwlog *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd start firmware logger");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_send(wl, CMD_START_FWLOGGER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send start firmware logger command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_cmd_stop_fwlog(struct cc33xx *wl)
{
	struct cc33xx_cmd_stop_fwlog *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd stop firmware logger");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_send(wl, CMD_STOP_FWLOGGER, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send stop firmware logger command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

static int cc33xx_cmd_roc(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			  u8 role_id, enum nl80211_band band, u8 channel)
{
	struct cc33xx_cmd_roc *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd roc %d (%d)", channel, role_id);

	if (WARN_ON(role_id == CC33XX_INVALID_ROLE_ID))
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->role_id = role_id;
	cmd->channel = channel;
	switch (band) {
	case NL80211_BAND_2GHZ:
		cmd->band = WLCORE_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		cmd->band = WLCORE_BAND_5GHZ;
		break;
	default:
		cc33xx_error("roc - unknown band: %d", (int)wlvif->band);
		ret = -EINVAL;
		goto out_free;
	}


	ret = cc33xx_cmd_send(wl, CMD_REMAIN_ON_CHANNEL, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send ROC command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

static int cc33xx_cmd_croc(struct cc33xx *wl, u8 role_id)
{
	struct cc33xx_cmd_croc *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd croc (%d)", role_id);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}
	cmd->role_id = role_id;

	ret = cc33xx_cmd_send(wl, CMD_CANCEL_REMAIN_ON_CHANNEL, cmd,
			      sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send ROC command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int cc33xx_roc(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 role_id,
	       enum nl80211_band band, u8 channel)
{
	int ret = 0;

	if (WARN_ON(test_bit(role_id, wl->roc_map)))
		return 0;

	ret = cc33xx_cmd_roc(wl, wlvif, role_id, band, channel);
	if (ret < 0)
		goto out;

	__set_bit(role_id, wl->roc_map);
out:
	return ret;
}

int cc33xx_croc(struct cc33xx *wl, u8 role_id)
{
	int ret = 0;

	if (WARN_ON(!test_bit(role_id, wl->roc_map)))
		return 0;

	ret = cc33xx_cmd_croc(wl, role_id);
	if (ret < 0)
		goto out;

	__clear_bit(role_id, wl->roc_map);

	/*
	 * Rearm the tx watchdog when removing the last ROC. This prevents
	 * recoveries due to just finished ROCs - when Tx hasn't yet had
	 * a chance to get out.
	 */
	if (find_first_bit(wl->roc_map, CC33XX_MAX_ROLES) >= CC33XX_MAX_ROLES)
		cc33xx_rearm_tx_watchdog_locked(wl);
out:
	return ret;
}

int cc33xx_cmd_stop_channel_switch(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_cmd_stop_channel_switch *cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "cmd stop channel switch");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(wl, CMD_STOP_CHANNEL_SWICTH, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to stop channel switch command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

/* start dev role and roc on its channel */
int cc33xx_start_dev(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		     enum nl80211_band band, int channel)
{
	int ret;

	if (WARN_ON(!(wlvif->bss_type == BSS_TYPE_STA_BSS ||
		      wlvif->bss_type == BSS_TYPE_IBSS)))
		return -EINVAL;

	/* the dev role is already started for p2p mgmt interfaces */

	if (!wlcore_is_p2p_mgmt(wlvif)) {
		ret = cc33xx_cmd_role_enable(wl,
					     cc33xx_wlvif_to_vif(wlvif)->addr,
					     CC33XX_ROLE_DEVICE,
					     &wlvif->dev_role_id);
		if (ret < 0)
			goto out;
	}

	cc33xx_debug(DEBUG_CMD, "cmd role start dev");
	ret = cc33xx_cmd_role_start_dev(wl, wlvif, band, channel);
	if (ret < 0)
		goto out_disable;

	cc33xx_debug(DEBUG_CMD, "cmd roc");
	ret = cc33xx_roc(wl, wlvif, wlvif->dev_role_id, band, channel);
	if (ret < 0)
		goto out_stop;

	return 0;

out_stop:
	cc333xx_cmd_role_stop_dev(wl, wlvif);
out_disable:
	if (!wlcore_is_p2p_mgmt(wlvif))
		cc33xx_cmd_role_disable(wl, &wlvif->dev_role_id);
out:
	return ret;
}

/* croc dev hlid, and stop the role */
int cc33xx_stop_dev(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	int ret;

	if (WARN_ON(!(wlvif->bss_type == BSS_TYPE_STA_BSS ||
		      wlvif->bss_type == BSS_TYPE_IBSS)))
		return -EINVAL;

	/* flush all pending packets */
	ret = wlcore_tx_work_locked(wl);
	if (ret < 0)
		goto out;

	if (test_bit(wlvif->dev_role_id, wl->roc_map)) {
		ret = cc33xx_croc(wl, wlvif->dev_role_id);
		if (ret < 0)
			goto out;
	}

	ret = cc333xx_cmd_role_stop_dev(wl, wlvif);
	if (ret < 0)
		goto out;

	if (!wlcore_is_p2p_mgmt(wlvif)) {
		ret = cc33xx_cmd_role_disable(wl, &wlvif->dev_role_id);
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

int wlcore_cmd_generic_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			   u8 feature, u8 enable, u8 value)
{
	struct wlcore_cmd_generic_cfg *cmd;
	int ret;

	cc33xx_debug(DEBUG_CMD,
		     "cmd generic cfg (role %d feature %d enable %d value %d)",
		     wlvif->role_id, feature, enable, value);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;
	cmd->feature = feature;
	cmd->enable = enable;
	cmd->value = value;

	ret = cc33xx_cmd_send(wl, CMD_GENERIC_CFG, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send generic cfg command");
		goto out_free;
	}
out_free:
	kfree(cmd);
	return ret;
}

int cmd_channel_switch(struct cc33xx *wl,
			      struct cc33xx_vif *wlvif,
			      struct ieee80211_channel_switch *ch_switch)
{
	struct cmd_channel_switch *cmd;
	u32 supported_rates;
	int ret;

	cc33xx_debug(DEBUG_ACX, "cmd channel switch (count=%d)",
		     ch_switch->count);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->role_id = wlvif->role_id;
	cmd->channel = ch_switch->chandef.chan->hw_value;
	cmd->switch_time = ch_switch->count;
	cmd->stop_tx = ch_switch->block_tx;

	switch (ch_switch->chandef.chan->band) {
	case NL80211_BAND_2GHZ:
		cmd->band = WLCORE_BAND_2_4GHZ;
		break;
	case NL80211_BAND_5GHZ:
		cmd->band = WLCORE_BAND_5GHZ;
		break;
	default:
		cc33xx_error("invalid channel switch band: %d",
			     ch_switch->chandef.chan->band);
		ret = -EINVAL;
		goto out_free;
	}

	supported_rates = CONF_TX_ENABLED_RATES | CONF_TX_MCS_RATES;
	supported_rates |= wlvif->rate_set;
	if (wlvif->p2p)
		supported_rates &= ~CONF_TX_CCK_RATES;
	cmd->local_supported_rates = cpu_to_le32(supported_rates);
	cmd->channel_type = wlvif->channel_type;

	ret = cc33xx_cmd_send(wl, CMD_CHANNEL_SWITCH, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send channel switch command");
		goto out_free;
	}

out_free:
	kfree(cmd);
out:
	return ret;
}

int cmd_dfs_master_restart(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cmd_dfs_master_restart *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd dfs master restart (role %d)",
		     wlvif->role_id);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;

	ret = cc33xx_cmd_send(wl, CMD_DFS_MASTER_RESTART,
			      cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send dfs master restart command");
		goto out_free;
	}
out_free:
	kfree(cmd);
	return ret;
}

int cmd_set_cac(struct cc33xx *wl, struct cc33xx_vif *wlvif, bool start)
{
	struct cmd_cac_start *cmd;
	int ret = 0;

	cc33xx_debug(DEBUG_CMD, "cmd cac (channel %d) %s",
		     wlvif->channel, start ? "start" : "stop");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->role_id = wlvif->role_id;
	cmd->channel = wlvif->channel;
	if (wlvif->band == NL80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;
	cmd->bandwidth = wlcore_get_native_channel_type(wlvif->channel_type);

	ret = cc33xx_cmd_send(wl,
			      start ? CMD_CAC_START : CMD_CAC_STOP,
			      cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to send cac command");
		goto out_free;
	}

out_free:
	kfree(cmd);
	return ret;
}

int cmd_set_bd_addr(struct cc33xx *wl, u8 *bd_addr)
{
	struct cmd_set_bd_addr *cmd;
	int ret = 0;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(cmd->bd_addr, bd_addr, sizeof cmd->bd_addr);

	ret = cc33xx_cmd_send(wl, CMD_SET_BD_ADDR, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		cc33xx_error("failed to set BD address");
		goto out_free;
	}

out_free:
	kfree(cmd);
out:
	return ret;
}

int cmd_get_device_info(struct cc33xx *wl, u8 *info_buffer, size_t buffer_len)
{
	struct cc33xx_cmd_get_device_info *cmd;
	int ret = 0;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	ret = cc33xx_cmd_send(wl, CMD_BM_READ_DEVICE_INFO, cmd,
			      sizeof(*cmd), sizeof(*cmd));
	if (ret < 0) {
		cc33xx_error("Device info command failure ");
	} else {
		WARN_ON(buffer_len > sizeof cmd->device_info);
		memcpy(info_buffer, cmd->device_info, buffer_len);
	}

	kfree(cmd);

	return ret;
}

int cmd_download_container_chunk(struct cc33xx *wl, u8 *chunk, size_t chunk_len, bool is_last_chunk)
{
	struct cc33xx_cmd_container_download *cmd;
	const size_t command_size = sizeof(*cmd) + chunk_len;
	int ret;
	bool  is_sync_transfer = !is_last_chunk;

	cmd = kzalloc(command_size, GFP_KERNEL);

	if (!cmd) {
		cc33xx_error("Chunk buffer allocation failure");
		return -ENOMEM;
	}

	memcpy(cmd->payload, chunk, chunk_len);
	cmd->length = cpu_to_le32(chunk_len);

	if (is_last_chunk){
		cc33xx_debug(DEBUG_BOOT, "Suspending IRQ while device reboots");
		wlcore_disable_interrupts_nosync(wl);
	}

	ret = __wlcore_cmd_send(wl, CMD_CONTAINER_DOWNLOAD, cmd,
				command_size, sizeof (u32), is_sync_transfer);

	kfree(cmd);

	if (is_last_chunk){
		msleep(CC33XX_REBOOT_TIMEOUT_MSEC);
		cc33xx_debug(DEBUG_BOOT, "Resuming IRQ");
		wlcore_enable_interrupts(wl);
	}

	return ret;
}
