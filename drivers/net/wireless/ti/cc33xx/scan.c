// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/ieee80211.h>

#include "wlcore.h"
#include "debug.h"
#include "cmd.h"
#include "scan.h"
#include "acx.h"
#include "tx.h"



static void cc33xx_adjust_channels(struct scan_param *scanParam,
				   struct wlcore_scan_channels *cmd_channels,
				   EScanRequestType scan_type)
{

    struct conn_scan_ch_info      *ch_list;
    struct conn_scan_dwell_info   *dwell_info;

    u8 *passive;
    u8 *dfs;
    u8 *active;
    int i,j;

    if(scan_type == SCAN_REQUEST_CONNECT_PERIODIC_SCAN)
    {
        //struct scan_periodic_info *pPeriodicScanParams = &scanParam->u.periodic;

        ch_list        = scanParam->u.periodic.channel_list;
        dwell_info   = scanParam->u.periodic.dwell_info;
        active        = (u8*)&scanParam->u.periodic.active;
        passive       = (u8*)&scanParam->u.periodic.passive;
        dfs           = (u8*)&scanParam->u.periodic.dfs;

    }
    else
    {
        ch_list        = scanParam->u.one_shot.channel_list;
        dwell_info   = scanParam->u.one_shot.dwell_info;
        active        = (u8*)&scanParam->u.one_shot.active;
        passive       = (u8*)&scanParam->u.one_shot.passive;
        dfs           = (u8*)&scanParam->u.one_shot.dfs;
    }

    memcpy(passive, cmd_channels->passive, sizeof(cmd_channels->passive));
    memcpy(active, cmd_channels->active, sizeof(cmd_channels->active));
    *dfs = cmd_channels->dfs;

    for (i = 0; i < MAX_CHANNELS_2GHZ; ++i)
    {
	ch_list[i].channel = cmd_channels->channels_2[i].channel;
	ch_list[i].flags = cmd_channels->channels_2[i].flags;
	ch_list[i].tx_power_att = cmd_channels->channels_2[i].tx_power_att;
    }

    dwell_info[NL80211_BAND_2GHZ].min_duration     = cmd_channels->channels_2[0].min_duration;
    dwell_info[NL80211_BAND_2GHZ].max_duration     = cmd_channels->channels_2[0].max_duration;
    dwell_info[NL80211_BAND_2GHZ].passive_duration = cmd_channels->channels_2[0].passive_duration;

    for (j = 0; j < MAX_CHANNELS_5GHZ; ++i, ++j)
    {
        ch_list[i].channel = cmd_channels->channels_5[j].channel;
        ch_list[i].flags = cmd_channels->channels_5[j].flags;
        ch_list[i].tx_power_att = cmd_channels->channels_5[j].tx_power_att;
    }
    dwell_info[NL80211_BAND_5GHZ].min_duration     = cmd_channels->channels_5[0].min_duration;
    dwell_info[NL80211_BAND_5GHZ].max_duration     = cmd_channels->channels_5[0].max_duration;
    dwell_info[NL80211_BAND_5GHZ].passive_duration = cmd_channels->channels_5[0].passive_duration;

}


int cc33xx_cmd_build_probe_req(struct cc33xx *wl, struct cc33xx_vif *wlvif,
                   u8 role_id, u8 scan_type,
                   const u8 *ssid, size_t ssid_len,
                   const u8 *ie0, size_t ie0_len, const u8 *ie1,
                   size_t ie1_len, bool sched_scan)
{
    struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);
    struct sk_buff *skb=NULL;

    struct cc33xx_cmd_set_ies *cmd;
    int ret;

    cc33xx_debug(DEBUG_SCAN, "build probe request scan_type %d", scan_type);

    cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
    if (!cmd) {
        ret = -ENOMEM;
        goto out;
    }

    skb = ieee80211_probereq_get(wl->hw, vif->addr, ssid, ssid_len,
                     ie0_len + ie1_len);
    if (!skb) {
        ret = -ENOMEM;
        goto out_free;
    }
    if (ie0_len)
        skb_put_data(skb, ie0, ie0_len);
    if (ie1_len)
        skb_put_data(skb, ie1, ie1_len);

    cmd->scan_type = scan_type;
    cmd->role_id = role_id;

    cmd->len = cpu_to_le16(skb->len) - sizeof(struct ieee80211_hdr_3addr);

    if (skb->data)
        memcpy(cmd->data, skb->data + sizeof(struct ieee80211_hdr_3addr), cmd->len);

    //Katya - temporary workaround - untill scan module is changed
    usleep_range(10000, 11000);
    ret = cc33xx_cmd_send(wl, CMD_SET_PROBE_IE, cmd, sizeof(*cmd), 0);

    if (ret < 0) {
        cc33xx_warning("cmd set_template failed: %d", ret);
        goto out_free;
    }

out_free:
    dev_kfree_skb(skb);
    kfree(cmd);
out:
    return ret;
}


static int cc33xx_scan_send(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			    struct cfg80211_scan_request *req)
{
	struct cc33xx_cmd_scan_params *cmd;
	struct wlcore_scan_channels *cmd_channels = NULL;
	struct cc33xx_ssid *cmd_ssid;
	u16 alloc_size;
	int ret;

	alloc_size =  sizeof(*cmd) + (sizeof(struct cc33xx_ssid)*req->n_ssids);
	cmd = kzalloc(alloc_size, GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	/* scan on the dev role if the regular one is not started */
	if (wlcore_is_p2p_mgmt(wlvif))
		cmd->role_id = wlvif->dev_role_id;
	else
		cmd->role_id = wlvif->role_id;

	if (WARN_ON(cmd->role_id == CC33XX_INVALID_ROLE_ID)) {
		ret = -EINVAL;
		goto out;
	}

	cmd->scan_type = SCAN_REQUEST_ONE_SHOT;
	cmd->rssi_threshold = -127;
	cmd->snr_threshold = 0;


	cmd->ssid_from_list = 0;
	cmd->filter = 0;
	WARN_ON(req->n_ssids > 1);

	/* configure channels */
	cmd_channels = kzalloc(sizeof(*cmd_channels), GFP_KERNEL);
	if (!cmd_channels) {
		ret = -ENOMEM;
		goto out;
	}

	wlcore_set_scan_chan_params(wl, cmd_channels, req->channels,
				    req->n_channels, req->n_ssids,
				    SCAN_TYPE_SEARCH);

	cc33xx_adjust_channels(&cmd->params, cmd_channels, cmd->scan_type);
	if (req->n_ssids > 0)
	{
		cmd->ssid_from_list = 1;
		cmd->num_of_ssids = req->n_ssids;
		cmd_ssid = (struct cc33xx_ssid *)((u8*)cmd + sizeof(*cmd));

		cmd_ssid->len = req->ssids[0].ssid_len;
		memcpy(cmd_ssid->ssid, req->ssids[0].ssid, cmd_ssid->len);
		cmd_ssid->type = (req->ssids[0].ssid_len) ?
				SCAN_SSID_TYPE_HIDDEN : SCAN_SSID_TYPE_PUBLIC;
	}

	ret = cc33xx_cmd_build_probe_req(wl, wlvif,
             cmd->role_id, cmd->scan_type,
             req->ssids ? req->ssids[0].ssid : NULL,
             req->ssids ? req->ssids[0].ssid_len : 0,
             req->ie,
             req->ie_len,
             NULL,
             0,
             false);
	if (ret < 0) {
		cc33xx_error("PROBE request template failed");
		goto out;
	}
	cc33xx_dump(DEBUG_SCAN, "SCAN: ", cmd, alloc_size);

	ret = cc33xx_cmd_send(wl, CMD_SCAN, cmd, alloc_size, 0);
	if (ret < 0) {
		cc33xx_error("SCAN failed");
		goto out;
	}

out:
	kfree(cmd_channels);
	kfree(cmd);
	return ret;
}

int
cc33xx_scan_sched_scan_ssid_list(struct cc33xx *wl,
				 struct cc33xx_vif *wlvif,
				 struct cfg80211_sched_scan_request *req,
				 struct cc33xx_cmd_ssid_list *cmd)
{
	struct cfg80211_match_set *sets = req->match_sets;
	struct cfg80211_ssid *ssids = req->ssids;
	int ret = 0, type, i, j, n_match_ssids = 0;

	cc33xx_debug((DEBUG_CMD | DEBUG_SCAN), "cmd sched scan ssid list");
	/* count the match sets that contain SSIDs */
	for (i = 0; i < req->n_match_sets; i++)
	{
		if (sets[i].ssid.ssid_len > 0)
			n_match_ssids++;
	}
	/* No filter, no ssids or only bcast ssid */
	if (!n_match_ssids &&
	    (!req->n_ssids ||
	     (req->n_ssids == 1 && req->ssids[0].ssid_len == 0))) {
		type = SCAN_SSID_FILTER_ANY;
		goto out;
	}

	cmd->role_id = wlvif->role_id;
	if (!n_match_ssids) {
		/* No filter, with ssids */
		type = SCAN_SSID_FILTER_DISABLED;

		for (i = 0; i < req->n_ssids; i++) {
			cmd->ssids[cmd->n_ssids].type = (ssids[i].ssid_len) ?
				SCAN_SSID_TYPE_HIDDEN : SCAN_SSID_TYPE_PUBLIC;
			cmd->ssids[cmd->n_ssids].len = ssids[i].ssid_len;
			memcpy(cmd->ssids[cmd->n_ssids].ssid, ssids[i].ssid,
			       ssids[i].ssid_len);
			cmd->n_ssids++;
		}
	} else {
		type = SCAN_SSID_FILTER_LIST;

		/* Add all SSIDs from the filters */
		for (i = 0; i < req->n_match_sets; i++) {
			/* ignore sets without SSIDs */
			if (!sets[i].ssid.ssid_len)
				continue;

			cmd->ssids[cmd->n_ssids].type = SCAN_SSID_TYPE_PUBLIC;
			cmd->ssids[cmd->n_ssids].len = sets[i].ssid.ssid_len;
			memcpy(cmd->ssids[cmd->n_ssids].ssid,
			       sets[i].ssid.ssid, sets[i].ssid.ssid_len);
			cmd->n_ssids++;
		}
		if ((req->n_ssids > 1) ||
		    (req->n_ssids == 1 && req->ssids[0].ssid_len > 0)) {
			/*
			 * Mark all the SSIDs passed in the SSID list as HIDDEN,
			 * so they're used in probe requests.
			 */
			for (i = 0; i < req->n_ssids; i++) {
				if (!req->ssids[i].ssid_len)
					continue;

				for (j = 0; j < cmd->n_ssids; j++)
				{
					if ((req->ssids[i].ssid_len ==
					     cmd->ssids[j].len) &&
					    !memcmp(req->ssids[i].ssid,
						   cmd->ssids[j].ssid,
						   req->ssids[i].ssid_len)) {
						cmd->ssids[j].type =
							SCAN_SSID_TYPE_HIDDEN;
						break;
					}
				}
				/* Fail if SSID isn't present in the filters */
				if (j == cmd->n_ssids) {
					ret = -EINVAL;
					goto out;
				}
			}
		}
	}

	cc33xx_debug(DEBUG_CMD, "cmd sched scan with ssid list %d",cmd->n_ssids);
	return cmd->n_ssids;
out:
	if (ret < 0)
		return ret;
	return 0;

}


static
int cc33xx_scan_sched_scan_config(struct cc33xx *wl,
				  struct cc33xx_vif *wlvif,
				  struct cfg80211_sched_scan_request *req,
				  struct ieee80211_scan_ies *ies)
{
	struct cc33xx_cmd_scan_params *cmd;
	struct cc33xx_cmd_ssid_list *ssid_list;
	struct wlcore_scan_channels *cmd_channels = NULL;
	struct conf_sched_scan_settings *c = &wl->conf.host_conf.sched_scan;
	int ret;
	int n_ssids = 0;
	int alloc_size = sizeof(*cmd);

	cc33xx_debug(DEBUG_CMD, "cmd sched_scan scan config");

	ssid_list = kzalloc(sizeof(*ssid_list), GFP_KERNEL);
	if (!ssid_list) {
		ret = -ENOMEM;
		goto out_ssid_free;
	}

	n_ssids = cc33xx_scan_sched_scan_ssid_list(wl, wlvif, req, ssid_list);
	if(n_ssids < 0) {
		return n_ssids;
	}
	cc33xx_debug(DEBUG_CMD, "ssid list num of ssids %d",
		ssid_list->n_ssids);
	if(n_ssids <= 5)
		alloc_size += (n_ssids * sizeof(struct cc33xx_ssid));
	else //n_ssids > 5
	{
		ssid_list->scan_type = SCAN_REQUEST_CONNECT_PERIODIC_SCAN;
		ret = cc33xx_cmd_send(wl, CMD_CONNECTION_SCAN_SSID_CFG, ssid_list,
			      sizeof(*ssid_list), 0);
		if (ret < 0) {
			cc33xx_error("cmd sched scan ssid list failed");
			goto out_ssid_free;
		}
	}

	cmd = kzalloc(alloc_size, GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out_free;
	}

	cmd->role_id = wlvif->role_id;

	if (WARN_ON(cmd->role_id == CC33XX_INVALID_ROLE_ID)) {
		ret = -EINVAL;
		goto out_free;
	}

	cmd->scan_type = SCAN_REQUEST_CONNECT_PERIODIC_SCAN;
	cmd->rssi_threshold = c->rssi_threshold;
	cmd->snr_threshold = c->snr_threshold;

	cmd->filter = 1;
	cmd->num_of_ssids = n_ssids;

	cc33xx_debug(DEBUG_CMD, "ssid list num of n_ssids %d",
				n_ssids);
	if( (n_ssids > 0) && (n_ssids <= 5) )
	{
		cmd->ssid_from_list = 1;
		memcpy((u8*)cmd + sizeof(*cmd),
			ssid_list->ssids,
			n_ssids * sizeof(struct cc33xx_ssid));
	}

	cmd_channels = kzalloc(sizeof(*cmd_channels), GFP_KERNEL);
	if (!cmd_channels) {
		ret = -ENOMEM;
		goto out_free;
	}

	/* configure channels */
	wlcore_set_scan_chan_params(wl, cmd_channels, req->channels,
				    req->n_channels, req->n_ssids,
				    SCAN_TYPE_PERIODIC);
	cc33xx_adjust_channels(&cmd->params, cmd_channels, cmd->scan_type);


	memcpy(cmd->params.u.periodic.sched_scan_plans,
	       req->scan_plans,
	       sizeof(struct sched_scan_plan_cmd)*req->n_scan_plans);

	cmd->params.u.periodic.sched_scan_plans_num = req->n_scan_plans;

	cc33xx_debug(DEBUG_SCAN, "interval[0]: %d, iterations[0]: %d, num_plans: %d",
		     cmd->params.u.periodic.sched_scan_plans[0].interval,
		     cmd->params.u.periodic.sched_scan_plans[0].iterations,
		     cmd->params.u.periodic.sched_scan_plans_num);


	ret = cc33xx_cmd_build_probe_req(wl, wlvif,
             cmd->role_id, cmd->scan_type,
             req->ssids ? req->ssids[0].ssid : NULL,
             req->ssids ? req->ssids[0].ssid_len : 0,
	     ies->ies[NL80211_BAND_2GHZ],
	     ies->len[NL80211_BAND_2GHZ],
             ies->common_ies,
	     ies->common_ie_len,
	     true);
    if (ret < 0) {
        cc33xx_error("PROBE request template failed");
        goto out_free;
    }


	cc33xx_dump(DEBUG_SCAN, "SCAN: ", cmd, alloc_size);

	ret = cc33xx_cmd_send(wl, CMD_SCAN, cmd, alloc_size, 0);
	if (ret < 0) {
		cc33xx_error("SCAN failed");
		goto out_free;
	}

out_free:
	kfree(cmd_channels);
	kfree(cmd);
out_ssid_free:
	kfree(ssid_list);

	return ret;
}
int cc33xx_sched_scan_start(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			    struct cfg80211_sched_scan_request *req,
			    struct ieee80211_scan_ies *ies)
{
	return cc33xx_scan_sched_scan_config(wl, wlvif, req, ies);
}

static int __cc33xx_scan_stop(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			       u8 scan_type)
{

	struct cc33xx_cmd_scan_stop *stop;

	int ret;

	cc33xx_debug(DEBUG_CMD, "cmd periodic scan stop");

	stop = kzalloc(sizeof(*stop), GFP_KERNEL);
	if (!stop) {
		cc33xx_error("failed to alloc memory to send sched scan stop");
		return -ENOMEM;
	}

	stop->role_id = wlvif->role_id;
	stop->scan_type = scan_type;

	ret = cc33xx_cmd_send(wl, CMD_STOP_SCAN, stop, sizeof(*stop), 0);
	if (ret < 0) {
		cc33xx_error("failed to send sched scan stop command");
		goto out_free;
	}

out_free:
	kfree(stop);
	return ret;
}

void cc33xx_scan_sched_scan_stop(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	__cc33xx_scan_stop(wl, wlvif, SCAN_REQUEST_CONNECT_PERIODIC_SCAN);
}

int cc33xx_scan_start(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		      struct cfg80211_scan_request *req)
{
	return cc33xx_scan_send(wl, wlvif, req);
}

int cc33xx_scan_stop(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	return __cc33xx_scan_stop(wl, wlvif, SCAN_REQUEST_ONE_SHOT);

}

void cc33xx_scan_complete_work(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct cc33xx *wl;
	struct cc33xx_vif *wlvif;
	struct cfg80211_scan_info info = {
		.aborted = false,
	};

	dwork = to_delayed_work(work);
	wl = container_of(dwork, struct cc33xx, scan_complete_work);

	cc33xx_debug(DEBUG_SCAN, "Scanning complete");

	mutex_lock(&wl->mutex);

	if (unlikely(wl->state != WLCORE_STATE_ON))
		goto out;

	if (wl->scan.state == CC33XX_SCAN_STATE_IDLE)
		goto out;

	wlvif = wl->scan_wlvif;

	/*
	 * Rearm the tx watchdog just before idling scan. This
	 * prevents just-finished scans from triggering the watchdog
	 */
	cc33xx_rearm_tx_watchdog_locked(wl);

	wl->scan.state = CC33XX_SCAN_STATE_IDLE;
	memset(wl->scan.scanned_ch, 0, sizeof(wl->scan.scanned_ch));
	wl->scan.req = NULL;
	wl->scan_wlvif = NULL;

	if (test_bit(WLVIF_FLAG_STA_ASSOCIATED, &wlvif->flags)) {
		/* restore hardware connection monitoring template */
		cc33xx_cmd_build_ap_probe_req(wl, wlvif, wlvif->probereq);
	}

	if (wl->scan.failed) {
		cc33xx_info("Scan completed due to error.");
		cc33xx_queue_recovery_work(wl);
	}

	wlcore_cmd_regdomain_config_locked(wl);

	ieee80211_scan_completed(wl->hw, &info);

out:
	mutex_unlock(&wl->mutex);

}

static void wlcore_started_vifs_iter(void *data, u8 *mac,
				     struct ieee80211_vif *vif)
{
	struct cc33xx_vif *wlvif = cc33xx_vif_to_data(vif);
	bool active = false;
	int *count = (int *)data;

	/*
	 * count active interfaces according to interface type.
	 * checking only bss_conf.idle is bad for some cases, e.g.
	 * we don't want to count sta in p2p_find as active interface.
	 */
	switch (wlvif->bss_type) {
	case BSS_TYPE_STA_BSS:
		if (test_bit(WLVIF_FLAG_STA_ASSOCIATED, &wlvif->flags))
			active = true;
		break;

	case BSS_TYPE_AP_BSS:
		if (wlvif->wl->active_sta_count > 0)
			active = true;
		break;

	default:
		break;
	}

	if (active)
		(*count)++;
}

static int wlcore_count_started_vifs(struct cc33xx *wl)
{
	int count = 0;

	ieee80211_iterate_active_interfaces_atomic(wl->hw,
					IEEE80211_IFACE_ITER_RESUME_ALL,
					wlcore_started_vifs_iter, &count);
	return count;
}

static int
wlcore_scan_get_channels(struct cc33xx *wl,
			 struct ieee80211_channel *req_channels[],
			 u32 n_channels,
			 u32 n_ssids,
			 struct conn_scan_ch_params *channels,
			 u32 band, bool radar, bool passive,
			 int start, int max_channels,
			 u8 *n_pactive_ch,
			 int scan_type)
{
	int i, j;
	u32 flags;
	bool force_passive = !n_ssids;
	u32 min_dwell_time_active, max_dwell_time_active;
	u32 dwell_time_passive, dwell_time_dfs;

	/* configure dwell times according to scan type */
	if (scan_type == SCAN_TYPE_SEARCH) {
		struct conf_scan_settings *c = &wl->conf.host_conf.scan;
		bool active_vif_exists = !!wlcore_count_started_vifs(wl);

		min_dwell_time_active = active_vif_exists ?
			c->min_dwell_time_active :
			c->min_dwell_time_active_long;
		max_dwell_time_active = active_vif_exists ?
			c->max_dwell_time_active :
			c->max_dwell_time_active_long;
		dwell_time_passive = c->dwell_time_passive;
		dwell_time_dfs = c->dwell_time_dfs;
	} else {
		struct conf_sched_scan_settings *c = &wl->conf.host_conf.sched_scan;
		u32 delta_per_probe;

		if (band == NL80211_BAND_5GHZ)
			delta_per_probe = c->dwell_time_delta_per_probe_5;
		else
			delta_per_probe = c->dwell_time_delta_per_probe;

		min_dwell_time_active = c->base_dwell_time +
			 n_ssids * c->num_probe_reqs * delta_per_probe;

		max_dwell_time_active = min_dwell_time_active +
					c->max_dwell_time_delta;
		dwell_time_passive = c->dwell_time_passive;
		dwell_time_dfs = c->dwell_time_dfs;
	}
	min_dwell_time_active = DIV_ROUND_UP(min_dwell_time_active, 1000);
	max_dwell_time_active = DIV_ROUND_UP(max_dwell_time_active, 1000);
	dwell_time_passive = DIV_ROUND_UP(dwell_time_passive, 1000);
	dwell_time_dfs = DIV_ROUND_UP(dwell_time_dfs, 1000);

	for (i = 0, j = start;
	     i < n_channels && j < max_channels;
	     i++) {
		flags = req_channels[i]->flags;

		if (force_passive)
			flags |= IEEE80211_CHAN_NO_IR;

		if ((req_channels[i]->band == band) &&
		    !(flags & IEEE80211_CHAN_DISABLED) &&
		    (!!(flags & IEEE80211_CHAN_RADAR) == radar) &&
		    /* if radar is set, we ignore the passive flag */
		    (radar ||
		     !!(flags & IEEE80211_CHAN_NO_IR) == passive)) {
			if (flags & IEEE80211_CHAN_RADAR) {
				channels[j].flags |= SCAN_CHANNEL_FLAGS_DFS;

				channels[j].passive_duration =
					cpu_to_le16(dwell_time_dfs);
			} else {
				channels[j].passive_duration =
					cpu_to_le16(dwell_time_passive);
			}

			channels[j].min_duration =
				cpu_to_le16(min_dwell_time_active);
			channels[j].max_duration =
				cpu_to_le16(max_dwell_time_active);

			channels[j].tx_power_att = req_channels[i]->max_power;
			channels[j].channel = req_channels[i]->hw_value;

			if (n_pactive_ch &&
			    (band == NL80211_BAND_2GHZ) &&
			    (channels[j].channel >= 12) &&
			    (channels[j].channel <= 14) &&
			    (flags & IEEE80211_CHAN_NO_IR) &&
			    !force_passive) {
				/* pactive channels treated as DFS */
				channels[j].flags = SCAN_CHANNEL_FLAGS_DFS;

				/*
				 * n_pactive_ch is counted down from the end of
				 * the passive channel list
				 */
				(*n_pactive_ch)++;
				cc33xx_debug(DEBUG_SCAN, "n_pactive_ch = %d",
					     *n_pactive_ch);
			}

			cc33xx_debug(DEBUG_SCAN, "freq %d, ch. %d, flags 0x%x, power %d, min/max_dwell %d/%d%s%s",
				     req_channels[i]->center_freq,
				     req_channels[i]->hw_value,
				     req_channels[i]->flags,
				     req_channels[i]->max_power,
				     min_dwell_time_active,
				     max_dwell_time_active,
				     flags & IEEE80211_CHAN_RADAR ?
					", DFS" : "",
				     flags & IEEE80211_CHAN_NO_IR ?
					", NO-IR" : "");
			j++;
		}
	}

	return j - start;
}

bool
wlcore_set_scan_chan_params(struct cc33xx *wl,
			    struct wlcore_scan_channels *cfg,
			    struct ieee80211_channel *channels[],
			    u32 n_channels,
			    u32 n_ssids,
			    int scan_type)
{
	u8 n_pactive_ch = 0;

	cfg->passive[0] =
		wlcore_scan_get_channels(wl,
					 channels,
					 n_channels,
					 n_ssids,
					 cfg->channels_2,
					 NL80211_BAND_2GHZ,
					 false, true, 0,
					 MAX_CHANNELS_2GHZ,
					 &n_pactive_ch,
					 scan_type);
	cfg->active[0] =
		wlcore_scan_get_channels(wl,
					 channels,
					 n_channels,
					 n_ssids,
					 cfg->channels_2,
					 NL80211_BAND_2GHZ,
					 false, false,
					 cfg->passive[0],
					 MAX_CHANNELS_2GHZ,
					 &n_pactive_ch,
					 scan_type);
	cfg->passive[1] =
		wlcore_scan_get_channels(wl,
					 channels,
					 n_channels,
					 n_ssids,
					 cfg->channels_5,
					 NL80211_BAND_5GHZ,
					 false, true, 0,
					 wl->max_channels_5,
					 &n_pactive_ch,
					 scan_type);
	cfg->dfs =
		wlcore_scan_get_channels(wl,
					 channels,
					 n_channels,
					 n_ssids,
					 cfg->channels_5,
					 NL80211_BAND_5GHZ,
					 true, true,
					 cfg->passive[1],
					 wl->max_channels_5,
					 &n_pactive_ch,
					 scan_type);
	cfg->active[1] =
		wlcore_scan_get_channels(wl,
					 channels,
					 n_channels,
					 n_ssids,
					 cfg->channels_5,
					 NL80211_BAND_5GHZ,
					 false, false,
					 cfg->passive[1] + cfg->dfs,
					 wl->max_channels_5,
					 &n_pactive_ch,
					 scan_type);

	/* 802.11j channels are not supported yet */
	cfg->passive[2] = 0;
	cfg->active[2] = 0;

	cfg->passive_active = n_pactive_ch;

	cc33xx_debug(DEBUG_SCAN, "    2.4GHz: active %d passive %d",
		     cfg->active[0], cfg->passive[0]);
	cc33xx_debug(DEBUG_SCAN, "    5GHz: active %d passive %d",
		     cfg->active[1], cfg->passive[1]);
	cc33xx_debug(DEBUG_SCAN, "    DFS: %d", cfg->dfs);

	return  cfg->passive[0] || cfg->active[0] ||
		cfg->passive[1] || cfg->active[1] || cfg->dfs ||
		cfg->passive[2] || cfg->active[2];
}

int wlcore_scan(struct cc33xx *wl, struct ieee80211_vif *vif,
		const u8 *ssid, size_t ssid_len,
		struct cfg80211_scan_request *req)
{
	struct cc33xx_vif *wlvif = cc33xx_vif_to_data(vif);

	/*
	 * cfg80211 should guarantee that we don't get more channels
	 * than what we have registered.
	 */
	BUG_ON(req->n_channels > CC33XX_MAX_CHANNELS);

	if (wl->scan.state != CC33XX_SCAN_STATE_IDLE)
		return -EBUSY;

	wl->scan.state = CC33XX_SCAN_STATE_2GHZ_ACTIVE;

	if (ssid_len && ssid) {
		wl->scan.ssid_len = ssid_len;
		memcpy(wl->scan.ssid, ssid, ssid_len);
	} else {
		wl->scan.ssid_len = 0;
	}

	wl->scan_wlvif = wlvif;
	wl->scan.req = req;
	memset(wl->scan.scanned_ch, 0, sizeof(wl->scan.scanned_ch));

	/* we assume failure so that timeout scenarios are handled correctly */
	wl->scan.failed = true;
	ieee80211_queue_delayed_work(wl->hw, &wl->scan_complete_work,
				     msecs_to_jiffies(CC33XX_SCAN_TIMEOUT));

	cc33xx_scan_start(wl, wlvif, req);

	return 0;
}

void wlcore_scan_sched_scan_results(struct cc33xx *wl)
{
	cc33xx_debug(DEBUG_SCAN, "got periodic scan results");

	ieee80211_sched_scan_results(wl->hw);
}

void cc33xx_scan_completed(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	cc33xx_debug(DEBUG_SCAN, "calling scan complete!");
	wl->scan.failed = false;
	cancel_delayed_work(&wl->scan_complete_work);
	ieee80211_queue_delayed_work(wl->hw, &wl->scan_complete_work,
				     msecs_to_jiffies(0));
}
