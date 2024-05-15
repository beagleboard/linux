/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __SCAN_H__
#define __SCAN_H__

#include "wlcore.h"

#define CC33XX_SCAN_TIMEOUT    30000 /* msec */


enum {
	CC33XX_SCAN_STATE_IDLE,
	CC33XX_SCAN_STATE_2GHZ_ACTIVE,
	CC33XX_SCAN_STATE_2GHZ_PASSIVE,
	CC33XX_SCAN_STATE_5GHZ_ACTIVE,
	CC33XX_SCAN_STATE_5GHZ_PASSIVE,
	CC33XX_SCAN_STATE_DONE
};

struct conn_scan_ch_params {
	__le16 min_duration;
	__le16 max_duration;
	__le16 passive_duration;

	u8  channel;
	u8  tx_power_att;

	/* bit 0: DFS channel; bit 1: DFS enabled */
	u8  flags;

	u8  padding[3];
} __packed;

enum {
	SCAN_SSID_TYPE_PUBLIC = 0,
	SCAN_SSID_TYPE_HIDDEN = 1,
};

#define MAX_CHANNELS_2GHZ	14
#define MAX_CHANNELS_4GHZ	4
#define MAX_CHANNELS_5GHZ	32

#define SCAN_MAX_CYCLE_INTERVALS 16
#define SCAN_MAX_BANDS 3
#define SCHED_SCAN_MAX_SSIDS 16

/******************************************************************************
* ** ***                                                               *** ** *
* ** ***                          SCAN API                             *** ** *
* ** ***                                                               *** ** *
*******************************************************************************/

#define CONN_SCAN_MAX_BAND                          (2)
#define CONN_SCAN_MAX_CHANNELS_ALL_BANDS            (46)
#define SCAN_MAX_SCHED_SCAN_PLANS           (12)

typedef enum
{
	SCAN_REQUEST_NONE,
	SCAN_REQUEST_CONNECT_PERIODIC_SCAN,
	SCAN_REQUEST_ONE_SHOT,
	SCAN_REQUEST_SURVEY_SCAN,
	SCAN_NUM_OF_REQUEST_TYPE
} EScanRequestType;

/******************************************************************************
        ID:     CMD_SCAN
        Desc:   This command will start scan process depending scan request
                type
        Return: CMD_COMPLETE
******************************************************************************/
/**
* struct cc33xx_ssid - SSIDs connection scan description
*
* @type: SSID type - SCAN_SSID_TYPE_HIDDEN/SCAN_SSID_TYPE_PUBLIC
*
* @len:  Length of the ssid
*
* @ssid: SSID
*/
struct cc33xx_ssid
{
	u8 type;
	u8 len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 padding[2];
} __packed;

/**
 * struct cc33xx_cmd_ssid_list - scan SSID list description
 *
 * @role_id:            roleID
 *
 * @num_of_ssids:       Number of SSID in the list. MAX 16 entries
 *
 * @ssid_list:          SSIDs to scan for (active scan only)
*/
struct cc33xx_cmd_ssid_list
{
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 scan_type;
	u8 n_ssids;
	struct cc33xx_ssid ssids[SCHED_SCAN_MAX_SSIDS];
	u8 padding;
}__packed;

/**
 * struct conn_scan_dwell_info - Channels duration info per band
 *
 * @min_duration:        Min duration (in ms)
 *
 * @max_duration:        Max duration (in ms)
 *
 * @passive_duration:    Duration to use for passive scans (in ms)
*/
struct conn_scan_dwell_info
{
	__le16  min_duration;
	__le16  max_duration;
	__le16  passive_duration;
} __packed ;

/**
 * struct conn_scan_ch_info - Channels info
 *
 * @channel:            channel number (channel_e)
 *
 * @tx_power_att:    TX power level in dbm
 *
 * @flags:       0 - DFS channel, 1 - DFS enabled (to be included in active scan)
*/
struct conn_scan_ch_info
{
	u8   channel;
	u8   tx_power_att;
	u8   flags;
} __packed;

/**
 * struct scan_one_shot_info - ONE_SHOT scan param
 *
 * @passive:           		Number of passive scan channels in bands BG,A
 *
 * @active:            		Number of active scan channels in bands BG,A
 *
 * @dfs:               		Number of DFS channels in A band
 *
 * @channel_list:           Channel list info
 *                          channels that are belonged to BG band are set from place 0 and forward.
 *                          channels that are belonged to A band are set from place CONN_SCAN_MAX_CHANNELS_BG (14) and forward.
 *                          channels that are belonged to 6Ghz band are set from place
 *                          CONN_SCAN_MAX_CHANNELS_A_BG(14+32) and forward.
 * @dwell_info:             Scan duration time info per band
 *
 * @reserved:
 *
*/
struct scan_one_shot_info
{
	u8  passive[CONN_SCAN_MAX_BAND];
	u8  active[CONN_SCAN_MAX_BAND];
	u8  dfs;

	struct conn_scan_ch_info    channel_list[ CONN_SCAN_MAX_CHANNELS_ALL_BANDS ];
	struct conn_scan_dwell_info dwell_info[CONN_SCAN_MAX_BAND];
	u8  reserved;
};

/**
 * sched_scan_plans - Scan plans for scheduled scan
 *
 * Each scan plan consists of the number of iterations to scan and the
 * interval between scans. When a scan plan finishes (i.e., it was run
 * for the specified number of iterations), the next scan plan is
 * executed. The scan plans are executed in the order they appear in
 * the array (lower index first). The last scan plan will run infinitely
 * (until requested to stop), thus must not specify the number of
 * iterations. All other scan plans must specify the number of
 * iterations.
 */
struct sched_scan_plan_cmd {
	u32 interval; /* In seconds */
	u32 iterations; /* Zero to run infinitely */
 } ;

 /**
 * struct periodicScanParams_t - Periodic scan param
 *
 * @sched_scan_plans:       Scan plans for a scheduled scan (defined in supplicant's driver.h)
 *                          interval and iterations
 *
 * @sched_scan_plans_num:    Number of scan plans in sched_scan_plans array
 *
 * @passive:           Number of passive scan channels in bands BG,A
 *
 * @active:            Number of active scan channels in bands BG,A
 *
 * @dfs:               number of DFS channels in A band
 *
 * @channel_list:            Channel list info
 *                          channels that are belonged to BG band are set from place 0 and forward.
 *                          channels that are belonged to A band are set from place CONN_SCAN_MAX_CHANNELS_BG (14) and forward.
 *                          channels that are belonged to 6Ghz band are set from place
 *                          CONN_SCAN_MAX_CHANNELS_A_BG(14+32) and forward.
 * @dwell_info:             Scan duration time info per band
 *
*/
struct scan_periodic_info
{
	struct sched_scan_plan_cmd  sched_scan_plans[SCAN_MAX_SCHED_SCAN_PLANS];
	u16 sched_scan_plans_num;

	u8 passive[CONN_SCAN_MAX_BAND];
	u8 active[CONN_SCAN_MAX_BAND];
	u8 dfs;

	struct conn_scan_ch_info      channel_list[ CONN_SCAN_MAX_CHANNELS_ALL_BANDS ];
	struct conn_scan_dwell_info   dwell_info[CONN_SCAN_MAX_BAND];
}__packed;

/**
 * struct scan_param - union for ONE_SHOT/PERIODIC scan param
 *
 * @one_shot:       ONE_SHOT scan param
 *
 * @periodic:       Periodic scan param
*/
struct scan_param
{
	union
	{
		struct scan_one_shot_info    one_shot;
		struct scan_periodic_info    periodic;
	} u;
}__packed;

/**
 * struct cc33xx_cmd_scan_params - scan configured param
 *
 * @scan_type:    		ONE_SHOT/PERIODIC scan
 *
 * @role_id:            role ID
 *
 * @params:         	Scan parameter for ONE_SHOT/PERIODIC Scan
 *
 * @rssi_threshold:     RSSI threshold for basic filter
 *
 * @snr_threshold:      SNR threshold for basic filter
 *
 * @bssid:              BSSID to scan for
 *
 * @ssid_from_list:     0 - if there are more than 5 SSIDs entries, (list was sent SSID CONFIGURE COMMAND),
 * 						1 - 5 or less SSIDs entries, the list is at the end of the scan command
 *
 * @filter:       		0 - not using filter and all the beacons/probe response frame
 *                      forward to upper mac,  1 - using filter
 *
 * @num_of_ssids: 		Number of SSIDs
*/
struct cc33xx_cmd_scan_params{
	struct cc33xx_cmd_header header;
	u8 scan_type;
	u8 role_id;

	struct scan_param   params;
	s8 rssi_threshold; /* for filtering (in dBm) */
	s8 snr_threshold;  /* for filtering (in dB) */

	u8 bssid[ETH_ALEN];
	u8 padding[2];

	u8 ssid_from_list; /* use ssid from configured ssid list */
	u8 filter;         /* forward only results with matching ssids */

	u8 num_of_ssids;
} __packed;

/******************************************************************************
        ID:     CMD_SET_PROBE_IE
        Desc:   This command will  set the Info elements data for
                probe request
        Return: CMD_COMPLETE
******************************************************************************/
#define MAX_EXTRA_IES_LEN 512
/**
 * struct cc33xx_cmd_set_ies - Probe request info elements
 *
 * @scan_type:    ONE_SHOT/PERIODIC scan
 *
 * @role_id:      roleID
 *
 * @data:         info element buffer
 *
 * @len:          info element length
*/
struct cc33xx_cmd_set_ies{
	struct cc33xx_cmd_header header;
	u8 scan_type;
	u8 role_id;
	__le16 len;
	u8                   data[MAX_EXTRA_IES_LEN];
} __packed;

/******************************************************************************
        ID:     CMD_STOP_SCAN
        Desc:   This command will stop scan process depending scan request
                type, and if early termination is on
        Return: CMD_COMPLETE
******************************************************************************/
/**
 * struct cc33xx_cmd_scan_stop - scan stop param
 *
 * @scan_type:           Scan request type
 *
 * @role_id:             role ID
 *
 * @is_ET:               TRUE - Early termination is on, FALSE - no ET
*/
struct cc33xx_cmd_scan_stop {
	struct cc33xx_cmd_header header;

	u8 scan_type;
	u8 role_id;
	u8 is_ET;
	u8 padding;
} __packed;


int cc33xx_scan_stop(struct cc33xx *wl, struct cc33xx_vif *wlvif);
void cc33xx_scan_completed(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_sched_scan_start(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			    struct cfg80211_sched_scan_request *req,
			    struct ieee80211_scan_ies *ies);
void cc33xx_scan_sched_scan_stop(struct cc33xx *wl, struct cc33xx_vif *wlvif);

int wlcore_scan(struct cc33xx *wl, struct ieee80211_vif *vif,
		const u8 *ssid, size_t ssid_len,
		struct cfg80211_scan_request *req);
void cc33xx_scan_complete_work(struct work_struct *work);
void wlcore_scan_sched_scan_results(struct cc33xx *wl);

enum {
	SCAN_SSID_FILTER_ANY      = 0,
	SCAN_SSID_FILTER_SPECIFIC = 1,
	SCAN_SSID_FILTER_LIST     = 2,
	SCAN_SSID_FILTER_DISABLED = 3
};

#define SCAN_CHANNEL_FLAGS_DFS		BIT(0) /* channel is passive until an
						  activity is detected on it */
#define SCAN_CHANNEL_FLAGS_DFS_ENABLED	BIT(1)

struct wlcore_scan_channels {
	u8 passive[SCAN_MAX_BANDS]; /* number of passive scan channels */
	u8 active[SCAN_MAX_BANDS];  /* number of active scan channels */
	u8 dfs;		   /* number of dfs channels in 5ghz */
	u8 passive_active; /* number of passive before active channels 2.4ghz */

	struct conn_scan_ch_params channels_2[MAX_CHANNELS_2GHZ];
	struct conn_scan_ch_params channels_5[MAX_CHANNELS_5GHZ];
	struct conn_scan_ch_params channels_4[MAX_CHANNELS_4GHZ];
};

enum {
	SCAN_TYPE_SEARCH	= 0,
	SCAN_TYPE_PERIODIC	= 1,
	SCAN_TYPE_TRACKING	= 2,
};

#endif /* __CC33XX_SCAN_H__ */
