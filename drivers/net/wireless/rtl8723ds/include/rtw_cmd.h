/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#ifndef __RTW_CMD_H_
#define __RTW_CMD_H_


#define C2H_MEM_SZ (16*1024)

#define FREE_CMDOBJ_SZ	128

#define MAX_CMDSZ	1536
#define MAX_RSPSZ	512
#define MAX_EVTSZ	1024

#define CMDBUFF_ALIGN_SZ 512

struct cmd_obj {
	_adapter *padapter;
	u16	cmdcode;
	u8	res;
	u8	*parmbuf;
	u32	cmdsz;
	u8	*rsp;
	u32	rspsz;
	struct submit_ctx *sctx;
	u8 no_io;
	/* _sema 	cmd_sem; */
	_list	list;
};

/* cmd flags */
enum {
	RTW_CMDF_DIRECTLY = BIT0,
	RTW_CMDF_WAIT_ACK = BIT1,
};

struct cmd_priv {
	_sema	cmd_queue_sema;
	/* _sema	cmd_done_sema; */
	_sema	start_cmdthread_sema;

	_queue	cmd_queue;
	u8	cmd_seq;
	u8	*cmd_buf;	/* shall be non-paged, and 4 bytes aligned */
	u8	*cmd_allocated_buf;
	u8	*rsp_buf;	/* shall be non-paged, and 4 bytes aligned		 */
	u8	*rsp_allocated_buf;
	u32	cmd_issued_cnt;
	u32	cmd_done_cnt;
	u32	rsp_cnt;
	ATOMIC_T cmdthd_running;
	/* u8 cmdthd_running; */

	_adapter *padapter;
	_mutex sctx_mutex;
};

#ifdef CONFIG_EVENT_THREAD_MODE
struct evt_obj {
	u16	evtcode;
	u8	res;
	u8	*parmbuf;
	u32	evtsz;
	_list	list;
};
#endif

struct	evt_priv {
#ifdef CONFIG_EVENT_THREAD_MODE
	_sema	evt_notify;

	_queue	evt_queue;
#endif

#ifdef CONFIG_FW_C2H_REG
	#define CONFIG_C2H_WK
#endif

#ifdef CONFIG_C2H_WK
	_workitem c2h_wk;
	bool c2h_wk_alive;
	struct rtw_cbuf *c2h_queue;
	#define C2H_QUEUE_MAX_LEN 10
#endif

#ifdef CONFIG_H2CLBK
	_sema	lbkevt_done;
	u8	lbkevt_limit;
	u8	lbkevt_num;
	u8	*cmdevt_parm;
#endif
	ATOMIC_T event_seq;
	u8	*evt_buf;	/* shall be non-paged, and 4 bytes aligned		 */
	u8	*evt_allocated_buf;
	u32	evt_done_cnt;
#if defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
	u8	*c2h_mem;
	u8	*allocated_c2h_mem;
#endif

};

#define init_h2fwcmd_w_parm_no_rsp(pcmd, pparm, code) \
	do {\
		_rtw_init_listhead(&pcmd->list);\
		pcmd->cmdcode = code;\
		pcmd->parmbuf = (u8 *)(pparm);\
		pcmd->cmdsz = sizeof (*pparm);\
		pcmd->rsp = NULL;\
		pcmd->rspsz = 0;\
	} while (0)

#define init_h2fwcmd_w_parm_no_parm_rsp(pcmd, code) \
	do {\
		_rtw_init_listhead(&pcmd->list);\
		pcmd->cmdcode = code;\
		pcmd->parmbuf = NULL;\
		pcmd->cmdsz = 0;\
		pcmd->rsp = NULL;\
		pcmd->rspsz = 0;\
	} while (0)

struct P2P_PS_Offload_t {
	u8 Offload_En:1;
	u8 role:1; /* 1: Owner, 0: Client */
	u8 CTWindow_En:1;
	u8 NoA0_En:1;
	u8 NoA1_En:1;
	u8 AllStaSleep:1; /* Only valid in Owner */
	u8 discovery:1;
	u8 rsvd:1;
#ifdef CONFIG_P2P_PS_NOA_USE_MACID_SLEEP
	u8 p2p_macid:7;
	u8 disable_close_rf:1; /*1: not close RF but just pause p2p_macid when NoA duration*/
#endif /* CONFIG_P2P_PS_NOA_USE_MACID_SLEEP */
};

struct P2P_PS_CTWPeriod_t {
	u8 CTWPeriod;	/* TU */
};

#ifdef CONFIG_P2P_WOWLAN

struct P2P_WoWlan_Offload_t {
	u8 Disconnect_Wkup_Drv:1;
	u8 role:2;
	u8 Wps_Config[2];
};

#endif /* CONFIG_P2P_WOWLAN */

extern u32 rtw_enqueue_cmd(struct cmd_priv *pcmdpriv, struct cmd_obj *obj);
extern struct cmd_obj *rtw_dequeue_cmd(struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_obj(struct cmd_obj *pcmd);

#ifdef CONFIG_EVENT_THREAD_MODE
extern u32 rtw_enqueue_evt(struct evt_priv *pevtpriv, struct evt_obj *obj);
extern struct evt_obj *rtw_dequeue_evt(_queue *queue);
extern void rtw_free_evt_obj(struct evt_obj *pcmd);
#endif

void rtw_stop_cmd_thread(_adapter *adapter);
thread_return rtw_cmd_thread(thread_context context);

extern u32 rtw_init_cmd_priv(struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_priv(struct cmd_priv *pcmdpriv);

extern u32 rtw_init_evt_priv(struct evt_priv *pevtpriv);
extern void rtw_free_evt_priv(struct evt_priv *pevtpriv);
extern void rtw_cmd_clr_isr(struct cmd_priv *pcmdpriv);
extern void rtw_evt_notify_isr(struct evt_priv *pevtpriv);
#ifdef CONFIG_P2P
u8 p2p_protocol_wk_cmd(_adapter *padapter, int intCmdType);
#endif /* CONFIG_P2P */

#ifdef CONFIG_IOCTL_CFG80211
struct rtw_roch_parm {
	u64 cookie;
	struct wireless_dev *wdev;
	struct ieee80211_channel ch;
	enum nl80211_channel_type ch_type;
	unsigned int duration;
};

u8 rtw_roch_cmd(_adapter *adapter
	, u64 cookie, struct wireless_dev *wdev
	, struct ieee80211_channel *ch, enum nl80211_channel_type ch_type
	, unsigned int duration
	, u8 flags
);

u8 rtw_cancel_roch_cmd(_adapter *adapter, u64 cookie, struct wireless_dev *wdev, u8 flags);

u8 rtw_mgnt_tx_cmd(_adapter *adapter, u8 tx_ch, u8 no_cck, const u8 *buf, size_t len, int wait_ack, u8 flags);
struct mgnt_tx_parm {
	u8 tx_ch;
	u8 no_cck;
	const u8 *buf;
	size_t len;
	int wait_ack;
};
#endif

enum rtw_drvextra_cmd_id {
	NONE_WK_CID,
	STA_MSTATUS_RPT_WK_CID,
	DYNAMIC_CHK_WK_CID,
	DM_CTRL_WK_CID,
	PBC_POLLING_WK_CID,
	POWER_SAVING_CTRL_WK_CID,/* IPS,AUTOSuspend */
	LPS_CTRL_WK_CID,
	ANT_SELECT_WK_CID,
	P2P_PS_WK_CID,
	P2P_PROTO_WK_CID,
	CHECK_HIQ_WK_CID,/* for softap mode, check hi queue if empty */
	C2H_WK_CID,
	RTP_TIMER_CFG_WK_CID,
	RESET_SECURITYPRIV, /* add for CONFIG_IEEE80211W, none 11w also can use */
	FREE_ASSOC_RESOURCES, /* add for CONFIG_IEEE80211W, none 11w also can use */
	DM_IN_LPS_WK_CID,
	DM_RA_MSK_WK_CID, /* add for STA update RAMask when bandwith change. */
	BEAMFORMING_WK_CID,
	LPS_CHANGE_DTIM_CID,
	BTINFO_WK_CID,
	BTC_REDUCE_WL_TXPWR_CID,
	DFS_RADAR_DETECT_WK_CID,
	DFS_RADAR_DETECT_EN_DEC_WK_CID,
	SESSION_TRACKER_WK_CID,
	EN_HW_UPDATE_TSF_WK_CID,
	PERIOD_TSF_UPDATE_END_WK_CID,
	TEST_H2C_CID,
	MP_CMD_WK_CID,
	CUSTOMER_STR_WK_CID,
#ifdef CONFIG_RTW_REPEATER_SON
	RSON_SCAN_WK_CID,
#endif
	ROCH_WK_CID,
	MGNT_TX_WK_CID,
	REQ_PER_CMD_WK_CID,
	SSMPS_WK_CID,
#ifdef CONFIG_CTRL_TXSS_BY_TP
	TXSS_WK_CID,
#endif
	AC_PARM_CMD_WK_CID,
#ifdef CONFIG_AP_MODE
	STOP_AP_WK_CID,
#endif
#ifdef CONFIG_RTW_TOKEN_BASED_XMIT
	TBTX_CONTROL_TX_WK_CID,
#endif
	MAX_WK_CID
};

enum LPS_CTRL_TYPE {
	LPS_CTRL_SCAN = 0,
	LPS_CTRL_JOINBSS = 1,
	LPS_CTRL_CONNECT = 2,
	LPS_CTRL_DISCONNECT = 3,
	LPS_CTRL_SPECIAL_PACKET = 4,
	LPS_CTRL_LEAVE = 5,
	LPS_CTRL_TRAFFIC_BUSY = 6,
	LPS_CTRL_TX_TRAFFIC_LEAVE = 7,
	LPS_CTRL_RX_TRAFFIC_LEAVE = 8,
	LPS_CTRL_ENTER = 9,
	LPS_CTRL_LEAVE_CFG80211_PWRMGMT = 10,
	LPS_CTRL_LEAVE_SET_OPTION = 11,
};

enum STAKEY_TYPE {
	GROUP_KEY		= 0,
	UNICAST_KEY		= 1,
	TDLS_KEY		= 2,
};

enum RFINTFS {
	SWSI,
	HWSI,
	HWPI,
};


/*
Caller Mode: Infra, Ad-Hoc

Notes: To join the specified bss

Command Event Mode

*/
struct joinbss_parm {
	WLAN_BSSID_EX network;
};

/*
Caller Mode: Infra, Ad-HoC(C)

Notes: To disconnect the current associated BSS

Command Mode

*/
struct disconnect_parm {
	u32 deauth_timeout_ms;
};

/*
Caller Mode: AP, Ad-HoC(M)

Notes: To create a BSS

Command Mode
*/
struct createbss_parm {
	bool adhoc;

	/* used by AP/Mesh mode now */
	u8 ifbmp;
	u8 excl_ifbmp;
	s16 req_ch;
	s8 req_bw;
	s8 req_offset;
};


struct	setopmode_parm {
	u8	mode;
	u8	rsvd[3];
};

/*
Caller Mode: AP, Ad-HoC, Infra

Notes: To ask RTL8711 performing site-survey

Command-Event Mode

*/

#define RTW_SSID_SCAN_AMOUNT 9 /* for WEXT_CSCAN_AMOUNT 9 */
#define RTW_CHANNEL_SCAN_AMOUNT (14+37)
struct sitesurvey_parm {
	sint scan_mode;	/* active: 1, passive: 0 */
	/* sint bsslimit;	// 1 ~ 48 */
	u8 ssid_num;
	u8 ch_num;
	NDIS_802_11_SSID ssid[RTW_SSID_SCAN_AMOUNT];
	struct rtw_ieee80211_channel ch[RTW_CHANNEL_SCAN_AMOUNT];

	u32 token; 	/* 80211k use it to identify caller */
	u16 duration;	/* 0: use default, otherwise: channel scan time */
	u8 igi;		/* 0: use defalut */
	u8 bw;		/* 0: use default */

	bool acs; /* aim to trigger channel selection when scan done */
	u8 reason;
};

/*
Caller Mode: Any

Notes: To set the auth type of RTL8711. open/shared/802.1x

Command Mode

*/
struct setauth_parm {
	u8 mode;  /* 0: legacy open, 1: legacy shared 2: 802.1x */
	u8 _1x;   /* 0: PSK, 1: TLS */
	u8 rsvd[2];
};

/*
Caller Mode: Infra

a. algorithm: wep40, wep104, tkip & aes
b. keytype: grp key/unicast key
c. key contents

when shared key ==> keyid is the camid
when 802.1x ==> keyid [0:1] ==> grp key
when 802.1x ==> keyid > 2 ==> unicast key

*/
struct setkey_parm {
	u8	algorithm;	/* encryption algorithm, could be none, wep40, TKIP, CCMP, wep104 */
	u8	keyid;
	u8	set_tx;		/* 1: main tx key for wep. 0: other key. */
	u8	key[32];	/* this could be 40 or 104 */
};

/*
When in AP or Ad-Hoc mode, this is used to
allocate an sw/hw entry for a newly associated sta.

Command

when shared key ==> algorithm/keyid

*/
struct set_stakey_parm {
	u8 addr[ETH_ALEN];
	u8 algorithm;
	u8 keyid;
	u8 key[32];
	u8 gk;
};

struct set_stakey_rsp {
	u8	addr[ETH_ALEN];
	u8	keyid;
	u8	rsvd;
};

struct Tx_Beacon_param {
	WLAN_BSSID_EX network;
};

/*
	Notes: This command is used for H2C/C2H loopback testing

	mac[0] == 0
	==> CMD mode, return H2C_SUCCESS.
	The following condition must be ture under CMD mode
		mac[1] == mac[4], mac[2] == mac[3], mac[0]=mac[5]= 0;
		s0 == 0x1234, s1 == 0xabcd, w0 == 0x78563412, w1 == 0x5aa5def7;
		s2 == (b1 << 8 | b0);

	mac[0] == 1
	==> CMD_RSP mode, return H2C_SUCCESS_RSP

	The rsp layout shall be:
	rsp:			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   mac[3];
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   s1;
		s1		=   swap16(s0);
		w0		=	swap32(w1);
		b0		=	b1
		s2		=	s0 + s1
		b1		=	b0
		w1		=	w0

	mac[0] ==	2
	==> CMD_EVENT mode, return	H2C_SUCCESS
	The event layout shall be:
	event:			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   event's sequence number, starting from 1 to parm's marc[3]
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   swap16(s0) - event.mac[2];
		s1		=   s1 + event.mac[2];
		w0		=	swap32(w0);
		b0		=	b1
		s2		=	s0 + event.mac[2]
		b1		=	b0
		w1		=	swap32(w1) - event.mac[2];

		parm->mac[3] is the total event counts that host requested.


	event will be the same with the cmd's param.

*/

#ifdef CONFIG_H2CLBK

struct seth2clbk_parm {
	u8 mac[6];
	u16	s0;
	u16	s1;
	u32	w0;
	u8	b0;
	u16  s2;
	u8	b1;
	u32	w1;
};

struct geth2clbk_parm {
	u32 rsv;
};

struct geth2clbk_rsp {
	u8	mac[6];
	u16	s0;
	u16	s1;
	u32	w0;
	u8	b0;
	u16	s2;
	u8	b1;
	u32	w1;
};

#endif	/* CONFIG_H2CLBK */

/* CMD param Formart for driver extra cmd handler */
struct drvextra_cmd_parm {
	int ec_id; /* extra cmd id */
	int type; /* Can use this field as the type id or command size */
	int size; /* buffer size */
	unsigned char *pbuf;
};

/*------------------- Below are used for RF/BB tunning ---------------------*/
struct addBaReq_parm {
	unsigned int tid;
	u8	addr[ETH_ALEN];
};

struct addBaRsp_parm {
	unsigned int tid;
	unsigned int start_seq;
	u8 addr[ETH_ALEN];
	u8 status;
	u8 size;
};

struct set_ch_parm {
	u8 ch;
	u8 bw;
	u8 ch_offset;
};

struct SetChannelPlan_param {
	enum regd_src_t regd_src;
	const struct country_chplan *country_ent;
	u8 channel_plan;
};

struct get_channel_plan_param {
	struct get_chplan_resp **resp;
};

struct LedBlink_param {
	void *pLed;
};

struct TDLSoption_param {
	u8 addr[ETH_ALEN];
	u8 option;
};

struct RunInThread_param {
	void (*func)(void *);
	void *context;
};

#ifdef CONFIG_WRITE_BCN_LEN_TO_FW
struct write_bcnlen_param {
	u16 bcn_len;
};
#endif

#define GEN_CMD_CODE(cmd)	cmd ## _CMD_


/*

Result:
0x00: success
0x01: sucess, and check Response.
0x02: cmd ignored due to duplicated sequcne number
0x03: cmd dropped due to invalid cmd code
0x04: reserved.

*/

#define H2C_RSP_OFFSET			512

#define H2C_SUCCESS			0x00
#define H2C_SUCCESS_RSP			0x01
#define H2C_DUPLICATED			0x02
#define H2C_DROPPED			0x03
#define H2C_PARAMETERS_ERROR		0x04
#define H2C_REJECTED			0x05
#define H2C_CMD_OVERFLOW		0x06
#define H2C_RESERVED			0x07
#define H2C_ENQ_HEAD			0x08
#define H2C_ENQ_HEAD_FAIL		0x09
#define H2C_CMD_FAIL			0x0A

void rtw_init_sitesurvey_parm(_adapter *padapter, struct sitesurvey_parm *pparm);
u8 rtw_sitesurvey_cmd(_adapter *padapter, struct sitesurvey_parm *pparm);
#ifdef CONFIG_AP_MODE
u8 rtw_create_ibss_cmd(_adapter *adapter, int flags);
u8 rtw_startbss_cmd(_adapter *adapter, int flags);
#endif

#define REQ_CH_NONE		-1
#define REQ_CH_INT_INFO	-2
#define REQ_BW_NONE		-1
#define REQ_BW_ORI		-2
#define REQ_OFFSET_NONE	-1

struct sta_info;
extern u8 rtw_setstakey_cmd(_adapter  *padapter, struct sta_info *sta, u8 key_type, bool enqueue);
extern u8 rtw_clearstakey_cmd(_adapter *padapter, struct sta_info *sta, u8 enqueue);

extern u8 rtw_joinbss_cmd(_adapter  *padapter, struct wlan_network *pnetwork);
u8 rtw_disassoc_cmd(_adapter *padapter, u32 deauth_timeout_ms, int flags);
#ifdef CONFIG_AP_MODE
u8 rtw_change_bss_chbw_cmd(_adapter *adapter, int flags
	, u8 ifbmp, u8 excl_ifbmp, s16 req_ch, s8 req_bw, s8 req_offset);
u8 rtw_stop_ap_cmd(_adapter *adapter, u8 flags);
#endif
#ifdef CONFIG_RTW_TOKEN_BASED_XMIT
u8 rtw_tx_control_cmd(_adapter *adapter);
#endif
extern u8 rtw_setopmode_cmd(_adapter  *padapter, NDIS_802_11_NETWORK_INFRASTRUCTURE networktype, u8 flags);

extern u8 rtw_addbareq_cmd(_adapter *padapter, u8 tid, u8 *addr);
extern u8 rtw_addbarsp_cmd(_adapter *padapter, u8 *addr, u16 tid, u8 status, u8 size, u16 start_seq);
/* add for CONFIG_IEEE80211W, none 11w also can use */
extern u8 rtw_reset_securitypriv_cmd(_adapter *padapter);
extern u8 rtw_free_assoc_resources_cmd(_adapter *padapter, u8 lock_scanned_queue, int flags);
extern u8 rtw_dynamic_chk_wk_cmd(_adapter *adapter);

u8 rtw_lps_ctrl_wk_cmd(_adapter *padapter, u8 lps_ctrl_type, u8 flags);
u8 rtw_lps_ctrl_leave_set_level_cmd(_adapter *adapter, u8 lps_level, u8 flags);
#ifdef CONFIG_LPS_1T1R
u8 rtw_lps_ctrl_leave_set_1t1r_cmd(_adapter *adapter, u8 lps_1t1r, u8 flags);
#endif
u8 rtw_dm_in_lps_wk_cmd(_adapter *padapter);
u8 rtw_lps_change_dtim_cmd(_adapter *padapter, u8 dtim);

#if (RATE_ADAPTIVE_SUPPORT == 1)
u8 rtw_rpt_timer_cfg_cmd(_adapter *padapter, u16 minRptTime);
#endif

#ifdef CONFIG_ANTENNA_DIVERSITY
extern  u8 rtw_antenna_select_cmd(_adapter *padapter, u8 antenna, u8 enqueue);
#endif

u8 rtw_dm_ra_mask_wk_cmd(_adapter *padapter, u8 *psta);

extern u8 rtw_ps_cmd(_adapter *padapter);

#if CONFIG_DFS
void rtw_dfs_ch_switch_hdl(struct dvobj_priv *dvobj);
#endif

#ifdef CONFIG_AP_MODE
u8 rtw_chk_hi_queue_cmd(_adapter *padapter);
#ifdef CONFIG_DFS_MASTER
u8 rtw_dfs_rd_cmd(_adapter *adapter, bool enqueue);
void rtw_dfs_rd_timer_hdl(void *ctx);
void rtw_dfs_rd_en_decision(_adapter *adapter, u8 mlme_act, u8 excl_ifbmp);
u8 rtw_dfs_rd_en_decision_cmd(_adapter *adapter);
#endif /* CONFIG_DFS_MASTER */
#endif /* CONFIG_AP_MODE */

#ifdef CONFIG_BT_COEXIST
u8 rtw_btinfo_cmd(PADAPTER padapter, u8 *pbuf, u16 length);
u8 rtw_btc_reduce_wl_txpwr_cmd(_adapter *adapter, u32 val);
#endif

u8 rtw_test_h2c_cmd(_adapter *adapter, u8 *buf, u8 len);

u8 rtw_enable_hw_update_tsf_cmd(_adapter *padapter);
u8 rtw_periodic_tsf_update_end_cmd(_adapter *adapter);

u8 rtw_set_chbw_cmd(_adapter *padapter, u8 ch, u8 bw, u8 ch_offset, u8 flags);
u8 rtw_iqk_cmd(_adapter *padapter, u8 flags);

u8 rtw_set_chplan_cmd(_adapter *adapter, int flags, u8 chplan, u8 swconfig);
u8 rtw_set_country_cmd(_adapter *adapter, int flags, const char *country_code, u8 swconfig);
#ifdef CONFIG_REGD_SRC_FROM_OS
u8 rtw_sync_os_regd_cmd(_adapter *adapter, int flags, const char *country_code, u8 dfs_region);
#endif
u8 rtw_get_chplan_cmd(_adapter *adapter, int flags, struct get_chplan_resp **resp);

extern u8 rtw_led_blink_cmd(_adapter *padapter, void *pLed);
extern u8 rtw_set_csa_cmd(_adapter *adapter);
extern u8 rtw_tdls_cmd(_adapter *padapter, u8 *addr, u8 option);

u8 rtw_mp_cmd(_adapter *adapter, u8 mp_cmd_id, u8 flags);

#ifdef CONFIG_RTW_CUSTOMER_STR
u8 rtw_customer_str_req_cmd(_adapter *adapter);
u8 rtw_customer_str_write_cmd(_adapter *adapter, const u8 *cstr);
#endif

#ifdef CONFIG_FW_C2H_REG
u8 rtw_c2h_reg_wk_cmd(_adapter *adapter, u8 *c2h_evt);
#endif
#ifdef CONFIG_FW_C2H_PKT
u8 rtw_c2h_packet_wk_cmd(_adapter *adapter, u8 *c2h_evt, u16 length);
#endif

#ifdef CONFIG_RTW_REPEATER_SON
#define RSON_SCAN_PROCESS		10
#define RSON_SCAN_DISABLE		11
u8 rtw_rson_scan_wk_cmd(_adapter *adapter, int op);
#endif

u8 rtw_run_in_thread_cmd(_adapter *adapter, void (*func)(void *), void *context);
u8 rtw_run_in_thread_cmd_wait(_adapter *adapter, void (*func)(void *), void *context, s32 timeout_ms);

struct ssmps_cmd_parm {
	struct sta_info *sta;
	u8 smps;
};
u8 rtw_ssmps_wk_cmd(_adapter *adapter, struct sta_info *sta, u8 smps, u8 enqueue);

u8 session_tracker_chk_cmd(_adapter *adapter, struct sta_info *sta);
u8 session_tracker_add_cmd(_adapter *adapter, struct sta_info *sta, u8 *local_naddr, u8 *local_port, u8 *remote_naddr, u8 *remote_port);
u8 session_tracker_del_cmd(_adapter *adapter, struct sta_info *sta, u8 *local_naddr, u8 *local_port, u8 *remote_naddr, u8 *remote_port);

u8 set_txq_params_cmd(_adapter *adapter, u32 ac_parm, u8 ac_type);

#if defined(CONFIG_RTW_MESH) && defined(RTW_PER_CMD_SUPPORT_FW)
u8 rtw_req_per_cmd(_adapter * adapter);
#endif
#ifdef CONFIG_RTW_TOKEN_BASED_XMIT
u8 rtw_tbtx_chk_cmd(_adapter *adapter);
u8 rtw_tbtx_token_dispatch_cmd(_adapter *adapter);
#endif

#ifdef CONFIG_WRITE_BCN_LEN_TO_FW
u8 rtw_write_bcnlen_to_fw_cmd(_adapter *padapter, u16 bcn_len);
#endif

#ifdef CONFIG_CTRL_TXSS_BY_TP
struct txss_cmd_parm {
	struct sta_info *sta;
	bool tx_1ss;
};

void rtw_ctrl_txss_update_mimo_type(_adapter *adapter, struct sta_info *sta);
u8 rtw_ctrl_txss(_adapter *adapter, struct sta_info *sta, bool tx_1ss);
void rtw_ctrl_tx_ss_by_tp(_adapter *adapter, u8 from_timer);

#ifdef DBG_CTRL_TXSS
void dbg_ctrl_txss(_adapter *adapter, bool tx_1ss);
#endif
#endif

u8 rtw_drvextra_cmd_hdl(_adapter *padapter, unsigned char *pbuf);

extern void rtw_survey_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_disassoc_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
extern void rtw_joinbss_cmd_callback(_adapter  *padapter, struct cmd_obj *pcmd);
void rtw_create_ibss_post_hdl(_adapter *padapter, int status);
extern void rtw_readtssi_cmdrsp_callback(_adapter	*padapter,  struct cmd_obj *pcmd);

extern void rtw_setstaKey_cmdrsp_callback(_adapter  *padapter,  struct cmd_obj *pcmd);
extern void rtw_getrttbl_cmdrsp_callback(_adapter  *padapter,  struct cmd_obj *pcmd);

enum rtw_cmd_id {
	CMD_JOINBSS, /*0*/
	CMD_DISCONNECT, /*1*/
	CMD_CREATE_BSS,/*2*/
	CMD_SET_OPMODE, /*3*/
	CMD_SITE_SURVEY, /*4*/
	CMD_SET_AUTH, /*5*/
	CMD_SET_KEY, /*6*/
	CMD_SET_STAKEY, /*7*/
	CMD_ADD_BAREQ, /*8*/
	CMD_SET_CHANNEL, /*9*/
	CMD_TX_BEACON, /*10*/
	CMD_SET_MLME_EVT, /*11*/
	CMD_SET_DRV_EXTRA, /*12*/
	CMD_SET_CHANPLAN, /*13*/
	CMD_LEDBLINK, /*14*/
	CMD_SET_CHANSWITCH, /*15*/
	CMD_TDLS, /*16*/
	CMD_CHK_BMCSLEEPQ,  /*17*/
	CMD_RUN_INTHREAD, /*18*/
	CMD_ADD_BARSP, /*19*/
	CMD_RM_POST_EVENT, /*20*/
	CMD_SET_MESH_PLINK_STATE, /* 21 */
	CMD_DO_IQK, /* 22 */
	CMD_GET_CHANPLAN, /*23*/
	CMD_WRITE_BCN_LEN, /*24 */
	CMD_ID_MAX
};

#define CMD_FMT "cmd=%d,%d,%d"
#define CMD_ARG(cmd) \
	(cmd)->cmdcode, \
	(cmd)->cmdcode == CMD_SET_DRV_EXTRA ? ((struct drvextra_cmd_parm *)(cmd)->parmbuf)->ec_id : ((cmd)->cmdcode == CMD_SET_MLME_EVT ? ((struct rtw_evt_header *)(cmd)->parmbuf)->id : 0), \
	(cmd)->cmdcode == CMD_SET_DRV_EXTRA ? ((struct drvextra_cmd_parm *)(cmd)->parmbuf)->type : 0

#endif /* _CMD_H_ */
