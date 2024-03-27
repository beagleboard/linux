/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2011 Texas Instruments Inc.
 */

#ifndef __WLCORE_H__
#define __WLCORE_H__

#include <linux/platform_device.h>

#include "wlcore_i.h"
#include "event.h"
#include "boot.h"
#include "rx.h"


/* Wireless Driver Version */
#define MAJOR_VERSION 	1
#define MINOR_VERSION 	7
#define API_VERSION 	0
#define BUILD_VERSION	27


/* The maximum number of Tx descriptors in all chip families */
#define WLCORE_MAX_TX_DESCRIPTORS 32

#define CC33XX_CMD_MAX_SIZE          (896)
#define CC33XX_INI_PARAM_COMMAND_SIZE (16UL)//size of struct cc33xx_cmd_ini_params_download
#define CC33XX_INI_CMD_MAX_SIZE      (CC33X_CONF_SIZE + CC33XX_INI_PARAM_COMMAND_SIZE + sizeof(int))

#define CC33XX_CMD_BUFFER_SIZE ((CC33XX_INI_CMD_MAX_SIZE > CC33XX_CMD_MAX_SIZE) ? CC33XX_INI_CMD_MAX_SIZE : CC33XX_CMD_MAX_SIZE)

#define WLCORE_NUM_MAC_ADDRESSES 3

/* Texas Instruments pre assigned OUI */
#define WLCORE_TI_OUI_ADDRESS 0x080028

#define CC33XX_AGGR_BUFFER_SIZE		(8 * PAGE_SIZE)

#define CC33XX_NUM_TX_DESCRIPTORS 32
#define CC33XX_NUM_RX_DESCRIPTORS 32

#define CC33XX_RX_BA_MAX_SESSIONS 13

#define CC33XX_MAX_AP_STATIONS 16

/* forward declaration */
struct cc33xx_tx_hw_descr;
struct cc33xx_rx_descriptor;
struct partial_rx_frame;
struct core_fw_status;
struct core_status;

enum wl_rx_buf_align;

struct driver_versions{
	__u16 major_version;
	__u16 minor_version;
	__u16 api_version;
	__u16 build_version;
};

struct driver_fw_versions{
	struct driver_versions driver_ver;
	struct cc33xx_acx_fw_versions *fw_ver;

};

struct cc33xx_stats {
	void *fw_stats;
	unsigned long fw_stats_update;
	size_t fw_stats_len;

	unsigned int retry_count;
	unsigned int excessive_retries;
};


struct cc33xx {
	bool initialized;
	struct ieee80211_hw *hw;
	bool mac80211_registered;

	struct device *dev;
	struct platform_device *pdev;


	struct cc33xx_if_operations *if_ops;

	int wakeirq;


	spinlock_t wl_lock;

	enum wlcore_state state;
	bool plt;
	enum plt_mode plt_mode;
	u8 plt_role_id;
	u8 fem_manuf;
	u8 last_vif_count;
	struct mutex mutex;
	struct core_status *core_status;
	u8 last_fw_rls_idx;
	/* Temp O3 solution for transferring command results from IRQ
		to API-caller context. */
	u8 command_result[CC33XX_CMD_MAX_SIZE];
	u16 result_length;
	struct partial_rx_frame partial_rx;

	unsigned long flags;

	void *nvs_mac_addr;
	size_t nvs_mac_addr_len;
	struct cc33xx_fw_download *fw_download;

	struct mac_address addresses[WLCORE_NUM_MAC_ADDRESSES];

	unsigned long links_map[BITS_TO_LONGS(CC33XX_MAX_LINKS)];
	unsigned long roles_map[BITS_TO_LONGS(CC33XX_MAX_ROLES)];
	unsigned long roc_map[BITS_TO_LONGS(CC33XX_MAX_ROLES)];
	unsigned long rate_policies_map[
			BITS_TO_LONGS(CC33XX_MAX_RATE_POLICIES)];

	u8 session_ids[CC33XX_MAX_LINKS];

	struct list_head wlvif_list;

	u8 sta_count;
	u8 ap_count;

	struct cc33xx_acx_mem_map *target_mem_map;

	/* Accounting for allocated / available TX blocks on HW */

	u32 tx_blocks_available;
	u32 tx_allocated_blocks;

	/* Accounting for allocated / available Tx packets in HW */


	u32 tx_allocated_pkts[NUM_TX_QUEUES];


	/* Time-offset between host and chipset clocks */


	/* Frames scheduled for transmission, not handled yet */
	int tx_queue_count[NUM_TX_QUEUES];
	unsigned long queue_stop_reasons[
				NUM_TX_QUEUES * WLCORE_NUM_MAC_ADDRESSES];

	/* Frames received, not handled yet by mac80211 */
	struct sk_buff_head deferred_rx_queue;

	/* Frames sent, not returned yet to mac80211 */
	struct sk_buff_head deferred_tx_queue;

	struct work_struct tx_work;
	struct workqueue_struct *freezable_wq;

	/*freezable wq for netstack_work*/
	struct workqueue_struct *freezable_netstack_wq;

	/* Pending TX frames */
	unsigned long tx_frames_map[BITS_TO_LONGS(WLCORE_MAX_TX_DESCRIPTORS)];
	struct sk_buff *tx_frames[WLCORE_MAX_TX_DESCRIPTORS];
	int tx_frames_cnt;

	/* FW Rx counter */
	u32 rx_counter;

	/* Intermediate buffer, used for packet aggregation */
	u8 *aggr_buf;
	u32 aggr_buf_size;
	size_t max_transaction_len;

	/* Reusable dummy packet template */
	struct sk_buff *dummy_packet;

	/* Network stack work  */
	struct work_struct netstack_work;
	/* FW log buffer */
	u8 *fwlog;

	/* Number of valid bytes in the FW log buffer */
	ssize_t fwlog_size;

	/* Hardware recovery work */
	struct work_struct recovery_work;

	struct work_struct irq_deferred_work;

	/* Reg domain last configuration */
	DECLARE_BITMAP(reg_ch_conf_last, 64);
	/* Reg domain pending configuration */
	DECLARE_BITMAP(reg_ch_conf_pending, 64);

	/* Lock-less list for deferred event handling */
	struct llist_head event_list;
	/* The mbox event mask */
	u32 event_mask;
	/* events to unmask only when ap interface is up */
	u32 ap_event_mask;

	/* Are we currently scanning */
	struct cc33xx_vif *scan_wlvif;
	struct cc33xx_scan scan;
	struct delayed_work scan_complete_work;

	struct ieee80211_vif *roc_vif;
	struct delayed_work roc_complete_work;

	struct cc33xx_vif *sched_vif;

	u8 mac80211_scan_stopped;

	/* The current band */
	enum nl80211_band band;

	/* in dBm */
	int power_level;

	struct cc33xx_stats stats;

	__le32 *buffer_32;

	/* Current chipset configuration */
	struct cc33xx_conf_file conf;

	bool enable_11a;

	/* bands supported by this instance of cc33xx */
	struct ieee80211_supported_band bands[WLCORE_NUM_BANDS];

	/*
	 * wowlan trigger was configured during suspend.
	 * (currently, only "ANY" trigger is supported)
	 */
	bool keep_device_power;

	/*
	 * AP-mode - links indexed by HLID. The global and broadcast links
	 * are always active.
	 */
	struct cc33xx_link links[CC33XX_MAX_LINKS];

	/* number of currently active links */
	int active_link_count;

	/* AP-mode - a bitmap of links currently in PS mode according to FW */
	unsigned long ap_fw_ps_map;

	/* AP-mode - a bitmap of links currently in PS mode in mac80211 */
	unsigned long ap_ps_map;

	/* Quirks of specific hardware revisions */
	unsigned int quirks;

	/* number of currently active RX BA sessions */
	int ba_rx_session_count;

	/* Maximum number of supported RX BA sessions */
	int ba_rx_session_count_max;

	/* AP-mode - number of currently connected stations */
	int active_sta_count;

	/* last wlvif we transmitted from */
	struct cc33xx_vif *last_wlvif;

	/* work to fire when Tx is stuck */
	struct delayed_work tx_watchdog_work;

	u8 scan_templ_id_2_4;
	u8 scan_templ_id_5;
	u8 sched_scan_templ_id_2_4;
	u8 sched_scan_templ_id_5;
	u8 max_channels_5;

	/* number of TX descriptors the HW supports. */
	u32 num_tx_desc;
	/* number of RX descriptors the HW supports. */
	u32 num_rx_desc;
	/* number of links the HW supports */
	u8 num_links;
	/* max stations a single AP can support */
	u8 max_ap_stations;

	/* translate HW Tx rates to standard rate-indices */
	const u8 **band_rate_to_idx;

	/* size of table for HW rates that can be received from chip */
	u8 hw_tx_rate_tbl_size;

	/* HW HT (11n) capabilities */
	struct ieee80211_sta_ht_cap ht_cap[WLCORE_NUM_BANDS];

	/* the current dfs region */
	enum nl80211_dfs_regions dfs_region;
	bool radar_debug_mode;

	/* RX Data filter rule state - enabled/disabled */
	unsigned long rx_filter_enabled[BITS_TO_LONGS(CC33XX_MAX_RX_FILTERS)];//used in CONFIG PM AND W8 Code

	/* mutex for protecting the tx_flush function */
	struct mutex flush_mutex;

	/* sleep auth value currently configured to FW */
	int sleep_auth;

	/*ble_enable value - if 0 ble not enabled , if 1 is enabled..cant be disabled after enable*/
	int ble_enable;

	/* sta role index - if 0 - wlan0 primary station interface, if 1 - wlan2 - secondary station interface*/

	u8 sta_role_idx;

	u16 max_cmd_size;

	struct completion nvs_loading_complete;
	struct completion command_complete;

	/* interface combinations supported by the hw */
	const struct ieee80211_iface_combination *iface_combinations;
	u8 n_iface_combinations;

	/* dynamic fw traces */
	u32 dynamic_fw_traces;

	/* buffer for sending commands to FW */
	u8 cmd_buf[CC33XX_CMD_BUFFER_SIZE];

	/* number of keys requiring extra spare mem-blocks */
	int extra_spare_key_count;

	u8 efuse_mac_address[ETH_ALEN];

	u32 fuse_rom_structure_version;
	u32 device_part_number;
	u32 pg_version;
	u8	disable_5g;
	u8 	disable_6g;

	struct driver_fw_versions all_versions;

	char* all_versions_str;

	u8 antenna_selection;

	/* burst mode cfg */
	u8 burst_disable;

};

int wlcore_probe(struct cc33xx *wl, struct platform_device *pdev);
int wlcore_remove(struct platform_device *pdev);
struct ieee80211_hw *wlcore_alloc_hw(u32 aggr_buf_size);
int wlcore_free_hw(struct cc33xx *wl);
int wlcore_set_key(struct cc33xx *wl, enum set_key_cmd cmd,
		   struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta,
		   struct ieee80211_key_conf *key_conf);
void wlcore_regdomain_config(struct cc33xx *wl);
void wlcore_update_inconn_sta(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			      struct cc33xx_station *wl_sta, bool in_conn);
bool cc33xx_is_mimo_supported(struct cc33xx *wl);

static inline void
wlcore_set_ht_cap(struct cc33xx *wl, enum nl80211_band band,
		  struct ieee80211_sta_ht_cap *ht_cap)
{
	memcpy(&wl->ht_cap[band], ht_cap, sizeof(*ht_cap));
}

/* Tell wlcore not to care about this element when checking the version */
#define WLCORE_FW_VER_IGNORE	-1


/* Firmware image load chunk size */
#define CHUNK_SIZE	16384

/* Quirks */

/* Each RX/TX transaction requires an end-of-transaction transfer */
#define WLCORE_QUIRK_END_OF_TRANSACTION		BIT(0)

/* the first start_role(sta) sometimes doesn't work on wl12xx */
#define WLCORE_QUIRK_START_STA_FAILS		BIT(1)

/* wl127x and SPI don't support SDIO block size alignment */
#define WLCORE_QUIRK_TX_BLOCKSIZE_ALIGN		BIT(2)

/* means aggregated Rx packets are aligned to a SDIO block */
#define WLCORE_QUIRK_RX_BLOCKSIZE_ALIGN		BIT(3)

/* Older firmwares did not implement the FW logger over bus feature */
#define WLCORE_QUIRK_FWLOG_NOT_IMPLEMENTED	BIT(4)

/* Older firmwares use an old NVS format */
#define WLCORE_QUIRK_LEGACY_NVS			BIT(5)

/* pad only the last frame in the aggregate buffer */
#define WLCORE_QUIRK_TX_PAD_LAST_FRAME		BIT(7)

/* extra header space is required for TKIP */
#define WLCORE_QUIRK_TKIP_HEADER_SPACE		BIT(8)

/* Some firmwares not support sched scans while connected */
#define WLCORE_QUIRK_NO_SCHED_SCAN_WHILE_CONN	BIT(9)

/* separate probe response templates for one-shot and sched scans */
#define WLCORE_QUIRK_DUAL_PROBE_TMPL		BIT(10)

/* Firmware requires reg domain configuration for active calibration */
#define WLCORE_QUIRK_REGDOMAIN_CONF		BIT(11)

/* The FW only support a zero session id for AP */
#define WLCORE_QUIRK_AP_ZERO_SESSION_ID		BIT(12)

/* TODO: move all these common registers and values elsewhere */
#define HW_ACCESS_ELP_CTRL_REG		0x1FFFC

/* ELP register commands */
#define ELPCTRL_WAKE_UP             0x1
#define ELPCTRL_WAKE_UP_WLAN_READY  0x5
#define ELPCTRL_SLEEP               0x0
/* ELP WLAN_READY bit */
#define ELPCTRL_WLAN_READY          0x2

/*************************************************************************

    Interrupt Trigger Register (Host -> WiLink)

**************************************************************************/

/* Hardware to Embedded CPU Interrupts - first 32-bit register set */

/*
 * The host sets this bit to inform the Wlan
 * FW that a TX packet is in the XFER
 * Buffer #0.
 */
#define INTR_TRIG_TX_PROC0 BIT(2)

/*
 * The host sets this bit to inform the FW
 * that it read a packet from RX XFER
 * Buffer #0.
 */
#define INTR_TRIG_RX_PROC0 BIT(3)

#define INTR_TRIG_DEBUG_ACK BIT(4)

#define INTR_TRIG_STATE_CHANGED BIT(5)

/* Hardware to Embedded CPU Interrupts - second 32-bit register set */

/*
 * The host sets this bit to inform the FW
 * that it read a packet from RX XFER
 * Buffer #1.
 */
#define INTR_TRIG_RX_PROC1 BIT(17)

/*
 * The host sets this bit to inform the Wlan
 * hardware that a TX packet is in the XFER
 * Buffer #1.
 */
#define INTR_TRIG_TX_PROC1 BIT(18)

#define ACX_SLV_SOFT_RESET_BIT	BIT(1)
#define SOFT_RESET_MAX_TIME	1000000
#define SOFT_RESET_STALL_TIME	1000

#define ECPU_CONTROL_HALT	0x00000101

#define WELP_ARM_COMMAND_VAL	0x4

enum CC33xx_FRAME_FORMAT {
 CC33xx_B_SHORT = 0,
 CC33xx_B_LONG,
 CC33xx_LEGACY_OFDM,
 CC33xx_HT_MF,
 CC33xx_HT_GF,
 CC33xx_HE_SU,
 CC33xx_HE_MU,
 CC33xx_HE_SU_ER,
 CC33xx_HE_TB,
 CC33xx_HE_TB_NDP_FB,
 CC33xx_VHT
};

/* CC33xx HW Common Definitions */

#define HOST_SYNC_PATTERN 	0x5C5C5C5C
#define DEVICE_SYNC_PATTERN 0xABCDDCBA
#define NAB_DATA_ADDR		0x0000BFF0
#define NAB_CONTROL_ADDR	0x0000BFF8
#define NAB_STATUS_ADDR		0x0000BFFC


#define NAB_SEND_CMD        0x940d // 0x900D
#define NAB_SEND_FLAGS      0x08
#define CC33xx_INTERNAL_DESC_SIZE   200
#define NAB_EXTRA_BYTES 4

#define TX_RESULT_QUEUE_SIZE  108



struct control_info_descriptor
{
    __le16 type 	:4;
    __le16 length 	:12;
};

enum control_message_type
{
    CTRL_MSG_NONE = 0,
    CTRL_MSG_EVENT = 1,
    CTRL_MSG_COMMND_COMPLETE = 2
};

struct core_fw_status
{
    u8   txResultQueueIndex;
    u8   reserved1[3];
    u8   txResultQueue[TX_RESULT_QUEUE_SIZE];
    __le32	link_ps_bitmap;                     /* A bitmap (where each bit represents a single HLID) to indicate PS/Active mode of the link */
    __le32	link_fast_bitmap;                   /* A bitmap (where each bit represents a single HLID) to indicate if the station is in Fast mode */
    __le32	link_suspend_bitmap;                /* A bitmap (where each bit represents a single HLID) to indicate if a links is suspended/aboout to be suspended*/
    u8      TxFlowControlAcThreshold;           /* Host TX Flow Control descriptor per AC threshold */
    u8      tx_ps_threshold;                    /* Host TX Flow Control descriptor PS link threshold */
    u8      tx_suspend_threshold;               /* Host TX Flow Control descriptor Suspended link threshold */
    u8      tx_slow_link_prio_threshold;        /* Host TX Flow Control descriptor Slow link threshold */
    u8      tx_fast_link_prio_threshold;        /* Host TX Flow Control descriptor Fast link threshold */
    u8      tx_slow_stop_threshold;             /* Host TX Flow Control descriptor Stop Slow link threshold */
    u8      tx_fast_stop_threshold;             /* Host TX Flow Control descriptor Stop Fast link threshold */
    u8      reserved2;
    // Additional information can be added here
} __packed;

struct core_status {
    //__le32 bloc_pad[92];
    __le32 block_pad[28];
    __le32 host_interrupt_status;
    __le32 rx_status;
    struct core_fw_status fwInfo;
    __le32 tsf;
} __packed;

struct NAB_header{
	__le32 sync_pattern;
	__le16 opcode;
	__le16 len;
};

/* rx_status lower bytes hold the rx byte count */
#define RX_BYTE_COUNT_MASK 0xFFFF




#define HINT_NEW_TX_RESULT						0x1
#define HINT_COMMAND_COMPLETE 					0x2
#define HINT_RX_DATA_PENDING 					0x4
#define HINT_ROM_LOADER_INIT_COMPLETE 			0x8
#define HINT_SECOND_LOADER_INIT_COMPLETE 		0x10
#define HINT_FW_WAKEUP_COMPLETE 				0x20
#define HINT_FW_INIT_COMPLETE  					0x40
#define HINT_GENERAL_ERROR						0x80000000

#define BOOT_TIME_INTERRUPTS (\
	HINT_ROM_LOADER_INIT_COMPLETE    | \
	HINT_SECOND_LOADER_INIT_COMPLETE | \
	HINT_FW_WAKEUP_COMPLETE | \
	HINT_FW_INIT_COMPLETE )

struct NAB_tx_header{
    __le32 sync;
    __le16 opcode;
    __le16 len;
    __le16 desc_length;
    u8     sd;
    u8     flags;
} __packed;

struct NAB_rx_header{
    __le32 cnys;
    __le16 opcode;
    __le16 len;
    __le32 rx_desc;
    __le32 reserved;
} __packed;






#endif /* __WLCORE_H__ */
