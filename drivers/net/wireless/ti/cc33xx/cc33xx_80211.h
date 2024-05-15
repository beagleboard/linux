/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CC33XX_80211_H__
#define __CC33XX_80211_H__

#include <linux/if_arp.h>


/* Headers */
struct ieee80211_header {
	__le16 frame_ctl;
	__le16 duration_id;
	u8 da[ETH_ALEN];
	u8 sa[ETH_ALEN];
	u8 bssid[ETH_ALEN];
	__le16 seq_ctl;
	u8 payload[0];
} __packed;

/* Templates */

struct cc33xx_arp_rsp_template {
	/* not including ieee80211 header */

	u8 llc_hdr[sizeof(rfc1042_header)];
	__be16 llc_type;

	struct arphdr arp_hdr;
	u8 sender_hw[ETH_ALEN];
	__be32 sender_ip;
	u8 target_hw[ETH_ALEN];
	__be32 target_ip;
} __packed;

struct cc33xx_disconn_template {
	struct ieee80211_header header;
	__le16 disconn_reason;
} __packed;


#endif /* __CC33XX_80211_H__ */
