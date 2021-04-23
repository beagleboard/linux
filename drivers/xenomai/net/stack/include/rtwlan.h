/* rtwlan.h
 *
 * This file is a rtnet adaption from ieee80211/ieee80211.h used by the
 * rt2x00-2.0.0-b3 sourceforge project
 *
 * Merged with mainline ieee80211.h in Aug 2004.  Original ieee802_11
 * remains copyright by the original authors
 *
 * Portions of the merged code are based on Host AP (software wireless
 * LAN access point) driver for Intersil Prism2/2.5/3.
 *
 * Copyright (c) 2001-2002, SSH Communications Security Corp and Jouni Malinen
 * <jkmaline@cc.hut.fi>
 * Copyright (c) 2002-2003, Jouni Malinen <jkmaline@cc.hut.fi>
 *
 * Adaption to a generic IEEE 802.11 stack by James Ketrenos
 * <jketreno@linux.intel.com>
 * Copyright (c) 2004-2005, Intel Corporation
 *
 * Adaption to rtnet
 * Copyright (c) 2006, Daniel Gregorek <dxg@gmx.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef RTWLAN_H
#define RTWLAN_H

#include <linux/if_ether.h> /* ETH_ALEN */
#include <linux/kernel.h> /* ARRAY_SIZE */

#include <rtskb.h>
#include <rtwlan_io.h>

#define IEEE80211_1ADDR_LEN 10
#define IEEE80211_2ADDR_LEN 16
#define IEEE80211_3ADDR_LEN 24
#define IEEE80211_4ADDR_LEN 30
#define IEEE80211_FCS_LEN 4
#define IEEE80211_HLEN (IEEE80211_4ADDR_LEN)
#define IEEE80211_FRAME_LEN (IEEE80211_DATA_LEN + IEEE80211_HLEN)

#define MIN_FRAG_THRESHOLD 256U
#define MAX_FRAG_THRESHOLD 2346U

/* Frame control field constants */
#define IEEE80211_FCTL_VERS 0x0003
#define IEEE80211_FCTL_FTYPE 0x000c
#define IEEE80211_FCTL_STYPE 0x00f0
#define IEEE80211_FCTL_TODS 0x0100
#define IEEE80211_FCTL_FROMDS 0x0200
#define IEEE80211_FCTL_MOREFRAGS 0x0400
#define IEEE80211_FCTL_RETRY 0x0800
#define IEEE80211_FCTL_PM 0x1000
#define IEEE80211_FCTL_MOREDATA 0x2000
#define IEEE80211_FCTL_PROTECTED 0x4000
#define IEEE80211_FCTL_ORDER 0x8000

#define IEEE80211_FTYPE_MGMT 0x0000
#define IEEE80211_FTYPE_CTL 0x0004
#define IEEE80211_FTYPE_DATA 0x0008

/* management */
#define IEEE80211_STYPE_ASSOC_REQ 0x0000
#define IEEE80211_STYPE_ASSOC_RESP 0x0010
#define IEEE80211_STYPE_REASSOC_REQ 0x0020
#define IEEE80211_STYPE_REASSOC_RESP 0x0030
#define IEEE80211_STYPE_PROBE_REQ 0x0040
#define IEEE80211_STYPE_PROBE_RESP 0x0050
#define IEEE80211_STYPE_BEACON 0x0080
#define IEEE80211_STYPE_ATIM 0x0090
#define IEEE80211_STYPE_DISASSOC 0x00A0
#define IEEE80211_STYPE_AUTH 0x00B0
#define IEEE80211_STYPE_DEAUTH 0x00C0
#define IEEE80211_STYPE_ACTION 0x00D0

/* control */
#define IEEE80211_STYPE_PSPOLL 0x00A0
#define IEEE80211_STYPE_RTS 0x00B0
#define IEEE80211_STYPE_CTS 0x00C0
#define IEEE80211_STYPE_ACK 0x00D0
#define IEEE80211_STYPE_CFEND 0x00E0
#define IEEE80211_STYPE_CFENDACK 0x00F0

/* data */
#define IEEE80211_STYPE_DATA 0x0000
#define IEEE80211_STYPE_DATA_CFACK 0x0010
#define IEEE80211_STYPE_DATA_CFPOLL 0x0020
#define IEEE80211_STYPE_DATA_CFACKPOLL 0x0030
#define IEEE80211_STYPE_NULLFUNC 0x0040
#define IEEE80211_STYPE_CFACK 0x0050
#define IEEE80211_STYPE_CFPOLL 0x0060
#define IEEE80211_STYPE_CFACKPOLL 0x0070
#define IEEE80211_STYPE_QOS_DATA 0x0080

#define RTWLAN_SCTL_SEQ 0xFFF0

#define WLAN_FC_GET_VERS(fc) ((fc)&IEEE80211_FCTL_VERS)
#define WLAN_FC_GET_TYPE(fc) ((fc)&IEEE80211_FCTL_FTYPE)
#define WLAN_FC_GET_STYPE(fc) ((fc)&IEEE80211_FCTL_STYPE)

#define IEEE80211_DSSS_RATE_1MB 0x02
#define IEEE80211_DSSS_RATE_2MB 0x04
#define IEEE80211_DSSS_RATE_5MB 0x0B
#define IEEE80211_DSSS_RATE_11MB 0x16
#define IEEE80211_OFDM_RATE_6MB 0x0C
#define IEEE80211_OFDM_RATE_9MB 0x12
#define IEEE80211_OFDM_RATE_12MB 0x18
#define IEEE80211_OFDM_RATE_18MB 0x24
#define IEEE80211_OFDM_RATE_24MB 0x30
#define IEEE80211_OFDM_RATE_36MB 0x48
#define IEEE80211_OFDM_RATE_48MB 0x60
#define IEEE80211_OFDM_RATE_54MB 0x6C
#define IEEE80211_BASIC_RATE_MASK 0x80

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(x)                                                             \
	((u8 *)(x))[0], ((u8 *)(x))[1], ((u8 *)(x))[2], ((u8 *)(x))[3],        \
		((u8 *)(x))[4], ((u8 *)(x))[5]

#ifdef CONFIG_RTWLAN_DEBUG
#define RTWLAN_DEBUG_PRINTK(__message...)                                      \
	do {                                                                   \
		rtdm_printk(__message);                                        \
	} while (0)
#define RTWLAN_DEBUG(__message, __args...)                                     \
	RTWLAN_DEBUG_PRINTK(KERN_DEBUG "rtwlan->%s: Debug - " __message,       \
			    __FUNCTION__, ##__args);
#else
#define RTWLAN_DEBUG(__message...)                                             \
	do {                                                                   \
	} while (0)
#endif

struct rtwlan_stats {
	unsigned long rx_packets; /* total packets received	*/
	unsigned long tx_packets; /* total packets transmitted	*/
	unsigned long tx_retry; /* total packets transmitted with retry */
};

struct rtwlan_device {
	struct rtwlan_stats stats;

	struct rtskb_pool skb_pool;

	int mode;

	int (*hard_start_xmit)(struct rtskb *rtskb,
			       struct rtnet_device *rtnet_dev);

	/* This must be the last item */
	u8 priv[0];
};

/* Minimal header; can be used for passing 802.11 frames with sufficient
 * information to determine what type of underlying data type is actually
 * stored in the data. */
struct ieee80211_hdr {
	u16 frame_ctl;
	u16 duration_id;
	u8 payload[0];
} __attribute__((packed));

struct ieee80211_hdr_3addr {
	u16 frame_ctl;
	u16 duration_id;
	u8 addr1[ETH_ALEN];
	u8 addr2[ETH_ALEN];
	u8 addr3[ETH_ALEN];
	u16 seq_ctl;
	u8 payload[0];
} __attribute__((packed));

static inline int ieee80211_get_hdrlen(u16 fc)
{
	int hdrlen = IEEE80211_3ADDR_LEN;
	u16 stype = WLAN_FC_GET_STYPE(fc);

	switch (WLAN_FC_GET_TYPE(fc)) {
	case IEEE80211_FTYPE_DATA:
		if ((fc & IEEE80211_FCTL_FROMDS) && (fc & IEEE80211_FCTL_TODS))
			hdrlen = IEEE80211_4ADDR_LEN;
		if (stype & IEEE80211_STYPE_QOS_DATA)
			hdrlen += 2;
		break;

	case IEEE80211_FTYPE_CTL:
		switch (WLAN_FC_GET_STYPE(fc)) {
		case IEEE80211_STYPE_CTS:
		case IEEE80211_STYPE_ACK:
			hdrlen = IEEE80211_1ADDR_LEN;
			break;

		default:
			hdrlen = IEEE80211_2ADDR_LEN;
			break;
		}
		break;
	}

	return hdrlen;
}

static inline int ieee80211_is_ofdm_rate(u8 rate)
{
	switch (rate & ~IEEE80211_BASIC_RATE_MASK) {
	case IEEE80211_OFDM_RATE_6MB:
	case IEEE80211_OFDM_RATE_9MB:
	case IEEE80211_OFDM_RATE_12MB:
	case IEEE80211_OFDM_RATE_18MB:
	case IEEE80211_OFDM_RATE_24MB:
	case IEEE80211_OFDM_RATE_36MB:
	case IEEE80211_OFDM_RATE_48MB:
	case IEEE80211_OFDM_RATE_54MB:
		return 1;
	}
	return 0;
}

static inline int ieee80211_is_dsss_rate(u8 rate)
{
	switch (rate & ~IEEE80211_BASIC_RATE_MASK) {
	case IEEE80211_DSSS_RATE_1MB:
	case IEEE80211_DSSS_RATE_2MB:
	case IEEE80211_DSSS_RATE_5MB:
	case IEEE80211_DSSS_RATE_11MB:
		return 1;
	}
	return 0;
}

static inline void *rtwlan_priv(struct rtwlan_device *rtwlan_dev)
{
	return (void *)rtwlan_dev + sizeof(struct rtwlan_device);
}

struct rtnet_device *rtwlan_alloc_dev(unsigned sizeof_priv,
				      unsigned dev_pool_size);
int rtwlan_rx(struct rtskb *rtskb, struct rtnet_device *rtnet_dev);
int rtwlan_tx(struct rtskb *rtskb, struct rtnet_device *rtnet_dev);

#ifdef CONFIG_XENO_DRIVERS_NET_RTWLAN
int __init rtwlan_init(void);
void rtwlan_exit(void);
#else /* !CONFIG_XENO_DRIVERS_NET_RTWLAN */
#define rtwlan_init() 0
#define rtwlan_exit()
#endif /* CONFIG_XENO_DRIVERS_NET_RTWLAN */

#endif
