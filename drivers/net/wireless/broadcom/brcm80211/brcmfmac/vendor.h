// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */

#ifndef _vendor_h_
#define _vendor_h_

#define BROADCOM_OUI	0x001018

enum brcmf_vndr_cmds {
	BRCMF_VNDR_CMDS_UNSPEC,
	BRCMF_VNDR_CMDS_DCMD,
	BRCMF_VNDR_CMDS_LAST
};

enum brcmf_vndr_evts {
	BRCMF_VNDR_EVTS_PHY_TEMP,
	BRCMF_VNDR_EVTS_LAST
};

/**
 * enum brcmf_nlattrs - nl80211 message attributes
 *
 * @BRCMF_NLATTR_LEN: message body length
 * @BRCMF_NLATTR_DATA: message body
 */
enum brcmf_nlattrs {
	BRCMF_NLATTR_UNSPEC,

	BRCMF_NLATTR_LEN,
	BRCMF_NLATTR_DATA,
	BRCMF_NLATTR_VERS,
	BRCMF_NLATTR_PHY_TEMP,
	BRCMF_NLATTR_PHY_TEMPDELTA,

	__BRCMF_NLATTR_AFTER_LAST,
	BRCMF_NLATTR_MAX = __BRCMF_NLATTR_AFTER_LAST - 1
};

/* structure of event sent up by firmware: is this the right place for it? */
struct brcmf_phy_temp_evt {
	__le32 version;
	__le32 temp;
	__le32 tempdelta;
} __packed;

/**
 * struct brcmf_vndr_dcmd_hdr - message header for cfg80211 vendor command dcmd
 *				support
 *
 * @cmd: common dongle cmd definition
 * @len: length of expecting return buffer
 * @offset: offset of data buffer
 * @set: get or set request(optional)
 * @magic: magic number for verification
 */
struct brcmf_vndr_dcmd_hdr {
	uint cmd;
	int len;
	uint offset;
	uint set;
	uint magic;
};

extern const struct wiphy_vendor_command brcmf_vendor_cmds[];
extern const struct nl80211_vendor_cmd_info brcmf_vendor_events[];
s32 brcmf_wiphy_phy_temp_evt_handler(struct brcmf_if *ifp,
				     const struct brcmf_event_msg *e,
				     void *data);

#endif /* _vendor_h_ */
