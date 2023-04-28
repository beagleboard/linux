/*
 * Header file for the Packet dump helper functions
 *
 * Copyright (C) 2020, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

#ifndef __DHD_LINUX_PKTDUMP_H_
#define __DHD_LINUX_PKTDUMP_H_

#include <typedefs.h>
#include <dhd.h>

typedef enum {
	EAPOL_OTHER = 0,
	EAPOL_4WAY_M1,
	EAPOL_4WAY_M2,
	EAPOL_4WAY_M3,
	EAPOL_4WAY_M4,
	EAPOL_GROUPKEY_M1,
	EAPOL_GROUPKEY_M2
} msg_eapol_t;

typedef enum pkt_cnt_rsn {
	PKT_CNT_RSN_INVALID	= 0,
	PKT_CNT_RSN_ROAM	= 1,
	PKT_CNT_RSN_GRPKEY_UP	= 2,
	PKT_CNT_RSN_CONNECT	= 3,
	PKT_CNT_RSN_MAX		= 4
} pkt_cnt_rsn_t;

enum pkt_type {
	PKT_TYPE_DATA = 0,
	PKT_TYPE_DHCP = 1,
	PKT_TYPE_ICMP = 2,
	PKT_TYPE_DNS = 3,
	PKT_TYPE_ARP = 4,
	PKT_TYPE_EAP = 5
};

extern msg_eapol_t dhd_is_4way_msg(uint8 *pktdata);
extern void dhd_dump_pkt(dhd_pub_t *dhd, int ifidx, uint8 *pktdata,
	uint32 pktlen, bool tx, uint32 *pkthash, uint16 *pktfate);
#ifdef BCMPCIE
extern bool dhd_match_pkt_type(dhd_pub_t *dhd, uint8 *pktdata, uint32 pktlen);
#endif /* BCMPCIE */
#ifdef DHD_PKTDUMP_ROAM
extern void dhd_dump_mod_pkt_timer(dhd_pub_t *dhdp, uint16 rsn);
extern void dhd_dump_pkt_init(dhd_pub_t *dhdp);
extern void dhd_dump_pkt_deinit(dhd_pub_t *dhdp);
extern void dhd_dump_pkt_clear(dhd_pub_t *dhdp);
#else
static INLINE void dhd_dump_mod_pkt_timer(dhd_pub_t *dhdp, uint16 rsn) { }
static INLINE void dhd_dump_pkt_init(dhd_pub_t *dhdp) { }
static INLINE void dhd_dump_pkt_deinit(dhd_pub_t *dhdp) { }
static INLINE void dhd_dump_pkt_clear(dhd_pub_t *dhdp) { }
#endif /* DHD_PKTDUMP_ROAM */

/* Rx packet dump */
#ifdef DHD_TRX_DUMP
extern void dhd_trx_pkt_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, uint32 pktlen, bool tx);
#else
static INLINE void dhd_trx_pkt_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, uint32 pktlen, bool tx) { }
#endif /* DHD_TRX_DUMP */

/* DHCP packet dump */
#ifdef DHD_DHCP_DUMP
extern void dhd_dhcp_dump(dhd_pub_t *dhdp, int ifidx, uint8 *pktdata, bool tx,
	uint32 *pkthash, uint16 *pktfate);
#else
static INLINE void dhd_dhcp_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, bool tx, uint32 *pkthash, uint16 *pktfate) { }
#endif /* DHD_DHCP_DUMP */

/* DNS packet dump */
#ifdef DHD_DNS_DUMP
extern void dhd_dns_dump(dhd_pub_t *dhdp, int ifidx, uint8 *pktdata, bool tx,
	uint32 *pkthash, uint16 *pktfate);
#else
static INLINE void dhd_dns_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, bool tx, uint32 *pkthash, uint16 *pktfate) { }
#endif /* DHD_DNS_DUMP */

/* ICMP packet dump */
#ifdef DHD_ICMP_DUMP
extern void dhd_icmp_dump(dhd_pub_t *dhdp, int ifidx, uint8 *pktdata, bool tx,
	uint32 *pkthash, uint16 *pktfate);
#else
static INLINE void dhd_icmp_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, bool tx, uint32 *pkthash, uint16 *pktfate) { }
#endif /* DHD_ICMP_DUMP */

/* ARP packet dump */
#ifdef DHD_ARP_DUMP
extern void dhd_arp_dump(dhd_pub_t *dhdp, int ifidx, uint8 *pktdata, bool tx,
	uint32 *pkthash, uint16 *pktfate);
#else
static INLINE void dhd_arp_dump(dhd_pub_t *dhdp, int ifidx,
	uint8 *pktdata, bool tx, uint32 *pkthash, uint16 *pktfate) { }
#endif /* DHD_ARP_DUMP */

/* 802.1X packet dump */
#ifdef DHD_8021X_DUMP
extern void dhd_dump_eapol_message(dhd_pub_t *dhd, int ifidx,
        uint8 *pktdata, uint32 pktlen, bool tx, uint32 *pkthash, uint16 *pktfate);
#else
static INLINE void dhd_dump_eapol_message(dhd_pub_t *dhd, int ifidx,
        uint8 *pktdata, uint32 pktlen, bool tx, uint32 *pkthash, uint16 *pktfate) { }
#endif /* DHD_8021X_DUMP */
extern bool dhd_check_ip_prot(uint8 *pktdata, uint16 ether_type);
extern bool dhd_check_arp(uint8 *pktdata, uint16 ether_type);
extern bool dhd_check_dhcp(uint8 *pktdata);
extern bool dhd_check_icmp(uint8 *pktdata);
extern bool dhd_check_dns(uint8 *pktdata);
#endif /* __DHD_LINUX_PKTDUMP_H_ */
