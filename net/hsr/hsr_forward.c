// SPDX-License-Identifier: GPL-2.0
/* Copyright 2011-2014 Autronica Fire and Security AS
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 *
 * Frame router for HSR and PRP.
 */

#include "hsr_forward.h"
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include "hsr_main.h"
#include "hsr_framereg.h"

struct hsr_node;

static inline int is_hsr_l2ptp(struct sk_buff *skb)
{
	struct hsr_ethhdr *hsr_ethhdr;

	hsr_ethhdr = (struct hsr_ethhdr *)skb_mac_header(skb);

	return (hsr_ethhdr->ethhdr.h_proto == htons(ETH_P_HSR) &&
		hsr_ethhdr->hsr_tag.encap_proto == htons(ETH_P_1588));
}

/* The uses I can see for these HSR supervision frames are:
 * 1) Use the frames that are sent after node initialization ("HSR_TLV.Type =
 *    22") to reset any sequence_nr counters belonging to that node. Useful if
 *    the other node's counter has been reset for some reason.
 *    --
 *    Or not - resetting the counter and bridging the frame would create a
 *    loop, unfortunately.
 *
 * 2) Use the LifeCheck frames to detect ring breaks. I.e. if no LifeCheck
 *    frame is received from a particular node, we know something is wrong.
 *    We just register these (as with normal frames) and throw them away.
 *
 * 3) Allow different MAC addresses for the two slave interfaces, using the
 *    MacAddressA field.
 */
static bool is_supervision_frame(struct hsr_priv *hsr, struct sk_buff *skb)
{
	struct hsrv1_ethhdr_vlan_sp *hsr_v1_vlan_hdr;
	struct hsr_sup_tag *hsr_sup_tag;
	struct hsrv1_ethhdr_sp *hsr_v1_hdr;
	struct ethhdr *eth_hdr;
	bool vlan = false;
	__be16 proto;

	WARN_ON_ONCE(!skb_mac_header_was_set(skb));
	eth_hdr = (struct ethhdr *)skb_mac_header(skb);

	/* Correct addr? */
	if (!ether_addr_equal(eth_hdr->h_dest,
			      hsr->sup_multicast_addr))
		return false;

	if (skb_vlan_tagged(skb)) {
		proto = vlan_get_protocol(skb);
		vlan = true;
	} else {
		proto = eth_hdr->h_proto;
	}

	/* Correct ether type?. */
	if (!(eth_hdr->h_proto == htons(ETH_P_PRP) ||
	      eth_hdr->h_proto == htons(ETH_P_HSR)))
		return false;

	/* Get the supervision header from correct location. */
	if (proto == htons(ETH_P_HSR)) { /* Okay HSRv1. */
		if (!vlan) {
			hsr_v1_hdr = (struct hsrv1_ethhdr_sp *)eth_hdr;
			if (hsr_v1_hdr->hsr.encap_proto != htons(ETH_P_PRP))
				return false;
			hsr_sup_tag = &hsr_v1_hdr->hsr_sup;
		} else {
			hsr_v1_vlan_hdr =
				(struct hsrv1_ethhdr_vlan_sp *)eth_hdr;
			if (hsr_v1_vlan_hdr->hsr.encap_proto !=
						htons(ETH_P_PRP))
				return false;
			hsr_sup_tag = &hsr_v1_vlan_hdr->hsr_sup;
		}
	} else {
		if (!vlan)
			hsr_sup_tag =
			&((struct hsrv0_ethhdr_sp *)eth_hdr)->hsr_sup;
		else
			hsr_sup_tag =
			&((struct hsrv0_ethhdr_vlan_sp *)eth_hdr)->hsr_sup;
	}

	if (hsr_sup_tag->HSR_TLV_type != HSR_TLV_ANNOUNCE &&
	    hsr_sup_tag->HSR_TLV_type != HSR_TLV_LIFE_CHECK &&
	    hsr_sup_tag->HSR_TLV_type != PRP_TLV_LIFE_CHECK_DD &&
	    hsr_sup_tag->HSR_TLV_type != PRP_TLV_LIFE_CHECK_DA)
		return false;
	if (hsr_sup_tag->HSR_TLV_length != 12 &&
	    hsr_sup_tag->HSR_TLV_length != sizeof(struct hsr_sup_payload))
		return false;

	return true;
}

static struct sk_buff *create_stripped_skb_hsr(struct sk_buff *skb_in,
					       struct hsr_frame_info *frame)
{
	struct sk_buff *skb;
	int copylen;
	unsigned char *dst, *src;

	skb_pull(skb_in, HSR_HLEN);
	skb = __pskb_copy(skb_in, skb_headroom(skb_in) - HSR_HLEN, GFP_ATOMIC);
	skb_push(skb_in, HSR_HLEN);
	if (!skb)
		return NULL;

	skb_reset_mac_header(skb);

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		skb->csum_start -= HSR_HLEN;

	copylen = 2 * ETH_ALEN;
	if (frame->is_vlan)
		copylen += VLAN_HLEN;
	src = skb_mac_header(skb_in);
	dst = skb_mac_header(skb);
	memcpy(dst, src, copylen);

	skb->protocol = eth_hdr(skb)->h_proto;
	return skb;
}

struct sk_buff *hsr_get_untagged_frame(struct hsr_frame_info *frame,
				       struct hsr_port *port)
{
	if (!frame->skb_std) {
		if (frame->skb_hsr)
			frame->skb_std =
				create_stripped_skb_hsr(frame->skb_hsr, frame);
		else
			netdev_warn_once(port->dev,
					 "Unexpected frame received in hsr_get_untagged_frame()\n");

		if (!frame->skb_std)
			return NULL;
	}

	return skb_clone(frame->skb_std, GFP_ATOMIC);
}

struct sk_buff *prp_get_untagged_frame(struct hsr_frame_info *frame,
				       struct hsr_port *port)
{
	struct hsr_priv *hsr = port->hsr;

	if (!frame->skb_std) {
		if (frame->skb_prp) {
			if (!hsr->rx_offloaded) {
				/* trim the skb by len - HSR_HLEN to exclude RCT */
				skb_trim(frame->skb_prp,
					 frame->skb_prp->len - HSR_HLEN);
			}
			frame->skb_std =
				__pskb_copy(frame->skb_prp,
					    skb_headroom(frame->skb_prp),
					    GFP_ATOMIC);
		} else {
			/* Unexpected */
			WARN_ONCE(1, "%s:%d: Unexpected frame received (port_src %s)\n",
				  __FILE__, __LINE__, port->dev->name);
			return NULL;
		}
	}

	return skb_clone(frame->skb_std, GFP_ATOMIC);
}

/* only prp skb should be passed in */
static void prp_update_lre_error_stats(struct sk_buff *skb,
				       struct hsr_port *port)
{
	int lan_id;
	struct prp_rct *trailer = skb_get_PRP_rct(skb);

	if (!trailer) {
		INC_CNT_RX_ERROR_AB(port->type, port->hsr);
		return;
	}

	lan_id = get_prp_lan_id(trailer);

	if (port->type == HSR_PT_SLAVE_A) {
		if (lan_id & 1)
			INC_CNT_RX_WRONG_LAN_AB(port->type, port->hsr);
	} else {
		if (!(lan_id & 1))
			INC_CNT_RX_WRONG_LAN_AB(port->type, port->hsr);
	}
}

static void prp_set_lan_id(struct prp_rct *trailer, struct hsr_port *port)
{
	int lane_id;

	if (port->type == HSR_PT_SLAVE_A)
		lane_id = 0;
	else
		lane_id = 1;

	/* Add net_id in the upper 3 bits of lane_id */
	lane_id |= port->hsr->net_id;
	set_prp_lan_id(trailer, lane_id);
}

/* Tailroom for PRP rct should have been created before calling this */
static struct sk_buff *prp_fill_rct(struct sk_buff *skb,
				    struct hsr_frame_info *frame,
				    struct hsr_port *port)
{
	struct prp_rct *trailer;
	int min_size = ETH_ZLEN;
	int lsdu_size;

	if (!skb)
		return skb;

	if (frame->is_vlan)
		min_size = VLAN_ETH_ZLEN;

	if (skb_put_padto(skb, min_size))
		return NULL;

	trailer = (struct prp_rct *)skb_put(skb, HSR_HLEN);
	lsdu_size = skb->len - 14;
	if (frame->is_vlan)
		lsdu_size -= 4;
	prp_set_lan_id(trailer, port);
	set_prp_LSDU_size(trailer, lsdu_size);
	trailer->sequence_nr = htons(frame->sequence_nr);
	trailer->PRP_suffix = htons(ETH_P_PRP);

	return skb;
}

static void hsr_set_path_id(struct hsr_ethhdr *hsr_ethhdr,
			    struct hsr_port *port)
{
	int path_id;

	if (port->type == HSR_PT_SLAVE_A)
		path_id = 0;
	else
		path_id = 1;

	set_hsr_tag_path(&hsr_ethhdr->hsr_tag, path_id);
}

static struct sk_buff *hsr_fill_tag(struct sk_buff *skb,
				    struct hsr_frame_info *frame,
				    struct hsr_port *port, u8 proto_version)
{
	struct hsr_ethhdr *hsr_ethhdr;
	unsigned char *pc;
	int lsdu_size;

	/* pad to minimum packet size which is 60 + 6 (HSR tag) */
	if (skb_put_padto(skb, ETH_ZLEN + HSR_HLEN))
		return NULL;

	lsdu_size = skb->len - 14;
	if (frame->is_vlan)
		lsdu_size -= 4;

	pc = skb_mac_header(skb);
	if (frame->is_vlan)
		/* This 4-byte shift (size of a vlan tag) does not
		 * mean that the ethhdr starts there. But rather it
		 * provides the proper environment for accessing
		 * the fields, such as hsr_tag etc., just like
		 * when the vlan tag is not there. This is because
		 * the hsr tag is after the vlan tag.
		 */
		hsr_ethhdr = (struct hsr_ethhdr *)(pc + VLAN_HLEN);
	else
		hsr_ethhdr = (struct hsr_ethhdr *)pc;

	if (REDINFO_T(skb) == DIRECTED_TX)
		set_hsr_tag_path(&hsr_ethhdr->hsr_tag, REDINFO_PATHID(skb));
	else
		hsr_set_path_id(hsr_ethhdr, port);

	set_hsr_tag_LSDU_size(&hsr_ethhdr->hsr_tag, lsdu_size);
	hsr_ethhdr->hsr_tag.sequence_nr = htons(frame->sequence_nr);
	hsr_ethhdr->hsr_tag.encap_proto = hsr_ethhdr->ethhdr.h_proto;
	hsr_ethhdr->ethhdr.h_proto = htons(proto_version ?
			ETH_P_HSR : ETH_P_PRP);

	return skb;
}

/* If the original frame was an HSR tagged frame, just clone it to be sent
 * unchanged. Otherwise, create a private frame especially tagged for 'port'.
 */
struct sk_buff *hsr_create_tagged_frame(struct hsr_frame_info *frame,
					struct hsr_port *port)
{
	struct skb_redundant_info *sred;
	struct hsr_ethhdr *hsr_ethhdr;
	unsigned char *dst, *src;
	struct sk_buff *skb;
	u16 s;
	int movelen;

	if (frame->skb_hsr) {
		u8 *pc;
		struct hsr_ethhdr *hsr_ethhdr =
			(struct hsr_ethhdr *)skb_mac_header(frame->skb_hsr);

		/* This case is for SV frame created by this device */
		pc = (u8 *)hsr_ethhdr;
		if (frame->is_vlan)
			/* This 4-byte shift (size of a vlan tag) does not
			 * mean that the ethhdr starts there. But rather it
			 * provides the proper environment for accessing
			 * the fields, such as hsr_tag etc., just like
			 * when the vlan tag is not there. This is because
			 * the hsr tag is after the vlan tag.
			 */
			hsr_ethhdr = (struct hsr_ethhdr *)(pc + VLAN_HLEN);
		else
			hsr_ethhdr = (struct hsr_ethhdr *)pc;
		/* set the lane id properly */
		hsr_set_path_id(hsr_ethhdr, port);
		return skb_clone(frame->skb_hsr, GFP_ATOMIC);
	} else if (port->dev->features & NETIF_F_HW_HSR_TAG_INS) {
		return skb_clone(frame->skb_std, GFP_ATOMIC);
	}

	/* Create the new skb with enough headroom to fit the HSR tag */
	skb = __pskb_copy(frame->skb_std,
			  skb_headroom(frame->skb_std) + HSR_HLEN, GFP_ATOMIC);
	if (!skb)
		return NULL;
	skb_reset_mac_header(skb);

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		skb->csum_start += HSR_HLEN;

	movelen = ETH_HLEN;
	if (frame->is_vlan)
		movelen += VLAN_HLEN;

	src = skb_mac_header(skb);
	dst = skb_push(skb, HSR_HLEN);
	memmove(dst, src, movelen);
	skb_reset_mac_header(skb);

	/* skb_put_padto free skb on error and hsr_fill_tag returns NULL in
	 * that case
	 */
	skb = hsr_fill_tag(skb, frame, port, port->hsr->prot_version);
	if (!skb)
		return NULL;

	if (REDINFO_T(skb) == DIRECTED_TX)
		return skb;

	skb_shinfo(skb)->tx_flags |= skb_shinfo(frame->skb_std)->tx_flags & SKBTX_ANY_TSTAMP;
	skb->sk = frame->skb_std->sk;

	/* TODO: should check socket option instead? */
	if (is_hsr_l2ptp(skb)) {
		sred = skb_redinfo(skb);
		/* assumes no vlan */
		hsr_ethhdr = (struct hsr_ethhdr *)skb_mac_header(skb);
		sred->io_port = (PTP_EVT_OUT | BIT(port->type - 1));
		sred->ethertype = ntohs(hsr_ethhdr->ethhdr.h_proto);
		s = ntohs(hsr_ethhdr->hsr_tag.path_and_LSDU_size);
		sred->lsdu_size = s & 0xfff;
		sred->pathid = (s >> 12) & 0xf;
		sred->seqnr = hsr_get_skb_sequence_nr(skb);
	}

	return skb;
}

struct sk_buff *prp_create_tagged_frame(struct hsr_frame_info *frame,
					struct hsr_port *port)
{
	struct sk_buff *skb;

	if (frame->skb_prp) {
		struct prp_rct *trailer = skb_get_PRP_rct(frame->skb_prp);

		if (trailer) {
			prp_set_lan_id(trailer, port);
		} else {
			WARN_ONCE(!trailer, "errored PRP skb");
			return NULL;
		}
		return skb_clone(frame->skb_prp, GFP_ATOMIC);
	} else if (port->dev->features & NETIF_F_HW_HSR_TAG_INS) {
		return skb_clone(frame->skb_std, GFP_ATOMIC);
	}

	skb = skb_copy_expand(frame->skb_std, skb_headroom(frame->skb_std),
			      skb_tailroom(frame->skb_std) + HSR_HLEN,
			      GFP_ATOMIC);
	prp_fill_rct(skb, frame, port);

	return skb;
}

static void hsr_deliver_master(struct sk_buff *skb, struct hsr_node *node_src,
			       struct hsr_port *port)
{
	struct hsr_priv *hsr = port->hsr;
	struct net_device *dev = port->dev;
	bool was_multicast_frame;
	int res, recv_len;

	was_multicast_frame = (skb->pkt_type == PACKET_MULTICAST);
	/* For LRE offloaded case, assume same MAC address is on both
	 * interfaces of the remote node and hence no need to substitute
	 * the source MAC address.
	 */
	if (!port->hsr->rx_offloaded)
		hsr_addr_subst_source(node_src, skb);
	skb_pull(skb, ETH_HLEN);
	recv_len = skb->len;
	res = netif_rx(skb);
	if (res == NET_RX_DROP) {
		dev->stats.rx_dropped++;
	} else {
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += recv_len;
		if (was_multicast_frame)
			dev->stats.multicast++;
		INC_CNT_TX_C(hsr);
	}
}

static int hsr_xmit(struct sk_buff *skb, struct hsr_port *port,
		    struct hsr_frame_info *frame)
{
	if (!port->hsr->rx_offloaded &&
	    frame->port_rcv->type == HSR_PT_MASTER) {
		hsr_addr_subst_dest(frame->node_src, skb, port);

		/* Address substitution (IEC62439-3 pp 26, 50): replace mac
		 * address of outgoing frame with that of the outgoing slave's.
		 */
		ether_addr_copy(eth_hdr(skb)->h_source, port->dev->dev_addr);
	}
	INC_CNT_TX_AB(port->type, port->hsr);
	return dev_queue_xmit(skb);
}

bool prp_drop_frame(struct hsr_frame_info *frame, struct hsr_port *port)
{
	return ((frame->port_rcv->type == HSR_PT_SLAVE_A &&
		 port->type ==  HSR_PT_SLAVE_B) ||
		(frame->port_rcv->type == HSR_PT_SLAVE_B &&
		 port->type ==  HSR_PT_SLAVE_A));
}

bool hsr_drop_frame(struct hsr_frame_info *frame, struct hsr_port *port)
{
	if (port->dev->features & NETIF_F_HW_HSR_FWD)
		return prp_drop_frame(frame, port);

	return false;
};

static void stripped_skb_get_shared_info(struct sk_buff *skb_stripped,
					 struct hsr_frame_info *frame)
{
	struct hsr_port *port_rcv = frame->port_rcv;
	struct skb_redundant_info *sred;
	struct sk_buff *skb_hsr, *skb;
	struct hsr_ethhdr *hsr_ethhdr;
	u16 s;

	if (port_rcv->hsr->prot_version > HSR_V1)
		return;

	if (!frame->skb_hsr)
		return;

	skb_hsr = frame->skb_hsr;
	skb = skb_stripped;

	if (is_hsr_l2ptp(skb_hsr)) {
		skb_hwtstamps(skb)->hwtstamp = skb_hwtstamps(skb_hsr)->hwtstamp;
		skb_redinfo_hwtstamps(skb)->hwtstamp =
			skb_redinfo_hwtstamps(skb_hsr)->hwtstamp;

		sred = skb_redinfo(skb);
		/* assumes no vlan */
		hsr_ethhdr = (struct hsr_ethhdr *)skb_mac_header(skb_hsr);
		sred->io_port = (PTP_MSG_IN | BIT(port_rcv->type - 1));
		sred->ethertype = ntohs(hsr_ethhdr->ethhdr.h_proto);
		s = ntohs(hsr_ethhdr->hsr_tag.path_and_LSDU_size);
		sred->lsdu_size = s & 0xfff;
		sred->pathid = (s >> 12) & 0xf;
		sred->seqnr = frame->sequence_nr;
	}
}

static unsigned int
hsr_directed_tx_ports(struct hsr_frame_info *frame)
{
	struct sk_buff *skb;

	if (frame->skb_std)
		skb = frame->skb_std;
	else
		return 0;

	if (REDINFO_T(skb) == DIRECTED_TX)
		return REDINFO_PORTS(skb);

	return 0;
}

/* Forward the frame through all devices except:
 * - Back through the receiving device
 * - If it's a HSR frame: through a device where it has passed before
 * - if it's a PRP frame: through another PRP slave device (no bridge)
 * - To the local HSR master only if the frame is directly addressed to it, or
 *   a non-supervision multicast or broadcast frame.
 *
 * HSR slave devices should insert a HSR tag into the frame, or forward the
 * frame unchanged if it's already tagged. Interlink devices should strip HSR
 * tags if they're of the non-HSR type (but only after duplicate discard). The
 * master device always strips HSR tags.
 */
static void hsr_forward_do(struct hsr_frame_info *frame)
{
	struct sk_buff *skb = NULL;
	unsigned int dir_ports = 0;
	struct hsr_port *port;
	bool sent = false;

	hsr_for_each_port(frame->port_rcv->hsr, port) {
		struct hsr_priv *hsr = port->hsr;
		/* Don't send frame back the way it came */
		if (port == frame->port_rcv)
			continue;

		/* Don't deliver locally unless we should */
		if (port->type == HSR_PT_MASTER && !frame->is_local_dest)
			continue;

		/* Deliver frames directly addressed to us to master only */
		if (port->type != HSR_PT_MASTER && frame->is_local_exclusive)
			continue;

		/* If hardware duplicate generation is enabled, only send out
		 * one port.
		 */
		if ((port->dev->features & NETIF_F_HW_HSR_DUP) && sent)
			continue;

		/* Don't send frame over port where it has been sent before.
		 * Also if rx LRE is offloaded, hardware does duplication
		 * detection and discard and send only one copy to the upper
		 * device and thus discard duplicate detection. For PRP, frame
		 * could be from a SAN for which bypass duplicate discard here.
		 */
		if (!port->hsr->rx_offloaded && !frame->is_from_san &&
		    hsr_register_frame_out(port, frame->node_src,
					   frame->sequence_nr))
			continue;

		/* In LRE offloaded case, don't expect supervision frames from
		 * slave ports for host as they get processed at the h/w or
		 * firmware.
		 */
		if (frame->is_supervision &&
		    port->type == HSR_PT_MASTER && !port->hsr->rx_offloaded) {
			if (frame->skb_hsr)
				skb = frame->skb_hsr;
			else if (frame->skb_prp)
				skb = frame->skb_prp;
			if (skb)
				hsr_handle_sup_frame(frame);
			continue;
		}

		/* Check if frame is to be dropped. Eg. for PRP no forward
		 * between ports.
		 */
		if (hsr->proto_ops->drop_frame &&
		    hsr->proto_ops->drop_frame(frame, port))
			continue;

		dir_ports = hsr_directed_tx_ports(frame);
		if (dir_ports && !(dir_ports & BIT(port->type - 1)))
			continue;

		if (port->type != HSR_PT_MASTER) {
			skb = hsr->proto_ops->create_tagged_frame(frame, port);
		} else {
			skb = hsr->proto_ops->get_untagged_frame(frame, port);
			stripped_skb_get_shared_info(skb, frame);
		}

		if (!skb) {
			frame->port_rcv->dev->stats.rx_dropped++;
			if (frame->port_rcv->type == HSR_PT_SLAVE_A ||
			    frame->port_rcv->type ==  HSR_PT_SLAVE_B)
				INC_CNT_RX_ERROR_AB(frame->port_rcv->type,
						 port->hsr);
			continue;
		}

		skb->dev = port->dev;
		if (port->type == HSR_PT_MASTER) {
			hsr_deliver_master(skb, frame->node_src, port);
		} else {
			if (!hsr_xmit(skb, port, frame))
				sent = true;
		}
	}
}

static void check_local_dest(struct hsr_priv *hsr, struct sk_buff *skb,
			     struct hsr_frame_info *frame)
{
	if (hsr_addr_is_self(hsr, eth_hdr(skb)->h_dest)) {
		frame->is_local_exclusive = true;
		skb->pkt_type = PACKET_HOST;
	} else {
		frame->is_local_exclusive = false;
	}

	if (skb->pkt_type == PACKET_HOST ||
	    skb->pkt_type == PACKET_MULTICAST ||
	    skb->pkt_type == PACKET_BROADCAST) {
		frame->is_local_dest = true;
	} else {
		frame->is_local_dest = false;
	}
}

static void handle_std_frame(struct sk_buff *skb,
			     struct hsr_frame_info *frame)
{
	struct hsr_port *port = frame->port_rcv;
	struct hsr_priv *hsr = port->hsr;
	unsigned long irqflags;

	frame->skb_hsr = NULL;
	frame->skb_prp = NULL;
	frame->skb_std = skb;

	if (port->type != HSR_PT_MASTER) {
		frame->is_from_san = true;
	} else {
		if ((REDINFO_T(skb) == DIRECTED_TX) &&
		    (REDINFO_LSDU_SIZE(skb))) {
			frame->sequence_nr = REDINFO_SEQNR(skb);
		} else {
			/* Sequence nr for the master node */
			spin_lock_irqsave(&hsr->seqnr_lock, irqflags);
			frame->sequence_nr = hsr->sequence_nr;
			hsr->sequence_nr++;
			spin_unlock_irqrestore(&hsr->seqnr_lock, irqflags);
		}
	}
}

int hsr_fill_frame_info(__be16 proto, struct sk_buff *skb,
			struct hsr_frame_info *frame)
{
	if (proto == htons(ETH_P_PRP) ||
	    proto == htons(ETH_P_HSR)) {
		/* Check if skb contains hsr_ethhdr */
		if (skb->mac_len < sizeof(struct hsr_ethhdr))
			return -EINVAL;

		/* HSR tagged frame :- Data or Supervision */
		frame->skb_std = NULL;
		frame->skb_prp = NULL;
		frame->skb_hsr = skb;
		frame->sequence_nr = hsr_get_skb_sequence_nr(skb);
		return 0;
	}

	/* Standard frame or PRP from master port */
	handle_std_frame(skb, frame);

	return 0;
}

int prp_fill_frame_info(__be16 proto, struct sk_buff *skb,
			struct hsr_frame_info *frame)
{
	/* Supervision frame */
	struct prp_rct *rct = skb_get_PRP_rct(skb);

	if (rct &&
	    prp_check_lsdu_size(skb, rct, frame->is_supervision)) {
		frame->skb_hsr = NULL;
		frame->skb_std = NULL;
		frame->skb_prp = skb;
		frame->sequence_nr = prp_get_skb_sequence_nr(rct);
		return 0;
	}
	handle_std_frame(skb, frame);

	return 0;
}

static int fill_frame_info(struct hsr_frame_info *frame,
			   struct sk_buff *skb, struct hsr_port *port)
{
	struct hsr_priv *hsr = port->hsr;
	struct hsr_vlan_ethhdr *vlan_hdr;
	struct ethhdr *ethhdr;
	__be16 proto;
	int ret;

	/* Check if skb contains ethhdr */
	if (!hsr->rx_offloaded && skb->mac_len < sizeof(struct ethhdr))
		return -EINVAL;

	memset(frame, 0, sizeof(*frame));
	frame->is_supervision = is_supervision_frame(hsr, skb);

	/* When offloaded, don't expect Supervision frame which
	 * is terminated at h/w or f/w that offload the LRE
	 */
	if (frame->is_supervision && hsr->rx_offloaded &&
	    port->type != HSR_PT_MASTER)
		return -1;

	if (frame->is_supervision) {
		if (port->type == HSR_PT_SLAVE_A)
			INC_CNT_RX_SUP_A(hsr);
		else if (port->type == HSR_PT_SLAVE_B)
			INC_CNT_RX_SUP_B(hsr);
	}

	/* For Offloaded case, there is no need for node list since
	 * firmware/hardware implements LRE function.
	 */
	if (!hsr->rx_offloaded) {
		frame->node_src = hsr_get_node(port, &hsr->node_db, skb,
					       frame->is_supervision,
					       port->type);
		/* Unknown node and !is_supervision, or no mem */
		if (!frame->node_src)
			return -1;
	}

	ethhdr = (struct ethhdr *)skb_mac_header(skb);
	frame->is_vlan = false;
	proto = ethhdr->h_proto;

	if (proto == htons(ETH_P_8021Q))
		frame->is_vlan = true;

	if (frame->is_vlan) {
		vlan_hdr = (struct hsr_vlan_ethhdr *)ethhdr;
		proto = vlan_hdr->vlanhdr.h_vlan_encapsulated_proto;
	}

	frame->is_from_san = false;
	frame->port_rcv = port;
	ret = hsr->proto_ops->fill_frame_info(proto, skb, frame);
	if (ret)
		return ret;

	check_local_dest(port->hsr, skb, frame);

	return 0;
}

/* Must be called holding rcu read lock (because of the port parameter) */
void hsr_forward_skb(struct sk_buff *skb, struct hsr_port *port)
{
	struct hsr_frame_info frame;

	if (fill_frame_info(&frame, skb, port) < 0)
		goto out_drop;

	/* Check for LAN_ID only for PRP */
	if (frame.skb_prp) {
		if (port->type == HSR_PT_SLAVE_A  ||
		    port->type == HSR_PT_SLAVE_B)
			prp_update_lre_error_stats(frame.skb_prp, port);
	}

	/* No need to register frame when rx offload is supported */
	if (!port->hsr->rx_offloaded)
		hsr_register_frame_in(frame.node_src, port,
				      frame.sequence_nr);
	hsr_forward_do(&frame);
	/* Gets called for ingress frames as well as egress from master port.
	 * So check and increment stats for master port only here.
	 */
	if (port->type == HSR_PT_MASTER) {
		port->dev->stats.tx_packets++;
		port->dev->stats.tx_bytes += skb->len;
	}

	kfree_skb(frame.skb_hsr);
	kfree_skb(frame.skb_prp);
	kfree_skb(frame.skb_std);
	return;

out_drop:
	INC_CNT_RX_ERROR_AB(port->type, port->hsr);
	port->dev->stats.tx_dropped++;
	kfree_skb(skb);
}
