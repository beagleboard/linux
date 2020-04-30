/***
 *
 *  rtmac/rtmac_proto.c
 *
 *  rtmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
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


#include <rtdm/driver.h>
#include <stack_mgr.h>
#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_proto.h>
#include <rtmac/rtmac_vnic.h>



int rtmac_proto_rx(struct rtskb *skb, struct rtpacket_type *pt)
{
    struct rtmac_disc *disc = skb->rtdev->mac_disc;
    struct rtmac_hdr  *hdr;


    if (disc == NULL) {
        goto error;
    }

    hdr = (struct rtmac_hdr *)skb->data;
    rtskb_pull(skb, sizeof(struct rtmac_hdr));

    if (hdr->ver != RTMAC_VERSION) {
        rtdm_printk("RTmac: received unsupported RTmac protocol version on "
                    "device %s.  Got 0x%x but expected 0x%x\n",
                    skb->rtdev->name, hdr->ver, RTMAC_VERSION);
        goto error;
    }

    if (hdr->flags & RTMAC_FLAG_TUNNEL)
        rtmac_vnic_rx(skb, hdr->type);
    else if (disc->disc_type == hdr->type)
        disc->packet_rx(skb);
    return 0;

  error:
    kfree_rtskb(skb);
    return 0;
}



struct rtpacket_type rtmac_packet_type = {
    .type =     __constant_htons(ETH_RTMAC),
    .handler =  rtmac_proto_rx
};



void rtmac_proto_release(void)
{
    rtdev_remove_pack(&rtmac_packet_type);
}
