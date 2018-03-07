/* ip_fragment.c
 *
 * Copyright (C) 2002      Ulrich Marx <marx@kammer.uni-hannover.de>
 *               2003      Mathias Koehrer <mathias_koehrer@yahoo.de>
 *               2003-2005 Jan Kiszka <jan.kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <net/checksum.h>
#include <net/ip.h>

#include <rtdev.h>
#include <rtnet_internal.h>
#include <rtnet_socket.h>

#include <linux/ip.h>
#include <linux/in.h>

#include <ipv4/ip_fragment.h>

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
#include <ipv4/ip_input.h>
#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_PROXY */

/*
 * This defined sets the number of incoming fragmented IP messages that
 * can be handled in parallel.
 */
#define COLLECTOR_COUNT 10

struct ip_collector
{
    int   in_use;
    __u32 saddr;
    __u32 daddr;
    __u16 id;
    __u8  protocol;

    struct rtskb_queue frags;
    struct rtsocket *sock;
    unsigned int buf_size;
};

static struct ip_collector collector[COLLECTOR_COUNT];


static void alloc_collector(struct rtskb *skb, struct rtsocket *sock)
{
    int                 i;
    rtdm_lockctx_t      context;
    struct ip_collector *p_coll;
    struct iphdr        *iph = skb->nh.iph;


    /*
     * Find a free collector
     *
     * Note: We once used to clean up probably outdated chains, but the
     * algorithm was not stable enough and could cause incorrect drops even
     * under medium load. If we run in overload, we will loose data anyhow.
     * What we should do in the future is to account collectors per socket or
     * socket owner and set quotations.
     * Garbage collection is now performed only on socket close.
     */
    for (i = 0; i < COLLECTOR_COUNT; i++) {
        p_coll = &collector[i];
        rtdm_lock_get_irqsave(&p_coll->frags.lock, context);

        if (!p_coll->in_use) {
            p_coll->in_use        = 1;
            p_coll->buf_size      = skb->len;
            p_coll->frags.first   = skb;
            p_coll->frags.last    = skb;
            p_coll->saddr         = iph->saddr;
            p_coll->daddr         = iph->daddr;
            p_coll->id            = iph->id;
            p_coll->protocol      = iph->protocol;
            p_coll->sock          = sock;

            rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);

            return;
        }

        rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
    }

    rtdm_printk("RTnet: IP fragmentation - no collector available\n");
    kfree_rtskb(skb);
}



/*
 * Return a pointer to the collector that holds the message which
 * fits to the iphdr of the passed rtskb.
 * */
static struct rtskb *add_to_collector(struct rtskb *skb, unsigned int offset, int more_frags)
{
	int                 i, err;
    rtdm_lockctx_t      context;
    struct ip_collector *p_coll;
    struct iphdr        *iph = skb->nh.iph;
    struct rtskb        *first_skb;


    /* Search in existing collectors */
    for (i = 0; i < COLLECTOR_COUNT; i++)
    {
        p_coll = &collector[i];
        rtdm_lock_get_irqsave(&p_coll->frags.lock, context);

        if (p_coll->in_use  &&
            (iph->saddr    == p_coll->saddr) &&
            (iph->daddr    == p_coll->daddr) &&
            (iph->id       == p_coll->id) &&
            (iph->protocol == p_coll->protocol))
        {
            first_skb = p_coll->frags.first;

            /* Acquire the rtskb at the expense of the protocol pool */
            if (rtskb_acquire(skb, &p_coll->sock->skb_pool) != 0) {
                /* We have to drop this fragment => clean up the whole chain */
                p_coll->in_use = 0;

                rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);

#ifdef FRAG_DBG
                rtdm_printk("RTnet: Compensation pool empty - IP fragments "
                            "dropped (saddr:%x, daddr:%x)\n",
                            iph->saddr, iph->daddr);
#endif

                kfree_rtskb(first_skb);
                kfree_rtskb(skb);
                return NULL;
            }

            /* Optimized version of __rtskb_queue_tail */
            skb->next = NULL;
            p_coll->frags.last->next = skb;
            p_coll->frags.last = skb;

            /* Extend the chain */
            first_skb->chain_end = skb;

            /* Sanity check: unordered fragments are not allowed! */
            if (offset != p_coll->buf_size) {
                /* We have to drop this fragment => clean up the whole chain */
                p_coll->in_use = 0;
                skb = first_skb;

                rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
                break; /* leave the for loop */
            }

            p_coll->buf_size += skb->len;

            if (!more_frags) {
                p_coll->in_use = 0;

		err = rt_socket_reference(p_coll->sock);

                rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);

		if (err < 0) {
			kfree_rtskb(first_skb);
			return NULL;
		}

                return first_skb;
            } else {
                rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
                return NULL;
            }
        }

        rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
    }

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
    if (rt_ip_fallback_handler) {
            __rtskb_push(skb, iph->ihl*4);
            rt_ip_fallback_handler(skb);
            return NULL;
    }
#endif

#ifdef FRAG_DBG
    rtdm_printk("RTnet: Unordered IP fragment (saddr:%x, daddr:%x)"
                " - dropped\n", iph->saddr, iph->daddr);
#endif

    kfree_rtskb(skb);
    return NULL;
}



/*
 * Cleans up all collectors referring to the specified socket.
 * This is now the only kind of garbage collection we do.
 */
void rt_ip_frag_invalidate_socket(struct rtsocket *sock)
{
    int                 i;
    rtdm_lockctx_t      context;
    struct ip_collector *p_coll;


    for (i = 0; i < COLLECTOR_COUNT; i++)
    {
        p_coll = &collector[i];
        rtdm_lock_get_irqsave(&p_coll->frags.lock, context);

        if ((p_coll->in_use) && (p_coll->sock == sock))
        {
            p_coll->in_use = 0;
            kfree_rtskb(p_coll->frags.first);
        }

        rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
    }
}
EXPORT_SYMBOL_GPL(rt_ip_frag_invalidate_socket);



/*
 * Cleans up all existing collectors
 */
static void cleanup_all_collectors(void)
{
    int                 i;
    rtdm_lockctx_t      context;
    struct ip_collector *p_coll;


    for (i = 0; i < COLLECTOR_COUNT; i++)
    {
        p_coll = &collector[i];
        rtdm_lock_get_irqsave(&p_coll->frags.lock, context);

        if (p_coll->in_use)
        {
            p_coll->in_use = 0;
            kfree_rtskb(p_coll->frags.first);
        }

        rtdm_lock_put_irqrestore(&p_coll->frags.lock, context);
    }
}



/*
 * This function returns an rtskb that contains the complete, accumulated IP message.
 * If not all fragments of the IP message have been received yet, it returns NULL
 * Note: the IP header must have already been pulled from the rtskb!
 * */
struct rtskb *rt_ip_defrag(struct rtskb *skb, struct rtinet_protocol *ipprot)
{
    unsigned int    more_frags;
    unsigned int    offset;
    struct rtsocket *sock;
    struct iphdr    *iph = skb->nh.iph;
    int             ret;


    /* Parse the IP header */
    offset = ntohs(iph->frag_off);
    more_frags = offset & IP_MF;
    offset &= IP_OFFSET;
    offset <<= 3;   /* offset is in 8-byte chunks */

    /* First fragment? */
    if (offset == 0)
    {
        /* Get the destination socket */
        if ((sock = ipprot->dest_socket(skb)) == NULL) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
                if (rt_ip_fallback_handler) {
                    __rtskb_push(skb, iph->ihl*4);
                    rt_ip_fallback_handler(skb);
                    return NULL;
                }
#endif
            /* Drop the rtskb */
            kfree_rtskb(skb);
            return NULL;
        }

	/* Acquire the rtskb, to unlock the device skb pool */
        ret = rtskb_acquire(skb, &sock->skb_pool);

        if (ret != 0) {
            /* Drop the rtskb */
            kfree_rtskb(skb);
        } else {
            /* Allocates a new collector */
            alloc_collector(skb, sock);
        }

        /* Packet is queued or freed, socket can be released */
        rt_socket_dereference(sock);

        return NULL;
    }
    else
    {
        /* Add to an existing collector */
        return add_to_collector(skb, offset, more_frags);
    }
}



int __init rt_ip_fragment_init(void)
{
    int i;


    /* Probably not needed (static variable...) */
    memset(collector, 0, sizeof(collector));

    for (i = 0; i < COLLECTOR_COUNT; i++)
        rtdm_lock_init(&collector[i].frags.lock);

    return 0;
}



void rt_ip_fragment_cleanup(void)
{
    cleanup_all_collectors();
}
