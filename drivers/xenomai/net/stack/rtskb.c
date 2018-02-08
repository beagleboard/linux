/***
 *
 *  stack/rtskb.c - rtskb implementation for rtnet
 *
 *  Copyright (C) 2002      Ulrich Marx <marx@fet.uni-hannover.de>,
 *  Copyright (C) 2003-2006 Jan Kiszka <jan.kiszka@web.de>
 *  Copyright (C) 2006 Jorge Almeida <j-almeida@criticalsoftware.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <net/checksum.h>

#include <rtdev.h>
#include <rtnet_internal.h>
#include <rtskb.h>
#include <rtnet_port.h>

static unsigned int global_rtskbs    = DEFAULT_GLOBAL_RTSKBS;
module_param(global_rtskbs, uint, 0444);
MODULE_PARM_DESC(global_rtskbs, "Number of realtime socket buffers in global pool");


/* Linux slab pool for rtskbs */
static struct kmem_cache *rtskb_slab_pool;

/* pool of rtskbs for global use */
struct rtskb_pool global_pool;
EXPORT_SYMBOL_GPL(global_pool);

/* pool statistics */
unsigned int rtskb_pools=0;
unsigned int rtskb_pools_max=0;
unsigned int rtskb_amount=0;
unsigned int rtskb_amount_max=0;

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
/* RTcap interface */
rtdm_lock_t rtcap_lock;
EXPORT_SYMBOL_GPL(rtcap_lock);

void (*rtcap_handler)(struct rtskb *skb) = NULL;
EXPORT_SYMBOL_GPL(rtcap_handler);
#endif


/***
 *  rtskb_copy_and_csum_bits
 */
unsigned int rtskb_copy_and_csum_bits(const struct rtskb *skb, int offset,
				      u8 *to, int len, unsigned int csum)
{
    int copy;

    /* Copy header. */
    if ((copy = skb->len-offset) > 0) {
	if (copy > len)
	    copy = len;
	csum = csum_partial_copy_nocheck(skb->data+offset, to, copy, csum);
	if ((len -= copy) == 0)
	    return csum;
	offset += copy;
	to += copy;
    }

    RTNET_ASSERT(len == 0, );
    return csum;
}

EXPORT_SYMBOL_GPL(rtskb_copy_and_csum_bits);


/***
 *  rtskb_copy_and_csum_dev
 */
void rtskb_copy_and_csum_dev(const struct rtskb *skb, u8 *to)
{
    unsigned int csum;
    unsigned int csstart;

    if (skb->ip_summed == CHECKSUM_PARTIAL) {
	csstart = skb->h.raw - skb->data;

	if (csstart > skb->len)
	    BUG();
    } else
	csstart = skb->len;

    memcpy(to, skb->data, csstart);

    csum = 0;
    if (csstart != skb->len)
	csum = rtskb_copy_and_csum_bits(skb, csstart, to+csstart, skb->len-csstart, 0);

    if (skb->ip_summed == CHECKSUM_PARTIAL) {
	unsigned int csstuff = csstart + skb->csum;

	*((unsigned short *)(to + csstuff)) = csum_fold(csum);
    }
}

EXPORT_SYMBOL_GPL(rtskb_copy_and_csum_dev);


#ifdef CONFIG_XENO_DRIVERS_NET_CHECKED
/**
 *  skb_over_panic - private function
 *  @skb: buffer
 *  @sz: size
 *  @here: address
 *
 *  Out of line support code for rtskb_put(). Not user callable.
 */
void rtskb_over_panic(struct rtskb *skb, int sz, void *here)
{
    rtdm_printk("RTnet: rtskb_put :over: %p:%d put:%d dev:%s\n", here,
		skb->len, sz, (skb->rtdev) ? skb->rtdev->name : "<NULL>");
}

EXPORT_SYMBOL_GPL(rtskb_over_panic);


/**
 *  skb_under_panic - private function
 *  @skb: buffer
 *  @sz: size
 *  @here: address
 *
 *  Out of line support code for rtskb_push(). Not user callable.
 */
void rtskb_under_panic(struct rtskb *skb, int sz, void *here)
{
    rtdm_printk("RTnet: rtskb_push :under: %p:%d put:%d dev:%s\n", here,
		skb->len, sz, (skb->rtdev) ? skb->rtdev->name : "<NULL>");
}

EXPORT_SYMBOL_GPL(rtskb_under_panic);
#endif /* CONFIG_XENO_DRIVERS_NET_CHECKED */

static struct rtskb *__rtskb_pool_dequeue(struct rtskb_pool *pool)
{
    struct rtskb_queue *queue = &pool->queue;
    struct rtskb *skb;

    if (!pool->lock_ops->trylock(pool->lock_cookie))
	    return NULL;
    skb = __rtskb_dequeue(queue);
    if (skb == NULL)
	    pool->lock_ops->unlock(pool->lock_cookie);

    return skb;
}

struct rtskb *rtskb_pool_dequeue(struct rtskb_pool *pool)
{
    struct rtskb_queue *queue = &pool->queue;
    rtdm_lockctx_t context;
    struct rtskb *skb;

    rtdm_lock_get_irqsave(&queue->lock, context);
    skb = __rtskb_pool_dequeue(pool);
    rtdm_lock_put_irqrestore(&queue->lock, context);

    return skb;
}
EXPORT_SYMBOL_GPL(rtskb_pool_dequeue);

static void __rtskb_pool_queue_tail(struct rtskb_pool *pool, struct rtskb *skb)
{
    struct rtskb_queue *queue = &pool->queue;

    __rtskb_queue_tail(queue,skb);
    pool->lock_ops->unlock(pool->lock_cookie);
}

void rtskb_pool_queue_tail(struct rtskb_pool *pool, struct rtskb *skb)
{
    struct rtskb_queue *queue = &pool->queue;
    rtdm_lockctx_t context;

    rtdm_lock_get_irqsave(&queue->lock, context);
    __rtskb_pool_queue_tail(pool, skb);
    rtdm_lock_put_irqrestore(&queue->lock, context);
}
EXPORT_SYMBOL_GPL(rtskb_pool_queue_tail);

/***
 *  alloc_rtskb - allocate an rtskb from a pool
 *  @size: required buffer size (to check against maximum boundary)
 *  @pool: pool to take the rtskb from
 */
struct rtskb *alloc_rtskb(unsigned int size, struct rtskb_pool *pool)
{
    struct rtskb *skb;

    RTNET_ASSERT(size <= SKB_DATA_ALIGN(RTSKB_SIZE), return NULL;);

    skb = rtskb_pool_dequeue(pool);
    if (!skb)
	return NULL;

    /* Load the data pointers. */
    skb->data = skb->buf_start;
    skb->tail = skb->buf_start;
    skb->end  = skb->buf_start + size;

    /* Set up other states */
    skb->chain_end = skb;
    skb->len = 0;
    skb->pkt_type = PACKET_HOST;
    skb->xmit_stamp = NULL;

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
    skb->cap_flags = 0;
#endif

    return skb;
}

EXPORT_SYMBOL_GPL(alloc_rtskb);


/***
 *  kfree_rtskb
 *  @skb    rtskb
 */
void kfree_rtskb(struct rtskb *skb)
{
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
    rtdm_lockctx_t  context;
    struct rtskb    *comp_skb;
    struct rtskb    *next_skb;
    struct rtskb    *chain_end;
#endif


    RTNET_ASSERT(skb != NULL, return;);
    RTNET_ASSERT(skb->pool != NULL, return;);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
    next_skb  = skb;
    chain_end = skb->chain_end;

    do {
	skb      = next_skb;
	next_skb = skb->next;

	rtdm_lock_get_irqsave(&rtcap_lock, context);

	if (skb->cap_flags & RTSKB_CAP_SHARED) {
	    skb->cap_flags &= ~RTSKB_CAP_SHARED;

	    comp_skb  = skb->cap_comp_skb;
	    skb->pool = xchg(&comp_skb->pool, skb->pool);

	    rtdm_lock_put_irqrestore(&rtcap_lock, context);

	    rtskb_pool_queue_tail(comp_skb->pool, comp_skb);
	}
	else {
	    rtdm_lock_put_irqrestore(&rtcap_lock, context);

	    skb->chain_end = skb;
	    rtskb_pool_queue_tail(skb->pool, skb);
	}

    } while (chain_end != skb);

#else  /* CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP */

    rtskb_pool_queue_tail(skb->pool, skb);


#endif /* CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP */
}

EXPORT_SYMBOL_GPL(kfree_rtskb);


static int rtskb_nop_pool_trylock(void *cookie)
{
    return 1;
}

static void rtskb_nop_pool_unlock(void *cookie)
{
}

static const struct rtskb_pool_lock_ops rtskb_nop_pool_lock_ops = {
    .trylock = rtskb_nop_pool_trylock,
    .unlock = rtskb_nop_pool_unlock,
};


/***
 *  rtskb_pool_init
 *  @pool: pool to be initialized
 *  @initial_size: number of rtskbs to allocate
 *  return: number of actually allocated rtskbs
 */
unsigned int rtskb_pool_init(struct rtskb_pool *pool,
			    unsigned int initial_size,
			    const struct rtskb_pool_lock_ops *lock_ops,
			    void *lock_cookie)
{
    unsigned int i;

    rtskb_queue_init(&pool->queue);

    i = rtskb_pool_extend(pool, initial_size);

    rtskb_pools++;
    if (rtskb_pools > rtskb_pools_max)
	rtskb_pools_max = rtskb_pools;

    pool->lock_ops = lock_ops ?: &rtskb_nop_pool_lock_ops;
    pool->lock_cookie = lock_cookie;

    return i;
}

EXPORT_SYMBOL_GPL(rtskb_pool_init);

static int rtskb_module_pool_trylock(void *cookie)
{
    int err = 1;
    if (cookie)
	err = try_module_get(cookie);
    return err;
}

static void rtskb_module_pool_unlock(void *cookie)
{
    if (cookie)
	module_put(cookie);
}

static const struct rtskb_pool_lock_ops rtskb_module_lock_ops = {
    .trylock = rtskb_module_pool_trylock,
    .unlock = rtskb_module_pool_unlock,
};

unsigned int __rtskb_module_pool_init(struct rtskb_pool *pool,
				    unsigned int initial_size,
				    struct module *module)
{
    return rtskb_pool_init(pool, initial_size, &rtskb_module_lock_ops, module);
}
EXPORT_SYMBOL_GPL(__rtskb_module_pool_init);


/***
 *  __rtskb_pool_release
 *  @pool: pool to release
 */
void rtskb_pool_release(struct rtskb_pool *pool)
{
    struct rtskb *skb;

    while ((skb = rtskb_dequeue(&pool->queue)) != NULL) {
	rtdev_unmap_rtskb(skb);
	kmem_cache_free(rtskb_slab_pool, skb);
	rtskb_amount--;
    }

    rtskb_pools--;
}

EXPORT_SYMBOL_GPL(rtskb_pool_release);


unsigned int rtskb_pool_extend(struct rtskb_pool *pool,
			       unsigned int add_rtskbs)
{
    unsigned int i;
    struct rtskb *skb;


    RTNET_ASSERT(pool != NULL, return -EINVAL;);

    for (i = 0; i < add_rtskbs; i++) {
	/* get rtskb from slab pool */
	if (!(skb = kmem_cache_alloc(rtskb_slab_pool, GFP_KERNEL))) {
	    printk(KERN_ERR "RTnet: rtskb allocation from slab pool failed\n");
	    break;
	}

	/* fill the header with zero */
	memset(skb, 0, sizeof(struct rtskb));

	skb->chain_end = skb;
	skb->pool = pool;
	skb->buf_start = ((unsigned char *)skb) + ALIGN_RTSKB_STRUCT_LEN;
#ifdef CONFIG_XENO_DRIVERS_NET_CHECKED
	skb->buf_end = skb->buf_start + SKB_DATA_ALIGN(RTSKB_SIZE) - 1;
#endif

	if (rtdev_map_rtskb(skb) < 0)
	    break;

	rtskb_queue_tail(&pool->queue, skb);

	rtskb_amount++;
	if (rtskb_amount > rtskb_amount_max)
	    rtskb_amount_max = rtskb_amount;
    }

    return i;
}


unsigned int rtskb_pool_shrink(struct rtskb_pool *pool,
			       unsigned int rem_rtskbs)
{
    unsigned int    i;
    struct rtskb    *skb;


    for (i = 0; i < rem_rtskbs; i++) {
	if ((skb = rtskb_dequeue(&pool->queue)) == NULL)
	    break;

	rtdev_unmap_rtskb(skb);
	kmem_cache_free(rtskb_slab_pool, skb);
	rtskb_amount--;
    }

    return i;
}


/* Note: acquires only the first skb of a chain! */
int rtskb_acquire(struct rtskb *rtskb, struct rtskb_pool *comp_pool)
{
    struct rtskb *comp_rtskb;
    struct rtskb_pool *release_pool;
    rtdm_lockctx_t context;


    rtdm_lock_get_irqsave(&comp_pool->queue.lock, context);

    comp_rtskb = __rtskb_pool_dequeue(comp_pool);
    if (!comp_rtskb) {
	rtdm_lock_put_irqrestore(&comp_pool->queue.lock, context);
	return -ENOMEM;
    }

    rtdm_lock_put(&comp_pool->queue.lock);

    comp_rtskb->chain_end = comp_rtskb;
    comp_rtskb->pool = release_pool = rtskb->pool;

    rtdm_lock_get(&release_pool->queue.lock);

    __rtskb_pool_queue_tail(release_pool, comp_rtskb);

    rtdm_lock_put_irqrestore(&release_pool->queue.lock, context);

    rtskb->pool = comp_pool;

    return 0;
}

EXPORT_SYMBOL_GPL(rtskb_acquire);


/* clone rtskb to another, allocating the new rtskb from pool */
struct rtskb* rtskb_clone(struct rtskb *rtskb, struct rtskb_pool *pool)
{
    struct rtskb    *clone_rtskb;
    unsigned int    total_len;

    clone_rtskb = alloc_rtskb(rtskb->end - rtskb->buf_start, pool);
    if (clone_rtskb == NULL)
	return NULL;

    /* Note: We don't clone
	- rtskb.sk
	- rtskb.xmit_stamp
       until real use cases show up. */

    clone_rtskb->priority   = rtskb->priority;
    clone_rtskb->rtdev      = rtskb->rtdev;
    clone_rtskb->time_stamp = rtskb->time_stamp;

    clone_rtskb->mac.raw    = clone_rtskb->buf_start;
    clone_rtskb->nh.raw     = clone_rtskb->buf_start;
    clone_rtskb->h.raw      = clone_rtskb->buf_start;

    clone_rtskb->data       += rtskb->data - rtskb->buf_start;
    clone_rtskb->tail       += rtskb->tail - rtskb->buf_start;
    clone_rtskb->mac.raw    += rtskb->mac.raw - rtskb->buf_start;
    clone_rtskb->nh.raw     += rtskb->nh.raw - rtskb->buf_start;
    clone_rtskb->h.raw      += rtskb->h.raw - rtskb->buf_start;

    clone_rtskb->protocol   = rtskb->protocol;
    clone_rtskb->pkt_type   = rtskb->pkt_type;

    clone_rtskb->ip_summed  = rtskb->ip_summed;
    clone_rtskb->csum       = rtskb->csum;

    total_len = rtskb->len + rtskb->data - rtskb->mac.raw;
    memcpy(clone_rtskb->mac.raw, rtskb->mac.raw, total_len);
    clone_rtskb->len = rtskb->len;

    return clone_rtskb;
}

EXPORT_SYMBOL_GPL(rtskb_clone);


int rtskb_pools_init(void)
{
    rtskb_slab_pool = kmem_cache_create("rtskb_slab_pool",
	ALIGN_RTSKB_STRUCT_LEN + SKB_DATA_ALIGN(RTSKB_SIZE),
	0, SLAB_HWCACHE_ALIGN, NULL);
    if (rtskb_slab_pool == NULL)
	return -ENOMEM;

    /* reset the statistics (cache is accounted separately) */
    rtskb_pools      = 0;
    rtskb_pools_max  = 0;
    rtskb_amount     = 0;
    rtskb_amount_max = 0;

    /* create the global rtskb pool */
    if (rtskb_module_pool_init(&global_pool, global_rtskbs) < global_rtskbs)
	goto err_out;

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_RTCAP)
    rtdm_lock_init(&rtcap_lock);
#endif

    return 0;

err_out:
    rtskb_pool_release(&global_pool);
    kmem_cache_destroy(rtskb_slab_pool);

    return -ENOMEM;
}


void rtskb_pools_release(void)
{
    rtskb_pool_release(&global_pool);
    kmem_cache_destroy(rtskb_slab_pool);
}
