/*
 * Copyright (C) 2018 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/random.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/heap.h>
#include <rtdm/uapi/testing.h>
#include <rtdm/driver.h>

#define complain(__fmt, __args...)	\
	printk(XENO_WARNING "heap check: " __fmt "\n", ##__args)

static struct xnheap test_heap = {
	.name = "test_heap"
};

enum pattern {
	alphabet_series,
	digit_series,
	binary_series,
};

struct chunk {
	void *ptr;
	enum pattern pattern;
};

struct runstats {
	struct rttst_heap_stats stats;
	struct runstats *next;
};

static struct runstats *statistics;

static int nrstats;

static inline void breathe(int loops)
{
	if ((loops % 1000) == 0)
		rtdm_task_sleep(300000ULL);
}

static inline void do_swap(void *left, void *right, const size_t size)
{
	char trans[size];

	memcpy(trans, left, size);
	memcpy(left, right, size);
	memcpy(right, trans, size);
}

static void random_shuffle(void *vbase, size_t nmemb, const size_t size)
{
	struct {
		char x[size];
	} __attribute__((packed)) *base = vbase;
	unsigned int j, k;

	for (j = nmemb; j > 0; j--) {
		k = (unsigned int)(prandom_u32() % nmemb) + 1;
		if (j == k)
			continue;
		do_swap(&base[j - 1], &base[k - 1], size);
	}
}

static void fill_pattern(char *p, size_t size, enum pattern pat)
{
	unsigned int val, count;

	switch (pat) {
	case alphabet_series:
		val = 'a';
		count = 26;
		break;
	case digit_series:
		val = '0';
		count = 10;
		break;
	default:
		val = 0;
		count = 255;
		break;
	}

	while (size-- > 0) {
		*p++ = (char)(val % count);
		val++;
	}
}

static int check_pattern(const char *p, size_t size, enum pattern pat)
{
	unsigned int val, count;

	switch (pat) {
	case alphabet_series:
		val = 'a';
		count = 26;
		break;
	case digit_series:
		val = '0';
		count = 10;
		break;
	default:
		val = 0;
		count = 255;
		break;
	}

	while (size-- > 0) {
		if (*p++ != (char)(val % count))
			return 0;
		val++;
	}

	return 1;
}

static size_t find_largest_free(size_t free_size, size_t block_size)
{
	void *p;

	for (;;) {
		p = xnheap_alloc(&test_heap, free_size);
		if (p) {
			xnheap_free(&test_heap, p);
			break;
		}
		if (free_size <= block_size)
			break;
		free_size -= block_size;
	}

	return free_size;
}

static int test_seq(size_t heap_size, size_t block_size, int flags)
{
	long alloc_sum_ns, alloc_avg_ns, free_sum_ns, free_avg_ns,
		alloc_max_ns, free_max_ns, d;
	size_t user_size, largest_free, maximum_free, freed;
	int ret, n, k, maxblocks, nrblocks;
	nanosecs_rel_t start, end;
	struct chunk *chunks;
	struct runstats *st;
	bool done_frag;
	void *mem, *p;

	maxblocks = heap_size / block_size;

	mem = vmalloc(heap_size);
	if (mem == NULL)
		return -ENOMEM;

	ret = xnheap_init(&test_heap, mem, heap_size);
	if (ret) {
		complain("cannot init heap with size %zu",
		       heap_size);
		goto out;
	}

	chunks = vmalloc(sizeof(*chunks) * maxblocks);
	if (chunks == NULL) {
		ret = -ENOMEM;
		goto no_chunks;
	}
	memset(chunks, 0, sizeof(*chunks) * maxblocks);

	ret = xnthread_harden();
	if (ret)
		goto done;

	if (xnheap_get_size(&test_heap) != heap_size) {
		complain("memory size inconsistency (%zu / %zu bytes)",
			 heap_size, xnheap_get_size(&test_heap));
		goto bad;
	}

	user_size = 0;
	alloc_avg_ns = 0;
	free_avg_ns = 0;
	alloc_max_ns = 0;
	free_max_ns = 0;
	maximum_free = 0;
	largest_free = 0;

	for (n = 0, alloc_sum_ns = 0; ; n++) {
		start = rtdm_clock_read_monotonic();
		p = xnheap_alloc(&test_heap, block_size);
		end = rtdm_clock_read_monotonic();
		d = end - start;
		if (d > alloc_max_ns)
			alloc_max_ns = d;
		alloc_sum_ns += d;
		if (p == NULL)
			break;
		user_size += block_size;
		if (n >= maxblocks) {
			complain("too many blocks fetched"
			       " (heap=%zu, block=%zu, "
			       "got more than %d blocks)",
			       heap_size, block_size, maxblocks);
			goto bad;
		}
		chunks[n].ptr = p;
		if (flags & RTTST_HEAPCHECK_PATTERN) {
			chunks[n].pattern = (enum pattern)(prandom_u32() % 3);
			fill_pattern(chunks[n].ptr, block_size, chunks[n].pattern);
		}
		breathe(n);
	}

	nrblocks = n;
	if (nrblocks == 0)
		goto do_stats;

	if ((flags & RTTST_HEAPCHECK_ZEROOVRD) && nrblocks != maxblocks) {
		complain("too few blocks fetched, unexpected overhead"
			 " (heap=%zu, block=%zu, "
			 "got %d, less than %d blocks)",
			 heap_size, block_size, nrblocks, maxblocks);
		goto bad;
	}

	breathe(0);

	/* Make sure we did not trash any busy block while allocating. */
	if (flags & RTTST_HEAPCHECK_PATTERN) {
		for (n = 0; n < nrblocks; n++) {
			if (!check_pattern(chunks[n].ptr, block_size,
					   chunks[n].pattern)) {
				complain("corrupted block #%d on alloc"
					 " sequence (pattern %d)",
					 n, chunks[n].pattern);
				goto bad;
			}
			breathe(n);
		}
	}
	
	if (flags & RTTST_HEAPCHECK_SHUFFLE)
		random_shuffle(chunks, nrblocks, sizeof(*chunks));

	/*
	 * Release all blocks.
	 */
	for (n = 0, free_sum_ns = 0, freed = 0, done_frag = false;
	     n < nrblocks; n++) {
		start = rtdm_clock_read_monotonic();
		xnheap_free(&test_heap, chunks[n].ptr);
		end = rtdm_clock_read_monotonic();
		d = end - start;
		if (d > free_max_ns)
			free_max_ns = d;
		free_sum_ns += d;
		chunks[n].ptr = NULL;
		/* Make sure we did not trash busy blocks while freeing. */
		if (flags & RTTST_HEAPCHECK_PATTERN) {
			for (k = 0; k < nrblocks; k++) {
				if (chunks[k].ptr &&
				    !check_pattern(chunks[k].ptr, block_size,
						   chunks[k].pattern)) {
					complain("corrupted block #%d on release"
						 " sequence (pattern %d)",
						 k, chunks[k].pattern);
					goto bad;
				}
				breathe(k);
			}
		}
		freed += block_size;
		/*
		 * Get a sense of the fragmentation for the tested
		 * allocation pattern, heap and block sizes when half
		 * of the usable heap size should be available to us.
		 * NOTE: user_size excludes the overhead, this is
		 * actually what we managed to get from the current
		 * heap out of the allocation loop.
		 */
		if (!done_frag && freed >= user_size / 2) {
			/* Calculate the external fragmentation. */
			largest_free = find_largest_free(freed, block_size);
			maximum_free = freed;
			done_frag = true;
		}
		breathe(n);
	}

	/*
	 * If the deallocation mechanism is broken, we might not be
	 * able to reproduce the same allocation pattern with the same
	 * outcome, check this.
	 */
	if (flags & RTTST_HEAPCHECK_HOT) {
		for (n = 0, alloc_max_ns = alloc_sum_ns = 0; ; n++) {
			start = rtdm_clock_read_monotonic();
			p = xnheap_alloc(&test_heap, block_size);
			end = rtdm_clock_read_monotonic();
			d = end - start;
			if (d > alloc_max_ns)
				alloc_max_ns = d;
			alloc_sum_ns += d;
			if (p == NULL)
				break;
			if (n >= maxblocks) {
				complain("too many blocks fetched during hot pass"
					 " (heap=%zu, block=%zu, "
					 "got more than %d blocks)",
					 heap_size, block_size, maxblocks);
				goto bad;
			}
			chunks[n].ptr = p;
			breathe(n);
		}
		if (n != nrblocks) {
			complain("inconsistent block count fetched"
				 " during hot pass (heap=%zu, block=%zu, "
				 "got %d blocks vs %d during alloc)",
				 heap_size, block_size, n, nrblocks);
			goto bad;
		}
		for (n = 0, free_max_ns = free_sum_ns = 0; n < nrblocks; n++) {
			start = rtdm_clock_read_monotonic();
			xnheap_free(&test_heap, chunks[n].ptr);
			end = rtdm_clock_read_monotonic();
			d = end - start;
			if (d > free_max_ns)
				free_max_ns = d;
			free_sum_ns += d;
			breathe(n);
		}
	}

	alloc_avg_ns = alloc_sum_ns / nrblocks;
	free_avg_ns = free_sum_ns / nrblocks;

	if ((flags & RTTST_HEAPCHECK_ZEROOVRD) && heap_size != user_size) {
		complain("unexpected overhead reported");
		goto bad;
	}

	if (xnheap_get_used(&test_heap) > 0) {
		complain("memory leakage reported: %zu bytes missing",
			 xnheap_get_used(&test_heap));
		goto bad;
	}
		
do_stats:
	xnthread_relax(0, 0);
	ret = 0;
	/*
	 * Don't report stats when running a pattern check, timings
	 * are affected.
	 */
	if (!(flags & RTTST_HEAPCHECK_PATTERN)) {
		st = kmalloc(sizeof(*st), GFP_KERNEL);
		if (st == NULL) {
			complain("failed allocating memory");
			ret = -ENOMEM;
			goto out;
		}
		st->stats.heap_size = heap_size;
		st->stats.user_size = user_size;
		st->stats.block_size = block_size;
		st->stats.nrblocks = nrblocks;
		st->stats.alloc_avg_ns = alloc_avg_ns;
		st->stats.alloc_max_ns = alloc_max_ns;
		st->stats.free_avg_ns = free_avg_ns;
		st->stats.free_max_ns = free_max_ns;
		st->stats.maximum_free = maximum_free;
		st->stats.largest_free = largest_free;
		st->stats.flags = flags;
		st->next = statistics;
		statistics = st;
		nrstats++;
	}

done:
	vfree(chunks);
no_chunks:
	xnheap_destroy(&test_heap);
out:
	vfree(mem);

	return ret;
bad:
	xnthread_relax(0, 0);
	ret = -EPROTO;
	goto done;
}

static int collect_stats(struct rtdm_fd *fd,
			 struct rttst_heap_stats __user *buf, int nr)
{
	struct runstats *p, *next;
	int ret, n;

	if (nr < 0)
		return -EINVAL;

	for (p = statistics, n = nr; p && n > 0 && nrstats > 0;
	     n--, nrstats--, p = next, buf += sizeof(p->stats)) {
		ret = rtdm_copy_to_user(fd, buf, &p->stats, sizeof(p->stats));
		if (ret)
			return ret;
		next = p->next;
		statistics = next;
		kfree(p);
	}

	return nr - n;
}

static void heapcheck_close(struct rtdm_fd *fd)
{
	struct runstats *p, *next;

	for (p = statistics; p; p = next) {
		next = p->next;
		kfree(p);
	}

	statistics = NULL;
}

static int heapcheck_ioctl(struct rtdm_fd *fd,
			   unsigned int request, void __user *arg)
{
	struct rttst_heap_stathdr sthdr;
	struct rttst_heap_parms parms;
	int ret;

	switch (request) {
	case RTTST_RTIOC_HEAP_CHECK:
		ret = rtdm_copy_from_user(fd, &parms, arg, sizeof(parms));
		if (ret)
			return ret;
		ret = test_seq(parms.heap_size,
			       parms.block_size,
			       parms.flags);
		if (ret)
			return ret;
		parms.nrstats = nrstats;
		ret = rtdm_copy_to_user(fd, arg, &parms, sizeof(parms));
		break;
	case RTTST_RTIOC_HEAP_STAT_COLLECT:
		sthdr.buf = NULL;
		ret = rtdm_copy_from_user(fd, &sthdr, arg, sizeof(sthdr));
		if (ret)
			return ret;
		ret = collect_stats(fd, sthdr.buf, sthdr.nrstats);
		if (ret < 0)
			return ret;
		sthdr.nrstats = ret;
		ret = rtdm_copy_to_user(fd, arg, &sthdr, sizeof(sthdr));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct rtdm_driver heapcheck_driver = {
	.profile_info		= RTDM_PROFILE_INFO(heap_check,
						    RTDM_CLASS_TESTING,
						    RTDM_SUBCLASS_HEAPCHECK,
						    RTTST_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.ops = {
		.close		= heapcheck_close,
		.ioctl_nrt	= heapcheck_ioctl,
	},
};

static struct rtdm_device heapcheck_device = {
	.driver = &heapcheck_driver,
	.label = "heapcheck",
};

static int __init heapcheck_init(void)
{
	return rtdm_dev_register(&heapcheck_device);
}

static void __exit heapcheck_exit(void)
{
	rtdm_dev_unregister(&heapcheck_device);
}

module_init(heapcheck_init);
module_exit(heapcheck_exit);

MODULE_LICENSE("GPL");
