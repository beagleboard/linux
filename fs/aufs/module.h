/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2005-2022 Junjiro R. Okajima
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * module initialization and module-global
 */

#ifndef __AUFS_MODULE_H__
#define __AUFS_MODULE_H__

#ifdef __KERNEL__

#include <linux/slab.h>
#include "debug.h"
#include "dentry.h"
#include "dir.h"
#include "file.h"
#include "inode.h"

struct path;
struct seq_file;

/* module parameters */
extern int sysaufs_brs;
extern bool au_userns;

/* ---------------------------------------------------------------------- */

extern int au_dir_roflags;

void *au_krealloc(void *p, unsigned int new_sz, gfp_t gfp, int may_shrink);
void *au_kzrealloc(void *p, unsigned int nused, unsigned int new_sz, gfp_t gfp,
		   int may_shrink);

/*
 * Comparing the size of the object with sizeof(struct rcu_head)
 * case 1: object is always larger
 *	--> au_kfree_rcu() or au_kfree_do_rcu()
 * case 2: object is always smaller
 *	--> au_kfree_small()
 * case 3: object can be any size
 *	--> au_kfree_try_rcu()
 */

static inline void au_kfree_do_rcu(const void *p)
{
	struct {
		struct rcu_head rcu;
	} *a = (void *)p;

	kfree_rcu(a, rcu);
}

#define au_kfree_rcu(_p) do {						\
		typeof(_p) p = (_p);					\
		BUILD_BUG_ON(sizeof(*p) < sizeof(struct rcu_head));	\
		if (p)							\
			au_kfree_do_rcu(p);				\
	} while (0)

#define au_kfree_do_sz_test(sz)	(sz >= sizeof(struct rcu_head))
#define au_kfree_sz_test(p)	(p && au_kfree_do_sz_test(ksize(p)))

static inline void au_kfree_try_rcu(const void *p)
{
	if (!p)
		return;
	if (au_kfree_sz_test(p))
		au_kfree_do_rcu(p);
	else
		kfree(p);
}

static inline void au_kfree_small(const void *p)
{
	if (!p)
		return;
	AuDebugOn(au_kfree_sz_test(p));
	kfree(p);
}

static inline int au_kmidx_sub(size_t sz, size_t new_sz)
{
#ifndef CONFIG_SLOB
	return __kmalloc_index(sz, false) - __kmalloc_index(new_sz, false);
#else
	return -1; /* SLOB is untested */
#endif
}

int au_seq_path(struct seq_file *seq, struct path *path);

#ifdef CONFIG_PROC_FS
/* procfs.c */
int __init au_procfs_init(void);
void au_procfs_fin(void);
#else
AuStubInt0(au_procfs_init, void);
AuStubVoid(au_procfs_fin, void);
#endif

/* ---------------------------------------------------------------------- */

/* kmem cache */
enum {
	AuCache_DINFO,
	AuCache_ICNTNR,
	AuCache_FINFO,
	AuCache_VDIR,
	AuCache_DEHSTR,
	AuCache_HNOTIFY, /* must be last */
	AuCache_Last
};

extern struct kmem_cache *au_cache[AuCache_Last];

#define AuCacheFlags		(SLAB_RECLAIM_ACCOUNT | SLAB_MEM_SPREAD)
#define AuCache(type)		KMEM_CACHE(type, AuCacheFlags)
#define AuCacheCtor(type, ctor)	\
	kmem_cache_create(#type, sizeof(struct type), \
			  __alignof__(struct type), AuCacheFlags, ctor)

#define AuCacheFuncAlloc(name, index)					\
	static inline struct au_##name *au_cache_alloc_##name(void)	\
	{ return kmem_cache_alloc(au_cache[AuCache_##index], GFP_NOFS); }

#define AuCacheFuncs(name, index)					\
	static inline void au_cache_free_##name##_norcu(struct au_##name *p) \
	{ kmem_cache_free(au_cache[AuCache_##index], p); }		\
									\
	static inline void au_cache_free_##name##_rcu_cb(struct rcu_head *rcu) \
	{ void *p = rcu;						\
		p -= offsetof(struct au_##name, rcu);			\
		kmem_cache_free(au_cache[AuCache_##index], p); }	\
	static inline void au_cache_free_##name##_rcu(struct au_##name *p) \
	{ BUILD_BUG_ON(sizeof(struct au_##name) < sizeof(struct rcu_head)); \
		call_rcu(&p->rcu, au_cache_free_##name##_rcu_cb); }	\
									\
	static inline void au_cache_free_##name(struct au_##name *p)	\
	{ /* au_cache_free_##name##_norcu(p); */			\
		au_cache_free_##name##_rcu(p); }

AuCacheFuncs(dinfo, DINFO);
AuCacheFuncAlloc(dinfo, DINFO);

AuCacheFuncs(icntnr, ICNTNR);
static inline struct au_icntnr *au_cache_alloc_icntnr(struct super_block *sb)
{ return alloc_inode_sb(sb, au_cache[AuCache_ICNTNR], GFP_NOFS); }

AuCacheFuncs(finfo, FINFO);
AuCacheFuncAlloc(finfo, FINFO);

AuCacheFuncs(vdir, VDIR);
AuCacheFuncAlloc(vdir, VDIR);

AuCacheFuncs(vdir_dehstr, DEHSTR);
AuCacheFuncAlloc(vdir_dehstr, DEHSTR);

#ifdef CONFIG_AUFS_HNOTIFY
AuCacheFuncs(hnotify, HNOTIFY);
AuCacheFuncAlloc(hnotify, HNOTIFY);
#endif

#endif /* __KERNEL__ */
#endif /* __AUFS_MODULE_H__ */
