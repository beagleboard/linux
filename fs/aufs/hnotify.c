// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005-2021 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
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
 * abstraction to notify the direct changes on lower directories
 */

/* #include <linux/iversion.h> */
#include "aufs.h"

int au_hn_alloc(struct au_hinode *hinode, struct inode *inode)
{
	int err;
	struct au_hnotify *hn;

	err = -ENOMEM;
	hn = au_cache_alloc_hnotify();
	if (hn) {
		hn->hn_aufs_inode = inode;
		hinode->hi_notify = hn;
		err = au_hnotify_op.alloc(hinode);
		AuTraceErr(err);
		if (unlikely(err)) {
			hinode->hi_notify = NULL;
			au_cache_free_hnotify(hn);
			/*
			 * The upper dir was removed by udba, but the same named
			 * dir left. In this case, aufs assigns a new inode
			 * number and set the monitor again.
			 * For the lower dir, the old monitor is still left.
			 */
			if (err == -EEXIST)
				err = 0;
		}
	}

	AuTraceErr(err);
	return err;
}

void au_hn_free(struct au_hinode *hinode)
{
	struct au_hnotify *hn;

	hn = hinode->hi_notify;
	if (hn) {
		hinode->hi_notify = NULL;
		if (au_hnotify_op.free(hinode, hn))
			au_cache_free_hnotify(hn);
	}
}

/* ---------------------------------------------------------------------- */

void au_hn_ctl(struct au_hinode *hinode, int do_set)
{
	if (hinode->hi_notify)
		au_hnotify_op.ctl(hinode, do_set);
}

void au_hn_reset(struct inode *inode, unsigned int flags)
{
	aufs_bindex_t bindex, bbot;
	struct inode *hi;
	struct dentry *iwhdentry;

	bbot = au_ibbot(inode);
	for (bindex = au_ibtop(inode); bindex <= bbot; bindex++) {
		hi = au_h_iptr(inode, bindex);
		if (!hi)
			continue;

		/* inode_lock_nested(hi, AuLsc_I_CHILD); */
		iwhdentry = au_hi_wh(inode, bindex);
		if (iwhdentry)
			dget(iwhdentry);
		au_igrab(hi);
		au_set_h_iptr(inode, bindex, NULL, 0);
		au_set_h_iptr(inode, bindex, au_igrab(hi),
			      flags & ~AuHi_XINO);
		iput(hi);
		dput(iwhdentry);
		/* inode_unlock(hi); */
	}
}

/* ---------------------------------------------------------------------- */

static int hn_xino(struct inode *inode, struct inode *h_inode)
{
	int err;
	aufs_bindex_t bindex, bbot, bfound, btop;
	struct inode *h_i;

	err = 0;
	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		pr_warn("branch root dir was changed\n");
		goto out;
	}

	bfound = -1;
	bbot = au_ibbot(inode);
	btop = au_ibtop(inode);
#if 0 /* reserved for future use */
	if (bindex == bbot) {
		/* keep this ino in rename case */
		goto out;
	}
#endif
	for (bindex = btop; bindex <= bbot; bindex++)
		if (au_h_iptr(inode, bindex) == h_inode) {
			bfound = bindex;
			break;
		}
	if (bfound < 0)
		goto out;

	for (bindex = btop; bindex <= bbot; bindex++) {
		h_i = au_h_iptr(inode, bindex);
		if (!h_i)
			continue;

		err = au_xino_write(inode->i_sb, bindex, h_i->i_ino, /*ino*/0);
		/* ignore this error */
		/* bad action? */
	}

	/* children inode number will be broken */

out:
	AuTraceErr(err);
	return err;
}

static int hn_gen_tree(struct dentry *dentry)
{
	int err, i, j, ndentry;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;
	struct dentry **dentries;

	err = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(err))
		goto out;
	err = au_dcsub_pages(&dpages, dentry, NULL, NULL);
	if (unlikely(err))
		goto out_dpages;

	for (i = 0; i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		dentries = dpage->dentries;
		ndentry = dpage->ndentry;
		for (j = 0; j < ndentry; j++) {
			struct dentry *d;

			d = dentries[j];
			if (IS_ROOT(d))
				continue;

			au_digen_dec(d);
			if (d_really_is_positive(d))
				/* todo: reset children xino?
				   cached children only? */
				au_iigen_dec(d_inode(d));
		}
	}

out_dpages:
	au_dpages_free(&dpages);
out:
	return err;
}

/*
 * return 0 if processed.
 */
static int hn_gen_by_inode(char *name, unsigned int nlen, struct inode *inode,
			   const unsigned int isdir)
{
	int err;
	struct dentry *d;
	struct qstr *dname;

	err = 1;
	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		pr_warn("branch root dir was changed\n");
		err = 0;
		goto out;
	}

	if (!isdir) {
		AuDebugOn(!name);
		au_iigen_dec(inode);
		spin_lock(&inode->i_lock);
		hlist_for_each_entry(d, &inode->i_dentry, d_u.d_alias) {
			spin_lock(&d->d_lock);
			dname = &d->d_name;
			if (dname->len != nlen
			    && memcmp(dname->name, name, nlen)) {
				spin_unlock(&d->d_lock);
				continue;
			}
			err = 0;
			au_digen_dec(d);
			spin_unlock(&d->d_lock);
			break;
		}
		spin_unlock(&inode->i_lock);
	} else {
		au_fset_si(au_sbi(inode->i_sb), FAILED_REFRESH_DIR);
		d = d_find_any_alias(inode);
		if (!d) {
			au_iigen_dec(inode);
			goto out;
		}

		spin_lock(&d->d_lock);
		dname = &d->d_name;
		if (dname->len == nlen && !memcmp(dname->name, name, nlen)) {
			spin_unlock(&d->d_lock);
			err = hn_gen_tree(d);
			spin_lock(&d->d_lock);
		}
		spin_unlock(&d->d_lock);
		dput(d);
	}

out:
	AuTraceErr(err);
	return err;
}

static int hn_gen_by_name(struct dentry *dentry, const unsigned int isdir)
{
	int err;

	if (IS_ROOT(dentry)) {
		pr_warn("branch root dir was changed\n");
		return 0;
	}

	err = 0;
	if (!isdir) {
		au_digen_dec(dentry);
		if (d_really_is_positive(dentry))
			au_iigen_dec(d_inode(dentry));
	} else {
		au_fset_si(au_sbi(dentry->d_sb), FAILED_REFRESH_DIR);
		if (d_really_is_positive(dentry))
			err = hn_gen_tree(dentry);
	}

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* hnotify job flags */
#define AuHnJob_XINO0		1
#define AuHnJob_GEN		(1 << 1)
#define AuHnJob_DIRENT		(1 << 2)
#define AuHnJob_ISDIR		(1 << 3)
#define AuHnJob_TRYXINO0	(1 << 4)
#define AuHnJob_MNTPNT		(1 << 5)
#define au_ftest_hnjob(flags, name)	((flags) & AuHnJob_##name)
#define au_fset_hnjob(flags, name) \
	do { (flags) |= AuHnJob_##name; } while (0)
#define au_fclr_hnjob(flags, name) \
	do { (flags) &= ~AuHnJob_##name; } while (0)

enum {
	AuHn_CHILD,
	AuHn_PARENT,
	AuHnLast
};

struct au_hnotify_args {
	struct inode *h_dir, *dir, *h_child_inode;
	u32 mask;
	unsigned int flags[AuHnLast];
	unsigned int h_child_nlen;
	char h_child_name[];
};

struct hn_job_args {
	unsigned int flags;
	struct inode *inode, *h_inode, *dir, *h_dir;
	struct dentry *dentry;
	char *h_name;
	int h_nlen;
};

static int hn_job(struct hn_job_args *a)
{
	const unsigned int isdir = au_ftest_hnjob(a->flags, ISDIR);
	int e;

	/* reset xino */
	if (au_ftest_hnjob(a->flags, XINO0) && a->inode)
		hn_xino(a->inode, a->h_inode); /* ignore this error */

	if (au_ftest_hnjob(a->flags, TRYXINO0)
	    && a->inode
	    && a->h_inode) {
		inode_lock_shared_nested(a->h_inode, AuLsc_I_CHILD);
		if (!a->h_inode->i_nlink
		    && !(a->h_inode->i_state & I_LINKABLE))
			hn_xino(a->inode, a->h_inode); /* ignore this error */
		inode_unlock_shared(a->h_inode);
	}

	/* make the generation obsolete */
	if (au_ftest_hnjob(a->flags, GEN)) {
		e = -1;
		if (a->inode)
			e = hn_gen_by_inode(a->h_name, a->h_nlen, a->inode,
					      isdir);
		if (e && a->dentry)
			hn_gen_by_name(a->dentry, isdir);
		/* ignore this error */
	}

	/* make dir entries obsolete */
	if (au_ftest_hnjob(a->flags, DIRENT) && a->inode) {
		struct au_vdir *vdir;

		vdir = au_ivdir(a->inode);
		if (vdir)
			vdir->vd_jiffy = 0;
		/* IMustLock(a->inode); */
		/* inode_inc_iversion(a->inode); */
	}

	/* can do nothing but warn */
	if (au_ftest_hnjob(a->flags, MNTPNT)
	    && a->dentry
	    && d_mountpoint(a->dentry))
		pr_warn("mount-point %pd is removed or renamed\n", a->dentry);

	return 0;
}

/* ---------------------------------------------------------------------- */

static struct dentry *lookup_wlock_by_name(char *name, unsigned int nlen,
					   struct inode *dir)
{
	struct dentry *dentry, *d, *parent;
	struct qstr *dname;

	parent = d_find_any_alias(dir);
	if (!parent)
		return NULL;

	dentry = NULL;
	spin_lock(&parent->d_lock);
	list_for_each_entry(d, &parent->d_subdirs, d_child) {
		/* AuDbg("%pd\n", d); */
		spin_lock_nested(&d->d_lock, DENTRY_D_LOCK_NESTED);
		dname = &d->d_name;
		if (dname->len != nlen || memcmp(dname->name, name, nlen))
			goto cont_unlock;
		if (au_di(d))
			au_digen_dec(d);
		else
			goto cont_unlock;
		if (au_dcount(d) > 0) {
			dentry = dget_dlock(d);
			spin_unlock(&d->d_lock);
			break;
		}

cont_unlock:
		spin_unlock(&d->d_lock);
	}
	spin_unlock(&parent->d_lock);
	dput(parent);

	if (dentry)
		di_write_lock_child(dentry);

	return dentry;
}

static struct inode *lookup_wlock_by_ino(struct super_block *sb,
					 aufs_bindex_t bindex, ino_t h_ino)
{
	struct inode *inode;
	ino_t ino;
	int err;

	inode = NULL;
	err = au_xino_read(sb, bindex, h_ino, &ino);
	if (!err && ino)
		inode = ilookup(sb, ino);
	if (!inode)
		goto out;

	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		pr_warn("wrong root branch\n");
		iput(inode);
		inode = NULL;
		goto out;
	}

	ii_write_lock_child(inode);

out:
	return inode;
}

static void au_hn_bh(void *_args)
{
	struct au_hnotify_args *a = _args;
	struct super_block *sb;
	aufs_bindex_t bindex, bbot, bfound;
	unsigned char xino, try_iput;
	int err;
	struct inode *inode;
	ino_t h_ino;
	struct hn_job_args args;
	struct dentry *dentry;
	struct au_sbinfo *sbinfo;

	AuDebugOn(!_args);
	AuDebugOn(!a->h_dir);
	AuDebugOn(!a->dir);
	AuDebugOn(!a->mask);
	AuDbg("mask 0x%x, i%lu, hi%lu, hci%lu\n",
	      a->mask, a->dir->i_ino, a->h_dir->i_ino,
	      a->h_child_inode ? a->h_child_inode->i_ino : 0);

	inode = NULL;
	dentry = NULL;
	/*
	 * do not lock a->dir->i_mutex here
	 * because of d_revalidate() may cause a deadlock.
	 */
	sb = a->dir->i_sb;
	AuDebugOn(!sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!sbinfo);
	si_write_lock(sb, AuLock_NOPLMW);

	if (au_opt_test(sbinfo->si_mntflags, DIRREN))
		switch (a->mask & FS_EVENTS_POSS_ON_CHILD) {
		case FS_MOVED_FROM:
		case FS_MOVED_TO:
			AuWarn1("DIRREN with UDBA may not work correctly "
				"for the direct rename(2)\n");
		}

	ii_read_lock_parent(a->dir);
	bfound = -1;
	bbot = au_ibbot(a->dir);
	for (bindex = au_ibtop(a->dir); bindex <= bbot; bindex++)
		if (au_h_iptr(a->dir, bindex) == a->h_dir) {
			bfound = bindex;
			break;
		}
	ii_read_unlock(a->dir);
	if (unlikely(bfound < 0))
		goto out;

	xino = !!au_opt_test(au_mntflags(sb), XINO);
	h_ino = 0;
	if (a->h_child_inode)
		h_ino = a->h_child_inode->i_ino;

	if (a->h_child_nlen
	    && (au_ftest_hnjob(a->flags[AuHn_CHILD], GEN)
		|| au_ftest_hnjob(a->flags[AuHn_CHILD], MNTPNT)))
		dentry = lookup_wlock_by_name(a->h_child_name, a->h_child_nlen,
					      a->dir);
	try_iput = 0;
	if (dentry && d_really_is_positive(dentry))
		inode = d_inode(dentry);
	if (xino && !inode && h_ino
	    && (au_ftest_hnjob(a->flags[AuHn_CHILD], XINO0)
		|| au_ftest_hnjob(a->flags[AuHn_CHILD], TRYXINO0)
		|| au_ftest_hnjob(a->flags[AuHn_CHILD], GEN))) {
		inode = lookup_wlock_by_ino(sb, bfound, h_ino);
		try_iput = 1;
	}

	args.flags = a->flags[AuHn_CHILD];
	args.dentry = dentry;
	args.inode = inode;
	args.h_inode = a->h_child_inode;
	args.dir = a->dir;
	args.h_dir = a->h_dir;
	args.h_name = a->h_child_name;
	args.h_nlen = a->h_child_nlen;
	err = hn_job(&args);
	if (dentry) {
		if (au_di(dentry))
			di_write_unlock(dentry);
		dput(dentry);
	}
	if (inode && try_iput) {
		ii_write_unlock(inode);
		iput(inode);
	}

	ii_write_lock_parent(a->dir);
	args.flags = a->flags[AuHn_PARENT];
	args.dentry = NULL;
	args.inode = a->dir;
	args.h_inode = a->h_dir;
	args.dir = NULL;
	args.h_dir = NULL;
	args.h_name = NULL;
	args.h_nlen = 0;
	err = hn_job(&args);
	ii_write_unlock(a->dir);

out:
	iput(a->h_child_inode);
	iput(a->h_dir);
	iput(a->dir);
	si_write_unlock(sb);
	au_nwt_done(&sbinfo->si_nowait);
	au_kfree_rcu(a);
}

/* ---------------------------------------------------------------------- */

int au_hnotify(struct inode *h_dir, struct au_hnotify *hnotify, u32 mask,
	       const struct qstr *h_child_qstr, struct inode *h_child_inode)
{
	int err, len;
	unsigned int flags[AuHnLast], f;
	unsigned char isdir, isroot, wh;
	struct inode *dir;
	struct au_hnotify_args *args;
	char *p, *h_child_name;

	err = 0;
	AuDebugOn(!hnotify || !hnotify->hn_aufs_inode);
	dir = igrab(hnotify->hn_aufs_inode);
	if (!dir)
		goto out;

	isroot = (dir->i_ino == AUFS_ROOT_INO);
	wh = 0;
	h_child_name = (void *)h_child_qstr->name;
	len = h_child_qstr->len;
	if (h_child_name) {
		if (len > AUFS_WH_PFX_LEN
		    && !memcmp(h_child_name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
			h_child_name += AUFS_WH_PFX_LEN;
			len -= AUFS_WH_PFX_LEN;
			wh = 1;
		}
	}

	isdir = 0;
	if (h_child_inode)
		isdir = !!S_ISDIR(h_child_inode->i_mode);
	flags[AuHn_PARENT] = AuHnJob_ISDIR;
	flags[AuHn_CHILD] = 0;
	if (isdir)
		flags[AuHn_CHILD] = AuHnJob_ISDIR;
	au_fset_hnjob(flags[AuHn_PARENT], DIRENT);
	au_fset_hnjob(flags[AuHn_CHILD], GEN);
	switch (mask & ALL_FSNOTIFY_DIRENT_EVENTS) {
	case FS_MOVED_FROM:
	case FS_MOVED_TO:
		au_fset_hnjob(flags[AuHn_CHILD], XINO0);
		au_fset_hnjob(flags[AuHn_CHILD], MNTPNT);
		fallthrough;
	case FS_CREATE:
		AuDebugOn(!h_child_name);
		break;

	case FS_DELETE:
		/*
		 * aufs never be able to get this child inode.
		 * revalidation should be in d_revalidate()
		 * by checking i_nlink, i_generation or d_unhashed().
		 */
		AuDebugOn(!h_child_name);
		au_fset_hnjob(flags[AuHn_CHILD], TRYXINO0);
		au_fset_hnjob(flags[AuHn_CHILD], MNTPNT);
		break;

	default:
		AuDebugOn(1);
	}

	if (wh)
		h_child_inode = NULL;

	err = -ENOMEM;
	/* iput() and kfree() will be called in au_hnotify() */
	args = kmalloc(sizeof(*args) + len + 1, GFP_NOFS);
	if (unlikely(!args)) {
		AuErr1("no memory\n");
		iput(dir);
		goto out;
	}
	args->flags[AuHn_PARENT] = flags[AuHn_PARENT];
	args->flags[AuHn_CHILD] = flags[AuHn_CHILD];
	args->mask = mask;
	args->dir = dir;
	args->h_dir = igrab(h_dir);
	if (h_child_inode)
		h_child_inode = igrab(h_child_inode); /* can be NULL */
	args->h_child_inode = h_child_inode;
	args->h_child_nlen = len;
	if (len) {
		p = (void *)args;
		p += sizeof(*args);
		memcpy(p, h_child_name, len);
		p[len] = 0;
	}

	/* NFS fires the event for silly-renamed one from kworker */
	f = 0;
	if (!dir->i_nlink
	    || (au_test_nfs(h_dir->i_sb) && (mask & FS_DELETE)))
		f = AuWkq_NEST;
	err = au_wkq_nowait(au_hn_bh, args, dir->i_sb, f);
	if (unlikely(err)) {
		pr_err("wkq %d\n", err);
		iput(args->h_child_inode);
		iput(args->h_dir);
		iput(args->dir);
		au_kfree_rcu(args);
	}

out:
	return err;
}

/* ---------------------------------------------------------------------- */

int au_hnotify_reset_br(unsigned int udba, struct au_branch *br, int perm)
{
	int err;

	AuDebugOn(!(udba & AuOptMask_UDBA));

	err = 0;
	if (au_hnotify_op.reset_br)
		err = au_hnotify_op.reset_br(udba, br, perm);

	return err;
}

int au_hnotify_init_br(struct au_branch *br, int perm)
{
	int err;

	err = 0;
	if (au_hnotify_op.init_br)
		err = au_hnotify_op.init_br(br, perm);

	return err;
}

void au_hnotify_fin_br(struct au_branch *br)
{
	if (au_hnotify_op.fin_br)
		au_hnotify_op.fin_br(br);
}

static void au_hn_destroy_cache(void)
{
	kmem_cache_destroy(au_cache[AuCache_HNOTIFY]);
	au_cache[AuCache_HNOTIFY] = NULL;
}

int __init au_hnotify_init(void)
{
	int err;

	err = -ENOMEM;
	au_cache[AuCache_HNOTIFY] = AuCache(au_hnotify);
	if (au_cache[AuCache_HNOTIFY]) {
		err = 0;
		if (au_hnotify_op.init)
			err = au_hnotify_op.init();
		if (unlikely(err))
			au_hn_destroy_cache();
	}
	AuTraceErr(err);
	return err;
}

void au_hnotify_fin(void)
{
	if (au_hnotify_op.fin)
		au_hnotify_op.fin();

	/* cf. au_cache_fin() */
	if (au_cache[AuCache_HNOTIFY])
		au_hn_destroy_cache();
}
