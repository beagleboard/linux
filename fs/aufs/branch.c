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
 * branch management
 */

#include <linux/compat.h>
#include <linux/statfs.h>
#include "aufs.h"

/*
 * free a single branch
 */
static void au_br_do_free(struct au_branch *br)
{
	int i;
	struct au_wbr *wbr;
	struct au_dykey **key;

	au_hnotify_fin_br(br);
	/* always, regardless the mount option */
	au_dr_hino_free(&br->br_dirren);
	au_xino_put(br);

	AuLCntZero(au_lcnt_read(&br->br_nfiles, /*do_rev*/0));
	au_lcnt_fin(&br->br_nfiles, /*do_sync*/0);
	AuLCntZero(au_lcnt_read(&br->br_count, /*do_rev*/0));
	au_lcnt_fin(&br->br_count, /*do_sync*/0);

	wbr = br->br_wbr;
	if (wbr) {
		for (i = 0; i < AuBrWh_Last; i++)
			dput(wbr->wbr_wh[i]);
		AuDebugOn(atomic_read(&wbr->wbr_wh_running));
		AuRwDestroy(&wbr->wbr_wh_rwsem);
	}

	if (br->br_fhsm) {
		au_br_fhsm_fin(br->br_fhsm);
		au_kfree_try_rcu(br->br_fhsm);
	}

	key = br->br_dykey;
	for (i = 0; i < AuBrDynOp; i++, key++)
		if (*key)
			au_dy_put(*key);
		else
			break;

	/* recursive lock, s_umount of branch's */
	/* synchronize_rcu(); */ /* why? */
	lockdep_off();
	path_put(&br->br_path);
	lockdep_on();
	au_kfree_rcu(wbr);
	au_lcnt_wait_for_fin(&br->br_nfiles);
	au_lcnt_wait_for_fin(&br->br_count);
	/* I don't know why, but percpu_refcount requires this */
	/* synchronize_rcu(); */
	au_kfree_rcu(br);
}

/*
 * frees all branches
 */
void au_br_free(struct au_sbinfo *sbinfo)
{
	aufs_bindex_t bmax;
	struct au_branch **br;

	AuRwMustWriteLock(&sbinfo->si_rwsem);

	bmax = sbinfo->si_bbot + 1;
	br = sbinfo->si_branch;
	while (bmax--)
		au_br_do_free(*br++);
}

/*
 * find the index of a branch which is specified by @br_id.
 */
int au_br_index(struct super_block *sb, aufs_bindex_t br_id)
{
	aufs_bindex_t bindex, bbot;

	bbot = au_sbbot(sb);
	for (bindex = 0; bindex <= bbot; bindex++)
		if (au_sbr_id(sb, bindex) == br_id)
			return bindex;
	return -1;
}

/* ---------------------------------------------------------------------- */

/*
 * add a branch
 */

static int test_overlap(struct super_block *sb, struct dentry *h_adding,
			struct dentry *h_root)
{
	if (unlikely(h_adding == h_root
		     || au_test_loopback_overlap(sb, h_adding)))
		return 1;
	if (h_adding->d_sb != h_root->d_sb)
		return 0;
	return au_test_subdir(h_adding, h_root)
		|| au_test_subdir(h_root, h_adding);
}

/*
 * returns a newly allocated branch. @new_nbranch is a number of branches
 * after adding a branch.
 */
static struct au_branch *au_br_alloc(struct super_block *sb, int new_nbranch,
				     int perm)
{
	struct au_branch *add_branch;
	struct dentry *root;
	struct inode *inode;
	int err;

	err = -ENOMEM;
	add_branch = kzalloc(sizeof(*add_branch), GFP_NOFS);
	if (unlikely(!add_branch))
		goto out;
	add_branch->br_xino = au_xino_alloc(/*nfile*/1);
	if (unlikely(!add_branch->br_xino))
		goto out_br;
	err = au_hnotify_init_br(add_branch, perm);
	if (unlikely(err))
		goto out_xino;

	if (au_br_writable(perm)) {
		/* may be freed separately at changing the branch permission */
		add_branch->br_wbr = kzalloc(sizeof(*add_branch->br_wbr),
					     GFP_NOFS);
		if (unlikely(!add_branch->br_wbr))
			goto out_hnotify;
	}

	if (au_br_fhsm(perm)) {
		err = au_fhsm_br_alloc(add_branch);
		if (unlikely(err))
			goto out_wbr;
	}

	root = sb->s_root;
	err = au_sbr_realloc(au_sbi(sb), new_nbranch, /*may_shrink*/0);
	if (!err)
		err = au_di_realloc(au_di(root), new_nbranch, /*may_shrink*/0);
	if (!err) {
		inode = d_inode(root);
		err = au_hinode_realloc(au_ii(inode), new_nbranch,
					/*may_shrink*/0);
	}
	if (!err)
		return add_branch; /* success */

out_wbr:
	au_kfree_rcu(add_branch->br_wbr);
out_hnotify:
	au_hnotify_fin_br(add_branch);
out_xino:
	au_xino_put(add_branch);
out_br:
	au_kfree_rcu(add_branch);
out:
	return ERR_PTR(err);
}

/*
 * test if the branch permission is legal or not.
 */
static int test_br(struct inode *inode, int brperm, char *path)
{
	int err;

	err = (au_br_writable(brperm) && IS_RDONLY(inode));
	if (!err)
		goto out;

	err = -EINVAL;
	pr_err("write permission for readonly mount or inode, %s\n", path);

out:
	return err;
}

/*
 * returns:
 * 0: success, the caller will add it
 * plus: success, it is already unified, the caller should ignore it
 * minus: error
 */
static int test_add(struct super_block *sb, struct au_opt_add *add, int remount)
{
	int err;
	aufs_bindex_t bbot, bindex;
	struct dentry *root, *h_dentry;
	struct inode *inode, *h_inode;

	root = sb->s_root;
	bbot = au_sbbot(sb);
	if (unlikely(bbot >= 0
		     && au_find_dbindex(root, add->path.dentry) >= 0)) {
		err = 1;
		if (!remount) {
			err = -EINVAL;
			pr_err("%s duplicated\n", add->pathname);
		}
		goto out;
	}

	err = -ENOSPC; /* -E2BIG; */
	if (unlikely(AUFS_BRANCH_MAX <= add->bindex
		     || AUFS_BRANCH_MAX - 1 <= bbot)) {
		pr_err("number of branches exceeded %s\n", add->pathname);
		goto out;
	}

	err = -EDOM;
	if (unlikely(add->bindex < 0 || bbot + 1 < add->bindex)) {
		pr_err("bad index %d\n", add->bindex);
		goto out;
	}

	inode = d_inode(add->path.dentry);
	err = -ENOENT;
	if (unlikely(!inode->i_nlink)) {
		pr_err("no existence %s\n", add->pathname);
		goto out;
	}

	err = -EINVAL;
	if (unlikely(inode->i_sb == sb)) {
		pr_err("%s must be outside\n", add->pathname);
		goto out;
	}

	if (unlikely(au_test_fs_unsuppoted(inode->i_sb))) {
		pr_err("unsupported filesystem, %s (%s)\n",
		       add->pathname, au_sbtype(inode->i_sb));
		goto out;
	}

	if (unlikely(inode->i_sb->s_stack_depth)) {
		pr_err("already stacked, %s (%s)\n",
		       add->pathname, au_sbtype(inode->i_sb));
		goto out;
	}

	err = test_br(d_inode(add->path.dentry), add->perm, add->pathname);
	if (unlikely(err))
		goto out;

	if (bbot < 0)
		return 0; /* success */

	err = -EINVAL;
	for (bindex = 0; bindex <= bbot; bindex++)
		if (unlikely(test_overlap(sb, add->path.dentry,
					  au_h_dptr(root, bindex)))) {
			pr_err("%s is overlapped\n", add->pathname);
			goto out;
		}

	err = 0;
	if (au_opt_test(au_mntflags(sb), WARN_PERM)) {
		h_dentry = au_h_dptr(root, 0);
		h_inode = d_inode(h_dentry);
		if ((h_inode->i_mode & S_IALLUGO) != (inode->i_mode & S_IALLUGO)
		    || !uid_eq(h_inode->i_uid, inode->i_uid)
		    || !gid_eq(h_inode->i_gid, inode->i_gid))
			pr_warn("uid/gid/perm %s %u/%u/0%o, %u/%u/0%o\n",
				add->pathname,
				i_uid_read(inode), i_gid_read(inode),
				(inode->i_mode & S_IALLUGO),
				i_uid_read(h_inode), i_gid_read(h_inode),
				(h_inode->i_mode & S_IALLUGO));
	}

out:
	return err;
}

/*
 * initialize or clean the whiteouts for an adding branch
 */
static int au_br_init_wh(struct super_block *sb, struct au_branch *br,
			 int new_perm)
{
	int err, old_perm;
	aufs_bindex_t bindex;
	struct inode *h_inode;
	struct au_wbr *wbr;
	struct au_hinode *hdir;
	struct dentry *h_dentry;

	err = vfsub_mnt_want_write(au_br_mnt(br));
	if (unlikely(err))
		goto out;

	wbr = br->br_wbr;
	old_perm = br->br_perm;
	br->br_perm = new_perm;
	hdir = NULL;
	h_inode = NULL;
	bindex = au_br_index(sb, br->br_id);
	if (0 <= bindex) {
		hdir = au_hi(d_inode(sb->s_root), bindex);
		au_hn_inode_lock_nested(hdir, AuLsc_I_PARENT);
	} else {
		h_dentry = au_br_dentry(br);
		h_inode = d_inode(h_dentry);
		inode_lock_nested(h_inode, AuLsc_I_PARENT);
	}
	if (!wbr)
		err = au_wh_init(br, sb);
	else {
		wbr_wh_write_lock(wbr);
		err = au_wh_init(br, sb);
		wbr_wh_write_unlock(wbr);
	}
	if (hdir)
		au_hn_inode_unlock(hdir);
	else
		inode_unlock(h_inode);
	vfsub_mnt_drop_write(au_br_mnt(br));
	br->br_perm = old_perm;

	if (!err && wbr && !au_br_writable(new_perm)) {
		au_kfree_rcu(wbr);
		br->br_wbr = NULL;
	}

out:
	return err;
}

static int au_wbr_init(struct au_branch *br, struct super_block *sb,
		       int perm)
{
	int err;
	struct kstatfs kst;
	struct au_wbr *wbr;

	wbr = br->br_wbr;
	au_rw_init(&wbr->wbr_wh_rwsem);
	atomic_set(&wbr->wbr_wh_running, 0);

	/*
	 * a limit for rmdir/rename a dir
	 * cf. AUFS_MAX_NAMELEN in include/uapi/linux/aufs_type.h
	 */
	err = vfs_statfs(&br->br_path, &kst);
	if (unlikely(err))
		goto out;
	err = -EINVAL;
	if (kst.f_namelen >= NAME_MAX)
		err = au_br_init_wh(sb, br, perm);
	else
		pr_err("%pd(%s), unsupported namelen %ld\n",
		       au_br_dentry(br),
		       au_sbtype(au_br_dentry(br)->d_sb), kst.f_namelen);

out:
	return err;
}

/* initialize a new branch */
static int au_br_init(struct au_branch *br, struct super_block *sb,
		      struct au_opt_add *add)
{
	int err;
	struct au_branch *brbase;
	struct file *xf;
	struct inode *h_inode;

	err = 0;
	br->br_perm = add->perm;
	br->br_path = add->path; /* set first, path_get() later */
	spin_lock_init(&br->br_dykey_lock);
	au_lcnt_init(&br->br_nfiles, /*release*/NULL);
	au_lcnt_init(&br->br_count, /*release*/NULL);
	br->br_id = au_new_br_id(sb);
	AuDebugOn(br->br_id < 0);

	/* always, regardless the given option */
	err = au_dr_br_init(sb, br, &add->path);
	if (unlikely(err))
		goto out_err;

	if (au_br_writable(add->perm)) {
		err = au_wbr_init(br, sb, add->perm);
		if (unlikely(err))
			goto out_err;
	}

	if (au_opt_test(au_mntflags(sb), XINO)) {
		brbase = au_sbr(sb, 0);
		xf = au_xino_file(brbase->br_xino, /*idx*/-1);
		AuDebugOn(!xf);
		h_inode = d_inode(add->path.dentry);
		err = au_xino_init_br(sb, br, h_inode->i_ino, &xf->f_path);
		if (unlikely(err)) {
			AuDebugOn(au_xino_file(br->br_xino, /*idx*/-1));
			goto out_err;
		}
	}

	sysaufs_br_init(br);
	path_get(&br->br_path);
	goto out; /* success */

out_err:
	memset(&br->br_path, 0, sizeof(br->br_path));
out:
	return err;
}

static void au_br_do_add_brp(struct au_sbinfo *sbinfo, aufs_bindex_t bindex,
			     struct au_branch *br, aufs_bindex_t bbot,
			     aufs_bindex_t amount)
{
	struct au_branch **brp;

	AuRwMustWriteLock(&sbinfo->si_rwsem);

	brp = sbinfo->si_branch + bindex;
	memmove(brp + 1, brp, sizeof(*brp) * amount);
	*brp = br;
	sbinfo->si_bbot++;
	if (unlikely(bbot < 0))
		sbinfo->si_bbot = 0;
}

static void au_br_do_add_hdp(struct au_dinfo *dinfo, aufs_bindex_t bindex,
			     aufs_bindex_t bbot, aufs_bindex_t amount)
{
	struct au_hdentry *hdp;

	AuRwMustWriteLock(&dinfo->di_rwsem);

	hdp = au_hdentry(dinfo, bindex);
	memmove(hdp + 1, hdp, sizeof(*hdp) * amount);
	au_h_dentry_init(hdp);
	dinfo->di_bbot++;
	if (unlikely(bbot < 0))
		dinfo->di_btop = 0;
}

static void au_br_do_add_hip(struct au_iinfo *iinfo, aufs_bindex_t bindex,
			     aufs_bindex_t bbot, aufs_bindex_t amount)
{
	struct au_hinode *hip;

	AuRwMustWriteLock(&iinfo->ii_rwsem);

	hip = au_hinode(iinfo, bindex);
	memmove(hip + 1, hip, sizeof(*hip) * amount);
	au_hinode_init(hip);
	iinfo->ii_bbot++;
	if (unlikely(bbot < 0))
		iinfo->ii_btop = 0;
}

static void au_br_do_add(struct super_block *sb, struct au_branch *br,
			 aufs_bindex_t bindex)
{
	struct dentry *root, *h_dentry;
	struct inode *root_inode, *h_inode;
	aufs_bindex_t bbot, amount;

	root = sb->s_root;
	root_inode = d_inode(root);
	bbot = au_sbbot(sb);
	amount = bbot + 1 - bindex;
	h_dentry = au_br_dentry(br);
	au_sbilist_lock();
	au_br_do_add_brp(au_sbi(sb), bindex, br, bbot, amount);
	au_br_do_add_hdp(au_di(root), bindex, bbot, amount);
	au_br_do_add_hip(au_ii(root_inode), bindex, bbot, amount);
	au_set_h_dptr(root, bindex, dget(h_dentry));
	h_inode = d_inode(h_dentry);
	au_set_h_iptr(root_inode, bindex, au_igrab(h_inode), /*flags*/0);
	au_sbilist_unlock();
}

int au_br_add(struct super_block *sb, struct au_opt_add *add, int remount)
{
	int err;
	aufs_bindex_t bbot, add_bindex;
	struct dentry *root, *h_dentry;
	struct inode *root_inode;
	struct au_branch *add_branch;

	root = sb->s_root;
	root_inode = d_inode(root);
	IMustLock(root_inode);
	IiMustWriteLock(root_inode);
	err = test_add(sb, add, remount);
	if (unlikely(err < 0))
		goto out;
	if (err) {
		err = 0;
		goto out; /* success */
	}

	bbot = au_sbbot(sb);
	add_branch = au_br_alloc(sb, bbot + 2, add->perm);
	err = PTR_ERR(add_branch);
	if (IS_ERR(add_branch))
		goto out;

	err = au_br_init(add_branch, sb, add);
	if (unlikely(err)) {
		au_br_do_free(add_branch);
		goto out;
	}

	add_bindex = add->bindex;
	sysaufs_brs_del(sb, add_bindex);	/* remove successors */
	au_br_do_add(sb, add_branch, add_bindex);
	sysaufs_brs_add(sb, add_bindex);	/* append successors */
	dbgaufs_brs_add(sb, add_bindex, /*topdown*/0);	/* rename successors */

	h_dentry = add->path.dentry;
	if (!add_bindex) {
		au_cpup_attr_all(root_inode, /*force*/1);
		sb->s_maxbytes = h_dentry->d_sb->s_maxbytes;
	} else
		au_add_nlink(root_inode, d_inode(h_dentry));

out:
	return err;
}

/* ---------------------------------------------------------------------- */

static unsigned long long au_farray_cb(struct super_block *sb, void *a,
				       unsigned long long max __maybe_unused,
				       void *arg)
{
	unsigned long long n;
	struct file **p, *f;
	struct hlist_bl_head *files;
	struct hlist_bl_node *pos;
	struct au_finfo *finfo;

	n = 0;
	p = a;
	files = &au_sbi(sb)->si_files;
	hlist_bl_lock(files);
	hlist_bl_for_each_entry(finfo, pos, files, fi_hlist) {
		f = finfo->fi_file;
		if (file_count(f)
		    && !special_file(file_inode(f)->i_mode)) {
			get_file(f);
			*p++ = f;
			n++;
			AuDebugOn(n > max);
		}
	}
	hlist_bl_unlock(files);

	return n;
}

static struct file **au_farray_alloc(struct super_block *sb,
				     unsigned long long *max)
{
	struct au_sbinfo *sbi;

	sbi = au_sbi(sb);
	*max = au_lcnt_read(&sbi->si_nfiles, /*do_rev*/1);
	return au_array_alloc(max, au_farray_cb, sb, /*arg*/NULL);
}

static void au_farray_free(struct file **a, unsigned long long max)
{
	unsigned long long ull;

	for (ull = 0; ull < max; ull++)
		if (a[ull])
			fput(a[ull]);
	kvfree(a);
}

/* ---------------------------------------------------------------------- */

/*
 * delete a branch
 */

/* to show the line number, do not make it inlined function */
#define AuVerbose(do_info, fmt, ...) do { \
	if (do_info) \
		pr_info(fmt, ##__VA_ARGS__); \
} while (0)

static int au_test_ibusy(struct inode *inode, aufs_bindex_t btop,
			 aufs_bindex_t bbot)
{
	return (inode && !S_ISDIR(inode->i_mode)) || btop == bbot;
}

static int au_test_dbusy(struct dentry *dentry, aufs_bindex_t btop,
			 aufs_bindex_t bbot)
{
	return au_test_ibusy(d_inode(dentry), btop, bbot);
}

/*
 * test if the branch is deletable or not.
 */
static int test_dentry_busy(struct dentry *root, aufs_bindex_t bindex,
			    unsigned int sigen, const unsigned int verbose)
{
	int err, i, j, ndentry;
	aufs_bindex_t btop, bbot;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;
	struct dentry *d;

	err = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(err))
		goto out;
	err = au_dcsub_pages(&dpages, root, NULL, NULL);
	if (unlikely(err))
		goto out_dpages;

	for (i = 0; !err && i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		ndentry = dpage->ndentry;
		for (j = 0; !err && j < ndentry; j++) {
			d = dpage->dentries[j];
			AuDebugOn(au_dcount(d) <= 0);
			if (!au_digen_test(d, sigen)) {
				di_read_lock_child(d, AuLock_IR);
				if (unlikely(au_dbrange_test(d))) {
					di_read_unlock(d, AuLock_IR);
					continue;
				}
			} else {
				di_write_lock_child(d);
				if (unlikely(au_dbrange_test(d))) {
					di_write_unlock(d);
					continue;
				}
				err = au_reval_dpath(d, sigen);
				if (!err)
					di_downgrade_lock(d, AuLock_IR);
				else {
					di_write_unlock(d);
					break;
				}
			}

			/* AuDbgDentry(d); */
			btop = au_dbtop(d);
			bbot = au_dbbot(d);
			if (btop <= bindex
			    && bindex <= bbot
			    && au_h_dptr(d, bindex)
			    && au_test_dbusy(d, btop, bbot)) {
				err = -EBUSY;
				AuVerbose(verbose, "busy %pd\n", d);
				AuDbgDentry(d);
			}
			di_read_unlock(d, AuLock_IR);
		}
	}

out_dpages:
	au_dpages_free(&dpages);
out:
	return err;
}

static int test_inode_busy(struct super_block *sb, aufs_bindex_t bindex,
			   unsigned int sigen, const unsigned int verbose)
{
	int err;
	unsigned long long max, ull;
	struct inode *i, **array;
	aufs_bindex_t btop, bbot;

	array = au_iarray_alloc(sb, &max);
	err = PTR_ERR(array);
	if (IS_ERR(array))
		goto out;

	err = 0;
	AuDbg("b%d\n", bindex);
	for (ull = 0; !err && ull < max; ull++) {
		i = array[ull];
		if (unlikely(!i))
			break;
		if (i->i_ino == AUFS_ROOT_INO)
			continue;

		/* AuDbgInode(i); */
		if (au_iigen(i, NULL) == sigen)
			ii_read_lock_child(i);
		else {
			ii_write_lock_child(i);
			err = au_refresh_hinode_self(i);
			au_iigen_dec(i);
			if (!err)
				ii_downgrade_lock(i);
			else {
				ii_write_unlock(i);
				break;
			}
		}

		btop = au_ibtop(i);
		bbot = au_ibbot(i);
		if (btop <= bindex
		    && bindex <= bbot
		    && au_h_iptr(i, bindex)
		    && au_test_ibusy(i, btop, bbot)) {
			err = -EBUSY;
			AuVerbose(verbose, "busy i%lu\n", i->i_ino);
			AuDbgInode(i);
		}
		ii_read_unlock(i);
	}
	au_iarray_free(array, max);

out:
	return err;
}

static int test_children_busy(struct dentry *root, aufs_bindex_t bindex,
			      const unsigned int verbose)
{
	int err;
	unsigned int sigen;

	sigen = au_sigen(root->d_sb);
	DiMustNoWaiters(root);
	IiMustNoWaiters(d_inode(root));
	di_write_unlock(root);
	err = test_dentry_busy(root, bindex, sigen, verbose);
	if (!err)
		err = test_inode_busy(root->d_sb, bindex, sigen, verbose);
	di_write_lock_child(root); /* aufs_write_lock() calls ..._child() */

	return err;
}

static int test_dir_busy(struct file *file, aufs_bindex_t br_id,
			 struct file **to_free, int *idx)
{
	int err;
	unsigned char matched, root;
	aufs_bindex_t bindex, bbot;
	struct au_fidir *fidir;
	struct au_hfile *hfile;

	err = 0;
	root = IS_ROOT(file->f_path.dentry);
	if (root) {
		get_file(file);
		to_free[*idx] = file;
		(*idx)++;
		goto out;
	}

	matched = 0;
	fidir = au_fi(file)->fi_hdir;
	AuDebugOn(!fidir);
	bbot = au_fbbot_dir(file);
	for (bindex = au_fbtop(file); bindex <= bbot; bindex++) {
		hfile = fidir->fd_hfile + bindex;
		if (!hfile->hf_file)
			continue;

		if (hfile->hf_br->br_id == br_id) {
			matched = 1;
			break;
		}
	}
	if (matched)
		err = -EBUSY;

out:
	return err;
}

static int test_file_busy(struct super_block *sb, aufs_bindex_t br_id,
			  struct file **to_free, int opened)
{
	int err, idx;
	unsigned long long ull, max;
	aufs_bindex_t btop;
	struct file *file, **array;
	struct dentry *root;
	struct au_hfile *hfile;

	array = au_farray_alloc(sb, &max);
	err = PTR_ERR(array);
	if (IS_ERR(array))
		goto out;

	err = 0;
	idx = 0;
	root = sb->s_root;
	di_write_unlock(root);
	for (ull = 0; ull < max; ull++) {
		file = array[ull];
		if (unlikely(!file))
			break;

		/* AuDbg("%pD\n", file); */
		fi_read_lock(file);
		btop = au_fbtop(file);
		if (!d_is_dir(file->f_path.dentry)) {
			hfile = &au_fi(file)->fi_htop;
			if (hfile->hf_br->br_id == br_id)
				err = -EBUSY;
		} else
			err = test_dir_busy(file, br_id, to_free, &idx);
		fi_read_unlock(file);
		if (unlikely(err))
			break;
	}
	di_write_lock_child(root);
	au_farray_free(array, max);
	AuDebugOn(idx > opened);

out:
	return err;
}

static void br_del_file(struct file **to_free, unsigned long long opened,
			aufs_bindex_t br_id)
{
	unsigned long long ull;
	aufs_bindex_t bindex, btop, bbot, bfound;
	struct file *file;
	struct au_fidir *fidir;
	struct au_hfile *hfile;

	for (ull = 0; ull < opened; ull++) {
		file = to_free[ull];
		if (unlikely(!file))
			break;

		/* AuDbg("%pD\n", file); */
		AuDebugOn(!d_is_dir(file->f_path.dentry));
		bfound = -1;
		fidir = au_fi(file)->fi_hdir;
		AuDebugOn(!fidir);
		fi_write_lock(file);
		btop = au_fbtop(file);
		bbot = au_fbbot_dir(file);
		for (bindex = btop; bindex <= bbot; bindex++) {
			hfile = fidir->fd_hfile + bindex;
			if (!hfile->hf_file)
				continue;

			if (hfile->hf_br->br_id == br_id) {
				bfound = bindex;
				break;
			}
		}
		AuDebugOn(bfound < 0);
		au_set_h_fptr(file, bfound, NULL);
		if (bfound == btop) {
			for (btop++; btop <= bbot; btop++)
				if (au_hf_dir(file, btop)) {
					au_set_fbtop(file, btop);
					break;
				}
		}
		fi_write_unlock(file);
	}
}

static void au_br_do_del_brp(struct au_sbinfo *sbinfo,
			     const aufs_bindex_t bindex,
			     const aufs_bindex_t bbot)
{
	struct au_branch **brp, **p;

	AuRwMustWriteLock(&sbinfo->si_rwsem);

	brp = sbinfo->si_branch + bindex;
	if (bindex < bbot)
		memmove(brp, brp + 1, sizeof(*brp) * (bbot - bindex));
	sbinfo->si_branch[0 + bbot] = NULL;
	sbinfo->si_bbot--;

	p = au_krealloc(sbinfo->si_branch, sizeof(*p) * bbot, AuGFP_SBILIST,
			/*may_shrink*/1);
	if (p)
		sbinfo->si_branch = p;
	/* harmless error */
}

static void au_br_do_del_hdp(struct au_dinfo *dinfo, const aufs_bindex_t bindex,
			     const aufs_bindex_t bbot)
{
	struct au_hdentry *hdp, *p;

	AuRwMustWriteLock(&dinfo->di_rwsem);

	hdp = au_hdentry(dinfo, bindex);
	if (bindex < bbot)
		memmove(hdp, hdp + 1, sizeof(*hdp) * (bbot - bindex));
	/* au_h_dentry_init(au_hdentry(dinfo, bbot); */
	dinfo->di_bbot--;

	p = au_krealloc(dinfo->di_hdentry, sizeof(*p) * bbot, AuGFP_SBILIST,
			/*may_shrink*/1);
	if (p)
		dinfo->di_hdentry = p;
	/* harmless error */
}

static void au_br_do_del_hip(struct au_iinfo *iinfo, const aufs_bindex_t bindex,
			     const aufs_bindex_t bbot)
{
	struct au_hinode *hip, *p;

	AuRwMustWriteLock(&iinfo->ii_rwsem);

	hip = au_hinode(iinfo, bindex);
	if (bindex < bbot)
		memmove(hip, hip + 1, sizeof(*hip) * (bbot - bindex));
	/* au_hinode_init(au_hinode(iinfo, bbot)); */
	iinfo->ii_bbot--;

	p = au_krealloc(iinfo->ii_hinode, sizeof(*p) * bbot, AuGFP_SBILIST,
			/*may_shrink*/1);
	if (p)
		iinfo->ii_hinode = p;
	/* harmless error */
}

static void au_br_do_del(struct super_block *sb, aufs_bindex_t bindex,
			 struct au_branch *br)
{
	aufs_bindex_t bbot;
	struct au_sbinfo *sbinfo;
	struct dentry *root, *h_root;
	struct inode *inode, *h_inode;
	struct au_hinode *hinode;

	SiMustWriteLock(sb);

	root = sb->s_root;
	inode = d_inode(root);
	sbinfo = au_sbi(sb);
	bbot = sbinfo->si_bbot;

	h_root = au_h_dptr(root, bindex);
	hinode = au_hi(inode, bindex);
	h_inode = au_igrab(hinode->hi_inode);
	au_hiput(hinode);

	au_sbilist_lock();
	au_br_do_del_brp(sbinfo, bindex, bbot);
	au_br_do_del_hdp(au_di(root), bindex, bbot);
	au_br_do_del_hip(au_ii(inode), bindex, bbot);
	au_sbilist_unlock();

	/* ignore an error */
	au_dr_br_fin(sb, br); /* always, regardless the mount option */

	dput(h_root);
	iput(h_inode);
	au_br_do_free(br);
}

static unsigned long long empty_cb(struct super_block *sb, void *array,
				   unsigned long long max, void *arg)
{
	return max;
}

int au_br_del(struct super_block *sb, struct au_opt_del *del, int remount)
{
	int err, rerr, i;
	unsigned long long opened;
	unsigned int mnt_flags;
	aufs_bindex_t bindex, bbot, br_id;
	unsigned char do_wh, verbose;
	struct au_branch *br;
	struct au_wbr *wbr;
	struct dentry *root;
	struct file **to_free;

	err = 0;
	opened = 0;
	to_free = NULL;
	root = sb->s_root;
	bindex = au_find_dbindex(root, del->h_path.dentry);
	if (bindex < 0) {
		if (remount)
			goto out; /* success */
		err = -ENOENT;
		pr_err("%s no such branch\n", del->pathname);
		goto out;
	}
	AuDbg("bindex b%d\n", bindex);

	err = -EBUSY;
	mnt_flags = au_mntflags(sb);
	verbose = !!au_opt_test(mnt_flags, VERBOSE);
	bbot = au_sbbot(sb);
	if (unlikely(!bbot)) {
		AuVerbose(verbose, "no more branches left\n");
		goto out;
	}

	br = au_sbr(sb, bindex);
	AuDebugOn(!path_equal(&br->br_path, &del->h_path));
	if (unlikely(au_lcnt_read(&br->br_count, /*do_rev*/1))) {
		AuVerbose(verbose, "br %pd2 is busy now\n", del->h_path.dentry);
		goto out;
	}

	br_id = br->br_id;
	opened = au_lcnt_read(&br->br_nfiles, /*do_rev*/1);
	if (unlikely(opened)) {
		to_free = au_array_alloc(&opened, empty_cb, sb, NULL);
		err = PTR_ERR(to_free);
		if (IS_ERR(to_free))
			goto out;

		err = test_file_busy(sb, br_id, to_free, opened);
		if (unlikely(err)) {
			AuVerbose(verbose, "%llu file(s) opened\n", opened);
			goto out;
		}
	}

	wbr = br->br_wbr;
	do_wh = wbr && (wbr->wbr_whbase || wbr->wbr_plink || wbr->wbr_orph);
	if (do_wh) {
		/* instead of WbrWhMustWriteLock(wbr) */
		SiMustWriteLock(sb);
		for (i = 0; i < AuBrWh_Last; i++) {
			dput(wbr->wbr_wh[i]);
			wbr->wbr_wh[i] = NULL;
		}
	}

	err = test_children_busy(root, bindex, verbose);
	if (unlikely(err)) {
		if (do_wh)
			goto out_wh;
		goto out;
	}

	err = 0;
	if (to_free) {
		/*
		 * now we confirmed the branch is deletable.
		 * let's free the remaining opened dirs on the branch.
		 */
		di_write_unlock(root);
		br_del_file(to_free, opened, br_id);
		di_write_lock_child(root);
	}

	sysaufs_brs_del(sb, bindex);	/* remove successors */
	dbgaufs_xino_del(br);		/* remove one */
	au_br_do_del(sb, bindex, br);
	sysaufs_brs_add(sb, bindex);	/* append successors */
	dbgaufs_brs_add(sb, bindex, /*topdown*/1);	/* rename successors */

	if (!bindex) {
		au_cpup_attr_all(d_inode(root), /*force*/1);
		sb->s_maxbytes = au_sbr_sb(sb, 0)->s_maxbytes;
	} else
		au_sub_nlink(d_inode(root), d_inode(del->h_path.dentry));
	if (au_opt_test(mnt_flags, PLINK))
		au_plink_half_refresh(sb, br_id);

	goto out; /* success */

out_wh:
	/* revert */
	rerr = au_br_init_wh(sb, br, br->br_perm);
	if (rerr)
		pr_warn("failed re-creating base whiteout, %s. (%d)\n",
			del->pathname, rerr);
out:
	if (to_free)
		au_farray_free(to_free, opened);
	return err;
}

/* ---------------------------------------------------------------------- */

static int au_ibusy(struct super_block *sb, struct aufs_ibusy __user *arg)
{
	int err;
	aufs_bindex_t btop, bbot;
	struct aufs_ibusy ibusy;
	struct inode *inode, *h_inode;

	err = -EPERM;
	if (unlikely(!capable(CAP_SYS_ADMIN)))
		goto out;

	err = copy_from_user(&ibusy, arg, sizeof(ibusy));
	if (!err)
		/* VERIFY_WRITE */
		err = !access_ok(&arg->h_ino, sizeof(arg->h_ino));
	if (unlikely(err)) {
		err = -EFAULT;
		AuTraceErr(err);
		goto out;
	}

	err = -EINVAL;
	si_read_lock(sb, AuLock_FLUSH);
	if (unlikely(ibusy.bindex < 0 || ibusy.bindex > au_sbbot(sb)))
		goto out_unlock;

	err = 0;
	ibusy.h_ino = 0; /* invalid */
	inode = ilookup(sb, ibusy.ino);
	if (!inode
	    || inode->i_ino == AUFS_ROOT_INO
	    || au_is_bad_inode(inode))
		goto out_unlock;

	ii_read_lock_child(inode);
	btop = au_ibtop(inode);
	bbot = au_ibbot(inode);
	if (btop <= ibusy.bindex && ibusy.bindex <= bbot) {
		h_inode = au_h_iptr(inode, ibusy.bindex);
		if (h_inode && au_test_ibusy(inode, btop, bbot))
			ibusy.h_ino = h_inode->i_ino;
	}
	ii_read_unlock(inode);
	iput(inode);

out_unlock:
	si_read_unlock(sb);
	if (!err) {
		err = __put_user(ibusy.h_ino, &arg->h_ino);
		if (unlikely(err)) {
			err = -EFAULT;
			AuTraceErr(err);
		}
	}
out:
	return err;
}

long au_ibusy_ioctl(struct file *file, unsigned long arg)
{
	return au_ibusy(file->f_path.dentry->d_sb, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
long au_ibusy_compat_ioctl(struct file *file, unsigned long arg)
{
	return au_ibusy(file->f_path.dentry->d_sb, compat_ptr(arg));
}
#endif

/* ---------------------------------------------------------------------- */

/*
 * change a branch permission
 */

static void au_warn_ima(void)
{
#ifdef CONFIG_IMA
	/* since it doesn't support mark_files_ro() */
	AuWarn1("RW -> RO makes IMA to produce wrong message\n");
#endif
}

static int do_need_sigen_inc(int a, int b)
{
	return au_br_whable(a) && !au_br_whable(b);
}

static int need_sigen_inc(int old, int new)
{
	return do_need_sigen_inc(old, new)
		|| do_need_sigen_inc(new, old);
}

static int au_br_mod_files_ro(struct super_block *sb, aufs_bindex_t bindex)
{
	int err, do_warn;
	unsigned int mnt_flags;
	unsigned long long ull, max;
	aufs_bindex_t br_id;
	unsigned char verbose, writer;
	struct file *file, *hf, **array;
	struct au_hfile *hfile;
	struct inode *h_inode;

	mnt_flags = au_mntflags(sb);
	verbose = !!au_opt_test(mnt_flags, VERBOSE);

	array = au_farray_alloc(sb, &max);
	err = PTR_ERR(array);
	if (IS_ERR(array))
		goto out;

	do_warn = 0;
	br_id = au_sbr_id(sb, bindex);
	for (ull = 0; ull < max; ull++) {
		file = array[ull];
		if (unlikely(!file))
			break;

		/* AuDbg("%pD\n", file); */
		fi_read_lock(file);
		if (unlikely(au_test_mmapped(file))) {
			err = -EBUSY;
			AuVerbose(verbose, "mmapped %pD\n", file);
			AuDbgFile(file);
			FiMustNoWaiters(file);
			fi_read_unlock(file);
			goto out_array;
		}

		hfile = &au_fi(file)->fi_htop;
		hf = hfile->hf_file;
		if (!d_is_reg(file->f_path.dentry)
		    || !(file->f_mode & FMODE_WRITE)
		    || hfile->hf_br->br_id != br_id
		    || !(hf->f_mode & FMODE_WRITE))
			array[ull] = NULL;
		else {
			do_warn = 1;
			get_file(file);
		}

		FiMustNoWaiters(file);
		fi_read_unlock(file);
		fput(file);
	}

	err = 0;
	if (do_warn)
		au_warn_ima();

	for (ull = 0; ull < max; ull++) {
		file = array[ull];
		if (!file)
			continue;

		/* todo: already flushed? */
		/*
		 * fs/super.c:mark_files_ro() is gone, but aufs keeps its
		 * approach which resets f_mode and calls mnt_drop_write() and
		 * file_release_write() for each file, because the branch
		 * attribute in aufs world is totally different from the native
		 * fs rw/ro mode.
		 */
		/* fi_read_lock(file); */
		hfile = &au_fi(file)->fi_htop;
		hf = hfile->hf_file;
		/* fi_read_unlock(file); */
		spin_lock(&hf->f_lock);
		writer = !!(hf->f_mode & FMODE_WRITER);
		hf->f_mode &= ~(FMODE_WRITE | FMODE_WRITER);
		spin_unlock(&hf->f_lock);
		if (writer) {
			h_inode = file_inode(hf);
			if (hf->f_mode & FMODE_READ)
				i_readcount_inc(h_inode);
			put_write_access(h_inode);
			__mnt_drop_write(hf->f_path.mnt);
		}
	}

out_array:
	au_farray_free(array, max);
out:
	AuTraceErr(err);
	return err;
}

int au_br_mod(struct super_block *sb, struct au_opt_mod *mod, int remount,
	      int *do_refresh)
{
	int err, rerr;
	aufs_bindex_t bindex;
	struct dentry *root;
	struct au_branch *br;
	struct au_br_fhsm *bf;

	root = sb->s_root;
	bindex = au_find_dbindex(root, mod->h_root);
	if (bindex < 0) {
		if (remount)
			return 0; /* success */
		err = -ENOENT;
		pr_err("%s no such branch\n", mod->path);
		goto out;
	}
	AuDbg("bindex b%d\n", bindex);

	err = test_br(d_inode(mod->h_root), mod->perm, mod->path);
	if (unlikely(err))
		goto out;

	br = au_sbr(sb, bindex);
	AuDebugOn(mod->h_root != au_br_dentry(br));
	if (br->br_perm == mod->perm)
		return 0; /* success */

	/* pre-allocate for non-fhsm --> fhsm */
	bf = NULL;
	if (!au_br_fhsm(br->br_perm) && au_br_fhsm(mod->perm)) {
		err = au_fhsm_br_alloc(br);
		if (unlikely(err))
			goto out;
		bf = br->br_fhsm;
		br->br_fhsm = NULL;
	}

	if (au_br_writable(br->br_perm)) {
		/* remove whiteout base */
		err = au_br_init_wh(sb, br, mod->perm);
		if (unlikely(err))
			goto out_bf;

		if (!au_br_writable(mod->perm)) {
			/* rw --> ro, file might be mmapped */
			DiMustNoWaiters(root);
			IiMustNoWaiters(d_inode(root));
			di_write_unlock(root);
			err = au_br_mod_files_ro(sb, bindex);
			/* aufs_write_lock() calls ..._child() */
			di_write_lock_child(root);

			if (unlikely(err)) {
				rerr = -ENOMEM;
				br->br_wbr = kzalloc(sizeof(*br->br_wbr),
						     GFP_NOFS);
				if (br->br_wbr)
					rerr = au_wbr_init(br, sb, br->br_perm);
				if (unlikely(rerr)) {
					AuIOErr("nested error %d (%d)\n",
						rerr, err);
					br->br_perm = mod->perm;
				}
			}
		}
	} else if (au_br_writable(mod->perm)) {
		/* ro --> rw */
		err = -ENOMEM;
		br->br_wbr = kzalloc(sizeof(*br->br_wbr), GFP_NOFS);
		if (br->br_wbr) {
			err = au_wbr_init(br, sb, mod->perm);
			if (unlikely(err)) {
				au_kfree_rcu(br->br_wbr);
				br->br_wbr = NULL;
			}
		}
	}
	if (unlikely(err))
		goto out_bf;

	if (au_br_fhsm(br->br_perm)) {
		if (!au_br_fhsm(mod->perm)) {
			/* fhsm --> non-fhsm */
			au_br_fhsm_fin(br->br_fhsm);
			au_kfree_rcu(br->br_fhsm);
			br->br_fhsm = NULL;
		}
	} else if (au_br_fhsm(mod->perm))
		/* non-fhsm --> fhsm */
		br->br_fhsm = bf;

	*do_refresh |= need_sigen_inc(br->br_perm, mod->perm);
	br->br_perm = mod->perm;
	goto out; /* success */

out_bf:
	au_kfree_try_rcu(bf);
out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

int au_br_stfs(struct au_branch *br, struct aufs_stfs *stfs)
{
	int err;
	struct kstatfs kstfs;

	err = vfs_statfs(&br->br_path, &kstfs);
	if (!err) {
		stfs->f_blocks = kstfs.f_blocks;
		stfs->f_bavail = kstfs.f_bavail;
		stfs->f_files = kstfs.f_files;
		stfs->f_ffree = kstfs.f_ffree;
	}

	return err;
}
