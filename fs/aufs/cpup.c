// SPDX-License-Identifier: GPL-2.0
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
 * copy-up functions, see wbr_policy.c for copy-down
 */

#include <linux/fs_stack.h>
#include <linux/mm.h>
#include <linux/task_work.h>
#include "aufs.h"

void au_cpup_attr_flags(struct inode *dst, unsigned int iflags)
{
	const unsigned int mask = S_DEAD | S_SWAPFILE | S_PRIVATE
		| S_NOATIME | S_NOCMTIME | S_AUTOMOUNT;

	BUILD_BUG_ON(sizeof(iflags) != sizeof(dst->i_flags));

	dst->i_flags |= iflags & ~mask;
	if (au_test_fs_notime(dst->i_sb))
		dst->i_flags |= S_NOATIME | S_NOCMTIME;
}

void au_cpup_attr_timesizes(struct inode *inode)
{
	struct inode *h_inode;

	h_inode = au_h_iptr(inode, au_ibtop(inode));
	fsstack_copy_attr_times(inode, h_inode);
	fsstack_copy_inode_size(inode, h_inode);
}

void au_cpup_attr_nlink(struct inode *inode, int force)
{
	struct inode *h_inode;
	struct super_block *sb;
	aufs_bindex_t bindex, bbot;

	sb = inode->i_sb;
	bindex = au_ibtop(inode);
	h_inode = au_h_iptr(inode, bindex);
	if (!force
	    && !S_ISDIR(h_inode->i_mode)
	    && au_opt_test(au_mntflags(sb), PLINK)
	    && au_plink_test(inode))
		return;

	/*
	 * 0 can happen in revalidating.
	 * h_inode->i_mutex may not be held here, but it is harmless since once
	 * i_nlink reaches 0, it will never become positive except O_TMPFILE
	 * case.
	 * todo: O_TMPFILE+linkat(AT_SYMLINK_FOLLOW) bypassing aufs may cause
	 *	 the incorrect link count.
	 */
	set_nlink(inode, h_inode->i_nlink);

	/*
	 * fewer nlink makes find(1) noisy, but larger nlink doesn't.
	 * it may includes whplink directory.
	 */
	if (S_ISDIR(h_inode->i_mode)) {
		bbot = au_ibbot(inode);
		for (bindex++; bindex <= bbot; bindex++) {
			h_inode = au_h_iptr(inode, bindex);
			if (h_inode)
				au_add_nlink(inode, h_inode);
		}
	}
}

void au_cpup_attr_changeable(struct inode *inode)
{
	struct inode *h_inode;

	h_inode = au_h_iptr(inode, au_ibtop(inode));
	inode->i_mode = h_inode->i_mode;
	inode->i_uid = h_inode->i_uid;
	inode->i_gid = h_inode->i_gid;
	au_cpup_attr_timesizes(inode);
	au_cpup_attr_flags(inode, h_inode->i_flags);
}

void au_cpup_igen(struct inode *inode, struct inode *h_inode)
{
	struct au_iinfo *iinfo = au_ii(inode);

	IiMustWriteLock(inode);

	iinfo->ii_higen = h_inode->i_generation;
	iinfo->ii_hsb1 = h_inode->i_sb;
}

void au_cpup_attr_all(struct inode *inode, int force)
{
	struct inode *h_inode;

	h_inode = au_h_iptr(inode, au_ibtop(inode));
	au_cpup_attr_changeable(inode);
	if (inode->i_nlink > 0)
		au_cpup_attr_nlink(inode, force);
	inode->i_rdev = h_inode->i_rdev;
	inode->i_blkbits = h_inode->i_blkbits;
	au_cpup_igen(inode, h_inode);
}

/* ---------------------------------------------------------------------- */

/* Note: dt_dentry and dt_h_dentry are not dget/dput-ed */

/* keep the timestamps of the parent dir when cpup */
void au_dtime_store(struct au_dtime *dt, struct dentry *dentry,
		    struct path *h_path)
{
	struct inode *h_inode;

	dt->dt_dentry = dentry;
	dt->dt_h_path = *h_path;
	h_inode = d_inode(h_path->dentry);
	dt->dt_atime = h_inode->i_atime;
	dt->dt_mtime = h_inode->i_mtime;
	/* smp_mb(); */
}

void au_dtime_revert(struct au_dtime *dt)
{
	struct iattr attr;
	int err;

	attr.ia_atime = dt->dt_atime;
	attr.ia_mtime = dt->dt_mtime;
	attr.ia_valid = ATTR_FORCE | ATTR_MTIME | ATTR_MTIME_SET
		| ATTR_ATIME | ATTR_ATIME_SET;

	/* no delegation since this is a directory */
	err = vfsub_notify_change(&dt->dt_h_path, &attr, /*delegated*/NULL);
	if (unlikely(err))
		pr_warn("restoring timestamps failed(%d). ignored\n", err);
}

/* ---------------------------------------------------------------------- */

/* internal use only */
struct au_cpup_reg_attr {
	int		valid;
	struct kstat	st;
	unsigned int	iflags; /* inode->i_flags */
};

static noinline_for_stack
int cpup_iattr(struct dentry *dst, aufs_bindex_t bindex, struct path *h_src,
	       struct au_cpup_reg_attr *h_src_attr)
{
	int err, sbits, icex;
	unsigned int mnt_flags;
	unsigned char verbose;
	struct iattr ia;
	struct path h_path;
	struct inode *h_isrc, *h_idst;
	struct kstat *h_st;
	struct au_branch *br;

	br = au_sbr(dst->d_sb, bindex);
	h_path.mnt = au_br_mnt(br);
	h_path.dentry = au_h_dptr(dst, bindex);
	h_idst = d_inode(h_path.dentry);
	h_isrc = d_inode(h_src->dentry);
	ia.ia_valid = ATTR_FORCE | ATTR_UID | ATTR_GID
		| ATTR_ATIME | ATTR_MTIME
		| ATTR_ATIME_SET | ATTR_MTIME_SET;
	if (h_src_attr && h_src_attr->valid) {
		h_st = &h_src_attr->st;
		ia.ia_uid = h_st->uid;
		ia.ia_gid = h_st->gid;
		ia.ia_atime = h_st->atime;
		ia.ia_mtime = h_st->mtime;
		if (h_idst->i_mode != h_st->mode
		    && !S_ISLNK(h_idst->i_mode)) {
			ia.ia_valid |= ATTR_MODE;
			ia.ia_mode = h_st->mode;
		}
		sbits = !!(h_st->mode & (S_ISUID | S_ISGID));
		au_cpup_attr_flags(h_idst, h_src_attr->iflags);
	} else {
		ia.ia_uid = h_isrc->i_uid;
		ia.ia_gid = h_isrc->i_gid;
		ia.ia_atime = h_isrc->i_atime;
		ia.ia_mtime = h_isrc->i_mtime;
		if (h_idst->i_mode != h_isrc->i_mode
		    && !S_ISLNK(h_idst->i_mode)) {
			ia.ia_valid |= ATTR_MODE;
			ia.ia_mode = h_isrc->i_mode;
		}
		sbits = !!(h_isrc->i_mode & (S_ISUID | S_ISGID));
		au_cpup_attr_flags(h_idst, h_isrc->i_flags);
	}
	/* no delegation since it is just created */
	err = vfsub_notify_change(&h_path, &ia, /*delegated*/NULL);

	/* is this nfs only? */
	if (!err && sbits && au_test_nfs(h_path.dentry->d_sb)) {
		ia.ia_valid = ATTR_FORCE | ATTR_MODE;
		ia.ia_mode = h_isrc->i_mode;
		err = vfsub_notify_change(&h_path, &ia, /*delegated*/NULL);
	}

	icex = br->br_perm & AuBrAttr_ICEX;
	if (!err) {
		mnt_flags = au_mntflags(dst->d_sb);
		verbose = !!au_opt_test(mnt_flags, VERBOSE);
		err = au_cpup_xattr(&h_path, h_src, icex, verbose);
	}

	return err;
}

/* ---------------------------------------------------------------------- */

static int au_do_copy_file(struct file *dst, struct file *src, loff_t len,
			   char *buf, unsigned long blksize)
{
	int err;
	size_t sz, rbytes, wbytes;
	unsigned char all_zero;
	char *p, *zp;
	struct inode *h_inode;
	/* reduce stack usage */
	struct iattr *ia;

	zp = page_address(ZERO_PAGE(0));
	if (unlikely(!zp))
		return -ENOMEM; /* possible? */

	err = 0;
	all_zero = 0;
	while (len) {
		AuDbg("len %lld\n", len);
		sz = blksize;
		if (len < blksize)
			sz = len;

		rbytes = 0;
		/* todo: signal_pending? */
		while (!rbytes || err == -EAGAIN || err == -EINTR) {
			rbytes = vfsub_read_k(src, buf, sz, &src->f_pos);
			err = rbytes;
		}
		if (unlikely(err < 0))
			break;

		all_zero = 0;
		if (len >= rbytes && rbytes == blksize)
			all_zero = !memcmp(buf, zp, rbytes);
		if (!all_zero) {
			wbytes = rbytes;
			p = buf;
			while (wbytes) {
				size_t b;

				b = vfsub_write_k(dst, p, wbytes, &dst->f_pos);
				err = b;
				/* todo: signal_pending? */
				if (unlikely(err == -EAGAIN || err == -EINTR))
					continue;
				if (unlikely(err < 0))
					break;
				wbytes -= b;
				p += b;
			}
			if (unlikely(err < 0))
				break;
		} else {
			loff_t res;

			AuLabel(hole);
			res = vfsub_llseek(dst, rbytes, SEEK_CUR);
			err = res;
			if (unlikely(res < 0))
				break;
		}
		len -= rbytes;
		err = 0;
	}

	/* the last block may be a hole */
	if (!err && all_zero) {
		AuLabel(last hole);

		err = 1;
		if (au_test_nfs(dst->f_path.dentry->d_sb)) {
			/* nfs requires this step to make last hole */
			/* is this only nfs? */
			do {
				/* todo: signal_pending? */
				err = vfsub_write_k(dst, "\0", 1, &dst->f_pos);
			} while (err == -EAGAIN || err == -EINTR);
			if (err == 1)
				dst->f_pos--;
		}

		if (err == 1) {
			ia = (void *)buf;
			ia->ia_size = dst->f_pos;
			ia->ia_valid = ATTR_SIZE | ATTR_FILE;
			ia->ia_file = dst;
			h_inode = file_inode(dst);
			inode_lock_nested(h_inode, AuLsc_I_CHILD2);
			/* no delegation since it is just created */
			err = vfsub_notify_change(&dst->f_path, ia,
						  /*delegated*/NULL);
			inode_unlock(h_inode);
		}
	}

	return err;
}

int au_copy_file(struct file *dst, struct file *src, loff_t len)
{
	int err;
	unsigned long blksize;
	unsigned char do_kfree;
	char *buf;
	struct super_block *h_sb;

	err = -ENOMEM;
	h_sb = file_inode(dst)->i_sb;
	blksize = h_sb->s_blocksize;
	if (!blksize || PAGE_SIZE < blksize)
		blksize = PAGE_SIZE;
	AuDbg("blksize %lu\n", blksize);
	do_kfree = (blksize != PAGE_SIZE && blksize >= sizeof(struct iattr *));
	if (do_kfree)
		buf = kmalloc(blksize, GFP_NOFS);
	else
		buf = (void *)__get_free_page(GFP_NOFS);
	if (unlikely(!buf))
		goto out;

	if (len > (1 << 22))
		AuDbg("copying a large file %lld\n", (long long)len);

	src->f_pos = 0;
	dst->f_pos = 0;
	err = au_do_copy_file(dst, src, len, buf, blksize);
	if (do_kfree) {
		AuDebugOn(!au_kfree_do_sz_test(blksize));
		au_kfree_do_rcu(buf);
	} else
		free_page((unsigned long)buf);

out:
	return err;
}

static int au_do_copy(struct file *dst, struct file *src, loff_t len)
{
	int err;
	struct super_block *h_src_sb;
	struct inode *h_src_inode;

	h_src_inode = file_inode(src);
	h_src_sb = h_src_inode->i_sb;

	/* XFS acquires inode_lock */
	if (!au_test_xfs(h_src_sb))
		err = au_copy_file(dst, src, len);
	else {
		inode_unlock_shared(h_src_inode);
		err = au_copy_file(dst, src, len);
		inode_lock_shared_nested(h_src_inode, AuLsc_I_CHILD);
	}

	return err;
}

static int au_clone_or_copy(struct file *dst, struct file *src, loff_t len)
{
	int err;
	loff_t lo;
	struct super_block *h_src_sb;
	struct inode *h_src_inode;

	h_src_inode = file_inode(src);
	h_src_sb = h_src_inode->i_sb;
	if (h_src_sb != file_inode(dst)->i_sb
	    || !dst->f_op->remap_file_range) {
		err = au_do_copy(dst, src, len);
		goto out;
	}

	if (!au_test_nfs(h_src_sb)) {
		inode_unlock_shared(h_src_inode);
		lo = vfsub_clone_file_range(src, dst, len);
		inode_lock_shared_nested(h_src_inode, AuLsc_I_CHILD);
	} else
		lo = vfsub_clone_file_range(src, dst, len);
	if (lo == len) {
		err = 0;
		goto out; /* success */
	} else if (lo >= 0)
		/* todo: possible? */
		/* paritially succeeded */
		AuDbg("lo %lld, len %lld. Retrying.\n", lo, len);
	else if (lo != -EOPNOTSUPP) {
		/* older XFS has a condition in cloning */
		err = lo;
		goto out;
	}

	/* the backend fs on NFS may not support cloning */
	err = au_do_copy(dst, src, len);

out:
	AuTraceErr(err);
	return err;
}

/*
 * to support a sparse file which is opened with O_APPEND,
 * we need to close the file.
 */
static int au_cp_regular(struct au_cp_generic *cpg)
{
	int err, i;
	enum { SRC, DST };
	struct {
		aufs_bindex_t bindex;
		unsigned int flags;
		struct dentry *dentry;
		int force_wr;
		struct file *file;
	} *f, file[] = {
		{
			.bindex = cpg->bsrc,
			.flags = O_RDONLY | O_NOATIME | O_LARGEFILE,
		},
		{
			.bindex = cpg->bdst,
			.flags = O_WRONLY | O_NOATIME | O_LARGEFILE,
			.force_wr = !!au_ftest_cpup(cpg->flags, RWDST),
		}
	};
	struct au_branch *br;
	struct super_block *sb, *h_src_sb;
	struct inode *h_src_inode;
	struct task_struct *tsk = current;

	/* bsrc branch can be ro/rw. */
	sb = cpg->dentry->d_sb;
	f = file;
	for (i = 0; i < 2; i++, f++) {
		f->dentry = au_h_dptr(cpg->dentry, f->bindex);
		f->file = au_h_open(cpg->dentry, f->bindex, f->flags,
				    /*file*/NULL, f->force_wr);
		if (IS_ERR(f->file)) {
			err = PTR_ERR(f->file);
			if (i == SRC)
				goto out;
			else
				goto out_src;
		}
	}

	/* try stopping to update while we copyup */
	h_src_inode = d_inode(file[SRC].dentry);
	h_src_sb = h_src_inode->i_sb;
	if (!au_test_nfs(h_src_sb))
		IMustLock(h_src_inode);
	err = au_clone_or_copy(file[DST].file, file[SRC].file, cpg->len);

	/* i wonder if we had O_NO_DELAY_FPUT flag */
	if (tsk->flags & PF_KTHREAD)
		__fput_sync(file[DST].file);
	else {
		/* it happened actually */
		fput(file[DST].file);
		/*
		 * too bad.
		 * we have to call both since we don't know which place the file
		 * was added to.
		 */
		task_work_run();
		flush_delayed_fput();
	}
	br = au_sbr(sb, file[DST].bindex);
	au_lcnt_dec(&br->br_nfiles);

out_src:
	fput(file[SRC].file);
	br = au_sbr(sb, file[SRC].bindex);
	au_lcnt_dec(&br->br_nfiles);
out:
	return err;
}

static int au_do_cpup_regular(struct au_cp_generic *cpg,
			      struct au_cpup_reg_attr *h_src_attr)
{
	int err, rerr;
	loff_t l;
	struct path h_path;
	struct inode *h_src_inode, *h_dst_inode;

	err = 0;
	h_src_inode = au_h_iptr(d_inode(cpg->dentry), cpg->bsrc);
	l = i_size_read(h_src_inode);
	if (cpg->len == -1 || l < cpg->len)
		cpg->len = l;
	if (cpg->len) {
		/* try stopping to update while we are referencing */
		inode_lock_shared_nested(h_src_inode, AuLsc_I_CHILD);
		au_pin_hdir_unlock(cpg->pin);

		h_path.dentry = au_h_dptr(cpg->dentry, cpg->bsrc);
		h_path.mnt = au_sbr_mnt(cpg->dentry->d_sb, cpg->bsrc);
		h_src_attr->iflags = h_src_inode->i_flags;
		if (!au_test_nfs(h_src_inode->i_sb))
			err = vfsub_getattr(&h_path, &h_src_attr->st);
		else {
			inode_unlock_shared(h_src_inode);
			err = vfsub_getattr(&h_path, &h_src_attr->st);
			inode_lock_shared_nested(h_src_inode, AuLsc_I_CHILD);
		}
		if (unlikely(err)) {
			inode_unlock_shared(h_src_inode);
			goto out;
		}
		h_src_attr->valid = 1;
		if (!au_test_nfs(h_src_inode->i_sb)) {
			err = au_cp_regular(cpg);
			inode_unlock_shared(h_src_inode);
		} else {
			inode_unlock_shared(h_src_inode);
			err = au_cp_regular(cpg);
		}
		rerr = au_pin_hdir_relock(cpg->pin);
		if (!err && rerr)
			err = rerr;
	}
	if (!err && (h_src_inode->i_state & I_LINKABLE)) {
		h_path.dentry = au_h_dptr(cpg->dentry, cpg->bdst);
		h_dst_inode = d_inode(h_path.dentry);
		spin_lock(&h_dst_inode->i_lock);
		h_dst_inode->i_state |= I_LINKABLE;
		spin_unlock(&h_dst_inode->i_lock);
	}

out:
	return err;
}

static int au_do_cpup_symlink(struct path *h_path, struct dentry *h_src,
			      struct inode *h_dir)
{
	int err;
	DEFINE_DELAYED_CALL(done);
	const char *sym;

	sym = vfs_get_link(h_src, &done);
	err = PTR_ERR(sym);
	if (IS_ERR(sym))
		goto out;

	err = vfsub_symlink(h_dir, h_path, sym);

out:
	do_delayed_call(&done);
	return err;
}

/*
 * regardless 'acl' option, reset all ACL.
 * All ACL will be copied up later from the original entry on the lower branch.
 */
static int au_reset_acl(struct path *h_path, umode_t mode)
{
	int err;
	struct dentry *h_dentry;
	struct inode *h_inode;
	struct user_namespace *h_userns;

	h_userns = mnt_user_ns(h_path->mnt);
	h_dentry = h_path->dentry;
	h_inode = d_inode(h_dentry);
	/* forget_all_cached_acls(h_inode)); */
	err = vfsub_removexattr(h_userns, h_dentry, XATTR_NAME_POSIX_ACL_ACCESS);
	AuTraceErr(err);
	if (err == -EOPNOTSUPP)
		err = 0;
	if (!err)
		err = vfsub_acl_chmod(h_userns, h_inode, mode);

	AuTraceErr(err);
	return err;
}

static int au_do_cpup_dir(struct au_cp_generic *cpg, struct dentry *dst_parent,
			  struct inode *h_dir, struct path *h_path)
{
	int err;
	struct inode *dir, *inode;
	struct user_namespace *h_userns;

	h_userns = mnt_user_ns(h_path->mnt);
	err = vfsub_removexattr(h_userns, h_path->dentry,
				XATTR_NAME_POSIX_ACL_DEFAULT);
	AuTraceErr(err);
	if (err == -EOPNOTSUPP)
		err = 0;
	if (unlikely(err))
		goto out;

	/*
	 * strange behaviour from the users view,
	 * particularly setattr case
	 */
	dir = d_inode(dst_parent);
	if (au_ibtop(dir) == cpg->bdst)
		au_cpup_attr_nlink(dir, /*force*/1);
	inode = d_inode(cpg->dentry);
	au_cpup_attr_nlink(inode, /*force*/1);

out:
	return err;
}

static noinline_for_stack
int cpup_entry(struct au_cp_generic *cpg, struct dentry *dst_parent,
	       struct au_cpup_reg_attr *h_src_attr)
{
	int err;
	umode_t mode;
	unsigned int mnt_flags;
	unsigned char isdir, isreg, force;
	const unsigned char do_dt = !!au_ftest_cpup(cpg->flags, DTIME);
	struct au_dtime dt;
	struct path h_path;
	struct dentry *h_src, *h_dst, *h_parent;
	struct inode *h_inode, *h_dir;
	struct super_block *sb;

	/* bsrc branch can be ro/rw. */
	h_src = au_h_dptr(cpg->dentry, cpg->bsrc);
	h_inode = d_inode(h_src);
	AuDebugOn(h_inode != au_h_iptr(d_inode(cpg->dentry), cpg->bsrc));

	/* try stopping to be referenced while we are creating */
	h_dst = au_h_dptr(cpg->dentry, cpg->bdst);
	if (au_ftest_cpup(cpg->flags, RENAME))
		AuDebugOn(strncmp(h_dst->d_name.name, AUFS_WH_PFX,
				  AUFS_WH_PFX_LEN));
	h_parent = h_dst->d_parent; /* dir inode is locked */
	h_dir = d_inode(h_parent);
	IMustLock(h_dir);
	AuDebugOn(h_parent != h_dst->d_parent);

	sb = cpg->dentry->d_sb;
	h_path.mnt = au_sbr_mnt(sb, cpg->bdst);
	if (do_dt) {
		h_path.dentry = h_parent;
		au_dtime_store(&dt, dst_parent, &h_path);
	}
	h_path.dentry = h_dst;

	isreg = 0;
	isdir = 0;
	mode = h_inode->i_mode;
	switch (mode & S_IFMT) {
	case S_IFREG:
		isreg = 1;
		err = vfsub_create(h_dir, &h_path, 0600, /*want_excl*/true);
		if (!err)
			err = au_do_cpup_regular(cpg, h_src_attr);
		break;
	case S_IFDIR:
		isdir = 1;
		err = vfsub_mkdir(h_dir, &h_path, mode);
		if (!err)
			err = au_do_cpup_dir(cpg, dst_parent, h_dir, &h_path);
		break;
	case S_IFLNK:
		err = au_do_cpup_symlink(&h_path, h_src, h_dir);
		break;
	case S_IFCHR:
	case S_IFBLK:
		AuDebugOn(!capable(CAP_MKNOD));
		fallthrough;
	case S_IFIFO:
	case S_IFSOCK:
		err = vfsub_mknod(h_dir, &h_path, mode, h_inode->i_rdev);
		break;
	default:
		AuIOErr("Unknown inode type 0%o\n", mode);
		err = -EIO;
	}
	if (!err)
		err = au_reset_acl(&h_path, mode);

	mnt_flags = au_mntflags(sb);
	if (!au_opt_test(mnt_flags, UDBA_NONE)
	    && !isdir
	    && au_opt_test(mnt_flags, XINO)
	    && (h_inode->i_nlink == 1
		|| (h_inode->i_state & I_LINKABLE))
	    /* todo: unnecessary? */
	    /* && d_inode(cpg->dentry)->i_nlink == 1 */
	    && cpg->bdst < cpg->bsrc
	    && !au_ftest_cpup(cpg->flags, KEEPLINO))
		au_xino_write(sb, cpg->bsrc, h_inode->i_ino, /*ino*/0);
		/* ignore this error */

	if (!err) {
		force = 0;
		if (isreg) {
			force = !!cpg->len;
			if (cpg->len == -1)
				force = !!i_size_read(h_inode);
		}
		au_fhsm_wrote(sb, cpg->bdst, force);
	}

	if (do_dt)
		au_dtime_revert(&dt);
	return err;
}

static int au_do_ren_after_cpup(struct au_cp_generic *cpg, struct path *h_path)
{
	int err;
	struct dentry *dentry, *h_dentry, *h_parent, *parent;
	struct path h_ppath;
	struct inode *h_dir;
	aufs_bindex_t bdst;

	dentry = cpg->dentry;
	bdst = cpg->bdst;
	h_ppath.mnt = au_sbr_mnt(dentry->d_sb, bdst);
	h_dentry = au_h_dptr(dentry, bdst);
	if (!au_ftest_cpup(cpg->flags, OVERWRITE)) {
		dget(h_dentry);
		au_set_h_dptr(dentry, bdst, NULL);
		err = au_lkup_neg(dentry, bdst, /*wh*/0);
		if (!err)
			h_path->dentry = dget(au_h_dptr(dentry, bdst));
		au_set_h_dptr(dentry, bdst, h_dentry);
	} else {
		err = 0;
		parent = dget_parent(dentry);
		h_ppath.dentry = au_h_dptr(parent, bdst);
		dput(parent);
		h_path->dentry = vfsub_lkup_one(&dentry->d_name, &h_ppath);
		if (IS_ERR(h_path->dentry))
			err = PTR_ERR(h_path->dentry);
	}
	if (unlikely(err))
		goto out;

	h_parent = h_dentry->d_parent; /* dir inode is locked */
	h_dir = d_inode(h_parent);
	IMustLock(h_dir);
	AuDbg("%pd %pd\n", h_dentry, h_path->dentry);
	/* no delegation since it is just created */
	err = vfsub_rename(h_dir, h_dentry, h_dir, h_path, /*delegated*/NULL,
			   /*flags*/0);
	dput(h_path->dentry);

out:
	return err;
}

/*
 * copyup the @dentry from @bsrc to @bdst.
 * the caller must set the both of lower dentries.
 * @len is for truncating when it is -1 copyup the entire file.
 * in link/rename cases, @dst_parent may be different from the real one.
 * basic->bsrc can be larger than basic->bdst.
 * aufs doesn't touch the credential so
 * security_inode_copy_up{,_xattr}() are unnecessary.
 */
static int au_cpup_single(struct au_cp_generic *cpg, struct dentry *dst_parent)
{
	int err, rerr;
	aufs_bindex_t old_ibtop;
	unsigned char isdir, plink;
	struct dentry *h_src, *h_dst, *h_parent;
	struct inode *dst_inode, *h_dir, *inode, *delegated, *src_inode;
	struct super_block *sb;
	struct au_branch *br;
	struct path h_src_path;
	/* to reduce stack size */
	struct {
		struct au_dtime dt;
		struct path h_path;
		struct au_cpup_reg_attr h_src_attr;
	} *a;

	err = -ENOMEM;
	a = kmalloc(sizeof(*a), GFP_NOFS);
	if (unlikely(!a))
		goto out;
	a->h_src_attr.valid = 0;

	sb = cpg->dentry->d_sb;
	br = au_sbr(sb, cpg->bdst);
	a->h_path.mnt = au_br_mnt(br);
	h_dst = au_h_dptr(cpg->dentry, cpg->bdst);
	h_parent = h_dst->d_parent; /* dir inode is locked */
	h_dir = d_inode(h_parent);
	IMustLock(h_dir);

	h_src = au_h_dptr(cpg->dentry, cpg->bsrc);
	inode = d_inode(cpg->dentry);

	if (!dst_parent)
		dst_parent = dget_parent(cpg->dentry);
	else
		dget(dst_parent);

	plink = !!au_opt_test(au_mntflags(sb), PLINK);
	dst_inode = au_h_iptr(inode, cpg->bdst);
	if (dst_inode) {
		if (unlikely(!plink)) {
			err = -EIO;
			AuIOErr("hi%lu(i%lu) exists on b%d "
				"but plink is disabled\n",
				dst_inode->i_ino, inode->i_ino, cpg->bdst);
			goto out_parent;
		}

		if (dst_inode->i_nlink) {
			const int do_dt = au_ftest_cpup(cpg->flags, DTIME);

			h_src = au_plink_lkup(inode, cpg->bdst);
			err = PTR_ERR(h_src);
			if (IS_ERR(h_src))
				goto out_parent;
			if (unlikely(d_is_negative(h_src))) {
				err = -EIO;
				AuIOErr("i%lu exists on b%d "
					"but not pseudo-linked\n",
					inode->i_ino, cpg->bdst);
				dput(h_src);
				goto out_parent;
			}

			if (do_dt) {
				a->h_path.dentry = h_parent;
				au_dtime_store(&a->dt, dst_parent, &a->h_path);
			}

			a->h_path.dentry = h_dst;
			delegated = NULL;
			err = vfsub_link(h_src, h_dir, &a->h_path, &delegated);
			if (!err && au_ftest_cpup(cpg->flags, RENAME))
				err = au_do_ren_after_cpup(cpg, &a->h_path);
			if (do_dt)
				au_dtime_revert(&a->dt);
			if (unlikely(err == -EWOULDBLOCK)) {
				pr_warn("cannot retry for NFSv4 delegation"
					" for an internal link\n");
				iput(delegated);
			}
			dput(h_src);
			goto out_parent;
		} else
			/* todo: cpup_wh_file? */
			/* udba work */
			au_update_ibrange(inode, /*do_put_zero*/1);
	}

	isdir = S_ISDIR(inode->i_mode);
	old_ibtop = au_ibtop(inode);
	err = cpup_entry(cpg, dst_parent, &a->h_src_attr);
	if (unlikely(err))
		goto out_rev;
	dst_inode = d_inode(h_dst);
	inode_lock_nested(dst_inode, AuLsc_I_CHILD2);
	/* todo: necessary? */
	/* au_pin_hdir_unlock(cpg->pin); */

	h_src_path.dentry = h_src;
	h_src_path.mnt = au_sbr_mnt(sb, cpg->bsrc);
	err = cpup_iattr(cpg->dentry, cpg->bdst, &h_src_path, &a->h_src_attr);
	if (unlikely(err)) {
		/* todo: necessary? */
		/* au_pin_hdir_relock(cpg->pin); */ /* ignore an error */
		inode_unlock(dst_inode);
		goto out_rev;
	}

	if (cpg->bdst < old_ibtop) {
		if (S_ISREG(inode->i_mode)) {
			err = au_dy_iaop(inode, cpg->bdst, dst_inode);
			if (unlikely(err)) {
				/* ignore an error */
				/* au_pin_hdir_relock(cpg->pin); */
				inode_unlock(dst_inode);
				goto out_rev;
			}
		}
		au_set_ibtop(inode, cpg->bdst);
	} else
		au_set_ibbot(inode, cpg->bdst);
	au_set_h_iptr(inode, cpg->bdst, au_igrab(dst_inode),
		      au_hi_flags(inode, isdir));

	/* todo: necessary? */
	/* err = au_pin_hdir_relock(cpg->pin); */
	inode_unlock(dst_inode);
	if (unlikely(err))
		goto out_rev;

	src_inode = d_inode(h_src);
	if (!isdir
	    && (src_inode->i_nlink > 1
		|| src_inode->i_state & I_LINKABLE)
	    && plink)
		au_plink_append(inode, cpg->bdst, h_dst);

	if (au_ftest_cpup(cpg->flags, RENAME)) {
		a->h_path.dentry = h_dst;
		err = au_do_ren_after_cpup(cpg, &a->h_path);
	}
	if (!err)
		goto out_parent; /* success */

	/* revert */
out_rev:
	a->h_path.dentry = h_parent;
	au_dtime_store(&a->dt, dst_parent, &a->h_path);
	a->h_path.dentry = h_dst;
	rerr = 0;
	if (d_is_positive(h_dst)) {
		if (!isdir) {
			/* no delegation since it is just created */
			rerr = vfsub_unlink(h_dir, &a->h_path,
					    /*delegated*/NULL, /*force*/0);
		} else
			rerr = vfsub_rmdir(h_dir, &a->h_path);
	}
	au_dtime_revert(&a->dt);
	if (rerr) {
		AuIOErr("failed removing broken entry(%d, %d)\n", err, rerr);
		err = -EIO;
	}
out_parent:
	dput(dst_parent);
	au_kfree_rcu(a);
out:
	return err;
}

#if 0 /* reserved */
struct au_cpup_single_args {
	int *errp;
	struct au_cp_generic *cpg;
	struct dentry *dst_parent;
};

static void au_call_cpup_single(void *args)
{
	struct au_cpup_single_args *a = args;

	au_pin_hdir_acquire_nest(a->cpg->pin);
	*a->errp = au_cpup_single(a->cpg, a->dst_parent);
	au_pin_hdir_release(a->cpg->pin);
}
#endif

/*
 * prevent SIGXFSZ in copy-up.
 * testing CAP_MKNOD is for generic fs,
 * but CAP_FSETID is for xfs only, currently.
 */
static int au_cpup_sio_test(struct au_pin *pin, umode_t mode)
{
	int do_sio;
	struct super_block *sb;
	struct inode *h_dir;

	do_sio = 0;
	sb = au_pinned_parent(pin)->d_sb;
	if (!au_wkq_test()
	    && (!au_sbi(sb)->si_plink_maint_pid
		|| au_plink_maint(sb, AuLock_NOPLM))) {
		switch (mode & S_IFMT) {
		case S_IFREG:
			/* no condition about RLIMIT_FSIZE and the file size */
			do_sio = 1;
			break;
		case S_IFCHR:
		case S_IFBLK:
			do_sio = !capable(CAP_MKNOD);
			break;
		}
		if (!do_sio)
			do_sio = ((mode & (S_ISUID | S_ISGID))
				  && !capable(CAP_FSETID));
		/* this workaround may be removed in the future */
		if (!do_sio) {
			h_dir = au_pinned_h_dir(pin);
			do_sio = h_dir->i_mode & S_ISVTX;
		}
	}

	return do_sio;
}

#if 0 /* reserved */
int au_sio_cpup_single(struct au_cp_generic *cpg, struct dentry *dst_parent)
{
	int err, wkq_err;
	struct dentry *h_dentry;

	h_dentry = au_h_dptr(cpg->dentry, cpg->bsrc);
	if (!au_cpup_sio_test(pin, d_inode(h_dentry)->i_mode))
		err = au_cpup_single(cpg, dst_parent);
	else {
		struct au_cpup_single_args args = {
			.errp		= &err,
			.cpg		= cpg,
			.dst_parent	= dst_parent
		};
		wkq_err = au_wkq_wait(au_call_cpup_single, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	return err;
}
#endif

/*
 * copyup the @dentry from the first active lower branch to @bdst,
 * using au_cpup_single().
 */
static int au_cpup_simple(struct au_cp_generic *cpg)
{
	int err;
	unsigned int flags_orig;
	struct dentry *dentry;

	AuDebugOn(cpg->bsrc < 0);

	dentry = cpg->dentry;
	DiMustWriteLock(dentry);

	err = au_lkup_neg(dentry, cpg->bdst, /*wh*/1);
	if (!err) {
		flags_orig = cpg->flags;
		au_fset_cpup(cpg->flags, RENAME);
		err = au_cpup_single(cpg, NULL);
		cpg->flags = flags_orig;
		if (!err)
			return 0; /* success */

		/* revert */
		au_set_h_dptr(dentry, cpg->bdst, NULL);
		au_set_dbtop(dentry, cpg->bsrc);
	}

	return err;
}

struct au_cpup_simple_args {
	int *errp;
	struct au_cp_generic *cpg;
};

static void au_call_cpup_simple(void *args)
{
	struct au_cpup_simple_args *a = args;

	au_pin_hdir_acquire_nest(a->cpg->pin);
	*a->errp = au_cpup_simple(a->cpg);
	au_pin_hdir_release(a->cpg->pin);
}

static int au_do_sio_cpup_simple(struct au_cp_generic *cpg)
{
	int err, wkq_err;
	struct dentry *dentry, *parent;
	struct file *h_file;
	struct inode *h_dir;
	struct user_namespace *h_userns;

	dentry = cpg->dentry;
	h_file = NULL;
	if (au_ftest_cpup(cpg->flags, HOPEN)) {
		AuDebugOn(cpg->bsrc < 0);
		h_file = au_h_open_pre(dentry, cpg->bsrc, /*force_wr*/0);
		err = PTR_ERR(h_file);
		if (IS_ERR(h_file))
			goto out;
	}

	parent = dget_parent(dentry);
	h_dir = au_h_iptr(d_inode(parent), cpg->bdst);
	h_userns = au_sbr_userns(dentry->d_sb, cpg->bdst);
	if (!au_test_h_perm_sio(h_userns, h_dir, MAY_EXEC | MAY_WRITE)
	    && !au_cpup_sio_test(cpg->pin, d_inode(dentry)->i_mode))
		err = au_cpup_simple(cpg);
	else {
		struct au_cpup_simple_args args = {
			.errp		= &err,
			.cpg		= cpg
		};
		wkq_err = au_wkq_wait(au_call_cpup_simple, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	dput(parent);
	if (h_file)
		au_h_open_post(dentry, cpg->bsrc, h_file);

out:
	return err;
}

int au_sio_cpup_simple(struct au_cp_generic *cpg)
{
	aufs_bindex_t bsrc, bbot;
	struct dentry *dentry, *h_dentry;

	if (cpg->bsrc < 0) {
		dentry = cpg->dentry;
		bbot = au_dbbot(dentry);
		for (bsrc = cpg->bdst + 1; bsrc <= bbot; bsrc++) {
			h_dentry = au_h_dptr(dentry, bsrc);
			if (h_dentry) {
				AuDebugOn(d_is_negative(h_dentry));
				break;
			}
		}
		AuDebugOn(bsrc > bbot);
		cpg->bsrc = bsrc;
	}
	AuDebugOn(cpg->bsrc <= cpg->bdst);
	return au_do_sio_cpup_simple(cpg);
}

int au_sio_cpdown_simple(struct au_cp_generic *cpg)
{
	AuDebugOn(cpg->bdst <= cpg->bsrc);
	return au_do_sio_cpup_simple(cpg);
}

/* ---------------------------------------------------------------------- */

/*
 * copyup the deleted file for writing.
 */
static int au_do_cpup_wh(struct au_cp_generic *cpg, struct dentry *wh_dentry,
			 struct file *file)
{
	int err;
	unsigned int flags_orig;
	aufs_bindex_t bsrc_orig;
	struct au_dinfo *dinfo;
	struct {
		struct au_hdentry *hd;
		struct dentry *h_dentry;
	} hdst, hsrc;

	dinfo = au_di(cpg->dentry);
	AuRwMustWriteLock(&dinfo->di_rwsem);

	bsrc_orig = cpg->bsrc;
	cpg->bsrc = dinfo->di_btop;
	hdst.hd = au_hdentry(dinfo, cpg->bdst);
	hdst.h_dentry = hdst.hd->hd_dentry;
	hdst.hd->hd_dentry = wh_dentry;
	dinfo->di_btop = cpg->bdst;

	hsrc.h_dentry = NULL;
	if (file) {
		hsrc.hd = au_hdentry(dinfo, cpg->bsrc);
		hsrc.h_dentry = hsrc.hd->hd_dentry;
		hsrc.hd->hd_dentry = au_hf_top(file)->f_path.dentry;
	}
	flags_orig = cpg->flags;
	cpg->flags = !AuCpup_DTIME;
	err = au_cpup_single(cpg, /*h_parent*/NULL);
	cpg->flags = flags_orig;
	if (file) {
		if (!err)
			err = au_reopen_nondir(file);
		hsrc.hd->hd_dentry = hsrc.h_dentry;
	}
	hdst.hd->hd_dentry = hdst.h_dentry;
	dinfo->di_btop = cpg->bsrc;
	cpg->bsrc = bsrc_orig;

	return err;
}

static int au_cpup_wh(struct au_cp_generic *cpg, struct file *file)
{
	int err;
	aufs_bindex_t bdst;
	struct au_dtime dt;
	struct dentry *dentry, *parent, *h_parent, *wh_dentry;
	struct au_branch *br;
	struct path h_path;

	dentry = cpg->dentry;
	bdst = cpg->bdst;
	br = au_sbr(dentry->d_sb, bdst);
	parent = dget_parent(dentry);
	h_parent = au_h_dptr(parent, bdst);
	wh_dentry = au_whtmp_lkup(h_parent, br, &dentry->d_name);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	h_path.dentry = h_parent;
	h_path.mnt = au_br_mnt(br);
	au_dtime_store(&dt, parent, &h_path);
	err = au_do_cpup_wh(cpg, wh_dentry, file);
	if (unlikely(err))
		goto out_wh;

	dget(wh_dentry);
	h_path.dentry = wh_dentry;
	if (!d_is_dir(wh_dentry)) {
		/* no delegation since it is just created */
		err = vfsub_unlink(d_inode(h_parent), &h_path,
				   /*delegated*/NULL, /*force*/0);
	} else
		err = vfsub_rmdir(d_inode(h_parent), &h_path);
	if (unlikely(err)) {
		AuIOErr("failed remove copied-up tmp file %pd(%d)\n",
			wh_dentry, err);
		err = -EIO;
	}
	au_dtime_revert(&dt);
	au_set_hi_wh(d_inode(dentry), bdst, wh_dentry);

out_wh:
	dput(wh_dentry);
out:
	dput(parent);
	return err;
}

struct au_cpup_wh_args {
	int *errp;
	struct au_cp_generic *cpg;
	struct file *file;
};

static void au_call_cpup_wh(void *args)
{
	struct au_cpup_wh_args *a = args;

	au_pin_hdir_acquire_nest(a->cpg->pin);
	*a->errp = au_cpup_wh(a->cpg, a->file);
	au_pin_hdir_release(a->cpg->pin);
}

int au_sio_cpup_wh(struct au_cp_generic *cpg, struct file *file)
{
	int err, wkq_err;
	aufs_bindex_t bdst;
	struct dentry *dentry, *parent, *h_orph, *h_parent;
	struct inode *dir, *h_dir, *h_tmpdir;
	struct au_wbr *wbr;
	struct au_pin wh_pin, *pin_orig;
	struct user_namespace *h_userns;

	dentry = cpg->dentry;
	bdst = cpg->bdst;
	parent = dget_parent(dentry);
	dir = d_inode(parent);
	h_orph = NULL;
	h_parent = NULL;
	h_dir = au_igrab(au_h_iptr(dir, bdst));
	h_tmpdir = h_dir;
	pin_orig = NULL;
	if (!h_dir->i_nlink) {
		wbr = au_sbr(dentry->d_sb, bdst)->br_wbr;
		h_orph = wbr->wbr_orph;

		h_parent = dget(au_h_dptr(parent, bdst));
		au_set_h_dptr(parent, bdst, dget(h_orph));
		h_tmpdir = d_inode(h_orph);
		au_set_h_iptr(dir, bdst, au_igrab(h_tmpdir), /*flags*/0);

		inode_lock_nested(h_tmpdir, AuLsc_I_PARENT3);
		/* todo: au_h_open_pre()? */

		pin_orig = cpg->pin;
		au_pin_init(&wh_pin, dentry, bdst, AuLsc_DI_PARENT,
			    AuLsc_I_PARENT3, cpg->pin->udba, AuPin_DI_LOCKED);
		cpg->pin = &wh_pin;
	}

	h_userns = au_sbr_userns(dentry->d_sb, bdst);
	if (!au_test_h_perm_sio(h_userns, h_tmpdir, MAY_EXEC | MAY_WRITE)
	    && !au_cpup_sio_test(cpg->pin, d_inode(dentry)->i_mode))
		err = au_cpup_wh(cpg, file);
	else {
		struct au_cpup_wh_args args = {
			.errp	= &err,
			.cpg	= cpg,
			.file	= file
		};
		wkq_err = au_wkq_wait(au_call_cpup_wh, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	if (h_orph) {
		inode_unlock(h_tmpdir);
		/* todo: au_h_open_post()? */
		au_set_h_iptr(dir, bdst, au_igrab(h_dir), /*flags*/0);
		au_set_h_dptr(parent, bdst, h_parent);
		AuDebugOn(!pin_orig);
		cpg->pin = pin_orig;
	}
	iput(h_dir);
	dput(parent);

	return err;
}

/* ---------------------------------------------------------------------- */

/*
 * generic routine for both of copy-up and copy-down.
 */
/* cf. revalidate function in file.c */
int au_cp_dirs(struct dentry *dentry, aufs_bindex_t bdst,
	       int (*cp)(struct dentry *dentry, aufs_bindex_t bdst,
			 struct au_pin *pin,
			 struct dentry *h_parent, void *arg),
	       void *arg)
{
	int err;
	struct au_pin pin;
	struct dentry *d, *parent, *h_parent, *real_parent, *h_dentry;

	err = 0;
	parent = dget_parent(dentry);
	if (IS_ROOT(parent))
		goto out;

	au_pin_init(&pin, dentry, bdst, AuLsc_DI_PARENT2, AuLsc_I_PARENT2,
		    au_opt_udba(dentry->d_sb), AuPin_MNT_WRITE);

	/* do not use au_dpage */
	real_parent = parent;
	while (1) {
		dput(parent);
		parent = dget_parent(dentry);
		h_parent = au_h_dptr(parent, bdst);
		if (h_parent)
			goto out; /* success */

		/* find top dir which is necessary to cpup */
		do {
			d = parent;
			dput(parent);
			parent = dget_parent(d);
			di_read_lock_parent3(parent, !AuLock_IR);
			h_parent = au_h_dptr(parent, bdst);
			di_read_unlock(parent, !AuLock_IR);
		} while (!h_parent);

		if (d != real_parent)
			di_write_lock_child3(d);

		/* somebody else might create while we were sleeping */
		h_dentry = au_h_dptr(d, bdst);
		if (!h_dentry || d_is_negative(h_dentry)) {
			if (h_dentry)
				au_update_dbtop(d);

			au_pin_set_dentry(&pin, d);
			err = au_do_pin(&pin);
			if (!err) {
				err = cp(d, bdst, &pin, h_parent, arg);
				au_unpin(&pin);
			}
		}

		if (d != real_parent)
			di_write_unlock(d);
		if (unlikely(err))
			break;
	}

out:
	dput(parent);
	return err;
}

static int au_cpup_dir(struct dentry *dentry, aufs_bindex_t bdst,
		       struct au_pin *pin,
		       struct dentry *h_parent __maybe_unused,
		       void *arg __maybe_unused)
{
	struct au_cp_generic cpg = {
		.dentry	= dentry,
		.bdst	= bdst,
		.bsrc	= -1,
		.len	= 0,
		.pin	= pin,
		.flags	= AuCpup_DTIME
	};
	return au_sio_cpup_simple(&cpg);
}

int au_cpup_dirs(struct dentry *dentry, aufs_bindex_t bdst)
{
	return au_cp_dirs(dentry, bdst, au_cpup_dir, NULL);
}

int au_test_and_cpup_dirs(struct dentry *dentry, aufs_bindex_t bdst)
{
	int err;
	struct dentry *parent;
	struct inode *dir;

	parent = dget_parent(dentry);
	dir = d_inode(parent);
	err = 0;
	if (au_h_iptr(dir, bdst))
		goto out;

	di_read_unlock(parent, AuLock_IR);
	di_write_lock_parent(parent);
	/* someone else might change our inode while we were sleeping */
	if (!au_h_iptr(dir, bdst))
		err = au_cpup_dirs(dentry, bdst);
	di_downgrade_lock(parent, AuLock_IR);

out:
	dput(parent);
	return err;
}
