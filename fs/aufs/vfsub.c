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
 * sub-routines for VFS
 */

#include <linux/mnt_namespace.h>
#include <linux/nsproxy.h>
#include <linux/security.h>
#include <linux/splice.h>
#include "aufs.h"

#ifdef CONFIG_AUFS_BR_FUSE
int vfsub_test_mntns(struct vfsmount *mnt, struct super_block *h_sb)
{
	if (!au_test_fuse(h_sb) || !au_userns)
		return 0;

	return is_current_mnt_ns(mnt) ? 0 : -EACCES;
}
#endif

int vfsub_sync_filesystem(struct super_block *h_sb)
{
	int err;

	lockdep_off();
	down_read(&h_sb->s_umount);
	err = sync_filesystem(h_sb);
	up_read(&h_sb->s_umount);
	lockdep_on();

	return err;
}

/* ---------------------------------------------------------------------- */

int vfsub_update_h_iattr(struct path *h_path, int *did)
{
	int err;
	struct kstat st;
	struct super_block *h_sb;

	/*
	 * Always needs h_path->mnt for LSM or FUSE branch.
	 */
	AuDebugOn(!h_path->mnt);

	/* for remote fs, leave work for its getattr or d_revalidate */
	/* for bad i_attr fs, handle them in aufs_getattr() */
	/* still some fs may acquire i_mutex. we need to skip them */
	err = 0;
	if (!did)
		did = &err;
	h_sb = h_path->dentry->d_sb;
	*did = (!au_test_fs_remote(h_sb) && au_test_fs_refresh_iattr(h_sb));
	if (*did)
		err = vfsub_getattr(h_path, &st);

	return err;
}

/* ---------------------------------------------------------------------- */

struct file *vfsub_dentry_open(struct path *path, int flags)
{
	return dentry_open(path, flags /* | __FMODE_NONOTIFY */,
			   current_cred());
}

struct file *vfsub_filp_open(const char *path, int oflags, int mode)
{
	struct file *file;

	lockdep_off();
	file = filp_open(path,
			 oflags /* | __FMODE_NONOTIFY */,
			 mode);
	lockdep_on();
	if (IS_ERR(file))
		goto out;
	vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/

out:
	return file;
}

/*
 * Ideally this function should call VFS:do_last() in order to keep all its
 * checkings. But it is very hard for aufs to regenerate several VFS internal
 * structure such as nameidata. This is a second (or third) best approach.
 * cf. linux/fs/namei.c:do_last(), lookup_open() and atomic_open().
 */
int vfsub_atomic_open(struct inode *dir, struct dentry *dentry,
		      struct vfsub_aopen_args *args)
{
	int err;
	struct au_branch *br = args->br;
	struct file *file = args->file;
	/* copied from linux/fs/namei.c:atomic_open() */
	struct dentry *const DENTRY_NOT_SET = (void *)-1UL;

	IMustLock(dir);
	AuDebugOn(!dir->i_op->atomic_open);

	err = au_br_test_oflag(args->open_flag, br);
	if (unlikely(err))
		goto out;

	au_lcnt_inc(&br->br_nfiles);
	file->f_path.dentry = DENTRY_NOT_SET;
	file->f_path.mnt = au_br_mnt(br);
	AuDbg("%ps\n", dir->i_op->atomic_open);
	err = dir->i_op->atomic_open(dir, dentry, file, args->open_flag,
				     args->create_mode);
	if (unlikely(err < 0)) {
		au_lcnt_dec(&br->br_nfiles);
		goto out;
	}

	/* temporary workaround for nfsv4 branch */
	if (au_test_nfs(dir->i_sb))
		nfs_mark_for_revalidate(dir);

	if (file->f_mode & FMODE_CREATED)
		fsnotify_create(dir, dentry);
	if (!(file->f_mode & FMODE_OPENED)) {
		au_lcnt_dec(&br->br_nfiles);
		goto out;
	}

	/* todo: call VFS:may_open() here */
	/* todo: ima_file_check() too? */
	if (!err && (args->open_flag & __FMODE_EXEC))
		err = deny_write_access(file);
	if (!err)
		fsnotify_open(file);
	else
		au_lcnt_dec(&br->br_nfiles);
	/* note that the file is created and still opened */

out:
	return err;
}

int vfsub_kern_path(const char *name, unsigned int flags, struct path *path)
{
	int err;

	err = kern_path(name, flags, path);
	if (!err && d_is_positive(path->dentry))
		vfsub_update_h_iattr(path, /*did*/NULL); /*ignore*/
	return err;
}

struct dentry *vfsub_lookup_one_len_unlocked(const char *name,
					     struct path *ppath, int len)
{
	struct path path;

	path.dentry = lookup_one_len_unlocked(name, ppath->dentry, len);
	if (IS_ERR(path.dentry))
		goto out;
	if (d_is_positive(path.dentry)) {
		path.mnt = ppath->mnt;
		vfsub_update_h_iattr(&path, /*did*/NULL); /*ignore*/
	}

out:
	AuTraceErrPtr(path.dentry);
	return path.dentry;
}

struct dentry *vfsub_lookup_one_len(const char *name, struct path *ppath,
				    int len)
{
	struct path path;

	/* VFS checks it too, but by WARN_ON_ONCE() */
	IMustLock(d_inode(ppath->dentry));

	path.dentry = lookup_one_len(name, ppath->dentry, len);
	if (IS_ERR(path.dentry))
		goto out;
	if (d_is_positive(path.dentry)) {
		path.mnt = ppath->mnt;
		vfsub_update_h_iattr(&path, /*did*/NULL); /*ignore*/
	}

out:
	AuTraceErrPtr(path.dentry);
	return path.dentry;
}

void vfsub_call_lkup_one(void *args)
{
	struct vfsub_lkup_one_args *a = args;
	*a->errp = vfsub_lkup_one(a->name, a->ppath);
}

/* ---------------------------------------------------------------------- */

struct dentry *vfsub_lock_rename(struct dentry *d1, struct au_hinode *hdir1,
				 struct dentry *d2, struct au_hinode *hdir2)
{
	struct dentry *d;

	lockdep_off();
	d = lock_rename(d1, d2);
	lockdep_on();
	au_hn_suspend(hdir1);
	if (hdir1 != hdir2)
		au_hn_suspend(hdir2);

	return d;
}

void vfsub_unlock_rename(struct dentry *d1, struct au_hinode *hdir1,
			 struct dentry *d2, struct au_hinode *hdir2)
{
	au_hn_resume(hdir1);
	if (hdir1 != hdir2)
		au_hn_resume(hdir2);
	lockdep_off();
	unlock_rename(d1, d2);
	lockdep_on();
}

/* ---------------------------------------------------------------------- */

int vfsub_create(struct inode *dir, struct path *path, int mode, bool want_excl)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_mknod(path, d, mode, 0);
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_create(userns, dir, path->dentry, mode, want_excl);
	lockdep_on();
	if (!err) {
		struct path tmp = *path;
		int did;

		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = path->dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

int vfsub_symlink(struct inode *dir, struct path *path, const char *symname)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_symlink(path, d, symname);
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_symlink(userns, dir, path->dentry, symname);
	lockdep_on();
	if (!err) {
		struct path tmp = *path;
		int did;

		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = path->dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

int vfsub_mknod(struct inode *dir, struct path *path, int mode, dev_t dev)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_mknod(path, d, mode, new_encode_dev(dev));
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_mknod(userns, dir, path->dentry, mode, dev);
	lockdep_on();
	if (!err) {
		struct path tmp = *path;
		int did;

		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = path->dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

static int au_test_nlink(struct inode *inode)
{
	const unsigned int link_max = UINT_MAX >> 1; /* rough margin */

	if (!au_test_fs_no_limit_nlink(inode->i_sb)
	    || inode->i_nlink < link_max)
		return 0;
	return -EMLINK;
}

int vfsub_link(struct dentry *src_dentry, struct inode *dir, struct path *path,
	       struct inode **delegated_inode)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	err = au_test_nlink(d_inode(src_dentry));
	if (unlikely(err))
		return err;

	/* we don't call may_linkat() */
	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_link(src_dentry, path, d);
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_link(src_dentry, userns, dir, path->dentry, delegated_inode);
	lockdep_on();
	if (!err) {
		struct path tmp = *path;
		int did;

		/* fuse has different memory inode for the same inumber */
		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = path->dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
			tmp.dentry = src_dentry;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

int vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		 struct inode *dir, struct path *path,
		 struct inode **delegated_inode, unsigned int flags)
{
	int err;
	struct renamedata rd;
	struct path tmp = {
		.mnt	= path->mnt
	};
	struct dentry *d;

	IMustLock(dir);
	IMustLock(src_dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	tmp.dentry = src_dentry->d_parent;
	err = security_path_rename(&tmp, src_dentry, path, d, /*flags*/0);
	path->dentry = d;
	if (unlikely(err))
		goto out;

	rd.old_mnt_userns = mnt_user_ns(path->mnt);
	rd.old_dir = src_dir;
	rd.old_dentry = src_dentry;
	rd.new_mnt_userns = rd.old_mnt_userns;
	rd.new_dir = dir;
	rd.new_dentry = path->dentry;
	rd.delegated_inode = delegated_inode;
	rd.flags = flags;
	lockdep_off();
	err = vfs_rename(&rd);
	lockdep_on();
	if (!err) {
		int did;

		tmp.dentry = d->d_parent;
		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = src_dentry;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
			tmp.dentry = src_dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

int vfsub_mkdir(struct inode *dir, struct path *path, int mode)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_mkdir(path, d, mode);
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_mkdir(userns, dir, path->dentry, mode);
	lockdep_on();
	if (!err) {
		struct path tmp = *path;
		int did;

		vfsub_update_h_iattr(&tmp, &did);
		if (did) {
			tmp.dentry = path->dentry->d_parent;
			vfsub_update_h_iattr(&tmp, /*did*/NULL);
		}
		/*ignore*/
	}

out:
	return err;
}

int vfsub_rmdir(struct inode *dir, struct path *path)
{
	int err;
	struct dentry *d;
	struct user_namespace *userns;

	IMustLock(dir);

	d = path->dentry;
	path->dentry = d->d_parent;
	err = security_path_rmdir(path, d);
	path->dentry = d;
	if (unlikely(err))
		goto out;
	userns = mnt_user_ns(path->mnt);

	lockdep_off();
	err = vfs_rmdir(userns, dir, path->dentry);
	lockdep_on();
	if (!err) {
		struct path tmp = {
			.dentry	= path->dentry->d_parent,
			.mnt	= path->mnt
		};

		vfsub_update_h_iattr(&tmp, /*did*/NULL); /*ignore*/
	}

out:
	return err;
}

/* ---------------------------------------------------------------------- */

/* todo: support mmap_sem? */
ssize_t vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
		     loff_t *ppos)
{
	ssize_t err;

	lockdep_off();
	err = vfs_read(file, ubuf, count, ppos);
	lockdep_on();
	if (err >= 0)
		vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/
	return err;
}

ssize_t vfsub_read_k(struct file *file, void *kbuf, size_t count,
		     loff_t *ppos)
{
	ssize_t err;

	lockdep_off();
	err = kernel_read(file, kbuf, count, ppos);
	lockdep_on();
	AuTraceErr(err);
	if (err >= 0)
		vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/
	return err;
}

ssize_t vfsub_write_u(struct file *file, const char __user *ubuf, size_t count,
		      loff_t *ppos)
{
	ssize_t err;

	lockdep_off();
	err = vfs_write(file, ubuf, count, ppos);
	lockdep_on();
	if (err >= 0)
		vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/
	return err;
}

ssize_t vfsub_write_k(struct file *file, void *kbuf, size_t count, loff_t *ppos)
{
	ssize_t err;

	lockdep_off();
	err = kernel_write(file, kbuf, count, ppos);
	lockdep_on();
	if (err >= 0)
		vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/
	return err;
}

int vfsub_flush(struct file *file, fl_owner_t id)
{
	int err;

	err = 0;
	if (file->f_op->flush) {
		if (!au_test_nfs(file->f_path.dentry->d_sb))
			err = file->f_op->flush(file, id);
		else {
			lockdep_off();
			err = file->f_op->flush(file, id);
			lockdep_on();
		}
		if (!err)
			vfsub_update_h_iattr(&file->f_path, /*did*/NULL);
		/*ignore*/
	}
	return err;
}

int vfsub_iterate_dir(struct file *file, struct dir_context *ctx)
{
	int err;

	AuDbg("%pD, ctx{%ps, %llu}\n", file, ctx->actor, ctx->pos);

	lockdep_off();
	err = iterate_dir(file, ctx);
	lockdep_on();
	if (err >= 0)
		vfsub_update_h_iattr(&file->f_path, /*did*/NULL); /*ignore*/

	return err;
}

long vfsub_splice_to(struct file *in, loff_t *ppos,
		     struct pipe_inode_info *pipe, size_t len,
		     unsigned int flags)
{
	long err;

	lockdep_off();
	err = do_splice_to(in, ppos, pipe, len, flags);
	lockdep_on();
	file_accessed(in);
	if (err >= 0)
		vfsub_update_h_iattr(&in->f_path, /*did*/NULL); /*ignore*/
	return err;
}

long vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
		       loff_t *ppos, size_t len, unsigned int flags)
{
	long err;

	lockdep_off();
	err = do_splice_from(pipe, out, ppos, len, flags);
	lockdep_on();
	if (err >= 0)
		vfsub_update_h_iattr(&out->f_path, /*did*/NULL); /*ignore*/
	return err;
}

int vfsub_fsync(struct file *file, struct path *path, int datasync)
{
	int err;

	/* file can be NULL */
	lockdep_off();
	err = vfs_fsync(file, datasync);
	lockdep_on();
	if (!err) {
		if (!path) {
			AuDebugOn(!file);
			path = &file->f_path;
		}
		vfsub_update_h_iattr(path, /*did*/NULL); /*ignore*/
	}
	return err;
}

/* cf. open.c:do_sys_truncate() and do_sys_ftruncate() */
int vfsub_trunc(struct path *h_path, loff_t length, unsigned int attr,
		struct file *h_file)
{
	int err;
	struct inode *h_inode;
	struct super_block *h_sb;
	struct user_namespace *h_userns;

	if (!h_file) {
		err = vfsub_truncate(h_path, length);
		goto out;
	}

	h_inode = d_inode(h_path->dentry);
	h_sb = h_inode->i_sb;
	lockdep_off();
	sb_start_write(h_sb);
	lockdep_on();
	err = security_path_truncate(h_path);
	if (!err) {
		h_userns = mnt_user_ns(h_path->mnt);
		lockdep_off();
		err = do_truncate(h_userns, h_path->dentry, length, attr,
				  h_file);
		lockdep_on();
	}
	lockdep_off();
	sb_end_write(h_sb);
	lockdep_on();

out:
	return err;
}

/* ---------------------------------------------------------------------- */

struct au_vfsub_mkdir_args {
	int *errp;
	struct inode *dir;
	struct path *path;
	int mode;
};

static void au_call_vfsub_mkdir(void *args)
{
	struct au_vfsub_mkdir_args *a = args;
	*a->errp = vfsub_mkdir(a->dir, a->path, a->mode);
}

int vfsub_sio_mkdir(struct inode *dir, struct path *path, int mode)
{
	int err, do_sio, wkq_err;
	struct user_namespace *userns;

	userns = mnt_user_ns(path->mnt);
	do_sio = au_test_h_perm_sio(userns, dir, MAY_EXEC | MAY_WRITE);
	if (!do_sio) {
		lockdep_off();
		err = vfsub_mkdir(dir, path, mode);
		lockdep_on();
	} else {
		struct au_vfsub_mkdir_args args = {
			.errp	= &err,
			.dir	= dir,
			.path	= path,
			.mode	= mode
		};
		wkq_err = au_wkq_wait(au_call_vfsub_mkdir, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	return err;
}

struct au_vfsub_rmdir_args {
	int *errp;
	struct inode *dir;
	struct path *path;
};

static void au_call_vfsub_rmdir(void *args)
{
	struct au_vfsub_rmdir_args *a = args;
	*a->errp = vfsub_rmdir(a->dir, a->path);
}

int vfsub_sio_rmdir(struct inode *dir, struct path *path)
{
	int err, do_sio, wkq_err;
	struct user_namespace *userns;

	userns = mnt_user_ns(path->mnt);
	do_sio = au_test_h_perm_sio(userns, dir, MAY_EXEC | MAY_WRITE);
	if (!do_sio) {
		lockdep_off();
		err = vfsub_rmdir(dir, path);
		lockdep_on();
	} else {
		struct au_vfsub_rmdir_args args = {
			.errp	= &err,
			.dir	= dir,
			.path	= path
		};
		wkq_err = au_wkq_wait(au_call_vfsub_rmdir, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	return err;
}

/* ---------------------------------------------------------------------- */

struct notify_change_args {
	int *errp;
	struct path *path;
	struct iattr *ia;
	struct inode **delegated_inode;
};

static void call_notify_change(void *args)
{
	struct notify_change_args *a = args;
	struct inode *h_inode;
	struct user_namespace *userns;

	h_inode = d_inode(a->path->dentry);
	IMustLock(h_inode);

	*a->errp = -EPERM;
	if (!IS_IMMUTABLE(h_inode) && !IS_APPEND(h_inode)) {
		userns = mnt_user_ns(a->path->mnt);
		lockdep_off();
		*a->errp = notify_change(userns, a->path->dentry, a->ia,
					 a->delegated_inode);
		lockdep_on();
		if (!*a->errp)
			vfsub_update_h_iattr(a->path, /*did*/NULL); /*ignore*/
	}
	AuTraceErr(*a->errp);
}

int vfsub_notify_change(struct path *path, struct iattr *ia,
			struct inode **delegated_inode)
{
	int err;
	struct notify_change_args args = {
		.errp			= &err,
		.path			= path,
		.ia			= ia,
		.delegated_inode	= delegated_inode
	};

	call_notify_change(&args);

	return err;
}

int vfsub_sio_notify_change(struct path *path, struct iattr *ia,
			    struct inode **delegated_inode)
{
	int err, wkq_err;
	struct notify_change_args args = {
		.errp			= &err,
		.path			= path,
		.ia			= ia,
		.delegated_inode	= delegated_inode
	};

	wkq_err = au_wkq_wait(call_notify_change, &args);
	if (unlikely(wkq_err))
		err = wkq_err;

	return err;
}

/* ---------------------------------------------------------------------- */

struct unlink_args {
	int *errp;
	struct inode *dir;
	struct path *path;
	struct inode **delegated_inode;
};

static void call_unlink(void *args)
{
	struct unlink_args *a = args;
	struct dentry *d = a->path->dentry;
	struct inode *h_inode;
	struct user_namespace *userns;
	const int stop_sillyrename = (au_test_nfs(d->d_sb)
				      && au_dcount(d) == 1);

	IMustLock(a->dir);

	a->path->dentry = d->d_parent;
	*a->errp = security_path_unlink(a->path, d);
	a->path->dentry = d;
	if (unlikely(*a->errp))
		return;

	if (!stop_sillyrename)
		dget(d);
	h_inode = NULL;
	if (d_is_positive(d)) {
		h_inode = d_inode(d);
		ihold(h_inode);
	}

	userns = mnt_user_ns(a->path->mnt);
	lockdep_off();
	*a->errp = vfs_unlink(userns, a->dir, d, a->delegated_inode);
	lockdep_on();
	if (!*a->errp) {
		struct path tmp = {
			.dentry = d->d_parent,
			.mnt	= a->path->mnt
		};
		vfsub_update_h_iattr(&tmp, /*did*/NULL); /*ignore*/
	}

	if (!stop_sillyrename)
		dput(d);
	if (h_inode)
		iput(h_inode);

	AuTraceErr(*a->errp);
}

/*
 * @dir: must be locked.
 * @dentry: target dentry.
 */
int vfsub_unlink(struct inode *dir, struct path *path,
		 struct inode **delegated_inode, int force)
{
	int err;
	struct unlink_args args = {
		.errp			= &err,
		.dir			= dir,
		.path			= path,
		.delegated_inode	= delegated_inode
	};

	if (!force)
		call_unlink(&args);
	else {
		int wkq_err;

		wkq_err = au_wkq_wait(call_unlink, &args);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	return err;
}
