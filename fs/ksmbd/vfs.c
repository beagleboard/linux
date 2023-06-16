// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
#include <linux/filelock.h>
#endif
#include <linux/uaccess.h>
#include <linux/backing-dev.h>
#include <linux/writeback.h>
#include <linux/xattr.h>
#include <linux/falloc.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
#include <linux/genhd.h>
#endif
#include <linux/fsnotify.h>
#include <linux/dcache.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/crc32c.h>
#include <linux/sched/xacct.h>

#include "glob.h"
#include "oplock.h"
#include "connection.h"
#include "vfs.h"
#include "vfs_cache.h"
#include "smbacl.h"
#include "ndr.h"
#include "auth.h"
#include "misc.h"

#include "smb_common.h"
#include "mgmt/share_config.h"
#include "mgmt/tree_connect.h"
#include "mgmt/user_session.h"
#include "mgmt/user_config.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
extern int vfs_path_lookup(struct dentry *, struct vfsmount *,
			   const char *, unsigned int, struct path *);
#endif

static char *extract_last_component(char *path)
{
	char *p = strrchr(path, '/');

	if (p && p[1] != '\0') {
		*p = '\0';
		p++;
	} else {
		p = NULL;
	}
	return p;
}

static void ksmbd_vfs_inherit_owner(struct ksmbd_work *work,
				    struct inode *parent_inode,
				    struct inode *inode)
{
	if (!test_share_config_flag(work->tcon->share_conf,
				    KSMBD_SHARE_FLAG_INHERIT_OWNER))
		return;

	i_uid_write(inode, i_uid_read(parent_inode));
}

/**
 * ksmbd_vfs_lock_parent() - lock parent dentry if it is stable
 *
 * the parent dentry got by dget_parent or @parent could be
 * unstable, we try to lock a parent inode and lookup the
 * child dentry again.
 *
 * the reference count of @parent isn't incremented.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_lock_parent(struct mnt_idmap *idmap, struct dentry *parent,
#else
int ksmbd_vfs_lock_parent(struct user_namespace *user_ns, struct dentry *parent,
#endif
			  struct dentry *child)
{
	struct dentry *dentry;
	int ret = 0;

	inode_lock_nested(d_inode(parent), I_MUTEX_PARENT);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	dentry = lookup_one(idmap, child->d_name.name, parent,
#else
	dentry = lookup_one(user_ns, child->d_name.name, parent,
#endif
			    child->d_name.len);
#else
	dentry = lookup_one_len(child->d_name.name, parent,
				child->d_name.len);
#endif
	if (IS_ERR(dentry)) {
		ret = PTR_ERR(dentry);
		goto out_err;
	}

	if (dentry != child) {
		ret = -ESTALE;
		dput(dentry);
		goto out_err;
	}

	dput(dentry);
	return 0;
out_err:
	inode_unlock(d_inode(parent));
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_may_delete(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_may_delete(struct user_namespace *user_ns,
#endif
			 struct dentry *dentry)
{
	struct dentry *parent;
	int ret;

	parent = dget_parent(dentry);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	ret = ksmbd_vfs_lock_parent(idmap, parent, dentry);
#else
	ret = ksmbd_vfs_lock_parent(user_ns, parent, dentry);
#endif
	if (ret) {
		dput(parent);
		return ret;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	ret = inode_permission(idmap, d_inode(parent),
#else
	ret = inode_permission(user_ns, d_inode(parent),
#endif
			       MAY_EXEC | MAY_WRITE);
#else
	ret = inode_permission(d_inode(parent), MAY_EXEC | MAY_WRITE);
#endif

	inode_unlock(d_inode(parent));
	dput(parent);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_query_maximal_access(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_query_maximal_access(struct user_namespace *user_ns,
#endif
				   struct dentry *dentry, __le32 *daccess)
{
	struct dentry *parent;
	int ret = 0;

	*daccess = cpu_to_le32(FILE_READ_ATTRIBUTES | READ_CONTROL);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	if (!inode_permission(idmap, d_inode(dentry), MAY_OPEN | MAY_WRITE))
#else
	if (!inode_permission(user_ns, d_inode(dentry), MAY_OPEN | MAY_WRITE))
#endif
#else
	if (!inode_permission(d_inode(dentry), MAY_OPEN | MAY_WRITE))
#endif
		*daccess |= cpu_to_le32(WRITE_DAC | WRITE_OWNER | SYNCHRONIZE |
				FILE_WRITE_DATA | FILE_APPEND_DATA |
				FILE_WRITE_EA | FILE_WRITE_ATTRIBUTES |
				FILE_DELETE_CHILD);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	if (!inode_permission(idmap, d_inode(dentry), MAY_OPEN | MAY_READ))
#else
	if (!inode_permission(user_ns, d_inode(dentry), MAY_OPEN | MAY_READ))
#endif
#else
	if (!inode_permission(d_inode(dentry), MAY_OPEN | MAY_READ))
#endif
		*daccess |= FILE_READ_DATA_LE | FILE_READ_EA_LE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	if (!inode_permission(idmap, d_inode(dentry), MAY_OPEN | MAY_EXEC))
#else
	if (!inode_permission(user_ns, d_inode(dentry), MAY_OPEN | MAY_EXEC))
#endif
#else
	if (!inode_permission(d_inode(dentry), MAY_OPEN | MAY_EXEC))
#endif
		*daccess |= FILE_EXECUTE_LE;

	parent = dget_parent(dentry);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	ret = ksmbd_vfs_lock_parent(idmap, parent, dentry);
#else
	ret = ksmbd_vfs_lock_parent(user_ns, parent, dentry);
#endif
	if (ret) {
		dput(parent);
		return ret;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	if (!inode_permission(idmap, d_inode(parent), MAY_EXEC | MAY_WRITE))
#else
	if (!inode_permission(user_ns, d_inode(parent), MAY_EXEC | MAY_WRITE))
#endif
#else
	if (!inode_permission(d_inode(parent), MAY_EXEC | MAY_WRITE))
#endif
		*daccess |= FILE_DELETE_LE;

	inode_unlock(d_inode(parent));
	dput(parent);
	return ret;
}

/**
 * ksmbd_vfs_create() - vfs helper for smb create file
 * @work:	work
 * @name:	file name that is relative to share
 * @mode:	file create mode
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_create(struct ksmbd_work *work, const char *name, umode_t mode)
{
	struct path path;
	struct dentry *dentry;
	int err;

	dentry = ksmbd_vfs_kern_path_create(work, name,
					    LOOKUP_NO_SYMLINKS, &path);
	if (IS_ERR(dentry)) {
		err = PTR_ERR(dentry);
		if (err != -ENOENT)
			pr_err("path create failed for %s, err %d\n",
			       name, err);
		return err;
	}

	mode |= S_IFREG;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = vfs_create(mnt_idmap(path.mnt), d_inode(path.dentry),
			 dentry, mode, true);
#else
	err = vfs_create(mnt_user_ns(path.mnt), d_inode(path.dentry),
			 dentry, mode, true);
#endif
#else
	err = vfs_create(d_inode(path.dentry), dentry, mode, true);
#endif
	if (!err) {
		ksmbd_vfs_inherit_owner(work, d_inode(path.dentry),
					d_inode(dentry));
	} else {
		pr_err("File(%s): creation failed (err:%d)\n", name, err);
	}
	done_path_create(&path, dentry);
	return err;
}

/**
 * ksmbd_vfs_mkdir() - vfs helper for smb create directory
 * @work:	work
 * @name:	directory name that is relative to share
 * @mode:	directory create mode
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_mkdir(struct ksmbd_work *work, const char *name, umode_t mode)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap;
#else
	struct user_namespace *user_ns;
#endif
	struct path path;
	struct dentry *dentry;
	int err;

	dentry = ksmbd_vfs_kern_path_create(work, name,
					    LOOKUP_NO_SYMLINKS | LOOKUP_DIRECTORY,
					    &path);
	if (IS_ERR(dentry)) {
		err = PTR_ERR(dentry);
		if (err != -EEXIST)
			ksmbd_debug(VFS, "path create failed for %s, err %d\n",
				    name, err);
		return err;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	idmap = mnt_idmap(path.mnt);
#else
	user_ns = mnt_user_ns(path.mnt);
#endif
	mode |= S_IFDIR;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = vfs_mkdir(idmap, d_inode(path.dentry), dentry, mode);
#else
	err = vfs_mkdir(user_ns, d_inode(path.dentry), dentry, mode);
#endif
#else
	err = vfs_mkdir(d_inode(path.dentry), dentry, mode);
#endif
	if (err) {
		goto out;
	} else if (d_unhashed(dentry)) {
		struct dentry *d;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		d = lookup_one(idmap, dentry->d_name.name, dentry->d_parent,
#else
		d = lookup_one(user_ns, dentry->d_name.name, dentry->d_parent,
#endif
			       dentry->d_name.len);
#else
		d = lookup_one_len(dentry->d_name.name, dentry->d_parent,
				   dentry->d_name.len);
#endif
		if (IS_ERR(d)) {
			err = PTR_ERR(d);
			goto out;
		}
		if (unlikely(d_is_negative(d))) {
			dput(d);
			err = -ENOENT;
			goto out;
		}

		ksmbd_vfs_inherit_owner(work, d_inode(path.dentry), d_inode(d));
		dput(d);
	}
out:
	done_path_create(&path, dentry);
	if (err)
		pr_err("mkdir(%s): creation failed (err:%d)\n", name, err);
	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static ssize_t ksmbd_vfs_getcasexattr(struct mnt_idmap *idmap,
#else
static ssize_t ksmbd_vfs_getcasexattr(struct user_namespace *user_ns,
#endif
				      struct dentry *dentry, char *attr_name,
				      int attr_name_len, char **attr_value)
{
	char *name, *xattr_list = NULL;
	ssize_t value_len = -ENOENT, xattr_list_len;

	xattr_list_len = ksmbd_vfs_listxattr(dentry, &xattr_list);
	if (xattr_list_len <= 0)
		goto out;

	for (name = xattr_list; name - xattr_list < xattr_list_len;
			name += strlen(name) + 1) {
		ksmbd_debug(VFS, "%s, len %zd\n", name, strlen(name));
		if (strncasecmp(attr_name, name, attr_name_len))
			continue;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		value_len = ksmbd_vfs_getxattr(idmap,
#else
		value_len = ksmbd_vfs_getxattr(user_ns,
#endif
					       dentry,
					       name,
					       attr_value);
		if (value_len < 0)
			pr_err("failed to get xattr in file\n");
		break;
	}

out:
	kvfree(xattr_list);
	return value_len;
}

static int ksmbd_vfs_stream_read(struct ksmbd_file *fp, char *buf, loff_t *pos,
				 size_t count)
{
	ssize_t v_len;
	char *stream_buf = NULL;

	ksmbd_debug(VFS, "read stream data pos : %llu, count : %zd\n",
		    *pos, count);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	v_len = ksmbd_vfs_getcasexattr(file_mnt_idmap(fp->filp),
#else
	v_len = ksmbd_vfs_getcasexattr(file_mnt_user_ns(fp->filp),
#endif
				       fp->filp->f_path.dentry,
				       fp->stream.name,
				       fp->stream.size,
				       &stream_buf);
	if ((int)v_len <= 0)
		return (int)v_len;

	if (v_len <= *pos) {
		count = -EINVAL;
		goto free_buf;
	}

	if (v_len - *pos < count)
		count = v_len - *pos;

	memcpy(buf, &stream_buf[*pos], count);

free_buf:
	kvfree(stream_buf);
	return count;
}

/**
 * check_lock_range() - vfs helper for smb byte range file locking
 * @filp:	the file to apply the lock to
 * @start:	lock start byte offset
 * @end:	lock end byte offset
 * @type:	byte range type read/write
 *
 * Return:	0 on success, otherwise error
 */
static int check_lock_range(struct file *filp, loff_t start, loff_t end,
			    unsigned char type)
{
	struct file_lock *flock;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
	struct file_lock_context *ctx = locks_inode_context(file_inode(filp));
#else
	struct file_lock_context *ctx = file_inode(filp)->i_flctx;
#endif
	int error = 0;

	if (!ctx || list_empty_careful(&ctx->flc_posix))
		return 0;

	spin_lock(&ctx->flc_lock);
	list_for_each_entry(flock, &ctx->flc_posix, fl_list) {
		/* check conflict locks */
		if (flock->fl_end >= start && end >= flock->fl_start) {
			if (flock->fl_type == F_RDLCK) {
				if (type == WRITE) {
					pr_err("not allow write by shared lock\n");
					error = 1;
					goto out;
				}
			} else if (flock->fl_type == F_WRLCK) {
				/* check owner in lock */
				if (flock->fl_file != filp) {
					error = 1;
					pr_err("not allow rw access by exclusive lock from other opens\n");
					goto out;
				}
			}
		}
	}
out:
	spin_unlock(&ctx->flc_lock);
	return error;
}

/**
 * ksmbd_vfs_read() - vfs helper for smb file read
 * @work:	smb work
 * @fid:	file id of open file
 * @count:	read byte count
 * @pos:	file pos
 *
 * Return:	number of read bytes on success, otherwise error
 */
int ksmbd_vfs_read(struct ksmbd_work *work, struct ksmbd_file *fp, size_t count,
		   loff_t *pos)
{
	struct file *filp = fp->filp;
	ssize_t nbytes = 0;
	char *rbuf = work->aux_payload_buf;
	struct inode *inode = file_inode(filp);

	if (S_ISDIR(inode->i_mode))
		return -EISDIR;

	if (unlikely(count == 0))
		return 0;

	if (work->conn->connection_type) {
		if (!(fp->daccess & (FILE_READ_DATA_LE | FILE_EXECUTE_LE))) {
			pr_err("no right to read(%pD)\n", fp->filp);
			return -EACCES;
		}
	}

	if (ksmbd_stream_fd(fp))
		return ksmbd_vfs_stream_read(fp, rbuf, pos, count);

	if (!work->tcon->posix_extensions) {
		int ret;

		ret = check_lock_range(filp, *pos, *pos + count - 1, READ);
		if (ret) {
			pr_err("unable to read due to lock\n");
			return -EAGAIN;
		}
	}

	nbytes = kernel_read(filp, rbuf, count, pos);
	if (nbytes < 0) {
		pr_err("smb read failed, err = %zd\n", nbytes);
		return nbytes;
	}

	filp->f_pos = *pos;
	return nbytes;
}

static int ksmbd_vfs_stream_write(struct ksmbd_file *fp, char *buf, loff_t *pos,
				  size_t count)
{
	char *stream_buf = NULL, *wbuf;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap = file_mnt_idmap(fp->filp);
#else
	struct user_namespace *user_ns = file_mnt_user_ns(fp->filp);
#endif
	size_t size, v_len;
	int err = 0;

	ksmbd_debug(VFS, "write stream data pos : %llu, count : %zd\n",
		    *pos, count);

	size = *pos + count;
	if (size > XATTR_SIZE_MAX) {
		size = XATTR_SIZE_MAX;
		count = (*pos + count) - XATTR_SIZE_MAX;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	v_len = ksmbd_vfs_getcasexattr(idmap,
#else
	v_len = ksmbd_vfs_getcasexattr(user_ns,
#endif
				       fp->filp->f_path.dentry,
				       fp->stream.name,
				       fp->stream.size,
				       &stream_buf);
	if ((int)v_len < 0) {
		pr_err("not found stream in xattr : %zd\n", v_len);
		err = (int)v_len;
		goto out;
	}

	if (v_len < size) {
		wbuf = kvmalloc(size, GFP_KERNEL | __GFP_ZERO);
		if (!wbuf) {
			err = -ENOMEM;
			goto out;
		}

		if (v_len > 0)
			memcpy(wbuf, stream_buf, v_len);
		kvfree(stream_buf);
		stream_buf = wbuf;
	}

	memcpy(&stream_buf[*pos], buf, count);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = ksmbd_vfs_setxattr(idmap,
#else
	err = ksmbd_vfs_setxattr(user_ns,
#endif
				 fp->filp->f_path.dentry,
				 fp->stream.name,
				 (void *)stream_buf,
				 size,
				 0);
	if (err < 0)
		goto out;

	fp->filp->f_pos = *pos;
	err = 0;
out:
	kvfree(stream_buf);
	return err;
}

/**
 * ksmbd_vfs_write() - vfs helper for smb file write
 * @work:	work
 * @fid:	file id of open file
 * @buf:	buf containing data for writing
 * @count:	read byte count
 * @pos:	file pos
 * @sync:	fsync after write
 * @written:	number of bytes written
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_write(struct ksmbd_work *work, struct ksmbd_file *fp,
		    char *buf, size_t count, loff_t *pos, bool sync,
		    ssize_t *written)
{
	struct file *filp;
	loff_t	offset = *pos;
	int err = 0;

	if (work->conn->connection_type) {
		if (!(fp->daccess & FILE_WRITE_DATA_LE)) {
			pr_err("no right to write(%pD)\n", fp->filp);
			err = -EACCES;
			goto out;
		}
	}

	filp = fp->filp;

	if (ksmbd_stream_fd(fp)) {
		err = ksmbd_vfs_stream_write(fp, buf, pos, count);
		if (!err)
			*written = count;
		goto out;
	}

	if (!work->tcon->posix_extensions) {
		err = check_lock_range(filp, *pos, *pos + count - 1, WRITE);
		if (err) {
			pr_err("unable to write due to lock\n");
			err = -EAGAIN;
			goto out;
		}
	}

	/* Do we need to break any of a levelII oplock? */
	smb_break_all_levII_oplock(work, fp, 1);

	err = kernel_write(filp, buf, count, pos);
	if (err < 0) {
		ksmbd_debug(VFS, "smb write failed, err = %d\n", err);
		goto out;
	}

	filp->f_pos = *pos;
	*written = err;
	err = 0;
	if (sync) {
		err = vfs_fsync_range(filp, offset, offset + *written, 0);
		if (err < 0)
			pr_err("fsync failed for filename = %pD, err = %d\n",
			       fp->filp, err);
	}

out:
	return err;
}

/**
 * ksmbd_vfs_getattr() - vfs helper for smb getattr
 * @work:	work
 * @fid:	file id of open file
 * @attrs:	inode attributes
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_getattr(const struct path *path, struct kstat *stat)
{
	int err;

	err = vfs_getattr(path, stat, STATX_BTIME, AT_STATX_SYNC_AS_STAT);
	if (err)
		pr_err("getattr failed, err %d\n", err);
	return err;
}

#ifdef CONFIG_SMB_INSECURE_SERVER
/**
 * smb_check_attrs() - sanitize inode attributes
 * @inode:	inode
 * @attrs:	inode attributes
 */
static void smb_check_attrs(struct inode *inode, struct iattr *attrs)
{
	/* sanitize the mode change */
	if (attrs->ia_valid & ATTR_MODE) {
		attrs->ia_mode &= S_IALLUGO;
		attrs->ia_mode |= (inode->i_mode & ~S_IALLUGO);
	}

	/* Revoke setuid/setgid on chown */
	if (!S_ISDIR(inode->i_mode) &&
	    (((attrs->ia_valid & ATTR_UID) &&
	      !uid_eq(attrs->ia_uid, inode->i_uid)) ||
	     ((attrs->ia_valid & ATTR_GID) &&
	      !gid_eq(attrs->ia_gid, inode->i_gid)))) {
		attrs->ia_valid |= ATTR_KILL_PRIV;
		if (attrs->ia_valid & ATTR_MODE) {
			/* we're setting mode too, just clear the s*id bits */
			attrs->ia_mode &= ~S_ISUID;
			if (attrs->ia_mode & 0010)
				attrs->ia_mode &= ~S_ISGID;
		} else {
			/* set ATTR_KILL_* bits and let VFS handle it */
			attrs->ia_valid |= (ATTR_KILL_SUID | ATTR_KILL_SGID);
		}
	}
}

/**
 * ksmbd_vfs_setattr() - vfs helper for smb setattr
 * @work:	work
 * @name:	file name
 * @fid:	file id of open file
 * @attrs:	inode attributes
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_setattr(struct ksmbd_work *work, const char *name, u64 fid,
		      struct iattr *attrs)
{
	struct file *filp;
	struct dentry *dentry;
	struct inode *inode;
	struct path path;
	bool update_size = false;
	int err = 0;
	struct ksmbd_file *fp = NULL;
	struct user_namespace *user_ns;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	if (name) {
		err = kern_path(name, 0, &path);
		if (err) {
			ksmbd_revert_fsids(work);
			ksmbd_debug(VFS, "lookup failed for %s, err = %d\n",
				    name, err);
			return -ENOENT;
		}
		dentry = path.dentry;
		inode = d_inode(dentry);
		user_ns = mnt_user_ns(path.mnt);
	} else {
		fp = ksmbd_lookup_fd_fast(work, fid);
		if (!fp) {
			ksmbd_revert_fsids(work);
			pr_err("failed to get filp for fid %llu\n", fid);
			return -ENOENT;
		}

		filp = fp->filp;
		dentry = filp->f_path.dentry;
		inode = d_inode(dentry);
		user_ns = file_mnt_user_ns(filp);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	err = inode_permission(user_ns, d_inode(dentry), MAY_WRITE);
#else
	err = inode_permission(d_inode(dentry), MAY_WRITE);
#endif
	if (err)
		goto out;

	/* no need to update mode of symlink */
	if (S_ISLNK(inode->i_mode))
		attrs->ia_valid &= ~ATTR_MODE;

	/* skip setattr, if nothing to update */
	if (!attrs->ia_valid) {
		err = 0;
		goto out;
	}

	smb_check_attrs(inode, attrs);
	if (attrs->ia_valid & ATTR_SIZE) {
		err = get_write_access(inode);
		if (err)
			goto out;
		update_size = true;
	}

	attrs->ia_valid |= ATTR_CTIME;

	inode_lock(inode);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	err = notify_change(user_ns, dentry, attrs, NULL);
#else
	err = notify_change(dentry, attrs, NULL);
#endif
	inode_unlock(inode);

	if (update_size)
		put_write_access(inode);

	if (!err) {
		sync_inode_metadata(inode, 1);
		ksmbd_debug(VFS, "fid %llu, setattr done\n", fid);
	}

out:
	if (name)
		path_put(&path);
	ksmbd_fd_put(work, fp);
	ksmbd_revert_fsids(work);
	return err;
}

/**
 * ksmbd_vfs_symlink() - vfs helper for creating smb symlink
 * @name:	source file name
 * @symname:	symlink name
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_symlink(struct ksmbd_work *work, const char *name,
		      const char *symname)
{
	struct path path;
	struct dentry *dentry;
	int err;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	dentry = kern_path_create(AT_FDCWD, symname, &path, 0);
	if (IS_ERR(dentry)) {
		ksmbd_revert_fsids(work);
		err = PTR_ERR(dentry);
		pr_err("path create failed for %s, err %d\n", name, err);
		return err;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	err = vfs_symlink(mnt_user_ns(path.mnt), d_inode(dentry->d_parent), dentry, name);
#else
	err = vfs_symlink(d_inode(dentry->d_parent), dentry, name);
#endif
	if (err && (err != -EEXIST || err != -ENOSPC))
		ksmbd_debug(VFS, "failed to create symlink, err %d\n", err);

	done_path_create(&path, dentry);
	ksmbd_revert_fsids(work);
	return err;
}

/**
 * ksmbd_vfs_readlink() - vfs helper for reading value of symlink
 * @path:	path of symlink
 * @buf:	destination buffer for symlink value
 * @lenp:	destination buffer length
 *
 * Return:	symlink value length on success, otherwise error
 */
int ksmbd_vfs_readlink(struct path *path, char *buf, int lenp)
{
	struct inode *inode;
	int err;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	const char *link;
	DEFINE_DELAYED_CALL(done);
	int len;
#else
	mm_segment_t old_fs;
#endif

	if (!path)
		return -ENOENT;

	inode = d_inode(path->dentry);
	if (!S_ISLNK(inode->i_mode))
		return -EINVAL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	link = vfs_get_link(path->dentry, &done);
	if (IS_ERR(link)) {
		err = PTR_ERR(link);
		pr_err("readlink failed, err = %d\n", err);
		return err;
	}

	len = strlen(link);
	if (len > lenp)
		len = lenp;

	memcpy(buf, link, len);
	do_delayed_call(&done);

	return 0;
#else
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = inode->i_op->readlink(path->dentry, (char __user *)buf, lenp);
	set_fs(old_fs);
	if (err < 0)
		pr_err("readlink failed, err = %d\n", err);

	return err;
#endif
}

int ksmbd_vfs_readdir_name(struct ksmbd_work *work,
			   struct user_namespace *user_ns,
			   struct ksmbd_kstat *ksmbd_kstat,
			   const char *de_name, int de_name_len,
			   const char *dir_path)
{
	struct path path;
	int rc, file_pathlen, dir_pathlen;
	char *name;

	dir_pathlen = strlen(dir_path);
	/* 1 for '/'*/
	file_pathlen = dir_pathlen +  de_name_len + 1;
	name = kmalloc(file_pathlen + 1, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	memcpy(name, dir_path, dir_pathlen);
	memset(name + dir_pathlen, '/', 1);
	memcpy(name + dir_pathlen + 1, de_name, de_name_len);
	name[file_pathlen] = '\0';

	rc = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path, 1);
	if (rc) {
		pr_err("lookup failed: %s [%d]\n", name, rc);
		kfree(name);
		return -ENOMEM;
	}

	ksmbd_vfs_fill_dentry_attrs(work, user_ns, path.dentry, ksmbd_kstat);
	path_put(&path);
	kfree(name);
	return 0;
}
#endif

/**
 * ksmbd_vfs_fsync() - vfs helper for smb fsync
 * @work:	work
 * @fid:	file id of open file
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_fsync(struct ksmbd_work *work, u64 fid, u64 p_id)
{
	struct ksmbd_file *fp;
	int err;

	fp = ksmbd_lookup_fd_slow(work, fid, p_id);
	if (!fp) {
		pr_err("failed to get filp for fid %llu\n", fid);
		return -ENOENT;
	}
	err = vfs_fsync(fp->filp, 0);
	if (err < 0)
		pr_err("smb fsync failed, err = %d\n", err);
	ksmbd_fd_put(work, fp);
	return err;
}

/**
 * ksmbd_vfs_remove_file() - vfs helper for smb rmdir or unlink
 * @name:	directory or file name that is relative to share
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_remove_file(struct ksmbd_work *work, char *name)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap;
#else
	struct user_namespace *user_ns;
#endif
	struct path path;
	struct dentry *parent;
	int err;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	err = ksmbd_vfs_kern_path(work, name, LOOKUP_NO_SYMLINKS, &path, false);
	if (err) {
		ksmbd_debug(VFS, "can't get %s, err %d\n", name, err);
		ksmbd_revert_fsids(work);
		return err;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	idmap = mnt_idmap(path.mnt);
#else
	user_ns = mnt_user_ns(path.mnt);
#endif
	parent = dget_parent(path.dentry);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = ksmbd_vfs_lock_parent(idmap, parent, path.dentry);
#else
	err = ksmbd_vfs_lock_parent(user_ns, parent, path.dentry);
#endif
	if (err) {
		dput(parent);
		path_put(&path);
		ksmbd_revert_fsids(work);
		return err;
	}

	if (!d_inode(path.dentry)->i_nlink) {
		err = -ENOENT;
		goto out_err;
	}

	if (S_ISDIR(d_inode(path.dentry)->i_mode)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		err = vfs_rmdir(idmap, d_inode(parent), path.dentry);
#else
		err = vfs_rmdir(user_ns, d_inode(parent), path.dentry);
#endif
#else
		err = vfs_rmdir(d_inode(parent), path.dentry);
#endif
		if (err && err != -ENOTEMPTY)
			ksmbd_debug(VFS, "%s: rmdir failed, err %d\n", name,
				    err);
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		err = vfs_unlink(idmap, d_inode(parent), path.dentry, NULL);
#else
		err = vfs_unlink(user_ns, d_inode(parent), path.dentry, NULL);
#endif
#else
		err = vfs_unlink(d_inode(parent), path.dentry, NULL);
#endif
		if (err)
			ksmbd_debug(VFS, "%s: unlink failed, err %d\n", name,
				    err);
	}

out_err:
	inode_unlock(d_inode(parent));
	dput(parent);
	path_put(&path);
	ksmbd_revert_fsids(work);
	return err;
}

/**
 * ksmbd_vfs_link() - vfs helper for creating smb hardlink
 * @oldname:	source file name
 * @newname:	hardlink name that is relative to share
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_link(struct ksmbd_work *work, const char *oldname,
		   const char *newname)
{
	struct path oldpath, newpath;
	struct dentry *dentry;
	int err;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	err = kern_path(oldname, LOOKUP_NO_SYMLINKS, &oldpath);
	if (err) {
		pr_err("cannot get linux path for %s, err = %d\n",
		       oldname, err);
		goto out1;
	}

	dentry = ksmbd_vfs_kern_path_create(work, newname,
					    LOOKUP_NO_SYMLINKS | LOOKUP_REVAL,
					    &newpath);
	if (IS_ERR(dentry)) {
		err = PTR_ERR(dentry);
		pr_err("path create err for %s, err %d\n", newname, err);
		goto out2;
	}

	err = -EXDEV;
	if (oldpath.mnt != newpath.mnt) {
		pr_err("vfs_link failed err %d\n", err);
		goto out3;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = vfs_link(oldpath.dentry, mnt_idmap(newpath.mnt),
		       d_inode(newpath.dentry),
		       dentry, NULL);
#else
	err = vfs_link(oldpath.dentry, mnt_user_ns(newpath.mnt),
		       d_inode(newpath.dentry),
		       dentry, NULL);
#endif
#else
	err = vfs_link(oldpath.dentry, d_inode(newpath.dentry), dentry, NULL);
#endif
	if (err)
		ksmbd_debug(VFS, "vfs_link failed err %d\n", err);

out3:
	done_path_create(&newpath, dentry);
out2:
	path_put(&oldpath);
out1:
	ksmbd_revert_fsids(work);
	return err;
}

static int ksmbd_validate_entry_in_use(struct dentry *src_dent)
{
	struct dentry *dst_dent;

	spin_lock(&src_dent->d_lock);
	list_for_each_entry(dst_dent, &src_dent->d_subdirs, d_child) {
		struct ksmbd_file *child_fp;

		if (d_really_is_negative(dst_dent))
			continue;

		child_fp = ksmbd_lookup_fd_inode(d_inode(dst_dent));
		if (child_fp) {
			spin_unlock(&src_dent->d_lock);
			ksmbd_debug(VFS, "Forbid rename, sub file/dir is in use\n");
			return -EACCES;
		}
	}
	spin_unlock(&src_dent->d_lock);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int __ksmbd_vfs_rename(struct ksmbd_work *work,
			      struct mnt_idmap *src_idmap,
			      struct dentry *src_dent_parent,
			      struct dentry *src_dent,
			      struct mnt_idmap *dst_idmap,
			      struct dentry *dst_dent_parent,
			      struct dentry *trap_dent,
			      char *dst_name)
#else
static int __ksmbd_vfs_rename(struct ksmbd_work *work,
			      struct user_namespace *src_user_ns,
			      struct dentry *src_dent_parent,
			      struct dentry *src_dent,
			      struct user_namespace *dst_user_ns,
			      struct dentry *dst_dent_parent,
			      struct dentry *trap_dent,
			      char *dst_name)
#endif
{
	struct dentry *dst_dent;
	int err;

	if (!work->tcon->posix_extensions) {
		err = ksmbd_validate_entry_in_use(src_dent);
		if (err)
			return err;
	}

	if (d_really_is_negative(src_dent_parent))
		return -ENOENT;
	if (d_really_is_negative(dst_dent_parent))
		return -ENOENT;
	if (d_really_is_negative(src_dent))
		return -ENOENT;
	if (src_dent == trap_dent)
		return -EINVAL;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	dst_dent = lookup_one(dst_idmap, dst_name,
			      dst_dent_parent, strlen(dst_name));
#else
	dst_dent = lookup_one(dst_user_ns, dst_name, dst_dent_parent,
			      strlen(dst_name));
#endif
#else
	dst_dent = lookup_one_len(dst_name, dst_dent_parent,
				  strlen(dst_name));
#endif
	err = PTR_ERR(dst_dent);
	if (IS_ERR(dst_dent)) {
		pr_err("lookup failed %s [%d]\n", dst_name, err);
		goto out;
	}

	err = -ENOTEMPTY;
	if (dst_dent != trap_dent && !d_really_is_positive(dst_dent)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		struct renamedata rd = {
			.old_mnt_idmap	= src_idmap,
			.old_dir	= d_inode(src_dent_parent),
			.old_dentry	= src_dent,
			.new_mnt_idmap	= dst_idmap,
			.new_dir	= d_inode(dst_dent_parent),
			.new_dentry	= dst_dent,
		};
#else
		struct renamedata rd = {
			.old_mnt_userns	= src_user_ns,
			.old_dir	= d_inode(src_dent_parent),
			.old_dentry	= src_dent,
			.new_mnt_userns	= dst_user_ns,
			.new_dir	= d_inode(dst_dent_parent),
			.new_dentry	= dst_dent,
		};
#endif
		err = vfs_rename(&rd);
#else
		err = vfs_rename(d_inode(src_dent_parent),
				 src_dent,
				 d_inode(dst_dent_parent),
				 dst_dent,
				 NULL,
				 0);
#endif
	}
	if (err)
		pr_err("vfs_rename failed err %d\n", err);
	if (dst_dent)
		dput(dst_dent);
out:
	ksmbd_revert_fsids(work);
	return err;
}

int ksmbd_vfs_fp_rename(struct ksmbd_work *work, struct ksmbd_file *fp,
			char *newname)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	struct mnt_idmap *idmap;
#else
	struct user_namespace *user_ns;
#endif
	struct path dst_path;
	struct dentry *src_dent_parent, *dst_dent_parent;
	struct dentry *src_dent, *trap_dent, *src_child;
	char *dst_name;
	int err;

	dst_name = extract_last_component(newname);
	if (!dst_name) {
		dst_name = newname;
		newname = "";
	}

	src_dent_parent = dget_parent(fp->filp->f_path.dentry);
	src_dent = fp->filp->f_path.dentry;

	err = ksmbd_vfs_kern_path(work, newname,
				  LOOKUP_NO_SYMLINKS | LOOKUP_DIRECTORY,
				  &dst_path, false);
	if (err) {
		ksmbd_debug(VFS, "Cannot get path for %s [%d]\n", newname, err);
		goto out;
	}
	dst_dent_parent = dst_path.dentry;

	trap_dent = lock_rename(src_dent_parent, dst_dent_parent);
	dget(src_dent);
	dget(dst_dent_parent);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	idmap = file_mnt_idmap(fp->filp);
#else
	user_ns = file_mnt_user_ns(fp->filp);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	src_child = lookup_one(idmap, src_dent->d_name.name, src_dent_parent,
#else
	src_child = lookup_one(user_ns, src_dent->d_name.name, src_dent_parent,
#endif
			       src_dent->d_name.len);
#else
	src_child = lookup_one_len(src_dent->d_name.name, src_dent_parent,
				   src_dent->d_name.len);
#endif
	if (IS_ERR(src_child)) {
		err = PTR_ERR(src_child);
		goto out_lock;
	}

	if (src_child != src_dent) {
		err = -ESTALE;
		dput(src_child);
		goto out_lock;
	}
	dput(src_child);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = __ksmbd_vfs_rename(work,
				 idmap,
				 src_dent_parent,
				 src_dent,
				 mnt_idmap(dst_path.mnt),
				 dst_dent_parent,
				 trap_dent,
				 dst_name);
#else
	err = __ksmbd_vfs_rename(work,
				 user_ns,
				 src_dent_parent,
				 src_dent,
				 mnt_user_ns(dst_path.mnt),
				 dst_dent_parent,
				 trap_dent,
				 dst_name);
#endif
out_lock:
	dput(src_dent);
	dput(dst_dent_parent);
	unlock_rename(src_dent_parent, dst_dent_parent);
	path_put(&dst_path);
out:
	dput(src_dent_parent);
	return err;
}

/**
 * ksmbd_vfs_truncate() - vfs helper for smb file truncate
 * @work:	work
 * @name:	old filename
 * @fid:	file id of old file
 * @size:	truncate to given size
 *
 * Return:	0 on success, otherwise error
 */
int ksmbd_vfs_truncate(struct ksmbd_work *work,
		       struct ksmbd_file *fp, loff_t size)
{
	int err = 0;
	struct file *filp;

	if (size < 0)
		return -EINVAL;

	filp = fp->filp;

	/* Do we need to break any of a levelII oplock? */
	smb_break_all_levII_oplock(work, fp, 1);

	if (!work->tcon->posix_extensions) {
		struct inode *inode = file_inode(filp);

		if (size < inode->i_size) {
			err = check_lock_range(filp, size,
					       inode->i_size - 1, WRITE);
		} else {
			err = check_lock_range(filp, inode->i_size,
					       size - 1, WRITE);
		}

		if (err) {
			pr_err("failed due to lock\n");
			return -EAGAIN;
		}
	}

	err = vfs_truncate(&filp->f_path, size);
	if (err)
		pr_err("truncate failed, err %d\n", err);
	return err;
}

/**
 * ksmbd_vfs_listxattr() - vfs helper for smb list extended attributes
 * @dentry:	dentry of file for listing xattrs
 * @list:	destination buffer
 * @size:	destination buffer length
 *
 * Return:	xattr list length on success, otherwise error
 */
ssize_t ksmbd_vfs_listxattr(struct dentry *dentry, char **list)
{
	ssize_t size;
	char *vlist = NULL;

	size = vfs_listxattr(dentry, NULL, 0);
	if (size <= 0)
		return size;

	vlist = kvmalloc(size, GFP_KERNEL | __GFP_ZERO);
	if (!vlist)
		return -ENOMEM;

	*list = vlist;
	size = vfs_listxattr(dentry, vlist, size);
	if (size < 0) {
		ksmbd_debug(VFS, "listxattr failed\n");
		kvfree(vlist);
		*list = NULL;
	}

	return size;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static ssize_t ksmbd_vfs_xattr_len(struct mnt_idmap *idmap,
#else
static ssize_t ksmbd_vfs_xattr_len(struct user_namespace *user_ns,
#endif
				   struct dentry *dentry, char *xattr_name)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	return vfs_getxattr(idmap, dentry, xattr_name, NULL, 0);
#else
	return vfs_getxattr(user_ns, dentry, xattr_name, NULL, 0);
#endif
#else
	return vfs_getxattr(dentry, xattr_name, NULL, 0);
#endif
}

/**
 * ksmbd_vfs_getxattr() - vfs helper for smb get extended attributes value
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
+  @idmap:	idmap
#else
+  @user_ns:	user namespace
#endif
 * @dentry:	dentry of file for getting xattrs
 * @xattr_name:	name of xattr name to query
 * @xattr_buf:	destination buffer xattr value
 *
 * Return:	read xattr value length on success, otherwise error
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
ssize_t ksmbd_vfs_getxattr(struct mnt_idmap *idmap,
#else
ssize_t ksmbd_vfs_getxattr(struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   char *xattr_name, char **xattr_buf)
{
	ssize_t xattr_len;
	char *buf;

	*xattr_buf = NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	xattr_len = ksmbd_vfs_xattr_len(idmap, dentry, xattr_name);
#else
	xattr_len = ksmbd_vfs_xattr_len(user_ns, dentry, xattr_name);
#endif
	if (xattr_len < 0)
		return xattr_len;

	buf = kmalloc(xattr_len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	xattr_len = vfs_getxattr(idmap, dentry, xattr_name,
#else
	xattr_len = vfs_getxattr(user_ns, dentry, xattr_name,
#endif
				 (void *)buf, xattr_len);
#else
	xattr_len = vfs_getxattr(dentry, xattr_name, (void *)buf, xattr_len);
#endif
	if (xattr_len > 0)
		*xattr_buf = buf;
	else
		kfree(buf);
	return xattr_len;
}

/**
 * ksmbd_vfs_setxattr() - vfs helper for smb set extended attributes value
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
 * @idmap:	idmap of the relevant mount
#else
 * @user_ns:	user namespace
#endif
 * @dentry:	dentry to set XATTR at
 * @name:	xattr name for setxattr
 * @value:	xattr value to set
 * @size:	size of xattr value
 * @flags:	destination buffer length
 *
 * Return:	0 on success, otherwise error
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_setxattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_setxattr(struct user_namespace *user_ns,
#endif
		       struct dentry *dentry, const char *attr_name,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
		       void *attr_value, size_t attr_size, int flags)
#else
		       const void *attr_value, size_t attr_size, int flags)
#endif
{
	int err;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = vfs_setxattr(idmap,
#else
	err = vfs_setxattr(user_ns,
#endif
			   dentry,
#else
	err = vfs_setxattr(dentry,
#endif
			   attr_name,
			   attr_value,
			   attr_size,
			   flags);
	if (err)
		ksmbd_debug(VFS, "setxattr failed, err %d\n", err);
	return err;
}

#ifdef CONFIG_SMB_INSECURE_SERVER
int ksmbd_vfs_fsetxattr(struct ksmbd_work *work, const char *filename,
			const char *attr_name, const void *attr_value,
			size_t attr_size, int flags)
{
	struct path path;
	int err;

	if (ksmbd_override_fsids(work))
		return -ENOMEM;

	err = kern_path(filename, 0, &path);
	if (err) {
		ksmbd_revert_fsids(work);
		ksmbd_debug(VFS, "cannot get linux path %s, err %d\n",
			    filename, err);
		return err;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
	err = vfs_setxattr(mnt_user_ns(path.mnt), path.dentry,
#else
	err = vfs_setxattr(path.dentry,
#endif
			   attr_name,
			   attr_value,
			   attr_size,
			   flags);
	if (err)
		ksmbd_debug(VFS, "setxattr failed, err %d\n", err);
	path_put(&path);
	ksmbd_revert_fsids(work);
	return err;
}
#endif

struct dentry *ksmbd_vfs_kern_path_create(struct ksmbd_work *work,
					  const char *name,
					  unsigned int flags,
					  struct path *path)
{
	char *abs_name;
	struct dentry *dent;

	abs_name = convert_to_unix_name(work->tcon->share_conf, name);
	if (!abs_name)
		return ERR_PTR(-ENOMEM);

	dent = kern_path_create(AT_FDCWD, abs_name, path, flags);
	kfree(abs_name);
	return dent;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_acl_xattrs(struct mnt_idmap *idmap,
				struct dentry *dentry)
#else
int ksmbd_vfs_remove_acl_xattrs(struct user_namespace *user_ns,
				struct dentry *dentry)
#endif
{
	char *name, *xattr_list = NULL;
	ssize_t xattr_list_len;
	int err = 0;

	xattr_list_len = ksmbd_vfs_listxattr(dentry, &xattr_list);
	if (xattr_list_len < 0) {
		goto out;
	} else if (!xattr_list_len) {
		ksmbd_debug(SMB, "empty xattr in the file\n");
		goto out;
	}

	for (name = xattr_list; name - xattr_list < xattr_list_len;
	     name += strlen(name) + 1) {
		ksmbd_debug(SMB, "%s, len %zd\n", name, strlen(name));

		if (!strncmp(name, XATTR_NAME_POSIX_ACL_ACCESS,
			     sizeof(XATTR_NAME_POSIX_ACL_ACCESS) - 1) ||
		    !strncmp(name, XATTR_NAME_POSIX_ACL_DEFAULT,
			     sizeof(XATTR_NAME_POSIX_ACL_DEFAULT) - 1)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			err = vfs_remove_acl(idmap, dentry, name);
#else
			err = vfs_remove_acl(user_ns, dentry, name);
#endif
#else
			err = ksmbd_vfs_remove_xattr(user_ns, dentry, name);
#endif
			if (err)
				ksmbd_debug(SMB,
					    "remove acl xattr failed : %s\n", name);
		}
	}
out:
	kvfree(xattr_list);
	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_sd_xattrs(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_remove_sd_xattrs(struct user_namespace *user_ns,
#endif
			       struct dentry *dentry)
{
	char *name, *xattr_list = NULL;
	ssize_t xattr_list_len;
	int err = 0;

	xattr_list_len = ksmbd_vfs_listxattr(dentry, &xattr_list);
	if (xattr_list_len < 0) {
		goto out;
	} else if (!xattr_list_len) {
		ksmbd_debug(SMB, "empty xattr in the file\n");
		goto out;
	}

	for (name = xattr_list; name - xattr_list < xattr_list_len;
			name += strlen(name) + 1) {
		ksmbd_debug(SMB, "%s, len %zd\n", name, strlen(name));

		if (!strncmp(name, XATTR_NAME_SD, XATTR_NAME_SD_LEN)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			err = ksmbd_vfs_remove_xattr(idmap, dentry, name);
#else
			err = ksmbd_vfs_remove_xattr(user_ns, dentry, name);
#endif
			if (err)
				ksmbd_debug(SMB, "remove xattr failed : %s\n", name);
		}
	}
out:
	kvfree(xattr_list);
	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static struct xattr_smb_acl *ksmbd_vfs_make_xattr_posix_acl(struct mnt_idmap *idmap,
#else
static struct xattr_smb_acl *ksmbd_vfs_make_xattr_posix_acl(struct user_namespace *user_ns,
#endif
							    struct inode *inode,
							    int acl_type)
{
	struct xattr_smb_acl *smb_acl = NULL;
	struct posix_acl *posix_acls;
	struct posix_acl_entry *pa_entry;
	struct xattr_acl_entry *xa_entry;
	int i;

	if (!IS_ENABLED(CONFIG_FS_POSIX_ACL))
		return NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
	posix_acls = get_inode_acl(inode, acl_type);
#else
	posix_acls = get_acl(inode, acl_type);
#endif
	if (!posix_acls)
		return NULL;

	smb_acl = kzalloc(sizeof(struct xattr_smb_acl) +
			  sizeof(struct xattr_acl_entry) * posix_acls->a_count,
			  GFP_KERNEL);
	if (!smb_acl)
		goto out;

	smb_acl->count = posix_acls->a_count;
	pa_entry = posix_acls->a_entries;
	xa_entry = smb_acl->entries;
	for (i = 0; i < posix_acls->a_count; i++, pa_entry++, xa_entry++) {
		switch (pa_entry->e_tag) {
		case ACL_USER:
			xa_entry->type = SMB_ACL_USER;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			xa_entry->uid = posix_acl_uid_translate(idmap, pa_entry);
#else
			xa_entry->uid = posix_acl_uid_translate(user_ns, pa_entry);
#endif
			break;
		case ACL_USER_OBJ:
			xa_entry->type = SMB_ACL_USER_OBJ;
			break;
		case ACL_GROUP:
			xa_entry->type = SMB_ACL_GROUP;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			xa_entry->gid = posix_acl_gid_translate(idmap, pa_entry);
#else
			xa_entry->gid = posix_acl_gid_translate(user_ns, pa_entry);
#endif
			break;
		case ACL_GROUP_OBJ:
			xa_entry->type = SMB_ACL_GROUP_OBJ;
			break;
		case ACL_OTHER:
			xa_entry->type = SMB_ACL_OTHER;
			break;
		case ACL_MASK:
			xa_entry->type = SMB_ACL_MASK;
			break;
		default:
			pr_err("unknown type : 0x%x\n", pa_entry->e_tag);
			goto out;
		}

		if (pa_entry->e_perm & ACL_READ)
			xa_entry->perm |= SMB_ACL_READ;
		if (pa_entry->e_perm & ACL_WRITE)
			xa_entry->perm |= SMB_ACL_WRITE;
		if (pa_entry->e_perm & ACL_EXECUTE)
			xa_entry->perm |= SMB_ACL_EXECUTE;
	}
out:
	posix_acl_release(posix_acls);
	return smb_acl;
}

int ksmbd_vfs_set_sd_xattr(struct ksmbd_conn *conn,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			   struct mnt_idmap *idmap,
#else
			   struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   struct smb_ntsd *pntsd, int len)
{
	int rc;
	struct ndr sd_ndr = {0}, acl_ndr = {0};
	struct xattr_ntacl acl = {0};
	struct xattr_smb_acl *smb_acl, *def_smb_acl = NULL;
	struct inode *inode = d_inode(dentry);

	acl.version = 4;
	acl.hash_type = XATTR_SD_HASH_TYPE_SHA256;
	acl.current_time = ksmbd_UnixTimeToNT(current_time(inode));

	memcpy(acl.desc, "posix_acl", 9);
	acl.desc_len = 10;

	pntsd->osidoffset =
		cpu_to_le32(le32_to_cpu(pntsd->osidoffset) + NDR_NTSD_OFFSETOF);
	pntsd->gsidoffset =
		cpu_to_le32(le32_to_cpu(pntsd->gsidoffset) + NDR_NTSD_OFFSETOF);
	pntsd->dacloffset =
		cpu_to_le32(le32_to_cpu(pntsd->dacloffset) + NDR_NTSD_OFFSETOF);

	acl.sd_buf = (char *)pntsd;
	acl.sd_size = len;

	rc = ksmbd_gen_sd_hash(conn, acl.sd_buf, acl.sd_size, acl.hash);
	if (rc) {
		pr_err("failed to generate hash for ndr acl\n");
		return rc;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	smb_acl = ksmbd_vfs_make_xattr_posix_acl(idmap, inode,
#else
	smb_acl = ksmbd_vfs_make_xattr_posix_acl(user_ns, inode,
#endif
						 ACL_TYPE_ACCESS);
	if (S_ISDIR(inode->i_mode))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		def_smb_acl = ksmbd_vfs_make_xattr_posix_acl(idmap, inode,
#else
		def_smb_acl = ksmbd_vfs_make_xattr_posix_acl(user_ns, inode,
#endif
							     ACL_TYPE_DEFAULT);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = ndr_encode_posix_acl(&acl_ndr, idmap, inode,
#else
	rc = ndr_encode_posix_acl(&acl_ndr, user_ns, inode,
#endif
				  smb_acl, def_smb_acl);
	if (rc) {
		pr_err("failed to encode ndr to posix acl\n");
		goto out;
	}

	rc = ksmbd_gen_sd_hash(conn, acl_ndr.data, acl_ndr.offset,
			       acl.posix_acl_hash);
	if (rc) {
		pr_err("failed to generate hash for ndr acl\n");
		goto out;
	}

	rc = ndr_encode_v4_ntacl(&sd_ndr, &acl);
	if (rc) {
		pr_err("failed to encode ndr to posix acl\n");
		goto out;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = ksmbd_vfs_setxattr(idmap, dentry,
#else
	rc = ksmbd_vfs_setxattr(user_ns, dentry,
#endif
				XATTR_NAME_SD, sd_ndr.data,
				sd_ndr.offset, 0);
	if (rc < 0)
		pr_err("Failed to store XATTR ntacl :%d\n", rc);

	kfree(sd_ndr.data);
out:
	kfree(acl_ndr.data);
	kfree(smb_acl);
	kfree(def_smb_acl);
	return rc;
}

int ksmbd_vfs_get_sd_xattr(struct ksmbd_conn *conn,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			   struct mnt_idmap *idmap,
#else
			   struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   struct smb_ntsd **pntsd)
{
	int rc;
	struct ndr n;
	struct inode *inode = d_inode(dentry);
	struct ndr acl_ndr = {0};
	struct xattr_ntacl acl;
	struct xattr_smb_acl *smb_acl = NULL, *def_smb_acl = NULL;
	__u8 cmp_hash[XATTR_SD_HASH_SIZE] = {0};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = ksmbd_vfs_getxattr(idmap, dentry, XATTR_NAME_SD, &n.data);
#else
	rc = ksmbd_vfs_getxattr(user_ns, dentry, XATTR_NAME_SD, &n.data);
#endif
	if (rc <= 0)
		return rc;

	n.length = rc;
	rc = ndr_decode_v4_ntacl(&n, &acl);
	if (rc)
		goto free_n_data;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	smb_acl = ksmbd_vfs_make_xattr_posix_acl(idmap, inode,
#else
	smb_acl = ksmbd_vfs_make_xattr_posix_acl(user_ns, inode,
#endif
						 ACL_TYPE_ACCESS);
	if (S_ISDIR(inode->i_mode))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		def_smb_acl = ksmbd_vfs_make_xattr_posix_acl(idmap, inode,
#else
		def_smb_acl = ksmbd_vfs_make_xattr_posix_acl(user_ns, inode,
#endif
							     ACL_TYPE_DEFAULT);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = ndr_encode_posix_acl(&acl_ndr, idmap, inode, smb_acl,
#else
	rc = ndr_encode_posix_acl(&acl_ndr, user_ns, inode, smb_acl,
#endif
				  def_smb_acl);
	if (rc) {
		pr_err("failed to encode ndr to posix acl\n");
		goto out_free;
	}

	rc = ksmbd_gen_sd_hash(conn, acl_ndr.data, acl_ndr.offset, cmp_hash);
	if (rc) {
		pr_err("failed to generate hash for ndr acl\n");
		goto out_free;
	}

	if (memcmp(cmp_hash, acl.posix_acl_hash, XATTR_SD_HASH_SIZE)) {
		pr_err("hash value diff\n");
		rc = -EINVAL;
		goto out_free;
	}

	*pntsd = acl.sd_buf;
	if (acl.sd_size < sizeof(struct smb_ntsd)) {
		pr_err("sd size is invalid\n");
		goto out_free;
	}

	(*pntsd)->osidoffset = cpu_to_le32(le32_to_cpu((*pntsd)->osidoffset) -
					   NDR_NTSD_OFFSETOF);
	(*pntsd)->gsidoffset = cpu_to_le32(le32_to_cpu((*pntsd)->gsidoffset) -
					   NDR_NTSD_OFFSETOF);
	(*pntsd)->dacloffset = cpu_to_le32(le32_to_cpu((*pntsd)->dacloffset) -
					   NDR_NTSD_OFFSETOF);

	rc = acl.sd_size;
out_free:
	kfree(acl_ndr.data);
	kfree(smb_acl);
	kfree(def_smb_acl);
	if (rc < 0) {
		kfree(acl.sd_buf);
		*pntsd = NULL;
	}

free_n_data:
	kfree(n.data);
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_set_dos_attrib_xattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_set_dos_attrib_xattr(struct user_namespace *user_ns,
#endif
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da)
{
	struct ndr n;
	int err;

	err = ndr_encode_dos_attr(&n, da);
	if (err)
		return err;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = ksmbd_vfs_setxattr(idmap, dentry, XATTR_NAME_DOS_ATTRIBUTE,
#else
	err = ksmbd_vfs_setxattr(user_ns, dentry, XATTR_NAME_DOS_ATTRIBUTE,
#endif
				 (void *)n.data, n.offset, 0);
	if (err)
		ksmbd_debug(SMB, "failed to store dos attribute in xattr\n");
	kfree(n.data);

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_get_dos_attrib_xattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_get_dos_attrib_xattr(struct user_namespace *user_ns,
#endif
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da)
{
	struct ndr n;
	int err;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = ksmbd_vfs_getxattr(idmap, dentry, XATTR_NAME_DOS_ATTRIBUTE,
#else
	err = ksmbd_vfs_getxattr(user_ns, dentry, XATTR_NAME_DOS_ATTRIBUTE,
#endif
				 (char **)&n.data);
	if (err > 0) {
		n.length = err;
		if (ndr_decode_dos_attr(&n, da))
			err = -EINVAL;
		kfree(n.data);
	} else {
		ksmbd_debug(SMB, "failed to load dos attribute in xattr\n");
	}

	return err;
}

/**
 * ksmbd_vfs_set_fadvise() - convert smb IO caching options to linux options
 * @filp:	file pointer for IO
 * @options:	smb IO options
 */
void ksmbd_vfs_set_fadvise(struct file *filp, __le32 option)
{
	struct address_space *mapping;

	mapping = filp->f_mapping;

	if (!option || !mapping)
		return;

	if (option & FILE_WRITE_THROUGH_LE) {
		filp->f_flags |= O_SYNC;
	} else if (option & FILE_SEQUENTIAL_ONLY_LE) {
		filp->f_ra.ra_pages = inode_to_bdi(mapping->host)->ra_pages * 2;
		spin_lock(&filp->f_lock);
		filp->f_mode &= ~FMODE_RANDOM;
		spin_unlock(&filp->f_lock);
	} else if (option & FILE_RANDOM_ACCESS_LE) {
		spin_lock(&filp->f_lock);
		filp->f_mode |= FMODE_RANDOM;
		spin_unlock(&filp->f_lock);
	}
}

int ksmbd_vfs_zero_data(struct ksmbd_work *work, struct ksmbd_file *fp,
			loff_t off, loff_t len)
{
	smb_break_all_levII_oplock(work, fp, 1);
	if (fp->f_ci->m_fattr & ATTR_SPARSE_FILE_LE)
		return vfs_fallocate(fp->filp,
				     FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
				     off, len);

	return vfs_fallocate(fp->filp,
			     FALLOC_FL_ZERO_RANGE | FALLOC_FL_KEEP_SIZE,
			     off, len);
}

int ksmbd_vfs_fqar_lseek(struct ksmbd_file *fp, loff_t start, loff_t length,
			 struct file_allocated_range_buffer *ranges,
			 unsigned int in_count, unsigned int *out_count)
{
	struct file *f = fp->filp;
	struct inode *inode = file_inode(fp->filp);
	loff_t maxbytes = (u64)inode->i_sb->s_maxbytes, end;
	loff_t extent_start, extent_end;
	int ret = 0;

	if (start > maxbytes)
		return -EFBIG;

	if (!in_count)
		return 0;

	/*
	 * Shrink request scope to what the fs can actually handle.
	 */
	if (length > maxbytes || (maxbytes - length) < start)
		length = maxbytes - start;

	if (start + length > inode->i_size)
		length = inode->i_size - start;

	*out_count = 0;
	end = start + length;
	while (start < end && *out_count < in_count) {
		extent_start = vfs_llseek(f, start, SEEK_DATA);
		if (extent_start < 0) {
			if (extent_start != -ENXIO)
				ret = (int)extent_start;
			break;
		}

		if (extent_start >= end)
			break;

		extent_end = vfs_llseek(f, extent_start, SEEK_HOLE);
		if (extent_end < 0) {
			if (extent_end != -ENXIO)
				ret = (int)extent_end;
			break;
		} else if (extent_start >= extent_end) {
			break;
		}

		ranges[*out_count].file_offset = cpu_to_le64(extent_start);
		ranges[(*out_count)++].length =
			cpu_to_le64(min(extent_end, end) - extent_start);

		start = extent_end;
	}

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_xattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_remove_xattr(struct user_namespace *user_ns,
#endif
			   struct dentry *dentry, char *attr_name)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	return vfs_removexattr(idmap, dentry, attr_name);
#else
	return vfs_removexattr(user_ns, dentry, attr_name);
#endif
#else
	return vfs_removexattr(dentry, attr_name);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_unlink(struct mnt_idmap *idmap,
		     struct dentry *dir, struct dentry *dentry)
#else
int ksmbd_vfs_unlink(struct user_namespace *user_ns,
		     struct dentry *dir, struct dentry *dentry)
#endif
{
	int err = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	err = ksmbd_vfs_lock_parent(idmap, dir, dentry);
#else
	err = ksmbd_vfs_lock_parent(user_ns, dir, dentry);
#endif
	if (err)
		return err;

	dget(dentry);
	if (S_ISDIR(d_inode(dentry)->i_mode))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		err = vfs_rmdir(idmap, d_inode(dir), dentry);
	else
		err = vfs_unlink(idmap, d_inode(dir), dentry, NULL);
#else
		err = vfs_rmdir(user_ns, d_inode(dir), dentry);
	else
		err = vfs_unlink(user_ns, d_inode(dir), dentry, NULL);
#endif
#else
		err = vfs_rmdir(d_inode(dir), dentry);
	else
		err = vfs_unlink(d_inode(dir), dentry, NULL);
#endif

	dput(dentry);
	inode_unlock(d_inode(dir));
	if (err)
		ksmbd_debug(VFS, "failed to delete, err %d\n", err);

	return err;
}

#ifdef CONFIG_SMB_INSECURE_SERVER
/**
 * ksmbd_vfs_dentry_open() - open a dentry and provide fid for it
 * @work:	smb work ptr
 * @path:	path of dentry to be opened
 * @flags:	open flags
 * @ret_id:	fid returned on this
 * @option:	file access pattern options for fadvise
 * @fexist:	file already present or not
 *
 * Return:	allocated struct ksmbd_file on success, otherwise error pointer
 */
struct ksmbd_file *ksmbd_vfs_dentry_open(struct ksmbd_work *work,
					 const struct path *path, int flags,
					 __le32 option, int fexist)
{
	struct file *filp;
	int err = 0;
	struct ksmbd_file *fp = NULL;

	filp = dentry_open(path, flags | O_LARGEFILE, current_cred());
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		pr_err("dentry open failed, err %d\n", err);
		return ERR_PTR(err);
	}

	ksmbd_vfs_set_fadvise(filp, option);

	fp = ksmbd_open_fd(work, filp);
	if (IS_ERR(fp)) {
		fput(filp);
		err = PTR_ERR(fp);
		pr_err("id insert failed\n");
		goto err_out;
	}

	if (flags & O_TRUNC) {
		if (fexist)
			smb_break_all_oplock(work, fp);
		err = vfs_truncate((struct path *)path, 0);
		if (err)
			goto err_out;
	}
	return fp;

err_out:
	if (!IS_ERR(fp))
		ksmbd_close_fd(work, fp->volatile_id);
	if (err) {
		fp = ERR_PTR(err);
		pr_err("err : %d\n", err);
	}
	return fp;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static bool __dir_empty(struct dir_context *ctx, const char *name, int namlen,
#else
static int __dir_empty(struct dir_context *ctx, const char *name, int namlen,
#endif
		       loff_t offset, u64 ino, unsigned int d_type)
{
	struct ksmbd_readdir_data *buf;

	buf = container_of(ctx, struct ksmbd_readdir_data, ctx);
	buf->dirent_count++;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	return buf->dirent_count <= 2;
#else
	if (buf->dirent_count > 2)
		return -ENOTEMPTY;
	return 0;
#endif
}

/**
 * ksmbd_vfs_empty_dir() - check for empty directory
 * @fp:	ksmbd file pointer
 *
 * Return:	true if directory empty, otherwise false
 */
int ksmbd_vfs_empty_dir(struct ksmbd_file *fp)
{
	int err;
	struct ksmbd_readdir_data readdir_data;

	memset(&readdir_data, 0, sizeof(struct ksmbd_readdir_data));

	set_ctx_actor(&readdir_data.ctx, __dir_empty);
	readdir_data.dirent_count = 0;

	err = iterate_dir(fp->filp, &readdir_data.ctx);
	if (readdir_data.dirent_count > 2)
		err = -ENOTEMPTY;
	else
		err = 0;
	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static bool __caseless_lookup(struct dir_context *ctx, const char *name,
#else
static int __caseless_lookup(struct dir_context *ctx, const char *name,
#endif
			     int namlen, loff_t offset, u64 ino,
			     unsigned int d_type)
{
	struct ksmbd_readdir_data *buf;
	int cmp = -EINVAL;

	buf = container_of(ctx, struct ksmbd_readdir_data, ctx);

	if (buf->used != namlen)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return true;
#else
		return 0;
#endif
	if (IS_ENABLED(CONFIG_UNICODE) && buf->um) {
		const struct qstr q_buf = {.name = buf->private,
					   .len = buf->used};
		const struct qstr q_name = {.name = name,
					    .len = namlen};

		cmp = utf8_strncasecmp(buf->um, &q_buf, &q_name);
	}
	if (cmp < 0)
		cmp = strncasecmp((char *)buf->private, name, namlen);
	if (!cmp) {
		memcpy((char *)buf->private, name, namlen);
		buf->dirent_count = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return false;
#else
		return -EEXIST;
#endif
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	return true;
#else
	return 0;
#endif
}

/**
 * ksmbd_vfs_lookup_in_dir() - lookup a file in a directory
 * @dir:	path info
 * @name:	filename to lookup
 * @namelen:	filename length
 *
 * Return:	0 on success, otherwise error
 */
static int ksmbd_vfs_lookup_in_dir(const struct path *dir, char *name,
				   size_t namelen, struct unicode_map *um)
{
	int ret;
	struct file *dfilp;
	int flags = O_RDONLY | O_LARGEFILE;
	struct ksmbd_readdir_data readdir_data = {
		.ctx.actor	= __caseless_lookup,
		.private	= name,
		.used		= namelen,
		.dirent_count	= 0,
		.um		= um,
	};

	dfilp = dentry_open(dir, flags, current_cred());
	if (IS_ERR(dfilp))
		return PTR_ERR(dfilp);

	ret = iterate_dir(dfilp, &readdir_data.ctx);
	if (readdir_data.dirent_count > 0)
		ret = 0;
	fput(dfilp);
	return ret;
}

/**
 * ksmbd_vfs_kern_path() - lookup a file and get path info
 * @name:	file path that is relative to share
 * @flags:	lookup flags
 * @path:	if lookup succeed, return path info
 * @caseless:	caseless filename lookup
 *
 * Return:	0 on success, otherwise error
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
int ksmbd_vfs_kern_path(struct ksmbd_work *work, char *name,
			unsigned int flags, struct path *path, bool caseless)
{
	struct ksmbd_share_config *share_conf = work->tcon->share_conf;
	int err;

	flags |= LOOKUP_BENEATH;
	err = vfs_path_lookup(share_conf->vfs_path.dentry,
			      share_conf->vfs_path.mnt,
			      name,
			      flags,
			      path);
	if (!err)
		return 0;

	if (caseless) {
		char *filepath;
		struct path parent;
		size_t path_len, remain_len;

		filepath = kstrdup(name, GFP_KERNEL);
		if (!filepath)
			return -ENOMEM;

		path_len = strlen(filepath);
		remain_len = path_len;

		parent = share_conf->vfs_path;
		path_get(&parent);

		while (d_can_lookup(parent.dentry)) {
			char *filename = filepath + path_len - remain_len;
			char *next = strchrnul(filename, '/');
			size_t filename_len = next - filename;
			bool is_last = !next[0];

			if (filename_len == 0)
				break;

			err = ksmbd_vfs_lookup_in_dir(&parent, filename,
						      filename_len,
						      work->conn->um);
			path_put(&parent);
			if (err)
				goto out;

			next[0] = '\0';

			err = vfs_path_lookup(share_conf->vfs_path.dentry,
					      share_conf->vfs_path.mnt,
					      filepath,
					      flags,
					      &parent);
			if (err)
				goto out;
			else if (is_last) {
				*path = parent;
				goto out;
			}

			next[0] = '/';
			remain_len -= filename_len + 1;
		}

		path_put(&parent);
		err = -EINVAL;
out:
		kfree(filepath);
	}
	return err;
}
#else
int ksmbd_vfs_kern_path(struct ksmbd_work *work, char *name,
			unsigned int flags, struct path *path, bool caseless)
{
	char *abs_name;
	int err;

	abs_name = convert_to_unix_name(work->tcon->share_conf, name);
	if (IS_ERR(abs_name))
		return PTR_ERR(abs_name);

	err = kern_path(abs_name, flags, path);
	if (!err) {
		err = 0;
		goto free_abs_name;
	}

	if (caseless) {
		char *filepath;
		struct path parent;
		size_t path_len, remain_len;

		filepath = kstrdup(abs_name, GFP_KERNEL);
		if (!filepath) {
			err = -ENOMEM;
			goto free_abs_name;
		}

		path_len = strlen(filepath);
		remain_len = path_len - 1;

		err = kern_path("/", flags, &parent);
		if (err)
			goto out;

		while (d_can_lookup(parent.dentry)) {
			char *filename = filepath + path_len - remain_len;
			char *next = strchrnul(filename, '/');
			size_t filename_len = next - filename;
			bool is_last = !next[0];

			if (filename_len == 0)
				break;

			err = ksmbd_vfs_lookup_in_dir(&parent, filename,
						      filename_len,
						      work->conn->um);
			if (err) {
				path_put(&parent);
				goto out;
			}

			path_put(&parent);
			next[0] = '\0';

			err = kern_path(filepath, flags, &parent);
			if (err)
				goto out;

			if (is_last) {
				path->mnt = parent.mnt;
				path->dentry = parent.dentry;
				goto out;
			}

			next[0] = '/';
			remain_len -= filename_len + 1;
		}

		path_put(&parent);
		err = -EINVAL;
out:
		kfree(filepath);
	}

free_abs_name:
	kfree(abs_name);
	return err;
}
#endif

/**
 * ksmbd_vfs_init_kstat() - convert unix stat information to smb stat format
 * @p:          destination buffer
 * @ksmbd_kstat:      ksmbd kstat wrapper
 */
void *ksmbd_vfs_init_kstat(char **p, struct ksmbd_kstat *ksmbd_kstat)
{
	struct file_directory_info *info = (struct file_directory_info *)(*p);
	struct kstat *kstat = ksmbd_kstat->kstat;
	u64 time;

	info->FileIndex = 0;
	info->CreationTime = cpu_to_le64(ksmbd_kstat->create_time);
	time = ksmbd_UnixTimeToNT(kstat->atime);
	info->LastAccessTime = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(kstat->mtime);
	info->LastWriteTime = cpu_to_le64(time);
	time = ksmbd_UnixTimeToNT(kstat->ctime);
	info->ChangeTime = cpu_to_le64(time);

	if (ksmbd_kstat->file_attributes & ATTR_DIRECTORY_LE) {
		info->EndOfFile = 0;
		info->AllocationSize = 0;
	} else {
		info->EndOfFile = cpu_to_le64(kstat->size);
		info->AllocationSize = cpu_to_le64(kstat->blocks << 9);
	}
	info->ExtFileAttributes = ksmbd_kstat->file_attributes;

	return info;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_fill_dentry_attrs(struct ksmbd_work *work,
				struct mnt_idmap *idmap,
				struct dentry *dentry,
				struct ksmbd_kstat *ksmbd_kstat)
#else
int ksmbd_vfs_fill_dentry_attrs(struct ksmbd_work *work,
				struct user_namespace *user_ns,
				struct dentry *dentry,
				struct ksmbd_kstat *ksmbd_kstat)
#endif
{
	u64 time;
	int rc;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	generic_fillattr(idmap, d_inode(dentry), ksmbd_kstat->kstat);
#else
	generic_fillattr(user_ns, d_inode(dentry), ksmbd_kstat->kstat);
#endif
#else
	generic_fillattr(d_inode(dentry), ksmbd_kstat->kstat);
#endif

	time = ksmbd_UnixTimeToNT(ksmbd_kstat->kstat->ctime);
	ksmbd_kstat->create_time = time;

	/*
	 * set default value for the case that store dos attributes is not yes
	 * or that acl is disable in server's filesystem and the config is yes.
	 */
	if (S_ISDIR(ksmbd_kstat->kstat->mode))
		ksmbd_kstat->file_attributes = ATTR_DIRECTORY_LE;
	else
		ksmbd_kstat->file_attributes = ATTR_ARCHIVE_LE;

	if (test_share_config_flag(work->tcon->share_conf,
				   KSMBD_SHARE_FLAG_STORE_DOS_ATTRS)) {
		struct xattr_dos_attrib da;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = ksmbd_vfs_get_dos_attrib_xattr(idmap, dentry, &da);
#else
		rc = ksmbd_vfs_get_dos_attrib_xattr(user_ns, dentry, &da);
#endif
		if (rc > 0) {
			ksmbd_kstat->file_attributes = cpu_to_le32(da.attr);
			ksmbd_kstat->create_time = da.create_time;
		} else {
			ksmbd_debug(VFS, "fail to load dos attribute.\n");
		}
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
ssize_t ksmbd_vfs_casexattr_len(struct mnt_idmap *idmap,
#else
ssize_t ksmbd_vfs_casexattr_len(struct user_namespace *user_ns,
#endif
				struct dentry *dentry, char *attr_name,
				int attr_name_len)
{
	char *name, *xattr_list = NULL;
	ssize_t value_len = -ENOENT, xattr_list_len;

	xattr_list_len = ksmbd_vfs_listxattr(dentry, &xattr_list);
	if (xattr_list_len <= 0)
		goto out;

	for (name = xattr_list; name - xattr_list < xattr_list_len;
			name += strlen(name) + 1) {
		ksmbd_debug(VFS, "%s, len %zd\n", name, strlen(name));
		if (strncasecmp(attr_name, name, attr_name_len))
			continue;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		value_len = ksmbd_vfs_xattr_len(idmap, dentry, name);
#else
		value_len = ksmbd_vfs_xattr_len(user_ns, dentry, name);
#endif
		break;
	}

out:
	kvfree(xattr_list);
	return value_len;
}

int ksmbd_vfs_xattr_stream_name(char *stream_name, char **xattr_stream_name,
				size_t *xattr_stream_name_size, int s_type)
{
	char *type, *buf;

	if (s_type == DIR_STREAM)
		type = ":$INDEX_ALLOCATION";
	else
		type = ":$DATA";

	buf = kasprintf(GFP_KERNEL, "%s%s%s",
			XATTR_NAME_STREAM, stream_name,	type);
	if (!buf)
		return -ENOMEM;

	*xattr_stream_name = buf;
	*xattr_stream_name_size = strlen(buf) + 1;

	return 0;
}

int ksmbd_vfs_copy_file_ranges(struct ksmbd_work *work,
			       struct ksmbd_file *src_fp,
			       struct ksmbd_file *dst_fp,
			       struct srv_copychunk *chunks,
			       unsigned int chunk_count,
			       unsigned int *chunk_count_written,
			       unsigned int *chunk_size_written,
			       loff_t *total_size_written)
{
	unsigned int i;
	loff_t src_off, dst_off, src_file_size;
	size_t len;
	int ret;

	*chunk_count_written = 0;
	*chunk_size_written = 0;
	*total_size_written = 0;

	if (!(src_fp->daccess & (FILE_READ_DATA_LE | FILE_EXECUTE_LE))) {
		pr_err("no right to read(%pD)\n", src_fp->filp);
		return -EACCES;
	}
	if (!(dst_fp->daccess & (FILE_WRITE_DATA_LE | FILE_APPEND_DATA_LE))) {
		pr_err("no right to write(%pD)\n", dst_fp->filp);
		return -EACCES;
	}

	if (ksmbd_stream_fd(src_fp) || ksmbd_stream_fd(dst_fp))
		return -EBADF;

	smb_break_all_levII_oplock(work, dst_fp, 1);

	if (!work->tcon->posix_extensions) {
		for (i = 0; i < chunk_count; i++) {
			src_off = le64_to_cpu(chunks[i].SourceOffset);
			dst_off = le64_to_cpu(chunks[i].TargetOffset);
			len = le32_to_cpu(chunks[i].Length);

			if (check_lock_range(src_fp->filp, src_off,
					     src_off + len - 1, READ))
				return -EAGAIN;
			if (check_lock_range(dst_fp->filp, dst_off,
					     dst_off + len - 1, WRITE))
				return -EAGAIN;
		}
	}

	src_file_size = i_size_read(file_inode(src_fp->filp));

	for (i = 0; i < chunk_count; i++) {
		src_off = le64_to_cpu(chunks[i].SourceOffset);
		dst_off = le64_to_cpu(chunks[i].TargetOffset);
		len = le32_to_cpu(chunks[i].Length);

		if (src_off + len > src_file_size)
			return -E2BIG;

		ret = vfs_copy_file_range(src_fp->filp, src_off,
					  dst_fp->filp, dst_off, len, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0)
		if (ret == -EOPNOTSUPP || ret == -EXDEV)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
			ret = vfs_copy_file_range(src_fp->filp, src_off,
						  dst_fp->filp, dst_off, len,
						  COPY_FILE_SPLICE);
#else
			ret = generic_copy_file_range(src_fp->filp, src_off,
						      dst_fp->filp, dst_off,
						      len, 0);
#endif
#endif
		if (ret < 0)
			return ret;

		*chunk_count_written += 1;
		*total_size_written += ret;
	}
	return 0;
}

void ksmbd_vfs_posix_lock_wait(struct file_lock *flock)
{
	wait_event(flock->fl_wait, !flock->fl_blocker);
}

int ksmbd_vfs_posix_lock_wait_timeout(struct file_lock *flock, long timeout)
{
	return wait_event_interruptible_timeout(flock->fl_wait,
						!flock->fl_blocker,
						timeout);
}

void ksmbd_vfs_posix_lock_unblock(struct file_lock *flock)
{
	locks_delete_block(flock);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_set_init_posix_acl(struct mnt_idmap *idmap,
				 struct dentry *dentry)
#else
int ksmbd_vfs_set_init_posix_acl(struct user_namespace *user_ns,
				 struct dentry *dentry)
#endif
{
	struct posix_acl_state acl_state;
	struct posix_acl *acls;
	struct inode *inode = d_inode(dentry);
	int rc;

	if (!IS_ENABLED(CONFIG_FS_POSIX_ACL))
		return -EOPNOTSUPP;

	ksmbd_debug(SMB, "Set posix acls\n");
	rc = init_acl_state(&acl_state, 1);
	if (rc)
		return rc;

	/* Set default owner group */
	acl_state.owner.allow = (inode->i_mode & 0700) >> 6;
	acl_state.group.allow = (inode->i_mode & 0070) >> 3;
	acl_state.other.allow = inode->i_mode & 0007;
	acl_state.users->aces[acl_state.users->n].uid = inode->i_uid;
	acl_state.users->aces[acl_state.users->n++].perms.allow =
		acl_state.owner.allow;
	acl_state.groups->aces[acl_state.groups->n].gid = inode->i_gid;
	acl_state.groups->aces[acl_state.groups->n++].perms.allow =
		acl_state.group.allow;
	acl_state.mask.allow = 0x07;

	acls = posix_acl_alloc(6, GFP_KERNEL);
	if (!acls) {
		free_acl_state(&acl_state);
		return -ENOMEM;
	}
	posix_state_to_acl(&acl_state, acls->a_entries);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = set_posix_acl(idmap, dentry, ACL_TYPE_ACCESS, acls);
#else
	rc = set_posix_acl(user_ns, dentry, ACL_TYPE_ACCESS, acls);
#endif
#else
	rc = set_posix_acl(user_ns, inode, ACL_TYPE_ACCESS, acls);
#endif
#else
	rc = set_posix_acl(inode, ACL_TYPE_ACCESS, acls);
#endif
	if (rc < 0)
		ksmbd_debug(SMB, "Set posix acl(ACL_TYPE_ACCESS) failed, rc : %d\n",
			    rc);
	else if (S_ISDIR(inode->i_mode)) {
		posix_state_to_acl(&acl_state, acls->a_entries);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = set_posix_acl(idmap, dentry, ACL_TYPE_DEFAULT, acls);
#else
		rc = set_posix_acl(user_ns, dentry, ACL_TYPE_DEFAULT, acls);
#endif
#else
		rc = set_posix_acl(user_ns, inode, ACL_TYPE_DEFAULT, acls);
#endif
#else
		rc = set_posix_acl(inode, ACL_TYPE_DEFAULT, acls);
#endif
		if (rc < 0)
			ksmbd_debug(SMB, "Set posix acl(ACL_TYPE_DEFAULT) failed, rc : %d\n",
				    rc);
	}
	free_acl_state(&acl_state);
	posix_acl_release(acls);
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_inherit_posix_acl(struct mnt_idmap *idmap,
				struct dentry *dentry, struct inode *parent_inode)
#else
int ksmbd_vfs_inherit_posix_acl(struct user_namespace *user_ns,
				struct dentry *dentry, struct inode *parent_inode)
#endif
{
	struct posix_acl *acls;
	struct posix_acl_entry *pace;
	struct inode *inode = d_inode(dentry);
	int rc, i;

	if (!IS_ENABLED(CONFIG_FS_POSIX_ACL))
		return -EOPNOTSUPP;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
	acls = get_inode_acl(parent_inode, ACL_TYPE_DEFAULT);
#else
	acls = get_acl(parent_inode, ACL_TYPE_DEFAULT);
#endif
	if (!acls)
		return -ENOENT;
	pace = acls->a_entries;

	for (i = 0; i < acls->a_count; i++, pace++) {
		if (pace->e_tag == ACL_MASK) {
			pace->e_perm = 0x07;
			break;
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	rc = set_posix_acl(idmap, dentry, ACL_TYPE_ACCESS, acls);
#else
	rc = set_posix_acl(user_ns, dentry, ACL_TYPE_ACCESS, acls);
#endif
#else
	rc = set_posix_acl(user_ns, inode, ACL_TYPE_ACCESS, acls);
#endif
#else
	rc = set_posix_acl(inode, ACL_TYPE_ACCESS, acls);
#endif
	if (rc < 0)
		ksmbd_debug(SMB, "Set posix acl(ACL_TYPE_ACCESS) failed, rc : %d\n",
			    rc);
	if (S_ISDIR(inode->i_mode)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
		rc = set_posix_acl(idmap, dentry, ACL_TYPE_DEFAULT,
				   acls);
#else
		rc = set_posix_acl(user_ns, dentry, ACL_TYPE_DEFAULT,
				   acls);
#endif
#else
		rc = set_posix_acl(user_ns, inode, ACL_TYPE_DEFAULT,
				   acls);
#endif
#else
		rc = set_posix_acl(inode, ACL_TYPE_DEFAULT, acls);
#endif
		if (rc < 0)
			ksmbd_debug(SMB, "Set posix acl(ACL_TYPE_DEFAULT) failed, rc : %d\n",
				    rc);
	}
	posix_acl_release(acls);
	return rc;
}
