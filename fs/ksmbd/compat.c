#include <linux/version.h>
#include <linux/fs.h>
#include "compat.h"
#include "vfs.h"

#ifndef STATX_BASIC_STATS
#define STATX_BASIC_STATS 0x000007ffU
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
void compat_generic_fillattr(struct path *path, u32 request_mask,
			     struct inode *inode, struct kstat *kstat)
{
	generic_fillattr(mnt_idmap(path->mnt), request_mask,
			 inode, kstat);

}

int compat_ksmbd_vfs_get_dos_attrib_xattr(const struct path *path,
					  struct dentry *dentry,
					  struct xattr_dos_attrib *da)
{
	return ksmbd_vfs_get_dos_attrib_xattr(mnt_idmap(path->mnt), dentry,
					      da);
}

int compat_ksmbd_vfs_set_dos_attrib_xattr(const struct path *path,
					  struct xattr_dos_attrib *da,
					  bool get_write)
{
	return ksmbd_vfs_set_dos_attrib_xattr(mnt_idmap(path->mnt), path, da,
			get_write);
}

ssize_t compat_ksmbd_vfs_getxattr(struct path *path, struct dentry *dentry,
				  char *xattr_name, char **xattr_buf)
{
	return ksmbd_vfs_getxattr(mnt_idmap(path->mnt), dentry,
				  xattr_name, xattr_buf);
}
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int compat_inode_permission(struct path *path, struct inode *inode, int mask)
{
	return inode_permission(mnt_idmap(path->mnt), inode, mask);

}

void compat_generic_fillattr(struct path *path, u32 request_mask,
			     struct inode *inode, struct kstat *kstat)
{
	generic_fillattr(mnt_idmap(path->mnt), inode, kstat);

}

int compat_ksmbd_vfs_get_dos_attrib_xattr(const struct path *path,
					  struct dentry *dentry,
					  struct xattr_dos_attrib *da)
{
	return ksmbd_vfs_get_dos_attrib_xattr(mnt_idmap(path->mnt), dentry,
					      da);
}

int compat_ksmbd_vfs_set_dos_attrib_xattr(const struct path *path,
					  struct xattr_dos_attrib *da,
					  bool get_write)
{
	return ksmbd_vfs_set_dos_attrib_xattr(mnt_idmap(path->mnt), path, da,
			get_write);
}

ssize_t compat_ksmbd_vfs_getxattr(struct path *path, struct dentry *dentry,
				  char *xattr_name, char **xattr_buf)
{
	return ksmbd_vfs_getxattr(mnt_idmap(path->mnt), dentry,
				  xattr_name, xattr_buf);
}
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0)
int compat_inode_permission(struct path *path, struct inode *inode, int mask)
{
	return inode_permission(mnt_user_ns(path->mnt), inode, mask);

}
void compat_generic_fillattr(struct path *path, u32 request_mask,
			     struct inode *inode, struct kstat *kstat)
{
	generic_fillattr(mnt_user_ns(path->mnt), inode, kstat);

}

int compat_ksmbd_vfs_get_dos_attrib_xattr(const struct path *path,
					  struct dentry *dentry,
					  struct xattr_dos_attrib *da)
{
	return ksmbd_vfs_get_dos_attrib_xattr(mnt_user_ns(path->mnt), dentry,
					      da);
}

int compat_ksmbd_vfs_set_dos_attrib_xattr(const struct path *path,
					  struct xattr_dos_attrib *da,
					  bool get_write)
{
	return ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path->mnt), path, da,
			get_write);
}

ssize_t compat_ksmbd_vfs_getxattr(struct path *path, struct dentry *dentry,
				  char *xattr_name, char **xattr_buf)
{
	return ksmbd_vfs_getxattr(mnt_user_ns(path->mnt), dentry,
				  xattr_name, xattr_buf);
}
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
int compat_inode_permission(struct path *path, struct inode *inode, int mask)
{
	return inode_permission(inode, mask);

}
void compat_generic_fillattr(struct path *path, u32 request_mask,
			     struct inode *inode, struct kstat *kstat)
{
	generic_fillattr(inode, kstat);
}
int compat_ksmbd_vfs_get_dos_attrib_xattr(const struct path *path,
					  struct dentry *dentry,
					  struct xattr_dos_attrib *da)
{
	return ksmbd_vfs_get_dos_attrib_xattr(mnt_user_ns(path->mnt), dentry,
					      da);
}

int compat_ksmbd_vfs_set_dos_attrib_xattr(const struct path *path,
					  struct xattr_dos_attrib *da,
					  bool get_write)
{
	return ksmbd_vfs_set_dos_attrib_xattr(mnt_user_ns(path->mnt), path,
					      da, get_write);
}
ssize_t compat_ksmbd_vfs_getxattr(struct path *path, struct dentry *dentry,
				  char *xattr_name, char **xattr_buf)
{
	return ksmbd_vfs_getxattr(mnt_user_ns(path->mnt), dentry,
				  xattr_name, xattr_buf);
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0) */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0) */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0) */
