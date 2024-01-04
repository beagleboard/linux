#ifndef COMPAT_H
#define COMPAT_H

struct dentry;
struct inode;
struct path;
struct xattr_dos_attrib;

/* linux vfs */
int compat_inode_permission(struct path *path, struct inode *inode, int mask);
void compat_generic_fillattr(struct path *path, u32 request_mask,
			     struct inode *inode, struct kstat *kstat);


/* ksmbd vfs */
ssize_t compat_ksmbd_vfs_getxattr(struct path *path, struct dentry *dentry,
				  char *xattr_name, char **xattr_buf);
int compat_ksmbd_vfs_get_dos_attrib_xattr(const struct path *path,
					  struct dentry *dentry,
					  struct xattr_dos_attrib *da);

int compat_ksmbd_vfs_set_dos_attrib_xattr(const struct path *path,
					  struct xattr_dos_attrib *da,
					  bool get_write);

#endif
