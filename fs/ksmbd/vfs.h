/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *   Copyright (C) 2016 Namjae Jeon <linkinjeon@kernel.org>
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#ifndef __KSMBD_VFS_H__
#define __KSMBD_VFS_H__

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <uapi/linux/xattr.h>
#include <linux/posix_acl.h>
#include <linux/unicode.h>

#include "smbacl.h"
#include "xattr.h"

/*
 * Enumeration for stream type.
 */
enum {
	DATA_STREAM	= 1,	/* type $DATA */
	DIR_STREAM		/* type $INDEX_ALLOCATION */
};

/* CreateOptions */
/* Flag is set, it must not be a file , valid for directory only */
#define FILE_DIRECTORY_FILE_LE			cpu_to_le32(0x00000001)
#define FILE_WRITE_THROUGH_LE			cpu_to_le32(0x00000002)
#define FILE_SEQUENTIAL_ONLY_LE			cpu_to_le32(0x00000004)

/* Should not buffer on server*/
#define FILE_NO_INTERMEDIATE_BUFFERING_LE	cpu_to_le32(0x00000008)
/* MBZ */
#define FILE_SYNCHRONOUS_IO_ALERT_LE		cpu_to_le32(0x00000010)
/* MBZ */
#define FILE_SYNCHRONOUS_IO_NONALERT_LE		cpu_to_le32(0x00000020)

/* Flaf must not be set for directory */
#define FILE_NON_DIRECTORY_FILE_LE		cpu_to_le32(0x00000040)

/* Should be zero */
#define CREATE_TREE_CONNECTION			cpu_to_le32(0x00000080)
#define FILE_COMPLETE_IF_OPLOCKED_LE		cpu_to_le32(0x00000100)
#define FILE_NO_EA_KNOWLEDGE_LE			cpu_to_le32(0x00000200)
#define FILE_OPEN_REMOTE_INSTANCE		cpu_to_le32(0x00000400)

/**
 * Doc says this is obsolete "open for recovery" flag should be zero
 * in any case.
 */
#define CREATE_OPEN_FOR_RECOVERY		cpu_to_le32(0x00000400)
#define FILE_RANDOM_ACCESS_LE			cpu_to_le32(0x00000800)
#define FILE_DELETE_ON_CLOSE_LE			cpu_to_le32(0x00001000)
#define FILE_OPEN_BY_FILE_ID_LE			cpu_to_le32(0x00002000)
#define FILE_OPEN_FOR_BACKUP_INTENT_LE		cpu_to_le32(0x00004000)
#define FILE_NO_COMPRESSION_LE			cpu_to_le32(0x00008000)

/* Should be zero*/
#define FILE_OPEN_REQUIRING_OPLOCK		cpu_to_le32(0x00010000)
#define FILE_DISALLOW_EXCLUSIVE			cpu_to_le32(0x00020000)
#define FILE_RESERVE_OPFILTER_LE		cpu_to_le32(0x00100000)
#define FILE_OPEN_REPARSE_POINT_LE		cpu_to_le32(0x00200000)
#define FILE_OPEN_NO_RECALL_LE			cpu_to_le32(0x00400000)

/* Should be zero */
#define FILE_OPEN_FOR_FREE_SPACE_QUERY_LE	cpu_to_le32(0x00800000)
#define CREATE_OPTIONS_MASK			cpu_to_le32(0x00FFFFFF)
#define CREATE_OPTION_READONLY			0x10000000
/* system. NB not sent over wire */
#define CREATE_OPTION_SPECIAL			0x20000000

struct ksmbd_work;
struct ksmbd_file;
struct ksmbd_conn;

struct ksmbd_dir_info {
	const char	*name;
#ifdef CONFIG_SMB_INSECURE_SERVER
	char		*smb1_name;
#endif
	char		*wptr;
	char		*rptr;
	int		name_len;
	int		out_buf_len;
	int		num_entry;
	int		data_count;
	int		last_entry_offset;
	bool		hide_dot_file;
	int		flags;
	int		last_entry_off_align;
};

struct ksmbd_readdir_data {
	struct dir_context	ctx;
	union {
		void		*private;
		char		*dirent;
	};

	unsigned int		used;
	unsigned int		dirent_count;
	unsigned int		file_attr;
	struct unicode_map	*um;
};

/* ksmbd kstat wrapper to get valid create time when reading dir entry */
struct ksmbd_kstat {
	struct kstat		*kstat;
	unsigned long long	create_time;
	__le32			file_attributes;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
static inline struct user_namespace *mnt_user_ns(const struct vfsmount *mnt)
{
	return &init_user_ns;
}

static inline struct user_namespace *file_mnt_user_ns(struct file *file)
{
	return &init_user_ns;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_lock_parent(struct mnt_idmap *idmap, struct dentry *parent,
			  struct dentry *child);
int ksmbd_vfs_may_delete(struct mnt_idmap *idmap, struct dentry *dentry);
int ksmbd_vfs_query_maximal_access(struct mnt_idmap *idmap,
				   struct dentry *dentry, __le32 *daccess);
#else
int ksmbd_vfs_lock_parent(struct user_namespace *user_ns, struct dentry *parent,
			  struct dentry *child);
int ksmbd_vfs_may_delete(struct user_namespace *user_ns, struct dentry *dentry);
int ksmbd_vfs_query_maximal_access(struct user_namespace *user_ns,
				   struct dentry *dentry, __le32 *daccess);
#endif
int ksmbd_vfs_create(struct ksmbd_work *work, const char *name, umode_t mode);
int ksmbd_vfs_mkdir(struct ksmbd_work *work, const char *name, umode_t mode);
int ksmbd_vfs_read(struct ksmbd_work *work, struct ksmbd_file *fp,
		   size_t count, loff_t *pos);
int ksmbd_vfs_write(struct ksmbd_work *work, struct ksmbd_file *fp,
		    char *buf, size_t count, loff_t *pos, bool sync,
		    ssize_t *written);
int ksmbd_vfs_fsync(struct ksmbd_work *work, u64 fid, u64 p_id);
int ksmbd_vfs_remove_file(struct ksmbd_work *work, char *name);
int ksmbd_vfs_link(struct ksmbd_work *work,
		   const char *oldname, const char *newname);
int ksmbd_vfs_getattr(const struct path *path, struct kstat *stat);
#ifdef CONFIG_SMB_INSECURE_SERVER
int ksmbd_vfs_setattr(struct ksmbd_work *work, const char *name,
		      u64 fid, struct iattr *attrs);
int ksmbd_vfs_symlink(struct ksmbd_work *work,
		      const char *name, const char *symname);
int ksmbd_vfs_readlink(struct path *path, char *buf, int lenp);
int ksmbd_vfs_readdir_name(struct ksmbd_work *work,
			   struct user_namespace *user_ns,
			   struct ksmbd_kstat *ksmbd_kstat,
			   const char *de_name, int de_name_len,
			   const char *dir_path);
#endif
int ksmbd_vfs_fp_rename(struct ksmbd_work *work, struct ksmbd_file *fp,
			char *newname);
int ksmbd_vfs_truncate(struct ksmbd_work *work,
		       struct ksmbd_file *fp, loff_t size);
struct srv_copychunk;
int ksmbd_vfs_copy_file_ranges(struct ksmbd_work *work,
			       struct ksmbd_file *src_fp,
			       struct ksmbd_file *dst_fp,
			       struct srv_copychunk *chunks,
			       unsigned int chunk_count,
			       unsigned int *chunk_count_written,
			       unsigned int *chunk_size_written,
			       loff_t  *total_size_written);
struct ksmbd_file *ksmbd_vfs_dentry_open(struct ksmbd_work *work,
					 const struct path *path, int flags,
					 __le32 option, int fexist);
ssize_t ksmbd_vfs_listxattr(struct dentry *dentry, char **list);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
ssize_t ksmbd_vfs_getxattr(struct mnt_idmap *idmap,
#else
ssize_t ksmbd_vfs_getxattr(struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   char *xattr_name,
			   char **xattr_buf);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
ssize_t ksmbd_vfs_casexattr_len(struct mnt_idmap *idmap,
#else
ssize_t ksmbd_vfs_casexattr_len(struct user_namespace *user_ns,
#endif
				struct dentry *dentry, char *attr_name,
				int attr_name_len);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_setxattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_setxattr(struct user_namespace *user_ns,
#endif
		       struct dentry *dentry, const char *attr_name,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
		       void *attr_value, size_t attr_size, int flags);
#else
		       const void *attr_value, size_t attr_size, int flags);
#endif
int ksmbd_vfs_fsetxattr(struct ksmbd_work *work, const char *filename,
			const char *attr_name, const void *attr_value,
			size_t attr_size, int flags);
int ksmbd_vfs_xattr_stream_name(char *stream_name, char **xattr_stream_name,
				size_t *xattr_stream_name_size, int s_type);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_xattr(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_remove_xattr(struct user_namespace *user_ns,
#endif
			   struct dentry *dentry, char *attr_name);
int ksmbd_vfs_kern_path(struct ksmbd_work *work,
			char *name, unsigned int flags, struct path *path,
			bool caseless);
struct dentry *ksmbd_vfs_kern_path_create(struct ksmbd_work *work,
					  const char *name,
					  unsigned int flags,
					  struct path *path);
int ksmbd_vfs_empty_dir(struct ksmbd_file *fp);
void ksmbd_vfs_set_fadvise(struct file *filp, __le32 option);
int ksmbd_vfs_zero_data(struct ksmbd_work *work, struct ksmbd_file *fp,
			loff_t off, loff_t len);
struct file_allocated_range_buffer;
int ksmbd_vfs_fqar_lseek(struct ksmbd_file *fp, loff_t start, loff_t length,
			 struct file_allocated_range_buffer *ranges,
			 unsigned int in_count, unsigned int *out_count);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_unlink(struct mnt_idmap *idmap, struct dentry *dir,
		     struct dentry *dentry);
#else
int ksmbd_vfs_unlink(struct user_namespace *user_ns,
		     struct dentry *dir, struct dentry *dentry);
#endif
void *ksmbd_vfs_init_kstat(char **p, struct ksmbd_kstat *ksmbd_kstat);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_fill_dentry_attrs(struct ksmbd_work *work,
				struct mnt_idmap *idmap,
				struct dentry *dentry,
				struct ksmbd_kstat *ksmbd_kstat);
#else
int ksmbd_vfs_fill_dentry_attrs(struct ksmbd_work *work,
				struct user_namespace *user_ns,
				struct dentry *dentry,
				struct ksmbd_kstat *ksmbd_kstat);
#endif
void ksmbd_vfs_posix_lock_wait(struct file_lock *flock);
int ksmbd_vfs_posix_lock_wait_timeout(struct file_lock *flock, long timeout);
void ksmbd_vfs_posix_lock_unblock(struct file_lock *flock);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_acl_xattrs(struct mnt_idmap *idmap,
				struct dentry *dentry);
#else
int ksmbd_vfs_remove_acl_xattrs(struct user_namespace *user_ns,
				struct dentry *dentry);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_remove_sd_xattrs(struct mnt_idmap *idmap,
#else
int ksmbd_vfs_remove_sd_xattrs(struct user_namespace *user_ns,
#endif
			       struct dentry *dentry);
int ksmbd_vfs_set_sd_xattr(struct ksmbd_conn *conn,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			   struct mnt_idmap *idmap,
#else
			   struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   struct smb_ntsd *pntsd, int len);
int ksmbd_vfs_get_sd_xattr(struct ksmbd_conn *conn,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
			   struct mnt_idmap *idmap,
#else
			   struct user_namespace *user_ns,
#endif
			   struct dentry *dentry,
			   struct smb_ntsd **pntsd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_set_dos_attrib_xattr(struct mnt_idmap *idmap,
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da);
int ksmbd_vfs_get_dos_attrib_xattr(struct mnt_idmap *idmap,
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da);
#else
int ksmbd_vfs_set_dos_attrib_xattr(struct user_namespace *user_ns,
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da);
int ksmbd_vfs_get_dos_attrib_xattr(struct user_namespace *user_ns,
				   struct dentry *dentry,
				   struct xattr_dos_attrib *da);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
int ksmbd_vfs_set_init_posix_acl(struct mnt_idmap *idmap,
				 struct dentry *dentry);
int ksmbd_vfs_inherit_posix_acl(struct mnt_idmap *idmap,
				struct dentry *dentry,
				struct inode *parent_inode);
#else
int ksmbd_vfs_set_init_posix_acl(struct user_namespace *user_ns,
				 struct dentry *dentry);
int ksmbd_vfs_inherit_posix_acl(struct user_namespace *user_ns,
				struct dentry *dentry,
				struct inode *parent_inode);
#endif
#endif /* __KSMBD_VFS_H__ */
