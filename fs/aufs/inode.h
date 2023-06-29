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
 * inode operations
 */

#ifndef __AUFS_INODE_H__
#define __AUFS_INODE_H__

#ifdef __KERNEL__

#include <linux/fsnotify.h>
#include "rwsem.h"

struct vfsmount;

struct au_hnotify {
#ifdef CONFIG_AUFS_HNOTIFY
#ifdef CONFIG_AUFS_HFSNOTIFY
	/* never use fsnotify_add_vfsmount_mark() */
	struct fsnotify_mark		hn_mark;
#endif
	struct inode		*hn_aufs_inode;	/* no get/put */
	struct rcu_head		rcu;
#endif
} ____cacheline_aligned_in_smp;

struct au_hinode {
	struct inode		*hi_inode;
	aufs_bindex_t		hi_id;
#ifdef CONFIG_AUFS_HNOTIFY
	struct au_hnotify	*hi_notify;
#endif

	/* reference to the copied-up whiteout with get/put */
	struct dentry		*hi_whdentry;
};

/* ig_flags */
#define AuIG_HALF_REFRESHED		1
#define au_ig_ftest(flags, name)	((flags) & AuIG_##name)
#define au_ig_fset(flags, name) \
	do { (flags) |= AuIG_##name; } while (0)
#define au_ig_fclr(flags, name) \
	do { (flags) &= ~AuIG_##name; } while (0)

struct au_iigen {
	spinlock_t	ig_spin;
	__u32		ig_generation, ig_flags;
};

struct au_vdir;
struct au_iinfo {
	struct au_iigen		ii_generation;
	struct super_block	*ii_hsb1;	/* no get/put */

	struct au_rwsem		ii_rwsem;
	aufs_bindex_t		ii_btop, ii_bbot;
	__u32			ii_higen;
	struct au_hinode	*ii_hinode;
	struct au_vdir		*ii_vdir;
};

struct au_icntnr {
	struct au_iinfo		iinfo;
	struct inode		vfs_inode;
	struct hlist_bl_node	plink;
	struct rcu_head		rcu;
} ____cacheline_aligned_in_smp;

/* au_pin flags */
#define AuPin_DI_LOCKED		1
#define AuPin_MNT_WRITE		(1 << 1)
#define au_ftest_pin(flags, name)	((flags) & AuPin_##name)
#define au_fset_pin(flags, name) \
	do { (flags) |= AuPin_##name; } while (0)
#define au_fclr_pin(flags, name) \
	do { (flags) &= ~AuPin_##name; } while (0)

struct au_pin {
	/* input */
	struct dentry *dentry;
	unsigned int udba;
	unsigned char lsc_di, lsc_hi, flags;
	aufs_bindex_t bindex;

	/* output */
	struct dentry *parent;
	struct au_hinode *hdir;
	struct vfsmount *h_mnt;

	/* temporary unlock/relock for copyup */
	struct dentry *h_dentry, *h_parent;
	struct au_branch *br;
	struct task_struct *task;
};

void au_pin_hdir_unlock(struct au_pin *p);
int au_pin_hdir_lock(struct au_pin *p);
int au_pin_hdir_relock(struct au_pin *p);
void au_pin_hdir_acquire_nest(struct au_pin *p);
void au_pin_hdir_release(struct au_pin *p);

/* ---------------------------------------------------------------------- */

static inline struct au_iinfo *au_ii(struct inode *inode)
{
	BUG_ON(is_bad_inode(inode));
	return &(container_of(inode, struct au_icntnr, vfs_inode)->iinfo);
}

/* ---------------------------------------------------------------------- */

/* inode.c */
struct inode *au_igrab(struct inode *inode);
void au_refresh_iop(struct inode *inode, int force_getattr);
int au_refresh_hinode_self(struct inode *inode);
int au_refresh_hinode(struct inode *inode, struct dentry *dentry);
int au_ino(struct super_block *sb, aufs_bindex_t bindex, ino_t h_ino,
	   unsigned int d_type, ino_t *ino);
struct inode *au_new_inode(struct dentry *dentry, int must_new);
int au_test_ro(struct super_block *sb, aufs_bindex_t bindex,
	       struct inode *inode);
int au_test_h_perm(struct user_namespace *h_userns, struct inode *h_inode,
		   int mask);
int au_test_h_perm_sio(struct user_namespace *h_userns, struct inode *h_inode,
		       int mask);

static inline int au_wh_ino(struct super_block *sb, aufs_bindex_t bindex,
			    ino_t h_ino, unsigned int d_type, ino_t *ino)
{
#ifdef CONFIG_AUFS_SHWH
	return au_ino(sb, bindex, h_ino, d_type, ino);
#else
	return 0;
#endif
}

/* i_op.c */
enum {
	AuIop_SYMLINK,
	AuIop_DIR,
	AuIop_OTHER,
	AuIop_Last
};
extern struct inode_operations aufs_iop[AuIop_Last], /* not const */
	aufs_iop_nogetattr[AuIop_Last];

/* au_wr_dir flags */
#define AuWrDir_ADD_ENTRY	1
#define AuWrDir_ISDIR		(1 << 1)
#define AuWrDir_TMPFILE		(1 << 2)
#define au_ftest_wrdir(flags, name)	((flags) & AuWrDir_##name)
#define au_fset_wrdir(flags, name) \
	do { (flags) |= AuWrDir_##name; } while (0)
#define au_fclr_wrdir(flags, name) \
	do { (flags) &= ~AuWrDir_##name; } while (0)

struct au_wr_dir_args {
	aufs_bindex_t force_btgt;
	unsigned char flags;
};
int au_wr_dir(struct dentry *dentry, struct dentry *src_dentry,
	      struct au_wr_dir_args *args);

struct dentry *au_pinned_h_parent(struct au_pin *pin);
void au_pin_init(struct au_pin *pin, struct dentry *dentry,
		 aufs_bindex_t bindex, int lsc_di, int lsc_hi,
		 unsigned int udba, unsigned char flags);
int au_pin(struct au_pin *pin, struct dentry *dentry, aufs_bindex_t bindex,
	   unsigned int udba, unsigned char flags) __must_check;
int au_do_pin(struct au_pin *pin) __must_check;
void au_unpin(struct au_pin *pin);
int au_reval_for_attr(struct dentry *dentry, unsigned int sigen);

#define AuIcpup_DID_CPUP	1
#define au_ftest_icpup(flags, name)	((flags) & AuIcpup_##name)
#define au_fset_icpup(flags, name) \
	do { (flags) |= AuIcpup_##name; } while (0)
#define au_fclr_icpup(flags, name) \
	do { (flags) &= ~AuIcpup_##name; } while (0)

struct au_icpup_args {
	unsigned char flags;
	unsigned char pin_flags;
	aufs_bindex_t btgt;
	unsigned int udba;
	struct au_pin pin;
	struct path h_path;
	struct inode *h_inode;
};

int au_pin_and_icpup(struct dentry *dentry, struct iattr *ia,
		     struct au_icpup_args *a);

int au_h_path_getattr(struct dentry *dentry, struct inode *inode, int force,
		      struct path *h_path, int locked);

/* i_op_add.c */
int au_may_add(struct dentry *dentry, aufs_bindex_t bindex,
	       struct dentry *h_parent, int isdir);
int aufs_mknod(struct user_namespace *userns, struct inode *dir,
	       struct dentry *dentry, umode_t mode, dev_t dev);
int aufs_symlink(struct user_namespace *userns, struct inode *dir,
		 struct dentry *dentry, const char *symname);
int aufs_create(struct user_namespace *userns, struct inode *dir,
		struct dentry *dentry, umode_t mode, bool want_excl);
struct vfsub_aopen_args;
int au_aopen_or_create(struct inode *dir, struct dentry *dentry,
		       struct vfsub_aopen_args *args);
int aufs_tmpfile(struct user_namespace *userns, struct inode *dir,
		 struct file *file, umode_t mode);
int aufs_link(struct dentry *src_dentry, struct inode *dir,
	      struct dentry *dentry);
int aufs_mkdir(struct user_namespace *userns, struct inode *dir,
	       struct dentry *dentry, umode_t mode);

/* i_op_del.c */
int au_wr_dir_need_wh(struct dentry *dentry, int isdir, aufs_bindex_t *bcpup);
int au_may_del(struct dentry *dentry, aufs_bindex_t bindex,
	       struct dentry *h_parent, int isdir);
int aufs_unlink(struct inode *dir, struct dentry *dentry);
int aufs_rmdir(struct inode *dir, struct dentry *dentry);

/* i_op_ren.c */
int au_wbr(struct dentry *dentry, aufs_bindex_t btgt);
int aufs_rename(struct user_namespace *userns,
		struct inode *_src_dir, struct dentry *_src_dentry,
		struct inode *_dst_dir, struct dentry *_dst_dentry,
		unsigned int _flags);

/* iinfo.c */
struct inode *au_h_iptr(struct inode *inode, aufs_bindex_t bindex);
void au_hiput(struct au_hinode *hinode);
void au_set_hi_wh(struct inode *inode, aufs_bindex_t bindex,
		  struct dentry *h_wh);
unsigned int au_hi_flags(struct inode *inode, int isdir);

/* hinode flags */
#define AuHi_XINO	1
#define AuHi_HNOTIFY	(1 << 1)
#define au_ftest_hi(flags, name)	((flags) & AuHi_##name)
#define au_fset_hi(flags, name) \
	do { (flags) |= AuHi_##name; } while (0)
#define au_fclr_hi(flags, name) \
	do { (flags) &= ~AuHi_##name; } while (0)

#ifndef CONFIG_AUFS_HNOTIFY
#undef AuHi_HNOTIFY
#define AuHi_HNOTIFY	0
#endif

void au_set_h_iptr(struct inode *inode, aufs_bindex_t bindex,
		   struct inode *h_inode, unsigned int flags);

void au_update_iigen(struct inode *inode, int half);
void au_update_ibrange(struct inode *inode, int do_put_zero);

void au_icntnr_init_once(void *_c);
void au_hinode_init(struct au_hinode *hinode);
int au_iinfo_init(struct inode *inode);
void au_iinfo_fin(struct inode *inode);
int au_hinode_realloc(struct au_iinfo *iinfo, int nbr, int may_shrink);

#ifdef CONFIG_PROC_FS
/* plink.c */
int au_plink_maint(struct super_block *sb, int flags);
struct au_sbinfo;
void au_plink_maint_leave(struct au_sbinfo *sbinfo);
int au_plink_maint_enter(struct super_block *sb);
#ifdef CONFIG_AUFS_DEBUG
void au_plink_list(struct super_block *sb);
#else
AuStubVoid(au_plink_list, struct super_block *sb)
#endif
int au_plink_test(struct inode *inode);
struct dentry *au_plink_lkup(struct inode *inode, aufs_bindex_t bindex);
void au_plink_append(struct inode *inode, aufs_bindex_t bindex,
		     struct dentry *h_dentry);
void au_plink_put(struct super_block *sb, int verbose);
void au_plink_clean(struct super_block *sb, int verbose);
void au_plink_half_refresh(struct super_block *sb, aufs_bindex_t br_id);
#else
AuStubInt0(au_plink_maint, struct super_block *sb, int flags);
AuStubVoid(au_plink_maint_leave, struct au_sbinfo *sbinfo);
AuStubInt0(au_plink_maint_enter, struct super_block *sb);
AuStubVoid(au_plink_list, struct super_block *sb);
AuStubInt0(au_plink_test, struct inode *inode);
AuStub(struct dentry *, au_plink_lkup, return NULL,
       struct inode *inode, aufs_bindex_t bindex);
AuStubVoid(au_plink_append, struct inode *inode, aufs_bindex_t bindex,
	   struct dentry *h_dentry);
AuStubVoid(au_plink_put, struct super_block *sb, int verbose);
AuStubVoid(au_plink_clean, struct super_block *sb, int verbose);
AuStubVoid(au_plink_half_refresh, struct super_block *sb, aufs_bindex_t br_id);
#endif /* CONFIG_PROC_FS */

#ifdef CONFIG_AUFS_XATTR
/* xattr.c */
int au_cpup_xattr(struct path *h_dst, struct path *h_src, int ignore_flags,
		  unsigned int verbose);
ssize_t aufs_listxattr(struct dentry *dentry, char *list, size_t size);
void au_xattr_init(struct super_block *sb);
#else
AuStubInt0(au_cpup_xattr, struct path *h_dst, struct path *h_src,
	   int ignore_flags, unsigned int verbose);
AuStubVoid(au_xattr_init, struct super_block *sb);
#endif

#ifdef CONFIG_FS_POSIX_ACL
struct posix_acl *aufs_get_acl(struct inode *inode, int type, bool rcu);
int aufs_set_acl(struct user_namespace *userns, struct inode *inode,
		 struct posix_acl *acl, int type);
#endif

#if IS_ENABLED(CONFIG_AUFS_XATTR) || IS_ENABLED(CONFIG_FS_POSIX_ACL)
enum {
	AU_XATTR_SET,
	AU_ACL_SET
};

struct au_sxattr {
	int type;
	union {
		struct {
			const char	*name;
			const void	*value;
			size_t		size;
			int		flags;
		} set;
		struct {
			struct posix_acl *acl;
			int		type;
		} acl_set;
	} u;
};
ssize_t au_sxattr(struct dentry *dentry, struct inode *inode,
		  struct au_sxattr *arg);
#endif

/* ---------------------------------------------------------------------- */

/* lock subclass for iinfo */
enum {
	AuLsc_II_CHILD,		/* child first */
	AuLsc_II_CHILD2,	/* rename(2), link(2), and cpup at hnotify */
	AuLsc_II_CHILD3,	/* copyup dirs */
	AuLsc_II_PARENT,	/* see AuLsc_I_PARENT in vfsub.h */
	AuLsc_II_PARENT2,
	AuLsc_II_PARENT3,	/* copyup dirs */
	AuLsc_II_NEW_CHILD
};

/*
 * ii_read_lock_child, ii_write_lock_child,
 * ii_read_lock_child2, ii_write_lock_child2,
 * ii_read_lock_child3, ii_write_lock_child3,
 * ii_read_lock_parent, ii_write_lock_parent,
 * ii_read_lock_parent2, ii_write_lock_parent2,
 * ii_read_lock_parent3, ii_write_lock_parent3,
 * ii_read_lock_new_child, ii_write_lock_new_child,
 */
#define AuReadLockFunc(name, lsc) \
static inline void ii_read_lock_##name(struct inode *i) \
{ \
	au_rw_read_lock_nested(&au_ii(i)->ii_rwsem, AuLsc_II_##lsc); \
}

#define AuWriteLockFunc(name, lsc) \
static inline void ii_write_lock_##name(struct inode *i) \
{ \
	au_rw_write_lock_nested(&au_ii(i)->ii_rwsem, AuLsc_II_##lsc); \
}

#define AuRWLockFuncs(name, lsc) \
	AuReadLockFunc(name, lsc) \
	AuWriteLockFunc(name, lsc)

AuRWLockFuncs(child, CHILD);
AuRWLockFuncs(child2, CHILD2);
AuRWLockFuncs(child3, CHILD3);
AuRWLockFuncs(parent, PARENT);
AuRWLockFuncs(parent2, PARENT2);
AuRWLockFuncs(parent3, PARENT3);
AuRWLockFuncs(new_child, NEW_CHILD);

#undef AuReadLockFunc
#undef AuWriteLockFunc
#undef AuRWLockFuncs

#define ii_read_unlock(i)	au_rw_read_unlock(&au_ii(i)->ii_rwsem)
#define ii_write_unlock(i)	au_rw_write_unlock(&au_ii(i)->ii_rwsem)
#define ii_downgrade_lock(i)	au_rw_dgrade_lock(&au_ii(i)->ii_rwsem)

#define IiMustNoWaiters(i)	AuRwMustNoWaiters(&au_ii(i)->ii_rwsem)
#define IiMustAnyLock(i)	AuRwMustAnyLock(&au_ii(i)->ii_rwsem)
#define IiMustWriteLock(i)	AuRwMustWriteLock(&au_ii(i)->ii_rwsem)

/* ---------------------------------------------------------------------- */

static inline void au_icntnr_init(struct au_icntnr *c)
{
#ifdef CONFIG_AUFS_DEBUG
	c->vfs_inode.i_mode = 0;
#endif
}

static inline unsigned int au_iigen(struct inode *inode, unsigned int *igflags)
{
	unsigned int gen;
	struct au_iinfo *iinfo;
	struct au_iigen *iigen;

	iinfo = au_ii(inode);
	iigen = &iinfo->ii_generation;
	spin_lock(&iigen->ig_spin);
	if (igflags)
		*igflags = iigen->ig_flags;
	gen = iigen->ig_generation;
	spin_unlock(&iigen->ig_spin);

	return gen;
}

/* tiny test for inode number */
/* tmpfs generation is too rough */
static inline int au_test_higen(struct inode *inode, struct inode *h_inode)
{
	struct au_iinfo *iinfo;

	iinfo = au_ii(inode);
	AuRwMustAnyLock(&iinfo->ii_rwsem);
	return !(iinfo->ii_hsb1 == h_inode->i_sb
		 && iinfo->ii_higen == h_inode->i_generation);
}

static inline void au_iigen_dec(struct inode *inode)
{
	struct au_iinfo *iinfo;
	struct au_iigen *iigen;

	iinfo = au_ii(inode);
	iigen = &iinfo->ii_generation;
	spin_lock(&iigen->ig_spin);
	iigen->ig_generation--;
	spin_unlock(&iigen->ig_spin);
}

static inline int au_iigen_test(struct inode *inode, unsigned int sigen)
{
	int err;

	err = 0;
	if (unlikely(inode && au_iigen(inode, NULL) != sigen))
		err = -EIO;

	return err;
}

/* ---------------------------------------------------------------------- */

static inline struct au_hinode *au_hinode(struct au_iinfo *iinfo,
					  aufs_bindex_t bindex)
{
	return iinfo->ii_hinode + bindex;
}

static inline int au_is_bad_inode(struct inode *inode)
{
	return !!(is_bad_inode(inode) || !au_hinode(au_ii(inode), 0));
}

static inline aufs_bindex_t au_ii_br_id(struct inode *inode,
					aufs_bindex_t bindex)
{
	IiMustAnyLock(inode);
	return au_hinode(au_ii(inode), bindex)->hi_id;
}

static inline aufs_bindex_t au_ibtop(struct inode *inode)
{
	IiMustAnyLock(inode);
	return au_ii(inode)->ii_btop;
}

static inline aufs_bindex_t au_ibbot(struct inode *inode)
{
	IiMustAnyLock(inode);
	return au_ii(inode)->ii_bbot;
}

static inline struct au_vdir *au_ivdir(struct inode *inode)
{
	IiMustAnyLock(inode);
	return au_ii(inode)->ii_vdir;
}

static inline struct dentry *au_hi_wh(struct inode *inode, aufs_bindex_t bindex)
{
	IiMustAnyLock(inode);
	return au_hinode(au_ii(inode), bindex)->hi_whdentry;
}

static inline void au_set_ibtop(struct inode *inode, aufs_bindex_t bindex)
{
	IiMustWriteLock(inode);
	au_ii(inode)->ii_btop = bindex;
}

static inline void au_set_ibbot(struct inode *inode, aufs_bindex_t bindex)
{
	IiMustWriteLock(inode);
	au_ii(inode)->ii_bbot = bindex;
}

static inline void au_set_ivdir(struct inode *inode, struct au_vdir *vdir)
{
	IiMustWriteLock(inode);
	au_ii(inode)->ii_vdir = vdir;
}

static inline struct au_hinode *au_hi(struct inode *inode, aufs_bindex_t bindex)
{
	IiMustAnyLock(inode);
	return au_hinode(au_ii(inode), bindex);
}

/* ---------------------------------------------------------------------- */

static inline struct dentry *au_pinned_parent(struct au_pin *pin)
{
	if (pin)
		return pin->parent;
	return NULL;
}

static inline struct inode *au_pinned_h_dir(struct au_pin *pin)
{
	if (pin && pin->hdir)
		return pin->hdir->hi_inode;
	return NULL;
}

static inline struct au_hinode *au_pinned_hdir(struct au_pin *pin)
{
	if (pin)
		return pin->hdir;
	return NULL;
}

static inline void au_pin_set_dentry(struct au_pin *pin, struct dentry *dentry)
{
	if (pin)
		pin->dentry = dentry;
}

static inline void au_pin_set_parent_lflag(struct au_pin *pin,
					   unsigned char lflag)
{
	if (pin) {
		if (lflag)
			au_fset_pin(pin->flags, DI_LOCKED);
		else
			au_fclr_pin(pin->flags, DI_LOCKED);
	}
}

#if 0 /* reserved */
static inline void au_pin_set_parent(struct au_pin *pin, struct dentry *parent)
{
	if (pin) {
		dput(pin->parent);
		pin->parent = dget(parent);
	}
}
#endif

/* ---------------------------------------------------------------------- */

struct au_branch;
#ifdef CONFIG_AUFS_HNOTIFY
struct au_hnotify_op {
	void (*ctl)(struct au_hinode *hinode, int do_set);
	int (*alloc)(struct au_hinode *hinode);

	/*
	 * if it returns true, the caller should free hinode->hi_notify,
	 * otherwise ->free() frees it.
	 */
	int (*free)(struct au_hinode *hinode,
		    struct au_hnotify *hn) __must_check;

	void (*fin)(void);
	int (*init)(void);

	int (*reset_br)(unsigned int udba, struct au_branch *br, int perm);
	void (*fin_br)(struct au_branch *br);
	int (*init_br)(struct au_branch *br, int perm);
};

/* hnotify.c */
int au_hn_alloc(struct au_hinode *hinode, struct inode *inode);
void au_hn_free(struct au_hinode *hinode);
void au_hn_ctl(struct au_hinode *hinode, int do_set);
void au_hn_reset(struct inode *inode, unsigned int flags);
int au_hnotify(struct inode *h_dir, struct au_hnotify *hnotify, u32 mask,
	       const struct qstr *h_child_qstr, struct inode *h_child_inode);
int au_hnotify_reset_br(unsigned int udba, struct au_branch *br, int perm);
int au_hnotify_init_br(struct au_branch *br, int perm);
void au_hnotify_fin_br(struct au_branch *br);
int __init au_hnotify_init(void);
void au_hnotify_fin(void);

/* hfsnotify.c */
extern const struct au_hnotify_op au_hnotify_op;

static inline
void au_hn_init(struct au_hinode *hinode)
{
	hinode->hi_notify = NULL;
}

static inline struct au_hnotify *au_hn(struct au_hinode *hinode)
{
	return hinode->hi_notify;
}

#else
AuStub(int, au_hn_alloc, return -EOPNOTSUPP,
       struct au_hinode *hinode __maybe_unused,
       struct inode *inode __maybe_unused)
AuStub(struct au_hnotify *, au_hn, return NULL, struct au_hinode *hinode)
AuStubVoid(au_hn_free, struct au_hinode *hinode __maybe_unused)
AuStubVoid(au_hn_ctl, struct au_hinode *hinode __maybe_unused,
	   int do_set __maybe_unused)
AuStubVoid(au_hn_reset, struct inode *inode __maybe_unused,
	   unsigned int flags __maybe_unused)
AuStubInt0(au_hnotify_reset_br, unsigned int udba __maybe_unused,
	   struct au_branch *br __maybe_unused,
	   int perm __maybe_unused)
AuStubInt0(au_hnotify_init_br, struct au_branch *br __maybe_unused,
	   int perm __maybe_unused)
AuStubVoid(au_hnotify_fin_br, struct au_branch *br __maybe_unused)
AuStubInt0(__init au_hnotify_init, void)
AuStubVoid(au_hnotify_fin, void)
AuStubVoid(au_hn_init, struct au_hinode *hinode __maybe_unused)
#endif /* CONFIG_AUFS_HNOTIFY */

static inline void au_hn_suspend(struct au_hinode *hdir)
{
	au_hn_ctl(hdir, /*do_set*/0);
}

static inline void au_hn_resume(struct au_hinode *hdir)
{
	au_hn_ctl(hdir, /*do_set*/1);
}

static inline void au_hn_inode_lock(struct au_hinode *hdir)
{
	inode_lock(hdir->hi_inode);
	au_hn_suspend(hdir);
}

static inline void au_hn_inode_lock_nested(struct au_hinode *hdir,
					  unsigned int sc __maybe_unused)
{
	inode_lock_nested(hdir->hi_inode, sc);
	au_hn_suspend(hdir);
}

#if 0 /* unused */
#include "vfsub.h"
static inline void au_hn_inode_lock_shared_nested(struct au_hinode *hdir,
						  unsigned int sc)
{
	inode_lock_shared_nested(hdir->hi_inode, sc);
	au_hn_suspend(hdir);
}
#endif

static inline void au_hn_inode_unlock(struct au_hinode *hdir)
{
	au_hn_resume(hdir);
	inode_unlock(hdir->hi_inode);
}

#endif /* __KERNEL__ */
#endif /* __AUFS_INODE_H__ */
