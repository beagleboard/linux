/* SPDX-License-Identifier: GPL-2.0 */
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
 * mount options/flags
 */

#ifndef __AUFS_OPTS_H__
#define __AUFS_OPTS_H__

#ifdef __KERNEL__

#include <linux/fs_parser.h>
#include <linux/namei.h>
#include <linux/path.h>

enum {
	Opt_br,
	Opt_add, Opt_del, Opt_mod, Opt_append, Opt_prepend,
	Opt_idel, Opt_imod,
	Opt_dirwh, Opt_rdcache, Opt_rdblk, Opt_rdhash,
	Opt_xino, Opt_noxino,
	Opt_trunc_xino, Opt_trunc_xino_v,
	Opt_trunc_xino_path, Opt_itrunc_xino,
	Opt_trunc_xib,
	Opt_shwh,
	Opt_plink, Opt_list_plink,
	Opt_udba,
	Opt_dio,
	Opt_diropq, Opt_diropq_a, Opt_diropq_w,
	Opt_warn_perm,
	Opt_wbr_copyup, Opt_wbr_create,
	Opt_fhsm_sec,
	Opt_verbose, Opt_noverbose,
	Opt_sum, Opt_wsum,
	Opt_dirperm1,
	Opt_dirren,
	Opt_acl,
	Opt_tail, Opt_ignore, Opt_ignore_silent, Opt_err
};

/* ---------------------------------------------------------------------- */

/* mount flags */
#define AuOpt_XINO		1		/* external inode number bitmap
						   and translation table */
#define AuOpt_TRUNC_XINO	(1 << 1)	/* truncate xino files */
#define AuOpt_UDBA_NONE		(1 << 2)	/* users direct branch access */
#define AuOpt_UDBA_REVAL	(1 << 3)
#define AuOpt_UDBA_HNOTIFY	(1 << 4)
#define AuOpt_SHWH		(1 << 5)	/* show whiteout */
#define AuOpt_PLINK		(1 << 6)	/* pseudo-link */
#define AuOpt_DIRPERM1		(1 << 7)	/* ignore the lower dir's perm
						   bits */
#define AuOpt_ALWAYS_DIROPQ	(1 << 9)	/* policy to creating diropq */
#define AuOpt_SUM		(1 << 10)	/* summation for statfs(2) */
#define AuOpt_SUM_W		(1 << 11)	/* unimplemented */
#define AuOpt_WARN_PERM		(1 << 12)	/* warn when add-branch */
#define AuOpt_VERBOSE		(1 << 13)	/* print the cause of error */
#define AuOpt_DIO		(1 << 14)	/* direct io */
#define AuOpt_DIRREN		(1 << 15)	/* directory rename */

#ifndef CONFIG_AUFS_HNOTIFY
#undef AuOpt_UDBA_HNOTIFY
#define AuOpt_UDBA_HNOTIFY	0
#endif
#ifndef CONFIG_AUFS_DIRREN
#undef AuOpt_DIRREN
#define AuOpt_DIRREN		0
#endif
#ifndef CONFIG_AUFS_SHWH
#undef AuOpt_SHWH
#define AuOpt_SHWH		0
#endif

#define AuOpt_Def	(AuOpt_XINO \
			 | AuOpt_UDBA_REVAL \
			 | AuOpt_PLINK \
			 /* | AuOpt_DIRPERM1 */ \
			 | AuOpt_WARN_PERM)
#define AuOptMask_UDBA	(AuOpt_UDBA_NONE \
			 | AuOpt_UDBA_REVAL \
			 | AuOpt_UDBA_HNOTIFY)

#define AuOpt_LkupDirFlags	(LOOKUP_FOLLOW | LOOKUP_DIRECTORY)

#define au_opt_test(flags, name)	(flags & AuOpt_##name)
#define au_opt_set(flags, name) do { \
	BUILD_BUG_ON(AuOpt_##name & AuOptMask_UDBA); \
	((flags) |= AuOpt_##name); \
} while (0)
#define au_opt_set_udba(flags, name) do { \
	(flags) &= ~AuOptMask_UDBA; \
	((flags) |= AuOpt_##name); \
} while (0)
#define au_opt_clr(flags, name) do { \
	((flags) &= ~AuOpt_##name); \
} while (0)

static inline unsigned int au_opts_plink(unsigned int mntflags)
{
#ifdef CONFIG_PROC_FS
	return mntflags;
#else
	return mntflags & ~AuOpt_PLINK;
#endif
}

/* ---------------------------------------------------------------------- */

/* policies to select one among multiple writable branches */
enum {
	AuWbrCreate_TDP,	/* top down parent */
	AuWbrCreate_RR,		/* round robin */
	AuWbrCreate_MFS,	/* most free space */
	AuWbrCreate_MFSV,	/* mfs with seconds */
	AuWbrCreate_MFSRR,	/* mfs then rr */
	AuWbrCreate_MFSRRV,	/* mfs then rr with seconds */
	AuWbrCreate_TDMFS,	/* top down regardless parent and mfs */
	AuWbrCreate_TDMFSV,	/* top down regardless parent and mfs */
	AuWbrCreate_PMFS,	/* parent and mfs */
	AuWbrCreate_PMFSV,	/* parent and mfs with seconds */
	AuWbrCreate_PMFSRR,	/* parent, mfs and round-robin */
	AuWbrCreate_PMFSRRV,	/* plus seconds */

	AuWbrCreate_Def = AuWbrCreate_TDP
};

enum {
	AuWbrCopyup_TDP,	/* top down parent */
	AuWbrCopyup_BUP,	/* bottom up parent */
	AuWbrCopyup_BU,		/* bottom up */

	AuWbrCopyup_Def = AuWbrCopyup_TDP
};

/* ---------------------------------------------------------------------- */

struct file;

struct au_opt_add {
	aufs_bindex_t	bindex;
	char		*pathname;
	int		perm;
	struct path	path;
};

struct au_opt_del {
	char		*pathname;
	struct path	h_path;
};

struct au_opt_mod {
	char		*path;
	int		perm;
	struct dentry	*h_root;
};

struct au_opt_xino {
	char		*path;
	struct file	*file;
};

struct au_opt_xino_itrunc {
	aufs_bindex_t	bindex;
};

struct au_opt_wbr_create {
	int			wbr_create;
	int			mfs_second;
	unsigned long long	mfsrr_watermark;
};

struct au_opt {
	int type;
	union {
		struct au_opt_xino	xino;
		struct au_opt_xino_itrunc xino_itrunc;
		struct au_opt_add	add;
		struct au_opt_del	del;
		struct au_opt_mod	mod;
		int			dirwh;
		int			rdcache;
		unsigned int		rdblk;
		unsigned int		rdhash;
		int			udba;
		struct au_opt_wbr_create wbr_create;
		int			wbr_copyup;
		unsigned int		fhsm_second;
		bool			tf; /* generic flag, true or false */
	};
};

/* opts flags */
#define AuOpts_REMOUNT		1
#define AuOpts_REFRESH		(1 << 1)
#define AuOpts_TRUNC_XIB	(1 << 2)
#define AuOpts_REFRESH_DYAOP	(1 << 3)
#define AuOpts_REFRESH_IDOP	(1 << 4)
#define AuOpts_DR_FLUSHED	(1 << 5)
#define au_ftest_opts(flags, name)	((flags) & AuOpts_##name)
#define au_fset_opts(flags, name) \
	do { (flags) |= AuOpts_##name; } while (0)
#define au_fclr_opts(flags, name) \
	do { (flags) &= ~AuOpts_##name; } while (0)

#ifndef CONFIG_AUFS_DIRREN
#undef AuOpts_DR_FLUSHED
#define AuOpts_DR_FLUSHED	0
#endif

struct au_opts {
	struct au_opt	*opt;
	int		max_opt;

	unsigned int	given_udba;
	unsigned int	flags;
	unsigned long	sb_flags;
};

/* ---------------------------------------------------------------------- */

/* opts.c */
int au_br_perm_val(char *perm);
void au_optstr_br_perm(au_br_perm_str_t *str, int perm);
int au_udba_val(char *str);
const char *au_optstr_udba(int udba);
int au_wbr_create_val(char *str, struct au_opt_wbr_create *create);
const char *au_optstr_wbr_create(int wbr_create);
int au_wbr_copyup_val(char *str);
const char *au_optstr_wbr_copyup(int wbr_copyup);

int au_opt_add(struct au_opt *opt, char *opt_str, unsigned long sb_flags,
	       aufs_bindex_t bindex);
struct super_block;
int au_opts_verify(struct super_block *sb, unsigned long sb_flags,
		   unsigned int pending);
int au_opts_mount(struct super_block *sb, struct au_opts *opts);
int au_opts_remount(struct super_block *sb, struct au_opts *opts);

unsigned int au_opt_udba(struct super_block *sb);

/* fsctx.c */
int aufs_fsctx_init(struct fs_context *fc);
extern const struct fs_parameter_spec aufs_fsctx_paramspec[];

#endif /* __KERNEL__ */
#endif /* __AUFS_OPTS_H__ */
