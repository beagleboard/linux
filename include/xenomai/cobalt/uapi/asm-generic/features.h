/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_ASM_GENERIC_FEATURES_H
#define _COBALT_UAPI_ASM_GENERIC_FEATURES_H

#include <linux/types.h>

#define XNFEAT_STRING_LEN 64

struct cobalt_featinfo {
	/** Real-time clock frequency */
	__u64 clock_freq;
	/** Offset of nkvdso in the sem heap. */
	__u32 vdso_offset;
	/** ABI revision level. */
	__u32 feat_abirev;
	/** Available feature set. */
	__u32 feat_all;
	/** Mandatory features (when requested). */
	__u32 feat_man;
	/** Requested feature set. */
	__u32 feat_req;
	/** Missing features. */
	__u32 feat_mis;
	char feat_all_s[XNFEAT_STRING_LEN];
	char feat_man_s[XNFEAT_STRING_LEN];
	char feat_req_s[XNFEAT_STRING_LEN];
	char feat_mis_s[XNFEAT_STRING_LEN];
	/* Architecture-specific features. */
	struct cobalt_featinfo_archdep feat_arch;
};

#define __xn_feat_smp         0x80000000
#define __xn_feat_nosmp       0x40000000
#define __xn_feat_fastsynch   0x20000000
#define __xn_feat_nofastsynch 0x10000000
#define __xn_feat_control     0x08000000
#define __xn_feat_prioceiling 0x04000000

#ifdef CONFIG_SMP
#define __xn_feat_smp_mask __xn_feat_smp
#else
#define __xn_feat_smp_mask __xn_feat_nosmp
#endif

/*
 * Revisit: all archs currently support fast locking, and there is no
 * reason for any future port not to provide this. This will be
 * written in stone at the next ABI update, when fastsynch support is
 * dropped from the optional feature set.
 */
#define __xn_feat_fastsynch_mask __xn_feat_fastsynch

/* List of generic features kernel or userland may support */
#define __xn_feat_generic_mask			\
	(__xn_feat_smp_mask		|	\
	 __xn_feat_fastsynch_mask 	|	\
	 __xn_feat_prioceiling)

/*
 * List of features both sides have to agree on: If userland supports
 * it, the kernel has to provide it, too. This means backward
 * compatibility between older userland and newer kernel may be
 * supported for those features, but forward compatibility between
 * newer userland and older kernel cannot.
 */
#define __xn_feat_generic_man_mask		\
	(__xn_feat_fastsynch		|	\
	 __xn_feat_nofastsynch		|	\
	 __xn_feat_nosmp		|	\
	 __xn_feat_prioceiling)

static inline
const char *get_generic_feature_label(unsigned int feature)
{
	switch (feature) {
	case __xn_feat_smp:
		return "smp";
	case __xn_feat_nosmp:
		return "nosmp";
	case __xn_feat_fastsynch:
		return "fastsynch";
	case __xn_feat_nofastsynch:
		return "nofastsynch";
	case __xn_feat_control:
		return "control";
	case __xn_feat_prioceiling:
		return "prioceiling";
	default:
		return 0;
	}
}

static inline int check_abi_revision(unsigned long abirev)
{
	return abirev == XENOMAI_ABI_REV;
}

#endif /* !_COBALT_UAPI_ASM_GENERIC_FEATURES_H */
