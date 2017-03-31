/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * ARM port
 *   Copyright (C) 2005 Stelian Pop
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
#ifndef _COBALT_ARM_ASM_UAPI_FEATURES_H
#define _COBALT_ARM_ASM_UAPI_FEATURES_H

/* The ABI revision level we use on this arch. */
#define XENOMAI_ABI_REV   16UL

#define XENOMAI_FEAT_DEP (__xn_feat_generic_mask)

#define XENOMAI_FEAT_MAN (__xn_feat_generic_man_mask)

#define XNARCH_HAVE_LLMULSHFT    1
#define XNARCH_HAVE_NODIV_LLIMD  1

struct cobalt_featinfo_archdep { /* no arch-specific feature */ };

#include <cobalt/uapi/asm-generic/features.h>

static inline const char *get_feature_label(unsigned int feature)
{
	return get_generic_feature_label(feature);
}

#endif /* !_COBALT_ARM_ASM_UAPI_FEATURES_H */
