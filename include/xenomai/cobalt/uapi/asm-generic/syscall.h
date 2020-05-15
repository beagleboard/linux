/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_ASM_GENERIC_SYSCALL_H
#define _COBALT_UAPI_ASM_GENERIC_SYSCALL_H

#include <linux/types.h>
#include <asm/xenomai/uapi/features.h>
#include <asm/xenomai/uapi/syscall.h>

#define __COBALT_SYSCALL_BIT	0x10000000

struct cobalt_bindreq {
	/** Features userland requires. */
	__u32 feat_req;
	/** ABI revision userland uses. */
	__u32 abi_rev;
	/** Features the Cobalt core provides. */
	struct cobalt_featinfo feat_ret;
};

#define COBALT_SECONDARY  0
#define COBALT_PRIMARY    1

#endif /* !_COBALT_UAPI_ASM_GENERIC_SYSCALL_H */
