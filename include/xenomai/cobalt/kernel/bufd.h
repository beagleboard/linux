/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_BUFD_H
#define _COBALT_KERNEL_BUFD_H

#include <linux/types.h>

/**
 * @addtogroup cobalt_core_bufd
 *
 * @{
 */

struct mm_struct;

struct xnbufd {
	caddr_t b_ptr;		/* src/dst buffer address */
	size_t b_len;		/* total length of buffer */
	off_t b_off;		/* # of bytes read/written */
	struct mm_struct *b_mm;	/* src/dst address space */
	caddr_t b_carry;	/* pointer to carry over area */
	char b_buf[64];		/* fast carry over area */
};

void xnbufd_map_umem(struct xnbufd *bufd,
		     void __user *ptr, size_t len);

static inline void xnbufd_map_uread(struct xnbufd *bufd,
				    const void __user *ptr, size_t len)
{
	xnbufd_map_umem(bufd, (void __user *)ptr, len);
}

static inline void xnbufd_map_uwrite(struct xnbufd *bufd,
				     void __user *ptr, size_t len)
{
	xnbufd_map_umem(bufd, ptr, len);
}

ssize_t xnbufd_unmap_uread(struct xnbufd *bufd);

ssize_t xnbufd_unmap_uwrite(struct xnbufd *bufd);

void xnbufd_map_kmem(struct xnbufd *bufd,
		     void *ptr, size_t len);

static inline void xnbufd_map_kread(struct xnbufd *bufd,
				    const void *ptr, size_t len)
{
	xnbufd_map_kmem(bufd, (void *)ptr, len);
}

static inline void xnbufd_map_kwrite(struct xnbufd *bufd,
				     void *ptr, size_t len)
{
	xnbufd_map_kmem(bufd, ptr, len);
}

ssize_t xnbufd_unmap_kread(struct xnbufd *bufd);

ssize_t xnbufd_unmap_kwrite(struct xnbufd *bufd);

ssize_t xnbufd_copy_to_kmem(void *ptr,
			    struct xnbufd *bufd, size_t len);

ssize_t xnbufd_copy_from_kmem(struct xnbufd *bufd,
			      void *from, size_t len);

void xnbufd_invalidate(struct xnbufd *bufd);

static inline void xnbufd_reset(struct xnbufd *bufd)
{
	bufd->b_off = 0;
}

/** @} */

#endif /* !_COBALT_KERNEL_BUFD_H */
