/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _KERNEL_COBALT_PROCFS_H
#define _KERNEL_COBALT_PROCFS_H

#ifdef CONFIG_XENO_OPT_VFILE
int xnprocfs_init_tree(void);
void xnprocfs_cleanup_tree(void);
#else
static inline int xnprocfs_init_tree(void) { return 0; }
static inline void xnprocfs_cleanup_tree(void) { }
#endif /* !CONFIG_XENO_OPT_VFILE */

#endif /* !_KERNEL_COBALT_PROCFS_H */
