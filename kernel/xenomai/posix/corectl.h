/*
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_POSIX_CORECTL_H
#define _COBALT_POSIX_CORECTL_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <xenomai/posix/syscall.h>
#include <cobalt/uapi/corectl.h>

struct cobalt_config_vector {
	void __user *u_buf;
	size_t u_bufsz;
};

COBALT_SYSCALL_DECL(corectl,
		    (int request, void __user *u_buf, size_t u_bufsz));

void cobalt_add_config_chain(struct notifier_block *nb);

void cobalt_remove_config_chain(struct notifier_block *nb);

#endif /* !_COBALT_POSIX_CORECTL_H */
