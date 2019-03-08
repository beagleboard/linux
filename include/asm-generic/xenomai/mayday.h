/*
 * Copyright (C) 2009, 2012 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#ifndef _COBALT_ASM_GENERIC_MAYDAY_H
#define _COBALT_ASM_GENERIC_MAYDAY_H

struct xnarchtcb;
struct task_struct;
struct pt_regs;

int xnarch_init_mayday(void);

void xnarch_cleanup_mayday(void);

void *xnarch_get_mayday_page(void);

void xnarch_handle_mayday(struct xnarchtcb *tcb,
			  struct pt_regs *regs,
			  unsigned long tramp);

void xnarch_fixup_mayday(struct xnarchtcb *tcb,
			 struct pt_regs *regs);

#endif /* !_COBALT_ASM_GENERIC_MAYDAY_H */
