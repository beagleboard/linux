/**
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
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
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_ANALOGY_COMMAND_H
#define _COBALT_RTDM_ANALOGY_COMMAND_H

#include <rtdm/uapi/analogy.h>
#include <rtdm/analogy/context.h>

#define CR_CHAN(a) CHAN(a)
#define CR_RNG(a) (((a)>>16)&0xff)
#define CR_AREF(a) (((a)>>24)&0xf)

/* --- Command related function --- */
void a4l_free_cmddesc(struct a4l_cmd_desc * desc);

/* --- Upper layer functions --- */
int a4l_check_cmddesc(struct a4l_device_context * cxt, struct a4l_cmd_desc * desc);
int a4l_ioctl_cmd(struct a4l_device_context * cxt, void *arg);

#endif /* !_COBALT_RTDM_ANALOGY_COMMAND_H */
