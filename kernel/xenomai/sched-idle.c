/*
 * Copyright (C) 2008 Philippe Gerum <rpm@xenomai.org>.
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
#include <cobalt/kernel/sched.h>

static struct xnthread *xnsched_idle_pick(struct xnsched *sched)
{
	return &sched->rootcb;
}

void xnsched_idle_setparam(struct xnthread *thread,
			   const union xnsched_policy_param *p)
{
	__xnsched_idle_setparam(thread, p);
}

void xnsched_idle_getparam(struct xnthread *thread,
			   union xnsched_policy_param *p)
{
	__xnsched_idle_getparam(thread, p);
}

void xnsched_idle_trackprio(struct xnthread *thread,
			   const union xnsched_policy_param *p)
{
	__xnsched_rt_trackprio(thread, p);
}

struct xnsched_class xnsched_class_idle = {
	.sched_init		=	NULL,
	.sched_enqueue		=	NULL,
	.sched_dequeue		=	NULL,
	.sched_requeue		=	NULL,
	.sched_tick		=	NULL,
	.sched_rotate		=	NULL,
	.sched_forget		=	NULL,
	.sched_kick		=	NULL,
	.sched_declare		=	NULL,
	.sched_pick		=	xnsched_idle_pick,
	.sched_setparam		=	xnsched_idle_setparam,
	.sched_getparam		=	xnsched_idle_getparam,
	.sched_trackprio	=	xnsched_idle_trackprio,
	.weight			=	XNSCHED_CLASS_WEIGHT(0),
	.policy			=	SCHED_IDLE,
	.name			=	"idle"
};
