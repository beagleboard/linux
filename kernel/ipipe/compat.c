/* -*- linux-c -*-
 * linux/kernel/ipipe/compat.c
 *
 * Copyright (C) 2012 Philippe Gerum.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * I-pipe legacy interface.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ipipe.h>

static int ptd_key_count;

static unsigned long ptd_key_map;

IPIPE_DECLARE_SPINLOCK(__ipipe_lock);

void ipipe_init_attr(struct ipipe_domain_attr *attr)
{
	attr->name = "anon";
	attr->domid = 1;
	attr->entry = NULL;
	attr->priority = IPIPE_ROOT_PRIO;
	attr->pdd = NULL;
}
EXPORT_SYMBOL_GPL(ipipe_init_attr);

int ipipe_register_domain(struct ipipe_domain *ipd,
			  struct ipipe_domain_attr *attr)
{
	struct ipipe_percpu_domain_data *p;
	unsigned long flags;

	BUG_ON(attr->priority != IPIPE_HEAD_PRIORITY);

	ipipe_register_head(ipd, attr->name);
	ipd->legacy.domid = attr->domid;
	ipd->legacy.pdd = attr->pdd;
	ipd->legacy.priority = INT_MAX;

	if (attr->entry == NULL)
		return 0;

	flags = hard_smp_local_irq_save();
	__ipipe_set_current_domain(ipd);
	hard_smp_local_irq_restore(flags);

	attr->entry();

	flags = hard_local_irq_save();
	__ipipe_set_current_domain(ipipe_root_domain);
	p = ipipe_this_cpu_root_context();
	if (__ipipe_ipending_p(p) &&
	    !test_bit(IPIPE_STALL_FLAG, &p->status))
		__ipipe_sync_stage();
	hard_local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_register_domain);

int ipipe_unregister_domain(struct ipipe_domain *ipd)
{
	ipipe_unregister_head(ipd);

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_unregister_domain);

int ipipe_alloc_ptdkey(void)
{
	unsigned long flags;
	int key = -1;

	spin_lock_irqsave(&__ipipe_lock,flags);

	if (ptd_key_count < IPIPE_ROOT_NPTDKEYS) {
		key = ffz(ptd_key_map);
		set_bit(key,&ptd_key_map);
		ptd_key_count++;
	}

	spin_unlock_irqrestore(&__ipipe_lock,flags);

	return key;
}
EXPORT_SYMBOL_GPL(ipipe_alloc_ptdkey);

int ipipe_free_ptdkey(int key)
{
	unsigned long flags;

	if (key < 0 || key >= IPIPE_ROOT_NPTDKEYS)
		return -EINVAL;

	spin_lock_irqsave(&__ipipe_lock,flags);

	if (test_and_clear_bit(key,&ptd_key_map))
		ptd_key_count--;

	spin_unlock_irqrestore(&__ipipe_lock,flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_free_ptdkey);

int ipipe_set_ptd(int key, void *value)
{
	if (key < 0 || key >= IPIPE_ROOT_NPTDKEYS)
		return -EINVAL;

	current->ptd[key] = value;

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_set_ptd);

void *ipipe_get_ptd(int key)
{
	if (key < 0 || key >= IPIPE_ROOT_NPTDKEYS)
		return NULL;

	return current->ptd[key];
}
EXPORT_SYMBOL_GPL(ipipe_get_ptd);

int ipipe_virtualize_irq(struct ipipe_domain *ipd,
			 unsigned int irq,
			 ipipe_irq_handler_t handler,
			 void *cookie,
			 ipipe_irq_ackfn_t ackfn,
			 unsigned int modemask)
{
	if (handler == NULL) {
		ipipe_free_irq(ipd, irq);
		return 0;
	}

	return ipipe_request_irq(ipd, irq, handler, cookie, ackfn);
}
EXPORT_SYMBOL_GPL(ipipe_virtualize_irq);

static int null_handler(unsigned int event,
			struct ipipe_domain *from, void *data)
{
	/*
	 * Legacy mode users will trap all events, at worst most
	 * frequent ones. Therefore it is actually faster to run a
	 * dummy handler once in a while rather than testing for a
	 * null handler pointer each time an event is fired.
	 */
	return 0;
}

ipipe_event_handler_t ipipe_catch_event(struct ipipe_domain *ipd,
					unsigned int event,
					ipipe_event_handler_t handler)
{
	ipipe_event_handler_t oldhandler;
	int n, enables = 0;

	if (event & IPIPE_EVENT_SELF) {
		event &= ~IPIPE_EVENT_SELF;
		IPIPE_WARN(event >= IPIPE_NR_FAULTS);
	}

	if (event >= IPIPE_NR_EVENTS)
		return NULL;

	/*
	 * It makes no sense to run a SETSCHED notification handler
	 * over the head domain, this introduces a useless domain
	 * switch for doing work which ought to be root specific.
	 * Unfortunately, some client domains using the legacy
	 * interface still ask for this, so we silently fix their
	 * request. This prevents ipipe_set_hooks() from yelling at us
	 * because of an attempt to enable kernel event notifications
	 * for the head domain.
	 */
	if (event == IPIPE_EVENT_SETSCHED)
		ipd = ipipe_root_domain;

	oldhandler = ipd->legacy.handlers[event];
	ipd->legacy.handlers[event] = handler ?: null_handler;

	for (n = 0; n < IPIPE_NR_FAULTS; n++) {
		if (ipd->legacy.handlers[n] != null_handler) {
			enables |= __IPIPE_TRAP_E;
			break;
		}
	}

	for (n = IPIPE_FIRST_EVENT; n < IPIPE_LAST_EVENT; n++) {
		if (ipd->legacy.handlers[n] != null_handler) {
			enables |= __IPIPE_KEVENT_E;
			break;
		}
	}

	if (ipd->legacy.handlers[IPIPE_EVENT_SYSCALL] != null_handler)
		enables |= __IPIPE_SYSCALL_E;

	ipipe_set_hooks(ipd, enables);

	return oldhandler == null_handler ? NULL : oldhandler;
}
EXPORT_SYMBOL_GPL(ipipe_catch_event);

int ipipe_setscheduler_root(struct task_struct *p, int policy, int prio)
{
	struct sched_param param = { .sched_priority = prio };
	return sched_setscheduler_nocheck(p, policy, &param);
}
EXPORT_SYMBOL_GPL(ipipe_setscheduler_root);

int ipipe_syscall_hook(struct ipipe_domain *ipd, struct pt_regs *regs)
{
	const int event = IPIPE_EVENT_SYSCALL;
	return ipipe_current_domain->legacy.handlers[event](event, ipd, regs);
}

int ipipe_trap_hook(struct ipipe_trap_data *data)
{
	struct ipipe_domain *ipd = ipipe_head_domain;
	struct pt_regs *regs = data->regs;
	int ex = data->exception;

	return ipd->legacy.handlers[ex](ex, ipd, regs);
}

int ipipe_kevent_hook(int kevent, void *data)
{
	unsigned int event = IPIPE_FIRST_EVENT + kevent;
	struct ipipe_domain *ipd = ipipe_root_domain;

	return ipd->legacy.handlers[event](event, ipd, data);
}

void __ipipe_legacy_init_stage(struct ipipe_domain *ipd)
{
	int n;

	for (n = 0; n < IPIPE_NR_EVENTS; n++)
		ipd->legacy.handlers[n] = null_handler;

	if (ipd == &ipipe_root) {
		ipd->legacy.domid = IPIPE_ROOT_ID;
		ipd->legacy.priority = IPIPE_ROOT_PRIO;
	}
}

notrace asmlinkage int __ipipe_check_root(void) /* hw IRQs off */
{
	return __ipipe_root_p;
}
