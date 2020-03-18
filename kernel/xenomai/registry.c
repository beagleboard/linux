/*
 * Copyright (C) 2004 Philippe Gerum <rpm@xenomai.org>
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

#include <linux/slab.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/registry.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/apc.h>
#include <cobalt/kernel/assert.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_registry Registry services
 *
 * The registry provides a mean to index object descriptors on unique
 * alphanumeric keys. When labeled this way, an object is globally
 * exported; it can be searched for, and its descriptor returned to
 * the caller for further use; the latter operation is called a
 * "binding". When no object has been registered under the given name
 * yet, the registry can be asked to set up a rendez-vous, blocking
 * the caller until the object is eventually registered.
 *
 *@{
 */

struct xnobject *registry_obj_slots;
EXPORT_SYMBOL_GPL(registry_obj_slots);

static LIST_HEAD(free_object_list); /* Free objects. */

static LIST_HEAD(busy_object_list); /* Active and exported objects. */

static unsigned int nr_active_objects;

static unsigned long next_object_stamp;

static struct hlist_head *object_index;

static int nr_object_entries;

static struct xnsynch register_synch;

#ifdef CONFIG_XENO_OPT_VFILE

#include <linux/workqueue.h>

static void proc_callback(struct work_struct *work);

static void registry_proc_schedule(void *cookie);

static LIST_HEAD(proc_object_list);	/* Objects waiting for /proc handling. */

static DECLARE_WORK(registry_proc_work, proc_callback);

static int proc_apc;

static struct xnvfile_directory registry_vfroot;

static int usage_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	xnvfile_printf(it, "%u/%u\n",
		       nr_active_objects,
		       CONFIG_XENO_OPT_REGISTRY_NRSLOTS);
	return 0;
}

static struct xnvfile_regular_ops usage_vfile_ops = {
	.show = usage_vfile_show,
};

static struct xnvfile_regular usage_vfile = {
	.ops = &usage_vfile_ops,
};

#endif /* CONFIG_XENO_OPT_VFILE */

unsigned xnregistry_hash_size(void)
{
	static const int primes[] = {
		101, 211, 307, 401, 503, 601,
		701, 809, 907, 1009, 1103
	};

#define obj_hash_max(n)			 \
((n) < sizeof(primes) / sizeof(int) ? \
 (n) : sizeof(primes) / sizeof(int) - 1)

	return primes[obj_hash_max(CONFIG_XENO_OPT_REGISTRY_NRSLOTS / 100)];
}

int xnregistry_init(void)
{
	int n, ret __maybe_unused;

	registry_obj_slots = kmalloc(CONFIG_XENO_OPT_REGISTRY_NRSLOTS *
				     sizeof(struct xnobject), GFP_KERNEL);
	if (registry_obj_slots == NULL)
		return -ENOMEM;

#ifdef CONFIG_XENO_OPT_VFILE
	ret = xnvfile_init_dir("registry", &registry_vfroot, &cobalt_vfroot);
	if (ret)
		return ret;

	ret = xnvfile_init_regular("usage", &usage_vfile, &registry_vfroot);
	if (ret) {
		xnvfile_destroy_dir(&registry_vfroot);
		return ret;
	}

	proc_apc =
	    xnapc_alloc("registry_export", &registry_proc_schedule, NULL);

	if (proc_apc < 0) {
		xnvfile_destroy_regular(&usage_vfile);
		xnvfile_destroy_dir(&registry_vfroot);
		return proc_apc;
	}
#endif /* CONFIG_XENO_OPT_VFILE */

	next_object_stamp = 0;

	for (n = 0; n < CONFIG_XENO_OPT_REGISTRY_NRSLOTS; n++) {
		registry_obj_slots[n].objaddr = NULL;
		list_add_tail(&registry_obj_slots[n].link, &free_object_list);
	}

	/* Slot #0 is reserved/invalid. */
	list_get_entry(&free_object_list, struct xnobject, link);
	nr_active_objects = 1;

	nr_object_entries = xnregistry_hash_size();
	object_index = kmalloc(sizeof(*object_index) *
				      nr_object_entries, GFP_KERNEL);

	if (object_index == NULL) {
#ifdef CONFIG_XENO_OPT_VFILE
		xnvfile_destroy_regular(&usage_vfile);
		xnvfile_destroy_dir(&registry_vfroot);
		xnapc_free(proc_apc);
#endif /* CONFIG_XENO_OPT_VFILE */
		return -ENOMEM;
	}

	for (n = 0; n < nr_object_entries; n++)
		INIT_HLIST_HEAD(&object_index[n]);

	xnsynch_init(&register_synch, XNSYNCH_FIFO, NULL);

	return 0;
}

void xnregistry_cleanup(void)
{
#ifdef CONFIG_XENO_OPT_VFILE
	struct hlist_node *enext;
	struct xnobject *ecurr;
	struct xnpnode *pnode;
	int n;

	flush_scheduled_work();

	for (n = 0; n < nr_object_entries; n++)
		hlist_for_each_entry_safe(ecurr, enext, 
					&object_index[n], hlink) {
			pnode = ecurr->pnode;
			if (pnode == NULL)
				continue;

			pnode->ops->unexport(ecurr, pnode);

			if (--pnode->entries > 0)
				continue;

			xnvfile_destroy_dir(&pnode->vdir);

			if (--pnode->root->entries == 0)
				xnvfile_destroy_dir(&pnode->root->vdir);
		}
#endif /* CONFIG_XENO_OPT_VFILE */

	kfree(object_index);
	xnsynch_destroy(&register_synch);

#ifdef CONFIG_XENO_OPT_VFILE
	xnapc_free(proc_apc);
	flush_scheduled_work();
	xnvfile_destroy_regular(&usage_vfile);
	xnvfile_destroy_dir(&registry_vfroot);
#endif /* CONFIG_XENO_OPT_VFILE */

	kfree(registry_obj_slots);
}

#ifdef CONFIG_XENO_OPT_VFILE

static DEFINE_SEMAPHORE(export_mutex);

/*
 * The following stuff implements the mechanism for delegating
 * export/unexport requests to/from the /proc interface from the
 * Xenomai domain to the Linux kernel (i.e. the "lower stage"). This
 * ends up being a bit complex due to the fact that such requests
 * might lag enough before being processed by the Linux kernel so that
 * subsequent requests might just contradict former ones before they
 * even had a chance to be applied (e.g. export -> unexport in the
 * Xenomai domain for short-lived objects). This situation and the
 * like are hopefully properly handled due to a careful
 * synchronization of operations across domains.
 */
static void proc_callback(struct work_struct *work)
{
	struct xnvfile_directory *rdir, *dir;
	const char *rname, *type;
	struct xnobject *object;
	struct xnpnode *pnode;
	int ret;
	spl_t s;

	down(&export_mutex);

	xnlock_get_irqsave(&nklock, s);

	while (!list_empty(&proc_object_list)) {
		object = list_get_entry(&proc_object_list,
					struct xnobject, link);
		pnode = object->pnode;
		type = pnode->dirname;
		dir = &pnode->vdir;
		rdir = &pnode->root->vdir;
		rname = pnode->root->dirname;

		if (object->vfilp != XNOBJECT_EXPORT_SCHEDULED)
			goto unexport;

		object->vfilp = XNOBJECT_EXPORT_INPROGRESS;
		list_add_tail(&object->link, &busy_object_list);

		xnlock_put_irqrestore(&nklock, s);

		if (pnode->entries++ == 0) {
			if (pnode->root->entries++ == 0) {
				/* Create the root directory on the fly. */
				ret = xnvfile_init_dir(rname, rdir, &registry_vfroot);
				if (ret) {
					xnlock_get_irqsave(&nklock, s);
					object->pnode = NULL;
					pnode->root->entries = 0;
					pnode->entries = 0;
					continue;
				}
			}
			/* Create the class directory on the fly. */
			ret = xnvfile_init_dir(type, dir, rdir);
			if (ret) {
				if (pnode->root->entries == 1) {
					pnode->root->entries = 0;
					xnvfile_destroy_dir(rdir);
				}
				xnlock_get_irqsave(&nklock, s);
				object->pnode = NULL;
				pnode->entries = 0;
				continue;
			}
		}

		ret = pnode->ops->export(object, pnode);
		if (ret && --pnode->entries == 0) {
			xnvfile_destroy_dir(dir);
			if (--pnode->root->entries == 0)
				xnvfile_destroy_dir(rdir);
			xnlock_get_irqsave(&nklock, s);
			object->pnode = NULL;
		} else
			xnlock_get_irqsave(&nklock, s);

		continue;

	unexport:
		object->vfilp = NULL;
		object->pnode = NULL;

		if (object->vfilp == XNOBJECT_EXPORT_ABORTED)
			object->objaddr = NULL;

		if (object->objaddr)
			list_add_tail(&object->link, &busy_object_list);
		else {
			/*
			 * Trap the case where we are unexporting an
			 * already unregistered object.
			 */
			list_add_tail(&object->link, &free_object_list);
			nr_active_objects--;
		}

		xnlock_put_irqrestore(&nklock, s);

		pnode->ops->unexport(object, pnode);

		if (--pnode->entries == 0) {
			xnvfile_destroy_dir(dir);
			if (--pnode->root->entries == 0)
				xnvfile_destroy_dir(rdir);
		}

		xnlock_get_irqsave(&nklock, s);
	}

	xnlock_put_irqrestore(&nklock, s);

	up(&export_mutex);
}

static void registry_proc_schedule(void *cookie)
{
	/*
	 * schedule_work() will check for us if the work has already
	 * been scheduled, so just be lazy and submit blindly.
	 */
	schedule_work(&registry_proc_work);
}

static int registry_export_vfsnap(struct xnobject *object,
				  struct xnpnode *pnode)
{
	struct xnpnode_snapshot *p;
	int ret;

	/*
	 * Make sure to initialize _all_ mandatory vfile fields; most
	 * of the time we are using sane NULL defaults based on static
	 * storage for the vfile struct, but here we are building up a
	 * vfile object explicitly.
	 */
	p = container_of(pnode, struct xnpnode_snapshot, node);
	object->vfile_u.vfsnap.file.datasz = p->vfile.datasz;
	object->vfile_u.vfsnap.file.privsz = p->vfile.privsz;
	/*
	 * Make the vfile refer to the provided tag struct if any,
	 * otherwise use our default tag space. In the latter case,
	 * each object family has its own private revision tag.
	 */
	object->vfile_u.vfsnap.file.tag = p->vfile.tag ?:
		&object->vfile_u.vfsnap.tag;
	object->vfile_u.vfsnap.file.ops = p->vfile.ops;
	object->vfile_u.vfsnap.file.entry.lockops = p->vfile.lockops;

	ret = xnvfile_init_snapshot(object->key, &object->vfile_u.vfsnap.file,
				    &pnode->vdir);
	if (ret)
		return ret;

	object->vfilp = &object->vfile_u.vfsnap.file.entry;
	object->vfilp->private = object->objaddr;

	return 0;
}

static void registry_unexport_vfsnap(struct xnobject *object,
				    struct xnpnode *pnode)
{
	xnvfile_destroy_snapshot(&object->vfile_u.vfsnap.file);
}

static void registry_touch_vfsnap(struct xnobject *object)
{
	xnvfile_touch(&object->vfile_u.vfsnap.file);
}

struct xnpnode_ops xnregistry_vfsnap_ops = {
	.export = registry_export_vfsnap,
	.unexport = registry_unexport_vfsnap,
	.touch = registry_touch_vfsnap,
};
EXPORT_SYMBOL_GPL(xnregistry_vfsnap_ops);

static int registry_export_vfreg(struct xnobject *object,
				 struct xnpnode *pnode)
{
	struct xnpnode_regular *p;
	int ret;

	/* See registry_export_vfsnap() for hints. */
	p = container_of(pnode, struct xnpnode_regular, node);
	object->vfile_u.vfreg.privsz = p->vfile.privsz;
	object->vfile_u.vfreg.ops = p->vfile.ops;
	object->vfile_u.vfreg.entry.lockops = p->vfile.lockops;

	ret = xnvfile_init_regular(object->key, &object->vfile_u.vfreg,
				   &pnode->vdir);
	if (ret)
		return ret;

	object->vfilp = &object->vfile_u.vfreg.entry;
	object->vfilp->private = object->objaddr;

	return 0;
}

static void registry_unexport_vfreg(struct xnobject *object,
				    struct xnpnode *pnode)
{
	xnvfile_destroy_regular(&object->vfile_u.vfreg);
}

struct xnpnode_ops xnregistry_vfreg_ops = {
	.export = registry_export_vfreg,
	.unexport = registry_unexport_vfreg,
};
EXPORT_SYMBOL_GPL(xnregistry_vfreg_ops);

static int registry_export_vlink(struct xnobject *object,
				 struct xnpnode *pnode)
{
	struct xnpnode_link *link_desc;
	char *link_target;
	int ret;

	link_desc = container_of(pnode, struct xnpnode_link, node);
	link_target = link_desc->target(object->objaddr);
	if (link_target == NULL)
		return -ENOMEM;

	ret = xnvfile_init_link(object->key, link_target,
				&object->vfile_u.link, &pnode->vdir);
	kfree(link_target);
	if (ret)
		return ret;

	object->vfilp = &object->vfile_u.link.entry;
	object->vfilp->private = object->objaddr;

	return 0;
}

static void registry_unexport_vlink(struct xnobject *object,
				    struct xnpnode *pnode)
{
	xnvfile_destroy_link(&object->vfile_u.link);
}

struct xnpnode_ops xnregistry_vlink_ops = {
	.export = registry_export_vlink,
	.unexport = registry_unexport_vlink,
};
EXPORT_SYMBOL_GPL(xnregistry_vlink_ops);

static inline void registry_export_pnode(struct xnobject *object,
					 struct xnpnode *pnode)
{
	object->vfilp = XNOBJECT_EXPORT_SCHEDULED;
	object->pnode = pnode;
	list_del(&object->link);
	list_add_tail(&object->link, &proc_object_list);
	__xnapc_schedule(proc_apc);
}

static inline void registry_unexport_pnode(struct xnobject *object)
{
	if (object->vfilp != XNOBJECT_EXPORT_SCHEDULED) {
		/*
		 * We might have preempted a v-file read op, so bump
		 * the object's revtag to make sure the data
		 * collection is aborted next, if we end up deleting
		 * the object being read.
		 */
		if (object->pnode->ops->touch)
			object->pnode->ops->touch(object);
		list_del(&object->link);
		list_add_tail(&object->link, &proc_object_list);
		__xnapc_schedule(proc_apc);
	} else {
		/*
		 * Unexporting before the lower stage has had a chance
		 * to export. Move back the object to the busyq just
		 * like if no export had been requested.
		 */
		list_del(&object->link);
		list_add_tail(&object->link, &busy_object_list);
		object->pnode = NULL;
		object->vfilp = NULL;
	}
}

#endif /* CONFIG_XENO_OPT_VFILE */

static unsigned registry_hash_crunch(const char *key)
{
	unsigned int h = 0, g;

#define HQON    24		/* Higher byte position */
#define HBYTE   0xf0000000	/* Higher nibble on */

	while (*key) {
		h = (h << 4) + *key++;
		if ((g = (h & HBYTE)) != 0)
			h = (h ^ (g >> HQON)) ^ g;
	}

	return h % nr_object_entries;
}

static inline int registry_hash_enter(const char *key, struct xnobject *object)
{
	struct xnobject *ecurr;
	unsigned s;

	object->key = key;
	s = registry_hash_crunch(key);

	hlist_for_each_entry(ecurr, &object_index[s], hlink)
		if (ecurr == object || strcmp(key, ecurr->key) == 0)
			return -EEXIST;

	hlist_add_head(&object->hlink, &object_index[s]);

	return 0;
}

static inline int registry_hash_remove(struct xnobject *object)
{
	unsigned int s = registry_hash_crunch(object->key);
	struct xnobject *ecurr;

	hlist_for_each_entry(ecurr, &object_index[s], hlink)
		if (ecurr == object) {
			hlist_del(&ecurr->hlink);
			return 0;
		}

	return -ESRCH;
}

static struct xnobject *registry_hash_find(const char *key)
{
	struct xnobject *ecurr;

	hlist_for_each_entry(ecurr, 
			&object_index[registry_hash_crunch(key)], hlink)
		if (strcmp(key, ecurr->key) == 0)
			return ecurr;

	return NULL;
}

struct registry_wait_context {
	struct xnthread_wait_context wc;
	const char *key;
};

static inline int registry_wakeup_sleepers(const char *key)
{
	struct registry_wait_context *rwc;
	struct xnthread_wait_context *wc;
	struct xnthread *sleeper, *tmp;
	int cnt = 0;

	xnsynch_for_each_sleeper_safe(sleeper, tmp, &register_synch) {
		wc = xnthread_get_wait_context(sleeper);
		rwc = container_of(wc, struct registry_wait_context, wc);
		if (*key == *rwc->key && strcmp(key, rwc->key) == 0) {
			xnsynch_wakeup_this_sleeper(&register_synch, sleeper);
			++cnt;
		}
	}

	return cnt;
}

/**
 * @fn int xnregistry_enter(const char *key,void *objaddr,xnhandle_t *phandle,struct xnpnode *pnode)
 * @brief Register a real-time object.
 *
 * This service allocates a new registry slot for an associated
 * object, and indexes it by an alphanumeric key for later retrieval.
 *
 * @param key A valid NULL-terminated string by which the object will
 * be indexed and later retrieved in the registry. Since it is assumed
 * that such key is stored into the registered object, it will *not*
 * be copied but only kept by reference in the registry. Pass an empty
 * or NULL string if the object shall only occupy a registry slot for
 * handle-based lookups. The slash character is not accepted in @a key
 * if @a pnode is non-NULL.
 *
 * @param objaddr An opaque pointer to the object to index by @a
 * key.
 *
 * @param phandle A pointer to a generic handle defined by the
 * registry which will uniquely identify the indexed object, until the
 * latter is unregistered using the xnregistry_remove() service.
 *
 * @param pnode A pointer to an optional /proc node class
 * descriptor. This structure provides the information needed to
 * export all objects from the given class through the /proc
 * filesystem, under the /proc/xenomai/registry entry. Passing NULL
 * indicates that no /proc support is available for the newly
 * registered object.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a objaddr is NULL.
 *
 * - -EINVAL if @a pnode is non-NULL, and @a key points to a valid
 * string containing a '/' character.
 *
 * - -ENOMEM is returned if the system fails to get enough dynamic
 * memory from the global real-time heap in order to register the
 * object.
 *
 * - -EEXIST is returned if the @a key is already in use.
 *
 * @coretags{unrestricted, might-switch, atomic-entry}
 */
int xnregistry_enter(const char *key, void *objaddr,
		     xnhandle_t *phandle, struct xnpnode *pnode)
{
	struct xnobject *object;
	spl_t s;
	int ret;

	if (objaddr == NULL ||
	    (pnode != NULL && key != NULL && strchr(key, '/')))
		return -EINVAL;

	xnlock_get_irqsave(&nklock, s);

	if (list_empty(&free_object_list)) {
		ret = -EAGAIN;
		goto unlock_and_exit;
	}

	object = list_get_entry(&free_object_list, struct xnobject, link);
	nr_active_objects++;
	object->objaddr = objaddr;
	object->cstamp = ++next_object_stamp;
#ifdef CONFIG_XENO_OPT_VFILE
	object->pnode = NULL;
#endif
	if (key == NULL || *key == '\0') {
		object->key = NULL;
		*phandle = object - registry_obj_slots;
		ret = 0;
		goto unlock_and_exit;
	}

	ret = registry_hash_enter(key, object);
	if (ret) {
		nr_active_objects--;
		list_add_tail(&object->link, &free_object_list);
		goto unlock_and_exit;
	}

	list_add_tail(&object->link, &busy_object_list);

	/*
	 * <!> Make sure the handle is written back before the
	 * rescheduling takes place.
	 */
	*phandle = object - registry_obj_slots;

#ifdef CONFIG_XENO_OPT_VFILE
	if (pnode)
		registry_export_pnode(object, pnode);
#endif /* CONFIG_XENO_OPT_VFILE */

	if (registry_wakeup_sleepers(key))
		xnsched_run();

unlock_and_exit:

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnregistry_enter);

/**
 * @fn int xnregistry_bind(const char *key,xnticks_t timeout,int timeout_mode,xnhandle_t *phandle)
 * @brief Bind to a real-time object.
 *
 * This service retrieves the registry handle of a given object
 * identified by its key. Unless otherwise specified, this service
 * will block the caller if the object is not registered yet, waiting
 * for such registration to occur.
 *
 * @param key A valid NULL-terminated string which identifies the
 * object to bind to.
 *
 * @param timeout The timeout which may be used to limit the time the
 * thread wait for the object to be registered. This value is a wait
 * time given as a count of nanoseconds. It can either be relative,
 * absolute monotonic (XN_ABSOLUTE), or absolute adjustable
 * (XN_REALTIME) depending on @a timeout_mode. Passing XN_INFINITE @b
 * and setting @a timeout_mode to XN_RELATIVE specifies an unbounded
 * wait. Passing XN_NONBLOCK causes the service to return immediately
 * without waiting if the object is not registered on entry. All other
 * values are used as a wait limit.
 *
 * @param timeout_mode The mode of the @a timeout parameter. It can
 * either be set to XN_RELATIVE, XN_ABSOLUTE, or XN_REALTIME (see also
 * xntimer_start()).
 *
 * @param phandle A pointer to a memory location which will be written
 * upon success with the generic handle defined by the registry for
 * the retrieved object. Contents of this memory is undefined upon
 * failure.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a key is NULL.
 *
 * - -EINTR is returned if xnthread_unblock() has been called for the
 * waiting thread before the retrieval has completed.
 *
 * - -EWOULDBLOCK is returned if @a timeout is equal to XN_NONBLOCK
 * and the searched object is not registered on entry. As a special
 * exception, this error is also returned if this service should
 * block, but was called from a context which cannot sleep
 * (e.g. interrupt, non-realtime or scheduler locked).
 *
 * - -ETIMEDOUT is returned if the object cannot be retrieved within
 * the specified amount of time.
 *
 * @coretags{primary-only, might-switch}
 *
 * @note xnregistry_bind() only returns the index portion of a handle,
 * which might include other fixed bits to be complete
 * (e.g. XNSYNCH_PSHARED). The caller is responsible for completing
 * the handle returned with those bits if applicable, depending on the
 * context.
 */
int xnregistry_bind(const char *key, xnticks_t timeout, int timeout_mode,
		    xnhandle_t *phandle)
{
	struct registry_wait_context rwc;
	struct xnobject *object;
	int ret = 0, info;
	spl_t s;

	if (key == NULL)
		return -EINVAL;

	xnlock_get_irqsave(&nklock, s);

	if (timeout_mode == XN_RELATIVE &&
	    timeout != XN_INFINITE && timeout != XN_NONBLOCK) {
		timeout_mode = XN_ABSOLUTE;
		timeout += xnclock_read_monotonic(&nkclock);
	}

	for (;;) {
		object = registry_hash_find(key);
		if (object) {
			*phandle = object - registry_obj_slots;
			goto unlock_and_exit;
		}

		if ((timeout_mode == XN_RELATIVE && timeout == XN_NONBLOCK) ||
		    xnsched_unblockable_p()) {
			ret = -EWOULDBLOCK;
			goto unlock_and_exit;
		}

		rwc.key = key;
		xnthread_prepare_wait(&rwc.wc);
		info = xnsynch_sleep_on(&register_synch, timeout, timeout_mode);
		if (info & XNTIMEO) {
			ret = -ETIMEDOUT;
			goto unlock_and_exit;
		}
		if (info & XNBREAK) {
			ret = -EINTR;
			goto unlock_and_exit;
		}
	}

unlock_and_exit:

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnregistry_bind);

/**
 * @fn int xnregistry_remove(xnhandle_t handle)
 * @brief Forcibly unregister a real-time object.
 *
 * This service forcibly removes an object from the registry. The
 * removal is performed regardless of the current object's locking
 * status.
 *
 * @param handle The generic handle of the object to remove.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -ESRCH is returned if @a handle does not reference a registered
 * object.
 *
 * @coretags{unrestricted}
 */
int xnregistry_remove(xnhandle_t handle)
{
	struct xnobject *object;
	void *objaddr;
	int ret = 0;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	object = xnregistry_validate(handle);
	if (object == NULL) {
		ret = -ESRCH;
		goto unlock_and_exit;
	}

	objaddr = object->objaddr;
	object->objaddr = NULL;
	object->cstamp = 0;

	if (object->key) {
		registry_hash_remove(object);

#ifdef CONFIG_XENO_OPT_VFILE
		if (object->pnode) {
			if (object->vfilp == XNOBJECT_EXPORT_INPROGRESS) {
				object->vfilp = XNOBJECT_EXPORT_ABORTED;
				object->objaddr = objaddr;
			}

			registry_unexport_pnode(object);
			/*
			 * Leave the update of the object queues to
			 * the work callback if it has been kicked.
			 */
			if (object->pnode) {
				xnlock_put_irqrestore(&nklock, s);
				if (ipipe_root_p)
					flush_work(&registry_proc_work);
				return 0;
			}
		}
#endif /* CONFIG_XENO_OPT_VFILE */

		list_del(&object->link);
	}

	if (!IS_ENABLED(CONFIG_XENO_OPT_VFILE) || !object->objaddr) {
		list_add_tail(&object->link, &free_object_list);
		nr_active_objects--;
	}

unlock_and_exit:

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnregistry_remove);

/**
 * Turn a named object into an anonymous object
 *
 * @coretags{unrestricted}
 */
int xnregistry_unlink(const char *key)
{
	struct xnobject *object;
	int ret = 0;
	spl_t s;

	if (key == NULL)
		return -EINVAL;

	xnlock_get_irqsave(&nklock, s);

	object = registry_hash_find(key);
	if (object == NULL) {
		ret = -ESRCH;
		goto unlock_and_exit;
	}

	ret = registry_hash_remove(object);
	if (ret < 0)
		goto unlock_and_exit;

#ifdef CONFIG_XENO_OPT_VFILE
	if (object->pnode) {
		registry_unexport_pnode(object);
		/*
		 * Leave the update of the object queues to
		 * the work callback if it has been kicked.
		 */
		if (object->pnode)
			goto unlock_and_exit;
	}
#endif /* CONFIG_XENO_OPT_VFILE */

	list_del(&object->link);

	object->key = NULL;

unlock_and_exit:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

/**
 * @fn void *xnregistry_lookup(xnhandle_t handle, unsigned long *cstamp_r)
 * @brief Find a real-time object into the registry.
 *
 * This service retrieves an object from its handle into the registry
 * and returns the memory address of its descriptor. Optionally, it
 * also copies back the object's creation stamp which is unique across
 * object registration calls.
 *
 * @param handle The generic handle of the object to fetch.
 *
 * @param cstamp_r If not-NULL, the object's creation stamp will be
 * copied to this memory area.
 *
 * @return The memory address of the object's descriptor is returned
 * on success. Otherwise, NULL is returned if @a handle does not
 * reference a registered object.
 *
 * @coretags{unrestricted}
 */

/** @} */
