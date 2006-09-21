/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/mailbox.h>
#include "uaccess_dsp.h"
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ipbuf.h"
#include "fifo.h"
#include "proclist.h"
#include "ioctl.h"

#define is_aligned(adr,align)	(!((adr)&((align)-1)))

/*
 * devstate: task device state machine
 * NOTASK:	task is not attached.
 * ATTACHED:	task is attached.
 * GARBAGE:	task is detached. waiting for all processes to close this device.
 * ADDREQ:	requesting for tadd
 * DELREQ:	requesting for tdel. no process is opening this device.
 * FREEZED:	task is attached, but reserved to be killed.
 * ADDFAIL:	tadd failed.
 * ADDING:	tadd in process.
 * DELING:	tdel in process.
 * KILLING:	tkill in process.
 */
#define TASKDEV_ST_NOTASK	0x00000001
#define TASKDEV_ST_ATTACHED	0x00000002
#define TASKDEV_ST_GARBAGE	0x00000004
#define TASKDEV_ST_INVALID	0x00000008
#define TASKDEV_ST_ADDREQ	0x00000100
#define TASKDEV_ST_DELREQ	0x00000200
#define TASKDEV_ST_FREEZED	0x00000400
#define TASKDEV_ST_ADDFAIL	0x00001000
#define TASKDEV_ST_ADDING	0x00010000
#define TASKDEV_ST_DELING	0x00020000
#define TASKDEV_ST_KILLING	0x00040000
#define TASKDEV_ST_STATE_MASK	0x7fffffff
#define TASKDEV_ST_STALE	0x80000000

struct {
	long state;
	char *name;
} devstate_desc[] = {
	{ TASKDEV_ST_NOTASK,   "notask" },
	{ TASKDEV_ST_ATTACHED, "attached" },
	{ TASKDEV_ST_GARBAGE,  "garbage" },
	{ TASKDEV_ST_INVALID,  "invalid" },
	{ TASKDEV_ST_ADDREQ,   "addreq" },
	{ TASKDEV_ST_DELREQ,   "delreq" },
	{ TASKDEV_ST_FREEZED,  "freezed" },
	{ TASKDEV_ST_ADDFAIL,  "addfail" },
	{ TASKDEV_ST_ADDING,   "adding" },
	{ TASKDEV_ST_DELING,   "deling" },
	{ TASKDEV_ST_KILLING,  "killing" },
};

static char *devstate_name(long state) {
	int i;
	int max = ARRAY_SIZE(devstate_desc);

	for (i = 0; i < max; i++) {
		if (state & devstate_desc[i].state)
			return devstate_desc[i].name;
	}
	return "unknown";
}

struct rcvdt_bk_struct {
	struct ipblink link;
	unsigned int rp;
};

struct taskdev {
	struct bus_type *bus;
//	struct device_driver *driver;
	struct device dev;	/* Generic device interface */

	long state;
	struct rw_semaphore state_sem;
	wait_queue_head_t state_wait_q;
	struct mutex usecount_lock;
	unsigned int usecount;
	char name[TNM_LEN];
	struct file_operations fops;
	spinlock_t proc_list_lock;
	struct list_head proc_list;
	struct dsptask *task;

	/* read stuff */
	wait_queue_head_t read_wait_q;
	struct mutex read_mutex;
	union {
		struct fifo_struct fifo;	/* for active word */
		struct rcvdt_bk_struct bk;
	} rcvdt;

	/* write stuff */
	wait_queue_head_t write_wait_q;
	struct mutex write_mutex;
	spinlock_t wsz_lock;
	size_t wsz;

	/* tctl stuff */
	wait_queue_head_t tctl_wait_q;
	struct mutex tctl_mutex;
	int tctl_stat;
	int tctl_ret;	/* return value for tctl_show() */

	/* device lock */
	struct mutex lock;
	pid_t lock_pid;
};

#define to_taskdev(n) container_of(n, struct taskdev, dev)

struct dsptask {
	enum {
		TASK_ST_ERR = 0,
		TASK_ST_READY,
		TASK_ST_CFGREQ
	} state;
	u8 tid;
	char name[TNM_LEN];
	u16 ttyp;
	struct taskdev *dev;

	/* read stuff */
	struct ipbuf_p *ipbuf_pvt_r;

	/* write stuff */
	struct ipbuf_p *ipbuf_pvt_w;

	/* mmap stuff */
	void *map_base;
	size_t map_length;
};

#define sndtyp_acv(ttyp)	((ttyp) & TTYP_ASND)
#define sndtyp_psv(ttyp)	(!((ttyp) & TTYP_ASND))
#define sndtyp_bk(ttyp)		((ttyp) & TTYP_BKDM)
#define sndtyp_wd(ttyp)		(!((ttyp) & TTYP_BKDM))
#define sndtyp_pvt(ttyp)	((ttyp) & TTYP_PVDM)
#define sndtyp_gbl(ttyp)	(!((ttyp) & TTYP_PVDM))
#define rcvtyp_acv(ttyp)	((ttyp) & TTYP_ARCV)
#define rcvtyp_psv(ttyp)	(!((ttyp) & TTYP_ARCV))
#define rcvtyp_bk(ttyp)		((ttyp) & TTYP_BKMD)
#define rcvtyp_wd(ttyp)		(!((ttyp) & TTYP_BKMD))
#define rcvtyp_pvt(ttyp)	((ttyp) & TTYP_PVMD)
#define rcvtyp_gbl(ttyp)	(!((ttyp) & TTYP_PVMD))

static __inline__ int has_taskdev_lock(struct taskdev *dev);
static int dsp_rmdev_minor(unsigned char minor);
static int taskdev_init(struct taskdev *dev, char *name, unsigned char minor);
static void taskdev_delete(unsigned char minor);
static int taskdev_attach_task(struct taskdev *dev, struct dsptask *task);
static int dsp_tdel_bh(struct taskdev *dev, u16 type);

static struct bus_type dsptask_bus = {
	.name = "dsptask",
};

static struct class *dsp_task_class;
static DEFINE_MUTEX(devmgr_lock);
static struct taskdev *taskdev[TASKDEV_MAX];
static struct dsptask *dsptask[TASKDEV_MAX];
static DEFINE_MUTEX(cfg_lock);
static u16 cfg_cmd;
static u8 cfg_tid;
static DECLARE_WAIT_QUEUE_HEAD(cfg_wait_q);
static u8 n_task;	/* static task count */
static void *heap;

#define is_dynamic_task(tid)	((tid) >= n_task)

#define devstate_read_lock(dev, devstate) \
		devstate_read_lock_timeout(dev, devstate, 0)
#define devstate_read_unlock(dev)	up_read(&(dev)->state_sem)
#define devstate_write_lock(dev, devstate) \
		devstate_write_lock_timeout(dev, devstate, 0)
#define devstate_write_unlock(dev)	up_write(&(dev)->state_sem)

static ssize_t devname_show(struct device *d, struct device_attribute *attr,
			    char *buf);
static ssize_t devstate_show(struct device *d, struct device_attribute *attr,
			     char *buf);
static ssize_t proc_list_show(struct device *d, struct device_attribute *attr,
			      char *buf);
static ssize_t taskname_show(struct device *d, struct device_attribute *attr,
			     char *buf);
static ssize_t ttyp_show(struct device *d, struct device_attribute *attr,
			 char *buf);
static ssize_t fifosz_show(struct device *d, struct device_attribute *attr,
			   char *buf);
static int fifosz_store(struct device *d, struct device_attribute *attr,
			const char *buf, size_t count);
static ssize_t fifocnt_show(struct device *d, struct device_attribute *attr,
			    char *buf);
static ssize_t ipblink_show(struct device *d, struct device_attribute *attr,
			    char *buf);
static ssize_t wsz_show(struct device *d, struct device_attribute *attr,
			char *buf);
static ssize_t mmap_show(struct device *d, struct device_attribute *attr,
			 char *buf);

#define __ATTR_RW(_name,_mode) { \
	.attr = {.name = __stringify(_name), .mode = _mode, .owner = THIS_MODULE },	\
	.show	= _name##_show,					\
	.store	= _name##_store,					\
}

static struct device_attribute dev_attr_devname   = __ATTR_RO(devname);
static struct device_attribute dev_attr_devstate  = __ATTR_RO(devstate);
static struct device_attribute dev_attr_proc_list = __ATTR_RO(proc_list);
static struct device_attribute dev_attr_taskname  = __ATTR_RO(taskname);
static struct device_attribute dev_attr_ttyp      = __ATTR_RO(ttyp);
static struct device_attribute dev_attr_fifosz    = __ATTR_RW(fifosz, 0666);
static struct device_attribute dev_attr_fifocnt   = __ATTR_RO(fifocnt);
static struct device_attribute dev_attr_ipblink   = __ATTR_RO(ipblink);
static struct device_attribute dev_attr_wsz       = __ATTR_RO(wsz);
static struct device_attribute dev_attr_mmap      = __ATTR_RO(mmap);

/*
 * devstate_read_lock_timeout()
 * devstate_write_lock_timeout():
 * timeout != 0: dev->state can be diffeent from what you want.
 * timeout == 0: no timeout
 */
static int devstate_read_lock_timeout(struct taskdev *dev, long devstate,
				      int timeout)
{
	DECLARE_WAITQUEUE(wait, current);
	long current_state = current->state;
	int ret = 0;

	down_read(&dev->state_sem);
	if (dev->state & devstate)
		return 0;

	add_wait_queue(&dev->state_wait_q, &wait);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		up_read(&dev->state_sem);
		if (timeout) {
			if ((timeout = schedule_timeout(timeout)) == 0) {
				/* timeout */
				down_read(&dev->state_sem);
				break;
			}
		} else
			schedule();
		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
		down_read(&dev->state_sem);
	} while (!(dev->state & devstate));
	remove_wait_queue(&dev->state_wait_q, &wait);
	set_current_state(current_state);
	return ret;
}

static int devstate_read_lock_and_test(struct taskdev *dev, long devstate)
{
	down_read(&dev->state_sem);
	if (dev->state & devstate)
		return 1;	/* success */
	/* failure */
	up_read(&dev->state_sem);
	return 0;
}

static int devstate_write_lock_timeout(struct taskdev *dev, long devstate,
				       int timeout)
{
	DECLARE_WAITQUEUE(wait, current);
	long current_state = current->state;
	int ret = 0;

	down_write(&dev->state_sem);
	if (dev->state & devstate)
		return 0;

	add_wait_queue(&dev->state_wait_q, &wait);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		up_write(&dev->state_sem);
		if (timeout) {
			if ((timeout = schedule_timeout(timeout)) == 0) {
				/* timeout */
				down_write(&dev->state_sem);
				break;
			}
		} else
			schedule();
		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
		down_write(&dev->state_sem);
	} while (!(dev->state & devstate));
	remove_wait_queue(&dev->state_wait_q, &wait);
	set_current_state(current_state);
	return ret;
}

static int devstate_write_lock_and_test(struct taskdev *dev, long devstate)
{
	down_write(&dev->state_sem);
	if (dev->state & devstate)	/* success */
		return 1;

	/* failure */
	up_write(&dev->state_sem);
	return -1;
}

static int taskdev_lock_interruptible(struct taskdev *dev,
				      struct mutex *lock)
{
	int ret;

	if (has_taskdev_lock(dev))
		ret = mutex_lock_interruptible(lock);
	else {
		if ((ret = mutex_lock_interruptible(&dev->lock)) != 0)
			return ret;
		ret = mutex_lock_interruptible(lock);
		mutex_unlock(&dev->lock);
	}

	return ret;
}

static int taskdev_lock_and_statelock_attached(struct taskdev *dev,
					       struct mutex *lock)
{
	int ret;

	if (!devstate_read_lock_and_test(dev, TASKDEV_ST_ATTACHED))
		return -ENODEV;

	if ((ret = taskdev_lock_interruptible(dev, lock)) != 0)
		devstate_read_unlock(dev);

	return ret;
}

static __inline__ void taskdev_unlock_and_stateunlock(struct taskdev *dev,
						      struct mutex *lock)
{
	mutex_unlock(lock);
	devstate_read_unlock(dev);
}

/*
 * taskdev_flush_buf()
 * must be called under state_lock(ATTACHED) and dev->read_mutex.
 */
static int taskdev_flush_buf(struct taskdev *dev)
{
	u16 ttyp = dev->task->ttyp;

	if (sndtyp_wd(ttyp)) {
		/* word receiving */
		flush_fifo(&dev->rcvdt.fifo);
	} else {
		/* block receiving */
		struct rcvdt_bk_struct *rcvdt = &dev->rcvdt.bk;

		if (sndtyp_gbl(ttyp))
			ipblink_flush(&rcvdt->link);
		else {
			ipblink_flush_pvt(&rcvdt->link);
			release_ipbuf_pvt(dev->task->ipbuf_pvt_r);
		}
	}

	return 0;
}

/*
 * taskdev_set_fifosz()
 * must be called under dev->read_mutex.
 */
static int taskdev_set_fifosz(struct taskdev *dev, unsigned long sz)
{
	u16 ttyp = dev->task->ttyp;
	int stat;

	if (!(sndtyp_wd(ttyp) && sndtyp_acv(ttyp))) {
		printk(KERN_ERR
		       "omapdsp: buffer size can be changed only for "
		       "active word sending task.\n");
		return -EINVAL;
	}
	if ((sz == 0) || (sz & 1)) {
		printk(KERN_ERR "omapdsp: illegal buffer size! (%ld)\n"
				"it must be even and non-zero value.\n", sz);
		return -EINVAL;
	}

	stat = realloc_fifo(&dev->rcvdt.fifo, sz);
	if (stat == -EBUSY) {
		printk(KERN_ERR "omapdsp: buffer is not empty!\n");
		return stat;
	} else if (stat < 0) {
		printk(KERN_ERR
		       "omapdsp: unable to change receive buffer size. "
		       "(%ld bytes for %s)\n", sz, dev->name);
		return stat;
	}

	return 0;
}

static __inline__ int has_taskdev_lock(struct taskdev *dev)
{
	return (dev->lock_pid == current->pid);
}

static int taskdev_lock(struct taskdev *dev)
{
	if (mutex_lock_interruptible(&dev->lock))
		return -EINTR;
	dev->lock_pid = current->pid;
	return 0;
}

static int taskdev_unlock(struct taskdev *dev)
{
	if (!has_taskdev_lock(dev)) {
		printk(KERN_ERR
		       "omapdsp: an illegal process attempted to "
		       "unlock the dsptask lock!\n");
		return -EINVAL;
	}
	dev->lock_pid = 0;
	mutex_unlock(&dev->lock);
	return 0;
}

static int dsp_task_config(struct dsptask *task, u8 tid)
{
	u16 ttyp;
	int ret;

	task->tid = tid;
	dsptask[tid] = task;

	/* TCFG request */
	task->state = TASK_ST_CFGREQ;
	if (mutex_lock_interruptible(&cfg_lock)) {
		ret = -EINTR;
		goto fail_out;
	}
	cfg_cmd = MBOX_CMD_DSP_TCFG;
	mbcompose_send_and_wait(TCFG, tid, 0, &cfg_wait_q);
	cfg_cmd = 0;
	mutex_unlock(&cfg_lock);

	if (task->state != TASK_ST_READY) {
		printk(KERN_ERR "omapdsp: task %d configuration error!\n", tid);
		ret = -EINVAL;
		goto fail_out;
	}

	if (strlen(task->name) <= 1)
		sprintf(task->name, "%d", tid);
	printk(KERN_INFO "omapdsp: task %d: name %s\n", tid, task->name);

	ttyp = task->ttyp;

	/*
	 * task info sanity check
	 */

	/* task type check */
	if (rcvtyp_psv(ttyp) && rcvtyp_pvt(ttyp)) {
		printk(KERN_ERR "omapdsp: illegal task type(0x%04x), tid=%d\n",
		       tid, ttyp);
		ret = -EINVAL;
		goto fail_out;
	}

	/* private buffer address check */
	if (sndtyp_pvt(ttyp) &&
	    (ipbuf_p_validate(task->ipbuf_pvt_r, DIR_D2A) < 0)) {
		ret = -EINVAL;
		goto fail_out;
	}
	if (rcvtyp_pvt(ttyp) &&
	    (ipbuf_p_validate(task->ipbuf_pvt_w, DIR_A2D) < 0)) {
		ret = -EINVAL;
		goto fail_out;
	}

	/* mmap buffer configuration check */
	if ((task->map_length > 0) &&
	    ((!is_aligned((unsigned long)task->map_base, PAGE_SIZE)) ||
	     (!is_aligned(task->map_length, PAGE_SIZE)) ||
	     (dsp_mem_type(task->map_base, task->map_length) != MEM_TYPE_EXTERN))) {
		printk(KERN_ERR
		       "omapdsp: illegal mmap buffer address(0x%p) or "
		       "length(0x%x).\n"
		       "  It needs to be page-aligned and located at "
		       "external memory.\n",
		       task->map_base, task->map_length);
		ret = -EINVAL;
		goto fail_out;
	}

	return 0;

fail_out:
	dsptask[tid] = NULL;
	return ret;
}

static void dsp_task_init(struct dsptask *task)
{
	mbcompose_send(TCTL, task->tid, TCTL_TINIT);
}

int dsp_task_config_all(u8 n)
{
	int i, ret;
	struct taskdev *devheap;
	struct dsptask *taskheap;
	size_t devheapsz, taskheapsz;

	memset(taskdev, 0, sizeof(void *) * TASKDEV_MAX);
	memset(dsptask, 0, sizeof(void *) * TASKDEV_MAX);

	printk(KERN_INFO "omapdsp: found %d task(s)\n", n);
	if (n == 0)
		return 0;

	/*
	 * reducing kmalloc!
	 */
	devheapsz  = sizeof(struct taskdev) * n;
	taskheapsz = sizeof(struct dsptask) * n;
	heap = kzalloc(devheapsz + taskheapsz, GFP_KERNEL);
	if (heap == NULL)
		return -ENOMEM;
	devheap  = heap;
	taskheap = heap + devheapsz;

	n_task = n;
	for (i = 0; i < n; i++) {
		struct taskdev *dev  = &devheap[i];
		struct dsptask *task = &taskheap[i];

		if ((ret = dsp_task_config(task, i)) < 0)
			return ret;
		if ((ret = taskdev_init(dev, task->name, i)) < 0)
			return ret;
		if ((ret = taskdev_attach_task(dev, task)) < 0)
			return ret;
		dsp_task_init(task);
		printk(KERN_INFO "omapdsp: taskdev %s enabled.\n", dev->name);
	}

	return 0;
}

static void dsp_task_unconfig(struct dsptask *task)
{
	dsptask[task->tid] = NULL;
}

void dsp_task_unconfig_all(void)
{
	unsigned char minor;
	u8 tid;
	struct dsptask *task;

	for (minor = 0; minor < n_task; minor++) {
		/*
		 * taskdev[minor] can be NULL in case of
		 * configuration failure
		 */
		if (taskdev[minor])
			taskdev_delete(minor);
	}
	for (; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor])
			dsp_rmdev_minor(minor);
	}

	for (tid = 0; tid < n_task; tid++) {
		/*
		 * dsptask[tid] can be NULL in case of
		 * configuration failure
		 */
		task = dsptask[tid];
		if (task)
			dsp_task_unconfig(task);
	}
	for (; tid < TASKDEV_MAX; tid++) {
		task = dsptask[tid];
		if (task) {
			/*
			 * on-demand tasks should be deleted in
			 * rmdev_minor(), but just in case.
			 */
			dsp_task_unconfig(task);
			kfree(task);
		}
	}

	if (heap) {
		kfree(heap);
		heap = NULL;
	}

	n_task = 0;
}

static struct device_driver dsptask_driver = {
	.name	= "dsptask",
	.bus	= &dsptask_bus,
};

u8 dsp_task_count(void)
{
	return n_task;
}

int dsp_taskmod_busy(void)
{
	struct taskdev *dev;
	unsigned char minor;
	unsigned int usecount;

	for (minor = 0; minor < TASKDEV_MAX; minor++) {
		dev = taskdev[minor];
		if (dev == NULL)
			continue;
		if ((usecount = dev->usecount) > 0) {
			printk("dsp_taskmod_busy(): %s: usecount=%d\n",
			       dev->name, usecount);
			return 1;
		}
/*
		if ((dev->state & (TASKDEV_ST_ADDREQ |
				   TASKDEV_ST_DELREQ)) {
*/
		if (dev->state & TASKDEV_ST_ADDREQ) {
			printk("dsp_taskmod_busy(): %s is in %s\n",
			       dev->name, devstate_name(dev->state));
			return 1;
		}
	}
	return 0;
}

/*
 * DSP task device file operations
 */
static ssize_t dsp_task_read_wd_acv(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
		return -ENODEV;

	if (fifo_empty(&dev->rcvdt.fifo)) {
		long current_state = current->state;
		DECLARE_WAITQUEUE(wait, current);

		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&dev->read_wait_q, &wait);
		if (fifo_empty(&dev->rcvdt.fifo))	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&dev->read_wait_q, &wait);
		if (fifo_empty(&dev->rcvdt.fifo)) {
			/* failure */
			if (signal_pending(current))
				ret = -EINTR;
			goto up_out;
		}
	}

	ret = copy_to_user_fm_fifo(buf, &dev->rcvdt.fifo, count);

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
	return ret;
}

static ssize_t dsp_task_read_bk_acv(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct rcvdt_bk_struct *rcvdt = &dev->rcvdt.bk;
	ssize_t ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for "
		       "dsp_task_read().\n");
		return -EINVAL;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
		return -ENODEV;

	if (ipblink_empty(&rcvdt->link)) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->read_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (ipblink_empty(&rcvdt->link))	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&dev->read_wait_q, &wait);
		if (ipblink_empty(&rcvdt->link)) {
			/* failure */
			if (signal_pending(current))
				ret = -EINTR;
			goto up_out;
		}
	}

	/* copy from delayed IPBUF */
	if (sndtyp_pvt(dev->task->ttyp)) {
		/* private */
		if (!ipblink_empty(&rcvdt->link)) {
			struct ipbuf_p *ipbp = dev->task->ipbuf_pvt_r;
			unsigned char *base, *src;
			size_t bkcnt;

			if (dsp_mem_enable(ipbp) < 0) {
				ret = -EBUSY;
				goto up_out;
			}
			base = MKVIRT(ipbp->ah, ipbp->al);
			bkcnt = ((unsigned long)ipbp->c) * 2 - rcvdt->rp;
			if (dsp_address_validate(base, bkcnt,
						 "task %s read buffer",
						 dev->task->name) < 0) {
				ret = -EINVAL;
				goto pv_out1;
			}
			if (dsp_mem_enable(base) < 0) {
				ret = -EBUSY;
				goto pv_out1;
			}
			src = base + rcvdt->rp;
			if (bkcnt > count) {
				if (copy_to_user_dsp(buf, src, count)) {
					ret = -EFAULT;
					goto pv_out2;
				}
				ret = count;
				rcvdt->rp += count;
			} else {
				if (copy_to_user_dsp(buf, src, bkcnt)) {
					ret = -EFAULT;
					goto pv_out2;
				}
				ret = bkcnt;
				ipblink_del_pvt(&rcvdt->link);
				release_ipbuf_pvt(ipbp);
				rcvdt->rp = 0;
			}
pv_out2:
			dsp_mem_disable(src);
pv_out1:
			dsp_mem_disable(ipbp);
		}
	} else {
		/* global */
		if (dsp_mem_enable_ipbuf() < 0) {
			ret = -EBUSY;
			goto up_out;
		}
		while (!ipblink_empty(&rcvdt->link)) {
			unsigned char *src;
			size_t bkcnt;
			struct ipbuf_head *ipb_h = bid_to_ipbuf(rcvdt->link.top);

			src = ipb_h->p->d + rcvdt->rp;
			bkcnt = ((unsigned long)ipb_h->p->c) * 2 - rcvdt->rp;
			if (bkcnt > count) {
				if (copy_to_user_dsp(buf, src, count)) {
					ret = -EFAULT;
					goto gb_out;
				}
				ret += count;
				rcvdt->rp += count;
				break;
			} else {
				if (copy_to_user_dsp(buf, src, bkcnt)) {
					ret = -EFAULT;
					goto gb_out;
				}
				ret += bkcnt;
				buf += bkcnt;
				count -= bkcnt;
				ipblink_del_top(&rcvdt->link);
				unuse_ipbuf(ipb_h);
				rcvdt->rp = 0;
			}
		}
gb_out:
		dsp_mem_disable_ipbuf();
	}

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
	return ret;
}

static ssize_t dsp_task_read_wd_psv(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	} else {
		/* force! */
		count = 2;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
		return -ENODEV;

	mbcompose_send_and_wait(WDREQ, dev->task->tid, 0, &dev->read_wait_q);

	if (fifo_empty(&dev->rcvdt.fifo)) {
		/* failure */
		if (signal_pending(current))
			ret = -EINTR;
		goto up_out;
	}

	ret = copy_to_user_fm_fifo(buf, &dev->rcvdt.fifo, count);

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
	return ret;
}

static ssize_t dsp_task_read_bk_psv(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct rcvdt_bk_struct *rcvdt = &dev->rcvdt.bk;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for "
		       "dsp_task_read().\n");
		return -EINVAL;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
		return -ENODEV;

	mbcompose_send_and_wait(BKREQ, dev->task->tid, count/2,
				&dev->read_wait_q);

	if (ipblink_empty(&rcvdt->link)) {
		/* failure */
		if (signal_pending(current))
			ret = -EINTR;
		goto up_out;
	}

	/*
	 * We will not receive more than requested count.
	 */
	if (sndtyp_pvt(dev->task->ttyp)) {
		/* private */
		struct ipbuf_p *ipbp = dev->task->ipbuf_pvt_r;
		size_t rcvcnt;
		void *src;

		if (dsp_mem_enable(ipbp) < 0) {
			ret = -EBUSY;
			goto up_out;
		}
		src = MKVIRT(ipbp->ah, ipbp->al);
		rcvcnt = ((unsigned long)ipbp->c) * 2;
		if (dsp_address_validate(src, rcvcnt, "task %s read buffer",
					 dev->task->name) < 0) {
			ret = -EINVAL;
			goto pv_out1;
		}
		if (dsp_mem_enable(src) < 0) {
			ret = -EBUSY;
			goto pv_out1;
		}
		if (count > rcvcnt)
			count = rcvcnt;
		if (copy_to_user_dsp(buf, src, count)) {
			ret = -EFAULT;
			goto pv_out2;
		}
		ipblink_del_pvt(&rcvdt->link);
		release_ipbuf_pvt(ipbp);
		ret = count;
pv_out2:
		dsp_mem_disable(src);
pv_out1:
		dsp_mem_disable(ipbp);
	} else {
		/* global */
		struct ipbuf_head *ipb_h = bid_to_ipbuf(rcvdt->link.top);
		size_t rcvcnt;

		if (dsp_mem_enable_ipbuf() < 0) {
			ret = -EBUSY;
			goto up_out;
		}
		rcvcnt = ((unsigned long)ipb_h->p->c) * 2;
		if (count > rcvcnt)
			count = rcvcnt;
		if (copy_to_user_dsp(buf, ipb_h->p->d, count)) {
			ret = -EFAULT;
			goto gb_out;
		}
		ipblink_del_top(&rcvdt->link);
		unuse_ipbuf(ipb_h);
		ret = count;
gb_out:
		dsp_mem_disable_ipbuf();
	}

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
	return ret;
}

static ssize_t dsp_task_write_wd(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	u16 wd;
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	} else {
		/* force! */
		count = 2;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->write_mutex))
		return -ENODEV;

	if (dev->wsz == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->write_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (dev->wsz == 0)	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&dev->write_wait_q, &wait);
		if (dev->wsz == 0) {
			/* failure */
			if (signal_pending(current))
				ret = -EINTR;
			goto up_out;
		}
	}

	if (copy_from_user(&wd, buf, count)) {
		ret = -EFAULT;
		goto up_out;
	}

	spin_lock(&dev->wsz_lock);
	if (mbcompose_send(WDSND, dev->task->tid, wd) < 0) {
		spin_unlock(&dev->wsz_lock);
		goto up_out;
	}
	ret = count;
	if (rcvtyp_acv(dev->task->ttyp))
		dev->wsz = 0;
	spin_unlock(&dev->wsz_lock);

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->write_mutex);
	return ret;
}

static ssize_t dsp_task_write_bk(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret = 0;

	if (count == 0) {
		return 0;
	} else if (count & 0x1) {
		printk(KERN_ERR
		       "omapdsp: odd count is illegal for DSP task device.\n");
		return -EINVAL;
	} else if ((int)buf & 0x1) {
		printk(KERN_ERR
		       "omapdsp: buf should be word aligned for "
		       "dsp_task_write().\n");
		return -EINVAL;
	}

	if (taskdev_lock_and_statelock_attached(dev, &dev->write_mutex))
		return -ENODEV;

	if (dev->wsz == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&dev->write_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (dev->wsz == 0)	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&dev->write_wait_q, &wait);
		if (dev->wsz == 0) {
			/* failure */
			if (signal_pending(current))
				ret = -EINTR;
			goto up_out;
		}
	}

	if (count > dev->wsz)
		count = dev->wsz;

	if (rcvtyp_pvt(dev->task->ttyp)) {
		/* private */
		struct ipbuf_p *ipbp = dev->task->ipbuf_pvt_w;
		unsigned char *dst;

		if (dsp_mem_enable(ipbp) < 0) {
			ret = -EBUSY;
			goto up_out;
		}
		dst = MKVIRT(ipbp->ah, ipbp->al);
		if (dsp_address_validate(dst, count, "task %s write buffer",
					 dev->task->name) < 0) {
			ret = -EINVAL;
			goto pv_out1;
		}
		if (dsp_mem_enable(dst) < 0) {
			ret = -EBUSY;
			goto pv_out1;
		}
		if (copy_from_user_dsp(dst, buf, count)) {
			ret = -EFAULT;
			goto pv_out2;
		}
		ipbp->c = count/2;
		ipbp->s = dev->task->tid;
		spin_lock(&dev->wsz_lock);
		if (mbcompose_send(BKSNDP, dev->task->tid, 0) == 0) {
			if (rcvtyp_acv(dev->task->ttyp))
				dev->wsz = 0;
			ret = count;
		}
		spin_unlock(&dev->wsz_lock);
pv_out2:
		dsp_mem_disable(dst);
pv_out1:
		dsp_mem_disable(ipbp);
	} else {
		/* global */
		struct ipbuf_head *ipb_h;

		if (dsp_mem_enable_ipbuf() < 0) {
			ret = -EBUSY;
			goto up_out;
		}
		if ((ipb_h = get_free_ipbuf(dev->task->tid)) == NULL)
			goto gb_out;
		if (copy_from_user_dsp(ipb_h->p->d, buf, count)) {
			release_ipbuf(ipb_h);
			ret = -EFAULT;
			goto gb_out;
		}
		ipb_h->p->c  = count/2;
		ipb_h->p->sa = dev->task->tid;
		spin_lock(&dev->wsz_lock);
		if (mbcompose_send(BKSND, dev->task->tid, ipb_h->bid) == 0) {
			if (rcvtyp_acv(dev->task->ttyp))
				dev->wsz = 0;
			ret = count;
			ipb_bsycnt_inc(&ipbcfg);
		} else
			release_ipbuf(ipb_h);
		spin_unlock(&dev->wsz_lock);
gb_out:
		dsp_mem_disable_ipbuf();
	}

up_out:
	taskdev_unlock_and_stateunlock(dev, &dev->write_mutex);
	return ret;
}

static unsigned int dsp_task_poll(struct file * file, poll_table * wait)
{
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct dsptask *task = dev->task;
	unsigned int mask = 0;

	if (!devstate_read_lock_and_test(dev, TASKDEV_ST_ATTACHED))
		return 0;
	poll_wait(file, &dev->read_wait_q, wait);
	poll_wait(file, &dev->write_wait_q, wait);
	if (sndtyp_psv(task->ttyp) ||
	    (sndtyp_wd(task->ttyp) && !fifo_empty(&dev->rcvdt.fifo)) ||
	    (sndtyp_bk(task->ttyp) && !ipblink_empty(&dev->rcvdt.bk.link)))
		mask |= POLLIN | POLLRDNORM;
	if (dev->wsz)
		mask |= POLLOUT | POLLWRNORM;
	devstate_read_unlock(dev);

	return mask;
}

static int dsp_tctl_issue(struct taskdev *dev, u16 cmd, int argc, u16 argv[])
{
	int tctl_argc;
	struct mb_exarg mbarg, *mbargp;
	int interactive;
	u8 tid;
	int ret = 0;

	if (cmd < 0x8000) {
		/*
		 * 0x0000 - 0x7fff
		 * system reserved TCTL commands
		 */
		switch (cmd) {
		case TCTL_TEN:
		case TCTL_TDIS:
			tctl_argc = 0;
			interactive = 0;
			break;
		default:
			return -EINVAL;
		}
	}
	/*
	 * 0x8000 - 0xffff
	 * user-defined TCTL commands
	 */
	else if (cmd < 0x8100) {
		/* 0x8000-0x80ff: no arg, non-interactive */
		tctl_argc = 0;
		interactive = 0;
	} else if (cmd < 0x8200) {
		/* 0x8100-0x81ff: 1 arg, non-interactive */
		tctl_argc = 1;
		interactive = 0;
	} else if (cmd < 0x9000) {
		/* 0x8200-0x8fff: reserved */
		return -EINVAL;
	} else if (cmd < 0x9100) {
		/* 0x9000-0x90ff: no arg, interactive */
		tctl_argc = 0;
		interactive = 1;
	} else if (cmd < 0x9200) {
		/* 0x9100-0x91ff: 1 arg, interactive */
		tctl_argc = 1;
		interactive = 1;
	} else {
		/* 0x9200-0xffff: reserved */
		return -EINVAL;
	}

	/*
	 * if argc < 0, use tctl_argc as is.
	 * if argc >= 0, check arg count.
	 */
	if ((argc >= 0) && (argc != tctl_argc))
		return -EINVAL;

	/*
	 * issue TCTL
	 */
	if (taskdev_lock_interruptible(dev, &dev->tctl_mutex))
		return -EINTR;

	tid = dev->task->tid;
	if (tctl_argc > 0) {
		mbarg.argc = tctl_argc;
		mbarg.tid  = tid;
		mbarg.argv = argv;
		mbargp = &mbarg;
	} else
		mbargp = NULL;

	if (interactive) {
		dev->tctl_stat = -EINVAL;

		mbcompose_send_and_wait_exarg(TCTL, tid, cmd, mbargp,
					      &dev->tctl_wait_q);
		if (signal_pending(current)) {
			ret = -EINTR;
			goto up_out;
		}
		if ((ret = dev->tctl_stat) < 0) {
			printk(KERN_ERR "omapdsp: TCTL not responding.\n");
			goto up_out;
		}
	} else
		mbcompose_send_exarg(TCTL, tid, cmd, mbargp);

up_out:
	mutex_unlock(&dev->tctl_mutex);
	return ret;
}

static int dsp_task_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	int ret;

	if (cmd < 0x10000) {
		/* issue TCTL */
		u16 mbargv[1];

		mbargv[0] = arg & 0xffff;
		return dsp_tctl_issue(dev, cmd, -1, mbargv);
	}

	/* non TCTL ioctls */
	switch (cmd) {

	case TASK_IOCTL_LOCK:
		ret = taskdev_lock(dev);
		break;

	case TASK_IOCTL_UNLOCK:
		ret = taskdev_unlock(dev);
		break;

	case TASK_IOCTL_BFLSH:
		if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
			return -ENODEV;
		ret = taskdev_flush_buf(dev);
		taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
		break;

	case TASK_IOCTL_SETBSZ:
		if (taskdev_lock_and_statelock_attached(dev, &dev->read_mutex))
			return -ENODEV;
		ret = taskdev_set_fifosz(dev, arg);
		taskdev_unlock_and_stateunlock(dev, &dev->read_mutex);
		break;

	case TASK_IOCTL_GETNAME:
		ret = 0;
		if (copy_to_user((void __user *)arg, dev->name,
				 strlen(dev->name) + 1))
			ret = -EFAULT;
		break;

	default:
		ret = -ENOIOCTLCMD;

	}

	return ret;
}

static void dsp_task_mmap_open(struct vm_area_struct *vma)
{
	struct taskdev *dev = (struct taskdev *)vma->vm_private_data;
	struct dsptask *task;
	size_t len = vma->vm_end - vma->vm_start;

	BUG_ON(!(dev->state & TASKDEV_ST_ATTACHED));
	task = dev->task;
	exmap_use(task->map_base, len);
}

static void dsp_task_mmap_close(struct vm_area_struct *vma)
{
	struct taskdev *dev = (struct taskdev *)vma->vm_private_data;
	struct dsptask *task;
	size_t len = vma->vm_end - vma->vm_start;

	BUG_ON(!(dev->state & TASKDEV_ST_ATTACHED));
	task = dev->task;
	exmap_unuse(task->map_base, len);
}

/**
 * On demand page allocation is not allowed. The mapping area is defined by
 * corresponding DSP tasks.
 */
static struct page *dsp_task_mmap_nopage(struct vm_area_struct *vma,
					 unsigned long address, int *type)
{
	return NOPAGE_SIGBUS;
}

static struct vm_operations_struct dsp_task_vm_ops = {
	.open = dsp_task_mmap_open,
	.close = dsp_task_mmap_close,
	.nopage = dsp_task_mmap_nopage,
};

static int dsp_task_mmap(struct file *filp, struct vm_area_struct *vma)
{
	void *tmp_vadr;
	unsigned long tmp_padr, tmp_vmadr, off;
	size_t req_len, tmp_len;
	unsigned int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
	struct taskdev *dev = taskdev[minor];
	struct dsptask *task;
	int ret = 0;

	if (!devstate_read_lock_and_test(dev, TASKDEV_ST_ATTACHED))
		return -ENODEV;
	task = dev->task;

	/*
	 * Don't swap this area out
	 * Don't dump this area to a core file
	 */
	vma->vm_flags |= VM_RESERVED | VM_IO;

	/* Do not cache this area */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	req_len = vma->vm_end - vma->vm_start;
	off = vma->vm_pgoff << PAGE_SHIFT;
	tmp_vmadr = vma->vm_start;
	tmp_vadr = task->map_base + off;
	do {
		tmp_padr = dsp_virt_to_phys(tmp_vadr, &tmp_len);
		if (tmp_padr == 0) {
			printk(KERN_ERR
			       "omapdsp: task %s: illegal address "
			       "for mmap: %p", task->name, tmp_vadr);
			/* partial mapping will be cleared in upper layer */
			ret = -EINVAL;
			goto unlock_out;
		}
		if (tmp_len > req_len)
			tmp_len = req_len;

		printk(KERN_DEBUG
		       "omapdsp: mmap info: "
		       "vmadr = %08lx, padr = %08lx, len = %x\n",
		       tmp_vmadr, tmp_padr, tmp_len);
		if (remap_pfn_range(vma, tmp_vmadr, tmp_padr >> PAGE_SHIFT,
				    tmp_len, vma->vm_page_prot) != 0) {
			printk(KERN_ERR
			       "omapdsp: task %s: remap_page_range() failed.\n",
			       task->name);
			/* partial mapping will be cleared in upper layer */
			ret = -EINVAL;
			goto unlock_out;
		}

		req_len   -= tmp_len;
		tmp_vmadr += tmp_len;
		tmp_vadr  += tmp_len;
	} while (req_len);

	vma->vm_ops = &dsp_task_vm_ops;
	vma->vm_private_data = dev;
	exmap_use(task->map_base, vma->vm_end - vma->vm_start);

unlock_out:
	devstate_read_unlock(dev);
	return ret;
}

static int dsp_task_open(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev;
	int ret = 0;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL))
		return -ENODEV;

restart:
	mutex_lock(&dev->usecount_lock);
	down_write(&dev->state_sem);

	/* state can be NOTASK, ATTACHED/FREEZED, KILLING, GARBAGE or INVALID here. */
	switch (dev->state & TASKDEV_ST_STATE_MASK) {
		case TASKDEV_ST_NOTASK:
			break;
		case TASKDEV_ST_ATTACHED:
			goto attached;

		case TASKDEV_ST_INVALID:
			up_write(&dev->state_sem);
			mutex_unlock(&dev->usecount_lock);
			return -ENODEV;

		case TASKDEV_ST_FREEZED:
		case TASKDEV_ST_KILLING:
		case TASKDEV_ST_GARBAGE:
		case TASKDEV_ST_DELREQ:
			/* on the kill process. wait until it becomes NOTASK. */
			up_write(&dev->state_sem);
			mutex_unlock(&dev->usecount_lock);
			if (devstate_write_lock(dev, TASKDEV_ST_NOTASK) < 0)
				return -EINTR;
			devstate_write_unlock(dev);
			goto restart;
	}

	/* NOTASK */
	dev->state = TASKDEV_ST_ADDREQ;
	/* wake up twch daemon for tadd */
	dsp_twch_touch();
	up_write(&dev->state_sem);
	if (devstate_write_lock(dev, TASKDEV_ST_ATTACHED |
				     TASKDEV_ST_ADDFAIL) < 0) {
		/* cancelled */
		if (!devstate_write_lock_and_test(dev, TASKDEV_ST_ADDREQ)) {
			mutex_unlock(&dev->usecount_lock);
			/* out of control ??? */
			return -EINTR;
		}
		dev->state = TASKDEV_ST_NOTASK;
		ret = -EINTR;
		goto change_out;
	}
	if (dev->state & TASKDEV_ST_ADDFAIL) {
		printk(KERN_ERR "omapdsp: task attach failed for %s!\n",
		       dev->name);
		ret = -EBUSY;
		dev->state = TASKDEV_ST_NOTASK;
		goto change_out;
	}

attached:
	/* ATTACHED */
#ifndef CONFIG_OMAP_DSP_TASK_MULTIOPEN
	if (dev->usecount > 0) {
		up_write(&dev->state_sem);
		return -EBUSY;
	}
#endif
	dev->usecount++;
	proc_list_add(&dev->proc_list_lock, &dev->proc_list, current, file);
	file->f_op = &dev->fops;
	up_write(&dev->state_sem);
	mutex_unlock(&dev->usecount_lock);

#ifdef DSP_PTE_FREE	/* not used currently. */
	dsp_map_update(current);
	dsp_cur_users_add(current);
#endif /* DSP_PTE_FREE */
	return 0;

change_out:
	wake_up_interruptible_all(&dev->state_wait_q);
	up_write(&dev->state_sem);
	mutex_unlock(&dev->usecount_lock);
	return ret;
}

static int dsp_task_release(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct taskdev *dev = taskdev[minor];

#ifdef DSP_PTE_FREE	/* not used currently. */
	dsp_cur_users_del(current);
#endif /* DSP_PTE_FREE */

	if (has_taskdev_lock(dev))
		taskdev_unlock(dev);

	proc_list_del(&dev->proc_list_lock, &dev->proc_list, current, file);
	mutex_lock(&dev->usecount_lock);
	if (--dev->usecount > 0) {
		/* other processes are using this device. no state change. */
		mutex_unlock(&dev->usecount_lock);
		return 0;
	}

	/* usecount == 0 */
	down_write(&dev->state_sem);

	/* state can be ATTACHED/FREEZED, KILLING or GARBAGE here. */
	switch (dev->state & TASKDEV_ST_STATE_MASK) {

	case TASKDEV_ST_KILLING:
		break;

	case TASKDEV_ST_GARBAGE:
		dev->state = TASKDEV_ST_NOTASK;
		wake_up_interruptible_all(&dev->state_wait_q);
		break;

	case TASKDEV_ST_ATTACHED:
	case TASKDEV_ST_FREEZED:
		if (is_dynamic_task(minor)) {
			dev->state = TASKDEV_ST_DELREQ;
			/* wake up twch daemon for tdel */
			dsp_twch_touch();
		}
		break;

	}

	up_write(&dev->state_sem);
	mutex_unlock(&dev->usecount_lock);
	return 0;
}

/*
 * mkdev / rmdev
 */
int dsp_mkdev(char *name)
{
	struct taskdev *dev;
	int status;
	unsigned char minor;
	int ret;

	if (dsp_cfgstat_get_stat() != CFGSTAT_READY) {
		printk(KERN_ERR "omapdsp: dsp has not been configured.\n");
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&devmgr_lock))
		return -EINTR;

	/* naming check */
	for (minor = 0; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor] && !strcmp(taskdev[minor]->name, name)) {
			printk(KERN_ERR
			       "omapdsp: task device name %s is already "
			       "in use.\n", name);
			ret = -EINVAL;
			goto out;
		}
	}

	/* find free minor number */
	for (minor = n_task; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor] == NULL)
			goto do_make;
	}
	printk(KERN_ERR "omapdsp: Too many task devices.\n");
	ret = -EBUSY;
	goto out;

do_make:
	if ((dev = kzalloc(sizeof(struct taskdev), GFP_KERNEL)) == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	if ((status = taskdev_init(dev, name, minor)) < 0) {
		kfree(dev);
		ret = status;
		goto out;
	}
	ret = minor;

out:
	mutex_unlock(&devmgr_lock);
	return ret;
}

int dsp_rmdev(char *name)
{
	unsigned char minor;
	int status;
	int ret;

	if (dsp_cfgstat_get_stat() != CFGSTAT_READY) {
		printk(KERN_ERR "omapdsp: dsp has not been configured.\n");
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&devmgr_lock))
		return -EINTR;

	/* find in dynamic devices */
	for (minor = n_task; minor < TASKDEV_MAX; minor++) {
		if (taskdev[minor] && !strcmp(taskdev[minor]->name, name))
			goto do_remove;
	}

	/* find in static devices */
	for (minor = 0; minor < n_task; minor++) {
		if (taskdev[minor] && !strcmp(taskdev[minor]->name, name)) {
			printk(KERN_ERR
			       "omapdsp: task device %s is static.\n", name);
			ret = -EINVAL;
			goto out;
		}
	}

	printk(KERN_ERR "omapdsp: task device %s not found.\n", name);
	return -EINVAL;

do_remove:
	ret = minor;
	if ((status = dsp_rmdev_minor(minor)) < 0)
		ret = status;
out:
	mutex_unlock(&devmgr_lock);
	return ret;
}

static int dsp_rmdev_minor(unsigned char minor)
{
	struct taskdev *dev = taskdev[minor];

	while (!down_write_trylock(&dev->state_sem)) {
		down_read(&dev->state_sem);
		if (dev->state & (TASKDEV_ST_ATTACHED |
				  TASKDEV_ST_FREEZED)) {
			/*
			 * task is working. kill it.
			 * ATTACHED -> FREEZED can be changed under
			 * down_read of state_sem..
			 */
			dev->state = TASKDEV_ST_FREEZED;
			wake_up_interruptible_all(&dev->read_wait_q);
			wake_up_interruptible_all(&dev->write_wait_q);
			wake_up_interruptible_all(&dev->tctl_wait_q);
		}
		up_read(&dev->state_sem);
		schedule();
	}

	switch (dev->state & TASKDEV_ST_STATE_MASK) {

	case TASKDEV_ST_NOTASK:
		/* fine */
		goto notask;

	case TASKDEV_ST_ATTACHED:
	case TASKDEV_ST_FREEZED:
		/* task is working. kill it. */
		dev->state = TASKDEV_ST_KILLING;
		up_write(&dev->state_sem);
		dsp_tdel_bh(dev, TDEL_KILL);
		goto invalidate;

	case TASKDEV_ST_ADDREQ:
		/* open() is waiting. drain it. */
		dev->state = TASKDEV_ST_ADDFAIL;
		wake_up_interruptible_all(&dev->state_wait_q);
		break;

	case TASKDEV_ST_DELREQ:
		/* nobody is waiting. */
		dev->state = TASKDEV_ST_NOTASK;
		wake_up_interruptible_all(&dev->state_wait_q);
		break;

	case TASKDEV_ST_ADDING:
	case TASKDEV_ST_DELING:
	case TASKDEV_ST_KILLING:
	case TASKDEV_ST_GARBAGE:
	case TASKDEV_ST_ADDFAIL:
		/* transient state. wait for a moment. */
		break;

	}

	up_write(&dev->state_sem);

invalidate:
	/* wait for some time and hope the state is settled */
	devstate_read_lock_timeout(dev, TASKDEV_ST_NOTASK, 5 * HZ);
	if (!(dev->state & TASKDEV_ST_NOTASK)) {
		printk(KERN_WARNING
		       "omapdsp: illegal device state (%s) on rmdev %s.\n",
		       devstate_name(dev->state), dev->name);
	}
notask:
	dev->state = TASKDEV_ST_INVALID;
	devstate_read_unlock(dev);

	taskdev_delete(minor);
	kfree(dev);

	return 0;
}

struct file_operations dsp_task_fops = {
	.owner   = THIS_MODULE,
	.poll    = dsp_task_poll,
	.ioctl   = dsp_task_ioctl,
	.open    = dsp_task_open,
	.release = dsp_task_release,
};

static void dsptask_dev_release(struct device *dev)
{
}

static int taskdev_init(struct taskdev *dev, char *name, unsigned char minor)
{
	taskdev[minor] = dev;

	spin_lock_init(&dev->proc_list_lock);
	INIT_LIST_HEAD(&dev->proc_list);
	init_waitqueue_head(&dev->read_wait_q);
	init_waitqueue_head(&dev->write_wait_q);
	init_waitqueue_head(&dev->tctl_wait_q);
	mutex_init(&dev->read_mutex);
	mutex_init(&dev->write_mutex);
	mutex_init(&dev->tctl_mutex);
	mutex_init(&dev->lock);
	spin_lock_init(&dev->wsz_lock);
	dev->tctl_ret = -EINVAL;
	dev->lock_pid = 0;

	strncpy(dev->name, name, TNM_LEN);
	dev->name[TNM_LEN-1] = '\0';
	dev->state = (minor < n_task) ? TASKDEV_ST_ATTACHED : TASKDEV_ST_NOTASK;
	dev->usecount = 0;
	mutex_init(&dev->usecount_lock);
	memcpy(&dev->fops, &dsp_task_fops, sizeof(struct file_operations));

	dev->dev.parent = omap_dsp->dev;
	dev->dev.bus = &dsptask_bus;
	sprintf(dev->dev.bus_id, "dsptask%d", minor);
	dev->dev.release = dsptask_dev_release;
	device_register(&dev->dev);
	device_create_file(&dev->dev, &dev_attr_devname);
	device_create_file(&dev->dev, &dev_attr_devstate);
	device_create_file(&dev->dev, &dev_attr_proc_list);
	class_device_create(dsp_task_class, NULL,
			    MKDEV(OMAP_DSP_TASK_MAJOR, minor),
			    NULL, "dsptask%d", minor);

	init_waitqueue_head(&dev->state_wait_q);
	init_rwsem(&dev->state_sem);

	return 0;
}

static void taskdev_delete(unsigned char minor)
{
	struct taskdev *dev = taskdev[minor];

	if (!dev)
		return;
	device_remove_file(&dev->dev, &dev_attr_devname);
	device_remove_file(&dev->dev, &dev_attr_devstate);
	device_remove_file(&dev->dev, &dev_attr_proc_list);
	class_device_destroy(dsp_task_class, MKDEV(OMAP_DSP_TASK_MAJOR, minor));
	device_unregister(&dev->dev);
	proc_list_flush(&dev->proc_list_lock, &dev->proc_list);
	taskdev[minor] = NULL;
}

static int taskdev_attach_task(struct taskdev *dev, struct dsptask *task)
{
	u16 ttyp = task->ttyp;

	dev->fops.read =
		sndtyp_acv(ttyp) ?
			sndtyp_wd(ttyp) ? dsp_task_read_wd_acv:
			/* sndtyp_bk */   dsp_task_read_bk_acv:
		/* sndtyp_psv */
			sndtyp_wd(ttyp) ? dsp_task_read_wd_psv:
			/* sndtyp_bk */   dsp_task_read_bk_psv;
	if (sndtyp_wd(ttyp)) {
		/* word */
		size_t fifosz;

		fifosz = sndtyp_psv(ttyp) ? 2 :	/* passive */
					    32;	/* active */
		if (init_fifo(&dev->rcvdt.fifo, fifosz) < 0) {
			printk(KERN_ERR
			       "omapdsp: unable to allocate receive buffer. "
			       "(%d bytes for %s)\n", fifosz, dev->name);
			return -ENOMEM;
		}
	} else {
		/* block */
		INIT_IPBLINK(&dev->rcvdt.bk.link);
		dev->rcvdt.bk.rp = 0;
	}

	dev->fops.write =
		rcvtyp_wd(ttyp) ? dsp_task_write_wd:
		/* rcvbyp_bk */	  dsp_task_write_bk;
	dev->wsz = rcvtyp_acv(ttyp) ? 0 :		/* active */
		   rcvtyp_wd(ttyp)  ? 2 :		/* passive word */
				      ipbcfg.lsz*2;	/* passive block */

	if (task->map_length)
		dev->fops.mmap = dsp_task_mmap;

	device_create_file(&dev->dev, &dev_attr_taskname);
	device_create_file(&dev->dev, &dev_attr_ttyp);
	if (sndtyp_wd(ttyp)) {
		device_create_file(&dev->dev, &dev_attr_fifosz);
		device_create_file(&dev->dev, &dev_attr_fifocnt);
	} else
		device_create_file(&dev->dev, &dev_attr_ipblink);
	device_create_file(&dev->dev, &dev_attr_wsz);
	if (task->map_length)
		device_create_file(&dev->dev, &dev_attr_mmap);

	dev->task = task;
	task->dev = dev;

	return 0;
}

static void taskdev_detach_task(struct taskdev *dev)
{
	u16 ttyp = dev->task->ttyp;

	device_remove_file(&dev->dev, &dev_attr_taskname);
	device_remove_file(&dev->dev, &dev_attr_ttyp);
	if (sndtyp_wd(ttyp)) {
		device_remove_file(&dev->dev, &dev_attr_fifosz);
		device_remove_file(&dev->dev, &dev_attr_fifocnt);
	} else
		device_remove_file(&dev->dev, &dev_attr_ipblink);
	device_remove_file(&dev->dev, &dev_attr_wsz);
	if (dev->task->map_length)
		device_remove_file(&dev->dev, &dev_attr_mmap);

	dev->fops.read = NULL;
	taskdev_flush_buf(dev);
	if (sndtyp_wd(ttyp))
		free_fifo(&dev->rcvdt.fifo);

	dev->fops.write = NULL;
	dev->wsz = 0;

	printk(KERN_INFO "omapdsp: taskdev %s disabled.\n", dev->name);
	dev->task = NULL;
}

/*
 * tadd / tdel / tkill
 */
static int dsp_tadd(struct taskdev *dev, dsp_long_t adr)
{
	struct dsptask *task;
	struct mb_exarg arg;
	u8 tid, tid_response;
	u16 argv[2];
	int ret = 0;

	if (!devstate_write_lock_and_test(dev, TASKDEV_ST_ADDREQ)) {
		printk(KERN_ERR
		       "omapdsp: taskdev %s is not requesting for tadd. "
		       "(state is %s)\n", dev->name, devstate_name(dev->state));
		return -EINVAL;
	}
	dev->state = TASKDEV_ST_ADDING;
	devstate_write_unlock(dev);

	if (adr == TADD_ABORTADR) {
		/* aborting tadd intentionally */
		printk(KERN_INFO "omapdsp: tadd address is ABORTADR.\n");
		goto fail_out;
	}
	if (adr >= DSPSPACE_SIZE) {
		printk(KERN_ERR
		       "omapdsp: illegal address 0x%08x for tadd\n", adr);
		ret = -EINVAL;
		goto fail_out;
	}

	adr >>= 1;	/* word address */
	argv[0] = adr >> 16;	/* addrh */
	argv[1] = adr & 0xffff;	/* addrl */

	if (mutex_lock_interruptible(&cfg_lock)) {
		ret = -EINTR;
		goto fail_out;
	}
	cfg_tid = TID_ANON;
	cfg_cmd = MBOX_CMD_DSP_TADD;
	arg.tid  = TID_ANON;
	arg.argc = 2;
	arg.argv = argv;

	if (dsp_mem_sync_inc() < 0) {
		printk(KERN_ERR "omapdsp: memory sync failed!\n");
		ret = -EBUSY;
		goto fail_out;
	}
	mbcompose_send_and_wait_exarg(TADD, 0, 0, &arg, &cfg_wait_q);

	tid = cfg_tid;
	cfg_tid = TID_ANON;
	cfg_cmd = 0;
	mutex_unlock(&cfg_lock);

	if (tid == TID_ANON) {
		printk(KERN_ERR "omapdsp: tadd failed!\n");
		ret = -EINVAL;
		goto fail_out;
	}
	if ((tid < n_task) || dsptask[tid]) {
		printk(KERN_ERR "omapdsp: illegal tid (%d)!\n", tid);
		ret = -EINVAL;
		goto fail_out;
	}
	if ((task = kzalloc(sizeof(struct dsptask), GFP_KERNEL)) == NULL) {
		ret = -ENOMEM;
		goto del_out;
	}

	if ((ret = dsp_task_config(task, tid)) < 0)
		goto free_out;

	if (strcmp(dev->name, task->name)) {
		printk(KERN_ERR
		       "omapdsp: task name (%s) doesn't match with "
		       "device name (%s).\n", task->name, dev->name);
		ret = -EINVAL;
		goto free_out;
	}

	if ((ret = taskdev_attach_task(dev, task)) < 0)
		goto free_out;

	dsp_task_init(task);
	printk(KERN_INFO "omapdsp: taskdev %s enabled.\n", dev->name);
	dev->state = TASKDEV_ST_ATTACHED;
	wake_up_interruptible_all(&dev->state_wait_q);
	return 0;

free_out:
	kfree(task);

del_out:
	printk(KERN_ERR "omapdsp: deleting the task...\n");

	dev->state = TASKDEV_ST_DELING;

	if (mutex_lock_interruptible(&cfg_lock)) {
		printk(KERN_ERR "omapdsp: aborting tdel process. "
				"DSP side could be corrupted.\n");
		goto fail_out;
	}
	cfg_tid = TID_ANON;
	cfg_cmd = MBOX_CMD_DSP_TDEL;
	mbcompose_send_and_wait(TDEL, tid, TDEL_KILL, &cfg_wait_q);
	tid_response = cfg_tid;
	cfg_tid = TID_ANON;
	cfg_cmd = 0;
	mutex_unlock(&cfg_lock);

	if (tid_response != tid)
		printk(KERN_ERR "omapdsp: tdel failed. "
				"DSP side could be corrupted.\n");

fail_out:
	dev->state = TASKDEV_ST_ADDFAIL;
	wake_up_interruptible_all(&dev->state_wait_q);
	return ret;
}

int dsp_tadd_minor(unsigned char minor, dsp_long_t adr)
{
	struct taskdev *dev;
	int status;
	int ret;

	if (mutex_lock_interruptible(&devmgr_lock))
		return -EINTR;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		ret = -EINVAL;
		goto out;
	}
	ret = minor;
	if ((status = dsp_tadd(dev, adr)) < 0)
		ret = status;

out:
	mutex_unlock(&devmgr_lock);
	return ret;
}

static int dsp_tdel(struct taskdev *dev)
{
	if (!devstate_write_lock_and_test(dev, TASKDEV_ST_DELREQ)) {
		printk(KERN_ERR
		       "omapdsp: taskdev %s is not requesting for tdel. "
		       "(state is %s)\n", dev->name, devstate_name(dev->state));
		return -EINVAL;
	}
	dev->state = TASKDEV_ST_DELING;
	devstate_write_unlock(dev);

	return dsp_tdel_bh(dev, TDEL_SAFE);
}

int dsp_tdel_minor(unsigned char minor)
{
	struct taskdev *dev;
	int status;
	int ret;

	if (mutex_lock_interruptible(&devmgr_lock))
		return -EINTR;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		ret = -EINVAL;
		goto out;
	}

	ret = minor;
	if ((status = dsp_tdel(dev)) < 0)
		ret = status;

out:
	mutex_unlock(&devmgr_lock);
	return ret;
}

static int dsp_tkill(struct taskdev *dev)
{
	while (!down_write_trylock(&dev->state_sem)) {
		if (!devstate_read_lock_and_test(dev, (TASKDEV_ST_ATTACHED |
						       TASKDEV_ST_FREEZED))) {
			printk(KERN_ERR
			       "omapdsp: task has not been attached for "
			       "taskdev %s\n", dev->name);
			return -EINVAL;
		}
		/* ATTACHED -> FREEZED can be changed under read semaphore. */
		dev->state = TASKDEV_ST_FREEZED;
		wake_up_interruptible_all(&dev->read_wait_q);
		wake_up_interruptible_all(&dev->write_wait_q);
		wake_up_interruptible_all(&dev->tctl_wait_q);
		devstate_read_unlock(dev);
		schedule();
	}

	if (!(dev->state & (TASKDEV_ST_ATTACHED |
			    TASKDEV_ST_FREEZED))) {
		printk(KERN_ERR
		       "omapdsp: task has not been attached for taskdev %s\n",
		       dev->name);
		devstate_write_unlock(dev);
		return -EINVAL;
	}
	if (!is_dynamic_task(dev->task->tid)) {
		printk(KERN_ERR "omapdsp: task %s is not a dynamic task.\n",
		       dev->name);
		devstate_write_unlock(dev);
		return -EINVAL;
	}
	dev->state = TASKDEV_ST_KILLING;
	devstate_write_unlock(dev);

	return dsp_tdel_bh(dev, TDEL_KILL);
}

int dsp_tkill_minor(unsigned char minor)
{
	struct taskdev *dev;
	int status;
	int ret;

	if (mutex_lock_interruptible(&devmgr_lock))
		return -EINTR;

	if ((minor >= TASKDEV_MAX) || ((dev = taskdev[minor]) == NULL)) {
		printk(KERN_ERR
		       "omapdsp: no task device with minor %d\n", minor);
		ret = -EINVAL;
		goto out;
	}

	ret = minor;
	if ((status = dsp_tkill(dev)) < 0)
		ret = status;

out:
	mutex_unlock(&devmgr_lock);
	return ret;
}

static int dsp_tdel_bh(struct taskdev *dev, u16 type)
{
	struct dsptask *task;
	u8 tid, tid_response;
	int ret = 0;

	task = dev->task;
	tid = task->tid;
	if (mutex_lock_interruptible(&cfg_lock)) {
		if (type == TDEL_SAFE) {
			dev->state = TASKDEV_ST_DELREQ;
			return -EINTR;
		} else {
			tid_response = TID_ANON;
			ret = -EINTR;
			goto detach_out;
		}
	}
	cfg_tid = TID_ANON;
	cfg_cmd = MBOX_CMD_DSP_TDEL;
	mbcompose_send_and_wait(TDEL, tid, type, &cfg_wait_q);
	tid_response = cfg_tid;
	cfg_tid = TID_ANON;
	cfg_cmd = 0;
	mutex_unlock(&cfg_lock);

detach_out:
	taskdev_detach_task(dev);
	dsp_task_unconfig(task);
	kfree(task);

	if (tid_response != tid) {
		printk(KERN_ERR "omapdsp: %s failed!\n",
		       (type == TDEL_SAFE) ? "tdel" : "tkill");
		ret = -EINVAL;
	}
	down_write(&dev->state_sem);
	dev->state = (dev->usecount > 0) ? TASKDEV_ST_GARBAGE :
					   TASKDEV_ST_NOTASK;
	wake_up_interruptible_all(&dev->state_wait_q);
	up_write(&dev->state_sem);

	return ret;
}

/*
 * state inquiry
 */
long taskdev_state_stale(unsigned char minor)
{
	if (taskdev[minor]) {
		long state = taskdev[minor]->state;
		taskdev[minor]->state |= TASKDEV_ST_STALE;
		return state;
	} else
		return TASKDEV_ST_NOTASK;
}

/*
 * functions called from mailbox interrupt routine
 */
void mbox_wdsnd(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: WDSND with illegal tid! %d\n", tid);
		return;
	}
	if (sndtyp_bk(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: WDSND from block sending task! (task%d)\n", tid);
		return;
	}
	if (sndtyp_psv(task->ttyp) &&
	    !waitqueue_active(&task->dev->read_wait_q)) {
		printk(KERN_WARNING
		       "mbox: WDSND from passive sending task (task%d) "
		       "without request!\n", tid);
		return;
	}

	write_word_to_fifo(&task->dev->rcvdt.fifo, mb->data);
	wake_up_interruptible(&task->dev->read_wait_q);
}

void mbox_wdreq(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	struct taskdev *dev;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: WDREQ with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: WDREQ from passive receiving task! (task%d)\n",
		       tid);
		return;
	}

	dev = task->dev;
	spin_lock(&dev->wsz_lock);
	dev->wsz = 2;
	spin_unlock(&dev->wsz_lock);
	wake_up_interruptible(&dev->write_wait_q);
}

void mbox_bksnd(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	u16 bid = mb->data;
	struct dsptask *task = dsptask[tid];
	struct ipbuf_head *ipb_h;
	u16 cnt;

	if (bid >= ipbcfg.ln) {
		printk(KERN_ERR "mbox: BKSND with illegal bid! %d\n", bid);
		return;
	}
	ipb_h = bid_to_ipbuf(bid);
	ipb_bsycnt_dec(&ipbcfg);
	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: BKSND with illegal tid! %d\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sndtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKSND from word sending task! (task%d)\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sndtyp_pvt(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKSND from private sending task! (task%d)\n", tid);
		goto unuse_ipbuf_out;
	}
	if (sync_with_dsp(&ipb_h->p->sd, tid, 10) < 0) {
		printk(KERN_ERR "mbox: BKSND - IPBUF sync failed!\n");
		return;
	}

	/* should be done in DSP, but just in case. */
	ipb_h->p->next = BID_NULL;

	cnt = ipb_h->p->c;
	if (cnt > ipbcfg.lsz) {
		printk(KERN_ERR "mbox: BKSND cnt(%d) > ipbuf line size(%d)!\n",
		       cnt, ipbcfg.lsz);
		goto unuse_ipbuf_out;
	}

	if (cnt == 0) {
		/* 0-byte send from DSP */
		unuse_ipbuf_nowait(ipb_h);
		goto done;
	}
	ipblink_add_tail(&task->dev->rcvdt.bk.link, bid);
	/* we keep coming bid and return alternative line to DSP. */
	balance_ipbuf();

done:
	wake_up_interruptible(&task->dev->read_wait_q);
	return;

unuse_ipbuf_out:
	unuse_ipbuf_nowait(ipb_h);
	return;
}

void mbox_bkreq(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	u16 cnt = mb->data;
	struct dsptask *task = dsptask[tid];
	struct taskdev *dev;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: BKREQ with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQ from word receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_pvt(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQ from private receiving task! (task%d)\n",
		       tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQ from passive receiving task! (task%d)\n",
		       tid);
		return;
	}

	dev = task->dev;
	spin_lock(&dev->wsz_lock);
	dev->wsz = cnt*2;
	spin_unlock(&dev->wsz_lock);
	wake_up_interruptible(&dev->write_wait_q);
}

void mbox_bkyld(struct mbcmd *mb)
{
	u16 bid = mb->data;
	struct ipbuf_head *ipb_h;

	if (bid >= ipbcfg.ln) {
		printk(KERN_ERR "mbox: BKYLD with illegal bid! %d\n", bid);
		return;
	}
	ipb_h = bid_to_ipbuf(bid);

	/* should be done in DSP, but just in case. */
	ipb_h->p->next = BID_NULL;

	/* we don't need to sync with DSP */
	ipb_bsycnt_dec(&ipbcfg);
	release_ipbuf(ipb_h);
}

void mbox_bksndp(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	struct ipbuf_p *ipbp;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: BKSNDP with illegal tid! %d\n", tid);
		return;
	}
	if (sndtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKSNDP from word sending task! (task%d)\n", tid);
		return;
	}
	if (sndtyp_gbl(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKSNDP from non-private sending task! (task%d)\n",
		       tid);
		return;
	}

	/*
	 * we should not have delayed block at this point
	 * because read() routine releases the lock of the buffer and
	 * until then DSP can't send next data.
	 */

	ipbp = task->ipbuf_pvt_r;
	if (sync_with_dsp(&ipbp->s, tid, 10) < 0) {
		printk(KERN_ERR "mbox: BKSNDP - IPBUF sync failed!\n");
		return;
	}
	printk(KERN_DEBUG "mbox: ipbuf_pvt_r->a = 0x%08lx\n",
	       MKLONG(ipbp->ah, ipbp->al));
	ipblink_add_pvt(&task->dev->rcvdt.bk.link);
	wake_up_interruptible(&task->dev->read_wait_q);
}

void mbox_bkreqp(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	struct taskdev *dev;
	struct ipbuf_p *ipbp;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: BKREQP with illegal tid! %d\n", tid);
		return;
	}
	if (rcvtyp_wd(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQP from word receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_gbl(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQP from non-private receiving task! (task%d)\n", tid);
		return;
	}
	if (rcvtyp_psv(task->ttyp)) {
		printk(KERN_ERR
		       "mbox: BKREQP from passive receiving task! (task%d)\n", tid);
		return;
	}

	ipbp = task->ipbuf_pvt_w;
	if (sync_with_dsp(&ipbp->s, TID_FREE, 10) < 0) {
		printk(KERN_ERR "mbox: BKREQP - IPBUF sync failed!\n");
		return;
	}
	printk(KERN_DEBUG "mbox: ipbuf_pvt_w->a = 0x%08lx\n",
	       MKLONG(ipbp->ah, ipbp->al));
	dev = task->dev;
	spin_lock(&dev->wsz_lock);
	dev->wsz = ipbp->c*2;
	spin_unlock(&dev->wsz_lock);
	wake_up_interruptible(&dev->write_wait_q);
}

void mbox_tctl(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: TCTL with illegal tid! %d\n", tid);
		return;
	}

	if (!waitqueue_active(&task->dev->tctl_wait_q)) {
		printk(KERN_WARNING "mbox: unexpected TCTL from DSP!\n");
		return;
	}

	task->dev->tctl_stat = mb->data;
	wake_up_interruptible(&task->dev->tctl_wait_q);
}

void mbox_tcfg(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	struct dsptask *task = dsptask[tid];
	u16 *tnm;
	volatile u16 *buf;
	int i;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: TCFG with illegal tid! %d\n", tid);
		return;
	}
	if ((task->state != TASK_ST_CFGREQ) || (cfg_cmd != MBOX_CMD_DSP_TCFG)) {
		printk(KERN_WARNING "mbox: unexpected TCFG from DSP!\n");
		return;
	}

	if (dsp_mem_enable(ipbuf_sys_da) < 0) {
		printk(KERN_ERR "mbox: TCFG - ipbuf_sys_da read failed!\n");
		dsp_mem_disable(ipbuf_sys_da);
		goto out;
	}
	if (sync_with_dsp(&ipbuf_sys_da->s, tid, 10) < 0) {
		printk(KERN_ERR "mbox: TCFG - IPBUF sync failed!\n");
		dsp_mem_disable(ipbuf_sys_da);
		goto out;
	}

	/*
	 * read configuration data on system IPBUF
	 */
	buf = ipbuf_sys_da->d;
	task->ttyp        = buf[0];
	task->ipbuf_pvt_r = MKVIRT(buf[1], buf[2]);
	task->ipbuf_pvt_w = MKVIRT(buf[3], buf[4]);
	task->map_base    = MKVIRT(buf[5], buf[6]);
	task->map_length  = MKLONG(buf[7], buf[8]) << 1;	/* word -> byte */
	tnm               = MKVIRT(buf[9], buf[10]);
	release_ipbuf_pvt(ipbuf_sys_da);
	dsp_mem_disable(ipbuf_sys_da);

	/*
	 * copy task name string
	 */
	if (dsp_address_validate(tnm, TNM_LEN, "task name buffer") < 0) {
		task->name[0] = '\0';
		goto out;
	}

	for (i = 0; i < TNM_LEN-1; i++) {
		/* avoiding byte access */
		u16 tmp = tnm[i];
		task->name[i] = tmp & 0x00ff;
		if (!tmp)
			break;
	}
	task->name[TNM_LEN-1] = '\0';

	task->state = TASK_ST_READY;
out:
	wake_up_interruptible(&cfg_wait_q);
}

void mbox_tadd(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;

	if ((!waitqueue_active(&cfg_wait_q)) || (cfg_cmd != MBOX_CMD_DSP_TADD)) {
		printk(KERN_WARNING "mbox: unexpected TADD from DSP!\n");
		return;
	}
	cfg_tid = tid;
	wake_up_interruptible(&cfg_wait_q);
}

void mbox_tdel(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;

	if ((!waitqueue_active(&cfg_wait_q)) || (cfg_cmd != MBOX_CMD_DSP_TDEL)) {
		printk(KERN_WARNING "mbox: unexpected TDEL from DSP!\n");
		return;
	}
	cfg_tid = tid;
	wake_up_interruptible(&cfg_wait_q);
}

void mbox_err_fatal(u8 tid)
{
	struct dsptask *task = dsptask[tid];
	struct taskdev *dev;

	if ((tid >= TASKDEV_MAX) || (task == NULL)) {
		printk(KERN_ERR "mbox: FATAL ERR with illegal tid! %d\n", tid);
		return;
	}

	/* wake up waiting processes */
	dev = task->dev;
	wake_up_interruptible_all(&dev->read_wait_q);
	wake_up_interruptible_all(&dev->write_wait_q);
	wake_up_interruptible_all(&dev->tctl_wait_q);
}

static u16 *dbg_buf;
static u16 dbg_buf_sz, dbg_line_sz;
static int dbg_rp;

int dsp_dbg_config(u16 *buf, u16 sz, u16 lsz)
{
#ifdef OLD_BINARY_SUPPORT
	if ((mbox_revision == MBREV_3_0) || (mbox_revision == MBREV_3_2)) {
		dbg_buf = NULL;
		dbg_buf_sz = 0;
		dbg_line_sz = 0;
		dbg_rp = 0;
		return 0;
	}
#endif

	if (dsp_address_validate(buf, sz, "debug buffer") < 0)
		return -1;

	if (lsz > sz) {
		printk(KERN_ERR
		       "omapdsp: dbg_buf lsz (%d) is greater than its "
		       "buffer size (%d)\n", lsz, sz);
		return -1;
	}

	dbg_buf = buf;
	dbg_buf_sz = sz;
	dbg_line_sz = lsz;
	dbg_rp = 0;

	return 0;
}

void dsp_dbg_stop(void)
{
	dbg_buf = NULL;
}

#ifdef OLD_BINARY_SUPPORT
static void mbox_dbg_old(struct mbcmd *mb);
#endif

void mbox_dbg(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	int cnt = mb->data;
	char s[80], *s_end = &s[79], *p;
	u16 *src;
	int i;

#ifdef OLD_BINARY_SUPPORT
	if ((mbox_revision == MBREV_3_0) || (mbox_revision == MBREV_3_2)) {
		mbox_dbg_old(mb);
		return;
	}
#endif

	if (((tid >= TASKDEV_MAX) || (dsptask[tid] == NULL)) &&
	    (tid != TID_ANON)) {
		printk(KERN_ERR "mbox: DBG with illegal tid! %d\n", tid);
		return;
	}
	if (dbg_buf == NULL) {
		printk(KERN_ERR "mbox: DBG command received, but "
		       "dbg_buf has not been configured yet.\n");
		return;
	}

	if (dsp_mem_enable(dbg_buf) < 0)
		return;

	src = &dbg_buf[dbg_rp];
	p = s;
	for (i = 0; i < cnt; i++) {
		u16 tmp;
		/*
		 * Be carefull that dbg_buf should not be read with
		 * 1-byte access since it might be placed in DARAM/SARAM
		 * and it can cause unexpected byteswap.
		 * For example,
		 *   *(p++) = *(src++) & 0xff;
		 * causes 1-byte access!
		 */
		tmp = *src++;
		*(p++) = tmp & 0xff;
		if (*(p-1) == '\n') {
			*p = '\0';
			printk(KERN_INFO "%s", s);
			p = s;
			continue;
		}
		if (p == s_end) {
			*p = '\0';
			printk(KERN_INFO "%s\n", s);
			p = s;
			continue;
		}
	}
	if (p > s) {
		*p = '\0';
		printk(KERN_INFO "%s\n", s);
	}
	if ((dbg_rp += cnt + 1) > dbg_buf_sz - dbg_line_sz)
		dbg_rp = 0;

	dsp_mem_disable(dbg_buf);
}

#ifdef OLD_BINARY_SUPPORT
static void mbox_dbg_old(struct mbcmd *mb)
{
	u8 tid = mb->cmd_l;
	char s[80], *s_end = &s[79], *p;
	u16 *src;
	volatile u16 *buf;
	int cnt;
	int i;

	if (((tid >= TASKDEV_MAX) || (dsptask[tid] == NULL)) &&
	    (tid != TID_ANON)) {
		printk(KERN_ERR "mbox: DBG with illegal tid! %d\n", tid);
		return;
	}
	if (dsp_mem_enable(ipbuf_sys_da) < 0) {
		printk(KERN_ERR "mbox: DBG - ipbuf_sys_da read failed!\n");
		return;
	}
	if (sync_with_dsp(&ipbuf_sys_da->s, tid, 10) < 0) {
		printk(KERN_ERR "mbox: DBG - IPBUF sync failed!\n");
		goto out1;
	}
	buf = ipbuf_sys_da->d;
	cnt = buf[0];
	src = MKVIRT(buf[1], buf[2]);
	if (dsp_address_validate(src, cnt, "dbg buffer") < 0)
		goto out2;

	if (dsp_mem_enable(src) < 0)
		goto out2;

	p = s;
	for (i = 0; i < cnt; i++) {
		u16 tmp;
		/*
		 * Be carefull that ipbuf should not be read with
		 * 1-byte access since it might be placed in DARAM/SARAM
		 * and it can cause unexpected byteswap.
		 * For example,
		 *   *(p++) = *(src++) & 0xff;
		 * causes 1-byte access!
		 */
		tmp = *src++;
		*(p++) = tmp & 0xff;
		if (*(p-1) == '\n') {
			*p = '\0';
			printk(KERN_INFO "%s", s);
			p = s;
			continue;
		}
		if (p == s_end) {
			*p = '\0';
			printk(KERN_INFO "%s\n", s);
			p = s;
			continue;
		}
	}
	if (p > s) {
		*p = '\0';
		printk(KERN_INFO "%s\n", s);
	}

	dsp_mem_disable(src);
out2:
	release_ipbuf_pvt(ipbuf_sys_da);
out1:
	dsp_mem_disable(ipbuf_sys_da);
}
#endif /* OLD_BINARY_SUPPORT */

/*
 * sysfs files: for each device
 */

/* devname */
static ssize_t devname_show(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%s\n", to_taskdev(d)->name);
}

/* devstate */
static ssize_t devstate_show(struct device *d, struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%s\n", devstate_name(to_taskdev(d)->state));
}

/* proc_list */
static ssize_t proc_list_show(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	struct taskdev *dev;
	struct proc_list *pl;
	int len = 0;

	dev = to_taskdev(d);
	spin_lock(&dev->proc_list_lock);
	list_for_each_entry(pl, &dev->proc_list, list_head) {
		/* need to lock tasklist_lock before calling
		 * find_task_by_pid_type. */
		if (find_task_by_pid_type(PIDTYPE_PID, pl->pid) != NULL)
			len += sprintf(buf + len, "%d\n", pl->pid);
		read_unlock(&tasklist_lock);
	}
	spin_unlock(&dev->proc_list_lock);

	return len;
}

/* taskname */
static ssize_t taskname_show(struct device *d, struct device_attribute *attr,
			     char *buf)
{
	struct taskdev *dev = to_taskdev(d);
	int len;

	if (!devstate_read_lock_and_test(dev, TASKDEV_ST_ATTACHED))
		return -ENODEV;

	len = sprintf(buf, "%s\n", dev->task->name);

	devstate_read_unlock(dev);
	return len;
}

/* ttyp */
static ssize_t ttyp_show(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	struct taskdev *dev = to_taskdev(d);
	u16 ttyp;
	int len = 0;

	if (!devstate_read_lock_and_test(dev, TASKDEV_ST_ATTACHED))
		return -ENODEV;

	ttyp = dev->task->ttyp;
	len += sprintf(buf + len, "0x%04x\n", ttyp);
	len += sprintf(buf + len, "%s %s send\n",
			(sndtyp_acv(ttyp)) ? "active" :
					     "passive",
			(sndtyp_wd(ttyp))  ? "word" :
			(sndtyp_pvt(ttyp)) ? "private block" :
					     "global block");
	len += sprintf(buf + len, "%s %s receive\n",
			(rcvtyp_acv(ttyp)) ? "active" :
					     "passive",
			(rcvtyp_wd(ttyp))  ? "word" :
			(rcvtyp_pvt(ttyp)) ? "private block" :
					     "global block");

	devstate_read_unlock(dev);
	return len;
}

/* fifosz */
static ssize_t fifosz_show(struct device *d, struct device_attribute *attr,
			   char *buf)
{
	struct fifo_struct *fifo = &to_taskdev(d)->rcvdt.fifo;
	return sprintf(buf, "%d\n", fifo->sz);
}

static int fifosz_store(struct device *d, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct taskdev *dev = to_taskdev(d);
	unsigned long fifosz;
	int ret;

	fifosz = simple_strtol(buf, NULL, 10);
	ret = taskdev_set_fifosz(dev, fifosz);

	return (ret < 0) ? ret : strlen(buf);
}

/* fifocnt */
static ssize_t fifocnt_show(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	struct fifo_struct *fifo = &to_taskdev(d)->rcvdt.fifo;
	return sprintf(buf, "%d\n", fifo->cnt);
}

/* ipblink */
static __inline__ char *bid_name(u16 bid)
{
	static char s[6];

	switch (bid) {
	case BID_NULL:
		return "NULL";
	case BID_PVT:
		return "PRIVATE";
	default:
		sprintf(s, "%d", bid);
		return s;
	}
}

static ssize_t ipblink_show(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	struct rcvdt_bk_struct *rcvdt = &to_taskdev(d)->rcvdt.bk;
	int len;

	spin_lock(&rcvdt->link.lock);
	len = sprintf(buf, "top  %s\ntail %s\n",
		      bid_name(rcvdt->link.top), bid_name(rcvdt->link.tail));
	spin_unlock(&rcvdt->link.lock);

	return len;
}

/* wsz */
static ssize_t wsz_show(struct device *d, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", to_taskdev(d)->wsz);
}

/* mmap */
static ssize_t mmap_show(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	struct dsptask *task = to_taskdev(d)->task;
	return sprintf(buf, "0x%p 0x%x\n", task->map_base, task->map_length);
}

/*
 * called from ipbuf_show()
 */
int ipbuf_is_held(u8 tid, u16 bid)
{
	struct dsptask *task = dsptask[tid];
	struct ipblink *link;
	u16 b;
	int ret = 0;

	if (task == NULL)
		return 0;

	link = &task->dev->rcvdt.bk.link;
	spin_lock(&link->lock);
	ipblink_for_each(b, link) {
		if (b == bid) {	/* found */
			ret = 1;
			break;
		}
	}
	spin_unlock(&link->lock);

	return ret;
}

int __init dsp_taskmod_init(void)
{
	int retval;

	memset(taskdev, 0, sizeof(void *) * TASKDEV_MAX);
	memset(dsptask, 0, sizeof(void *) * TASKDEV_MAX);

	retval = register_chrdev(OMAP_DSP_TASK_MAJOR, "dsptask",
				 &dsp_task_fops);
	if (retval < 0) {
		printk(KERN_ERR
		       "omapdsp: failed to register task device: %d\n", retval);
		return retval;
	}

	bus_register(&dsptask_bus);
	retval = driver_register(&dsptask_driver);
	if (retval) {
		printk(KERN_ERR
		       "omapdsp: failed to register DSP task driver: %d\n",
		       retval);
		bus_unregister(&dsptask_bus);
		unregister_chrdev(OMAP_DSP_TASK_MAJOR, "dsptask");
		return -EINVAL;
	}
	dsp_task_class = class_create(THIS_MODULE, "dsptask");

	return 0;
}

void dsp_taskmod_exit(void)
{
	class_destroy(dsp_task_class);
	driver_unregister(&dsptask_driver);
	bus_unregister(&dsptask_bus);
	unregister_chrdev(OMAP_DSP_TASK_MAJOR, "dsptask");
}
