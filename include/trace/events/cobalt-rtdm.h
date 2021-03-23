/*
 * Copyright (C) 2014 Jan Kiszka <jan.kiszka@siemens.com>.
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>.
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
#undef TRACE_SYSTEM
#define TRACE_SYSTEM cobalt_rtdm

#if !defined(_TRACE_COBALT_RTDM_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_COBALT_RTDM_H

#include <linux/tracepoint.h>
#include <linux/mman.h>
#include <linux/sched.h>

struct rtdm_fd;
struct rtdm_event;
struct rtdm_sem;
struct rtdm_mutex;
struct xnthread;
struct rtdm_device;
struct rtdm_dev_context;
struct _rtdm_mmap_request;

DECLARE_EVENT_CLASS(fd_event,
	TP_PROTO(struct rtdm_fd *fd, int ufd),
	TP_ARGS(fd, ufd),

	TP_STRUCT__entry(
		__field(struct rtdm_device *, dev)
		__field(int, ufd)
	),

	TP_fast_assign(
		__entry->dev = rtdm_fd_to_context(fd)->device;
		__entry->ufd = ufd;
	),

	TP_printk("device=%p fd=%d",
		  __entry->dev, __entry->ufd)
);

DECLARE_EVENT_CLASS(fd_request,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd, unsigned long arg),
	TP_ARGS(task, fd, ufd, arg),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__field(struct rtdm_device *, dev)
		__field(int, ufd)
		__field(unsigned long, arg)
	),

	TP_fast_assign(
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid = task_pid_nr(task);
		__entry->dev = rtdm_fd_to_context(fd)->device;
		__entry->ufd = ufd;
		__entry->arg = arg;
	),

	TP_printk("device=%p fd=%d arg=%#lx pid=%d comm=%s",
		  __entry->dev, __entry->ufd, __entry->arg,
		  __entry->pid, __entry->comm)
);

DECLARE_EVENT_CLASS(fd_request_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd, int status),
	TP_ARGS(task, fd, ufd, status),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__field(struct rtdm_device *, dev)
		__field(int, ufd)
	),

	TP_fast_assign(
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid = task_pid_nr(task);
		__entry->dev =
			!IS_ERR(fd) ? rtdm_fd_to_context(fd)->device : NULL;
		__entry->ufd = ufd;
	),

	TP_printk("device=%p fd=%d pid=%d comm=%s",
		  __entry->dev, __entry->ufd, __entry->pid, __entry->comm)
);

DECLARE_EVENT_CLASS(task_op,
	TP_PROTO(struct xnthread *task),
	TP_ARGS(task),

	TP_STRUCT__entry(
		__field(struct xnthread *, task)
		__string(task_name, task->name)
	),

	TP_fast_assign(
		__entry->task = task;
		__assign_str(task_name, task->name);
	),

	TP_printk("task %p(%s)", __entry->task, __get_str(task_name))
);

DECLARE_EVENT_CLASS(event_op,
	TP_PROTO(struct rtdm_event *ev),
	TP_ARGS(ev),

	TP_STRUCT__entry(
		__field(struct rtdm_event *, ev)
	),

	TP_fast_assign(
		__entry->ev = ev;
	),

	TP_printk("event=%p", __entry->ev)
);

DECLARE_EVENT_CLASS(sem_op,
	TP_PROTO(struct rtdm_sem *sem),
	TP_ARGS(sem),

	TP_STRUCT__entry(
		__field(struct rtdm_sem *, sem)
	),

	TP_fast_assign(
		__entry->sem = sem;
	),

	TP_printk("sem=%p", __entry->sem)
);

DECLARE_EVENT_CLASS(mutex_op,
	TP_PROTO(struct rtdm_mutex *mutex),
	TP_ARGS(mutex),

	TP_STRUCT__entry(
		__field(struct rtdm_mutex *, mutex)
	),

	TP_fast_assign(
		__entry->mutex = mutex;
	),

	TP_printk("mutex=%p", __entry->mutex)
);

TRACE_EVENT(cobalt_device_register,
	TP_PROTO(struct rtdm_device *dev),
	TP_ARGS(dev),

	TP_STRUCT__entry(
		__field(struct rtdm_device *, dev)
		__string(device_name, dev->name)
		__field(int, flags)
		__field(int, class_id)
		__field(int, subclass_id)
		__field(int, profile_version)
	),

	TP_fast_assign(
		__entry->dev	= dev;
		__assign_str(device_name, dev->name);
		__entry->flags = dev->driver->device_flags;
		__entry->class_id = dev->driver->profile_info.class_id;
		__entry->subclass_id = dev->driver->profile_info.subclass_id;
		__entry->profile_version = dev->driver->profile_info.version;
	),

	TP_printk("%s device %s=%p flags=0x%x, class=%d.%d profile=%d",
		  (__entry->flags & RTDM_DEVICE_TYPE_MASK)
		  == RTDM_NAMED_DEVICE ? "named" : "protocol",
		  __get_str(device_name), __entry->dev,
		  __entry->flags, __entry->class_id, __entry->subclass_id,
		  __entry->profile_version)
);

TRACE_EVENT(cobalt_device_unregister,
	TP_PROTO(struct rtdm_device *dev),
	TP_ARGS(dev),

	TP_STRUCT__entry(
		__field(struct rtdm_device *, dev)
		__string(device_name, dev->name)
	),

	TP_fast_assign(
		__entry->dev	= dev;
		__assign_str(device_name, dev->name);
	),

	TP_printk("device %s=%p",
		  __get_str(device_name), __entry->dev)
);

DEFINE_EVENT(fd_event, cobalt_fd_created,
	TP_PROTO(struct rtdm_fd *fd, int ufd),
	TP_ARGS(fd, ufd)
);

DEFINE_EVENT(fd_request, cobalt_fd_open,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long oflags),
	TP_ARGS(task, fd, ufd, oflags)
);

DEFINE_EVENT(fd_request, cobalt_fd_close,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long lock_count),
	TP_ARGS(task, fd, ufd, lock_count)
);

DEFINE_EVENT(fd_request, cobalt_fd_socket,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long protocol_family),
	TP_ARGS(task, fd, ufd, protocol_family)
);

DEFINE_EVENT(fd_request, cobalt_fd_read,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long len),
	TP_ARGS(task, fd, ufd, len)
);

DEFINE_EVENT(fd_request, cobalt_fd_write,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long len),
	TP_ARGS(task, fd, ufd, len)
);

DEFINE_EVENT(fd_request, cobalt_fd_ioctl,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long request),
	TP_ARGS(task, fd, ufd, request)
);

DEFINE_EVENT(fd_request, cobalt_fd_sendmsg,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long flags),
	TP_ARGS(task, fd, ufd, flags)
);

DEFINE_EVENT(fd_request, cobalt_fd_sendmmsg,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long flags),
	TP_ARGS(task, fd, ufd, flags)
);

DEFINE_EVENT(fd_request, cobalt_fd_recvmsg,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long flags),
	TP_ARGS(task, fd, ufd, flags)
);

DEFINE_EVENT(fd_request, cobalt_fd_recvmmsg,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 unsigned long flags),
	TP_ARGS(task, fd, ufd, flags)
);

#define cobalt_print_protbits(__prot)		\
	__print_flags(__prot,  "|", 		\
		      {PROT_EXEC, "exec"},	\
		      {PROT_READ, "read"},	\
		      {PROT_WRITE, "write"})

#define cobalt_print_mapbits(__flags)		\
	__print_flags(__flags,  "|", 		\
		      {MAP_SHARED, "shared"},	\
		      {MAP_PRIVATE, "private"},	\
		      {MAP_ANONYMOUS, "anon"},	\
		      {MAP_FIXED, "fixed"},	\
		      {MAP_HUGETLB, "huge"},	\
		      {MAP_NONBLOCK, "nonblock"},	\
		      {MAP_NORESERVE, "noreserve"},	\
		      {MAP_POPULATE, "populate"},	\
		      {MAP_UNINITIALIZED, "uninit"})

TRACE_EVENT(cobalt_fd_mmap,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd, struct _rtdm_mmap_request *rma),
        TP_ARGS(task, fd, ufd, rma),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__field(struct rtdm_device *, dev)
		__field(int, ufd)
		__field(size_t, length)
		__field(off_t, offset)
		__field(int, prot)
		__field(int, flags)
	),

	TP_fast_assign(
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid = task_pid_nr(task);
		__entry->dev = rtdm_fd_to_context(fd)->device;
		__entry->ufd = ufd;
		__entry->length = rma->length;
		__entry->offset = rma->offset;
		__entry->prot = rma->prot;
		__entry->flags = rma->flags;
	),

	TP_printk("device=%p fd=%d area={ len:%zu, off:%Lu }"
		  " prot=%#x(%s) flags=%#x(%s) pid=%d comm=%s",
		  __entry->dev, __entry->ufd, __entry->length,
		  (unsigned long long)__entry->offset,
		  __entry->prot, cobalt_print_protbits(__entry->prot),
		  __entry->flags, cobalt_print_mapbits(__entry->flags),
		  __entry->pid, __entry->comm)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_ioctl_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_read_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_write_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_recvmsg_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_recvmmsg_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_sendmsg_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_sendmmsg_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(fd_request_status, cobalt_fd_mmap_status,
	TP_PROTO(struct task_struct *task,
		 struct rtdm_fd *fd, int ufd,
		 int status),
	TP_ARGS(task, fd, ufd, status)
);

DEFINE_EVENT(task_op, cobalt_driver_task_join,
	TP_PROTO(struct xnthread *task),
	TP_ARGS(task)
);

TRACE_EVENT(cobalt_driver_event_init,
	TP_PROTO(struct rtdm_event *ev, unsigned long pending),
	TP_ARGS(ev, pending),

	TP_STRUCT__entry(
		__field(struct rtdm_event *, ev)
		__field(unsigned long,	pending)
	),

	TP_fast_assign(
		__entry->ev = ev;
		__entry->pending = pending;
	),

	TP_printk("event=%p pending=%#lx",
		  __entry->ev, __entry->pending)
);

TRACE_EVENT(cobalt_driver_event_wait,
	TP_PROTO(struct rtdm_event *ev, struct xnthread *task),
	TP_ARGS(ev, task),

	TP_STRUCT__entry(
		__field(struct xnthread *, task)
		__string(task_name, task->name)
		__field(struct rtdm_event *, ev)
	),

	TP_fast_assign(
		__entry->task = task;
		__assign_str(task_name, task->name);
		__entry->ev = ev;
	),

	TP_printk("event=%p task=%p(%s)",
		  __entry->ev, __entry->task, __get_str(task_name))
);

DEFINE_EVENT(event_op, cobalt_driver_event_signal,
	TP_PROTO(struct rtdm_event *ev),
	TP_ARGS(ev)
);

DEFINE_EVENT(event_op, cobalt_driver_event_clear,
	TP_PROTO(struct rtdm_event *ev),
	TP_ARGS(ev)
);

DEFINE_EVENT(event_op, cobalt_driver_event_pulse,
	TP_PROTO(struct rtdm_event *ev),
	TP_ARGS(ev)
);

DEFINE_EVENT(event_op, cobalt_driver_event_destroy,
	TP_PROTO(struct rtdm_event *ev),
	TP_ARGS(ev)
);

TRACE_EVENT(cobalt_driver_sem_init,
	TP_PROTO(struct rtdm_sem *sem, unsigned long value),
	TP_ARGS(sem, value),

	TP_STRUCT__entry(
		__field(struct rtdm_sem *, sem)
		__field(unsigned long, value)
	),

	TP_fast_assign(
		__entry->sem = sem;
		__entry->value = value;
	),

	TP_printk("sem=%p value=%lu",
		  __entry->sem, __entry->value)
);

TRACE_EVENT(cobalt_driver_sem_wait,
	TP_PROTO(struct rtdm_sem *sem, struct xnthread *task),
	TP_ARGS(sem, task),

	TP_STRUCT__entry(
		__field(struct xnthread *, task)
		__string(task_name, task->name)
		__field(struct rtdm_sem *, sem)
	),

	TP_fast_assign(
		__entry->task = task;
		__assign_str(task_name, task->name);
		__entry->sem = sem;
	),

	TP_printk("sem=%p task=%p(%s)",
		  __entry->sem, __entry->task, __get_str(task_name))
);

DEFINE_EVENT(sem_op, cobalt_driver_sem_up,
	TP_PROTO(struct rtdm_sem *sem),
	TP_ARGS(sem)
);

DEFINE_EVENT(sem_op, cobalt_driver_sem_destroy,
	TP_PROTO(struct rtdm_sem *sem),
	TP_ARGS(sem)
);

DEFINE_EVENT(mutex_op, cobalt_driver_mutex_init,
	TP_PROTO(struct rtdm_mutex *mutex),
	TP_ARGS(mutex)
);

DEFINE_EVENT(mutex_op, cobalt_driver_mutex_release,
	TP_PROTO(struct rtdm_mutex *mutex),
	TP_ARGS(mutex)
);

DEFINE_EVENT(mutex_op, cobalt_driver_mutex_destroy,
	TP_PROTO(struct rtdm_mutex *mutex),
	TP_ARGS(mutex)
);

TRACE_EVENT(cobalt_driver_mutex_wait,
	TP_PROTO(struct rtdm_mutex *mutex, struct xnthread *task),
	TP_ARGS(mutex, task),

	TP_STRUCT__entry(
		__field(struct xnthread *, task)
		__string(task_name, task->name)
		__field(struct rtdm_mutex *, mutex)
	),

	TP_fast_assign(
		__entry->task = task;
		__assign_str(task_name, task->name);
		__entry->mutex = mutex;
	),

	TP_printk("mutex=%p task=%p(%s)",
		  __entry->mutex, __entry->task, __get_str(task_name))
);

#endif /* _TRACE_COBALT_RTDM_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE cobalt-rtdm
#include <trace/define_trace.h>
