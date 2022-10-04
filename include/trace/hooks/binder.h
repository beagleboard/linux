/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM binder
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks
#if !defined(_TRACE_HOOK_BINDER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_BINDER_H
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>
/*
 * Following tracepoints are not exported in tracefs and provide a
 * mechanism for vendor modules to hook and extend functionality
 */
struct binder_transaction;
struct task_struct;
struct binder_alloc;
struct binder_proc;
struct binder_thread;
struct binder_transaction_data;
struct seq_file;
DECLARE_HOOK(android_vh_binder_transaction_init,
	TP_PROTO(struct binder_transaction *t),
	TP_ARGS(t));
DECLARE_HOOK(android_vh_binder_priority_skip,
	TP_PROTO(struct task_struct *task, bool *skip),
	TP_ARGS(task, skip));
DECLARE_HOOK(android_vh_binder_set_priority,
	TP_PROTO(struct binder_transaction *t, struct task_struct *task),
	TP_ARGS(t, task));
DECLARE_HOOK(android_vh_binder_restore_priority,
	TP_PROTO(struct binder_transaction *t, struct task_struct *task),
	TP_ARGS(t, task));
struct binder_proc;
struct binder_thread;
DECLARE_HOOK(android_vh_binder_wakeup_ilocked,
	TP_PROTO(struct task_struct *task, bool sync, struct binder_proc *proc),
	TP_ARGS(task, sync, proc));
DECLARE_HOOK(android_vh_binder_wait_for_work,
	TP_PROTO(bool do_proc_work, struct binder_thread *tsk, struct binder_proc *proc),
	TP_ARGS(do_proc_work, tsk, proc));
DECLARE_HOOK(android_vh_sync_txn_recvd,
	TP_PROTO(struct task_struct *tsk, struct task_struct *from),
	TP_ARGS(tsk, from));
DECLARE_HOOK(android_vh_binder_alloc_new_buf_locked,
	TP_PROTO(size_t size, struct binder_alloc *alloc, int is_async),
	TP_ARGS(size, alloc, is_async));
DECLARE_HOOK(android_vh_binder_reply,
	TP_PROTO(struct binder_proc *target_proc, struct binder_proc *proc,
		struct binder_thread *thread, struct binder_transaction_data *tr),
	TP_ARGS(target_proc, proc, thread, tr));
DECLARE_HOOK(android_vh_binder_trans,
	TP_PROTO(struct binder_proc *target_proc, struct binder_proc *proc,
		struct binder_thread *thread, struct binder_transaction_data *tr),
	TP_ARGS(target_proc, proc, thread, tr));
DECLARE_RESTRICTED_HOOK(android_rvh_binder_transaction,
	TP_PROTO(struct binder_proc *target_proc, struct binder_proc *proc,
		struct binder_thread *thread, struct binder_transaction_data *tr),
	TP_ARGS(target_proc, proc, thread, tr), 1);
DECLARE_HOOK(android_vh_binder_preset,
	TP_PROTO(struct hlist_head *hhead, struct mutex *lock),
	TP_ARGS(hhead, lock));
DECLARE_HOOK(android_vh_binder_proc_transaction_entry,
	TP_PROTO(struct binder_proc *proc, struct binder_transaction *t,
	struct binder_thread **thread, int node_debug_id, bool pending_async,
	bool sync, bool *skip),
	TP_ARGS(proc, t, thread, node_debug_id, pending_async, sync, skip));
DECLARE_HOOK(android_vh_binder_proc_transaction,
	TP_PROTO(struct task_struct *caller_task, struct task_struct *binder_proc_task,
		struct task_struct *binder_th_task, int node_debug_id,
		unsigned int code, bool pending_async),
	TP_ARGS(caller_task, binder_proc_task, binder_th_task, node_debug_id, code, pending_async));
DECLARE_HOOK(android_vh_binder_proc_transaction_end,
	TP_PROTO(struct task_struct *caller_task, struct task_struct *binder_proc_task,
		struct task_struct *binder_th_task, unsigned int code,
		bool pending_async, bool sync),
	TP_ARGS(caller_task, binder_proc_task, binder_th_task, code, pending_async, sync));
DECLARE_HOOK(android_vh_binder_select_worklist_ilocked,
	TP_PROTO(struct list_head **list, struct binder_thread *thread, struct binder_proc *proc,
	int wait_for_proc_work),
	TP_ARGS(list, thread, proc, wait_for_proc_work));
DECLARE_HOOK(android_vh_binder_new_ref,
	TP_PROTO(struct task_struct *proc, uint32_t ref_desc, int node_debug_id),
	TP_ARGS(proc, ref_desc, node_debug_id));
DECLARE_HOOK(android_vh_binder_del_ref,
	TP_PROTO(struct task_struct *proc, uint32_t ref_desc),
	TP_ARGS(proc, ref_desc));
DECLARE_HOOK(android_vh_binder_print_transaction_info,
	TP_PROTO(struct seq_file *m, struct binder_proc *proc,
		 const char *prefix, struct binder_transaction *t),
	TP_ARGS(m, proc, prefix, t));
DECLARE_HOOK(android_vh_binder_looper_state_registered,
	TP_PROTO(struct binder_thread *thread, struct binder_proc *proc),
	TP_ARGS(thread, proc));
DECLARE_HOOK(android_vh_binder_thread_read,
	TP_PROTO(struct list_head **list, struct binder_proc *proc,
		struct binder_thread *thread),
	TP_ARGS(list, proc, thread));
DECLARE_HOOK(android_vh_binder_free_proc,
	TP_PROTO(struct binder_proc *proc),
	TP_ARGS(proc));
DECLARE_HOOK(android_vh_binder_thread_release,
	TP_PROTO(struct binder_proc *proc, struct binder_thread *thread),
	TP_ARGS(proc, thread));
DECLARE_HOOK(android_vh_binder_read_done,
	TP_PROTO(struct binder_proc *proc, struct binder_thread *thread),
	TP_ARGS(proc, thread));
DECLARE_HOOK(android_vh_binder_has_work_ilocked,
	TP_PROTO(struct binder_thread *thread, bool do_proc_work, int *ret),
	TP_ARGS(thread, do_proc_work, ret));
/* macro versions of hooks are no longer required */

#endif /* _TRACE_HOOK_BINDER_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
