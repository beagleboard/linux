/*
 * Copyright (C) 2010 Philippe Gerum <rpm@xenomai.org>.
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
#include <linux/types.h>
#include <linux/limits.h>
#include <linux/ctype.h>
#include <linux/jhash.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/vmalloc.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/ppd.h>
#include <cobalt/uapi/signal.h>
#include <asm/xenomai/syscall.h>
#include "posix/process.h"
#include "debug.h"

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_debug Debugging services
 * @{
 */
struct xnvfile_directory cobalt_debug_vfroot;
EXPORT_SYMBOL_GPL(cobalt_debug_vfroot);

#ifdef CONFIG_XENO_OPT_DEBUG_TRACE_RELAX

#define SYMBOL_HSLOTS	(1 << 8)

struct hashed_symbol {
	struct hashed_symbol *next;
	char symbol[0];
};

static struct hashed_symbol *symbol_jhash[SYMBOL_HSLOTS];

static struct xnheap memory_pool;

/*
 * This is a permanent storage for ASCII strings which comes handy to
 * get a unique and constant reference to a symbol while preserving
 * storage space. Hashed symbols have infinite lifetime and are never
 * flushed.
 */
DEFINE_PRIVATE_XNLOCK(symbol_lock);

static const char *hash_symbol(const char *symbol)
{
	struct hashed_symbol *p, **h;
	const char *str;
	size_t len;
	u32 hash;
	spl_t s;

	len = strlen(symbol);
	hash = jhash(symbol, len, 0);

	xnlock_get_irqsave(&symbol_lock, s);

	h = &symbol_jhash[hash & (SYMBOL_HSLOTS - 1)];
	p = *h;
	while (p &&
	       (*p->symbol != *symbol ||
		strcmp(p->symbol + 1, symbol + 1)))
	       p = p->next;

	if (p)
		goto done;

	p = xnheap_alloc(&memory_pool, sizeof(*p) + len + 1);
	if (p == NULL) {
		str = NULL;
		goto out;
	}

	strcpy(p->symbol, symbol);
	p->next = *h;
	*h = p;
done:
	str = p->symbol;
out:
	xnlock_put_irqrestore(&symbol_lock, s);

	return str;
}

/*
 * We define a static limit (RELAX_SPOTNR) for spot records to limit
 * the memory consumption (we pull record memory from the system
 * heap). The current value should be reasonable enough unless the
 * application is extremely unsane, given that we only keep unique
 * spots. Said differently, if the application has more than
 * RELAX_SPOTNR distinct code locations doing spurious relaxes, then
 * the first issue to address is likely PEBKAC.
 */
#define RELAX_SPOTNR	128
#define RELAX_HSLOTS	(1 << 8)

struct relax_record {
	/* Number of hits for this location */
	u32 hits;
	struct relax_spot {
		/* Faulty thread name. */
		char thread[XNOBJECT_NAME_LEN];
		/* call stack the relax originates from. */
		int depth;
		struct backtrace {
			unsigned long pc;
			const char *mapname;
		} backtrace[SIGSHADOW_BACKTRACE_DEPTH];
		/* Program hash value of the caller. */
		u32 proghash;
		/* Pid of the caller. */
		pid_t pid;
		/* Reason for relaxing. */
		int reason;
	} spot;
	struct relax_record *r_next;
	struct relax_record *h_next;
	const char *exe_path;
};

static struct relax_record *relax_jhash[RELAX_HSLOTS];

static struct relax_record *relax_record_list;

static int relax_overall, relax_queued;

DEFINE_PRIVATE_XNLOCK(relax_lock);

/*
 * The motivation to centralize tracing information about relaxes
 * directly into kernel space is fourfold:
 *
 * - this allows to gather all the trace data into a single location
 * and keep it safe there, with no external log file involved.
 *
 * - enabling the tracing does not impose any requirement on the
 * application (aside of being compiled with debug symbols for best
 * interpreting that information). We only need a kernel config switch
 * for this (i.e. CONFIG_XENO_OPT_DEBUG_TRACE_RELAX).
 *
 * - the data is collected and can be made available exactly the same
 * way regardless of the application emitting the relax requests, or
 * whether it is still alive when the trace data are displayed.
 *
 * - the kernel is able to provide accurate and detailed trace
 * information, such as the relative offset of instructions causing
 * relax requests within dynamic shared objects, without having to
 * guess it roughly from /proc/pid/maps, or relying on ldd's
 * --function-relocs feature, which both require to run on the target
 * system to get the needed information. Instead, we allow a build
 * host to use a cross-compilation toolchain later to extract the
 * source location, from the raw data the kernel has provided on the
 * target system.
 *
 * However, collecting the call frames within the application to
 * determine the full context of a relax spot is not something we can
 * do purely from kernel space, notably because it depends on build
 * options we just don't know about (e.g. frame pointers availability
 * for the app, or other nitty-gritty details depending on the
 * toolchain). To solve this, we ask the application to send us a
 * complete backtrace taken from the context of a specific signal
 * handler, which we know is stacked over the relax spot. That
 * information is then stored by the kernel after some
 * post-processing, along with other data identifying the caller, and
 * made available through the /proc/xenomai/debug/relax vfile.
 *
 * Implementation-wise, xndebug_notify_relax and xndebug_trace_relax
 * routines are paired: first, xndebug_notify_relax sends a SIGSHADOW
 * request to userland when a relax spot is detected from
 * xnthread_relax, which should then trigger a call back to
 * xndebug_trace_relax with the complete backtrace information, as
 * seen from userland (via the internal sc_cobalt_backtrace
 * syscall). All this runs on behalf of the relaxing thread, so we can
 * make a number of convenient assumptions (such as being able to scan
 * the current vma list to get detailed information about the
 * executable mappings that could be involved).
 */

void xndebug_notify_relax(struct xnthread *thread, int reason)
{
	xnthread_signal(thread, SIGSHADOW,
			  sigshadow_int(SIGSHADOW_ACTION_BACKTRACE, reason));
}

void xndebug_trace_relax(int nr, unsigned long *backtrace,
			 int reason)
{
	struct relax_record *p, **h;
	struct vm_area_struct *vma;
	struct xnthread *thread;
	struct relax_spot spot;
	struct mm_struct *mm;
	struct file *file;
	unsigned long pc;
	char *mapname;
	int n, depth;
	char *tmp;
	u32 hash;
	spl_t s;

	thread = xnthread_current();
	if (thread == NULL)
		return;		/* Can't be, right? What a mess. */

	/*
	 * We compute PC values relative to the base of the shared
	 * executable mappings we find in the backtrace, which makes
	 * it possible for the slackspot utility to match the
	 * corresponding source code locations from unrelocated file
	 * offsets.
	 */

	tmp = (char *)__get_free_page(GFP_KERNEL);
	if (tmp == NULL)
		/*
		 * The situation looks really bad, but we can't do
		 * anything about it. Just bail out.
		 */
		return;

	memset(&spot, 0, sizeof(spot));
	mm = get_task_mm(current);
	down_read(&mm->mmap_sem);

	for (n = 0, depth = 0; n < nr; n++) {
		pc = backtrace[n];

		vma = find_vma(mm, pc);
		if (vma == NULL)
			continue;

		/*
		 * Hack. Unlike DSOs, executables and interpreters
		 * (e.g. dynamic linkers) are protected against write
		 * attempts. Use this to determine when $pc should be
		 * fixed up by subtracting the mapping base address in
		 * the DSO case.
		 */
		if (!(vma->vm_flags & VM_DENYWRITE))
			pc -= vma->vm_start;

		spot.backtrace[depth].pc = pc;

		/*
		 * Even in case we can't fetch the map name, we still
		 * record the PC value, which may still give some hint
		 * downstream.
		 */
		file = vma->vm_file;
		if (file == NULL)
			goto next_frame;

		mapname = d_path(&file->f_path, tmp, PAGE_SIZE);
		if (IS_ERR(mapname))
			goto next_frame;

		spot.backtrace[depth].mapname = hash_symbol(mapname);
	next_frame:
		depth++;
	}

	up_read(&mm->mmap_sem);
	mmput(mm);
	free_page((unsigned long)tmp);

	/*
	 * Most of the time we will be sent duplicates, since the odds
	 * of seeing the same thread running the same code doing the
	 * same mistake all over again are high. So we probe the hash
	 * table for an identical spot first, before going for a
	 * complete record allocation from the system heap if no match
	 * was found. Otherwise, we just take the fast exit path.
	 */
	spot.depth = depth;
	spot.proghash = thread->proghash;
	spot.pid = xnthread_host_pid(thread);
	spot.reason = reason;
	strcpy(spot.thread, thread->name);
	hash = jhash2((u32 *)&spot, sizeof(spot) / sizeof(u32), 0);

	xnlock_get_irqsave(&relax_lock, s);

	h = &relax_jhash[hash & (RELAX_HSLOTS - 1)];
	p = *h;
	while (p &&
	       /* Try quick guesses first, then memcmp */
	       (p->spot.depth != spot.depth ||
		p->spot.pid != spot.pid ||
		memcmp(&p->spot, &spot, sizeof(spot))))
	       p = p->h_next;

	if (p) {
		p->hits++;
		goto out;	/* Spot already recorded. */
	}

	if (relax_queued >= RELAX_SPOTNR)
		goto out;	/* No more space -- ignore. */
	/*
	 * We can only compete with other shadows which have just
	 * switched to secondary mode like us. So holding the
	 * relax_lock a bit more without disabling interrupts is not
	 * an issue. This allows us to postpone the record memory
	 * allocation while probing and updating the hash table in a
	 * single move.
	 */
	p = xnheap_alloc(&memory_pool, sizeof(*p));
	if (p == NULL)
		goto out;      /* Something is about to go wrong... */

	memcpy(&p->spot, &spot, sizeof(p->spot));
	p->exe_path = hash_symbol(thread->exe_path);
	p->hits = 1;
	p->h_next = *h;
	*h = p;
	p->r_next = relax_record_list;
	relax_record_list = p;
	relax_queued++;
out:
	relax_overall++;

	xnlock_put_irqrestore(&relax_lock, s);
}

static DEFINE_VFILE_HOSTLOCK(relax_mutex);

struct relax_vfile_priv {
	int queued;
	int overall;
	int ncurr;
	struct relax_record *head;
	struct relax_record *curr;
};

static void *relax_vfile_begin(struct xnvfile_regular_iterator *it)
{
	struct relax_vfile_priv *priv = xnvfile_iterator_priv(it);
	struct relax_record *p;
	spl_t s;
	int n;

	/*
	 * Snapshot the counters under lock, to make sure they remain
	 * mutually consistent despite we dump the record list in a
	 * lock-less manner. Additionally, the vfile layer already
	 * holds the relax_mutex lock for us, so that we can't race
	 * with ->store().
	 */
	xnlock_get_irqsave(&relax_lock, s);

	if (relax_queued == 0 || it->pos > relax_queued) {
		xnlock_put_irqrestore(&relax_lock, s);
		return NULL;
	}
	priv->overall = relax_overall;
	priv->queued = relax_queued;
	priv->head = relax_record_list;

	xnlock_put_irqrestore(&relax_lock, s);

	if (it->pos == 0) {
		priv->curr = NULL;
		priv->ncurr = -1;
		return VFILE_SEQ_START;
	}

	for (n = 1, p = priv->head; n < it->pos; n++)
		p = p->r_next;

	priv->curr = p;
	priv->ncurr = n;

	return p;
}

static void *relax_vfile_next(struct xnvfile_regular_iterator *it)
{
	struct relax_vfile_priv *priv = xnvfile_iterator_priv(it);
	struct relax_record *p;
	int n;

	if (it->pos > priv->queued)
		return NULL;

	if (it->pos == priv->ncurr + 1)
		p = priv->curr->r_next;
	else {
		for (n = 1, p = priv->head; n < it->pos; n++)
			p = p->r_next;
	}

	priv->curr = p;
	priv->ncurr = it->pos;

	return p;
}

static const char *reason_str[] = {
    [SIGDEBUG_UNDEFINED] = "undefined",
    [SIGDEBUG_MIGRATE_SIGNAL] = "signal",
    [SIGDEBUG_MIGRATE_SYSCALL] = "syscall",
    [SIGDEBUG_MIGRATE_FAULT] = "fault",
    [SIGDEBUG_MIGRATE_PRIOINV] = "pi-error",
    [SIGDEBUG_NOMLOCK] = "mlock-check",
    [SIGDEBUG_WATCHDOG] = "runaway-break",
    [SIGDEBUG_RESCNT_IMBALANCE] = "resource-count-imbalance",
    [SIGDEBUG_MUTEX_SLEEP] = "sleep-holding-mutex",
    [SIGDEBUG_LOCK_BREAK] = "scheduler-lock-break",
};

static int relax_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct relax_vfile_priv *priv = xnvfile_iterator_priv(it);
	struct relax_record *p = data;
	int n;

	/*
	 * No need to grab any lock to read a record from a previously
	 * validated index: the data must be there and won't be
	 * touched anymore.
	 */
	if (p == NULL) {
		xnvfile_printf(it, "%d\n", priv->overall);
		return 0;
	}

	xnvfile_printf(it, "%s\n", p->exe_path ?: "?");
	xnvfile_printf(it, "%d %d %s %s\n", p->spot.pid, p->hits,
		       reason_str[p->spot.reason], p->spot.thread);

	for (n = 0; n < p->spot.depth; n++)
		xnvfile_printf(it, "0x%lx %s\n",
			       p->spot.backtrace[n].pc,
			       p->spot.backtrace[n].mapname ?: "?");

	xnvfile_printf(it, ".\n");

	return 0;
}

static ssize_t relax_vfile_store(struct xnvfile_input *input)
{
	struct relax_record *p, *np;
	spl_t s;

	/*
	 * Flush out all records. Races with ->show() are prevented
	 * using the relax_mutex lock. The vfile layer takes care of
	 * this internally.
	 */
	xnlock_get_irqsave(&relax_lock, s);
	p = relax_record_list;
	relax_record_list = NULL;
	relax_overall = 0;
	relax_queued = 0;
	memset(relax_jhash, 0, sizeof(relax_jhash));
	xnlock_put_irqrestore(&relax_lock, s);

	while (p) {
		np = p->r_next;
		xnheap_free(&memory_pool, p);
		p = np;
	}

	return input->size;
}

static struct xnvfile_regular_ops relax_vfile_ops = {
	.begin = relax_vfile_begin,
	.next = relax_vfile_next,
	.show = relax_vfile_show,
	.store = relax_vfile_store,
};

static struct xnvfile_regular relax_vfile = {
	.privsz = sizeof(struct relax_vfile_priv),
	.ops = &relax_vfile_ops,
	.entry = { .lockops = &relax_mutex.ops },
};

static inline int init_trace_relax(void)
{
	u32 size = CONFIG_XENO_OPT_DEBUG_TRACE_LOGSZ * 1024;
	void *p;
	int ret;

	p = vmalloc(size);
	if (p == NULL)
		return -ENOMEM;

	ret = xnheap_init(&memory_pool, p, size);
	if (ret)
		return ret;

	xnheap_set_name(&memory_pool, "debug log");

	ret = xnvfile_init_regular("relax", &relax_vfile, &cobalt_debug_vfroot);
	if (ret) {
		xnheap_destroy(&memory_pool);
		vfree(p);
	}

	return ret;
}

static inline void cleanup_trace_relax(void)
{
	void *p;

	xnvfile_destroy_regular(&relax_vfile);
	p = xnheap_get_membase(&memory_pool);
	xnheap_destroy(&memory_pool);
	vfree(p);
}

#else /* !CONFIG_XENO_OPT_DEBUG_TRACE_RELAX */

static inline int init_trace_relax(void)
{
	return 0;
}

static inline void cleanup_trace_relax(void)
{
}

static inline void init_thread_relax_trace(struct xnthread *thread)
{
}

#endif /* !XENO_OPT_DEBUG_TRACE_RELAX */

#ifdef CONFIG_XENO_OPT_DEBUG_LOCKING

void xnlock_dbg_prepare_acquire(unsigned long long *start)
{
	*start = xnclock_read_raw(&nkclock);
}
EXPORT_SYMBOL_GPL(xnlock_dbg_prepare_acquire);

void xnlock_dbg_acquired(struct xnlock *lock, int cpu, unsigned long long *start,
			 const char *file, int line, const char *function)
{
	lock->lock_date = *start;
	lock->spin_time = xnclock_read_raw(&nkclock) - *start;
	lock->file = file;
	lock->function = function;
	lock->line = line;
	lock->cpu = cpu;
}
EXPORT_SYMBOL_GPL(xnlock_dbg_acquired);

int xnlock_dbg_release(struct xnlock *lock,
		       const char *file, int line, const char *function)
{
	unsigned long long lock_time;
	struct xnlockinfo *stats;
	int cpu;

	lock_time = xnclock_read_raw(&nkclock) - lock->lock_date;
	cpu = ipipe_processor_id();
	stats = &per_cpu(xnlock_stats, cpu);

	if (lock->file == NULL) {
		lock->file = "??";
		lock->line = 0;
		lock->function = "invalid";
	}

	if (unlikely(lock->owner != cpu)) {
		ipipe_prepare_panic();
		printk(XENO_ERR "lock %p already unlocked on CPU #%d\n"
				"          last owner = %s:%u (%s(), CPU #%d)\n",
		       lock, cpu, lock->file, lock->line, lock->function,
		       lock->cpu);
		show_stack(NULL,NULL);
		return 1;
	}

	/* File that we released it. */
	lock->cpu = -lock->cpu;
	lock->file = file;
	lock->line = line;
	lock->function = function;

	if (lock_time > stats->lock_time) {
		stats->lock_time = lock_time;
		stats->spin_time = lock->spin_time;
		stats->file = lock->file;
		stats->function = lock->function;
		stats->line = lock->line;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xnlock_dbg_release);

#endif /* CONFIG_XENO_OPT_DEBUG_LOCKING */

void xndebug_shadow_init(struct xnthread *thread)
{
	struct cobalt_ppd *sys_ppd;
	size_t len;

	sys_ppd = cobalt_ppd_get(0);
	/*
	 * The caller is current, so we know for sure that sys_ppd
	 * will still be valid after we dropped the lock.
	 *
	 * NOTE: Kernel shadows all share the system global ppd
	 * descriptor with no refcounting.
	 */
	thread->exe_path = sys_ppd->exe_path ?: "(unknown)";
	/*
	 * The program hash value is a unique token debug features may
	 * use to identify all threads which belong to a given
	 * executable file. Using this value for quick probes is often
	 * handier and more efficient than testing the whole exe_path.
	 */
	len = strlen(thread->exe_path);
	thread->proghash = jhash(thread->exe_path, len, 0);
}

int xndebug_init(void)
{
	int ret;

	ret = init_trace_relax();
	if (ret)
		return ret;

	return 0;
}

void xndebug_cleanup(void)
{
	cleanup_trace_relax();
}

/** @} */
