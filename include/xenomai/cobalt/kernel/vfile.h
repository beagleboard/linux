/*
 * Copyright (C) 2010 Philippe Gerum <rpm@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or
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

#ifndef _COBALT_KERNEL_VFILE_H
#define _COBALT_KERNEL_VFILE_H

#if defined(CONFIG_XENO_OPT_VFILE) || defined(DOXYGEN_CPP)

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <cobalt/kernel/lock.h>

/**
 * @addtogroup cobalt_core_vfile
 * @{
 */

struct xnvfile_directory;
struct xnvfile_regular_iterator;
struct xnvfile_snapshot_iterator;
struct xnvfile_lock_ops;

struct xnvfile {
	struct proc_dir_entry *pde;
	struct file *file;
	struct xnvfile_lock_ops *lockops;
	int refcnt;
	void *private;
};

/**
 * @brief Vfile locking operations
 * @anchor vfile_lockops
 *
 * This structure describes the operations to be provided for
 * implementing locking support on vfiles. They apply to both
 * snapshot-driven and regular vfiles.
 */
struct xnvfile_lock_ops {
	/**
	 * @anchor lockops_get
	 * This handler should grab the desired lock.
	 *
	 * @param vfile A pointer to the virtual file which needs
	 * locking.
	 *
	 * @return zero should be returned if the call
	 * succeeds. Otherwise, a negative error code can be returned;
	 * upon error, the current vfile operation is aborted, and the
	 * user-space caller is passed back the error value.
	 */
	int (*get)(struct xnvfile *vfile);
	/**
	 * @anchor lockops_put This handler should release the lock
	 * previously grabbed by the @ref lockops_get "get() handler".
	 *
	 * @param vfile A pointer to the virtual file which currently
	 * holds the lock to release.
	 */
	void (*put)(struct xnvfile *vfile);
};

struct xnvfile_hostlock_class {
	struct xnvfile_lock_ops ops;
	struct mutex mutex;
};

struct xnvfile_nklock_class {
	struct xnvfile_lock_ops ops;
	spl_t s;
};

struct xnvfile_input {
	const char __user *u_buf;
	size_t size;
	struct xnvfile *vfile;
};

/**
 * @brief Regular vfile operation descriptor
 * @anchor regular_ops
 *
 * This structure describes the operations available with a regular
 * vfile. It defines handlers for sending back formatted kernel data
 * upon a user-space read request, and for obtaining user data upon a
 * user-space write request.
 */
struct xnvfile_regular_ops {
	/**
	 * @anchor regular_rewind This handler is called only once,
	 * when the virtual file is opened, before the @ref
	 * regular_begin "begin() handler" is invoked.
	 *
	 * @param it A pointer to the vfile iterator which will be
	 * used to read the file contents.
	 *
	 * @return Zero should be returned upon success. Otherwise, a
	 * negative error code aborts the operation, and is passed
	 * back to the reader.
	 *
	 * @note This handler is optional. It should not be used to
	 * allocate resources but rather to perform consistency
	 * checks, since no closure call is issued in case the open
	 * sequence eventually fails.
	 */
	int (*rewind)(struct xnvfile_regular_iterator *it);
	/**
	 * @anchor regular_begin
	 * This handler should prepare for iterating over the records
	 * upon a read request, starting from the specified position.
	 *
	 * @param it A pointer to the current vfile iterator. On
	 * entry, it->pos is set to the (0-based) position of the
	 * first record to output. This handler may be called multiple
	 * times with different position requests.
	 *
	 * @return A pointer to the first record to format and output,
	 * to be passed to the @ref regular_show "show() handler" as
	 * its @a data parameter, if the call succeeds. Otherwise:
	 *
	 * - NULL in case no record is available, in which case the
	 * read operation will terminate immediately with no output.
	 *
	 * - VFILE_SEQ_START, a special value indicating that @ref
	 * regular_show "the show() handler" should receive a NULL
	 * data pointer first, in order to output a header.
	 *
	 * - ERR_PTR(errno), where errno is a negative error code;
	 * upon error, the current operation will be aborted
	 * immediately.
	 *
	 * @note This handler is optional; if none is given in the
	 * operation descriptor (i.e. NULL value), the @ref
	 * regular_show "show() handler()" will be called only once
	 * for a read operation, with a NULL @a data parameter. This
	 * particular setting is convenient for simple regular vfiles
	 * having a single, fixed record to output.
	 */
	void *(*begin)(struct xnvfile_regular_iterator *it);
	/**
	 * @anchor regular_next
	 * This handler should return the address of the next record
	 * to format and output by the @ref regular_show "show()
	 * handler".
	 *
	 * @param it A pointer to the current vfile iterator. On
	 * entry, it->pos is set to the (0-based) position of the
	 * next record to output.
	 *
	 * @return A pointer to the next record to format and output,
	 * to be passed to the @ref regular_show "show() handler" as
	 * its @a data parameter, if the call succeeds. Otherwise:
	 *
	 * - NULL in case no record is available, in which case the
	 * read operation will terminate immediately with no output.
	 *
	 * - ERR_PTR(errno), where errno is a negative error code;
	 * upon error, the current operation will be aborted
	 * immediately.
	 *
	 * @note This handler is optional; if none is given in the
	 * operation descriptor (i.e. NULL value), the read operation
	 * will stop after the first invocation of the @ref regular_show
	 * "show() handler".
	 */
	void *(*next)(struct xnvfile_regular_iterator *it);
	/**
	 * @anchor regular_end
	 * This handler is called after all records have been output.
	 *
	 * @param it A pointer to the current vfile iterator.
	 *
	 * @note This handler is optional and the pointer may be NULL.
	 */
	void (*end)(struct xnvfile_regular_iterator *it);
	/**
	 * @anchor regular_show
	 * This handler should format and output a record.
	 *
	 * xnvfile_printf(), xnvfile_write(), xnvfile_puts() and
	 * xnvfile_putc() are available to format and/or emit the
	 * output. All routines take the iterator argument @a it as
	 * their first parameter.
	 *
	 * @param it A pointer to the current vfile iterator.
	 *
	 * @param data A pointer to the record to format then
	 * output. The first call to the handler may receive a NULL @a
	 * data pointer, depending on the presence and/or return of a
	 * @ref regular_begin "hander"; the show handler should test
	 * this special value to output any header that fits, prior to
	 * receiving more calls with actual records.
	 *
	 * @return zero if the call succeeds, also indicating that the
	 * handler should be called for the next record if
	 * any. Otherwise:
	 *
	 * - A negative error code. This will abort the output phase,
	 * and return this status to the reader.
	 *
	 * - VFILE_SEQ_SKIP, a special value indicating that the
	 * current record should be skipped and will not be output.
	 */
	int (*show)(struct xnvfile_regular_iterator *it, void *data);
	/**
	 * @anchor regular_store
	 * This handler receives data written to the vfile, likely for
	 * updating some kernel setting, or triggering any other
	 * action which fits. This is the only handler which deals
	 * with the write-side of a vfile.  It is called when writing
	 * to the /proc entry of the vfile from a user-space process.
	 *
	 * The input data is described by a descriptor passed to the
	 * handler, which may be subsequently passed to parsing helper
	 * routines.  For instance, xnvfile_get_string() will accept
	 * the input descriptor for returning the written data as a
	 * null-terminated character string. On the other hand,
	 * xnvfile_get_integer() will attempt to return a long integer
	 * from the input data.
	 *
	 * @param input A pointer to an input descriptor. It refers to
	 * an opaque data from the handler's standpoint.
	 *
	 * @return the number of bytes read from the input descriptor
	 * if the call succeeds. Otherwise, a negative error code.
	 * Return values from parsing helper routines are commonly
	 * passed back to the caller by the @ref regular_store
	 * "store() handler".
	 *
	 * @note This handler is optional, and may be omitted for
	 * read-only vfiles.
	 */
	ssize_t (*store)(struct xnvfile_input *input);
};

struct xnvfile_regular {
	struct xnvfile entry;
	size_t privsz;
	struct xnvfile_regular_ops *ops;
};

struct xnvfile_regular_template {
	size_t privsz;
	struct xnvfile_regular_ops *ops;
	struct xnvfile_lock_ops *lockops;
};

/**
 * @brief Regular vfile iterator
 * @anchor regular_iterator
 *
 * This structure defines an iterator over a regular vfile.
 */
struct xnvfile_regular_iterator {
	/** Current record position while iterating. */
	loff_t pos;
	/** Backlink to the host sequential file supporting the vfile. */
	struct seq_file *seq;
	/** Backlink to the vfile being read. */
	struct xnvfile_regular *vfile;
	/**
	 * Start of private area. Use xnvfile_iterator_priv() to
	 * address it.
	 */
	char private[0];
};

/**
 * @brief Snapshot vfile operation descriptor
 * @anchor snapshot_ops
 *
 * This structure describes the operations available with a
 * snapshot-driven vfile. It defines handlers for returning a
 * printable snapshot of some Xenomai object contents upon a
 * user-space read request, and for updating this object upon a
 * user-space write request.
 */
struct xnvfile_snapshot_ops {
	/**
	 * @anchor snapshot_rewind
	 * This handler (re-)initializes the data collection, moving
	 * the seek pointer at the first record. When the file
	 * revision tag is touched while collecting data, the current
	 * reading is aborted, all collected data dropped, and the
	 * vfile is eventually rewound.
	 *
	 * @param it A pointer to the current snapshot iterator. Two
	 * useful information can be retrieved from this iterator in
	 * this context:
	 *
	 * - it->vfile is a pointer to the descriptor of the virtual
	 * file being rewound.
	 *
	 * - xnvfile_iterator_priv(it) returns a pointer to the
	 * private data area, available from the descriptor, which
	 * size is vfile->privsz. If the latter size is zero, the
	 * returned pointer is meaningless and should not be used.
	 *
	 * @return A negative error code aborts the data collection,
	 * and is passed back to the reader. Otherwise:
	 *
	 * - a strictly positive value is interpreted as the total
	 * number of records which will be returned by the @ref
	 * snapshot_next "next() handler" during the data collection
	 * phase. If no @ref snapshot_begin "begin() handler" is
	 * provided in the @ref snapshot_ops "operation descriptor",
	 * this value is used to allocate the snapshot buffer
	 * internally. The size of this buffer would then be
	 * vfile->datasz * value.
	 *
	 * - zero leaves the allocation to the @ref snapshot_begin
	 * "begin() handler" if present, or indicates that no record
	 * is to be output in case such handler is not given.
	 *
	 * @note This handler is optional; a NULL value indicates that
	 * nothing needs to be done for rewinding the vfile.  It is
	 * called with the vfile lock held.
	 */
	int (*rewind)(struct xnvfile_snapshot_iterator *it);
	/**
	 * @anchor snapshot_begin
	 * This handler should allocate the snapshot buffer to hold
	 * records during the data collection phase.  When specified,
	 * all records collected via the @ref snapshot_next "next()
	 * handler" will be written to a cell from the memory area
	 * returned by begin().
	 *
	 * @param it A pointer to the current snapshot iterator.
	 *
	 * @return A pointer to the record buffer, if the call
	 * succeeds. Otherwise:
	 *
	 * - NULL in case of allocation error. This will abort the data
	 * collection, and return -ENOMEM to the reader.
	 *
	 * - VFILE_SEQ_EMPTY, a special value indicating that no
	 * record will be output. In such a case, the @ref
	 * snapshot_next "next() handler" will not be called, and the
	 * data collection will stop immediately. However, the @ref
	 * snapshot_show "show() handler" will still be called once,
	 * with a NULL data pointer (i.e. header display request).
	 *
	 * @note This handler is optional; if none is given, an
	 * internal allocation depending on the value returned by the
	 * @ref snapshot_rewind "rewind() handler" can be obtained.
	 */
	void *(*begin)(struct xnvfile_snapshot_iterator *it);
	/**
	 * @anchor snapshot_end
	 * This handler releases the memory buffer previously obtained
	 * from begin(). It is usually called after the snapshot data
	 * has been output by show(), but it may also be called before
	 * rewinding the vfile after a revision change, to release the
	 * dropped buffer.
	 *
	 * @param it A pointer to the current snapshot iterator.
	 *
	 * @param buf A pointer to the buffer to release.
	 *
	 * @note This routine is optional and the pointer may be
	 * NULL. It is not needed upon internal buffer allocation;
	 * see the description of the @ref snapshot_rewind "rewind()
	 * handler".
	 */
	void (*end)(struct xnvfile_snapshot_iterator *it, void *buf);
	/**
	 * @anchor snapshot_next
	 * This handler fetches the next record, as part of the
	 * snapshot data to be sent back to the reader via the
	 * show().
	 *
	 * @param it A pointer to the current snapshot iterator.
	 *
	 * @param data A pointer to the record to fill in.
	 *
	 * @return a strictly positive value, if the call succeeds and
	 * leaves a valid record into @a data, which should be passed
	 * to the @ref snapshot_show "show() handler()" during the
	 * formatting and output phase. Otherwise:
	 *
	 * - A negative error code. This will abort the data
	 * collection, and return this status to the reader.
	 *
	 * - VFILE_SEQ_SKIP, a special value indicating that the
	 * current record should be skipped. In such a case, the @a
	 * data pointer is not advanced to the next position before
	 * the @ref snapshot_next "next() handler" is called anew.
	 *
	 * @note This handler is called with the vfile lock
	 * held. Before each invocation of this handler, the vfile
	 * core checks whether the revision tag has been touched, in
	 * which case the data collection is restarted from scratch. A
	 * data collection phase succeeds whenever all records can be
	 * fetched via the @ref snapshot_next "next() handler", while
	 * the revision tag remains unchanged, which indicates that a
	 * consistent snapshot of the object state was taken.
	 */
	int (*next)(struct xnvfile_snapshot_iterator *it, void *data);
	/**
	 * @anchor snapshot_show
	 * This handler should format and output a record from the
	 * collected data.
	 *
	 * xnvfile_printf(), xnvfile_write(), xnvfile_puts() and
	 * xnvfile_putc() are available to format and/or emit the
	 * output. All routines take the iterator argument @a it as
	 * their first parameter.
	 *
	 * @param it A pointer to the current snapshot iterator.
	 *
	 * @param data A pointer to the record to format then
	 * output. The first call to the handler is always passed a
	 * NULL @a data pointer; the show handler should test this
	 * special value to output any header that fits, prior to
	 * receiving more calls with actual records.
	 *
	 * @return zero if the call succeeds, also indicating that the
	 * handler should be called for the next record if
	 * any. Otherwise:
	 *
	 * - A negative error code. This will abort the output phase,
	 * and return this status to the reader.
	 *
	 * - VFILE_SEQ_SKIP, a special value indicating that the
	 * current record should be skipped and will not be output.
	 */
	int (*show)(struct xnvfile_snapshot_iterator *it, void *data);
	/**
	 * @anchor snapshot_store
	 * This handler receives data written to the vfile, likely for
	 * updating the associated Xenomai object's state, or
	 * triggering any other action which fits. This is the only
	 * handler which deals with the write-side of a vfile.  It is
	 * called when writing to the /proc entry of the vfile
	 * from a user-space process.
	 *
	 * The input data is described by a descriptor passed to the
	 * handler, which may be subsequently passed to parsing helper
	 * routines.  For instance, xnvfile_get_string() will accept
	 * the input descriptor for returning the written data as a
	 * null-terminated character string. On the other hand,
	 * xnvfile_get_integer() will attempt to return a long integer
	 * from the input data.
	 *
	 * @param input A pointer to an input descriptor. It refers to
	 * an opaque data from the handler's standpoint.
	 *
	 * @return the number of bytes read from the input descriptor
	 * if the call succeeds. Otherwise, a negative error code.
	 * Return values from parsing helper routines are commonly
	 * passed back to the caller by the @ref snapshot_store
	 * "store() handler".
	 *
	 * @note This handler is optional, and may be omitted for
	 * read-only vfiles.
	 */
	ssize_t (*store)(struct xnvfile_input *input);
};

/**
 * @brief Snapshot revision tag
 * @anchor revision_tag
 *
 * This structure defines a revision tag to be used with @ref
 * snapshot_vfile "snapshot-driven vfiles".
 */
struct xnvfile_rev_tag {
	/** Current revision number. */
	int rev;
};

struct xnvfile_snapshot_template {
	size_t privsz;
	size_t datasz;
	struct xnvfile_rev_tag *tag;
	struct xnvfile_snapshot_ops *ops;
	struct xnvfile_lock_ops *lockops;
};

/**
 * @brief Snapshot vfile descriptor
 * @anchor snapshot_vfile
 *
 * This structure describes a snapshot-driven vfile.  Reading from
 * such a vfile involves a preliminary data collection phase under
 * lock protection, and a subsequent formatting and output phase of
 * the collected data records. Locking is done in a way that does not
 * increase worst-case latency, regardless of the number of records to
 * be collected for output.
 */
struct xnvfile_snapshot {
	struct xnvfile entry;
	size_t privsz;
	size_t datasz;
	struct xnvfile_rev_tag *tag;
	struct xnvfile_snapshot_ops *ops;
};

/**
 * @brief Snapshot-driven vfile iterator
 * @anchor snapshot_iterator
 *
 * This structure defines an iterator over a snapshot-driven vfile.
 */
struct xnvfile_snapshot_iterator {
	/** Number of collected records. */
	int nrdata;
	/** Address of record buffer. */
	caddr_t databuf;
	/** Backlink to the host sequential file supporting the vfile. */
	struct seq_file *seq;
	/** Backlink to the vfile being read. */
	struct xnvfile_snapshot *vfile;
	/** Buffer release handler. */
	void (*endfn)(struct xnvfile_snapshot_iterator *it, void *buf);
	/**
	 * Start of private area. Use xnvfile_iterator_priv() to
	 * address it.
	 */
	char private[0];
};

struct xnvfile_directory {
	struct xnvfile entry;
};

struct xnvfile_link {
	struct xnvfile entry;
};

/* vfile.begin()=> */
#define VFILE_SEQ_EMPTY			((void *)-1)
/* =>vfile.show() */
#define VFILE_SEQ_START			SEQ_START_TOKEN
/* vfile.next/show()=> */
#define VFILE_SEQ_SKIP			2

#define xnvfile_printf(it, args...)	seq_printf((it)->seq, ##args)
#define xnvfile_write(it, data, len)	seq_write((it)->seq, (data),(len))
#define xnvfile_puts(it, s)		seq_puts((it)->seq, (s))
#define xnvfile_putc(it, c)		seq_putc((it)->seq, (c))

static inline void xnvfile_touch_tag(struct xnvfile_rev_tag *tag)
{
	tag->rev++;
}

static inline void xnvfile_touch(struct xnvfile_snapshot *vfile)
{
	xnvfile_touch_tag(vfile->tag);
}

#define xnvfile_noentry			\
	{				\
		.pde = NULL,		\
		.private = NULL,	\
		.file = NULL,		\
		.refcnt = 0,		\
	}

#define xnvfile_nodir	{ .entry = xnvfile_noentry }
#define xnvfile_nolink	{ .entry = xnvfile_noentry }
#define xnvfile_nofile	{ .entry = xnvfile_noentry }

#define xnvfile_priv(e)			((e)->entry.private)
#define xnvfile_nref(e)			((e)->entry.refcnt)
#define xnvfile_file(e)			((e)->entry.file)
#define xnvfile_iterator_priv(it)	((void *)(&(it)->private))

extern struct xnvfile_nklock_class xnvfile_nucleus_lock;

extern struct xnvfile_directory cobalt_vfroot;

int xnvfile_init_root(void);

void xnvfile_destroy_root(void);

int xnvfile_init_snapshot(const char *name,
			  struct xnvfile_snapshot *vfile,
			  struct xnvfile_directory *parent);

int xnvfile_init_regular(const char *name,
			 struct xnvfile_regular *vfile,
			 struct xnvfile_directory *parent);

int xnvfile_init_dir(const char *name,
		     struct xnvfile_directory *vdir,
		     struct xnvfile_directory *parent);

int xnvfile_init_link(const char *from,
		      const char *to,
		      struct xnvfile_link *vlink,
		      struct xnvfile_directory *parent);

void xnvfile_destroy(struct xnvfile *vfile);

ssize_t xnvfile_get_blob(struct xnvfile_input *input,
			 void *data, size_t size);

ssize_t xnvfile_get_string(struct xnvfile_input *input,
			   char *s, size_t maxlen);

ssize_t xnvfile_get_integer(struct xnvfile_input *input, long *valp);

int __vfile_hostlock_get(struct xnvfile *vfile);

void __vfile_hostlock_put(struct xnvfile *vfile);

static inline
void xnvfile_destroy_snapshot(struct xnvfile_snapshot *vfile)
{
	xnvfile_destroy(&vfile->entry);
}

static inline
void xnvfile_destroy_regular(struct xnvfile_regular *vfile)
{
	xnvfile_destroy(&vfile->entry);
}

static inline
void xnvfile_destroy_dir(struct xnvfile_directory *vdir)
{
	xnvfile_destroy(&vdir->entry);
}

static inline
void xnvfile_destroy_link(struct xnvfile_link *vlink)
{
	xnvfile_destroy(&vlink->entry);
}

#define DEFINE_VFILE_HOSTLOCK(name)					\
	struct xnvfile_hostlock_class name = {				\
		.ops = {						\
			.get = __vfile_hostlock_get,			\
			.put = __vfile_hostlock_put,			\
		},							\
		.mutex = __MUTEX_INITIALIZER(name.mutex),		\
	}

#else /* !CONFIG_XENO_OPT_VFILE */

#define xnvfile_touch_tag(tag)	do { } while (0)

#define xnvfile_touch(vfile)	do { } while (0)

#endif /* !CONFIG_XENO_OPT_VFILE */

/** @} */

#endif /* !_COBALT_KERNEL_VFILE_H */
