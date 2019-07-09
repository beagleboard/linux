/*
 * Copyright (C) 2010 Philippe Gerum <rpm@xenomai.org>
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
#include <stdarg.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/vfile.h>
#include <asm/xenomai/wrappers.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_vfile Virtual file services
 *
 * Virtual files provide a mean to export Xenomai object states to
 * user-space, based on common kernel interfaces.  This encapsulation
 * is aimed at:
 *
 * - supporting consistent collection of very large record-based
 * output, without encurring latency peaks for undergoing real-time
 * activities.
 *
 * - in the future, hiding discrepancies between linux kernel
 * releases, regarding the proper way to export kernel object states
 * to userland, either via the /proc interface or by any other mean.
 *
 * This virtual file implementation offers record-based read support
 * based on seq_files, single-buffer write support, directory and link
 * handling, all visible from the /proc namespace.
 *
 * The vfile support exposes four filesystem object types:
 *
 * - snapshot-driven file (struct xnvfile_snapshot). This is commonly
 * used to export real-time object states via the /proc filesystem. To
 * minimize the latency involved in protecting the vfile routines from
 * changes applied by real-time code on such objects, a snapshot of
 * the data to output is first taken under proper locking, before the
 * collected data is formatted and sent out in a lockless manner.
 *
 * Because a large number of records may have to be output, the data
 * collection phase is not strictly atomic as a whole, but only
 * protected at record level. The vfile implementation can be notified
 * of updates to the underlying data set, and restart the collection
 * from scratch until the snapshot is fully consistent.
 *
 * - regular sequential file (struct xnvfile_regular). This is
 * basically an encapsulated sequential file object as available from
 * the host kernel (i.e. seq_file), with a few additional features to
 * make it more handy in a Xenomai environment, like implicit locking
 * support and shortened declaration for simplest, single-record
 * output.
 *
 * - virtual link (struct xnvfile_link). This is a symbolic link
 * feature integrated with the vfile semantics. The link target is
 * computed dynamically at creation time from a user-given helper
 * routine.
 *
 * - virtual directory (struct xnvfile_directory). A directory object,
 * which can be used to create a hierarchy for ordering a set of vfile
 * objects.
 *
 *@{*/

/**
 * @var struct xnvfile_directory cobalt_vfroot
 * @brief Xenomai vfile root directory
 *
 * This vdir maps the /proc/xenomai directory. It can be used to
 * create a hierarchy of Xenomai-related vfiles under this root.
 */
struct xnvfile_directory cobalt_vfroot;
EXPORT_SYMBOL_GPL(cobalt_vfroot);

static struct xnvfile_directory sysroot;

static void *vfile_snapshot_start(struct seq_file *seq, loff_t *offp)
{
	struct xnvfile_snapshot_iterator *it = seq->private;
	loff_t pos = *offp;

	if (pos > it->nrdata)
		return NULL;

	if (pos == 0)
		return SEQ_START_TOKEN;

	return it->databuf + (pos - 1) * it->vfile->datasz;
}

static void *vfile_snapshot_next(struct seq_file *seq, void *v, loff_t *offp)
{
	struct xnvfile_snapshot_iterator *it = seq->private;
	loff_t pos = *offp;

	if (pos >= it->nrdata)
		return NULL;

	++*offp;

	return it->databuf + pos * it->vfile->datasz;
}

static void vfile_snapshot_stop(struct seq_file *seq, void *v)
{
}

static int vfile_snapshot_show(struct seq_file *seq, void *v)
{
	struct xnvfile_snapshot_iterator *it = seq->private;
	void *data = v == SEQ_START_TOKEN ? NULL : v;
	int ret;

	ret = it->vfile->ops->show(it, data);

	return ret == VFILE_SEQ_SKIP ? SEQ_SKIP : ret;
}

static struct seq_operations vfile_snapshot_ops = {
	.start = vfile_snapshot_start,
	.next = vfile_snapshot_next,
	.stop = vfile_snapshot_stop,
	.show = vfile_snapshot_show
};

static void vfile_snapshot_free(struct xnvfile_snapshot_iterator *it, void *buf)
{
	kfree(buf);
}

static int vfile_snapshot_open(struct inode *inode, struct file *file)
{
	struct xnvfile_snapshot *vfile = PDE_DATA(inode);
	struct xnvfile_snapshot_ops *ops = vfile->ops;
	struct xnvfile_snapshot_iterator *it;
	int revtag, ret, nrdata;
	struct seq_file *seq;
	caddr_t data;

	if ((file->f_mode & FMODE_WRITE) != 0 && ops->store == NULL)
		return -EACCES;

	/*
	 * Make sure to create the seq_file backend only when reading
	 * from the v-file is possible.
	 */
	if ((file->f_mode & FMODE_READ) == 0) {
		file->private_data = NULL;
		return 0;
	}

	if ((file->f_flags & O_EXCL) != 0 && xnvfile_nref(vfile) > 0)
		return -EBUSY;

	it = kzalloc(sizeof(*it) + vfile->privsz, GFP_KERNEL);
	if (it == NULL)
		return -ENOMEM;

	it->vfile = vfile;
	xnvfile_file(vfile) = file;

	ret = vfile->entry.lockops->get(&vfile->entry);
	if (ret)
		goto fail;
redo:
	/*
	 * The ->rewind() method is optional; there may be cases where
	 * we don't have to take an atomic snapshot of the v-file
	 * contents before proceeding. In case ->rewind() detects a
	 * stale backend object, it can force us to bail out.
	 *
	 * If present, ->rewind() may return a strictly positive
	 * value, indicating how many records at most may be returned
	 * by ->next(). We use this hint to allocate the snapshot
	 * buffer, in case ->begin() is not provided. The size of this
	 * buffer would then be vfile->datasz * hint value.
	 *
	 * If ->begin() is given, we always expect the latter do the
	 * allocation for us regardless of the hint value. Otherwise,
	 * a NULL return from ->rewind() tells us that the vfile won't
	 * output any snapshot data via ->show().
	 */
	nrdata = 0;
	if (ops->rewind) {
		nrdata = ops->rewind(it);
		if (nrdata < 0) {
			ret = nrdata;
			vfile->entry.lockops->put(&vfile->entry);
			goto fail;
		}
	}
	revtag = vfile->tag->rev;

	vfile->entry.lockops->put(&vfile->entry);

	/* Release the data buffer, in case we had to restart. */
	if (it->databuf) {
		it->endfn(it, it->databuf);
		it->databuf = NULL;
	}

	/*
	 * Having no record to output is fine, in which case ->begin()
	 * shall return VFILE_SEQ_EMPTY if present. ->begin() may be
	 * absent, meaning that no allocation is even required to
	 * collect the records to output. NULL is kept for allocation
	 * errors in all other cases.
	 */
	if (ops->begin) {
		XENO_BUG_ON(COBALT, ops->end == NULL);
		data = ops->begin(it);
		if (data == NULL) {
			kfree(it);
			return -ENOMEM;
		}
		if (data != VFILE_SEQ_EMPTY) {
			it->databuf = data;
			it->endfn = ops->end;
		}
	} else if (nrdata > 0 && vfile->datasz > 0) {
		/* We have a hint for auto-allocation. */
		data = kmalloc(vfile->datasz * nrdata, GFP_KERNEL);
		if (data == NULL) {
			kfree(it);
			return -ENOMEM;
		}
		it->databuf = data;
		it->endfn = vfile_snapshot_free;
	}

	ret = seq_open(file, &vfile_snapshot_ops);
	if (ret)
		goto fail;

	it->nrdata = 0;
	data = it->databuf;
	if (data == NULL)
		goto finish;

	/*
	 * Take a snapshot of the vfile contents, redo if the revision
	 * tag of the scanned data set changed concurrently.
	 */
	for (;;) {
		ret = vfile->entry.lockops->get(&vfile->entry);
		if (ret)
			break;
		if (vfile->tag->rev != revtag)
			goto redo;
		ret = ops->next(it, data);
		vfile->entry.lockops->put(&vfile->entry);
		if (ret <= 0)
			break;
		if (ret != VFILE_SEQ_SKIP) {
			data += vfile->datasz;
			it->nrdata++;
		}
	}

	if (ret < 0) {
		seq_release(inode, file);
	fail:
		if (it->databuf)
			it->endfn(it, it->databuf);
		kfree(it);
		return ret;
	}

finish:
	seq = file->private_data;
	it->seq = seq;
	seq->private = it;
	xnvfile_nref(vfile)++;

	return 0;
}

static int vfile_snapshot_release(struct inode *inode, struct file *file)
{
	struct seq_file *seq = file->private_data;
	struct xnvfile_snapshot_iterator *it;

	if (seq) {
		it = seq->private;
		if (it) {
			--xnvfile_nref(it->vfile);
			XENO_BUG_ON(COBALT, it->vfile->entry.refcnt < 0);
			if (it->databuf)
				it->endfn(it, it->databuf);
			kfree(it);
		}

		return seq_release(inode, file);
	}

	return 0;
}

ssize_t vfile_snapshot_write(struct file *file, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	struct xnvfile_snapshot *vfile =
		PDE_DATA(file->f_path.dentry->d_inode);
	struct xnvfile_input input;
	ssize_t ret;

	if (vfile->entry.lockops) {
		ret = vfile->entry.lockops->get(&vfile->entry);
		if (ret)
			return ret;
	}

	input.u_buf = buf;
	input.size = size;
	input.vfile = &vfile->entry;

	ret = vfile->ops->store(&input);

	if (vfile->entry.lockops)
		vfile->entry.lockops->put(&vfile->entry);

	return ret;
}

static struct file_operations vfile_snapshot_fops = {
	.open = vfile_snapshot_open,
	.read = seq_read,
	.write = vfile_snapshot_write,
	.llseek = seq_lseek,
	.release = vfile_snapshot_release,
};

/**
 * @fn int xnvfile_init_snapshot(const char *name, struct xnvfile_snapshot *vfile, struct xnvfile_directory *parent)
 * @brief Initialize a snapshot-driven vfile.
 *
 * @param name The name which should appear in the pseudo-filesystem,
 * identifying the vfile entry.
 *
 * @param vfile A pointer to a vfile descriptor to initialize
 * from. The following fields in this structure should be filled in
 * prior to call this routine:
 *
 * - .privsz is the size (in bytes) of the private data area to be
 * reserved in the @ref snapshot_iterator "vfile iterator". A NULL
 * value indicates that no private area should be reserved.
 *
 * - .datasz is the size (in bytes) of a single record to be collected
 * by the @ref snapshot_next "next() handler" from the @ref
 * snapshot_ops "operation descriptor".
 *
 * - .tag is a pointer to a mandatory vfile revision tag structure
 * (struct xnvfile_rev_tag). This tag will be monitored for changes by
 * the vfile core while collecting data to output, so that any update
 * detected will cause the current snapshot data to be dropped, and
 * the collection to restart from the beginning. To this end, any
 * change to the data which may be part of the collected records,
 * should also invoke xnvfile_touch() on the associated tag.
 *
 * - entry.lockops is a pointer to a @ref vfile_lockops "lock descriptor",
 * defining the lock and unlock operations for the vfile. This pointer
 * may be left to NULL, in which case the operations on the nucleus
 * lock (i.e. nklock) will be used internally around calls to data
 * collection handlers (see @ref snapshot_ops "operation descriptor").
 *
 * - .ops is a pointer to an @ref snapshot_ops "operation descriptor".
 *
 * @param parent A pointer to a virtual directory descriptor; the
 * vfile entry will be created into this directory. If NULL, the /proc
 * root directory will be used. /proc/xenomai is mapped on the
 * globally available @a cobalt_vfroot vdir.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -ENOMEM is returned if the virtual file entry cannot be created
 * in the /proc hierarchy.
 *
 * @coretags{secondary-only}
 */
int xnvfile_init_snapshot(const char *name,
			  struct xnvfile_snapshot *vfile,
			  struct xnvfile_directory *parent)
{
	struct proc_dir_entry *ppde, *pde;
	int mode;

	XENO_BUG_ON(COBALT, vfile->tag == NULL);

	if (vfile->entry.lockops == NULL)
		/* Defaults to nucleus lock */
		vfile->entry.lockops = &xnvfile_nucleus_lock.ops;

	if (parent == NULL)
		parent = &sysroot;

	mode = vfile->ops->store ? 0644 : 0444;
	ppde = parent->entry.pde;
	pde = proc_create_data(name, mode, ppde, &vfile_snapshot_fops, vfile);
	if (pde == NULL)
		return -ENOMEM;

	vfile->entry.pde = pde;

	return 0;
}
EXPORT_SYMBOL_GPL(xnvfile_init_snapshot);

static void *vfile_regular_start(struct seq_file *seq, loff_t *offp)
{
	struct xnvfile_regular_iterator *it = seq->private;
	struct xnvfile_regular *vfile = it->vfile;
	int ret;

	it->pos = *offp;

	if (vfile->entry.lockops) {
		ret = vfile->entry.lockops->get(&vfile->entry);
		if (ret)
			return ERR_PTR(ret);
	}

	/*
	 * If we have no begin() op, then we allow a single call only
	 * to ->show(), by returning the start token once. Otherwise,
	 * we are done.
	 */
	if (vfile->ops->begin == NULL)
		return it->pos > 0 ? NULL : SEQ_START_TOKEN;

	return vfile->ops->begin(it);
}

static void *vfile_regular_next(struct seq_file *seq, void *v, loff_t *offp)
{
	struct xnvfile_regular_iterator *it = seq->private;
	struct xnvfile_regular *vfile = it->vfile;
	void *data;

	if (vfile->ops->next == NULL)
		return NULL;

	it->pos = *offp + 1;

	data = vfile->ops->next(it);
	if (data == NULL)
		return NULL;

	*offp = it->pos;

	return data;
}

static void vfile_regular_stop(struct seq_file *seq, void *v)
{
	struct xnvfile_regular_iterator *it = seq->private;
	struct xnvfile_regular *vfile = it->vfile;

	if (vfile->entry.lockops)
		vfile->entry.lockops->put(&vfile->entry);

	if (vfile->ops->end)
		vfile->ops->end(it);
}

static int vfile_regular_show(struct seq_file *seq, void *v)
{
	struct xnvfile_regular_iterator *it = seq->private;
	struct xnvfile_regular *vfile = it->vfile;
	void *data = v == SEQ_START_TOKEN ? NULL : v;
	int ret;

	ret = vfile->ops->show(it, data);

	return ret == VFILE_SEQ_SKIP ? SEQ_SKIP : ret;
}

static struct seq_operations vfile_regular_ops = {
	.start = vfile_regular_start,
	.next = vfile_regular_next,
	.stop = vfile_regular_stop,
	.show = vfile_regular_show
};

static int vfile_regular_open(struct inode *inode, struct file *file)
{
	struct xnvfile_regular *vfile = PDE_DATA(inode);
	struct xnvfile_regular_ops *ops = vfile->ops;
	struct xnvfile_regular_iterator *it;
	struct seq_file *seq;
	int ret;

	if ((file->f_flags & O_EXCL) != 0 && xnvfile_nref(vfile) > 0)
		return -EBUSY;

	if ((file->f_mode & FMODE_WRITE) != 0 && ops->store == NULL)
		return -EACCES;

	if ((file->f_mode & FMODE_READ) == 0) {
		file->private_data = NULL;
		return 0;
	}

	it = kzalloc(sizeof(*it) + vfile->privsz, GFP_KERNEL);
	if (it == NULL)
		return -ENOMEM;

	it->vfile = vfile;
	it->pos = -1;
	xnvfile_file(vfile) = file;

	if (ops->rewind) {
		ret = ops->rewind(it);
		if (ret) {
		fail:
			kfree(it);
			return ret;
		}
	}

	ret = seq_open(file, &vfile_regular_ops);
	if (ret)
		goto fail;

	seq = file->private_data;
	it->seq = seq;
	seq->private = it;
	xnvfile_nref(vfile)++;

	return 0;
}

static int vfile_regular_release(struct inode *inode, struct file *file)
{
	struct seq_file *seq = file->private_data;
	struct xnvfile_regular_iterator *it;

	if (seq) {
		it = seq->private;
		if (it) {
			--xnvfile_nref(it->vfile);
			XENO_BUG_ON(COBALT, xnvfile_nref(it->vfile) < 0);
			kfree(it);
		}

		return seq_release(inode, file);
	}

	return 0;
}

ssize_t vfile_regular_write(struct file *file, const char __user *buf,
			    size_t size, loff_t *ppos)
{
	struct xnvfile_regular *vfile =
		PDE_DATA(file->f_path.dentry->d_inode);
	struct xnvfile_input input;
	ssize_t ret;

	if (vfile->entry.lockops) {
		ret = vfile->entry.lockops->get(&vfile->entry);
		if (ret)
			return ret;
	}

	input.u_buf = buf;
	input.size = size;
	input.vfile = &vfile->entry;

	ret = vfile->ops->store(&input);

	if (vfile->entry.lockops)
		vfile->entry.lockops->put(&vfile->entry);

	return ret;
}

static struct file_operations vfile_regular_fops = {
	.open = vfile_regular_open,
	.read = seq_read,
	.write = vfile_regular_write,
	.llseek = seq_lseek,
	.release = vfile_regular_release,
};

/**
 * @fn int xnvfile_init_regular(const char *name, struct xnvfile_regular *vfile, struct xnvfile_directory *parent)
 * @brief Initialize a regular vfile.
 *
 * @param name The name which should appear in the pseudo-filesystem,
 * identifying the vfile entry.
 *
 * @param vfile A pointer to a vfile descriptor to initialize
 * from. The following fields in this structure should be filled in
 * prior to call this routine:
 *
 * - .privsz is the size (in bytes) of the private data area to be
 * reserved in the @ref regular_iterator "vfile iterator". A NULL
 * value indicates that no private area should be reserved.
 *
 * - entry.lockops is a pointer to a @ref vfile_lockops "locking
 * descriptor", defining the lock and unlock operations for the
 * vfile. This pointer may be left to NULL, in which case no
 * locking will be applied.
 *
 * - .ops is a pointer to an @ref regular_ops "operation descriptor".
 *
 * @param parent A pointer to a virtual directory descriptor; the
 * vfile entry will be created into this directory. If NULL, the /proc
 * root directory will be used. /proc/xenomai is mapped on the
 * globally available @a cobalt_vfroot vdir.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -ENOMEM is returned if the virtual file entry cannot be created
 * in the /proc hierarchy.
 *
 * @coretags{secondary-only}
 */
int xnvfile_init_regular(const char *name,
			 struct xnvfile_regular *vfile,
			 struct xnvfile_directory *parent)
{
	struct proc_dir_entry *ppde, *pde;
	int mode;

	if (parent == NULL)
		parent = &sysroot;

	mode = vfile->ops->store ? 0644 : 0444;
	ppde = parent->entry.pde;
	pde = proc_create_data(name, mode, ppde, &vfile_regular_fops, vfile);
	if (pde == NULL)
		return -ENOMEM;

	vfile->entry.pde = pde;

	return 0;
}
EXPORT_SYMBOL_GPL(xnvfile_init_regular);

/**
 * @fn int xnvfile_init_dir(const char *name, struct xnvfile_directory *vdir, struct xnvfile_directory *parent)
 * @brief Initialize a virtual directory entry.
 *
 * @param name The name which should appear in the pseudo-filesystem,
 * identifying the vdir entry.
 *
 * @param vdir A pointer to the virtual directory descriptor to
 * initialize.
 *
 * @param parent A pointer to a virtual directory descriptor standing
 * for the parent directory of the new vdir.  If NULL, the /proc root
 * directory will be used. /proc/xenomai is mapped on the globally
 * available @a cobalt_vfroot vdir.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -ENOMEM is returned if the virtual directory entry cannot be
 * created in the /proc hierarchy.
 *
 * @coretags{secondary-only}
 */
int xnvfile_init_dir(const char *name,
		     struct xnvfile_directory *vdir,
		     struct xnvfile_directory *parent)
{
	struct proc_dir_entry *ppde, *pde;

	if (parent == NULL)
		parent = &sysroot;

	ppde = parent->entry.pde;
	pde = proc_mkdir(name, ppde);
	if (pde == NULL)
		return -ENOMEM;

	vdir->entry.pde = pde;
	vdir->entry.lockops = NULL;
	vdir->entry.private = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(xnvfile_init_dir);

/**
 * @fn int xnvfile_init_link(const char *from, const char *to, struct xnvfile_link *vlink, struct xnvfile_directory *parent)
 * @brief Initialize a virtual link entry.
 *
 * @param from The name which should appear in the pseudo-filesystem,
 * identifying the vlink entry.
 *
 * @param to The target file name which should be referred to
 * symbolically by @a name.
 *
 * @param vlink A pointer to the virtual link descriptor to
 * initialize.
 *
 * @param parent A pointer to a virtual directory descriptor standing
 * for the parent directory of the new vlink. If NULL, the /proc root
 * directory will be used. /proc/xenomai is mapped on the globally
 * available @a cobalt_vfroot vdir.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -ENOMEM is returned if the virtual link entry cannot be created
 * in the /proc hierarchy.
 *
 * @coretags{secondary-only}
 */
int xnvfile_init_link(const char *from,
		      const char *to,
		      struct xnvfile_link *vlink,
		      struct xnvfile_directory *parent)
{
	struct proc_dir_entry *ppde, *pde;

	if (parent == NULL)
		parent = &sysroot;

	ppde = parent->entry.pde;
	pde = proc_symlink(from, ppde, to);
	if (pde == NULL)
		return -ENOMEM;

	vlink->entry.pde = pde;
	vlink->entry.lockops = NULL;
	vlink->entry.private = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(xnvfile_init_link);

/**
 * @fn void xnvfile_destroy(struct xnvfile *vfile)
 * @brief Removes a virtual file entry.
 *
 * @param vfile A pointer to the virtual file descriptor to
 * remove.
 *
 * @coretags{secondary-only}
 */
void xnvfile_destroy(struct xnvfile *vfile)
{
	proc_remove(vfile->pde);
}
EXPORT_SYMBOL_GPL(xnvfile_destroy);

/**
 * @fn ssize_t xnvfile_get_blob(struct xnvfile_input *input, void *data, size_t size)
 * @brief Read in a data bulk written to the vfile.
 *
 * When writing to a vfile, the associated store() handler from the
 * @ref snapshot_store "snapshot-driven vfile" or @ref regular_store
 * "regular vfile" is called, with a single argument describing the
 * input data. xnvfile_get_blob() retrieves this data as an untyped
 * binary blob, and copies it back to the caller's buffer.
 *
 * @param input A pointer to the input descriptor passed to the
 * store() handler.
 *
 * @param data The address of the destination buffer to copy the input
 * data to.
 *
 * @param size The maximum number of bytes to copy to the destination
 * buffer. If @a size is larger than the actual data size, the input
 * is truncated to @a size.
 *
 * @return The number of bytes read and copied to the destination
 * buffer upon success. Otherwise, a negative error code is returned:
 *
 * - -EFAULT indicates an invalid source buffer address.
 *
 * @coretags{secondary-only}
 */
ssize_t xnvfile_get_blob(struct xnvfile_input *input,
			 void *data, size_t size)
{
	ssize_t nbytes = input->size;

	if (nbytes > size)
		nbytes = size;

	if (nbytes > 0 && copy_from_user(data, input->u_buf, nbytes))
		return -EFAULT;

	return nbytes;
}
EXPORT_SYMBOL_GPL(xnvfile_get_blob);

/**
 * @fn ssize_t xnvfile_get_string(struct xnvfile_input *input, char *s, size_t maxlen)
 * @brief Read in a C-string written to the vfile.
 *
 * When writing to a vfile, the associated store() handler from the
 * @ref snapshot_store "snapshot-driven vfile" or @ref regular_store
 * "regular vfile" is called, with a single argument describing the
 * input data. xnvfile_get_string() retrieves this data as a
 * null-terminated character string, and copies it back to the
 * caller's buffer.
 *
 * @param input A pointer to the input descriptor passed to the
 * store() handler.
 *
 * @param s The address of the destination string buffer to copy the
 * input data to.
 *
 * @param maxlen The maximum number of bytes to copy to the
 * destination buffer, including the ending null character. If @a
 * maxlen is larger than the actual string length, the input is
 * truncated to @a maxlen.
 *
 * @return The number of characters read upon success. Otherwise, a
 * negative error code is returned:
 *
 * - -EFAULT indicates an invalid source buffer address.
 *
 * @coretags{secondary-only}
 */
ssize_t xnvfile_get_string(struct xnvfile_input *input,
			   char *s, size_t maxlen)
{
	ssize_t nbytes, eol;

	if (maxlen < 1)
		return -EINVAL;

	nbytes = xnvfile_get_blob(input, s, maxlen - 1);
	if (nbytes < 0)
		return nbytes;

	eol = nbytes;
	if (eol > 0 && s[eol - 1] == '\n')
		eol--;

	s[eol] = '\0';

	return nbytes;
}
EXPORT_SYMBOL_GPL(xnvfile_get_string);

/**
 * @fn ssize_t xnvfile_get_integer(struct xnvfile_input *input, long *valp)
 * @brief Evaluate the string written to the vfile as a long integer.
 *
 * When writing to a vfile, the associated store() handler from the
 * @ref snapshot_store "snapshot-driven vfile" or @ref regular_store
 * "regular vfile" is called, with a single argument describing the
 * input data. xnvfile_get_integer() retrieves and interprets this
 * data as a long integer, and copies the resulting value back to @a
 * valp.
 *
 * The long integer can be expressed in decimal, octal or hexadecimal
 * bases depending on the prefix found.
 *
 * @param input A pointer to the input descriptor passed to the
 * store() handler.
 *
 * @param valp The address of a long integer variable to receive the
 * value.
 *
 * @return The number of characters read while evaluating the input as
 * a long integer upon success. Otherwise, a negative error code is
 * returned:
 *
 * - -EINVAL indicates a parse error on the input stream; the written
 * text cannot be evaluated as a long integer.
 *
 * - -EFAULT indicates an invalid source buffer address.
 *
 * @coretags{secondary-only}
 */
ssize_t xnvfile_get_integer(struct xnvfile_input *input, long *valp)
{
	char *end, buf[32];
	ssize_t nbytes;
	long val;

	nbytes = xnvfile_get_blob(input, buf, sizeof(buf) - 1);
	if (nbytes < 0)
		return nbytes;

	if (nbytes == 0)
		return -EINVAL;

	buf[nbytes] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (*end != '\0' && !isspace(*end))
		return -EINVAL;

	*valp = val;

	return nbytes;
}
EXPORT_SYMBOL_GPL(xnvfile_get_integer);

int __vfile_hostlock_get(struct xnvfile *vfile)
{
	struct xnvfile_hostlock_class *lc;

	lc = container_of(vfile->lockops, struct xnvfile_hostlock_class, ops);
	mutex_lock(&lc->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(__vfile_hostlock_get);

void __vfile_hostlock_put(struct xnvfile *vfile)
{
	struct xnvfile_hostlock_class *lc;

	lc = container_of(vfile->lockops, struct xnvfile_hostlock_class, ops);
	mutex_unlock(&lc->mutex);
}
EXPORT_SYMBOL_GPL(__vfile_hostlock_put);

static int __vfile_nklock_get(struct xnvfile *vfile)
{
	struct xnvfile_nklock_class *lc;

	lc = container_of(vfile->lockops, struct xnvfile_nklock_class, ops);
	xnlock_get_irqsave(&nklock, lc->s);

	return 0;
}

static void __vfile_nklock_put(struct xnvfile *vfile)
{
	struct xnvfile_nklock_class *lc;

	lc = container_of(vfile->lockops, struct xnvfile_nklock_class, ops);
	xnlock_put_irqrestore(&nklock, lc->s);
}

struct xnvfile_nklock_class xnvfile_nucleus_lock = {
	.ops = {
		.get = __vfile_nklock_get,
		.put = __vfile_nklock_put,
	},
};

int __init xnvfile_init_root(void)
{
	struct xnvfile_directory *vdir = &cobalt_vfroot;
	struct proc_dir_entry *pde;

	pde = proc_mkdir("xenomai", NULL);
	if (pde == NULL)
		return -ENOMEM;

	vdir->entry.pde = pde;
	vdir->entry.lockops = NULL;
	vdir->entry.private = NULL;

	return 0;
}

void xnvfile_destroy_root(void)
{
	cobalt_vfroot.entry.pde = NULL;
	remove_proc_entry("xenomai", NULL);
}

/** @} */
