/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/tinydrm/tinydrm.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/**
 * DOC: Performance reporting
 *
 * tinydrm can provide performance reporting when built with CONFIG_DEBUG_FS.
 * This is available through the file ``dirty``.
 * Writing a positive number <n> to this file (re)starts the process of
 * collecting statistics for the last <n> framebuffer flushes.
 * Writing a zero stops it.
 * Reading this file will provide a list of the last <n> flushes.
 * Reading will not clear the list.
 *
 * Example use::
 *     # cd /sys/kernel/debug/dri/0
 *     # echo 4 > dirty
 *     # cat dirty
 *     [ 2140.061740] 2798 KiB/s, 151 KiB in 54 ms,    full(320x240+0+0)
 *     [ 2140.161710] 2798 KiB/s, 151 KiB in 54 ms,    full(320x240+0+0),  99 ms since last, 10 fps
 *     [ 2140.301724] 2798 KiB/s, 151 KiB in 54 ms,    full(320x240+0+0), 140 ms since last,  7 fps
 *     [ 2140.361702] 3552 KiB/s,  10 KiB in  3 ms, partial(320x16+0+224),  59 ms since last
 *
 * To get this functionality the driver needs to use tinydrm_debugfs_init() and
 * tinydrm_debugfs_cleanup() in their &drm_driver. Additionally it has to call
 * tinydrm_debugfs_dirty_init() to set it up and then bracket the framebuffer
 * flushes with calls to tinydrm_debugfs_dirty_begin() and
 * tinydrm_debugfs_dirty_end().
 */

#define MAX_DIRTY_ENTRIES 128

struct tinydrm_dirty_entry {
	struct list_head list;
	struct drm_clip_rect clip;
	bool full;
	size_t len;
	u64 start;
	u64 end;
};

struct tinydrm_debugfs_dirty {
	struct list_head list;
	struct mutex list_lock;
};

/**
 * tinydrm_debugfs_dirty_init - Initialize performance reporting
 * @tdev: tinydrm device
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int tinydrm_debugfs_dirty_init(struct tinydrm_device *tdev)
{
	struct tinydrm_debugfs_dirty *dirty;

	dirty = devm_kzalloc(tdev->drm.dev, sizeof(*dirty), GFP_KERNEL);
	if (!dirty)
		return -ENOMEM;

	mutex_init(&dirty->list_lock);
	INIT_LIST_HEAD(&dirty->list);
	tdev->debugfs_dirty = dirty;

	return 0;
}
EXPORT_SYMBOL(tinydrm_debugfs_dirty_init);

static struct tinydrm_dirty_entry *
tinydrm_debugfs_dirty_get_entry(struct tinydrm_debugfs_dirty *dirty)
{
	struct tinydrm_dirty_entry *entry;

	entry = list_last_entry(&dirty->list,
				struct tinydrm_dirty_entry, list);
	if (entry->start) {
		if (entry->end)
			return NULL; /* buffer is full */
		else
			return entry; /* in progress */
	}
	/* The buffer hasn't been filled yet */
	list_for_each_entry(entry, &dirty->list, list) {
		if (!entry->end)
			return entry;
	}

	WARN_ON(1);
	return NULL;
}

/**
 * tinydrm_debugfs_dirty_begin - Display update is starting
 * @tdev: tinydrm device
 * @fb: framebuffer
 * @clip: The part of the display that is to be updated.
 */
void tinydrm_debugfs_dirty_begin(struct tinydrm_device *tdev,
				 struct drm_framebuffer *fb,
				 const struct drm_clip_rect *clip)
{
	struct tinydrm_debugfs_dirty *dirty = tdev->debugfs_dirty;
	struct tinydrm_dirty_entry *entry;

	if (!dirty)
		return;

	mutex_lock(&dirty->list_lock);

	if (list_empty(&dirty->list))
		goto out_unlock;

	entry = tinydrm_debugfs_dirty_get_entry(dirty);
	if (!entry) {
		list_rotate_left(&dirty->list);
		entry = list_last_entry(&dirty->list,
					struct tinydrm_dirty_entry, list);
	}

	entry->clip = *clip;
	entry->full = clip->x1 == 0 && clip->x2 == fb->width &&
		      clip->y1 == 0 && clip->y2 == fb->height;
	entry->start = local_clock();
	entry->end = 0;

out_unlock:
	mutex_unlock(&dirty->list_lock);
}
EXPORT_SYMBOL(tinydrm_debugfs_dirty_begin);

/**
 * tinydrm_debugfs_dirty_end - Display update has ended
 * @tdev: tinydrm device
 * @len: Length of transfer buffer
 * @bits_per_pixel: Used to calculate transfer length if @len is zero by
 *                  multiplying with number of pixels in clip.
 */
void tinydrm_debugfs_dirty_end(struct tinydrm_device *tdev, size_t len,
			       unsigned int bits_per_pixel)
{
	struct tinydrm_debugfs_dirty *dirty = tdev->debugfs_dirty;
	struct tinydrm_dirty_entry *entry;

	if (!dirty)
		return;

	mutex_lock(&dirty->list_lock);

	if (list_empty(&dirty->list))
		goto out_unlock;

	entry = tinydrm_debugfs_dirty_get_entry(dirty);
	if (WARN_ON(!entry))
		goto out_unlock;

	if (!entry->start)
		goto out_unlock; /* enabled during an update */

	if (!len)
		len = (entry->clip.x2 - entry->clip.x1) *
		      (entry->clip.y2 - entry->clip.y1) *
		      bits_per_pixel / 8;
	entry->end = local_clock();
	entry->len = len;

out_unlock:
	mutex_unlock(&dirty->list_lock);
}
EXPORT_SYMBOL(tinydrm_debugfs_dirty_end);

static int tinydrm_debugfs_dirty_seq_show(struct seq_file *s, void *v)
{
	struct tinydrm_debugfs_dirty *dirty = s->private;
	struct tinydrm_dirty_entry *entry;
	u64 previous_start = 0;
	bool previous_full = false;

	if (!dirty) {
		seq_puts(s, "Performance reporting is not supported by this driver.\n");
		return 0;
	}

	mutex_lock(&dirty->list_lock);

	if (list_empty(&dirty->list)) {
		seq_puts(s, "Performance reporting is disabled.\n");
		seq_printf(s, "Enable by writing the number of wanted entries to this file (<%i)\n",
			   MAX_DIRTY_ENTRIES + 1);
		goto out_unlock;
	}

	list_for_each_entry(entry, &dirty->list, list) {
		u32 start_rem_nsec, duration_ms, last_ms = 0;
		u64 start_sec, throughput;

		/* stop on empty entry (buffer not full nor empty) */
		if (!entry->start)
			break;

		start_sec = div_u64_rem(entry->start, 1000000000,
					&start_rem_nsec);
		seq_printf(s, "[%5llu.%06u]", start_sec,
			   start_rem_nsec / 1000);

		if (!entry->end) {
			seq_puts(s, " update in progress\n");
			break;
		}

		if (entry->end <= entry->start) {
			seq_puts(s, " illegal entry\n");
			continue;
		}

		duration_ms = div_u64(entry->end - entry->start, 1000000);
		if (!duration_ms)
			duration_ms = 1;

		throughput = entry->len * 1000 / duration_ms / SZ_1K;
		seq_printf(s, " %5llu KiB/s", throughput);
		if (entry->len < SZ_4K)
			seq_printf(s, ", %4u bytes", entry->len);
		else
			seq_printf(s, ", %6u KiB", entry->len / SZ_1K);

		seq_printf(s, " in %3u ms", duration_ms);

		seq_printf(s, ", %s(%ux%u+%u+%u)",
			   entry->full ? "   full" : "partial",
			   entry->clip.x2 - entry->clip.x1,
			   entry->clip.y2 - entry->clip.y1,
			   entry->clip.x1, entry->clip.y1);

		if (previous_start) {
			last_ms = div_u64(entry->start - previous_start,
					  1000000);
			seq_printf(s, ", %3u ms since last", last_ms);
		}

		if (entry->full && previous_full && last_ms)
			seq_printf(s, ", %2u fps", 1000 / last_ms);

		seq_puts(s, "\n");
		previous_start = entry->start;
		previous_full = entry->full;
	}

out_unlock:
	mutex_unlock(&dirty->list_lock);

	return 0;
}

static int tinydrm_debugfs_dirty_open(struct inode *inode, struct file *file)
{
	struct drm_info_node *node = inode->i_private;
	struct drm_device *drm = node->minor->dev;
	struct tinydrm_device *tdev = drm_to_tinydrm(drm);

	return single_open(file, tinydrm_debugfs_dirty_seq_show,
			   tdev->debugfs_dirty);
}

static void
tinydrm_debugfs_dirty_list_delete(struct tinydrm_debugfs_dirty *dirty)
{
	struct tinydrm_dirty_entry *entry, *tmp;

	list_for_each_entry_safe(entry, tmp, &dirty->list, list) {
		list_del(&entry->list);
		kfree(entry);
	}
}

static ssize_t tinydrm_debugfs_dirty_write(struct file *file,
					   const char __user *buf,
					   size_t len, loff_t *ppos)
{
	struct tinydrm_debugfs_dirty *dirty;
	struct tinydrm_dirty_entry *entry;
	unsigned long long val;
	char set_buf[24];
	ssize_t ret = 0;
	size_t size;
	int i;

	dirty = ((struct seq_file *)file->private_data)->private;
	if (!dirty)
		return -ENODEV;

	size = min(sizeof(set_buf) - 1, len);
	if (copy_from_user(set_buf, buf, size))
		return -EFAULT;

	set_buf[size] = '\0';
	ret = kstrtoull(set_buf, 0, &val);
	if (ret)
		return ret;

	if (val > MAX_DIRTY_ENTRIES)
		return -ERANGE;

	mutex_lock(&dirty->list_lock);

	if (!list_empty(&dirty->list))
		tinydrm_debugfs_dirty_list_delete(dirty);

	for (i = 0; i < val; i++) {
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry) {
			tinydrm_debugfs_dirty_list_delete(dirty);
			ret = -ENOMEM;
			break;
		}
		list_add(&entry->list, &dirty->list);
	}

	mutex_unlock(&dirty->list_lock);

	return ret < 0 ? ret : len;
}

static const struct file_operations tinydrm_debugfs_dirty_file_ops = {
	.owner   = THIS_MODULE,
	.open    = tinydrm_debugfs_dirty_open,
	.read    = seq_read,
	.write   = tinydrm_debugfs_dirty_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

/*
 * TODO
 * Maybe drm_debugfs_cleanup() can use debugfs_remove_recursive() instead of
 * debugfs_remove(minor->debugfs_root). Then this hack wouldn't be needed.
 * armada, i915, nouveau and sti do similar things.
 */
static int tinydrm_debugfs_create_file(const char *name, umode_t mode,
				       struct dentry *root,
				       struct drm_minor *minor,
				       const struct file_operations *fops)
{
	struct drm_info_node *node;
	struct dentry *ent;

	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	ent = debugfs_create_file(name, mode, root, node, fops);
	if (!ent) {
		DRM_ERROR("Cannot create /sys/kernel/debug/dri/%s/%s\n",
			  root->d_name.name, name);
		kfree(node);
		return -ENOMEM;
	}

	node->minor = minor;
	node->dent = ent;
	node->info_ent = (const void *)fops;

	mutex_lock(&minor->debugfs_lock);
	list_add(&node->list, &minor->debugfs_list);
	mutex_unlock(&minor->debugfs_lock);

	return 0;
}

static void tinydrm_debugfs_remove_file(struct drm_minor *minor,
					const struct file_operations *fops)
{
	drm_debugfs_remove_files((struct drm_info_list *)fops, 1, minor);
}

static const struct drm_info_list tinydrm_debugfs_list[] = {
	{ "fb",   drm_fb_cma_debugfs_show, 0 },
};

/**
 * tinydrm_debugfs_init - Create debugfs entries
 * @minor: DRM minor
 *
 * Drivers can use this as their &drm_driver->debugfs_init callback.
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int tinydrm_debugfs_init(struct drm_minor *minor)
{
	int ret;

	ret = drm_debugfs_create_files(tinydrm_debugfs_list,
				       ARRAY_SIZE(tinydrm_debugfs_list),
				       minor->debugfs_root, minor);
	if (ret)
		return ret;

	ret = tinydrm_debugfs_create_file("dirty", S_IRUGO | S_IWUSR,
					  minor->debugfs_root, minor,
					  &tinydrm_debugfs_dirty_file_ops);

	return ret;
}
EXPORT_SYMBOL(tinydrm_debugfs_init);

/**
 * tinydrm_debugfs_cleanup - Cleanup debugfs entries
 * @minor: DRM minor
 *
 * Drivers can use this as their &drm_driver->debugfs_cleanup callback.
 */
void tinydrm_debugfs_cleanup(struct drm_minor *minor)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(minor->dev);

	drm_debugfs_remove_files(tinydrm_debugfs_list,
				 ARRAY_SIZE(tinydrm_debugfs_list), minor);
	tinydrm_debugfs_remove_file(minor, &tinydrm_debugfs_dirty_file_ops);
	if (tdev->debugfs_dirty) {
		tinydrm_debugfs_dirty_list_delete(tdev->debugfs_dirty);
		tdev->debugfs_dirty = NULL;
	}
}
EXPORT_SYMBOL(tinydrm_debugfs_cleanup);
