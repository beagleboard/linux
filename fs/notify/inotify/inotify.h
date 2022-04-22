#include <linux/fsnotify_backend.h>
#include <linux/inotify.h>
#include <linux/slab.h> /* struct kmem_cache */

struct inotify_event_info {
	struct fsnotify_event fse;
	int wd;
	u32 sync_cookie;
	int name_len;
	char name[];
};

struct inotify_inode_mark {
	struct fsnotify_mark fsn_mark;
	int wd;
};

static inline struct inotify_event_info *INOTIFY_E(struct fsnotify_event *fse)
{
	return container_of(fse, struct inotify_event_info, fse);
}

/*
 * INOTIFY_USER_FLAGS represents all of the mask bits that we expose to
 * userspace.  There is at least one bit (FS_EVENT_ON_CHILD) which is
 * used only internally to the kernel.
 */
#define INOTIFY_USER_MASK (IN_ALL_EVENTS | IN_ONESHOT | IN_EXCL_UNLINK)

static inline __u32 inotify_mark_user_mask(struct fsnotify_mark *fsn_mark)
{
	return fsn_mark->mask & INOTIFY_USER_MASK;
}

extern void inotify_ignored_and_remove_idr(struct fsnotify_mark *fsn_mark,
					   struct fsnotify_group *group);
extern int inotify_handle_event(struct fsnotify_group *group,
				struct inode *inode,
				struct fsnotify_mark *inode_mark,
				struct fsnotify_mark *vfsmount_mark,
				u32 mask, void *data, int data_type,
				const unsigned char *file_name, u32 cookie);

extern const struct fsnotify_ops inotify_fsnotify_ops;
