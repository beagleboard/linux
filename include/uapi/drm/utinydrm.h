/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _UAPI__UTINYDRM_H_
#define _UAPI__UTINYDRM_H_

#if defined(__KERNEL__)
#include <uapi/drm/drm_mode.h>
#include <linux/types.h>
#else
#include <linux/types.h>
#include <drm/drm_mode.h>
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#define UTINYDRM_MAX_NAME_SIZE    80

/* ioctl */
/* FIXME: what should this be? */
#define UTINYDRM_IOCTL_BASE       'U'

struct utinydrm_dev_create {
	char name[UTINYDRM_MAX_NAME_SIZE];
	struct drm_mode_modeinfo mode;
	int index;
};

#define UTINYDRM_DEV_CREATE       _IOWR(UTINYDRM_IOCTL_BASE, 1, struct utinydrm_dev_create)

struct utinydrm_event {
	__u32 type;
	__u32 length;
};

#define UTINYDRM_EVENT_PIPE_ENABLE	1
#define UTINYDRM_EVENT_PIPE_DISABLE	2
#define UTINYDRM_EVENT_FB_CREATE	3

struct utinydrm_event_fb_create {
	struct utinydrm_event base;
	struct drm_mode_fb_cmd2 fb;
};

#define UTINYDRM_EVENT_FB_DESTROY	4

struct utinydrm_event_fb_destroy {
	struct utinydrm_event base;
	__u32 fb_id;
};

#define UTINYDRM_EVENT_FB_DIRTY 	5

struct utinydrm_event_fb_dirty {
	struct utinydrm_event base;
	struct drm_mode_fb_dirty_cmd fb_dirty_cmd;
	struct drm_clip_rect clips[];
};

#define UTINYDRM_PRIME_HANDLE_TO_FD 0x01
#define DRM_IOCTL_UTINYDRM_PRIME_HANDLE_TO_FD    DRM_IOWR(DRM_COMMAND_BASE + UTINYDRM_PRIME_HANDLE_TO_FD, struct drm_prime_handle)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI__UTINYDRM_H_ */
