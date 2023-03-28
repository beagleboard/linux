/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_REMOTEDEV_H__
#define __RPMSG_REMOTEDEV_H__

struct rpmsg_remotedev;

/* Defines for demo device */
#define REMOTEDEV_DEMO_MSG_LEN		(128)

/* Defines for display device */
#define RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES		(3)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_DISPS		(8)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES		(8)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_FORMATS		(32)
#define RPMSG_REMOTEDEV_DISPLAY_MAX_ZORDERS		(8)

/* Struct definitions for display device */
struct rpmsg_remotedev_display_buffer {
	u32 width;
	u32 height;
	u32 format;
	u32 num_planes;
	dma_addr_t planes[RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES];
	u32 pitches[RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES];
	struct rpmsg_remotedev *rdev;
	void *priv;
};

struct rpmsg_remotedev_display_pipe_update {
	u32 pipe_id;
	bool enabled;
	u32 dst_w;
	u32 dst_h;
	u32 dst_x;
	u32 dst_y;
	struct rpmsg_remotedev_display_buffer *buffer;
};

struct rpmsg_remotedev_display_commit {
	u32 disp_id;
	u32 num_pipe_updates;
	struct rpmsg_remotedev_display_pipe_update pipes[RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES];
	struct rpmsg_remotedev *rdev;
	void *priv;
};

struct rpmsg_remotedev_display_pipe {
	u32 pipe_id;
	bool can_scale;
	bool can_mod_win;
	u32 fixed_win_x;
	u32 fixed_win_y;
	u32 fixed_win_w;
	u32 fixed_win_h;
	u32 initial_zorder;
	u32 num_formats;
	u32 formats[RPMSG_REMOTEDEV_DISPLAY_MAX_FORMATS];
	u32 num_allowed_zorders;
	u32 allowed_zorders[RPMSG_REMOTEDEV_DISPLAY_MAX_ZORDERS];
};

struct rpmsg_remotedev_display_disp {
	u32 disp_id;
	u32 width;
	u32 height;
	u32 refresh;
	u32 num_pipes;
	struct rpmsg_remotedev_display_pipe pipes[RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES];
};

struct rpmsg_remotedev_display_resinfo {
	u32 num_disps;
	struct rpmsg_remotedev_display_disp disps[RPMSG_REMOTEDEV_DISPLAY_MAX_DISPS];
};

/* callbacks for demo device */
struct rpmsg_remotedev_demo_cb {
	void (*s2c_message)(void *data, ssize_t len, void *cb_data);
};

/* callbacks for display device */
struct rpmsg_remotedev_display_cb {
	void (*commit_done)(struct rpmsg_remotedev_display_commit *commit, void *cb_data);
	void (*buffer_done)(struct rpmsg_remotedev_display_buffer *buffer, void *cb_data);
};

/* requests for demo device */
struct rpmsg_remotedev_demo_ops {
	int (*get_data)(struct rpmsg_remotedev *rdev, void *data, ssize_t len);
	int (*ping)(struct rpmsg_remotedev *rdev, void *ping_data, ssize_t ping_len);
	int (*c2s_message)(struct rpmsg_remotedev *rdev, void *c2s_msg_data, ssize_t len);
};

/* requests for display device */
struct rpmsg_remotedev_display_ops {
	bool (*ready)(struct rpmsg_remotedev *rdev);
	int (*get_res_info)(struct rpmsg_remotedev *rdev,
			    struct rpmsg_remotedev_display_resinfo *res);
	int (*commit)(struct rpmsg_remotedev *rdev, struct rpmsg_remotedev_display_commit *commit);
};

enum rpmsg_remotedev_type {
	RPMSG_REMOTEDEV_DEMO_DEVICE,
	RPMSG_REMOTEDEV_DISPLAY_DEVICE,
};

struct rpmsg_remotedev {
	enum rpmsg_remotedev_type type;
	union {
		struct {
			struct rpmsg_remotedev_demo_ops *ops;
			struct rpmsg_remotedev_demo_cb *cb_ops;
		} demo;
		struct {
			struct rpmsg_remotedev_display_ops *ops;
			struct rpmsg_remotedev_display_cb *cb_ops;
		} display;
	} device;
	void *cb_data;
};

#if IS_REACHABLE(CONFIG_RPMSG_KDRV)
struct rpmsg_remotedev *rpmsg_remotedev_get_named_device(const char *device_name);
void rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev);
#else
static inline struct rpmsg_remotedev * __maybe_unused rpmsg_remotedev_get_named_device(const char
										       *device_name)
{
	return NULL;
}

static inline void __maybe_unused rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev)
{
}
#endif

#endif
