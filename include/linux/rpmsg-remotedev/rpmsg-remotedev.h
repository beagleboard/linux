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


/* callbacks for demo device */
struct rpmsg_remotedev_demo_cb {
	void (*s2c_message)(void *data, ssize_t len, void *cb_data);
};

/* requests for demo device */
struct rpmsg_remotedev_demo_ops {
	int (*get_data)(struct rpmsg_remotedev *rdev, void *data, ssize_t len);
	int (*ping)(struct rpmsg_remotedev *rdev, void *ping_data, ssize_t ping_len);
	int (*c2s_message)(struct rpmsg_remotedev *rdev, void *c2s_msg_data, ssize_t len);
};

enum rpmsg_remotedev_type {
	RPMSG_REMOTEDEV_DEMO_DEVICE,
};

struct rpmsg_remotedev {
	enum rpmsg_remotedev_type type;
	union {
		struct {
			struct rpmsg_remotedev_demo_ops *ops;
			struct rpmsg_remotedev_demo_cb *cb_ops;
		} demo;
	} device;
	void *cb_data;

};

#if IS_REACHABLE(CONFIG_RPMSG_KDRV)
extern struct rpmsg_remotedev *rpmsg_remotedev_get_named_device(const char *device_name);
extern void rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev);
#else
static inline struct rpmsg_remotedev * __maybe_unused rpmsg_remotedev_get_named_device(const char *device_name)
{
	return NULL;
}

static inline void __maybe_unused rpmsg_remotedev_put_device(struct rpmsg_remotedev *rdev)
{
}
#endif

#endif
