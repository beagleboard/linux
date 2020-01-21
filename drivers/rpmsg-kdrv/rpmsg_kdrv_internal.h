/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_KDRV_INTERNAL_H__
#define __RPMSG_KDRV_INTERNAL_H__

#define RRMSG_KDRV_CALLBACK_DONE		(0)
#define RRMSG_KDRV_CALLBACK_MORE		(1)

struct rpmsg_kdrv_device {
	struct device dev;
	struct rpmsg_device *rpdev;
	int device_type;
	int device_id;
	void *device_data;
	int device_data_len;
	char *device_name;
	void *device_private;
	void *driver_private;
};

struct rpmsg_kdrv_driver {
	struct device_driver drv;
	int device_type;
	int (*probe)(struct rpmsg_kdrv_device *dev);
	void (*remove)(struct rpmsg_kdrv_device *dev);
	int (*callback)(struct rpmsg_kdrv_device *dev, void *msg, int len);
};

typedef int (*request_cb_t)(void *data, void *req, int req_sz, void *resp, int resp_sz);

extern int rpmsg_kdrv_register_driver(struct rpmsg_kdrv_driver *drv);

extern int rpmsg_kdrv_send_request_with_callback(struct rpmsg_device *rpdev,
		uint32_t device_id, void *message, uint32_t message_size, void *cb_data,
		request_cb_t callback);
extern int rpmsg_kdrv_send_request_with_response(struct rpmsg_device *rpdev,
		uint32_t device_id, void *message, uint32_t message_size,
		void *response, uint32_t response_size);
extern int rpmsg_kdrv_send_message(struct rpmsg_device *rpdev,
		uint32_t device_id, void *message, uint32_t message_size);


#endif
