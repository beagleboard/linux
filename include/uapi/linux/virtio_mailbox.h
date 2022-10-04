/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _LIGHT_MAILBOX_H
#define _LIGHT_MAILBOX_H

#include <linux/virtio_types.h>

/* Feature bits */
#define VIRTIO_MAILBOX_F_CHAN_NUM	1	/* Indicates maximum mailbox channels */
#define VIRTIO_MAILBOX_F_MSG_SIZE	2	/* Indicates msg size for one transport */

/* Default values when feature not set */
#define VIRTIO_MAILBOX_MSG_SIZE_DEFAULT	128
#define VIRTIO_MAILBOX_CHAN_NUM_DEFAULT	8

#define VIRTIO_MAILBOX_CHAN_MAX		32

struct virtio_mailbox_config {
	__virtio32 chan_num;
	__virtio32 msg_size;
	__virtio32 msg_g[VIRTIO_MAILBOX_CHAN_MAX];
	__virtio32 msg_h[VIRTIO_MAILBOX_CHAN_MAX];
	__virtio32 ack_g[VIRTIO_MAILBOX_CHAN_MAX];
	__virtio32 ack_h[VIRTIO_MAILBOX_CHAN_MAX];
} __attribute__((packed));

static const unsigned int offsets[] = {
	offsetof(struct virtio_mailbox_config, chan_num),
	offsetof(struct virtio_mailbox_config, msg_size),
	offsetof(struct virtio_mailbox_config, msg_g),
	offsetof(struct virtio_mailbox_config, msg_h),
	offsetof(struct virtio_mailbox_config, ack_g),
	offsetof(struct virtio_mailbox_config, ack_h),
};

enum offsets_index {
	CHAN_NUM,
	MSG_SIZE,
	MSG_G,
	MSG_H,
	ACK_G,
	ACK_H,
};

#endif
