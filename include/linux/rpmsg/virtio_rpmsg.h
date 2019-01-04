/* SPDX-License-Identifier: BSD-3-Clause */

#ifndef _LINUX_RPMSG_VIRTIO_RPMSG_H
#define _LINUX_RPMSG_VIRTIO_RPMSG_H

/**
 * struct rpmsg_hdr - common header for all virtio rpmsg messages
 * @src: source address
 * @dst: destination address
 * @reserved: reserved for future use
 * @len: length of payload (in bytes)
 * @flags: message flags
 * @data: @len bytes of message payload data
 *
 * Every message sent(/received) on the rpmsg bus begins with this header.
 */
struct rpmsg_hdr {
	u32 src;
	u32 dst;
	u32 reserved;
	u16 len;
	u16 flags;
	u8 data[0];
} __packed;

#endif
