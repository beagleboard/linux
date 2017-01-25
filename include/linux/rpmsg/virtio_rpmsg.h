
#ifndef _LINUX_RPMSG_VIRTIO_RPMSG_H
#define _LINUX_RPMSG_VIRTIO_RPMSG_H

struct rpmsg_device;
struct virtio_device;

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

#if IS_ENABLED(CONFIG_RPMSG_VIRTIO)

struct virtio_device *virtio_rpmsg_get_vdev(struct rpmsg_device *rpdev);

#else

static inline
struct virtio_device *virtio_rpmsg_get_vdev(struct rpmsg_device *rpdev);
{
	return NULL;
}

#endif

#endif

