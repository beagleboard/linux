
#ifndef _LINUX_RPMSG_VIRTIO_RPMSG_H
#define _LINUX_RPMSG_VIRTIO_RPMSG_H

struct rpmsg_device;
struct virtio_device;

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

