/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _LINUX_VIRTIO_LIGHT_H
#define _LINUX_VIRTIO_LIGHT_H

/*
 * Virtio device common config offset
 */

/* Magic value ("virt" string) - Read Only */
#define VIRTIO_LIGHT_MAGIC_VALUE	0x000

/* Virtio device version - Read Only */
#define VIRTIO_LIGHT_VERSION		0x004

/* Virtio device ID - Read Only */
#define VIRTIO_LIGHT_DEVICE_ID		0x008

/* Virtio vendor ID - Read Only */
#define VIRTIO_LIGHT_VENDOR_ID		0x00C

/* Device status register - Read Write */
#define VIRTIO_LIGHT_STATUS		0x010

/* Bitmask of the features supported by the device (host)
 * (32 bits per set) - Read Only
 */

/* Low 32 bits */
#define VIRTIO_LIGHT_DEVICE_FEATURES_LOW	0x014

/* High 32bits */
#define VIRTIO_LIGHT_DEVICE_FEATURES_HIGH	0x018

/* Maximum size of the currently selected queue - Read Only */
#define VIRTIO_LIGHT_QUEUE_SIZE_MAX	0x01C

/* Queue size for the currently selected queue - Write Only */
#define VIRTIO_LIGHT_QUEUE_SIZE		0x020

/* Used Ring alignment for the currently selected queue - Write Only */
#define VIRTIO_LIGHT_QUEUE_ALIGN	0x024

/* Numbers of virt queue */
#define VIRTIO_LIGHT_QUEUE_NUM		0x028

/* Guest's PFN for the queue with number index - Read Write
 * Please notes, if some virtual device support multiple vqs,
 * it will occupy more config space size(vq_num*4). So we
 * should reserve more space starting from this offset.
 * eg, Offset between 0x2C~0x100 is for QUEUE_PFN config space.
 */
#define VIRTIO_LIGHT_QUEUE_PFN		0x2C


/* The config space is defined by each driver as
 * the per-driver configuration space - Read Write */
#define VIRTIO_LIGHT_CONFIG		0x100

#define VIRTIO_LIGHT_CONFIG_LEN		0x1000

#endif
