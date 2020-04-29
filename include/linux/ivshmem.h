/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_IVSHMEM_H
#define _LINUX_IVSHMEM_H

#include <linux/types.h>

#define IVSHM_PROTO_UNDEFINED		0x0000
#define IVSHM_PROTO_NET			0x0001
#define IVSHM_PROTO_VIRTIO_FRONT	0x8000
#define IVSHM_PROTO_VIRTIO_BACK		0xc000
#define IVSHM_PROTO_VIRTIO_DEVID_MASK	0x7fff

#define IVSHM_CFG_PRIV_CNTL		0x03
# define IVSHM_PRIV_CNTL_ONESHOT_INT	BIT(0)
#define IVSHM_CFG_STATE_TAB_SZ		0x04
#define IVSHM_CFG_RW_SECTION_SZ		0x08
#define IVSHM_CFG_OUTPUT_SECTION_SZ	0x10
#define IVSHM_CFG_ADDRESS		0x18

struct ivshm_regs {
	u32 id;
	u32 max_peers;
	u32 int_control;
	u32 doorbell;
	u32 state;
};

#define IVSHM_INT_ENABLE		BIT(0)

#endif /* _LINUX_IVSHMEM_H */
