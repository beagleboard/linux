/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from RTnet project file stack/include/rtdev.h:
 *
 * Copyright (C) 1999       Lineo, Inc
 *               1999, 2002 David A. Schleef <ds@schleef.org>
 *               2003-2005  Jan Kiszka <jan.kiszka@web.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RTCAN_DEV_H_
#define __RTCAN_DEV_H_


#ifdef __KERNEL__

#include <asm/atomic.h>
#include <linux/netdevice.h>
#include <linux/semaphore.h>

#include "rtcan_list.h"


/* Number of MSCAN devices the driver can handle */
#define RTCAN_MAX_DEVICES    CONFIG_XENO_DRIVERS_CAN_MAX_DEVICES

/* Maximum number of single filters per controller which can be registered
 * for reception at the same time using Bind */
#define RTCAN_MAX_RECEIVERS  CONFIG_XENO_DRIVERS_CAN_MAX_RECEIVERS

/* Suppress handling of refcount if module support is not enabled
 * or modules cannot be unloaded */

#if defined(CONFIG_MODULES) && defined(CONFIG_MODULE_UNLOAD)
#define RTCAN_USE_REFCOUNT
#endif

/*
 * CAN harware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const {
	char name[16];		/* Name of the CAN controller hardware */
	__u32 tseg1_min;	/* Time segement 1 = prop_seg + phase_seg1 */
	__u32 tseg1_max;
	__u32 tseg2_min;	/* Time segement 2 = phase_seg2 */
	__u32 tseg2_max;
	__u32 sjw_max;		/* Synchronisation jump width */
	__u32 brp_min;		/* Bit-rate prescaler */
	__u32 brp_max;
	__u32 brp_inc;
};

struct rtcan_device {
    unsigned int        version;

    char                name[IFNAMSIZ];

    char                *ctrl_name; /* Name of CAN controller */
    char                *board_name;/* Name of CAN board */

    unsigned long       base_addr;  /* device I/O address   */
    rtdm_irq_t          irq_handle; /* RTDM IRQ handle */

    int                 ifindex;
#ifdef RTCAN_USE_REFCOUNT
    atomic_t            refcount;
#endif

    void                *priv;      /* pointer to chip private data */

    void                *board_priv;/* pointer to board private data*/

    struct semaphore    nrt_lock;   /* non-real-time locking        */

    /* Spinlock for all devices (but not for all attributes) and also for HW
     * access to all CAN controllers
     */
    rtdm_lock_t         device_lock;

    /* Acts as a mutex allowing only one sender to write to the MSCAN
     * simultaneously. Created when the controller goes into operating mode,
     * destroyed if it goes into reset mode. */
    rtdm_sem_t          tx_sem;

    /* Baudrate of this device. Protected by device_lock in all device
     * structures. */
    unsigned int        can_sys_clock;


    /* Baudrate of this device. Protected by device_lock in all device
     * structures. */
    can_baudrate_t      baudrate;

    struct can_bittime  bit_time;
    const struct can_bittiming_const *bittiming_const;

    /* State which the controller is in. Protected by device_lock in all
     * device structures. */
    can_state_t state;

    /* State which the controller was before sleeping. Protected by
     * device_lock in all device structures. */
    can_state_t          state_before_sleep;

    /* Controller specific settings. Protected by device_lock in all
     * device structures. */
    can_ctrlmode_t       ctrl_mode;

    /* Device operations */
    int                 (*hard_start_xmit)(struct rtcan_device *dev,
					   struct can_frame *frame);
    int                 (*do_set_mode)(struct rtcan_device *dev,
				       can_mode_t mode,
				       rtdm_lockctx_t *lock_ctx);
    can_state_t         (*do_get_state)(struct rtcan_device *dev);
    int                 (*do_set_bit_time)(struct rtcan_device *dev,
					   struct can_bittime *bit_time,
					   rtdm_lockctx_t *lock_ctx);
#ifdef CONFIG_XENO_DRIVERS_CAN_BUS_ERR
    void                (*do_enable_bus_err)(struct rtcan_device *dev);
#endif

    /* Reception list head. This list contains all filters which have been
     * registered via a bind call. */
    struct rtcan_recv               *recv_list;

    /* Empty list head. This list contains all empty entries not needed
     * by the reception list and therefore is disjunctive with it. */
    struct rtcan_recv               *empty_list;

    /* Preallocated array for the list entries. To increase cache
     * locality all list elements are kept in this array. */
    struct rtcan_recv               receivers[RTCAN_MAX_RECEIVERS];

    /* Indicates the length of the empty list */
    int                             free_entries;

    /* A few statistics counters */
    unsigned int tx_count;
    unsigned int rx_count;
    unsigned int err_count;

#ifdef CONFIG_PROC_FS
    struct proc_dir_entry *proc_root;
#endif
#ifdef CONFIG_XENO_DRIVERS_CAN_LOOPBACK
    struct rtcan_skb tx_skb;
    struct rtcan_socket *tx_socket;
#endif /* CONFIG_XENO_DRIVERS_CAN_LOOPBACK */
};


extern struct semaphore rtcan_devices_nrt_lock;


void rtcan_dev_free(struct rtcan_device *dev);

int rtcan_dev_register(struct rtcan_device *dev);
int rtcan_dev_unregister(struct rtcan_device *dev);

struct rtcan_device *rtcan_dev_alloc(int sizeof_priv, int sizeof_board_priv);
void rtcan_dev_alloc_name (struct rtcan_device *dev, const char *name_mask);

struct rtcan_device *rtcan_dev_get_by_name(const char *if_name);
struct rtcan_device *rtcan_dev_get_by_index(int ifindex);

#ifdef RTCAN_USE_REFCOUNT
#define rtcan_dev_reference(dev)      atomic_inc(&(dev)->refcount)
#define rtcan_dev_dereference(dev)    atomic_dec(&(dev)->refcount)
#else
#define rtcan_dev_reference(dev)      do {} while(0)
#define rtcan_dev_dereference(dev)    do {} while(0)
#endif

#ifdef CONFIG_PROC_FS
int rtcan_dev_create_proc(struct rtcan_device* dev);
void rtcan_dev_remove_proc(struct rtcan_device* dev);
#else /* !CONFIG_PROC_FS */
static inline int rtcan_dev_create_proc(struct rtcan_device* dev)
{
	return 0;
}
static inline void rtcan_dev_remove_proc(struct rtcan_device* dev) { }
#endif /* !CONFIG_PROC_FS */

#endif  /* __KERNEL__ */

#endif  /* __RTCAN_DEV_H_ */
