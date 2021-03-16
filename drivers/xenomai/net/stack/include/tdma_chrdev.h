/***
 *
 *  include/tdma_chrdev.h
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __TDMA_CHRDEV_H_
#define __TDMA_CHRDEV_H_

#ifndef __KERNEL__
# include <inttypes.h>
#endif

#include <rtnet_chrdev.h>


#define MIN_SLOT_SIZE       60


struct tdma_config {
    struct rtnet_ioctl_head head;

    union {
        struct {
            __u64       cycle_period;
            __u64       backup_sync_offset;
            __u32       cal_rounds;
            __u32       max_cal_requests;
            __u32       max_slot_id;
        } master;

        struct {
            __u32       cal_rounds;
            __u32       max_slot_id;
        } slave;

        struct {
            __s32       id;
            __u32       period;
            __u64       offset;
            __u32       phasing;
            __u32       size;
            __s32       joint_slot;
            __u32       cal_timeout;
            __u64       *cal_results;
        } set_slot;

        struct {
            __s32       id;
        } remove_slot;

        __u64 __padding[8];
    } args;
};


#define TDMA_IOC_MASTER                 _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 0, \
                                             struct tdma_config)
#define TDMA_IOC_SLAVE                  _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 1, \
                                             struct tdma_config)
#define TDMA_IOC_CAL_RESULT_SIZE        _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 2, \
                                             struct tdma_config)
#define TDMA_IOC_SET_SLOT               _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 3, \
                                             struct tdma_config)
#define TDMA_IOC_REMOVE_SLOT            _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 4, \
                                             struct tdma_config)
#define TDMA_IOC_DETACH                 _IOW(RTNET_IOC_TYPE_RTMAC_TDMA, 5, \
                                             struct tdma_config)

#endif /* __TDMA_CHRDEV_H_ */
