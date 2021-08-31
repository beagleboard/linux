/*
 * Copyright (C) 2006 Wolfgang Grandegger, <wg@grandegger.com>
 * Copyright (C) 2005 Marc Kleine-Budde, Pengutronix
 * Copyright (C) 2006 Andrey Volkov, Varma Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <rtdm/driver.h>

#include <rtdm/can.h>
#include "rtcan_dev.h"
#include "rtcan_raw.h"
#include "rtcan_internal.h"

#ifdef CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD

#define RTCAN_MAX_TSEG1  15
#define RTCAN_MAX_TSEG2  7

/*
 * Calculate standard bit-time values for odd bitrates.
 * Most parts of this code is from Arnaud Westenberg <arnaud@wanadoo.nl>
 */
static int rtcan_calc_bit_time(struct rtcan_device *dev,
			       can_baudrate_t rate,
			       struct can_bittime_std *bit_time)
{
    int best_error = 1000000000;
    int error;
    int best_tseg=0, best_brp=0, best_rate=0, brp=0;
    int tseg=0, tseg1=0, tseg2=0;
    int clock = dev->can_sys_clock;
    int sjw = 0;
    int sampl_pt = 90;

    /* some heuristic specials */
    if (rate > ((1000000 + 500000) / 2))
	sampl_pt = 75;

    if (rate < ((12500 + 10000) / 2))
	sampl_pt = 75;

    if (rate < ((100000 + 125000) / 2))
	sjw = 1;

    /* tseg even = round down, odd = round up */
    for (tseg = (0 + 0 + 2) * 2;
	 tseg <= (RTCAN_MAX_TSEG2 + RTCAN_MAX_TSEG1 + 2) * 2 + 1;
	 tseg++) {
	brp = clock / ((1 + tseg / 2) * rate) + tseg % 2;
	if ((brp == 0) || (brp > 64))
	    continue;

	error = rate - clock / (brp * (1 + tseg / 2));
	if (error < 0)
	    error = -error;

	if (error <= best_error) {
	    best_error = error;
	    best_tseg = tseg/2;
	    best_brp = brp - 1;
	    best_rate = clock / (brp * (1 + tseg / 2));
	}
    }

    if (best_error && (rate / best_error < 10)) {
	RTCAN_RTDM_DBG("%s: bitrate %d is not possible with %d Hz clock\n",
		       dev->name, rate, clock);
	return -EDOM;
    }

    tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100;

    if (tseg2 < 0)
	tseg2 = 0;

    if (tseg2 > RTCAN_MAX_TSEG2)
	tseg2 = RTCAN_MAX_TSEG2;

    tseg1 = best_tseg - tseg2 - 2;

    if (tseg1 > RTCAN_MAX_TSEG1)  {
	tseg1 = RTCAN_MAX_TSEG1;
	tseg2 = best_tseg-tseg1-2;
    }

    bit_time->brp = best_brp + 1;
    bit_time->prop_seg = 0;
    bit_time->phase_seg1 = tseg1 + 1;
    bit_time->phase_seg2 = tseg2 + 1;
    bit_time->sjw = sjw + 1;
    bit_time->sam = 0;

    return 0;
}

#else /* !CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD */

/* This is the bit-time calculation method from the Linux kernel */

#define CAN_CALC_MAX_ERROR 50 /* in one-tenth of a percent */

static int can_update_spt(const struct can_bittiming_const *btc,
			  unsigned int sampl_pt, unsigned int tseg,
			  unsigned int *tseg1, unsigned int *tseg2)
{
    *tseg2 = tseg + 1 - (sampl_pt * (tseg + 1)) / 1000;
    *tseg2 = clamp(*tseg2, btc->tseg2_min, btc->tseg2_max);
    *tseg1 = tseg - *tseg2;
    if (*tseg1 > btc->tseg1_max) {
	*tseg1 = btc->tseg1_max;
	*tseg2 = tseg - *tseg1;
    }

    return 1000 * (tseg + 1 - *tseg2) / (tseg + 1);
}

static int rtcan_calc_bit_time(struct rtcan_device *dev,
			       can_baudrate_t bitrate,
			       struct can_bittime_std *bt)
{
    const struct can_bittiming_const *btc = dev->bittiming_const;
    long rate;	/* current bitrate */
    long rate_error;/* difference between current and target value */
    long best_rate_error = 1000000000;
    int spt;	/* current sample point in thousandth */
    int spt_error;	/* difference between current and target value */
    int best_spt_error = 1000;
    int sampl_pt;	/* target sample point */
    int best_tseg = 0, best_brp = 0;	/* current best values for tseg and brp */
    unsigned int brp, tsegall, tseg, tseg1, tseg2;
    u64 v64;

    if (!dev->bittiming_const)
	return -ENOTSUPP;

    /* Use CIA recommended sample points */
    if (bitrate > 800000)
	sampl_pt = 750;
    else if (bitrate > 500000)
	sampl_pt = 800;
    else
	sampl_pt = 875;

    /* tseg even = round down, odd = round up */
    for (tseg = (btc->tseg1_max + btc->tseg2_max) * 2 + 1;
	 tseg >= (btc->tseg1_min + btc->tseg2_min) * 2; tseg--) {
	tsegall = 1 + tseg / 2;

	/* Compute all possible tseg choices (tseg=tseg1+tseg2) */
	brp = dev->can_sys_clock / (tsegall * bitrate) + tseg % 2;

	/* chose brp step which is possible in system */
	brp = (brp / btc->brp_inc) * btc->brp_inc;
	if ((brp < btc->brp_min) || (brp > btc->brp_max))
	    continue;

	rate = dev->can_sys_clock / (brp * tsegall);
	rate_error = abs((long)(bitrate - rate));

	/* tseg brp biterror */
	if (rate_error > best_rate_error)
	    continue;

	/* reset sample point error if we have a better bitrate */
	if (rate_error < best_rate_error)
	    best_spt_error = 1000;

	spt = can_update_spt(btc, sampl_pt, tseg / 2, &tseg1, &tseg2);
	spt_error = abs((long)(sampl_pt - spt));
	if (spt_error > best_spt_error)
	    continue;

	best_spt_error = spt_error;
	best_rate_error = rate_error;
	best_tseg = tseg / 2;
	best_brp = brp;

	if (rate_error == 0 && spt_error == 0)
	    break;
    }

    if (best_rate_error) {
	/* Error in one-tenth of a percent */
	rate_error = (best_rate_error * 1000) / bitrate;
	if (rate_error > CAN_CALC_MAX_ERROR) {
	    rtcandev_err(dev,
			 "bitrate error %ld.%ld%% too high\n",
			 rate_error / 10, rate_error % 10);
	    return -EDOM;
	} else {
	    rtcandev_warn(dev, "bitrate error %ld.%ld%%\n",
			  rate_error / 10, rate_error % 10);
	}
    }

    /* real sample point */
    sampl_pt = can_update_spt(btc, sampl_pt, best_tseg, &tseg1, &tseg2);

    v64 = (u64)best_brp * 1000000000UL;
    do_div(v64, dev->can_sys_clock);
    bt->prop_seg = tseg1 / 2;
    bt->phase_seg1 = tseg1 - bt->prop_seg;
    bt->phase_seg2 = tseg2;
    bt->sjw = 1;
    bt->sam = 0;
    bt->brp = best_brp;

    /* real bit-rate */
    rate = dev->can_sys_clock / (bt->brp * (tseg1 + tseg2 + 1));

    rtcandev_dbg(dev, "real bitrate %ld, sampling point %d.%d%%\n",
		 rate, sampl_pt/10, sampl_pt%10);

    return 0;
}

#endif /* CONFIG_XENO_DRIVERS_CAN_CALC_BITTIME_OLD */

static inline int rtcan_raw_ioctl_dev_get(struct rtcan_device *dev,
					  int request, struct can_ifreq *ifr)
{
    rtdm_lockctx_t lock_ctx;

    switch (request) {

    case SIOCGIFINDEX:
	ifr->ifr_ifindex = dev->ifindex;
	break;

    case SIOCGCANSTATE:
	rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);
	if (dev->do_get_state)
	    dev->state = dev->do_get_state(dev);
	ifr->ifr_ifru.state = dev->state;
	rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);
	break;

    case SIOCGCANCTRLMODE:
	ifr->ifr_ifru.ctrlmode = dev->ctrl_mode;
	break;

    case SIOCGCANBAUDRATE:
	ifr->ifr_ifru.baudrate = dev->baudrate;
	break;

    case SIOCGCANCUSTOMBITTIME:
	ifr->ifr_ifru.bittime = dev->bit_time;
	break;
    }

    return 0;
}

static inline int rtcan_raw_ioctl_dev_set(struct rtcan_device *dev,
					  int request, struct can_ifreq *ifr)
{
    rtdm_lockctx_t lock_ctx;
    int ret = 0, started = 0;
    struct can_bittime bit_time, *bt;

    switch (request) {
    case SIOCSCANBAUDRATE:
	if (!dev->do_set_bit_time)
	    return 0;
	ret = rtcan_calc_bit_time(dev, ifr->ifr_ifru.baudrate, &bit_time.std);
	if (ret)
	    break;
	bit_time.type = CAN_BITTIME_STD;
	break;
    }

    rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);

    if (dev->do_get_state)
	dev->state = dev->do_get_state(dev);

    switch (request) {
    case SIOCSCANCTRLMODE:
    case SIOCSCANBAUDRATE:
    case SIOCSCANCUSTOMBITTIME:
	if ((started = CAN_STATE_OPERATING(dev->state))) {
	    if ((ret = dev->do_set_mode(dev, CAN_MODE_STOP, &lock_ctx)))
		goto out;
	}
	break;
    }

    switch (request) {
    case SIOCSCANMODE:
	if (dev->do_set_mode &&
	    !(ifr->ifr_ifru.mode == CAN_MODE_START &&
	      CAN_STATE_OPERATING(dev->state)))
	    ret = dev->do_set_mode(dev, ifr->ifr_ifru.mode, &lock_ctx);
	break;

    case SIOCSCANCTRLMODE:
	dev->ctrl_mode = ifr->ifr_ifru.ctrlmode;
	break;

    case SIOCSCANBAUDRATE:
	ret = dev->do_set_bit_time(dev, &bit_time, &lock_ctx);
	if (!ret) {
	    dev->baudrate = ifr->ifr_ifru.baudrate;
	    dev->bit_time = bit_time;
	}
	break;

    case SIOCSCANCUSTOMBITTIME:
	bt = &ifr->ifr_ifru.bittime;
	ret = dev->do_set_bit_time(dev, bt, &lock_ctx);
	if (!ret) {
	    dev->bit_time = *bt;
	    if (bt->type == CAN_BITTIME_STD && bt->std.brp)
		dev->baudrate = (dev->can_sys_clock /
				 (bt->std.brp * (1 + bt->std.prop_seg +
						 bt->std.phase_seg1 +
						 bt->std.phase_seg2)));
	    else
		dev->baudrate = CAN_BAUDRATE_UNKNOWN;
	}
	break;

    default:
	ret = -EOPNOTSUPP;
	break;
    }

 out:
    if (started)
	dev->do_set_mode(dev, CAN_MODE_START, &lock_ctx);

    rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);

    return ret;
}

int rtcan_raw_ioctl_dev(struct rtdm_fd *fd, int request, void *arg)
{
    struct can_ifreq *ifr;
    int ret = 0, get = 0;
    union {
	    /*
	     * We need to deal with callers still passing struct ifreq
	     * instead of can_ifreq, which might have a larger memory
	     * footprint (but can't be smaller though). Field offsets
	     * will be the same regardless.
	     */
	    struct ifreq ifr_legacy;
	    struct can_ifreq ifr_can;
    } ifr_buf;
    struct rtcan_device *dev;

    switch (request) {

    case SIOCGIFINDEX:
    case SIOCGCANSTATE:
    case SIOCGCANBAUDRATE:
    case SIOCGCANCUSTOMBITTIME:
	    get = 1;
	    /* fallthrough */
    case SIOCSCANMODE:
    case SIOCSCANCTRLMODE:
    case SIOCSCANBAUDRATE:
    case SIOCSCANCUSTOMBITTIME:

	if (rtdm_fd_is_user(fd)) {
	    /* Copy struct can_ifreq from userspace */
	    if (!rtdm_read_user_ok(fd, arg,
				   sizeof(struct can_ifreq)) ||
		rtdm_copy_from_user(fd, &ifr_buf, arg,
				    sizeof(struct can_ifreq)))
		return -EFAULT;

	    ifr = &ifr_buf.ifr_can;
	} else
	    ifr = (struct can_ifreq *)arg;

	/* Get interface index and data */
	dev = rtcan_dev_get_by_name(ifr->ifr_name);
	if (dev == NULL)
	    return -ENODEV;

	if (get) {
		ret = rtcan_raw_ioctl_dev_get(dev, request, ifr);
		rtcan_dev_dereference(dev);
		if (ret == 0 && rtdm_fd_is_user(fd)) {
		    /*
		     * Since we yet tested if user memory is rw safe,
		     * we can copy to user space directly.
		     */
		    if (rtdm_copy_to_user(fd, arg, ifr,
					  sizeof(struct can_ifreq)))
			    return -EFAULT;
		}
	} else {
		ret = rtcan_raw_ioctl_dev_set(dev, request, ifr);
		rtcan_dev_dereference(dev);
	}
	break;

    default:
	ret = -EOPNOTSUPP;
	break;

    }

    return ret;
}

#ifdef CONFIG_XENO_DRIVERS_CAN_BUS_ERR
void __rtcan_raw_enable_bus_err(struct rtcan_socket *sock)
{
    int i, begin, end;
    struct rtcan_device *dev;
    rtdm_lockctx_t lock_ctx;
    int ifindex = atomic_read(&sock->ifindex);

    if (ifindex) {
	begin = ifindex;
	end   = ifindex;
    } else {
	begin = 1;
	end = RTCAN_MAX_DEVICES;
    }

    for (i = begin; i <= end; i++) {
	if ((dev = rtcan_dev_get_by_index(i)) == NULL)
	    continue;

	if (dev->do_enable_bus_err) {
	    rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);
	    dev->do_enable_bus_err(dev);
	    rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);
	}
	rtcan_dev_dereference(dev);
    }
}
#endif /* CONFIG_XENO_DRIVERS_CAN_BUS_ERR*/
