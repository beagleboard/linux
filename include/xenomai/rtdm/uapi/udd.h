/**
 * @file
 * This file is part of the Xenomai project.
 *
 * @author Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _RTDM_UAPI_UDD_H
#define _RTDM_UAPI_UDD_H

/**
 * @addtogroup rtdm_udd
 *
 * @{
 */

/**
 * @anchor udd_signotify
 * @brief UDD event notification descriptor
 *
 * This structure shall be used to pass the information required to
 * enable/disable the notification by signal upon interrupt receipt.
 *
 * If PID is zero or negative, the notification is disabled.
 * Otherwise, the Cobalt thread whose PID is given will receive the
 * Cobalt signal also mentioned, along with the count of interrupts at
 * the time of the receipt stored in siginfo.si_int. A Cobalt thread
 * must explicitly wait for notifications using the sigwaitinfo() or
 * sigtimedwait() services (no asynchronous mode available).
 */
struct udd_signotify {
	/**
	 * PID of the Cobalt thread to notify upon interrupt
	 * receipt. If @a pid is zero or negative, the notification is
	 * disabled.
	 */
	pid_t pid;
	/**
	 * Signal number to send to PID for notifying, which must be
	 * in the range [SIGRTMIN .. SIGRTMAX] inclusive. This value
	 * is not considered if @a pid is zero or negative.
	 */
	int sig;
};

/**
 * @anchor udd_ioctl_codes @name UDD_IOCTL
 * IOCTL requests
 *
 * @{
 */

/**
 * Enable the interrupt line. The UDD-class mini-driver should handle
 * this request when received through its ->ioctl() handler if
 * provided. Otherwise, the UDD core enables the interrupt line in the
 * interrupt controller before returning to the caller.
 */
#define UDD_RTIOC_IRQEN		_IO(RTDM_CLASS_UDD, 0)
/**
 * Disable the interrupt line. The UDD-class mini-driver should handle
 * this request when received through its ->ioctl() handler if
 * provided. Otherwise, the UDD core disables the interrupt line in
 * the interrupt controller before returning to the caller.
 *
 * @note The mini-driver must handle the UDD_RTIOC_IRQEN request for a
 * custom IRQ from its ->ioctl() handler, otherwise such request
 * receives -EIO from the UDD core.
 */
#define UDD_RTIOC_IRQDIS	_IO(RTDM_CLASS_UDD, 1)
/**
 * Enable/Disable signal notification upon interrupt event. A valid
 * @ref udd_signotify "notification descriptor" must be passed along
 * with this request, which is handled by the UDD core directly.
 *
 * @note The mini-driver must handle the UDD_RTIOC_IRQDIS request for
 * a custom IRQ from its ->ioctl() handler, otherwise such request
 * receives -EIO from the UDD core.
 */
#define UDD_RTIOC_IRQSIG	_IOW(RTDM_CLASS_UDD, 2, struct udd_signotify)

/** @} */
/** @} */

#endif /* !_RTDM_UAPI_UDD_H */
