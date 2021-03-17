/*
 * Analogy for Linux, RTDM helpers
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include <rtdm/analogy/rtdm_helpers.h>

/* --- Time section --- */

static nanosecs_abs_t a4l_clkofs;

void a4l_init_time(void)
{
	nanosecs_abs_t t1, t2;
	t1 = rtdm_clock_read();
	t2 = ktime_to_ns(ktime_get_real());
	a4l_clkofs = t2 - t1;
}

nanosecs_abs_t a4l_get_time(void)
{
	return a4l_clkofs + rtdm_clock_read();
}

/* --- IRQ section --- */

static int a4l_handle_irq(rtdm_irq_t *irq_handle)
{
	struct a4l_irq_descriptor *dsc =
		rtdm_irq_get_arg(irq_handle, struct a4l_irq_descriptor);

	if (dsc->handler((unsigned int)irq_handle->irq, dsc->cookie) == 0)
		return RTDM_IRQ_HANDLED;
	else
		return RTDM_IRQ_NONE;
}

int __a4l_request_irq(struct a4l_irq_descriptor *dsc,
		      unsigned int irq,
		      a4l_irq_hdlr_t handler,
		      unsigned long flags, void *cookie)
{
	/* Fills the IRQ descriptor */
	dsc->handler = handler;
	dsc->cookie = cookie;
	dsc->irq = irq;

	/* Registers the RT IRQ handler */
	return rtdm_irq_request(&dsc->rtdm_desc,
				(int)irq,
				a4l_handle_irq, flags, "Analogy device", dsc);
}

int __a4l_free_irq(struct a4l_irq_descriptor * dsc)
{
	return rtdm_irq_free(&dsc->rtdm_desc);
}

/* --- Synchronization section --- */

static void a4l_nrt_sync_handler(rtdm_nrtsig_t *nrt_sig, void *arg)
{
	struct a4l_sync *snc = (struct a4l_sync *) arg;
	wake_up_interruptible(&snc->wq);
}

int a4l_init_sync(struct a4l_sync *snc)
{
	int ret = 0;

	/* Initializes the flags field */
	snc->status = 0;

	/* If the process is NRT, we need a wait queue structure */
	init_waitqueue_head(&snc->wq);

	/* Initializes the RTDM event */
	rtdm_event_init(&snc->rtdm_evt, 0);

	/* Initializes the gateway to NRT context */
	rtdm_nrtsig_init(&snc->nrt_sig, a4l_nrt_sync_handler, snc);

	return ret;
}

void a4l_cleanup_sync(struct a4l_sync *snc)
{
	rtdm_nrtsig_destroy(&snc->nrt_sig);
	rtdm_event_destroy(&snc->rtdm_evt);
}

int a4l_wait_sync(struct a4l_sync *snc, int rt)
{
	int ret = 0;

	if (test_bit(__EVT_PDING, &snc->status))
		goto out_wait;

	if (rt != 0) {
		/* If the calling process is in primary mode,
		   we can use RTDM API ... */
		set_bit(__RT_WAITER, &snc->status);
		ret = rtdm_event_wait(&snc->rtdm_evt);
	} else {
		/* ... else if the process is NRT,
		   the Linux wait queue system is used */
		set_bit(__NRT_WAITER, &snc->status);
		ret = wait_event_interruptible(snc->wq,
					       test_bit(__EVT_PDING,
							&snc->status));
	}

out_wait:

	clear_bit(__EVT_PDING, &snc->status);

	return ret;
}

int a4l_timedwait_sync(struct a4l_sync * snc,
		       int rt, unsigned long long ns_timeout)
{
	int ret = 0;
	unsigned long timeout;

	if (test_bit(__EVT_PDING, &snc->status))
		goto out_wait;

	if (rt != 0) {
		/* If the calling process is in primary mode,
		   we can use RTDM API ... */
		set_bit(__RT_WAITER, &snc->status);
		ret = rtdm_event_timedwait(&snc->rtdm_evt, ns_timeout, NULL);
	} else {
		/* ... else if the process is NRT,
		   the Linux wait queue system is used */

		timeout = do_div(ns_timeout, 1000);

		/* We consider the Linux kernel cannot tick at a frequency
		   higher than 1 MHz
		   If the timeout value is lower than 1us, we round up to 1us */
		timeout = (timeout == 0) ? 1 : usecs_to_jiffies(timeout);

		set_bit(__NRT_WAITER, &snc->status);

		ret = wait_event_interruptible_timeout(snc->wq,
						       test_bit(__EVT_PDING,
								&snc->status),
						       timeout);
	}

out_wait:

	clear_bit(__EVT_PDING, &snc->status);

	return ret;
}

void a4l_flush_sync(struct a4l_sync * snc)
{
	/* Clear the status bitfield */
	snc->status = 0;

	/* Flush the RTDM event */
	rtdm_event_clear(&snc->rtdm_evt);
}

void a4l_signal_sync(struct a4l_sync * snc)
{
	int hit = 0;

	set_bit(__EVT_PDING, &snc->status);

	/* a4l_signal_sync() is bound not to be called upon the right
	   user process context; so, the status flags stores its mode.
	   Thus the proper event signaling function is called */
	if (test_and_clear_bit(__RT_WAITER, &snc->status)) {
		rtdm_event_signal(&snc->rtdm_evt);
		hit++;
	}

	if (test_and_clear_bit(__NRT_WAITER, &snc->status)) {
		rtdm_nrtsig_pend(&snc->nrt_sig);
		hit++;
	}

	if (hit == 0) {
		/* At first signaling, we may not know the proper way
		   to send the event */
		rtdm_event_signal(&snc->rtdm_evt);
		rtdm_nrtsig_pend(&snc->nrt_sig);
	}
}
