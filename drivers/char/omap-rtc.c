/*
 *	TI OMAP Real Time Clock interface for Linux	
 *
 *	Copyright (C) 2003 MontaVista Software, Inc.
 *      Author: George G. Davis <gdavis@mvista.com> or <source@mvista.com>
 *
 *	Initially based on linux-2.4.20/drivers/char/rtc.c
 *	Copyright (C) 1996 Paul Gortmaker
 *
 *	This driver allows use of the real time clock (built into
 *	nearly all computers) from user space. It exports the /dev/rtc
 *	interface supporting various ioctl() and also the
 *	/proc/driver/rtc pseudo-file for status information.
 *
 *	The ioctls can be used to set the interrupt behaviour from the
 *	RTC via IRQs. Then the /dev/rtc	interface can be used to make
 *	use of RTC interrupts, be they time update or alarm based.
 *
 *	The /dev/rtc interface will block on reads until an interrupt
 *	has been received. If a RTC interrupt has already happened,
 *	it will output an unsigned long and then block. The output value
 *	contains the interrupt status in the low byte and the number of
 *	interrupts since the last read in the remaining high bytes. The 
 *	/dev/rtc interface can also be used with the select(2) call.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Based on other minimal char device drivers, like Alan's
 *	watchdog, Ted's random, etc. etc.
 *
 * Change Log :
 *      v1.0    <gdavis@mvista.com> Initial version based on rtc.c v1.10e
 *              <ramakrishnan@india.ti.com> Added support for 2.6 kernel, 
 *                  - changed the return value of the interrupt handler
 */

/*
 *	Note that *all* calls to CMOS_READ and CMOS_WRITE are done with
 *	interrupts disabled.
 *	REVISIT: Elaborate on OMAP1510 TRM 15uS BUSY access rule.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/rtc.h>
#include <asm/mach/time.h>

#include "omap-rtc.h"

extern spinlock_t rtc_lock;

static int omap_rtc_alarm = NO_IRQ;
static int omap_rtc_timer = NO_IRQ;


/* OMAP RTC register access macros: */

#define CMOS_READ(addr)		omap_readb(addr)
#define CMOS_WRITE(val, addr)	omap_writeb(val, addr)

static struct fasync_struct *rtc_async_queue;

static DECLARE_WAIT_QUEUE_HEAD(rtc_wait);

static void get_rtc_time (struct rtc_time *rtc_tm);
static void get_rtc_alm_time (struct rtc_time *alm_tm);

static void set_rtc_irq_bit(unsigned char bit);
static void mask_rtc_irq_bit(unsigned char bit);

static int rtc_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data);

/*
 *	Bits in rtc_status. (7 bits of room for future expansion)
 */

#define RTC_IS_OPEN		0x01	/* means /dev/rtc is in use	*/

/*
 * REVISIT: fix this comment:
 * rtc_status is never changed by rtc_interrupt, and ioctl/open/close is
 * protected by the big kernel lock.
 */
static unsigned long rtc_status = 0;	/* bitmapped status byte.	*/
static unsigned long rtc_irq_data = 0;	/* our output to the world	*/

/*
 *	If this driver ever becomes modularised, it will be really nice
 *	to make the epoch retain its value across module reload...
 */

static unsigned long epoch = 1900;	/* year corresponding to 0x00	*/

static const unsigned char days_in_mo[] = 
{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*
 *	A very tiny interrupt handler. It runs with SA_INTERRUPT set.
 */

irqreturn_t rtc_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	/*
	 *	Either an alarm interrupt or update complete interrupt.
	 *	We store the status in the low byte and the number of
	 *	interrupts received since the last read in the remainder
	 *	of rtc_irq_data.
	 */

	spin_lock (&rtc_lock);

	rtc_irq_data += 0x100;
	rtc_irq_data &= ~0xff;
	rtc_irq_data |= CMOS_READ(OMAP_RTC_STATUS_REG);

	if (rtc_irq_data & OMAP_RTC_STATUS_ALARM)
		CMOS_WRITE(OMAP_RTC_STATUS_ALARM, OMAP_RTC_STATUS_REG);

	spin_unlock (&rtc_lock);

	/* Now do the rest of the actions */
	wake_up_interruptible(&rtc_wait);	

	kill_fasync (&rtc_async_queue, SIGIO, POLL_IN);
	return IRQ_HANDLED;
}

/*
 *	Now all the various file operations that we export.
 */

static ssize_t rtc_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data;
	ssize_t retval;
	
	if (count < sizeof(unsigned long))
		return -EINVAL;

	add_wait_queue(&rtc_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	for (;;) {
		spin_lock_irq (&rtc_lock);
		data = rtc_irq_data;
		if (data != 0) {
			rtc_irq_data = 0;
			break;
		}
		spin_unlock_irq (&rtc_lock);

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	}

	spin_unlock_irq (&rtc_lock);
	retval = put_user(data, (unsigned long __user *)buf);
	if (!retval)
		retval = sizeof(unsigned long); 
 out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&rtc_wait, &wait);

	return retval;
}

/* convert from userspace struct to hardware BCD-encoded version,
 * or return error code
 */
static int utm2bcd(struct rtc_time __user *arg, struct rtc_time *tm)
{
	unsigned char leap_yr;

	if (copy_from_user(tm, arg, sizeof(struct rtc_time)))
		return -EFAULT;

	tm->tm_year += 1900;
	tm->tm_mon++;

	if (tm->tm_year < 1970)
		return -EINVAL;

	leap_yr = (!(tm->tm_year % 4) && (tm->tm_year % 100))
			|| !(tm->tm_year % 400);

	if ((tm->tm_mon > 12) || (tm->tm_mday == 0))
		return -EINVAL;

	if (tm->tm_mday > (days_in_mo[tm->tm_mon] + ((tm->tm_mon == 2) && leap_yr)))
		return -EINVAL;

	if ((tm->tm_hour >= 24) || (tm->tm_min >= 60) || (tm->tm_sec >= 60))
		return -EINVAL;

	if ((tm->tm_year -= epoch) > 255)    /* They are unsigned */
		return -EINVAL;

	if (tm->tm_year > 169)
		return -EINVAL;

	if (tm->tm_year >= 100)
		tm->tm_year -= 100;

	BIN_TO_BCD(tm->tm_sec);
	BIN_TO_BCD(tm->tm_min);
	BIN_TO_BCD(tm->tm_hour);
	BIN_TO_BCD(tm->tm_mday);
	BIN_TO_BCD(tm->tm_mon);
	BIN_TO_BCD(tm->tm_year);

	return 0;
}

static int rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	struct rtc_time wtime; 
	int status = 0;
	u8 save_control;

	switch (cmd) {
	case RTC_AIE_OFF:	/* Mask alarm int. enab. bit	*/
		mask_rtc_irq_bit(OMAP_RTC_INTERRUPTS_IT_ALARM);
		break;
	case RTC_AIE_ON:	/* Allow alarm interrupts.	*/
		set_rtc_irq_bit(OMAP_RTC_INTERRUPTS_IT_ALARM);
		break;
	case RTC_UIE_OFF:	/* Mask ints from RTC updates.	*/
		mask_rtc_irq_bit(OMAP_RTC_INTERRUPTS_IT_TIMER);
		break;
	case RTC_UIE_ON:	/* Allow ints for RTC updates.	*/
		set_rtc_irq_bit(OMAP_RTC_INTERRUPTS_IT_TIMER);
		break;
	case RTC_ALM_READ:	/* Read the present alarm time */
		/*
		 * This returns a struct rtc_time. Reading >= 0xc0
		 * means "don't care" or "match all". Only the tm_hour,
		 * tm_min, and tm_sec values are filled in.
		 */
		memset(&wtime, 0, sizeof(struct rtc_time));
		get_rtc_alm_time(&wtime);
		goto return_wtime;
	case RTC_ALM_SET:	/* Store a time into the alarm */
		status = utm2bcd((void __user *)arg, &wtime);
		if (status != 0)
			return status;

		spin_lock_irq(&rtc_lock);
		CMOS_WRITE(wtime.tm_year, OMAP_RTC_ALARM_YEARS_REG);
		CMOS_WRITE(wtime.tm_mon, OMAP_RTC_ALARM_MONTHS_REG);
		CMOS_WRITE(wtime.tm_mday, OMAP_RTC_ALARM_DAYS_REG);
		CMOS_WRITE(wtime.tm_hour, OMAP_RTC_ALARM_HOURS_REG);
		CMOS_WRITE(wtime.tm_min, OMAP_RTC_ALARM_MINUTES_REG);
		CMOS_WRITE(wtime.tm_sec, OMAP_RTC_ALARM_SECONDS_REG);
		spin_unlock_irq(&rtc_lock);

		break;
	case RTC_RD_TIME:	/* Read the time/date from RTC	*/
		memset(&wtime, 0, sizeof(struct rtc_time));
		get_rtc_time(&wtime);
		goto return_wtime;
	case RTC_SET_TIME:	/* Set the RTC */
		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		status = utm2bcd((void __user *)arg, &wtime);
		if (status != 0)
			return status;

		spin_lock_irq(&rtc_lock);
		save_control = CMOS_READ(OMAP_RTC_CTRL_REG);
		CMOS_WRITE((save_control & ~OMAP_RTC_CTRL_STOP),
			   OMAP_RTC_CTRL_REG);
		CMOS_WRITE(wtime.tm_year, OMAP_RTC_YEARS_REG);
		CMOS_WRITE(wtime.tm_mon, OMAP_RTC_MONTHS_REG);
		CMOS_WRITE(wtime.tm_mday, OMAP_RTC_DAYS_REG);
		CMOS_WRITE(wtime.tm_hour, OMAP_RTC_HOURS_REG);
		CMOS_WRITE(wtime.tm_min, OMAP_RTC_MINUTES_REG);
		CMOS_WRITE(wtime.tm_sec, OMAP_RTC_SECONDS_REG);
		CMOS_WRITE((save_control | OMAP_RTC_CTRL_STOP),
			   OMAP_RTC_CTRL_REG);
		spin_unlock_irq(&rtc_lock);

		break;
	case RTC_EPOCH_READ:	/* Read the epoch.	*/
		status = put_user (epoch, (unsigned long  __user *)arg);
		break;
	case RTC_EPOCH_SET:	/* Set the epoch.	*/
		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		/* 
		 * There were no RTC clocks before 1900.
		 */
		if (arg < 1900)
			status = -EINVAL;
		else
			epoch = arg;
		break;
	default:
		status = -ENOTTY;
	}
	return status;

return_wtime:
	return copy_to_user((void  __user *)arg, &wtime, sizeof wtime)
		? -EFAULT
		: 0;
}

/*
 *	We enforce only one user at a time here with the open/close.
 *	Also clear the previous interrupt data on an open, and clean
 *	up things on a close.
 */

/* We use rtc_lock to protect against concurrent opens. So the BKL is not
 * needed here. Or anywhere else in this driver. */
static int rtc_open(struct inode *inode, struct file *file)
{
	spin_lock_irq (&rtc_lock);

	if (rtc_status & RTC_IS_OPEN)
		goto out_busy;

	rtc_status |= RTC_IS_OPEN;

	rtc_irq_data = 0;
	spin_unlock_irq (&rtc_lock);
	return 0;

out_busy:
	spin_unlock_irq (&rtc_lock);
	return -EBUSY;
}

static int rtc_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper (fd, filp, on, &rtc_async_queue);
}

static int rtc_release(struct inode *inode, struct file *file)
{
	unsigned char tmp;

	/*
	 * Turn off all interrupts once the device is no longer
	 * in use, and clear the data.
	 */

	spin_lock_irq(&rtc_lock);
	tmp = CMOS_READ(OMAP_RTC_INTERRUPTS_REG);
	tmp &=  ~OMAP_RTC_INTERRUPTS_IT_ALARM;
	tmp &=  ~OMAP_RTC_INTERRUPTS_IT_TIMER;
	CMOS_WRITE(tmp, OMAP_RTC_INTERRUPTS_REG);
	spin_unlock_irq(&rtc_lock);

	if (file->f_flags & FASYNC) {
		rtc_fasync (-1, file, 0);
	}

	spin_lock_irq (&rtc_lock);
	rtc_irq_data = 0;
	spin_unlock_irq (&rtc_lock);

	/* No need for locking -- nobody else can do anything until this rmw
	 * is committed, and we don't implement timer support in omap-rtc.
	 */
	rtc_status &= ~RTC_IS_OPEN;
	return 0;
}

/* Called without the kernel lock - fine */
static unsigned int rtc_poll(struct file *file, poll_table *wait)
{
	unsigned long l;

	poll_wait(file, &rtc_wait, wait);

	spin_lock_irq (&rtc_lock);
	l = rtc_irq_data;
	spin_unlock_irq (&rtc_lock);

	if (l != 0)
		return POLLIN | POLLRDNORM;
	return 0;
}

/*
 *	The various file operations we support.
 */

static struct file_operations rtc_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= rtc_read,
	.poll		= rtc_poll,
	.ioctl		= rtc_ioctl,
	.open		= rtc_open,
	.release	= rtc_release,
	.fasync		= rtc_fasync,
};

static struct miscdevice rtc_dev = {
	.minor		= RTC_MINOR,
	.name		= "rtc",
	.fops		= &rtc_fops,
};

static int __init omap_rtc_probe(struct platform_device *pdev)
{
	struct resource		*res, *mem;

	/* find the IRQs */

	omap_rtc_timer = platform_get_irq(pdev, 0);
	if (omap_rtc_timer <= 0) {
		dev_err(&pdev->dev, "no irq for rtc timer\n");
		return -ENOENT;
	}

	omap_rtc_alarm = platform_get_irq(pdev, 1);
	if (omap_rtc_alarm <= 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		mem = request_mem_region(res->start,
				res->end - res->start + 1,
				pdev->name);
	else
		mem = NULL;
	if (!mem) {
		pr_debug("%s: RTC registers at %x are not free.\n",
			pdev->name, OMAP_RTC_BASE);
		return -EBUSY;
	}
	platform_set_drvdata(pdev, mem);

	if (CMOS_READ(OMAP_RTC_STATUS_REG) & OMAP_RTC_STATUS_POWER_UP) {
		pr_info("%s: RTC power up reset detected.\n",
			pdev->name);
		/* Clear OMAP_RTC_STATUS_POWER_UP */
		CMOS_WRITE(OMAP_RTC_STATUS_POWER_UP, OMAP_RTC_STATUS_REG);
	}

	if (CMOS_READ(OMAP_RTC_STATUS_REG) & OMAP_RTC_STATUS_ALARM) {
		pr_debug("%s: Clearing RTC ALARM interrupt.\n",
			pdev->name);
		/* Clear OMAP_RTC_STATUS_ALARM */
		CMOS_WRITE(OMAP_RTC_STATUS_ALARM, OMAP_RTC_STATUS_REG);
	}

	if (request_irq(omap_rtc_timer, rtc_interrupt, SA_INTERRUPT,
			pdev->name, NULL)) {
		pr_debug("%s: RTC timer interrupt IRQ%d is not free.\n",
			pdev->name, omap_rtc_timer);
		goto fail;
	}

	if (request_irq(omap_rtc_alarm, rtc_interrupt, SA_INTERRUPT,
			pdev->name, NULL)) {
		pr_debug("%s: RTC alarm interrupt IRQ%d is not free.\n",
			pdev->name, omap_rtc_alarm);
		free_irq(omap_rtc_timer, NULL);
		goto fail;
	}

	/* On boards with split power, RTC_ON_NOFF resets all but the RTC */
	if (!(CMOS_READ(OMAP_RTC_CTRL_REG) & OMAP_RTC_CTRL_STOP)) {
		pr_info("%s: Enabling RTC.\n", pdev->name);
		CMOS_WRITE(OMAP_RTC_CTRL_STOP, OMAP_RTC_CTRL_REG);
	} else
		pr_info("%s: RTC already running.\n", pdev->name);

	spin_lock_init(&rtc_lock);
	misc_register(&rtc_dev);
	create_proc_read_entry("driver/rtc", 0, NULL, rtc_read_proc, NULL);

	return 0;

fail:
	release_resource(mem);
	return -EIO;
}

static int omap_rtc_remove(struct platform_device *pdev)
{
	free_irq (omap_rtc_timer, NULL);
	free_irq (omap_rtc_alarm, NULL);

	remove_proc_entry ("driver/rtc", NULL);
	misc_deregister(&rtc_dev);

	release_resource(platform_get_drvdata(pdev));
	return 0;
}

/*
 *	Info exported via "/proc/driver/rtc".
 */

static int rtc_proc_output (char *buf)
{
#define YN(value) ((value) ? "yes" : "no")
	char *p;
	struct rtc_time tm;

	p = buf;

	get_rtc_time(&tm);

	/*
	 * There is no way to tell if the luser has the RTC set for local
	 * time or for Universal Standard Time (GMT). Probably local though.
	 */
	p += sprintf(p,
		     "rtc_time\t: %02d:%02d:%02d\n"
		     "rtc_date\t: %04d-%02d-%02d\n"
	 	     "rtc_epoch\t: %04lu\n",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, epoch);

	get_rtc_alm_time(&tm);

	/*
	 * We implicitly assume 24hr mode here. Alarm values >= 0xc0 will
	 * match any value for that particular field. Values that are
	 * greater than a valid time, but less than 0xc0 shouldn't appear.
	 */
	p += sprintf(p,
		     "alarm_time\t: %02d:%02d:%02d\n"
		     "alarm_date\t: %04d-%02d-%02d\n",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);

	p += sprintf(p,
		     "BCD\t\t: %s\n"
		     "24hr\t\t: %s\n"
		     "alarm_IRQ\t: %s\n"
		     "update_IRQ\t: %s\n"
		     "update_rate\t: %ud\n",
		     YN(1),
		     YN(1),
		     YN(CMOS_READ(OMAP_RTC_INTERRUPTS_REG) &
			OMAP_RTC_INTERRUPTS_IT_ALARM),
		     YN(CMOS_READ(OMAP_RTC_INTERRUPTS_REG) &
			OMAP_RTC_INTERRUPTS_IT_TIMER),
		     CMOS_READ(OMAP_RTC_INTERRUPTS_REG) & 3 /* REVISIT */);

	return  p - buf;
#undef YN
}

static int rtc_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = rtc_proc_output (page);

	if (len <= off+count)
		*eof = 1;
        *start = page + off;
        len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
        return len;
}

/*
 * Returns true if a clock update is in progress
 */
static inline unsigned char rtc_is_updating(void)
{
	unsigned char uip;

	spin_lock_irq(&rtc_lock);
	uip = (CMOS_READ(OMAP_RTC_STATUS_REG) & OMAP_RTC_STATUS_BUSY);
	spin_unlock_irq(&rtc_lock);
	return uip;
}

static void bcd2tm(struct rtc_time *tm)
{
	BCD_TO_BIN(tm->tm_sec);
	BCD_TO_BIN(tm->tm_min);
	BCD_TO_BIN(tm->tm_hour);
	BCD_TO_BIN(tm->tm_mday);
	BCD_TO_BIN(tm->tm_mon);
	BCD_TO_BIN(tm->tm_year);

	/*
	 * Account for differences between how the RTC uses the values
	 * and how they are defined in a struct rtc_time;
	 */
	if ((tm->tm_year += (epoch - 1900)) <= 69)
		tm->tm_year += 100;

	tm->tm_mon--;
}


static void get_rtc_time(struct rtc_time *rtc_tm)
{
	unsigned char ctrl;

	/* REVISIT: Fix this comment!!!
	 * read RTC once any update in progress is done. The update
	 * can take just over 2ms. We wait 10 to 20ms. There is no need to
	 * to poll-wait (up to 1s - eeccch) for the falling edge of OMAP_RTC_STATUS_BUSY.
	 * If you need to know *exactly* when a second has started, enable
	 * periodic update complete interrupts, (via ioctl) and then 
	 * immediately read /dev/rtc which will block until you get the IRQ.
	 * Once the read clears, read the RTC time (again via ioctl). Easy.
	 */

#if	0 /* REVISIT: This need to do as the TRM says. */
	unsigned long uip_watchdog = jiffies;
	if (rtc_is_updating() != 0)
		while (jiffies - uip_watchdog < 2*HZ/100) {
			barrier();
			cpu_relax();
		}
#endif

	/*
	 * Only the values that we read from the RTC are set. We leave
	 * tm_wday, tm_yday and tm_isdst untouched. Even though the
	 * RTC has RTC_DAY_OF_WEEK, we ignore it, as it is only updated
	 * by the RTC when initially set to a non-zero value.
	 */
	spin_lock_irq(&rtc_lock);
	rtc_tm->tm_sec = CMOS_READ(OMAP_RTC_SECONDS_REG);
	rtc_tm->tm_min = CMOS_READ(OMAP_RTC_MINUTES_REG);
	rtc_tm->tm_hour = CMOS_READ(OMAP_RTC_HOURS_REG);
	rtc_tm->tm_mday = CMOS_READ(OMAP_RTC_DAYS_REG);
	rtc_tm->tm_mon = CMOS_READ(OMAP_RTC_MONTHS_REG);
	rtc_tm->tm_year = CMOS_READ(OMAP_RTC_YEARS_REG);
	ctrl = CMOS_READ(OMAP_RTC_CTRL_REG);
	spin_unlock_irq(&rtc_lock);

	bcd2tm(rtc_tm);
}

static void get_rtc_alm_time(struct rtc_time *alm_tm)
{
	unsigned char ctrl;

	spin_lock_irq(&rtc_lock);
	alm_tm->tm_sec = CMOS_READ(OMAP_RTC_ALARM_SECONDS_REG);
	alm_tm->tm_min = CMOS_READ(OMAP_RTC_ALARM_MINUTES_REG);
	alm_tm->tm_hour = CMOS_READ(OMAP_RTC_ALARM_HOURS_REG);
	alm_tm->tm_mday = CMOS_READ(OMAP_RTC_ALARM_DAYS_REG);
	alm_tm->tm_mon = CMOS_READ(OMAP_RTC_ALARM_MONTHS_REG);
	alm_tm->tm_year = CMOS_READ(OMAP_RTC_ALARM_YEARS_REG);
	ctrl = CMOS_READ(OMAP_RTC_CTRL_REG);
	spin_unlock_irq(&rtc_lock);

	bcd2tm(alm_tm);
}

/*
 * Used to disable/enable UIE and AIE interrupts.
 */

static void mask_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;

	spin_lock_irq(&rtc_lock);
	val = CMOS_READ(OMAP_RTC_INTERRUPTS_REG);
	val &=  ~bit;
	CMOS_WRITE(val, OMAP_RTC_INTERRUPTS_REG);
	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);
}

static void set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;

	spin_lock_irq(&rtc_lock);
	val = CMOS_READ(OMAP_RTC_INTERRUPTS_REG);
	val |= bit;
	CMOS_WRITE(val, OMAP_RTC_INTERRUPTS_REG);
	rtc_irq_data = 0;
	spin_unlock_irq(&rtc_lock);
}

#ifdef CONFIG_PM
static struct timespec rtc_delta;

static int omap_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time rtc_tm;
	struct timespec time;

	time.tv_nsec = 0;
	get_rtc_time(&rtc_tm);
	rtc_tm_to_time(&rtc_tm, &time.tv_sec);

	save_time_delta(&rtc_delta, &time);

	return 0;
}

static int omap_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time rtc_tm;
	struct timespec time;

	time.tv_nsec = 0;
	get_rtc_time(&rtc_tm);
	rtc_tm_to_time(&rtc_tm, &time.tv_sec);

	restore_time_delta(&rtc_delta, &time);

	return 0;
}
#else
#define omap_rtc_suspend NULL
#define omap_rtc_resume  NULL
#endif

static struct platform_driver omap_rtc_driver = {
	.probe		= omap_rtc_probe,
	.remove		= omap_rtc_remove,
	.suspend	= omap_rtc_suspend,
	.resume		= omap_rtc_resume,
	.driver		= {
		.name	= "omap_rtc",
		.owner	= THIS_MODULE,
	},
};

static char __initdata banner[] = KERN_INFO "OMAP RTC Driver\n";

static int __init rtc_init(void)
{
	printk(banner);
	return platform_driver_register(&omap_rtc_driver);
}

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&omap_rtc_driver);
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("George G. Davis (and others)");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(RTC_MINOR);
