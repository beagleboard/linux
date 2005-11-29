/*
 * linux/arch/arm/mach-omap/dsp/dsp_core.c
 *
 * OMAP DSP driver
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2005/06/07:  DSP Gateway version 3.3
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/signal.h>
#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/arch/dsp.h>
#include "hardware_dsp.h"
#include "dsp.h"
#include "ipbuf.h"


MODULE_AUTHOR("Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>");
MODULE_DESCRIPTION("OMAP DSP driver module");
MODULE_LICENSE("GPL");

enum mbseq_check_level {
	MBSEQ_CHECK_NONE,	/* no check */
	MBSEQ_CHECK_VERBOSE,	/* discard the illegal command and
				   error report */
	MBSEQ_CHECK_SILENT,	/* discard the illegal command */
};

static enum mbseq_check_level mbseq_check_level = MBSEQ_CHECK_VERBOSE;

static int mbx1_valid;
static struct sync_seq *mbseq;
static unsigned short mbseq_expect_tmp;
static unsigned short *mbseq_expect = &mbseq_expect_tmp;

/*
 * mailbox commands
 */
extern void mbx1_wdsnd(struct mbcmd *mb);
extern void mbx1_wdreq(struct mbcmd *mb);
extern void mbx1_bksnd(struct mbcmd *mb);
extern void mbx1_bkreq(struct mbcmd *mb);
extern void mbx1_bkyld(struct mbcmd *mb);
extern void mbx1_bksndp(struct mbcmd *mb);
extern void mbx1_bkreqp(struct mbcmd *mb);
extern void mbx1_tctl(struct mbcmd *mb);
extern void mbx1_poll(struct mbcmd *mb);
#ifdef OLD_BINARY_SUPPORT
/* v3.3 obsolete */
extern void mbx1_wdt(struct mbcmd *mb);
#endif
extern void mbx1_suspend(struct mbcmd *mb);
static void mbx1_kfunc(struct mbcmd *mb);
extern void mbx1_tcfg(struct mbcmd *mb);
extern void mbx1_tadd(struct mbcmd *mb);
extern void mbx1_tdel(struct mbcmd *mb);
extern void mbx1_dspcfg(struct mbcmd *mb);
extern void mbx1_regrw(struct mbcmd *mb);
extern void mbx1_getvar(struct mbcmd *mb);
extern void mbx1_err(struct mbcmd *mb);
extern void mbx1_dbg(struct mbcmd *mb);

static const struct cmdinfo
	cif_null     = { "Unknown",  CMD_L_TYPE_NULL,   NULL         },
	cif_wdsnd    = { "WDSND",    CMD_L_TYPE_TID,    mbx1_wdsnd   },
	cif_wdreq    = { "WDREQ",    CMD_L_TYPE_TID,    mbx1_wdreq   },
	cif_bksnd    = { "BKSND",    CMD_L_TYPE_TID,    mbx1_bksnd   },
	cif_bkreq    = { "BKREQ",    CMD_L_TYPE_TID,    mbx1_bkreq   },
	cif_bkyld    = { "BKYLD",    CMD_L_TYPE_NULL,   mbx1_bkyld   },
	cif_bksndp   = { "BKSNDP",   CMD_L_TYPE_TID,    mbx1_bksndp  },
	cif_bkreqp   = { "BKREQP",   CMD_L_TYPE_TID,    mbx1_bkreqp  },
	cif_tctl     = { "TCTL",     CMD_L_TYPE_TID,    mbx1_tctl    },
	cif_poll     = { "POLL",     CMD_L_TYPE_NULL,   mbx1_poll    },
#ifdef OLD_BINARY_SUPPORT
	/* v3.3 obsolete */
	cif_wdt      = { "WDT",      CMD_L_TYPE_NULL,   mbx1_wdt     },
#endif
	cif_runlevel = { "RUNLEVEL", CMD_L_TYPE_SUBCMD, NULL         },
	cif_pm       = { "PM",       CMD_L_TYPE_SUBCMD, NULL         },
	cif_suspend  = { "SUSPEND",  CMD_L_TYPE_NULL,   mbx1_suspend },
	cif_kfunc    = { "KFUNC",    CMD_L_TYPE_SUBCMD, mbx1_kfunc   },
	cif_tcfg     = { "TCFG",     CMD_L_TYPE_TID,    mbx1_tcfg    },
	cif_tadd     = { "TADD",     CMD_L_TYPE_TID,    mbx1_tadd    },
	cif_tdel     = { "TDEL",     CMD_L_TYPE_TID,    mbx1_tdel    },
	cif_tstop    = { "TSTOP",    CMD_L_TYPE_TID,    NULL         },
	cif_dspcfg   = { "DSPCFG",   CMD_L_TYPE_SUBCMD, mbx1_dspcfg  },
	cif_regrw    = { "REGRW",    CMD_L_TYPE_SUBCMD, mbx1_regrw   },
	cif_getvar   = { "GETVAR",   CMD_L_TYPE_SUBCMD, mbx1_getvar  },
	cif_setvar   = { "SETVAR",   CMD_L_TYPE_SUBCMD, NULL         },
	cif_err      = { "ERR",      CMD_L_TYPE_SUBCMD, mbx1_err     },
	cif_dbg      = { "DBG",      CMD_L_TYPE_NULL,   mbx1_dbg     };

const struct cmdinfo *cmdinfo[128] = {
/*00*/	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*10*/	&cif_wdsnd, &cif_wdreq, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*20*/	&cif_bksnd, &cif_bkreq, &cif_null, &cif_bkyld,
	&cif_bksndp, &cif_bkreqp, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*30*/	&cif_tctl, &cif_null, &cif_poll, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*40*/	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
#ifdef OLD_BINARY_SUPPORT
	/* v3.3 obsolete */
/*50*/	&cif_wdt, &cif_runlevel, &cif_pm, &cif_suspend,
#else
/*50*/	&cif_null, &cif_runlevel, &cif_pm, &cif_suspend,
#endif
	&cif_kfunc, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*60*/	&cif_tcfg, &cif_null, &cif_tadd, &cif_tdel,
	&cif_null, &cif_tstop, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null,
/*70*/	&cif_dspcfg, &cif_null, &cif_regrw, &cif_null,
	&cif_getvar, &cif_setvar, &cif_null, &cif_null,
	&cif_err, &cif_dbg, &cif_null, &cif_null,
	&cif_null, &cif_null, &cif_null, &cif_null
};

int sync_with_dsp(unsigned short *syncwd, unsigned short tid, int try_cnt)
{
	int try;

	if (*(volatile unsigned short *)syncwd == tid)
		return 0;

	for (try = 0; try < try_cnt; try++) {
		udelay(1);
		if (*(volatile unsigned short *)syncwd == tid) {
			/* success! */
			printk(KERN_INFO
			       "omapdsp: sync_with_dsp(): try = %d\n", try);
			return 0;
		}
	}

	/* fail! */
	return -1;
}

static __inline__ int mbsync_irq_save(unsigned long *flags, int try_cnt)
{
	int cnt;

	local_irq_save(*flags);
	if (omap_readw(MAILBOX_ARM2DSP1_Flag) == 0)
		return 0;
	/*
	 * mailbox is busy. wait for some usecs...
	 */
	local_irq_restore(*flags);
	for (cnt = 0; cnt < try_cnt; cnt++) {
		udelay(1);
		local_irq_save(*flags);
		if (omap_readw(MAILBOX_ARM2DSP1_Flag) == 0)	/* success! */
			return 0;
		local_irq_restore(*flags);
	}

	/* fail! */
	return -1;
}

#ifdef CONFIG_OMAP_DSP_MBCMD_VERBOSE
#define print_mb_busy_abort(mb) \
	printk(KERN_DEBUG \
	       "mbx: mailbox is busy. %s is aborting.\n", cmd_name(*mb))
#define print_mb_mmu_abort(mb) \
	printk(KERN_DEBUG \
	       "mbx: mmu interrupt is set. %s is aborting.\n", cmd_name(*mb))
#else /* CONFIG_OMAP_DSP_MBCMD_VERBOSE */
#define print_mb_busy_abort(mb)	do {} while(0)
#define print_mb_mmu_abort(mb)	do {} while(0)
#endif /* !CONFIG_OMAP_DSP_MBCMD_VERBOSE */

int __mbcmd_send(struct mbcmd *mb)
{
	struct mbcmd_hw *mb_hw = (struct mbcmd_hw *)mb;
	unsigned long flags;

	/*
	 * DSP mailbox interrupt latency must be less than 1ms.
	 */
	if (mbsync_irq_save(&flags, 1000) < 0) {
		print_mb_busy_abort(mb);
		return -1;
	}

	if (mbseq) {
		mb->seq = mbseq->ad_arm;
		mbseq->ad_arm++;
	} else
		mb->seq = 0;
	mblog_add(mb, DIR_A2D);
	mblog_printcmd(mb, DIR_A2D);

	omap_writew(mb_hw->data, MAILBOX_ARM2DSP1);
	omap_writew(mb_hw->cmd, MAILBOX_ARM2DSP1b);

	local_irq_restore(flags);
	return 0;
}

/*
 * __dsp_mbcmd_send(): mailbox dispatcher
 */
int __dsp_mbcmd_send(struct mbcmd *mb, struct mb_exarg *arg, int recovery_flag)
{
	static DECLARE_MUTEX(mbsend_sem);
	int ret = 0;

	/*
	 * while MMU fault is set,
	 * only recovery command can be executed
	 */
	if (dsp_err_mmu_isset() && !recovery_flag) {
		print_mb_mmu_abort(mb);
		return -1;
	}

	if (down_interruptible(&mbsend_sem) < 0)
		return -1;

	if (arg) {	/* we have extra argument */
		int i;

		/*
		 * even if ipbuf_sys_ad is in DSP internal memory, 
		 * dsp_mem_enable() never cause to call PM mailbox command
		 * because in that case DSP memory should be always enabled.
		 * (see ipbuf_sys_hold_mem_active in ipbuf.c)
		 *
		 * Therefore, we can call this function here safely.
		 */
		dsp_mem_enable(ipbuf_sys_ad);
		if (sync_with_dsp(&ipbuf_sys_ad->s, OMAP_DSP_TID_FREE, 10) < 0) {
			printk(KERN_ERR "omapdsp: ipbuf_sys_ad is busy.\n");
			dsp_mem_disable(ipbuf_sys_ad);
			ret = -EBUSY;
			goto out;
		}
		for (i = 0; i < arg->argc; i++) {
			ipbuf_sys_ad->d[i] = arg->argv[i];
		}
		ipbuf_sys_ad->s = arg->tid;
		dsp_mem_disable(ipbuf_sys_ad);
	}

	ret = __mbcmd_send(mb);

out:
	up(&mbsend_sem);
	return ret;
}

int __dsp_mbcmd_send_and_wait(struct mbcmd *mb, struct mb_exarg *arg,
			      wait_queue_head_t *q)
{
	long current_state;
	DECLARE_WAITQUEUE(wait, current);

	add_wait_queue(q, &wait);
	current_state = current->state;
	set_current_state(TASK_INTERRUPTIBLE);
	if (dsp_mbcmd_send_exarg(mb, arg) < 0) {
		set_current_state(current_state);
		remove_wait_queue(q, &wait);
		return -1;
	}
	schedule_timeout(DSP_TIMEOUT);
	set_current_state(current_state);
	remove_wait_queue(q, &wait);

	return 0;
}

int __dsp_mbsend(unsigned char cmdh, unsigned char cmdl, unsigned short data,
		 int recovery_flag)
{
	struct mbcmd mb;

	mbcmd_set(mb, cmdh, cmdl, data);
	return __dsp_mbcmd_send(&mb, NULL, recovery_flag);
}

static int mbsync_hold_mem_active;

void dsp_mb_start(void)
{
	mbx1_valid = 1;	/* start interpreting */
	mbseq_expect_tmp = 0;
}

void dsp_mb_stop(void)
{
	mbx1_valid = 0;	/* stop interpreting */
	if (mbsync_hold_mem_active) {
		dsp_mem_disable((void *)daram_base);
		mbsync_hold_mem_active = 0;
	}
	mbseq = NULL;
	mbseq_expect = &mbseq_expect_tmp;
}

int dsp_mb_config(void *p)
{
	unsigned long flags;

	if (dsp_address_validate(p, sizeof(struct sync_seq), "mbseq") < 0)
		return -1;
	if (dsp_mem_type(p, sizeof(struct sync_seq)) != MEM_TYPE_EXTERN) {
		printk(KERN_WARNING
		       "omapdsp: mbseq is placed in DSP internal memory.\n"
		       "         It will prevent DSP from idling.\n");
		mbsync_hold_mem_active = 1;
		/*
		 * dsp_mem_enable() never fails because
		 * it has been already enabled in dspcfg process and
		 * this will just increment the usecount.
		 */
		dsp_mem_enable((void *)daram_base);
	}

	local_irq_save(flags);
	mbseq = p;
	mbseq->da_arm = mbseq_expect_tmp;
	mbseq_expect = &mbseq->da_arm;
	local_irq_restore(flags);

	return 0;
}

/*
 * mbq: mailbox queue
 */
#define MBQ_DEPTH	16
struct mbq {
	struct mbcmd mb[MBQ_DEPTH];
	int rp, wp, full;
} mbq = {
	.rp = 0,
	.wp = 0,
};

#define mbq_inc(p)	do { if (++(p) == MBQ_DEPTH) (p) = 0; } while(0)

/*
 * workqueue for mbx1
 */
static void do_mbx1(void)
{
	int empty = 0;

	disable_irq(INT_D2A_MB1);
	if ((mbq.rp == mbq.wp) && !mbq.full)
		empty = 1;
	enable_irq(INT_D2A_MB1);

	while (!empty) {
		struct mbcmd *mb;

		mb = &mbq.mb[mbq.rp];

		mblog_add(mb, DIR_D2A);
		mblog_printcmd(mb, DIR_D2A);

		/*
		 * call handler for each command
		 */
		if (cmdinfo[mb->cmd_h]->handler)
			cmdinfo[mb->cmd_h]->handler(mb);
		else if (cmdinfo[mb->cmd_h] != &cif_null)
			printk(KERN_ERR "mbx: %s is not allowed from DSP.\n",
			       cmd_name(*mb));
		else
			printk(KERN_ERR
			       "mbx: Unrecognized command: "
			       "cmd=0x%04x, data=0x%04x\n",
			       ((struct mbcmd_hw *)mb)->cmd & 0x7fff, mb->data);

		disable_irq(INT_D2A_MB1);
		mbq_inc(mbq.rp);
		if (mbq.rp == mbq.wp)
			empty = 1;
		/* if mbq has been full, now we have a room. */
		if (mbq.full) {
			mbq.full = 0;
			enable_irq(INT_D2A_MB1);
		}
		enable_irq(INT_D2A_MB1);
	}
}

static DECLARE_WORK(mbx1_work, (void (*)(void *))do_mbx1, NULL);

/*
 * kernel function dispatcher
 */
extern void mbx1_fbctl_disable(void);

static void mbx1_kfunc_fbctl(unsigned short data)
{
	switch (data) {
	case OMAP_DSP_MBCMD_FBCTL_DISABLE:
		mbx1_fbctl_disable();
		break;
	default:
		printk(KERN_ERR
		       "mailbox: Unknown FBCTL from DSP: 0x%04x\n", data);
	}
}

static void mbx1_kfunc(struct mbcmd *mb)
{
	switch (mb->cmd_l) {
	case OMAP_DSP_MBCMD_KFUNC_FBCTL:
		mbx1_kfunc_fbctl(mb->data);
		break;

	default:
		printk(KERN_ERR
		       "mailbox: Unknown kfunc from DSP: 0x%02x\n", mb->cmd_l);
	}
}

/*
 * mailbox interrupt handler
 */
static irqreturn_t mbx1_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	union {
		struct mbcmd sw;
		struct mbcmd_hw hw;
	} *mb = (void *)&mbq.mb[mbq.wp];

#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
	mb->hw.data = omap_readw(MAILBOX_DSP2ARM1);
	mb->hw.cmd  = omap_readw(MAILBOX_DSP2ARM1b);
#elif (INT_D2A_MB1 == INT_DSP_MAILBOX2)
	mb->hw.data = omap_readw(MAILBOX_DSP2ARM2);
	mb->hw.cmd  = omap_readw(MAILBOX_DSP2ARM2b);
#endif

	/* if mbx1 has not been validated yet, discard. */
	if (!mbx1_valid)
		return IRQ_HANDLED;

	if (mb->sw.seq != (*mbseq_expect & 1)) {
		switch (mbseq_check_level) {
		case MBSEQ_CHECK_NONE:
			break;
		case MBSEQ_CHECK_VERBOSE:
			printk(KERN_INFO
			       "mbx: illegal seq bit!!!  ignoring this command."
			       " (%04x:%04x)\n", mb->hw.cmd, mb->hw.data);
			return IRQ_HANDLED;
		case MBSEQ_CHECK_SILENT:
			return IRQ_HANDLED;
		}
	}

	(*mbseq_expect)++;

	mbq_inc(mbq.wp);
	if (mbq.wp == mbq.rp) {	/* mbq is full */
		mbq.full = 1;
		disable_irq(INT_D2A_MB1);
	}
	schedule_work(&mbx1_work);

	return IRQ_HANDLED;
}

static irqreturn_t mbx2_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned short cmd, data;

#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
	data = omap_readw(MAILBOX_DSP2ARM2);
	cmd  = omap_readw(MAILBOX_DSP2ARM2b);
#elif (INT_D2A_MB1 == INT_DSP_MAILBOX2)
	data = omap_readw(MAILBOX_DSP2ARM1);
	cmd  = omap_readw(MAILBOX_DSP2ARM1b);
#endif
	printk(KERN_DEBUG
	       "mailbox2 interrupt!  cmd=%04x, data=%04x\n", cmd, data);

	return IRQ_HANDLED;
}

#if 0
static void mpuio_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	printk(KERN_INFO "MPUIO interrupt!\n");
}
#endif

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *procdir_dsp = NULL;

static void dsp_create_procdir_dsp(void)
{
	procdir_dsp = proc_mkdir("dsp", 0);
	if (procdir_dsp == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc directory: dsp\n");
	}
}

static void dsp_remove_procdir_dsp(void)
{
	procdir_dsp = NULL;
	remove_proc_entry("dsp", 0);
}
#else /* CONFIG_PROC_FS */
#define dsp_create_procdir_dsp()	do { } while (0)
#define dsp_remove_procdir_dsp()	do { } while (0)
#endif /* CONFIG_PROC_FS */

extern irqreturn_t dsp_mmu_interrupt(int irq, void *dev_id,
				     struct pt_regs *regs);

extern int  dsp_ctl_core_init(void);
extern void dsp_ctl_core_exit(void);
extern void dsp_ctl_init(void);
extern void dsp_ctl_exit(void);
extern int  dsp_mem_init(void);
extern void dsp_mem_exit(void);
extern void mblog_init(void);
extern void mblog_exit(void);
extern int  dsp_taskmod_init(void);
extern void dsp_taskmod_exit(void);

/*
 * device functions
 */
static void dsp_dev_release(struct device *dev)
{
}

/*
 * driver functions
 */
#if (INT_D2A_MB1 == INT_DSP_MAILBOX1)
#	define INT_D2A_MB2 INT_DSP_MAILBOX2
#elif(INT_D2A_MB1 == INT_DSP_MAILBOX2)	/* swap MB1 and MB2 */
#	define INT_D2A_MB2 INT_DSP_MAILBOX1
#endif

static int __init dsp_drv_probe(struct platform_device *pdev)
{
	int ret;

	printk(KERN_INFO "OMAP DSP driver initialization\n");

	//__dsp_enable(); // XXX

	dsp_create_procdir_dsp();

	if ((ret = dsp_ctl_core_init()) < 0)
		goto fail1;
	if ((ret = dsp_mem_init()) < 0)
		goto fail2;
	dsp_ctl_init();
	mblog_init();
	if ((ret = dsp_taskmod_init()) < 0)
		goto fail3;

	/*
	 * mailbox interrupt handlers registration
	 */
	ret = request_irq(INT_D2A_MB1, mbx1_interrupt, SA_INTERRUPT, "dsp",
			  &pdev->dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register mailbox1 interrupt: %d\n", ret);
		goto fail4;
	}

	ret = request_irq(INT_D2A_MB2, mbx2_interrupt, SA_INTERRUPT, "dsp",
			  &pdev->dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register mailbox2 interrupt: %d\n", ret);
		goto fail5;
	}

	ret = request_irq(INT_DSP_MMU, dsp_mmu_interrupt, SA_INTERRUPT, "dsp",
			  &pdev->dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register DSP MMU interrupt: %d\n", ret);
		goto fail6;
	}

	/* MMU interrupt is not enabled until DSP runs */
	disable_irq(INT_DSP_MMU);

#if 0
	ret = request_irq(INT_MPUIO, mpuio_interrupt, SA_INTERRUPT, "dsp", dev);
	if (ret) {
		printk(KERN_ERR
		       "failed to register MPUIO interrupt: %d\n", ret);
		goto fail7;
	}
#endif

	return 0;

fail6:
	free_irq(INT_D2A_MB2, &pdev->dev);
fail5:
	free_irq(INT_D2A_MB1, &pdev->dev);
fail4:
	dsp_taskmod_exit();
fail3:
	mblog_exit();
	dsp_ctl_exit();
	dsp_mem_exit();
fail2:
	dsp_ctl_core_exit();
fail1:
	dsp_remove_procdir_dsp();

	//__dsp_disable(); // XXX
	return ret;
}

static int dsp_drv_remove(struct platform_device *pdev)
{
	dsp_cpustat_request(CPUSTAT_RESET);

#if 0
	free_irq(INT_MPUIO, dev);
#endif
	free_irq(INT_DSP_MMU, &pdev->dev);
	free_irq(INT_D2A_MB2, &pdev->dev);
	free_irq(INT_D2A_MB1, &pdev->dev);

	/* recover disable_depth */
	enable_irq(INT_DSP_MMU);

	dspuncfg();
	dsp_taskmod_exit();
	mblog_exit();
	dsp_ctl_exit();
	dsp_mem_exit();

	dsp_ctl_core_exit();
	dsp_remove_procdir_dsp();

	//__dsp_disable(); // XXX

	return 0;
}

#ifdef CONFIG_PM
static int dsp_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	dsp_suspend();

	return 0;
}

static int dsp_drv_resume(struct platform_device *pdev)
{
	dsp_resume();

	return 0;
}
#else
#define dsp_drv_suspend		NULL
#define dsp_drv_resume		NULL
#endif /* CONFIG_PM */

static struct resource dsp_resources[] = {
	{
		.start = INT_DSP_MAILBOX1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_DSP_MAILBOX2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_DSP_MMU,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device dsp_device = {
	.name		= "dsp",
	.id		= -1,
	.dev = {
		.release	= dsp_dev_release,
	},
	.num_resources	= ARRAY_SIZE(&dsp_resources),
	.resource	= dsp_resources,
};

static struct platform_driver dsp_driver = {
	.probe		= dsp_drv_probe,
	.remove		= dsp_drv_remove,
	.suspend	= dsp_drv_suspend,
	.resume		= dsp_drv_resume,
	.driver		= {
		.name	= "dsp",
	},
};

static int __init omap_dsp_mod_init(void)
{
	int ret;

	ret = platform_device_register(&dsp_device);
	if (ret) {
		printk(KERN_ERR "failed to register the DSP device: %d\n", ret);
		goto fail1;
	}

	ret = platform_driver_register(&dsp_driver);
	if (ret) {
		printk(KERN_ERR "failed to register the DSP driver: %d\n", ret);
		goto fail2;
	}

	return 0;

fail2:
	platform_device_unregister(&dsp_device);
fail1:
	return -ENODEV;
}

static void __exit omap_dsp_mod_exit(void)
{
	platform_driver_unregister(&dsp_driver);
	platform_device_unregister(&dsp_device);
}

module_init(omap_dsp_mod_init);
module_exit(omap_dsp_mod_exit);
