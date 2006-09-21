/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctls.h>
#include <asm/arch/mailbox.h>
#include "hardware_dsp.h"
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ipbuf.h"
#include "ioctl.h"

enum dsp_space_e {
	SPACE_MEM,
	SPACE_IO,
};

#ifdef CONFIG_OMAP_DSP_FBEXPORT
static enum fbstat_e {
	FBSTAT_DISABLED = 0,
	FBSTAT_ENABLED,
	FBSTAT_MAX,
} fbstat = FBSTAT_ENABLED;
#endif

static enum cfgstat_e cfgstat;
int mbox_revision;
static u8 n_stask;

static ssize_t ifver_show(struct device *dev, struct device_attribute *attr,
			  char *buf);
static ssize_t cpustat_show(struct device *dev, struct device_attribute *attr,
			    char *buf);
static ssize_t icrmask_show(struct device *dev, struct device_attribute *attr,
			    char *buf);
static ssize_t icrmask_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count);
static ssize_t loadinfo_show(struct device *dev, struct device_attribute *attr,
			     char *buf);

#define __ATTR_RW(_name, _mode) { \
	.attr = {.name = __stringify(_name), .mode = _mode, .owner = THIS_MODULE },	\
	.show	= _name##_show,					\
	.store	= _name##_store,					\
}

static struct device_attribute dev_attr_ifver     = __ATTR_RO(ifver);
static struct device_attribute dev_attr_cpustat   = __ATTR_RO(cpustat);
static struct device_attribute dev_attr_icrmask   = __ATTR_RW(icrmask, 0644);
static struct device_attribute dev_attr_loadinfo  = __ATTR_RO(loadinfo);

/*
 * misc interactive mailbox command operations
 */
static struct misc_mb_wait_struct {
	struct mutex lock;
	wait_queue_head_t wait_q;
	u8 cmd_h;
	u8 cmd_l;
	u16 *retvp;
} misc_mb_wait = {
	.lock = __MUTEX_INITIALIZER(misc_mb_wait.lock),
	.wait_q = __WAIT_QUEUE_HEAD_INITIALIZER(misc_mb_wait.wait_q),
};

static int __misc_mbcompose_send_and_wait(u8 cmd_h, u8 cmd_l, u16 data,
					  u16 *retvp)
{
	struct mbcmd mb = MBCMD_INIT(cmd_h, cmd_l, data);
	int ret = 0;

	if (mutex_lock_interruptible(&misc_mb_wait.lock))
		return -EINTR;

	misc_mb_wait.cmd_h = mb.cmd_h;
	misc_mb_wait.cmd_l = mb.cmd_l;
	misc_mb_wait.retvp = retvp;
	dsp_mbcmd_send_and_wait(&mb, &misc_mb_wait.wait_q);

	if (misc_mb_wait.cmd_h != 0)
		ret = -EINVAL;

	mutex_unlock(&misc_mb_wait.lock);
	return ret;
}

#define misc_mbcompose_send_and_wait(cmd_h, cmd_l, data, retvp) \
		__misc_mbcompose_send_and_wait(MBOX_CMD_DSP_##cmd_h, (cmd_l), \
					       (data), (retvp));

static int misc_mbcmd_response(struct mbcmd *mb, int argc, int match_cmd_l_flag)
{
	volatile u16 *buf;
	int i;

	/* if match_cmd_l_v flag is set, cmd_l needs to be matched as well. */
	if (!waitqueue_active(&misc_mb_wait.wait_q) ||
	    (misc_mb_wait.cmd_h != mb->cmd_h) ||
	    (match_cmd_l_flag && (misc_mb_wait.cmd_l != mb->cmd_l))) {
		const struct cmdinfo *ci = cmdinfo[mb->cmd_h];
		char cmdstr[32];

		if (ci->cmd_l_type == CMD_L_TYPE_SUBCMD)
			sprintf(cmdstr, "%s:%s", ci->name, subcmd_name(mb));
		else
			strcpy(cmdstr, ci->name);
		printk(KERN_WARNING
		       "mbox: unexpected command %s received!\n", cmdstr);
		return -1;
	}

	/*
	 * if argc == 1, receive data through mbox:data register.
	 * if argc > 1, receive through ipbuf_sys.
	 */
	if (argc == 1)
		misc_mb_wait.retvp[0] = mb->data;
	else if (argc > 1) {
		if (dsp_mem_enable(ipbuf_sys_da) < 0) {
			printk(KERN_ERR "mbox: %s - ipbuf_sys_da read failed!\n",
			       cmdinfo[mb->cmd_h]->name);
			return -1;
		}
		if (sync_with_dsp(&ipbuf_sys_da->s, TID_ANON, 10) < 0) {
			printk(KERN_ERR "mbox: %s - IPBUF sync failed!\n",
			       cmdinfo[mb->cmd_h]->name);
			dsp_mem_disable(ipbuf_sys_da);
			return -1;
		}
		/* need word access. do not use memcpy. */
		buf = ipbuf_sys_da->d;
		for (i = 0; i < argc; i++)
			misc_mb_wait.retvp[i] = buf[i];
		release_ipbuf_pvt(ipbuf_sys_da);
		dsp_mem_disable(ipbuf_sys_da);
	}

	misc_mb_wait.cmd_h = 0;
	wake_up_interruptible(&misc_mb_wait.wait_q);
	return 0;
}

static int dsp_regread(enum dsp_space_e space, u16 adr, u16 *val)
{
	u8 cmd_l = (space == SPACE_MEM) ? REGRW_MEMR : REGRW_IOR;
	int ret;

	ret = misc_mbcompose_send_and_wait(REGRW, cmd_l, adr, val);
	if ((ret < 0) && (ret != -EINTR))
		printk(KERN_ERR "omapdsp: register read error!\n");

	return ret;
}

static int dsp_regwrite(enum dsp_space_e space, u16 adr, u16 val)
{
	u8 cmd_l = (space == SPACE_MEM) ? REGRW_MEMW : REGRW_IOW;
	struct mb_exarg arg = {
		.tid  = TID_ANON,
		.argc = 1,
		.argv = &val,
	};

	mbcompose_send_exarg(REGRW, cmd_l, adr, &arg);
	return 0;
}

static int dsp_getvar(u8 varid, u16 *val)
{
	int ret;

	ret = misc_mbcompose_send_and_wait(GETVAR, varid, 0, val);
	if ((ret < 0) && (ret != -EINTR))
		printk(KERN_ERR "omapdsp: variable read error!\n");

	return ret;
}

static int dsp_setvar(u8 varid, u16 val)
{
	mbcompose_send(SETVAR, varid, val);
	return 0;
}

/*
 * dsp_cfg() return value
 *  = 0: OK
 *  = 1: failed, but state is clear. (DSPCFG command failed)
 *  < 0: failed. need cleanup.
 */
static int dsp_cfg(void)
{
	int ret = 0;

#ifdef CONFIG_ARCH_OMAP1
	/* for safety */
	dsp_mem_usecount_clear();
#endif

	/*
	 * DSPCFG command and dsp_mem_start() must be called
	 * while internal mem is on.
	 */
	dsp_mem_enable((void *)dspmem_base);

	dsp_mbox_start();
	dsp_twch_start();
	dsp_mem_start();
	dsp_err_start();

	mbox_revision = -1;

	ret = misc_mbcompose_send_and_wait(DSPCFG, DSPCFG_REQ, 0, NULL);
	if (ret < 0) {
		if (ret != -EINTR)
			printk(KERN_ERR "omapdsp: configuration error!\n");
		ret = 1;
		goto out;
	}

#if defined(CONFIG_ARCH_OMAP1) && defined(OLD_BINARY_SUPPORT)
	/*
	 * MBREV 3.2 or earlier doesn't assume DMA domain is on
	 * when DSPCFG command is sent
	 */
	if ((mbox_revision == MBREV_3_0) ||
	    (mbox_revision == MBREV_3_2)) {
		if ((ret = mbcompose_send(PM, PM_ENABLE, DSPREG_ICR_DMA)) < 0)
			goto out;
	}
#endif

	if ((ret = dsp_task_config_all(n_stask)) < 0)
		goto out;

	/* initialization */
#ifdef CONFIG_OMAP_DSP_FBEXPORT
	fbstat = FBSTAT_ENABLED;
#endif

	/* send parameter */
	if ((ret = dsp_setvar(VARID_ICRMASK, dsp_cpustat_get_icrmask())) < 0)
		goto out;

	/* create runtime sysfs entries */
	device_create_file(omap_dsp->dev, &dev_attr_loadinfo);

out:
	dsp_mem_disable((void *)dspmem_base);
	return ret;
}

static int dsp_uncfg(void)
{
	if (dsp_taskmod_busy()) {
		printk(KERN_WARNING "omapdsp: tasks are busy.\n");
		return -EBUSY;
	}

	/* FIXME: lock task module */

	/* remove runtime sysfs entries */
	device_remove_file(omap_dsp->dev, &dev_attr_loadinfo);

	dsp_mbox_stop();
	dsp_twch_stop();
	dsp_mem_stop();
	dsp_err_stop();
	dsp_dbg_stop();
	dsp_task_unconfig_all();
	ipbuf_stop();

	return 0;
}

static int dsp_suspend(void)
{
	int ret;

	ret = misc_mbcompose_send_and_wait(SUSPEND, 0, 0, NULL);
	if (ret < 0) {
		if (ret != -EINVAL)
			printk(KERN_ERR "omapdsp: DSP suspend error!\n");
		return ret;
	}

	udelay(100);	/* wait for DSP-side execution */
	return 0;
}

int dsp_cfgstat_request(enum cfgstat_e st_req)
{
	static DEFINE_MUTEX(cfgstat_lock);
	int ret = 0, ret_override = 0;

	if (mutex_lock_interruptible(&cfgstat_lock))
		return -EINTR;

again:
	switch (st_req) {

	/* cfgstat takes CLEAN, READY or SUSPEND,
	   while st_req can take SUSPEND in addition. */

	case CFGSTAT_CLEAN:
		if (cfgstat == CFGSTAT_CLEAN)
			goto up_out;
		if ((ret = dsp_uncfg()) < 0)
			goto up_out;
		break;

	case CFGSTAT_READY:
		if (cfgstat != CFGSTAT_CLEAN) {
			printk(KERN_ERR "omapdsp: DSP is ready already!\n");
			ret = -EINVAL;
			goto up_out;
		}

		ret = dsp_cfg();
		if (ret > 0) {	/* failed, but state is clear. */
			ret = -EINVAL;
			goto up_out;
		} else if (ret < 0) {	/* failed, need cleanup. */
			st_req = CFGSTAT_CLEAN;
			ret_override = ret;
			goto again;
		}
		break;

	/*
	 * suspend / resume
	 * DSP is not reset within this code, but done in omap_pm_suspend.
	 * so if these functions are called from sysfs,
	 * DSP should be reset / unreset out of these functions.
	 */
	case CFGSTAT_SUSPEND:
		switch (cfgstat) {

		case CFGSTAT_CLEAN:
			if (dsp_cpustat_get_stat() == CPUSTAT_RUN) {
				printk(KERN_WARNING
				       "omapdsp: illegal operation -- trying "
				       "suspend DSP while it is running but "
				       "not configured.\n"
				       "  Resetting DSP.\n");
				dsp_cpustat_request(CPUSTAT_RESET);
				ret = -EINVAL;
			}
			goto up_out;

		case CFGSTAT_READY:
			if ((ret = dsp_suspend()) < 0)
				goto up_out;
			break;

		case CFGSTAT_SUSPEND:
			goto up_out;

		default:
			BUG();

		}

		break;

	case CFGSTAT_RESUME:
		if (cfgstat != CFGSTAT_SUSPEND) {
			printk(KERN_WARNING
			       "omapdsp: DSP resume request, but DSP is not in "
			       "suspend state.\n");
			ret = -EINVAL;
			goto up_out;
		}
		st_req = CFGSTAT_READY;
		break;

	default:
		BUG();

	}

	cfgstat = st_req;
up_out:
	mutex_unlock(&cfgstat_lock);
	return ret_override ? ret_override : ret;
}

enum cfgstat_e dsp_cfgstat_get_stat(void)
{
	return cfgstat;
}

/*
 * polls all tasks
 */
static int dsp_poll(void)
{
	int ret;

	ret = misc_mbcompose_send_and_wait(POLL, 0, 0, NULL);
	if ((ret < 0) && (ret != -EINTR))
		printk(KERN_ERR "omapdsp: poll error!\n");

	return ret;
}

int dsp_set_runlevel(u8 level)
{
	if (level == RUNLEVEL_RECOVERY) {
		if (mbcompose_send_recovery(RUNLEVEL, level, 0) < 0)
			return -EINVAL;
	} else {
		if ((level < RUNLEVEL_USER) ||
		    (level > RUNLEVEL_SUPER))
			return -EINVAL;
		if (mbcompose_send(RUNLEVEL, level, 0) < 0)
			return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_OMAP_DSP_FBEXPORT
static void dsp_fbctl_enable(void)
{
	mbcompose_send(KFUNC, KFUNC_FBCTL, FBCTL_ENABLE);
}

static int dsp_fbctl_disable(void)
{
	int ret;

	ret = misc_mbcompose_send_and_wait(KFUNC, KFUNC_FBCTL, FBCTL_DISABLE,
					   NULL);
	if ((ret < 0) && (ret != -EINTR))
		printk(KERN_ERR "omapdsp: fb disable error!\n");

	return 0;
}

static int dsp_fbstat_request(enum fbstat_e st)
{
	static DEFINE_MUTEX(fbstat_lock);
	int ret = 0;

	if (mutex_lock_interruptible(&fbstat_lock))
		return -EINTR;

	if (st == fbstat)
		goto up_out;

	switch (st) {
	case FBSTAT_ENABLED:
		dsp_fbctl_enable();
		break;
	case FBSTAT_DISABLED:
		if ((ret = dsp_fbctl_disable()) < 0)
			goto up_out;
		break;
	default:
		BUG();
	}

	fbstat = st;
up_out:
	mutex_unlock(&fbstat_lock);
	return 0;
}
#endif /* CONFIG_OMAP_DSP_FBEXPORT */

/*
 * DSP control device file operations
 */
static int dsp_ctl_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	/*
	 * command level 1: commands which don't need lock
	 */
	case DSPCTL_IOCTL_RUN:
		dsp_cpustat_request(CPUSTAT_RUN);
		break;

	case DSPCTL_IOCTL_RESET:
		dsp_cpustat_request(CPUSTAT_RESET);
		break;

	case DSPCTL_IOCTL_SETRSTVECT:
		ret = dsp_set_rstvect((dsp_long_t)arg);
		break;

#ifdef CONFIG_ARCH_OMAP1
	case DSPCTL_IOCTL_CPU_IDLE:
		dsp_cpustat_request(CPUSTAT_CPU_IDLE);
		break;

	case DSPCTL_IOCTL_GBL_IDLE:
		dsp_cpustat_request(CPUSTAT_GBL_IDLE);
		break;

	case DSPCTL_IOCTL_MPUI_WORDSWAP_ON:
		mpui_wordswap_on();
		break;

	case DSPCTL_IOCTL_MPUI_WORDSWAP_OFF:
		mpui_wordswap_off();
		break;

	case DSPCTL_IOCTL_MPUI_BYTESWAP_ON:
		mpui_byteswap_on();
		break;

	case DSPCTL_IOCTL_MPUI_BYTESWAP_OFF:
		mpui_byteswap_off();
		break;
#endif /* CONFIG_ARCH_OMAP1 */

	case DSPCTL_IOCTL_TASKCNT:
		ret = dsp_task_count();
		break;

	case DSPCTL_IOCTL_MBSEND:
		{
			struct omap_dsp_mailbox_cmd u_cmd;
			mbox_msg_t msg;
			if (copy_from_user(&u_cmd, (void *)arg, sizeof(u_cmd)))
				return -EFAULT;
			msg = (u_cmd.cmd << 16) | u_cmd.data;
			ret = dsp_mbcmd_send((struct mbcmd *)&msg);
			break;
		}

	case DSPCTL_IOCTL_SETVAR:
		{
			struct omap_dsp_varinfo var;
			if (copy_from_user(&var, (void *)arg, sizeof(var)))
				return -EFAULT;
			ret = dsp_setvar(var.varid, var.val[0]);
			break;
		}

	case DSPCTL_IOCTL_RUNLEVEL:
		ret = dsp_set_runlevel(arg);
		break;

#ifdef CONFIG_OMAP_DSP_FBEXPORT
	case DSPCTL_IOCTL_FBEN:
		ret = dsp_fbstat_request(FBSTAT_ENABLED);
		break;
#endif

	/*
	 * command level 2: commands which need lock
	 */
	case DSPCTL_IOCTL_DSPCFG:
		ret = dsp_cfgstat_request(CFGSTAT_READY);
		break;

	case DSPCTL_IOCTL_DSPUNCFG:
		ret = dsp_cfgstat_request(CFGSTAT_CLEAN);
		break;

	case DSPCTL_IOCTL_POLL:
		ret = dsp_poll();
		break;

#ifdef CONFIG_OMAP_DSP_FBEXPORT
	case DSPCTL_IOCTL_FBDIS:
		ret = dsp_fbstat_request(FBSTAT_DISABLED);
		break;
#endif

	case DSPCTL_IOCTL_SUSPEND:
		if ((ret = dsp_cfgstat_request(CFGSTAT_SUSPEND)) < 0)
			break;
		dsp_cpustat_request(CPUSTAT_RESET);
		break;

	case DSPCTL_IOCTL_RESUME:
		if ((ret = dsp_cfgstat_request(CFGSTAT_RESUME)) < 0)
			break;
		dsp_cpustat_request(CPUSTAT_RUN);
		break;

	case DSPCTL_IOCTL_REGMEMR:
		{
			struct omap_dsp_reginfo *u_reg = (void *)arg;
			u16 adr, val;

			if (copy_from_user(&adr, &u_reg->adr, sizeof(u16)))
				return -EFAULT;
			if ((ret = dsp_regread(SPACE_MEM, adr, &val)) < 0)
				return ret;
			if (copy_to_user(&u_reg->val, &val, sizeof(u16)))
				return -EFAULT;
			break;
		}

	case DSPCTL_IOCTL_REGMEMW:
		{
			struct omap_dsp_reginfo reg;

			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
				return -EFAULT;
			ret = dsp_regwrite(SPACE_MEM, reg.adr, reg.val);
			break;
		}

	case DSPCTL_IOCTL_REGIOR:
		{
			struct omap_dsp_reginfo *u_reg = (void *)arg;
			u16 adr, val;

			if (copy_from_user(&adr, &u_reg->adr, sizeof(u16)))
				return -EFAULT;
			if ((ret = dsp_regread(SPACE_IO, adr, &val)) < 0)
				return ret;
			if (copy_to_user(&u_reg->val, &val, sizeof(u16)))
				return -EFAULT;
			break;
		}

	case DSPCTL_IOCTL_REGIOW:
		{
			struct omap_dsp_reginfo reg;

			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
				return -EFAULT;
			ret = dsp_regwrite(SPACE_IO, reg.adr, reg.val);
			break;
		}

	case DSPCTL_IOCTL_GETVAR:
		{
			struct omap_dsp_varinfo *u_var = (void *)arg;
			u8 varid;
			u16 val[5]; /* maximum */
			int argc;

			if (copy_from_user(&varid, &u_var->varid, sizeof(u8)))
				return -EFAULT;
			switch (varid) {
			case VARID_ICRMASK:
				argc = 1;
				break;
			case VARID_LOADINFO:
				argc = 5;
				break;
			default:
				return -EINVAL;
			}
			if ((ret = dsp_getvar(varid, val)) < 0)
				return ret;
			if (copy_to_user(&u_var->val, val, sizeof(u16) * argc))
				return -EFAULT;
			break;
		}

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

/*
 * functions called from mailbox interrupt routine
 */
void mbox_suspend(struct mbcmd *mb)
{
	misc_mbcmd_response(mb, 0, 0);
}

void mbox_dspcfg(struct mbcmd *mb)
{
	u8 last   = mb->cmd_l & 0x80;
	u8 cfgcmd = mb->cmd_l & 0x7f;
	static dsp_long_t tmp_ipb_adr;

	if (!waitqueue_active(&misc_mb_wait.wait_q) ||
	    (misc_mb_wait.cmd_h != MBOX_CMD_DSP_DSPCFG)) {
		printk(KERN_WARNING
		       "mbox: DSPCFG command received, "
		       "but nobody is waiting for it...\n");
		return;
	}

	/* mailbox protocol check */
	if (cfgcmd == DSPCFG_PROTREV) {
		mbox_revision = mb->data;
		if (mbox_revision == MBPROT_REVISION)
			return;
#ifdef OLD_BINARY_SUPPORT
		else if ((mbox_revision == MBREV_3_0) ||
			 (mbox_revision == MBREV_3_2)) {
			printk(KERN_WARNING
			       "mbox: ***** old DSP binary *****\n"
			       "  Please update your DSP application.\n");
			return;
		}
#endif
		else {
			printk(KERN_ERR
			       "mbox: protocol revision check error!\n"
			       "  expected=0x%04x, received=0x%04x\n",
			       MBPROT_REVISION, mb->data);
			mbox_revision = -1;
			goto abort1;
		}
	}

	/*
	 * following commands are accepted only after
	 * revision check has been passed.
	 */
	if (!mbox_revision < 0) {
		printk(KERN_INFO
		       "mbox: DSPCFG command received, "
		       "but revision check has not been passed.\n");
		return;
	}

	switch (cfgcmd) {
	case DSPCFG_SYSADRH:
		tmp_ipb_adr = (u32)mb->data << 16;
		break;

	case DSPCFG_SYSADRL:
		tmp_ipb_adr |= mb->data;
		break;

	case DSPCFG_ABORT:
		goto abort1;

	default:
		printk(KERN_ERR
		       "mbox: Unknown CFG command: cmd_l=0x%02x, data=0x%04x\n",
		       mb->cmd_l, mb->data);
		return;
	}

	if (last) {
		void *badr;
		u16 bln;
		u16 bsz;
		volatile u16 *buf;
		void *ipb_sys_da, *ipb_sys_ad;
		void *mbseq;	 /* FIXME: 3.4 obsolete */
		short *dbg_buf;
		u16 dbg_buf_sz, dbg_line_sz;
		struct mem_sync_struct mem_sync, *mem_syncp;

		ipb_sys_da = dspword_to_virt(tmp_ipb_adr);
		if (ipbuf_sys_config(ipb_sys_da, DIR_D2A) < 0)
			goto abort1;

		if (dsp_mem_enable(ipbuf_sys_da) < 0) {
			printk(KERN_ERR "mbox: DSPCFG - ipbuf_sys_da read failed!\n");
			goto abort1;
		}
		if (sync_with_dsp(&ipbuf_sys_da->s, TID_ANON, 10) < 0) {
			printk(KERN_ERR "mbox: DSPCFG - IPBUF sync failed!\n");
			dsp_mem_disable(ipbuf_sys_da);
			goto abort1;
		}
		/*
		 * read configuration data on system IPBUF
		 * we must read with 16bit-access
		 */
#ifdef OLD_BINARY_SUPPORT
		if (mbox_revision == MBPROT_REVISION) {
#endif
			buf = ipbuf_sys_da->d;
			n_stask        = buf[0];
			bln            = buf[1];
			bsz            = buf[2];
			badr           = MKVIRT(buf[3], buf[4]);
			/* ipb_sys_da     = MKVIRT(buf[5], buf[6]); */
			ipb_sys_ad     = MKVIRT(buf[7], buf[8]);
			mbseq          = MKVIRT(buf[9], buf[10]);
			dbg_buf        = MKVIRT(buf[11], buf[12]);
			dbg_buf_sz     = buf[13];
			dbg_line_sz    = buf[14];
			mem_sync.DARAM = MKVIRT(buf[15], buf[16]);
			mem_sync.SARAM = MKVIRT(buf[17], buf[18]);
			mem_sync.SDRAM = MKVIRT(buf[19], buf[20]);
			mem_syncp = &mem_sync;
#ifdef OLD_BINARY_SUPPORT
		} else if (mbox_revision == MBREV_3_2) {
			buf = ipbuf_sys_da->d;
			n_stask     = buf[0];
			bln         = buf[1];
			bsz         = buf[2];
			badr        = MKVIRT(buf[3], buf[4]);
			/* ipb_sys_da  = MKVIRT(buf[5], buf[6]); */
			ipb_sys_ad  = MKVIRT(buf[7], buf[8]);
			mbseq       = MKVIRT(buf[9], buf[10]);
			dbg_buf     = NULL;
			dbg_buf_sz  = 0;
			dbg_line_sz = 0;
			mem_syncp   = NULL;
		} else if (mbox_revision == MBREV_3_0) {
			buf = ipbuf_sys_da->d;
			n_stask     = buf[0];
			bln         = buf[1];
			bsz         = buf[2];
			badr        = MKVIRT(buf[3], buf[4]);
			/* bkeep       = buf[5]; */
			/* ipb_sys_da  = MKVIRT(buf[6], buf[7]); */
			ipb_sys_ad  = MKVIRT(buf[8], buf[9]);
			mbseq       = MKVIRT(buf[10], buf[11]);
			dbg_buf     = NULL;
			dbg_buf_sz  = 0;
			dbg_line_sz = 0;
			mem_syncp   = NULL;
		} else { /* should not occur */
			dsp_mem_disable(ipbuf_sys_da);
			goto abort1;
		}
#endif /* OLD_BINARY_SUPPORT */

		release_ipbuf_pvt(ipbuf_sys_da);
		dsp_mem_disable(ipbuf_sys_da);

		/*
		 * following configurations need to be done before
		 * waking up the dspcfg initiator process.
		 */
		if (ipbuf_sys_config(ipb_sys_ad, DIR_A2D) < 0)
			goto abort1;
		if (ipbuf_config(bln, bsz, badr) < 0)
			goto abort1;
		if (dsp_mbox_config(mbseq) < 0)
			goto abort2;
		if (dsp_dbg_config(dbg_buf, dbg_buf_sz, dbg_line_sz) < 0)
			goto abort2;
		if (dsp_mem_sync_config(mem_syncp) < 0)
			goto abort2;

		misc_mb_wait.cmd_h = 0;
		wake_up_interruptible(&misc_mb_wait.wait_q);
	}
	return;

abort2:
	ipbuf_stop();
abort1:
	wake_up_interruptible(&misc_mb_wait.wait_q);
	return;
}

void mbox_poll(struct mbcmd *mb)
{
	misc_mbcmd_response(mb, 0, 0);
}

void mbox_regrw(struct mbcmd *mb)
{
	switch (mb->cmd_l) {
	case REGRW_DATA:
		misc_mbcmd_response(mb, 1, 0);
		break;
	default:
		printk(KERN_ERR
		       "mbox: Illegal REGRW command: "
		       "cmd_l=0x%02x, data=0x%04x\n", mb->cmd_l, mb->data);
		return;
	}
}

void mbox_getvar(struct mbcmd *mb)
{
	switch (mb->cmd_l) {
	case VARID_ICRMASK:
		misc_mbcmd_response(mb, 1, 1);
		break;
	case VARID_LOADINFO:
		misc_mbcmd_response(mb, 5, 1);
		break;
	default:
		printk(KERN_ERR
		       "mbox: Illegal GETVAR command: "
		       "cmd_l=0x%02x, data=0x%04x\n", mb->cmd_l, mb->data);
		return;
	}
}

void mbox_fbctl_disable(struct mbcmd *mb)
{
	misc_mbcmd_response(mb, 0, 0);
}

struct file_operations dsp_ctl_fops = {
	.owner   = THIS_MODULE,
	.ioctl   = dsp_ctl_ioctl,
};

/*
 * sysfs files
 */

/* ifver */
static ssize_t ifver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len = 0;

	/*
	 * I/F VERSION descriptions:
	 *
	 * 3.2: sysfs / udev support
	 *      KMEM_RESERVE / KMEM_RELEASE ioctls for mem device
	 * 3.3: added following ioctls
	 *      DSPCTL_IOCTL_GBL_IDLE
	 *      DSPCTL_IOCTL_CPU_IDLE (instead of DSPCTL_IOCTL_IDLE)
	 *      DSPCTL_IOCTL_POLL
	 */

	/*
	 * print all supporting I/F VERSIONs, like followings.
	 *
	 * len += sprintf(buf, "3.2\n");
	 * len += sprintf(buf, "3.3\n");
	 */
	len += sprintf(buf + len, "3.2\n");
	len += sprintf(buf + len, "3.3\n");

	return len;
}

/* cpustat */
static char *cpustat_name[CPUSTAT_MAX] = {
	[CPUSTAT_RESET]    = "reset",
#ifdef CONFIG_ARCH_OMAP1
	[CPUSTAT_GBL_IDLE] = "gbl_idle",
	[CPUSTAT_CPU_IDLE] = "cpu_idle",
#endif
	[CPUSTAT_RUN]      = "run",
};

static ssize_t cpustat_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%s\n", cpustat_name[dsp_cpustat_get_stat()]);
}

/* icrmask */
static ssize_t icrmask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "0x%04x\n", dsp_cpustat_get_icrmask());
}

static ssize_t icrmask_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u16 mask;
	int ret;

	mask = simple_strtol(buf, NULL, 16);
	dsp_cpustat_set_icrmask(mask);

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY) {
		ret = dsp_setvar(VARID_ICRMASK, mask);
		if (ret < 0)
			return ret;
	}

	return count;
}

/* loadinfo */
static ssize_t loadinfo_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int len;
	int ret;
	u16 val[5];

	if ((ret = dsp_getvar(VARID_LOADINFO, val)) < 0)
		return ret;

	/*
	 * load info value range is 0(free) - 10000(busy):
	 * if CPU load is not measured on DSP, it sets 0xffff at val[0].
	 */

	if (val[0] == 0xffff) {
		len = sprintf(buf,
			      "currently DSP load info is not available.\n");
		goto out;
	}

	len = sprintf(buf,
		      "DSP load info:\n"
		      "  10ms average = %3d.%02d%%\n"
		      "  1sec average = %3d.%02d%%  busiest 10ms = %3d.%02d%%\n"
		      "  1min average = %3d.%02d%%  busiest 1s   = %3d.%02d%%\n",
		      val[0]/100, val[0]%100,
		      val[1]/100, val[1]%100, val[2]/100, val[2]%100,
		      val[3]/100, val[3]%100, val[4]/100, val[4]%100);
out:
	return len;
}

void __init dsp_ctl_init(void)
{
	device_create_file(omap_dsp->dev, &dev_attr_ifver);
	device_create_file(omap_dsp->dev, &dev_attr_cpustat);
	device_create_file(omap_dsp->dev, &dev_attr_icrmask);
}

void dsp_ctl_exit(void)
{
	device_remove_file(omap_dsp->dev, &dev_attr_ifver);
	device_remove_file(omap_dsp->dev, &dev_attr_cpustat);
	device_remove_file(omap_dsp->dev, &dev_attr_icrmask);
}
