/*
 * linux/arch/arm/mach-omap/dsp/dsp_ctl.c
 *
 * OMAP DSP control device driver
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
 * 2005/02/17:  DSP Gateway version 3.2
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctls.h>
#include <asm/hardware/clock.h>
#include <asm/arch/dsp.h>
#include "hardware_dsp.h"
#include "dsp.h"
#include "ipbuf.h"

static ssize_t loadinfo_show(struct device *dev, struct device_attribute *attr,
			     char *buf);
static struct device_attribute dev_attr_loadinfo = __ATTR_RO(loadinfo);
extern struct device_attribute dev_attr_ipbuf;

static enum {
	CFG_ERR,
	CFG_READY,
	CFG_SUSPEND
} cfgstat;
static int mbx_revision;
static DECLARE_WAIT_QUEUE_HEAD(ioctl_wait_q);
static unsigned short ioctl_wait_cmd;
static DECLARE_MUTEX(ioctl_sem);

static unsigned char n_stask;

/*
 * control functions
 */
static void dsp_run(void)
{
	disable_irq(INT_DSP_MMU);
	preempt_disable();
	if (dsp_runstat == RUNSTAT_RESET) {
		clk_use(api_ck_handle);
		__dsp_run();
		dsp_runstat = RUNSTAT_RUN;
	}
	preempt_enable();
	enable_irq(INT_DSP_MMU);
}

static void dsp_reset(void)
{
	disable_irq(INT_DSP_MMU);
	preempt_disable();
	if (dsp_runstat > RUNSTAT_RESET) {
		__dsp_reset();
		if (dsp_runstat == RUNSTAT_RUN)
			clk_unuse(api_ck_handle);
		dsp_runstat = RUNSTAT_RESET;
	}
	preempt_enable();
	enable_irq(INT_DSP_MMU);
}

static short varread_val[5]; /* maximum */

static int dsp_regread(unsigned short cmd_l, unsigned short adr,
		       unsigned short *val)
{
	struct mbcmd mb;
	int ret = 0;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	ioctl_wait_cmd = MBCMD(REGRW);
	mbcmd_set(mb, MBCMD(REGRW), cmd_l, adr);
	dsp_mbsend_and_wait(&mb, &ioctl_wait_q);

	if (ioctl_wait_cmd != 0) {
		printk(KERN_ERR "omapdsp: register read error!\n");
		ret = -EINVAL;
		goto up_out;
	}

	*val = varread_val[0];

up_out:
	up(&ioctl_sem);
	return ret;
}

static int dsp_regwrite(unsigned short cmd_l, unsigned short adr,
			unsigned short val)
{
	struct mbcmd mb;
	struct mb_exarg arg = {
		.tid  = OMAP_DSP_TID_ANON,
		.argc = 1,
		.argv = &val,
	};

	mbcmd_set(mb, MBCMD(REGRW), cmd_l, adr);
	dsp_mbsend_exarg(&mb, &arg);
	return 0;
}

static int dsp_getvar(unsigned char varid, unsigned short *val, int sz)
{
	struct mbcmd mb;
	int ret = 0;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	ioctl_wait_cmd = MBCMD(GETVAR);
	mbcmd_set(mb, MBCMD(GETVAR), varid, 0);
	dsp_mbsend_and_wait(&mb, &ioctl_wait_q);

	if (ioctl_wait_cmd != 0) {
		printk(KERN_ERR "omapdsp: variable read error!\n");
		ret = -EINVAL;
		goto up_out;
	}

	memcpy(val, varread_val, sz * sizeof(short));

up_out:
	up(&ioctl_sem);
	return ret;
}

static int dsp_setvar(unsigned char varid, unsigned short val)
{
	struct mbcmd mb;

	mbcmd_set(mb, MBCMD(SETVAR), varid, val);
	dsp_mbsend(&mb);
	return 0;
}

static int dspcfg(void)
{
	struct mbcmd mb;
	int ret = 0;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	if (cfgstat != CFG_ERR) {
		printk(KERN_ERR
		       "omapdsp: DSP has been already configured. "
		       "do unconfig!\n");
		ret = -EBUSY;
		goto up_out;
	}

	dsp_mb_start();
	dsp_twch_start();
	dsp_mem_start();
	dsp_err_start();

	mbx_revision = -1;
	ioctl_wait_cmd = MBCMD(DSPCFG);
	mbcmd_set(mb, MBCMD(DSPCFG), OMAP_DSP_MBCMD_DSPCFG_REQ, 0);
	dsp_mbsend_and_wait(&mb, &ioctl_wait_q);

	if (ioctl_wait_cmd != 0) {
		printk(KERN_ERR "omapdsp: configuration error!\n");
		ret = -EINVAL;
		cfgstat = CFG_ERR;
		goto up_out;
	}

	if ((ret = dsp_task_config_all(n_stask)) < 0) {
		up(&ioctl_sem);
		dspuncfg();
		return -EINVAL;
	}

	cfgstat = CFG_READY;

	/* send parameter */
	if ((ret = dsp_setvar(OMAP_DSP_MBCMD_VARID_ICRMASK, dsp_icrmask)) < 0)
		goto up_out;

	/* create runtime sysfs entries */
	device_create_file(&dsp_device.dev, &dev_attr_loadinfo);
	device_create_file(&dsp_device.dev, &dev_attr_ipbuf);

up_out:
	up(&ioctl_sem);
	return ret;
}

int dspuncfg(void)
{
	if (dsp_taskmod_busy()) {
		printk(KERN_WARNING "omapdsp: tasks are busy.\n");
		return -EBUSY;
	}

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;
	
	/* FIXME: lock task module */

	/* remove runtime sysfs entries */
	device_remove_file(&dsp_device.dev, &dev_attr_loadinfo);
	device_remove_file(&dsp_device.dev, &dev_attr_ipbuf);

	dsp_mb_stop();
	dsp_twch_stop();
	dsp_err_stop();
	dsp_task_unconfig_all();
	ipbuf_stop();
	cfgstat = CFG_ERR;

	up(&ioctl_sem);
	return 0;
}

int dsp_is_ready(void)
{
	return (cfgstat == CFG_READY) ? 1 : 0;
}

void dsp_runlevel(unsigned char level)
{
	struct mbcmd mb;

	mbcmd_set(mb, MBCMD(RUNLEVEL), level, 0);
	if (level == OMAP_DSP_MBCMD_RUNLEVEL_RECOVERY)
		dsp_mbsend_recovery(&mb);
	else
		dsp_mbsend(&mb);
}

int dsp_suspend(void)
{
	struct mbcmd mb;
	int ret = 0;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	if (!dsp_is_ready()) {
		ret = -EINVAL;
		goto up_out;
	}

	ioctl_wait_cmd = MBCMD(SUSPEND);
	mbcmd_set(mb, MBCMD(SUSPEND), 0, 0);
	dsp_mbsend_and_wait(&mb, &ioctl_wait_q);

	if (ioctl_wait_cmd != 0) {
		printk(KERN_ERR "omapdsp: DSP suspend error!\n");
		ret = -EINVAL;
		goto up_out;
	}

	udelay(100);
	cfgstat = CFG_SUSPEND;
up_out:
	up(&ioctl_sem);
	return ret;
}

int dsp_resume(void)
{
	if (cfgstat != CFG_SUSPEND)
		return 0;

	cfgstat = CFG_READY;
	return 0;
}

static void dsp_fbctl_enable(void)
{
#ifdef CONFIG_FB_OMAP_EXTERNAL_LCDC
	struct mbcmd mb;

	mbcmd_set(mb, MBCMD(KFUNC), OMAP_DSP_MBCMD_KFUNC_FBCTL,
		  OMAP_DSP_MBCMD_FBCTL_ENABLE);
	dsp_mbsend(&mb);
#endif
}

static int dsp_fbctl_disable(void)
{
	int ret = 0;

#ifdef CONFIG_FB_OMAP_EXTERNAL_LCDC
	struct mbcmd mb;

	if (down_interruptible(&ioctl_sem))
		return -ERESTARTSYS;

	ioctl_wait_cmd = MBCMD(KFUNC);
	mbcmd_set(mb, MBCMD(KFUNC), OMAP_DSP_MBCMD_KFUNC_FBCTL,
		  OMAP_DSP_MBCMD_FBCTL_DISABLE);
	dsp_mbsend_and_wait(&mb, &ioctl_wait_q);
	if (ioctl_wait_cmd != 0) {
		printk(KERN_ERR "omapdsp: fb disable error!\n");
		ret = -EINVAL;
	}
	up(&ioctl_sem);
#endif

	return ret;
}

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
	case OMAP_DSP_IOCTL_RUN:
		dsp_run();
		break;

	case OMAP_DSP_IOCTL_RESET:
		dsp_reset();
		break;

	case OMAP_DSP_IOCTL_SETRSTVECT:
		ret = dsp_set_rstvect((unsigned long)arg);
		break;

	case OMAP_DSP_IOCTL_IDLE:
		dsp_idle();
		break;

	case OMAP_DSP_IOCTL_MPUI_WORDSWAP_ON:
		mpui_wordswap_on();
		break;

	case OMAP_DSP_IOCTL_MPUI_WORDSWAP_OFF:
		mpui_wordswap_off();
		break;

	case OMAP_DSP_IOCTL_MPUI_BYTESWAP_ON:
		mpui_byteswap_on();
		break;

	case OMAP_DSP_IOCTL_MPUI_BYTESWAP_OFF:
		mpui_byteswap_off();
		break;

	case OMAP_DSP_IOCTL_MBSEND:
		{
			struct omap_dsp_mailbox_cmd u_cmd;
			struct mbcmd_hw mb;
			if (copy_from_user(&u_cmd, (void *)arg, sizeof(u_cmd)))
				return -EFAULT;
			mb.cmd  = u_cmd.cmd;
			mb.data = u_cmd.data;
			ret = dsp_mbsend((struct mbcmd *)&mb);
			break;
		}

	case OMAP_DSP_IOCTL_SETVAR:
		{
			struct omap_dsp_varinfo var;
			if (copy_from_user(&var, (void *)arg, sizeof(var)))
				return -EFAULT;
			ret = dsp_setvar(var.varid, var.val[0]);
			break;
		}

	case OMAP_DSP_IOCTL_RUNLEVEL:
		dsp_runlevel(arg);
		break;

	case OMAP_DSP_IOCTL_FBEN:
		dsp_fbctl_enable();
		return 0;

	/*
	 * command level 2: commands which need lock
	 */
	case OMAP_DSP_IOCTL_DSPCFG:
		ret = dspcfg();
		break;

	case OMAP_DSP_IOCTL_DSPUNCFG:
		ret = dspuncfg();
		break;

	case OMAP_DSP_IOCTL_TASKCNT:
		ret = dsp_task_count();
		break;

	case OMAP_DSP_IOCTL_FBDIS:
		ret = dsp_fbctl_disable();
		break;

	case OMAP_DSP_IOCTL_SUSPEND:
		if ((ret = dsp_suspend()) < 0)
			break;
		dsp_reset();
		break;

	case OMAP_DSP_IOCTL_RESUME:
		if ((ret = dsp_resume()) < 0)
			break;
		dsp_run();
		break;

	case OMAP_DSP_IOCTL_REGMEMR:
		{
			struct omap_dsp_reginfo *u_reg = (void *)arg;
			unsigned short adr, val;

			if (copy_from_user(&adr, &u_reg->adr, sizeof(short)))
				return -EFAULT;
			if ((ret = dsp_regread(OMAP_DSP_MBCMD_REGRW_MEMR,
					       adr, &val)) < 0)
				return ret;
			if (copy_to_user(&u_reg->val, &val, sizeof(short)))
				return -EFAULT;
			break;
		}

	case OMAP_DSP_IOCTL_REGMEMW:
		{
			struct omap_dsp_reginfo reg;

			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
				return -EFAULT;
			ret = dsp_regwrite(OMAP_DSP_MBCMD_REGRW_MEMW,
					   reg.adr, reg.val);
			break;
		}

	case OMAP_DSP_IOCTL_REGIOR:
		{
			struct omap_dsp_reginfo *u_reg = (void *)arg;
			unsigned short adr, val;

			if (copy_from_user(&adr, &u_reg->adr, sizeof(short)))
				return -EFAULT;
			if ((ret = dsp_regread(OMAP_DSP_MBCMD_REGRW_IOR,
					       adr, &val)) < 0)
				return ret;
			if (copy_to_user(&u_reg->val, &val, sizeof(short)))
				return -EFAULT;
			break;
		}

	case OMAP_DSP_IOCTL_REGIOW:
		{
			struct omap_dsp_reginfo reg;

			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
				return -EFAULT;
			ret = dsp_regwrite(OMAP_DSP_MBCMD_REGRW_IOW,
					   reg.adr, reg.val);
			break;
		}

	case OMAP_DSP_IOCTL_GETVAR:
		{
			struct omap_dsp_varinfo *u_var = (void *)arg;
			unsigned char varid;
			unsigned short val[5]; /* maximum */
			int argc;

			if (copy_from_user(&varid, &u_var->varid, sizeof(char)))
				return -EFAULT;
			switch (varid) {
			case OMAP_DSP_MBCMD_VARID_ICRMASK:
				argc = 1;
				break;
			case OMAP_DSP_MBCMD_VARID_LOADINFO:
				argc = 5;
				break;
			default:
				return -EINVAL;
			}
			if ((ret = dsp_getvar(varid, val, argc)) < 0)
				return ret;
			if (copy_to_user(&u_var->val, val, sizeof(short) * argc))
				return -EFAULT;
			break;
		}

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

/*
 * functions called from mailbox1 interrupt routine
 */
void mbx1_suspend(struct mbcmd *mb)
{
	if (!waitqueue_active(&ioctl_wait_q) ||
	    (ioctl_wait_cmd != MBCMD(SUSPEND))) {
		printk(KERN_WARNING
		       "mbx: SUSPEND command received, "
		       "but nobody is waiting for it...\n");
		return;
	}

	ioctl_wait_cmd = 0;
	wake_up_interruptible(&ioctl_wait_q);
}

void mbx1_dspcfg(struct mbcmd *mb)
{
	unsigned char last   = mb->cmd_l & 0x80;
	unsigned char cfgcmd = mb->cmd_l & 0x7f;
	static unsigned long tmp_ipbuf_sys_da;

	/* mailbox protocol check */
	if (cfgcmd == OMAP_DSP_MBCMD_DSPCFG_PROTREV) {
		if (!waitqueue_active(&ioctl_wait_q) ||
		    (ioctl_wait_cmd != MBCMD(DSPCFG))) {
			printk(KERN_WARNING
			       "mbx: DSPCFG command received, "
			       "but nobody is waiting for it...\n");
			return;
		}

		mbx_revision = mb->data;
		if (mbx_revision == OMAP_DSP_MBPROT_REVISION)
			return;
#ifdef OLD_BINARY_SUPPORT
		else if (mbx_revision == MBREV_3_0) {
			printk(KERN_WARNING
			       "mbx: ***** old DSP binary *****\n"
			       "  Please update your DSP application.\n");
			return;
		}
#endif
		else {
			printk(KERN_ERR
			       "mbx: protocol revision check error!\n"
			       "  expected=0x%04x, received=0x%04x\n",
			       OMAP_DSP_MBPROT_REVISION, mb->data);
			mbx_revision = -1;
			goto abort;
		}
	}

	/*
	 * following commands are accepted only after
	 * revision check has been passed.
	 */
	if (!mbx_revision < 0) {
		printk(KERN_INFO
		       "mbx: DSPCFG command received, "
		       "but revision check has not been passed.\n");
		return;
	}

	if (!waitqueue_active(&ioctl_wait_q) ||
	    (ioctl_wait_cmd != MBCMD(DSPCFG))) {
		printk(KERN_WARNING
		       "mbx: DSPCFG command received, "
		       "but nobody is waiting for it...\n");
		return;
	}

	switch (cfgcmd) {
	case OMAP_DSP_MBCMD_DSPCFG_SYSADRH:
		tmp_ipbuf_sys_da = (unsigned long)mb->data << 16;
		break;

	case OMAP_DSP_MBCMD_DSPCFG_SYSADRL:
		tmp_ipbuf_sys_da |= mb->data;
		break;

	case OMAP_DSP_MBCMD_DSPCFG_ABORT:
		goto abort;

	default:
		printk(KERN_ERR
		       "mbx: Unknown CFG command: cmd_l=0x%02x, data=0x%04x\n",
		       mb->cmd_l, mb->data);
		return;
	}

	if (last) {
		unsigned long badr;
		unsigned short bln;
		unsigned short bsz;
		volatile unsigned short *buf;
		void *sync_seq;

		/* system IPBUF initialization */
		if (tmp_ipbuf_sys_da & 0x1) {
			printk(KERN_ERR
			       "mbx: system ipbuf address (0x%lx) "
			       "is odd number!\n", tmp_ipbuf_sys_da);
			goto abort;
		}
		ipbuf_sys_da = dspword_to_virt(tmp_ipbuf_sys_da);

		if (sync_with_dsp(&ipbuf_sys_da->s, OMAP_DSP_TID_ANON, 10) < 0) {
			printk(KERN_ERR "mbx: DSPCFG - IPBUF sync failed!\n");
			return;
		}
		/*
		 * read configuration data on system IPBUF
		 * we must read with 16bit-access
		 */
#ifdef OLD_BINARY_SUPPORT
		if (mbx_revision == OMAP_DSP_MBPROT_REVISION) {
#endif
			buf = ipbuf_sys_da->d;
			n_stask = buf[0];
			bln     = buf[1];
			bsz     = buf[2];
			badr    = MKLONG(buf[3], buf[4]);
			/*ipbuf_sys_da = dspword_to_virt(MKLONG(buf[5], buf[6])); */
			ipbuf_sys_ad = dspword_to_virt(MKLONG(buf[7], buf[8]));
			sync_seq = dspword_to_virt(MKLONG(buf[9], buf[10]));
#ifdef OLD_BINARY_SUPPORT
		} else if (mbx_revision == MBREV_3_0) {
			buf = ipbuf_sys_da->d;
			n_stask = buf[0];
			bln     = buf[1];
			bsz     = buf[2];
			badr    = MKLONG(buf[3], buf[4]);
			/* bkeep   = buf[5]; */
			/*ipbuf_sys_da = dspword_to_virt(MKLONG(buf[6], buf[67)); */
			ipbuf_sys_ad = dspword_to_virt(MKLONG(buf[8], buf[9]));
			sync_seq = dspword_to_virt(MKLONG(buf[10], buf[11]));
		} else /* should not occur */
			goto abort;
#endif

		/* ipbuf_config() should be done in interrupt routine. */
		if (ipbuf_config(bln, bsz, badr) < 0)
			goto abort;

		ipbuf_sys_da->s = OMAP_DSP_TID_FREE;

		/* mb_config() should be done in interrupt routine. */
		dsp_mb_config(sync_seq);

		ioctl_wait_cmd = 0;
		wake_up_interruptible(&ioctl_wait_q);
	}
	return;

abort:
	wake_up_interruptible(&ioctl_wait_q);
	return;
}

void mbx1_regrw(struct mbcmd *mb)
{
	if (!waitqueue_active(&ioctl_wait_q) ||
	    (ioctl_wait_cmd != MBCMD(REGRW))) {
		printk(KERN_WARNING
		       "mbx: REGRW command received, "
		       "but nobody is waiting for it...\n");
		return;
	}

	switch (mb->cmd_l) {
	case OMAP_DSP_MBCMD_REGRW_DATA:
		ioctl_wait_cmd = 0;
		varread_val[0] = mb->data;
		wake_up_interruptible(&ioctl_wait_q);
		return;

	default:
		printk(KERN_ERR
		       "mbx: Illegal REGRW command: "
		       "cmd_l=0x%02x, data=0x%04x\n", mb->cmd_l, mb->data);
		return;
	}
}

void mbx1_getvar(struct mbcmd *mb)
{
	unsigned char varid = mb->cmd_l;
	int i;
	volatile unsigned short *buf;

	if (!waitqueue_active(&ioctl_wait_q) ||
	    (ioctl_wait_cmd != MBCMD(GETVAR))) {
		printk(KERN_WARNING
		       "mbx: GETVAR command received, "
		       "but nobody is waiting for it...\n");
		return;
	}

	ioctl_wait_cmd = 0;
	switch (varid) {
	case OMAP_DSP_MBCMD_VARID_ICRMASK:
		varread_val[0] = mb->data;
		break;
	case OMAP_DSP_MBCMD_VARID_LOADINFO:
		{
			if (sync_with_dsp(&ipbuf_sys_da->s, OMAP_DSP_TID_ANON, 10) < 0) {
				printk(KERN_ERR
				       "mbx: GETVAR - IPBUF sync failed!\n");
				return;
			}
			/* need word access. do not use memcpy. */
			buf = ipbuf_sys_da->d;
			for (i = 0; i < 5; i++) {
				varread_val[i] = buf[i];
			}
			ipbuf_sys_da->s = OMAP_DSP_TID_FREE;
			break;
		}
	}
	wake_up_interruptible(&ioctl_wait_q);

	return;
}

/*
 * sysfs files
 */
static ssize_t ifver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len = 0;

	/*
	 * I/F VERSION descriptions:
	 *
	 * 3.2: sysfs / udev support
	 *      KMEM_RESERVE / KMEM_RELEASE ioctls for mem device
	 */

	/*
	 * print all supporting I/F VERSIONs, like followings.
	 *
	 * len += sprintf(buf, "3.1\n");
	 * len += sprintf(buf, "3.2\n");
	 */
	len += sprintf(buf + len, "3.2\n");

	return len;
}

static struct device_attribute dev_attr_ifver = __ATTR_RO(ifver);

static ssize_t icrmask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
#if 0
	if (dsp_is_ready()) {
		int ret;
		unsigned short val;

		if ((ret = dsp_getvar(OMAP_DSP_MBCMD_VARID_ICRMASK, &val, 1)) < 0)
			return ret;
		if (val != dsp_icrmask)
			printk(KERN_WARNING
			       "omapdsp: icrmask value is inconsistent!\n");
	}
#endif
	return sprintf(buf, "0x%04x\n", dsp_icrmask);
}

static ssize_t icrmask_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	dsp_icrmask = simple_strtol(buf, NULL, 16);

	if (dsp_is_ready()) {
		ret = dsp_setvar(OMAP_DSP_MBCMD_VARID_ICRMASK, dsp_icrmask);
		if (ret < 0)
			return ret;
	}

	return strlen(buf);
}

static struct device_attribute dev_attr_icrmask = 
	__ATTR(icrmask, S_IWUSR | S_IRUGO, icrmask_show, icrmask_store);

static ssize_t loadinfo_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int len;
	int ret;
	static unsigned short val[5];

	if ((ret = dsp_getvar(OMAP_DSP_MBCMD_VARID_LOADINFO, val, 5)) < 0)
		return ret;

	/* load info value range is 0(free) - 10000(busy) */
	len = sprintf(buf,
		      "DSP load info:\n"
		      "  10ms average = %3d.%02d%%\n"
		      "  1sec average = %3d.%02d%%  busiest 10ms = %3d.%02d%%\n"
		      "  1min average = %3d.%02d%%  busiest 1s   = %3d.%02d%%\n",
		      val[0]/100, val[0]%100,
		      val[1]/100, val[1]%100, val[2]/100, val[2]%100,
		      val[3]/100, val[3]%100, val[4]/100, val[4]%100);
	return len;
}

/*
 * This is declared at the top of this file.
 *
 * static struct device_attribute dev_attr_loadinfo = __ATTR_RO(loadinfo);
 */

#ifdef CONFIG_FB_OMAP_EXTERNAL_LCDC
void mbx1_fbctl_disable(void)
{
	if (!waitqueue_active(&ioctl_wait_q) ||
	    (ioctl_wait_cmd != MBCMD(KFUNC))) {
		printk(KERN_WARNING
		       "mbx: KFUNC:FBCTL command received, "
		       "but nobody is waiting for it...\n");
		return;
	}
	ioctl_wait_cmd = 0;
	wake_up_interruptible(&ioctl_wait_q);
}
#endif

#ifdef CONFIG_PROC_FS
/* for backward compatibility */
static int version_read_proc(char *page, char **start, off_t off, int count,
			     int *eof, void *data)
{
	/*
	 * This entry is read by 3.1 tools only, so leave it as is.
	 * 3.2 and later will read from sysfs file.
	 */
	return sprintf(page, "3.1\n");
}

static void __init dsp_ctl_create_proc(void)
{
	struct proc_dir_entry *ent;

	/* version */
	ent = create_proc_read_entry("version", 0, procdir_dsp,
				     version_read_proc, NULL);
	if (ent == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to register proc device: version\n");
	}
}

static void dsp_ctl_remove_proc(void)
{
	remove_proc_entry("version", procdir_dsp);
}
#endif /* CONFIG_PROC_FS */

struct file_operations dsp_ctl_fops = {
	.owner   = THIS_MODULE,
	.ioctl   = dsp_ctl_ioctl,
};

void __init dsp_ctl_init(void)
{
	device_create_file(&dsp_device.dev, &dev_attr_ifver);
	device_create_file(&dsp_device.dev, &dev_attr_icrmask);
#ifdef CONFIG_PROC_FS
	dsp_ctl_create_proc();
#endif
}

void dsp_ctl_exit(void)
{
	device_remove_file(&dsp_device.dev, &dev_attr_ifver);
	device_remove_file(&dsp_device.dev, &dev_attr_icrmask);
#ifdef CONFIG_PROC_FS
	dsp_ctl_remove_proc();
#endif
}
