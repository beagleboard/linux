/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2003-2006 Nokia Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/arch/mailbox.h>
#include "dsp_mbcmd.h"
#include "dsp.h"

char *subcmd_name(struct mbcmd *mb)
{
	u8 cmd_h = mb->cmd_h;
	u8 cmd_l = mb->cmd_l;
	char *s;

	switch (cmd_h) {
	case MBOX_CMD_DSP_RUNLEVEL:
		s = (cmd_l == RUNLEVEL_USER)     ? "USER":
		    (cmd_l == RUNLEVEL_SUPER)    ? "SUPER":
		    (cmd_l == RUNLEVEL_RECOVERY) ? "RECOVERY":
		    NULL;
		break;
	case MBOX_CMD_DSP_PM:
		s = (cmd_l == PM_DISABLE) ? "DISABLE":
		    (cmd_l == PM_ENABLE)  ? "ENABLE":
		    NULL;
		break;
	case MBOX_CMD_DSP_KFUNC:
		s = (cmd_l == KFUNC_FBCTL) ? "FBCTL":
		    (cmd_l == KFUNC_POWER) ? "POWER":
		    NULL;
		break;
	case MBOX_CMD_DSP_DSPCFG:
		{
			u8 cfgc = cmd_l & 0x7f;
			s = (cfgc == DSPCFG_REQ)     ? "REQ":
			    (cfgc == DSPCFG_SYSADRH) ? "SYSADRH":
			    (cfgc == DSPCFG_SYSADRL) ? "SYSADRL":
			    (cfgc == DSPCFG_ABORT)   ? "ABORT":
			    (cfgc == DSPCFG_PROTREV) ? "PROTREV":
			    NULL;
			break;
		}
	case MBOX_CMD_DSP_REGRW:
		s = (cmd_l == REGRW_MEMR) ? "MEMR":
		    (cmd_l == REGRW_MEMW) ? "MEMW":
		    (cmd_l == REGRW_IOR)  ? "IOR":
		    (cmd_l == REGRW_IOW)  ? "IOW":
		    (cmd_l == REGRW_DATA) ? "DATA":
		    NULL;
		break;
	case MBOX_CMD_DSP_GETVAR:
	case MBOX_CMD_DSP_SETVAR:
		s = (cmd_l == VARID_ICRMASK)  ? "ICRMASK":
		    (cmd_l == VARID_LOADINFO) ? "LOADINFO":
		    NULL;
		break;
	case MBOX_CMD_DSP_ERR:
		s = (cmd_l == EID_BADTID)     ? "BADTID":
		    (cmd_l == EID_BADTCN)     ? "BADTCN":
		    (cmd_l == EID_BADBID)     ? "BADBID":
		    (cmd_l == EID_BADCNT)     ? "BADCNT":
		    (cmd_l == EID_NOTLOCKED)  ? "NOTLOCKED":
		    (cmd_l == EID_STVBUF)     ? "STVBUF":
		    (cmd_l == EID_BADADR)     ? "BADADR":
		    (cmd_l == EID_BADTCTL)    ? "BADTCTL":
		    (cmd_l == EID_BADPARAM)   ? "BADPARAM":
		    (cmd_l == EID_FATAL)      ? "FATAL":
		    (cmd_l == EID_WDT)        ? "WDT":
		    (cmd_l == EID_NOMEM)      ? "NOMEM":
		    (cmd_l == EID_NORES)      ? "NORES":
		    (cmd_l == EID_IPBFULL)    ? "IPBFULL":
		    (cmd_l == EID_TASKNOTRDY) ? "TASKNOTRDY":
		    (cmd_l == EID_TASKBSY)    ? "TASKBSY":
		    (cmd_l == EID_TASKERR)    ? "TASKERR":
		    (cmd_l == EID_BADCFGTYP)  ? "BADCFGTYP":
		    (cmd_l == EID_DEBUG)      ? "DEBUG":
		    (cmd_l == EID_BADSEQ)     ? "BADSEQ":
		    (cmd_l == EID_BADCMD)     ? "BADCMD":
		    NULL;
		break;
	default:
		s = NULL;
	}

	return s;
}

/* output of show() method should fit to PAGE_SIZE */
#define MBLOG_DEPTH	64

struct mblogent {
	unsigned long jiffies;
	mbox_msg_t msg;
	arm_dsp_dir_t dir;
};

static struct {
	spinlock_t lock;
	int wp;
	unsigned long cnt, cnt_ad, cnt_da;
	struct mblogent ent[MBLOG_DEPTH];
} mblog = {
	.lock = SPIN_LOCK_UNLOCKED,
};

#ifdef CONFIG_OMAP_DSP_MBCMD_VERBOSE
static inline void mblog_print_cmd(struct mbcmd *mb, arm_dsp_dir_t dir)
{
	const struct cmdinfo *ci = cmdinfo[mb->cmd_h];
	char *dir_str;
	char *subname;

	dir_str = (dir == DIR_A2D) ? "sending  " : "receiving";
	switch (ci->cmd_l_type) {
	case CMD_L_TYPE_SUBCMD:
		subname = subcmd_name(mb);
		if (unlikely(!subname))
			subname = "Unknown";
		printk(KERN_DEBUG
		       "mbox: %s seq=%d, cmd=%02x:%02x(%s:%s), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, subname, mb->data);
		break;
	case CMD_L_TYPE_TID:
		printk(KERN_DEBUG
		       "mbox: %s seq=%d, cmd=%02x:%02x(%s:task %d), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, mb->cmd_l, mb->data);
		break;
	case CMD_L_TYPE_NULL:
		printk(KERN_DEBUG
		       "mbox: %s seq=%d, cmd=%02x:%02x(%s), data=%04x\n",
		       dir_str, mb->seq, mb->cmd_h, mb->cmd_l,
		       ci->name, mb->data);
		break;
	}
}
#else
static inline void mblog_print_cmd(struct mbcmd *mb, arm_dsp_dir_t dir) { }
#endif

void mblog_add(struct mbcmd *mb, arm_dsp_dir_t dir)
{
	struct mblogent *ent;

	spin_lock(&mblog.lock);
	ent = &mblog.ent[mblog.wp];
	ent->jiffies = jiffies;
	ent->msg = *(mbox_msg_t *)mb;
	ent->dir = dir;
	if (mblog.cnt < 0xffffffff)
		mblog.cnt++;
	switch (dir) {
	case DIR_A2D:
		if (mblog.cnt_ad < 0xffffffff)
			mblog.cnt_ad++;
		break;
	case DIR_D2A:
		if (mblog.cnt_da < 0xffffffff)
			mblog.cnt_da++;
		break;
	}
	if (++mblog.wp == MBLOG_DEPTH)
		mblog.wp = 0;
	spin_unlock(&mblog.lock);

	mblog_print_cmd(mb, dir);
}

/*
 * sysfs file
 */
static ssize_t mblog_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len = 0;
	int wp;
	int i;

	spin_lock(&mblog.lock);

	wp = mblog.wp;
	len += sprintf(buf + len,
		       "log count:%ld / ARM->DSP:%ld, DSP->ARM:%ld\n",
		       mblog.cnt, mblog.cnt_ad, mblog.cnt_da);
	if (mblog.cnt == 0)
		goto done;

	len += sprintf(buf + len, "           ARM->DSP   ARM<-DSP\n");
	len += sprintf(buf + len, " jiffies  cmd  data  cmd  data\n");
	i = (mblog.cnt >= MBLOG_DEPTH) ? wp : 0;
	do {
		struct mblogent *ent = &mblog.ent[i];
		struct mbcmd *mb = (struct mbcmd *)&ent->msg;
		char *subname;
		struct cmdinfo ci_null = {
			.name = "Unknown",
			.cmd_l_type = CMD_L_TYPE_NULL,
		};
		const struct cmdinfo *ci;

		len += sprintf(buf + len,
			       (ent->dir == DIR_A2D) ?
				"%08lx  %04x %04x            ":
				"%08lx             %04x %04x ",
			       ent->jiffies,
			       (ent->msg >> 16) & 0x7fff, ent->msg & 0xffff);

		if ((ci = cmdinfo[mb->cmd_h]) == NULL)
			ci = &ci_null;

		switch (ci->cmd_l_type) {
		case CMD_L_TYPE_SUBCMD:
			if ((subname = subcmd_name(mb)) == NULL)
				subname = "Unknown";
			len += sprintf(buf + len, "%s:%s\n",
				       ci->name, subname);
			break;
		case CMD_L_TYPE_TID:
			len += sprintf(buf + len, "%s:task %d\n",
				       ci->name, mb->cmd_l);
			break;
		case CMD_L_TYPE_NULL:
			len += sprintf(buf + len, "%s\n", ci->name);
			break;
		}

		if (++i == MBLOG_DEPTH)
			i = 0;
	} while (i != wp);

done:
	spin_unlock(&mblog.lock);

	return len;
}

static struct device_attribute dev_attr_mblog = __ATTR_RO(mblog);

void __init mblog_init(void)
{
	device_create_file(omap_dsp->dev, &dev_attr_mblog);
}

void mblog_exit(void)
{
	device_remove_file(omap_dsp->dev, &dev_attr_mblog);
}
