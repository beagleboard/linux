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

/*
 * for /dev/dspctl/ctl
 */
#define DSPCTL_IOCTL_RESET		1
#define DSPCTL_IOCTL_RUN		2
#define DSPCTL_IOCTL_SETRSTVECT		3
#ifdef CONFIG_ARCH_OMAP1
#define DSPCTL_IOCTL_CPU_IDLE		4
#define DSPCTL_IOCTL_MPUI_WORDSWAP_ON	5
#define DSPCTL_IOCTL_MPUI_WORDSWAP_OFF	6
#define DSPCTL_IOCTL_MPUI_BYTESWAP_ON	7
#define DSPCTL_IOCTL_MPUI_BYTESWAP_OFF	8
#define DSPCTL_IOCTL_GBL_IDLE		9
#endif /* CONFIG_ARCH_OMAP1 */
#define DSPCTL_IOCTL_DSPCFG		10
#define DSPCTL_IOCTL_DSPUNCFG		11
#define DSPCTL_IOCTL_TASKCNT		12
#define DSPCTL_IOCTL_POLL		13
#define DSPCTL_IOCTL_REGMEMR		40
#define DSPCTL_IOCTL_REGMEMW		41
#define DSPCTL_IOCTL_REGIOR		42
#define DSPCTL_IOCTL_REGIOW		43
#define DSPCTL_IOCTL_GETVAR		44
#define DSPCTL_IOCTL_SETVAR		45
#define DSPCTL_IOCTL_RUNLEVEL		50
#define DSPCTL_IOCTL_SUSPEND		51
#define DSPCTL_IOCTL_RESUME		52
#ifdef CONFIG_OMAP_DSP_FBEXPORT
#define DSPCTL_IOCTL_FBEN		53
#define DSPCTL_IOCTL_FBDIS		54
#endif /* CONFIG_OMAP_DSP_FBEXPORT */
#define DSPCTL_IOCTL_MBSEND		99

struct omap_dsp_mailbox_cmd {
	__u16 cmd;
	__u16 data;
};

struct omap_dsp_reginfo {
	__u16 adr;
	__u16 val;
};

struct omap_dsp_varinfo {
	__u8 varid;
	__u16 val[0];
};

/*
 * for taskdev
 * (ioctls below should be >= 0x10000)
 */
#define TASK_IOCTL_BFLSH	0x10000
#define TASK_IOCTL_SETBSZ	0x10001
#define TASK_IOCTL_LOCK		0x10002
#define TASK_IOCTL_UNLOCK	0x10003
#define TASK_IOCTL_GETNAME	0x10004

/*
 * for /dev/dspctl/mem
 */
#define MEM_IOCTL_EXMAP		1
#define MEM_IOCTL_EXUNMAP	2
#define MEM_IOCTL_EXMAP_FLUSH	3
#define MEM_IOCTL_FBEXPORT	5
#ifdef CONFIG_ARCH_OMAP1
#define MEM_IOCTL_MMUITACK	7
#endif
#define MEM_IOCTL_MMUINIT	9
#define MEM_IOCTL_KMEM_RESERVE	11
#define MEM_IOCTL_KMEM_RELEASE	12

struct omap_dsp_mapinfo {
	__u32 dspadr;
	__u32 size;
};

/*
 * for /dev/dspctl/twch
 */
#define TWCH_IOCTL_MKDEV	1
#define TWCH_IOCTL_RMDEV	2
#define TWCH_IOCTL_TADD		11
#define TWCH_IOCTL_TDEL		12
#define TWCH_IOCTL_TKILL	13

struct omap_dsp_taddinfo {
	__u8 minor;
	__u32 taskadr;
};

#define TADD_ABORTADR	0xffffffff
