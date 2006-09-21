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
 * mailbox command: 0x00 - 0x7f
 * when a driver wants to use mailbox, it must reserve mailbox commands here.
 */
#define MBOX_CMD_DSP_WDSND	0x10
#define MBOX_CMD_DSP_WDREQ	0x11
#define MBOX_CMD_DSP_BKSND	0x20
#define MBOX_CMD_DSP_BKREQ	0x21
#define MBOX_CMD_DSP_BKYLD	0x23
#define MBOX_CMD_DSP_BKSNDP	0x24
#define MBOX_CMD_DSP_BKREQP	0x25
#define MBOX_CMD_DSP_TCTL	0x30
#define MBOX_CMD_DSP_TCTLDATA	0x31
#define MBOX_CMD_DSP_POLL	0x32
#define MBOX_CMD_DSP_WDT	0x50
#define MBOX_CMD_DSP_RUNLEVEL	0x51
#define MBOX_CMD_DSP_PM		0x52
#define MBOX_CMD_DSP_SUSPEND	0x53
#define MBOX_CMD_DSP_KFUNC	0x54
#define MBOX_CMD_DSP_TCFG	0x60
#define MBOX_CMD_DSP_TADD	0x62
#define MBOX_CMD_DSP_TDEL	0x63
#define MBOX_CMD_DSP_TSTOP	0x65
#define MBOX_CMD_DSP_DSPCFG	0x70
#define MBOX_CMD_DSP_REGRW	0x72
#define MBOX_CMD_DSP_GETVAR	0x74
#define MBOX_CMD_DSP_SETVAR	0x75
#define MBOX_CMD_DSP_ERR	0x78
#define MBOX_CMD_DSP_DBG	0x79

/*
 * DSP mailbox protocol definitions
 */
#define MBPROT_REVISION	0x0019

#define TCTL_TINIT		0x0000
#define TCTL_TEN		0x0001
#define TCTL_TDIS		0x0002
#define TCTL_TCLR		0x0003
#define TCTL_TCLR_FORCE		0x0004

#define RUNLEVEL_USER		0x01
#define RUNLEVEL_SUPER		0x0e
#define RUNLEVEL_RECOVERY	0x10

#define PM_DISABLE		0x00
#define PM_ENABLE		0x01

#define KFUNC_FBCTL		0x00
#define KFUNC_POWER		0x01

#define FBCTL_UPD		0x0000
#define FBCTL_ENABLE		0x0002
#define FBCTL_DISABLE		0x0003

/* KFUNC_POWER */
#define AUDIO_PWR_UP		0x0000	/* ARM(exe/ack)	<->  DSP(req)	*/
#define AUDIO_PWR_DOWN		0x0001	/* ARM(exe)	<-  DSP(req)	*/
#define AUDIO_PWR_DOWN1		AUDIO_PWR_DOWN
#define AUDIO_PWR_DOWN2		0x0002
#define DSP_PWR_UP		0x0003	/* ARM(exe/snd)	->  DSP(exe)	*/
#define DSP_PWR_DOWN		0x0004	/* ARM(exe)	<-  DSP(req)	*/
#define DVFS_START		0x0006	/* ARM(req)	<-> DSP(exe/ack)*/
#define DVFS_STOP		0x0007	/* ARM(req)	 -> DSP(exe)	*/

#define TDEL_SAFE		0x0000
#define TDEL_KILL		0x0001

#define DSPCFG_REQ		0x00
#define DSPCFG_SYSADRH		0x28
#define DSPCFG_SYSADRL		0x29
#define DSPCFG_PROTREV		0x70
#define DSPCFG_ABORT		0x78
#define DSPCFG_LAST		0x80

#define REGRW_MEMR		0x00
#define REGRW_MEMW		0x01
#define REGRW_IOR		0x02
#define REGRW_IOW		0x03
#define REGRW_DATA		0x04

#define VARID_ICRMASK		0x00
#define VARID_LOADINFO		0x01

#define TTYP_ARCV		0x0001
#define TTYP_ASND		0x0002
#define TTYP_BKMD		0x0004
#define TTYP_BKDM		0x0008
#define TTYP_PVMD		0x0010
#define TTYP_PVDM		0x0020

#define EID_BADTID		0x10
#define EID_BADTCN		0x11
#define EID_BADBID		0x20
#define EID_BADCNT		0x21
#define EID_NOTLOCKED		0x22
#define EID_STVBUF		0x23
#define EID_BADADR		0x24
#define EID_BADTCTL		0x30
#define EID_BADPARAM		0x50
#define EID_FATAL		0x58
#define EID_NOMEM		0xc0
#define EID_NORES		0xc1
#define EID_IPBFULL		0xc2
#define EID_WDT			0xd0
#define EID_TASKNOTRDY		0xe0
#define EID_TASKBSY		0xe1
#define EID_TASKERR		0xef
#define EID_BADCFGTYP		0xf0
#define EID_DEBUG		0xf8
#define EID_BADSEQ		0xfe
#define EID_BADCMD		0xff

#define TNM_LEN			16

#define TID_FREE		0xff
#define TID_ANON		0xfe

#define BID_NULL		0xffff
#define BID_PVT			0xfffe
