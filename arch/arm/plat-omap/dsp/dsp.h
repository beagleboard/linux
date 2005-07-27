/*
 * linux/arch/arm/mach-omap/dsp/dsp.h
 *
 * Header for OMAP DSP driver
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
 * 2005/06/09:  DSP Gateway version 3.3
 */

#include "hardware_dsp.h"
#include "dsp_common.h"

#define OLD_BINARY_SUPPORT	y

#ifdef OLD_BINARY_SUPPORT
#define MBREV_3_0	0x0017
#define MBREV_3_2	0x0018
#endif

#define DSP_INIT_PAGE	0xfff000
/* idle program will be placed at IDLEPG_BASE. */
#define IDLEPG_BASE	0xfffe00
#define IDLEPG_SIZE	0x100

/* timeout value for DSP response */
#define DSP_TIMEOUT	(10 * HZ)

enum dsp_mem_type_e {
	MEM_TYPE_CROSSING = -1,
	MEM_TYPE_NONE = 0,
	MEM_TYPE_DARAM,
	MEM_TYPE_SARAM,
	MEM_TYPE_EXTERN,
};

enum arm_dsp_dir {
	DIR_A2D,
	DIR_D2A,
};

/*
 * INT_D2A_MB value definition
 *   INT_DSP_MAILBOX1: use Mailbox 1 (INT 10) for DSP->ARM mailbox
 *   INT_DSP_MAILBOX2: use Mailbox 2 (INT 11) for DSP->ARM mailbox
 */
#define INT_D2A_MB1	INT_DSP_MAILBOX1

/* keep 2 entries for OMAP_DSP_TID_FREE and OMAP_DSP_TID_ANON */
#define TASKDEV_MAX	254

#define MKLONG(uw,lw)	(((unsigned long)(uw)) << 16 | (lw))
#define MKVIRT(uw,lw)	dspword_to_virt(MKLONG((uw), (lw)));
#define MBCMD(nm)	OMAP_DSP_MBCMD_##nm

struct sync_seq {
	unsigned short da_dsp;
	unsigned short da_arm;
	unsigned short ad_dsp;
	unsigned short ad_arm;
};

struct mem_sync_struct {
	struct sync_seq *DARAM;
	struct sync_seq *SARAM;
	struct sync_seq *SDRAM;
};

/* struct mbcmd and struct mbcmd_hw must be compatible */
struct mbcmd {
	unsigned short cmd_l:8;
	unsigned short cmd_h:7;
	unsigned short seq:1;
	unsigned short data;
};

struct mbcmd_hw {
	unsigned short cmd;
	unsigned short data;
};

#define mbcmd_set(mb, h, l, d) \
	do { \
		(mb).cmd_h = (h); \
		(mb).cmd_l = (l); \
		(mb).data  = (d); \
	} while(0)

struct mb_exarg {
	unsigned char tid;
	int argc;
	unsigned short *argv;
};

extern void dsp_mb_start(void);
extern void dsp_mb_stop(void);
extern int dsp_mb_config(void *p);
extern int sync_with_dsp(unsigned short *syncwd, unsigned short tid,
			 int try_cnt);
extern int __mbcmd_send(struct mbcmd *mb);
extern int __dsp_mbcmd_send(struct mbcmd *mb, struct mb_exarg *arg,
			    int recovery_flag);
#define dsp_mbcmd_send(mb)		__dsp_mbcmd_send(mb, NULL, 0)
#define dsp_mbcmd_send_exarg(mb, arg)	__dsp_mbcmd_send(mb, arg, 0)
extern int __dsp_mbcmd_send_and_wait(struct mbcmd *mb, struct mb_exarg *arg,
				     wait_queue_head_t *q);
#define dsp_mbcmd_send_and_wait(mb, q) \
	__dsp_mbcmd_send_and_wait(mb, NULL, q)
#define dsp_mbcmd_send_and_wait_exarg(mb, arg, q) \
	__dsp_mbcmd_send_and_wait(mb, arg, q)
int __dsp_mbsend(unsigned char cmdh, unsigned char cmdl, unsigned short data,
		 int recovery_flag);
#define dsp_mbsend(cmdh, cmdl, data) \
	__dsp_mbsend(cmdh, cmdl, data, 0)
#define dsp_mbsend_recovery(cmdh, cmdl, data) \
	__dsp_mbsend(cmdh, cmdl, data, 1)

extern void ipbuf_start(void);
extern void ipbuf_stop(void);
extern int ipbuf_config(unsigned short ln, unsigned short lsz, void *base);
extern int ipbuf_sys_config(void *p, enum arm_dsp_dir dir);
extern int ipbuf_p_validate(void *p, enum arm_dsp_dir dir);
extern unsigned short get_free_ipbuf(unsigned char tid);
extern void unuse_ipbuf_nowait(unsigned short bid);
extern void unuse_ipbuf(unsigned short bid);
extern void release_ipbuf(unsigned short bid);
extern void balance_ipbuf(void);

#define release_ipbuf_pvt(ipbuf_pvt) \
	do { \
		(ipbuf_pvt)->s = OMAP_DSP_TID_FREE; \
	} while(0)

extern int mbx_revision;

extern int dsp_is_ready(void);
extern int dspuncfg(void);
extern void dsp_runlevel(unsigned char level);
extern int dsp_suspend(void);
extern int dsp_resume(void);

extern int dsp_task_config_all(unsigned char n);
extern void dsp_task_unconfig_all(void);
extern unsigned char dsp_task_count(void);
extern int dsp_taskmod_busy(void);
extern int dsp_mkdev(char *name);
extern int dsp_rmdev(char *name);
extern int dsp_tadd(unsigned char minor, unsigned long adr);
extern int dsp_tdel(unsigned char minor);
extern int dsp_tkill(unsigned char minor);
extern long taskdev_state_stale(unsigned char minor);
extern int dsp_dbg_config(short *buf, unsigned short sz, unsigned short lsz);
extern void dsp_dbg_stop(void);

extern int ipbuf_is_held(unsigned char tid, unsigned short bid);

extern void dsp_mem_sync_inc(void);
extern int dsp_mem_sync_config(struct mem_sync_struct *sync);
extern enum dsp_mem_type_e dsp_mem_type(void *vadr, size_t len);
extern int dsp_address_validate(void *p, size_t len, char *fmt, ...);
extern int dsp_mem_enable(void *adr);
extern void dsp_mem_disable(void *adr);
extern void dsp_mem_usecount_clear(void);
extern void exmap_use(void *vadr, size_t len);
extern void exmap_unuse(void *vadr, size_t len);
extern unsigned long dsp_virt_to_phys(void *vadr, size_t *len);
extern void dsp_mem_start(void);
extern void dsp_mem_stop(void);

extern void dsp_twch_start(void);
extern void dsp_twch_stop(void);
extern void dsp_twch_touch(void);

extern void dsp_err_start(void);
extern void dsp_err_stop(void);
extern void dsp_err_mmu_set(unsigned long adr);
extern void dsp_err_mmu_clear(void);
extern int dsp_err_mmu_isset(void);
extern void dsp_err_wdt_clear(void);
extern int dsp_err_wdt_isset(void);

enum cmd_l_type {
	CMD_L_TYPE_NULL,
	CMD_L_TYPE_TID,
	CMD_L_TYPE_SUBCMD,
};

struct cmdinfo {
	char *name;
	enum cmd_l_type cmd_l_type;
	void (*handler)(struct mbcmd *mb);
};

extern const struct cmdinfo *cmdinfo[];

#define cmd_name(mb)	(cmdinfo[(mb).cmd_h]->name)
extern char *subcmd_name(struct mbcmd *mb);

extern void mblog_add(struct mbcmd *mb, enum arm_dsp_dir dir);
#ifdef CONFIG_OMAP_DSP_MBCMD_VERBOSE
extern void mblog_printcmd(struct mbcmd *mb, enum arm_dsp_dir dir);
#else /* CONFIG_OMAP_DSP_MBCMD_VERBOSE */
#define mblog_printcmd(mb, dir)	do {} while(0)
#endif /* CONFIG_OMAP_DSP_MBCMD_VERBOSE */

#ifdef CONFIG_PROC_FS
extern struct proc_dir_entry *procdir_dsp;
#endif /* CONFIG_PROC_FS */

extern struct platform_device dsp_device;
