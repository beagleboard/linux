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

#include <linux/platform_device.h>
#include "hardware_dsp.h"
#include "dsp_common.h"

/*
 * MAJOR device number: !! allocated arbitrary !!
 */
#define OMAP_DSP_CTL_MAJOR		96
#define OMAP_DSP_TASK_MAJOR		97

#define OLD_BINARY_SUPPORT	y

#ifdef OLD_BINARY_SUPPORT
#define MBREV_3_0	0x0017
#define MBREV_3_2	0x0018
#endif

#define DSP_INIT_PAGE	0xfff000

#ifdef CONFIG_ARCH_OMAP1
/* idle program will be placed at IDLEPG_BASE. */
#define IDLEPG_BASE	0xfffe00
#define IDLEPG_SIZE	0x100
#endif /* CONFIG_ARCH_OMAP1 */

/* timeout value for DSP response */
#define DSP_TIMEOUT	(10 * HZ)

enum dsp_mem_type_e {
	MEM_TYPE_CROSSING = -1,
	MEM_TYPE_NONE = 0,
	MEM_TYPE_DARAM,
	MEM_TYPE_SARAM,
	MEM_TYPE_EXTERN,
};


typedef int __bitwise arm_dsp_dir_t;
#define DIR_A2D	((__force arm_dsp_dir_t) 1)
#define DIR_D2A	((__force arm_dsp_dir_t) 2)

enum cfgstat_e {
	CFGSTAT_CLEAN = 0,
	CFGSTAT_READY,
	CFGSTAT_SUSPEND,
	CFGSTAT_RESUME,	/* request only */
	CFGSTAT_MAX
};

enum errcode_e {
	ERRCODE_WDT = 0,
	ERRCODE_MMU,
	ERRCODE_MAX
};

/* keep 2 entries for TID_FREE and TID_ANON */
#define TASKDEV_MAX	254

#define MK32(uw,lw)	(((u32)(uw)) << 16 | (lw))
#define MKLONG(uw,lw)	(((unsigned long)(uw)) << 16 | (lw))
#define MKVIRT(uw,lw)	dspword_to_virt(MKLONG((uw), (lw)));

struct sync_seq {
	u16 da_dsp;
	u16 da_arm;
	u16 ad_dsp;
	u16 ad_arm;
};

struct mem_sync_struct {
	struct sync_seq *DARAM;
	struct sync_seq *SARAM;
	struct sync_seq *SDRAM;
};

/* struct mbcmd and union mbcmd_hw must be compatible */
struct mbcmd {
	u32 data:16;
	u32 cmd_l:8;
	u32 cmd_h:7;
	u32 seq:1;
};

#define MBCMD_INIT(h, l, d) { \
		.cmd_h = (h), \
		.cmd_l = (l), \
		.data  = (d), \
	}

struct mb_exarg {
	u8 tid;
	int argc;
	u16 *argv;
};

extern void dsp_mbox_start(void);
extern void dsp_mbox_stop(void);
extern int dsp_mbox_config(void *p);
extern int sync_with_dsp(u16 *syncwd, u16 tid, int try_cnt);
extern int __dsp_mbcmd_send_exarg(struct mbcmd *mb, struct mb_exarg *arg,
				  int recovery_flag);
#define dsp_mbcmd_send(mb)		__dsp_mbcmd_send_exarg((mb), NULL, 0)
#define dsp_mbcmd_send_exarg(mb, arg)	__dsp_mbcmd_send_exarg((mb), (arg), 0)
extern int dsp_mbcmd_send_and_wait_exarg(struct mbcmd *mb, struct mb_exarg *arg,
					 wait_queue_head_t *q);
#define dsp_mbcmd_send_and_wait(mb, q) \
	dsp_mbcmd_send_and_wait_exarg((mb), NULL, (q))

static __inline__ int __mbcompose_send_exarg(u8 cmd_h, u8 cmd_l, u16 data,
					     struct mb_exarg *arg,
					     int recovery_flag)
{
	struct mbcmd mb = MBCMD_INIT(cmd_h, cmd_l, data);
	return __dsp_mbcmd_send_exarg(&mb, arg, recovery_flag);
}
#define mbcompose_send(cmd_h, cmd_l, data) \
	__mbcompose_send_exarg(MBOX_CMD_DSP_##cmd_h, (cmd_l), (data), NULL, 0)
#define mbcompose_send_exarg(cmd_h, cmd_l, data, arg) \
	__mbcompose_send_exarg(MBOX_CMD_DSP_##cmd_h, (cmd_l), (data), arg, 0)
#define mbcompose_send_recovery(cmd_h, cmd_l, data) \
	__mbcompose_send_exarg(MBOX_CMD_DSP_##cmd_h, (cmd_l), (data), NULL, 1)

static __inline__ int __mbcompose_send_and_wait_exarg(u8 cmd_h, u8 cmd_l,
						      u16 data,
						      struct mb_exarg *arg,
						      wait_queue_head_t *q)
{
	struct mbcmd mb = MBCMD_INIT(cmd_h, cmd_l, data);
	return dsp_mbcmd_send_and_wait_exarg(&mb, arg, q);
}
#define mbcompose_send_and_wait(cmd_h, cmd_l, data, q) \
	__mbcompose_send_and_wait_exarg(MBOX_CMD_DSP_##cmd_h, (cmd_l), (data), \
					NULL, (q))
#define mbcompose_send_and_wait_exarg(cmd_h, cmd_l, data, arg, q) \
	__mbcompose_send_and_wait_exarg(MBOX_CMD_DSP_##cmd_h, (cmd_l), (data), \
					(arg), (q))

extern struct ipbuf_head *bid_to_ipbuf(u16 bid);
extern void ipbuf_start(void);
extern void ipbuf_stop(void);
extern int ipbuf_config(u16 ln, u16 lsz, void *base);
extern int ipbuf_sys_config(void *p, arm_dsp_dir_t dir);
extern int ipbuf_p_validate(void *p, arm_dsp_dir_t dir);
extern struct ipbuf_head *get_free_ipbuf(u8 tid);
extern void release_ipbuf(struct ipbuf_head *ipb_h);
extern void balance_ipbuf(void);
extern void unuse_ipbuf(struct ipbuf_head *ipb_h);
extern void unuse_ipbuf_nowait(struct ipbuf_head *ipb_h);

#define release_ipbuf_pvt(ipbuf_pvt) \
	do { \
		(ipbuf_pvt)->s = TID_FREE; \
	} while(0)

extern int mbox_revision;

extern int dsp_cfgstat_request(enum cfgstat_e st);
extern enum cfgstat_e dsp_cfgstat_get_stat(void);
extern int dsp_set_runlevel(u8 level);

extern int dsp_task_config_all(u8 n);
extern void dsp_task_unconfig_all(void);
extern u8 dsp_task_count(void);
extern int dsp_taskmod_busy(void);
extern int dsp_mkdev(char *name);
extern int dsp_rmdev(char *name);
extern int dsp_tadd_minor(unsigned char minor, dsp_long_t adr);
extern int dsp_tdel_minor(unsigned char minor);
extern int dsp_tkill_minor(unsigned char minor);
extern long taskdev_state_stale(unsigned char minor);
extern int dsp_dbg_config(u16 *buf, u16 sz, u16 lsz);
extern void dsp_dbg_stop(void);

extern int ipbuf_is_held(u8 tid, u16 bid);

extern int dsp_mem_sync_inc(void);
extern int dsp_mem_sync_config(struct mem_sync_struct *sync);
extern enum dsp_mem_type_e dsp_mem_type(void *vadr, size_t len);
extern int dsp_address_validate(void *p, size_t len, char *fmt, ...);
extern int dsp_mem_enable(void *adr);
extern void dsp_mem_disable(void *adr);
#ifdef CONFIG_ARCH_OMAP1
extern void dsp_mem_usecount_clear(void);
#endif
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
extern void dsp_err_set(enum errcode_e code, unsigned long arg);
extern void dsp_err_clear(enum errcode_e code);
extern int dsp_err_isset(enum errcode_e code);

enum cmd_l_type_e {
	CMD_L_TYPE_NULL,
	CMD_L_TYPE_TID,
	CMD_L_TYPE_SUBCMD,
};

struct cmdinfo {
	char *name;
	enum cmd_l_type_e cmd_l_type;
	void (*handler)(struct mbcmd *mb);
};

extern const struct cmdinfo *cmdinfo[];

#define cmd_name(mb)	(cmdinfo[(mb).cmd_h]->name)
extern char *subcmd_name(struct mbcmd *mb);

extern void mblog_add(struct mbcmd *mb, arm_dsp_dir_t dir);
