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

#ifndef __PLAT_OMAP_DSP_DSP_H
#define __PLAT_OMAP_DSP_DSP_H

#include "hardware_dsp.h"
#include <mach/dsp_common.h>
#include <mach/mmu.h>


#ifdef CONFIG_ARCH_OMAP2
#include "../../../arch/arm/mach-omap2/prm.h"
#include "../../../arch/arm/mach-omap2/prm-regbits-24xx.h"
#include "../../../arch/arm/mach-omap2/cm.h"
#include "../../../arch/arm/mach-omap2/cm-regbits-24xx.h"
#endif

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

typedef u32 dsp_long_t;	/* must have ability to carry TADD_ABORTADR */

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

static inline int __mbcompose_send_exarg(u8 cmd_h, u8 cmd_l, u16 data,
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

static inline int __mbcompose_send_and_wait_exarg(u8 cmd_h, u8 cmd_l,
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

extern struct omap_mmu dsp_mmu;

#define dsp_mem_enable(addr)	omap_mmu_mem_enable(&dsp_mmu, (addr))
#define dsp_mem_disable(addr)	omap_mmu_mem_disable(&dsp_mmu, (addr))

#define DSPSPACE_SIZE	0x1000000

#define omap_set_bit_regw(b,r) \
	do { omap_writew(omap_readw(r) | (b), (r)); } while(0)
#define omap_clr_bit_regw(b,r) \
	do { omap_writew(omap_readw(r) & ~(b), (r)); } while(0)
#define omap_set_bit_regl(b,r) \
	do { omap_writel(omap_readl(r) | (b), (r)); } while(0)
#define omap_clr_bit_regl(b,r) \
	do { omap_writel(omap_readl(r) & ~(b), (r)); } while(0)
#define omap_set_bits_regl(val,mask,r) \
	do { omap_writel((omap_readl(r) & ~(mask)) | (val), (r)); } while(0)

#define dspword_to_virt(dw)	((void *)(dspmem_base + ((dw) << 1)))
#define dspbyte_to_virt(db)	((void *)(dspmem_base + (db)))
#define virt_to_dspword(va) \
	((dsp_long_t)(((unsigned long)(va) - dspmem_base) >> 1))
#define virt_to_dspbyte(va) \
	((dsp_long_t)((unsigned long)(va) - dspmem_base))
#define is_dsp_internal_mem(va) \
	(((unsigned long)(va) >= dspmem_base) &&  \
	 ((unsigned long)(va) < dspmem_base + dspmem_size))
#define is_dspbyte_internal_mem(db)	((db) < dspmem_size)
#define is_dspword_internal_mem(dw)	(((dw) << 1) < dspmem_size)

#ifdef CONFIG_ARCH_OMAP1
/*
 * MPUI byteswap/wordswap on/off
 *   default setting: wordswap = all, byteswap = APIMEM only
 */
#define mpui_wordswap_on() \
	omap_set_bits_regl(MPUI_CTRL_WORDSWAP_ALL, MPUI_CTRL_WORDSWAP_MASK, \
			   MPUI_CTRL)

#define mpui_wordswap_off() \
	omap_set_bits_regl(MPUI_CTRL_WORDSWAP_NONE, MPUI_CTRL_WORDSWAP_MASK, \
			   MPUI_CTRL)

#define mpui_byteswap_on() \
	omap_set_bits_regl(MPUI_CTRL_BYTESWAP_API, MPUI_CTRL_BYTESWAP_MASK, \
			   MPUI_CTRL)

#define mpui_byteswap_off() \
	omap_set_bits_regl(MPUI_CTRL_BYTESWAP_NONE, MPUI_CTRL_BYTESWAP_MASK, \
			   MPUI_CTRL)

/*
 * TC wordswap on / off
 */
#define tc_wordswap() \
	do { \
		omap_writel(TC_ENDIANISM_SWAP_WORD | TC_ENDIANISM_EN, \
			    TC_ENDIANISM); \
	} while(0)

#define tc_noswap()	omap_clr_bit_regl(TC_ENDIANISM_EN, TC_ENDIANISM)

/*
 * enable priority registers, EMIF, MPUI control logic
 */
#define __dsp_enable()	omap_set_bit_regw(ARM_RSTCT1_DSP_RST, ARM_RSTCT1)
#define __dsp_disable()	omap_clr_bit_regw(ARM_RSTCT1_DSP_RST, ARM_RSTCT1)
#define __dsp_run()	omap_set_bit_regw(ARM_RSTCT1_DSP_EN, ARM_RSTCT1)
#define __dsp_reset()	omap_clr_bit_regw(ARM_RSTCT1_DSP_EN, ARM_RSTCT1)
#endif /* CONFIG_ARCH_OMAP1 */

#ifdef CONFIG_ARCH_OMAP2
/*
 * PRCM / IPI control logic
 *
 * REVISIT: these macros should probably be static inline functions
 */
#define __dsp_core_enable() \
	do { prm_write_mod_reg(prm_read_mod_reg(OMAP24XX_DSP_MOD, RM_RSTCTRL) \
	     & ~OMAP24XX_RST1_DSP, OMAP24XX_DSP_MOD, RM_RSTCTRL); } while (0)
#define __dsp_core_disable() \
	do { prm_write_mod_reg(prm_read_mod_reg(OMAP24XX_DSP_MOD, RM_RSTCTRL) \
	     | OMAP24XX_RST1_DSP, OMAP24XX_DSP_MOD, RM_RSTCTRL); } while (0)
#define __dsp_per_enable() \
	do { prm_write_mod_reg(prm_read_mod_reg(OMAP24XX_DSP_MOD, RM_RSTCTRL) \
	     & ~OMAP24XX_RST2_DSP, OMAP24XX_DSP_MOD, RM_RSTCTRL); } while (0)
#define __dsp_per_disable() \
	do { prm_write_mod_reg(prm_read_mod_reg(OMAP24XX_DSP_MOD, RM_RSTCTRL) \
	     | OMAP24XX_RST2_DSP, OMAP24XX_DSP_MOD, RM_RSTCTRL); } while (0)
#endif /* CONFIG_ARCH_OMAP2 */

#if defined(CONFIG_ARCH_OMAP1)
extern struct clk *dsp_ck_handle;
extern struct clk *api_ck_handle;
#elif defined(CONFIG_ARCH_OMAP2)
extern struct clk *dsp_fck_handle;
extern struct clk *dsp_ick_handle;
#endif
extern dsp_long_t dspmem_base, dspmem_size,
		  daram_base, daram_size,
		  saram_base, saram_size;

enum cpustat_e {
	CPUSTAT_RESET = 0,
#ifdef CONFIG_ARCH_OMAP1
	CPUSTAT_GBL_IDLE,
	CPUSTAT_CPU_IDLE,
#endif
	CPUSTAT_RUN,
	CPUSTAT_MAX
};

int dsp_set_rstvect(dsp_long_t adr);
dsp_long_t dsp_get_rstvect(void);
void dsp_set_idle_boot_base(dsp_long_t adr, size_t size);
void dsp_reset_idle_boot_base(void);
void dsp_cpustat_request(enum cpustat_e req);
enum cpustat_e dsp_cpustat_get_stat(void);
u16 dsp_cpustat_get_icrmask(void);
void dsp_cpustat_set_icrmask(u16 mask);
void dsp_register_mem_cb(int (*req_cb)(void), void (*rel_cb)(void));
void dsp_unregister_mem_cb(void);

#if defined(CONFIG_ARCH_OMAP1)
#define command_dvfs_stop(m) (0)
#define command_dvfs_start(m) (0)
#elif defined(CONFIG_ARCH_OMAP2)
#define command_dvfs_stop(m) \
	(((m)->cmd_l == KFUNC_POWER) && ((m)->data == DVFS_STOP))
#define command_dvfs_start(m) \
	(((m)->cmd_l == KFUNC_POWER) && ((m)->data == DVFS_START))
#endif

extern struct omap_dsp *omap_dsp;

extern int dsp_late_init(void);

#endif /* __PLAT_OMAP_DSP_DSP_H */
