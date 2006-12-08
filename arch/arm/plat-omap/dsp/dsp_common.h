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

#ifndef DRIVER_DSP_COMMON_H
#define DRIVER_DSP_COMMON_H

#include "hardware_dsp.h"

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
 */
#define RSTCTRL_RST1_DSP	0x00000001
#define RSTCTRL_RST2_DSP	0x00000002
#define __dsp_core_enable() \
	do { RM_RSTCTRL_DSP &= ~RSTCTRL_RST1_DSP; } while (0)
#define __dsp_core_disable() \
	do { RM_RSTCTRL_DSP |= RSTCTRL_RST1_DSP; } while (0)
#define __dsp_per_enable() \
	do { RM_RSTCTRL_DSP &= ~RSTCTRL_RST2_DSP; } while (0)
#define __dsp_per_disable() \
	do { RM_RSTCTRL_DSP |= RSTCTRL_RST2_DSP; } while (0)
#endif /* CONFIG_ARCH_OMAP2 */

typedef u32 dsp_long_t;	/* must have ability to carry TADD_ABORTADR */

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
#ifdef CONFIG_ARCH_OMAP1
void dsp_set_idle_boot_base(dsp_long_t adr, size_t size);
void dsp_reset_idle_boot_base(void);
#endif
void dsp_cpustat_request(enum cpustat_e req);
enum cpustat_e dsp_cpustat_get_stat(void);
u16 dsp_cpustat_get_icrmask(void);
void dsp_cpustat_set_icrmask(u16 mask);
#ifdef CONFIG_ARCH_OMAP1
void dsp_register_mem_cb(int (*req_cb)(void), void (*rel_cb)(void));
void dsp_unregister_mem_cb(void);
#endif

#if defined(CONFIG_ARCH_OMAP1)
static inline void dsp_clk_autoidle(void) {}
#elif defined(CONFIG_ARCH_OMAP2)
static inline void dsp_clk_autoidle(void)
{
	/*XXX should be handled in mach-omap[1,2] XXX*/
	PM_PWSTCTRL_DSP = (1 << 18) | (1 << 0);
	CM_AUTOIDLE_DSP |= (1 << 1);
	CM_CLKSTCTRL_DSP |= (1 << 0);
}
#endif

struct dsp_kfunc_device {
	char		*name;
	struct clk	*fck;
	struct clk	*ick;;
	struct mutex	 lock;
	int		 enabled;
	int		 type;
#define DSP_KFUNC_DEV_TYPE_COMMON	1
#define DSP_KFUNC_DEV_TYPE_AUDIO	2

	struct list_head	entry;

	int	(*probe)(struct dsp_kfunc_device *);
	int	(*remove)(struct dsp_kfunc_device *);
	int	(*enable)(struct dsp_kfunc_device *, int);
	int	(*disable)(struct dsp_kfunc_device *, int);
};

extern int dsp_kfunc_device_register(struct dsp_kfunc_device *);

struct dsp_platform_data {
	struct list_head kdev_list;
};

struct omap_dsp {
	struct mutex		lock;
	int			enabled;	/* stored peripheral status */
	int			mmu_irq;
	struct omap_mbox	*mbox;
	struct device		*dev;
	struct list_head 	*kdev_list;
	int			initialized;
};

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

#endif /* DRIVER_DSP_COMMON_H */
