/*
 * linux/arch/arm/mach-omap/dsp/dsp_common.h
 *
 * Header for OMAP DSP driver static part
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
 * 2005/06/13:  DSP Gateway version 3.3
 */

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

#define dspword_to_virt(dw)	((void *)(dspmem_base + ((dw) << 1)))
#define dspbyte_to_virt(db)	((void *)(dspmem_base + (db)))
#define virt_to_dspword(va)	(((unsigned long)(va) - dspmem_base) >> 1)
#define virt_to_dspbyte(va)	((unsigned long)(va) - dspmem_base)
#define is_dsp_internal_mem(va) \
	(((unsigned long)(va) >= dspmem_base) &&  \
	 ((unsigned long)(va) < dspmem_base + dspmem_size))
#define is_dspbyte_internal_mem(db)	((db) < dspmem_size)
#define is_dspword_internal_mem(dw)	(((dw) << 1) < dspmem_size)

/*
 * MPUI byteswap/wordswap on/off
 *   default setting: wordswap = all, byteswap = APIMEM only
 */
#define mpui_wordswap_on() \
	do { \
		omap_writel( \
			(omap_readl(MPUI_CTRL) & ~MPUI_CTRL_WORDSWAP_MASK) | \
			MPUI_CTRL_WORDSWAP_ALL, MPUI_CTRL); \
	} while(0)

#define mpui_wordswap_off() \
	do { \
		omap_writel( \
			(omap_readl(MPUI_CTRL) & ~MPUI_CTRL_WORDSWAP_MASK) | \
			MPUI_CTRL_WORDSWAP_NONE, MPUI_CTRL); \
	} while(0)

#define mpui_byteswap_on() \
	do { \
		omap_writel( \
			(omap_readl(MPUI_CTRL) & ~MPUI_CTRL_BYTESWAP_MASK) | \
			MPUI_CTRL_BYTESWAP_API, MPUI_CTRL); \
	} while(0)

#define mpui_byteswap_off() \
	do { \
		omap_writel( \
			(omap_readl(MPUI_CTRL) & ~MPUI_CTRL_BYTESWAP_MASK) | \
			MPUI_CTRL_BYTESWAP_NONE, MPUI_CTRL); \
	} while(0)

/*
 * TC wordswap on / off
 */
#define tc_wordswap() \
	do { \
		omap_writel(TC_ENDIANISM_SWAP_WORD | TC_ENDIANISM_EN, \
			    TC_ENDIANISM); \
	} while(0)

#define tc_noswap() \
	do {  \
		omap_writel(omap_readl(TC_ENDIANISM) & ~TC_ENDIANISM_EN, \
			    TC_ENDIANISM); \
	} while(0)

/*
 * enable priority registers, EMIF, MPUI control logic
 */
#define __dsp_enable()	omap_set_bit_regw(ARM_RSTCT1_DSP_RST, ARM_RSTCT1)
#define __dsp_disable()	omap_clr_bit_regw(ARM_RSTCT1_DSP_RST, ARM_RSTCT1)
#define __dsp_run()	omap_set_bit_regw(ARM_RSTCT1_DSP_EN, ARM_RSTCT1)
#define __dsp_reset()	omap_clr_bit_regw(ARM_RSTCT1_DSP_EN, ARM_RSTCT1)

extern struct clk *dsp_ck_handle;
extern struct clk *api_ck_handle;
extern unsigned long dspmem_base, dspmem_size,
		     daram_base, daram_size,
		     saram_base, saram_size;

enum e_cpustat {
	CPUSTAT_RESET = 0,
	CPUSTAT_GBL_IDLE = 1,
	CPUSTAT_CPU_IDLE = 2,
	CPUSTAT_RUN = 3
};

#define cpustat_name(stat) \
	((stat == CPUSTAT_RESET)    ? "RESET" :\
	 (stat == CPUSTAT_GBL_IDLE) ? "GBL_IDLE" :\
	 (stat == CPUSTAT_CPU_IDLE) ? "CPU_IDLE" :\
	 (stat == CPUSTAT_RUN)      ? "RUN" :\
	 			      "unknown")

int dsp_set_rstvect(unsigned long adr);
void dsp_set_idle_boot_base(unsigned long adr, size_t size);
void dsp_cpustat_request(enum e_cpustat req);
enum e_cpustat dsp_cpustat_get_stat(void);
unsigned short dsp_cpustat_get_icrmask(void);
void dsp_cpustat_set_icrmask(unsigned short mask);
void dsp_register_mem_cb(int (*req_cb)(void), void (*rel_cb)(void));
void dsp_unregister_mem_cb(void);
