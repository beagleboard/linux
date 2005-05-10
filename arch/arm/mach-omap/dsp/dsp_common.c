/*
 * linux/arch/arm/mach-omap/dsp/dsp_common.c
 *
 * OMAP DSP driver static part
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
 * 2004/11/19:  DSP Gateway version 3.2
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/tlbflush.h>
#include <asm/irq.h>
#include <asm/arch/dsp.h>
#include <asm/arch/tc.h>
#include <asm/hardware/clock.h>
#include "dsp_common.h"

struct clk *dsp_ck_handle;
struct clk *api_ck_handle;
unsigned long dspmem_base, dspmem_size;
int dsp_runstat = RUNSTAT_RESET;
unsigned short dsp_icrmask = DSPREG_ICR_EMIF_IDLE_DOMAIN |
			     DSPREG_ICR_DPLL_IDLE_DOMAIN |
			     DSPREG_ICR_PER_IDLE_DOMAIN |
			     DSPREG_ICR_CACHE_IDLE_DOMAIN |
			     DSPREG_ICR_DMA_IDLE_DOMAIN |
			     DSPREG_ICR_CPU_IDLE_DOMAIN;

int dsp_set_rstvect(unsigned long adr)
{
	unsigned long *dst_adr;

	if (adr >= DSPSPACE_SIZE)
		return -EINVAL;

	dst_adr = dspbyte_to_virt(DSP_BOOT_ADR_DIRECT);
	/* word swap */
	*dst_adr = ((adr & 0xffff) << 16) | (adr >> 16);
	/* fill 8 bytes! */
	*(dst_adr+1) = 0;
	/* direct boot */
	omap_writew(MPUI_DSP_BOOT_CONFIG_DIRECT, MPUI_DSP_BOOT_CONFIG);

	return 0;
}

static void simple_load_code(unsigned char *src_c, unsigned short *dst, int len)
{
	int i;
	unsigned short *src = (unsigned short *)src_c;
	int len_w;

	/* len must be multiple of 2. */
	if (len & 1)
		BUG();

	len_w = len / 2;
	for (i = 0; i < len_w; i++) {
		/* byte swap copy */
		*dst = ((*src & 0x00ff) << 8) |
		       ((*src & 0xff00) >> 8);
		src++;
		dst++;
	}
}

/* program size must be multiple of 2 */
#define IDLE_TEXT_SIZE	28
#define IDLE_TEXT(icr) { \
	/* disable WDT */ \
	0x76, 0x34, 0x04, 0xb8,		/* 0x763404b8: mov AR3 0x3404 */ \
	0xfb, 0x61, 0x00, 0xf5,		/* 0xfb6100f5: mov *AR3 0x00f5 */ \
	0x9a,				/* 0x9a:       port */ \
	0xfb, 0x61, 0x00, 0xa0,		/* 0xfb6100a0: mov *AR3 0x00a0 */ \
	0x9a,				/* 0x9a:       port */ \
	/* set ICR = icr */ \
	0x3c, 0x1b,			/* 0x3c1b:     mov AR3 0x1 */ \
	0xe6, 0x61, (icr),		/* 0xe661**:   mov *AR3, icr */ \
	0x9a,				/* 0x9a:       port */ \
	/* idle and loop forever */ \
	0x7a, 0x00, 0x00, 0x0c,		/* 0x7a00000c: idle */ \
	0x4a, 0x7a,			/* 0x4a7a:     b -6 (infinite loop) */ \
	0x20, 0x20			/* 0x20: nop */ \
}

/*
 * idle_boot base:
 * Initialized with DSP_BOOT_ADR_MPUI (=0x010000).
 * This value is used before DSP Gateway driver is initialized.
 * DSP Gateway driver will overwrite this value with other value,
 * to avoid confliction with the user program.
 */
static unsigned long idle_boot_base = DSP_BOOT_ADR_MPUI;

void dsp_idle(void)
{
	unsigned char icr;

	disable_irq(INT_DSP_MMU);
	preempt_disable();
	__dsp_reset();
	clk_use(api_ck_handle);

	/*
	 * icr settings:
	 * DMA should not sleep for DARAM/SARAM access
	 * DPLL should not sleep for DMA.
	 */
	icr = dsp_icrmask &
	      ~(DSPREG_ICR_DMA_IDLE_DOMAIN | DSPREG_ICR_DPLL_IDLE_DOMAIN) &
	      0xff;
	{
		unsigned char idle_text[IDLE_TEXT_SIZE] = IDLE_TEXT(icr);
		simple_load_code(idle_text, dspbyte_to_virt(idle_boot_base),
				 IDLE_TEXT_SIZE);
	}
	if (idle_boot_base == DSP_BOOT_ADR_MPUI)
		omap_writew(MPUI_DSP_BOOT_CONFIG_MPUI, MPUI_DSP_BOOT_CONFIG);
	else
		dsp_set_rstvect(idle_boot_base);
	clk_unuse(api_ck_handle);
	udelay(10);	/* to make things stable */
	__dsp_run();
	dsp_runstat = RUNSTAT_IDLE;
	preempt_enable();
	enable_irq(INT_DSP_MMU);
}

void dsp_set_idle_boot_base(unsigned long adr, size_t size)
{
	if (adr == idle_boot_base)
		return;
	idle_boot_base = adr;
	if (size < IDLE_TEXT_SIZE) {
		printk(KERN_ERR
		       "omapdsp: size for idle program is not enough!\n");
		BUG();
	}
	if (dsp_runstat == RUNSTAT_IDLE)
		dsp_idle();
}

static unsigned short save_dsp_idlect2;

/*
 * note: if we are in pm_suspend / pm_resume function,
 * we are out of clk_use() management.
 */
void omap_dsp_pm_suspend(void)
{
	unsigned short save_arm_idlect2;

	/* Reset DSP */
	__dsp_reset();

	clk_disable(dsp_ck_handle);

	/* Stop any DSP domain clocks */
	save_arm_idlect2 = omap_readw(ARM_IDLECT2); // api_ck is in ARM_IDLECT2
	clk_enable(api_ck_handle);
	save_dsp_idlect2 = __raw_readw(DSP_IDLECT2);
	__raw_writew(0, DSP_IDLECT2);
	omap_writew(save_arm_idlect2, ARM_IDLECT2);
}

void omap_dsp_pm_resume(void)
{
	unsigned short save_arm_idlect2;

	/* Restore DSP domain clocks */
	save_arm_idlect2 = omap_readw(ARM_IDLECT2); // api_ck is in ARM_IDLECT2
	clk_enable(api_ck_handle);
	__raw_writew(save_dsp_idlect2, DSP_IDLECT2);
	omap_writew(save_arm_idlect2, ARM_IDLECT2);

	/* Run DSP, if it was running */
	if (dsp_runstat != RUNSTAT_RESET)
		__dsp_run();
}
static int init_done;

static int __init omap_dsp_init(void)
{
	dspmem_size = 0;
#ifdef CONFIG_ARCH_OMAP1510
	if (cpu_is_omap1510()) {
		dspmem_base = OMAP1510_DSP_BASE;
		dspmem_size = OMAP1510_DSP_SIZE;
	}
#endif
#ifdef CONFIG_ARCH_OMAP16XX
	if (cpu_is_omap16xx()) {
		dspmem_base = OMAP16XX_DSP_BASE;
		dspmem_size = OMAP16XX_DSP_SIZE;
	}
#endif
	if (dspmem_size == 0) {
		printk(KERN_ERR "omapdsp: unsupported omap architecture.\n");
		return -ENODEV;
	}

	dsp_ck_handle = clk_get(0, "dsp_ck");
	if (IS_ERR(dsp_ck_handle)) {
		printk(KERN_ERR "omapdsp: could not acquire dsp_ck handle.\n");
		return PTR_ERR(dsp_ck_handle);
	}

	api_ck_handle = clk_get(0, "api_ck");
	if (IS_ERR(api_ck_handle)) {
		printk(KERN_ERR "omapdsp: could not acquire api_ck handle.\n");
		return PTR_ERR(api_ck_handle);
	}

	__dsp_enable();
	mpui_byteswap_off();
	mpui_wordswap_on();
	tc_wordswap();

	init_done = 1;
	return 0;
}

void omap_dsp_request_idle(void)
{
	if (dsp_runstat == RUNSTAT_RESET) {
		if (!init_done)
			omap_dsp_init();
		dsp_idle();
	}
}

arch_initcall(omap_dsp_init);

EXPORT_SYMBOL(omap_dsp_pm_suspend);
EXPORT_SYMBOL(omap_dsp_pm_resume);
EXPORT_SYMBOL(omap_dsp_request_idle);

#ifdef CONFIG_OMAP_DSP_MODULE
EXPORT_SYMBOL(dsp_ck_handle);
EXPORT_SYMBOL(api_ck_handle);
EXPORT_SYMBOL(dspmem_base);
EXPORT_SYMBOL(dspmem_size);
EXPORT_SYMBOL(dsp_runstat);
EXPORT_SYMBOL(dsp_icrmask);
EXPORT_SYMBOL(dsp_set_rstvect);
EXPORT_SYMBOL(dsp_idle);
EXPORT_SYMBOL(dsp_set_idle_boot_base);

EXPORT_SYMBOL(__cpu_flush_kern_tlb_range);
#endif
