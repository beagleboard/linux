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
 * 2005/06/13:  DSP Gateway version 3.3
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
unsigned long dspmem_base, dspmem_size,
	      daram_base, daram_size,
	      saram_base, saram_size;

struct cpustat {
	struct semaphore sem;
	enum e_cpustat stat;
	enum e_cpustat req;
	unsigned short icrmask;
	struct {
		int mpui;
		int mem;
		int mem_delayed;
	} usecount;
	int (*mem_req_cb)(void);
	void (*mem_rel_cb)(void);
};
struct cpustat cpustat = {
	.sem = __SEMAPHORE_INIT(cpustat.sem, 1),
	.stat = CPUSTAT_RESET,
	.icrmask = 0xffff,
};

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
#define GBL_IDLE_TEXT_SIZE	52
#define GBL_IDLE_TEXT_INIT { \
	/* SAM */ \
	0x3c, 0x4a,			/* 0x3c4a:     MOV 0x4, AR2 */ \
	0xf4, 0x41, 0xfc, 0xff,		/* 0xf441fcff: AND 0xfcff, *AR2 */ \
	/* disable WDT */ \
	0x76, 0x34, 0x04, 0xb8,		/* 0x763404b8: MOV 0x3404, AR3 */ \
	0xfb, 0x61, 0x00, 0xf5,		/* 0xfb6100f5: MOV 0x00f5, *AR3 */ \
	0x9a,				/* 0x9a:       PORT */ \
	0xfb, 0x61, 0x00, 0xa0,		/* 0xfb6100a0: MOV 0x00a0, *AR3 */ \
	0x9a,				/* 0x9a:       PORT */ \
	/* *IER0 = 0, *IER1 = 0 */ \
	0x3c, 0x0b,			/* 0x3c0b:     MOV 0x0, AR3 */ \
	0xe6, 0x61, 0x00,		/* 0xe66100:   MOV 0, *AR3 */ \
	0x76, 0x00, 0x45, 0xb8,		/* 0x76004508: MOV 0x45, AR3 */ \
	0xe6, 0x61, 0x00,		/* 0xe66100:   MOV 0, *AR3 */ \
	/* *ICR = 0xffff */ \
	0x3c, 0x1b,			/* 0x3c1b:     MOV 0x1, AR3 */ \
	0xfb, 0x61, 0xff, 0xff,		/* 0xfb61ffff: MOV 0xffff, *AR3 */ \
	0x9a,				/* 0x9a:       PORT */ \
	/* HOM */ \
	0xf5, 0x41, 0x03, 0x00,		/* 0xf5410300: OR 0x0300, *AR2 */ \
	/* idle and loop forever */ \
	0x7a, 0x00, 0x00, 0x0c,		/* 0x7a00000c: IDLE */ \
	0x4a, 0x7a,			/* 0x4a7a:     B -6 (infinite loop) */ \
	0x20, 0x20, 0x20,		/* 0x20:       NOP */ \
}

/* program size must be multiple of 2 */
#define CPU_IDLE_TEXT_SIZE	48
#define CPU_IDLE_TEXT_INIT(icrh, icrl) { \
	/* SAM */ \
	0x3c, 0x4b,			/* 0x3c4b:     MOV 0x4, AR3 */ \
	0xf4, 0x61, 0xfc, 0xff,		/* 0xf461fcff: AND 0xfcff, *AR3 */ \
	/* disable WDT */ \
	0x76, 0x34, 0x04, 0xb8,		/* 0x763404b8: MOV 0x3404, AR3 */ \
	0xfb, 0x61, 0x00, 0xf5,		/* 0xfb6100f5: MOV 0x00f5, *AR3 */ \
	0x9a,				/* 0x9a:       PORT */ \
	0xfb, 0x61, 0x00, 0xa0,		/* 0xfb6100a0: MOV 0x00a0, *AR3 */ \
	0x9a,				/* 0x9a:       PORT */ \
	/* *IER0 = 0, *IER1 = 0 */ \
	0x3c, 0x0b,			/* 0x3c0b:     MOV 0x0, AR3 */ \
	0xe6, 0x61, 0x00,		/* 0xe66100:   MOV 0, *AR3 */ \
	0x76, 0x00, 0x45, 0xb8,		/* 0x76004508: MOV 0x45, AR3 */ \
	0xe6, 0x61, 0x00,		/* 0xe66100:   MOV 0, *AR3 */ \
	/* set ICR = icr */ \
	0x3c, 0x1b,			/* 0x3c1b:     MOV AR3 0x1 */ \
	0xfb, 0x61, (icrh), (icrl),	/* 0xfb61****: MOV *AR3, icr */ \
	0x9a,				/* 0x9a:       PORT */ \
	/* idle and loop forever */ \
	0x7a, 0x00, 0x00, 0x0c,		/* 0x7a00000c: IDLE */ \
	0x4a, 0x7a,			/* 0x4a7a:     B -6 (infinite loop) */ \
	0x20, 0x20, 0x20		/* 0x20: nop */ \
}

/*
 * idle_boot base:
 * Initialized with DSP_BOOT_ADR_MPUI (=0x010000).
 * This value is used before DSP Gateway driver is initialized.
 * DSP Gateway driver will overwrite this value with other value,
 * to avoid confliction with the user program.
 */
static unsigned long idle_boot_base = DSP_BOOT_ADR_MPUI;

static void dsp_gbl_idle(void)
{
	unsigned char idle_text[GBL_IDLE_TEXT_SIZE] = GBL_IDLE_TEXT_INIT;

	__dsp_reset();
	clk_enable(api_ck_handle);

#if 0
	omap_writew(MPUI_DSP_BOOT_CONFIG_IDLE, MPUI_DSP_BOOT_CONFIG);
#endif
	simple_load_code(idle_text, dspbyte_to_virt(idle_boot_base),
			 GBL_IDLE_TEXT_SIZE);
	if (idle_boot_base == DSP_BOOT_ADR_MPUI)
		omap_writew(MPUI_DSP_BOOT_CONFIG_MPUI, MPUI_DSP_BOOT_CONFIG);
	else
		dsp_set_rstvect(idle_boot_base);

	__dsp_run();
	udelay(100);	/* to make things stable */
	clk_disable(api_ck_handle);
}

static void dsp_cpu_idle(void)
{
	unsigned short icr_tmp;
	unsigned char icrh, icrl;

	__dsp_reset();
	clk_enable(api_ck_handle);

	/*
	 * icr settings:
	 * DMA should not sleep for DARAM/SARAM access
	 * DPLL should not sleep while any other domain is active
	 */
	icr_tmp = cpustat.icrmask & ~(DSPREG_ICR_DMA_IDLE_DOMAIN |
				      DSPREG_ICR_DPLL_IDLE_DOMAIN);
	icrh = icr_tmp >> 8;
	icrl = icr_tmp & 0xff;
	{
		unsigned char idle_text[CPU_IDLE_TEXT_SIZE] = CPU_IDLE_TEXT_INIT(icrh, icrl);
		simple_load_code(idle_text, dspbyte_to_virt(idle_boot_base),
				 CPU_IDLE_TEXT_SIZE);
	}
	if (idle_boot_base == DSP_BOOT_ADR_MPUI)
		omap_writew(MPUI_DSP_BOOT_CONFIG_MPUI, MPUI_DSP_BOOT_CONFIG);
	else
		dsp_set_rstvect(idle_boot_base);
	__dsp_run();
	udelay(100);	/* to make things stable */
	clk_disable(api_ck_handle);
}

void dsp_set_idle_boot_base(unsigned long adr, size_t size)
{
	if (adr == idle_boot_base)
		return;
	idle_boot_base = adr;
	if ((size < GBL_IDLE_TEXT_SIZE) ||
	    (size < CPU_IDLE_TEXT_SIZE)) {
		printk(KERN_ERR
		       "omapdsp: size for idle program is not enough!\n");
		BUG();
	}

	/* restart idle program with new base address */
	if (cpustat.stat == CPUSTAT_GBL_IDLE)
		dsp_gbl_idle();
	if (cpustat.stat == CPUSTAT_CPU_IDLE)
		dsp_cpu_idle();
}

static int init_done;

static int __init omap_dsp_init(void)
{
	dspmem_size = 0;
#ifdef CONFIG_ARCH_OMAP15XX
	if (cpu_is_omap1510()) {
		dspmem_base = OMAP1510_DSP_BASE;
		dspmem_size = OMAP1510_DSP_SIZE;
		daram_base = OMAP1510_DARAM_BASE;
		daram_size = OMAP1510_DARAM_SIZE;
		saram_base = OMAP1510_SARAM_BASE;
		saram_size = OMAP1510_SARAM_SIZE;
	}
#endif
#ifdef CONFIG_ARCH_OMAP16XX
	if (cpu_is_omap16xx()) {
		dspmem_base = OMAP16XX_DSP_BASE;
		dspmem_size = OMAP16XX_DSP_SIZE;
		daram_base = OMAP16XX_DARAM_BASE;
		daram_size = OMAP16XX_DARAM_SIZE;
		saram_base = OMAP16XX_SARAM_BASE;
		saram_size = OMAP16XX_SARAM_SIZE;
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

static void dsp_cpustat_update(void)
{
	if (!init_done)
		omap_dsp_init();

	if (cpustat.req == CPUSTAT_RUN) {
		if (cpustat.stat < CPUSTAT_RUN) {
			__dsp_reset();
			clk_enable(api_ck_handle);
			udelay(10);
			__dsp_run();
			cpustat.stat = CPUSTAT_RUN;
			enable_irq(INT_DSP_MMU);
		}
		return;
	}

	/* cpustat.stat < CPUSTAT_RUN */

	if (cpustat.stat == CPUSTAT_RUN) {
		disable_irq(INT_DSP_MMU);
		clk_disable(api_ck_handle);
	}

	/*
	 * (1) when ARM wants DARAM access, MPUI should be SAM and
	 *     DSP needs to be on.
	 * (2) if any bits of icr is masked, we can not enter global idle.
	 */
	if ((cpustat.req == CPUSTAT_CPU_IDLE) ||
	    (cpustat.usecount.mem > 0) ||
	    (cpustat.usecount.mem_delayed > 0) ||
	    ((cpustat.usecount.mpui > 0) && (cpustat.icrmask != 0xffff))) {
		if (cpustat.stat != CPUSTAT_CPU_IDLE) {
			dsp_cpu_idle();
			cpustat.stat = CPUSTAT_CPU_IDLE;
		}
		return;
	}

	/*
	 * when ARM only needs MPUI access, MPUI can be HOM and
	 * DSP can be idling.
	 */
	if ((cpustat.req == CPUSTAT_GBL_IDLE) ||
	    (cpustat.usecount.mpui > 0)) {
		if (cpustat.stat != CPUSTAT_GBL_IDLE) {
			dsp_gbl_idle();
			cpustat.stat = CPUSTAT_GBL_IDLE;
		}
		return;
	}

	/*
	 * no user, no request
	 */
	if (cpustat.stat != CPUSTAT_RESET) {
		__dsp_reset();
		cpustat.stat = CPUSTAT_RESET;
	}
}

void dsp_cpustat_request(enum e_cpustat req)
{
	down(&cpustat.sem);
	cpustat.req = req;
	dsp_cpustat_update();
	up(&cpustat.sem);
}

enum e_cpustat dsp_cpustat_get_stat(void)
{
	return cpustat.stat;
}

unsigned short dsp_cpustat_get_icrmask(void)
{
	return cpustat.icrmask;
}

void dsp_cpustat_set_icrmask(unsigned short mask)
{
	down(&cpustat.sem);
	cpustat.icrmask = mask;
	dsp_cpustat_update();
	up(&cpustat.sem);
}

void omap_dsp_request_mpui(void)
{
	down(&cpustat.sem);
	if (cpustat.usecount.mpui++ == 0)
		dsp_cpustat_update();
	up(&cpustat.sem);
}

void omap_dsp_release_mpui(void)
{
	down(&cpustat.sem);
	if (cpustat.usecount.mpui-- == 0) {
		printk(KERN_ERR
		       "omapdsp: unbalanced mpui request/release detected.\n"
		       "         cpustat.usecount.mpui is going to be "
		       "less than zero! ... fixed to be zero.\n");
		cpustat.usecount.mpui = 0;
	}
	if (cpustat.usecount.mpui == 0)
		dsp_cpustat_update();
	up(&cpustat.sem);
}

int omap_dsp_request_mem(void)
{
	int ret = 0;

	down(&cpustat.sem);
	if ((cpustat.usecount.mem++ == 0) &&
	    (cpustat.usecount.mem_delayed == 0)) {
		if (cpustat.mem_req_cb) {
			if ((ret = cpustat.mem_req_cb()) < 0) {
				cpustat.usecount.mem--;
				goto out;
			}
		}
		dsp_cpustat_update();
	}
out:
	up(&cpustat.sem);

	return ret;
}

/*
 * release_mem will be delayed.
 */
static void do_release_mem(void) {
	down(&cpustat.sem);
	cpustat.usecount.mem_delayed = 0;
	if (cpustat.usecount.mem == 0) {
		dsp_cpustat_update();
		if (cpustat.mem_rel_cb)
			cpustat.mem_rel_cb();
	}
	up(&cpustat.sem);
}

static DECLARE_WORK(mem_rel_work, (void (*)(void *))do_release_mem, NULL);

int omap_dsp_release_mem(void)
{
	down(&cpustat.sem);

	/* cancel previous release work */
	cancel_delayed_work(&mem_rel_work);
	cpustat.usecount.mem_delayed = 0;

	if (cpustat.usecount.mem-- == 0) {
		printk(KERN_ERR
		       "omapdsp: unbalanced memory request/release detected.\n"
		       "         cpustat.usecount.mem is going to be "
		       "less than zero! ... fixed to be zero.\n");
		cpustat.usecount.mem = 0;
	}
	if (cpustat.usecount.mem == 0) {
		cpustat.usecount.mem_delayed = 1;
		schedule_delayed_work(&mem_rel_work, HZ);
	}

	up(&cpustat.sem);

	return 0;
}

void dsp_register_mem_cb(int (*req_cb)(void), void (*rel_cb)(void))
{
	down(&cpustat.sem);

	cpustat.mem_req_cb = req_cb;
	cpustat.mem_rel_cb = rel_cb;

	/*
	 * This function must be called while mem is enabled!
	 */
	BUG_ON(cpustat.usecount.mem == 0);

	up(&cpustat.sem);
}

void dsp_unregister_mem_cb(void)
{
	down(&cpustat.sem);
	cpustat.mem_req_cb = NULL;
	cpustat.mem_rel_cb = NULL;
	up(&cpustat.sem);
}

arch_initcall(omap_dsp_init);

EXPORT_SYMBOL(omap_dsp_request_mpui);
EXPORT_SYMBOL(omap_dsp_release_mpui);
EXPORT_SYMBOL(omap_dsp_request_mem);
EXPORT_SYMBOL(omap_dsp_release_mem);

#ifdef CONFIG_OMAP_DSP_MODULE
EXPORT_SYMBOL(dsp_ck_handle);
EXPORT_SYMBOL(api_ck_handle);
EXPORT_SYMBOL(dspmem_base);
EXPORT_SYMBOL(dspmem_size);
EXPORT_SYMBOL(daram_base);
EXPORT_SYMBOL(daram_size);
EXPORT_SYMBOL(saram_base);
EXPORT_SYMBOL(saram_size);
EXPORT_SYMBOL(dsp_set_rstvect);
EXPORT_SYMBOL(dsp_set_idle_boot_base);
EXPORT_SYMBOL(dsp_cpustat_request);
EXPORT_SYMBOL(dsp_cpustat_get_stat);
EXPORT_SYMBOL(dsp_cpustat_get_icrmask);
EXPORT_SYMBOL(dsp_cpustat_set_icrmask);
EXPORT_SYMBOL(dsp_register_mem_cb);
EXPORT_SYMBOL(dsp_unregister_mem_cb);

EXPORT_SYMBOL(__cpu_flush_kern_tlb_range);
#endif
