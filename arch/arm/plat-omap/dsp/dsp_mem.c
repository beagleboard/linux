/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * Conversion to mempool API and ARM MMU section mapping
 * by Paul Mundt <paul.mundt@nokia.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mempool.h>
#include <linux/clk.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/arch/tc.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/dsp.h>
#include <asm/arch/mailbox.h>
#include <asm/arch/mmu.h>
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ipbuf.h"

#if defined(CONFIG_ARCH_OMAP1)
#include "../../mach-omap1/mmu.h"
#elif defined(CONFIG_ARCH_OMAP2)
#include "../../mach-omap2/mmu.h"
#endif

#include "mmu.h"

static struct mem_sync_struct mem_sync;

int dsp_mem_sync_inc(void)
{
	if (dsp_mem_enable((void *)dspmem_base) < 0)
		return -1;
	if (mem_sync.DARAM)
		mem_sync.DARAM->ad_arm++;
	if (mem_sync.SARAM)
		mem_sync.SARAM->ad_arm++;
	if (mem_sync.SDRAM)
		mem_sync.SDRAM->ad_arm++;
	dsp_mem_disable((void *)dspmem_base);

	return 0;
}

/*
 * dsp_mem_sync_config() is called from mbox1 workqueue
 */
int dsp_mem_sync_config(struct mem_sync_struct *sync)
{
	size_t sync_seq_sz = sizeof(struct sync_seq);

#ifdef OLD_BINARY_SUPPORT
	if (sync == NULL) {
		memset(&mem_sync, 0, sizeof(struct mem_sync_struct));
		return 0;
	}
#endif
	if ((dsp_mem_type(sync->DARAM, sync_seq_sz) != MEM_TYPE_DARAM) ||
	    (dsp_mem_type(sync->SARAM, sync_seq_sz) != MEM_TYPE_SARAM) ||
	    (dsp_mem_type(sync->SDRAM, sync_seq_sz) != MEM_TYPE_EXTERN)) {
		printk(KERN_ERR
		       "omapdsp: mem_sync address validation failure!\n"
		       "  mem_sync.DARAM = 0x%p,\n"
		       "  mem_sync.SARAM = 0x%p,\n"
		       "  mem_sync.SDRAM = 0x%p,\n",
		       sync->DARAM, sync->SARAM, sync->SDRAM);
		return -1;
	}

	memcpy(&mem_sync, sync, sizeof(struct mem_sync_struct));

	return 0;
}


enum dsp_mem_type_e dsp_mem_type(void *vadr, size_t len)
{
	void *ds = (void *)daram_base;
	void *de = (void *)daram_base + daram_size;
	void *ss = (void *)saram_base;
	void *se = (void *)saram_base + saram_size;
	int ret;

	if ((vadr >= ds) && (vadr < de)) {
		if (vadr + len > de)
			return MEM_TYPE_CROSSING;
		else
			return MEM_TYPE_DARAM;
	} else if ((vadr >= ss) && (vadr < se)) {
		if (vadr + len > se)
			return MEM_TYPE_CROSSING;
		else
			return MEM_TYPE_SARAM;
	} else {
		down_read(&dsp_mmu.exmap_sem);
		if (exmap_valid(&dsp_mmu, vadr, len))
			ret = MEM_TYPE_EXTERN;
		else
			ret = MEM_TYPE_NONE;
		up_read(&dsp_mmu.exmap_sem);
		return ret;
	}
}

int dsp_address_validate(void *p, size_t len, char *fmt, ...)
{
	char s[64];
	va_list args;

	if (dsp_mem_type(p, len) > 0)
		return 0;

	if (fmt == NULL)
		goto out;

	va_start(args, fmt);
	vsprintf(s, fmt, args);
	va_end(args);
	printk(KERN_ERR
	       "omapdsp: %s address(0x%p) and size(0x%x) is not valid!\n"
	       "(crossing different type of memories, or external memory\n"
	       "space where no actual memory is mapped)\n", s, p, len);
 out:
	return -1;
}

#ifdef CONFIG_OMAP_DSP_FBEXPORT

static inline unsigned long lineup_offset(unsigned long adr,
					  unsigned long ref,
					  unsigned long mask)
{
	unsigned long newadr;

	newadr = (adr & ~mask) | (ref & mask);
	if (newadr < adr)
		newadr += mask + 1;
	return newadr;
}

/*
 * fb update functions:
 * fbupd_response() is executed by the workqueue.
 * fbupd_cb() is called when fb update is done, in interrupt context.
 * mbox_fbupd() is called when KFUNC:FBCTL:UPD is received from DSP.
 */
static void fbupd_response(struct work_struct *unused)
{
	int status;

	status = mbcompose_send(KFUNC, KFUNC_FBCTL, FBCTL_UPD);
	if (status == 0)
		return;

	/* FIXME: DSP is busy !! */
	printk(KERN_ERR
	       "omapdsp:"
	       "DSP is busy when trying to send FBCTL:UPD response!\n");
}

static DECLARE_WORK(fbupd_response_work, fbupd_response);

static void fbupd_cb(void *arg)
{
	schedule_work(&fbupd_response_work);
}

void mbox_fbctl_upd(void)
{
	struct omapfb_update_window win;
	volatile unsigned short *buf = ipbuf_sys_da->d;

	if (sync_with_dsp(&ipbuf_sys_da->s, TID_ANON, 5000) < 0) {
		printk(KERN_ERR "mbox: FBCTL:UPD - IPBUF sync failed!\n");
		return;
	}
	win.x = buf[0];
	win.y = buf[1];
	win.width = buf[2];
	win.height = buf[3];
	win.format = buf[4];
	release_ipbuf_pvt(ipbuf_sys_da);

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	if (!omapfb_ready) {
		printk(KERN_WARNING
		       "omapdsp: fbupd() called while HWA742 is not ready!\n");
		return;
	}
#endif
	omapfb_update_window_async(registered_fb[0], &win, fbupd_cb, NULL);
}

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
static int omapfb_notifier_cb(struct notifier_block *omapfb_nb,
			      unsigned long event, void *fbi)
{
	pr_info("omapfb_notifier_cb(): event = %s\n",
		(event == OMAPFB_EVENT_READY)    ? "READY" :
		(event == OMAPFB_EVENT_DISABLED) ? "DISABLED" : "Unknown");
	if (event == OMAPFB_EVENT_READY)
		omapfb_ready = 1;
	else if (event == OMAPFB_EVENT_DISABLED)
		omapfb_ready = 0;
	return 0;
}
#endif

static int dsp_fbexport(dsp_long_t *dspadr)
{
	dsp_long_t dspadr_actual;
	unsigned long padr_sys, padr, fbsz_sys, fbsz;
	int cnt;
#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	int status;
#endif

	pr_debug( "omapdsp: frame buffer export\n");

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	if (omapfb_nb) {
		printk(KERN_WARNING
		       "omapdsp: frame buffer has been exported already!\n");
		return -EBUSY;
	}
#endif

	if (num_registered_fb == 0) {
		pr_info("omapdsp: frame buffer not registered.\n");
		return -EINVAL;
	}
	if (num_registered_fb != 1) {
		pr_info("omapdsp: %d frame buffers found. we use first one.\n",
			num_registered_fb);
	}
	padr_sys = registered_fb[0]->fix.smem_start;
	fbsz_sys = registered_fb[0]->fix.smem_len;
	if (fbsz_sys == 0) {
		printk(KERN_ERR
		       "omapdsp: framebuffer doesn't seem to be configured "
		       "correctly! (size=0)\n");
		return -EINVAL;
	}

	/*
	 * align padr and fbsz to 4kB boundary
	 * (should be noted to the user afterwards!)
	 */
	padr = padr_sys & ~(SZ_4K-1);
	fbsz = (fbsz_sys + padr_sys - padr + SZ_4K-1) & ~(SZ_4K-1);

	/* line up dspadr offset with padr */
	dspadr_actual =
		(fbsz > SZ_1M) ?  lineup_offset(*dspadr, padr, SZ_1M-1) :
		(fbsz > SZ_64K) ? lineup_offset(*dspadr, padr, SZ_64K-1) :
		/* (fbsz > SZ_4KB) ? */ *dspadr;
	if (dspadr_actual != *dspadr)
		pr_debug(
			"omapdsp: actual dspadr for FBEXPORT = %08x\n",
			dspadr_actual);
	*dspadr = dspadr_actual;

	cnt = omap_mmu_exmap(&dsp_mmu, dspadr_actual, padr, fbsz,
			     EXMAP_TYPE_FB);
	if (cnt < 0) {
		printk(KERN_ERR "omapdsp: exmap failure.\n");
		return cnt;
	}

	if ((padr != padr_sys) || (fbsz != fbsz_sys)) {
		printk(KERN_WARNING
"  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
"  !!  screen base address or size is not aligned in 4kB:           !!\n"
"  !!    actual screen  adr = %08lx, size = %08lx                   !!\n"
"  !!    exporting      adr = %08lx, size = %08lx                   !!\n"
"  !!  Make sure that the framebuffer is allocated with 4kB-order!  !!\n"
"  !!  Otherwise DSP can corrupt the kernel memory.                 !!\n"
"  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
		       padr_sys, fbsz_sys, padr, fbsz);
	}

	/* increase the DMA priority */
	set_emiff_dma_prio(15);

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	omapfb_nb = kzalloc(sizeof(struct omapfb_notifier_block), GFP_KERNEL);
	if (omapfb_nb == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to allocate memory for omapfb_nb!\n");
		omap_mmu_exunmap(&dsp_mmu, (unsigned long)dspadr);
		return -ENOMEM;
	}

	status = omapfb_register_client(omapfb_nb, omapfb_notifier_cb, NULL);
	if (status)
		pr_info("omapfb_register_client(): failure(%d)\n", status);
#endif

	return cnt;
}
#else
void mbox_fbctl_upd(void) { }
#endif

/* dsp/mem fops: backward compatibility */
static ssize_t dsp_mem_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	return __omap_mmu_mem_read(&dsp_mmu, (char __user *)buf, *ppos, count);
}

static ssize_t dsp_mem_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	return __omap_mmu_mem_write(&dsp_mmu,
				    (char __user *)buf, *ppos, count);
}

static int dsp_mem_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct omap_dsp_mapinfo mapinfo;
	__u32 size;

	switch (cmd) {
	case MEM_IOCTL_MMUINIT:
		if (dsp_mmu.exmap_tbl)
			omap_mmu_unregister(&dsp_mmu);
		dsp_mem_ipi_init();
		return omap_mmu_register(&dsp_mmu);

	case MEM_IOCTL_EXMAP:
		if (copy_from_user(&mapinfo, (void __user *)arg,
				   sizeof(mapinfo)))
			return -EFAULT;
		return omap_mmu_exmap(&dsp_mmu, mapinfo.dspadr,
				      0, mapinfo.size, EXMAP_TYPE_MEM);

	case MEM_IOCTL_EXUNMAP:
		return omap_mmu_exunmap(&dsp_mmu, (unsigned long)arg);

	case MEM_IOCTL_EXMAP_FLUSH:
		omap_mmu_exmap_flush(&dsp_mmu);
		return 0;
#ifdef CONFIG_OMAP_DSP_FBEXPORT
	case MEM_IOCTL_FBEXPORT:
	{
		dsp_long_t dspadr;
		int ret;
		if (copy_from_user(&dspadr, (void __user *)arg,
				   sizeof(dsp_long_t)))
			return -EFAULT;
		ret = dsp_fbexport(&dspadr);
		if (copy_to_user((void __user *)arg, &dspadr,
				 sizeof(dsp_long_t)))
			return -EFAULT;
		return ret;
	}
#endif
	case MEM_IOCTL_MMUITACK:
		return dsp_mmu_itack();

	case MEM_IOCTL_KMEM_RESERVE:

		if (copy_from_user(&size, (void __user *)arg,
				   sizeof(__u32)))
			return -EFAULT;
		return omap_mmu_kmem_reserve(&dsp_mmu, size);


	case MEM_IOCTL_KMEM_RELEASE:
		omap_mmu_kmem_release();
		return 0;

	default:
		return -ENOIOCTLCMD;
	}
}

struct file_operations dsp_mem_fops = {
	.owner   = THIS_MODULE,
	.read	 = dsp_mem_read,
	.write	 = dsp_mem_write,
	.ioctl   = dsp_mem_ioctl,
};

void dsp_mem_start(void)
{
	dsp_register_mem_cb(intmem_enable, intmem_disable);
}

void dsp_mem_stop(void)
{
	memset(&mem_sync, 0, sizeof(struct mem_sync_struct));
	dsp_unregister_mem_cb();
}

static void dsp_mmu_irq_work(struct work_struct *work)
{
	struct omap_mmu *mmu = container_of(work, struct omap_mmu, irq_work);

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY) {
		dsp_err_set(ERRCODE_MMU, mmu->fault_address);
		return;
	}
	omap_mmu_itack(mmu);
	pr_info("Resetting DSP...\n");
	dsp_cpustat_request(CPUSTAT_RESET);
	omap_mmu_enable(mmu, 0);
}

/*
 * later half of dsp memory initialization
 */
int dsp_mem_late_init(void)
{
	int ret;

	dsp_mem_ipi_init();

	INIT_WORK(&dsp_mmu.irq_work, dsp_mmu_irq_work);
	ret = omap_mmu_register(&dsp_mmu);
	if (ret) {
		dsp_reset_idle_boot_base();
		goto out;
	}
	omap_dsp->mmu = &dsp_mmu;
 out:
	return ret;
}

int __init dsp_mem_init(void)
{
#ifdef CONFIG_ARCH_OMAP2
	dsp_mmu.clk    = dsp_fck_handle;
	dsp_mmu.memclk = dsp_ick_handle;
#elif defined(CONFIG_ARCH_OMAP1)
	dsp_mmu.clk    = dsp_ck_handle;
	dsp_mmu.memclk = api_ck_handle;
#endif
	return 0;
}

void dsp_mem_exit(void)
{
	dsp_reset_idle_boot_base();
	omap_mmu_unregister(&dsp_mmu);
}
