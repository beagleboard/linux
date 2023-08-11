/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2021 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2021 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


#ifndef __gc_hal_kernel_linux_h_
#define __gc_hal_kernel_linux_h_

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
#include <linux/iommu.h>
#include <linux/iova.h>
#endif

#include <linux/idr.h>

#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif
#include <asm/io.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,7,0)
    #include <linux/uaccess.h>
#else
    #include <asm/uaccess.h>
#endif

#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
#include <linux/clk.h>
#endif

#define NTSTRSAFE_NO_CCH_FUNCTIONS
#include "gc_hal.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_device.h"
#include "gc_hal_kernel_os.h"
#include "gc_hal_kernel_debugfs.h"
#include "gc_hal_ta.h"


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#define FIND_TASK_BY_PID(x) pid_task(find_vpid(x), PIDTYPE_PID)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#define FIND_TASK_BY_PID(x) find_task_by_vpid(x)
#else
#define FIND_TASK_BY_PID(x) find_task_by_pid(x)
#endif

#ifndef DEVICE_NAME
#   define DEVICE_NAME              "galcore"
#endif

#ifndef CLASS_NAME
#   define CLASS_NAME               "graphics_class"
#endif

#define GetPageCount(size, offset)     ((((size) + ((offset) & ~PAGE_MASK)) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,7,0)
#define gcdVM_FLAGS (VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP)
#else
#define gcdVM_FLAGS (VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_RESERVED)
#endif

/* Protection bit when mapping memroy to user sapce */
#define gcmkPAGED_MEMROY_PROT(x)    pgprot_writecombine(x)

#define gcdSUPPRESS_OOM_MESSAGE 1

#if gcdSUPPRESS_OOM_MESSAGE
#define gcdNOWARN __GFP_NOWARN
#else
#define gcdNOWARN 0
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION (4, 1, 0)
#ifdef gcdIRQ_SHARED
#       define gcdIRQF_FLAG   (IRQF_SHARED | IRQF_TRIGGER_HIGH)
#   else
#       define gcdIRQF_FLAG   (IRQF_TRIGGER_HIGH)
#   endif
#else
#ifdef gcdIRQ_SHARED
#       define gcdIRQF_FLAG   (IRQF_DISABLED | IRQF_SHARED | IRQF_TRIGGER_HIGH)
#   else
#       define gcdIRQF_FLAG   (IRQF_DISABLED | IRQF_TRIGGER_HIGH)
#   endif
#endif

/* gcdLINUX_SYNC_FILE and CONFIG_SYNC_FILE. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
#  define dma_fence                         fence
#  define dma_fence_array                   fence_array
#  define dma_fence_ops                     fence_ops

#  define dma_fence_default_wait            fence_default_wait

#  define dma_fence_signal(f)               fence_signal(f)
#  define dma_fence_signal_locked(f)        fence_signal_locked(f)
#  define dma_fence_get(f)                  fence_get(f)
#  define dma_fence_put(f)                  fence_put(f)
#  define dma_fence_is_array(f)             fence_is_array(f)
#  define dma_fence_is_signaled(f)          fence_is_signaled(f)
#  define to_dma_fence_array(f)             to_fence_array(f)
#  define dma_fence_wait_timeout(f, n, t)   fence_wait_timeout((f), (n), (t))
#  define dma_fence_init(f, o, l, t, s)     fence_init((f), (o), (l), (t), (s))
#  define dma_fence_context_alloc(s)        fence_context_alloc(s)

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
#define current_mm_mmap_sem current->mm->mmap_lock
#else
#define current_mm_mmap_sem current->mm->mmap_sem
#endif

#ifndef untagged_addr
#define untagged_addr(addr) (addr)
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION (4,20,17) && !defined(CONFIG_ARCH_NO_SG_CHAIN)) ||   \
    (LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0)       \
    && (defined(ARCH_HAS_SG_CHAIN) || defined(CONFIG_ARCH_HAS_SG_CHAIN)))
#define gcdUSE_Linux_SG_TABLE_API 1
#else
#define gcdUSE_Linux_SG_TABLE_API 0
#endif


extern struct device *galcore_device;

/******************************************************************************\
********************************** Structures **********************************
\******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
/* Just to compile without error */
struct iommu_domain
{
    void *priv;
}
#endif

typedef struct _gcsIOMMU
{
    struct iommu_domain * domain;
    struct device *       device;
    dma_addr_t            paddingPageDmaHandle;
}
gcsIOMMU;
typedef struct _gcsIOMMU * gckIOMMU;

typedef struct _gcsINTEGER_DB * gcsINTEGER_DB_PTR;
typedef struct _gcsINTEGER_DB
{
    struct idr                  idr;
    spinlock_t                  lock;
    gctINT                      curr;
}
gcsINTEGER_DB;

struct _gckOS
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to device */
    gckGALDEVICE                device;

    /* Memory management */
    struct mutex                mdlMutex;
    struct list_head            mdlHead;

    /* Kernel process ID. */
    gctUINT32                   kernelProcessID;

    /* Signal management. */

    /* Lock. */
    spinlock_t                  signalLock;

    /* signal id database. */
    gcsINTEGER_DB               signalDB;

    /* workqueue for os timer. */
    struct workqueue_struct *   workqueue;

    /* Allocate extra page to avoid cache overflow */
    struct page* paddingPage;

    /* Detect unfreed allocation. */
    atomic_t                    allocateCount;

    struct list_head            allocatorList;

    gcsDEBUGFS_DIR              allocatorDebugfsDir;

    /* Lock for register access check. */
    spinlock_t                  registerAccessLock;

    /* External power states. */
    gctBOOL                     powerStates[gcdMAX_GPU_COUNT];

    /* External clock states. */
    gctBOOL                     clockStates[gcdMAX_GPU_COUNT];

    /* IOMMU. */
    gckIOMMU                    iommu;

    /* Dump in kernel. */
    struct file *               dumpFilp;
    struct mutex                dumpFilpMutex;

    int                         dumpTarget;
    char                        dumpFileName[256];
    gcsDEBUGFS_DIR              dumpDebugfsDir;
};

typedef struct _gcsSIGNAL * gcsSIGNAL_PTR;
typedef struct _gcsSIGNAL
{
    /* Kernel sync primitive. */
    volatile unsigned int done;
    spinlock_t lock;

    wait_queue_head_t wait;

    /* Manual reset flag. */
    gctBOOL manualReset;

    /* The reference counter. */
    atomic_t ref;

    /* The owner of the signal. */
    gctHANDLE process;

    /* ID. */
    gctUINT32 id;

#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
    /* Parent timeline. */
    struct sync_timeline * timeline;
#  else
    struct dma_fence *fence;
#  endif
#endif
}
gcsSIGNAL;

typedef struct _gcsOSTIMER * gcsOSTIMER_PTR;
typedef struct _gcsOSTIMER
{
    struct delayed_work     work;
    gctTIMERFUNCTION        function;
    gctPOINTER              data;
} gcsOSTIMER;

gceSTATUS
gckOS_ImportAllocators(
    gckOS Os
    );

gceSTATUS
gckOS_FreeAllocators(
    gckOS Os
    );

gceSTATUS
_ConvertLogical2Physical(
    IN gckOS Os,
    IN gctPOINTER Logical,
    IN gctUINT32 ProcessID,
    IN PLINUX_MDL Mdl,
    OUT gctPHYS_ADDR_T * Physical
    );

gceSTATUS
_QuerySignal(
    IN gckOS Os,
    IN gctSIGNAL Signal
    );

static inline gctINT
_GetProcessID(
    void
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    return task_tgid_vnr(current);
#else
    return current->tgid;
#endif
}

static inline void
_MemoryBarrier(
    void
    )
{
#if defined(CONFIG_ARM) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34))
    dsb();
#else
    mb();
#endif
}

static inline void
_Barrier(
    void
    )
{
    barrier();
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
static inline int
is_vmalloc_addr(
    void *Addr
    )
{
    unsigned long addr = (unsigned long)Addr;

    return addr >= VMALLOC_START && addr < VMALLOC_END;
}
#endif

void
gckIOMMU_Destory(
    IN gckOS Os,
    IN gckIOMMU Iommu
    );

gceSTATUS
gckIOMMU_Construct(
    IN gckOS Os,
    OUT gckIOMMU * Iommu
    );

#endif /* __gc_hal_kernel_linux_h_ */
