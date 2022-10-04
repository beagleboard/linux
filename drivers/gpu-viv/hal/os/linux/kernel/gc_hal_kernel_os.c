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


#include "gc_hal_kernel_linux.h"
#include "gc_hal_dump.h"

#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/irqflags.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
#include <linux/math64.h>
#endif
#include <linux/delay.h>
#include <linux/platform_device.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/anon_inodes.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,5,0)
#include <linux/io.h>
#endif

#if gcdLINUX_SYNC_FILE
#  include <linux/file.h>
#  include "gc_hal_kernel_sync.h"
#endif

#if defined(CONFIG_DMA_SHARED_BUFFER)
#include <linux/dma-buf.h>
#endif

#define _GC_OBJ_ZONE    gcvZONE_OS

#include "gc_hal_kernel_allocator.h"

#define gcmkBUG_ON(x) \
    do { \
        if (unlikely(!!(x))) \
        { \
            printk("[galcore]: BUG ON @ %s(%d)\n", __func__, __LINE__); \
            dump_stack(); \
        } \
    } while (0)

/******************************************************************************\
******************************* Private Functions ******************************
\******************************************************************************/
static gctINT
_GetThreadID(
    void
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    return task_pid_vnr(current);
#else
    return current->pid;
#endif
}

/* Must hold Mdl->mpasMutex before call this function. */
static inline PLINUX_MDL_MAP
_CreateMdlMap(
    IN PLINUX_MDL Mdl,
    IN gctINT ProcessID
    )
{
    PLINUX_MDL_MAP mdlMap = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Mdl=%p ProcessID=%d", Mdl, ProcessID);

    mdlMap = (PLINUX_MDL_MAP)kmalloc(sizeof(struct _LINUX_MDL_MAP), GFP_KERNEL | gcdNOWARN);
    if (mdlMap == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    mdlMap->pid     = ProcessID;
    mdlMap->vmaAddr = gcvNULL;
    mdlMap->count   = 0;

    list_add(&mdlMap->link, &Mdl->mapsHead);

OnError:
    gcmkFOOTER_ARG("ret=%p", mdlMap);
    return mdlMap;
}

/* Must hold Mdl->mpasMutex before call this function. */
static inline gceSTATUS
_DestroyMdlMap(
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap
    )
{
    gcmkHEADER_ARG("Mdl=%p MdlMap=%p", Mdl, MdlMap);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(MdlMap != gcvNULL);

    list_del(&MdlMap->link);
    kfree(MdlMap);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/* Must hold Mdl->mpasMutex before call this function. */
extern PLINUX_MDL_MAP
FindMdlMap(
    IN PLINUX_MDL Mdl,
    IN gctINT ProcessID
    )
{
    PLINUX_MDL_MAP mdlMap = gcvNULL;

    gcmkHEADER_ARG("Mdl=%p ProcessID=%d", Mdl, ProcessID);

    if (Mdl)
    {
        PLINUX_MDL_MAP iter = gcvNULL;
        list_for_each_entry(iter, &Mdl->mapsHead, link)
        {
            if (iter->pid == ProcessID)
            {
                mdlMap = iter;
                break;
            }
        }
    }

    gcmkFOOTER_ARG("ret=%p", mdlMap);
    return mdlMap;
}


static PLINUX_MDL
_CreateMdl(
    IN gckOS Os
    )
{
    PLINUX_MDL mdl;

    gcmkHEADER();

    mdl = (PLINUX_MDL)kzalloc(sizeof(struct _LINUX_MDL), GFP_KERNEL | gcdNOWARN);

    if (mdl)
    {
        mdl->os = Os;
        atomic_set(&mdl->refs, 1);
        mutex_init(&mdl->mapsMutex);
        INIT_LIST_HEAD(&mdl->mapsHead);
        INIT_LIST_HEAD(&mdl->rmaHead);
    }

    gcmkFOOTER_ARG("%p", mdl);
    return mdl;
}

static gceSTATUS
_DestroyMdl(
    IN PLINUX_MDL Mdl
    )
{
    gcmkHEADER_ARG("Mdl=%p", Mdl);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Mdl != gcvNULL);

    if (atomic_dec_and_test(&Mdl->refs))
    {
        gckOS os = Mdl->os;
        gckALLOCATOR allocator = Mdl->allocator;
        PLINUX_MDL_MAP mdlMap, next;

        /* Valid private means alloc/attach successfully */
        if (Mdl->priv)
        {
            if (Mdl->addr)
            {
                gcmALLOCATOR_UnmapKernel(allocator, Mdl, Mdl->addr);
                Mdl->addr = gcvNULL;
            }
            gcmALLOCATOR_Free(allocator, Mdl);
        }

        mutex_lock(&Mdl->mapsMutex);
        list_for_each_entry_safe(mdlMap, next, &Mdl->mapsHead, link)
        {
            gcmkVERIFY_OK(_DestroyMdlMap(Mdl, mdlMap));
        }
        mutex_unlock(&Mdl->mapsMutex);

        if (Mdl->link.next)
        {
            /* Remove the node from global list.. */
            mutex_lock(&os->mdlMutex);
            list_del(&Mdl->link);
            mutex_unlock(&os->mdlMutex);
        }
        else if (Mdl->rmaLink.next)
        {
            /* Remove the sub node from root mdl */
            mutex_lock(&os->mdlMutex);
            list_del(&Mdl->rmaLink);
            mutex_unlock(&os->mdlMutex);
        }

        kfree(Mdl);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
** Integer Id Management.
*/
gceSTATUS
_AllocateIntegerId(
    IN gcsINTEGER_DB_PTR Database,
    IN gctPOINTER KernelPointer,
    OUT gctUINT32 *Id
    )
{
    int result;
    gctINT next;
    unsigned long flags = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
    idr_preload(GFP_KERNEL | gcdNOWARN);

    if(in_irq()){
        spin_lock(&Database->lock);
    }else{
        spin_lock_irqsave(&Database->lock, flags);
    }


    next = (Database->curr + 1 <= 0) ? 1 : Database->curr + 1;

    result = idr_alloc(&Database->idr, KernelPointer, next, 0, GFP_ATOMIC);

    /* ID allocated should not be 0. */
    gcmkASSERT(result != 0);

    if (result > 0)
    {
        Database->curr = *Id = result;
    }

    if(in_irq()){
        spin_unlock(&Database->lock);
    }else{
        spin_unlock_irqrestore(&Database->lock, flags);
    }


    idr_preload_end();

    if (result < 0)
    {
        return gcvSTATUS_OUT_OF_RESOURCES;
    }
#else
again:
    if (idr_pre_get(&Database->idr, GFP_KERNEL | gcdNOWARN) == 0)
    {
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    if(in_irq()){
        spin_lock(&Database->lock);
    }else{
        spin_lock_irqsave(&Database->lock, flags);
    }


    next = (Database->curr + 1 <= 0) ? 1 : Database->curr + 1;

    /* Try to get a id greater than 0. */
    result = idr_get_new_above(&Database->idr, KernelPointer, next, Id);

    if (!result)
    {
        Database->curr = *Id;
    }

    if(in_irq()){
        spin_unlock(&Database->lock);
    }else{
        spin_unlock_irqrestore(&Database->lock, flags);
    }


    if (result == -EAGAIN)
    {
        goto again;
    }

    if (result != 0)
    {
        return gcvSTATUS_OUT_OF_RESOURCES;
    }
#endif

    return gcvSTATUS_OK;
}

gceSTATUS
_QueryIntegerId(
    IN gcsINTEGER_DB_PTR Database,
    IN gctUINT32  Id,
    OUT gctPOINTER * KernelPointer
    )
{
    gctPOINTER pointer;
    unsigned long flags = 0;

    if(in_irq()){
        spin_lock(&Database->lock);
    }else{
        spin_lock_irqsave(&Database->lock, flags);
    }


    pointer = idr_find(&Database->idr, Id);

    if(in_irq()){
        spin_unlock(&Database->lock);
    }else{
        spin_unlock_irqrestore(&Database->lock, flags);
    }


    if (pointer)
    {
        *KernelPointer = pointer;
        return gcvSTATUS_OK;
    }
    else
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_OS,
                "%s(%d) Id = %d is not found",
                __FUNCTION__, __LINE__, Id);

        return gcvSTATUS_NOT_FOUND;
    }
}

gceSTATUS
_DestroyIntegerId(
    IN gcsINTEGER_DB_PTR Database,
    IN gctUINT32 Id
    )
{
    unsigned long flags = 0;

    if(in_irq()){
        spin_lock(&Database->lock);
    }else{
        spin_lock_irqsave(&Database->lock, flags);
    }


    idr_remove(&Database->idr, Id);

    if(in_irq()){
        spin_unlock(&Database->lock);
    }else{
        spin_unlock_irqrestore(&Database->lock, flags);
    }


    return gcvSTATUS_OK;
}

static inline gceSTATUS
_QueryProcessPageTable(
    IN gctPOINTER Logical,
    OUT gctPHYS_ADDR_T * Address
    )
{
    unsigned long logical = (unsigned long)Logical;
    unsigned long offset = logical & ~PAGE_MASK;

    if (is_vmalloc_addr(Logical))
    {
        /* vmalloc area. */
        *Address = page_to_phys(vmalloc_to_page(Logical)) | offset;
        return gcvSTATUS_OK;
    }
    else if (virt_addr_valid(logical))
    {
        /* Kernel logical address. */
        *Address = virt_to_phys(Logical);
        return gcvSTATUS_OK;
    }
    else
    {
        /* Try user VM area. */
        struct vm_area_struct *vma;
        spinlock_t *ptl;
        pgd_t *pgd;
#if LINUX_VERSION_CODE >= KERNEL_VERSION (5,4,0)
        p4d_t *p4d;
#endif
        pud_t *pud;
        pmd_t *pmd;
        pte_t *pte;

        if (!current->mm)
            return gcvSTATUS_NOT_FOUND;

        down_read(&current_mm_mmap_sem);
        vma = find_vma(current->mm, logical);
        up_read(&current_mm_mmap_sem);

        /* To check if mapped to user. */
        if (!vma)
            return gcvSTATUS_NOT_FOUND;

        pgd = pgd_offset(current->mm, logical);
        if (pgd_none(*pgd) || pgd_bad(*pgd))
            return gcvSTATUS_NOT_FOUND;

#if (defined(CONFIG_CPU_CSKYV2) || defined(CONFIG_X86)) \
    && LINUX_VERSION_CODE >= KERNEL_VERSION (4,12,0)
        pud = pud_offset((p4d_t*)pgd, logical);
#elif (defined(CONFIG_CPU_CSKYV2)) \
    && LINUX_VERSION_CODE >= KERNEL_VERSION (4,11,0)
        pud = pud_offset((p4d_t*)pgd, logical);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION (5,4,0)
        p4d = p4d_offset(pgd, logical);
        if (p4d_none(READ_ONCE(*p4d)))
            return gcvSTATUS_NOT_FOUND;

        pud = pud_offset(p4d, logical);
#else
        pud = pud_offset(pgd, logical);
#endif
#endif
        if (pud_none(*pud) || pud_bad(*pud))
            return gcvSTATUS_NOT_FOUND;

        pmd = pmd_offset(pud, logical);
        if (pmd_none(*pmd) || pmd_bad(*pmd))
            return gcvSTATUS_NOT_FOUND;

        pte = pte_offset_map_lock(current->mm, pmd, logical, &ptl);

        if (!pte_present(*pte))
        {
            pte_unmap_unlock(pte, ptl);
            return gcvSTATUS_NOT_FOUND;
        }

        *Address = (pte_pfn(*pte) << PAGE_SHIFT) | offset;
        pte_unmap_unlock(pte, ptl);

        return gcvSTATUS_OK;
    }
}


static gceSTATUS
_ShrinkMemory(
    IN gckOS Os
    )
{
    gcsPLATFORM * platform;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p", Os);
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    platform = Os->device->platform;

    if (platform && platform->ops->shrinkMemory)
    {
        status = platform->ops->shrinkMemory(platform);
    }
    else
    {
        status = gcvSTATUS_NOT_SUPPORTED;
    }

    gcmkFOOTER();
    return status;
}

#if gcdDUMP_IN_KERNEL

#define DUMP_TO_KERNEL_DMESG    0
#define DUMP_TO_FILE            1
#define DUMP_IGNORE             2

static void set_dump_file(gckOS os, char * fname)
{
    if (os->dumpFilp)
    {
        /* close exist opened file. */
        printk("galcore: end dump to file: %s\n", os->dumpFileName);

        filp_close(os->dumpFilp, NULL);
        os->dumpFilp = NULL;
    }

    if (fname[0] == '\0' || !strcmp(fname, "[ignored]"))
    {
        /* empty or [ignored] means ignored. */
        printk("galcore: dump ignored\n");

        os->dumpTarget = DUMP_IGNORE;
        strcpy(os->dumpFileName, "[ignored]");
    }
    else if (!strcmp(fname, "[dmesg]"))
    {
        /* [dmesg] means dump to kernel dmesg. */
        printk("galcore: dump to kernel dmesg\n");

        os->dumpTarget = DUMP_TO_KERNEL_DMESG;
        strcpy(os->dumpFileName, "[dmesg]");
    }
    else if (fname[0] != '/')
    {
        /* invalid path, switch to kernel dmesg. */
        printk(KERN_ERR "galcore: invalid path: %s\n", fname);
        printk(KERN_ERR "galcore: must be absolute path start with '/'\n");
        printk("galcore: dump to kernel dmesg\n");

        os->dumpTarget = DUMP_TO_KERNEL_DMESG;
        strcpy(os->dumpFileName, "[dmesg]");
    }
    else
    {
        /* try open file. */
        os->dumpFilp = filp_open(fname, O_RDWR | O_CREAT, 0644);

        if (IS_ERR(os->dumpFilp))
        {
            printk(KERN_ERR "galcore: failed to open file: %s\n", fname);
            printk("galcore: dump to kernel dmesg\n");

            os->dumpFilp = NULL;
            os->dumpTarget = DUMP_TO_KERNEL_DMESG;
            strcpy(os->dumpFileName, "[dmesg]");
        }
        else
        {
            printk("galcore: start dump to file: %s\n", fname);

            os->dumpTarget = DUMP_TO_FILE;
            strcpy(os->dumpFileName, fname);
        }
    }
}

static int dump_file_show(struct seq_file *m, void *unused)
{
    gcsINFO_NODE *node = m->private;
    gckOS os = node->device;

    seq_printf(m, "%s\n", os->dumpFileName);
    return 0;
}

static int dump_file_write(const char __user *buf, size_t count, void* data)
{
    gcsINFO_NODE *node = data;
    gckOS os = node->device;
    char fname[256];
    size_t len = min(count, sizeof(fname) - 1);

    if (copy_from_user(fname, buf, len))
    {
        return -EFAULT;
    }

    /* Remove tailing space. */
    while (len > 0 && (fname[len - 1] == '\n' || fname[len - 1] == ' '))
    {
        fname[len - 1] = '\0';
    }

    fname[len] = '\0';

    mutex_lock(&os->dumpFilpMutex);
    set_dump_file(os, fname);
    mutex_unlock(&os->dumpFilpMutex);

    return count;
}

static gcsINFO dumpDebugList[] =
{
    {"dump_file", dump_file_show, dump_file_write},
};

static gceSTATUS
_DumpDebugfsInit(
    IN gckOS Os
    )
{
    gceSTATUS status;
    gckGALDEVICE device = Os->device;
    gckDEBUGFS_DIR dir = &Os->dumpDebugfsDir;

    gcmkONERROR(gckDEBUGFS_DIR_Init(dir, device->debugfsDir.root, "dump"));

    gcmkONERROR(
        gckDEBUGFS_DIR_CreateFiles(dir, dumpDebugList,
                                   gcmCOUNTOF(dumpDebugList), Os));

OnError:
    return status;
}

static void
_DumpDebugfsCleanup(
    IN gckOS Os
    )
{
    gckDEBUGFS_DIR dir = &Os->dumpDebugfsDir;

    if (dir->root)
    {
        gckDEBUGFS_DIR_RemoveFiles(dir, dumpDebugList, gcmCOUNTOF(dumpDebugList));
        gckDEBUGFS_DIR_Deinit(dir);
    }
}
#endif

/*******************************************************************************
**
**  gckOS_Construct
**
**  Construct a new gckOS object.
**
**  INPUT:
**
**      gctPOINTER Context
**          Pointer to the gckGALDEVICE class.
**
**  OUTPUT:
**
**      gckOS * Os
**          Pointer to a variable that will hold the pointer to the gckOS object.
*/
gceSTATUS
gckOS_Construct(
    IN gctPOINTER Context,
    OUT gckOS * Os
    )
{
    gckOS os = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Context=%p", Context);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Os != gcvNULL);

    /* Allocate the gckOS object. */
    os = (gckOS)kmalloc(gcmSIZEOF(struct _gckOS), GFP_KERNEL | gcdNOWARN);

    if (os == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Zero the memory. */
    gckOS_ZeroMemory(os, gcmSIZEOF(struct _gckOS));

    /* Initialize the gckOS object. */
    os->object.type = gcvOBJ_OS;

    /* Set device device. */
    os->device = Context;

    /* Set allocateCount to 0, gckOS_Allocate has not been used yet. */
    atomic_set(&os->allocateCount, 0);

    /* Initialize the memory lock. */
    mutex_init(&os->mdlMutex);

    INIT_LIST_HEAD(&os->mdlHead);

    /* Get the kernel process ID. */
    os->kernelProcessID = _GetProcessID();

    /*
     * Initialize the signal manager.
     */

    /* Initialize spinlock. */
    spin_lock_init(&os->signalLock);

    /* Initialize signal id database lock. */
    spin_lock_init(&os->signalDB.lock);

    /* Initialize signal id database. */
    idr_init(&os->signalDB.idr);

    /* Create a workqueue for os timer. */
    os->workqueue = create_singlethread_workqueue("galcore workqueue");

    if (os->workqueue == gcvNULL)
    {
        /* Out of memory. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    os->paddingPage = alloc_page(GFP_KERNEL | __GFP_HIGHMEM | gcdNOWARN);
    if (os->paddingPage == gcvNULL)
    {
        /* Out of memory. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }
    else
    {
        SetPageReserved(os->paddingPage);
    }

    spin_lock_init(&os->registerAccessLock);


    /* Check iommu. */
    if (gcmIS_ERROR(gckIOMMU_Construct(os, &os->iommu)))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): Fail to setup IOMMU",
            __FUNCTION__, __LINE__
            );
    }

    gckOS_ImportAllocators(os);

#if gcdDUMP_IN_KERNEL
    mutex_init(&os->dumpFilpMutex);

    /* Set default dump file. */
    set_dump_file(os, gcdDUMP_FILE_IN_KERNEL);

    /* Init debugfs for kernel dump feature. */
    _DumpDebugfsInit(os);
#endif

    /* Return pointer to the gckOS object. */
    *Os = os;

OnError:
    if (gcmIS_ERROR(status) && os)
    {
        if (os->workqueue != gcvNULL)
        {
            destroy_workqueue(os->workqueue);
        }

        kfree(os);
        os = gcvNULL;
    }

    /* Return the error. */
    gcmkFOOTER_ARG("*Os=%p", os);
    return status;
}

/*******************************************************************************
**
**  gckOS_Destroy
**
**  Destroy an gckOS object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object that needs to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Destroy(
    IN gckOS Os
    )
{
    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    if (Os->paddingPage != gcvNULL)
    {
        ClearPageReserved(Os->paddingPage);
        __free_page(Os->paddingPage);
        Os->paddingPage = gcvNULL;
    }

    /*
     * Destroy the signal manager.
     */

    /* Wait for all works done. */
    flush_workqueue(Os->workqueue);

    /* Destory work queue. */
    destroy_workqueue(Os->workqueue);

    gckOS_FreeAllocators(Os);

    if (Os->iommu)
    {
        gckIOMMU_Destory(Os, Os->iommu);
    }

    /* Mark the gckOS object as unknown. */
    Os->object.type = gcvOBJ_UNKNOWN;


#if gcdDUMP_IN_KERNEL
    mutex_lock(&Os->dumpFilpMutex);

    if (Os->dumpFilp)
    {
        filp_close(Os->dumpFilp, NULL);
        Os->dumpFilp = NULL;
        Os->dumpTarget = DUMP_IGNORE;
    }

    mutex_unlock(&Os->dumpFilpMutex);

    /* Cleanup debugfs for kernel dump feature. */
    _DumpDebugfsCleanup(Os);
#endif

    /* Free the gckOS object. */
    kfree(Os);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_CreateKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;
    gckALLOCATOR allocator = mdl->allocator;

    gcmkHEADER_ARG("Os=%p Physical=%p Offset=0x%zx Bytes=0x%zx",
                   Os, Physical, Offset, Bytes);

    if (mdl->addr)
    {
        /* Already mapped whole memory. */
        *Logical = (gctUINT8_PTR)mdl->addr + Offset;
    }
    else
    {
        gcmkONERROR(gcmALLOCATOR_MapKernel(allocator, mdl, Offset, Bytes, Logical));
        if (Offset == 0 && Bytes == mdl->bytes)
        {
            /* the whole mdl has mapped */
            mdl->addr = *Logical;
        }
    }

OnError:
    gcmkFOOTER_ARG("*Logical=%p", gcmOPT_POINTER(Logical));
    return status;
}

gceSTATUS
gckOS_DestroyKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;
    gckALLOCATOR allocator = mdl->allocator;

    gcmkHEADER_ARG("Os=%p Physical=%p Logical=%p", Os, Physical, Logical);

    if (mdl->addr)
    {
        /* Nothing to do.
         * it will be unmpped in vidmem free
         */
    }
    else
    {
        gcmALLOCATOR_UnmapKernel(allocator, mdl, Logical);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_Allocate
**
**  Allocate memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIZE_T Bytes
**          Number of bytes to allocate.
**
**  OUTPUT:
**
**      gctPOINTER * Memory
**          Pointer to a variable that will hold the allocated memory location.
*/
gceSTATUS
gckOS_Allocate(
    IN gckOS Os,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Memory
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Bytes=0x%zx", Os, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);

    gcmkONERROR(gckOS_AllocateMemory(Os, Bytes, Memory));

OnError:
    /* Return the status. */
    gcmkFOOTER_ARG("*Memory=%p", gcmOPT_POINTER(Memory));
    return status;
}

/*******************************************************************************
**
**  gckOS_Free
**
**  Free allocated memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Memory
**          Pointer to memory allocation to free.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Free(
    IN gckOS Os,
    IN gctPOINTER Memory
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Memory=%p", Os, Memory);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);

    gcmkONERROR(gckOS_FreeMemory(Os, Memory));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_AllocateMemory
**
**  Allocate memory wrapper.
**
**  INPUT:
**
**      gctSIZE_T Bytes
**          Number of bytes to allocate.
**
**  OUTPUT:
**
**      gctPOINTER * Memory
**          Pointer to a variable that will hold the allocated memory location.
*/
gceSTATUS
gckOS_AllocateMemory(
    IN gckOS Os,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Memory
    )
{
    gctPOINTER memory = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Bytes=0x%zx", Os, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);

    if (Bytes > PAGE_SIZE)
    {
        memory = (gctPOINTER) vmalloc(Bytes);
    }
    else
    {
        memory = (gctPOINTER) kmalloc(Bytes, GFP_KERNEL | gcdNOWARN);
    }

    if (memory == gcvNULL)
    {
        /* Out of memory. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Increase count. */
    atomic_inc(&Os->allocateCount);

    /* Return pointer to the memory allocation. */
    *Memory = memory;

OnError:
    /* Return the status. */
    gcmkFOOTER_ARG("*Memory=%p", gcmOPT_POINTER(Memory));
    return status;
}

/*******************************************************************************
**
**  gckOS_FreeMemory
**
**  Free allocated memory wrapper.
**
**  INPUT:
**
**      gctPOINTER Memory
**          Pointer to memory allocation to free.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_FreeMemory(
    IN gckOS Os,
    IN gctPOINTER Memory
    )
{
    gcmkHEADER_ARG("Os=%p Memory=%p", Os, Memory);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);

    /* Free the memory from the OS pool. */
    if (is_vmalloc_addr(Memory))
    {
        vfree(Memory);
    }
    else
    {
        kfree(Memory);
    }

    /* Decrease count. */
    atomic_dec(&Os->allocateCount);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_MapMemory
**
**  Map physical memory into the current process.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Start of physical address memory.
**
**      gctSIZE_T Bytes
**          Number of bytes to map.
**
**  OUTPUT:
**
**      gctPOINTER * Memory
**          Pointer to a variable that will hold the logical address of the
**          mapped memory.
*/
gceSTATUS
gckOS_MapMemory(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    PLINUX_MDL_MAP  mdlMap;
    PLINUX_MDL      mdl = (PLINUX_MDL) Physical;
    gckALLOCATOR allocator;
    gctINT pid = _GetProcessID();

    gcmkHEADER_ARG("Os=%p Physical=%p Bytes=0x%zx", Os, Physical, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != 0);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    mutex_lock(&mdl->mapsMutex);

    mdlMap = FindMdlMap(mdl, pid);

    if (mdlMap == gcvNULL)
    {
        mdlMap = _CreateMdlMap(mdl, pid);
        if (mdlMap == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }

    if (mdlMap->vmaAddr == gcvNULL)
    {
        allocator = mdl->allocator;

        gcmkONERROR(gcmALLOCATOR_MapUser(allocator, mdl, mdlMap, gcvFALSE));
    }

    mutex_unlock(&mdl->mapsMutex);

    *Logical = mdlMap->vmaAddr;
    gcmkFOOTER_ARG("*Logical=%p", Logical);
    return gcvSTATUS_OK;

OnError:
    mutex_unlock(&mdl->mapsMutex);

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_UnmapMemory
**
**  Unmap physical memory out of the current process.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Start of physical address memory.
**
**      gctSIZE_T Bytes
**          Number of bytes to unmap.
**
**      gctPOINTER Memory
**          Pointer to a previously mapped memory region.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UnmapMemory(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical
    )
{
    gcmkHEADER_ARG("Os=%p Physical=0%p Bytes=0x%zx Logical=%p",
                   Os, Physical, Bytes, Logical);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != 0);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    gckOS_UnmapMemoryEx(Os, Physical, Bytes, Logical, _GetProcessID());

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}


/*******************************************************************************
**
**  gckOS_UnmapMemoryEx
**
**  Unmap physical memory in the specified process.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Start of physical address memory.
**
**      gctSIZE_T Bytes
**          Number of bytes to unmap.
**
**      gctPOINTER Memory
**          Pointer to a previously mapped memory region.
**
**      gctUINT32 PID
**          Pid of the process that opened the device and mapped this memory.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UnmapMemoryEx(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical,
    IN gctUINT32 PID
    )
{
    PLINUX_MDL_MAP mdlMap;
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Physical=%p Bytes=0x%zx Logical=%p PID=%d",
                   Os, Physical, Bytes, Logical, PID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != 0);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);
    gcmkVERIFY_ARGUMENT(PID != 0);

    if (Logical)
    {
        gckALLOCATOR allocator = mdl->allocator;

        mutex_lock(&mdl->mapsMutex);

        mdlMap = FindMdlMap(mdl, PID);

        if (mdlMap == gcvNULL)
        {
            mutex_unlock(&mdl->mapsMutex);
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        if (mdlMap->vmaAddr != gcvNULL)
        {
            BUG_ON(!allocator || !allocator->ops->UnmapUser);
            gcmALLOCATOR_UnmapUser(allocator, mdl, mdlMap, mdl->bytes);
        }

        gcmkVERIFY_OK(_DestroyMdlMap(mdl, mdlMap));

        mutex_unlock(&mdl->mapsMutex);
    }

OnError:
    gcmkFOOTER_NO();
    return status;
}

/*******************************************************************************
**
**  gckOS_AllocateNonPagedMemory
**
**  Allocate a number of pages from non-paged memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctBOOL InUserSpace
**          gcvTRUE if the pages need to be mapped into user space.
**
**      gctUINT32 Flag
**          Allocation attribute.
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that holds the number of bytes to allocate.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that hold the number of bytes allocated.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will hold the physical address of the
**          allocation.
**
**      gctPOINTER * Logical
**          Pointer to a variable that will hold the logical address of the
**          allocation.
*/
gceSTATUS
gckOS_AllocateNonPagedMemory(
    IN gckOS Os,
    IN gctBOOL InUserSpace,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    )
{
    gctSIZE_T bytes;
    gctSIZE_T numPages;
    PLINUX_MDL mdl = gcvNULL;
    PLINUX_MDL_MAP mdlMap = gcvNULL;
    gctPOINTER addr;
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckALLOCATOR allocator;
    gctBOOL zoneDMA32 = gcvFALSE;

    gcmkHEADER_ARG("Os=%p InUserSpace=%d *Bytes=0x%zx",
                   Os, InUserSpace, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes != gcvNULL);
    gcmkVERIFY_ARGUMENT(*Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    /* Align number of bytes to page size. */
    bytes = gcmALIGN(*Bytes, PAGE_SIZE);

    /* Get total number of pages.. */
    numPages = GetPageCount(bytes, 0);

    /* Allocate mdl structure */
    mdl = _CreateMdl(Os);
    if (mdl == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    gcmkASSERT(Flag & gcvALLOC_FLAG_CONTIGUOUS);

#if defined(CONFIG_ZONE_DMA32) || defined(CONFIG_ZONE_DMA)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    zoneDMA32 = gcvTRUE;
#endif
#endif

    if ((Flag & gcvALLOC_FLAG_4GB_ADDR) && !zoneDMA32)
    {
        Flag &= ~gcvALLOC_FLAG_4GB_ADDR;
    }

    /* Walk all allocators. */
    list_for_each_entry(allocator, &Os->allocatorList, link)
    {
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS,
                       "%s(%d) flag = %x allocator->capability = %x",
                        __FUNCTION__, __LINE__, Flag, allocator->capability);

#ifndef NO_DMA_COHERENT
        /* Point to dma coherent allocator. */
        if (!strcmp(allocator->name, "dma") ||
            ((Flag & allocator->capability) == Flag && numPages == 1))
        {
            status = gcmALLOCATOR_Alloc(allocator, mdl, numPages, Flag);

            if (gcmIS_SUCCESS(status))
            {
                mdl->allocator = allocator;
                break;
            }
        }
#else
        if ((Flag & allocator->capability) == Flag)
        {
            status = gcmALLOCATOR_Alloc(allocator, mdl, numPages, Flag);

            if (gcmIS_SUCCESS(status))
            {
                mdl->allocator = allocator;
                break;
            }
        }
#endif
    }

    /* Check status. */
    gcmkONERROR(status);

    mdl->cacheable = Flag & gcvALLOC_FLAG_CACHEABLE;

    mdl->bytes    = bytes;
    mdl->numPages = numPages;

    mdl->contiguous = gcvTRUE;
    mdl->cpuAccessible = gcvTRUE;

    gcmkONERROR(gcmALLOCATOR_MapKernel(allocator, mdl, 0, bytes, &addr));

    if (!strcmp(allocator->name, "gfp"))
    {
        /* Trigger a page fault. */
        memset(addr, 0, numPages * PAGE_SIZE);
    }

    mdl->addr = addr;

    if (InUserSpace)
    {
        mdlMap = _CreateMdlMap(mdl, _GetProcessID());

        if (mdlMap == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

        gcmkONERROR(gcmALLOCATOR_MapUser(allocator, mdl, mdlMap, gcvFALSE));

        *Logical = mdlMap->vmaAddr;
    }
    else
    {
        *Logical = addr;
    }

    /*
     * Add this to a global list.
     * Will be used by get physical address
     * and mapuser pointer functions.
     */
    mutex_lock(&Os->mdlMutex);
    list_add_tail(&mdl->link, &Os->mdlHead);
    mutex_unlock(&Os->mdlMutex);

    /* Return allocated memory. */
    *Bytes = bytes;
    *Physical = (gctPHYS_ADDR) mdl;

    /* Success. */
    status = gcvSTATUS_OK;

OnError:
    if (gcmIS_ERROR(status))
    {
        if (mdlMap)
        {
            /* Free LINUX_MDL_MAP. */
            gcmkVERIFY_OK(_DestroyMdlMap(mdl, mdlMap));
        }

        if (mdl)
        {
            /* Free LINUX_MDL. */
            gcmkVERIFY_OK(_DestroyMdl(mdl));
        }
    }
    /* Return the status. */
    gcmkFOOTER_ARG("*Bytes=0x%zx *Physical=%p *Logical=%p",
                   gcmOPT_VALUE(Bytes), gcmOPT_POINTER(Physical), gcmOPT_POINTER(Logical));
    return status;
}


/*******************************************************************************
**
**  gckOS_FreeNonPagedMemory
**
**  Free previously allocated and mapped pages from non-paged memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Physical address of the allocated memory.
**
**      gctPOINTER Logical
**          Logical address of the allocated memory.
**
**      gctSIZE_T Bytes
**          Number of bytes allocated.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS gckOS_FreeNonPagedMemory(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;

    gcmkHEADER_ARG("Os=%p Bytes=0x%zx Physical=%p Logical=%p",
                   Os, Bytes, Physical, Logical);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != 0);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    gcmkVERIFY_OK(_DestroyMdl(mdl));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static inline gckALLOCATOR
_FindAllocator(
    gckOS Os,
    gctUINT Flag
    )
{
    gckALLOCATOR allocator;

    list_for_each_entry(allocator, &Os->allocatorList, link)
    {
        if ((allocator->capability & Flag) == Flag)
        {
            return allocator;
        }
    }

    return gcvNULL;
}

gceSTATUS
gckOS_RequestReservedMemory(
    gckOS Os,
    gctPHYS_ADDR_T Start,
    gctSIZE_T Size,
    const char * Name,
    gctBOOL Requested,
    gctBOOL CpuAccessible,
    gctPOINTER * MemoryHandle
    )
{
    PLINUX_MDL mdl = gcvNULL;
    gceSTATUS status;
    gckALLOCATOR allocator;
    gcsATTACH_DESC desc;

    gcmkHEADER_ARG("start=0x%lx size=0x%lx name=%s", Start, Size, Name);

    /* Round up to page size. */
    Size = (Size + ~PAGE_MASK) & PAGE_MASK;

    mdl = _CreateMdl(Os);
    if (!mdl)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    desc.reservedMem.start     = Start;
    desc.reservedMem.size      = Size;
    desc.reservedMem.name      = Name;
    desc.reservedMem.requested = Requested;
    desc.reservedMem.root      = gcvTRUE;

    allocator = _FindAllocator(Os, gcvALLOC_FLAG_LINUX_RESERVED_MEM);
    if (!allocator)
    {
        gcmkPRINT("reserved-mem allocator not integrated!");
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Call attach. */
    gcmkONERROR(gcmALLOCATOR_Attach(allocator, &desc, mdl));

    /* Assign alloator. */
    mdl->allocator      = allocator;
    mdl->bytes          = Size;
    mdl->numPages       = Size >> PAGE_SHIFT;
    mdl->cpuAccessible  = CpuAccessible;
    mdl->contiguous     = gcvTRUE;
    mdl->addr           = gcvNULL;
    mdl->dmaHandle      = Start;
    mdl->gid            = 0;

    /*
     * Add this to a global list.
     * Will be used by get physical address
     * and mapuser pointer functions.
     */
    mutex_lock(&Os->mdlMutex);
    list_add_tail(&mdl->link, &Os->mdlHead);
    mutex_unlock(&Os->mdlMutex);

    *MemoryHandle = (void *)mdl;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (mdl)
    {
        gcmkVERIFY_OK(_DestroyMdl(mdl));
    }

    gcmkFOOTER();
    return status;
}

void
gckOS_ReleaseReservedMemory(
    gckOS Os,
    gctPOINTER MemoryHandle
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)MemoryHandle;

    if (mdl)
    {
        gcmkVERIFY_OK(_DestroyMdl(mdl));
    }
}

/*******************************************************************************
**
**  gckOS_RequestReservedMemoryArea
**
**  request a reserved memory area. it is used for dynamic mapping for usr
**
**  INPUT:
**
**      gctPOINTER MemoryHandle
**          Pointer to the root MDL.
**
**      gctSIZE_T Offset
**          Offset from the root reserved memory.
**
**       gctSIZE_T Size
**          Area size.
**
**       gctPOINTER * MemoryAreaHandle
**          Sub MDL address to save
**  OUTPUT:
**
**      gctPOINTER * MemoryAreaHandle
**          Pointer to sub MDL.
*/
gceSTATUS
gckOS_RequestReservedMemoryArea(
    IN gctPOINTER MemoryHandle,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Size,
    OUT gctPOINTER * MemoryAreaHandle
    )
{
    PLINUX_MDL rootMdl = (PLINUX_MDL)MemoryHandle;
    PLINUX_MDL subMdl = gcvNULL;
    gceSTATUS status;
    gcsATTACH_DESC desc;

    gcmkHEADER_ARG("MemoryHandle=%p Offset=0x%lx size=0x%lx", MemoryHandle, Offset, Size);

    /* Round up to page size. */
    Size = (Size + ~PAGE_MASK) & PAGE_MASK;

    subMdl = _CreateMdl(rootMdl->os);
    if (!subMdl)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    desc.reservedMem.start     = rootMdl->dmaHandle + Offset;
    desc.reservedMem.size      = Size;
    desc.reservedMem.name      = "subRMA";
    /* consider that the memory region has requested by the root mdl */
    desc.reservedMem.requested = gcvTRUE;
    desc.reservedMem.root      = gcvFALSE;

    /* Call attach. */
    gcmkONERROR(gcmALLOCATOR_Attach((gckALLOCATOR)rootMdl->allocator, &desc, subMdl));

    /* Assign alloator. */
    subMdl->allocator      = rootMdl->allocator;
    subMdl->bytes          = Size;
    subMdl->numPages       = Size >> PAGE_SHIFT;
    subMdl->cpuAccessible  = rootMdl->cpuAccessible;
    subMdl->contiguous     = gcvTRUE;
    subMdl->addr           = gcvNULL;
    subMdl->dmaHandle      = rootMdl->dmaHandle + Offset;
    subMdl->gid            = 0;

    mutex_lock(&rootMdl->os->mdlMutex);
    list_add_tail(&subMdl->rmaLink, &rootMdl->rmaHead);
    mutex_unlock(&rootMdl->os->mdlMutex);

    *MemoryAreaHandle = (void *)subMdl;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (subMdl)
    {
        gcmkVERIFY_OK(_DestroyMdl(subMdl));
    }

    gcmkFOOTER();
    return status;
}

void
gckOS_ReleaseReservedMemoryArea(
    gctPOINTER MemoryAreaHandle
    )
{
    PLINUX_MDL subMdl = (PLINUX_MDL)MemoryAreaHandle;

    if (subMdl)
    {
        gcmkVERIFY_OK(_DestroyMdl(subMdl));
    }
}

/*******************************************************************************
**
**  gckOS_ReadRegister
**
**  Read data from a register.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Address
**          Address of register.
**
**  OUTPUT:
**
**      gctUINT32 * Data
**          Pointer to a variable that receives the data read from the register.
*/
gceSTATUS
gckOS_ReadRegister(
    IN gckOS Os,
    IN gctUINT32 Address,
    OUT gctUINT32 * Data
    )
{
    return gckOS_ReadRegisterEx(Os, gcvCORE_MAJOR, Address, Data);
}

gceSTATUS
gckOS_ReadRegisterEx(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT32 Address,
    OUT gctUINT32 * Data
    )
{
    if (Address > Os->device->registerSizes[Core] - 1)
    {
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    if (in_irq())
    {
        uint32_t data;

        spin_lock(&Os->registerAccessLock);

        if (unlikely(Os->clockStates[Core] == gcvFALSE))
        {
            spin_unlock(&Os->registerAccessLock);

            /*
             * Read register when external clock off:
             * 1. In shared IRQ, read register may be called and that's not our irq.
             */
            return gcvSTATUS_GENERIC_IO;
        }

        data = readl(Os->device->registerBases[Core]);

        if (unlikely((data & 0x3) == 0x3))
        {
            spin_unlock(&Os->registerAccessLock);

            /*
             * Read register when internal clock off:
             * a. In shared IRQ, read register may be called and that's not our irq.
             * b. In some condition, when ISR handled normal FE/PE, PM thread could
             *    trun off internal clock before ISR read register of async FE. And
             *    then IRQ handler will call read register with internal clock off.
             *    So here we just skip for such case.
             */
            return gcvSTATUS_GENERIC_IO;
        }

        *Data = readl((gctUINT8 *)Os->device->registerBases[Core] + Address);
        spin_unlock(&Os->registerAccessLock);
    }
    else
    {
        unsigned long flags;

        spin_lock_irqsave(&Os->registerAccessLock, flags);

        if (unlikely(Os->clockStates[Core] == gcvFALSE))
        {
            spin_unlock_irqrestore(&Os->registerAccessLock, flags);

            /*
             * Read register when external clock off:
             * 2. In non-irq context, register access should not be called,
             *    otherwise it's driver bug.
             */
            printk(KERN_ERR "[galcore]: %s(%d) GPU[%d] external clock off",
                   __func__, __LINE__, Core);
            gcmkBUG_ON(1);
            return gcvSTATUS_GENERIC_IO;
        }

        *Data = readl((gctUINT8 *)Os->device->registerBases[Core] + Address);
        spin_unlock_irqrestore(&Os->registerAccessLock, flags);

#if gcdDUMP_AHB_ACCESS
        /* Dangerous to print in interrupt context, skip. */
        gcmkPRINT("@[RD %d] %08x %08x", Core, Address, *Data);
#endif
    }

    /* Success. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_WriteRegisterEx(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT32 Address,
    IN gctUINT32 Data,
    IN gctBOOL Dump
    )
{
    if (Address > Os->device->registerSizes[Core] - 1)
    {
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    if (in_irq())
    {
        spin_lock(&Os->registerAccessLock);

        if (unlikely(Os->clockStates[Core] == gcvFALSE))
        {
            spin_unlock(&Os->registerAccessLock);

            printk(KERN_ERR "[galcore]: %s(%d) GPU[%d] external clock off",
                   __func__, __LINE__, Core);

            /* Driver bug: register write when clock off. */
            gcmkBUG_ON(1);
            return gcvSTATUS_GENERIC_IO;
        }

        writel(Data, (gctUINT8 *)Os->device->registerBases[Core] + Address);
        spin_unlock(&Os->registerAccessLock);
    }
    else
    {
        unsigned long flags;

        if (Dump)
        {
            gcmkDUMP(Os, "@[register.write %u 0x%05X 0x%08X]", Core, Address, Data);
        }

        spin_lock_irqsave(&Os->registerAccessLock, flags);

        if (unlikely(Os->clockStates[Core] == gcvFALSE))
        {
            spin_unlock_irqrestore(&Os->registerAccessLock, flags);

            printk(KERN_ERR "[galcore]: %s(%d) GPU[%d] external clock off",
                      __func__, __LINE__, Core);

            /* Driver bug: register write when clock off. */
            gcmkBUG_ON(1);
            return gcvSTATUS_GENERIC_IO;
        }

        writel(Data, (gctUINT8 *)Os->device->registerBases[Core] + Address);
        spin_unlock_irqrestore(&Os->registerAccessLock, flags);

#if gcdDUMP_AHB_ACCESS
        /* Dangerous to print in interrupt context, skip. */
        gcmkPRINT("@[WR %d] %08x %08x", Core, Address, Data);
#endif
    }

    /* Success. */
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_WriteRegister
**
**  Write data to a register.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Address
**          Address of register.
**
**      gctUINT32 Data
**          Data for register.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_WriteRegister(
    IN gckOS Os,
    IN gctUINT32 Address,
    IN gctUINT32 Data
    )
{
    return _WriteRegisterEx(Os, gcvCORE_MAJOR, Address, Data, gcvTRUE);
}

gceSTATUS
gckOS_WriteRegisterEx(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT32 Address,
    IN gctUINT32 Data
    )
{
    return _WriteRegisterEx(Os, Core, Address, Data, gcvTRUE);
}

gceSTATUS
gckOS_WriteRegisterEx_NoDump(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT32 Address,
    IN gctUINT32 Data
    )
{
    return _WriteRegisterEx(Os, Core, Address, Data, gcvFALSE);
}

/*******************************************************************************
**
**  gckOS_GetPageSize
**
**  Get the system's page size.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**  OUTPUT:
**
**      gctSIZE_T * PageSize
**          Pointer to a variable that will receive the system's page size.
*/
gceSTATUS gckOS_GetPageSize(
    IN gckOS Os,
    OUT gctSIZE_T * PageSize
    )
{
    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(PageSize != gcvNULL);

    /* Return the page size. */
    *PageSize = (gctSIZE_T) PAGE_SIZE;

    /* Success. */
    gcmkFOOTER_ARG("*PageSize=0x%zx", *PageSize);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  _GetPhysicalAddressProcess
**
**  Get the physical system address of a corresponding virtual address for a
**  given process.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctPOINTER Logical
**          Logical address.
**
**      gctUINT32 ProcessID
**          Process ID.
**
**  OUTPUT:
**
**      gctUINT32 * Address
**          Poinetr to a variable that receives the 32-bit physical adress.
*/
static gceSTATUS
_GetPhysicalAddressProcess(
    IN gckOS Os,
    IN gctPOINTER Logical,
    IN gctUINT32 ProcessID,
    OUT gctPHYS_ADDR_T * Address
    )
{
    PLINUX_MDL mdl;
    gceSTATUS status = gcvSTATUS_INVALID_ADDRESS;

    gcmkHEADER_ARG("Os=%p Logical=%p ProcessID=%d", Os, Logical, ProcessID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Address != gcvNULL);

    mutex_lock(&Os->mdlMutex);

    if (Os->device->contiguousPhysical)
    {
        /* Try the contiguous memory pool. */
        mdl = (PLINUX_MDL) Os->device->contiguousPhysical;

        mutex_lock(&mdl->mapsMutex);

        status = _ConvertLogical2Physical(Os, Logical, ProcessID, mdl, Address);

        mutex_unlock(&mdl->mapsMutex);
    }

    if (gcmIS_ERROR(status))
    {
        /* Walk all MDLs. */
        list_for_each_entry(mdl, &Os->mdlHead, link)
        {
            mutex_lock(&mdl->mapsMutex);

            if (mdl->addr != gcvNULL)
            {
                status = _ConvertLogical2Physical(Os, Logical, ProcessID, mdl, Address);
            }
            else if (!list_empty(&mdl->rmaHead))
            {
                PLINUX_MDL subMdl;

                list_for_each_entry(subMdl, &mdl->rmaHead, rmaLink)
                {
                    status = _ConvertLogical2Physical(Os, Logical, ProcessID, subMdl, Address);
                    if (gcmIS_SUCCESS(status))
                    {
                        break;
                    }
                }
            }

            mutex_unlock(&mdl->mapsMutex);

            if (gcmIS_SUCCESS(status))
            {
                break;
            }
        }
    }

    mutex_unlock(&Os->mdlMutex);

    gcmkONERROR(status);
    /* Success. */

OnError:
    /* Return the status. */
    gcmkFOOTER_ARG("*Address=%p", *Address);
    return status;
}



/*******************************************************************************
**
**  gckOS_GetPhysicalAddress
**
**  Get the physical system address of a corresponding virtual address.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Logical
**          Logical address.
**
**  OUTPUT:
**
**      gctUINT32 * Address
**          Poinetr to a variable that receives the 32-bit physical adress.
*/
gceSTATUS
gckOS_GetPhysicalAddress(
    IN gckOS Os,
    IN gctPOINTER Logical,
    OUT gctPHYS_ADDR_T * Address
    )
{
    gceSTATUS status;
    gctUINT32 processID;

    gcmkHEADER_ARG("Os=%p Logical=%p", Os, Logical);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Address != gcvNULL);

    /* Query page table of current process first. */
    status = _QueryProcessPageTable(Logical, Address);

    if (gcmIS_ERROR(status))
    {
        /* Get current process ID. */
        processID = _GetProcessID();

        /* Route through other function. */
        gcmkONERROR(
            _GetPhysicalAddressProcess(Os, Logical, processID, Address));
    }

    /* Success. */
    gcmkFOOTER_ARG("*Address=0x%llx", *Address);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckOS_GetPhysicalFromHandle(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * PhysicalAddress
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;
    gckALLOCATOR allocator = mdl->allocator;

    return gcmALLOCATOR_Physical(allocator, mdl, Offset, PhysicalAddress);
}


/*******************************************************************************
**
**  gckOS_UserLogicalToPhysical
**
**  Get the physical system address of a corresponding user virtual address.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Logical
**          Logical address.
**
**  OUTPUT:
**
**      gctUINT32 * Address
**          Pointer to a variable that receives the 32-bit physical address.
*/
gceSTATUS gckOS_UserLogicalToPhysical(
    IN gckOS Os,
    IN gctPOINTER Logical,
    OUT gctPHYS_ADDR_T * Address
    )
{
    return gckOS_GetPhysicalAddress(Os, Logical, Address);
}

gceSTATUS
_ConvertLogical2Physical(
    IN gckOS Os,
    IN gctPOINTER Logical,
    IN gctUINT32 ProcessID,
    IN PLINUX_MDL Mdl,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    gckALLOCATOR allocator = Mdl->allocator;
    gctUINT32 offset;
    gceSTATUS status = gcvSTATUS_NOT_FOUND;
    gctINT8_PTR vBase;

    /* TASK_SIZE is userspace - kernelspace virtual memory split. */
    if ((gctUINTPTR_T)Logical >= TASK_SIZE)
    {
        /* Kernel virtual address. */
        vBase = Mdl->addr;
    }
    else
    {
        /* User virtual address. */
        PLINUX_MDL_MAP map;

        map   = FindMdlMap(Mdl, (gctINT) ProcessID);
        vBase = (map == gcvNULL) ? gcvNULL : (gctINT8_PTR) map->vmaAddr;
    }

    /* Is the given address within that range. */
    if ((vBase != gcvNULL)
    &&  ((gctINT8_PTR) Logical >= vBase)
    &&  ((gctINT8_PTR) Logical <  vBase + Mdl->bytes)
    )
    {
        offset = (gctINT8_PTR) Logical - vBase;

        gcmALLOCATOR_Physical(allocator, Mdl, offset, Physical);

        status = gcvSTATUS_OK;
    }

    return status;
}

/*******************************************************************************
**
**  gckOS_MapPhysical
**
**  Map a physical address into kernel space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR_T Physical
**          Physical address of the memory to map.
**
**      gctSIZE_T Bytes
**          Number of bytes to map.
**
**  OUTPUT:
**
**      gctPOINTER * Logical
**          Pointer to a variable that receives the base address of the mapped
**          memory.
*/
gceSTATUS
gckOS_MapPhysical(
    IN gckOS Os,
    IN gctPHYS_ADDR_T Physical,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    )
{
    gctPOINTER logical;
    PLINUX_MDL mdl;
    gctBOOL found = gcvFALSE;
    dma_addr_t physical = Physical;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Physical=0x%llx Bytes=0x%zx", Os, Physical, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    mutex_lock(&Os->mdlMutex);

    /* Go through our mapping to see if we know this physical address already. */
    list_for_each_entry(mdl, &Os->mdlHead, link)
    {
        if (mdl->dmaHandle != 0)
        {
            if ((physical >= mdl->dmaHandle)
            &&  (physical < mdl->dmaHandle + mdl->bytes))
            {

                if (mdl->addr != gcvNULL)
                {
                    *Logical = mdl->addr + (physical - mdl->dmaHandle);
                    found = gcvTRUE;
                }
                else if (!list_empty(&mdl->rmaHead))
                {
                    PLINUX_MDL subMdl;

                    /* when enable dynamic mapping, MDL from the mdlHead is the root.
                     * the sub MDL should be looped from root MDL
                     */
                    list_for_each_entry(subMdl, &mdl->rmaHead, rmaLink)
                    {
                        if ((physical >= subMdl->dmaHandle)
                            && (physical < subMdl->dmaHandle + subMdl->bytes)
                            && (subMdl->addr != 0))
                        {
                            *Logical = subMdl->addr + (physical - subMdl->dmaHandle);
                            found = gcvTRUE;
                            break;
                        }
                    }
                }

                if (found)
                {
                    break;
                }
            }
        }
    }

    mutex_unlock(&Os->mdlMutex);

    if (!found)
    {
        unsigned long pfn = physical >> PAGE_SHIFT;

        if (pfn_valid(pfn))
        {
            gctUINT32 offset = physical & ~PAGE_MASK;
            struct page ** pages;
            struct page * page;
            gctSIZE_T numPages;
            gctSIZE_T i;
            pgprot_t pgprot;

            numPages = GetPageCount(PAGE_ALIGN(offset + Bytes), 0);

            pages = kmalloc(sizeof(struct page *) * numPages, GFP_KERNEL | gcdNOWARN);
            if (!pages)
            {
                gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
            }

            page = pfn_to_page(pfn);

            for (i = 0; i < numPages; i++)
            {
                pages[i] = nth_page(page, i);
            }

#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
            pgprot = pgprot_writecombine(PAGE_KERNEL);
#else
            pgprot = pgprot_noncached(PAGE_KERNEL);
#endif

            logical = vmap(pages, numPages, 0, pgprot);

            kfree(pages);

            if (logical == gcvNULL)
            {
                gcmkTRACE_ZONE(
                    gcvLEVEL_INFO, gcvZONE_OS,
                    "%s(%d): Failed to vmap",
                    __FUNCTION__, __LINE__
                    );

                /* Out of resources. */
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }

            logical += offset;
        }
        else
        {
            /* Map memory as cached memory. */
            request_mem_region(physical, Bytes, "MapRegion");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
            logical = (gctPOINTER) ioremap(physical, Bytes);
#else
            logical = (gctPOINTER) ioremap_nocache(physical, Bytes);
#endif
            if (logical == gcvNULL)
            {
                gcmkTRACE_ZONE(
                    gcvLEVEL_INFO, gcvZONE_OS,
                    "%s(%d): Failed to ioremap",
                    __FUNCTION__, __LINE__
                    );

                /* Out of resources. */
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }

        /* Return pointer to mapped memory. */
        *Logical = logical;
    }

OnError:
    /* Success. */
    gcmkFOOTER_ARG("*Logical=%p", *Logical);
    return status;
}

/*******************************************************************************
**
**  gckOS_UnmapPhysical
**
**  Unmap a previously mapped memory region from kernel memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Logical
**          Pointer to the base address of the memory to unmap.
**
**      gctSIZE_T Bytes
**          Number of bytes to unmap.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UnmapPhysical(
    IN gckOS Os,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    PLINUX_MDL  mdl;
    gctBOOL found = gcvFALSE;

    gcmkHEADER_ARG("Os=%p Logical=%p Bytes=0x%zx", Os, Logical, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);

    mutex_lock(&Os->mdlMutex);

    list_for_each_entry(mdl, &Os->mdlHead, link)
    {
        if (mdl->addr != gcvNULL)
        {
            if ((Logical >= (gctPOINTER)mdl->addr) &&
                (Logical < (gctPOINTER)((gctSTRING)mdl->addr + mdl->bytes)))
            {
                found = gcvTRUE;
            }
        }
        else if (!list_empty(&mdl->rmaHead))
        {
            PLINUX_MDL subMdl;
            /* Find the subMDL */
            list_for_each_entry(subMdl, &mdl->rmaHead, rmaLink)
            {
                if ((subMdl->addr != gcvNULL) &&
                    (Logical >= (gctPOINTER)subMdl->addr) &&
                    (Logical < (gctPOINTER)((gctSTRING)subMdl->addr + subMdl->bytes)))
                {
                    found = gcvTRUE;
                    break;
                }
            }
        }

        if (found)
        {
            break;
        }
    }

    mutex_unlock(&Os->mdlMutex);

    if (!found)
    {
        /* Unmap the memory. */
        vunmap((void *)((unsigned long)Logical & PAGE_MASK));
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_DeleteMutex
**
**  Delete a mutex.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Mutex
**          Pointer to the mute to be deleted.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_DeleteMutex(
    IN gckOS Os,
    IN gctPOINTER Mutex
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Mutex=%p", Os, Mutex);

    /* Validate the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Mutex != gcvNULL);

    /* Destroy the mutex. */
    mutex_destroy((struct mutex *)Mutex);

    /* Free the mutex structure. */
    gcmkONERROR(gckOS_Free(Os, Mutex));

OnError:
    /* Return status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_AcquireMutex
**
**  Acquire a mutex.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Mutex
**          Pointer to the mutex to be acquired.
**
**      gctUINT32 Timeout
**          Timeout value specified in milliseconds.
**          Specify the value of gcvINFINITE to keep the thread suspended
**          until the mutex has been acquired.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AcquireMutex(
    IN gckOS Os,
    IN gctPOINTER Mutex,
    IN gctUINT32 Timeout
    )
{
    gceSTATUS status = gcvSTATUS_TIMEOUT;
    gcmkHEADER_ARG("Os=%p Mutex=%p Timeout=%u", Os, Mutex, Timeout);

    /* Validate the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Mutex != gcvNULL);

    if (Timeout == gcvINFINITE)
    {
        /* Lock the mutex. */
        mutex_lock(Mutex);

        /* Success. */
        status = gcvSTATUS_OK;
    }
    else
    {
        for (;;)
        {
            /* Try to acquire the mutex. */
            if (mutex_trylock(Mutex))
            {
                /* Success. */
                status = gcvSTATUS_OK;
                break;
            }

            if (Timeout-- == 0)
            {
                break;
            }

            /* Wait for 1 millisecond. */
            gcmkVERIFY_OK(gckOS_Delay(Os, 1));
        }
    }

    /* Timeout. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_ReleaseMutex
**
**  Release an acquired mutex.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Mutex
**          Pointer to the mutex to be released.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_ReleaseMutex(
    IN gckOS Os,
    IN gctPOINTER Mutex
    )
{
    gcmkHEADER_ARG("Os=%p Mutex=%p", Os, Mutex);

    /* Validate the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Mutex != gcvNULL);

    /* Release the mutex. */
    mutex_unlock(Mutex);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomicExchange
**
**  Atomically exchange a pair of 32-bit values.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      IN OUT gctINT32_PTR Target
**          Pointer to the 32-bit value to exchange.
**
**      IN gctINT32 NewValue
**          Specifies a new value for the 32-bit value pointed to by Target.
**
**      OUT gctINT32_PTR OldValue
**          The old value of the 32-bit value pointed to by Target.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomicExchange(
    IN gckOS Os,
    IN OUT gctUINT32_PTR Target,
    IN gctUINT32 NewValue,
    OUT gctUINT32_PTR OldValue
    )
{
    /* Exchange the pair of 32-bit values. */
    *OldValue = (gctUINT32) atomic_xchg((atomic_t *) Target, (int) NewValue);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomicExchangePtr
**
**  Atomically exchange a pair of pointers.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      IN OUT gctPOINTER * Target
**          Pointer to the 32-bit value to exchange.
**
**      IN gctPOINTER NewValue
**          Specifies a new value for the pointer pointed to by Target.
**
**      OUT gctPOINTER * OldValue
**          The old value of the pointer pointed to by Target.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomicExchangePtr(
    IN gckOS Os,
    IN OUT gctPOINTER * Target,
    IN gctPOINTER NewValue,
    OUT gctPOINTER * OldValue
    )
{
    /* Exchange the pair of pointers. */
    *OldValue = (gctPOINTER)(gctUINTPTR_T) atomic_xchg((atomic_t *) Target, (int)(gctUINTPTR_T) NewValue);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomicSetMask
**
**  Atomically set mask to Atom
**
**  INPUT:
**      IN OUT gctPOINTER Atom
**          Pointer to the atom to set.
**
**      IN gctUINT32 Mask
**          Mask to set.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomSetMask(
    IN gctPOINTER Atom,
    IN gctUINT32 Mask
    )
{
    gctUINT32 oval, nval;

    do
    {
        oval = atomic_read((atomic_t *) Atom);
        nval = oval | Mask;
    }
    while (atomic_cmpxchg((atomic_t *) Atom, oval, nval) != oval);

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomClearMask
**
**  Atomically clear mask from Atom
**
**  INPUT:
**      IN OUT gctPOINTER Atom
**          Pointer to the atom to clear.
**
**      IN gctUINT32 Mask
**          Mask to clear.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomClearMask(
    IN gctPOINTER Atom,
    IN gctUINT32 Mask
    )
{
    gctUINT32 oval, nval;

    do
    {
        oval = atomic_read((atomic_t *) Atom);
        nval = oval & ~Mask;
    }
    while (atomic_cmpxchg((atomic_t *) Atom, oval, nval) != oval);

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomConstruct
**
**  Create an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**  OUTPUT:
**
**      gctPOINTER * Atom
**          Pointer to a variable receiving the constructed atom.
*/
gceSTATUS
gckOS_AtomConstruct(
    IN gckOS Os,
    OUT gctPOINTER * Atom
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Atom != gcvNULL);

    /* Allocate the atom. */
    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(atomic_t), Atom));

    /* Initialize the atom. */
    atomic_set((atomic_t *) *Atom, 0);

OnError:
    /* Return the status. */
    gcmkFOOTER_ARG("*Atom=%p", *Atom);
    return status;
}

/*******************************************************************************
**
**  gckOS_AtomDestroy
**
**  Destroy an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gctPOINTER Atom
**          Pointer to the atom to destroy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomDestroy(
    IN gckOS Os,
    OUT gctPOINTER Atom
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Atom=%p", Os, Atom);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Atom != gcvNULL);

    /* Free the atom. */
    gcmkONERROR(gcmkOS_SAFE_FREE(Os, Atom));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_AtomGet
**
**  Get the 32-bit value protected by an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gctPOINTER Atom
**          Pointer to the atom.
**
**  OUTPUT:
**
**      gctINT32_PTR Value
**          Pointer to a variable the receives the value of the atom.
*/
gceSTATUS
gckOS_AtomGet(
    IN gckOS Os,
    IN gctPOINTER Atom,
    OUT gctINT32_PTR Value
    )
{
    /* Return the current value of atom. */
    *Value = atomic_read((atomic_t *) Atom);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomSet
**
**  Set the 32-bit value protected by an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gctPOINTER Atom
**          Pointer to the atom.
**
**      gctINT32 Value
**          The value of the atom.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AtomSet(
    IN gckOS Os,
    IN gctPOINTER Atom,
    IN gctINT32 Value
    )
{
    /* Set the current value of atom. */
    atomic_set((atomic_t *) Atom, Value);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomIncrement
**
**  Atomically increment the 32-bit integer value inside an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gctPOINTER Atom
**          Pointer to the atom.
**
**  OUTPUT:
**
**      gctINT32_PTR Value
**          Pointer to a variable that receives the original value of the atom.
*/
gceSTATUS
gckOS_AtomIncrement(
    IN gckOS Os,
    IN gctPOINTER Atom,
    OUT gctINT32_PTR Value
    )
{
    *Value = atomic_inc_return((atomic_t *) Atom) - 1;
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AtomDecrement
**
**  Atomically decrement the 32-bit integer value inside an atom.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gctPOINTER Atom
**          Pointer to the atom.
**
**  OUTPUT:
**
**      gctINT32_PTR Value
**          Pointer to a variable that receives the original value of the atom.
*/
gceSTATUS
gckOS_AtomDecrement(
    IN gckOS Os,
    IN gctPOINTER Atom,
    OUT gctINT32_PTR Value
    )
{
    /* Decrement the atom. */
    *Value = atomic_dec_return((atomic_t *) Atom) + 1;
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_Delay
**
**  Delay execution of the current thread for a number of milliseconds.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Delay
**          Delay to sleep, specified in milliseconds.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Delay(
    IN gckOS Os,
    IN gctUINT32 Delay
    )
{
    gcmkHEADER_ARG("Os=%p Delay=%u", Os, Delay);

    if (Delay > 0)
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
        ktime_t delay = ktime_set((Delay / MSEC_PER_SEC), (Delay % MSEC_PER_SEC) * NSEC_PER_MSEC);
        __set_current_state(TASK_UNINTERRUPTIBLE);
        schedule_hrtimeout(&delay, HRTIMER_MODE_REL);
#else
        msleep(Delay);
#endif
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_Udelay
**
**  Delay execution of the current thread for a number of microseconds.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Delay
**          Delay to sleep, specified in microseconds.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Udelay(
    IN gckOS Os,
    IN gctUINT32 Delay
    )
{
    gcmkHEADER_ARG("Os=%p Delay=%u", Os, Delay);

    if (Delay > 0)
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
        ktime_t delay = ktime_set((Delay / USEC_PER_SEC), (Delay % USEC_PER_SEC) * NSEC_PER_USEC);
        __set_current_state(TASK_UNINTERRUPTIBLE);
        schedule_hrtimeout(&delay, HRTIMER_MODE_REL);
#else
        usleep_range((unsigned long)Delay, (unsigned long)Delay + 1);
#endif
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_GetTicks
**
**  Get the number of milliseconds since the system started.
**
**  INPUT:
**
**  OUTPUT:
**
**      gctUINT32_PTR Time
**          Pointer to a variable to get time.
**
*/
gceSTATUS
gckOS_GetTicks(
    OUT gctUINT32_PTR Time
    )
{
     gcmkHEADER();

    *Time = jiffies_to_msecs(jiffies);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_TicksAfter
**
**  Compare time values got from gckOS_GetTicks.
**
**  INPUT:
**      gctUINT32 Time1
**          First time value to be compared.
**
**      gctUINT32 Time2
**          Second time value to be compared.
**
**  OUTPUT:
**
**      gctBOOL_PTR IsAfter
**          Pointer to a variable to result.
**
*/
gceSTATUS
gckOS_TicksAfter(
    IN gctUINT32 Time1,
    IN gctUINT32 Time2,
    OUT gctBOOL_PTR IsAfter
    )
{
    gcmkHEADER();

    *IsAfter = time_after((unsigned long)Time1, (unsigned long)Time2);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_GetTime
**
**  Get the number of microseconds since the system started.
**
**  INPUT:
**
**  OUTPUT:
**
**      gctUINT64_PTR Time
**          Pointer to a variable to get time.
**
*/
gceSTATUS
gckOS_GetTime(
    OUT gctUINT64_PTR Time
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
    struct timespec64 tv;
    gcmkHEADER();

    /* Return the time of day in microseconds. */
    ktime_get_real_ts64(&tv);
    *Time = (tv.tv_sec * 1000000ULL) + (tv.tv_nsec / 1000);
#else
    struct timeval tv;
    gcmkHEADER();

     /* Return the time of day in microseconds. */
    do_gettimeofday(&tv);
    *Time = (tv.tv_sec * 1000000ULL) + tv.tv_usec;
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  _ExternalCacheOperation
**
**  External device cache operation, if support. If the core has any additional caches
**  they must be invalidated after this function returns. If the core does not
**  have any addional caches the externalCacheOperation in the platform->ops should
**  remain NULL.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gceCACHEOPERATION Operation
**          Cache Operation: gcvCACHE_FLUSH, gcvCACHE_CLEAN or gcvCACHE_INVALIDATE.
**
**  OUTPUT:
**
**      Nothing.
*/
static void
_ExternalCacheOperation(
    IN gckOS Os,
    IN gceCACHEOPERATION Operation
    )
{
    gcsPLATFORM *platform = Os->device->platform;

    if (platform && platform->ops->externalCacheOperation)
    {
        platform->ops->externalCacheOperation(platform, Operation);
    }
}

/*******************************************************************************
**
**  gckOS_MemoryBarrier
**
**  Make sure the CPU has executed everything up to this point and the data got
**  written to the specified pointer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Address
**          Address of memory that needs to be barriered.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_MemoryBarrier(
    IN gckOS Os,
    IN gctPOINTER Address
    )
{
    _MemoryBarrier();

    _ExternalCacheOperation(Os, gcvCACHE_INVALIDATE);

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_AllocatePagedMemory
**
**  Allocate memory from the paged pool.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Flag
**          Allocation attribute.
**
**      gctSIZE_T * Bytes
**          Number of bytes to allocate.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Return number of bytes actually allocated.
**
**      gctUINT32 * Gid
**          Save the global ID for the piece of allocated memory.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that receives the physical address of the
**          memory allocation.
*/
gceSTATUS
gckOS_AllocatePagedMemory(
    IN gckOS Os,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    OUT gctUINT32 * Gid,
    OUT gctPHYS_ADDR * Physical
    )
{
    gctSIZE_T numPages;
    PLINUX_MDL mdl = gcvNULL;
    gctSIZE_T bytes;
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckALLOCATOR allocator;
    gctBOOL zoneDMA32 = gcvFALSE;

    gcmkHEADER_ARG("Os=%p Flag=%x *Bytes=0x%zx", Os, Flag, *Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(*Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);

    bytes = gcmALIGN(*Bytes, PAGE_SIZE);

    numPages = GetPageCount(bytes, 0);

    mdl = _CreateMdl(Os);
    if (mdl == gcvNULL)
    {
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }

#if defined(CONFIG_ZONE_DMA32) || defined(CONFIG_ZONE_DMA)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    zoneDMA32 = gcvTRUE;
#endif
#endif

    if ((Flag & gcvALLOC_FLAG_4GB_ADDR) && !zoneDMA32)
    {
        Flag &= ~gcvALLOC_FLAG_4GB_ADDR;
    }

    /* Walk all allocators. */
    list_for_each_entry(allocator, &Os->allocatorList, link)
    {
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS,
                       "%s(%d) flag = %x allocator->capability = %x",
                        __FUNCTION__, __LINE__, Flag, allocator->capability);

        if ((Flag & allocator->capability) != Flag)
        {
            continue;
        }

        status = gcmALLOCATOR_Alloc(allocator, mdl, numPages, Flag);

        if (gcmIS_SUCCESS(status))
        {
            mdl->allocator = allocator;
            break;
        }
    }

    /* Check status. */
    if (status == gcvSTATUS_OUT_OF_MEMORY)
    {
        /* Ignore error print in this function, leave it to high level function. */
        goto OnError;
    }
    else
    {
        gcmkONERROR(status);
    }

    mdl->dmaHandle  = 0;
    mdl->addr       = 0;
    mdl->bytes      = bytes;
    mdl->numPages   = numPages;
    mdl->contiguous = Flag & gcvALLOC_FLAG_CONTIGUOUS;
    mdl->cacheable  = Flag & gcvALLOC_FLAG_CACHEABLE;
    mdl->cpuAccessible = gcvTRUE;

    /*
     * Add this to a global list.
     * Will be used by get physical address
     * and mapuser pointer functions.
     */
    mutex_lock(&Os->mdlMutex);
    list_add_tail(&mdl->link, &Os->mdlHead);
    mutex_unlock(&Os->mdlMutex);

    /* Return allocated bytes. */
    *Bytes = bytes;

    if (Gid != gcvNULL)
    {
        *Gid = mdl->gid;
    }

    /* Return physical address. */
    *Physical = (gctPHYS_ADDR) mdl;

    /* Success. */
    status = gcvSTATUS_OK;

OnError:
    if (gcmIS_ERROR(status) && mdl)
    {
        /* Free the memory. */
        _DestroyMdl(mdl);
    }

    /* Return the status. */
    gcmkFOOTER_ARG("*Physical=%p", *Physical);
    return status;
}

/*******************************************************************************
**
**  gckOS_FreePagedMemory
**
**  Free memory allocated from the paged pool.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Physical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes of the allocation.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_FreePagedMemory(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)Physical;

    gcmkHEADER_ARG("Os=%p Physical=%p Bytes=0x%zx", Os, Physical, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);

    /* Free the structure... */
    gcmkVERIFY_OK(_DestroyMdl(mdl));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_LockPages
**
**  Lock memory allocated from the paged pool.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Physical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes of the allocation.
**
**      gctBOOL Cacheable
**          Cache mode of mapping.
**
**  OUTPUT:
**
**      gctPOINTER * Logical
**          Pointer to a variable that receives the address of the mapped
**          memory.
*/
gceSTATUS
gckOS_LockPages(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctBOOL Cacheable,
    OUT gctPOINTER * Logical
    )
{
    gceSTATUS       status = gcvSTATUS_OK;
    PLINUX_MDL      mdl;
    PLINUX_MDL_MAP  mdlMap;
    gckALLOCATOR    allocator;

    gcmkHEADER_ARG("Os=%p Physical=%p Bytes=0x%zx", Os, Physical, Logical);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    mdl = (PLINUX_MDL)Physical;
    allocator = mdl->allocator;

    mutex_lock(&mdl->mapsMutex);

    mdlMap = FindMdlMap(mdl, _GetProcessID());

    if (mdlMap == gcvNULL)
    {
        mdlMap = _CreateMdlMap(mdl, _GetProcessID());

        if (mdlMap == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }

    if (mdlMap->vmaAddr == gcvNULL)
    {
        gcmkONERROR(gcmALLOCATOR_MapUser(allocator, mdl, mdlMap, Cacheable));
    }

    mdlMap->count++;

    /* Convert pointer to MDL. */
    *Logical = mdlMap->vmaAddr;

OnError:
    mutex_unlock(&mdl->mapsMutex);
    /* Success. */
    gcmkFOOTER_ARG("*Logical=%p", *Logical);
    return status;
}

/* PageCount is GPU page count. */
gceSTATUS
gckOS_MapPagesEx(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T PageCount,
    IN gctUINT32 Address,
    IN gctPOINTER PageTable,
    IN gctBOOL Writable,
    IN gceVIDMEM_TYPE Type
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    PLINUX_MDL  mdl;
    gctUINT32*  table;
    gctUINT32   offset = 0;

    gctUINT32 bytes = PageCount * 4;
    gckALLOCATOR allocator;

    gctUINT32 policyID = 0;
    gctUINT32 axiConfig = 0;

    gcsPLATFORM * platform = Os->device->platform;

    gcmkHEADER_ARG("Os=%p Core=%d Physical=%p PageCount=0x%zx Address=0x%x PageTable=%p",
                   Os, Core, Physical, PageCount, Address, PageTable);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(PageCount > 0);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);

    /* Convert pointer to MDL. */
    mdl = (PLINUX_MDL)Physical;

    allocator = mdl->allocator;

    gcmkASSERT(allocator != gcvNULL);

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): Physical->0x%X PageCount->0x%X",
        __FUNCTION__, __LINE__,
        (gctUINT32)(gctUINTPTR_T)Physical,
        (gctUINT32)(gctUINTPTR_T)PageCount
        );

    table = (gctUINT32 *)PageTable;

    if (platform && platform->ops->getPolicyID)
    {
        platform->ops->getPolicyID(platform, Type, &policyID, &axiConfig);

        gcmkBUG_ON(policyID > 0x1F);

        /* ID[3:0] is used in STLB. */
        policyID &= 0xF;
    }

     /* Get all the physical addresses and store them in the page table. */

    PageCount = PageCount / (PAGE_SIZE / 4096);

    /* Try to get the user pages so DMA can happen. */
    while (PageCount-- > 0)
    {
        gctUINT i;
        gctPHYS_ADDR_T phys = ~0ULL;

        gcmALLOCATOR_Physical(allocator, mdl, offset, &phys);

        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Os, phys, &phys));

        if (policyID)
        {
            /* AxUSER must not used for address currently. */
            gcmkBUG_ON((phys >> 32) & 0xF);

            /* Merge policyID to AxUSER[7:4].*/
            phys |= ((gctPHYS_ADDR_T)policyID << 36);
        }

        {
            /* remove LSB. */
            phys &= ~(4096ull - 1);

            {
                for (i = 0; i < (PAGE_SIZE / 4096); i++)
                {
                    gcmkONERROR(
                        gckMMU_SetPage(Os->device->kernels[Core]->mmu,
                            phys + (i * 4096),
                            gcvPAGE_TYPE_4K,
                            Writable,
                            table++));
                }
            }
        }

        offset += PAGE_SIZE;
    }

    {
        gckMMU mmu = Os->device->kernels[Core]->mmu;
        gcsADDRESS_AREA * area = &mmu->dynamicArea4K;

        offset = (gctUINT8_PTR)PageTable - (gctUINT8_PTR)area->stlbLogical;

        /* must be in dynamic area. */
        gcmkASSERT(offset < area->stlbSize);

        gcmkVERIFY_OK(gckVIDMEM_NODE_CleanCache(
            Os->device->kernels[Core],
            area->stlbVideoMem,
            offset,
            PageTable,
            bytes
            ));

        if (mmu->mtlbVideoMem)
        {
            /* Flush MTLB table. */
            gcmkVERIFY_OK(gckVIDMEM_NODE_CleanCache(
                Os->device->kernels[Core],
                mmu->mtlbVideoMem,
                offset,
                mmu->mtlbLogical,
                mmu->mtlbSize
                ));
        }
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/* PageCount is GPU page count. */
gceSTATUS
gckOS_UnmapPages(
    IN gckOS Os,
    IN gctSIZE_T PageCount,
    IN gctUINT32 Address
    )
{
    return gcvSTATUS_OK;
}

/* Map 1M size GPU page */
gceSTATUS
gckOS_Map1MPages(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T PageCount,
    IN gctUINT32 Address,
    IN gctPOINTER PageTable,
    IN gctBOOL Writable,
    IN gceVIDMEM_TYPE Type
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    PLINUX_MDL mdl;
    gctUINT32* table;
    gctUINT32  offset = 0;
    gctSIZE_T bytes = PageCount * 4;
    gckALLOCATOR allocator;

    gctUINT32 policyID = 0;
    gctUINT32 axiConfig = 0;

    gcsPLATFORM * platform = Os->device->platform;

    gcmkHEADER_ARG("Os=%p Core=%d Physical=%p PageCount=0x%zx Address=0x%x PageTable=%p",
                   Os, Core, Physical, PageCount, Address, PageTable);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(PageCount > 0);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);

    /* Convert pointer to MDL. */
    mdl = (PLINUX_MDL)Physical;

    allocator = mdl->allocator;

    gcmkASSERT(allocator != gcvNULL);

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): Physical->0x%X PageCount->0x%X",
        __FUNCTION__, __LINE__,
        (gctUINT32)(gctUINTPTR_T)Physical,
        (gctUINT32)(gctUINTPTR_T)PageCount
        );

    table = (gctUINT32 *)PageTable;

    if (platform && platform->ops->getPolicyID)
    {
        platform->ops->getPolicyID(platform, Type, &policyID, &axiConfig);

        gcmkBUG_ON(policyID > 0x1F);

        /* ID[3:0] is used in STLB. */
        policyID &= 0xF;
    }

    while (PageCount-- > 0)
    {
        gctPHYS_ADDR_T phys = ~0ULL;

        gcmALLOCATOR_Physical(allocator, mdl, offset, &phys);

        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Os, phys, &phys));

        if (policyID)
        {
            /* AxUSER must not used for address currently. */
            gcmkBUG_ON((phys >> 32) & 0xF);

            /* Merge policyID to AxUSER[7:4].*/
            phys |= ((gctPHYS_ADDR_T)policyID << 36);
        }

        /* Get the start physical of 1M page. */
        phys &= ~(gcd1M_PAGE_SIZE - 1);

        gcmkONERROR(gckMMU_SetPage(
            Os->device->kernels[Core]->mmu,
            phys,
            gcvPAGE_TYPE_1M,
            Writable,
            table++
            ));

        offset += gcd1M_PAGE_SIZE;
    }

    /* Flush the page table cache. */
    {
        gckMMU mmu = Os->device->kernels[Core]->mmu;
        gcsADDRESS_AREA * area = &mmu->dynamicArea1M;

        offset = (gctUINT8_PTR)PageTable - (gctUINT8_PTR)area->stlbLogical;

        /* must be in dynamic area. */
        gcmkASSERT(offset < area->stlbSize);

        gcmkVERIFY_OK(gckVIDMEM_NODE_CleanCache(
            Os->device->kernels[Core],
            area->stlbVideoMem,
            offset,
            PageTable,
            bytes
            ));

        if (mmu->mtlbVideoMem)
        {
            /* Flush MTLB table. */
            gcmkVERIFY_OK(gckVIDMEM_NODE_CleanCache(
                Os->device->kernels[Core],
                mmu->mtlbVideoMem,
                offset,
                mmu->mtlbLogical,
                mmu->mtlbSize
                ));
        }
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_UnlockPages
**
**  Unlock memory allocated from the paged pool.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPHYS_ADDR Physical
**          Physical address of the allocation.
**
**      gctSIZE_T Bytes
**          Number of bytes of the allocation.
**
**      gctPOINTER Logical
**          Address of the mapped memory.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UnlockPages(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical
    )
{
    PLINUX_MDL_MAP mdlMap;
    PLINUX_MDL  mdl = (PLINUX_MDL)Physical;
    gckALLOCATOR allocator = mdl->allocator;
    gctINT pid = _GetProcessID();

    gcmkHEADER_ARG("Os=%p Physical=%p Bytes=0x%zx Logical=%p",
                   Os, Physical, Bytes, Logical);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);

    mutex_lock(&mdl->mapsMutex);

    list_for_each_entry(mdlMap, &mdl->mapsHead, link)
    {
        if ((mdlMap->vmaAddr != gcvNULL) && (mdlMap->pid == pid))
        {
            if (--mdlMap->count == 0)
            {
                gcmALLOCATOR_UnmapUser(
                    allocator,
                    mdl,
                    mdlMap,
                    mdl->bytes);

                mdlMap->vmaAddr = gcvNULL;
            }
        }
    }

    mutex_unlock(&mdl->mapsMutex);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_MapUserPointer
**
**  Map a pointer from the user process into the kernel address space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Pointer
**          Pointer in user process space that needs to be mapped.
**
**      gctSIZE_T Size
**          Number of bytes that need to be mapped.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Pointer to a variable receiving the mapped pointer in kernel address
**          space.
*/
gceSTATUS
gckOS_MapUserPointer(
    IN gckOS Os,
    IN gctPOINTER Pointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gcmkHEADER_ARG("Os=%p Pointer=%p Size=0x%zx", Os, Pointer, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);

    *KernelPointer = Pointer;

    gcmkFOOTER_ARG("*KernelPointer=%p", *KernelPointer);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_UnmapUserPointer
**
**  Unmap a user process pointer from the kernel address space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Pointer
**          Pointer in user process space that needs to be unmapped.
**
**      gctSIZE_T Size
**          Number of bytes that need to be unmapped.
**
**      gctPOINTER KernelPointer
**          Pointer in kernel address space that needs to be unmapped.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UnmapUserPointer(
    IN gckOS Os,
    IN gctPOINTER Pointer,
    IN gctSIZE_T Size,
    IN gctPOINTER KernelPointer
    )
{
    gcmkHEADER_ARG("Os=%p Pointer=%p Size=0x%zx KernelPointer=%p",
                   Os, Pointer, Size, KernelPointer);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_QueryNeedCopy
**
**  Query whether the memory can be accessed or mapped directly or it has to be
**  copied.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 ProcessID
**          Process ID of the current process.
**
**  OUTPUT:
**
**      gctBOOL_PTR NeedCopy
**          Pointer to a boolean receiving gcvTRUE if the memory needs a copy or
**          gcvFALSE if the memory can be accessed or mapped dircetly.
*/
gceSTATUS
gckOS_QueryNeedCopy(
    IN gckOS Os,
    IN gctUINT32 ProcessID,
    OUT gctBOOL_PTR NeedCopy
    )
{
    gcmkHEADER_ARG("Os=%p ProcessID=%d", Os, ProcessID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(NeedCopy != gcvNULL);

    /* We need to copy data. */
    *NeedCopy = gcvTRUE;

    /* Success. */
    gcmkFOOTER_ARG("*NeedCopy=%d", *NeedCopy);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_CopyFromUserData
**
**  Copy data from user to kernel memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER KernelPointer
**          Pointer to kernel memory.
**
**      gctPOINTER Pointer
**          Pointer to user memory.
**
**      gctSIZE_T Size
**          Number of bytes to copy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_CopyFromUserData(
    IN gckOS Os,
    IN gctPOINTER KernelPointer,
    IN gctPOINTER Pointer,
    IN gctSIZE_T Size
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p KernelPointer=%p Pointer=%p Size=0x%zx",
                   Os, KernelPointer, Pointer, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    /* Copy data from user. */
    if (copy_from_user(KernelPointer, Pointer, Size) != 0)
    {
        /* Could not copy all the bytes. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_CopyToUserData
**
**  Copy data from kernel to user memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER KernelPointer
**          Pointer to kernel memory.
**
**      gctPOINTER Pointer
**          Pointer to user memory.
**
**      gctSIZE_T Size
**          Number of bytes to copy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_CopyToUserData(
    IN gckOS Os,
    IN gctPOINTER KernelPointer,
    IN gctPOINTER Pointer,
    IN gctSIZE_T Size
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p KernelPointer=%p Pointer=%p Size=0x%zx",
                   Os, KernelPointer, Pointer, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    /* Copy data to user. */
    if (copy_to_user(Pointer, KernelPointer, Size) != 0)
    {
        /* Could not copy all the bytes. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_WriteMemory
**
**  Write data to a memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Address
**          Address of the memory to write to.
**
**      gctUINT32 Data
**          Data for register.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_WriteMemory(
    IN gckOS Os,
    IN gctPOINTER Address,
    IN gctUINT32 Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Address=%p Data=%u", Os, Address, Data);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Address != gcvNULL);

    /* Write memory. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
    if (access_ok(Address, 4))
#else
    if (access_ok(VERIFY_WRITE, Address, 4))
#endif
    {
        /* User address. */
        if (put_user(Data, (gctUINT32*)Address))
        {
            gcmkONERROR(gcvSTATUS_INVALID_ADDRESS);
        }
    }
    else
    {
        /* don't check the virtual address, maybe it come from io memory or reserved memory */
        /* Kernel address. */
        *(gctUINT32 *)Address = Data;
    }

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckOS_ReadMappedPointer(
    IN gckOS Os,
    IN gctPOINTER Address,
    IN gctUINT32_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcmkHEADER_ARG("Os=%p Address=%p Data=%u", Os, Address, Data);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Address != gcvNULL);

    /* Write memory. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
    if (access_ok(Address, 4))
#else
    if (access_ok(VERIFY_READ, Address, 4))
#endif
    {
        /* User address. */
        if (get_user(*Data, (gctUINT32*)Address))
        {
            gcmkONERROR(gcvSTATUS_INVALID_ADDRESS);
        }
    }
    else
    {
        /* Kernel address. */
        *Data = *(gctUINT32_PTR)Address;
    }

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_GetBaseAddress
**
**  Get the base address for the physical memory.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**  OUTPUT:
**
**      gctUINT32_PTR BaseAddress
**          Pointer to a variable that will receive the base address.
*/
gceSTATUS
gckOS_GetBaseAddress(
    IN gckOS Os,
    OUT gctUINT32_PTR BaseAddress
    )
{
    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(BaseAddress != gcvNULL);

    /* Return base address. */
    *BaseAddress = Os->device->baseAddress;

    /* Success. */
    gcmkFOOTER_ARG("*BaseAddress=0x%08x", *BaseAddress);
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_SuspendInterrupt(
    IN gckOS Os
    )
{
    return gckOS_SuspendInterruptEx(Os, gcvCORE_MAJOR);
}

gceSTATUS
gckOS_SuspendInterruptEx(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    gcmkHEADER_ARG("Os=%p Core=%d", Os, Core);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    if (Os->device->irqLines[Core] != -1)
    {
        disable_irq(Os->device->irqLines[Core]);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_ResumeInterrupt(
    IN gckOS Os
    )
{
    return gckOS_ResumeInterruptEx(Os, gcvCORE_MAJOR);
}

gceSTATUS
gckOS_ResumeInterruptEx(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    gcmkHEADER_ARG("Os=%p Core=%d", Os, Core);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    if (Os->device->irqLines[Core] != -1)
    {
        enable_irq(Os->device->irqLines[Core]);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_MemCopy(
    IN gctPOINTER Destination,
    IN gctCONST_POINTER Source,
    IN gctSIZE_T Bytes
    )
{
    gcmkHEADER_ARG("Destination=%p Source=%p Bytes=0x%zx",
                   Destination, Source, Bytes);

    gcmkVERIFY_ARGUMENT(Destination != gcvNULL);
    gcmkVERIFY_ARGUMENT(Source != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);

    memcpy(Destination, Source, Bytes);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_ZeroMemory(
    IN gctPOINTER Memory,
    IN gctSIZE_T Bytes
    )
{
    gcmkHEADER_ARG("Memory=%p Bytes=0x%zx", Memory, Bytes);

    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);

    memset(Memory, 0, Bytes);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
********************************* Cache Control ********************************
*******************************************************************************/
static gceSTATUS
_CacheOperation(
    IN gckOS Os,
    IN gctUINT32 ProcessID,
    IN gctPHYS_ADDR Handle,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    PLINUX_MDL mdl = (PLINUX_MDL)Handle;
    PLINUX_MDL_MAP mdlMap;
    gckALLOCATOR allocator;

    if (!mdl || !mdl->allocator)
    {
        gcmkPRINT("[galcore]: %s: Logical=%p no mdl", __FUNCTION__, Logical);
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    allocator = mdl->allocator;

    if (allocator->ops->Cache)
    {
        mutex_lock(&mdl->mapsMutex);

        mdlMap = FindMdlMap(mdl, ProcessID);

        mutex_unlock(&mdl->mapsMutex);

        if (ProcessID && !mdlMap && !mdl->wrapFromPhysical && !mdl->wrapFromLogical)
        {
            return gcvSTATUS_INVALID_ARGUMENT;
        }

        if ((!ProcessID && mdl->cacheable) ||
            (mdlMap && mdlMap->cacheable)  ||
            mdl->wrapFromLogical)
        {
            gcmALLOCATOR_Cache(allocator,
                mdl, Offset, Logical, Bytes, Operation);

            if (Operation == gcvCACHE_CLEAN || Operation == gcvCACHE_FLUSH)
            {
                _ExternalCacheOperation(Os, gcvCACHE_INVALIDATE);
            }

            return gcvSTATUS_OK;
        }
    }

    _MemoryBarrier();

    if (Operation == gcvCACHE_CLEAN || Operation == gcvCACHE_FLUSH)
    {
        _ExternalCacheOperation(Os, gcvCACHE_INVALIDATE);
    }

    return gcvSTATUS_OK;
}

/*******************************************************************************
**  gckOS_CacheClean
**
**  Clean the cache for the specified addresses.  The GPU is going to need the
**  data.  If the system is allocating memory as non-cachable, this function can
**  be ignored.
**
**  ARGUMENTS:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctUINT32 ProcessID
**          Process ID Logical belongs.
**
**      gctPHYS_ADDR Handle
**          Physical address handle.  If gcvNULL it is video memory.
**
**      gctSIZE_T Offset
**          Offset to this memory block.
**
**      gctPOINTER Logical
**          Logical address to flush.
**
**      gctSIZE_T Bytes
**          Size of the address range in bytes to flush.
*/

/*

Following patch can be applied to kernel in case cache API is not exported.

diff --git a/arch/arm/mm/proc-syms.c b/arch/arm/mm/proc-syms.c
index 054b491..e9e74ec 100644
--- a/arch/arm/mm/proc-syms.c
+++ b/arch/arm/mm/proc-syms.c
@@ -30,6 +30,9 @@ EXPORT_SYMBOL(__cpuc_flush_user_all);
 EXPORT_SYMBOL(__cpuc_flush_user_range);
 EXPORT_SYMBOL(__cpuc_coherent_kern_range);
 EXPORT_SYMBOL(__cpuc_flush_dcache_area);
+EXPORT_SYMBOL(__glue(_CACHE,_dma_map_area));
+EXPORT_SYMBOL(__glue(_CACHE,_dma_unmap_area));
+EXPORT_SYMBOL(__glue(_CACHE,_dma_flush_range));
 #else
 EXPORT_SYMBOL(cpu_cache);
 #endif

*/
gceSTATUS
gckOS_CacheClean(
    IN gckOS Os,
    IN gctUINT32 ProcessID,
    IN gctPHYS_ADDR Handle,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Os=%p ProcessID=%d Handle=%p Offset=0x%llx Logical=%p Bytes=0x%zx",
                   Os, ProcessID, Handle, Offset, Logical, Bytes);

    gcmkONERROR(_CacheOperation(Os, ProcessID,
                                Handle, Offset, Logical, Bytes,
                                gcvCACHE_CLEAN));

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckOS_CacheInvalidate
**
**  Invalidate the cache for the specified addresses. The GPU is going to need
**  data.  If the system is allocating memory as non-cachable, this function can
**  be ignored.
**
**  ARGUMENTS:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctUINT32 ProcessID
**          Process ID Logical belongs.
**
**      gctPHYS_ADDR Handle
**          Physical address handle.  If gcvNULL it is video memory.
**
**      gctPOINTER Logical
**          Logical address to flush.
**
**      gctSIZE_T Bytes
**          Size of the address range in bytes to flush.
*/
gceSTATUS
gckOS_CacheInvalidate(
    IN gckOS Os,
    IN gctUINT32 ProcessID,
    IN gctPHYS_ADDR Handle,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Os=%p ProcessID=%d Handle=%p Offset=0x%llx Logical=%p Bytes=0x%zx",
                   Os, ProcessID, Handle, Offset, Logical, Bytes);

    gcmkONERROR(_CacheOperation(Os, ProcessID,
                                Handle, Offset, Logical, Bytes,
                                gcvCACHE_INVALIDATE));

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckOS_CacheFlush
**
**  Clean the cache for the specified addresses and invalidate the lines as
**  well.  The GPU is going to need and modify the data.  If the system is
**  allocating memory as non-cachable, this function can be ignored.
**
**  ARGUMENTS:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctUINT32 ProcessID
**          Process ID Logical belongs.
**
**      gctPHYS_ADDR Handle
**          Physical address handle.  If gcvNULL it is video memory.
**
**      gctPOINTER Logical
**          Logical address to flush.
**
**      gctSIZE_T Bytes
**          Size of the address range in bytes to flush.
*/
gceSTATUS
gckOS_CacheFlush(
    IN gckOS Os,
    IN gctUINT32 ProcessID,
    IN gctPHYS_ADDR Handle,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Os=%p ProcessID=%d Handle=%p Offset=0x%llx Logical=%p Bytes=0x%zx",
                   Os, ProcessID, Handle, Offset, Logical, Bytes);

    gcmkONERROR(_CacheOperation(Os, ProcessID,
                                Handle, Offset, Logical, Bytes,
                                gcvCACHE_FLUSH));

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
********************************* Broadcasting *********************************
*******************************************************************************/

/*******************************************************************************
**
**  gckOS_Broadcast
**
**  System hook for broadcast events from the kernel driver.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**      gceBROADCAST Reason
**          Reason for the broadcast.  Can be one of the following values:
**
**              gcvBROADCAST_GPU_IDLE
**                  Broadcasted when the kernel driver thinks the GPU might be
**                  idle.  This can be used to handle power management.
**
**              gcvBROADCAST_GPU_COMMIT
**                  Broadcasted when any client process commits a command
**                  buffer.  This can be used to handle power management.
**
**              gcvBROADCAST_GPU_STUCK
**                  Broadcasted when the kernel driver hits the timeout waiting
**                  for the GPU.
**
**              gcvBROADCAST_FIRST_PROCESS
**                  First process is trying to connect to the kernel.
**
**              gcvBROADCAST_LAST_PROCESS
**                  Last process has detached from the kernel.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Broadcast(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gceBROADCAST Reason
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gceCHIPPOWERSTATE state;

    gcmkHEADER_ARG("Os=%p Hardware=%p Reason=%d", Os, Hardware, Reason);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    switch (Reason)
    {
    case gcvBROADCAST_FIRST_PROCESS:
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS, "First process has attached");
        break;

    case gcvBROADCAST_LAST_PROCESS:
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS, "Last process has detached");

        /* Put GPU OFF. */
        gcmkONERROR(
            gckHARDWARE_SetPowerState(Hardware,
                                      gcvPOWER_OFF_BROADCAST));
        break;

    case gcvBROADCAST_GPU_IDLE:
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS, "GPU idle.");
#if gcdPOWER_SUSPEND_WHEN_IDLE
        state = gcvPOWER_SUSPEND_BROADCAST;
#else
        state = gcvPOWER_IDLE_BROADCAST;
#endif

        /* Put GPU IDLE or SUSPEND. */
        gcmkONERROR(
            gckHARDWARE_SetPowerState(Hardware, state));

        /* Add idle process DB. */
        gcmkONERROR(gckKERNEL_AddProcessDB(Hardware->kernel,
                                           1,
                                           gcvDB_IDLE,
                                           gcvNULL, gcvNULL, 0));
        break;

    case gcvBROADCAST_GPU_COMMIT:
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS, "COMMIT has arrived.");

        /* Add busy process DB. */
        gcmkONERROR(gckKERNEL_AddProcessDB(Hardware->kernel,
                                           0,
                                           gcvDB_IDLE,
                                           gcvNULL, gcvNULL, 0));

#if gcdENABLE_PER_DEVICE_PM
        if (Hardware->type == gcvHARDWARE_3D ||
            Hardware->type == gcvHARDWARE_3D2D ||
            Hardware->type == gcvHARDWARE_VIP)
        {
            gckKERNEL kernel = Hardware->kernel;
            gckDEVICE device = kernel->device;
            gctUINT32 broCoreMask;
            gctUINT i;

            gcmkONERROR(gckOS_AcquireMutex(Hardware->os, device->powerMutex, gcvINFINITE));

            gcmkVERIFY_OK(gckOS_AtomGet(Hardware->os, kernel->atomBroCoreMask, (gctINT32_PTR)&broCoreMask));

            /* I am along. */
            if ((gceCORE)broCoreMask == Hardware->core)
            {
                /* Put GPU ON. */
                gcmkONERROR(
                    gckHARDWARE_SetPowerState(Hardware, gcvPOWER_ON_AUTO));
            }
            else
            {
                /* Power on all the brother cores. */
                for (i = 0; i < device->coreNum; i++)
                {
                    kernel = device->coreInfoArray[i].kernel;

                    if ((1 << i) & broCoreMask)
                    {
                        /* Put GPU ON. */
                        gcmkONERROR(
                            gckHARDWARE_SetPowerState(kernel->hardware, gcvPOWER_ON_AUTO));
                    }
                }
            }

            gcmkONERROR(gckOS_ReleaseMutex(Hardware->os, device->powerMutex));
        }
        else
#endif
        {
            /* Put GPU ON. */
            gcmkONERROR(
                gckHARDWARE_SetPowerState(Hardware, gcvPOWER_ON_AUTO));
        }

        break;

    case gcvBROADCAST_GPU_STUCK:
        gcmkTRACE_N(gcvLEVEL_ERROR, 0, "gcvBROADCAST_GPU_STUCK\n");
        gcmkONERROR(gckKERNEL_Recovery(Hardware->kernel));
        break;

    case gcvBROADCAST_AXI_BUS_ERROR:
        gcmkTRACE_N(gcvLEVEL_ERROR, 0, "gcvBROADCAST_AXI_BUS_ERROR\n");
        gcmkONERROR(gckHARDWARE_DumpGPUState(Hardware));
        gcmkONERROR(gckKERNEL_Recovery(Hardware->kernel));
        break;

    case gcvBROADCAST_OUT_OF_MEMORY:
        gcmkTRACE_N(gcvLEVEL_INFO, 0, "gcvBROADCAST_OUT_OF_MEMORY\n");

        status = _ShrinkMemory(Os);

        if (status == gcvSTATUS_NOT_SUPPORTED)
        {
            goto OnError;
        }

        gcmkONERROR(status);

        break;

    default:
        /* Skip unimplemented broadcast. */
        break;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_BroadcastHurry
**
**  The GPU is running too slow.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**      gctUINT Urgency
**          The higher the number, the higher the urgency to speed up the GPU.
**          The maximum value is defined by the gcdDYNAMIC_EVENT_THRESHOLD.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_BroadcastHurry(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT Urgency
    )
{
    gcmkHEADER_ARG("Os=%p Hardware=%p Urgency=%u", Os, Hardware, Urgency);

    /* Do whatever you need to do to speed up the GPU now. */

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_BroadcastCalibrateSpeed
**
**  Calibrate the speed of the GPU.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**      gctUINT Idle, Time
**          Idle/Time will give the percentage the GPU is idle, so you can use
**          this to calibrate the working point of the GPU.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_BroadcastCalibrateSpeed(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT Idle,
    IN gctUINT Time
    )
{
    gcmkHEADER_ARG("Os=%p Hardware=%p Idle=%u Time=%u",
                   Os, Hardware, Idle, Time);

    /* Do whatever you need to do to callibrate the GPU speed. */

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
********************************** Semaphores **********************************
*******************************************************************************/

/*******************************************************************************
**
**  gckOS_CreateSemaphore
**
**  Create a semaphore.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**  OUTPUT:
**
**      gctPOINTER * Semaphore
**          Pointer to the variable that will receive the created semaphore.
*/
gceSTATUS
gckOS_CreateSemaphore(
    IN gckOS Os,
    OUT gctPOINTER * Semaphore
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    struct semaphore *sem = gcvNULL;

    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    /* Allocate the semaphore structure. */
    sem = (struct semaphore *)kmalloc(gcmSIZEOF(struct semaphore), GFP_KERNEL | gcdNOWARN);
    if (sem == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Initialize the semaphore. */
    sema_init(sem, 1);

    /* Return to caller. */
    *Semaphore = (gctPOINTER) sem;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_AcquireSemaphore
**
**  Acquire a semaphore.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Semaphore
**          Pointer to the semaphore thet needs to be acquired.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_AcquireSemaphore(
    IN gckOS Os,
    IN gctPOINTER Semaphore
    )
{
    gcmkHEADER_ARG("Os=%p Semaphore=%p", Os, Semaphore);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    /* Acquire the semaphore. */
    down((struct semaphore *) Semaphore);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_TryAcquireSemaphore
**
**  Try to acquire a semaphore.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Semaphore
**          Pointer to the semaphore thet needs to be acquired.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_TryAcquireSemaphore(
    IN gckOS Os,
    IN gctPOINTER Semaphore
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    /* Acquire the semaphore. */
    if (down_trylock((struct semaphore *) Semaphore))
    {
        /* Timeout. */
        status = gcvSTATUS_TIMEOUT;
    }

    /* Success. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_ReleaseSemaphore
**
**  Release a previously acquired semaphore.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Semaphore
**          Pointer to the semaphore thet needs to be released.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_ReleaseSemaphore(
    IN gckOS Os,
    IN gctPOINTER Semaphore
    )
{
    gcmkHEADER_ARG("Os=%p Semaphore=%p", Os, Semaphore);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    /* Release the semaphore. */
    up((struct semaphore *) Semaphore);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_ReleaseSemaphoreEx(
    IN gckOS Os,
    IN gctPOINTER Semaphore
    )
{
    struct semaphore *sem;
    unsigned long flags;

    gcmkHEADER_ARG("Os=%p Semaphore=%p", Os, Semaphore);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    sem = Semaphore;

    raw_spin_lock_irqsave(&sem->lock, flags);

    if (!sem->count)
    {
        raw_spin_unlock_irqrestore(&sem->lock, flags);
        up((struct semaphore *) Semaphore);
    }
    else
    {
        raw_spin_unlock_irqrestore(&sem->lock, flags);
    }


    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_DestroySemaphore
**
**  Destroy a semaphore.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Semaphore
**          Pointer to the semaphore thet needs to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_DestroySemaphore(
    IN gckOS Os,
    IN gctPOINTER Semaphore
    )
{
    gcmkHEADER_ARG("Os=%p Semaphore=%p", Os, Semaphore);

     /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Semaphore != gcvNULL);

    /* Free the sempahore structure. */
    kfree(Semaphore);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_GetProcessID
**
**  Get current process ID.
**
**  INPUT:
**
**      Nothing.
**
**  OUTPUT:
**
**      gctUINT32_PTR ProcessID
**          Pointer to the variable that receives the process ID.
*/
gceSTATUS
gckOS_GetProcessID(
    OUT gctUINT32_PTR ProcessID
    )
{
    /* Get process ID. */
    *ProcessID = _GetProcessID();

    /* Success. */
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_GetThreadID
**
**  Get current thread ID.
**
**  INPUT:
**
**      Nothing.
**
**  OUTPUT:
**
**      gctUINT32_PTR ThreadID
**          Pointer to the variable that receives the thread ID.
*/
gceSTATUS
gckOS_GetThreadID(
    OUT gctUINT32_PTR ThreadID
    )
{
    /* Get thread ID. */
    if (ThreadID != gcvNULL)
    {
        *ThreadID = _GetThreadID();
    }

    /* Success. */
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_SetClockState
**
**  Set the clock state on or off.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gceCORE Core
**          GPU whose power is set.
**
**      gctBOOL Clock
**          gcvTRUE to turn on the clock, or gcvFALSE to turn off the clock.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_SetClockState(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctBOOL Clock
    )
{
    gctBOOL clockChange = gcvFALSE;

    gcmkHEADER_ARG("Os=%p Core=%d Clock=%d", Os, Core, Clock);
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    clockChange = (Clock != Os->clockStates[Core]);

    if (clockChange)
    {
        unsigned long flags;

        if (!Clock)
        {
            spin_lock_irqsave(&Os->registerAccessLock, flags);

            /* Record clock off, ahead. */
            Os->clockStates[Core] = gcvFALSE;

            spin_unlock_irqrestore(&Os->registerAccessLock, flags);
        }


        if (Clock)
        {
            spin_lock_irqsave(&Os->registerAccessLock, flags);

            /* Record clock on, behind. */
            Os->clockStates[Core] = gcvTRUE;

            spin_unlock_irqrestore(&Os->registerAccessLock, flags);
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_GetClockState
**
**  Get the clock state on or off.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gceCORE Core
**          GPU whose power is set.
**
**      gctBOOL Clock
**          gcvTRUE to turn on the clock, or gcvFALSE to turn off the clock.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_GetClockState(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctBOOL * Clock
    )
{
    *Clock = Os->clockStates[Core];

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_SetGPUPower
**
**  Set the power of the GPU on or off.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gceCORE Core
**          GPU whose power is set.
**
**      gctBOOL Clock
**          gcvTRUE to turn on the clock, or gcvFALSE to turn off the clock.
**
**      gctBOOL Power
**          gcvTRUE to turn on the power, or gcvFALSE to turn off the power.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_SetGPUPower(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctBOOL Clock,
    IN gctBOOL Power
    )
{
    gcsPLATFORM * platform;

    gctBOOL powerChange = gcvFALSE;
    gctBOOL clockChange = gcvFALSE;

    gcmkHEADER_ARG("Os=%p Core=%d Clock=%d Power=%d", Os, Core, Clock, Power);
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    platform = Os->device->platform;

    powerChange = (Power != Os->powerStates[Core]);

    clockChange = (Clock != Os->clockStates[Core]);

    if (powerChange && (Power == gcvTRUE))
    {
        if (platform && platform->ops->setPower)
        {
            gcmkVERIFY_OK(platform->ops->setPower(platform, Core, Power));
        }

        Os->powerStates[Core] = Power;
    }

    if (clockChange)
    {
        unsigned long flags;

        if (!Clock)
        {
            spin_lock_irqsave(&Os->registerAccessLock, flags);

            /* Record clock off, ahead. */
            Os->clockStates[Core] = gcvFALSE;

            spin_unlock_irqrestore(&Os->registerAccessLock, flags);
        }

        if (platform && platform->ops->setClock)
        {
            gcmkVERIFY_OK(platform->ops->setClock(platform, Core, Clock));
        }

        if (Clock)
        {
            spin_lock_irqsave(&Os->registerAccessLock, flags);

            /* Record clock on, behind. */
            Os->clockStates[Core] = gcvTRUE;

            spin_unlock_irqrestore(&Os->registerAccessLock, flags);
        }
    }

    if (powerChange && (Power == gcvFALSE))
    {
        if (platform && platform->ops->setPower)
        {
            gcmkVERIFY_OK(platform->ops->setPower(platform, Core, Power));
        }

        Os->powerStates[Core] = Power;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_ResetGPU
**
**  Reset the GPU.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gckCORE Core
**          GPU whose power is set.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_ResetGPU(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gcsPLATFORM * platform;

    gcmkHEADER_ARG("Os=%p Core=%d", Os, Core);
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    platform = Os->device->platform;

    if (platform && platform->ops->reset)
    {
        status = platform->ops->reset(platform, Core);
    }

    gcmkFOOTER_NO();
    return status;
}

/*******************************************************************************
**
**  gckOS_PrepareGPUFrequency
**
**  Prepare to set GPU frequency and voltage.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gckCORE Core
**          GPU whose frequency and voltage will be set.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_PrepareGPUFrequency(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_FinishGPUFrequency
**
**  Finish GPU frequency setting.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gckCORE Core
**          GPU whose frequency and voltage is set.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_FinishGPUFrequency(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_QueryGPUFrequency
**
**  Query the current frequency of the GPU.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gckCORE Core
**          GPU whose power is set.
**
**      gctUINT32 * Frequency
**          Pointer to a gctUINT32 to obtain current frequency, in MHz.
**
**      gctUINT8 * Scale
**          Pointer to a gctUINT8 to obtain current scale(1 - 64).
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_QueryGPUFrequency(
    IN gckOS Os,
    IN gceCORE Core,
    OUT gctUINT32 * Frequency,
    OUT gctUINT8 * Scale
    )
{
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_SetGPUFrequency
**
**  Set frequency and voltage of the GPU.
**
**      1. DVFS manager gives the target scale of full frequency, BSP must find
**         a real frequency according to this scale and board's configure.
**
**      2. BSP should find a suitable voltage for this frequency.
**
**      3. BSP must make sure setting take effect before this function returns.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to a gckOS object.
**
**      gckCORE Core
**          GPU whose power is set.
**
**      gctUINT8 Scale
**          Target scale of full frequency, range is [1, 64]. 1 means 1/64 of
**          full frequency and 64 means 64/64 of full frequency.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_SetGPUFrequency(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT8 Scale
    )
{
    return gcvSTATUS_OK;
}

/*----------------------------------------------------------------------------*/
/*----- Profile --------------------------------------------------------------*/

gceSTATUS
gckOS_GetProfileTick(
    OUT gctUINT64_PTR Tick
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,5,0)
    struct timespec64 time;

    ktime_get_ts64(&time);
#else
    struct timespec time;

    ktime_get_ts(&time);
#endif
    *Tick = time.tv_nsec + time.tv_sec * 1000000000ULL;

    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_QueryProfileTickRate(
    OUT gctUINT64_PTR TickRate
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,5,0)
    struct timespec64 res;
#else
    struct timespec res;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)
    res.tv_sec = 0;
    res.tv_nsec = hrtimer_resolution;
#else
    hrtimer_get_res(CLOCK_MONOTONIC, &res);
#endif

    *TickRate = res.tv_nsec + res.tv_sec * 1000000000ULL;

    return gcvSTATUS_OK;
}

gctUINT32
gckOS_ProfileToMS(
    IN gctUINT64 Ticks
    )
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
    return div_u64(Ticks, 1000000);
#else
    gctUINT64 rem = Ticks;
    gctUINT64 b = 1000000;
    gctUINT64 res, d = 1;
    gctUINT32 high = rem >> 32;

    /* Reduce the thing a bit first */
    res = 0;
    if (high >= 1000000)
    {
        high /= 1000000;
        res   = (gctUINT64) high << 32;
        rem  -= (gctUINT64) (high * 1000000) << 32;
    }

    while (((gctINT64) b > 0) && (b < rem))
    {
        b <<= 1;
        d <<= 1;
    }

    do
    {
        if (rem >= b)
        {
            rem -= b;
            res += d;
        }

        b >>= 1;
        d >>= 1;
    }
    while (d);

    return (gctUINT32) res;
#endif
}

/******************************************************************************\
******************************* Signal Management ******************************
\******************************************************************************/

#undef _GC_OBJ_ZONE
#define _GC_OBJ_ZONE    gcvZONE_SIGNAL

/*******************************************************************************
**
**  gckOS_CreateSignal
**
**  Create a new signal.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctBOOL ManualReset
**          If set to gcvTRUE, gckOS_Signal with gcvFALSE must be called in
**          order to set the signal to nonsignaled state.
**          If set to gcvFALSE, the signal will automatically be set to
**          nonsignaled state by gckOS_WaitSignal function.
**
**  OUTPUT:
**
**      gctSIGNAL * Signal
**          Pointer to a variable receiving the created gctSIGNAL.
*/
gceSTATUS
gckOS_CreateSignal(
    IN gckOS Os,
    IN gctBOOL ManualReset,
    OUT gctSIGNAL * Signal
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsSIGNAL_PTR signal = gcvNULL;

    gcmkHEADER_ARG("Os=%p ManualReset=%d", Os, ManualReset);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Signal != gcvNULL);

    /* Create an event structure. */
    signal = (gcsSIGNAL_PTR) kmalloc(sizeof(gcsSIGNAL), GFP_KERNEL | gcdNOWARN);

    if (signal == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Save the process ID. */
    signal->process = (gctHANDLE)(gctUINTPTR_T) _GetProcessID();

    signal->done = 0;
    init_waitqueue_head(&signal->wait);
    spin_lock_init(&signal->lock);
    signal->manualReset = ManualReset;

    atomic_set(&signal->ref, 1);

#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
    signal->timeline = gcvNULL;
#  else
    signal->fence = gcvNULL;
#  endif
#endif

    gcmkONERROR(_AllocateIntegerId(&Os->signalDB, signal, &signal->id));

    *Signal = (gctSIGNAL)(gctUINTPTR_T)signal->id;

OnError:
    if (gcmIS_ERROR(status) && signal)
    {
        kfree(signal);
    }

    gcmkFOOTER_ARG("*Signal=%p", *Signal);
    return status;
}

/*******************************************************************************
**
**  gckOS_DestroySignal
**
**  Destroy a signal.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_DestroySignal(
    IN gckOS Os,
    IN gctSIGNAL Signal
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsSIGNAL_PTR signal;
    gctBOOL acquired = gcvFALSE;
    unsigned long flags = 0;

    gcmkHEADER_ARG("Os=%p Signal=%p", Os, Signal);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Signal != gcvNULL);

    if(in_irq()){
        spin_lock(&Os->signalLock);
    }else{
        spin_lock_irqsave(&Os->signalLock, flags);
    }
    acquired = gcvTRUE;

    gcmkONERROR(_QueryIntegerId(&Os->signalDB, (gctUINT32)(gctUINTPTR_T)Signal, (gctPOINTER)&signal));

    gcmkASSERT(signal->id == (gctUINT32)(gctUINTPTR_T)Signal);

    if (atomic_dec_and_test(&signal->ref))
    {
        gcmkVERIFY_OK(_DestroyIntegerId(&Os->signalDB, signal->id));

        /* Free the sgianl. */
        kfree(signal);
    }

    if(in_irq()){
        spin_unlock(&Os->signalLock);
    }else{
        spin_unlock_irqrestore(&Os->signalLock, flags);
    }
    acquired = gcvFALSE;

OnError:
    if (acquired)
    {
        /* Release the mutex. */
        if(in_irq()){
            spin_unlock(&Os->signalLock);
        }else{
            spin_unlock_irqrestore(&Os->signalLock, flags);
        }
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_Signal
**
**  Set a state of the specified signal.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**      gctBOOL State
**          If gcvTRUE, the signal will be set to signaled state.
**          If gcvFALSE, the signal will be set to nonsignaled state.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_Signal(
    IN gckOS Os,
    IN gctSIGNAL Signal,
    IN gctBOOL State
    )
{
    gceSTATUS status;
    gcsSIGNAL_PTR signal;
#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
    struct sync_timeline * timeline = gcvNULL;
#  else
    struct dma_fence * fence = gcvNULL;
#  endif
#endif
    unsigned long flags = 0;

    gcmkHEADER_ARG("Os=%p Signal=%p State=%d", Os, Signal, State);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Signal != gcvNULL);

    spin_lock_irqsave(&Os->signalLock, flags);

    status = _QueryIntegerId(&Os->signalDB,
                             (gctUINT32)(gctUINTPTR_T)Signal,
                             (gctPOINTER)&signal);

    if (gcmIS_ERROR(status))
    {
        spin_unlock_irqrestore(&Os->signalLock, flags);
        gcmkONERROR(status);
    }

    /*
     * Signal saved in event is not referenced. Inc reference here to avoid
     * concurrent issue: signaling the signal while another thread is destroying
     * it.
     */
    atomic_inc(&signal->ref);

    spin_unlock_irqrestore(&Os->signalLock, flags);

    gcmkONERROR(status);

    gcmkASSERT(signal->id == (gctUINT32)(gctUINTPTR_T)Signal);

    spin_lock(&signal->lock);

    if (State)
    {
        signal->done = 1;

        wake_up(&signal->wait);

#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
        timeline = signal->timeline;
#  else
        fence = signal->fence;
        signal->fence = NULL;
#  endif
#endif
    }
    else
    {
        signal->done = 0;
    }

    spin_unlock(&signal->lock);

#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
    /* Signal timeline. */
    if (timeline)
    {
        sync_timeline_signal(timeline);
    }
#  else
    if (fence)
    {
        dma_fence_signal(fence);
        dma_fence_put(fence);
    }
#  endif
#endif

    spin_lock_irqsave(&Os->signalLock, flags);

    if (atomic_dec_and_test(&signal->ref))
    {
        gcmkVERIFY_OK(_DestroyIntegerId(&Os->signalDB, signal->id));

        /* Free the sgianl. */
        kfree(signal);
    }

    spin_unlock_irqrestore(&Os->signalLock, flags);

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_UserSignal
**
**  Set the specified signal which is owned by a process to signaled state.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**      gctHANDLE Process
**          Handle of process owning the signal.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_UserSignal(
    IN gckOS Os,
    IN gctSIGNAL Signal,
    IN gctHANDLE Process
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Os=%p Signal=%p Process=%p", Os, Signal, Process);

    /* Signal. */
    status = gckOS_Signal(Os, Signal, gcvTRUE);

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_WaitSignal
**
**  Wait for a signal to become signaled.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to the gctSIGNAL.
**
**      gctUINT32 Wait
**          Number of milliseconds to wait.
**          Pass the value of gcvINFINITE for an infinite wait.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_WaitSignal(
    IN gckOS Os,
    IN gctSIGNAL Signal,
    IN gctBOOL Interruptable,
    IN gctUINT32 Wait
    )
{
    gceSTATUS status;
    gcsSIGNAL_PTR signal = gcvNULL;
    int done;

    gcmkHEADER_ARG("Os=%p Signal=%p Wait=0x%08X", Os, Signal, Wait);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Signal != gcvNULL);

    gcmkONERROR(_QueryIntegerId(&Os->signalDB, (gctUINT32)(gctUINTPTR_T)Signal, (gctPOINTER)&signal));

    gcmkASSERT(signal->id == (gctUINT32)(gctUINTPTR_T)Signal);

    spin_lock(&signal->lock);
    done = signal->done;
    spin_unlock(&signal->lock);

    /*
     * Do not need to lock below:
     * 1. If signal already done, return immediately.
     * 2. If signal not done, wait_event_xxx will handle correctly even read of
     *    signal->done is not atomic.
     *
     * Rest signal->done do not require lock either:
     * No other thread can query/wait auto-reseted signal, because that is
     * logic error.
     */
    if (done)
    {
        status = gcvSTATUS_OK;

        if (!signal->manualReset)
        {
            signal->done = 0;
        }
    }
    else if (Wait == 0)
    {
        status = gcvSTATUS_TIMEOUT;
    }
    else
    {
        /* Convert wait to milliseconds. */
        long timeout = (Wait == gcvINFINITE)
                     ? MAX_SCHEDULE_TIMEOUT
                     : msecs_to_jiffies(Wait);

        long ret;

        if (Interruptable)
        {
            ret = wait_event_interruptible_timeout(signal->wait, signal->done, timeout);
        }
        else
        {
            ret = wait_event_timeout(signal->wait, signal->done, timeout);
        }

        if (likely(ret > 0))
        {
            status = gcvSTATUS_OK;

            if (!signal->manualReset)
            {
                /* Auto reset. */
                signal->done = 0;
            }
        }
        else
        {
            status = (ret == -ERESTARTSYS) ? gcvSTATUS_INTERRUPTED
                   : gcvSTATUS_TIMEOUT;
        }
    }

OnError:
    /* Return status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
_QuerySignal(
    IN gckOS Os,
    IN gctSIGNAL Signal
    )
{
    /*
     * This function is called by 'has_signaled' callback of sync_timeline.
     * By design, 'has_signaled' could be called in interrupt context, but
     * in current driver, it can be called only when 'gckOS_Signal' and
     * 'gckOS_CreateNativeFence'. Thus its safe to use normal version of
     * spinlock for 'Os->signalDB.lock' and 'signal->obj.wait.lock'.
     */
    gceSTATUS status;
    gcsSIGNAL_PTR signal = gcvNULL;

    status = _QueryIntegerId(&Os->signalDB,
                             (gctUINT32)(gctUINTPTR_T)Signal,
                             (gctPOINTER)&signal);

    if (gcmIS_SUCCESS(status))
    {
        spin_lock(&signal->lock);
        status = signal->done ? gcvSTATUS_TRUE : gcvSTATUS_FALSE;
        spin_unlock(&signal->lock);
    }

    return status;
}

/*******************************************************************************
**
**  gckOS_MapSignal
**
**  Map a signal in to the current process space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to tha gctSIGNAL to map.
**
**      gctHANDLE Process
**          Handle of process owning the signal.
**
**  OUTPUT:
**
**      gctSIGNAL * MappedSignal
**          Pointer to a variable receiving the mapped gctSIGNAL.
*/
gceSTATUS
gckOS_MapSignal(
    IN gckOS Os,
    IN gctSIGNAL Signal,
    IN gctHANDLE Process,
    OUT gctSIGNAL * MappedSignal
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsSIGNAL_PTR signal = gcvNULL;
    unsigned long flags = 0;
    gcmkHEADER_ARG("Os=%p Signal=%p Process=%p", Os, Signal, Process);

    gcmkVERIFY_ARGUMENT(Signal != gcvNULL);
    gcmkVERIFY_ARGUMENT(MappedSignal != gcvNULL);

    spin_lock_irqsave(&Os->signalLock, flags);

    gcmkONERROR(_QueryIntegerId(&Os->signalDB, (gctUINT32)(gctUINTPTR_T)Signal, (gctPOINTER)&signal));

    if (atomic_inc_return(&signal->ref) <= 1)
    {
        /* The previous value is 0, it has been deleted. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    *MappedSignal = (gctSIGNAL) Signal;

OnError:
    spin_unlock_irqrestore(&Os->signalLock, flags);

    gcmkFOOTER_ARG("*MappedSignal=%p", *MappedSignal);
    return status;
}

/*******************************************************************************
**
**  gckOS_UnmapSignal
**
**  Unmap a signal .
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctSIGNAL Signal
**          Pointer to that gctSIGNAL mapped.
*/
gceSTATUS
gckOS_UnmapSignal(
    IN gckOS Os,
    IN gctSIGNAL Signal
    )
{
    return gckOS_DestroySignal(Os, Signal);
}

/*******************************************************************************
**
**  gckOS_CreateUserSignal
**
**  Create a new signal to be used in the user space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctBOOL ManualReset
**          If set to gcvTRUE, gckOS_Signal with gcvFALSE must be called in
**          order to set the signal to nonsignaled state.
**          If set to gcvFALSE, the signal will automatically be set to
**          nonsignaled state by gckOS_WaitSignal function.
**
**  OUTPUT:
**
**      gctINT * SignalID
**          Pointer to a variable receiving the created signal's ID.
*/
gceSTATUS
gckOS_CreateUserSignal(
    IN gckOS Os,
    IN gctBOOL ManualReset,
    OUT gctINT * SignalID
    )
{
    gceSTATUS status;
    gctSIZE_T signal;

    /* Create a new signal. */
    gcmkONERROR(gckOS_CreateSignal(Os, ManualReset, (gctSIGNAL *) &signal));
    *SignalID = (gctINT) signal;

OnError:
    return status;
}

/*******************************************************************************
**
**  gckOS_DestroyUserSignal
**
**  Destroy a signal to be used in the user space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctINT SignalID
**          The signal's ID.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_DestroyUserSignal(
    IN gckOS Os,
    IN gctINT SignalID
    )
{
    return gckOS_DestroySignal(Os, (gctSIGNAL)(gctUINTPTR_T)SignalID);
}

/*******************************************************************************
**
**  gckOS_WaitUserSignal
**
**  Wait for a signal used in the user mode to become signaled.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctINT SignalID
**          Signal ID.
**
**      gctUINT32 Wait
**          Number of milliseconds to wait.
**          Pass the value of gcvINFINITE for an infinite wait.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_WaitUserSignal(
    IN gckOS Os,
    IN gctINT SignalID,
    IN gctUINT32 Wait
    )
{
    return gckOS_WaitSignal(Os, (gctSIGNAL)(gctUINTPTR_T)SignalID, gcvTRUE, Wait);
}

/*******************************************************************************
**
**  gckOS_SignalUserSignal
**
**  Set a state of the specified signal to be used in the user space.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctINT SignalID
**          SignalID.
**
**      gctBOOL State
**          If gcvTRUE, the signal will be set to signaled state.
**          If gcvFALSE, the signal will be set to nonsignaled state.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_SignalUserSignal(
    IN gckOS Os,
    IN gctINT SignalID,
    IN gctBOOL State
    )
{
    return gckOS_Signal(Os, (gctSIGNAL)(gctUINTPTR_T)SignalID, State);
}


/******************************************************************************\
******************************** Software Timer ********************************
\******************************************************************************/

void
_TimerFunction(
    struct work_struct * work
    )
{
    gcsOSTIMER_PTR timer = (gcsOSTIMER_PTR)work;

    gctTIMERFUNCTION function = timer->function;

    function(timer->data);
}

/*******************************************************************************
**
**  gckOS_CreateTimer
**
**  Create a software timer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctTIMERFUNCTION Function.
**          Pointer to a call back function which will be called when timer is
**          expired.
**
**      gctPOINTER Data.
**          Private data which will be passed to call back function.
**
**  OUTPUT:
**
**      gctPOINTER * Timer
**          Pointer to a variable receiving the created timer.
*/
gceSTATUS
gckOS_CreateTimer(
    IN gckOS Os,
    IN gctTIMERFUNCTION Function,
    IN gctPOINTER Data,
    OUT gctPOINTER * Timer
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsOSTIMER_PTR pointer;
    gcmkHEADER_ARG("Os=%p Function=0%p Data=%p", Os, Function, Data);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Timer != gcvNULL);

    gcmkONERROR(gckOS_Allocate(Os, sizeof(gcsOSTIMER), (gctPOINTER)&pointer));

    pointer->function = Function;
    pointer->data = Data;

    INIT_DELAYED_WORK(&pointer->work, _TimerFunction);

    *Timer = pointer;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckOS_DestroyTimer
**
**  Destory a software timer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Timer
**          Pointer to the timer to be destoryed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_DestroyTimer(
    IN gckOS Os,
    IN gctPOINTER Timer
    )
{
    gcsOSTIMER_PTR timer;

    gcmkHEADER_ARG("Os=%p Timer=%p", Os, Timer);

    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Timer != gcvNULL);

    timer = (gcsOSTIMER_PTR)Timer;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
    cancel_delayed_work_sync(&timer->work);
#else
    cancel_delayed_work(&timer->work);
    flush_workqueue(Os->workqueue);
#endif

    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, Timer));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_StartTimer
**
**  Schedule a software timer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Timer
**          Pointer to the timer to be scheduled.
**
**      gctUINT32 Delay
**          Delay in milliseconds.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_StartTimer(
    IN gckOS Os,
    IN gctPOINTER Timer,
    IN gctUINT32 Delay
    )
{
    gcsOSTIMER_PTR timer;

    gcmkHEADER_ARG("Os=%p Timer=%p Delay=%u", Os, Timer, Delay);

    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Timer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Delay != 0);

    timer = (gcsOSTIMER_PTR)Timer;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
    mod_delayed_work(Os->workqueue, &timer->work, msecs_to_jiffies(Delay));
#else
    if (unlikely(delayed_work_pending(&timer->work)))
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
        cancel_delayed_work_sync(&timer->work);
#else
        cancel_delayed_work(&timer->work);
        flush_workqueue(Os->workqueue);
#endif
    }

    queue_delayed_work(Os->workqueue, &timer->work, msecs_to_jiffies(Delay));
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_StopTimer
**
**  Cancel a unscheduled timer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to the gckOS object.
**
**      gctPOINTER Timer
**          Pointer to the timer to be cancel.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_StopTimer(
    IN gckOS Os,
    IN gctPOINTER Timer
    )
{
    gcsOSTIMER_PTR timer;
    gcmkHEADER_ARG("Os=%p Timer=%p", Os, Timer);

    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Timer != gcvNULL);

    timer = (gcsOSTIMER_PTR)Timer;

    cancel_delayed_work(&timer->work);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_GetProcessNameByPid(
    IN gctINT Pid,
    IN gctSIZE_T Length,
    OUT gctUINT8_PTR String
    )
{
    struct task_struct *task;
    gceSTATUS status = gcvSTATUS_OK;

    /* Get the task_struct of the task with pid. */
    rcu_read_lock();

    task = FIND_TASK_BY_PID(Pid);
    if (task)
    {
        /* Get name of process. */
        strncpy(String, task->comm, Length);
    }
    else
    {
        status = gcvSTATUS_NOT_FOUND;
    }

    rcu_read_unlock();

    return status;
}

gceSTATUS
gckOS_DumpCallStack(
    IN gckOS Os
    )
{
    gcmkHEADER_ARG("Os=%p", Os);

    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);

    dump_stack();

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckOS_DetectProcessByName
**
**      task->comm maybe part of process name, so this function
**      can only be used for debugging.
**
**  INPUT:
**
**      gctCONST_POINTER Name
**          Pointer to a string to hold name to be check. If the length
**          of name is longer than TASK_COMM_LEN (16), use part of name
**          to detect.
**
**  OUTPUT:
**
**      gcvSTATUS_TRUE if name of current process matches Name.
**
*/
gceSTATUS
gckOS_DetectProcessByName(
    IN gctCONST_POINTER Name
    )
{
    char comm[sizeof(current->comm)];

    memset(comm, 0, sizeof(comm));

    gcmkVERIFY_OK(
        gckOS_GetProcessNameByPid(_GetProcessID(), sizeof(current->comm), comm));

    return strstr(comm, Name) ? gcvSTATUS_TRUE
                              : gcvSTATUS_FALSE;
}

#if gcdLINUX_SYNC_FILE
#ifndef CONFIG_SYNC_FILE
gceSTATUS
gckOS_CreateSyncTimeline(
    IN gckOS Os,
    IN gceCORE Core,
    OUT gctHANDLE * Timeline
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    struct viv_sync_timeline * timeline;
    char name[32];

    snprintf(name, 32, "gccore-%u", (unsigned int) Core);

    /* Create viv sync timeline. */
    timeline = viv_sync_timeline_create(name, Os);

    if (timeline)
    {
        *Timeline = (gctHANDLE)timeline;
    }
    else
    {
        /* Out of memory. */
        status = gcvSTATUS_OUT_OF_MEMORY;
    }


    return status;
}

gceSTATUS
gckOS_DestroySyncTimeline(
    IN gckOS Os,
    IN gctHANDLE Timeline
    )
{
    struct viv_sync_timeline * timeline;
    gcmkASSERT(Timeline != gcvNULL);

    /* Destroy timeline. */
    timeline = (struct viv_sync_timeline *) Timeline;
    sync_timeline_destroy(&timeline->obj);

    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_CreateNativeFence(
    IN gckOS Os,
    IN gctHANDLE Timeline,
    IN gctSIGNAL Signal,
    OUT gctINT * FenceFD
    )
{
    int fd = -1;
    struct viv_sync_timeline *timeline;
    struct sync_pt * pt = gcvNULL;
    struct sync_fence * fence;
    char name[32];
    gcsSIGNAL_PTR signal;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Timeline=%p Signal=%p", Os, Timeline, Signal);

    gcmkONERROR(
        _QueryIntegerId(&Os->signalDB,
                        (gctUINT32)(gctUINTPTR_T)Signal,
                        (gctPOINTER)&signal));

    /* Cast timeline. */
    timeline = (struct viv_sync_timeline *) Timeline;

    fd = get_unused_fd_flags(O_CLOEXEC);

    if (fd < 0)
    {
        /* Out of resources. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    /* Create viv_sync_pt. */
    pt = viv_sync_pt_create(timeline, Signal);

    if (pt == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Reference sync_timeline. */
    signal->timeline = &timeline->obj;

    /* Build fence name. */
    snprintf(name, 32, "%.16s-signal_%lu",
             current->comm,
             (unsigned long)Signal);

    /* Create sync_fence. */
    fence = sync_fence_create(name, pt);

    if (fence == NULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Install fence to fd. */
    sync_fence_install(fence, fd);

    *FenceFD = fd;
    return gcvSTATUS_OK;

OnError:
    if (gcmIS_ERROR(status))
    {
        /* Error roll back. */
        if (pt)
        {
            sync_pt_free(pt);
        }

        if (fd > 0)
        {
            put_unused_fd(fd);
        }
    }

    gcmkFOOTER_ARG("*FenceFD=%d", fd);
    return status;
}

static void
_NativeFenceSignaled(
    struct sync_fence *fence,
    struct sync_fence_waiter *waiter
    )
{
    kfree(waiter);
    sync_fence_put(fence);
}

gceSTATUS
gckOS_WaitNativeFence(
    IN gckOS Os,
    IN gctHANDLE Timeline,
    IN gctINT FenceFD,
    IN gctUINT32 Timeout
    )
{
    struct sync_timeline * timeline;
    struct sync_fence * fence;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Os=%p Timeline=%p FenceFD=%d Timeout=%u",
                   Os, Timeline, FenceFD, Timeout);

    /* Get shortcut. */
    timeline = (struct sync_timeline *) Timeline;

    /* Get sync fence. */
    fence = sync_fence_fdget(FenceFD);

    if (!fence)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (sync_fence_wait(fence, 0) == 0)
    {
        /* Already signaled. */
        sync_fence_put(fence);

        goto OnError;
    }
    else
    {
        gctBOOL wait = gcvFALSE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,17,0)
        int i;

        for (i = 0; i < fence->num_fences; i++)
        {
            struct fence *f = fence->cbs[i].sync_pt;
            struct sync_pt *pt = container_of(f, struct sync_pt, base);

            /* Do not need to wait on same timeline. */
            if ((sync_pt_parent(pt) != timeline) && !fence_is_signaled(f))
            {
                wait = gcvTRUE;
                break;
            }
        }
#else
        {
            struct list_head *pos;
            list_for_each(pos, &fence->pt_list_head)
            {
                struct sync_pt * pt =
                container_of(pos, struct sync_pt, pt_list);

                /* Do not need to wait on same timeline. */
                if (pt->parent != timeline)
                {
                    wait = gcvTRUE;
                    break;
                }
            }
        }
#endif

        if (wait)
        {
            int err;
            long timeout = (Timeout == gcvINFINITE) ? - 1 : (long) Timeout;
            err = sync_fence_wait(fence, timeout);

            /* Put the fence. */
            sync_fence_put(fence);

            switch (err)
            {
            case 0:
                break;
            case -ETIME:
                status = gcvSTATUS_TIMEOUT;
                break;
            default:
                gcmkONERROR(gcvSTATUS_GENERIC_IO);
                break;
            }
        }
        else
        {
            int err;
            struct sync_fence_waiter *waiter;
            waiter = (struct sync_fence_waiter *)kmalloc(
                    sizeof (struct sync_fence_waiter), gcdNOWARN | GFP_KERNEL);

            if (!waiter)
            {
                sync_fence_put(fence);
                gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
            }

            /* Schedule a waiter callback. */
            sync_fence_waiter_init(waiter, _NativeFenceSignaled);
            err = sync_fence_wait_async(fence, waiter);

            switch (err)
            {
            case 0:
                /* Put fence in callback function. */
                break;
            case 1:
                /* already signaled. */
                sync_fence_put(fence);
                break;
            default:
                sync_fence_put(fence);
                gcmkONERROR(gcvSTATUS_GENERIC_IO);
                break;
            }
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

#  else /* !CONFIG_SYNC_FILE */

gceSTATUS
gckOS_CreateSyncTimeline(
    IN gckOS Os,
    IN gceCORE Core,
    OUT gctHANDLE * Timeline
    )
{
    struct viv_sync_timeline *timeline;

    char name[32];

    snprintf(name, 32, "gccore-%u", (unsigned int) Core);
    timeline = viv_sync_timeline_create(name, Os);

    if (timeline == gcvNULL)
    {
        /* Out of memory. */
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    *Timeline = (gctHANDLE) timeline;
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_DestroySyncTimeline(
    IN gckOS Os,
    IN gctHANDLE Timeline
    )
{
    struct viv_sync_timeline * timeline;

    /* Destroy timeline. */
    timeline = (struct viv_sync_timeline *) Timeline;
    viv_sync_timeline_destroy(timeline);

    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_CreateNativeFence(
    IN gckOS Os,
    IN gctHANDLE Timeline,
    IN gctSIGNAL Signal,
    OUT gctINT * FenceFD
    )
{
    struct dma_fence *fence = NULL;
    struct sync_file *sync = NULL;
    int fd = -1;
    struct viv_sync_timeline *timeline;
    gcsSIGNAL_PTR signal = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    /* Create fence. */
    timeline = (struct viv_sync_timeline *) Timeline;

    gcmkONERROR(
        _QueryIntegerId(&Os->signalDB,
                        (gctUINT32)(gctUINTPTR_T)Signal,
                        (gctPOINTER)&signal));

    fence = viv_fence_create(timeline, signal);

    if (!fence)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Create sync_file. */
    sync = sync_file_create(fence);

    if (!sync)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Get a unused fd. */
    fd = get_unused_fd_flags(O_CLOEXEC);

    if (fd < 0)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    fd_install(fd, sync->file);

    *FenceFD = fd;
    return gcvSTATUS_OK;

OnError:
    if (sync)
    {
        fput(sync->file);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,68)
    if (fence)
    {
        dma_fence_put(fence);
    }
#endif

    if (fd > 0)
    {
        put_unused_fd(fd);
    }

    *FenceFD = -1;
    return status;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
/**
 * sync_file_fdget() - get a sync_file from an fd
 * @fd:     fd referencing a fence
 *
 * Ensures @fd references a valid sync_file, increments the refcount of the
 * backing file. Returns the sync_file or NULL in case of error.
 */
static struct sync_file *sync_file_fdget(int fd)
{
    struct file *file = fget(fd);

    if (!file)
        return NULL;

    return file->private_data;
}

gceSTATUS
gckOS_WaitNativeFence(
    IN gckOS Os,
    IN gctHANDLE Timeline,
    IN gctINT FenceFD,
    IN gctUINT32 Timeout
    )
{
    struct viv_sync_timeline *timeline;
    gceSTATUS status = gcvSTATUS_OK;
    unsigned int i;
    unsigned long timeout;
    unsigned int numFences;
    struct sync_file *sync_file;

    timeline = (struct viv_sync_timeline *) Timeline;

    sync_file = sync_file_fdget(FenceFD);

    if (!sync_file)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    numFences = sync_file->num_fences;

    timeout = msecs_to_jiffies(Timeout);

    for (i = 0; i < numFences; i++)
    {
        struct fence *f = sync_file->cbs[i].fence;
        fence_get(f);

        if (f->context != timeline->context &&
            !fence_is_signaled(f))
        {
            signed long ret;
            ret = fence_wait_timeout(f, 1, timeout);

            if (ret == -ERESTARTSYS)
            {
                fence_put(f);
                gcmkONERROR(gcvSTATUS_INTERRUPTED);
            }
            else if (ret <= 0)
            {
                fence_put(f);
                gcmkONERROR(gcvSTATUS_TIMEOUT);
            }
            else
            {
                /* wait success. */
                timeout -= ret;
            }
        }

        fence_put(f);
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

#    else

gceSTATUS
gckOS_WaitNativeFence(
    IN gckOS Os,
    IN gctHANDLE Timeline,
    IN gctINT FenceFD,
    IN gctUINT32 Timeout
    )
{
    struct viv_sync_timeline *timeline;
    gceSTATUS status = gcvSTATUS_OK;
    unsigned int i;
    unsigned long timeout;
    unsigned int numFences;
    struct dma_fence *fence;
    struct dma_fence **fences;

    timeline = (struct viv_sync_timeline *) Timeline;

    fence = sync_file_get_fence(FenceFD);

    if (!fence)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    if (dma_fence_is_array(fence))
    {
        struct dma_fence_array *array = to_dma_fence_array(fence);
        fences = array->fences;
        numFences = array->num_fences;
    }
    else
    {
        fences = &fence;
        numFences = 1;
    }

    timeout = msecs_to_jiffies(Timeout);

    for (i = 0; i < numFences; i++)
    {
        struct dma_fence *f = fences[i];

        if(!dma_fence_is_signaled(fence))
        {
            signed long ret;
            ret = dma_fence_wait_timeout(f, 1, timeout);

            if (ret == -ERESTARTSYS)
            {
                dma_fence_put(fence);
                gcmkONERROR(gcvSTATUS_INTERRUPTED);
            }
            else if (ret <= 0)
            {
                dma_fence_put(fence);
                gcmkONERROR(gcvSTATUS_TIMEOUT);
            }
            else
            {
                /* wait success. */
                timeout -= ret;
            }
        }
    }

    dma_fence_put(fence);

    return gcvSTATUS_OK;

OnError:
    return status;
}

#    endif
#  endif
#endif


gceSTATUS
gckOS_CPUPhysicalToGPUPhysical(
    IN gckOS Os,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T * GPUPhysical
    )
{
    gcsPLATFORM * platform;
    gcmkHEADER_ARG("CPUPhysical=%p", CPUPhysical);

    platform = Os->device->platform;

    if (platform && platform->ops->getGPUPhysical)
    {
        gcmkVERIFY_OK(
            platform->ops->getGPUPhysical(platform, CPUPhysical, GPUPhysical));
    }
    else
    {
        *GPUPhysical = CPUPhysical;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckOS_GPUPhysicalToCPUPhysical(
    IN gckOS Os,
    IN gctUINT32 GPUPhysical,
    IN gctPHYS_ADDR_T * CPUPhysical
    )
{
    gcsPLATFORM * platform;
    gcmkHEADER_ARG("Os=%p GPUPhysical=0x%x", Os, GPUPhysical);

    platform = Os->device->platform;

    if (platform && platform->ops->getCPUPhysical)
    {
        gcmkVERIFY_OK(
            platform->ops->getCPUPhysical(platform, GPUPhysical, CPUPhysical));
    }
    else
    {
        *CPUPhysical = GPUPhysical;
    }

    gcmkFOOTER_ARG("CPUPhysical=0x%llx", gcmOPT_VALUE(CPUPhysical));
    return gcvSTATUS_OK;
}

static int fd_release(struct inode *inode, struct file *file)
{
    gcsFDPRIVATE_PTR private = (gcsFDPRIVATE_PTR)file->private_data;

    return (private && private->release) ? private->release(private) : 0;
}

static const struct file_operations fd_fops =
{
    .release = fd_release,
};

gceSTATUS
gckOS_GetFd(
    IN gctSTRING Name,
    IN gcsFDPRIVATE_PTR Private,
    OUT gctINT * Fd
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
    *Fd = anon_inode_getfd(Name, &fd_fops, Private, O_RDWR);

    if (*Fd < 0)
    {
        return gcvSTATUS_OUT_OF_RESOURCES;
    }

    return gcvSTATUS_OK;
#else
    return gcvSTATUS_NOT_SUPPORTED;
#endif
}

gceSTATUS
gckOS_QueryOption(
    IN gckOS Os,
    IN gctCONST_STRING Option,
    OUT gctUINT64 * Value
    )
{
    gckGALDEVICE device = Os->device;
    gceSTATUS status = gcvSTATUS_OK;

    if (!strcmp(Option, "physBase"))
    {
        *Value = (gctUINT64)device->physBase;
    }
    else if (!strcmp(Option, "physSize"))
    {
        *Value = (gctUINT64)device->physSize;
    }
    else if (!strcmp(Option, "mmu"))
    {
        *Value = (gctUINT64)device->args.enableMmu;
    }
    else if (!strcmp(Option, "contiguousSize"))
    {
        *Value = (gctUINT64)device->contiguousSize;
    }
    else if (!strcmp(Option, "contiguousBase"))
    {
        *Value = (gctUINT64)device->contiguousBase;
    }
    else if (!strcmp(Option, "externalSize"))
    {
        *Value = (gctUINT64)device->externalSize;
        return gcvSTATUS_OK;
    }
    else if (!strcmp(Option, "exclusiveBase"))
    {
        *Value = (gctUINT64)device->exclusiveBase;
        return gcvSTATUS_OK;
    }
    else if (!strcmp(Option, "exclusiveSize"))
    {
        *Value = (gctUINT64)device->exclusiveSize;
        return gcvSTATUS_OK;
    }
    else if (!strcmp(Option, "externalBase"))
    {
        *Value = (gctUINT64)device->externalBase;
        return gcvSTATUS_OK;
    }
    else if (!strcmp(Option, "recovery"))
    {
        *Value = (gctUINT64)device->args.recovery;
    }
    else if (!strcmp(Option, "stuckDump"))
    {
        *Value = (gctUINT64)device->args.stuckDump;
    }
    else if (!strcmp(Option, "powerManagement"))
    {
        *Value = (gctUINT64)device->args.powerManagement;
    }
    else if (!strcmp(Option, "TA"))
    {
        *Value = 0;
    }
    else if (!strcmp(Option, "userClusterMasks"))
    {
        if (gcmSIZEOF(device->args.userClusterMasks) >= gcmSIZEOF(gctUINT32) * gcdMAX_MAJOR_CORE_COUNT)
            memcpy(Value, device->args.userClusterMasks, gcmSIZEOF(gctUINT32) * gcdMAX_MAJOR_CORE_COUNT);
        else
            return gcvSTATUS_NOT_SUPPORTED;
    }
    else if (!strcmp(Option, "smallBatch"))
    {
        *Value = device->args.smallBatch;
    }
    else if (!strcmp(Option, "sRAMBases"))
    {
        if (gcmSIZEOF(device->args.sRAMBases) >= gcmSIZEOF(gctUINT64) * gcvSRAM_INTER_COUNT * gcvCORE_COUNT)
            memcpy(Value, device->args.sRAMBases, gcmSIZEOF(gctUINT64) * gcvSRAM_INTER_COUNT * gcvCORE_COUNT);
        else
            return gcvSTATUS_NOT_SUPPORTED;
    }
    else if (!strcmp(Option, "sRAMSizes"))
    {
        if (gcmSIZEOF(device->args.sRAMSizes) >= gcmSIZEOF(gctUINT32) * gcvSRAM_INTER_COUNT * gcvCORE_COUNT)
            memcpy(Value, device->args.sRAMSizes, gcmSIZEOF(gctUINT32) * gcvSRAM_INTER_COUNT * gcvCORE_COUNT);
        else
            return gcvSTATUS_NOT_SUPPORTED;
    }
    else if (!strcmp(Option, "extSRAMBases"))
    {
        if (gcmSIZEOF(device->args.extSRAMBases) >= gcmSIZEOF(gctUINT64) * gcvSRAM_EXT_COUNT)
            memcpy(Value, device->args.extSRAMBases, gcmSIZEOF(gctUINT64) * gcvSRAM_EXT_COUNT);
        else
            return gcvSTATUS_NOT_SUPPORTED;
    }
    else if (!strcmp(Option, "extSRAMSizes"))
    {
        if (gcmSIZEOF(device->args.extSRAMSizes) >= gcmSIZEOF(gctUINT32) * gcvSRAM_EXT_COUNT)
            memcpy(Value, device->args.extSRAMSizes, gcmSIZEOF(gctUINT32) * gcvSRAM_EXT_COUNT);
        else
            return gcvSTATUS_NOT_SUPPORTED;
    }
    else if (!strcmp(Option, "sRAMRequested"))
    {
        *Value = (gctUINT64)device->args.sRAMRequested;
    }
    else if (!strcmp(Option, "sRAMLoopMode"))
    {
        *Value = (gctUINT64)device->args.sRAMLoopMode;
    }
    else if (!strcmp(Option, "platformFlagBits"))
    {
        *Value = (gctUINT64)device->platform->flagBits;
    }
    else if (!strcmp(Option, "mmuPageTablePool"))
    {
        *Value = (gctUINT64)device->args.mmuPageTablePool;
    }
    else if (!strcmp(Option, "mmuDynamicMap"))
    {
        *Value = (gctUINT64)device->args.mmuDynamicMap;
    }
    else if (!strcmp(Option, "allMapInOne"))
    {
        *Value = (gctUINT64)device->args.allMapInOne;
    }
    else if (!strcmp(Option, "isrPoll"))
    {
        *Value = (gctUINT64)device->args.isrPoll;
    }
    else if (!strcmp(Option, "registerAPB"))
    {
        *Value = (gctUINT64)device->args.registerAPB;
    }
    else if (!strcmp(Option, "enableNN"))
    {
        *Value = (gctUINT64)device->args.enableNN;
    }
    else if (!strcmp(Option, "softReset"))
    {
        *Value = (gctUINT64)device->args.softReset;
    }
    else
    {
        status = gcvSTATUS_NOT_SUPPORTED;
    }

    return status;
}

gceSTATUS
gckOS_MemoryGetSGT(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *SGT
    )
{
    PLINUX_MDL mdl;
    gckALLOCATOR allocator;
    gceSTATUS status = gcvSTATUS_OK;

    if (!Physical)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    mdl = (PLINUX_MDL)Physical;
    allocator = mdl->allocator;

    if (!allocator->ops->GetSGT)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    if (Bytes > 0)
    {
        gcmkONERROR(gcmALLOCATOR_GetSGT(allocator, mdl, Offset, Bytes, SGT));
    }

OnError:
    return status;
}

gceSTATUS
gckOS_MemoryMmap(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T skipPages,
    IN gctSIZE_T numPages,
    INOUT gctPOINTER Vma
    )
{
    PLINUX_MDL mdl;
    PLINUX_MDL_MAP mdlMap;
    gckALLOCATOR allocator;
    gceSTATUS status = gcvSTATUS_OK;
    gctBOOL cacheable = gcvFALSE;

    if (!Physical)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    mdl = (PLINUX_MDL)Physical;
    allocator = mdl->allocator;

    if (!allocator->ops->Mmap)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    mutex_lock(&mdl->mapsMutex);

    mdlMap = FindMdlMap(mdl, _GetProcessID());
    if (mdlMap)
    {
        cacheable = mdlMap->cacheable;
    }

    mutex_unlock(&mdl->mapsMutex);

    gcmkONERROR(gcmALLOCATOR_Mmap(allocator, mdl, cacheable, skipPages, numPages, Vma));

OnError:
    return status;
}

/*******************************************************************************
**
**  gckOS_WrapMemory
**
**  Import a number of pages allocated by other allocator.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctUINT32 Flag
**          Memory type.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that hold the number of bytes allocated.
**
**      gctPHYS_ADDR * Physical
**          Pointer to a variable that will hold the physical address of the
**          allocation.
*/
gceSTATUS
gckOS_WrapMemory(
    IN gckOS Os,
    IN gcsUSER_MEMORY_DESC_PTR Desc,
    OUT gctSIZE_T *Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctBOOL *Contiguous,
    OUT gctSIZE_T * PageCountCpu
    )
{
    PLINUX_MDL mdl = gcvNULL;
    gceSTATUS status = gcvSTATUS_OUT_OF_MEMORY;
    gckALLOCATOR allocator;
    gcsATTACH_DESC desc;
    gctSIZE_T bytes = 0;

    gcmkHEADER_ARG("Os=%p ", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Desc != gcvNULL);

    mdl = _CreateMdl(Os);
    if (mdl == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    mdl->wrapFromPhysical = gcvFALSE;
    mdl->wrapFromLogical  = gcvFALSE;

    if (Desc->flag & gcvALLOC_FLAG_DMABUF)
    {
        if (IS_ERR(gcmUINT64_TO_PTR(Desc->dmabuf)))
        {
            /* Won't enter here currently, the caller confirms the dmabuf is valid. */

            gcmkPRINT("Wrap memory: invalid dmabuf.\n");
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        desc.dmaBuf.dmabuf = gcmUINT64_TO_PTR(Desc->dmabuf);

#if defined(CONFIG_DMA_SHARED_BUFFER)
        {
            struct dma_buf *dmabuf = (struct dma_buf*)desc.dmaBuf.dmabuf;
            bytes = dmabuf->size;
        }
#endif
    }
    else if (Desc->flag & gcvALLOC_FLAG_USERMEMORY)
    {
        desc.userMem.memory   = gcmUINT64_TO_PTR(Desc->logical);
        desc.userMem.physical = Desc->physical;
        desc.userMem.size     = Desc->size;
        bytes                 = Desc->size;

        if (Desc->physical == gcvINVALID_PHYSICAL_ADDRESS)
        {
            mdl->wrapFromLogical = gcvTRUE;
        }
        else
        {
            mdl->wrapFromPhysical = gcvTRUE;
        }
    }
    else if (Desc->flag & gcvALLOC_FLAG_EXTERNAL_MEMORY)
    {
        desc.externalMem.info = Desc->externalMemoryInfo;
    }
    else
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    /* Walk all allocators. */
    list_for_each_entry(allocator, &Os->allocatorList, link)
    {
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS,
                       "%s(%d) Flag = %x allocator->capability = %x",
                        __FUNCTION__, __LINE__, Desc->flag, allocator->capability);

        if ((Desc->flag & allocator->capability) != Desc->flag)
        {
            status = gcvSTATUS_NOT_SUPPORTED;
            continue;
        }

        if (Desc->flag == gcvALLOC_FLAG_EXTERNAL_MEMORY)
        {
            /* Use name to match suitable allocator for external memory. */
            if (!strncmp(Desc->externalMemoryInfo.allocatorName,
                         allocator->name, gcdEXTERNAL_MEMORY_NAME_MAX))
            {
                status = gcvSTATUS_NOT_SUPPORTED;
                continue;
            }
        }

        status = gcmALLOCATOR_Attach(allocator, &desc, mdl);

        if (gcmIS_SUCCESS(status))
        {
            mdl->allocator = allocator;
            break;
        }
    }

    /* Check status. */
    gcmkONERROR(status);

    mdl->dmaHandle  = 0;
    mdl->addr       = 0;

    mdl->bytes = bytes ? bytes : mdl->numPages * PAGE_SIZE;
    *Bytes = mdl->bytes;

    /* Return physical address. */
    *Physical = (gctPHYS_ADDR) mdl;

    *Contiguous = mdl->contiguous;

    if (PageCountCpu)
    {
        *PageCountCpu = mdl->numPages;
    }

    /*
     * Add this to a global list.
     * Will be used by get physical address
     * and mapuser pointer functions.
     */
    mutex_lock(&Os->mdlMutex);
    list_add_tail(&mdl->link, &Os->mdlHead);
    mutex_unlock(&Os->mdlMutex);

    /* Success. */
    status = gcvSTATUS_OK;

OnError:
    if (gcmIS_ERROR(status) && mdl)
    {
        /* Free the memory. */
        _DestroyMdl(mdl);
    }

    /* Return the status. */
    gcmkFOOTER_ARG("*Physical=%p", *Physical);
    return status;
}

gceSTATUS
gckOS_GetPolicyID(
    IN gckOS Os,
    IN gceVIDMEM_TYPE Type,
    OUT gctUINT32_PTR PolicyID,
    OUT gctUINT32_PTR AXIConfig
    )
{
    gcsPLATFORM * platform = Os->device->platform;
    gceSTATUS status = (platform && platform->ops->getPolicyID)
                     ? platform->ops->getPolicyID(platform, Type, PolicyID, AXIConfig)
                     : gcvSTATUS_NOT_SUPPORTED;

    return status;
}

#if gcdENABLE_MP_SWITCH
gceSTATUS
gckOS_SwitchCoreCount(
    IN gckOS Os,
    OUT gctUINT32 *Count
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsPLATFORM * platform = Os->device->platform;

    gcmkHEADER_ARG("Os=%p", Os);

    status = (platform && platform->ops->switchCoreCount)
           ? platform->ops->switchCoreCount(platform, Count)
           : gcvSTATUS_OK;

    gcmkFOOTER_ARG("*Count=%d", *Count);
    return status;
}
#endif

