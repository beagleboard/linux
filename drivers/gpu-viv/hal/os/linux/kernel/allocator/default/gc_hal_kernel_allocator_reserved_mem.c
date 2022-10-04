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
#include "gc_hal_kernel_allocator.h"
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ioport.h>

#define _GC_OBJ_ZONE    gcvZONE_OS

/*
 * reserved_mem is for contiguous pool, internal pool and external pool, etc.
 */

/* mdl private. */
struct reserved_mem
{
    unsigned long start;
    unsigned long size;
    unsigned int offset_in_page;
    char name[32];
    int  release;

    /* Link together. */
    struct list_head link;
    /* the mdl is root or not */
    gctBOOL root;
};

/* allocator info. */
struct reserved_mem_alloc
{
    /* Record allocated reserved memory regions. */
    struct list_head region;
    struct mutex lock;
};

static int reserved_mem_show(struct seq_file* m, void* data)
{
    struct list_head *pos;
    gcsINFO_NODE *node = m->private;
    gckALLOCATOR Allocator = node->device;
    struct reserved_mem_alloc *alloc = Allocator->privateData;

    list_for_each(pos, &alloc->region)
    {
        struct reserved_mem * res= list_entry(pos, struct reserved_mem, link);

        seq_printf(m, "0x%08lx-0x%08lx : %s\n",
            res->start, res->start + res->size -1, res->name);
    }

    return 0;
}

static gcsINFO info_list[] =
{
    {"reserved-mem", reserved_mem_show},
};

static void
reserved_mem_debugfs_init(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "reserved-mem"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        info_list,
        gcmCOUNTOF(info_list),
        Allocator
        ));
}

static void
reserved_mem_debugfs_cleanup(
    IN gckALLOCATOR Allocator
    )
{
    gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(
        &Allocator->debugfsDir,
        info_list,
        gcmCOUNTOF(info_list)
        ));

    gckDEBUGFS_DIR_Deinit(&Allocator->debugfsDir);
}

static gceSTATUS
reserved_mem_alloc(
    IN gckALLOCATOR Allocator,
    INOUT PLINUX_MDL Mdl,
    IN gctSIZE_T NumPages,
    IN gctUINT32 Flags
    )
{
    return gcvSTATUS_OUT_OF_MEMORY;
}

static gceSTATUS
reserved_mem_attach(
    IN gckALLOCATOR Allocator,
    IN gcsATTACH_DESC_PTR Desc,
    IN PLINUX_MDL Mdl
    )
{
    struct reserved_mem_alloc *alloc = Allocator->privateData;
    struct reserved_mem *res;
    struct resource *region = NULL;

    if (Desc == gcvNULL)
    {
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    res = kzalloc(sizeof(struct reserved_mem), GFP_KERNEL | gcdNOWARN);

    if (!res)
        return gcvSTATUS_OUT_OF_MEMORY;

    res->start = Desc->reservedMem.start;
    res->size  = Desc->reservedMem.size;
    res->offset_in_page = Desc->reservedMem.start & (PAGE_SIZE - 1);
    strncpy(res->name, Desc->reservedMem.name, sizeof(res->name)-1);
    res->release = 0;
    res->root = Desc->reservedMem.root;

    /* the region requierd is handed by root MDL */
    if (Desc->reservedMem.root)
    {
        if (!Desc->reservedMem.requested)
        {
            region = request_mem_region(res->start, res->size, res->name);

            if (!region)
            {
                printk("request mem %s(0x%lx - 0x%lx) failed\n",
                    res->name, res->start, res->start + res->size - 1);

                kfree(res);
                return gcvSTATUS_OUT_OF_RESOURCES;
            }

            res->release = 1;
        }

        mutex_lock(&alloc->lock);
        list_add(&res->link, &alloc->region);
        mutex_unlock(&alloc->lock);
    }

    Mdl->priv = res;

    if ((res->start + res->size) < 0xFFFFFFFF)
    {
        Allocator->capability |= gcvALLOC_FLAG_4GB_ADDR;
    }

    return gcvSTATUS_OK;
}

static void
reserved_mem_detach(
    IN gckALLOCATOR Allocator,
    IN OUT PLINUX_MDL Mdl
    )
{
    struct reserved_mem_alloc *alloc = Allocator->privateData;
    struct reserved_mem *res = Mdl->priv;

    if (res->root)
    {
        /* unlink from region list. */
        mutex_lock(&alloc->lock);
        list_del_init(&res->link);
        mutex_unlock(&alloc->lock);

        if (res->release)
        {
            release_mem_region(res->start, res->size);
        }
    }

    kfree(res);
}

static gceSTATUS
reserved_mem_mmap(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctBOOL Cacheable,
    IN gctSIZE_T skipPages,
    IN gctSIZE_T numPages,
    IN struct vm_area_struct *vma
    )
{
    struct reserved_mem *res = (struct reserved_mem*)Mdl->priv;
    unsigned long pfn;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p vma=%p", Allocator, Mdl, vma);

    if (Mdl->cpuAccessible)
    {
        gcmkASSERT(skipPages + numPages <= Mdl->numPages);

        pfn = (res->start >> PAGE_SHIFT) + skipPages;

        /* Make this mapping non-cached. */
        vma->vm_flags |= gcdVM_FLAGS;

#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
        vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
#else
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif

        if (remap_pfn_range(vma, vma->vm_start,
                pfn, numPages << PAGE_SHIFT, vma->vm_page_prot) < 0)
        {
            gcmkTRACE(
                gcvLEVEL_ERROR,
                "%s(%d): remap_pfn_range error.",
                __FUNCTION__, __LINE__
                );

            status = gcvSTATUS_OUT_OF_MEMORY;
        }
    }
    else
    {
        status = gcvSTATUS_NOT_SUPPORTED;
    }


    gcmkFOOTER();
    return status;
}

static void
reserved_mem_unmap_user(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctUINT32 Size
    )
{
    struct reserved_mem *res = (struct reserved_mem*)Mdl->priv;

    if (unlikely(!current->mm))
        return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
    if (vm_munmap((unsigned long)MdlMap->vmaAddr - res->offset_in_page, res->size) < 0)
    {
        printk("%s: vm_munmap failed\n", __func__);
    }
#else
    down_write(&current_mm_mmap_sem);
    if (do_munmap(current->mm, (unsigned long)MdlMap->vmaAddr - res->offset_in_page, res->size) < 0)
    {
        printk("%s: do_munmap failed\n", __func__);
    }
    up_write(&current_mm_mmap_sem);
#endif

    MdlMap->vma = NULL;
    MdlMap->vmaAddr = NULL;
}

static gceSTATUS
reserved_mem_map_user(
    gckALLOCATOR Allocator,
    PLINUX_MDL Mdl,
    PLINUX_MDL_MAP MdlMap,
    gctBOOL Cacheable
    )
{
    struct reserved_mem *res = (struct reserved_mem*)Mdl->priv;
    gctPOINTER userLogical = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p Cacheable=%d", Allocator, Mdl, Cacheable);

    if (!Mdl->cpuAccessible)
    {
        status = gcvSTATUS_NOT_SUPPORTED;
        goto Out;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#if gcdANON_FILE_FOR_ALLOCATOR
    userLogical = (gctPOINTER)vm_mmap(Allocator->anon_file, 0L, res->size,
#else
    userLogical = (gctPOINTER)vm_mmap(NULL, 0L, res->size,
#endif
                PROT_READ | PROT_WRITE, MAP_SHARED | MAP_NORESERVE, 0);
#else
    down_write(&current_mm_mmap_sem);
    userLogical = (gctPOINTER)do_mmap_pgoff(NULL, 0L, res->size,
                PROT_READ | PROT_WRITE, MAP_SHARED, 0);
    up_write(&current_mm_mmap_sem);
#endif

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): vmaAddr->%p for phys_addr->%p",
        __FUNCTION__, __LINE__, userLogical, Mdl
        );

    if (IS_ERR(userLogical))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): do_mmap_pgoff error",
            __FUNCTION__, __LINE__
            );

        userLogical = gcvNULL;

        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    down_write(&current_mm_mmap_sem);

    do
    {
        struct vm_area_struct *vma = find_vma(current->mm, (unsigned long)userLogical);
        if (vma == gcvNULL)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_INFO, gcvZONE_OS,
                "%s(%d): find_vma error",
                __FUNCTION__, __LINE__
                );

            gcmkERR_BREAK(gcvSTATUS_OUT_OF_RESOURCES);
        }

        gcmkERR_BREAK(reserved_mem_mmap(Allocator, Mdl, gcvFALSE, 0, Mdl->numPages, vma));

        MdlMap->vmaAddr = userLogical + res->offset_in_page;
        MdlMap->cacheable = gcvFALSE;
        MdlMap->vma = vma;
    }
    while (gcvFALSE);

    up_write(&current_mm_mmap_sem);

OnError:
    if (gcmIS_ERROR(status) && userLogical)
    {
        MdlMap->vmaAddr = userLogical + res->offset_in_page;
        reserved_mem_unmap_user(Allocator, Mdl, MdlMap, res->size);
    }
Out:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
reserved_mem_map_kernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical
    )
{
    struct reserved_mem *res = Mdl->priv;
    void *vaddr;

    if (!Mdl->cpuAccessible)
    {
        return gcvSTATUS_NOT_SUPPORTED;
    }

    if (Offset + Bytes > res->size)
    {
        return gcvSTATUS_INVALID_ARGUMENT;
    }

#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
    vaddr = ioremap_wc(res->start + Offset, Bytes);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    vaddr = ioremap(res->start + Offset, Bytes);
#else
    vaddr = ioremap_nocache(res->start + Offset, Bytes);
#endif
#endif

    if (!vaddr)
    {
        return gcvSTATUS_OUT_OF_MEMORY;
    }

    *Logical = vaddr;
    return gcvSTATUS_OK;;
}

static gceSTATUS
reserved_mem_unmap_kernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
    if (!Mdl->cpuAccessible)
    {
        return gcvSTATUS_NOT_SUPPORTED;
    }

    iounmap((void *)Logical);

    return gcvSTATUS_OK;
}

static gceSTATUS
reserved_mem_cache_op(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    /* Always WC or UC, safe to use mb. */
    switch (Operation)
    {
    case gcvCACHE_CLEAN:
    case gcvCACHE_FLUSH:
        _MemoryBarrier();
        break;
    case gcvCACHE_INVALIDATE:
        break;
    default:
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
reserved_mem_get_physical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    struct reserved_mem *res = Mdl->priv;
    *Physical = res->start + Offset;

    return gcvSTATUS_OK;
}

static gceSTATUS
reserved_mem_GetSGT(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *SGT
    )
{
    struct page * page = gcvNULL;
    struct sg_table *sgt = NULL;
    struct reserved_mem *res = Mdl->priv;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkASSERT(Offset + Bytes <= Mdl->numPages << PAGE_SHIFT);

    sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
    if (!sgt)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    page = pfn_to_page(res->start >> PAGE_SHIFT);

    if (sg_alloc_table(sgt, 1, GFP_KERNEL)){
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }
    sg_set_page(sgt->sgl, page, PAGE_ALIGN(Bytes), Offset);

    *SGT = (gctPOINTER)sgt;

OnError:
    if (gcmIS_ERROR(status) && sgt)
    {
        kfree(sgt);
    }

    return status;
}

static void
reserved_mem_dtor(
    gcsALLOCATOR *Allocator
    )
{
    reserved_mem_debugfs_cleanup(Allocator);

    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* GFP allocator operations. */
static gcsALLOCATOR_OPERATIONS reserved_mem_ops = {
    .Alloc              = reserved_mem_alloc,
    .Attach             = reserved_mem_attach,
    .Free               = reserved_mem_detach,
    .Mmap               = reserved_mem_mmap,
    .MapUser            = reserved_mem_map_user,
    .UnmapUser          = reserved_mem_unmap_user,
    .MapKernel          = reserved_mem_map_kernel,
    .UnmapKernel        = reserved_mem_unmap_kernel,
    .Cache              = reserved_mem_cache_op,
    .Physical           = reserved_mem_get_physical,
    .GetSGT             = reserved_mem_GetSGT,
};

/* GFP allocator entry. */
gceSTATUS
_ReservedMemoryAllocatorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator = gcvNULL;
    struct reserved_mem_alloc *alloc = NULL;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &reserved_mem_ops, &allocator));

    alloc = kzalloc(sizeof(*alloc), GFP_KERNEL | gcdNOWARN);

    if (!alloc)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    INIT_LIST_HEAD(&alloc->region);
    mutex_init(&alloc->lock);

    /* Register private data. */
    allocator->privateData = alloc;
    allocator->destructor = reserved_mem_dtor;

    reserved_mem_debugfs_init(allocator, Parent);

    allocator->capability = gcvALLOC_FLAG_LINUX_RESERVED_MEM
                          | gcvALLOC_FLAG_CONTIGUOUS
                          | gcvALLOC_FLAG_CPU_ACCESS
                          | gcvALLOC_FLAG_DMABUF_EXPORTABLE
                          | gcvALLOC_FLAG_NON_CPU_ACCESS;

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    if (allocator)
    {
        kfree(allocator);
    }
    return status;
}

