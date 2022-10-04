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
#include <linux/slab.h>
#include <linux/mutex.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>

#include <linux/dma-buf.h>
#include <linux/platform_device.h>

#define _GC_OBJ_ZONE gcvZONE_OS

/* Descriptor of a dma_buf imported. */
typedef struct _gcsDMABUF
{
    struct dma_buf            * dmabuf;
    struct dma_buf_attachment * attachment;
    struct sg_table           * sgt;
    unsigned long             * pagearray;

    int                         npages;
    int                         pid;
    struct list_head            list;
}
gcsDMABUF;

struct allocator_priv
{
    struct mutex lock;
    struct list_head buf_list;
};

/*
* Debugfs support.
*/
static int dma_buf_info_show(struct seq_file* m, void* data)
{
    int ret;
    gcsDMABUF *buf_desc;
    struct dma_buf_attachment *attach_obj;
    int count = 0;
    size_t size = 0;
    int npages = 0;
    const char *exp_name;

    gcsINFO_NODE *node = m->private;
    gckALLOCATOR allocator = node->device;
    struct allocator_priv *priv = allocator->privateData;

    ret = mutex_lock_interruptible(&priv->lock);

    if (ret)
        return ret;

    seq_puts(m, "Attached dma-buf objects:\n");
    seq_puts(m, "   pid     fd    pages     size   exporter attached-devices\n");

    list_for_each_entry(buf_desc, &priv->buf_list, list) {
        struct dma_buf *buf_obj = buf_desc->dmabuf;

        ret = mutex_lock_interruptible(&buf_obj->lock);

        if (ret) {
            seq_puts(m,
                 "ERROR locking buffer object: skipping\n");
            continue;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
        exp_name = buf_obj->exp_name;
#else
        exp_name = "unknown";
#endif

        seq_printf(m, "%6d %p %8d %8zu %10s",
                buf_desc->pid,
                buf_desc->dmabuf,
                buf_desc->npages,
                buf_obj->size,
                exp_name);

        list_for_each_entry(attach_obj, &buf_obj->attachments, node) {
            seq_printf(m, " %s", dev_name(attach_obj->dev));
        }
        seq_puts(m, "\n");

        count++;
        size += buf_obj->size;
        npages += buf_desc->npages;

        mutex_unlock(&buf_obj->lock);
    }

    seq_printf(m, "\nTotal %d objects, %d pages, %zu bytes\n", count, npages, size);

    mutex_unlock(&priv->lock);
    return 0;
}

static gcsINFO _InfoList[] =
{
    {"bufinfo", dma_buf_info_show},
};

static void
_DebugfsInit(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "dma_buf"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        _InfoList,
        gcmCOUNTOF(_InfoList),
        Allocator
        ));
}

static void
_DebugfsCleanup(
    IN gckALLOCATOR Allocator
    )
{
    gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(
        &Allocator->debugfsDir,
        _InfoList,
        gcmCOUNTOF(_InfoList)
        ));

    gckDEBUGFS_DIR_Deinit(&Allocator->debugfsDir);
}

static gceSTATUS
_DmabufAttach(
    IN gckALLOCATOR Allocator,
    IN gcsATTACH_DESC_PTR Desc,
    IN PLINUX_MDL Mdl
    )
{
    gceSTATUS status;

    gckOS os = Allocator->os;

    struct dma_buf *dmabuf = Desc->dmaBuf.dmabuf;
    struct sg_table *sgt = NULL;
    struct dma_buf_attachment *attachment = NULL;
    int npages = 0;
    unsigned long *pagearray = NULL;
    int i, j, k = 0;
    struct scatterlist *s;
    struct allocator_priv *priv = Allocator->privateData;
    gcsDMABUF *buf_desc = NULL;

    gcmkHEADER();

    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    if (!dmabuf)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    get_dma_buf(dmabuf);
    attachment = dma_buf_attach(dmabuf, &os->device->platform->device->dev);

    if (!attachment)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);

    if (!sgt)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    if (os->device->args.enableMmu == 0 &&
        os->iommu == gcvNULL &&
        sgt->nents != 1)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    /* Prepare page array. */
    /* Get number of pages. */
    for_each_sg(sgt->sgl, s, sgt->orig_nents, i)
    {
        npages += (sg_dma_len(s) + PAGE_SIZE - 1) / PAGE_SIZE;
    }

    /* Allocate page array. */
    gcmkONERROR(gckOS_Allocate(os, npages * gcmSIZEOF(*pagearray), (gctPOINTER *)&pagearray));

    /* Fill page array. */
    for_each_sg(sgt->sgl, s, sgt->orig_nents, i)
    {
        for (j = 0; j < (sg_dma_len(s) + PAGE_SIZE - 1) / PAGE_SIZE; j++)
        {
            pagearray[k++] = sg_dma_address(s) + j * PAGE_SIZE;
        }
    }

    /* Prepare descriptor. */
    gcmkONERROR(gckOS_Allocate(os, sizeof(gcsDMABUF), (gctPOINTER *)&buf_desc));

    buf_desc->dmabuf = dmabuf;
    buf_desc->pagearray = pagearray;
    buf_desc->attachment = attachment;
    buf_desc->sgt = sgt;

    /* Record in buffer list to support debugfs. */
    buf_desc->npages = npages;
    buf_desc->pid    = _GetProcessID();

    mutex_lock(&priv->lock);
    list_add(&buf_desc->list, &priv->buf_list);
    mutex_unlock(&priv->lock);

    /* Record page number. */
    Mdl->numPages = npages;

    Mdl->priv = buf_desc;

    Mdl->contiguous = (sgt->nents == 1) ? gcvTRUE : gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (pagearray)
    {
        gcmkOS_SAFE_FREE(os, pagearray);
    }

    if (sgt)
    {
        dma_buf_unmap_attachment(attachment, sgt, DMA_BIDIRECTIONAL);
    }

    gcmkFOOTER();
    return status;
}


static void
_DmabufFree(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl
    )
{
    gcsDMABUF *buf_desc = Mdl->priv;
    gckOS os = Allocator->os;
    struct allocator_priv *priv = Allocator->privateData;

    mutex_lock(&priv->lock);
    list_del(&buf_desc->list);
    mutex_unlock(&priv->lock);

    dma_buf_unmap_attachment(buf_desc->attachment, buf_desc->sgt, DMA_BIDIRECTIONAL);

    dma_buf_detach(buf_desc->dmabuf, buf_desc->attachment);

    dma_buf_put(buf_desc->dmabuf);

    gckOS_Free(os, buf_desc->pagearray);

    gckOS_Free(os, buf_desc);
}

static void
_DmabufUnmapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctUINT32 Size
    )
{
    gcsDMABUF *buf_desc = Mdl->priv;
    gctINT8_PTR userLogical = MdlMap->vmaAddr;

    if (unlikely(current->mm == gcvNULL))
    {
        /* Do nothing if process is exiting. */
        return;
    }

    userLogical -= buf_desc->sgt->sgl->offset;
    vm_munmap((unsigned long)userLogical, Mdl->numPages << PAGE_SHIFT);
}

static gceSTATUS
_DmabufMapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctBOOL Cacheable
    )
{
    gcsDMABUF *buf_desc = Mdl->priv;
    gctINT8_PTR userLogical = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;
    struct file *fd = buf_desc->dmabuf->file;
    unsigned long flag = 0;

    flag |= (fd->f_mode & FMODE_READ ? PROT_READ : 0);
    flag |= (fd->f_mode & FMODE_WRITE ? PROT_WRITE : 0);

    userLogical = (gctINT8_PTR)vm_mmap(fd,
                    0L,
                    Mdl->numPages << PAGE_SHIFT,
                    flag,
                    MAP_SHARED | MAP_NORESERVE,
                    0);

    if (IS_ERR(userLogical))
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }
    userLogical += buf_desc->sgt->sgl->offset;

    MdlMap->vmaAddr = (gctPOINTER)userLogical;
    MdlMap->cacheable = Cacheable;

OnError:
    if (gcmIS_ERROR(status) && MdlMap->vmaAddr)
    {
        _DmabufUnmapUser(Allocator, Mdl, MdlMap, Mdl->numPages << PAGE_SHIFT);
    }
    return status;
}

static gceSTATUS
_DmabufMapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical
    )
{
    /* Kernel doesn't acess video memory. */
    return gcvSTATUS_NOT_SUPPORTED;

}

static gceSTATUS
_DmabufUnmapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
    /* Kernel doesn't acess video memory. */
    return gcvSTATUS_NOT_SUPPORTED;
}

static gceSTATUS
_DmabufCache(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    gcsDMABUF *buf_desc = Mdl->priv;
    struct sg_table *sgt = buf_desc->sgt;
    enum dma_data_direction dir;

    switch (Operation)
    {
    case gcvCACHE_CLEAN:
        dir = DMA_TO_DEVICE;
        dma_sync_sg_for_device(galcore_device, sgt->sgl, sgt->nents, dir);
        break;
    case gcvCACHE_FLUSH:
        dir = DMA_TO_DEVICE;
        dma_sync_sg_for_device(galcore_device, sgt->sgl, sgt->nents, dir);
        dir = DMA_FROM_DEVICE;
        dma_sync_sg_for_cpu(galcore_device, sgt->sgl, sgt->nents, dir);
        break;
    case gcvCACHE_INVALIDATE:
        dir = DMA_FROM_DEVICE;
        dma_sync_sg_for_cpu(galcore_device, sgt->sgl, sgt->nents, dir);
        break;
    default:
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    return gcvSTATUS_OK;
}


static gceSTATUS
_DmabufPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    gcsDMABUF *buf_desc = Mdl->priv;
    gctUINT32 offsetInPage = Offset & ~PAGE_MASK;
    gctUINT32 index = Offset / PAGE_SIZE;

    *Physical = buf_desc->pagearray[index] + offsetInPage;


    return gcvSTATUS_OK;
}

/* Default allocator operations. */
static gcsALLOCATOR_OPERATIONS DmabufAllocatorOperations =
{
    .Attach             = _DmabufAttach,
    .Free               = _DmabufFree,
    .MapUser            = _DmabufMapUser,
    .UnmapUser          = _DmabufUnmapUser,
    .MapKernel          = _DmabufMapKernel,
    .UnmapKernel        = _DmabufUnmapKernel,
    .Cache              = _DmabufCache,
    .Physical           = _DmabufPhysical,
};

static void
_DmabufAllocatorDestructor(
    gcsALLOCATOR *Allocator
    )
{
    _DebugfsCleanup(Allocator);

    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* Default allocator entry. */
gceSTATUS
_DmabufAlloctorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator;
    struct allocator_priv *priv = NULL;

    priv = kmalloc(sizeof (struct allocator_priv), GFP_KERNEL | gcdNOWARN);

    if (!priv)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    mutex_init(&priv->lock);
    INIT_LIST_HEAD(&priv->buf_list);

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &DmabufAllocatorOperations, &allocator));

    allocator->capability = gcvALLOC_FLAG_DMABUF
                          | gcvALLOC_FLAG_DMABUF_EXPORTABLE
                          ;

    /* Register private data. */
    allocator->privateData = priv;
    allocator->destructor  = _DmabufAllocatorDestructor;

    _DebugfsInit(allocator, Parent);

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    if (priv)
    {
        kfree(priv);
    }

    return status;
}

