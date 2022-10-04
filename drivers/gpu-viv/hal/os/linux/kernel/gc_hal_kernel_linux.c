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

#define _GC_OBJ_ZONE    gcvZONE_KERNEL

/******************************************************************************\
******************************* gckKERNEL API Code ******************************
\******************************************************************************/

/*******************************************************************************
**
**  gckKERNEL_QueryVideoMemory
**
**  Query the amount of video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to an gcsHAL_INTERFACE structure that will be filled in with
**          the memory information.
*/
gceSTATUS
gckKERNEL_QueryVideoMemory(
    IN gckKERNEL Kernel,
    OUT gcsHAL_INTERFACE * Interface
    )
{
    gckGALDEVICE device;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface != NULL);

    /* Extract the pointer to the gckGALDEVICE class. */
    device = (gckGALDEVICE) Kernel->context;

    /* Get internal memory size and physical address. */
    Interface->u.QueryVideoMemory.internalSize = device->internalSize;
    Interface->u.QueryVideoMemory.internalPhysName = device->internalPhysName;

    /* Get external memory size and physical address. */
    Interface->u.QueryVideoMemory.externalSize = device->externalSize;
    Interface->u.QueryVideoMemory.externalPhysName = device->externalPhysName;

    /* Get contiguous memory size and physical address. */
    Interface->u.QueryVideoMemory.contiguousSize = device->contiguousSize;
    Interface->u.QueryVideoMemory.contiguousPhysName = device->contiguousPhysName;

    /* Get exclusive memory size and physical address. */
    Interface->u.QueryVideoMemory.exclusiveSize = device->exclusiveSize;
    Interface->u.QueryVideoMemory.exclusivePhysName = device->exclusivePhysName;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckKERNEL_GetVideoMemoryPool
**
**  Get the gckVIDMEM object belonging to the specified pool.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcePOOL Pool
**          Pool to query gckVIDMEM object for.
**
**  OUTPUT:
**
**      gckVIDMEM * VideoMemory
**          Pointer to a variable that will hold the pointer to the gckVIDMEM
**          object belonging to the requested pool.
*/
gceSTATUS
gckKERNEL_GetVideoMemoryPool(
    IN gckKERNEL Kernel,
    IN gcePOOL Pool,
    OUT gckVIDMEM * VideoMemory
    )
{
    gckGALDEVICE device;
    gckVIDMEM videoMemory;

    gcmkHEADER_ARG("Kernel=%p Pool=%d", Kernel, Pool);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(VideoMemory != NULL);

    /* Extract the pointer to the gckGALDEVICE class. */
    device = (gckGALDEVICE) Kernel->context;

    /* Dispatch on pool. */
    switch (Pool)
    {
    case gcvPOOL_LOCAL_INTERNAL:
        /* Internal memory. */
        videoMemory = device->internalVidMem;
        break;

    case gcvPOOL_LOCAL_EXTERNAL:
        /* External memory. */
        videoMemory = device->externalVidMem;
        break;

    case gcvPOOL_SYSTEM:
        /* System memory. */
        videoMemory = device->contiguousVidMem;
        break;

    case gcvPOOL_LOCAL_EXCLUSIVE:
        /* gpu exclusive memory. */
        videoMemory = device->exclusiveVidMem;
        break;

    case gcvPOOL_INTERNAL_SRAM:
        /* Internal SRAM memory. */
        videoMemory = Kernel->sRAMVidMem[Kernel->sRAMIndex];
        break;

    case gcvPOOL_EXTERNAL_SRAM:
        /* External SRAM memory. */
        videoMemory = device->extSRAMVidMem[Kernel->extSRAMIndex];
        break;

    default:
        /* Unknown pool. */
        videoMemory = NULL;
    }

    /* Return pointer to the gckVIDMEM object. */
    *VideoMemory = videoMemory;

    /* Return status. */
    gcmkFOOTER_ARG("*VideoMemory=%p", *VideoMemory);
    return (videoMemory == NULL) ? gcvSTATUS_OUT_OF_MEMORY : gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckKERNEL_MapMemory
**
**  Map video memory into the current process space.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctPHYS_ADDR Physical
**          Physical address of video memory to map.
**
**      gctSIZE_T Bytes
**          Number of bytes to map.
**
**  OUTPUT:
**
**      gctPOINTER * Logical
**          Pointer to a variable that will hold the base address of the mapped
**          memory region.
*/
gceSTATUS
gckKERNEL_MapMemory(
    IN gckKERNEL Kernel,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    )
{
    gckKERNEL kernel = Kernel;
    gctPHYS_ADDR physical = gcmNAME_TO_PTR(Physical);

    return gckOS_MapMemory(Kernel->os, physical, Bytes, Logical);
}

/*******************************************************************************
**
**  gckKERNEL_UnmapMemory
**
**  Unmap video memory from the current process space.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctPHYS_ADDR Physical
**          Physical address of video memory to map.
**
**      gctSIZE_T Bytes
**          Number of bytes to map.
**
**      gctPOINTER Logical
**          Base address of the mapped memory region.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_UnmapMemory(
    IN gckKERNEL Kernel,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical,
    IN gctUINT32 ProcessID
    )
{
    gckKERNEL kernel = Kernel;
    gctPHYS_ADDR physical = gcmNAME_TO_PTR(Physical);

    return gckOS_UnmapMemoryEx(Kernel->os, physical, Bytes, Logical, ProcessID);
}

/****************************************************************************
**
**  gckKERNEL_DestroyProcessReservedUserMap
**
**  Destroy process reserved memory
**
**  INPUT:
**
**      gctPHYS_ADDR Physical
**          Physical address of video memory to map.
**
**      gctUINT32 Pid
**          Process ID.
*/
gceSTATUS
gckKERNEL_DestroyProcessReservedUserMap(
    IN gckKERNEL Kernel,
    IN gctUINT32 Pid
    )
{
    gceSTATUS status      = gcvSTATUS_OK;
    gckGALDEVICE device   = gcvNULL;
    gctSIZE_T bytes       = 0;
    gctPHYS_ADDR physHandle = gcvNULL;
    /* when unmap reserved memory, we don't need real logical*/
    gctPOINTER Logical = (gctPOINTER)0xFFFFFFFF;
    gctINT i;
    PLINUX_MDL mdl;
    PLINUX_MDL_MAP mdlMap = gcvNULL;

    gcmkHEADER_ARG("Logical=0x%08x pid=%u",
                   Logical, Pid);
    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    /* Extract the pointer to the gckGALDEVICE class. */
    device = (gckGALDEVICE) Kernel->context;

    physHandle = (PLINUX_MDL)device->internalPhysical;
    bytes = device->internalSize;
    if (bytes)
    {
        mdl = physHandle;
        mutex_lock(&mdl->mapsMutex);
        mdlMap = FindMdlMap(mdl, Pid);
        mutex_unlock(&mdl->mapsMutex);
        if (mdlMap)
        {
            gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
        }
    }

    physHandle = (PLINUX_MDL)device->externalPhysical;
    bytes = device->externalSize;
    if (bytes)
    {
        mdl = physHandle;
        mutex_lock(&mdl->mapsMutex);
        mdlMap = FindMdlMap(mdl, Pid);
        mutex_unlock(&mdl->mapsMutex);
        if (mdlMap)
        {
            gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
        }
    }

    /* System memory. */
    physHandle = (PLINUX_MDL)device->contiguousPhysical;
    bytes = device->contiguousSize;
    if (bytes)
    {
        mdl = physHandle;
        mutex_lock(&mdl->mapsMutex);
        mdlMap = FindMdlMap(mdl, Pid);
        mutex_unlock(&mdl->mapsMutex);
        if (mdlMap)
        {
            gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
        }
    }

    /* External shared SRAM memory. */
    for(i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        physHandle = (PLINUX_MDL)device->extSRAMPhysical[i];
        bytes = device->extSRAMSizes[i];
        if (bytes)
        {
            mdl = physHandle;
            mutex_lock(&mdl->mapsMutex);
            mdlMap = FindMdlMap(mdl, Pid);
            mutex_unlock(&mdl->mapsMutex);
            if (mdlMap)
            {
                gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
            }
        }
    }

    /* Per core SRAM reserved usage. */
    for(i = 0; i < gcvSRAM_INTER_COUNT; i++)
    {
        if (!Kernel->sRAMPhysFaked[i])
        {
            physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[i];
            bytes = Kernel->sRAMSizes[i];
            if (bytes)
            {
                mdl = physHandle;
                mutex_lock(&mdl->mapsMutex);
                mdlMap = FindMdlMap(mdl, Pid);
                mutex_unlock(&mdl->mapsMutex);
                if (mdlMap)
                {
                    gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
                }
            }
        }
    }

    /* Retunn the status. */
    gcmkFOOTER_NO();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_MapVideoMemory
**
**  Get the logical address for a hardware specific memory address for the
**  current process.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL InUserSpace
**          gcvTRUE to map the memory into the user space.
**
**      gcePOOL Pool
**          Specify pool type.
**
**      gctUINT32 Offset
**          Offset to pool start.
**
**      gctUINT32 Bytes
**          Number of bytes to map.
**
**  OUTPUT:
**
**      gctPOINTER * Logical
**          Pointer to a variable that will hold the logical address of the
**          specified memory address.
*/
gceSTATUS
gckKERNEL_MapVideoMemory(
    IN gckKERNEL Kernel,
    IN gctBOOL InUserSpace,
    IN gcePOOL Pool,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 Offset,
    IN gctUINT32 Bytes,
    OUT gctPOINTER * Logical
    )
{
    gckGALDEVICE device   = gcvNULL;
    gctSIZE_T bytes       = 0;
    gctPHYS_ADDR physHandle = gcvNULL;
    gceSTATUS status      = gcvSTATUS_OK;
    gctPOINTER logical    = gcvNULL;
    gctUINT64 mappingInOne  = 1;

    gcmkHEADER_ARG("Kernel=%p InUserSpace=%d Pool=%d Offset=%X Bytes=%X",
                   Kernel, InUserSpace, Pool, Offset, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Logical != NULL);

    if (Physical)
    {
        gcmkONERROR(gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne));
    }

    if (mappingInOne)
    {
        /* Extract the pointer to the gckGALDEVICE class. */
        device = (gckGALDEVICE) Kernel->context;

        /* Dispatch on pool. */
        switch (Pool)
        {
        case gcvPOOL_LOCAL_INTERNAL:
            physHandle = (PLINUX_MDL)device->internalPhysical;
            bytes = device->internalSize;
            break;

        case gcvPOOL_LOCAL_EXTERNAL:
            physHandle = (PLINUX_MDL)device->externalPhysical;
            bytes = device->externalSize;
            break;

        case gcvPOOL_SYSTEM:
            /* System memory. */
            physHandle = (PLINUX_MDL)device->contiguousPhysical;
            bytes = device->contiguousSize;
            break;

        case gcvPOOL_EXTERNAL_SRAM:
            /* External shared SRAM memory. */
            physHandle = (PLINUX_MDL)device->extSRAMPhysical[Kernel->extSRAMIndex];
            bytes = device->extSRAMSizes[Kernel->extSRAMIndex];
            break;

        case gcvPOOL_INTERNAL_SRAM:
            /* Per core SRAM reserved usage. */
            if (Kernel->sRAMPhysFaked[Kernel->sRAMIndex])
            {
                *Logical = gcvNULL;

                gcmkFOOTER_NO();
                return gcvSTATUS_OK;
            }
            /* Per core SRAM memory block. */
            else
            {
                physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[Kernel->sRAMIndex];
                bytes = Kernel->sRAMSizes[Kernel->sRAMIndex];
                break;
            }

        default:
            /* Invalid memory pool. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

    }
    else
    {
        physHandle = (PLINUX_MDL)Physical;
        bytes = Bytes;
        Offset = 0;
    }

    gcmkONERROR(gckOS_LockPages(Kernel->os, physHandle, bytes, gcvFALSE, &logical));
    /* Build logical address of specified address. */
    *Logical = (gctPOINTER)((gctUINT8_PTR)logical + Offset);
OnError:
    /* Retunn the status. */
    gcmkFOOTER_ARG("*Logical=%p", gcmOPT_POINTER(Logical));
    return status;
}


/*******************************************************************************
**
**  gckKERNEL_UnmapVideoMemory
**
**  Unmap video memory for the current process.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcePOOL Pool
**          Specify pool type.

**      gctUINT32 Address
**          Hardware specific memory address.
**
**      gctUINT32 Pid
**          Process ID of the current process.
**
**      gctSIZE_T Bytes
**          Number of bytes to map.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_UnmapVideoMemory(
    IN gckKERNEL Kernel,
    IN gcePOOL Pool,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical,
    IN gctUINT32 Pid,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status      = gcvSTATUS_OK;
    gckGALDEVICE device   = gcvNULL;
    gctSIZE_T bytes       = 0;
    gctPHYS_ADDR physHandle = gcvNULL;
    gctUINT64 mappingInOne  = 1;

    gcmkHEADER_ARG("Logical=0x%08x pid=%u Bytes=%u",
                   Logical, Pid, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Logical == gcvNULL)
    {
        return gcvSTATUS_OK;
    }

    if (Physical)
    {
        gcmkONERROR(gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne));
    }

    if (mappingInOne)
    {
        /* Extract the pointer to the gckGALDEVICE class. */
        device = (gckGALDEVICE) Kernel->context;

        /* Dispatch on pool. */
        switch (Pool)
        {
        case gcvPOOL_LOCAL_INTERNAL:
            physHandle = (PLINUX_MDL)device->internalPhysical;
            bytes = device->internalSize;
            break;

        case gcvPOOL_LOCAL_EXTERNAL:
            physHandle = (PLINUX_MDL)device->externalPhysical;
            bytes = device->externalSize;
            break;

        case gcvPOOL_SYSTEM:
            /* System memory. */
            physHandle = (PLINUX_MDL)device->contiguousPhysical;
            bytes = device->contiguousSize;
            break;

        case gcvPOOL_EXTERNAL_SRAM:
            /* External shared SRAM memory. */
            physHandle = (PLINUX_MDL)device->extSRAMPhysical[Kernel->extSRAMIndex];
            bytes = device->extSRAMSizes[Kernel->extSRAMIndex];
            break;

        case gcvPOOL_INTERNAL_SRAM:
            /* Per core SRAM reserved usage. */
            if (Kernel->sRAMPhysFaked[Kernel->sRAMIndex])
            {
                gcmkFOOTER_NO();
                return gcvSTATUS_OK;
            }
            /* Per core SRAM memory block. */
            else
            {
                physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[Kernel->sRAMIndex];
                bytes = Kernel->sRAMSizes[Kernel->sRAMIndex];
                break;
            }

        default:
            /* Invalid memory pool. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
    }
    else
    {
        physHandle = (PLINUX_MDL)Physical;
        bytes = Bytes;
    }

    gcmkONERROR(gckOS_UnlockPages(Kernel->os, physHandle, bytes, Logical));

OnError:
    /* Retunn the status. */
    gcmkFOOTER_NO();
    return status;

}

/*******************************************************************************
**
**  gckKERNEL_Notify
**
**  This function iscalled by clients to notify the gckKERNRL object of an event.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gceNOTIFY Notification
**          Notification event.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Notify(
    IN gckKERNEL Kernel,
    IN gceNOTIFY Notification
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Kernel=%p Notification=%d", Kernel, Notification);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Dispatch on notifcation. */
    switch (Notification)
    {
    case gcvNOTIFY_INTERRUPT:
        /* Process the interrupt. */
#if COMMAND_PROCESSOR_VERSION > 1
        status = gckINTERRUPT_Notify(Kernel->interrupt, 0);
#else
        status = gckHARDWARE_Notify(Kernel->hardware);
#endif
        break;

    default:
        break;
    }

    /* Success. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SyncVideoMemory(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node,
    IN gctUINT32 Reason
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gcsPLATFORM * platform;

    gcmkHEADER();

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    platform = Kernel->os->device->platform;

    if (platform && platform->ops->syncMemory)
    {
        status = platform->ops->syncMemory(Kernel, (gctPOINTER)Node, Reason);
    }

    gcmkFOOTER();
    return status;
}

