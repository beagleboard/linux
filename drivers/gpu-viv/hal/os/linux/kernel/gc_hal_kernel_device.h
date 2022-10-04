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


#ifndef __gc_hal_kernel_device_h_
#define __gc_hal_kernel_device_h_

#include "gc_hal_kernel_debugfs.h"
#include "gc_hal_ta.h"

/******************************************************************************\
************************** gckGALDEVICE Structure ******************************
\******************************************************************************/

typedef struct _gckGALDEVICE
{
    /* Objects. */
    gckOS               os;
    gckKERNEL           kernels[gcdMAX_GPU_COUNT];

    gcsPLATFORM*        platform;

    /* Attributes. */
    gctPHYS_ADDR_T      internalBase;
    gctSIZE_T           internalSize;
    gctPHYS_ADDR        internalPhysical;
    gctUINT32           internalPhysName;
    gctPOINTER          internalLogical;
    gckVIDMEM           internalVidMem;

    gctPHYS_ADDR_T      externalBase;
    gctSIZE_T           externalSize;
    gctPHYS_ADDR        externalPhysical;
    gctUINT32           externalPhysName;
    gctPOINTER          externalLogical;
    gckVIDMEM           externalVidMem;

    /* Shared external SRAMs. */
    gctPHYS_ADDR_T      extSRAMBases[gcvSRAM_EXT_COUNT];
    gctSIZE_T           extSRAMSizes[gcvSRAM_EXT_COUNT];
    gctPHYS_ADDR        extSRAMPhysical[gcvSRAM_EXT_COUNT];
    gckVIDMEM           extSRAMVidMem[gcvSRAM_EXT_COUNT];

    gctPHYS_ADDR_T      contiguousBase;
    gctSIZE_T           contiguousSize;
    gctPHYS_ADDR        contiguousPhysical;
    gctUINT32           contiguousPhysName;
    gctPOINTER          contiguousLogical;
    gckVIDMEM           contiguousVidMem;

    gctPHYS_ADDR_T      exclusiveBase;
    gctSIZE_T           exclusiveSize;
    gctPHYS_ADDR        exclusivePhysical;
    gctUINT32           exclusivePhysName;
    gctPOINTER          exclusiveLogical;
    gckVIDMEM           exclusiveVidMem;

    /* By request_mem_region. */
    gctUINT64           requestedContiguousBase;
    gctSIZE_T           requestedContiguousSize;

    /* IRQ management. */
    gctINT              irqLines[gcdMAX_GPU_COUNT];
    gctBOOL             isrInitializeds[gcdMAX_GPU_COUNT];
    struct task_struct  *isrThread[gcdMAX_GPU_COUNT];
    gctBOOL             killIsrThread;

    /* Register memory. */
    gctPOINTER          registerBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           registerSizes[gcdMAX_GPU_COUNT];

    /* By request_mem_region. */
    gctUINT64           requestedRegisterMemBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           requestedRegisterMemSizes[gcdMAX_GPU_COUNT];

    gctUINT32           baseAddress;
    gctUINT32           physBase;
    gctUINT32           physSize;


    /* PCIE Bar */
    gctINT              bars[gcdMAX_GPU_COUNT];

    /* Thread management. */
    struct task_struct *threadCtxts[gcdMAX_GPU_COUNT];
    struct semaphore    semas[gcdMAX_GPU_COUNT];
    gctBOOL             threadInitializeds[gcdMAX_GPU_COUNT];
    gctBOOL             killThread;

    /* States before suspend. */
    gceCHIPPOWERSTATE   statesStored[gcdMAX_GPU_COUNT];

    gcsDEBUGFS_DIR      debugfsDir;

    gckDEVICE           device;

    gcsMODULE_PARAMETERS args;

    /* gctsOs object for trust application. */
    gctaOS              taos;

#if gcdENABLE_DRM
    void *              drm;
#endif

#if gcdENABLE_SW_PREEMPTION
    struct task_struct *preemptThread[gcdMAX_GPU_COUNT];
    struct semaphore    preemptSemas[gcdMAX_GPU_COUNT];
    gctBOOL             preemptThreadInits[gcdMAX_GPU_COUNT];
    gctBOOL             killPreemptThread;
#endif
}
* gckGALDEVICE;

typedef struct _gcsHAL_PRIVATE_DATA
{
    gckGALDEVICE        device;
    /*
     * 'fput' schedules actual work in '__fput' in a different thread.
     * So the process opens the device may not be the same as the one that
     * closes it.
     */
    gctUINT32           pidOpen;
    gctBOOL             isLocked;
}
gcsHAL_PRIVATE_DATA, * gcsHAL_PRIVATE_DATA_PTR;

gceSTATUS
gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    );

gceSTATUS
gckGALDEVICE_Stop(
    gckGALDEVICE Device
    );

gceSTATUS
gckGALDEVICE_Construct(
    IN gcsPLATFORM * Platform,
    IN const gcsMODULE_PARAMETERS * Args,
    OUT gckGALDEVICE *Device
    );

gceSTATUS
gckGALDEVICE_Destroy(
    IN gckGALDEVICE Device
    );

static gcmINLINE gckKERNEL
_GetValidKernel(
    gckGALDEVICE Device
    )
{
    if (Device->kernels[gcvCORE_MAJOR])
    {
        return Device->kernels[gcvCORE_MAJOR];
    }
    else if (Device->kernels[gcvCORE_2D])
    {
        return Device->kernels[gcvCORE_2D];
    }
    else if (Device->kernels[gcvCORE_VG])
    {
        return Device->kernels[gcvCORE_VG];
    }
    else
    {
        gcmkASSERT(gcvFALSE);
        return gcvNULL;
    }
}

#endif /* __gc_hal_kernel_device_h_ */
