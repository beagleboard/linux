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


#include "gc_hal_kernel_precomp.h"

#if gcdDEC_ENABLE_AHB
#include "viv_dec300_main.h"
#endif

#if gcdCAPTURE_ONLY_MODE
#include "arch/gc_hal_kernel_context.h"
#endif

#define _GC_OBJ_ZONE    gcvZONE_KERNEL

/*******************************************************************************
***** Version Signature *******************************************************/

#define _gcmTXT2STR(t) #t
#define gcmTXT2STR(t) _gcmTXT2STR(t)
const char * _VERSION = "\n\0$VERSION$"
                        gcmTXT2STR(gcvVERSION_MAJOR) "."
                        gcmTXT2STR(gcvVERSION_MINOR) "."
                        gcmTXT2STR(gcvVERSION_PATCH) ":"
                        gcmTXT2STR(gcvVERSION_BUILD) "$\n";

/******************************************************************************\
******************************* gckKERNEL API Code ******************************
\******************************************************************************/

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
#define gcmDEFINE2TEXT(d) #d
gctCONST_STRING _DispatchText[] =
{
    gcmDEFINE2TEXT(gcvHAL_CHIP_INFO),
    gcmDEFINE2TEXT(gcvHAL_VERSION),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_IDENTITY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_OPTION),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_FREQUENCY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_WRAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_RELEASE_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_LOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNLOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_MAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNMAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_CACHE),
    gcmDEFINE2TEXT(gcvHAL_ATTACH),
    gcmDEFINE2TEXT(gcvHAL_DETACH),
    gcmDEFINE2TEXT(gcvHAL_EVENT_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_SET_TIMEOUT),
    gcmDEFINE2TEXT(gcvHAL_USER_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_SET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_READ_PROFILER_REGISTER_SETTING),
    gcmDEFINE2TEXT(gcvHAL_READ_ALL_PROFILE_REGISTERS_PART1),
    gcmDEFINE2TEXT(gcvHAL_READ_ALL_PROFILE_REGISTERS_PART2),
    gcmDEFINE2TEXT(gcvHAL_DATABASE),
    gcmDEFINE2TEXT(gcvHAL_CONFIG_POWER_MANAGEMENT),
    gcmDEFINE2TEXT(gcvHAL_DEBUG_DUMP),
    gcmDEFINE2TEXT(gcvHAL_READ_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_WRITE_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_PROFILE_REGISTERS_2D),
    gcmDEFINE2TEXT(gcvHAL_GET_BASE_ADDRESS),
    gcmDEFINE2TEXT(gcvHAL_GET_FRAME_INFO),
    gcmDEFINE2TEXT(gcvHAL_SET_VIDEO_MEMORY_METADATA),
    gcmDEFINE2TEXT(gcvHAL_QUERY_COMMAND_BUFFER),
    gcmDEFINE2TEXT(gcvHAL_QUERY_RESET_TIME_STAMP),
    gcmDEFINE2TEXT(gcvHAL_CREATE_NATIVE_FENCE),
    gcmDEFINE2TEXT(gcvHAL_WAIT_NATIVE_FENCE),
    gcmDEFINE2TEXT(gcvHAL_WAIT_FENCE),
    gcmDEFINE2TEXT(gcvHAL_EXPORT_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_NAME_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_IMPORT_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_DEVICE_MUTEX),
    gcmDEFINE2TEXT(gcvHAL_DEC200_TEST),
    gcmDEFINE2TEXT(gcvHAL_DEC300_READ),
    gcmDEFINE2TEXT(gcvHAL_DEC300_WRITE),
    gcmDEFINE2TEXT(gcvHAL_DEC300_FLUSH),
    gcmDEFINE2TEXT(gcvHAL_DEC300_FLUSH_WAIT),
    gcmDEFINE2TEXT(gcvHAL_SHBUF),
    gcmDEFINE2TEXT(gcvHAL_GET_GRAPHIC_BUFFER_FD),
    gcmDEFINE2TEXT(gcvHAL_UPDATE_DEBUG_CALLBACK),
    gcmDEFINE2TEXT(gcvHAL_CONFIG_CTX_FRAMEWORK),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_WRITE_DATA),
    gcmDEFINE2TEXT(gcvHAL_APB_AXIFE_ACCESS),
    gcmDEFINE2TEXT(gcvHAL_RESET),
    gcmDEFINE2TEXT(gcvHAL_COMMIT_DONE),
    gcmDEFINE2TEXT(gcvHAL_GET_VIDEO_MEMORY_FD),
    gcmDEFINE2TEXT(gcvHAL_GET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_READ_REGISTER_EX),
    gcmDEFINE2TEXT(gcvHAL_WRITE_REGISTER_EX),
    gcmDEFINE2TEXT(gcvHAL_SET_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_SET_DEBUG_LEVEL_ZONE),
    gcmDEFINE2TEXT(gcvHAL_DUMP_GPU_STATE),
    gcmDEFINE2TEXT(gcvHAL_SYNC_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_DUMP_GPU_PROFILE),
    gcmDEFINE2TEXT(gcvHAL_TIMESTAMP),
    gcmDEFINE2TEXT(gcvHAL_SET_FSCALE_VALUE),
    gcmDEFINE2TEXT(gcvHAL_GET_FSCALE_VALUE),
    gcmDEFINE2TEXT(gcvHAL_DESTROY_MMU),
};
#endif

#if gcdGPU_TIMEOUT && gcdINTERRUPT_STATISTIC
void
_MonitorTimerFunction(
    gctPOINTER Data
    )
{
    gckKERNEL kernel = (gckKERNEL)Data;
    gctINT32 pendingInterrupt;
    gctBOOL reset = gcvFALSE;
    gctINT32 mask;
    gctUINT32 advance = kernel->timeOut/2;


    if (kernel->monitorTimerStop)
    {
        /* Stop. */
        return;
    }

    gckOS_AtomGet(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);

    if (pendingInterrupt < 0)
    {
        gctINT i = 0 - pendingInterrupt;
        gctINT pendingMask;

        gcmkVERIFY_OK(gckOS_AtomGet(
            kernel->os,
            kernel->hardware->pendingEvent,
            &pendingMask
            ));

        gcmkTRACE_N(
            gcvLEVEL_ERROR,
            gcmSIZEOF(pendingInterrupt) + gcmSIZEOF(pendingMask),
            "[galcore]: Number of pending interrupt is %d mask is %x",
            pendingInterrupt, pendingMask
            );

        while (i--)
        {
            /* Ignore counting which should not exist. */
            gckOS_AtomIncrement(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);
        }

        gckOS_AtomGet(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);
    }

    if (kernel->monitoring == gcvFALSE)
    {
        if (pendingInterrupt)
        {
            /* Begin to mointor GPU state. */
            kernel->monitoring = gcvTRUE;

            /* Record current state. */
            kernel->lastCommitStamp = kernel->eventObj->lastCommitStamp;
            kernel->restoreAddress  = kernel->hardware->lastWaitLink;
            gcmkVERIFY_OK(gckOS_AtomGet(
                kernel->os,
                kernel->hardware->pendingEvent,
                &kernel->restoreMask
                ));

            /* Clear timeout. */
            kernel->timer = 0;
        }
    }
    else
    {
        if (pendingInterrupt)
        {
            gcmkVERIFY_OK(gckOS_AtomGet(
                kernel->os,
                kernel->hardware->pendingEvent,
                &mask
                ));

            if (kernel->eventObj->lastCommitStamp == kernel->lastCommitStamp
             && kernel->hardware->lastWaitLink    == kernel->restoreAddress
             && mask                              == kernel->restoreMask
            )
            {
                /* GPU state is not changed, accumlate timeout. */
                kernel->timer += advance;

                if (kernel->timer >= kernel->timeOut)
                {
                    /* GPU stuck, trigger reset. */
                    reset = gcvTRUE;
                }
            }
            else
            {
                /* GPU state changed, cancel current timeout.*/
                kernel->monitoring = gcvFALSE;
            }
        }
        else
        {
            /* GPU finish all jobs, cancel current timeout*/
            kernel->monitoring = gcvFALSE;
        }
    }

    if (reset)
    {
        gckKERNEL_Recovery(kernel);

        /* Work in this timeout is done. */
        kernel->monitoring = gcvFALSE;
    }

    gcmkVERIFY_OK(gckOS_StartTimer(kernel->os, kernel->monitorTimer, advance));
}
#endif

void
_DumpDriverConfigure(
    IN gckKERNEL Kernel
    )
{
    gcmkPRINT_N(0, "**************************\n");
    gcmkPRINT_N(0, "***   GPU DRV CONFIG   ***\n");
    gcmkPRINT_N(0, "**************************\n");

    gcmkPRINT("Galcore version %d.%d.%d.%d\n",
              gcvVERSION_MAJOR, gcvVERSION_MINOR, gcvVERSION_PATCH, gcvVERSION_BUILD);

    gckOS_DumpParam();
}

void
_DumpState(
    IN gckKERNEL Kernel
    )
{
    /* Dump GPU Debug registers. */
    gcmkVERIFY_OK(gckHARDWARE_DumpGPUState(Kernel->hardware));

    /* Dump Pending event. */
    gcmkVERIFY_OK(gckEVENT_Dump(Kernel->eventObj));

    /* Dump Process DB. */
    gcmkVERIFY_OK(gckKERNEL_DumpProcessDB(Kernel));

#if gcdRECORD_COMMAND
    /* Dump record. */
    gckRECORDER_Dump(Kernel->command->recorder);
#endif

    if (Kernel->command)
    {
        gcmkVERIFY_OK(gckCOMMAND_DumpExecutingBuffer(Kernel->command));
    }
}

gceSTATUS
gckKERNEL_GetHardwareType(
    IN gckKERNEL Kernel,
    OUT gceHARDWARE_TYPE *Type
    )
{
    gceHARDWARE_TYPE type;
    gcmkHEADER();
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    {
        type = Kernel->hardware->type;
    }

    *Type = type;

    gcmkFOOTER_ARG("type=%d", type);
    return gcvSTATUS_OK;
}

gceSTATUS
_SetRecovery(
    IN gckKERNEL Kernel,
    IN gctBOOL  Recovery,
    IN gctUINT32 StuckDump
    )
{
    Kernel->recovery = Recovery;

    if (Recovery == gcvFALSE)
    {
        /* Dump stuck information if Recovery is disabled. */
        Kernel->stuckDump = gcmMAX(StuckDump, gcvSTUCK_DUMP_USER_COMMAND);
    }

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckKERNEL_Construct
**
**  Construct a new gckKERNEL object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gceCORE Core
**          Specified core.
**
**      IN gctPOINTER Context
**          Pointer to a driver defined context.
**
**      IN gckDB SharedDB,
**          Pointer to a shared DB.
**
**  OUTPUT:
**
**      gckKERNEL * Kernel
**          Pointer to a variable that will hold the pointer to the gckKERNEL
**          object.
*/

gceSTATUS
gckKERNEL_Construct(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT ChipID,
    IN gctPOINTER Context,
    IN gckDEVICE Device,
    IN gckDB SharedDB,
    OUT gckKERNEL * Kernel
    )
{
    gckKERNEL kernel = gcvNULL;
    gceSTATUS status;
    gctSIZE_T i;
    gctPOINTER pointer = gcvNULL;
    gctUINT64 data;
    gctUINT32 recovery;
    gctUINT32 stuckDump;
    gctUINT64 dynamicMap = 1;

    gcmkHEADER_ARG("Os=%p Context=%p", Os, Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    /* Allocate the gckKERNEL object. */
    gcmkONERROR(gckOS_Allocate(Os,
                               gcmSIZEOF(struct _gckKERNEL),
                               &pointer));

    /* Zero the object. */
    gckOS_ZeroMemory(pointer, gcmSIZEOF(struct _gckKERNEL));

    kernel = pointer;

    /* Initialize the gckKERNEL object. */
    kernel->object.type = gcvOBJ_KERNEL;
    kernel->os          = Os;
    kernel->core        = Core;
    kernel->device      = Device;
    kernel->chipID      = ChipID;
    kernel->threadInitialized = gcvTRUE;

#if gcdENABLE_TRUST_APPLICATION
    /* Connect to security service for this GPU. */
    gcmkONERROR(gckKERNEL_SecurityOpen(kernel, kernel->core, &kernel->securityChannel));
#endif

    if (SharedDB == gcvNULL)
    {
        gcmkONERROR(gckOS_Allocate(Os,
                                   gcmSIZEOF(struct _gckDB),
                                   &pointer));

        kernel->db               = pointer;
        kernel->dbCreated        = gcvTRUE;
        kernel->db->freeDatabase = gcvNULL;
        kernel->db->freeRecord   = gcvNULL;
        kernel->db->dbMutex      = gcvNULL;
        kernel->db->lastDatabase = gcvNULL;
        kernel->db->idleTime     = 0;
        kernel->db->lastIdle     = 0;
        kernel->db->lastSlowdown = 0;

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
        {
            kernel->db->db[i] = gcvNULL;
        }

        /* Construct a database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->dbMutex));

        /* Construct a video memory name database. */
        gcmkONERROR(gckKERNEL_CreateIntegerDatabase(
            kernel,
            512,
            &kernel->db->nameDatabase
            ));

        /* Construct a video memory name database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->nameDatabaseMutex));

        /* Construct a pointer name database. */
        gcmkONERROR(gckKERNEL_CreateIntegerDatabase(
            kernel,
            512,
            &kernel->db->pointerDatabase
            ));

        /* Initialize video memory node list. */
        gcsLIST_Init(&kernel->db->videoMemList);

        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->videoMemListMutex));
    }
    else
    {
        kernel->db               = SharedDB;
        kernel->dbCreated        = gcvFALSE;
    }

    for (i = 0; i < gcmCOUNTOF(kernel->timers); ++i)
    {
        kernel->timers[i].startTime = 0;
        kernel->timers[i].stopTime = 0;
    }

    gcmkONERROR(gckOS_CreateMutex(Os, &kernel->vidMemBlockMutex));

    /* Save context. */
    kernel->context = Context;

    /* Construct atom holding number of clients. */
    kernel->atomClients = gcvNULL;
    gcmkONERROR(gckOS_AtomConstruct(Os, &kernel->atomClients));

    kernel->recovery  = gcvTRUE;
    kernel->stuckDump = gcvSTUCK_DUMP_NONE;

    /* Override default recovery and stuckDump setting. */
    status = gckOS_QueryOption(Os, "recovery", &data);
    recovery = (gctUINT32)data;

    if (gcmIS_SUCCESS(status))
    {
        status = gckOS_QueryOption(Os, "stuckDump", &data);
        stuckDump = (gctUINT32)data;

        gcmkASSERT(status == gcvSTATUS_OK);

        _SetRecovery(kernel, recovery, stuckDump);
    }

    status = gckOS_QueryOption(Os, "sRAMLoopMode", &data);
    kernel->sRAMLoopMode = (status == gcvSTATUS_OK) ? data : 0;

    /* Need the kernel reference before gckKERNEL_Construct() completes.
       gckOS_MapPagesEx() is called to map kernel virtual command buffers. */
    *Kernel = kernel;

    {
        /* Construct the gckHARDWARE object. */
        gcmkONERROR(
            gckHARDWARE_Construct(Os, kernel->device, kernel->core, &kernel->hardware));

        /* Set pointer to gckKERNEL object in gckHARDWARE object. */
        kernel->hardware->kernel = kernel;

        kernel->sRAMIndex = 0;
        kernel->extSRAMIndex = 0;

        for (i = gcvSRAM_INTERNAL0; i < gcvSRAM_INTER_COUNT; i++)
        {
            kernel->sRAMVidMem[i]    = kernel->hardware->sRAMVidMem[i];
            kernel->sRAMPhysical[i]  = kernel->hardware->sRAMPhysical[i];
            kernel->sRAMPhysFaked[i] = gcvFALSE;
        }

        kernel->timeOut = kernel->hardware->type == gcvHARDWARE_2D
                        ? gcdGPU_2D_TIMEOUT
                        : gcdGPU_TIMEOUT
                        ;

#if gcdSHARED_PAGETABLE
        /* Construct the gckMMU object. */
        gcmkONERROR(
            gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));
#else
        if (Device == gcvNULL)
        {
            /* Construct the gckMMU object. */
            gcmkONERROR(
                gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));
        }
        else
        {
            gcmkONERROR(gckDEVICE_GetMMU(Device, kernel->hardware->type, &kernel->mmu));

            if (kernel->mmu == gcvNULL)
            {
                gcmkONERROR(
                    gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));

                gcmkONERROR(
                    gckDEVICE_SetMMU(Device, kernel->hardware->type, kernel->mmu));
            }
        }

        gcmkONERROR(
            gckMMU_SetupSRAM(kernel->mmu, kernel->hardware, kernel->device));

        status = gckOS_QueryOption(Os, "mmuDynamicMap", &dynamicMap);
        if (dynamicMap && kernel->hardware->mmuVersion && !kernel->mmu->dynamicAreaSetuped)
        {
            gcmkONERROR(
                gckMMU_SetupDynamicSpace(kernel->mmu));

            kernel->mmu->dynamicAreaSetuped = gcvTRUE;
        }

        if (kernel->hardware->mmuVersion > 0)
        {
            /* Flush MTLB table. */
            gcmkONERROR(gckVIDMEM_NODE_CleanCache(
                kernel,
                kernel->mmu->mtlbVideoMem,
                0,
                kernel->mmu->mtlbLogical,
                kernel->mmu->mtlbSize
                ));
        }
#endif

        kernel->contiguousBaseAddress = kernel->mmu->contiguousBaseAddress;
        kernel->externalBaseAddress   = kernel->mmu->externalBaseAddress;
        kernel->exclusiveBaseAddress  = kernel->mmu->exclusiveBaseAddress;

        /* Construct the gckCOMMAND object, either MCFE or wait-link FE can exist. */
        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware, gcvFEATURE_MCFE))
        {
            /* Construct the gckCOMMAND object for multi-channel FE. */
            gcmkONERROR(gckCOMMAND_Construct(kernel, gcvHW_FE_MULTI_CHANNEL, &kernel->command));

            /* Construct gckEVENT for multi-channel FE. */
            gcmkONERROR(gckEVENT_Construct(kernel, kernel->command, &kernel->eventObj));
        }
        else
        {
            /* Construct the gckCOMMAND object for legacy wait-link FE. */
            gcmkONERROR(gckCOMMAND_Construct(kernel, gcvHW_FE_WAIT_LINK, &kernel->command));

            /* Construct the gckEVENT object. */
            gcmkONERROR(gckEVENT_Construct(kernel, kernel->command, &kernel->eventObj));
        }

        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware, gcvFEATURE_ASYNC_BLIT))
        {
            /* Construct the gckCOMMAND object for BLT engine. */
            gcmkONERROR(gckCOMMAND_Construct(kernel, gcvHW_FE_ASYNC, &kernel->asyncCommand));

            /* Construct gckEVENT for BLT. */
            gcmkONERROR(gckEVENT_Construct(kernel, kernel->asyncCommand, &kernel->asyncEvent));
        }

        gcmkVERIFY_OK(gckOS_GetTime(&kernel->resetTimeStamp));

        /* Post construct hardware elements after MMU settle. */
        gcmkONERROR(gckHARDWARE_PostConstruct(kernel->hardware));

        /* Initialize the GPU. */
        gcmkONERROR(
            gckHARDWARE_InitializeHardware(kernel->hardware));

#if gcdDVFS
        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware,
                                           gcvFEATURE_DYNAMIC_FREQUENCY_SCALING))
        {
            gcmkONERROR(gckDVFS_Construct(kernel->hardware, &kernel->dvfs));
            gcmkONERROR(gckDVFS_Start(kernel->dvfs));
        }
#endif

#if COMMAND_PROCESSOR_VERSION == 1
        if (kernel->command)
        {
            /* Start the command queue. */
            gcmkONERROR(gckCOMMAND_Start(kernel->command));
        }

        if (kernel->asyncCommand)
        {
            /* Start the async command queue. */
            gcmkONERROR(gckCOMMAND_Start(kernel->asyncCommand));
        }
#endif
    }

#if VIVANTE_PROFILER
    /* Initialize profile setting */
    kernel->profiler.profileEnable = gcvFALSE;
    kernel->profiler.profileMode = gcvPROFILER_UNKNOWN_MODE;
    kernel->profiler.probeMode = gcvPROFILER_UNKNOWN_PROBE;
    kernel->profiler.profileCleanRegister = gcvTRUE;
#endif

#if gcdLINUX_SYNC_FILE
    gcmkONERROR(gckOS_CreateSyncTimeline(Os, Core, &kernel->timeline));
#endif


#if gcdGPU_TIMEOUT && gcdINTERRUPT_STATISTIC
    if (kernel->timeOut)
    {
        gcmkVERIFY_OK(gckOS_CreateTimer(
            Os,
            (gctTIMERFUNCTION)_MonitorTimerFunction,
            (gctPOINTER)kernel,
            &kernel->monitorTimer
            ));

        kernel->monitoring  = gcvFALSE;

        kernel->monitorTimerStop = gcvFALSE;

        gcmkVERIFY_OK(gckOS_StartTimer(
            Os,
            kernel->monitorTimer,
            100
            ));
    }
#endif

#if gcdENABLE_SW_PREEMPTION
    gcmkONERROR(gckOS_CreateSemaphore(Os, &kernel->preemptSema));

    /* Init the priority queue. */
    for (i = 0; i < gcdMAX_PRIORITY_QUEUE_NUM; i++)
    {
        kernel->priorityQueues[i] = gcvNULL;
        kernel->priorityDBCreated[i] = gcvFALSE;
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->priorityQueueMutex[i]));
    }

    kernel->preemptionMode = gcvFULLY_PREEMPTIBLE_MODE;
#endif

    gcmkONERROR(gckOS_AtomConstruct(Os, &kernel->atomBroCoreMask));

    /* Initially all the cores are brothers. */
    gcmkONERROR(gckOS_AtomSet(Os, kernel->atomBroCoreMask, (1 << gcdMAX_MAJOR_CORE_COUNT) - 1));

    /* Return pointer to the gckKERNEL object. */
    *Kernel = kernel;

    /* Success. */
    gcmkFOOTER_ARG("*Kernel=%p", *Kernel);
    return gcvSTATUS_OK;

OnError:
    if (kernel != gcvNULL)
    {
        gckOS_SetGPUPower(Os, kernel->core, gcvFALSE, gcvFALSE);
        gckKERNEL_Destroy(kernel);
    }
    *Kernel = gcvNULL;

    /* Return the error. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_Destroy
**
**  Destroy an gckKERNEL object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object to destroy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Destroy(
    IN gckKERNEL Kernel
    )
{
    gctSIZE_T i;
    gcsDATABASE_PTR database, databaseNext;
    gcsDATABASE_RECORD_PTR record, recordNext;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
#if QNX_SINGLE_THREADED_DEBUGGING
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->debugMutex));
#endif

#if gcdENABLE_SW_PREEMPTION
    gcmkVERIFY_OK(gckOS_DestroySemaphore(Kernel->os, Kernel->preemptSema));

    for (i = 0; i < gcdMAX_PRIORITY_QUEUE_NUM; i++)
    {
        gcsPRIORITY_QUEUE_PTR queue = gcvNULL;

        gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->priorityQueueMutex[i]));
        queue = Kernel->priorityQueues[i];

        if (queue && !queue->head)
        {
            gcmkVERIFY_OK(gckKERNEL_PriorityQueueDestroy(Kernel, queue));
            Kernel->priorityQueues[i] = gcvNULL;
        }
    }
#endif

    if (Kernel->monitorTimer)
    {
        /* Stop and destroy monitor timer. */
        gcmkVERIFY_OK(gckOS_StopTimer(Kernel->os, Kernel->monitorTimer));
        gcmkVERIFY_OK(gckOS_DestroyTimer(Kernel->os, Kernel->monitorTimer));
    }

    {
        if (Kernel->command)
        {
            /* Destroy the gckCOMMNAND object. */
            gcmkVERIFY_OK(gckCOMMAND_Destroy(Kernel->command));
        }

        if (Kernel->asyncCommand)
        {
            gcmkVERIFY_OK(gckCOMMAND_Destroy(Kernel->asyncCommand));
        }

        if (Kernel->asyncEvent)
        {
            gcmkVERIFY_OK(gckEVENT_Destroy(Kernel->asyncEvent));
        }

        if (Kernel->eventObj)
        {
            /* Destroy the gckEVENT object. */
            gcmkVERIFY_OK(gckEVENT_Destroy(Kernel->eventObj));
        }

        /* Destroy hardware resources before destroying MMU. */
        gcmkVERIFY_OK(gckHARDWARE_PreDestroy(Kernel->hardware));

        if (Kernel->mmu)
        {
#if gcdSHARED_PAGETABLE
            /* Destroy the gckMMU object. */
            gcmkVERIFY_OK(gckMMU_Destroy(Kernel->mmu));
#else
            if (Kernel->mmu->hardware == Kernel->hardware)
            {
                /* Destroy the gckMMU object. */
                gcmkVERIFY_OK(gckMMU_Destroy(Kernel->mmu));
            }
#endif
        }

        /* Destroy the gckHARDWARE object. */
        gcmkVERIFY_OK(gckHARDWARE_Destroy(Kernel->hardware));
    }

    if (Kernel->atomClients)
    {
        /* Detsroy the client atom. */
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, Kernel->atomClients));
    }

    if (Kernel->atomBroCoreMask)
    {
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, Kernel->atomBroCoreMask));
    }

    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->vidMemBlockMutex));

    /* Destroy the database. */
    if (Kernel->dbCreated)
    {
        for (i = 0; i < gcmCOUNTOF(Kernel->db->db); ++i)
        {
            if (Kernel->db->db[i] != gcvNULL)
            {
                gcmkVERIFY_OK(
                    gckKERNEL_DestroyProcessDB(Kernel, Kernel->db->db[i]->processID));
            }
        }

        /* Free all databases. */
        for (database = Kernel->db->freeDatabase;
             database != gcvNULL;
             database = databaseNext)
        {
            databaseNext = database->next;

            if (database->counterMutex)
            {
                gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, database->counterMutex));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, database));
        }

        if (Kernel->db->lastDatabase != gcvNULL)
        {
            if (Kernel->db->lastDatabase->counterMutex)
            {
                gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->lastDatabase->counterMutex));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel->db->lastDatabase));
        }

        /* Free all database records. */
        for (record = Kernel->db->freeRecord; record != gcvNULL; record = recordNext)
        {
            recordNext = record->next;
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, record));
        }

        if (Kernel->db->dbMutex)
        {
            /* Destroy the database mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->dbMutex));
        }

        if (Kernel->db->nameDatabase)
        {
            /* Destroy video memory name database. */
            gcmkVERIFY_OK(gckKERNEL_DestroyIntegerDatabase(Kernel, Kernel->db->nameDatabase));
        }

        if (Kernel->db->nameDatabaseMutex)
        {
            /* Destroy video memory name database mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->nameDatabaseMutex));
        }

        if (Kernel->db->pointerDatabase)
        {
            /* Destroy id-pointer database. */
            gcmkVERIFY_OK(gckKERNEL_DestroyIntegerDatabase(Kernel, Kernel->db->pointerDatabase));
        }

        if (Kernel->db->videoMemListMutex)
        {
            /* Destroy video memory list mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->videoMemListMutex));
        }

        /* Destroy the database. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel->db));

        /* Notify stuck timer to quit. */
        Kernel->monitorTimerStop = gcvTRUE;
    }

#if gcdDVFS
    if (Kernel->dvfs)
    {
        gcmkVERIFY_OK(gckDVFS_Stop(Kernel->dvfs));
        gcmkVERIFY_OK(gckDVFS_Destroy(Kernel->dvfs));
    }
#endif

#if gcdLINUX_SYNC_FILE
    if (Kernel->timeline)
    {
        gcmkVERIFY_OK(gckOS_DestroySyncTimeline(Kernel->os, Kernel->timeline));
    }
#endif


    /* Mark the gckKERNEL object as unknown. */
    Kernel->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckKERNEL object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckKERNEL_AllocateVideoMemory
**
**  Walk requested pools to allocate video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      gckVIDMEM_NODE * NodeObject
**          Pointer to a variable receiving video memory represetation.
*/
gceSTATUS
gckKERNEL_AllocateVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 Alignment,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    IN OUT gcePOOL * Pool,
    OUT gckVIDMEM_NODE * NodeObject
    )
{
    gceSTATUS status;
    gcePOOL pool;
    gckVIDMEM videoMemory;
    gctINT loopCount;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctBOOL contiguous = gcvFALSE;
    gctBOOL cacheable = gcvFALSE;
    gctBOOL secure = gcvFALSE;
    gctBOOL fastPools = gcvFALSE;
#if gcdENABLE_GPU_1M_PAGE
    gctBOOL virtualPool4K = gcvFALSE;
#endif
    gctBOOL hasFastPools = gcvFALSE;
    gctSIZE_T bytes = *Bytes;

    gcmkHEADER_ARG("Kernel=%p *Pool=%d *Bytes=%lu Alignment=%lu Type=%d",
                   Kernel, *Pool, *Bytes, Alignment, Type);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    *NodeObject = gcvNULL;

    /* Check flags. */
    contiguous = Flag & gcvALLOC_FLAG_CONTIGUOUS;
    cacheable  = Flag & gcvALLOC_FLAG_CACHEABLE;
    secure     = Flag & gcvALLOC_FLAG_SECURITY;

    gcmkASSERT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU)
    {
        contiguous = gcvTRUE;
    }

    if (Flag & gcvALLOC_FLAG_FAST_POOLS)
    {
        fastPools = gcvTRUE;
        Flag &= ~gcvALLOC_FLAG_FAST_POOLS;
    }

    if (Flag & gcvALLOC_FLAG_4K_PAGES)
    {
#if gcdENABLE_GPU_1M_PAGE
        virtualPool4K = gcvTRUE;
#endif
        Flag &= ~gcvALLOC_FLAG_4K_PAGES;
    }

#if gcdALLOC_ON_FAULT
    if (Type == gcvVIDMEM_COLOR_BUFFER)
    {
        Flag |= gcvALLOC_FLAG_ALLOC_ON_FAULT;
    }
#endif

    if (Flag & gcvALLOC_FLAG_ALLOC_ON_FAULT)
    {
        *Pool = gcvPOOL_VIRTUAL;
    }

#ifdef __QNXNTO__
    if (Flag & gcvALLOC_FLAG_4GB_ADDR) {
        /* Use the Virtual pool, since the system pool may be allocated above 4G limit */
        *Pool = gcvPOOL_VIRTUAL;
    }
#endif
    if (Flag & gcvALLOC_FLAG_DMABUF_EXPORTABLE)
    {
        gctSIZE_T pageSize = 0;
        gckOS_GetPageSize(Kernel->os, &pageSize);

        /* Usually, the exported dmabuf might be later imported to DRM,
        ** while DRM requires input size to be page aligned.
        */
        bytes = gcmALIGN(bytes, pageSize);
    }

    if (Type == gcvVIDMEM_TYPE_COMMAND)
    {
#if gcdALLOC_CMD_FROM_RESERVE || gcdDISABLE_GPU_VIRTUAL_ADDRESS || !USE_KERNEL_VIRTUAL_BUFFERS
        Flag |= gcvALLOC_FLAG_CONTIGUOUS;
#endif
    }


    if (Type == gcvVIDMEM_TYPE_TILE_STATUS)
    {
        gctBOOL tileStatusInVirtual;

        {
            tileStatusInVirtual =
                gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_MC20);
        }

        if (!tileStatusInVirtual)
        {
            /* Must be contiguous if not support virtual tile status. */
            Flag |= gcvALLOC_FLAG_CONTIGUOUS;
        }
    }


AllocateMemory:

#if gcdCAPTURE_ONLY_MODE
    if (*Pool != gcvPOOL_VIRTUAL)
    {
        *Pool = gcvPOOL_SYSTEM;
    }
#endif

    /* Get initial pool. */
    switch (pool = *Pool)
    {
    case gcvPOOL_DEFAULT:
    case gcvPOOL_LOCAL:
        pool      = gcvPOOL_LOCAL_INTERNAL;
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_UNIFIED:
#if USE_LINUX_PCIE
        pool      = gcvPOOL_LOCAL;
#else
        pool      = gcvPOOL_SYSTEM;
#endif
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    default:
        loopCount = 1;
        break;
    }

    while (loopCount-- > 0)
    {
        if (pool == gcvPOOL_VIRTUAL)
        {
            /* Try contiguous virtual first. */
#if gcdCONTIGUOUS_SIZE_LIMIT
            if (bytes > gcdCONTIGUOUS_SIZE_LIMIT && contiguous == gcvFALSE)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
            }
            else
#endif
#if gcdENABLE_GPU_1M_PAGE
            if (!virtualPool4K && Kernel->core != gcvCORE_VG && Kernel->hardware->mmuVersion)
            {
                /* Create a gckVIDMEM_NODE from contiguous memory. */
                status = gckVIDMEM_NODE_AllocateVirtualChunk(
                            Kernel,
                            pool,
                            Type,
                            Flag | gcvALLOC_FLAG_CONTIGUOUS,
                            &bytes,
                            &nodeObject);

                if (gcmIS_SUCCESS(status))
                {
                    /* Memory allocated. */
                    break;
                }
            }
#endif
            {
                /* Create a gckVIDMEM_NODE from contiguous memory. */
                status = gckVIDMEM_NODE_AllocateVirtual(
                            Kernel,
                            pool,
                            Type,
                            Flag | gcvALLOC_FLAG_CONTIGUOUS,
                            &bytes,
                            &nodeObject);
            }

            if (gcmIS_SUCCESS(status))
            {
                /* Memory allocated. */
                break;
            }

            if (contiguous)
            {
                break;
            }

#if gcdENABLE_GPU_1M_PAGE
            /* Try non-contiguous virtual chunk. */
            if (!virtualPool4K && Kernel->hardware->mmuVersion && Kernel->core != gcvCORE_VG)
            {
                /* Create a gckVIDMEM_NODE from contiguous memory. */
                status = gckVIDMEM_NODE_AllocateVirtualChunk(
                            Kernel,
                            pool,
                            Type,
                            Flag | gcvALLOC_FLAG_NON_CONTIGUOUS,
                            &bytes,
                            &nodeObject);

                if (gcmIS_SUCCESS(status))
                {
                    /* Memory allocated. */
                    break;
                }
            }
#endif
            /* Try non-contiguous virtual. */
            /* Create a gckVIDMEM_NODE for virtual memory. */
            gcmkONERROR(
                gckVIDMEM_NODE_AllocateVirtual(Kernel,
                                               pool,
                                               Type,
                                               Flag | gcvALLOC_FLAG_NON_CONTIGUOUS,
                                               &bytes, &nodeObject));

            /* Success. */
            break;
        }
        /* gcvPOOL_SYSTEM/gcvPOOL_SRAM can't be cacheable. */
        else if (cacheable == gcvFALSE && secure == gcvFALSE)
        {
#ifdef EMULATOR
            /* Cmodel only support 1 SRAM currently. */
            Kernel->sRAMIndex = 0;
            Kernel->extSRAMIndex = 0;
#endif

            /* Get pointer to gckVIDMEM object for pool. */
            status = gckKERNEL_GetVideoMemoryPool(Kernel, pool, &videoMemory);

            if (gcmIS_SUCCESS(status))
            {
                /* Allocate memory. */
                if ((Flag & videoMemory->capability) != Flag)
                {
                    status = gcvSTATUS_NOT_SUPPORTED;
                }
#if defined(gcdLINEAR_SIZE_LIMIT)
                /* 512 KB */
                else if (bytes > gcdLINEAR_SIZE_LIMIT)
                {
                    status = gcvSTATUS_OUT_OF_MEMORY;
                }
#endif
                else
                {
                    hasFastPools = gcvTRUE;
                    status = gckVIDMEM_NODE_AllocateLinear(Kernel,
                                                           videoMemory,
                                                           pool,
                                                           Type,
                                                           Flag,
                                                           Alignment,
                                                           (pool == gcvPOOL_SYSTEM ||
                                                            pool == gcvPOOL_INTERNAL_SRAM ||
                                                            pool == gcvPOOL_EXTERNAL_SRAM),
                                                           &bytes,
                                                           &nodeObject);
                }

                if (gcmIS_SUCCESS(status))
                {
                    /* Memory allocated. */
                    break;
                }
#if gcdCAPTURE_ONLY_MODE
                else
                {
                    gcmkPRINT("Capture only mode: Out of Memory");
                }
#endif

            }
        }

        if (pool == gcvPOOL_LOCAL)
        {
            pool = gcvPOOL_LOCAL_INTERNAL;
        }
        else if (pool == gcvPOOL_LOCAL_INTERNAL)
        {
            /* Get pointer to gckVIDMEM object for pool. */
            status = gckKERNEL_GetVideoMemoryPool(Kernel, gcvPOOL_LOCAL_EXCLUSIVE, &videoMemory);
            if (gcmIS_ERROR(status))
            {
                /* Advance to external memory. */
                pool = gcvPOOL_LOCAL_EXTERNAL;
            }
            else
            {
                status = gckKERNEL_GetVideoMemoryPool(Kernel, gcvPOOL_LOCAL_EXTERNAL, &videoMemory);
                if (gcmIS_SUCCESS(status) &&
                    (videoMemory->freeBytes < videoMemory->bytes / 3) &&
                    Type != gcvVIDMEM_TYPE_BITMAP)
                {
                    pool = gcvPOOL_LOCAL_EXCLUSIVE;
                }
                else
                {
                    pool = gcvPOOL_LOCAL_EXTERNAL;
                }
            }
        }
        else if (pool == gcvPOOL_LOCAL_EXTERNAL)
        {
            pool = gcvPOOL_LOCAL_EXCLUSIVE;
        }
        else if (pool == gcvPOOL_LOCAL_EXCLUSIVE)
        {
            if (Kernel->sRAMLoopMode)
            {
                /* Advance to Internal SRAM memory block. */
                pool = gcvPOOL_INTERNAL_SRAM;
            }
            else
            {
                /* Advance to contiguous reserved memory. */
                pool = gcvPOOL_SYSTEM;
            }
        }
        else if (pool == gcvPOOL_INTERNAL_SRAM)
        {
            if (Kernel->sRAMIndex < gcvSRAM_INTER_COUNT - 1 && !Kernel->sRAMPhysFaked[Kernel->sRAMIndex])
            {
                Kernel->sRAMIndex++;
                loopCount++;
            }
            else
            {
                /* Advance to contiguous reserved memory. */
                pool = gcvPOOL_SYSTEM;
            }
        }
        else if (pool == gcvPOOL_SYSTEM)
        {
            /* Do not go ahead to try relative slow pools */
            if (fastPools && hasFastPools)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
                break;
            }

            /* Advance to virtual memory. */
            pool = gcvPOOL_VIRTUAL;
        }
        else
        {
            /* Out of pools. */
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }

    if (nodeObject == gcvNULL)
    {
        if (contiguous)
        {
            /* Broadcast OOM message. */
            status = gckOS_Broadcast(Kernel->os, Kernel->hardware, gcvBROADCAST_OUT_OF_MEMORY);

            if (gcmIS_SUCCESS(status))
            {
                /* Get some memory. */
                gckOS_Delay(gcvNULL, 1);
                goto AllocateMemory;
            }
        }

        /* Nothing allocated. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

#if gcdCAPTURE_ONLY_MODE
    nodeObject->captureSize = bytes;
#endif

    /* Return node and pool used for allocation. */
    *Pool  = pool;
    *Bytes = bytes;
    *NodeObject  = nodeObject;

    /* Return status. */
    gcmkFOOTER_ARG("*Pool=%d *NodeObject=%p", *Pool, *NodeObject);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}


/*******************************************************************************
**
**  _AllocateLinearMemory
**
**  Private function to allocate the requested amount of video memory, output
**  video memory handle.
*/
gceSTATUS
_AllocateLinearMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctUINT32 handle = 0;
    gceDATABASE_TYPE dbType;
    gcePOOL pool = (gcePOOL)Interface->u.AllocateLinearVideoMemory.pool;
    gctSIZE_T bytes = (gctSIZE_T)Interface->u.AllocateLinearVideoMemory.bytes;
    gctUINT32 alignment = Interface->u.AllocateLinearVideoMemory.alignment;
    gceVIDMEM_TYPE type = (Interface->u.AllocateLinearVideoMemory.type & 0xFF);
    gctUINT32 flag = Interface->u.AllocateLinearVideoMemory.flag;
    gctUINT64 mappingInOne  = 1;
    gctBOOL isContiguous;

    gcmkHEADER_ARG("Kernel=%p pool=%d bytes=%lu alignment=%lu type=%d",
                   Kernel, pool, bytes, alignment, type);

    gcmkVERIFY_ARGUMENT(bytes != 0);

    if (Interface->u.AllocateLinearVideoMemory.sRAMIndex >= 0)
    {
        Kernel->sRAMIndex = Interface->u.AllocateLinearVideoMemory.sRAMIndex;
    }

    if (Interface->u.AllocateLinearVideoMemory.extSRAMIndex >= 0)
    {
        Kernel->extSRAMIndex = Interface->u.AllocateLinearVideoMemory.extSRAMIndex;
    }

    gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne);
    if (mappingInOne == 0)
    {
        alignment = gcmALIGN(alignment, 4096);
    }

    /* Allocate video memory node. */
    gcmkONERROR(
        gckKERNEL_AllocateVideoMemory(Kernel,
                                      alignment,
                                      type,
                                      flag,
                                      &bytes,
                                      &pool,
                                      &nodeObject));

    /* Allocate handle for this video memory. */
    gcmkONERROR(
        gckVIDMEM_HANDLE_Allocate(Kernel, nodeObject, &handle));

    /* Return node and pool used for allocation. */
    Interface->u.AllocateLinearVideoMemory.node = handle;
    Interface->u.AllocateLinearVideoMemory.pool = pool;
    Interface->u.AllocateLinearVideoMemory.bytes = bytes;

    /* Encode surface type and pool to database type. */
    dbType = gcvDB_VIDEO_MEMORY
           | (type << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
           | (pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    /* Record in process db. */
    gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   ProcessID,
                                   dbType,
                                   gcmINT2PTR(handle),
                                   gcvNULL,
                                   bytes));

    gcmkONERROR(gckVIDMEM_NODE_IsContiguous(Kernel, nodeObject, &isContiguous));

    if (isContiguous)
    {
        /* Record in process db. */
        gcmkONERROR(
                gckKERNEL_AddProcessDB(Kernel,
                                       ProcessID,
                                       gcvDB_CONTIGUOUS,
                                       gcmINT2PTR(handle),
                                       gcvNULL,
                                       bytes));
    }

    if (type & gcvVIDMEM_TYPE_COMMAND)
    {
        /* Record in process db. */
        gcmkONERROR(
                gckKERNEL_AddProcessDB(Kernel,
                                       ProcessID,
                                       gcvDB_COMMAND_BUFFER,
                                       gcmINT2PTR(handle),
                                       gcvNULL,
                                       bytes));
    }

    /* Return status. */
    gcmkFOOTER_ARG("pool=%d node=0x%x", pool, handle);
    return gcvSTATUS_OK;

OnError:
    if (handle)
    {
        /* Destroy handle allocated. */
        gcmkVERIFY_OK(gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, handle));
    }

    if (nodeObject)
    {
        /* Free video memory allocated. */
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, nodeObject));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  _ReleaseVideoMemory
**
**  Release handle of a video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**      gctUINT32 Handle
**          Handle of video memory.
**
**  OUTPUT:
**
**          Nothing.
*/
gceSTATUS
_ReleaseVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject;
    gceDATABASE_TYPE type;
    gctBOOL isContiguous;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%d Handle=%d",
                   Kernel, ProcessID, Handle);

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, Handle, &nodeObject));

    type = gcvDB_VIDEO_MEMORY
         | (nodeObject->type << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
         | (nodeObject->pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    gcmkONERROR(
        gckKERNEL_RemoveProcessDB(Kernel,
            ProcessID,
            type,
            gcmINT2PTR(Handle)));

    gcmkONERROR(gckVIDMEM_NODE_IsContiguous(Kernel, nodeObject, &isContiguous));

    if (isContiguous)
    {
        gckKERNEL_RemoveProcessDB(Kernel,
            ProcessID,
            gcvDB_CONTIGUOUS,
            gcmINT2PTR(Handle));
    }

    if (nodeObject->type & gcvVIDMEM_TYPE_COMMAND)
    {
        gckKERNEL_RemoveProcessDB(Kernel,
            ProcessID,
            gcvDB_COMMAND_BUFFER,
            gcmINT2PTR(Handle));
    }

    gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, Handle);

    gckVIDMEM_NODE_Dereference(Kernel, nodeObject);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  _LockVideoMemory
**
**      Lock a video memory node. It will generate a cpu virtual address used
**      by software and a GPU address used by GPU.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gceCORE Core
**          GPU to which video memory is locked.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
static gceSTATUS
_LockVideoMemory(
    IN gckKERNEL Kernel,
    IN gceCORE Core,
    IN gctUINT32 ProcessID,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gctUINT32 handle;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctBOOL referenced = gcvFALSE;
    gctUINT32 address = gcvINVALID_ADDRESS;
    gctPOINTER logical = gcvNULL;
    gctPHYS_ADDR_T physical = gcvINVALID_PHYSICAL_ADDRESS;
    gctUINT32 gid = 0;
    gctBOOL asynchronous = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%d",
                   Kernel, ProcessID);

    handle = Interface->u.LockVideoMemory.node;

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, handle, &nodeObject));

    if (Interface->u.LockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_LOCK)
    {
        /* Ref node. */
        gcmkONERROR(gckVIDMEM_NODE_Reference(Kernel, nodeObject));
        referenced = gcvTRUE;
    }

#if gcdCAPTURE_ONLY_MODE
    if (Interface->u.LockVideoMemory.queryCapSize)
    {
        Interface->u.LockVideoMemory.captureSize = nodeObject->captureSize;
        return gcvSTATUS_OK;
    }
    else
    {
        nodeObject->captureLogical = Interface->u.LockVideoMemory.captureLogical;
    }
#endif

    if (Interface->u.LockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_LOCK)
    {
        /* Lock for GPU address. */
        gcmkONERROR(gckVIDMEM_NODE_Lock(Kernel, nodeObject, &address));

        /* Get CPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(Kernel, nodeObject, 0, &physical));
        gcmkONERROR(gckVIDMEM_NODE_GetGid(Kernel, nodeObject, &gid));

        Interface->u.LockVideoMemory.address = address;
        Interface->u.LockVideoMemory.physicalAddress = physical;
        Interface->u.LockVideoMemory.gid = gid;
        Interface->u.LockVideoMemory.memory = 0;
    }

    if (Interface->u.LockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_MAP)
    {
        /* Lock for userspace CPU userspace. */
        gcmkONERROR(
            gckVIDMEM_NODE_LockCPU(Kernel,
                                   nodeObject,
                                   Interface->u.LockVideoMemory.cacheable,
                                   gcvTRUE,
                                   &logical));

        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(logical);
    }

    if (Interface->u.LockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_LOCK)
    {
        gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   ProcessID,
                                   gcvDB_VIDEO_MEMORY_LOCKED,
                                   gcmINT2PTR(handle),
                                   logical,
                                   0));

        /* Ref handle. */
        gckVIDMEM_HANDLE_Reference(Kernel, ProcessID, handle);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (logical)
    {
        gckVIDMEM_NODE_UnlockCPU(Kernel, nodeObject, ProcessID, gcvTRUE, gcvFALSE);
    }

    if (address != gcvINVALID_ADDRESS)
    {
        gckVIDMEM_NODE_Unlock(Kernel, nodeObject, ProcessID, &asynchronous);

        if (asynchronous)
        {
            gckVIDMEM_NODE_Unlock(Kernel, nodeObject, ProcessID, gcvNULL);
        }
    }

    if (referenced)
    {
        gckVIDMEM_NODE_Dereference(Kernel, nodeObject);
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  _UnlockVideoMemory
**
**      Unlock a video memory node.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
static gceSTATUS
_UnlockVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK vidMemBlock = gcvNULL;
    gctSIZE_T bytes;
    gctUINT64 mappingInOne = 1;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%d",
                   Kernel, ProcessID);

    Interface->u.UnlockVideoMemory.pool = gcvPOOL_UNKNOWN;
    Interface->u.UnlockVideoMemory.bytes = 0;

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Kernel,
        ProcessID,
        (gctUINT32)Interface->u.UnlockVideoMemory.node,
        &nodeObject
        ));

    if (Interface->u.UnlockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_UNMAP)
    {
        gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne);
        /* Unlock CPU. */
        gcmkONERROR(gckVIDMEM_NODE_UnlockCPU(
            Kernel, nodeObject, ProcessID, gcvTRUE, mappingInOne == 1));
    }

    if (Interface->u.UnlockVideoMemory.op & gcvLOCK_VIDEO_MEMORY_OP_UNLOCK)
    {
        /* Unlock video memory. */
        gcmkONERROR(gckVIDMEM_NODE_Unlock(
            Kernel,
            nodeObject,
            ProcessID,
            &Interface->u.UnlockVideoMemory.asynchroneous
            ));

        /* Leave deref handle and deref node in later operation. */

        node = nodeObject->node;

        vidMemBlock = node->VirtualChunk.parent;

        if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        {
            bytes = node->VidMem.bytes;
        }
        else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        {
            bytes = node->VirtualChunk.bytes;
        }
        else
        {
            bytes = node->Virtual.bytes;
        }

        Interface->u.UnlockVideoMemory.pool  = nodeObject->pool;
        Interface->u.UnlockVideoMemory.bytes = bytes;
    }

#if gcdCAPTURE_ONLY_MODE
    Interface->u.UnlockVideoMemory.captureLogical = nodeObject->captureLogical;
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /*
     * Unlikely to fail expect error node or unlocked, there's no error roll
     * back requried for those two conditions.
     */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  _BottomHalfUnlockVideoMemory
**
**  Unlock video memory from gpu.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          Process ID owning this memory.
**
**      gceVIDMEM_TYPE
**          Video memory allocation type.
**
**      gctPOINTER Pointer
**          Video memory to be unlock.
*/
static gceSTATUS
_BottomHalfUnlockVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Node
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;

    /* Remove record from process db. */
    gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
        Kernel,
        ProcessID,
        gcvDB_VIDEO_MEMORY_LOCKED,
        gcmINT2PTR(Node)
        ));

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Kernel,
        ProcessID,
        Node,
        &nodeObject
        ));

    /* Deref handle. */
    gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, Node);

    /* Unlock video memory, synced. */
    gcmkONERROR(gckVIDMEM_NODE_Unlock(Kernel, nodeObject, ProcessID, gcvNULL));

    /* Deref node. */
    gcmkONERROR(gckVIDMEM_NODE_Dereference(Kernel, nodeObject));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_WrapUserMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gceDATABASE_TYPE type;
    gctUINT32 handle = 0;
    gctBOOL isContiguous;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%x", Kernel, ProcessID);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    gcmkASSERT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    gcmkONERROR(
        gckVIDMEM_NODE_WrapUserMemory(Kernel,
                                      &Interface->u.WrapUserMemory.desc,
                                      Interface->u.WrapUserMemory.type,
                                      &nodeObject,
                                      &Interface->u.WrapUserMemory.bytes));

    /* Create handle representation for userspace. */
    gcmkONERROR(
        gckVIDMEM_HANDLE_Allocate(Kernel,
                                  nodeObject,
                                  &handle));

    type = gcvDB_VIDEO_MEMORY
         | (nodeObject->type << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
         | (nodeObject->pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    gcmkONERROR(
        gckKERNEL_AddProcessDB(Kernel,
                               ProcessID,
                               type,
                               gcmINT2PTR(handle),
                               gcvNULL,
                               (gctSIZE_T)Interface->u.WrapUserMemory.bytes));

    gcmkONERROR(gckVIDMEM_NODE_IsContiguous(Kernel, nodeObject, &isContiguous));

    if (isContiguous)
    {
        /* Record in process db. */
        gcmkONERROR(
                gckKERNEL_AddProcessDB(Kernel,
                                       ProcessID,
                                       gcvDB_CONTIGUOUS,
                                       gcmINT2PTR(handle),
                                       gcvNULL,
                                       (gctSIZE_T)Interface->u.WrapUserMemory.bytes));
    }

    Interface->u.WrapUserMemory.node = handle;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (handle)
    {
        gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, handle);
    }

    if (nodeObject)
    {
        gckVIDMEM_NODE_Dereference(Kernel, nodeObject);
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_ExportVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel,
                                ProcessID,
                                Interface->u.ExportVideoMemory.node,
                                &nodeObject));

    gcmkONERROR(
        gckVIDMEM_NODE_Export(Kernel,
                              nodeObject,
                              Interface->u.ExportVideoMemory.flags,
                              gcvNULL,
                              &Interface->u.ExportVideoMemory.fd));

OnError:
    return status;
}

static gceSTATUS
_NameVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel,
                                ProcessID,
                                Interface->u.NameVideoMemory.handle,
                                &nodeObject));

    gcmkONERROR(
        gckVIDMEM_NODE_Name(Kernel,
                            nodeObject,
                            &Interface->u.NameVideoMemory.name));
OnError:
    return status;
}

static gceSTATUS
_ImportVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctUINT32 handle = 0;

    gcmkONERROR(
        gckVIDMEM_NODE_Import(Kernel,
                              Interface->u.ImportVideoMemory.name,
                              &nodeObject));

    /* Create handle representation for userspace. */
    gcmkONERROR(
        gckVIDMEM_HANDLE_Allocate(Kernel,
                                  nodeObject,
                                  &handle));

    gcmkONERROR(
        gckKERNEL_AddProcessDB(Kernel,
                               ProcessID,
                               gcvDB_VIDEO_MEMORY,
                               gcmINT2PTR(handle),
                               gcvNULL,
                               0));

    Interface->u.ImportVideoMemory.handle = handle;
    return gcvSTATUS_OK;

OnError:
    if (handle)
    {
        gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, handle);
    }

    if (nodeObject)
    {
        gckVIDMEM_NODE_Dereference(Kernel, nodeObject);
    }

    return status;
}

/*******************************************************************************
**
**  gckKERNEL_SetVidMemMetadata
**
**  Set/Get metadata to/from gckVIDMEM_NODE object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**  INOUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a interface structure
*/
#if defined(CONFIG_DMA_SHARED_BUFFER)
#include <linux/dma-buf.h>

gceSTATUS
_SetVidMemMetadata(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    INOUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckVIDMEM_NODE nodeObj = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%d", Kernel, ProcessID);

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, Interface->u.SetVidMemMetadata.node, &nodeObj));

    if (Interface->u.SetVidMemMetadata.readback)
    {
        Interface->u.SetVidMemMetadata.ts_fd            = nodeObj->metadata.ts_fd;
        Interface->u.SetVidMemMetadata.fc_enabled       = nodeObj->metadata.fc_enabled;
        Interface->u.SetVidMemMetadata.fc_value         = nodeObj->metadata.fc_value;
        Interface->u.SetVidMemMetadata.fc_value_upper   = nodeObj->metadata.fc_value_upper;
        Interface->u.SetVidMemMetadata.compressed       = nodeObj->metadata.compressed;
        Interface->u.SetVidMemMetadata.compress_format  = nodeObj->metadata.compress_format;
    }
    else
    {
        if ((nodeObj->metadata.ts_fd >= 0) && (nodeObj->metadata.ts_dma_buf != NULL)
            && !(IS_ERR(nodeObj->metadata.ts_dma_buf))) {
            dma_buf_put(nodeObj->metadata.ts_dma_buf);
        }

        nodeObj->metadata.ts_fd             = Interface->u.SetVidMemMetadata.ts_fd;

        if (nodeObj->metadata.ts_fd >= 0)
        {
            nodeObj->metadata.ts_dma_buf    = dma_buf_get(nodeObj->metadata.ts_fd);

            if (IS_ERR(nodeObj->metadata.ts_dma_buf))
            {
                gcmkONERROR(gcvSTATUS_NOT_FOUND);
            }
        }
        else
        {
            nodeObj->metadata.ts_dma_buf    = NULL;
        }

        nodeObj->metadata.fc_enabled        = Interface->u.SetVidMemMetadata.fc_enabled;
        nodeObj->metadata.fc_value          = Interface->u.SetVidMemMetadata.fc_value;
        nodeObj->metadata.fc_value_upper    = Interface->u.SetVidMemMetadata.fc_value_upper;
        nodeObj->metadata.compressed        = Interface->u.SetVidMemMetadata.compressed;
        nodeObj->metadata.compress_format   = Interface->u.SetVidMemMetadata.compress_format;
    }

OnError:
    gcmkFOOTER();
    return status;
}

#else

gceSTATUS
_SetVidMemMetadata(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    INOUT gcsHAL_INTERFACE * Interface
    )
{
    gcmkFATAL("The kernel did NOT support CONFIG_DMA_SHARED_BUFFER");
    return gcvSTATUS_NOT_SUPPORTED;
}
#endif

static gceSTATUS
_GetVideoMemoryFd(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel,
                                ProcessID,
                                Interface->u.GetVideoMemoryFd.handle,
                                &nodeObject));

    gcmkONERROR(
        gckVIDMEM_NODE_GetFd(Kernel,
                             nodeObject,
                             &Interface->u.GetVideoMemoryFd.fd));

    /* No need to add it to processDB because OS will release all fds when
    ** process quits.
    */
OnError:
    return status;
}

gceSTATUS
gckKERNEL_QueryDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gctINT i;

    gceDATABASE_TYPE type[2] = {
        gcvDB_VIDEO_MEMORY | (gcvPOOL_SYSTEM << gcdDB_VIDEO_MEMORY_POOL_SHIFT),
        gcvDB_VIDEO_MEMORY | (gcvPOOL_VIRTUAL << gcdDB_VIDEO_MEMORY_POOL_SHIFT),
    };

    gcmkHEADER();

    /* Query video memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_VIDEO_MEMORY,
                                 &Interface->u.Database.vidMem));

    /* Query non-paged memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_NON_PAGED,
                                 &Interface->u.Database.nonPaged));

    /* Query contiguous memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_CONTIGUOUS,
                                 &Interface->u.Database.contiguous));

    /* Query GPU idle time. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_IDLE,
                                 &Interface->u.Database.gpuIdle));
    for (i = 0; i < 2; i++)
    {
        /* Query each video memory pool. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     type[i],
                                     &Interface->u.Database.vidMemPool[i]));
    }

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gckKERNEL_DumpVidMemUsage(Kernel, Interface->u.Database.processID);
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_ConfigPowerManagement(
    IN gckKERNEL Kernel,
    IN OUT gcsHAL_INTERFACE * Interface
)
{
    gceSTATUS status;
    gctBOOL enable = Interface->u.ConfigPowerManagement.enable;

    gcmkHEADER();

    gcmkONERROR(gckHARDWARE_QueryPowerManagement(Kernel->hardware, &Interface->u.ConfigPowerManagement.oldValue));

    gcmkONERROR(gckHARDWARE_EnablePowerManagement(Kernel->hardware, enable));

    if (enable == gcvFALSE)
    {
        gcmkONERROR(
            gckHARDWARE_SetPowerState(Kernel->hardware, gcvPOWER_ON));
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}


static gceSTATUS
gckKERNEL_CacheOperation(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Node,
    IN gceCACHEOPERATION Operation,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gcuVIDMEM_NODE_PTR node = gcvNULL;
    gckVIDMEM_BLOCK vidMemBlock = gcvNULL;
    gctSIZE_T offset = 0;
    void *memHandle;
    gceSYNC_VIDEO_MEMORY_REASON reason = gcvSYNC_REASON_NONE;

    gcmkHEADER_ARG("Kernel=%p pid=%u Node=%u op=%d Logical=%p Bytes=0x%lx",
                   Kernel, ProcessID, Node, Operation, Logical, Bytes);

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(Kernel,
                                        ProcessID,
                                        Node,
                                        &nodeObject));
    if (nodeObject->pool == gcvPOOL_LOCAL_EXCLUSIVE)
    {
        node = nodeObject->transitNode;
    }
    else
    {
        node = nodeObject->node;
    }

    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
    {
        static gctBOOL printed;

        if (!printed)
        {
            printed = gcvTRUE;
            gcmkPRINT("[galcore]: %s: Flush Video Memory", __FUNCTION__);
        }

        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
    {
        memHandle = vidMemBlock->physical;
        offset = node->VirtualChunk.offset;
    }
    else
    {
        memHandle = node->Virtual.physical;
    }

    switch (Operation)
    {
    case gcvCACHE_FLUSH:
        /* Clean and invalidate the cache. */
        status = gckOS_CacheFlush(Kernel->os,
                                  ProcessID,
                                  memHandle,
                                  offset,
                                  Logical,
                                  Bytes);
        reason = gcvSYNC_REASON_AFTER_WRITE;
        break;
    case gcvCACHE_CLEAN:
        /* Clean the cache. */
        status = gckOS_CacheClean(Kernel->os,
                                  ProcessID,
                                  memHandle,
                                  offset,
                                  Logical,
                                  Bytes);
        reason = gcvSYNC_REASON_AFTER_WRITE;
        break;
    case gcvCACHE_INVALIDATE:
        /* Invalidate the cache. */
        status = gckOS_CacheInvalidate(Kernel->os,
                                       ProcessID,
                                       memHandle,
                                       offset,
                                       Logical,
                                       Bytes);
        reason = gcvSYNC_REASON_BEFORE_READ;
        break;

    case gcvCACHE_MEMORY_BARRIER:
        status = gckOS_MemoryBarrier(Kernel->os, Logical);
        reason = gcvSYNC_REASON_AFTER_WRITE;
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

    if (nodeObject->pool == gcvPOOL_LOCAL_EXCLUSIVE && reason != gcvSYNC_REASON_NONE)
    {
        status = gckKERNEL_SyncVideoMemory(Kernel, nodeObject, reason);
    }
OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_WaitFence(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE node;
    gckCOMMAND command = Kernel->command;
    gckCOMMAND asyncCommand = Kernel->asyncCommand;
    gckFENCE fence = gcvNULL;
    gctUINT i;

    gcmkASSERT(command != gcvNULL);

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel,
                                ProcessID,
                                Interface->u.WaitFence.handle,
                                &node));

    gcmkONERROR(gckVIDMEM_NODE_Reference(Kernel, node));

    /* Wait for fence of all engines. */
    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
    {
        gckFENCE_SYNC sync = &node->sync[i];

        if (i == gcvENGINE_RENDER)
        {
            fence = command->fence;
        }
        else
        {
            fence = asyncCommand->fence;
        }

        gcmkONERROR(gckVIDMEM_NODE_InvalidateCache(
            Kernel,
            fence->videoMem,
            0,
            fence->logical,
            8
            ));

        if (sync->commitStamp <= *(gctUINT64_PTR)fence->logical)
        {
            continue;
        }
        else
        {
            gckOS_Signal(Kernel->os, sync->signal, gcvFALSE);

            gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, &fence->mutex, gcvINFINITE));

            /* Add to waiting list. */
            gcsLIST_AddTail(&sync->head, &fence->waitingList);

            gcmkASSERT(sync->inList == gcvFALSE);

            sync->inList = gcvTRUE;

            gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, &fence->mutex));

            /* Wait. */
            status = gckOS_WaitSignal(
                Kernel->os,
                sync->signal,
                gcvTRUE,
                Interface->u.WaitFence.timeOut
                );

            gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, &fence->mutex, gcvINFINITE));

            if (sync->inList)
            {
                gcsLIST_Del(&sync->head);
                sync->inList = gcvFALSE;
            }

            gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, &fence->mutex));
        }
    }

    gckVIDMEM_NODE_Dereference(Kernel, node);

OnError:
    return status;
}

static gceSTATUS
_Commit(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE HwType,
    IN gceENGINE Engine,
    IN gctUINT32 ProcessId,
    IN gctUINT32 BroCoreMask,
    IN OUT gcsHAL_COMMIT * Commit
    )
{
    gceSTATUS status;
    gcsHAL_SUBCOMMIT *subCommit = &Commit->subCommit;
    gcsHAL_SUBCOMMIT _subCommit;
    gctPOINTER userPtr = gcvNULL;
    gctBOOL needCopy = gcvFALSE;
    gckKERNEL kernel;

    gcmkVERIFY_OK(gckOS_QueryNeedCopy(Device->os, ProcessId, &needCopy));

    do
    {
        gckCOMMAND command;
        gckEVENT eventObj;
        gctUINT64 next;

        /* Skip the first nested sub-commit struct. */
        if (userPtr)
        {
            /* Copy/map sub-commit from user. */
            if (needCopy)
            {
                subCommit = &_subCommit;

                status = gckOS_CopyFromUserData(
                    Device->os,
                    subCommit,
                    userPtr,
                    gcmSIZEOF(gcsHAL_SUBCOMMIT)
                    );
            }
            else
            {
                status = gckOS_MapUserPointer(
                    Device->os,
                    userPtr,
                    gcmSIZEOF(gcsHAL_SUBCOMMIT),
                    (gctPOINTER *)&subCommit
                    );
            }

            if (gcmIS_ERROR(status))
            {
                userPtr = gcvNULL;

                gcmkONERROR(status);
            }
        }

        if (subCommit->coreId >= gcvCORE_COUNT)
        {
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        /* Determine the objects. */
        if (HwType == gcvHARDWARE_3D || HwType == gcvHARDWARE_3D2D || HwType == gcvHARDWARE_VIP)
        {
            kernel = Device->coreInfoArray[subCommit->coreId].kernel;

#if (gcdENABLE_PER_DEVICE_PM == 1)
            gcmkONERROR(gckOS_AtomSet(kernel->os, kernel->atomBroCoreMask, BroCoreMask));
#endif
        }
        else
        {
            kernel = Device->map[HwType].kernels[subCommit->coreId];
        }
        if (Engine == gcvENGINE_BLT)
        {
            command  = kernel->asyncCommand;
            eventObj = kernel->asyncEvent;
        }
        else
        {
            command  = kernel->command;
            eventObj = kernel->eventObj;
        }

        {
#if gcdENABLE_SW_PREEMPTION
            /* Commit command with preemption. */
            gcmkONERROR(
                gckKERNEL_CommandCommitPreemption(kernel,
                                                  Engine,
                                                  ProcessId,
                                                  command,
                                                  eventObj,
                                                  subCommit,
                                                  Commit));
#else
            /* Commit command buffers. */
            status = gckCOMMAND_Commit(command,
                                       subCommit,
                                       ProcessId,
                                       Commit->shared,
                                       &Commit->commitStamp,
                                       &Commit->contextSwitched);

            if (status != gcvSTATUS_INTERRUPTED)
            {
                gcmkONERROR(status);
            }

            /* Commit events. */
            status = gckEVENT_Commit(
                eventObj,
                gcmUINT64_TO_PTR(subCommit->queue),
                kernel->hardware->options.powerManagement
                );

            if (status != gcvSTATUS_INTERRUPTED)
            {
                gcmkONERROR(status);
            }
#endif
        }

        next = subCommit->next;

        /* Unmap user pointer if mapped. */
        if (!needCopy && userPtr)
        {
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Device->os,
                userPtr,
                gcmSIZEOF(gcsHAL_SUBCOMMIT),
                subCommit
                ));
        }

        /* Advance to next sub-commit from user. */
        userPtr = gcmUINT64_TO_PTR(next);
    }
    while (userPtr);

    subCommit = &Commit->subCommit;
    userPtr = gcvNULL;

    if (HwType == gcvHARDWARE_3D || HwType == gcvHARDWARE_3D2D || HwType == gcvHARDWARE_VIP)
    {
        kernel = Device->coreInfoArray[subCommit->coreId].kernel;
    }
    else
    {
        kernel = Device->map[HwType].kernels[subCommit->coreId];
    }

#if VIVANTE_PROFILER
    if (!kernel->profiler.profileEnable || (kernel->profiler.profileMode != gcvPROFILER_AHB_MODE))
    {
        return gcvSTATUS_OK;
    }
#endif

    do
    {
        gctUINT64 next;

        /* Skip the first nested sub-commit struct. */
        if (userPtr)
        {
            /* Copy/map sub-commit from user. */
            if (needCopy)
            {
                subCommit = &_subCommit;

                status = gckOS_CopyFromUserData(
                    Device->os,
                    subCommit,
                    userPtr,
                    gcmSIZEOF(gcsHAL_SUBCOMMIT)
                    );
            }
            else
            {
                status = gckOS_MapUserPointer(
                    Device->os,
                    userPtr,
                    gcmSIZEOF(gcsHAL_SUBCOMMIT),
                    (gctPOINTER *)&subCommit
                    );
            }

            if (gcmIS_ERROR(status))
            {
                userPtr = gcvNULL;

                gcmkONERROR(status);
            }
        }

        if (HwType == gcvHARDWARE_3D || HwType == gcvHARDWARE_3D2D || HwType == gcvHARDWARE_VIP)
        {
            kernel = Device->coreInfoArray[subCommit->coreId].kernel;
        }
        else
        {
            kernel = Device->map[HwType].kernels[subCommit->coreId];
        }

#if VIVANTE_PROFILER
        if ((kernel->profiler.profileEnable == gcvTRUE) &&
            (kernel->profiler.profileMode == gcvPROFILER_AHB_MODE))
        {
            gcmkONERROR(gckCOMMAND_Stall(kernel->command, gcvTRUE));

            gcmkONERROR(gckHARDWARE_UpdateContextProfile(kernel->hardware));
        }
#endif

        next = subCommit->next;

        /* Unmap user pointer if mapped. */
        if (!needCopy && userPtr)
        {
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Device->os,
                userPtr,
                gcmSIZEOF(gcsHAL_SUBCOMMIT),
                subCommit
                ));
        }

        /* Advance to next sub-commit from user. */
        userPtr = gcmUINT64_TO_PTR(next);
    }
    while (userPtr);

    return gcvSTATUS_OK;

OnError:
    if (!needCopy && userPtr)
    {
        gckOS_UnmapUserPointer(
            Device->os,
            userPtr,
            gcmSIZEOF(gcsHAL_SUBCOMMIT),
            subCommit
            );
    }

    return status;
}

#ifdef __linux__
typedef struct _gcsGRRAPHIC_BUFFER_PARCLE
{
    gcsFDPRIVATE base;
    gckKERNEL kernel;

    gckVIDMEM_NODE node[3];
    gctSHBUF  shBuf;
    gctINT32  signal;
}
gcsGRAPHIC_BUFFER_PARCLE;

static void
_ReleaseGraphicBuffer(
    gckKERNEL Kernel,
    gcsGRAPHIC_BUFFER_PARCLE * Parcle
    )
{
    gctUINT i;

    for (i = 0; i < 3; i++)
    {
        if (Parcle->node[i])
        {
            gckVIDMEM_NODE_Dereference(Kernel, Parcle->node[i]);
        }
    }

    if (Parcle->shBuf)
    {
        gckKERNEL_DestroyShBuffer(Kernel, Parcle->shBuf);
    }

    if (Parcle->signal)
    {
        gckOS_DestroyUserSignal(Kernel->os, Parcle->signal);
    }

    gcmkOS_SAFE_FREE(Kernel->os, Parcle);
}

static gctINT
_FdReleaseGraphicBuffer(
    gcsFDPRIVATE_PTR Private
    )
{
    gcsGRAPHIC_BUFFER_PARCLE * parcle = (gcsGRAPHIC_BUFFER_PARCLE *) Private;

    _ReleaseGraphicBuffer(parcle->kernel, parcle);
    return 0;
}

static gceSTATUS
_GetGraphicBufferFd(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Node[3],
    IN gctUINT64 ShBuf,
    IN gctUINT64 Signal,
    OUT gctINT32 * Fd
    )
{
    gceSTATUS status;
    gctUINT i;
    gcsGRAPHIC_BUFFER_PARCLE * parcle = gcvNULL;

    gcmkONERROR(gckOS_Allocate(
        Kernel->os,
        gcmSIZEOF(gcsGRAPHIC_BUFFER_PARCLE),
        (gctPOINTER *)&parcle
        ));

    gckOS_ZeroMemory(parcle, sizeof(gcsGRAPHIC_BUFFER_PARCLE));

    parcle->base.release = _FdReleaseGraphicBuffer;
    parcle->kernel = Kernel;

    for (i = 0; i < 3 && Node[i] != 0; i++)
    {
        gckVIDMEM_NODE nodeObject = gcvNULL;

        gcmkONERROR(
            gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, Node[i], &nodeObject));

        gcmkONERROR(gckVIDMEM_NODE_Reference(Kernel, nodeObject));

        parcle->node[i] = nodeObject;
    }

    if (ShBuf)
    {
        gctSHBUF shBuf = gcmUINT64_TO_PTR(ShBuf);

        gcmkONERROR(gckKERNEL_MapShBuffer(Kernel, shBuf));
        parcle->shBuf = shBuf;
    }

    if (Signal)
    {
        gctSIGNAL signal = gcmUINT64_TO_PTR(Signal);

        gcmkONERROR(
            gckOS_MapSignal(Kernel->os,
                            signal,
                            (gctHANDLE)(gctUINTPTR_T)ProcessID,
                            &signal));

        parcle->signal= (gctINT32)Signal;
    }

    gcmkONERROR(gckOS_GetFd("viv-gr", &parcle->base, Fd));

    return gcvSTATUS_OK;

OnError:
    if (parcle)
    {
        _ReleaseGraphicBuffer(Kernel, parcle);
    }
    return status;
}
#endif

/*******************************************************************************
**
**  gckKERNEL_Dispatch
**
**  Dispatch a command received from the user HAL layer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS
gckKERNEL_Dispatch(
    IN gckKERNEL Kernel,
    IN gckDEVICE Device,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPHYS_ADDR physical = gcvNULL;
    gctSIZE_T bytes;
    gctPOINTER logical = gcvNULL;
    gckCONTEXT context = gcvNULL;
    gckKERNEL kernel = Kernel;
    gctUINT32 processID;
#if !USE_NEW_LINUX_SIGNAL
    gctSIGNAL   signal;
#endif
    gctBOOL powerMutexAcquired = gcvFALSE;
    gctBOOL commitMutexAcquired = gcvFALSE;
    gctBOOL idle = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p Interface=%p", Kernel, Interface);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface != gcvNULL);

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "Dispatching command %d (%s)",
                   Interface->command, _DispatchText[Interface->command]);

    gcmSTATIC_ASSERT(gcvHAL_DESTROY_MMU == gcmCOUNTOF(_DispatchText) - 1,
                     "DispatchText array does not match command codes");
#endif
#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_AcquireMutex(Kernel->os, Kernel->debugMutex, gcvINFINITE);
#endif

    /* Get the current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

    /* Dispatch on command. */
    switch (Interface->command)
    {
    case gcvHAL_GET_BASE_ADDRESS:
        /* Get base address. */
        Interface->u.GetBaseAddress.baseAddress = Kernel->hardware->baseAddress;
        Interface->u.GetBaseAddress.flatMappingRangeCount = Kernel->mmu->gpuPhysicalRangeCount;
        if (Kernel->mmu->gpuPhysicalRangeCount)
        {
            gckOS_MemCopy(Interface->u.GetBaseAddress.flatMappingRanges, Kernel->mmu->gpuPhysicalRanges,
                gcmSIZEOF(gcsFLAT_MAPPING_RANGE) * Kernel->mmu->gpuPhysicalRangeCount);
        }
        break;

    case gcvHAL_QUERY_VIDEO_MEMORY:
        /* Query video memory size. */
        gcmkONERROR(gckKERNEL_QueryVideoMemory(Kernel, Interface));
        break;

    case gcvHAL_QUERY_CHIP_IDENTITY:
        /* Query chip identity. */
        gcmkONERROR(
            gckHARDWARE_QueryChipIdentity(
                Kernel->hardware,
                &Interface->u.QueryChipIdentity));
        break;

    case gcvHAL_QUERY_CHIP_FREQUENCY:
        /* Query chip clock. */
        gcmkONERROR(
            gckHARDWARE_QueryFrequency(Kernel->hardware));

        Interface->u.QueryChipFrequency.mcClk = Kernel->hardware->mcClk;
        Interface->u.QueryChipFrequency.shClk = Kernel->hardware->shClk;
        break;

    case gcvHAL_MAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.MapMemory.physName);

        /* Map memory. */
        gcmkONERROR(
            gckKERNEL_MapMemory(Kernel,
                                physical,
                                (gctSIZE_T) Interface->u.MapMemory.bytes,
                                &logical));

        Interface->u.MapMemory.logical = gcmPTR_TO_UINT64(logical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_MAP_MEMORY,
                                   logical,
                                   physical,
                                   (gctSIZE_T) Interface->u.MapMemory.bytes));
        break;

    case gcvHAL_UNMAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.UnmapMemory.physName);

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_MAP_MEMORY,
                                      gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical)));

        /* Unmap memory. */
        gcmkONERROR(
            gckKERNEL_UnmapMemory(Kernel,
                                  physical,
                                  (gctSIZE_T) Interface->u.UnmapMemory.bytes,
                                  gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical),
                                  processID));
        break;

    case gcvHAL_ALLOCATE_NON_PAGED_MEMORY:
        bytes = (gctSIZE_T) Interface->u.AllocateNonPagedMemory.bytes;

        /* Allocate non-paged memory. */
        gcmkONERROR(
            gckOS_AllocateNonPagedMemory(
                Kernel->os,
                gcvTRUE,
                Interface->u.AllocateNonPagedMemory.flags | gcvALLOC_FLAG_CONTIGUOUS,
                &bytes,
                &physical,
                &logical));

        Interface->u.AllocateNonPagedMemory.bytes    = bytes;
        Interface->u.AllocateNonPagedMemory.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateNonPagedMemory.physName = gcmPTR_TO_NAME(physical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_NON_PAGED,
                                   logical,
                                   gcmINT2PTR(Interface->u.AllocateNonPagedMemory.physName),
                                   bytes));
        break;

    case gcvHAL_FREE_NON_PAGED_MEMORY:
        physical = gcmNAME_TO_PTR(Interface->u.FreeNonPagedMemory.physName);

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_NON_PAGED,
                                      gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        /* Free non-paged memory. */
        gcmkONERROR(
            gckOS_FreeNonPagedMemory(Kernel->os,
                                     physical,
                                     gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical),
                                     (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes));

        gcmRELEASE_NAME(Interface->u.FreeNonPagedMemory.physName);
        break;

    case gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY:
        /* Allocate memory. */
        gcmkONERROR(_AllocateLinearMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_RELEASE_VIDEO_MEMORY:
        /* Release video memory. */
        gcmkONERROR(_ReleaseVideoMemory(
            Kernel, processID,
            (gctUINT32)Interface->u.ReleaseVideoMemory.node
            ));
    break;

    case gcvHAL_LOCK_VIDEO_MEMORY:
        /* Lock video memory. */
        gcmkONERROR(_LockVideoMemory(Kernel, Kernel->core, processID, Interface));
        break;

    case gcvHAL_UNLOCK_VIDEO_MEMORY:
        /* Unlock video memory. */
        gcmkONERROR(_UnlockVideoMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY:
        gcmkERR_BREAK(_BottomHalfUnlockVideoMemory(Kernel, processID,
                                                   Interface->u.BottomHalfUnlockVideoMemory.type,
                                                   Interface->u.BottomHalfUnlockVideoMemory.node));
        break;

    case gcvHAL_EVENT_COMMIT:
        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Kernel->device->commitMutex,
                gcvINFINITE
                ));

            commitMutexAcquired = gcvTRUE;
        }

#if (gcdENABLE_PER_DEVICE_PM == 1)
        gcmkONERROR(gckOS_AtomSet(Kernel->os, Kernel->atomBroCoreMask, Interface->u.Event.broCoreMask));
#endif

#if gcdENABLE_SW_PREEMPTION
        /* Commit event with preemption. */
        gcmkONERROR(
            gckKERNEL_EventCommitPreemption(Kernel,
                                            Interface->engine,
                                            processID,
                                            gcmUINT64_TO_PTR(Interface->u.Event.queue),
                                            Interface->u.Event.priorityID,
                                            Interface->u.Event.topPriority));
#else
        /* Commit an event queue. */
        if (Interface->engine == gcvENGINE_BLT)
        {
            if (!gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_ASYNC_BLIT))
            {
                gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
            }

            gcmkONERROR(gckEVENT_Commit(
                Kernel->asyncEvent, gcmUINT64_TO_PTR(Interface->u.Event.queue), gcvFALSE));
        }
        else
        {
            gcmkONERROR(gckEVENT_Commit(
                Kernel->eventObj, gcmUINT64_TO_PTR(Interface->u.Event.queue), gcvFALSE));
        }
#endif

        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
            commitMutexAcquired = gcvFALSE;
        }

        break;

    case gcvHAL_COMMIT:
        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Device->commitMutex,
                gcvINFINITE
                ));
            commitMutexAcquired = gcvTRUE;
        }

#if gcdENABLE_MP_SWITCH
        gcmkONERROR(gckKERNEL_DetectMpModeSwitch(Kernel, Interface->u.Commit.mpMode, &Interface->u.Commit.switchMpMode));
#endif

        gcmkONERROR(_Commit(Device,
                            Kernel->hardware->type,
                            Interface->engine,
                            processID,
                            Interface->u.Commit.broCoreMask,
                            &Interface->u.Commit
                            ));


        if (!Interface->commitMutex)
        {
            gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Device->commitMutex));
            commitMutexAcquired = gcvFALSE;
        }
        break;

    case gcvHAL_COMMIT_DONE:
#if gcdENABLE_SW_PREEMPTION
        /* Commit done and trigger the lower priority queue. */
        {
            gctINT32 id;

            gcmkVERIFY_OK(gckOS_AtomGet(Kernel->os, Device->atomPriorityID, &id));
            if (id > 0 && Interface->u.CommitDone.priorityID == (gctUINT32)id)
            {
                gcmkONERROR(gckOS_AtomDecrement(Kernel->os, Device->atomPriorityID, &id));

                while (--id)
                {
                    gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->priorityQueueMutex[id], gcvINFINITE));
                    if (!Kernel->priorityQueues[id] || !Kernel->priorityQueues[id]->head)
                    {
                        gcmkONERROR(gckOS_AtomDecrement(Kernel->os, Device->atomPriorityID, &id));
                    }

                    gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->priorityQueueMutex[id]));
                }
            }

            gcmkONERROR(gckOS_ReleaseSemaphoreEx(Kernel->os, Kernel->preemptSema));
        }
#endif

        break;

#if !USE_NEW_LINUX_SIGNAL
    case gcvHAL_USER_SIGNAL:
        /* Dispatch depends on the user signal subcommands. */
        switch(Interface->u.UserSignal.command)
        {
        case gcvUSER_SIGNAL_CREATE:
            /* Create a signal used in the user space. */
            gcmkONERROR(
                gckOS_CreateUserSignal(Kernel->os,
                                       Interface->u.UserSignal.manualReset,
                                       &Interface->u.UserSignal.id));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_DESTROY:
            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));

            /* Destroy the signal. */
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));
            break;

        case gcvUSER_SIGNAL_SIGNAL:
            /* Signal the signal. */
            gcmkONERROR(
                gckOS_SignalUserSignal(Kernel->os,
                                       Interface->u.UserSignal.id,
                                       Interface->u.UserSignal.state));
            break;

        case gcvUSER_SIGNAL_WAIT:
            /* Wait on the signal. */
            status = gckOS_WaitUserSignal(Kernel->os,
                                          Interface->u.UserSignal.id,
                                          Interface->u.UserSignal.wait);
            break;

        case gcvUSER_SIGNAL_MAP:
            gcmkONERROR(
                gckOS_MapSignal(Kernel->os,
                               (gctSIGNAL)(gctUINTPTR_T)Interface->u.UserSignal.id,
                               (gctHANDLE)(gctUINTPTR_T)processID,
                               &signal));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_UNMAP:
            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));

            /* Destroy the signal. */
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));
            break;

        default:
            /* Invalid user signal command. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        break;
#endif

    case gcvHAL_SET_POWER_MANAGEMENT_STATE:
        /* Set the power management state. */
        gcmkONERROR(
            gckHARDWARE_SetPowerState(Kernel->hardware,
                                      Interface->u.SetPowerManagement.state));
        break;

    case gcvHAL_QUERY_POWER_MANAGEMENT_STATE:
        /* Chip is not idle. */
        Interface->u.QueryPowerManagement.isIdle = gcvFALSE;

        /* Query the power management state. */
        gcmkONERROR(gckHARDWARE_QueryPowerState(
            Kernel->hardware,
            &Interface->u.QueryPowerManagement.state));

        /* Query the idle state. */
        gcmkONERROR(
            gckHARDWARE_QueryIdle(Kernel->hardware,
                                  &Interface->u.QueryPowerManagement.isIdle));
        break;

    case gcvHAL_READ_REGISTER:
#if gcdREGISTER_READ_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerState(Kernel->hardware,
                                                              &power));
            if (power == gcvPOWER_ON)
            {
                /* Read a register. */
                gcmkONERROR(gckOS_ReadRegisterEx(
                    Kernel->os,
                    Kernel->core,
                    Interface->u.ReadRegisterData.address,
                    &Interface->u.ReadRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
#else
        /* No access from user land to read registers. */
        Interface->u.ReadRegisterData.data = 0;
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_WRITE_REGISTER:
#if gcdREGISTER_WRITE_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerState(Kernel->hardware,
                                                                  &power));
            if (power == gcvPOWER_ON)
            {
                /* Write a register. */
                gcmkONERROR(
                    gckOS_WriteRegisterEx(Kernel->os,
                                          Kernel->core,
                                          Interface->u.WriteRegisterData.address,
                                          Interface->u.WriteRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.WriteRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
#else
        /* No access from user land to write registers. */
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

        case gcvHAL_WRITE_REGISTER_EX:
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerState(Kernel->hardware,
                                                                  &power));
            if (power == gcvPOWER_ON)
            {
                /* Write a register. */
                gcmkONERROR(
                    gckOS_WriteRegisterEx(Kernel->os,
                                          Kernel->core,
                                          Interface->u.WriteRegisterData.address,
                                          Interface->u.WriteRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.WriteRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
        break;

    case gcvHAL_APB_AXIFE_ACCESS:
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerState(Kernel->hardware,
                                                                  &power));
            if (power == gcvPOWER_ON)
            {
                if (Interface->u.APBAXIFEAccess.isRead)
                {
                    /* Read a register. */
                    gcmkONERROR(gckOS_ReadRegisterEx(
                        Kernel->os,
                        Kernel->core,
                        Interface->u.APBAXIFEAccess.address,
                        &Interface->u.APBAXIFEAccess.data));
                }
                else
                {
                    /* Write a register. */
                    gcmkONERROR(
                        gckOS_WriteRegisterEx(Kernel->os,
                                              Kernel->core,
                                              Interface->u.APBAXIFEAccess.address,
                                              Interface->u.APBAXIFEAccess.data));
                }
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.APBAXIFEAccess.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
        break;

    case gcvHAL_RESET:
        /* Reset the hardware. */
        gcmkONERROR(
            gckHARDWARE_Reset(Kernel->hardware));
        break;

    case gcvHAL_SET_DEBUG_LEVEL_ZONE:
        /* Set debug level and zones. */
        gckOS_SetDebugLevel(Interface->u.DebugLevelZone.level);
        gckOS_SetDebugZones(Interface->u.DebugLevelZone.zones,
                            Interface->u.DebugLevelZone.enable);
        break;

    case gcvHAL_DEBUG_DUMP:
        gckOS_DumpBuffer(Kernel->os,
                         Interface->u.DebugDump.type,
                         gcmUINT64_TO_PTR(Interface->u.DebugDump.ptr),
                         Interface->u.DebugDump.address,
                         Interface->u.DebugDump.size);
        status = gcvSTATUS_OK;
        break;

    case gcvHAL_DUMP_GPU_STATE:
        {
            gceCHIPPOWERSTATE power;

            _DumpDriverConfigure(Kernel);

            gcmkONERROR(gckHARDWARE_QueryPowerState(
                Kernel->hardware,
                &power
                ));

            if (power == gcvPOWER_ON)
            {
                Interface->u.ReadRegisterData.data = 1;

                _DumpState(Kernel);
            }
            else
            {
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;

                gcmkPRINT("[galcore]: Can't dump state if GPU isn't POWER ON.");
            }
        }
        break;

    case gcvHAL_CACHE:
        logical = gcmUINT64_TO_PTR(Interface->u.Cache.logical);
        bytes = (gctSIZE_T) Interface->u.Cache.bytes;

        gcmkONERROR(gckKERNEL_CacheOperation(Kernel,
                                             processID,
                                             Interface->u.Cache.node,
                                             Interface->u.Cache.operation,
                                             logical,
                                             bytes));
        break;

    case gcvHAL_TIMESTAMP:
        /* Check for invalid timer. */
        if ((Interface->u.TimeStamp.timer >= gcmCOUNTOF(Kernel->timers))
        ||  (Interface->u.TimeStamp.request != 2))
        {
            Interface->u.TimeStamp.timeDelta = 0;
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        /* Return timer results and reset timer. */
        {
            gcsTIMER_PTR timer = &(Kernel->timers[Interface->u.TimeStamp.timer]);
            gctUINT64 timeDelta = 0;

            if (timer->stopTime < timer->startTime )
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            timeDelta = timer->stopTime - timer->startTime;

            /* Check truncation overflow. */
            Interface->u.TimeStamp.timeDelta = (gctINT32) timeDelta;
            /*bit0~bit30 is available*/
            if (timeDelta>>31)
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            status = gcvSTATUS_OK;
        }
        break;

    case gcvHAL_DATABASE:
        gcmkONERROR(gckKERNEL_QueryDatabase(Kernel, processID, Interface));
        break;

    case gcvHAL_VERSION:
        Interface->u.Version.major = gcvVERSION_MAJOR;
        Interface->u.Version.minor = gcvVERSION_MINOR;
        Interface->u.Version.patch = gcvVERSION_PATCH;
        Interface->u.Version.build = gcvVERSION_BUILD;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                       "KERNEL version %s", gcvVERSION_STRING);
#endif
        break;

    case gcvHAL_CHIP_INFO:
        /* Only if not support multi-core */
        Interface->u.ChipInfo.count = 1;
        Interface->u.ChipInfo.types[0] = Kernel->hardware->type;
        break;

    case gcvHAL_ATTACH:
        if (Kernel->command)
        {
#if gcdCAPTURE_ONLY_MODE
           gckVIDMEM_NODE nodeObject = gcvNULL;

           if (Interface->u.Attach.queryCapSize)
           {
                /* Attach user process. */
                gcmkONERROR(
                    gckCOMMAND_Attach(Kernel->command,
                                      &context,
                                      &bytes,
                                      &Interface->u.Attach.numStates,
                                      processID));

                Interface->u.Attach.maxState = bytes;
                Interface->u.Attach.context = gcmPTR_TO_NAME(context);

                gcmkONERROR(
                    gckVIDMEM_HANDLE_Lookup(Kernel, processID, context->buffer->handle, &nodeObject));

                Interface->u.Attach.captureSize = nodeObject->captureSize;
                break;
            }
            else
            {
                gctUINT i = 0;
                context = gcmNAME_TO_PTR(Interface->u.Attach.context);

                for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; ++i)
                {
                    gcsCONTEXT_PTR buffer = context->buffer;

                    gckOS_CopyToUserData(Kernel->os, buffer->logical, Interface->u.Attach.contextLogical[i], Interface->u.Attach.captureSize);

                    buffer = buffer->next;
                }
            }

#else
            /* Attach user process. */
            gcmkONERROR(
                gckCOMMAND_Attach(Kernel->command,
                                  &context,
                                  &bytes,
                                  &Interface->u.Attach.numStates,
                                  processID));

            Interface->u.Attach.maxState = bytes;
            Interface->u.Attach.context = gcmPTR_TO_NAME(context);
#endif

            if (Interface->u.Attach.map)
            {
                if (context != gcvNULL)
                {
#if gcdCAPTURE_ONLY_MODE
                    gctUINT i = 0;

                    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; ++i)
                    {
                        Interface->u.Attach.logicals[i] = gcmPTR_TO_UINT64(Interface->u.Attach.contextLogical[i]);
                    }

                    Interface->u.Attach.bytes = (gctUINT)context->totalSize;
#else
                    if (Kernel->command->feType == gcvHW_FE_WAIT_LINK)
                    {
                        gcmkVERIFY_OK(
                            gckCONTEXT_MapBuffer(context,
                                                 Interface->u.Attach.logicals,
                                                 &Interface->u.Attach.bytes));
                    }
#endif
                }
                else
                {
                    gctUINT i;

                    for (i = 0; i < gcmCOUNTOF(Interface->u.Attach.logicals); i++)
                    {
                        Interface->u.Attach.logicals[i] = 0;
                    }

                    Interface->u.Attach.bytes = 0;
                }
            }

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_CONTEXT,
                                       gcmINT2PTR(Interface->u.Attach.context),
                                       gcvNULL,
                                       0));
        }
        break;

    case gcvHAL_DETACH:
        if (Kernel->command)
        {
            gcmkVERIFY_OK(
                gckKERNEL_RemoveProcessDB(Kernel,
                                  processID, gcvDB_CONTEXT,
                                  gcmINT2PTR(Interface->u.Detach.context)));

            /* Detach user process. */
            gcmkONERROR(
                gckCOMMAND_Detach(Kernel->command,
                                  gcmNAME_TO_PTR(Interface->u.Detach.context)));

            gcmRELEASE_NAME(Interface->u.Detach.context);
        }
        break;

    case gcvHAL_GET_FRAME_INFO:
        gcmkONERROR(gckHARDWARE_GetFrameInfo(
                    Kernel->hardware,
                    gcmUINT64_TO_PTR(Interface->u.GetFrameInfo.frameInfo)));
        break;

    case gcvHAL_DUMP_GPU_PROFILE:
        gcmkONERROR(gckHARDWARE_DumpGpuProfile(Kernel->hardware));
        break;

    case gcvHAL_SET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        /* Wait for HW idle, otherwise it is not safe. */
        gcmkONERROR(gckCOMMAND_Stall(Kernel->command, gcvFALSE));

        for (;;)
        {
            gcmkONERROR(gckHARDWARE_QueryIdle(Kernel->hardware, &idle));

            if (idle)
            {
                break;
            }

            gcmkVERIFY_OK(gckOS_Delay(Kernel->os, 1));
        }

        status = gckHARDWARE_SetFscaleValue(Kernel->hardware,
                                            Interface->u.SetFscaleValue.value,
                                            Interface->u.SetFscaleValue.shValue);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;
    case gcvHAL_GET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        status = gckHARDWARE_GetFscaleValue(Kernel->hardware,
                                            &Interface->u.GetFscaleValue.value,
                                            &Interface->u.GetFscaleValue.minValue,
                                            &Interface->u.GetFscaleValue.maxValue);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_EXPORT_VIDEO_MEMORY:
        /* Unlock video memory. */
        gcmkONERROR(_ExportVideoMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_NAME_VIDEO_MEMORY:
        gcmkONERROR(_NameVideoMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_IMPORT_VIDEO_MEMORY:
        gcmkONERROR(_ImportVideoMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_SET_VIDEO_MEMORY_METADATA:
        gcmkONERROR(_SetVidMemMetadata(Kernel, processID, Interface));
        break;

    case gcvHAL_GET_VIDEO_MEMORY_FD:
        gcmkONERROR(_GetVideoMemoryFd(Kernel, processID, Interface));
        break;

    case gcvHAL_QUERY_RESET_TIME_STAMP:
        Interface->u.QueryResetTimeStamp.timeStamp = Kernel->resetTimeStamp;
        Interface->u.QueryResetTimeStamp.contextID = Kernel->hardware->contextID;
        break;

#if gcdLINUX_SYNC_FILE
    case gcvHAL_CREATE_NATIVE_FENCE:
        {
            gctINT fenceFD;
            gctSIGNAL signal =
                gcmUINT64_TO_PTR(Interface->u.CreateNativeFence.signal);

            gcmkONERROR(
                gckOS_CreateNativeFence(Kernel->os,
                                        Kernel->timeline,
                                        signal,
                                        &fenceFD));

            Interface->u.CreateNativeFence.fenceFD = fenceFD;
        }
        break;

    case gcvHAL_WAIT_NATIVE_FENCE:
        {
            gctINT fenceFD;
            gctUINT32 timeout;

            fenceFD = Interface->u.WaitNativeFence.fenceFD;
            timeout = Interface->u.WaitNativeFence.timeout;

            gcmkONERROR(
                gckOS_WaitNativeFence(Kernel->os,
                                      Kernel->timeline,
                                      fenceFD,
                                      timeout));
        }
        break;
#endif

    case gcvHAL_SHBUF:
        {
            gctSHBUF shBuf;
            gctPOINTER uData;
            gctUINT32 bytes;

            switch (Interface->u.ShBuf.command)
            {
            case gcvSHBUF_CREATE:
                bytes = Interface->u.ShBuf.bytes;

                /* Create. */
                gcmkONERROR(gckKERNEL_CreateShBuffer(Kernel, bytes, &shBuf));

                Interface->u.ShBuf.id = gcmPTR_TO_UINT64(shBuf);

                gcmkVERIFY_OK(
                    gckKERNEL_AddProcessDB(Kernel,
                                           processID,
                                           gcvDB_SHBUF,
                                           shBuf,
                                           gcvNULL,
                                           0));
                break;

            case gcvSHBUF_DESTROY:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);

                /* Check db first to avoid illegal destroy in the process. */
                gcmkONERROR(
                    gckKERNEL_RemoveProcessDB(Kernel,
                                              processID,
                                              gcvDB_SHBUF,
                                              shBuf));

                gcmkONERROR(gckKERNEL_DestroyShBuffer(Kernel, shBuf));
                break;

            case gcvSHBUF_MAP:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);

                /* Map for current process access. */
                gcmkONERROR(gckKERNEL_MapShBuffer(Kernel, shBuf));

                gcmkVERIFY_OK(
                    gckKERNEL_AddProcessDB(Kernel,
                                           processID,
                                           gcvDB_SHBUF,
                                           shBuf,
                                           gcvNULL,
                                           0));
                break;

            case gcvSHBUF_WRITE:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);
                uData = gcmUINT64_TO_PTR(Interface->u.ShBuf.data);
                bytes = Interface->u.ShBuf.bytes;

                /* Write. */
                gcmkONERROR(
                    gckKERNEL_WriteShBuffer(Kernel, shBuf, uData, bytes));
                break;

            case gcvSHBUF_READ:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);
                uData = gcmUINT64_TO_PTR(Interface->u.ShBuf.data);
                bytes = Interface->u.ShBuf.bytes;

                /* Read. */
                gcmkONERROR(
                    gckKERNEL_ReadShBuffer(Kernel,
                                           shBuf,
                                           uData,
                                           bytes,
                                           &bytes));

                /* Return copied size. */
                Interface->u.ShBuf.bytes = bytes;
                break;

            default:
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
                break;
            }
        }
        break;

#ifdef __linux__
    case gcvHAL_GET_GRAPHIC_BUFFER_FD:
        gcmkONERROR(_GetGraphicBufferFd(
            Kernel,
            processID,
            Interface->u.GetGraphicBufferFd.node,
            Interface->u.GetGraphicBufferFd.shBuf,
            Interface->u.GetGraphicBufferFd.signal,
            &Interface->u.GetGraphicBufferFd.fd
            ));
        break;
#endif


    case gcvHAL_CONFIG_POWER_MANAGEMENT:
        gcmkONERROR(gckKERNEL_ConfigPowerManagement(Kernel, Interface));
        break;

    case gcvHAL_WRAP_USER_MEMORY:
        gcmkONERROR(_WrapUserMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_WAIT_FENCE:
        gcmkONERROR(_WaitFence(Kernel, processID, Interface));
        break;

    case gcvHAL_DEVICE_MUTEX:
        if (Interface->u.DeviceMutex.isMutexLocked)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Kernel->device->commitMutex,
                gcvINFINITE
                ));
        }
        else
        {
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
        }
        break;

#if gcdDEC_ENABLE_AHB
    case gcvHAL_DEC300_READ:
        gcmkONERROR(viv_dec300_read(
            Interface->u.DEC300Read.enable,
            Interface->u.DEC300Read.readId,
            Interface->u.DEC300Read.format,
            Interface->u.DEC300Read.strides,
            Interface->u.DEC300Read.is3D,
            Interface->u.DEC300Read.isMSAA,
            Interface->u.DEC300Read.clearValue,
            Interface->u.DEC300Read.isTPC,
            Interface->u.DEC300Read.isTPCCompressed,
            Interface->u.DEC300Read.surfAddrs,
            Interface->u.DEC300Read.tileAddrs
            ));
        break;

    case gcvHAL_DEC300_WRITE:
        gcmkONERROR(viv_dec300_write(
            Interface->u.DEC300Write.enable,
            Interface->u.DEC300Write.readId,
            Interface->u.DEC300Write.writeId,
            Interface->u.DEC300Write.format,
            Interface->u.DEC300Write.surfAddr,
            Interface->u.DEC300Write.tileAddr
            ));
        break;

    case gcvHAL_DEC300_FLUSH:
        gcmkONERROR(viv_dec300_flush(0));
        break;

    case gcvHAL_DEC300_FLUSH_WAIT:
        gcmkONERROR(viv_dec300_flush_done(&Interface->u.DEC300FlushWait.done));
        break;
#endif


    case gcvHAL_QUERY_CHIP_OPTION:
        /* Query chip options. */
        gcmkONERROR(
            gckHARDWARE_QueryChipOptions(
                Kernel->hardware,
                &Interface->u.QueryChipOptions));
        break;

    case gcvHAL_SYNC_VIDEO_MEMORY:
        {
            gckVIDMEM_NODE nodeObject;

            gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
                Kernel,
                processID,
                (gctUINT32)Interface->u.SyncVideoMemory.node,
                &nodeObject
                ));

            if (nodeObject->pool == gcvPOOL_LOCAL_EXCLUSIVE
                && nodeObject->transitNode != gcvNULL
                && nodeObject->transitNode->Virtual.logical != gcvNULL)
            {
                gcmkONERROR(gckKERNEL_SyncVideoMemory(
                    Kernel,
                    nodeObject,
                    Interface->u.SyncVideoMemory.reason
                    ));
            }
        }
        break;

    default:
        /* Invalid command. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

OnError:
    /* Save status. */
    Interface->status = status;

#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_ReleaseMutex(Kernel->os, Kernel->debugMutex);
#endif

    if (powerMutexAcquired == gcvTRUE)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
    }

    if (commitMutexAcquired == gcvTRUE)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcess
**
**  Attach or detach a process.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcess(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach
    )
{
    gceSTATUS status;
    gctUINT32 processID;

    gcmkHEADER_ARG("Kernel=%p Attach=%d", Kernel, Attach);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Get current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

    gcmkONERROR(gckKERNEL_AttachProcessEx(Kernel, Attach, processID));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcessEx
**
**  Attach or detach a process with the given PID. Can be paired with gckKERNEL_AttachProcess
**     provided the programmer is aware of the consequences.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**      gctUINT32 PID
**          PID of the process to attach or detach.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcessEx(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach,
    IN gctUINT32 PID
    )
{
    gceSTATUS status;
    gctINT32 old;

    gcmkHEADER_ARG("Kernel=%p Attach=%d PID=%d", Kernel, Attach, PID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Attach)
    {
        /* Increment the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomIncrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 0)
        {
            {
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_FIRST_PROCESS));
            }
        }

        if (Kernel->dbCreated)
        {
            /* Create the process database. */
            gcmkONERROR(gckKERNEL_CreateProcessDB(Kernel, PID));
        }
    }
    else
    {
        if (Kernel->dbCreated)
        {
            /* Clean up the process database. */
            gcmkONERROR(gckKERNEL_DestroyProcessDB(Kernel, PID));

            /* Save the last know process ID. */
            Kernel->db->lastProcessID = PID;
        }

        {
            status = gckEVENT_Submit(Kernel->eventObj, gcvTRUE, gcvFALSE, gcvTRUE);

            if (status == gcvSTATUS_INTERRUPTED && Kernel->eventObj->submitTimer)
            {
                gcmkONERROR(gckOS_StartTimer(Kernel->os,
                                             Kernel->eventObj->submitTimer,
                                             1));
            }
            else
            {
                gcmkONERROR(status);
            }
        }

        /* Decrement the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomDecrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 1)
        {
            {
                /* Last client detached, switch to SUSPEND power state. */
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_LAST_PROCESS));
            }
        }

        if (Kernel->timeoutPID == PID)
        {
            Kernel->timeOut = Kernel->hardware->type == gcvHARDWARE_2D
                            ? gcdGPU_2D_TIMEOUT
                            : gcdGPU_TIMEOUT;
        }
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_Recovery
**
**  Try to recover the GPU from a fatal error.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Recovery(
    IN gckKERNEL Kernel
    )
{
    gceSTATUS status;
    gckEVENT eventObj;
    gckHARDWARE hardware;
    gctUINT32 mask = 0;
    gctUINT32 i = 0, count = 0;
#if gcdINTERRUPT_STATISTIC
    gctINT32 oldValue;
#endif

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Grab gckEVENT object. */
    eventObj = Kernel->eventObj;
    gcmkVERIFY_OBJECT(eventObj, gcvOBJ_EVENT);

    /* Grab gckHARDWARE object. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    gckOS_AcquireMutex(Kernel->os, Kernel->device->commitMutex, gcdRECOVERY_FORCE_TIMEOUT);

    if (Kernel->stuckDump == gcvSTUCK_DUMP_NONE)
    {
        gcmkPRINT("[galcore]: GPU[%d] hang, automatic recovery.", Kernel->core);
    }
    else if (Kernel->stuckDump == gcvSTUCK_DUMP_ALL_CORE)
    {
        gckDEVICE device = Kernel->device;
        gckKERNEL kernel = gcvNULL;

        gcmkASSERT(device != gcvNULL);

        gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->device->stuckDumpMutex, gcvINFINITE));

        for (i = 0; i < device->coreNum; i++)
        {
            kernel = device->coreInfoArray[i].kernel;
            gcmkASSERT(kernel != gcvNULL);

            _DumpDriverConfigure(kernel);
            _DumpState(kernel);
        }

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->stuckDumpMutex));
    }
    else
    {
        gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->device->stuckDumpMutex, gcvINFINITE));

        _DumpDriverConfigure(Kernel);
        _DumpState(Kernel);

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->stuckDumpMutex));
    }

    if (Kernel->recovery == gcvFALSE)
    {
        gcmkPRINT("[galcore]: Stop driver to keep scene.");

        /* Stop monitor timer. */
        Kernel->monitorTimerStop = gcvTRUE;

        /* Success. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    /* Issuing a soft reset for the GPU. */
    gcmkONERROR(gckHARDWARE_Reset(hardware));

    gcmkVERIFY_OK(gckOS_AtomGet(
        Kernel->os,
        Kernel->hardware->pendingEvent,
        (gctINT32 *)&mask
        ));

    if (mask)
    {
        /* Handle all outstanding events now. */
        gcmkONERROR(gckOS_AtomSet(Kernel->os, eventObj->pending, mask));
    }

    for (i = 0; i < 32; i++)
    {
        if (mask & (1 << i))
        {
            count++;
        }
    }


#if gcdINTERRUPT_STATISTIC
    while (count--)
    {
        gcmkONERROR(gckOS_AtomDecrement(
            Kernel->os,
            eventObj->interruptCount,
            &oldValue
            ));
    }

    gckOS_AtomClearMask(Kernel->hardware->pendingEvent, mask);
#endif

    gcmkONERROR(gckEVENT_Notify(eventObj, 1, gcvNULL));

    gcmkVERIFY_OK(gckOS_GetTime(&Kernel->resetTimeStamp));

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_OpenUserData
**
**  Get access to the user data.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctPOINTER StaticStorage
**          Pointer to the kernel storage where the data is to be copied if
**          NeedCopy is gcvTRUE.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Pointer to the kernel pointer that will be pointing to the data.
*/
gceSTATUS
gckKERNEL_OpenUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctPOINTER StaticStorage,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG(
        "Kernel=%p NeedCopy=%d StaticStorage=%p "
        "UserPointer=%p Size=%lu KernelPointer=%p",
        Kernel, NeedCopy, StaticStorage, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(!NeedCopy || (StaticStorage != gcvNULL));
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    if (NeedCopy)
    {
        /* Copy the user data to the static storage. */
        gcmkONERROR(gckOS_CopyFromUserData(
            Kernel->os, StaticStorage, UserPointer, Size
            ));

        /* Set the kernel pointer. */
        * KernelPointer = StaticStorage;
    }
    else
    {
        gctPOINTER pointer = gcvNULL;

        /* Map the user pointer. */
        gcmkONERROR(gckOS_MapUserPointer(
            Kernel->os, UserPointer, Size, &pointer
            ));

        /* Set the kernel pointer. */
        * KernelPointer = pointer;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_CloseUserData
**
**  Release resources associated with the user data connection opened by
**  gckKERNEL_OpenUserData.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctBOOL FlushData
**          If gcvTRUE, the data is written back to the user.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Kernel pointer to the data.
*/
gceSTATUS
gckKERNEL_CloseUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctBOOL FlushData,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPOINTER pointer;

    gcmkHEADER_ARG(
        "Kernel=%p NeedCopy=%d FlushData=%d "
        "UserPointer=%p Size=%lu KernelPointer=%p",
        Kernel, NeedCopy, FlushData, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    /* Get a shortcut to the kernel pointer. */
    pointer = * KernelPointer;

    if (pointer != gcvNULL)
    {
        if (NeedCopy)
        {
            if (FlushData)
            {
                gcmkONERROR(gckOS_CopyToUserData(
                    Kernel->os, * KernelPointer, UserPointer, Size
                    ));
            }
        }
        else
        {
            /* Unmap record from kernel memory. */
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Kernel->os,
                UserPointer,
                Size,
                * KernelPointer
                ));
        }

        /* Reset the kernel pointer. */
        * KernelPointer = gcvNULL;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static void
gckQUEUE_Dequeue(
    IN gckQUEUE LinkQueue
    )
{
    gcmkASSERT(LinkQueue->count == LinkQueue->size);

    LinkQueue->count--;
    LinkQueue->front = (LinkQueue->front + 1) % gcdLINK_QUEUE_SIZE;
}

void
gckQUEUE_Enqueue(
    IN gckQUEUE LinkQueue,
    IN gcuQUEUEDATA *Data
    )
{
    gcuQUEUEDATA * datas = LinkQueue->datas;

    if (LinkQueue->count == LinkQueue->size)
    {
        gckQUEUE_Dequeue(LinkQueue);
    }

    gcmkASSERT(LinkQueue->count < LinkQueue->size);

    LinkQueue->count++;

    datas[LinkQueue->rear] = *Data;

    LinkQueue->rear = (LinkQueue->rear + 1) % LinkQueue->size;
}

void
gckQUEUE_GetData(
    IN gckQUEUE LinkQueue,
    IN gctUINT32 Index,
    OUT gcuQUEUEDATA ** Data
    )
{
    gcuQUEUEDATA * datas = LinkQueue->datas;

    gcmkASSERT(Index < LinkQueue->size);

    *Data = &datas[(Index + LinkQueue->front) % LinkQueue->size];
}

gceSTATUS
gckQUEUE_Allocate(
    IN gckOS Os,
    IN gckQUEUE Queue,
    IN gctUINT32 Size
    )
{
    gceSTATUS status;

    gcmkONERROR(gckOS_Allocate(
        Os,
        gcmSIZEOF(struct _gckLINKDATA) * Size,
        (gctPOINTER *)&Queue->datas
        ));

    Queue->size = Size;

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckQUEUE_Free(
    IN gckOS Os,
    IN gckQUEUE Queue
    )
{
    if (Queue->datas)
    {
        gcmkVERIFY_OK(gckOS_Free(Os, (gctPOINTER)Queue->datas));
    }

    return gcvSTATUS_OK;
}

/******************************************************************************\
*************************** Pointer - ID translation ***************************
\******************************************************************************/

/* The capacity is 32 initially, double when expand, but no more than 512. */
#define gcdID_TABLE_MAX_EXPAND    512

typedef struct _gcsINTEGERDB * gckINTEGERDB;
typedef struct _gcsINTEGERDB
{
    gckOS        os;
    gctPOINTER * table;
    gctUINT32 *  bitmap;
    gctPOINTER   mutex;
    gctUINT32    capacity;
    gctUINT32    nextId;
    gctUINT32    freeCount;
}
gcsINTEGERDB;

gceSTATUS
gckKERNEL_CreateIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 Capacity,
    OUT gctPOINTER * Database
    )
{
    gceSTATUS status;
    gckINTEGERDB database = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p Capacity=%u Datbase=%p", Kernel, Capacity, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* round up to 32 alignment. */
    Capacity = (Capacity + 31) & ~31;

    /* Allocate a database. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, gcmSIZEOF(gcsINTEGERDB), (gctPOINTER *)&database));

    gcmkONERROR(gckOS_ZeroMemory(database, gcmSIZEOF(gcsINTEGERDB)));

    /* Allocate a pointer table. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os,
        gcmSIZEOF(gctPOINTER) * Capacity,
        (gctPOINTER *)&database->table
        ));

    /* Allocate bitmap. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, Capacity / 8, (gctPOINTER *)&database->bitmap));

    gcmkONERROR(gckOS_ZeroMemory(database->bitmap, Capacity / 8));

    /* Allocate a database mutex. */
    gcmkONERROR(gckOS_CreateMutex(Kernel->os, &database->mutex));

    /* Initialize. */
    database->os        = Kernel->os;

    database->nextId    = 0;
    database->freeCount = Capacity;
    database->capacity  = Capacity;

    *Database = database;

    gcmkFOOTER_ARG("*Database=0x%08X", *Database);
    return gcvSTATUS_OK;

OnError:
    /* Rollback. */
    if (database)
    {
        if (database->table)
        {
            gckOS_Free(Kernel->os, database->table);
        }

        if (database->bitmap)
        {
            gckOS_Free(Kernel->os, database->bitmap);
        }

        gckOS_Free(Kernel->os, database);
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_DestroyIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctPOINTER Database
    )
{
    gckINTEGERDB database = Database;

    gcmkHEADER_ARG("Kernel=%p Datbase=%p", Kernel, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* Destroy pointer table. */
    gcmkOS_SAFE_FREE(Kernel->os, database->table);
    gcmkOS_SAFE_FREE(Kernel->os, database->bitmap);

    /* Destroy database mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, database->mutex));

    /* Destroy database. */
    gckOS_Free(Kernel->os, database);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_AllocateIntegerId(
    IN gctPOINTER Database,
    IN gctPOINTER Pointer,
    OUT gctUINT32 * Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gctUINT32 pos;
    gctUINT32 n, i;
    gckOS os = database->os;
    gctPOINTER * table = gcvNULL;

    gcmkHEADER_ARG("Database=%p Pointer=%p", Database, Pointer);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));

    if (database->freeCount < 1)
    {
        gctUINT32 * bitmap = gcvNULL;
        gctUINT32 expand;
        gctUINT32 capacity;

        expand = database->capacity < gcdID_TABLE_MAX_EXPAND
               ? database->capacity : gcdID_TABLE_MAX_EXPAND;

        capacity = database->capacity + expand;

        /* Extend table. */
        gcmkONERROR(
            gckOS_Allocate(os,
                           gcmSIZEOF(gctPOINTER) * capacity,
                           (gctPOINTER *)&table));

        gcmkONERROR(
            gckOS_Allocate(os, capacity / 32 * sizeof(gctUINT32), (gctPOINTER *)&bitmap));

        gckOS_ZeroMemory(bitmap + database->capacity / 32, expand / 8);

        /* Copy data from old table. */
        gckOS_MemCopy(table,
                      database->table,
                      database->capacity * gcmSIZEOF(gctPOINTER));

        gckOS_MemCopy(bitmap,
                      database->bitmap,
                      database->capacity / 32 * sizeof(gctUINT32));

        gckOS_Free(os, database->table);
        gckOS_Free(os, database->bitmap);

        /* Update databse with new allocated table. */
        database->table      = table;
        database->bitmap     = bitmap;
        database->nextId     = database->capacity;
        database->capacity   = capacity;
        database->freeCount += expand;
    }

    pos = database->nextId;
    n = pos >> 5;
    i = pos & 31;

    while (database->bitmap[n] & (1u << i))
    {
        pos++;

        if (pos == database->capacity)
        {
            /* Wrap to the begin. */
            pos = 0;
        }

        n = pos >> 5;
        i = pos & 31;
    }

    /* Connect id with pointer. */
    database->table[pos] = Pointer;
    database->bitmap[n] |= (1u << i);

    *Id = pos + 1;

    database->nextId = (pos + 1) % database->capacity;
    database->freeCount--;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));

    gcmkFOOTER_ARG("*Id=%u", *Id);
    return gcvSTATUS_OK;

OnError:
    if (table)
    {
        gckOS_Free(os, table);
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_FreeIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gckOS os = database->os;
    gctUINT32 pos = Id - 1;
    gctUINT32 n, i;

    gcmkHEADER_ARG("Database=%p Id=%d", Database, Id);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));

    n = pos >> 5;
    i = pos & 31;

    if (pos >= database->capacity || (database->bitmap[n] & (1u << i)) == 0)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    /* Clear id. */
    database->bitmap[n] &= ~(1u << i);
    database->table[pos] = gcvNULL;

    ++database->freeCount;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_QueryIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id,
    OUT gctPOINTER * Pointer
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gckOS os = database->os;
    gctUINT32 pos = Id - 1;
    gctUINT32 n, i;

    gcmkHEADER_ARG("Database=%p Id=%d", Database, Id);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));

    n = pos >> 5;
    i = pos & 31;

    if (pos >= database->capacity || (database->bitmap[n] & (1u << i)) == 0)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    *Pointer = database->table[pos];

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));

    gcmkFOOTER_ARG("*Pointer=0x%08X", *Pointer);
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gctUINT32
gckKERNEL_AllocateNameFromPointer(
    IN gckKERNEL Kernel,
    IN gctPOINTER Pointer
    )
{
    gceSTATUS status;
    gctUINT32 name;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=%p Pointer=%p", Kernel, Pointer);

    gcmkONERROR(gckKERNEL_AllocateIntegerId(database, Pointer, &name));

    gcmkFOOTER_ARG("name=%d", name);
    return name;

OnError:
    gcmkFOOTER();
    return 0;
}

gctPOINTER
gckKERNEL_QueryPointerFromName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gceSTATUS status;
    gctPOINTER pointer = gcvNULL;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=%p Name=%d", Kernel, Name);

    /* Lookup in database to get pointer. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Name, &pointer));

    gcmkFOOTER_ARG("pointer=0x%X", pointer);
    return pointer;

OnError:
    gcmkFOOTER();
    return gcvNULL;
}

gceSTATUS
gckKERNEL_DeleteName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=%p Name=%p", Kernel, Name);

    /* Free name if exists. */
    gcmkVERIFY_OK(gckKERNEL_FreeIntegerId(database, Name));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
***** Shared Buffer ************************************************************
*******************************************************************************/

/*******************************************************************************
**
**  gckKERNEL_CreateShBuffer
**
**  Create shared buffer.
**  The shared buffer can be used across processes. Other process needs call
**  gckKERNEL_MapShBuffer before use it.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 Size
**          Specify the shared buffer size.
**
**  OUTPUT:
**
**      gctSHBUF * ShBuf
**          Pointer to hold return shared buffer handle.
*/
gceSTATUS
gckKERNEL_CreateShBuffer(
    IN gckKERNEL Kernel,
    IN gctUINT32 Size,
    OUT gctSHBUF * ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p, Size=%u", Kernel, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Size == 0)
    {
        /* Invalid size. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }
    else if (Size > 1024)
    {
        /* Limite shared buffer size. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    /* Create a shared buffer structure. */
    gcmkONERROR(
        gckOS_Allocate(Kernel->os,
                       sizeof (gcsSHBUF),
                       (gctPOINTER *)&shBuf));

    /* Initialize shared buffer. */
    shBuf->id        = 0;
    shBuf->reference = gcvNULL;
    shBuf->size      = Size;
    shBuf->data      = gcvNULL;

    /* Allocate integer id for this shared buffer. */
    gcmkONERROR(
        gckKERNEL_AllocateIntegerId(Kernel->db->pointerDatabase,
                                    shBuf,
                                    &shBuf->id));

    /* Allocate atom. */
    gcmkONERROR(gckOS_AtomConstruct(Kernel->os, &shBuf->reference));

    /* Set default reference count to 1. */
    gcmkVERIFY_OK(gckOS_AtomSet(Kernel->os, shBuf->reference, 1));

    /* Return integer id. */
    *ShBuf = (gctSHBUF)(gctUINTPTR_T)shBuf->id;

    gcmkFOOTER_ARG("*ShBuf=%u", shBuf->id);
    return gcvSTATUS_OK;

OnError:
    /* Error roll back. */
    if (shBuf != gcvNULL)
    {
        if (shBuf->id != 0)
        {
            gcmkVERIFY_OK(
                gckKERNEL_FreeIntegerId(Kernel->db->pointerDatabase,
                                        shBuf->id));
        }

        gcmkOS_SAFE_FREE(Kernel->os, shBuf);
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_DestroyShBuffer
**
**  Destroy shared buffer.
**  This will decrease reference of specified shared buffer and do actual
**  destroy when no reference on it.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_DestroyShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctINT32 oldValue = 0;

    gcmkHEADER_ARG("Kernel=%p ShBuf=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    /* Decrease the reference count. */
    gckOS_AtomDecrement(Kernel->os, shBuf->reference, &oldValue);

    if (oldValue == 1)
    {
        /* Free integer id. */
        gcmkVERIFY_OK(
            gckKERNEL_FreeIntegerId(Kernel->db->pointerDatabase,
                                    shBuf->id));

        /* Free atom. */
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, shBuf->reference));

        if (shBuf->data)
        {
            gcmkOS_SAFE_FREE(Kernel->os, shBuf->data);
            shBuf->data = gcvNULL;
        }

        /* Free the shared buffer. */
        gcmkOS_SAFE_FREE(Kernel->os, shBuf);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_MapShBuffer
**
**  Map shared buffer into this process so that it can be used in this process.
**  This will increase reference count on the specified shared buffer.
**  Call gckKERNEL_DestroyShBuffer to dereference.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be mapped.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_MapShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctINT32 oldValue = 0;

    gcmkHEADER_ARG("Kernel=%p ShBuf=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    /* Increase the reference count. */
    gckOS_AtomIncrement(Kernel->os, shBuf->reference, &oldValue);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_WriteShBuffer
**
**  Write user data into shared buffer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be written to.
**
**      gctPOINTER UserData
**          User mode pointer to hold the source data.
**
**      gctUINT32 ByteCount
**          Specify number of bytes to write. If this is larger than
**          shared buffer size, gcvSTATUS_INVALID_ARGUMENT is returned.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_WriteShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p ShBuf=%u UserData=%p ByteCount=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf, UserData, ByteCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    if ((ByteCount > shBuf->size) ||
        (ByteCount == 0) ||
        (UserData  == gcvNULL))
    {
        /* Exceeds buffer max size or invalid. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (shBuf->data == gcvNULL)
    {
        /* Allocate buffer data when first time write. */
        gcmkONERROR(gckOS_Allocate(Kernel->os, ByteCount, &shBuf->data));
    }

    /* Copy data from user. */
    gcmkONERROR(
        gckOS_CopyFromUserData(Kernel->os,
                               shBuf->data,
                               UserData,
                               ByteCount));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (shBuf && shBuf->data)
    {
        gcmkOS_SAFE_FREE(Kernel->os, shBuf->data);
        shBuf->data = gcvNULL;
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_ReadShBuffer
**
**  Read data from shared buffer and copy to user pointer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be read from.
**
**      gctPOINTER UserData
**          User mode pointer to save output data.
**
**      gctUINT32 ByteCount
**          Specify number of bytes to read.
**          If this is larger than shared buffer size, only avaiable bytes are
**          copied. If smaller, copy requested size.
**
**  OUTPUT:
**
**      gctUINT32 * BytesRead
**          Pointer to hold how many bytes actually read from shared buffer.
*/
gceSTATUS
gckKERNEL_ReadShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount,
    OUT gctUINT32 * BytesRead
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctUINT32 bytes;

    gcmkHEADER_ARG("Kernel=%p ShBuf=%u UserData=%p ByteCount=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf, UserData, ByteCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    if (shBuf->data == gcvNULL)
    {
        *BytesRead = 0;

        /* No data in shared buffer, skip copy. */
        status = gcvSTATUS_SKIP;
        goto OnError;
    }
    else if (ByteCount == 0)
    {
        /* Invalid size to read. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Determine bytes to copy. */
    bytes = (ByteCount < shBuf->size) ? ByteCount : shBuf->size;

    /* Copy data to user. */
    gcmkONERROR(
        gckOS_CopyToUserData(Kernel->os,
                             shBuf->data,
                             UserData,
                             bytes));

    /* Return copied size. */
    *BytesRead = bytes;

    gcmkFOOTER_ARG("*BytesRead=%u", bytes);
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************\
*************************** List Helper *****************************************
\*******************************************************************************/

static void
_ListAdd(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Prev,
    gcsLISTHEAD_PTR Next
    )
{
    Next->prev = New;
    New->next  = Next;
    New->prev  = Prev;
    Prev->next = New;
}

void
_ListDel(
    gcsLISTHEAD_PTR Prev,
    gcsLISTHEAD_PTR Next
    )
{
    Next->prev = Prev;
    Prev->next = Next;
}

void
gcsLIST_Init(
    gcsLISTHEAD_PTR Node
    )
{
    Node->prev = Node;
    Node->next = Node;
}

void
gcsLIST_Add(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    )
{
    _ListAdd(New, Head, Head->next);
}

void
gcsLIST_AddTail(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    )
{
    _ListAdd(New, Head->prev, Head);
}

void
gcsLIST_Del(
    gcsLISTHEAD_PTR Node
    )
{
    _ListDel(Node->prev, Node->next);
}

gctBOOL
gcsLIST_Empty(
    gcsLISTHEAD_PTR Head
    )
{
    return Head->next == Head;
}

/*******************************************************************************\
********************************* Fence *****************************************
\*******************************************************************************/

gceSTATUS
gckFENCE_Create(
    IN gckOS Os,
    IN gckKERNEL Kernel,
    OUT gckFENCE * Fence
    )
{
    gceSTATUS status;

#if !gcdCAPTURE_ONLY_MODE
    gcePOOL pool = gcvPOOL_DEFAULT;
#else
    gcePOOL pool = gcvPOOL_VIRTUAL;
#endif

    gckFENCE fence = gcvNULL;
    gctSIZE_T size = 8;
    gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

#ifdef MSDX
    gctUINT64 wddmMode = 0;

    if ((gckOS_QueryOption(Os, "wddmMode", &wddmMode) == gcvSTATUS_OK) &&
        (wddmMode))
    {
        allocFlag &= ~gcvALLOC_FLAG_CONTIGUOUS;
    }
#endif

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif


    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcsFENCE), (gctPOINTER *)&fence));
    gcmkONERROR(gckOS_ZeroMemory(fence, gcmSIZEOF(gcsFENCE)));
    gcmkONERROR(gckOS_CreateMutex(Os, (gctPOINTER *)&fence->mutex));

    fence->kernel = Kernel;

    /* Allocate video memory node for fence. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        Kernel,
        64,
        gcvVIDMEM_TYPE_FENCE,
        allocFlag,
        &size,
        &pool,
        &fence->videoMem
        ));

    /* Lock for GPU access. */
    gcmkONERROR(gckVIDMEM_NODE_Lock(
        Kernel,
        fence->videoMem,
        &fence->address
        ));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        Kernel,
        fence->videoMem,
        gcvFALSE,
        gcvFALSE,
        &fence->logical
        ));

    gcsLIST_Init(&fence->waitingList);

    *Fence = fence;

    return gcvSTATUS_OK;
OnError:
    if (fence)
    {
        gckFENCE_Destory(Os, fence);
    }

    return status;
}

gceSTATUS
gckFENCE_Destory(
    IN gckOS Os,
    OUT gckFENCE Fence
    )
{
    if (Fence->mutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Fence->mutex));
    }

    if (Fence->logical)
    {
        gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
            Fence->kernel,
            Fence->videoMem,
            0,
            gcvFALSE,
            gcvFALSE
            ));

        /* Synchronueous unlock. */
        gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
            Fence->kernel,
            Fence->videoMem,
            0,
            gcvNULL
            ));

        /* Free video memory. */
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            Fence->kernel,
            Fence->videoMem
            ));
    }

    gcmkOS_SAFE_FREE(Os, Fence);

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckFENCE_Signal
**
**  Signal all completed nodes.
**
**
*/
gceSTATUS
gckFENCE_Signal(
    IN gckOS Os,
    IN gckFENCE Fence
    )
{
    gcsLISTHEAD_PTR list = &Fence->waitingList;
    gcsLISTHEAD_PTR nodeHead, nodeTemp;
    gckFENCE_SYNC sync;
    gckOS os = Os;
    gctUINT64 stamp = *(gctUINT64 *)Fence->logical;

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, Fence->mutex, gcvINFINITE));

    gcmkLIST_FOR_EACH_SAFE(nodeHead, nodeTemp, list)
    {
        sync = gcmCONTAINEROF(nodeHead, struct _gcsFENCE_SYNC, head);

        /* Signal all nodes which are complete. */
        if (sync->commitStamp <= stamp && sync->inList)
        {
            /* Signal. */
            gckOS_Signal(os, sync->signal, gcvTRUE);

            /* Remove from wait list. */
            gcsLIST_Del(nodeHead);

            /* Mark node not in waiting list. */
            sync->inList = gcvFALSE;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Fence->mutex));

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Construct(
    IN gckOS Os,
    OUT gckDEVICE * Device
    )
{
    gceSTATUS status;
    gckDEVICE device;
    gctUINT i, j;

    gcmkHEADER();

    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcsDEVICE), (gctPOINTER *)&device));

    gckOS_ZeroMemory(device, gcmSIZEOF(gcsDEVICE));

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        device->coreInfoArray[i].type = gcvHARDWARE_INVALID;

        /* Initialize internal SRAM. */
        for (j = 0; j < gcvSRAM_INTER_COUNT; j++)
        {
            device->sRAMBases[i][j] = gcvINVALID_PHYSICAL_ADDRESS;
            device->sRAMSizes[i][j] = 0;
            device->sRAMPhysFaked[i][j] = gcvFALSE;
        }
    }

    /* Initialize external SRAM. */
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        device->extSRAMGPUBases[i] = gcvINVALID_PHYSICAL_ADDRESS;
        device->extSRAMBases[i] = gcvINVALID_PHYSICAL_ADDRESS;
        device->extSRAMSizes[i] = 0;
    }

    device->defaultHwType = gcvHARDWARE_INVALID;

    gcmkONERROR(gckOS_CreateMutex(Os, &device->stuckDumpMutex));
    gcmkONERROR(gckOS_CreateMutex(Os, &device->commitMutex));
    gcmkONERROR(gckOS_CreateMutex(Os, &device->powerMutex));

#if gcdENABLE_SW_PREEMPTION
    gcmkONERROR(gckOS_AtomConstruct(Os, &device->atomPriorityID));

    gcmkVERIFY_OK(gckOS_AtomSet(Os, device->atomPriorityID, 0));
#endif

    device->os = Os;
    device->showSRAMMapInfo = 0;

    *Device = device;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    if (device != gcvNULL)
    {
        gckDEVICE_Destroy(Os, device);
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckDEVICE_AddCore(
    IN gckDEVICE Device,
    IN gceCORE Core,
    IN gctUINT ChipID,
    IN gctPOINTER Context,
    IN gckKERNEL * Kernel
    )
{
    gceSTATUS status;
    gcsCORE_INFO * info = Device->coreInfoArray;
    gceHARDWARE_TYPE type = (gceHARDWARE_TYPE)((gctUINT)gcvHARDWARE_INVALID);
    gctUINT32 index = Device->coreNum;
    gctUINT32 i;
    gcsCORE_LIST *coreList;
    gceHARDWARE_TYPE kernelType;
    gceHARDWARE_TYPE defaultHwType;
    gckKERNEL kernel;

    gcmkHEADER_ARG("Device=%p Core=%d ChipID=%d Context=%p Kernel=%p",
        Device, Core, ChipID, Context, Kernel);

    gcmkASSERT(Device->coreNum < gcvCORE_COUNT);

    if (Core >= gcvCORE_MAJOR && Core <= gcvCORE_3D_MAX)
    {
        if (ChipID == gcvCHIP_ID_DEFAULT)
        {
            /* Apply default chipID if it is not set. */
            ChipID = Core;
        }
    }

    if (Core >= gcvCORE_2D && Core <= gcvCORE_2D_MAX)
    {
        if (ChipID == gcvCHIP_ID_DEFAULT)
        {
            /* Apply default 2D chipID if it is not set. */
            ChipID = Core - gcvCORE_2D;
        }
    }

    /* Construct gckKERNEL for this core. */
    gcmkONERROR(gckKERNEL_Construct(
        Device->os, Core, ChipID, Context, Device, Device->database, Kernel));

    kernel = *Kernel;

    if (Device->database == gcvNULL)
    {
        Device->database = kernel->db;
    }

    gcmkVERIFY_OK(
        gckKERNEL_GetHardwareType(kernel,
                                  &kernelType));

    if (kernelType >= gcvHARDWARE_NUM_TYPES)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    info[index].type = kernelType;
    info[index].core = Core;
    info[index].kernel = kernel;
    info[index].chipID = ChipID;

    if (index == 0)
    {
        /* First core, map all type/core to it. */
        for (; type != gcvHARDWARE_NUM_TYPES; type = (gceHARDWARE_TYPE)((gctUINT)type + 1))
        {
            Device->map[type].num = 0;

            for (i = 0 ; i < 4; i++)
            {
                Device->map[type].kernels[i] = kernel;
            }
        }
    }

    /* Get core list of this type. */
    coreList = &Device->map[kernelType];

    /* Setup gceHARDWARE_TYPE to gceCORE mapping. */
    coreList->kernels[coreList->num++] = kernel;

    defaultHwType = kernelType;
    if (kernelType == gcvHARDWARE_3D2D)
    {
        coreList = &Device->map[gcvHARDWARE_3D];
        coreList->kernels[coreList->num++] = kernel;
        defaultHwType = gcvHARDWARE_3D;
    }

    /* Advance total core number. */
    Device->coreNum++;

    /* Default HW type was chosen: 3D > 2D > VG */
    if (Device->defaultHwType == gcvHARDWARE_INVALID)
    {
        Device->defaultHwType = defaultHwType;
    }
    else if (Device->defaultHwType > defaultHwType)
    {
        Device->defaultHwType = defaultHwType;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckDEVICE_ChipInfo(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    {
        gctUINT i;
        gcsCORE_INFO * info = Device->coreInfoArray;

        for (i = 0; i < Device->coreNum; i++)
        {
            Interface->u.ChipInfo.types[i] = info[i].type;
            Interface->u.ChipInfo.ids[i] = info[i].chipID;

            Interface->u.ChipInfo.coreIndexs[i] = i;
        }

        Interface->u.ChipInfo.count = Device->coreNum;
    }

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Version(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    Interface->u.Version.major = gcvVERSION_MAJOR;
    Interface->u.Version.minor = gcvVERSION_MINOR;
    Interface->u.Version.patch = gcvVERSION_PATCH;
    Interface->u.Version.build = gcvVERSION_BUILD;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "KERNEL version %s", gcvVERSION_STRING);
#endif

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Destroy(
    IN gckOS Os,
    IN gckDEVICE Device
    )
{
    gctINT i;
    gcsCORE_INFO * info = Device->coreInfoArray;

    for (i = Device->coreNum - 1; i >= 0 ; i--)
    {
        if (info[i].kernel != gcvNULL)
        {
            gckKERNEL_Destroy(info[i].kernel);
        }
    }

    if (Device->powerMutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Device->powerMutex));
    }

    if (Device->commitMutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Device->commitMutex));
    }
    if (Device->stuckDumpMutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Device->stuckDumpMutex));
    }

#if gcdENABLE_SW_PREEMPTION
    if (Device->atomPriorityID)
    {
        gcmkVERIFY_OK(gckOS_AtomDestroy(Os, Device->atomPriorityID));
    }
#endif

    gcmkOS_SAFE_FREE(Os, Device);

    return gcvSTATUS_OK;
}

static gceSTATUS
gckDEVICE_SetTimeOut(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
#if gcdGPU_TIMEOUT
    gckKERNEL kernel;
    gctUINT i;
    gceHARDWARE_TYPE type = Interface->hardwareType;
    gcsCORE_LIST *coreList;
    gctUINT32 processID = 0;
    gcsCORE_INFO *info = Device->coreInfoArray;

    coreList = &Device->map[type];

    /* Get the current process ID. */
    gckOS_GetProcessID(&processID);

    for (i = 0; i < Device->coreNum; i++)
    {
        if (type == gcvHARDWARE_3D || type == gcvHARDWARE_3D2D || type == gcvHARDWARE_VIP)
        {
            kernel = info[i].kernel;
        }
        else
        {
            kernel = coreList->kernels[i];
        }

        kernel->timeOut = Interface->u.SetTimeOut.timeOut;

        kernel->timeoutPID = processID;
    }
#endif

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckKERNEL kernel;
    gceHARDWARE_TYPE type = Interface->hardwareType;
    gctUINT32 coreIndex = Interface->coreIndex;

    switch (Interface->command)
    {
    case gcvHAL_CHIP_INFO:
        status = gckDEVICE_ChipInfo(Device, Interface);
        break;

    case gcvHAL_VERSION:
        status = gckDEVICE_Version(Device, Interface);
        break;

    case gcvHAL_SET_TIMEOUT:
        status = gckDEVICE_SetTimeOut(Device, Interface);
        break;

    default:
        status = gcvSTATUS_NOT_SUPPORTED;
        break;
    }

    if (gcmIS_SUCCESS(status))
    {
        /* Dispatch handled in this layer. */
        Interface->status = status;
    }
    else
    {
        /* Need go through gckKERNEL dispatch. */
        if (type == gcvHARDWARE_3D || type == gcvHARDWARE_3D2D || type == gcvHARDWARE_VIP)
        {
            kernel = Device->coreInfoArray[coreIndex].kernel;
        }
        else if (type > gcvHARDWARE_INVALID && type < gcvHARDWARE_NUM_TYPES)
        {
            kernel = Device->map[type].kernels[coreIndex];
        }
        else
        {
            status = gcvSTATUS_INVALID_ARGUMENT;
            return status;
        }

        {
            status = gckKERNEL_Dispatch(kernel, Device, Interface);
        }

        /* Interface->status is handled in gckKERNEL_Dispatch(). */
    }

    return status;
}

#if VIVANTE_PROFILER
gceSTATUS
gckDEVICE_Profiler_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_PROFILER_INTERFACE_PTR Interface
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckKERNEL kernel;
    gctUINT32 coreIndex = Interface->coreIndex;

    kernel = Device->coreInfoArray[coreIndex].kernel;

    /* Dispatch on profiler command. */
    switch (Interface->command)
    {
    case gcvHAL_READ_ALL_PROFILE_REGISTERS_PART1:
        /* Read profile data according to the context. */
        gcmkONERROR(
            gckHARDWARE_QueryContextProfile(
                kernel->hardware,
                kernel->profiler.profileCleanRegister,
                &Interface->u.RegisterProfileData_part1.Counters,
                gcvNULL));

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_READ_ALL_PROFILE_REGISTERS_PART2:
        /* Read profile data according to the context. */
        gcmkONERROR(
            gckHARDWARE_QueryContextProfile(
                kernel->hardware,
                kernel->profiler.profileCleanRegister,
                gcvNULL,
                &Interface->u.RegisterProfileData_part2.Counters));

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_GET_PROFILE_SETTING:
        /* Get profile setting */
        Interface->u.GetProfileSetting.enable = kernel->profiler.profileEnable;

        Interface->u.GetProfileSetting.profileMode = kernel->profiler.profileMode;

        if (kernel->profiler.profileMode == gcvPROFILER_PROBE_MODE)
        {
            Interface->u.GetProfileSetting.probeMode = kernel->profiler.probeMode;
        }

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_SET_PROFILE_SETTING:
        /* Set profile setting */
        kernel->profiler.profileEnable = Interface->u.SetProfileSetting.enable;

        if(kernel->profiler.profileEnable)
        {
            kernel->profiler.profileMode = Interface->u.SetProfileSetting.profileMode;

            gcmkONERROR(gckHARDWARE_SetGpuProfiler(
                kernel->hardware,
                gcvTRUE
                ));

            if (kernel->profiler.profileMode == gcvPROFILER_AHB_MODE)
            {
                gcmkONERROR(gckHARDWARE_InitProfiler(kernel->hardware));
            }
            else if (kernel->profiler.profileMode == gcvPROFILER_PROBE_MODE)
            {
                kernel->profiler.probeMode = Interface->u.SetProfileSetting.probeMode;
            }
            else
            {
                gcmkPRINT("unknown profileMode argument");
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }
        }
        else
        {
            gcmkONERROR(gckHARDWARE_SetGpuProfiler(
                kernel->hardware,
                gcvFALSE
                ));
        }

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_READ_PROFILER_REGISTER_SETTING:
        kernel->profiler.profileCleanRegister = Interface->u.SetProfilerRegisterClear.bclear;
        status = gcvSTATUS_OK;
        break;

    default:
        /* Invalid command. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

OnError:
    /* Save status. */
    Interface->status = status;

    /* Return the status. */
    return status;
}
#endif

gceSTATUS
gckDEVICE_GetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU *Mmu
    )
{
    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(Type < gcvHARDWARE_NUM_TYPES);

    *Mmu = Device->mmus[Type];

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_SetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU Mmu
    )
{
    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(Type < gcvHARDWARE_NUM_TYPES);

    Device->mmus[Type] = Mmu;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_MapInTrustApplicaiton(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 GPUAddress,
    IN gctSIZE_T PageCount
    )
{
    gceSTATUS status;
    gctUINT32 * physicalArrayLogical = gcvNULL;
    gctSIZE_T bytes;
    gctPOINTER logical = Logical;
    gctUINT32 i;
    gctSIZE_T pageSize;
    gctUINT32 pageMask;

    gcmkHEADER();

    gcmkVERIFY_OK(gckOS_GetPageSize(Kernel->os, &pageSize));

    pageMask = (gctUINT32)pageSize - 1;

    bytes = PageCount * gcmSIZEOF(gctUINT32);

     gcmkONERROR(gckOS_Allocate(
        Kernel->os,
        bytes,
        (gctPOINTER *)&physicalArrayLogical
        ));

    /* Fill in physical array. */
    for (i = 0; i < PageCount; i++)
    {
        gctPHYS_ADDR_T phys;
        status = gckOS_GetPhysicalFromHandle(
            Kernel->os,
            Physical,
            i * 4096,
            &phys
            );

        if (status == gcvSTATUS_NOT_SUPPORTED)
        {
            gcmkONERROR(gckOS_GetPhysicalAddress(
                Kernel->os,
                logical,
                &phys
                ));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, phys, &phys));
        }

        phys &= ~pageMask;

        gcmkSAFECASTPHYSADDRT(physicalArrayLogical[i], phys);

        logical = (gctUINT8_PTR)logical + 4096;
    }

    gcmkONERROR(gckKERNEL_SecurityMapMemory(
        Kernel,
        physicalArrayLogical,
        0,
        (gctUINT32)PageCount,
        &GPUAddress
        ));

    gcmkVERIFY_OK(gckOS_Free(
        Kernel->os,
        physicalArrayLogical
        ));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (physicalArrayLogical != gcvNULL)
    {
        gcmkVERIFY_OK(gckOS_Free(
            Kernel->os,
            (gctPOINTER)physicalArrayLogical
            ));
    }

    gcmkFOOTER();
    return status;
}
#endif

#if gcdENABLE_MP_SWITCH
gceSTATUS
gckKERNEL_DetectMpModeSwitch(
    IN gckKERNEL Kernel,
    IN gceMULTI_PROCESSOR_MODE Mode,
    OUT gctUINT32 *SwitchMpMode
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 switchMpMode = gcvMP_MODE_NO_SWITCH;
    gctUINT32 count = 0;

    gcmkHEADER_ARG("Kernel=%p Mode=%x", Kernel, Mode);

    gcmkONERROR(gckOS_SwitchCoreCount(Kernel->os, &count));

    if (count == 1 && Mode != gcvMP_MODE_INDEPENDENT)
    {
        switchMpMode = gcvMP_MODE_SWITCH_TO_SINGLE;
    }
    else if (count > 1 && Mode == gcvMP_MODE_INDEPENDENT)
    {

        switchMpMode = gcvMP_MODE_SWITCH_TO_MULTI;
    }

    *SwitchMpMode = switchMpMode;

    gcmkFOOTER_ARG("*SwitchMpMode=%x", *SwitchMpMode);
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}
#endif


