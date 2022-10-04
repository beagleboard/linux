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
#include "gc_hal_kernel_context.h"

#define _GC_OBJ_ZONE            gcvZONE_COMMAND

/******************************************************************************\
********************************* Support Code *********************************
\******************************************************************************/

/*******************************************************************************
**
**  _NewQueue
**
**  Allocate a new command queue.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object.
**
**      gctBOOL Stalled
**          Indicate if hardware is stalled already.
**
**  OUTPUT:
**
**      gckCOMMAND Command
**          gckCOMMAND object has been updated with a new command queue.
*/
static gceSTATUS
_NewQueue(
    IN OUT gckCOMMAND Command,
    IN gctBOOL Stalled
    )
{
    gceSTATUS status;
    gctINT currentIndex, newIndex;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Switch to the next command buffer. */
    currentIndex = Command->index;
    newIndex     = (currentIndex + 1) % gcdCOMMAND_QUEUES;

    /* Wait for availability. */
    gcmkDUMP(Command->os, "#[kernel.waitsignal]");

    gcmkONERROR(gckOS_WaitSignal(
        Command->os,
        Command->queues[newIndex].signal,
        gcvFALSE,
        gcvINFINITE
        ));

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    if (newIndex < currentIndex)
    {
        Command->wrapCount += 1;

        gcmkTRACE_ZONE_N(
            gcvLEVEL_INFO, gcvZONE_COMMAND,
            2 * 4,
            "%s(%d): queue array wrapped around.\n",
            __FUNCTION__, __LINE__
            );
    }

    gcmkTRACE_ZONE_N(
        gcvLEVEL_INFO, gcvZONE_COMMAND,
        3 * 4,
        "%s(%d): total queue wrap arounds %d.\n",
        __FUNCTION__, __LINE__, Command->wrapCount
        );

    gcmkTRACE_ZONE_N(
        gcvLEVEL_INFO, gcvZONE_COMMAND,
        3 * 4,
        "%s(%d): switched to queue %d.\n",
        __FUNCTION__, __LINE__, newIndex
        );
#endif

    /* Update gckCOMMAND object with new command queue. */
    Command->index    = newIndex;
    Command->newQueue = gcvTRUE;
    Command->videoMem = Command->queues[newIndex].videoMem;
    Command->logical  = Command->queues[newIndex].logical;
    Command->address  = Command->queues[newIndex].address;
    Command->offset   = 0;

    if (currentIndex != -1)
    {
        if (Stalled)
        {
            gckOS_Signal(
                Command->os,
                Command->queues[currentIndex].signal,
                gcvTRUE
                );
        }
        else
        {
            /* Mark the command queue as available. */
            gcmkONERROR(gckEVENT_Signal(
                Command->kernel->eventObj,
                Command->queues[currentIndex].signal,
                gcvKERNEL_COMMAND
                ));
        }
    }

    /* Success. */
    gcmkFOOTER_ARG("Command->index=%d", Command->index);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_IncrementCommitAtom(
    IN gckCOMMAND Command,
    IN gctBOOL Increment
    )
{
    gceSTATUS status;
    gckHARDWARE hardware;
    gctINT32 atomValue;
    gctBOOL powerAcquired = gcvFALSE;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Extract the gckHARDWARE and gckEVENT objects. */
    hardware = Command->kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Grab the power mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        Command->os, hardware->powerMutex, gcvINFINITE
        ));
    powerAcquired = gcvTRUE;

    /* Increment the commit atom. */
    if (Increment)
    {
        gcmkONERROR(gckOS_AtomIncrement(
            Command->os, Command->atomCommit, &atomValue
            ));
    }
    else
    {
        gcmkONERROR(gckOS_AtomDecrement(
            Command->os, Command->atomCommit, &atomValue
            ));
    }

    /* Release the power mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(
        Command->os, hardware->powerMutex
        ));
    powerAcquired = gcvFALSE;

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (powerAcquired)
    {
        /* Release the power mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(
            Command->os, hardware->powerMutex
            ));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CheckFlushMMU(
    IN gckCOMMAND Command,
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 oldValue;
    gctBOOL pause = gcvFALSE;

    gctUINT8_PTR pointer;
    gctUINT32 address;
    gctUINT32 eventBytes;
    gctUINT32 endBytes;
    gctUINT32 bufferSize;
    gctUINT32 executeBytes;

    gckOS_AtomicExchange(Command->os,
                         Hardware->pageTableDirty[gcvENGINE_RENDER],
                         0,
                         &oldValue);

    if (oldValue)
    {
        /* Page Table is upated, flush mmu before commit. */
        gctUINT32 flushBytes;

        gcmkONERROR(gckHARDWARE_FlushMMU(
            Hardware,
            gcvNULL,
            gcvINVALID_ADDRESS,
            0,
            &flushBytes
            ));

        gcmkONERROR(gckCOMMAND_Reserve(
            Command,
            flushBytes,
            (gctPOINTER *)&pointer,
            &bufferSize
            ));

        /* Pointer to reserved address. */
        address = Command->address  + Command->offset;

        /*
         * subsequent 8 bytes are wait-link commands.
         * Set more existed bytes for now.
         */
        gcmkONERROR(gckHARDWARE_FlushMMU(
            Hardware,
            pointer,
            address,
            (bufferSize - flushBytes),
            &flushBytes
            ));

        gcmkONERROR(gckCOMMAND_Execute(Command, flushBytes));

        if ((oldValue & gcvPAGE_TABLE_DIRTY_BIT_FE)
          && (!Hardware->stallFEPrefetch)
        )
        {
            pause = gcvTRUE;
        }
    }

    if (pause)
    {
        /* Query size. */
        gcmkONERROR(gckWLFE_Event(Hardware, gcvNULL, 0, gcvKERNEL_PIXEL, &eventBytes));
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));

        executeBytes = eventBytes + endBytes;

        /* Reserve space. */
        gcmkONERROR(gckCOMMAND_Reserve(
            Command,
            executeBytes,
            (gctPOINTER *)&pointer,
            &bufferSize
            ));

        /* Pointer to reserved address. */
        address = Command->address  + Command->offset;

        /* Append EVENT(29). */
        gcmkONERROR(gckWLFE_Event(
            Hardware,
            pointer,
            29,
            gcvKERNEL_PIXEL,
            &eventBytes
            ));

        /* Append END. */
        pointer += eventBytes;
        address += eventBytes;

        gcmkONERROR(gckWLFE_End(Hardware, pointer, address, &endBytes));

        gcmkONERROR(gckCOMMAND_Execute(Command, executeBytes));
    }

    return gcvSTATUS_OK;
OnError:
    return status;
}

/* WaitLink FE only. */
static gceSTATUS
_DummyDraw(
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;
    gckHARDWARE hardware = Command->kernel->hardware;

    gctUINT8_PTR pointer;
    gctUINT32 bufferSize;

    gctUINT32 dummyDrawBytes;
    gceDUMMY_DRAW_TYPE dummyDrawType = gcvDUMMY_DRAW_INVALID;

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_FE_NEED_DUMMYDRAW))
    {
        dummyDrawType = gcvDUMMY_DRAW_GC400;
    }

    if (!gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_USC_DEFER_FILL_FIX) &&
        gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_USC))
    {
        dummyDrawType = gcvDUMMY_DRAW_V60;
    }

    if (dummyDrawType != gcvDUMMY_DRAW_INVALID)
    {
        gckHARDWARE_DummyDraw(hardware, gcvNULL, Command->queues[0].address, dummyDrawType, &dummyDrawBytes);

        /* Reserve space. */
        gcmkONERROR(gckCOMMAND_Reserve(
            Command,
            dummyDrawBytes,
            (gctPOINTER *)&pointer,
            &bufferSize
            ));

        gckHARDWARE_DummyDraw(hardware, pointer, Command->queues[0].address, dummyDrawType, &dummyDrawBytes);

        gcmkONERROR(gckCOMMAND_Execute(Command, dummyDrawBytes));
    }

    return gcvSTATUS_OK;
OnError:
    return status;
}

/*
 * Wait pending semaphores.
 *
 * next == free: full, no more semaphores.
 * (free + 1) = next: empty
 */
static gcmINLINE gceSTATUS
_WaitPendingMcfeSema(
    gckCOMMAND Command
    )
{
    gceSTATUS status;
    const gctUINT count = gcmCOUNTOF(Command->pendingSema);
    gctUINT32 nextFreePos;
    gctUINT32 timeout = gcvINFINITE;

    gcmkHEADER_ARG("freePendingPos=%u nextPendingPos=%u",
                   Command->freePendingPos, Command->nextPendingPos);

    nextFreePos = (Command->freePendingPos + 1) % count;

    if (nextFreePos == Command->nextPendingPos)
    {
        /* No pendings. */
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    while (nextFreePos != Command->nextPendingPos)
    {
        /* Timeout is infinite in the first to at least free one slot. */
        status = gckOS_WaitSignal(Command->os,
                                  Command->pendingSema[nextFreePos].signal,
                                  gcvFALSE,
                                  timeout);

        if (status == gcvSTATUS_TIMEOUT)
        {
            /* Timeout out is OK for later pendings. */
            break;
        }

        gcmkONERROR(status);

        /* Do not wait for the later slots. */
        timeout = 0;

        /* More free semaphores can be used. */
        Command->freeSemaId = Command->pendingSema[nextFreePos].semaId;

        /* Advance free pos. */
        Command->freePendingPos = nextFreePos;
        nextFreePos = (nextFreePos + 1) % count;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gctUINT32
_GetFreeMcfeSemaNum(
    gckCOMMAND Command
    )
{
    gctUINT32 num = 0;

    if (Command->nextSemaId <= Command->freeSemaId)
    {
        num = Command->freeSemaId - Command->nextSemaId;
    }
    else
    {
        num = Command->totalSemaId - Command->nextSemaId + Command->freeSemaId;
    }

    return num;
}

/*
 * Get next semaphore id in semaphore ring.
 *
 * There are semaMinThreshhold semaphores are reserved for system operations,
 * such as _SyncToSystemChannel etc.
 * The rest semaphores are regular ones.
 */
static gcmINLINE gceSTATUS
_GetNextMcfeSemaId(
    gckCOMMAND Command,
    gctBOOL regularSema,
    gctUINT32 * SemaId
    )
{
    gctUINT32 freeSemaNum = 0;

    gceSTATUS status = gcvSTATUS_OK;

    /*
     * See the comments in struct definition.
     * wait when run out of semaphores.
     */
    freeSemaNum = _GetFreeMcfeSemaNum(Command);

    if ((regularSema && (freeSemaNum <= Command->semaMinThreshhold)) ||
        (!regularSema && (freeSemaNum == 0)))
    {
        gcmkONERROR(_WaitPendingMcfeSema(Command));
    }

    gcmkASSERT(Command->nextSemaId != Command->freeSemaId);

    /* Output the semaphore ID. */
    *SemaId = Command->nextSemaId;

    /* Advance to next. */
    if (++Command->nextSemaId == Command->totalSemaId)
    {
        Command->nextSemaId = 0;
    }

OnError:
    return status;
}

/*
 * Get next pending pos in pending semaphore tracking ring.
 */
static gcmINLINE gceSTATUS
_GetNextPendingPos(
    gckCOMMAND Command,
    gctUINT32 * Pos
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    /* Wait when out of pending ring. */
    if (Command->nextPendingPos == Command->freePendingPos)
    {
        /* Run out of pending semaphore tracking ring. */
        gcmkONERROR(_WaitPendingMcfeSema(Command));
    }

    gcmkASSERT(Command->nextPendingPos != Command->freePendingPos);

    *Pos = Command->nextPendingPos;

    /* Advance to next. */
    if (++Command->nextPendingPos == gcmCOUNTOF(Command->pendingSema))
    {
        Command->nextPendingPos = 0;
    }

OnError:
    return status;
}

/*
 * Sync specific channels to system channel.
 * Record semaphores to pendingSema structure.
 * 'SyncChannel' is cleared upon function return.
 */
static gceSTATUS
_SyncToSystemChannel(
    gckCOMMAND Command,
    gctUINT64 SyncChannel[2],
    gctBOOL BroadcastCommit
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckKERNEL kernel = Command->kernel;
    gckHARDWARE hardware = kernel->hardware;
    gctUINT8 semaId[128];
    gctUINT32 semaCount = 0;
    gctUINT32 reqBytes = 0;
    gctUINT32 bytes = 0;
    gctUINT8_PTR buffer;
    gctUINT32 i;
    gctUINT32 pri;
    gctBOOL commitEntered = gcvFALSE;

    /* Ignore system channel. */
    SyncChannel[0] &= ~((gctUINT64)1ull);
    SyncChannel[1] &= ~((gctUINT64)1ull);

    if (!SyncChannel[0] && !SyncChannel[1])
    {
        return gcvSTATUS_OK;
    }

    if (BroadcastCommit)
    {
        /* Acquire the command queue. */
        gcmkONERROR(gckCOMMAND_EnterCommit(Command, gcvFALSE));
        commitEntered = gcvTRUE;
    }

    /* Query SendSemaphore command size. */
    gckMCFE_SendSemaphore(hardware, gcvNULL, 0, &reqBytes);

    for (pri = 0; pri < 2; pri++)
    {
        for (i = 1; i < 64 && SyncChannel[pri]; i++)
        {
            gctUINT32 id;

            if (!(SyncChannel[pri] & (1ull << i)))
            {
                continue;
            }

            /* Get a free semaphore id. */
            gcmkONERROR(_GetNextMcfeSemaId(Command, gcvFALSE, &id));
            semaId[semaCount++] = (gctUINT8)id;

            gcmkONERROR(gckCOMMAND_Reserve(Command, reqBytes, (gctPOINTER *)&buffer, &bytes));

            /* Send semaphore executed in specified channel. */
            gckMCFE_SendSemaphore(hardware, buffer, id, &bytes);

            gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(Command, pri, i, reqBytes));

            /* Remove the sync'ed channel. */
            SyncChannel[pri] &= ~(1ull << i);
        }
    }

    if (semaCount > 0)
    {
        gctUINT32 pos = 0;
        gckEVENT eventObj = kernel->eventObj;
        gctUINT32 bufferLen = 0;

        /* Query WaitSemaphore command size. */
        gckMCFE_WaitSemaphore(hardware, gcvNULL, 0, &reqBytes);
        reqBytes *= semaCount;

        gcmkONERROR(gckCOMMAND_Reserve(Command, reqBytes, (gctPOINTER *)&buffer, &bufferLen));

        for (i = 0; i < semaCount; i++)
        {
            bytes = bufferLen;

            /* Wait semaphores executed in fixed system channel. */
            gckMCFE_WaitSemaphore(hardware, buffer, semaId[i], &bytes);

            buffer    += bytes;
            bufferLen -= bytes;
        }

        gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(Command, 0, 0, reqBytes));

        if (BroadcastCommit)
        {
            /* Release the command queue. */
            gcmkONERROR(gckCOMMAND_ExitCommit(Command, gcvFALSE));
            commitEntered = gcvFALSE;
        }

        /* Now upload the pending semaphore tracking ring. */
        gcmkONERROR(_GetNextPendingPos(Command, &pos));

        /* Update latest pending semaphore id. */
        Command->pendingSema[pos].semaId = (gctUINT32)semaId[semaCount - 1];

        /* Send the signal by event. */
        gcmkONERROR(gckEVENT_Signal(
            eventObj,
            Command->pendingSema[pos].signal,
            gcvKERNEL_PIXEL
            ));

        gcmkONERROR(gckEVENT_Submit(
            eventObj,
            gcvTRUE,
            gcvFALSE,
            BroadcastCommit
            ));
    }

OnError:
    if (commitEntered)
    {
        /* Release the command queue mutex. */
        gcmkVERIFY_OK(gckCOMMAND_ExitCommit(Command, gcvFALSE));
    }

    return status;
}

static gcmINLINE gceSTATUS
_SyncFromSystemChannel(
    gckCOMMAND Command,
    gctBOOL Priority,
    gctUINT32 ChannelId
    )
{
    gceSTATUS status;
    gckHARDWARE hardware = Command->kernel->hardware;
    gctUINT32 reqBytes = 0;
    gctUINT32 bytes = 0;
    gctUINT8_PTR buffer;
    gctUINT32 id;

    gcmkHEADER_ARG("priority=%d channelId=%d", Priority, ChannelId);

    if (!(Command->syncChannel[Priority ? 1 : 0] & (1ull << ChannelId)))
    {
        /* No need to sync. */
        return gcvSTATUS_OK;
    }

    /* Get a semaphore. */
    gcmkONERROR(_GetNextMcfeSemaId(Command, gcvFALSE, &id));

    /* Send the semaphore in system channel. */
    gckMCFE_SendSemaphore(hardware, gcvNULL, 0, &reqBytes);

    gcmkONERROR(gckCOMMAND_Reserve(
        Command,
        reqBytes,
        (gctPOINTER *)&buffer,
        &bytes
        ));

    gckMCFE_SendSemaphore(hardware, buffer, id, &bytes);

    gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(Command, gcvFALSE, 0, reqBytes));

    /* Wait the semaphore in specific channel. */
    gckMCFE_WaitSemaphore(hardware, gcvNULL, 0, &reqBytes);

    gcmkONERROR(gckCOMMAND_Reserve(
        Command,
        reqBytes,
        (gctPOINTER *)&buffer,
        &bytes
        ));

    gckMCFE_WaitSemaphore(hardware, buffer, id, &bytes);

    gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(
        Command,
        Priority,
        ChannelId,
        reqBytes
        ));

    /* Clear the sync flag. */
    Command->syncChannel[Priority ? 1 : 0] &= ~(1ull << ChannelId);

    gcmkFOOTER_NO();
    /* Can not track the semaphore here. */
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CheckFlushMcfeMMU(
    IN gckCOMMAND Command,
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 oldValue;
    gctUINT32 reqBytes;
    gctUINT32 bytes;
    gctUINT8_PTR buffer;
    gctUINT32 id = 0;

    gckOS_AtomicExchange(Command->os,
                         Hardware->pageTableDirty[gcvENGINE_RENDER],
                         0,
                         &oldValue);

    if (!oldValue)
    {
        return gcvSTATUS_OK;
    }

    /*
     * This sync is earlier than in Commit, see comments in Commit.
     * Blindly sync dirty other channels to the system channel here.
     */
    gcmkONERROR(_SyncToSystemChannel(Command, Command->dirtyChannel, gcvFALSE));

    /* Query flush Mcfe MMU cache command bytes. */
    gcmkONERROR(gckHARDWARE_FlushMcfeMMU(Hardware, gcvNULL, &reqBytes));

    /* Query semaphore command bytes. */
    gcmkONERROR(
        gckMCFE_SendSemaphore(Hardware, gcvNULL, 0, &bytes));
    reqBytes += bytes;

    gcmkONERROR(
        gckMCFE_WaitSemaphore(Hardware, gcvNULL, 0, &bytes));
    reqBytes += bytes;

    /* Get a semaphore. */
    gcmkONERROR(_GetNextMcfeSemaId(Command, gcvFALSE, &id));

    /* Request command buffer for system channel. */
    gcmkONERROR(gckCOMMAND_Reserve(
        Command,
        reqBytes,
        (gctPOINTER *)&buffer,
        &bytes
        ));

    /* Do flush mmu. */
    gckHARDWARE_FlushMcfeMMU(Hardware, buffer, &bytes);
    buffer += bytes;

    /* Send and wait semaphore in the system channel itself. */
    gcmkONERROR(gckMCFE_SendSemaphore(Hardware, buffer, id, &bytes));
    buffer += bytes;

    gcmkONERROR(gckMCFE_WaitSemaphore(Hardware, buffer, id, &bytes));

    /* Execute flush mmu and send semaphores. */
    gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(Command, 0, 0, reqBytes));

    /* Need sync from system channel. */
    Command->syncChannel[0] = ~1ull;
    Command->syncChannel[1] = ~1ull;

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*
 * Find sema id from the map.
 * Returns semaphore Id, ie the array index of semaHandleMap.
 * -1 if not found.
 */
static gcmINLINE gctINT32
_FindSemaIdFromMap(
    IN gckCOMMAND Command,
    IN gctUINT32 SemaHandle
    )
{
    gctUINT32 semaId = Command->nextSemaId;

    do
    {
        /*
         * Only need to check semaId between signaledId (inclusive) and
         * nextSemaId (exclusive).
         */
        semaId = (semaId == 0) ? (Command->totalSemaId - 1) : (semaId - 1);

        if (Command->semaHandleMap[semaId] == SemaHandle)
        {
            return (gctINT32)semaId;
        }
    }
    while (semaId != Command->freeSemaId);

    return -1;
}

/* Patch item handler typedef. */
typedef gceSTATUS
(* PATCH_ITEM_HANDLER)(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gctPOINTER Patch,
    IN gcsPATCH_LIST_VARIABLE * PatchListVar
    );

static const gctUINT32 _PatchItemSize[] =
{
    0,
    (gctUINT32)sizeof(gcsHAL_PATCH_VIDMEM_ADDRESS),
    (gctUINT32)sizeof(gcsHAL_PATCH_MCFE_SEMAPHORE),
    (gctUINT32)sizeof(gcsHAL_PATCH_VIDMEM_TIMESTAMP),
};

static gceSTATUS
_HandleVidmemAddressPatch(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gctPOINTER Patch,
    IN gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsHAL_PATCH_VIDMEM_ADDRESS * patch = Patch;

    gcmkHEADER_ARG("Command=%p location=0x%x node=0x%x offset=%x",
                   Command, patch->location, patch->node, patch->offset);

    (void)status;
    (void)patch;

    gcmkFOOTER();
    return gcvSTATUS_OK;
}

static gceSTATUS
_HandleMCFESemaphorePatch(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gctPOINTER Patch,
    IN gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gckHARDWARE hardware = Command->kernel->hardware;
    gctINT32 index;
    gctUINT32 semaId;
    gceSTATUS status;
    gctUINT32 bytes = 8;
    gctUINT32 buffer[2];
    gctUINT8_PTR location;
    gcsHAL_PATCH_MCFE_SEMAPHORE * patch = (gcsHAL_PATCH_MCFE_SEMAPHORE *)Patch;

    gcmkHEADER_ARG("Command=%p location=0x%x semaHandle=%d",
                   Command, patch->location, patch->semaHandle);

    index = _FindSemaIdFromMap(Command, patch->semaHandle);

    if (index < 0)
    {
        status = _GetNextMcfeSemaId(Command, gcvTRUE, &semaId);

        if (gcmIS_ERROR(status))
        {
            gcmkONERROR(_SyncToSystemChannel(Command, Command->dirtyChannel, gcvFALSE));

            gcmkONERROR(_GetNextMcfeSemaId(Command, gcvTRUE, &semaId));
        }

        Command->semaHandleMap[semaId] = patch->semaHandle;
    }
    else
    {
        semaId = (gctUINT32)index;

        /* One send must match one wait, will assign new id next time. */
        Command->semaHandleMap[semaId] = 0;
    }

    if (patch->sendSema)
    {
        gcmkONERROR(gckMCFE_SendSemaphore(hardware, buffer, semaId, &bytes));
    }
    else
    {
        gcmkONERROR(gckMCFE_WaitSemaphore(hardware, buffer, semaId, &bytes));
    }

    gcmkASSERT(bytes == 8);

    location = gcmUINT64_TO_PTR(CommandBuffer->logical + patch->location);

    /* Patch the command buffer. */
    gckOS_WriteMemory(Command->os, location, buffer[0]);
    gckOS_WriteMemory(Command->os, location + 4, buffer[1]);


    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_HandleTimestampPatch(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gctPOINTER Patch,
    IN gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status;
    gctUINT32 processID;
    gckVIDMEM_NODE videoMem = gcvNULL;
    gcsHAL_PATCH_VIDMEM_TIMESTAMP * patch = Patch;
    gceENGINE engine = Command->feType == gcvHW_FE_ASYNC ? gcvENGINE_BLT
                     : gcvENGINE_RENDER;

    gcmkHEADER_ARG("Command=%p node=0x%x", Command, patch->handle);

    /* Get the current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Command->kernel,
                                processID,
                                patch->handle,
                                &videoMem));

    gcmkVERIFY_OK(gckVIDMEM_NODE_Reference(Command->kernel, videoMem));

    /* Touch video memory node. */
    gcmkVERIFY_OK(
        gckVIDMEM_NODE_SetCommitStamp(Command->kernel,
                                      engine,
                                      videoMem,
                                      Command->commitStamp));

    if ((engine == gcvENGINE_RENDER) && Command->kernel->asyncCommand)
    {
        /* Find the latest timestamp of the nodes used in async FE. */
        gctUINT64 stamp = 0;

        /* Get stamp touched async command buffer. */
        gcmkVERIFY_OK(
            gckVIDMEM_NODE_GetCommitStamp(Command->kernel,
                                          gcvENGINE_BLT,
                                          videoMem,
                                          &stamp));

        /* Find latest one. */
        if (PatchListVar->maxAsyncTimestamp < stamp)
        {
            PatchListVar->maxAsyncTimestamp = stamp;
        }
    }

OnError:
    if (videoMem)
    {
        gckVIDMEM_NODE_Dereference(Command->kernel, videoMem);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static const PATCH_ITEM_HANDLER patchHandler[] =
{
    gcvNULL,
    _HandleVidmemAddressPatch,
    _HandleMCFESemaphorePatch,
    _HandleTimestampPatch,
};
PATCH_ITEM_HANDLER handler;

static gceSTATUS
_HandlePatchListSingle(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gcsHAL_PATCH_LIST * PatchList,
    IN gctBOOL NeedCopy,
    IN gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status;
    /* 256 bytes for storage. */
    gctUINT64 storage[32];
    gctPOINTER kArray = gcvNULL;
    gctPOINTER userPtr = gcvNULL;
    gctUINT32 index = 0;
    gctUINT32 count = 0;
    gctUINT32 itemSize = 0;
    gctUINT32 batchCount = 0;

    gcmkHEADER_ARG("Command=%p CommandBuffer=%p PatchList=%p type=%d",
                   Command, CommandBuffer, PatchList, PatchList->type);

    if (PatchList->type >= gcmCOUNTOF(_PatchItemSize) || PatchList->type >= gcmCOUNTOF(patchHandler))
    {
        /* Exceeds buffer max size. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    itemSize = _PatchItemSize[PatchList->type];

    batchCount = (gctUINT32)(sizeof(storage) / itemSize);

    handler = patchHandler[PatchList->type];

    while (index < PatchList->count)
    {
        gctUINT i;
        gctUINT8_PTR ptr;

        /* Determine batch count, don't handle too many in one batch. */
        count = PatchList->count - index;

        if (count > batchCount)
        {
            count = batchCount;
        }

        userPtr = gcmUINT64_TO_PTR(PatchList->patchArray + itemSize * index);

        /* Copy/map a patch array batch from user. */
        if (NeedCopy)
        {
            kArray = storage;

            status = gckOS_CopyFromUserData(
                Command->os,
                kArray,
                userPtr,
                itemSize * count
                );
        }
        else
        {
            status = gckOS_MapUserPointer(
                Command->os,
                userPtr,
                itemSize * count,
                (gctPOINTER *)&kArray
                );
        }

        if (gcmIS_ERROR(status))
        {
            userPtr = gcvNULL;
            gcmkONERROR(status);
        }

        /* Advance to next batch. */
        index += count;

        ptr = (gctUINT8_PTR)kArray;

        for (i = 0; i < count; i++)
        {
            /* Call handler. */
            gcmkONERROR(
                handler(Command, CommandBuffer, ptr, PatchListVar));

            /* Advance to next patch. */
            ptr += itemSize;
        }

        /* Unmap user pointer if mapped. */
        if (!NeedCopy)
        {
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Command->os,
                userPtr,
                itemSize * count,
                kArray
                ));
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (!NeedCopy && userPtr)
    {
        gcmkVERIFY_OK(gckOS_UnmapUserPointer(
            Command->os,
            userPtr,
            itemSize * count,
            kArray
            ));

        userPtr = gcvNULL;
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_HandlePatchList(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    OUT gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status;
    gctBOOL needCopy = gcvFALSE;
    gcsHAL_PATCH_LIST storage;
    gcsHAL_PATCH_LIST * kPatchList = gcvNULL;
    gctPOINTER userPtr = gcmUINT64_TO_PTR(CommandBuffer->patchHead);

    gcmkHEADER_ARG("Command=%p CommandBuffer=%p", Command, CommandBuffer);

    /* Check wehther we need to copy the structures or not. */
    gcmkONERROR(gckOS_QueryNeedCopy(Command->os, 0, &needCopy));

    while (userPtr)
    {
        gctUINT64 next;

        /* Copy/map a patch from user. */
        if (needCopy)
        {
            kPatchList = &storage;

            status = gckOS_CopyFromUserData(
                Command->os,
                kPatchList,
                userPtr,
                sizeof(gcsHAL_PATCH_LIST)
                );
        }
        else
        {
            status = gckOS_MapUserPointer(
                Command->os,
                userPtr,
                sizeof(gcsHAL_PATCH_LIST),
                (gctPOINTER *)&kPatchList
                );
        }

        if (gcmIS_ERROR(status))
        {
            userPtr = gcvNULL;
            gcmkONERROR(status);
        }

        /* Do handle patch. */
        gcmkASSERT(kPatchList->type < gcvHAL_PATCH_TYPE_COUNT);

        gcmkONERROR(
            _HandlePatchListSingle(Command,
                                   CommandBuffer,
                                   kPatchList,
                                   needCopy,
                                   PatchListVar));

        next = kPatchList->next;

        /* Unmap user pointer if mapped. */
        if (!needCopy)
        {
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Command->os,
                userPtr,
                sizeof(gcsHAL_PATCH_LIST),
                kPatchList
                ));
        }

        /* Advance to next patch from user. */
        userPtr = gcmUINT64_TO_PTR(next);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (!needCopy && userPtr)
    {
        gcmkVERIFY_OK(gckOS_UnmapUserPointer(
            Command->os,
            userPtr,
            sizeof(gcsHAL_PATCH_LIST),
            kPatchList
            ));
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_WaitForAsyncCommandStamp(
    IN gckCOMMAND Command,
    IN gctUINT64 Stamp
    )
{
    gctUINT32 bytes;
    gceSTATUS status;
    gctUINT32 fenceAddress;
    gctUINT32 bufferSize;
    gctPOINTER pointer;
    gckCOMMAND asyncCommand = Command->kernel->asyncCommand;

    gcmkHEADER_ARG("Stamp = 0x%llx", Stamp);

    if (*(gctUINT64 *)asyncCommand->fence->logical >= Stamp)
    {
        /* Already satisfied, skip. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    fenceAddress = asyncCommand->fence->address;

    gcmkONERROR(gckHARDWARE_WaitFence(
        Command->kernel->hardware,
        gcvNULL,
        Stamp,
        fenceAddress,
        &bytes
        ));

    gcmkONERROR(gckCOMMAND_Reserve(
        Command,
        bytes,
        &pointer,
        &bufferSize
        ));

    gcmkONERROR(gckHARDWARE_WaitFence(
        Command->kernel->hardware,
        pointer,
        Stamp,
        fenceAddress,
        &bytes
        ));

    gcmkONERROR(gckCOMMAND_Execute(Command, bytes));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/******************************************************************************\
****************************** gckCOMMAND API Code ******************************
\******************************************************************************/

/*******************************************************************************
**
**  gckCOMMAND_Construct
**
**  Construct a new gckCOMMAND object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      gckCOMMAND * Command
**          Pointer to a variable that will hold the pointer to the gckCOMMAND
**          object.
*/
gceSTATUS
gckCOMMAND_Construct(
    IN gckKERNEL Kernel,
    IN gceHW_FE_TYPE FeType,
    OUT gckCOMMAND * Command
    )
{
    gckOS os;
    gckCOMMAND command = gcvNULL;
    gceSTATUS status;
    gctINT i;
    gctPOINTER pointer = gcvNULL;
    gctSIZE_T pageSize;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Command != gcvNULL);

    /* Extract the gckOS object. */
    os = Kernel->os;

    /* Allocate the gckCOMMAND structure. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(struct _gckCOMMAND), &pointer));
    command = pointer;

    /* Reset the entire object. */
    gcmkONERROR(gckOS_ZeroMemory(command, gcmSIZEOF(struct _gckCOMMAND)));

    /* Initialize the gckCOMMAND object.*/
    command->object.type    = gcvOBJ_COMMAND;
    command->kernel         = Kernel;
    command->os             = os;

    command->feType         = FeType;

    /* Get the command buffer requirements. */
    gcmkONERROR(gckHARDWARE_QueryCommandBuffer(
        Kernel->hardware,
        gcvENGINE_RENDER,
        &command->alignment,
        gcvNULL,
        gcvNULL
        ));

    /* Create the command queue mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &command->mutexQueue));

    /* Create the context switching mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &command->mutexContext));

    /* Create the context switching mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &command->mutexContextSeq));

    /* Create the power management semaphore. */
    gcmkONERROR(gckOS_CreateSemaphore(os, &command->powerSemaphore));

    /* Create the commit atom. */
    gcmkONERROR(gckOS_AtomConstruct(os, &command->atomCommit));

    /* Get the page size from teh OS. */
    gcmkONERROR(gckOS_GetPageSize(os, &pageSize));

    gcmkSAFECASTSIZET(command->pageSize, pageSize);

    /* Get process ID. */
    gcmkONERROR(gckOS_GetProcessID(&command->kernelProcessID));

    /* Set hardware to pipe 0. */
    command->pipeSelect = gcvPIPE_INVALID;

    /* Pre-allocate the command queues. */
    for (i = 0; i < gcdCOMMAND_QUEUES; ++i)
    {
#if !gcdCAPTURE_ONLY_MODE
        gcePOOL pool = gcvPOOL_DEFAULT;
#else
        gcePOOL pool = gcvPOOL_VIRTUAL;
#endif

        gctSIZE_T size = pageSize;
        gckVIDMEM_NODE videoMem = gcvNULL;
        gctUINT32 allocFlag = 0;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag = gcvALLOC_FLAG_CACHEABLE;
#endif

        /* Allocate video memory node for command buffers. */
        gcmkONERROR(gckKERNEL_AllocateVideoMemory(
            Kernel,
            64,
            gcvVIDMEM_TYPE_COMMAND,
            allocFlag,
            &size,
            &pool,
            &videoMem
            ));

        command->queues[i].videoMem = videoMem;

        /* Lock for GPU access. */
        gcmkONERROR(gckVIDMEM_NODE_Lock(
            Kernel,
            videoMem,
            &command->queues[i].address
            ));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
            Kernel,
            videoMem,
            gcvFALSE,
            gcvFALSE,
            &command->queues[i].logical
            ));

        gcmkONERROR(gckOS_CreateSignal(
            os, gcvFALSE, &command->queues[i].signal
            ));

        gcmkONERROR(gckOS_Signal(
            os, command->queues[i].signal, gcvTRUE
            ));
    }

#if gcdRECORD_COMMAND
    gcmkONERROR(gckRECORDER_Construct(os, Kernel->hardware, &command->recorder));
#endif

    gcmkONERROR(gckFENCE_Create(
        os, Kernel, &command->fence
        ));

    /* No command queue in use yet. */
    command->index    = -1;
    command->logical  = gcvNULL;
    command->newQueue = gcvFALSE;

    /* Query mcfe semaphore count. */
    if (FeType == gcvHW_FE_MULTI_CHANNEL)
    {
        command->totalSemaId = 128;

        /* Empty sema id ring. */
        command->nextSemaId = 0;
        command->freeSemaId = command->totalSemaId - 1;

        command->semaMinThreshhold = 16;

        /* Create signals. */
        for (i = 0; i < (gctINT)gcmCOUNTOF(command->pendingSema); i++)
        {
            gcmkONERROR(gckOS_CreateSignal(
                os,
                gcvFALSE,
                &command->pendingSema[i].signal
                ));
        }

        /* Empty pending sema tracking ring. */
        command->nextPendingPos = 0;
        command->freePendingPos = gcmCOUNTOF(command->pendingSema) - 1;

        /* Allocate sema handle mapping. */
        gcmkONERROR(gckOS_Allocate(
            os,
            command->totalSemaId * sizeof(gctUINT32),
            &pointer
            ));

        command->semaHandleMap = (gctUINT32 *)pointer;

        gcmkVERIFY_OK(gckOS_ZeroMemory(
            command->semaHandleMap,
            command->totalSemaId * sizeof(gctUINT32)
            ));
    }

    /* Command is not yet running. */
    command->running = gcvFALSE;

    /* Command queue is idle. */
    command->idle = gcvTRUE;

    /* Commit stamp start from 1. */
    command->commitStamp = 1;

    command->dummyDraw = gcvTRUE;

    /* Return pointer to the gckCOMMAND object. */
    *Command = command;

    /* Success. */
    gcmkFOOTER_ARG("*Command=0x%x", *Command);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (command != gcvNULL)
    {
        gcmkVERIFY_OK(gckCOMMAND_Destroy(command));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_Destroy
**
**  Destroy an gckCOMMAND object.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object to destroy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Destroy(
    IN gckCOMMAND Command
    )
{
    gctINT i;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Stop the command queue. */
    gcmkVERIFY_OK(gckCOMMAND_Stop(Command));

    for (i = 0; i < gcdCOMMAND_QUEUES; ++i)
    {
        if (Command->queues[i].signal)
        {
            gcmkVERIFY_OK(gckOS_DestroySignal(
                Command->os, Command->queues[i].signal
                ));
        }

        if (Command->queues[i].logical)
        {
            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                Command->kernel,
                Command->queues[i].videoMem,
                0,
                gcvFALSE,
                gcvFALSE
                ));

            gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
                Command->kernel,
                Command->queues[i].videoMem,
                0,
                gcvNULL
                ));

            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
                Command->kernel,
                Command->queues[i].videoMem
                ));

            Command->queues[i].videoMem = gcvNULL;
            Command->queues[i].logical  = gcvNULL;
        }
    }

    if (Command->mutexContext)
    {
        /* Delete the context switching mutex. */
        gcmkVERIFY_OK(gckOS_DeleteMutex(Command->os, Command->mutexContext));
    }

    if (Command->mutexContextSeq != gcvNULL)
        gcmkVERIFY_OK(gckOS_DeleteMutex(Command->os, Command->mutexContextSeq));

    if (Command->mutexQueue)
    {
        /* Delete the command queue mutex. */
        gcmkVERIFY_OK(gckOS_DeleteMutex(Command->os, Command->mutexQueue));
    }

    if (Command->powerSemaphore)
    {
        /* Destroy the power management semaphore. */
        gcmkVERIFY_OK(gckOS_DestroySemaphore(Command->os, Command->powerSemaphore));
    }

    if (Command->atomCommit)
    {
        /* Destroy the commit atom. */
        gcmkVERIFY_OK(gckOS_AtomDestroy(Command->os, Command->atomCommit));
    }

#if gcdRECORD_COMMAND
    gckRECORDER_Destory(Command->os, Command->recorder);
#endif

    if (Command->stateMap)
    {
        gcmkOS_SAFE_FREE(Command->os, Command->stateMap);
    }

    if (Command->semaHandleMap)
    {
        gcmkOS_SAFE_FREE(Command->os, Command->semaHandleMap);
    }

    if (Command->fence)
    {
        gcmkVERIFY_OK(gckFENCE_Destory(Command->os, Command->fence));
    }

    /* Mark object as unknown. */
    Command->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckCOMMAND object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Command->os, Command));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckCOMMAND_EnterCommit
**
**  Acquire command queue synchronization objects.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object to destroy.
**
**      gctBOOL FromPower
**          Determines whether the call originates from inside the power
**          management or not.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_EnterCommit(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    )
{
    gceSTATUS status;
    gckHARDWARE hardware;
    gctBOOL atomIncremented = gcvFALSE;
    gctBOOL semaAcquired = gcvFALSE;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Extract the gckHARDWARE and gckEVENT objects. */
    hardware = Command->kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    if (!FromPower)
    {
        /* Increment COMMIT atom to let power management know that a commit is
        ** in progress. */
        gcmkONERROR(_IncrementCommitAtom(Command, gcvTRUE));
        atomIncremented = gcvTRUE;

        /* Notify the system the GPU has a commit. */
        gcmkONERROR(gckOS_Broadcast(Command->os,
                                    hardware,
                                    gcvBROADCAST_GPU_COMMIT));

        /* Acquire the power management semaphore. */
        gcmkONERROR(gckOS_AcquireSemaphore(Command->os,
                                           Command->powerSemaphore));
        semaAcquired = gcvTRUE;
    }

    /* Grab the conmmand queue mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Command->os,
                                   Command->mutexQueue,
                                   gcvINFINITE));

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (semaAcquired)
    {
        /* Release the power management semaphore. */
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(
            Command->os, Command->powerSemaphore
            ));
    }

    if (atomIncremented)
    {
        /* Decrement the commit atom. */
        gcmkVERIFY_OK(_IncrementCommitAtom(
            Command, gcvFALSE
            ));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_ExitCommit
**
**  Release command queue synchronization objects.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object to destroy.
**
**      gctBOOL FromPower
**          Determines whether the call originates from inside the power
**          management or not.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_ExitCommit(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Release the power mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(Command->os, Command->mutexQueue));

    if (!FromPower)
    {
        /* Release the power management semaphore. */
        gcmkONERROR(gckOS_ReleaseSemaphore(Command->os,
                                           Command->powerSemaphore));

        /* Decrement the commit atom. */
        gcmkONERROR(_IncrementCommitAtom(Command, gcvFALSE));
    }

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_StartWaitLinkFE(
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;
    gckHARDWARE hardware;
    gctUINT32 waitOffset = 0;
    gctUINT32 waitLinkBytes;
    gctPOINTER logical;
    gctUINT32 address;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    if (Command->running)
    {
        /* Command queue already running. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    /* Extract the gckHARDWARE object. */
    hardware = Command->kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Query the size of WAIT/LINK command sequence. */
    gcmkONERROR(gckWLFE_WaitLink(
        hardware,
        gcvNULL,
        ~0U,
        Command->offset,
        &waitLinkBytes,
        gcvNULL,
        gcvNULL
        ));

    if ((Command->pageSize - Command->offset < waitLinkBytes)
     || (Command->logical == gcvNULL)
     )
    {
        /* Start at beginning of a new queue. */
        gcmkONERROR(_NewQueue(Command, gcvTRUE));
    }

    logical  = (gctUINT8_PTR) Command->logical + Command->offset;
    address  =                Command->address + Command->offset;

    /* Append WAIT/LINK. */
    gcmkONERROR(gckWLFE_WaitLink(
        hardware,
        logical,
        address,
        0,
        &waitLinkBytes,
        &waitOffset,
        &Command->waitPos.size
        ));

    /* Update wait command position. */
    Command->waitPos.videoMem = Command->videoMem;
    Command->waitPos.offset   = Command->offset + waitOffset;
    Command->waitPos.logical  = (gctUINT8_PTR) logical  + waitOffset;
    Command->waitPos.address  =                address  + waitOffset;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Command->kernel,
        Command->videoMem,
        Command->offset,
        logical,
        waitLinkBytes
        ));

    /* Adjust offset. */
    Command->offset   += waitLinkBytes;
    Command->newQueue = gcvFALSE;

    gcmkDUMP(Command->os, "#[wait-link: fe start]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        logical,
        address,
        waitLinkBytes
        );


#if !gcdCAPTURE_ONLY_MODE
    /* Enable command processor. */
    gcmkONERROR(gckWLFE_Execute(
        hardware,
        address,
        waitLinkBytes
        ));
#endif


    /* Command queue is running. */
    Command->running = gcvTRUE;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_StartAsyncFE(
    IN gckCOMMAND Command
    )
{
    if ((Command->pageSize <= Command->offset) ||
        (Command->logical == gcvNULL))
    {
        /* Start at beginning of a new queue. */
        gcmkVERIFY_OK(_NewQueue(Command, gcvTRUE));
    }

    /* Command queue is running. */
    Command->running = gcvTRUE;

    /* Nothing to do. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_StartMCFE(
    IN gckCOMMAND Command
    )
{
    if ((Command->pageSize <= Command->offset) ||
        (Command->logical == gcvNULL))
    {
        /* Start at beginning of a new queue. */
        gcmkVERIFY_OK(_NewQueue(Command, gcvTRUE));
    }

    /* Command queue is running. */
    Command->running = gcvTRUE;

    /* Nothing to do. */
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckCOMMAND_Start
**
**  Start up the command queue.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object to start.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Start(
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Command=%p", Command);

    if (Command->feType == gcvHW_FE_WAIT_LINK)
    {
        gcmkONERROR(_StartWaitLinkFE(Command));
    }
    else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        gcmkONERROR(_StartMCFE(Command));
    }
    else
    {
        gcmkONERROR(_StartAsyncFE(Command));
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;

}

static gceSTATUS
_StopWaitLinkFE(
    IN gckCOMMAND Command
    )
{
    gckHARDWARE hardware;
    gceSTATUS status;
    gctUINT32 idle;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Extract the gckHARDWARE object. */
    hardware = Command->kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Replace last WAIT with END. */
    gcmkONERROR(gckWLFE_End(
        hardware,
        Command->waitPos.logical,
        Command->waitPos.address,
        &Command->waitPos.size
        ));

    gcmkDUMP(Command->os, "#[end: fe stop]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        Command->waitPos.logical,
        Command->waitPos.address,
        Command->waitPos.size
        );


    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(Command->kernel->hardware,
                                            Command->logical,
                                            Command->offset));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Command->kernel,
        Command->waitPos.videoMem,
        Command->waitPos.offset,
        Command->waitPos.logical,
        Command->waitPos.size
        ));

    /* Wait for idle. */
    gcmkONERROR(gckHARDWARE_GetIdle(hardware, gcvTRUE, &idle));

    /* Command queue is no longer running. */
    Command->running = gcvFALSE;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_StopAsyncFE(
    IN gckCOMMAND Command
    )
{
    gckHARDWARE hardware;
    gceSTATUS status;
    gctUINT32 idle;

    gcmkHEADER_ARG("Command=%p", Command);

    hardware = Command->kernel->hardware;

    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(hardware,
                                            Command->logical,
                                            Command->offset));

    /* Wait for idle. */
    gcmkONERROR(gckHARDWARE_GetIdle(hardware, gcvTRUE, &idle));

    /* Command queue is no longer running. */
    Command->running = gcvFALSE;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_StopMCFE(
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;
    gckHARDWARE hardware;

    gcmkHEADER_ARG("Command=%p", Command);

    hardware = Command->kernel->hardware;

    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(hardware,
                                            Command->logical,
                                            Command->offset));

    /* Command queue is no longer running. */
    Command->running = gcvFALSE;

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
**  gckCOMMAND_Stop
**
**  Stop the command queue.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object to stop.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Stop(
    IN gckCOMMAND Command
    )
{
    if (!Command->running)
    {
        /* Command queue is not running. */
        return gcvSTATUS_OK;
    }

    if (Command->feType == gcvHW_FE_WAIT_LINK)
    {
        return _StopWaitLinkFE(Command);
    }
    else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        return _StopMCFE(Command);
    }
    else
    {
        return _StopAsyncFE(Command);
    }
}

static gceSTATUS
_CommitWaitLinkOnce(
    IN gckCOMMAND Command,
    IN gckCONTEXT Context,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gcsSTATE_DELTA_PTR StateDelta,
    IN gctUINT32 ProcessID,
    IN gctBOOL Shared,
    INOUT gctBOOL *contextSwitched,
    IN gctPOINTER PreemptCommit,
    IN gctBOOL InPreemptThread,
    IN gctUINT64 MaxAsyncTimeStamp
    )
{
    gceSTATUS status;
    gctBOOL contextAcquired = gcvFALSE;
    gckHARDWARE hardware;

    gcsCONTEXT_PTR contextBuffer;
    gctUINT8_PTR commandBufferLogical = gcvNULL;
    gctUINT32 commandBufferAddress = 0;
    gckVIDMEM_NODE commandBufferVideoMem = gcvNULL;
    gctUINT8_PTR commandBufferTail = gcvNULL;
    gctUINT commandBufferSize;
    gctUINT32 linkBytes;
    gctSIZE_T bytes;
    gctUINT32 offset;
    gctPOINTER entryLogical;
    gctUINT32 entryAddress;
    gctUINT32 entryBytes;
    gctUINT32 exitAddress;
    gctUINT32 exitBytes;
    gctPOINTER waitLinkLogical;
    gctUINT32 waitLinkAddress;
    gctUINT32 waitLinkBytes;
    gctUINT32 waitOffset;
    gctUINT32 waitSize;

#if gcdCAPTURE_ONLY_MODE
    gctINT i;
#endif

#ifdef __QNXNTO__
    gctPOINTER userCommandBufferLogical       = gcvNULL;
    gctBOOL    userCommandBufferLogicalMapped = gcvFALSE;
#endif

#if gcdDUMP_IN_KERNEL
    gctPOINTER contextDumpLogical = gcvNULL;
# endif
    gctUINT32 exitLinkLow = 0, exitLinkHigh = 0;
    gctUINT32 entryLinkLow = 0, entryLinkHigh = 0;
    gctUINT32 commandLinkLow = 0, commandLinkHigh = 0;

    gcmkHEADER_ARG("Command=%p CommandBuffer=%p ProcessID=%d",
        Command, CommandBuffer, ProcessID
        );

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    gcmkASSERT(Command->feType == gcvHW_FE_WAIT_LINK);

    /* Acquire the context switching mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        Command->os, Command->mutexContext, gcvINFINITE
        ));
    contextAcquired = gcvTRUE;

    /* Extract the gckHARDWARE and gckEVENT objects. */
    hardware = Command->kernel->hardware;

    /* Query the size of LINK command. */
    gcmkONERROR(gckWLFE_Link(
        hardware, gcvNULL, 0, 0, &linkBytes, gcvNULL, gcvNULL
        ));

    /* Compute the command buffer entry and the size. */
    commandBufferLogical
        = (gctUINT8_PTR) gcmUINT64_TO_PTR(CommandBuffer->logical)
        +                CommandBuffer->startOffset;

    commandBufferAddress = CommandBuffer->address
                         + CommandBuffer->startOffset;

#ifdef __QNXNTO__
    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Command->kernel,
        ProcessID,
        CommandBuffer->videoMemNode,
        &commandBufferVideoMem
        ));

    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        Command->kernel,
        commandBufferVideoMem,
        gcvFALSE,
        gcvFALSE,
        &userCommandBufferLogical
        ));

    commandBufferLogical = (gctUINT8_PTR)userCommandBufferLogical + CommandBuffer->startOffset;
    userCommandBufferLogicalMapped =gcvTRUE;
#endif

    commandBufferSize = CommandBuffer->size;

    gcmkONERROR(_CheckFlushMMU(Command, hardware));

    if (Command->dummyDraw == gcvTRUE &&
        Context != gcvNULL)
    {
        Command->dummyDraw = gcvFALSE;
        gcmkONERROR(_DummyDraw(Command));
    }

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_FENCE_64BIT) &&
        Command->kernel->asyncCommand &&
        MaxAsyncTimeStamp != 0)
    {
        gcmkONERROR(_WaitForAsyncCommandStamp(
            Command,
            MaxAsyncTimeStamp
            ));
    }

    /* Get the current offset. */
    offset = Command->offset;

    /* Compute number of bytes left in current kernel command queue. */
    bytes = Command->pageSize - offset;

    /* Query the size of WAIT/LINK command sequence. */
    gcmkONERROR(gckWLFE_WaitLink(
        hardware,
        gcvNULL,
        ~0U,
        offset,
        &waitLinkBytes,
        gcvNULL,
        gcvNULL
        ));

    /* Is there enough space in the current command queue? */
    if (bytes < waitLinkBytes)
    {
        /* No, create a new one. */
        gcmkONERROR(_NewQueue(Command, gcvFALSE));

        /* Get the new current offset. */
        offset = Command->offset;

        /* Recompute the number of bytes in the new kernel command queue. */
        bytes = Command->pageSize - offset;
        gcmkASSERT(bytes >= waitLinkBytes);
    }

    /* Compute the location if WAIT/LINK command sequence. */
    waitLinkLogical  = (gctUINT8_PTR) Command->logical  + offset;
    waitLinkAddress  =                Command->address  + offset;

    /* Context switch required? */
    if (Context == gcvNULL)
    {
        /* See if we have to switch pipes for the command buffer. */
        if (CommandBuffer->entryPipe == (gctUINT32)(Command->pipeSelect))
        {
            /* Skip reserved head bytes. */
            offset = CommandBuffer->reservedHead;
        }
        else
        {
            gctUINT32 pipeBytes = CommandBuffer->reservedHead;

            /* The current hardware and the entry command buffer pipes
            ** are different, switch to the correct pipe. */
            gcmkONERROR(gckHARDWARE_PipeSelect(
                Command->kernel->hardware,
                commandBufferLogical,
                CommandBuffer->entryPipe,
                &pipeBytes
                ));

            /* Do not skip pipe switching sequence. */
            offset = 0;

            /* Reserved bytes in userspace must be exact for a pipeSelect. */
            gcmkASSERT(pipeBytes == CommandBuffer->reservedHead);
        }

        /* Compute the entry. */
        entryLogical  =                commandBufferLogical  + offset;
        entryAddress  =                commandBufferAddress  + offset;
        entryBytes    =                commandBufferSize     - offset;

        Command->currContext = gcvNULL;
    }
#if gcdDEBUG_OPTION && gcdDEBUG_FORCE_CONTEXT_UPDATE
    else if (1)
#else
    else if (Command->currContext != Context)
#endif
    {
        /* Get the current context buffer. */
        contextBuffer = Context->buffer;

        /* Yes, merge in the deltas. */
#if gcdENABLE_SW_PREEMPTION
        if (InPreemptThread || Command->kernel->preemptionMode == gcvNON_FULLY_PREEMPTIBLE_MODE)
        {
            gckPREEMPT_COMMIT preemptCommit = (gckPREEMPT_COMMIT)PreemptCommit;
            gcmkONERROR(gckCONTEXT_PreemptUpdate(Context, preemptCommit));
        }
        else
        {
            gcmkONERROR(gckCONTEXT_Update(Context, ProcessID, StateDelta));

            gcmkONERROR(gckCONTEXT_ConstructPrevDelta(Context, ProcessID, StateDelta));
        }
#else
        gcmkONERROR(gckCONTEXT_Update(Context, ProcessID, StateDelta));
#endif

        /***************************************************************
        ** SWITCHING CONTEXT.
        */

        /* Determine context buffer entry offset. */
        offset = (Command->pipeSelect == gcvPIPE_3D)

            /* Skip pipe switching sequence. */
            ? Context->entryOffset3D + Context->pipeSelectBytes

            /* Do not skip pipe switching sequence. */
            : Context->entryOffset3D;

        /* Compute the entry. */
        entryLogical  = (gctUINT8_PTR) contextBuffer->logical  + offset;
        entryAddress  =                contextBuffer->address  + offset;
        entryBytes    =                Context->bufferSize     - offset;

        /* See if we have to switch pipes between the context
            and command buffers. */
        if (CommandBuffer->entryPipe == gcvPIPE_3D)
        {
            /* Skip reserved head bytes. */
            offset = CommandBuffer->reservedHead;
        }
        else
        {
            gctUINT32 pipeBytes = CommandBuffer->reservedHead;

            /* The current hardware and the initial context pipes are
                different, switch to the correct pipe. */
            gcmkONERROR(gckHARDWARE_PipeSelect(
                Command->kernel->hardware,
                commandBufferLogical,
                CommandBuffer->entryPipe,
                &pipeBytes
                ));

            /* Do not skip pipe switching sequence. */
            offset = 0;

            /* Reserved bytes in userspace must be exact for a pipeSelect. */
            gcmkASSERT(pipeBytes == CommandBuffer->reservedHead);
        }

        /* Generate a LINK from the context buffer to
            the command buffer. */
        gcmkONERROR(gckWLFE_Link(
            hardware,
            contextBuffer->link3D,
            commandBufferAddress + offset,
            commandBufferSize    - offset,
            &linkBytes,
            &commandLinkLow,
            &commandLinkHigh
            ));

#if gcdCAPTURE_ONLY_MODE
        for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; ++i)
        {
            gcsCONTEXT_PTR buffer = contextBuffer;

            gckOS_CopyToUserData(Command->os, buffer->logical, CommandBuffer->contextLogical[i], Context->bufferSize);

            buffer = buffer->next;
        }
#endif

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            contextBuffer->videoMem,
            entryAddress - contextBuffer->address,
            entryLogical,
            entryBytes
            ));

        /* Update the current context. */
        Command->currContext = Context;

        if (contextSwitched)
        {
            *contextSwitched = gcvTRUE;
        }

#if gcdDUMP_IN_KERNEL
        contextDumpLogical = entryLogical;
#endif


#if gcdRECORD_COMMAND
        gckRECORDER_Record(
            Command->recorder,
            gcvNULL,
            0xFFFFFFFF,
            entryLogical,
            entryBytes
            );
#endif
    }

    /* Same context. */
    else
    {
        /* See if we have to switch pipes for the command buffer. */
        if (CommandBuffer->entryPipe == (gctUINT32)(Command->pipeSelect))
        {
            /* Skip reserved head bytes. */
            offset = CommandBuffer->reservedHead;
        }
        else
        {
            gctUINT32 pipeBytes = CommandBuffer->reservedHead;

            /* The current hardware and the entry command buffer pipes
            ** are different, switch to the correct pipe. */
            gcmkONERROR(gckHARDWARE_PipeSelect(
                Command->kernel->hardware,
                commandBufferLogical,
                CommandBuffer->entryPipe,
                &pipeBytes
                ));

            /* Do not skip pipe switching sequence. */
            offset = 0;

            /* Reserved bytes in userspace must be exact for a pipeSelect. */
            gcmkASSERT(pipeBytes == CommandBuffer->reservedHead);
        }

        /* Compute the entry. */
        entryLogical  =                commandBufferLogical  + offset;
        entryAddress  =                commandBufferAddress  + offset;
        entryBytes    =                commandBufferSize     - offset;
    }

    (void)entryLogical;

    /* Determine the location to jump to for the command buffer being
    ** scheduled. */
    if (Command->newQueue)
    {
        /* New command queue, jump to the beginning of it. */
        /* Some extra commands (at beginning) are required for new queue. */
        exitAddress  = Command->address;
        exitBytes    = Command->offset + waitLinkBytes;
    }
    else
    {
        /* Still within the preexisting command queue, jump to the new
           WAIT/LINK command sequence. */
        exitAddress  = waitLinkAddress;
        exitBytes    = waitLinkBytes;
    }

    /* Add a new WAIT/LINK command sequence. When the command buffer which is
       currently being scheduled is fully executed by the GPU, the FE will
       jump to this WAIT/LINK sequence. */
    gcmkONERROR(gckWLFE_WaitLink(
        hardware,
        waitLinkLogical,
        waitLinkAddress,
        offset,
        &waitLinkBytes,
        &waitOffset,
        &waitSize
        ));

    if (Command->newQueue)
    {
        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            0,
            Command->logical,
            exitBytes
            ));
    }
    else
    {
        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            Command->offset,
            waitLinkLogical,
            exitBytes
            ));
    }

    /* Determine the location of the TAIL in the command buffer. */
    commandBufferTail
        = commandBufferLogical
        + commandBufferSize
        - CommandBuffer->reservedTail;

    /* Generate command which writes out commit stamp. */
    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_FENCE_64BIT))
    {
        gctUINT32 bytes;

        gcmkONERROR(gckHARDWARE_Fence(
            hardware,
            gcvENGINE_RENDER,
            commandBufferTail,
            Command->fence->address,
            Command->commitStamp,
            &bytes
            ));

        commandBufferTail += bytes;
    }

    /* Generate a LINK from the end of the command buffer being scheduled
       back to the kernel command queue. */
    if (Shared == gcvFALSE)
    {
        gcmkONERROR(gckWLFE_Link(
            hardware,
            commandBufferTail,
            exitAddress,
            exitBytes,
            &linkBytes,
            &exitLinkLow,
            &exitLinkHigh
            ));
    }
    else
    {
        gctUINT8_PTR link = commandBufferTail + CommandBuffer->exitIndex * 16;
        gctSIZE_T bytes = 8;
        gceCORE_3D_MASK mask = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_CLUSTER) ?
            gcvCORE_3D_ALL_MASK : ((gceCORE_3D_MASK)(1 << hardware->kernel->chipID));

        gcmkONERROR(gckWLFE_ChipEnable(
            hardware,
            link,
            mask,
            &bytes
            ));

        link += bytes;

        gcmkONERROR(gckWLFE_Link(
            hardware,
            link,
            exitAddress,
            exitBytes,
            &linkBytes,
            &exitLinkLow,
            &exitLinkHigh
            ));

        link += linkBytes;
    }

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Command->kernel,
        ProcessID,
        CommandBuffer->videoMemNode,
        &commandBufferVideoMem
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Command->kernel,
        commandBufferVideoMem,
        CommandBuffer->startOffset,
        commandBufferLogical,
        commandBufferSize
        ));

#if gcdRECORD_COMMAND
    gckRECORDER_Record(
        Command->recorder,
        commandBufferLogical + offset,
        commandBufferSize - offset,
        gcvNULL,
        0xFFFFFFFF
        );

    gckRECORDER_AdvanceIndex(Command->recorder, Command->commitStamp);
#endif

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /*
     * Skip link to entryAddress.
     * Instead, we directly link to final wait link position.
     */
    gcmkONERROR(gckWLFE_Link(
        hardware,
        Command->waitPos.logical,
        waitLinkAddress,
        waitLinkBytes,
        &Command->waitPos.size,
        &entryLinkLow,
        &entryLinkHigh
        ));
#  else
    /* Generate a LINK from the previous WAIT/LINK command sequence to the
       entry determined above (either the context or the command buffer).
       This LINK replaces the WAIT instruction from the previous WAIT/LINK
       pair, therefore we use WAIT metrics for generation of this LINK.
       This action will execute the entire sequence. */
    gcmkONERROR(gckWLFE_Link(
        hardware,
        Command->waitPos.logical,
        entryAddress,
        entryBytes,
        &Command->waitPos.size,
        &entryLinkLow,
        &entryLinkHigh
        ));
#  endif

#if gcdLINK_QUEUE_SIZE
    if (Command->kernel->stuckDump >= gcvSTUCK_DUMP_USER_COMMAND)
    {
        gcuQUEUEDATA data;

        gcmkVERIFY_OK(gckOS_GetProcessID(&data.linkData.pid));

        data.linkData.start    = entryAddress;
        data.linkData.end      = entryAddress + entryBytes;
        data.linkData.linkLow  = entryLinkLow;
        data.linkData.linkHigh = entryLinkHigh;

        gckQUEUE_Enqueue(&hardware->linkQueue, &data);

        if (commandBufferAddress + offset != entryAddress)
        {
             data.linkData.start    =  commandBufferAddress + offset;
             data.linkData.end      =  commandBufferAddress + commandBufferSize;
             data.linkData.linkLow  = commandLinkLow;
             data.linkData.linkHigh = commandLinkHigh;

            gckQUEUE_Enqueue(&hardware->linkQueue, &data);
        }

        if (Command->kernel->stuckDump >= gcvSTUCK_DUMP_ALL_COMMAND)
        {
            data.linkData.start    = exitAddress;
            data.linkData.end      = exitAddress + exitBytes;
            data.linkData.linkLow  = exitLinkLow;
            data.linkData.linkHigh = exitLinkHigh;

            /* Dump kernel command.*/
            gckQUEUE_Enqueue(&hardware->linkQueue, &data);
        }
    }
#endif

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Command->kernel,
        Command->waitPos.videoMem,
        Command->waitPos.offset,
        Command->waitPos.logical,
        Command->waitPos.size
        ));

    if (entryAddress != commandBufferAddress + offset)
    {
        gcmkDUMP(Command->os, "#[context]");
        gcmkDUMP_BUFFER(
            Command->os,
            gcvDUMP_BUFFER_KERNEL_CONTEXT,
            contextDumpLogical,
            entryAddress,
            entryBytes
            );

        /* execute context. */
        gcmkDUMP(Command->os,
            "@[execute 0 0 0x%08X 0x%08X]",
            entryAddress + offset,
            entryBytes - offset - 8
            );
    }

    gcmkDUMP(Command->os, "#[command: user]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_COMMAND,
        commandBufferLogical + offset,
        commandBufferAddress + offset,
        commandBufferSize - offset
        );

    /* execute user commands. */
    gcmkDUMP(
        Command->os,
        "@[execute 0 0 0x%08X 0x%08X]",
        commandBufferAddress
            + CommandBuffer->reservedHead,
        commandBufferSize
            - CommandBuffer->reservedHead
            - CommandBuffer->reservedTail
        );

    gcmkDUMP(Command->os, "#[wait-link]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        waitLinkLogical,
        waitLinkAddress,
        waitLinkBytes
        );

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    gcmkDUMP(
        Command->os,
        "#[null driver: below command skipped link to 0x%08X 0x%08X]",
        entryAddress,
        entryBytes
        );
#endif

    gcmkDUMP(Command->os, "#[link: break prev wait-link]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        Command->waitPos.logical,
        Command->waitPos.address,
        Command->waitPos.size
        );

    /* Update the current pipe. */
    Command->pipeSelect = CommandBuffer->exitPipe;

    /* Update command queue offset. */
    Command->offset  += waitLinkBytes;
    Command->newQueue = gcvFALSE;

    /* Update address of last WAIT. */
    Command->waitPos.videoMem = Command->videoMem;
    Command->waitPos.offset   = Command->offset - waitLinkBytes + waitOffset;
    Command->waitPos.logical  = (gctUINT8_PTR)waitLinkLogical  + waitOffset;
    Command->waitPos.address  = waitLinkAddress  + waitOffset;
    Command->waitPos.size     = waitSize;

    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(
        hardware, Command->logical, Command->offset
        ));

    /* Release the context switching mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
    contextAcquired = gcvFALSE;

    if (status == gcvSTATUS_INTERRUPTED)
    {
        gcmkTRACE(
            gcvLEVEL_INFO,
            "%s(%d): Intterupted in gckEVENT_Submit",
            __FUNCTION__, __LINE__
            );
        status = gcvSTATUS_OK;
    }
    else
    {
        gcmkONERROR(status);
    }

#ifdef __QNXNTO__
    if(userCommandBufferLogicalMapped)
    {
        gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
            Command->kernel,
            commandBufferVideoMem,
            ProcessID,
            gcvFALSE,
            gcvFALSE
            ));

        userCommandBufferLogicalMapped =gcvFALSE;
    }
#endif

    /* Return status. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (contextAcquired)
    {
        /* Release the context switching mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
    }

#ifdef __QNXNTO__
    if (userCommandBufferLogicalMapped)
    {
        gcmkVERIFY_OK(gckOS_UnmapUserPointer(
            Command->os,
            userCommandBufferLogical,
            0,
            commandBufferLogical));
    }
#endif

    /* Return status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CommitAsyncOnce(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer
    )
{
    gceSTATUS       status;
    gckHARDWARE     hardware = Command->kernel->hardware;
    gctBOOL         available = gcvFALSE;
    gctBOOL         acquired = gcvFALSE;
    gctUINT8_PTR    commandBufferLogical;
    gctUINT8_PTR    commandBufferTail;
    gctUINT         commandBufferSize;
    gctUINT32       commandBufferAddress;
    gctUINT32       fenceBytes;
    gctUINT32       oldValue;
    gctUINT32       flushBytes;

    gcmkHEADER();

    gckOS_AtomicExchange(Command->os,
                         hardware->pageTableDirty[gcvENGINE_BLT],
                         0,
                         &oldValue);

    if (oldValue)
    {
        gckHARDWARE_FlushAsyncMMU(hardware, gcvNULL, &flushBytes);

        gcmkASSERT(flushBytes <= CommandBuffer->reservedHead);

        /* Compute the command buffer entry to insert the flushMMU commands. */
        commandBufferLogical = (gctUINT8_PTR)gcmUINT64_TO_PTR(CommandBuffer->logical)
                             + CommandBuffer->startOffset
                             + CommandBuffer->reservedHead
                             - flushBytes;

        commandBufferAddress = CommandBuffer->address
                             + CommandBuffer->startOffset
                             + CommandBuffer->reservedHead
                             - flushBytes;

        gckHARDWARE_FlushAsyncMMU(hardware, commandBufferLogical, &flushBytes);
    }
    else
    {
        /* Compute the command buffer entry. */
        commandBufferLogical = (gctUINT8_PTR)gcmUINT64_TO_PTR(CommandBuffer->logical)
                             + CommandBuffer->startOffset
                             + CommandBuffer->reservedHead;

        commandBufferAddress = CommandBuffer->address
                             + CommandBuffer->startOffset
                             + CommandBuffer->reservedHead;

        flushBytes = 0;
    }

    commandBufferTail = (gctUINT8_PTR)gcmUINT64_TO_PTR(CommandBuffer->logical)
                      + CommandBuffer->startOffset
                      + CommandBuffer->size
                      - CommandBuffer->reservedTail;

    gcmkONERROR(gckHARDWARE_Fence(
        hardware,
        gcvENGINE_BLT,
        commandBufferTail,
        Command->fence->address,
        Command->commitStamp,
        &fenceBytes
        ));

    gcmkASSERT(fenceBytes <= CommandBuffer->reservedTail);

    commandBufferSize = CommandBuffer->size
                      - CommandBuffer->reservedHead
                      - CommandBuffer->reservedTail
                      + flushBytes
                      + fenceBytes;

    gckOS_AcquireMutex(Command->os, Command->mutexContext, gcvINFINITE);
    acquired = gcvTRUE;

    /* Acquire a slot. */
    for (;;)
    {
        gcmkONERROR(gckASYNC_FE_ReserveSlot(hardware, &available));

        if (available)
        {
            break;
        }
        else
        {
            gcmkTRACE_ZONE(gcvLEVEL_INFO, _GC_OBJ_ZONE, "No available slot, have to wait");

            gckOS_Delay(Command->os, 1);
        }
    }

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /* Skip submit to hardware for NULL driver. */
    gcmkDUMP(Command->os, "#[null driver: below command is skipped]");
#endif

    gcmkDUMP(Command->os, "#[async-command: user]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_ASYNC_COMMAND,
        commandBufferLogical,
        commandBufferAddress,
        commandBufferSize
        );

    gcmkDUMP(
        Command->os,
        "@[execute 1 0 0x%08X 0x%08X]",
        commandBufferAddress
            + CommandBuffer->reservedHead,
        CommandBuffer->size
            - CommandBuffer->reservedHead
            - CommandBuffer->reservedTail
        );

#if !gcdNULL_DRIVER
    /* Execute command buffer. */
    gckASYNC_FE_Execute(hardware, commandBufferAddress, commandBufferSize);
#endif

    gckOS_ReleaseMutex(Command->os, Command->mutexContext);
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gckOS_ReleaseMutex(Command->os, Command->mutexContext);
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CommitMultiChannelOnce(
    IN gckCOMMAND Command,
    IN gckCONTEXT Context,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer
    )
{
    gceSTATUS    status;
    gctUINT8_PTR commandBufferLogical;
    gctUINT      commandBufferSize;
    gctUINT32    commandBufferAddress;
    gckHARDWARE  hardware;
    gctUINT64    bit;

    gcmkHEADER_ARG("priority=%d channelId=%d videoMemNode=%u size=0x%x patchHead=%p",
                   CommandBuffer->priority, CommandBuffer->channelId,
                   CommandBuffer->videoMemNode, CommandBuffer->size,
                   gcmUINT64_TO_PTR(CommandBuffer->patchHead));

    gcmkASSERT(Command->feType == gcvHW_FE_MULTI_CHANNEL);

    hardware = Command->kernel->hardware;

    /* Check flush mcfe MMU cache. */
    gcmkONERROR(_CheckFlushMcfeMMU(Command, hardware));

    /* Compute the command buffer entry and the size. */
    commandBufferLogical
        = (gctUINT8_PTR) gcmUINT64_TO_PTR(CommandBuffer->logical)
        +                CommandBuffer->startOffset
        +                CommandBuffer->reservedHead;

    commandBufferAddress = CommandBuffer->address
                         + CommandBuffer->startOffset
                         + CommandBuffer->reservedHead;

    /* reservedTail bytes are not used, because fence not enable. */
    commandBufferSize
        = CommandBuffer->size
        - CommandBuffer->reservedHead
        - CommandBuffer->reservedTail;

    if (commandBufferSize & 8)
    {
        /*
         * Need 16 byte alignment for MCFE command size.
         * command is already 8 byte aligned, if not 16 byte aligned,
         * we need append 8 bytes.
         */
        gctUINT32 nop[2];
        gctSIZE_T bytes = 8;
        gctUINT8_PTR tail = commandBufferLogical + commandBufferSize;

        gckMCFE_Nop(hardware, nop, &bytes);
        gcmkASSERT(bytes == 8);

        gckOS_WriteMemory(Command->os, tail, nop[0]);
        gckOS_WriteMemory(Command->os, tail + 4, nop[1]);

        commandBufferSize += 8;
    }

    /* Large command buffer size does not make sense. */
    gcmkASSERT(commandBufferSize < 0x800000);

    if (CommandBuffer->channelId != 0)
    {
        /* Sync from the system channel. */
        gcmkONERROR(_SyncFromSystemChannel(
            Command,
            (gctBOOL)CommandBuffer->priority,
            (gctUINT32)CommandBuffer->channelId
            ));
    }

    Command->currContext = Context;

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /* Skip submit to hardware for NULL driver. */
    gcmkDUMP(Command->os, "#[null driver: below command is skipped]");
#endif

    gcmkDUMP(Command->os, "#[mcfe-command: user]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_COMMAND,
        commandBufferLogical,
        commandBufferAddress,
        commandBufferSize
        );

    gcmkDUMP(Command->os,
             "@[execute %d %d 0x%08X 0x%08X]",
             CommandBuffer->channelId,
             CommandBuffer->priority,
             commandBufferAddress,
             commandBufferSize);

#if !gcdNULL_DRIVER
    /* Execute command buffer. */
    gcmkONERROR(gckMCFE_Execute(
        hardware,
        (gctBOOL)CommandBuffer->priority,
        (gctUINT32)CommandBuffer->channelId,
        commandBufferAddress,
        commandBufferSize
        ));
#endif

    bit = 1ull << CommandBuffer->channelId;

    /* This channel is dirty. */
    Command->dirtyChannel[CommandBuffer->priority ? 1 : 0] |= bit;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
_ValidCommandBuffer(
    IN gckCOMMAND Command,
    IN gctUINT32 ProcessId,
    IN gcsHAL_COMMAND_LOCATION *cmdLoc
    )
{
    gceSTATUS status;
    gcsDATABASE_RECORD Record;

    gcmkHEADER_ARG("Command=%p CommandLocation=%p Pid=%u",
                    Command, cmdLoc, ProcessId);

    gcmkONERROR(gckKERNEL_FindProcessDB(
        Command->kernel,
        ProcessId,
        0,
        gcvDB_VIDEO_MEMORY_LOCKED,
        gcmINT2PTR(cmdLoc->videoMemNode),
        &Record
        ));

    if (gcmPTR_TO_UINT64(Record.physical) != cmdLoc->logical)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ADDRESS);
    }

OnError:
    gcmkFOOTER();
    return status;
}
/*******************************************************************************
**
**  gckCOMMAND_Commit
**
**  Commit command buffers to the command queue.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to a gckCOMMAND object.
**
**      gcsHAL_SUBCOMMIT * SubCommit
**          Commit information, includes context, delta, and command buffer
*           locations.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**  OUTPUT:
**      gctBOOL *contextSwitched
**          pass context Switch flag to upper
*/
gceSTATUS
gckCOMMAND_Commit(
    IN gckCOMMAND Command,
    IN gcsHAL_SUBCOMMIT * SubCommit,
    IN gctUINT32 ProcessId,
    IN gctBOOL Shared,
    OUT gctUINT64_PTR CommitStamp,
    INOUT gctBOOL *contextSwitched
    )
{
    gceSTATUS status;
    gcsSTATE_DELTA_PTR delta = gcmUINT64_TO_PTR(SubCommit->delta);
    gckCONTEXT context = gcvNULL;
    gcsHAL_COMMAND_LOCATION *cmdLoc = &SubCommit->commandBuffer;
    gcsHAL_COMMAND_LOCATION _cmdLoc;
    gctPOINTER userPtr = gcvNULL;
    gctBOOL needCopy = gcvFALSE;
    gcsPATCH_LIST_VARIABLE patchListVar = {0, 0};
    gctBOOL commitEntered = gcvFALSE;

    gcmkHEADER_ARG("Command=%p SubCommit=%p delta=%p context=%u pid=%u",
                   Command, SubCommit, delta, SubCommit->context, ProcessId);

    gcmkVERIFY_OK(gckOS_QueryNeedCopy(Command->os, ProcessId, &needCopy));

    if (SubCommit->context)
    {
        context = gckKERNEL_QueryPointerFromName(
            Command->kernel,
            (gctUINT32)(SubCommit->context)
            );
    }

    do
    {
        gctUINT64 next;

        /* Skip the first nested one. */
        if (userPtr)
        {
            /* Copy/map command buffer location from user. */
            if (needCopy)
            {
                cmdLoc = &_cmdLoc;

                status = gckOS_CopyFromUserData(
                    Command->os,
                    cmdLoc,
                    userPtr,
                    gcmSIZEOF(gcsHAL_COMMAND_LOCATION)
                    );
            }
            else
            {
                status = gckOS_MapUserPointer(
                    Command->os,
                    userPtr,
                    gcmSIZEOF(gcsHAL_COMMAND_LOCATION),
                    (gctPOINTER *)&cmdLoc
                    );
            }

            if (gcmIS_ERROR(status))
            {
                userPtr = gcvNULL;

                gcmkONERROR(status);
            }
        }

        gcmkONERROR(_ValidCommandBuffer(Command, ProcessId, cmdLoc));

        /* Acquire the command queue. */
        gcmkONERROR(gckCOMMAND_EnterCommit(Command, gcvFALSE));
        commitEntered = gcvTRUE;

        gcmkVERIFY_OK(
            _HandlePatchList(Command, cmdLoc, &patchListVar));

        if (Command->feType == gcvHW_FE_WAIT_LINK)
        {
            /* Commit command buffers. */
            status = _CommitWaitLinkOnce(Command,
                                         context,
                                         cmdLoc,
                                         delta,
                                         ProcessId,
                                         Shared,
                                         contextSwitched,
                                         gcvNULL,
                                         gcvFALSE,
                                         patchListVar.maxAsyncTimestamp);
        }
        else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
        {
#if gcdENABLE_SW_PREEMPTION
            gcmkDUMP(Command->os, "Priority: %d", SubCommit->priorityID);
#endif
            status = _CommitMultiChannelOnce(Command, context, cmdLoc);
        }
        else
        {
            gcmkASSERT(Command->feType == gcvHW_FE_ASYNC);

            status = _CommitAsyncOnce(Command, cmdLoc);
        }

        if (status != gcvSTATUS_INTERRUPTED)
        {
            gcmkONERROR(status);
        }

        /* Release the command queue. */
        gcmkONERROR(gckCOMMAND_ExitCommit(Command, gcvFALSE));
        commitEntered = gcvFALSE;

        /* Do not need context or delta for later commands. */
        context = gcvNULL;
        delta   = gcvNULL;

        next    = cmdLoc->next;

        /* Unmap user pointer if mapped. */
        if (!needCopy && userPtr)
        {
            gcmkVERIFY_OK(gckOS_UnmapUserPointer(
                Command->os,
                userPtr,
                gcmSIZEOF(gcsHAL_COMMAND_LOCATION),
                cmdLoc
                ));
        }

        /* Advance to next command buffer location from user. */
        userPtr = gcmUINT64_TO_PTR(next);
    }
    while (userPtr);

    if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        /*
         * Semphore synchronization.
         *
         * Here we blindly sync dirty other channels to the system channel.
         * The scenario to sync channels to the system channel:
         * 1. Need to sync channels who sent semaphores.
         * 2. Need to sync dirty channels when event(interrupt) is to sent.
         * 3. Need to sync dirty channels when system channel need run something
         *    such as flush mmu.
         *
         * When power management is on, blindly sync dirty channels is OK because
         * there's always a event(intrrupt).
         *
         * The only condition we sync more than needed is:
         * a. power manangement is off.
         * b. no user event is attached when commit.
         * c. no user event is to be submitted in next ioctl.
         * That's a rare condition.
         *
         * Conclusion is that, blindly sync dirty channels is a good choice for
         * now.
         */
         gcmkONERROR(_SyncToSystemChannel(Command, Command->dirtyChannel, gcvTRUE));
    }

    /* Output commit stamp. */
    *CommitStamp = Command->commitStamp;
    Command->commitStamp++;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (commitEntered)
    {
        /* Release the command queue mutex. */
        gcmkVERIFY_OK(gckCOMMAND_ExitCommit(Command, gcvFALSE));
    }

    if (!needCopy && userPtr)
    {
        gckOS_UnmapUserPointer(
            Command->os,
            userPtr,
            gcmSIZEOF(gcsHAL_COMMAND_LOCATION),
            cmdLoc
            );
    }

    gcmkFOOTER();
    return status;
}


/*******************************************************************************
**
**  gckCOMMAND_Reserve
**
**  Reserve space in the command queue.  Also acquire the command queue mutex.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object.
**
**      gctSIZE_T RequestedBytes
**          Number of bytes previously reserved.
**
**  OUTPUT:
**
**      gctPOINTER * Buffer
**          Pointer to a variable that will receive the address of the reserved
**          space.
**
**      gctSIZE_T * BufferSize
**          Pointer to a variable that will receive the number of bytes
**          available in the command queue.
*/
gceSTATUS
gckCOMMAND_Reserve(
    IN gckCOMMAND Command,
    IN gctUINT32 RequestedBytes,
    OUT gctPOINTER * Buffer,
    OUT gctUINT32 * BufferSize
    )
{
    gceSTATUS status;
    gctUINT32 bytes;
    gctUINT32 requiredBytes;
    gctUINT32 requestedAligned;

    gcmkHEADER_ARG("Command=%p RequestedBytes=%lu", Command, RequestedBytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    if (Command->feType == gcvHW_FE_WAIT_LINK)
    {
        /* Compute aligned number of reuested bytes. */
        requestedAligned = gcmALIGN(RequestedBytes, Command->alignment);

        /* Another WAIT/LINK command sequence will have to be appended after
           the requested area being reserved. Compute the number of bytes
           required for WAIT/LINK at the location after the reserved area. */
        gcmkONERROR(gckWLFE_WaitLink(
            Command->kernel->hardware,
            gcvNULL,
            ~0U,
            Command->offset + requestedAligned,
            &requiredBytes,
            gcvNULL,
            gcvNULL
            ));

        /* Compute total number of bytes required. */
        requiredBytes += requestedAligned;
    }
    else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        requiredBytes = gcmALIGN(RequestedBytes, 16);
    }
    else
    {
        requiredBytes = gcmALIGN(RequestedBytes, 8);
    }

    /* Compute number of bytes available in command queue. */
    bytes = Command->pageSize - Command->offset;

    /* Is there enough space in the current command queue? */
    if (bytes <= requiredBytes)
    {
        /* Create a new command queue. */
        gcmkONERROR(_NewQueue(Command, gcvFALSE));

        /* Recompute the number of bytes in the new kernel command queue. */
        bytes = Command->pageSize - Command->offset;

        /* Still not enough space? */
        if (bytes < requiredBytes)
        {
            /* Rare case, not enough room in command queue. */
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }
    }

    /* Return pointer to empty slot command queue. */
    *Buffer = (gctUINT8 *) Command->logical + Command->offset;

    /* Return number of bytes left in command queue. */
    *BufferSize = bytes;

    /* Success. */
    gcmkFOOTER_ARG("*Buffer=0x%x *BufferSize=%lu", *Buffer, *BufferSize);
    return gcvSTATUS_OK;

OnError:
    /* Return status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_Execute
**
**  Execute a previously reserved command queue by appending a WAIT/LINK command
**  sequence after it and modifying the last WAIT into a LINK command.  The
**  command FIFO mutex will be released whether this function succeeds or not.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object.
**
**      gctSIZE_T RequestedBytes
**          Number of bytes previously reserved.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Execute(
    IN gckCOMMAND Command,
    IN gctUINT32 RequestedBytes
    )
{
    gceSTATUS status;

    gctUINT8_PTR waitLinkLogical;
    gctUINT32 waitLinkAddress;
    gctUINT32 waitLinkOffset;
    gctUINT32 waitLinkBytes;

    gctUINT32 waitOffset;
    gctUINT32 waitBytes;

    gctUINT32 linkLow, linkHigh;

    gctPOINTER execLogical;
    gctUINT32 execAddress;
    gctUINT32 execBytes;

    gcmkHEADER_ARG("Command=%p RequestedBytes=%lu", Command, RequestedBytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Compute offset for WAIT/LINK. */
    waitLinkOffset = Command->offset + RequestedBytes;

    /* Compute number of bytes left in command queue. */
    waitLinkBytes = Command->pageSize - waitLinkOffset;

    /* Compute the location if WAIT/LINK command sequence. */
    waitLinkLogical  = (gctUINT8_PTR) Command->logical  + waitLinkOffset;
    waitLinkAddress  =                Command->address  + waitLinkOffset;

    /* Append WAIT/LINK in command queue. */
    gcmkONERROR(gckWLFE_WaitLink(
        Command->kernel->hardware,
        waitLinkLogical,
        waitLinkAddress,
        waitLinkOffset,
        &waitLinkBytes,
        &waitOffset,
        &waitBytes
        ));


    /* Determine the location to jump to for the command buffer being
    ** scheduled. */
    if (Command->newQueue)
    {
        /* New command queue, jump to the beginning of it. */
        execLogical  = Command->logical;
        execAddress  = Command->address;
        execBytes    = Command->offset + RequestedBytes + waitLinkBytes;

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            0,
            execLogical,
            execBytes
            ));
    }
    else
    {
        /* Still within the preexisting command queue, jump directly to the
           reserved area. */
        execLogical  = (gctUINT8 *) Command->logical  + Command->offset;
        execAddress  =              Command->address  + Command->offset;
        execBytes    = RequestedBytes + waitLinkBytes;

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            Command->offset,
            execLogical,
            execBytes
            ));
    }

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /*
     * Skip link to execAddress.
     * Instead, we directly link to final wait link position.
     */
    gcmkONERROR(gckWLFE_Link(
        Command->kernel->hardware,
        Command->waitPos.logical,
        waitLinkAddress,
        waitLinkBytes,
        &Command->waitPos.size,
        &linkLow,
        &linkHigh
        ));
#else
    /* Convert the last WAIT into a LINK. */
    gcmkONERROR(gckWLFE_Link(
        Command->kernel->hardware,
        Command->waitPos.logical,
        execAddress,
        execBytes,
        &Command->waitPos.size,
        &linkLow,
        &linkHigh
        ));
#endif

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Command->kernel,
        Command->waitPos.videoMem,
        Command->waitPos.offset,
        Command->waitPos.logical,
        Command->waitPos.size
        ));

#if gcdLINK_QUEUE_SIZE
    if (Command->kernel->stuckDump >= gcvSTUCK_DUMP_ALL_COMMAND)
    {
        gcuQUEUEDATA data;

        gcmkVERIFY_OK(gckOS_GetProcessID(&data.linkData.pid));

        data.linkData.start    = execAddress;
        data.linkData.end      = execAddress + execBytes;
        data.linkData.linkLow  = linkLow;
        data.linkData.linkHigh = linkHigh;

        gckQUEUE_Enqueue(&Command->kernel->hardware->linkQueue, &data);
    }
#endif

    gcmkDUMP(Command->os, "#[command: kernel execute]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        execLogical,
        execAddress,
        execBytes
        );

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    gcmkDUMP(Command->os,
             "#[null driver: below command skipped link to 0x%08X 0x%08X]",
             execAddress,
             execBytes);
#endif

    gcmkDUMP(Command->os, "#[link: break prev wait-link]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        Command->waitPos.logical,
        Command->waitPos.address,
        Command->waitPos.size
        );

    /* Update the pointer to the last WAIT. */
    Command->waitPos.videoMem = Command->videoMem;
    Command->waitPos.offset  = waitLinkOffset + waitOffset;
    Command->waitPos.logical = (gctUINT8_PTR)waitLinkLogical  + waitOffset;
    Command->waitPos.address = waitLinkAddress  + waitOffset;
    Command->waitPos.size    = waitBytes;

    /* Update the command queue. */
    Command->offset  += RequestedBytes + waitLinkBytes;
    Command->newQueue = gcvFALSE;

    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(
        Command->kernel->hardware, Command->logical, Command->offset
        ));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckCOMMAND_ExecuteAsync(
    IN gckCOMMAND Command,
    IN gctUINT32 RequestedBytes
    )
{
    gceSTATUS status;
    gckHARDWARE hardware;
    gctBOOL available;
    gctPOINTER execLogical;
    gctUINT32 execAddress;
    gctUINT32 execBytes;

    hardware = Command->kernel->hardware;

    /* Determine the location to jump to for the command buffer being
    ** scheduled. */
    if (Command->newQueue)
    {
        /* New command queue, jump to the beginning of it. */
        execLogical  = Command->logical;
        execAddress  = Command->address;
        execBytes    = Command->offset + RequestedBytes;

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            0,
            execLogical,
            execBytes
            ));
    }
    else
    {
        /* Still within the preexisting command queue, jump directly to the
           reserved area. */
        execLogical  = (gctUINT8 *) Command->logical  + Command->offset;
        execAddress  =              Command->address  + Command->offset;
        execBytes    = RequestedBytes;

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            Command->offset,
            execLogical,
            execBytes
            ));
    }

    /* Acquire a slot. */
    for (;;)
    {
        gcmkONERROR(gckASYNC_FE_ReserveSlot(hardware, &available));

        if (available)
        {
            break;
        }
        else
        {
            gckOS_Delay(Command->os, 1);
        }
    }

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /* Skip submit to hardware for NULL driver. */
    gcmkDUMP(Command->os, "#[null driver: below command is skipped]");
#endif

    gcmkDUMP(Command->os, "#[async-command: kernel execute]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        execLogical,
        execAddress,
        execBytes
        );

#if !gcdNULL_DRIVER
    /* Send descriptor. */
    gckASYNC_FE_Execute(hardware, execAddress, execBytes);
#endif

    /* Update the command queue. */
    Command->offset   += RequestedBytes;
    Command->newQueue  = gcvFALSE;

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckCOMMAND_ExecuteMultiChannel(
    IN gckCOMMAND Command,
    IN gctBOOL Priority,
    IN gctUINT32 ChannelId,
    IN gctUINT32 RequestedBytes
    )
{
    gceSTATUS status;
    gctPOINTER execLogical;
    gctUINT32 execAddress;
    gctUINT32 execBytes;

    /* Determine the location to jump to for the command buffer being
    ** scheduled. */
    if (Command->newQueue)
    {
        /* New command queue, jump to the beginning of it. */
        execLogical  = Command->logical;
        execAddress  = Command->address;
        execBytes    = Command->offset + RequestedBytes;
    }
    else
    {
        /* Still within the preexisting command queue, jump directly to the
           reserved area. */
        execLogical  = (gctUINT8 *) Command->logical  + Command->offset;
        execAddress  =              Command->address  + Command->offset;
        execBytes    = RequestedBytes;
    }

    if (execBytes & 8)
    {
        gctSIZE_T bytes = 8;
        gctUINT8_PTR tail = (gctUINT8_PTR)execLogical + execBytes;

        gckMCFE_Nop(Command->kernel->hardware, tail, &bytes);
        gcmkASSERT(bytes == 8);

        execBytes += 8;
        RequestedBytes += 8;
    }

    if (Command->newQueue)
    {
        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            0,
            execLogical,
            execBytes
            ));
    }
    else
    {
        gcmkONERROR(gckVIDMEM_NODE_CleanCache(
            Command->kernel,
            Command->videoMem,
            Command->offset,
            execLogical,
            execBytes
            ));
    }

#if gcdNULL_DRIVER || gcdCAPTURE_ONLY_MODE
    /* Skip submit to hardware for NULL driver. */
    gcmkDUMP(Command->os, "#[null driver: below command is skipped]");
#endif

    gcmkDUMP(Command->os, "#[mcfe-command: kernel execute]");
    gcmkDUMP_BUFFER(
        Command->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        execLogical,
        execAddress,
        execBytes
        );

    gcmkDUMP(Command->os,
             "@[execute %u %u 0x%08X 0x%08X]",
             ChannelId, Priority, execAddress, execBytes);

#if !gcdNULL_DRIVER
    /* Send descriptor. */
    gcmkONERROR(
        gckMCFE_Execute(Command->kernel->hardware,
                        Priority,
                        ChannelId,
                        execAddress,
                        execBytes));
#endif

    /* Update the command queue. */
    Command->offset   += RequestedBytes;
    Command->newQueue  = gcvFALSE;

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_Stall
**
**  The calling thread will be suspended until the command queue has been
**  completed.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to an gckCOMMAND object.
**
**      gctBOOL FromPower
**          Determines whether the call originates from inside the power
**          management or not.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Stall(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    )
{
    gckOS os;
    gckHARDWARE hardware;
    gckEVENT eventObject;
    gceSTATUS status;
    gctSIGNAL signal = gcvNULL;
    gctUINT timer = 0;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Extract the gckOS object pointer. */
    os = Command->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Extract the gckHARDWARE object pointer. */
    hardware = Command->kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Extract the gckEVENT object pointer. */
    eventObject = Command->kernel->eventObj;
    gcmkVERIFY_OBJECT(eventObject, gcvOBJ_EVENT);

    /* Allocate the signal. */
    gcmkONERROR(gckOS_CreateSignal(os, gcvTRUE, &signal));

    /* Append the EVENT command to trigger the signal. */
    gcmkONERROR(gckEVENT_Signal(eventObject, signal, gcvKERNEL_PIXEL));

    /* Submit the event queue. */
    gcmkONERROR(gckEVENT_Submit(eventObject, gcvTRUE, FromPower, gcvTRUE));

    gcmkDUMP(Command->os, "#[kernel.stall]");

    if (status == gcvSTATUS_CHIP_NOT_READY)
    {
        /* Error. */
        goto OnError;
    }

    do
    {
        /* Wait for the signal. */
        status = gckOS_WaitSignal(os, signal, !FromPower, gcdGPU_ADVANCETIMER);

        if (status == gcvSTATUS_TIMEOUT)
        {
#if gcmIS_DEBUG(gcdDEBUG_CODE)
            gctUINT32 idle;

            /* Read idle register. */
            gcmkVERIFY_OK(gckHARDWARE_GetIdle(
                hardware, gcvFALSE, &idle
                ));

            gcmkTRACE(
                gcvLEVEL_ERROR,
                "%s(%d): idle=%08x",
                __FUNCTION__, __LINE__, idle
                );

            gcmkVERIFY_OK(gckOS_MemoryBarrier(os, gcvNULL));
#endif

            /* Advance timer. */
            timer += gcdGPU_ADVANCETIMER;
        }
        else if (status == gcvSTATUS_INTERRUPTED)
        {
            gcmkONERROR(gcvSTATUS_INTERRUPTED);
        }

    }
    while (gcmIS_ERROR(status));

    /* Bail out on timeout. */
    if (gcmIS_ERROR(status))
    {
        /* Broadcast the stuck GPU. */
        gcmkONERROR(gckOS_Broadcast(
            os, hardware, gcvBROADCAST_GPU_STUCK
            ));
    }

    /* Delete the signal. */
    gcmkVERIFY_OK(gckOS_DestroySignal(os, signal));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (signal != gcvNULL)
    {
        /* Free the signal. */
        gcmkVERIFY_OK(gckOS_DestroySignal(os, signal));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_AttachWaitLinkFECommand(
    IN gckCOMMAND Command,
    OUT gckCONTEXT * Context,
    OUT gctSIZE_T * MaxState,
    OUT gctUINT32 * NumStates,
    IN gctUINT32 ProcessID
    )
{
    gceSTATUS status;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Command=%p", Command);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Acquire the context switching mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        Command->os, Command->mutexContext, gcvINFINITE
        ));
    acquired = gcvTRUE;

    /* Construct a gckCONTEXT object. */
    gcmkONERROR(gckCONTEXT_Construct(
        Command->os,
        Command->kernel->hardware,
        ProcessID,
        Context
        ));

    /* Return the number of states in the context. */
    * MaxState  = (* Context)->maxState;
    * NumStates = (* Context)->numStates;

    /* Release the context switching mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
    acquired = gcvFALSE;

    /* Success. */
    gcmkFOOTER_ARG("*Context=0x%x", *Context);
    return gcvSTATUS_OK;

OnError:
    /* Release mutex. */
    if (acquired)
    {
        /* Release the context switching mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
        acquired = gcvFALSE;
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_Attach
**
**  Attach user process.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to a gckCOMMAND object.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**  OUTPUT:
**
**      gckCONTEXT * Context
**          Pointer to a variable that will receive a pointer to a new
**          gckCONTEXT object.
**
**      gctSIZE_T * StateCount
**          Pointer to a variable that will receive the number of states
**          in the context buffer.
*/
gceSTATUS
gckCOMMAND_Attach(
    IN gckCOMMAND Command,
    OUT gckCONTEXT * Context,
    OUT gctSIZE_T * MaxState,
    OUT gctUINT32 * NumStates,
    IN gctUINT32 ProcessID
    )
{
    gctUINT32 allocationSize;
    gctPOINTER pointer;
    gceSTATUS status;

    if (Command->feType == gcvHW_FE_WAIT_LINK)
    {
        status = _AttachWaitLinkFECommand(Command,
                                        Context,
                                        MaxState,
                                        NumStates,
                                        ProcessID);
    }
    else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        /*
         * For mcfe, we only allocate context which is used to
         * store profile counters.
         */
        allocationSize = gcmSIZEOF(struct _gckCONTEXT);

        /* Allocate the object. */
        gckOS_Allocate(Command->os, allocationSize, &pointer);
        if (!pointer)
        {
            return gcvSTATUS_OUT_OF_MEMORY;
        }
        *Context = pointer;
        /* Reset the entire object. */
        gckOS_ZeroMemory(*Context, allocationSize);

        /* Initialize the gckCONTEXT object. */
        (*Context)->object.type = gcvOBJ_CONTEXT;
        (*Context)->os          = Command->os;
        (*Context)->hardware    = Command->kernel->hardware;
        *MaxState  = 0;
        *NumStates = 0;

        status = gcvSTATUS_OK;
    }
    else
    {
        /* Nothing to do. */
        *Context   = gcvNULL;
        *MaxState  = 0;
        *NumStates = 0;

        status = gcvSTATUS_OK;
    }

    return status;
}

static gceSTATUS
_DetachWaitLinkFECommand(
    IN gckCOMMAND Command,
    IN gckCONTEXT Context
    )
{
    gceSTATUS status;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Command=%p Context=%p", Command, Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Command, gcvOBJ_COMMAND);

    /* Acquire the context switching mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        Command->os, Command->mutexContext, gcvINFINITE
        ));
    acquired = gcvTRUE;

    /* Construct a gckCONTEXT object. */
    gcmkONERROR(gckCONTEXT_Destroy(Context));

    if (Command->currContext == Context)
    {
        /* Detach from gckCOMMAND object if the destoryed context is current context. */
        Command->currContext = gcvNULL;
    }

    /* Release the context switching mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
    acquired = gcvFALSE;

    /* Return the status. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    /* Release mutex. */
    if (acquired)
    {
        /* Release the context switching mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Command->os, Command->mutexContext));
        acquired = gcvFALSE;
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_Detach
**
**  Detach user process.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to a gckCOMMAND object.
**
**      gckCONTEXT Context
**          Pointer to a gckCONTEXT object to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_Detach(
    IN gckCOMMAND Command,
    IN gckCONTEXT Context
    )
{
    if (Command->feType == gcvHW_FE_WAIT_LINK)
    {
        return _DetachWaitLinkFECommand(Command, Context);
    }
    else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        gcmkOS_SAFE_FREE(Context->os, Context);
        return gcvSTATUS_OK;
    }
    else
    {
        /* Nothing to do. */
        return gcvSTATUS_OK;
    }
}

static void
_DumpBuffer(
    IN gctPOINTER Buffer,
    IN gctUINT32 GpuAddress,
    IN gctSIZE_T Size
    )
{
    gctSIZE_T i, line, left;
    gctUINT32_PTR data = Buffer;

    line = Size / 32;
    left = Size % 32;

    for (i = 0; i < line; i++)
    {
        gcmkPRINT("%08X : %08X %08X %08X %08X %08X %08X %08X %08X",
                  GpuAddress, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        data += 8;
        GpuAddress += 8 * 4;
    }

    switch(left)
    {
        case 28:
            gcmkPRINT("%08X : %08X %08X %08X %08X %08X %08X %08X",
                      GpuAddress, data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
            break;
        case 24:
            gcmkPRINT("%08X : %08X %08X %08X %08X %08X %08X",
                      GpuAddress, data[0], data[1], data[2], data[3], data[4], data[5]);
            break;
        case 20:
            gcmkPRINT("%08X : %08X %08X %08X %08X %08X",
                      GpuAddress, data[0], data[1], data[2], data[3], data[4]);
            break;
        case 16:
            gcmkPRINT("%08X : %08X %08X %08X %08X",
                      GpuAddress, data[0], data[1], data[2], data[3]);
            break;
        case 12:
            gcmkPRINT("%08X : %08X %08X %08X",
                      GpuAddress, data[0], data[1], data[2]);
            break;
        case 8:
            gcmkPRINT("%08X : %08X %08X",
                      GpuAddress, data[0], data[1]);
            break;
        case 4:
            gcmkPRINT("%08X : %08X",
                      GpuAddress, data[0]);
            break;
        default:
            break;
    }
}

#if gcdDUMP_TPNN_SUBCOMMAND
typedef struct _gcsRECORD_SUBCOMMAND
{
    gctSTRING name;
    gctUINT type;
    gctUINT reg;
}
gcsRECORD_SUBCOMMAND;

typedef struct _LNode
{
    gctUINT32 type;
    gctUINT32 address;
    gctUINT32 count;
    struct _LNode* next;
}
LNode, *LNodePtr;

gceSTATUS
_createNode(gckOS Os, LNodePtr* node)
{
    gceSTATUS status;

    gcmkONERROR(gckOS_Allocate(
        Os,
        gcmSIZEOF(struct _LNode),
        (gctPOINTER *)node
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
_freeList(gckOS Os, LNode* head)
{
    gceSTATUS status;
    LNode* next;
    LNode* curNode;
    next = head->next;

    while (next)
    {
        curNode = next;
        next = curNode->next;

        gcmkONERROR(gckOS_Free(Os, (gctPOINTER)curNode));
    };

    return gcvSTATUS_OK;
OnError:
    return status;
}

static void
_CheckBuffer(
    IN gckOS Os,
    IN gctPOINTER Buffer,
    IN gctSIZE_T Size,
    IN gcsRECORD_SUBCOMMAND* SubCommand,
    IN gctINT Count,
    IN LNode* ListHead
    )
{
    gctINT i, j, count;
    gctUINT32_PTR data = Buffer;
    LNode* node;

    count = (gctINT)(Size / 4);

    for (i = 0; i < count; i += 2)
    {
        for (j = 0; j < Count; j++)
        {
            if (data[i] == SubCommand[j].reg)
            {
                _createNode(Os, &node);

                node->type = j;
                node->address = data[i + 1];

                ListHead->count++;

                node->next = ListHead->next;
                ListHead->next = node;
            }
        }
    }
}
#endif
/*******************************************************************************
**
**  gckCOMMAND_DumpExecutingBuffer
**
**  Dump the command buffer which GPU is executing.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to a gckCOMMAND object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_DumpExecutingBuffer(
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctUINT32 gpuAddress;
    gctPOINTER entry = gcvNULL;
    gckOS os = Command->os;
    gckKERNEL kernel = Command->kernel;
    gctUINT32 i;
    gckQUEUE queue = &kernel->hardware->linkQueue;
    gctSIZE_T bytes;
    gctUINT32 offset;
    gctPOINTER entryDump;
    gctUINT8 processName[24] = {0};
#if gcdDUMP_TPNN_SUBCOMMAND
    static gcsRECORD_SUBCOMMAND subCommand[] =
    {
        {"NN", 0, 0x08010428},
        {"TP", 1, 0x0801042E},
    };
    gctINT checkCount = gcmCOUNTOF(subCommand);
    LNode* node;
    LNode subCommandList;
    /* reset list count */
    subCommandList.count = 0;
    subCommandList.next = gcvNULL;
#endif

    gcmkPRINT("**************************\n");
    gcmkPRINT("**** COMMAND BUF DUMP ****\n");
    gcmkPRINT("**************************\n");

    gcmkPRINT("  Submitted commit stamp = %lld", Command->commitStamp - 1);
    gcmkPRINT("  Executed commit stamp  = %lld", *(gctUINT64_PTR)Command->fence->logical);

    if (Command->feType != gcvHW_FE_MULTI_CHANNEL)
    {
        gcmkVERIFY_OK(gckOS_ReadRegisterEx(os, kernel->core, 0x664, &gpuAddress));
        gcmkVERIFY_OK(gckOS_ReadRegisterEx(os, kernel->core, 0x664, &gpuAddress));

        gcmkPRINT("DMA Address 0x%08X", gpuAddress);

        /* Find GPU address in video memory list. */
        status = gckVIDMEM_NODE_Find(kernel, gpuAddress, &nodeObject, &offset);

        if (gcmIS_SUCCESS(status) && nodeObject->type == gcvVIDMEM_TYPE_COMMAND)
        {
            gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                kernel,
                nodeObject,
                gcvFALSE,
                gcvFALSE,
                &entryDump
                ));

            gcmkVERIFY_OK(gckVIDMEM_NODE_GetSize(
                kernel,
                nodeObject,
                &bytes
                ));

            gcmkPRINT("Command buffer around 0x%08X:", gpuAddress);

            /* Align to 4096. */
            offset &= 0xfffff000;
            gpuAddress &= 0xfffff000;

            /* Dump max 4096 bytes. */
            bytes = (bytes - offset) > 4096 ? 4096 : (bytes - offset);

            /* Kernel address of page where stall point stay. */
            entryDump = (gctUINT8_PTR)entryDump + offset;

            _DumpBuffer(entryDump, gpuAddress, bytes);

            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                kernel,
                nodeObject,
                0,
                gcvFALSE,
                gcvFALSE
                ));
        }
        else
        {
            gcmkPRINT("Can not find command buffer around 0x%08X.\n", gpuAddress);
        }
    }

    /* new line. */
    gcmkPRINT(" ");

    gcmkPRINT("Kernel command buffers:");

    for (i = 0; i < gcdCOMMAND_QUEUES; i++)
    {
        entry = Command->queues[i].logical;
        gpuAddress = Command->queues[i].address;

        gcmkPRINT("command buffer %d at 0x%08X size %u",
                  i, gpuAddress, Command->pageSize);
        _DumpBuffer(entry, gpuAddress, Command->pageSize);
    }

    /* new line. */
    gcmkPRINT(" ");

    if (queue->count)
    {
        gcmkPRINT("Dump Level is %d, dump %d valid record in link queue:",
                  Command->kernel->stuckDump, queue->count);
    }

    for (i = 0; i < queue->count; i++)
    {
        gcuQUEUEDATA * queueData;
        gckLINKDATA linkData;

        gckQUEUE_GetData(queue, i, &queueData);

        linkData = &queueData->linkData;

        /* Get gpu address of this command buffer. */
        gpuAddress = linkData->start;
        bytes = linkData->end - gpuAddress;

        processName[0] = '\0';
        gckOS_GetProcessNameByPid(linkData->pid, sizeof(processName), processName);

        gcmkPRINT("Link record %d: [%08X - %08X] from command %08X %08X pid %u (%s):",
                  i,
                  linkData->start,
                  linkData->end,
                  linkData->linkLow,
                  linkData->linkHigh,
                  linkData->pid,
                  processName);

        /* Find GPU address in video memory list. */
        status = gckVIDMEM_NODE_Find(kernel, gpuAddress, &nodeObject, &offset);

        if (gcmIS_SUCCESS(status) && nodeObject->type == gcvVIDMEM_TYPE_COMMAND)
        {
            gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                kernel,
                nodeObject,
                gcvFALSE,
                gcvFALSE,
                &entryDump
                ));

            /* Kernel address of page where stall point stay. */
            entryDump = (gctUINT8_PTR)entryDump + offset;

#if gcdDUMP_TPNN_SUBCOMMAND
            _CheckBuffer(kernel->os, entryDump, bytes, subCommand, checkCount, &subCommandList);
#endif
            _DumpBuffer(entryDump, gpuAddress, bytes);

            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                kernel,
                nodeObject,
                0,
                gcvFALSE,
                gcvFALSE
                ));
        }
        else
        {
            gcmkPRINT("Not found");
        }

        /* new line. */
        gcmkPRINT(" ");
    }

#if gcdDUMP_TPNN_SUBCOMMAND
    gcmkPRINT("Sub command:");
    node = subCommandList.next;

    while (node)
    {
        status = gckVIDMEM_NODE_Find(kernel, node->address, &nodeObject, &offset);

        if (gcmIS_SUCCESS(status))
        {
            gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                kernel,
                nodeObject,
                gcvFALSE,
                gcvFALSE,
                &entryDump
                ));

            /* Kernel address of page where stall point stay. */
            entryDump = (gctUINT8_PTR)entryDump + offset;

            gcmkVERIFY_OK(gckVIDMEM_NODE_GetSize(kernel, nodeObject, &bytes));

            bytes -= offset;

            gcmkPRINT("%s: %08X sub command:", subCommand[node->type].name, node->address);

            _DumpBuffer(entryDump, node->address, bytes);

            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                kernel,
                nodeObject,
                0,
                gcvFALSE,
                gcvFALSE
                ));
        }
        else
        {
            gcmkPRINT("%08X sub command not found", node->address);
        }

        /* new line */
        gcmkPRINT(" ");

        node = node->next;
    };

    _freeList(kernel->os, &subCommandList);
#endif

    return gcvSTATUS_OK;

OnError:
    return status;
}

#if gcdENABLE_SW_PREEMPTION
static gceSTATUS
_PreemptHandlePatchListSingle(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    IN gcsHAL_PATCH_LIST * PatchList,
    OUT gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status;
    gctUINT32 index = 0;
    gctUINT32 count = 0;
    gctUINT32 itemSize = 0;
    gctUINT32 batchCount = 0;
    gcsPATCH_ARRAY *patchArray = (gcsPATCH_ARRAY *)gcmUINT64_TO_PTR(PatchList->patchArray);

    gcmkHEADER_ARG("Command=%p CommandBuffer=%p PatchList=%p type=%d",
                   Command, CommandBuffer, PatchList, PatchList->type);

    if (PatchList->type >= gcmCOUNTOF(_PatchItemSize) || PatchList->type >= gcmCOUNTOF(patchHandler) || patchArray == gcvNULL)
    {
        /* Exceeds buffer max size. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    itemSize = _PatchItemSize[PatchList->type];

    batchCount = (gctUINT32)(sizeof(gctUINT64) * 32 / itemSize);

    handler = patchHandler[PatchList->type];

    while (index < PatchList->count)
    {
        gctUINT i;
        gctUINT8_PTR ptr;

        /* Determine batch count, don't handle too many in one batch. */
        count = PatchList->count - index;

        if (count > batchCount)
        {
            count = batchCount;
        }

        /* Advance to next batch. */
        index += count;

        ptr = (gctUINT8_PTR)patchArray->kArray;

        for (i = 0; i < count; i++)
        {
            /* Call handler. */
            gcmkONERROR(
                handler(Command, CommandBuffer, ptr, PatchListVar));

            /* Advance to next patch. */
            ptr += itemSize;
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;

}

static gceSTATUS
_PreemptHandlePatchList(
    IN gckCOMMAND Command,
    IN gcsHAL_COMMAND_LOCATION * CommandBuffer,
    OUT gcsPATCH_LIST_VARIABLE * PatchListVar
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsHAL_PATCH_LIST *patchList;

    patchList = (gcsHAL_PATCH_LIST *)gcmUINT64_TO_PTR(CommandBuffer->patchHead);

    while (patchList)
    {
        gcmkONERROR(_PreemptHandlePatchListSingle(
            Command,
            CommandBuffer,
            patchList,
            PatchListVar));

        patchList = (gcsHAL_PATCH_LIST *)gcmUINT64_TO_PTR(patchList->next);
    }

OnError:
    return status;
}

/*******************************************************************************
**
**  gckCOMMAND_PreemptCommit
**
**  Commit command in preemption mode.
**
**  INPUT:
**
**      gckCOMMAND Command
**          Pointer to a gckCOMMAND object.
**
**      gckPREEMPT_COMMIT PreemptCommit
**          The preempt commit.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCOMMAND_PreemptCommit(
    IN gckCOMMAND Command,
    IN gckPREEMPT_COMMIT PreemptCommit
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctBOOL contextSwitched = gcvFALSE;
    gcsHAL_COMMAND_LOCATION *cmdLoc = PreemptCommit->cmdLoc;
    gckCONTEXT context = PreemptCommit->context;
    gcsSTATE_DELTA_PTR delta = PreemptCommit->delta;
    gcsPATCH_LIST_VARIABLE patchListVar = {0, 0};
    gctBOOL commitEntered = gcvFALSE;

    gcmkHEADER();

    /* Acquire the command queue. */
    gcmkONERROR(gckCOMMAND_EnterCommit(Command, gcvFALSE));
    commitEntered = gcvTRUE;

    do
    {
        gcmkVERIFY_OK(_PreemptHandlePatchList(
            Command,
            cmdLoc,
            &patchListVar
            ));

        if (Command->feType == gcvHW_FE_WAIT_LINK)
        {
            status = _CommitWaitLinkOnce(Command,
                                         context,
                                         cmdLoc,
                                         delta,
                                         PreemptCommit->pid,
                                         PreemptCommit->shared,
                                         &contextSwitched,
                                         PreemptCommit,
                                         gcvTRUE,
                                         0);

        }
        else if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
        {
            status = _CommitMultiChannelOnce(Command, context, cmdLoc);
        }
        else
        {
            gcmkPRINT("Don't enable SW preemption for aysnc FE.\n");

            gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        }

        if (status != gcvSTATUS_INTERRUPTED)
        {
            gcmkONERROR(status);
        }

        context = gcvNULL;
        delta   = gcvNULL;

        cmdLoc  = (gcsHAL_COMMAND_LOCATION *)gcmUINT64_TO_PTR(cmdLoc->next);
    }
    while(cmdLoc);

    /* Release the command queue. */
    gcmkONERROR(gckCOMMAND_ExitCommit(Command, gcvFALSE));
    commitEntered = gcvFALSE;

    if (Command->feType == gcvHW_FE_MULTI_CHANNEL)
    {
        /*
         * Semphore synchronization.
         *
         * Here we blindly sync dirty other channels to the system channel.
         * The scenario to sync channels to the system channel:
         * 1. Need to sync channels who sent semaphores.
         * 2. Need to sync dirty channels when event(interrupt) is to sent.
         * 3. Need to sync dirty channels when system channel need run something
         *    such as flush mmu.
         *
         * When power management is on, blindly sync dirty channels is OK because
         * there's always a event(intrrupt).
         *
         * The only condition we sync more than needed is:
         * a. power manangement is off.
         * b. no user event is attached when commit.
         * c. no user event is to be submitted in next ioctl.
         * That's a rare condition.
         *
         * Conclusion is that, blindly sync dirty channels is a good choice for
         * now.
         */
        gcmkONERROR(_SyncToSystemChannel(Command, Command->dirtyChannel, gcvTRUE));
    }

OnError:
    if (commitEntered)
    {
        /* Release the command queue mutex. */
        gcmkVERIFY_OK(gckCOMMAND_ExitCommit(Command, gcvFALSE));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}
#endif
