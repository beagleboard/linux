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


#include "gc_hal.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_context.h"

/******************************************************************************\
******************************** Debugging Macro *******************************
\******************************************************************************/

/* Zone used for header/footer. */
#define _GC_OBJ_ZONE    gcvZONE_HARDWARE


/******************************************************************************\
************************** Context State Buffer Helpers ************************
\******************************************************************************/

#define _STATE(reg)                                                            \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT(reg, count)                                               \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT_OFFSET(reg, offset, count)                                \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + offset, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_MIRROR_COUNT(reg, mirror, count)                                \
    _StateMirror(\
        Context, \
        reg ## _Address >> 2, \
        count, \
        mirror ## _Address >> 2                                                \
        )

#define _STATE_HINT(reg)                                                       \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_HINT_BLOCK(reg, block, count)                                   \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + (block << reg ## _BLK), \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_COUNT_OFFSET_HINT(reg, offset, count)                           \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + offset, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_X(reg)                                                          \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvTRUE, gcvFALSE                                                      \
        )

#define _STATE_INIT_VALUE(reg, value)                                          \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        value, \
        reg ## _Count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_INIT_VALUE_OFFSET(reg, offset, value)                           \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + offset, \
        value, \
        1, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_INIT_VALUE_BLOCK(reg, value, block, count)                      \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + (block << reg ## _BLK), \
        value, \
        count, \
        gcvFALSE, gcvFALSE                                                     \
        )


#define _CLOSE_RANGE()                                                         \
    _TerminateStateBlock(Context, index)

#define _ENABLE(reg, field)                                                    \
    do                                                                         \
    {                                                                          \
        if (gcmVERIFYFIELDVALUE(data, reg, MASK_ ## field, ENABLED))           \
        {                                                                      \
            enable |= gcmFIELDMASK(reg, field);                                \
        }                                                                      \
    }                                                                          \
    while (gcvFALSE)

#define _BLOCK_COUNT(reg)                                                      \
    ((reg ## _Count) >> (reg ## _BLK))


/******************************************************************************\
*********************** Support Functions and Definitions **********************
\******************************************************************************/

#define gcdSTATE_MASK \
    (gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP) | 0xC0FFEE)






#if gcdENABLE_SW_PREEMPTION
typedef struct {
    gctUINT inputBase;
    gctUINT count;
    gctUINT outputBase;
}
gcsSTATEMIRROR;

const gcsSTATEMIRROR mirroredStates[] =
{
    {0x5800, 0x300, 0x5B00},
    {0x5600, 0x100, 0x5700},
    {0xD800, 0x140, 0xD000},
};

gctUINT mirroredStatesCount = 0;

static gceSTATUS
_ResetDelta(
    IN gcsSTATE_DELTA_PTR StateDelta
    )
{
    /* Not attached yet, advance the ID. */
    StateDelta->id += 1;

    /* Did ID overflow? */
    if (StateDelta->id == 0)
    {
        /* Reset the map to avoid erroneous ID matches. */
        gckOS_ZeroMemory(gcmUINT64_TO_PTR(StateDelta->mapEntryID), StateDelta->mapEntryIDSize);

        /* Increment the main ID to avoid matches after reset. */
        StateDelta->id += 1;
    }

    /* Reset the vertex element count. */
    StateDelta->elementCount = 0;

    /* Reset the record count. */
    StateDelta->recordCount = 0;

    /* Success. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_DestroyDelta(
    IN gckCONTEXT Context,
    IN gcsSTATE_DELTA_PTR delta
)
{
    gctUINT_PTR mapEntryIndex = gcmUINT64_TO_PTR(delta->mapEntryIndex);
    gctUINT_PTR mapEntryID = gcmUINT64_TO_PTR(delta->mapEntryID);
    gcsSTATE_DELTA_RECORD_PTR recordArray = gcmUINT64_TO_PTR(delta->recordArray);
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER();

    /* Free map index array. */
    if (mapEntryIndex != gcvNULL)
    {
        gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, mapEntryIndex));
    }

    /* Allocate map ID array. */
    if (mapEntryID != gcvNULL)
    {
        gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, mapEntryID));
    }

    /* Free state record array. */
    if (recordArray != gcvNULL)
    {
        gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, recordArray));
    }

    gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, delta));

OnError:

    /* Return the status. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static gceSTATUS
_AllocateDelta(
    IN gckCONTEXT Context,
    OUT gcsSTATE_DELTA_PTR * Delta
    )
{
    gckCONTEXT context = Context;
    gckOS os = context->os;
    gceSTATUS status;
    gcsSTATE_DELTA_PTR delta = gcvNULL;
    gctPOINTER pointer = gcvNULL;

    gcmkHEADER();

    if (context->maxState == 0)
    {
         *Delta = NULL;
         gcmkFOOTER_NO();
         return gcvSTATUS_OK;
    }

    /* Allocate the state delta structure. */
    gcmkONERROR(gckOS_Allocate(
        os, gcmSIZEOF(gcsSTATE_DELTA), (gctPOINTER *) &delta
        ));

    /* Reset the context buffer structure. */
    gckOS_ZeroMemory(delta, gcmSIZEOF(gcsSTATE_DELTA));

    if (context->maxState > 0)
    {
        /* Compute UINT array size. */
        gctSIZE_T bytes = gcmSIZEOF(gctUINT) * context->maxState;

        /* Allocate map ID array. */
        gcmkONERROR(gckOS_Allocate(
            os, bytes, &pointer
            ));

        delta->mapEntryID = gcmPTR_TO_UINT64(pointer);

        /* Set the map ID size. */
        delta->mapEntryIDSize = (gctUINT32)bytes;

        /* Reset the record map. */
        gckOS_ZeroMemory(gcmUINT64_TO_PTR(delta->mapEntryID), bytes);

        /* Allocate map index array. */
        gcmkONERROR(gckOS_Allocate(
            os, bytes, &pointer
            ));

        delta->mapEntryIndex = gcmPTR_TO_UINT64(pointer);

    }

    if (context->numStates > 0)
    {
        /* Allocate state record array. */
        gcmkONERROR(gckOS_Allocate(
            os,
            gcmSIZEOF(gcsSTATE_DELTA_RECORD) * context->numStates,
            &pointer
            ));

        delta->recordArray = gcmPTR_TO_UINT64(pointer);
    }

    /* Reset the new state delta. */
    _ResetDelta(delta);

    *Delta = delta;

    gcmkFOOTER();
    return status;

OnError:
    if (delta)
    {
        _DestroyDelta(Context, delta);
    }

    gcmkFOOTER_NO();
    return status;
}
#endif

static gctUINT32
_SwitchPipe(
    IN gckCONTEXT Context,
    IN gctUINT32 Index,
    IN gcePIPE_SELECT Pipe
    )
{
    gctUINT32 slots = 2;

    if (Context->buffer != gcvNULL)
    {
        gctUINT32_PTR buffer;

        /* Address correct index. */
        buffer = Context->buffer->logical + Index;

        /* LoadState(AQPipeSelect, 1), pipe. */
        *buffer++
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E00) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

        *buffer
            = (Pipe == gcvPIPE_2D)
                ? 0x1
                : 0x0;
    }

    Context->pipeSelectBytes = slots * gcmSIZEOF(gctUINT32);

    return slots;
}



static gceSTATUS
_InitializeNoShaderAndPixelEngine(
    IN gckCONTEXT Context
    )
{
    gctUINT32_PTR buffer;
    gctUINT32 index;


    gckHARDWARE hardware;

    gcmkHEADER();

    hardware = Context->hardware;

    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Reset the buffer index. */
    index = 0;

    /* Reset the last state address. */
    Context->lastAddress = ~0U;

    /* Get the buffer pointer. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;


    /**************************************************************************/
    /* Build 2D states. *******************************************************/



    /**************************************************************************/
    /* Link to another address. ***********************************************/

    Context->linkIndex3D = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        buffer[index + 1]
            = 0;
    }

    index += 2;

    /* Store the end of the context buffer. */
    Context->bufferSize = index * gcmSIZEOF(gctUINT32);


    /**************************************************************************/
    /* Pipe switch for the case where neither 2D nor 3D are used. *************/

    /* Store the 3D entry index. */
    Context->entryOffsetXDFrom2D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    /* Store the location of the link. */
    Context->linkIndexXD = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        buffer[index + 1]
            = 0;
    }

    index += 2;


    /**************************************************************************/
    /* Save size for buffer. **************************************************/

    Context->totalSize = index * gcmSIZEOF(gctUINT32);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

}

static gceSTATUS
_InitializeContextBuffer(
    IN gckCONTEXT Context
    )
{
    gctUINT32_PTR buffer = gcvNULL;
    gctUINT32 index;


    gckHARDWARE hardware;

    gcmkHEADER();

    hardware = Context->hardware;

    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    if (!hardware->options.hasShader)
    {
        return _InitializeNoShaderAndPixelEngine(Context);
    }

    /* Reset the buffer index. */
    index = 0;

    /* Reset the last state address. */
    Context->lastAddress = ~0U;

    /* Get the buffer pointer. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;


    /**************************************************************************/
    /* Build 2D states. *******************************************************/



    /**************************************************************************/
    /* Link to another address. ***********************************************/

    Context->linkIndex3D = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        buffer[index + 1]
            = 0;
    }

    index += 2;

    /* Store the end of the context buffer. */
    Context->bufferSize = index * gcmSIZEOF(gctUINT32);


    /**************************************************************************/
    /* Pipe switch for the case where neither 2D nor 3D are used. *************/

    /* Store the 3D entry index. */
    Context->entryOffsetXDFrom2D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    /* Store the location of the link. */
    Context->linkIndexXD = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        buffer[index + 1]
            = 0;
    }

    index += 2;


    /**************************************************************************/
    /* Save size for buffer. **************************************************/

    Context->totalSize = index * gcmSIZEOF(gctUINT32);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static gceSTATUS
_DestroyContext(
    IN gckCONTEXT Context
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (Context != gcvNULL)
    {
        gcsCONTEXT_PTR bufferHead;

#if gcdENABLE_SW_PREEMPTION
        gcsSTATE_DELTA_PTR delta, next;

        /* Free state deltas. */
        for (Context->delta = Context->deltaHead; Context->delta != gcvNULL;)
        {
            delta = Context->delta;

            /* Get the next delta. */
            next = gcmUINT64_TO_PTR(delta->next);

            /* Last item? */
            if (next == Context->deltaHead)
            {
                next = gcvNULL;
            }

            _DestroyDelta(Context, delta);

            /* Remove from the list. */
            Context->delta = next;
        }

        gcmkVERIFY_OK(gckCONTEXT_DestroyPrevDelta(Context));
#endif

        /* Free context buffers. */
        for (bufferHead = Context->buffer; Context->buffer != gcvNULL;)
        {
            /* Get a shortcut to the current buffer. */
            gcsCONTEXT_PTR buffer = Context->buffer;

            /* Get the next buffer. */
            gcsCONTEXT_PTR next = buffer->next;

            /* Last item? */
            if (next == bufferHead)
            {
                next = gcvNULL;
            }

            /* Destroy the signal. */
            if (buffer->signal != gcvNULL)
            {
                gcmkONERROR(gckOS_DestroySignal(
                    Context->os, buffer->signal
                    ));

                buffer->signal = gcvNULL;
            }

            /* Free state delta map. */
            if (buffer->logical != gcvNULL)
            {
                gckKERNEL kernel = Context->hardware->kernel;

#if gcdCAPTURE_ONLY_MODE
                gceDATABASE_TYPE dbType;
                gctUINT32 processID;
#endif

                /* End cpu access. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                    kernel,
                    buffer->videoMem,
                    0,
                    gcvFALSE,
                    gcvFALSE
                    ));

                /* Synchronized unlock. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
                    kernel,
                    buffer->videoMem,
                    0,
                    gcvNULL
                    ));

#if gcdCAPTURE_ONLY_MODE
                /* Encode surface type and pool to database type. */
                dbType = gcvDB_VIDEO_MEMORY
                       | (gcvVIDMEM_TYPE_GENERIC << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
                       | (buffer->videoMem->pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

                gcmkONERROR(gckOS_GetProcessID(&processID));

                gcmkONERROR(
                    gckKERNEL_RemoveProcessDB(kernel,
                        processID,
                        dbType,
                        buffer->videoMem));
#endif

                /* Free video memory. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
                    kernel,
                    buffer->videoMem
                    ));

                buffer->logical = gcvNULL;
            }

            /* Free context buffer. */
            gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, buffer));

            /* Remove from the list. */
            Context->buffer = next;
        }

        /* Mark the gckCONTEXT object as unknown. */
        Context->object.type = gcvOBJ_UNKNOWN;

        /* Free the gckCONTEXT object. */
        gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, Context));
    }

OnError:
    return status;
}

static gceSTATUS
_AllocateContextBuffer(
    IN gckCONTEXT Context,
    IN gcsCONTEXT_PTR Buffer
    )
{
    gceSTATUS status;
    gckKERNEL kernel = Context->hardware->kernel;
    gcePOOL pool = gcvPOOL_DEFAULT;
    gctSIZE_T totalSize = Context->totalSize;
    gctUINT32 allocFlag = 0;

#if gcdCAPTURE_ONLY_MODE
    gceDATABASE_TYPE dbType;
    gctUINT32 processID;
#endif

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag = gcvALLOC_FLAG_CACHEABLE;
#endif

    /* Allocate video memory node for command buffers. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        kernel,
        64,
        gcvVIDMEM_TYPE_COMMAND,
        allocFlag,
        &totalSize,
        &pool,
        &Buffer->videoMem
        ));

#if gcdCAPTURE_ONLY_MODE
    gcmkONERROR(gckVIDMEM_HANDLE_Allocate(kernel, Buffer->videoMem, &Context->buffer->handle));

    /* Encode surface type and pool to database type. */
    dbType = gcvDB_VIDEO_MEMORY
           | (gcvVIDMEM_TYPE_GENERIC << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
           | (pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    gcmkONERROR(gckOS_GetProcessID(&processID));

    /* Record in process db. */
    gcmkONERROR(
            gckKERNEL_AddProcessDB(kernel,
                                   processID,
                                   dbType,
                                   Buffer->videoMem,
                                   gcvNULL,
                                   totalSize));
#endif

    /* Lock for GPU access. */
    gcmkONERROR(gckVIDMEM_NODE_Lock(
        kernel,
        Buffer->videoMem,
        &Buffer->address
        ));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        kernel,
        Buffer->videoMem,
        gcvFALSE,
        gcvFALSE,
        (gctPOINTER *)&Buffer->logical
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

/******************************************************************************\
**************************** Context Management API ****************************
\******************************************************************************/

/******************************************************************************\
**
**  gckCONTEXT_Construct
**
**  Construct a new gckCONTEXT object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**      gckHARDWARE Hardware
**          Pointer to gckHARDWARE object.
**
**  OUTPUT:
**
**      gckCONTEXT * Context
**          Pointer to a variable thet will receive the gckCONTEXT object
**          pointer.
*/
gceSTATUS
gckCONTEXT_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT32 ProcessID,
    OUT gckCONTEXT * Context
    )
{
    gceSTATUS status;
    gckCONTEXT context = gcvNULL;
    gctUINT32 allocationSize;
    gctUINT i;
    gctPOINTER pointer = gcvNULL;

    gcmkHEADER_ARG("Os=%p Hardware=%p", Os, Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Context != gcvNULL);


    /**************************************************************************/
    /* Allocate and initialize basic fields of gckCONTEXT. ********************/

    /* The context object size. */
    allocationSize = gcmSIZEOF(struct _gckCONTEXT);

    /* Allocate the object. */
    gcmkONERROR(gckOS_Allocate(
        Os, allocationSize, &pointer
        ));

    context = pointer;

    /* Reset the entire object. */
    gcmkONERROR(gckOS_ZeroMemory(context, allocationSize));

    /* Initialize the gckCONTEXT object. */
    context->object.type = gcvOBJ_CONTEXT;
    context->os          = Os;
    context->hardware    = Hardware;


    context->entryPipe = gcvPIPE_2D;
    context->exitPipe  = gcvPIPE_2D;

    /* Get the command buffer requirements. */
    gcmkONERROR(gckHARDWARE_QueryCommandBuffer(
        Hardware,
        gcvENGINE_RENDER,
        &context->alignment,
        &context->reservedHead,
        gcvNULL
        ));

    /**************************************************************************/
    /* Get the size of the context buffer. ************************************/

    gcmkONERROR(_InitializeContextBuffer(context));

    if (context->maxState > 0)
    {
        /**************************************************************************/
        /* Allocate and reset the state mapping table. ****************************/
        if (context->hardware->kernel->command->stateMap == gcvNULL)
        {
            /* Allocate the state mapping table. */
            gcmkONERROR(gckOS_Allocate(
                Os,
                gcmSIZEOF(gcsSTATE_MAP) * context->maxState,
                &pointer
                ));

            context->map = pointer;

            /* Zero the state mapping table. */
            gcmkONERROR(gckOS_ZeroMemory(
                context->map, gcmSIZEOF(gcsSTATE_MAP) * context->maxState
                ));

            context->hardware->kernel->command->stateMap = pointer;
        }
        else
        {
            context->map = context->hardware->kernel->command->stateMap;
        }
    }

    /**************************************************************************/
    /* Allocate the context and state delta buffers. **************************/

    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; i += 1)
    {
        /* Allocate a context buffer. */
        gcsCONTEXT_PTR buffer;

        /* Allocate the context buffer structure. */
        gcmkONERROR(gckOS_Allocate(
            Os,
            gcmSIZEOF(gcsCONTEXT),
            &pointer
            ));

        buffer = pointer;

        /* Reset the context buffer structure. */
        gcmkVERIFY_OK(gckOS_ZeroMemory(
            buffer, gcmSIZEOF(gcsCONTEXT)
            ));

        /* Append to the list. */
        if (context->buffer == gcvNULL)
        {
            buffer->next    = buffer;
            context->buffer = buffer;
        }
        else
        {
            buffer->next          = context->buffer->next;
            context->buffer->next = buffer;
        }

        /* Set the number of delta in the order of creation. */
#if gcmIS_DEBUG(gcdDEBUG_CODE)
        buffer->num = i;
#endif

        /* Create the busy signal. */
        gcmkONERROR(gckOS_CreateSignal(
            Os, gcvFALSE, &buffer->signal
            ));

        /* Set the signal, buffer is currently not busy. */
        gcmkONERROR(gckOS_Signal(
            Os, buffer->signal, gcvTRUE
            ));

        /* Create a new physical context buffer. */
        gcmkONERROR(_AllocateContextBuffer(
            context, buffer
            ));

        /* Set gckEVENT object pointer. */
        buffer->eventObj = Hardware->kernel->eventObj;

        /* Set the pointers to the LINK commands. */
        if (context->linkIndex2D != 0)
        {
            buffer->link2D = &buffer->logical[context->linkIndex2D];
        }

        if (context->linkIndex3D != 0)
        {
            buffer->link3D = &buffer->logical[context->linkIndex3D];
        }

        if (context->linkIndexXD != 0)
        {
            gctPOINTER xdLink;
            gctUINT32 xdEntryAddress;
            gctUINT32 xdEntrySize;
            gctUINT32 linkBytes;

            /* Determine LINK parameters. */
            xdLink
                = &buffer->logical[context->linkIndexXD];

            xdEntryAddress
                = buffer->address
                + context->entryOffsetXDFrom3D;

            xdEntrySize
                = context->bufferSize
                - context->entryOffsetXDFrom3D;

            /* Query LINK size. */
            gcmkONERROR(gckWLFE_Link(
                Hardware, gcvNULL, 0, 0, &linkBytes, gcvNULL, gcvNULL
                ));

            /* Generate a LINK. */
            gcmkONERROR(gckWLFE_Link(
                Hardware,
                xdLink,
                xdEntryAddress,
                xdEntrySize,
                &linkBytes,
                gcvNULL,
                gcvNULL
                ));
        }
    }

    /**************************************************************************/
    /* Initialize the context buffers. ****************************************/

    /* Initialize the current context buffer. */
    gcmkONERROR(_InitializeContextBuffer(context));

#if gcdENABLE_SW_PREEMPTION
    if (context->maxState > 0 && context->numStates > 0)
    {
        for (i = 0; i < gcdCONTEXT_BUFFER_COUNT + 1; i += 1)
        {
            /* Allocate a state delta. */
            gcsSTATE_DELTA_PTR delta = gcvNULL;
            gcsSTATE_DELTA_PTR prev;

            /* Allocate the state delta structure. */
            _AllocateDelta(context, &delta);

            /* Append to the list. */
            if (context->delta == gcvNULL)
            {
                delta->prev = gcmPTR_TO_UINT64(delta);
                delta->next = gcmPTR_TO_UINT64(delta);
                context->deltaHead = context->delta = delta;
            }
            else
            {
                delta->next = gcmPTR_TO_UINT64(context->delta);
                delta->prev = context->delta->prev;

                prev = gcmUINT64_TO_PTR(context->delta->prev);
                prev->next = gcmPTR_TO_UINT64(delta);
                context->delta->prev = gcmPTR_TO_UINT64(delta);
            }
        }
    }

    if (gckHARDWARE_IsFeatureAvailable(context->hardware, gcvFEATURE_HALTI5) &&
        !(gckHARDWARE_IsFeatureAvailable(context->hardware, gcvFEATURE_SMALL_BATCH) && context->hardware->options.smallBatch))
    {
        mirroredStatesCount = sizeof(mirroredStates) / sizeof(mirroredStates[0]);
    }

    context->prevRecordArray = gcvNULL;
    context->prevMapEntryID = gcvNULL;
    context->prevMapEntryIndex = gcvNULL;
    context->prevDeltaPtr = gcvNULL;
#endif

    /* Make all created contexts equal. */
    {
        gcsCONTEXT_PTR currContext, tempContext;

        /* Set the current context buffer. */
        currContext = context->buffer;

        /* Get the next context buffer. */
        tempContext = currContext->next;

        /* Loop through all buffers. */
        while (tempContext != currContext)
        {
            if (tempContext == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_NOT_FOUND);
            }

            /* Copy the current context. */
            gckOS_MemCopy(
                tempContext->logical,
                currContext->logical,
                context->totalSize
                );

            /* Get the next context buffer. */
            tempContext = tempContext->next;
        }
    }

    /* Return pointer to the gckCONTEXT object. */
    *Context = context;

    /* Success. */
    gcmkFOOTER_ARG("*Context=0x%08X", *Context);
    return gcvSTATUS_OK;

OnError:
    /* Roll back on error. */
    gcmkVERIFY_OK(_DestroyContext(context));

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/******************************************************************************\
**
**  gckCONTEXT_Destroy
**
**  Destroy a gckCONTEXT object.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCONTEXT_Destroy(
    IN gckCONTEXT Context
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Context=%p", Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    /* Destroy the context and all related objects. */
    status = _DestroyContext(Context);

    /* Success. */
    gcmkFOOTER_NO();
    return status;
}

/******************************************************************************\
**
**  gckCONTEXT_Update
**
**  Merge all pending state delta buffers into the current context buffer.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**      gcsSTATE_DELTA_PTR StateDelta
**          Pointer to the state delta.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCONTEXT_Update(
    IN gckCONTEXT Context,
    IN gctUINT32 ProcessID,
    IN gcsSTATE_DELTA_PTR StateDelta
    )
{
    return gcvSTATUS_OK;
}

gceSTATUS
gckCONTEXT_MapBuffer(
    IN gckCONTEXT Context,
    OUT gctUINT64 *Logicals,
    OUT gctUINT32 *Bytes
    )
{
    gceSTATUS status;
    int i = 0;
    gckKERNEL kernel = Context->hardware->kernel;
    gctPOINTER logical;
    gcsCONTEXT_PTR buffer;

    gcmkHEADER_ARG("Context=%p", Context);

    buffer = Context->buffer;

    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; i++)
    {
        /* Lock for userspace CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
            kernel,
            buffer->videoMem,
            gcvFALSE,
            gcvTRUE,
            &logical
            ));

        Logicals[i] = gcmPTR_TO_UINT64(logical);
        buffer = buffer->next;
    }

    *Bytes = (gctUINT)Context->totalSize;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

#if gcdENABLE_SW_PREEMPTION
static void
_CopyDelta(
    IN gcsSTATE_DELTA_PTR DstDelta,
    IN gcsSTATE_DELTA_PTR SrcDelta
    )
{
    DstDelta->recordCount = SrcDelta->recordCount;

    if (DstDelta->recordCount)
    {
        gckOS_MemCopy(
            gcmUINT64_TO_PTR(DstDelta->recordArray),
            gcmUINT64_TO_PTR(SrcDelta->recordArray),
            gcmSIZEOF(gcsSTATE_DELTA_RECORD) * DstDelta->recordCount
            );
    }

    if (SrcDelta->mapEntryIDSize)
    {
        gckOS_MemCopy(
            gcmUINT64_TO_PTR(DstDelta->mapEntryID),
            gcmUINT64_TO_PTR(SrcDelta->mapEntryID),
            SrcDelta->mapEntryIDSize
            );

        gckOS_MemCopy(
            gcmUINT64_TO_PTR(DstDelta->mapEntryIndex),
            gcmUINT64_TO_PTR(SrcDelta->mapEntryIndex),
            SrcDelta->mapEntryIDSize
            );
    }

    DstDelta->mapEntryIDSize = SrcDelta->mapEntryIDSize;
    DstDelta->id = SrcDelta->id;
    DstDelta->elementCount = SrcDelta->elementCount;
}

static void
_MergeDelta(
    IN gcsSTATE_DELTA_PTR StateDelta,
    IN gctUINT32 Address,
    IN gctUINT32 Mask,
    IN gctUINT32 Data
    )
{
    gcsSTATE_DELTA_RECORD_PTR recordArray;
    gcsSTATE_DELTA_RECORD_PTR recordEntry;
    gctUINT32_PTR mapEntryID;
    gctUINT32_PTR mapEntryIndex;
    gctUINT deltaID;
    gctUINT32 i;

    if (!StateDelta)
    {
        return;
    }

    /* Get the current record array. */
    recordArray = (gcsSTATE_DELTA_RECORD_PTR)(gcmUINT64_TO_PTR(StateDelta->recordArray));

    /* Get shortcuts to the fields. */
    deltaID       = StateDelta->id;
    mapEntryID    = (gctUINT32_PTR)(gcmUINT64_TO_PTR(StateDelta->mapEntryID));
    mapEntryIndex = (gctUINT32_PTR)(gcmUINT64_TO_PTR(StateDelta->mapEntryIndex));

    gcmkASSERT(Address < (StateDelta->mapEntryIDSize / gcmSIZEOF(gctUINT)));

    for (i = 0; i < mirroredStatesCount; i++)
    {
        if ((Address >= mirroredStates[i].inputBase) &&
            (Address < (mirroredStates[i].inputBase + mirroredStates[i].count)))
        {
            Address = mirroredStates[i].outputBase + (Address - mirroredStates[i].inputBase);
            break;
        }
    }

    /* Has the entry been initialized? */
    if (mapEntryID[Address] != deltaID)
    {
        /* No, initialize the map entry. */
        mapEntryID    [Address] = deltaID;
        mapEntryIndex [Address] = StateDelta->recordCount;

        /* Get the current record. */
        recordEntry = &recordArray[mapEntryIndex[Address]];

        /* Add the state to the list. */
        recordEntry->address = Address;
        recordEntry->mask    = Mask;
        recordEntry->data    = Data;

        /* Update the number of valid records. */
        StateDelta->recordCount += 1;
    }

    /* Regular (not masked) states. */
    else if (Mask == 0)
    {
        /* Get the current record. */
        recordEntry = &recordArray[mapEntryIndex[Address]];

        /* Update the state record. */
        recordEntry->mask = 0;
        recordEntry->data = Data;
    }

    /* Masked states. */
    else
    {
        /* Get the current record. */
        recordEntry = &recordArray[mapEntryIndex[Address]];

        /* Update the state record. */
        recordEntry->mask |=  Mask;
        recordEntry->data &= ~Mask;
        recordEntry->data |= (Data & Mask);
    }
}

/******************************************************************************\
**
**  gckCONTEXT_UpdateDelta
**
**  Update delta in kernel driver.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**      gcsSTATE_DELTA_PTR Delta
**          Pointer to the state delta.
*/
gceSTATUS
gckCONTEXT_UpdateDelta(
    IN gckCONTEXT Context,
    IN gcsSTATE_DELTA_PTR Delta
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsSTATE_DELTA_PTR delta = gcvNULL;
    gcsSTATE_DELTA_PTR prevDelta = gcvNULL;
    gcsCONTEXT_PTR buffer = gcvNULL;
    gcsSTATE_DELTA_RECORD_PTR record = gcvNULL;

    gcmkHEADER_ARG("Context=%p Delta=%p", Context, Delta);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);
    gcmkVERIFY_ARGUMENT(Delta != gcvNULL);

    delta = Delta;

    if (delta && Context->delta)
    {
        _CopyDelta(Context->delta, delta);

        buffer = Context->buffer;

        do
        {
            if (buffer->kDelta == gcvNULL)
            {
                buffer->kDelta = Context->delta;
            }

            buffer->kDeltaCount = 1;

            buffer = buffer->next;

            if (buffer == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_NOT_FOUND);
            }
        }
        while (Context->buffer != buffer);

        if (Context->deltaHead != Context->delta)
        {
            gctUINT count = 0;
            gctUINT i = 0;

            delta = Context->delta;

            count = delta->recordCount;

            record = gcmUINT64_TO_PTR(delta->recordArray);

            prevDelta = gcmUINT64_TO_PTR(delta->prev);

            /* Go through all records. */
            for (i = 0; i < count; i += 1)
            {
                _MergeDelta(
                    prevDelta, record->address, record->mask, record->data
                    );

                /* Advance to the next state. */
                record += 1;
            }

            /* Update the element count. */
            if (delta->elementCount != 0)
            {
                prevDelta->elementCount = delta->elementCount;
            }
        }
        else
        {
            Context->delta = (gcsSTATE_DELTA_PTR)gcmUINT64_TO_PTR(Context->delta->next);
        }

        _ResetDelta(Context->delta);
    }

OnError:
    gcmkFOOTER();
    return status;
}

/******************************************************************************\
**
**  gckCONTEXT_PreemptUpdate
**
**  Context update in preemption mode.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**      gckPREEMPT_COMMIT PreemptCommit
**          The preemptCommit.
*/
gceSTATUS
gckCONTEXT_PreemptUpdate(
    IN gckCONTEXT Context,
    IN gckPREEMPT_COMMIT PreemptCommit
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsCONTEXT_PTR buffer;
    gcsSTATE_MAP_PTR map;
    gcsSTATE_DELTA_RECORD_PTR record;
    gcsSTATE_DELTA_RECORD_PTR recordArray = gcvNULL;
    gctUINT elementCount;
    gctUINT address;
    gctUINT32 mask;
    gctUINT32 data;
    gctUINT index;
    gctUINT i, j;
    gctUINT32 dirtyRecordArraySize = 0;
    gcsSTATE_DELTA_PTR kDelta = gcvNULL;

    gcmkHEADER_ARG("Context=%p PreemptCommit=%p", Context, PreemptCommit);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    buffer = Context->buffer;

    gcmkONERROR(gckOS_WaitSignal(
        Context->os, buffer->signal, gcvFALSE, gcvINFINITE
        ));

    /* Are there any pending deltas? */
    if (buffer->kDeltaCount != 0)
    {
        /* Get the state map. */
        map = Context->map;

        kDelta = buffer->kDelta;

        /* Reset the vertex stream count. */
        elementCount = 0;

        /* Merge all pending deltas. */
        for (i = 0; i < buffer->kDeltaCount; i += 1)
        {
            dirtyRecordArraySize
                = gcmSIZEOF(gcsSTATE_DELTA_RECORD) * kDelta->recordCount;

            if (dirtyRecordArraySize)
            {
                /* Merge all pending states. */
                for (j = 0; j < kDelta->recordCount; j += 1)
                {
                    if (j >= Context->numStates)
                    {
                        break;
                    }

                    recordArray = gcmUINT64_TO_PTR(kDelta->recordArray);

                    /* Get the current state record. */
                    record = &recordArray[j];

                    /* Get the state address. */
                    address = record->address;

                    /* Make sure the state is a part of the mapping table. */
                    if (address >= Context->maxState)
                    {
                        gcmkTRACE(
                            gcvLEVEL_ERROR,
                            "%s(%d): State 0x%04X (0x%04X) is not mapped.\n",
                            __FUNCTION__, __LINE__,
                            address, address << 2
                            );

                        continue;
                    }

                    /* Get the state index. */
                    index = map[address].index;

                    /* Skip the state if not mapped. */
                    if (index == 0)
                    {
                        continue;
                    }

                    /* Get the data mask. */
                    mask = record->mask;

                    /* Get the new data value. */
                    data = record->data;

                    /* Masked states that are being completly reset or regular states. */
                    if ((mask == 0) || (mask == ~0U))
                    {
                        /* Process special states. */
                        if (address == 0x0595)
                        {
                            /* Force auto-disable to be disabled. */
                            data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));
                            data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
                            data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:13) - (0 ?
 13:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:13) - (0 ?
 13:13) + 1))))))) << (0 ?
 13:13))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 13:13) - (0 ?
 13:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:13) - (0 ? 13:13) + 1))))))) << (0 ? 13:13)));
                        }

                        /* Set new data. */
                        buffer->logical[index] = data;
                    }

                    /* Masked states that are being set partially. */
                    else
                    {
                        buffer->logical[index]
                            = (~mask & buffer->logical[index])
                            | (mask & data);
                    }
                }
            }

            /* Get the element count. */
            if (kDelta->elementCount != 0)
            {
                elementCount = kDelta->elementCount;
            }

            if (dirtyRecordArraySize)
            {
                recordArray = gcvNULL;
            }

            /* Get the next state delta. */
            kDelta = gcmUINT64_TO_PTR(kDelta->next);
        }

        /* Hardware disables all input attribute when the attribute 0 is programmed,
           it then reenables those attributes that were explicitely programmed by
           the software. Because of this we cannot program the entire array of
           values, otherwise we'll get all attributes reenabled, but rather program
           only those that are actully needed by the software.
           elementCount = attribCount + 1 to make sure 0 is a flag to indicate if UMD
           touches it.
        */
        if (elementCount != 0)
        {
            gctUINT base;
            gctUINT nopCount;
            gctUINT32_PTR nop;
            gctUINT fe2vsCount;
            gctUINT attribCount = elementCount -1;
            gctUINT32 feAttributeStatgeAddr = 0x0180;
            if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI5))
            {
                fe2vsCount = 32;
                base = map[0x5E00].index;
                feAttributeStatgeAddr = 0x5E00;
            }
            else if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI0))
            {
                fe2vsCount = 16;
                base = map[0x0180].index;
            }
            else
            {
                fe2vsCount = 12;
                base = map[0x0180].index;
            }

            /* Set the proper state count. */
            if (attribCount == 0)
            {
                gcmkASSERT(gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_ZERO_ATTRIB_SUPPORT));

                buffer->logical[base - 1]
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (feAttributeStatgeAddr) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                /* Set the proper state count. */
                buffer->logical[base + 1] =
                        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x01F2) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
                buffer->logical[base + 2] = 0x1;
                attribCount = 3;
            }
            else
            {
                buffer->logical[base - 1]
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (attribCount) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (feAttributeStatgeAddr) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
            }

            /* Determine the number of NOP commands. */
            nopCount = (fe2vsCount / 2) - (attribCount / 2);
            /* Determine the location of the first NOP. */
            nop = &buffer->logical[base + (attribCount | 1)];

            /* Fill the unused space with NOPs. */
            for (i = 0; i < nopCount; i += 1)
            {
                if (nop >= buffer->logical + Context->totalSize)
                {
                    break;
                }

                /* Generate a NOP command. */
                *nop = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                /* Advance. */
                nop += 2;
            }
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SMALL_BATCH) &&
            Context->hardware->options.smallBatch)
        {
            gctUINT numConstant = (gctUINT)Context->hardware->identity.numConstants;
            gctUINT32 constCount = 0;

            /* Get the const number after merge. */
            index = map[0x042B].index;
            data = buffer->logical[index];
            constCount = (((((gctUINT32) (data)) >> (0 ? 8:0)) & ((gctUINT32) ((((1 ? 8:0) - (0 ? 8:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 8:0) - (0 ? 8:0) + 1)))))) );

            _UpdateUnifiedReg(Context, 0xD000, numConstant << 2, constCount << 2);
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SMALL_BATCH) &&
            Context->hardware->options.smallBatch)
        {
            gctUINT numSamplers = 80;
            gctUINT32 samplerCount = 0;

            /* Get the sampler number after merge. */
            index = map[0x042C].index;
            data = buffer->logical[index];
            samplerCount = (((((gctUINT32) (data)) >> (0 ? 6:0)) & ((gctUINT32) ((((1 ? 6:0) - (0 ? 6:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 6:0) - (0 ? 6:0) + 1)))))) );

            _UpdateUnifiedReg(Context, 0x5800, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5880, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5900, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5980, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5A00, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5600, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, 0x5680, numSamplers, samplerCount);
        }

        /* Reset pending deltas. */
        buffer->kDeltaCount = 0;
    }

    /* Schedule an event to mark the context buffer as available. */
    gcmkONERROR(gckEVENT_Signal(
        buffer->eventObj, buffer->signal, gcvKERNEL_PIXEL
        ));

    /* Advance to the next context buffer. */
    Context->buffer = buffer->next;

OnError:
    gcmkFOOTER();
    return status;
}

/* Destroy previous context switch delta. */
gceSTATUS
gckCONTEXT_DestroyPrevDelta(
    IN gckCONTEXT Context
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Context=%p", Context);

    if (Context->prevRecordArray)
    {
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Context->os, Context->prevRecordArray));
    }

    if (Context->prevMapEntryID)
    {
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Context->os, Context->prevMapEntryID));
    }

    if (Context->prevMapEntryIndex)
    {
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Context->os, Context->prevMapEntryIndex));
    }

    Context->prevDeltaPtr = gcvNULL;

    gcmkFOOTER();

    return status;
}

/* Construct and store previous context switch delta. */
gceSTATUS
gckCONTEXT_ConstructPrevDelta(
    IN gckCONTEXT Context,
    IN gctUINT32 ProcessID,
    IN gcsSTATE_DELTA_PTR StateDelta
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsSTATE_DELTA_PTR uDelta = gcvNULL;
    gcsSTATE_DELTA_PTR kDelta = gcvNULL;
    gcsSTATE_DELTA_RECORD_PTR kRecordArray = gcvNULL;
    gctBOOL needCopy = gcvFALSE;
    gctPOINTER pointer = gcvNULL;
    gctUINT32 dirtyRecordArraySize = 0;
    gckKERNEL kernel = gcvNULL;
    gctBOOL allocated = gcvFALSE;

    gcmkHEADER_ARG(
        "Context=%p ProcessID=%d StateDelta=%p",
        Context, ProcessID, StateDelta
        );

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    if (StateDelta)
    {
        kernel = Context->hardware->kernel;

        gcmkVERIFY_OK(gckCONTEXT_DestroyPrevDelta(Context));

        gcmkVERIFY_OK(gckOS_QueryNeedCopy(kernel->os, ProcessID, &needCopy));

        uDelta = StateDelta;

        gcmkONERROR(gckKERNEL_OpenUserData(
            kernel, needCopy,
            &Context->prevDelta,
            uDelta, gcmSIZEOF(gcsSTATE_DELTA),
            (gctPOINTER *)&kDelta
            ));

        allocated = gcvTRUE;

        dirtyRecordArraySize
            = gcmSIZEOF(gcsSTATE_DELTA_RECORD) * kDelta->recordCount;

        if (dirtyRecordArraySize)
        {
            gcmkONERROR(gckOS_Allocate(
                kernel->os,
                gcmSIZEOF(gcsSTATE_DELTA_RECORD) * dirtyRecordArraySize,
                &pointer
                ));

            Context->prevRecordArray = (gcsSTATE_DELTA_RECORD_PTR)pointer;

            gcmkONERROR(gckKERNEL_OpenUserData(
                kernel, needCopy,
                Context->prevRecordArray,
                gcmUINT64_TO_PTR(kDelta->recordArray),
                dirtyRecordArraySize,
                (gctPOINTER *) &kRecordArray
                ));

            if (kRecordArray == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            gcmkONERROR(gckKERNEL_CloseUserData(
                kernel, needCopy,
                gcvFALSE,
                gcmUINT64_TO_PTR(kDelta->recordArray),
                dirtyRecordArraySize,
                (gctPOINTER *) &kRecordArray
                ));

        }
        else
        {
            Context->prevRecordArray = gcvNULL;
        }

        kDelta->recordArray = gcmPTR_TO_UINT64(Context->prevRecordArray);

        if (Context && Context->maxState > 0)
        {
            /* Compute UINT array size. */
            gctSIZE_T bytes = gcmSIZEOF(gctUINT) * Context->maxState;
            gctUINT32 *kMapEntryID = gcvNULL;
            gctUINT32 *kMapEntryIndex = gcvNULL;

            /* Allocate map ID array. */
            gcmkONERROR(gckOS_Allocate(
                kernel->os, bytes, &pointer
                ));

            Context->prevMapEntryID = (gctUINT32 *)pointer;

            /* Set the map ID size. */
            kDelta->mapEntryIDSize = (gctUINT32)bytes;

            gcmkONERROR(gckKERNEL_OpenUserData(
                kernel, needCopy,
                Context->prevMapEntryID,
                gcmUINT64_TO_PTR(kDelta->mapEntryID),
                bytes,
                (gctPOINTER *) &kMapEntryID
                ));

            if (kMapEntryID == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            gcmkONERROR(gckKERNEL_CloseUserData(
                kernel, needCopy,
                gcvFALSE,
                gcmUINT64_TO_PTR(kDelta->mapEntryID),
                bytes,
                (gctPOINTER *) &kMapEntryID
                ));

            kDelta->mapEntryID = gcmPTR_TO_UINT64(Context->prevMapEntryID);

            /* Allocate map index array. */
            gcmkONERROR(gckOS_Allocate(
                kernel->os, bytes, &pointer
                ));

            Context->prevMapEntryIndex = (gctUINT32 *)pointer;

            gcmkONERROR(gckKERNEL_OpenUserData(
                kernel, needCopy,
                Context->prevMapEntryIndex,
                gcmUINT64_TO_PTR(kDelta->mapEntryIndex),
                bytes,
                (gctPOINTER *) &kMapEntryIndex
                ));

            if (kMapEntryIndex == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            gcmkONERROR(gckKERNEL_CloseUserData(
                kernel, needCopy,
                gcvFALSE,
                gcmUINT64_TO_PTR(kDelta->mapEntryIndex),
                bytes,
                (gctPOINTER *) &kMapEntryIndex
                ));

            kDelta->mapEntryIndex = gcmPTR_TO_UINT64(Context->prevMapEntryIndex);
        }

        Context->prevDeltaPtr = kDelta;

        gcmkONERROR(gckKERNEL_CloseUserData(
            kernel, needCopy,
            gcvFALSE,
            uDelta, gcmSIZEOF(gcsSTATE_DELTA),
            (gctPOINTER *) &kDelta
            ));
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (allocated)
    {
        gcmkVERIFY_OK(gckCONTEXT_DestroyPrevDelta(Context));
    }

    gcmkFOOTER();
    return status;
}


#endif

