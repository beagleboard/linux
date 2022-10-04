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


#include <gc_hal.h>
#include <gc_feature_database.h>
#include "gc_hal_kernel_hardware_func_flop_reset.h"
#include "gc_hal_kernel_hardware_func_flop_reset_config.h"

/*
 * Flop reset.
 *
 * The flops can be reset with PPU, NN and TP programs.
 * PPU:
 *   Requirements:
 *   1. DP inst with all bins enabled.
 *   2. Load inst which has at least two shader group,
 *      and every thread should load from different 64-byte address.
 *   3. Stroe inst which has at least 6 threads, whose addresses are
 *      from different 64-byte address and flush.
 *   Case:
 *   * InImage: 64x6 = {1}, unsigned int8
 *   * OutImage: 64x6, unsigned int8
 *   * OutImage = InImage + InImage
 * NN:
 *   Requirements:
 *   1. A XYDP6 case.
 *   2. NN cmd that uses only 1 core and make othere core's kernel size
 *      to be 0.
 *   Case:
 *   * Input: 3x2x1 = {1}
 *   * Kernel: 2x2x1 = {1}
 *   * Output: 2x1x1
 * TP:
 *   Requirements:
 *   1. Run TP fc on all TP cores.
 *   Case:
 *   * Input: 1x1x2 = {1}
 *   * Kernel: 1x1x2x64 = {1}
 *   * Output: 1x64
 */

/*
 * PPU.
 */



static gceSTATUS
_AllocateVideoMemory(
    IN gckKERNEL Kernel,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    IN OUT gctSIZE_T *Bytes,
    OUT gckVIDMEM_NODE *Node,
    OUT gctPOINTER *Logical,
    OUT gctUINT32 *Address
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gcePOOL pool = gcvPOOL_DEFAULT;

    if (!Bytes || *Bytes == 0)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (Pool)
    {
        pool = *Pool;
    }

    bufferBytes = *Bytes;

    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        Kernel,
        64,
        Type,
        AllocFlag,
        &bufferBytes,
        &pool,
        &bufferNode
        ));

    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        Kernel,
        bufferNode,
        gcvFALSE,
        gcvFALSE,
        &bufferLogical
        ));

    gcmkONERROR(gckVIDMEM_NODE_Lock(
        Kernel,
        bufferNode,
        &bufferAddress
        ));

    gcmkONERROR(gckOS_ZeroMemory(bufferLogical, bufferBytes));

    *Bytes = bufferBytes;

    if (Pool)
    {
        *Pool = pool;
    }

    if (Node)
    {
        *Node = bufferNode;
    }

    if (Logical)
    {
        *Logical = bufferLogical;
    }

    if (Address)
    {
        *Address = bufferAddress;
    }

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        if (bufferAddress)
        {
            gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
                Kernel,
                bufferNode,
                0,
                gcvNULL
                ));
        }

        if (bufferLogical)
        {
            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                Kernel,
                bufferNode,
                0,
                gcvFALSE,
                gcvFALSE
                ));
        }

        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            Kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_FreeVideoMemory(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Node)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
        Kernel,
        Node,
        0,
        gcvNULL
        ));

    gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
        Kernel,
        Node,
        0,
        gcvFALSE,
        gcvFALSE
        ));

    gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
        Kernel,
        Node
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_BitValue(
    IN gctUINT8_PTR *Base,
    IN gctUINT32 Value,
    IN gctUINT32_PTR Offset,
    IN gctUINT Length
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32_PTR msb = (gctUINT32_PTR)(*Base) + 1, lsb = (gctUINT32_PTR)(*Base);

    gcmkASSERT(*Offset <= 32 && Length <= 32);

    if ((*Offset) < 32)
    {
        gctUINT32 end = (*Offset) + Length, data = *lsb;

        if (end < 32)
        {
            /************************************************************************
             *       offset    32           64                                      *
             *     _________________________                                        *
             *    |_____|////|_|____________|                                       *
             *              end                                                     *
             ************************************************************************/
            data  = (*lsb & ((1 << *Offset) - 1));
            data |= (*lsb & ~((1 << end) - 1));
            data |= (Value << *Offset);

            *lsb = data;
            *Offset = end;
        }
        else if (end < 64)
        {
            /************************************************************************
             *       offset    32           64                                      *
             *     _________________________                                        *
             *    |_____|//////|//|_________|                                       *
             *                   end                                                *
             ************************************************************************/
            gctUINT32 length_m = end - 32;
            gctUINT32 data_l = (*lsb & ((1 << *Offset) - 1));
            gctUINT32 data_m = (*msb & ~((1 << length_m) - 1));

            data_l |= (Value << *Offset);
            data_m |= (Value >> (32 - *Offset));

            *lsb = data_l;

            if (end > 32)
                *msb = data_m;

            *Offset = length_m;

            *Base = (gctUINT8_PTR)msb;
        }

    }

    return status;
}

static gceSTATUS
_GetVIPCoreInfo(
    IN gckHARDWARE Hardware,
    OUT gceVIP_ARCH_TYPE *ArchType,
    OUT gctUINT8 *DataType,
    OUT gctUINT32 *CoreCount,
    OUT gctUINT32 *Zdp,
    OUT gctUINT32 *KernelBurstSize
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcsFEATURE_DATABASE *database = (gcsFEATURE_DATABASE *)(Hardware->featureDatabase);
    gceVIP_ARCH_TYPE archType;
    gctUINT8 dataType = 0x0;
    gctUINT32 coreCount = 0;
    gctUINT32 zdp = 1;
    gctUINT32 kernelBurstSize;

    gcmkASSERT(database);

    /* Make compiler happy. */
    gcQueryFeatureDB(0, 0, 0, 0, 0);

    /* Choose one supported format. */
    if (database->NNCoreCount_INT8 > 0)
    {
        dataType = 0x0;
        coreCount = database->NNCoreCount_INT8;
    }
    else if (database->NNCoreCount_INT16 > 0)
    {
        dataType = 0x4;
        coreCount = database->NNCoreCount_INT16;
    }
    else if (database->NNCoreCount_FLOAT16 > 0)
    {
        dataType = 0x1;
        coreCount = database->NNCoreCount_FLOAT16;
    }
    else if (database->NNCoreCount_BFLOAT > 0)
    {
        dataType = 0x7;
        coreCount = database->NNCoreCount_BFLOAT;
    }
    else
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    if (database->NN_XYDP0)
    {
        archType = gcvVIP_ARCH_TYPE_V8;
    }
    else if (database->VIP_V7)
    {
        archType = gcvVIP_ARCH_TYPE_V7;
    }
    else
    {
        archType = gcvVIP_ARCH_TYPE_V6;
    }

    zdp = database->NN_ZDP3 ? 3 : 1;

    kernelBurstSize = database->DDR_KERNEL_BURST_SIZE;

    if (ArchType)
    {
        *ArchType = archType;
    }

    if(Hardware->identity.customerID == 0x23 || Hardware->identity.customerID == 0x83)
    {
        dataType = 0x1;
    }
    else if(Hardware->identity.customerID == 0x96)
    {
        dataType = 0x2;
    }

    if (DataType)
    {
        *DataType = dataType;
    }

    if (CoreCount)
    {
        *CoreCount = coreCount;
    }

    if (Zdp)
    {
        *Zdp = zdp;
    }

    if (KernelBurstSize)
    {
        *KernelBurstSize = kernelBurstSize;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_GetNNDataSize(
   IN gctUINT8 DataType,
   OUT gctUINT32_PTR DataSize
   )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!DataSize)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (DataType)
    {
    case 0x2:
    case 0x0:
        *DataSize = 1;
        break;

    case 0x4:
    case 0x1:
    case 0x7:
        *DataSize = 2;
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*
 * PPU.
 */
static gceSTATUS
_ProgramPPUInput(
    IN gckHARDWARE Hardware,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 1; /* U8 format. */
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT32 *buffer = gcvNULL;
    gctUINT32 i;

    bufferBytes = bytes = (gctSIZE_T)(InImageXSize * InImageYSize * itemBytes);

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    for (i = 0; i < bytes / 4; i++)
    {
        buffer[i] = PPU_IMAGE_DATA;
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: ppu input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramPPUOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Width,
    IN gctUINT32 Height,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 1;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    bufferBytes = bytes = (gctSIZE_T)(Width * Height * itemBytes);

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gctUINT32
_SETBITS(
    IN gctUINT32 Data,
    IN gctUINT32 Start,
    IN gctUINT32 End,
    IN gctUINT32 Value
    )
{
    gctUINT32 data = Data;
    gctUINT32 mask;

    if (End >= Start)
    {
        mask =  ((~0ULL >> (63 - End + Start)) << Start);
        data &= ~mask;
        data |= ((Value) << Start) & mask;
        return data;
    }
    else
    {
        mask =  ((~0ULL >> (63 - Start + End)) << End);
        data &= ~mask;
        data |= ((Value) << End) & mask;
        return data;
    }
}

static gctUINT32
_SETBIT(
    IN gctUINT32 Data,
    IN gctUINT32 Position,
    IN gctUINT32 Value
    )
{
    gctUINT32 data;

    data = _SETBITS(Data, Position, Position, Value);

    return data;
}

static gctUINT32
_GETBITS(
    IN gctUINT32 Data,
    IN gctUINT32 Start,
    IN gctUINT32 End
    )
{
    gctUINT32 data = Data;
    gctUINT32 mask;

    if (End >= Start)
    {
        mask = (~0ULL >> (63 - (End - Start)));
        return (data >> Start) & mask;
    }
    else
    {
        mask = (~0ULL >> (63 - (Start - End)));
        return (data >> End) & mask;;
    }
}

static gctUINT32
_GETBIT(
    IN gctUINT32 Data,
    IN gctUINT32 Position
    )
{
    gctUINT32 data;

    data = _GETBITS(Data, Position, Position);

    return data;
}

static gceSTATUS
gckPPU_SetImmediate(
    IN gctUINT32 Where,
    IN gctUINT32 Value,
    IN gctUINT32 Type,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (Where)
    {
    case 0:
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:12) - (0 ?
 20:12) + 1))))))) << (0 ?
 20:12))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 8, 0)) & ((gctUINT32) ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 20:12) - (0 ? 20:12) + 1))))))) << (0 ? 20:12)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:22) - (0 ?
 29:22) + 1))))))) << (0 ?
 29:22))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 16, 9)) & ((gctUINT32) ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:22) - (0 ? 29:22) + 1))))))) << (0 ? 29:22)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 17)) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 18)) & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 19) | (Type << 1)) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:3) - (0 ?
 5:3) + 1))))))) << (0 ?
 5:3))) | (((gctUINT32) ((gctUINT32) (0x7) & ((gctUINT32) ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:3) - (0 ? 5:3) + 1))))))) << (0 ? 5:3)));
        break;

    case 1:
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:7) - (0 ?
 15:7) + 1))))))) << (0 ?
 15:7))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 8, 0)) & ((gctUINT32) ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:7) - (0 ? 15:7) + 1))))))) << (0 ? 15:7)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 24:17) - (0 ?
 24:17) + 1))))))) << (0 ?
 24:17))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 16, 9)) & ((gctUINT32) ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 24:17) - (0 ? 24:17) + 1))))))) << (0 ? 24:17)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:25) - (0 ?
 25:25) + 1))))))) << (0 ?
 25:25))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 17)) & ((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:25) - (0 ? 25:25) + 1))))))) << (0 ? 25:25)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 18)) & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:27) - (0 ?
 29:27) + 1))))))) << (0 ?
 29:27))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 19) | (Type << 1)) & ((gctUINT32) ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:27) - (0 ? 29:27) + 1))))))) << (0 ? 29:27)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (0x7) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)));
        break;

    case 2:
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:4) - (0 ?
 12:4) + 1))))))) << (0 ?
 12:4))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 8, 0)) & ((gctUINT32) ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:4) - (0 ? 12:4) + 1))))))) << (0 ? 12:4)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:14) - (0 ?
 21:14) + 1))))))) << (0 ?
 21:14))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Value, 16, 9)) & ((gctUINT32) ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 21:14) - (0 ? 21:14) + 1))))))) << (0 ? 21:14)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 17)) & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))))) << (0 ?
 23:23))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 18)) & ((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:23) - (0 ? 23:23) + 1))))))) << (0 ? 23:23)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 27:25) - (0 ?
 27:25) + 1))))))) << (0 ?
 27:25))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Value, 19) | (Type << 1)) & ((gctUINT32) ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 27:25) - (0 ? 27:25) + 1))))))) << (0 ? 27:25)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))))) << (0 ?
 30:28))) | (((gctUINT32) ((gctUINT32) (0x7) & ((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:28) - (0 ? 30:28) + 1))))))) << (0 ? 30:28)));
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetInstructionType(
    IN gctUINT32 Type,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:21) - (0 ?
 21:21) + 1))))))) << (0 ?
 21:21))) | (((gctUINT32) ((gctUINT32) (_GETBIT(Type, 0)) & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 21:21) - (0 ? 21:21) + 1))))))) << (0 ? 21:21)));
    Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:30) - (0 ?
 31:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:30) - (0 ?
 31:30) + 1))))))) << (0 ?
 31:30))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Type, 2, 1)) & ((gctUINT32) ((((1 ?
 31:30) - (0 ?
 31:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:30) - (0 ? 31:30) + 1))))))) << (0 ? 31:30)));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_IsEndOfBB(
    IN gckHARDWARE Hardware,
    IN gctUINT32 OpCode,
    OUT gctUINT32_PTR Inst
)
{
    gceSTATUS status = gcvSTATUS_OK;

    gcsFEATURE_DATABASE *database = (gcsFEATURE_DATABASE *)Hardware->featureDatabase;
    gctUINT32 bits = 0;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (!database->SH_END_OF_BB)
    {
        return gcvSTATUS_OK;
    }

    switch (OpCode)
    {
    case 0x09:
    case 0x56:
    case 0x0A:
    case 0x0B:
    case 0x0F:
    case 0x31:
    case 0x10:
        bits = _SETBITS(Inst[1], 3, 3, 1);
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:3) - (0 ?
 10:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:3) - (0 ?
 10:3) + 1))))))) << (0 ?
 10:3))) | (((gctUINT32) ((gctUINT32) (bits) & ((gctUINT32) ((((1 ?
 10:3) - (0 ?
 10:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:3) - (0 ? 10:3) + 1))))))) << (0 ? 10:3)));
        break;

    case 0x65:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
    case 0x6C:
    case 0x46:
        bits = _SETBITS(Inst[0], 2, 2, 1);
        Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))))) << (0 ?
 10:6))) | (((gctUINT32) ((gctUINT32) (bits) & ((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:6) - (0 ? 10:6) + 1))))))) << (0 ? 10:6)));
        break;

    case 0x32:
    case 0x39:
    case 0x33:
    case 0x3A:
    case 0x79:
    case 0x34:
    case 0x7A:
    case 0x35:
        bits = _SETBITS(Inst[0], 2, 2, 1);
        Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))))) << (0 ?
 10:6))) | (((gctUINT32) ((gctUINT32) (bits) & ((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:6) - (0 ? 10:6) + 1))))))) << (0 ? 10:6)));
        break;

    default:
        if (OpCode != 0x16 &&
            OpCode != 0x24 &&
            OpCode != 0x14 &&
            OpCode != 0x15 &&
            OpCode != 0x17)
        {
            bits = _SETBITS(Inst[0], 2, 2, 1);
            Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))))) << (0 ?
 10:6))) | (((gctUINT32) ((gctUINT32) (bits) & ((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:6) - (0 ? 10:6) + 1))))))) << (0 ? 10:6)));
        }
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_AddOpCode(
    IN gckHARDWARE Hardware,
    IN gctUINT32 OpCode,
    IN gctUINT32 Extended,
    IN gctUINT32 Type,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (_GETBITS(OpCode, 5, 0)) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0)));
    Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (_GETBIT(OpCode, 6)) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));

    switch (OpCode)
    {
    case 0x7F:
        gcmkONERROR(gckPPU_SetImmediate(2, Extended, 0x2, Inst));
        break;

    case 0x45:
        Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:13) - (0 ?
 15:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:13) - (0 ?
 15:13) + 1))))))) << (0 ?
 15:13))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Extended, 2, 0)) & ((gctUINT32) ((((1 ?
 15:13) - (0 ?
 15:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:13) - (0 ? 15:13) + 1))))))) << (0 ? 15:13)));
        Inst[0] = _SETBIT(Inst[0], 31, _GETBIT(Extended, 3));
        Inst[1] = _SETBITS(Inst[1], 1, 0, _GETBITS(Extended, 5, 4));
        break;

    case 0x31:
    case 0x09:
    case 0x0F:
        Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))))) << (0 ?
 10:6))) | (((gctUINT32) ((gctUINT32) (_GETBITS(Extended, 4, 0)) & ((gctUINT32) ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:6) - (0 ? 10:6) + 1))))))) << (0 ? 10:6)));
        break;

    default:
        break;
    }

    if (Type != GCREG_SH_INSTRUCTION_TYPE_INVALID)
    {
        gcmkONERROR(gckPPU_SetInstructionType(Type, Inst));
    }

    gcmkONERROR(gckPPU_IsEndOfBB(Hardware, OpCode, Inst));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetDestination(
    IN gctUINT32 Address,
    IN gctUINT32 WriteEnable,
    IN gctUINT32 Saturate,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)));
    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:16) - (0 ?
 22:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:16) - (0 ?
 22:16) + 1))))))) << (0 ?
 22:16))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 22:16) - (0 ?
 22:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:16) - (0 ? 22:16) + 1))))))) << (0 ? 22:16)));
    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:23) - (0 ?
 26:23) + 1))))))) << (0 ?
 26:23))) | (((gctUINT32) ((gctUINT32) (WriteEnable) & ((gctUINT32) ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:23) - (0 ? 26:23) + 1))))))) << (0 ? 26:23)));
    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) ((gctUINT32) (Saturate) & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

    return gcvSTATUS_OK;

OnError:
    return status;
}

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
#define gcdVX_ENABLE4(X, Y, Z, W) ((1 << (X)) | (1 << (Y)) | (1 << (Z)) | (1 << (W)))
#define gcdVX_ENABLE1(X) (1 << (X))
#define gcdVX_ENABLE2(X, Y) ((1 << (X)) | (1 << (Y)))
#define gcdVX_ENABLE3(X, Y, Z) ((1 << (X)) | (1 << (Y)) | (1 << (Z)))
#define gcdVX_SWIZZLE (0 | (1 << 2) | (2 << 4) | (3 << 6))
#define gcdVX_SWIZZLE1(X) ((X) | ((X) << 2) | ((X) << 4) | ((X) << 6))
#define gcdVX_SWIZZLE2(X, Y) ((X) | ((Y) << 2) | ((Y) << 4) | ((Y) << 6))
#define gcdVX_SWIZZLE4(X, Y, Z, W) ((X) | ((Y) << 2) | ((Z) << 4) | ((W) << 6))

static gctUINT32
gckPPU_GetPixel(
    IN gctUINT32 Format
    )
{
    gctUINT32 pixel = 0;

    switch(Format)
    {
    case 0x7:
        pixel = 15;
        break;

    case 0x3:
    case 0x6:
        pixel = 7;
        break;

    default:
        pixel = 15;
        break;
    }

    return pixel;
}

gceSTATUS
gckPPU_SetEVIS(
    IN gctUINT32 Start,
    IN gctUINT32 End,
    IN gctUINT32 Evis,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = ((((gctUINT32) (Inst[0])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:23) - (0 ?
 26:23) + 1))))))) << (0 ?
 26:23))) | (((gctUINT32) ((gctUINT32) (Start) & ((gctUINT32) ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:23) - (0 ? 26:23) + 1))))))) << (0 ? 26:23)));
    Inst[0] = _SETBITS(Inst[0], 30, 27, End);
    Inst[1] = _SETBITS(Inst[1], 10, 2, Evis);

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetSource(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Type,
    IN gctBOOL Negate,
    IN gctBOOL Absolute,
    IN gctUINT32 Relative,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (Where)
    {
    case 0:
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:12) - (0 ?
 20:12) + 1))))))) << (0 ?
 20:12))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 20:12) - (0 ? 20:12) + 1))))))) << (0 ? 20:12)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:22) - (0 ?
 29:22) + 1))))))) << (0 ?
 29:22))) | (((gctUINT32) ((gctUINT32) (Swizzle) & ((gctUINT32) ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:22) - (0 ? 29:22) + 1))))))) << (0 ? 29:22)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) (Negate) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30)));
        Inst[1] = ((((gctUINT32) (Inst[1])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) ((gctUINT32) (Absolute) & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (Relative) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:3) - (0 ?
 5:3) + 1))))))) << (0 ?
 5:3))) | (((gctUINT32) ((gctUINT32) (Type) & ((gctUINT32) ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:3) - (0 ? 5:3) + 1))))))) << (0 ? 5:3)));
        break;

    case 1:
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:7) - (0 ?
 15:7) + 1))))))) << (0 ?
 15:7))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:7) - (0 ? 15:7) + 1))))))) << (0 ? 15:7)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 24:17) - (0 ?
 24:17) + 1))))))) << (0 ?
 24:17))) | (((gctUINT32) ((gctUINT32) (Swizzle) & ((gctUINT32) ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 24:17) - (0 ? 24:17) + 1))))))) << (0 ? 24:17)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:25) - (0 ?
 25:25) + 1))))))) << (0 ?
 25:25))) | (((gctUINT32) ((gctUINT32) (Negate) & ((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:25) - (0 ? 25:25) + 1))))))) << (0 ? 25:25)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) ((gctUINT32) (Absolute) & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)));
        Inst[2] = ((((gctUINT32) (Inst[2])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:27) - (0 ?
 29:27) + 1))))))) << (0 ?
 29:27))) | (((gctUINT32) ((gctUINT32) (Relative) & ((gctUINT32) ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:27) - (0 ? 29:27) + 1))))))) << (0 ? 29:27)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (Type) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)));
        break;

    case 2:
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:4) - (0 ?
 12:4) + 1))))))) << (0 ?
 12:4))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:4) - (0 ? 12:4) + 1))))))) << (0 ? 12:4)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:14) - (0 ?
 21:14) + 1))))))) << (0 ?
 21:14))) | (((gctUINT32) ((gctUINT32) (Swizzle) & ((gctUINT32) ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 21:14) - (0 ? 21:14) + 1))))))) << (0 ? 21:14)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) ((gctUINT32) (Negate) & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))))) << (0 ?
 23:23))) | (((gctUINT32) ((gctUINT32) (Absolute) & ((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:23) - (0 ? 23:23) + 1))))))) << (0 ? 23:23)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 27:25) - (0 ?
 27:25) + 1))))))) << (0 ?
 27:25))) | (((gctUINT32) ((gctUINT32) (Relative) & ((gctUINT32) ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 27:25) - (0 ? 27:25) + 1))))))) << (0 ? 27:25)));
        Inst[3] = ((((gctUINT32) (Inst[3])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))))) << (0 ?
 30:28))) | (((gctUINT32) ((gctUINT32) (Type) & ((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:28) - (0 ? 30:28) + 1))))))) << (0 ? 30:28)));
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static const gctUINT32 NEGATE_FLAG   = 1 << 0;
static const gctUINT32 ABSOLUTE_FLAG = 1 << 1;

static gceSTATUS
gckPPU_SetUniform(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Modifiers,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate = (Modifiers & NEGATE_FLAG) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(gckPPU_SetSource(
        Where,
        Address,
        Swizzle,
        0x2,
        negate,
        absolute,
        0,
        Inst
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckPPU_SetTempReg(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Modifiers,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate = (Modifiers & NEGATE_FLAG) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(gckPPU_SetSource(
        Where,
        Address,
        Swizzle,
        0x0,
        negate,
        absolute,
        0,
        Inst
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}


static gceSTATUS
_ProgramPPUInstruction(
    IN gckHARDWARE Hardware,
    IN gctUINT32 DataType,
    IN gctUINT32 numShaderCores,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gctUINT32 *InstCount,
    OUT gctUINT32 *RegCount,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    gctUINT32 instCount = 0;
    gctUINT32_PTR inst = gcvNULL;

    gctUINT32 inImage1DataType = DataType;
    gctUINT32 inImage2DataType = DataType;
    gctUINT32 outImageDataType = DataType;

    if (!Data || !InstCount || !RegCount)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    bufferBytes = bytes = gcmSIZEOF(gctUINT32) * MAX_PPU_INSTRUCTION_COUNT;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    inst = (gctUINT32_PTR)bufferLogical;

    /* img_load.u8 r1, c0, r0.xy */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, 0x79, 0, inImage1DataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, gckPPU_GetPixel(inImage1DataType), 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    instCount += 4;

    /*img_load.u8 r2, c0, r0.xy */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, 0x79, 0, inImage2DataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(2, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, gckPPU_GetPixel(inImage2DataType), 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    instCount += 4;

    /* dp2x8 r1, r1, r2, c3_512 */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, 0x45, 0x0B, outImageDataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, 7, (inImage1DataType | (inImage2DataType << 3)), &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(0, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 2, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetSource (2, 2, gcdVX_SWIZZLE, 0x4, gcvFALSE, gcvFALSE, 0, &inst[instCount]));
    instCount += 4;

    /* img_store.u8 r1, c2, r0.xy, r1 */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, 0x7A, 0, outImageDataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, (gckPPU_GetPixel(outImageDataType) + 1) / numShaderCores - 1, 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(2, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    instCount += 4;

    bytes = gcmSIZEOF(gctUINT32) * instCount;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: ppu instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    *InstCount = instCount;
    *RegCount = 0x3;

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramPPUCommand(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Stride,
    IN gctUINT32 Width,
    IN gctUINT32 Height,
    IN gctUINT32 WorkDim,
    IN gctUINT32 ValueOrder,
    IN gctUINT32 GroupSizeX,
    IN gctUINT32 GroupSizeY,
    IN gctUINT32 GroupSizeZ,
    IN gctUINT32 GlobalScaleX,
    IN gctUINT32 GlobalScaleY,
    IN gctUINT32 GlobalScaleZ,
    IN gctUINT32 GlobalOffsetX,
    IN gctUINT32 GlobalOffsetY,
    IN gctUINT32 GlobalOffsetZ,
    IN gctUINT32 ThreadAllocation,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 InstAddress,
    IN gctUINT32 InstCount,
    IN gctUINT32 RegCount,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gctUINT32 bytes = 0;
    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gctUINT32_PTR commands = gcvNULL;
    gctUINT32 index = 0;
    gctUINT32 groupCountX = (Width +  GlobalScaleX - 1) / GlobalScaleX;
    gctUINT32 groupCountY = (Height +  GlobalScaleY - 1) / GlobalScaleY;
    gctUINT32 groupCountZ = 0;

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    bufferBytes = gcmSIZEOF(gctUINT32) * MAX_PPU_COMMAND_NUM;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32_PTR)bufferLogical;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E13) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) (0x2 & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0xD800) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));
    commands[index++] = InImageAddress;
    commands[index++] = Stride;
    commands[index++] = Height << 16 | Width;
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:4) - (0 ?
 5:4) + 1))))))) << (0 ?
 5:4))) | (((gctUINT32) ((gctUINT32) (0x3) & ((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:4) - (0 ? 5:4) + 1))))))) << (0 ? 5:4)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:6) - (0 ?
 9:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:6) - (0 ?
 9:6) + 1))))))) << (0 ?
 9:6))) | (((gctUINT32) ((gctUINT32) (0x7) & ((gctUINT32) ((((1 ?
 9:6) - (0 ?
 9:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:6) - (0 ? 9:6) + 1))))))) << (0 ? 9:6)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:10) - (0 ?
 11:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:10) - (0 ?
 11:10) + 1))))))) << (0 ?
 11:10))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 11:10) - (0 ?
 11:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:10) - (0 ? 11:10) + 1))))))) << (0 ? 11:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:14) - (0 ?
 15:14) + 1))))))) << (0 ?
 15:14))) | (((gctUINT32) ((gctUINT32) (0x1) & ((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:14) - (0 ? 15:14) + 1))))))) << (0 ? 15:14)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:16) - (0 ?
 18:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:16) - (0 ?
 18:16) + 1))))))) << (0 ?
 18:16))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 18:16) - (0 ?
 18:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:16) - (0 ? 18:16) + 1))))))) << (0 ? 18:16)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:20) - (0 ?
 22:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:20) - (0 ?
 22:20) + 1))))))) << (0 ?
 22:20))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 22:20) - (0 ?
 22:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:20) - (0 ? 22:20) + 1))))))) << (0 ? 22:20)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:24) - (0 ?
 26:24) + 1))))))) << (0 ?
 26:24))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:24) - (0 ? 26:24) + 1))))))) << (0 ? 26:24)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))))) << (0 ?
 30:28))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:28) - (0 ? 30:28) + 1))))))) << (0 ? 30:28)));

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0xD800 + 0x04) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));
    commands[index++] = OutImageAddress;
    commands[index++] = Stride;
    commands[index++] = Height << 16 | Width;
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:4) - (0 ?
 5:4) + 1))))))) << (0 ?
 5:4))) | (((gctUINT32) ((gctUINT32) (0x3) & ((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:4) - (0 ? 5:4) + 1))))))) << (0 ? 5:4)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:6) - (0 ?
 9:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:6) - (0 ?
 9:6) + 1))))))) << (0 ?
 9:6))) | (((gctUINT32) ((gctUINT32) (0x7) & ((gctUINT32) ((((1 ?
 9:6) - (0 ?
 9:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:6) - (0 ? 9:6) + 1))))))) << (0 ? 9:6)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:10) - (0 ?
 11:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:10) - (0 ?
 11:10) + 1))))))) << (0 ?
 11:10))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 11:10) - (0 ?
 11:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:10) - (0 ? 11:10) + 1))))))) << (0 ? 11:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:14) - (0 ?
 15:14) + 1))))))) << (0 ?
 15:14))) | (((gctUINT32) ((gctUINT32) (0x1) & ((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:14) - (0 ? 15:14) + 1))))))) << (0 ? 15:14)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:16) - (0 ?
 18:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:16) - (0 ?
 18:16) + 1))))))) << (0 ?
 18:16))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 18:16) - (0 ?
 18:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:16) - (0 ? 18:16) + 1))))))) << (0 ? 18:16)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:20) - (0 ?
 22:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:20) - (0 ?
 22:20) + 1))))))) << (0 ?
 22:20))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 22:20) - (0 ?
 22:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:20) - (0 ? 22:20) + 1))))))) << (0 ? 22:20)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:24) - (0 ?
 26:24) + 1))))))) << (0 ?
 26:24))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:24) - (0 ? 26:24) + 1))))))) << (0 ? 26:24)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))))) << (0 ?
 30:28))) | (((gctUINT32) ((gctUINT32) (0x4) & ((gctUINT32) ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:28) - (0 ? 30:28) + 1))))))) << (0 ? 30:28)));

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0xD800 + 0x08) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));
    commands[index++] = 0x55555555;
    commands[index++] = 0x00000000; /* TCfg. */
    commands[index++] = 0x01234567;
    commands[index++] = 0x89abcdef;
    commands[index++] = 0x55555555;
    commands[index++] = 0x01234567;
    commands[index++] = 0x89abcdef; /* BinSelect. */
    commands[index++] = 0x00000000; /* AccumType, ConstantType, and PostShift. */
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000; /* Constant. */

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0240) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) ((gctUINT32) (0x2) & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:4) - (0 ?
 6:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:4) - (0 ?
 6:4) + 1))))))) << (0 ?
 6:4))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 6:4) - (0 ?
 6:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:4) - (0 ? 6:4) + 1))))))) << (0 ? 6:4)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:24) - (0 ?
 26:24) + 1))))))) << (0 ?
 26:24))) | (((gctUINT32) ((gctUINT32) (0x2) & ((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:24) - (0 ? 26:24) + 1))))))) << (0 ? 26:24)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x022C) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0420) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0403) & ((gctUINT32) ((((1 ?
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
    commands[index++] = RegCount;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0416) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0409) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x021F) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0424) & ((gctUINT32) ((((1 ?
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
    commands[index++] = InstCount / 4;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x040A) & ((gctUINT32) ((((1 ?
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
    commands[index++] = InstAddress;

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI5))
    {
        commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x5580) & ((gctUINT32) ((((1 ?
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
        commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)));
    }
    else
    {
        commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0218) & ((gctUINT32) ((((1 ?
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
        commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)));
    }

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x021A) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0425) & ((gctUINT32) ((((1 ?
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
    commands[index++] = InstCount / 4 - 1;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0402) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) ((gctUINT32) (~0) & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0228) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x02AA) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E07) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x040C) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0201) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E22) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0412) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0240) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) ((gctUINT32) (0x2) & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:4) - (0 ?
 6:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:4) - (0 ?
 6:4) + 1))))))) << (0 ?
 6:4))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 6:4) - (0 ?
 6:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:4) - (0 ? 6:4) + 1))))))) << (0 ? 6:4)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:24) - (0 ?
 26:24) + 1))))))) << (0 ?
 26:24))) | (((gctUINT32) ((gctUINT32) (0x3) & ((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:24) - (0 ? 26:24) + 1))))))) << (0 ? 26:24)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0249) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0247) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ThreadAllocation;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x024B) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalOffsetX;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x024D) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalOffsetY;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x024F) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalOffsetZ;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0256) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalScaleX;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0257) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalScaleY;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0258) & ((gctUINT32) ((((1 ?
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
    commands[index++] = GlobalScaleZ;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0250) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (6) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));
    commands[index++] = groupCountX - 1;
    commands[index++] = groupCountY - 1;
    commands[index++] = groupCountZ - 1;
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:0) - (0 ?
 9:0) + 1))))))) << (0 ?
 9:0))) | (((gctUINT32) ((gctUINT32) (GroupSizeX - 1) & ((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:0) - (0 ? 9:0) + 1))))))) << (0 ? 9:0)));
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:0) - (0 ?
 9:0) + 1))))))) << (0 ?
 9:0))) | (((gctUINT32) ((gctUINT32) (GroupSizeY - 1) & ((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:0) - (0 ? 9:0) + 1))))))) << (0 ? 9:0)));
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:0) - (0 ?
 9:0) + 1))))))) << (0 ?
 9:0))) | (((gctUINT32) ((gctUINT32) (GroupSizeZ - 1) & ((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:0) - (0 ? 9:0) + 1))))))) << (0 ? 9:0)));
    commands[index++] = 0x00000000;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0248) & ((gctUINT32) ((((1 ?
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
    commands[index++] = 0xBADABEEB;

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0594) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
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
    commands[index++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

    bytes = gcmSIZEOF(gctUINT32) * index;

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

/*
**  gckHARDWARE_ResetFlopWithPPU
**
**  Generate the command to do PPU program as follows.
**      InImage: 64x6 = {1}, unsigned int8
**      OutImage: 64x6, unsigned int8
**      OutImage = InImage + InImage
**
**  INPUT:
**
**      gckHARDWARE Hardware
**
**      gctUINT32 AllocFlag
**
**      gcePOOL *Pool
**
**  OUTPUT:
**
**      gcePOOL *Pool
**
**      gcsFUNCTION_COMMAND *Command
*/
gceSTATUS
gckHARDWARE_ResetFlopWithPPU(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND *Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 dataType = 0x7;
    gcsFEATURE_DATABASE *database = gcvNULL;
    gctUINT32 numShaderCores;
    gctUINT32 stride, width, height;
    gctUINT32 workDim;
    gctUINT32 valueOrder;
    gctUINT32 groupSizeX, groupSizeY, groupSizeZ;
    gctUINT32 globalScaleX, globalScaleY, globalScaleZ;
    gctUINT32 globalOffsetX, globalOffsetY, globalOffsetZ;
    gctUINT32 threadAllocation;
    gctUINT32 inImageAddress = 0, outImageAddress = 0, instAddress = 0;
    gctUINT32 instCount = 0, regCount = 0;
    gctUINT32 dataCount;
    gctPOINTER pointer = gcvNULL;
    gcsFUNCTION_EXECUTION_DATA *data = gcvNULL;
    gctUINT32 i;

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_PPU_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA_PTR)pointer;

    database = (gcsFEATURE_DATABASE *)Hardware->featureDatabase;

    numShaderCores = database->NumShaderCores;

    stride = PPU_IMAGE_XSIZE * 1;
    width = PPU_IMAGE_XSIZE;
    height = PPU_IMAGE_YSIZE;

    gcmkONERROR(_ProgramPPUInput(
        Hardware,
        width,
        height,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_PPU_INPUT]
        ));

    gcmkONERROR(_ProgramPPUOutput(
        Hardware,
        width,
        height,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_PPU_OUTPUT]
        ));

    gcmkONERROR(_ProgramPPUInstruction(
        Hardware,
        dataType,
        numShaderCores,
        AllocFlag,
        Pool,
        &instCount,
        &regCount,
        &data[gcvFLOP_RESET_PPU_INSTRUCTION]
        ));

    workDim = 0x2;
    valueOrder = 0x2;
    groupSizeX = 1;
    groupSizeY = 1;
    groupSizeZ = 0;
    globalScaleX = 4;
    globalScaleY = 1;
    globalScaleZ = 0;
    globalOffsetX = 0;
    globalOffsetY = 0;
    globalOffsetZ = 0;
    threadAllocation = (groupSizeX * groupSizeY + numShaderCores * 4 - 1) / (numShaderCores * 4);
    inImageAddress = data[gcvFLOP_RESET_PPU_INPUT].address;
    outImageAddress = data[gcvFLOP_RESET_PPU_OUTPUT].address;
    instAddress = data[gcvFLOP_RESET_PPU_INSTRUCTION].address;

    gcmkONERROR(_ProgramPPUCommand(
        Hardware,
        stride,
        width,
        height,
        workDim,
        valueOrder,
        groupSizeX,
        groupSizeY,
        groupSizeZ,
        globalScaleX,
        globalScaleY,
        globalScaleZ,
        globalOffsetX,
        globalOffsetY,
        globalOffsetZ,
        threadAllocation,
        inImageAddress,
        outImageAddress,
        instAddress,
        instCount,
        regCount,
        AllocFlag,
        Pool,
        Command
        ));

    Command->data = data;
    Command->dataCount = dataCount;
    if(Hardware->identity.customerID == 0x85)
    {
        Command->channelId = 1;
    }

    return gcvSTATUS_OK;

OnError:
    if (Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}

/*
 * NN
 */
static gceSTATUS
_ProgramNNKernel(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress =0;
    gctSIZE_T bufferBytes;
    gctSIZE_T bytes = 0;
    gctUINT32 *buffer = gcvNULL;
    gctBOOL need_refine = (Hardware->identity.customerID == 0x85 && Hardware->options.configNNPowerControl != 0);

    /*define bufferbytes*/
    bytes = bufferBytes = cd->NNkerLen;
    /* hardcode */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    gckOS_MemCopy(bufferLogical, cd->NNKer, bytes);

    if(need_refine)
    {
        buffer[16] = buffer[32];
        buffer[17] = buffer[33];
        buffer[18] = buffer[34];
        buffer[32] = 0x0;
        buffer[33] = 0x0;
        buffer[34] = 0x0;
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));


#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn kernel]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        need_refine? (bytes - 0x40):bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = need_refine ? (bufferBytes - 0x40) : bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = need_refine ? (bytes - 0x40) : bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramNNInput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 InImageZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 inputSize = InImageXSize * InImageYSize * InImageZSize;
    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT8_PTR buffer = gcvNULL;

    gctUINT32 i = 0;
    gctUINT32 offset = 0;

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = inputSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT8_PTR)bufferLogical;

    for (i = 0; i < inputSize; i++)
    {
        _BitValue(&buffer, flopResetInputs[DataType], &offset, itemBytes * 8);
    }

    bytes = buffer + (offset + 7) / 8 - (gctUINT8_PTR)bufferLogical;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramNNOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 OutputXSize,
    IN gctUINT32 OutputYSize,
    IN gctUINT32 OutputZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    if (!Data)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = bytes = (gctSIZE_T)(OutputXSize * OutputYSize * OutputZSize * itemBytes);

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramNNInstruction(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 OutImageXSize,
    IN gctUINT32 OutImageYSize,
    IN gctUINT32 OutImageZSize,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 KernelAddress,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gctUINT32 itemBytes = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT32 *command = gcvNULL;
    gctSIZE_T outbufferBytes = 0;

    bufferBytes = bytes = gckHARDWARE_IsFeatureAvailable(Hardware, gcFEATURE_BIT_NN_TENSOR_ADD_FIELD_MOVE_TO_EXT_CMD)? NN_INSTRUCTION_LEN_EXT:NN_INSTRUCTION_LEN;

    /* Allocate buffer. */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));
    outbufferBytes = (gctSIZE_T)(OutImageXSize * OutImageYSize * OutImageZSize * itemBytes);
    command = (gctUINT32_PTR)bufferLogical;
    gckOS_MemCopy(command, cd->NNIns, bytes);

    /* Fill the data. */
    command[5] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:26) - (0 ?
 31:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:26) - (0 ?
 31:26) + 1))))))) << (0 ?
 31:26))) | (((gctUINT32) ((gctUINT32) (((KernelZSize >> 14) & 0x3F)) & ((gctUINT32) ((((1 ?
 31:26) - (0 ?
 31:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:26) - (0 ? 31:26) + 1))))))) << (0 ? 31:26)))
      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) ((KernelAddress >> 6)) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)));
    command[6] = InImageAddress;
    command[7] = OutImageAddress;

    if(Hardware->identity.customerID == 0x85)
    {
        if(Hardware->options.configNNPowerControl == 3)
        {
            command[12] = 0x00000900;
        }
        else if(Hardware->options.configNNPowerControl == 4)
        {
            command[12] = 0x00000A00;
        }
        else if(Hardware->options.configNNPowerControl == 0)
        {
            command[12] = 0x00000B00;
        }
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramNNCommand(
    IN gckHARDWARE Hardware,
    IN gctUINT32 InstAddress,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gctUINT32 bytes;
    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gctUINT32 *commands = gcvNULL;

    bufferBytes = gcmSIZEOF(gctUINT32) *  MAX_NN_COMMAND_NUM;
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32_PTR)bufferLogical;
    gckOS_MemCopy(commands, cd->NNCmd, cd->NNCmdLen);
    if(Hardware->identity.customerID == 0x85)
    {
         commands[cd->NNCmdOffset - 4] = commands[cd->NNCmdOffset - 4]
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:8) - (0 ?
 11:8) + 1))))))) << (0 ?
 11:8))) | (((gctUINT32) ((gctUINT32) (Hardware->options.configNNPowerControl) & ((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:8) - (0 ? 11:8) + 1))))))) << (0 ? 11:8)));
         commands[cd->NNCmdOffset] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) ((InstAddress >> 6)) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)));
    }
    else
    {
        commands[cd->NNCmdOffset] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:6) - (0 ?
 31:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:6) - (0 ?
 31:6) + 1))))))) << (0 ?
 31:6))) | (((gctUINT32) ((gctUINT32) ((InstAddress >> 6)) & ((gctUINT32) ((((1 ?
 31:6) - (0 ?
 31:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:6) - (0 ? 31:6) + 1))))))) << (0 ? 31:6)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)));
    }

    bytes = cd->NNCmdLen;

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;
#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn commands]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif
    return gcvSTATUS_OK;


OnError:
    if (bufferNode)
    {
        gcmkONERROR(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

gceSTATUS
gckHARDWARE_ResetFlopWithNN(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND *Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 kernelXSize = NN_KERNEL_XSIZE;
    gctUINT32 kernelYSize = NN_KERNEL_YSIZE;
    gctUINT32 kernelZSize = NN_KERNEL_ZSIZE;

    gctUINT32 inImageXSize = NN_INPUT_XSIZE;
    gctUINT32 inImageYSize = NN_INPUT_YSIZE;
    gctUINT32 inImageZSize = NN_INPUT_ZSIZE;

    gctUINT32 outImageXSize = NN_OUTPUT_XSIZE;
    gctUINT32 outImageYSize = NN_OUTPUT_YSIZE;
    gctUINT32 outImageZSize = NN_OUTPUT_ZSIZE;

    gctUINT32 i;
    gctPOINTER pointer = gcvNULL;

    gctUINT8 dataType;
    gctUINT32 itemBytes = 0;
    gcsFUNCTION_EXECUTION_DATA_PTR data = gcvNULL;
    gctUINT32 dataCount = 0;
    chipCmdData *cd = gcvNULL;

#if gcdENABLE_FLOP_RESET_DEBUG
    gctUINT8_PTR golden;
    gctSIZE_T outBufBytes;

#endif

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetVIPCoreInfo(
        Hardware,
        gcvNULL,
        &dataType,
        gcvNULL,
        gcvNULL,
        gcvNULL
        ));

    cd = gcQuerychipCmdDB(Hardware->identity.customerID, Hardware->identity.chipModel, Hardware->identity.chipRevision, Hardware->identity.productID, Hardware->identity.ecoID);
    if(cd == gcvNULL)
    {
        return status;
    }
    gcmkASSERT(dataType == cd->InputDataType);

    gcmkONERROR(_GetNNDataSize(dataType, &itemBytes));

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_NN_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA *)pointer;

    /* Kernel. */
    gcmkONERROR(_ProgramNNKernel(
        Hardware,
        dataType,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        cd,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_KERNEL]
        ));

    /* Input. */
    gcmkONERROR(_ProgramNNInput(
        Hardware,
        dataType,
        inImageXSize,
        inImageYSize,
        inImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_INPUT]
        ));

    /* Output. */
    gcmkONERROR(_ProgramNNOutput(
        Hardware,
        dataType,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_OUTPUT]
        ));

    /* Commands. */
    gcmkONERROR(_ProgramNNInstruction(
        Hardware,
        dataType,
        inImageXSize,
        inImageYSize,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        data[gcvFLOP_RESET_NN_INPUT].address,
        data[gcvFLOP_RESET_NN_OUTPUT].address,
        data[gcvFLOP_RESET_NN_KERNEL].address,
        cd,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_INSTRUCTION]
        ));

    gcmkONERROR(_ProgramNNCommand(
        Hardware,
        data[gcvFLOP_RESET_NN_INSTRUCTION].address,
        cd,
        AllocFlag,
        Pool,
        Command
        ));

#if gcdENABLE_FLOP_RESET_DEBUG
    outBufBytes = outImageXSize * outImageYSize * outImageZSize * itemBytes;

    gcmkONERROR(gckOS_Allocate(Hardware->os, outBufBytes, &Command->golden));
    gckOS_ZeroMemory(Command->golden, outBufBytes);
    golden = (gctUINT8_PTR)Command->golden;
    if(Hardware->identity.customerID == 0x23 || Hardware->identity.customerID == 0x83)
    {
        golden[0] = 0xe3;
        golden[1] = 0x34;
        golden[2] = 0xe3;
        golden[3] = 0x34;
    }
    else if(Hardware->identity.customerID == 0x96)
    {
        golden[0] = 0x50;
        golden[1] = 0x50;
    }
    else
    {
        for (i = 0; i < outBufBytes; ++i)
        {
            golden[i] = '3';
        }
    }
    Command->outlogical = data[gcvFLOP_RESET_NN_OUTPUT].logical;
    Command->outSize = outBufBytes;
#endif

    if(Hardware->identity.customerID == 0x85)
    {
        Command->channelId = 2;
    }
    Command->data = data;
    Command->dataCount = dataCount;

    return gcvSTATUS_OK;

OnError:
    if (Command && Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }
#if gcdENABLE_FLOP_RESET_DEBUG
        if(Command->golden)
        {
            gcmkVERIFY_OK(gckOS_Free(Hardware->os, Command->golden));
            Command->golden = gcvNULL;
        }
#endif

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}

static gceSTATUS
_ProgramTPKernel(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gctSIZE_T bytes;
    gctUINT32 *buffer = gcvNULL;

    bytes = bufferBytes = cd->TPkerLen;
    /* hardcode */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    gckOS_MemCopy(bufferLogical, cd->TPKer, bytes);

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP kernel]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramTPInput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 InImageZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 inputSize = InImageXSize * InImageYSize * InImageZSize;
    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT8_PTR buffer = gcvNULL;
    gctUINT32 i = 0;
    gctUINT32 offset = 0;

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = inputSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT8_PTR)bufferLogical;

    for (i = 0; i < inputSize; i++)
    {
        _BitValue(&buffer, flopResetInputs[DataType], &offset, itemBytes * 8);
    }

    bytes = buffer + (offset + 7) / 8 - (gctUINT8_PTR)bufferLogical;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramTPOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 OutputXSize,
    IN gctUINT32 OutputYSize,
    IN gctUINT32 OutputZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    if (!Data)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = bytes = (gctSIZE_T)(OutputXSize * OutputYSize * OutputZSize * itemBytes);

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramTPInstruction(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 OutImageXSize,
    IN gctUINT32 OutImageYSize,
    IN gctUINT32 OutImageZSize,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 KernelAddress,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes;
    gctSIZE_T bytes;
    gctUINT32 *command = gcvNULL;
    gctUINT32 i;
    gctSIZE_T steps = 1;
    gctUINT32 KernelAnchor = 0, OutImageAnchor = 0;

    bytes = bufferBytes = cd->TPCoreCount * TP_INSTRUCTION_LEN;
    /* Allocate buffer. */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));
    gckOS_MemCopy(bufferLogical, cd->TPIns, bytes);
    command = (gctUINT32_PTR)bufferLogical;
    steps = bytes / TP_INSTRUCTION_LEN;
    KernelAnchor = command[11];
    OutImageAnchor = command[13];
    for(i = 0; i < steps; ++i)
    {
        command[i*32 + 10] = InImageAddress;
        command[i*32 + 11] = KernelAddress + command[i*32 + 11] - KernelAnchor;
        command[i*32 + 13] = OutImageAddress + command[i*32 + 13] - OutImageAnchor;
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramTPCommand(
    IN gckHARDWARE Hardware,
    IN gctUINT32 InstAddress,
    IN chipCmdData* cd,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes;
    gctUINT32 bytes;
    gctUINT32 *commands;
    gctUINT32 startAnchor = 0;

    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gctUINT32 i = 0;
    gctUINT32 k = 1;

    bufferBytes = gcmSIZEOF(gctUINT32) * 280;
    bytes = cd->TPCmdLen;


    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32_PTR)bufferLogical;
    gckOS_MemCopy(commands, cd->TPCmd, bytes);

    i = cd->TPCoreCount;
    startAnchor = commands[cd->TPCmdOffset[0]];
    commands[cd->TPCmdOffset[0]] = (cd->TPCoreCount == 1)?(InstAddress & 0xffffffC0):((InstAddress & 0xffffffC0) | (0x1));
    if(Hardware->identity.customerID == 0x85)
    {
        commands[cd->TPCmdOffset[0]] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) ((InstAddress >> 6)) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)));
    }
    for(k = 1; k < i; k++)
    {
        commands[cd->TPCmdOffset[k]] =  commands[cd->TPCmdOffset[0]] + commands[cd->TPCmdOffset[k]] - startAnchor;
    }

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);


    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP Command]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkONERROR(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

/*
 * TP.
 */
gceSTATUS
gckHARDWARE_ResetFlopWithTP(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 kernelXSize = TP_KERNEL_XSIZE;
    gctUINT32 kernelYSize = TP_KERNEL_YSIZE;
    gctUINT32 kernelZSize = TP_KERNEL_ZSIZE;

    gctUINT32 inImageXSize = TP_INPUT_XSIZE;
    gctUINT32 inImageYSize = TP_INPUT_YSIZE;
    gctUINT32 inImageZSize = TP_INPUT_ZSIZE;

    gctUINT32 outImageXSize = TP_OUTPUT_XSIZE;
    gctUINT32 outImageYSize = TP_OUTPUT_YSIZE;
    gctUINT32 outImageZSize = TP_OUTPUT_ZSIZE;

    gctUINT32 i;
    gctPOINTER pointer = gcvNULL;

    gctUINT8 dataType;
    gctUINT32 itemBytes = 0;
    gcsFUNCTION_EXECUTION_DATA_PTR data = gcvNULL;
    gctUINT32 dataCount = 0;
    chipCmdData* cd;

#if gcdENABLE_FLOP_RESET_DEBUG
    gctUINT8_PTR golden;
    gctSIZE_T outBufBytes;

#endif

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    cd = gcQuerychipCmdDB(Hardware->identity.customerID, Hardware->identity.chipModel, Hardware->identity.chipRevision, Hardware->identity.productID, Hardware->identity.ecoID);
    if(cd == gcvNULL)
    {
        return status;
    }
    dataType = cd->InputDataType;
    gcmkONERROR(_GetNNDataSize(dataType, &itemBytes));

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_TP_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA *)pointer;

    /* Kernel. */
    gcmkONERROR(_ProgramTPKernel(
        Hardware,
        dataType,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        cd,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_KERNEL]
        ));

    /* Input. */
    gcmkONERROR(_ProgramTPInput(
        Hardware,
        dataType,
        inImageXSize,
        inImageYSize,
        inImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_INPUT]
        ));

    /* Output. */
    gcmkONERROR(_ProgramTPOutput(
        Hardware,
        dataType,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_OUTPUT]
        ));

    /* Commands. */
    gcmkONERROR(_ProgramTPInstruction(
        Hardware,
        dataType,
        inImageXSize,
        inImageYSize,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        data[gcvFLOP_RESET_TP_INPUT].address,
        data[gcvFLOP_RESET_TP_OUTPUT].address,
        data[gcvFLOP_RESET_TP_KERNEL].address,
        cd,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_INSTRUCTION]
        ));

    gcmkONERROR(_ProgramTPCommand(
        Hardware,
        data[gcvFLOP_RESET_TP_INSTRUCTION].address,
        cd,
        AllocFlag,
        Pool,
        Command
        ));

#if gcdENABLE_FLOP_RESET_DEBUG
    outBufBytes = outImageXSize * outImageYSize * outImageZSize * itemBytes;
    gcmkONERROR(gckOS_Allocate(Hardware->os, outBufBytes, &Command->golden));
    gckOS_ZeroMemory(Command->golden, outBufBytes);
    golden = (gctUINT8_PTR)Command->golden;
    if(Hardware->identity.customerID == 0x23 || Hardware->identity.customerID == 0x83)
    {
        golden[0] = 0x19;
        golden[1] = 0xa9;
        golden[2] = 0x31;
        golden[3] = 0x31;
        golden[4] = 0xf9;
        golden[5] = 0x35;
        golden[6] = 0x53;
        golden[7] = 0x30;
        golden[8] = 0x3c;
        golden[9] = 0x29;
        golden[10] = 0x3b;
        golden[11] = 0x37;
        golden[12] = 0x24;
        golden[13] = 0x38;
        golden[14] = 0x33;
        golden[15] = 0x37;
        golden[16] = 0xd;
        golden[17] = 0x39;
        golden[18] = 0x6e;
        golden[19] = 0x32;
        golden[20] = 0x68;
        golden[21] = 0xba;
        golden[22] = 0x40;
        golden[23] = 0x2a;
        golden[24] = 0xab;
        golden[25] = 0xa9;
        golden[26] = 0xb;
        golden[27] = 0xb4;
        golden[28] = 0xe8;
        golden[29] = 0xab;
        golden[30] = 0x9e;
        golden[31] = 0x30;
        golden[32] = 0xf0;
        golden[33] = 0x20;
        golden[34] = 0x98;
        golden[35] = 0xb6;
        golden[36] = 0x5a;
        golden[37] = 0xb1;
        golden[38] = 0x90;
        golden[39] = 0xb4;
        golden[40] = 0xff;
        golden[41] = 0x38;
        golden[42] = 0x7c;
        golden[43] = 0xb4;
        golden[44] = 0xb6;
        golden[45] = 0x31;
        golden[46] = 0x34;
        golden[47] = 0xae;
        golden[48] = 0xa3;
        golden[49] = 0x38;
        golden[50] = 0xb4;
        golden[51] = 0x32;
        golden[52] = 0x5f;
        golden[53] = 0x31;
        golden[54] = 0x12;
        golden[55] = 0x34;
        golden[56] = 0xc0;
        golden[57] = 0x34;
        golden[58] = 0xac;
        golden[59] = 0xa6;
        golden[60] = 0x6f;
        golden[61] = 0x38;
        golden[62] = 0xfd;
        golden[63] = 0x34;
        golden[64] = 0xa7;
        golden[65] = 0xb7;
        golden[66] = 0xa0;
        golden[67] = 0x33;
        golden[68] = 0x89;
        golden[69] = 0x34;
        golden[70] = 0xb6;
        golden[71] = 0x33;
        golden[72] = 0x80;
        golden[73] = 0x94;
        golden[74] = 0x5d;
        golden[75] = 0xb2;
        golden[76] = 0x68;
        golden[77] = 0x37;
        golden[78] = 0xbb;
        golden[79] = 0xb1;
        golden[80] = 0x23;
        golden[81] = 0xb5;
        golden[82] = 0xc3;
        golden[83] = 0x28;
        golden[84] = 0xac;
        golden[85] = 0x35;
        golden[86] = 0x8a;
        golden[87] = 0xb3;
        golden[88] = 0x12;
        golden[89] = 0x34;
        golden[90] = 0x47;
        golden[91] = 0xb4;
        golden[92] = 0xa6;
        golden[93] = 0x32;
        golden[94] = 0x86;
        golden[95] = 0xb1;
        golden[96] = 0x83;
        golden[97] = 0xae;
        golden[98] = 0x6a;
        golden[99] = 0x32;
        golden[100] = 0x1a;
        golden[101] = 0xb1;
        golden[102] = 0x99;
        golden[103] = 0xb4;
        golden[104] = 0xcd;
        golden[105] = 0x32;
        golden[106] = 0x78;
        golden[107] = 0xb4;
        golden[108] = 0x66;
        golden[109] = 0x8a;
        golden[110] = 0xa6;
        golden[111] = 0xad;
        golden[112] = 0xf3;
        golden[113] = 0x2f;
        golden[114] = 0x79;
        golden[115] = 0xa0;
        golden[116] = 0x15;
        golden[117] = 0xb5;
        golden[118] = 0x1a;
        golden[119] = 0xb5;
        golden[120] = 0x4a;
        golden[121] = 0xb5;
        golden[122] = 0x4;
        golden[123] = 0xb8;
        golden[124] = 0xdc;
        golden[125] = 0x2f;
        golden[126] = 0x8e;
        golden[127] = 0x31;
    }
    else if(Hardware->identity.customerID == 0x96)
    {
        golden[0] = 0x32;
        golden[1] = 0x0;
        golden[2] = 0xfe;
        golden[3] = 0xfa;
        golden[4] = 0x16;
        golden[5] = 0xfe;
        golden[6] = 0xfc;
        golden[7] = 0xf7;
        golden[8] = 0x2;
        golden[9] = 0xf;
        golden[10] = 0x3;
        golden[11] = 0xf;
        golden[12] = 0x18;
        golden[13] = 0xec;
        golden[14] = 0x6;
        golden[15] = 0xf6;
        golden[16] = 0xf7;
        golden[17] = 0xc;
        golden[18] = 0xf8;
        golden[19] = 0x4;
        golden[20] = 0xef;
        golden[21] = 0x4;
        golden[22] = 0xea;
        golden[23] = 0xfa;
        golden[24] = 0xf4;
        golden[25] = 0xd;
        golden[26] = 0xe;
        golden[27] = 0xfd;
        golden[28] = 0xee;
        golden[29] = 0xff;
        golden[30] = 0xe6;
        golden[31] = 0xfc;
        golden[32] = 0x13;
        golden[33] = 0x1;
        golden[34] = 0xf7;
        golden[35] = 0xdf;
        golden[36] = 0xe9;
        golden[37] = 0xec;
        golden[38] = 0xf;
        golden[39] = 0xf0;
        golden[40] = 0xf0;
        golden[41] = 0xb;
        golden[42] = 0xa;
        golden[43] = 0xe7;
        golden[44] = 0x0;
        golden[45] = 0xec;
        golden[46] = 0x1b;
        golden[47] = 0xf4;
        golden[48] = 0xee;
        golden[49] = 0x1b;
        golden[50] = 0xe2;
        golden[51] = 0x20;
        golden[52] = 0xe9;
        golden[53] = 0x1;
        golden[54] = 0xfe;
        golden[55] = 0x1;
        golden[56] = 0x17;
        golden[57] = 0xf7;
        golden[58] = 0x26;
        golden[59] = 0x8;
        golden[60] = 0xd8;
        golden[61] = 0xf3;
        golden[62] = 0x5;
        golden[63] = 0x1c;
    }
    else
    {
        for(i = 0; i < outBufBytes; ++i)
        {
            golden[i] = '3';
        }
    }

    Command->outlogical = data[gcvFLOP_RESET_TP_OUTPUT].logical;
    Command->outSize = outBufBytes;
#endif

    if(Hardware->identity.customerID == 0x85)
    {
        Command->channelId = 3;
    }
    Command->data = data;
    Command->dataCount = dataCount;


    return gcvSTATUS_OK;

OnError:
    if (Command && Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }
#if gcdENABLE_FLOP_RESET_DEBUG
        if(Command->golden)
        {
            gcmkVERIFY_OK(gckOS_Free(Hardware->os, Command->golden));
            Command->golden = gcvNULL;
        }
#endif

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}

