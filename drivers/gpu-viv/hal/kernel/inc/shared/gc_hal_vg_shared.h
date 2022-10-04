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


#ifndef __gc_hal_shared_vg_h_
#define __gc_hal_shared_vg_h_

#ifdef __cplusplus
extern "C" {
#endif

/* Command buffer header. */
typedef struct _gcsCMDBUFFER * gcsCMDBUFFER_PTR;
typedef struct _gcsCMDBUFFER
{
    /* Pointer to the completion signal. */
    gcsCOMPLETION_SIGNAL_PTR    completion;

    /* The user sets this to the node of the container buffer whitin which
       this particular command buffer resides. The kernel sets this to the
       node of the internally allocated buffer. */
    gcuVIDMEM_NODE_PTR          node;

    /* Command buffer hardware address. */
    gctUINT32                   address;

    /* The offset of the buffer from the beginning of the header. */
    gctUINT32                   bufferOffset;

    /* Size of the area allocated for the data portion of this particular
       command buffer (headers and tail reserves are excluded). */
    gctUINT32                   size;

    /* Offset into the buffer [0..size]; reflects exactly how much data has
       been put into the command buffer. */
    gctUINT                     offset;

    /* The number of command units in the buffer for the hardware to
       execute. */
    gctUINT32                   dataCount;

    /* MANAGED BY : user HAL (gcoBUFFER object).
       USED BY    : user HAL (gcoBUFFER object).
       Points to the immediate next allocated command buffer. */
    gcsCMDBUFFER_PTR            nextAllocated;

    /* MANAGED BY : user layers (HAL and drivers).
       USED BY    : kernel HAL (gcoBUFFER object).
       Points to the next subbuffer if any. A family of subbuffers are chained
       together and are meant to be executed inseparably as a unit. Meaning
       that context switching cannot occur while a chain of subbuffers is being
       executed. */
    gcsCMDBUFFER_PTR            nextSubBuffer;
}
gcsCMDBUFFER;

/* Command queue element. */
typedef struct _gcsVGCMDQUEUE
{
    /* Pointer to the command buffer header. */
    gcsCMDBUFFER_PTR            commandBuffer;

    /* Dynamic vs. static command buffer state. */
    gctBOOL                     dynamic;
}
gcsVGCMDQUEUE;

/* Context map entry. */
typedef struct _gcsVGCONTEXT_MAP
{
    /* State index. */
    gctUINT32                   index;

    /* New state value. */
    gctUINT32                   data;

    /* Points to the next entry in the mod list. */
    gcsVGCONTEXT_MAP_PTR            next;
}
gcsVGCONTEXT_MAP;

/* gcsVGCONTEXT structure that holds the current context. */
typedef struct _gcsVGCONTEXT
{
    /* Context ID. */
    gctUINT64                   id;

    /* State caching ebable flag. */
    gctBOOL                     stateCachingEnabled;

    /* Current pipe. */
    gctUINT32                   currentPipe;

    /* State map/mod buffer. */
    gctUINT32                   mapFirst;
    gctUINT32                   mapLast;
    gcsVGCONTEXT_MAP_PTR        mapContainer;
    gcsVGCONTEXT_MAP_PTR        mapPrev;
    gcsVGCONTEXT_MAP_PTR        mapCurr;
    gcsVGCONTEXT_MAP_PTR        firstPrevMap;
    gcsVGCONTEXT_MAP_PTR        firstCurrMap;

    /* Main context buffer. */
    gcsCMDBUFFER_PTR            header;
    gctUINT32_PTR               buffer;

    /* Completion signal. */
    gctHANDLE                   process;
    gctSIGNAL                   signal;

#if defined(__QNXNTO__)
    gctSIGNAL                   userSignal;
    gctINT32                    coid;
    gctINT32                    rcvid;
#endif
}
gcsVGCONTEXT;

/* User space task header. */
typedef struct _gcsTASK * gcsTASK_PTR;
typedef struct _gcsTASK
{
    /* Pointer to the next task for the same interrupt in user space. */
    gcsTASK_PTR                 next;

    /* Size of the task data that immediately follows the structure. */
    gctUINT                     size;

    /* Task data starts here. */
    /* ... */
}
gcsTASK;

/* User space task master table entry. */
typedef struct _gcsTASK_MASTER_ENTRY * gcsTASK_MASTER_ENTRY_PTR;
typedef struct _gcsTASK_MASTER_ENTRY
{
    /* Pointers to the head and to the tail of the task chain. */
    gcsTASK_PTR                 head;
    gcsTASK_PTR                 tail;
}
gcsTASK_MASTER_ENTRY;

/* User space task master table entry. */
typedef struct _gcsTASK_MASTER_TABLE
{
    /* Table with one entry per block. */
    gcsTASK_MASTER_ENTRY        table[gcvBLOCK_COUNT];

    /* The total number of tasks sckeduled. */
    gctUINT                     count;

    /* The total size of event data in bytes. */
    gctUINT                     size;

#if defined(__QNXNTO__)
    gctINT32                    coid;
    gctINT32                    rcvid;
#endif
}
gcsTASK_MASTER_TABLE;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __gc_hal_shared_h_ */


