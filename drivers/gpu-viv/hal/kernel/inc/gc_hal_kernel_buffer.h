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


#ifndef __gc_hal_kernel_buffer_h_
#define __gc_hal_kernel_buffer_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
************************ Command Buffer and Event Objects **********************
\******************************************************************************/

#define gcdRESERVED_FLUSHCACHE_LENGTH               (2 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_PAUSE_OQ_LENGTH                 (2 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_PAUSE_XFBWRITTEN_QUERY_LENGTH   (4 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_PAUSE_PRIMGEN_QUERY_LENGTH      (4 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_PAUSE_XFB_LENGTH                (2 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_HW_FENCE_32BIT                  (4 * gcmSIZEOF(gctUINT32))
#define gcdRESERVED_HW_FENCE_64BIT                  (6 * gcmSIZEOF(gctUINT32))

#define gcdRESUME_OQ_LENGTH                         (2 * gcmSIZEOF(gctUINT32))
#define gcdRESUME_XFBWRITTEN_QUERY_LENGTH           (4 * gcmSIZEOF(gctUINT32))
#define gcdRESUME_PRIMGEN_QUERY_LENGTH              (4 * gcmSIZEOF(gctUINT32))
#define gcdRESUME_XFB_LENGH                         (2 * gcmSIZEOF(gctUINT32))

#define FENCE_NODE_LIST_INIT_COUNT         100

typedef struct _gcsFENCE_APPEND_NODE
{
    gcsSURF_NODE_PTR    node;
    gceFENCE_TYPE       type;

}gcsFENCE_APPEND_NODE;

typedef gcsFENCE_APPEND_NODE   *   gcsFENCE_APPEND_NODE_PTR;

typedef struct _gcsFENCE_LIST    *   gcsFENCE_LIST_PTR;

typedef struct _gcsFENCE_LIST
{
    /* Resource that need get fence, but command used this resource not generated */
    gcsFENCE_APPEND_NODE_PTR        pendingList;
    gctUINT                         pendingCount;
    gctUINT                         pendingAllocCount;

    /* Resoure that already generated command in this command buffer but not get fence */
    gcsFENCE_APPEND_NODE_PTR        onIssueList;
    gctUINT                         onIssueCount;
    gctUINT                         onIssueAllocCount;
}
gcsFENCE_LIST;

/* Command buffer object. */
/*
 * Initial (before put commands):
 *
 *    +-------------------------------------------
 * ...|reservedHead|
 *    +-------------------------------------------
 *    ^            ^
 *    |            |
 * startOffset   offset
 *
 *
 * After put command, in commit:
 *
 *    +------------------------------------------+
 * .. |reservedHead| .. commands .. |reservedTail| ..
 *    +------------------------------------------+
 *    ^                             ^
 *    |                             |
 * startOffset                    offset
 *
 *
 * Commit done, becomes initial state:
 *
 *    +------------------------------------------+-----------------
 * .. |reservedHead| .. commands .. |reservedTail|reservedHead| ..
 *    +------------------------------------------+-----------------
 *                                               ^            ^
 *                                               |            |
 *                                          startOffset    offset
 *
 * reservedHead:
 * Select pipe commands.
 *
 * reservedTail:
 * Link, Fence, ChipEnable
 *
 */
struct _gcoCMDBUF
{
    /* The object. */
    gcsOBJECT                   object;

    /* Commit count. */
    gctUINT64                   commitCount;

    /* Command buffer entry and exit pipes. */
    gcePIPE_SELECT              entryPipe;
    gcePIPE_SELECT              exitPipe;

    /* Feature usage flags. */
    gctBOOL                     using2D;
    gctBOOL                     using3D;

    /* Size of reserved head and tail for each commit. */
    gctUINT32                   reservedHead;
    gctUINT32                   reservedTail;

    /* Video memory handle of command buffer. */
    gctUINT32                   videoMemNode;

    /* GPU address of command buffer. */
    gctUINT32                   address;

    /* Logical address of command buffer. */
    gctUINT64                   logical;

    /* Number of bytes in command buffer. */
    gctUINT32                   bytes;

    /* Start offset into the command buffer. */
    gctUINT32                   startOffset;

    /* Current offset into the command buffer. */
    gctUINT32                   offset;

    /* Number of free bytes in command buffer. */
    gctUINT32                   free;

    /* Location of the last reserved area. */
    gctUINT64                   lastReserve;
    gctUINT32                   lastOffset;

    /* Last load state command location and hardware address. */
    gctUINT64                   lastLoadStatePtr;
    gctUINT32                   lastLoadStateAddress;
    gctUINT32                   lastLoadStateCount;

    /*
    * Put pointer type member after this line.
    */

    /* Completion signal. */
    gctSIGNAL                   signal;

    /* Link to the siblings. */
    gcoCMDBUF                   prev;
    gcoCMDBUF                   next;

    /* Mirror command buffer(s). */
    gcoCMDBUF                   *mirrors;
    gctUINT32                   mirrorCount;
};

/* Event queue. */
struct _gcoQUEUE
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to current event queue. */
    gcsQUEUE_PTR                head;
    gcsQUEUE_PTR                tail;

    /* List of free records. */
    gcsQUEUE_PTR                freeList;

    /* chunks of the records. */
    gcsQUEUE_CHUNK *            chunks;

    #define gcdIN_QUEUE_RECORD_LIMIT 16
    /* Number of records currently in queue */
    gctUINT32                   recordCount;
    /* Number of records which release resource currently in queue */
    gctUINT32                   tmpBufferRecordCount;

    /* Max size of pending unlock node in vidmem pool not committed */
    gctUINT                     maxUnlockBytes;

    gceENGINE                   engine;

    /* Pointer to gcoHARDWARE object. */
    gcoHARDWARE                 hardware;

    /* Brother cores mask in one queue. */
    gctUINT32                   broCoreMask;

#if gcdENABLE_SW_PREEMPTION
    gctUINT                     priorityID;
    gctBOOL                     topPriority;
#endif
};

struct _gcsTEMPCMDBUF
{
    gctUINT32 currentByteSize;
    gctPOINTER buffer;
    gctBOOL  inUse;
};

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_kernel_buffer_h_ */
