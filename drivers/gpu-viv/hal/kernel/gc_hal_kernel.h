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


#ifndef __gc_hal_kernel_h_
#define __gc_hal_kernel_h_

#include "gc_hal.h"
#include "gc_hal_kernel_hardware.h"
#include "gc_hal_kernel_hardware_fe.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel_mutex.h"
#include "gc_hal_metadata.h"

#if gcdENABLE_SW_PREEMPTION
#include "gc_hal_kernel_preemption.h"
#endif


#if gcdENABLE_TRUST_APPLICATION
#include "gc_hal_security_interface.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define gcdRECOVERY_FORCE_TIMEOUT 100

/*******************************************************************************
***** New MMU Defination *******************************************************/

#if gcdENABLE_MMU_1KMODE
/* 1k mode */
#define gcdMMU_MTLB_SHIFT           24
#define gcdMMU_STLB_4K_SHIFT        12
#define gcdMMU_STLB_64K_SHIFT       16
#define gcdMMU_STLB_1M_SHIFT        20
#define gcdMMU_STLB_16M_SHIFT       24

#define gcdMMU_MTLB_BITS            (32 - gcdMMU_MTLB_SHIFT)
#define gcdMMU_PAGE_4K_BITS         gcdMMU_STLB_4K_SHIFT
#define gcdMMU_STLB_4K_BITS         (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_4K_BITS)
#define gcdMMU_PAGE_64K_BITS        gcdMMU_STLB_64K_SHIFT
#define gcdMMU_STLB_64K_BITS        (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_64K_BITS)
#define gcdMMU_PAGE_1M_BITS         gcdMMU_STLB_1M_SHIFT
#define gcdMMU_STLB_1M_BITS        (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_1M_BITS)
#define gcdMMU_PAGE_16M_BITS        gcdMMU_STLB_16M_SHIFT
#define gcdMMU_STLB_16M_BITS        4


#define gcdMMU_MTLB_ENTRY_NUM       (1 << gcdMMU_MTLB_BITS)
#define gcdMMU_MTLB_SIZE            (gcdMMU_MTLB_ENTRY_NUM << 2)
#define gcdMMU_STLB_4K_ENTRY_NUM    (1 << gcdMMU_STLB_4K_BITS)
#define gcdMMU_STLB_4K_SIZE         (gcdMMU_STLB_4K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_4K_SIZE         (1 << gcdMMU_STLB_4K_SHIFT)
#define gcdMMU_STLB_64K_ENTRY_NUM   (1 << gcdMMU_STLB_64K_BITS)
#define gcdMMU_STLB_64K_SIZE        (gcdMMU_STLB_64K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_64K_SIZE        (1 << gcdMMU_STLB_64K_SHIFT)
#define gcdMMU_STLB_1M_ENTRY_NUM   (1 << gcdMMU_STLB_1M_BITS)
#define gcdMMU_STLB_1M_SIZE        (gcdMMU_STLB_1M_ENTRY_NUM << 2)
#define gcdMMU_PAGE_1M_SIZE        (1 << gcdMMU_STLB_1M_SHIFT)
#define gcdMMU_STLB_16M_ENTRY_NUM   (1 << gcdMMU_STLB_16M_BITS)
#define gcdMMU_STLB_16M_SIZE        (gcdMMU_STLB_16M_ENTRY_NUM << 2)
#define gcdMMU_PAGE_16M_SIZE        (1 << gcdMMU_STLB_16M_SHIFT)


#define gcdMMU_MTLB_MASK            (~((1U << gcdMMU_MTLB_SHIFT)-1))
#define gcdMMU_STLB_4K_MASK         ((~0U << gcdMMU_STLB_4K_SHIFT) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_4K_MASK         (gcdMMU_PAGE_4K_SIZE - 1)
#define gcdMMU_STLB_64K_MASK        ((~((1U << gcdMMU_STLB_64K_SHIFT)-1)) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_64K_MASK        (gcdMMU_PAGE_64K_SIZE - 1)
#define gcdMMU_STLB_1M_MASK        ((~((1U << gcdMMU_STLB_1M_SHIFT)-1)) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_1M_MASK        (gcdMMU_PAGE_1M_SIZE - 1)
#define gcdMMU_STLB_16M_MASK        0x0F000000
#define gcdMMU_PAGE_16M_MASK        (gcdMMU_PAGE_16M_SIZE - 1)


/* Page offset definitions. */
#define gcdMMU_OFFSET_4K_BITS       (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_4K_BITS)
#define gcdMMU_OFFSET_4K_MASK       ((1U << gcdMMU_OFFSET_4K_BITS) - 1)
#define gcdMMU_OFFSET_64K_BITS      (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_64K_BITS)
#define gcdMMU_OFFSET_64K_MASK      ((1U << gcdMMU_OFFSET_64K_BITS) - 1)
#define gcdMMU_OFFSET_1M_BITS      (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_1M_BITS)
#define gcdMMU_OFFSET_1M_MASK      ((1U << gcdMMU_OFFSET_1M_BITS) - 1)
#define gcdMMU_OFFSET_16M_BITS      (32 - gcdMMU_MTLB_BITS)
#define gcdMMU_OFFSET_16M_MASK      ((1U << gcdMMU_OFFSET_16M_BITS) - 1)


#define gcdMMU_MTLB_ENTRY_HINTS_BITS 6
#define gcdMMU_MTLB_ENTRY_STLB_MASK  (~((1U << gcdMMU_MTLB_ENTRY_HINTS_BITS) - 1))

#define gcdMMU_MTLB_PRESENT         0x00000001
#define gcdMMU_MTLB_EXCEPTION       0x00000002
#define gcdMMU_MTLB_4K_PAGE         (0 << 2)
#define gcdMMU_MTLB_64K_PAGE        (1 << 2)
#define gcdMMU_MTLB_1M_PAGE         (2 << 2)
#define gcdMMU_MTBL_16M_PAGE        (3 << 2)

#define gcdMMU_STLB_PRESENT         0x00000001
#define gcdMMU_STLB_EXCEPTION       0x00000002
#define gcdMMU_STBL_WRITEABLE       0x00000004

#else /* 4K mode */

#define gcdMMU_MTLB_SHIFT           22
#define gcdMMU_STLB_4K_SHIFT        12
#define gcdMMU_STLB_64K_SHIFT       16

#define gcdMMU_MTLB_BITS            (32 - gcdMMU_MTLB_SHIFT)
#define gcdMMU_PAGE_4K_BITS         gcdMMU_STLB_4K_SHIFT
#define gcdMMU_STLB_4K_BITS         (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_4K_BITS)
#define gcdMMU_PAGE_64K_BITS        gcdMMU_STLB_64K_SHIFT
#define gcdMMU_STLB_64K_BITS        (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_64K_BITS)

#define gcdMMU_MTLB_ENTRY_NUM       (1 << gcdMMU_MTLB_BITS)
#define gcdMMU_MTLB_SIZE            (gcdMMU_MTLB_ENTRY_NUM << 2)
#define gcdMMU_STLB_4K_ENTRY_NUM    (1 << gcdMMU_STLB_4K_BITS)
#define gcdMMU_STLB_4K_SIZE         (gcdMMU_STLB_4K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_4K_SIZE         (1 << gcdMMU_STLB_4K_SHIFT)
#define gcdMMU_STLB_64K_ENTRY_NUM   (1 << gcdMMU_STLB_64K_BITS)
#define gcdMMU_STLB_64K_SIZE        (gcdMMU_STLB_64K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_64K_SIZE        (1 << gcdMMU_STLB_64K_SHIFT)

#define gcdMMU_MTLB_MASK            (~((1U << gcdMMU_MTLB_SHIFT)-1))
#define gcdMMU_STLB_4K_MASK         ((~0U << gcdMMU_STLB_4K_SHIFT) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_4K_MASK         (gcdMMU_PAGE_4K_SIZE - 1)
#define gcdMMU_STLB_64K_MASK        ((~((1U << gcdMMU_STLB_64K_SHIFT)-1)) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_64K_MASK        (gcdMMU_PAGE_64K_SIZE - 1)

/* Page offset definitions. */
#define gcdMMU_OFFSET_4K_BITS       (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_4K_BITS)
#define gcdMMU_OFFSET_4K_MASK       ((1U << gcdMMU_OFFSET_4K_BITS) - 1)
#define gcdMMU_OFFSET_64K_BITS      (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_64K_BITS)
#define gcdMMU_OFFSET_64K_MASK      ((1U << gcdMMU_OFFSET_64K_BITS) - 1)

#define gcdMMU_MTLB_ENTRY_HINTS_BITS 6
#define gcdMMU_MTLB_ENTRY_STLB_MASK  (~((1U << gcdMMU_MTLB_ENTRY_HINTS_BITS) - 1))

#define gcdMMU_MTLB_PRESENT         0x00000001
#define gcdMMU_MTLB_EXCEPTION       0x00000002
#define gcdMMU_MTLB_4K_PAGE         (0 << 2)
#define gcdMMU_MTLB_64K_PAGE        (1 << 2)


#define gcdMMU_STLB_PRESENT         0x00000001
#define gcdMMU_STLB_EXCEPTION       0x00000002
#define gcdMMU_STLB_WRITEABLE       0x00000004

#endif

#define gcd1M_PAGE_SIZE (1 << 20)
#define gcd1M_PAGE_SHIFT 20

/*******************************************************************************
***** Stuck Dump Level ********************************************************/

/* Dump nonthing when stuck happens. */
#define gcvSTUCK_DUMP_NONE          0

/* Dump GPU state and memory near stuck point. */
#define gcvSTUCK_DUMP_NEARBY_MEMORY 1

/* Beside gcvSTUCK_DUMP_NEARBY_MEMORY, dump context buffer and user command buffer. */
#define gcvSTUCK_DUMP_USER_COMMAND  2

/* Beside gcvSTUCK_DUMP_USER_COMMAND, commit will be stall
** to make sure command causing stuck isn't missed. */
#define gcvSTUCK_DUMP_STALL_COMMAND 3

/* Beside gcvSTUCK_DUMP_USER_COMMAND, dump kernel command buffer. */
#define gcvSTUCK_DUMP_ALL_COMMAND   4

/* Dump all the cores with level 4 dump. */
#define gcvSTUCK_DUMP_ALL_CORE      5

/*******************************************************************************
***** Page table **************************************************************/

#define gcvPAGE_TABLE_DIRTY_BIT_OTHER   (1 << 0)
#define gcvPAGE_TABLE_DIRTY_BIT_FE      (1 << 1)

/*******************************************************************************
***** Process Database Management *********************************************/

typedef enum _gceDATABASE_TYPE
{
    gcvDB_VIDEO_MEMORY = 1,             /* Video memory created. */
    gcvDB_COMMAND_BUFFER,               /* Command Buffer. */
    gcvDB_NON_PAGED,                    /* Non paged memory. */
    gcvDB_CONTIGUOUS,                   /* Contiguous memory. */
    gcvDB_SIGNAL,                       /* Signal. */
    gcvDB_VIDEO_MEMORY_LOCKED,          /* Video memory locked. */
    gcvDB_CONTEXT,                      /* Context */
    gcvDB_IDLE,                         /* GPU idle. */
    gcvDB_MAP_MEMORY,                   /* Map memory */
    gcvDB_MAP_USER_MEMORY,              /* Map user memory */
    gcvDB_SHBUF,                        /* Shared buffer. */
#if gcdENABLE_SW_PREEMPTION
    gcvDB_PRIORITY,
#endif

    gcvDB_NUM_TYPES,
}
gceDATABASE_TYPE;

#define gcdDATABASE_TYPE_MASK           0x000000FF
#define gcdDB_VIDEO_MEMORY_TYPE_MASK    0x0000FF00
#define gcdDB_VIDEO_MEMORY_TYPE_SHIFT   8

#define gcdDB_VIDEO_MEMORY_POOL_MASK    0x00FF0000
#define gcdDB_VIDEO_MEMORY_POOL_SHIFT   16

typedef struct _gcsDATABASE_RECORD *    gcsDATABASE_RECORD_PTR;
typedef struct _gcsDATABASE_RECORD
{
    /* Pointer to kernel. */
    gckKERNEL                           kernel;

    /* Pointer to next database record. */
    gcsDATABASE_RECORD_PTR              next;

    /* Type of record. */
    gceDATABASE_TYPE                    type;

    /* Data for record. */
    gctPOINTER                          data;
    gctPHYS_ADDR                        physical;
    gctSIZE_T                           bytes;
}
gcsDATABASE_RECORD;

typedef struct _gcsDATABASE *           gcsDATABASE_PTR;
typedef struct _gcsDATABASE
{
    /* Pointer to next entry is hash list. */
    gcsDATABASE_PTR                     next;
    gctSIZE_T                           slot;

    /* Process ID. */
    gctUINT32                           processID;

    /* Open-Close ref count */
    gctPOINTER                          refs;

    /* Already mark for delete and cannot reenter */
    gctBOOL                             deleted;

    /* Sizes to query. */
    gcsDATABASE_COUNTERS                vidMem;
    gcsDATABASE_COUNTERS                nonPaged;
    gcsDATABASE_COUNTERS                contiguous;
    gcsDATABASE_COUNTERS                mapUserMemory;
    gcsDATABASE_COUNTERS                mapMemory;

    gcsDATABASE_COUNTERS                vidMemType[gcvVIDMEM_TYPE_COUNT];
    /* Counter for each video memory pool. */
    gcsDATABASE_COUNTERS                vidMemPool[gcvPOOL_NUMBER_OF_POOLS];
    gctPOINTER                          counterMutex;

    /* Idle time management. */
    gctUINT64                           lastIdle;
    gctUINT64                           idle;

    /* Pointer to database. */
    gcsDATABASE_RECORD_PTR              list[48];

    gctPOINTER                          handleDatabase;
    gctPOINTER                          handleDatabaseMutex;
}
gcsDATABASE;

typedef struct _gcsFDPRIVATE *          gcsFDPRIVATE_PTR;
typedef struct _gcsFDPRIVATE
{
    gctINT                              (* release) (gcsFDPRIVATE_PTR Private);
}
gcsFDPRIVATE;

typedef struct _gcsRECORDER * gckRECORDER;

typedef enum _gceMMU_INIT_MODE
{
    gcvMMU_INIT_FROM_REG,
    gcvMMU_INIT_FROM_CMD,
}
gceMMU_INIT_MODE;

typedef enum _gceEVENT_FAULT
{
    gcvEVENT_NO_FAULT,
    gcvEVENT_BUS_ERROR_FAULT,
}
gceEVENT_FAULT;

/* Create a process database that will contain all its allocations. */
gceSTATUS
gckKERNEL_CreateProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID
    );

/* Add a record to the process database. */
gceSTATUS
gckKERNEL_AddProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Size
    );

/* Remove a record to the process database. */
gceSTATUS
gckKERNEL_RemoveProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer
    );

/* Destroy the process database. */
gceSTATUS
gckKERNEL_DestroyProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID
    );

/* Find a record to the process database. */
gceSTATUS
gckKERNEL_FindProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 ThreadID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer,
    OUT gcsDATABASE_RECORD_PTR Record
    );

/* Query the process database. */
gceSTATUS
gckKERNEL_QueryProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctBOOL LastProcessID,
    IN gceDATABASE_TYPE Type,
    OUT gcuDATABASE_INFO * Info
    );

/* Dump the process database. */
gceSTATUS
gckKERNEL_DumpProcessDB(
    IN gckKERNEL Kernel
    );

/* Dump the video memory usage for process specified. */
gceSTATUS
gckKERNEL_DumpVidMemUsage(
    IN gckKERNEL Kernel,
    IN gctINT32 ProcessID
    );

gceSTATUS
gckKERNEL_FindDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctBOOL LastProcessID,
    OUT gcsDATABASE_PTR * Database
    );

gceSTATUS
gckKERNEL_FindHandleDatbase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    OUT gctPOINTER * HandleDatabase,
    OUT gctPOINTER * HandleDatabaseMutex
    );

gceSTATUS
gckMMU_GetPageEntry(
    IN gckMMU Mmu,
    IN gcePAGE_TYPE PageType,
    IN gctUINT32 Address,
    IN gctUINT32_PTR *PageTable
    );

gceSTATUS
gckMMU_SetupSRAM(
    IN gckMMU Mmu,
    IN gckHARDWARE Hardware,
    IN gckDEVICE Device
    );

gceSTATUS
gckMMU_SetupDynamicSpace(
    IN gckMMU Mmu
    );

void
gckMMU_DumpRecentFreedAddress(
    IN gckMMU Mmu
    );

gceSTATUS
gckKERNEL_CreateIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 Capacity,
    OUT gctPOINTER * Database
    );

gceSTATUS
gckKERNEL_DestroyIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctPOINTER Database
    );

gceSTATUS
gckKERNEL_AllocateIntegerId(
    IN gctPOINTER Database,
    IN gctPOINTER Pointer,
    OUT gctUINT32 * Id
    );

gceSTATUS
gckKERNEL_FreeIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id
    );

gceSTATUS
gckKERNEL_QueryIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id,
    OUT gctPOINTER * Pointer
    );

/* Pointer rename  */
gctUINT32
gckKERNEL_AllocateNameFromPointer(
    IN gckKERNEL Kernel,
    IN gctPOINTER Pointer
    );

gctPOINTER
gckKERNEL_QueryPointerFromName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    );

gceSTATUS
gckKERNEL_DeleteName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    );

/*******************************************************************************
********* Timer Management ****************************************************/
typedef struct _gcsTIMER *           gcsTIMER_PTR;
typedef struct _gcsTIMER
{
    /* Start and Stop time holders. */
    gctUINT64                           startTime;
    gctUINT64                           stopTime;
}
gcsTIMER;

/******************************************************************************\
********************************** Structures **********************************
\******************************************************************************/

/* gckDB object. */
struct _gckDB
{
    /* Database management. */
    gcsDATABASE_PTR             db[16];
    gctPOINTER                  dbMutex;
    gcsDATABASE_PTR             freeDatabase;
    gcsDATABASE_RECORD_PTR      freeRecord;
    gcsDATABASE_PTR             lastDatabase;
    gctUINT32                   lastProcessID;
    gctUINT64                   lastIdle;
    gctUINT64                   idleTime;
    gctUINT64                   lastSlowdown;
    gctUINT64                   lastSlowdownIdle;

    gctPOINTER                  nameDatabase;
    gctPOINTER                  nameDatabaseMutex;

    gctPOINTER                  pointerDatabase;

    gcsLISTHEAD                 videoMemList;
    gctPOINTER                  videoMemListMutex;
};

/* gckKERNEL object. */
struct _gckKERNEL
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckDEVICE object. */
    gckDEVICE                   device;

    /* Pointer to gckHARDWARE object. */
    gckHARDWARE                 hardware;

    /* Core */
    gceCORE                     core;
    gctUINT                     chipID;

    /* Brothers */
    gctPOINTER                  atomBroCoreMask;

    /* Main command module, event and context. */
    gckCOMMAND                  command;
    gckEVENT                    eventObj;
    gctPOINTER                  context;

    /* Async FE command and event modules. */
    gckCOMMAND                  asyncCommand;
    gckEVENT                    asyncEvent;

    /* Pointer to gckMMU object. */
    gckMMU                      mmu;

    /* Arom holding number of clients. */
    gctPOINTER                  atomClients;

#if VIVANTE_PROFILER
    gckPROFILER                 profiler;
#endif

#ifdef QNX_SINGLE_THREADED_DEBUGGING
    gctPOINTER                  debugMutex;
#endif

    /* Database management. */
    gckDB                       db;
    gctBOOL                     dbCreated;

    gctUINT64                   resetTimeStamp;

    /* Pointer to gckEVENT object. */
    gcsTIMER                    timers[8];
    gctUINT32                   timeOut;


#if gcdDVFS
    gckDVFS                     dvfs;
#endif

#if gcdLINUX_SYNC_FILE
    gctHANDLE                   timeline;
#endif

    /* Enable recovery. */
    gctBOOL                     recovery;

    /* Level of dump information after stuck. */
    gctUINT                     stuckDump;

#if gcdENABLE_TRUST_APPLICATION
    gctUINT32                   securityChannel;
#endif

    /* Timer to monitor GPU stuck. */
    gctPOINTER                  monitorTimer;

    /* Flag to quit monitor timer. */
    gctBOOL                     monitorTimerStop;

    /* Monitor states. */
    gctBOOL                     monitoring;
    gctUINT32                   lastCommitStamp;
    gctUINT32                   timer;
    gctUINT32                   restoreAddress;
    gctINT32                    restoreMask;

    gckVIDMEM_BLOCK             vidMemBlock;
    gctPOINTER                  vidMemBlockMutex;

    gctUINT32                   contiguousBaseAddress;
    gctUINT32                   externalBaseAddress;
    gctUINT32                   internalBaseAddress;
    gctUINT32                   exclusiveBaseAddress;

    /* External shared SRAM. */
    gctUINT32                   extSRAMBaseAddresses[gcvSRAM_EXT_COUNT];
    gctUINT32                   extSRAMIndex;

    /* Per core SRAM description. */
    gctUINT32                   sRAMIndex;
    gckVIDMEM                   sRAMVidMem[gcvSRAM_INTER_COUNT];
    gctPHYS_ADDR                sRAMPhysical[gcvSRAM_INTER_COUNT];
    gctUINT32                   sRAMBaseAddresses[gcvSRAM_INTER_COUNT];
    gctUINT32                   sRAMSizes[gcvSRAM_INTER_COUNT];
    gctBOOL                     sRAMPhysFaked[gcvSRAM_INTER_COUNT];
    gctUINT64                   sRAMLoopMode;

    gctUINT32                   timeoutPID;
    gctBOOL                     threadInitialized;

#if gcdENABLE_SW_PREEMPTION
    gctPOINTER                  priorityQueueMutex[gcdMAX_PRIORITY_QUEUE_NUM];
    gcsPRIORITY_QUEUE_PTR       priorityQueues[gcdMAX_PRIORITY_QUEUE_NUM];
    gctBOOL                     priorityDBCreated[gcdMAX_PRIORITY_QUEUE_NUM];
    gctSEMAPHORE                preemptSema;
    gcePREEMPTION_MODE          preemptionMode;
#endif
};

struct _FrequencyHistory
{
    gctUINT32                   frequency;
    gctUINT32                   count;
};

/* gckDVFS object. */
struct _gckDVFS
{
    gckOS                       os;
    gckHARDWARE                 hardware;
    gctPOINTER                  timer;
    gctUINT32                   pollingTime;
    gctBOOL                     stop;
    gctUINT32                   totalConfig;
    gctUINT32                   loads[8];
    gctUINT8                    currentScale;
    struct _FrequencyHistory    frequencyHistory[16];
};

typedef struct _gcsFENCE * gckFENCE;
typedef struct _gcsFENCE
{
    /* Pointer to required object. */
    gckKERNEL                   kernel;

    /* Fence location. */
    gckVIDMEM_NODE              videoMem;
    gctPOINTER                  logical;
    gctUINT32                   address;

    gcsLISTHEAD                 waitingList;
    gctPOINTER                  mutex;
}
gcsFENCE;

/* A sync point attached to fence. */
typedef struct _gcsFENCE_SYNC * gckFENCE_SYNC;
typedef struct _gcsFENCE_SYNC
{
    /* Stamp of commit access this node. */
    gctUINT64                   commitStamp;

    /* Attach to waiting list. */
    gcsLISTHEAD                 head;

    gctPOINTER                  signal;

    gctBOOL                     inList;
}
gcsFENCE_SYNC;

typedef struct _gcsCOMMAND_QUEUE
{
    gctSIGNAL               signal;
    gckVIDMEM_NODE          videoMem;
    gctPOINTER              logical;
    gctUINT32               address;
}
gcsCOMMAND_QUEUE;

/* gckCOMMAND object. */
struct _gckCOMMAND
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to required object. */
    gckKERNEL                   kernel;
    gckOS                       os;

    gceHW_FE_TYPE               feType;

    /* Number of bytes per page. */
    gctUINT32                   pageSize;

    /* Current pipe select. */
    gcePIPE_SELECT              pipeSelect;

    /* Command queue running flag. */
    gctBOOL                     running;

    /* Idle flag and commit stamp. */
    gctBOOL                     idle;
    gctUINT64                   commitStamp;

    /* Command queue mutex. */
    gctPOINTER                  mutexQueue;

    /* Context switching mutex. */
    gctPOINTER                  mutexContext;

    /* Context sequence mutex. */
    gctPOINTER                  mutexContextSeq;

    /* Command queue power semaphore. */
    gctPOINTER                  powerSemaphore;

    /* Command queues. */
    gcsCOMMAND_QUEUE            queues[gcdCOMMAND_QUEUES];

    /* Current queue. */
    gckVIDMEM_NODE              videoMem;
    gctPOINTER                  logical;
    gctUINT32                   address;

    gctUINT32                   offset;
    gctINT                      index;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gctUINT                     wrapCount;
#endif

    /* The command queue is new. */
    gctBOOL                     newQueue;

    /* Context management. */
    gckCONTEXT                  currContext;
    gctPOINTER                  stateMap;

    /* Pointer to last WAIT command. */
    /* Wait-Link FE only. */
    struct
    {
        gckVIDMEM_NODE          videoMem;
        gctUINT32               offset;
        gctPOINTER              logical;
        gctUINT32               address;
        gctUINT32               size;
    }
    waitPos;

    /* MCFE. */
    gctUINT32                   totalSemaId;

    /*
     * freeSemaId   ------>  [pendingSema]
     * ^                    freePos -->+ +
     * |                    ^          | |
     * |                    |          | |
     * |                    nextPos ---+ |
     * |                                 |
     * |                                 |
     * |                                 v
     * nextSemaId <----------------------+
     *
     * Terminology:
     * 'freeSemaId': the top (final) free sema id can be used, signaled already.
     * 'nextSemaId': the next sema id to be used.
     * 'pendingSema': the used semaphores which are tracked in the ring.
     *
     * 3 Id sections:
     * Id(s) between 'nextSemaId' and 'freeSemaId':
     * Available for immediate use.
     *
     * Id(s) between 'freeSemaId' (exclusive) and 'pendingSema':
     * To be signed, will be available to use later.
     *
     * Id(s) between 'pendingSema' and 'nextSemaId':
     * The semaphores just used, not tracking in pendingSema ring.
     *
     * Conditions:
     * 'nextSemaId' = 'freeSemaId':
     * Using the final free semaphore, means the ring is full of used ones.
     *
     * 'freeSemaId' + 1 = 'nextSemaId':
     * Can use 'freeSema' + 1 to 'nextSema'(loop back), means empty ring,
     * ie, no used one.
     */

    /* MCFE semaphore id tracking ring. */
    gctUINT32                   nextSemaId;
    gctUINT32                   freeSemaId;

    gctUINT32                   semaMinThreshhold;

    /* pending semaphore id tracking ring. */
    struct
    {
        gctUINT32               semaId;
        gctSIGNAL               signal;
    }
    pendingSema[8];

    gctUINT32                   nextPendingPos;
    gctUINT32                   freePendingPos;

    /* semaphore id (array index) to handle value map. */
    gctUINT32 *                 semaHandleMap;

    /*
     * Dirty channels, ie channels ever submitted commands.
     * Need sync to (send semaphore to) system channel.
     */
    gctUINT64                   dirtyChannel[2];

    /*
     * Sync channels, need sync from (wait semaphore from) the system
     * channel before its own jobs.
     */
    gctUINT64                   syncChannel[2];

    /* Command buffer alignment. */
    gctUINT32                   alignment;

    /* Commit counter. */
    gctPOINTER                  atomCommit;

    /* Kernel process ID. */
    gctUINT32                   kernelProcessID;

#if gcdRECORD_COMMAND
    gckRECORDER                 recorder;
#endif

    gckFENCE                    fence;

    gctBOOL                     dummyDraw;
};

typedef struct _gcsEVENT *      gcsEVENT_PTR;

/* Structure holding one event to be processed. */
typedef struct _gcsEVENT
{
    /* Pointer to next event in queue. */
    gcsEVENT_PTR                next;

    /* Event information. */
    gcsHAL_INTERFACE            info;

    /* Process ID owning the event. */
    gctUINT32                   processID;

#ifdef __QNXNTO__
    /* Kernel. */
    gckKERNEL                   kernel;
#endif

    gctBOOL                     fromKernel;
}
gcsEVENT;

/* Structure holding a list of events to be processed by an interrupt. */
typedef struct _gcsEVENT_QUEUE * gcsEVENT_QUEUE_PTR;
typedef struct _gcsEVENT_QUEUE
{
    /* Time stamp. */
    gctUINT64                   stamp;

    /* Source of the event. */
    gceKERNEL_WHERE             source;

    /* Pointer to head of event queue. */
    gcsEVENT_PTR                head;

    /* Pointer to tail of event queue. */
    gcsEVENT_PTR                tail;

    /* Next list of events. */
    gcsEVENT_QUEUE_PTR          next;

    /* Current commit stamp. */
    gctUINT64                   commitStamp;
}
gcsEVENT_QUEUE;

/*
    gcdREPO_LIST_COUNT defines the maximum number of event queues with different
    hardware module sources that may coexist at the same time. Only two sources
    are supported - gcvKERNEL_COMMAND and gcvKERNEL_PIXEL. gcvKERNEL_COMMAND
    source is used only for managing the kernel command queue and is only issued
    when the current command queue gets full. Since we commit event queues every
    time we commit command buffers, in the worst case we can have up to three
    pending event queues:
        - gcvKERNEL_PIXEL
        - gcvKERNEL_COMMAND (queue overflow)
        - gcvKERNEL_PIXEL
*/
#define gcdREPO_LIST_COUNT      3


/* gckEVENT object. */
struct _gckEVENT
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to required objects. */
    gckOS                       os;
    gckKERNEL                   kernel;

    /* Pointer to COMMAND object, either one of the 3. */
    gckCOMMAND                  command;

    /* Submit function pointer, different for different command module. */
    gceSTATUS                (* submitEvent)(gckEVENT, gctBOOL, gctBOOL);

    /* Time stamp. */
    gctUINT64                   stamp;
    gctUINT32                   lastCommitStamp;

    /* Queue mutex. */
    gctPOINTER                  eventQueueMutex;

    /* Array of event queues. */
    gcsEVENT_QUEUE              queues[29];
    gctINT32                    freeQueueCount;
    gctUINT8                    lastID;

    /* Pending events. */
    gctPOINTER                  pending;

    /* List of free event structures and its mutex. */
    gcsEVENT_PTR                freeEventList;
    gctSIZE_T                   freeEventCount;
    gctPOINTER                  freeEventMutex;

    /* Event queues. */
    gcsEVENT_QUEUE_PTR          queueHead;
    gcsEVENT_QUEUE_PTR          queueTail;
    gcsEVENT_QUEUE_PTR          freeList;
    gcsEVENT_QUEUE              repoList[gcdREPO_LIST_COUNT];
    gctPOINTER                  eventListMutex;

    gctPOINTER                  submitTimer;

#if gcdINTERRUPT_STATISTIC
    gctPOINTER                  interruptCount;
#endif

    gctINT                      notifyState;
};

/* Construct a new gckEVENT object. */
gceSTATUS
gckEVENT_Construct(
    IN gckKERNEL Kernel,
    IN gckCOMMAND Command,
    OUT gckEVENT * Event
    );

/* Destroy an gckEVENT object. */
gceSTATUS
gckEVENT_Destroy(
    IN gckEVENT Event
    );

/* Reserve the next available hardware event. */
gceSTATUS
gckEVENT_GetEvent(
    IN gckEVENT Event,
    IN gctBOOL Wait,
    OUT gctUINT8 * EventID,
    IN gceKERNEL_WHERE Source
   );

/* Add a new event to the list of events. */
gceSTATUS
gckEVENT_AddListEx(
    IN gckEVENT Event,
    IN gcsHAL_INTERFACE_PTR Interface,
    IN gceKERNEL_WHERE FromWhere,
    IN gctBOOL AllocateAllowed,
    IN gctBOOL FromKernel,
    IN gctUINT32 ProcessID
    );

/* Add a new event to the list of events. */
gceSTATUS
gckEVENT_AddList(
    IN gckEVENT Event,
    IN gcsHAL_INTERFACE_PTR Interface,
    IN gceKERNEL_WHERE FromWhere,
    IN gctBOOL AllocateAllowed,
    IN gctBOOL FromKernel
    );

/* Schedule a FreeVideoMemory event. */
gceSTATUS
gckEVENT_FreeVideoMemory(
    IN gckEVENT Event,
    IN gcuVIDMEM_NODE_PTR VideoMemory,
    IN gceKERNEL_WHERE FromWhere
    );

/* Schedule a signal event. */
gceSTATUS
gckEVENT_Signal(
    IN gckEVENT Event,
    IN gctSIGNAL Signal,
    IN gceKERNEL_WHERE FromWhere
    );

/* Schedule an Unlock event. */
gceSTATUS
gckEVENT_Unlock(
    IN gckEVENT Event,
    IN gceKERNEL_WHERE FromWhere,
    IN gctPOINTER Node
    );

gceSTATUS
gckEVENT_CommitDone(
    IN gckEVENT Event,
    IN gceKERNEL_WHERE FromWhere,
    IN gckCONTEXT Context
    );

gceSTATUS
gckEVENT_Submit(
    IN gckEVENT Event,
    IN gctBOOL Wait,
    IN gctBOOL FromPower,
    IN gctBOOL BroadcastCommit
    );

gceSTATUS
gckEVENT_Commit(
    IN gckEVENT Event,
    IN gcsQUEUE_PTR Queue,
    IN gctBOOL Forced
    );

/* Event callback routine. */
gceSTATUS
gckEVENT_Notify(
    IN gckEVENT Event,
    IN gctUINT32 IDs,
    OUT gceEVENT_FAULT *Fault
    );

/* Event callback routine. */
gceSTATUS
gckEVENT_Interrupt(
    IN gckEVENT Event,
    IN gctUINT32 IDs
    );

gceSTATUS
gckEVENT_Dump(
    IN gckEVENT Event
    );

/* Free all events belonging to a process. */
gceSTATUS
gckEVENT_FreeProcess(
    IN gckEVENT Event,
    IN gctUINT32 ProcessID
    );

/* gcuVIDMEM_NODE structure. */
typedef union _gcuVIDMEM_NODE
{
    /* Allocated from gckVIDMEM. */
    struct _gcsVIDMEM_NODE_VIDMEM
    {
        /* Owner of this node. */
        gckVIDMEM               parent;

        /* Dual-linked list of nodes. */
        gcuVIDMEM_NODE_PTR      next;
        gcuVIDMEM_NODE_PTR      prev;

        /* Dual linked list of free nodes. */
        gcuVIDMEM_NODE_PTR      nextFree;
        gcuVIDMEM_NODE_PTR      prevFree;

        /* Information for this node. */
        gctSIZE_T               offset;

        gctUINT32               address;
        gctSIZE_T               bytes;
        gctUINT32               alignment;

        /* Client virtual address. */
        gctPOINTER              logical;

        /* Process ID owning this memory. */
        gctUINT32               processID;

        /* Locked counter. */
        gctINT32                locked;

        /* Memory pool. */
        gcePOOL                 pool;

        /* Kernel virtual address. */
        gctPOINTER              kvaddr;

        /* mdl record pointer. */
        gctPHYS_ADDR            physical;

    }
    VidMem;

    /* Allocated from gckOS. */
    struct _gcsVIDMEM_NODE_VIRTUAL
    {
        /* Pointer to gckKERNEL object. */
        gckKERNEL               kernel;

        /* Information for this node. */
        /* Contiguously allocated? */
        gctBOOL                 contiguous;
        /* mdl record pointer... a kmalloc address. Process agnostic. */
        gctPHYS_ADDR            physical;
        gctSIZE_T               bytes;

        /* do_mmap_pgoff address... mapped per-process. */
        gctPOINTER              logical;

        /* Kernel virtual address. */
        gctPOINTER              kvaddr;


        /* Customer private handle */
        gctUINT32               gid;

        /* Page table information. */
        /* Used only when node is not contiguous */
        gctSIZE_T               pageCount;

        /* Used only when node is not contiguous */
        gctPOINTER              pageTables[gcvHARDWARE_NUM_TYPES];
        /* Actual physical address */
        gctUINT32               addresses[gcvHARDWARE_NUM_TYPES];

        /* Locked counter. */
        gctINT32                lockeds[gcvHARDWARE_NUM_TYPES];

        /* MMU page size type */
        gcePAGE_TYPE            pageType;

        gceVIDMEM_TYPE          type;

        /* Secure GPU virtual address. */
        gctBOOL                 secure;

        gctBOOL                 onFault;
    }
    Virtual;

    struct _gcsVIDMEM_NODE_VIRTUAL_CHUNK
    {
        /* Owner of this chunk */
        gckVIDMEM_BLOCK         parent;

        /* Pointer to gckKERNEL object. */
        gckKERNEL               kernel;

        /* Dual-linked list of chunk. */
        gcuVIDMEM_NODE_PTR      next;
        gcuVIDMEM_NODE_PTR      prev;

        /* Dual linked list of free chunk. */
        gcuVIDMEM_NODE_PTR      nextFree;
        gcuVIDMEM_NODE_PTR      prevFree;

        /* Information for this chunk. */
        gctSIZE_T               offset;
        gctUINT32               addresses[gcvHARDWARE_NUM_TYPES];
        gctINT32                lockeds[gcvHARDWARE_NUM_TYPES];
        gctSIZE_T               bytes;

        /* Mapped user logical */
        gctPOINTER              logical;

        /* Kernel virtual address. */
        gctPOINTER              kvaddr;

        /* Locked counter. */
    }
    VirtualChunk;

}
gcuVIDMEM_NODE;

/* gckVIDMEM object. */
struct _gckVIDMEM
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* mdl record pointer... a kmalloc address. Process agnostic. */
    gctPHYS_ADDR                physical;

    /* Information for this video memory heap. */
    gctPHYS_ADDR_T              physicalBase;
    gctSIZE_T                   bytes;
    gctSIZE_T                   freeBytes;
    gctSIZE_T                   minFreeBytes;

    /* caps inherit from its allocator, ~0u if allocator was not applicable. */
    gctUINT32                   capability;

    /* Mapping for each type of surface. */
    gctINT                      mapping[gcvVIDMEM_TYPE_COUNT];

    /* Sentinel nodes for up to 8 banks. */
    gcuVIDMEM_NODE              sentinel[8];

    /* Allocation threshold. */
    gctSIZE_T                   threshold;

    /* The heap mutex. */
    gctPOINTER                  mutex;
};

/* gckVIDMEM_BLOCK object. */
typedef struct _gcsVIDMEM_BLOCK
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* linked list of nodes. */
    gckVIDMEM_BLOCK             next;

    /* Contiguously allocated? */
    gctBOOL                     contiguous;

    /* Customer private handle */
    gctUINT32                   gid;

    /* mdl record pointer... a kmalloc address. Process agnostic. */
    gctPHYS_ADDR                physical;

    /* Information for this video memory virtual block. */
    gctSIZE_T                   bytes;
    gctSIZE_T                   freeBytes;

    /* 1M page count. */
    gctUINT32                   pageCount;
    gctUINT32                   fixedPageCount;

    /* Gpu virtual base of this video memory heap. */
    gctUINT32                   addresses[gcvHARDWARE_NUM_TYPES];
    gctPOINTER                  pageTables[gcvHARDWARE_NUM_TYPES];

    gceVIDMEM_TYPE              type;

    /* Virtual chunk. */
    gcuVIDMEM_NODE              node;

    gctPOINTER                  mutex;

    gctBOOL                     secure;
    gctBOOL                     onFault;
}
gcsVIDMEM_BLOCK;

typedef struct _gcsVIDMEM_NODE
{
    _VIV_VIDMEM_METADATA        metadata;

    /* Pointer to gcuVIDMEM_NODE. */
    gcuVIDMEM_NODE_PTR          node;

    /* Pointer to gcuVIDMEM_NODE. */
    gcuVIDMEM_NODE_PTR          transitNode;

    /* Pointer to gckKERNEL object. */
    gckKERNEL                   kernel;

    /* Mutex to protect node. */
    gctPOINTER                  mutex;

    /* Reference count. */
    gctPOINTER                  reference;

    /* Name for client to import. */
    gctUINT32                   name;

    /* Link in _gckDB::videoMemList. */
    gcsLISTHEAD                 link;

    /* dma_buf */
    gctPOINTER                  dmabuf;

    /* Video memory allocation type. */
    gceVIDMEM_TYPE              type;

    /* Pool from which node is allocated. */
    gcePOOL                     pool;

    gcsFENCE_SYNC               sync[gcvENGINE_GPU_ENGINE_COUNT];

    /* For DRM usage */
    gctUINT64                   timeStamp;
    gckVIDMEM_NODE              tsNode;
    gctUINT32                   tilingMode;
    gctUINT32                   tsMode;
    gctUINT32                   tsCacheMode;
    gctUINT64                   clearValue;

#if gcdCAPTURE_ONLY_MODE
    gctSIZE_T                   captureSize;
    gctPOINTER                  captureLogical;
#endif
}
gcsVIDMEM_NODE;

typedef struct _gcsVIDMEM_HANDLE * gckVIDMEM_HANDLE;
typedef struct _gcsVIDMEM_HANDLE
{
    /* Pointer to gckVIDMEM_NODE. */
    gckVIDMEM_NODE              node;

    /* Handle for current process. */
    gctUINT32                   handle;

    /* Reference count for this handle. */
    gctPOINTER                  reference;
}
gcsVIDMEM_HANDLE;

typedef struct _gcsSHBUF * gcsSHBUF_PTR;
typedef struct _gcsSHBUF
{
    /* ID. */
    gctUINT32                   id;

    /* Reference count. */
    gctPOINTER                  reference;

    /* Data size. */
    gctUINT32                   size;

    /* Data. */
    gctPOINTER                  data;
}
gcsSHBUF;

typedef struct _gcsCORE_INFO
{
    gceHARDWARE_TYPE            type;
    gceCORE                     core;
    gckKERNEL                   kernel;
    gctUINT                     chipID;
}
gcsCORE_INFO;

typedef struct _gcsCORE_LIST
{
    gckKERNEL                   kernels[gcvCORE_COUNT];
    gctUINT32                   num;
}
gcsCORE_LIST;

/* A gckDEVICE is a group of cores (gckKERNEL in software). */
typedef struct _gcsDEVICE
{
    gcsCORE_INFO                coreInfoArray[gcvCORE_COUNT];
    gctUINT32                   coreNum;
    gcsCORE_LIST                map[gcvHARDWARE_NUM_TYPES];
    gceHARDWARE_TYPE            defaultHwType;

    gckOS                       os;

    /* Process resource database. */
    gckDB                       database;

    /* Same hardware type shares one MMU. */
    gckMMU                      mmus[gcvHARDWARE_NUM_TYPES];

    /* Physical address of internal SRAMs. */
    gctUINT64                   sRAMBases[gcvCORE_COUNT][gcvSRAM_INTER_COUNT];
    /* Internal SRAMs' size. */
    gctUINT32                   sRAMSizes[gcvCORE_COUNT][gcvSRAM_INTER_COUNT];
    /* GPU/VIP virtual address of internal SRAMs. */
    gctUINT32                   sRAMBaseAddresses[gcvCORE_COUNT][gcvSRAM_INTER_COUNT];
    gctBOOL                     sRAMPhysFaked[gcvCORE_COUNT][gcvSRAM_INTER_COUNT];

    /* CPU physical address of external SRAMs. */
    gctUINT64                   extSRAMBases[gcvSRAM_EXT_COUNT];
    /* GPU physical address of external SRAMs. */
    gctUINT64                   extSRAMGPUBases[gcvSRAM_EXT_COUNT];
    /* External SRAMs' size. */
    gctUINT32                   extSRAMSizes[gcvSRAM_EXT_COUNT];
    /* GPU/VIP virtual address of external SRAMs. */
    gctUINT32                   extSRAMBaseAddresses[gcvSRAM_EXT_COUNT];
    /* MDL. */
    gctPHYS_ADDR                extSRAMPhysical[gcvSRAM_EXT_COUNT];
    /* IntegerId. */
    gctUINT32                   extSRAMGPUPhysNames[gcvSRAM_EXT_COUNT];

    /* Show SRAM mapping info or not. */
    gctUINT                     showSRAMMapInfo;

    /* Mutex to make sure stuck dump for multiple cores doesn't interleave. */
    gctPOINTER                  stuckDumpMutex;

    /* Mutex for multi-core combine mode command submission */
    gctPOINTER                  commitMutex;

    /* Mutex for per-device power management. */
    gctPOINTER                  powerMutex;

#if gcdENABLE_SW_PREEMPTION
    gctPOINTER                  atomPriorityID;
#endif
}
gcsDEVICE;

/* video memory pool functions. */
/* Construct a new gckVIDMEM object. */
gceSTATUS
gckVIDMEM_Construct(
    IN gckOS Os,
    IN gctPHYS_ADDR_T PhysicalBase,
    IN gctSIZE_T Bytes,
    IN gctSIZE_T Threshold,
    IN gctSIZE_T Banking,
    OUT gckVIDMEM * Memory
    );

/* Destroy an gckVDIMEM object. */
gceSTATUS
gckVIDMEM_Destroy(
    IN gckVIDMEM Memory
    );


gceSTATUS
gckVIDMEM_HANDLE_Allocate(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node,
    OUT gctUINT32 * Handle
    );

gceSTATUS
gckVIDMEM_HANDLE_Reference(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    );

gceSTATUS
gckVIDMEM_HANDLE_Dereference(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    );

gceSTATUS
gckVIDMEM_HANDLE_Lookup(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle,
    OUT gckVIDMEM_NODE * Node
    );

gceSTATUS
gckVIDMEM_HANDLE_Lookup2(
    IN gckKERNEL Kernel,
    IN gcsDATABASE_PTR Database,
    IN gctUINT32 Handle,
    OUT gckVIDMEM_NODE * Node
    );

/* video memory node functions. */
gceSTATUS
gckVIDMEM_NODE_AllocateLinear(
    IN gckKERNEL Kernel,
    IN gckVIDMEM VideoMemory,
    IN gcePOOL Pool,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Flag,
    IN gctUINT32 Alignment,
    IN gctBOOL Specified,
    IN OUT gctSIZE_T * Bytes,
    OUT gckVIDMEM_NODE * NodeObject
    );

gceSTATUS
gckVIDMEM_NODE_AllocateVirtual(
    IN gckKERNEL Kernel,
    IN gcePOOL Pool,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    OUT gckVIDMEM_NODE * NodeObject
    );

gceSTATUS
gckVIDMEM_NODE_AllocateVirtualChunk(
    IN gckKERNEL Kernel,
    IN gcePOOL Pool,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    OUT gckVIDMEM_NODE * NodeObject
    );

gceSTATUS
gckVIDMEM_NODE_Reference(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    );

gceSTATUS
gckVIDMEM_NODE_Dereference(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    );

gceSTATUS
gckVIDMEM_NODE_GetReference(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctINT32 * ReferenceCount
    );

gceSTATUS
gckVIDMEM_NODE_Lock(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctUINT32 *Address
    );

gceSTATUS
gckVIDMEM_NODE_Unlock(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctUINT32 ProcessID,
    IN OUT gctBOOL * Asynchroneous
    );

gceSTATUS
gckVIDMEM_NODE_CleanCache(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    );

gceSTATUS
gckVIDMEM_NODE_InvalidateCache(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    );

gceSTATUS
gckVIDMEM_NODE_GetLockCount(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctINT32 * LockCount
    );

gceSTATUS
gckVIDMEM_NODE_LockCPU(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctBOOL Cacheable,
    IN gctBOOL FromUser,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckVIDMEM_NODE_UnlockCPU(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctUINT32 ProcessID,
    IN gctBOOL FromUser,
    IN gctBOOL Defer
    );

gceSTATUS
gckVIDMEM_NODE_GetPhysical(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * PhysicalAddress
    );

gceSTATUS
gckVIDMEM_NODE_GetGid(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctUINT32 * Gid
    );

gceSTATUS
gckVIDMEM_NODE_GetSize(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctSIZE_T * Size
    );

gceSTATUS
gckVIDMEM_NODE_GetType(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gceVIDMEM_TYPE * Type,
    OUT gcePOOL * Pool
    );

gceSTATUS
gckVIDMEM_NODE_Export(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    IN gctINT32 Flags,
    OUT gctPOINTER *DmaBuf,
    OUT gctINT32 *FD
    );

gceSTATUS
gckVIDMEM_NODE_Name(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctUINT32 * Name
    );

gceSTATUS
gckVIDMEM_NODE_Import(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name,
    OUT gckVIDMEM_NODE * NodeObject
    );

gceSTATUS
gckVIDMEM_NODE_GetFd(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctINT * Fd
    );

gceSTATUS
gckVIDMEM_NODE_WrapUserMemory(
    IN gckKERNEL Kernel,
    IN gcsUSER_MEMORY_DESC_PTR Desc,
    IN gceVIDMEM_TYPE Type,
    OUT gckVIDMEM_NODE * NodeObject,
    OUT gctUINT64 * Bytes
    );

gceSTATUS
gckVIDMEM_NODE_SetCommitStamp(
    IN gckKERNEL Kernel,
    IN gceENGINE Engine,
    IN gckVIDMEM_NODE NodeObject,
    IN gctUINT64 CommitStamp
    );

gceSTATUS
gckVIDMEM_NODE_GetCommitStamp(
    IN gckKERNEL Kernel,
    IN gceENGINE Engine,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctUINT64_PTR CommitStamp
    );

gceSTATUS
gckVIDMEM_NODE_Find(
    IN gckKERNEL Kernel,
    IN gctUINT32 Address,
    OUT gckVIDMEM_NODE * NodeObject,
    OUT gctUINT32 * Offset
    );

gceSTATUS
gckVIDMEM_NODE_IsContiguous(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE NodeObject,
    OUT gctBOOL * Contiguous
    );

typedef struct _gcsADDRESS_AREA * gcsADDRESS_AREA_PTR;
typedef struct _gcsADDRESS_AREA
{
    /* Page table / STLB table information. */
    gctSIZE_T                   stlbSize;
    gckVIDMEM_NODE              stlbVideoMem;
    gctUINT32_PTR               stlbLogical;
    gctUINT32                   stlbEntries;
    /* stlb physical address. */
    gctPHYS_ADDR_T              stlbPhysical;

    /* Free entries. */
    gctUINT32                   heapList;
    gctBOOL                     freeNodes;

    gceAREA_TYPE                areaType;

    gctUINT32                   mappingStart;
    gctUINT32                   mappingEnd;

    gctUINT32_PTR               mapLogical;
}
gcsADDRESS_AREA;

/* gckMMU object. */
struct _gckMMU
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckHARDWARE object. */
    gckHARDWARE                 hardware;

    /* The page table mutex. */
    gctPOINTER                  pageTableMutex;

    /* Master TLB information. */
    gctSIZE_T                   mtlbSize;
    gckVIDMEM_NODE              mtlbVideoMem;
    gctUINT32_PTR               mtlbLogical;
    gctUINT32                   mtlbEntries;
    /* mtlb physical address. */
    gctPHYS_ADDR_T              mtlbPhysical;

    /* memory pool used for page table */
    gcePOOL                     pool;

    gctPOINTER                  staticSTLB;
    gctBOOL                     enabled;

    gctSIZE_T                   safePageSize;
    gckVIDMEM_NODE              safePageVideoMem;
    gctPOINTER                  safePageLogical;
    gctUINT32                   safeAddress;
    /* Safe page physical address. */
    gctPHYS_ADDR_T              safePagePhysical;

    /* GPU physical address flat mapping area. */
    gctUINT32                   gpuPhysicalRangeCount;
    gcsFLAT_MAPPING_RANGE       gpuPhysicalRanges[gcdMAX_FLAT_MAPPING_COUNT];

    /* GPU virtual address flat mapping area*/
    gctUINT32                   gpuAddressRangeCount;
    gcsFLAT_MAPPING_RANGE       gpuAddressRanges[gcdMAX_FLAT_MAPPING_COUNT];

    /* List of hardware which uses this MMU. */
    gcsLISTHEAD                 hardwareList;

    struct _gckQUEUE            recentFreedAddresses;

    gcsADDRESS_AREA             dynamicArea1M;
    gcsADDRESS_AREA             dynamicArea4K;
    gcsADDRESS_AREA             secureArea;

    gctBOOL                     dynamicAreaSetuped;

    gctBOOL                     sRAMMapped;

    gctUINT32                   contiguousBaseAddress;
    gctUINT32                   externalBaseAddress;
    gctUINT32                   internalBaseAddress;
    gctUINT32                   exclusiveBaseAddress;

    gceMMU_INIT_MODE            initMode;
    gctBOOL                     pageTableOver4G;

    gcePAGE_TYPE                flatMappingMode;

    /* If the stlb is allocated when page size is 16M . */
    gctBOOL                     stlbAllocated[gcdMMU_STLB_16M_ENTRY_NUM];
};


gceSTATUS
gckOS_CreateKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckOS_DestroyKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    );

gceSTATUS
gckOS_GetFd(
    IN gctSTRING Name,
    IN gcsFDPRIVATE_PTR Private,
    OUT gctINT *Fd
    );

/*******************************************************************************
**
**  gckOS_ReadMappedPointer
**
**  Read pointer mapped from user pointer which returned by gckOS_MapUserPointer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Address
**          Pointer returned by gckOS_MapUserPointer.
**
**      gctUINT32_PTR Data
**          Pointer to hold 32 bits data.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_ReadMappedPointer(
    IN gckOS Os,
    IN gctPOINTER Address,
    IN gctUINT32_PTR Data
    );

gceSTATUS
gckKERNEL_AttachProcess(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach
    );

gceSTATUS
gckKERNEL_AttachProcessEx(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach,
    IN gctUINT32 PID
    );

gceSTATUS
gckKERNEL_AllocateVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 Alignment,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 Flag,
    IN OUT gctSIZE_T * Bytes,
    IN OUT gcePOOL * Pool,
    OUT gckVIDMEM_NODE * NodeObject
    );

gceSTATUS
gckHARDWARE_QchannelPowerControl(
    IN gckHARDWARE Hardware,
    IN gctBOOL ClockState,
    IN gctBOOL PowerState
    );

gceSTATUS
gckHARDWARE_QchannelBypass(
    IN gckHARDWARE Hardware,
    IN gctBOOL Enable
    );

gceSTATUS
gckHARDWARE_QueryIdle(
    IN gckHARDWARE Hardware,
    OUT gctBOOL_PTR IsIdle
    );

gceSTATUS
gckHARDWARE_WaitFence(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT64 FenceData,
    IN gctUINT32 FenceAddress,
    OUT gctUINT32 *Bytes
    );

gceSTATUS
gckHARDWARE_UpdateContextID(
    IN gckHARDWARE Hardware
    );

gceSTATUS
gckHARDWARE_QueryMcfe(
    IN gckHARDWARE Hardware,
    OUT const gceMCFE_CHANNEL_TYPE * Channels[],
    OUT gctUINT32 * Count
    );


#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_SecurityOpen(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPU,
    OUT gctUINT32 *Channel
    );

/*
** Close a security service channel
*/
gceSTATUS
gckKERNEL_SecurityClose(
    IN gctUINT32 Channel
    );

/*
** Security service interface.
*/
gceSTATUS
gckKERNEL_SecurityCallService(
    IN gctUINT32 Channel,
    IN OUT gcsTA_INTERFACE * Interface
    );

gceSTATUS
gckKERNEL_SecurityStartCommand(
    IN gckKERNEL Kernel,
    IN gctUINT32 Address,
    IN gctUINT32 Bytes
    );

gceSTATUS
gckKERNEL_SecurityMapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 *PhysicalArray,
    IN gctPHYS_ADDR_T Physical,
    IN gctUINT32 PageCount,
    OUT gctUINT32 * GPUAddress
    );

gceSTATUS
gckKERNEL_SecurityUnmapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    IN gctUINT32 PageCount
    );

gceSTATUS
gckKERNEL_SecurityDumpMMUException(
    IN gckKERNEL Kernel
    );

gceSTATUS
gckKERNEL_ReadMMUException(
    IN gckKERNEL Kernel,
    IN gctUINT32_PTR MMUStatus,
    IN gctUINT32_PTR MMUException
    );

gceSTATUS
gckKERNEL_HandleMMUException(
    IN gckKERNEL Kernel,
    IN gctUINT32 MMUStatus,
    IN gctPHYS_ADDR_T Physical,
    IN gctUINT32 GPUAddres
    );
#endif

gceSTATUS
gckKERNEL_CreateShBuffer(
    IN gckKERNEL Kernel,
    IN gctUINT32 Size,
    OUT gctSHBUF * ShBuf
    );

gceSTATUS
gckKERNEL_DestroyShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    );

gceSTATUS
gckKERNEL_MapShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    );

gceSTATUS
gckKERNEL_WriteShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount
    );

gceSTATUS
gckKERNEL_ReadShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount,
    OUT gctUINT32 * BytesRead
    );

gceSTATUS
gckKERNEL_GetHardwareType(
    IN gckKERNEL Kernel,
    OUT gceHARDWARE_TYPE *Type
    );

#if gcdENABLE_MP_SWITCH
gceSTATUS
gckKERNEL_DetectMpModeSwitch(
    IN gckKERNEL Kernel,
    IN gceMULTI_PROCESSOR_MODE Mode,
    OUT gctUINT32 *SwitchMpMode
    );
#endif

/******************************************************************************\
******************************* gckCONTEXT Object *******************************
\******************************************************************************/

gceSTATUS
gckCONTEXT_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT32 ProcessID,
    OUT gckCONTEXT * Context
    );

gceSTATUS
gckCONTEXT_Destroy(
    IN gckCONTEXT Context
    );

gceSTATUS
gckCONTEXT_Update(
    IN gckCONTEXT Context,
    IN gctUINT32 ProcessID,
    IN gcsSTATE_DELTA_PTR StateDelta
    );

gceSTATUS
gckCONTEXT_MapBuffer(
    IN gckCONTEXT Context,
    OUT gctUINT64 *Logicals,
    OUT gctUINT32 *Bytes
    );

void
gckQUEUE_Enqueue(
    IN gckQUEUE LinkQueue,
    IN gcuQUEUEDATA *Data
    );

void
gckQUEUE_GetData(
    IN gckQUEUE LinkQueue,
    IN gctUINT32 Index,
    OUT gcuQUEUEDATA ** Data
    );

gceSTATUS
gckQUEUE_Allocate(
    IN gckOS Os,
    IN gckQUEUE Queue,
    IN gctUINT32 Size
    );

gceSTATUS
gckQUEUE_Free(
    IN gckOS Os,
    IN gckQUEUE Queue
    );

/******************************************************************************\
****************************** gckRECORDER Object ******************************
\******************************************************************************/
gceSTATUS
gckRECORDER_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    OUT gckRECORDER * Recorder
    );

gceSTATUS
gckRECORDER_Destory(
    IN gckOS Os,
    IN gckRECORDER Recorder
    );

void
gckRECORDER_AdvanceIndex(
    gckRECORDER Recorder,
    gctUINT64   CommitStamp
    );

void
gckRECORDER_Record(
    gckRECORDER Recorder,
    gctUINT8_PTR CommandBuffer,
    gctUINT32 CommandBytes,
    gctUINT8_PTR ContextBuffer,
    gctUINT32 ContextBytes
    );

void
gckRECORDER_Dump(
    gckRECORDER Recorder
    );

gceSTATUS
gckRECORDER_UpdateMirror(
    gckRECORDER Recorder,
    gctUINT32 State,
    gctUINT32 Data
    );

/******************************************************************************\
****************************** gckCOMMAND Object *******************************
\******************************************************************************/

/* Construct a new gckCOMMAND object. */
gceSTATUS
gckCOMMAND_Construct(
    IN gckKERNEL Kernel,
    IN gceHW_FE_TYPE FeType,
    OUT gckCOMMAND * Command
    );

/* Destroy an gckCOMMAND object. */
gceSTATUS
gckCOMMAND_Destroy(
    IN gckCOMMAND Command
    );

/* Acquire command queue synchronization objects. */
gceSTATUS
gckCOMMAND_EnterCommit(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    );

/* Release command queue synchronization objects. */
gceSTATUS
gckCOMMAND_ExitCommit(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    );

/* Start the command queue. */
gceSTATUS
gckCOMMAND_Start(
    IN gckCOMMAND Command
    );

/* Stop the command queue. */
gceSTATUS
gckCOMMAND_Stop(
    IN gckCOMMAND Command
    );

/* Commit command buffers. */
gceSTATUS
gckCOMMAND_Commit(
    IN gckCOMMAND Command,
    IN gcsHAL_SUBCOMMIT * SubCommit,
    IN gctUINT32 ProcessId,
    IN gctBOOL Shared,
    OUT gctUINT64_PTR CommitStamp,
    INOUT gctBOOL *contextSwitched
    );

/* Reserve space in the command buffer. */
gceSTATUS
gckCOMMAND_Reserve(
    IN gckCOMMAND Command,
    IN gctUINT32 RequestedBytes,
    OUT gctPOINTER * Buffer,
    OUT gctUINT32 * BufferSize
    );

/*
 * Execute reserved space in the command buffer.
 * Wait link FE version.
 */
gceSTATUS
gckCOMMAND_Execute(
    IN gckCOMMAND Command,
    IN gctUINT32 RequstedBytes
    );

/*
 * Execute reserved space in the command buffer.
 * Async FE version.
 */
gceSTATUS
gckCOMMAND_ExecuteAsync(
    IN gckCOMMAND Command,
    IN gctUINT32 RequestedBytes
    );

/*
 * Execute reserved space in the command buffer.
 * MC-FE version.
 */
gceSTATUS
gckCOMMAND_ExecuteMultiChannel(
    IN gckCOMMAND Command,
    IN gctBOOL Priority,
    IN gctUINT32 ChannelId,
    IN gctUINT32 RequstedBytes
    );

/* Stall the command queue. */
gceSTATUS
gckCOMMAND_Stall(
    IN gckCOMMAND Command,
    IN gctBOOL FromPower
    );

/* Attach user process. */
gceSTATUS
gckCOMMAND_Attach(
    IN gckCOMMAND Command,
    OUT gckCONTEXT * Context,
    OUT gctSIZE_T * MaxState,
    OUT gctUINT32 * NumStates,
    IN gctUINT32 ProcessID
    );

/* Dump command buffer being executed by GPU. */
gceSTATUS
gckCOMMAND_DumpExecutingBuffer(
    IN gckCOMMAND Command
    );

/* Detach user process. */
gceSTATUS
gckCOMMAND_Detach(
    IN gckCOMMAND Command,
    IN gckCONTEXT Context
    );

void
gcsLIST_Init(
    gcsLISTHEAD_PTR Node
    );

void
gcsLIST_Add(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    );

void
gcsLIST_AddTail(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    );

void
gcsLIST_Del(
    gcsLISTHEAD_PTR Node
    );

gctBOOL
gcsLIST_Empty(
    gcsLISTHEAD_PTR Head
    );

#define gcmkLIST_FOR_EACH(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

#define gcmkLIST_FOR_EACH_SAFE(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

gceSTATUS
gckFENCE_Create(
    IN gckOS Os,
    IN gckKERNEL Kernel,
    OUT gckFENCE * Fence
    );

gceSTATUS
gckFENCE_Destory(
    IN gckOS Os,
    OUT gckFENCE Fence
    );

gceSTATUS
gckFENCE_Signal(
    IN gckOS Os,
    IN gckFENCE Fence
    );

gceSTATUS
gckDEVICE_Construct(
    IN gckOS Os,
    OUT gckDEVICE * Device
    );

gceSTATUS
gckDEVICE_AddCore(
    IN gckDEVICE Device,
    IN gceCORE Core,
    IN gctUINT chipID,
    IN gctPOINTER Context,
    IN gckKERNEL * Kernel
    );

gceSTATUS
gckDEVICE_Destroy(
    IN gckOS Os,
    IN gckDEVICE Device
    );

gceSTATUS
gckDEVICE_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    );

#if VIVANTE_PROFILER
gceSTATUS
gckDEVICE_Profiler_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_PROFILER_INTERFACE_PTR Interface
    );
#endif

gceSTATUS
gckDEVICE_GetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU *Mmu
    );

gceSTATUS
gckDEVICE_SetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU Mmu
    );

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_MapInTrustApplicaiton(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 GPUAddress,
    IN gctSIZE_T PageCount
    );
#endif

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckOS_OpenSecurityChannel(
    IN gckOS Os,
    IN gceCORE Core,
    OUT gctUINT32 *Channel
    );

gceSTATUS
gckOS_CloseSecurityChannel(
    IN gctUINT32 Channel
    );

gceSTATUS
gckOS_CallSecurityService(
    IN gctUINT32 Channel,
    IN gcsTA_INTERFACE * Interface
    );

gceSTATUS
gckOS_InitSecurityChannel(
    OUT gctUINT32 Channel
    );

gceSTATUS
gckOS_AllocatePageArray(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T PageCount,
    OUT gctPOINTER * PageArrayLogical,
    OUT gctPHYS_ADDR * PageArrayPhysical
    );
#endif

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_kernel_h_ */
