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
#include <linux/sched.h>
#include <linux/io.h>

#define _GC_OBJ_ZONE    gcvZONE_DEVICE

static gckGALDEVICE galDevice;

extern gcTA globalTA[gcdMAX_GPU_COUNT];

#ifdef CONFIG_DEBUG_FS
#if defined(CONFIG_CPU_CSKYV2) && LINUX_VERSION_CODE <= KERNEL_VERSION(3,0,8)
static void
seq_vprintf(
    struct seq_file *m,
    const char *f,
    va_list args
    )
{
    int len;
    if (m->count < m->size)
    {
        len = vsnprintf(m->buf + m->count, m->size - m->count, f, args);
        if (m->count + len < m->size)
        {
            m->count += len;
            return;
        }
    }
    m->count = m->size;
}
#endif

static int
debugfs_printf(
    IN void* obj,
    IN const char* fmt,
    ...
    )
{
    va_list args;
    va_start(args, fmt);
    seq_vprintf((struct seq_file*)obj, fmt, args);
    va_end(args);

    return 0;
}
#else
static int
sys_printf(
    IN void* obj,
    IN const char* fmt,
    ...
    )
{
    int len = 0;
    va_list args;
    va_start(args, fmt);
    len = vsprintf((char*)obj, fmt, args);
    va_end(args);

    return len;
}
#endif

#ifdef CONFIG_DEBUG_FS
#define fs_printf   debugfs_printf
#else
#define fs_printf   sys_printf
#endif

/******************************************************************************\
******************************** Debugfs Support *******************************
\******************************************************************************/

/******************************************************************************\
***************************** DEBUG SHOW FUNCTIONS *****************************
\******************************************************************************/

int gc_info_show(void* m, void* data)
{
    gckGALDEVICE device = galDevice;
    int i = 0;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif
    gceCHIPMODEL chipModel = 0;
    gctUINT32 chipRevision = 0;
    gctUINT32 productID = 0;
    gctUINT32 ecoID = 0;

    if (!device)
        return -ENXIO;

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i])
        {
            if (i == gcvCORE_VG)
            {
            }
            else
            {
                chipModel = device->kernels[i]->hardware->identity.chipModel;
                chipRevision = device->kernels[i]->hardware->identity.chipRevision;
                productID = device->kernels[i]->hardware->identity.productID;
                ecoID = device->kernels[i]->hardware->identity.ecoID;
            }

            len = fs_printf(ptr, "gpu      : %d\n", i);
            len += fs_printf(ptr + len, "model    : %4x\n", chipModel);
            len += fs_printf(ptr + len, "revision : %4x\n", chipRevision);
            len += fs_printf(ptr + len, "product  : %4x\n", productID);
            len += fs_printf(ptr + len, "eco      : %4x\n", ecoID);
            len += fs_printf(ptr + len, "\n");
        }
    }
    return len;
}

int gc_clients_show(void* m, void* data)
{
    gckGALDEVICE device = galDevice;

    gckKERNEL kernel = _GetValidKernel(device);

    gcsDATABASE_PTR database;
    gctINT i, pid;
    char name[24];
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!kernel)
        return -ENXIO;

    len = fs_printf(ptr, "%-8s%s\n", "PID", "NAME");
    len += fs_printf(ptr + len, "------------------------\n");

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            pid = database->processID;

            gcmkVERIFY_OK(gckOS_GetProcessNameByPid(pid, gcmSIZEOF(name), name));

            len += fs_printf(ptr + len, "%-8d%s\n", pid, name);
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    /* Success. */
    return len;
}

int gc_meminfo_show(void* m, void* data)
{
    gckGALDEVICE device = galDevice;
    gckKERNEL kernel = _GetValidKernel(device);
    gckVIDMEM memory;
    gceSTATUS status;
    gcsDATABASE_PTR database;
    gctUINT32 i;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    gctUINT32 free = 0, used = 0, total = 0, minFree = 0, maxUsed = 0;

    gcsDATABASE_COUNTERS virtualCounter = {0, 0, 0};
    gcsDATABASE_COUNTERS nonPagedCounter = {0, 0, 0};

    if (!kernel)
        return -ENXIO;

    status = gckKERNEL_GetVideoMemoryPool(kernel, gcvPOOL_SYSTEM, &memory);

    if (gcmIS_SUCCESS(status))
    {
        gcmkVERIFY_OK(
            gckOS_AcquireMutex(memory->os, memory->mutex, gcvINFINITE));

        free    = memory->freeBytes;
        minFree = memory->minFreeBytes;
        used    = memory->bytes - memory->freeBytes;
        maxUsed = memory->bytes - memory->minFreeBytes;
        total   = memory->bytes;

        gcmkVERIFY_OK(gckOS_ReleaseMutex(memory->os, memory->mutex));
    }

    len  = fs_printf(ptr, "VIDEO MEMORY:\n");
    len += fs_printf(ptr + len, "  POOL SYSTEM:\n");
    len += fs_printf(ptr + len, "    Free :    %10u B\n", free);
    len += fs_printf(ptr + len, "    Used :    %10u B\n", used);
    len += fs_printf(ptr + len, "    MinFree : %10u B\n", minFree);
    len += fs_printf(ptr + len, "    MaxUsed : %10u B\n", maxUsed);
    len += fs_printf(ptr + len, "    Total :   %10u B\n", total);

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            gcsDATABASE_COUNTERS * counter;
            counter = &database->vidMemPool[gcvPOOL_VIRTUAL];
            virtualCounter.bytes += counter->bytes;
            virtualCounter.maxBytes += counter->maxBytes;

            counter = &database->nonPaged;
            nonPagedCounter.bytes += counter->bytes;
            nonPagedCounter.bytes += counter->maxBytes;
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    len += fs_printf(ptr + len, "  POOL VIRTUAL:\n");
    len += fs_printf(ptr + len, "    Used :    %10llu B\n", virtualCounter.bytes);
    len += fs_printf(ptr + len, "    MaxUsed : %10llu B\n", virtualCounter.bytes);

    return len;
}

int gc_load_show(void* m, void* data)
{
    int len = 0;
    gctUINT32 i = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE device = galDevice;
    gceCHIPPOWERSTATE statesStored, state;
    gctUINT32 load[gcvCORE_3D_MAX + 1] = {0};
    gctUINT32 hi_total_cycle_count[gcvCORE_3D_MAX + 1] = {0};
    gctUINT32 hi_total_idle_cycle_count[gcvCORE_3D_MAX + 1] = {0};
    static gctBOOL profilerEnable[gcvCORE_3D_MAX + 1] = {gcvFALSE};

#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!device)
        return -ENXIO;

    for (i = 0; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->kernels[i])
        {
            if (device->kernels[i]->hardware)
            {
                gckHARDWARE Hardware = device->kernels[i]->hardware;
                gctBOOL powerManagement = Hardware->options.powerManagement;

                if (powerManagement)
                {
                    gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                        Hardware, gcvFALSE
                        ));
                }

                gcmkONERROR(gckHARDWARE_QueryPowerState(
                    Hardware, &statesStored
                    ));

                gcmkONERROR(gckHARDWARE_SetPowerState(
                    Hardware, gcvPOWER_ON_AUTO
                    ));

                if (!profilerEnable[i])
                {
                    gcmkONERROR(gckHARDWARE_SetGpuProfiler(
                        Hardware,
                        gcvTRUE
                        ));

                    gcmkONERROR(gckHARDWARE_InitProfiler(Hardware));

                    profilerEnable[i] = gcvTRUE;
                }

                Hardware->waitCount = 200 * 100;
            }
        }
    }

    for (i = 0; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->kernels[i])
        {
            if (device->kernels[i]->hardware)
            {
                gcmkONERROR(gckHARDWARE_CleanCycleCount(device->kernels[i]->hardware));
            }
        }
    }

    for (i = 0; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->kernels[i])
        {
            if (device->kernels[i]->hardware)
            {
                gcmkONERROR(gckHARDWARE_QueryCycleCount(device->kernels[i]->hardware, &hi_total_cycle_count[i], &hi_total_idle_cycle_count[i]));
            }
        }
    }

    for (i = 0; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->kernels[i])
        {
            if (device->kernels[i]->hardware)
            {
                gckHARDWARE Hardware = device->kernels[i]->hardware;
                gctBOOL powerManagement = Hardware->options.powerManagement;

                switch(statesStored)
                {
                case gcvPOWER_OFF:
                    state = gcvPOWER_OFF_BROADCAST;
                    break;
                case gcvPOWER_IDLE:
                    state = gcvPOWER_IDLE_BROADCAST;
                    break;
                case gcvPOWER_SUSPEND:
                    state = gcvPOWER_SUSPEND_BROADCAST;
                    break;
                case gcvPOWER_ON:
                    state = gcvPOWER_ON_AUTO;
                    break;
                default:
                    state = statesStored;
                    break;
                }

                Hardware->waitCount = 200;

                if (powerManagement)
                {
                    gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                        Hardware, gcvTRUE
                        ));
                }

                gcmkONERROR(gckHARDWARE_SetPowerState(
                    Hardware, state
                    ));

                load[i] = (hi_total_cycle_count[i] - hi_total_idle_cycle_count[i]) * 100 / hi_total_cycle_count[i];

                len += fs_printf(ptr, "core      : %d\n", i);
                len += fs_printf(ptr + len, "load      : %d%%\n",load[i]);
                len += fs_printf(ptr + len, "\n");
            }
        }
    }

OnError:
    return len;
}

static const char * vidmemTypeStr[gcvVIDMEM_TYPE_COUNT] =
{
    "Generic",
    "Index",
    "Vertex",
    "Texture",
    "RenderTarget",
    "Depth",
    "Bitmap",
    "TileStatus",
    "Image",
    "Mask",
    "Scissor",
    "HZ",
    "ICache",
    "TxDesc",
    "Fence",
    "TFBHeader",
    "Command",
};

static const char * poolStr[gcvPOOL_NUMBER_OF_POOLS] =
{
    "Unknown",
    "Default",
    "Local",
    "Internal",
    "External",
    "Unified",
    "System",
    "Sram",
    "Virtual",
    "User",
    "Insram",
    "Exsram",
    "Exclusive",
};

static int
_ShowCounters(
    void *File,
    gcsDATABASE_PTR Database
    )
{
    gctUINT i = 0;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = File;
#else
    char* ptr = (char*)File;
#endif

    static const char * otherCounterNames[] = {
        "AllocNonPaged",
        "AllocContiguous",
        "MapUserMemory",
        "MapMemory",
    };

    gcsDATABASE_COUNTERS * otherCounters[] = {
        &Database->nonPaged,
        &Database->contiguous,
        &Database->mapUserMemory,
        &Database->mapMemory,
    };

    len = fs_printf(ptr, "%-16s %16s %16s %16s\n", "", "Current", "Maximum", "Total");

    /* Print surface type counters. */
    len += fs_printf(ptr + len, "%-16s %16lld %16lld %16lld\n",
               "All-Types",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvVIDMEM_TYPE_COUNT; i++)
    {
        len += fs_printf(ptr + len, "%-16s %16lld %16lld %16lld\n",
                   vidmemTypeStr[i],
                   Database->vidMemType[i].bytes,
                   Database->vidMemType[i].maxBytes,
                   Database->vidMemType[i].totalBytes);
    }
    /*seq_puts(File, "\n");*/
    len += fs_printf(ptr + len, "\n");

    /* Print surface pool counters. */
    len += fs_printf(ptr + len, "%-16s %16lld %16lld %16lld\n",
               "All-Pools",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvPOOL_NUMBER_OF_POOLS; i++)
    {
        len += fs_printf(ptr + len, "%-16s %16lld %16lld %16lld\n",
                   poolStr[i],
                   Database->vidMemPool[i].bytes,
                   Database->vidMemPool[i].maxBytes,
                   Database->vidMemPool[i].totalBytes);
    }
    /*seq_puts(File, "\n");*/
    len += fs_printf(ptr + len, "\n");

    /* Print other counters. */
    for (i = 0; i < gcmCOUNTOF(otherCounterNames); i++)
    {
        len += fs_printf(ptr + len, "%-16s %16lld %16lld %16lld\n",
                   otherCounterNames[i],
                   otherCounters[i]->bytes,
                   otherCounters[i]->maxBytes,
                   otherCounters[i]->totalBytes);
    }
    /*seq_puts(File, "\n");*/
    len += fs_printf(ptr + len, "\n");
    return len;
}

static int
_ShowRecord(
    IN void *File,
    IN gcsDATABASE_PTR Database,
    IN gcsDATABASE_RECORD_PTR Record
    )
{
    gctUINT32 handle;
    gckVIDMEM_NODE nodeObject;
    gctPHYS_ADDR_T physical;
    gceSTATUS status = gcvSTATUS_OK;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = File;
#else
    char* ptr = (char*)File;
#endif

    static const char * recordTypes[gcvDB_NUM_TYPES] = {
        "Unknown",
        "VideoMemory",
        "CommandBuffer",
        "NonPaged",
        "Contiguous",
        "Signal",
        "VidMemLock",
        "Context",
        "Idel",
        "MapMemory",
        "MapUserMemory",
        "ShBuf",
    };

    handle = gcmPTR2INT32(Record->data);

    if (Record->type == gcvDB_VIDEO_MEMORY || Record->type == gcvDB_VIDEO_MEMORY_LOCKED)
    {
        status = gckVIDMEM_HANDLE_Lookup2(
            Record->kernel,
            Database,
            handle,
            &nodeObject
        );

        if (gcmIS_ERROR(status))
        {
            len += fs_printf(ptr + len, "%6u Invalid Node\n", handle);
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(Record->kernel, nodeObject, 0, &physical));
    }
    else
    {
        physical = (gctUINT64)(gctUINTPTR_T)Record->physical;
    }

    len += fs_printf(ptr + len, "%-14s %3d %16x %16zx %16zu\n",
        recordTypes[Record->type],
        Record->kernel->core,
        gcmPTR2INT32(Record->data),
        (size_t) physical,
        Record->bytes
        );

OnError:
    return len;
}

static int
_ShowDataBaseOldFormat(
    IN void *File,
    IN gcsDATABASE_PTR Database
    )
{
    gctINT pid;
    gctUINT i;
    char name[24];
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = File;
#else
    char* ptr = (char*)File;
#endif

    /* Process ID and name */
    pid = Database->processID;
    gcmkVERIFY_OK(gckOS_GetProcessNameByPid(pid, gcmSIZEOF(name), name));

    len = fs_printf(ptr, "--------------------------------------------------------------------------------\n");
    len += fs_printf(ptr + len, "Process: %-8d %s\n", pid, name);

    len += fs_printf(ptr + len, "Records:\n");

    len += fs_printf(ptr + len, "%14s %3s %16s %16s %16s\n",
               "Type", "GPU", "Data/Node", "Physical/Node", "Bytes");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR record = Database->list[i];

        while (record != NULL)
        {
            len += _ShowRecord(ptr + len, Database, record);
            record = record->next;
        }
    }

    len += fs_printf(ptr + len, "Counters:\n");

    len += _ShowCounters(ptr + len, Database);
    return len;
}

static int
gc_db_old_show(void *m, void *data, gctBOOL all)
{
    gcsDATABASE_PTR database;
    gctINT i;
    static gctUINT64 idleTime = 0;
    gckGALDEVICE device = galDevice;
    gckKERNEL kernel = _GetValidKernel(device);
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!kernel)
        return -ENXIO;

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    if (kernel->db->idleTime)
    {
        /* Record idle time if DB upated. */
        idleTime = kernel->db->idleTime;
        kernel->db->idleTime = 0;
    }

    /* Idle time since last call */
    len = fs_printf(ptr, "GPU Idle: %llu ns\n",  idleTime);

    if (all)
    {
        /* Walk the databases. */
        for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
        {
            for (database = kernel->db->db[i];
                 database != gcvNULL;
                 database = database->next)
            {
                len += _ShowDataBaseOldFormat(ptr + len, database);
            }
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    return len;
}

static int
gc_db_show(void *m, void *data, gctBOOL all)
{
    return 0;
}

static int
gc_version_show(void *m, void *data)
{
    gckGALDEVICE device = galDevice;
    gcsPLATFORM * platform = gcvNULL;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!device)
        return -ENXIO;

    platform = device->platform;
    if (!platform)
        return -ENXIO;

#ifdef CONFIG_DEBUG_FS
    len = fs_printf(ptr, "%s built at %s\n",  gcvVERSION_STRING, HOST);

    if (platform->name)
    {
        len += fs_printf(ptr + len, "Platform path: %s\n", platform->name);
    }
    else
    {
        len += fs_printf(ptr + len, "Code path: %s\n", __FILE__);
    }
#else
    len = fs_printf(ptr, "%s\n",  gcvVERSION_STRING);
#endif

    return len;
}

static void print_ull(char dest[32], unsigned long long u)
{
    unsigned t[7];
    int i;

    if (u < 1000)
    {
        sprintf(dest, "%27llu", u);
        return;
    }

    for (i = 0; i < 7 && u; i++)
    {
        t[i] = do_div(u, 1000);
    }

    dest += sprintf(dest, "%*s", (7 - i) * 4, "");
    dest += sprintf(dest, "%3u", t[--i]);

    for (i--; i >= 0; i--)
    {
        dest += sprintf(dest, ",%03u", t[i]);
    }
}

/*******************************************************************************
**
** Show PM state timer.
**
** Entry is called as 'idle' for compatible reason, it shows more information
** than idle actually.
**
**  Start: Start time of this counting period.
**  End: End time of this counting peroid.
**  On: Time GPU stays in gcvPOWER_0N.
**  Off: Time GPU stays in gcvPOWER_0FF.
**  Idle: Time GPU stays in gcvPOWER_IDLE.
**  Suspend: Time GPU stays in gcvPOWER_SUSPEND.
*/
static int
gc_idle_show(void *m, void *data)
{
    gckGALDEVICE device = galDevice;
    gckKERNEL kernel = _GetValidKernel(device);
    char str[32];

    gctUINT64 on;
    gctUINT64 off;
    gctUINT64 idle;
    gctUINT64 suspend;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!kernel)
        return -ENXIO;

    gckHARDWARE_QueryStateTimer(kernel->hardware, &on, &off, &idle, &suspend);

    /* Idle time since last call */
    print_ull(str, on);
    len = fs_printf(ptr, "On:      %s ns\n",  str);
    print_ull(str, off);
    len += fs_printf(ptr + len, "Off:     %s ns\n",  str);
    print_ull(str, idle);
    len += fs_printf(ptr + len, "Idle:    %s ns\n",  str);
    print_ull(str, suspend);
    len += fs_printf(ptr + len, "Suspend: %s ns\n",  str);

    return len;
}

extern void
_DumpState(
    IN gckKERNEL Kernel
    );

/*******************************************************************************
**
** Show PM state timer.
**
** Entry is called as 'idle' for compatible reason, it shows more information
** than idle actually.
**
**  Start: Start time of this counting period.
**  End: End time of this counting peroid.
**  On: Time GPU stays in gcvPOWER_0N.
**  Off: Time GPU stays in gcvPOWER_0FF.
**  Idle: Time GPU stays in gcvPOWER_IDLE.
**  Suspend: Time GPU stays in gcvPOWER_SUSPEND.
*/
static int dumpCore = 0;
static gctBOOL dumpAllCore = gcvFALSE;

static int
gc_dump_trigger_show(void *m, void *data)
{
    int len = 0;

#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    gckGALDEVICE device = galDevice;
    gckKERNEL kernel = gcvNULL;
    gckHARDWARE Hardware = gcvNULL;
    gctBOOL powerManagement = gcvFALSE;
    gceSTATUS status = gcvSTATUS_OK;
    gceCHIPPOWERSTATE statesStored, state;

    if (((dumpCore < gcvCORE_MAJOR) || (dumpCore >= gcvCORE_COUNT)) && (!dumpAllCore))
    {
        return -ENXIO;
    }

    len += fs_printf(ptr + len, "Dump one core: For example, dump core 0: echo 0 > /sys/kernel/debug/gc/dump_trigger; cat /sys/kernel/debug/gc/dump_trigger\n");
    len += fs_printf(ptr + len, "Dump all cores: echo all > /sys/kernel/debug/gc/dump_trigger; cat /sys/kernel/debug/gc/dump_trigger\n");
    len += fs_printf(ptr + len, "The dump will be in [dmesg].\n");

    if (dumpAllCore)
    {
        gctINT8 i = 0;

        for (i = 0; i < gcvCORE_COUNT; ++i)
        {
            if (!device->kernels[i])
            {
                continue;
            }

            kernel = device->kernels[i];
            Hardware = kernel->hardware;
            powerManagement = Hardware->options.powerManagement;

            if (powerManagement)
            {
                gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                    Hardware, gcvFALSE
                    ));
            }

            gcmkONERROR(gckHARDWARE_QueryPowerState(
                Hardware, &statesStored
                ));

            gcmkONERROR(gckHARDWARE_SetPowerState(
                Hardware, gcvPOWER_ON_AUTO
                ));

            _DumpState(kernel);

            switch(statesStored)
            {
            case gcvPOWER_OFF:
                state = gcvPOWER_OFF_BROADCAST;
                break;
            case gcvPOWER_IDLE:
                state = gcvPOWER_IDLE_BROADCAST;
                break;
            case gcvPOWER_SUSPEND:
                state = gcvPOWER_SUSPEND_BROADCAST;
                break;
            case gcvPOWER_ON:
                state = gcvPOWER_ON_AUTO;
                break;
            default:
                state = statesStored;
                break;
            }

            if (powerManagement)
            {
                gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                    Hardware, gcvTRUE
                    ));
            }

            gcmkONERROR(gckHARDWARE_SetPowerState(
                Hardware, state
                ));

        }
    }
    else
    {
        if (device->kernels[dumpCore])
        {
            kernel = device->kernels[dumpCore];
        }
        else
        {
            len += fs_printf(ptr + len, "Dump core from invalid coreid.\n");
            goto OnError;
        }

        Hardware = kernel->hardware;
        powerManagement = Hardware->options.powerManagement;

        if (powerManagement)
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Hardware, gcvFALSE
                ));
        }

        gcmkONERROR(gckHARDWARE_QueryPowerState(
            Hardware, &statesStored
            ));

        gcmkONERROR(gckHARDWARE_SetPowerState(
            Hardware, gcvPOWER_ON_AUTO
            ));

        _DumpState(kernel);

        switch(statesStored)
        {
        case gcvPOWER_OFF:
            state = gcvPOWER_OFF_BROADCAST;
            break;
        case gcvPOWER_IDLE:
            state = gcvPOWER_IDLE_BROADCAST;
            break;
        case gcvPOWER_SUSPEND:
            state = gcvPOWER_SUSPEND_BROADCAST;
            break;
        case gcvPOWER_ON:
            state = gcvPOWER_ON_AUTO;
            break;
        default:
            state = statesStored;
            break;
        }

        if (powerManagement)
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Hardware, gcvTRUE
                ));
        }

        gcmkONERROR(gckHARDWARE_SetPowerState(
            Hardware, state
            ));
    }

OnError:
    return len;
}

static int dumpProcess = 0;

static int
_ShowVideoMemoryOldFormat(
    void *File,
    gcsDATABASE_PTR Database,
    gctBOOL All
    )
{
    gctUINT i = 0;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = File;
#else
    char* ptr = (char*)File;
#endif

    static const char * otherCounterNames[] = {
        "AllocNonPaged",
        "AllocContiguous",
        "MapUserMemory",
        "MapMemory",
    };

    gcsDATABASE_COUNTERS * otherCounters[] = {
        &Database->nonPaged,
        &Database->contiguous,
        &Database->mapUserMemory,
        &Database->mapMemory,
    };

    len = fs_printf(ptr, "%-16s %16s %16s %16s\n", "", "Current", "Maximum", "Total");

    /* Print surface type counters. */
    len += fs_printf(ptr + len, "%-16s %16llu %16llu %16llu\n",
               "All-Types",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    if (All)
    {
        for (i = 1; i < gcvVIDMEM_TYPE_COUNT; i++)
        {
            len += fs_printf(ptr + len, "%-16s %16llu %16llu %16llu\n",
                       vidmemTypeStr[i],
                       Database->vidMemType[i].bytes,
                       Database->vidMemType[i].maxBytes,
                       Database->vidMemType[i].totalBytes);
        }
        /*seq_puts(File, "\n");*/
        len += fs_printf(ptr + len, "\n");

        /* Print surface pool counters. */
        len += fs_printf(ptr + len, "%-16s %16llu %16llu %16llu\n",
                   "All-Pools",
                   Database->vidMem.bytes,
                   Database->vidMem.maxBytes,
                   Database->vidMem.totalBytes);

        for (i = 1; i < gcvPOOL_NUMBER_OF_POOLS; i++)
        {
            len += fs_printf(ptr + len, "%-16s %16llu %16llu %16llu\n",
                       poolStr[i],
                       Database->vidMemPool[i].bytes,
                       Database->vidMemPool[i].maxBytes,
                       Database->vidMemPool[i].totalBytes);
        }
        /*seq_puts(File, "\n");*/
        len += fs_printf(ptr + len, "\n");

        /* Print other counters. */
        for (i = 0; i < gcmCOUNTOF(otherCounterNames); i++)
        {
            len += fs_printf(ptr + len, "%-16s %16llu %16llu %16llu\n",
                       otherCounterNames[i],
                       otherCounters[i]->bytes,
                       otherCounters[i]->maxBytes,
                       otherCounters[i]->totalBytes);
        }
        /*seq_puts(File, "\n");*/
        len += fs_printf(ptr + len, "\n");
    }

    return len;
}

static int gc_vidmem_old_show(void *m, void *unused, gctBOOL all)
{
    gceSTATUS status;
    gcsDATABASE_PTR database;
    gckGALDEVICE device = galDevice;
    char name[64];
    int i;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    gckKERNEL kernel = _GetValidKernel(device);

    if (!kernel)
        return -ENXIO;

    if (dumpProcess == 0)
    {
        /* Acquire the database mutex. */
        gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); i++)
        {
            for (database = kernel->db->db[i];
                 database != gcvNULL;
                 database = database->next)
            {
                gckOS_GetProcessNameByPid(database->processID, gcmSIZEOF(name), name);
                len += fs_printf(ptr + len, "VidMem Usage (Process %u: %s):\n", database->processID, name);
                len += _ShowVideoMemoryOldFormat(ptr + len, database, all);
                /*seq_puts(m, "\n");*/
                len += fs_printf(ptr + len, "\n");
            }
        }

        /* Release the database mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));
    }
    else
    {
        /* Find the database. */
        status = gckKERNEL_FindDatabase(kernel, dumpProcess, gcvFALSE, &database);

        if (gcmIS_ERROR(status))
        {
            len += fs_printf(ptr + len, "ERROR: process %d not found\n", dumpProcess);
            return len;
        }

        gckOS_GetProcessNameByPid(dumpProcess, gcmSIZEOF(name), name);
        len += fs_printf(ptr + len, "VidMem Usage (Process %d: %s):\n", dumpProcess, name);
        len += _ShowVideoMemoryOldFormat(ptr + len, database, all);
    }

    return len;
}

static int gc_vidmem_show(void *m, void *unused, gctBOOL all)
{
    return 0;
}

#ifdef CONFIG_DEBUG_FS
static inline int strtoint_from_user(const char __user *s,
                        size_t count, int *res)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    int ret = kstrtoint_from_user(s, count, 10, res);

    return ret < 0 ? ret : count;
#else
    /* sign, base 2 representation, newline, terminator */
    char buf[1 + sizeof(long) * 8 + 1 + 1];

    size_t len = min(count, sizeof(buf) - 1);

    if (copy_from_user(buf, s, len))
        return -EFAULT;
    buf[len] = '\0';

    *res = (int) simple_strtol(buf, NULL, 0);

    return count;
#endif
}

static int gc_vidmem_write(const char __user *buf, size_t count, void* data)
{
    return strtoint_from_user(buf, count, &dumpProcess);
}

static int gc_dump_trigger_write(const char __user *buf, size_t count, void* data)
{
    char str[1 + sizeof(long) * 8 + 1 + 1];

    size_t len = min(count, sizeof(str) - 1);

    if (copy_from_user(str, buf, len))
        return -EFAULT;

    str[len] = '\0';

    if (str[0] == 'a' && str[1] == 'l' && str[2] == 'l')
    {
        dumpAllCore = gcvTRUE;
        return count;
    }
    else
    {
        dumpAllCore = gcvFALSE;
        return strtoint_from_user(buf, count, &dumpCore);
    }
}

#if gcdENABLE_MP_SWITCH
static int gc_switch_core_count(void* m, void* data)
{
    return 0;
}

static int gc_switch_core_count_write(const char __user *buf, size_t count, void* data)
{
    gckGALDEVICE device = galDevice;
    int coreCount = 0;
    int ret;

    ret = strtoint_from_user(buf, count, &coreCount);

    if (ret && coreCount)
    {
        device->platform->coreCount = coreCount;
    }

    return ret;
}
#endif
#endif

static int gc_clk_show(void* m, void* data)
{
    gckGALDEVICE device = galDevice;
    gctUINT i;
    gceSTATUS status;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!device)
        return -ENXIO;

    for (i = gcvCORE_MAJOR; i < gcvCORE_COUNT; i++)
    {
        if (device->kernels[i])
        {
            gckHARDWARE hardware = device->kernels[i]->hardware;

            if (i == gcvCORE_VG)
            {
                continue;
            }

            status = gckHARDWARE_QueryFrequency(hardware);
            if (gcmIS_ERROR(status))
            {
                len += fs_printf(ptr + len, "query gpu%d clock fail.\n", i);
                continue;
            }

            if (hardware->mcClk)
            {
                len += fs_printf(ptr + len, "gpu%d mc clock: %d HZ.\n", i, hardware->mcClk);
            }

            if (hardware->shClk)
            {
                len += fs_printf(ptr + len, "gpu%d sh clock: %d HZ.\n", i, hardware->shClk);
            }
        }
    }

    return len;
}

static gctINT clkScale[2] = {0, 0};

static int set_clk(const char* buf)
{
    gckHARDWARE hardware;
    gckGALDEVICE device = galDevice;
    gctINT n, j, k;
    gctBOOL isSpace = gcvFALSE;
    char data[20];

    memset(data, 0, 20);
    n = j = k = 0;

    while (gcvTRUE)
    {
        if ((buf[k] >= '0') && (buf[k] <= '9'))
        {
            if (isSpace)
            {
                data[n++] = ' ';
                isSpace = gcvFALSE;
            }
            data[n++] = buf[k];
        }
        else if (buf[k] == ' ')
        {
            if (n > 0)
            {
                isSpace = gcvTRUE;
            }
        }
        else if (buf[k] == '\n')
        {
            break;
        }
        else
        {
            printk("Error: command format must be this: echo \"0 32 32\" > /sys/kernel/debug/gc/clk\n");
            return 0;
        }

        k++;

        if (k >= 20)
        {
            break;
        }
    }

    if (3 == sscanf(data, "%d %d %d", &dumpCore, &clkScale[0], &clkScale[1])) {
        printk("Change core:%d MC scale:%d SH scale:%d\n",
                dumpCore, clkScale[0], clkScale[1]);
    } else {
        printk("usage: echo \"0 32 32\" > clk\n");
        return 0;
    }

    if (device->kernels[dumpCore])
    {
        hardware = device->kernels[dumpCore]->hardware;

        gckHARDWARE_SetClock(hardware, dumpCore, clkScale[0], clkScale[1]);
    }
    else
    {
        printk("Error: invalid core\n");
    }

    return 0;
}

static int gc_poweroff_timeout_show(void* m, void* data)
{
    gckGALDEVICE device = galDevice;
    gckHARDWARE hardware;
    int len = 0;
#ifdef CONFIG_DEBUG_FS
    void* ptr = m;
#else
    char* ptr = (char*)m;
#endif

    if (!device)
        return -ENXIO;

    hardware = device->kernels[0]->hardware;

#ifdef CONFIG_DEBUG_FS
    len += fs_printf(ptr + len, "power off timeout: %d ms.\n", hardware->powerOffTimeout);
#else
    len += sprintf(ptr + len, "power off timeout: %d ms.\n", hardware->powerOffTimeout);
#endif

    return len;
}

static int poweroff_timeout_set(const char* buf)
{
    gckGALDEVICE device = galDevice;
    gctINT i, ret;

    if (!device)
        return -ENXIO;

    for (i = gcvCORE_MAJOR; i < gcvCORE_COUNT; i++)
    {
        if (device->kernels[i])
        {
            gckHARDWARE hardware = device->kernels[i]->hardware;

            if (i == gcvCORE_VG)
            {
                continue;
            }

            ret = kstrtouint(buf, 0, &hardware->powerOffTimeout);
            if (ret < 0)
                return ret;
        }
    }

    return 0;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_copy_from_user(char *k_buf, const char __user *buf, size_t count)
{
    int ret;

    count = min_t(size_t, count, (sizeof(k_buf) - 1));

    ret = copy_from_user(k_buf, buf, count);
    if (ret != 0)
    {
        printk("Error: lost data: %d\n", (int)ret);
        return -1;
    }

    k_buf[count] = 0;

    return count;
}

static int gc_clk_write(const char __user *buf, size_t count, void* data)
{
    size_t ret;
    char k_buf[30];

    ret = debugfs_copy_from_user(k_buf, buf, count);
    if (ret == -1)
        return ret;

    set_clk(k_buf);

    return ret;
}

static int gc_poweroff_timeout_write(const char __user *buf, size_t count, void* data)
{
    size_t ret;
    char k_buf[30];

    ret = debugfs_copy_from_user(k_buf, buf, count);
    if (ret == -1)
        return ret;

    poweroff_timeout_set(k_buf);

    return ret;
}

int gc_info_show_debugfs(struct seq_file* m, void* data)
{
    return gc_info_show((void*)m , data);
}
int gc_clients_show_debugfs(struct seq_file* m, void* data)
{
    return gc_clients_show((void*)m , data);
}
int gc_meminfo_show_debugfs(struct seq_file* m, void* data)
{
    return gc_meminfo_show((void*)m , data);
}
int gc_idle_show_debugfs(struct seq_file* m, void* data)
{
    return gc_idle_show((void*)m , data);
}
int gc_db_old_show_debugfs(struct seq_file* m, void* data)
{
    return gc_db_old_show((void*)m , data, gcvTRUE);
}
int gc_db_show_debugfs(struct seq_file* m, void* data)
{
    return gc_db_show((void*)m , data, gcvTRUE);
}
int gc_version_show_debugfs(struct seq_file* m, void* data)
{
    return gc_version_show((void*)m , data);
}
int gc_vidmem_old_show_debugfs(struct seq_file* m, void* data)
{
    return gc_vidmem_old_show((void*)m , data, gcvTRUE);
}
int gc_vidmem_show_debugfs(struct seq_file* m, void* data)
{
    return gc_vidmem_show((void*)m , data, gcvTRUE);
}
int gc_dump_trigger_show_debugfs(struct seq_file* m, void* data)
{
    return gc_dump_trigger_show((void*)m , data);
}
int gc_clk_show_debugfs(struct seq_file* m, void* data)
{
    return gc_clk_show((void*)m , data);
}

int gc_poweroff_timeout_show_debugfs(struct seq_file* m, void* data)
{
    return gc_poweroff_timeout_show((void*)m, data);
}

#if gcdENABLE_MP_SWITCH
int gc_switch_core_count_debugfs(struct seq_file* m, void* data)
{
    return gc_switch_core_count((void*)m , data);
}
#endif

#if VIVANTE_PROFILER
int gc_load_show_debugfs(struct seq_file* m, void* data)
{
    return gc_load_show((void*)m , data);
}
#endif

static gcsINFO InfoList[] =
{
    {"info", gc_info_show_debugfs},
    {"clients", gc_clients_show_debugfs},
    {"meminfo", gc_meminfo_show_debugfs},
    {"idle", gc_idle_show_debugfs},
    {"database", gc_db_old_show_debugfs},
    {"database64x", gc_db_show_debugfs},
    {"version", gc_version_show_debugfs},
    {"vidmem", gc_vidmem_old_show_debugfs, gc_vidmem_write},
    {"vidmem64x", gc_vidmem_show_debugfs, gc_vidmem_write},
    {"dump_trigger", gc_dump_trigger_show_debugfs, gc_dump_trigger_write},
    {"clk", gc_clk_show_debugfs, gc_clk_write},
    {"poweroff_timeout", gc_poweroff_timeout_show_debugfs, gc_poweroff_timeout_write},
#if gcdENABLE_MP_SWITCH
    {"core_count", gc_switch_core_count_debugfs, gc_switch_core_count_write},
#endif
#if VIVANTE_PROFILER
    {"load", gc_load_show_debugfs},
#endif
};

#else
static ssize_t info_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_info_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(info);

static ssize_t clients_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_clients_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(clients);

static ssize_t meminfo_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_meminfo_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(meminfo);

static ssize_t idle_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_idle_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(idle);

static ssize_t database_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_db_old_show((void*)buf, NULL, gcvFALSE);
}
DEVICE_ATTR_RO(database);

static ssize_t database64x_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_db_show((void*)buf, NULL, gcvFALSE);
}
DEVICE_ATTR_RO(database64x);

static ssize_t version_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_version_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(version);

static ssize_t load_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_load_show((void*)buf, NULL);
}
DEVICE_ATTR_RO(load);

static ssize_t vidmem_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_vidmem_old_show((void*)buf, NULL, gcvFALSE);
}
static ssize_t vidmem_store(struct device *dev, struct device_attribute* attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &dumpProcess);
    return count;
}
DEVICE_ATTR_RW(vidmem);

static ssize_t vidmem64x_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_vidmem_show((void*)buf, NULL, gcvFALSE);
}
static ssize_t vidmem64x_store(struct device *dev, struct device_attribute* attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &dumpProcess);
    return count;
}
DEVICE_ATTR_RW(vidmem64x);

static ssize_t dump_trigger_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_dump_trigger_show((void*)buf, NULL);
}
static ssize_t dump_trigger_store(struct device *dev, struct device_attribute* attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &dumpCore);
    return count;
}
DEVICE_ATTR_RW(dump_trigger);

static ssize_t clk_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_clk_show((void*)buf, NULL);
}
static ssize_t clk_store(struct device *dev, struct device_attribute* attr, const char *buf, size_t count)
{
    set_clk(buf);
    return count;
}
DEVICE_ATTR_RW(clk);

static ssize_t poweroff_timeout_show(struct device *dev, struct device_attribute* attr, char *buf)
{
    return gc_poweroff_timeout_show((void*)buf, NULL);
}
static ssize_t poweroff_timeout_store(struct device *dev, struct device_attribute* attr, const char *buf, size_t count)
{
    poweroff_timeout_set(buf);
    return count;
}
DEVICE_ATTR_RW(poweroff_timeout);

static struct attribute *Info_attrs[] = {
    &dev_attr_info.attr,
    &dev_attr_clients.attr,
    &dev_attr_meminfo.attr,
    &dev_attr_idle.attr,
    &dev_attr_database.attr,
    &dev_attr_database64x.attr,
    &dev_attr_version.attr,
    &dev_attr_vidmem.attr,
    &dev_attr_vidmem64x.attr,
    &dev_attr_dump_trigger.attr,
    &dev_attr_clk.attr,
    &dev_attr_poweroff_timeout.attr,
    NULL,
};
ATTRIBUTE_GROUPS(Info);
#endif

static gceSTATUS
_DebugfsInit(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status = gcvSTATUS_OK;

#ifdef CONFIG_DEBUG_FS
    gckDEBUGFS_DIR dir = &Device->debugfsDir;

    gcmkONERROR(gckDEBUGFS_DIR_Init(dir, gcvNULL, "gc"));
    gcmkONERROR(gckDEBUGFS_DIR_CreateFiles(dir, InfoList, gcmCOUNTOF(InfoList), Device));
#else
    int ret;
    ret = sysfs_create_groups(&galcore_device->kobj, Info_groups);
    if (ret < 0)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }
#endif
    galDevice = Device;

OnError:
    return status;
}

static void
_DebugfsCleanup(
    IN gckGALDEVICE Device
    )
{
#ifdef CONFIG_DEBUG_FS
    gckDEBUGFS_DIR dir = &Device->debugfsDir;

    if (Device->debugfsDir.root)
    {
        gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(dir, InfoList, gcmCOUNTOF(InfoList)));

        gckDEBUGFS_DIR_Deinit(dir);
    }
#else
    sysfs_remove_groups(&galcore_device->kobj, Info_groups);
#endif
}


/******************************************************************************\
*************************** Memory Allocation Wrappers *************************
\******************************************************************************/

static gceSTATUS
_AllocateMemory(
    IN gckGALDEVICE Device,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical,
    OUT gctPHYS_ADDR *Physical,
    OUT gctUINT64 *PhysAddr
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPHYS_ADDR_T physAddr;

    gcmkHEADER_ARG("Device=%p Bytes=0x%zx", Device, Bytes);

    gcmkVERIFY_ARGUMENT(Device != NULL);
    gcmkVERIFY_ARGUMENT(Logical != NULL);
    gcmkVERIFY_ARGUMENT(Physical != NULL);
    gcmkVERIFY_ARGUMENT(PhysAddr != NULL);

    gcmkONERROR(gckOS_AllocateNonPagedMemory(
        Device->os, gcvFALSE, gcvALLOC_FLAG_CONTIGUOUS, &Bytes, Physical, Logical
        ));

    gcmkONERROR(gckOS_GetPhysicalFromHandle(
        Device->os, *Physical, 0, &physAddr
        ));

    *PhysAddr = physAddr;

OnError:
    gcmkFOOTER_ARG(
        "*Logical=%p *Physical=%p *PhysAddr=0x%llx",
        gcmOPT_POINTER(Logical), gcmOPT_POINTER(Physical), gcmOPT_VALUE(PhysAddr)
        );

    return status;
}

static gceSTATUS
_FreeMemory(
    IN gckGALDEVICE Device,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=%p Logical=%p Physical=%p",
                   Device, Logical, Physical);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    status = gckOS_FreeNonPagedMemory(
        Device->os, Physical, Logical,
        ((PLINUX_MDL) Physical)->numPages * PAGE_SIZE
        );

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_SetupContiguousVidMem(
    IN gckGALDEVICE Device,
    IN const gcsMODULE_PARAMETERS * Args
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT64 physAddr = ~0ULL;
    gckGALDEVICE device = Device;

    gcmkHEADER_ARG("Device=%p Args=%p", Device, Args);

    /* set up the contiguous memory */
    device->contiguousBase = Args->contiguousBase;
    device->contiguousSize = Args->contiguousSize;

    if (Args->contiguousSize == 0)
    {
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    if (Args->contiguousBase == 0)
    {
        while (device->contiguousSize > 0)
        {
            /* Allocate contiguous memory. */
            status = _AllocateMemory(
                device,
                device->contiguousSize,
                &device->contiguousLogical,
                &device->contiguousPhysical,
                &physAddr
                );

            if (gcmIS_SUCCESS(status))
            {
                status = gckVIDMEM_Construct(
                    device->os,
                    physAddr,
                    device->contiguousSize,
                    64,
                    Args->bankSize,
                    &device->contiguousVidMem
                    );

                if (gcmIS_SUCCESS(status))
                {
                    gckALLOCATOR allocator = ((PLINUX_MDL)device->contiguousPhysical)->allocator;
                    device->contiguousVidMem->capability = allocator->capability | gcvALLOC_FLAG_MEMLIMIT;
                    device->contiguousVidMem->physical = device->contiguousPhysical;
                    device->contiguousBase = physAddr;
                    if (device->contiguousBase > 0xFFFFFFFFULL)
                    {
                        device->contiguousVidMem->capability &= ~gcvALLOC_FLAG_4GB_ADDR;
                    }
                    break;
                }

                gcmkONERROR(_FreeMemory(
                    device,
                    device->contiguousLogical,
                    device->contiguousPhysical
                    ));

                device->contiguousLogical  = gcvNULL;
                device->contiguousPhysical = gcvNULL;
            }

            if (device->contiguousSize <= (4 << 20))
            {
                device->contiguousSize = 0;
            }
            else
            {
                device->contiguousSize -= (4 << 20);
            }
        }
    }
    else if (device->os->iommu)
    {
        /* Disable contiguous memory pool. */
        device->contiguousVidMem = gcvNULL;
        device->contiguousSize   = 0;
    }
    else
    {
        /* Create the contiguous memory heap. */
        status = gckVIDMEM_Construct(
            device->os,
            Args->contiguousBase,
            Args->contiguousSize,
            64,
            Args->bankSize,
            &device->contiguousVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable contiguous memory pool. */
            device->contiguousVidMem = gcvNULL;
            device->contiguousSize   = 0;
        }
        else
        {
            gckALLOCATOR allocator;
            gctBOOL contiguousRequested = Args->contiguousRequested;

#if gcdCAPTURE_ONLY_MODE
            contiguousRequested = gcvTRUE;
#endif

            gcmkONERROR(gckOS_RequestReservedMemory(
                device->os, Args->contiguousBase, Args->contiguousSize,
                "gcContMem",
                contiguousRequested,
                gcvTRUE,
                &device->contiguousPhysical
                ));

            allocator = ((PLINUX_MDL)device->contiguousPhysical)->allocator;

            device->contiguousVidMem->capability = allocator->capability | gcvALLOC_FLAG_MEMLIMIT;
            device->contiguousVidMem->physical = device->contiguousPhysical;
            device->requestedContiguousBase = Args->contiguousBase;
            device->requestedContiguousSize = Args->contiguousSize;

            device->contiguousPhysName = 0;
            device->contiguousSize = Args->contiguousSize;
        }
    }

    if (Args->showArgs)
    {
        gcmkPRINT("Galcore Info: ContiguousBase=0x%llx ContiguousSize=0x%zx\n", device->contiguousBase, device->contiguousSize);
    }

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_SetupExternalSRAMVidMem(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE device = Device;
    gctINT32 i, j = 0;

    gcmkHEADER_ARG("Device=%p", Device);

    /* Setup external SRAM memory region. */
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        if (!device->extSRAMSizes[i])
        {
            /* Keep this path for internal test, read from feature database. */
            device->extSRAMSizes[i] = device->device->extSRAMSizes[i];
        }

        if (device->extSRAMSizes[i] > 0)
        {
            /* create the external SRAM memory heap */
            status = gckVIDMEM_Construct(
                device->os,
                device->extSRAMBases[i],
                device->extSRAMSizes[i],
                64,
                0,
                &device->extSRAMVidMem[i]
                );

            if (gcmIS_ERROR(status))
            {
                /* Error, disable external SRAM heap. */
                device->extSRAMSizes[i] = 0;
            }
            else
            {
                char sRAMName[40];
                snprintf(sRAMName, gcmSIZEOF(sRAMName) - 1, "Galcore external sram%d", i);

#if gcdCAPTURE_ONLY_MODE
                device->args.sRAMRequested = gcvTRUE;
#endif
                /* Map external SRAM memory. */
                gcmkONERROR(gckOS_RequestReservedMemory(
                        device->os,
                        device->extSRAMBases[i], device->extSRAMSizes[i],
                        sRAMName,
                        device->args.sRAMRequested,
                        gcvTRUE,
                        &device->extSRAMPhysical[i]
                        ));

                device->extSRAMVidMem[i]->physical = device->extSRAMPhysical[i];
                device->device->extSRAMPhysical[i] = device->extSRAMPhysical[i];

                for (j = 0; j < gcdMAX_GPU_COUNT; j++)
                {
                    if (device->irqLines[j] != -1 && device->kernels[j])
                    {
                        device->kernels[j]->hardware->options.extSRAMGPUPhysNames[i] = gckKERNEL_AllocateNameFromPointer(device->kernels[j], device->extSRAMPhysical[i]);
                    }
                }
            }
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

/******************************************************************************\
******************************* Interrupt Handler ******************************
\******************************************************************************/
static irqreturn_t isrRoutine(int irq, void *ctxt)
{
    gceSTATUS status;
    gckGALDEVICE device;
    gceCORE core = (gceCORE)gcmPTR2INT32(ctxt) - 1;

    device = galDevice;

    /* Call kernel interrupt notification. */
    status = gckHARDWARE_Interrupt(device->kernels[core]->hardware);

    if (gcmIS_SUCCESS(status))
    {
        up(&device->semas[core]);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static irqreturn_t isrRoutineVG(int irq, void *ctxt)
{
    return IRQ_NONE;
}

static const char *isrNames[] =
{
    "galcore:0",
    "galcore:3d-1",
    "galcore:3d-2",
    "galcore:3d-3",
    "galcore:3d-4",
    "galcore:3d-5",
    "galcore:3d-6",
    "galcore:3d-7",
    "galcore:3d-8",
    "galcore:3d-9",
    "galcore:3d-10",
    "galcore:3d-11",
    "galcore:3d-12",
    "galcore:3d-13",
    "galcore:3d-14",
    "galcore:3d-15",
    "galcore:2d",
    "galcore:2d1",
    "galcore:2d2",
    "galcore:2d3",
    "galcore:vg",
#if gcdDEC_ENABLE_AHB
    "galcore:dec"
#endif
};

static int isrRoutinePoll(void *ctxt)
{
    gckGALDEVICE device;
    gceCORE core = (gceCORE)gcmPTR2INT32(ctxt);

    device = galDevice;

    gcmSTATIC_ASSERT(gcvCORE_COUNT == gcmCOUNTOF(isrNames),
                     "isrNames array does not match core types");

    while (1)
    {
        if (unlikely(device->killThread))
        {
            /* The daemon exits. */
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        if (core == gcvCORE_VG)
        {
            isrRoutineVG(-1, gcvNULL);
        }
        else
        {
            isrRoutine(-1, (gctPOINTER)(uintptr_t)(core + 1));
        }

        gckOS_Delay(device->os, 10);
    }

    return 0;
}

static gceSTATUS
_SetupIsr(
    IN gceCORE Core
    )
{
    gctINT ret = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE Device = galDevice;
    irq_handler_t handler;

    gcmkHEADER_ARG("Device=%p Core=%d", Device, Core);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    gcmSTATIC_ASSERT(gcvCORE_COUNT == gcmCOUNTOF(isrNames),
                     "isrNames array does not match core types");

    if (Device->irqLines[Core] == -1)
    {
        gctUINT64 isrPolling = -1;

        if (Device->isrThread[Core])
        {
            return status;
        }

        gckOS_QueryOption(Device->os, "isrPoll", &isrPolling);

        /* use kthread to poll int stat */
        if (gcmBITTEST(isrPolling, Core) != 0)
        {
            struct task_struct * task;

            Device->killIsrThread = gcvFALSE;

            task = kthread_run(isrRoutinePoll, (gctPOINTER)Core, "%s_poll", isrNames[Core]);

            if (IS_ERR(task))
            {
                gcmkTRACE_ZONE(
                    gcvLEVEL_ERROR, gcvZONE_DRIVER,
                    "%s(%d): Could not start the intr poll thread.\n",
                    __FUNCTION__, __LINE__
                    );

                gcmkONERROR(gcvSTATUS_GENERIC_IO);
            }

            gcmkPRINT("galcore: polling core%d int state\n", Core);

            Device->isrThread[Core] = task;
            Device->isrInitializeds[Core] = gcvTRUE;

            return status;
        }
        /* it should not run to here */
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    handler = (Core == gcvCORE_VG) ? isrRoutineVG : isrRoutine;

    /*
     * Hook up the isr based on the irq line.
     * For shared irq, device-id can not be 0, but CORE_MAJOR value is.
     * Add by 1 here and subtract by 1 in isr to fix the issue.
     */
    ret = request_irq(
        Device->irqLines[Core], handler, gcdIRQF_FLAG,
        isrNames[Core], (void *)(uintptr_t)(Core + 1)
        );

    if (ret != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not register irq line %d (error=%d)\n",
            __FUNCTION__, __LINE__,
            Device->irqLines[Core], ret
            );

        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Mark ISR as initialized. */
    Device->isrInitializeds[Core] = gcvTRUE;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_ReleaseIsr(
    IN gceCORE Core
    )
{
    gckGALDEVICE Device = galDevice;

    gcmkHEADER_ARG("Device=%p Core=%d", Device, Core);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    /* release the irq */
    if (Device->isrInitializeds[Core])
    {
        if (Device->isrThread[Core])
        {
            Device->killIsrThread = gcvTRUE;
            kthread_stop(Device->isrThread[Core]);
            Device->isrThread[Core] = gcvNULL;
        }
        else
        {
            free_irq(Device->irqLines[Core], (void *)(uintptr_t)(Core + 1));
        }

        Device->isrInitializeds[Core] = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static int threadRoutine(void *ctxt)
{
    gckGALDEVICE device = galDevice;
    gceCORE core = (gceCORE) gcmPTR2INT32(ctxt);

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                   "Starting isr Thread with extension=%p",
                   device);


    for (;;)
    {
        int down;

        down = down_interruptible(&device->semas[core]);
        if (down && down != -EINTR)
        {
            return down;
        }

        if (unlikely(device->killThread))
        {
            /* The daemon exits. */
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        gckKERNEL_Notify(device->kernels[core], gcvNOTIFY_INTERRUPT);
    }
}

static gceSTATUS
_StartThread(
    IN gckGALDEVICE Device,
    IN gceCORE Core
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE device = galDevice;
    struct task_struct * task;

    if (device->kernels[Core] != gcvNULL)
    {
        /* Start the kernel thread. */
        task = kthread_run(threadRoutine, (void *)Core,
                "galcore_deamon/%d", Core);

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        device->threadCtxts[Core]         = task;
        device->threadInitializeds[Core] = device->kernels[Core]->threadInitialized = gcvTRUE;

        set_user_nice(task, -20);
    }
    else
    {
        device->threadInitializeds[Core] = gcvFALSE;
    }

OnError:
    return status;
}

static void
_StopThread(
    gckGALDEVICE Device,
    gceCORE Core
    )
{
    if (Device->threadInitializeds[Core])
    {
        Device->killThread = gcvTRUE;
        up(&Device->semas[Core]);

        kthread_stop(Device->threadCtxts[Core]);
        Device->threadCtxts[Core]        = gcvNULL;
        Device->threadInitializeds[Core] = gcvFALSE;
    }
}

#if gcdENABLE_SW_PREEMPTION

static int
_ThreadPreempt(
    void *ctxt
    )
{
    gckGALDEVICE device = galDevice;
    gceCORE core = (gceCORE) gcmPTR2INT32(ctxt);

    sema_init((struct semaphore *)(device->kernels[core]->preemptSema), 0);

    for (;;)
    {
        int down;

        down = down_interruptible(device->kernels[core]->preemptSema);
        if (down && down != -EINTR)
        {
            return down;
        }

        if (unlikely(device->killPreemptThread))
        {
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        gckKERNEL_PreemptionThread(device->kernels[core]);
    }
}

static gceSTATUS
_StartPreemptThread(
    IN gckGALDEVICE Device,
    IN gceCORE Core
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE device = galDevice;
    struct task_struct * task;

    if (device->kernels[Core] != gcvNULL)
    {
        /* Start the kernel thread. */
        task = kthread_run(_ThreadPreempt, (void *)Core,
                "galcore_preempt/%d", Core);

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel preempt thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        device->preemptThread[Core]      = task;
        device->preemptThreadInits[Core] = gcvTRUE;
    }
    else
    {
        device->preemptThreadInits[Core] = gcvFALSE;
    }

OnError:
    return status;
}

static void
_StopPreemptThread(
    gckGALDEVICE Device,
    gceCORE Core
    )
{
    if (Device->preemptThreadInits[Core])
    {
        Device->killPreemptThread = gcvTRUE;
        up(Device->kernels[Core]->preemptSema);

        kthread_stop(Device->preemptThread[Core]);
        Device->preemptThread[Core]      = gcvNULL;
        Device->preemptThreadInits[Core] = gcvFALSE;
    }
}
#endif

/*******************************************************************************
**
**  gckGALDEVICE_Construct
**
**  Constructor.
**
**  INPUT:
**
**  OUTPUT:
**
**      gckGALDEVICE * Device
**          Pointer to a variable receiving the gckGALDEVICE object pointer on
**          success.
*/
gceSTATUS
gckGALDEVICE_Construct(
    IN gcsPLATFORM * Platform,
    IN const gcsMODULE_PARAMETERS * Args,
    OUT gckGALDEVICE *Device
    )
{
    gckKERNEL kernel = gcvNULL;
    gckGALDEVICE device;
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT64 isrPolling = -1;
    gctINT32 i;

    gcmkHEADER_ARG("Platform=%p Args=%p", Platform, Args);

    /* Allocate device structure. */
    device = kmalloc(sizeof(struct _gckGALDEVICE), GFP_KERNEL | __GFP_NOWARN);

    if (!device)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    memset(device, 0, sizeof(struct _gckGALDEVICE));

    device->platform = Platform;
    device->platform->dev = gcvNULL;

    device->args = *Args;

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        device->irqLines[i]                  = Args->irqs[i];
        device->requestedRegisterMemBases[i] = Args->registerBases[i];
        device->requestedRegisterMemSizes[i] = Args->registerSizes[i];
#if USE_LINUX_PCIE
        device->bars[i]                      = Args->bars[i];
#endif
        gcmkTRACE_ZONE(gcvLEVEL_INFO, _GC_OBJ_ZONE,
                       "Get register base %llx of core %d",
                       Args->registerBases[i], i);
    }

    device->requestedContiguousBase  = 0;
    device->requestedContiguousSize  = 0;

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        unsigned long physical;
        physical = (unsigned long)device->requestedRegisterMemBases[i];

        /* Set up register memory region. */
        if (physical != 0)
        {
            if (Args->registerBasesMapped[i])
            {
                device->registerBases[i] = Args->registerBasesMapped[i];
                device->requestedRegisterMemBases[i] = 0;
            }
            else
            {
#if USE_LINUX_PCIE
                gcmkPRINT("register should be mapped in platform layer");
#endif
                if (!request_mem_region(physical,
                        device->requestedRegisterMemSizes[i],
                        "galcore register region"))
                {
                    gcmkTRACE_ZONE(
                            gcvLEVEL_ERROR, gcvZONE_DRIVER,
                            "%s(%d): Failed to claim %lu bytes @ 0x%llx\n",
                            __FUNCTION__, __LINE__,
                            device->requestedRegisterMemSizes[i], physical
                            );

                    gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
                device->registerBases[i] = (gctPOINTER)ioremap(physical, device->requestedRegisterMemSizes[i]);
#else
                device->registerBases[i] = (gctPOINTER)ioremap_nocache(physical, device->requestedRegisterMemSizes[i]);
#endif

                if (device->registerBases[i] == gcvNULL)
                {
                    gcmkTRACE_ZONE(
                            gcvLEVEL_ERROR, gcvZONE_DRIVER,
                            "%s(%d): Unable to map %ld bytes @ 0x%zx\n",
                            __FUNCTION__, __LINE__,
                            physical, device->requestedRegisterMemSizes[i]
                            );

                    gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                }
            }
        }
    }

    /* Set the base address */
    device->baseAddress = device->physBase = Args->baseAddress;
    device->physSize = Args->physSize;

    /* Set the external base address */
    device->externalBase = Args->externalBase;
    device->externalSize = Args->externalSize;

    /* Set the extern base address and none cpu access */
    device->exclusiveBase = Args->exclusiveBase;
    device->exclusiveSize = Args->exclusiveSize;
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        device->extSRAMBases[i] = Args->extSRAMBases[i];
        device->extSRAMSizes[i] = Args->extSRAMSizes[i];
    }

    /* Construct the gckOS object. */
    gcmkONERROR(gckOS_Construct(device, &device->os));


    if (device->externalSize > 0)
    {
        /* create the external memory heap */
        status = gckVIDMEM_Construct(
            device->os,
            device->externalBase,
            device->externalSize,
            64,
            0,
            &device->externalVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable external heap. */
            device->externalSize = 0;
        }
        else
        {
            /* Map external memory. */
            gcmkONERROR(gckOS_RequestReservedMemory(
                    device->os,
                    device->externalBase, device->externalSize,
                    "gcExtMem",
                    gcvTRUE,
                    gcvTRUE,
                    &device->externalPhysical
                    ));

            device->externalVidMem->physical = device->externalPhysical;
        }
    }

    if (device->exclusiveSize > 0)
    {
        /* create the exclusive memory heap */
        status = gckVIDMEM_Construct(
            device->os,
            device->exclusiveBase,
            device->exclusiveSize,
            64,
            0,
            &device->exclusiveVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable exclusive heap. */
            device->exclusiveSize = 0;
        }
        else
        {
            gckALLOCATOR allocator;
            /* Map exclusive memory. */
            gcmkONERROR(gckOS_RequestReservedMemory(
                    device->os,
                    device->exclusiveBase, device->exclusiveSize,
                    "gcExclMem",
                    gcvTRUE,
                    gcvFALSE,
                    &device->exclusivePhysical
                    ));
            allocator = ((PLINUX_MDL)device->exclusivePhysical)->allocator;
            device->exclusiveVidMem->physical = device->exclusivePhysical;
            device->exclusiveVidMem->capability |= allocator->capability;
        }
    }

    /* Construct the gckDEVICE object for os independent core management. */
    gcmkONERROR(gckDEVICE_Construct(device->os, &device->device));

    device->device->showSRAMMapInfo = Args->showArgs;

    device->platform->dev = device->device;

    gckOS_QueryOption(device->os, "isrPoll", &isrPolling);

    if (device->irqLines[gcvCORE_MAJOR] != -1 || gcmBITTEST(isrPolling, gcvCORE_MAJOR)!= 0)
    {
        gcmkONERROR(gctaOS_ConstructOS(device->os, &device->taos));
    }

    /* Setup contiguous video memory pool. */
    gcmkONERROR(_SetupContiguousVidMem(device, Args));

#if gcdEXTERNAL_SRAM_USAGE
    /* Setup external SRAM video memory pool. */
    gcmkONERROR(_SetupExternalSRAMVidMem(device));
#endif

    /* Add core for all available major cores. */
    for (i = gcvCORE_MAJOR; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->irqLines[i] != -1 || gcmBITTEST(isrPolling, i)!= 0)
        {
            gcmkONERROR(gckDEVICE_AddCore(
                device->device,
                (gceCORE)i,
                Args->chipIDs[i],
                device,
                &device->kernels[i]
                ));

            if (device->kernels[i]->hardware->options.secureMode == gcvSECURE_IN_TA)
            {
                gcmkONERROR(gcTA_Construct(
                    device->taos,
                    (gceCORE)i,
                    &globalTA[i]
                    ));
            }

            gcmkONERROR(gckHARDWARE_SetFastClear(
                device->kernels[i]->hardware,
                Args->fastClear,
                Args->compression
                ));

            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                device->kernels[i]->hardware,
                Args->powerManagement
                ));

#if gcdENABLE_FSCALE_VAL_ADJUST
            gcmkONERROR(gckHARDWARE_SetMinFscaleValue(
                device->kernels[i]->hardware,
                Args->gpu3DMinClock
                ));
#endif
        }
        else
        {
            device->kernels[i] = gcvNULL;
        }
    }

    for (i = gcvCORE_2D; i <= gcvCORE_2D_MAX; i++)
    {
#if !gcdCAPTURE_ONLY_MODE
        if (device->irqLines[i] != -1 || gcmBITTEST(isrPolling, i)!= 0)
        {
            gcmkONERROR(gckDEVICE_AddCore(
                device->device,
                (gceCORE)i,
                Args->chipIDs[i],
                device,
                &device->kernels[i]
                ));

            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                device->kernels[i]->hardware,
                Args->powerManagement
                ));

#if gcdENABLE_FSCALE_VAL_ADJUST
            gcmkONERROR(gckHARDWARE_SetMinFscaleValue(
                device->kernels[i]->hardware, 1
                ));
#endif
        }
        else
        {
            device->kernels[i] = gcvNULL;
        }
#else
        device->kernels[i] = gcvNULL;
#endif
    }

#if !gcdCAPTURE_ONLY_MODE
    if (device->irqLines[gcvCORE_VG] != -1 || gcmBITTEST(isrPolling, gcvCORE_VG)!= 0)
    {
    }
    else
    {
        device->kernels[gcvCORE_VG] = gcvNULL;
    }
#else
    device->kernels[gcvCORE_VG] = gcvNULL;
#endif

#if !gcdEXTERNAL_SRAM_USAGE
    /* Setup external SRAM video memory pool. */
    gcmkONERROR(_SetupExternalSRAMVidMem(device));
#endif

    /* Initialize the kernel thread semaphores. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if ((device->irqLines[i] != -1 || gcmBITTEST(isrPolling, i)!= 0)
            && device->kernels[i])
        {
            sema_init(&device->semas[i], 0);
        }
    }

    /* Grab the first valid kernel. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i] != gcvNULL)
        {
            kernel = device->kernels[i];
            break;
        }
    }

    if (!kernel)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (device->internalPhysical)
    {
        device->internalPhysName = gcmPTR_TO_NAME(device->internalPhysical);
    }

    if (device->externalPhysical)
    {
        device->externalPhysName = gcmPTR_TO_NAME(device->externalPhysical);
    }

    if (device->exclusivePhysical)
    {
        device->exclusivePhysName = gcmPTR_TO_NAME(device->exclusivePhysical);
    }

    if (device->contiguousPhysical)
    {
        device->contiguousPhysName = gcmPTR_TO_NAME(device->contiguousPhysical);
    }

    gcmkONERROR(_DebugfsInit(device));

    /* Return pointer to the device. */
    *Device = galDevice = device;

OnError:
    if (gcmIS_ERROR(status))
    {
        /* Roll back. */
        gcmkVERIFY_OK(gckGALDEVICE_Destroy(device));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Destroy
**
**  Class destructor.
**
**  INPUT:
**
**      Nothing.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Destroy(
    gckGALDEVICE Device)
{
    gctINT i, j = 0;
    gckKERNEL kernel = gcvNULL;

    gcmkHEADER_ARG("Device=%p", Device);

    if (Device != gcvNULL)
    {
        /* Grab the first available kernel */
        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->kernels[i])
            {
                kernel = Device->kernels[i];
                break;
            }
        }

        if (kernel)
        {
            if (Device->internalPhysName != 0)
            {
                gcmRELEASE_NAME(Device->internalPhysName);
                Device->internalPhysName = 0;
            }
            if (Device->externalPhysName != 0)
            {
                gcmRELEASE_NAME(Device->externalPhysName);
                Device->externalPhysName = 0;
            }
            if (Device->contiguousPhysName != 0)
            {
                gcmRELEASE_NAME(Device->contiguousPhysName);
                Device->contiguousPhysName = 0;
            }
            if (Device->exclusivePhysName != 0)
            {
                gcmRELEASE_NAME(Device->exclusivePhysName);
                Device->exclusivePhysName = 0;
            }
        }

        /* Destroy per-core SRAM heap. */
        for (i = 0; i < gcvCORE_COUNT; i++)
        {
            if (Device->kernels[i])
            {
                kernel = Device->kernels[i];

                for (j = gcvSRAM_INTERNAL0; j < gcvSRAM_INTER_COUNT; j++)
                {
                    if (kernel->sRAMPhysical[j] != gcvNULL)
                    {
                        /* Release reserved SRAM memory. */
                        gckOS_ReleaseReservedMemory(
                            Device->os,
                            kernel->sRAMPhysical[j]
                            );

                        kernel->sRAMPhysical[j] = gcvNULL;
                    }

                    if (kernel->sRAMVidMem[j] != gcvNULL)
                    {
                        /* Destroy the SRAM contiguous heap. */
                        gcmkVERIFY_OK(gckVIDMEM_Destroy(kernel->sRAMVidMem[j]));
                        kernel->sRAMVidMem[j] = gcvNULL;
                    }
                }
            }
        }

        if (Device->device)
        {
            gcmkVERIFY_OK(gckDEVICE_Destroy(Device->os, Device->device));

            for (i = 0; i < gcdMAX_GPU_COUNT; i++)
            {
                if (globalTA[i])
                {
                    gcTA_Destroy(globalTA[i]);
                    globalTA[i] = gcvNULL;
                }
            }

            Device->device = gcvNULL;
        }

        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->kernels[i] != gcvNULL)
            {
                Device->kernels[i] = gcvNULL;
            }
        }

        if (Device->internalLogical != gcvNULL)
        {
            /* Unmap the internal memory. */
            iounmap(Device->internalLogical);
            Device->internalLogical = gcvNULL;
        }

        if (Device->internalVidMem != gcvNULL)
        {
            /* Destroy the internal heap. */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->internalVidMem));
            Device->internalVidMem = gcvNULL;
        }

        for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
        {
            if (Device->extSRAMPhysical[i] != gcvNULL)
            {
                gckOS_ReleaseReservedMemory(
                    Device->os,
                    Device->extSRAMPhysical[i]
                    );
                Device->extSRAMPhysical[i] = gcvNULL;
            }

            if (Device->extSRAMVidMem[i] != gcvNULL)
            {
                gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->extSRAMVidMem[i]));
                Device->extSRAMVidMem[i] = gcvNULL;
            }
        }

        if (Device->externalPhysical != gcvNULL)
        {
            gckOS_ReleaseReservedMemory(
                Device->os,
                Device->externalPhysical
                );
            Device->externalPhysical = gcvNULL;
        }

        if (Device->externalLogical != gcvNULL)
        {
            Device->externalLogical = gcvNULL;
        }

        if (Device->externalVidMem != gcvNULL)
        {
            /* destroy the external heap */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->externalVidMem));
            Device->externalVidMem = gcvNULL;
        }

        if (Device->exclusivePhysical != gcvNULL)
        {
            gckOS_ReleaseReservedMemory(
                Device->os,
                Device->exclusivePhysical
                );
            Device->exclusivePhysical = gcvNULL;
        }

        if (Device->exclusiveLogical != gcvNULL)
        {
            Device->exclusiveLogical = gcvNULL;
        }

        if (Device->exclusiveVidMem != gcvNULL)
        {
            /* destroy the external heap */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->exclusiveVidMem));
            Device->exclusiveVidMem = gcvNULL;
        }

        /*
         * Destroy contiguous memory pool after gckDEVICE destroyed. gckDEVICE
         * may allocates GPU memory types from SYSTEM pool.
         */
        if (Device->contiguousPhysical != gcvNULL)
        {
            if (Device->requestedContiguousBase == 0)
            {
                gcmkVERIFY_OK(_FreeMemory(
                    Device,
                    Device->contiguousLogical,
                    Device->contiguousPhysical
                    ));
            }
            else
            {
                gckOS_ReleaseReservedMemory(
                    Device->os,
                    Device->contiguousPhysical
                    );
                Device->contiguousPhysical = gcvNULL;
                Device->requestedContiguousBase = 0;
                Device->requestedContiguousSize = 0;
            }

            Device->contiguousLogical  = gcvNULL;
            Device->contiguousPhysical = gcvNULL;
        }

        if (Device->contiguousVidMem != gcvNULL)
        {
            /* Destroy the contiguous heap. */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->contiguousVidMem));
            Device->contiguousVidMem = gcvNULL;
        }

        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->registerBases[i])
            {
                /* Unmap register memory. */
                if (Device->requestedRegisterMemBases[i] != 0)
                {
                    iounmap(Device->registerBases[i]);

                    release_mem_region(Device->requestedRegisterMemBases[i],
                            Device->requestedRegisterMemSizes[i]);
                }

                Device->registerBases[i] = gcvNULL;
                Device->requestedRegisterMemBases[i] = 0;
                Device->requestedRegisterMemSizes[i] = 0;
            }
        }


        if (Device->taos)
        {
            gcmkVERIFY_OK(gctaOS_DestroyOS(Device->taos));
            Device->taos = gcvNULL;
        }

        /* Destroy the gckOS object. */
        if (Device->os != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_Destroy(Device->os));
            Device->os = gcvNULL;
        }

        _DebugfsCleanup(Device);

        /* Free the device. */
        kfree(Device);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckGALDEVICE_Start
**
**  Start the gal device, including the following actions: setup the isr routine
**  and start the daemoni thread.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Start successfully.
*/
gceSTATUS
gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Device=%p", Device);

    /* Start the kernel threads. */
    for (i = 0; i < gcvCORE_COUNT; ++i)
    {
        if (i == gcvCORE_VG)
        {
            continue;
        }

        gcmkONERROR(_StartThread(Device, i));

#if gcdENABLE_SW_PREEMPTION
        gcmkONERROR(_StartPreemptThread(Device, i));
#endif
    }

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        /* Setup the ISR routine. */
        gcmkONERROR(_SetupIsr(i));

        if (i == gcvCORE_VG)
        {
        }
        else
        {
            /* Switch to SUSPEND power state. */
            gcmkONERROR(gckHARDWARE_SetPowerState(
                Device->kernels[i]->hardware, gcvPOWER_OFF_BROADCAST
                ));

            gcmkONERROR(gckHARDWARE_StartTimerReset(Device->kernels[i]->hardware));
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Stop
**
**  Stop the gal device, including the following actions: stop the daemon
**  thread, release the irq.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Stop(
    gckGALDEVICE Device
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Device=%p", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        if (i == gcvCORE_VG)
        {
        }
        else
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Device->kernels[i]->hardware, gcvTRUE
                ));

            /* Switch to OFF power state. */
            gcmkONERROR(gckHARDWARE_SetPowerState(
                Device->kernels[i]->hardware, gcvPOWER_OFF
                ));

            gckHARDWARE_StartTimerReset(Device->kernels[i]->hardware);
        }

        /* Stop the ISR routine. */
        gcmkONERROR(_ReleaseIsr(i));

    }

    /* Stop the kernel thread. */
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        _StopThread(Device, i);
#if gcdENABLE_SW_PREEMPTION
        _StopPreemptThread(Device, i);
#endif
    }

OnError:
    gcmkFOOTER();
    return status;
}

