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


#ifndef __gc_hal_options_h_
#define __gc_hal_options_h_

/*
    gcdSECURITY

*/
#ifndef gcdSECURITY
#   define gcdSECURITY                          0
#endif

/*
    gcdPRINT_VERSION

        Print HAL version.
*/
#ifndef gcdPRINT_VERSION
#   define gcdPRINT_VERSION                     0
#endif

/*
USE_KERNEL_VIRTUAL_BUFFERS

This define enables the use of VM for gckCommand and fence buffers.
*/
#ifndef USE_KERNEL_VIRTUAL_BUFFERS
#if defined(UNDER_CE)
#   define USE_KERNEL_VIRTUAL_BUFFERS           1
#else
#   define USE_KERNEL_VIRTUAL_BUFFERS           1
#endif
#endif

/*
    USE_NEW_LINUX_SIGNAL

        This define enables the Linux kernel signaling between kernel and user.
*/
#ifndef USE_NEW_LINUX_SIGNAL
#   define USE_NEW_LINUX_SIGNAL                 0
#endif

/*
    USE_LINUX_PCIE

        This define enables galcore as a Linux PCIE driver.
*/
#ifndef USE_LINUX_PCIE
#   define USE_LINUX_PCIE                       0
#endif

/*
    VIVANTE_PROFILER

        This define enables the profiler for hardware counters.
*/
#ifndef VIVANTE_PROFILER
#   define VIVANTE_PROFILER                     1
#endif

/*
    gcdUSE_VG

        Enable VG HAL layer (only for GC350).
*/
#ifndef gcdUSE_VG
#   define gcdUSE_VG                            0
#endif

/*
    gcdUSE_VX

        Enable VX HAL layer.
*/
#ifndef gcdUSE_VX
#   define gcdUSE_VX                            1
#endif

/*
    PROFILE_HAL_COUNTERS

        This define enables HAL counter profiling support.  HW and SHADER
        counter profiling depends on this.
*/
#ifndef PROFILE_HAL_COUNTERS
#   define PROFILE_HAL_COUNTERS                 1
#endif

/*
    PROFILE_HW_COUNTERS

        This define enables HW counter profiling support.
*/
#ifndef PROFILE_HW_COUNTERS
#   define PROFILE_HW_COUNTERS                  1
#endif

/*
    PROFILE_SHADER_COUNTERS

        This define enables SHADER counter profiling support.
*/
#ifndef PROFILE_SHADER_COUNTERS
#   define PROFILE_SHADER_COUNTERS              1
#endif

/*
    COMMAND_PROCESSOR_VERSION

        The version of the command buffer and task manager.
*/
#define COMMAND_PROCESSOR_VERSION               1

/*
    gcdDUMP

        Dump for hw capture.
        When set to 1, a dump of all states and memory uploads, as well as other
        hardware related execution will be printed to the debug console.  This
        data can be used for playing back applications.

        When set to 2, for vxc, all output memory will be dump.

        Please get tweak settings in gc_hal_dump.h.
*/
#ifndef gcdDUMP
#   define gcdDUMP                              0
#endif

/*
    gcdDUMP_IN_KERNEL

        Enhanced feature for hw capture capture.
        Required for MCFE.
        When set to 1, all dumps will happen in the kernel.  This is handy if
        you want the kernel to dump its command buffers as well and the data
        needs to be in sync.

        Dump in kernel implies kernel command dump.
        See debugfs:/gc/dump/ for runtime configuration.
*/
#ifndef gcdDUMP_IN_KERNEL
#   define gcdDUMP_IN_KERNEL                    0
#endif

/*
    gcdDUMP_TPNN_SUBCOMMAND

        Dump for TP/NN command buffer
        When set to 1, will dump TP/NN command buffer when GPU/VIP hang.
*/
#ifndef gcdDUMP_TPNN_SUBCOMMAND
#   define gcdDUMP_TPNN_SUBCOMMAND              1
#endif

/*
    gcdDUMP_2D

        Dump for 2D capture.
        When set to non-zero, it will dump the 2D command and surface.

        Please get tweak settings in gc_hal_dump.h.
*/
#ifndef gcdDUMP_2D
#   define gcdDUMP_2D                           0
#endif

/*
    gcdDUMP_API

        Dump driver level API.
        When set to 1, a high level dump of the EGL and GL/VG APs's are
        captured.

        Please get tweak settings in gc_hal_dump.h.
*/
#ifndef gcdDUMP_API
#   define gcdDUMP_API                          0
#endif

/*
    gcdDUMP_PER_OPERATION

        Operation based dump.

        Dump the block as below.
        1. Multiple operations belong to the same SW tiling block.
        2. Single operation which is NOT in any SW tiling block.
*/
#ifndef gcdDUMP_PER_OPERATION
#   define gcdDUMP_PER_OPERATION                0
#endif

/*
    gcdDEBUG_OPTION
        When set to 1, the debug options are enabled. We must set other MACRO to enable
        sub case.
*/
#ifndef gcdDEBUG_OPTION
#   define gcdDEBUG_OPTION                      0

#if gcdDEBUG_OPTION
/*
    gcdDEBUG_OPTION_KEY
        The process name of debug application.
*/
#ifndef gcdDEBUG_OPTION_KEY
#          define gcdDEBUG_OPTION_KEY                           "process"
#       endif
/*
    gcdDEBUG_OPTION_NO_GL_DRAWS
        When set to 1, all glDrawArrays and glDrawElements will be skip.
*/
#ifndef gcdDEBUG_OPTION_NO_GL_DRAWS
#           define gcdDEBUG_OPTION_NO_GL_DRAWS                  0
#       endif
/*
    gcdDEBUG_OPTION_NO_DRAW_PRIMITIVES
        When set to 1, all DrawPrimitives will be skip.
*/
#ifndef gcdDEBUG_OPTION_NO_DRAW_PRIMITIVES
#           define gcdDEBUG_OPTION_NO_DRAW_PRIMITIVES           0
#       endif
/*
    gcdDEBUG_OPTION_SKIP_SWAP
        When set to 1, just one out of gcdDEBUG_OPTION_SKIP_FRAMES(such as 1/10) eglSwapBuffers will be resolve,
        others skip.
*/
#ifndef gcdDEBUG_OPTION_SKIP_SWAP
#           define gcdDEBUG_OPTION_SKIP_SWAP                    0
#           define gcdDEBUG_OPTION_SKIP_FRAMES                  10
#       endif
/*
    gcdDEBUG_OPTION_FORCE_16BIT_RENDER_TARGET
        When set to 1, the format of render target will force to RGB565.
*/
#ifndef gcdDEBUG_OPTION_FORCE_16BIT_RENDER_TARGET
#           define gcdDEBUG_OPTION_FORCE_16BIT_RENDER_TARGET    0
#       endif
/*
    gcdDEBUG_OPTION_NONE_TEXTURE
        When set to 1, the type of texture will be set to 0x0.
*/
#ifndef gcdDEBUG_OPTION_NONE_TEXTURE
#           define gcdDEBUG_OPTION_NONE_TEXTURE                 0
#       endif
/*
    gcdDEBUG_OPTION_NONE_DEPTH
        When set to 1, the depth format of surface will be set to gcvSURF_UNKNOWN.
*/
#ifndef gcdDEBUG_OPTION_NONE_DEPTH
#           define gcdDEBUG_OPTION_NONE_DEPTH                   0
#       endif

/*
    gcdDEBUG_FORCE_CONTEXT_UPDATE
        When set to 1, context will be updated before every commit.
*/
#ifndef gcdDEBUG_FORCE_CONTEXT_UPDATE
#           define gcdDEBUG_FORCE_CONTEXT_UPDATE                0
#       endif


/*
    gcdDEBUG_OPTION_SPECIFY_POOL
        When set to 1, pool of each type surface can be specified by
        changing poolPerType[] in gcsSURF_NODE_Construct.
*/
#ifndef gcdDEBUG_OPTION_SPECIFY_POOL
#           define gcdDEBUG_OPTION_SPECIFY_POOL                 0
#       endif

#   endif
#endif

/*
    gcdENABLE_FSCALE_VAL_ADJUST
        When non-zero, FSCALE_VAL when gcvPOWER_ON can be adjusted externally.
 */
#ifndef gcdENABLE_FSCALE_VAL_ADJUST
#   define gcdENABLE_FSCALE_VAL_ADJUST          1
#endif

#ifndef gcdCAPTURE_ONLY_MODE
#   define gcdCAPTURE_ONLY_MODE                 0
#endif

/*
    gcdNULL_DRIVER

    Set to 1 for infinite speed hardware.
    Set to 2 for bypassing the HAL.
*/
#ifndef gcdNULL_DRIVER
#   define gcdNULL_DRIVER  0
#endif

/*
    gcdENABLE_TIMEOUT_DETECTION

        Enable timeout detection.
*/
#ifndef gcdENABLE_TIMEOUT_DETECTION
#   define gcdENABLE_TIMEOUT_DETECTION          0
#endif

/*
    gcdCMD_BUFFER_SIZE

        Number of bytes in a command buffer.
*/
#ifndef gcdCMD_BUFFER_SIZE
#if gcdCAPTURE_ONLY_MODE
#   define gcdCMD_BUFFER_SIZE                   (4 << 10)
#else
#   define gcdCMD_BUFFER_SIZE                   (128 << 10)
#endif
#endif

/*
    gcdCMD_BLT_BUFFER_SIZE

        Number of bytes in a command buffer.
*/
#ifndef gcdCMD_BLT_BUFFER_SIZE
#   define gcdCMD_BLT_BUFFER_SIZE                (1 << 10)
#endif

/*
    gcdCMD_BUFFERS

        Number of command buffers to use per client.
*/
#ifndef gcdCMD_BUFFERS
#if gcdCAPTURE_ONLY_MODE
#   define gcdCMD_BUFFERS                       1
#else
#   define gcdCMD_BUFFERS                       2
#endif
#endif

/*
    gcdMAX_CMD_BUFFERS

        Maximum number of command buffers to use per client.
*/
#ifndef gcdMAX_CMD_BUFFERS
#   define gcdMAX_CMD_BUFFERS                   8
#endif

/*
    gcdCOMMAND_QUEUES

        Number of command queues in the kernel.
*/
#ifndef gcdCOMMAND_QUEUES
#   define gcdCOMMAND_QUEUES                    2
#endif

/*
    gcdPOWER_CONTROL_DELAY

        The delay in milliseconds required to wait until the GPU has woke up
        from a suspend or power-down state.  This is system dependent because
        the bus clock also needs to stabalize.
*/
#ifndef gcdPOWER_CONTROL_DELAY
#   define gcdPOWER_CONTROL_DELAY               0
#endif

/*
    gcdMMU_SIZE

        Size of the MMU page table in bytes.  Each 4 bytes can hold 4kB worth of
        virtual data.
*/
#ifndef gcdMMU_SIZE
#   define gcdMMU_SIZE                          (256 << 10)
#endif

#ifndef gcdGC355_VGMMU_MEMORY_SIZE_KB
#   define gcdGC355_VGMMU_MEMORY_SIZE_KB   32
#endif

/*
    gcdREGISTER_READ_FROM_USER
    gcdREGISTER_WRITE_FROM_USER

        Set to 1 to allow IOCTL calls to get through from user land.  This
        should only be in debug or development drops.
*/
#ifndef gcdREGISTER_READ_FROM_USER
#   define gcdREGISTER_READ_FROM_USER           1
#endif

#ifndef gcdREGISTER_WRITE_FROM_USER
#   define gcdREGISTER_WRITE_FROM_USER          0
#endif

/*
    gcdHEAP_SIZE

        Set the allocation size for the internal heaps.  Each time a heap is
        full, a new heap will be allocated with this minmimum amount of bytes.
        The bigger this size, the fewer heaps there are to allocate, the better
        the performance.  However, heaps won't be freed until they are
        completely free, so there might be some more memory waste if the size is
        too big.
*/
#ifndef gcdHEAP_SIZE
#   define gcdHEAP_SIZE                         (64 << 10)
#endif

/*
    gcdPOWER_SUSPEND_WHEN_IDLE

        Set to 1 to make GPU enter gcvPOWER_SUSPEND when idle detected,
        otherwise GPU will enter gcvPOWER_IDLE.
*/
#ifndef gcdPOWER_SUSPEND_WHEN_IDLE
#   define gcdPOWER_SUSPEND_WHEN_IDLE          1
#endif

#ifndef gcdFPGA_BUILD
#   define gcdFPGA_BUILD                       0
#endif

/*
    gcdGPU_TIMEOUT

        This define specified the number of milliseconds the system will wait
        before it broadcasts the GPU is stuck.  In other words, it will define
        the timeout of any operation that needs to wait for the GPU.

        If the value is 0, no timeout will be checked for.
*/
#ifndef gcdGPU_TIMEOUT
#if gcdFPGA_BUILD
#   define gcdGPU_TIMEOUT                   2000000
#else
#   define gcdGPU_TIMEOUT                   20000
#endif
#endif

/*
    gcdGPU_2D_TIMEOUT

        This define specified the number of milliseconds the system will wait
        before it broadcasts the 2D GPU is stuck.  In other words, it will define
        the timeout of any operation that needs to wait for the GPU.

        If the value is 0, no timeout will be checked for.
*/
#ifndef gcdGPU_2D_TIMEOUT
#   define gcdGPU_2D_TIMEOUT                20000
#endif


/*
    gcdGPU_ADVANCETIMER

        it is advance timer.
*/
#ifndef gcdGPU_ADVANCETIMER
#   define gcdGPU_ADVANCETIMER                  250
#endif

/*
    gcdSTATIC_LINK

        This define disalbes static linking;
*/
#ifndef gcdSTATIC_LINK
#   define gcdSTATIC_LINK                       0
#endif

/*
    gcdUSE_NEW_HEAP

        Setting this define to 1 enables new heap.
*/
#ifndef gcdUSE_NEW_HEAP
#   define gcdUSE_NEW_HEAP                      0
#endif

/*
    gcdCMD_NO_2D_CONTEXT

        This define enables no-context 2D command buffer.
*/
#ifndef gcdCMD_NO_2D_CONTEXT
#   define gcdCMD_NO_2D_CONTEXT                 1
#endif

/*
    gcdENABLE_BUFFER_ALIGNMENT

    When enabled, video memory is allocated  with atleast 16KB aligment
    between multiple sub-buffers.
*/
#ifndef gcdENABLE_BUFFER_ALIGNMENT
#if gcdCAPTURE_ONLY_MODE
#   define gcdENABLE_BUFFER_ALIGNMENT             0
#else
#   define gcdENABLE_BUFFER_ALIGNMENT             1
#endif
#endif

/*
    gcdENABLE_BANK_ALIGNMENT

    When enabled, video memory is allocated bank aligned. The vendor can modify
    _GetSurfaceBankAlignment() and _GetBankOffsetBytes() to define how
    different types of allocations are bank and channel aligned.
    When disabled (default), no bank alignment is done.
    For CAPTURE ONLY MODE, should make sure that gcdENABLE_BANK_ALIGNMENT is disabled.
*/
#ifndef gcdENABLE_BANK_ALIGNMENT
#if gcdCAPTURE_ONLY_MODE
#   define gcdENABLE_BANK_ALIGNMENT             0
#else
#   define gcdENABLE_BANK_ALIGNMENT             0
#endif
#endif

/*
    gcdBANK_BIT_START

    Specifies the start bit of the bank (inclusive).
*/
#ifndef gcdBANK_BIT_START
#   define gcdBANK_BIT_START                    12
#endif

/*
    gcdBANK_BIT_END

    Specifies the end bit of the bank (inclusive).
*/
#ifndef gcdBANK_BIT_END
#   define gcdBANK_BIT_END                      14
#endif

/*
    gcdBANK_CHANNEL_BIT

    When set, video memory when allocated bank aligned is allocated such that
    render and depth buffer addresses alternate on the channel bit specified.
    This option has an effect only when gcdENABLE_BANK_ALIGNMENT is enabled.
    When disabled (default), no alteration is done.
*/
#ifndef gcdBANK_CHANNEL_BIT
#   define gcdBANK_CHANNEL_BIT                  7
#endif

/*
    gcdDYNAMIC_SPEED

        When non-zero, it informs the kernel driver to use the speed throttling
        broadcasting functions to inform the system the GPU should be spet up or
        slowed down. It will send a broadcast for slowdown each "interval"
        specified by this define in milliseconds
        (gckOS_BroadcastCalibrateSpeed).
*/
#ifndef gcdDYNAMIC_SPEED
#    define gcdDYNAMIC_SPEED                    2000
#endif

/*
    gcdDYNAMIC_EVENT_THRESHOLD

        When non-zero, it specifies the maximum number of available events at
        which the kernel driver will issue a broadcast to speed up the GPU
        (gckOS_BroadcastHurry).
*/
#ifndef gcdDYNAMIC_EVENT_THRESHOLD
#    define gcdDYNAMIC_EVENT_THRESHOLD          5
#endif

/*
    gcdENABLE_PROFILING

        Enable profiling macros.
*/
#ifndef gcdENABLE_PROFILING
#   define gcdENABLE_PROFILING                  0
#endif

/*
    gcdENABLE_128B_MERGE

        Enable 128B merge for the BUS control.
*/
#ifndef gcdENABLE_128B_MERGE
#   define gcdENABLE_128B_MERGE                 0
#endif

/*
    gcdFRAME_DB

        When non-zero, it specified the number of frames inside the frame
        database. The frame DB will collect per-frame timestamps and hardware
        counters.
*/
#ifndef gcdFRAME_DB
#   define gcdFRAME_DB                          0
#   define gcdFRAME_DB_RESET                    0
#   define gcdFRAME_DB_NAME                     "/var/log/frameDB.log"
#endif

/*
   gcdENABLE_CACHEABLE_COMMAND_BUFFER

        When non-zero, command buffer will be cacheable.
*/
#ifndef gcdENABLE_CACHEABLE_COMMAND_BUFFER
#   define gcdENABLE_CACHEABLE_COMMAND_BUFFER          0
#endif

/*
   gcdENABLE_BUFFERABLE_VIDEO_MEMORY

        When non-zero, all video memory will be bufferable by default.
*/
#ifndef gcdENABLE_BUFFERABLE_VIDEO_MEMORY
#   define gcdENABLE_BUFFERABLE_VIDEO_MEMORY           1
#endif

/*
    gcdENABLE_INFINITE_SPEED_HW
            enable the Infinte HW, this is for 2D openVG
*/
#ifndef gcdENABLE_INFINITE_SPEED_HW
#   define gcdENABLE_INFINITE_SPEED_HW          0
#endif

/*
    gcdPOWEROFF_TIMEOUT

        When non-zero, GPU will power off automatically from
        idle state, and gcdPOWEROFF_TIMEOUT is also the default
        timeout in milliseconds.
 */
#ifndef gcdPOWEROFF_TIMEOUT
#   define gcdPOWEROFF_TIMEOUT                  300
#endif

/*
    QNX_SINGLE_THREADED_DEBUGGING
*/
#ifndef QNX_SINGLE_THREADED_DEBUGGING
#   define QNX_SINGLE_THREADED_DEBUGGING        0
#endif

/*
    gcdSHARED_RESOLVE_BUFFER_ENABLED

        Use shared resolve buffer for all app buffers.
*/
#ifndef gcdSHARED_RESOLVE_BUFFER_ENABLED
#   define gcdSHARED_RESOLVE_BUFFER_ENABLED     0
#endif

/*
     gcdUSE_TRIANGLE_STRIP_PATCH
 */
#ifndef gcdUSE_TRIANGLE_STRIP_PATCH
#   define gcdUSE_TRIANGLE_STRIP_PATCH          1
#endif

/*
    gcdSHARED_PAGETABLE

        When non-zero, multiple GPUs in one chip with same MMU use
        one shared pagetable. So that when accessing same surface,
        they can use same GPU virtual address.
*/
#ifndef gcdSHARED_PAGETABLE
#   define gcdSHARED_PAGETABLE                  0
#endif

#ifndef gcdUSE_PVR
#   define gcdUSE_PVR                           1
#endif

/*
    gcdSMALL_BLOCK_SIZE

        When non-zero, a part of VIDMEM will be reserved for requests
        whose requesting size is less than gcdSMALL_BLOCK_SIZE.

        For Linux, it's the size of a page. If this requeset fallbacks
        to gcvPOOL_VIRTUAL, memory will be wasted
        because they allocate a page at least.
*/
#ifndef gcdSMALL_BLOCK_SIZE
#   define gcdSMALL_BLOCK_SIZE                  4096
#   define gcdRATIO_FOR_SMALL_MEMORY            32
#endif

/*
    gcdENABLE_VIRTUAL_ADDR_UNMAP
        enable virtual address unmap for the weight_bias and the virtual image
*/
#ifndef gcdENABLE_VIRTUAL_ADDRESS_UNMAP
#   define gcdENABLE_VIRTUAL_ADDRESS_UNMAP      0
#endif

/*
    gcdENABLE_GPU_1M_PAGE
        When non-zero, GPU page size will be 1M until the pool is out of memory
        and low-level to 4K pages. When zero, it uses 4k GPU pages.
*/
#ifndef gcdENABLE_GPU_1M_PAGE
#if !gcdSECURITY && defined(LINUX)
#ifdef EMULATOR
#   define gcdENABLE_GPU_1M_PAGE                0
#else
#   define gcdENABLE_GPU_1M_PAGE                1
#endif
#else
#   define gcdENABLE_GPU_1M_PAGE                0
#endif
#endif

/*
    gcdCONTIGUOUS_SIZE_LIMIT
        When non-zero, size of video node from gcvPOOL_VIRTUAL contiguous is
        limited by gcdCONTIGUOUS_SIZE_LIMIT.
*/
#ifndef gcdCONTIGUOUS_SIZE_LIMIT
#   define gcdCONTIGUOUS_SIZE_LIMIT             0
#endif

/*
    gcdLINK_QUEUE_SIZE

        When non-zero, driver maintains a queue to record information of
        latest lined context buffer and command buffer. Data in this queue
        is be used to debug.
*/
#ifndef gcdLINK_QUEUE_SIZE
#   define gcdLINK_QUEUE_SIZE                   64
#endif

/*  gcdALPHA_KILL_IN_SHADER

        Enable alpha kill inside the shader. This will be set automatically by the
        HAL if certain states match a criteria.
*/
#ifndef gcdALPHA_KILL_IN_SHADER
#   define gcdALPHA_KILL_IN_SHADER              1
#endif


#ifndef gcdPRINT_SWAP_TIME
#   define gcdPRINT_SWAP_TIME                   0
#endif

/*
    gcdDVFS

        When non-zero, software will make use of dynamic voltage and
        frequency feature.
 */
#ifndef gcdDVFS
#   define gcdDVFS                              0
#   define gcdDVFS_ANAYLSE_WINDOW               4
#   define gcdDVFS_POLLING_TIME                 (gcdDVFS_ANAYLSE_WINDOW * 4)
#endif

#ifndef gcdSYNC
#   define gcdSYNC                              1
#endif

#ifndef gcdSHADER_SRC_BY_MACHINECODE
#   define gcdSHADER_SRC_BY_MACHINECODE         1
#endif

#ifndef gcdGLB27_SHADER_REPLACE_OPTIMIZATION
#    define gcdGLB27_SHADER_REPLACE_OPTIMIZATION 1
#endif


/*
    gcdSUPPORT_SWAP_RECTANGLE

        Support swap with a specific rectangle.

        Set the rectangle with eglSetSwapRectangleVIV api.
        Android only.
*/
#ifndef gcdSUPPORT_SWAP_RECTANGLE
#   define gcdSUPPORT_SWAP_RECTANGLE            0
#endif

/*
    gcdGPU_LINEAR_BUFFER_ENABLED

        Use linear buffer for GPU apps so HWC can do 2D composition.
        Android only.
*/
#ifndef gcdGPU_LINEAR_BUFFER_ENABLED
#   define gcdGPU_LINEAR_BUFFER_ENABLED         1
#endif

/*
    gcdENABLE_RENDER_INTO_WINDOW

        Enable Render-Into-Window (ie, No-Resolve) feature on android.
        NOTE that even if enabled, it still depends on hardware feature and
        android application behavior. When hardware feature or application
        behavior can not support render into window mode, it will fail back
        to normal mode.
        When Render-Into-Window is finally used, window back buffer of android
        applications will be allocated matching render target tiling format.
        Otherwise buffer tiling is decided by the above option
        'gcdGPU_LINEAR_BUFFER_ENABLED'.
        Android only for now.
*/
#ifndef gcdENABLE_RENDER_INTO_WINDOW
#   define gcdENABLE_RENDER_INTO_WINDOW         1
#endif

/*
    gcdENABLE_RENDER_INTO_WINDOW_WITH_FC

        Enable Direct-rendering (ie, No-Resolve) with tile status.
        This is expremental and in development stage.
        This will dynamically check if color compression is available.
*/
#ifndef gcdENABLE_RENDER_INTO_WINDOW_WITH_FC
#   define gcdENABLE_RENDER_INTO_WINDOW_WITH_FC 0
#endif

/*
    gcdENABLE_BLIT_BUFFER_PRESERVE

        Render-Into-Window (ie, No-Resolve) does not include preserved swap
        behavior.  This feature can enable buffer preserve in No-Resolve mode.
        When enabled, previous buffer (may be part of ) will be resolve-blitted
        to current buffer.
*/
#ifndef gcdENABLE_BLIT_BUFFER_PRESERVE
#   define gcdENABLE_BLIT_BUFFER_PRESERVE       1
#endif

/*
    gcdANDROID_NATIVE_FENCE_SYNC

        Enable android native fence sync. It is introduced since jellybean-4.2.
        Depends on linux kernel option: CONFIG_SYNC.

        0: Disabled
        1: Build framework for native fence sync feature, and EGL extension
        2: Enable async swap buffers for client
           * Native fence sync for client 'queueBuffer' in EGL, which is
             'acquireFenceFd' for layer in compositor side.
        3. Enable async hwcomposer composition.
           * 'releaseFenceFd' for layer in compositor side, which is native
             fence sync when client 'dequeueBuffer'
           * Native fence sync for compositor 'queueBuffer' in EGL, which is
             'acquireFenceFd' for framebuffer target for DC
 */
#ifndef gcdANDROID_NATIVE_FENCE_SYNC
#   define gcdANDROID_NATIVE_FENCE_SYNC         0
#endif

#ifndef gcdLINUX_SYNC_FILE
#   define gcdLINUX_SYNC_FILE                   0
#endif

/*
    gcdANDROID_IMPLICIT_NATIVE_BUFFER_SYNC

        Enable implicit android native buffer sync.

        For non-HW_RENDER buffer, CPU (or other hardware) and GPU can access
        the buffer at the same time. This is to add implicit synchronization
        between CPU (or the hardware) and GPU.

        Eventually, please do not use implicit native buffer sync, but use
        "fence sync" or "android native fence sync" instead in libgui, which
        can be enabled in frameworks/native/libs/gui/Android.mk. This kind
        of synchronization should be done by app but not driver itself.

        Please disable this option when either "fence sync" or
        "android native fence sync" is enabled.
 */
#ifndef gcdANDROID_IMPLICIT_NATIVE_BUFFER_SYNC
#   define gcdANDROID_IMPLICIT_NATIVE_BUFFER_SYNC   1
#endif

/*
 * Implicit native buffer sync is not needed when ANDROID_native_fence_sync
 * is available.
 */
#if gcdANDROID_NATIVE_FENCE_SYNC
#   undef  gcdANDROID_IMPLICIT_NATIVE_BUFFER_SYNC
#   define gcdANDROID_IMPLICIT_NATIVE_BUFFER_SYNC   0
#endif

/*
    gcdUSE_WCLIP_PATCH

        Enable wclipping patch.
*/
#ifndef gcdUSE_WCLIP_PATCH
#   define gcdUSE_WCLIP_PATCH                   1
#endif

#ifndef gcdUSE_NPOT_PATCH
#   define gcdUSE_NPOT_PATCH                    1
#endif

/*
    gcdINTERNAL_COMMENT

        Wrap internal comment, content wrapped by it and the macor itself
        will be removed in release driver.
*/
#ifndef gcdINTERNAL_COMMENT
#   define gcdINTERNAL_COMMENT                  1
#endif

/*
    gcdRTT_DISABLE_FC

        Disable RTT FC support. For test only.
*/
#ifndef gcdRTT_DISABLE_FC
#   define gcdRTT_DISABLE_FC                    0
#endif

/*
    gcdFORCE_MIPMAP

        Force generate mipmap for texture.
*/
#ifndef gcdFORCE_MIPMAP
#   define gcdFORCE_MIPMAP                      0
#endif

/*
    gcdFORCE_BILINEAR

        Force bilinear for mipfilter.
*/
#ifndef gcdFORCE_BILINEAR
#   define gcdFORCE_BILINEAR                    1
#endif

/*
    gcdBINARY_TRACE

        When non-zero, binary trace will be generated.

        When gcdBINARY_TRACE_FILE_SIZE is non-zero, binary trace buffer will
        be written to a file which size is limited to
        gcdBINARY_TRACE_FILE_SIZE.
*/
#ifndef gcdBINARY_TRACE
#   define gcdBINARY_TRACE                       0
#   define gcdBINARY_TRACE_FILE_SIZE             0
#endif

#ifndef gcdMOVG
#   define gcdMOVG                              0
#   define gcdENABLE_TS_DOUBLE_BUFFER           1
#else
#if gcdMOVG
#   define gcdENABLE_TS_DOUBLE_BUFFER           0
#else
#       define gcdENABLE_TS_DOUBLE_BUFFER       1
#endif
#endif

/*  gcdINTERRUPT_STATISTIC
 *
 *  Monitor the event send to GPU and interrupt issued by GPU.
 */

#ifndef gcdINTERRUPT_STATISTIC
#if defined(LINUX) || defined(__QNXNTO__) || defined(UNDER_CE) || defined(__VXWORKS__)
#   define gcdINTERRUPT_STATISTIC               1
#else
#   define gcdINTERRUPT_STATISTIC               0
#endif
#endif

/*
    gcdFENCE_WAIT_LOOP_COUNT
        Wait fence, loop count.
*/
#ifndef gcdFENCE_WAIT_LOOP_COUNT
#   define gcdFENCE_WAIT_LOOP_COUNT 10000
#endif

/*
    gcdPARTIAL_FAST_CLEAR
        When it's not zero, partial fast clear is enabled.
        Depends on gcdHAL_3D_DRAWBLIT, if gcdHAL_3D_DRAWBLIT is not enabled,
        only available when scissor box is completely aligned.
        Expremental, under test only. Not ready for production.
*/
#ifndef gcdPARTIAL_FAST_CLEAR
#   define gcdPARTIAL_FAST_CLEAR                0
#endif

/*
    gcdREMOVE_SURF_ORIENTATION
        When it's not zero, we will remove surface orientation function.
        It wil become to a parameter of resolve function.
*/
#ifndef gcdREMOVE_SURF_ORIENTATION
#   define gcdREMOVE_SURF_ORIENTATION 1
#endif



/*
    gcdTEST_DEC200
        Test part for DEC200. Remove when release.
*/
#ifndef gcdTEST_DEC200
#   define gcdTEST_DEC200                       0
#endif

/*
    gcdPATTERN_FAST_PATH
         For pattern match
*/
#ifndef gcdPATTERN_FAST_PATH
#   define gcdPATTERN_FAST_PATH       1
#endif

/*
    gcdUSE_INPUT_DEVICE
         disable input devices usage under fb mode to support fb+vdk multi-process
*/
#ifndef gcdUSE_INPUT_DEVICE
#   define gcdUSE_INPUT_DEVICE        1
#endif

/*
    gcdPERFORMANCE_ANALYSIS

        When set to 1, driver will pass information through loadstate
        to HW. This loadstate does not impact HW execution.
*/
#ifndef gcdPERFORMANCE_ANALYSIS
#   define gcdPERFORMANCE_ANALYSIS              0
#endif

/*
    gcdFRAMEINFO_STATISTIC
        When enable, collect frame information.
*/
#ifndef gcdFRAMEINFO_STATISTIC

#if (defined(DBG) && DBG) || defined(DEBUG)                || \
     defined(_DEBUG) || gcdDUMP || gcdPERFORMANCE_ANALYSIS || \
     (defined(WIN32) && !defined(UNDER_CE))                || \
     gcdFPGA_BUILD

#   define gcdFRAMEINFO_STATISTIC      1
#else
#   define gcdFRAMEINFO_STATISTIC      0
#endif

#endif

/*
    gcdDEC_ENABLE_AHB
        Enable DEC300 compression AHB mode or not.
*/
#ifndef gcdDEC_ENABLE_AHB
#   define gcdDEC_ENABLE_AHB                    0
#endif

/*
    gcdENABLE_UNIFIED_CONSTANT
        Enable unified constant or not.
*/
#ifndef gcdENABLE_UNIFIED_CONSTANT
#   define gcdENABLE_UNIFIED_CONSTANT           1
#endif

/*
    Core configurations. By default enable all cores.
*/
#ifndef gcdENABLE_3D
#   define gcdENABLE_3D                         1
#endif

#ifndef gcdENABLE_2D
#   define gcdENABLE_2D                         1
#endif

#ifndef gcdENABLE_VG
#   define gcdENABLE_VG                         0
#endif

#ifndef gcdVG_ONLY
#   define  gcdVG_ONLY  (!gcdENABLE_3D && !gcdENABLE_2D && gcdENABLE_VG)
#endif

#if defined(WIN32) && !defined(UNDER_CE) && (gcdENABLE_VG == 1)

#ifdef gcdUSE_VX
#undef gcdUSE_VX
#endif

#ifdef COMMAND_PROCESSOR_VERSION
#undef  COMMAND_PROCESSOR_VERSION
#endif

#ifdef gcdENABLE_TRUST_APPLICATION
#undef  gcdENABLE_TRUST_APPLICATION
#endif

#ifdef gcdENABLE_3D
#undef  gcdENABLE_3D
#endif

#ifdef gcdENABLE_2D
#undef  gcdENABLE_2D
#endif

#define gcdENABLE_3D 0
#define gcdENABLE_2D 0
#define gcdUSE_VX 0
#define COMMAND_PROCESSOR_VERSION 2
#define gcdENABLE_TRUST_APPLICATION 0

#endif  /* Only for GC355 Cmodel build. */

#ifndef gcdGC355_PROFILER
#   define gcdGC355_PROFILER                    0
#endif

/*
    This definition must be paired with VIVANTE_PROFILER_SYSTEM_MEMORY
*/
#ifndef gcdGC355_MEM_PRINT
#   define gcdGC355_MEM_PRINT                      0
#else
#if (!((gcdENABLE_3D == 0) && (gcdENABLE_2D == 0) && (gcdENABLE_VG == 1)))
#      undef gcdGC355_MEM_PRINT
#      define gcdGC355_MEM_PRINT                   0
#   endif
#endif


/*
    gcdRECORD_COMMAND
*/
#ifndef gcdRECORD_COMMAND
#   define gcdRECORD_COMMAND                    0
#endif

/*
    gcdALLOC_CMD_FROM_RESERVE

    Provide a way by which location of command buffer can be
    specified. This is a DEBUG option to limit command buffer
    to some memory range.
*/
#ifndef gcdALLOC_CMD_FROM_RESERVE
#   define gcdALLOC_CMD_FROM_RESERVE            0
#endif

/*
    gcdBOUNDARY_CHECK

    When enabled, add bounary before and after a range of
    GPU address. So overflow can be trapped by MMU exception.
    This is a debug option for new MMU and gcdUSE_MMU_EXCEPTION
    is enabled.
*/
#ifndef gcdBOUNDARY_CHECK
#   define gcdBOUNDARY_CHECK                    0
#endif

/*
    gcdRENDER_QUALITY_CHECK

    When enabled, we disable performance opt patch
    to get know rendering quality comparing with other vendor.
*/
#ifndef gcdRENDER_QUALITY_CHECK
#   define gcdRENDER_QUALITY_CHECK              0
#endif

/*
    gcdSYSTRACE

    When enabled, we embed systrace in function header/footer
    to gather time information on linux platforms include android.
    '1' to trace API (EGL, ES11, ES2x, ES3x, etc)
    '2' to trace HAL (except compiler)
    '4' to trace HAL compiler
    See gc_hal_user_debug.c for more detailed trace zones.
*/
#ifndef gcdSYSTRACE
#   define gcdSYSTRACE                          0
#endif

#ifndef gcdENABLE_APPCTXT_BLITDRAW
#   define gcdENABLE_APPCTXT_BLITDRAW                     0
#endif

/*
    When enabled, use 1K mode for MMU version 2.0. otherwise use 4K mode.
*/
#ifndef gcdENABLE_MMU_1KMODE
#   define gcdENABLE_MMU_1KMODE                  1
#endif

/*
    gcdENABLE_TRUST_APPLICATION

    When enabled, trust application is used to handle 'security' registers.

    1) If HW doesn't have robust and security feature, this option is meaningless.
    2) If HW have robust and security and this option is not enable,
       security registers are handled by non secure driver. It is for
       platform doesn't want/need to use trust zone.
*/
#ifndef gcdENABLE_TRUST_APPLICATION
#if (defined(_WIN32) && !defined(UNDER_CE)) || (defined (LINUX) && !defined(EMULATOR))
#   define gcdENABLE_TRUST_APPLICATION          1
#else
#   define gcdENABLE_TRUST_APPLICATION          0
#endif
#endif

/* Disable gcdENABLE_TRUST_APPLICATION when oboslete gcdSECURITY enabled. */
#if gcdSECURITY
#undef gcdENABLE_TRUST_APPLICATION
#define gcdENABLE_TRUST_APPLICATION             0
#endif

#ifndef gcdMMU_SECURE_AREA_SIZE
#if defined(gcdENABLE_MMU_1KMODE)
#   define gcdMMU_SECURE_AREA_SIZE              32
#else
#   define gcdMMU_SECURE_AREA_SIZE              128
#endif
#endif

#ifndef gcdUSE_MMU_EXCEPTION
#   define gcdUSE_MMU_EXCEPTION                 1
#endif

#ifndef gcdVX_OPTIMIZER
#   define gcdVX_OPTIMIZER                      0
#endif

#ifndef gcdALLOC_ON_FAULT
#   define gcdALLOC_ON_FAULT                    0
#endif

/*
    gcdDISABLE_GPU_VIRTUAL_ADDRESS

        When enabled, disable MMU and all virtual allocated from MMU.
*/
#ifndef gcdDISABLE_GPU_VIRTUAL_ADDRESS
#   define gcdDISABLE_GPU_VIRTUAL_ADDRESS       0
#endif

/*
    gcd2D_COMPRESSION_DEC400_ALIGN_MODE

        Only for DEC400 compression.
        Set 0 as 16bytes aligned. 1 as 32bytes aligned. 2 as 64bytes aligned.
        Default is 0 which means 32bytes aligned.
*/
#ifndef gcd2D_COMPRESSION_DEC400_ALIGN_MODE
#   define gcd2D_COMPRESSION_DEC400_ALIGN_MODE  1
#endif

/*
    gcdENABLE_KERNEL_FENCE
        When enabled, use kernel fence to do resource tracking.
*/
#ifndef gcdENABLE_KENREL_FENCE
#   define gcdENABLE_KERNEL_FENCE               0
#endif

/*
    gcdUSE_VXC_BINARY
        When enabled, will use prebuilt shader binary in VX driver.
 */
#ifndef gcdUSE_VXC_BINARY
#   define gcdUSE_VXC_BINARY                    0
#endif

/*
    gcdFEATURE_SANITYCHECK
        When enabled, will do hardware feature sanity check, each
        used hardware feature should be printed out.
 */
#ifndef gcdFEATURE_SANITYCHECK
#   define gcdFEATURE_SANITYCHECK                0
#endif


/*
    VIVANTE_PROFILER_SYSTEM_MEMORY

        This define enables the profiling data for system memory allocated by driver
*/
#ifndef VIVANTE_PROFILER_SYSTEM_MEMORY
#   define VIVANTE_PROFILER_SYSTEM_MEMORY        1
#   define VP_MALLOC_OFFSET                     (16)

#endif

#define gcdHAL_TEST 1

/*
    gcdUSE_ZWP_SYNCHRONIZATION

        When enabled, will use the zwp_linux_surface_synchronization path,
        otherwise switch to old wayland path.
 */
#define gcdUSE_ZWP_SYNCHRONIZATION 1

/*
    gcdUSE_SINGLE_CONTEXT
        When enabled, will enable single context.
 */
#ifndef gcdUSE_SINGLE_CONTEXT
#   define gcdUSE_SINGLE_CONTEXT                   0
#endif

/*
    gcdKERNEL_QUERY_PERFORMANCE_COUNTER_V8
        When enabled, will enable query new performance counter of V8.0 in kernel
        space.
 */
#ifndef gcdKERNEL_QUERY_PERFORMANCE_COUNTER_V8
#   define gcdKERNEL_QUERY_PERFORMANCE_COUNTER_V8  0
#endif

/*
    gcdIGNORE_DRIVER_VERSIONS_MISMATCH
        When enabled, driver will ignore user and kernel driver version mismatch.
*/
#ifndef gcdIGNORE_DRIVER_VERSIONS_MISMATCH
#   define gcdIGNORE_DRIVER_VERSIONS_MISMATCH  0
#endif

/*
    gcdEXTERNAL_SRAM_USAGE
        '0': User driver queries the whole external SRAM and manages the memory.
             Or user driver dynamically allocate the external SRAM with pool type gcvPOOL_EXTERNAL_SRAM.

        '1': External SRAM only can be used for the initial command,
             but the external SRAM base and size must be set by customer.
             And it only can be used if pool type is gcvPOOL_EXTERNAL_SRAM when allocating video memory.

        '2': To be extended.
*/
#ifndef gcdEXTERNAL_SRAM_USAGE
#   define gcdEXTERNAL_SRAM_USAGE 0
#endif

/*
    gcdENABLE_SW_PREEMPTION
        Enable software preemption if set to 1, disable by default.
        Only support Linux OS currently.
*/
#ifndef gcdENABLE_SW_PREEMPTION
#   define gcdENABLE_SW_PREEMPTION 0
#endif

/*
    gcdSUPPORT_DEVICE_TREE_SOURCE
        To suppor device tree feature if set to 1, disable by default.
        Only works on linux OS.
*/
#ifndef gcdSUPPORT_DEVICE_TREE_SOURCE
#   define gcdSUPPORT_DEVICE_TREE_SOURCE        0
#endif

/*
    gcdENABLE_PER_DEVICE_PM
        Enable per device power management if set to 2, all the hardware cores will be one device.
        Enable per user device power management if set to 1, the brother cores of a device depends on user driver.
        Disable per device power mangement if set to 0.
        Only support Linux OS currently.
*/
#ifndef gcdENABLE_PER_DEVICE_PM
#   define gcdENABLE_PER_DEVICE_PM 0
#endif

/*
    gcdUSE_CAPBUF
 */
#ifndef gcdUSE_CAPBUF
#   define gcdUSE_CAPBUF 1
#endif

/*
    gcdENABLE_MP_SWITCH
        Enable multi-processor mode dynamic switch, the processor count is determined by specific conditions.
        Only support Linux OS currently.
*/
#ifndef gcdENABLE_MP_SWITCH
#   define gcdENABLE_MP_SWITCH 0
#endif

/*
    gcdANON_FILE_FOR_ALLOCATOR
        Enable this macro can replace the /dev/zero by anon_inode:[galcore] in /proc/<pid>/maps.
        Without the macro, run 'cat /proc/<pid>/maps' will print "/dev/zero".
*/
#ifndef gcdANON_FILE_FOR_ALLOCATOR
#   define gcdANON_FILE_FOR_ALLOCATOR 0
#endif

#endif /* __gc_hal_options_h_ */



