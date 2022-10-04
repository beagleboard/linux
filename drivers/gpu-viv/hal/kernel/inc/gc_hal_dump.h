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


#ifndef __gc_hal_dump_h_
#define __gc_hal_dump_h_

/*
    gcdDUMP_KEY

        Set this to a string that appears in 'cat /proc/<pid>/cmdline'. E.g. 'camera'.
        HAL will create dumps for the processes matching this key.
*/
#ifndef gcdDUMP_KEY
#   define gcdDUMP_KEY                          "process"
#endif

/*
    gcdDUMP_PATH

        The dump file location. Some processes cannot write to the sdcard.
        Try apps' data dir, e.g. /data/data/com.android.launcher
*/
#ifndef gcdDUMP_PATH
#if defined(ANDROID)
#   define gcdDUMP_PATH                         "/mnt/sdcard/"
#else
#   define gcdDUMP_PATH                         "./"
#endif
#endif

/*
    gcdDUMP_FILE_IN_KERNEL

        Default dump file for gcdDUMP_IN_KERNEL.
        The file will be writen globally in kernel side.
        Can be overwriten in runtime by debugfs:/gc/dump/dump_file

        2 pseudo files:
        [dmesg]:    means dump to kernel debug message.
        [ignored]:  means dump ignored, nothing will be dumpped.
 */
#ifndef gcdDUMP_FILE_IN_KERNEL
#  define gcdDUMP_FILE_IN_KERNEL                "[dmesg]"
#endif

/*
    gcdDUMP_VERIFY_PER_DRAW

        Sub feature of gcdDUMP.
        When set to 1, verify RT and images(if used) for every single draw to ease simulation debug.
        Only valid for ES3 driver for now.
*/
#ifndef gcdDUMP_VERIFY_PER_DRAW
#   define gcdDUMP_VERIFY_PER_DRAW              0
#endif

/* Standalone dump features below. */

/*
    gcdDUMP_FRAMERATE
        When set to a value other than zero, averaqe frame rate will be dumped.
        The value set is the starting frame that the average will be calculated.
        This is needed because sometimes first few frames are too slow to be included
        in the average. Frame count starts from 1.
*/
#ifndef gcdDUMP_FRAMERATE
#   define gcdDUMP_FRAMERATE                    0
#endif


/*
    gcdDUMP_FRAME_TGA

    When set to a value other than 0, a dump of the frame specified by the value,
    will be done into frame.tga. Frame count starts from 1.
 */
#ifndef gcdDUMP_FRAME_TGA
#   define gcdDUMP_FRAME_TGA                    0
#endif

/*
    gcdDUMP_AHB_ACCESS

        When set to 1, a dump of all AHB register access will be printed to kernel
        message.
*/
#ifndef gcdDUMP_AHB_ACCESS
#   define gcdDUMP_AHB_ACCESS                   0
#endif

#endif /* __gc_hal_dump_h_ */



