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


//#include "gc_hal_user_hardware_precomp.h"
//#include "gc_hal_user.h"
/*
**
*/
#if gcdRESET_USC1

#if gcdRESET_USC_C

#include "gc_feature_database.h"

#define USC_DEBUG 0

typedef enum _USC_NN_TYPE
{
    USC_NN_TYPE_V6,
    USC_NN_TYPE_V7,
    USC_NN_TYPE_V8,
}
USC_NN_TYPE;
gceSTATUS
_InitializeUSC_NNCommands(
    IN gckHARDWARE Hardware,
    IN USC_NN_TYPE hw_type,
    IN gctUINT32 kernelAddress,
    IN gctUINT32 inImageAddress,
    IN gctUINT32 outImageAddress,
    IN OUT gctUINT8_PTR data_type,
    IN OUT gctUINT32_PTR item_size,
    IN OUT gctUINT32_PTR core_count,
    IN OUT gctSIZE_T_PTR patchBufferSizes,
    IN OUT gctUINT32_PTR* nnCommands
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckOS os = Hardware->os;

    gctUINT32_PTR command = *nnCommands;

    gctUINT8 kernelDataType = *data_type;
    gctUINT8 inImageDataType = *data_type;
    gctUINT8 outImageDataType = *data_type;

    gctUINT32 kernelsPerCore = 1;

    gctUINT32 inImageXSize = 3;
    gctUINT32 inImageYSize = 2;

    gctUINT32 outImageXSize = 2;
    gctUINT32 outImageYSize = 1;
    gctUINT32 outImageZSize = 1;

    gctUINT32 kernelXYSize = 2;
    gctUINT32 kernelZSize = 1;

    gctUINT32 nn_layer_flush = 1, noZOffset = 0, size = gcmSIZEOF(gctUINT32) * ((hw_type == USC_NN_TYPE_V6)?16:32);
    gctUINT32 imageEndAddress = 2048, post_shift = 0, post_shift_bit56 = 0;
    gctUINT8 coefZP = 0, outputZP = 0;
    gcsFEATURE_DATABASE * db = (gcsFEATURE_DATABASE *)Hardware->featureDatabase;
    gctUINT32 config = 0, index = 0;
    gctUINT32 configuration[][3] = {
        /*item size, data type, core count*/
        {1, 0x2, db->NNCoreCount_INT8},
        {2, 0x4, db->NNCoreCount_INT16},
        {2, 0x1, db->NNCoreCount_FLOAT16},
        {2, 0x7, db->NNCoreCount_BFLOAT},
    };

    *patchBufferSizes = size;

    if (command == NULL)
    {
        gcmkONERROR(gckOS_Allocate(os, size, (gctPOINTER*)(&command)));
        gcmkONERROR(gckOS_ZeroMemory(command, size));

        *nnCommands = command;
    }

    gcmkASSERT(command != NULL);
    gcQueryFeatureDB(0, 0, 0, 0, 0);

    for (index = 0; index < gcmCOUNTOF(configuration); index ++)
    {
        if(configuration[index][2] > 0)
            break;
    }

    if (index == gcmCOUNTOF(configuration))
         gcmkASSERT("Hardware not support NN!");
#if USC_DEBUG
    gcmkPRINT("The hardware(0x%0x) support %d\n", db->customerID, index);
    gcmkPRINT("\tcore int8  %d\n\tcore int16 %d\n\tcore fp16  %d\n\tcore bfp16 %d\n",
        db->NNCoreCount_INT8, db->NNCoreCount_INT16, db->NNCoreCount_FLOAT16, db->NNCoreCount_BFLOAT);

#endif
    *item_size = configuration [index][0];
    *data_type = (gctUINT8)configuration[index][1];
    *core_count = configuration[index][2];

    kernelDataType = *data_type;
    inImageDataType = *data_type;
    outImageDataType = *data_type;

    switch (hw_type)
    {
    case USC_NN_TYPE_V8:
        noZOffset = 1;
        outputZP = 0;
        post_shift = (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_FLOAT_POST_MULT))?0x1f:0;
        post_shift_bit56 = (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_FLOAT_POST_MULT))?3:0;

        break;
    case USC_NN_TYPE_V7:
    case USC_NN_TYPE_V6:
        post_shift = (*data_type == 0x2)?15:0;
        break;
    default:
        break;
    }

    /* gcregNNInstWord0 */
    gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (noZOffset) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:2) - (0 ?
 5:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:2) - (0 ?
 5:2) + 1))))))) << (0 ?
 5:2))) | (((gctUINT32) ((gctUINT32) (kernelXYSize) & ((gctUINT32) ((((1 ?
 5:2) - (0 ?
 5:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:2) - (0 ? 5:2) + 1))))))) << (0 ? 5:2)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:6) - (0 ?
 19:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:6) - (0 ?
 19:6) + 1))))))) << (0 ?
 19:6))) | (((gctUINT32) ((gctUINT32) ((kernelZSize & 0x3FFF)) & ((gctUINT32) ((((1 ?
 19:6) - (0 ?
 19:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:6) - (0 ? 19:6) + 1))))))) << (0 ? 19:6)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:20) - (0 ?
 26:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:20) - (0 ?
 26:20) + 1))))))) << (0 ?
 26:20))) | (((gctUINT32) ((gctUINT32) (kernelsPerCore) & ((gctUINT32) ((((1 ?
 26:20) - (0 ?
 26:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:20) - (0 ? 26:20) + 1))))))) << (0 ? 26:20)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 28:27) - (0 ?
 28:27) + 1))))))) << (0 ?
 28:27))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 28:27) - (0 ? 28:27) + 1))))))) << (0 ? 28:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) ((gctUINT32) (nn_layer_flush) & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31))));


    config =   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:6) - (0 ?
 18:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:6) - (0 ?
 18:6) + 1))))))) << (0 ?
 18:6))) | (((gctUINT32) ((gctUINT32) (inImageXSize) & ((gctUINT32) ((((1 ?
 18:6) - (0 ?
 18:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:6) - (0 ? 18:6) + 1))))))) << (0 ? 18:6)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:19) - (0 ?
 31:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:19) - (0 ?
 31:19) + 1))))))) << (0 ?
 31:19))) | (((gctUINT32) ((gctUINT32) (inImageYSize) & ((gctUINT32) ((((1 ?
 31:19) - (0 ?
 31:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:19) - (0 ? 31:19) + 1))))))) << (0 ? 31:19)))

                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (kernelDataType >> 1) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (inImageDataType >> 1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) ((gctUINT32) (outImageDataType >> 1) & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))

                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (kernelDataType & 0x1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (inImageDataType & 0x1) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (outImageDataType & 0x1) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));

    /* gcregNNInstWord1 */
    gcmkWRITE_MEMORY(command, config);

    /* gcregNNInstWord2 */
    gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 24:24) - (0 ?
 24:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 24:24) - (0 ?
 24:24) + 1))))))) << (0 ?
 24:24))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 24:24) - (0 ?
 24:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 24:24) - (0 ? 24:24) + 1))))))) << (0 ? 24:24)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:25) - (0 ?
 25:25) + 1))))))) << (0 ?
 25:25))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:25) - (0 ? 25:25) + 1))))))) << (0 ? 25:25)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:3) - (0 ?
 5:3) + 1))))))) << (0 ?
 5:3))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:3) - (0 ? 5:3) + 1))))))) << (0 ? 5:3)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:8) - (0 ?
 23:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:8) - (0 ?
 23:8) + 1))))))) << (0 ?
 23:8))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:8) - (0 ?
 23:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:8) - (0 ? 23:8) + 1))))))) << (0 ? 23:8)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) ((gctUINT32) (post_shift) & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))));

    /* gcregNNInstWord3 */
     gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:6) - (0 ?
 18:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:6) - (0 ?
 18:6) + 1))))))) << (0 ?
 18:6))) | (((gctUINT32) ((gctUINT32) (outImageXSize) & ((gctUINT32) ((((1 ?
 18:6) - (0 ?
 18:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:6) - (0 ? 18:6) + 1))))))) << (0 ? 18:6)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:19) - (0 ?
 31:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:19) - (0 ?
 31:19) + 1))))))) << (0 ?
 31:19))) | (((gctUINT32) ((gctUINT32) (outImageYSize) & ((gctUINT32) ((((1 ?
 31:19) - (0 ?
 31:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:19) - (0 ? 31:19) + 1))))))) << (0 ? 31:19)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3))));

    /* gcregNNInstWord4 */
    gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:0) - (0 ?
 13:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:0) - (0 ?
 13:0) + 1))))))) << (0 ?
 13:0))) | (((gctUINT32) ((gctUINT32) (outImageZSize) & ((gctUINT32) ((((1 ?
 13:0) - (0 ?
 13:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:0) - (0 ? 13:0) + 1))))))) << (0 ? 13:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:14) - (0 ?
 15:14) + 1))))))) << (0 ?
 15:14))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 15:14) - (0 ?
 15:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:14) - (0 ? 15:14) + 1))))))) << (0 ? 15:14)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 24:18) - (0 ?
 24:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 24:18) - (0 ?
 24:18) + 1))))))) << (0 ?
 24:18))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 24:18) - (0 ?
 24:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 24:18) - (0 ? 24:18) + 1))))))) << (0 ? 24:18)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:25) - (0 ?
 31:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:25) - (0 ?
 31:25) + 1))))))) << (0 ?
 31:25))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 31:25) - (0 ?
 31:25) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:25) - (0 ? 31:25) + 1))))))) << (0 ? 31:25)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) ((0 >> 3) & 0x1) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) ((0 >> 3) & 0x1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17))));


    /* gcregNNInstWord5 */
    gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:26) - (0 ?
 31:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:26) - (0 ?
 31:26) + 1))))))) << (0 ?
 31:26))) | (((gctUINT32) ((gctUINT32) (((kernelZSize >> 14) & 0x3F)) & ((gctUINT32) ((((1 ?
 31:26) - (0 ?
 31:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:26) - (0 ? 31:26) + 1))))))) << (0 ? 31:26)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) ((kernelAddress >> 6)) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0))));

    /* gcregNNInstWord6 */
    gcmkWRITE_MEMORY(command, inImageAddress);
    /* gcregNNInstWord7 */
    gcmkWRITE_MEMORY(command, outImageAddress);

    gcmkWRITE_MEMORY(command,
                     /* gcmSETFIELD (0, GCREG_NN_INST_WORD8, IMAGE_CACHING_MODE, imageCachingMode)
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:2) - (0 ?
 3:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:2) - (0 ?
 3:2) + 1))))))) << (0 ?
 3:2))) | (((gctUINT32) ((gctUINT32) (kernelCachingMode) & ((gctUINT32) ((((1 ?
 3:2) - (0 ?
 3:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:2) - (0 ? 3:2) + 1))))))) << (0 ? 3:2)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:4) - (0 ?
 5:4) + 1))))))) << (0 ?
 5:4))) | (((gctUINT32) ((gctUINT32) (partialCacheDataUnit) & ((gctUINT32) ((((1 ?
 5:4) - (0 ?
 5:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:4) - (0 ? 5:4) + 1))))))) << (0 ? 5:4)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:6) - (0 ?
 11:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:6) - (0 ?
 11:6) + 1))))))) << (0 ?
 11:6))) | (((gctUINT32) ((gctUINT32) (kernelPatternMsb) & ((gctUINT32) ((((1 ?
 11:6) - (0 ?
 11:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:6) - (0 ? 11:6) + 1))))))) << (0 ? 11:6)))
                    | */((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:12) - (0 ?
 15:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:12) - (0 ?
 15:12) + 1))))))) << (0 ?
 15:12))) | (((gctUINT32) ((gctUINT32) (kernelXYSize) & ((gctUINT32) ((((1 ?
 15:12) - (0 ?
 15:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:12) - (0 ? 15:12) + 1))))))) << (0 ? 15:12)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (outImageYSize) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16))));

    /* 31:0 */
    gcmkWRITE_MEMORY(command, 0);

    /* 31:0 */
    gcmkWRITE_MEMORY(command, 0);

    /* 31:0 */
    gcmkWRITE_MEMORY(command, 0);

    /* CREG_NN_INST_WORD12_KERNEL_CACHE_END_ADDRESS */
    gcmkWRITE_MEMORY(command, 0);

    /* 31:0 */
    gcmkWRITE_MEMORY(command, 0);

    /* GCREG_NN_INST_WORD14, IMAGE_END_ADDRESS */
    gcmkWRITE_MEMORY(command, imageEndAddress);


    /*1:0*/
    gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:2) - (0 ?
 17:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:2) - (0 ?
 17:2) + 1))))))) << (0 ?
 17:2))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 17:2) - (0 ?
 17:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:2) - (0 ? 17:2) + 1))))))) << (0 ? 17:2)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (kernelDataType >> 2) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:20) - (0 ?
 20:20) + 1))))))) << (0 ?
 20:20))) | (((gctUINT32) ((gctUINT32) (inImageDataType >> 2) & ((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 20:20) - (0 ? 20:20) + 1))))))) << (0 ? 20:20)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:21) - (0 ?
 21:21) + 1))))))) << (0 ?
 21:21))) | (((gctUINT32) ((gctUINT32) (outImageDataType >> 2) & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 21:21) - (0 ? 21:21) + 1))))))) << (0 ? 21:21)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 27:22) - (0 ?
 27:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 27:22) - (0 ?
 27:22) + 1))))))) << (0 ?
 27:22))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 27:22) - (0 ?
 27:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 27:22) - (0 ? 27:22) + 1))))))) << (0 ? 27:22)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:28) - (0 ?
 29:28) + 1))))))) << (0 ?
 29:28))) | (((gctUINT32) ((gctUINT32) (post_shift_bit56) & ((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:28) - (0 ? 29:28) + 1))))))) << (0 ? 29:28))));

    /* V7 or V8 */
    if (hw_type == USC_NN_TYPE_V7 || hw_type == USC_NN_TYPE_V8)
    {
        /*GCREG_NN_INST_WORD16*/
        gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (inImageXSize * (*item_size)) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (inImageYSize) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16))));

        /*GCREG_NN_INST_WORD17*/
        gcmkWRITE_MEMORY(command, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (outImageXSize * (*item_size)) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))));


        /*GCREG_NN_INST_WORD18*/
        gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) (0 >> 6) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) ((0 >> 4) & 0x1) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) ((0 >> 4) & 0x1) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:28) - (0 ?
 28:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 28:28) - (0 ?
 28:28) + 1))))))) << (0 ?
 28:28))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 28:28) - (0 ?
 28:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 28:28) - (0 ? 28:28) + 1))))))) << (0 ? 28:28)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) ((gctUINT32) (kernelDataType >> 3) & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31))));

        /*25:0*/
         gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) (0xFFFFFFFF >> 6) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) ((gctUINT32) (inImageDataType >> 3) & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31))));

        /*GCREG_NN_INST_WORD20*/
        gcmkWRITE_MEMORY(command, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) (0 >> 6) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) ((gctUINT32) (outImageDataType >> 3) & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26))));

        /*25:0*/
        gcmkWRITE_MEMORY(command, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:0) - (0 ?
 25:0) + 1))))))) << (0 ?
 25:0))) | (((gctUINT32) ((gctUINT32) (0xFFFFFFFF >> 6) & ((gctUINT32) ((((1 ?
 25:0) - (0 ?
 25:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:0) - (0 ? 25:0) + 1))))))) << (0 ? 25:0))));

        /*GCREG_NN_INST_WORD22*/
        gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (coefZP) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (outputZP) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:18) - (0 ?
 25:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:18) - (0 ?
 25:18) + 1))))))) << (0 ?
 25:18))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 25:18) - (0 ?
 25:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:18) - (0 ? 25:18) + 1))))))) << (0 ? 25:18))));

        /*GCREG_NN_INST_WORD23*/
        gcmkWRITE_MEMORY(command, 0);

        /*GCREG_NN_INST_WORD24*/
        gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:0) - (0 ?
 3:0) + 1))))))) << (0 ?
 3:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:0) - (0 ? 3:0) + 1))))))) << (0 ? 3:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:4) - (0 ?
 31:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:4) - (0 ?
 31:4) + 1))))))) << (0 ?
 31:4))) | (((gctUINT32) ((gctUINT32) (0 >> 4) & ((gctUINT32) ((((1 ?
 31:4) - (0 ?
 31:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:4) - (0 ? 31:4) + 1))))))) << (0 ? 31:4))));

        /*GCREG_NN_INST_WORD25*/
        gcmkWRITE_MEMORY(command,
                      ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:0) - (0 ?
 3:0) + 1))))))) << (0 ?
 3:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:0) - (0 ? 3:0) + 1))))))) << (0 ? 3:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:4) - (0 ?
 31:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:4) - (0 ?
 31:4) + 1))))))) << (0 ?
 31:4))) | (((gctUINT32) ((gctUINT32) (0 >> 4) & ((gctUINT32) ((((1 ?
 31:4) - (0 ?
 31:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:4) - (0 ? 31:4) + 1))))))) << (0 ? 31:4))));
    }

OnError:
    return status;
}

gceSTATUS
_BitValue(
    IN gctUINT8_PTR* base,
    IN gctUINT32 value,
    IN gctUINT32_PTR offset,
    IN gctUINT length)
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32_PTR msb = (gctUINT32_PTR)(*base) + 1, lsb = (gctUINT32_PTR)(*base);

    gcmkASSERT(*offset <= 32 && length <= 32);

    if ((*offset) < 32)
    {
        gctUINT32 end = (*offset) + length, data = *lsb;

        if (end < 32)
        {
            /************************************************************************
             *       offset    32           64                                      *
             *     _________________________                                        *
             *    |_____|////|_|____________|                                       *
             *              end                                                     *
             ************************************************************************/
            data  = (*lsb & ((1 << *offset) - 1));
            data |= (*lsb & ~((1 << end) - 1));
            data |= (value << *offset);

            *lsb = data;
            *offset = end;
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
            gctUINT32 data_l = (*lsb & ((1 << *offset) - 1));
            gctUINT32 data_m = (*msb & ~((1 << length_m) - 1));

            data_l |= (value << *offset);
            data_m |= (value >> length_m);

            *lsb = data_l;

            if (end > 32)
                *msb = data_m;

            *offset = length_m;

            *base = (gctUINT8_PTR)msb;
        }

    }

    return status;
}

gceSTATUS
_InitializeUSC_NNKernel(
    IN gckHARDWARE Hardware,
    IN USC_NN_TYPE hw_type,
    IN gctUINT8 data_type,
    IN gctUINT32 item_size,
    IN gctUINT32 core_count,
    IN OUT gctUINT32_PTR nnKernels
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT8_PTR kernels = (gctUINT8_PTR)nnKernels;
    gctUINT8_PTR kernel_stream_size_ptr = 0;
    gctUINT32 filterTotalCount = 1, filterSize = 2 * 2 * 1 * item_size, biasSize = 4;

    /* v8 huffman encoder */
    if (USC_NN_TYPE_V8 == hw_type)
    {
        gctUINT32 i = 0, offset = 0;
                          /*uint8, fp16, int8, uint16, int16, uint4, int4, bf16*/
        gctUINT8 rlt[][18] = {{0}, {1, 1, 0, 1}, {7, 1}, {0}, {3, 1, 0, 1}, {0}, {0}, {1, 1, 0, 1} };
        gctUINT8 map[][9] = {
            {1, 8, 7, 0, 4, 5, 6, 2, 3},
            {1, 5, 0, 7, 8, 2, 6, 3, 4},
            {1, 0, 7, 8, 4, 5, 6, 2, 3},
        };
        gctBOOL bit16 = (data_type == 0x4) || data_type == (0x1) || data_type == (0x7);
        gctBOOL fp16 = (data_type == 0x1);
        gctUINT32 index = (data_type == 0x1)?1:((data_type == 0x7)?2:0);

        gcmkONERROR(_BitValue(&kernels, 0, &offset, 1));/*precode*/
        gcmkONERROR(_BitValue(&kernels, bit16, &offset, 1));/*bit16*/
        gcmkONERROR(_BitValue(&kernels, fp16, &offset, 1));/*fp16*/
        gcmkONERROR(_BitValue(&kernels, 0, &offset, 1));/*reserve*/
        gcmkONERROR(_BitValue(&kernels, 1, &offset, 4));/*version, 1*/
        gcmkONERROR(_BitValue(&kernels, 4, &offset, 8));/*zero run length size*/

        for(i = 0; i < 18; i++)
            gcmkONERROR(_BitValue(&kernels, rlt[data_type][i], &offset, 8));/*zero run length x 18*/

        for(i = 0; i < 4; i++)
        {
            gcmkONERROR(_BitValue(&kernels, (map[index][2 * i + 1] << 4) + map[index][2 * i], &offset, 8));/*map x 4*/
        }

        gcmkONERROR(_BitValue(&kernels, 0, &offset, 16));/*avg bias*/

        gcmkONERROR(_BitValue(&kernels, 0, &offset, 16));/*reserved, must zero*/

        kernel_stream_size_ptr = kernels;
        for (i = 0; i < core_count; i ++)
            gcmkONERROR(_BitValue(&kernels, 0, &offset, 32));/*stream size*/

        kernels = (gctUINT8_PTR)nnKernels + gcmALIGN_NP2((gctUINT32)((gctUINT32_PTR)kernels - nnKernels), 64);

        switch (data_type)
        {
            case 0x4:
                gcmkONERROR(_BitValue(&kernels, 0x04058000, &offset, 32));/*huffman data*/ /*00000018 00924600*/
                gcmkONERROR(_BitValue(&kernels, 0x640101fc, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x00001200, &offset, 32));/*huffman data*/

                gcmkONERROR(_BitValue(&kernel_stream_size_ptr, 0x0000006d, &offset, 32));/*only on core, stream size*/


                break;
            case 0x0:
            case 0x2:
                gcmkONERROR(_BitValue(&kernels, 0xec000038, &offset, 32));/*huffman data*/

                gcmkONERROR(_BitValue(&kernel_stream_size_ptr, 0x35, &offset, 32));/*only on core, stream size*/

                break;
            case 0x1:
                gcmkONERROR(_BitValue(&kernels, 0x0009db68, &offset, 32));/*huffman data*/ /*0009db68 000006c0 000001f0 00000900 00024000*/
                gcmkONERROR(_BitValue(&kernels, 0x000006c0, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x000001f0, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x00000900, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x00024000, &offset, 32));/*huffman data*/

                gcmkONERROR(_BitValue(&kernel_stream_size_ptr, 0x000000a3, &offset, 32));/*only on core, stream size*/

                break;
            case 0x7:
                gcmkONERROR(_BitValue(&kernels, 0x0007fff8, &offset, 32));/*huffman data*/ /*0007fff8 7f00fdfc c0397f00 0900001f 40000000 00000002*/
                gcmkONERROR(_BitValue(&kernels, 0x7f00fdfc, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0xc0397f00, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x0900001f, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x40000000, &offset, 32));/*huffman data*/
                gcmkONERROR(_BitValue(&kernels, 0x00000002, &offset, 32));/*huffman data*/

                gcmkONERROR(_BitValue(&kernel_stream_size_ptr, 0x000000b2, &offset, 32));/*only on core, stream size*/

                break;
            default:
                gcmkASSERT("Huffman encode not support this format! Please check!");
                break;
        }


    }
    else
    {
        gctBOOL zero_all = gcvFALSE;
        gctUINT8 zrl = 0;
        gctUINT16 vznum = 1;
        gctUINT32 bias = 0;
        gctUINT32 total_size = gcmALIGN_NP2((filterTotalCount * (filterSize  +  biasSize +  3) + 3), 64);//1 + 2 + *item_size;;

        gckOS_ZeroMemory(kernels, total_size + 64);

        *((gctUINT32_PTR)kernels) = total_size;
        kernels += total_size;
        if (zero_all)
            *((gctUINT32_PTR)kernels) = (vznum << (8 * item_size));/*zrl & coreFilterCount, both compressed weight and bias are zero, the size(1 * 1 * 2 * 2 + 4 ) < 64, align to 64*/
        else
        {
            gctINT16 value = (data_type == 0x1)?0x3c00/*1.0f*/:1;
            gctUINT32 i = 0, offset = 0;

            _BitValue(&kernels, zrl, &offset, 8);
            _BitValue(&kernels, vznum, &offset, 16);
            _BitValue(&kernels, value, &offset, 8 * item_size);
            _BitValue(&kernels, bias, &offset, 32);
            if (data_type == 0x3 || data_type == 0x4)
                _BitValue(&kernels, 0, &offset, 16);

            for (i = 1; i < filterSize/item_size; i ++)
                _BitValue(&kernels, value, &offset, 8 * item_size);
        }

    }
OnError:
    return status;
}
static gceSTATUS _InitializeUSC_NNCmdBuffer(IN gckHARDWARE    Hardware,
                                            IN USC_NN_TYPE    hw_type,
                                            IN gctUINT32_PTR  flushCommands,
                                            IN gctUINT32      CmdAddress,
                                            IN gctUINT32      SramRemapAddress,
                                            OUT gctUINT32_PTR Bytes)
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsFEATURE_DATABASE *db = (gcsFEATURE_DATABASE *)(Hardware->featureDatabase);
    gctUINT32 idx = 0, kernel_brust_size = db->DDR_KERNEL_BURST_SIZE;
    gctINT32 disableZDPN = 1, disableSWTiling = 1, smallBatch = 1, ddrBurstSize = 0;
    gctBOOL enableNNStride = gcvFALSE;

    disableZDPN = (db->NN_ZDP3 || db->NN_ZDP6) ? 0 : 1;

    enableNNStride = db->NN_STRIDE_SUPPORT;
    disableSWTiling = enableNNStride ? 0 : 1;

    switch(kernel_brust_size)
    {
    case 256:
        ddrBurstSize = 0x2;
        break;
    case 64:
        ddrBurstSize = 0x0;
        break;
    default:
        break;
    }

    if (Hardware->identity.chipModel == 0x8000 && Hardware->identity.chipRevision == 0x7120 &&
            (Hardware->identity.customerID == 0x80 || Hardware->identity.customerID == 0x92))
        smallBatch = 0x0;
    else
        smallBatch = (db->NN_SMALLBATCH_PHASE1 && db->NN_COMMAND_KERNEL_REQUEST_CONFICT_FIX)
                     ? 0x0 : 0x1;

    if (hw_type == USC_NN_TYPE_V6)
    {
        flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x006B) & ((gctUINT32) ((((1 ?
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
        flushCommands[idx++] = 0;

        flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
        flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));
    }

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E4E) & ((gctUINT32) ((((1 ?
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
    flushCommands[idx++] = SramRemapAddress;

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E4F) & ((gctUINT32) ((((1 ?
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
    flushCommands[idx++] = 0x00000000;

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E50) & ((gctUINT32) ((((1 ?
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
    flushCommands[idx++] = 0x00000000;

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E4C) & ((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (disableZDPN) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (disableSWTiling) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (smallBatch) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:5) - (0 ?
 6:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:5) - (0 ?
 6:5) + 1))))))) << (0 ?
 6:5))) | (((gctUINT32) ((gctUINT32) (ddrBurstSize) & ((gctUINT32) ((((1 ?
 6:5) - (0 ?
 6:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:5) - (0 ? 6:5) + 1))))))) << (0 ? 6:5)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)));


    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E54) & ((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
          | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)));


    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0428) & ((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:6) - (0 ?
 31:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:6) - (0 ?
 31:6) + 1))))))) << (0 ?
 31:6))) | (((gctUINT32) ((gctUINT32) ((CmdAddress >> 6)) & ((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0429) & ((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = 0;

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

    flushCommands[idx++] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));

    *Bytes = idx * 4;

    /* Return the status. */
    return status;
}
#endif /*gcdRESET_USC_C*/

#endif /*gcdRESET_USC1*/


