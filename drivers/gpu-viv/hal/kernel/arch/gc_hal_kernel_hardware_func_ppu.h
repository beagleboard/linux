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
#if gcdINITIALIZE_PPU

#define gcdRESET_PPU_SH         1


#define INPUT_PPU_IDX            0
#define OUTPUT_PPU_IDX            1
#define INST_PPU_IDX            2

#include "gc_feature_database.h"

#define GCREG_SH_INSTRUCTION_TYPE_INVALID ~0U

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
#define gcdVX_ENABLE4(X, Y, Z, W) ((1 << (X)) | (1 << (Y)) | (1 << (Z)) | (1 << (W)))
#define gcdVX_ENABLE1(X) (1 << (X))
#define gcdVX_ENABLE2(X, Y) ((1 << (X)) | (1 << (Y)))
#define gcdVX_ENABLE3(X, Y, Z) ((1 << (X)) | (1 << (Y)) | (1 << (Z)))
#define gcdVX_SWIZZLE (0 | (1 << 2) | (2 << 4) | (3 << 6))
#define gcdVX_SWIZZLE1(X) ((X) | ((X) << 2) | ((X) << 4) | ((X) << 6))
#define gcdVX_SWIZZLE2(X, Y) ((X) | ((Y) << 2) | ((Y) << 4) | ((Y) << 6))
#define gcdVX_SWIZZLE4(X, Y, Z, W) ((X) | ((Y) << 2) | ((Z) << 4) | ((W) << 6))


#define GETBIT(data, position) (((data) >> (position)) & 0x1)

#define SETBIT(data, position, value)   (\
            ((data) & (~((1ULL) << position)))   \
             |  \
             (((value) << position) & ((1ULL) << position))   \
             )

#define _START(reg_field)       (0 ? reg_field)
#define _END(reg_field)         (1 ? reg_field)
#define _GETSIZE(reg_field)     (_END(reg_field) - _START(reg_field) + 1)
#define _ALIGN(data, reg_field) ((gctUINT32)(data) << _START(reg_field))
#define _MASK(reg_field)        ((_GETSIZE(reg_field) == 32) \
                                    ?  ~0 \
                                    : (gctUINT32)(~((gctUINT64)(~0) << _GETSIZE(reg_field))))

#define AQSETFIELD(data, reg, field, value) \
(\
    ((gctUINT32)(data) & ~_ALIGN(_MASK(reg##_##field), reg##_##field)) \
        | \
    _ALIGN((gctUINT32)(value) & _MASK(reg##_##field), reg##_##field) \
)

#define AQSETFIELDVALUE(data, reg, field, value) \
(\
    ((gctUINT32)(data) & ~_ALIGN(_MASK(reg##_##field), reg##_##field)) \
        | \
    _ALIGN(reg##_##field##_##value & _MASK(reg##_##field), reg##_##field) \
)

gctUINT32 HwFunc_SETBITS(
    IN gctUINT32 Data,
    IN const unsigned int Start,
    IN const unsigned int End,
    IN const gctUINT32 Value
    )
{
    gctUINT32 data = Data;
    if (End >= Start)
    {
        gctUINT32 _Mask =  ((~0ULL >> (63 - End + Start)) << Start);
        data &= ~_Mask;
        data |= ((Value) << Start) & _Mask;
        return data;
    }
    else
    {
        gctUINT32 _Mask =  ((~0ULL >> (63 - Start + End)) << End);
        data &= ~_Mask;
        data |= ((Value) << End) & _Mask;
        return data;
    }
}

gctUINT32 HwFunc_GETBITS(
    IN gctUINT32 Data,
    IN const unsigned int Start,
    IN const unsigned int End
    )
{
    gctUINT32 data = Data;
    if (End >= Start)
    {
        gctUINT32 _Mask = (~0ULL >> (63 - (End - Start)));
        return (data >> Start) & _Mask;
    }
    else
    {
        gctUINT32 _Mask = (~0ULL >> (63 - (Start - End)));
        return (data >> End) & _Mask;;
    }
}

gceSTATUS
_InitializePPU_SetImmediate(
    IN gctUINT32                            Where,
    IN gctUINT32                            Value,
    IN gctUINT32                            Type,
    IN OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Where=0x%x", Where);

    switch (Where)
    {
        case 0:
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 20:12) - (0 ?
 20:12) + 1))))) << (0 ?
 20:12))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 8, 0)) & ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 20:12) - (0 ? 20:12) + 1))))) << (0 ? 20:12)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:22) - (0 ?
 29:22) + 1))))) << (0 ?
 29:22))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 16, 9)) & ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:22) - (0 ? 29:22) + 1))))) << (0 ? 29:22)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))) << (0 ?
 30:30))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 17)) & ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:30) - (0 ? 30:30) + 1))))) << (0 ? 30:30)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))) << (0 ?
 31:31))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 18)) & ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:31) - (0 ? 31:31) + 1))))) << (0 ? 31:31)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))) << (0 ?
 2:0))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 19) | (Type << 1)) & ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ? 2:0) + 1))))) << (0 ? 2:0)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:3) - (0 ?
 5:3) + 1))))) << (0 ?
 5:3))) | ((gctUINT32)((gctUINT32)(0x7) & ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:3) - (0 ? 5:3) + 1))))) << (0 ? 5:3)));
            break;

        case 1:
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:7) - (0 ?
 15:7) + 1))))) << (0 ?
 15:7))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 8, 0)) & ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:7) - (0 ? 15:7) + 1))))) << (0 ? 15:7)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 24:17) - (0 ?
 24:17) + 1))))) << (0 ?
 24:17))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 16, 9)) & ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 24:17) - (0 ? 24:17) + 1))))) << (0 ? 24:17)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 25:25) - (0 ?
 25:25) + 1))))) << (0 ?
 25:25))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 17)) & ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 25:25) - (0 ? 25:25) + 1))))) << (0 ? 25:25)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))) << (0 ?
 26:26))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 18)) & ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:26) - (0 ? 26:26) + 1))))) << (0 ? 26:26)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:27) - (0 ?
 29:27) + 1))))) << (0 ?
 29:27))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 19) | (Type << 1)) & ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:27) - (0 ? 29:27) + 1))))) << (0 ? 29:27)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))) << (0 ?
 2:0))) | ((gctUINT32)((gctUINT32)(0x7) & ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ? 2:0) + 1))))) << (0 ? 2:0)));
            break;

        case 2:
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:4) - (0 ?
 12:4) + 1))))) << (0 ?
 12:4))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 8, 0)) & ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:4) - (0 ? 12:4) + 1))))) << (0 ? 12:4)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:14) - (0 ?
 21:14) + 1))))) << (0 ?
 21:14))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Value, 16, 9)) & ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:14) - (0 ? 21:14) + 1))))) << (0 ? 21:14)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))) << (0 ?
 22:22))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 17)) & ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:22) - (0 ? 22:22) + 1))))) << (0 ? 22:22)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))) << (0 ?
 23:23))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 18)) & ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 23:23) - (0 ? 23:23) + 1))))) << (0 ? 23:23)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 27:25) - (0 ?
 27:25) + 1))))) << (0 ?
 27:25))) | ((gctUINT32)((gctUINT32)(GETBIT(Value, 19) | (Type << 1)) & ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 27:25) - (0 ? 27:25) + 1))))) << (0 ? 27:25)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))) << (0 ?
 30:28))) | ((gctUINT32)((gctUINT32)(0x7) & ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:28) - (0 ? 30:28) + 1))))) << (0 ? 30:28)));
            break;
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetInstructionType(
    IN gctUINT32                            Type,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Type=0x%x", Type);

    binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:21) - (0 ?
 21:21) + 1))))) << (0 ?
 21:21))) | ((gctUINT32)((gctUINT32)(GETBIT(Type, 0)) & ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:21) - (0 ? 21:21) + 1))))) << (0 ? 21:21)));
    binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 31:30) - (0 ?
 31:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:30) - (0 ?
 31:30) + 1))))) << (0 ?
 31:30))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Type, 2, 1)) & ((((1 ?
 31:30) - (0 ?
 31:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:30) - (0 ? 31:30) + 1))))) << (0 ? 31:30)));

    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_IsEndOfBB(
    IN gckHARDWARE                            Hardware,
    IN gctUINT32                            Opcode,
    OUT gctUINT32_PTR                        binary
)
{
    gceSTATUS status = gcvSTATUS_OK;
    gcmkHEADER_ARG("Opcode=0x%x", Opcode);

    if (binary != NULL)
    {
        if (((gcsFEATURE_DATABASE *)Hardware->featureDatabase)->SH_END_OF_BB)
        {
            switch (Opcode)
            {
            case 0x09:
            case 0x56:
            case 0x0A:
            case 0x0B:
            case 0x0F:
            case 0x31:
            case 0x10:
                binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 10:3) - (0 ?
 10:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:3) - (0 ?
 10:3) + 1))))) << (0 ?
 10:3))) | ((gctUINT32)((gctUINT32)(HwFunc_SETBITS(binary[1], 3, 3, 1)) & ((((1 ?
 10:3) - (0 ?
 10:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:3) - (0 ? 10:3) + 1))))) << (0 ? 10:3)));
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
                binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))) << (0 ?
 10:6))) | ((gctUINT32)((gctUINT32)(HwFunc_SETBITS(binary[0], 2, 2, 1)) & ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ? 10:6) + 1))))) << (0 ? 10:6)));
                break;
            case 0x32:
            case 0x39:
            case 0x33:
            case 0x3A:
            case 0x79:
            case 0x34:
            case 0x7A:
            case 0x35:
                binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))) << (0 ?
 10:6))) | ((gctUINT32)((gctUINT32)(HwFunc_SETBITS(binary[0], 2, 2, 1)) & ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ? 10:6) + 1))))) << (0 ? 10:6)));
                break;
            default:
                if (Opcode != 0x16 &&
                    Opcode != 0x24 &&
                    Opcode != 0x14 &&
                    Opcode != 0x15 &&
                    Opcode != 0x17)
                    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))) << (0 ?
 10:6))) | ((gctUINT32)((gctUINT32)(HwFunc_SETBITS(binary[0], 2, 2, 1)) & ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ? 10:6) + 1))))) << (0 ? 10:6)));
                break;
            }
        }
    }

//OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}
gceSTATUS
_InitializePPU_AddOpcode(
    IN gckHARDWARE                            Hardware,
    IN gctUINT32                            Opcode,
    IN gctUINT32                            Extended,
    IN gctINT32                             Type,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Opcode=0x%x", Opcode);

    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))) << (0 ?
 5:0))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Opcode, 5, 0)) & ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:0) - (0 ? 5:0) + 1))))) << (0 ? 5:0)));
    binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))) << (0 ?
 16:16))) | ((gctUINT32)((gctUINT32)(GETBIT(Opcode, 6)) & ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 16:16) - (0 ? 16:16) + 1))))) << (0 ? 16:16)));
    if (Opcode == 0x7F)
    {
        gcmkONERROR(_InitializePPU_SetImmediate(2, Extended, 0x2, binary));
    }
    else if (Opcode == 0x45)
    {
        binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 15:13) - (0 ?
 15:13) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:13) - (0 ?
 15:13) + 1))))) << (0 ?
 15:13))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Extended, 2, 0)) & ((((1 ?
 15:13) - (0 ?
 15:13) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:13) - (0 ? 15:13) + 1))))) << (0 ? 15:13)));
        binary[0] = SETBIT(binary[0], 31, GETBIT(Extended, 3));
        binary[1] = HwFunc_SETBITS(binary[1], 1, 0, HwFunc_GETBITS(Extended, 5, 4));
    }
    else if (Opcode == 0x31 || Opcode == 0x09 || Opcode == 0x0F)
    {
        binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ?
 10:6) + 1))))) << (0 ?
 10:6))) | ((gctUINT32)((gctUINT32)(HwFunc_GETBITS(Extended, 4, 0)) & ((((1 ?
 10:6) - (0 ?
 10:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 10:6) - (0 ? 10:6) + 1))))) << (0 ? 10:6)));
    }

    if((gctUINT32)Type != GCREG_SH_INSTRUCTION_TYPE_INVALID)
        gcmkONERROR(_InitializePPU_SetInstructionType(Type, binary));

    gcmkONERROR(_InitializePPU_IsEndOfBB(Hardware, Opcode, binary));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetDestination(
    IN gctUINT32                            Address,
    IN gctUINT32                            Enable,
    IN gctBOOL                              Saturate,
    IN OUT gctUINT32_PTR                    binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Address=0x%x", Address);

    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))) << (0 ?
 12:12))) | ((gctUINT32)((gctUINT32)(1) & ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:12) - (0 ? 12:12) + 1))))) << (0 ? 12:12)));
    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 22:16) - (0 ?
 22:16) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:16) - (0 ?
 22:16) + 1))))) << (0 ?
 22:16))) | ((gctUINT32)((gctUINT32)(Address) & ((((1 ?
 22:16) - (0 ?
 22:16) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:16) - (0 ? 22:16) + 1))))) << (0 ? 22:16)));
    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:23) - (0 ?
 26:23) + 1))))) << (0 ?
 26:23))) | ((gctUINT32)((gctUINT32)(Enable) & ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:23) - (0 ? 26:23) + 1))))) << (0 ? 26:23)));
    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))) << (0 ?
 11:11))) | ((gctUINT32)((gctUINT32)(Saturate) & ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 11:11) - (0 ? 11:11) + 1))))) << (0 ? 11:11)));

    gcmkFOOTER();
    return status;
}

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))

static gctUINT32 _InitializePPU_GetPixel(gctUINT32 format)
{
    gctUINT32 pixel = 0;
    switch(format)
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
_InitializePPU_SetEVIS(
    IN gctUINT32                            Start,
    IN gctUINT32                            End,
    IN gctUINT32                            Evis,
    IN OUT gctUINT32_PTR                    binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Evis=0x%x", Evis);

    binary[0] = (((gctUINT32)(binary[0]) & ~((gctUINT32)(((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:23) - (0 ?
 26:23) + 1))))) << (0 ?
 26:23))) | ((gctUINT32)((gctUINT32)(Start) & ((((1 ?
 26:23) - (0 ?
 26:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:23) - (0 ? 26:23) + 1))))) << (0 ? 26:23)));
    binary[0] = HwFunc_SETBITS(binary[0], 30, 27, End);
    binary[1] = HwFunc_SETBITS(binary[1], 10, 2, Evis);

    gcmkFOOTER();
    return status;
}

#define HwFunc_SETSOURCE(Where, Common_I, REL_ADR_I, TYPE_I, Address, Swizzle, Type, Negate, Absolute, Relative, Binary) \
    binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRCWhere_VALID, 1); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_ADR, Address); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_SWIZZLE, Swizzle); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_MODIFIER_NEG, Negate); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_MODIFIER_ABS, Absolute); \
            binary[REL_ADR_I] = AQSETFIELD(binary[REL_ADR_I], AQ_INST, SRC0_REL_ADR, Relative); \
            binary[TYPE_I] = AQSETFIELD(binary[TYPE_I], AQ_INST, SRC0_TYPE, Type);

gceSTATUS
_InitializePPU_SetSource(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Type,
    IN gctBOOL                              Negate,
    IN gctBOOL                              Absolute,
    IN gctUINT32                            Relative,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Where=0x%x", Where);
    switch (Where)
    {
        case 0:
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))) << (0 ?
 11:11))) | ((gctUINT32)((gctUINT32)(1) & ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 11:11) - (0 ? 11:11) + 1))))) << (0 ? 11:11)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 20:12) - (0 ?
 20:12) + 1))))) << (0 ?
 20:12))) | ((gctUINT32)((gctUINT32)(Address) & ((((1 ?
 20:12) - (0 ?
 20:12) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 20:12) - (0 ? 20:12) + 1))))) << (0 ? 20:12)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:22) - (0 ?
 29:22) + 1))))) << (0 ?
 29:22))) | ((gctUINT32)((gctUINT32)(Swizzle) & ((((1 ?
 29:22) - (0 ?
 29:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:22) - (0 ? 29:22) + 1))))) << (0 ? 29:22)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))) << (0 ?
 30:30))) | ((gctUINT32)((gctUINT32)(Negate) & ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:30) - (0 ? 30:30) + 1))))) << (0 ? 30:30)));
            binary[1] = (((gctUINT32)(binary[1]) & ~((gctUINT32)(((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))) << (0 ?
 31:31))) | ((gctUINT32)((gctUINT32)(Absolute) & ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 31:31) - (0 ? 31:31) + 1))))) << (0 ? 31:31)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))) << (0 ?
 2:0))) | ((gctUINT32)((gctUINT32)(Relative) & ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ? 2:0) + 1))))) << (0 ? 2:0)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:3) - (0 ?
 5:3) + 1))))) << (0 ?
 5:3))) | ((gctUINT32)((gctUINT32)(Type) & ((((1 ?
 5:3) - (0 ?
 5:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 5:3) - (0 ? 5:3) + 1))))) << (0 ? 5:3)));
            break;

        case 1:
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))) << (0 ?
 6:6))) | ((gctUINT32)((gctUINT32)(1) & ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 6:6) - (0 ? 6:6) + 1))))) << (0 ? 6:6)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:7) - (0 ?
 15:7) + 1))))) << (0 ?
 15:7))) | ((gctUINT32)((gctUINT32)(Address) & ((((1 ?
 15:7) - (0 ?
 15:7) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 15:7) - (0 ? 15:7) + 1))))) << (0 ? 15:7)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 24:17) - (0 ?
 24:17) + 1))))) << (0 ?
 24:17))) | ((gctUINT32)((gctUINT32)(Swizzle) & ((((1 ?
 24:17) - (0 ?
 24:17) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 24:17) - (0 ? 24:17) + 1))))) << (0 ? 24:17)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 25:25) - (0 ?
 25:25) + 1))))) << (0 ?
 25:25))) | ((gctUINT32)((gctUINT32)(Negate) & ((((1 ?
 25:25) - (0 ?
 25:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 25:25) - (0 ? 25:25) + 1))))) << (0 ? 25:25)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))) << (0 ?
 26:26))) | ((gctUINT32)((gctUINT32)(Absolute) & ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 26:26) - (0 ? 26:26) + 1))))) << (0 ? 26:26)));
            binary[2] = (((gctUINT32)(binary[2]) & ~((gctUINT32)(((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:27) - (0 ?
 29:27) + 1))))) << (0 ?
 29:27))) | ((gctUINT32)((gctUINT32)(Relative) & ((((1 ?
 29:27) - (0 ?
 29:27) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 29:27) - (0 ? 29:27) + 1))))) << (0 ? 29:27)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))) << (0 ?
 2:0))) | ((gctUINT32)((gctUINT32)(Type) & ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 2:0) - (0 ? 2:0) + 1))))) << (0 ? 2:0)));
            break;

        case 2:
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))) << (0 ?
 3:3))) | ((gctUINT32)((gctUINT32)(1) & ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 3:3) - (0 ? 3:3) + 1))))) << (0 ? 3:3)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:4) - (0 ?
 12:4) + 1))))) << (0 ?
 12:4))) | ((gctUINT32)((gctUINT32)(Address) & ((((1 ?
 12:4) - (0 ?
 12:4) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 12:4) - (0 ? 12:4) + 1))))) << (0 ? 12:4)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:14) - (0 ?
 21:14) + 1))))) << (0 ?
 21:14))) | ((gctUINT32)((gctUINT32)(Swizzle) & ((((1 ?
 21:14) - (0 ?
 21:14) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:14) - (0 ? 21:14) + 1))))) << (0 ? 21:14)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))) << (0 ?
 22:22))) | ((gctUINT32)((gctUINT32)(Negate) & ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 22:22) - (0 ? 22:22) + 1))))) << (0 ? 22:22)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))) << (0 ?
 23:23))) | ((gctUINT32)((gctUINT32)(Absolute) & ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 23:23) - (0 ? 23:23) + 1))))) << (0 ? 23:23)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 27:25) - (0 ?
 27:25) + 1))))) << (0 ?
 27:25))) | ((gctUINT32)((gctUINT32)(Relative) & ((((1 ?
 27:25) - (0 ?
 27:25) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 27:25) - (0 ? 27:25) + 1))))) << (0 ? 27:25)));
            binary[3] = (((gctUINT32)(binary[3]) & ~((gctUINT32)(((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:28) - (0 ?
 30:28) + 1))))) << (0 ?
 30:28))) | ((gctUINT32)((gctUINT32)(Type) & ((((1 ?
 30:28) - (0 ?
 30:28) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 30:28) - (0 ? 30:28) + 1))))) << (0 ? 30:28)));
            break;
    }
    gcmkFOOTER();
    return status;
}

static const gctUINT32 NEGATE_FLAG   = 1 << 0;
static const gctUINT32 ABSOLUTE_FLAG = 1 << 1;

gceSTATUS
_InitializePPU_SetUniform(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Modifiers,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate   = (Modifiers & NEGATE_FLAG  ) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;
    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, Address, Swizzle, 0x2, negate, absolute, 0, binary));

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetTempReg(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Modifiers,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate   = (Modifiers & NEGATE_FLAG  ) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;
    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, Address, Swizzle, 0x0, negate, absolute, 0, binary));

OnError:
    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_SetSourceBin(
    IN gctUINT32                            SourceBin,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("SourceBin=0x%x", SourceBin);

    binary[1] = HwFunc_SETBITS(binary[1], 25, 22, SourceBin);

    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_SetImmediateValue(
    IN gctUINT32                            Where,
    IN gctUINT32                            Value,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctINT32 raw = (((gctUINT32)(0) & ~((gctUINT32)(((((1 ?
 19:0) - (0 ?
 19:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 19:0) - (0 ?
 19:0) + 1))))) << (0 ?
 19:0))) | ((gctUINT32)((gctUINT32)(Value) & ((((1 ?
 19:0) - (0 ?
 19:0) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 19:0) - (0 ?
 19:0) + 1))))) << (0 ?
 19:0))) | (((gctUINT32)(0) & ~((gctUINT32)(((((1 ?
 21:20) - (0 ?
 21:20) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:20) - (0 ?
 21:20) + 1))))) << (0 ?
 21:20))) | ((gctUINT32)(0x2 & ((((1 ?
 21:20) - (0 ?
 21:20) + 1) == 32) ?
 ~0 : (gctUINT32)(~((gctUINT64)(~0) << ((1 ?
 21:20) - (0 ? 21:20) + 1))))) << (0 ? 21:20)));

    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, HwFunc_GETBITS(raw, 8, 0), HwFunc_GETBITS(raw, 16, 9), 0x7, GETBIT(raw, 17), GETBIT(raw, 18), HwFunc_GETBITS(raw, 21, 19), binary));

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS gcoHwFunc_SH_CMD(
    IN gckHARDWARE            Hardware,
    IN gctUINT32            Data_type,
    IN OUT gctUINT32_PTR    binarys,
    OUT  gctUINT32_PTR        command_count,
    OUT  gctUINT32_PTR        reg_count
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 count = 0;
    gctUINT32 Input1 = Data_type;
    gctUINT32 Input2 = Data_type;
    gctUINT32 Output = Data_type;

    gcmkHEADER_ARG("binarys=0x%x", binarys);

    /* a. DP instruction with all bin */
    /* b. Store to 6 line which size are 64 bytes and flush out */

    /*img_load.u8 r1, c0, r0.xy*/
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, 0x79, 0, Input1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Input1), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    count += 4;

    /*img_load.u8 r2, c0, r0.xy */
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, 0x79, 0, Input2, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(2, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Input2), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    count += 4;

    /* dp2x8 r1, r1, r2, c3_512 */
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, 0x45, 0x0B, Output, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, 7, (Input1 | (Input2 << 3)), &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(0, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 2, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetSource (2, 2, gcdVX_SWIZZLE, 0x4, gcvFALSE, gcvFALSE, 0, &binarys[count]));
    count += 4;


    /*img_store.u8 r1, c2, r0.xy, r1*/
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, 0x7A, 0, Output, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Output), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(2, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    count += 4;

    *command_count = count;
    *reg_count = 0x3;
OnError:
    gcmkFOOTER();
    return status;
}

#endif /*gcdINITIALIZE_PPU*/


