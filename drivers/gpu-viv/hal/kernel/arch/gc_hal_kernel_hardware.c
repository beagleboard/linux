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

#include "gc_feature_database.h"
#include <gc_hal_kernel_debug.h>

#define _GC_OBJ_ZONE    gcvZONE_HARDWARE

typedef struct _gcsiDEBUG_REGISTERS * gcsiDEBUG_REGISTERS_PTR;
typedef struct _gcsiDEBUG_REGISTERS
{
    gctSTRING       module;
    gctUINT         index;
    gctUINT         shift;
    gctUINT         data;
    gctUINT         count;
    gctUINT32       pipeMask;
    gctUINT32       selectStart;
    gctBOOL         avail;
    gctBOOL         inCluster;
}
gcsiDEBUG_REGISTERS;

typedef struct _gcsFE_STACK
{
    gctSTRING       name;
    gctINT          count;
    gctUINT32       highSelect;
    gctUINT32       lowSelect;
    gctUINT32       linkSelect;
    gctUINT32       clear;
    gctUINT32       next;
}
gcsFE_STACK;

/******************************************************************************\
********************************* Support Code *********************************
\******************************************************************************/
static gctBOOL
_IsHardwareMatch(
    IN gckHARDWARE Hardware,
    IN gctINT32 ChipModel,
    IN gctUINT32 ChipRevision
    )
{
    return ((Hardware->identity.chipModel == ChipModel) &&
            (Hardware->identity.chipRevision == ChipRevision));
}

static gceSTATUS
_ResetGPU(
    IN gckHARDWARE Hardware,
    IN gckOS Os,
    IN gceCORE Core
    );

static void
_GetEcoID(
    IN gckHARDWARE Hardware,
    IN OUT gcsHAL_QUERY_CHIP_IDENTITY_PTR Identity
    )
{
    gcmkVERIFY_OK(gckOS_ReadRegisterEx(
        Hardware->os,
        Hardware->core,
        0x000E8,
        &Identity->ecoID
        ));

    if (_IsHardwareMatch(Hardware, 0x1000, 0x5037) && (Identity->chipDate == 0x20120617))
    {
        Identity->ecoID = 1;
    }

    if (_IsHardwareMatch(Hardware, 0x320, 0x5303) && (Identity->chipDate == 0x20140511))
    {
        Identity->ecoID = 1;
    }

}

static gceSTATUS
_IdentifyHardwareByDatabase(
    IN gckHARDWARE Hardware,
    IN gckOS Os,
    IN gckDEVICE Device,
    IN gceCORE Core,
    OUT gcsHAL_QUERY_CHIP_IDENTITY_PTR Identity
    )
{
    gceSTATUS status;
    gctUINT32 chipIdentity;
    gctUINT32 debugControl0;
    gctUINT32 chipInfo;
    gcsFEATURE_DATABASE *database;
    gctUINT i = 0;

    gcmkHEADER_ARG("Os=0x%x", Os);

    /* Get chip date. */
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00028, &Identity->chipDate));

    /***************************************************************************
    ** Get chip ID and revision.
    */

    /* Read chip identity register. */
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                             0x00018,
                             &chipIdentity));

    /* Special case for older graphic cores. */
    if (((((gctUINT32) (chipIdentity)) >> (0 ?
 31:24) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1)))))) == (0x01 & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))))
    {
        Identity->chipModel    = gcv500;
        Identity->chipRevision = (((((gctUINT32) (chipIdentity)) >> (0 ? 15:12)) & ((gctUINT32) ((((1 ? 15:12) - (0 ? 15:12) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 15:12) - (0 ? 15:12) + 1)))))) );
    }

    else
    {
        /* Read chip identity register. */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x00020,
                                 (gctUINT32_PTR) &Identity->chipModel));

        if (((Identity->chipModel & 0xFF00) == 0x0400)
          && (Identity->chipModel != 0x0420)
          && (Identity->chipModel != 0x0428))
        {
            Identity->chipModel = (gceCHIPMODEL) (Identity->chipModel & 0x0400);
        }

        /* Read CHIP_REV register. */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x00024,
                                 &Identity->chipRevision));

        if ((Identity->chipModel    == gcv300)
        &&  (Identity->chipRevision == 0x2201)
        )
        {
            gctUINT32 chipDate;
            gctUINT32 chipTime;

            /* Read date and time registers. */
            gcmkONERROR(
                gckOS_ReadRegisterEx(Os, Core,
                                     0x00028,
                                     &chipDate));

            gcmkONERROR(
                gckOS_ReadRegisterEx(Os, Core,
                                     0x0002C,
                                     &chipTime));

            if ((chipDate == 0x20080814) && (chipTime == 0x12051100))
            {
                /* This IP has an ECO; put the correct revision in it. */
                Identity->chipRevision = 0x1051;
            }
        }

        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x000A8,
                                 &Identity->productID));
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                   "Identity: chipModel=%X",
                   Identity->chipModel);

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                   "Identity: chipRevision=%X",
                   Identity->chipRevision);

    _GetEcoID(Hardware, Identity);

    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00030,
                                &Identity->customerID));

     /*get hw minor features*/
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x0001C,
                                &Identity->chipFeatures));

    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00034,
                                &Identity->chipMinorFeatures));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00074,
                                &Identity->chipMinorFeatures1));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00084,
                                &Identity->chipMinorFeatures2));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00088,
                                &Identity->chipMinorFeatures3));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x00094,
                                &Identity->chipMinorFeatures4));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x000A0,
                                &Identity->chipMinorFeatures5));
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                                0x000DC,
                                &Identity->chipMinorFeatures6));

    /***************************************************************************
    ** Get chip features.
    */

    database =
    Hardware->featureDatabase =
    gcQueryFeatureDB(
        Hardware->identity.chipModel,
        Hardware->identity.chipRevision,
        Hardware->identity.productID,
        Hardware->identity.ecoID,
        Hardware->identity.customerID);

    if (database == gcvNULL)
    {
        gcmkPRINT("[galcore]: Feature database is not found,"
                  "chipModel=0x%0x, chipRevision=0x%x, productID=0x%x, ecoID=0x%x, customerID=0x%x",
                  Hardware->identity.chipModel,
                  Hardware->identity.chipRevision,
                  Hardware->identity.productID,
                  Hardware->identity.ecoID,
                  Hardware->identity.customerID);
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }
    else if (database->chipVersion != Hardware->identity.chipRevision)
    {
        gcmkPRINT("[galcore]: Warning: chipRevision mismatch, database chipRevision=0x%x register read chipRevision=0x%x\n",
                  database->chipVersion, Hardware->identity.chipRevision);
    }

    if (database->Q_CHANNEL_SUPPORT)
    {
        /* Qchannel power on. */
        gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvTRUE, gcvTRUE));
    }


    Identity->pixelPipes                    = database->NumPixelPipes;
    Identity->resolvePipes                  = database->NumResolvePipes;
    Identity->instructionCount              = database->InstructionCount;
    Identity->numConstants                  = database->NumberOfConstants;
    Identity->varyingsCount                 = database->VaryingCount;
    Identity->gpuCoreCount                  = database->CoreCount;
    Identity->streamCount                   = database->Streams;
    Identity->clusterAvailMask              = database->ClusterAliveMask;
    Identity->nnClusterNum                  = database->NN_CLUSTER_NUM_FOR_POWER_CONTROL;

    if (gcmIS_SUCCESS(gckOS_QueryOption(Hardware->os, "sRAMBases", Device->sRAMBases[0])))
    {
        gckOS_MemCopy(
            Identity->sRAMBases,
            Device->sRAMBases[Core],
            sizeof(gctUINT64) * gcvSRAM_INTER_COUNT
            );
    }
    else
    {
        for (i = 0; i < gcvSRAM_INTER_COUNT; i++)
        {
            Identity->sRAMBases[i] = gcvINVALID_PHYSICAL_ADDRESS;
        }
    }

    if (gcmIS_SUCCESS(gckOS_QueryOption(Hardware->os, "sRAMSizes", (gctUINT64 *)Device->sRAMSizes[0])))
    {
        gckOS_MemCopy(
            Identity->sRAMSizes,
            Device->sRAMSizes[Core],
            sizeof(gctUINT32) * gcvSRAM_INTER_COUNT
            );
    }
    else
    {
        for (i = gcvSRAM_INTERNAL0; i < gcvSRAM_INTER_COUNT; i++)
        {
            Identity->sRAMSizes[i] = 0;
        }
    }


    for (i = gcvSRAM_INTERNAL0; i < gcvSRAM_INTER_COUNT; i++)
    {
        if (Identity->sRAMSizes[i])
        {
            break;
        }
    }

    /* If module parameter doesn't set per-core SRAM sizes. */
    if (i == gcvSRAM_INTER_COUNT)
    {
        gctUINT j = 0;
        for (i = Core; i < gcvCORE_COUNT; i++)
        {
            for (j = gcvSRAM_INTERNAL0; j < gcvSRAM_INTER_COUNT; j++)
            {
                /* Try to get SRAM sizes from database. */
                Device->sRAMSizes[i][j] = Identity->sRAMSizes[j] = database->VIP_SRAM_SIZE;
            }
        }
    }

    gckOS_QueryOption(Hardware->os, "extSRAMBases", Device->extSRAMBases);

    gckOS_QueryOption(Hardware->os, "extSRAMSizes", (gctUINT64 *)Device->extSRAMSizes);

    for (i = gcvSRAM_EXTERNAL0; i < gcvSRAM_EXT_COUNT; i++)
    {
        if (Device->extSRAMSizes[i])
        {
            break;
        }
    }

    /* If module parameter doesn't set external SRAM sizes. */
    if (i == gcvSRAM_EXT_COUNT)
    {
        for (i = gcvSRAM_EXTERNAL0; i < gcvSRAM_EXT_COUNT; i++)
        {
            /* Try to get SRAM sizes from database. */
            Device->extSRAMSizes[i] = database->AXI_SRAM_SIZE;
        }
    }

    if (Identity->chipModel == gcv320)
    {
        gctUINT32 data;

        gcmkONERROR(
            gckOS_ReadRegisterEx(Os,
                                 Core,
                                 0x0002C,
                                 &data));

        if ((data != 33956864) &&
            ((Identity->chipRevision == 0x5007) ||
            (Identity->chipRevision == 0x5220)))
        {
            Hardware->maxOutstandingReads = 0xFF &
                (Identity->chipRevision == 0x5220 ? 8 :
                (Identity->chipRevision == 0x5007 ? 12 : 0));
        }
    }

    if (_IsHardwareMatch(Hardware, gcv880, 0x5107))
    {
        Hardware->maxOutstandingReads = 0x00010;
    }

    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00470, &debugControl0));

    if (debugControl0 & (1 << 16))
    {
        Identity->chipFlags |= gcvCHIP_FLAG_MSAA_COHERENCEY_ECO_FIX;
    }

    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x000A4, &chipInfo));

    if (((((gctUINT32) (chipInfo)) >> (0 ?
 21:21) & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:21) - (0 ?
 21:21) + 1)))))) == (0x1 & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 21:21) - (0 ? 21:21) + 1))))))))
    {
        Identity->chipFlags |= gcvCHIP_AXI_BUS128_BITS;
    }

    gckOS_QueryOption(Os, "platformFlagBits", &Identity->platformFlagBits);

    gckOS_QueryOption(Os, "registerAPB", &Identity->registerAPB);

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_GetHardwareSignature(
    IN gckHARDWARE Hardware,
    IN gckOS Os,
    IN gceCORE Core,
    OUT gcsHARDWARE_SIGNATURE * Signature
    )
{
    gceSTATUS status;

    gctUINT32 chipIdentity;

    gcmkHEADER_ARG("Os=0x%x", Os);

    /***************************************************************************
    ** Get chip ID and revision.
    */

    /* Read chip identity register. */
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                             0x00018,
                             &chipIdentity));

    /* Special case for older graphic cores. */
    if (((((gctUINT32) (chipIdentity)) >> (0 ?
 31:24) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1)))))) == (0x01 & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))))
    {
        Signature->chipModel    = gcv500;
        Signature->chipRevision = (((((gctUINT32) (chipIdentity)) >> (0 ? 15:12)) & ((gctUINT32) ((((1 ? 15:12) - (0 ? 15:12) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 15:12) - (0 ? 15:12) + 1)))))) );
    }

    else
    {
        /* Read chip identity register. */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x00020,
                                 (gctUINT32_PTR) &Signature->chipModel));

        /* Read CHIP_REV register. */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x00024,
                                 &Signature->chipRevision));
    }

    /***************************************************************************
    ** Get chip features.
    */

    /* Read chip feature register. */
    gcmkONERROR(
        gckOS_ReadRegisterEx(Os, Core,
                             0x0001C,
                             &Signature->chipFeatures));

    if (((Signature->chipModel == gcv500) && (Signature->chipRevision < 2))
    ||  ((Signature->chipModel == gcv300) && (Signature->chipRevision < 0x2000))
    )
    {
        /* GC500 rev 1.x and GC300 rev < 2.0 doesn't have these registers. */
        Signature->chipMinorFeatures  = 0;
        Signature->chipMinorFeatures1 = 0;
        Signature->chipMinorFeatures2 = 0;
    }
    else
    {
        /* Read chip minor feature register #0. */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Os, Core,
                                 0x00034,
                                 &Signature->chipMinorFeatures));

        if (((((gctUINT32) (Signature->chipMinorFeatures)) >> (0 ?
 21:21) & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:21) - (0 ?
 21:21) + 1)))))) == (0x1 & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 21:21) - (0 ? 21:21) + 1)))))))
        )
        {
            /* Read chip minor features register #1. */
            gcmkONERROR(
                gckOS_ReadRegisterEx(Os, Core,
                                     0x00074,
                                     &Signature->chipMinorFeatures1));

            /* Read chip minor features register #2. */
            gcmkONERROR(
                gckOS_ReadRegisterEx(Os, Core,
                                     0x00084,
                                     &Signature->chipMinorFeatures2));
        }
        else
        {
            /* Chip doesn't has minor features register #1 or 2 or 3 or 4 or 5. */
            Signature->chipMinorFeatures1 = 0;
            Signature->chipMinorFeatures2 = 0;
        }
    }

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/* Set to 1 to enable module clock gating debug function.
*  Following options take effect when it is set to 1.
*/
#define gcdDEBUG_MODULE_CLOCK_GATING          0
/* Set to 1 to disable module clock gating of all modules. */
#define gcdDISABLE_MODULE_CLOCK_GATING        0
/* Set to 1 to disable module clock gating of each module. */
#define gcdDISABLE_STARVE_MODULE_CLOCK_GATING 0
#define gcdDISABLE_FE_CLOCK_GATING            0
#define gcdDISABLE_PE_CLOCK_GATING            0
#define gcdDISABLE_SH_CLOCK_GATING            0
#define gcdDISABLE_PA_CLOCK_GATING            0
#define gcdDISABLE_SE_CLOCK_GATING            0
#define gcdDISABLE_RA_CLOCK_GATING            0
#define gcdDISABLE_RA_EZ_CLOCK_GATING         0
#define gcdDISABLE_RA_HZ_CLOCK_GATING         0
#define gcdDISABLE_TX_CLOCK_GATING            0
#define gcdDISABLE_TFB_CLOCK_GATING           0
#define gcdDISABLE_GPIPE_CLOCK_GATING         0
#define gcdDISABLE_BLT_CLOCK_GATING           0
#define gcdDISABLE_TPG_CLOCK_GATING           0
#define gcdDISABLE_VX_CLOCK_GATING            0

#if gcdDEBUG_MODULE_CLOCK_GATING
gceSTATUS
_ConfigureModuleLevelClockGating(
    gckHARDWARE Hardware
    )
{
    gctUINT32 data;

    gcmkVERIFY_OK(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             Hardware->powerBaseAddress
                             + 0x00104,
                             &data));

#if gcdDISABLE_FE_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
#endif

#if gcdDISABLE_PE_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)));
#endif

#if gcdDISABLE_SH_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));
#endif

#if gcdDISABLE_PA_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
#endif

#if gcdDISABLE_SE_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));
#endif

#if gcdDISABLE_RA_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));
#endif

#if gcdDISABLE_TX_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7)));
#endif

#if gcdDISABLE_RA_EZ_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));
#endif

#if gcdDISABLE_RA_HZ_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)));
#endif

#if gcdDISABLE_TFB_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)));
#endif

#if gcdDISABLE_GPIPE_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));
#endif

#if gcdDISABLE_BLT_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:20) - (0 ?
 20:20) + 1))))))) << (0 ?
 20:20))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 20:20) - (0 ? 20:20) + 1))))))) << (0 ? 20:20)));
#endif

#if gcdDISABLE_TPG_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:18) - (0 ?
 18:18) + 1))))))) << (0 ?
 18:18))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:18) - (0 ? 18:18) + 1))))))) << (0 ? 18:18)));
#endif

#if gcdDISABLE_VX_CLOCK_GATING
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 21:21) - (0 ?
 21:21) + 1))))))) << (0 ?
 21:21))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 21:21) - (0 ?
 21:21) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 21:21) - (0 ? 21:21) + 1))))))) << (0 ? 21:21)));
#endif

    gcmkVERIFY_OK(
        gckOS_WriteRegisterEx(Hardware->os,
                              Hardware->core,
                              Hardware->powerBaseAddress
                              + 0x00104,
                              data));

#if gcdDISABLE_STARVE_MODULE_CLOCK_GATING
    gcmkVERIFY_OK(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             Hardware->powerBaseAddress +
                             0x00100,
                             &data));

    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)));

    gcmkVERIFY_OK(
        gckOS_WriteRegisterEx(Hardware->os,
                              Hardware->core,
                              Hardware->powerBaseAddress
                              + 0x00100,
                              data));

#endif

#if gcdDISABLE_MODULE_CLOCK_GATING
    gcmkVERIFY_OK(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             Hardware->powerBaseAddress +
                             0x00100,
                             &data));

    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));


    gcmkVERIFY_OK(
        gckOS_WriteRegisterEx(Hardware->os,
                              Hardware->core,
                              Hardware->powerBaseAddress
                              + 0x00100,
                              data));
#endif

    return gcvSTATUS_OK;
}
#endif

static void
_PowerStateTimerFunc(
    gctPOINTER Data
    )
{
    gckHARDWARE hardware = (gckHARDWARE)Data;

    gcmkVERIFY_OK(gckHARDWARE_SetPowerState(hardware, hardware->nextPowerState));
}

static gceSTATUS
_VerifyDMA(
    IN gckOS Os,
    IN gceCORE Core,
    gctUINT32_PTR Address1,
    gctUINT32_PTR Address2,
    gctUINT32_PTR State1,
    gctUINT32_PTR State2
    )
{
    gceSTATUS status;
    gctUINT32 i;

    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00660, State1));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00660, State1));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00664, Address1));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00664, Address1));

    for (i = 0; i < 500; i += 1)
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00660, State2));
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00660, State2));
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00664, Address2));
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00664, Address2));

        if (*Address1 != *Address2)
        {
            break;
        }

        if (*State1 != *State2)
        {
            break;
        }
    }

OnError:
    return status;
}

static gceSTATUS
_DumpDebugRegisters(
    IN gckOS Os,
    IN gceCORE Core,
    IN gcsiDEBUG_REGISTERS_PTR Descriptor
    )
{
/* If this value is changed, print formats need to be changed too. */
#define REG_PER_LINE 8
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 select;
    gctUINT i, j, pipe;
    gctUINT32 datas[REG_PER_LINE];
    gctUINT32 oldControl, control;

    gcmkHEADER_ARG("Os=0x%X Descriptor=0x%X", Os, Descriptor);

    /* Record control. */
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x0, &oldControl));

    for (pipe = 0; pipe < 4; pipe++)
    {
        if (!Descriptor->avail)
        {
            continue;
        }
        if (!(Descriptor->pipeMask & (1 << pipe)))
        {
            continue;
        }

        gcmkPRINT_N(8, "    %s[%d] debug registers:\n", Descriptor->module, pipe);

        /* Switch pipe. */
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x0, &control));
        control &= ~(0xF << 20);
        control |= (pipe << 20);
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x0, control));

        gcmkASSERT(!(Descriptor->count % REG_PER_LINE));

        for (i = 0; i < Descriptor->count; i += REG_PER_LINE)
        {
            /* Select of first one in the group. */
            select = i + Descriptor->selectStart;

            /* Read a group of registers. */
            for (j = 0; j < REG_PER_LINE; j++)
            {
                /* Shift select to right position. */
                gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, (select + j) << Descriptor->shift));
                gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &datas[j]));
            }

            gcmkPRINT_N(32, "    [%02X] %08X %08X %08X %08X %08X %08X %08X %08X\n",
                        select, datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7]);
        }
    }

    /* Restore control. */
    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x0, oldControl));

OnError:
    /* Return the error. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_DumpLinkStack(
    IN gckOS Os,
    IN gceCORE Core,
    IN gcsiDEBUG_REGISTERS_PTR Descriptor
    )
{
    /* Get wrptr */
    gctUINT32 shift = Descriptor->shift;
    gctUINT32 pointerSelect = 0xE << shift;
    gctUINT32 pointer, wrPtr, rdPtr, links[16];
    gctUINT32 stackSize = 16;
    gctUINT32 oldestPtr = 0;
    gctUINT32 i;

    gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, pointerSelect));
    gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &pointer));

    wrPtr = (pointer & 0xF0) >> 4;
    rdPtr = pointer & 0xF;

    /* Move rdptr to the oldest one (next one to the latest one. ) */
    oldestPtr = (wrPtr + 1) % stackSize;

    while (rdPtr != oldestPtr)
    {
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, 0x0));
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, 0xF << shift));


        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, pointerSelect));
        gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &pointer));

        rdPtr = pointer & 0xF;
    }

    gcmkPRINT("    Link stack:");

    /* Read from stack bottom*/
    for (i = 0; i < stackSize; i++)
    {
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, 0xD << shift));
        gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &links[i]));

        /* Advance rdPtr. */
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, 0x0));
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, 0xF << shift));
    }

    /* Print. */
    for (i = 0; i < stackSize; i += 4)
    {
        gcmkPRINT_N(32, "      [0x%02X] 0x%08X [0x%02X] 0x%08X [0x%02X] 0x%08X [0x%02X] 0x%08X\n",
                    i, links[i], i + 1, links[i + 1], i + 2, links[i + 2], i + 3, links[i + 3]);
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_DumpFEStack(
    IN gckOS Os,
    IN gceCORE Core,
    IN gcsiDEBUG_REGISTERS_PTR Descriptor
    )
{
    gctUINT i;
    gctINT j;
    gctUINT32 stack[32][2];
    gctUINT32 link[32];

    static gcsFE_STACK _feStacks[] =
    {
        { "PRE_STACK", 32, 0x1A, 0x9A, 0x00, 0x1B, 0x1E },
        { "CMD_STACK", 32, 0x1C, 0x9C, 0x1E, 0x1D, 0x1E },
    };

    for (i = 0; i < gcmCOUNTOF(_feStacks); i++)
    {
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, _feStacks[i].clear));

        for (j = 0; j < _feStacks[i].count; j++)
        {
            gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, _feStacks[i].highSelect));

            gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &stack[j][0]));

            gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, _feStacks[i].lowSelect));

            gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &stack[j][1]));

            gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, _feStacks[i].next));

            if (_feStacks[i].linkSelect)
            {
                gcmkVERIFY_OK(gckOS_WriteRegisterEx(Os, Core, Descriptor->index, _feStacks[i].linkSelect));

                gcmkVERIFY_OK(gckOS_ReadRegisterEx(Os, Core, Descriptor->data, &link[j]));
            }
        }

        gcmkPRINT("  %s:", _feStacks[i].name);

        for (j = 31; j >= 3; j -= 4)
        {
            gcmkPRINT("    %08X %08X %08X %08X %08X %08X %08X %08X",
                      stack[j][0], stack[j][1], stack[j - 1][0], stack[j - 1][1],
                      stack[j - 2][0], stack[j - 2][1], stack[j - 3][0], stack[j - 3][1]);
        }

        if (_feStacks[i].linkSelect)
        {
            gcmkPRINT("  LINK_STACK:");

            for (j = 31; j >= 3; j -= 4)
            {
                gcmkPRINT("    %08X %08X %08X %08X %08X %08X %08X %08X",
                          link[j], link[j], link[j - 1], link[j - 1],
                          link[j - 2], link[j - 2], link[j - 3], link[j - 3]);
            }
        }

    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_IsGPUPresent(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gcsHARDWARE_SIGNATURE signature;
    gctUINT32 control;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x00000,
                                     &control));

    control = ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)));
    control = ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      control));

    gckOS_ZeroMemory((gctPOINTER)&signature, gcmSIZEOF(gcsHARDWARE_SIGNATURE));

    /* Identify the hardware. */
    gcmkONERROR(_GetHardwareSignature(Hardware,
                                      Hardware->os,
                                      Hardware->core,
                                      &signature));

    /* Check if these are the same values as saved before. */
    if ((Hardware->signature.chipModel          != signature.chipModel)
    ||  (Hardware->signature.chipRevision       != signature.chipRevision)
    ||  (Hardware->signature.chipFeatures       != signature.chipFeatures)
    ||  (Hardware->signature.chipMinorFeatures  != signature.chipMinorFeatures)
    ||  (Hardware->signature.chipMinorFeatures1 != signature.chipMinorFeatures1)
    ||  (Hardware->signature.chipMinorFeatures2 != signature.chipMinorFeatures2)
    )
    {
        gcmkPRINT("[galcore]: GPU is not present.");
        gcmkONERROR(gcvSTATUS_GPU_NOT_RESPONDING);
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the error. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_FlushCache(
    gckHARDWARE Hardware,
    gckCOMMAND Command
    )
{
    gceSTATUS status;
    gctUINT32 bytes, requested;
    gctPOINTER buffer;

    /* Get the size of the flush command. */
    gcmkONERROR(gckHARDWARE_Flush(Hardware,
                                  gcvFLUSH_ALL,
                                  gcvNULL,
                                  &requested));

    /* Reserve space in the command queue. */
    gcmkONERROR(gckCOMMAND_Reserve(Command,
                                   requested,
                                   &buffer,
                                   &bytes));

    /* Append a flush. */
    gcmkONERROR(gckHARDWARE_Flush(
        Hardware, gcvFLUSH_ALL, buffer, &bytes
        ));

    /* Execute the command queue. */
    gcmkONERROR(gckCOMMAND_Execute(Command, requested));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gctBOOL
_IsHWIdle(
    IN gctUINT32 Idle,
    IN gckHARDWARE Hardware
    )
{
    if (Hardware->identity.customerID == 0x15 ||
        gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_VIP_SCALER) ||
        gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_VIP_SCALER_4K))
    {
        Idle = (Idle | (1 << 14));
    }
    return Idle == 0x7FFFFFFF;
}

static gctBOOL
_QueryFeatureDatabase(
    IN gckHARDWARE Hardware,
    IN gceFEATURE Feature
    )
{
    gctBOOL available;

    gcsFEATURE_DATABASE *database = Hardware->featureDatabase;

    gcmkHEADER_ARG("Hardware=0x%x Feature=%d", Hardware, Feature);

     /* Only features needed by common kernel logic added here. */
    switch (Feature)
    {
    case gcvFEATURE_END_EVENT:
        available = gcvFALSE;
        break;

    case gcvFEATURE_MC20:
        available = database->REG_MC20;
        break;

    case gcvFEATURE_EARLY_Z:
        available = database->REG_NoEZ == 0;
        break;

    case gcvFEATURE_HZ:
        available = database->REG_HierarchicalZ;
        break;

    case gcvFEATURE_NEW_HZ:
        available = database->REG_NewHZ;
        break;

    case gcvFEATURE_FAST_MSAA:
        available = database->REG_FastMSAA;
        break;

    case gcvFEATURE_SMALL_MSAA:
        available = database->REG_SmallMSAA;
        break;

    case gcvFEATURE_DYNAMIC_FREQUENCY_SCALING:
        /* This feature doesn't apply for 2D cores. */
        available = database->REG_DynamicFrequencyScaling && database->REG_Pipe3D;

        if (Hardware->identity.chipModel == gcv1000 &&
            (Hardware->identity.chipRevision == 0x5039 ||
            Hardware->identity.chipRevision == 0x5040))
        {
            available = gcvFALSE;
        }
        break;

    case gcvFEATURE_ACE:
        available = database->REG_ACE;
        break;

    case gcvFEATURE_HALTI2:
        available = database->REG_Halti2;
        break;

    case gcvFEATURE_PIPE_2D:
        available = database->REG_Pipe2D;
        break;

    case gcvFEATURE_PIPE_3D:
        available = gcvFALSE;
        break;

    case gcvFEATURE_FC_FLUSH_STALL:
        available = database->REG_FcFlushStall;
        break;

    case gcvFEATURE_BLT_ENGINE:
        available = database->REG_BltEngine;
       break;

    case gcvFEATURE_HALTI0:
        available = database->REG_Halti0;
        break;

    case gcvFEATURE_FE_ALLOW_STALL_PREFETCH_ENG:
        available = database->REG_FEAllowStallPrefetchEng;
        break;

    case gcvFEATURE_MMU:
#if gcdCAPTURE_ONLY_MODE
        available = gcvTRUE;
#else
        available = database->REG_MMU;
#endif

        break;

    case gcvFEATURE_FENCE_64BIT:
        available = database->FENCE_64BIT;
        break;

    case gcvFEATURE_TEX_BASELOD:
        available = database->REG_Halti2;

        if (_IsHardwareMatch(Hardware, gcv900, 0x5250))
        {
            available = gcvTRUE;
        }
        break;

    case gcvFEATURE_TEX_CACHE_FLUSH_FIX:
        available = database->REG_Halti5;
        break;

    case gcvFEATURE_BUG_FIXES1:
        available = database->REG_BugFixes1;
        break;

    case gcvFEATURE_MULTI_SOURCE_BLT:
        available = database->REG_MultiSourceBlt;
        break;

    case gcvFEATURE_HALTI5:
        available = database->REG_Halti5;
        break;

    case gcvFEATURE_FAST_CLEAR:
        available = database->REG_FastClear;

        if (Hardware->identity.chipModel == gcv700)
        {
            available = gcvFALSE;
        }
        break;

    case gcvFEATURE_BUG_FIXES7:
        available = database->REG_BugFixes7;
        break;

    case gcvFEATURE_ZCOMPRESSION:
        available = database->REG_ZCompression;
        break;

    case gcvFEATURE_SHADER_HAS_INSTRUCTION_CACHE:
        available = database->REG_InstructionCache;
        break;

    case gcvFEATURE_YUV420_TILER:
        available = database->REG_YUV420Tiler;
        break;

    case gcvFEATURE_2DPE20:
        available = database->REG_2DPE20;
        break;

    case gcvFEATURE_DITHER_AND_FILTER_PLUS_ALPHA_2D:
        available = database->REG_DitherAndFilterPlusAlpha2D;
        break;

    case gcvFEATURE_ONE_PASS_2D_FILTER:
        available = database->REG_OnePass2DFilter;
        break;

    case gcvFEATURE_HALTI1:
        available = database->REG_Halti1;
        break;

    case gcvFEATURE_HALTI3:
        available = database->REG_Halti3;
        break;

    case gcvFEATURE_HALTI4:
        available = database->REG_Halti4;
        break;

    case gcvFEATURE_GEOMETRY_SHADER:
        available = database->REG_GeometryShader;
        break;

    case gcvFEATURE_TESSELLATION:
        available = database->REG_TessellationShaders;
        break;

    case gcvFEATURE_GENERIC_ATTRIB:
        available = database->REG_Generics;
        break;

    case gcvFEATURE_TEXTURE_LINEAR:
        available = database->REG_LinearTextureSupport;
        break;

    case gcvFEATURE_TX_FILTER:
        available = database->REG_TXFilter;
        break;

    case gcvFEATURE_TX_SUPPORT_DEC:
        available = database->REG_TXSupportDEC;
        break;

    case gcvFEATURE_TX_FRAC_PRECISION_6BIT:
        available = database->REG_TX6bitFrac;
        break;

    case gcvFEATURE_TEXTURE_ASTC:
        available = database->REG_TXEnhancements4 && !database->NO_ASTC;
        break;

    case gcvFEATURE_SHADER_ENHANCEMENTS2:
        available = database->REG_SHEnhancements2;
        break;

    case gcvFEATURE_BUG_FIXES18:
        available = database->REG_BugFixes18;
        break;

    case gcvFEATURE_64K_L2_CACHE:
        available = gcvFALSE;
        break;

    case gcvFEATURE_BUG_FIXES4:
        available = database->REG_BugFixes4;
        break;

    case gcvFEATURE_BUG_FIXES12:
        available = database->REG_BugFixes12;
        break;

    case gcvFEATURE_HW_TFB:
        available = database->HWTFB;
        break;

    case gcvFEATURE_SNAPPAGE_CMD_FIX:
        available = database->SH_SNAP2PAGE_FIX;
        break;

    case gcvFEATURE_SECURITY:
        available = database->SECURITY;
        break;

    case gcvFEATURE_TX_DESCRIPTOR:
        available = database->REG_Halti5;
        break;

    case gcvFEATURE_TX_DESC_CACHE_CLOCKGATE_FIX:
        available = database->TX_DESC_CACHE_CLOCKGATE_FIX;
        break;

    case gcvFEATURE_ROBUSTNESS:
        available = database->ROBUSTNESS;
        break;

    case gcvFEATURE_SNAPPAGE_CMD:
        available = database->SNAPPAGE_CMD;
        break;

    case gcvFEATURE_HALF_FLOAT_PIPE:
        available = database->REG_HalfFloatPipe;
        break;

    case gcvFEATURE_SH_INSTRUCTION_PREFETCH:
        available = database->SH_ICACHE_PREFETCH;
        break;

    case gcvFEATURE_FE_NEED_DUMMYDRAW:
        available = database->FE_NEED_DUMMYDRAW;

        if (_IsHardwareMatch(Hardware, gcv600, 0x4653) || _IsHardwareMatch(Hardware, gcv600, 0x4633))
        {
            available = gcvTRUE;
        }

        break;

    case gcvFEATURE_DEC300_COMPRESSION:
        available = database->REG_DEC;
        break;

    case gcvFEATURE_DEC400_COMPRESSION:
        available = database->G2D_DEC400;
        break;

    case gcvFEATURE_DEC400EX_COMPRESSION:
        available = database->G2D_DEC400EX;
        break;

    case gcvFEATURE_TPC_COMPRESSION:
        available = database->REG_ThirdPartyCompression;
        break;

    case gcvFEATURE_TPCV11_COMPRESSION:
        available = database->G2D_3rd_PARTY_COMPRESSION_1_1;
        break;

    case gcvFEATURE_USC_DEFER_FILL_FIX:
        available = database->USC_DEFER_FILL_FIX;
        break;

    case gcvFEATURE_USC:
        available = database->REG_Halti5;
        break;

    case gcvFEATURE_RA_CG_FIX:
        available = database->RA_CG_FIX;
        break;

    case gcvFEATURE_MULTI_CLUSTER:
        available = database->MULTI_CLUSTER;
        break;

    case gcvFEATURE_ZERO_ATTRIB_SUPPORT:
        available = database->REG_Halti4;
        break;

    case gcvFEATURE_SH_CLOCK_GATE_FIX:
        available = database->SH_CLOCK_GATE_FIX;
        break;

    case gcvFEATURE_GPIPE_CLOCK_GATE_FIX:
        available = gcvFALSE;
        break;

    case gcvFEATURE_NEW_GPIPE:
        available = database->NEW_GPIPE;
        break;

    case gcvFEATURE_MULTI_CORE_BLOCK_SET_CONFIG2:
        available = database->MULTI_CORE_BLOCK_SET_CONFIG2;
        break;

    case gcvFEATURE_SECURITY_AHB:
        available = database->SECURITY_AHB;
        break;

    case gcvFEATURE_SMALL_BATCH:
        available = database->SMALLBATCH;
        break;

    case gcvFEATURE_ASYNC_BLIT:
        available = database->ASYNC_BLT;
        break;

    case gcvFEATURE_PSCS_THROTTLE:
        available = database->PSCS_THROTTLE;
        break;

    case gcvFEATURE_SEPARATE_LS:
        available = database->SEPARATE_LS;
        break;

    case gcvFEATURE_MCFE:
        available = database->MCFE;
        break;

    case gcvFEATURE_COMPUTE_ONLY:
        available = database->COMPUTE_ONLY;
        break;

    case gcvFEATURE_USC_FULLCACHE_FIX:
        available = database->USC_FULL_CACHE_FIX;
        break;

    case gcvFEATURE_PE_TILE_CACHE_FLUSH_FIX:
        available = database->PE_TILE_CACHE_FLUSH_FIX;
        break;

    case gcvFEATURE_TILE_STATUS_2BITS:
        available = database->REG_TileStatus2Bits;
        break;

    case gcvFEATURE_128BTILE:
        available = database->CACHE128B256BPERLINE;
        break;

    case gcvFEATURE_COMPRESSION_DEC400:
        available = database->DEC400;
        break;

    case gcvFEATURE_SUPPORT_GCREGTX:
        available = database->REG_Halti1;

        if (Hardware->identity.chipModel == gcv880 &&
            ((Hardware->identity.chipRevision & 0xfff0) == 0x5120))
        {
            available = gcvTRUE;
        }
        break;

    case gcvFEATURE_MSAA_FRAGMENT_OPERATION:
        available = database->MSAA_FRAGMENT_OPERATION;
        break;

    case gcvFEATURE_OCB_COUNTER:
        available = database->OCB_COUNTER;
        break;

    case gcvFEATURE_AI_GPU:
        available = database->AI_GPU;
        break;

    case gcvFEATURE_NN_ENGINE:
        available = database->NNCoreCount > 0;
        break;

    case gcvFEATURE_TP_ENGINE:
        available = database->TP_ENGINE;
        break;

    case gcvFEATURE_HI_REORDER_FIX:
        available = database->HI_REORDER_FIX;
        break;

    case gcvFEATURE_EVIS2_FLOP_RESET_FIX:
        available = database->EVIS2_FLOP_RESET_FIX;
        break;

    case gcvFEATURE_USC_ASYNC_CP_RTN_FLOP_RESET_FIX:
        available = database->USC_ASYNC_CP_RTN_FLOP_RESET_FIX;
        break;

    case gcvFEATURE_TS_FC_VULKAN_SUPPORT:
        available = database->TS_FC_VULKAN_SUPPORT;
        break;

    case gcvFEATURE_USC_EVICT_CTRL_FIFO_FLOP_RESET_FIX:
        available = database->USC_EVICT_CTRL_FIFO_FLOP_RESET_FIX;
        break;

    case gcvFEATURE_Q_CHANNEL_SUPPORT:
        available = database->Q_CHANNEL_SUPPORT;
        break;

    case gcvFEATURE_MMU_PAGE_DESCRIPTOR:
        available = database->MMU_PAGE_DESCRIPTOR;
        break;

    case gcvFEATURE_VIP_REMOVE_MMU:
        available = database->VIP_REMOVE_MMU;
        break;

    case gcvFEATURE_VIP_SCALER:
        available = database->SCALER;
        break;

    case gcvFEATURE_VIP_SCALER_4K:
        available = database->SCALER_4K;
        break;

        /*FALLTHRU*/
    default:
        gcmkFATAL("Invalid feature has been requested.");
        available = gcvFALSE;
        /*FALLTHRU*/
    }

    gcmkFOOTER_ARG("%d", available ? gcvSTATUS_TRUE : gcvSTATUS_FALSE);
    return available;
}

static void
_ConfigurePolicyID(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 policyID;
    gctUINT32 auxBit = ~0U;
    gctUINT32 axiConfig;
    gckOS os = Hardware->os;
    gceCORE core = Hardware->core;
    gctUINT32 i;
    gctUINT32 offset;
    gctUINT32 shift;
    gctUINT32 currentAxiConfig;

    status = gckOS_GetPolicyID(os, gcvVIDMEM_TYPE_GENERIC, &policyID, &axiConfig);

    if (status == gcvSTATUS_NOT_SUPPORTED)
    {
        /* No customized policyID setting. */
        return;
    }

    for (i = 0; i < 16; i++)
    {
        /* Mapping 16 surface type.*/
        status = gckOS_GetPolicyID(os, (gceVIDMEM_TYPE) i, &policyID, &axiConfig);

        if (gcmIS_SUCCESS(status))
        {
            if (auxBit == ~0U)
            {
                /* There is a customized policyID setting for this type. */
                auxBit = (policyID >> 4) & 0x1;
            }
            else
            {
                /* Check whether this bit changes. */
                if (auxBit != ((policyID >> 4) & 0x1))
                {
                    gcmkPRINT("[galcore]: AUX_BIT changes");
                    return;
                }
            }

            offset = policyID >> 1;

            shift  = (policyID & 0x1) * 16;

            axiConfig &= 0xFFFF;

            gcmkVERIFY_OK(gckOS_ReadRegisterEx(
                os,
                core,
                (0x0070 + offset) << 2,
                &currentAxiConfig
                ));

            currentAxiConfig |= (axiConfig << shift);

            gcmkVERIFY_OK(gckOS_WriteRegisterEx(
                os,
                core,
                (0x0070 + offset) << 2,
                currentAxiConfig
                ));
        }
    }

    if (auxBit != ~0U)
    {
        gcmkVERIFY_OK(gckOS_WriteRegisterEx(
            os,
            core,
            0x000EC,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)))
          | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:8) - (0 ?
 8:8) + 1))))))) << (0 ?
 8:8))) | (((gctUINT32) ((gctUINT32) (auxBit) & ((gctUINT32) ((((1 ?
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:8) - (0 ? 8:8) + 1))))))) << (0 ? 8:8)))
            ));
    }
}

static gceSTATUS
_QueryNNClusters(
    IN gckHARDWARE Hardware
    )
{
    gctUINT64 enableNN = ~0UL;
    gctUINT32 value = 0;
    gceSTATUS status = gcvSTATUS_OK;

    if (gcmIS_SUCCESS(gckOS_QueryOption(Hardware->os, "enableNN", &enableNN)))
    {
        if (!enableNN)
        {
            value = 0x2;
        }
        else if (enableNN == 0xFF || (enableNN == Hardware->identity.nnClusterNum))
        {
            value = 0;
        }
        else
        {
            /* We only support maximum 8 clusters by current. */
            if (enableNN > 0x7)
            {
                gcmkPRINT("[Galcore warning]: Invalid enableNN value is configured.");

                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            value = (gctUINT32)enableNN + 0x2;
        }
    }

    Hardware->options.enableNNClusters = (gctUINT32)enableNN;

    if (value && Hardware->identity.customerID != 0x85)
    {
        gcmkPRINT("Galcore warning: Don't set enableNN as this chip not support NN cluster power control!\n");
    }

    Hardware->options.configNNPowerControl = value;

OnError:
    return status;
}

/****************************
** Initialise hardware options
*/
static void
_SetHardwareOptions(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT64 data = 0;
    gcsHAL_QUERY_CHIP_OPTIONS *options = &Hardware->options;
    gcsFEATURE_DATABASE *database = Hardware->featureDatabase;
    gctBOOL featureUSC = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_USC) ? gcvTRUE : gcvFALSE;
    gctBOOL featureSeparateLS = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SEPARATE_LS) ? gcvTRUE : gcvFALSE;
    gctBOOL featureComputeOnly = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_COMPUTE_ONLY) ? gcvTRUE : gcvFALSE;
    gctBOOL featureTS = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_TESSELLATION) ? gcvTRUE : gcvFALSE;
    gctUINT32 featureL1CacheSize = database->L1CacheSize;
    gctUINT32 featureUSCMaxPages = database->USC_MAX_PAGES;
    gctUINT32 i;

    status = gckOS_QueryOption(Hardware->os, "powerManagement", &data);
    options->powerManagement = (data != 0);

    if (status == gcvSTATUS_NOT_SUPPORTED)
    {
        /* Enable power management by default. */
        options->powerManagement = gcvTRUE;
    }

#ifndef EMULATOR
    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_VIP_REMOVE_MMU))
    {
        options->enableMMU = gcvFALSE;
    }
    else
#endif
    {
        status = gckOS_QueryOption(Hardware->os, "mmu", &data);
        options->enableMMU = (data != 0);

        if (status == gcvSTATUS_NOT_SUPPORTED)
        {
            /* Disable MMU if we can't get result from OS layer query */
            options->enableMMU = gcvFALSE;
        }
    }

    if (options->enableMMU == gcvFALSE)
    {
        gcmkPRINT("Galcore warning: MMU is disabled!\n");
    }

    /* Query enabled NN clusters. */
    _QueryNNClusters(Hardware);

    gcmCONFIGUSC2(gcmk, featureUSC, featureSeparateLS, featureComputeOnly, featureTS,
                 featureL1CacheSize, featureUSCMaxPages,
                 Hardware->options.uscAttribCacheRatio, Hardware->options.uscL1CacheRatio);

    status = gckOS_QueryOption(Hardware->os, "smallBatch", &data);
    options->smallBatch = (data != 0);

    if (status == gcvSTATUS_NOT_SUPPORTED)
    {
        options->smallBatch = gcvTRUE;
    }

    status = gckOS_QueryOption(Hardware->os, "userClusterMasks", (gctUINT64 *)options->userClusterMasks);

    for (i = 0; i < gcdMAX_MAJOR_CORE_COUNT; i++)
    {
        options->userClusterMasks[i] &= Hardware->identity.clusterAvailMask;
    }

    options->userClusterMask = options->userClusterMasks[Hardware->core];

    if (status == gcvSTATUS_NOT_SUPPORTED || (options->userClusterMask == 0))
    {
        /* use all clusters identified in database */
        options->userClusterMasks[Hardware->core] = options->userClusterMask = Hardware->identity.clusterAvailMask;
    }
    else if (options->userClusterMask & (~Hardware->identity.clusterAvailMask))
    {
        gcmkPRINT("%s(%d): user cluster mask(0x%x) must be a subset of available clusters(0x%x),ignored it!",
                  __FUNCTION__, __LINE__, options->userClusterMask, Hardware->identity.clusterAvailMask);
        options->userClusterMasks[Hardware->core] = options->userClusterMask = Hardware->identity.clusterAvailMask;
    }

    options->secureMode = gcvSECURE_NONE;

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SECURITY))
    {
        gcmkASSERT(gcvSTATUS_TRUE == gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SECURITY_AHB));

        options->secureMode = gcvSECURE_IN_NORMAL;

        status = gckOS_QueryOption(Hardware->os, "TA", &data);

        if (gcmIS_SUCCESS(status) && data)
        {
            options->secureMode = gcvSECURE_IN_TA;
        }
    }
    else if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SECURITY_AHB))
    {
        options->secureMode = gcvSECURE_IN_NORMAL;
    }

    options->hasShader = database->NumShaderCores;

    return;
}

/*
* State timer helper must be called with powerMutex held.
*/
static void
gckSTATETIMER_Reset(
    IN gcsSTATETIMER * StateTimer,
    IN gctUINT64 Start
    )
{
    gctUINT64 now;

    if (Start)
    {
        now = Start;
    }
    else
    {
        gckOS_GetProfileTick(&now);
    }

    StateTimer->recent = StateTimer->start = now;

    gckOS_ZeroMemory(StateTimer->elapse, gcmSIZEOF(StateTimer->elapse));
}

gceSTATUS
gckHARDWARE_StartTimerReset(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcmkHEADER();

    gckSTATETIMER_Reset(&Hardware->powerStateCounter, 0);

    gcmkFOOTER();
    return status;
}

static void
gckSTATETIMER_Accumulate(
    IN gcsSTATETIMER * StateTimer,
    IN gceCHIPPOWERSTATE OldState
    )
{
    gctUINT64 now;
    gctUINT64 elapse;

    gckOS_GetProfileTick(&now);

    elapse = now - StateTimer->recent;

    StateTimer->recent = now;

    StateTimer->elapse[OldState] += elapse;
}

static void
gckSTATETIMER_Query(
    IN gcsSTATETIMER * StateTimer,
    IN gceCHIPPOWERSTATE State,
    OUT gctUINT64_PTR On,
    OUT gctUINT64_PTR Off,
    OUT gctUINT64_PTR Idle,
    OUT gctUINT64_PTR Suspend
    )
{
    gckSTATETIMER_Accumulate(StateTimer, State);

    *On      = StateTimer->elapse[gcvPOWER_ON];
    *Off     = StateTimer->elapse[gcvPOWER_OFF];
    *Idle    = StateTimer->elapse[gcvPOWER_IDLE];
    *Suspend = StateTimer->elapse[gcvPOWER_SUSPEND];
}

static gceSTATUS
_InitPageTableArray(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gcmkHEADER_ARG("Hardware=%p", Hardware);

    if (Hardware->options.secureMode == gcvSECURE_IN_NORMAL)
    {
        gcePOOL pool = gcvPOOL_DEFAULT;
        gctUINT32 flags = gcvALLOC_FLAG_CONTIGUOUS;

#if defined(CONFIG_ZONE_DMA32) || defined(CONFIG_ZONE_DMA)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
        flags |= gcvALLOC_FLAG_4GB_ADDR;
#endif
#endif

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        flags |= gcvALLOC_FLAG_CACHEABLE;
#endif

        Hardware->pagetableArray.size = 1024;

        /* Allocate mmu table array within 32bit space */
        gcmkONERROR(gckKERNEL_AllocateVideoMemory(
            Hardware->kernel,
            64,
            gcvVIDMEM_TYPE_COMMAND,
            flags,
            &Hardware->pagetableArray.size,
            &pool,
            &Hardware->pagetableArray.videoMem
            ));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
            Hardware->kernel,
            Hardware->pagetableArray.videoMem,
            gcvFALSE,
            gcvFALSE,
            &Hardware->pagetableArray.logical
            ));

        /* Get CPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
            Hardware->kernel,
            Hardware->pagetableArray.videoMem,
            0,
            &Hardware->pagetableArray.address
            ));

        /* Convert to GPU physical address. */
        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(
            Hardware->os,
            Hardware->pagetableArray.address,
            &Hardware->pagetableArray.address
            ));
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (Hardware->pagetableArray.videoMem)
    {
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            Hardware->kernel,
            Hardware->pagetableArray.videoMem
            ));
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_SetupSRAMVidMem(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT i;

    for (i = gcvSRAM_INTERNAL0; i < gcvSRAM_INTER_COUNT; i++)
    {
        if (Hardware->identity.sRAMSizes[i] &&
           (Hardware->identity.sRAMBases[i] != gcvINVALID_PHYSICAL_ADDRESS))
        {
            /* If the internal SRAM usage is memory block. */
            status = gckVIDMEM_Construct(
                Hardware->os,
                Hardware->identity.sRAMBases[i],
                Hardware->identity.sRAMSizes[i],
                64,
                0,
                &Hardware->sRAMVidMem[i]
                );

            if (gcmIS_ERROR(status))
            {
                Hardware->identity.sRAMSizes[i] = 0;
                Hardware->sRAMVidMem[i] = gcvNULL;
            }
            else
            {
                char sRAMName[20];
                gctUINT64 data = 0;
                gctBOOL sRAMRequested;

                gcmkSPRINTF(sRAMName, gcmSIZEOF(sRAMName) - 1, "gcCore%dSRAM%d", Hardware->core, i);
                status = gckOS_QueryOption(Hardware->os, "sRAMRequested", (gctUINT64 *)&data);
                sRAMRequested = (status == gcvSTATUS_OK) ? (data != 0) : gcvFALSE;

                gcmkONERROR(gckOS_RequestReservedMemory(
                    Hardware->os,
                    Hardware->identity.sRAMBases[i],
                    Hardware->identity.sRAMSizes[i],
                    sRAMName,
                    sRAMRequested,
                    gcvTRUE,
                    &Hardware->sRAMPhysical[i]
                    ));

                Hardware->sRAMVidMem[i]->physical = Hardware->sRAMPhysical[i];
            }
        }
    }

OnError:
    return status;
}

/******************************************************************************\
****************************** gckHARDWARE API code *****************************
\******************************************************************************/

/*******************************************************************************
**
**  gckHARDWARE_Construct
**
**  Construct a new gckHARDWARE object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an initialized gckOS object.
**
**      gceCORE Core
**          Specified core.
**
**  OUTPUT:
**
**      gckHARDWARE * Hardware
**          Pointer to a variable that will hold the pointer to the gckHARDWARE
**          object.
*/
gceSTATUS
gckHARDWARE_Construct(
    IN gckOS Os,
    IN gckDEVICE Device,
    IN gceCORE Core,
    OUT gckHARDWARE * Hardware
    )
{
    gceSTATUS status;
    gckHARDWARE hardware = gcvNULL;
    gctUINT16 data = 0xff00;
    gctPOINTER pointer = gcvNULL;
    gctUINT    i;
    gctUINT64 enableSoftReset = 1;
    gcmkHEADER_ARG("Os=0x%x", Os);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    /* Enable the GPU. */
    gcmkONERROR(gckOS_SetGPUPower(Os, Core, gcvTRUE, gcvTRUE));
    gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                      Core,
                                      0x00000,
                                      0x00010900));

    /* Allocate the gckHARDWARE object. */
    gcmkONERROR(gckOS_Allocate(Os,
                               gcmSIZEOF(struct _gckHARDWARE),
                               &pointer));

    gckOS_ZeroMemory(pointer, gcmSIZEOF(struct _gckHARDWARE));

    hardware = (gckHARDWARE) pointer;

    /* Initialize the gckHARDWARE object. */
    hardware->object.type = gcvOBJ_HARDWARE;
    hardware->os          = Os;
    hardware->core        = Core;

    gcmkONERROR(_GetHardwareSignature(hardware, Os, Core, &hardware->signature));

    /* Identify the hardware. */
    gcmkONERROR(_IdentifyHardwareByDatabase(hardware, Os, Device, Core, &hardware->identity));

    /* Setup SRAM memory heap. */
    gcmkONERROR(_SetupSRAMVidMem(hardware));

    _SetHardwareOptions(hardware);

    hardware->hasQchannel = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_Q_CHANNEL_SUPPORT);

    /* Bypass Qchannel power management. */
    if (!hardware->options.powerManagement && hardware->hasQchannel)
    {
        gcmkONERROR(gckHARDWARE_QchannelBypass(hardware, gcvTRUE));
    }

    hardware->mmuVersion = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MMU);

    /* Get the system's physical base address for old MMU */
    if (hardware->mmuVersion == 0)
    {
        gcmkONERROR(gckOS_GetBaseAddress(Os, &hardware->baseAddress));
    }

    /* Determine the hardware type */
    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_PIPE_3D)
     && gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_PIPE_2D)
    )
    {
        hardware->type = gcvHARDWARE_3D2D;
    }
    else if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_PIPE_2D))
    {
        hardware->type = gcvHARDWARE_2D;
    }
    else if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_AI_GPU))
    {
        hardware->type = gcvHARDWARE_3D;
    }
    else if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_NN_ENGINE)
     || gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TP_ENGINE)
    )
    {
        hardware->type = gcvHARDWARE_VIP;
    }
    else
    {
        hardware->type = gcvHARDWARE_3D;
    }

    hardware->powerBaseAddress
        = ((hardware->identity.chipModel   == gcv300)
        && (hardware->identity.chipRevision < 0x2000))
            ? 0x0100
            : 0x0000;

    status = gckOS_QueryOption(Os, "softReset", (gctUINT64 *)&enableSoftReset);
    if (enableSoftReset == 1)
    {
        /* _ResetGPU need powerBaseAddress. */
        status = _ResetGPU(hardware, Os, Core);

        if (status != gcvSTATUS_OK)
        {
            gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE, "_ResetGPU failed: status=%d\n", status);
        }
    }

#if gcdDEC_ENABLE_AHB
    gcmkONERROR(gckOS_WriteRegisterEx(Os, gcvCORE_DEC, 0x18180, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)))));
#endif

    hardware->hasL2Cache = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_64K_L2_CACHE);

    if (!hardware->hasL2Cache)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                          Core,
                                          0x0055C,
                                          0x00FFFFFF));
    }

    hardware->powerMutex = gcvNULL;

    /* Determine whether bug fixes #1 are present. */
    hardware->extraEventStates = (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_BUG_FIXES1) == gcvFALSE);

    /* Check if big endian */
    hardware->bigEndian = (*(gctUINT8 *)&data == 0xff);

    /* Initialize the fast clear. */
    gcmkONERROR(gckHARDWARE_SetFastClear(hardware, -1, -1));

#if !gcdENABLE_128B_MERGE

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_SOURCE_BLT))
    {
        /* 128B merge is turned on by default. Disable it. */
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x00558, 0));
    }

#endif

#if (gcdFPGA_BUILD && 1)
    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TPCV11_COMPRESSION))
    {
        gctUINT32 data;
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00558, &data));
        data |= 0x1 << 27;
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x00558, data));
    }
#endif

    {
        gctUINT32 value;
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x00090, &value));
#if gcdDEC_ENABLE_AHB
        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_DEC300_COMPRESSION))
        {
            value |= ~0xFFFFFFBF;
        }
        else
#endif
        {
            value &= 0xFFFFFFBF;
        }
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x00090, value));
    }

    /* Set power state to ON. */
    hardware->chipPowerState  = gcvPOWER_ON;
    hardware->clockState      = gcvTRUE;
    hardware->powerState      = gcvTRUE;
    hardware->lastWaitLink    = ~0U;
    hardware->lastEnd         = ~0U;
    hardware->globalSemaphore = gcvNULL;
#if gcdENABLE_FSCALE_VAL_ADJUST
    hardware->powerOnFscaleVal = 64;
#endif
#if gcdPOWEROFF_TIMEOUT
    hardware->powerOffTimeout = gcdPOWEROFF_TIMEOUT;
#endif

    gcmkONERROR(gckOS_CreateMutex(Os, &hardware->powerMutex));
    gcmkONERROR(gckOS_CreateSemaphore(Os, &hardware->globalSemaphore));

    gcmkVERIFY_OK(gckOS_CreateTimer(Os,
                                    _PowerStateTimerFunc,
                                    (gctPOINTER)hardware,
                                    &hardware->powerStateTimer));

    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
    {
        gcmkONERROR(gckOS_AtomConstruct(Os, &hardware->pageTableDirty[i]));
    }

    gcmkONERROR(gckOS_AtomConstruct(Os, &hardware->pendingEvent));


#if defined(LINUX) || defined(__QNXNTO__) || defined(UNDER_CE)
    if (hardware->mmuVersion)
    {
        hardware->stallFEPrefetch
            = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_FE_ALLOW_STALL_PREFETCH_ENG);
    }
    else
#endif
    {
        hardware->stallFEPrefetch =
            _QueryFeatureDatabase(hardware, gcvFEATURE_MCFE) == gcvFALSE;
    }

    hardware->minFscaleValue = 1;
    hardware->waitCount = 200;

#if gcdLINK_QUEUE_SIZE
    gcmkONERROR(gckQUEUE_Allocate(hardware->os, &hardware->linkQueue, gcdLINK_QUEUE_SIZE));
#endif

    /* Initialize FEs, either MCFE or wait-link FE. */
    if (_QueryFeatureDatabase(hardware, gcvFEATURE_MCFE))
    {
        hardware->mcfeChannels[0] = gcvMCFE_CHANNEL_SYSTEM;
        hardware->mcfeChannels[1] = gcvMCFE_CHANNEL_SHADER;
        hardware->mcfeChannels[2] = gcvMCFE_CHANNEL_NN;
        hardware->mcfeChannels[3] = gcvMCFE_CHANNEL_TP;

        hardware->mcfeChannelCount = 4;

        gcmkONERROR(gckMCFE_Construct(hardware, &hardware->mcFE));
    }
    else
    {
        gcmkONERROR(gckWLFE_Construct(hardware, &hardware->wlFE));
    }

    if (_QueryFeatureDatabase(hardware, gcvFEATURE_ASYNC_BLIT))
    {
        gcmkONERROR(gckASYNC_FE_Construct(hardware, &hardware->asyncFE));
    }

    /* Construct hardware function */
    gcmkONERROR(gckFUNCTION_Construct(hardware));

    /* Return pointer to the gckHARDWARE object. */
    *Hardware = hardware;

    /* Success. */
    gcmkFOOTER_ARG("*Hardware=0x%x", *Hardware);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (hardware != gcvNULL)
    {
        /* Turn off the power. */
        gcmkVERIFY_OK(gckOS_SetGPUPower(Os, Core, gcvFALSE, gcvFALSE));

        if (hardware->globalSemaphore != gcvNULL)
        {
            /* Destroy the global semaphore. */
            gcmkVERIFY_OK(gckOS_DestroySemaphore(Os,
                                                 hardware->globalSemaphore));
        }

        if (hardware->powerMutex != gcvNULL)
        {
            /* Destroy the power mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Os, hardware->powerMutex));
        }

        if (hardware->powerStateTimer != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_StopTimer(Os, hardware->powerStateTimer));
            gcmkVERIFY_OK(gckOS_DestroyTimer(Os, hardware->powerStateTimer));
        }

        for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
        {
            if (hardware->pageTableDirty[i] != gcvNULL)
            {
                gcmkVERIFY_OK(gckOS_AtomDestroy(Os, hardware->pageTableDirty[i]));
            }
        }

        if (hardware->pendingEvent != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_AtomDestroy(Os, hardware->pendingEvent));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, hardware));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_PostConstruct(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT i;

    /* Initialize MMU page table array. */
    gcmkONERROR(_InitPageTableArray(Hardware));

    for (i = 0; i < gcvFUNCTION_EXECUTION_NUM; i++)
    {
        gctBOOL funcVaild = gcvFALSE;

        gckFUNCTION_Validate(&Hardware->functions[i], &funcVaild);
        if (funcVaild)
        {
            gcmkONERROR(gckFUNCTION_Init(&Hardware->functions[i]));
        }
    }

    return gcvSTATUS_OK;

OnError:
    for (i = 0; i < gcvFUNCTION_EXECUTION_NUM; i++)
    {
        gckFUNCTION_Release(&Hardware->functions[i]);
    }
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_PreDestroy
**
**  Prepare destroying an gckHARDWARE object.
**  This is to destroy resources relevant to other modules such as MMU.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object that needs to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_PreDestroy(
    IN gckHARDWARE Hardware
    )
{
    gcmkHEADER_ARG("%x", Hardware);

    gcmkVERIFY_OK(gckFUNCTION_Destory(Hardware));

    if (Hardware->pagetableArray.videoMem)
    {
        gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
            Hardware->kernel,
            Hardware->pagetableArray.videoMem,
            0,
            gcvFALSE,
            gcvFALSE
            ));

        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            Hardware->kernel,
            Hardware->pagetableArray.videoMem
            ));

        Hardware->pagetableArray.videoMem = gcvNULL;
        Hardware->pagetableArray.logical  = gcvNULL;
    }

    if (Hardware->wlFE)
    {
        gckWLFE_Destroy(Hardware, Hardware->wlFE);
        Hardware->wlFE = gcvNULL;
    }

    if (Hardware->asyncFE)
    {
        gckASYNC_FE_Destroy(Hardware, Hardware->asyncFE);
        Hardware->asyncFE = gcvNULL;
    }

    if (Hardware->mcFE)
    {
        gckMCFE_Destroy(Hardware, Hardware->mcFE);
        Hardware->mcFE = gcvNULL;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_Destroy
**
**  Destroy an gckHARDWARE object.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object that needs to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_Destroy(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT i;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Destroy the power semaphore. */
    gcmkVERIFY_OK(gckOS_DestroySemaphore(Hardware->os,
                                         Hardware->globalSemaphore));

    /* Destroy the power mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Hardware->os, Hardware->powerMutex));

    gcmkVERIFY_OK(gckOS_StopTimer(Hardware->os, Hardware->powerStateTimer));
    gcmkVERIFY_OK(gckOS_DestroyTimer(Hardware->os, Hardware->powerStateTimer));

    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
    {
        gcmkVERIFY_OK(gckOS_AtomDestroy(Hardware->os, Hardware->pageTableDirty[i]));
    }

    gcmkVERIFY_OK(gckOS_AtomDestroy(Hardware->os, Hardware->pendingEvent));

#if gcdLINK_QUEUE_SIZE
    gckQUEUE_Free(Hardware->os, &Hardware->linkQueue);
#endif

    /* Mark the object as unknown. */
    Hardware->object.type = gcvOBJ_UNKNOWN;

    /* Free the object. */
    gcmkONERROR(gcmkOS_SAFE_FREE(Hardware->os, Hardware));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_GetType
**
**  Get the hardware type.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      gceHARDWARE_TYPE * Type
**          Pointer to a variable that receives the type of hardware object.
*/
gceSTATUS
gckHARDWARE_GetType(
    IN gckHARDWARE Hardware,
    OUT gceHARDWARE_TYPE * Type
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);
    gcmkVERIFY_ARGUMENT(Type != gcvNULL);

    *Type = Hardware->type;

    gcmkFOOTER_ARG("*Type=%d", *Type);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_InitializeHardware
**
**  Initialize the hardware.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_InitializeHardware(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 control;
    gctUINT32 data;
    gctUINT32 regPMC = 0;
    gctUINT32 i;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Disable isolate GPU bit. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      ((((gctUINT32) (0x00010900)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)))));

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x00000,
                                     &control));

    /* Enable debug register. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)))));

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SECURITY_AHB) &&
        (Hardware->options.secureMode == gcvSECURE_IN_NORMAL))
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x003A8,
                                          ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))));
    }
    /* Reset memory counters. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0003C,
                                      ~0U));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0003C,
                                      0));

    if (Hardware->mmuVersion == 0)
    {
        /* Program the base addesses. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x0041C,
                                          Hardware->baseAddress));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00418,
                                          Hardware->baseAddress));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00428,
                                          Hardware->baseAddress));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00420,
                                          Hardware->baseAddress));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00424,
                                          Hardware->baseAddress));
    }

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     Hardware->powerBaseAddress +
                                     0x00100,
                                     &data));

    /* Enable clock gating. */
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

    if ((Hardware->identity.chipRevision == 0x4301)
    ||  (Hardware->identity.chipRevision == 0x4302)
    )
    {
        /* Disable stall module level clock gating for 4.3.0.1 and 4.3.0.2
        ** revisions. */
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)));
    }

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      Hardware->powerBaseAddress
                                      + 0x00100,
                                      data));

    /* Initialize FE. */
    if (Hardware->wlFE)
    {
        gckWLFE_Initialize(Hardware, Hardware->wlFE);
    }
    else if (Hardware->mcFE)
    {
        gckMCFE_Initialize(Hardware, gcvFALSE, Hardware->mcFE);
    }

    if (Hardware->asyncFE)
    {
        gckASYNC_FE_Initialize(Hardware, Hardware->asyncFE);
    }


    if (Hardware->identity.chipModel == gcv4000 &&
        ((Hardware->identity.chipRevision == 0x5208) || (Hardware->identity.chipRevision == 0x5222)))
    {
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x0010C,
                                  ((((gctUINT32) (0x015B0880)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))))) << (0 ?
 23:23))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:23) - (0 ? 23:23) + 1))))))) << (0 ? 23:23)))));
    }

    if ((Hardware->identity.chipModel == gcv1000 &&
         (Hardware->identity.chipRevision == 0x5039 ||
          Hardware->identity.chipRevision == 0x5040))
        ||
        (Hardware->identity.chipModel == gcv2000 &&
         Hardware->identity.chipRevision == 0x5140)
       )
    {
        gctUINT32 pulseEater;

        pulseEater = ((((gctUINT32) (0x015B0880)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));

        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x0010C,
                                  ((((gctUINT32) (pulseEater)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)))));
    }

    if ((gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI2) == gcvSTATUS_FALSE)
     || (Hardware->identity.chipRevision < 0x5422)
    )
    {
        if (regPMC == 0)
        {
            gcmkONERROR(
                gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     Hardware->powerBaseAddress
                                     + 0x00104,
                                     &regPMC));
        }

        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:15) - (0 ?
 15:15) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:15) - (0 ?
 15:15) + 1))))))) << (0 ?
 15:15))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 15:15) - (0 ?
 15:15) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:15) - (0 ? 15:15) + 1))))))) << (0 ? 15:15)));
    }

    if (_IsHardwareMatch(Hardware, gcv2000, 0x5108))
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 0x00480,
                                 &data));

        /* Set FE bus to one, TX bus to zero */
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7)));

        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00480,
                                  data));
    }

    /* AHBDEC400 */
    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_DEC400EX_COMPRESSION))
    {
        data = 0x0201018A;
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)));
        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os,
            Hardware->core,
            0x00800,
            data));

        data = 0x003FC810;
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:0) - (0 ?
 6:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:0) - (0 ?
 6:0) + 1))))))) << (0 ?
 6:0))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 6:0) - (0 ?
 6:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:0) - (0 ? 6:0) + 1))))))) << (0 ? 6:0)));
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:7) - (0 ?
 13:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:7) - (0 ?
 13:7) + 1))))))) << (0 ?
 13:7))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 13:7) - (0 ?
 13:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:7) - (0 ? 13:7) + 1))))))) << (0 ? 13:7)));
        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os,
            Hardware->core,
            0x00808,
            data));
    }

    if (Hardware->identity.chipModel == 0x620 &&
        Hardware->identity.productID == 0x6200 &&
        ((Hardware->identity.chipRevision == 0x5552 &&
        Hardware->identity.customerID == 0x209) ||
        (Hardware->identity.chipRevision == 0x5553 &&
        Hardware->identity.customerID == 0x210) ||
        (Hardware->identity.chipRevision == 0x5553 &&
        Hardware->identity.customerID == 0x20F)))
    {
        gctUINT32 offset = 0;

        gcmkSAFECASTPHYSADDRT(offset, Hardware->identity.registerAPB);

        gcmkPRINT("Initailize APB1 registers, APB offset is 0x%x.\n", offset);

        /* APB FE ctrl. */
        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os,
            Hardware->core,
            offset + 0x28,
            0x2));

        /* APB FE cfg. */
        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os,
            Hardware->core,
            offset + 0x2C,
            0x2));
    }

#if !gcdCAPTURE_ONLY_MODE
    gcmkONERROR(
        gckHARDWARE_SetMMU(Hardware,
                           Hardware->kernel->mmu));
#endif

    if (Hardware->mcFE)
    {
        /* Reinitialize MCFE, now MMU is enabled. */
        gckMCFE_Initialize(Hardware, gcvTRUE, Hardware->mcFE);
    }

    if (Hardware->identity.chipModel >= gcv400
    &&  Hardware->identity.chipModel != gcv420
    &&  !gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_BUG_FIXES12))
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable PA clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
    }

    /* Limit 2D outstanding request. */
    if (Hardware->maxOutstandingReads)
    {
        gctUINT32 data;

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 0x00414,
                                 &data));

        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (Hardware->maxOutstandingReads & 0xFF) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)));

        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00414,
                                  data));
    }

    if (_IsHardwareMatch(Hardware, gcv1000, 0x5035))
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 0x00414,
                                 &data));

        /* Disable HZ-L2. */
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)));

        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00414,
                                  data));
    }

    if (_IsHardwareMatch(Hardware, gcv4000, 0x5222)
     || _IsHardwareMatch(Hardware, gcv2000, 0x5108)
     || _IsHardwareMatch(Hardware, gcv7000, 0x6202)
     || _IsHardwareMatch(Hardware, gcv7000, 0x6203)
     || (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_TX_DESCRIPTOR)
       && !gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_TX_DESC_CACHE_CLOCKGATE_FIX)
        )
    )
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable TX clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7)));
    }

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NEW_GPIPE) &&
        !gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_GPIPE_CLOCK_GATE_FIX))
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable GPIPE clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));
    }

    if (_IsHardwareMatch(Hardware, gcv880, 0x5106))
    {
        Hardware->kernel->timeOut = 140 * 1000;
    }

    if (regPMC == 0)
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
    }

    /* Disable RA HZ clock gating. */
    regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)));

    /* Disable RA EZ clock gating. */
    regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));

    if ((gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI5)
       && !gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_RA_CG_FIX)
        )
       )
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable RA clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));
    }

    if ((gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI5)
       && !gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SH_CLOCK_GATE_FIX)
        )
       )
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable SH clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));
    }

    if (_IsHardwareMatch(Hardware, gcv7000, 0x6202))
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));

        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));

        gcmkVERIFY_OK(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress +
                                 0x00100,
                                 &data));

        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));


        gcmkVERIFY_OK(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00100,
                                  data));
    }

    if (_IsHardwareMatch(Hardware, gcv8000, 0x7200))
    {
        if (regPMC == 0)
        {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &regPMC));
        }

        /* Disable SH_EU clock gating. */
        regPMC = ((((gctUINT32) (regPMC)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)));
    }

    if (regPMC != 0)
    {
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00104,
                                  regPMC));
    }

    if (_IsHardwareMatch(Hardware, gcv2000, 0x5108)
     || (_IsHardwareMatch(Hardware, gcv2000, 0xffff5450))
     || _IsHardwareMatch(Hardware, gcv320, 0x5007)
     || _IsHardwareMatch(Hardware, gcv320, 0x5303)
     || _IsHardwareMatch(Hardware, gcv880, 0x5106)
     || _IsHardwareMatch(Hardware, gcv400, 0x4645)
    )
    {
        /* Update GPU AXI cache atttribute. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00008,
                                          0x00002200));
    }

    if ((Hardware->identity.chipRevision > 0x5420)
     && gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_PIPE_3D))
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x0010C,
                                     &data));

        /* Disable internal DFS. */
        data =
#if gcdDVFS
            ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:18) - (0 ?
 18:18) + 1))))))) << (0 ?
 18:18))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:18) - (0 ? 18:18) + 1))))))) << (0 ? 18:18))) |
#endif
            ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16))) |
            ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0010C,
                                      data));
    }

    if (_IsHardwareMatch(Hardware, gcv2500, 0x5422))
    {
        gcmkONERROR(gckOS_ReadRegisterEx(
            Hardware->os, Hardware->core, 0x00090, &data));

        /* AXI switch setup to SPLIT_TO64 mode */
        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) (0x2 & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)));

        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os, Hardware->core, 0x00090, data));
    }

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_ENGINE) &&
        (!gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HI_REORDER_FIX) ||
            (Hardware->kernel->device->extSRAMSizes[0] == 0)) &&
        (((gcsFEATURE_DATABASE *)Hardware->featureDatabase)->HI_DEFAULT_ENABLE_REORDER_FIX)
        )
    {
        gcmkONERROR(gckOS_ReadRegisterEx(
            Hardware->os, Hardware->core, 0x00090, &data));

        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)));

        gcmkONERROR(gckOS_WriteRegisterEx(
            Hardware->os, Hardware->core, 0x00090, data));
    }

    _ConfigurePolicyID(Hardware);

    gcmkONERROR(gckHARDWARE_PowerControlClusters(Hardware, Hardware->options.configNNPowerControl, gcvTRUE));

#if gcdDEBUG_MODULE_CLOCK_GATING
    _ConfigureModuleLevelClockGating(Hardware);
#endif

    /* Perfrom hardware functions */
    for (i = 0; i < gcvFUNCTION_EXECUTION_NUM; i++)
    {
        gctBOOL funcVaild = gcvFALSE;

        /* Skip functions since it will perform at special place */
        if (i == gcvFUNCTION_EXECUTION_MMU || i == gcvFUNCTION_EXECUTION_FLUSH)
        {
            continue;
        }

        gckFUNCTION_Validate(&Hardware->functions[i], &funcVaild);
        if (funcVaild)
        {
            gckFUNCTION_Execute(&Hardware->functions[i]);
            gckFUNCTION_Execute(&Hardware->functions[gcvFUNCTION_EXECUTION_FLUSH]);
        }
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the error. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryMemory
**
**  Query the amount of memory available on the hardware.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**  OUTPUT:
**
**      gctSIZE_T * InternalSize
**          Pointer to a variable that will hold the size of the internal video
**          memory in bytes.  If 'InternalSize' is gcvNULL, no information of the
**          internal memory will be returned.
**
**      gctUINT32 * InternalBaseAddress
**          Pointer to a variable that will hold the hardware's base address for
**          the internal video memory.  This pointer cannot be gcvNULL if
**          'InternalSize' is also non-gcvNULL.
**
**      gctUINT32 * InternalAlignment
**          Pointer to a variable that will hold the hardware's base address for
**          the internal video memory.  This pointer cannot be gcvNULL if
**          'InternalSize' is also non-gcvNULL.
**
**      gctSIZE_T * ExternalSize
**          Pointer to a variable that will hold the size of the external video
**          memory in bytes.  If 'ExternalSize' is gcvNULL, no information of the
**          external memory will be returned.
**
**      gctUINT32 * ExternalBaseAddress
**          Pointer to a variable that will hold the hardware's base address for
**          the external video memory.  This pointer cannot be gcvNULL if
**          'ExternalSize' is also non-gcvNULL.
**
**      gctUINT32 * ExternalAlignment
**          Pointer to a variable that will hold the hardware's base address for
**          the external video memory.  This pointer cannot be gcvNULL if
**          'ExternalSize' is also non-gcvNULL.
**
**      gctUINT32 * HorizontalTileSize
**          Number of horizontal pixels per tile.  If 'HorizontalTileSize' is
**          gcvNULL, no horizontal pixel per tile will be returned.
**
**      gctUINT32 * VerticalTileSize
**          Number of vertical pixels per tile.  If 'VerticalTileSize' is
**          gcvNULL, no vertical pixel per tile will be returned.
*/
gceSTATUS
gckHARDWARE_QueryMemory(
    IN gckHARDWARE Hardware,
    OUT gctSIZE_T * InternalSize,
    OUT gctUINT32 * InternalBaseAddress,
    OUT gctUINT32 * InternalAlignment,
    OUT gctSIZE_T * ExternalSize,
    OUT gctUINT32 * ExternalBaseAddress,
    OUT gctUINT32 * ExternalAlignment,
    OUT gctUINT32 * HorizontalTileSize,
    OUT gctUINT32 * VerticalTileSize
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (InternalSize != gcvNULL)
    {
        /* No internal memory. */
        *InternalSize = 0;
    }

    if (ExternalSize != gcvNULL)
    {
        /* No external memory. */
        *ExternalSize = 0;
    }

    if (HorizontalTileSize != gcvNULL)
    {
        /* 4x4 tiles. */
        *HorizontalTileSize = 4;
    }

    if (VerticalTileSize != gcvNULL)
    {
        /* 4x4 tiles. */
        *VerticalTileSize = 4;
    }

    /* Success. */
    gcmkFOOTER_ARG("*InternalSize=%lu *InternalBaseAddress=0x%08x "
                   "*InternalAlignment=0x%08x *ExternalSize=%lu "
                   "*ExternalBaseAddress=0x%08x *ExtenalAlignment=0x%08x "
                   "*HorizontalTileSize=%u *VerticalTileSize=%u",
                   gcmOPT_VALUE(InternalSize),
                   gcmOPT_VALUE(InternalBaseAddress),
                   gcmOPT_VALUE(InternalAlignment),
                   gcmOPT_VALUE(ExternalSize),
                   gcmOPT_VALUE(ExternalBaseAddress),
                   gcmOPT_VALUE(ExternalAlignment),
                   gcmOPT_VALUE(HorizontalTileSize),
                   gcmOPT_VALUE(VerticalTileSize));
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryChipIdentity
**
**  Query the identity of the hardware.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**  OUTPUT:
**
**      gcsHAL_QUERY_CHIP_IDENTITY_PTR Identity
**          Pointer to the identity structure.
**
*/
gceSTATUS
gckHARDWARE_QueryChipIdentity(
    IN gckHARDWARE Hardware,
    OUT gcsHAL_QUERY_CHIP_IDENTITY_PTR Identity
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Identity != gcvNULL);

    *Identity = Hardware->identity;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryChipOptions
**
**  Query the options of the hardware.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**  OUTPUT:
**
**      gcsHAL_QUERY_CHIP_OPTIONS_PTR Options
**          Pointer to the identity structure.
**
*/
gceSTATUS
gckHARDWARE_QueryChipOptions(
    IN gckHARDWARE Hardware,
    OUT gcsHAL_QUERY_CHIP_OPTIONS_PTR Options
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Options != gcvNULL);

    *Options = Hardware->options;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}


/*******************************************************************************
**
**  gckHARDWARE_SplitMemory
**
**  Split a hardware specific memory address into a pool and offset.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to the gckHARDWARE object.
**
**      gctUINT32 Address
**          Address in hardware specific format.
**
**  OUTPUT:
**
**      gcePOOL * Pool
**          Pointer to a variable that will hold the pool type for the address.
**
**      gctUINT32 * Offset
**          Pointer to a variable that will hold the offset for the address.
*/
gceSTATUS
gckHARDWARE_SplitMemory(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Address,
    OUT gcePOOL * Pool,
    OUT gctUINT32 * Offset
    )
{
    gcmkHEADER_ARG("Hardware=0x%x Addres=0x%08x", Hardware, Address);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Pool != gcvNULL);
    gcmkVERIFY_ARGUMENT(Offset != gcvNULL);

    if (Hardware->mmuVersion == 0)
    {
        /* Dispatch on memory type. */
        switch ((((((gctUINT32) (Address)) >> (0 ? 31:31)) & ((gctUINT32) ((((1 ? 31:31) - (0 ? 31:31) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1)))))) ))
        {
        case 0x0:
            /* System memory. */
            *Pool = gcvPOOL_SYSTEM;
            break;

        case 0x1:
            /* Virtual memory. */
            *Pool = gcvPOOL_VIRTUAL;
            break;

        default:
            /* Invalid memory type. */
            gcmkFOOTER_ARG("status=%d", gcvSTATUS_INVALID_ARGUMENT);
            return gcvSTATUS_INVALID_ARGUMENT;
        }

        /* Return offset of address. */
        *Offset = (((((gctUINT32) (Address)) >> (0 ? 30:0)) & ((gctUINT32) ((((1 ? 30:0) - (0 ? 30:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 30:0) - (0 ? 30:0) + 1)))))) );
    }
    else
    {
        *Pool = gcvPOOL_SYSTEM;
        *Offset = Address;
    }

    /* Success. */
    gcmkFOOTER_ARG("*Pool=%d *Offset=0x%08x", *Pool, *Offset);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_PipeSelect
**
**  Append a PIPESELECT command at the specified user logical memory.
**
**  WARNING: Only for writing to USERSPACE!
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**      gctPOINTER Logical
**          Pointer to the current location inside the command queue to append
**          the PIPESELECT command at or gcvNULL just to query the size of the
**          PIPESELECT command.
**
**      gcePIPE_SELECT Pipe
**          Pipe value to select.
**
**      gctSIZE_T * Bytes
**          Pointer to the number of bytes available for the PIPESELECT command.
**          If 'Logical' is gcvNULL, this argument will be ignored.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that will receive the number of bytes required
**          for the PIPESELECT command.  If 'Bytes' is gcvNULL, nothing will be
**          returned.
*/
gceSTATUS
gckHARDWARE_PipeSelect(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gcePIPE_SELECT Pipe,
    IN OUT gctUINT32 * Bytes
    )
{
    gctUINT32_PTR logical = (gctUINT32_PTR) Logical;
    gceSTATUS status;

    gcmkHEADER_ARG("Hardware=0x%x Logical=0x%x Pipe=%d *Bytes=%lu",
                   Hardware, Logical, Pipe, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT((Logical == gcvNULL) || (Bytes != gcvNULL));

    /* Append a PipeSelect. */
    if (Logical != gcvNULL)
    {
        gctUINT32 flush, stall;

        if (*Bytes < 32)
        {
            /* Command queue too small. */
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }

        flush = (Pipe == gcvPIPE_2D)
              ?
 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
              : ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));

        stall = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

        /* LoadState(AQFlush, 1), flush. */
        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical,
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 1,
            flush
            ));

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                       "0x%x: FLUSH 0x%x", logical, flush);

        /* LoadState(AQSempahore, 1), stall. */
        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 2,
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 3,
            stall
            ));

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                       "0x%x: SEMAPHORE 0x%x", logical + 2, stall);

        /* Stall, stall. */
        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 4,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 5,
            stall
            ));

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                       "0x%x: STALL 0x%x", logical + 4, stall);

        /* LoadState(AQPipeSelect, 1), pipe. */
        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 6,
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            logical + 7,
            (Pipe == gcvPIPE_2D)
            ? 0x1
            : 0x0
            ));

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                       "0x%x: PIPE %d", logical + 6, Pipe);
    }

    if (Bytes != gcvNULL)
    {
        /* Return number of bytes required by the PIPESELECT command. */
        *Bytes = 32;
    }

    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_FenceRender(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT32 FenceAddress,
    IN gctUINT64 FenceData,
    IN OUT gctUINT32 * Bytes
    )
{
    gckOS os = Hardware->os;
    gctUINT32_PTR logical = (gctUINT32_PTR)Logical;

    gctUINT32 dataLow = (gctUINT32)FenceData;
    gctUINT32 dataHigh = (gctUINT32)(FenceData >> 32);

    if (logical)
    {
        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E1A) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            FenceAddress
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E26) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            dataHigh
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E1B) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            dataLow
            );
    }

    if (Bytes)
    {
        *Bytes = gcdRENDER_FENCE_LENGTH;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_FenceBlt(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT32 FenceAddress,
    IN gctUINT64 FenceData,
    IN OUT gctUINT32 * Bytes
    )
{
    gckOS os = Hardware->os;
    gctUINT32_PTR logical = (gctUINT32_PTR)Logical;

    gctUINT32 dataLow = (gctUINT32)FenceData;
    gctUINT32 dataHigh = (gctUINT32)(FenceData >> 32);

    if (logical)
    {
        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x5029) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            FenceAddress
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502D) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            dataHigh
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502A) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            dataLow
            );

        gcmkWRITE_MEMORY(
            logical,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            );

        gcmkWRITE_MEMORY(
            logical,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
            );
    }

    if (Bytes)
    {
        *Bytes = gcdBLT_FENCE_LENGTH;
    }

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_Fence
**
**  Append a HW FENCE states at the specified user logical memory.
**
**  WARNING: Only for writing to USERSPACE!
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**      gceENGINE Engine
**          Engine type, render or 3d-blit currently.
**
**
**      gctPOINTER Logical
**          Pointer to the current location inside the command queue to append
**          the PIPESELECT command at or gcvNULL just to query the size of the
**          PIPESELECT command.
**
**      gctUINT32 FenceAddress
**          GPU address to write out the data.
**
**      gctUINT64 FenceData
**          The 64bit data to write out.
**
**      gctSIZE_T * Bytes
**          Pointer to the number of bytes available for the PIPESELECT command.
**          If 'Logical' is gcvNULL, this argument will be ignored.
**
**  OUTPUT:
**
**      gctSIZE_T * Bytes
**          Pointer to a variable that will receive the number of bytes required
**          for the PIPESELECT command.  If 'Bytes' is gcvNULL, nothing will be
**          returned.
*/
gceSTATUS
gckHARDWARE_Fence(
    IN gckHARDWARE Hardware,
    IN gceENGINE  Engine,
    IN gctPOINTER Logical,
    IN gctUINT32 FenceAddress,
    IN gctUINT64 FenceData,
    IN OUT gctUINT32 * Bytes
    )
{
    if (Engine == gcvENGINE_RENDER)
    {
        return _FenceRender(Hardware, Logical, FenceAddress, FenceData, Bytes);
    }
    else
    {
         return _FenceBlt(Hardware, Logical, FenceAddress, FenceData, Bytes);
    }
}

/*******************************************************************************
**
**  gckHARDWARE_UpdateQueueTail
**
**  Update the tail of the command queue.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**      gctPOINTER Logical
**          Logical address of the start of the command queue.
**
**      gctUINT32 Offset
**          Offset into the command queue of the tail (last command).
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_UpdateQueueTail(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT32 Offset
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Hardware=0x%x Logical=0x%x Offset=0x%08x",
                   Hardware, Logical, Offset);

    /* Verify the hardware. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Force a barrier. */
    gcmkONERROR(
        gckOS_MemoryBarrier(Hardware->os, Logical));

    /* Notify gckKERNEL object of change. */
    gcmkONERROR(
        gckKERNEL_Notify(Hardware->kernel,
                         gcvNOTIFY_COMMAND_QUEUE));

    if (status == gcvSTATUS_CHIP_NOT_READY)
    {
        gcmkONERROR(gcvSTATUS_DEVICE);
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}


static void
_ResumeWaitLinkFE(
    gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 resume;
    gctUINT32 bytes;
    gctUINT32 idle;

    /* Make sure FE is idle. */
    do
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             0x00004,
                             &idle));
    }
    while (idle != 0x7FFFFFFF);

    gcmkDUMP(Hardware->os, "@[register.wait 0x%05X 0x%08X 0x%08X]",
             0x00004,
             ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))),
             idle);

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                         Hardware->core,
                         0x00664,
                         &resume));

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                         Hardware->core,
                         0x00664,
                         &resume));

    bytes = Hardware->hasL2Cache ? 24 : 16;

    /* Start Command Parser. */
    gckWLFE_Execute(Hardware, resume, bytes);

OnError:
    return;
}

/*******************************************************************************
**
**  gckHARDWARE_Interrupt
**
**  Process an interrupt. This function will read the interrupt acknowledge
**  register, stores the data, and return whether the interrupt is from us.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_Interrupt(
    IN gckHARDWARE Hardware
    )
{
    gctUINT32 data = 0;
    gctUINT32 dataEx = 0;
    gceSTATUS status;
    gceSTATUS statusEx;

    /*
     * Notice:
     * In isr here.
     * We should return success when either FE or AsyncFE reports correct
     * interrupts, so that isr can wake up threadRoutine for either FE.
     * That means, only need return ERROR when both FEs reports ERROR.
     */
    /* Read AQIntrAcknowledge register. */
    status = gckOS_ReadRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00010,
                                  &data);

    if (gcmIS_ERROR(status))
    {
        goto OnError;
    }

    if (data == 0)
    {
        /* Not our interrupt. */
        status = gcvSTATUS_NOT_OUR_INTERRUPT;
    }
    else
    {
#if gcdINTERRUPT_STATISTIC
        gckOS_AtomClearMask(Hardware->pendingEvent, data);
#endif

        /* Inform gckEVENT of the interrupt. */
        status = gckEVENT_Interrupt(Hardware->kernel->eventObj, data);
    }

    if (!Hardware->asyncFE)
    {
        /* Done. */
        goto OnError;
    }

    /* Read BLT interrupt. */
    statusEx = gckOS_ReadRegisterEx(
        Hardware->os,
        Hardware->core,
        0x000D4,
        &dataEx
        );

    if (gcmIS_ERROR(statusEx))
    {
        /*
         * Do not overwrite status here, so that former status from
         * AQIntrAck is returned.
         */
        goto OnError;
    }

    /*
     * This bit looks useless now, we can use this check if this interrupt is
     * from FE.
     */
    dataEx &= ~0x80000000;

    /*
     * Descriptor fetched, update counter.
     * We can't do this at dataEx != 0 only, because read HW acknowledge
     * register will overwrite 0x007E4. If one
     * interrupt we don't read it, we will miss it for ever.
     */
    gckASYNC_FE_UpdateAvaiable(Hardware);

    /* Do not need report NOT_OUT_INTERRUPT error if dataEx is 0. */
    if (dataEx)
    {
        statusEx = gckEVENT_Interrupt(Hardware->kernel->asyncEvent, dataEx);

        if (gcmIS_SUCCESS(statusEx))
        {
            /* At least AsyncFE is success, treat all as success. */
            status = gcvSTATUS_OK;
        }
    }

OnError:
    /* Return the status. */
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_Notify
**
**  This functions will handle the event notifications.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_Notify(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gceEVENT_FAULT fault;
    gctUINT32 pending;

    gcmkHEADER_ARG("Hardware=%p", Hardware);

    gckOS_AtomGet(Hardware->os, Hardware->kernel->eventObj->pending, (gctINT32_PTR)&pending);

    if (pending & (1 << 29))
    {
        /* Event ID 29 is not a normal event, but for invalidating pipe. */
        _ResumeWaitLinkFE(Hardware);
        pending &= ~(1 << 29);
    }

    gckOS_AtomSetMask(Hardware->kernel->eventObj->pending, pending);

    /* Handle events. */
    status = gckEVENT_Notify(Hardware->kernel->eventObj, 0, &fault);

    if (Hardware->asyncFE)
    {
        status = gckEVENT_Notify(Hardware->kernel->asyncEvent, 0, &fault);
    }

    if (fault & gcvEVENT_BUS_ERROR_FAULT)
    {
        status = gckKERNEL_Recovery(Hardware->kernel);
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryCommandBuffer
**
**  Query the command buffer alignment and number of reserved bytes.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      gctSIZE_T * Alignment
**          Pointer to a variable receiving the alignment for each command.
**
**      gctSIZE_T * ReservedHead
**          Pointer to a variable receiving the number of reserved bytes at the
**          head of each command buffer.
**
**      gctSIZE_T * ReservedTail
**          Pointer to a variable receiving the number of bytes reserved at the
**          tail of each command buffer.
*/
gceSTATUS
gckHARDWARE_QueryCommandBuffer(
    IN gckHARDWARE Hardware,
    IN gceENGINE Engine,
    OUT gctUINT32 * Alignment,
    OUT gctUINT32 * ReservedHead,
    OUT gctUINT32 * ReservedTail
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (Alignment != gcvNULL)
    {
        /* Align every 8 bytes. */
        *Alignment = 8;
    }

    if (ReservedHead != gcvNULL)
    {
        /* Reserve space for SelectPipe(). */
        *ReservedHead = 32;
    }

    if (ReservedTail != gcvNULL)
    {
        if (Engine == gcvENGINE_RENDER)
        {
            gcmkFOOTER_NO();
            return gcvSTATUS_NOT_SUPPORTED;
        }
        else
        {
            *ReservedTail = gcdBLT_FENCE_LENGTH;
        }
    }

    /* Success. */
    gcmkFOOTER_ARG("*Alignment=%lu *ReservedHead=%lu *ReservedTail=%lu",
                   gcmOPT_VALUE(Alignment), gcmOPT_VALUE(ReservedHead),
                   gcmOPT_VALUE(ReservedTail));
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_QuerySystemMemory
**
**  Query the command buffer alignment and number of reserved bytes.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      gctSIZE_T * SystemSize
**          Pointer to a variable that receives the maximum size of the system
**          memory.
**
**      gctUINT32 * SystemBaseAddress
**          Poinetr to a variable that receives the base address for system
**          memory.
*/
gceSTATUS
gckHARDWARE_QuerySystemMemory(
    IN gckHARDWARE Hardware,
    OUT gctSIZE_T * SystemSize,
    OUT gctUINT32 * SystemBaseAddress
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (SystemSize != gcvNULL)
    {
        /* Maximum system memory can be 2GB. */
        *SystemSize = 1U << 31;
    }

    if (SystemBaseAddress != gcvNULL)
    {
        /* Set system memory base address. */
        *SystemBaseAddress = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
    }

    /* Success. */
    gcmkFOOTER_ARG("*SystemSize=%lu *SystemBaseAddress=%lu",
                   gcmOPT_VALUE(SystemSize), gcmOPT_VALUE(SystemBaseAddress));
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_SetMMU
**
**  Set the page table base address.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gckMMU Mmu
**          Pointer to mmu object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_SetMMU(
    IN gckHARDWARE Hardware,
    IN gckMMU Mmu
    )
{
    gceSTATUS status;
    gctUINT32 address = 0;

    gcmkHEADER_ARG("Hardware=0x%x Mmu=0x%x", Hardware, Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (Hardware->mmuVersion == 0)
    {
        /* mmu v0 only support 1 level translation, only uses stlb level mapping. */
        gcmkSAFECASTPHYSADDRT(address, Mmu->dynamicArea4K.stlbPhysical);

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                       "Setting page table to 0x%08X",
                       address);

        /* Write the AQMemoryFePageTable register. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00400,
                                  address));

        /* Write the AQMemoryRaPageTable register. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00410,
                                  address));

        /* Write the AQMemoryTxPageTable register. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00404,
                                  address));


        /* Write the AQMemoryPePageTable register. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x00408,
                                  address));

        /* Write the AQMemoryPezPageTable register. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x0040C,
                                  address));
    }
    else
    {
        gctBOOL mmuValid = gcvTRUE;

        gcmkONERROR(gckFUNCTION_Validate(&Hardware->functions[gcvFUNCTION_EXECUTION_MMU], &mmuValid));

        if (mmuValid)
        {
            gctBOOL hwMmuDisabled = gcvTRUE;

            /* Force Disable MMU to guarantee setup command be read from physical addr */
            if (Hardware->options.secureMode == gcvSECURE_IN_NORMAL)
            {
                gctUINT32 regMmuCtrl = 0;
                gcmkONERROR(gckOS_ReadRegisterEx(
                    Hardware->os,
                    Hardware->core,
                    0x00388,
                    &regMmuCtrl
                    ));

                hwMmuDisabled = ((((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) ) == 0x1)
                           ? gcvFALSE
                           : gcvTRUE;
            }
            else
            {
                gctUINT32 regMmuCtrl = 0;

                gcmkONERROR(gckOS_ReadRegisterEx(
                    Hardware->os,
                    Hardware->core,
                    0x0018C,
                    &regMmuCtrl
                    ));

                hwMmuDisabled = ((((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) ) == 0x1)
                              ? gcvFALSE
                              : gcvTRUE;
            }

            if (hwMmuDisabled)
            {
                gcmkONERROR(gckFUNCTION_Execute(&Hardware->functions[gcvFUNCTION_EXECUTION_MMU]));

                /* Enable MMU. */
                if (Hardware->options.secureMode == gcvSECURE_IN_NORMAL)
                {
                    gctUINT32 config;

                    if (Mmu->initMode == gcvMMU_INIT_FROM_REG)
                    {
                        config = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));
                    }
                    else
                    {
                        config = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
                    }

                    gcmkONERROR(gckOS_WriteRegisterEx_NoDump(
                        Hardware->os,
                        Hardware->core,
                        0x00388,
                        config
                        ));
                }
                else
                {
                    gcmkONERROR(gckOS_WriteRegisterEx(
                        Hardware->os,
                        Hardware->core,
                        0x0018C,
                        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (gcvTRUE) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                        ));
                }
            }
        }

    }

    /* Return the status. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_FlushMMU
**
**  Flush the page table.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_FlushMMU(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT32 Address,
    IN gctUINT32 SubsequentBytes,
    IN OUT gctUINT32 * Bytes
    )
{
    gceSTATUS status;
    gctUINT32_PTR buffer;
    gctUINT32 flushSize;
    gctBOOL bltEngine = gcvFALSE;
    gctBOOL multiCluster = gcvFALSE;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    if (Hardware->mmuVersion == 0)
    {
        flushSize = 8;
    }
    else
    {
        flushSize = 10 * 4;

        bltEngine    = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_BLT_ENGINE);
        multiCluster = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_MULTI_CLUSTER);

        if (bltEngine)
        {
            flushSize +=  4 * 4;

            if (multiCluster)
            {
                flushSize += 2 * 4;
            }
        }
    }

    if (Logical)
    {
        if (*Bytes < flushSize)
        {
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }

        /* Flush the memory controller. */
        if (Hardware->mmuVersion == 0)
        {
            buffer = (gctUINT32_PTR)Logical;

            buffer[0] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E04) & ((gctUINT32) ((((1 ?
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

            buffer[1] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
        }
        else
        {
            gctUINT32 count;
            gctUINT32 offset = 2;
            gctUINT32 prefetchCount = 4;
            gctUINT32 semaphore, stall;

            if (bltEngine)
            {
                prefetchCount += 2;
                if (multiCluster)
                {
                    prefetchCount++;
                }
            }

            buffer = (gctUINT32_PTR)Logical;

            count = SubsequentBytes >> 3;

            /* LINK to next slot to flush FE FIFO. */
            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (prefetchCount) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++ = Address + offset * gcmSIZEOF(gctUINT32);

            /* Flush MMU cache. */
            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0061) & ((gctUINT32) ((((1 ?
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

            *buffer++ = (((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) &  ((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7))));

            if (bltEngine)
            {
                /* Blt lock. */
                *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                if (multiCluster)
                {
                    *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x50CE) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (Hardware->identity.clusterAvailMask & Hardware->options.userClusterMask) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)));
                }
            }

            /* Arm the PE-FE Semaphore. */
            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            semaphore = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)));

            if (Hardware->stallFEPrefetch)
            {
                semaphore |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:28) - (0 ?
 29:28) + 1))))))) << (0 ?
 29:28))) | (((gctUINT32) (0x3 & ((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:28) - (0 ? 29:28) + 1))))))) << (0 ? 29:28)));
            }

            if (bltEngine)
            {
                semaphore |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }
            else
            {
                semaphore |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }

            *buffer++ = semaphore;

            /* STALL FE until PE is done flushing. */
            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

            stall = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)));

            if (Hardware->stallFEPrefetch)
            {
               stall |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:28) - (0 ?
 29:28) + 1))))))) << (0 ?
 29:28))) | (((gctUINT32) (0x3 & ((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:28) - (0 ? 29:28) + 1))))))) << (0 ? 29:28)));
            }

            if (bltEngine)
            {
                stall |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }
            else
            {
                stall |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }

            *buffer++ = stall;

            if (bltEngine)
            {
                /* Blt unlock. */
                *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }

            /* LINK to next slot to flush FE FIFO. */
            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (count) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++ = Address + flushSize;
        }
    }

    if (Bytes)
    {
        *Bytes = flushSize;
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_FlushAsyncMMU(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN OUT gctUINT32 * Bytes
    )
{
    gctUINT32 semaphore, stall;
    gctUINT32_PTR buffer;
    gceSTATUS status;

    gcmkHEADER_ARG("Hardware=0x%x Logical=0x%x *Bytes=%lu",
                   Hardware, Logical, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT((Logical == gcvNULL) || (Bytes != gcvNULL));

    if (Logical != gcvNULL)
    {
        buffer = (gctUINT32_PTR) Logical;

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 1,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 2,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0061) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
          ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 3,
            (((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) &  ((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7))))
            ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 4,
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            ));

        semaphore = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

        if (Hardware->stallFEPrefetch)
        {
            semaphore |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:28) - (0 ?
 29:28) + 1))))))) << (0 ?
 29:28))) | (((gctUINT32) (0x3 & ((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:28) - (0 ? 29:28) + 1))))))) << (0 ? 29:28)));
        }

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 5,
            semaphore));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 6,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            ));

        stall = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

        if (Hardware->stallFEPrefetch)
        {
           stall |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:28) - (0 ?
 29:28) + 1))))))) << (0 ?
 29:28))) | (((gctUINT32) (0x3 & ((gctUINT32) ((((1 ?
 29:28) - (0 ?
 29:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:28) - (0 ? 29:28) + 1))))))) << (0 ? 29:28)));
        }

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 7,
            stall));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 8,
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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
          ));

        gcmkONERROR(gckOS_WriteMemory(
            Hardware->os,
            buffer + 9,
            ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
            ));
    }

    if (Bytes != gcvNULL)
    {
        /* Return number of bytes required by the PIPESELECT command. */
        *Bytes = 40;
    }

    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return gcvSTATUS_OK;

OnError:
    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return status;
}

gceSTATUS
gckHARDWARE_FlushMcfeMMU(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN OUT gctUINT32 * Bytes
    )
{
    gceSTATUS status;
    gctUINT32_PTR buffer;
    gctUINT32 flushSize;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* MCFE with old mmu? nonsense!. */
    gcmkASSERT(Hardware->mmuVersion > 0);

    flushSize = 4 * 6;

    if (Logical)
    {
        if (*Bytes < flushSize)
        {
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }

        buffer = (gctUINT32_PTR)Logical;

        /* Flush MMU cache. */
        *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0061) & ((gctUINT32) ((((1 ?
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

        *buffer++ = (((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) &  ((((gctUINT32) (~0U)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7))));

        /*
         * System channel can only take one command at a time, Trigger and
         * SubmitJob are not required.
         */

        /* AQHiIdle to trigger. */
        *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0001) & ((gctUINT32) ((((1 ?
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

        *buffer++ = 0;

        /* SubmitJob. */
        *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x16 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) (0x001 & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

        *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));
    }

    if (Bytes)
    {
        *Bytes = flushSize;
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_BuildVirtualAddress
**
**  Build a virtual address.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gctUINT32 Index
**          Index into page table.
**
**      gctUINT32 Offset
**          Offset into page.
**
**  OUTPUT:
**
**      gctUINT32 * Address
**          Pointer to a variable receiving te hardware address.
*/
gceSTATUS
gckHARDWARE_BuildVirtualAddress(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Index,
    IN gctUINT32 Offset,
    OUT gctUINT32 * Address
    )
{
    gcmkHEADER_ARG("Hardware=0x%x Index=%u Offset=%u", Hardware, Index, Offset);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Address != gcvNULL);

    /* Build virtual address. */
    *Address = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)))
             | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:0) - (0 ?
 30:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 30:0) - (0 ?
 30:0) + 1))))))) << (0 ?
 30:0))) | (((gctUINT32) ((gctUINT32) (Offset | (Index << 12)) & ((gctUINT32) ((((1 ?
 30:0) - (0 ?
 30:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 30:0) - (0 ? 30:0) + 1))))))) << (0 ? 30:0)));

    /* Success. */
    gcmkFOOTER_ARG("*Address=0x%08x", *Address);
    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_GetIdle(
    IN gckHARDWARE Hardware,
    IN gctBOOL Wait,
    OUT gctUINT32 * Data
    )
{
    gceSTATUS status;
    gctUINT32 idle = 0;
    gctINT retry, poll, pollCount;
    gctUINT32 address;

    gcmkHEADER_ARG("Hardware=0x%x Wait=%d", Hardware, Wait);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Data != gcvNULL);


    /* If we have to wait, try 100 polls per millisecond. */
    pollCount = Wait ? 100 : 1;

    /* At most, try for 1 second. */
    for (retry = 0; retry < 1000; ++retry)
    {
        /* If we have to wait, try 100 polls per millisecond. */
        for (poll = pollCount; poll > 0; --poll)
        {
            /* Read register. */
            gcmkONERROR(
                gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00004, &idle));

            /* Read the current FE address. */
            gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                             Hardware->core,
                                             0x00664,
                                             &address));

            gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                             Hardware->core,
                                             0x00664,
                                             &address));

            /* See if we have to wait for FE idle. */
            if (_IsHWIdle(idle, Hardware)
             && (address == Hardware->lastEnd + 8)
             )
            {
                /* FE is idle. */
                break;
            }
        }

        /* Check if we need to wait for FE and FE is busy. */
        if (Wait && !_IsHWIdle(idle, Hardware))
        {
            /* Wait a little. */
            gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                           "%s: Waiting for idle: 0x%08X",
                           __FUNCTION__, idle);

            gcmkVERIFY_OK(gckOS_Delay(Hardware->os, 1));
        }
        else
        {
            break;
        }
    }

    /* Return idle to caller. */
    *Data = idle;

#if defined(EMULATOR)
    /* Wait a little while until CModel FE gets END.
     * END is supposed to be appended by caller.
     */
    gckOS_Delay(Hardware->os, 100);
#endif

    /* Success. */
    gcmkFOOTER_ARG("*Data=0x%08x", *Data);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/* Flush the caches. */
gceSTATUS
gckHARDWARE_Flush(
    IN gckHARDWARE Hardware,
    IN gceKERNEL_FLUSH Flush,
    IN gctPOINTER Logical,
    IN OUT gctUINT32 * Bytes
    )
{
    gctUINT32 pipe;
    gctUINT32 flush = 0;
    gctUINT32 flushVST = 0;
    gctBOOL flushTileStatus;
    gctUINT32_PTR logical = (gctUINT32_PTR) Logical;
    gceSTATUS status;
    gctBOOL halti5;
    gctBOOL flushICache;
    gctBOOL flushTXDescCache;
    gctBOOL flushTFB;
    gctBOOL hwTFB;
    gctBOOL blt;
    gctBOOL peTSFlush;
    gctBOOL multiCluster;
    gctBOOL computeOnly;

    gcmkHEADER_ARG("Hardware=0x%x Flush=0x%x Logical=0x%x *Bytes=%lu",
                   Hardware, Flush, Logical, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Get current pipe. */
    pipe = Hardware->kernel->command->pipeSelect;

    halti5 = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI5);

    hwTFB = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HW_TFB);

    blt = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_BLT_ENGINE);
    multiCluster = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_MULTI_CLUSTER);

    peTSFlush = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_PE_TILE_CACHE_FLUSH_FIX);

    computeOnly = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_COMPUTE_ONLY);

    /* Flush tile status cache. */
    flushTileStatus = Flush & gcvFLUSH_TILE_STATUS;

    /* Flush Icache for halti5 hardware as we dont do it when program or context switches*/
    flushICache = (Flush & gcvFLUSH_ICACHE) && halti5;

    /* Flush texture descriptor cache */
    flushTXDescCache = Flush & gcvFLUSH_TXDESC;

    /* Flush USC cache for TFB client */
    flushTFB = (Flush & gcvFLUSH_TFBHEADER) && hwTFB;

    /* Flush TFB for vertex buffer */
    if (Flush & gcvFLUSH_VERTEX)
    {
        if (hwTFB)
        {
            flushTFB = gcvTRUE;
        }

        if (multiCluster)
        {
            flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 14:14) - (0 ?
 14:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 14:14) - (0 ?
 14:14) + 1))))))) << (0 ?
 14:14))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 14:14) - (0 ?
 14:14) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 14:14) - (0 ? 14:14) + 1))))))) << (0 ? 14:14)));
        }
    }

    /* Flush 3D color cache. */
    if ((Flush & gcvFLUSH_COLOR) && (pipe == 0x0))
    {
        flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

    /* Flush 3D depth cache. */
    if ((Flush & gcvFLUSH_DEPTH) && (pipe == 0x0))
    {
        flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
    }

    /* Flush 3D texture cache. */
    if ((Flush & gcvFLUSH_TEXTURE) && (pipe == 0x0))
    {
        flush |= multiCluster ?
 0 : ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)));
        flushVST = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
    }

    /* Flush 2D cache. */
    if ((Flush & gcvFLUSH_2D) && (pipe == 0x1))
    {
        flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)));
    }

    /* Flush L2 cache. */
    if ((Flush & gcvFLUSH_L2) && (pipe == 0x0))
    {
        flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

    /* Vertex buffer and texture could be touched by SHL1 for SSBO and image load/store */
    if ((Flush & (gcvFLUSH_VERTEX | gcvFLUSH_TEXTURE)) && (pipe == 0x0))
    {
        flush |= ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
              |  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));
    }

    /* See if there is a valid flush. */
    if ((flush == 0) &&
        (flushTileStatus == gcvFALSE) &&
        (flushICache == gcvFALSE) &&
        (flushTXDescCache == gcvFALSE) &&
        (flushTFB == gcvFALSE))
    {
        if (Bytes != gcvNULL)
        {
            /* No bytes required. */
            *Bytes = 0;
        }
    }
    else
    {
        gctBOOL appendNop = gcvFALSE;
        gctUINT32 reserveBytes = 0;
        gctBOOL txCacheFix = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_TEX_CACHE_FLUSH_FIX)
                           ? gcvTRUE : gcvFALSE;

        /* Determine reserve bytes. */
        if (!txCacheFix || flushICache || flushTXDescCache)
        {
            /* Semaphore/Stall */
            reserveBytes += blt ? (8 * gcmSIZEOF(gctUINT32)) : (4 * gcmSIZEOF(gctUINT32));
        }

        if (flush)
        {
            reserveBytes += 2 * gcmSIZEOF(gctUINT32);
        }

        if (flushVST)
        {
            reserveBytes += 2 * gcmSIZEOF(gctUINT32);
        }

        if (flushTileStatus && !computeOnly)
        {
            reserveBytes += (!peTSFlush && blt) ? 6 * gcmSIZEOF(gctUINT32) : 2 * gcmSIZEOF(gctUINT32);
        }

        if (flushICache)
        {
            reserveBytes += 2 * gcmSIZEOF(gctUINT32);
        }

        if (flushTXDescCache)
        {
            reserveBytes += 2 * gcmSIZEOF(gctUINT32);
        }

        if (flushTFB)
        {
            reserveBytes += 2 * gcmSIZEOF(gctUINT32);
        }

        /* Semaphore/Stall */
        reserveBytes += blt ? (8 * gcmSIZEOF(gctUINT32)) : (4 * gcmSIZEOF(gctUINT32));

        if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_MCFE) && (reserveBytes & 8))
        {
            appendNop = gcvTRUE;
            reserveBytes += 8;
        }

        /* Copy to command queue. */
        if (Logical != gcvNULL)
        {
            if (*Bytes < reserveBytes)
            {
                /* Command queue too small. */
                gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
            }

            if (!txCacheFix || flushICache || flushTXDescCache)
            {
                if (blt)
                {
                    /* Semaphore from FE to BLT. */
                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                    /* Stall from FE to BLT. */
                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
                }
                else
                {
                    /* Semaphore. */
                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

                    /* Stall. */
                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
                }
            }

            if (flush)
            {
                /* Append LOAD_STATE to AQFlush. */
                *logical++
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

                *logical++ = flush;

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE, "0x%x: FLUSH 0x%x", logical - 1, flush);
            }

            if (flushVST)
            {
                /* Append LOAD_STATE to AQFlush. */
                *logical++
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

                *logical++ = flushVST;

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE, "0x%x: FLUSH 0x%x", logical - 1, flush);
            }

            if (flushTileStatus && !computeOnly)
            {
                if (!peTSFlush && blt)
                {
                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502B) & ((gctUINT32) ((((1 ?
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

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                    *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
                }
                else
                {
                    *logical++
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

                    *logical++
                        = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
                }

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                               "0x%x: FLUSH TILE STATUS 0x%x", logical - 1, logical[-1]);
            }

            if (flushICache)
            {
                *logical++
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

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                               "0x%x: FLUSH Icache 0x%x", logical - 1, logical[-1]);

            }

            if (flushTXDescCache)
            {
                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x5311) & ((gctUINT32) ((((1 ?
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

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:28) - (0 ?
 31:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:28) - (0 ?
 31:28) + 1))))))) << (0 ?
 31:28))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:28) - (0 ?
 31:28) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:28) - (0 ? 31:28) + 1))))))) << (0 ? 31:28)));

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                               "0x%x: FLUSH Icache 0x%x", logical - 1, logical[-1]);

            }

            if (flushTFB)
            {
                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x7003) & ((gctUINT32) ((((1 ?
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

                *logical++
                    = 0x1;

                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                               "0x%x: FLUSH TFB cache 0x%x", logical - 1, logical[-1]);

            }

            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to BLT. */
                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }
            else
            {
                /* Semaphore. */
                *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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

                /* Stall. */
                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
            }
            if (appendNop)
            {
                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *logical++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));
            }
        }

        if (Bytes != gcvNULL)
        {
            /* bytes required. */
            *Bytes = reserveBytes;
        }
    }

    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_SetFastClear(
    IN gckHARDWARE Hardware,
    IN gctINT Enable,
    IN gctINT Compression
    )
{
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_PowerControlClusters
**
**  Power control clusters of one core.
**  Currently only support NN clusters.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gctUINT32 PowerControlVaule
**          The value programmed to power control register.
**
**      gctBOOL PowerState
**          Power State to switch.
**
*/

gceSTATUS
gckHARDWARE_PowerControlClusters(
    gckHARDWARE Hardware,
    gctUINT32  PowerControlValue,
    gctBOOL PowerState
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckCOMMAND command = gcvNULL;
    gctPOINTER buffer = gcvNULL;
    gctUINT32_PTR logical = gcvNULL;
    gctUINT32 reqBytes = 16;
    gctUINT32 bytes;
    gctUINT32 idle, timer = 0;

    gcmkHEADER_ARG("Hardware=%p PowerControlValue=%x", Hardware, PowerControlValue);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    command = Hardware->kernel->command;

#if !gcdFPGA_BUILD
    if (Hardware->identity.customerID != 0x85 || !Hardware->mcFE ||
        Hardware->options.enableNNClusters == ~0UL ||
        (!PowerState && !Hardware->powerState))
#endif
    {
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    /* Program cluster power control command, only with MCFE by current. */
    {
        /* Start the command parser. */
        gcmkONERROR(gckCOMMAND_Start(command));

        /* Reserve space in the command queue. */
        gcmkONERROR(gckCOMMAND_Reserve(command,
                                       reqBytes,
                                       &buffer,
                                       &bytes));

        logical = (gctUINT32_PTR)buffer;

        *logical++
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

        *logical++
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:8) - (0 ?
 11:8) + 1))))))) << (0 ?
 11:8))) | (((gctUINT32) ((gctUINT32) (PowerControlValue) & ((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:8) - (0 ? 11:8) + 1))))))) << (0 ? 11:8)));

        *logical++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));
        *logical++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

        gcmkONERROR(gckCOMMAND_ExecuteMultiChannel(command, gcvFALSE, 2, reqBytes));

        do
        {
            gckOS_Udelay(Hardware->os, 10);

            gcmkONERROR(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                0x00004,
                &idle));

            timer += 1;

#if gcdGPU_TIMEOUT
            if (timer >= Hardware->kernel->timeOut)
            {
                gcmkPRINT("%s %d Galcore timeout...\n", __FUNCTION__, __LINE__);

                gcmkONERROR(gcvSTATUS_DEVICE);
            }
#endif
        }
        while (!_IsHWIdle(idle, Hardware));
        gcmkDUMP(Hardware->os, "@[register.wait 0x%05X 0x%08X 0x%08X]",
                 0x00004,
                 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))),
                 idle);

        /* Stop the command parser. */
        gcmkONERROR(gckCOMMAND_Stop(command));
    }

OnError:
    gcmkFOOTER();
    return status;
}

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
static gctCONST_STRING
_PowerEnum(gceCHIPPOWERSTATE State)
{
    const gctCONST_STRING baseStates[] =
    {
        "ON",
        "IDLE",
        "SUSPEND",
        "OFF",
        "ON[auto]",
    };

    const gctCONST_STRING broadcastStates[] =
    {
        "",
        "IDLE[broadcast]",
        "SUSPEND[broadcast]",
        "OFF[broadcast]",
    };

    const gctCONST_STRING timeoutStates[] =
    {
        "",
        "IDLE[timeout]",
        "SUSPEND[timeout]",
        "OFF[timeout]",
    };

    gcmSTATIC_ASSERT(gcvPOWER_ON == 0 && gcvPOWER_IDLE == 1 &&
                     gcvPOWER_SUSPEND == 2 && gcvPOWER_OFF == 3 &&
                     gcvPOWER_ON_AUTO == 4,
                     "array subscript does not match");

    if (State & gcvPOWER_FLAG_BROADCAST)
    {
        return broadcastStates[State & ~gcvPOWER_FLAG_BROADCAST];
    }
    else if (State & gcvPOWER_FLAG_TIMEOUT)
    {
        return timeoutStates[State & ~gcvPOWER_FLAG_TIMEOUT];
    }
    else if ((State >= gcvPOWER_ON) && (State <= gcvPOWER_ON_AUTO))
    {
        return baseStates[State - gcvPOWER_ON];
    }

    return "unknown";
}
#endif

static gceSTATUS
_PmClockOn(
    IN gckHARDWARE Hardware,
    OUT gctBOOL * RequireInit
    )
{
    gceSTATUS status;

    /* Turn on the power. */
    gcmkONERROR(gckOS_SetGPUPower(Hardware->os, Hardware->core,
                                  gcvTRUE, gcvTRUE));

    Hardware->clockState = Hardware->powerState = gcvTRUE;

    /* Check if GPU is present and awake. */
    while (_IsGPUPresent(Hardware) == gcvSTATUS_GPU_NOT_RESPONDING)
    {
        /* Turn off the power and clock. */
        gcmkONERROR(gckOS_SetGPUPower(Hardware->os, Hardware->core,
                                      gcvFALSE, gcvFALSE));

        Hardware->clockState = Hardware->powerState = gcvFALSE;

        /* Wait a little. */
        gckOS_Delay(Hardware->os, 1);

        /* Turn on the power and clock. */
        gcmkONERROR(gckOS_SetGPUPower(Hardware->os, Hardware->core,
                                      gcvTRUE, gcvTRUE));

        Hardware->clockState = Hardware->powerState = gcvTRUE;

        if (RequireInit)
        {
            *RequireInit = gcvTRUE;
        }
    }

OnError:
    return status;
}

static gceSTATUS
_PmClockOff(
    IN gckHARDWARE Hardware,
    IN gctBOOL PowerState
    )
{
    gceSTATUS status;

    gcmkONERROR(gckOS_SetGPUPower(Hardware->os, Hardware->core,
                                  gcvFALSE, PowerState));

    Hardware->clockState = gcvFALSE;
    Hardware->powerState = PowerState;

OnError:
    return status;
}

static gceSTATUS
_PmClockControl(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State
    )
{
    gceSTATUS status;
    gctUINT32 clock;

    static const gctUINT clocks[4] =
    {
        /* gcvPOWER_ON */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (64) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9))),

        /* gcvPOWER_IDLE */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9))),

        /* gcvPOWER_SUSPEND */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9))),

        /* gcvPOWER_OFF */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2))) |
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9))),

    };

    clock = clocks[State];

    if (Hardware->identity.customerID == 0xc6)
        return gcvSTATUS_OK;


#if gcdENABLE_FSCALE_VAL_ADJUST
    if (State == gcvPOWER_ON)
    {
        clock = ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (Hardware->powerOnFscaleVal) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2)));
    }
#endif

    if (Hardware->clockState && Hardware->powerState
#if gcdDVFS
    /* Don't touch clock control if dynamic frequency scaling is available. */
    && !gckHARDWARE_IsFeatureAvailable(Hardware,
                                       gcvFEATURE_DYNAMIC_FREQUENCY_SCALING)
#endif
    )
    {
        if ((State == gcvPOWER_OFF) &&
            (Hardware->identity.chipModel == gcv4000) &&
            ((Hardware->identity.chipRevision == 0x5208) ||
             (Hardware->identity.chipRevision == 0x5222)))
        {
            clock &= ~2U;
        }

        /* Write the clock control register. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core,
                                          0x00000,
                                          clock));

        clock = ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)));

        /* Done loading the frequency scaler. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core,
                                          0x00000,
                                          clock));
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_PmInitializeGPU(
    IN gckHARDWARE Hardware,
    IN gckCOMMAND Command
    )
{
    gceSTATUS status;

    gctBOOL hwMmuDisabled =  gcvTRUE;

    /* VIV for 8MM_EVK, maybe power off is failed, the GPU still has been power on,
    so we need to check the mmu enable flag to see if we need to dummy draw */
    if (_IsHardwareMatch(Hardware, gcv600, 0x4653) || _IsHardwareMatch(Hardware, gcv600, 0x4633))
    {
        if (Hardware->options.secureMode == gcvSECURE_IN_NORMAL)
        {
            gctUINT32 regMmuCtrl = 0;
            gcmkONERROR(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                0x00388,
                &regMmuCtrl
                ));

            hwMmuDisabled = ((((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) ) == 0x1)
                       ? gcvFALSE
                       : gcvTRUE;
        }
        else
        {
            gctUINT32 regMmuCtrl = 0;

            gcmkONERROR(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                0x0018C,
                &regMmuCtrl
                ));

            hwMmuDisabled = ((((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) ) == 0x1)
                          ? gcvFALSE
                          : gcvTRUE;
        }
    }

    if(hwMmuDisabled)
    {
        Command->dummyDraw = gcvTRUE;
    }

    /* Initialize hardware. */
    gcmkONERROR(gckHARDWARE_InitializeHardware(Hardware));

    gcmkONERROR(gckHARDWARE_SetFastClear(Hardware,
                                         Hardware->options.allowFastClear,
                                         Hardware->options.allowCompression));

    /* Force the command queue to reload the next context. */
    Command->currContext = gcvNULL;

OnError:
    return status;
}

/*
 * Notice: this function may return gcvSTATUS_NOT_READY, which is not an error,
 * but either not success.
 */
static gceSTATUS
_PmStallCommand(
    gckHARDWARE Hardware,
    gckCOMMAND Command,
    gctBOOL Broadcast
    )
{
    gceSTATUS status;
    gctBOOL idle;

    if (Broadcast)
    {
        /* Check for idle. */
        gcmkONERROR(gckHARDWARE_QueryIdle(Hardware, &idle));

        if (!idle)
        {
            status = gcvSTATUS_CHIP_NOT_READY;
            goto OnError;
        }
    }
    else
    {
        /* Wait to finish all commands. */
        status = gckCOMMAND_Stall(Command, gcvTRUE);

        if (!gcmIS_SUCCESS(status))
        {
            goto OnError;
        }

        for (;;)
        {
            gcmkONERROR(gckHARDWARE_QueryIdle(Hardware, &idle));

            if (idle)
            {
                break;
            }

            gcmkVERIFY_OK(gckOS_Delay(Hardware->os, 1));
        }
    }

OnError:
    return status;
}

static gceSTATUS
_PmFlushCache(
    gckHARDWARE Hardware,
    gckCOMMAND Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (Hardware->clockState == gcvFALSE)
    {
        /* Turn on the GPU power. */
        gcmkONERROR(gckOS_SetGPUPower(Hardware->os, Hardware->core,
                                      gcvTRUE, gcvTRUE));

        Hardware->clockState = gcvTRUE;

        /* Clock control, to ON state. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_ON));
    }

    if (_IsHardwareMatch(Hardware, gcv400, 0x4645))
    {
        gcmkONERROR(gckCOMMAND_Start(Command));

        gcmkONERROR(_FlushCache(Hardware, Command));
        gckOS_Delay(gcvNULL, 1);

        /* Stop the command parser. */
        gcmkONERROR(gckCOMMAND_Stop(Command));
    }
    else
    {
        gcmkONERROR(gckFUNCTION_Execute(&Hardware->functions[gcvFUNCTION_EXECUTION_FLUSH]));
    }

OnError:
    return status;
}

/*
 * Put power to on direction:
 * Off -> Suspend
 * Off            -> Idle
 * Off                    -> On
 *        Suspend -> Idle
 *        Suspend         -> On
 *                   Idle -> On
 */
static gceSTATUS
_PmSetPowerOnDirection(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State
    )
{
    gceSTATUS status;
    gckCOMMAND command = Hardware->kernel->command;
    gctBOOL clockOn = gcvFALSE;
    gctBOOL requireInit = gcvFALSE;

    switch (Hardware->chipPowerState)
    {
    case gcvPOWER_OFF:
        if (State == gcvPOWER_SUSPEND)
        {
            gcmkONERROR(_PmClockOn(Hardware, gcvNULL));
            clockOn = gcvTRUE;

            /* Clock control, put to suspend. */
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_SUSPEND));

            /* Initialize GPU. */
            gcmkONERROR(_PmInitializeGPU(Hardware, command));

            /* Suspend: clock off, power on. */
            gcmkONERROR(_PmClockOff(Hardware, gcvTRUE));
            break;
        }

        requireInit = gcvTRUE;
        /* FALLTHRU */

    case gcvPOWER_SUSPEND:
        /* Clock on. */
        gcmkONERROR(_PmClockOn(Hardware, &requireInit));
        clockOn = gcvTRUE;

        /* Clock control, put to target state (On or idle). */
        gcmkONERROR(_PmClockControl(Hardware, State));

        /* Delay. */
        gcmkONERROR(gckOS_Delay(Hardware->os, gcdPOWER_CONTROL_DELAY));

        if (requireInit)
        {
            /* Initialize. */
            gcmkONERROR(_PmInitializeGPU(Hardware, command));
        }

        /* Start. */
        gcmkONERROR(gckCOMMAND_Start(command));
        break;

    case gcvPOWER_IDLE:
        /* Clock control, put to ON state. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_ON));
        break;

    default:
        break;
    }

    return gcvSTATUS_OK;

OnError:
    if (clockOn)
    {
        gctBOOL powerState =
            (Hardware->chipPowerState == gcvPOWER_SUSPEND);

        _PmClockOff(Hardware, powerState);
    }

    return status;
}

/*
 * Put power to off direction:
 * On -> Idle
 * On         -> Suspend
 * On                    -> Off
 *       Idle -> Suspend
 *       Idle            -> Off
 *               Suspend -> Off
 */
static gceSTATUS
_PmSetPowerOffDirection(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State,
    IN gctBOOL Broadcast
    )
{
    gceSTATUS status;
    gckCOMMAND command = Hardware->kernel->command;

    switch (Hardware->chipPowerState)
    {
    case gcvPOWER_ON:
        if (Hardware->kernel->threadInitialized == gcvTRUE)
        {
            /* Stall. */
            status = _PmStallCommand(Hardware, command, Broadcast);

            if (!gcmIS_SUCCESS(status))
            {
                /* abort for error and NOT READY. */
                goto OnError;
            }
        }

        if (State == gcvPOWER_IDLE)
        {
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_IDLE));
            break;
        }
        /* FALLTHRU */

    case gcvPOWER_IDLE:
        /* Stop. */
        gcmkONERROR(gckCOMMAND_Stop(command));

        if (State == gcvPOWER_SUSPEND)
        {
            /* Clock control, put to suspend state. */
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_SUSPEND));

            /* Power on, clock off. */
            gcmkONERROR(_PmClockOff(Hardware, gcvTRUE));
            break;
        }

        /* FALLTHRU */

    case gcvPOWER_SUSPEND:
        if (Hardware->kernel->threadInitialized == gcvTRUE)
        {
            /* Flush. */
            gcmkONERROR(_PmFlushCache(Hardware, command));
        }

        gcmkONERROR(gckHARDWARE_PowerControlClusters(Hardware, 0x2, gcvFALSE));

        /* Clock control. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_OFF));

        /* Power off, clock off. */
        gcmkONERROR(_PmClockOff(Hardware, gcvFALSE));

        break;

    default:
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

/******************************************************************************\
****************************** Qchannel Power Management *****************************
\******************************************************************************/

gceSTATUS
gckHARDWARE_QchannelPowerControl(
    IN gckHARDWARE Hardware,
    IN gctBOOL ClockState,
    IN gctBOOL PowerState
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 reg = 0, delay = 1;
    gctBOOL powerChange = gcvFALSE;
    gctBOOL clockChange = gcvFALSE;

    gcmkHEADER_ARG("Hardware=%p", Hardware);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    powerChange = (PowerState != Hardware->powerState);
    clockChange = (ClockState != Hardware->clockState);

    if (clockChange)
    {
        gcmkVERIFY_OK(gckOS_SetClockState(Hardware->os, Hardware->core, ClockState));
    }

    if (powerChange && PowerState == gcvTRUE)
    {
        gctBOOL state;

        gcmkVERIFY_OK(gckOS_GetClockState(Hardware->os, Hardware->core, &state));
        gcmkVERIFY_OK(gckOS_SetClockState(Hardware->os, Hardware->core, gcvTRUE));

        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  0x005E8,
                                  ((((gctUINT32) (0x00000000)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));

        do
        {
            gckOS_Delay(Hardware->os, delay);

            gcmkVERIFY_OK(
                gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x005E4,
                                     &reg));
            delay *= 2;

        } while(!(reg & 0x1));

        gcmkVERIFY_OK(gckOS_SetClockState(Hardware->os, Hardware->core, state));
    }

    if (powerChange && PowerState == gcvFALSE)
    {
        gctBOOL state;

        gcmkVERIFY_OK(gckOS_GetClockState(Hardware->os, Hardware->core, &state));
        gcmkVERIFY_OK(gckOS_SetClockState(Hardware->os, Hardware->core, gcvTRUE));

        gcmkONERROR(
           gckOS_WriteRegisterEx(Hardware->os,
                                 Hardware->core,
                                 0x005E8,
                                 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))));

        gcmkVERIFY_OK(gckOS_SetClockState(Hardware->os, Hardware->core, state));
    }

    Hardware->clockState = ClockState;
    Hardware->powerState = PowerState;

OnError:
    gcmkFOOTER();
    return status;
}


gceSTATUS
gckHARDWARE_QchannelBypass(
    IN gckHARDWARE Hardware,
    IN gctBOOL Enable
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Hardware=%p", Hardware);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os,
                              Hardware->core,
                              0x005E8,
                              ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) ((gctUINT32) (Enable) & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))));

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_QchannelFlushCache(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Hardware=%p", Hardware);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    if (Hardware->clockState == gcvFALSE)
    {
        /* Turn on the GPU power. */
        gcmkONERROR(
            gckHARDWARE_QchannelPowerControl(Hardware, gcvTRUE, gcvTRUE));

        /* Clock control, to ON state. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_ON));
    }

    gcmkONERROR(gckFUNCTION_Execute(&Hardware->functions[gcvFUNCTION_EXECUTION_FLUSH]));

OnError:
    gcmkFOOTER();

    return status;
}

static gceSTATUS
_QchannelPowerOnDirection(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State
    )
{
    gceSTATUS status;
    gckCOMMAND command = Hardware->kernel->command;
    gctBOOL clockOn = gcvFALSE;
    gctBOOL requireInit = gcvTRUE;

    switch (Hardware->chipPowerState)
    {
    case gcvPOWER_OFF:
        if (State == gcvPOWER_SUSPEND)
        {
            gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvTRUE, gcvTRUE));
            clockOn = gcvTRUE;

            /* Clock control, put to suspend. */
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_SUSPEND));

            /* Initialize GPU. */
            gcmkONERROR(_PmInitializeGPU(Hardware, command));

            /* Suspend: clock off, power on. */
            gcmkONERROR(_PmClockOff(Hardware, gcvTRUE));
            break;
        }

        requireInit = gcvTRUE;
        /* FALLTHRU */

    case gcvPOWER_SUSPEND:
        /* Power on, clock on. */
        gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvTRUE, gcvTRUE));

        clockOn = gcvTRUE;

        /* Clock control, put to target state (On or idle). */
        gcmkONERROR(_PmClockControl(Hardware, State));

        /* Delay. */
        gcmkONERROR(gckOS_Delay(Hardware->os, gcdPOWER_CONTROL_DELAY));

        if (requireInit)
        {
            /* Initialize. */
            gcmkONERROR(_PmInitializeGPU(Hardware, command));
        }

        /* Start. */
        gcmkONERROR(gckCOMMAND_Start(command));
        break;

    case gcvPOWER_IDLE:
        /* Clock control, put to ON state. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_ON));

        break;

    default:
        break;
    }

    return gcvSTATUS_OK;

OnError:
    if (clockOn)
    {
        gctBOOL powerState =
            (Hardware->chipPowerState == gcvPOWER_SUSPEND);

        gcmkVERIFY_OK(gckHARDWARE_QchannelPowerControl(Hardware, gcvFALSE, powerState));
    }

    return status;
}


static gceSTATUS
_QchannelPowerOffDirection(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State,
    IN gctBOOL Broadcast
    )
{
    gceSTATUS status;
    gckCOMMAND command = Hardware->kernel->command;

    switch (Hardware->chipPowerState)
    {
    case gcvPOWER_ON:
        if (Hardware->kernel->threadInitialized == gcvTRUE)
        {
            /* Stall. */
            status = _PmStallCommand(Hardware, command, Broadcast);

            if (!gcmIS_SUCCESS(status))
            {
                /* abort for error and NOT READY. */
                goto OnError;
            }
        }

        if (State == gcvPOWER_IDLE)
        {
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_IDLE));
            break;
        }
        /* FALLTHRU */

    case gcvPOWER_IDLE:
        /* Stop. */
        gcmkONERROR(gckCOMMAND_Stop(command));

        if (State == gcvPOWER_SUSPEND)
        {
            /* Clock control, put to suspend state. */
            gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_SUSPEND));

            /* Power on, clock off. */
            gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvFALSE, gcvTRUE));
            break;
        }

        /* FALLTHRU */

    case gcvPOWER_SUSPEND:
        if (Hardware->kernel->threadInitialized == gcvTRUE)
        {
            /* Flush. */
            gcmkONERROR(gckHARDWARE_QchannelFlushCache(Hardware));
        }

        /* Clock control, put to off state. */
        gcmkONERROR(_PmClockControl(Hardware, gcvPOWER_OFF));

        /* Power off, clock off. */
        gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvFALSE, gcvFALSE));

        break;

    default:
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}
/*******************************************************************************
**
**  gckHARDWARE_SetPowerState
**
**  Set GPU to a specified power state.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gceCHIPPOWERSTATE State
**          Power State.
**
*/
gceSTATUS
gckHARDWARE_SetPowerState(
    IN gckHARDWARE Hardware,
    IN gceCHIPPOWERSTATE State
    )
{
    gceSTATUS status;
    gckCOMMAND command = gcvNULL;
    gckOS os;
    gctBOOL acquired = gcvFALSE;
    gctBOOL mutexAcquired = gcvFALSE;
    gctBOOL broadcast = gcvFALSE;
    gctBOOL timeout = gcvFALSE;
    gceCHIPPOWERSTATE state = gcvPOWER_INVALID;

    /*
     * Acquire globalSempahore when set to to global OFF, IDLE, SUSPEND, then
     * can not switch to non-global state unless global ON comes and release
     * the globalSemaphore.
     */
    gctBOOL global = gcvFALSE;
    gctBOOL globalAcquired = gcvFALSE;

    gcmkHEADER_ARG("Hardware=0x%x State=%d", Hardware, State);

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                   "Switching to power state %d(%s)",
                   State, _PowerEnum(State));
#endif

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    os = Hardware->os;
    command = Hardware->kernel->command;

    /* Convert power state. */
    switch (State)
    {
    case gcvPOWER_ON:
    case gcvPOWER_OFF:
    case gcvPOWER_IDLE:
    case gcvPOWER_SUSPEND:
        global = gcvTRUE;
        state  = State;
        break;

    case gcvPOWER_ON_AUTO:
        state = gcvPOWER_ON;
        break;

    case gcvPOWER_OFF_TIMEOUT:
    case gcvPOWER_IDLE_TIMEOUT:
    case gcvPOWER_SUSPEND_TIMEOUT:
        timeout   = gcvTRUE;
        /* FALLTHRU */
    case gcvPOWER_OFF_BROADCAST:
    case gcvPOWER_IDLE_BROADCAST:
    case gcvPOWER_SUSPEND_BROADCAST:
        broadcast = gcvTRUE;
        state     = State & ~(gcvPOWER_FLAG_BROADCAST | gcvPOWER_FLAG_TIMEOUT);
        break;

    case gcvPOWER_INVALID:
        /* Canceled, nothing to do. */
        status = gcvSTATUS_OK;
        goto OnError;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (global == gcvFALSE &&
        Hardware->options.powerManagement == gcvFALSE &&
        (Hardware->chipPowerState == gcvPOWER_ON || state != gcvPOWER_ON))
    {
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    if (broadcast)
    {
        /* Try to acquire the power mutex. */
        status = gckOS_AcquireMutex(os, Hardware->powerMutex, 0);

        if (status == gcvSTATUS_TIMEOUT)
        {
            /* Pm in progress, abort. */
            gcmkFOOTER_NO();
            return gcvSTATUS_OK;
        }
    }
    else
    {
        /* Acquire the power mutex. */
        status = gckOS_AcquireMutex(os, Hardware->powerMutex, gcvINFINITE);
    }

    gcmkONERROR(status);
    mutexAcquired = gcvTRUE;

    if (Hardware->chipPowerState == state)
    {
        /* No state change. */
        gckOS_ReleaseMutex(os, Hardware->powerMutex);
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    if (broadcast &&
        state == gcvPOWER_SUSPEND && Hardware->chipPowerState == gcvPOWER_OFF)
    {
        /* Do nothing, donot change chipPowerState. */
        gckOS_ReleaseMutex(os, Hardware->powerMutex);
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    if (timeout)
    {
        if (Hardware->nextPowerState == gcvPOWER_INVALID)
        {
            /* delayed power state change is canceled. */
            gckOS_ReleaseMutex(os, Hardware->powerMutex);
            gcmkFOOTER_NO();
            return gcvSTATUS_OK;
        }
    }

    if (global)
    {
        if (state != gcvPOWER_ON)
        {
            /*
             * Switch to global non-ON (OFF, IDLE or SUSPEND), try acquire the
             * global semaphore if it has not been acquired.
             */
            status = gckOS_TryAcquireSemaphore(os, Hardware->globalSemaphore);

            if (status == gcvSTATUS_OK)
            {
                globalAcquired = gcvTRUE;
            }
            else if (status == gcvSTATUS_TIMEOUT)
            {
                /*
                 * Ignore and leave globalSemaphore not acquired in this try.
                 * In this condition, power state is changing between global
                 * OFF, SUSPEND and IDLE, which is allowed.
                 */
                gcmkASSERT(Hardware->chipPowerState != gcvPOWER_ON);
            }
            else
            {
                /* Other errors. */
                gcmkONERROR(status);
            }
        }
    }
    else
    {
        /* Try to acquire the global semaphore. */
        status = gckOS_TryAcquireSemaphore(os, Hardware->globalSemaphore);

        if (status == gcvSTATUS_TIMEOUT)
        {
            /* In global SUSPEND, IDLE, or OFF state. */
            gcmkONERROR(gckOS_ReleaseMutex(os, Hardware->powerMutex));
            mutexAcquired = gcvFALSE;

            if (broadcast)
            {
                /* Abort power state change. */
                gcmkFOOTER_NO();
                return gcvSTATUS_OK;
            }

            /*
             * Only ON_AUTO can run here.
             * ON_AUTO state can not be skipped, either can not change a global
             * state.
             * So we need to wait until global ON state here.
             */
            gcmkONERROR(gckOS_AcquireSemaphore(os, Hardware->globalSemaphore));
            globalAcquired = gcvTRUE;

            /* Acquire the power mutex. */
            gcmkONERROR(gckOS_AcquireMutex(os,
                                           Hardware->powerMutex,
                                           gcvINFINITE));
            mutexAcquired = gcvTRUE;

            if (Hardware->chipPowerState == state)
            {
                /* Done. */
                status = gcvSTATUS_OK;
                goto OnError;
            }
        }
        else
        {
            /* Check error. */
            gcmkONERROR(status);
        }

        /* We just check if in non-global state, but need not to acquire it. */
        gcmkONERROR(gckOS_ReleaseSemaphore(os, Hardware->globalSemaphore));
        globalAcquired = gcvFALSE;
    }

    if (Hardware->chipPowerState == gcvPOWER_ON)
    {
        /* Switch from power ON to other non-runnable states. */
        if (broadcast)
        {
            gctINT32 atomValue;

            /* Try to acquire the semaphore to block/sync with commit. */
            status = gckOS_TryAcquireSemaphore(os, command->powerSemaphore);

            if (gcmIS_ERROR(status))
            {
                status = gcvSTATUS_CHIP_NOT_READY;
                goto OnError;
            }

            acquired = gcvTRUE;

            /* Check commit atom, abort when commit is in progress. */
            gcmkONERROR(gckOS_AtomGet(Hardware->os,
                                      command->atomCommit,
                                      &atomValue));

            if (atomValue > 0)
            {
                status = gcvSTATUS_CHIP_NOT_READY;
                goto OnError;
            }
        }
        else
        {
            /* Acquire/wait the semaphore to block/sync with command commit. */
            gcmkONERROR(gckOS_AcquireSemaphore(os, command->powerSemaphore));
            acquired = gcvTRUE;
        }
    }

    /* Do hardware power state change. */
    if (Hardware->chipPowerState < state)
    {
        /* On to off direction. */
        if (Hardware->hasQchannel)
        {
            gcmkONERROR(_QchannelPowerOffDirection(Hardware, state, broadcast));
        }
        else
        {
            gcmkONERROR(_PmSetPowerOffDirection(Hardware, state, broadcast));
        }
    }
    else
    {
        /* Off to on direction. */
        if (Hardware->hasQchannel)
        {
            gcmkONERROR(_QchannelPowerOnDirection(Hardware, state));
        }
        else
        {
            gcmkONERROR(_PmSetPowerOnDirection(Hardware, state));
        }
    }

    if (status == gcvSTATUS_CHIP_NOT_READY)
    {
        /* CHIP_NOT_READY is not an error, either not success. */
        goto OnError;
    }

    if (state == gcvPOWER_ON)
    {
        /* Switched to power ON from other non-runnable states. */
        gcmkONERROR(gckOS_ReleaseSemaphore(os, command->powerSemaphore));
        acquired = gcvFALSE;

        if (global)
        {
            /*
             * Global semaphore should be acquired already before, when
             * global OFF, IDLE or SUSPEND.
             */
            status = gckOS_TryAcquireSemaphore(os, Hardware->globalSemaphore);
            if (status != gcvSTATUS_TIMEOUT && Hardware->isLastPowerGlobal)
            {
                gcmkPRINT("%s: global state error", __FUNCTION__);
            }

            /* Switched to global ON, now release the global semaphore. */
            gcmkONERROR(gckOS_ReleaseSemaphore(os, Hardware->globalSemaphore));
            globalAcquired = gcvFALSE;
        }
    }

    gckSTATETIMER_Accumulate(&Hardware->powerStateCounter,
                             Hardware->chipPowerState);

    /* Save the new power state. */
    Hardware->chipPowerState = state;
    Hardware->isLastPowerGlobal = global;

#if gcdDVFS
    if (state == gcvPOWER_ON && Hardware->kernel->dvfs)
    {
        gckDVFS_Start(Hardware->kernel->dvfs);
    }
#endif

    if (!broadcast)
    {
        /*
         * Cancel delayed power state change.
         * Stop timer is not as good as set as no state change. Timer may run
         * into this function already when try to to stop the timer.
         */
        Hardware->nextPowerState = gcvPOWER_INVALID;
    }

    if (Hardware->powerOffTimeout && (state == gcvPOWER_IDLE || state == gcvPOWER_SUSPEND))
    {
        /* Delayed power off. */
        Hardware->nextPowerState = gcvPOWER_OFF_TIMEOUT;

        /* Start a timer to power off GPU when GPU enters IDLE or SUSPEND. */
        gcmkVERIFY_OK(gckOS_StartTimer(os, Hardware->powerStateTimer, Hardware->powerOffTimeout));
    }

    /* Release the power mutex. */
    gcmkONERROR(gckOS_ReleaseMutex(os, Hardware->powerMutex));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        /* Release semaphore. */
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(Hardware->os,
                                             command->powerSemaphore));
    }

    if (globalAcquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(Hardware->os,
                                             Hardware->globalSemaphore));
    }

    if (mutexAcquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryPowerState
**
**  Get GPU power state.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gceCHIPPOWERSTATE* State
**          Power State.
**
*/
gceSTATUS
gckHARDWARE_QueryPowerState(
    IN gckHARDWARE Hardware,
    OUT gceCHIPPOWERSTATE* State
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(State != gcvNULL);

    /* Return the statue. */
    *State = Hardware->chipPowerState;

    /* Success. */
    gcmkFOOTER_ARG("*State=%d", *State);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryPowerManagement
**
**  Query GPU power management function.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      gctBOOL *Enable
**          Power Mangement Enabling State.
**
*/
gceSTATUS
gckHARDWARE_QueryPowerManagement(
    IN gckHARDWARE Hardware,
    OUT gctBOOL *Enable
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (_IsHardwareMatch(Hardware, gcv7000, 0x6008))
    {
        *Enable = gcvFALSE;
    }
    else
    {
        gcmkVERIFY_OK(gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE));

        *Enable = Hardware->options.powerManagement;

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex));
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_EnablePowerManagement
**
**  Configure GPU power management function.
**  Only used in driver initialization stage.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gctBOOL Enable
**          Power Mangement Enabling State.
**
*/
gceSTATUS
gckHARDWARE_EnablePowerManagement(
    IN gckHARDWARE Hardware,
    IN gctBOOL Enable
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    gcmkVERIFY_OK(
        gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE));

    Hardware->options.powerManagement = Enable;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_SetGpuProfiler
**
**  Configure GPU profiler function.
**  Only used in driver initialization stage.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**      gctBOOL GpuProfiler
**          GOU Profiler State.
**
*/
gceSTATUS
gckHARDWARE_SetGpuProfiler(
    IN gckHARDWARE Hardware,
    IN gctBOOL GpuProfiler
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (GpuProfiler == gcvTRUE)
    {
        gctUINT32 data = 0;

        /* Need to disable clock gating when doing profiling. */
        gcmkVERIFY_OK(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress +
                                 0x00100,
                                 &data));

        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));


        gcmkVERIFY_OK(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00100,
                                  data));
    }
    else
    {
        gctUINT32 data = 0;

        /* enable clock gating when disable profile. */
        gcmkVERIFY_OK(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress +
                                 0x00100,
                                 &data));

        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

        gcmkVERIFY_OK(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00100,
                                  data));
    }

    if (GpuProfiler == gcvTRUE)
    {
        Hardware->waitCount = 200 * 100;
    }
    else
    {
        Hardware->waitCount = 200;
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

#if gcdENABLE_FSCALE_VAL_ADJUST
gceSTATUS
gckHARDWARE_SetFscaleValue(
    IN gckHARDWARE Hardware,
    IN gctUINT32   FscaleValue,
    IN gctUINT32   ShaderFscaleValue
    )
{
    gceSTATUS status;
    gctUINT32 clock;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Hardware=0x%x FscaleValue=%d", Hardware, FscaleValue);

    gcmkVERIFY_ARGUMENT(FscaleValue > 0 && FscaleValue <= 64);

    gcmkONERROR(
        gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE));
    acquired =  gcvTRUE;

    Hardware->powerOnFscaleVal = FscaleValue;

    if (Hardware->chipPowerState == gcvPOWER_ON)
    {
        gctUINT32 data;

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
                                 Hardware->core,
                                 Hardware->powerBaseAddress
                                 + 0x00104,
                                 &data));

        /* Disable all clock gating. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00104,
                                  ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:6) - (0 ?
 6:6) + 1))))))) << (0 ?
 6:6))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:6) - (0 ?
 6:6) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:6) - (0 ? 6:6) + 1))))))) << (0 ? 6:6)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:8) - (0 ?
 8:8) + 1))))))) << (0 ?
 8:8))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:8) - (0 ? 8:8) + 1))))))) << (0 ? 8:8)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)))
                                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)))));

        /* Scale the core clock. */
        clock = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
              | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (FscaleValue) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2)))
              | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00000,
                                          clock));

        /* Done loading the frequency scaler. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00000,
                                          ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)))));

        /* A option to support shader clock scaling. */
        if (ShaderFscaleValue != ~0U && ShaderFscaleValue > 0 && ShaderFscaleValue < 64)
        {
            /* Scale the shader clock. */
            clock = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:1) - (0 ?
 7:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:1) - (0 ?
 7:1) + 1))))))) << (0 ?
 7:1))) | (((gctUINT32) ((gctUINT32) (ShaderFscaleValue) & ((gctUINT32) ((((1 ?
 7:1) - (0 ?
 7:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:1) - (0 ? 7:1) + 1))))))) << (0 ? 7:1)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));


            gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                              Hardware->core,
                                              0x0010C,
                                              clock));

            /* Done loading the frequency scaler. */
            gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                              Hardware->core,
                                              0x0010C,
                                              ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));
        }

        /* Restore all clock gating. */
        gcmkONERROR(
            gckOS_WriteRegisterEx(Hardware->os,
                                  Hardware->core,
                                  Hardware->powerBaseAddress
                                  + 0x00104,
                                  data));
    }

    gcmkVERIFY(gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY(gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_GetFscaleValue(
    IN gckHARDWARE Hardware,
    IN gctUINT * FscaleValue,
    IN gctUINT * MinFscaleValue,
    IN gctUINT * MaxFscaleValue
    )
{
    *FscaleValue = Hardware->powerOnFscaleVal;
    *MinFscaleValue = Hardware->minFscaleValue;
    *MaxFscaleValue = 64;

    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_SetMinFscaleValue(
    IN gckHARDWARE Hardware,
    IN gctUINT MinFscaleValue
    )
{
    if (MinFscaleValue >= 1 && MinFscaleValue <= 64)
    {
        Hardware->minFscaleValue = MinFscaleValue;
    }

    return gcvSTATUS_OK;
}
#endif

gceSTATUS
gckHARDWARE_QueryIdle(
    IN gckHARDWARE Hardware,
    OUT gctBOOL_PTR IsIdle
    )
{
    gceSTATUS status;
    gctUINT32 idle;
    gctUINT32 address;
    gctBOOL isIdle = gcvFALSE;

#if gcdINTERRUPT_STATISTIC
    gckEVENT eventObj = Hardware->kernel->eventObj;
    gctINT32 pendingInterrupt;
#endif

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

#if gcdCAPTURE_ONLY_MODE
    *IsIdle = gcvTRUE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
#endif

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(IsIdle != gcvNULL);

    do
    {
        /* We are idle when the power is not ON. */
        if (Hardware->chipPowerState != gcvPOWER_ON)
        {
            isIdle = gcvTRUE;
            break;
        }

        if (Hardware->mcFE)
        {
            gctBOOL isIdle;

            gcmkONERROR(gckMCFE_HardwareIdle(Hardware, &isIdle));

            if(!isIdle)
            {
                break;
            }
        }
        else
        {
            /* Read idle register. */
            gcmkONERROR(
                gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00004, &idle));

            /* Pipe must be idle. */
            if ((idle | (1 << 14)) != 0x7ffffffe)
            {
                /* Something is busy. */
                break;
            }

            /* Read the current FE address. */
            gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                             Hardware->core,
                                             0x00664,
                                             &address));

            gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                             Hardware->core,
                                             0x00664,
                                             &address));

            /* Test if address is inside the last WAIT/LINK sequence. */
            if ((address < Hardware->lastWaitLink) ||
                (address > (gctUINT64)Hardware->lastWaitLink + 16))
            {
                /* FE is not in WAIT/LINK yet. */
                break;
            }
        } /* end of else */

#if gcdINTERRUPT_STATISTIC
        gcmkONERROR(gckOS_AtomGet(
            Hardware->os,
            eventObj->interruptCount,
            &pendingInterrupt
            ));

        if (pendingInterrupt)
        {
            /* Pending interrupts, not idle. */
            break;
        }

        if (Hardware->asyncFE)
        {
            gckEVENT asyncEvent = Hardware->kernel->asyncEvent;

            gcmkONERROR(gckOS_AtomGet(
                Hardware->os,
                asyncEvent->interruptCount,
                &pendingInterrupt
                ));

            if (pendingInterrupt)
            {
                /* Pending async FE interrupts, not idle. */
                break;
            }
        }
#endif

        /* Is really idle. */
        isIdle = gcvTRUE;
    }
    while (gcvFALSE);

    *IsIdle = isIdle;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
** Handy macros that will help in reading those debug registers.
*/
#define gcmkREAD_DEBUG_REGISTER_PART1(control, block, index, data) \
    gcmkONERROR(\
        gckOS_WriteRegisterEx(Hardware->os, \
                              Hardware->core, \
                              GC_DEBUG_CONTROL##control##_Address, \
                              gcmSETFIELD(0, \
                                          GC_DEBUG_CONTROL##control, \
                                          block, \
                                          index))); \
    gcmkONERROR(\
        gckOS_ReadRegisterEx(Hardware->os, \
                             Hardware->core, \
                             GC_DEBUG_SIGNALS_##block##_Address, \
                             &profiler_part1->data))

#define gcmkREAD_DEBUG_REGISTER_PART2(control, block, index, data) \
    gcmkONERROR(\
        gckOS_WriteRegisterEx(Hardware->os, \
                              Hardware->core, \
                              GC_DEBUG_CONTROL##control##_Address, \
                              gcmSETFIELD(0, \
                                          GC_DEBUG_CONTROL##control, \
                                          block, \
                                          index))); \
    gcmkONERROR(\
        gckOS_ReadRegisterEx(Hardware->os, \
                             Hardware->core, \
                             GC_DEBUG_SIGNALS_##block##_Address, \
                             &profiler_part2->data))

#define gcmkREAD_DEBUG_REGISTER_N(control, block, index, data) \
    gcmkONERROR(\
        gckOS_WriteRegisterEx(Hardware->os, \
                              Hardware->core, \
                              GC_DEBUG_CONTROL##control##_Address, \
                              gcmSETFIELD(0, \
                                          GC_DEBUG_CONTROL##control, \
                                          block, \
                                          index))); \
    gcmkONERROR(\
        gckOS_ReadRegisterEx(Hardware->os, \
                             Hardware->core, \
                             GC_DEBUG_SIGNALS_##block##_Address, \
                             &data))

#define gcmkRESET_DEBUG_REGISTER(control, block, value) \
    gcmkONERROR(\
        gckOS_WriteRegisterEx(Hardware->os, \
                              Hardware->core, \
                              GC_DEBUG_CONTROL##control##_Address, \
                              gcmSETFIELD(0, \
                                          GC_DEBUG_CONTROL##control, \
                                          block, \
                                          value))); \
    gcmkONERROR(\
        gckOS_WriteRegisterEx(Hardware->os, \
                              Hardware->core, \
                              GC_DEBUG_CONTROL##control##_Address, \
                              gcmSETFIELD(0, \
                                          GC_DEBUG_CONTROL##control, \
                                          block, \
                                          0)))

static gctUINT32
CalcDelta(
    IN gctUINT32 newval,
    IN gctUINT32 oldval
    )
{
    if (newval >= oldval)
    {
        return newval - oldval;
    }
    else
    {
        return (gctUINT32)((gctUINT64)newval + 0x100000000ll - oldval);
    }
}


#if USE_SW_RESET
#define gcmkRESET_PROFILE_DATA_PART1(counterName) \
    temp = profiler_part1->counterName; \
    profiler_part1->counterName = CalcDelta(temp, Hardware->kernel->profiler.preProfiler_part1.counterName); \
    Hardware->kernel->profiler.preProfiler_part1.counterName = temp
#endif

#define gcmkUPDATE_PROFILE_DATA_PART1(data) \
    profilerHistroy_part1->data += profiler_part1->data
#define gcmkUPDATE_PROFILE_DATA_PART2(data) \
    profilerHistroy_part2->data += profiler_part2->data

gceSTATUS
gckHARDWARE_QueryContextProfile(
    IN gckHARDWARE Hardware,
    IN gctBOOL Reset,
    OUT gcsPROFILER_COUNTERS_PART1 * Counters_part1,
    OUT gcsPROFILER_COUNTERS_PART2 * Counters_part2
)
{
    gceSTATUS status;
    gckCOMMAND command = Hardware->kernel->command;
    gcsPROFILER_COUNTERS_PART1 * profiler_part1 = Counters_part1;
    gcsPROFILER_COUNTERS_PART2 * profiler_part2 = Counters_part2;

    gcmkHEADER_ARG("Hardware=0x%x Counters_part1=0x%x, Counters_part2=0x%x", Hardware, Counters_part1, Counters_part2);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Acquire the context sequnence mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        command->os, command->mutexContextSeq, gcvINFINITE
        ));

    /* Read the counters. */
    if (Counters_part1)
    {
        gcmkVERIFY_OK(gckOS_MemCopy(
            profiler_part1, &Hardware->kernel->profiler.histroyProfiler_part1, gcmSIZEOF(gcsPROFILER_COUNTERS_PART1)
            ));
    }
    else if (Counters_part2)
    {
        gcmkVERIFY_OK(gckOS_MemCopy(
            profiler_part2, &Hardware->kernel->profiler.histroyProfiler_part2, gcmSIZEOF(gcsPROFILER_COUNTERS_PART2)
            ));
    }

    /* Reset counters. */
    if (Reset)
    {
        if (Counters_part1)
        {
            gcmkVERIFY_OK(gckOS_ZeroMemory(
                &Hardware->kernel->profiler.histroyProfiler_part1, gcmSIZEOF(gcsPROFILER_COUNTERS_PART1)
                ));
        }
        else if (Counters_part2)
        {
            gcmkVERIFY_OK(gckOS_ZeroMemory(
                &Hardware->kernel->profiler.histroyProfiler_part2, gcmSIZEOF(gcsPROFILER_COUNTERS_PART2)
                ));
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(
        command->os, command->mutexContextSeq
        ));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_UpdateContextProfile(
    IN gckHARDWARE Hardware
)
{
    gceSTATUS status;
    gcsPROFILER_COUNTERS_PART1 * profiler_part1 = &Hardware->kernel->profiler.latestProfiler_part1;
    gcsPROFILER_COUNTERS_PART1 * profilerHistroy_part1 = &Hardware->kernel->profiler.histroyProfiler_part1;
    gcsPROFILER_COUNTERS_PART2 * profiler_part2 = &Hardware->kernel->profiler.latestProfiler_part2;
    gcsPROFILER_COUNTERS_PART2 * profilerHistroy_part2 = &Hardware->kernel->profiler.histroyProfiler_part2;
    gceCHIPMODEL chipModel;
    gctUINT32 chipRevision;
    gctUINT32 i;
    gctUINT32 resetValue = 0xF;
    gctBOOL newCounters0 = gcvFALSE;
    gctUINT32 clock;
    gctUINT32 colorKilled = 0, colorDrawn = 0, depthKilled = 0, depthDrawn = 0;
    gctUINT32 totalRead, totalWrite;
    gctUINT32 mc_axi_max_min_latency;
    gctUINT32 temp;
    gckCOMMAND command = Hardware->kernel->command;
    gctBOOL mutexAcquired = gcvFALSE;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    /* Acquire the context sequnence mutex. */
    gcmkONERROR(gckOS_AcquireMutex(
        command->os, command->mutexContextSeq, gcvINFINITE
        ));
    mutexAcquired = gcvTRUE;

    chipModel = Hardware->identity.chipModel;
    chipRevision = Hardware->identity.chipRevision;
    if ((chipModel == gcv5000 && chipRevision == 0x5434) || (chipModel == gcv3000 && chipRevision == 0x5435))
    {
        resetValue = 0xFF;
        newCounters0 = gcvTRUE;
    }
    if (chipModel == gcv2100 || chipModel == gcv2000 || chipModel == gcv880)
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00078,
            &profiler_part2->hi_total_idle_cycle_count));

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00438,
            &profiler_part2->hi_total_cycle_count));
    }
    else
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x0007C,
            &profiler_part2->hi_total_idle_cycle_count));

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00078,
            &profiler_part2->hi_total_cycle_count));
    }
    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_cycle_count);
    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_idle_cycle_count);

    /* Read clock control register. */
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x00000,
                                     &clock));

    profiler_part2->hi_total_read_8B_count = 0;
    profiler_part2->hi_total_write_8B_count = 0;
    profiler_part2->hi_total_readOCB_16B_count = 0;
    profiler_part2->hi_total_writeOCB_16B_count = 0;
    profiler_part1->pe0_pixel_count_drawn_by_color_pipe = 0;
    profiler_part1->pe0_pixel_count_drawn_by_depth_pipe = 0;
    profiler_part1->pe0_pixel_count_killed_by_color_pipe = 0;
    profiler_part1->pe0_pixel_count_killed_by_depth_pipe = 0;
    profiler_part1->pe1_pixel_count_drawn_by_color_pipe = 0;
    profiler_part1->pe1_pixel_count_drawn_by_depth_pipe = 0;
    profiler_part1->pe1_pixel_count_killed_by_color_pipe = 0;
    profiler_part1->pe1_pixel_count_killed_by_depth_pipe = 0;

    /* Walk through all avaiable pixel pipes. */
    for (i = 0; i < Hardware->identity.pixelPipes; ++i)
    {
        /* Select proper pipe. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
            Hardware->core,
            0x00000,
            ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:20) - (0 ?
 23:20) + 1))))))) << (0 ?
 23:20))) | (((gctUINT32) ((gctUINT32) (i) & ((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:20) - (0 ? 23:20) + 1))))))) << (0 ? 23:20)))));

        /* BW */
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00040,
            &totalRead));
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00044,
            &totalWrite));

        profiler_part2->hi_total_read_8B_count += totalRead;
        profiler_part2->hi_total_write_8B_count += totalWrite;

        /* OCB-only BW */
        if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_OCB_COUNTER))
        {
            if (Hardware->identity.customerID == 0x7e ||
                Hardware->identity.customerID == 0x7d)
            {
                gcmkONERROR(
                    gckOS_ReadRegisterEx(Hardware->os,
                    Hardware->core,
                    0x17E00,
                    &totalRead));
                gcmkONERROR(
                    gckOS_ReadRegisterEx(Hardware->os,
                    Hardware->core,
                    0x17E10,
                    &totalWrite));
            }
            else
            {
                gcmkONERROR(
                    gckOS_ReadRegisterEx(Hardware->os,
                    Hardware->core,
                    0x005C0,
                    &totalRead));
                gcmkONERROR(
                    gckOS_ReadRegisterEx(Hardware->os,
                    Hardware->core,
                    0x005D0,
                    &totalWrite));

            }
        }
        else
        {
            totalRead = 0;
            totalWrite = 0;
        }


        profiler_part2->hi_total_readOCB_16B_count += totalRead;
        profiler_part2->hi_total_writeOCB_16B_count += totalWrite;

        /* PE */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))));gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00454, &colorKilled));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))));gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00454, &depthKilled));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))));gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00454, &colorDrawn));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (3) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))));gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00454, &depthDrawn));


        if (i == 0)
        {
            profiler_part1->pe0_pixel_count_killed_by_color_pipe = colorKilled;
            profiler_part1->pe0_pixel_count_killed_by_depth_pipe = depthKilled;
            profiler_part1->pe0_pixel_count_drawn_by_color_pipe = colorDrawn;
            profiler_part1->pe0_pixel_count_drawn_by_depth_pipe = depthDrawn;
        }
        else if (i == 1)
        {
            profiler_part1->pe1_pixel_count_killed_by_color_pipe = colorKilled;
            profiler_part1->pe1_pixel_count_killed_by_depth_pipe = depthKilled;
            profiler_part1->pe1_pixel_count_drawn_by_color_pipe = colorDrawn;
            profiler_part1->pe1_pixel_count_drawn_by_depth_pipe = depthDrawn;
        }
    }

    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_read_8B_count);
    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_write_8B_count);
    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_readOCB_16B_count);
    gcmkUPDATE_PROFILE_DATA_PART2(hi_total_writeOCB_16B_count);
#if USE_SW_RESET
    gcmkRESET_PROFILE_DATA_PART1(pe0_pixel_count_killed_by_color_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe0_pixel_count_killed_by_depth_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe0_pixel_count_drawn_by_color_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe0_pixel_count_drawn_by_depth_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe1_pixel_count_killed_by_color_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe1_pixel_count_killed_by_depth_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe1_pixel_count_drawn_by_color_pipe);
    gcmkRESET_PROFILE_DATA_PART1(pe1_pixel_count_drawn_by_depth_pipe);
#endif
    gcmkUPDATE_PROFILE_DATA_PART1(pe0_pixel_count_killed_by_color_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe0_pixel_count_killed_by_depth_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe0_pixel_count_drawn_by_color_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe0_pixel_count_drawn_by_depth_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe1_pixel_count_killed_by_color_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe1_pixel_count_killed_by_depth_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe1_pixel_count_drawn_by_color_pipe);
    gcmkUPDATE_PROFILE_DATA_PART1(pe1_pixel_count_drawn_by_depth_pipe);

    /* Reset clock control register. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
        Hardware->core,
        0x00000,
        clock));

    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0007C, 0));
    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00438, 0));
    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00078, 0));

#if !USE_SW_RESET
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))
));
#endif

    /* FE */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_draw_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (11) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_out_vertex_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_cache_miss_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_cache_lk_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (17) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_stall_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (18) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00450, &profiler_part1->fe_process_count));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)))
));

    gcmkUPDATE_PROFILE_DATA_PART1(fe_draw_count);
    gcmkUPDATE_PROFILE_DATA_PART1(fe_out_vertex_count);
    gcmkUPDATE_PROFILE_DATA_PART1(fe_cache_miss_count);
    gcmkUPDATE_PROFILE_DATA_PART1(fe_cache_lk_count);
    gcmkUPDATE_PROFILE_DATA_PART1(fe_process_count);

    /* SH */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (7) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_inst_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_rendered_pixel_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_inst_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_rendered_vertice_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (11) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_branch_inst_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_texld_inst_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_branch_inst_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (14) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_texld_inst_counter));
    if (newCounters0)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (19) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_non_idle_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (15) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (21) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->vs_process_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (20) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_non_idle_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (17) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (18) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (22) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->ps_process_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->shader_cycle_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (23) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->tx_non_idle_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (24) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->tx_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (25) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->tx_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (26) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &profiler_part1->tx_process_count));
    }
#if USE_SW_RESET
    gcmkRESET_PROFILE_DATA_PART1(ps_inst_counter);
    gcmkRESET_PROFILE_DATA_PART1(ps_rendered_pixel_counter);
    gcmkRESET_PROFILE_DATA_PART1(vs_inst_counter);
    gcmkRESET_PROFILE_DATA_PART1(vs_rendered_vertice_counter);
    gcmkRESET_PROFILE_DATA_PART1(vs_branch_inst_counter);
    gcmkRESET_PROFILE_DATA_PART1(vs_texld_inst_counter);
    gcmkRESET_PROFILE_DATA_PART1(ps_branch_inst_counter);
    gcmkRESET_PROFILE_DATA_PART1(ps_texld_inst_counter);
    if (newCounters0)
    {
        gcmkRESET_PROFILE_DATA_PART1(vs_non_idle_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(vs_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(vs_stall_count);
        gcmkRESET_PROFILE_DATA_PART1(vs_process_count);
        gcmkRESET_PROFILE_DATA_PART1(ps_non_idle_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(ps_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(ps_stall_count);
        gcmkRESET_PROFILE_DATA_PART1(ps_process_count);
        gcmkRESET_PROFILE_DATA_PART1(shader_cycle_count);
        gcmkRESET_PROFILE_DATA_PART1(tx_non_idle_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(tx_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(tx_stall_count);
        gcmkRESET_PROFILE_DATA_PART1(tx_process_count);
    }
#endif
    gcmkUPDATE_PROFILE_DATA_PART1(ps_inst_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ps_rendered_pixel_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(vs_inst_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(vs_rendered_vertice_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(vs_branch_inst_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(vs_texld_inst_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ps_branch_inst_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ps_texld_inst_counter);
    if (newCounters0)
    {
        gcmkUPDATE_PROFILE_DATA_PART1(vs_non_idle_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(vs_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(vs_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(vs_process_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ps_non_idle_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ps_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ps_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ps_process_count);
        gcmkUPDATE_PROFILE_DATA_PART1(shader_cycle_count);
        gcmkUPDATE_PROFILE_DATA_PART1(tx_non_idle_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(tx_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(tx_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(tx_process_count);
    }
#if !USE_SW_RESET
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24)))
));
#endif

    /* PA */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (3) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_input_vtx_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_input_prim_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (5) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_output_prim_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (6) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_depth_clipped_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (7) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_trivial_rejected_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_culled_prim_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_droped_prim_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_frustum_clipped_prim_counter));
    if (newCounters0)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_non_idle_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (14) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (15) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00460, &profiler_part1->pa_process_count));
    }
#if USE_SW_RESET
    gcmkRESET_PROFILE_DATA_PART1(pa_input_vtx_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_input_prim_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_output_prim_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_depth_clipped_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_trivial_rejected_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_culled_prim_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_droped_prim_counter);
    gcmkRESET_PROFILE_DATA_PART1(pa_frustum_clipped_prim_counter);
    if (newCounters0)
    {
        gcmkRESET_PROFILE_DATA_PART1(pa_non_idle_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(pa_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(pa_stall_count);
        gcmkRESET_PROFILE_DATA_PART1(pa_process_count);
    }
#endif
    gcmkUPDATE_PROFILE_DATA_PART1(pa_input_vtx_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_input_prim_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_output_prim_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_depth_clipped_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_trivial_rejected_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_culled_prim_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_droped_prim_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(pa_frustum_clipped_prim_counter);
    if (newCounters0)
    {
        gcmkUPDATE_PROFILE_DATA_PART1(pa_non_idle_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(pa_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(pa_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(pa_process_count);
    }
#if !USE_SW_RESET
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)))
));
#endif

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (15) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_clipped_triangle_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_clipped_line_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (17) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_culled_triangle_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (18) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_culled_lines_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (19) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_trivial_rejected_line_count));
    if (newCounters0)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_receive_triangle_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (11) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_send_triangle_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_receive_lines_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_send_lines_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (14) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_process_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (20) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00464, &profiler_part1->se_non_idle_starve_count));
    }
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8)))
));

    gcmkUPDATE_PROFILE_DATA_PART1(se_clipped_triangle_count);
    gcmkUPDATE_PROFILE_DATA_PART1(se_clipped_line_count);
    gcmkUPDATE_PROFILE_DATA_PART1(se_culled_triangle_count);
    gcmkUPDATE_PROFILE_DATA_PART1(se_culled_lines_count);
    gcmkUPDATE_PROFILE_DATA_PART1(se_trivial_rejected_line_count);
    if (newCounters0)
    {
        gcmkUPDATE_PROFILE_DATA_PART1(se_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_receive_triangle_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_send_triangle_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_receive_lines_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_send_lines_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_process_count);
        gcmkUPDATE_PROFILE_DATA_PART1(se_non_idle_starve_count);
    }

    /* RA */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_valid_pixel_count_to_render));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_total_quad_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_valid_quad_count_after_early_z));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (3) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_input_prim_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_pipe_cache_miss_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_prefetch_cache_miss_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (11) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_eez_culled_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (17) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_pipe_hz_cache_miss_counter));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (18) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_prefetch_hz_cache_miss_counter));
    if (newCounters0)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_non_idle_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (14) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_starve_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (15) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_stall_count));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00448, &profiler_part1->ra_process_count));
    }
#if USE_SW_RESET
    gcmkRESET_PROFILE_DATA_PART1(ra_valid_pixel_count_to_render);
    gcmkRESET_PROFILE_DATA_PART1(ra_total_quad_count);
    gcmkRESET_PROFILE_DATA_PART1(ra_valid_quad_count_after_early_z);
    gcmkRESET_PROFILE_DATA_PART1(ra_input_prim_count);
    gcmkRESET_PROFILE_DATA_PART1(ra_pipe_cache_miss_counter);
    gcmkRESET_PROFILE_DATA_PART1(ra_prefetch_cache_miss_counter);
    gcmkRESET_PROFILE_DATA_PART1(ra_eez_culled_counter);
    gcmkRESET_PROFILE_DATA_PART1(ra_pipe_hz_cache_miss_counter);
    gcmkRESET_PROFILE_DATA_PART1(ra_prefetch_hz_cache_miss_counter);
    if (newCounters0)
    {
        gcmkRESET_PROFILE_DATA_PART1(ra_non_idle_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(ra_starve_count);
        gcmkRESET_PROFILE_DATA_PART1(ra_stall_count);
        gcmkRESET_PROFILE_DATA_PART1(ra_process_count);
    }
#endif
    gcmkUPDATE_PROFILE_DATA_PART1(ra_valid_pixel_count_to_render);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_total_quad_count);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_valid_quad_count_after_early_z);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_input_prim_count);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_pipe_cache_miss_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_prefetch_cache_miss_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_eez_culled_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_pipe_hz_cache_miss_counter);
    gcmkUPDATE_PROFILE_DATA_PART1(ra_prefetch_hz_cache_miss_counter);
    if (newCounters0)
    {
        gcmkUPDATE_PROFILE_DATA_PART1(ra_non_idle_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ra_starve_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ra_stall_count);
        gcmkUPDATE_PROFILE_DATA_PART1(ra_process_count);
    }
#if !USE_SW_RESET
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))
));
#endif

    /* TX */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_total_bilinear_requests));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_total_trilinear_requests));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_total_discarded_texture_requests));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (3) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_total_texture_requests));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (5) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_mc0_miss_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (6) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_mc0_request_byte_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (7) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_mc1_miss_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0044C, &profiler_part1->tx_mc1_request_byte_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00474,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24)))
));

    gcmkUPDATE_PROFILE_DATA_PART1(tx_total_bilinear_requests);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_total_trilinear_requests);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_total_discarded_texture_requests);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_total_texture_requests);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_mc0_miss_count);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_mc0_request_byte_count);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_mc1_miss_count);
    gcmkUPDATE_PROFILE_DATA_PART1(tx_mc1_request_byte_count);

    /* MC */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_8B_from_colorpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_8B_sentout_from_colorpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (3) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_8B_from_colorpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_sentout_from_colorpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (5) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_from_colorpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (7) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_8B_from_depthpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_8B_sentout_from_depthpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_8B_from_depthpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (10) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_sentout_from_depthpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (11) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_from_depthpipe));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_8B_from_others));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_8B_from_others));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (14) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_read_req_from_others));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (15) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mcc_total_write_req_from_others));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (21) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_fe_read_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (22) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_mmu_read_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (23) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_blt_read_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (24) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_sh0_read_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (25) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_sh1_read_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (26) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_pe_write_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (27) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_blt_write_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (28) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_sh0_write_bandwidth));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (29) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00468, &profiler_part2->mc_sh1_write_bandwidth));

    /* Reset counters. */
    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0003C, 1));
    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0003C, 0));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:0) - (0 ?
 7:0) + 1))))))) << (0 ?
 7:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:0) - (0 ?
 7:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:0) - (0 ? 7:0) + 1))))))) << (0 ? 7:0)))
));

    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_8B_from_colorpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_8B_sentout_from_colorpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_8B_from_colorpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_sentout_from_colorpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_from_colorpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_8B_from_depthpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_8B_sentout_from_depthpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_8B_from_depthpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_sentout_from_depthpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_from_depthpipe);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_8B_from_others);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_8B_from_others);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_read_req_from_others);
    gcmkUPDATE_PROFILE_DATA_PART2(mcc_total_write_req_from_others);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_fe_read_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_mmu_read_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_blt_read_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_sh0_read_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_sh1_read_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_pe_write_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_blt_write_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_sh0_write_bandwidth);
    gcmkUPDATE_PROFILE_DATA_PART2(mc_sh1_write_bandwidth);

    /* read latency counters */
    if (newCounters0)
    {
        /* latency */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x0056C,
            &mc_axi_max_min_latency));

        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00570,
            &profiler_part2->mcc_axi_total_latency));

        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00574,
            &profiler_part2->mcc_axi_sample_count));

        /* Reset Latency counters */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
            Hardware->core,
            0x00568,
            0x10a));
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
            Hardware->core,
            0x00568,
            0xa));

        profiler_part2->mcc_axi_min_latency = (mc_axi_max_min_latency & 0xffff0000) >> 16;
        profiler_part2->mcc_axi_max_latency = (mc_axi_max_min_latency & 0x0000ffff);
        if (profiler_part2->mcc_axi_min_latency == 4095)
            profiler_part2->mcc_axi_min_latency = 0;

        gcmkUPDATE_PROFILE_DATA_PART2(mcc_axi_min_latency);
        gcmkUPDATE_PROFILE_DATA_PART2(mcc_axi_max_latency);
        gcmkUPDATE_PROFILE_DATA_PART2(mcc_axi_total_latency);
        gcmkUPDATE_PROFILE_DATA_PART2(mcc_axi_sample_count);
    }

    /* HI */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0046C, &profiler_part2->hi0_axi_cycles_read_request_stalled));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0046C, &profiler_part2->hi0_axi_cycles_write_request_stalled));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0046C, &profiler_part2->hi0_axi_cycles_write_data_stalled));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8)))
));

    gcmkUPDATE_PROFILE_DATA_PART2(hi0_axi_cycles_read_request_stalled);
    gcmkUPDATE_PROFILE_DATA_PART2(hi0_axi_cycles_write_request_stalled);
    gcmkUPDATE_PROFILE_DATA_PART2(hi0_axi_cycles_write_data_stalled);

    /* L2 */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_axi0_read_request_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_axi0_write_request_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (5) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_axi1_write_request_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (8) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_read_transactions_request_by_axi0));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (9) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_read_transactions_request_by_axi1));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (12) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_write_transactions_request_by_axi0));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (13) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_total_write_transactions_request_by_axi1));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (16) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi0_minmax_latency));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (17) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi0_total_latency));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (18) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi0_total_request_count));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (19) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi1_minmax_latency));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (20) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi1_total_latency));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (21) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00564, &profiler_part2->l2_axi1_total_request_count));

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (resetValue) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16))) ));
gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00478,   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))
));

    profiler_part2->l2_axi0_min_latency = (profiler_part2->l2_axi0_minmax_latency & 0xffff0000) >> 16;
    profiler_part2->l2_axi0_max_latency = (profiler_part2->l2_axi0_minmax_latency & 0x0000ffff);
    profiler_part2->l2_axi1_min_latency = (profiler_part2->l2_axi0_minmax_latency & 0xffff0000) >> 16;
    profiler_part2->l2_axi1_max_latency = (profiler_part2->l2_axi0_minmax_latency & 0x0000ffff);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_axi0_read_request_count);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_axi1_read_request_count);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_axi0_write_request_count);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_axi1_write_request_count);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_read_transactions_request_by_axi0);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_read_transactions_request_by_axi1);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_write_transactions_request_by_axi0);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_total_write_transactions_request_by_axi1);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi0_min_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi0_max_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi0_total_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi0_total_request_count);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi1_min_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi1_max_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi1_total_latency);
    gcmkUPDATE_PROFILE_DATA_PART2(l2_axi1_total_request_count);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(
        command->os, command->mutexContextSeq
        ));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (mutexAcquired)
    {
        gckOS_ReleaseMutex(
            command->os, command->mutexContextSeq
            );
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_InitProfiler(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 control;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x00000,
                                     &control));
    /* Enable debug register. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)))));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_ResetGPU(
    IN gckHARDWARE Hardware,
    IN gckOS Os,
    IN gceCORE Core
    )
{
    gctUINT32 control, idle;
    gceSTATUS status;
    gctUINT32 count = 0;
    gctUINT32 mmuEnabled;

    while (count < 2)
    {
        /* Disable clock gating. */
        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    Hardware->powerBaseAddress +
                    0x00104,
                    0x00000000));

        control = ((((gctUINT32) (0x015B0880)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)));

        /* Disable pulse-eater. */
        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    0x0010C,
                    control));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    0x0010C,
                    ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    0x0010C,
                    control));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    0x00000,
                    ((((gctUINT32) (0x00010900)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)))));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                    Core,
                    0x00000,
                    0x00010900));

        /* Wait for clock being stable. */
        gcmkONERROR(gckOS_Delay(Os, 1));

        /* Isolate the GPU. */
        control = ((((gctUINT32) (0x00010900)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                          Core,
                                          0x00000,
                                          control));

        if ((Hardware->core == gcvCORE_2D) &&
            _IsHardwareMatch(Hardware, 0x620, 0x5552))
        {
            gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                              Core,
                                              0x00800,
                                              ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)))));
        }

        if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_SECURITY_AHB) &&
            (Hardware->options.secureMode == gcvSECURE_IN_NORMAL))
        {
            gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                              Core,
                                              0x003A8,
                                              ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));
        }
        else
        {
            /* Set soft reset. */
            gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                              Core,
                                              0x00000,
                                              ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)))));
        }

        if (Hardware->hasQchannel)
        {
            /* Reset Qchannel. */
            gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                              Core,
                                              0x005E8,
                                              ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))));
        }

#if gcdFPGA_BUILD
        /* Wait more time on FPGA for reset as lower frequency */
        gcmkONERROR(gckOS_Delay(Os, 10));
#else
        /* Wait for reset. */
        gcmkONERROR(gckOS_Delay(Os, 1));
#endif
        /* Reset soft reset bit. */
        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                          Core,
                                          0x00000,
                                          ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:12) - (0 ?
 12:12) + 1))))))) << (0 ?
 12:12))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 12:12) - (0 ?
 12:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:12) - (0 ? 12:12) + 1))))))) << (0 ? 12:12)))));

        if (Hardware->hasQchannel)
        {
            Hardware->powerState = gcvFALSE;

            gcmkONERROR(gckHARDWARE_QchannelPowerControl(Hardware, gcvTRUE, gcvTRUE));


            /* Bypass Qchannel power management after reset. */
            if (!Hardware->options.powerManagement)
            {
                gcmkONERROR(gckHARDWARE_QchannelBypass(Hardware, gcvTRUE));
            }
        }

        /* Reset GPU isolation. */
        control = ((((gctUINT32) (control)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)));

        gcmkONERROR(gckOS_WriteRegisterEx(Os,
                                          Core,
                                          0x00000,
                                          control));

        /* Read idle register. */
        gcmkONERROR(gckOS_ReadRegisterEx(Os,
                                         Core,
                                         0x00004,
                                         &idle));


        if ((((((gctUINT32) (idle)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) ) == 0)
        {
            continue;
        }

        gcmkDUMP(Os, "@[register.wait 0x%05X 0x%08X 0x%08X]",
                 0x00004,
                 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))),
                 idle);

        /* Read reset register. */
        gcmkONERROR(gckOS_ReadRegisterEx(Os,
                                         Core,
                                         0x00000,
                                         &control));
        if (((((((gctUINT32) (control)) >> (0 ? 16:16)) & ((gctUINT32) ((((1 ? 16:16) - (0 ? 16:16) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1)))))) ) == 0)
        ||  ((((((gctUINT32) (control)) >> (0 ? 17:17)) & ((gctUINT32) ((((1 ? 17:17) - (0 ? 17:17) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1)))))) ) == 0)
        )
        {
            continue;
        }

        gcmkDUMP(Os, "@[register.wait 0x%05X 0x%08X 0x%08X]",
                 0x00000,
                 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)))
                 | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17))),
                 control);

        /* Force Disable MMU to guarantee setup command be read from physical addr */
        if (Hardware->options.secureMode == gcvSECURE_IN_NORMAL)
        {
            gctUINT32 regMmuCtrl = 0;
            gcmkONERROR(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                0x00388,
                &regMmuCtrl
                ));

            mmuEnabled = (((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) );
        }
        else
        {
            gctUINT32 regMmuCtrl = 0;

            gcmkONERROR(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                0x0018C,
                &regMmuCtrl
                ));

            mmuEnabled = (((((gctUINT32) (regMmuCtrl)) >> (0 ? 0:0)) & ((gctUINT32) ((((1 ? 0:0) - (0 ? 0:0) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1)))))) );
        }

        if (mmuEnabled)
        {
            /* Not reset properly, reset again. */
            continue;
        }

        count++;
    }

    /* Success. */
    return gcvSTATUS_OK;

OnError:

    /* Return the error. */
    return status;
}

gceSTATUS
gckHARDWARE_Reset(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctBOOL powerManagement = gcvFALSE;
    gctBOOL globalAcquired = gcvFALSE;
    gceCHIPPOWERSTATE statesStored, state;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_OBJECT(Hardware->kernel, gcvOBJ_KERNEL);

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

    /* Grab the global semaphore. */
    gcmkONERROR(gckOS_AcquireSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvTRUE;

    /* Record context ID in debug register before reset. */
    gcmkONERROR(gckHARDWARE_UpdateContextID(Hardware));

    /* Hardware reset. */
    status = gckOS_ResetGPU(Hardware->os, Hardware->core);

    if (gcmIS_ERROR(status))
    {
        if (Hardware->identity.chipRevision < 0x4600)
        {
            /* Not supported - we need the isolation bit. */
            gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        }

        /* Soft reset. */
        gcmkONERROR(_ResetGPU(Hardware, Hardware->os, Hardware->core));
    }

    /* Force the command queue to reload the next context. */
    Hardware->kernel->command->currContext = gcvNULL;

    /* Initialize hardware. */
    gcmkONERROR(gckHARDWARE_InitializeHardware(Hardware));

    /* Jump to address into which GPU should run if it doesn't stuck. */
    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_Execute(Hardware, Hardware->lastWaitLink, 16));
    }

    /* Release the global semaphore. */
    gcmkONERROR(gckOS_ReleaseSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvFALSE;

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

    gcmkPRINT("[galcore]: recovery done");

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkPRINT("[galcore]: Hardware not reset successfully, give up");

    if (globalAcquired)
    {
        /* Release the global semaphore. */
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(
            Hardware->os, Hardware->globalSemaphore
            ));
    }

    /* Return the error. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_GetBaseAddress(
    IN gckHARDWARE Hardware,
    OUT gctUINT32_PTR BaseAddress
    )
{
    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(BaseAddress != gcvNULL);

    /* Get the base address from the OS. */
    *BaseAddress = Hardware->baseAddress;

    /* Success. */
    gcmkFOOTER_ARG("*BaseAddress=0x%08x", *BaseAddress);
    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_NeedBaseAddress(
    IN gckHARDWARE Hardware,
    IN gctUINT32 State,
    OUT gctBOOL_PTR NeedBase
    )
{
    gctBOOL need = gcvFALSE;

    gcmkHEADER_ARG("Hardware=0x%x State=0x%08x", Hardware, State);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(NeedBase != gcvNULL);

    /* Make sure this is a load state. */
    if (((((gctUINT32) (State)) >> (0 ?
 31:27) & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1)))))) == (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))))
    {
        /* 2D addresses don't need a base address. */
    }

    /* Return the flag. */
    *NeedBase = need;

    /* Success. */
    gcmkFOOTER_ARG("*NeedBase=%d", *NeedBase);
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckHARDWARE_IsFeatureAvailable
**
**  Verifies whether the specified feature is available in hardware.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
**      gceFEATURE Feature
**          Feature to be verified.
*/
gceSTATUS
gckHARDWARE_IsFeatureAvailable(
    IN gckHARDWARE Hardware,
    IN gceFEATURE Feature
    )
{
    gctBOOL available;

    gcmkHEADER_ARG("Hardware=0x%x Feature=%d", Hardware, Feature);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    available = _QueryFeatureDatabase(Hardware, Feature);

    /* Return result. */
    gcmkFOOTER_ARG("%d", available ? gcvSTATUS_TRUE : gcvSTATUS_FALSE);
    return available ? gcvSTATUS_TRUE : gcvSTATUS_FALSE;
}

gceSTATUS
gckHARDWARE_QueryMcfe(
    IN gckHARDWARE Hardware,
    OUT const gceMCFE_CHANNEL_TYPE * Channels[],
    OUT gctUINT32 * Count
    )
{
    if (!_QueryFeatureDatabase(Hardware, gcvFEATURE_MCFE))
    {
        /* No MCFE feature. */
        return gcvSTATUS_NOT_SUPPORTED;
    }

    if (Channels)
    {
        *Channels = Hardware->mcfeChannels;
    }

    if (Count)
    {
        *Count = Hardware->mcfeChannelCount;
    }

    return gcvSTATUS_OK;
}


/*******************************************************************************
**
**  gckHARDWARE_DumpMMUException
**
**  Dump the MMU debug info on an MMU exception.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_DumpMMUException(
    IN gckHARDWARE Hardware
    )
{
    gctUINT32 mmu       = 0;
    gctUINT32 mmuStatus = 0;
    gctUINT32 address   = 0;
    gctUINT32 i         = 0;
    gctUINT32 mtlb      = 0;
    gctUINT32 stlb      = 0;
    gctUINT32 offset    = 0;
    gctUINT32 mmuStatusRegAddress;
    gctUINT32 mmuExceptionAddress;
    gceAREA_TYPE areaType = gcvAREA_TYPE_UNKNOWN;
    gctUINT32 stlbShift;
    gctUINT32 stlbMask;
    gctUINT32 pgoffMask;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

#if gcdENABLE_TRUST_APPLICATION
    if (Hardware->options.secureMode == gcvSECURE_IN_TA)
    {
        gcmkVERIFY_OK(gckKERNEL_SecurityDumpMMUException(Hardware->kernel));

        gckMMU_DumpRecentFreedAddress(Hardware->kernel->mmu);

        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }
    else
#endif
    if (Hardware->options.secureMode == gcvSECURE_NONE)
    {
        mmuStatusRegAddress = 0x00188;
        mmuExceptionAddress = 0x00190;
    }
    else
    {
        mmuStatusRegAddress = 0x00384;
        mmuExceptionAddress = 0x00380;
    }

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    gcmkPRINT("GPU[%d](ChipModel=0x%x ChipRevision=0x%x):\n",
              Hardware->core,
              Hardware->identity.chipModel,
              Hardware->identity.chipRevision);

    gcmkPRINT("**************************\n");
    gcmkPRINT("***   MMU STATUS DUMP   ***\n");
    gcmkPRINT("**************************\n");


    gcmkVERIFY_OK(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             mmuStatusRegAddress,
                             &mmuStatus));

    gcmkPRINT("  MMU status = 0x%08X\n", mmuStatus);

    for (i = 0; i < 4; i += 1)
    {
        mmu = mmuStatus & 0xF;
        mmuStatus >>= 4;

        if (mmu == 0)
        {
            continue;
        }

        switch (mmu)
        {
        case 1:
              gcmkPRINT("  MMU%d: slave not present\n", i);
              break;

        case 2:
              gcmkPRINT("  MMU%d: page not present\n", i);
              break;

        case 3:
              gcmkPRINT("  MMU%d: write violation\n", i);
              break;

        case 4:
              gcmkPRINT("  MMU%d: out of bound", i);
              break;

        case 5:
              gcmkPRINT("  MMU%d: read security violation", i);
              break;

        case 6:
              gcmkPRINT("  MMU%d: write security violation", i);
              break;

        default:
              gcmkPRINT("  MMU%d: unknown state\n", i);
        }

        if (Hardware->options.secureMode == gcvSECURE_NONE)
        {
            gcmkVERIFY_OK(
                gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     mmuExceptionAddress + i * 4,
                                     &address));
        }
        else
        {
            gcmkVERIFY_OK(
                gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     mmuExceptionAddress,
                                     &address));
        }

        gckMMU_GetAreaType(Hardware->kernel->mmu, address, &areaType);

        if (areaType == gcvAREA_TYPE_UNKNOWN)
        {
            gcmkPRINT("  MMU%d: exception address = 0x%08X, it is not mapped.\n", i, address);
            gcmkFOOTER_NO();
            return gcvSTATUS_OK;
        }

        pgoffMask = (areaType == gcvAREA_TYPE_4K) ? gcdMMU_OFFSET_4K_MASK : gcdMMU_OFFSET_1M_MASK;
        stlbShift = (areaType == gcvAREA_TYPE_4K) ? gcdMMU_STLB_4K_SHIFT : gcdMMU_STLB_1M_SHIFT;
        stlbMask  = (areaType == gcvAREA_TYPE_4K) ? gcdMMU_STLB_4K_MASK : gcdMMU_STLB_1M_MASK;

        mtlb   = (address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT;
        stlb   = (address & stlbMask) >> stlbShift;
        offset =  address & pgoffMask;

        gcmkPRINT("  MMU%d: exception address = 0x%08X\n", i, address);

        gcmkPRINT("    MTLB entry = %d\n", mtlb);

        gcmkPRINT("    STLB entry = %d\n", stlb);

        gcmkPRINT("    Offset = 0x%08X (%d)\n", offset, offset);

        gckMMU_DumpPageTableEntry(Hardware->kernel->mmu, areaType, address);

        gckMMU_DumpRecentFreedAddress(Hardware->kernel->mmu);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_HandleFault(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gctUINT32 mmu, mmuStatus, address = gcvINVALID_ADDRESS, i = 0;
    gctUINT32 mmuStatusRegAddress;
    gctUINT32 mmuExceptionAddress;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    if (Hardware->options.secureMode == gcvSECURE_NONE)
    {
        mmuStatusRegAddress = 0x00188;
        mmuExceptionAddress = 0x00190;
    }
    else
    {
        mmuStatusRegAddress = 0x00384;
        mmuExceptionAddress = 0x00380;
    }

    /* Get MMU exception address. */
#if gcdENABLE_TRUST_APPLICATION
    if (Hardware->options.secureMode == gcvSECURE_IN_TA)
    {
        gckKERNEL_ReadMMUException(Hardware->kernel, &mmuStatus, &address);
    }
    else
#endif
    {
        gcmkVERIFY_OK(gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            mmuStatusRegAddress,
            &mmuStatus
            ));

        gcmkPRINT("  MMU status = 0x%08X\n", mmuStatus);

        for (i = 0; i < 4; i += 1)
        {
            mmu = mmuStatus & 0xF;
            mmuStatus >>= 4;

            if (mmu == 0)
            {
                continue;
            }

            gcmkVERIFY_OK(gckOS_ReadRegisterEx(
                Hardware->os,
                Hardware->core,
                mmuExceptionAddress + i * 4,
                &address
                ));

            break;
        }
    }

    if (address != gcvINVALID_ADDRESS)
    {
        gckVIDMEM_NODE nodeObject = gcvNULL;
        gctUINT32 offset = 0;
        gctPHYS_ADDR_T physicalAddress = 0;
        gceAREA_TYPE areaType;
        gctUINT32 pageMask;
        gcePAGE_TYPE pageType;

        gckMMU_GetAreaType(Hardware->kernel->mmu, address, &areaType);

        pageMask = (areaType == gcvAREA_TYPE_4K) ? gcdMMU_PAGE_4K_MASK : gcdMMU_PAGE_1M_MASK;
        pageType = (areaType == gcvAREA_TYPE_4K) ? gcvPAGE_TYPE_4K : gcvPAGE_TYPE_1M;

#if gcdENABLE_TRUST_APPLICATION
        address &= ~gcdMMU_PAGE_4K_MASK;
#else
        address &= ~pageMask;
#endif

        /* Try to allocate memory and setup map for exception address. */
        gcmkONERROR(gckVIDMEM_NODE_Find(
            Hardware->kernel,
            address,
            &nodeObject,
            &offset
            ));

        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
            Hardware->kernel,
            nodeObject,
            offset,
            &physicalAddress
            ));

#if gcdENABLE_TRUST_APPLICATION
        if (Hardware->options.secureMode == gcvSECURE_IN_TA)
        {
            gckKERNEL_HandleMMUException(
                Hardware->kernel,
                mmuStatus,
                physicalAddress,
                address
                );
        }
        else
#endif
        {
            gctUINT32_PTR entry;

            /* Setup page table. */
            gcmkONERROR(gckMMU_GetPageEntry(Hardware->kernel->mmu, pageType, address, &entry));

            gckMMU_SetPage(Hardware->kernel->mmu, physicalAddress, pageType, gcvTRUE, entry);

            /* Resume hardware execution. */
            gcmkVERIFY_OK(gckOS_WriteRegisterEx(
                Hardware->os,
                Hardware->core,
                mmuExceptionAddress + i * 4,
                *entry
                ));
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_DumpMCFEState(
    IN gckOS Os,
    IN gceCORE Core
    )
{
    gctUINT32 i, j, data = 0, array[8] = { 0 };
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER();

    gcmkPRINT("**************************\n");
    gcmkPRINT("*****   MCFE STATE   *****\n");
    gcmkPRINT("**************************\n");

    /* Fetch address of channels. */
    gcmkPRINT("Channel fetch addresses:\n");
    gcmkPRINT("     [00]        [01]        [02]        [03]\n");

    for (i = 0; i < 4; i++)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x3 + 2 * i));
    }

    for (i = 0; i < 16; i++)
    {
        for (j = 0; j < 4; j++)
        {
            gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x2 + 2 * j));
            gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &array[j]));
        }

        gcmkPRINT("  0x%08X  0x%08X  0x%08X  0x%08X\n", array[0], array[1], array[2], array[3]);
    }

    /* Command data of channels. */
    gcmkPRINT_N(0, "Channel command data:\n");
    gcmkPRINT_N(0, "           [00]                    [01]                    [02]                    [03]\n");
    gcmkPRINT_N(0, "     [Low        High]       [Low        High]       [Low        High]       [Low        High]\n");


    for (i = 0; i < 4; i++)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x11 + 4 * i));
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x11 + 4 * i + 2));
    }

    for (i = 0; i < 32; i++)
    {
        for (j = 0; j < 4; j++)
        {
            gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x10 + 4 * j));
            gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &array[j * 2]));

            gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x10 + 4 * j + 2));
            gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &array[j * 2 + 1]));
        }

        gcmkPRINT_N(0, "  0x%08X  0x%08X  0x%08X  0x%08X  0x%08X  0x%08X  0x%08X  0x%08X\n",
                    array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7]);
    }

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x00));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "0x00: 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x01));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "0x01: 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0A));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "WaitSemaphore: 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0B));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "WaitEventID(channel 0 and 1): 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0C));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "WaitEventID(channel 2 and 3): 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0D));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "DecodeState: 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0E));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "DebugSelect(0x0E): 0x%08X\n", data);

    gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x0F));
    gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
    gcmkPRINT_N(0, "DebugSelect(0x0F): 0x%08X\n", data);

    for (i = 0; i < 9; i++)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Os, Core, 0x470, 0x20 + i));
        gcmkONERROR(gckOS_ReadRegisterEx(Os, Core, 0x450, &data));
        gcmkPRINT_N(0, "DebugSelect(0x%02X): 0x%08X\n", 0x20 + i, data);
    }

OnError:
    gcmkFOOTER();

    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_DumpGPUState
**
**  Dump the GPU debug registers.
**
**  INPUT:
**
**      gckHARDWARE Harwdare
**          Pointer to an gckHARDWARE object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckHARDWARE_DumpGPUState(
    IN gckHARDWARE Hardware
    )
{
    static gctCONST_STRING _cmdState[] =
    {
        "PAR_IDLE_ST", "PAR_DEC_ST", "PAR_ADR0_ST", "PAR_LOAD0_ST",
        "PAR_ADR1_ST", "PAR_LOAD1_ST", "PAR_3DADR_ST", "PAR_3DCMD_ST",
        "PAR_3DCNTL_ST", "PAR_3DIDXCNTL_ST", "PAR_INITREQDMA_ST", "PAR_DRAWIDX_ST",
        "PAR_DRAW_ST", "PAR_2DRECT0_ST", "PAR_2DRECT1_ST", "PAR_2DDATA0_ST",
        "PAR_2DDATA1_ST", "PAR_WAITFIFO_ST", "PAR_WAIT_ST", "PAR_LINK_ST",
        "PAR_END_ST", "PAR_STALL_ST", "INVALID_PAR_ST", "INVALID_PAR_ST",
        "INVALID_PAR_ST", "INVALID_PAR_ST", "INVALID_PAR_ST", "INVALID_PAR_ST",
        "INVALID_PAR_ST", "INVALID_PAR_ST", "INVALID_PAR_ST", "INVALID_PAR_ST"
    };

    static gctCONST_STRING _cmdDmaState[] =
    {
        "CMD_IDLE_ST", "CMD_START_ST", "CMD_REQ_ST", "CMD_END_ST"
    };

    static gctCONST_STRING _cmdFetState[] =
    {
        "FET_IDLE_ST", "FET_RAMVALID_ST", "FET_VALID_ST", "INVALID_FET_ST"
    };

    static gctCONST_STRING _reqDmaState[] =
    {
        "REQ_IDLE_ST", "REQ_WAITIDX_ST", "REQ_CAL_ST", "INVALID_REQ_ST"
    };

    static gctCONST_STRING _calState[] =
    {
        "CAL_IDLE_ST", "CAL_LDADR_ST", "CAL_IDXCALC_ST", "INVALID_CAL_ST"
    };

    static gctCONST_STRING _veReqState[] =
    {
        "VER_IDLE_ST", "VER_CKCACHE_ST", "VER_MISS_ST", "INVALID_VER_ST"
    };

    enum
    {
        RA_INDEX      = 0,
        TX_INDEX      = 1,
        FE_INDEX      = 2,
        PE_INDEX      = 3,
        DE_INDEX      = 4,
        SH_INDEX      = 5,
        PA_INDEX      = 6,
        SE_INDEX      = 7,
        MC_INDEX      = 8,
        HI_INDEX      = 9,
        TPG_INDEX     = 10,
        TFB_INDEX     = 11,
        USC_INDEX     = 12,
        L2_INDEX      = 13,
        BLT_INDEX     = 14,
        WD_INDEX      = 15,
        VTXDATA_INDEX = 16,
        DIR_INDEX     = 17,
        PPA_INDEX     = 18,
        NN_INDEX      = 19,
        QC_INDEX      = 20,
        MODULE_MAX_INDEX,
    };

    /* must keep order correctly for _dbgRegs, we need ajust some value base on the index */
    static gcsiDEBUG_REGISTERS _dbgRegs[MODULE_MAX_INDEX] =
    {
        { "RA", 0x474, 16, 0x448, 256, 0x1, 0x00, gcvTRUE, gcvTRUE  },
        { "TX", 0x474, 24, 0x44C, 128, 0x1, 0x00, gcvTRUE, gcvTRUE  },
        { "FE", 0x470, 0, 0x450, 256, 0x1, 0x00, gcvTRUE, gcvFALSE },
        { "PE", 0x470, 16, 0x454, 256, 0x3, 0x00, gcvTRUE, gcvTRUE  },
        { "DE", 0x470, 8, 0x458, 256, 0x1, 0x00, gcvTRUE, gcvFALSE },
        { "SH", 0x470, 24, 0x45C, 256, 0x1, 0x00, gcvTRUE, gcvTRUE  },
        { "PA", 0x474, 0, 0x460, 256, 0x1, 0x00, gcvTRUE, gcvTRUE  },
        { "SE", 0x474, 8, 0x464, 256, 0x1, 0x00, gcvTRUE, gcvTRUE  },
        { "MC", 0x478, 0, 0x468, 256, 0x3, 0x00, gcvTRUE, gcvTRUE  },
        { "HI", 0x478, 8, 0x46C, 256, 0x1, 0x00, gcvTRUE, gcvFALSE },
        { "TPG", 0x474, 24, 0x44C, 32, 0x2, 0x80, gcvFALSE, gcvTRUE  },
        { "TFB", 0x474, 24, 0x44C, 32, 0x2, 0xA0, gcvFALSE, gcvTRUE  },
        { "USC", 0x474, 24, 0x44C, 64, 0x2, 0xC0, gcvFALSE, gcvTRUE  },
        { "L2", 0x478, 0, 0x564, 256, 0x1, 0x00, gcvTRUE, gcvFALSE },
        { "BLT", 0x478, 24, 0x1A4, 256, 0x1, 0x00, gcvFALSE, gcvTRUE  },
        { "WD", 0xF0, 16, 0xF4, 256, 0x1, 0x00, gcvFALSE, gcvFALSE },
        { "VTXDATA", 0x474, 24, 0x44C, 64, 0x1, 0x40, gcvFALSE, gcvTRUE  },
        { "DIR", 0xF0, 24, 0xF8, 256, 0x1, 0x00, gcvFALSE, gcvTRUE  },
        { "PPA", 0x474, 0, 0x598, 256, 0x1, 0x00, gcvFALSE, gcvTRUE  },
        { "NN", 0x474, 24, 0x44C, 256, 0x2, 0x00, gcvFALSE, gcvTRUE  },
        { "QC", 0x5E8, 4, 0x59C, 256, 0x1, 0x00, gcvFALSE, gcvFALSE },

    };

    static gctUINT32 _otherRegs[] =
    {
        0x040, 0x044, 0x04C, 0x050, 0x054, 0x058, 0x05C, 0x060,
        0x43c, 0x440, 0x444, 0x414, 0x100
    };

    gceSTATUS status;
    gctUINT32 idle = 0, axi = 0, hiControl = 0;
    gctUINT32 dmaAddress1 = 0, dmaAddress2 = 0;
    gctUINT32 dmaState1 = 0, dmaState2 = 0;
    gctUINT32 dmaLow = 0, dmaHigh = 0;
    gctUINT32 cmdState = 0, cmdDmaState = 0, cmdFetState = 0;
    gctUINT32 dmaReqState = 0, calState = 0, veReqState = 0;
    gctUINT i;
    gctUINT pipe = 0, pipeMask = 0x1;
    static const gctUINT maxNumOfPipes = 4;
    gctUINT32 control = 0, oldControl = 0;
    gckOS os = Hardware->os;
    gceCORE core = Hardware->core;
    gceSTATUS hwTFB = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HW_TFB);
    gceSTATUS usc = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_USC);
    gceSTATUS multiCluster = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_MULTI_CLUSTER);
    gceSTATUS bltEngine = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_BLT_ENGINE);
    gceSTATUS gsShader = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_GEOMETRY_SHADER);
    gceSTATUS nnEngine = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_ENGINE);

    gcmkHEADER_ARG("Hardware=0x%X", Hardware);

    gcmkPRINT_N(12, "GPU[%d](ChipModel=0x%x ChipRevision=0x%x):\n",
                core,
                Hardware->identity.chipModel,
                Hardware->identity.chipRevision);

    /* Reset register values. */
    idle        = axi         =
    dmaState1   = dmaState2   =
    dmaAddress1 = dmaAddress2 =
    dmaLow      = dmaHigh     = 0;

    switch (Hardware->identity.pixelPipes)
    {
    case 2:
        pipeMask = 0x3;
        break;
    case 1:
        pipeMask = 0x1;
        break;
    default:
        gcmkASSERT(0);
    }

    if (!Hardware->mcFE)
    {
        /* Verify whether DMA is running. */
        gcmkONERROR(_VerifyDMA(
            os, core, &dmaAddress1, &dmaAddress2, &dmaState1, &dmaState2
            ));

        cmdState    =  dmaState2        & 0x1F;
        cmdDmaState = (dmaState2 >>  8) & 0x03;
        cmdFetState = (dmaState2 >> 10) & 0x03;
        dmaReqState = (dmaState2 >> 12) & 0x03;
        calState    = (dmaState2 >> 14) & 0x03;
        veReqState  = (dmaState2 >> 16) & 0x03;

        gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x00668, &dmaLow));
        gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x00668, &dmaLow));
        gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x0066C, &dmaHigh));
        gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x0066C, &dmaHigh));
    }

    gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x00004, &idle));
    gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x00000, &hiControl));
    gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x0000C, &axi));

    gcmkPRINT_N(0, "**************************\n");
    gcmkPRINT_N(0, "***   GPU STATE DUMP   ***\n");
    gcmkPRINT_N(0, "**************************\n");

    gcmkPRINT_N(4, "  axi      = 0x%08X\n", axi);

    gcmkPRINT_N(4, "  idle     = 0x%08X\n", idle);
    if ((idle & 0x00000001) == 0) gcmkPRINT_N(0, "    FE not idle\n");
    if ((idle & 0x00000002) == 0) gcmkPRINT_N(0, "    DE not idle\n");
    if ((idle & 0x00000004) == 0) gcmkPRINT_N(0, "    PE not idle\n");
    if ((idle & 0x00000008) == 0) gcmkPRINT_N(0, "    SH not idle\n");
    if ((idle & 0x00000010) == 0) gcmkPRINT_N(0, "    PA not idle\n");
    if ((idle & 0x00000020) == 0) gcmkPRINT_N(0, "    SE not idle\n");
    if ((idle & 0x00000040) == 0) gcmkPRINT_N(0, "    RA not idle\n");
    if ((idle & 0x00000080) == 0) gcmkPRINT_N(0, "    TX not idle\n");
    if ((idle & 0x00000100) == 0) gcmkPRINT_N(0, "    VG not idle\n");
    if ((idle & 0x00000200) == 0) gcmkPRINT_N(0, "    IM not idle\n");
    if ((idle & 0x00000400) == 0) gcmkPRINT_N(0, "    FP not idle\n");
    if ((idle & 0x00000800) == 0) gcmkPRINT_N(0, "    TS not idle\n");
    if ((idle & 0x00001000) == 0) gcmkPRINT_N(0, "    BL not idle\n");
    if ((idle & 0x00002000) == 0) gcmkPRINT_N(0, "    ASYNCFE not idle\n");
    if ((idle & 0x00004000) == 0) gcmkPRINT_N(0, "    MC not idle\n");
    if ((idle & 0x00008000) == 0) gcmkPRINT_N(0, "    PPA not idle\n");
    if ((idle & 0x00010000) == 0) gcmkPRINT_N(0, "    DC not idle\n");
    if ((idle & 0x00020000) == 0) gcmkPRINT_N(0, "    WD not idle\n");
    if ((idle & 0x00040000) == 0) gcmkPRINT_N(0, "    NN not idle\n");
    if ((idle & 0x00080000) == 0) gcmkPRINT_N(0, "    TP not idle\n");
    if ((idle & 0x80000000) != 0) gcmkPRINT_N(0, "    AXI low power mode\n");

    gcmkPRINT_N(4, "  AQ_HI_CLOCK_CONTROL  = 0x%08X\n", hiControl);

    if (!Hardware->mcFE)
    {
        if ((dmaAddress1 == dmaAddress2)
         && (dmaState1 == dmaState2))
        {
            gcmkPRINT_N(0, "  DMA appears to be stuck at this address:\n");
            gcmkPRINT_N(4, "    0x%08X\n", dmaAddress1);
        }
        else
        {
            if (dmaAddress1 == dmaAddress2)
            {
                gcmkPRINT_N(0, "  DMA address is constant, but state is changing:\n");
                gcmkPRINT_N(4, "    0x%08X\n", dmaState1);
                gcmkPRINT_N(4, "    0x%08X\n", dmaState2);
            }
            else
            {
                gcmkPRINT_N(0, "  DMA is running; known addresses are:\n");
                gcmkPRINT_N(4, "    0x%08X\n", dmaAddress1);
                gcmkPRINT_N(4, "    0x%08X\n", dmaAddress2);
            }
        }

        gcmkPRINT_N(4, "  dmaLow   = 0x%08X\n", dmaLow);
        gcmkPRINT_N(4, "  dmaHigh  = 0x%08X\n", dmaHigh);
        gcmkPRINT_N(4, "  dmaState = 0x%08X\n", dmaState2);
        gcmkPRINT_N(8, "    command state       = %d (%s)\n", cmdState, _cmdState   [cmdState]);
        gcmkPRINT_N(8, "    command DMA state   = %d (%s)\n", cmdDmaState, _cmdDmaState[cmdDmaState]);
        gcmkPRINT_N(8, "    command fetch state = %d (%s)\n", cmdFetState, _cmdFetState[cmdFetState]);
        gcmkPRINT_N(8, "    DMA request state   = %d (%s)\n", dmaReqState, _reqDmaState[dmaReqState]);
        gcmkPRINT_N(8, "    cal state           = %d (%s)\n", calState, _calState   [calState]);
        gcmkPRINT_N(8, "    VE request state    = %d (%s)\n", veReqState, _veReqState [veReqState]);
    }

    gcmkPRINT_N(0, "  Debug registers:\n");

    if (bltEngine)
    {
        _dbgRegs[BLT_INDEX].avail = gcvTRUE;
    }
    if (hwTFB)
    {
        _dbgRegs[TFB_INDEX].avail = gcvTRUE;
    }
    if (usc)
    {
        _dbgRegs[USC_INDEX].avail = gcvTRUE;
    }
    if (gsShader)
    {
        _dbgRegs[TPG_INDEX].avail = gcvTRUE;
    }

    if (Hardware->hasQchannel)
    {
        _dbgRegs[QC_INDEX].avail = gcvTRUE;
    }

    if (multiCluster)
    {
        _dbgRegs[WD_INDEX].avail = gcvTRUE;
        _dbgRegs[DIR_INDEX].avail = gcvTRUE;
        _dbgRegs[VTXDATA_INDEX].avail = gcvTRUE;
        _dbgRegs[PPA_INDEX].avail = gcvTRUE;
        _dbgRegs[FE_INDEX].index = 0xF0;
        _dbgRegs[HI_INDEX].index = 0xF0;
        /*spare 64 DWORDS debug values from TX for VTXDATA prefetch in USC */
        _dbgRegs[TX_INDEX].count = 64;

        for (i = 0; i < gcmCOUNTOF(_dbgRegs); i++)
        {
            if (_dbgRegs[i].inCluster)
            {
                _dbgRegs[i].pipeMask =
                    Hardware->identity.clusterAvailMask & Hardware->options.userClusterMask;
            }
        }
        pipeMask = Hardware->identity.clusterAvailMask & Hardware->options.userClusterMask;
    }
    if (nnEngine)
    {
        _dbgRegs[NN_INDEX].avail = gcvTRUE;
    }

    for (i = 0; i < gcmCOUNTOF(_dbgRegs); i += 1)
    {
        gcmkONERROR(_DumpDebugRegisters(os, core, &_dbgRegs[i]));
    }

    /* Record control. */
    gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x0, &oldControl));

    for (pipe = 0; pipe < maxNumOfPipes; pipe++)
    {
        if (((1 << pipe) & pipeMask) == 0)
            continue;

        gcmkPRINT_N(4, "    Other Registers[%d]:\n", pipe);

        /* Switch pipe. */
        gcmkONERROR(gckOS_ReadRegisterEx(os, core, 0x0, &control));
        control &= ~(0xF << 20);
        control |= (pipe << 20);
        gcmkONERROR(gckOS_WriteRegisterEx(os, core, 0x0, control));

        for (i = 0; i < gcmCOUNTOF(_otherRegs); i += 1)
        {
            gctUINT32 read;
            gcmkONERROR(gckOS_ReadRegisterEx(os, core, _otherRegs[i], &read));
            gcmkPRINT_N(12, "      [0x%04X] 0x%08X\n", _otherRegs[i], read);
        }

         if (Hardware->mmuVersion)
         {
             gcmkPRINT("    MMU status from MC[%d]:", pipe);
             gckHARDWARE_DumpMMUException(Hardware);
         }
    }

    /* MCFE state. */
    if (Hardware->mcFE)
    {
        gcmkVERIFY_OK(_DumpMCFEState(os, core));
    }

    /* Restore control. */
    gcmkONERROR(gckOS_WriteRegisterEx(os, core, 0x0, oldControl));


    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI0) &&
        !Hardware->mcFE)
    {
        /* FE debug register. */
        gcmkVERIFY_OK(_DumpLinkStack(os, core, &_dbgRegs[2]));
    }

    _DumpFEStack(os, core, &_dbgRegs[2]);

    gcmkPRINT_N(0, "**************************\n");
    gcmkPRINT_N(0, "*****   SW COUNTERS  *****\n");
    gcmkPRINT_N(0, "**************************\n");
    gcmkPRINT_N(4, "    Execute Count = 0x%08X\n", Hardware->executeCount);
    gcmkPRINT_N(4, "    Execute Addr  = 0x%08X\n", Hardware->lastExecuteAddress);
    gcmkPRINT_N(4, "    End     Addr  = 0x%08X\n", Hardware->lastEnd);

    /* dump stack. */
    gckOS_DumpCallStack(os);

OnError:

    /* Return the error. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckHARDWARE_ReadPerformanceRegister(
    IN gckHARDWARE Hardware,
    IN gctUINT PerformanceAddress,
    IN gctUINT IndexAddress,
    IN gctUINT IndexShift,
    IN gctUINT Index,
    OUT gctUINT32_PTR Value
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Hardware=0x%x PerformanceAddress=0x%x IndexAddress=0x%x "
                   "IndexShift=%u Index=%u",
                   Hardware, PerformanceAddress, IndexAddress, IndexShift,
                   Index);

    /* Write the index. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      IndexAddress,
                                      Index << IndexShift));

    /* Read the register. */
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     PerformanceAddress,
                                     Value));

    /* Test for reset. */
    if (Index == 15)
    {
        /* Index another register to get out of reset. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, IndexAddress, 0));
    }

    /* Success. */
    gcmkFOOTER_ARG("*Value=0x%x", *Value);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_GetFrameInfo(
    IN gckHARDWARE Hardware,
    OUT gcsHAL_FRAME_INFO * FrameInfo
    )
{
    gceSTATUS status;
    gctUINT i, clock;
    gcsHAL_FRAME_INFO info;
#if gcdFRAME_DB_RESET
    gctUINT reset;
#endif

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Get profile tick. */
    gcmkONERROR(gckOS_GetProfileTick(&info.ticks));

    /* Read SH counters and reset them. */
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        4,
        &info.shaderCycles));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        9,
        &info.vsInstructionCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        12,
        &info.vsTextureCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        7,
        &info.psInstructionCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        14,
        &info.psTextureCount));
#if gcdFRAME_DB_RESET
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0045C,
        0x00470,
        24,
        15,
        &reset));
#endif

    /* Read PA counters and reset them. */
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        3,
        &info.vertexCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        4,
        &info.primitiveCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        7,
        &info.rejectedPrimitives));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        8,
        &info.culledPrimitives));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        6,
        &info.clippedPrimitives));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        5,
        &info.outPrimitives));
#if gcdFRAME_DB_RESET
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00460,
        0x00474,
        0,
        15,
        &reset));
#endif

    /* Read RA counters and reset them. */
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        3,
        &info.inPrimitives));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        11,
        &info.culledQuadCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        1,
        &info.totalQuadCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        2,
        &info.quadCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        0,
        &info.totalPixelCount));
#if gcdFRAME_DB_RESET
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00448,
        0x00474,
        16,
        15,
        &reset));
#endif

    /* Read TX counters and reset them. */
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        0,
        &info.bilinearRequests));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        1,
        &info.trilinearRequests));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        8,
        &info.txHitCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        9,
        &info.txMissCount));
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        6,
        &info.txBytes8));
#if gcdFRAME_DB_RESET
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x0044C,
        0x00474,
        24,
        15,
        &reset));
#endif

    /* Read clock control register. */
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x00000,
                                     &clock));

    /* Walk through all avaiable pixel pipes. */
    for (i = 0; i < Hardware->identity.pixelPipes; ++i)
    {
        /* Select proper pipe. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00000,
                                          ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:20) - (0 ?
 23:20) + 1))))))) << (0 ?
 23:20))) | (((gctUINT32) ((gctUINT32) (i) & ((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:20) - (0 ? 23:20) + 1))))))) << (0 ? 23:20)))));

        /* Read cycle registers. */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x0007C,
                                         &info.idleCycles[i]));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00078,
                                         &info.cycles[i]));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00438,
                                         &info.mcCycles[i]));

        /* Read bandwidth registers. */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x0005C,
                                         &info.readRequests[i]));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00040,
                                         &info.readBytes8[i]));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00050,
                                         &info.writeRequests[i]));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00044,
                                         &info.writeBytes8[i]));

        /* Read PE counters. */
        gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
            Hardware,
            0x00454,
            0x00470,
            16,
            0,
            &info.colorKilled[i]));
        gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
            Hardware,
            0x00454,
            0x00470,
            16,
            2,
            &info.colorDrawn[i]));
        gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
            Hardware,
            0x00454,
            0x00470,
            16,
            1,
            &info.depthKilled[i]));
        gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
            Hardware,
            0x00454,
            0x00470,
            16,
            3,
            &info.depthDrawn[i]));
    }

    /* Zero out remaning reserved counters. */
    for (; i < 8; ++i)
    {
        info.readBytes8[i]    = 0;
        info.writeBytes8[i]   = 0;
        info.cycles[i]        = 0;
        info.idleCycles[i]    = 0;
        info.mcCycles[i]      = 0;
        info.readRequests[i]  = 0;
        info.writeRequests[i] = 0;
        info.colorKilled[i]   = 0;
        info.colorDrawn[i]    = 0;
        info.depthKilled[i]   = 0;
        info.depthDrawn[i]    = 0;
    }

    /* Reset clock control register. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      clock));

    /* Reset cycle and bandwidth counters. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0003C,
                                      1));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0003C,
                                      0));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00078,
                                      0));

#if gcdFRAME_DB_RESET
    /* Reset PE counters. */
    gcmkONERROR(gckHARDWARE_ReadPerformanceRegister(
        Hardware,
        0x00454,
        0x00470,
        16,
        15,
        &reset));
#endif

    /* Copy to user. */
    gcmkONERROR(gckOS_CopyToUserData(Hardware->os,
                                     &info,
                                     FrameInfo,
                                     gcmSIZEOF(info)));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_DumpGpuProfile(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT clock, i;
    gctUINT32 totalRead, totalWrite, read, write;

    gcmkHEADER_ARG("Hardware=0x%x", Hardware);

    /* Read clock control register. */
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x00000,
                                      &clock));

    totalRead = 0;
    totalWrite = 0;

    /* Walk through all avaiable pixel pipes. */
    for (i = 0; i < Hardware->identity.pixelPipes; ++i)
    {
        /* Select proper pipe. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00000,
                                          ((((gctUINT32) (clock)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:20) - (0 ?
 23:20) + 1))))))) << (0 ?
 23:20))) | (((gctUINT32) ((gctUINT32) (i) & ((gctUINT32) ((((1 ?
 23:20) - (0 ?
 23:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:20) - (0 ? 23:20) + 1))))))) << (0 ? 23:20)))));

        /* BW */
        gcmkONERROR(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             0x00040,
                             &read));
        totalRead += read;

        gcmkONERROR(
        gckOS_ReadRegisterEx(Hardware->os,
                             Hardware->core,
                             0x00044,
                             &write));
        totalWrite += write;
     }

     gcmkPRINT("==============GPU Profile: read request : %d\n", totalRead);
     gcmkPRINT("==============GPU Profile: write request: %d\n", totalWrite);

     /* Reset clock control register. */
     gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                       Hardware->core,
                                       0x00000,
                                       clock));
    /* Reset counters. */
    gcmkONERROR(
       gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0003C, 1));
    gcmkONERROR(
       gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0003C, 0));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

#if gcdDVFS
#define READ_FROM_EATER1 0

gceSTATUS
gckHARDWARE_QueryLoad(
    IN gckHARDWARE Hardware,
    OUT gctUINT32 * Load
    )
{
    gctUINT32 debug1;
    gceSTATUS status;
    gcmkHEADER_ARG("Hardware=0x%X", Hardware);

    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT(Load != gcvNULL);

    gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE);

    if (Hardware->chipPowerState == gcvPOWER_ON)
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00110,
                                         Load));
#if READ_FROM_EATER1
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00134,
                                         Load));
#endif

        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00114,
                                         &debug1));

        /* Patch result of 0x110 with result of 0x114. */
        if ((debug1 & 0xFF) == 1)
        {
            *Load &= ~0xFF;
            *Load |= 1;
        }

        if (((debug1 & 0xFF00) >> 8) == 1)
        {
            *Load &= ~(0xFF << 8);
            *Load |= 1 << 8;
        }

        if (((debug1 & 0xFF0000) >> 16) == 1)
        {
            *Load &= ~(0xFF << 16);
            *Load |= 1 << 16;
        }

        if (((debug1 & 0xFF000000) >> 24) == 1)
        {
            *Load &= ~(0xFF << 24);
            *Load |= 1 << 24;
        }
    }
    else
    {
        status = gcvSTATUS_INVALID_REQUEST;
    }

OnError:

    gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex);

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_SetDVFSPeroid(
    IN gckHARDWARE Hardware,
    OUT gctUINT32 Frequency
    )
{
    gceSTATUS status;
    gctUINT32 period;
    gctUINT32 eater;

#if READ_FROM_EATER1
    gctUINT32 period1;
    gctUINT32 eater1;
#endif

    gcmkHEADER_ARG("Hardware=0x%X Frequency=%d", Hardware, Frequency);

    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    period = 0;

    while((64 << period) < (gcdDVFS_ANAYLSE_WINDOW * Frequency * 1000) )
    {
        period++;
    }

#if READ_FROM_EATER1
    /*
    *  Peroid = F * 1000 * 1000 / (60 * 16 * 1024);
    */
    period1 = Frequency * 6250 / 6114;
#endif

    gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE);

    if (Hardware->chipPowerState == gcvPOWER_ON)
    {
        /* Get current configure. */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x0010C,
                                         &eater));

        /* Change peroid. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x0010C,
                                          ((((gctUINT32) (eater)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:8) - (0 ?
 15:8) + 1))))))) << (0 ?
 15:8))) | (((gctUINT32) ((gctUINT32) (period) & ((gctUINT32) ((((1 ?
 15:8) - (0 ?
 15:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:8) - (0 ? 15:8) + 1))))))) << (0 ? 15:8)))));

#if READ_FROM_EATER1
        /* Config eater1. */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                         Hardware->core,
                                         0x00130,
                                         &eater1));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                          Hardware->core,
                                          0x00130,
                                          ((((gctUINT32) (eater1)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (period1) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)))));
#endif
    }
    else
    {
        status = gcvSTATUS_INVALID_REQUEST;
    }

OnError:
    gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex);

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_InitDVFS(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 data;

    gcmkHEADER_ARG("Hardware=0x%X", Hardware);

    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os,
                                     Hardware->core,
                                     0x0010C,
                                     &data));

    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 18:18) - (0 ?
 18:18) + 1))))))) << (0 ?
 18:18))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 18:18) - (0 ?
 18:18) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 18:18) - (0 ? 18:18) + 1))))))) << (0 ? 18:18)));
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 19:19) - (0 ?
 19:19) + 1))))))) << (0 ?
 19:19))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 19:19) - (0 ?
 19:19) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 19:19) - (0 ? 19:19) + 1))))))) << (0 ? 19:19)));
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:20) - (0 ?
 20:20) + 1))))))) << (0 ?
 20:20))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 20:20) - (0 ?
 20:20) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 20:20) - (0 ? 20:20) + 1))))))) << (0 ? 20:20)));
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:23) - (0 ?
 23:23) + 1))))))) << (0 ?
 23:23))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 23:23) - (0 ?
 23:23) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:23) - (0 ? 23:23) + 1))))))) << (0 ? 23:23)));
    data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                   "DVFS Configure=0x%X",
                   data);

    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os,
                                      Hardware->core,
                                      0x0010C,
                                      data));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}
#endif

gceSTATUS
gckHARDWARE_ExecuteFunctions(
    IN gcsFUNCTION_EXECUTION_PTR Execution
    )
{
    gceSTATUS status;
    gctUINT32 idle;
    gctUINT32 i, timer = 0, delay = 10;
    gctUINT32 address;
    gckHARDWARE hardware = (gckHARDWARE)Execution->hardware;

#if gcdCAPTURE_ONLY_MODE
    return gcvSTATUS_OK;
#endif

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(hardware->os, "#[function: %s]", Execution->funcName);
#endif

    for (i  = 0; i < Execution->funcCmdCount; i++)
    {
        address = Execution->funcCmd[i].address;

#if gcdDUMP_IN_KERNEL
        gcmkDUMP_BUFFER(
            hardware->os,
            gcvDUMP_BUFFER_KERNEL_COMMAND,
            Execution->funcCmd[i].logical,
            Execution->funcCmd[i].address,
            Execution->funcCmd[i].bytes
            );
#endif

        /* Execute prepared command sequence. */
        if (hardware->mcFE)
        {
            gcmkONERROR(gckMCFE_Execute(hardware, gcvFALSE, Execution->funcCmd[i].channelId, address, Execution->funcCmd[i].bytes));
        }
        else
        {
            gcmkONERROR(gckWLFE_Execute(hardware, address, Execution->funcCmd[i].bytes));
        }

#if gcdLINK_QUEUE_SIZE
        {
            gcuQUEUEDATA data;

            gcmkVERIFY_OK(gckOS_GetProcessID(&data.linkData.pid));

            data.linkData.start    = address;
            data.linkData.end      = address + Execution->funcCmd[i].bytes;
            data.linkData.linkLow  = 0;
            data.linkData.linkHigh = 0;

            gckQUEUE_Enqueue(&hardware->linkQueue, &data);
        }
#endif

        /* Wait until GPU idle. */
        do
        {
            gckOS_Udelay(hardware->os, delay);

            gcmkONERROR(gckOS_ReadRegisterEx(
                hardware->os,
                hardware->core,
                0x00004,
                &idle));

            timer += delay;

#if gcdGPU_TIMEOUT
            if (timer >= hardware->kernel->timeOut)
            {
                gckHARDWARE_DumpGPUState(hardware);

                if (hardware->kernel->command)
                {
                    gckCOMMAND_DumpExecutingBuffer(hardware->kernel->command);
                }

                /* Even if hardware is not reset correctly, let software
                ** continue to avoid software stuck. Software will timeout again
                ** and try to recover GPU in next timeout.
                */
                gcmkONERROR(gcvSTATUS_DEVICE);
            }
#endif
        }
        while (!_IsHWIdle(idle, hardware));
        gcmkDUMP(Hardware->os, "@[register.wait 0x%05X 0x%08X 0x%08X]",
                 0x00004,
                 ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (~0U) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))),
                 idle);
    }
    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckHARDWARE_QueryStateTimer(
    IN gckHARDWARE Hardware,
    OUT gctUINT64_PTR On,
    OUT gctUINT64_PTR Off,
    OUT gctUINT64_PTR Idle,
    OUT gctUINT64_PTR Suspend
    )
{
    gckOS_AcquireMutex(Hardware->os, Hardware->powerMutex, gcvINFINITE);

    gckSTATETIMER_Query(
        &Hardware->powerStateCounter,
        Hardware->chipPowerState,
        On, Off, Idle, Suspend);

    gckOS_ReleaseMutex(Hardware->os, Hardware->powerMutex);

    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_WaitFence(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT64 FenceData,
    IN gctUINT32 FenceAddress,
    OUT gctUINT32 *Bytes
    )
{
    gctUINT32_PTR logical = (gctUINT32_PTR)Logical;

    gctUINT32 dataLow = (gctUINT32)FenceData;
    gctUINT32 dataHigh = (gctUINT32)(FenceData >> 32);

    if (logical)
    {
        *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x01FD) & ((gctUINT32) ((((1 ?
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

        *logical++
            = dataHigh;

        *logical++
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x01FA) & ((gctUINT32) ((((1 ?
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

        *logical++
            = dataLow;

        *logical++
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x0F & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (Hardware->waitCount) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:16) - (0 ?
 17:16) + 1))))))) << (0 ?
 17:16))) | (((gctUINT32) (0x2 & ((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:16) - (0 ? 17:16) + 1))))))) << (0 ? 17:16)));

        *logical++
            = FenceAddress;
    }
    else
    {
        *Bytes = 6 * gcmSIZEOF(gctUINT32);
    }

    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_UpdateContextID(
    IN gckHARDWARE Hardware
    )
{
    static gcsiDEBUG_REGISTERS fe = { "FE", 0x470, 0, 0x450, 256, 0x1, 0x00, gcvTRUE, gcvFALSE};
    gckOS os = Hardware->os;
    gceCORE core = Hardware->core;
    gctUINT32 contextIDLow, contextIDHigh;
    gceSTATUS status;

    gcmkONERROR(gckOS_WriteRegisterEx(os, core, fe.index, 0x53 << fe.shift));
    gcmkONERROR(gckOS_ReadRegisterEx(os, core, fe.data, &contextIDLow));

    gcmkONERROR(gckOS_WriteRegisterEx(os, core, fe.index, 0x54 << fe.shift));
    gcmkONERROR(gckOS_ReadRegisterEx(os, core, fe.data, &contextIDHigh));

    Hardware->contextID = ((gctUINT64)contextIDHigh << 32) + contextIDLow;

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckHARDWARE_DummyDraw(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT32 Address,
    IN gceDUMMY_DRAW_TYPE DummyDrawType,
    IN OUT gctUINT32 * Bytes
    )
{
    gctUINT32 dummyDraw_gc400[] = {

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0193) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x000000,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0194) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0180) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:0) - (0 ?
 3:0) + 1))))))) << (0 ?
 3:0))) | (((gctUINT32) (0x8 & ((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:0) - (0 ? 3:0) + 1))))))) << (0 ? 3:0)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:12) - (0 ?
 13:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:12) - (0 ?
 13:12) + 1))))))) << (0 ?
 13:12))) | (((gctUINT32) ((gctUINT32) (4) & ((gctUINT32) ((((1 ?
 13:12) - (0 ?
 13:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:12) - (0 ? 13:12) + 1))))))) << (0 ? 13:12)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:16) - (0 ?
 23:16) + 1))))))) << (0 ?
 23:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:16) - (0 ?
 23:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:16) - (0 ? 23:16) + 1))))))) << (0 ? 23:16)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:24) - (0 ?
 31:24) + 1))))))) << (0 ?
 31:24))) | (((gctUINT32) ((gctUINT32) (4 * gcmSIZEOF(float)) & ((gctUINT32) ((((1 ?
 31:24) - (0 ?
 31:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:24) - (0 ? 31:24) + 1))))))) << (0 ? 31:24)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:7) - (0 ?
 7:7) + 1))))))) << (0 ?
 7:7))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 7:7) - (0 ?
 7:7) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:7) - (0 ? 7:7) + 1))))))) << (0 ? 7:7))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E05) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0202) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0208) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0))),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0204) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x1000) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x0, 0x0, 0x0, 0x0,
        0xDEADDEAD,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0203) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 6:0) - (0 ?
 6:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 6:0) - (0 ?
 6:0) + 1))))))) << (0 ?
 6:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 6:0) - (0 ?
 6:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 6:0) - (0 ? 6:0) + 1))))))) << (0 ? 6:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x020E) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0200) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        1,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x020C) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x000F003F,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x028C) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:8) - (0 ?
 11:8) + 1))))))) << (0 ?
 11:8))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 11:8) - (0 ?
 11:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:8) - (0 ? 11:8) + 1))))))) << (0 ? 11:8))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0500) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x028D) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:12) - (0 ?
 13:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:12) - (0 ?
 13:12) + 1))))))) << (0 ?
 13:12))) | (((gctUINT32) (0x2 & ((gctUINT32) ((((1 ?
 13:12) - (0 ?
 13:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:12) - (0 ? 13:12) + 1))))))) << (0 ? 13:12)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:8) - (0 ?
 9:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:8) - (0 ?
 9:8) + 1))))))) << (0 ?
 9:8))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 9:8) - (0 ?
 9:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:8) - (0 ? 9:8) + 1))))))) << (0 ? 9:8)))
        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:16) - (0 ?
 17:16) + 1))))))) << (0 ?
 17:16))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:16) - (0 ? 17:16) + 1))))))) << (0 ? 17:16))),


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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0300) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0301) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0302) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0303) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0289) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:0) - (0 ?
 3:0) + 1))))))) << (0 ?
 3:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:0) - (0 ? 3:0) + 1))))))) << (0 ? 3:0))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x05 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:0) - (0 ?
 3:0) + 1))))))) << (0 ?
 3:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:0) - (0 ?
 3:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:0) - (0 ? 3:0) + 1))))))) << (0 ? 3:0))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:0) - (0 ?
 23:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:0) - (0 ?
 23:0) + 1))))))) << (0 ?
 23:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 23:0) - (0 ?
 23:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:0) - (0 ? 23:0) + 1))))))) << (0 ? 23:0))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 23:0) - (0 ?
 23:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 23:0) - (0 ?
 23:0) + 1))))))) << (0 ?
 23:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 23:0) - (0 ?
 23:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 23:0) - (0 ? 23:0) + 1))))))) << (0 ? 23:0))),
        };

    gctUINT32 dummyDraw_v60[] = {

        /* Semaphore from FE to PE. */
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8))),

        /* Stall from FE to PE. */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8))),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E06) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) ((gctUINT32) (0x0) & ((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:0) - (0 ? 1:0) + 1))))))) << (0 ? 1:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 14:12) - (0 ?
 14:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 14:12) - (0 ?
 14:12) + 1))))))) << (0 ?
 14:12))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 14:12) - (0 ?
 14:12) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 14:12) - (0 ? 14:12) + 1))))))) << (0 ? 14:12)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:16) - (0 ?
 17:16) + 1))))))) << (0 ?
 17:16))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 17:16) - (0 ?
 17:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:16) - (0 ? 17:16) + 1))))))) << (0 ? 17:16)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:4) - (0 ?
 7:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:4) - (0 ?
 7:4) + 1))))))) << (0 ?
 7:4))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 7:4) - (0 ?
 7:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:4) - (0 ? 7:4) + 1))))))) << (0 ? 7:4))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0401) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x0,
        0x2,
        0x0,
        0x0,
        0x0,
        0x0,
        (gctUINT32)~0x0,


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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x020C) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0xffffffff,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        2,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E08) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        2,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0))),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        1,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        3,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E21) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) (0x2 & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:0) - (0 ? 2:0) + 1))))))) << (0 ? 2:0))),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0,

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x2000) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1 << 2) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x07801033,0x3fc00900,0x00000040,0x00390008,
        (gctUINT32)~0,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        0x0,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:0) - (0 ?
 1:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:0) - (0 ?
 1:0) + 1))))))) << (0 ?
 1:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
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
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:8) - (0 ?
 8:8) + 1))))))) << (0 ?
 8:8))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 8:8) - (0 ?
 8:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:8) - (0 ? 8:8) + 1))))))) << (0 ? 8:8)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:24) - (0 ?
 26:24) + 1))))))) << (0 ?
 26:24))) | (((gctUINT32) (0x3 & ((gctUINT32) ((((1 ?
 26:24) - (0 ?
 26:24) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:24) - (0 ? 26:24) + 1))))))) << (0 ? 26:24))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0241) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (31) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16))),

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
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0244) & ((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:0) - (0 ?
 9:0) + 1))))))) << (0 ?
 9:0))) | (((gctUINT32) ((gctUINT32) (31) & ((gctUINT32) ((((1 ?
 9:0) - (0 ?
 9:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:0) - (0 ? 9:0) + 1))))))) << (0 ? 9:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16))),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),

        (32+(4*(((gcsFEATURE_DATABASE *)Hardware->featureDatabase)->NumShaderCores)-1))/(4*(((gcsFEATURE_DATABASE *)Hardware->featureDatabase)->NumShaderCores)),

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        1,

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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5))),

        /* Semaphore from FE to PE. */
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
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8))),

        /* Stall from FE to PE. */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),

        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8))),

        /* Invalidate I cache.*/
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
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
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
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4))),

        /* SubmitJob. */
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),
        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27))),
    };

    gctUINT32 bytes = 0;
    gctUINT32_PTR dummyDraw = gcvNULL;


    switch(DummyDrawType)
    {
    case gcvDUMMY_DRAW_GC400:
        dummyDraw = dummyDraw_gc400;
        bytes = gcmSIZEOF(dummyDraw_gc400);
        *(dummyDraw + 1) = Address;
        break;
    case gcvDUMMY_DRAW_V60:
        dummyDraw = dummyDraw_v60;
        bytes = gcmSIZEOF(dummyDraw_v60);
        if (_QueryFeatureDatabase(Hardware, gcvFEATURE_MCFE))
        {
            gctUINT32 submitJob;

            submitJob = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x16 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                      | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) (0x001 & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

            if (bytes & 8)
            {
                /* To keep 16 byte alignment. */
                bytes -= 8;
            }

            dummyDraw[(bytes >> 2) - 2] = submitJob;
        }
        break;
    default:
        /* other chip no need dummy draw.*/
        gcmkASSERT(0);
        break;
    };

    if (Logical != gcvNULL)
    {
        gckOS_MemCopy(Logical, dummyDraw, bytes);
    }

    *Bytes = bytes;

    return gcvSTATUS_OK;
}

gceSTATUS
gckHARDWARE_EnterQueryClock(
    IN gckHARDWARE Hardware,
    OUT gctUINT64 *McStart,
    OUT gctUINT64 *ShStart
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT64 mcStart, shStart;

    gcmkONERROR(gckOS_GetTime(&mcStart));
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00438, 0));

    *McStart = mcStart;

    if (Hardware->core <= gcvCORE_3D_MAX)
    {
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, 0xFFU << 24));

        gcmkONERROR(gckOS_GetTime(&shStart));

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00470, 0x4U << 24));

        *ShStart = shStart;
    }

OnError:
    return status;
}

gceSTATUS
gckHARDWARE_ExitQueryClock(
    IN gckHARDWARE Hardware,
    IN gctUINT64 McStart,
    IN gctUINT64 ShStart,
    OUT gctUINT32 *McClk,
    OUT gctUINT32 *ShClk
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT64 mcEnd, shEnd;
    gctUINT32 mcCycle, shCycle;
    gctUINT64 mcFreq, shFreq = 0;

    gcmkONERROR(gckOS_GetTime(&mcEnd));
    gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x00438, &mcCycle));

    if (mcCycle == 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* cycle = (gctUINT64)cycle * 1000000 / (end - start); */
    mcFreq = ((gctUINT64)mcCycle * ((1000000U << 12) / (gctUINT32)(mcEnd - McStart))) >> 12;

    *McClk = (gctUINT32)mcFreq;

    if (Hardware->core <= gcvCORE_3D_MAX)
    {
        gcmkONERROR(gckOS_GetTime(&shEnd));
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, Hardware->core, 0x0045C, &shCycle));

        if (!shCycle)
        {
            *ShClk = *McClk;
            return gcvSTATUS_OK;
        }

        if (!ShStart)
        {
            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        shFreq = ((gctUINT64)shCycle * ((1000000U << 12) / (gctUINT32)(shEnd - ShStart))) >> 12;
    }

    *ShClk = (gctUINT32)shFreq;

OnError:
    return status;
}

/*******************************************************************************
**
**  gckHARDWARE_QueryFrequency
**
**  Query current hardware frequency.
**
**  INPUT:
**
**      gckHARDWARE Hardware
**          Pointer to an gckHARDWARE object.
**
*/
gceSTATUS
gckHARDWARE_QueryFrequency(
    IN gckHARDWARE Hardware
    )
{
    gctUINT64 mcStart, shStart;
    gctUINT32 mcClk, shClk;
    gceSTATUS status;
    gctUINT64 powerManagement = 0;
    gctBOOL globalAcquired = gcvFALSE;
    gceCHIPPOWERSTATE statesStored, state;

    gcmkHEADER_ARG("Hardware=0x%p", Hardware);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    mcStart = shStart = 0;
    mcClk   = shClk   = 0;

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

    /* Grab the global semaphore. */
    gcmkONERROR(gckOS_AcquireSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvTRUE;

    gckHARDWARE_EnterQueryClock(Hardware, &mcStart, &shStart);

    gcmkONERROR(gckOS_Delay(Hardware->os, 50));

    if (mcStart)
    {
        gckHARDWARE_ExitQueryClock(Hardware,
                                   mcStart, shStart,
                                   &mcClk, &shClk);

        Hardware->mcClk = mcClk;
        Hardware->shClk = shClk;
    }

    /* Release the global semaphore. */
    gcmkONERROR(gckOS_ReleaseSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvFALSE;

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

    gcmkFOOTER_NO();

    return gcvSTATUS_OK;

OnError:
    if (globalAcquired)
    {
        /* Release the global semaphore. */
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(
            Hardware->os, Hardware->globalSemaphore
            ));
    }

    gcmkFOOTER();

    return status;
}

/*******************************************************************************
**
** Set MC and SH clock
**
** Core   : which core clock do you want to set
** mcScale: MC clock scale
** shScale: SH clock scale
*/
gceSTATUS
gckHARDWARE_SetClock(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Core,
    IN gctUINT32 MCScale,
    IN gctUINT32 SHScale
    )
{
    gceSTATUS status;
    gctUINT64 powerManagement = 0;
    gctBOOL globalAcquired = gcvFALSE;
    gctUINT32 org;
    gctUINT32 core = Core;
    gctUINT32 mcScale = MCScale;
    gctUINT32 shScale = SHScale;

    gcmkHEADER();

    status = gckOS_QueryOption(Hardware->os, "powerManagement", &powerManagement);
    if (gcmIS_ERROR(status))
    {
        powerManagement = 0;
    }

    if (powerManagement)
    {
        gcmkONERROR(gckHARDWARE_EnablePowerManagement(
            Hardware, gcvFALSE
            ));

        gcmkPRINT("Warning: Power management status will be changed forever!\n");
    }

    gcmkONERROR(gckHARDWARE_SetPowerState(
        Hardware, gcvPOWER_ON_AUTO
        ));

    /* Grab the global semaphore. */
    gcmkONERROR(gckOS_AcquireSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvTRUE;

    if (mcScale > 0 && mcScale <= 64)
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, core, 0x00000, &org));

        org = ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 8:2) - (0 ?
 8:2) + 1))))))) << (0 ?
 8:2))) | (((gctUINT32) ((gctUINT32) (mcScale) & ((gctUINT32) ((((1 ?
 8:2) - (0 ?
 8:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 8:2) - (0 ? 8:2) + 1))))))) << (0 ? 8:2)));

        /* Write the clock control register. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, core,
                            0x00000,
                            ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)))));

        /* Done loading the frequency scaler. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, core,
                            0x00000,
                            ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 9:9) - (0 ?
 9:9) + 1))))))) << (0 ?
 9:9))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 9:9) - (0 ?
 9:9) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1))))))) << (0 ? 9:9)))));

        /* Need to change 0x0010C when it is introduced. */
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, core, 0x0010C, &org));

        /* Never impact shader clk. */
        org = 0x01020800 | (org & 0xFF);

        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, core, 0x0010C, org));
    }

    /* set SH clock */
    if (shScale > 0 && shScale <= 64)
    {
        gcmkONERROR(gckOS_ReadRegisterEx(Hardware->os, core, 0x0010C, &org));

        org = ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 7:1) - (0 ?
 7:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 7:1) - (0 ?
 7:1) + 1))))))) << (0 ?
 7:1))) | (((gctUINT32) ((gctUINT32) (shScale) & ((gctUINT32) ((((1 ?
 7:1) - (0 ?
 7:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 7:1) - (0 ? 7:1) + 1))))))) << (0 ? 7:1)));
        org = ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16)));
        org = ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 17:17) - (0 ?
 17:17) + 1))))))) << (0 ?
 17:17))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 17:17) - (0 ?
 17:17) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 17:17) - (0 ? 17:17) + 1))))))) << (0 ? 17:17)));

        /* Write the clock control register. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, core,
                            0x0010C,
                            ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));

        /* Done loading the frequency scaler. */
        gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, core,
                            0x0010C,
                            ((((gctUINT32) (org)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))));
    }

    /* Release the global semaphore. */
    gcmkONERROR(gckOS_ReleaseSemaphore(
        Hardware->os, Hardware->globalSemaphore
        ));

    globalAcquired = gcvFALSE;

    gcmkFOOTER_NO();

    return gcvSTATUS_OK;

OnError:
    if (globalAcquired)
    {
        /* Release the global semaphore. */
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(
            Hardware->os, Hardware->globalSemaphore
            ));
    }

    gcmkFOOTER_NO();

    return status;
}

gceSTATUS
gckHARDWARE_QueryCycleCount(
    IN gckHARDWARE Hardware,
    OUT gctUINT32 *hi_total_cycle_count,
    OUT gctUINT32 *hi_total_idle_cycle_count
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gceCHIPMODEL chipModel;

    gcmkHEADER_ARG("Hardware=0x%p hi_total_cycle_count=0x%p hi_total_idle_cycle_count=0x%p", Hardware, hi_total_cycle_count, hi_total_idle_cycle_count);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    chipModel = Hardware->identity.chipModel;

    if (chipModel == gcv2100 || chipModel == gcv2000 || chipModel == gcv880)
    {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00078,
            hi_total_idle_cycle_count));

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00438,
            hi_total_cycle_count));
     }
     else
     {
        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x0007C,
            hi_total_idle_cycle_count));

        gcmkONERROR(
            gckOS_ReadRegisterEx(Hardware->os,
            Hardware->core,
            0x00078,
            hi_total_cycle_count));
      }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_CleanCycleCount(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Hardware=0x%p", Hardware);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x0007C, 0));

    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00438, 0));

    gcmkONERROR(
        gckOS_WriteRegisterEx(Hardware->os, Hardware->core, 0x00078, 0));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckHARDWARE_QueryCoreLoad(
    IN gckHARDWARE Hardware,
    OUT gctUINT32 *Load
    )
{
    gctUINT32 i = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gceCHIPPOWERSTATE statesStored, state;
    gctBOOL powerManagement = gcvFALSE;
    static gctUINT32 hardwareCount = 0;
    static gctBOOL profilerEnable = gcvFALSE;
    gckHARDWARE hardware[gcvCORE_3D_MAX + 1] = {gcvNULL};
    gctUINT32 hi_total_cycle_count = 0, hi_total_idle_cycle_count =0;

    gcmkHEADER_ARG("Hardware=0x%p Load=0x%p", Hardware, Load);

    gcmkVERIFY_ARGUMENT(Hardware != gcvNULL);

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

    if (hardwareCount == 0)
    {
        hardware[0] = Hardware;
        hardwareCount = 1;
    }
    else
    {
        for (i = 0; i < hardwareCount; i++)
        {
            if (Hardware == hardware[i])
            {
                break;
            }
            else if (i == hardwareCount - 1)
            {
                profilerEnable = gcvFALSE;
                hardware[hardwareCount] = Hardware;
                hardwareCount++;
                break;
            }
        }
    }

    if (!profilerEnable)
    {
        gcmkONERROR(gckHARDWARE_SetGpuProfiler(
            Hardware,
            gcvTRUE
            ));

        gcmkONERROR(gckHARDWARE_InitProfiler(Hardware));

        profilerEnable = gcvTRUE;
    }

    Hardware->waitCount = 200 * 100;

    gcmkONERROR(gckHARDWARE_CleanCycleCount(Hardware));

    gcmkONERROR(gckHARDWARE_QueryCycleCount(Hardware,
        &hi_total_cycle_count,
        &hi_total_idle_cycle_count));

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

    *Load = (hi_total_cycle_count - hi_total_idle_cycle_count) * 100 / hi_total_cycle_count;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

