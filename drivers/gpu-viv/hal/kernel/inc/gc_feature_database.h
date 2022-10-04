/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2020 Vivante Corporation
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
*    Copyright (C) 2014 - 2020 Vivante Corporation
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


/*Auto created on 2021-05-18 17:28*/
#ifndef _gc_feature_database_h_
#define _gc_feature_database_h_

typedef struct
{
    /* Chip ID. */
    gctUINT32 chipID;
    gctUINT32 chipVersion;
    gctUINT32 productID;
    gctUINT32 ecoID;
    gctUINT32 customerID;
    gctUINT32 patchVersion;
    const char *productName;
    gctUINT32 formalRelease;
    gctUINT32 TempRegisters;
    gctUINT32 ThreadCount;
    gctUINT32 NumShaderCores;
    gctUINT32 InstructionCount;
    gctUINT32 NumberOfConstants;
    gctUINT32 CoreCount;
    gctUINT32 LocalStorageSize;
    gctUINT32 LocalStorageSize_1;
    gctUINT32 LocalStorageSize_2;
    gctUINT32 L1CacheSize;
    gctUINT32 L1CacheSize_1;
    gctUINT32 L1CacheSize_2;
    gctUINT32 InstructionMemorySize;
    gctUINT32 ShaderPCLength;
    gctUINT32 USC_MAX_PAGES;
    gctUINT32 USC_MAX_PAGES_1;
    gctUINT32 USC_MAX_PAGES_2;
    gctUINT32 NumPixelPipes;
    gctUINT32 USC_CACHE_CONTROLLERS;
    gctUINT32 USC_CACHE_CONTROLLERS_1;
    gctUINT32 USC_CACHE_CONTROLLERS_2;
    gctUINT32 USC_BANKS;
    gctUINT32 USC_BANKS_1;
    gctUINT32 USC_BANKS_2;
    gctUINT32 Streams;
    gctUINT32 VaryingCount;
    gctUINT32 VertexOutputBufferSize;
    gctUINT32 BufferSize;
    gctUINT32 VertexCacheSize;
    gctUINT32 NumResolvePipes;
    gctUINT32 RESULT_WINDOW_MAX_SIZE;
    gctUINT32 ClusterAliveMask;
    gctUINT32 G2D_DEC400_MINOR;
    gctUINT32 PS_INSTRUCTION_COUNT;
    gctUINT32 NNMadPerCore;
    gctUINT32 NNCoreCount;
    gctUINT32 NN_ACTIVE_CORE_COUNT;
    gctUINT32 NNCoreCount_INT8;
    gctUINT32 NNCoreCount_INT16;
    gctUINT32 NNCoreCount_FLOAT16;
    gctUINT32 NNCoreCount_BFLOAT;
    gctUINT32 NNInputBufferDepth;
    gctUINT32 NNAccumBufferDepth;
    gctUINT32 TPEngine_PwlLUTCount;
    gctUINT32 TPEngine_PwlLUTSize;
    gctUINT32 VIP_SRAM_SIZE;
    gctUINT32 TPEngine_CoreCount;
    gctUINT32 AXI_SRAM_SIZE;
    gctUINT32 NN_INIMAGE_OFFSET_BITS;
    gctUINT32 TP_REORDER_INIMAGE_SIZE;
    gctUINT32 TPLite_CoreCount;
    gctUINT32 NN_PREPROCESSOR_MAX_SEGMENT_PER_CYCLE;
    gctUINT32 NNFP16_XYDP_X;
    gctUINT32 NNFP16_XYDP_Y;
    gctUINT32 NNFP16_ZDP;
    gctUINT32 NN_LANES_PER_OUT_CYCLE;
    gctUINT32 MAX_OT_NUMBER;
    gctUINT32 PHYSICAL_VIP_SRAM_WIDTH_IN_BYTE;
    gctUINT32 EQUIVALENT_VIP_SRAM_WIDTH_INBYTE;
    gctUINT32 TP_ZRL_BITS;
    gctUINT32 NN_ZRL_BITS;
    gctUINT32 LATENCY_HIDING_AT_FULL_AXI_BW;
    gctUINT32 AXI_BUS_WIDTH;
    gctUINT32 NN_KERNEL_X_SIZE;
    gctUINT32 NN_KERNEL_Y_SIZE;
    gctUINT32 NN_FC_KERNEL_Y_SIZE;
    gctUINT32 NN_KERNEL_Z_SIZE;
    gctUINT32 NN_X_OFFSET;
    gctUINT32 NN_Y_OFFSET;
    gctUINT32 DDR_KERNEL_BURST_SIZE;
    gctUINT32 OUTIMAGE_X_STRIDE_BITS;
    gctUINT32 OUTIMAGE_Y_STRIDE_BITS;
    gctUINT32 INIMAGE_X_STRIDE_BITS;
    gctUINT32 IMIMAGE_Y_STRIDE_BITS;
    gctUINT32 OUTIMAGE_X_SIZE_BITS;
    gctUINT32 OUTIMAGE_Y_SIZE_BITS;
    gctUINT32 OUTIMAGE_Z_SIZE_BITS;
    gctUINT32 INIMAGE_X_SIZE_BITS;
    gctUINT32 INIMAGE_Y_SIZE_BITS;
    gctUINT32 MAX_TILE_X_SIZE;
    gctUINT32 NN_CLUSTER_NUM_FOR_POWER_CONTROL;
    gctUINT32 NN_IN_LINES_PER_CYCLE;
    gctUINT32 VIP_CLUSTER_COUNT;
    gctUINT32 NN_MP_INTER_CONNECT_RING_COUNT;
    gctUINT32 REG_FastClear:1;
    gctUINT32 REG_SpecialAntiAliasing:1;
    gctUINT32 REG_Pipe3D:1;
    gctUINT32 REG_DXTTextureCompression:1;
    gctUINT32 REG_DebugMode:1;
    gctUINT32 REG_ZCompression:1;
    gctUINT32 REG_YUV420Filter:1;
    gctUINT32 REG_MSAA:1;
    gctUINT32 REG_DC:1;
    gctUINT32 REG_Pipe2D:1;
    gctUINT32 REG_ETC1TextureCompression:1;
    gctUINT32 REG_FastScaler:1;
    gctUINT32 REG_HighDynamicRange:1;
    gctUINT32 REG_YUV420Tiler:1;
    gctUINT32 REG_ModuleCG:1;
    gctUINT32 REG_MinArea:1;
    gctUINT32 REG_NoEZ:1;
    gctUINT32 REG_No422Texture:1;
    gctUINT32 REG_BufferInterleaving:1;
    gctUINT32 REG_ByteWrite2D:1;
    gctUINT32 REG_NoScaler:1;
    gctUINT32 REG_YUY2Averaging:1;
    gctUINT32 REG_HalfPECache:1;
    gctUINT32 REG_HalfTXCache:1;
    gctUINT32 REG_YUY2RenderTarget:1;
    gctUINT32 REG_Mem32BitSupport:1;
    gctUINT32 REG_PipeVG:1;
    gctUINT32 REG_VGTS:1;
    gctUINT32 REG_FE20:1;
    gctUINT32 REG_ByteWrite3D:1;
    gctUINT32 REG_RsYuvTarget:1;
    gctUINT32 REG_FE20BitIndex:1;
    gctUINT32 REG_FlipY:1;
    gctUINT32 REG_DualReturnBus:1;
    gctUINT32 REG_EndiannessConfig:1;
    gctUINT32 REG_Texture8K:1;
    gctUINT32 REG_CorrectTextureConverter:1;
    gctUINT32 REG_SpecialMsaaLod:1;
    gctUINT32 REG_FastClearFlush:1;
    gctUINT32 REG_2DPE20:1;
    gctUINT32 REG_CorrectAutoDisable:1;
    gctUINT32 REG_Render8K:1;
    gctUINT32 REG_TileStatus2Bits:1;
    gctUINT32 REG_SeparateTileStatusWhenInterleaved:1;
    gctUINT32 REG_SuperTiled32x32:1;
    gctUINT32 REG_VG20:1;
    gctUINT32 REG_TSExtendedCommands:1;
    gctUINT32 REG_CompressionFifoFixed:1;
    gctUINT32 REG_ExtraShaderInstructions0:1;
    gctUINT32 REG_VGFilter:1;
    gctUINT32 REG_VG21:1;
    gctUINT32 REG_ShaderGetsW:1;
    gctUINT32 REG_ExtraShaderInstructions1:1;
    gctUINT32 REG_DefaultReg0:1;
    gctUINT32 REG_MC20:1;
    gctUINT32 REG_ShaderMSAASideband:1;
    gctUINT32 REG_BugFixes0:1;
    gctUINT32 REG_VAA:1;
    gctUINT32 REG_BypassInMSAA:1;
    gctUINT32 REG_HierarchicalZ:1;
    gctUINT32 REG_NewTexture:1;
    gctUINT32 REG_A8TargetSupport:1;
    gctUINT32 REG_CorrectStencil:1;
    gctUINT32 REG_EnhanceVR:1;
    gctUINT32 REG_RSUVSwizzle:1;
    gctUINT32 REG_V2Compression:1;
    gctUINT32 REG_VGDoubleBuffer:1;
    gctUINT32 REG_BugFixes1:1;
    gctUINT32 REG_BugFixes2:1;
    gctUINT32 REG_TextureStride:1;
    gctUINT32 REG_BugFixes3:1;
    gctUINT32 REG_CorrectAutoDisable1:1;
    gctUINT32 REG_AutoRestartTS:1;
    gctUINT32 REG_BugFixes4:1;
    gctUINT32 REG_L2Windowing:1;
    gctUINT32 REG_HalfFloatPipe:1;
    gctUINT32 REG_PixelDither:1;
    gctUINT32 REG_TwoStencilReference:1;
    gctUINT32 REG_ExtendedPixelFormat:1;
    gctUINT32 REG_CorrectMinMaxDepth:1;
    gctUINT32 REG_DitherAndFilterPlusAlpha2D:1;
    gctUINT32 REG_BugFixes5:1;
    gctUINT32 REG_New2D:1;
    gctUINT32 REG_NewFloatingPointArithmetic:1;
    gctUINT32 REG_TextureHorizontalAlignmentSelect:1;
    gctUINT32 REG_NonPowerOfTwo:1;
    gctUINT32 REG_LinearTextureSupport:1;
    gctUINT32 REG_Halti0:1;
    gctUINT32 REG_CorrectOverflowVG:1;
    gctUINT32 REG_NegativeLogFix:1;
    gctUINT32 REG_ResolveOffset:1;
    gctUINT32 REG_OkToGateAxiClock:1;
    gctUINT32 REG_MMU:1;
    gctUINT32 REG_WideLine:1;
    gctUINT32 REG_BugFixes6:1;
    gctUINT32 REG_FcFlushStall:1;
    gctUINT32 REG_LineLoop:1;
    gctUINT32 REG_LogicOp:1;
    gctUINT32 REG_SeamlessCubeMap:1;
    gctUINT32 REG_SuperTiledTexture:1;
    gctUINT32 REG_LinearPE:1;
    gctUINT32 REG_RectPrimitive:1;
    gctUINT32 REG_Composition:1;
    gctUINT32 REG_CorrectAutoDisableCountWidth:1;
    gctUINT32 REG_PESwizzle:1;
    gctUINT32 REG_EndEvent:1;
    gctUINT32 REG_S1S8:1;
    gctUINT32 REG_Halti1:1;
    gctUINT32 REG_RGB888:1;
    gctUINT32 REG_TX_YUVAssembler:1;
    gctUINT32 REG_DynamicFrequencyScaling:1;
    gctUINT32 REG_TXFilter:1;
    gctUINT32 REG_FullDirectFB:1;
    gctUINT32 REG_OnePass2DFilter:1;
    gctUINT32 REG_ThreadWalkerInPS:1;
    gctUINT32 REG_TileFiller:1;
    gctUINT32 REG_YUVStandard:1;
    gctUINT32 REG_MultiSourceBlt:1;
    gctUINT32 REG_YUVConversion:1;
    gctUINT32 REG_FlushFixed2D:1;
    gctUINT32 REG_Interleaver:1;
    gctUINT32 REG_MixedStreams:1;
    gctUINT32 REG_L2CacheFor2D420:1;
    gctUINT32 REG_BugFixes7:1;
    gctUINT32 REG_NoIndexPattern:1;
    gctUINT32 REG_TextureTileStatus:1;
    gctUINT32 REG_DecompressZ16:1;
    gctUINT32 REG_BugFixes8:1;
    gctUINT32 REG_DERotationStallFix:1;
    gctUINT32 REG_OclOnly:1;
    gctUINT32 REG_NewFeatures0:1;
    gctUINT32 REG_InstructionCache:1;
    gctUINT32 REG_GeometryShader:1;
    gctUINT32 REG_TexCompressionSupertiled:1;
    gctUINT32 REG_Generics:1;
    gctUINT32 REG_BugFixes9:1;
    gctUINT32 REG_FastMSAA:1;
    gctUINT32 REG_WClip:1;
    gctUINT32 REG_BugFixes10:1;
    gctUINT32 REG_UnifiedSamplers:1;
    gctUINT32 REG_BugFixes11:1;
    gctUINT32 REG_PerformanceCounters:1;
    gctUINT32 REG_ExtraShaderInstructions2:1;
    gctUINT32 REG_BugFixes12:1;
    gctUINT32 REG_BugFixes13:1;
    gctUINT32 REG_DEEnhancements1:1;
    gctUINT32 REG_ACE:1;
    gctUINT32 REG_TXEnhancements1:1;
    gctUINT32 REG_SHEnhancements1:1;
    gctUINT32 REG_SHEnhancements2:1;
    gctUINT32 REG_PEEnhancements1:1;
    gctUINT32 REG_DEEnhancements2:1;
    gctUINT32 REG_BugFixes14:1;
    gctUINT32 REG_PowerOptimizations0:1;
    gctUINT32 REG_NewHZ:1;
    gctUINT32 REG_BugFixes15:1;
    gctUINT32 REG_DEEnhancements3:1;
    gctUINT32 REG_SHEnhancements3:1;
    gctUINT32 REG_SHEnhancements4:1;
    gctUINT32 REG_TXEnhancements2:1;
    gctUINT32 REG_FEEnhancements1:1;
    gctUINT32 REG_PEEnhancements2:1;
    gctUINT32 REG_PAEnhancements1:1;
    gctUINT32 REG_DENoGamma:1;
    gctUINT32 REG_PAEnhancements2:1;
    gctUINT32 REG_DEEnhancements4:1;
    gctUINT32 REG_PEEnhancements3:1;
    gctUINT32 REG_HIEnhancements1:1;
    gctUINT32 REG_TXEnhancements3:1;
    gctUINT32 REG_SHEnhancements5:1;
    gctUINT32 REG_FEEnhancements2:1;
    gctUINT32 REG_BugFixes16:1;
    gctUINT32 REG_DEEnhancements5:1;
    gctUINT32 REG_TXEnhancements4:1;
    gctUINT32 REG_PEEnhancements4:1;
    gctUINT32 REG_MCEnhancements1:1;
    gctUINT32 REG_Halti2:1;
    gctUINT32 REG_DEMirrorRotate:1;
    gctUINT32 REG_SmallMSAA:1;
    gctUINT32 REG_BugFixes17:1;
    gctUINT32 REG_Rasterizer2:1;
    gctUINT32 REG_DualPipeOPF:1;
    gctUINT32 REG_MultiSrcV2:1;
    gctUINT32 REG_CSCV2:1;
    gctUINT32 REG_PAEnhancements3:1;
    gctUINT32 REG_BugFixes18:1;
    gctUINT32 REG_Compression2D:1;
    gctUINT32 REG_Probe:1;
    gctUINT32 REG_MediumPrecision:1;
    gctUINT32 REG_DESupertile:1;
    gctUINT32 REG_BugFixes19:1;
    gctUINT32 REG_SHEnhancements6:1;
    gctUINT32 REG_SHEnhancements7:1;
    gctUINT32 REG_BugFixes20:1;
    gctUINT32 REG_DEAddress40:1;
    gctUINT32 REG_MiniMMUFix:1;
    gctUINT32 REG_EEZ:1;
    gctUINT32 REG_BugFixes21:1;
    gctUINT32 REG_ExtraVgCaps:1;
    gctUINT32 REG_MultiSrcV15:1;
    gctUINT32 REG_BugFixes22:1;
    gctUINT32 REG_Halti3:1;
    gctUINT32 REG_TessellationShaders:1;
    gctUINT32 REG_OPF9Tap:1;
    gctUINT32 REG_MultiSrcV2StrQuad:1;
    gctUINT32 REG_SeperateSRCAndDstCache:1;
    gctUINT32 REG_Halti4:1;
    gctUINT32 REG_RAWriteDepth:1;
    gctUINT32 REG_AndroidOnly:1;
    gctUINT32 REG_HasChipProductReg:1;
    gctUINT32 REG_TXSupportDEC:1;
    gctUINT32 REG_S8MSAACompression:1;
    gctUINT32 REG_BugFixesIn544:1;
    gctUINT32 REG_L2CacheRemove:1;
    gctUINT32 REG_FEAllowRndVtxCnt:1;
    gctUINT32 REG_CubeMapFL28:1;
    gctUINT32 REG_TX6bitFrac:1;
    gctUINT32 REG_FEAllowStallPrefetchEng:1;
    gctUINT32 REG_ThirdPartyCompression:1;
    gctUINT32 REG_RSS8:1;
    gctUINT32 REG_MSAACoherencyCheck:1;
    gctUINT32 REG_Halti5:1;
    gctUINT32 REG_Evis:1;
    gctUINT32 REG_BltEngine:1;
    gctUINT32 REG_BugFixes23:1;
    gctUINT32 REG_BugFixes24:1;
    gctUINT32 REG_DEC:1;
    gctUINT32 REG_VSTileNV12:1;
    gctUINT32 REG_VSTileNV12_10BIT:1;
    gctUINT32 REG_DisableVIP:1;
    gctUINT32 RenderTarget8:1;
    gctUINT32 TxLodFlowCorrection:1;
    gctUINT32 FaceLod:1;
    gctUINT32 MultiCoreSemaphoreStallV2:1;
    gctUINT32 VMSAA:1;
    gctUINT32 ChipEnableLink:1;
    gctUINT32 MULTI_SRC_BLT_1_5_ENHANCEMENT:1;
    gctUINT32 MULTI_SRC_BLT_BILINEAR_FILTER:1;
    gctUINT32 RA_HZEZ_CLOCK_CONTROL:1;
    gctUINT32 CACHE128B256BPERLINE:1;
    gctUINT32 V4Compression:1;
    gctUINT32 DE2D_MAJOR_SUPER_TILE:1;
    gctUINT32 PE2D_MAJOR_SUPER_TILE:1;
    gctUINT32 PE_32BPC_COLORMASK_FIX:1;
    gctUINT32 ALPHA_BLENDING_OPT:1;
    gctUINT32 NEW_GPIPE:1;
    gctUINT32 PIPELINE_32_ATTRIBUTES:1;
    gctUINT32 MSAA_SHADING:1;
    gctUINT32 NO_ANISTRO_FILTER:1;
    gctUINT32 NO_ASTC:1;
    gctUINT32 NO_DXT:1;
    gctUINT32 HWTFB:1;
    gctUINT32 RA_DEPTH_WRITE_MSAA1X_FIX:1;
    gctUINT32 EZHZ_CLOCKGATE_FIX:1;
    gctUINT32 SH_SNAP2PAGE_FIX:1;
    gctUINT32 SH_HALFDEPENDENCY_FIX:1;
    gctUINT32 USC_MCFILL_FIX:1;
    gctUINT32 TPG_TCPERF_FIX:1;
    gctUINT32 USC_MDFIFO_OVERFLOW_FIX:1;
    gctUINT32 SH_TEXLD_BARRIER_IN_CS_FIX:1;
    gctUINT32 RS_NEW_BASEADDR:1;
    gctUINT32 PE_8bpp_DUALPIPE_FIX:1;
    gctUINT32 SH_ADVANCED_INSTR:1;
    gctUINT32 SH_FLAT_INTERPOLATION_DUAL16_FIX:1;
    gctUINT32 USC_CONTINUOUS_FLUS_FIX:1;
    gctUINT32 SH_SUPPORT_V4:1;
    gctUINT32 SH_SUPPORT_ALPHA_KILL:1;
    gctUINT32 PE_NO_ALPHA_TEST:1;
    gctUINT32 TX_LOD_NEAREST_SELECT:1;
    gctUINT32 SH_FIX_LDEXP:1;
    gctUINT32 SUPPORT_MOVAI:1;
    gctUINT32 SH_SNAP2PAGE_MAXPAGES_FIX:1;
    gctUINT32 PE_RGBA16I_FIX:1;
    gctUINT32 BLT_8bpp_256TILE_FC_FIX:1;
    gctUINT32 PE_64bit_FENCE_FIX:1;
    gctUINT32 USC_FULL_CACHE_FIX:1;
    gctUINT32 TX_YUV_ASSEMBLER_10BIT:1;
    gctUINT32 FE_32bit_INDEX_FIX:1;
    gctUINT32 BLT_64bpp_MASKED_CLEAR_FIX:1;
    gctUINT32 SECURITY:1;
    gctUINT32 ROBUSTNESS:1;
    gctUINT32 USC_ATOMIC_FIX:1;
    gctUINT32 SH_PSO_MSAA1x_FIX:1;
    gctUINT32 USC_VX_PERF_FIX:1;
    gctUINT32 USC_GOS_ADDR_FIX:1;
    gctUINT32 TX_8bit_UVFrac:1;
    gctUINT32 TX_DESC_CACHE_CLOCKGATE_FIX:1;
    gctUINT32 RSBLT_MSAA_DECOMPRESSION:1;
    gctUINT32 TX_INTEGER_COORDINATE:1;
    gctUINT32 DRAWID:1;
    gctUINT32 PSIO_SAMPLEMASK_IN_R0ZW_FIX:1;
    gctUINT32 TX_INTEGER_COORDINATE_V2:1;
    gctUINT32 MULTI_CORE_BLOCK_SET_CONFIG:1;
    gctUINT32 SNAPPAGE_CMD:1;
    gctUINT32 SH_NO_INDEX_CONST_ON_A0:1;
    gctUINT32 SH_NO_ONECONST_LIMIT:1;
    gctUINT32 SH_IMG_LDST_ON_TEMP:1;
    gctUINT32 COMPUTE_ONLY:1;
    gctUINT32 SH_IMG_LDST_CLAMP:1;
    gctUINT32 SH_ICACHE_ALLOC_COUNT_FIX:1;
    gctUINT32 SH_ICACHE_PREFETCH:1;
    gctUINT32 PE2D_SEPARATE_CACHE:1;
    gctUINT32 PE_MSAA_OQ_FIX:1;
    gctUINT32 PSIO_MSAA_CL_FIX:1;
    gctUINT32 USC_DEFER_FILL_FIX:1;
    gctUINT32 SH_CLOCK_GATE_FIX:1;
    gctUINT32 FE_NEED_DUMMYDRAW:1;
    gctUINT32 PE2D_LINEAR_YUV420_OUTPUT:1;
    gctUINT32 PE2D_LINEAR_YUV420_10BIT:1;
    gctUINT32 MULTI_CLUSTER:1;
    gctUINT32 SH_MULTI_WG_PACK:1;
    gctUINT32 SH_DUAL16_SAMPLEMASK_ZW:1;
    gctUINT32 TPG_TRIVIAL_MODE_FIX:1;
    gctUINT32 TX_ASTC_MULTISLICE_FIX:1;
    gctUINT32 FE_ROBUST_FIX:1;
    gctUINT32 SH_GPIPE_ACCESS_FULLTEMPS:1;
    gctUINT32 PSIO_INTERLOCK:1;
    gctUINT32 PA_WIDELINE_FIX:1;
    gctUINT32 WIDELINE_HELPER_FIX:1;
    gctUINT32 G2D_3rd_PARTY_COMPRESSION_1_1:1;
    gctUINT32 TX_FLUSH_L1CACHE:1;
    gctUINT32 PE_DITHER_FIX2:1;
    gctUINT32 SH_TEXLD_U_FIX:1;
    gctUINT32 MC_FCCACHE_BYTEMASK:1;
    gctUINT32 SH_MULTI_WG_PACK_FIX:1;
    gctUINT32 PE_ADVANCE_BLEND_PART0:1;
    gctUINT32 FE_PATCHLIST_FETCH_FIX:1;
    gctUINT32 RA_CG_FIX:1;
    gctUINT32 DEC400:1;
    gctUINT32 LS_SUPPORT_PERCOMP_DEPENDENCY:1;
    gctUINT32 MULTI_CORE_BLOCK_SET_CONFIG2:1;
    gctUINT32 PE_VMSAA_COVERAGE_CACHE_FIX:1;
    gctUINT32 SECURITY_AHB:1;
    gctUINT32 MULTICORE_SEMAPHORESTALL_V3:1;
    gctUINT32 SMALLBATCH:1;
    gctUINT32 SH_CMPLX:1;
    gctUINT32 SH_IDIV0_SWZL_EHS:1;
    gctUINT32 TX_LERP_LESS_BIT:1;
    gctUINT32 SH_GM_ENDIAN:1;
    gctUINT32 SH_GM_USC_UNALLOC:1;
    gctUINT32 SH_END_OF_BB:1;
    gctUINT32 TX_BORDER_CLAMP_FIX:1;
    gctUINT32 SH_IMG_LD_LASTPIXEL_FIX:1;
    gctUINT32 ASYNC_BLT:1;
    gctUINT32 ASYNC_FE_FENCE_FIX:1;
    gctUINT32 PSCS_THROTTLE:1;
    gctUINT32 SEPARATE_LS:1;
    gctUINT32 WIDELINE_TRIANGLE_EMU:1;
    gctUINT32 FENCE_32BIT:1;
    gctUINT32 FENCE_64BIT:1;
    gctUINT32 PE_DEPTH_ONLY_OQFIX:1;
    gctUINT32 TX_SEAMLESS_CUBE:1;
    gctUINT32 TX_SNORM_SUPPORT:1;
    gctUINT32 SH_SCATTER_GATHER:1;
    gctUINT32 HWMANAGED_LS:1;
    gctUINT32 SH_IMAGE_ENABLE_FIX:1;
    gctUINT32 MSAA_FRAGMENT_OPERATION:1;
    gctUINT32 PE_TILE_CACHE_FLUSH_FIX:1;
    gctUINT32 BLT_YUV_OUTPUT:1;
    gctUINT32 SH_IO_CG_FIX:1;
    gctUINT32 PE_SWIZZLE:1;
    gctUINT32 SH_ROBUSTNESS_FIX:1;
    gctUINT32 USC_ATOMIC_FIX2:1;
    gctUINT32 PE_A8B8G8R8:1;
    gctUINT32 MULTIVIEW_RENDER:1;
    gctUINT32 FE_DRAW_DIRECT:1;
    gctUINT32 TX_VKBORDER_MODE:1;
    gctUINT32 TX_UNNORMALIZED_COORD:1;
    gctUINT32 PA_LINECLIP_FIX:1;
    gctUINT32 TX_8bit_UVFrac_ROUNDING_FIX:1;
    gctUINT32 MP_ARCH:1;
    gctUINT32 TX_NO_FIXED_FILTER:1;
    gctUINT32 SHARE_Z:1;
    gctUINT32 DE_2D_FAST_CLEAR:1;
    gctUINT32 DE_TILESTATUS_ROTATION_FIX:1;
    gctUINT32 TX_CLEAR_PENDING_FIX:1;
    gctUINT32 HI1_L2_CACHE:1;
    gctUINT32 USC_EVICT_CTRL_FIFO_FLOP_RESET_FIX:1;
    gctUINT32 FORMAT_10BIT_CROSS_4K:1;
    gctUINT32 FORMAT_P010LSB_I010:1;
    gctUINT32 ENDIAN_CONTROL:1;
    gctUINT32 G2D_RGB_PLANAR:1;
    gctUINT32 G2D_DEC400EX:1;
    gctUINT32 G2D_NO_YUV420_SOURCE:1;
    gctUINT32 G2D_YUV420_101010:1;
    gctUINT32 G2D_MultiSrcBlt_Pipe:1;
    gctUINT32 G2D_Normalization:1;
    gctUINT32 G2D_MASK_AND_COLORKEY:1;
    gctUINT32 SH_VX2_FLOATING_MAD_FIX:1;
    gctUINT32 TS_FC_VULKAN_SUPPORT:1;
    gctUINT32 MSAA_FLOAT_64BIT:1;
    gctUINT32 INDIRECT_COMPUTE_ZERODIM_FIX:1;
    gctUINT32 Q_CHANNEL_SUPPORT:1;
    gctUINT32 MMU_PAGE_DESCRIPTOR:1;
    gctUINT32 YUV_LINEAR_TO_TILE_ROTATE:1;
    gctUINT32 VEC2_IMULIMAD32_SUPPORT:1;
    gctUINT32 VEC4_IMULIMAD32_SUPPORT:1;
    gctUINT32 VEC2_IDIVIMOD16_SUPPORT:1;
    gctUINT32 DST_TEX_I2F_F2I_INST_DEPRECATE:1;
    gctUINT32 ALU_FP16_INSTRUCTIONS:1;
    gctUINT32 DUAL16_14BIT_PC_SUPPORT:1;
    gctUINT32 LDST_CONV_4ROUNDING_MODES:1;
    gctUINT32 FULL_PACK_MODE_SUPPORT:1;
    gctUINT32 DEPTH_FLOAT32_SUPPORT:1;
    gctUINT32 GPU_INSPECTOR_COUNTERS:1;
    gctUINT32 FP32_TO_FP16_CONV_FIX:1;
    gctUINT32 IMGLD_COMP_COUNT_FIX:1;
    gctUINT32 IMGLD_WIDTH_LT16_FIX:1;
    gctUINT32 TX_FILTER_ROUND_FIX:1;
    gctUINT32 SH_FP32_FMA_SUPPORT:1;
    gctUINT32 TX_ETC2_COMPRESSION:1;
    gctUINT32 VG_TS_CULLING:1;
    gctUINT32 VG_FP25:1;
    gctUINT32 VG_AYUV_INPUT_OUTPUT:1;
    gctUINT32 VG_DOUBLE_IMAGE:1;
    gctUINT32 VG_RECTANGLE_STRIPE_MODE:1;
    gctUINT32 VG_MMU:1;
    gctUINT32 VG_IM_FILTER:1;
    gctUINT32 VG_IM_YUV_PACKET:1;
    gctUINT32 VG_IM_YUV_PLANAR:1;
    gctUINT32 VG_PE_YUV_PACKET:1;
    gctUINT32 VG_COLOR_PRECISION_8_BIT:1;
    gctUINT32 VG_RESOLVE_ENGINE:1;
    gctUINT32 VG_PE_COLOR_KEY:1;
    gctUINT32 VG_IM_INDEX_FORMAT:1;
    gctUINT32 VG_RESOLUTION_8K:1;
    gctUINT32 VG_IMAGE_16K:1;
    gctUINT32 VG_FORMAT_ARGB2222:1;
    gctUINT32 G2D_DEC400:1;
    gctUINT32 DC_OVERLAY_SCALING:1;
    gctUINT32 DC_SOURCE_ROTATION:1;
    gctUINT32 DC_TILED:1;
    gctUINT32 DC_YUV_L1:1;
    gctUINT32 DC_D30_OUTPUT:1;
    gctUINT32 DC_MMU:1;
    gctUINT32 DC_COMPRESSION:1;
    gctUINT32 DC_QOS:1;
    gctUINT32 VIP_HW_FINAL_RELEASE:1;
    gctUINT32 NN_SINGLEPORT_ACCUMBUFFER:1;
    gctUINT32 NN_STRIDE_SUPPORT:1;
    gctUINT32 SWTILING_PHASE1:1;
    gctUINT32 SWTILING_PHASE2:1;
    gctUINT32 TP_SIMPLE_INT16:1;
    gctUINT32 TP_REAL_INT16:1;
    gctUINT32 TP_ROI_POOLING:1;
    gctUINT32 TP_MAX_POOLING_STRIDE1:1;
    gctUINT32 TP_LRN:1;
    gctUINT32 TP_REORDER:1;
    gctUINT32 TF_QUANTIZATION:1;
    gctUINT32 NN_NONZERO_BORDER:1;
    gctUINT32 NN_MIRROR_BORDER:1;
    gctUINT32 AI_GPU:1;
    gctUINT32 EVIS_NO_ABSDIFF:1;
    gctUINT32 EVIS_NO_BITREPLACE:1;
    gctUINT32 EVIS_NO_BOXFILTER:1;
    gctUINT32 EVIS_NO_CORDIAC:1;
    gctUINT32 EVIS_NO_DP32:1;
    gctUINT32 EVIS_NO_FILTER:1;
    gctUINT32 EVIS_NO_IADD:1;
    gctUINT32 EVIS_NO_SELECTADD:1;
    gctUINT32 EVIS_LERP_7OUTPUT:1;
    gctUINT32 EVIS_ACCSQ_8OUTPUT:1;
    gctUINT32 EVIS_VX2:1;
    gctUINT32 TP_ENGINE:1;
    gctUINT32 VIP_V7:1;
    gctUINT32 TP_TENSOR_ADD_MUL:1;
    gctUINT32 NN_DEPTHWISE_INT16XINT8:1;
    gctUINT32 NN_DEPTHWISE_8BIT_VIP_V7:1;
    gctUINT32 TP_SOFTMAX:1;
    gctUINT32 NN_23BITS_POST_MULTIPLIER_VIP_V7:1;
    gctUINT32 TP_23BITS_POST_MULTIPLIER_VIP_V7:1;
    gctUINT32 CONV_INT16X8BIT_VIP_V7:1;
    gctUINT32 NN_REMOVE_POOLING:1;
    gctUINT32 NN_40BIT_BIAS:1;
    gctUINT32 TP_REMOVE_USC:1;
    gctUINT32 NN_ZDP6:1;
    gctUINT32 NN_XYDP9:1;
    gctUINT32 NN_FIRST_PIXEL_POOLING:1;
    gctUINT32 NN_ZDP3:1;
    gctUINT32 NN_XYDP6:1;
    gctUINT32 SWTILING_PHASE3:1;
    gctUINT32 MCFE:1;
    gctUINT32 USC_STAY_LRU:1;
    gctUINT32 COEF_COMPRESSION_ENHANCEMENT:1;
    gctUINT32 TP_COEF_COMPRESSION_ENHANCEMENT:1;
    gctUINT32 NN_COEF_DECOMPRESS_PERF2X:1;
    gctUINT32 TP_SMALLBATCH_PHASE1:1;
    gctUINT32 OCB_COUNTER:1;
    gctUINT32 SCALER:1;
    gctUINT32 SCALER_4K:1;
    gctUINT32 INPUT_4BIT:1;
    gctUINT32 NN_NO_Z_LOCATION_OFFSET:1;
    gctUINT32 OCB_REMAP_PHYSICAL_ADDRESS:1;
    gctUINT32 NN_SLOW_OUTPUT:1;
    gctUINT32 NO_NARROW_POST_PROCESS_PIPE:1;
    gctUINT32 TP_NN_PROBE:1;
    gctUINT32 NN_DEPTHWISE_SUPPORT:1;
    gctUINT32 NN_XYDP0:1;
    gctUINT32 NN_WRITE_WITHOUT_USC:1;
    gctUINT32 NN_HW_LIMITATION_NATIVE_KER_1x2_2x1:1;
    gctUINT32 NN_SMALLBATCH_PHASE1:1;
    gctUINT32 NN_SLICE_PADDING_TO_64BYTE_ALIGN:1;
    gctUINT32 NN_DW_1x1_CONV_MERGE:1;
    gctUINT32 TP_BFLOAT16:1;
    gctUINT32 TP_23BITS_POST_MULTIPLIER:1;
    gctUINT32 NN_TRANSPOSE:1;
    gctUINT32 NN_ZDP_TRANSPOSE_CH9_ONLY:1;
    gctUINT32 USE_SINGLE_PORT_VIPSRAM:1;
    gctUINT32 NN_LEAKY_RELU:1;
    gctUINT32 NN_PRELU:1;
    gctUINT32 NN_PER_CHANNEL_QUANT:1;
    gctUINT32 NN_PER_CHANNEL_QUANT_ASYM:1;
    gctUINT32 NN_ASYMMETRIC_INT8:1;
    gctUINT32 NN_FLOAT_POST_MULT:1;
    gctUINT32 PRELU_LEAKLY_RELU_CLAMP:1;
    gctUINT32 TPLITE_BFLOAT16:1;
    gctUINT32 PREPROCESS_IMG_BUF_640BYTE_LIMIT:1;
    gctUINT32 NN_POST_OUT_SUPPORT_FP16:1;
    gctUINT32 NN_POST_OUT_SUPPORT_BF16:1;
    gctUINT32 NN_POST_OUT_SUPPORT_FP32:1;
    gctUINT32 TP_KERNEL_1BYTE_ALGIN:1;
    gctUINT32 BFLOAT_COEF_COMPRESSION_ZERO_COEFBIT14_INVERSE:1;
    gctUINT32 NN_COMPRESSION_BYPASSS:1;
    gctUINT32 TP_3_USC:1;
    gctUINT32 BFP_COEF_AUTO_PAD_INCOMPLETE_ZERO_IN_KZ_PLANE:1;
    gctUINT32 NN_NATIVE_STRIDE_TWO:1;
    gctUINT32 NN_TENSOR_ADD:1;
    gctUINT32 NN_FLOAT32_IO:1;
    gctUINT32 TP_FLOAT32_IO:1;
    gctUINT32 NN_SMALL_BATCH_PHASE2:1;
    gctUINT32 TILE_ACCESS_CAPABILITY:1;
    gctUINT32 FAST_DP3_PREPROCESSOR:1;
    gctUINT32 DEPTHWISE_SUPPORT_16BIT_FORMAT:1;
    gctUINT32 NN_SUPPORT_ALU:1;
    gctUINT32 NN_ENHANCED_MAX_POOLING:1;
    gctUINT32 NN_TRANSPOSE_PHASE2:1;
    gctUINT32 NN_TENSOR_ADD_FIELD_MOVE_TO_EXT_CMD:1;
    gctUINT32 NN_CONV_CORE_BYPASS:1;
    gctUINT32 NN_TENSOR_ADD_RELU:1;
    gctUINT32 TPLITE_SUPPORT_TP_DATA_TRANSPOSE:1;
    gctUINT32 NN_SUPPORT_CONV_1D:1;
    gctUINT32 USE_VIPSRAM_FOR_KERNEL_STREAMING:1;
    gctUINT32 NN_SUPPORT_DUMMY_TILE:1;
    gctUINT32 NN_SUPPORT_KERNEL_1BYTE_ALIGN:1;
    gctUINT32 NN_1x1_NON_POOLING_PACKING:1;
    gctUINT32 NN_SUPPORT_BOTH_CONV_NATIVE_STRIDE2_AND_POOLING:1;
    gctUINT32 NN_SUPPORT_CONV1x1_AND_NATIVE_CONV_STRIDE2:1;
    gctUINT32 TP_REMOVE_FC:1;
    gctUINT32 VIP_REMOVE_MMU:1;
    gctUINT32 NN_MP_INTER_CONNECT_RING:1;
    gctUINT32 NN_SUPPORT_BATCH:1;
    gctUINT32 NN_2D_AVERAGE_OUTPUT:1;
    gctUINT32 NN_JOB_CANCELATION:1;
    gctUINT32 NN_DISTRIBUTED_VIPSRAM:1;
    gctUINT32 NN_FC_ENHANCEMENT:1;
    gctUINT32 VIP_DEC400:1;
    gctUINT32 NN_PER3DTILE_BUBBLE_FIX:1;
    gctUINT32 NN_CACHELINE_MODE_PERF_FIX:1;
    gctUINT32 NN_CONV1x1_PERF_FIX:1;
    gctUINT32 TP_REORDER_FIX:1;
    gctUINT32 NN_CONVOUT_FIFO_DEPTH_FIX:1;
    gctUINT32 NN_ZXDP3_KERNEL_READ_CONFLICT_FIX:1;
    gctUINT32 NN_ZDP3_NO_COMPRESS_FIX:1;
    gctUINT32 NN_ASYNC_COPY_PERF_FIX:1;
    gctUINT32 HI_REORDER_FIX:1;
    gctUINT32 INCORRECT_WR_REQ_TO_USC_BETWEEN_REORDER_AND_NORMAL_LAYER_FIX:1;
    gctUINT32 TP_REORDER_LAYER_SUSPEND_FIX:1;
    gctUINT32 NN_ASYNC_COPY_MERGE_FIX:1;
    gctUINT32 USC_INVALIDATE_CACHE_LINE_FIX:1;
    gctUINT32 NN_REQ_SLOWARBITRATION_FIX:1;
    gctUINT32 IMAGE_PARTIAL_CACHE_FIX:1;
    gctUINT32 FULLCACHE_KERNELHEAD_FIX:1;
    gctUINT32 NN_ZDP_INIMAGE_SIZE_FIX:1;
    gctUINT32 IDLE_BEFORE_FLUSH_COMPLETE_FIX:1;
    gctUINT32 NO_FLUSH_USC_FIX:1;
    gctUINT32 SMALL_BATCH_FLOPS_RESET_FIX:1;
    gctUINT32 SMALL_BATCH_DISBLE_FIX:1;
    gctUINT32 OUTPUT_CONVERT_UINT8_INT8_TO_UINT16_INT16_FIX:1;
    gctUINT32 IMAGE_NOT_PACKED_IN_SRAM_FIX:1;
    gctUINT32 COEF_DELTA_CORD_OVERFLOW_ZRL_8BIT_FIX:1;
    gctUINT32 USC_INDIVIDUAL_PORT_WRT_EARLY_EVICT_DATA_CORRUPT_FIX:1;
    gctUINT32 LOW_EFFICIENCY_OF_ID_WRITE_IMGBUF_FIX:1;
    gctUINT32 KERNEL_VIP_SRAM_READ_BW_LIMITATION_FIX:1;
    gctUINT32 USC_BOTTLENECK_FIX:1;
    gctUINT32 KERNEL_PER_CORE_LESS_THAN_THIRD_COEF_BUFF_DEPTH_FIX:1;
    gctUINT32 NN_TILE_NUM_BIGGER_THAN_1024_FIX:1;
    gctUINT32 KERNEL_SIZE_WASTE_IN_PARTIAL_MODE_FIX:1;
    gctUINT32 NN_COMMAND_KERNEL_REQUEST_CONFICT_FIX:1;
    gctUINT32 TP_REORDER_INTILE_X_SIZE_512_FIX:1;
    gctUINT32 IMG_POP_PIPELINE_PAUSE_FIX:1;
    gctUINT32 FULLCACHE_KERNEL_INTERLEAVE_FIX:1;
    gctUINT32 V8_SINGLE_PORT_ACCUMULATION_BUFFER_RW_CONFICT_ZERO_SKIP_PERF_FIX:1;
    gctUINT32 V8_ACCUMLATION_READ_OUT_HAS_BUBBLES_PERF_FIX:1;
    gctUINT32 DEPTHWISE_NEIGHBOR_IMG_DATA_TRANSFER_NOT_EFFICIENT_FIX:1;
    gctUINT32 DR_JD_DIFF_CONDITION_FOR_CACHELINE_MODE_PRE_FIX:1;
    gctUINT32 TP_ACCESS_VIPSRAM_OT_IS_ONE_FIX:1;
    gctUINT32 EVIS2_FLOP_RESET_FIX:1;
    gctUINT32 OUTIMAGE_X_BITWIDTH_LIMIT_FOR_NN_TRANSPOSE_FIX:1;
    gctUINT32 USC_ASYNC_CP_RTN_FLOP_RESET_FIX:1;
    gctUINT32 IMG_ADDR_NOT_WRAP_IF_OVER_OCB_ADDR_FIX:1;
    gctUINT32 NEGATIVE_POST_SHIFT_FIX:1;
    gctUINT32 INIMAGE_2DTILE_NOT_LESS_160PIXEL_FIX:1;
    gctUINT32 IMG_CAHCE_MODE_MUST_0_IN_IMG_DIRECT_MODE_FIX:1;
    gctUINT32 BURST_COLLECT_DUMMY_DATA_WASTE_CYCLES_FIX:1;
    gctUINT32 INIMG_NOT_64BYTE_ALIGN_CACHELINE_MODE_FIX:1;
    gctUINT32 TP_FC_FLOAT_LAST_PIXEL_NEGATIVE_0_FIX:1;
    gctUINT32 NN_WASTE_COEF_READ_WRITE_BANDWIDTH_128BYTE_VIPSRAM_IN_FULL_PATIAL_CACHE_MODE_FIX:1;
    gctUINT32 NN_IN_TILE_DATA_IS_ALL_PAD_FIX:1;
    gctUINT32 NN_TP_INSTR_COMPLETE_IN_SAME_CYCLE_WITH_WAIT_EVENT_FIX:1;
    gctUINT32 CORE_IMAGE_TRANSER_NOT_EFFICIENT_BETWEEN_PARTITION_FIX:1;
    gctUINT32 TP_FC_KERNEL_STREAM_MUST_LESS_THAN_OR_EQUAL_TO_64BYTE_WHEN_1BYTE_ALGINE_FIX:1;
    gctUINT32 NN_KERNEL_1x1_NO_PAD_FIX:1;
    gctUINT32 NN_DEPTHWISE_AFTER_16BIT_LAYER_LIMIT_FIX:1;
    gctUINT32 TP_NOT_FULL_USE_CACHE_LINE_FIX:1;
    gctUINT32 SH_MOVAI_MOVAR_UNUSED_COMPONENTS_WRITE_DIRTY_DATA_FIX:1;
    gctUINT32 BURST_COLLECT_CONSUMES_MC_DATA_WIDTH_PER_CYCLE_FIX:1;
    gctUINT32 TP_ASSYM_INT8_FIX:1;
    gctUINT32 NN_PAD_SLICE_ERROR_WHEN_TRANSPSE_FIX:1;
    gctUINT32 NN_2ND_IMG_BASE_ADDR_FIX:1;
    gctUINT32 NN_TP_SYSTEM_FIX:1;
    gctUINT32 NN_INTILE_YSIZE_128_LIMIT_FIX:1;
    gctUINT32 SH_CLOCK_GATOR_IDLE_CONDITON_FIX:1;
    gctUINT32 NN_BURST_COLLECTER_LAST_FLAG_FIX:1;
    gctUINT32 NN_2ND_IMG_SMALL_3D_TILE_FIX:1;
    gctUINT32 NN_TILE_YSIZE_127_LIMITATION_FIX:1;
    gctUINT32 NN_CONV_1D_16BIT_FORMAT_INTILE_SIZE_LIMITATION_FIX:1;
    gctUINT32 NN_VIPSRAM_DOUBLE_BUFFER_FIX:1;
    gctUINT32 NN_JD_DIRECT_MODE_FIX:1;
    gctUINT32 NN_KERNEL_DIRECT_WRONG_PUSH_FIX:1;
    gctUINT32 HI_DEFAULT_ENABLE_REORDER_FIX:1;
    gctUINT32 V83_INTILESIZE_1X1_10BITS_FIX:1;
    gctUINT32 DEPTHWISE_FLOAT_FIX:1;
    gctUINT32 TP_CIRCULAR_BUF_WRAP_ADDRESS_OVERFLOW_FIX:1;
    gctUINT32 NN_CIRCULAR_BUF_WRAP_ADDRESS_OVERFLOW_FIX:1;
    gctUINT32 CLOCK_DIV2_FREQ_CHANGE_FIX:1;
    gctUINT32 SMALL_TILE_TENSOR_ADD_FIX:1;
    gctUINT32 DECOMPRESSOR_DEPTHWISE_FLOAT_FIX:1;
    gctUINT32 NN_INTERLEVE8:1;
    gctUINT32 NN_FP16_ALU:1;
    gctUINT32 NN_INT16_ALU:1;
    gctUINT32 NN_INT8_SCALE:1;
    gctUINT32 NN_POWER_ISOLATION:1;
    gctUINT32 ZRL_7BIT:1;
    gctUINT32 NN_SMALLBATCH:1;
    gctUINT32 TP_SMALLBATCH:1;
    gctUINT32 ZRL_8BIT:1;
    gctUINT32 DDR_BURST_LEN_256B:1;
    gctUINT32 XY_OFFSET_LIMITATION_FIX:1;
    gctUINT32 NN_NONZERO_MIRROR_BORDER:1;
    gctUINT32 IMAGE_PARTIAL_CACHE:1;
} gcsFEATURE_DATABASE;

#define FEATURE_BIT_START 93
#define FEATURE_BIT_END 742
static gcsFEATURE_DATABASE gChipInfo[] = {
    /* gc620s_5_5_5_rc2p */
    {
        0x620, /* ChipID */
        0x5552, /* ChipRevision */
        0x6200, /* ProductID */
        0x0, /* EcoID */
        0x20b, /* CustomerID */
        0x0, /* PatchVersion */
        "", /* ProductName */
        0x1, /* FormalRelease */
        0x40, /* gcFEATURE_VALUE_TempRegisters */
        0x100, /* gcFEATURE_VALUE_ThreadCount */
        0x1, /* gcFEATURE_VALUE_NumShaderCores */
        0x100, /* gcFEATURE_VALUE_InstructionCount */
        0xa8, /* gcFEATURE_VALUE_NumberOfConstants */
        0x1, /* gcFEATURE_VALUE_CoreCount */
        0x0, /* gcFEATURE_VALUE_LocalStorageSize */
        0x0, /* gcFEATURE_VALUE_LocalStorageSize_1 */
        0x0, /* gcFEATURE_VALUE_LocalStorageSize_2 */
        0x0, /* gcFEATURE_VALUE_L1CacheSize */
        0x0, /* gcFEATURE_VALUE_L1CacheSize_1 */
        0x0, /* gcFEATURE_VALUE_L1CacheSize_2 */
        0x0, /* gcFEATURE_VALUE_InstructionMemorySize */
        0x0, /* gcFEATURE_VALUE_ShaderPCLength */
        0x0, /* gcFEATURE_VALUE_USC_MAX_PAGES */
        0x0, /* gcFEATURE_VALUE_USC_MAX_PAGES_1 */
        0x0, /* gcFEATURE_VALUE_USC_MAX_PAGES_2 */
        0x1, /* gcFEATURE_VALUE_NumPixelPipes */
        0x0, /* gcFEATURE_VALUE_USC_CACHE_CONTROLLERS */
        0x0, /* gcFEATURE_VALUE_USC_CACHE_CONTROLLERS_1 */
        0x0, /* gcFEATURE_VALUE_USC_CACHE_CONTROLLERS_2 */
        0x0, /* gcFEATURE_VALUE_USC_BANKS */
        0x0, /* gcFEATURE_VALUE_USC_BANKS_1 */
        0x0, /* gcFEATURE_VALUE_USC_BANKS_2 */
        0x1, /* gcFEATURE_VALUE_Streams */
        0x8, /* gcFEATURE_VALUE_VaryingCount */
        0x200, /* gcFEATURE_VALUE_VertexOutputBufferSize */
        0x0, /* gcFEATURE_VALUE_BufferSize */
        0x8, /* gcFEATURE_VALUE_VertexCacheSize */
        0x1, /* gcFEATURE_VALUE_NumResolvePipes */
        0x0, /* gcFEATURE_VALUE_RESULT_WINDOW_MAX_SIZE */
        0x0, /* gcFEATURE_VALUE_ClusterAliveMask */
        0x3, /* gcFEATURE_VALUE_G2D_DEC400_MINOR */
        0x0, /* gcFEATURE_VALUE_PS_INSTRUCTION_COUNT */
        0x0, /* gcFEATURE_VALUE_NNMadPerCore */
        0x0, /* gcFEATURE_VALUE_NNCoreCount */
        0x0, /* gcFEATURE_VALUE_NN_ACTIVE_CORE_COUNT */
        0x0, /* gcFEATURE_VALUE_NNCoreCount_INT8 */
        0x0, /* gcFEATURE_VALUE_NNCoreCount_INT16 */
        0x0, /* gcFEATURE_VALUE_NNCoreCount_FLOAT16 */
        0x0, /* gcFEATURE_VALUE_NNCoreCount_BFLOAT */
        0x0, /* gcFEATURE_VALUE_NNInputBufferDepth */
        0x0, /* gcFEATURE_VALUE_NNAccumBufferDepth */
        0x0, /* gcFEATURE_VALUE_TPEngine_PwlLUTCount */
        0x0, /* gcFEATURE_VALUE_TPEngine_PwlLUTSize */
        0x0, /* gcFEATURE_VALUE_VIP_SRAM_SIZE */
        0x0, /* gcFEATURE_VALUE_TPEngine_CoreCount */
        0x0, /* gcFEATURE_VALUE_AXI_SRAM_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_INIMAGE_OFFSET_BITS */
        0x0, /* gcFEATURE_VALUE_TP_REORDER_INIMAGE_SIZE */
        0x0, /* gcFEATURE_VALUE_TPLite_CoreCount */
        0x0, /* gcFEATURE_VALUE_NN_PREPROCESSOR_MAX_SEGMENT_PER_CYCLE */
        0x0, /* gcFEATURE_VALUE_NNFP16_XYDP_X */
        0x0, /* gcFEATURE_VALUE_NNFP16_XYDP_Y */
        0x0, /* gcFEATURE_VALUE_NNFP16_ZDP */
        0x0, /* gcFEATURE_VALUE_NN_LANES_PER_OUT_CYCLE */
        0x0, /* gcFEATURE_VALUE_MAX_OT_NUMBER */
        0x0, /* gcFEATURE_VALUE_PHYSICAL_VIP_SRAM_WIDTH_IN_BYTE */
        0x0, /* gcFEATURE_VALUE_EQUIVALENT_VIP_SRAM_WIDTH_INBYTE */
        0x0, /* gcFEATURE_VALUE_TP_ZRL_BITS */
        0x0, /* gcFEATURE_VALUE_NN_ZRL_BITS */
        0x0, /* gcFEATURE_VALUE_LATENCY_HIDING_AT_FULL_AXI_BW */
        0x0, /* gcFEATURE_VALUE_AXI_BUS_WIDTH */
        0x0, /* gcFEATURE_VALUE_NN_KERNEL_X_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_KERNEL_Y_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_FC_KERNEL_Y_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_KERNEL_Z_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_X_OFFSET */
        0x0, /* gcFEATURE_VALUE_NN_Y_OFFSET */
        0x0, /* gcFEATURE_VALUE_DDR_KERNEL_BURST_SIZE */
        0x0, /* gcFEATURE_VALUE_OUTIMAGE_X_STRIDE_BITS */
        0x0, /* gcFEATURE_VALUE_OUTIMAGE_Y_STRIDE_BITS */
        0x0, /* gcFEATURE_VALUE_INIMAGE_X_STRIDE_BITS */
        0x0, /* gcFEATURE_VALUE_IMIMAGE_Y_STRIDE_BITS */
        0x0, /* gcFEATURE_VALUE_OUTIMAGE_X_SIZE_BITS */
        0x0, /* gcFEATURE_VALUE_OUTIMAGE_Y_SIZE_BITS */
        0x0, /* gcFEATURE_VALUE_OUTIMAGE_Z_SIZE_BITS */
        0x0, /* gcFEATURE_VALUE_INIMAGE_X_SIZE_BITS */
        0x0, /* gcFEATURE_VALUE_INIMAGE_Y_SIZE_BITS */
        0x0, /* gcFEATURE_VALUE_MAX_TILE_X_SIZE */
        0x0, /* gcFEATURE_VALUE_NN_CLUSTER_NUM_FOR_POWER_CONTROL */
        0x0, /* gcFEATURE_VALUE_NN_IN_LINES_PER_CYCLE */
        0x0, /* gcFEATURE_VALUE_VIP_CLUSTER_COUNT */
        0x0, /* gcFEATURE_VALUE_NN_MP_INTER_CONNECT_RING_COUNT */
        0x0, /* gcFEATURE_BIT_REG_FastClear */
        0x0, /* gcFEATURE_BIT_REG_SpecialAntiAliasing */
        0x0, /* gcFEATURE_BIT_REG_Pipe3D */
        0x0, /* gcFEATURE_BIT_REG_DXTTextureCompression */
        0x0, /* gcFEATURE_BIT_REG_DebugMode */
        0x0, /* gcFEATURE_BIT_REG_ZCompression */
        0x1, /* gcFEATURE_BIT_REG_YUV420Filter */
        0x0, /* gcFEATURE_BIT_REG_MSAA */
        0x0, /* gcFEATURE_BIT_REG_DC */
        0x1, /* gcFEATURE_BIT_REG_Pipe2D */
        0x0, /* gcFEATURE_BIT_REG_ETC1TextureCompression */
        0x1, /* gcFEATURE_BIT_REG_FastScaler */
        0x0, /* gcFEATURE_BIT_REG_HighDynamicRange */
        0x0, /* gcFEATURE_BIT_REG_YUV420Tiler */
        0x1, /* gcFEATURE_BIT_REG_ModuleCG */
        0x0, /* gcFEATURE_BIT_REG_MinArea */
        0x1, /* gcFEATURE_BIT_REG_NoEZ */
        0x1, /* gcFEATURE_BIT_REG_No422Texture */
        0x0, /* gcFEATURE_BIT_REG_BufferInterleaving */
        0x1, /* gcFEATURE_BIT_REG_ByteWrite2D */
        0x1, /* gcFEATURE_BIT_REG_NoScaler */
        0x0, /* gcFEATURE_BIT_REG_YUY2Averaging */
        0x0, /* gcFEATURE_BIT_REG_HalfPECache */
        0x0, /* gcFEATURE_BIT_REG_HalfTXCache */
        0x0, /* gcFEATURE_BIT_REG_YUY2RenderTarget */
        0x0, /* gcFEATURE_BIT_REG_Mem32BitSupport */
        0x0, /* gcFEATURE_BIT_REG_PipeVG */
        0x0, /* gcFEATURE_BIT_REG_VGTS */
        0x0, /* gcFEATURE_BIT_REG_FE20 */
        0x0, /* gcFEATURE_BIT_REG_ByteWrite3D */
        0x0, /* gcFEATURE_BIT_REG_RsYuvTarget */
        0x0, /* gcFEATURE_BIT_REG_FE20BitIndex */
        0x0, /* gcFEATURE_BIT_REG_FlipY */
        0x0, /* gcFEATURE_BIT_REG_DualReturnBus */
        0x0, /* gcFEATURE_BIT_REG_EndiannessConfig */
        0x0, /* gcFEATURE_BIT_REG_Texture8K */
        0x0, /* gcFEATURE_BIT_REG_CorrectTextureConverter */
        0x0, /* gcFEATURE_BIT_REG_SpecialMsaaLod */
        0x0, /* gcFEATURE_BIT_REG_FastClearFlush */
        0x1, /* gcFEATURE_BIT_REG_2DPE20 */
        0x0, /* gcFEATURE_BIT_REG_CorrectAutoDisable */
        0x0, /* gcFEATURE_BIT_REG_Render8K */
        0x0, /* gcFEATURE_BIT_REG_TileStatus2Bits */
        0x0, /* gcFEATURE_BIT_REG_SeparateTileStatusWhenInterleaved */
        0x0, /* gcFEATURE_BIT_REG_SuperTiled32x32 */
        0x0, /* gcFEATURE_BIT_REG_VG20 */
        0x0, /* gcFEATURE_BIT_REG_TSExtendedCommands */
        0x0, /* gcFEATURE_BIT_REG_CompressionFifoFixed */
        0x0, /* gcFEATURE_BIT_REG_ExtraShaderInstructions0 */
        0x0, /* gcFEATURE_BIT_REG_VGFilter */
        0x0, /* gcFEATURE_BIT_REG_VG21 */
        0x0, /* gcFEATURE_BIT_REG_ShaderGetsW */
        0x0, /* gcFEATURE_BIT_REG_ExtraShaderInstructions1 */
        0x1, /* gcFEATURE_BIT_REG_DefaultReg0 */
        0x1, /* gcFEATURE_BIT_REG_MC20 */
        0x0, /* gcFEATURE_BIT_REG_ShaderMSAASideband */
        0x0, /* gcFEATURE_BIT_REG_BugFixes0 */
        0x0, /* gcFEATURE_BIT_REG_VAA */
        0x0, /* gcFEATURE_BIT_REG_BypassInMSAA */
        0x0, /* gcFEATURE_BIT_REG_HierarchicalZ */
        0x0, /* gcFEATURE_BIT_REG_NewTexture */
        0x1, /* gcFEATURE_BIT_REG_A8TargetSupport */
        0x0, /* gcFEATURE_BIT_REG_CorrectStencil */
        0x1, /* gcFEATURE_BIT_REG_EnhanceVR */
        0x0, /* gcFEATURE_BIT_REG_RSUVSwizzle */
        0x0, /* gcFEATURE_BIT_REG_V2Compression */
        0x0, /* gcFEATURE_BIT_REG_VGDoubleBuffer */
        0x0, /* gcFEATURE_BIT_REG_BugFixes1 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes2 */
        0x0, /* gcFEATURE_BIT_REG_TextureStride */
        0x0, /* gcFEATURE_BIT_REG_BugFixes3 */
        0x0, /* gcFEATURE_BIT_REG_CorrectAutoDisable1 */
        0x0, /* gcFEATURE_BIT_REG_AutoRestartTS */
        0x0, /* gcFEATURE_BIT_REG_BugFixes4 */
        0x0, /* gcFEATURE_BIT_REG_L2Windowing */
        0x0, /* gcFEATURE_BIT_REG_HalfFloatPipe */
        0x0, /* gcFEATURE_BIT_REG_PixelDither */
        0x0, /* gcFEATURE_BIT_REG_TwoStencilReference */
        0x0, /* gcFEATURE_BIT_REG_ExtendedPixelFormat */
        0x0, /* gcFEATURE_BIT_REG_CorrectMinMaxDepth */
        0x1, /* gcFEATURE_BIT_REG_DitherAndFilterPlusAlpha2D */
        0x0, /* gcFEATURE_BIT_REG_BugFixes5 */
        0x1, /* gcFEATURE_BIT_REG_New2D */
        0x0, /* gcFEATURE_BIT_REG_NewFloatingPointArithmetic */
        0x0, /* gcFEATURE_BIT_REG_TextureHorizontalAlignmentSelect */
        0x0, /* gcFEATURE_BIT_REG_NonPowerOfTwo */
        0x0, /* gcFEATURE_BIT_REG_LinearTextureSupport */
        0x0, /* gcFEATURE_BIT_REG_Halti0 */
        0x0, /* gcFEATURE_BIT_REG_CorrectOverflowVG */
        0x0, /* gcFEATURE_BIT_REG_NegativeLogFix */
        0x0, /* gcFEATURE_BIT_REG_ResolveOffset */
        0x1, /* gcFEATURE_BIT_REG_OkToGateAxiClock */
        0x1, /* gcFEATURE_BIT_REG_MMU */
        0x0, /* gcFEATURE_BIT_REG_WideLine */
        0x0, /* gcFEATURE_BIT_REG_BugFixes6 */
        0x0, /* gcFEATURE_BIT_REG_FcFlushStall */
        0x0, /* gcFEATURE_BIT_REG_LineLoop */
        0x0, /* gcFEATURE_BIT_REG_LogicOp */
        0x0, /* gcFEATURE_BIT_REG_SeamlessCubeMap */
        0x0, /* gcFEATURE_BIT_REG_SuperTiledTexture */
        0x0, /* gcFEATURE_BIT_REG_LinearPE */
        0x0, /* gcFEATURE_BIT_REG_RectPrimitive */
        0x0, /* gcFEATURE_BIT_REG_Composition */
        0x0, /* gcFEATURE_BIT_REG_CorrectAutoDisableCountWidth */
        0x0, /* gcFEATURE_BIT_REG_PESwizzle */
        0x0, /* gcFEATURE_BIT_REG_EndEvent */
        0x0, /* gcFEATURE_BIT_REG_S1S8 */
        0x0, /* gcFEATURE_BIT_REG_Halti1 */
        0x0, /* gcFEATURE_BIT_REG_RGB888 */
        0x0, /* gcFEATURE_BIT_REG_TX_YUVAssembler */
        0x0, /* gcFEATURE_BIT_REG_DynamicFrequencyScaling */
        0x0, /* gcFEATURE_BIT_REG_TXFilter */
        0x1, /* gcFEATURE_BIT_REG_FullDirectFB */
        0x1, /* gcFEATURE_BIT_REG_OnePass2DFilter */
        0x0, /* gcFEATURE_BIT_REG_ThreadWalkerInPS */
        0x0, /* gcFEATURE_BIT_REG_TileFiller */
        0x1, /* gcFEATURE_BIT_REG_YUVStandard */
        0x1, /* gcFEATURE_BIT_REG_MultiSourceBlt */
        0x1, /* gcFEATURE_BIT_REG_YUVConversion */
        0x1, /* gcFEATURE_BIT_REG_FlushFixed2D */
        0x0, /* gcFEATURE_BIT_REG_Interleaver */
        0x0, /* gcFEATURE_BIT_REG_MixedStreams */
        0x1, /* gcFEATURE_BIT_REG_L2CacheFor2D420 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes7 */
        0x0, /* gcFEATURE_BIT_REG_NoIndexPattern */
        0x0, /* gcFEATURE_BIT_REG_TextureTileStatus */
        0x0, /* gcFEATURE_BIT_REG_DecompressZ16 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes8 */
        0x1, /* gcFEATURE_BIT_REG_DERotationStallFix */
        0x0, /* gcFEATURE_BIT_REG_OclOnly */
        0x1, /* gcFEATURE_BIT_REG_NewFeatures0 */
        0x0, /* gcFEATURE_BIT_REG_InstructionCache */
        0x0, /* gcFEATURE_BIT_REG_GeometryShader */
        0x0, /* gcFEATURE_BIT_REG_TexCompressionSupertiled */
        0x0, /* gcFEATURE_BIT_REG_Generics */
        0x0, /* gcFEATURE_BIT_REG_BugFixes9 */
        0x0, /* gcFEATURE_BIT_REG_FastMSAA */
        0x0, /* gcFEATURE_BIT_REG_WClip */
        0x0, /* gcFEATURE_BIT_REG_BugFixes10 */
        0x0, /* gcFEATURE_BIT_REG_UnifiedSamplers */
        0x0, /* gcFEATURE_BIT_REG_BugFixes11 */
        0x0, /* gcFEATURE_BIT_REG_PerformanceCounters */
        0x0, /* gcFEATURE_BIT_REG_ExtraShaderInstructions2 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes12 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes13 */
        0x1, /* gcFEATURE_BIT_REG_DEEnhancements1 */
        0x1, /* gcFEATURE_BIT_REG_ACE */
        0x0, /* gcFEATURE_BIT_REG_TXEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_PEEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_DEEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes14 */
        0x0, /* gcFEATURE_BIT_REG_PowerOptimizations0 */
        0x0, /* gcFEATURE_BIT_REG_NewHZ */
        0x0, /* gcFEATURE_BIT_REG_BugFixes15 */
        0x0, /* gcFEATURE_BIT_REG_DEEnhancements3 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements3 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements4 */
        0x0, /* gcFEATURE_BIT_REG_TXEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_FEEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_PEEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_PAEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_DENoGamma */
        0x0, /* gcFEATURE_BIT_REG_PAEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_DEEnhancements4 */
        0x0, /* gcFEATURE_BIT_REG_PEEnhancements3 */
        0x0, /* gcFEATURE_BIT_REG_HIEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_TXEnhancements3 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements5 */
        0x0, /* gcFEATURE_BIT_REG_FEEnhancements2 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes16 */
        0x1, /* gcFEATURE_BIT_REG_DEEnhancements5 */
        0x0, /* gcFEATURE_BIT_REG_TXEnhancements4 */
        0x0, /* gcFEATURE_BIT_REG_PEEnhancements4 */
        0x1, /* gcFEATURE_BIT_REG_MCEnhancements1 */
        0x0, /* gcFEATURE_BIT_REG_Halti2 */
        0x1, /* gcFEATURE_BIT_REG_DEMirrorRotate */
        0x0, /* gcFEATURE_BIT_REG_SmallMSAA */
        0x0, /* gcFEATURE_BIT_REG_BugFixes17 */
        0x0, /* gcFEATURE_BIT_REG_Rasterizer2 */
        0x1, /* gcFEATURE_BIT_REG_DualPipeOPF */
        0x1, /* gcFEATURE_BIT_REG_MultiSrcV2 */
        0x0, /* gcFEATURE_BIT_REG_CSCV2 */
        0x0, /* gcFEATURE_BIT_REG_PAEnhancements3 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes18 */
        0x0, /* gcFEATURE_BIT_REG_Compression2D */
        0x0, /* gcFEATURE_BIT_REG_Probe */
        0x0, /* gcFEATURE_BIT_REG_MediumPrecision */
        0x1, /* gcFEATURE_BIT_REG_DESupertile */
        0x0, /* gcFEATURE_BIT_REG_BugFixes19 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements6 */
        0x0, /* gcFEATURE_BIT_REG_SHEnhancements7 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes20 */
        0x1, /* gcFEATURE_BIT_REG_DEAddress40 */
        0x1, /* gcFEATURE_BIT_REG_MiniMMUFix */
        0x0, /* gcFEATURE_BIT_REG_EEZ */
        0x0, /* gcFEATURE_BIT_REG_BugFixes21 */
        0x0, /* gcFEATURE_BIT_REG_ExtraVgCaps */
        0x0, /* gcFEATURE_BIT_REG_MultiSrcV15 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes22 */
        0x0, /* gcFEATURE_BIT_REG_Halti3 */
        0x0, /* gcFEATURE_BIT_REG_TessellationShaders */
        0x0, /* gcFEATURE_BIT_REG_OPF9Tap */
        0x1, /* gcFEATURE_BIT_REG_MultiSrcV2StrQuad */
        0x1, /* gcFEATURE_BIT_REG_SeperateSRCAndDstCache */
        0x0, /* gcFEATURE_BIT_REG_Halti4 */
        0x0, /* gcFEATURE_BIT_REG_RAWriteDepth */
        0x1, /* gcFEATURE_BIT_REG_AndroidOnly */
        0x1, /* gcFEATURE_BIT_REG_HasChipProductReg */
        0x0, /* gcFEATURE_BIT_REG_TXSupportDEC */
        0x0, /* gcFEATURE_BIT_REG_S8MSAACompression */
        0x0, /* gcFEATURE_BIT_REG_BugFixesIn544 */
        0x0, /* gcFEATURE_BIT_REG_L2CacheRemove */
        0x0, /* gcFEATURE_BIT_REG_FEAllowRndVtxCnt */
        0x0, /* gcFEATURE_BIT_REG_CubeMapFL28 */
        0x0, /* gcFEATURE_BIT_REG_TX6bitFrac */
        0x0, /* gcFEATURE_BIT_REG_FEAllowStallPrefetchEng */
        0x0, /* gcFEATURE_BIT_REG_ThirdPartyCompression */
        0x0, /* gcFEATURE_BIT_REG_RSS8 */
        0x0, /* gcFEATURE_BIT_REG_MSAACoherencyCheck */
        0x0, /* gcFEATURE_BIT_REG_Halti5 */
        0x0, /* gcFEATURE_BIT_REG_Evis */
        0x0, /* gcFEATURE_BIT_REG_BltEngine */
        0x0, /* gcFEATURE_BIT_REG_BugFixes23 */
        0x0, /* gcFEATURE_BIT_REG_BugFixes24 */
        0x0, /* gcFEATURE_BIT_REG_DEC */
        0x0, /* gcFEATURE_BIT_REG_VSTileNV12 */
        0x0, /* gcFEATURE_BIT_REG_VSTileNV12_10BIT */
        0x0, /* gcFEATURE_BIT_REG_DisableVIP */
        0x0, /* gcFEATURE_BIT_RenderTarget8 */
        0x0, /* gcFEATURE_BIT_TxLodFlowCorrection */
        0x0, /* gcFEATURE_BIT_FaceLod */
        0x0, /* gcFEATURE_BIT_MultiCoreSemaphoreStallV2 */
        0x0, /* gcFEATURE_BIT_VMSAA */
        0x0, /* gcFEATURE_BIT_ChipEnableLink */
        0x0, /* gcFEATURE_BIT_MULTI_SRC_BLT_1_5_ENHANCEMENT */
        0x0, /* gcFEATURE_BIT_MULTI_SRC_BLT_BILINEAR_FILTER */
        0x0, /* gcFEATURE_BIT_RA_HZEZ_CLOCK_CONTROL */
        0x0, /* gcFEATURE_BIT_CACHE128B256BPERLINE */
        0x0, /* gcFEATURE_BIT_V4Compression */
        0x0, /* gcFEATURE_BIT_DE2D_MAJOR_SUPER_TILE */
        0x0, /* gcFEATURE_BIT_PE2D_MAJOR_SUPER_TILE */
        0x0, /* gcFEATURE_BIT_PE_32BPC_COLORMASK_FIX */
        0x0, /* gcFEATURE_BIT_ALPHA_BLENDING_OPT */
        0x0, /* gcFEATURE_BIT_NEW_GPIPE */
        0x0, /* gcFEATURE_BIT_PIPELINE_32_ATTRIBUTES */
        0x0, /* gcFEATURE_BIT_MSAA_SHADING */
        0x0, /* gcFEATURE_BIT_NO_ANISTRO_FILTER */
        0x0, /* gcFEATURE_BIT_NO_ASTC */
        0x0, /* gcFEATURE_BIT_NO_DXT */
        0x0, /* gcFEATURE_BIT_HWTFB */
        0x0, /* gcFEATURE_BIT_RA_DEPTH_WRITE_MSAA1X_FIX */
        0x0, /* gcFEATURE_BIT_EZHZ_CLOCKGATE_FIX */
        0x0, /* gcFEATURE_BIT_SH_SNAP2PAGE_FIX */
        0x0, /* gcFEATURE_BIT_SH_HALFDEPENDENCY_FIX */
        0x0, /* gcFEATURE_BIT_USC_MCFILL_FIX */
        0x0, /* gcFEATURE_BIT_TPG_TCPERF_FIX */
        0x0, /* gcFEATURE_BIT_USC_MDFIFO_OVERFLOW_FIX */
        0x0, /* gcFEATURE_BIT_SH_TEXLD_BARRIER_IN_CS_FIX */
        0x0, /* gcFEATURE_BIT_RS_NEW_BASEADDR */
        0x0, /* gcFEATURE_BIT_PE_8bpp_DUALPIPE_FIX */
        0x0, /* gcFEATURE_BIT_SH_ADVANCED_INSTR */
        0x0, /* gcFEATURE_BIT_SH_FLAT_INTERPOLATION_DUAL16_FIX */
        0x0, /* gcFEATURE_BIT_USC_CONTINUOUS_FLUS_FIX */
        0x0, /* gcFEATURE_BIT_SH_SUPPORT_V4 */
        0x0, /* gcFEATURE_BIT_SH_SUPPORT_ALPHA_KILL */
        0x0, /* gcFEATURE_BIT_PE_NO_ALPHA_TEST */
        0x0, /* gcFEATURE_BIT_TX_LOD_NEAREST_SELECT */
        0x0, /* gcFEATURE_BIT_SH_FIX_LDEXP */
        0x1, /* gcFEATURE_BIT_SUPPORT_MOVAI */
        0x0, /* gcFEATURE_BIT_SH_SNAP2PAGE_MAXPAGES_FIX */
        0x0, /* gcFEATURE_BIT_PE_RGBA16I_FIX */
        0x0, /* gcFEATURE_BIT_BLT_8bpp_256TILE_FC_FIX */
        0x0, /* gcFEATURE_BIT_PE_64bit_FENCE_FIX */
        0x0, /* gcFEATURE_BIT_USC_FULL_CACHE_FIX */
        0x0, /* gcFEATURE_BIT_TX_YUV_ASSEMBLER_10BIT */
        0x0, /* gcFEATURE_BIT_FE_32bit_INDEX_FIX */
        0x0, /* gcFEATURE_BIT_BLT_64bpp_MASKED_CLEAR_FIX */
        0x0, /* gcFEATURE_BIT_SECURITY */
        0x0, /* gcFEATURE_BIT_ROBUSTNESS */
        0x0, /* gcFEATURE_BIT_USC_ATOMIC_FIX */
        0x0, /* gcFEATURE_BIT_SH_PSO_MSAA1x_FIX */
        0x0, /* gcFEATURE_BIT_USC_VX_PERF_FIX */
        0x0, /* gcFEATURE_BIT_USC_GOS_ADDR_FIX */
        0x0, /* gcFEATURE_BIT_TX_8bit_UVFrac */
        0x0, /* gcFEATURE_BIT_TX_DESC_CACHE_CLOCKGATE_FIX */
        0x0, /* gcFEATURE_BIT_RSBLT_MSAA_DECOMPRESSION */
        0x0, /* gcFEATURE_BIT_TX_INTEGER_COORDINATE */
        0x0, /* gcFEATURE_BIT_DRAWID */
        0x0, /* gcFEATURE_BIT_PSIO_SAMPLEMASK_IN_R0ZW_FIX */
        0x0, /* gcFEATURE_BIT_TX_INTEGER_COORDINATE_V2 */
        0x0, /* gcFEATURE_BIT_MULTI_CORE_BLOCK_SET_CONFIG */
        0x0, /* gcFEATURE_BIT_SNAPPAGE_CMD */
        0x0, /* gcFEATURE_BIT_SH_NO_INDEX_CONST_ON_A0 */
        0x0, /* gcFEATURE_BIT_SH_NO_ONECONST_LIMIT */
        0x0, /* gcFEATURE_BIT_SH_IMG_LDST_ON_TEMP */
        0x0, /* gcFEATURE_BIT_COMPUTE_ONLY */
        0x0, /* gcFEATURE_BIT_SH_IMG_LDST_CLAMP */
        0x0, /* gcFEATURE_BIT_SH_ICACHE_ALLOC_COUNT_FIX */
        0x0, /* gcFEATURE_BIT_SH_ICACHE_PREFETCH */
        0x0, /* gcFEATURE_BIT_PE2D_SEPARATE_CACHE */
        0x0, /* gcFEATURE_BIT_PE_MSAA_OQ_FIX */
        0x0, /* gcFEATURE_BIT_PSIO_MSAA_CL_FIX */
        0x0, /* gcFEATURE_BIT_USC_DEFER_FILL_FIX */
        0x0, /* gcFEATURE_BIT_SH_CLOCK_GATE_FIX */
        0x0, /* gcFEATURE_BIT_FE_NEED_DUMMYDRAW */
        0x1, /* gcFEATURE_BIT_PE2D_LINEAR_YUV420_OUTPUT */
        0x1, /* gcFEATURE_BIT_PE2D_LINEAR_YUV420_10BIT */
        0x0, /* gcFEATURE_BIT_MULTI_CLUSTER */
        0x0, /* gcFEATURE_BIT_SH_MULTI_WG_PACK */
        0x0, /* gcFEATURE_BIT_SH_DUAL16_SAMPLEMASK_ZW */
        0x0, /* gcFEATURE_BIT_TPG_TRIVIAL_MODE_FIX */
        0x0, /* gcFEATURE_BIT_TX_ASTC_MULTISLICE_FIX */
        0x0, /* gcFEATURE_BIT_FE_ROBUST_FIX */
        0x0, /* gcFEATURE_BIT_SH_GPIPE_ACCESS_FULLTEMPS */
        0x0, /* gcFEATURE_BIT_PSIO_INTERLOCK */
        0x0, /* gcFEATURE_BIT_PA_WIDELINE_FIX */
        0x0, /* gcFEATURE_BIT_WIDELINE_HELPER_FIX */
        0x0, /* gcFEATURE_BIT_G2D_3rd_PARTY_COMPRESSION_1_1 */
        0x0, /* gcFEATURE_BIT_TX_FLUSH_L1CACHE */
        0x0, /* gcFEATURE_BIT_PE_DITHER_FIX2 */
        0x0, /* gcFEATURE_BIT_SH_TEXLD_U_FIX */
        0x0, /* gcFEATURE_BIT_MC_FCCACHE_BYTEMASK */
        0x0, /* gcFEATURE_BIT_SH_MULTI_WG_PACK_FIX */
        0x0, /* gcFEATURE_BIT_PE_ADVANCE_BLEND_PART0 */
        0x0, /* gcFEATURE_BIT_FE_PATCHLIST_FETCH_FIX */
        0x0, /* gcFEATURE_BIT_RA_CG_FIX */
        0x0, /* gcFEATURE_BIT_DEC400 */
        0x0, /* gcFEATURE_BIT_LS_SUPPORT_PERCOMP_DEPENDENCY */
        0x0, /* gcFEATURE_BIT_MULTI_CORE_BLOCK_SET_CONFIG2 */
        0x0, /* gcFEATURE_BIT_PE_VMSAA_COVERAGE_CACHE_FIX */
        0x1, /* gcFEATURE_BIT_SECURITY_AHB */
        0x0, /* gcFEATURE_BIT_MULTICORE_SEMAPHORESTALL_V3 */
        0x0, /* gcFEATURE_BIT_SMALLBATCH */
        0x0, /* gcFEATURE_BIT_SH_CMPLX */
        0x0, /* gcFEATURE_BIT_SH_IDIV0_SWZL_EHS */
        0x0, /* gcFEATURE_BIT_TX_LERP_LESS_BIT */
        0x0, /* gcFEATURE_BIT_SH_GM_ENDIAN */
        0x0, /* gcFEATURE_BIT_SH_GM_USC_UNALLOC */
        0x0, /* gcFEATURE_BIT_SH_END_OF_BB */
        0x0, /* gcFEATURE_BIT_TX_BORDER_CLAMP_FIX */
        0x0, /* gcFEATURE_BIT_SH_IMG_LD_LASTPIXEL_FIX */
        0x0, /* gcFEATURE_BIT_ASYNC_BLT */
        0x0, /* gcFEATURE_BIT_ASYNC_FE_FENCE_FIX */
        0x0, /* gcFEATURE_BIT_PSCS_THROTTLE */
        0x0, /* gcFEATURE_BIT_SEPARATE_LS */
        0x0, /* gcFEATURE_BIT_WIDELINE_TRIANGLE_EMU */
        0x0, /* gcFEATURE_BIT_FENCE_32BIT */
        0x0, /* gcFEATURE_BIT_FENCE_64BIT */
        0x0, /* gcFEATURE_BIT_PE_DEPTH_ONLY_OQFIX */
        0x0, /* gcFEATURE_BIT_TX_SEAMLESS_CUBE */
        0x0, /* gcFEATURE_BIT_TX_SNORM_SUPPORT */
        0x0, /* gcFEATURE_BIT_SH_SCATTER_GATHER */
        0x0, /* gcFEATURE_BIT_HWMANAGED_LS */
        0x0, /* gcFEATURE_BIT_SH_IMAGE_ENABLE_FIX */
        0x0, /* gcFEATURE_BIT_MSAA_FRAGMENT_OPERATION */
        0x0, /* gcFEATURE_BIT_PE_TILE_CACHE_FLUSH_FIX */
        0x0, /* gcFEATURE_BIT_BLT_YUV_OUTPUT */
        0x0, /* gcFEATURE_BIT_SH_IO_CG_FIX */
        0x0, /* gcFEATURE_BIT_PE_SWIZZLE */
        0x0, /* gcFEATURE_BIT_SH_ROBUSTNESS_FIX */
        0x0, /* gcFEATURE_BIT_USC_ATOMIC_FIX2 */
        0x0, /* gcFEATURE_BIT_PE_A8B8G8R8 */
        0x0, /* gcFEATURE_BIT_MULTIVIEW_RENDER */
        0x0, /* gcFEATURE_BIT_FE_DRAW_DIRECT */
        0x0, /* gcFEATURE_BIT_TX_VKBORDER_MODE */
        0x0, /* gcFEATURE_BIT_TX_UNNORMALIZED_COORD */
        0x0, /* gcFEATURE_BIT_PA_LINECLIP_FIX */
        0x0, /* gcFEATURE_BIT_TX_8bit_UVFrac_ROUNDING_FIX */
        0x0, /* gcFEATURE_BIT_MP_ARCH */
        0x0, /* gcFEATURE_BIT_TX_NO_FIXED_FILTER */
        0x0, /* gcFEATURE_BIT_SHARE_Z */
        0x0, /* gcFEATURE_BIT_DE_2D_FAST_CLEAR */
        0x0, /* gcFEATURE_BIT_DE_TILESTATUS_ROTATION_FIX */
        0x0, /* gcFEATURE_BIT_TX_CLEAR_PENDING_FIX */
        0x0, /* gcFEATURE_BIT_HI1_L2_CACHE */
        0x0, /* gcFEATURE_BIT_USC_EVICT_CTRL_FIFO_FLOP_RESET_FIX */
        0x1, /* gcFEATURE_BIT_FORMAT_10BIT_CROSS_4K */
        0x1, /* gcFEATURE_BIT_FORMAT_P010LSB_I010 */
        0x1, /* gcFEATURE_BIT_ENDIAN_CONTROL */
        0x1, /* gcFEATURE_BIT_G2D_RGB_PLANAR */
        0x1, /* gcFEATURE_BIT_G2D_DEC400EX */
        0x1, /* gcFEATURE_BIT_G2D_NO_YUV420_SOURCE */
        0x0, /* gcFEATURE_BIT_G2D_YUV420_101010 */
        0x0, /* gcFEATURE_BIT_G2D_MultiSrcBlt_Pipe */
        0x0, /* gcFEATURE_BIT_G2D_Normalization */
        0x0, /* gcFEATURE_BIT_G2D_MASK_AND_COLORKEY */
        0x0, /* gcFEATURE_BIT_SH_VX2_FLOATING_MAD_FIX */
        0x0, /* gcFEATURE_BIT_TS_FC_VULKAN_SUPPORT */
        0x0, /* gcFEATURE_BIT_MSAA_FLOAT_64BIT */
        0x0, /* gcFEATURE_BIT_INDIRECT_COMPUTE_ZERODIM_FIX */
        0x0, /* gcFEATURE_BIT_Q_CHANNEL_SUPPORT */
        0x0, /* gcFEATURE_BIT_MMU_PAGE_DESCRIPTOR */
        0x0, /* gcFEATURE_BIT_YUV_LINEAR_TO_TILE_ROTATE */
        0x0, /* gcFEATURE_BIT_VEC2_IMULIMAD32_SUPPORT */
        0x0, /* gcFEATURE_BIT_VEC4_IMULIMAD32_SUPPORT */
        0x0, /* gcFEATURE_BIT_VEC2_IDIVIMOD16_SUPPORT */
        0x0, /* gcFEATURE_BIT_DST_TEX_I2F_F2I_INST_DEPRECATE */
        0x0, /* gcFEATURE_BIT_ALU_FP16_INSTRUCTIONS */
        0x0, /* gcFEATURE_BIT_DUAL16_14BIT_PC_SUPPORT */
        0x0, /* gcFEATURE_BIT_LDST_CONV_4ROUNDING_MODES */
        0x0, /* gcFEATURE_BIT_FULL_PACK_MODE_SUPPORT */
        0x0, /* gcFEATURE_BIT_DEPTH_FLOAT32_SUPPORT */
        0x0, /* gcFEATURE_BIT_GPU_INSPECTOR_COUNTERS */
        0x0, /* gcFEATURE_BIT_FP32_TO_FP16_CONV_FIX */
        0x0, /* gcFEATURE_BIT_IMGLD_COMP_COUNT_FIX */
        0x0, /* gcFEATURE_BIT_IMGLD_WIDTH_LT16_FIX */
        0x0, /* gcFEATURE_BIT_TX_FILTER_ROUND_FIX */
        0x0, /* gcFEATURE_BIT_SH_FP32_FMA_SUPPORT */
        0x0, /* gcFEATURE_BIT_TX_ETC2_COMPRESSION */
        0x0, /* gcFEATURE_BIT_VG_TS_CULLING */
        0x0, /* gcFEATURE_BIT_VG_FP25 */
        0x0, /* gcFEATURE_BIT_VG_AYUV_INPUT_OUTPUT */
        0x0, /* gcFEATURE_BIT_VG_DOUBLE_IMAGE */
        0x0, /* gcFEATURE_BIT_VG_RECTANGLE_STRIPE_MODE */
        0x0, /* gcFEATURE_BIT_VG_MMU */
        0x0, /* gcFEATURE_BIT_VG_IM_FILTER */
        0x0, /* gcFEATURE_BIT_VG_IM_YUV_PACKET */
        0x0, /* gcFEATURE_BIT_VG_IM_YUV_PLANAR */
        0x0, /* gcFEATURE_BIT_VG_PE_YUV_PACKET */
        0x0, /* gcFEATURE_BIT_VG_COLOR_PRECISION_8_BIT */
        0x0, /* gcFEATURE_BIT_VG_RESOLVE_ENGINE */
        0x0, /* gcFEATURE_BIT_VG_PE_COLOR_KEY */
        0x0, /* gcFEATURE_BIT_VG_IM_INDEX_FORMAT */
        0x0, /* gcFEATURE_BIT_VG_RESOLUTION_8K */
        0x0, /* gcFEATURE_BIT_VG_IMAGE_16K */
        0x0, /* gcFEATURE_BIT_VG_FORMAT_ARGB2222 */
        0x0, /* gcFEATURE_BIT_G2D_DEC400 */
        0x0, /* gcFEATURE_BIT_DC_OVERLAY_SCALING */
        0x0, /* gcFEATURE_BIT_DC_SOURCE_ROTATION */
        0x0, /* gcFEATURE_BIT_DC_TILED */
        0x0, /* gcFEATURE_BIT_DC_YUV_L1 */
        0x0, /* gcFEATURE_BIT_DC_D30_OUTPUT */
        0x0, /* gcFEATURE_BIT_DC_MMU */
        0x0, /* gcFEATURE_BIT_DC_COMPRESSION */
        0x0, /* gcFEATURE_BIT_DC_QOS */
        0x0, /* gcFEATURE_BIT_VIP_HW_FINAL_RELEASE */
        0x0, /* gcFEATURE_BIT_NN_SINGLEPORT_ACCUMBUFFER */
        0x0, /* gcFEATURE_BIT_NN_STRIDE_SUPPORT */
        0x0, /* gcFEATURE_BIT_SWTILING_PHASE1 */
        0x0, /* gcFEATURE_BIT_SWTILING_PHASE2 */
        0x0, /* gcFEATURE_BIT_TP_SIMPLE_INT16 */
        0x0, /* gcFEATURE_BIT_TP_REAL_INT16 */
        0x0, /* gcFEATURE_BIT_TP_ROI_POOLING */
        0x0, /* gcFEATURE_BIT_TP_MAX_POOLING_STRIDE1 */
        0x0, /* gcFEATURE_BIT_TP_LRN */
        0x0, /* gcFEATURE_BIT_TP_REORDER */
        0x0, /* gcFEATURE_BIT_TF_QUANTIZATION */
        0x0, /* gcFEATURE_BIT_NN_NONZERO_BORDER */
        0x0, /* gcFEATURE_BIT_NN_MIRROR_BORDER */
        0x0, /* gcFEATURE_BIT_AI_GPU */
        0x0, /* gcFEATURE_BIT_EVIS_NO_ABSDIFF */
        0x0, /* gcFEATURE_BIT_EVIS_NO_BITREPLACE */
        0x0, /* gcFEATURE_BIT_EVIS_NO_BOXFILTER */
        0x0, /* gcFEATURE_BIT_EVIS_NO_CORDIAC */
        0x0, /* gcFEATURE_BIT_EVIS_NO_DP32 */
        0x0, /* gcFEATURE_BIT_EVIS_NO_FILTER */
        0x0, /* gcFEATURE_BIT_EVIS_NO_IADD */
        0x0, /* gcFEATURE_BIT_EVIS_NO_SELECTADD */
        0x0, /* gcFEATURE_BIT_EVIS_LERP_7OUTPUT */
        0x0, /* gcFEATURE_BIT_EVIS_ACCSQ_8OUTPUT */
        0x0, /* gcFEATURE_BIT_EVIS_VX2 */
        0x0, /* gcFEATURE_BIT_TP_ENGINE */
        0x0, /* gcFEATURE_BIT_VIP_V7 */
        0x0, /* gcFEATURE_BIT_TP_TENSOR_ADD_MUL */
        0x0, /* gcFEATURE_BIT_NN_DEPTHWISE_INT16XINT8 */
        0x0, /* gcFEATURE_BIT_NN_DEPTHWISE_8BIT_VIP_V7 */
        0x0, /* gcFEATURE_BIT_TP_SOFTMAX */
        0x0, /* gcFEATURE_BIT_NN_23BITS_POST_MULTIPLIER_VIP_V7 */
        0x0, /* gcFEATURE_BIT_TP_23BITS_POST_MULTIPLIER_VIP_V7 */
        0x0, /* gcFEATURE_BIT_CONV_INT16X8BIT_VIP_V7 */
        0x0, /* gcFEATURE_BIT_NN_REMOVE_POOLING */
        0x0, /* gcFEATURE_BIT_NN_40BIT_BIAS */
        0x0, /* gcFEATURE_BIT_TP_REMOVE_USC */
        0x0, /* gcFEATURE_BIT_NN_ZDP6 */
        0x0, /* gcFEATURE_BIT_NN_XYDP9 */
        0x0, /* gcFEATURE_BIT_NN_FIRST_PIXEL_POOLING */
        0x0, /* gcFEATURE_BIT_NN_ZDP3 */
        0x0, /* gcFEATURE_BIT_NN_XYDP6 */
        0x0, /* gcFEATURE_BIT_SWTILING_PHASE3 */
        0x0, /* gcFEATURE_BIT_MCFE */
        0x0, /* gcFEATURE_BIT_USC_STAY_LRU */
        0x0, /* gcFEATURE_BIT_COEF_COMPRESSION_ENHANCEMENT */
        0x0, /* gcFEATURE_BIT_TP_COEF_COMPRESSION_ENHANCEMENT */
        0x0, /* gcFEATURE_BIT_NN_COEF_DECOMPRESS_PERF2X */
        0x0, /* gcFEATURE_BIT_TP_SMALLBATCH_PHASE1 */
        0x0, /* gcFEATURE_BIT_OCB_COUNTER */
        0x0, /* gcFEATURE_BIT_SCALER */
        0x0, /* gcFEATURE_BIT_SCALER_4K */
        0x0, /* gcFEATURE_BIT_INPUT_4BIT */
        0x0, /* gcFEATURE_BIT_NN_NO_Z_LOCATION_OFFSET */
        0x0, /* gcFEATURE_BIT_OCB_REMAP_PHYSICAL_ADDRESS */
        0x0, /* gcFEATURE_BIT_NN_SLOW_OUTPUT */
        0x0, /* gcFEATURE_BIT_NO_NARROW_POST_PROCESS_PIPE */
        0x0, /* gcFEATURE_BIT_TP_NN_PROBE */
        0x0, /* gcFEATURE_BIT_NN_DEPTHWISE_SUPPORT */
        0x0, /* gcFEATURE_BIT_NN_XYDP0 */
        0x0, /* gcFEATURE_BIT_NN_WRITE_WITHOUT_USC */
        0x0, /* gcFEATURE_BIT_NN_HW_LIMITATION_NATIVE_KER_1x2_2x1 */
        0x0, /* gcFEATURE_BIT_NN_SMALLBATCH_PHASE1 */
        0x0, /* gcFEATURE_BIT_NN_SLICE_PADDING_TO_64BYTE_ALIGN */
        0x0, /* gcFEATURE_BIT_NN_DW_1x1_CONV_MERGE */
        0x0, /* gcFEATURE_BIT_TP_BFLOAT16 */
        0x0, /* gcFEATURE_BIT_TP_23BITS_POST_MULTIPLIER */
        0x0, /* gcFEATURE_BIT_NN_TRANSPOSE */
        0x0, /* gcFEATURE_BIT_NN_ZDP_TRANSPOSE_CH9_ONLY */
        0x0, /* gcFEATURE_BIT_USE_SINGLE_PORT_VIPSRAM */
        0x0, /* gcFEATURE_BIT_NN_LEAKY_RELU */
        0x0, /* gcFEATURE_BIT_NN_PRELU */
        0x0, /* gcFEATURE_BIT_NN_PER_CHANNEL_QUANT */
        0x0, /* gcFEATURE_BIT_NN_PER_CHANNEL_QUANT_ASYM */
        0x0, /* gcFEATURE_BIT_NN_ASYMMETRIC_INT8 */
        0x0, /* gcFEATURE_BIT_NN_FLOAT_POST_MULT */
        0x0, /* gcFEATURE_BIT_PRELU_LEAKLY_RELU_CLAMP */
        0x0, /* gcFEATURE_BIT_TPLITE_BFLOAT16 */
        0x0, /* gcFEATURE_BIT_PREPROCESS_IMG_BUF_640BYTE_LIMIT */
        0x0, /* gcFEATURE_BIT_NN_POST_OUT_SUPPORT_FP16 */
        0x0, /* gcFEATURE_BIT_NN_POST_OUT_SUPPORT_BF16 */
        0x0, /* gcFEATURE_BIT_NN_POST_OUT_SUPPORT_FP32 */
        0x0, /* gcFEATURE_BIT_TP_KERNEL_1BYTE_ALGIN */
        0x0, /* gcFEATURE_BIT_BFLOAT_COEF_COMPRESSION_ZERO_COEFBIT14_INVERSE */
        0x0, /* gcFEATURE_BIT_NN_COMPRESSION_BYPASSS */
        0x0, /* gcFEATURE_BIT_TP_3_USC */
        0x0, /* gcFEATURE_BIT_BFP_COEF_AUTO_PAD_INCOMPLETE_ZERO_IN_KZ_PLANE */
        0x0, /* gcFEATURE_BIT_NN_NATIVE_STRIDE_TWO */
        0x0, /* gcFEATURE_BIT_NN_TENSOR_ADD */
        0x0, /* gcFEATURE_BIT_NN_FLOAT32_IO */
        0x0, /* gcFEATURE_BIT_TP_FLOAT32_IO */
        0x0, /* gcFEATURE_BIT_NN_SMALL_BATCH_PHASE2 */
        0x0, /* gcFEATURE_BIT_TILE_ACCESS_CAPABILITY */
        0x0, /* gcFEATURE_BIT_FAST_DP3_PREPROCESSOR */
        0x0, /* gcFEATURE_BIT_DEPTHWISE_SUPPORT_16BIT_FORMAT */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_ALU */
        0x0, /* gcFEATURE_BIT_NN_ENHANCED_MAX_POOLING */
        0x0, /* gcFEATURE_BIT_NN_TRANSPOSE_PHASE2 */
        0x0, /* gcFEATURE_BIT_NN_TENSOR_ADD_FIELD_MOVE_TO_EXT_CMD */
        0x0, /* gcFEATURE_BIT_NN_CONV_CORE_BYPASS */
        0x0, /* gcFEATURE_BIT_NN_TENSOR_ADD_RELU */
        0x0, /* gcFEATURE_BIT_TPLITE_SUPPORT_TP_DATA_TRANSPOSE */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_CONV_1D */
        0x0, /* gcFEATURE_BIT_USE_VIPSRAM_FOR_KERNEL_STREAMING */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_DUMMY_TILE */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_KERNEL_1BYTE_ALIGN */
        0x0, /* gcFEATURE_BIT_NN_1x1_NON_POOLING_PACKING */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_BOTH_CONV_NATIVE_STRIDE2_AND_POOLING */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_CONV1x1_AND_NATIVE_CONV_STRIDE2 */
        0x0, /* gcFEATURE_BIT_TP_REMOVE_FC */
        0x0, /* gcFEATURE_BIT_VIP_REMOVE_MMU */
        0x0, /* gcFEATURE_BIT_NN_MP_INTER_CONNECT_RING */
        0x0, /* gcFEATURE_BIT_NN_SUPPORT_BATCH */
        0x0, /* gcFEATURE_BIT_NN_2D_AVERAGE_OUTPUT */
        0x0, /* gcFEATURE_BIT_NN_JOB_CANCELATION */
        0x0, /* gcFEATURE_BIT_NN_DISTRIBUTED_VIPSRAM */
        0x0, /* gcFEATURE_BIT_NN_FC_ENHANCEMENT */
        0x0, /* gcFEATURE_BIT_VIP_DEC400 */
        0x0, /* gcFEATURE_BIT_NN_PER3DTILE_BUBBLE_FIX */
        0x0, /* gcFEATURE_BIT_NN_CACHELINE_MODE_PERF_FIX */
        0x0, /* gcFEATURE_BIT_NN_CONV1x1_PERF_FIX */
        0x0, /* gcFEATURE_BIT_TP_REORDER_FIX */
        0x0, /* gcFEATURE_BIT_NN_CONVOUT_FIFO_DEPTH_FIX */
        0x0, /* gcFEATURE_BIT_NN_ZXDP3_KERNEL_READ_CONFLICT_FIX */
        0x0, /* gcFEATURE_BIT_NN_ZDP3_NO_COMPRESS_FIX */
        0x0, /* gcFEATURE_BIT_NN_ASYNC_COPY_PERF_FIX */
        0x0, /* gcFEATURE_BIT_HI_REORDER_FIX */
        0x0, /* gcFEATURE_BIT_INCORRECT_WR_REQ_TO_USC_BETWEEN_REORDER_AND_NORMAL_LAYER_FIX */
        0x0, /* gcFEATURE_BIT_TP_REORDER_LAYER_SUSPEND_FIX */
        0x0, /* gcFEATURE_BIT_NN_ASYNC_COPY_MERGE_FIX */
        0x0, /* gcFEATURE_BIT_USC_INVALIDATE_CACHE_LINE_FIX */
        0x0, /* gcFEATURE_BIT_NN_REQ_SLOWARBITRATION_FIX */
        0x0, /* gcFEATURE_BIT_IMAGE_PARTIAL_CACHE_FIX */
        0x0, /* gcFEATURE_BIT_FULLCACHE_KERNELHEAD_FIX */
        0x0, /* gcFEATURE_BIT_NN_ZDP_INIMAGE_SIZE_FIX */
        0x0, /* gcFEATURE_BIT_IDLE_BEFORE_FLUSH_COMPLETE_FIX */
        0x0, /* gcFEATURE_BIT_NO_FLUSH_USC_FIX */
        0x0, /* gcFEATURE_BIT_SMALL_BATCH_FLOPS_RESET_FIX */
        0x0, /* gcFEATURE_BIT_SMALL_BATCH_DISBLE_FIX */
        0x0, /* gcFEATURE_BIT_OUTPUT_CONVERT_UINT8_INT8_TO_UINT16_INT16_FIX */
        0x0, /* gcFEATURE_BIT_IMAGE_NOT_PACKED_IN_SRAM_FIX */
        0x0, /* gcFEATURE_BIT_COEF_DELTA_CORD_OVERFLOW_ZRL_8BIT_FIX */
        0x0, /* gcFEATURE_BIT_USC_INDIVIDUAL_PORT_WRT_EARLY_EVICT_DATA_CORRUPT_FIX */
        0x0, /* gcFEATURE_BIT_LOW_EFFICIENCY_OF_ID_WRITE_IMGBUF_FIX */
        0x0, /* gcFEATURE_BIT_KERNEL_VIP_SRAM_READ_BW_LIMITATION_FIX */
        0x0, /* gcFEATURE_BIT_USC_BOTTLENECK_FIX */
        0x0, /* gcFEATURE_BIT_KERNEL_PER_CORE_LESS_THAN_THIRD_COEF_BUFF_DEPTH_FIX */
        0x0, /* gcFEATURE_BIT_NN_TILE_NUM_BIGGER_THAN_1024_FIX */
        0x0, /* gcFEATURE_BIT_KERNEL_SIZE_WASTE_IN_PARTIAL_MODE_FIX */
        0x0, /* gcFEATURE_BIT_NN_COMMAND_KERNEL_REQUEST_CONFICT_FIX */
        0x0, /* gcFEATURE_BIT_TP_REORDER_INTILE_X_SIZE_512_FIX */
        0x0, /* gcFEATURE_BIT_IMG_POP_PIPELINE_PAUSE_FIX */
        0x0, /* gcFEATURE_BIT_FULLCACHE_KERNEL_INTERLEAVE_FIX */
        0x0, /* gcFEATURE_BIT_V8_SINGLE_PORT_ACCUMULATION_BUFFER_RW_CONFICT_ZERO_SKIP_PERF_FIX */
        0x0, /* gcFEATURE_BIT_V8_ACCUMLATION_READ_OUT_HAS_BUBBLES_PERF_FIX */
        0x0, /* gcFEATURE_BIT_DEPTHWISE_NEIGHBOR_IMG_DATA_TRANSFER_NOT_EFFICIENT_FIX */
        0x0, /* gcFEATURE_BIT_DR_JD_DIFF_CONDITION_FOR_CACHELINE_MODE_PRE_FIX */
        0x0, /* gcFEATURE_BIT_TP_ACCESS_VIPSRAM_OT_IS_ONE_FIX */
        0x0, /* gcFEATURE_BIT_EVIS2_FLOP_RESET_FIX */
        0x0, /* gcFEATURE_BIT_OUTIMAGE_X_BITWIDTH_LIMIT_FOR_NN_TRANSPOSE_FIX */
        0x0, /* gcFEATURE_BIT_USC_ASYNC_CP_RTN_FLOP_RESET_FIX */
        0x0, /* gcFEATURE_BIT_IMG_ADDR_NOT_WRAP_IF_OVER_OCB_ADDR_FIX */
        0x0, /* gcFEATURE_BIT_NEGATIVE_POST_SHIFT_FIX */
        0x0, /* gcFEATURE_BIT_INIMAGE_2DTILE_NOT_LESS_160PIXEL_FIX */
        0x0, /* gcFEATURE_BIT_IMG_CAHCE_MODE_MUST_0_IN_IMG_DIRECT_MODE_FIX */
        0x0, /* gcFEATURE_BIT_BURST_COLLECT_DUMMY_DATA_WASTE_CYCLES_FIX */
        0x0, /* gcFEATURE_BIT_INIMG_NOT_64BYTE_ALIGN_CACHELINE_MODE_FIX */
        0x0, /* gcFEATURE_BIT_TP_FC_FLOAT_LAST_PIXEL_NEGATIVE_0_FIX */
        0x0, /* gcFEATURE_BIT_NN_WASTE_COEF_READ_WRITE_BANDWIDTH_128BYTE_VIPSRAM_IN_FULL_PATIAL_CACHE_MODE_FIX */
        0x0, /* gcFEATURE_BIT_NN_IN_TILE_DATA_IS_ALL_PAD_FIX */
        0x0, /* gcFEATURE_BIT_NN_TP_INSTR_COMPLETE_IN_SAME_CYCLE_WITH_WAIT_EVENT_FIX */
        0x0, /* gcFEATURE_BIT_CORE_IMAGE_TRANSER_NOT_EFFICIENT_BETWEEN_PARTITION_FIX */
        0x0, /* gcFEATURE_BIT_TP_FC_KERNEL_STREAM_MUST_LESS_THAN_OR_EQUAL_TO_64BYTE_WHEN_1BYTE_ALGINE_FIX */
        0x0, /* gcFEATURE_BIT_NN_KERNEL_1x1_NO_PAD_FIX */
        0x0, /* gcFEATURE_BIT_NN_DEPTHWISE_AFTER_16BIT_LAYER_LIMIT_FIX */
        0x0, /* gcFEATURE_BIT_TP_NOT_FULL_USE_CACHE_LINE_FIX */
        0x1, /* gcFEATURE_BIT_SH_MOVAI_MOVAR_UNUSED_COMPONENTS_WRITE_DIRTY_DATA_FIX */
        0x1, /* gcFEATURE_BIT_BURST_COLLECT_CONSUMES_MC_DATA_WIDTH_PER_CYCLE_FIX */
        0x0, /* gcFEATURE_BIT_TP_ASSYM_INT8_FIX */
        0x0, /* gcFEATURE_BIT_NN_PAD_SLICE_ERROR_WHEN_TRANSPSE_FIX */
        0x0, /* gcFEATURE_BIT_NN_2ND_IMG_BASE_ADDR_FIX */
        0x0, /* gcFEATURE_BIT_NN_TP_SYSTEM_FIX */
        0x0, /* gcFEATURE_BIT_NN_INTILE_YSIZE_128_LIMIT_FIX */
        0x0, /* gcFEATURE_BIT_SH_CLOCK_GATOR_IDLE_CONDITON_FIX */
        0x0, /* gcFEATURE_BIT_NN_BURST_COLLECTER_LAST_FLAG_FIX */
        0x0, /* gcFEATURE_BIT_NN_2ND_IMG_SMALL_3D_TILE_FIX */
        0x0, /* gcFEATURE_BIT_NN_TILE_YSIZE_127_LIMITATION_FIX */
        0x0, /* gcFEATURE_BIT_NN_CONV_1D_16BIT_FORMAT_INTILE_SIZE_LIMITATION_FIX */
        0x0, /* gcFEATURE_BIT_NN_VIPSRAM_DOUBLE_BUFFER_FIX */
        0x0, /* gcFEATURE_BIT_NN_JD_DIRECT_MODE_FIX */
        0x0, /* gcFEATURE_BIT_NN_KERNEL_DIRECT_WRONG_PUSH_FIX */
        0x0, /* gcFEATURE_BIT_HI_DEFAULT_ENABLE_REORDER_FIX */
        0x0, /* gcFEATURE_BIT_V83_INTILESIZE_1X1_10BITS_FIX */
        0x0, /* gcFEATURE_BIT_DEPTHWISE_FLOAT_FIX */
        0x0, /* gcFEATURE_BIT_TP_CIRCULAR_BUF_WRAP_ADDRESS_OVERFLOW_FIX */
        0x0, /* gcFEATURE_BIT_NN_CIRCULAR_BUF_WRAP_ADDRESS_OVERFLOW_FIX */
        0x0, /* gcFEATURE_BIT_CLOCK_DIV2_FREQ_CHANGE_FIX */
        0x0, /* gcFEATURE_BIT_SMALL_TILE_TENSOR_ADD_FIX */
        0x0, /* gcFEATURE_BIT_DECOMPRESSOR_DEPTHWISE_FLOAT_FIX */
        0x0, /* gcFEATURE_BIT_NN_INTERLEVE8 */
        0x0, /* gcFEATURE_BIT_NN_FP16_ALU */
        0x0, /* gcFEATURE_BIT_NN_INT16_ALU */
        0x0, /* gcFEATURE_BIT_NN_INT8_SCALE */
        0x0, /* gcFEATURE_BIT_NN_POWER_ISOLATION */
        0x0, /* gcFEATURE_BIT_ZRL_7BIT */
        0x0, /* gcFEATURE_BIT_NN_SMALLBATCH */
        0x0, /* gcFEATURE_BIT_TP_SMALLBATCH */
        0x0, /* gcFEATURE_BIT_ZRL_8BIT */
        0x0, /* gcFEATURE_BIT_DDR_BURST_LEN_256B */
        0x0, /* gcFEATURE_BIT_XY_OFFSET_LIMITATION_FIX */
        0x0, /* gcFEATURE_BIT_NN_NONZERO_MIRROR_BORDER */
        0x0, /* gcFEATURE_BIT_IMAGE_PARTIAL_CACHE */
    },
};

static gcsFEATURE_DATABASE*
gcQueryFeatureDB(
    gctUINT32 ChipID,
    gctUINT32 ChipVersion,
    gctUINT32 ProductID,
    gctUINT32 EcoID,
    gctUINT32 CustomerID
    )
{
    gctINT entryNum = sizeof(gChipInfo) / sizeof(gChipInfo[0]);
    gctINT i;

    /* check formal release entries first */
    for (i = 0; i < entryNum; ++i)
    {

        if ((gChipInfo[i].chipID == ChipID)
            && (gChipInfo[i].chipVersion == ChipVersion)
            && (gChipInfo[i].productID == ProductID)
            && (gChipInfo[i].ecoID == EcoID)
            && (gChipInfo[i].customerID == CustomerID)
            && (gChipInfo[i].formalRelease)
           )
        {
            return &gChipInfo[i];
        }
    }

    /* check informal release entries if we dont find in formal entries */
    for (i = 0; i < entryNum; ++i)
    {

        if ((gChipInfo[i].chipID == ChipID)
            && ((gChipInfo[i].chipVersion & 0xFFF0) == (ChipVersion & 0xFFF0))
            && (gChipInfo[i].productID == ProductID)
            && (gChipInfo[i].ecoID == EcoID)
            && (gChipInfo[i].customerID == CustomerID)
            && (!gChipInfo[i].formalRelease)
           )
        {
            return &gChipInfo[i];
        }
    }

    return gcvNULL;
}
#endif /* _gc_feature_database_h_ */
