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


#ifndef __gc_hal_profiler_shared_h_
#define __gc_hal_profiler_shared_h_

#ifdef __cplusplus
extern "C" {
#endif

#define ANDROID_PROFILER_COUNTERS   1
#define APP_INFO   1
#define FPGA_INFO   0
#define RECORD_COUNTER_ADDRESS  0

/* HW profile information. */
typedef struct _gcsPROFILER_COUNTERS_PART1
{
    gctUINT32       gpuTotalRead64BytesPerFrame;
    gctUINT32       gpuTotalWrite64BytesPerFrame;

    /* FE */
    gctUINT32       fe_draw_count;
    gctUINT32       fe_out_vertex_count;
    gctUINT32       fe_cache_miss_count;
    gctUINT32       fe_cache_lk_count;
    gctUINT32       fe_stall_count;
    gctUINT32       fe_starve_count;
    gctUINT32       fe_process_count;

    /* PE */
    gctUINT32       pe0_pixel_count_killed_by_color_pipe;
    gctUINT32       pe0_pixel_count_killed_by_depth_pipe;
    gctUINT32       pe0_pixel_count_drawn_by_color_pipe;
    gctUINT32       pe0_pixel_count_drawn_by_depth_pipe;
    gctUINT32       pe1_pixel_count_killed_by_color_pipe;
    gctUINT32       pe1_pixel_count_killed_by_depth_pipe;
    gctUINT32       pe1_pixel_count_drawn_by_color_pipe;
    gctUINT32       pe1_pixel_count_drawn_by_depth_pipe;

    /* SH */
    gctUINT32       shader_cycle_count;
    gctUINT32       vs_shader_cycle_count;
    gctUINT32       ps_shader_cycle_count;
    gctUINT32       ps_inst_counter;
    gctUINT32       ps_rendered_pixel_counter;
    gctUINT32       vs_inst_counter;
    gctUINT32       vs_rendered_vertice_counter;
    gctUINT32       vs_branch_inst_counter;
    gctUINT32       vs_texld_inst_counter;
    gctUINT32       ps_branch_inst_counter;
    gctUINT32       ps_texld_inst_counter;
    gctUINT32       vs_non_idle_starve_count;
    gctUINT32       vs_starve_count;
    gctUINT32       vs_stall_count;
    gctUINT32       vs_process_count;
    gctUINT32       ps_non_idle_starve_count;
    gctUINT32       ps_starve_count;
    gctUINT32       ps_stall_count;
    gctUINT32       ps_process_count;

    /* PA */
    gctUINT32       pa_input_vtx_counter;
    gctUINT32       pa_input_prim_counter;
    gctUINT32       pa_output_prim_counter;
    gctUINT32       pa_depth_clipped_counter;
    gctUINT32       pa_trivial_rejected_counter;
    gctUINT32       pa_culled_prim_counter;
    gctUINT32       pa_droped_prim_counter;
    gctUINT32       pa_frustum_clipped_prim_counter;
    gctUINT32       pa_frustum_clipdroped_prim_counter;
    gctUINT32       pa_non_idle_starve_count;
    gctUINT32       pa_starve_count;
    gctUINT32       pa_stall_count;
    gctUINT32       pa_process_count;

    /* SE */
    gctUINT32       se_culled_triangle_count;
    gctUINT32       se_culled_lines_count;
    gctUINT32       se_clipped_triangle_count;
    gctUINT32       se_clipped_line_count;
    gctUINT32       se_starve_count;
    gctUINT32       se_stall_count;
    gctUINT32       se_receive_triangle_count;
    gctUINT32       se_send_triangle_count;
    gctUINT32       se_receive_lines_count;
    gctUINT32       se_send_lines_count;
    gctUINT32       se_process_count;
    gctUINT32       se_trivial_rejected_line_count;
    gctUINT32       se_non_idle_starve_count;

    /* RA */
    gctUINT32       ra_input_prim_count;
    gctUINT32       ra_total_quad_count;
    gctUINT32       ra_valid_quad_count_after_early_z;
    gctUINT32       ra_valid_pixel_count_to_render;
    gctUINT32       ra_output_valid_quad_count;
    gctUINT32       ra_output_valid_pixel_count;
    gctUINT32       ra_pipe_cache_miss_counter;
    gctUINT32       ra_pipe_hz_cache_miss_counter;
    gctUINT32       ra_prefetch_cache_miss_counter;
    gctUINT32       ra_prefetch_hz_cache_miss_counter;
    gctUINT32       ra_eez_culled_counter;
    gctUINT32       ra_non_idle_starve_count;
    gctUINT32       ra_starve_count;
    gctUINT32       ra_stall_count;
    gctUINT32       ra_process_count;

    /* TX */
    gctUINT32       tx_total_bilinear_requests;
    gctUINT32       tx_total_trilinear_requests;
    gctUINT32       tx_total_discarded_texture_requests;
    gctUINT32       tx_total_texture_requests;
    gctUINT32       tx_mc0_miss_count;
    gctUINT32       tx_mc0_request_byte_count;
    gctUINT32       tx_mc1_miss_count;
    gctUINT32       tx_mc1_request_byte_count;
    gctUINT32       tx_non_idle_starve_count;
    gctUINT32       tx_starve_count;
    gctUINT32       tx_stall_count;
    gctUINT32       tx_process_count;
}
gcsPROFILER_COUNTERS_PART1;

typedef struct _gcsPROFILER_COUNTERS_PART2
{
    /* MCC */
    gctUINT32       mcc_total_read_req_8B_from_colorpipe;
    gctUINT32       mcc_total_read_req_8B_sentout_from_colorpipe;
    gctUINT32       mcc_total_write_req_8B_from_colorpipe;
    gctUINT32       mcc_total_read_req_sentout_from_colorpipe;
    gctUINT32       mcc_total_write_req_from_colorpipe;
    gctUINT32       mcc_total_read_req_8B_from_depthpipe;
    gctUINT32       mcc_total_read_req_8B_sentout_from_depthpipe;
    gctUINT32       mcc_total_write_req_8B_from_depthpipe;
    gctUINT32       mcc_total_read_req_sentout_from_depthpipe;
    gctUINT32       mcc_total_write_req_from_depthpipe;
    gctUINT32       mcc_total_read_req_8B_from_others;
    gctUINT32       mcc_total_write_req_8B_from_others;
    gctUINT32       mcc_total_read_req_from_others;
    gctUINT32       mcc_total_write_req_from_others;
    gctUINT32       mcc_axi_total_latency;
    gctUINT32       mcc_axi_sample_count;
    gctUINT32       mcc_axi_max_latency;
    gctUINT32       mcc_axi_min_latency;
    gctUINT32       mc_fe_read_bandwidth;
    gctUINT32       mc_mmu_read_bandwidth;
    gctUINT32       mc_blt_read_bandwidth;
    gctUINT32       mc_sh0_read_bandwidth;
    gctUINT32       mc_sh1_read_bandwidth;
    gctUINT32       mc_pe_write_bandwidth;
    gctUINT32       mc_blt_write_bandwidth;
    gctUINT32       mc_sh0_write_bandwidth;
    gctUINT32       mc_sh1_write_bandwidth;

    /* MCZ */
    gctUINT32       mcz_total_read_req_8B_from_colorpipe;
    gctUINT32       mcz_total_read_req_8B_sentout_from_colorpipe;
    gctUINT32       mcz_total_write_req_8B_from_colorpipe;
    gctUINT32       mcz_total_read_req_sentout_from_colorpipe;
    gctUINT32       mcz_total_write_req_from_colorpipe;
    gctUINT32       mcz_total_read_req_8B_from_depthpipe;
    gctUINT32       mcz_total_read_req_8B_sentout_from_depthpipe;
    gctUINT32       mcz_total_write_req_8B_from_depthpipe;
    gctUINT32       mcz_total_read_req_sentout_from_depthpipe;
    gctUINT32       mcz_total_write_req_from_depthpipe;
    gctUINT32       mcz_total_read_req_8B_from_others;
    gctUINT32       mcz_total_write_req_8B_from_others;
    gctUINT32       mcz_total_read_req_from_others;
    gctUINT32       mcz_total_write_req_from_others;
    gctUINT32       mcz_axi_total_latency;
    gctUINT32       mcz_axi_sample_count;
    gctUINT32       mcz_axi_max_latency;
    gctUINT32       mcz_axi_min_latency;

    /* HI */
    gctUINT32       hi0_total_read_8B_count;
    gctUINT32       hi0_total_write_8B_count;
    gctUINT32       hi0_total_read_request_count;
    gctUINT32       hi0_total_write_request_count;
    gctUINT32       hi0_axi_cycles_read_request_stalled;
    gctUINT32       hi0_axi_cycles_write_request_stalled;
    gctUINT32       hi0_axi_cycles_write_data_stalled;
    gctUINT32       hi1_total_read_8B_count;
    gctUINT32       hi1_total_write_8B_count;
    gctUINT32       hi1_total_read_request_count;
    gctUINT32       hi1_total_write_request_count;
    gctUINT32       hi1_axi_cycles_read_request_stalled;
    gctUINT32       hi1_axi_cycles_write_request_stalled;
    gctUINT32       hi1_axi_cycles_write_data_stalled;
    gctUINT32       hi_total_cycle_count;
    gctUINT32       hi_total_idle_cycle_count;
    gctUINT32       hi_total_read_8B_count;
    gctUINT32       hi_total_write_8B_count;
    gctUINT32       hi_total_readOCB_16B_count;
    gctUINT32       hi_total_writeOCB_16B_count;

    /* L2 */
    gctUINT32       l2_total_axi0_read_request_count;
    gctUINT32       l2_total_axi1_read_request_count;
    gctUINT32       l2_total_axi0_write_request_count;
    gctUINT32       l2_total_axi1_write_request_count;
    gctUINT32       l2_total_read_transactions_request_by_axi0;
    gctUINT32       l2_total_read_transactions_request_by_axi1;
    gctUINT32       l2_total_write_transactions_request_by_axi0;
    gctUINT32       l2_total_write_transactions_request_by_axi1;
    gctUINT32       l2_axi0_minmax_latency;
    gctUINT32       l2_axi0_min_latency;
    gctUINT32       l2_axi0_max_latency;
    gctUINT32       l2_axi0_total_latency;
    gctUINT32       l2_axi0_total_request_count;
    gctUINT32       l2_axi1_minmax_latency;
    gctUINT32       l2_axi1_min_latency;
    gctUINT32       l2_axi1_max_latency;
    gctUINT32       l2_axi1_total_latency;
    gctUINT32       l2_axi1_total_request_count;
}
gcsPROFILER_COUNTERS_PART2;

typedef struct _gcsPROFILER_COUNTERS
{
    gcsPROFILER_COUNTERS_PART1 counters_part1;
    gcsPROFILER_COUNTERS_PART2 counters_part2;
}
gcsPROFILER_COUNTERS;

typedef enum _gceVIP_PROBE_COUNTER
{
    gcvVIP_PROBE_COUNTER_NEURAL_NET,
    gcvVIP_PROBE_COUNTER_TENSOR_PROCESSOR,
    gcvVIP_PROBE_COUNTER_COUNT
}
gceVIP_PROBE_COUNTER;

/* Mask definations for overflow indicator of TP */
typedef enum _gceTPCOUNTER_OVERFLOW
{
    gcvTPCOUNTER_LAYER_ID_OVERFLOW                  = (1 << 0),
    gcvTPCOUNTER_TOTAL_BUSY_CYCLE_OVERFLOW          = (1 << 1),
    gcvTPCOUNTER_TOTAL_READ_BW_DDR_OVERFLOW         = (1 << 2),
    gcvTPCOUNTER_TOTAL_WRITE_BW_DDR_OVERFLOW        = (1 << 3),
    gcvTPCOUNTER_TOTAL_READ_BW_SRAM_OVERFLOW        = (1 << 4),
    gcvTPCOUNTER_TOTAL_WRITE_BW_SRAM_OVERFLOW       = (1 << 5),
    gcvTPCOUNTER_TOTAL_READ_BW_OCB_OVERFLOW         = (1 << 6),
    gcvTPCOUNTER_TOTAL_WRITE_BW_OCB_OVERFLOW        = (1 << 7),
    gcvTPCOUNTER_FC_PIX_CNT_OVERFLOW                = (1 << 8),
    gcvTPCOUNTER_FC_ZERO_SKIP_OVERFLOW              = (1 << 9),
    gcvTPCOUNTER_FC_COEF_CNT_OVERFLOW               = (1 << 10),
    gcvTPCOUNTER_FC_COEF_ZERO_CNT_OVERFLOW          = (1 << 11),
    gcvTPCOUNTER_TOTAL_IDLE_CYCLE_CORE0_OVERFLOW    = (1 << 0),
    gcvTPCOUNTER_TOTAL_IDLE_CYCLE_CORE1_OVERFLOW    = (1 << 1),
    gcvTPCOUNTER_TOTAL_IDLE_CYCLE_CORE2_OVERFLOW    = (1 << 2),
    gcvTPCOUNTER_TOTAL_IDLE_CYCLE_CORE3_OVERFLOW    = (1 << 3),
}
_gceTPCOUNTER_OVERFLOW;

/* Mask definations for overflow indicator of NN */
typedef enum _gceNNCOUNTER_OVERFLOW
{
    gcvNNCOUNTER_TOTAL_BUSY_CYCLE_OVERFLOW          = (1 << 0),
    gcvNNCOUNTER_TOTAL_READ_CYCLE_DDR_OVERFLOW      = (1 << 2),
    gcvNNCOUNTER_TOTAL_READ_BW_DDR_OVERFLOW         = (1 << 3),
    gcvNNCOUNTER_TOTAL_WRITE_CYCLE_DDR_OVERFLOW     = (1 << 4),
    gcvNNCOUNTER_TOTAL_WRITE_BW_DDR_OVERFLOW        = (1 << 5),
    gcvNNCOUNTER_TOTAL_READ_SYCLE_SRAM_OVERFLOW     = (1 << 6),
    gcvNNCOUNTER_TOTAL_WRITE_CYCLE_SRAM_OVERFLOW    = (1 << 7),
    gcvNNCOUNTER_TOTAL_MAC_CYCLE_OVERFLOW           = (1 << 8),
    gcvNNCOUNTER_TOTAL_MAC_COUNT_OVERFLOW           = (1 << 9),
    gcvNNCOUNTER_ZERO_COEF_SKIP_COUNT_OVERFLOW      = (1 << 10),
    gcvNNCOUNTER_NON_ZERO_COEF_COUNT_OVERFLOW       = (1 << 11),
}
_gceNNCOUNTER_OVERFLOW;

#define   MODULE_NN_RESERVED_COUNTER_NUM           0x9
typedef struct _gcsPROFILER_VIP_PROBE_COUNTERS
{
    /* NN */
    gctUINT32       nn_layer_id;
    gctUINT32       nn_layer_id_overflow;
    gctUINT32       nn_instr_info;
    gctUINT32       nn_total_busy_cycle;
    gctUINT32       nn_total_busy_cycle_overflow;
    gctUINT32       nn_total_read_cycle_ddr;
    gctUINT32       nn_total_read_cycle_ddr_overflow;
    gctUINT32       nn_total_read_valid_bandwidth_ddr;
    gctUINT32       nn_total_read_valid_bandwidth_ddr_overflow;
    gctUINT32       nn_total_write_cycle_ddr;
    gctUINT32       nn_total_write_cycle_ddr_overflow;
    gctUINT32       nn_total_write_valid_bandwidth_ddr;
    gctUINT32       nn_total_write_valid_bandwidth_ddr_overflow;
    gctUINT32       nn_total_read_cycle_sram;
    gctUINT32       nn_total_read_cycle_sram_overflow;
    gctUINT32       nn_total_write_cycle_sram;
    gctUINT32       nn_total_write_cycle_sram_overflow;
    gctUINT32       nn_total_mac_cycle;
    gctUINT32       nn_total_mac_cycle_overflow;
    gctUINT32       nn_total_mac_count;
    gctUINT32       nn_total_mac_count_overflow;
    gctUINT32       nn_zero_coef_skip_count;
    gctUINT32       nn_zero_coef_skip_count_overflow;
    gctUINT32       nn_non_zero_coef_count;
    gctUINT32       nn_non_zero_coef_count_overflow;

    gctUINT32       nn_reserved_counter[4 * MODULE_NN_RESERVED_COUNTER_NUM];
    gctUINT32       nn_total_idle_cycle_core_overflow[4];
    gctUINT32       nn_total_idle_cycle_core[32];

    /* TP */
    gctUINT32       tp_layer_id;
    gctUINT32       tp_layer_id_overflow;
    gctUINT32       tp_total_busy_cycle;
    gctUINT32       tp_total_busy_cycle_overflow;

    gctUINT32       tp_total_read_bandwidth_cache;
    gctUINT32       tp_total_read_bandwidth_cache_overflow;
    gctUINT32       tp_total_write_bandwidth_cache;
    gctUINT32       tp_total_write_bandwidth_cache_overflow;

    gctUINT32       tp_total_read_bandwidth_sram;
    gctUINT32       tp_total_read_bandwidth_sram_overflow;
    gctUINT32       tp_total_write_bandwidth_sram;
    gctUINT32       tp_total_write_bandwidth_sram_overflow;


    gctUINT32       tp_total_read_bandwidth_ocb;
    gctUINT32       tp_total_read_bandwidth_ocb_overflow;
    gctUINT32       tp_total_write_bandwidth_ocb;
    gctUINT32       tp_total_write_bandwidth_ocb_overflow;

    gctUINT32       tp_fc_pix_count;
    gctUINT32       tp_fc_zero_skip_count;
    gctUINT32       tp_fc_pix_count_overflow;
    gctUINT32       tp_fc_zero_skip_count_overflow;

    gctUINT32       tp_fc_coef_count;
    gctUINT32       tp_fc_coef_zero_count;
    gctUINT32       tp_fc_coef_count_overflow;
    gctUINT32       tp_fc_coef_zero_count_overflow;

    gctUINT32       tp_total_idle_cycle_core[16];
    gctUINT32       tp_total_idle_cycle_core_overflows[16];

    /* VIP SH */
}
gcsPROFILER_VIP_PROBE_COUNTERS;

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_profiler_shared_h_ */



