/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <string.h>
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"

#include "bsf.h"
#include "bsf_internal.h"
#include "cbs.h"
#include "cbs_h265.h"
#include "hevc/hevc.h"
#include "hevc/ps.h"
#include "hevc/hevcdec.h"
#include "h2645_vui.h"

/* 预定义的纵横比 */
extern const AVRational ff_h2645_pixel_aspect[17];

typedef struct USVCTranscoderContext {
    const AVClass *class;
    
    int verbose;           /* 详细日志选项 */
    
    CodedBitstreamContext *cbc;
    CodedBitstreamFragment fragment;
    
    HEVCParamSets ps;     /* HEVC参数集 */

} USVCTranscoderContext;

/**
 * 初始化HEVC参数集
 */
static void hevc_ps_init(HEVCParamSets *ps)
{
    int i;
    
    // 初始化VPS列表
    for (i = 0; i < HEVC_MAX_VPS_COUNT; i++)
        ps->vps_list[i] = NULL;
    
    // 初始化SPS列表
    for (i = 0; i < HEVC_MAX_SPS_COUNT; i++)
        ps->sps_list[i] = NULL;
    
    // 初始化PPS列表
    for (i = 0; i < HEVC_MAX_PPS_COUNT; i++)
        ps->pps_list[i] = NULL;
}

/**
 * 处理NAL单元
 * 根据NAL类型执行不同的处理
 */
static int process_nal_unit(USVCTranscoderContext *s, AVBSFContext *ctx, 
                           CodedBitstreamUnit *unit) {
    int ret = 0;
    HEVCVPS *vps;
    HEVCSPS *sps;
    HEVCPPS *pps;
    int vps_id, sps_id, pps_id;
    int i;
    
    if (s->verbose) {
        /* 根据 NAL 类型输出详细信息 */
        switch (unit->type) {
            case HEVC_NAL_VPS:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: VPS\n");
                break;
            case HEVC_NAL_SPS:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: SPS\n");
                break;
            case HEVC_NAL_PPS:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: PPS\n");
                break;
            case HEVC_NAL_IDR_W_RADL:
            case HEVC_NAL_IDR_N_LP:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: IDR slice\n");
                break;
            case HEVC_NAL_TRAIL_R:
            case HEVC_NAL_TRAIL_N:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: Trail slice\n");
                break;
            case HEVC_NAL_SEI_PREFIX:
            case HEVC_NAL_SEI_SUFFIX:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: SEI\n");
                break;
            default:
                av_log(ctx, AV_LOG_DEBUG, "Processing NAL: type %d\n", unit->type);
                break;
        }
    }
    
    /* 根据NAL类型执行不同处理 */
    switch (unit->type) {
        case HEVC_NAL_VPS:
            {
                /* 处理VPS */
                H265RawVPS *vps_cbs = (H265RawVPS *)unit->content;
                if (!vps_cbs)
                    break;
                
                vps_id = vps_cbs->vps_video_parameter_set_id;
                
                /* 检查VPS ID是否有效 */
                if (vps_id >= HEVC_MAX_VPS_COUNT) {
                    av_log(ctx, AV_LOG_ERROR, "VPS id %d out of range\n", vps_id);
                    return AVERROR_INVALIDDATA;
                }
                
                /* 如果该ID已存在VPS，则释放它 */
                if (s->ps.vps_list[vps_id] != NULL)
                    av_freep(&s->ps.vps_list[vps_id]);
                
                /* 分配新的VPS结构 */
                vps = av_mallocz(sizeof(*vps));
                if (!vps)
                    return AVERROR(ENOMEM);
                
                /* 将CBS解析的VPS数据复制到参数集中 */
                vps->vps_id = vps_id;
                vps->vps_temporal_id_nesting_flag = vps_cbs->vps_temporal_id_nesting_flag;
                vps->vps_max_layers = vps_cbs->vps_max_layers_minus1 + 1;
                vps->vps_max_sub_layers = vps_cbs->vps_max_sub_layers_minus1 + 1;
                
                /* 复制PTL信息 */
                vps->ptl.general_ptl.profile_space = vps_cbs->profile_tier_level.general_profile_space;
                vps->ptl.general_ptl.tier_flag = vps_cbs->profile_tier_level.general_tier_flag;
                vps->ptl.general_ptl.profile_idc = vps_cbs->profile_tier_level.general_profile_idc;
                
                /* 复制兼容性标志 */
                for (i = 0; i < 32; i++) {
                    vps->ptl.general_ptl.profile_compatibility_flag[i] = 
                        vps_cbs->profile_tier_level.general_profile_compatibility_flag[i];
                }
                
                /* 复制约束标志 */
                vps->ptl.general_ptl.progressive_source_flag = vps_cbs->profile_tier_level.general_progressive_source_flag;
                vps->ptl.general_ptl.interlaced_source_flag = vps_cbs->profile_tier_level.general_interlaced_source_flag;
                vps->ptl.general_ptl.non_packed_constraint_flag = vps_cbs->profile_tier_level.general_non_packed_constraint_flag;
                vps->ptl.general_ptl.frame_only_constraint_flag = vps_cbs->profile_tier_level.general_frame_only_constraint_flag;
                
                /* 复制级别信息 */
                vps->ptl.general_ptl.level_idc = vps_cbs->profile_tier_level.general_level_idc;
                
                /* 复制子层信息 */
                for (i = 0; i < vps->vps_max_sub_layers - 1; i++) {
                    vps->ptl.sub_layer_profile_present_flag[i] = vps_cbs->profile_tier_level.sub_layer_profile_present_flag[i];
                    vps->ptl.sub_layer_level_present_flag[i] = vps_cbs->profile_tier_level.sub_layer_level_present_flag[i];
                    
                    if (vps->ptl.sub_layer_profile_present_flag[i]) {
                        vps->ptl.sub_layer_ptl[i].profile_space = vps_cbs->profile_tier_level.sub_layer_profile_space[i];
                        vps->ptl.sub_layer_ptl[i].tier_flag = vps_cbs->profile_tier_level.sub_layer_tier_flag[i];
                        vps->ptl.sub_layer_ptl[i].profile_idc = vps_cbs->profile_tier_level.sub_layer_profile_idc[i];
                        
                        /* 复制子层兼容性标志 */
                        for (int j = 0; j < 32; j++) {
                            vps->ptl.sub_layer_ptl[i].profile_compatibility_flag[j] = 
                                vps_cbs->profile_tier_level.sub_layer_profile_compatibility_flag[i][j];
                        }
                        
                        /* 复制子层约束标志 */
                        vps->ptl.sub_layer_ptl[i].progressive_source_flag = 
                            vps_cbs->profile_tier_level.sub_layer_progressive_source_flag[i];
                        vps->ptl.sub_layer_ptl[i].interlaced_source_flag = 
                            vps_cbs->profile_tier_level.sub_layer_interlaced_source_flag[i];
                        vps->ptl.sub_layer_ptl[i].non_packed_constraint_flag = 
                            vps_cbs->profile_tier_level.sub_layer_non_packed_constraint_flag[i];
                        vps->ptl.sub_layer_ptl[i].frame_only_constraint_flag = 
                            vps_cbs->profile_tier_level.sub_layer_frame_only_constraint_flag[i];
                    }
                    
                    if (vps->ptl.sub_layer_level_present_flag[i]) {
                        vps->ptl.sub_layer_ptl[i].level_idc = vps_cbs->profile_tier_level.sub_layer_level_idc[i];
                    }
                }
                
                /* 复制视频参数集扩展标志 */
                vps->vps_sub_layer_ordering_info_present_flag = vps_cbs->vps_sub_layer_ordering_info_present_flag;
                
                /* 复制子层排序信息 */
                for (i = (vps->vps_sub_layer_ordering_info_present_flag ? 0 : vps->vps_max_sub_layers - 1);
                     i < vps->vps_max_sub_layers; i++) {
                    vps->vps_max_dec_pic_buffering[i] = vps_cbs->vps_max_dec_pic_buffering_minus1[i] + 1;
                    vps->vps_num_reorder_pics[i] = vps_cbs->vps_max_num_reorder_pics[i];
                    vps->vps_max_latency_increase[i] = vps_cbs->vps_max_latency_increase_plus1[i] - 1;
                }
                
                /* 复制VPS最大层ID和层集信息 */
                vps->vps_max_layer_id = vps_cbs->vps_max_layer_id;
                vps->vps_num_layer_sets = vps_cbs->vps_num_layer_sets_minus1 + 1;
                
                /* 复制时序信息标志 */
                vps->vps_timing_info_present_flag = vps_cbs->vps_timing_info_present_flag;
                
                /* 如果存在时序信息，复制相关参数 */
                if (vps->vps_timing_info_present_flag) {
                    vps->vps_num_units_in_tick = vps_cbs->vps_num_units_in_tick;
                    vps->vps_time_scale = vps_cbs->vps_time_scale;
                    vps->vps_poc_proportional_to_timing_flag = vps_cbs->vps_poc_proportional_to_timing_flag;
                    
                    if (vps->vps_poc_proportional_to_timing_flag) {
                        vps->vps_num_ticks_poc_diff_one = vps_cbs->vps_num_ticks_poc_diff_one_minus1 + 1;
                    }
                }
                
                /* 将VPS保存到列表中 */
                s->ps.vps_list[vps_id] = vps;
                
                if (s->verbose)
                    av_log(ctx, AV_LOG_DEBUG, "Added VPS id: %d, max_layers: %d, max_sub_layers: %d, profile_idc: %d, level_idc: %d\n", 
                           vps_id, vps->vps_max_layers, vps->vps_max_sub_layers, 
                           vps->ptl.general_ptl.profile_idc, vps->ptl.general_ptl.level_idc);
            }
            break;
            
        case HEVC_NAL_SPS:
            {
                /* 处理SPS */
                H265RawSPS *sps_cbs = (H265RawSPS *)unit->content;
                if (!sps_cbs)
                    break;
                
                sps_id = sps_cbs->sps_seq_parameter_set_id;
                
                /* 检查SPS ID是否有效 */
                if (sps_id >= HEVC_MAX_SPS_COUNT) {
                    av_log(ctx, AV_LOG_ERROR, "SPS id %d out of range\n", sps_id);
                    return AVERROR_INVALIDDATA;
                }
                
                /* 如果该ID已存在SPS，则释放它 */
                if (s->ps.sps_list[sps_id] != NULL) {
                    av_freep(&s->ps.sps_list[sps_id]);
                }
                
                /* 分配新的SPS结构 */
                sps = av_mallocz(sizeof(*sps));
                if (!sps)
                    return AVERROR(ENOMEM);
                
                /* 将CBS解析的SPS数据复制到参数集中 */
                sps->vps_id = sps_cbs->sps_video_parameter_set_id;
                sps->chroma_format_idc = sps_cbs->chroma_format_idc;
                sps->separate_colour_plane = sps_cbs->separate_colour_plane_flag;
                sps->width = sps_cbs->pic_width_in_luma_samples;
                sps->height = sps_cbs->pic_height_in_luma_samples;
                
                /* 色度位深 */
                sps->bit_depth = sps_cbs->bit_depth_luma_minus8 + 8;
                sps->bit_depth_chroma = sps_cbs->bit_depth_chroma_minus8 + 8;
                
                /* 编码块相关参数 */
                sps->log2_min_cb_size = sps_cbs->log2_min_luma_coding_block_size_minus3 + 3;
                sps->log2_diff_max_min_coding_block_size = sps_cbs->log2_diff_max_min_luma_coding_block_size;
                sps->log2_min_tb_size = sps_cbs->log2_min_luma_transform_block_size_minus2 + 2;
                sps->log2_diff_max_min_transform_block_size = sps_cbs->log2_diff_max_min_luma_transform_block_size;
                sps->max_transform_hierarchy_depth_inter = sps_cbs->max_transform_hierarchy_depth_inter;
                sps->max_transform_hierarchy_depth_intra = sps_cbs->max_transform_hierarchy_depth_intra;
                
                /* CTB相关参数 */
                sps->log2_ctb_size = sps_cbs->log2_min_luma_coding_block_size_minus3 + 3 + 
                                    sps_cbs->log2_diff_max_min_luma_coding_block_size;
                sps->ctb_width = (sps->width + (1 << sps->log2_ctb_size) - 1) >> sps->log2_ctb_size;
                sps->ctb_height = (sps->height + (1 << sps->log2_ctb_size) - 1) >> sps->log2_ctb_size;
                sps->ctb_size = sps->ctb_width * sps->ctb_height;
                
                /* PCM相关参数 */
                sps->pcm_enabled = sps_cbs->pcm_enabled_flag;
                if (sps->pcm_enabled) {
                    sps->pcm.bit_depth = sps_cbs->pcm_sample_bit_depth_luma_minus1 + 1;
                    sps->pcm.bit_depth_chroma = sps_cbs->pcm_sample_bit_depth_chroma_minus1 + 1;
                    sps->pcm.log2_min_pcm_cb_size = sps_cbs->log2_min_pcm_luma_coding_block_size_minus3 + 3;
                    sps->pcm_loop_filter_disabled = sps_cbs->pcm_loop_filter_disabled_flag;
                }
                
                /* 配置窗口相关参数 */
                sps->conformance_window = sps_cbs->conformance_window_flag;
                if (sps->conformance_window) {
                    sps->pic_conf_win.left_offset = sps_cbs->conf_win_left_offset;
                    sps->pic_conf_win.right_offset = sps_cbs->conf_win_right_offset;
                    sps->pic_conf_win.top_offset = sps_cbs->conf_win_top_offset;
                    sps->pic_conf_win.bottom_offset = sps_cbs->conf_win_bottom_offset;
                }
                
                /* POC相关参数 */
                sps->log2_max_poc_lsb = sps_cbs->log2_max_pic_order_cnt_lsb_minus4 + 4;
                
                /* 子层排序信息 */
                sps->sublayer_ordering_info = sps_cbs->sps_sub_layer_ordering_info_present_flag;
                
                for (i = (sps->sublayer_ordering_info ? 0 : sps->max_sub_layers - 1);
                     i < sps->max_sub_layers; i++) {
                    sps->temporal_layer[i].max_dec_pic_buffering = sps_cbs->sps_max_dec_pic_buffering_minus1[i] + 1;
                    sps->temporal_layer[i].num_reorder_pics = sps_cbs->sps_max_num_reorder_pics[i];
                    sps->temporal_layer[i].max_latency_increase = sps_cbs->sps_max_latency_increase_plus1[i] - 1;
                }
                
                /* 时间相关参数 */
                sps->max_sub_layers = sps_cbs->sps_max_sub_layers_minus1 + 1;
                sps->temporal_id_nesting = sps_cbs->sps_temporal_id_nesting_flag;
                
                /* Profile, Tier, Level 信息 */
                sps->ptl.general_ptl.profile_space = sps_cbs->profile_tier_level.general_profile_space;
                sps->ptl.general_ptl.tier_flag = sps_cbs->profile_tier_level.general_tier_flag;
                sps->ptl.general_ptl.profile_idc = sps_cbs->profile_tier_level.general_profile_idc;
                
                /* 兼容性标志 */
                for (i = 0; i < 32; i++) {
                    sps->ptl.general_ptl.profile_compatibility_flag[i] = 
                        sps_cbs->profile_tier_level.general_profile_compatibility_flag[i];
                }
                
                /* 约束标志 */
                sps->ptl.general_ptl.progressive_source_flag = sps_cbs->profile_tier_level.general_progressive_source_flag;
                sps->ptl.general_ptl.interlaced_source_flag = sps_cbs->profile_tier_level.general_interlaced_source_flag;
                sps->ptl.general_ptl.non_packed_constraint_flag = sps_cbs->profile_tier_level.general_non_packed_constraint_flag;
                sps->ptl.general_ptl.frame_only_constraint_flag = sps_cbs->profile_tier_level.general_frame_only_constraint_flag;
                
                /* 级别信息 */
                sps->ptl.general_ptl.level_idc = sps_cbs->profile_tier_level.general_level_idc;
                
                /* 子层PTL信息 */
                for (i = 0; i < sps->max_sub_layers - 1; i++) {
                    sps->ptl.sub_layer_profile_present_flag[i] = sps_cbs->profile_tier_level.sub_layer_profile_present_flag[i];
                    sps->ptl.sub_layer_level_present_flag[i] = sps_cbs->profile_tier_level.sub_layer_level_present_flag[i];
                    
                    if (sps->ptl.sub_layer_profile_present_flag[i]) {
                        sps->ptl.sub_layer_ptl[i].profile_space = sps_cbs->profile_tier_level.sub_layer_profile_space[i];
                        sps->ptl.sub_layer_ptl[i].tier_flag = sps_cbs->profile_tier_level.sub_layer_tier_flag[i];
                        sps->ptl.sub_layer_ptl[i].profile_idc = sps_cbs->profile_tier_level.sub_layer_profile_idc[i];
                        
                        /* 子层兼容性标志 */
                        for (int j = 0; j < 32; j++) {
                            sps->ptl.sub_layer_ptl[i].profile_compatibility_flag[j] = 
                                sps_cbs->profile_tier_level.sub_layer_profile_compatibility_flag[i][j];
                        }
                        
                        sps->ptl.sub_layer_ptl[i].progressive_source_flag = 
                            sps_cbs->profile_tier_level.sub_layer_progressive_source_flag[i];
                        sps->ptl.sub_layer_ptl[i].interlaced_source_flag = 
                            sps_cbs->profile_tier_level.sub_layer_interlaced_source_flag[i];
                        sps->ptl.sub_layer_ptl[i].non_packed_constraint_flag = 
                            sps_cbs->profile_tier_level.sub_layer_non_packed_constraint_flag[i];
                        sps->ptl.sub_layer_ptl[i].frame_only_constraint_flag = 
                            sps_cbs->profile_tier_level.sub_layer_frame_only_constraint_flag[i];
                    }
                    
                    if (sps->ptl.sub_layer_level_present_flag[i]) {
                        sps->ptl.sub_layer_ptl[i].level_idc = sps_cbs->profile_tier_level.sub_layer_level_idc[i];
                    }
                }
                
                /* 短期参考图像集 */
                sps->nb_st_rps = sps_cbs->num_short_term_ref_pic_sets;
                for (i = 0; i < sps->nb_st_rps; i++) {
                    H265RawSTRefPicSet *st_ref_pic_set = &sps_cbs->st_ref_pic_set[i];
                    ShortTermRPS *st_rps = &sps->st_rps[i];
                    
                    st_rps->num_negative_pics = st_ref_pic_set->num_negative_pics;
                    st_rps->num_delta_pocs = st_ref_pic_set->num_negative_pics + st_ref_pic_set->num_positive_pics;
                    
                    /* 复制delta_poc值 */
                    for (int j = 0; j < st_rps->num_negative_pics; j++) {
                        st_rps->delta_poc[j] = -(st_ref_pic_set->delta_poc_s0_minus1[j] + 1);
                        if (st_ref_pic_set->used_by_curr_pic_s0_flag[j])
                            st_rps->used |= (1 << j);
                    }
                    
                    for (int j = 0; j < st_ref_pic_set->num_positive_pics; j++) {
                        st_rps->delta_poc[st_rps->num_negative_pics + j] = st_ref_pic_set->delta_poc_s1_minus1[j] + 1;
                        if (st_ref_pic_set->used_by_curr_pic_s1_flag[j])
                            st_rps->used |= (1 << (st_rps->num_negative_pics + j));
                    }
                }
                
                /* 长期参考图像集 */
                sps->long_term_ref_pics_present = sps_cbs->long_term_ref_pics_present_flag;
                if (sps->long_term_ref_pics_present) {
                    sps->num_long_term_ref_pics_sps = sps_cbs->num_long_term_ref_pics_sps;
                    for (i = 0; i < sps->num_long_term_ref_pics_sps; i++) {
                        sps->lt_ref_pic_poc_lsb_sps[i] = sps_cbs->lt_ref_pic_poc_lsb_sps[i];
                        if (sps_cbs->used_by_curr_pic_lt_sps_flag[i])
                            sps->used_by_curr_pic_lt |= (1 << i);
                    }
                }
                
                /* 剩余标志位 */
                sps->scaling_list_enabled = sps_cbs->scaling_list_enabled_flag;
                sps->amp_enabled = sps_cbs->amp_enabled_flag;
                sps->sao_enabled = sps_cbs->sample_adaptive_offset_enabled_flag;
                sps->temporal_mvp_enabled = sps_cbs->sps_temporal_mvp_enabled_flag;
                sps->strong_intra_smoothing_enabled = sps_cbs->strong_intra_smoothing_enabled_flag;
                
                /* VUI参数 */
                sps->vui_present = sps_cbs->vui_parameters_present_flag;
                if (sps->vui_present) {
                    /* 定时信息 */
                    sps->vui.vui_timing_info_present_flag = sps_cbs->vui.vui_timing_info_present_flag;
                    if (sps->vui.vui_timing_info_present_flag) {
                        sps->vui.vui_num_units_in_tick = sps_cbs->vui.vui_num_units_in_tick;
                        sps->vui.vui_time_scale = sps_cbs->vui.vui_time_scale;
                    }
                    
                    /* 视频信号类型 */
                    if (sps_cbs->vui.video_signal_type_present_flag) {
                        sps->vui.common.video_format = sps_cbs->vui.video_format;
                        sps->vui.common.video_full_range_flag = sps_cbs->vui.video_full_range_flag;
                    }
                    
                    /* 比特流限制 */
                    sps->vui.bitstream_restriction_flag = sps_cbs->vui.bitstream_restriction_flag;
                }
                
                /* SPS扩展标志 */
                sps->extension_present = sps_cbs->sps_extension_present_flag;
                if (sps->extension_present) {
                    sps->range_extension = sps_cbs->sps_range_extension_flag;
                    sps->scc_extension = sps_cbs->sps_scc_extension_flag;
                    
                    /* 范围扩展 */
                    if (sps->range_extension) {
                        sps->transform_skip_rotation_enabled = sps_cbs->transform_skip_rotation_enabled_flag;
                        sps->transform_skip_context_enabled = sps_cbs->transform_skip_context_enabled_flag;
                        sps->implicit_rdpcm_enabled = sps_cbs->implicit_rdpcm_enabled_flag;
                        sps->explicit_rdpcm_enabled = sps_cbs->explicit_rdpcm_enabled_flag;
                        sps->extended_precision_processing = sps_cbs->extended_precision_processing_flag;
                        sps->intra_smoothing_disabled = sps_cbs->intra_smoothing_disabled_flag;
                        sps->high_precision_offsets_enabled = sps_cbs->high_precision_offsets_enabled_flag;
                        sps->persistent_rice_adaptation_enabled = sps_cbs->persistent_rice_adaptation_enabled_flag;
                        sps->cabac_bypass_alignment_enabled = sps_cbs->cabac_bypass_alignment_enabled_flag;
                    }
                    
                    /* SCC扩展 */
                    if (sps->scc_extension) {
                        sps->curr_pic_ref_enabled = sps_cbs->sps_curr_pic_ref_enabled_flag;
                        sps->palette_mode_enabled = sps_cbs->palette_mode_enabled_flag;
                        if (sps->palette_mode_enabled) {
                            sps->palette_max_size = sps_cbs->palette_max_size;
                            sps->delta_palette_max_predictor_size = sps_cbs->delta_palette_max_predictor_size;
                            
                            sps->palette_predictor_initializers_present = sps_cbs->sps_palette_predictor_initializer_present_flag;
                            if (sps->palette_predictor_initializers_present) {
                                sps->sps_num_palette_predictor_initializers = sps_cbs->sps_num_palette_predictor_initializer_minus1 + 1;
                                
                                /* 复制调色板预测器初始化值 */
                                for (i = 0; i < 3; i++) {
                                    for (int j = 0; j < sps->sps_num_palette_predictor_initializers; j++) {
                                        sps->sps_palette_predictor_initializer[i][j] = sps_cbs->sps_palette_predictor_initializers[i][j];
                                    }
                                }
                            }
                        }
                        
                        sps->motion_vector_resolution_control_idc = sps_cbs->motion_vector_resolution_control_idc;
                        sps->intra_boundary_filtering_disabled = sps_cbs->intra_boundary_filtering_disable_flag;
                    }
                }
                
                /* 计算一些派生值 */
                sps->min_cb_width = sps->width >> sps->log2_min_cb_size;
                sps->min_cb_height = sps->height >> sps->log2_min_cb_size;
                sps->min_tb_width = sps->width >> sps->log2_min_tb_size;
                sps->min_tb_height = sps->height >> sps->log2_min_tb_size;
                sps->min_pu_width = sps->min_cb_width;
                sps->min_pu_height = sps->min_cb_height;
                sps->tb_mask = (1 << (sps->log2_ctb_size - sps->log2_min_tb_size)) - 1;
                
                /* 色度子采样移位 */
                if (sps->chroma_format_idc == 1) {
                    sps->hshift[0] = 0;
                    sps->vshift[0] = 0;
                    sps->hshift[1] = 1;
                    sps->hshift[2] = 1;
                    sps->vshift[1] = 1;
                    sps->vshift[2] = 1;
                } else if (sps->chroma_format_idc == 2) {
                    sps->hshift[0] = 0;
                    sps->vshift[0] = 0;
                    sps->hshift[1] = 1;
                    sps->hshift[2] = 1;
                    sps->vshift[1] = 0;
                    sps->vshift[2] = 0;
                } else {
                    sps->hshift[0] = 0;
                    sps->vshift[0] = 0;
                    sps->hshift[1] = 0;
                    sps->hshift[2] = 0;
                    sps->vshift[1] = 0;
                    sps->vshift[2] = 0;
                }
                
                /* 像素格式转换 */
                sps->pixel_shift = sps->bit_depth > 8;
                
                /* 量化参数偏移 */
                sps->qp_bd_offset = 6 * (sps->bit_depth - 8);
                
                /* 链接VPS参考 */
                if (s->ps.vps_list[sps->vps_id] != NULL) {
                    sps->vps = s->ps.vps_list[sps->vps_id];
                }
                
                /* 将SPS保存到列表中 */
                s->ps.sps_list[sps_id] = sps;
                
                if (s->verbose)
                    av_log(ctx, AV_LOG_DEBUG, "Added SPS id: %d, resolution: %dx%d, bit_depth: %d, profile_idc: %d, level_idc: %d\n", 
                           sps_id, sps->width, sps->height, sps->bit_depth, 
                           sps->ptl.general_ptl.profile_idc, sps->ptl.general_ptl.level_idc);
            }
            break;
            
        case HEVC_NAL_PPS:
            {
                /* 处理PPS */
                H265RawPPS *pps_cbs = (H265RawPPS *)unit->content;
                if (!pps_cbs)
                    break;
                
                pps_id = pps_cbs->pps_pic_parameter_set_id;
                
                /* 检查PPS ID是否有效 */
                if (pps_id >= HEVC_MAX_PPS_COUNT) {
                    av_log(ctx, AV_LOG_ERROR, "PPS id %d out of range\n", pps_id);
                    return AVERROR_INVALIDDATA;
                }
                
                /* 如果该ID已存在PPS，则释放它 */
                if (s->ps.pps_list[pps_id] != NULL) {
                    av_freep(&s->ps.pps_list[pps_id]);
                }
                
                /* 分配新的PPS结构 */
                pps = av_mallocz(sizeof(*pps));
                if (!pps)
                    return AVERROR(ENOMEM);
                
                /* 将CBS解析的PPS数据复制到参数集中 */
                pps->pps_id = pps_id;
                pps->sps_id = pps_cbs->pps_seq_parameter_set_id;
                
                /* 查找对应的SPS */
                if (s->ps.sps_list[pps->sps_id] == NULL) {
                    av_log(ctx, AV_LOG_WARNING, "SPS id %d for PPS id %d not available\n", 
                           pps->sps_id, pps_id);
                } else {
                    /* 将SPS链接到PPS */
                    pps->sps = s->ps.sps_list[pps->sps_id];
                }
                
                /* 复制PPS标志 - 使用正确的字段名 */
                pps->dependent_slice_segments_enabled_flag = pps_cbs->dependent_slice_segments_enabled_flag;
                pps->output_flag_present_flag = pps_cbs->output_flag_present_flag;
                pps->num_extra_slice_header_bits = pps_cbs->num_extra_slice_header_bits;
                pps->sign_data_hiding_flag = pps_cbs->sign_data_hiding_enabled_flag;
                pps->cabac_init_present_flag = pps_cbs->cabac_init_present_flag;
                pps->constrained_intra_pred_flag = pps_cbs->constrained_intra_pred_flag;
                pps->transform_skip_enabled_flag = pps_cbs->transform_skip_enabled_flag;
                pps->cu_qp_delta_enabled_flag = pps_cbs->cu_qp_delta_enabled_flag;
                
                /* 添加默认参考索引和初始QP参数 */
                pps->num_ref_idx_l0_default_active = pps_cbs->num_ref_idx_l0_default_active_minus1 + 1;
                pps->num_ref_idx_l1_default_active = pps_cbs->num_ref_idx_l1_default_active_minus1 + 1;
                pps->pic_init_qp_minus26 = pps_cbs->init_qp_minus26;

                /* 加权预测相关参数 */
                pps->weighted_pred_flag = pps_cbs->weighted_pred_flag;
                pps->weighted_bipred_flag = pps_cbs->weighted_bipred_flag;
                
                /* QP相关参数 */
                if (pps->cu_qp_delta_enabled_flag)
                    pps->diff_cu_qp_delta_depth = pps_cbs->diff_cu_qp_delta_depth;
                
                pps->cb_qp_offset = pps_cbs->pps_cb_qp_offset;
                pps->cr_qp_offset = pps_cbs->pps_cr_qp_offset;
                
                /* 滤波相关标志 */
                pps->deblocking_filter_control_present_flag = pps_cbs->deblocking_filter_control_present_flag;
                if (pps->deblocking_filter_control_present_flag) {
                    pps->deblocking_filter_override_enabled_flag = pps_cbs->deblocking_filter_override_enabled_flag;
                    pps->disable_dbf = pps_cbs->pps_deblocking_filter_disabled_flag;
                    if (!pps->disable_dbf) {
                        pps->beta_offset = pps_cbs->pps_beta_offset_div2 * 2;
                        pps->tc_offset = pps_cbs->pps_tc_offset_div2 * 2;
                    }
                }
                
                /* Tile相关参数 */
                pps->tiles_enabled_flag = pps_cbs->tiles_enabled_flag;
                if (pps->tiles_enabled_flag) {
                    pps->num_tile_columns = pps_cbs->num_tile_columns_minus1 + 1;
                    pps->num_tile_rows = pps_cbs->num_tile_rows_minus1 + 1;
                    pps->uniform_spacing_flag = pps_cbs->uniform_spacing_flag;
                    
                    /* 如果不是uniform spacing，需要复制tile参数 */
                    if (!pps->uniform_spacing_flag) {
                        if (pps->num_tile_columns > 1) {
                            for (i = 0; i < pps->num_tile_columns - 1; i++)
                                pps->column_width[i] = pps_cbs->column_width_minus1[i] + 1;
                        }
                        
                        if (pps->num_tile_rows > 1) {
                            for (i = 0; i < pps->num_tile_rows - 1; i++)
                                pps->row_height[i] = pps_cbs->row_height_minus1[i] + 1;
                        }
                    }
                    
                    pps->loop_filter_across_tiles_enabled_flag = pps_cbs->loop_filter_across_tiles_enabled_flag;
                }
                
                /* 环路滤波标志 */
                pps->seq_loop_filter_across_slices_enabled_flag = pps_cbs->pps_loop_filter_across_slices_enabled_flag;
                
                /* 熵编码相关标志 */
                pps->entropy_coding_sync_enabled_flag = pps_cbs->entropy_coding_sync_enabled_flag;
                
                /* 复杂滤波相关标志 */
                pps->transquant_bypass_enable_flag = pps_cbs->transquant_bypass_enabled_flag;
                
                /* 并行合并级别 */
                pps->log2_parallel_merge_level = pps_cbs->log2_parallel_merge_level_minus2 + 2;
                
                /* 列表修改存在标志 */
                pps->lists_modification_present_flag = pps_cbs->lists_modification_present_flag;
                
                /* PPS扩展标志 */
                pps->pps_extension_present_flag = pps_cbs->pps_extension_present_flag;
                if (pps->pps_extension_present_flag) {
                    pps->pps_range_extensions_flag = pps_cbs->pps_range_extension_flag;
                    pps->pps_scc_extension_flag = pps_cbs->pps_scc_extension_flag;
                    
                    /* SCC扩展处理 */
                    if (pps->pps_scc_extension_flag) {
                        /* 如果有其他SCC扩展字段需要处理，可以在这里添加 */
                        if (s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "PPS id: %d has SCC extension\n", pps_id);
                        }
                    }
                }
                
                /* 将PPS保存到列表中 */
                s->ps.pps_list[pps_id] = pps;
                
                if (s->verbose) {
                    av_log(ctx, AV_LOG_DEBUG, "Added PPS id: %d, SPS id: %d, tiles: %s, entropy_sync: %s\n", 
                           pps_id, pps->sps_id, 
                           pps->tiles_enabled_flag ? "enabled" : "disabled",
                           pps->entropy_coding_sync_enabled_flag ? "enabled" : "disabled");
                }
            }
            break;
            
        case HEVC_NAL_IDR_W_RADL:
        case HEVC_NAL_IDR_N_LP:
        case HEVC_NAL_TRAIL_R:
        case HEVC_NAL_TRAIL_N:
            {
                /* 处理slice NAL */
                H265RawSlice *slice = (H265RawSlice *)unit->content;
                int pps_id;
                const HEVCPPS *pps;
                const HEVCSPS *sps;
                SliceHeader sh;
                
                if (!slice)
                    break;
                
                /* 初始化SliceHeader结构 */
                memset(&sh, 0, sizeof(sh));
                
                /* 在这里添加slice处理逻辑 */
                pps_id = slice->header.slice_pic_parameter_set_id;
                sh.pps_id = pps_id;
                
                /* 检查引用的PPS是否存在 */
                if (s->ps.pps_list[pps_id] == NULL) {
                    av_log(ctx, AV_LOG_WARNING, "Referenced PPS id %d not available\n", pps_id);
                    break;
                }
                
                pps = s->ps.pps_list[pps_id];
                sps = pps->sps;
                
                /* 复制Slice标志位和基础信息 */
                sh.first_slice_in_pic_flag = slice->header.first_slice_segment_in_pic_flag;
                sh.dependent_slice_segment_flag = slice->header.dependent_slice_segment_flag;
                sh.slice_segment_addr = slice->header.slice_segment_address;
                sh.slice_type = slice->header.slice_type;
                sh.pic_output_flag = slice->header.pic_output_flag;
                
                /* 复制参考图像相关参数 */
                sh.short_term_ref_pic_set_sps_flag = slice->header.short_term_ref_pic_set_sps_flag;
                
                /* 如果短期参考图像集不是从SPS获取的，则需要复制短期参考图像集参数 */
                if (!sh.short_term_ref_pic_set_sps_flag && sps) {
                    sh.short_term_ref_pic_set_size = 0; /* 需要从码流中解析 */
                    /* 注意：这里需要进一步处理short_term_rps的具体字段，这里只是简单示例 */
                }
                
                /* 复制时间MVP和参考图像索引相关标志 */
                sh.slice_temporal_mvp_enabled_flag = slice->header.slice_temporal_mvp_enabled_flag;
                sh.collocated_list = !slice->header.collocated_from_l0_flag;
                sh.collocated_ref_idx = slice->header.collocated_ref_idx;
                
                /* 复制参考图像列表数量 */
                if (slice->header.num_ref_idx_active_override_flag) {
                    sh.nb_refs[0] = slice->header.num_ref_idx_l0_active_minus1 + 1;
                    sh.nb_refs[1] = slice->header.num_ref_idx_l1_active_minus1 + 1;
                } else if (pps) {
                    sh.nb_refs[0] = pps->num_ref_idx_l0_default_active;
                    sh.nb_refs[1] = pps->num_ref_idx_l1_default_active;
                }
                
                /* 复制CABAC初始化标志 */
                sh.cabac_init_flag = slice->header.cabac_init_flag;
                
                /* 复制QP相关参数 */
                sh.slice_qp_delta = slice->header.slice_qp_delta;
                sh.slice_cb_qp_offset = slice->header.slice_cb_qp_offset;
                sh.slice_cr_qp_offset = slice->header.slice_cr_qp_offset;
                sh.cu_chroma_qp_offset_enabled_flag = slice->header.cu_chroma_qp_offset_enabled_flag;
                
                /* 复制SAO（样本自适应偏移）标志 */
                sh.slice_sample_adaptive_offset_flag[0] = slice->header.slice_sao_luma_flag;
                sh.slice_sample_adaptive_offset_flag[1] = slice->header.slice_sao_chroma_flag;
                sh.slice_sample_adaptive_offset_flag[2] = slice->header.slice_sao_chroma_flag;
                
                /* 复制去块滤波相关参数 */
                if (slice->header.deblocking_filter_override_flag) {
                    sh.disable_deblocking_filter_flag = slice->header.slice_deblocking_filter_disabled_flag;
                    if (!sh.disable_deblocking_filter_flag) {
                        sh.beta_offset = slice->header.slice_beta_offset_div2 * 2;
                        sh.tc_offset = slice->header.slice_tc_offset_div2 * 2;
                    }
                } else if (pps) {
                    sh.disable_deblocking_filter_flag = pps->disable_dbf;
                    sh.beta_offset = pps->beta_offset;
                    sh.tc_offset = pps->tc_offset;
                }
                
                /* 复制环路滤波标志 */
                sh.slice_loop_filter_across_slices_enabled_flag = 
                    slice->header.slice_loop_filter_across_slices_enabled_flag;
                
                /* 最大合并候选数 */
                sh.max_num_merge_cand = 5 - slice->header.five_minus_max_num_merge_cand;
                
                /* MVD L1零标志 */
                sh.mvd_l1_zero_flag = slice->header.mvd_l1_zero_flag;
                
                /* 复制图像顺序计数 */
                if (sps) {
                    sh.pic_order_cnt_lsb = slice->header.slice_pic_order_cnt_lsb;
                    /* POC计算需要更复杂的处理，这里简化处理 */
                    sh.poc = sh.pic_order_cnt_lsb;
                }
                
                if (s->verbose) {
                    if (sps)
                        av_log(ctx, AV_LOG_DEBUG, "Slice: type=%d, POC=%d, refs L0=%d L1=%d, QP delta=%d\n", 
                               sh.slice_type, sh.poc, sh.nb_refs[0], sh.nb_refs[1], sh.slice_qp_delta);
                }
            }
            break;
            
        default:
            /* 其他类型的NAL单元不需要处理 */
            break;
    }
    
    return ret;
}

static int usvc_transcoder_filter(AVBSFContext *ctx, AVPacket *pkt)
{
    USVCTranscoderContext *s = ctx->priv_data;
    int ret;
    int i;
    AVPacket *new_pkt = NULL;
    
    ret = ff_bsf_get_packet_ref(ctx, pkt);
    if (ret < 0)
        return ret;
    
    /* 使用CBS系统解析HEVC数据包 */
    ret = ff_cbs_read_packet(s->cbc, &s->fragment, pkt);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to parse HEVC packet.\n");
        goto fail;
    }
    
    /* 输出NAL单元信息（如果启用了详细日志） */
    if (s->verbose) {
        av_log(ctx, AV_LOG_INFO, "HEVC packet with %d NAL units\n", s->fragment.nb_units);
    }

    /* 处理每个NAL单元 */
    for (i = 0; i < s->fragment.nb_units; i++) {
        ret = process_nal_unit(s, ctx, &s->fragment.units[i]);
        if (ret < 0)
            goto fail;
    }
    
    /* 创建一个新的数据包来存储修改后的数据 */
    new_pkt = av_packet_alloc();
    if (!new_pkt) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    
    /* 将修改后的NAL单元写回到新的数据包 */
    ret = ff_cbs_write_packet(s->cbc, new_pkt, &s->fragment);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to write packet.\n");
        av_packet_free(&new_pkt);
        goto fail;
    }
    
    /* 复制时间戳和其他元数据 */
    av_packet_copy_props(new_pkt, pkt);
    
    /* 替换原始数据包 */
    av_packet_unref(pkt);
    av_packet_move_ref(pkt, new_pkt);
    av_packet_free(&new_pkt);
    
    /* 重置片段，释放资源 */
    ff_cbs_fragment_reset(&s->fragment);
    
    return 0;
    
fail:
    ff_cbs_fragment_reset(&s->fragment);
    av_packet_unref(pkt);
    return ret;
}

static int usvc_transcoder_init(AVBSFContext *ctx)
{
    USVCTranscoderContext *s = ctx->priv_data;
    int ret;
    
    /* 检查输入是否为HEVC */
    if (ctx->par_in->codec_id != AV_CODEC_ID_HEVC) {
        av_log(ctx, AV_LOG_ERROR, "Input codec is not HEVC.\n");
        return AVERROR(EINVAL);
    }
    
    /* 初始化CBS上下文用于解析HEVC */
    ret = ff_cbs_init(&s->cbc, AV_CODEC_ID_HEVC, ctx);
    if (ret < 0)
        return ret;
    
    /* 初始化HEVC参数集 */
    hevc_ps_init(&s->ps);
    
    /* 输出参数与输入相同 */
    ret = avcodec_parameters_copy(ctx->par_out, ctx->par_in);
    if (ret < 0)
        return ret;
    
    /* 其他初始化 */
    
    return 0;
}

static void usvc_transcoder_close(AVBSFContext *ctx)
{
    USVCTranscoderContext *s = ctx->priv_data;
    
    /* 释放HEVC参数集 */
    ff_hevc_ps_uninit(&s->ps);
    
    /* 释放CBS资源 */
    ff_cbs_fragment_free(&s->fragment);
    ff_cbs_close(&s->cbc);
}

static const AVOption usvc_transcoder_options[] = {
    {
        "verbose", "Enable verbose logging",
        offsetof(USVCTranscoderContext, verbose),
        AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, 0
    },
    { NULL }
};

static const AVClass usvc_transcoder_class = {
    .class_name = "usvc_transcoder_bsf",
    .item_name  = av_default_item_name,
    .option     = usvc_transcoder_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFBitStreamFilter ff_usvc_transcoder_bsf = {
    .p.name         = "usvc_transcoder",
    .p.codec_ids    = (const enum AVCodecID[]){ AV_CODEC_ID_HEVC, AV_CODEC_ID_NONE },
    .p.priv_class   = &usvc_transcoder_class,
    .priv_data_size = sizeof(USVCTranscoderContext),
    .init           = usvc_transcoder_init,
    .filter         = usvc_transcoder_filter,
    .close          = usvc_transcoder_close,
}; 