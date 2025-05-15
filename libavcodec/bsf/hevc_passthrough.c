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
#include "../hevc/hevc.h"

typedef struct HEVCPassthroughContext {
    const AVClass *class;
    
    int verbose;           /* 详细日志选项 */
    
    CodedBitstreamContext *cbc;
    CodedBitstreamFragment fragment;

    int have_B_frame;
    
    /* POC跟踪 */
    int real_poc;          /* 当前真实POC值（始终递增） */
    int last_poc_lsb;      /* 上一帧的POC LSB */
    int max_poc_lsb;       /* MaxPicOrderCntLsb */
    
    /* 上一个slice的关键信息用于检测重复帧 */
    int last_slice_type;
    int last_slice_pic_order_cnt_lsb;
    int last_slice_pic_parameter_set_id;
    
    /* 用于保存PPS原始的output_flag_present_flag值 */
    int original_output_flag_present[HEVC_MAX_PPS_COUNT];
    
    /* 简化的参考帧跟踪 */
    uint8_t ref_frames[16];  /* 标记最近几帧是否为复制帧 (0=原始帧,1=复制帧) */
    int ref_head;          /* 当前位置指针 */
} HEVCPassthroughContext;

/* 函数声明 */
static int is_duplicate_slice(HEVCPassthroughContext *s, const H265RawSliceHeader *slice);
static void modify_strps_for_duplicate(HEVCPassthroughContext *s, AVBSFContext *ctx, H265RawSlice *slice);
static void fix_normal_frame_strps(HEVCPassthroughContext *s, AVBSFContext *ctx, H265RawSlice *slice);

/* 检查是否是重复帧（根据slice header的关键字段） */
static int is_duplicate_slice(HEVCPassthroughContext *s, const H265RawSliceHeader *slice) {
    /* 如果尚未处理任何帧，则这不可能是重复帧 */
    if (s->last_slice_pic_order_cnt_lsb < 0)
        return 0;
    
    /* 检查关键字段是否匹配 */
    /* 重复帧将具有与上一帧相同的POC、slice类型和PPS ID */
    if (slice->slice_pic_order_cnt_lsb == s->last_slice_pic_order_cnt_lsb &&
        slice->slice_type == s->last_slice_type &&
        slice->slice_pic_parameter_set_id == s->last_slice_pic_parameter_set_id) {
        return 1;
    }
    
    return 0;
}

/* 在参考帧列表中记录当前帧 */
static void update_ref_list(HEVCPassthroughContext *s, AVBSFContext *ctx, int is_duplicate) {
    /* 记录当前帧 */
    s->ref_frames[s->ref_head] = 0;

    // 如果当前帧是复制帧，则将前一帧标记为1，即跳过参考
    s->ref_frames[(s->ref_head - 1 + 16) % 16] = is_duplicate;
    /* 更新位置指针 */
    s->ref_head = (s->ref_head + 1) % 16;
    
    if (s->verbose) {
        char ref_list[17] = {0};
        int i;
        for (i = 0; i < 16; i++) {
            int idx = (s->ref_head - i - 1 + 16) % 16;
            ref_list[i] = s->ref_frames[idx] ? '1' : '0';
        }
        av_log(ctx, AV_LOG_DEBUG, "Updated ref list: %s\n", ref_list);
    }
}

/* 修改复制帧的STRPS */
static void modify_strps_for_duplicate(HEVCPassthroughContext *s, AVBSFContext *ctx, H265RawSlice *slice) {
    H265RawSliceHeader *header = &slice->header;
    H265RawSTRefPicSet *st_ref_pic_set = NULL;
    CodedBitstreamH265Context *h265 = s->cbc->priv_data;
    
    /* I帧没有参考帧，直接返回 */
    if (header->slice_type == HEVC_SLICE_I)
        return;
    
    /* 检查STRPS是在SPS中还是在slice中 */
    if (header->short_term_ref_pic_set_sps_flag) {
        /* 从SPS中获取STRPS */
        if (!h265 || !h265->active_sps) {
            av_log(ctx, AV_LOG_ERROR, "Active SPS not found, cannot modify STRPS\n");
            return;
        }
        
        /* 复制SPS中的STRPS到slice header */
        int idx = header->short_term_ref_pic_set_idx;
        if (idx >= h265->active_sps->num_short_term_ref_pic_sets) {
            av_log(ctx, AV_LOG_ERROR, "Invalid STRPS index %d (max %d)\n", 
                   idx, h265->active_sps->num_short_term_ref_pic_sets);
            return;
        }
        
        /* 复制SPS中对应的STRPS */
        memcpy(&header->short_term_ref_pic_set, 
               &h265->active_sps->st_ref_pic_set[idx], 
               sizeof(H265RawSTRefPicSet));
        
        /* 将标志设为0，表示我们现在使用slice中的STRPS */
        header->short_term_ref_pic_set_sps_flag = 0;
        
        if (s->verbose) {
            av_log(ctx, AV_LOG_DEBUG, "Copied STRPS from SPS[%d] to slice header\n", idx);
        }
    }
    
    /* 现在可以安全地修改slice中的STRPS */
    st_ref_pic_set = &header->short_term_ref_pic_set;
    
    /* 分析当前STRPS，看看它参考哪些帧，然后在第一位添加对原始帧的引用(但不使用) */
    int i;
    
    /* 禁用STRPS预测 */
    st_ref_pic_set->inter_ref_pic_set_prediction_flag = 0;
    
    /* 统计原始参考数 */
    int orig_neg_count = st_ref_pic_set->num_negative_pics;
    /* 保存原始的deltaPOC值和used标志 */
    uint16_t orig_delta_poc[HEVC_MAX_REFS];
    uint8_t orig_used[HEVC_MAX_REFS];
    
    for (i = 0; i < orig_neg_count; i++) {
        orig_delta_poc[i] = st_ref_pic_set->delta_poc_s0_minus1[i];
        orig_used[i] = st_ref_pic_set->used_by_curr_pic_s0_flag[i];
    }
    orig_delta_poc[0] = orig_delta_poc[0] + 1;
    // /* 重新组织参考关系 */
    // /* 首先添加对原始帧的引用(delta=0)并标记为不使用 */
    // st_ref_pic_set->delta_poc_s0_minus1[0] = 0;  /* delta=0 */
    // st_ref_pic_set->used_by_curr_pic_s0_flag[0] = 0; /* 不使用 */
    
    /* 然后添加原来的参考 */
    for (i = 0; i < orig_neg_count; i++) {
        if(s->ref_frames[(s->ref_head - i - 2 + 16) % 16] == 1){
            st_ref_pic_set->delta_poc_s0_minus1[i] = orig_delta_poc[i] + 1;
        }
        else
        {
            st_ref_pic_set->delta_poc_s0_minus1[i] = orig_delta_poc[i];
        }
        st_ref_pic_set->used_by_curr_pic_s0_flag[i] = orig_used[i];
    }
    
    if (s->verbose) {
        av_log(ctx, AV_LOG_DEBUG, "Modified STRPS for duplicate frame: added reference to original frame\n");
    }
    st_ref_pic_set->num_negative_pics = 4;
}

/* 修复普通帧的STRPS，确保正确的参考关系 */
static void fix_normal_frame_strps(HEVCPassthroughContext *s, AVBSFContext *ctx, H265RawSlice *slice) {
    H265RawSliceHeader *header = &slice->header;
    H265RawSTRefPicSet *st_ref_pic_set = NULL;
    CodedBitstreamH265Context *h265 = s->cbc->priv_data;
    
    /* I帧没有参考帧，直接返回 */
    if (header->slice_type == HEVC_SLICE_I)
        return;
    
    /* 检查STRPS是在SPS中还是在slice中 */
    if (header->short_term_ref_pic_set_sps_flag) {
        /* 从SPS中获取STRPS */
        if (!h265 || !h265->active_sps) {
            av_log(ctx, AV_LOG_ERROR, "Active SPS not found, cannot modify STRPS\n");
            return;
        }
        
        /* 复制SPS中的STRPS到slice header */
        int idx = header->short_term_ref_pic_set_idx;
        if (idx >= h265->active_sps->num_short_term_ref_pic_sets) {
            av_log(ctx, AV_LOG_ERROR, "Invalid STRPS index %d (max %d)\n", 
                   idx, h265->active_sps->num_short_term_ref_pic_sets);
            return;
        }
        
        /* 复制SPS中对应的STRPS */
        memcpy(&header->short_term_ref_pic_set, 
               &h265->active_sps->st_ref_pic_set[idx], 
               sizeof(H265RawSTRefPicSet));
        
        /* 将标志设为0，表示我们现在使用slice中的STRPS */
        header->short_term_ref_pic_set_sps_flag = 0;
        
        if (s->verbose) {
            av_log(ctx, AV_LOG_DEBUG, "Copied STRPS from SPS[%d] to slice header\n", idx);
        }
    }
    
    /* 获取STRPS */
    st_ref_pic_set = &header->short_term_ref_pic_set;
    /* 统计原始参考数 */
    int orig_neg_count = st_ref_pic_set->num_negative_pics;

    /* 检查前面的帧是否有复制帧 */
    int has_dup = 0;
    for (int i = 0; i < orig_neg_count + 1; i++) {
        int idx = (s->ref_head - i - 1 + 16) % 16;
        if (s->ref_frames[idx] == 1) {
            /* 前面有复制帧 */
            has_dup = 1;
            break;
        }
    }
    
    /* 如果前面没有复制帧，不需要修改 */
    if (!has_dup)
        return;
    
    /* 前面有复制帧，需要修改当前帧的STRPS */
    
    /* 禁用STRPS预测 */
    st_ref_pic_set->inter_ref_pic_set_prediction_flag = 0;
    
    /* 保存原始的deltaPOC值和used标志 */
    uint16_t orig_delta_poc[HEVC_MAX_REFS];
    uint8_t orig_used[HEVC_MAX_REFS];
    
    int i;
    for (i = 0; i < orig_neg_count; i++) {
        orig_delta_poc[i] = st_ref_pic_set->delta_poc_s0_minus1[i];
        orig_used[i] = st_ref_pic_set->used_by_curr_pic_s0_flag[i];
    }
    
    /* 然后添加原来的参考 */
    for (i = 0; i < orig_neg_count; i++) {
        if(s->ref_frames[(s->ref_head - i - 1 + 16) % 16] == 1)
        {
            st_ref_pic_set->delta_poc_s0_minus1[i] = orig_delta_poc[i] + 1;
        }
        else
        {
            st_ref_pic_set->delta_poc_s0_minus1[i] = orig_delta_poc[i];
        }
        st_ref_pic_set->used_by_curr_pic_s0_flag[i] = orig_used[i];
    }
    
    /* 不需要特别处理temporal MV - 普通帧应该正常参考其前一帧 */
    /* 即使前一帧是复制帧，也应该保持正常的参考关系 */
    /* 原始编码器设置的temporal MV参考应该已经是正确的 */
    
    st_ref_pic_set->num_negative_pics = 4;
}

static int hevc_passthrough_filter(AVBSFContext *ctx, AVPacket *pkt)
{
    HEVCPassthroughContext *s = ctx->priv_data;
    int ret;
    int current_poc_lsb;
    int new_poc_lsb;
    int is_duplicate;
    int i;
    AVPacket *new_pkt = NULL;
    H265RawSlice *slice = NULL;
    CodedBitstreamH265Context *h265;
    
    ret = ff_bsf_get_packet_ref(ctx, pkt);
    if (ret < 0)
        return ret;
        
    /* 如果已知有B帧，直接跳过处理返回当前包 */
    if(s->have_B_frame) {
        av_log(ctx, AV_LOG_DEBUG, "B frame detected, passing through without processing\n");
        return 0;
    }
    
    /* 使用 CBS 系统解析 HEVC 数据包 */
    ret = ff_cbs_read_packet(s->cbc, &s->fragment, pkt);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to parse HEVC packet.\n");
        goto fail;
    }
    
    /* 获取H265上下文 */
    h265 = s->cbc->priv_data;
    
    /* 输出 NAL 单元信息（如果启用了详细日志） */
    if (s->verbose) {
        av_log(ctx, AV_LOG_INFO, "HEVC packet with %d NAL units\n", s->fragment.nb_units);
    }

    /* 处理每个 NAL 单元 */
    for (i = 0; i < s->fragment.nb_units; i++) {
        CodedBitstreamUnit *unit = &s->fragment.units[i];
        
        if (s->verbose) {
            /* 根据 NAL 类型输出更详细的信息 */
            switch (unit->type) {
                case HEVC_NAL_VPS:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: VPS\n", i);
                    break;
                case HEVC_NAL_SPS:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: SPS\n", i);
                    break;
                case HEVC_NAL_PPS:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: PPS\n", i);
                    break;
                case HEVC_NAL_IDR_W_RADL:
                case HEVC_NAL_IDR_N_LP:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: IDR slice\n", i);
                    break;
                case HEVC_NAL_TRAIL_R:
                case HEVC_NAL_TRAIL_N:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: Trail slice\n", i);
                    break;
                case HEVC_NAL_SEI_PREFIX:
                case HEVC_NAL_SEI_SUFFIX:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: SEI\n", i);
                    break;
                default:
                    av_log(ctx, AV_LOG_DEBUG, "  NAL %d: type %d\n", i, unit->type);
                    break;
            }
        }
        
        /* 处理NAL单元，检测和修改重复POC */
        switch (unit->type) {
            case HEVC_NAL_VPS:
                {
                    /* 修改VPS中的max_dec_pic_buffering_minus1参数 */
                    H265RawVPS *vps = (H265RawVPS *)unit->content;
                    if (vps) {
                        int vps_id = vps->vps_video_parameter_set_id;
                        
                        /* 设置sub_layer_ordering_info_present_flag = 1，为所有层级设置独立值 */
                        vps->vps_sub_layer_ordering_info_present_flag = 1;
                        
                        /* 为所有层级设置参数 */
                        for (int j = 0; j <= vps->vps_max_sub_layers_minus1; j++) {
                            /* 确保max_dec_pic_buffering_minus1至少为4 */
                            if (vps->vps_max_dec_pic_buffering_minus1[j] < 4) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "VPS %d: Increasing max_dec_pic_buffering_minus1[%d] from %d to 4\n", 
                                           vps_id, j, vps->vps_max_dec_pic_buffering_minus1[j]);
                                }
                                vps->vps_max_dec_pic_buffering_minus1[j] = 4;
                            }
                            
                            /* 确保num_reorder_pics不超过max_dec_pic_buffering_minus1 */
                            if (vps->vps_max_num_reorder_pics[j] > vps->vps_max_dec_pic_buffering_minus1[j]) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "VPS %d: Adjusting vps_max_num_reorder_pics[%d] from %d to %d\n", 
                                           vps_id, j, vps->vps_max_num_reorder_pics[j], vps->vps_max_dec_pic_buffering_minus1[j]);
                                }
                                vps->vps_max_num_reorder_pics[j] = vps->vps_max_dec_pic_buffering_minus1[j];
                            }
                        }
                        
                        /* 修改内部缓存的VPS（重要）*/
                        if (h265) {
                            /* 修改对应ID的VPS */
                            if (h265->vps[vps_id]) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "Also updating internal VPS cache for id=%d\n", vps_id);
                                }
                                /* 设置sub_layer_ordering_info_present_flag = 1 */
                                h265->vps[vps_id]->vps_sub_layer_ordering_info_present_flag = 1;
                                
                                /* 更新所有层级的参数 */
                                for (int j = 0; j <= h265->vps[vps_id]->vps_max_sub_layers_minus1; j++) {
                                    /* 更新max_dec_pic_buffering_minus1 */
                                    if (h265->vps[vps_id]->vps_max_dec_pic_buffering_minus1[j] < 4) {
                                        h265->vps[vps_id]->vps_max_dec_pic_buffering_minus1[j] = 4;
                                    }
                                    
                                    /* 更新num_reorder_pics */
                                    if (h265->vps[vps_id]->vps_max_num_reorder_pics[j] > h265->vps[vps_id]->vps_max_dec_pic_buffering_minus1[j]) {
                                        h265->vps[vps_id]->vps_max_num_reorder_pics[j] = h265->vps[vps_id]->vps_max_dec_pic_buffering_minus1[j];
                                    }
                                }
                            }
                            
                            /* 如果有活动VPS也一同更新 */
                            if (h265->active_vps && h265->active_vps->vps_video_parameter_set_id == vps_id) {
                                const H265RawVPS *active_vps = h265->active_vps;
                                /* 由于active_vps是const指针，需要使用强制类型转换 */
                                H265RawVPS *mutable_vps = (H265RawVPS *)active_vps;
                                
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "Updating active VPS\n");
                                }
                                
                                /* 设置sub_layer_ordering_info_present_flag = 1 */
                                mutable_vps->vps_sub_layer_ordering_info_present_flag = 1;
                                
                                /* 更新所有层级的参数 */
                                for (int j = 0; j <= mutable_vps->vps_max_sub_layers_minus1; j++) {
                                    /* 更新max_dec_pic_buffering_minus1 */
                                    if (mutable_vps->vps_max_dec_pic_buffering_minus1[j] < 4) {
                                        mutable_vps->vps_max_dec_pic_buffering_minus1[j] = 4;
                                    }
                                    
                                    /* 更新num_reorder_pics */
                                    if (mutable_vps->vps_max_num_reorder_pics[j] > mutable_vps->vps_max_dec_pic_buffering_minus1[j]) {
                                        mutable_vps->vps_max_num_reorder_pics[j] = mutable_vps->vps_max_dec_pic_buffering_minus1[j];
                                    }
                                }
                            }
                        }
                    }
                }
                break;
                
            case HEVC_NAL_SPS:
                {
                    /* 从SPS获取MaxPicOrderCntLsb */
                    H265RawSPS *sps = (H265RawSPS *)unit->content;
                    if (sps) {
                        int sps_id = sps->sps_seq_parameter_set_id;
                        
                        /* log2_max_pic_order_cnt_lsb_minus4 + 4 计算 log2_max_pic_order_cnt_lsb */
                        int log2_max_poc_lsb = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
                        s->max_poc_lsb = 1 << log2_max_poc_lsb;
                        
                        /* 设置sub_layer_ordering_info_present_flag = 1 */
                        sps->sps_sub_layer_ordering_info_present_flag = 1;
                        
                        /* 为所有层级设置参数 */
                        for (int j = 0; j <= sps->sps_max_sub_layers_minus1; j++) {
                            /* 确保max_dec_pic_buffering_minus1至少为4 */
                            if (sps->sps_max_dec_pic_buffering_minus1[j] < 4) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "SPS %d: Increasing sps_max_dec_pic_buffering_minus1[%d] from %d to 4\n", 
                                           sps_id, j, sps->sps_max_dec_pic_buffering_minus1[j]);
                                }
                                sps->sps_max_dec_pic_buffering_minus1[j] = 4;
                            }
                            
                            /* 确保num_reorder_pics不超过max_dec_pic_buffering_minus1 */
                            if (sps->sps_max_num_reorder_pics[j] > sps->sps_max_dec_pic_buffering_minus1[j]) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "SPS %d: Adjusting sps_max_num_reorder_pics[%d] from %d to %d\n", 
                                           sps_id, j, sps->sps_max_num_reorder_pics[j], sps->sps_max_dec_pic_buffering_minus1[j]);
                                }
                                sps->sps_max_num_reorder_pics[j] = sps->sps_max_dec_pic_buffering_minus1[j];
                            }
                        }
                        
                        /* 修改内部缓存的SPS（重要）*/
                        if (h265) {
                            /* 修改对应ID的SPS */
                            if (h265->sps[sps_id]) {
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "Also updating internal SPS cache for id=%d\n", sps_id);
                                }
                                /* 设置sub_layer_ordering_info_present_flag = 1 */
                                h265->sps[sps_id]->sps_sub_layer_ordering_info_present_flag = 1;
                                
                                /* 更新所有层级的参数 */
                                for (int j = 0; j <= h265->sps[sps_id]->sps_max_sub_layers_minus1; j++) {
                                    /* 更新max_dec_pic_buffering_minus1 */
                                    if (h265->sps[sps_id]->sps_max_dec_pic_buffering_minus1[j] < 4) {
                                        h265->sps[sps_id]->sps_max_dec_pic_buffering_minus1[j] = 4;
                                    }
                                    
                                    /* 更新num_reorder_pics */
                                    if (h265->sps[sps_id]->sps_max_num_reorder_pics[j] > h265->sps[sps_id]->sps_max_dec_pic_buffering_minus1[j]) {
                                        h265->sps[sps_id]->sps_max_num_reorder_pics[j] = h265->sps[sps_id]->sps_max_dec_pic_buffering_minus1[j];
                                    }
                                }
                            }
                            
                            /* 如果有活动SPS也一同更新 */
                            if (h265->active_sps && h265->active_sps->sps_seq_parameter_set_id == sps_id) {
                                const H265RawSPS *active_sps = h265->active_sps;
                                /* 由于active_sps是const指针，需要使用强制类型转换 */
                                H265RawSPS *mutable_sps = (H265RawSPS *)active_sps;
                                
                                if (s->verbose) {
                                    av_log(ctx, AV_LOG_DEBUG, "Updating active SPS\n");
                                }
                                
                                /* 设置sub_layer_ordering_info_present_flag = 1 */
                                mutable_sps->sps_sub_layer_ordering_info_present_flag = 1;
                                
                                /* 更新所有层级的参数 */
                                for (int j = 0; j <= mutable_sps->sps_max_sub_layers_minus1; j++) {
                                    /* 更新max_dec_pic_buffering_minus1 */
                                    if (mutable_sps->sps_max_dec_pic_buffering_minus1[j] < 4) {
                                        mutable_sps->sps_max_dec_pic_buffering_minus1[j] = 4;
                                    }
                                    
                                    /* 更新num_reorder_pics */
                                    if (mutable_sps->sps_max_num_reorder_pics[j] > mutable_sps->sps_max_dec_pic_buffering_minus1[j]) {
                                        mutable_sps->sps_max_num_reorder_pics[j] = mutable_sps->sps_max_dec_pic_buffering_minus1[j];
                                    }
                                }
                            }
                        }
                        
                        if (s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "SPS found. MaxPicOrderCntLsb = %d\n", s->max_poc_lsb);
                        }
                    }
                }
                break;
                
            case HEVC_NAL_PPS:
                {
                    H265RawPPS *pps = (H265RawPPS *)unit->content;
                    if (pps) {
                        int pps_id = pps->pps_pic_parameter_set_id;
                        
                        /* 保存PPS原始的output_flag_present_flag值 */
                        s->original_output_flag_present[pps_id] = pps->output_flag_present_flag;
                        
                        if (s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "PPS (id=%d) original output_flag_present_flag=%d\n", 
                                  pps_id, pps->output_flag_present_flag);
                        }
                    }
                }
                break;
                
            case HEVC_NAL_IDR_W_RADL:
            case HEVC_NAL_IDR_N_LP:
                {
                    slice = (H265RawSlice *)unit->content;
                    if (!slice) {
                        break;
                    }
                
                    /* IDR帧会重置POC计数 */
                    s->real_poc = 0;
                    
                    if (s->verbose) {
                        av_log(ctx, AV_LOG_DEBUG, "IDR frame: Resetting POC tracking\n");
                    }
                    
                    /* IDR帧POC应该为0 */
                    if (slice->header.slice_pic_order_cnt_lsb != 0) {
                        av_log(ctx, AV_LOG_DEBUG, "IDR frame: Correcting POC LSB from %d to 0\n", 
                              slice->header.slice_pic_order_cnt_lsb);
                        slice->header.slice_pic_order_cnt_lsb = 0;
                    }
                    
                    /* IDR帧设置pic_output_flag=1确保显示 */
                    slice->header.pic_output_flag = 1;
                    if (s->verbose) {
                        av_log(ctx, AV_LOG_DEBUG, "IDR frame: Setting pic_output_flag to 1\n");
                    }
                    
                    /* 重置跟踪信息 */
                    s->last_poc_lsb = 0;  /* IDR帧后第一帧的POC预期为0 */
                    s->last_slice_type = slice->header.slice_type;
                    s->last_slice_pic_order_cnt_lsb = 0;
                    s->last_slice_pic_parameter_set_id = slice->header.slice_pic_parameter_set_id;
                    
                    /* 清除缓存的任何重复帧状态 */
                    s->have_B_frame = 0;
                    
                    /* 重置参考帧列表 */
                    memset(s->ref_frames, 0, sizeof(s->ref_frames));
                    s->ref_head = 0;
                    
                    /* 将IDR帧记录到参考帧列表 (非复制帧) */
                    update_ref_list(s, ctx, 0);
                }
                break;
                
            case HEVC_NAL_TRAIL_R:
            case HEVC_NAL_TRAIL_N:
                {
                    slice = (H265RawSlice *)unit->content;
                    if (!slice) {
                        break;
                    }
                    
                    /* 检测B帧 */
                    if (slice->header.slice_type == HEVC_SLICE_B) {
                        s->have_B_frame = 1;
                        av_log(ctx, AV_LOG_DEBUG, "B frame detected at NAL %d\n", i);
                        continue;
                    }
                    
                    /* 获取当前帧的POC LSB值 */
                    current_poc_lsb = slice->header.slice_pic_order_cnt_lsb;
                    
                    /* 无论是否重复帧，都基于real_poc计算新的POC LSB值，确保POC连续 */
                    new_poc_lsb = (s->real_poc + 1) % s->max_poc_lsb;
                    
                    /* 检查是否是重复帧 */
                    is_duplicate = is_duplicate_slice(s, &slice->header);
                    
                    if (is_duplicate) {
                        av_log(ctx, AV_LOG_DEBUG, "Duplicate slice detected. Changing POC LSB from %d to %d\n", 
                              current_poc_lsb, new_poc_lsb);
                        
                        /* 重复帧设置pic_output_flag=0不显示 */
                        slice->header.pic_output_flag = 0;
                                        if (s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "Duplicate frame: Setting pic_output_flag to 0\n");
                        }
                        
                        /* 修改复制帧的STRPS */
                        modify_strps_for_duplicate(s, ctx, slice);
                        
                        /* 更新参考帧列表 - 标记为复制帧 */
                        update_ref_list(s, ctx, 1);
                    } else {
                        /* 非重复帧处理 */
                        if (current_poc_lsb != new_poc_lsb && s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "Normal frame: Changing POC LSB from %d to %d\n", 
                                  current_poc_lsb, new_poc_lsb);
                        }
                        
                        /* 普通帧设置pic_output_flag=1确保显示 */
                        slice->header.pic_output_flag = 1;
                        if (s->verbose) {
                            av_log(ctx, AV_LOG_DEBUG, "Normal frame: Setting pic_output_flag to 1\n");
                        }
                        
                        /* 修复普通帧的STRPS */
                        fix_normal_frame_strps(s, ctx, slice);
                        
                        /* 更新参考帧列表 - 标记为原始帧 */
                        update_ref_list(s, ctx, 0);
                    }
                    s->last_slice_pic_order_cnt_lsb = slice->header.slice_pic_order_cnt_lsb;  /* 使用新的POC值更新 */
                    /* 修改POC值 */
                    slice->header.slice_pic_order_cnt_lsb = new_poc_lsb;
                    
                    /* 更新真实POC计数 - 无论是否为重复帧都需要更新 */
                    s->real_poc++;
                    
                    /* 更新跟踪信息 - 无论是否为重复帧都需要更新 */
                    s->last_poc_lsb = new_poc_lsb;
                    s->last_slice_type = slice->header.slice_type;
                    
                    s->last_slice_pic_parameter_set_id = slice->header.slice_pic_parameter_set_id;
                }
                break;
                
            default:
                /* 其他类型的NAL单元不需要处理 */
                break;
        }
    }
    
    /* 如果发现了B帧，可以选择使用原始包或跳过处理 */
    if (s->have_B_frame) {
        av_log(ctx, AV_LOG_DEBUG, "B frame detected, passing through without modifications\n");
        ff_cbs_fragment_reset(&s->fragment);
        return 0;
    }
    
    /* 创建一个新的数据包来存储修改后的数据 */
    new_pkt = av_packet_alloc();
    if (!new_pkt) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    
    /* 修改所有PPS的output_flag_present_flag为1，以便能写入pic_output_flag */
    for (i = 0; i < HEVC_MAX_PPS_COUNT; i++) {
        if (h265->pps[i]) {
            /* 临时修改output_flag_present_flag为1以启用pic_output_flag写入 */
            h265->pps[i]->output_flag_present_flag = 1;
            if (s->verbose) {
                av_log(ctx, AV_LOG_DEBUG, "Temporarily setting PPS (id=%d) output_flag_present_flag to 1\n", i);
            }
        }
    }
    
    /* 将修改后的 NAL 单元写回到新的数据包 */
    ret = ff_cbs_write_packet(s->cbc, new_pkt, &s->fragment);
    
    /* 恢复所有PPS的原始output_flag_present_flag值 */
    for (i = 0; i < HEVC_MAX_PPS_COUNT; i++) {
        if (h265->pps[i]) {
            /* 恢复原始值 */
            h265->pps[i]->output_flag_present_flag = s->original_output_flag_present[i];
            if (s->verbose) {
                av_log(ctx, AV_LOG_DEBUG, "Restored PPS (id=%d) output_flag_present_flag to %d\n", 
                      i, s->original_output_flag_present[i]);
            }
        }
    }
    
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

static int hevc_passthrough_init(AVBSFContext *ctx)
{
    HEVCPassthroughContext *s = ctx->priv_data;
    int ret;
    
    /* 检查输入是否为HEVC */
    if (ctx->par_in->codec_id != AV_CODEC_ID_HEVC) {
        av_log(ctx, AV_LOG_ERROR, "Input codec is not HEVC.\n");
        return AVERROR(EINVAL);
    }
    
    /* 初始化POC跟踪 */
    s->last_poc_lsb = -1;  /* 设置为-1表示尚未处理任何帧 */
    s->max_poc_lsb = 0;    /* 将在解析SPS时更新 */
    s->real_poc = 0;
    
    /* 初始化slice信息 */
    s->last_slice_type = -1;
    s->last_slice_pic_order_cnt_lsb = -1;
    s->last_slice_pic_parameter_set_id = -1;
    
    /* 初始化PPS状态跟踪数组 */
    memset(s->original_output_flag_present, 0, sizeof(s->original_output_flag_present));
    
    /* 初始化参考帧列表 */
    memset(s->ref_frames, 0, sizeof(s->ref_frames));
    s->ref_head = 0;
    
    /* 初始化B帧检测标志 */
    s->have_B_frame = 0;
    
    /* 始终初始化CBS上下文用于解析HEVC */
    ret = ff_cbs_init(&s->cbc, AV_CODEC_ID_HEVC, ctx);
    if (ret < 0)
        return ret;
    
    /* 输出参数与输入相同 */
    ret = avcodec_parameters_copy(ctx->par_out, ctx->par_in);
    if (ret < 0)
        return ret;
    
    return 0;
}

static void hevc_passthrough_close(AVBSFContext *ctx)
{
    HEVCPassthroughContext *s = ctx->priv_data;
    
    /* 始终释放CBS资源 */
    ff_cbs_fragment_free(&s->fragment);
    ff_cbs_close(&s->cbc);
}

static const AVOption hevc_passthrough_options[] = {
    {
        "verbose", "Enable verbose logging",
        offsetof(HEVCPassthroughContext, verbose),
        AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, 0
    },
    { NULL }
};

static const AVClass hevc_passthrough_class = {
    .class_name = "hevc_passthrough_bsf",
    .item_name  = av_default_item_name,
    .option     = hevc_passthrough_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFBitStreamFilter ff_hevc_passthrough_bsf = {
    .p.name         = "hevc_passthrough",
    .p.codec_ids    = (const enum AVCodecID[]){ AV_CODEC_ID_HEVC, AV_CODEC_ID_NONE },
    .p.priv_class   = &hevc_passthrough_class,
    .priv_data_size = sizeof(HEVCPassthroughContext),
    .init           = hevc_passthrough_init,
    .filter         = hevc_passthrough_filter,
    .close          = hevc_passthrough_close,
}; 