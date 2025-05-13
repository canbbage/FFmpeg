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
} HEVCPassthroughContext;

/* 函数声明 */
static int is_duplicate_slice(HEVCPassthroughContext *s, const H265RawSliceHeader *slice);

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
            case HEVC_NAL_SPS:
                {
                    /* 从SPS获取MaxPicOrderCntLsb */
                    H265RawSPS *sps = (H265RawSPS *)unit->content;
                    if (sps) {
                        /* log2_max_pic_order_cnt_lsb_minus4 + 4 计算 log2_max_pic_order_cnt_lsb */
                        int log2_max_poc_lsb = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
                        s->max_poc_lsb = 1 << log2_max_poc_lsb;
                        
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
    int i;
    
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
    for (i = 0; i < HEVC_MAX_PPS_COUNT; i++) {
        s->original_output_flag_present[i] = 0;
    }
    
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