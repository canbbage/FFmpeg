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

#include "libavutil/log.h"
#include "libavutil/opt.h"

#include "bsf.h"
#include "bsf_internal.h"
#include "cbs.h"
#include "cbs_h265.h"
#include "../hevc/hevc.h"

typedef struct HEVCPassthroughContext {
    const AVClass *class;
    
    // 选项参数
    int verbose;           // 详细日志选项
    
    // 上下文状态
    CodedBitstreamContext *cbc;
    CodedBitstreamFragment fragment;
} HEVCPassthroughContext;

static int hevc_passthrough_filter(AVBSFContext *ctx, AVPacket *pkt)
{
    HEVCPassthroughContext *s = ctx->priv_data;
    int ret;
    
    ret = ff_bsf_get_packet_ref(ctx, pkt);
    if (ret < 0)
        return ret;
        
    // 使用 CBS 系统解析 HEVC 数据包
    ret = ff_cbs_read_packet(s->cbc, &s->fragment, pkt);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to parse HEVC packet.\n");
        goto fail;
    }
    
    // 输出 NAL 单元信息（如果启用了详细日志）
    if (s->verbose) {
        av_log(ctx, AV_LOG_INFO, "HEVC packet with %d NAL units\n", s->fragment.nb_units);
    }
    
    // 处理每个 NAL 单元
    for (int i = 0; i < s->fragment.nb_units; i++) {
        CodedBitstreamUnit *unit = &s->fragment.units[i];
        
        if (s->verbose) {
            // 根据 NAL 类型输出更详细的信息
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
        
        // 这里可以根据 NAL 类型添加不同的处理逻辑
        // 以下是一些示例处理方式
        switch (unit->type) {
            case HEVC_NAL_VPS:
                // 假设修改 VPS
                // H265RawVPS *vps = (H265RawVPS *)unit->unit_data;
                // 在这里修改 VPS 参数
                break;
                
            case HEVC_NAL_SPS:
                // 假设修改 SPS
                // H265RawSPS *sps = (H265RawSPS *)unit->unit_data;
                // 在这里修改 SPS 参数，例如修改分辨率、编码参数等
                break;
                
            case HEVC_NAL_PPS:
                // 假设修改 PPS
                // H265RawPPS *pps = (H265RawPPS *)unit->unit_data;
                // 在这里修改 PPS 参数
                break;
                
            case HEVC_NAL_SEI_PREFIX:
            case HEVC_NAL_SEI_SUFFIX:
                // 假设修改或添加 SEI 消息
                // H265RawSEI *sei = (H265RawSEI *)unit->unit_data;
                // 在这里修改 SEI 消息
                break;
                
            case HEVC_NAL_IDR_W_RADL:
            case HEVC_NAL_IDR_N_LP:
            case HEVC_NAL_TRAIL_R:
            case HEVC_NAL_TRAIL_N:
                // 假设修改片头或片数据
                // H265RawSlice *slice = (H265RawSlice *)unit->unit_data;
                // 在这里修改片参数
                break;
                
            default:
                // 其他类型的 NAL 单元保持不变
                break;
        }
    }
    
    // 创建一个新的数据包来存储修改后的数据
    AVPacket *new_pkt = NULL;
    new_pkt = av_packet_alloc();
    if (!new_pkt) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    
    // 将修改后的 NAL 单元写回到新的数据包
    ret = ff_cbs_write_packet(s->cbc, new_pkt, &s->fragment);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to write packet.\n");
        av_packet_free(&new_pkt);
        goto fail;
    }
    
    // 复制时间戳和其他元数据
    av_packet_copy_props(new_pkt, pkt);
    
    // 替换原始数据包
    av_packet_unref(pkt);
    av_packet_move_ref(pkt, new_pkt);
    av_packet_free(&new_pkt);
    
    // 重置片段，释放资源
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
    
    // 检查输入是否为HEVC
    if (ctx->par_in->codec_id != AV_CODEC_ID_HEVC) {
        av_log(ctx, AV_LOG_ERROR, "Input codec is not HEVC.\n");
        return AVERROR(EINVAL);
    }
    
    if (s->verbose) {
        // 如果启用了详细日志，初始化CBS上下文用于解析HEVC
        ret = ff_cbs_init(&s->cbc, AV_CODEC_ID_HEVC, ctx);
        if (ret < 0)
            return ret;
    }
    
    // 输出参数与输入相同
    ret = avcodec_parameters_copy(ctx->par_out, ctx->par_in);
    if (ret < 0)
        return ret;
    
    return 0;
}

static void hevc_passthrough_close(AVBSFContext *ctx)
{
    HEVCPassthroughContext *s = ctx->priv_data;
    
    if (s->verbose) {
        ff_cbs_fragment_free(&s->fragment);
        ff_cbs_close(&s->cbc);
    }
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