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
        
    if (s->verbose) {
        // 如果启用了详细日志，解析HEVC NAL单元信息并输出
        ret = ff_cbs_read_packet(s->cbc, &s->fragment, pkt);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to parse HEVC packet.\n");
            goto fail;
        }
        
        av_log(ctx, AV_LOG_INFO, "HEVC packet with %d NAL units\n", 
               s->fragment.nb_units);
               
        for (int i = 0; i < s->fragment.nb_units; i++) {
            CodedBitstreamUnit *unit = &s->fragment.units[i];
            if (unit->type == HEVC_NAL_VPS)
                av_log(ctx, AV_LOG_DEBUG, "  NAL %d: VPS\n", i);
            else if (unit->type == HEVC_NAL_SPS)
                av_log(ctx, AV_LOG_DEBUG, "  NAL %d: SPS\n", i);
            else if (unit->type == HEVC_NAL_PPS)
                av_log(ctx, AV_LOG_DEBUG, "  NAL %d: PPS\n", i);
            else
                av_log(ctx, AV_LOG_DEBUG, "  NAL %d: type %d\n", i, unit->type);
        }
        
        ff_cbs_fragment_reset(&s->fragment);
    }
    
    // 在这里你可以增加其他处理，暂时我们只进行透明通过
    
    return 0;
    
fail:
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