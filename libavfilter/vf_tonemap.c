/*
 * Copyright (c) 2017 Vittorio Giovara <vittorio.giovara@gmail.com>
 * Copyright (c) 2019 Limin Wang <lance.lmwang@gmail.com>
 *
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

/**
 * @file
 * tonemap algorithms
 */

#include <float.h>
#include <stdio.h>
#include <string.h>

#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "colorspace.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

enum TonemapAlgorithm {
    TONEMAP_NONE,
    TONEMAP_LINEAR,
    TONEMAP_GAMMA,
    TONEMAP_CLIP,
    TONEMAP_REINHARD,
    TONEMAP_HABLE,
    TONEMAP_MOBIUS,
    TONEMAP_MAX,
};

static const struct LumaCoefficients luma_coefficients[AVCOL_SPC_NB] = {
    [AVCOL_SPC_FCC]        = { 0.30,   0.59,   0.11   },
    [AVCOL_SPC_BT470BG]    = { 0.299,  0.587,  0.114  },
    [AVCOL_SPC_SMPTE170M]  = { 0.299,  0.587,  0.114  },
    [AVCOL_SPC_BT709]      = { 0.2126, 0.7152, 0.0722 },
    [AVCOL_SPC_SMPTE240M]  = { 0.212,  0.701,  0.087  },
    [AVCOL_SPC_BT2020_NCL] = { 0.2627, 0.6780, 0.0593 },
    [AVCOL_SPC_BT2020_CL]  = { 0.2627, 0.6780, 0.0593 },
};

typedef struct TonemapContext {
    const AVClass *class;

    enum TonemapAlgorithm tonemap;
    double param;
    double desat;
    double peak;

    const struct LumaCoefficients *coeffs;
} TonemapContext;

typedef struct ThreadData {
    AVFrame *in, *out;
    double peak;
    const struct AVPixFmtDescriptor *desc;
    const struct AVPixFmtDescriptor *odesc;
} ThreadData;

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GBRPF32,
    AV_PIX_FMT_GBRAPF32,
    AV_PIX_FMT_NONE,
};

static int query_formats(AVFilterContext *ctx)
{
    return ff_set_common_formats(ctx, ff_make_format_list(pix_fmts));
}

static av_cold int init(AVFilterContext *ctx)
{
    TonemapContext *s = ctx->priv;

    switch(s->tonemap) {
    case TONEMAP_GAMMA:
        if (isnan(s->param))
            s->param = 1.8f;
        break;
    case TONEMAP_REINHARD:
        if (!isnan(s->param))
            s->param = (1.0f - s->param) / s->param;
        break;
    case TONEMAP_MOBIUS:
        if (isnan(s->param))
            s->param = 0.3f;
        break;
    }

    if (isnan(s->param))
        s->param = 1.0f;

    return 0;
}

static float hable(float in)
{
    float a = 0.15f, b = 0.50f, c = 0.10f, d = 0.20f, e = 0.02f, f = 0.30f;
    return (in * (in * a + b * c) + d * e) / (in * (in * a + b) + d * f) - e / f;
}

static float mobius(float in, float j, double peak)
{
    float a, b;

    if (in <= j)
        return in;

    a = -j * j * (peak - 1.0f) / (j * j - 2.0f * j + peak);
    b = (j * j - 2.0f * j * peak + peak) / FFMAX(peak - 1.0f, 1e-6);

    return (b * b + 2.0f * b * j + j * j) / (b - a) * (in + a) / (in + b);
}

#define MIX(x,y,a) (x) * (1 - (a)) + (y) * (a)
static void tonemap(TonemapContext *s, float *r_out, float *b_out, float *g_out,
                    const float *r_in, const float *b_in, const float *g_in,
                    const AVPixFmtDescriptor *desc, double peak)
{
    float sig, sig_orig;

    /* load values */
    *r_out = *r_in;
    *b_out = *b_in;
    *g_out = *g_in;

    /* desaturate to prevent unnatural colors */
    if (s->desat > 0) {
        float luma = s->coeffs->cr * *r_in + s->coeffs->cg * *g_in + s->coeffs->cb * *b_in;
        float overbright = FFMAX(luma - s->desat, 1e-6) / FFMAX(luma, 1e-6);
        *r_out = MIX(*r_in, luma, overbright);
        *g_out = MIX(*g_in, luma, overbright);
        *b_out = MIX(*b_in, luma, overbright);
    }

    /* pick the brightest component, reducing the value range as necessary
     * to keep the entire signal in range and preventing discoloration due to
     * out-of-bounds clipping */
    sig = FFMAX(FFMAX3(*r_out, *g_out, *b_out), 1e-6);
    sig_orig = sig;

    switch(s->tonemap) {
    default:
    case TONEMAP_NONE:
        // do nothing
        break;
    case TONEMAP_LINEAR:
        sig = sig * s->param / peak;
        break;
    case TONEMAP_GAMMA:
        sig = sig > 0.05f ? pow(sig / peak, 1.0f / s->param)
                          : sig * pow(0.05f / peak, 1.0f / s->param) / 0.05f;
        break;
    case TONEMAP_CLIP:
        sig = av_clipf(sig * s->param, 0, 1.0f);
        break;
    case TONEMAP_HABLE:
        sig = hable(sig) / hable(peak);
        break;
    case TONEMAP_REINHARD:
        sig = sig / (sig + s->param) * (peak + s->param) / peak;
        break;
    case TONEMAP_MOBIUS:
        sig = mobius(sig, s->param, peak);
        break;
    }

    /* apply the computed scale factor to the color,
     * linearly to prevent discoloration */
    *r_out *= sig / sig_orig;
    *g_out *= sig / sig_orig;
    *b_out *= sig / sig_orig;

}

static int do_tonemap_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    TonemapContext *s = ctx->priv;
    const ThreadData *td = arg;
    const AVFrame *in = td->in;
    AVFrame *out = td->out;
    int x, y;
    const AVPixFmtDescriptor *desc = td->desc;
    const AVPixFmtDescriptor *odesc = td->odesc;
    const int slice_start = (out->height *  jobnr   ) / nb_jobs;
    const int slice_end   = (out->height * (jobnr+1)) / nb_jobs;
    const int slice_h   = slice_end - slice_start;
    uint8_t *dstr = out->data[0] + slice_start * out->linesize[0];
    uint8_t *dstb = out->data[1] + slice_start * out->linesize[1];
    uint8_t *dstg = out->data[2] + slice_start * out->linesize[2];
    const uint8_t *srcr = in->data[0] + slice_start * in->linesize[0];
    const uint8_t *srcb = in->data[1] + slice_start * in->linesize[1];
    const uint8_t *srcg = in->data[2] + slice_start * in->linesize[2];
    uint8_t *dsta = out->data[3] + slice_start * out->linesize[3];
    const uint8_t *srca = in ->data[3] + slice_start * in->linesize[3];

    /* do the tone map */
    for (y = slice_start; y < slice_end; y++) {
        for (x = 0; x < out->width; x++) {
            const float *r_in = (const float *)(srcr + x * desc->comp[0].step);
            const float *b_in = (const float *)(srcb + x * desc->comp[1].step);
            const float *g_in = (const float *)(srcg + x * desc->comp[2].step);
            float *r_out = (float *)(dstr + x * desc->comp[0].step);
            float *b_out = (float *)(dstb + x * desc->comp[1].step);
            float *g_out = (float *)(dstg + x * desc->comp[2].step);

            tonemap(s, r_out, b_out, g_out, r_in, b_in, g_in, desc, td->peak);
        }
        srcr += in->linesize[0];
        srcg += in->linesize[1];
        srcb += in->linesize[2];
        dstr += out->linesize[0];
        dstg += out->linesize[1];
        dstb += out->linesize[2];
    }

    /* copy/generate alpha if needed */
    if (desc->flags & AV_PIX_FMT_FLAG_ALPHA && odesc->flags & AV_PIX_FMT_FLAG_ALPHA) {
        av_image_copy_plane(dsta, out->linesize[3],
                srca, in->linesize[3],
                out->linesize[3], slice_h);
    } else if (odesc->flags & AV_PIX_FMT_FLAG_ALPHA) {
        for (y = slice_start; y < slice_end; y++) {
            for (x = 0; x < out->width; x++) {
                AV_WN32(dsta + x * odesc->comp[3].step + y * out->linesize[3],
                        av_float2int(1.0f));
            }
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    TonemapContext *s = ctx->priv;
    AVFilterLink *outlink = link->dst->outputs[0];
    AVFrame *out;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(link->format);
    const AVPixFmtDescriptor *odesc = av_pix_fmt_desc_get(outlink->format);
    int ret;
    double peak = s->peak;
    ThreadData td;

    if (!desc || !odesc) {
        av_frame_free(&in);
        return AVERROR_BUG;
    }

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    ret = av_frame_copy_props(out, in);
    if (ret < 0) {
        av_frame_free(&in);
        av_frame_free(&out);
        return ret;
    }

    /* input and output transfer will be linear */
    if (in->color_trc == AVCOL_TRC_UNSPECIFIED) {
        av_log(s, AV_LOG_WARNING, "Untagged transfer, assuming linear light\n");
        out->color_trc = AVCOL_TRC_LINEAR;
    } else if (in->color_trc != AVCOL_TRC_LINEAR)
        av_log(s, AV_LOG_WARNING, "Tonemapping works on linear light only\n");

    /* read peak from side data if not passed in */
    if (!peak) {
        peak = ff_determine_signal_peak(in);
        av_log(s, AV_LOG_DEBUG, "Computed signal peak: %f\n", peak);
    }

    /* load original color space even if pixel format is RGB to compute overbrights */
    s->coeffs = &luma_coefficients[in->colorspace];
    if (s->desat > 0 && (in->colorspace == AVCOL_SPC_UNSPECIFIED || !s->coeffs)) {
        if (in->colorspace == AVCOL_SPC_UNSPECIFIED)
            av_log(s, AV_LOG_WARNING, "Missing color space information, ");
        else if (!s->coeffs)
            av_log(s, AV_LOG_WARNING, "Unsupported color space '%s', ",
                   av_color_space_name(in->colorspace));
        av_log(s, AV_LOG_WARNING, "desaturation is disabled\n");
        s->desat = 0;
    }

    td.in = in;
    td.out = out;
    td.desc = desc;
    td.odesc = odesc;
    td.peak = peak;
    ctx->internal->execute(ctx, do_tonemap_slice, &td, NULL, FFMIN(outlink->h, ff_filter_get_nb_threads(ctx)));

    av_frame_free(&in);

    ff_update_hdr_metadata(out, peak);

    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(TonemapContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
static const AVOption tonemap_options[] = {
    { "tonemap",      "tonemap algorithm selection", OFFSET(tonemap), AV_OPT_TYPE_INT, {.i64 = TONEMAP_NONE}, TONEMAP_NONE, TONEMAP_MAX - 1, FLAGS, "tonemap" },
    {     "none",     0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_NONE},              0, 0, FLAGS, "tonemap" },
    {     "linear",   0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_LINEAR},            0, 0, FLAGS, "tonemap" },
    {     "gamma",    0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_GAMMA},             0, 0, FLAGS, "tonemap" },
    {     "clip",     0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_CLIP},              0, 0, FLAGS, "tonemap" },
    {     "reinhard", 0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_REINHARD},          0, 0, FLAGS, "tonemap" },
    {     "hable",    0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_HABLE},             0, 0, FLAGS, "tonemap" },
    {     "mobius",   0, 0, AV_OPT_TYPE_CONST, {.i64 = TONEMAP_MOBIUS},            0, 0, FLAGS, "tonemap" },
    { "param",        "tonemap parameter", OFFSET(param), AV_OPT_TYPE_DOUBLE, {.dbl = NAN}, DBL_MIN, DBL_MAX, FLAGS },
    { "desat",        "desaturation strength", OFFSET(desat), AV_OPT_TYPE_DOUBLE, {.dbl = 2}, 0, DBL_MAX, FLAGS },
    { "peak",         "signal peak override", OFFSET(peak), AV_OPT_TYPE_DOUBLE, {.dbl = 0}, 0, DBL_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(tonemap);

static const AVFilterPad tonemap_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

static const AVFilterPad tonemap_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_tonemap = {
    .name            = "tonemap",
    .description     = NULL_IF_CONFIG_SMALL("Conversion to/from different dynamic ranges."),
    .init            = init,
    .query_formats   = query_formats,
    .priv_size       = sizeof(TonemapContext),
    .priv_class      = &tonemap_class,
    .inputs          = tonemap_inputs,
    .outputs         = tonemap_outputs,
    .flags           = AVFILTER_FLAG_SLICE_THREADS,
};
