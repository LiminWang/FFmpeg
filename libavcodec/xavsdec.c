/*
 * AVS plus decoder using the libxavsplusdec library
 * Copyright (C) 2018 lance.lmwang<lance.lmwang@gmail.com>
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
#include "libavutil/avassert.h"
#include "avcodec.h"
#include "internal.h"
#include "mpeg12data.h"
#include "mpegvideo.h"
#include "libavutil/imgutils.h"
#include "xavs_decoder.h"

typedef struct AVSContext {
    AVCodecContext *avctx;
    void* p_decoder; /* decoder handle */
} AVSContext;

static av_cold int xavs_init(AVCodecContext *avctx) {
    AVSContext *h = avctx->priv_data;
    int i_result;
    
    h->avctx = avctx;
    avctx->pix_fmt= AV_PIX_FMT_YUV420P;

    /* creat decoder */
    i_result = xavs_decoder_create(&h->p_decoder);
    if( h->p_decoder == NULL || i_result < 0 )
    {
        av_log(avctx, AV_LOG_ERROR, "xavs_decoder_create() failed.\n");
        return -1;
    }

    av_log(avctx, AV_LOG_ERROR, "xavs_decoder_create() ok %p.\n", h->p_decoder);

    return 0;
}

static av_cold int xavs_end(AVCodecContext *avctx) {
    AVSContext *h = avctx->priv_data;
    
    xavs_decoder_destroy(h->p_decoder);
    h->p_decoder = NULL;

    return 0;
}

static void xavs_flush(AVCodecContext * avctx)
{
    AVSContext *h = avctx->priv_data;

    xavs_decoder_reset(h->p_decoder);
}


static int xavs_decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                             AVPacket *avpkt)
{
    AVSContext *h      = avctx->priv_data;
    AVFrame *frame = data;
    XAVSFrame avsframe;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int ret;

    *got_frame = 0;
    memset(&avsframe, 0, sizeof(XAVSFrame));
    if (buf_size == 0) {
        if ((ret = xavs_decoder_get_delay_frame(h->p_decoder, got_frame, &avsframe)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "xavs_decoder_get_delay_frame() failed. ret=%d\n", ret);
            xavs_decoder_reset(h->p_decoder);
            return buf_size;
        }
    } else {
        if (ret = xavs_decoder_put_data(h->p_decoder, buf, buf_size) == -1) {
            av_log(avctx, AV_LOG_ERROR, "xavs_decoder_put_data() failed.\n");
            return buf_size;
        }

        if ((ret = xavs_decoder_get_decode_video(h->p_decoder, got_frame, &avsframe)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "xavs_decoder_get_decode_video() failed. ret=%d\n", ret);
            return buf_size;
        }
    }

    if (*got_frame) {
        frame->width = avsframe.nWidth;
        frame->height = avsframe.nHeight;
        frame->key_frame = (avsframe.nFrameType == 0);
        frame->pict_type = avsframe.nFrameType == 0 ? AV_PICTURE_TYPE_I : 
                           avsframe.nFrameType == 1 ? AV_PICTURE_TYPE_P :
                           avsframe.nFrameType == 2 ? AV_PICTURE_TYPE_B :
                           AV_PICTURE_TYPE_NONE;
        frame->interlaced_frame = !avsframe.nFrameCoded;
        frame->top_field_first = avsframe.nTopFieldFirst;
        frame->pts     = avpkt->pts;
        frame->pkt_dts = avpkt->dts;

        ret = ff_set_dimensions(avctx, avsframe.nWidth, avsframe.nHeight);
        if (ret < 0)
            return ret;

        if (ff_get_buffer(avctx, frame, 0) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Unable to allocate buffer\n");
            return AVERROR(ENOMEM);
        }

        av_image_copy(frame->data, frame->linesize, avsframe.data, avsframe.linesize, 
                avctx->pix_fmt, frame->width, frame->height);
        *got_frame = 1;
    }

    return buf_size;
}

AVCodec ff_cavs_decoder = {
    .name           = "cavs",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_CAVS,
    .priv_data_size = sizeof(AVSContext),
    .init           = xavs_init,
    .close          = xavs_end,
    .decode         = xavs_decode_frame,
    .capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
    .flush          = xavs_flush,
    .long_name      = NULL_IF_CONFIG_SMALL("Bravo Chinese AVS(AVS1-P2, JiZhun profile) and (AVS1-P16, Guangdian profile)"),
};
