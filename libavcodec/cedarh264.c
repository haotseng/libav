/*
 * H.264 encoding using the cedarx library
 * Copyright (C) 2014 Alcantor <alcantor@hotmail.com>
 */

#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/stereo3d.h"
#include "avcodec.h"
#include "internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*#define LOG_NDEBUG 0
#define LOG_TAG "venc-file"
#include "../sunxicedar/include/CDX_Debug.h"*/
#include "../sunxicedar/include/include_vencoder/venc.h"
#include "../sunxicedar/include/include_system/cedarx_hardware.h"

typedef struct cedarh264Context {
       AVClass        *class;
       cedarv_encoder_t *venc_device;
       int64_t framecnt;
} cedarh264Context;

static void fillBufferInterleavedChroma(const AVFrame *frame, enum AVPixelFormat pix_fmt, unsigned char * dstY, unsigned char * dstC){
       int i, j, size[4] = { 0 }, has_plane[4] = { 0 }, nb_chromaplane = 0;
       const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);

       for (i = 0; i < 4; i++){
               int h, s = (i == 1 || i == 2) ? desc->log2_chroma_h : 0;
               has_plane[desc->comp[i].plane] = 1;
                       h = (frame->height + (1 << s) - 1) >> s;
               size[i] = h * frame->linesize[i];
       }

       for (i = 1; i < 4; i++) if(has_plane[i]) ++nb_chromaplane;

       if(has_plane[0]) memcpy(dstY, frame->data[0], size[0]);

       for (i = 1; i < 4 && has_plane[i]; i++) {
               for(j = 0; j < size[i]; ++j)
                       dstC[(j*nb_chromaplane)+i-1] = ++frame->data[i][j];
       }
}

static int cedarh264_frame(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame,
                      int *got_packet)
{
       cedarh264Context *c4 = avctx->priv_data;

       /* Cedar variables */
       cedarv_encoder_t *venc_device = c4->venc_device;
       VencInputBuffer input_buffer;
       VencOutputBuffer output_buffer;
       int result;
       *got_packet = 0;

       /* Don't try to read empty frame */
       if(frame == NULL) return 0;

       /* Get alloc buffer */
       while(1) {
               result = c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_GET_ALLOCATE_INPUT_BUFFER, &input_buffer);
               if(result == 0) {
                       av_log(avctx, AV_LOG_DEBUG, "Get alloc id copy to encoder, input_buffer: %x\n", input_buffer.id);

                       /* Copy data */
                       fillBufferInterleavedChroma(frame, avctx->pix_fmt, input_buffer.addrvirY, input_buffer.addrvirC);
                       /*result = avpicture_get_size(avctx->pix_fmt, avctx->width, avctx->height);
                       if(result < 0) return result;
                       result = avpicture_layout((const AVPicture *)frame, avctx->pix_fmt,
                               avctx->width, avctx->height, input_buffer.addrvirY, result); //frame->size);*/

                       /* Flush */
                       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_FLUSHCACHE_ALLOCATE_INPUT_BUFFER, &input_buffer);
                       break;
               }

               av_log(avctx, AV_LOG_WARNING, "No alloc input buffer right now\n");
               usleep(10*1000);
       }

       /* Encode */
       result = venc_device->ioctrl(venc_device, VENC_CMD_ENCODE, &input_buffer);
       av_log(avctx, AV_LOG_DEBUG, "Encoder result: %d, input_buffer.id: %d\n", result, input_buffer.id);

       /* Return the buffer to the alloc buffer quene after encoder */
       venc_device->ioctrl(venc_device, VENC_CMD_RETURN_ALLOCATE_INPUT_BUFFER, &input_buffer);
       if(result == 0) {
               memset(&output_buffer, 0, sizeof(VencOutputBuffer));
               result = venc_device->ioctrl(venc_device, VENC_CMD_GET_BITSTREAM, &output_buffer);
               if(result == 0) {
                       //av_log(avctx, AV_LOG_DEBUG, "Alloc packet\n");
                       result = ff_alloc_packet(pkt, output_buffer.size0+output_buffer.size1);
                       if(result < 0) return result;
                       //av_log(avctx, AV_LOG_DEBUG, "Copy from encoder\n");
                       memcpy(pkt->data, output_buffer.ptr0 ,output_buffer.size0);
                       if(output_buffer.size1) memcpy(&pkt->data[output_buffer.size0], output_buffer.ptr1 ,output_buffer.size1);
                       venc_device->ioctrl(venc_device, VENC_CMD_RETURN_BITSTREAM, &output_buffer);
                       pkt->pts = c4->framecnt;
                       pkt->dts = c4->framecnt++;
                       *got_packet = 1;
                       //av_log(avctx, AV_LOG_DEBUG, "Got packet\n");
               }
       } else av_log(avctx, AV_LOG_ERROR, "Encoder fatal error\n");

       return 0;
}

static av_cold int cedarh264_close(AVCodecContext *avctx)
{
       cedarh264Context *c4 = avctx->priv_data;

       av_log(avctx, AV_LOG_INFO, "cedarx_hardware_exit\n");

       /* Close Cedar */
       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_CLOSE, 0);
       c4->venc_device = NULL;
       cedarx_hardware_exit(0);

       return 0;
}

static av_cold int cedarh264_init(AVCodecContext *avctx)
{
       cedarh264Context *c4 = avctx->priv_data;

       /* Cedar variables */
       VencBaseConfig base_cfg;
       VencAllocateBufferParam alloc_parm;
       VencSeqHeader header_data;

       /* Cedar encoder configuration */
       base_cfg.codectype = VENC_CODEC_H264;
       base_cfg.framerate = 1.0d / av_q2d(avctx->time_base); //get_fps(avctx);
       base_cfg.input_width = avctx->width;
       base_cfg.input_height = avctx->height;
       base_cfg.dst_width = avctx->width; /* Output size same as input */
       base_cfg.dst_height = avctx->height;
       base_cfg.maxKeyInterval = 25;
       switch(avctx->pix_fmt){
               case AV_PIX_FMT_YUV420P: base_cfg.inputformat = VENC_PIXEL_YUV420; break;
               case AV_PIX_FMT_RGB32: base_cfg.inputformat = VENC_PIXEL_RGBA; break;
               default:
                       av_log(avctx, AV_LOG_FATAL, "Unsupported pixel format!\n");
                       return AVERROR(ENOSYS);
       }
       base_cfg.targetbitrate = avctx->bit_rate; /* Output bitrate */

       /* Cedar Init */
       av_log(avctx, AV_LOG_INFO, "cedarx_hardware_init\n");
       alloc_parm.buffernum = 4;
       cedarx_hardware_init(0);
       c4->venc_device = cedarvEncInit();
       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_BASE_CONFIG, &base_cfg);
       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_ALLOCATE_INPUT_BUFFER, &alloc_parm);
       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_OPEN, 0);

       av_log(avctx, AV_LOG_INFO, "Input Stream: %ix%i@%i fps encode at %i bits/s\n", base_cfg.input_width,
               base_cfg.input_height, base_cfg.framerate, base_cfg.targetbitrate);

       /* Write h264 header */
       c4->venc_device->ioctrl(c4->venc_device, VENC_CMD_HEADER_DATA, &header_data);
       avctx->extradata = header_data.bufptr;
       avctx->extradata_size = header_data.length;

       /* Avlib stuffs */
       c4->framecnt = 0;

       /* Check width */
       if(avctx->width % 32 != 0){
               av_log(avctx, AV_LOG_FATAL, "Input width is not a multiple of 32!\n");
               return AVERROR(ENOSYS);
       }

       return 0;
}

static av_cold void cedarh264_init_static(AVCodec *codec)
{

}

static const AVOption options[] = {
    { NULL },
};

static const AVClass class = {
    .class_name = "cedarh264enc",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVCodecDefault cedarh264_defaults[] = {
    { NULL },
};

AVCodec ff_cedarh264_encoder = {
    .name             = "cedarh264",
    .long_name        = NULL_IF_CONFIG_SMALL("SUNXI Cedar H.264 Encoder"),
    .type             = AVMEDIA_TYPE_VIDEO,
    .id               = AV_CODEC_ID_H264,
    .priv_data_size   = sizeof(cedarh264Context),
    .init             = cedarh264_init,
    .encode2          = cedarh264_frame,
    .close            = cedarh264_close,
    .capabilities     = CODEC_CAP_DELAY | CODEC_CAP_AUTO_THREADS,
    .priv_class       = &class,
    .defaults         = cedarh264_defaults,
    .init_static_data = cedarh264_init_static,
};
