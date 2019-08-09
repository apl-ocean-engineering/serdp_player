
/*
 * Copyright (c) 2001 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/**
 * @file
 * libavcodec API use example.
 *
 * Note that libavcodec only handles codecs (mpeg, mpeg4, etc...),
 * not file formats (avi, vob, mp4, mov, mkv, mxf, flv, mpegts, mpegps, etc...).
 * See library 'libavformat' for the format handling
 * @example doc/examples/decoding_encoding.c
 */
#include "liboculus/SonarPlayer.h"
#include "serdp_common/OpenCVDisplay.h"
#include "serdp_common/PingDecoder.h"
#include <iostream>
#include <libg3logger/g3logger.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

#include "active_object/active.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
// #include <libavutil/mem.h>
#include <libswscale/swscale.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "GPMF_mp4reader.h"
}

using namespace cv;

#define AVMEDIA_TYPE_VIDEO 0
#define AVMEDIA_TYPE_GPMF 2

extern void PrintGPMF(GPMF_stream *ms);

void singleSonarDisplay(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                        std::shared_ptr<liboculus::SonarPlayerBase> player) {

  std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
  if (ping->valid()) {
    serdp_common::PingDecoder pingDecoder;
    serdp_common::PingDecoder::SonarData sonarData =
        pingDecoder.pingPlayback(ping);
    display->sonarDisplay(ping);
    cv::waitKey(1);
  }
}

int runSonar(char *filename) {
  int32_t ret = GPMF_OK;
  GPMF_stream metadata_stream, *ms = &metadata_stream;
  double metadatalength;
  uint32_t *payload = NULL; // buffer to store GPMF samples from the MP4.

  size_t mp4 =
      OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);
  if (mp4 == 0) {
    printf("error: %s is an invalid MP4/MOV\n", filename);
    return -1;
  }
  //
  metadatalength = GetDuration(mp4);

  if (metadatalength > 0.0) {

    uint32_t index, payloads = GetNumberPayloads(mp4);

    for (index = 0; index < payloads; index++) {
      uint32_t payloadsize = GetPayloadSize(mp4, index);
      float in = 0.0, out = 0.0; // times
      payload = GetPayload(mp4, payload, index);
      if (payload == NULL)
        goto cleanup;

      ret = GetPayloadTime(mp4, index, &in, &out);
      if (ret != GPMF_OK)
        goto cleanup;

      ret = GPMF_Init(ms, payload, payloadsize);
      if (ret != GPMF_OK)
        goto cleanup;

      // Find all the available Streams and the data carrying FourCC
      ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
      while (GPMF_OK == ret) {
        ret = GPMF_SeekToSamples(ms);
        // Display sonar
        std::shared_ptr<liboculus::SonarPlayerBase> player(
            liboculus::SonarPlayerBase::createGPMFSonarPlayer());
        player->setStream(ms);
        std::shared_ptr<serdp_common::OpenCVDisplay> display;
        singleSonarDisplay(display, player);
      }
    }

  cleanup:
    if (payload)
      FreePayload(payload);
    payload = NULL;
    CloseSource(mp4);
  }
}

// int playGPMF(AVCodecContext *pCodecCtx, AVPacket packet) {
int playGPMF(GPMF_stream *ms) {
  // AVFrame *pFrameRGB = NULL;
  // AVFrame *pFrame = NULL;
  //
  //
  // GPMF_stream metadata_stream, *ms = &metadata_stream;
  // int numBytes;
  // uint32_t *buffer = NULL;
  //
  // AVBufferRef *buf = packet.buf;
  // int ret = GPMF_Init(ms, (uint32_t *)buf->data, buf->size);
  // //
  // if (ret != GPMF_OK)
  //   return -1;
  // std::cout << "Found " << GPMF_RawDataSize(ms) << " bytes of sonar data"
  //           << std::endl;
  if (GPMF_RawDataSize(ms) > 0) {
    // Find all the available Streams and the data carrying FourCC
    int ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
    // std::cout << ret << " " << GPMF_OK << std::endl;
    while (GPMF_OK == ret) {
      ret = GPMF_SeekToSamples(ms);
      // Display sonar

      // std::cout << ret << std::endl;
      std::shared_ptr<liboculus::SonarPlayerBase> player(
          liboculus::SonarPlayerBase::createGPMFSonarPlayer());
      player->setStream(ms);
      std::shared_ptr<serdp_common::OpenCVDisplay> display;
      singleSonarDisplay(display, player);
    }
  }
}

int playVideo(AVCodecContext *pCodecCtx, AVFrame *pFrame, AVFrame *pFrameRGB,
              struct SwsContext *sws_ctx, AVPacket packet, int id) {
  // std::cout << "playVideo stream index: " << packet.stream_index <<
  // std::endl;
  AVCodecContext *pCodecCtxOrig = NULL;
  int frameFinished;

  int numBytes;
  uint8_t *buffer = NULL;

  int got_frame;
  avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet);
  if (got_frame) {

    // // Allocate an AVFrame structure
    // pFrame = av_frame_alloc();
    // if (pFrame == NULL)
    //   return -1;
    //
    sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
              0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    cv::Mat img(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
    cv::cvtColor(img, img, CV_BGR2RGB);
    std::string cam_img = "cam img" + std::to_string(id);
    cv::imshow(cam_img, img);
    cv::waitKey(1);
  }

  // AVBufferRef *buf = packet.buf;
  // cv::Mat img(pCodecCtx->height, pCodecCtx->width, CV_8U, buf->data);
  //
  // if (img.rows > 60 && img.cols > 60) {
  //   cv::imshow("camera img", img);
  //   cv::waitKey(1);
  // }
  //
  // numBytes =
  //     avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width,
  //     pCodecCtx->height);
  // buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
  // avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_RGB24,
  //                pCodecCtx->width, pCodecCtx->height);
  //
  // // // if (packet.stream_index == videoStream) {
  // // // Decode video frame
  // // avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);
  // //
  // // // Did we get a video frame ?
  // // if (frameFinished) {
  // // Convert the image from its native format to RGB
  // sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
  // 0,
  //           pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);
  //
  // cv::Mat img(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
  //}
  // }
}

int decodeMP4(char *filename) {
  std::unique_ptr<active_object::Active> _thread;

  AVFormatContext *pFormatCtx = NULL;
  AVCodec *pCodec = NULL;
  AVCodecContext *pCodecCtx = NULL;

  AVFrame *pFrame = NULL;

  if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) != 0)
    return -1; // Couldn't open file
  if (avformat_find_stream_info(pFormatCtx, NULL) > 0)
    return -1; // Couldn't find stream information

  // Dump information about file onto standard error
  av_dump_format(pFormatCtx, 0, filename, 0);
  int i;
  // Find the first video stream
  int videoStream = -1;
  int streamCodecArr[pFormatCtx->nb_streams];
  for (i = 0; i < pFormatCtx->nb_streams; i++) {
    // std::cout << "codec type: " <<
    // pFormatCtx->streams[i]->codec->codec_type
    //           << std::endl;
    if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
      videoStream = i;
    }
    streamCodecArr[i] = pFormatCtx->streams[i]->codec->codec_type;
    // break;
  }

  // VIDEO AVPICTURE FILL
  // Get a pointer to the codec context for the video stream
  pCodecCtx = pFormatCtx->streams[videoStream]->codec;
  pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
  if (pCodec == NULL) {
    fprintf(stderr, "Unsupported codec!\n");
    return -1;
  }

  if (avcodec_open2(pCodecCtx, pCodec, NULL) > 0) {
    // std::cout << "invalid CODEC" << std::endl;
    return -1;
  }

  // READING DATA
  struct SwsContext *sws_ctx = NULL;

  AVPacket packet;
  // initialize SWS context for software scaling
  sws_ctx = sws_getContext(
      pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,
      pCodecCtx->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);

  i = 0;
  AVFrame *pFrameRGB = NULL;

  pFrameRGB = av_frame_alloc();
  if (pFrameRGB == NULL)
    return -1;

  int numBytes =
      avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);

  uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_BGR24,
                 pCodecCtx->width, pCodecCtx->height);

  while (av_read_frame(pFormatCtx, &packet) >= 0) {
    // Is this a packet from the video stream?
    pFrame = av_frame_alloc();

    // std::cout << packet.stream_index << std::endl;

    if (streamCodecArr[packet.stream_index] == AVMEDIA_TYPE_VIDEO) {
      av_packet_rescale_ts(
          &packet, pFormatCtx->streams[packet.stream_index]->time_base,
          pFormatCtx->streams[packet.stream_index]->codec->time_base);

      // std::cout << av_frame_get_best_effort_timestamp(pFrame) << std::endl;

      // // if (got_frame) {
      playVideo(pCodecCtx, pFrame, pFrameRGB, sws_ctx, packet,
                packet.stream_index);
      // // }
    }

    else if (streamCodecArr[packet.stream_index] == AVMEDIA_TYPE_GPMF) {
      // Decode to GPMF

      // AVFrame *pFrameRGB = NULL;
      // AVFrame *pFrame = NULL;

      GPMF_stream metadata_stream, *ms = &metadata_stream;
      int numBytes;
      uint32_t *buffer = NULL;

      AVBufferRef *buf = packet.buf;
      int ret = GPMF_Init(ms, (uint32_t *)buf->data, buf->size);
      //
      if (ret != GPMF_OK)
        return -1;
      if (_thread) {
        _thread->send(std::bind(playGPMF, ms));
      } else {
        playGPMF(ms);
      }
    }
    // Free the packet that was allocated by av_read_frame
    av_free_packet(&packet);
  }

  if (videoStream == -1)
    return -1; // Didn't find a video stream
}

int main(int argc, char *argv[]) {
  av_register_all();

  libg3logger::G3Logger logger("ocClient");
  if (argc != 2) {
    printf("usage: %s <file_with_GPMF>\n", argv[0]);
    return -1;
  }

  // int ret = runSonar(argv[1]);
  int ret = decodeMP4(argv[1]);

  return ret;
}
