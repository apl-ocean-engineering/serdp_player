/*
 * Copyright (c) 2017-2020 Aaron Marburg <amarburg@uw.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of University of Washington nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "serdp_player/MovDecoder.h"
#include "serdp_common/DrawSonar.h"
#include "serdp_gpmf/GpmfSonarPlayer.h"

#include "gpmf-parser/GPMF_parser.h"

using namespace cv;

namespace Decoder {

// serdp_common::SonarPoint bearingRange2Cartesian(float bearing, float range) {
//   float x = range * sin(bearing);
//   float z = range * cos(bearing);
//
//   return serdp_common::SonarPoint(x, z);
// }

MovDecoder::MovDecoder()
    : pFormatCtx(NULL), pCodec(NULL), pCodecCtx(NULL), pFrame(NULL),
      pFrameRGB(NULL), sws_ctx(NULL), buffer(NULL), videoStream(0)
{;}

MovDecoder::~MovDecoder() {
  // Free the RGB image
  av_free(buffer);
  // Free the frames
  av_free(pFrameRGB);
  av_free(pFrame);
  // Close the codecs
  avcodec_close(pCodecCtx);
  // avcodec_close(pCodec);
  // Close the video file
  avformat_close_input(&pFormatCtx);
}

int MovDecoder::openFile(char *filename) {
  if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) != 0) {
    LOG(FATAL) << "Couldn't open file";
    return 0;
  }
  if (avformat_find_stream_info(pFormatCtx, NULL) > 0) {
    LOG(FATAL) << "Couldn't open file";
    return 0;
  }

  LOG(INFO) << "Successfully Opened file";

  // Print mov information
  av_dump_format(pFormatCtx, 0, filename, 0);

  return 1;
}

std::vector<int> MovDecoder::streamCodecParse() {
  // Find all present codecs
  bool foundVideo = false;
  std::vector<int> vec;

  for (int i = 0; i < pFormatCtx->nb_streams; i++) {
    if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO &&
        !foundVideo) {
      videoStream = i;
      foundVideo = true;
    }
    vec.push_back(pFormatCtx->streams[i]->codec->codec_type);
  }

  if (!foundVideo) return std::vector<int>();

  return vec;
}

void MovDecoder::initCodecs() {
  // Initalize the codecs in the streams
  pCodecCtx = pFormatCtx->streams[videoStream]->codec;
  pCodec = avcodec_find_decoder(pCodecCtx->codec_id);

  if (pCodec == NULL) {
    LOG(FATAL) << "Unsupported codec!";
    return;
  }
  if (avcodec_open2(pCodecCtx, pCodec, NULL) > 0) {
    LOG(FATAL) << "Unsupported codec!";
    return;
  }

  // initialize SWS context for software scaling
  sws_ctx = sws_getContext(
      pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,
      pCodecCtx->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);
  pFrameRGB = av_frame_alloc();
  if (pFrameRGB == NULL) {
    LOG(FATAL) << "Null frame!";
    return;
  }

  // construct the buffer
  int numBytes = avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);
  buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  // Construct the inital image
  avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_BGR24,
                 pCodecCtx->width, pCodecCtx->height);
}

DecodedPacket MovDecoder::unpackGPMF(AVPacket packet) {
  // Unpack an AVPacket to the base GPMF type
  DecodedPacket decodedPacket;
  //std::shared_ptr<serdp_common::SonarData> sonarData;
  std::shared_ptr<GPMF_stream> ms( new GPMF_stream );
  int numBytes;
  AVBufferRef *buf = packet.buf;
  int ret = GPMF_Init(ms.get(), (uint32_t *)buf->data, buf->size);

  // GPMF decoding
  std::string cam_img =
      nameConstants.sonarImg + std::to_string(packet.stream_index);
  decodedPacket.name = cam_img;
  decodedPacket.type = AVMEDIA_TYPE_GPMF;

  if (ret != GPMF_OK) {
    LOG(WARNING) << "Metadata stream initalization failure";
    return decodedPacket;
  }

  if (GPMF_RawDataSize(ms.get()) > 0) {
    // Find all the available Streams and the data carrying FourCC
    int ret = GPMF_FindNext(ms.get(), GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
    LOG(DEBUG) << "Unpacking GPMF data";
    while (GPMF_OK == ret) {
      // Setup GPMF player
      ret = GPMF_SeekToSamples(ms.get());

      serdp_gpmf::GPMFSonarPlayer player(ms);
      // player->setStream(ms);

      liboculus::SimplePingResult ping;
      if( !player.nextPing(ping) ) return decodedPacket;

      if (ping.valid()) {
        // If the ping is a valid GPMF type, upack sonar data and img
        //sonarData = pingDecoder.pingPlayback(ping);
        cv::Size sz = serdp_common::calculateImageSize( ping, cv::Size(0,0) );
        cv::Mat img( sz, CV_8UC3 );
        serdp_common::drawSonar( ping, img );

        // // Channel three is intensity, best for display
        // Mat bgr[3];
        // cv::split(img, bgr); // split

        decodedPacket.img = img; // bgr[2];
        //data.sonarData = sonarData;
      }
    }
  } else {
    LOG(DEBUG) << "No GPMF data found";
  }
  return decodedPacket;
}

DecodedPacket MovDecoder::unpackVideo(AVPacket packet) {
  // Unpack an AVPacket to PacketData type
  DecodedPacket decodedPacket;

  decodedPacket.name = nameConstants.cameraImage + std::to_string(packet.stream_index);
  decodedPacket.type = AVMEDIA_TYPE_VIDEO;

  // Something something timestamps?
  av_packet_rescale_ts(
      &packet, pFormatCtx->streams[packet.stream_index]->time_base,
      pFormatCtx->streams[packet.stream_index]->codec->time_base);

  // decode packet
  int got_frame;
  avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet);
  if (got_frame) {
    LOG(DEBUG) << "Unpacking video data";
    // Cast packet to image
    sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
              0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    cv::Mat img =
        cv::Mat(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
    cv::cvtColor(img, img, CV_BGR2RGB);
    decodedPacket.img = img;

  } else {
    LOG(DEBUG) << "No video to unpack";
  }

  return decodedPacket;
}

DecodedPacket MovDecoder::decodePacket(AVPacket packet,
                                       std::vector<int> streamCodecVec) {

  // Get frame
  pFrame = av_frame_alloc();

  if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_VIDEO) {

    // If video type, unpack as video
    return unpackVideo(packet);

  } else if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_GPMF) {


    // if GPMF type, unpack as GPMF
    return unpackGPMF(packet);
  }

  // Null packet on failure
  return DecodedPacket();
}


} // namespace Decoder
