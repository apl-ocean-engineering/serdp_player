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

 #include <iostream>

#include "liboculus/SonarPlayer.h"

#include "serdp_common/OpenCVDisplay.h"
#include "serdp_common/DataStructures.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "active_object/active.h"
#include <math.h>

extern "C" {
  #include <libavcodec/avcodec.h>
  #include <libavformat/avformat.h>
  #include <libswscale/swscale.h>

  #include <stdint.h>
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>
  #include <sys/stat.h>
  #include <sys/types.h>
}

#define AVMEDIA_TYPE_VIDEO 0
#define AVMEDIA_TYPE_GPMF 2

namespace Decoder {

const struct NameConstants {
  std::string cameraImage = "cam img";
  std::string sonarImg = "sonar img";
} nameConstants;

// struct PacketData {
//   PacketData()
//     : sonarData(nullptr) { ; }
//
//   cv::Mat img;
//
// //  std::shared_ptr<serdp_common::AbstractSonarInterface> sonarData; // Null if packet type is image
// };

struct DecodedPacket {
  std::string name;
  int type;
  cv::Mat img;
};

class MovDecoder {

  std::shared_ptr<serdp_common::OpenCVDisplay> display;

public:
  MovDecoder();
  ~MovDecoder();

  // FFmpeg frame data types
  AVFormatContext *pFormatCtx;
  AVCodec *pCodec;
  AVCodecContext *pCodecCtx;
  AVFrame *pFrame;
  struct SwsContext *sws_ctx;
  AVFrame *pFrameRGB;
  uint8_t *buffer;
  int videoStream;

  // Init ffmpeg functions
  int openFile(char *filename);
  std::vector<int> streamCodecParse();
  void initCodecs();

  // Actual functions
  DecodedPacket unpackGPMF(AVPacket packet);
  DecodedPacket unpackVideo(AVPacket packet);

  DecodedPacket decodePacket(AVPacket packet, std::vector<int> streamCodecVec);
};

} // namespace Decoder
