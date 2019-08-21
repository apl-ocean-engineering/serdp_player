#include <iostream>

#include "liboculus/SonarPlayer.h"

#include "serdp_common/OpenCVDisplay.h"
#include "serdp_common/PingDecoder.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "active_object/active.h"

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

const struct NameConstants {
  std::string cameraImage = "cam img";
  std::string sonarImg = "sonar img";
} nameConstants;

struct PacketData {
  PacketData() : sonarData(nullptr) { ; }
  cv::Mat img;
  std::shared_ptr<serdp_common::SonarData>
      sonarData; // Null if packet type is image
};

struct DecodedPacket {
  std::string name;
  int type;
  // cv::Mat img;
  PacketData data;
};

class MovDecoder {
  // Ping decoder types
  serdp_common::PingDecoder pingDecoder;
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
  PacketData unpackGPMF(AVPacket packet);
  PacketData unpackVideo(AVPacket packet);

  DecodedPacket decodePacket(AVPacket packet, std::vector<int> streamCodecVec);
};
