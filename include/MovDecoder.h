#include <iostream>
#include <libg3logger/g3logger.h>

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

struct DecodedPacket {
  std::string name;
  int type;
  cv::Mat img;
};

class MovDecoder {
public:
  MovDecoder();
  ~MovDecoder();

  AVFormatContext *pFormatCtx;
  AVCodec *pCodec;
  AVCodecContext *pCodecCtx;
  AVFrame *pFrame;
  struct SwsContext *sws_ctx;
  AVFrame *pFrameRGB;
  uint8_t *buffer;
  int videoStream;

  std::vector<int> streamCodecParse();
  void initCodecs();
  cv::Mat gpmfImg(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                  std::shared_ptr<liboculus::SonarPlayerBase> player);
  cv::Mat unpackGPMF(AVPacket packet);
  cv::Mat unpackVideo(AVPacket packet);
  DecodedPacket decodePacket(AVPacket packet, std::vector<int> streamCodecVec);
};
