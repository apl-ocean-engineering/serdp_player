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

class MovDecoder {
public:
  void SonarDisplay(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                    std::shared_ptr<liboculus::SonarPlayerBase> player);
  int playGPMF(GPMF_stream *ms);
  int playVideo(AVCodecContext *pCodecCtx, AVFrame *pFrame, AVFrame *pFrameRGB,
                struct SwsContext *sws_ctx, AVPacket packet, int id);
};
