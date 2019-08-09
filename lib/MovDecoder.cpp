#include "MovDecoder.h"

void MovDecoder::SonarDisplay(
    std::shared_ptr<serdp_common::OpenCVDisplay> display,
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

int MovDecoder::playGPMF(GPMF_stream *ms) {
  if (GPMF_RawDataSize(ms) > 0) {
    // Find all the available Streams and the data carrying FourCC
    int ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
    while (GPMF_OK == ret) {
      ret = GPMF_SeekToSamples(ms);
      // Display sonar
      std::shared_ptr<liboculus::SonarPlayerBase> player(
          liboculus::SonarPlayerBase::createGPMFSonarPlayer());
      player->setStream(ms);
      std::shared_ptr<serdp_common::OpenCVDisplay> display;
      SonarDisplay(display, player);
    }
    return 1;
  }
  return 0;
}

int MovDecoder::playVideo(AVCodecContext *pCodecCtx, AVFrame *pFrame,
                          AVFrame *pFrameRGB, struct SwsContext *sws_ctx,
                          AVPacket packet, int id) {
  AVCodecContext *pCodecCtxOrig = NULL;
  int frameFinished;

  int numBytes;
  uint8_t *buffer = NULL;

  int got_frame;
  avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet);
  if (got_frame) {
    sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
              0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    cv::Mat img(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
    cv::cvtColor(img, img, CV_BGR2RGB);
    std::string cam_img = "cam img" + std::to_string(id);
    cv::imshow(cam_img, img);
    cv::waitKey(1);
  }
}
