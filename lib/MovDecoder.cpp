#include "MovDecoder.h"

using namespace cv;

MovDecoder::MovDecoder()
    : pFormatCtx(NULL), pCodec(NULL), pCodecCtx(NULL), pFrame(NULL),
      pFrameRGB(NULL), sws_ctx(NULL), buffer(NULL), videoStream(0) {}

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

std::vector<int> MovDecoder::streamCodecParse() {
  bool foundVideo(false);
  std::vector<int> vec;
  for (int i = 0; i < pFormatCtx->nb_streams; i++) {
    if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO &&
        !foundVideo) {
      videoStream = i;
      foundVideo = true;
    }
    vec.push_back(pFormatCtx->streams[i]->codec->codec_type);
  }
  if (!foundVideo) {
    return std::vector<int>();
  }
  return vec;
}

cv::Mat
MovDecoder::sonarDisplay(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                         std::shared_ptr<liboculus::SonarPlayerBase> player) {
  cv::Mat img;
  std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
  if (ping->valid()) {
    serdp_common::PingDecoder pingDecoder;
    serdp_common::PingDecoder::SonarData sonarData =
        pingDecoder.pingPlayback(ping);
    img = display->sonarPing2Img(ping);
  }

  return img;
}

cv::Mat MovDecoder::playGPMF(AVPacket packet) {
  cv::Mat img;
  GPMF_stream metadata_stream, *ms = &metadata_stream;
  int numBytes;
  AVBufferRef *buf = packet.buf;
  int ret = GPMF_Init(ms, (uint32_t *)buf->data, buf->size);

  if (ret != GPMF_OK) {
    LOG(WARNING) << "Metadata stream initalization failure";
    return img;
  }

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
      img = sonarDisplay(display, player);
    }
  }
  return img;
}

cv::Mat MovDecoder::playVideo(AVPacket packet) {
  cv::Mat img;
  av_packet_rescale_ts(
      &packet, pFormatCtx->streams[packet.stream_index]->time_base,
      pFormatCtx->streams[packet.stream_index]->codec->time_base);
  AVCodecContext *pCodecCtxOrig = NULL;
  int frameFinished;

  int numBytes;
  uint8_t *buffer = NULL;

  int got_frame;
  avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet);
  if (got_frame) {
    sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
              0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    img = cv::Mat(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
    cv::cvtColor(img, img, CV_BGR2RGB);
  }
  return img;
}

DecodedPacket MovDecoder::decodePacket(AVPacket packet,
                                       std::vector<int> streamCodecVec) {
  DecodedPacket decodedPacket;
  cv::Mat img;
  pFrame = av_frame_alloc();
  if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_VIDEO) {
    std::string cam_img =
        nameConstants.cameraImage + std::to_string(packet.stream_index);
    decodedPacket.name = cam_img;
    decodedPacket.type = AVMEDIA_TYPE_VIDEO;

    img = playVideo(packet);
  } else if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_GPMF) {
    std::string cam_img =
        nameConstants.sonarImg + std::to_string(packet.stream_index);
    decodedPacket.name = cam_img;
    decodedPacket.type = AVMEDIA_TYPE_GPMF;

    img = playGPMF(packet);
  }
  decodedPacket.img = img;

  return decodedPacket;
}
