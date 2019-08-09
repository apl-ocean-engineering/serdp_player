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
  // Find all present codecs
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
  int numBytes =
      avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);
  buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  // Construct the inital image
  avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_BGR24,
                 pCodecCtx->width, pCodecCtx->height);
}

cv::Mat
MovDecoder::gpmfImg(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                    std::shared_ptr<liboculus::SonarPlayerBase> player) {
  // Cast the GPMF format into a cv mat
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

cv::Mat MovDecoder::unpackGPMF(AVPacket packet) {
  // Unpack an AVPacket to the base GPMF type
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
      img = gpmfImg(display, player);
    }
  }
  return img;
}

cv::Mat MovDecoder::unpackVideo(AVPacket packet) {
  // Unpack an AVPacket to cv Mat
  cv::Mat img;

  // Something something timestamps?
  av_packet_rescale_ts(
      &packet, pFormatCtx->streams[packet.stream_index]->time_base,
      pFormatCtx->streams[packet.stream_index]->codec->time_base);

  // decode packet
  int got_frame;
  avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet);
  if (got_frame) {
    // Cast packet to image
    sws_scale(sws_ctx, (uint8_t const *const *)pFrame->data, pFrame->linesize,
              0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    img = cv::Mat(pFrame->height, pFrame->width, CV_8UC3, pFrameRGB->data[0]);
    cv::cvtColor(img, img, CV_BGR2RGB);
  }
  return img;
}

DecodedPacket MovDecoder::decodePacket(AVPacket packet,
                                       std::vector<int> streamCodecVec) {
  // Decode ffmpeg packet to cv image types
  DecodedPacket decodedPacket;
  cv::Mat img;

  // Get frame
  pFrame = av_frame_alloc();

  if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_VIDEO) {
    // Standard video encoding
    std::string cam_img =
        nameConstants.cameraImage + std::to_string(packet.stream_index);
    decodedPacket.name = cam_img;
    decodedPacket.type = AVMEDIA_TYPE_VIDEO;

    img = unpackVideo(packet);

  } else if (streamCodecVec.at(packet.stream_index) == AVMEDIA_TYPE_GPMF) {
    // GPMF encoding
    std::string cam_img =
        nameConstants.sonarImg + std::to_string(packet.stream_index);
    decodedPacket.name = cam_img;
    decodedPacket.type = AVMEDIA_TYPE_GPMF;

    img = unpackGPMF(packet);
  }

  decodedPacket.img = img;

  return decodedPacket;
}
