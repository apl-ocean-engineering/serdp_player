#include "MovDecoder.h"

using namespace cv;

#define AVMEDIA_TYPE_VIDEO 0
#define AVMEDIA_TYPE_GPMF 2

int decodeMP4(char *filename) {
  MovDecoder movDecoder;

  std::unique_ptr<active_object::Active> _thread;

  AVFormatContext *pFormatCtx = NULL;
  AVCodec *pCodec = NULL;
  AVCodecContext *pCodecCtx = NULL;
  AVFrame *pFrame = NULL;
  bool foundVideo(false);
  int videoStream = -1;
  struct SwsContext *sws_ctx = NULL;

  if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) != 0)
    return -1; // Couldn't open file
  if (avformat_find_stream_info(pFormatCtx, NULL) > 0)
    return -1; // Couldn't find stream information

  // Dump information about file onto standard error
  av_dump_format(pFormatCtx, 0, filename, 0);

  int streamCodecArr[pFormatCtx->nb_streams];
  for (int i = 0; i < pFormatCtx->nb_streams; i++) {
    if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO &&
        !foundVideo) {
      videoStream = i;
      foundVideo = true;
    }
    streamCodecArr[i] = pFormatCtx->streams[i]->codec->codec_type;
  }

  if (!foundVideo)
    return -1; // Didn't find a video stream

  // Get a pointer to the codec context for the video stream
  pCodecCtx = pFormatCtx->streams[videoStream]->codec;
  pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
  if (pCodec == NULL) {
    fprintf(stderr, "Unsupported codec!\n");
    return -1;
  }

  if (avcodec_open2(pCodecCtx, pCodec, NULL) > 0) {
    return -1;
  }

  // READING DATA
  AVPacket packet;

  // initialize SWS context for software scaling
  sws_ctx = sws_getContext(
      pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,
      pCodecCtx->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);

  AVFrame *pFrameRGB = NULL;

  pFrameRGB = av_frame_alloc();
  if (pFrameRGB == NULL)
    return -1;

  // construct the buffer
  int numBytes =
      avpicture_get_size(AV_PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);
  uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
  avpicture_fill((AVPicture *)pFrameRGB, buffer, AV_PIX_FMT_BGR24,
                 pCodecCtx->width, pCodecCtx->height);

  while (av_read_frame(pFormatCtx, &packet) >= 0) {
    // Read through packets, decode as either video or GPMF
    pFrame = av_frame_alloc();
    if (streamCodecArr[packet.stream_index] == AVMEDIA_TYPE_VIDEO) {
      // If img packet, play image
      av_packet_rescale_ts(
          &packet, pFormatCtx->streams[packet.stream_index]->time_base,
          pFormatCtx->streams[packet.stream_index]->codec->time_base);
      movDecoder.playVideo(pCodecCtx, pFrame, pFrameRGB, sws_ctx, packet,
                           packet.stream_index);
    }

    else if (streamCodecArr[packet.stream_index] == AVMEDIA_TYPE_GPMF) {
      // If GPMF, decode to GPMF
      GPMF_stream metadata_stream, *ms = &metadata_stream;
      int numBytes;
      uint32_t *buffer = NULL;
      AVBufferRef *buf = packet.buf;
      int ret = GPMF_Init(ms, (uint32_t *)buf->data, buf->size);
      if (ret != GPMF_OK)
        return -1;
      if (_thread) {
        //_thread->send(std::bind(playGPMF, ms));
      } else {
        movDecoder.playGPMF(ms);
      }
    }
    av_free_packet(&packet);
  }
}

int main(int argc, char *argv[]) {
  av_register_all();

  libg3logger::G3Logger logger("ocClient");
  if (argc != 2) {
    printf("usage: %s <file_with_GPMF>\n", argv[0]);
    return -1;
  }
  int ret = decodeMP4(argv[1]);
  return ret;
}
