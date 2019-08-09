// Code based on a tutorial by Martin Bohme
// (boehme@inb.uni-luebeckREMOVETHIS.de) Tested on Gentoo, CVS version 5/01/07
// compiled with GCC 4.1.1 With updates from
// https://github.com/chelyaev/ffmpeg-tutorial Updates tested on:
// LAVC 54.59.100, LAVF 54.29.104, LSWS 2.1.101
// on GCC 4.7.2 in Debian February 2015

// Orignal tutorial at http://dranger.com/ffmpeg/tutorial01.html

#include "MovDecoder.h"
int decodeMP4(char *filename);

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

int decodeMP4(char *filename) {
  MovDecoder movDecoder;
  // std::unique_ptr<active_object::Active> _thread;

  if (avformat_open_input(&movDecoder.pFormatCtx, filename, NULL, NULL) != 0)
    return -1; // Couldn't open file
  if (avformat_find_stream_info(movDecoder.pFormatCtx, NULL) > 0)
    return -1; // Couldn't find stream information

  // Print mov information
  av_dump_format(movDecoder.pFormatCtx, 0, filename, 0);

  // Determine stream codec types
  std::vector<int> streamCodecVec = movDecoder.streamCodecParse();

  if (streamCodecVec.empty()) {
    LOG(FATAL) << "No video streams found";
    return -1;
  }

  movDecoder.pCodecCtx =
      movDecoder.pFormatCtx->streams[movDecoder.videoStream]->codec;
  movDecoder.pCodec = avcodec_find_decoder(movDecoder.pCodecCtx->codec_id);

  if (movDecoder.pCodec == NULL) {
    LOG(FATAL) << "Unsupported codec!";
    return -1;
  }
  if (avcodec_open2(movDecoder.pCodecCtx, movDecoder.pCodec, NULL) > 0) {
    LOG(FATAL) << "Unsupported codec!";
    return -1;
  }

  // Initalize packet
  AVPacket packet;

  // initialize SWS context for software scaling
  movDecoder.sws_ctx =
      sws_getContext(movDecoder.pCodecCtx->width, movDecoder.pCodecCtx->height,
                     movDecoder.pCodecCtx->pix_fmt, movDecoder.pCodecCtx->width,
                     movDecoder.pCodecCtx->height, AV_PIX_FMT_RGB24,
                     SWS_BILINEAR, NULL, NULL, NULL);
  movDecoder.pFrameRGB = av_frame_alloc();
  if (movDecoder.pFrameRGB == NULL) {
    LOG(FATAL) << "Null frame!";
    return -1;
  }

  // construct the buffer
  int numBytes =
      avpicture_get_size(AV_PIX_FMT_RGB24, movDecoder.pCodecCtx->width,
                         movDecoder.pCodecCtx->height);
  movDecoder.buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

  // Construct the inital image
  avpicture_fill((AVPicture *)movDecoder.pFrameRGB, movDecoder.buffer,
                 AV_PIX_FMT_BGR24, movDecoder.pCodecCtx->width,
                 movDecoder.pCodecCtx->height);

  while (av_read_frame(movDecoder.pFormatCtx, &packet) >= 0) {
    // Read through packets, decode as either video or GPMF
    DecodedPacket decodedPacket =
        movDecoder.decodePacket(packet, streamCodecVec);
    if (decodedPacket.img.rows < 60 | decodedPacket.img.cols < 60) {
      LOG(DEBUG) << "No valid image found";
    } else {
      // Display
      cv::imshow(decodedPacket.name, decodedPacket.img);
      cv::waitKey(1);
    }
    av_free_packet(&packet);
  }
}
