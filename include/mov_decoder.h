#include <iostream>

extern "C" {
#include "libavcodec/avcodec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif

#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>

#define INBUF_SIZE
#define AUDIO_INBUF_SIZE
}

void video_decode_example(const char *outfilename, const char *filename) {
  // AVFrame *frame = avcodec_alloc_frame();
  // if (!frame) {
  //   std::cout << "Error allocating the frame" << std::endl;
  //   return;
  // }
  //
  // // you can change the file name "01 Push Me to the Floor.wav" to whatever
  // the
  // // file is you're reading, like "myFile.ogg" or "someFile.webm" and this
  // // should still work
  // AVFormatContext *formatContext = NULL;
  // if (avformat_open_input(&formatContext,
  //                         "../CarBots Marines vs. Zerglings-WKvX3a2J86s.mp4",
  //                         NULL, NULL) != 0) {
  //   av_free(frame);
  //   std::cout << "Error opening the file" << std::endl;
  //   return;
  // }

  // AVCodec *codec;
  // AVCodecContext *c = NULL;
  // int frame, got_picture, len;
  // FILE *f;
  // // std::cout << "Here" << std::endl;
  // AVFrame *picture;
  // uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
  // char buf[1024];
  // AVPacket avpkt;
  // av_init_packet(&avpkt);
  /* set end of buffer to 0 (this ensures that no overreading
               happens for damaged mpeg streams) */
  // memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);

  //
  // codec = avcodec_find_decoder(CODEC_ID_MPEG1VIDEO);
  // if (!codec) {

  //   exit(1);
  // }
  // c = avcodec_alloc_context();
  // picture = avcodec_alloc_frame();
  // if (codec->capabilities & CODEC_CAP_TRUNCATED)
  //   c->flags |= CODEC_FLAG_TRUNCATED; /* we do not send complete frames */
  // if (avcodec_open(c, codec) < 0) {

  //   exit(1);
  // }
  // /* the codec gives us the frame size, in samples */
  // f = fopen(filename, "rb");
  // if (!f) {

  //   exit(1);
  // }
  // frame = 0;
  // for (;;) {
  //   avpkt.size = fread(inbuf, 1, INBUF_SIZE, f);
  //   if (avpkt.size == 0)
  //     break;
  //   avpkt.data = inbuf;
  //   while (avpkt.size > 0) {
  //     len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
  //     if (len < 0) {

  //       exit(1);
  //     }
  //     if (got_picture) {

  //       fflush(stdout);
  //       /* the picture is allocated by the decoder. no need to
  //                 free it */
  //       snprintf(buf, sizeof(buf), outfilename, frame);
  //       pgm_save(picture->data[0], picture->linesize[0], 00410 c->width,
  //                c->height, buf);
  //       frame++;
  //     }
  //     avpkt.size -= len;
  //     avpkt.data += len;
  //   }
  // }
  // /* some codecs, such as MPEG, transmit the I and P frame with a
  //         latency of one frame. You must do the following to have a
  //         chance to get the last frame of the video */
  // avpkt.data = NULL;
  // avpkt.size = 0;
  // len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
  // if (got_picture) {
  //
  //   fflush(stdout);
  //   /* the picture is allocated by the decoder. no need to
  //           free it */
  //   snprintf(buf, sizeof(buf), outfilename, frame);
  //   pgm_save(picture->data[0], picture->linesize[0], 00432 c->width,
  //   c->height,
  //            buf);
  //   frame++;
  // }
  // fclose(f);
  // avcodec_close(c);
  // av_free(c);
  // av_free(picture);
}
