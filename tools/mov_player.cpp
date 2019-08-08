
/*
 * Copyright (c) 2001 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/**
 * @file
 * libavcodec API use example.
 *
 * Note that libavcodec only handles codecs (mpeg, mpeg4, etc...),
 * not file formats (avi, vob, mp4, mov, mkv, mxf, flv, mpegts, mpegps, etc...).
 * See library 'libavformat' for the format handling
 * @example doc/examples/decoding_encoding.c
 */
#include "liboculus/SonarPlayer.h"
#include "serdp_common/OpenCVDisplay.h"
#include "serdp_common/PingDecoder.h"
#include <iostream>
#include <libg3logger/g3logger.h>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "GPMF_mp4reader.h"
}

extern void PrintGPMF(GPMF_stream *ms);

void singleSonarDisplay(std::shared_ptr<serdp_common::OpenCVDisplay> display,
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

int runSonar(char *filename) {
  int32_t ret = GPMF_OK;
  GPMF_stream metadata_stream, *ms = &metadata_stream;
  double metadatalength;
  uint32_t *payload = NULL; // buffer to store GPMF samples from the MP4.

  size_t mp4 =
      OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);
  if (mp4 == 0) {
    printf("error: %s is an invalid MP4/MOV\n", filename);
    return -1;
  }
  //
  metadatalength = GetDuration(mp4);

  if (metadatalength > 0.0) {

    uint32_t index, payloads = GetNumberPayloads(mp4);

    for (index = 0; index < payloads; index++) {
      uint32_t payloadsize = GetPayloadSize(mp4, index);
      float in = 0.0, out = 0.0; // times
      payload = GetPayload(mp4, payload, index);
      if (payload == NULL)
        goto cleanup;

      ret = GetPayloadTime(mp4, index, &in, &out);
      if (ret != GPMF_OK)
        goto cleanup;

      ret = GPMF_Init(ms, payload, payloadsize);
      if (ret != GPMF_OK)
        goto cleanup;

      // Find all the available Streams and the data carrying FourCC
      ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
      while (GPMF_OK == ret) {
        ret = GPMF_SeekToSamples(ms);
        // Display sonar
        std::shared_ptr<liboculus::SonarPlayerBase> player(
            liboculus::SonarPlayerBase::createGPMFSonarPlayer());
        player->setStream(ms);
        std::shared_ptr<serdp_common::OpenCVDisplay> display;
        singleSonarDisplay(display, player);
      }
    }

  cleanup:
    if (payload)
      FreePayload(payload);
    payload = NULL;
    CloseSource(mp4);
  }
}

int main(int argc, char *argv[]) {
  libg3logger::G3Logger logger("ocClient");
  if (argc != 2) {
    printf("usage: %s <file_with_GPMF>\n", argv[0]);
    return -1;
  }

  int ret = runSonar(argv[1]);

  return ret;
}
