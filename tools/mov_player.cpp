
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

//#include "gpmf-parser/GPMF_parser.h"
// #include "GPMF_print.cpp"
#include "gpmf-parser/demo/GPMF_mp4reader.h"
}

extern void PrintGPMF(GPMF_stream *ms);

// void sonarDisplay(GPMF_stream *ms);
void singleSonarDisplay(std::shared_ptr<serdp_common::OpenCVDisplay> display,
                        std::shared_ptr<liboculus::SonarPlayerBase> player) {

  std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
  // while (ping) {
  if (ping->valid()) {
    serdp_common::PingDecoder pingDecoder;
    serdp_common::PingDecoder::SonarData sonarData =
        pingDecoder.pingPlayback(ping);
    display->sonarDisplay(ping);
    cv::waitKey(1);
  }
  // ping = player->nextPing();
  //}
}

int runSonar(char *filename) {
  int32_t ret = GPMF_OK;
  GPMF_stream metadata_stream, *ms = &metadata_stream;
  double metadatalength;
  uint32_t *payload = NULL; // buffer to store GPMF samples from the MP4.

  size_t mp4 =
      OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);
  // OpenMP4SourceUDTA(argv[1]);
  if (mp4 == 0) {
    printf("error: %s is an invalid MP4/MOV\n", filename);
    return -1;
  }
  //
  metadatalength = GetDuration(mp4);

  if (metadatalength > 0.0) {
    // if (1) {
    uint32_t index, payloads = GetNumberPayloads(mp4);
    // metadatalength, payloads, argv[1]);
    // std::cout << payloads << std::endl;
    if (payloads == 1) // Printf the contents of the single payload
    {
      uint32_t payloadsize = GetPayloadSize(mp4, 0);
      payload = GetPayload(mp4, payload, 0);
      if (payload == NULL)
        goto cleanup;

      ret = GPMF_Init(ms, payload, payloadsize);
      if (ret != GPMF_OK)
        goto cleanup;

      // Output (printf) all the contained GPMF data within this payload
      ret = GPMF_Validate(ms, GPMF_RECURSE_LEVELS); // optional
      if (GPMF_OK != ret) {

        goto cleanup;
      }

      GPMF_ResetState(ms);
      do {

        PrintGPMF(ms); // printf current GPMF KLV
      } while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));
      GPMF_ResetState(ms);
    }

    //
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
      // if (index < 1000) // show first payload
      if (1) {
        ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);

        while (GPMF_OK == ret) {
          ret = GPMF_SeekToSamples(ms);
          std::shared_ptr<liboculus::SonarPlayerBase> player(
              liboculus::SonarPlayerBase::createGPMFSonarPlayer());
          player->setStream(ms);
          std::shared_ptr<serdp_common::OpenCVDisplay> display;
          singleSonarDisplay(display, player);
          if (GPMF_OK == ret) // find the last FOURCC within the stream
          {
            uint32_t key = GPMF_Key(ms);
            // GPMF_SampleType type = GPMF_Type(ms);
            uint32_t type = GPMF_Type(ms);
            uint32_t elements = GPMF_ElementsInStruct(ms);
            // uint32_t samples = GPMF_Repeat(ms);
            uint32_t samples = GPMF_PayloadSampleCount(ms);

            if (samples) {
              // printf("  STRM of %c%c%c%c ", PRINTF_4CC(key));

              if (type == GPMF_TYPE_COMPLEX) {
                GPMF_stream find_stream;
                GPMF_CopyState(ms, &find_stream);

                if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE,
                                             GPMF_CURRENT_LEVEL)) {
                  char tmp[64];
                  char *data = (char *)GPMF_RawData(&find_stream);
                  int size = GPMF_RawDataSize(&find_stream);

                  if (size < sizeof(tmp)) {
                    memcpy(tmp, data, size);
                    tmp[size] = 0;
                    // printf("of type %s ", tmp);
                  }
                }

              } else {
                // printf("of type %c ", type);
              }

              // printf("with %d sample%s \n", samples, samples > 1 ? "s" : "");

              // if (elements > 1)
              // printf("-- %d elements per sample", elements);
            }

            ret = GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS);
          } else {
            if (ret ==
                GPMF_ERROR_BAD_STRUCTURE) // some payload element was corrupt,
                                          // skip to the next valid GPMF KLV at
                                          // the previous level.
            {
              ret = GPMF_Next(
                  ms, GPMF_CURRENT_LEVEL); // this will be the nexstream if
              // any more are present.
            }
          }
        }
        GPMF_ResetState(ms);
      }
      // Find GPS values and return scaled doubles.
      if (index == 0) // show first payload
      {
        if (GPMF_OK ==
                GPMF_FindNext(ms, STR2FOURCC("GPS5"),
                              GPMF_RECURSE_LEVELS) || // GoPro Hero5 / 6 GPS
            GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("GPRI"),
                                     GPMF_RECURSE_LEVELS)) // GoPro Karma GPS
        {
          uint32_t key = GPMF_Key(ms);
          uint32_t samples = GPMF_Repeat(ms);
          uint32_t elements = GPMF_ElementsInStruct(ms);
          uint32_t buffersize = samples * elements * sizeof(double);
          GPMF_stream find_stream;
          double *ptr, *tmpbuffer = (double *)malloc(buffersize);
          char units[10][6] = {""};
          uint32_t unit_samples = 1;

          if (tmpbuffer && samples) {
            uint32_t i, j;

            // Search for any units to display
            GPMF_CopyState(ms, &find_stream);
            if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS,
                                         GPMF_CURRENT_LEVEL) ||
                GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS,
                                         GPMF_CURRENT_LEVEL)) {
              char *data = (char *)GPMF_RawData(&find_stream);
              int ssize = GPMF_StructSize(&find_stream);
              unit_samples = GPMF_Repeat(&find_stream);

              for (i = 0; i < unit_samples; i++) {
                memcpy(units[i], data, ssize);
                units[i][ssize] = 0;
                data += ssize;
              }
            }

            // GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples);
            //
            // Output data in LittleEnd, but no scale
            GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples,
                            GPMF_TYPE_DOUBLE); // Output scaled data as floats

            ptr = tmpbuffer;
            for (i = 0; i < samples; i++) {
              printf("%c%c%c%c ", PRINTF_4CC(key));
              for (j = 0; j < elements; j++)
                printf("%.3f%s, ", *ptr++, units[j % unit_samples]);
            }
            free(tmpbuffer);
          }
        }
        GPMF_ResetState(ms);
      }
    }
    // std::cout << GPMF_OK << " " << GPMF_Next(ms, GPMF_RECURSE_LEVELS)
    //          << std::endl;
    // Find all the available Streams and compute they sample rates
    //
    do {
      // std::cout << ms << std::endl;
      PrintGPMF(ms); // printf current GPMF KLV
    } while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));
    GPMF_ResetState(ms);
    //
    while (GPMF_OK == GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS)) {
      if (GPMF_OK ==
          GPMF_SeekToSamples(ms)) // find the last FOURCC within the stream

      {
        // sonarDisplay(ms);
        double in = 0.0, out = 0.0;
        uint32_t fourcc = GPMF_Key(ms);
        double rate = GetGPMFSampleRate(
            mp4, fourcc, GPMF_SAMPLE_RATE_PRECISE); // GPMF_SAMPLE_RATE_FAST);
        // std::cout << "rate " << rate << std::endl;
      }
      // else {
      //   std::cout << "NOT OK" << std::endl;
      // }
    }

    // // DO Processing
    // GPMF_ResetState(ms);

  cleanup:
    // std::cout << "cleanup" << std::endl;
    if (payload)
      FreePayload(payload);
    payload = NULL;
    CloseSource(mp4);
  }
}

int main(int argc, char *argv[]) {
  libg3logger::G3Logger logger("ocClient");
  // LOG(WARNING) << "start"
  if (argc != 2) {
    printf("usage: %s <file_with_GPMF>\n", argv[0]);
    return -1;
  }

  int ret = runSonar(argv[1]);

  return ret;
}

// void sonarDisplay(GPMF_stream *ms) {
//   if (ms) {
//     // std::cout << std::endl << std::endl << "here" << std::endl;
//     std::shared_ptr<liboculus::SonarPlayerBase> player(
//         liboculus::SonarPlayerBase::createGPMFSonarPlayer());
//     player->setStream(ms);
//
//     std::shared_ptr<liboculus::SimplePingResult> ping(player->nextPing());
//     std::shared_ptr<serdp_common::OpenCVDisplay> display;
//
//     while (ping) {
//       if (ping->valid()) {
//         serdp_common::PingDecoder pingDecoder;
//         serdp_common::PingDecoder::SonarData sonarData =
//             pingDecoder.pingPlayback(ping);
//         display->sonarDisplay(ping);
//         cv::waitKey(1);
//       }
//       ping = player->nextPing();
//     }
//   }
// }
