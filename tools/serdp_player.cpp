// Code based on a tutorial by Martin Bohme
// (boehme@inb.uni-luebeckREMOVETHIS.de) Tested on Gentoo, CVS version 5/01/07
// compiled with GCC 4.1.1 With updates from
// https://github.com/chelyaev/ffmpeg-tutorial Updates tested on:
// LAVC 54.59.100, LAVF 54.29.104, LSWS 2.1.101
// on GCC 4.7.2 in Debian February 2015

// Orignal tutorial at http://dranger.com/ffmpeg/tutorial01.html

#include "serdp_player/MovDecoder.h"
#include <CLI/CLI.hpp>

int decodeMP4(char *filename);

int main(int argc, char *argv[]) {
  av_register_all();

  libg3logger::G3Logger logger("serdp_player");
  CLI::App app{"Simple Serdp Mov Player app"};

  std::string inputFilename("");
  app.add_option("input", inputFilename,
                 ".mov or .mp4 file to read video and GPMF data from");

  CLI11_PARSE(app, argc, argv);

  if (inputFilename.empty()) {
    LOG(FATAL) << "Blank inputfile";
    return -1;
  }

  // Main function
  int ret = decodeMP4(argv[1]);

  return ret;
}

int decodeMP4(char *filename) {
  MovDecoder movDecoder;
  // std::unique_ptr<active_object::Active> _thread;

  if (avformat_open_input(&movDecoder.pFormatCtx, filename, NULL, NULL) != 0) {
    LOG(FATAL) << "Couldn't open file";
    return -1;
  }
  if (avformat_find_stream_info(movDecoder.pFormatCtx, NULL) > 0) {
    LOG(FATAL) << "Couldn't open file";
    return -1;
  }

  // Print mov information
  av_dump_format(movDecoder.pFormatCtx, 0, filename, 0);

  // Determine stream codec types
  std::vector<int> streamCodecVec = movDecoder.streamCodecParse();

  if (streamCodecVec.empty()) {
    LOG(FATAL) << "No video streams found";
    return -1;
  }

  movDecoder.initCodecs();

  // Initalize packet
  AVPacket packet;

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
