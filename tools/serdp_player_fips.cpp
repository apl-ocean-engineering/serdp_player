// Code based on a tutorial by Martin Bohme
// (boehme@inb.uni-luebeckREMOVETHIS.de) Tested on Gentoo, CVS version 5/01/07
// compiled with GCC 4.1.1 With updates from
// https://github.com/chelyaev/ffmpeg-tutorial Updates tested on:
// LAVC 54.59.100, LAVF 54.29.104, LSWS 2.1.101
// on GCC 4.7.2 in Debian February 2015

// Orignal tutorial at http://dranger.com/ffmpeg/tutorial01.html

#include "libg3logger/g3logger.h"
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
  bool display(true);
  app.add_option("-d, --display", display,
                 "to display or not the GPMF and video data read for MOV/Mp4");

  CLI11_PARSE(app, argc, argv);

  if (inputFilename.empty()) {
    LOG(FATAL) << "Blank inputfile";
    return -1;
  }

  char *filename[inputFilename.size() + 1];
  strcpy(*filename, inputFilename.c_str());

  MovDecoder movDecoder;

  movDecoder.openFile(*filename);

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
    if (decodedPacket.data.img.rows > 60 && decodedPacket.data.img.cols > 60 &&
        display) {
      // Display
      cv::imshow(decodedPacket.name, decodedPacket.data.img);
      cv::waitKey(1);
    } else {
      LOG(DEBUG) << "No valid image found";
    }

    av_free_packet(&packet);
  }
}
