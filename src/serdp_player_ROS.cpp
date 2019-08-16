#include "ros/ros.h"

#include "serdp_player/MovDecoder.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

int decodeMP4(char *filename);

int main(int argc, char **argv) {
  av_register_all();

  // Setup loggers
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  // init ros
  ros::init(argc, argv, "serdp_player_ROS");
  ros::NodeHandle nh_;

  std::string inputFilename("");
  nh_.getParam("mov_filename", inputFilename);

  char *filename[inputFilename.size() + 1];
  strcpy(*filename, inputFilename.c_str());

  if (inputFilename.empty()) {
    LOG(FATAL) << "Blank inputfile";
    return -1;
  }

  LOG(INFO) << "Recieved input file " << inputFilename;

  // Main function
  int ret = decodeMP4(*filename);

  return 0;
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

  LOG(INFO) << "Opened file";

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

  while (av_read_frame(movDecoder.pFormatCtx, &packet) >= 0 && ros::ok()) {
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
