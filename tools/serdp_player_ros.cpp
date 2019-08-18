#include "ros/ros.h"

#include "serdp_player/ROSEncode.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

int main(int argc, char **argv) {
  av_register_all();

  // Setup loggers
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  // init ros
  ros::init(argc, argv, "serdp_player_ROS");
  ros::NodeHandle nh_;

  ros::Publisher imgPub = nh_.advertise<sensor_msgs::Image>("camera_image", 1);
  ros::Publisher sonarPub =
      nh_.advertise<imaging_sonar_msgs::ImagingSonarMsg>("sonar_msg", 1);

  std::string inputFilename("");
  nh_.getParam("mov_filename", inputFilename);

  bool display(true);
  nh_.getParam("display", display);

  char *filename[inputFilename.size() + 1];
  strcpy(*filename, inputFilename.c_str());

  if (inputFilename.empty()) {
    LOG(FATAL) << "Blank inputfile";
    return -1;
  }

  LOG(INFO) << "Recieved input file " << inputFilename;

  MovDecoder movDecoder;
  // std::unique_ptr<active_object::Active> _thread;

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

  while (av_read_frame(movDecoder.pFormatCtx, &packet) >= 0 && ros::ok()) {
    // Read through packets, decode as either video or GPMF
    DecodedPacket decodedPacket =
        movDecoder.decodePacket(packet, streamCodecVec);
    //
    if (display) {
      if (decodedPacket.data.img.rows > 60 | decodedPacket.data.img.cols > 60) {
        // Display
        cv::imshow(decodedPacket.name, decodedPacket.data.img);
        cv::waitKey(1);
      } else {
        LOG(DEBUG) << "No valid image found";
      }
    }
    //
    if (decodedPacket.type == AVMEDIA_TYPE_VIDEO) {

      sensor_msgs::ImagePtr ros_img =
          ROSEncode::img2ROS(decodedPacket.data.img);
      imgPub.publish(ros_img);
    } else if (decodedPacket.type == AVMEDIA_TYPE_GPMF) {
      imaging_sonar_msgs::ImagingSonarMsg sonar_img =
          ROSEncode::GPMF2ROS(decodedPacket.data.sonarData);
      sonarPub.publish(sonar_img);
    } else {
      LOG(WARNING) << "Invalid data type decoded";
    }

    av_free_packet(&packet);
  }

  return 0;
}
