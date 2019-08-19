#include "ros/ros.h"

#include "serdp_player/ROSEncode.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include <queue>

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

  //Calculate relative video time elapsed for video stream 0
  int videoCount = 0;
  float videoInitalTimestamp;
  float videoTimeElapsed = 0;
  float videoTimeStep = (float) VIDEO_TIMESTAMP_STEP;
  float relativeVideoTimeElapsed = 0;

  //Calculate relative video time elapsed for video stream 2 (aka GPMF)
  int gpmfCount = 0;
  float gpmfInitalTimestamp;
  float gpmfTimeElapsed = 0;
  float gpmfTimeStep = (float) VIDEO_TIMESTAMP_STEP; //Definetly not correct, but inital time elapsed should never be 0
  float relativeGpmfTimeElapsed = 0;

  std::queue<AVPacket> videoPacketQueueSteam0;
  std::queue<AVPacket> videoPacketQueueSteam1;
  std::queue<AVPacket> gpmfPacketStream;

  while (av_read_frame(movDecoder.pFormatCtx, &packet) >= 0 && ros::ok()) {
    // std::cout << "stream index: " << packet.stream_index << std::endl;
    //std::cout << "timestamp: " << packet.dts << std::endl;
    if (packet.stream_index == 0){
      videoPacketQueueSteam0.push(packet);
      packet = videoPacketQueueSteam0.front();
    }
    else if (packet.stream_index == 1){
      videoPacketQueueSteam1.push(packet);
      packet = videoPacketQueueSteam1.front();
    }
    if (packet.stream_index == 2){
      gpmfPacketStream.push(packet);
      packet = gpmfPacketStream.front();
    }

    if (packet.stream_index == 0){
      //Video stream video relative time elapsed calculation
      if (videoCount == 0){
        videoInitalTimestamp = packet.dts;
      }

      else if (videoCount == 1){
        videoTimeStep = packet.dts - videoInitalTimestamp;
      }

      else{
        videoTimeElapsed = packet.dts - videoInitalTimestamp;
        relativeVideoTimeElapsed = (videoTimeElapsed/videoTimeStep);
        //std::cout << "videoTimeElapsed: " << videoTimeElapsed << std::endl;
        //std::cout << "Rel video time elapsed: " << relativeVideoTimeElapsed << std::endl;
      }
    }

    else if (packet.stream_index == 2){
      if (gpmfCount == 0){
        gpmfInitalTimestamp = packet.dts;
      }
      else if (gpmfCount == 1){
        gpmfTimeStep = packet.dts - gpmfInitalTimestamp;
      }
      else{
        gpmfTimeElapsed = packet.dts - gpmfInitalTimestamp;
        relativeGpmfTimeElapsed = (gpmfTimeElapsed/gpmfTimeStep);

        //std::cout << "Rel Gpmf time elapsed: " << relativeGpmfTimeElapsed << std::endl;

      }
    }

    if (packet.stream_index == 0 | packet.stream_index == 1)
    {
      if ((float)relativeVideoTimeElapsed > (float) relativeGpmfTimeElapsed){
        std::cout << "Rel video time elapsed1: " << relativeVideoTimeElapsed << std::endl;
        std::cout << "Rel Gpmf time elapsed1: " << relativeGpmfTimeElapsed << std::endl;
        continue;
      }
      else{
        std::cout << "Rel video time elapsed2: " << relativeVideoTimeElapsed << std::endl;
        std::cout << "Rel Gpmf time elapsed2: " << relativeGpmfTimeElapsed << std::endl;
        //bool greater = (relativeVideoTimeElapsed > gpmfTimeElapsed);
        //std::cout <<  greater << std::endl;
      }
    }

    //
    AVPacket _packet;
    if (packet.stream_index == 0){

      _packet =  videoPacketQueueSteam0.front();
      videoPacketQueueSteam0.pop();
    }

    else if (packet.stream_index == 1){
      _packet =  videoPacketQueueSteam1.front();
      videoPacketQueueSteam1.pop();
    }

    else if (packet.stream_index == 2){

      _packet = gpmfPacketStream.front();
      gpmfPacketStream.pop();
    }
    else{
      continue;
    }
    //
    //
    // Read through packets, decode as either video or GPMF
    DecodedPacket decodedPacket =
        movDecoder.decodePacket(_packet, streamCodecVec);
    // //
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

    av_free_packet(&_packet);
    if (_packet.stream_index == 0){
      videoCount++;
    }
    else if (_packet.stream_index == 2){
      gpmfCount++;
    }
  }

  return 0;
}
