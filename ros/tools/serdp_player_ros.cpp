/*
 * Copyright (c) 2017-2020 Aaron Marburg <amarburg@uw.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of University of Washington nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 #include "ros/ros.h"

#include "serdp_player/ROSEncode.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

using namespace Decoder;

int main(int argc, char **argv) {
  av_register_all();

  // Setup loggers
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  // init ros
  ros::init(argc, argv, "serdp_player_ros");
  ros::NodeHandle nh_("serdp_player_ros");

  ros::Publisher imgPubLeft =
      nh_.advertise<sensor_msgs::Image>("camera_image_left", 1);
  ros::Publisher imgPubRight =
      nh_.advertise<sensor_msgs::Image>("camera_image_right", 1);
  ros::Publisher sonarPub =
      nh_.advertise<imaging_sonar_msgs::ImagingSonarMsg>("sonar_msg", 1);

  std::string inputFilename("");

  nh_.getParam(nh_.resolveName("mov_filename"), inputFilename);

  LOG(INFO) << "Recieved input file " << inputFilename;

  bool display(true);
  nh_.getParam("display", display);

  if (inputFilename.empty()) {
    LOG(FATAL) << "Blank inputfile";
    return -1;
  }

  char *filename[inputFilename.size() + 1];
  strcpy(*filename, inputFilename.c_str());

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
      if (decodedPacket.data.img.rows > 60 &&
          decodedPacket.data.img.cols > 60) {
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
      if (packet.stream_index == 0)
        imgPubLeft.publish(ros_img);
      else if (packet.stream_index == 1)
        imgPubRight.publish(ros_img);
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
