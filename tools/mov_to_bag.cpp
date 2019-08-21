#pragma once

#include "ros/ros.h"

#include "imaging_sonar_msgs/ImagingSonarMsg.h"
#include "sensor_msgs/Image.h"

#include <rosbag/bag.h>

#include <iostream>
#include <string>

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

class MovToBag {
  std::string save_bag_location;
  rosbag::Bag bag;

public:
  MovToBag(std::string _save_bag_location);
  ~MovToBag();
  void run();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg, std::string camera);
  void sonarCallback(const imaging_sonar_msgs::ImagingSonarMsgPtr &msg);
};

MovToBag::MovToBag(std::string _save_bag_location)
    : save_bag_location(_save_bag_location) {
  bag.open(save_bag_location, rosbag::bagmode::Write);
}

MovToBag::~MovToBag() { bag.close(); }

void MovToBag::imageCallback(const sensor_msgs::ImageConstPtr &msg,
                             std::string camera) {
  std::string topicName;
  if (camera == "left")
    topicName = "left_image";
  else if (camera == "right")
    topicName = "right_image";

  bag.write(topicName, ros::Time::now(), *msg);
}

void MovToBag::sonarCallback(
    const imaging_sonar_msgs::ImagingSonarMsgPtr &msg) {
  std::string topicName("sonar_image");

  bag.write(topicName, ros::Time::now(), *msg);
}

// MovToBag::run() {
//   ros::Rate loop_rate(10);
//   while (ros::ok()) {
//
//     loop_rate.sleep();
//   }
// }

int main(int argc, char **argv) {
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  ros::init(argc, argv, "mov_to_bag");
  ros::NodeHandle nh_("mov_to_bag");

  std::string save_bag_location = "";
  nh_.getParam(nh_.resolveName("save_bag_location"), save_bag_location);
  if (save_bag_location.size() == 0)
    LOG(FATAL) << "no bag location save path specified";

  MovToBag movToBag(save_bag_location);
  ros::Subscriber leftImageSub = nh_.subscribe<sensor_msgs::Image>(
      "camera_image_left", 1,
      boost::bind(&MovToBag::imageCallback, &movToBag, _1, "left"));

  ros::Subscriber rightImageSub = nh_.subscribe<sensor_msgs::Image>(
      "camera_image_right", 1,
      boost::bind(&MovToBag::imageCallback, &movToBag, _1, "right"));
  ros::Subscriber sonarSub =
      nh_.subscribe("sonar_msg", 1, &MovToBag::sonarCallback, &movToBag);

  ros::spin();

  return 0;
}
