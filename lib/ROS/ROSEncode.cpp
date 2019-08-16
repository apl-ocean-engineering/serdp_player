#include "serdp_player/ROSEncode.h"

namespace ROSEncode {

sensor_msgs::ImagePtr img2ROS(cv::Mat img) {
  cv_bridge::CvImage ros_img;
  ros_img.header.stamp = ros::Time::now();
  ros_img.encoding = "bgr8";
  ros_img.image = img;

  return ros_img.toImageMsg();
}

imaging_sonar_msgs::ImagingSonarMsg
GPMF2ROS(std::shared_ptr<serdp_common::SonarData> sonarData) {
  imaging_sonar_msgs::ImagingSonarMsg sonar_msg;

  float bearing, range;
  uint intensity;

  float *bearings = sonarData->bearings;

  for (unsigned int b = 0; b < sonarData->nBearings; b++) {
    // bearing = *bearings;
    // sonar_msg.bearings.push_back(bearing);
    std::cout << bearings << std::endl;
    bearings++;
  }
  // for (unsigned int i = 0; i < sonarData->nRanges; i++) {
  //   range = sonarData->ranges[i];
  //   sonar_msg.ranges.push_back(range);
  // }
  // int count(0);
  // for (unsigned int r = 0; r < sonarData->nRanges; r++) {
  //   for (unsigned int b = 0; b < sonarData->nBearings; b++) {
  //     intensity = sonarData->intensities[count];
  //     sonar_msg.v2intensities.push_back(intensity);
  //
  //     count++;
  //   }
  // }

  return sonar_msg;
}

} // namespace ROSEncode
