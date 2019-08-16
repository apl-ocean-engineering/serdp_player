#include "serdp_player/ROSEncode.h"

namespace ROSEncode {

sensor_msgs::Image img2ROS(cv::Mat img) {
  sensor_msgs::Image ros_img;

  return ros_img;
}

imaging_sonar_msgs::ImagingSonarMsg GPMF2ROS(cv::Mat img) {
  imaging_sonar_msgs::ImagingSonarMsg sonar_img;

  return sonar_img;
}

} // namespace ROSEncode
