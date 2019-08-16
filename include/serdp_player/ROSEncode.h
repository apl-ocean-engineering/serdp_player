#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "imaging_sonar_msgs/ImagingSonarMsg.h"

#include "serdp_player/MovDecoder.h"

namespace ROSEncode {
sensor_msgs::ImagePtr img2ROS(cv::Mat img);
imaging_sonar_msgs::ImagingSonarMsg
GPMF2ROS(std::shared_ptr<serdp_common::SonarData> sonarData);
} // namespace ROSEncode
