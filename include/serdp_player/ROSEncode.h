#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "imaging_sonar_msgs/ImagingSonarMsg.h"

namespace ROSEncode {
sensor_msgs::Image img2ROS(cv::Mat img);
imaging_sonar_msgs::ImagingSonarMsg GPMF2ROS(cv::Mat img);
} // namespace ROSEncode
