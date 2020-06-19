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

#include "serdp_player/ROSEncode.h"

namespace ROSEncode {

sensor_msgs::ImagePtr img2ROS(cv::Mat img) {
  cv_bridge::CvImage ros_img;
  ros_img.header.stamp = ros::Time::now();
  ros_img.encoding = "bgr8";
  ros_img.image = img;

  return ros_img.toImageMsg();
}

imaging_sonar_msgs::ImagingSonarMsg GPMF2ROS(const serdp_common::AbstractSonarInterface &sonarData) {

  imaging_sonar_msgs::ImagingSonarMsg sonar_msg;

  float bearing, range;
  uint intensity;

  for (unsigned int b = 0; b < sonarData.nBearings(); b++) {
    sonar_msg.bearings.push_back( sonarData.bearing(b) );
  }

  for (unsigned int i = 0; i < sonarData.nRanges(); i++) {
    sonar_msg.ranges.push_back( sonarData.range(i) );
  }

  for (unsigned int r = 0; r < sonarData.nRanges(); r++) {
    for (unsigned int b = 0; b < sonarData.nBearings(); b++) {
      sonar_msg.v2intensities.push_back( sonarData.intensity(b,r) );
    }
  }

  return sonar_msg;
}

} // namespace ROSEncode
