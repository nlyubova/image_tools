/*
 * Copyright 2017 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class Converter
{
public:
  Converter();

  //convert an image to a required format
  void imageConvert(const sensor_msgs::ImageConstPtr& img);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  //required encoding
  std::string encoding_;

  //if the class is initialized
  bool initialized_;

  //image subscriber
  image_transport::Subscriber sub_img_;

  //image publisher
  image_transport::Publisher pub_;

  //final image
  cv::Mat image_out_;

  //final ROS message
  sensor_msgs::ImagePtr msg_out_;

  //initialize the output image
  void init(const int &w, const int&h);
};

#endif // EVALUATE_DEPTH_HPP
