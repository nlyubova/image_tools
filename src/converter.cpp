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

#include <sensor_msgs/image_encodings.h>

#include "image_tools/converter.hpp"

Converter::Converter():
  nh_(),
  it_(nh_),
  encoding_(sensor_msgs::image_encodings::RGB8),
  initialized_(false)
{
  //read parameters
  std::string topic_img_in = "/camera/color/image_raw";
  std::string topic_img_out = "/camera/rgb/image_raw";

  nh_.getParam("img_topic_in", topic_img_in);
  nh_.getParam("img_topic_out", topic_img_out);
  nh_.getParam("required_encoding", encoding_);

  ROS_INFO_STREAM("img_topic_in: " << topic_img_in);
  ROS_INFO_STREAM("img_topic_out: " << topic_img_out);
  ROS_INFO_STREAM("required_encoding: " << encoding_);

  pub_ = it_.advertise(topic_img_out.c_str(), 1);

  sub_img_ = it_.subscribe(topic_img_in.c_str(), 1, &Converter::imageConvert, this);
}

void Converter::init(const int &w, const int&h)
{
  image_out_ = cv::Mat(w, h, CV_8UC3, cv::Scalar(0, 0, 0));
}

void Converter::imageConvert(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
  }

  if ((img->encoding == sensor_msgs::image_encodings::YUV422)
    && (encoding_ == sensor_msgs::image_encodings::RGB8))
  {
    if (!initialized_)
    {
      init(img->width, img->height);
      initialized_ = true;
    }

    cvtColor(cv_ptr->image, image_out_, CV_YUV2RGB_YUYV);

    //convet to ROS message
    try
    {
      msg_out_ = cv_bridge::CvImage(std_msgs::Header(), encoding_, image_out_).toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //set the ROS message header 
    msg_out_->header.frame_id = img->header.frame_id;
    msg_out_->header.stamp = img->header.stamp;

    pub_.publish(msg_out_);
  }
}
