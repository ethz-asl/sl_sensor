#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

#include "sl_sensor_frame_grabber/GrabImages.h"
#include "sl_sensor_frame_grabber/ImageArray.h"

namespace sl_sensor
{
namespace frame_grabber
{
class FrameLoggerNodelet : public nodelet::Nodelet
{
public:
  FrameLoggerNodelet();

private:
  ros::Subscriber image_array_sub_;
  std::string image_array_sub_topic_ = "/image_array_receive";
  std::string log_directory_ = "";
  std::string file_header_ = "";
  const std::string file_format_ = ".bmp";

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  virtual void onInit();

  void ImageArrayCb(const sl_sensor_frame_grabber::ImageArrayConstPtr image_arr_ptr);
};

}  // namespace frame_grabber
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::frame_grabber::FrameLoggerNodelet, nodelet::Nodelet);
