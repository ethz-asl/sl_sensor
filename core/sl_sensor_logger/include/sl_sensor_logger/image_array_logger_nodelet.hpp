#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>

#include "sl_sensor_logger/logger_nodelet.hpp"

namespace sl_sensor {
namespace logger {
class ImageArrayLoggerNodelet : public sl_sensor::logger::LoggerNodelet {
 public:
  ImageArrayLoggerNodelet();

 private:
  ros::Subscriber image_array_sub_;
  std::string image_array_sub_topic_ = "/image_array_receive";
  std::string log_directory_ = "";
  std::string file_header_ = "";
  const std::string file_format_ = ".bmp";

  virtual void onInit() override;

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr);
};

}  // namespace logger
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::logger::ImageArrayLoggerNodelet, nodelet::Nodelet);
