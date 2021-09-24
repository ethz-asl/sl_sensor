/***************************************************************************************************
 * This file is part of sl_sensor.
 *
 * sl_sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sl_sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sl_sensor.  If not, see <https://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#ifndef SL_SENSOR_LOGGER_IMAGE_ARRAY_LOGGER_NODELET_HPP_
#define SL_SENSOR_LOGGER_IMAGE_ARRAY_LOGGER_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>

#include "sl_sensor_logger/logger_nodelet.hpp"

namespace sl_sensor {
namespace logger {

/**
 * @brief Nodelet that subscribes to a topic for sl_sensor_image_acquisition::ImageArray messages
 * and saves them into separate .bmp images
 */
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

#endif  // SL_SENSOR_LOGGER_IMAGE_ARRAY_LOGGER_NODELET_HPP_
