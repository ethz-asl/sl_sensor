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

#ifndef SL_SENSOR_LOGGER_POINT_CLOUD_LOGGER_NODELET_HPP_
#define SL_SENSOR_LOGGER_POINT_CLOUD_LOGGER_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <memory>

#include "sl_sensor_logger/logger_nodelet.hpp"

namespace sl_sensor {
namespace logger {

/**
 * @brief Nodelet that subscribes to a topic for sensor_msgs::PointCloud2 messages and saves
 * individual PCs as .pcd files
 */
class PointCloudLoggerNodelet : public sl_sensor::logger::LoggerNodelet {
 public:
  PointCloudLoggerNodelet();

 private:
  virtual void onInit() override;

  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr);

  ros::Subscriber pc_sub_;

  std::string pc_sub_topic_ = "/decoded_images_input";
  std::string save_folder_ = "/point_cloud_output";
  std::string header_ = "pc_";
  std::string base_frame_id_ = "";

  std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

  bool include_timestamp_ = false;
  unsigned int counter_ = 0;
};

}  // namespace logger
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::logger::PointCloudLoggerNodelet, nodelet::Nodelet);

#endif  // SL_SENSOR_LOGGER_POINT_CLOUD_LOGGER_NODELET_HPP_