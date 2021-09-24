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

#ifndef SL_SENSOR_LOGGER_LOGGER_NODELET_HPP_
#define SL_SENSOR_LOGGER_LOGGER_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <mutex>
#include <thread>

#include <sl_sensor_logger/EnableLogger.h>

namespace sl_sensor {
namespace logger {

/**
 * @brief Base class for data loggers. It has a service that can be called to enable and disable the
 * logger. See EnableLogger.srv for more information
 *
 */
class LoggerNodelet : public nodelet::Nodelet {
 public:
  LoggerNodelet();

 protected:
  bool enabled_ = false;
  std::string service_name_ = "/enable_logger";
  ros::ServiceServer enable_service_;

  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  virtual void onInit();

  bool ProcessEnableLoggerService(sl_sensor_logger::EnableLogger::Request& req,
                                  sl_sensor_logger::EnableLogger::Response& res);
};

}  // namespace logger
}  // namespace sl_sensor

#endif  // SL_SENSOR_LOGGER_LOGGER_NODELET_HPP_