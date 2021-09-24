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

#include "sl_sensor_logger/logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace sl_sensor {
namespace logger {
LoggerNodelet::LoggerNodelet() {}

void LoggerNodelet::onInit() {
  // Get Node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Get key information from ROS params
  private_nh_.param<std::string>("service_name", service_name_, service_name_);
  private_nh_.param<bool>("initially_enabled", enabled_, enabled_);

  // Setup enable service
  enable_service_ =
      nh_.advertiseService(service_name_, &LoggerNodelet::ProcessEnableLoggerService, this);
}

bool LoggerNodelet::ProcessEnableLoggerService(sl_sensor_logger::EnableLogger::Request& req,
                                               sl_sensor_logger::EnableLogger::Response& res) {
  std::scoped_lock lock(mutex_);

  enabled_ = req.enable;
  res.success = true;

  return true;
}

}  // namespace logger
}  // namespace sl_sensor
