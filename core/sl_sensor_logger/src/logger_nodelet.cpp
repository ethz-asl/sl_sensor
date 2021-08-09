#include "sl_sensor_logger/logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace logger
{
LoggerNodelet::LoggerNodelet()
{
}

void LoggerNodelet::onInit()
{
  // Get Node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Get key information from ROS params
  private_nh_.param<std::string>("service_name", service_name_, service_name_);
  private_nh_.param<bool>("initially_enabled", enabled_, enabled_);

  // Setup enable service
  enable_service_ = nh_.advertiseService(service_name_, &LoggerNodelet::ProcessEnableLoggerService, this);
}

bool LoggerNodelet::ProcessEnableLoggerService(sl_sensor_logger::EnableLogger::Request& req,
                                               sl_sensor_logger::EnableLogger::Response& res)
{
  boost::mutex::scoped_lock lock(mutex_);

  enabled_ = req.enable;
  res.success = true;

  return true;
}

}  // namespace logger
}  // namespace sl_sensor
