#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <mutex>
#include <thread>

#include <sl_sensor_logger/EnableLogger.h>

namespace sl_sensor {
namespace logger {
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
