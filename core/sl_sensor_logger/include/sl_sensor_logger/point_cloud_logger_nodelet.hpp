#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <memory>

#include "sl_sensor_logger/logger_nodelet.hpp"

namespace sl_sensor {
namespace logger {
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
