#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>

namespace sl_sensor
{
namespace reconstruction
{
/**
 * @brief Nodelet that saves point clouds
 *
 */
class PointCloudLoggerNodelet : public nodelet::Nodelet
{
public:
  PointCloudLoggerNodelet();

private:
  virtual void onInit();

  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber pc_sub_;

  std::string pc_sub_topic_ = "/decoded_images_input";
  std::string save_folder_ = "/point_cloud_output";
  std::string header_ = "pc_";

  unsigned int counter_ = 0;
};

}  // namespace reconstruction
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::reconstruction::PointCloudLoggerNodelet, nodelet::Nodelet);
