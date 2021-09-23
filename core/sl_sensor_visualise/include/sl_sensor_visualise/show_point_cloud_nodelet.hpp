#pragma once

#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <mutex>
#include <thread>

namespace sl_sensor
{
namespace visualise
{
/**
 * @brief Nodelet that displays a published point cloud message
 */
class ShowPointCloudNodelet : public nodelet::Nodelet
{
public:
  ShowPointCloudNodelet();

private:
  virtual void onInit();

  void Update();

  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& image_array);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber image_array_sub_;

  std::string pc_sub_topic_ = "/pc";
  std::string screen_title_ = "";
  std::string camera_parameters_filename_ = "";

  std::unique_ptr<pcl::visualization::PCLVisualizer> visualiser_ptr_;
  std::unique_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> colour_handler_ptr;
  std::shared_ptr<std::thread> main_loop_thread_ptr_;
  std::mutex mutex_;
};

}  // namespace visualise
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::visualise::ShowPointCloudNodelet, nodelet::Nodelet);
