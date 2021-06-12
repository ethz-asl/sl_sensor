#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <memory>

#include "sl_sensor_reconstruction/triangulator.hpp"

namespace sl_sensor
{
namespace reconstruction
{
class TriangulatorNodelet : public nodelet::Nodelet
{
public:
  TriangulatorNodelet();

private:
  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pc_pub_;
  ros::Subscriber image_array_sub_;

  std::string image_array_sub_topic_ = "/decoded_images_input";
  std::string pc_pub_topic_ = "/point_cloud_output";
  std::string calibration_filename_ = "";

  std::unique_ptr<Triangulator> triangulator_ptr_;

  int number_cameras_;
};

}  // namespace reconstruction
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::reconstruction::TriangulatorNodelet, nodelet::Nodelet);
