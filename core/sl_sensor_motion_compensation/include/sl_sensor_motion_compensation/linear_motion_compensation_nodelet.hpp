#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace sl_sensor
{
namespace motion_compensation
{
class LinearMotionCompensationNodelet : public nodelet::Nodelet
{
public:
  LinearMotionCompensationNodelet();

private:
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;

  std::string sub_topic_ = "/image_array_input";
  std::string pub_topic_ = "/image_array_output";

  double subsample_factor_ = 1.0f;
  int reference_indice_ = 1;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr);
};

}  // namespace motion_compensation
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::motion_compensation::LinearMotionCompensationNodelet, nodelet::Nodelet);
