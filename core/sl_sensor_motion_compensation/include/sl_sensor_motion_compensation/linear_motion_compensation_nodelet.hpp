#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "sl_sensor_motion_compensation/phase_correlation_utilities.hpp"

namespace sl_sensor {
namespace motion_compensation {
class LinearMotionCompensationNodelet : public nodelet::Nodelet {
 public:
  LinearMotionCompensationNodelet();

 private:
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;

  std::string sub_topic_ = "/image_array_input";
  std::string pub_topic_ = "/image_array_output";
  std::string pattern_direction_ = "both";
  std::string filter_id_ = "";
  ShiftingOption shifting_option_ = ShiftingOption::kBothDirectionsShifting;

  double subsample_factor_ = 1.0f;
  int reference_index_ = 1;
  int camera_index_ = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr);
};

}  // namespace motion_compensation
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::motion_compensation::LinearMotionCompensationNodelet,
                       nodelet::Nodelet);
