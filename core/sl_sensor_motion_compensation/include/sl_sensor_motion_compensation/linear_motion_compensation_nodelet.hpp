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

#ifndef SL_SENSOR_MOTION_COMPENSATION_LINEAR_MOTION_COMPENSATION_NODELET_HPP_
#define SL_SENSOR_MOTION_COMPENSATION_LINEAR_MOTION_COMPENSATION_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "sl_sensor_motion_compensation/phase_correlation_utilities.hpp"

namespace sl_sensor {
namespace motion_compensation {

/**
 * @brief Nodelet that subscribes to a image array topic, performs linear motion compensation and
 * then publishes the processed images as an image array. Currently only works for monochromatic
 * images
 */
class LinearMotionCompensationNodelet : public nodelet::Nodelet {
 public:
  LinearMotionCompensationNodelet();

 private:
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;

  std::string sub_topic_ = "/image_array_input";
  std::string pub_topic_ = "/image_array_output";
  std::string pattern_direction_ = "both";
  std::string filter_id_ = "";  // If this is non empty, nodelet will only process image arrays with
                                // an id field that matches filter_id. Used when you want different
                                // pipelines for different patterns/camera feeds
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

#endif  // SL_SENSOR_MOTION_COMPENSATION_LINEAR_MOTION_COMPENSATION_NODELET_HPP_
