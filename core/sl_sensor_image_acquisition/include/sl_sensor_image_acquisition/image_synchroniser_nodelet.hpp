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

#ifndef SL_SENSOR_IMAGE_ACQUISITION_IMAGE_SYNCHRONISER_NODELET_HPP_
#define SL_SENSOR_IMAGE_ACQUISITION_IMAGE_SYNCHRONISER_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <thread>
#include <vector>

#include "sl_sensor_image_acquisition/CommandImageSynchroniser.h"
#include "sl_sensor_image_acquisition/ImageArray.h"
#include "sl_sensor_image_acquisition/NotifyBadData.h"
#include "sl_sensor_image_acquisition/image_grouper.hpp"

namespace sl_sensor {
namespace image_acquisition {
/**
 * @brief Nodelet that groups images based on projector trigger timings for multiple cameras.
 * Publishes the grouped images in an Image Array message
 *
 */
class ImageSynchroniserNodelet : public nodelet::Nodelet {
 public:
  ImageSynchroniserNodelet();

 private:
  ros::Publisher image_array_pub_;
  ros::Subscriber projector_timing_sub_;
  ros::ServiceServer synchroniser_service_;

  std::string image_array_pub_topic_ = "/image_synchroniser_output";
  std::string projector_timing_sub_topic_ = "/projector_timing";
  std::string frame_id_ = "";
  std::string projector_yaml_directory_ = "";
  std::string fixed_pattern_name_ = "";
  std::string image_synchroniser_service_name_ = "command_image_synchroniser";
  std::string projector_service_name_ = "command_projector";

  std::mutex mutex_;

  std::vector<ros::Time> projector_time_buffer_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::vector<std::unique_ptr<ImageGrouper>> image_grouper_ptrs_ = {};

  YAML::Node projector_config_;
  ros::ServiceClient projector_client_;

  double lower_bound_tol_ = 0.0f;
  double upper_bound_tol_ = 0.0f;
  double image_trigger_period_ = 0.0f;

  std::shared_ptr<std::thread> main_loop_thread_ptr_;

  bool bad_data_ = false;

  virtual void onInit();

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  /**
   * @brief Function that will be called upon a service call to start/stop operation of image
   * synchroniser
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool ProcessImageSynchroniserCommand(
      sl_sensor_image_acquisition::CommandImageSynchroniser::Request& req,
      sl_sensor_image_acquisition::CommandImageSynchroniser::Response& res);

  /**
   * @brief Function that will be called upon to freeze image synchroniser temporarily due to bad
   * quality images
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool ProcessNotifyBadData(sl_sensor_image_acquisition::NotifyBadData::Request& req,
                            sl_sensor_image_acquisition::NotifyBadData::Response& res);

  /**
   * @brief Attempt to obtain and publish an image group based when in projector is in hardware
   * trigger mode
   *
   * @return true
   * @return false
   */
  bool ExecuteCommandHardwareTrigger();

  /**
   * @brief Attempt to obtain and publish an image group based when in projector is in software
   * trigger mode
   *
   * @return true
   * @return false
   */
  bool ExecuteCommandSoftwareTrigger();

  /**
   * @brief Send a service call to the projector.
   * @note If disable projector is set, this function does not do anything
   * @param command - Command to send
   * @param pattern_no - Pattern number
   */
  void SendProjectorCommand(const std::string& command, int pattern_no);

  /**
   * @brief Struct to keep track of the state of the Synchroniser
   *
   */
  struct SynchroniserState {
    std::string pattern_name = "";
    bool is_running = false;
    bool is_hardware_trigger = false;
    double delay = 0.0f;
    int target_number_scans = 0;
    int current_number_scans = 0;
    int number_images_per_scan = 0;

    SynchroniserState(){};

    void Reset() {
      pattern_name = "";
      is_running = false;
      is_hardware_trigger = false;
      delay = 0.0f;
      target_number_scans = 0;
      current_number_scans = 0;
      number_images_per_scan = 0;
    };
  };

  SynchroniserState synchroniser_state_;

  /**
   * @brief Update loop that is called in a separate thread.
   *
   */
  void MainLoop();

  /**
   * @brief To be called when syncrhoniser is no longer in use (clear ImageGroupers, reset state of
   * Synchroniser, etc)
   *
   */
  void Cleanup();

  /**
   * @brief Move elements from a nested vector to a vector. Note: Nested vector will be empty at the
   * end of the operation
   *
   * @tparam T - Variable stored in the vectors
   * @param input - input nested vector
   * @param output - output nested vector
   */
  template <typename T>
  void MergeNestedVectors(std::vector<std::vector<T>>& input, std::vector<T>& output) {
    output.clear();

    for (int i = 0; i < (int)input.size(); i++) {
      std::move(input[i].begin(), input[i].end(), std::back_inserter(output));
      input[i].clear();
    }
  };

  void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time);

  std::vector<std::string> SplitString(const std::string& s, const std::string& delimiter);
};

}  // namespace image_acquisition
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::image_acquisition::ImageSynchroniserNodelet, nodelet::Nodelet);

#endif  // SL_SENSOR_IMAGE_ACQUISITION_IMAGE_SYNCHRONISER_NODELET_HPP_