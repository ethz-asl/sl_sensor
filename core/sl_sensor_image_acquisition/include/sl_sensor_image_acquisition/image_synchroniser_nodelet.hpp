#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <memory>
#include <vector>

#include "sl_sensor_image_acquisition/CommandImageSynchroniser.h"
#include "sl_sensor_image_acquisition/ImageArray.h"
#include "sl_sensor_image_acquisition/image_grouper.hpp"

namespace sl_sensor
{
namespace image_acquisition
{
class ImageSynchroniserNodelet : public nodelet::Nodelet
{
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

  boost::mutex mutex_;

  std::vector<ros::Time> projector_time_buffer_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::vector<std::unique_ptr<ImageGrouper>> image_grouper_ptrs_ = {};

  YAML::Node projector_config_;
  std::string projector_service_name_;
  ros::ServiceClient projector_client_;

  double lower_bound_tol_ = 0.0f;
  double upper_bound_tol_ = 0.0f;
  double image_trigger_period_ = 0.0f;

  boost::shared_ptr<boost::thread> main_loop_thread_ptr_;

  virtual void onInit();

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  bool ProcessImageSynchroniserCommand(sl_sensor_image_acquisition::CommandImageSynchroniser::Request& req,
                                       sl_sensor_image_acquisition::CommandImageSynchroniser::Response& res);

  bool ExecuteCommandHardwareTrigger();

  bool ExecuteCommandSoftwareTrigger();

  void SendProjectorCommand(const std::string& command, int pattern_no);

  struct SynchroniserState
  {
    std::string pattern_name = "";
    bool is_running = false;
    bool is_hardware_trigger = false;
    double delay = 0.0f;
    int target_number_scans = 0;
    int current_number_scans = 0;
    int number_images_per_scan = 0;

    SynchroniserState(){};

    void Reset()
    {
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

  void MainLoop();

  void Cleanup();

  template <typename T>
  void MergeNestedVectors(std::vector<std::vector<T>>& input, std::vector<T>& output)
  {
    output.clear();

    for (int i = 0; i < (int)input.size(); i++)
    {
      std::move(input[i].begin(), input[i].end(), std::back_inserter(output));
      input[i].clear();  // TODO Need to test if clear() does the same job
    }
  };

  void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time);

  std::vector<std::string> SplitString(const std::string& s, const std::string& delimiter);
};

}  // namespace image_acquisition
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::image_acquisition::ImageSynchroniserNodelet, nodelet::Nodelet);
