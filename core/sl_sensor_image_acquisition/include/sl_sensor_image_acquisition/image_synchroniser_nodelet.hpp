#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <yaml-cpp/yaml.h>
#include <boost/thread/thread.hpp>
#include <string>
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

  std::string image_array_pub_topic_ = "";
  std::string projector_timing_sub_topic_ = "";
  std::string frame_id_ = "";

  std::string projector_yaml_directory_ = "";

  boost::mutex mutex_;

  ros::Time latest_projector_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ImageGrouper image_grouper_1_;
  ImageGrouper image_grouper_2_;

  YAML::Node projector_config_;
  std::string projector_service_name_;
  ros::ServiceClient projector_client_;

  double lower_bound_tol_ = 0.0f;
  double upper_bound_tol_ = 0.0f;
  double image_trigger_period_ = 0.0f;

  virtual void onInit();

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  bool ProcessImageSynchroniserCommand(sl_sensor_image_acquisition::CommandImageSynchroniser::Request& req,
                                       sl_sensor_image_acquisition::CommandImageSynchroniser::Response& res);

  void PublishImageArray(const std::vector<sensor_msgs::ImageConstPtr>& image_ptr_vec);

  void ExecuteCommandHardwareTrigger();

  void ExecuteCommandSoftwareTrigger();

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
  };

  SynchroniserState synchroniser_state_;
};

}  // namespace image_acquisition
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::image_acquisition::ImageSynchroniserNodelet, nodelet::Nodelet);
