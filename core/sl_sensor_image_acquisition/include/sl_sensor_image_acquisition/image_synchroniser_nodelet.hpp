#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <vector>

#include "sl_sensor_image_acquisition/CommandImageSynchroniser.h"
#include "sl_sensor_image_acquisition/ImageArray.h"
#include "sl_sensor_image_acquisition/image_grouper.hpp"

#include <sl_sensor_projector/SendProjectorCommand.h>`
#include <sl_sensor_projector/

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
  ros::ServiceServer grab_image_service_;

  std::string image_array_pub_topic_ = "";
  std::string projector_timing_sub_topic_ = "";
  std::string frame_id_ = "";

  std::string projector_yaml_directory_ = "";

  bool is_running_ = false;

  boost::mutex mutex_;

  ros::Time latest_projector_time_;

  int target_number_scans_ = 0;
  int current_number_scans_ = 0;
  int number_images_per_sequence = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ImageGrouper image_grouper_1_;
  ImageGrouper image_grouper_2_;

  YAML::Node projector_config_;

  virtual void onInit();

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  bool ProcessCommand(sl_sensor_frame_grabber::CommandImageSynchroniser::Request& req,
                      sl_sensor_frame_grabber::CommandImageSynchroniser::Response& res);

  void PublishImageArray(const std::vector<sensor_msgs::ImageConstPtr>& image_ptr_vec);

  bool GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_it);

  void ExecuteCommandHardwareTrigger();

  void ExecuteCommandSoftwareTrigger();
};

}  // namespace image_acquisition
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::image_acquisition::ImageSynchroniserNodelet, nodelet::Nodelet);
