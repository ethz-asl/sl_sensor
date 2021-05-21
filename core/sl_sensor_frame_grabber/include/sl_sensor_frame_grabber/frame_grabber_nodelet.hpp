#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <versavis/TimeNumbered.h>
#include <boost/thread/thread.hpp>
#include <iterator>
#include <string>
#include <vector>

#include "sl_sensor_frame_grabber/GrabImages.h"
#include "sl_sensor_frame_grabber/ImageArray.h"

namespace sl_sensor
{
namespace frame_grabber
{
class FrameGrabberNodelet : public nodelet::Nodelet
{
public:
  FrameGrabberNodelet();

private:
  ros::Publisher image_array_pub_;
  ros::Subscriber image_sub_;
  ros::Subscriber projector_timing_sub_;
  ros::ServiceServer grab_image_service_;

  std::string image_array_pub_topic_ = "";
  std::string image_sub_topic_ = "";
  std::string projector_timing_sub_topic_ = "";
  std::string frame_id_ = "";

  bool is_grabbing_ = false;

  boost::mutex mutex_;

  std::vector<sensor_msgs::ImageConstPtr> image_ptr_buffer_ = {};
  std::vector<ros::Time> projector_time_buffer_ = {};

  ros::Time start_time_;
  int target_number_frames_ = 0;
  int current_number_frames_ = 0;
  int number_images_ = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  double lower_bound_tol_ = 0.01;
  double upper_bound_tol_ = 0.01;
  double image_trigger_period_ = 0.01;

  virtual void onInit();

  void ImageCb(const sensor_msgs::ImageConstPtr& image_ptr);

  void ProjectorTimeCb(const versavis::TimeNumberedConstPtr& time_numbered_ptr);

  bool GrabImages(sl_sensor_frame_grabber::GrabImages::Request& req,
                  sl_sensor_frame_grabber::GrabImages::Response& res);

  void ClearAllProjectorTimingsFromBufferBeforeTiming(const ros::Time& target_time);
  void ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time);

  void PublishImageArray(const std::vector<sensor_msgs::ImageConstPtr>& image_ptr_vec);

  bool GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_it);

  void Update();
};

}  // namespace frame_grabber
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::frame_grabber::FrameGrabberNodelet, nodelet::Nodelet);
