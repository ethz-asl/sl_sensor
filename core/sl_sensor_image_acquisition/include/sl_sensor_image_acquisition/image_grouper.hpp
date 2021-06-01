#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace image_acquisition
{
class ImageGrouper
{
public:
  ImageGrouper(std::string image_sub_topic = "/image", int number_images_per_group = 1,
               double image_trigger_period = 0.01, double lower_bound_tol = 0.01, double upper_bound_tol = 0.01);

  void Init(ros::NodeHandle nh);

  void ImageCb(const sensor_msgs::ImageConstPtr& image_ptr);

  void Start();

  void Stop();

  void SetToleranceBounds(double lower_tol, double upper_tol);

  void SetImagesPerGroup(int number_images_per_group);

  void SetImageTriggerPeriod(double trigger_period);

  bool RetrieveImageGroup(const ros::Time& projector_time, std::vector<sensor_msgs::ImageConstPtr>& result_image_vec);

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;

  boost::mutex mutex_;

  std::string image_sub_topic_;

  std::vector<sensor_msgs::ImageConstPtr> image_ptr_buffer_ = {};

  bool is_running_ = false;
  int number_images_per_group_;
  double image_trigger_period_;
  double lower_bound_tol_;
  double upper_bound_tol_;

  void ClearAllImagesFromBufferBeforeTiming(const ros::Time& target_time);

  bool GetImagePtrFromBuffer(const ros::Time& target_time, sensor_msgs::ImageConstPtr& image_it);
};
}  // namespace image_acquisition
}  // namespace sl_sensor