#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sl_sensor_image_acquisition/ImageArray.h>
#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace visualise
{
/**
 * @brief Nodelet that displays a published image array message
 */
class ShowImageArrayNodelet : public nodelet::Nodelet
{
public:
  ShowImageArrayNodelet();

private:
  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array);

  /**
   * @brief Prepare an input image to be displayed. Currently only supports CV_8UC1 and CV_32FC1 formats
   *
   * @param input_image
   * @param output_image
   */
  void ProcessImage(const cv::Mat& input_image, cv::Mat& output_image);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber image_array_sub_;

  std::string image_array_sub_topic_ = "/image_array";
  std::string screen_title_ = "";

  int refresh_delay_ms_ = 1;

  float scaling_factor_ = 0.5;
};

}  // namespace visualise
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::visualise::ShowImageArrayNodelet, nodelet::Nodelet);
