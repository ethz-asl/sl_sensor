#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

#include <opencv2/opencv.hpp>
#include <string>

#include "sl_sensor_image_acquisition/ImageArray.h"

namespace sl_sensor
{
namespace calibration
{
class CalibrationDataAcquisitionNodelet : public nodelet::Nodelet
{
public:
  CalibrationDataAcquisitionNodelet();

private:
  ros::Subscriber image_array_sub_;
  std::vector<std::vector<ros::Publisher>> image_pubs_;
  ros::ServiceClient image_synchroniser_client_;

  boost::shared_ptr<boost::thread> initialisation_thread_ptr_;

  std::string image_array_sub_topic_ = "/image_array_receive";
  std::string save_directory_ = "/";
  std::string save_filename_ = "calibration_session";
  std::string image_synchroniser_service_name_;
  int checkerboard_num_rows_ = 10;
  int checkerboard_num_cols_ = 10;
  cv::Size checkerboard_size_;
  int number_cameras_ = 1;
  int counter_ = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  virtual void onInit();

  void ImageArrayCb(const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr);

  bool GetInputAndCheckIfItIsExpectedChar(const std::string& message, char expected_char);

  void ProcessFloatImage(const cv::Mat& input_image, cv::Mat& output_image);

  bool GenerateDataFolders();

  void Init();

  void SendCommandForNextCalibrationSequence();
};

}  // namespace calibration
}  // namespace sl_sensor

PLUGINLIB_EXPORT_CLASS(sl_sensor::calibration::CalibrationDataAcquisitionNodelet, nodelet::Nodelet);
