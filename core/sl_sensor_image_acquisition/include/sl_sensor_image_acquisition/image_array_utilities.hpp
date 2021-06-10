#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/Image.h"
#include "sl_sensor_image_acquisition/ImageArray.h"

namespace sl_sensor
{
namespace image_acquisition
{
void PublishImageArray(ros::Publisher& publisher, const std::vector<sensor_msgs::ImageConstPtr>& image_vec,
                       const ros::Time& timestamp, const std::string& frame_id);

void ConvertImgArrToCvPtrVec(const sl_sensor_image_acquisition::ImageArrayConstPtr image_array_ptr,
                             std::vector<cv_bridge::CvImageConstPtr>& cv_ptr_vec);

void PublishCvMatVec(ros::Publisher& publisher, const std::vector<cv::Mat>& cv_mat_vec, const std::string& frame_id,
                     const ros::Time& array_timestamp, const ros::Time& image_timestamp);

}  // namespace image_acquisition

}  // namespace sl_sensor