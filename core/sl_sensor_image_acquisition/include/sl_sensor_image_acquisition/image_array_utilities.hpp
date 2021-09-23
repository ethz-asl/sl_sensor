#pragma once

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/Image.h"
#include "sl_sensor_image_acquisition/ImageArray.h"

namespace sl_sensor {
namespace image_acquisition {
/**
 * @brief Publish an vector of images as a image array message using the provided publisher
 *
 * @param publisher - ROS Publisher
 * @param image_ptr_vec - Vector of ptrs to the images to publish
 * @param timestamp - Timestamp for message header
 * @param frame_id - Frame ID for message header
 * @param number_cameras - Optional field if number of cameras need to be specified for downstream
 * processes
 * @param id - Optional field if string information (.e.g. projector pattern name) needs to be
 * specified for downstream processes
 */
void PublishImageArray(ros::Publisher& publisher,
                       const std::vector<sensor_msgs::ImageConstPtr>& image_ptr_vec,
                       const ros::Time& timestamp, const std::string& frame_id,
                       int number_cameras = 1, const std::string& id = "");

/**
 * @brief Convert an Image array message to a vector of cvImage for processing
 *
 * @param image_array_ptr - Const pointer of image array received from a subscriber callback
 * function
 * @param cv_ptr_vec - Output vector of CvImages
 */
void ConvertImgArrToCvPtrVec(const sl_sensor_image_acquisition::ImageArrayConstPtr image_array_ptr,
                             std::vector<cv_bridge::CvImageConstPtr>& cv_ptr_vec);

/**
 * @brief Publish a vector of cv::Mat as an Image Array message
 *
 * @param publisher - ROS Publisher
 * @param cv_mat_vec - Vector of cv::Mat to be published
 * @param frame_id - Frame ID for message header
 * @param array_timestamp - Timestamp for the Header of the Image array message
 * @param image_timestamp - Timestamps for the Header of the individual images
 * @param encodings_vec - Vector of strings that contains the encoding of the images (see
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
 * @param number_cameras - Optional field if number of cameras need to be specified for downstream
 * processes
 * @param id - Optional field if string information (.e.g. projector pattern name) needs to be
 * specified for downstream processes
 */
void PublishCvMatVec(ros::Publisher& publisher, const std::vector<cv::Mat>& cv_mat_vec,
                     const std::string& frame_id, const ros::Time& array_timestamp,
                     const ros::Time& image_timestamp,
                     const std::vector<std::string>& encodings_vec, int number_cameras = 1,
                     const std::string& id = "");

/**
 * @brief Convenience function to help fill in image array information except for the images itself
 *
 * @param image_array - Image array message that is to be filled
 * @param frame_id - Frame ID for message header
 * @param array_timestamp - Timestamp for the Header of the Image array message
 * @param number_cameras - Number of cameras need to be specified for downstream processes
 * @param id - String information (.e.g. projector pattern name) needs to be specified for
 * downstream
 */
void PopulateImageArrayInformation(sl_sensor_image_acquisition::ImageArray& image_array_msg,
                                   const std::string& frame_id, const ros::Time& array_timestamp,
                                   int number_cameras, const std::string& id);

}  // namespace image_acquisition
}  // namespace sl_sensor