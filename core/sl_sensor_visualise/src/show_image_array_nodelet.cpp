#include "sl_sensor_visualise/show_image_array_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <sl_sensor_image_acquisition/image_array_utilities.hpp>

namespace sl_sensor {
namespace visualise {
ShowImageArrayNodelet::ShowImageArrayNodelet(){};

void ShowImageArrayNodelet::onInit() {
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", image_array_sub_topic_, image_array_sub_topic_);
  private_nh_.param<std::string>("screen_title", screen_title_, screen_title_);
  private_nh_.param<float>("scaling_factor", scaling_factor_, scaling_factor_);

  // Setup subscriber
  image_array_sub_ =
      nh_.subscribe(image_array_sub_topic_, 10, &ShowImageArrayNodelet::ImageArrayCb, this);
};

void ShowImageArrayNodelet::ImageArrayCb(
    const sl_sensor_image_acquisition::ImageArrayConstPtr& image_array_ptr) {
  std::vector<cv_bridge::CvImageConstPtr> temp_cv_ptr_vec = {};
  std::vector<cv::Mat> cv_mat_vec = {};

  image_acquisition::ConvertImgArrToCvPtrVec(image_array_ptr, temp_cv_ptr_vec);

  for (const auto cv_ptr : temp_cv_ptr_vec) {
    if (!cv_ptr->image.empty()) {
      cv::Mat processed_image;
      ProcessImage(cv_ptr->image, processed_image);
      cv_mat_vec.emplace_back(processed_image.clone());
    }
  }

  cv::Mat display_image;
  cv::hconcat(cv_mat_vec, display_image);

  cv::imshow(screen_title_, display_image);
  cv::waitKey(refresh_delay_ms_);
};

void ShowImageArrayNodelet::ProcessImage(const cv::Mat& input_image, cv::Mat& output_image) {
  cv::resize(input_image, output_image,
             cv::Size(input_image.cols * scaling_factor_, input_image.rows * scaling_factor_));

  if (input_image.type() == CV_8UC1) {
    output_image.convertTo(output_image, CV_32FC1, 1.0 / 0xff);
  }

  cv::normalize(output_image, output_image, 0, 1, cv::NORM_MINMAX);
}

}  // namespace visualise
}  // namespace sl_sensor
