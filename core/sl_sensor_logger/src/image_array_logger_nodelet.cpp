#include "sl_sensor_logger/image_array_logger_nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace sl_sensor {
namespace logger {
ImageArrayLoggerNodelet::ImageArrayLoggerNodelet() {}

void ImageArrayLoggerNodelet::onInit() {
  // Base Class onInit
  LoggerNodelet::onInit();

  // Get key information from ROS params
  private_nh_.param<std::string>("log_directory", log_directory_, log_directory_);
  private_nh_.param<std::string>("file_header", file_header_, file_header_);
  private_nh_.param<std::string>("log_topic", image_array_sub_topic_, image_array_sub_topic_);

  // Set up subscriber
  image_array_sub_ =
      nh_.subscribe(image_array_sub_topic_, 10, &ImageArrayLoggerNodelet::ImageArrayCb, this);
}

void ImageArrayLoggerNodelet::ImageArrayCb(
    const sl_sensor_image_acquisition::ImageArrayConstPtr image_arr_ptr) {
  if (enabled_) {
    std::string array_time = std::to_string(image_arr_ptr->header.stamp.toNSec());

    for (size_t i = 0; i < image_arr_ptr->data.size(); i++) {
      auto cv_img_ptr = cv_bridge::toCvShare(image_arr_ptr->data[i], nullptr);
      std::string filename =
          log_directory_ + file_header_ + "_" + array_time + "_" + std::to_string(i) + file_format_;
      cv::imwrite(filename, cv_img_ptr->image);
    }
  }
}

}  // namespace logger
}  // namespace sl_sensor
