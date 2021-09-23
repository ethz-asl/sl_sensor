#include "sl_sensor_slstudio/image_undistorter.h"

namespace sl_sensor {
namespace slstudio {
ImageUndistorter::ImageUndistorter(const CalibrationData& calib_data) : calib_data_(calib_data) {
  // Precompute lens correction maps
  cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
  cv::initUndistortRectifyMap(calib_data_.Kc, calib_data_.kc, eye, calib_data_.Kc,
                              cv::Size(calib_data_.frameWidth, calib_data_.frameHeight), CV_16SC2,
                              lens_map_1_, lens_map_2_);
}

void ImageUndistorter::UndistortImage(cv::Mat& input_image, cv::Mat& output_image) {
  cv::remap(input_image, output_image, lens_map_1_, lens_map_2_, cv::INTER_LINEAR);
}

void ImageUndistorter::UndistortImageSequence(std::vector<cv::Mat>& input_image_sequence,
                                              std::vector<cv::Mat>& output_image_sequence) {
  output_image_sequence.clear();

  for (auto& input_image : input_image_sequence) {
    cv::Mat temp;
    output_image_sequence.push_back(temp);
    UndistortImage(input_image, output_image_sequence.back());
  }
}

}  // namespace slstudio
}  // namespace sl_sensor