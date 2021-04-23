#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "sl_sensor_slstudio/calibration_data.h"

namespace sl_sensor
{
namespace slstudio
{
class Image_undistorter
{
public:
  Image_undistorter(const CalibrationData& calib_data);

  void undistort_image(cv::Mat& input_image, cv::Mat& output_image);

  void undistort_image_sequence(std::vector<cv::Mat>& input_image_sequence,
                                std::vector<cv::Mat>& output_image_sequence);

private:
  CalibrationData m_calib_data;
  cv::Mat m_lens_map_1;
  cv::Mat m_lens_map_2;
};

}  // namespace slstudio
}  // namespace sl_sensor