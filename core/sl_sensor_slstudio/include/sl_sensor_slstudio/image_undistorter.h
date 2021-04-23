#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "sl_sensor_slstudio/calibration_data.h"

namespace sl_sensor
{
namespace slstudio
{
/**
 * @brief Class object that undistorts images
 *
 */
class ImageUndistorter
{
public:
  /**
   * @brief Construct a new Image Undistorter object
   *
   * @param calib_data - CalibrationData from SLStudio
   */
  ImageUndistorter(const CalibrationData& calib_data);

  /**
   * @brief Undistorts a single image
   *
   * @param input_image
   * @param output_image
   */
  void UndistortImage(cv::Mat& input_image, cv::Mat& output_image);

  /**
   * @brief Undistorts an entire image sequence
   *
   * @param input_image_sequence
   * @param output_image_sequence
   */
  void UndistortImageSequence(std::vector<cv::Mat>& input_image_sequence, std::vector<cv::Mat>& output_image_sequence);

private:
  CalibrationData calib_data_;
  cv::Mat lens_map_1_;
  cv::Mat lens_map_2_;
};

}  // namespace slstudio
}  // namespace sl_sensor