#include "sl_sensor_slstudio/image_undistorter.h"

namespace sl_sensor
{
namespace slstudio
{
Image_undistorter::Image_undistorter(const CalibrationData& calib_data) : m_calib_data(calib_data)
{
  // Precompute lens correction maps
  cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
  cv::initUndistortRectifyMap(m_calib_data.Kc, m_calib_data.kc, eye, m_calib_data.Kc,
                              cv::Size(m_calib_data.frameWidth, m_calib_data.frameHeight), CV_16SC2, m_lens_map_1,
                              m_lens_map_2);
}

void Image_undistorter::undistort_image(cv::Mat& input_image, cv::Mat& output_image)
{
  cv::remap(input_image, output_image, m_lens_map_1, m_lens_map_2, cv::INTER_LINEAR);
}

void Image_undistorter::undistort_image_sequence(std::vector<cv::Mat>& input_image_sequence,
                                                 std::vector<cv::Mat>& output_image_sequence)
{
  output_image_sequence.clear();

  for (auto& input_image : input_image_sequence)
  {
    cv::Mat temp;
    output_image_sequence.push_back(temp);
    undistort_image(input_image, output_image_sequence.back());
  }
}

}  // namespace slstudio
}  // namespace sl_sensor