#pragma once
#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace calibration
{
struct CalibrationFlag
{
  bool fix_prinicple_point = true;
  bool fix_aspect_ratio = true;
  bool fix_k2 = false;
  bool fix_k3 = false;
  bool zero_tangential_distortion = true;
  bool fix_values = false;
  bool use_initial_guess = false;
  cv::Matx33f intrinsics_init = cv::Matx33f(1.0f, 0.0f, 100.0f, 0.0f, 1.0f, 100.0f, 0.0f, 0.0f, 1.0f);
  cv::Vec<float, 5> lens_distortion_init{ 0, 0, 0, 0, 0 };

  int GetCalibrationFlags()
  {
    int flag = 0;

    if (fix_prinicple_point)
    {
      flag += cv::CALIB_FIX_PRINCIPAL_POINT;
    }

    if (fix_aspect_ratio)
    {
      flag += cv::CALIB_FIX_ASPECT_RATIO;
    }

    if (fix_k2)
    {
      flag += cv::CALIB_FIX_K2;
    }

    if (fix_k3)
    {
      flag += cv::CALIB_FIX_K3;
    }

    if (zero_tangential_distortion)
    {
      flag += cv::CALIB_ZERO_TANGENT_DIST;
    }

    if (use_initial_guess)
    {
      flag += cv::CALIB_USE_INTRINSIC_GUESS;
    }
    return flag;
  };
};

}  // namespace calibration

}  // namespace sl_sensor