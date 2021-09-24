/***************************************************************************************************
 * This file is part of sl_sensor.
 *
 * sl_sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sl_sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sl_sensor.  If not, see <https://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#ifndef SL_SENSOR_CALIBRATION_CALIBRATION_FLAGS_HPP_
#define SL_SENSOR_CALIBRATION_CALIBRATION_FLAGS_HPP_

#include <opencv2/opencv.hpp>

namespace sl_sensor {
namespace calibration {

/**
 * @brief Struct to store information required for the OpenCV's method cv::calibrateCamera
 *
 */
struct CalibrationFlags {
  bool fix_prinicple_point = true;
  bool fix_aspect_ratio = true;
  bool fix_k2 = false;
  bool fix_k3 = false;
  bool zero_tangential_distortion = true;
  bool fix_values = false;
  bool use_initial_guess = false;
  cv::Matx33f intrinsics_init =
      cv::Matx33f(1.0f, 0.0f, 100.0f, 0.0f, 1.0f, 100.0f, 0.0f, 0.0f, 1.0f);
  cv::Vec<float, 5> lens_distortion_init{0, 0, 0, 0, 0};

  /**
   * @brief Get calibration flag int based on the booleans in the struct
   *
   * @return calibration flag
   */
  int GetCalibrationFlags() {
    int flag = 0;

    if (fix_prinicple_point) {
      flag += cv::CALIB_FIX_PRINCIPAL_POINT;
    }

    if (fix_aspect_ratio) {
      flag += cv::CALIB_FIX_ASPECT_RATIO;
    }

    if (fix_k2) {
      flag += cv::CALIB_FIX_K2;
    }

    if (fix_k3) {
      flag += cv::CALIB_FIX_K3;
    }

    if (zero_tangential_distortion) {
      flag += cv::CALIB_ZERO_TANGENT_DIST;
    }

    if (use_initial_guess) {
      flag += cv::CALIB_USE_INTRINSIC_GUESS;
    }
    return flag;
  };
};

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_CALIBRATION_FLAGS_HPP_