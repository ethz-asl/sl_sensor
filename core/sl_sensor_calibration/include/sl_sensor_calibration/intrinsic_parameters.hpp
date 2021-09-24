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

// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CALIBRATION_INTRINSIC_PARAMETERS_HPP_
#define SL_SENSOR_CALIBRATION_INTRINSIC_PARAMETERS_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object to store camera intrinsic parameters
 *
 */
class IntrinsicParameters {
 public:
  IntrinsicParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                      double calibration_error, int resolution_x, int resolution_y);

  IntrinsicParameters(const std::string& filename);

  IntrinsicParameters(){};

  virtual bool Save(const std::string& filename);

  virtual bool Load(const std::string& filename);

  const cv::Matx33f& intrinsic_mat() const;
  const cv::Vec<float, 5>& lens_distortion() const;
  const double& calibration_error() const;
  const int& resolution_x() const;
  const int& resolution_y() const;

  static std::string GetFileExtension(const std::string& filename);

 protected:
  cv::Matx33f intrinsic_mat_ = cv::Matx33f::eye();
  cv::Vec<float, 5> lens_distortion_ = cv::Vec<float, 5>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  double calibration_error_ = 0.0f;
  int resolution_x_ = 0;
  int resolution_y_ = 0;
};

std::ostream& operator<<(std::ostream& os, const IntrinsicParameters& dt);

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_INTRINSIC_PARAMETERS_HPP_