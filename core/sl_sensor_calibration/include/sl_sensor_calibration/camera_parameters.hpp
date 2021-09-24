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

#ifndef SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_
#define SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_

#include "sl_sensor_calibration/intrinsic_parameters.hpp"

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object to store Camera calibration information
 * It contains:
 * 1) Camera Intrinsic paramters
 * 2) Transformation between projector frame to camera frame
 *
 */
class CameraParameters {
 public:
  CameraParameters(const cv::Matx33f& intrinsic_mat, const cv::Vec<float, 5>& lens_distortion,
                   double calibration_error, int resolution_x, int resolution_y,
                   const cv::Matx33f& extrinsic_rot, const cv::Vec3f& extrinsic_trans,
                   double stereo_error);

  CameraParameters(const std::string& filename);

  CameraParameters();

  bool Save(const std::string& filename);

  bool Load(const std::string& filename);

  const cv::Matx33f& extrinsic_rot() const;

  const cv::Vec3f& extrinsic_trans() const;

  const double& stereo_error() const;

  cv::Mat GetTransformationMatrix() const;

  cv::Mat GetInverseTransformationMatrix() const;

  cv::Mat GetProjectionMatrix() const;

  const cv::Matx33f& intrinsic_mat() const;
  const cv::Vec<float, 5>& lens_distortion() const;
  const double& calibration_error() const;
  const int& resolution_x() const;
  const int& resolution_y() const;

  const IntrinsicParameters& intrinsic_parameters() const;

 private:
  IntrinsicParameters intrinsic_parameters_;
  cv::Matx33f extrinsic_rot_ = cv::Matx33f::eye();
  cv::Vec3f extrinsic_trans_ = cv::Vec3f(0, 0, 0);
  double stereo_error_;

  bool SaveExtrinsic(const std::string& filename);
  bool LoadExtrinsic(const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const CameraParameters& dt);

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_CAMERA_PARAMETERS_HPP_