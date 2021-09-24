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
// Code adapted from Projector-Camera Calibration Software http://dx.doi.org/10.1109/3DIMPVT.2012.77

#ifndef SL_SENSOR_CALIBRATION_CALIBRATOR_HPP_
#define SL_SENSOR_CALIBRATION_CALIBRATOR_HPP_

#pragma once

#include <opencv2/opencv.hpp>
#include <utility>

#include "sl_sensor_calibration/calibration_flags.hpp"
#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object used to before camera-projector calibration
 * Usage:
 * 1) Construct and input the necessary information using the Set methods
 * 2) Add sequences using AddSingleCalibrationSequence
 * 3) Call Calibrate to start the calibration process
 */
class Calibrator {
 public:
  Calibrator();
  void SetProjectorResolution(unsigned int projector_cols, unsigned int projector_rows);
  void SetCheckerboardInformation(unsigned int checkerboard_cols, unsigned int checkerboard_rows,
                                  double checkerboard_size);
  void SetCalibrateCameraOnly(bool calibrate_camera_only);
  void SetCameraCalibrationFlags(const CalibrationFlags& camera_calibration_flags);
  void SetProjectorCalibrationFlags(const CalibrationFlags& projector_calibration_flags);
  void SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels);
  void SetReprojectionErrorWarningThreshold(float threshold);

  /**
   * @brief Add calibration images to be used for calibration
   *
   * @param camera_shading
   * @param camera_mask
   * @param up
   * @param vp
   * @param label - A string that would be used to identify this set of calibration images
   * @return true - Checkerboard detected and corner information stored successfully
   * @return false - Checkerbaord information failed, images will not be used for calibration
   */
  bool AddSingleCalibrationSequence(const cv::Mat& camera_shading, const cv::Mat& camera_mask,
                                    const cv::Mat& up, const cv::Mat& vp,
                                    const std::string& label = "");

  /**
   * @brief Perform calibration
   *
   * @param proj_params - Output projector parameters
   * @param cam_params - Input camera parameters
   * @param projector_residuals - Projector reprojection errors for all corners used for calibration
   * @param camera_residuals - Camera reprojection errors for all corners used for calibration
   * @return true - Successful calibration
   * @return false - Unsuccessful calibration
   */
  bool Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params,
                 std::vector<double>& projector_residuals, std::vector<double>& camera_residuals);

  /**
   * @brief Function override, in the case that residuals are note required
   *
   * @param proj_params
   * @param cam_params
   * @return true
   * @return false
   */
  bool Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params);

  /**
   * @brief Clear all calibration data current stored in class object and start over
   *
   */
  void Clear();

 private:
  unsigned int resolution_x_proj_ = 0;
  unsigned int resolution_y_proj_ = 0;
  unsigned int window_radius_ = 10;
  unsigned int minimum_valid_pixels_ = 50;
  unsigned int checkerboard_cols_ = 10;
  unsigned int checkerboard_rows_ = 10;
  double checkerboard_size_ = 10;
  unsigned int resolution_x_cam_ = 0;
  unsigned int resolution_y_cam_ = 0;
  float reprojection_error_warning_threshold_ = 1.0f;

  CalibrationFlags camera_calibration_flags_;
  CalibrationFlags projector_calibration_flags_;

  std::vector<std::vector<cv::Point2f>> corner_camera_coordinates_storage_;
  std::vector<std::vector<cv::Point2f>> corner_projector_coordinates_storage_;
  std::vector<std::vector<cv::Point3f>> corner_3d_coordinates_storage_;
  std::vector<std::string> sequence_label_storage_;
};

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_CALIBRATOR_HPP_