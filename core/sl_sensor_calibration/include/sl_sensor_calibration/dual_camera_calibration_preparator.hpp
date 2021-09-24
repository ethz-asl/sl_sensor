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

#ifndef SL_SENSOR_CALIBRATION_DUAL_CAMERA_CALIBRATION_PREPARATOR_HPP_
#define SL_SENSOR_CALIBRATION_DUAL_CAMERA_CALIBRATION_PREPARATOR_HPP_

#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/intrinsic_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace sl_sensor {
namespace calibration {

/**
 * @brief Class object that takes in a calibration sequence where the calibration board is visible
 * to both cameras, extracts the checkerboard corners, and writes a bundle adjustment (BA) problem
 * files used for BA calibration (Experimental)
 *
 * Usage:
 * 1) Construct DualCameraCalibrationPreparator
 * 2) Add calibration sequences using AddSingleCalibrationSequence
 * 3) Generate BA problem file using ExportFile
 */
class DualCameraCalibrationPreparator {
 public:
  /**
   * @brief Construct a new Dual Camera Calibration Preparator object
   *
   * @param proj_params
   * @param pri_cam_params
   * @param sec_cam_params
   * @param checkerboard_cols
   * @param checkerboard_rows
   * @param checkerboard_size
   * @param projector_acceptance_tol
   */
  DualCameraCalibrationPreparator(const ProjectorParameters& proj_params,
                                  const CameraParameters& pri_cam_params,
                                  const CameraParameters& sec_cam_params,
                                  unsigned int checkerboard_cols, unsigned int checkerboard_rows,
                                  double checkerboard_size, double projector_acceptance_tol = 0.25);

  /**
   * @brief Process a single set of calibration images from both primary and secondary camera
   *
   * @param pri_camera_shading
   * @param pri_camera_mask
   * @param pri_up
   * @param pri_vp
   * @param sec_camera_shading
   * @param sec_camera_mask
   * @param sec_up
   * @param sec_vp
   * @param label - A string that would be used to identify this set of calibration images
   * @return true - Checkerboard detected and corner information stored successfully
   * @return false - Checkerbaord information failed, images will not be used for calibration
   */
  bool AddSingleCalibrationSequence(const cv::Mat& pri_camera_shading,
                                    const cv::Mat& pri_camera_mask, const cv::Mat& pri_up,
                                    const cv::Mat& pri_vp, const cv::Mat& sec_camera_shading,
                                    const cv::Mat& sec_camera_mask, const cv::Mat& sec_up,
                                    const cv::Mat& sec_vp, const std::string& label = "");

  /**
   * @brief Write BA adjustment
   *
   * @param filename - Name of BA problem file to be written
   */
  void ExportFile(const std::string& filename);

  /**
   * @brief Set the Local Homography Settings
   *
   * @param window_radius - Window size used for homography matrix
   * @param minimum_valid_pixels - Minimum number of non-masked pixels within window before we
   * perform homography
   */
  void SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels);

  /**
   * @brief Clear all intermediate data
   *
   */
  void Reset();

 private:
  ProjectorParameters proj_params_;
  CameraParameters pri_cam_params_;
  CameraParameters sec_cam_params_;

  cv::Mat projection_matrix_pri_cam_;
  cv::Mat projection_matrix_projector_;
  cv::Mat projection_matrix_sec_cam_;

  unsigned int window_radius_ = 10;
  unsigned int minimum_valid_pixels_ = 50;

  unsigned int checkerboard_cols_ = 10;
  unsigned int checkerboard_rows_ = 10;
  double checkerboard_size_ = 10;

  double projector_acceptance_tol_ = 0.25;

  std::vector<std::vector<cv::Point2f>> corner_pri_camera_coordinates_storage_;
  std::vector<std::vector<cv::Point2f>> corner_sec_camera_coordinates_storage_;
  std::vector<std::vector<cv::Point2f>> corner_projector_coordinates_storage_;
  std::vector<std::vector<cv::Point3f>> corner_3d_coordinates_storage_;
  std::vector<std::string> sequence_label_storage_;

  bool ProjectorCoordinatesConsistencyCheck(const cv::Point2f& coord_1, const cv::Point2f& coord_2);

  void WriteIntrinsics(std::ofstream& ba_file, const IntrinsicParameters& intrinsic_params);

  void WriteExtrinsics(std::ofstream& ba_file, const cv::Mat& rvec, const cv::Mat& tvec);
};

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_DUAL_CAMERA_CALIBRATION_PREPARATOR_HPP_