#pragma once

#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/intrinsic_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace calibration
{
class DualCameraCalibrationPreparator
{
public:
  DualCameraCalibrationPreparator(const ProjectorParameters& proj_params, const CameraParameters& pri_cam_params,
                                  const CameraParameters& sec_cam_params);

  bool AddSingleCalibrationSequence(const cv::Mat& pri_camera_shading, const cv::Mat& pri_camera_mask,
                                    const cv::Mat& pri_up, const cv::Mat& pri_vp, const cv::Mat& sec_camera_shading,
                                    const cv::Mat& sec_camera_mask, const cv::Mat& sec_up, const cv::Mat& sec_vp,
                                    const std::string& label = "");

  void SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels);

  void Reset();

  void Run();

  void ExportFile(const std::string& filename);

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
  unsigned int checkerboard_size_mm_ = 10;

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