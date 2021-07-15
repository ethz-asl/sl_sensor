#pragma once

#include "sl_sensor_calibration/camera_parameters.hpp"
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
                                    const cv::Mat& sec_camera_shading, const cv::Mat& sec_camera_mask,
                                    const std::string& label = "", const cv::Mat& up = cv::Mat(),
                                    const cv::Mat& vp = cv::Mat());
  void SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels);

  void Reset();

  void Run();

  void ExportFile(const std::string& filename);

private:
  ProjectorParameters proj_params_;
  CameraParameters pri_cam_params_;
  CameraParameters sec_cam_params_;

  cv::Mat projection_matrix_projector_;
  cv::Mat projection_matrix_sec_cam_;

  unsigned int window_radius_ = 10;
  unsigned int minimum_valid_pixels_ = 50;
  double projector_acceptance_tol_ = 0.1;

  std::vector<std::vector<cv::Point2f>> corner_pri_camera_coordinates_storage_;
  std::vector<std::vector<cv::Point2f>> corner_sec_camera_coordinates_storage_;
  std::vector<std::vector<cv::Point2f>> corner_projector_coordinates_storage_;
  std::vector<std::vector<cv::Point3f>> corner_3d_coordinates_storage_;
  std::vector<std::string> sequence_label_storage_;
};

}  // namespace calibration

}  // namespace sl_sensor