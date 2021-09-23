// Code adapted from SLStudio
// Code adapted from Projector-Camera Calibration Software http://dx.doi.org/10.1109/3DIMPVT.2012.77

#pragma once

#include <opencv2/opencv.hpp>
#include <utility>

#include "sl_sensor_calibration/calibration_flags.hpp"
#include "sl_sensor_calibration/camera_parameters.hpp"
#include "sl_sensor_calibration/projector_parameters.hpp"

namespace sl_sensor
{
namespace calibration
{
class Calibrator
{
public:
  Calibrator();
  void SetProjectorResolution(unsigned int projector_cols, unsigned int projector_rows);
  void SetCheckerboardInformation(unsigned int checkerboard_cols, unsigned int checkerboard_rows,
                                  double checkerboard_size);
  void SetCalibrateCameraOnly(bool calibrate_camera_only);
  void SetCameraCalibrationFlags(const CalibrationFlags& camera_calibration_flags);
  void SetProjectorCalibrationFlags(const CalibrationFlags& projector_calibration_flags);
  void SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels);
  bool AddSingleCalibrationSequence(const cv::Mat& camera_shading, const cv::Mat& camera_mask, const cv::Mat& up,
                                    const cv::Mat& vp, const std::string& label = "");
  bool Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params);

  bool Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params,
                 std::vector<double>& projector_residuals, std::vector<double>& camera_residuals);
  void SetReprojectionErrorWarningThreshold(float threshold);
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