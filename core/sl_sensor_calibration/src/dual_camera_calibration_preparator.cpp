#include "sl_sensor_calibration/dual_camera_calibration_preparator.hpp"

namespace sl_sensor
{
namespace calibration
{
DualCameraCalibrationPreparator::DualCameraCalibrationPreparator(const ProjectorParameters& proj_params,
                                                                 const CameraParameters& pri_cam_params,
                                                                 const CameraParameters& sec_cam_params)
  : : proj_params_(proj_params), pri_cam_params_(pri_cam_params), sec_cam_params_(sec_cam_params)
{
  // Recompute transformation of secondary camera wrt primary camera
}

bool DualCameraCalibrationPreparator::AddSingleCalibrationSequence(
    const cv::Mat& pri_camera_shading, const cv::Mat& pri_camera_mask, const cv::Mat& sec_camera_shading,
    const cv::Mat& sec_camera_mask, const std::string& label = "", const cv::Mat& up = cv::Mat(),
    const cv::Mat& vp = cv::Mat())
{
  // Extract Checkerboard From Primary Camera

  // Return if fail

  // Extract Checkerboard from Secondary Camera

  // Return if fail

  // Check that orientation of checkerboards are correct. If not, flip them

  // For each checkerboard corner

  // Perform local homography for Primary Camera

  // Continue if fail

  // Perform local homography for Secondary Camera

  // Continue if fail

  // Consistency check for projector coordinates

  // Continue if fail

  // Triangulate 3D coordinate of corner

  // Append information to storage vectors
}

void DualCameraCalibrationPreparator::SetLocalHomographySettings(unsigned int window_radius,
                                                                 unsigned int minimum_valid_pixels)
{
  window_radius_ = window_radius;
  minimum_valid_pixels_ = minimum_valid_pixels;
}

void DualCameraCalibrationPreparator::Reset()
{
  corner_pri_camera_coordinates_storage_.clear();
  corner_sec_camera_coordinates_storage_.clear();
  corner_projector_coordinates_storage_.clear();
  corner_3d_coordinates_storage_.clear();
  sequence_label_storage_.clear();
}

void DualCameraCalibrationPreparator::Run()
{
}

void DualCameraCalibrationPreparator::ExportFile(const std::string& filename)
{
}

}  // namespace calibration

}  // namespace sl_sensor