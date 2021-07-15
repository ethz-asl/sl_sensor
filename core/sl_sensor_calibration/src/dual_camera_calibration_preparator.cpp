#include "sl_sensor_calibration/dual_camera_calibration_preparator.hpp"

#include "sl_sensor_calibration/calibration_utils.hpp"

namespace sl_sensor
{
namespace calibration
{
DualCameraCalibrationPreparator::DualCameraCalibrationPreparator(const ProjectorParameters& proj_params,
                                                                 const CameraParameters& pri_cam_params,
                                                                 const CameraParameters& sec_cam_params)
  : proj_params_(proj_params), pri_cam_params_(pri_cam_params), sec_cam_params_(sec_cam_params)
{
  // Compute Projection Matrix of Projector wrt Primary Cam
  cv::Mat projection_matrix_projector_ = pri_cam_params_.GetProjectionMatrix();

  // Compute Projection Matrix of Secondary Cam wrt Primary Cam
  cv::Mat transformation_matrix_sec_cam_to_projector = sec_cam_params_.GetInverseTransformationMatrix();
  cv::Mat transformation_matrix_projector_to_pri_cam = pri_cam_params_.GetTransformationMatrix();
  cv::Mat transformation_matrix_sec_cam_to_pri_cam =
      transformation_matrix_sec_cam_to_projector * transformation_matrix_projector_to_pri_cam;
  cv::Mat projection_matrix_sec_cam_ =
      cv::Mat(sec_cam_params_.intrinsic_mat()) * transformation_matrix_sec_cam_to_pri_cam;
}

bool DualCameraCalibrationPreparator::AddSingleCalibrationSequence(
    const cv::Mat& pri_camera_shading, const cv::Mat& pri_camera_mask, const cv::Mat& pri_up, const cv::Mat& pri_vp,
    const cv::Mat& sec_camera_shading, const cv::Mat& sec_camera_mask, const cv::Mat& sec_up, const cv::Mat& sec_vp,
    const std::string& label)
{
  // TODO: Has a lot or similar code to Calibrator, refactor to reduce code repetition

  cv::Size checkerboard_size(checkerboard_cols_, checkerboard_rows_);

  // Extract Checkerboard From Primary Camera, return false if fail
  std::vector<cv::Point2f> pri_camera_checkerboard_corners;
  bool pri_checkerboard_detected =
      FindCheckerboardAndRefineCorners(pri_camera_shading, checkerboard_size, pri_camera_checkerboard_corners);
  if (!pri_checkerboard_detected)
  {
    std::cout << "[Calibrator] No checkerboard detected for primary camera" << std::endl;
    return false;
  }

  // Extract Checkerboard from Secondary Camera, return false if fail
  std::vector<cv::Point2f> sec_camera_checkerboard_corners;
  bool sec_checkerboard_detected =
      FindCheckerboardAndRefineCorners(sec_camera_shading, checkerboard_size, sec_camera_checkerboard_corners);
  if (!sec_checkerboard_detected)
  {
    std::cout << "[Calibrator] No checkerboard detected for secondary camera" << std::endl;
    return false;
  }

  // Check that orientation of checkerboards are correct. If not, flip them
  OrientCheckerBoardCorners(pri_camera_checkerboard_corners);
  OrientCheckerBoardCorners(sec_camera_checkerboard_corners);

  // Generate a vector of checkerboard 3d points, from the top left corners and going row-wise downwards
  std::vector<cv::Point3f> all_checkerboard_3d_corners;
  for (unsigned int h = 0; h < checkerboard_rows_; h++)
  {
    for (unsigned int w = 0; w < checkerboard_cols_; w++)
    {
      all_checkerboard_3d_corners.push_back(cv::Point3f(checkerboard_size_mm_ * w, checkerboard_size_mm_ * h, 0.0));
    }
  }

  // Initialise some variables required for next few steps
  std::vector<cv::Point2f> current_projector_corners;
  std::vector<cv::Point2f> current_pri_camera_corners;
  std::vector<cv::Point2f> current_sec_camera_corners;
  std::vector<cv::Point3f> current_3d_corners;

  // For each checkerboard corner
  for (unsigned int j = 0; j < all_checkerboard_3d_corners.size(); j++)
  {
    // Extract projector coordinate for primary camera, continue if fail
    const cv::Point2f& processed_pri_camera_corner = pri_camera_checkerboard_corners[j];
    std::vector<cv::Point2f> pri_neighbourhood_camera_coordinates, pri_neighbourhood_projector_coordinates;

    cv::Point2f pri_processed_projector_corner;
    bool pri_projector_coordinate_extracted = ExtractProjectorCoordinateUsingLocalHomography(
        processed_pri_camera_corner, pri_camera_mask, pri_up, pri_vp, window_radius_, minimum_valid_pixels_,
        pri_processed_projector_corner);

    if (!pri_projector_coordinate_extracted)
    {
      continue;
    }

    // Extract projector coordinate for secondary camera, continue if fail
    const cv::Point2f& processed_sec_camera_corner = sec_camera_checkerboard_corners[j];
    std::vector<cv::Point2f> sec_neighbourhood_camera_coordinates, sec_neighbourhood_projector_coordinates;

    cv::Point2f sec_processed_projector_corner;
    bool sec_projector_coordinate_extracted = ExtractProjectorCoordinateUsingLocalHomography(
        processed_sec_camera_corner, sec_camera_mask, sec_up, sec_vp, window_radius_, minimum_valid_pixels_,
        sec_processed_projector_corner);

    if (!sec_projector_coordinate_extracted)
    {
      continue;
    }

    // Consistency check for using extracted projector coordinates, continue if fail

    // Triangulate 3D coordinate of corner, wrt to primary camera

    // Append information to storage vectors
  }
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