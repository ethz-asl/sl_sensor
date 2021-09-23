// Code adapted from SLStudio https://github.com/jakobwilm/slstudio
// Code adapted from Projector-Camera Calibration Software http://dx.doi.org/10.1109/3DIMPVT.2012.77

#include "sl_sensor_calibration/calibrator.hpp"

#include "sl_sensor_calibration/calibration_utils.hpp"

#include <algorithm>

namespace sl_sensor {
namespace calibration {
Calibrator::Calibrator(){};

void Calibrator::SetProjectorResolution(unsigned int projector_cols, unsigned int projector_rows) {
  resolution_x_proj_ = projector_cols;
  resolution_y_proj_ = projector_rows;
}

void Calibrator::SetCheckerboardInformation(unsigned int checkerboard_cols,
                                            unsigned int checkerboard_rows,
                                            double checkerboard_size) {
  checkerboard_cols_ = checkerboard_cols;
  checkerboard_rows_ = checkerboard_rows;
  checkerboard_size_ = checkerboard_size;
}

void Calibrator::SetCameraCalibrationFlags(const CalibrationFlags& camera_calibration_flags) {
  camera_calibration_flags_ = camera_calibration_flags;
}

void Calibrator::SetProjectorCalibrationFlags(const CalibrationFlags& projector_calibration_flags) {
  projector_calibration_flags_ = projector_calibration_flags;
}

void Calibrator::SetLocalHomographySettings(unsigned int window_radius,
                                            unsigned int minimum_valid_pixels) {
  window_radius_ = window_radius;
  minimum_valid_pixels_ = minimum_valid_pixels;
}

void Calibrator::SetReprojectionErrorWarningThreshold(float threshold) {
  reprojection_error_warning_threshold_ = threshold;
}

bool Calibrator::AddSingleCalibrationSequence(const cv::Mat& camera_shading,
                                              const cv::Mat& camera_mask, const cv::Mat& up,
                                              const cv::Mat& vp, const std::string& label) {
  // Detect checkerboard
  cv::Size checkerboard_size(checkerboard_cols_, checkerboard_rows_);
  std::vector<cv::Point2f> camera_checkerboard_corners;
  bool checkerboard_detected = FindCheckerboardAndRefineCorners(camera_shading, checkerboard_size,
                                                                camera_checkerboard_corners);
  if (!checkerboard_detected) {
    std::cout << "[Calibrator] No checkerboard detected" << std::endl;
    return false;
  }

  // Update camera resolution
  resolution_x_cam_ = camera_shading.cols;
  resolution_y_cam_ = camera_shading.rows;

  // Check that orientation of checkerboards are correct. If not, flip them
  OrientCheckerBoardCorners(camera_checkerboard_corners);

  // If checkerboard detected successfully, we proceed to populating the storage variables

  // Generate a vector of checkerboard 3d points, from the top left corners and going row-wise
  // downwards
  std::vector<cv::Point3f> all_checkerboard_3d_corners;
  for (unsigned int h = 0; h < checkerboard_rows_; h++) {
    for (unsigned int w = 0; w < checkerboard_cols_; w++) {
      all_checkerboard_3d_corners.push_back(
          cv::Point3f(checkerboard_size_ * w, checkerboard_size_ * h, 0.0));
    }
  }

  // Otherwise, we compute projector corner coordinates using local homography method
  std::vector<cv::Point2f> current_projector_corners;
  std::vector<cv::Point2f> current_camera_corners;
  std::vector<cv::Point3f> current_3d_corners;

  // Loop through checkerboard corners
  for (unsigned int j = 0; j < all_checkerboard_3d_corners.size(); j++) {
    // Get 2d camera coordinate for that corner
    const cv::Point2f& processed_camera_corner = camera_checkerboard_corners[j];

    // Get 2d projector coordinate for that corner using local homography
    cv::Point2f processed_projector_corner;
    bool projector_coordinate_extracted = ExtractProjectorCoordinateUsingLocalHomography(
        processed_camera_corner, camera_mask, up, vp, window_radius_, minimum_valid_pixels_,
        processed_projector_corner);

    // If 2d projector coordinate extracted successfully
    if (projector_coordinate_extracted) {
      // Store results for this corner
      current_camera_corners.push_back(processed_camera_corner);
      current_projector_corners.push_back(processed_projector_corner);
      current_3d_corners.push_back(all_checkerboard_3d_corners[j]);
    } else {
      std::cout << "[Calibrator] Homography failed for " << j << "th corner" << std::endl;
    }
  }

  // Store results to storage for this calibration sequence
  if (!current_3d_corners.empty()) {
    corner_projector_coordinates_storage_.push_back(current_projector_corners);
    corner_camera_coordinates_storage_.push_back(current_camera_corners);
    corner_3d_coordinates_storage_.push_back(current_3d_corners);
    sequence_label_storage_.push_back(label);
  }

  return (!current_3d_corners.empty()) ? true : false;
}

void Calibrator::Clear() {
  corner_camera_coordinates_storage_.clear();
  corner_projector_coordinates_storage_.clear();
  corner_3d_coordinates_storage_.clear();
  sequence_label_storage_.clear();
}

bool Calibrator::Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params,
                           std::vector<double>& projector_residuals,
                           std::vector<double>& camera_residuals) {
  camera_residuals.clear();
  projector_residuals.clear();

  int number_calibration_sequences = corner_3d_coordinates_storage_.size();

  // If no calibration sequences, we return default calibration data
  if (number_calibration_sequences <= 0) {
    std::cout << "[Calibrator] No calibration sequences found, could not start calibration"
              << std::endl;
    return false;
  }

  // Calibrate camera
  auto intrinsic_cam = camera_calibration_flags_.intrinsics_init;
  auto lens_distortion_cam = camera_calibration_flags_.lens_distortion_init;
  std::vector<cv::Mat> cam_rvecs, cam_tvecs;

  cv::Size image_size(resolution_x_cam_, resolution_y_cam_);
  double cam_error = 0.0f;

  if (!camera_calibration_flags_.fix_values) {
    cam_error = cv::calibrateCamera(
        corner_3d_coordinates_storage_, corner_camera_coordinates_storage_, image_size,
        intrinsic_cam, lens_distortion_cam, cam_rvecs, cam_tvecs,
        camera_calibration_flags_.GetCalibrationFlags(),
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
  }

  // Calibrate projector
  auto intrinsic_proj = projector_calibration_flags_.intrinsics_init;
  auto lens_distortion_proj = projector_calibration_flags_.lens_distortion_init;
  std::vector<cv::Mat> proj_rvecs, proj_tvecs;
  cv::Size projector_size(resolution_x_proj_, resolution_y_proj_);
  double proj_error = 0.0f;

  if (!projector_calibration_flags_.fix_values) {
    proj_error = cv::calibrateCamera(
        corner_3d_coordinates_storage_, corner_projector_coordinates_storage_, projector_size,
        intrinsic_proj, lens_distortion_proj, proj_rvecs, proj_tvecs,
        projector_calibration_flags_.GetCalibrationFlags(),
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
  }
  // Calibrate extrinsics
  cv::Mat extrinsic_rot, extrinsic_trans, E, F;
  double stereo_error = cv::stereoCalibrate(
      corner_3d_coordinates_storage_, corner_camera_coordinates_storage_,
      corner_projector_coordinates_storage_, intrinsic_cam, lens_distortion_cam, intrinsic_proj,
      lens_distortion_proj, image_size, extrinsic_rot, extrinsic_trans, E, F,
      cv::CALIB_FIX_INTRINSIC,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));

  // Compute and display reprojection errors
  std::cout << "Per-view calibration errors: " << std::endl;

  std::vector<float> cam_error_per_view;
  cam_error_per_view.resize(number_calibration_sequences);
  std::vector<float> proj_error_per_view;
  proj_error_per_view.resize(number_calibration_sequences);

  for (unsigned int i = 0; i < (unsigned int)number_calibration_sequences; ++i) {
    int number_corners_this_sequence = (int)corner_3d_coordinates_storage_[i].size();

    // If camera and projector values were not calibrated we compute rvecs and tvecs now
    if (camera_calibration_flags_.fix_values) {
      cam_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cam_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cv::solvePnP(cv::Mat(corner_3d_coordinates_storage_[i]),
                   corner_camera_coordinates_storage_[i], intrinsic_cam, lens_distortion_cam,
                   cam_rvecs[i], cam_tvecs[i]);
    }

    if (projector_calibration_flags_.fix_values) {
      proj_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      proj_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cv::solvePnP(cv::Mat(corner_3d_coordinates_storage_[i]),
                   corner_projector_coordinates_storage_[i], intrinsic_proj, lens_distortion_proj,
                   proj_rvecs[i], proj_tvecs[i]);
    }

    // Compute camera error per view
    std::vector<cv::Point2f> qc_proj;
    cv::projectPoints(cv::Mat(corner_3d_coordinates_storage_[i]), cam_rvecs[i], cam_tvecs[i],
                      intrinsic_cam, lens_distortion_cam, qc_proj);

    float cam_err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++) {
      cv::Point2f d = corner_camera_coordinates_storage_[i][j] - qc_proj[j];
      float reprojection_error = cv::sqrt(d.x * d.x + d.y * d.y);
      cam_err += reprojection_error;

      if (reprojection_error > reprojection_error_warning_threshold_) {
        std::cout << "Warning: " << j << "th point from image with label "
                  << sequence_label_storage_[i] << " has a camera reprojection error of "
                  << reprojection_error << ">" << reprojection_error_warning_threshold_
                  << std::endl;
      }

      camera_residuals.push_back(d.x);
      camera_residuals.push_back(d.y);
    }
    cam_error_per_view[i] = cam_err / (float)number_corners_this_sequence;

    // Compute projector error per view
    std::vector<cv::Point2f> qp_proj;
    cv::projectPoints(cv::Mat(corner_3d_coordinates_storage_[i]), proj_rvecs[i], proj_tvecs[i],
                      intrinsic_proj, lens_distortion_proj, qp_proj);
    float proj_err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++) {
      cv::Point2f d = corner_projector_coordinates_storage_[i][j] - qp_proj[j];
      float reprojection_error = cv::sqrt(d.x * d.x + d.y * d.y);
      proj_err += reprojection_error;

      if (reprojection_error > reprojection_error_warning_threshold_) {
        std::cout << "Warning: " << j << "th point from image with label "
                  << sequence_label_storage_[i] << " has a projector reprojection error of "
                  << reprojection_error << ">" << reprojection_error_warning_threshold_
                  << std::endl;
      }

      projector_residuals.push_back(d.x);
      projector_residuals.push_back(d.y);
    }
    proj_error_per_view[i] = proj_err / (float)number_corners_this_sequence;

    std::cout << "Error " << i + 1 << ") Sequence " << sequence_label_storage_[i]
              << "):\n\tcam:" << cam_error_per_view[i] << " proj:" << proj_error_per_view[i]
              << std::endl;
  }

  proj_params = ProjectorParameters(intrinsic_proj, lens_distortion_proj, proj_error,
                                    resolution_x_proj_, resolution_y_proj_);

  cam_params = CameraParameters(intrinsic_cam, lens_distortion_cam, cam_error, resolution_x_cam_,
                                resolution_y_cam_, extrinsic_rot, extrinsic_trans, stereo_error);

  // Display Calibration Results
  // Print results
  std::cout << "### Calibration Results ###" << std::endl;
  std::cout << "Camera calibration error: " << cam_error << std::endl;
  std::cout << "Projector calibration error: " << proj_error << std::endl;
  std::cout << "Stereo calibration error: " << stereo_error << std::endl;
  std::cout << "Calibrated projector parameters: " << std::endl;
  std::cout << proj_params << std::endl;
  std::cout << "Calibrated camera parameters: " << std::endl;
  std::cout << cam_params << std::endl;

  return true;
}

bool Calibrator::Calibrate(ProjectorParameters& proj_params, CameraParameters& cam_params) {
  std::vector<double> camera_residuals;
  std::vector<double> projector_residuals;
  return Calibrate(proj_params, cam_params, camera_residuals, projector_residuals);
}

}  // namespace calibration
}  // namespace sl_sensor