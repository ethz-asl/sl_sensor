#include "sl_sensor_calibration/calibrator.hpp"

#include <algorithm>

namespace sl_sensor
{
namespace calibration
{
Calibrator::Calibrator(){};

void Calibrator::SetProjectorResolution(unsigned int projector_cols, unsigned int projector_rows)
{
  resolution_x_proj_ = projector_cols;
  resolution_y_proj_ = projector_rows;
}

void Calibrator::SetCheckerboardInformation(unsigned int checkerboard_cols, unsigned int checkerboard_rows,
                                            unsigned int checkerboard_size_mm)
{
  checkerboard_cols_ = checkerboard_cols;
  checkerboard_rows_ = checkerboard_rows;
  checkerboard_size_mm_ = checkerboard_size_mm;
}

void Calibrator::SetCameraCalibrationOption(const CalibrationOption& camera_calibration_option)
{
  camera_calibration_option_ = camera_calibration_option;
}

void Calibrator::SetProjectorCalibrationOption(const CalibrationOption& projector_calibration_option)
{
  projector_calibration_option_ = projector_calibration_option;
}

void Calibrator::SetLocalHomographySettings(unsigned int window_radius, unsigned int minimum_valid_pixels)
{
  window_radius_ = window_radius;
  minimum_valid_pixels_ = minimum_valid_pixels;
}

bool Calibrator::AddSingleCalibrationSequence(const cv::Mat& camera_shading, const cv::Mat& camera_mask,
                                              const std::string& label, const cv::Mat& up, const cv::Mat& vp)
{
  // Check that projector up and vp are provided if we need to calibrate it
  if (up.empty() || vp.empty())
  {
    std::cout << "[Calibrator] No projector coordinates provided." << std::endl;

    return false;
  }

  // Detect checkerboard
  cv::Size checkerboard_size(checkerboard_cols_, checkerboard_rows_);
  std::vector<cv::Point2f> camera_checkerboard_corners;
  bool checkerboard_detected = cv::findChessboardCorners(camera_shading, checkerboard_size, camera_checkerboard_corners,
                                                         cv::CALIB_CB_ADAPTIVE_THRESH);
  if (!checkerboard_detected)
  {
    std::cout << "No checkerboard detected" << std::endl;
    return false;
  }

  cv::cornerSubPix(camera_shading, camera_checkerboard_corners, cv::Size(5, 5), cv::Size(1, 1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.01));

  // If checkerboard detected successfully, we proceed to populating the storage variables
  std::vector<cv::Point3f> all_checkerboard_3d_corners;
  for (unsigned int h = 0; h < checkerboard_rows_; h++)
  {
    for (unsigned int w = 0; w < checkerboard_cols_; w++)
    {
      all_checkerboard_3d_corners.push_back(cv::Point3f(checkerboard_size_mm_ * w, checkerboard_size_mm_ * h, 0.0));
    }
  }

  // Otherwise, we compute projector corner coordinates usings local homography method
  std::vector<cv::Point2f> current_projector_corners;
  std::vector<cv::Point2f> current_camera_corners;
  std::vector<cv::Point3f> current_3d_corners;

  resolution_x_cam_ = camera_shading.cols;
  resolution_y_cam_ = camera_shading.rows;

  // Loop through checkerboard corners
  for (unsigned int j = 0; j < all_checkerboard_3d_corners.size(); j++)
  {
    const cv::Point2f& processed_camera_corner = camera_checkerboard_corners[j];

    // Collect neighbourhood points about corner that is currently processed
    std::vector<cv::Point2f> neighbourhood_camera_coordinates, neighbourhood_projector_coordinates;

    // Avoid going out of bounds
    unsigned int start_h = std::max(int(processed_camera_corner.y + 0.5) - window_radius_, 0u);
    unsigned int stop_h = std::min(int(processed_camera_corner.y + 0.5) + window_radius_, resolution_y_cam_ - 1);
    unsigned int start_w = std::max(int(processed_camera_corner.x + 0.5) - window_radius_, 0u);
    unsigned int stop_w = std::min(int(processed_camera_corner.x + 0.5) + window_radius_, resolution_x_cam_ - 1);

    for (unsigned int h = start_h; h <= stop_h; h++)
    {
      for (unsigned int w = start_w; w <= stop_w; w++)
      {
        // Only consider neighbourhood pixels that are within the mask
        if (camera_mask.at<bool>(h, w))
        {
          neighbourhood_camera_coordinates.push_back(cv::Point2f(w, h));

          float neighbourhood_projector_u = up.at<float>(h, w);
          float neighbourhood_projector_v = vp.at<float>(h, w);
          neighbourhood_projector_coordinates.push_back(
              cv::Point2f(neighbourhood_projector_u, neighbourhood_projector_v));
        }
      }
    }

    // If there are enough legitimate points around the corner, we perform local homography
    if (neighbourhood_projector_coordinates.size() >= minimum_valid_pixels_)
    {
      cv::Mat H = cv::findHomography(neighbourhood_camera_coordinates, neighbourhood_projector_coordinates, cv::LMEDS);
      if (!H.empty())
      {
        // Compute corresponding projector corner coordinate
        cv::Point3d Q =
            cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(processed_camera_corner.x, processed_camera_corner.y, 1.0))));
        cv::Point2f processed_projector_corner = cv::Point2f(Q.x / Q.z, Q.y / Q.z);

        // Store results for this corner
        current_projector_corners.push_back(processed_projector_corner);
        current_camera_corners.push_back(processed_camera_corner);
        current_3d_corners.push_back(all_checkerboard_3d_corners[j]);
      }
      else
      {
        std::cout << "Homography failed for " << j << "th corner" << std::endl;
      }
    }
  }

  // Store results to storage for this round
  if (!current_3d_corners.empty())
  {
    corner_projector_coordinates_storage_.push_back(current_projector_corners);
    corner_camera_coordinates_storage_.push_back(current_camera_corners);
    corner_3d_coordinates_storage_.push_back(current_3d_corners);
    sequence_label_storage_.push_back(label);
  }

  return (!current_3d_corners.empty()) ? true : false;
}

void Calibrator::Clear()
{
  corner_camera_coordinates_storage_.clear();
  corner_projector_coordinates_storage_.clear();
  corner_3d_coordinates_storage_.clear();
  sequence_label_storage_.clear();
}

std::pair<ProjectorParameters, CameraParameters> Calibrator::Calibrate()
{
  int number_calibration_sequences = corner_3d_coordinates_storage_.size();

  // If no calibration sequences, we return default calibration data
  if (number_calibration_sequences <= 0)
  {
    std::cout << "No calibration sequences found, could not start calibration" << std::endl;
    return std::make_pair(ProjectorParameters(), CameraParameters());
  }

  // Calibrate camera
  auto intrinsic_cam = camera_calibration_option_.intrinsics_init;
  auto lens_distortion_cam = camera_calibration_option_.lens_distortion_init;
  std::vector<cv::Mat> cam_rvecs, cam_tvecs;

  cv::Size image_size(resolution_x_cam_, resolution_y_cam_);
  double cam_error = 0.0f;

  if (!camera_calibration_option_.fix_values)
  {
    cam_error = cv::calibrateCamera(corner_3d_coordinates_storage_, corner_camera_coordinates_storage_, image_size,
                                    intrinsic_cam, lens_distortion_cam, cam_rvecs, cam_tvecs,
                                    camera_calibration_option_.GetCalibrationFlags(),
                                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
  }

  // Calibrate projector
  auto intrinsic_proj = projector_calibration_option_.intrinsics_init;
  auto lens_distortion_proj = projector_calibration_option_.lens_distortion_init;
  std::vector<cv::Mat> proj_rvecs, proj_tvecs;
  cv::Size projector_size(resolution_y_proj_, resolution_x_proj_);
  double proj_error = 0.0f;

  if (!projector_calibration_option_.fix_values)
  {
    proj_error = cv::calibrateCamera(
        corner_3d_coordinates_storage_, corner_projector_coordinates_storage_, projector_size, intrinsic_proj,
        lens_distortion_proj, proj_rvecs, proj_tvecs, projector_calibration_option_.GetCalibrationFlags(),
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));
  }
  // Calibrate extrinsics
  cv::Mat extrinsic_rot, extrinsic_trans, E, F;
  double stereo_error = cv::stereoCalibrate(
      corner_3d_coordinates_storage_, corner_camera_coordinates_storage_, corner_projector_coordinates_storage_,
      intrinsic_cam, lens_distortion_cam, intrinsic_proj, lens_distortion_proj, image_size, extrinsic_rot,
      extrinsic_trans, E, F, cv::CALIB_FIX_INTRINSIC,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));

  // Compute and display reprojection errors
  std::cout << "Per-view calibration errors: " << std::endl;

  std::vector<float> cam_error_per_view;
  cam_error_per_view.resize(number_calibration_sequences);
  std::vector<float> proj_error_per_view;
  proj_error_per_view.resize(number_calibration_sequences);

  for (unsigned int i = 0; i < (unsigned int)number_calibration_sequences; ++i)
  {
    int number_corners_this_sequence = (int)corner_3d_coordinates_storage_[i].size();

    // If camera and projector values were not calibrated we compute rvecs and tvecs now
    if (camera_calibration_option_.fix_values)
    {
      cam_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cam_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cv::solvePnP(cv::Mat(corner_3d_coordinates_storage_[i]), corner_camera_coordinates_storage_[i], intrinsic_cam,
                   lens_distortion_cam, cam_rvecs[i], cam_tvecs[i]);
    }

    if (projector_calibration_option_.fix_values)
    {
      proj_rvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      proj_tvecs.push_back(cv::Mat::zeros(3, 3, cv::DataType<double>::type));
      cv::solvePnP(cv::Mat(corner_3d_coordinates_storage_[i]), corner_projector_coordinates_storage_[i], intrinsic_proj,
                   lens_distortion_proj, proj_rvecs[i], proj_tvecs[i]);
    }

    // Compute camera error per view
    std::vector<cv::Point2f> qc_proj;
    cv::projectPoints(cv::Mat(corner_3d_coordinates_storage_[i]), cam_rvecs[i], cam_tvecs[i], intrinsic_cam,
                      lens_distortion_cam, qc_proj);

    float err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++)
    {
      cv::Point2f d = corner_camera_coordinates_storage_[i][j] - qc_proj[j];
      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
    cam_error_per_view[i] = (float)err / number_corners_this_sequence;

    // Compute projector error per view
    std::vector<cv::Point2f> qp_proj;
    cv::projectPoints(cv::Mat(corner_3d_coordinates_storage_[i]), proj_rvecs[i], proj_tvecs[i], intrinsic_proj,
                      lens_distortion_proj, qp_proj);
    err = 0;
    for (unsigned int j = 0; j < qc_proj.size(); j++)
    {
      cv::Point2f d = corner_projector_coordinates_storage_[i][j] - qp_proj[j];
      err += cv::sqrt(d.x * d.x + d.y * d.y);
    }
    proj_error_per_view[i] = (float)err / number_corners_this_sequence;

    std::cout << "Error " << i + 1 << ") Sequence " << sequence_label_storage_[i]
              << "):\n\tcam:" << cam_error_per_view[i] << " proj:" << proj_error_per_view[i] << std::endl;
  }

  ProjectorParameters proj_params(intrinsic_proj, lens_distortion_proj, proj_error, resolution_x_proj_,
                                  resolution_y_proj_);

  CameraParameters cam_params(intrinsic_cam, lens_distortion_cam, cam_error, resolution_x_cam_, resolution_y_cam_,
                              extrinsic_rot, extrinsic_trans, stereo_error);

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

  return std::make_pair(proj_params, cam_params);
}

}  // namespace calibration
}  // namespace sl_sensor