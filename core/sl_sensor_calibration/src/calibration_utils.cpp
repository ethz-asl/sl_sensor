#include "sl_sensor_calibration/calibration_utils.hpp"

namespace sl_sensor
{
namespace calibration
{
void OrientCheckerBoardCorners(std::vector<cv::Point2f>& corners)
{
  // If first corner in the vector is lower than the last corner, we reverse the order of the vector so the first corner
  // is always at the top left corner of the image
  // Note: Remember that camera frame axis is located at top left corner of image
  if (corners.front().y > corners.back().y)
  {
    std::reverse(corners.begin(), corners.end());
  }
}

bool FindCheckerboardAndRefineCorners(const cv::Mat& image, const cv::Size& checkerboard_size,
                                      std::vector<cv::Point2f>& checkerboard_corners)
{
  checkerboard_corners.clear();

  bool checkerboard_detected =
      cv::findChessboardCorners(image, checkerboard_size, checkerboard_corners, cv::CALIB_CB_ADAPTIVE_THRESH);

  if (!checkerboard_detected)
  {
    return false;
  }

  cv::cornerSubPix(image, checkerboard_corners, cv::Size(5, 5), cv::Size(1, 1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.01));
  return true;
}

void GetNeighbourhoodPoints(const cv::Point2f& cam_coordinate, const cv::Mat& mask, const cv::Mat& up,
                            const cv::Mat& vp, std::vector<cv::Point2f>& neighbourhood_camera_coordinates,
                            std::vector<cv::Point2f>& neighbourhood_projector_coordinates, unsigned int window_radius)
{
  unsigned int resolution_x_cam = mask.cols;
  unsigned int resolution_y_cam = mask.rows;

  // Avoid going out of bounds
  unsigned int start_h = std::max(int(cam_coordinate.y + 0.5) - window_radius, 0u);
  unsigned int stop_h = std::min(int(cam_coordinate.y + 0.5) + window_radius, resolution_y_cam - 1);
  unsigned int start_w = std::max(int(cam_coordinate.x + 0.5) - window_radius, 0u);
  unsigned int stop_w = std::min(int(cam_coordinate.x + 0.5) + window_radius, resolution_x_cam - 1);

  for (unsigned int h = start_h; h <= stop_h; h++)
  {
    for (unsigned int w = start_w; w <= stop_w; w++)
    {
      // Only consider neighbourhood pixels that are within the mask
      if (mask.at<bool>(h, w))
      {
        neighbourhood_camera_coordinates.push_back(cv::Point2f(w, h));

        float neighbourhood_projector_u = up.at<float>(h, w);
        float neighbourhood_projector_v = vp.at<float>(h, w);
        neighbourhood_projector_coordinates.push_back(
            cv::Point2f(neighbourhood_projector_u, neighbourhood_projector_v));
      }
    }
  }
}

bool ExtractProjectorCoordinateUsingLocalHomography(const cv::Point2f& cam_coordinate, const cv::Mat& mask,
                                                    const cv::Mat& up, const cv::Mat& vp, unsigned int window_radius,
                                                    unsigned int minimum_valid_pixels,
                                                    cv::Point2f& output_proj_coordinate)
{
  std::vector<cv::Point2f> neighbourhood_camera_coordinates, neighbourhood_projector_coordinates;

  GetNeighbourhoodPoints(cam_coordinate, mask, up, vp, neighbourhood_camera_coordinates,
                         neighbourhood_projector_coordinates, window_radius);

  if (neighbourhood_projector_coordinates.size() < minimum_valid_pixels)
  {
    return false;
  }

  cv::Mat H = cv::findHomography(neighbourhood_camera_coordinates, neighbourhood_projector_coordinates, cv::LMEDS);

  if (H.empty())
  {
    return false;
  }

  // Compute corresponding projector corner coordinate
  cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(cam_coordinate.x, cam_coordinate.y, 1.0))));
  output_proj_coordinate = cv::Point2f(Q.x / Q.z, Q.y / Q.z);

  return true;
}

cv::Point2f UndistortSinglePoint(const cv::Point2f& distorted_point, const cv::Mat& intrinsic_matrix,
                                 const cv::Mat& lens_distortion_coefficients)
{
  std::vector<cv::Point2f> distorted_point_vec = { distorted_point };
  std::vector<cv::Point2f> undistorted_point_vec;
  cv::undistortPoints(distorted_point_vec, undistorted_point_vec, intrinsic_matrix, lens_distortion_coefficients);
  return undistorted_point_vec[0];
}

}  // namespace calibration
}  // namespace sl_sensor