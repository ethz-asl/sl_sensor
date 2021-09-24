#ifndef SL_SENSOR_CALIBRATION_CALIBRATION_UTILITIES_HPP_
#define SL_SENSOR_CALIBRATION_CALIBRATION_UTILITIES_HPP_

#include <opencv2/opencv.hpp>

namespace sl_sensor {
namespace calibration {

/**
 * @brief Orient corner coordinates such that the first one is at the top left of the calibration
 * board
 *
 * @param corners
 */
void OrientCheckerBoardCorners(std::vector<cv::Point2f>& corners);

/**
 * @brief Detect checkerboard from an image
 *
 * @param image
 * @param checkerboard_size
 * @param checkerboard_corners - Output vector of 2D coordinates where the corners are located, if
 * checkerboard is detected successfully
 * @return true - Checkerboard detected
 * @return false - Checkerboard not detected
 */
bool FindCheckerboardAndRefineCorners(const cv::Mat& image, const cv::Size& checkerboard_size,
                                      std::vector<cv::Point2f>& checkerboard_corners);

/**
 * @brief Get a vector of 2D camera and projector coordinates that are within window_radius of
 * cam_coordinate in the camera image. 2D points that are false in the mask are excluded.
 *
 * @param cam_coordinate - Center of window
 * @param mask - Masking image
 * @param up - Decoded horizontal projector coordinate image
 * @param vp - Decoded vertial projector coordinate image
 * @param neighbourhood_camera_coordinates
 * @param neighbourhood_projector_coordinates
 * @param window_radius
 */
void GetNeighbourhoodPoints(const cv::Point2f& cam_coordinate, const cv::Mat& mask,
                            const cv::Mat& up, const cv::Mat& vp,
                            std::vector<cv::Point2f>& neighbourhood_camera_coordinates,
                            std::vector<cv::Point2f>& neighbourhood_projector_coordinates,
                            unsigned int window_radius);

/**
 * @brief Use local homography method to extract the projector coordinate at an image coordinate
 * Refer to Projector-Camera Calibration Software http://dx.doi.org/10.1109/3DIMPVT.2012.77 for more
 * information
 * @param cam_coordinate - Image coordinate to extract corresponding projector coordinate
 * @param mask - Masking image
 * @param up - Decoded horizontal projector coordinate image
 * @param vp - Decoded vertial projector coordinate image
 * @param window_radius - Window size used for homography matrix
 * @param minimum_valid_pixels - Minimum number of non-masked pixels within window before we perform
 * homography
 * @param output_proj_coordinate - Output projector coordinate
 * @return true - Success
 * @return false - Fail
 */
bool ExtractProjectorCoordinateUsingLocalHomography(const cv::Point2f& cam_coordinate,
                                                    const cv::Mat& mask, const cv::Mat& up,
                                                    const cv::Mat& vp, unsigned int window_radius,
                                                    unsigned int minimum_valid_pixels,
                                                    cv::Point2f& output_proj_coordinate);

/**
 * @brief Undistort a single 2D point using a camera's intrinsics parameters
 *
 * @param distorted_point - Original point
 * @param intrinsic_matrix - 3x3 camera instrinsic matrix
 * @param lens_distortion_coefficients - 1x5 lens distortion matrix
 * @return cv::Point2f - Undistorted point
 */
cv::Point2f UndistortSinglePoint(const cv::Point2f& distorted_point,
                                 const cv::Mat& intrinsic_matrix,
                                 const cv::Mat& lens_distortion_coefficients);

/**
 * @brief Write all residuals (reprojection errors) to a file for further processing
 *
 * @param directory
 * @param filename
 * @param residuals
 */
void WriteResidualTextFile(const std::string& directory, const std::string& filename,
                           const std::vector<double>& residuals);

/**
 * @brief Write a vector of residuals (reprojection errors) to multiple files based on the
 * camera_indices provided
 *
 * @param directory - Directory where residual files will be written
 * @param filenames - Vector of residual filenames
 * @param residuals - Vector of residuals
 * @param camera_indices - Vector of indices that specifies which file the residual to be written
 * to. Length should be same as residuals. Values should range from 0-(N-1), N is number of
 * filenames provided
 */
void WriteResidualTextFiles(const std::string& directory, const std::vector<std::string>& filenames,
                            const std::vector<double>& residuals,
                            const std::vector<int>& camera_indices);

/**
 * @brief Takes in a 4x4 transformation matrix that has the frame A wrt B and outputs a
 * transformation matrix that maps frame B wrt A
 *
 * @param input_transformation_matrix
 * @param output_transformation_matrix
 */
void SwapFramesCVMat(const cv::Mat& input_transformation_matrix,
                     cv::Mat& output_transformation_matrix);

}  // namespace calibration
}  // namespace sl_sensor

#endif  // SL_SENSOR_CALIBRATION_CALIBRATION_UTILITIES_HPP_