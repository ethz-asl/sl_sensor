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

// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_HPP_
#define SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sl_sensor_calibration/camera_parameters.hpp>
#include <sl_sensor_calibration/projector_parameters.hpp>
#include <utility>
#include <vector>

namespace sl_sensor {
namespace reconstruction {
/**
 * @brief Class object that triangulates decoded images
 *
 */
class Triangulator {
 public:
  /**
   * @brief Construct a new Triangulator object
   *
   * @param camera_parameters  camera parameters
   */
  Triangulator(const calibration::ProjectorParameters &projector_parameters,
               const calibration::CameraParameters &triangulation_camera_parameters);

  /**
   * @brief Construct a new Triangulator object
   *
   * @param camera_parameters  camera parameters
   */
  Triangulator(const calibration::ProjectorParameters &projector_parameters,
               const calibration::CameraParameters &triangulation_camera_parameters,
               const calibration::CameraParameters &secondary_camera_parameters);

  /**
   * @brief Get the Projector and Camera Parameters used for triangulation
   *
   * @return calibration::ProjectorParameters and calibration::CameraParameters
   */
  std::pair<calibration::ProjectorParameters, calibration::CameraParameters> GetCalibrationParams();

  /**
   * @brief Perform triangulation, for Monochrome shading
   *
   * @param up - Horizontal projector coordinates. If not applicable, provide an empty vector
   * @param vp - Vertical projector coordinates . If not applicable, provide an empty vector
   * @param mask - Masking layer
   * @param shading - Shading information, should be a monochrome image (CV_8UC1)
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr - Pointer to triangulated point cloud, shading is
   * in the intensity field
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr TriangulateMonochrome(const cv::Mat &up, const cv::Mat &vp,
                                                             const cv::Mat &mask,
                                                             const cv::Mat &shading);

  /**
   * @brief Perform triangulation, for rgb shading
   *
   * @param up - Horizontal projector coordinates. If not applicable, provide an empty vector
   * @param vp - Vertical projector coordinates . If not applicable, provide an empty vector
   * @param mask - Masking layer
   * @param shading - Shading information, should be a rgb image (CV_8UC3)
   * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr - Pointer to triangulated point cloud, shading
   * is in the rgb fields
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr TriangulateColour(const cv::Mat &up, const cv::Mat &vp,
                                                           const cv::Mat &mask,
                                                           const cv::Mat &colour_shading);

 private:
  /**
   * @brief Perform triangulation using horizontal projector coordinates
   *
   * @param up - Decoded horizontal pixel coordinate
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   */
  void TriangulateFromUp(const cv::Mat &up, cv::Mat &xyz);

  /**
   * @brief Perform triangulation using vertical projector coordinates
   *
   * @param vp - Decoded vertical pixel coordinate
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   */
  void TriangulateFromVp(const cv::Mat &vp, cv::Mat &xyz);

  /**
   * @brief Perform triangulation using both horizontal and vertical projector coordinates
   *
   * @param up - Decoded horizontal pixel coordinate
   * @param vp - Decoded vertical pixel coordinate
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   */
  void TriangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp, cv::Mat &xyz);

  /**
   * @brief Perform triangulation. Will call TriangulateFromUp, TriangulateFromVp or
   * TriangulateFromUpVp based on whether up and/or vp are non-empty then apply masking to the
   * points (NAN values for pixels where mask is set to false)
   *
   * @param up - Decoded horizontal pixel coordinate
   * @param vp - Decoded vertical pixel coordinate
   * @param mask - 0 for pixels to be masked
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   */
  void Triangulate(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask, cv::Mat &xyz);

  void InitColourShadingInfo();

  void InitTriangulationParameters();

  /**
   * @brief Undistort up, vp, mask and shading using the initialised camera intrinsic parameters
   *
   * @param up
   * @param vp
   * @param mask
   * @param shading
   * @param up_undistorted
   * @param vp_undistorted
   * @param mask_undistorted
   * @param shading_undistorted
   */
  void UndistortImages(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                       const cv::Mat &shading, cv::Mat &up_undistorted, cv::Mat &vp_undistorted,
                       cv::Mat &mask_undistorted, cv::Mat &shading_undistorted);

  /**
   * @brief Convert xyz matrix from triangulation methods to pcl rgb point clouds. Note: will use
   * colour_camera_parameters_ to project 3D points to colour camera's image (colour_shading) to
   * obtain rgb information
   *
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   * @param mask - 0 for pixels to be masked
   * @param colour_shading - Colour CV_8UC3 rgb image from colour camera
   * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr - Pointer to triangulated point cloud, shading
   * is in the rgb fields
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertToColourPCLPointCloud(
      const cv::Mat &xyz, const cv::Mat &mask, const cv::Mat &colour_shading);

  /**
   * @brief  Convert xyz matrix from triangulation methods to pcl rgb point clouds.
   *
   * @param xyz - Output is a matrix with the same number of rows and cols as the image with three
   * chanels (CV_32FC3). The channels contain the x, y and z coordinates for each image pixel
   * @param mask - 0 for pixels to be masked
   * @param shading - Monochrome CV_8UC1 image to be used for shading
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr - Pointer to triangulated point cloud, shading is
   * in the intensity field
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr ConvertToMonochomePCLPointCLoud(const cv::Mat &xyz,
                                                                       const cv::Mat &mask,
                                                                       const cv::Mat &shading);

  /**
   * @brief Struct to hold information required to get rgb information from coloured image from
   * colour camera
   *
   */
  struct ColourShadingInfo {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Matx33f intrinsic_mat;
    cv::Vec<float, 5> lens_distortion;
  };

  calibration::ProjectorParameters projector_parameters_;
  calibration::CameraParameters triangulation_camera_parameters_;
  calibration::CameraParameters colour_camera_parameters_;

  ColourShadingInfo colour_shading_info_;

  cv::Mat determinant_tensor_;
  cv::Mat uc_, vc_;
  cv::Mat projection_matrix_projector_, projection_matrix_camera_;
  cv::Mat lens_map_1_, lens_map_2_;
  cv::Mat proj_points_cam_;
  std::vector<cv::Mat> xyzw_precompute_offset_;
  std::vector<cv::Mat> xyzw_precompute_factor_;
  int number_pixels_;
  bool colour_shading_enabled_ = false;
};

}  // namespace reconstruction
}  // namespace sl_sensor

#endif  // SL_SENSOR_RECONSTRUCTION_TRIANGULATOR_HPP_