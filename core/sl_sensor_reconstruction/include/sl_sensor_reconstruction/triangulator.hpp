#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sl_sensor_calibration/camera_parameters.hpp>
#include <sl_sensor_calibration/projector_parameters.hpp>
#include <utility>
#include <vector>

namespace sl_sensor
{
namespace reconstruction
{
/**
 * @brief Class object that triangulates decoded images
 *
 */
class Triangulator
{
public:
  /**
   * @brief Construct a new Triangulator object
   *
   * @param camera_parameters  camera parameters
   */
  Triangulator(const calibration::ProjectorParameters &projector_parameters,
               const calibration::CameraParameters &primary_camera_parameters);

  /**
   * @brief Construct a new Triangulator object
   *
   * @param camera_parameters  camera parameters
   */
  Triangulator(const calibration::ProjectorParameters &projector_parameters,
               const calibration::CameraParameters &primary_camera_parameters,
               const calibration::CameraParameters &secondary_camera_parameters);

  /**
   * @brief Get the Projector and Camera Parameters used for triangulation
   *
   * @return calibration::ProjectorParameters and calibration::CameraParameters
   */
  std::pair<calibration::ProjectorParameters, calibration::CameraParameters> GetCalibrationParams();

  /**
   * @brief Perform triangulation
   *
   * @param up - Horizontal projector coordinates. If not applicable, provide an empty vector
   * @param vp - Vertical projector coordinates . If not applicable, provide an empty vector
   * @param mask - Masking layer
   * @param shading - Shading information
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr - Pointer to triangulated point cloud
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr TriangulateMonochrome(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                                                             const cv::Mat &shading);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr TriangulateColour(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                                                           const cv::Mat &shading);

private:
  /**
   * @brief Perform triangulation using horizontal projector coordinates
   *
   * @param up
   * @param xyz
   */
  void TriangulateFromUp(const cv::Mat &up, cv::Mat &xyz);

  /**
   * @brief Perform triangulation using vertical projector coordinates
   *
   * @param vp
   * @param xyz
   */
  void TriangulateFromVp(const cv::Mat &vp, cv::Mat &xyz);

  /**
   * @brief Perform triangulation using both horizontal and vertical projector coordinates
   *
   * @param up
   * @param vp
   * @param xyz
   */
  void TriangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp, cv::Mat &xyz);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ConvertToMonochomePCLPointCLoud(const cv::Mat &xyz, const cv::Mat &mask,
                                                                       const cv::Mat &shading);

  void Triangulate(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask, cv::Mat &xyz);

  void InitColouringParameters();

  void InitTriangulationParameters();

  void UndistortImages(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask, const cv::Mat &shading,
                       cv::Mat &up_undistorted, cv::Mat &vp_undistorted, cv::Mat &mask_undistorted,
                       cv::Mat &shading_undistorted);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertToColourPCLPointCloud(const cv::Mat &xyz, const cv::Mat &mask,
                                                                      const cv::Mat &colour_shading);

  struct ColourShadingParameters
  {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Matx33f intrinsic_mat;
    cv::Vec<float, 5> lens_distortion;
  };

  calibration::ProjectorParameters projector_parameters_;
  calibration::CameraParameters primary_camera_parameters_;
  calibration::CameraParameters secondary_camera_parameters_;

  ColourShadingParameters colour_camera_parameters_;

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
