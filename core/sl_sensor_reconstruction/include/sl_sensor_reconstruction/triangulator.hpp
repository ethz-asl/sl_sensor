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
   * @param projector_parameters projector parameters
   * @param camera_parameters  camera parameters
   */
  Triangulator(calibration::ProjectorParameters projector_parameters, calibration::CameraParameters camera_parameters);

  /**
   * @brief Get the Calibration Data
   *
   * @return calibration::CalibrationData
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr Triangulate(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
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

  calibration::ProjectorParameters projector_parameters_;
  calibration::CameraParameters camera_parameters_;
  cv::Mat determinant_tensor_;
  cv::Mat uc_, vc_;
  cv::Mat cam_matrix_projector, cam_matrix_camera;
  cv::Mat lens_map_1_, lens_map_2_;
  cv::Mat proj_points_cam_;
  std::vector<cv::Mat> xyzw_precompute_offset_;
  std::vector<cv::Mat> xyzw_precompute_factor_;
  int number_pixels_;
};

}  // namespace reconstruction

}  // namespace sl_sensor
