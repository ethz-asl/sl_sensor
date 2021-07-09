#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <sl_sensor_calibration/calibration_data.hpp>
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
   * @param calibration - CalibrationData of SL Sensor
   */
  Triangulator(calibration::CalibrationData calibration);

  /**
   * @brief Get the Calibration Data
   *
   * @return calibration::CalibrationData
   */
  calibration::CalibrationData GetCalibrationData();

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

  calibration::CalibrationData calibration_data_;
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
