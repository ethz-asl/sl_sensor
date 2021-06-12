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
class Triangulator
{
public:
  Triangulator(calibration::CalibrationData calibration);
  calibration::CalibrationData GetCalibrationData();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Triangulate(const cv::Mat &up, const cv::Mat &vp, const cv::Mat &mask,
                                                     const cv::Mat &shading);

private:
  void TriangulateFromUp(const cv::Mat &up, cv::Mat &xyz);
  void TriangulateFromVp(const cv::Mat &vp, cv::Mat &xyz);
  void TriangulateFromUpVp(const cv::Mat &up, const cv::Mat &vp, cv::Mat &xyz);
  calibration::CalibrationData calibration_data_;
  cv::Mat determinant_tensor_;
  cv::Mat uc_, vc_;
  cv::Mat Pp_, Pc_;
  cv::Mat lens_map_1_, lens_map_2_;
  cv::Mat proj_points_cam_;
  std::vector<cv::Mat> xyzw_precompute_offset_;
  std::vector<cv::Mat> xyzw_precompute_factor_;
  int number_pixels_;
};

}  // namespace reconstruction

}  // namespace sl_sensor
