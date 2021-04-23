#pragma once

#include "sl_sensor_slstudio/calibration_data.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sl_sensor
{
namespace slstudio
{
class Triangulator
{
public:
  Triangulator(CalibrationData _calibration);
  CalibrationData getCalibration()
  {
    return calibration;
  }
  ~Triangulator()
  {
  }
  // Reconstruction
  void triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, cv::Mat &pointCloud);

  pcl::PointXYZ triangulate_vp_single_point(float uc, float vc, float vp);

  void triangulate_vp_from_keypoints(const std::vector<cv::KeyPoint> &keypoints, const std::vector<float> &vps,
                                     pcl::PointCloud<pcl::PointXYZ> &pc);

private:
  void triangulateFromUp(cv::Mat &up, cv::Mat &xyz);
  void triangulateFromVp(cv::Mat &vp, cv::Mat &xyz);
  void triangulateFromUpVp(cv::Mat &up, cv::Mat &vp, cv::Mat &xyz);
  CalibrationData calibration;
  cv::Mat determinantTensor;
  cv::Mat uc, vc;
  cv::Mat lensMap1, lensMap2;
  std::vector<cv::Mat> xyzwPrecomputeOffset;
  std::vector<cv::Mat> xyzwPrecomputeFactor;
};

}  // namespace slstudio
}  // namespace sl_sensor