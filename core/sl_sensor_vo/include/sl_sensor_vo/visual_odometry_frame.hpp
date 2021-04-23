#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace sl_sensor
{
namespace vo
{
/**
 * @brief Struct that holds all the required information in a pattern sequence to perform VO
 *
 */
struct VisualOdometryFrame
{
  std::vector<cv::Mat> image_sequence;
  std::vector<cv::Mat> rectified_image_sequence;
  std::vector<cv::Mat> registered_image_sequence;
  std::vector<cv::Point2d> shifts;
  cv::Mat reference_image_8uc;
  std::vector<cv::KeyPoint> kps;
  std::vector<float> phases;
  cv::Mat descriptors;
  pcl::PointCloud<pcl::PointXYZ>::Ptr kps_pc_ptr =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<uint64_t> timestamps_nsec;
  uint64_t reference_image_timestamp_nsec;

  /**
   * @brief Remove points in point cloud of kps_pc_ptr and corresponding kps that are
   *  outside the designate bounding box. Note: does not affect descritpors. Should be called
   * before generating descriptors
   *
   * @param x_min
   * @param x_max
   * @param y_min
   * @param y_max
   * @param z_min
   * @param z_max
   */
  void RemoverUnreliableDepthKeypoints(double x_min, double x_max, double y_min, double y_max, double z_min,
                                       double z_max);

  /**
   * @brief Convert all points in kps_pc_ptr to a vector of openCV's cv::Point3d
   *
   * @param output_vec - Vector where 3d points will be written
   */
  void Get3DPointsCV(std::vector<cv::Point3d>& output_vec);
};

}  // namespace vo

}  // namespace sl_sensor
