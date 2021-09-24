#pragma once

#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"

#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <unordered_map>

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>

namespace sl_sensor {
namespace registration {

/**
 * @brief PointCloudRegistrationAlgorithm that runs LibPointMatcher ICP
 *
 */
class LibpointmatcherICP : public PointCloudRegistrationAlgorithm {
  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;

 public:
  /**
   * @brief Construct a new Libpointmatcher ICP object
   *
   */
  LibpointmatcherICP();

  /**
   * @brief Convert PCL Point cloud to LPM point cloud
   *
   * @param pcl_pc
   * @return DP
   */
  static DP PCLToLPBPointCloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_pc);

  /**
   * @brief Convert Eigen 4x4 transformation matrix to LPM's transformation parameter
   *
   * @param fs_matrix
   * @return PM::TransformationParameters
   */
  static PM::TransformationParameters EigenToTransformationParameter(
      const Eigen::Matrix4f& fs_matrix);

  /**
   * @brief Convert LPM's transformation parameter to Eigen 4x4 transformation matrix
   *
   * @param tp
   * @return Eigen::Matrix4f
   */
  static Eigen::Matrix4f TransformationParameterToEigen(const PM::TransformationParameters& tp);

  /**
   * @brief Load ICP settings from ROS node handle
   *
   * @param nh
   */
  virtual void LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) override;

  /**
   * @brief Init ICP algorithm, to be called before RegisterPointCloud is called
   *
   */
  virtual void Init() override;

  /**
   * @brief Register point cloud (XYZ Points)
   *
   * @param reference_pc - Point cloud to be registered
   * @param guess - initial guess transform
   */
  virtual void RegisterPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc_ptr,
      const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity()) override;

 private:
  std::string yaml_directory_ = "";
  bool received_first_point_cloud_ = false;
  std::shared_ptr<DP> prev_cloud_ptr_ = std::make_shared<DP>();
  std::shared_ptr<DP> curr_cloud_ptr_ = std::make_shared<DP>();
  PM::ICP icp_;
  std::unordered_map<std::string, double> default_settings_ = {{"leafsize", 0.8}};
};
}  // namespace registration
}  // namespace sl_sensor