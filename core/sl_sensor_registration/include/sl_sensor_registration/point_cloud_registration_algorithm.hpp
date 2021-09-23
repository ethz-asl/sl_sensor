#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "ros/ros.h"

namespace sl_sensor {
namespace registration {
/**
 * @brief Base class for point cloud registration algorithms
 *
 */
class PointCloudRegistrationAlgorithm {
 public:
  /**
   * @brief Get settings from ROS node
   *
   * @param nh
   */
  virtual void LoadSettingsFromROSNodeHandle(ros::NodeHandle nh);

  virtual void Init(){};

  /**
   * @brief Register point cloud (XYZ Points)
   *
   * @param reference_pc - Point cloud to be registered
   * @param guess - initial guess transform
   */
  virtual void RegisterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc,
                                  const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity()){};

  /**
   * @brief Register point cloud (XYZRGB points). By default, will convert point cloud to XYZ format
   * and register
   *
   * @param reference_pc - Point cloud to be registered
   * @param guess - initial guess transform
   */
  virtual void RegisterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_pc,
                                  const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity());

  /**
   * @brief Get last computed relative transform
   *
   * @return Eigen::Matrix4f
   */
  Eigen::Matrix4f GetRelativeTransform();

  /**
   * @brief Get abslolute transform
   *
   * @return Eigen::Matrix4f
   */
  Eigen::Matrix4f GetAbsoluteTransform();

  /**
   * @brief Set a particular settings
   *
   * @param setting_name
   * @param value
   */
  void SetSetting(const std::string& setting_name, double value);

  virtual void Close(){};

  void PrintSettings();

 protected:
  Eigen::Matrix4f absolute_transform_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f relative_transform_ = Eigen::Matrix4f::Identity();
  std::unordered_map<std::string, double> settings_ = {};
};
}  // namespace registration

}  // namespace sl_sensor