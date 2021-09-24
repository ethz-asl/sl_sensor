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

#ifndef SL_SENSOR_REGISTRATION_POINT_CLOUD_REGISTRATION_ALGORITHM_HPP_
#define SL_SENSOR_REGISTRATION_POINT_CLOUD_REGISTRATION_ALGORITHM_HPP_

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
  virtual void RegisterPointCloud(
      [[maybe_unused]] const pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pc,
      [[maybe_unused]] const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity()){};

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

#endif  // SL_SENSOR_REGISTRATION_POINT_CLOUD_REGISTRATION_ALGORITHM_HPP_