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

#ifndef SL_SENSOR_REGISTRATION_O3D_COLOR_ICP_HPP_
#define SL_SENSOR_REGISTRATION_O3D_COLOR_ICP_HPP_

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <boost/make_shared.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"

namespace sl_sensor {
namespace registration {

/**
 * @brief PointCloudRegistrationAlgorithm that runs Open3D Color ICP
 * Note: We follow the implementation documented in
 * http://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html but in
 * C++ instead of Python
 */
class O3dColorIcp : public PointCloudRegistrationAlgorithm {
 public:
  /**
   * @brief Construct a new O3dColorIcp object
   *
   */
  O3dColorIcp();

  /**
   * @brief Load ICP settings from ROS node handle
   *
   * @param nh
   */
  virtual void LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) override;

  /**
   * @brief Register point cloud (XYZ Points)
   *
   * @param reference_pc - Point cloud to be registered
   * @param guess - initial guess transform
   */
  virtual void RegisterPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_pc_ptr,
      const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity()) override;

  /**
   * @brief Given a point cloud, return multiple point cloud that are subsample to various degrees,
   * and also their normals computed so they are ready to be used for colour ICP
   *
   * @param input_cloud
   * @param voxel_radius - Vector of radii used for voxel subsampling
   * @param normal_radius - Vector of radii used for normal computation, should be of same length as
   * voxel_radius
   * @return std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>> - Pointer
   * to vector of output point clouds. Size of vector will be the same as voxel_radius and
   * normal_radius
   */
  std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>> GetDownsampledPcs(
      const open3d::geometry::PointCloud& input_cloud, const std::vector<double>& voxel_radius,
      const std::vector<double>& normal_radius);

 private:
  bool received_first_point_cloud_ = false;
  std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>>
      prev_downsampled_pcs_ptr_ = {};
  std::vector<int> max_iter_ = {};
  std::vector<double> voxel_radius_ = {};
  std::vector<double> normal_radius_ = {};
  open3d::pipelines::registration::TransformationEstimationForColoredICP color_icp_;
  const std::unordered_map<std::string, double> default_settings_ = {
      {"max_distance", 0.01f}, {"max_nearest_neighbours", 30}, {"relative_fitness", 1e-8},
      {"relative_rsme", 1e-8}, {"lambda_geometric", 0.968f},   {"visualise_point_cloud", 1}};
};
}  // namespace registration
}  // namespace sl_sensor

#endif  // SL_SENSOR_REGISTRATION_O3D_COLOR_ICP_HPP_