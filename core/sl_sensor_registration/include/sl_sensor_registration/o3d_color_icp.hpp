#pragma once
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <boost/make_shared.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include "sl_sensor_registration/point_cloud_registration_algorithm.hpp"

namespace sl_sensor
{
namespace registration
{
class O3dColorIcp : public PointCloudRegistrationAlgorithm
{
public:
  O3dColorIcp();

  virtual void LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) override;

  virtual void RegisterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_pc_ptr,
                                  const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity()) override;

  std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>>
  GetDownsampledPcs(const open3d::geometry::PointCloud& input_cloud, const std::vector<double>& voxel_radius,
                    const std::vector<double>& normal_radius);

  void VisualiseRegistration(const open3d::geometry::PointCloud& source, const open3d::geometry::PointCloud& target,
                             const Eigen::Matrix4d& Transformation);

private:
  bool received_first_point_cloud_ = false;
  std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>> prev_downsampled_pcs_ptr_ = {};
  std::vector<int> max_iter_ = {};
  std::vector<double> voxel_radius_ = {};
  std::vector<double> normal_radius_ = {};
  open3d::pipelines::registration::TransformationEstimationForColoredICP color_icp_;
  const std::unordered_map<std::string, double> default_settings_ = {
    { "max_distance", 0.01f }, { "max_nearest_neighbours", 30 }, { "relative_fitness", 1e-8 },
    { "relative_rsme", 1e-8 }, { "lambda_geometric", 0.968f },   { "visualise_point_cloud", 1 }
  };
};
}  // namespace registration
}  // namespace sl_sensor