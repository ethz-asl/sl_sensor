#include <sl_sensor_registration/o3d_color_icp.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <utility>

#include <open3d_conversions/open3d_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cassert>

#include <boost/algorithm/string.hpp>

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

namespace sl_sensor {
namespace registration {
O3dColorIcp::O3dColorIcp() { settings_ = default_settings_; }

void O3dColorIcp::LoadSettingsFromROSNodeHandle(ros::NodeHandle nh) {
  std::string max_iter_string = "";
  std::vector<std::string> max_iter_string_segments = {};
  nh.param<std::string>("max_iter", max_iter_string, "");
  boost::split(max_iter_string_segments, max_iter_string, boost::is_any_of(","));

  if (max_iter_string_segments.size() > 0) {
    max_iter_.clear();

    for (const auto& max_iter_string_segment : max_iter_string_segments) {
      max_iter_.push_back(std::stoi(max_iter_string_segment));
    }
  }

  std::string voxel_radius_string = "";
  std::vector<std::string> voxel_radius_string_segments = {};
  nh.param<std::string>("voxel_radius", voxel_radius_string, "");
  boost::split(voxel_radius_string_segments, voxel_radius_string, boost::is_any_of(","));

  if (voxel_radius_string_segments.size() > 0) {
    voxel_radius_.clear();

    for (const auto& voxel_radius_string_segment : voxel_radius_string_segments) {
      voxel_radius_.push_back(std::stod(voxel_radius_string_segment));
    }
  }

  std::string normal_radius_string = "";
  std::vector<std::string> normal_radius_string_segments = {};
  nh.param<std::string>("normal_radius", normal_radius_string, "");
  boost::split(normal_radius_string_segments, normal_radius_string, boost::is_any_of(","));

  if (normal_radius_string_segments.size() > 0) {
    normal_radius_.clear();

    for (const auto& normal_radius_string_segment : normal_radius_string_segments) {
      normal_radius_.push_back(std::stod(normal_radius_string_segment));
    }
  }

  assertm(max_iter_.size() == (voxel_radius_.size() == normal_radius_.size()),
          "Number of entries provided for maxt "
          "iter and voxel voxel_radius "
          "must "
          "be the same");

  PointCloudRegistrationAlgorithm::LoadSettingsFromROSNodeHandle(nh);
};

void O3dColorIcp::RegisterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_pc_ptr,
                                     const Eigen::Matrix4f& guess) {
  sensor_msgs::PointCloud2Ptr ros_pc_ptr = boost::make_shared<sensor_msgs::PointCloud2>();

  pcl::toROSMsg(*reference_pc_ptr, *ros_pc_ptr);

  open3d::geometry::PointCloud curr_pc{};

  open3d_conversions::rosToOpen3d(ros_pc_ptr, curr_pc);

  auto curr_downsampled_pcs_ptr = GetDownsampledPcs(curr_pc, voxel_radius_, normal_radius_);

  if (received_first_point_cloud_) {
    Eigen::Matrix4d current_transformation = guess.cast<double>();

    for (int i = 0; i < (int)voxel_radius_.size(); i++) {
      int max_iter = max_iter_[i];
      color_icp_.lambda_geometric_ = settings_["lambda_geometric"];

      auto result_icp = RegistrationColoredICP(
          *((*curr_downsampled_pcs_ptr)[i]), *((*prev_downsampled_pcs_ptr_)[i]),
          settings_["max_distance"], current_transformation, color_icp_,
          open3d::pipelines::registration::ICPConvergenceCriteria(settings_["rela"
                                                                            "tive"
                                                                            "_fit"
                                                                            "nes"
                                                                            "s"],
                                                                  settings_["rela"
                                                                            "tive"
                                                                            "_"
                                                                            "rsm"
                                                                            "e"],
                                                                  max_iter));

      current_transformation = result_icp.transformation_;

      std::cout << "[Colored ICP Iteration " << i + 1
                << " Results] Inlier RSME: " << result_icp.inlier_rmse_
                << " Fitness: " << result_icp.fitness_ << std::endl;
    }

    // Output Transformation is T[prev->curr]
    relative_transform_ = current_transformation.cast<float>();
    // T[0->curr] = T[0->prev] * T[prev->curr]
    absolute_transform_ = absolute_transform_ * relative_transform_;
  } else {
    received_first_point_cloud_ = true;
  }

  prev_downsampled_pcs_ptr_ = curr_downsampled_pcs_ptr;
}

std::shared_ptr<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>>
O3dColorIcp::GetDownsampledPcs(const open3d::geometry::PointCloud& input_cloud,
                               const std::vector<double>& voxel_radius,
                               const std::vector<double>& normal_radius) {
  auto output_ptr = std::make_shared<std::vector<std::shared_ptr<open3d::geometry::PointCloud>>>();

  assertm(voxel_radius.size() == normal_radius.size(),
          "Number of entries provided for maxt "
          "iter and voxel voxel_radius "
          "must "
          "be the same");

  for (int i = 0; i < voxel_radius.size(); i++) {
    double v_rad = voxel_radius[i];
    double n_rad = normal_radius[i];

    std::shared_ptr<open3d::geometry::PointCloud> current_pc_ptr =
        std::make_shared<open3d::geometry::PointCloud>();

    if (v_rad > 0.0) {
      current_pc_ptr = input_cloud.VoxelDownSample(v_rad);
    } else {
      *current_pc_ptr = input_cloud;
    }

    current_pc_ptr->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(n_rad * 2, (int)settings_["max_"
                                                                            "neare"
                                                                            "s"
                                                                            "t_"
                                                                            "neigh"
                                                                            "b"
                                                                            "our"
                                                                            "s"]));
    current_pc_ptr->OrientNormalsTowardsCameraLocation();

    (*output_ptr).push_back(current_pc_ptr);
  }

  return output_ptr;
}

}  // namespace registration
}  // namespace sl_sensor